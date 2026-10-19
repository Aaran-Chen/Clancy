"""Session video: the depth camera's color stream and the demo-area webcam, recorded while an AI task runs.

It runs as its own process beside the bridge, so a recording problem can never
touch arm control. Depth frames come from the bridge (GET /depthcam/frame.jpg,
the same stream the web app shows), so the recorder survives bridge restarts: it
waits and resumes. Prompts and answers are logged by the web app's dev server
(scripts/SessionLog.ts) and the bridge (object_locator), next to this video.

When (Aaran, 2026-10-08): only while an AI task runs. The page says when a task
starts and ends, and every 15 s that it still runs, through the dev server
(POST /session-log/recording, scripts/SessionVideo.ts), which passes it on to
POST /task here. Between tasks both cameras are still read, but only their last
PreRollS seconds are kept, in memory and still compressed, so a recording begins
PreRollS before its task did; it ends PostRollS after the task. A task its page
stops reporting (tab closed or crashed, dev server stopped) ends LeaseS after the
last word; one that never ends is cut at MaxTaskS. Until 2026-10-08 it recorded
around the clock: about 56 GB on a day with no task (audit 2 M4).

Kept 7 days: VideoRetention.py deletes older video (these segments, their frame
indexes and the page's per-task clips), and nothing else, when the recorder starts
and every hour, and logs each file in captures/video_retention.log.

Output, per day, in back end/captures/YYYY-MM-DD/recording/:
  video/HHMMSS_mmm_depthcam.mp4       H.264, fragmented (playable even after a
                                      crash), variable frame rate, at most 10-minute
                                      segments, named by the first frame's time
  video/HHMMSS_mmm_depthcam.frames.jsonl
                                      one line per frame: {"i", "seq", "t", "pts_ms"}
                                      (t = unix seconds; pts = t - segment start)
  video/HHMMSS_mmm_webcam.mp4 (+ .frames.jsonl)
                                      the same from the webcam that looks at the
                                      demo area (2026-10-04: a Logitech C920, read
                                      by this process over DirectShow, --webcam)
  recorder_events.jsonl               each task the page reported, and when and why
                                      each camera's recording started and stopped
A name is never reused: a segment started in the same millisecond as another gets
"-1", "-2" (HHMMSS_mmm-1_depthcam; audit 2 M5). Names before 2026-10-08 are
HHMMSS_<camera>. ReviewRecording.py maps any wall-clock time to a segment and frame.

A second copy exits at once: the recorder binds 127.0.0.1:8002 and serves
GET /health there, and POST /task (from programs on this PC only). With
--replace-older (the dev server's launcher) a recorder of an older version on
that port is stopped first, so an updated recorder takes over by itself.

Run:  python SessionRecorder.py              (the web app's dev server starts it)
      python SessionRecorder.py --webcam ""  (the depth camera only)
      python SessionRecorder.py --always     (every frame, task or not: for checks)
"""

from __future__ import annotations

import argparse
import collections
import datetime
import json
import os
import re
import shutil
import signal
import socket
import threading
import time
import urllib.error
import urllib.request
from fractions import Fraction
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Callable

import av
import cv2
import numpy as np

import KeepVideo
import VideoRetention

BackendRoot = Path(__file__).resolve().parent
DefaultRoot = Path(os.environ.get("SO101_CAPTURES_ROOT") or BackendRoot / "captures")
# What the dev server's launcher requires the running recorder to report (scripts/SessionVideo.ts reads the same file).
RecorderVersion = json.loads((BackendRoot / "RecorderVersion.json").read_text(encoding="utf-8"))["Version"]
StatusPort = 8002
SegmentSeconds = 600
TimeBase = Fraction(1, 1000)  # millisecond presentation times: exact, variable rate
PollTimeoutS = 5.0
RetryDelayS = 1.0
# While the bridge's camera is not streaming, its frame poll answers at once: then ask again after IdlePollS.
IdlePollS = 1.0
# While a task runs, both cameras together are about 2 to 5 GB an hour of task (the depth camera 1.0-3.8, median
# 1.9 GB/h; the webcam up to 1.7 GB/h, measured 2026-10-04..06). Recording pauses, and nothing is deleted for it,
# while the drive has less than this free.
MinimumFreeBytes = 20e9
DiskCheckS = 30.0
# The demo-area webcam (DirectShow name; SO101_WEBCAM overrides, "" turns it off).
# Read as MJPEG at its full rate and kept at WebcamFps, the depth camera's rate,
# so a recording costs about one core while Aaran games on the same PC.
DefaultWebcam = os.environ.get("SO101_WEBCAM", "HD Pro Webcam C920")
WebcamOptions = {"vcodec": "mjpeg", "video_size": "1280x720", "framerate": "30", "rtbufsize": "64M"}
WebcamFps = 15.0
# A webcam that is unplugged or held by another program is tried again this often.
WebcamRetryS = 5.0
# When to record (TaskGate). The page reports a running task every 15 s (src/lib/SessionVideo.ts); a background tab
# may be woken only once a minute, so a task is lost only after LeaseS without a word.
PreRollS = 5.0
PostRollS = 10.0
LeaseS = 180.0
MaxTaskS = 3600.0
MaxPreRollFrames = 400        # a bound on memory whatever the frame times say (5 s at 15 fps is 75)
StaleFrameS = 5.0             # a recording whose camera sends nothing for this long is closed
TickS = 1.0
RetentionEveryS = 3600.0
EventsName = "recorder_events.jsonl"
TaskKey = re.compile(r"^[A-Za-z0-9_.:-]{1,80}$")
MaxBodyBytes = 16384


def RecordingDirectory(Root: Path, When: float) -> Path:
    return Root / time.strftime("%Y-%m-%d", time.localtime(When)) / "recording"


def IsoTime(When: float) -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(When)) + f".{int(When % 1 * 1000):03d}Z"


def ReserveSegment(Directory: Path, Start: float, Camera: str) -> tuple[str, Any]:
    """A name no other segment has, and its frame index opened for writing: HHMMSS_mmm_<camera>, then
    HHMMSS_mmm-1_<camera>, ... Both files are created here, empty and exclusively, so neither a segment started in
    the same millisecond nor a second recorder can take them over. (Named by the second alone, two segments started
    in one second shared a name: the second emptied the first's video and appended to its index, audit 2 M5.)"""
    Base = time.strftime("%H%M%S", time.localtime(Start)) + f"_{int(Start * 1000) % 1000:03d}"
    for Count in range(1000):
        Stem = f"{Base}{f'-{Count}' if Count else ''}_{Camera}"
        try:
            Index = open(Directory / f"{Stem}.frames.jsonl", "x", encoding="utf-8")
        except FileExistsError:
            continue
        try:
            os.close(os.open(Directory / f"{Stem}.mp4", os.O_CREAT | os.O_EXCL | os.O_WRONLY))
        except FileExistsError:
            Index.close()
            (Directory / f"{Stem}.frames.jsonl").unlink(missing_ok=True)  # ours: made just above
            continue
        return Stem, Index
    raise RuntimeError(f"no free segment name for {Base} in {Directory}")


class Segment:
    """One H.264 MP4 file and its frame-time index."""

    def __init__(Self, Directory: Path, Start: float, Width: int, Height: int, Crf: int, Camera: str = "depthcam"):
        Directory.mkdir(parents=True, exist_ok=True)
        Stem, Self.Index = ReserveSegment(Directory, Start, Camera)
        Self.Path = Directory / f"{Stem}.mp4"
        Self.IndexPath = Directory / f"{Stem}.frames.jsonl"
        Self.Start, Self.Width, Self.Height = Start, Width, Height
        try:
            # Fragmented MP4: every fragment is complete, so a killed process loses
            # at most the last keyframe interval rather than the whole file.
            Self.Container = av.open(str(Self.Path), mode="w", format="mp4",
                                     options={"movflags": "frag_keyframe+empty_moov+default_base_moof"})
            Self.Stream = Self.Container.add_stream("libx264", rate=15)
            Self.Stream.width, Self.Stream.height = Width, Height
            Self.Stream.pix_fmt = "yuv420p"
            Self.Stream.time_base = TimeBase
            Self.Stream.codec_context.time_base = TimeBase
            Self.Stream.options = {"crf": str(Crf), "preset": "veryfast", "g": "30"}
        except Exception:
            Self.Index.close()
            raise
        Self.Frames = 0
        Self.LastPts = -1

    def Write(Self, Bgr: np.ndarray, Seq: int, Stamp: float) -> None:
        Pts = max(int(round((Stamp - Self.Start) * 1000)), Self.LastPts + 1)
        Frame = av.VideoFrame.from_ndarray(Bgr, format="bgr24")
        Frame.pts, Frame.time_base = Pts, TimeBase
        for Packet in Self.Stream.encode(Frame):
            Self.Container.mux(Packet)
        Self.LastPts = Pts
        Self.Index.write(json.dumps({"i": Self.Frames, "seq": Seq, "t": round(Stamp, 4), "pts_ms": Pts}) + "\n")
        Self.Index.flush()
        Self.Frames += 1

    def Close(Self) -> None:
        try:
            for Packet in Self.Stream.encode(None):
                Self.Container.mux(Packet)
        finally:
            Self.Container.close()
            Self.Index.close()


class TaskGate:
    """Whether to record now: while an AI task runs, and PostRoll after the last one ended.

    The page names each task by a key (src/lib/SessionVideo.ts) and reports it "start", then "alive" every 15 s,
    then "end". An "alive" for a key this process never saw starts it too (the recorder restarted mid-task). A task
    with no word for Lease ends there, with no post-roll (its page closed or crashed, or the dev server stopped:
    Lease already recorded past its last word); one that runs past MaxTask is cut and its key ignored until its
    "end", so a page stuck on a task cannot keep the cameras recording. Every change goes to OnEvent.
    """

    def __init__(Self, PostRoll: float = PostRollS, Lease: float = LeaseS, MaxTask: float = MaxTaskS,
                 OnEvent: Callable[[dict[str, Any]], None] | None = None):
        Self.PostRoll, Self.Lease, Self.MaxTask = PostRoll, Lease, MaxTask
        Self.OnEvent = OnEvent
        Self.Tasks: dict[str, dict[str, Any]] = {}
        Self.Cut: dict[str, float] = {}
        Self.Until = 0.0
        Self.Lock = threading.Lock()

    def Signal(Self, State: str, Key: str, Task: str = "", Reason: str = "", Now: float | None = None) -> dict[str, Any]:
        """One word from the page: "start", "alive" or "end". Returns {"accepted", "recording", ...}."""
        Now = time.time() if Now is None else Now
        with Self.Lock:
            Events = Self._Expire(Now)
            Accepted, Why = True, ""
            if State == "end":
                Self.Cut.pop(Key, None)
                Ended = Self.Tasks.pop(Key, None)
                Accepted = Ended is not None
                if Ended is not None:
                    Self.Until = max(Self.Until, Now + Self.PostRoll)
                    Events.append(Self._Record(Now, "task_end", Key, Ended["task"], Reason or "the task ended"))
            elif Key in Self.Cut:
                Accepted, Why = False, f"this task ran past {Self.MaxTask / 60:.0f} min, so its recording was stopped"
            elif Key not in Self.Tasks:
                Self.Tasks[Key] = {"task": Task[:200], "started": Now, "alive": Now}
                Events.append(Self._Record(Now, "task_start" if State == "start" else "task_resumed", Key, Task[:200],
                                           "" if State == "start" else "the recorder had not heard of this running task"))
            else:
                Self.Tasks[Key]["alive"] = Now
                Self.Tasks[Key]["task"] = Self.Tasks[Key]["task"] or Task[:200]
            Snapshot = Self._Snapshot(Now)
        Self._Emit(Events)
        return {"accepted": Accepted, **({"why": Why} if Why else {}), **Snapshot}

    def Recording(Self, Now: float | None = None) -> bool:
        Now = time.time() if Now is None else Now
        with Self.Lock:
            Events = Self._Expire(Now)
            Open = bool(Self.Tasks) or Now < Self.Until
        Self._Emit(Events)
        return Open

    def Snapshot(Self, Now: float | None = None) -> dict[str, Any]:
        Now = time.time() if Now is None else Now
        with Self.Lock:
            Events = Self._Expire(Now)
            Snapshot = Self._Snapshot(Now)
        Self._Emit(Events)
        return Snapshot

    def _Expire(Self, Now: float) -> list[dict[str, Any]]:
        Events = []
        for Key, Item in list(Self.Tasks.items()):
            if Now - Item["alive"] > Self.Lease:
                del Self.Tasks[Key]
                Events.append(Self._Record(Now, "task_lost", Key, Item["task"],
                                           f"no word from its page for {Self.Lease:.0f} s (closed, crashed or cut off)"))
            elif Now - Item["started"] > Self.MaxTask:
                del Self.Tasks[Key]
                Self.Cut[Key] = Now
                Events.append(Self._Record(Now, "task_cut", Key, Item["task"],
                                           f"still running after {Self.MaxTask / 60:.0f} min: its recording stops here"))
        for Key, At in list(Self.Cut.items()):
            if Now - At > 86400:
                del Self.Cut[Key]
        return Events

    def _Snapshot(Self, Now: float) -> dict[str, Any]:
        return {"recording": bool(Self.Tasks) or Now < Self.Until,
                "tasks": [{"task_id": Key, "task": Item["task"], "running_s": round(Now - Item["started"], 1),
                           "quiet_s": round(Now - Item["alive"], 1)} for Key, Item in Self.Tasks.items()],
                "post_roll_left_s": round(Self.Until - Now, 1) if Self.Until > Now else 0.0,
                "post_roll_s": Self.PostRoll, "lease_s": Self.Lease, "max_task_s": Self.MaxTask}

    @staticmethod
    def _Record(Now: float, Kind: str, Key: str, Task: str, Reason: str) -> dict[str, Any]:
        return {"t": round(Now, 3), "kind": Kind, "task_id": Key, "task": Task, "reason": Reason}

    def _Emit(Self, Events: list[dict[str, Any]]) -> None:
        for Event in Events:
            if Self.OnEvent:
                try:
                    Self.OnEvent(Event)
                except Exception:  # an event that cannot be written must not stop the recording
                    pass


def EventLog(Root: Path) -> Callable[[dict[str, Any]], None]:
    """recorder_events.jsonl beside the day's video (ReviewRecording's timeline shows it), and a line in this
    process's output log: each task the page reported, and when and why a camera's recording started or stopped."""
    Lock = threading.Lock()

    def Write(Record: dict[str, Any]) -> None:
        Record = {**Record, "time": IsoTime(Record["t"])}
        try:
            Directory = RecordingDirectory(Root, Record["t"])
            Directory.mkdir(parents=True, exist_ok=True)
            with Lock, open(Directory / EventsName, "a", encoding="utf-8") as File:
                File.write(json.dumps(Record) + "\n")
        except OSError as Exc:
            print(f"[recorder] could not record {Record['kind']}: {Exc}", flush=True)
        Parts = [Record.get(Name) for Name in ("camera", "task", "reason", "segment") if Record.get(Name)]
        print(f"{datetime.datetime.fromtimestamp(Record['t']):%Y-%m-%d %H:%M:%S} {Record['kind']}: "
              + " | ".join(str(Part) for Part in Parts), flush=True)

    return Write


class Recorder:
    """The depth camera's color frames, from the bridge.

    With a Gate, a frame is written only while the gate records; otherwise it joins the pre-roll (the last PreRoll
    seconds, still compressed), which leads the next recording. Without one, every frame is written (--always)."""
    Camera = "depthcam"

    def __init__(Self, Bridge: str, Root: Path, SegmentLength: float = SegmentSeconds, Crf: int = 23,
                 Gate: TaskGate | None = None, PreRoll: float = PreRollS):
        Self.Bridge, Self.Root = Bridge.rstrip("/"), Root
        Self.SegmentLength, Self.Crf = SegmentLength, Crf
        Self.Gate, Self.PreRollSeconds = Gate, PreRoll
        Self.PreRoll: collections.deque[tuple[int, float, Any]] = collections.deque()
        Self.OnEvent: Callable[[dict[str, Any]], None] | None = None
        Self.Segment: Segment | None = None
        Self.Running = False
        Self.Seq = 0
        Self.LastFrameAt: float | None = None   # the last frame written
        Self.LastSeenAt: float | None = None    # the last frame from the camera, written or kept for the pre-roll
        Self.WasRecording = Gate is None
        Self.Frames = 0
        Self.Undecodable = 0
        Self.Error: str | None = None
        Self.Lock = threading.Lock()
        # One frame's gate check and write, and the supervisor's Tick, never interleave: a recording closed by Tick
        # between a frame's check and its write would leave that frame in a segment nothing closes.
        Self.WriteLock = threading.Lock()
        Self.DiskCheckedAt = -DiskCheckS
        Self.DiskFull = False
        Self.DiskMessage = ""
        # The pre-roll written by a thread of its own when a recording starts (Main turns it on): decoding and encoding
        # its 75 frames held the camera thread 0.42-0.56 s, and the frames of that moment were never fetched (the
        # bridge serves only its latest) or were dropped by the webcam's rate limit: every recording had a 0.6 s gap
        # just after its task started (2026-10-08 review). Frames that arrive meanwhile wait in Pending, in order.
        Self.FlushInBackground = False
        Self.Flusher: threading.Thread | None = None
        Self.Pending: list[tuple[int, float, Any]] = []

    def DiskHasRoom(Self, Now: float) -> bool:
        if Now - Self.DiskCheckedAt >= DiskCheckS:
            Self.DiskCheckedAt = Now
            Self.Root.mkdir(parents=True, exist_ok=True)
            Free = shutil.disk_usage(Self.Root).free
            Self.DiskFull = Free < MinimumFreeBytes
            Self.DiskMessage = f"paused: only {Free / 1e9:.1f} GB free (needs {MinimumFreeBytes / 1e9:.0f} GB)"
        if Self.DiskFull:
            Self.Error = Self.DiskMessage
        return not Self.DiskFull

    def Fetch(Self) -> tuple[int, bytes] | None:
        """Next frame after Self.Seq; None while nothing newer has arrived. Raises for an answer that is no frame."""
        Url = f"{Self.Bridge}/depthcam/frame.jpg?after={Self.Seq}"
        with urllib.request.urlopen(Url, timeout=PollTimeoutS) as Response:
            if Response.status == 204:
                return None
            Seq = str(Response.headers.get("X-Frame-Seq") or "").strip()
            if not Seq.isdigit():
                raise ValueError(f"a frame answer without its frame number (X-Frame-Seq {Seq or 'missing'})")
            return int(Seq), Response.read()

    def BridgeStatus(Self) -> dict[str, Any] | None:
        try:
            with urllib.request.urlopen(f"{Self.Bridge}/depthcam/status", timeout=PollTimeoutS) as Response:
                Status = json.loads(Response.read())
            return Status if isinstance(Status, dict) else None
        except Exception:  # unreachable, cut short (http.client.IncompleteRead) or not JSON: no status
            return None

    def Handle(Self, Seq: int, Jpeg: bytes, Stamp: float) -> None:
        Self.Offer(Jpeg, Seq, Stamp)

    def Note(Self, Kind: str, Stamp: float, **Fields: Any) -> None:
        if Self.OnEvent:
            try:
                Self.OnEvent({"t": round(Stamp, 3), "kind": Kind, "camera": Self.Camera, **Fields})
            except Exception:
                pass

    def Recording(Self, Stamp: float) -> bool:
        """The gate's answer; while it is closed no segment of this camera stays open (its recording ended)."""
        Open = Self.Gate is None or Self.Gate.Recording(Stamp)
        Self.WasRecording = Open
        if not Open:
            Self.CloseSegment("its task's recording ended", Stamp)  # nothing when none is open
        return Open

    def Offer(Self, Payload: Any, Seq: int, Stamp: float) -> None:
        """One frame as the camera gave it (the bridge's JPEG, the webcam's MJPEG frame, or decoded pixels): written
        while the gate records, else kept for the pre-roll, compressed as it came (no decoding between tasks)."""
        with Self.WriteLock:
            Self.OfferLocked(Payload, Seq, Stamp)

    def OfferLocked(Self, Payload: Any, Seq: int, Stamp: float) -> None:
        Self.LastSeenAt = Stamp
        if not Self.Recording(Stamp):
            with Self.Lock:
                Self.PreRoll.append((Seq, Stamp, Payload))
                while Self.PreRoll and (Stamp - Self.PreRoll[0][1] > Self.PreRollSeconds or len(Self.PreRoll) > MaxPreRollFrames):
                    Self.PreRoll.popleft()
            return
        if not Self.DiskHasRoom(Stamp):
            Self.CloseSegment()
            return
        if Self.Flusher is not None:
            if Self.Flusher.is_alive():
                Self.Pending.append((Seq, Stamp, Payload))  # behind the pre-roll still being written
                return
            Self.FinishFlush()
        with Self.Lock:
            Starting = Self.Segment is None
            Lead = [Item for Item in Self.PreRoll if 0 <= Stamp - Item[1] <= Self.PreRollSeconds] if Starting else []
            Self.PreRoll.clear()
        if Starting and Self.Gate is not None:
            Self.Note("recording_start", Stamp, pre_roll_s=round(Stamp - Lead[0][1], 2) if Lead else 0.0,
                      pre_roll_frames=len(Lead))
        if Lead and Self.FlushInBackground:
            Self.Pending = [(Seq, Stamp, Payload)]
            Self.Flusher = threading.Thread(target=Self.WriteItems, args=(Lead,), name=f"{Self.Camera}-pre-roll", daemon=True)
            Self.Flusher.start()
            return
        Self.WriteItems(Lead + [(Seq, Stamp, Payload)])

    def WriteItems(Self, Items: list[tuple[int, float, Any]]) -> None:
        """Decode and write frames in order (the pre-roll's flusher thread, or the frame's own)."""
        for ItemSeq, ItemStamp, Item in Items:
            Bgr = Self.Decode(Item)
            if Bgr is None:
                Self.Undecodable += 1  # in /health: a camera whose frames all fail here would otherwise record nothing, unseen
                continue
            Self.Store(Bgr, ItemSeq, ItemStamp)

    def FinishFlush(Self) -> None:
        """Wait for the pre-roll's flusher, then write the frames that waited behind it. Called with WriteLock."""
        Flusher, Self.Flusher = Self.Flusher, None
        if Flusher is not None:
            Flusher.join()
        Pending, Self.Pending = Self.Pending, []
        Self.WriteItems(Pending)

    @staticmethod
    def Decode(Payload: Any) -> np.ndarray | None:
        """BGR pixels of a frame; compressed ones (JPEG, the webcam's MJPEG, Huffman tables or not) are decoded only
        when written: OpenCV's decoder, then FFmpeg's MJPEG decoder (what read the webcam before) for a frame
        OpenCV refuses."""
        if isinstance(Payload, np.ndarray):
            return Payload
        Bgr = cv2.imdecode(np.frombuffer(Payload, np.uint8), cv2.IMREAD_COLOR)
        if Bgr is not None:
            return Bgr
        try:
            Frames = av.CodecContext.create("mjpeg", "r").decode(av.Packet(bytes(Payload)))
            return Frames[0].to_ndarray(format="bgr24") if Frames else None
        except (av.error.FFmpegError, ValueError):
            return None

    def Store(Self, Bgr: np.ndarray, Seq: int, Stamp: float) -> None:
        """Write one frame, starting a new segment every SegmentLength, on a new size or a new day."""
        Height, Width = Bgr.shape[:2]
        Directory = RecordingDirectory(Self.Root, Stamp) / "video"
        with Self.Lock:
            Current = Self.Segment
            if Current is not None and (Stamp - Current.Start >= Self.SegmentLength
                                        or (Current.Width, Current.Height) != (Width, Height)
                                        or Current.Path.parent != Directory):
                Current.Close()
                Current = Self.Segment = None
            if Current is None:
                Current = Self.Segment = Segment(Directory, Stamp, Width, Height, Self.Crf, Self.Camera)
            Current.Write(Bgr, Seq, Stamp)
            Self.Frames += 1
            Self.LastFrameAt = Stamp

    def CloseSegment(Self, Why: str = "", Stamp: float | None = None) -> None:
        if Self.Flusher is not None:
            Self.FinishFlush()  # the recording's frames so far, the pre-roll first, then it closes
        with Self.Lock:
            Current, Self.Segment = Self.Segment, None
        if Current is None:
            return
        try:
            Current.Close()
        except Exception:
            pass
        if Why and Self.Gate is not None:
            Self.Note("recording_stop", time.time() if Stamp is None else Stamp, reason=Why, segment=Current.Path.name)

    def Tick(Self, Now: float) -> None:
        """Between frames (the supervisor, every second): a recording ends when its task's does even if no frame
        comes, and a recording whose camera went quiet is closed (2026-10-06: a segment stayed open on the last
        frame for 6 hours after the depth camera was unplugged)."""
        with Self.WriteLock:
            if not Self.Recording(Now):
                return
            Quiet = Self.LastSeenAt is None or Now - Self.LastSeenAt > StaleFrameS
            if Self.Segment is not None and Quiet:
                Self.Error = Self.Error or f"no frame from the {Self.Camera} for {StaleFrameS:.0f} s"
                Self.CloseSegment(f"no frame for {StaleFrameS:.0f} s", Now)

    def Run(Self) -> None:
        Self.Running = True
        Empty = 0
        while Self.Running:
            try:
                Got = Self.Fetch()
            except Exception as Exc:
                # The bridge restarting, an answer cut short (http.client.IncompleteRead: a bridge stopped mid-answer) or
                # not a frame: ask again. Only URLError, OSError and ValueError were caught, so one short answer ended
                # the recorder, and with it the running task's recording and its pre-roll (2026-10-08 regression review).
                Self.Error = f"bridge frames unavailable: {type(Exc).__name__}: {Exc}"
                time.sleep(RetryDelayS)
                continue
            if Got is None:
                Empty += 1
                if Empty >= 2:
                    Empty = 0
                    Status = Self.BridgeStatus()
                    if Status is not None and Status.get("available"):
                        # A restarted bridge counts frames from 1 again; never wait for
                        # a number it will not reach for hours.
                        if int(Status.get("seq") or 0) < Self.Seq:
                            Self.Seq = 0
                    else:
                        # Its camera is not streaming (unplugged), and the bridge then answers at once: ask again in
                        # a second, not some twenty times a second as before (most of a 337 MB access log, audit 2 M4).
                        Self.Error = "the bridge's depth camera is not streaming" if Status else "the bridge does not answer"
                        time.sleep(IdlePollS)
                continue
            Empty = 0
            Self.Error = None
            Self.Seq = Got[0]
            try:
                Self.Handle(Got[0], Got[1], time.time())
            except Exception as Exc:  # a bad frame or full disk must not end the recording
                Self.Error = f"{type(Exc).__name__}: {Exc}"
                Self.CloseSegment()
                time.sleep(RetryDelayS)

    def Stop(Self) -> None:
        Self.Running = False
        Self.CloseSegment("the recorder stopped" if Self.WasRecording else "")

    def Status(Self) -> dict[str, Any]:
        with Self.Lock:
            Current = Self.Segment
            Now = time.time()
            Age = None if Self.LastFrameAt is None else round(Now - Self.LastFrameAt, 2)
            Seen = None if Self.LastSeenAt is None else round(Now - Self.LastSeenAt, 2)
            Lead = round(Self.PreRoll[-1][1] - Self.PreRoll[0][1], 2) if len(Self.PreRoll) > 1 else 0.0
            return {"service": "so101-recorder", "ok": True,
                    "recording": Current is not None and Age is not None and Age < 3.0,
                    "segment": str(Current.Path) if Current else None, "frames_total": Self.Frames,
                    "last_frame_age_s": Age, "last_seen_age_s": Seen, "pre_roll_frames": len(Self.PreRoll),
                    "pre_roll_buffered_s": Lead, "undecodable_frames": Self.Undecodable, "error": Self.Error,
                    "disk_full": Self.DiskFull, "pid": os.getpid()}


def OpenWebcam(Name: str, Source: str | None = None, Format: str | None = "dshow", Options: dict[str, str] | None = None):
    """The webcam's frames as it sends them. With MJPEG (asked for), each frame is one JPEG, kept compressed so a
    frame that is not written costs no decoding (between tasks only the pre-roll is kept); other formats are decoded
    here. Raises while the webcam is missing or busy. Source, Format and Options read a file instead (tests)."""
    Container = av.open(Source or f"video={Name}", format=Format, options=WebcamOptions if Options is None else Options)
    try:
        Stream = Container.streams.video[0]
        Compressed = Stream.codec_context.name == "mjpeg"
        for Packet in Container.demux(Stream):
            if Packet.size == 0:  # the demuxer's last, empty packet
                continue
            if Compressed:
                yield bytes(Packet)
            else:
                for Frame in Packet.decode():
                    yield Frame.to_ndarray(format="bgr24")
    finally:
        Container.close()


class WebcamRecorder(Recorder):
    """The demo-area webcam, read by this process, in its own segments beside the depth camera's.

    Nothing here talks to the bridge or the arm. A webcam that is unplugged, or
    held by another program, is tried again every WebcamRetryS; the depth
    camera's recording carries on either way.
    """
    Camera = "webcam"

    def __init__(Self, Name: str, Root: Path, SegmentLength: float = SegmentSeconds, Crf: int = 23,
                 Fps: float = WebcamFps, Open=OpenWebcam, Gate: TaskGate | None = None, PreRoll: float = PreRollS):
        super().__init__("", Root, SegmentLength, Crf, Gate, PreRoll)
        Self.Name, Self.Fps, Self.Open = Name, Fps, Open

    def Run(Self) -> None:
        Self.Running = True
        while Self.Running:
            Kept = None
            try:
                for Item in Self.Open(Self.Name):
                    if not Self.Running:
                        break
                    Stamp = time.time()
                    # At most Fps frames a second; a slightly early frame still counts (the camera's 30 fps jitters).
                    if Kept is not None and Stamp - Kept < 0.9 / Self.Fps:
                        continue
                    Kept = Stamp
                    Self.Seq += 1
                    Self.Error = None
                    Self.Offer(Item, Self.Seq, Stamp)
                if Self.Running:
                    Self.Error = Self.Error or "webcam stream ended"
            except Exception as Exc:  # missing, busy or unplugged: try again, never end the recorder
                Self.Error = f"webcam {Self.Name!r} unavailable: {type(Exc).__name__}: {Exc}"
            Self.CloseSegment("the webcam stopped sending" if Self.Running and Self.WasRecording else "")
            if Self.Running:
                time.sleep(WebcamRetryS)

    def Status(Self) -> dict[str, Any]:
        return {**super().Status(), "camera": Self.Name}


class Supervisor:
    """Once a second, each recorder's Tick (a recording ends with its task even when no frame comes); when the
    recorder starts and every RetentionEveryS, the 7-day video retention pass (VideoRetention.py)."""

    def __init__(Self, Recorders: list[Recorder | None], Root: Path, KeepDays: float = VideoRetention.KeepDays,
                 Every: float = RetentionEveryS):
        Self.Recorders = [Item for Item in Recorders if Item is not None]
        Self.Root, Self.KeepDays, Self.Every = Root, KeepDays, Every
        Self.Stopped = threading.Event()
        Self.Retention: dict[str, Any] = {"keep_days": KeepDays, "last_pass": None}
        Self.FaultsKept = False

    def Retain(Self, Now: float) -> None:
        try:
            if not Self.FaultsKept:
                # Before its first deletion on this captures folder: the video around every recorded fault is kept, once
                # (KeepVideo.KeepFaultsOnce), so a merge before anyone ran KeepVideo.py loses no fault video. When that
                # cannot finish, this pass deletes nothing (the except below) and the next one tries again.
                Added = KeepVideo.KeepFaultsOnce(Self.Root, Now)
                Self.FaultsKept = True
                if Added:
                    print(f"{datetime.datetime.fromtimestamp(Now):%Y-%m-%d %H:%M:%S} retention: kept the video around every recorded "
                          f"fault ({len(Added)} files, {VideoRetention.KeepName}) before the first pass", flush=True)
            Report = VideoRetention.RunRetention(Self.Root, Now, Self.KeepDays)
            Self.Retention = {"keep_days": Self.KeepDays, "last_pass": IsoTime(Now), "deleted_files": len(Report["deleted"]),
                              "freed_gb": round(Report["bytes"] / 1e9, 3), "errors": Report["errors"][:5],
                              "kept_by_list": len(Report["kept"]), "next_pass": IsoTime(Now + Self.Every)}
            if Report["deleted"] or Report["errors"]:
                print(f"{datetime.datetime.fromtimestamp(Now):%Y-%m-%d %H:%M:%S} retention: deleted {len(Report['deleted'])} "
                      f"files ({Report['bytes'] / 1e9:.2f} GB) older than {Self.KeepDays:g} days"
                      + (f", {len(Report['errors'])} could not be deleted" if Report["errors"] else ""), flush=True)
        except Exception as Exc:  # retention must never stop the recording
            Self.Retention = {**Self.Retention, "error": f"{type(Exc).__name__}: {Exc}"}

    def Run(Self) -> None:
        NextPass = time.time()
        while not Self.Stopped.is_set():
            Now = time.time()
            for Item in Self.Recorders:
                try:
                    Item.Tick(Now)
                except Exception as Exc:
                    Item.Error = f"{type(Exc).__name__}: {Exc}"
            if Now >= NextPass:
                NextPass = Now + Self.Every
                Self.Retain(Now)
            Self.Stopped.wait(TickS)

    def Status(Self) -> dict[str, Any]:
        return {"retention": dict(Self.Retention)}


class ExclusiveServer(ThreadingHTTPServer):
    """A status server that refuses a port another process holds.

    http.server turns on SO_REUSEADDR, which on Windows lets a second process
    bind the same port, so a second recorder would run beside the first.
    """
    allow_reuse_address = False
    daemon_threads = True

    def server_bind(Self):
        if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
            Self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        super().server_bind()


def LocalProgramRefusal(Headers: Any, Port: int) -> tuple[int, str] | None:
    """POST /task and /quit come only from programs on this PC (the dev server). Never from a web page: a page's
    request carries Origin or Sec-Fetch-*, and its JSON would need a preflight this server never answers. Never by
    another name for this server (DNS rebinding). The server listens on 127.0.0.1 only, so no other device reaches it."""
    if any(Headers.get(Name) is not None for Name in ("Origin", "Sec-Fetch-Site", "Sec-Fetch-Mode")):
        return 403, "web pages may not control the recorder"
    if (Headers.get("Host") or "").strip().lower() not in (f"127.0.0.1:{Port}", f"localhost:{Port}"):
        return 403, "only programs on this PC may control the recorder"
    if "application/json" not in (Headers.get("Content-Type") or "").lower():
        return 415, "send JSON (Content-Type: application/json)"
    return None


def ServeStatus(Instance: Recorder, Port: int, Webcam: Recorder | None = None, Gate: TaskGate | None = None,
                Extra: Callable[[], dict[str, Any]] | None = None,
                OnQuit: Callable[[], None] | None = None) -> ThreadingHTTPServer:
    class Handler(BaseHTTPRequestHandler):
        def Reply(Self, Code: int, Body: dict[str, Any]) -> None:
            Data = json.dumps(Body).encode()
            Self.send_response(Code)
            Self.send_header("Content-Type", "application/json")
            Self.send_header("Cache-Control", "no-store")
            Self.send_header("Content-Length", str(len(Data)))
            Self.end_headers()
            Self.wfile.write(Data)

        def do_GET(Self):  # noqa: N802 (http.server naming)
            if Self.path.split("?")[0] != "/health":
                Self.send_error(404)
                return
            Self.Reply(200, {**Instance.Status(), "version": RecorderVersion,
                             "webcam": Webcam.Status() if Webcam else None,
                             "task": Gate.Snapshot() if Gate else {"always": True},
                             "pre_roll_s": Instance.PreRollSeconds, **(Extra() if Extra else {})})

        def do_POST(Self):  # noqa: N802
            Route = Self.path.split("?")[0]
            try:
                Length = int(Self.headers.get("Content-Length") or 0)
            except ValueError:
                Length = 0
            # The body is read before any answer, refusals too: an answer sent over unread data can reach the client
            # as a reset instead (Windows).
            Data = Self.rfile.read(min(Length, 4 * MaxBodyBytes)) if Length > 0 else b""
            if Route not in ("/task", "/quit"):
                Self.send_error(404)
                return
            Refusal = LocalProgramRefusal(Self.headers, Self.server.server_address[1])
            if Refusal:
                Self.Reply(Refusal[0], {"ok": False, "error": Refusal[1]})
                return
            if not 0 < Length <= MaxBodyBytes:
                Self.Reply(413 if Length else 411, {"ok": False, "error": f"send one JSON object of at most {MaxBodyBytes} bytes"})
                return
            try:
                Body = json.loads(Data)
            except ValueError:
                Body = None
            if not isinstance(Body, dict):
                Self.Reply(400, {"ok": False, "error": "send one JSON object"})
                return
            if Route == "/quit":
                Self.Reply(200, {"ok": True, "stopping": True, "pid": os.getpid()})
                if OnQuit:
                    threading.Thread(target=OnQuit, name="RecorderQuit", daemon=True).start()
                return
            if Gate is None:
                Self.Reply(409, {"ok": False, "error": "this recorder records every frame (--always)"})
                return
            State, Key = Body.get("state"), Body.get("task_id")
            if State not in ("start", "alive", "end") or not isinstance(Key, str) or not TaskKey.match(Key):
                Self.Reply(400, {"ok": False, "error": 'send {"state": "start" | "alive" | "end", "task_id": "<key>"}'})
                return
            Task, Reason = Body.get("task"), Body.get("reason")
            Self.Reply(200, {"ok": True, **Gate.Signal(State, Key, Task[:500] if isinstance(Task, str) else "",
                                                       Reason[:300] if isinstance(Reason, str) else "")})

        def log_message(Self, *Args):  # keep the log to recording events
            pass

    Server = ExclusiveServer(("127.0.0.1", Port), Handler)
    threading.Thread(target=Server.serve_forever, name="RecorderStatus", daemon=True).start()
    return Server


def ReadHealth(Port: int, Timeout: float = 2.0) -> dict[str, Any] | None:
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{Port}/health", timeout=Timeout) as Response:
            Health = json.loads(Response.read())
        return Health if isinstance(Health, dict) else None
    except (urllib.error.URLError, OSError, ValueError):
        return None


def ReplaceOlder(Port: int, WaitS: float = 10.0) -> bool:
    """Stop the recorder on Port when it is an older version, so this one can take the port (the dev server starts
    the recorder with --replace-older whenever the running one does not report RecorderVersion). Asks it to quit
    (versions that have POST /quit), then ends its process. False when the port's holder is this version, not a
    recorder, or still there after WaitS."""
    Health = ReadHealth(Port)
    if not Health or Health.get("service") != "so101-recorder" or Health.get("version") == RecorderVersion:
        return False
    Pid = Health.get("pid")
    if not isinstance(Pid, int) or Pid <= 0 or Pid == os.getpid():
        return False
    print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} stopping the older recorder on 127.0.0.1:{Port} "
          f"(pid {Pid}, version {Health.get('version') or 'from before versions'})", flush=True)
    if Health.get("version"):
        try:
            Request = urllib.request.Request(f"http://127.0.0.1:{Port}/quit", data=b"{}", method="POST",
                                             headers={"Content-Type": "application/json"})
            urllib.request.urlopen(Request, timeout=2.0).close()
        except (urllib.error.URLError, OSError):
            pass
    Deadline = time.monotonic() + WaitS
    Killed = not Health.get("version")
    if Killed:
        try:
            os.kill(Pid, signal.SIGTERM)  # Windows: TerminateProcess; its open segment loses at most one fragment
        except OSError:
            pass
    while time.monotonic() < Deadline:
        if ReadHealth(Port, 0.5) is None:
            return True
        if not Killed and time.monotonic() > Deadline - WaitS / 2:
            Killed = True
            try:
                os.kill(Pid, signal.SIGTERM)
            except OSError:
                pass
        time.sleep(0.2)
    return False


def Main(Arguments: list[str] | None = None) -> int:
    Parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    Parser.add_argument("--bridge", default=os.environ.get("SO101_BRIDGE_URL", "http://127.0.0.1:8000"))
    Parser.add_argument("--root", type=Path, default=DefaultRoot)
    Parser.add_argument("--segment-minutes", type=float, default=SegmentSeconds / 60)
    Parser.add_argument("--crf", type=int, default=23, help="x264 quality: lower is better and larger")
    Parser.add_argument("--port", type=int, default=StatusPort)
    Parser.add_argument("--webcam", default=DefaultWebcam, help='DirectShow name of the demo-area webcam ("" for none)')
    Parser.add_argument("--webcam-fps", type=float, default=WebcamFps)
    Parser.add_argument("--pre-roll", type=float, default=PreRollS, help="seconds kept before a task starts")
    Parser.add_argument("--post-roll", type=float, default=PostRollS, help="seconds recorded after a task ends")
    Parser.add_argument("--keep-days", type=float, default=VideoRetention.KeepDays, help="video older than this is deleted")
    Parser.add_argument("--always", action="store_true", help="record every frame, task or not (for checks)")
    Parser.add_argument("--replace-older", action="store_true", help="stop an older recorder on the port first")
    Args = Parser.parse_args(Arguments)
    Events = EventLog(Args.root)
    Gate = None if Args.always else TaskGate(PostRoll=Args.post_roll, OnEvent=Events)
    Instance = Recorder(Args.bridge, Args.root, Args.segment_minutes * 60, Args.crf, Gate, Args.pre_roll)
    Webcam = (WebcamRecorder(Args.webcam, Args.root, Args.segment_minutes * 60, Args.crf, Args.webcam_fps,
                             Gate=Gate, PreRoll=Args.pre_roll) if Args.webcam else None)
    for Item in (Instance, Webcam):
        if Item is not None:
            Item.OnEvent = Events
            Item.FlushInBackground = True
    Keeper = Supervisor([Instance, Webcam], Args.root, Args.keep_days)

    def Quit() -> None:
        Instance.Running = False
        if Webcam:
            Webcam.Running = False

    Server = None
    try:
        Server = ServeStatus(Instance, Args.port, Webcam, Gate, Keeper.Status, Quit)
    except OSError:
        Replaced = Args.replace_older and ReplaceOlder(Args.port)
        # The old process has let go of its port, or is about to: a few tries.
        for _ in range(15 if Replaced else 0):
            time.sleep(0.3)
            try:
                Server = ServeStatus(Instance, Args.port, Webcam, Gate, Keeper.Status, Quit)
                break
            except OSError:
                continue
        if Server is None:
            print(f"Another recorder already serves 127.0.0.1:{Args.port}; exiting.", flush=True)
            return 0
    print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} recorder {RecorderVersion}: {Args.bridge}"
          + (f" and webcam {Args.webcam!r}" if Webcam else "") + f" into {Args.root}, "
          + ("every frame (--always)" if Args.always else
             f"while an AI task runs ({Args.pre_roll:g} s before, {Args.post_roll:g} s after)")
          + f"; video kept {Args.keep_days:g} days", flush=True)
    threading.Thread(target=Keeper.Run, name="RecorderSupervisor", daemon=True).start()
    if Webcam:
        threading.Thread(target=Webcam.Run, name="WebcamRecorder", daemon=True).start()
    try:
        Instance.Run()
    except KeyboardInterrupt:
        pass
    finally:
        Keeper.Stopped.set()
        if Webcam:
            Webcam.Running = False
            Webcam.Stop()
        Instance.Stop()
        if Server is not None:
            Server.shutdown()
            Server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(Main())
