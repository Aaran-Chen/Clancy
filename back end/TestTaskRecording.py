"""The session video records only while an AI task runs (Aaran, 2026-10-08): SessionRecorder's TaskGate, the in-memory
pre-roll, the post-roll, the lease and the cap, POST /task, the retention pass, and an older recorder replaced.

Most tests feed frames with chosen times (deterministic); one runs the recorder against a fake bridge in real time,
and one starts recorder processes on a free port (never 8002, the live recorder's). Everything is written to a
scratch captures folder (TestArtifacts/TaskRecording, git-ignored); no camera is opened and no live service touched.

Run:  python TestTaskRecording.py
"""
import http.client
import hashlib
import json
import os
import shutil
import socket
import subprocess
import sys
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

import av
import cv2
import numpy as np

Here = Path(__file__).resolve().parent
Scratch = Here / "TestArtifacts" / "TaskRecording"
os.environ["SO101_CAPTURES_ROOT"] = str(Scratch / "captures")
os.environ["SO101_WEBCAM"] = ""

import ReviewRecording  # noqa: E402
import SessionRecorder  # noqa: E402
import TestSessionRecording  # noqa: E402  (its fake bridge)

Width, Height = 320, 240
LivePorts = {5181, 8000, 8001, 8002, 11434}


def Level(Index: int) -> int:
    return 40 + (Index % 60) * 3


def Jpeg(Index: int) -> bytes:
    return cv2.imencode(".jpg", np.full((Height, Width, 3), Level(Index), np.uint8), [cv2.IMWRITE_JPEG_QUALITY, 95])[1].tobytes()


def Decoded(Video: Path) -> list[np.ndarray]:
    with av.open(str(Video)) as Container:
        return [Frame.to_ndarray(format="bgr24") for Frame in Container.decode(Container.streams.video[0])]


def Segments(Root: Path, When: float, Camera: str = "depthcam") -> list[dict]:
    with patch.dict(os.environ, {"SO101_CAPTURES_ROOT": str(Root)}):
        return ReviewRecording.VideoSegments(time.strftime("%Y-%m-%d", time.localtime(When)), Camera)


def FreePort() -> int:
    with socket.socket() as Probe:
        Probe.bind(("127.0.0.1", 0))
        Port = Probe.getsockname()[1]
    assert Port not in LivePorts
    return Port


class FakeBridge(TestSessionRecording.FakeBridge):
    """TestSessionRecording's fake bridge, with frames that go on (its gray levels pass 255 after 72 frames)."""

    def Produce(Self):
        while Self.Produced < Self.Total:
            with Self.Condition:
                Self.Seq += 1
                Self.Jpeg = Jpeg(Self.Seq)
                Self.Produced += 1
                Self.Condition.notify_all()
            time.sleep(0.04)


class Feeder:
    """Frames at Fps with chosen times, into one recorder; Signals[i] runs just before frame i."""

    def __init__(Self, Recorder, Start: float, Fps: float = 15.0):
        Self.Recorder, Self.Start, Self.Fps = Recorder, Start, Fps
        Self.Jpegs = [Jpeg(Index) for Index in range(60)]

    def Time(Self, Index: int) -> float:
        # Half a frame off the whole seconds, so no frame sits exactly on a boundary under test.
        return Self.Start + (Index + 0.5) / Self.Fps

    def Run(Self, First: int, Last: int, Signals: dict | None = None) -> None:
        for Index in range(First, Last):
            if Signals and Index in Signals:
                Signals[Index]()
            Self.Recorder.Offer(Self.Jpegs[Index % 60], Index, Self.Time(Index))


class GateTests(unittest.TestCase):
    def TestTheDefaults(Self):
        Self.assertEqual((SessionRecorder.PreRollS, SessionRecorder.PostRollS), (5.0, 10.0))
        Self.assertEqual((SessionRecorder.LeaseS, SessionRecorder.MaxTaskS), (180.0, 3600.0))
        Self.assertEqual(SessionRecorder.VideoRetention.KeepDays, 7.0)
        Version = json.loads((Here / "RecorderVersion.json").read_text(encoding="utf-8"))["Version"]
        Self.assertEqual(SessionRecorder.RecorderVersion, Version)

    def TestStartAliveEndAndThePostRoll(Self):
        Events = []
        Gate = SessionRecorder.TaskGate(PostRoll=10, Lease=180, MaxTask=3600, OnEvent=Events.append)
        Self.assertFalse(Gate.Recording(100.0))
        Self.assertTrue(Gate.Signal("start", "a1", "grab the tin", Now=100.0)["accepted"])
        Self.assertTrue(Gate.Recording(150.0))
        Gate.Signal("alive", "a1", Now=160.0)
        Self.assertTrue(Gate.Signal("end", "a1", Reason="the AI reported the task done.", Now=170.0)["accepted"])
        Self.assertTrue(Gate.Recording(179.9), "the post-roll")
        Self.assertFalse(Gate.Recording(180.1))
        Self.assertEqual([Event["kind"] for Event in Events], ["task_start", "task_end"])
        Self.assertEqual(Events[1]["reason"], "the AI reported the task done.")
        Self.assertFalse(Gate.Signal("end", "never-seen", Now=200.0)["accepted"])
        Self.assertFalse(Gate.Recording(200.0), "an end for a task it never heard of starts no post-roll")

    def TestOverlappingTasksRecordUntilTheLastEnds(Self):
        Gate = SessionRecorder.TaskGate(PostRoll=10)
        Gate.Signal("start", "a", Now=0.0)
        Gate.Signal("start", "b", Now=5.0)  # another page's task
        Gate.Signal("end", "a", Now=8.0)
        Self.assertTrue(Gate.Recording(30.0), "b still runs")
        Gate.Signal("alive", "b", Now=30.0)
        Gate.Signal("end", "b", Now=40.0)
        Self.assertTrue(Gate.Recording(49.0))
        Self.assertFalse(Gate.Recording(50.5))

    def TestAPageThatGoesQuietLosesItsTaskAfterTheLease(Self):
        Events = []
        Gate = SessionRecorder.TaskGate(PostRoll=10, Lease=180, OnEvent=Events.append)
        Gate.Signal("start", "quiet", Now=0.0)
        Gate.Signal("alive", "quiet", Now=100.0)
        Self.assertTrue(Gate.Recording(279.0), "a minute-long gap (a background tab) is no loss")
        Self.assertFalse(Gate.Recording(281.0), "180 s without a word: lost, and no post-roll after it")
        Self.assertEqual(Events[-1]["kind"], "task_lost")
        Self.assertIn("no word from its page for 180 s", Events[-1]["reason"])
        # Its page comes back (a dev server restarted): the next alive records again.
        Self.assertTrue(Gate.Signal("alive", "quiet", Now=290.0)["accepted"])
        Self.assertEqual(Events[-1]["kind"], "task_resumed")
        Self.assertTrue(Gate.Recording(291.0))

    def TestATaskThatNeverEndsIsCutAtTheCap(Self):
        Events = []
        Gate = SessionRecorder.TaskGate(PostRoll=10, Lease=180, MaxTask=3600, OnEvent=Events.append)
        Gate.Signal("start", "stuck", "pick up the cup", Now=0.0)
        for Second in range(60, 3600, 60):
            Gate.Signal("alive", "stuck", Now=float(Second))
        Self.assertTrue(Gate.Recording(3599.0))
        Self.assertFalse(Gate.Recording(3601.0))
        Self.assertEqual(Events[-1]["kind"], "task_cut")
        Answer = Gate.Signal("alive", "stuck", Now=3615.0)
        Self.assertFalse(Answer["accepted"], "its page keeps saying alive: still not recorded")
        Self.assertIn("ran past 60 min", Answer["why"])
        Self.assertFalse(Gate.Recording(3616.0))
        Gate.Signal("end", "stuck", Now=3700.0)
        Self.assertTrue(Gate.Signal("start", "next", Now=3800.0)["accepted"], "the next task records")
        Self.assertTrue(Gate.Recording(3801.0))

    def TestAnAliveFromATaskItNeverSawStartsIt(Self):
        Events = []
        Gate = SessionRecorder.TaskGate(OnEvent=Events.append)
        Gate.Signal("alive", "mid-task", Now=10.0)
        Self.assertTrue(Gate.Recording(11.0))
        Self.assertEqual([Event["kind"] for Event in Events], ["task_resumed"])


class RollTests(unittest.TestCase):
    def setUp(Self):
        # A short folder per test (a digest of its name): named after the 40 to 63 character test names, the longest
        # segment index path was 246 of Windows' 259 characters here, and a checkout 14 characters deeper failed
        # (WinError 206; long paths are off on this PC; 2026-10-08 regression review).
        Self.Root = Scratch / "rolls" / hashlib.sha1(Self._testMethodName.encode()).hexdigest()[:8]
        shutil.rmtree(Self.Root, ignore_errors=True)
        Self.Events = []
        Self.Gate = SessionRecorder.TaskGate(PostRoll=10.0, Lease=180.0, MaxTask=3600.0, OnEvent=Self.Events.append)
        # 5.02 s, not 5: with frames every 1/15 s a 5 s window would end exactly on a frame, in or out by rounding.
        Self.Recorder = SessionRecorder.Recorder("http://127.0.0.1:9", Self.Root, SegmentLength=600, Gate=Self.Gate, PreRoll=5.02)
        Self.Recorder.OnEvent = Self.Events.append
        # Today, a few minutes ago (the day folder comes from the frame times).
        Self.Start = time.time() - 600
        Self.Feed = Feeder(Self.Recorder, Self.Start)

    def Check(Self, Segment: dict, From: float, Until: float) -> None:
        """Every frame of the segment lies in [From, Until), none is missing from it, and each decodes to its own."""
        Times = [Row["t"] for Row in Segment["frames"]]
        Self.assertGreaterEqual(Times[0], From - 1e-3)
        Self.assertLess(Times[0], From + 1 / 15)
        Self.assertLess(Times[-1], Until)
        Self.assertGreaterEqual(Times[-1], Until - 1 / 15 - 1e-3)
        Self.assertEqual(len(Times), round((Times[-1] - Times[0]) * 15) + 1, "no frame missing inside the recording")
        Images = Decoded(Segment["video"])
        Self.assertEqual(len(Images), len(Times))
        for Image, Row in list(zip(Images, Segment["frames"]))[::25]:
            Self.assertLess(abs(float(Image.mean()) - Level(Row["seq"])), 4.0)

    def TestOnlyTheTaskIsRecordedFromFiveSecondsBeforeToTenSecondsAfter(Self):
        Start = Self.Start
        Self.Feed.Run(0, 15 * 60, {15 * 20: lambda: Self.Gate.Signal("start", "t1", "grab the tin", Now=Start + 20),
                                   15 * 30: lambda: Self.Gate.Signal("end", "t1", Reason="done", Now=Start + 30)})
        [Segment] = Segments(Self.Root, Start)
        Self.Check(Segment, Start + 15, Start + 40)
        Kinds = [(Event["kind"], Event.get("camera")) for Event in Self.Events]
        Self.assertEqual(Kinds, [("task_start", None), ("recording_start", "depthcam"), ("task_end", None),
                                 ("recording_stop", "depthcam")])
        Started = Self.Events[1]
        Self.assertAlmostEqual(Started["pre_roll_s"], 5.0, delta=0.1)
        Self.assertEqual(Started["pre_roll_frames"], 75)
        Self.assertEqual(Self.Events[3]["segment"], Segment["video"].name)
        Self.assertIsNone(Self.Recorder.Segment, "closed with the post-roll's end")

    def TestThePreRollIsWrittenBesideTheCameraThreadWhichNeverWaitsOnIt(Self):
        # 2026-10-08 review: the first frame of a task held its camera thread 0.42-0.56 s while the 5 s pre-roll (75
        # frames) was decoded and encoded, so every recording had a 0.6 s gap just after its task started. With the
        # running recorder's FlushInBackground the camera thread hands it to a thread of its own and goes on; frames that
        # come meanwhile wait behind it, in order, and a close (here the task's end and its post-roll) writes them first.
        Start, Gate, Recorder = Self.Start, Self.Gate, Self.Recorder
        Recorder.FlushInBackground = True
        Original = SessionRecorder.Recorder.Decode
        Slow = staticmethod(lambda Payload: time.sleep(0.006) or Original(Payload))  # 75 frames: about 0.45 s, as measured
        Held = []
        with patch.object(SessionRecorder.Recorder, "Decode", Slow):
            Self.Feed.Run(0, 15 * 20)
            Gate.Signal("start", "t1", "grab the tin", Now=Start + 20)
            for Index in range(15 * 20, 15 * 23):
                Began = time.perf_counter()
                Recorder.Offer(Self.Feed.Jpegs[Index % 60], Index, Self.Feed.Time(Index))
                Held.append(time.perf_counter() - Began)
            Self.assertIsNotNone(Recorder.Flusher, "the pre-roll is being written beside the camera thread")
            Self.assertLess(max(Held[:5]), 0.2, f"the first frames waited {max(Held[:5]):.3f} s (the pre-roll took about 0.45)")
            Self.Feed.Run(15 * 23, 15 * 30)
            Gate.Signal("end", "t1", Reason="done", Now=Start + 30)
            Self.Feed.Run(15 * 30, 15 * 45)
        Self.assertIsNone(Recorder.Flusher)
        [Segment] = Segments(Self.Root, Start)
        Self.Check(Segment, Start + 15, Start + 40)  # the pre-roll, the task and its post-roll: no frame missing
        Self.assertEqual([Row["seq"] for Row in Segment["frames"]], sorted(Row["seq"] for Row in Segment["frames"]), "in order")
        Self.assertIsNone(Recorder.Segment, "closed with the post-roll's end")

    def TestAStopWhileThePreRollIsBeingWrittenWritesItAllThenCloses(Self):
        Start, Gate, Recorder = Self.Start, Self.Gate, Self.Recorder
        Recorder.FlushInBackground = True
        Original = SessionRecorder.Recorder.Decode
        with patch.object(SessionRecorder.Recorder, "Decode", staticmethod(lambda Payload: time.sleep(0.006) or Original(Payload))):
            Self.Feed.Run(0, 15 * 20)
            Gate.Signal("start", "t1", Now=Start + 20)
            Self.Feed.Run(15 * 20, 15 * 20 + 3)
            Self.assertTrue(Recorder.Flusher and Recorder.Flusher.is_alive())
            Recorder.Stop()  # the recorder told to quit mid-flush: everything so far is written, then the segment closes
        Self.assertIsNone(Recorder.Segment)
        [Segment] = Segments(Self.Root, Start)
        Self.assertEqual(len(Segment["frames"]), 75 + 3)
        Self.assertEqual(len(Decoded(Segment["video"])), 75 + 3)

    def TestEachTaskIsItsOwnRecordingAndATaskInThePostRollContinuesIt(Self):
        Start, Gate = Self.Start, Self.Gate
        Self.Feed.Run(0, 15 * 120, {
            15 * 10: lambda: Gate.Signal("start", "a", Now=Start + 10), 15 * 12: lambda: Gate.Signal("end", "a", Now=Start + 12),
            15 * 50: lambda: Gate.Signal("start", "b", Now=Start + 50), 15 * 55: lambda: Gate.Signal("end", "b", Now=Start + 55),
            15 * 60: lambda: Gate.Signal("start", "c", Now=Start + 60), 15 * 62: lambda: Gate.Signal("end", "c", Now=Start + 62)})
        First, Second = Segments(Self.Root, Start)
        Self.Check(First, Start + 5, Start + 22)
        Self.Check(Second, Start + 45, Start + 72)  # b's post-roll ran into c: one recording
        Self.assertEqual(sum(Event["kind"] == "recording_start" for Event in Self.Events), 2)

    def TestNothingIsDecodedOrWrittenBetweenTasksAndThePreRollStaysSmall(Self):
        Calls = []
        Original = SessionRecorder.Recorder.Decode
        with patch.object(SessionRecorder.Recorder, "Decode", staticmethod(lambda Payload: Calls.append(1) or Original(Payload))):
            Self.Feed.Run(0, 15 * 30)
            Self.assertEqual(Calls, [], "between tasks frames stay compressed")
            Self.assertFalse((Self.Root).exists() and any(Self.Root.rglob("*.mp4")), "and nothing is written")
            Self.assertIn(len(Self.Recorder.PreRoll), (75, 76, 77), "about 5 s at 15 fps")
        # Frames all stamped alike (a clock that stood still): memory still bounded.
        for Index in range(2000):
            Self.Recorder.Offer(b"x", Index, Self.Start)
        Self.assertLessEqual(len(Self.Recorder.PreRoll), SessionRecorder.MaxPreRollFrames)

    def TestALostTaskStopsAtTheLeaseAndACutOneAtTheCap(Self):
        Gate = SessionRecorder.TaskGate(PostRoll=2.0, Lease=6.0, MaxTask=20.0, OnEvent=Self.Events.append)
        Self.Recorder.Gate = Gate
        Start = Self.Start
        Signals = {15 * 5: lambda: Gate.Signal("start", "quiet", Now=Start + 5)}
        # A second task whose page keeps saying alive every 3 s past the 20 s cap.
        Signals[15 * 40] = lambda: Gate.Signal("start", "stuck", Now=Start + 40)
        for Second in range(43, 75, 3):
            Signals[15 * Second] = lambda Second=Second: Gate.Signal("alive", "stuck", Now=Start + Second)
        Self.Feed.Run(0, 15 * 80, Signals)
        Lost, Cut = Segments(Self.Root, Start)
        Self.Check(Lost, Start + 0, Start + 11)   # 5 s before its start; lost 6 s after its last word, no post-roll
        Self.Check(Cut, Start + 35, Start + 60)   # cut 20 s after it started, though its page kept on
        Self.assertIn("task_lost", [Event["kind"] for Event in Self.Events])
        Self.assertIn("task_cut", [Event["kind"] for Event in Self.Events])

    def TestTheSupervisorEndsARecordingWhenNoFrameComes(Self):
        Start, Gate = Self.Start, Self.Gate
        # The camera stops sending mid-task (unplugged): the recording is closed once it has been quiet 5 s.
        Self.Feed.Run(0, 15 * 12, {15 * 10: lambda: Gate.Signal("start", "t", Now=Start + 10)})
        Self.assertIsNotNone(Self.Recorder.Segment)
        Self.Recorder.Tick(Start + 15.0)
        Self.assertIsNotNone(Self.Recorder.Segment, "quiet 3 s: still open")
        Self.Recorder.Tick(Start + 17.6)
        Self.assertIsNone(Self.Recorder.Segment, "quiet over 5 s: closed (it stayed open for 6 h on 2026-10-06)")
        Self.assertIn("no frame", Self.Recorder.Error)
        # Frames come back while the task runs: a new recording begins.
        Self.Feed.Run(15 * 20, 15 * 26, {15 * 25: lambda: Gate.Signal("end", "t", Now=Start + 25)})
        Self.assertIsNotNone(Self.Recorder.Segment)
        # Then the camera goes quiet again inside the post-roll (until +35): closed 5 s after its last frame (+26).
        Self.Recorder.Tick(Start + 30.5)
        Self.assertIsNotNone(Self.Recorder.Segment)
        Self.Recorder.Tick(Start + 31.2)
        Self.assertIsNone(Self.Recorder.Segment)
        Stops = [Event["reason"] for Event in Self.Events if Event["kind"] == "recording_stop"]
        Self.assertEqual(Stops, ["no frame for 5 s", "no frame for 5 s"])
        First, Second = Segments(Self.Root, Start)
        Self.Check(First, Start + 5, Start + 12)
        Self.Check(Second, Start + 20, Start + 26)

    def TestATaskEndingWithNoFrameComingIsClosedByTheGate(Self):
        Start, Gate = Self.Start, Self.Gate
        Self.Feed.Run(0, 15 * 33, {15 * 30: lambda: Gate.Signal("start", "u", Now=Start + 30)})
        Gate.Signal("end", "u", Now=Start + 32)
        Self.Recorder.Tick(Start + 35.0)  # quiet 2 s, inside the post-roll
        Self.assertIsNotNone(Self.Recorder.Segment)
        Gate.Signal("start", "v", Now=Start + 36.0)
        Gate.Signal("end", "v", Now=Start + 36.5)   # ... another, shorter task: post-roll until +46.5
        Self.Recorder.Offer(Self.Feed.Jpegs[0], 999, Start + 37.0)
        Self.Recorder.Tick(Start + 46.6)
        Self.assertIsNone(Self.Recorder.Segment)
        Self.assertEqual([Event["reason"] for Event in Self.Events if Event["kind"] == "recording_stop"], ["its task's recording ended"])

    def TestTheWebcamsCompressedFramesAreDecodedOnlyWhenWritten(Self):
        """The webcam's MJPEG frames, also without Huffman tables (as many webcams send them), go into the pre-roll
        as they came and decode to the right picture when a task starts."""
        Webcam = SessionRecorder.WebcamRecorder("Fake Cam", Self.Root, SegmentLength=600, Gate=Self.Gate, PreRoll=5.02)

        def StripHuffman(Data: bytes) -> bytes:
            Out, Index = bytearray(Data[:2]), 2
            while Index < len(Data):
                Marker, Size = Data[Index + 1], (Data[Index + 2] << 8) | Data[Index + 3]
                if Marker == 0xDA:
                    return bytes(Out + Data[Index:])
                if Marker != 0xC4:
                    Out += Data[Index:Index + 2 + Size]
                Index += 2 + Size
            return bytes(Out)

        Start = Self.Start
        for Index in range(15 * 25):
            if Index == 15 * 10:
                Self.Gate.Signal("start", "w", Now=Start + 10)
            if Index == 15 * 12:
                Self.Gate.Signal("end", "w", Now=Start + 12)
            Webcam.Offer(StripHuffman(Jpeg(Index)), Index, Start + (Index + 0.5) / 15)
        Webcam.CloseSegment()
        [Segment] = Segments(Self.Root, Start, "webcam")
        Self.assertTrue(Segment["video"].name.endswith("_webcam.mp4"))
        Self.Check(Segment, Start + 5, Start + 22)

    def TestNoSegmentStaysOpenOnceItsTasksRecordingEnded(Self):
        """A frame written just as its task's recording ended (the supervisor's close landing between the frame's gate
        check and its write, before Offer and Tick were serialized) left a one-frame segment that nothing closed: the
        next task then appended to it and lost its pre-roll. Now the next frame, or the next Tick, closes any segment
        open while the gate is closed, and the next task begins in a segment of its own."""
        Start, Image = Self.Start, SessionRecorder.Recorder.Decode(Jpeg(0))
        Self.Recorder.Store(Image, 0, Start + 0.5)  # the stray frame, with no task running
        Self.assertIsNotNone(Self.Recorder.Segment)
        Self.Recorder.Offer(Jpeg(1), 1, Start + 0.6)
        Self.assertIsNone(Self.Recorder.Segment, "closed by the next frame")
        Self.Recorder.Store(Image, 2, Start + 0.7)
        Self.Recorder.Tick(Start + 0.8)
        Self.assertIsNone(Self.Recorder.Segment, "closed by the next Tick")
        Self.Feed.Run(15 * 2, 15 * 14, {15 * 10: lambda: Self.Gate.Signal("start", "n", Now=Start + 10)})
        Self.Recorder.CloseSegment()  # the task still runs: flush its video before reading it
        Strays, Task = Segments(Self.Root, Start)[:2], Segments(Self.Root, Start)[2]
        Self.assertEqual([len(Item["frames"]) for Item in Strays], [1, 1])
        Self.Check(Task, Start + 5, Start + 14)

    def TestUndecodableFramesAreCountedAndSkipped(Self):
        """A frame neither OpenCV nor FFmpeg can read is skipped and counted (undecodable_frames in /health), so a
        camera whose frames all fail is seen there instead of recording nothing in silence."""
        Self.assertIsNone(SessionRecorder.Recorder.Decode(b"not a jpeg"))
        Self.assertIsNone(SessionRecorder.Recorder.Decode(b"\xff\xd8\xff\xe0 cut short"))
        Start, Gate = Self.Start, Self.Gate
        Gate.Signal("start", "bad", Now=Start)
        for Index in range(10):
            Self.Recorder.Offer(b"\xff\xd8 garbage" if Index % 2 else Self.Feed.Jpegs[Index], Index, Start + 0.5 + Index / 15)
        Self.Recorder.CloseSegment()
        Self.assertEqual(Self.Recorder.Undecodable, 5)
        Self.assertEqual(Self.Recorder.Status()["undecodable_frames"], 5)
        [Segment] = Segments(Self.Root, Start)
        Self.assertEqual([Row["seq"] for Row in Segment["frames"]], [0, 2, 4, 6, 8])

    def TestOpenWebcamHandsOverMjpegFramesStillCompressed(Self):
        """OpenWebcam on a file with the webcam's format (MJPEG): each frame comes out as its JPEG bytes."""
        File = Self.Root / "fake_webcam.mkv"
        File.parent.mkdir(parents=True, exist_ok=True)
        with av.open(str(File), "w") as Out:
            Stream = Out.add_stream("mjpeg", rate=15)
            Stream.width, Stream.height, Stream.pix_fmt = Width, Height, "yuvj420p"
            for Index in range(10):
                for Packet in Stream.encode(av.VideoFrame.from_ndarray(np.full((Height, Width, 3), Level(Index), np.uint8), format="bgr24")):
                    Out.mux(Packet)
            for Packet in Stream.encode(None):
                Out.mux(Packet)
        Frames = list(SessionRecorder.OpenWebcam("unused", Source=str(File), Format=None, Options={}))
        Self.assertEqual(len(Frames), 10)
        Self.assertTrue(all(isinstance(Frame, bytes) and Frame[:2] == b"\xff\xd8" for Frame in Frames))
        for Index, Frame in enumerate(Frames):
            Self.assertLess(abs(float(SessionRecorder.Recorder.Decode(Frame).mean()) - Level(Index)), 4.0)


def Post(Port: int, Path_: str, Body, Headers: dict | None = None) -> tuple[int, dict]:
    Connection = http.client.HTTPConnection("127.0.0.1", Port, timeout=5)
    Data = Body if isinstance(Body, bytes) else json.dumps(Body).encode()
    Connection.request("POST", Path_, body=Data, headers={"Content-Type": "application/json", **(Headers or {})})
    Response = Connection.getresponse()
    Text = Response.read()
    Connection.close()
    try:
        return Response.status, json.loads(Text)
    except ValueError:
        return Response.status, {}


def Get(Port: int, Path_: str = "/health") -> tuple[int, dict, dict]:
    Connection = http.client.HTTPConnection("127.0.0.1", Port, timeout=5)
    Connection.request("GET", Path_)
    Response = Connection.getresponse()
    Body = json.loads(Response.read() or b"{}") if Response.status == 200 else {}
    Headers = dict(Response.getheaders())
    Connection.close()
    return Response.status, Body, Headers


class ControlTests(unittest.TestCase):
    def setUp(Self):
        Self.Gate = SessionRecorder.TaskGate()
        Self.Recorder = SessionRecorder.Recorder("http://127.0.0.1:9", Scratch / "control", Gate=Self.Gate)
        Self.Quit = threading.Event()
        Self.Server = SessionRecorder.ServeStatus(Self.Recorder, 0, None, Self.Gate, lambda: {"retention": {"keep_days": 7}},
                                                  Self.Quit.set)
        Self.Port = Self.Server.server_address[1]

    def tearDown(Self):
        Self.Server.shutdown()
        Self.Server.server_close()

    def TestThePagesTaskSignalsThroughTheDevServer(Self):
        Status, Body = Post(Self.Port, "/task", {"state": "start", "task_id": "page1-7", "task": "grab the tin"})
        Self.assertEqual((Status, Body["ok"], Body["accepted"], Body["recording"]), (200, True, True, True))
        Status, Health, Headers = Get(Self.Port)
        Self.assertEqual(Health["version"], SessionRecorder.RecorderVersion)
        Self.assertEqual(Health["task"]["tasks"][0]["task"], "grab the tin")
        Self.assertEqual((Health["pre_roll_s"], Health["task"]["post_roll_s"]), (5.0, 10.0))
        Self.assertEqual(Health["retention"], {"keep_days": 7})
        Self.assertNotIn("Access-Control-Allow-Origin", Headers, "no web page may read the prompts in it")
        Post(Self.Port, "/task", {"state": "alive", "task_id": "page1-7"})
        Status, Body = Post(Self.Port, "/task", {"state": "end", "task_id": "page1-7", "reason": "the AI reported the task done."})
        Self.assertEqual((Body["accepted"], Body["tasks"], Body["post_roll_left_s"] > 9), (True, [], True))

    def TestOnlyProgramsOnThisPcMayControlIt(Self):
        Start = {"state": "start", "task_id": "x1"}
        Cases = [({"Origin": "http://evil.example"}, 403), ({"Origin": "http://127.0.0.1:5181"}, 403),
                 ({"Sec-Fetch-Site": "cross-site"}, 403), ({"Sec-Fetch-Mode": "no-cors"}, 403),
                 ({"Host": f"evil.example:{Self.Port}"}, 403), ({"Host": "127.0.0.1:8002"}, 403),
                 ({"Content-Type": "text/plain"}, 415)]
        for Headers, Code in Cases:
            with Self.subTest(Headers=Headers):
                Self.assertEqual(Post(Self.Port, "/task", Start, Headers)[0], Code)
        for Body, Code in ((b"", 411), (b"x" * 20000, 413), (b"[1, 2]", 400), (b"not json", 400),
                           ({"state": "begin", "task_id": "x1"}, 400), ({"state": "start", "task_id": "has spaces"}, 400),
                           ({"state": "start"}, 400), ({"state": "start", "task_id": "y" * 81}, 400)):
            with Self.subTest(Body=str(Body)[:30]):
                Self.assertEqual(Post(Self.Port, "/task", Body)[0], Code)
        Self.assertEqual(Self.Gate.Snapshot()["tasks"], [], "no refused request started anything")
        Self.assertFalse(Self.Quit.is_set())
        Self.assertEqual(Post(Self.Port, "/quit", {}, {"Origin": "http://evil.example"})[0], 403)
        Self.assertFalse(Self.Quit.is_set())
        Self.assertEqual(Post(Self.Port, "/quit", {})[0], 200)
        Self.assertTrue(Self.Quit.wait(2))
        Self.assertEqual(Post(Self.Port, "/elsewhere", {})[0], 404)


class RealTimeTests(unittest.TestCase):
    def TestARecordingFollowsItsTaskAgainstAFakeBridge(Self):
        """The recorder's own loop against a fake bridge at 25 fps, the page's signals over POST /task, the supervisor
        closing the recording when its post-roll is over, and recorder_events.jsonl for the timeline."""
        Root = Scratch / "realtime"
        shutil.rmtree(Root, ignore_errors=True)
        Bridge = FakeBridge(Total=10**6)
        Gate = SessionRecorder.TaskGate(PostRoll=1.0, OnEvent=SessionRecorder.EventLog(Root))
        Recorder = SessionRecorder.Recorder(Bridge.Url, Root, Gate=Gate, PreRoll=1.0)
        Recorder.OnEvent = Gate.OnEvent
        Keeper = SessionRecorder.Supervisor([Recorder], Root)
        Server = SessionRecorder.ServeStatus(Recorder, 0, None, Gate, Keeper.Status)
        Port = Server.server_address[1]
        Threads = [threading.Thread(target=Recorder.Run, daemon=True), threading.Thread(target=Keeper.Run, daemon=True)]
        try:
            Bridge.Thread.start()
            for Thread in Threads:
                Thread.start()
            time.sleep(2.0)
            Self.assertIsNone(Recorder.Segment, "nothing is recorded before a task")
            Self.assertGreater(Recorder.Status()["pre_roll_frames"], 10)
            Started = time.time()
            Self.assertTrue(Post(Port, "/task", {"state": "start", "task_id": "rt1", "task": "wave"})[1]["accepted"])
            time.sleep(1.5)
            Self.assertTrue(Get(Port)[1]["recording"])
            Ended = time.time()
            Post(Port, "/task", {"state": "end", "task_id": "rt1", "reason": "done"})
            time.sleep(2.5)
            Self.assertIsNone(Recorder.Segment, "closed after the post-roll")
            Self.assertFalse(Get(Port)[1]["recording"])
        finally:
            Keeper.Stopped.set()
            Recorder.Running = False
            Bridge.Total = 0
            for Thread in Threads:
                Thread.join(timeout=10)
            Recorder.Stop()
            Server.shutdown()
            Server.server_close()
            Bridge.Close()
        [Segment] = Segments(Root, Started)
        Times = [Row["t"] for Row in Segment["frames"]]
        Slack = 0.35  # a loaded PC: the fetch loop and the signal can each be a little late
        Self.assertAlmostEqual(Times[0], Started - 1.0, delta=Slack)
        Self.assertAlmostEqual(Times[-1], Ended + 1.0, delta=Slack)
        Self.assertEqual(len(Decoded(Segment["video"])), len(Times))
        Day = time.strftime("%Y-%m-%d", time.localtime(Started))
        Kinds = [json.loads(Line)["kind"] for Line in (Root / Day / "recording" / "recorder_events.jsonl").read_text(encoding="utf-8").splitlines()]
        Self.assertEqual(Kinds, ["task_start", "recording_start", "task_end", "recording_stop"])
        Self.assertEqual(Keeper.Status()["retention"]["deleted_files"], 0)


class SupervisorTests(unittest.TestCase):
    def TestTheRetentionPassRunsAtStartAndReportsWhatItDeleted(Self):
        Root = Scratch / "supervisor"
        shutil.rmtree(Root, ignore_errors=True)
        Old = time.time() - 8 * 86400
        Video = Root / time.strftime("%Y-%m-%d", time.localtime(Old)) / "recording" / "video"
        Video.mkdir(parents=True)
        Stem = time.strftime("%H%M%S", time.localtime(Old)) + "_123_depthcam"
        for Name in (f"{Stem}.mp4", f"{Stem}.frames.jsonl", "ai.jsonl"):
            (Video / Name).write_bytes(b"x")
            os.utime(Video / Name, (Old, Old))
        Keeper = SessionRecorder.Supervisor([], Root, Every=3600)
        Thread = threading.Thread(target=Keeper.Run, daemon=True)
        Thread.start()
        Deadline = time.time() + 5
        while Keeper.Status()["retention"]["last_pass"] is None and time.time() < Deadline:
            time.sleep(0.05)
        Keeper.Stopped.set()
        Thread.join(timeout=5)
        Retention = Keeper.Status()["retention"]
        Self.assertEqual((Retention["keep_days"], Retention["deleted_files"], Retention["errors"]), (7.0, 2, []))
        Self.assertEqual(sorted(Path_.name for Path_ in Video.iterdir()), ["ai.jsonl"])
        Self.assertIn(f"deleted {Video.relative_to(Root).as_posix()}/{Stem}.mp4", (Root / "video_retention.log").read_text(encoding="utf-8"))

    def TestTheFirstPassKeepsTheFaultVideoBeforeItDeletesAnything(Self):
        # The merge starts this recorder by itself (Vite restarts on the changed vite.config.ts): its first pass keeps the
        # video around every fault the bridge recorded before deleting (KeepVideo.KeepFaultsOnce), and when that cannot
        # finish it deletes nothing that pass.
        Root = Scratch / "supervisor-faults"
        shutil.rmtree(Root, ignore_errors=True)
        Fault = time.time() - 9 * 86400
        Day = time.strftime("%Y-%m-%d", time.localtime(Fault))
        Video = Root / Day / "recording" / "video"
        Video.mkdir(parents=True)
        Stems = {Name: time.strftime("%H%M%S", time.localtime(Start)) + f"_000_{Name}" for Name, Start in (("depthcam", Fault - 60), ("webcam", Fault - 7200))}
        for Name, Stem in Stems.items():
            End = Fault + 60 if Name == "depthcam" else Fault - 3600
            for File in (Video / f"{Stem}.mp4", Video / f"{Stem}.frames.jsonl"):
                File.write_bytes(b"x")
                os.utime(File, (End, End))
        (Root / Day / "recording" / "bridge_events.jsonl").write_text(
            json.dumps({"kind": "motion_fault", "reason": "elbow flex torque unexpectedly off", "t": Fault}) + "\n", encoding="utf-8")
        Everything = sorted(Path_.name for Path_ in Video.iterdir())
        Keeper = SessionRecorder.Supervisor([], Root, Every=3600)
        with patch.object(SessionRecorder.KeepVideo, "KeepFaultsOnce", side_effect=OSError("bridge_events.jsonl unreadable")):
            Keeper.Retain(time.time())
        Self.assertEqual(sorted(Path_.name for Path_ in Video.iterdir()), Everything, "nothing deleted when the keep step failed")
        Self.assertIn("unreadable", Keeper.Status()["retention"]["error"])
        Keeper.Retain(time.time())
        Self.assertEqual(sorted(Path_.name for Path_ in Video.iterdir()), [f"{Stems['depthcam']}.frames.jsonl", f"{Stems['depthcam']}.mp4"])
        Retention = Keeper.Status()["retention"]
        Self.assertEqual((Retention["deleted_files"], Retention["kept_by_list"]), (2, 2))
        Self.assertTrue((Root / SessionRecorder.KeepVideo.FirstPassName).is_file())


FakeOldRecorder = r'''
import json, os, sys, threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
Port, Version = int(sys.argv[1]), (sys.argv[2] or None)
class Handler(BaseHTTPRequestHandler):
    def do_GET(Self):
        Body = json.dumps({"service": "so101-recorder", "ok": True, "pid": os.getpid(), **({"version": Version} if Version else {})}).encode()
        Self.send_response(200); Self.send_header("Content-Length", str(len(Body))); Self.end_headers(); Self.wfile.write(Body)
    def do_POST(Self):
        if Version and Self.path == "/quit":
            Self.send_response(200); Self.send_header("Content-Length", "2"); Self.end_headers(); Self.wfile.write(b"{}")
            threading.Thread(target=lambda: (Server.shutdown(), os._exit(0))).start(); return
        Self.send_error(501)
    def log_message(Self, *Args): pass
Server = ThreadingHTTPServer(("127.0.0.1", Port), Handler)
print("READY", flush=True)
Server.serve_forever()
'''


class ReplaceTests(unittest.TestCase):
    """The dev server starts the recorder with --replace-older when the running one reports another version: the
    recorder before 2026-10-08 (no version, recording around the clock) is ended, a versioned one asked to quit."""

    def setUp(Self):
        Self.Root = Scratch / "replace"
        shutil.rmtree(Self.Root, ignore_errors=True)
        Self.Root.mkdir(parents=True)
        (Self.Root / "FakeOldRecorder.py").write_text(FakeOldRecorder, encoding="utf-8")
        Self.Port = FreePort()
        Self.Env = {**os.environ, "SO101_CAPTURES_ROOT": str(Self.Root / "captures"), "SO101_WEBCAM": "", "SO101_RECORDER": "0",
                    "SO101_BRIDGE_URL": "http://127.0.0.1:9", "PYTHONDONTWRITEBYTECODE": "1"}
        Self.Processes = []

    def tearDown(Self):
        for Process in Self.Processes:
            if Process.poll() is None:
                Process.kill()
                Process.wait(timeout=10)

    def Start(Self, *Arguments) -> subprocess.Popen:
        Self.assertNotIn(Self.Port, LivePorts)
        Process = subprocess.Popen([sys.executable, *Arguments], cwd=Here, env=Self.Env, stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True)
        Self.Processes.append(Process)
        return Process

    def Recorder(Self) -> subprocess.Popen:
        return Self.Start("SessionRecorder.py", "--replace-older", "--port", str(Self.Port), "--root", str(Self.Root / "captures"),
                          "--webcam", "", "--bridge", "http://127.0.0.1:9")

    def WaitForVersion(Self, Version, Seconds: float = 20.0) -> dict | None:
        Deadline = time.time() + Seconds
        while time.time() < Deadline:
            Health = SessionRecorder.ReadHealth(Self.Port, 0.5)
            if Health and Health.get("version") == Version:
                return Health
            time.sleep(0.2)
        return None

    def Old(Self, Version: str) -> subprocess.Popen:
        Old = Self.Start(str(Self.Root / "FakeOldRecorder.py"), str(Self.Port), Version)
        Self.assertEqual(Old.stdout.readline().strip(), "READY")
        return Old

    def TestAnOlderRecorderIsReplacedAndACurrentOneIsLeftAlone(Self):
        Old = Self.Old("")
        New = Self.Recorder()
        Health = Self.WaitForVersion(SessionRecorder.RecorderVersion)
        Self.assertIsNotNone(Health, "the new recorder took over the port")
        Self.assertEqual(Health["pid"], New.pid)
        Self.assertIsNotNone(Old.wait(timeout=10), "the old recorder was ended")
        # A second launch finds the current version: it leaves it running and exits.
        Again = Self.Recorder()
        Output, _ = Again.communicate(timeout=30)
        Self.assertEqual(Again.returncode, 0)
        Self.assertIn("Another recorder already serves", Output)
        Self.assertEqual(SessionRecorder.ReadHealth(Self.Port)["pid"], New.pid)
        # POST /quit ends it cleanly.
        Self.assertEqual(Post(Self.Port, "/quit", {})[0], 200)
        Self.assertEqual(New.wait(timeout=20), 0)

    def TestAVersionedOlderRecorderIsAskedToQuit(Self):
        Old = Self.Old("task-video-v0")
        New = Self.Recorder()
        Self.assertIsNotNone(Self.WaitForVersion(SessionRecorder.RecorderVersion))
        Self.assertEqual(Old.wait(timeout=10), 0, "it quit by itself when asked (exit 0, not ended)")
        Post(Self.Port, "/quit", {})
        New.wait(timeout=20)

    def TestWithoutTheFlagNothingIsReplaced(Self):
        Old = Self.Old("")
        Plain = Self.Start("SessionRecorder.py", "--port", str(Self.Port), "--root", str(Self.Root / "captures"), "--webcam", "",
                           "--bridge", "http://127.0.0.1:9")
        Output, _ = Plain.communicate(timeout=30)
        Self.assertIn("Another recorder already serves", Output)
        Self.assertIsNone(Old.poll(), "the old one still runs")


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestTaskRecording` runs the Test* methods too (unittest names this hook)."""
    Prefixed = unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return unittest.TestSuite(Prefixed.loadTestsFromTestCase(Case) for Case in Cases)


Cases = (GateTests, RollTests, ControlTests, RealTimeTests, SupervisorTests, ReplaceTests)

if __name__ == "__main__":
    Loader = unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Result = unittest.TextTestRunner(verbosity=2).run(unittest.TestSuite(Loader.loadTestsFromTestCase(Case) for Case in Cases))
    raise SystemExit(not Result.wasSuccessful())
