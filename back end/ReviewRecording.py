"""Look back at a recorded session: what was asked, what the AI saw and said, what the arm did.

Reads a day's back end/captures/YYYY-MM-DD/recording/ (see SessionRecorder.py,
SessionLog.py and "AI Controlled WebApp/scripts/SessionLog.ts"):

  python ReviewRecording.py timeline [--date 2026-09-28] [--from 13:08] [--to 13:12] [--full]
      Every prompt, model request and reply (Ollama's native /api/chat, and the
      /v1 records before it), model warm-up, measurement, snap, approval, bridge
      refusal and fault (bridge_events.jsonl), task end, the recorder's own record
      (recorder_events.jsonl: each task it was told of, and when and why each
      camera's recording started and stopped) and video segment in time order.
      Times are local.
  python ReviewRecording.py frame --at 13:09:47.5 [--date ...] [--out file.jpg]
      The video frame nearest that moment, as a JPEG (prints its path). --camera webcam:
      the demo-area webcam's instead of the depth camera's (frame and sheet).
  python ReviewRecording.py sheet --from 13:09:40 --to 13:10:30 [--every 5] [--columns 4]
      A contact sheet of frames across a time span, each labelled with its time.

Since 2026-10-08 video exists only around AI tasks (from 5 s before each to 10 s
after it) and for 7 days (SessionRecorder.py, VideoRetention.py); the text logs
are kept. Segments are HHMMSS_mmm_<camera> (HHMMSS_<camera> before), with -1,
-2 for another segment started in the same millisecond.
"""

from __future__ import annotations

import argparse
import datetime
import json
import math
import re
import sys
from fractions import Fraction
from pathlib import Path
from typing import Any, Iterator

import av
import cv2
import numpy as np

import SessionLog

MaxFrameGapS = 2.0
AiLogName = "ai.jsonl"  # written by the web app's dev server (scripts/SessionLog.ts)
BridgeEventsName = "bridge_events.jsonl"  # the bridge's faults, trips, aborts and serial link drops
BridgeEventLabels = {"motion_fault": "BRIDGE FAULT", "protection_trip": "BRIDGE TRIP", "motion_abort": "BRIDGE ABORT",
                     "serial_link_lost": "SERIAL LOST", "serial_link_recovered": "SERIAL BACK",
                     "serial_link_failed": "SERIAL FAIL"}
RecorderEventsName = "recorder_events.jsonl"  # SessionRecorder.py: tasks it was told of, recordings started and stopped
NoVideoNote = "video is recorded only around AI tasks, from 5 s before each to 10 s after, and kept 7 days"
LoadShownS = 0.1  # a model load shorter than this (the model was already resident) is not printed


# --------------------------------------------------------------------------
# Loading
# --------------------------------------------------------------------------

def DayDirectory(Date: str) -> Path:
    return SessionLog.CapturesRoot() / Date


def ParseClock(Date: str, Clock: str | None) -> float | None:
    """Local wall-clock time on Date ("13:08", "13:08:47", "13:08:47.25") as unix seconds."""
    if not Clock:
        return None
    Match = re.fullmatch(r"(\d{1,2}):(\d{2})(?::(\d{2})(\.\d+)?)?", Clock.strip())
    if not Match:
        raise SystemExit(f"Times look like 13:08, 13:08:47 or 13:08:47.25, not {Clock!r}.")
    Hour, Minute, Second, Fractional = Match.groups()
    When = datetime.datetime.strptime(Date, "%Y-%m-%d").replace(hour=int(Hour), minute=int(Minute), second=int(Second or 0))
    return When.timestamp() + (float(Fractional) if Fractional else 0.0)


def Clock(Stamp: float) -> str:
    return datetime.datetime.fromtimestamp(Stamp).strftime("%H:%M:%S.%f")[:-3]


def ReadJsonLines(Path_: Path) -> Iterator[dict[str, Any]]:
    if not Path_.exists():
        return
    for Line in Path_.read_text(encoding="utf-8").splitlines():
        try:
            Record = json.loads(Line)
        except json.JSONDecodeError:
            continue  # a line cut short by a crash
        if isinstance(Record, dict) and isinstance(Record.get("t"), (int, float)):
            yield Record


def VideoSegments(Date: str, Camera: str | None = "depthcam") -> list[dict[str, Any]]:
    """Every recorded segment of one camera (None: every camera) with its frame index, in time order.

    Names are HHMMSS_mmm_<camera> (HHMMSS_<camera> before 2026-10-08), and HHMMSS_mmm-1_<camera> for a second
    segment started in the same millisecond, which sorts before the first by name: so the order is by first frame."""
    Segments = []
    Pattern = f"*_{Camera}.frames.jsonl" if Camera else "*.frames.jsonl"
    for Index in (DayDirectory(Date) / "recording" / "video").glob(Pattern):
        Frames = list(ReadJsonLines(Index))
        Video = Index.with_name(Index.name.replace(".frames.jsonl", ".mp4"))
        if Frames and Video.exists():
            Segments.append({"video": Video, "frames": Frames, "start": Frames[0]["t"], "end": Frames[-1]["t"]})
    return sorted(Segments, key=lambda Segment: (Segment["start"], Segment["video"].name))


# --------------------------------------------------------------------------
# Timeline
# --------------------------------------------------------------------------

def ImageName(Image: Any) -> str:
    return Image.get("image_file", "?") if isinstance(Image, dict) else "inline image"


def MessageText(Content: Any, Images: Any = None) -> tuple[str, list[str]]:
    """Text and image files of one chat message: /v1 content parts, or Ollama's native
    content string plus its "images" list."""
    Files = [ImageName(Image) for Image in Images] if isinstance(Images, list) else []
    if isinstance(Content, str):
        return Content, Files
    Texts, Parts = [], []
    for Part in Content or []:
        if not isinstance(Part, dict):
            continue
        if Part.get("type") == "text":
            Texts.append(str(Part.get("text", "")))
        elif Part.get("type") == "image_url":
            Parts.append(ImageName((Part.get("image_url") or {}).get("url")))
    return "\n".join(Texts), Parts + Files


def CurrentTurn(Messages: list[Any]) -> tuple[str, list[str]]:
    """Text and image files of a request's own user turn. The native /api/chat request
    sends the text and each image as separate user messages, so the turn is the run of
    user messages at the end (a /v1 request has one); otherwise the last user message."""
    Turn: list[dict[str, Any]] = []
    for Message in reversed(Messages):
        if not isinstance(Message, dict) or Message.get("role") != "user":
            break
        Turn.insert(0, Message)
    if not Turn:
        Last = next((M for M in reversed(Messages) if isinstance(M, dict) and M.get("role") == "user"), None)
        Turn = [Last] if Last else []
    Texts, Images = [], []
    for Message in Turn:
        Text, Files = MessageText(Message.get("content"), Message.get("images"))
        if Text:
            Texts.append(Text)
        Images += Files
    return "\n".join(Texts), Images


def ReplyText(Reply: Any) -> str:
    if isinstance(Reply, dict):
        Choices = Reply.get("choices") or []
        if Choices and isinstance(Choices[0], dict):
            return str((Choices[0].get("message") or {}).get("content", ""))
        if "message" in Reply:  # Ollama's native /api/chat
            return str((Reply.get("message") or {}).get("content", ""))
        return json.dumps(Reply)[:2000]
    return str(Reply or "")


def Tokens(Input: Any, Output: Any, LoadS: Any, CutOff: bool = False) -> str:
    """"5354 in / 88 out, load 14.9 s": what a call cost, from whichever numbers it has."""
    Parts = []
    if isinstance(Input, int) and isinstance(Output, int):
        Parts.append(f"{Input} in / {Output} out")
    if isinstance(LoadS, (int, float)) and LoadS >= LoadShownS:
        Parts.append(f"load {LoadS:.1f} s")
    if CutOff:
        Parts.append("cut off at the token limit")
    return ", ".join(Parts)


def ReplyTokens(Reply: Any) -> str:
    """Token counts and model load time of a native or /v1 reply ("" when it has none)."""
    if not isinstance(Reply, dict):
        return ""
    Usage = Reply.get("usage") if isinstance(Reply.get("usage"), dict) else {}
    Choices = Reply.get("choices") or [{}]
    Finish = Choices[0].get("finish_reason") if isinstance(Choices[0], dict) else None
    Load = Reply.get("load_duration")
    return Tokens(Usage.get("input_tokens", Usage.get("prompt_tokens", Reply.get("prompt_eval_count"))),
                  Usage.get("output_tokens", Usage.get("completion_tokens", Reply.get("eval_count"))),
                  Load / 1e9 if isinstance(Load, (int, float)) else None, "length" in (Reply.get("done_reason"), Finish))


def Number(Value: Any, Unit: str = "") -> str:
    return f"{Value:.1f}{Unit}" if isinstance(Value, (int, float)) else "?"


def Point(Target: Any, Pitch: Any = None) -> str:
    """A tip target as the AI's display frame gives it: "X 5.2 Y 34.7 Z 4.2, wrist -43.1°"."""
    Text = (f"X {Number(Target.get('x'))} Y {Number(Target.get('y'))} Z {Number(Target.get('z'))}"
            if isinstance(Target, dict) else "no target")
    return Text + (f", wrist {Number(Pitch, '°')}" if isinstance(Pitch, (int, float)) else "")


def SnapText(Record: dict[str, Any]) -> str:
    """The app's AI target snap (lib/AiTargetSnap.ts): what was asked, where it was moved, why.
    The snap's kind is event_kind in the record (every app event is kind "app"; records
    written before the evening of 2026-09-29 lost it)."""
    Lead = "re-snapped at approval: " if Record.get("resnapped") else ""
    Kind = Record.get("event_kind") or "snap"
    if Kind == "refused":
        Nearest = Record.get("nearest")
        return (f"{Lead}refused: {Record.get('text') or Record.get('reason') or Record.get('code')}"
                + (f" (nearest {Point(Nearest, Nearest.get('wrist_pitch_degrees'))}, {Number(Nearest.get('distance_cm'), ' cm')} away)"
                   if isinstance(Nearest, dict) else ""))
    Snapped = Record.get("snapped") if isinstance(Record.get("snapped"), dict) else {}
    Asked = f"{Point(Record.get('requested'), Record.get('requested_wrist_pitch_degrees'))} -> " if "requested" in Record else ""
    return (f"{Lead}{Kind}: {Asked}{Point(Snapped, Snapped.get('wrist_pitch_degrees'))} "
            f"({Number(Record.get('distance_cm'), ' cm')}, {Number(Record.get('pitch_change_deg'), '°')}): {Record.get('reason') or ''}").rstrip(": ")


def Short(Text: str, Full: bool, Limit: int = 240) -> str:
    Text = " ".join(str(Text).split())
    return Text if Full or len(Text) <= Limit else Text[:Limit] + " …"


def Motion(Record: dict[str, Any] | None) -> str:
    if not Record:
        return "no motion"
    Parts = []
    Target = Record.get("target_cm")
    if Target:
        Parts.append("target X {x:.1f} Y {y:.1f} Z {z:.1f}".format(**Target))
    if Record.get("wrist_pitch_degrees") is not None:
        Parts.append(f"wrist {Record['wrist_pitch_degrees']:.1f}°")
    for Servo in Record.get("servo_targets") or []:
        Parts.append(f"{Servo['name']}→{Servo['raw']}")
    # A chained motion: its gripper action runs before or after the tip move, on the same approval.
    Chain = Record.get("chain")
    if isinstance(Chain, dict):
        Parts.append(f"chained: gripper→{Chain.get('gripper_raw')} {Chain.get('order')} the tip move")
    return ", ".join(Parts) or "no motion"


def ChainLegText(Record: dict[str, Any]) -> str:
    """One leg of a chained motion as the app sent it (or why it did not); its own "kind" is kept as event_kind."""
    Target = Record.get("target_cm")
    What = "tip to X {x:.1f} Y {y:.1f} Z {z:.1f}".format(**Target) if isinstance(Target, dict) \
        else f"gripper→{Record.get('gripper_raw')}"
    Why = f": {Record['why']}" if Record.get("why") else ""
    return f"leg {Record.get('leg')} of {Record.get('of')}, {What}: {Record.get('result')}{Why} (task step {Record.get('step')})"


def RouteText(Record: dict[str, Any], Full: bool) -> str:
    """What the prompt router (AiPromptRouter.ts) gave the main model, who decided, how long it took, why, and
    Nimble's odds when it was asked. Records before 2026-10-10 have mode text/vision and no "sends"."""
    Text = (f"{Record.get('route') or Record.get('mode')} ({Record.get('source')}, {Record.get('duration_ms')} ms): "
            f"{Short(Record.get('reason') or '', Full)}")
    Sends = Record.get("sends")
    if isinstance(Sends, dict):
        Parts = [Name for Name, Key in (("image", "image"), ("/locate", "measured_positions"), ("depth map", "depth_image"), ("task", "task"))
                 if Sends.get(Key)]
        Text += f" | sends prompt{''.join(f' + {Part}' for Part in Parts)}"
    Odds = Record.get("probabilities")
    if isinstance(Odds, dict):
        Text += f" | Nimble {Record.get('nimble_choice')}: " + ", ".join(
            f"{Name} {Value:.2f}" for Name, Value in Odds.items() if isinstance(Value, (int, float)))
    return Text


def TimelineEntries(Date: str, Full: bool) -> list[tuple[float, str, str]]:
    Directory = DayDirectory(Date)
    Entries: list[tuple[float, str, str]] = []
    for Record in ReadJsonLines(Directory / "recording" / AiLogName):
        if Record.get("kind") == "model":
            Started = Record["t"] - (Record.get("duration_ms") or 0) / 1000
            Request = Record.get("request") if isinstance(Record.get("request"), dict) else {}
            Messages = Request.get("messages") or []
            Reply = Record.get("reply")
            if Record.get("path") == "/v1/systemone":
                State = Request.get("state", "")
                Text = State if isinstance(State, str) else json.dumps(State, ensure_ascii=False)
                Entries.append((Started, "ROUTER ASK", f"{Request.get('model', '?')}: {Short(Text, Full)}"))
                Answer = ReplyText(Reply) if Reply is not None else f"ERROR {Record.get('error')}"
                Cost = ReplyTokens(Reply)
                Entries.append((Record["t"], "ROUTER SAID", f"({Record.get('duration_ms')} ms{', ' + Cost if Cost else ''}) {Short(Answer, Full, 400)}"))
                continue
            # The page's warm-up: no messages, so Ollama only loaded the model.
            if Record.get("preload") or (isinstance(Request.get("messages"), list) and not Messages):
                Outcome = (Reply.get("done_reason") or "done") if isinstance(Reply, dict) else f"ERROR {Record.get('error') or Reply}"
                Entries.append((Record["t"], "MODEL LOAD", f"{Request.get('model', '?')} ({Record.get('duration_ms')} ms): {Outcome}"))
                continue
            Text, Images = CurrentTurn(Messages)
            Entries.append((Started, "MODEL ASK", f"{Request.get('model', '?')}, {len(Messages)} messages, "
                            f"images {Images or 'none'}: {Short(Text, Full)}"))
            Answer = ReplyText(Reply) if Reply is not None else f"ERROR {Record.get('error')}"
            Cost = ReplyTokens(Reply)
            Entries.append((Record["t"], "MODEL SAID", f"({Record.get('duration_ms')} ms{', ' + Cost if Cost else ''}) {Short(Answer, Full, 400)}"))
            continue
        Type = str(Record.get("type", "?"))
        if Type in ("prompt", "task_check_in"):
            Text = f"{Short(Record.get('prompt', ''), Full)} (task step {Record.get('task_step')}, {Record.get('model')})"
        elif Type == "vision":
            Objects = Record.get("measured_objects")
            Found = "not measured (camera uncalibrated or no objects)" if not Objects else "; ".join(
                f"{O.get('label')} X {O.get('x'):.1f} Y {O.get('y'):.1f} Z {O.get('z'):.1f}"
                + (f" top {O['topZ']:.1f}" if O.get("topZ") is not None else "") for O in Objects)
            Text = f"{Record.get('camera_source')} camera, depth frame {Record.get('depth_seq')}: {Found}"
        elif Type == "prompt_route":
            Text = RouteText(Record, Full)
        elif Type == "reply":
            Text = (f"attempt {Record.get('attempt')}: {Short(Record.get('reply', ''), Full)} | {Motion(Record.get('motion'))}"
                    f" | possible {Record.get('possible')} {Record.get('issues') or ''}")
        elif Type == "result":
            Text = (f"{Record.get('status')}: {Motion(Record.get('approvable') or Record.get('shown'))}, "
                    f"task {Record.get('task_status')}, estimate {Record.get('target_estimate_cm')}")
        elif Type in ("approve", "block", "approve_refused"):
            Text = f"{Motion(Record.get('motion'))} (task step {Record.get('step')}) {Record.get('issues') or ''}"
        elif Type == "snap":
            Text = SnapText(Record)
        elif Type == "bridge_refused":
            Text = f"{Record.get('reason')} | {Motion(Record.get('motion'))} (task step {Record.get('step')})"
        elif Type == "chain_leg":
            Text = ChainLegText(Record)
        elif Type == "chain_aborted":
            Text = (f"{Record.get('source')}: {Short(Record.get('reason') or '', Full)} (waiting on part {Record.get('part')}"
                    + (f"; the {Record['dropped']} leg was not sent)" if Record.get("dropped") else ")"))
        elif Type == "chain_done":
            Text = f"{Record.get('order')} the tip move: {', '.join(str(Result) for Result in Record.get('results') or [])}" \
                + (f" ({Record['why']})" if Record.get("why") else "")
        elif Type == "task_end":
            Text = f"{Record.get('source')}: {Short(Record.get('reason') or '', Full)} (task step {Record.get('step')})"
        elif Type == "task_object_bound":
            Distance = Record.get("distance_cm")
            Text = (f"\"{Record.get('label')}\" is the measured \"{Record.get('measured_label')}\" ({Record.get('source')}"
                    + (f", {Distance} cm)" if Distance is not None else ")"))
        elif Type == "bridge_fault":  # the bridge's fault as the app saw it (the bridge's own copy: BRIDGE FAULT)
            Text = (f"{Record['event_kind']}: " if Record.get("event_kind") else "") + str(Record.get("reason")) \
                + (f" ({Record['joint']})" if Record.get("joint") else "")
        else:
            Text = Short(json.dumps({K: V for K, V in Record.items() if K not in ("t", "time", "kind", "type", "client_time")}), Full)
        Entries.append((Record["t"], Type.upper(), Text))
    for Record in ReadJsonLines(Directory / "recording" / "grounding.jsonl"):
        Task = Record.get("task_object") if isinstance(Record.get("task_object"), dict) else None
        For = f" for \"{Task.get('phrase') or Task.get('label')}\"" if Task else ""
        if Record.get("kind") == "locate":
            Objects = Record.get("objects") or []
            Entries.append((Record["t"], "LOCATE", f"{len(Objects)} object(s){For}, depth {Record.get('depth')}: " + "; ".join(
                f"{O.get('label')} ({O.get('method')}) X {O.get('x')} Y {O.get('y')} Z {O.get('z')}"
                + (" [task query only]" if O.get("task_query_only") else " [task]" if O.get("task_match") else "") for O in Objects)))
        else:
            # A detection pass (or the bridge's task query): the reply text, and on the
            # native /api/chat its token counts and model load time (load_ms).
            Result = Record.get("result")
            if isinstance(Result, dict) and isinstance(Result.get("detections"), list):
                # The single-call locate (object_locator LOCATE_MODES): its rows, the task's first when it has one.
                Result = [{**D, "label": f"{D.get('label', '?')} [task]" if D.get("task_row") else D.get("label", "?")}
                          for D in Result["detections"]]
            Summary = (f"{len(Result)} box(es): " + ", ".join(D.get("label", "?") for D in Result)) if isinstance(Result, list) \
                else f"error {Result.get('error') if isinstance(Result, dict) else Result}"
            Load = Record.get("load_ms")
            Cost = Tokens(Record.get("prompt_eval_count"), Record.get("eval_count"), Load / 1000 if isinstance(Load, (int, float)) else None,
                          Record.get("done_reason") == "length")
            Raw = ReplyText(Record.get("reply")) if isinstance(Record.get("reply"), dict) else Record.get("reply") or ""
            Entries.append((Record["t"], {"task_query": "TASK QUERY", "single_call": "SINGLE CALL"}.get(Record.get("kind"), "GROUNDING"),
                            f"{Record.get('model')} {Record.get('image_size')}{For}: {Summary}" + (f" | {Cost}" if Cost else "")
                            + (f" | raw {Short(Raw, Full, 200)}" if Full else "")))
    for Record in ReadJsonLines(Directory / "recording" / BridgeEventsName):
        Kind = str(Record.get("kind", "?"))
        Details = Record.get("details")
        Entries.append((Record["t"], BridgeEventLabels.get(Kind, f"BRIDGE {Kind.upper()}"),
                        f"{Kind}: {Record.get('reason')}" + (f" ({Record['joint']})" if Record.get("joint") else "")
                        + (f" | {Short(json.dumps(Details), Full, 300)}" if Details else "")))
    for Record in ReadJsonLines(Directory / "recording" / RecorderEventsName):
        Kind = str(Record.get("kind", "?"))
        if Kind == "recording_start":
            Text = (f"{Record.get('camera')} recording starts, with the {Number(Record.get('pre_roll_s'), ' s')} before it "
                    f"({Record.get('pre_roll_frames')} frames kept in memory)")
        elif Kind == "recording_stop":
            Text = f"{Record.get('camera')} recording stops: {Record.get('reason')} ({Record.get('segment')})"
        else:  # task_start, task_resumed, task_end, task_lost, task_cut
            Text = f"{Kind}: \"{Short(Record.get('task') or '', Full, 120)}\"" + (f" ({Record['reason']})" if Record.get("reason") else "")
        Entries.append((Record["t"], "RECORDER", Text))
    for Segment in VideoSegments(Date, None):
        Entries.append((Segment["start"], "VIDEO", f"{Segment['video'].name} starts ({len(Segment['frames'])} frames, "
                        f"until {Clock(Segment['end'])})"))
    for Sidecar in sorted(Directory.glob("*_task.json")):
        try:
            Meta = json.loads(Sidecar.read_text(encoding="utf-8"))
            Started = datetime.datetime.fromisoformat(str(Meta["started_at"]).replace("Z", "+00:00")).timestamp()
        except (OSError, ValueError, KeyError, json.JSONDecodeError):
            continue
        Entries.append((Started, "TASK CLIP", f"{Sidecar.with_suffix('').name} \"{Meta.get('task')}\" "
                        f"{Meta.get('duration_s')} s, ended: {Meta.get('reason')}"))
    return sorted(Entries, key=lambda Entry: Entry[0])


# --------------------------------------------------------------------------
# Frames
# --------------------------------------------------------------------------

def NearestFrames(Date: str, Moments: list[float],
                  Camera: str = "depthcam") -> list[tuple[float, dict[str, Any] | None, dict[str, Any] | None]]:
    """(moment, segment, frame index row) for each moment; None where no frame is within MaxFrameGapS."""
    Segments = VideoSegments(Date, Camera)
    Found = []
    for Moment in Moments:
        Best = None
        for Segment in Segments:
            if Segment["start"] - MaxFrameGapS <= Moment <= Segment["end"] + MaxFrameGapS:
                Row = min(Segment["frames"], key=lambda Frame: abs(Frame["t"] - Moment))
                if Best is None or abs(Row["t"] - Moment) < abs(Best[1]["t"] - Moment):
                    Best = (Segment, Row)
        if Best and abs(Best[1]["t"] - Moment) <= MaxFrameGapS:
            Found.append((Moment, Best[0], Best[1]))
        else:
            Found.append((Moment, None, None))
    return Found


def DecodeFrames(Video: Path, PtsMs: list[int]) -> dict[int, np.ndarray]:
    """BGR frames at the given presentation times (ms), decoding forward from the nearest keyframe."""
    Wanted = sorted(set(PtsMs))
    Result: dict[int, np.ndarray] = {}
    with av.open(str(Video)) as Container:
        Stream = Container.streams.video[0]
        Base = Stream.time_base or Fraction(1, 1000)
        for Target in Wanted:
            Container.seek(int(math.floor(Target / 1000 / Base)), stream=Stream, backward=True, any_frame=False)
            Previous = None
            try:
                for Frame in Container.decode(Stream):
                    Milliseconds = round(float(Frame.pts * Base) * 1000)
                    if Milliseconds >= Target:
                        Result[Target] = Frame.to_ndarray(format="bgr24")
                        break
                    Previous = Frame
                else:
                    if Previous is not None:
                        Result[Target] = Previous.to_ndarray(format="bgr24")
            except av.error.InvalidDataError:
                # A segment cut off mid-fragment (the recorder killed, 2026-10-04 16:56:29): its last frame before the cut.
                if Previous is not None:
                    Result[Target] = Previous.to_ndarray(format="bgr24")
    return Result


def Label(Image: np.ndarray, Text: str) -> np.ndarray:
    Image = Image.copy()
    Scale = max(0.5, Image.shape[1] / 1280)
    cv2.putText(Image, Text, (12, int(34 * Scale)), cv2.FONT_HERSHEY_SIMPLEX, Scale, (0, 0, 0), int(5 * Scale), cv2.LINE_AA)
    cv2.putText(Image, Text, (12, int(34 * Scale)), cv2.FONT_HERSHEY_SIMPLEX, Scale, (255, 255, 255), int(2 * Scale), cv2.LINE_AA)
    return Image


def FramesAt(Date: str, Moments: list[float], Camera: str = "depthcam") -> list[tuple[float, np.ndarray | None]]:
    Matches = NearestFrames(Date, Moments, Camera)
    ByVideo: dict[Path, list[int]] = {}
    for _, Segment, Row in Matches:
        if Segment is not None:
            ByVideo.setdefault(Segment["video"], []).append(int(Row["pts_ms"]))
    Decoded = {Video: DecodeFrames(Video, Pts) for Video, Pts in ByVideo.items()}
    return [(Moment, None if Segment is None else Decoded[Segment["video"]].get(int(Row["pts_ms"])))
            for Moment, Segment, Row in Matches]


def DefaultOut(Date: str, Name: str) -> Path:
    Directory = DayDirectory(Date) / "recording" / "review"
    Directory.mkdir(parents=True, exist_ok=True)
    return Directory / Name


# --------------------------------------------------------------------------
# Command line
# --------------------------------------------------------------------------

def Main(Arguments: list[str] | None = None) -> int:
    Parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    Commands = Parser.add_subparsers(dest="command", required=True)
    for Name in ("timeline", "frame", "sheet"):
        Command = Commands.add_parser(Name)
        Command.add_argument("--date", default=datetime.date.today().isoformat())
        Command.add_argument("--out", type=Path)
        if Name != "timeline":
            Command.add_argument("--camera", choices=("depthcam", "webcam"), default="depthcam")
        if Name == "timeline":
            Command.add_argument("--from", dest="since")
            Command.add_argument("--to", dest="until")
            Command.add_argument("--full", action="store_true", help="do not shorten prompts and replies")
        elif Name == "frame":
            Command.add_argument("--at", required=True)
        else:
            Command.add_argument("--from", dest="since", required=True)
            Command.add_argument("--to", dest="until", required=True)
            Command.add_argument("--every", type=float, default=5.0, help="seconds between frames")
            Command.add_argument("--columns", type=int, default=4)
    Args = Parser.parse_args(Arguments)

    if Args.command == "timeline":
        Since, Until = ParseClock(Args.date, Args.since), ParseClock(Args.date, Args.until)
        Lines = [f"{Clock(Stamp)}  {Kind:<14} {Text}" for Stamp, Kind, Text in TimelineEntries(Args.date, Args.full)
                 if (Since is None or Stamp >= Since) and (Until is None or Stamp <= Until)]
        Output = "\n".join(Lines) if Lines else f"Nothing recorded for {Args.date} in that span."
        if Args.out:
            Args.out.write_text(Output + "\n", encoding="utf-8")
            print(Args.out)
        else:
            sys.stdout.reconfigure(encoding="utf-8")
            print(Output)
        return 0

    if Args.command == "frame":
        Moment = ParseClock(Args.date, Args.at)
        [(_, Image)] = FramesAt(Args.date, [Moment], Args.camera)
        if Image is None:
            print(f"No video within {MaxFrameGapS:g} s of {Args.at} on {Args.date} ({NoVideoNote}).")
            return 1
        Out = Args.out or DefaultOut(Args.date, f"frame_{Args.at.replace(':', '').replace('.', '_')}.jpg")
        cv2.imwrite(str(Out), Label(Image, f"{Args.date} {Clock(Moment)}"))
        print(Out)
        return 0

    Since, Until = ParseClock(Args.date, Args.since), ParseClock(Args.date, Args.until)
    if Until <= Since or Args.every <= 0:
        raise SystemExit("--to must come after --from, and --every must be positive.")
    Moments = list(np.arange(Since, Until + 1e-6, Args.every))[:64]
    Tiles, Missing = [], 0
    for Moment, Image in FramesAt(Args.date, Moments, Args.camera):
        Tile = np.zeros((270, 480, 3), np.uint8) if Image is None else cv2.resize(Image, (480, 270), interpolation=cv2.INTER_AREA)
        Tiles.append(Label(Tile, Clock(Moment) + ("" if Image is not None else " (no video)")))
        Missing += Image is None
    if Missing == len(Tiles):
        print(f"No video between {Args.since} and {Args.until} on {Args.date} ({NoVideoNote}).")
    Columns = max(1, Args.columns)
    while len(Tiles) % Columns:
        Tiles.append(np.zeros_like(Tiles[0]))
    Sheet = np.vstack([np.hstack(Tiles[Row:Row + Columns]) for Row in range(0, len(Tiles), Columns)])
    Out = Args.out or DefaultOut(Args.date, f"sheet_{Args.since.replace(':', '')}-{Args.until.replace(':', '')}.jpg")
    cv2.imwrite(str(Out), Sheet, [cv2.IMWRITE_JPEG_QUALITY, 88])
    print(Out)
    return 0


if __name__ == "__main__":
    raise SystemExit(Main())
