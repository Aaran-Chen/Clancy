"""Replay recorded /locate captures through this checkout's locate_objects, and compare with what was recorded.

Each "locate" record of a day's recording/grounding.jsonl names its raw depth (recording/depth); its colour frame is
the full-frame image its first grounding call sent (recording/images), the arm's pose the servo steps the session
recorded around it (as TestDeskGateGolden reads them) and the calibration the one it was taken with (--calibration,
default: the tracked 2026-10-02 fit for the 10-02..10-12 days, the main checkout's live camera_calibration.json for
10-14; read only, never written). The camera check, the measurements and the scene run as the bridge runs them; the
desk re-measure is never saved and the session record goes to TestArtifacts/ReplayLocate.

The vision model is the live Ollama (127.0.0.1:11434), each reply cached in TestArtifacts/ReplayLocate/Replies.json by
prompt and image, so a second run makes no model call (--offline: never call; a missing reply fails that capture).
Printed per capture: the old path's model calls and their summed time (from the recording), the new path's calls,
time (measured, and estimated at the recording's own speed from its calls' token counts), and each recorded object
against the nearest new one.

  python ReplayLocate.py 2026-10-14 2026-10-03 [--mode single_call|vlm] [--offline] [--limit N]
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
import math
import os
import sys
import time
from pathlib import Path
from unittest.mock import patch

Here = Path(__file__).resolve().parent
Work = Here / "TestArtifacts" / "ReplayLocate"
os.environ["SO101_CAPTURES_ROOT"] = str(Work / "SessionCaptures")
os.environ.setdefault("SO101_DECLARED_SURFACES", str(Here / "TestArtifacts" / "NoDeclaredSurfaces.json"))

import cv2  # noqa: E402
import numpy as np  # noqa: E402

import calibration  # noqa: E402
import object_locator as Locator  # noqa: E402
import RecordedCaptures as RC  # noqa: E402
import RobotReference as RR  # noqa: E402
from TestDeskGateGolden import ArmRaws  # noqa: E402

os.environ["SO101_CAPTURES_ROOT"] = str(Work / "SessionCaptures")  # TestDeskGateGolden points it at its own folder

# The recording's own model speed (2026-10-14 16:37, warm): a full frame with a 33-token reply took 1118 ms and
# with 255 tokens 3946 ms, so ~12.7 ms a reply token over ~700 ms for the frame's 1109 prompt tokens.
RecordedPromptMs = 700.0
RecordedTokenMs = 12.7
MatchCm = 5.0
OriginalHttp = Locator._http_json


def Calibration(Day: str, Named: str | None) -> dict:
    if Named:
        return json.loads(Path(Named).read_text(encoding="utf-8"))["camera_model"]
    if Day >= "2026-10-14":
        Main = RC.Root().parent / "camera_calibration.json"
        return json.loads(Main.read_text(encoding="utf-8"))["camera_model"]
    return RC.CalibrationOf20261002()


def Captures(Day: str) -> list[dict]:
    """The day's locate records with depth on disk, each with its colour frame and old-path call costs."""
    Folder = RC.Day(Day) / "recording"
    Mirrored = {}
    for Line in (Folder / "ai.jsonl").read_text(encoding="utf-8").splitlines():
        Record = json.loads(Line)
        if Record.get("type") == "vision" and Record.get("depth_seq") is not None:
            Mirrored[Record["depth_seq"]] = bool(Record.get("mirrored"))
    Rows, Frames, Calls, Ms, First = [], [], 0, 0, None
    for Line in (Folder / "grounding.jsonl").read_text(encoding="utf-8").splitlines():
        Record = json.loads(Line)
        Kind = Record.get("kind")
        if Kind in ("grounding", "task_query", "single_call"):
            Calls += 1
            Ms += Record.get("duration_ms") or 0
            First = First if First is not None else Record["t"] - (Record.get("duration_ms") or 0) / 1000
            Image = (Record.get("image") or {}).get("image_file") if isinstance(Record.get("image"), dict) else None
            if Image and Record.get("image_size") == [1280, 720] and Image not in Frames and Image not in [Row["image_file"] for Row in Rows]:
                Frames.append(Image)
            continue
        if Kind != "locate":
            continue
        Depth = Folder / str(Record.get("depth_file") or "")
        if Record.get("depth_file") and Depth.is_file():
            Rows.append({"day": Day, "time": Record["time"], "t": Record["t"], "depth": Depth, "seq": Record.get("depth_seq"),
                         "image_file": Frames.pop(0) if Frames else None, "mirrored": Mirrored.get(Record.get("depth_seq"), False),
                         "task": Record.get("task_object"), "old": Record, "old_calls": Calls, "old_model_ms": Ms,
                         "old_wall_s": round(Record["t"] - First, 1) if First is not None else None})
        Calls, Ms, First = 0, 0, None
    return Rows


class Replies:
    """Model replies by prompt and image (module notes)."""

    def __init__(Self, Offline: bool):
        Self.Path = Work / "Replies.json"
        Self.Offline = Offline
        Self.Saved = json.loads(Self.Path.read_text(encoding="utf-8")) if Self.Path.is_file() else {}
        Self.Live = Self.Cached = 0
        Self.Log: list[dict] = []

    def __call__(Self, Url, Payload=None, timeout=10):
        if Payload is None or not Url.endswith("/api/chat"):
            return OriginalHttp(Url, Payload, timeout)
        Images = [Image for Message in Payload["messages"] for Image in Message.get("images", [])]
        Key = hashlib.sha256((json.dumps([M["content"] for M in Payload["messages"]]) + "|".join(Images)).encode()).hexdigest()
        if Key in Self.Saved:
            Self.Cached += 1
            Reply = Self.Saved[Key]
        else:
            if Self.Offline:
                raise OSError("--offline and no cached reply for this call")
            Started = time.time()
            Reply = OriginalHttp(Url, Payload, timeout)
            Reply["replay_wall_ms"] = round((time.time() - Started) * 1000)
            Self.Saved[Key] = Reply
            Self.Path.parent.mkdir(parents=True, exist_ok=True)
            Self.Path.write_text(json.dumps(Self.Saved), encoding="utf-8")
            Self.Live += 1
        Self.Log.append({"prompt_tokens": Reply.get("prompt_eval_count"), "eval_tokens": Reply.get("eval_count"),
                         "wall_ms": Reply.get("replay_wall_ms"), "images": len(Images)})
        return Reply


def ArmPose(Raws: dict | None, Model: dict) -> dict:
    """CaptureArmPose for recorded servo steps, with the arm's link boxes (bridge_server's own conversion)."""
    if not Raws:
        return {"ok": False, "reason": "no recorded servo steps"}
    import bridge_server as Bridge
    from TestRobotReference import ArmFromRaw
    Pose = ArmFromRaw(Raws, RR.ReferenceRaw(Model))
    Controller = Bridge.BridgeController()
    for Name, Value in Raws.items():
        Controller.servos[Name].present_raw, Controller.servos[Name].gui_angle_raw = int(Value), None
    Boxes = Controller.ArmLinkBoxes(Controller._current_ik_angles())
    if Boxes is not None:
        Pose["link_boxes"] = Boxes
    return Pose


def Replay(Row: dict, Model: dict, Mode: str, Arm: dict, Cache: Replies) -> dict:
    Folder = RC.Day(Row["day"]) / "recording"
    Depth = cv2.imread(str(Row["depth"]), cv2.IMREAD_UNCHANGED)
    if Row["image_file"]:
        Image = (Folder / Row["image_file"]).read_bytes()
    else:
        # A capture the camera gate blocked made no model call: the day's archived frame just before it.
        from TestDeskGateGolden import Vision
        Before = [File for Stamp, File in Vision(Row["day"]) if 0.0 <= Row["t"] - Stamp <= 120.0]
        if not Before:
            raise OSError("no colour frame recorded for this capture")
        Image = Before[-1].read_bytes()
    Url = "data:image/jpeg;base64," + base64.b64encode(Image).decode()
    Intrinsics = {Key: Model[Key] for Key in ("fx", "fy", "cx", "cy", "dist")} | {"width": 1280, "height": 720}
    Task = Row["task"]
    Cache.Log = []
    with patch.object(calibration, "camera_model", return_value=json.loads(json.dumps(Model))), \
            patch.object(calibration, "status", return_value={"calibrated": True, "rms_cm": Row["old"].get("rms_cm")}), \
            patch.object(calibration, "save_remeasured_desk", return_value=(False, "replay: never saved")), \
            patch.object(Locator, "_http_json", Cache):
        Started = time.perf_counter()
        Result = Locator.locate_objects(Url, None, "qwen3.8:27b", depth={"depth_mm": Depth, "intrinsics": Intrinsics,
                                        "mirrored": Row["mirrored"], "seq": Row["seq"], "time": Row["t"]},
                                        mirrored=Row["mirrored"], arm=Arm, task_object=Task, locate_mode=Mode)
        Wall = (time.perf_counter() - Started) * 1000
    Result["replay_wall_ms"] = round(Wall)
    Result["replay_calls"] = list(Cache.Log)
    return Result


def Estimate(Log: list[dict]) -> float:
    """The model time the recording's own speed gives these calls (module notes), ms."""
    return sum(RecordedPromptMs * (Call["prompt_tokens"] or 0) / 1109 + RecordedTokenMs * (Call["eval_tokens"] or 0) for Call in Log)


def Compare(Old: dict, New: dict) -> list[dict]:
    """Each recorded object against the nearest new one (XY cm)."""
    Rows = []
    for Item in Old.get("objects") or []:
        Near = min(New.get("objects") or [], key=lambda Other: math.hypot(Other["x"] - Item["x"], Other["y"] - Item["y"]), default=None)
        Distance = math.hypot(Near["x"] - Item["x"], Near["y"] - Item["y"]) if Near else None
        Rows.append({"old": Item["label"], "new": Near["label"] if Near and Distance <= MatchCm else None,
                     "cm": round(Distance, 2) if Distance is not None else None, "old_task": bool(Item.get("task_match")),
                     "new_task": bool(Near and Distance <= MatchCm and Near.get("task_match")),
                     "dz_top": round(Near["top_z"] - Item["top_z"], 2) if Near and Distance <= MatchCm and "top_z" in Item else None})
    return Rows


def Main():
    Parser = argparse.ArgumentParser()
    Parser.add_argument("days", nargs="+")
    Parser.add_argument("--mode", default="single_call")
    Parser.add_argument("--offline", action="store_true")
    Parser.add_argument("--limit", type=int, default=0)
    Parser.add_argument("--every", type=int, default=1, help="every Nth capture of a day")
    Parser.add_argument("--offset", type=int, default=0, help="starting at this one")
    Parser.add_argument("--calibration")
    Parser.add_argument("--json", help="write every result here")
    Arguments = Parser.parse_args()
    Cache = Replies(Arguments.offline)
    Out = []
    for Day in Arguments.days:
        Model = Calibration(Day, Arguments.calibration)
        Requests, Routes = ArmRaws(Day)
        Rows = Captures(Day)[Arguments.offset::Arguments.every][:Arguments.limit or None]
        for Row in Rows:
            After = [Raw for Stamp, Raw in Requests if 0.0 <= Stamp - Row["t"] <= 90.0]
            Last = [Raw for Stamp, Raw in Routes if Stamp < Row["t"] - 0.5]
            Arm = ArmPose(After[0] if After else (Last[-1] if Last else None), Model)
            try:
                New = Replay(Row, Model, Arguments.mode, Arm, Cache)
            except OSError as Exc:
                print(f"{Row['time']}  skipped: {Exc}")
                continue
            Old = Row["old"]
            Pairs = Compare(Old, New) if New.get("ok") and Old.get("ok") else []
            Matched = [Pair for Pair in Pairs if Pair["new"]]
            Task = next((Pair for Pair in Pairs if Pair["old_task"]), None)
            Estimated = Estimate(New.get("replay_calls") or [])
            print(f"{Row['time']}  task={(Row['task'] or {}).get('label')!s:14s} old: {Row['old_calls']:2d} calls {Row['old_model_ms'] / 1000:5.1f} s"
                  f" ({len(Old.get('objects') or [])} obj, ok={Old.get('ok')})  new: {New.get('model_calls')} call(s)"
                  f" {New['replay_wall_ms'] / 1000:5.1f} s, est {Estimated / 1000 + (New['replay_wall_ms'] - sum(C['wall_ms'] or 0 for C in New['replay_calls'])) / 1000:4.1f} s"
                  f" at the recorded speed ({len(New.get('objects') or [])} obj, ok={New.get('ok')} {New.get('error_code') or ''})")
            if Pairs:
                Distances = sorted(Pair["cm"] for Pair in Matched)
                print(f"    matched {len(Matched)}/{len(Pairs)} recorded objects within {MatchCm:g} cm; median "
                      f"{np.median(Distances) if Distances else float('nan'):.2f} cm, max {max(Distances) if Distances else float('nan'):.2f} cm;"
                      f" task: old '{Task['old'] if Task else None}' -> new '{Task['new'] if Task else None}' {Task['cm'] if Task else ''} cm"
                      f" task_match kept={Task['new_task'] if Task else None}")
                for Pair in Pairs:
                    print(f"      {Pair['old']:24s} -> {str(Pair['new']):24s} {Pair['cm']} cm, top {Pair['dz_top']}")
            Out.append({"time": Row["time"], "day": Day, "old_calls": Row["old_calls"], "old_model_ms": Row["old_model_ms"],
                        "old_wall_s": Row["old_wall_s"], "new": {Key: New.get(Key) for Key in ("ok", "error_code", "model_calls", "timing_ms",
                                                                                              "replay_wall_ms", "replay_calls", "locate_mode")},
                        "estimated_model_ms": round(Estimated), "pairs": Pairs,
                        "new_objects": [{K: Item.get(K) for K in ("label", "x", "y", "top_z", "task_match", "refined", "depth_refined", "outline")}
                                        for Item in New.get("objects") or []], "new_withheld": New.get("withheld")})
    print(f"model calls: {Cache.Live} live, {Cache.Cached} from the cache")
    if Arguments.json:
        Path(Arguments.json).write_text(json.dumps(Out, indent=1), encoding="utf-8")


if __name__ == "__main__":
    Main()
