"""Session recording: video recorder, bridge-side log, grounding records and the look-back tool.

A fake bridge serves numbered solid-gray frames (and restarts mid-run, counting
from 1 again); everything is written to a scratch captures folder.

Run:  python TestSessionRecording.py
"""
import base64
import json
import os
os.environ.setdefault("SO101_LOCATE_MODE", "vlm")  # this suite tests the voted multi-call locate; TestSingleCallLocate.py the default
import shutil
import socket
import threading
import time
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from unittest.mock import patch
from urllib.parse import parse_qs, urlparse

import av
import cv2
import numpy as np

Scratch = Path(__file__).resolve().parent / "TestArtifacts" / "SessionRecording"
os.environ["SO101_CAPTURES_ROOT"] = str(Scratch)
# The surfaces declared for the real arm (DeclaredSurfaces.json) are no part of these scenes.
os.environ["SO101_DECLARED_SURFACES"] = str(Scratch.parent / "NoDeclaredSurfaces.json")

import CameraDrift  # noqa: E402
import object_locator  # noqa: E402
import ReviewRecording  # noqa: E402
import SessionLog  # noqa: E402
import SessionRecorder  # noqa: E402

Width, Height = 320, 240


def Gray(Epoch, Seq):
    """The level a frame is drawn with: distinct per frame, and per bridge run."""
    return int(40 + Seq * 3) if Epoch == 0 else int(220 - Seq * 3)


class FakeBridge:
    """Frames at 25 fps; after RestartAt frames the "bridge restarts" and counts from 1 again. Bad: {frame number:
    "short" (the body cut off halfway, as from a bridge stopped mid-answer) or "noseq" (no X-Frame-Seq)}, once each."""

    def __init__(Self, RestartAt=25, Total=55, Bad=None):
        Self.Seq, Self.Epoch, Self.Produced = 0, 0, 0
        Self.RestartAt, Self.Total = RestartAt, Total
        Self.Bad, Self.BadSent = dict(Bad or {}), []
        Self.Condition = threading.Condition()
        Self.Jpeg = b""
        Outer = Self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(Handler_):  # noqa: N802
                Url = urlparse(Handler_.path)
                if Url.path == "/depthcam/status":
                    Body = json.dumps({"ok": True, "available": True, "seq": Outer.Seq}).encode()
                    Handler_.send_response(200)
                    Handler_.send_header("Content-Type", "application/json")
                    Handler_.end_headers()
                    Handler_.wfile.write(Body)
                    return
                After = int(parse_qs(Url.query).get("after", ["0"])[0])
                with Outer.Condition:
                    Outer.Condition.wait_for(lambda: Outer.Seq > After, timeout=0.2)
                    Seq, Jpeg = Outer.Seq, Outer.Jpeg
                if Seq <= After or not Jpeg:
                    Handler_.send_response(204)
                    Handler_.end_headers()
                    return
                Kind = Outer.Bad.pop(Seq, None)
                if Kind:
                    Outer.BadSent.append((Seq, Kind))
                    Handler_.send_response(200)
                    Handler_.send_header("Content-Type", "image/jpeg")
                    if Kind == "short":
                        Handler_.send_header("X-Frame-Seq", str(Seq))
                    Handler_.send_header("Content-Length", str(len(Jpeg)))
                    Handler_.end_headers()
                    Handler_.wfile.write(Jpeg[: len(Jpeg) // 2] if Kind == "short" else Jpeg)
                    Handler_.wfile.flush()
                    if Kind == "short":
                        Handler_.close_connection = True
                        Handler_.connection.shutdown(socket.SHUT_WR)
                    return
                Handler_.send_response(200)
                Handler_.send_header("Content-Type", "image/jpeg")
                Handler_.send_header("X-Frame-Seq", str(Seq))
                Handler_.send_header("Content-Length", str(len(Jpeg)))
                Handler_.end_headers()
                Handler_.wfile.write(Jpeg)

            def log_message(Handler_, *Args):
                pass

        Self.Server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        Self.Url = f"http://127.0.0.1:{Self.Server.server_address[1]}"
        threading.Thread(target=Self.Server.serve_forever, daemon=True).start()
        Self.Thread = threading.Thread(target=Self.Produce, daemon=True)

    def Produce(Self):
        while Self.Produced < Self.Total:
            with Self.Condition:
                if Self.Epoch == 0 and Self.Seq >= Self.RestartAt:
                    Self.Epoch, Self.Seq = 1, 0
                Self.Seq += 1
                Frame = np.full((Height, Width, 3), Gray(Self.Epoch, Self.Seq), np.uint8)
                Self.Jpeg = cv2.imencode(".jpg", Frame, [cv2.IMWRITE_JPEG_QUALITY, 95])[1].tobytes()
                Self.Produced += 1
                Self.Condition.notify_all()
            time.sleep(0.04)

    def Close(Self):
        Self.Server.shutdown()


def Epochs(Rows):
    """Bridge run of each index row: a drop in the frame number means the bridge restarted."""
    Epoch, Previous, Result = 0, 0, []
    for Row in Rows:
        if Row["seq"] < Previous:
            Epoch += 1
        Previous = Row["seq"]
        Result.append(Epoch)
    return Result


class RecordingTests(unittest.TestCase):
    @classmethod
    def setUpClass(Class):
        shutil.rmtree(Scratch, ignore_errors=True)
        Bridge = FakeBridge()
        Class.Recorder = SessionRecorder.Recorder(Bridge.Url, Scratch, SegmentLength=0.8)
        Worker = threading.Thread(target=Class.Recorder.Run, daemon=True)
        Worker.start()
        Bridge.Thread.start()
        Bridge.Thread.join(timeout=20)
        time.sleep(0.8)
        Class.Recorder.Stop()
        Worker.join(timeout=10)
        Bridge.Close()
        Class.Date = time.strftime("%Y-%m-%d")
        Class.Segments = ReviewRecording.VideoSegments(Class.Date)

    def TestSegmentsRotateAndEveryFrameDecodes(Self):
        Self.assertGreaterEqual(len(Self.Segments), 2)
        Rows = [Row for Segment in Self.Segments for Row in Segment["frames"]]
        Self.assertGreaterEqual(len(Rows), 40, "the recorder dropped most frames")
        Self.assertEqual([Row["t"] for Row in Rows], sorted(Row["t"] for Row in Rows))
        Self.assertEqual(Epochs(Rows)[-1], 1, "recording did not resume after the bridge restarted")
        Offset = 0
        for Segment in Self.Segments:
            with av.open(str(Segment["video"])) as Container:
                Decoded = [Frame.to_ndarray(format="bgr24") for Frame in Container.decode(Container.streams.video[0])]
            Self.assertEqual(len(Decoded), len(Segment["frames"]))
            Levels = Epochs(Rows)[Offset:Offset + len(Decoded)]
            for Image, Row, Epoch in zip(Decoded, Segment["frames"], Levels):
                Self.assertLess(abs(float(Image.mean()) - Gray(Epoch, Row["seq"])), 4.0)
            Offset += len(Decoded)

    def TestFrameLookupByWallClock(Self):
        Rows = [Row for Segment in Self.Segments for Row in Segment["frames"]]
        Epoch = Epochs(Rows)
        Pick = len(Rows) * 2 // 3
        [(_, Image)] = ReviewRecording.FramesAt(Self.Date, [Rows[Pick]["t"] + 0.001])
        Self.assertIsNotNone(Image)
        Self.assertLess(abs(float(Image.mean()) - Gray(Epoch[Pick], Rows[Pick]["seq"])), 4.0)
        [(_, Missing)] = ReviewRecording.FramesAt(Self.Date, [Rows[-1]["t"] + 60])
        Self.assertIsNone(Missing)

    def TestSheetAndFrameCommands(Self):
        Start, End = Self.Segments[0]["start"], Self.Segments[-1]["end"]
        Clock = lambda Stamp: time.strftime("%H:%M:%S", time.localtime(Stamp)) + f".{int(Stamp % 1 * 1000):03d}"
        Out = Scratch / "sheet.jpg"
        Self.assertEqual(ReviewRecording.Main(["sheet", "--from", Clock(Start), "--to", Clock(End), "--every", "0.5",
                                               "--columns", "3", "--out", str(Out)]), 0)
        Sheet = cv2.imread(str(Out))
        Self.assertEqual(Sheet.shape[1], 3 * 480)
        Frame = Scratch / "frame.jpg"
        Self.assertEqual(ReviewRecording.Main(["frame", "--at", Clock((Start + End) / 2), "--out", str(Frame)]), 0)
        Self.assertEqual(cv2.imread(str(Frame)).shape[:2], (Height, Width))

    def TestTimelineMergesEverySource(Self):
        Directory = SessionLog.RecordingDirectory()
        Image = "data:image/png;base64," + base64.b64encode(cv2.imencode(".png", np.zeros((4, 4, 3), np.uint8))[1]).decode()
        with open(Directory / "ai.jsonl", "a", encoding="utf-8") as File:
            Now = time.time()
            for Offset, Record in enumerate([
                    {"kind": "app", "type": "prompt", "prompt": "Grab the eraser", "task_step": 1, "model": "qwen"},
                    {"kind": "model", "duration_ms": 500, "request": {"model": "qwen", "messages": [
                        {"role": "user", "content": [{"type": "text", "text": "Grab the eraser"},
                                                     {"type": "image_url", "image_url": {"url": {"image_file": "images/a.jpg"}}}]}]},
                     "reply": {"choices": [{"message": {"content": "{\"response_to_user\": \"On it\"}"}}]}},
                    {"kind": "app", "type": "approve", "step": 1, "motion": {"target_cm": {"x": 1.0, "y": 20.0, "z": 9.0},
                                                                             "wrist_pitch_degrees": -30.0, "servo_targets": []}},
                    # A chained motion (the app's lib/AiChain.ts): both legs are recorded, in order.
                    {"kind": "app", "type": "approve", "step": 2, "motion": {"target_cm": {"x": 6.9, "y": 31.7, "z": 4.6},
                     "wrist_pitch_degrees": -49.7, "servo_targets": [],
                     "chain": {"order": "before", "gripper_raw": 1922, "gripper_fraction": 0.8}}},
                    {"kind": "app", "type": "chain_leg", "leg": 1, "of": 2, "event_kind": "gripper", "step": 2, "result": "sent",
                     "gripper_raw": 1922, "target_cm": None},
                    {"kind": "app", "type": "chain_leg", "leg": 2, "of": 2, "event_kind": "tip", "step": 2, "result": "sent",
                     "gripper_raw": None, "target_cm": {"x": 6.9, "y": 31.7, "z": 4.6}},
                    {"kind": "app", "type": "chain_aborted", "reason": "Stop task was pressed.", "source": "stop_task", "part": 1,
                     "dropped": "tip"}]):
                File.write(json.dumps({**Record, "t": Now + Offset}) + "\n")
        SessionLog.Append("grounding.jsonl", {"kind": "grounding", "model": "qwen", "image": Image, "image_size": [4, 4],
                                              "result": [{"label": "eraser"}], "reply": "[]"})
        # The recorder's own record (SessionRecorder.EventLog): the task it was told of, and its recordings.
        Write = SessionRecorder.EventLog(SessionLog.CapturesRoot())
        Write({"t": Now, "kind": "task_start", "task_id": "k1", "task": "Grab the eraser", "reason": ""})
        Write({"t": Now + 0.1, "kind": "recording_start", "camera": "depthcam", "pre_roll_s": 5.0, "pre_roll_frames": 75})
        Write({"t": Now + 9, "kind": "task_lost", "task_id": "k1", "task": "Grab the eraser", "reason": "no word from its page for 180 s"})
        Write({"t": Now + 9.1, "kind": "recording_stop", "camera": "depthcam", "reason": "its task's recording ended",
               "segment": "101500_123_depthcam.mp4"})
        Kinds = [Kind for _, Kind, _ in ReviewRecording.TimelineEntries(Self.Date, Full=False)]
        for Kind in ("PROMPT", "MODEL ASK", "MODEL SAID", "APPROVE", "GROUNDING", "VIDEO", "RECORDER"):
            Self.assertIn(Kind, Kinds)
        Recorder = [Line for _, Kind, Line in ReviewRecording.TimelineEntries(Self.Date, Full=False) if Kind == "RECORDER"]
        Self.assertEqual(Recorder, ['task_start: "Grab the eraser"',
                                    "depthcam recording starts, with the 5.0 s before it (75 frames kept in memory)",
                                    'task_lost: "Grab the eraser" (no word from its page for 180 s)',
                                    "depthcam recording stops: its task's recording ended (101500_123_depthcam.mp4)"])
        Text = "\n".join(Line for _, _, Line in ReviewRecording.TimelineEntries(Self.Date, Full=False))
        Self.assertIn("images ['images/a.jpg']", Text)
        Self.assertIn("target X 1.0 Y 20.0 Z 9.0", Text)
        # A chained motion: the approval names its order, and each leg has its own line.
        Self.assertIn("chained: gripper→1922 before the tip move", Text)
        Self.assertIn("leg 1 of 2, gripper→1922: sent (task step 2)", Text)
        Self.assertIn("leg 2 of 2, tip to X 6.9 Y 31.7 Z 4.6: sent (task step 2)", Text)
        Self.assertIn("stop_task: Stop task was pressed. (waiting on part 1; the tip leg was not sent)", Text)
        Self.assertIn("CHAIN_LEG", Kinds)


class LogTests(unittest.TestCase):
    def TestImagesStoredOnceAndReferenced(Self):
        Directory = Scratch / "store"
        shutil.rmtree(Directory, ignore_errors=True)
        Url = "data:image/jpeg;base64," + base64.b64encode(b"not really a jpeg").decode()
        First = SessionLog.StoreImages({"a": [Url, {"b": Url}], "c": "plain"}, Directory)
        Self.assertEqual(First["a"][0], First["a"][1]["b"])
        Self.assertEqual(First["c"], "plain")
        Self.assertTrue(First["a"][0]["image_file"].endswith(".jpg"))
        Self.assertEqual(len(list((Directory / "images").iterdir())), 1)
        Self.assertEqual((Directory / First["a"][0]["image_file"]).read_bytes(), b"not really a jpeg")

    def TestAppendNeverRaises(Self):
        Blocker = Scratch / "blocker"
        Blocker.parent.mkdir(parents=True, exist_ok=True)
        Blocker.write_text("a file where a folder should be")
        with patch.dict(os.environ, {"SO101_CAPTURES_ROOT": str(Blocker)}):
            SessionLog.Append("grounding.jsonl", {"kind": "grounding"})  # must not raise

    def TestGroundingCallsAreRecorded(Self):
        # Ollama's native /api/chat reply. The record keeps the data-URL image (stored as a file, not the
        # request's bare base64), the reply text, and Ollama's token and load numbers.
        Reply = {"model": "qwen3.6:latest", "message": {"role": "assistant", "content": '[{"label": "eraser", "box": [100, 200, 300, 400]}]'},
                 "done": True, "done_reason": "stop", "prompt_eval_count": 1103, "eval_count": 31, "load_duration": 16_360_000_000}
        Image = "data:image/png;base64," + base64.b64encode(cv2.imencode(".png", np.full((8, 8, 3), 9, np.uint8))[1]).decode()
        Log = SessionLog.RecordingDirectory() / "grounding.jsonl"
        Before = Log.read_text(encoding="utf-8").count("\n") if Log.exists() else 0
        with patch.object(object_locator, "_http_json", return_value=Reply):
            Result = object_locator._detect_once(Image, ["eraser"], "qwen3.6:latest", 640, 480, 0)
        Self.assertEqual(Result[0]["label"], "eraser")
        Lines = Log.read_text(encoding="utf-8").splitlines()
        Self.assertEqual(len(Lines), Before + 1)
        Self.assertLess(len(Lines[-1]), 2000, "the image must be stored as a file, not written inline")
        Record = json.loads(Lines[-1])
        Self.assertEqual(Record["kind"], "grounding")
        Self.assertTrue(Record["image"]["image_file"].startswith("images/"))
        Self.assertIn("eraser", Record["reply"])
        Self.assertEqual((Record["endpoint"], Record["prompt_eval_count"], Record["eval_count"], Record["load_ms"]),
                         ("/api/chat", 1103, 31, 16360))
        Self.assertTrue(str(Log).startswith(str(Scratch)), "tests must never write to the real record")


class LocateRecordTests(unittest.TestCase):
    """Each /locate keeps its raw depth and every measurement's diagnostics (2026-09-28 had neither)."""

    Camera = {"fx": 610.0, "fy": 610.0, "cx": 640.0, "cy": 360.0, "width": 1280, "height": 720, "dist": [0.0] * 5}

    def Scene(Self):
        """A camera 50 cm straight above the desk at X -10 Y 40, and a 5 cm block (X -12..-8, Y 38..42) on it."""
        Rotation = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
        Centre = np.array([-10.0, 40.0, -2.3 + 50.0])
        Model = {"type": "pinhole_depth_v1", "rvec": cv2.Rodrigues(Rotation)[0].ravel().tolist(),
                 "tvec": (-Rotation @ Centre).tolist(), "mirror_x": False, "image_size": [1280, 720], "table_z": -2.3,
                 "input_mirrored": False, "fitted_at": "2026-09-28T21:00:00", **Self.Camera}
        Depth = np.full((720, 1280), 500, np.uint16)
        Scratch.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(Scratch / "FitDepth.png"), Depth)  # the fit's archived raw depth
        Desk = dict(CameraDrift.DeskPlane(Depth, Self.Camera), file=str(Scratch / "FitDepth.png"))
        Reference, Problem = CameraDrift.BuildReference([dict(Desk, pose_index=Index) for Index in range(6)], Model,
                                                        Intrinsics=Self.Camera)
        Self.assertIsNone(Problem)
        Depth[333:387, 613:667] = 450
        return dict(Model, drift_reference=Reference), Depth

    def Locate(Self, Model, Depth, Detections, Picked="test-model"):
        Image = "data:image/jpeg;base64," + base64.b64encode(cv2.imencode(".jpg", np.zeros((720, 1280, 3), np.uint8))[1]).decode()
        with patch.object(object_locator.calibration, "camera_model", return_value=Model), \
                patch.object(object_locator.calibration, "status", return_value={"calibrated": True, "rms_cm": 0.2}), \
                patch.object(object_locator.calibration, "pixel_to_plane", return_value={"ok": False}), \
                patch.object(object_locator, "_pick_model", return_value=Picked), \
                patch.object(object_locator, "_detect_once", return_value=Detections), \
                patch.object(object_locator, "_refine_detection", return_value=None):
            return object_locator.locate_objects(Image, depth={"depth_mm": Depth, "intrinsics": Self.Camera, "mirrored": False,
                                                               "seq": 41})

    def LastLocate(Self):
        Directory = SessionLog.RecordingDirectory()
        Record = [json.loads(Line) for Line in (Directory / "grounding.jsonl").read_text(encoding="utf-8").splitlines()][-1]
        Self.assertEqual(Record["kind"], "locate")
        return Directory, Record

    def TestLocateRecordsRawDepthAndDiagnostics(Self):
        Model, Depth = Self.Scene()
        Box = (598 / 1280, 318 / 720, 682 / 1280, 402 / 720)
        Detections = [{"label": "block", "box": Box, "u": 640.0, "v": 402.0},
                      {"label": "robot arm", "box": Box, "u": 640.0, "v": 402.0}]
        Result = Self.Locate(Model, Depth, Detections)
        Self.assertTrue(Result["ok"], Result)
        [Block] = Result["objects"]
        Self.assertAlmostEqual(Block["height_cm"], 5.0, delta=0.3)
        Directory, Record = Self.LastLocate()
        Saved = cv2.imread(str(Directory / Record["depth_file"]), cv2.IMREAD_UNCHANGED)
        Self.assertEqual(Saved.dtype, np.uint16)
        Self.assertTrue(np.array_equal(Saved, Depth), "the raw depth, to the millimetre")
        Self.assertEqual(Record["depth_seq"], 41)
        Self.assertTrue(Record["camera_check"]["ok"])
        Self.assertEqual(Record["camera_check"]["seq"], 41)
        Self.assertEqual(Record["withheld"][0]["label"], "robot arm")
        [Diagnostic] = Record["diagnostics"]
        for Key in ("support_measured", "support_z_measured", "valid_fraction", "points", "support_branch", "desk_ring_offset_cm"):
            Self.assertIn(Key, Diagnostic)
        Self.assertEqual((Diagnostic["support_branch"], Diagnostic["support_measured"]), ("desk", True))
        Self.assertTrue(str(Directory).startswith(str(Scratch)), "tests must never write to the real record")

    def TestBlockedLocateIsRecordedToo(Self):
        Model, Depth = Self.Scene()
        Moved = np.full((720, 1280), 520, np.uint16)  # the desk 2 cm farther: the camera rose
        Result = Self.Locate(Model, Moved, [])
        Self.assertEqual(Result["error_code"], "camera_moved")
        Directory, Record = Self.LastLocate()
        Self.assertEqual(Record["error_code"], "camera_moved")
        Self.assertAlmostEqual(Record["camera_check"]["offset_cm"], 2.0, delta=0.05)
        Self.assertTrue((Directory / Record["depth_file"]).is_file())

    def TestLocateThatFailsAfterTheGateIsRecordedToo(Self):
        # The gate passed, then Ollama had no model, or its request failed: the capture's depth
        # and gate numbers are still kept for the review.
        Model, Depth = Self.Scene()
        for Picked, Detections, Words in ((None, [], "No Ollama model"),
                                          ("test-model", {"error": "Ollama request failed: connection refused"}, "Ollama request failed")):
            with Self.subTest(failure=Words):
                Result = Self.Locate(Model, Depth, Detections, Picked)
                Self.assertFalse(Result["ok"])
                Self.assertIn(Words, Result["error"])
                Directory, Record = Self.LastLocate()
                Self.assertFalse(Record["ok"])
                Self.assertIn(Words, Record["error"])
                Self.assertTrue(Record["camera_check"]["ok"])
                Self.assertEqual(Record["depth_seq"], 41)
                Self.assertTrue(np.array_equal(cv2.imread(str(Directory / Record["depth_file"]), cv2.IMREAD_UNCHANGED), Depth))


class RecorderGuardTests(unittest.TestCase):
    def TestLowDiskPausesWithoutDeleting(Self):
        Instance = SessionRecorder.Recorder("http://127.0.0.1:9", Scratch / "disk", SegmentLength=60)
        Jpeg = cv2.imencode(".jpg", np.zeros((Height, Width, 3), np.uint8))[1].tobytes()
        Usage = shutil.disk_usage(Scratch)
        with patch.object(SessionRecorder.shutil, "disk_usage", return_value=Usage._replace(free=1e9)):
            Instance.Handle(1, Jpeg, time.time())
        Self.assertIsNone(Instance.Segment)
        Self.assertIn("paused", Instance.Error)
        Instance.DiskCheckedAt = -SessionRecorder.DiskCheckS  # due for a fresh check
        Instance.Error = None
        Instance.Handle(2, Jpeg, time.time())
        Self.assertIsNotNone(Instance.Segment)
        Self.assertEqual(Instance.Frames, 1)
        Instance.Stop()

    def TestSegmentsStartedInTheSameMillisecondKeepTheirOwnVideo(Self):
        """Audit 2 M5: named by the start second, a second segment emptied the first's video and appended its frames
        to the first's index, so frame lookups returned the wrong picture. Three segments with one start time now."""
        Directory = Scratch / "same" / "video"
        shutil.rmtree(Directory.parent, ignore_errors=True)
        Start = time.time()
        Segments = [SessionRecorder.Segment(Directory, Start, Width, Height, 23) for _ in range(3)]
        for Number, Segment in enumerate(Segments):
            for Index in range(5 + Number):
                Segment.Write(np.full((Height, Width, 3), 40 + 60 * Number, np.uint8), Index + 1, Start + Index / 15)
            Segment.Close()
        Base = time.strftime("%H%M%S", time.localtime(Start)) + f"_{int(Start * 1000) % 1000:03d}"
        Self.assertEqual([Segment.Path.name for Segment in Segments],
                         [f"{Base}_depthcam.mp4", f"{Base}-1_depthcam.mp4", f"{Base}-2_depthcam.mp4"])
        for Number, Segment in enumerate(Segments):
            Rows = [json.loads(Line) for Line in Segment.IndexPath.read_text(encoding="utf-8").splitlines()]
            Self.assertEqual([Row["i"] for Row in Rows], list(range(5 + Number)), Segment.IndexPath.name)
            with av.open(str(Segment.Path)) as Container:
                Decoded = [Frame.to_ndarray(format="bgr24") for Frame in Container.decode(Container.streams.video[0])]
            Self.assertEqual(len(Decoded), 5 + Number, Segment.Path.name)
            Self.assertLess(abs(float(Decoded[-1].mean()) - (40 + 60 * Number)), 4.0)
        # A name already on disk (another recorder's) is never opened again: the next free one is taken.
        (Directory / f"{Base}-3_depthcam.frames.jsonl").write_text("someone else's\n", encoding="utf-8")
        Fourth = SessionRecorder.Segment(Directory, Start, Width, Height, 23)
        Fourth.Close()
        Self.assertEqual(Fourth.Path.name, f"{Base}-4_depthcam.mp4")
        Self.assertEqual((Directory / f"{Base}-3_depthcam.frames.jsonl").read_text(encoding="utf-8"), "someone else's\n")

    def TestAFrameAnswerCutShortOrWithoutItsNumberDoesNotEndTheRecorder(Self):
        # 2026-10-08 regression review: one answer whose body was shorter than its Content-Length (http.client.
        # IncompleteRead) or that had no X-Frame-Seq (int(None)) ended the recorder's loop, so the task running then
        # went unrecorded until the dev server restarted the recorder and the page's next "alive" came.
        Root = Scratch / "BadAnswers"
        shutil.rmtree(Root, ignore_errors=True)
        Bridge = FakeBridge(RestartAt=10 ** 6, Total=40, Bad={6: "short", 12: "noseq"})
        Recorder = SessionRecorder.Recorder(Bridge.Url, Root, SegmentLength=60)
        Worker = threading.Thread(target=Recorder.Run, daemon=True)
        Errors = []
        with patch.object(SessionRecorder, "RetryDelayS", 0.05):
            Worker.start()
            Bridge.Thread.start()
            while Bridge.Thread.is_alive():
                Errors.append(Recorder.Error)
                time.sleep(0.01)
            time.sleep(0.5)
            Alive = Worker.is_alive()
            Recorder.Stop()
            Worker.join(timeout=10)
        Bridge.Close()
        Self.assertEqual(sorted(Bridge.BadSent), [(6, "short"), (12, "noseq")], "both bad answers went out")
        Self.assertTrue(Alive, "the recorder kept running")
        Self.assertTrue(any(Error and "IncompleteRead" in Error for Error in Errors) or any(Error and "frame number" in Error for Error in Errors), Errors)
        Rows = [json.loads(Line) for Index in sorted((Root / time.strftime("%Y-%m-%d") / "recording" / "video").glob("*.frames.jsonl"))
                for Line in Index.read_text(encoding="utf-8").splitlines() if Line]
        Self.assertTrue(Rows and max(Row["seq"] for Row in Rows) >= 35, f"frames after both bad answers were recorded: {[Row['seq'] for Row in Rows][-5:]}")

    def TestOnlyOneRecorderServesStatus(Self):
        Instance = SessionRecorder.Recorder("http://127.0.0.1:9", Scratch)
        Server = SessionRecorder.ServeStatus(Instance, 0)
        Port = Server.server_address[1]
        try:
            with Self.assertRaises(OSError):
                SessionRecorder.ServeStatus(Instance, Port)
        finally:
            Server.shutdown()


class WebcamTests(unittest.TestCase):
    def TestWebcamRecordsBesideTheDepthCamera(Self):
        """The demo-area webcam goes into its own *_webcam segments at most Fps frames a second, is opened again
        after it drops out or is busy, and the look-back tool keeps the two cameras apart."""
        Opened = []

        def FakeWebcam(Name):
            Opened.append(Name)
            if len(Opened) == 2:
                raise OSError("Could not run graph (device already in use)")
            for Index in range(45):  # 1.5 s at the camera's 30 fps
                time.sleep(1 / 30)
                yield np.full((Height, Width, 3), 40 + Index * 3, np.uint8)

        with patch.object(SessionRecorder, "WebcamRetryS", 0.2):
            Instance = SessionRecorder.WebcamRecorder("Fake Cam", Scratch, SegmentLength=60, Fps=10, Open=FakeWebcam)
            Worker = threading.Thread(target=Instance.Run, daemon=True)
            Worker.start()
            Deadline = time.time() + 20
            while len(Opened) < 4 and time.time() < Deadline:
                time.sleep(0.05)
            Instance.Running = False
            Worker.join(timeout=10)
            Instance.CloseSegment()
        Self.assertGreaterEqual(len(Opened), 4, "the webcam was not opened again after it was busy")
        Date = time.strftime("%Y-%m-%d")
        Segments = ReviewRecording.VideoSegments(Date, "webcam")
        Self.assertGreaterEqual(len(Segments), 2)
        Self.assertTrue(all(Segment["video"].name.endswith("_webcam.mp4") for Segment in Segments))
        for Segment in Segments[:2]:
            Self.assertTrue(9 <= len(Segment["frames"]) <= 17, f"{len(Segment['frames'])} frames in 1.5 s at 10 fps")
            with av.open(str(Segment["video"])) as Container:
                Decoded = [Frame.to_ndarray(format="bgr24") for Frame in Container.decode(Container.streams.video[0])]
            Self.assertEqual(len(Decoded), len(Segment["frames"]))
        Self.assertFalse(any("_webcam" in Segment["video"].name for Segment in ReviewRecording.VideoSegments(Date)),
                         "the depth camera's lookups must not pick webcam frames")
        [(_, Image)] = ReviewRecording.FramesAt(Date, [Segments[0]["frames"][-1]["t"]], "webcam")
        Self.assertIsNotNone(Image)
        Self.assertIn("webcam", Instance.Status()["segment"] or "webcam")


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestSessionRecording` runs the Test* methods too (unittest names this hook)."""
    Prefixed = unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return unittest.TestSuite(Prefixed.loadTestsFromTestCase(Case) for Case in (RecordingTests, LogTests, LocateRecordTests, WebcamTests,
                                                                                RecorderGuardTests))


if __name__ == "__main__":
    Loader = unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Suite = unittest.TestSuite(Loader.loadTestsFromTestCase(Case) for Case in (RecordingTests, LogTests, LocateRecordTests, WebcamTests,
                                                                               RecorderGuardTests))
    Result = unittest.TextTestRunner(verbosity=2).run(Suite)
    raise SystemExit(not Result.wasSuccessful())
