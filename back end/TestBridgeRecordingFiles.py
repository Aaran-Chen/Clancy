"""The bridge's share of the recording clean-up (2026-10-08): its access log without the camera polls, the camera
SDK's log capped before the SDK opens it, and capture names that are never reused (the 7-day retention matches them).

No camera is opened (the SDK is a fake module here) and everything is written to scratch folders
(TestArtifacts/BridgeRecordingFiles, git-ignored).

Run:  python TestBridgeRecordingFiles.py
"""
import asyncio
import json
import logging
import logging.config
import os
import shutil
import sys
import time
import types
import unittest
from pathlib import Path
from unittest.mock import patch

Scratch = Path(__file__).resolve().parent / "TestArtifacts" / "BridgeRecordingFiles"
os.environ["SO101_CAPTURES_ROOT"] = str(Scratch / "captures")
os.environ["SO101_DEPTH_CAMERA"] = "0"

import bridge_server  # noqa: E402
import OrbbecCamera  # noqa: E402
import VideoRetention  # noqa: E402


def AccessRecord(Method: str, Path_: str, Status: int = 200) -> logging.LogRecord:
    """A record as uvicorn's access logger makes it."""
    return logging.LogRecord("uvicorn.access", logging.INFO, __file__, 1, '%s - "%s %s HTTP/%s" %d',
                             ("127.0.0.1:50000", Method, Path_, "1.1", Status), None)


class AccessLogTests(unittest.TestCase):
    def TestTheCameraPollsAndHealthAreLeftOut(Self):
        Filter = bridge_server.RoutinePollFilter()
        for Path_ in ("/depthcam/frame.jpg?after=1072480", "/depthcam/status", "/depthcam/overlay.png?after=3&near=25&far=150",
                      "/depthcam/depth-at?u=1&v=2", "/health"):
            for Status in (200, 204, 503):
                Self.assertFalse(Filter.filter(AccessRecord("GET", Path_, Status)), f"{Path_} {Status}")
        for Method, Path_ in (("POST", "/locate"), ("POST", "/motion"), ("GET", "/ports"), ("POST", "/captures/video?task=x"),
                              ("GET", "/selfcal/poses"), ("POST", "/depthcam/depth"), ("POST", "/health"), ("GET", "/depthcam/check")):
            Self.assertTrue(Filter.filter(AccessRecord(Method, Path_)), f"{Method} {Path_} is still logged")
        Self.assertTrue(Filter.filter(logging.LogRecord("uvicorn.access", logging.INFO, __file__, 1, "plain message", None, None)))

    def TestTheFilterOutlivesUvicornsLoggingSetup(Self):
        """`python -m uvicorn bridge_server:app` sets up logging before it imports the bridge, SandboxBridge after:
        either way the filter stays on uvicorn.access and the polls never reach its handler."""
        import uvicorn.config
        Access = logging.getLogger("uvicorn.access")
        Self.assertTrue(any(isinstance(Item, bridge_server.RoutinePollFilter) for Item in Access.filters), "added at import")
        Saved = (list(Access.handlers), Access.level, Access.propagate, list(Access.filters))
        try:
            logging.config.dictConfig(uvicorn.config.LOGGING_CONFIG)  # what uvicorn.Config does
            Self.assertTrue(any(isinstance(Item, bridge_server.RoutinePollFilter) for Item in Access.filters))
            Lines = []
            Catch = logging.Handler()
            Catch.emit = lambda Record: Lines.append(Record.getMessage())
            Access.handlers = [Catch]
            Access.setLevel(logging.INFO)
            for Method, Path_ in (("GET", "/depthcam/frame.jpg?after=9"), ("GET", "/health"), ("POST", "/locate"), ("GET", "/depthcam/status")):
                Access.info('%s - "%s %s HTTP/%s" %d', "127.0.0.1:1", Method, Path_, "1.1", 200)
            Self.assertEqual(Lines, ['127.0.0.1:1 - "POST /locate HTTP/1.1" 200'])
        finally:
            Access.handlers, Access.propagate = Saved[0], Saved[2]
            Access.setLevel(Saved[1])
            Access.filters = Saved[3]


class FakeSdk(types.ModuleType):
    """pyorbbecsdk's names OrbbecSource touches, with no device behind them."""

    def __init__(Self, Calls: list):
        super().__init__("pyorbbecsdk")
        Self.OBLogLevel = types.SimpleNamespace(ERROR="ERROR")

        class Context:
            def __init__(Inner):
                Calls.append("Context")

            @staticmethod
            def set_logger_to_console(Level):
                Calls.append("console")

            @staticmethod
            def set_logger_to_file(Level, Directory):
                Calls.append(("file", Directory))

            def query_devices(Inner):
                return types.SimpleNamespace(get_count=lambda: 0)

        Self.Context = Context


class SdkLogTests(unittest.TestCase):
    def TestTheSdkLogIsCappedOnceBeforeTheSdkOpensIt(Self):
        Folder = Scratch / "Log"
        shutil.rmtree(Folder, ignore_errors=True)
        Folder.mkdir(parents=True)
        Log = Folder / "OrbbecSDK.log.txt"
        Log.write_bytes(b"".join(f"[warn] frame metadata missing {Index}\n".encode() for Index in range(20000)))
        Calls = []

        def Rotate(Directory=Folder):
            Calls.append("rotate")
            return Original(Directory)

        Original = OrbbecCamera.RotateSdkLog
        with patch.dict(sys.modules, {"pyorbbecsdk": FakeSdk(Calls)}), patch.object(OrbbecCamera, "SdkLogDirectory", Folder), \
                patch.object(OrbbecCamera, "SdkLogCapBytes", 50_000), patch.object(OrbbecCamera, "_SdkLogRotated", False), \
                patch.object(OrbbecCamera, "RotateSdkLog", Rotate):
            for _ in range(2):  # the bridge opens the camera again every 2 s while it is unplugged
                with Self.assertRaises(RuntimeError):
                    OrbbecCamera.OrbbecSource()
            Self.assertEqual(Calls[:4], ["rotate", "console", ("file", str(Folder)), "Context"], "rotated before the SDK opens its log")
            Self.assertEqual(Calls.count("rotate"), 2)
            Self.assertTrue(OrbbecCamera._SdkLogRotated)
        Self.assertFalse(Log.exists())
        Kept = Folder / "OrbbecSDK.log.1.txt"
        Self.assertLessEqual(Kept.stat().st_size, 50_000)
        Self.assertTrue(Kept.read_bytes().endswith(b"missing 19999\n"))
        Self.assertEqual(OrbbecCamera.SdkLogCapBytes, 10_000_000)
        Self.assertEqual(OrbbecCamera.SdkLogKeep, 3)

    def TestOnlyTheFirstOpeningRotates(Self):
        Folder = Scratch / "Log2"
        shutil.rmtree(Folder, ignore_errors=True)
        Folder.mkdir(parents=True)
        (Folder / "OrbbecSDK.log.txt").write_bytes(b"x\n" * 100)
        with patch.object(OrbbecCamera, "_SdkLogRotated", False), patch.object(OrbbecCamera, "SdkLogCapBytes", 10):
            Self.assertEqual(OrbbecCamera.RotateSdkLog(Folder), "cut")
            (Folder / "OrbbecSDK.log.txt").write_bytes(b"x\n" * 100)
            Self.assertEqual(OrbbecCamera.RotateSdkLog(Folder), "done before")
        Self.assertTrue((Folder / "OrbbecSDK.log.txt").exists(), "the SDK's open log is never moved from under it")


class FakeRequest:
    """What /captures/video reads from a request."""

    def __init__(Self, Body: bytes, ContentType: str, Query: dict):
        Self.Body, Self.headers, Self.query_params = Body, {"content-type": ContentType}, Query

    async def body(Self):
        return Self.Body


class CaptureNameTests(unittest.TestCase):
    def setUp(Self):
        shutil.rmtree(Scratch / "captures", ignore_errors=True)

    def TestTwoCapturesInOneMillisecondKeepTheirOwnFiles(Self):
        Now = time.time()
        First = bridge_server.CapturePath("task", "webm", ("json",), Now)
        Second = bridge_server.CapturePath("task", "webm", ("json",), Now)
        Base = time.strftime("%H%M%S", time.localtime(Now)) + f"_{int(Now * 1000) % 1000:03d}"
        Self.assertEqual((First.name, Second.name), (f"{Base}_task.webm", f"{Base}-1_task.webm"))
        Self.assertTrue(First.with_suffix(".json").exists() and Second.with_suffix(".json").exists(), "the sidecar's name is held too")
        Self.assertTrue(str(First).startswith(str(Scratch)), "SO101_CAPTURES_ROOT is followed")
        # A sidecar left without its clip still holds its name.
        Orphan = First.parent / f"{Base}-2_task.json"
        Orphan.write_text("{}", encoding="utf-8")
        Third = bridge_server.CapturePath("task", "webm", ("json",), Now)
        Self.assertEqual(Third.name, f"{Base}-3_task.webm")
        Self.assertEqual(Orphan.read_text(encoding="utf-8"), "{}")
        Self.assertFalse((First.parent / f"{Base}-2_task.webm").exists(), "no half-made pair left behind")
        for Path_ in (First, Second, Third):
            Self.assertTrue(VideoRetention.TaskClipName.match(Path_.name), "the retention pass knows the name")
            Self.assertTrue(VideoRetention.TaskClipName.match(Path_.with_suffix(".json").name))

    def TestATaskClipAndItsSidecar(Self):
        Body = b"\x1a\x45\xdf\xa3 webm bytes"
        Reply = asyncio.run(bridge_server.save_capture_video(FakeRequest(Body, "video/webm;codecs=vp9",
                                                                         {"task": "grab the tin", "reason": "the AI reported the task done.",
                                                                          "duration_s": "31.6", "started_at": "2026-10-08T10:00:00.000Z"})))
        Self.assertTrue(Reply["ok"], Reply)
        Clip = Path(Reply["path"])
        Self.assertEqual(Clip.read_bytes(), Body)
        Meta = json.loads(Clip.with_suffix(".json").read_text(encoding="utf-8"))
        Self.assertEqual((Meta["task"], Meta["duration_s"], Meta["bytes"]), ("grab the tin", "31.6", len(Body)))
        Self.assertTrue(VideoRetention.TaskClipName.match(Clip.name))
        Again = asyncio.run(bridge_server.save_capture_video(FakeRequest(b"second", "video/webm", {})))
        Self.assertNotEqual(Again["path"], Reply["path"])
        Self.assertEqual(Clip.read_bytes(), Body, "the first clip is untouched")

    def TestAPhotoTheAiSawIsNeverOverwrittenEither(Self):
        import base64
        import cv2
        import numpy as np
        Image = "data:image/jpeg;base64," + base64.b64encode(cv2.imencode(".jpg", np.zeros((8, 8, 3), np.uint8))[1].tobytes()).decode()
        Paths = {bridge_server.save_capture({"image": Image, "kind": "vision"})["path"] for _ in range(5)}
        Self.assertEqual(len(Paths), 5)
        Self.assertTrue(all(Path(Item).name.endswith("_vision.jpg") and Path(Item).stat().st_size > 0 for Item in Paths))
        Self.assertFalse(any(VideoRetention.TaskClipName.match(Path(Item).name) for Item in Paths), "photos are not video: kept")


Cases = (AccessLogTests, SdkLogTests, CaptureNameTests)


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestBridgeRecordingFiles` runs the Test* methods too (unittest names this hook)."""
    Prefixed = unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return unittest.TestSuite(Prefixed.loadTestsFromTestCase(Case) for Case in Cases)


if __name__ == "__main__":
    Loader = unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Result = unittest.TextTestRunner(verbosity=2).run(unittest.TestSuite(Loader.loadTestsFromTestCase(Case) for Case in Cases))
    raise SystemExit(not Result.wasSuccessful())
