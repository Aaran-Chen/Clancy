"""VLM box parsing, model choice, sampling, zoom-refine and the task query in object_locator, with Ollama mocked."""
import base64
import contextlib
import json
import os
os.environ.setdefault("SO101_LOCATE_MODE", "vlm")  # this suite tests the voted multi-call locate; TestSingleCallLocate.py the default
import re
import unittest
import urllib.error
from pathlib import Path
from unittest.mock import patch

# Detections are recorded (SessionLog); keep test runs out of the real record.
os.environ["SO101_CAPTURES_ROOT"] = str(Path(__file__).resolve().parent / "TestArtifacts" / "SessionCaptures")

import object_locator as OL  # noqa: E402
import SessionLog  # noqa: E402

SharedOptions = json.loads((Path(__file__).resolve().parent / "OllamaOptions.json").read_text(encoding="utf-8"))


def Reply(Content):
    """Ollama's native /api/chat reply."""
    return {"model": "qwen3.6:latest", "message": {"role": "assistant", "content": Content}, "done": True,
            "done_reason": "stop", "prompt_eval_count": 1103, "eval_count": 40, "load_duration": 2_000_000}


def Box(Label, X1, Y1, X2, Y2):
    """A detection as the model writes it, on the 0-1000 grid."""
    return {"label": Label, "box": [X1, Y1, X2, Y2]}


# One 1280x720 frame (0-1000 grid): the eraser and cup of 2026-09-29 07:58, a pen, and the arm at the right edge.
Eraser, Cup, Pen = Box("eraser", 450, 520, 500, 576), Box("cup", 600, 534, 675, 661), Box("pen", 250, 300, 300, 330)
Arm = Box("robot arm", 850, 500, 950, 900)
Frame = "data:image/jpeg;base64," + base64.b64encode(OL.cv2.imencode(".jpg", OL.np.zeros((720, 1280, 3), OL.np.uint8))[1]).decode()


class FakeOllama:
    """Replies by prompt: the coarse samples in order, the strict task query, and zoom-refines (which find nothing)."""

    def __init__(Self, Samples, TaskReply="[]", RefineFinds=False):
        Self.Samples, Self.TaskReply, Self.RefineFinds = Samples, TaskReply, RefineFinds
        Self.Coarse, Self.Refines, Self.TaskPayloads = 0, [], []

    def __call__(Self, Url, Payload=None, timeout=10):
        Prompt = Payload["messages"][0]["content"]
        if "The user asked for" in Prompt:
            Self.TaskPayloads.append(Payload)
            if isinstance(Self.TaskReply, Exception):
                raise Self.TaskReply
            return Reply(Self.TaskReply)
        Named = re.search(r"Find each of these objects: (.*)\.", Prompt)
        if Named:
            Self.Refines.append(Named.group(1))
            # RefineFinds: the crop's middle, where the coarse box sits.
            return Reply(json.dumps([Box(Named.group(1), 400, 400, 600, 600)]) if Self.RefineFinds else "[]")
        Sample = Self.Samples[min(Self.Coarse, len(Self.Samples) - 1)]
        Self.Coarse += 1
        return Reply(json.dumps(Sample))


def Measure(DepthMm, Box_, Intrinsics, Model, TableZ, Mirrored, Surfaces=None, Exclude=None):
    """DepthGeometry.MeasureBox stand-in: a 2 cm object under the box centre; the arm fills the right fifth."""
    Cx, Cy = (Box_[0] + Box_[2]) / 2, (Box_[1] + Box_[3]) / 2
    if Cx > 0.8:
        return {"ok": False, "robot_arm": True, "error": "the box shows only the robot arm", "valid_fraction": 0.9,
                "support_measured": False, "support_z_measured": None, "support_branch": None}
    return {"ok": True, "x": round(Cx * 100 - 50, 2), "y": round(60 - Cy * 50, 2), "z": -2.3, "top_z": -0.3, "height_cm": 2.0,
            "size_cm": [3.0, 2.0], "distance_cm": 55.0, "support_measured": True, "support_z_measured": -2.3,
            "valid_fraction": 0.95, "support_branch": "desk"}


def LastRecord(Kind):
    Lines = (SessionLog.RecordingDirectory() / "grounding.jsonl").read_text(encoding="utf-8").splitlines()
    return next(Record for Record in map(json.loads, reversed(Lines)) if Record.get("kind") == Kind)


class LocatorTests(unittest.TestCase):
    def Detect(Self, Content, Width=1280, Height=720):
        with patch.object(OL, "_http_json", return_value=Reply(Content)):
            return OL._detect_once("data:image/jpeg;base64,", None, "qwen3.6:latest", Width, Height, 0)

    def TestQwenGridBoxesBelowImageHeightAreNotPixels(Self):
        # qwen3.6's reply on the live 1280x720 frame: the clock box peaks at 710,
        # under the image height, and used to be read as pixels (~180 px off).
        Detections = Self.Detect('[{"label": "digital alarm clock", "box": [632, 364, 710, 458]}]')
        Box = Detections[0]["box"]
        for Value, Expected in zip(Box, (0.632, 0.364, 0.710, 0.458)):
            Self.assertAlmostEqual(Value, Expected, places=6)
        Self.assertAlmostEqual(Detections[0]["u"], (0.632 + 0.710) / 2 * 1280, places=3)
        Self.assertAlmostEqual(Detections[0]["v"], 0.458 * 720, places=3)

    def TestFractionsAndPixelsStillRecognised(Self):
        Fractions = Self.Detect('[{"label": "cup", "box": [0.1, 0.2, 0.3, 0.4]}]')[0]["box"]
        Self.assertEqual(tuple(round(Value, 6) for Value in Fractions), (0.1, 0.2, 0.3, 0.4))
        Pixels = Self.Detect('```json\n[{"label": "cup", "bbox_2d": [1100, 200, 1200, 300]}]\n```')[0]["box"]
        Self.assertAlmostEqual(Pixels[0], 1100 / 1280)
        Self.assertAlmostEqual(Pixels[3], 300 / 720)

    def Refine(Self, Box):
        # Coarse box 600-700 x 300-400 px: the refine crop is 500-800 x 200-500 (300 x 300),
        # with the coarse box as its middle third.
        Image = OL.np.zeros((720, 1280, 3), OL.np.uint8)
        Content = json.dumps([{"label": "cup", "box": Box}])
        with patch.object(OL, "_http_json", return_value=Reply(Content)):
            return OL._refine_detection(Image, "cup", (600.0, 300.0, 700.0, 400.0), "any-model")

    def TestRefineReadsCropPixelsWhereTheObjectIs(Self):
        # A model answering in crop pixels: read as the grid, this box would sit in the crop's corner.
        U, V, _ = Self.Refine([110, 105, 190, 195])
        Self.assertAlmostEqual(U, 500 + (110 + 190) / 2, places=3)
        Self.assertAlmostEqual(V, 200 + 195, places=3)

    def TestRefineKeepsGridAnswers(Self):
        U, V, _ = Self.Refine([350, 340, 640, 660])
        Self.assertAlmostEqual(U, 500 + (0.35 + 0.64) / 2 * 300, places=3)
        Self.assertAlmostEqual(V, 200 + 0.66 * 300, places=3)

    def TestRefineThatMissesTheObjectIsDropped(Self):
        Self.assertIsNone(Self.Refine([0, 0, 100, 100]))

    def TestPromptAsksForTheQwenGrid(Self):
        Self.assertIn("0-1000 grid", OL._grounding_prompt(None))

    def TestRobotPartsAreMatchedAsWholeWords(Self):
        # 2026-09-28: "robot arm" and "white robotic arm" were measured as objects in 6 of 8 locates.
        for Label in ("robot arm", "White Robotic Arm", "gripper", "claws", "servo motor", "cart", "depth camera",
                      "black bin", "wire basket", "arms"):
            Self.assertIsNotNone(OL._RobotPartReason(Label), Label)
        Self.assertIn("bin", OL._RobotPartReason("black bin"))
        for Label in ("binder", "alarm clock", "armchair", "ink cartridge", "eraser", "cup", "cabinet", "garment"):
            Self.assertIsNone(OL._RobotPartReason(Label), Label)
        # A task that asks for a servo means a loose one (2026-09-30 05:24): only then is "servo" not the robot's.
        Self.assertIsNone(OL._RobotPartReason("servo motor", "black servo motor thing"))
        Self.assertIsNone(OL._RobotPartReason("black servos", "black servo thing"))
        Self.assertIsNotNone(OL._RobotPartReason("servo motor", "eraser"))
        Self.assertIsNotNone(OL._RobotPartReason("robot arm servo", "black servo thing"), "the arm is still the robot")
        Self.assertIsNotNone(OL._RobotPartReason("gripper", "gripper servo"))

    def TestStandInLabelsMayBeTheTaskObjectUnderAnotherName(Self):
        # The labels the detector gave the "black servo motor thing" on 2026-09-30 05:24-05:31.
        Task = "black servo motor thing"
        for Label in ("small black object", "black object on tape", "small black box", "black", "small object", "object on tape"):
            Self.assertTrue(OL._StandIn(Label, Task), Label)
        for Label in ("usb drive", "battery", "tape roll", "roll of tape", "pencil sharpener", "white object", "tape"):
            Self.assertFalse(OL._StandIn(Label, Task), Label)
        Self.assertTrue(OL._StandIn("white object", "stapler"), "no colour asked: a generic label may be it")

    def TestImplausibleMeasurementsAreWithheld(Self):
        Plausible = {"x": 5.4, "y": 33.0, "height_cm": 1.1, "size_cm": [5.0, 2.0], "distance_cm": 56.0}
        Self.assertIsNone(OL._ImplausibleReason(Plausible))
        for Change, Words in (({"distance_cm": 553.3}, "553 cm"), ({"x": 194.97}, "outside the workspace"),
                              ({"y": -8.0}, "outside the workspace"), ({"y": 206.75}, "outside the workspace"),
                              ({"height_cm": 28.3}, "28.3 cm tall"), ({"height_cm": 0.2}, "0.2 cm tall"),
                              ({"size_cm": [41.0, 3.0]}, "41 cm across")):
            Reason = OL._ImplausibleReason({**Plausible, **Change})
            Self.assertIsNotNone(Reason, Change)
            Self.assertIn(Words, Reason)
            Self.assertTrue(Reason.startswith("not measurable"))

    def TestPickModelSkipsModelsThatCannotSee(Self):
        Capabilities = {"nomic-embed-text:latest": ["embedding"], "qwen2.5:7b": ["completion", "tools"],
                        "qwen3.6:latest": ["completion", "vision", "tools"], "small-vision:1b": ["completion", "vision"]}
        Installed = [{"name": "nomic-embed-text:latest", "size": 3e8}, {"name": "qwen2.5:7b", "size": 4e9},
                     {"name": "small-vision:1b", "size": 1e9}, {"name": "qwen3.6:latest", "size": 2.2e10}]

        def Fake(Url, Payload=None, timeout=10):
            if Url.endswith("/api/ps"):
                return {"models": [{"name": "qwen2.5:7b", "size": 4e9}]}  # a text model is loaded
            if Url.endswith("/api/tags"):
                return {"models": Installed}
            if Url.endswith("/api/show"):
                return {"capabilities": Capabilities[Payload["model"]]}
            raise AssertionError(Url)

        OL._VISION_CACHE.clear()
        with patch.object(OL, "_http_json", side_effect=Fake):
            Self.assertEqual(OL._pick_model(None), "qwen3.6:latest")
            Self.assertEqual(OL._pick_model("chosen:1b"), "chosen:1b")
        OL._VISION_CACHE.clear()

    def TestModelCallsUseNativeChatWithTheSharedOptions(Self):
        # /v1/chat/completions ignored num_ctx, so the model loaded at 262144 tokens with under 1 GiB of VRAM
        # free. The app and the bridge must send the same num_ctx and num_batch, or Ollama reloads (8-15 s).
        Sent = []

        def Fake(Url, Payload=None, timeout=10):
            Sent.append((Url, Payload))
            return Reply('[{"label": "cup", "box": [100, 200, 300, 400]}]')

        with patch.object(OL, "_http_json", side_effect=Fake):
            [Detected] = OL._detect_once("data:image/jpeg;base64,QUJD", None, "qwen3.6:latest", 1280, 720, 0.5)
        Self.assertEqual(Detected["label"], "cup")
        [(Url, Payload)] = Sent
        Self.assertEqual(Url, f"{OL.OLLAMA_BASE}/api/chat")
        Self.assertEqual((Payload["model"], Payload["stream"], Payload["think"], Payload["keep_alive"]),
                         ("qwen3.6:latest", False, False, SharedOptions["keep_alive"]))
        Options = Payload["options"]
        Self.assertEqual((Options["num_ctx"], Options["num_batch"], Options["temperature"]),
                         (SharedOptions["num_ctx"], SharedOptions["num_batch"], 0.5))
        # One message per content part, the prompt first, as /v1 split them (1103 prompt tokens either way);
        # the image goes as bare base64.
        Self.assertEqual(Payload["messages"], [{"role": "user", "content": OL._grounding_prompt(None)},
                                               {"role": "user", "content": "", "images": ["QUJD"]}])
        Record = LastRecord("grounding")
        Self.assertEqual((Record["prompt_eval_count"], Record["eval_count"], Record["done_reason"]), (1103, 40, "stop"))

    def TestStrictTaskQueryPromptAllowsEmptyReply(Self):
        # The "Find each of these objects" wording boxed something for 72 of 72 absent objects.
        Prompt = OL._task_query_prompt("black thing on the desk")
        Self.assertIn("The user asked for: black thing on the desk.", Prompt)
        Self.assertIn("may NOT contain it", Prompt)
        Self.assertIn("[] when it is not in the image", Prompt)
        Self.assertIn("0-1000 grid", Prompt)
        Self.assertNotIn("Find each of these objects", Prompt)
        Self.assertEqual(Self.Detect("[]"), [])


class LocateTests(unittest.TestCase):
    """locate_objects end to end: Ollama answers by prompt, depth and the camera gate are stand-ins."""

    def Locate(Self, Fake, UseDepth=True, MeasureWith=Measure, **Options):
        """With UseDepth False the capture has no depth frame: objects land on the table plane (X = u / 10)."""
        Depth = {"depth_mm": OL.np.zeros((720, 1280), OL.np.uint16), "intrinsics": {}, "mirrored": False, "seq": 7}
        Plane = {"return_value": {"ok": False}} if UseDepth else \
            {"side_effect": lambda U, V, Mirrored=None: {"ok": True, "x": U / 10, "y": V / 10, "z": -2.3}}
        with contextlib.ExitStack() as Stack:
            for Target, Name, Value in ((OL, "_http_json", {"side_effect": Fake}),
                                        (OL, "_pick_model", {"return_value": "qwen3.6:latest"}),
                                        (OL.calibration, "camera_model", {"return_value": {"table_z": -2.3}}),
                                        (OL.calibration, "status", {"return_value": {"calibrated": True, "rms_cm": 0.2}}),
                                        (OL.calibration, "pixel_to_plane", Plane),
                                        (OL.CameraDrift, "CheckCapture", {"return_value": {"ok": True, "angle_deg": 0.0, "offset_cm": 0.0}}),
                                        (OL.CameraDrift, "DisplayDeskPlane", {"return_value": None}),
                                        (OL.DepthGeometry, "ModelUsesDepthCamera", {"return_value": True}),
                                        (OL.DepthGeometry, "WorkspaceSurfaces", {"return_value": None}),
                                        (OL.DepthGeometry, "ArmExclusion", {"return_value": lambda Points: None}),
                                        (OL.DepthGeometry, "MeasureBox", {"side_effect": MeasureWith})):
                Stack.enter_context(patch.object(Target, Name, **Value))
            Result = OL.locate_objects(Frame, depth=Depth if UseDepth else None, **Options)
        Self.assertTrue(Result["ok"], Result)
        return Result

    def Labels(Self, Result, Key=None):
        return [Object["label"] for Object in Result["objects"] if Key is None or Object[Key]]

    # --- Early exit ------------------------------------------------------------------

    def TestEarlyExitComparesBoxPositionsNotLabels(Self):
        # The temperature-0.5 sample renames objects ("rubber eraser"), so the identical label sets the old
        # exit needed never happened (0 of 21 locates on 09-28/29): every locate paid for a third sample.
        Renamed = [Box("rubber eraser", 452, 518, 503, 579), Box("paper cup", 603, 530, 678, 664)]
        Fake = FakeOllama([[Eraser, Cup, Arm], Renamed, [Eraser, Cup]])
        Result = Self.Locate(Fake)
        Self.assertEqual((Fake.Coarse, Result["samples_run"]), (2, 2))

    def TestThirdSampleRunsWhenTheSamplesDisagree(Self):
        for Case, Second in (("a new object", [Eraser, Cup, Pen]), ("a box 51 px away", [Eraser, Box("cup", 640, 534, 715, 661)]),
                             ("an object missing", [Eraser]), ("only the arm", [Arm])):
            with Self.subTest(Case):
                First = [Arm] if Case == "only the arm" else [Eraser, Cup]
                Fake = FakeOllama([First, Second, First])
                Self.assertEqual(Self.Locate(Fake)["samples_run"], 3)
                Self.assertEqual(Fake.Coarse, 3)

    # --- Zoom-refine -----------------------------------------------------------------

    def TestZoomRefineRunsOnlyForTheTaskObject(Self):
        # 94% of the refine time on 09-28/29 went to objects the task never used (8.1 s per locate).
        Scene = [Box("red eraser", 450, 520, 500, 576), Cup, Pen]
        Fake = FakeOllama([Scene, Scene], TaskReply=json.dumps([Box("eraser", 452, 522, 498, 574)]))
        Result = Self.Locate(Fake, task_object={"phrase": "eraser", "label": "eraser"})
        Self.assertEqual(Fake.Refines, ["red eraser"])
        Self.assertEqual(sorted(Self.Labels(Result)), ["cup", "pen", "red eraser"])
        # The query alone can pick the group: the detector called the eraser "rubber" (09-28 20:09).
        Scene = [Box("rubber", 450, 520, 500, 576), Cup, Pen]
        Fake = FakeOllama([Scene, Scene], TaskReply=json.dumps([Box("eraser", 452, 522, 498, 574)]))
        Self.Locate(Fake, task_object={"phrase": "eraser", "label": "eraser"})
        Self.assertEqual(Fake.Refines, ["rubber"])

    def TestCoarseBoxesThatDisagreeAreStillRefined(Self):
        # A spread over SAMPLE_UNSTABLE_PX is what the refine exists to settle ([UNRELIABLE] without it).
        Samples = [[Eraser, Cup], [Eraser, Box("cup", 660, 534, 735, 661)], [Eraser, Box("cup", 540, 534, 615, 661)]]
        Fake = FakeOllama(Samples, TaskReply=json.dumps([Eraser]))
        Result = Self.Locate(Fake, task_object={"phrase": "eraser", "label": "eraser"})
        Self.assertEqual(sorted(Fake.Refines), ["cup", "eraser"])
        Self.assertEqual([Object["unstable"] for Object in Result["objects"] if Object["label"] == "cup"], [True])

    def TestEveryObjectIsRefinedWhenTheTaskObjectCannotBeLookedFor(Self):
        # No task object ("describe the objects"), or the query failed and no label names it: refine all, as before.
        Stapler = {"task_object": {"phrase": "stapler", "label": "stapler"}}
        for Case, Reply_, Options in (("no task object", "[]", {}),
                                      ("the query failed", urllib.error.URLError("connection reset"), Stapler)):
            with Self.subTest(Case):
                Fake = FakeOllama([[Eraser, Cup, Pen]] * 2, TaskReply=Reply_)
                Self.Locate(Fake, **Options)
                Self.assertEqual(sorted(Fake.Refines), ["cup", "eraser", "pen"])

    def TestAnAbsentTaskObjectRefinesNothingElse(Self):
        # The query answered [] and no label names it: 7 of the 24 recorded locates, where refining every
        # other object still cost 1.8-12.3 s.
        Fake = FakeOllama([[Eraser, Cup, Pen]] * 2, TaskReply="[]")
        Result = Self.Locate(Fake, task_object={"phrase": "stapler", "label": "stapler"})
        Self.assertEqual(Fake.Refines, [])
        Self.assertEqual([Object["refined"] for Object in Result["objects"]], [False, False, False])

    def TestStandInsAreRefinedWhenNothingNamesTheTaskObject(Self):
        # 2026-09-30 05:24:53, "grab the black servo motor thing": the three recorded samples (the second called the
        # servo "camera", the third "small black object") and the strict query's []. Only the unstable bottle and cup
        # were refined, so the servo reached the model as [UNRELIABLE: coarse box, not refined] though its depth was
        # 96% valid. The labels that may be it under another name are refined too; "white object" clashes with black.
        Samples = [[Box("can", 301, 328, 394, 576), Box("bottle", 380, 481, 460, 661), Box("tape", 439, 514, 521, 661),
                    Box("keyboard", 447, 186, 537, 382), Box("laptop", 280, 22, 389, 236), Box("monitor", 89, 0, 194, 147),
                    Box("mouse", 378, 15, 428, 65), Box("cup", 592, 514, 671, 647), Box("pen", 379, 275, 399, 333),
                    Box("white object", 220, 36, 271, 108)],
                   [Box("can", 301, 329, 385, 567), Box("bottle", 381, 482, 462, 662), Box("tape roll", 440, 525, 523, 661),
                    Box("camera", 473, 513, 518, 581), Box("cup", 594, 513, 680, 653), Box("keyboard", 420, 189, 537, 382),
                    Box("laptop", 283, 26, 389, 232), Box("mouse", 378, 22, 429, 65), Box("monitor", 90, 0, 198, 144),
                    Box("white object", 220, 36, 271, 101), Box("pen", 380, 275, 399, 332), Box("pliers", 423, 804, 651, 984),
                    Box("robot arm", 648, 539, 825, 971)],
                   [Box("cup", 301, 328, 365, 547), Box("bottle", 380, 480, 431, 600), Box("tape roll", 439, 540, 520, 660),
                    Box("small black object", 472, 515, 518, 580), Box("keyboard", 445, 190, 536, 380), Box("pen", 379, 275, 399, 330),
                    Box("laptop", 278, 27, 384, 230), Box("mouse", 377, 17, 426, 65), Box("white object", 220, 35, 271, 105),
                    Box("monitor", 90, 0, 197, 145)]]
        Task = {"phrase": "black servo motor thing", "label": "black servo motor thing"}
        Fake = FakeOllama(Samples, TaskReply="[]", RefineFinds=True)
        Result = Self.Locate(Fake, task_object=Task)
        Self.assertEqual(Result["samples_run"], 3)
        Self.assertEqual(Fake.Refines, ["bottle", "cup", "small black object"])
        Refined = {Object["label"]: Object["refined"] for Object in Result["objects"]}
        Self.assertTrue(Refined["small black object"])
        Self.assertFalse(Refined["tape"] or Refined["tape roll"] or Refined["white object"])
        Self.assertEqual(Self.Labels(Result, "task_match"), [])
        Self.assertIn({"label": "camera", "reason": OL.ROBOT_PARTS_REASON}, Result["withheld"])
        Notes = {Item["label"]: Item["refine"] for Item in LastRecord("locate")["diagnostics"]}
        Self.assertEqual(Notes["small black object"], "stand-in")
        # Without a colour every label naming no kind of thing may be it; the named kinds keep their coarse boxes.
        Fake = FakeOllama(Samples, TaskReply="[]")
        Self.Locate(Fake, task_object={"phrase": "stapler", "label": "stapler"})
        Self.assertEqual(Fake.Refines, ["bottle", "cup", "white object", "small black object"])

    def TestALooseServoTheTaskAsksForIsMeasured(Self):
        # "servo" names the arm's own parts, so a detection or a query box called a servo was always withheld, even
        # when the task asked for the servo lying on the desk.
        Scene = [Eraser, Box("black servo", 300, 450, 340, 480)]
        Task = {"phrase": "black servo motor thing", "label": "black servo motor thing"}
        Result = Self.Locate(FakeOllama([Scene] * 2, TaskReply="[]"), task_object=Task)
        Self.assertIn("black servo", Self.Labels(Result))
        Result = Self.Locate(FakeOllama([Scene] * 2, TaskReply="[]"), task_object={"phrase": "eraser", "label": "eraser"})
        Self.assertNotIn("black servo", Self.Labels(Result))
        Self.assertIn({"label": "black servo", "reason": OL.ROBOT_PARTS_REASON}, Result["withheld"])
        # The query's own box on no sampled object is measured under the task's label.
        Fake = FakeOllama([[Eraser, Cup]] * 2, TaskReply=json.dumps([Box("servo motor", 200, 600, 240, 640)]))
        Result = Self.Locate(Fake, task_object=Task)
        Self.assertEqual(Self.Labels(Result, "task_query_only"), ["black servo motor thing"])
        # The arm itself is still the robot's, whatever the task asks for.
        Result = Self.Locate(FakeOllama([[Eraser, Box("robot arm servo", 300, 450, 340, 480)]] * 2), task_object=Task)
        Self.assertNotIn("robot arm servo", Self.Labels(Result))

    # --- The strict task query -------------------------------------------------------

    def TestTaskQueryFlagsTheObjectItLandsOn(Self):
        # 16:05-16:10 "Grab the black thingon the desk": the detector said "USB drive", then "small black object".
        Scene = [Box("usb drive", 300, 450, 340, 480), Box("box", 600, 534, 675, 661), Pen]
        Fake = FakeOllama([Scene, Scene], TaskReply=json.dumps([Box("black thing", 298, 452, 338, 482)]))
        Task = {"phrase": "black thing on the desk", "label": "black thing"}
        Result = Self.Locate(Fake, task_object=Task)
        Self.assertEqual(Self.Labels(Result, "task_match"), ["usb drive"])
        Self.assertEqual(Self.Labels(Result, "task_query_only"), [])
        [Query] = Fake.TaskPayloads
        Self.assertIn("The user asked for: black thing on the desk.", Query["messages"][0]["content"])
        Self.assertEqual(Query["options"]["temperature"], 0)
        Self.assertEqual(Fake.Refines, ["usb drive"])  # "thing" names nothing: the query picked the group
        Record = LastRecord("task_query")
        Self.assertEqual(Record["task_object"], Task)
        Self.assertEqual(Record["result"][0]["label"], "black thing")
        Self.assertEqual(LastRecord("locate")["task_object"], Task)

    def TestTaskQueryEmptyReplyFlagsNothing(Self):
        Fake = FakeOllama([[Eraser, Cup]] * 2, TaskReply="[]")
        Result = Self.Locate(Fake, task_object={"phrase": "usb drive", "label": "usb drive"})
        Self.assertEqual(len(Fake.TaskPayloads), 1)
        Self.assertEqual(sorted(Self.Labels(Result)), ["cup", "eraser"])
        Self.assertEqual(Self.Labels(Result, "task_match") + Self.Labels(Result, "task_query_only"), [])

    def TestTaskQueryFailureKeepsTheLocate(Self):
        Fake = FakeOllama([[Eraser, Cup]] * 2, TaskReply=urllib.error.URLError("connection reset"))
        Result = Self.Locate(Fake, task_object={"phrase": "eraser", "label": "eraser"})
        Self.assertEqual(sorted(Self.Labels(Result)), ["cup", "eraser"])
        Self.assertEqual(Self.Labels(Result, "task_match"), [])
        Self.assertEqual(Fake.Refines, ["eraser"])  # still named by the task

    def TestTaskQueryOrphanIsMeasuredAndTagged(Self):
        # 23:09:57 UTC: only 1 of 3 samples named the USB drive. A query box on no group is measured itself;
        # the same spot answered twice is one object.
        Fake = FakeOllama([[Eraser, Cup]] * 2, TaskReply=json.dumps([Box("usb drive", 200, 600, 240, 640),
                                                                     Box("usb stick", 201, 601, 241, 641)]))
        Result = Self.Locate(Fake, task_object={"phrase": "black thing on the desk", "label": "task object"})
        [Orphan] = [Object for Object in Result["objects"] if Object["task_query_only"]]
        Self.assertEqual((Orphan["label"], Orphan["task_match"], Orphan["method"], Orphan["samples"]),
                         ("task object", True, "depth", 0))
        Self.assertAlmostEqual(Orphan["x"], 0.22 * 100 - 50, places=1)
        Self.assertEqual(Self.Labels(Result, "task_match"), ["task object"])
        # Refined under the query's own name ("task object" names nothing in a crop); the query found it,
        # so the rest keep their first-pass boxes.
        Self.assertEqual(Fake.Refines, ["usb drive"])

    def TestTaskQueryBoxOnTheArmIsWithheld(Self):
        Fake = FakeOllama([[Eraser, Cup]] * 2, TaskReply=json.dumps([Box("eraser", 850, 520, 900, 576)]))
        Result = Self.Locate(Fake, task_object={"phrase": "red eraser", "label": "eraser"})
        Self.assertEqual(Self.Labels(Result, "task_query_only"), [])
        Self.assertIn({"label": "eraser", "reason": "the robot's own arm fills the box", "task_query_only": True},
                      Result["withheld"])

    def TestArmFractionGoesWithAMeasurementTheArmWasMaskedFrom(Self):
        # 2026-10-03 16:41:36: the "phone" box held the open gripper (arm_fraction 0.356 in the diagnostics only); the
        # web app now reads it with the object (lib/ArmMadeObjects.ts). Without the arm pose nothing was masked: no share.
        def Masked(*Arguments, **Options):
            Result = Measure(*Arguments, **Options)
            return {**Result, "arm_fraction": 0.356} if Options.get("Exclude") is not None and Result["ok"] else Result
        Pose = {"ok": True, "joints": {"shoulder_pan": 0.0}}
        Result = Self.Locate(FakeOllama([[Eraser, Cup]] * 2), MeasureWith=Masked, arm=Pose)
        Self.assertEqual([(Object["label"], Object["arm_fraction"]) for Object in Result["objects"]], [("eraser", 0.356), ("cup", 0.356)])
        Result = Self.Locate(FakeOllama([[Eraser, Cup]] * 2), MeasureWith=Masked)
        Self.assertEqual(Self.Labels(Result), ["eraser", "cup"])
        Self.assertFalse(any("arm_fraction" in Object for Object in Result["objects"]))

    def TestTaskQueryBoxOnTheArmIsWithheldWithoutTheArmPose(Self):
        # With no arm pose the depth cannot mask the arm, so a query box on a spot the samples called the
        # robot is withheld, although depth alone would measure it. With the pose, depth decides.
        Held = [Eraser, Box("robot arm", 600, 300, 780, 900)]
        Query = json.dumps([Box("eraser", 650, 600, 700, 650)])
        Result = Self.Locate(FakeOllama([Held] * 2, TaskReply=Query), task_object={"phrase": "eraser", "label": "eraser"})
        Self.assertEqual(Self.Labels(Result, "task_query_only"), [])
        Self.assertIn("robot's own arm", next(Item["reason"] for Item in Result["withheld"] if Item.get("task_query_only")))
        Pose = {"ok": True, "joints": {"shoulder_pan": 0.0}}
        Result = Self.Locate(FakeOllama([Held] * 2, TaskReply=Query), task_object={"phrase": "eraser", "label": "eraser"}, arm=Pose)
        Self.assertEqual(Self.Labels(Result, "task_query_only"), ["eraser"])
        # Without depth the pose masks nothing: the table plane would place the arm like any object.
        Result = Self.Locate(FakeOllama([Held] * 2, TaskReply=Query), UseDepth=False,
                             task_object={"phrase": "eraser", "label": "eraser"}, arm=Pose)
        Self.assertEqual(Self.Labels(Result, "task_query_only"), [])
        Self.assertEqual(Self.Labels(Result), ["eraser"])
        Self.assertEqual(Result["objects"][0]["method"], "table-plane")

    def TestTaskQueryBoxOnTheArmIsWithheldWithoutDepth(Self):
        # A webcam or table-plane capture has no depth to mask the arm with, pose or not: a query box on a spot
        # the samples called the robot is withheld before any refine, never projected onto the desk as an
        # object. A query box off the arm in the same capture is still placed.
        Held = [Eraser, Box("robot arm", 600, 300, 780, 900)]
        Task = {"phrase": "eraser", "label": "eraser"}
        for Case, Options in (("webcam, no arm pose", {}), ("with the arm pose", {"arm": {"ok": True, "joints": {"shoulder_pan": 0.0}}})):
            with Self.subTest(Case):
                Fake = FakeOllama([Held] * 2, TaskReply=json.dumps([Box("eraser", 650, 600, 700, 650)]))
                Result = Self.Locate(Fake, UseDepth=False, task_object=Task, **Options)
                Self.assertFalse(Result["depth"])
                Self.assertIn({"label": "eraser", "reason": "on the robot's own arm, which this capture cannot mask out",
                               "task_query_only": True}, Result["withheld"])
                Self.assertEqual([(Object["label"], Object["method"], Object["task_query_only"]) for Object in Result["objects"]],
                                 [("eraser", "table-plane", False)])
                Self.assertEqual(Fake.Refines, ["eraser"])  # the sampled eraser only: the arm spot is never refined
                Fake = FakeOllama([Held] * 2, TaskReply=json.dumps([Box("eraser", 200, 600, 240, 640)]))
                Result = Self.Locate(Fake, UseDepth=False, task_object=Task, **Options)
                Self.assertEqual([(Object["method"], Object["task_match"]) for Object in Result["objects"] if Object["task_query_only"]],
                                 [("table-plane", True)])
                Self.assertEqual([Item for Item in Result["withheld"] if Item.get("task_query_only")], [])

    def TestTaskObjectPhraseWithBinIsNotWithheldAsTheBin(Self):
        # The bin check reads the label: "eraser in the bin" is an eraser, the detector's "black bin" is the bin.
        Fake = FakeOllama([[Cup, Box("black bin", 100, 600, 300, 900)]] * 2,
                          TaskReply=json.dumps([Box("eraser", 150, 700, 190, 740)]))
        Result = Self.Locate(Fake, task_object={"phrase": "eraser in the bin", "label": "eraser"})
        Self.assertEqual(Self.Labels(Result, "task_query_only"), ["eraser"])
        Self.assertEqual([Item["label"] for Item in Result["withheld"]], ["black bin"])

    def TestNoTaskObjectMakesNoExtraCall(Self):
        Fake = FakeOllama([[Eraser, Cup]] * 2)
        Result = Self.Locate(Fake)
        Self.assertEqual(Fake.TaskPayloads, [])
        Self.assertEqual(Fake.Coarse + len(Fake.Refines), 4)
        for Object in Result["objects"]:
            Self.assertEqual((Object["task_match"], Object["task_query_only"]), (False, False))


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestObjectLocator` runs the Test* methods too (unittest names this hook)."""
    Prefixed = unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return unittest.TestSuite(Prefixed.loadTestsFromTestCase(Case) for Case in (LocatorTests, LocateTests))


if __name__ == "__main__":
    Loader = unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Suite = unittest.TestSuite(Loader.loadTestsFromTestCase(Case) for Case in (LocatorTests, LocateTests))
    Result = unittest.TextTestRunner(verbosity=2).run(Suite)
    raise SystemExit(not Result.wasSuccessful())
