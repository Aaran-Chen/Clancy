"""The single-call locate (object_locator LOCATE_MODES "single_call"): one model call, depth measures every box.

Ollama is faked by prompt; the depth is the synthetic ray-cast workspace of TestDepthCamera (a desk, the bin, a can
and an eraser on the desk), and the camera check runs for real on it. Checked: one call per /locate, the reply's
forms, the task's object first and depth-refined, a box on nothing withheld, a box off the object grown back onto
it by depth, the one retry when the call misses the task's object, one measurement per object for the tripwire, the
robot's own parts withheld, and the fallbacks to the voted multi-call path.

Run:  python TestSingleCallLocate.py
"""
import json
import os
import time
import unittest
from unittest.mock import patch

import numpy as np

import TestSurfaceSupport as T  # (sets the scratch captures folder and no declared surfaces)
import DepthGeometry as DG
import object_locator as Locator
from TestAutomaticScene import Registry

# The suites imported above pin the voted path for themselves; this one tests the default.
os.environ.pop("SO101_LOCATE_MODE", None)

TableZ = T.TableZ
DeskCan = (np.array([-17.0, 36.0, TableZ]), np.array([-10.4, 42.6, TableZ + 12.0]))
Eraser = (np.array([4.0, 34.0, TableZ]), np.array([9.0, 37.0, TableZ + 2.5]))
Empty = (np.array([10.0, 44.0, TableZ]), np.array([16.0, 50.0, TableZ + 6.0]))  # nothing stands there
Scene = T.TDC.Workspace([DeskCan, Eraser])
Depth = T.TDC.RenderDepth(T.Camera, Scene, FloorZ=T.TDC.RoomFloorZ)


def Grid(Box):
    """A normalized capture box on the model's 0-1000 grid."""
    return [int(round(Value * 1000)) for Value in Box]


def Tight(Item):
    return T.TDC.CaptureBox(T.Camera, *Item, Pad=0.0)


def Reply(Content):
    return {"message": {"role": "assistant", "content": Content}, "done": True, "done_reason": "stop",
            "prompt_eval_count": 1200, "eval_count": 150, "load_duration": 1_000_000}


class FakeOllama:
    """The single call answers Single; the strict task query answers Query."""

    def __init__(Self, Single, Query="[]"):
        Self.Single, Self.Query, Self.Prompts = Single, Query, []

    def __call__(Self, Url, Payload=None, timeout=10):
        Prompt = Payload["messages"][0]["content"]
        Self.Prompts.append(Prompt)
        if "complete inventory" in Prompt or "List every distinct movable object" in Prompt:
            return Reply(Self.Single if isinstance(Self.Single, str) else json.dumps(Self.Single))
        if "The image may NOT contain it" in Prompt:
            return Reply(Self.Query)
        return Reply("[]")  # the voted samples: nothing


def Locate(Fake, Task=("the can", "can"), DepthMm=Depth, Mode=None, Arm=None):
    Model = T.TDC.Verified(T.Camera, T.TDC.Workspace(), FloorZ=T.TDC.RoomFloorZ)
    Records = []
    with patch.object(Locator, "SceneSafety", Registry()), \
            patch.object(T.DeclaredSurfaces, "Load", return_value=[]), \
            patch.object(T.calibration, "camera_model", return_value=Model), \
            patch.object(T.calibration, "status", return_value={"calibrated": True, "rms_cm": 0.3}), \
            patch.object(T.calibration, "pixel_to_plane", return_value={"ok": False}), \
            patch.object(Locator, "_pick_model", return_value="synthetic"), \
            patch.object(Locator, "_http_json", side_effect=Fake), \
            patch.object(Locator, "_Record", side_effect=lambda *Arguments, **Keywords: Records.append(Arguments)):
        Result = Locator.locate_objects(T.TDC.Encode(np.zeros((T.Height, T.Width, 3), np.uint8)),
                                        depth=None if DepthMm is None else {"depth_mm": DepthMm, "intrinsics": T.TDC.Intrinsics,
                                                                            "mirrored": True, "seq": 12, "time": time.time()},
                                        mirrored=True, arm=Arm, locate_mode=Mode,
                                        task_object={"phrase": Task[0], "label": Task[1]} if Task else None)
    return Result, Records


def Single(Rows, Task=1, Outline=None):
    return {"objects": Rows, "task": Task, "outline": Outline or []}


class ParseTests(unittest.TestCase):
    def TestTheTaskNumberPutsItsRowFirstWithTheOutline(Self):
        Content = json.dumps(Single([["eraser", 100, 100, 200, 200], ["can", 400, 500, 450, 600]], 2, [[400, 500], [450, 500], [450, 600]]))
        Rows, Absent = Locator._ParseRows(Content, 1280, 720, True)
        Self.assertFalse(Absent)
        Self.assertEqual([Row["label"] for Row in Rows], ["can", "eraser"])
        Self.assertTrue(Rows[0]["task_row"])
        Self.assertFalse(Rows[1]["task_row"])
        Self.assertEqual(Rows[0]["box"], (0.4, 0.5, 0.45, 0.6))
        Self.assertEqual(len(Rows[0]["outline"]), 3)

    def TestTheOutlinePicksTheRowWhenTheCountIsWrong(Self):
        # 2026-10-03 15:48:40: "task": 2 (the can) with the outline drawn on the 3rd row, the pill container.
        Content = ('{"objects": [["mouse", 198, 48, 258, 118], ["can", 325, 195, 385, 355], ["pill container", 488, 435, 528, 550], '
                   '["tape", 450, 475, 495, 575]], "task": 2, "outline": [[490, 440], [515, 438], [525, 450], [528, 535], [520, 548], '
                   '[495, 550], [488, 535], [490, 455]]}')
        Rows, Absent = Locator._ParseRows(Content, 1280, 720, True)
        Self.assertEqual((Rows[0]["label"], Rows[0]["task_row"], Absent), ("pill container", True, False))
        # An outline on no row leaves the count.
        Content = json.dumps(Single([["eraser", 100, 100, 200, 200], ["can", 400, 500, 450, 600]], 1, [[800, 800], [900, 800], [900, 900]]))
        Self.assertEqual(Locator._ParseRows(Content, 1280, 720, True)[0][0]["label"], "eraser")
        # The task-first form: {"task": row or null, "objects": [the others]}.
        Rows, Absent = Locator._ParseRows('{"task": ["can", 400, 500, 450, 600, [[400, 500], [450, 500], [425, 600]]], '
                                          '"objects": [["cup", 100, 100, 200, 200]]}', 1280, 720, True)
        Self.assertEqual([(Row["label"], Row["task_row"]) for Row in Rows], [("can", True), ("cup", False)])
        Rows, Absent = Locator._ParseRows('{"task": null, "objects": [["cup", 100, 100, 200, 200]]}', 1280, 720, True)
        Self.assertEqual((len(Rows), Absent), (1, True))

    def TestANullOrImpossibleTaskNumberMeansNoTaskRow(Self):
        for Number in (None, 0, 3, "1", True, 1.5):
            with Self.subTest(Number=Number):
                Rows, Absent = Locator._ParseRows(json.dumps(Single([["eraser", 100, 100, 200, 200], ["cup", 300, 300, 400, 400]], Number)),
                                                  1280, 720, True)
                Self.assertTrue(Absent)
                Self.assertEqual([Row["task_row"] for Row in Rows], [False, False])

    def TestTheArrayAndKeyedFormsAreReadToo(Self):
        Rows, Absent = Locator._ParseRows('```json\n[["can", 400, 500, 450, 600, [[400, 500], [450, 500], [425, 600]]], '
                                          '{"label": "cup", "box": [0.1, 0.1, 0.2, 0.2]}]\n```', 1280, 720, True)
        Self.assertFalse(Absent)
        Self.assertEqual([(Row["label"], Row["task_row"]) for Row in Rows], [("can", True), ("cup", False)])
        Self.assertEqual(Rows[1]["box"], (0.1, 0.1, 0.2, 0.2))
        Rows, Absent = Locator._ParseRows('[null, ["cup", 100, 100, 200, 200]]', 1280, 720, True)
        Self.assertTrue(Absent)
        Self.assertEqual(len(Rows), 1)
        # Without a task the first row is an ordinary object.
        Rows, Absent = Locator._ParseRows('[["can", 400, 500, 450, 600]]', 1280, 720, False)
        Self.assertEqual((Rows[0]["task_row"], Absent), (False, False))

    def TestBadRowsAreSkippedAndNoArrayIsAnError(Self):
        Rows, _ = Locator._ParseRows(json.dumps(Single([["huge", 0, 0, 1000, 1000], ["flat", 100, 100, 100, 200], [5, 1, 2, 3, 4],
                                                        ["short", 1, 2], ["ok", 100, 100, 200, 200]], None)), 1280, 720, True)
        Self.assertEqual([Row["label"] for Row in Rows], ["ok"])
        Self.assertIn("error", Locator._ParseRows("I see a can.", 1280, 720, True))

    def TestThePromptIsCompactAndNamesTheTask(Self):
        Prompt = Locator._single_call_prompt("black thing on the desk")
        Self.assertIn("The user asked for: black thing on the desk.", Prompt)
        Self.assertIn('"task": <number>', Prompt)
        Self.assertIn("0-1000 grid", Prompt)
        Self.assertNotIn("task", Locator._single_call_prompt(None).split("Reply")[1].split("\n")[1])

    def TestTheModeComesFromTheCallThenTheEnvironmentThenTheFile(Self):
        Self.assertEqual(Locator.OLLAMA_OPTIONS["locate_mode"], "single_call")
        Self.assertEqual(Locator._LocateMode(), "single_call")
        with patch.dict(os.environ, {"SO101_LOCATE_MODE": "vlm"}):
            Self.assertEqual(Locator._LocateMode(), "vlm")
            Self.assertEqual(Locator._LocateMode("single_call"), "single_call")
        with patch.dict(os.environ, {"SO101_LOCATE_MODE": "nonsense"}):
            Self.assertEqual(Locator._LocateMode("also nonsense"), "single_call")


class MeasureBoxHookTests(unittest.TestCase):
    """MeasureBox's opt-in Only and Trace leave every result as it was without them."""

    def TestWithoutTheHooksTheResultIsUnchanged(Self):
        Box = T.TDC.CaptureBox(T.Camera, *DeskCan)
        Plain = DG.MeasureBox(Depth, Box, T.TDC.Intrinsics, T.Camera, TableZ, True, Surfaces=DG.WorkspaceSurfaces(None, TableZ))
        Trace = {}
        Traced = DG.MeasureBox(Depth, Box, T.TDC.Intrinsics, T.Camera, TableZ, True, Surfaces=DG.WorkspaceSurfaces(None, TableZ), Trace=Trace)
        Self.assertEqual(Plain, Traced)
        Self.assertNotIn("_component", Plain)
        U0, V0, U1, V1 = DG.BoxToNative(Box, T.Width, T.Height, True)
        C = Trace["component_px"]
        Self.assertTrue(U0 <= C[0] < C[2] <= U1 and V0 <= C[1] < C[3] <= V1, (C, (U0, V0, U1, V1)))

    def TestOnlyKeepsTheObjectInsideTheOutline(Self):
        # A box over both the can and the eraser; the outline is the eraser's: the eraser is measured.
        Can, Rubber = T.TDC.CaptureBox(T.Camera, *DeskCan), T.TDC.CaptureBox(T.Camera, *Eraser)
        Both = (min(Can[0], Rubber[0]), min(Can[1], Rubber[1]), max(Can[2], Rubber[2]), max(Can[3], Rubber[3]))
        U0, V0, U1, V1 = DG.BoxToNative(Rubber, T.Width, T.Height, True)
        Only = np.zeros((T.Height, T.Width), bool)
        Only[V0:V1, U0:U1] = True
        Surfaces = DG.WorkspaceSurfaces(None, TableZ)
        Measured = DG.MeasureBox(Depth, Both, T.TDC.Intrinsics, T.Camera, TableZ, True, Surfaces=Surfaces, Only=Only)
        Self.assertTrue(Measured["ok"], Measured)
        Self.assertAlmostEqual(Measured["x"], 6.5, delta=0.8)
        Self.assertAlmostEqual(Measured["height_cm"], 2.5, delta=0.5)


class SingleCallLocateTests(unittest.TestCase):
    def Rows(Self, CanBox=None):
        return [["eraser", *Grid(Tight(Eraser))], ["can", *Grid(CanBox or Tight(DeskCan))]]

    def TestOneCallMeasuresEveryObjectAndTheTaskFirst(Self):
        Fake = FakeOllama(Single(Self.Rows(), 2))
        Result, Records = Locate(Fake)
        Self.assertTrue(Result["ok"], Result)
        Self.assertEqual((len(Fake.Prompts), Result["model_calls"], Result["locate_mode"]), (1, 1, "single_call"))
        Objects = {Item["label"]: Item for Item in Result["objects"]}
        Self.assertEqual(set(Objects), {"can", "eraser"})
        Can = Objects["can"]
        Self.assertTrue(Can["task_match"] and Can["refined"] and Can["depth_refined"])
        Self.assertFalse(Can["task_query_only"] or Can["unstable"])
        Self.assertAlmostEqual(Can["x"], -13.7, delta=0.6)
        Self.assertAlmostEqual(Can["y"], 39.3, delta=0.8)
        Self.assertAlmostEqual(Can["top_z"], TableZ + 12.0, delta=0.4)
        # The other objects keep the coarse-box flag the samples' unrefined boxes had.
        Self.assertFalse(Objects["eraser"]["task_match"] or Objects["eraser"]["refined"])
        Self.assertIn("scene", Result)
        # One measurement per object reaches the record (the tripwire counts each once).
        Diagnostics = Records[-1][3]
        Self.assertEqual(sorted(Item["label"] for Item in Diagnostics), ["can", "eraser"])
        Self.assertIn("timing_ms", Result)

    def TestABoxOnNothingIsWithheld(Self):
        Rows = [*Self.Rows(), ["mug", *Grid(Tight(Empty))]]
        Result, _ = Locate(FakeOllama(Single(Rows, 2)))
        Self.assertNotIn("mug", [Item["label"] for Item in Result["objects"]])
        Mug = next(Item for Item in Result["withheld"] if Item["label"] == "mug")
        Self.assertIn("nothing stands above", Mug["reason"])

    def TestABoxOffTheObjectIsGrownBackOntoItByDepth(Self):
        X1, Y1, X2, Y2 = Tight(DeskCan)
        Shift = 0.5 * (X2 - X1)
        Result, Records = Locate(FakeOllama(Single(Self.Rows((X1 + Shift, Y1, X2 + Shift, Y2)), 2)))
        Can = next(Item for Item in Result["objects"] if Item["label"] == "can")
        Self.assertTrue(Can["depth_refined"] and Can["refined"], Can)
        Self.assertAlmostEqual(Can["x"], -13.7, delta=0.6)
        Self.assertAlmostEqual(Can["y"], 39.3, delta=0.8)
        Self.assertGreater(Can["size_cm"][0], 5.0)
        Self.assertEqual([Item["label"] for Item in Records[-1][3]].count("can"), 1)
        # Measured on the half box alone the can would be too narrow and off-centre.
        Half = DG.MeasureBox(Depth, (X1 + Shift, Y1, X2 + Shift, Y2), T.TDC.Intrinsics, T.Camera, TableZ, True,
                             Surfaces=DG.WorkspaceSurfaces(None, TableZ))
        Self.assertLess(Half["size_cm"][0], 5.0)

    def TestAMissedTaskObjectGetsOneStrictQuery(Self):
        # No task row: one strict query, its box measured as the task query's only find.
        Fake = FakeOllama(Single([["eraser", *Grid(Tight(Eraser))]], None),
                          Query=json.dumps([{"label": "can", "box": Grid(Tight(DeskCan))}]))
        Result, _ = Locate(Fake)
        Self.assertEqual((len(Fake.Prompts), Result["model_calls"]), (2, 2))
        Can = next(Item for Item in Result["objects"] if Item["task_match"])
        Self.assertEqual((Can["label"], Can["task_query_only"]), ("can", True))
        Self.assertAlmostEqual(Can["x"], -13.7, delta=0.6)
        # A task row whose box holds nothing in depth: the query too. An empty answer leaves no task match.
        Fake = FakeOllama(Single([["can", *Grid(Tight(Empty))], ["eraser", *Grid(Tight(Eraser))]], 1))
        Result, _ = Locate(Fake)
        Self.assertEqual(Result["model_calls"], 2)
        Self.assertFalse(any(Item["task_match"] for Item in Result["objects"]))
        Self.assertIn("scene_error", Result)

    def TestAQueryBoxOnAMeasuredObjectMarksIt(Self):
        Fake = FakeOllama(Single(Self.Rows(), None), Query=json.dumps([{"label": "can", "box": Grid(Tight(DeskCan))}]))
        Result, _ = Locate(Fake)
        Self.assertEqual([Item["label"] for Item in Result["objects"] if Item["task_match"]], ["can"])
        Self.assertEqual(len(Result["objects"]), 2)

    def TestNoTaskMeansNoQueryAndNoTaskMatch(Self):
        Fake = FakeOllama(json.dumps(Self.Rows()))
        Result, _ = Locate(Fake, Task=None)
        Self.assertEqual(len(Fake.Prompts), 1)
        Self.assertEqual(sorted(Item["label"] for Item in Result["objects"]), ["can", "eraser"])
        Self.assertFalse(any(Item["task_match"] for Item in Result["objects"]))

    def TestTheRobotsOwnPartsAndRepeatsAreLeftOut(Self):
        Rows = [*Self.Rows(), ["robot arm", 800, 300, 950, 900], ["red eraser", *Grid(Tight(Eraser))]]
        Result, _ = Locate(FakeOllama(Single(Rows, 2)))
        Self.assertEqual(sorted(Item["label"] for Item in Result["objects"]), ["can", "eraser"])
        Self.assertEqual([Item["reason"] for Item in Result["withheld"] if Item["label"] == "robot arm"], [Locator.ROBOT_PARTS_REASON])

    def TestAFailedCallFailsTheLocate(Self):
        Result, Records = Locate(FakeOllama("I cannot see the image."))
        Self.assertFalse(Result["ok"])
        Self.assertIn("JSON", Result["error"])

        def Down(Url, Payload=None, timeout=10):
            raise OSError("connection refused")
        Result, _ = Locate(Down)
        Self.assertFalse(Result["ok"])
        Self.assertIn("Ollama request failed", Result["error"])

    def TestWithoutDepthOrWhenAskedTheVotedPathRuns(Self):
        for Case, Options in (("asked", {"Mode": "vlm"}), ("no depth", {"DepthMm": None})):
            with Self.subTest(Case):
                Fake = FakeOllama(Single(Self.Rows(), 2))
                # Without depth the gate passes as a webcam calibration's would (its own suites cover it).
                Checked = {"ok": True, "angle_deg": 0.0, "offset_cm": 0.0, "reason": "checked"}
                with patch.object(Locator.CameraDrift, "CheckCapture", return_value=Checked) if Case == "no depth" else patch.object(
                        Locator, "LOCATE_MODES", Locator.LOCATE_MODES):
                    Result, _ = Locate(Fake, **Options)
                Self.assertEqual(Result.get("locate_mode"), "vlm", Result)
                Self.assertFalse(any("complete inventory" in Prompt for Prompt in Fake.Prompts))
                Self.assertGreaterEqual(len(Fake.Prompts), 2)  # the samples and the task query

    def TestTheCameraGateStillRunsFirst(Self):
        Fake = FakeOllama(Single(Self.Rows(), 2))
        Blocked = {"ok": False, "error_code": "camera_moved", "error": "The camera moved."}
        with patch.object(Locator.CameraDrift, "CheckCapture", return_value=Blocked):
            Result, _ = Locate(Fake)
        Self.assertEqual((Result["ok"], Result["error_code"]), (False, "camera_moved"))
        Self.assertEqual(Fake.Prompts, [])


def load_tests(Loader, Tests, Pattern):
    Prefixed = unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return unittest.TestSuite(Prefixed.loadTestsFromTestCase(Case) for Case in (ParseTests, MeasureBoxHookTests, SingleCallLocateTests))


if __name__ == "__main__":
    Loader = unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Suite = unittest.TestSuite(Loader.loadTestsFromTestCase(Case) for Case in (ParseTests, MeasureBoxHookTests, SingleCallLocateTests))
    raise SystemExit(not unittest.TextTestRunner(verbosity=2).run(Suite).wasSuccessful())
