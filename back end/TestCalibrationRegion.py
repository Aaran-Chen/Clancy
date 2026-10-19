"""Where a depth-measured object counts as outside the camera calibration. No hardware.

Depth measures an object directly, so its accuracy is set by the fitted camera-to-arm transform,
which is good where the calibration tape was and degrades with distance from it. The old test
checked the object's desk contact pixel against the pixel extent of the tape, which is always
9-16 cm above the desk: every object of 2026-09-29 (and the desk under the tips) was tagged
[UNRELIABLE]. Depth objects are now judged by their XY distance from the calibrated tips' hull
(SelfCalibration.TipRegionMarginCm); table-plane objects keep the pixel test.

Run:  python TestCalibrationRegion.py
"""
import base64
import contextlib
import json
import math
import os
os.environ.setdefault("SO101_LOCATE_MODE", "vlm")  # this suite tests the voted multi-call locate; TestSingleCallLocate.py the default
import unittest
from pathlib import Path
from unittest.mock import patch

Backend = Path(__file__).resolve().parent
# Detections are recorded (SessionLog); keep test runs out of the real record.
os.environ["SO101_CAPTURES_ROOT"] = str(Backend / "TestArtifacts" / "SessionCaptures")

import numpy as np  # noqa: E402

import calibration  # noqa: E402
import RecordedCaptures as RC  # noqa: E402
import object_locator as OL  # noqa: E402
import SelfCalibration as SC  # noqa: E402

# The fit saved on 2026-09-28 22:45 and the run it came from (RecordedCaptures: not on disk, the test fails).
RunDirectory = RC.Day("2026-09-28") / "selfcal" / "runs" / "20260928_224357_b1e21f8c4fba"

# Depth-measured objects of 2026-09-29 (grounding.jsonl, display frame cm) and what they should be.
Trusted = {"eraser 07:58": (5.18, 34.69), "eraser 07:59": (4.95, 34.58), "USB drive 16:02": (4.11, 32.33),
           "USB drive 16:05": (4.07, 32.29), "small black object 16:10": (4.23, 32.50), "plastic cup 07:59": (-11.02, 30.73)}
Untrusted = {"eraser 07:57 (18 cm out)": (14.47, 29.34), "keyboard": (10.13, 97.97), "paper": (40.35, 42.57),
             "box": (37.26, 32.84), "pliers in the bin": (7.50, 8.82)}


def RunTips():
    """The 22:43 run's tips: the last joint of every archived pose, display frame."""
    Tips = []
    for Capture in sorted((RunDirectory / "observations").glob("*_pose*.json")):
        Joint = json.loads(Capture.read_text(encoding="utf-8"))["joints"][-1]
        Tips.append([Joint["x"], Joint["y"], Joint["z"]])
    return np.array(Tips)


class TipRegionTests(unittest.TestCase):
    def TestHullAndBoundsOfTheTips(Self):
        Tips = np.array([[0, 0, 5], [10, 0, 7], [10, 10, 9], [0, 10, 11], [5, 5, 6], [5, 0, 8]], dtype=float)
        Region = SC.TipRegion(Tips)
        Self.assertEqual(Region["observed_tip_bounds_cm"], {"x": [0.0, 10.0], "y": [0.0, 10.0], "z": [5.0, 11.0]})
        Hull = Region["observed_tip_hull_cm"]
        Self.assertEqual(sorted(map(tuple, Hull)), [(0.0, 0.0), (0.0, 10.0), (10.0, 0.0), (10.0, 10.0)])
        Area = 0.5 * sum(A[0] * B[1] - B[0] * A[1] for A, B in zip(Hull, Hull[1:] + Hull[:1]))
        Self.assertGreater(Area, 0, "the hull is stored counterclockwise")
        # Inside and on the edge: 0. Outside: the distance to the nearest edge or corner.
        for (X, Y), Expected in (((5, 5), 0.0), ((10, 4), 0.0), ((13, 5), 3.0), ((5, -4), 4.0), ((13, 14), 5.0)):
            Self.assertAlmostEqual(SC.TipRegionDistance(Region, X, Y), Expected, places=6, msg=(X, Y))
        Self.assertIsNone(SC.TipRegionDistance({"type": "pinhole_depth_v1"}, 5, 5), "an older fit stores no region")
        Self.assertEqual(SC.TipRegionDistance(Region, math.nan, 5), math.inf)
        # What /locate returns: a non-finite point is extrapolated and still serialises (no infinity in JSON).
        Judged = calibration.depth_point_extrapolated(Region, math.nan, 5)
        Self.assertEqual(Judged, {"extrapolated": True, "outside_calibration_cm": None})
        json.dumps(Judged, allow_nan=False)
        Self.assertEqual(calibration.depth_point_extrapolated(Region, 13, 5), {"extrapolated": False, "outside_calibration_cm": 3.0})
        Self.assertIsNone(calibration.depth_point_extrapolated(None, 13, 5))

    def TestDegenerateTipsStillGiveADistance(Self):
        Line = SC.TipRegion(np.array([[0, 0, 5], [10, 0, 6], [5, 0, 7]], dtype=float))
        Self.assertEqual(len(Line["observed_tip_hull_cm"]), 2)
        Self.assertAlmostEqual(SC.TipRegionDistance(Line, 5, 3), 3.0)
        Self.assertAlmostEqual(SC.TipRegionDistance(Line, 13, 4), 5.0)
        Point = SC.TipRegion(np.array([[1, 1, 5], [1, 1, 6]], dtype=float))
        Self.assertAlmostEqual(SC.TipRegionDistance(Point, 4, 5), 5.0)

    def TestRecordedObjectsAgainstTheRealRun(Self):
        RC.Need(RunDirectory / "observations", What="the 2026-09-28 22:43 calibration run")
        Tips = RunTips()
        Self.assertEqual(len(Tips), 24)
        Region = SC.TipRegion(Tips)
        Rows = []
        for Name, (X, Y) in {**Trusted, **Untrusted}.items():
            Outside = SC.TipRegionDistance(Region, X, Y)
            Rows.append(f"{Name:26s} ({X:6.2f}, {Y:6.2f}) {Outside:5.1f} cm outside")
            Self.assertEqual(Outside > SC.TipRegionMarginCm, Name in Untrusted, Rows[-1])
        print("\n  " + "\n  ".join(Rows))
        # The desk directly under the calibration is inside; the reviewer's far checks are not.
        Centre = Tips[:, :2].mean(axis=0)
        Self.assertEqual(SC.TipRegionDistance(Region, *Centre), 0.0)
        Self.assertGreater(SC.TipRegionDistance(Region, 10.0, 98.0), SC.TipRegionMarginCm)
        Self.assertGreater(SC.TipRegionDistance(Region, -45.0, 60.0), SC.TipRegionMarginCm)


def Reply(Content):
    return {"model": "qwen3.6:latest", "message": {"role": "assistant", "content": Content}, "done": True,
            "done_reason": "stop", "prompt_eval_count": 1, "eval_count": 1, "load_duration": 0}


Frame = "data:image/jpeg;base64," + base64.b64encode(OL.cv2.imencode(".jpg", np.zeros((720, 1280, 3), np.uint8))[1]).decode()
# Two boxes on the 0-1000 grid, measured 8 cm and 20 cm outside a tip hull shaped like the 22:43
# run's (X -35..-3, Y 26..37, 9-16 cm above the desk).
Scene = [{"label": "eraser", "box": [300, 500, 340, 560]}, {"label": "keyboard", "box": [600, 100, 700, 200]}]
Measured = {"eraser": (5.0, 34.6), "keyboard": (-10.0, 57.0)}
RegionModel = {"table_z": -2.3, "observed_tip_bounds_cm": {"x": [-35.0, -3.0], "y": [26.0, 37.0], "z": [7.0, 14.0]},
               "observed_tip_hull_cm": [[-35.0, 26.0], [-3.0, 26.0], [-3.0, 37.0], [-35.0, 37.0]]}


def Fake(Url, Payload=None, timeout=10):
    Prompt = Payload["messages"][0]["content"]
    return Reply("[]" if "Find each of these objects" in Prompt else json.dumps(Scene))


def Measure(DepthMm, Box, Intrinsics, Model, TableZ, Mirrored, Surfaces=None, Exclude=None):
    Label = "eraser" if Box[0] < 0.5 else "keyboard"
    X, Y = Measured[Label]
    return {"ok": True, "x": X, "y": Y, "z": -2.3, "top_z": -1.1, "height_cm": 1.2, "size_cm": [5.0, 2.0],
            "distance_cm": 55.0, "support_measured": True, "support_z_measured": -2.3, "valid_fraction": 0.95,
            "support_branch": "desk"}


class LocatorFlagTests(unittest.TestCase):
    """object_locator's 'extrapolated' for depth and table-plane entries, with the model and depth stood in."""

    def Locate(Self, Model, UseDepth=True, PixelSaysExtrapolated=True):
        Depth = {"depth_mm": np.zeros((720, 1280), np.uint16), "intrinsics": {}, "mirrored": False, "seq": 7}
        # On 2026-09-29 the pixel test flagged every object: the desk sits below the elevated tape's pixels.
        Plane = {"ok": True, "x": 1.0, "y": 30.0, "z": -2.3, "extrapolated": PixelSaysExtrapolated}
        with contextlib.ExitStack() as Stack:
            for Target, Name, Value in ((OL, "_http_json", {"side_effect": Fake}),
                                        (OL, "_pick_model", {"return_value": "qwen3.6:latest"}),
                                        (OL.calibration, "camera_model", {"return_value": Model}),
                                        (OL.calibration, "status", {"return_value": {"calibrated": True, "rms_cm": 0.2}}),
                                        (OL.calibration, "pixel_to_plane", {"return_value": Plane}),
                                        (OL.CameraDrift, "CheckCapture", {"return_value": {"ok": True, "angle_deg": 0.0, "offset_cm": 0.0}}),
                                        (OL.CameraDrift, "DisplayDeskPlane", {"return_value": None}),
                                        (OL.DepthGeometry, "ModelUsesDepthCamera", {"return_value": True}),
                                        (OL.DepthGeometry, "WorkspaceSurfaces", {"return_value": None}),
                                        (OL.DepthGeometry, "ArmExclusion", {"return_value": lambda Points: None}),
                                        (OL.DepthGeometry, "MeasureBox", {"side_effect": Measure})):
                Stack.enter_context(patch.object(Target, Name, **Value))
            Result = OL.locate_objects(Frame, depth=Depth if UseDepth else None)
        Self.assertTrue(Result["ok"], Result)
        return {Object["label"]: Object for Object in Result["objects"]}

    def TestDepthObjectsAreJudgedByTheCalibratedTips(Self):
        Objects = Self.Locate(RegionModel)
        Self.assertEqual(Objects["eraser"]["method"], "depth")
        Self.assertFalse(Objects["eraser"]["extrapolated"], "8 cm from the tips is inside the margin, whatever the pixel test says")
        Self.assertAlmostEqual(Objects["eraser"]["outside_calibration_cm"], 8.0, places=1)
        Self.assertTrue(Objects["keyboard"]["extrapolated"], "20 cm from the tips is outside the margin")
        Self.assertAlmostEqual(Objects["keyboard"]["outside_calibration_cm"], 20.0, places=1)
        # The flag follows the tips even when the pixel test would have passed the object.
        Self.assertTrue(Self.Locate(RegionModel, PixelSaysExtrapolated=False)["keyboard"]["extrapolated"])

    def TestAFitWithoutATipRegionKeepsThePixelTest(Self):
        Objects = Self.Locate({"table_z": -2.3})
        Self.assertTrue(all(Object["extrapolated"] for Object in Objects.values()))
        Self.assertTrue(all("outside_calibration_cm" not in Object for Object in Objects.values()))

    def TestTablePlaneObjectsKeepThePixelTest(Self):
        for PixelSays in (True, False):
            Objects = Self.Locate(RegionModel, UseDepth=False, PixelSaysExtrapolated=PixelSays)
            Self.assertTrue(Objects and all(Object["method"] == "table-plane" for Object in Objects.values()))
            Self.assertTrue(all(Object["extrapolated"] is PixelSays for Object in Objects.values()))


# BackfillTests checked a one-off backfill of 2026-09-29: the 22:45 fit in camera_calibration.json got its tip region from
# its own run. That fit was replaced on 2026-09-30 and never committed with the backfill, so the check skipped ever since
# ("a newer fit") and can never run again. A saved fit's tip region is checked where Solve stores it, on the same 22:43
# run: TestSelfCalibration's RealRunRegionCase (tip bounds and hull stored).


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestCalibrationRegion` runs the Test* methods too (unittest names this hook)."""
    Prefixed = unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return unittest.TestSuite(Prefixed.loadTestsFromTestCase(Case) for Case in (TipRegionTests, LocatorFlagTests))


if __name__ == "__main__":
    Loader = unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Suite = unittest.TestSuite(Loader.loadTestsFromTestCase(Case) for Case in (TipRegionTests, LocatorFlagTests))
    Result = unittest.TextTestRunner(verbosity=2).run(Suite)
    raise SystemExit(not Result.wasSuccessful())
