"""The basket rim's red strip is never taken for the gripper's tape (SelfCalibration.BasketStripExclusion). No hardware.

- Recorded (RecordedCaptures; not on disk, they fail, RECORDING MISSING): the 2026-10-10 10:41 run, where the strip
  (664,507) was a candidate in 5 of 24 poses and so survived RobustRedAssignment's recurrence rule, and the 2026-10-12
  09:31 run (strip at 662,485 in all 10 poses). Each run's camera is fitted to its own manual tape clicks (solvePnP on
  the clicked pixels and the tape where the arm put it); WorkspaceFrame.json's strip projects onto the strip through
  it, AddRedProbeObservation then keeps no candidate there, and every other candidate is kept. Without a saved
  camera model nothing is masked.
- The guard: the tape predicted within BasketStripGuardPx of the strip turns the mask off, so a tape passing in
  front of the strip is kept.
- The live checks (CameraDrift.TapeCheck at the desk, RobotReference.TapeCheck off it), on TestDepthCamera's
  synthetic scenes with the strip moved beside the arm's tape: the tape hidden, the strip 3.5 cm away would have
  been taken for the tape (camera_moved); it is ignored. The tape in front of the strip is still the tape.

Run:  python TestBasketStrip.py
"""
import json
import os
import time
import unittest
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np

# Records go to the test captures root, never the real one; no declared surfaces (as TestRobotReference).
os.environ["SO101_CAPTURES_ROOT"] = str(Path(__file__).resolve().parent / "TestArtifacts" / "SessionCaptures")
os.environ["SO101_DECLARED_SURFACES"] = str(Path(__file__).resolve().parent / "TestArtifacts" / "NoDeclaredSurfaces.json")

import TestDepthCamera as TDC  # noqa: E402  (the ray-caster and camera models)
from TestRobotReference import SyntheticArm  # noqa: E402
import CameraDrift as CD  # noqa: E402
import DepthGeometry as DG  # noqa: E402
import RecordedCaptures as RC  # noqa: E402
import RobotReference as RR  # noqa: E402
import SelfCalibration as SC  # noqa: E402

Root = Path(__file__).resolve().parent
Artifacts = Root / "TestArtifacts"
# The 2026-10-02 21:09 calibration's tape offset (pseudo gripper frame): the tape is where it was taped then.
Q1002 = np.array([-0.641375300301567, -0.4925752823905805, -0.3821964662074297])
# (run, the strip's pixel in its frames, poses that saw it)
Runs = {"2026-10-10": ("20261010_104108_026ec24b56df", (664.0, 507.0), 5),
        "2026-10-12": ("20261012_093139_eb5b289f88f3", (661.5, 484.7), 10)}


def RunFolder(Day):
    return RC.Day(Day) / "selfcal" / "runs" / Runs[Day][0]


def ClickedCamera(Session):
    """The run's camera as a depth model: its pose fitted to the manual tape clicks (RANSAC, then refined)."""
    Intrinsics = Session["depth_intrinsics"]
    Clicks = list(Session["manual_clicks"].values())
    World = np.array([(np.array(C["tip"]) + np.array(C["frame"]) @ Q1002) * DG.MirrorVector for C in Clicks])
    Pixels = np.array([C["corners"][0] for C in Clicks], dtype=np.float64)
    Matrix, Distortion = DG.CameraMatrix(Intrinsics), DG.Distortion(Intrinsics)
    _, Rvec, Tvec, Inliers = cv2.solvePnPRansac(World, Pixels, Matrix, Distortion, reprojectionError=12.0, flags=cv2.SOLVEPNP_EPNP)
    Inliers = Inliers.ravel()
    _, Rvec, Tvec = cv2.solvePnP(World[Inliers], Pixels[Inliers], Matrix, Distortion, Rvec, Tvec, useExtrinsicGuess=True)
    return {"type": "pinhole_depth_v1", "rvec": Rvec.ravel().tolist(), "tvec": Tvec.ravel().tolist(), "mirror_x": True,
            "image_size": list(Session["image_size"]), "input_mirrored": False, "marker_corners_gripper_cm": [Q1002.tolist()],
            **{Key: Intrinsics[Key] for Key in ("fx", "fy", "cx", "cy", "dist")}, "f": 0.5 * (Intrinsics["fx"] + Intrinsics["fy"])}


class RecordedRunTests(unittest.TestCase):
    def setUp(Self):
        Self.Patches = [patch.object(SC, "SessionPath", Artifacts / "StripSession.json"),
                        patch.object(SC, "CapturesDir", Artifacts / "StripCaptures")]
        for Patch in Self.Patches:
            Patch.start()

    def tearDown(Self):
        for Patch in Self.Patches:
            Patch.stop()
        SC._Session.clear()
        SC._Session.update({"image_size": None, "mirrored": None, "observations": []})
        SC._RedTracker.clear()
        for Child in sorted((Artifacts / "StripCaptures").rglob("*"), reverse=True) if (Artifacts / "StripCaptures").exists() else []:
            try:
                Child.unlink() if Child.is_file() else Child.rmdir()
            except OSError:
                pass
        try:
            (Artifacts / "StripSession.json").unlink()
        except OSError:
            pass

    def Replay(Self, Day, Model):
        """Every pose's archived frames through AddRedProbeObservation with Model as the saved camera model:
        {pose_index: red_poses entry}."""
        Folder = RunFolder(Day)
        RC.Need(Folder / "Session.json", Folder / "observations", What=f"the {Day} calibration run {Runs[Day][0]}")
        SC.ResetSession()
        SC._RedTracker.clear()
        Entries = {}
        with patch.object(SC, "SavedCameraModel", return_value=Model):
            for Archive in sorted((Folder / "observations").glob("*_pose*.json")):
                Record = json.loads(Archive.read_text(encoding="utf-8"))
                if Record["pose_index"] in Entries:
                    continue
                Before = len(SC._Session.get("red_poses", []))
                SC.AddRedProbeObservation(Record["images"], Record["joints"], Record["mirrored"], Record["pose_index"], None, None, None)
                if len(SC._Session.get("red_poses", [])) > Before:
                    Entries[Record["pose_index"]] = SC._Session["red_poses"][-1]
        return Entries

    def Near(Self, Entry, Pixel):
        return [C for C in Entry["candidates"] if np.linalg.norm(np.subtract(C["centroid"], Pixel)) <= SC.RedStaticRadiusPx]

    def TestTheStoredStripProjectsOntoTheStripInBothRuns(Self):
        for Day, (_, Strip, _) in Runs.items():
            with Self.subTest(day=Day):
                Folder = RunFolder(Day)
                RC.Need(Folder / "Session.json", What=f"the {Day} calibration run")
                Session = json.loads((Folder / "Session.json").read_text(encoding="utf-8"))
                Pixel = SC.ModelPixel(ClickedCamera(Session), SC.BasketStripDisplayCm, Session["depth_intrinsics"], Session["image_size"])
                Self.assertLess(float(np.linalg.norm(Pixel - Strip)), 5.0, Pixel)

    def TestTheRecordedStripIsNeverACalibrationCandidate(Self):
        for Day, (_, Strip, Seen) in Runs.items():
            with Self.subTest(day=Day):
                Folder = RunFolder(Day)
                RC.Need(Folder / "Session.json", What=f"the {Day} calibration run")
                Model = ClickedCamera(json.loads((Folder / "Session.json").read_text(encoding="utf-8")))
                Plain, Masked = Self.Replay(Day, None), Self.Replay(Day, Model)
                Self.assertEqual(sorted(Plain), sorted(Masked))
                # Without a camera model nothing is masked: the strip is a candidate where it was recorded.
                Self.assertEqual(sum(1 for Entry in Plain.values() if Self.Near(Entry, Strip)), Seen)
                Self.assertTrue(all(Entry["basket_strip_px"] is None for Entry in Plain.values()))
                for Pose, Entry in Masked.items():
                    Self.assertEqual(Self.Near(Entry, Strip), [], f"pose {Pose}")
                    # Everything else is kept, the tape included.
                    Others = [C["centroid"] for C in Plain[Pose]["candidates"] if not Self.Near({"candidates": [C]}, Strip)]
                    Self.assertEqual(Others, [C["centroid"] for C in Entry["candidates"]], f"pose {Pose}")
                # A pose whose strip was in only some of its frames (not a stable candidate) masks it too.
                Self.assertGreaterEqual(sum(1 for Entry in Masked.values() if Entry["basket_strip_excluded"]), Seen)


class GuardTests(unittest.TestCase):
    Model = TDC.HeadCamera

    def Scene(Self, Pixel):
        """A tape-sized red square on a light housing at Pixel (native)."""
        Image = np.full((TDC.Height, TDC.Width, 3), 90, np.uint8)
        U, V = np.round(Pixel).astype(int)
        cv2.rectangle(Image, (U - 60, V - 60), (U + 60, V + 60), (235, 235, 235), -1)
        cv2.rectangle(Image, (U - 10, V - 10), (U + 10, V + 10), (30, 30, 215), -1)
        return Image

    def TestTheMaskNeedsACameraModelAndAPredictedTape(Self):
        Strip = SC.ModelPixel(Self.Model, SC.BasketStripDisplayCm)
        Self.assertIsNotNone(Strip)
        Far = Strip + [120.0, 0.0]
        Self.assertEqual(SC.BasketStripExclusion(Self.Model, Far), [Strip.tolist()])
        Self.assertEqual(SC.BasketStripExclusion(None, Far), [])
        Self.assertEqual(SC.BasketStripExclusion(dict(Self.Model, type="pinhole_selfcal_v1"), Far), [], "not a depth model")
        Self.assertEqual(SC.BasketStripExclusion(Self.Model, Far, ImageSize=(640, 480)), [], "fitted at another size")
        Self.assertEqual(SC.BasketStripExclusion(Self.Model, None), [], "no tape prediction")
        with patch.object(SC, "BasketStripDisplayCm", None):
            Self.assertEqual(SC.BasketStripExclusion(Self.Model, Far), [], "no strip stored")
        # Mirrored frames: the strip flips with the image.
        Self.assertEqual(SC.BasketStripExclusion(Self.Model, [TDC.Width - 1 - Far[0], Far[1]], Mirrored=True, ImageSize=(TDC.Width, TDC.Height)),
                         [[TDC.Width - 1 - Strip[0], Strip[1]]])

    def TestATapePassingInFrontOfTheStripIsKept(Self):
        Strip = SC.ModelPixel(Self.Model, SC.BasketStripDisplayCm)
        Image = Self.Scene(Strip)
        for Offset, Kept in ((30.0, True), (SC.BasketStripGuardPx - 1, True), (SC.BasketStripGuardPx + 1, False), (200.0, False)):
            with Self.subTest(tape_px_from_strip=Offset):
                Detection = SC.DetectRedDetailed(Image, Exclude=SC.BasketStripExclusion(Self.Model, Strip + [Offset, 0.0]))
                Self.assertEqual(len(Detection["options"]) == 1, Kept, Detection["options"])
                Self.assertEqual(Detection["excluded"], 0 if Kept else 1)


Tape = np.array([1.2, -0.6, 0.8])
TapePose = {"wrist_roll_raw": 2048, "gripper_raw": 2100}


def StripView(Model, Tape, Strip, TapeVisible):
    """Native image and depth with no desk: a light gripper housing around the arm's tape (red if TapeVisible)
    and a tape-sized red strip with its own light surround, each at its own depth."""
    Image = np.full((TDC.Height, TDC.Width, 3), 90, np.uint8)
    Depth = TDC.RenderDepth(Model, [], FloorZ=TDC.RoomFloorZ)
    for Point, Red in ((Tape, TapeVisible), (Strip, True)):
        Camera = DG.DisplayToCamera(Point, Model)[0]
        U, V = np.round(DG.Project(Camera, TDC.Intrinsics)[0]).astype(int)
        cv2.rectangle(Image, (U - 30, V - 30), (U + 30, V + 30), (235, 235, 235), -1)
        Depth[V - 30:V + 31, U - 30:U + 31] = round(Camera[2] * 10.0)
    for Point, Red in ((Tape, TapeVisible), (Strip, True)):
        U, V = np.round(DG.Project(DG.DisplayToCamera(Point, Model)[0], TDC.Intrinsics)[0]).astype(int)
        if Red:
            cv2.rectangle(Image, (U - 8, V - 8), (U + 8, V + 8), (30, 30, 215), -1)
    return Image, Depth


class LiveCheckTests(unittest.TestCase):
    @classmethod
    def setUpClass(Class):
        Class.Model = dict(TDC.Verified(TDC.HeadCamera, TapePose=TapePose), marker_corners_gripper_cm=[Tape.tolist()])
        Class.Joints, Class.Point = TDC.ArmInView(TDC.HeadCamera, Tape)
        Class.Arm = SyntheticArm(Class.Joints, Class.Point)

    def Pixel(Self, Point):
        return DG.Project(DG.DisplayToCamera(Point, Self.Model)[0], TDC.Intrinsics)[0]

    def Checks(Self, Strip, TapeVisible):
        Image, Depth = StripView(Self.Model, Self.Point, Strip, TapeVisible)
        with patch.object(SC, "BasketStripDisplayCm", np.asarray(Strip, dtype=np.float64)):
            return {"desk": CD.TapeCheck(Image, Depth, TDC.Intrinsics, Self.Model, Self.Arm),
                    "robot": RR.TapeCheck(Image, Depth, TDC.Intrinsics, Self.Model, Self.Arm)}

    def TestTheStripBesideAHiddenTapeIsNotTheTape(Self):
        # 3.5 cm along the desk from the tape (inside TapeNearCm, at its height), past the guard in pixels.
        Strip = Self.Point + [3.5, 0.0, 0.0]
        Self.assertGreater(float(np.linalg.norm(Self.Pixel(Strip) - Self.Pixel(Self.Point))), SC.BasketStripGuardPx)
        # Unmasked (no strip stored) it is taken for the tape and reads as a moved camera.
        Image, Depth = StripView(Self.Model, Self.Point, Strip, TapeVisible=False)
        with patch.object(SC, "BasketStripDisplayCm", None):
            Unmasked = CD.TapeCheck(Image, Depth, TDC.Intrinsics, Self.Model, Self.Arm)
            UnmaskedRobot = RR.TapeCheck(Image, Depth, TDC.Intrinsics, Self.Model, Self.Arm)
        Self.assertEqual((Unmasked["tape_check"], UnmaskedRobot["tape_check"]), ("moved", "moved"), (Unmasked, UnmaskedRobot))
        for Name, Result in Self.Checks(Strip, TapeVisible=False).items():
            with Self.subTest(check=Name):
                Self.assertEqual(Result["tape_check"], "not_visible", Result)
                Self.assertEqual(Result["tape_basket_strip_excluded"], 1)
        # The tape in view beside it: the tape, with the strip set aside.
        for Name, Result in Self.Checks(Strip, TapeVisible=True).items():
            with Self.subTest(check=Name, tape="visible"):
                Self.assertEqual(Result["tape_check"], "ok", Result)
                Self.assertLess(Result["tape_error_cm"], 0.3)

    def TestTheTapeInFrontOfTheStripIsStillTheTape(Self):
        # The strip 0.5 cm from where the arm puts the tape (inside the mask's radius, so the tape's blob would
        # be masked): the guard keeps the tape, which is found where the arm puts it.
        Strip = Self.Point + [0.5, 0.0, 0.0]
        Self.assertLess(float(np.linalg.norm(Self.Pixel(Strip) - Self.Pixel(Self.Point))), SC.RedStaticRadiusPx)
        for Name, Result in Self.Checks(Strip, TapeVisible=True).items():
            with Self.subTest(check=Name):
                Self.assertNotIn("tape_basket_strip_excluded", Result)
                Self.assertEqual(Result["tape_check"], "ok", Result)
                Self.assertLess(Result["tape_error_cm"], 0.6)
        # Without the guard the tape would have been masked with the strip.
        Image, _ = StripView(Self.Model, Self.Point, Strip, TapeVisible=True)
        Self.assertEqual(SC.DetectRedDetailed(Image, Exclude=[Self.Pixel(Strip).tolist()])["options"], [])


Cases = (RecordedRunTests, GuardTests, LiveCheckTests)


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestBasketStrip` runs the Test* methods too (unittest names this hook)."""
    Prefixed = unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return unittest.TestSuite(Prefixed.loadTestsFromTestCase(Case) for Case in Cases)


if __name__ == "__main__":
    Started = time.time()
    Loader = unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Result = unittest.TextTestRunner(verbosity=2).run(unittest.TestSuite(Loader.loadTestsFromTestCase(Case) for Case in Cases))
    print(f"{time.time() - Started:.0f} s")
    raise SystemExit(not Result.wasSuccessful())
