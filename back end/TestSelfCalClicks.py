"""Semi-automatic calibration (SelfCalClicks.py): clicks on archived frames recover the camera.

Each test records a run the way the bridge does: /selfcal/observe's AddObservation archives
every pose's frames (a ray-cast desk with the red tape on a light gripper patch), joints and
16-bit depth. Clicks then name those archives, as the app's click stage does, and
Solve(Source="manual") fits them. No camera, arm or bridge server is opened; the real
camera_calibration.json is never written.

Run:  python TestSelfCalClicks.py      (or python -m unittest TestSelfCalClicks)
"""
import json
import math
import os
import time
import unittest
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np

os.environ.setdefault("SO101_CAPTURES_ROOT", str(Path(__file__).resolve().parent / "TestArtifacts" / "SessionCaptures"))

import arm_geometry as Geo  # noqa: E402
import bridge_server as Bridge  # noqa: E402
import CameraDrift as CD  # noqa: E402
import DepthGeometry as DG  # noqa: E402
import SelfCalClicks as Clicks  # noqa: E402
import SelfCalibration as SC  # noqa: E402
from TestDepthCamera import DisplayOffset, Encode, FrontCamera, HeadCamera, Intrinsics, RenderDepth  # noqa: E402

Root = Path(__file__).resolve().parent
Artifacts = Root / "TestArtifacts"
Width, Height = 1280, 720
TableZ = SC.DefaultTableZCm
Tape = np.array([-0.9, 0.1, 0.6])  # the tape centre's offset in the gripper frame, like the real runs' fits
_DeskCache: dict[int, np.ndarray] = {}


def Poses(Model, Count=22, Seed=3, AngleNoiseDeg=0.1):
    """Reachable arm poses whose tape the camera sees: reported display joints and the true tape pixel and point."""
    Controller = Bridge.controller
    Rng = np.random.default_rng(Seed)
    Limits = {Name: sorted((Controller._ik_angle_from_raw(Servo, Servo.range_min), Controller._ik_angle_from_raw(Servo, Servo.range_max)))
              for Name, Servo in Controller.servos.items()}
    Current = Controller._current_ik_angles()
    Rows = []
    for _ in range(6000):
        Angles = {Name: float(Rng.uniform(*Limits[Name])) if Name in Bridge.IK_JOINTS else Current[Name] for Name in Bridge.MOTOR_NAMES}
        Joints = np.array([[P["x"], P["y"], P["z"]] for P in Geo.forward_joint_positions(Controller.arm_geometry, Angles)])
        Tip, Frame = SC.GripperFrame(Joints * DG.MirrorVector + DisplayOffset)
        if Tip[2] < TableZ + 2.0 or Tip[1] < 8.0:
            continue
        Camera = DG.DisplayToCamera(Tip + Frame @ Tape, Model)[0]
        Pixel = DG.Project(Camera, Intrinsics)[0]
        if Camera[2] <= 5 or not (60 < Pixel[0] < Width - 60 and 50 < Pixel[1] < Height - 50):
            continue
        if any(np.linalg.norm(Pixel - Row["pixel"]) < 12 for Row in Rows):
            continue  # keep the tape patches apart, like distinct poses
        Noisy = {Name: Value + Rng.normal(0, AngleNoiseDeg) for Name, Value in Angles.items()}
        Reported = np.array([[P["x"], P["y"], P["z"]] for P in Geo.forward_joint_positions(Controller.arm_geometry, Noisy)])
        Display = Reported * DG.MirrorVector + DisplayOffset
        Rows.append({"joints": [{"x": float(X), "y": float(Y), "z": float(Z)} for X, Y, Z in Display], "pixel": Pixel, "camera": Camera})
        if len(Rows) == Count:
            break
    return Rows


def Frame(Pixel, Mirrored, Distractor=None):
    """A capture: grey desk, a light gripper patch with the red tape centred on Pixel (native), maybe mirrored."""
    Image = np.full((Height, Width, 3), 90, np.uint8)
    U, V = int(round(Pixel[0])), int(round(Pixel[1]))
    cv2.rectangle(Image, (U - 40, V - 30), (U + 40, V + 30), (205, 205, 205), -1)
    cv2.ellipse(Image, (U, V), (10, 7), 0, 0, 360, (40, 40, 220), -1)
    if Distractor is not None:
        X, Y = Distractor
        cv2.rectangle(Image, (X - 45, Y - 35), (X + 45, Y + 35), (205, 205, 205), -1)
        cv2.circle(Image, (X, Y), 16, (30, 30, 230), -1)
    return Encode(cv2.flip(Image, 1) if Mirrored else Image)


def Depth(Model, Row):
    """The desk the camera sees, with the tape patch at its true depth (native, mm)."""
    Key = id(Model)
    if Key not in _DeskCache:
        _DeskCache[Key] = RenderDepth(Model, [])
    Image = _DeskCache[Key].copy()
    cv2.circle(Image, (int(round(Row["pixel"][0])), int(round(Row["pixel"][1]))), 9, int(round(Row["camera"][2] * 10)), -1)
    return Image


def CaptureClick(Pixel, Mirrored, Noise=(0.0, 0.0)):
    """What the app posts for a click on native pixel Pixel: the clicked whole pixel's centre in capture space."""
    Column, Row = math.floor(Pixel[0] + 0.5 + Noise[0]), math.floor(Pixel[1] + 0.5 + Noise[1])  # native centre n sits at n + 0.5
    CaptureColumn = Width - 1 - Column if Mirrored else Column
    return CaptureColumn + 0.5, Row + 0.5


class ClickCalibrationTests(unittest.TestCase):
    def setUp(Self):
        Self.Patches = [patch.object(SC, "SessionPath", Artifacts / "ClickSession.json"),
                        patch.object(SC, "CapturesDir", Artifacts / "ClickCaptures")]
        for Patch in Self.Patches:
            Patch.start()
        SC._RedTracker.clear()

    def tearDown(Self):
        for Patch in Self.Patches:
            Patch.stop()
        SC._Session.clear()
        SC._Session.update({"image_size": None, "mirrored": None, "observations": []})
        SC._RedTracker.clear()

    def Record(Self, Model, Mirrored, Count=22, Seed=3, Frames=2, Distractor=None):
        """A new run with every pose observed as /selfcal/observe does; returns the poses with their archives."""
        SC.ResetSession()
        SC._RedTracker.clear()
        Rows = Poses(Model, Count, Seed)
        Self.assertGreaterEqual(len(Rows), Count, "not enough visible poses")
        for Index, Row in enumerate(Rows):
            Metadata = {"capture_valid": True, "pose_index": Index, "frame_metadata": [
                {"frame_index": K, "captured_at_epoch_ms": 1e12 + K, "joints": Row["joints"],
                 "servos": {"wrist_roll": {"presentRaw": 2048}}} for K in range(Frames)]}
            Result = SC.AddObservation([Frame(Row["pixel"], Mirrored, Distractor)] * Frames, Row["joints"], Mirrored, Index,
                                       Method="red", Metadata=Metadata, DepthFrames=[Depth(Model, Row)] * Frames,
                                       DepthIntrinsics=dict(Intrinsics))
            Self.assertTrue(Result["ok"], Result)
            Row["archive"] = Result["capture_archive"]
        return Rows

    def Click(Self, Row, Index, Mirrored, Noise=(0.0, 0.0), FrameIndex=1):
        U, V = CaptureClick(Row["pixel"], Mirrored, Noise)
        return Clicks.AddClick({"pose_index": Index, "capture_archive": Row["archive"], "frame_index": FrameIndex,
                                "u": U, "v": V, "mirrored": Mirrored, "image_size": [Width, Height]})

    def AssertRecovered(Self, Report, Truth, Clean=True):
        Self.assertTrue(Report["ok"], Report.get("error"))
        if Clean:
            Self.assertTrue(Report["quality_ok"], Report["problems"])
        Self.assertEqual(Report["method"], "depth")
        Self.assertEqual(Report["observation_source"], "manual")
        Self.assertEqual(Report["model"]["observation_source"], "manual")
        Self.assertEqual(Report["model"]["click_target"], "tape_centre")
        TrueEye = DG.CameraToDisplay(np.zeros(3), Truth)[0]
        Self.assertLess(float(np.linalg.norm(np.array(Report["camera_position_cm"]) - TrueEye)), 0.8, Report["camera_position_cm"])
        Self.assertLess(float(np.linalg.norm(np.array(Report["tape_offset_cm"]) - Tape)), 0.6, Report["tape_offset_cm"])
        # The drift baseline is set as an automatic run sets it, with the clicked blobs' tape areas.
        Self.assertTrue(Report["drift_baseline"], Report.get("error"))
        Self.assertIsNotNone(Report["model"]["drift_reference"]["tape_area_cm2"])

    def TestNoisyClicksRecoverTheCamera(Self):
        for Name, Truth, Mirrored in (("head, unmirrored", HeadCamera, False), ("front, mirrored", FrontCamera, True)):
            with Self.subTest(camera=Name):
                Rows = Self.Record(Truth, Mirrored)
                Rng = np.random.default_rng(11)
                for Index, Row in enumerate(Rows):
                    Result = Self.Click(Row, Index, Mirrored, tuple(Rng.normal(0, 1.0, 2)))
                    Self.assertTrue(Result["ok"], Result)
                    Self.assertIsNotNone(Result["depth_cm"], Result)
                    Self.assertIsNotNone(Result["red_blob"], "a click on the tape names its blob")
                Report = SC.Solve(TableZ=TableZ, Save=False, Source="manual")
                Self.AssertRecovered(Report, Truth)
                Self.assertEqual(Report["tape_assignment"]["clicked"], len(Rows))
                Self.assertEqual(len(Report["per_pose"]), len(Rows))
                Self.assertEqual([Row["pose_index"] for Row in Report["per_pose"]], list(range(len(Rows))))
                Self.assertFalse(any(Row["flagged"] for Row in Report["per_pose"]), Report["per_pose"])

    def TestClickIsStoredInNativePixelsWithItsFramesPose(Self):
        for Mirrored in (False, True):
            with Self.subTest(mirrored=Mirrored):
                Rows = Self.Record(HeadCamera, Mirrored, Count=12, Frames=3)
                Row = Rows[4]
                Result = Self.Click(Row, 4, Mirrored, FrameIndex=2)
                Self.assertTrue(Result["ok"], Result)
                Column, Line = math.floor(Row["pixel"][0] + 0.5), math.floor(Row["pixel"][1] + 0.5)
                Self.assertEqual(Result["native_px"], [float(Column), float(Line)], "the clicked pixel, in native OpenCV pixels")
                Record = SC._Session["manual_clicks"]["4"]
                Self.assertEqual(Record["frame_index"], 2)
                Self.assertEqual(Record["joint_source"], "frame_metadata")
                Tip, _ = SC.GripperFrame(Row["joints"])
                Self.assertTrue(np.allclose(Record["tip"], Tip))
                # Depth comes from that frame's archived depth at the native pixel: the tape patch, not the desk.
                Self.assertAlmostEqual(Result["depth_cm"], Row["camera"][2], delta=0.15)
                Self.assertLess(float(np.linalg.norm(np.array(Result["camera_cm"]) - Row["camera"])), 0.3)
                Self.assertTrue(Result["red_blob"]["inside"])
                Self.assertLess(Result["red_blob"]["click_to_centroid_px"], 2.0)
                # The archive keeps the click, and the session file survives a reload.
                Records = list((Path(SC._Session["archive_directory"]) / "records").glob("*_ManualClick_*.json"))
                Self.assertEqual(len(Records), 1)
                Saved = json.loads((Path(SC._Session["archive_directory"]) / "Session.json").read_text(encoding="utf-8"))
                Self.assertEqual(Saved["manual_clicks"]["4"]["corners"], [[float(Column), float(Line)]])

    def TestOutlierClicksAreFlaggedAndARedoClearsThem(Self):
        Rows = Self.Record(HeadCamera, False)
        Wrong = {3: (26.0, -18.0), 15: (-30.0, 12.0)}
        for Index, Row in enumerate(Rows):
            Self.assertTrue(Self.Click(Row, Index, False, Wrong.get(Index, (0.0, 0.0)))["ok"])
        Report = SC.Solve(TableZ=TableZ, Save=False, Source="manual")
        Flagged = {Row["pose_index"] for Row in Report["per_pose"] if Row["flagged"]}
        Self.assertEqual(Flagged, set(Wrong), Report["per_pose"])
        Self.assertTrue(all(Row["residual_px"] > 15 for Row in Report["per_pose"] if Row["pose_index"] in Wrong))
        # The solver drops them, so the camera is still right; re-clicking them clears the flags.
        Self.AssertRecovered(Report, HeadCamera)
        for Index in Wrong:
            Result = Self.Click(Rows[Index], Index, False)
            Self.assertTrue(Result["ok"] and Result["replaced"], Result)
        Report = SC.Solve(TableZ=TableZ, Save=False, Source="manual")
        Self.AssertRecovered(Report, HeadCamera)
        Self.assertFalse(any(Row["flagged"] for Row in Report["per_pose"]), Report["per_pose"])
        Self.assertEqual(Report["observations_dropped"], 0)

    def TestSkippedPosesAndTheMinimum(Self):
        Rows = Self.Record(HeadCamera, False)
        for Index, Row in enumerate(Rows):
            if Index % 3 == 0:
                Self.assertTrue(Clicks.SkipPose({"pose_index": Index, "reason": "tape hidden by the jaw"})["ok"])
            else:
                Self.assertTrue(Self.Click(Row, Index, False)["ok"])
        Report = SC.Solve(TableZ=TableZ, Save=False, Source="manual")
        Self.AssertRecovered(Report, HeadCamera)
        Self.assertEqual(Report["tape_assignment"]["skipped"], 8)
        Self.assertEqual(Report["observations_used"], 14)
        # Fewer clicks than recommended: still fitted and kept, with the count as a warning (only the user rejects).
        for Index in (1, 2, 4, 5, 7):
            Self.assertTrue(Clicks.SkipPose({"pose_index": Index})["ok"])
        Report = SC.Solve(TableZ=TableZ, Save=False, Source="manual")
        Self.AssertRecovered(Report, HeadCamera, Clean=False)
        Self.assertEqual(Report["observations_used"], 9)
        Self.assertIn("only 9 clicked poses could be used (10 or more recommended)", Report["problems"])
        Self.assertEqual(Report["model"]["quality_problems"], Report["problems"])
        # Too few to compute a camera at all: nothing is fitted and the current calibration stays.
        for Index in (8, 10, 11, 13):
            Self.assertTrue(Clicks.SkipPose({"pose_index": Index})["ok"])
        Report = SC.Solve(TableZ=TableZ, Save=False, Source="manual")
        Self.assertEqual((Report["ok"], Report["computed"], Report["saved"]), (False, False, False))
        Self.assertIn("Could not compute a calibration: only 5 poses have a click; at least 6 are needed", Report["error"])
        Self.assertIn("The current calibration is still in use.", Report["error"])
        Self.assertNotIn("model", Report)
        # Skipping a clicked pose removes its click; clicking a skipped pose un-skips it.
        Self.assertTrue(Self.Click(Rows[0], 0, False)["ok"])
        Self.assertNotIn(0, SC._Session["manual_skips"])

    def TestClicksWithoutDepthAreKeptWithWarnings(Self):
        # With the depth camera on, a manual fit whose clicks mostly miss depth used to be refused. It is
        # kept now: with 8 of 14 the depth fit uses them and warns; with 5 (too few for the depth fit) the
        # camera is fitted from colour alone, without the drift baseline, and the warning says what that
        # means for the camera check.
        Rows = Self.Record(HeadCamera, False, Count=14)
        for Index, Row in enumerate(Rows):
            Self.assertTrue(Self.Click(Row, Index, False)["ok"])
        for Index in range(8, 14):
            with SC._Lock:
                SC._Session["manual_clicks"][str(Index)].update(camera_cm=None, depth_cm=None)
        Report = SC.Solve(TableZ=TableZ, Save=False, Source="manual")
        Self.AssertRecovered(Report, HeadCamera, Clean=False)
        Self.assertTrue(any(Problem.startswith("only 8 of 14 clicks have depth under them (10 or more recommended)")
                            for Problem in Report["problems"]), Report["problems"])
        for Index in range(5, 8):
            with SC._Lock:
                SC._Session["manual_clicks"][str(Index)].update(camera_cm=None, depth_cm=None)
        Report = SC.Solve(TableZ=TableZ, Save=False, Source="manual")
        Self.assertTrue(Report["ok"] and Report["computed"], Report.get("error"))
        Self.assertNotEqual(Report.get("method"), "depth")
        Self.assertFalse(Report["drift_baseline"])
        Problem = next(Problem for Problem in Report["problems"] if Problem.startswith("only 5 of 14 clicks have depth under them"))
        Self.assertIn("fitted from colour alone", Problem)
        Self.assertIn("every task that acts on an object stops (camera not verified)", Problem)
        Self.assertIn("model", Report)

    def TestUndoReplaceAndSkipBookkeeping(Self):
        Rows = Self.Record(HeadCamera, False, Count=12)
        Self.assertFalse(Self.Click(Rows[2], 2, False)["replaced"])
        Self.assertFalse(Self.Click(Rows[5], 5, False)["replaced"])
        Again = Self.Click(Rows[2], 2, False, (3.0, 0.0))
        Self.assertTrue(Again["replaced"])
        Self.assertEqual(Again["clicks"], 2)
        Self.assertEqual(SC._Session["manual_click_order"], [5, 2], "a re-click is the newest click")
        Undone = Clicks.RemoveClick({})
        Self.assertEqual((Undone["pose_index"], Undone["clicks"]), (2, 1))
        Self.assertEqual(Clicks.RemoveClick({"pose_index": 5})["pose_index"], 5)
        Self.assertEqual(Clicks.RemoveClick({})["removed"], False)
        Skip = Clicks.SkipPose({"pose_index": 7})
        Self.assertEqual((Skip["clicks"], Skip["skipped"]), (0, 1))
        Status = Clicks.Status()
        Self.assertEqual((Status["clicks"], Status["skipped"], Status["target"]), ([], [7], "tape_centre"))

    def TestBadClicksAreRefused(Self):
        Rows = Self.Record(HeadCamera, False, Count=10, Frames=1)
        Row = Rows[0]
        U, V = CaptureClick(Row["pixel"], False)
        Good = {"pose_index": 0, "capture_archive": Row["archive"], "frame_index": 0, "u": U, "v": V, "mirrored": False,
                "image_size": [Width, Height]}
        Outside = Artifacts / "NotAnObservation.json"
        Outside.write_text(json.dumps({"pose_index": 0}), encoding="utf-8")
        for Name, Change, Expected in (
                ("archive outside the run", {"capture_archive": str(Outside)}, "not an observation of the current calibration run"),
                ("relative escape", {"capture_archive": str(Path(Row["archive"]).parent / ".." / "Manifest.json")}, "not an observation"),
                ("missing archive", {"capture_archive": None}, "capture_archive must name"),
                ("wrong pose", {"pose_index": 1}, "belongs to pose 0"),
                ("mirroring", {"mirrored": True}, "mirroring differs"),
                ("frame index", {"frame_index": 3}, "frame 3 does not exist"),
                ("off the frame", {"u": Width + 5.0}, "outside the 1280x720 frame"),
                ("size", {"image_size": [640, 360]}, "archived frame is 1280x720"),
                ("not a number", {"v": "12"}, "v must be a finite number"),
                ("infinite", {"u": float("inf")}, "u must be a finite number"),
                ("negative pose", {"pose_index": -1}, "pose_index must be a whole number")):
            with Self.subTest(case=Name):
                Result = Clicks.AddClick({**Good, **Change})
                Self.assertFalse(Result["ok"], Result)
                Self.assertIn(Expected, Result["error"])
        Self.assertEqual(SC._Session.get("manual_clicks", {}), {}, "no refused click is stored")
        # A capture the app marked unstill cannot be clicked.
        Archive = json.loads(Path(Row["archive"]).read_text(encoding="utf-8"))
        Archive["metadata"]["capture_valid"] = False
        Path(Row["archive"]).write_text(json.dumps(Archive), encoding="utf-8")
        Self.assertIn("retake the pose", Clicks.AddClick(Good)["error"])
        # Without a run, nothing is recorded.
        SC._Session.pop("archive_directory")
        Self.assertIn("No calibration run is recording", Clicks.AddClick(Good)["error"])

    def TestClicksBeatADistractorTheBlobSearchMayPick(Self):
        # A larger red object sits in view at every pose; the clicks only ever name the tape.
        Rows = Self.Record(HeadCamera, False, Distractor=(1180, 640))
        for Index, Row in enumerate(Rows):
            Result = Self.Click(Row, Index, False)
            Self.assertTrue(Result["ok"], Result)
            Self.assertLess(Result["red_blob"]["click_to_centroid_px"], 3.0)
        Self.AssertRecovered(SC.Solve(TableZ=TableZ, Save=False, Source="manual"), HeadCamera)


class BridgeClickEndpointTests(unittest.TestCase):
    """The bridge's click endpoints and the manual solve, called as FastAPI calls them."""

    def setUp(Self):
        Self.Patches = [patch.object(SC, "SessionPath", Artifacts / "ClickBridgeSession.json"),
                        patch.object(SC, "CapturesDir", Artifacts / "ClickCaptures"),
                        patch.object(Bridge.calibration, "save_camera_model",
                                     return_value={"replaced_archive": None, "replaced_fitted_at": None, "can_reject": True})]
        Self.Mocks = [Patch.start() for Patch in Self.Patches]
        SC._RedTracker.clear()

    def tearDown(Self):
        for Patch in Self.Patches:
            Patch.stop()
        SC._Session.clear()
        SC._Session.update({"image_size": None, "mirrored": None, "observations": []})
        SC._RedTracker.clear()

    def TestEndpointsRecordClicksAndSaveEveryComputedFit(Self):
        Helper = ClickCalibrationTests()
        Rows = Helper.Record(HeadCamera, False, Count=16, Frames=1)
        Save = Self.Mocks[2]
        Status = Bridge.SelfCalClickStatus()
        Self.assertEqual((Status["ok"], Status["clicks"], Status["version"]), (True, [], 1))

        def ClickRows(Start, Stop):
            for Index, Row in enumerate(Rows[Start:Stop], start=Start):
                U, V = CaptureClick(Row["pixel"], False)
                Self.assertTrue(Bridge.SelfCalClick({"pose_index": Index, "capture_archive": Row["archive"], "frame_index": 0,
                                                     "u": U, "v": V, "mirrored": False})["ok"])

        # Too few clicks to compute a camera: nothing is saved, the current calibration stays.
        ClickRows(0, 5)
        Report = Bridge.selfcal_solve({"source": "manual"})
        Self.assertEqual((Report["ok"], Report["computed"], Report["saved"]), (False, False, False))
        Self.assertIn("Could not compute a calibration: only 5 poses have a click", Report["error"])
        Save.assert_not_called()
        # Nine clicks, fewer than recommended: saved and applied, the count a warning in the reply and the model.
        ClickRows(5, 9)
        Report = Bridge.selfcal_solve({"source": "manual"})
        Self.assertEqual((Report["ok"], Report["computed"], Report["saved"], Report["quality_ok"]), (True, True, True, False))
        Self.assertIn("only 9 clicked poses could be used (10 or more recommended)", Report["problems"])
        Self.assertTrue(Report["can_reject"])
        Save.assert_called_once()
        Self.assertEqual(Save.call_args[0][0]["quality_problems"], Report["problems"])
        Self.assertEqual(Save.call_args[0][1], SC._Session["archive_directory"], "the save names its run, where the old file is archived")
        ClickRows(9, 16)
        Self.assertEqual(Bridge.SelfCalClickSkip({"pose_index": 15})["clicks"], 15)
        Self.assertEqual(Bridge.SelfCalClickUndo({})["pose_index"], 14)
        Self.assertFalse(Bridge.SelfCalClick({"pose_index": "x"})["ok"])
        Self.assertFalse(Bridge.selfcal_solve({"source": "sideways"})["ok"])
        Report = Bridge.selfcal_solve({"source": "manual"})
        Self.assertTrue(Report["ok"] and Report["quality_ok"], Report.get("problems"))
        Self.assertTrue(Report["saved"])
        Self.assertNotIn("model", Report, "the model stays on the bridge")
        Self.assertEqual(Save.call_count, 2)
        Saved = Save.call_args[0][0]
        Self.assertEqual((Saved["observation_source"], Saved["observations_used"], Saved["quality_problems"]), ("manual", 14, []))
        Self.assertTrue(CD.VerifiedReference(Saved)[0] is not None, "the saved model carries an intact drift baseline")
        Kinds = sorted(Path(Name).name.split("_")[1] for Name in (Path(SC._Session["archive_directory"]) / "records").glob("*.json"))
        for Kind in ("ManualClick", "ManualSkip", "ManualUndo", "SolveRequest", "FitReport", "CameraSaved"):
            Self.assertIn(Kind, Kinds)


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestSelfCalClicks` runs the Test* methods too."""
    Prefixed = unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return unittest.TestSuite(Prefixed.loadTestsFromTestCase(Case) for Case in (ClickCalibrationTests, BridgeClickEndpointTests))


if __name__ == "__main__":
    Started = time.time()
    Loader = unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Suite = unittest.TestSuite(Loader.loadTestsFromTestCase(Case) for Case in (ClickCalibrationTests, BridgeClickEndpointTests))
    Result = unittest.TextTestRunner(verbosity=2).run(Suite)
    print(f"{time.time() - Started:.0f} s")
    raise SystemExit(not Result.wasSuccessful())
