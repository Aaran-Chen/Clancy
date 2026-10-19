"""Archive durability and real offline replay checks; no hardware is opened."""
import base64
import copy
import hashlib
import json
from pathlib import Path
import unittest
from unittest.mock import patch

import cv2
import numpy as np
import CalibrationArchive as Archive
import ReplayCalibration
import SelfCalibration as SC
import bridge_server as Bridge

Root = Path(__file__).resolve().parent
Artifacts = Root / "TestArtifacts"


def Encode(Image):
    return "data:image/jpeg;base64," + base64.b64encode(cv2.imencode(".jpg", Image)[1]).decode("ascii")


def HashFiles(Directory):
    return {str(P.relative_to(Directory)): hashlib.sha256(P.read_bytes()).hexdigest()
            for P in Directory.rglob("*") if P.is_file()}


class ArchiveTests(unittest.TestCase):
    def setUp(Self):
        Self.BeforeSession, Self.BeforeTracker = copy.deepcopy(SC._Session), copy.deepcopy(SC._RedTracker)
        Self.SessionPatch = patch.object(SC, "SessionPath", Artifacts / "ArchiveSession.json")
        Self.RootPatch = patch.object(SC, "CapturesDir", Artifacts / "Archive")
        Self.SessionPatch.start()
        Self.RootPatch.start()
        SC._RedTracker.clear()
        Self.Run = Path(SC.ResetSession()["archive_directory"])

    def tearDown(Self):
        Self.RootPatch.stop()
        Self.SessionPatch.stop()
        SC._Session.clear()
        SC._Session.update(Self.BeforeSession)
        SC._RedTracker.clear()
        SC._RedTracker.update(Self.BeforeTracker)

    def TestNewRunDoesNotOverwritePreviousRun(Self):
        SC.RecordEvent("Finish", {"outcome": "cancelled"})
        Before = HashFiles(Self.Run)
        Next = Path(SC.ResetSession()["archive_directory"])
        Self.assertNotEqual(Next, Self.Run)
        Self.assertEqual(HashFiles(Self.Run), Before)

    def TestConfigurationAndSourceCodeArePreserved(Self):
        Context = {"workspace_frame": {"TableZCm": -7.17}, "home_preset": {"elbow_flex": 123}, "initial_state": {"torque": False}}
        Run = Path(SC.ResetSession(Context)["archive_directory"])
        Manifest = json.loads((Run / "Manifest.json").read_text())
        Self.assertEqual(Manifest["context"], Context)
        Self.assertTrue(Manifest["packages"]["numpy"])
        for Name in ("BasketSurface.py", "BasketSurface.ts", "BasketSurface.json", "RobotBodySurface.json",
                     "ArmMotionEnvelope.py", "ArmMotionEnvelope.ts"):
            Self.assertIn(Name, Manifest["sources"])
        for Name, Digest in Manifest["sources"].items():
            Self.assertEqual(hashlib.sha256((Run / "sources" / Name).read_bytes()).hexdigest(), Digest)

    def TestUnstableCaptureAndMissingJointsAreStillArchived(Self):
        Controller = Bridge.BridgeController()
        Payload = {"images": [Encode(np.full((120, 160, 3), 210, np.uint8))], "joints": [],
                   "pose_index": 7, "capture_valid": False, "frame_metadata": [{"captured_at_epoch_ms": 12345}],
                   "requested_pose": {"x": 2, "y": 18, "z": 6}, "mirrored": True}
        with patch.object(Bridge, "controller", Controller):
            Result = Bridge.selfcal_observe(Payload)
        Self.assertFalse(Result["ok"])
        Capture = json.loads(Path(Result["capture_archive"]).read_text())
        Self.assertEqual(Capture["images"], Payload["images"])
        Self.assertEqual(Capture["metadata"]["frame_metadata"], Payload["frame_metadata"])
        Self.assertIn("servos", Capture["metadata"]["bridge_state_at_receive"])
        Self.assertEqual(SC._Session["observations"], [])

    def TestPlanRejectionReportAndFinishAreArchived(Self):
        Controller = Bridge.BridgeController()
        with patch.object(Bridge, "controller", Controller), patch.object(Controller, "selfcal_poses", return_value={"ok": True, "poses": []}):
            Reset = Bridge.selfcal_reset()
            Self.assertTrue(Reset["ok"], Reset)
            Self.assertEqual(Reset["archive_version"], 2)
            Run = Path(Reset["archive_directory"])
            Bridge.selfcal_poses()
            Report = Bridge.selfcal_solve({"table_z": -7.17})
            Self.assertFalse(Report["ok"])
            Bridge.FinishCalibration({"outcome": "rejected"})
        Records = [json.loads(P.read_text()) for P in (Run / "records").glob("*.json")]
        Self.assertTrue({"Plan", "SolveRequest", "FitReport", "Finish"}.issubset({R["kind"] for R in Records}))
        Saved = next(R["payload"] for R in Records if R["kind"] == "FitReport")
        Self.assertEqual(Saved["error"], Report["error"])

    def TestArchiveFailureBlocksMotionBeforeCommand(Self):
        Controller = Bridge.BridgeController()
        with patch.object(Bridge, "controller", Controller), patch.object(SC, "RecordEvent", side_effect=OSError("disk full")), patch.object(Controller, "handle_command") as Command:
            Result = Bridge.MotionCommand({"type": "send_ik", "plan_id": "test-plan"})
        Self.assertFalse(Result["ok"])
        Command.assert_not_called()

    def TestRealOfflineReplayRecoversOriginallyMissedPoses(Self):
        SC.ResetSession({"workspace_frame": {"TableZCm": -7.17}})
        Run = Path(SC._Session["archive_directory"])
        Rng = np.random.default_rng(91)
        Eye = np.array([20., -35., 48.])
        Forward = np.array([0., 20., 1.]) - Eye
        Forward /= np.linalg.norm(Forward)
        Right = np.cross(Forward, [0., 0., 1.])
        Right /= np.linalg.norm(Right)
        Rotation = np.vstack([Right, np.cross(Forward, Right), Forward])
        Rvec = cv2.Rodrigues(Rotation)[0]
        Intrinsic = np.array([[900., 0., 639.5], [0., 900., 359.5], [0., 0., 1.]])
        Hints = {}
        for Index, Tip in enumerate(Rng.uniform([-12, 12, -2], [12, 28, 17], (36, 3))):
            Pan, Pitch = Rng.uniform(-1, 1), Rng.uniform(-1.2, 0.4)
            Radial = np.array([np.sin(Pan), np.cos(Pan), 0.])
            Direction = Radial * np.cos(Pitch) + np.array([0., 0., np.sin(Pitch)])
            Joints = [dict(zip(("x", "y", "z"), Point.tolist())) for Point in (Tip - 20 * Radial, Tip - 2 * Direction, Tip)]
            _, Frame = SC.GripperFrame(Joints)
            Pixel = cv2.projectPoints((Tip + Frame @ np.array([.4, -.6, .8]))[None], Rvec, -Rotation @ Eye, Intrinsic, None)[0].reshape(2)
            Image = np.full((720, 1280, 3), 210, np.uint8)
            cv2.circle(Image, tuple(np.round(Pixel).astype(int)), 11, (0, 20, 240), -1)
            Images = [Encode(cv2.flip(Image, 1))] * 3
            Result = SC.AddObservation(Images, Joints, True, Index, Method="red",
                                       Metadata={"capture_valid": True, "frame_metadata": [{"joints": Joints}] * 3})
            Self.assertTrue(Result["ok"], Result)
            Self.assertFalse(Result["detected"], "Original run deliberately has no selected marker")
            Hints[str(Index)] = {"u": float(Pixel[0]), "v": float(Pixel[1])}
        SourceHashes = HashFiles(Run)
        CameraBytes = (Root / "camera_calibration.json").read_bytes()
        with patch("calibration.save_camera_model", side_effect=AssertionError("Replay must not apply a camera model")):
            Result = ReplayCalibration.Replay(Run, Hints)
        Report = Result["report"]
        Self.assertTrue(Report["ok"], Report.get("error"))
        Self.assertFalse(Report["saved"])
        Self.assertEqual(Report["observations_used"], 36)
        Self.assertEqual(Report["table_z"], -7.17)
        Self.assertEqual(HashFiles(Run), SourceHashes)
        Self.assertEqual((Root / "camera_calibration.json").read_bytes(), CameraBytes)
        Self.assertEqual(SC._Session["observations"], [])
        print("Offline recovery:", Report["observations_used"], "poses; fit", Report["fit_rms_px"], "hold-out", Report["holdout_rms_px"])


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestCalibrationArchive` runs the Test* methods too (unittest names this hook)."""
    Prefixed = unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return Prefixed.loadTestsFromTestCase(ArchiveTests)


if __name__ == "__main__":
    Loader = unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Result = unittest.TextTestRunner(verbosity=2).run(Loader.loadTestsFromTestCase(ArchiveTests))
    raise SystemExit(not Result.wasSuccessful())
