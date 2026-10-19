"""A camera calibration is never rejected automatically (Aaran, 2026-10-10): only the user rejects one.

Every run that computes a camera is saved and applied through the bridge's /selfcal/solve, each
former rejection reason a warning with its numbers in the reply, in the saved model
(quality_problems) and in /calibration/status. The user's Reject calibration (/selfcal/reject)
restores the calibration it replaced from the run's archive (a CalibrationReplaced record) and
records the rejection (UserRejected). A run that computes no camera keeps the current calibration,
and says so. Calibration runs are synthetic (TestDepthCamera's rendered desk and tape); the real
camera_calibration.json is never written: calibration.CALIBRATION_PATH points at a scratch copy.
No camera, arm, serial port or server is opened.

Run:  python TestCalibrationNeverRejected.py      (or python -m unittest TestCalibrationNeverRejected)
"""
import hashlib
import json
import math
import os
import time
import unittest
import uuid
from pathlib import Path
from unittest.mock import patch

import numpy as np

os.environ.setdefault("SO101_CAPTURES_ROOT", str(Path(__file__).resolve().parent / "TestArtifacts" / "NRC"))

import bridge_server as Bridge  # noqa: E402
import calibration  # noqa: E402
import CameraDrift as CD  # noqa: E402
import DepthGeometry as DG  # noqa: E402
import SelfCalibration as SC  # noqa: E402
import SessionLog  # noqa: E402
import TestDepthCamera as TDC  # noqa: E402

Root = Path(__file__).resolve().parent
# Short folder names: a git worktree's run archives sit near Windows' 260-character path limit.
Artifacts = Root / "TestArtifacts" / "NR"
RealFile = Root / "camera_calibration.json"
Tape = np.array([1.2, -0.6, 0.8])


def Digest(File: Path) -> str | None:
    return hashlib.sha256(File.read_bytes()).hexdigest() if File.is_file() else None


def Records(Kind: str) -> list[dict]:
    Folder = Path(SC._Session["archive_directory"]) / "records"
    return [json.loads(File.read_text(encoding="utf-8")) for File in sorted(Folder.glob(f"*_{Kind}_*.json"))]


def SingleTiltRows():
    """TestDepthCamera's one-wrist-pitch poses: the camera height and the tape offset trade off exactly."""
    Rng, Rows = np.random.default_rng(5), []
    for Tip, Frame, _, _ in TDC.CalibrationPoses(TDC.HeadCamera, Tape, Count=40):
        Yaw = Frame[:, 0].copy(); Yaw[2] = 0.0
        if np.linalg.norm(Yaw) < 1e-6:
            continue
        Yaw /= np.linalg.norm(Yaw)
        X = np.append(Yaw[:2] * math.cos(math.radians(-12)), math.sin(math.radians(-12)))
        Y = np.cross([0.0, 0.0, 1.0], Yaw); Y /= np.linalg.norm(Y)
        Level = np.column_stack([X, Y, np.cross(X, Y)])
        Camera = DG.DisplayToCamera(Tip + Level @ Tape, TDC.HeadCamera)[0]
        Pixel = DG.Project(Camera, TDC.Intrinsics)[0]
        if Camera[2] > 5 and 40 < Pixel[0] < TDC.Width - 40 and 40 < Pixel[1] < TDC.Height - 40:
            Rows.append((Tip, Level, Pixel + Rng.normal(0, 0.5, 2), Camera + Rng.normal(0, 0.2, 3)))
    return Rows[:22]


class NeverRejectedTests(unittest.TestCase):
    # TestDepthCamera's session builder: rendered desk planes per pose, the tape's pixel and depth point.
    Inject = TDC.DepthCalibrationTests.Inject

    def setUp(Self):
        Artifacts.mkdir(parents=True, exist_ok=True)
        Self.RealDigest = Digest(RealFile)
        Self.File = Artifacts / f"camera_calibration.{uuid.uuid4().hex[:8]}.json"
        if RealFile.is_file():  # the calibration in use before the run: a copy of this checkout's
            Self.File.write_bytes(RealFile.read_bytes())
        Self.Patches = [patch.object(calibration, "CALIBRATION_PATH", Self.File),
                        patch.object(SC, "SessionPath", Artifacts / "Session.json"),
                        patch.object(SC, "CapturesDir", Artifacts / "C")]
        for Item in Self.Patches:
            Item.start()
        SC._RedTracker.clear()
        SC.ResetSession({"test": Self.id()})
        Self.Rows = TDC.CalibrationPoses(TDC.HeadCamera, Tape)
        Self.assertGreaterEqual(len(Self.Rows), 15)

    def tearDown(Self):
        for Item in Self.Patches:
            Item.stop()
        SC._Session.clear()
        SC._Session.update({"image_size": None, "mirrored": None, "observations": []})
        SC._RedTracker.clear()
        Self.assertEqual(Digest(RealFile), Self.RealDigest, "the real camera_calibration.json was never written")

    def Solve(Self):
        """The bridge's /selfcal/solve, as the app calls it after a run."""
        return Bridge.selfcal_solve({"table_z": TDC.TableZ})

    def Saved(Self):
        return json.loads(Self.File.read_text(encoding="utf-8"))

    def TestEachFormerRejectionIsSavedAndAppliedWithItsWarning(Self):
        Rows = Self.Rows

        def DeskAtThreePoses():
            Self.Inject(Rows, Desk=TDC.HeadCamera)
            with SC._Lock:
                SC._Session["desk_planes"] = [dict(Plane, frame=Frame) for Plane in SC._Session["desk_planes"][:3] for Frame in range(3)]

        Cases = (
            ("camera moved during the fit", lambda: Self.Inject(Rows, Desk=TDC.HeadCamera, DeskTiltDeg=0.3),
             "the camera moved 0.2", True),
            ("desk never found in depth: no baseline", lambda: Self.Inject(Rows),
             "the desk plane was found in depth at only 0 of 0 frames", False),
            ("camera stillness not verifiable", DeskAtThreePoses,
             "camera stillness could not be verified: the desk was found in depth at only 3 poses", True),
            ("one gripper tilt", lambda: Self.Inject(SingleTiltRows(), Desk=TDC.HeadCamera),
             "the gripper tilt barely varied", None),
            ("depth disagrees (wrong depth scale)", lambda: Self.Inject(TDC.CalibrationPoses(TDC.FrontCamera, np.array([1.0, 0.0, 0.5])),
                                                                         DepthScale=1.08, Desk=TDC.FrontCamera), "exceeds", None),
            ("fewer poses than recommended", lambda: Self.Inject(Rows[:8], Desk=TDC.HeadCamera),
             "only 8 poses had the tip tape in view (10 or more recommended)", True),
        )
        for Name, Prepare, Expected, Baseline in Cases:
            with Self.subTest(case=Name):
                Before = Self.Saved().get("fitted_at") if Self.File.is_file() else None
                time.sleep(1.05)  # fitted_at has one-second resolution: each case's calibration is its own
                Prepare()
                Report = Self.Solve()
                Self.assertEqual((Report["ok"], Report["computed"], Report["saved"], Report["quality_ok"]), (True, True, True, False),
                                 Report.get("error") or Report.get("problems"))
                Self.assertTrue(any(Expected in Problem for Problem in Report["problems"]), Report["problems"])
                Self.assertIn("Reject calibration", Report["quality_warning"])
                Self.assertTrue(Report["can_reject"])
                Self.assertEqual(Report["replaced_fitted_at"], Before)
                Self.assertNotIn("model", Report)
                if Baseline is not None:
                    Self.assertEqual(Report["drift_baseline"], Baseline)
                # Applied: the bridge's camera is this fit, its warnings saved with it and in the status.
                Model = calibration.camera_model()
                Self.assertEqual((Model["fitted_at"], Model["quality_problems"]), (Report["fitted_at"], Report["problems"]))
                Status = calibration.status()
                Self.assertEqual((Status["mode"], Status["fitted_at"], Status["quality_problems"], Status["can_reject"]),
                                 ("auto", Report["fitted_at"], Report["problems"], True))
                Self.assertEqual(Status["drift_baseline"], Report["drift_baseline"])
                Saved = Records("CameraSaved")[-1]["payload"]
                Self.assertEqual(Saved["quality_problems"], Report["problems"], "the session record keeps the warnings")
        # No baseline: the camera check itself is unchanged, and blocks every object task as unverified.
        Self.Inject(Rows)
        time.sleep(1.05)
        Report = Self.Solve()
        Self.assertFalse(Report["drift_baseline"])
        Problem = next(Problem for Problem in Report["problems"] if "desk plane was found" in Problem)
        Self.assertIn("so /locate cannot verify the camera: every task that acts on an object stops (camera not verified)", Problem)
        Check = CD.CheckCapture(TDC.RenderDepth(TDC.HeadCamera, []), TDC.Intrinsics, calibration.camera_model())
        Self.assertEqual((Check["ok"], Check["error_code"]), (False, "camera_not_verified"))

    def TestRejectRestoresThePreviousCalibration(Self):
        Original = Self.Saved() if Self.File.is_file() else None
        Self.Inject(Self.Rows, Desk=TDC.HeadCamera)
        First = Self.Solve()
        Self.assertTrue(First["saved"] and First["quality_ok"], First.get("problems"))
        Replaced = Records("CalibrationReplaced")
        Self.assertEqual(len(Replaced), 1)
        Self.assertEqual(Replaced[0]["payload"]["previous_calibration"], Original, "the file it replaced is archived whole")
        time.sleep(1.05)
        Self.Inject(Self.Rows, Desk=TDC.HeadCamera, DeskTiltDeg=0.3)
        Second = Self.Solve()
        Self.assertTrue(Second["saved"] and not Second["quality_ok"], Second.get("error"))
        Self.assertEqual(Second["replaced_fitted_at"], First["fitted_at"])
        Self.assertNotEqual(Second["fitted_at"], First["fitted_at"])
        # A reject of a calibration the user was not shown changes nothing.
        Stale = Bridge.RejectCalibration({"fitted_at": First["fitted_at"]})
        Self.assertFalse(Stale["ok"])
        Self.assertIn("changed since it was shown", Stale["error"])
        Self.assertEqual(calibration.camera_model()["fitted_at"], Second["fitted_at"])
        Self.assertFalse(Bridge.RejectCalibration({"fitted_at": 12})["ok"])
        # The user's reject: the first calibration is in use again, the rejection archived in the run.
        Result = Bridge.RejectCalibration({"fitted_at": Second["fitted_at"], "reason": "rejected in the test"})
        Self.assertTrue(Result["ok"], Result.get("error"))
        Self.assertEqual((Result["rejected_fitted_at"], Result["restored_fitted_at"], Result["restored_mode"], Result["calibrated"],
                          Result["drift_baseline"]), (Second["fitted_at"], First["fitted_at"], "auto", True, True))
        Saved = Self.Saved()
        Self.assertEqual(Saved["camera_model"]["fitted_at"], First["fitted_at"])
        Self.assertEqual(Saved["camera_model"]["quality_problems"], [])
        Self.assertTrue(CD.VerifiedReference(Saved["camera_model"])[0] is not None, "the restored baseline is intact")
        Self.assertEqual((Saved["last_rejected"]["fitted_at"], Saved["last_rejected"]["reason"]), (Second["fitted_at"], "rejected in the test"))
        Rejected = Records("UserRejected")
        Self.assertEqual(len(Rejected), 1)
        Self.assertEqual(Rejected[0]["payload"]["rejected_calibration"]["camera_model"]["fitted_at"], Second["fitted_at"])
        Self.assertEqual(Rejected[0]["payload"]["reason"], "rejected in the test")
        Self.assertEqual(Path(Saved["last_rejected"]["archive"]).parent, Path(Rejected[0]["payload"]["restored_from"]).parent,
                         "the rejection is recorded in the run, beside the record it restored")
        Status = calibration.status()
        Self.assertEqual((Status["fitted_at"], Status["can_reject"], Status["last_rejected"]["fitted_at"]),
                         (First["fitted_at"], True, Second["fitted_at"]))
        # The restored calibration can be rejected in turn: the file from before the run comes back.
        Result = Bridge.RejectCalibration({"fitted_at": First["fitted_at"]})
        Self.assertTrue(Result["ok"], Result.get("error"))
        Saved = Self.Saved()
        Self.assertEqual(Saved["last_rejected"]["fitted_at"], First["fitted_at"])
        Restored = {Key: Value for Key, Value in Saved.items() if Key != "last_rejected"}
        Expected = calibration._default_state()
        Expected.update(Original or {})
        Expected.pop("last_rejected", None)
        Restored.pop("last_rejected", None)
        Self.assertEqual(Restored, {Key: Value for Key, Value in Expected.items() if Key != "last_rejected"})
        Self.assertEqual(Result["restored_fitted_at"], (Original or {}).get("fitted_at"))
        # That file was saved before calibrations could be rejected: it names nothing to restore.
        if not (Original or {}).get("replaced"):
            Again = Bridge.RejectCalibration({})
            Self.assertFalse(Again["ok"])
            Self.assertIn("names no archived calibration to restore", Again["error"])
            Self.assertFalse(calibration.status()["can_reject"])
        Self.assertEqual(Bridge.calibration_status()["fitted_at"], (Original or {}).get("fitted_at"))

    def TestRejectingTheFirstCalibrationLeavesNone(Self):
        Self.File.unlink()
        Model = {"type": "pinhole_depth_v1", "fitted_at": "2026-10-10T12:00:00", "table_z": TDC.TableZ, "quality_problems": ["x"]}
        Saved = calibration.save_camera_model(Model)  # no calibration run: archived under captures/<date>/calibration_changes
        Self.assertTrue(Saved["can_reject"])
        Self.assertTrue(Path(Saved["replaced_archive"]).is_relative_to(SessionLog.CapturesRoot()), Saved["replaced_archive"])
        Self.assertTrue(calibration.status()["calibrated"])
        Result = calibration.reject_camera_model("2026-10-10T12:00:00", "test")
        Self.assertTrue(Result["ok"], Result.get("error"))
        Self.assertEqual((Result["calibrated"], Result["restored_mode"], Result["restored_fitted_at"]), (False, None, None))
        Status = calibration.status()
        Self.assertEqual((Status["calibrated"], Status["mode"], Status["last_rejected"]["fitted_at"]), (False, None, "2026-10-10T12:00:00"))
        Self.assertIsNone(calibration.camera_model())
        Self.assertFalse(calibration.reject_camera_model()["ok"], "nothing left to reject")

    def TestNothingComputedKeepsTheCurrentCalibration(Self):
        Before = Digest(Self.File)

        def AssertKept(Report, Text):
            Self.assertEqual((Report["ok"], Report["computed"], Report["saved"]), (False, False, False), Report)
            Self.assertTrue(Report["error"].startswith("Could not compute a calibration: "), Report["error"])
            Self.assertIn(Text, Report["error"])
            Self.assertIn("The current calibration is still in use.", Report["error"])
            Self.assertEqual(Digest(Self.File), Before, "the calibration in use is untouched")
            Self.assertEqual(Records("CalibrationReplaced"), [])

        Self.Inject(Self.Rows[:4], Desk=TDC.HeadCamera)
        AssertKept(Self.Solve(), "only 4 poses had red in view; at least 6 are needed to compute a camera")
        # Poses on one line: no 3-D camera can be fitted.
        with SC._Lock:
            SC._Session.update(image_size=[1280, 720], mirrored=False, red_poses=[], depth_intrinsics=None, observations=[
                {"pose_index": Index, "tip": [float(Index), 20.0, 5.0], "frame": np.eye(3).tolist(), "corners": [[100.0 + 30 * Index, 300.0]],
                 "method": "red"} for Index in range(12)])
        AssertKept(Self.Solve(), "the tip poses do not span three dimensions")
        # A blob that never moved was not the tip.
        with SC._Lock:
            SC._Session["observations"] = [{"pose_index": Index, "tip": [Index, Index % 3, Index % 5], "frame": np.eye(3).tolist(),
                                            "corners": [[200.0, 200.0]], "method": "red"} for Index in range(16)]
        AssertKept(Self.Solve(), "the detected blob stayed stationary")
        # A solver crash computes nothing either.
        with patch.object(SC, "Solve", side_effect=RuntimeError("boom")):
            AssertKept(Self.Solve(), "the solver failed (RuntimeError: boom)")

    def TestACalibrationThatCannotBeArchivedIsNotSavedOverTheOldOne(Self):
        # Saving first archives the file it replaces; when that fails, the old file stays (it would be lost).
        Before = Digest(Self.File)
        Self.Inject(Self.Rows, Desk=TDC.HeadCamera)
        with patch.object(calibration, "_record_change", side_effect=OSError("disk full")):
            Report = Self.Solve()
        Self.assertEqual((Report["ok"], Report["computed"], Report["saved"]), (False, True, False))
        Self.assertIn("computed but could not be saved (OSError: disk full)", Report["error"])
        Self.assertEqual(Digest(Self.File), Before)


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestCalibrationNeverRejected` runs the Test* methods too."""
    Prefixed = unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return Prefixed.loadTestsFromTestCase(NeverRejectedTests)


if __name__ == "__main__":
    Started = time.time()
    Loader = unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Result = unittest.TextTestRunner(verbosity=2).run(Loader.loadTestsFromTestCase(NeverRejectedTests))
    print(f"{time.time() - Started:.0f} s")
    raise SystemExit(not Result.wasSuccessful())
