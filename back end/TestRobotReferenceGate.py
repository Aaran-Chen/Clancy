"""CheckCapture's order: the desk path first and unchanged; the robot decides only with the calibrated desk out of view.

With the desk out of view the camera is assumed where it was calibrated and the gripper tape checks it
(RobotReference.Check): verified passes, moved blocks, not judgeable passes as "assumed" with a warning.
The basket rim is advisory there and never blocks. With the desk in view the robot's reading only rides
along (RobotReference.ShadowKey). The desk path and the tape are patched here where their own results are
beside the point; TestRobotReference.py checks the tape on real frames. No hardware.

Run:  python TestRobotReferenceGate.py
"""
import unittest
from unittest.mock import patch

import numpy as np

import CameraDrift
import RobotCameraReference
import RobotReference

Model = {"fitted_at": "2026-10-02T21:09:22"}
Reference = {"fitted_at": "2026-10-02T21:09:22", "tape_pose": {"wrist_roll_raw": 1570, "gripper_raw": 704}}
Depth = np.zeros((4, 4), np.uint16)
Image = np.zeros((4, 4, 3), np.uint8)
Intrinsics = {"fx": 600.0, "fy": 600.0, "cx": 2.0, "cy": 2.0, "width": 4, "height": 4}
Arm = {"ok": True, "joints": [[0.0, 0.0, 0.0], [0.0, 10.0, 10.0]], "servos_raw": {"wrist_roll": 1420, "gripper": 699}}


def Desk(Ok=False, Code="camera_check_failed", Reason="fewer than 200 cells of the calibrated desk are visible as desk", **Fields):
    Result = {"ok": Ok, "angle_deg": None, "offset_cm": None, "reason": "desk matches the calibration" if Ok else Reason,
              "reference_fitted_at": "2026-10-02T21:09:22", "seq": 7, **Fields}
    if not Ok:
        Result.update(error_code=Code, error=f"desk path: {Code}")
    return Result


Tapes = {"ok": {"tape_check": "ok", "tape_error_cm": 0.52, "tape_model": "roll_aware"},
         "moved": {"tape_check": "moved", "tape_error_cm": 2.66, "tape_model": "roll_aware"},
         "not_visible": {"tape_check": "not_visible", "tape_error_cm": None, "tape_reason": "no tape-sized red blob near where the arm puts the tape"},
         "pose_differs": {"tape_check": "pose_differs", "tape_error_cm": None, "tape_reason": "gripper differs from the calibration's by 300 steps"},
         "arm moving": {"tape_check": "unavailable", "tape_error_cm": None, "tape_reason": "arm moving"},
         "torque off": {"tape_check": "unavailable", "tape_error_cm": None, "tape_reason": "torque off"},
         "ambiguous": {"tape_check": "ambiguous", "tape_error_cm": 0.6, "tape_reason": "more than one red blob near the predicted tape"},
         "no offset": {"tape_check": "unavailable", "tape_error_cm": None, "tape_reason": "the calibration has no tape offset on the gripper"}}
RimMoved = {"rim_check": "moved", "rim_yaw_deg": -3.1, "rim_shift_cm": [0.41, 0.63], "rim_bins": 24, "rim_agreeing_bins": 18}


class GateOrderTests(unittest.TestCase):
    def setUp(Self):
        Self.Patches = [patch.object(CameraDrift, "VerifiedReference", return_value=(Reference, ""))]
        for Patch in Self.Patches:
            Patch.start()

    def tearDown(Self):
        for Patch in reversed(Self.Patches):
            Patch.stop()

    def Capture(Self, DeskResult, Absent, Tape="ok", Rim=None, Depth_=Depth):
        with patch.object(CameraDrift, "_DeskCapture", return_value=(dict(DeskResult), Absent)), \
                patch.object(RobotReference, "TapeCheck", return_value=dict(Tapes[Tape])), \
                patch.object(RobotReference, "RimCheck", return_value=dict(Rim or {"rim_check": "ok", "rim_yaw_deg": 0.2})):
            return CameraDrift.CheckCapture(Depth_, Intrinsics, Model, 7, Image, Arm)

    def TestDeskInViewDecidesAndTheRobotOnlyRidesAlong(Self):
        # Whatever the robot reads, a decided desk result stands key for key; the robot's reading is beside it.
        for Name, Result, Tape in (("desk passes, tape moved", Desk(Ok=True), "moved"),
                                   ("desk moved, tape ok", Desk(Code="camera_moved", Reason="desk plane changed by 0.80 deg"), "ok"),
                                   ("cluttered desk", Desk(Reason="only 52% of the calibrated desk in view agrees"), "ok"),
                                   ("desk moved, tape not judged", Desk(Code="desk_changed_unverified", Reason="the desk moved"), "ok")):
            with Self.subTest(Name):
                Checked = Self.Capture(Result, False, Tape)
                Self.assertEqual({Key: Value for Key, Value in Checked.items() if Key != RobotReference.ShadowKey}, Result)
                Self.assertEqual(Checked[RobotReference.ShadowKey]["tape_check"], Tapes[Tape]["tape_check"])
                Self.assertNotIn("alignment_source", Checked)

    def TestAMovedCameraFromTheDeskPathStandsWithTheDeskOutOfView(Self):
        # The desk out of view but the cart-move path's tape (at the calibration's roll) said moved: it stays moved.
        Moved = Desk(Code="camera_moved", Reason="gripper tape is 1.20 cm from where the arm puts it (limit 0.75 cm)")
        Checked = Self.Capture(Moved, True, "ok")
        Self.assertEqual({Key: Value for Key, Value in Checked.items() if Key != RobotReference.ShadowKey}, Moved)

    def TestDeskOutOfViewTheTipTapeDecides(Self):
        Failed = Desk()
        Verified = Self.Capture(Failed, True, "ok")
        Self.assertEqual((Verified["ok"], Verified["alignment_source"], Verified["robot_check"]), (True, "robot", "verified"), Verified)
        Self.assertNotIn("error_code", Verified)
        Self.assertEqual(Verified["desk_check"], Failed, "the desk path's result rides along unchanged")
        Self.assertIn("0.52 cm from where the arm puts it", Verified["reason"])
        Self.assertEqual((Verified["seq"], Verified["reference_fitted_at"]), (7, "2026-10-02T21:09:22"))
        Moved = Self.Capture(Failed, True, "moved")
        Self.assertEqual((Moved["ok"], Moved["error_code"], Moved["robot_check"]), (False, "camera_moved", "moved"))
        Self.assertEqual(Moved["error"], "The gripper tip is 2.7 cm from where the calibration at 21:09 puts it, so the camera moved "
                                         "on the robot: recalibrate (Automatic calibration).")
        # The no-plane-at-all failure is out of view too.
        NoPlane = Desk(Reason="no desk plane within 35 deg of the calibrated desk, 30-70 cm away")
        Self.assertEqual(Self.Capture(NoPlane, True, "ok")["robot_check"], "verified")
        # So is a cart-move block whose plane was the basket's own paper (2026-10-06 15:01).
        Beyond = Desk(Reason="fewer than 200 cells ...; the desk moved 6.16 deg / -1.58 cm against the camera since calibration, more than moving the cart does")
        Self.assertEqual(Self.Capture(Beyond, True, "ok")["robot_check"], "verified")
        Unverified = Desk(Code="desk_changed_unverified", Reason="the desk moved 2.0 deg, and the gripper tape could not check the camera")
        Self.assertEqual(Self.Capture(Unverified, True, "moved")["error_code"], "camera_moved")

    def TestAnUnjudgeableTipPassesAsAssumedWithAWarning(Self):
        for Tape, Why in (("not_visible", "is not seen where the arm puts it"), ("pose_differs", "the jaw is not where it was at calibration"),
                          ("arm moving", "the arm was moving around the capture"), ("torque off", "the arm's torque is off"),
                          ("ambiguous", "more than one red thing near the gripper"),
                          ("no offset", "the calibration has no tape offset on the gripper")):
            with Self.subTest(Tape):
                Checked = Self.Capture(Desk(), True, Tape)
                Self.assertEqual((Checked["ok"], Checked["robot_check"], Checked["alignment_source"]), (True, "assumed", "robot"), Checked)
                Self.assertNotIn("error_code", Checked)
                Self.assertTrue(Checked["warning"].startswith("Camera assumed where it was calibrated at 21:09: the gripper tip could not be checked ("),
                                Checked["warning"])
                Self.assertIn(Why, Checked["warning"])
                Self.assertEqual(Checked["warning"].count("\n"), 0)

    def TestTheRimNeverBlocksAndIsNeverRequired(Self):
        Verified = Self.Capture(Desk(), True, "ok", Rim=RimMoved)
        Self.assertEqual((Verified["ok"], Verified["robot_check"], Verified["rim_check"], Verified["rim_yaw_deg"]), (True, "verified", "moved", -3.1))
        Assumed = Self.Capture(Desk(), True, "not_visible", Rim=RimMoved)
        Self.assertEqual((Assumed["ok"], Assumed["robot_check"]), (True, "assumed"))
        Missing = Self.Capture(Desk(), True, "ok", Rim={"rim_check": "unavailable", "rim_reason": "no calibration frames of the rim to compare with"})
        Self.assertEqual((Missing["ok"], Missing["robot_check"]), (True, "verified"))
        # A rim keyframe match that fails outright (RobotCameraReference) is reported, never raised.
        with patch.object(RobotCameraReference, "Check", side_effect=RuntimeError("keyframe unreadable")), \
                patch.object(RobotReference, "RimBaseline", return_value=None):
            Rim = RobotReference.RimCheck(Image, Depth, Intrinsics, Model, Arm)
        Self.assertEqual(Rim["rim_check"], "unavailable")
        Self.assertIn("keyframe unreadable", Rim["rim_reason"])

    def TestAFaultInTheTipCheckFailsClosed(Self):
        # A fault in the check itself is no verdict on the camera: blocked, never passed as assumed.
        with patch.object(CameraDrift, "_DeskCapture", return_value=(Desk(), True)), \
                patch.object(RobotReference, "TapeCheck", side_effect=RuntimeError("detector crashed")), \
                patch.object(RobotReference, "RimCheck", return_value={"rim_check": "ok"}):
            Checked = CameraDrift.CheckCapture(Depth, Intrinsics, Model, 7, Image, Arm)
        Self.assertEqual((Checked["ok"], Checked["error_code"], Checked["robot_check"]), (False, "camera_check_failed", "failed"), Checked)
        Self.assertIn("detector crashed", Checked["reason"])
        Self.assertNotIn("warning", Checked)
        Self.assertEqual(CameraDrift.Verdict(Checked), "camera_check_failed")

    def TestAFailingShadowNeverStopsACaptureTheDeskPassed(Self):
        # A fault in the tip check, or in the robot's reading altogether: the desk's pass stands, the shadow says so.
        Passed = Desk(Ok=True)
        for Name, Target, Verdict in (("tip check", "TapeCheck", "failed"), ("robot reading", "Check", "unavailable")):
            with Self.subTest(Name), patch.object(CameraDrift, "_DeskCapture", return_value=(dict(Passed), False)), \
                    patch.object(RobotReference, Target, side_effect=RuntimeError("detector crashed")), \
                    patch.object(RobotReference, "RimCheck", return_value={"rim_check": "ok"}):
                Checked = CameraDrift.CheckCapture(Depth, Intrinsics, Model, 7, Image, Arm)
                Self.assertEqual({Key: Value for Key, Value in Checked.items() if Key != RobotReference.ShadowKey}, Passed)
                Self.assertEqual(Checked[RobotReference.ShadowKey]["robot_check"], Verdict)

    def TestUnverifiedOrUnmeasuredCapturesKeepTheirBlocks(Self):
        # No calibration baseline, or no depth: not a desk out of view, and nothing to assume against.
        for Result in (Desk(Code="camera_not_verified", Reason="the calibration has no camera-position baseline"),
                       Desk(Reason="no depth frame for this capture")):
            with Self.subTest(Result["reason"]):
                Checked = Self.Capture(Result, False, "ok", Depth_=None)
                Self.assertEqual(Checked, Result)

    def TestVerdictNamesEachOutcome(Self):
        Self.assertEqual(CameraDrift.Verdict(Desk(Ok=True)), "desk_verified")
        Self.assertEqual(CameraDrift.Verdict(dict(Desk(Ok=True), desk_remeasured=True)), "desk_remeasured")
        Self.assertEqual(CameraDrift.Verdict(Self.Capture(Desk(), True, "ok")), "verified")
        Self.assertEqual(CameraDrift.Verdict(Self.Capture(Desk(), True, "not_visible")), "assumed")
        Self.assertEqual(CameraDrift.Verdict(Self.Capture(Desk(), True, "moved")), "camera_moved")
        Self.assertEqual(CameraDrift.Verdict(Desk(Code="camera_not_verified")), "camera_not_verified")


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestRobotReferenceGate` runs the Test* methods too (unittest names this hook)."""
    Prefixed = unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return Prefixed.loadTestsFromTestCase(GateOrderTests)


if __name__ == "__main__":
    Loader = unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Result = unittest.TextTestRunner(verbosity=2).run(Loader.loadTestsFromTestCase(GateOrderTests))
    raise SystemExit(not Result.wasSuccessful())
