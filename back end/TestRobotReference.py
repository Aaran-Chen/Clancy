"""The roll-aware gripper tape, and the camera check with no calibrated desk in view (RobotReference). No hardware.

- One tape offset in the gripper link reproduces the calibration's pseudo-frame tape at all 24 poses of the
  2026-10-02 21:09 calibration (about 0.002 cm), their raw steps converted by the bridge's own kinematics.
- The two no-desk captures of 2026-10-06 replay (from the recordings: RecordedCaptures; not on disk, they fail,
  RECORDING MISSING), with the 2026-10-02 21:09 calibration they were taken with (its tracked copy, so a new
  calibration does not skip them): 15:01:01Z, the camera where it was calibrated, is
  verified (the tape 0.52 cm off at wrist roll 1420, where the roll rule refused it); 23:01:47Z, after the
  15:02:25 hand on the lens turned the head 4.16 deg, blocks as camera_moved (2.66 cm). The 15:01 frame
  re-rendered as the camera drooped or turned on the robot blocks too, a 6 deg droop included (the tape's
  height moved 2.6 cm there, past the desk path's height filter).
- An unjudgeable tip passes as assumed with a warning: tape covered, arm moving, torque off, jaw opened.
- Synthetic scenes (TestDepthCamera's ray-caster, no desk in view): the prediction turns with the wrist roll;
  red specks are not the tape; a tape-sized red blob beside the tip is taken for it, as at the desk.

Run:  python TestRobotReference.py
"""
import json
import math
import os
import time
import unittest
from pathlib import Path

import cv2
import numpy as np

# Records go to the test captures root, never the real one; no declared surfaces.
os.environ["SO101_CAPTURES_ROOT"] = str(Path(__file__).resolve().parent / "TestArtifacts" / "SessionCaptures")
os.environ["SO101_DECLARED_SURFACES"] = str(Path(__file__).resolve().parent / "TestArtifacts" / "NoDeclaredSurfaces.json")

import TestDepthCamera as TDC  # noqa: E402  (the ray-caster and camera models)
from TestCartMove import TurnedFrame  # noqa: E402  (a real frame as a camera turned about its centre saw it)
import bridge_server as Bridge  # noqa: E402
import CameraDrift as CD  # noqa: E402
import DepthGeometry as DG  # noqa: E402
import RecordedCaptures as RC  # noqa: E402
import RobotReference as RR  # noqa: E402
import SelfCalibration as SC  # noqa: E402

Root = Path(__file__).resolve().parent
Names = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
# The 2026-10-02 21:09 calibration: its tape offset (pseudo gripper frame) and its 24 manual-click poses
# (raw steps, as Session.json of run 20261002_210623_5e44514af6d7 stored them, with the tips it computed).
Q1002 = np.array([-0.641375300301567, -0.4925752823905805, -0.3821964662074297])
DesignLinkOffset = np.array([-1.214, -0.422, -9.134])  # .claude/NoDeskDesign.md, fitted over the 24 poses
CalibrationPoses = [
    ((3494, 121, 2006, 563, 1570, 704), (-14.536, 31.928, 14.354)), ((3350, 155, 1969, 563, 1570, 704), (-7.411, 32.052, 14.447)),
    ((3350, 296, 1905, 486, 1570, 704), (-7.104, 33.978, 12.446)), ((3411, 274, 1935, 482, 1570, 704), (-10.335, 33.948, 12.309)),
    ((3492, 274, 1937, 482, 1570, 704), (-14.564, 34.026, 12.223)), ((3492, 219, 2107, 361, 1570, 704), (-14.443, 32.003, 10.484)),
    ((3492, 498, 1813, 154, 1570, 704), (-14.745, 37.056, 14.208)), ((3431, 500, 1809, 154, 1570, 704), (-11.275, 37.074, 14.271)),
    ((3363, 583, 1784, 93, 1570, 704), (-7.362, 37.125, 12.417)), ((3196, 575, 1798, 84, 1570, 704), (1.779, 34.236, 12.487)),
    ((3123, 575, 1798, 84, 1570, 704), (5.511, 32.336, 12.487)), ((3123, 727, 1777, 4055, 1570, 704), (5.491, 32.301, 8.325)),
    ((3033, 753, 1722, 4072, 1570, 704), (10.155, 29.937, 8.638)), ((2947, 745, 1743, 4069, 1570, 704), (13.719, 26.418, 8.33)),
    ((2844, 765, 1702, 4084, 1570, 704), (17.93, 22.127, 8.476)), ((2861, 747, 1621, 93, 1570, 704), (18.279, 23.664, 10.15)),
    ((2952, 727, 1644, 86, 1570, 704), (14.508, 27.651, 10.493)), ((2952, 609, 1634, 212, 1570, 704), (14.524, 27.667, 14.416)),
    ((3025, 554, 1727, 184, 1570, 704), (10.615, 29.768, 14.135)), ((2842, 562, 1709, 184, 1570, 704), (18.071, 22.084, 14.468)),
    ((2927, 594, 1765, 101, 1570, 704), (14.553, 25.611, 12.417)), ((2921, 607, 1864, 4083, 1570, 704), (13.804, 24.429, 10.548)),
    ((3006, 609, 1864, 4083, 1570, 704), (10.398, 27.746, 10.445)), ((3085, 582, 1914, 4058, 1570, 704), (6.496, 29.817, 10.5))]
# The arm at Home through both 2026-10-06 captures (the last recorded route before each), wrist roll 1420.
Home1501 = {"shoulder_pan": 3487, "shoulder_lift": 3173, "elbow_flex": 2686, "wrist_flex": 864, "wrist_roll": 1420, "gripper": 699}
Home2301 = {"shoulder_pan": 3476, "shoulder_lift": 3172, "elbow_flex": 2686, "wrist_flex": 864, "wrist_roll": 1420, "gripper": 699}


def ArmFromRaw(Raw, ReferenceRaw=None):
    """CaptureArmPose for raw servo steps, by the bridge's own conversion (a fresh, unconnected controller):
    joints, gripper_link and, with ReferenceRaw, the same pose there."""
    Controller = Bridge.BridgeController()

    def Frames(Values):
        for Name, Value in Values.items():
            Controller.servos[Name].present_raw, Controller.servos[Name].gui_angle_raw = int(Value), None
        return Controller.ArmFrames(Controller._current_ik_angles())

    Joints, Link = Frames(Raw)
    Pose = {"ok": True, "joints": Joints, "servos_raw": dict(Raw), "gripper_link": Link}
    if ReferenceRaw:
        Pinned = {**Raw, **ReferenceRaw}
        PinnedJoints, PinnedLink = Frames(Pinned)
        Pose["reference"] = {"servos_raw": Pinned, "joints": PinnedJoints, "gripper_link": PinnedLink}
    return Pose


def Predicted(Arm, Offset):
    Link = np.asarray(Arm["gripper_link"])
    return Link[:3, :3] @ Offset + Link[:3, 3]


def PseudoFrameTape(Joints, Q):
    Tip, Frame = SC.GripperFrame(np.asarray(Joints, dtype=np.float64))
    return Tip + Frame @ Q


class LinkOffsetTests(unittest.TestCase):
    """The tape rigid on the gripper link, from the calibration's offset in the pseudo gripper frame."""

    def TestOneLinkOffsetReproducesTheCalibrationPoses(Self):
        Arms = [ArmFromRaw(dict(zip(Names, Raw))) for Raw, _ in CalibrationPoses]
        # The bridge converts these raw steps as the calibration did (its stored tips, to 3 decimals).
        for Arm, (_, Tip) in zip(Arms, CalibrationPoses):
            Self.assertLess(float(np.linalg.norm(np.asarray(Arm["joints"][-1]) - Tip)), 0.002)
        # One offset for all 24 (least squares): gripper_link * q against the pseudo-frame tape.
        Rotations = np.vstack([np.asarray(Arm["gripper_link"])[:3, :3] for Arm in Arms])
        Targets = np.concatenate([PseudoFrameTape(Arm["joints"], Q1002) - np.asarray(Arm["gripper_link"])[:3, 3] for Arm in Arms])
        Fitted = np.linalg.lstsq(Rotations, Targets, rcond=None)[0]
        Self.assertLess(float(np.linalg.norm(Fitted - DesignLinkOffset)), 0.01, Fitted)
        Errors = np.array([np.linalg.norm(Predicted(Arm, Fitted) - PseudoFrameTape(Arm["joints"], Q1002)) for Arm in Arms])
        Self.assertLess(float(np.sqrt(np.mean(Errors ** 2))), 0.003, Errors)
        Self.assertLess(float(Errors.max()), 0.006, Errors)
        # The check takes it at each capture's own pose (RobotReference.LinkOffset): from any one of the 24
        # it reproduces all of them within 0.01 cm, and at its own pose exactly.
        for Arm in Arms:
            Offset = RR.LinkOffset(Q1002, Arm)
            Self.assertLess(float(np.linalg.norm(Offset - Fitted)), 0.01)
            Self.assertLess(float(np.linalg.norm(Predicted(Arm, Offset) - PseudoFrameTape(Arm["joints"], Q1002))), 1e-9)
            Self.assertLess(max(float(np.linalg.norm(Predicted(Other, Offset) - PseudoFrameTape(Other["joints"], Q1002))) for Other in Arms), 0.01)
        print(f"  24 calibration poses: link offset {np.round(Fitted, 3)}, rms {np.sqrt(np.mean(Errors ** 2)):.4f} cm, max {Errors.max():.4f} cm")

    def TestTheTapeTurnsWithTheWristRoll(Self):
        # At the calibration's roll the roll-aware tape is the pseudo-frame tape; at 1420 (13 deg off) it moved
        # 0.29 cm, on a circle about the roll axis, 0.14 cm from where the pseudo frame put it there.
        Model = dict(RC.CalibrationOf20261002(), marker_corners_gripper_cm=[Q1002.tolist()])
        AtCalibration = ArmFromRaw(dict(Home1501, wrist_roll=1570), {"wrist_roll": 1570})
        Offset = RR.LinkOffset(Q1002, AtCalibration["reference"])
        Self.assertLess(float(np.linalg.norm(Predicted(AtCalibration, Offset) - PseudoFrameTape(AtCalibration["joints"], Q1002))), 1e-9)
        Rolled = ArmFromRaw(Home1501, {"wrist_roll": 1570})
        Self.assertLess(float(np.linalg.norm(RR.LinkOffset(Q1002, Rolled["reference"]) - Offset)), 1e-9, "the offset does not depend on the roll")
        Turned = Predicted(Rolled, Offset)
        Self.assertAlmostEqual(float(np.linalg.norm(Turned - Predicted(AtCalibration, Offset))), 0.29, delta=0.02)
        Self.assertAlmostEqual(float(np.linalg.norm(Turned - PseudoFrameTape(Rolled["joints"], Q1002))), 0.14, delta=0.02)
        Link = np.asarray(Rolled["gripper_link"])
        Axis, Through = Link[:3, 2] / np.linalg.norm(Link[:3, 2]), Link[:3, 3]

        def FromAxis(Point):
            Arm_ = Point - Through
            return float(np.linalg.norm(Arm_ - (Arm_ @ Axis) * Axis))
        Self.assertAlmostEqual(FromAxis(Turned), FromAxis(Predicted(AtCalibration, Offset)), places=6)
        Tape = RR.TapeCheck(np.zeros((720, 1280, 3), np.uint8), np.zeros((720, 1280), np.uint16), TDC.Intrinsics, Model, Rolled)
        Self.assertAlmostEqual(Tape["tape_roll_change_deg"], 150 * 360 / 4096, delta=0.05)
        Self.assertEqual(Tape["tape_wrist_roll_raw"], 1420)

    def TestTheBridgeReportsTheGripperLinkAndTheCalibrationRoll(Self):
        # CaptureArmPose's fields, from a controller on a fake bus (TestSafety's): the gripper link sits on the
        # roll joint, and the reference pose differs from the capture's only in the wrist roll and the tip.
        from TestSafety import Beat, MakeController
        Controller, _ = MakeController()
        Beat(Controller, 0.3)
        Captured = time.time() - 0.05
        Plain = Controller.CaptureArmPose(Captured)
        Self.assertTrue(Plain["ok"], Plain)
        Self.assertNotIn("reference", Plain)
        Link = np.asarray(Plain["gripper_link"])
        Self.assertEqual(Link.shape, (4, 4))
        # The arm's link boxes at the same angles (the measured scene leaves the arm out with them), and its collar's revolution.
        Self.assertEqual(Plain["link_boxes"], Controller.ArmLinkBoxes(Controller._current_ik_angles()))
        Self.assertEqual(len(Plain["link_boxes"]), len(Bridge.Collision.Model["ArmBoxes"]) + 3)
        Self.assertLess(float(np.linalg.norm(Link[:3, 3] - np.asarray(Plain["joints"][-2]))), 1e-9)
        Roll = Plain["servos_raw"]["wrist_roll"]
        Pinned = Controller.CaptureArmPose(Captured, {"wrist_roll": Roll + 150})
        Reference = Pinned["reference"]
        Self.assertEqual(Reference["servos_raw"], {**Plain["servos_raw"], "wrist_roll": Roll + 150})
        Self.assertEqual(Pinned["joints"], Plain["joints"])
        Self.assertEqual(Reference["joints"][:-1], Plain["joints"][:-1])
        Self.assertGreater(float(np.linalg.norm(np.subtract(Reference["joints"][-1], Plain["joints"][-1]))), 0.05)
        Turn = np.linalg.solve(np.asarray(Reference["gripper_link"])[:3, :3], Link[:3, :3])
        Self.assertAlmostEqual(math.degrees(math.acos((np.trace(Turn) - 1) / 2)), 150 * 360 / 4096, delta=0.01)
        Same = Controller.CaptureArmPose(Captured, {"wrist_roll": Roll})["reference"]
        Self.assertEqual((Same["joints"], Same["gripper_link"]), (Plain["joints"], Plain["gripper_link"]))
        # A pose not judged (the arm moving, torque off) carries no frames at all.
        Controller.torque_enabled = False
        Self.assertEqual(Controller.CaptureArmPose(Captured, {"wrist_roll": Roll}), {"ok": False, "reason": "torque off"})


# --------------------------------------------------------------------------
# 2026-10-06: the cart at the bookshelf cabinet, no desk in view
# --------------------------------------------------------------------------
Day = RC.Day("2026-10-06")
# (clock, depth seq, raw depth in the session record, the archived capture, the arm)
Cabinet = {"15:01": (516698, "c1e55020492ffe59b3a27f09a5f24af9e3b1491e", "080101_173_vision.jpg", Home1501),
           "23:01": (948941, "d6842ca82afb31b6d412689a519aee91d2a0bea5", "160147_210_vision.jpg", Home2301)}
Model1002 = RC.CalibrationOf20261002()


class CabinetTests(unittest.TestCase):
    """The two recorded no-desk captures of 2026-10-06, replayed."""

    @classmethod
    def setUpClass(Class):
        RC.Need(Day / "recording" / "grounding.jsonl", *(Path_ for _, Sha, Jpg, _ in Cabinet.values()
                                                        for Path_ in (Day / "recording" / "depth" / f"{Sha}.png", Day / Jpg)),
                What="the 2026-10-06 cabinet captures")
        Class.Model = Model1002
        Class.Camera = {Key: Class.Model[Key] for Key in ("fx", "fy", "cx", "cy", "dist")} | {"width": 1280, "height": 720}
        Class.Frames = {Clock: (cv2.imread(str(Day / "recording" / "depth" / f"{Sha}.png"), cv2.IMREAD_UNCHANGED), cv2.imread(str(Day / Jpg)), Seq,
                                ArmFromRaw(Raw, RR.ReferenceRaw(Class.Model)))
                        for Clock, (Seq, Sha, Jpg, Raw) in Cabinet.items()}
        Class.Recorded = {}
        for Line in (Day / "recording" / "grounding.jsonl").read_text(encoding="utf-8").splitlines():
            Record = json.loads(Line)
            if Record.get("kind") == "locate":
                Class.Recorded[Record["camera_check"]["seq"]] = Record["camera_check"]

    def Check(Self, Clock, Image=None, Arm=None, Depth=None):
        Depth_, Image_, Seq, Arm_ = Self.Frames[Clock]
        return CD.CheckCapture(Depth_ if Depth is None else Depth, Self.Camera, Self.Model, Seq, Image_ if Image is None else Image,
                               Arm_ if Arm is None else Arm)

    def TestTheCaptureRefusedAt1501IsVerified(Self):
        Result = Self.Check("15:01")
        Self.assertEqual((Result["ok"], Result["alignment_source"], Result["robot_check"], Result["tape_check"]),
                         (True, "robot", "verified", "ok"), Result)
        Self.assertAlmostEqual(Result["tape_error_cm"], 0.52, delta=0.05)
        Self.assertEqual(Result["tape_wrist_roll_raw"], 1420)
        Self.assertNotIn("error_code", Result)
        Self.assertNotIn("warning", Result)
        # The desk path's own result rides along: what the live bridge recorded at 15:01:01, key for key.
        Recorded = Self.Recorded[516698]
        Timing = ("check_ms", "tape_ms")
        Self.assertEqual({Key: Value for Key, Value in Result["desk_check"].items() if Key not in Timing},
                         {Key: Value for Key, Value in Recorded.items() if Key not in Timing})
        # The rim (advisory) agrees with the tape here: the camera was where it was calibrated.
        Self.assertEqual(Result["rim_check"], "ok", Result)
        Self.assertLess(abs(Result["rim_yaw_deg"]), RR.RimMaxYawDeg)
        print(f"  15:01: tape {Result['tape_error_cm']} cm, rim {Result['rim_yaw_deg']} deg / {Result['rim_shift_cm']} cm, {Result['check_ms']} ms")

    def TestTheCaptureAfterTheHeadTurnBlocksAsMoved(Self):
        Result = Self.Check("23:01")
        Self.assertEqual((Result["ok"], Result["error_code"], Result["robot_check"], Result["tape_check"]),
                         (False, "camera_moved", "moved", "moved"), Result)
        Self.assertAlmostEqual(Result["tape_error_cm"], 2.66, delta=0.25)
        Self.assertEqual(Result["error"], f"The gripper tip is {Result['tape_error_cm']:.1f} cm from where the calibration at 21:09 puts it, "
                                          "so the camera moved on the robot: recalibrate (Automatic calibration).")
        Self.assertEqual(Result["desk_check"]["error_code"], "camera_check_failed")
        print(f"  23:01: tape {Result['tape_error_cm']} cm, rim {Result['rim_check']}, {Result['check_ms']} ms")

    def TestTheCabinetFrameTurnedOnTheRobotBlocks(Self):
        # The 15:01 frame as a camera drooped (about its x axis) or turned (its y axis) on the robot would have
        # seen it. A 6 deg droop puts the tape 4.1 cm off, 2.6 cm of it in height: past the desk path's
        # height filter, but within TapeNearCm, so it is still the tape, and still a moved camera.
        Depth, Image, _, _ = Self.Frames["15:01"]
        for Axis, Name, Turns in ((0, "droop", (2.0, -2.0, 4.0, -4.0, 6.0, -6.0)), (1, "turn", (2.0, -2.0, 4.0, -4.0, 6.0, -6.0)),
                                  (2, "roll", (5.0, -5.0, 8.0, -8.0))):
            for Degrees in Turns:
                with Self.subTest(motion=Name, deg=Degrees):
                    Turned, Colour = TurnedFrame(Depth, Image, Self.Camera, cv2.Rodrigues(np.eye(3)[Axis] * math.radians(Degrees))[0])
                    Result = Self.Check("15:01", Image=Colour, Depth=Turned)
                    Self.assertEqual((Result["ok"], Result.get("error_code"), Result["tape_check"]), (False, "camera_moved", "moved"), Result)
                    Self.assertGreater(Result["tape_error_cm"], RR.TapeVerifiedCm)

    def TestAnUnjudgeableTipPassesAsAssumed(Self):
        Depth, Image, _, Arm = Self.Frames["15:01"]
        Covered = Image.copy()
        U, V = (int(round(Value)) for Value in Self.Check("15:01")["tape_measured_px"])
        Covered[V - 25:V + 26, U - 25:U + 26] = (200, 200, 200)  # a hand over the tape
        for Name, Frame, Arm_, Why in (("tape covered", Covered, Arm, "not seen where the arm puts it"),
                                       ("arm moving", Image, {"ok": False, "reason": "arm moving"}, "the arm was moving"),
                                       ("torque off", Image, {"ok": False, "reason": "torque off"}, "torque is off"),
                                       ("jaw opened", Image, dict(Arm, servos_raw=dict(Arm["servos_raw"], gripper=899)), "the jaw is not where it was"),
                                       ("older bridge", Image, {Key: Value for Key, Value in Arm.items() if Key != "reference"},
                                        "no gripper frame at the calibration's wrist roll")):
            with Self.subTest(Name):
                Result = Self.Check("15:01", Image=Frame, Arm=Arm_)
                Self.assertEqual((Result["ok"], Result["alignment_source"], Result["robot_check"]), (True, "robot", "assumed"), Result)
                Self.assertNotIn("error_code", Result)
                Self.assertTrue(Result["warning"].startswith("Camera assumed where it was calibrated at 21:09: the gripper tip could not be checked"))
                Self.assertIn(Why, Result["warning"])

    def TestTheDepthCheckEndpointReportsTheVerdictAndSavesNothing(Self):
        # GET /depthcam/check on the 15:01 frame (a fake camera; the arm pose patched in): verified, nothing written.
        from unittest.mock import patch
        Depth, Image, _, Arm = Self.Frames["15:01"]
        Camera = TDC.OC.DepthCamera(Opener=TDC.FakeSource)
        Camera.IntrinsicsValue = dict(Self.Camera)
        for _ in range(TDC.OC.PinMedianFrames):
            Camera.Push(Image, Depth)
        Before = (Root / "camera_calibration.json").read_bytes()
        with patch.object(Bridge, "depth_camera", Camera), patch.object(Bridge.controller, "CaptureArmPose", return_value=Arm) as Pose, \
                patch.object(Bridge.calibration, "camera_model", return_value=Self.Model):
            Reply = Bridge.depthcam_check()
        Self.assertEqual(Pose.call_args.args[1], {"wrist_roll": 1570}, "the pose is taken at the calibration's roll as well")
        Self.assertEqual((Reply["camera_verdict"], Reply["camera_check"]["robot_check"]), ("verified", "verified"), Reply["camera_check"])
        Self.assertEqual((Root / "camera_calibration.json").read_bytes(), Before)


# --------------------------------------------------------------------------
# Synthetic: no desk in view, the tape where a rolled gripper puts it
# --------------------------------------------------------------------------
Tape = np.array([1.2, -0.6, 0.8])
TapePose = {"wrist_roll_raw": 2048, "gripper_raw": 2100}


def SyntheticArm(Joints, Point, RollDeg=0.0):
    """CaptureArmPose's fields for a gripper whose link frame is the pseudo frame at the calibration's roll,
    rolled RollDeg about that frame's own z axis: the tape at Tip + Frame Rz(RollDeg) Tape."""
    Tip, Frame = SC.GripperFrame(Joints)
    Reference = np.eye(4)
    Reference[:3, :3], Reference[:3, 3] = Frame, Tip
    Cos, Sin = math.cos(math.radians(RollDeg)), math.sin(math.radians(RollDeg))
    Rolled = Reference @ np.array([[Cos, -Sin, 0, 0], [Sin, Cos, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])
    Steps = int(round(RollDeg * 4096 / 360))
    return {"ok": True, "joints": Joints.tolist(), "servos_raw": {"wrist_roll": 2048 + Steps, "gripper": 2100}, "gripper_link": Rolled.tolist(),
            "reference": {"servos_raw": {"wrist_roll": 2048, "gripper": 2100}, "joints": Joints.tolist(), "gripper_link": Reference.tolist()}}


def NoDeskView(Model, Points, Specks=(), Plate=60, Size=10):
    """Native image and depth with no desk (the room floor only): a white gripper plate around the first
    point, a tape-sized red square at each of Points and a 5 px red speck at each of Specks."""
    Image = np.full((TDC.Height, TDC.Width, 3), 90, np.uint8)
    Depth = TDC.RenderDepth(Model, [], FloorZ=TDC.RoomFloorZ)
    Pixels = [np.round(DG.Project(DG.DisplayToCamera(P, Model)[0], TDC.Intrinsics)[0]).astype(int) for P in [*Points, *Specks]]
    Centre = DG.DisplayToCamera(Points[0], Model)[0]
    U, V = Pixels[0]
    cv2.rectangle(Image, (U - Plate, V - Plate), (U + Plate, V + Plate), (235, 235, 235), -1)
    Depth[V - Plate:V + Plate + 1, U - Plate:U + Plate + 1] = round(Centre[2] * 10.0)
    for Index, (U, V) in enumerate(Pixels):
        Half = Size if Index < len(Points) else 2
        cv2.rectangle(Image, (U - Half, V - Half), (U + Half, V + Half), (30, 30, 215), -1)
    return Image, Depth


class SyntheticTests(unittest.TestCase):
    @classmethod
    def setUpClass(Class):
        Class.Model = dict(TDC.Verified(TDC.HeadCamera, [TDC.Workspace()[0]], FloorZ=TDC.RoomFloorZ, TapePose=TapePose),
                           marker_corners_gripper_cm=[Tape.tolist()])
        Class.Joints, Class.Point = TDC.ArmInView(TDC.HeadCamera, Tape)
        Tip, Frame = SC.GripperFrame(Class.Joints)
        Class.Tip, Class.Frame = Tip, Frame

    def Check(Self, Image, Depth, Arm):
        return CD.CheckCapture(Depth, TDC.Intrinsics, Self.Model, 5, Image, Arm)

    def TestTheTapeIsPredictedWhereTheRolledGripperPutsIt(Self):
        # Rolled 60 deg: the tape is 1.34 cm from where the pseudo frame (no roll) would put it.
        Turn = cv2.Rodrigues(np.array([0.0, 0.0, math.radians(60.0)]))[0]
        Rolled = Self.Tip + Self.Frame @ Turn @ Tape
        Self.assertAlmostEqual(float(np.linalg.norm(Rolled - Self.Point)), 2 * math.hypot(Tape[0], Tape[1]) * math.sin(math.radians(30)), places=6)
        Arm = SyntheticArm(Self.Joints, Self.Point, 60.0)
        Verified = Self.Check(*NoDeskView(TDC.HeadCamera, [Rolled]), Arm)
        Self.assertEqual((Verified["ok"], Verified["robot_check"], Verified["desk_check"]["error_code"]), (True, "verified", "camera_check_failed"), Verified)
        Self.assertLess(Verified["tape_error_cm"], 0.2)
        Self.assertAlmostEqual(Verified["tape_roll_change_deg"], 60.0, delta=0.01)
        # The tape where the unrolled frame would have it is 1.3 cm from the rolled gripper's: a moved camera.
        Moved = Self.Check(*NoDeskView(TDC.HeadCamera, [Self.Point]), Arm)
        Self.assertEqual((Moved["ok"], Moved["error_code"]), (False, "camera_moved"), Moved)
        Self.assertAlmostEqual(Moved["tape_error_cm"], 1.34, delta=0.2)
        # The desk path's roll rule set aside nothing here: at the calibration's roll both predictions agree.
        Same = Self.Check(*NoDeskView(TDC.HeadCamera, [Self.Point]), SyntheticArm(Self.Joints, Self.Point))
        Self.assertEqual((Same["ok"], Same["robot_check"]), (True, "verified"), Same)

    def TestRedSpecksAreNotTheTape(Self):
        # Specks (0.1 cm2, like the marks on the basket paper) 2 cm from the tape: never the tape.
        Arm = SyntheticArm(Self.Joints, Self.Point)
        Specks = [Self.Point + [2.0, 0.0, 0.0], Self.Point + [0.0, -2.0, 0.5]]
        Seen = Self.Check(*NoDeskView(TDC.HeadCamera, [Self.Point], Specks), Arm)
        Self.assertEqual(Seen["robot_check"], "verified", Seen)
        Self.assertGreaterEqual(Seen["tape_other_blobs"], 1, "a speck the detector saw is left out by its size")
        Image, Depth = NoDeskView(TDC.HeadCamera, [Self.Point], Specks)
        U, V = np.round(DG.Project(DG.DisplayToCamera(Self.Point, TDC.HeadCamera)[0], TDC.Intrinsics)[0]).astype(int)
        Image[V - 10:V + 11, U - 10:U + 11] = 235  # the tape turned away
        Hidden = Self.Check(Image, Depth, Arm)
        Self.assertEqual((Hidden["ok"], Hidden["robot_check"], Hidden["tape_check"]), (True, "assumed", "not_visible"), Hidden)

    def TestATapeSizedRedBlobBesideTheTipIsTakenForTheTape(Self):
        # As at the desk: with the tape hidden, red of the tape's size 3 cm from it is the tape as far as anything
        # can tell, so the capture blocks (the heights are not compared off a desk: a droop moves them).
        Arm = SyntheticArm(Self.Joints, Self.Point)
        Beside = Self.Point + [0.0, 0.0, 3.0]
        Image, Depth = NoDeskView(TDC.HeadCamera, [Self.Point, Beside])
        U, V = np.round(DG.Project(DG.DisplayToCamera(Self.Point, TDC.HeadCamera)[0], TDC.Intrinsics)[0]).astype(int)
        Image[V - 10:V + 11, U - 10:U + 11] = 235
        Result = Self.Check(Image, Depth, Arm)
        Self.assertEqual((Result["ok"], Result["error_code"]), (False, "camera_moved"), Result)

    def TestTheDeskInViewStillDecidesAsBefore(Self):
        # The calibrated desk in view: the desk path's result, the robot's reading only beside it.
        Arm = SyntheticArm(Self.Joints, Self.Point, 60.0)
        Image, Depth = TDC.TapeView(TDC.HeadCamera, Self.Point)
        Result = Self.Check(Image, Depth, dict(Arm, servos_raw={"wrist_roll": 2048, "gripper": 2100}))
        Self.assertTrue(Result["ok"], Result)
        Self.assertNotIn("alignment_source", Result)
        Self.assertEqual(Result["tape_check"], "ok")
        Self.assertEqual(Result[RR.ShadowKey]["robot_check"], "moved", "the rolled gripper's tape is not where the image has it")


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestRobotReference` runs the Test* methods too (unittest names this hook)."""
    Prefixed = unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return unittest.TestSuite(Prefixed.loadTestsFromTestCase(Case) for Case in (LinkOffsetTests, CabinetTests, SyntheticTests))


if __name__ == "__main__":
    Started = time.time()
    Loader = unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Suite = unittest.TestSuite(Loader.loadTestsFromTestCase(Case) for Case in (LinkOffsetTests, CabinetTests, SyntheticTests))
    Result = unittest.TextTestRunner(verbosity=2).run(Suite)
    print(f"{time.time() - Started:.0f} s")
    raise SystemExit(not Result.wasSuccessful())
