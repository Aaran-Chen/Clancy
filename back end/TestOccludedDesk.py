"""The arm out over the desk is no moved desk, and a mid-task wrist roll no longer refuses the gripper tape (CameraDrift).

On 2026-10-22 at 14:59:01Z, mid-task ("grab the pill bottle", step 2), /locate stopped the task as desk_changed_unverified:
"The desk moved against the arm since calibration at 09:33 (was the cart moved?) ... wrist_roll differs from the
calibration's by 59 steps". The cart had not moved. The visible desk read 0.10 deg / -0.05 cm, but only 63.6% of it agreed
(floor 67%): a binder's sheets and the keyboard covered about a third of it, and the arm out over the desk hid 216 of the
calibrated desk's cells (405 with the edge around it) that it hides none of at Home. CameraDrift's module notes say what
changed. Here:

- the 14:59:01 capture replays (TestFixtures/OccludedDesk-2026-10-22, kept in git, with the calibration as fitted): it
  passes as a desk matching where the arm leaves it in view, and the same frame turned 1-5 deg any way still blocks;
- the arm's exclusion never makes a desk in view absent: the same frame with only a small desk left about the arm
  (200-260 cells seen, under 200 without the arm's) is decided by all of it, so turned 1.5-2.5 deg it blocks as
  camera_moved instead of passing on the robot's assumed tape;
- the share floor still holds when the arm hides nothing (the same frame checked with the arm at Home);
- only the calibrated desk not seen as desk about the arm counts as hidden: the desk still seen in the arm's margin is
  judged by the fit like the rest, so margin cells made to disagree count against it (they used to be credited);
- synthetic (TestDepthCamera's ray-caster): a mat on the desk and the arm over it on a still camera pass, the same scene
  1.5 deg turned blocks, and an arm too small to explain the low share does not open the floor;
- the visible desk must agree on its own (MinOccludedAgreeingShare): the camera 0.6 cm farther from the desk, a mat at
  the old desk height over most of what is visible and the arm hiding the rest read as no change, but under half of
  the desk in view agrees, so it blocks however much the arm hides; the floor is exactly at the constant;
- nor may the bare desk lie beneath the fit (MaxOccludedBeneathShare), and the change read must sit well within the
  limits (MaxOccludedNormalChangeDeg, MaxOccludedOffsetChangeCm): the camera 0.6, 0.7 or 0.9 cm farther and a flat
  0.45-0.8 cm mat over 56-66% of the desk in view win the fit with the mat's top, the bare desk disagreeing beneath it
  and the arm hiding enough to clear the floor had it agreed; each used to pass as desk_occluded and now blocks, while
  the 14:59 replay (8.7% of its desk in view beneath the fit, 0.10 deg / -0.05 cm) still passes;
- items on the mat dilute that share (review of b1edc92): the camera 0.6-0.7 cm farther, a mat at the old desk height and
  a notebook 0.3-0.7 cm proud of it leave the bare desk only 15-21% of the desk in view, so it passed. Two checks now:
  against the session's latest clear view of the desk (CameraDrift.SessionChange; at 14:57:01, kept in the fixture, the
  arm at Home), every desk cell seen in both, item, mat or bare desk, must agree on no change: a real move shifts them
  all, so the 14:59 frame passes against it while the same frame 3 mm farther, which the occluded limits let through,
  blocks; with no such view, the bare desk is judged against what could be desk (agreeing or beneath:
  MaxOccludedBeneathOfDeskShare), 0.19-0.26 for every probe scene against 0.12 at 14:59;
- the cart-move fallback with the wrist rolled mid-task: the roll-aware tape (RobotReference) judges it, the desk is
  re-measured where the rolled gripper's tape agrees, and a tape off the rolled gripper blocks as camera_moved.

Run:  python TestOccludedDesk.py
"""
import itertools
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
from TestCartMove import CartMoved, CartMove, CartView, TurnedFrame, Without  # noqa: E402
from TestRobotReference import SyntheticArm  # noqa: E402
import bridge_server as Bridge  # noqa: E402
import CameraDrift as CD  # noqa: E402
import RecordedCaptures as RC  # noqa: E402
import RobotReference as RR  # noqa: E402
import SelfCalibration as SC  # noqa: E402
from unittest.mock import patch  # noqa: E402

Fixture = json.loads(RC.Fixture("OccludedDesk-2026-10-22", "Capture.json").read_text(encoding="utf-8"))
# The arm at Home before motion 1 (the model request at 14:57:39Z): hides none of the calibrated desk.
Home = {"shoulder_pan": 3468, "shoulder_lift": 3199, "elbow_flex": 2692, "wrist_flex": 931, "wrist_roll": 1586, "gripper": 693}


def ArmFromRaw(Raw, ReferenceRaw=None):
    """CaptureArmPose for raw servo steps, by the bridge's own conversion (a fresh, unconnected controller): joints,
    gripper_link, the link boxes /locate leaves out of the scene and, with ReferenceRaw, the same pose there."""
    Controller = Bridge.BridgeController()

    def Set(Values):
        for Name, Value in Values.items():
            Controller.servos[Name].present_raw, Controller.servos[Name].gui_angle_raw = int(Value), None
        return Controller._current_ik_angles()

    Angles = Set(Raw)
    Joints, Link = Controller.ArmFrames(Angles)
    Pose = {"ok": True, "joints": Joints, "servos_raw": dict(Raw), "gripper_link": Link, "link_boxes": Controller.ArmLinkBoxes(Angles)}
    if ReferenceRaw:
        Pinned = {**Raw, **ReferenceRaw}
        PinnedJoints, PinnedLink = Controller.ArmFrames(Set(Pinned))
        Pose["reference"] = {"servos_raw": Pinned, "joints": PinnedJoints, "gripper_link": PinnedLink}
    return Pose


class Fresh(unittest.TestCase):
    """Every test starts with no clear view of the desk remembered (CameraDrift.SessionChange): none leaks between them."""

    def setUp(Self):
        CD.ForgetSessionFrames()

    def tearDown(Self):
        CD.ForgetSessionFrames()


def Farther(Depth, Mm):
    """The frame as if the camera stood Mm farther along its axis from everything it sees (no depth stays no depth)."""
    return np.where(Depth > 0, np.clip(Depth.astype(np.int32) + Mm, 1, 65535), 0).astype(np.uint16)


class RealOccludedDeskTests(Fresh):
    """The 14:59:01 capture, as recorded, and turned."""

    @classmethod
    def setUpClass(Class):
        Class.Model = Fixture["model"]
        Class.Camera = {Key: Class.Model[Key] for Key in ("fx", "fy", "cx", "cy", "dist")} | {"width": 1280, "height": 720}
        Folder = RC.Fixture("OccludedDesk-2026-10-22")
        Class.Depth = cv2.imread(str(Folder / Fixture["depth"]), cv2.IMREAD_UNCHANGED)
        Class.Image = cv2.imread(str(Folder / Fixture["image"]))
        Class.Arm = ArmFromRaw(Fixture["raws"], RR.ReferenceRaw(Class.Model))
        Clear = Fixture["clear_view"]  # 14:57:01, the arm at Home: the session's latest clear view of the desk before 14:59
        Class.ClearDepth = cv2.imread(str(Folder / Clear["depth"]), cv2.IMREAD_UNCHANGED)
        Class.ClearImage = cv2.imread(str(Folder / Clear["image"]))
        Class.ClearArm = ArmFromRaw(Clear["raws"], RR.ReferenceRaw(Class.Model))

    def SeeTheClearView(Self):
        """The 14:57:01 capture through CheckCapture, as live: it passes on the desk's own terms and is remembered."""
        Result, New = Without(CD.CheckCapture(Self.ClearDepth, Self.Camera, Self.Model, Fixture["clear_view"]["seq"], Self.ClearImage, Self.ClearArm))
        Recorded = Fixture["clear_view"]["recorded_camera_check"]
        Self.assertEqual((Result["ok"], Result.get("error_code")), (True, None), Result)
        Self.assertIsNone(New)
        Self.assertNotIn("desk_occluded", Result)
        Self.assertAlmostEqual(Result["desk_agreeing_share"], Recorded["desk_agreeing_share"], delta=0.01)
        return Result

    def Check(Self, Depth=None, Image=None, Arm=None):
        return Without(CD.CheckCapture(Self.Depth if Depth is None else Depth, Self.Camera, Self.Model, Fixture["seq"],
                                       Self.Image if Image is None else Image, Self.Arm if Arm is None else Arm))

    def TestTheStoppedCaptureNowPasses(Self):
        # As live: desk_changed_unverified, the visible desk agreeing 63.6% and the wrist roll 59 steps off.
        Recorded = Fixture["recorded_camera_check"]
        Self.assertEqual((Recorded["ok"], Recorded["error_code"], Recorded["desk_agreeing_share"]), (False, "desk_changed_unverified", 0.636))
        Self.assertIn("wrist_roll differs from the calibration's by 59 steps", Recorded["error"])
        # The desk alone, with no arm pose to leave out, still says it could not check the camera: the share floor stands.
        Plain = CD.CheckDesk(Self.Depth, Self.Camera, Self.Model, Fixture["seq"])
        Self.assertEqual((Plain["ok"], Plain["error_code"]), (False, "camera_check_failed"), Plain)
        Self.assertLess(Plain["desk_agreeing_share"], CD.MinAgreeingShare)
        Self.assertLess(Plain["angle_deg"], 0.2, "the desk that is visible did not move")
        # With the arm where it stood: the cells it covers left out, the desk matches where it is visible.
        Result, New = Self.Check()
        Self.assertTrue(Result["ok"], Result)
        Self.assertIsNone(New, "a desk that did not move is not re-measured")
        Self.assertTrue(Result["desk_occluded"])
        Self.assertNotIn("desk_remeasured", Result)
        Self.assertNotIn("alignment_source", Result, "the desk path decided, the desk still the support")
        Self.assertGreater(Result["desk_occluded_cells"], 300)
        Self.assertLess(Result["desk_hidden_cells"], Result["desk_occluded_cells"], "the desk still seen about the arm is not hidden")
        Self.assertLess(Result["desk_agreeing_share"], CD.MinAgreeingShare)
        Self.assertGreaterEqual(Result["desk_agreeing_share_unhidden"], CD.MinAgreeingShare)
        Self.assertLess(Result["angle_deg"], 0.2)
        Self.assertLess(abs(Result["offset_cm"]), 0.2)
        Self.assertIn("where the arm leaves it in view", Result["reason"])
        # The fit is the desk's: little of the desk in view beneath it (the far patch the map holds 1 cm high), and the
        # change it reads well within the occluded limits.
        Self.assertLess(Result["desk_beneath_share"], CD.MaxOccludedBeneathShare - 0.05)
        # With no clear view remembered (this test sees none), the bare desk is judged against what could be desk: 0.12.
        Self.assertEqual(Result["desk_session"], {"session_check": "none"})
        Self.assertLess(Result["desk_beneath_of_desk_share"], CD.MaxOccludedBeneathOfDeskShare - 0.03)
        Self.assertLess(Result["angle_deg"], CD.MaxOccludedNormalChangeDeg)
        Self.assertLess(abs(Result["offset_cm"]), CD.MaxOccludedOffsetChangeCm)
        # The tape was looked for through every joint, not refused for the roll; the open jaw is what it cannot judge.
        Self.assertEqual((Result["tape_check"], Result["tape_model"]), ("pose_differs", "roll_aware"), Result)
        Self.assertIn("gripper differs", Result["tape_reason"])
        Self.assertIn("wrist_roll differs from the calibration's by 59 steps", Result["tape_roll_gated_reason"])

    def TestTheStoppedCapturePassesAgainstTheClearViewBeforeIt(Self):
        # As live: 14:57:01 (the arm at Home) passed, then 14:59:01. Cell for cell, the desk in view agrees with no change
        # since (Aaran had leaned over the keyboard at 14:57, so only about two thirds of the cells agree; still past the floor).
        Self.SeeTheClearView()
        Result, New = Self.Check()
        Self.assertEqual((Result["ok"], Result.get("desk_occluded")), (True, True), Result)
        Self.assertIsNone(New)
        Session = Result["desk_session"]
        Self.assertEqual(Session["session_check"], "still", Session)
        Self.assertGreater(Session["cells"], 2000)
        Self.assertGreaterEqual(Session["agreeing_share"], CD.MinSessionAgreeingShare + 0.1)
        Self.assertLess(abs(Session["offset_cm"]), 0.05)
        Self.assertLess(Session["angle_deg"], 0.1)
        Self.assertIn("unmoved since the clear view", Result["reason"])
        # A clear view older than SessionFrameMaxAgeS is not compared with: the capture is judged as with none.
        with patch.object(CD, "SessionFrameMaxAgeS", -1.0):
            Old, _ = Self.Check()
        Self.assertEqual((Old["ok"], Old.get("desk_occluded"), Old["desk_session"]), (True, True, {"session_check": "none"}))

    def TestAMoveTheOccludedLimitsMissBlocksAgainstTheClearView(Self):
        # The 14:59 frame 3 mm farther: the desk in view reads 0.23 cm, inside the occluded limit (0.25 cm), so with no clear
        # view it passes as before; against 14:57 every cell seen in both moved about 0.3 cm, and it blocks.
        Depth = Farther(Self.Depth, 3)
        Alone, _ = Self.Check(Depth)
        Self.assertEqual((Alone["ok"], Alone.get("desk_occluded")), (True, True), Alone)
        Self.assertLess(abs(Alone["offset_cm"]), CD.MaxOccludedOffsetChangeCm)
        Self.SeeTheClearView()
        Result, New = Self.Check(Depth)
        Self.assertFalse(Result["ok"], Result)
        Self.assertIsNone(New)
        Self.assertNotIn("desk_occluded", Result)
        Why = Result["desk_change"]["reason"] if "desk_change" in Result else Result["reason"]
        Self.assertIn("against the clear view of the desk", Why)
        Self.assertEqual(Result["desk_session"]["session_check"], "moved", Result["desk_session"])
        Self.assertGreater(Result["desk_session"]["offset_cm"], CD.MaxOccludedOffsetChangeCm)
        Self.assertGreaterEqual(Result["desk_session"]["agreeing_share"], CD.MinSessionAgreeingShare)
        # Nearer or farther by 6 mm the desk itself reads past the block limits, clear view or not.
        for Mm in (6, -6):
            with Self.subTest(mm=Mm):
                for Seen in (False, True):
                    CD.ForgetSessionFrames()
                    if Seen:
                        Self.SeeTheClearView()
                    Moved, New = Self.Check(Farther(Self.Depth, Mm))
                    Self.assertFalse(Moved["ok"], Moved)
                    Self.assertIsNone(New)
                    Self.assertNotIn("desk_occluded", Moved)

    def TestTheDeskSeenInTheArmsMarginCountsBothWays(Self):
        # The arm's mask, grown ArmOcclusionMarginPx, covers 405 calibrated desk cells; 48 of the edge ones still read as
        # desk. Only the 357 not seen as desk are hidden, and the 48 are judged by the fit (made without them).
        Reference, _ = CD.VerifiedReference(Self.Model)
        Plane = CD.DeskPlane(Self.Depth, Self.Camera, Reference["desk_n"], ReferenceOffset=Reference["desk_d"],
                             OffsetGateCm=CD.FitPlaneOffsetGateCm)
        Step = int(Reference["desk_map"]["step"])
        Grown = CD.ArmOcclusion(Self.Depth, Self.Camera, Self.Model, Self.Arm, Step)
        Map, Points = CD.DeskMap(Reference), CD.GridPoints(Self.Depth, Self.Camera, Step)
        with np.errstate(invalid="ignore"):
            OnDesk = np.abs(Points @ np.asarray(Plane["n"]) - float(Plane["d"])) <= CD.InlierCm
        Calibrated = np.isfinite(Map)
        Shown = Grown & Calibrated & OnDesk
        Self.assertEqual(int((Grown & Calibrated).sum()), 405)
        Self.assertGreater(int(Shown.sum()), 20)
        Change = CD.DeskChange(Self.Depth, Self.Camera, Reference, Plane, Grown)
        Self.assertEqual(Change["occluded_cells"], 405)
        Self.assertEqual(Change["hidden_cells"], int((Grown & Calibrated & ~OnDesk).sum()))
        Self.assertEqual(Change["hidden_cells"] + int(Shown.sum()), 405)
        # The edge's desk raised 4 mm (still desk, past the 0.25 cm band): the fit is the same, those cells now disagree.
        Rows, Cols = np.nonzero(Shown)
        Raised = Self.Depth.copy()
        Raised[Rows * Step, Cols * Step] += 4
        Off = CD.DeskChange(Raised, Self.Camera, Reference, Plane, Grown)
        Self.assertAlmostEqual(Off["angle_deg"], Change["angle_deg"], places=9)
        Self.assertAlmostEqual(Off["offset_cm"], Change["offset_cm"], places=9)
        Self.assertEqual(Off["cells"] + Off["hidden_cells"], Change["cells"] + Change["hidden_cells"])
        Self.assertLess(Off["agreeing_cells"], Change["agreeing_cells"] - 20, "margin desk that disagrees counts against")

    def TestTheArmAtHomeHidesNothingAndTheFloorHolds(Self):
        # The same frame checked as if the arm stood at Home: it hides none of the desk, so nothing explains the low share.
        Result, New = Self.Check(Arm=ArmFromRaw(Home, RR.ReferenceRaw(Self.Model)))
        Self.assertFalse(Result["ok"], Result)
        Self.assertIsNone(New)
        Self.assertNotIn("desk_occluded", Result)
        Self.assertLess(Result.get("desk_occluded_cells", 0), 30)
        Self.assertEqual(Result["desk_change"]["error_code"], "camera_check_failed")

    def TestAShiftOfTheVisibleSceneIsStillCaught(Self):
        # The frame as the camera turned on the arm about its centre would have seen it: from 1 deg any way the visible
        # desk reads past the 0.5 deg limit and blocks, occluded or not, and nothing is re-measured.
        for Axis, Name in ((0, "droop"), (1, "turn"), (2, "roll")):
            for Degrees in (1.0, -1.0, 2.0, -2.0, 3.0, -3.0, 5.0, -5.0):
                with Self.subTest(motion=Name, deg=Degrees):
                    Depth, Image = TurnedFrame(Self.Depth, Self.Image, Self.Camera, cv2.Rodrigues(np.eye(3)[Axis] * math.radians(Degrees))[0])
                    Result, New = Self.Check(Depth, Image)
                    Self.assertFalse(Result["ok"], Result)
                    Self.assertIsNone(New)
                    Self.assertNotIn("desk_occluded", Result)
                    Self.assertIn(Result["error_code"], ("camera_moved", "desk_changed_unverified", "camera_check_failed"), Result)
                    Self.assertGreater(Result["angle_deg"], CD.MaxNormalChangeDeg)

    def TestASmallDeskAboutTheArmIsNeverAbsent(Self):
        # Only the calibrated desk within K cells of the arm's mask left in depth (a small desk about the arm, like the
        # 2026-10-01 bin): 200-260 cells seen as desk, under 200 once the arm's are left out. The desk seen decides, as
        # without the arm, so a turned camera blocks on the desk and is never handed to the robot (whose tape, the jaw
        # off the calibration's, could only assume).
        Reference, _ = CD.VerifiedReference(Self.Model)
        Step, Map = int(Reference["desk_map"]["step"]), CD.DeskMap(Reference)
        for Axis, Degrees, K in ((0, 0.0, 4), (0, 1.5, 4), (1, 1.5, 4), (0, -1.5, 5), (1, -1.5, 4), (0, 2.5, 4), (1, 2.5, 4)):
            with Self.subTest(axis=Axis, deg=Degrees, k=K):
                Depth, Image = (Self.Depth, Self.Image) if not Degrees else TurnedFrame(
                    Self.Depth, Self.Image, Self.Camera, cv2.Rodrigues(np.eye(3)[Axis] * math.radians(Degrees))[0])
                Occluded = CD.ArmOcclusion(Depth, Self.Camera, Self.Model, Self.Arm, Step)
                Near = cv2.dilate(Occluded.astype(np.uint8), np.ones((2 * K + 1, 2 * K + 1), np.uint8)).astype(bool)
                Kept = np.kron(Near | ~np.isfinite(Map), np.ones((Step, Step), bool))[:Depth.shape[0], :Depth.shape[1]]
                Small = np.where(Kept, Depth, 0).astype(np.uint16)
                Plain = CD.CheckDesk(Small, Self.Camera, Self.Model, Fixture["seq"])
                Armed = CD.CheckDesk(Small, Self.Camera, Self.Model, Fixture["seq"], Self.Arm)
                Self.assertGreaterEqual(Plain["desk_cells"], CD.MinMapCells, Plain)
                Plane = CD.DeskPlane(Small, Self.Camera, Reference["desk_n"], ReferenceOffset=Reference["desk_d"],
                                     OffsetGateCm=CD.FitPlaneOffsetGateCm) or CD.DeskPlane(Small, Self.Camera, Reference["desk_n"])
                with np.errstate(invalid="ignore"):
                    Seen = (np.abs(CD.GridPoints(Small, Self.Camera, Step) @ np.asarray(Plane["n"]) - float(Plane["d"])) <= CD.InlierCm) & np.isfinite(Map)
                Self.assertEqual(int(Seen.sum()), Plain["desk_cells"])
                Self.assertLess(int((Seen & ~Occluded).sum()), CD.MinMapCells, "the arm's exclusion alone leaves too few to fit")
                for Key in ("ok", "error_code", "angle_deg", "offset_cm", "desk_cells", "desk_agreeing_share"):
                    Self.assertEqual(Armed.get(Key), Plain.get(Key), (Key, Armed, Plain))
                Self.assertNotIn("desk_occluded_cells", Armed, "the whole desk seen is fitted, nothing credited as hidden")
                Result, New = Self.Check(Small, Image)
                Self.assertNotEqual(Result.get("alignment_source"), "robot", Result)
                Self.assertIsNone(New)
                if Degrees:
                    Self.assertEqual(Armed["error_code"], "camera_moved", Armed)
                    Self.assertFalse(Result["ok"], Result)
                    Self.assertNotIn("desk_occluded", Result)
                else:
                    Self.assertTrue(Result["ok"], Result)


# --------------------------------------------------------------------------
# Synthetic: a mat on the desk and the arm over it
# --------------------------------------------------------------------------
TableZ = TDC.TableZ
Desk = TDC.Workspace()[0]
Tape = np.array([1.2, -0.6, 0.8])
TapePose = {"wrist_roll_raw": 2048, "gripper_raw": 2100}
Mat = (np.array([-40.0, 36.0, TableZ]), np.array([20.0, 90.0, TableZ + 0.6]))  # a 0.6 cm mat over the far desk


def ArmBox(Half, Joints):
    """A gripper-sized box over the near desk (display frame) and the CaptureArmPose that says the arm stood there
    (Joints: a pose whose tape the camera sees, for the tape check)."""
    Centre, Half = np.array([-12.0, 34.0, TableZ + 9.0]), np.asarray(Half, dtype=float)
    Pose = {"ok": True, "joints": np.asarray(Joints).tolist(), "servos_raw": dict(wrist_roll=2048, gripper=2100),
            "link_boxes": [{"name": "gripper", "centre": Centre.tolist(), "axes": np.eye(3).tolist(), "half": Half.tolist()}]}
    return (Centre - Half, Centre + Half), Pose


def Noisy(Depth):
    return np.where(Depth > 0, np.clip(Depth + np.random.default_rng(1).normal(0.0, 1.0, Depth.shape), 1, None), 0).astype(np.uint16)


class SyntheticOccludedDeskTests(Fresh):
    @classmethod
    def setUpClass(Class):
        Class.Model = dict(TDC.Verified(TDC.HeadCamera, [Desk], FloorZ=TDC.RoomFloorZ, TapePose=TapePose), marker_corners_gripper_cm=[Tape.tolist()])
        Class.Joints, _ = TDC.ArmInView(TDC.HeadCamera, Tape)

    def View(Self, Camera, Half):
        Box, Arm = ArmBox(Half, Self.Joints)
        Image = np.full((TDC.Height, TDC.Width, 3), 90, np.uint8)  # no red: the tape is not seen, as on a desk pass it never blocks
        return Image, Noisy(TDC.RenderDepth(Camera, [Desk, Mat, Box], FloorZ=TDC.RoomFloorZ)), Arm

    def TestTheArmOverAMatOnAStillDeskPasses(Self):
        Image, Depth, Arm = Self.View(TDC.HeadCamera, (10.0, 9.0, 2.0))
        Plain = CD.CheckDesk(Depth, TDC.Intrinsics, Self.Model)
        Self.assertEqual((Plain["ok"], Plain["error_code"]), (False, "camera_check_failed"), Plain)
        Result, New = Without(CD.CheckCapture(Depth, TDC.Intrinsics, Self.Model, 3, Image, Arm))
        Self.assertEqual((Result["ok"], Result.get("desk_occluded")), (True, True), Result)
        Self.assertIsNone(New)
        Self.assertLess(Result["angle_deg"], 0.1)
        Self.assertLess(Result["desk_beneath_share"], 0.02, "a mat on a still desk lies on it, never beneath")
        Self.assertEqual(Result["tape_check"], "not_visible")

    def TestTheSameSceneTurnedStillBlocks(Self):
        for Degrees in (1.5, -1.5):
            with Self.subTest(deg=Degrees):
                Image, Depth, Arm = Self.View(TDC.Turned(TDC.HeadCamera, Degrees), (10.0, 9.0, 2.0))
                Result, New = Without(CD.CheckCapture(Depth, TDC.Intrinsics, Self.Model, 3, Image, Arm))
                Self.assertFalse(Result["ok"], Result)
                Self.assertIsNone(New)
                Self.assertNotIn("desk_occluded", Result)
                Self.assertGreater(Result["angle_deg"], 1.0)

    def TestAnArmTooSmallToExplainItDoesNotOpenTheFloor(Self):
        Box, Arm = ArmBox((8.0, 7.0, 2.0), Self.Joints)
        Wider = (np.array([-40.0, 34.0, TableZ]), np.array([20.0, 90.0, TableZ + 0.6]))  # the mat to Y 34: 61% agree
        Depth = Noisy(TDC.RenderDepth(TDC.HeadCamera, [Desk, Wider, Box], FloorZ=TDC.RoomFloorZ))
        Result = CD.CheckDesk(Depth, TDC.Intrinsics, Self.Model, 3, Arm)
        Self.assertEqual((Result["ok"], Result["error_code"]), (False, "camera_check_failed"), Result)
        Self.assertGreater(Result["desk_occluded_cells"], 0)
        Self.assertNotIn("desk_occluded", Result)


class VisibleShareFloorTests(Fresh):
    """However much the arm hides, the desk in view must agree on its own (MinOccludedAgreeingShare)."""

    @classmethod
    def setUpClass(Class):
        Class.Model = dict(TDC.Verified(TDC.HeadCamera, [Desk], FloorZ=TDC.RoomFloorZ, TapePose=TapePose), marker_corners_gripper_cm=[Tape.tolist()])
        Class.Joints, _ = TDC.ArmInView(TDC.HeadCamera, Tape)

    def MovedUnderAMat(Self):
        # The camera 0.6 cm farther from the desk (past the 0.5 cm limit), a 0.6 cm mat over the far desk with its top at
        # the old desk height, and a long arm low over the near desk hiding most of the bare (moved) desk.
        Drop = 0.6
        Lowered = (Desk[0], Desk[1] - np.array([0.0, 0.0, Drop]))
        Over = (np.array([-60.0, 64.0, TableZ - Drop]), np.array([40.0, 90.0, TableZ]))
        Centre, Half = np.array([-10.0, 38.0, TableZ + 5.0]), np.array([35.0, 12.0, 2.0])
        Arm = {"ok": True, "joints": np.asarray(Self.Joints).tolist(), "servos_raw": dict(wrist_roll=2048, gripper=2100),
               "link_boxes": [{"name": "forearm", "centre": Centre.tolist(), "axes": np.eye(3).tolist(), "half": Half.tolist()}]}
        Depth = Noisy(TDC.RenderDepth(TDC.HeadCamera, [Lowered, Over, (Centre - Half, Centre + Half)], FloorZ=TDC.RoomFloorZ))
        return Depth, Arm

    def TestAMoveTheMatAndTheArmHideStillBlocks(Self):
        Depth, Arm = Self.MovedUnderAMat()
        Result = CD.CheckDesk(Depth, TDC.Intrinsics, Self.Model, 3, Arm)
        Self.assertEqual((Result["ok"], Result["error_code"]), (False, "camera_check_failed"), Result)
        Self.assertNotIn("desk_occluded", Result)
        # The mat won the fit: the move reads as no change, and the hidden cells alone would clear the share floor.
        Self.assertLess(Result["angle_deg"], CD.MaxNormalChangeDeg)
        Self.assertLess(abs(Result["offset_cm"]), CD.MaxOffsetChangeCm)
        Share, Hidden, Cells = Result["desk_agreeing_share"], Result["desk_hidden_cells"], Result["desk_cells"]
        Self.assertLess(Share, CD.MinOccludedAgreeingShare)
        Self.assertGreaterEqual((Share * Cells + Hidden) / (Cells + Hidden), CD.MinAgreeingShare + 0.03)
        # Without the visible floor (and the bare desk beneath the fit, which also stops it) this real move would pass.
        with patch.object(CD, "MinOccludedAgreeingShare", 0.0), patch.object(CD, "MaxOccludedBeneathShare", 1.0), \
                patch.object(CD, "MaxOccludedBeneathOfDeskShare", 1.0):
            Self.assertTrue(CD.CheckDesk(Depth, TDC.Intrinsics, Self.Model, 3, Arm).get("desk_occluded"))

    def TestTheFloorIsTheConstant(Self):
        # The gate itself, on fitted numbers: the arm hides as much again as is visible (the share it would explain is
        # about 0.78 at the floor), and the desk in view agrees just under or at MinOccludedAgreeingShare.
        Box, Arm = ArmBox((10.0, 9.0, 2.0), Self.Joints)
        Depth = Noisy(TDC.RenderDepth(TDC.HeadCamera, [Desk, Mat, Box], FloorZ=TDC.RoomFloorZ))
        Real, Floor = CD.DeskChange, round(CD.MinOccludedAgreeingShare * 1000)
        for Agreeing, Passes in ((Floor - 1, False), (Floor, True)):
            with Self.subTest(agreeing=Agreeing):
                CD.DeskChange = lambda *Args, **Kwargs: {"angle_deg": 0.05, "offset_cm": 0.02, "cells": 1000, "agreeing_cells": Agreeing,
                                                         "occluded_cells": 1000, "hidden_cells": 1000, "normal": [0.0, 0.0, 1.0]}
                try:
                    Result = CD.CheckDesk(Depth, TDC.Intrinsics, Self.Model, 3, Arm)
                finally:
                    CD.DeskChange = Real
                Self.assertEqual(Result["ok"], Passes, Result)
                Self.assertEqual(bool(Result.get("desk_occluded")), Passes)
                if not Passes:
                    Self.assertEqual(Result["error_code"], "camera_check_failed")

    def TestTheOccludedGatesAreTheConstants(Self):
        # The same gate on fitted numbers that clear the visible floor (0.6 agreeing, 0.8 had the hidden cells agreed):
        # the share beneath the fit and the change read each pass at their constant and block just past it.
        Box, Arm = ArmBox((10.0, 9.0, 2.0), Self.Joints)
        Depth = Noisy(TDC.RenderDepth(TDC.HeadCamera, [Desk, Mat, Box], FloorZ=TDC.RoomFloorZ))
        Real, Beneath = CD.DeskChange, round(CD.MaxOccludedBeneathShare * 1000)
        Edge = lambda Limit, Sign: Limit + Sign * 0.004
        for Name, Angle, Offset, Under, Passes in (("beneath at", 0.05, 0.02, Beneath, True), ("beneath past", 0.05, 0.02, Beneath + 1, False),
                                                   ("angle at", Edge(CD.MaxOccludedNormalChangeDeg, -1), 0.02, 0, True),
                                                   ("angle past", Edge(CD.MaxOccludedNormalChangeDeg, 1), 0.02, 0, False),
                                                   ("offset at", 0.05, Edge(CD.MaxOccludedOffsetChangeCm, -1), 0, True),
                                                   ("offset past", 0.05, Edge(CD.MaxOccludedOffsetChangeCm, 1), 0, False),
                                                   ("offset past, nearer", 0.05, -Edge(CD.MaxOccludedOffsetChangeCm, 1), 0, False)):
            with Self.subTest(Name):
                CD.DeskChange = lambda *Args, Angle=Angle, Offset=Offset, Under=Under, **Kwargs: {
                    "angle_deg": Angle, "offset_cm": Offset, "cells": 1000, "agreeing_cells": 600, "beneath_cells": Under,
                    "occluded_cells": 1000, "hidden_cells": 1000, "normal": [0.0, 0.0, 1.0]}
                try:
                    with patch.object(CD, "MaxOccludedBeneathOfDeskShare", 1.0):  # judged on its own (TestTheShareOfWhatCouldBeDesk)
                        Result = CD.CheckDesk(Depth, TDC.Intrinsics, Self.Model, 3, Arm)
                finally:
                    CD.DeskChange = Real
                Self.assertEqual((Result["ok"], bool(Result.get("desk_occluded"))), (Passes, Passes), Result)
                Self.assertEqual(Result["desk_beneath_share"], round(Under / 1000, 3))
                if not Passes:
                    Self.assertEqual(Result["error_code"], "camera_check_failed")
                    Self.assertIn("beneath the fit" if Under else "past what a desk the arm hides may read", Result["reason"])

    def Gate(Self, Under, Session=None, Agreeing=600):
        """CheckDesk on fitted numbers that clear every other occluded gate (0.6 agreeing, 0.8 had the hidden cells agreed,
        0.05 deg / 0.02 cm), Under cells beneath the fit and SessionChange as Session (None: no clear view)."""
        Box, Arm = ArmBox((10.0, 9.0, 2.0), Self.Joints)
        Depth = Noisy(TDC.RenderDepth(TDC.HeadCamera, [Desk, Mat, Box], FloorZ=TDC.RoomFloorZ))
        Fitted = lambda *Args, **Kwargs: {"angle_deg": 0.05, "offset_cm": 0.02, "cells": 1000, "agreeing_cells": Agreeing,
                                          "beneath_cells": Under, "occluded_cells": 1000, "hidden_cells": 1000, "normal": [0.0, 0.0, 1.0]}
        with patch.object(CD, "DeskChange", Fitted), patch.object(CD, "SessionChange", lambda *Args: Session or {"session_check": "none"}):
            return CD.CheckDesk(Depth, TDC.Intrinsics, Self.Model, 3, Arm)

    def TestTheShareOfWhatCouldBeDeskIsTheConstant(Self):
        # With no clear view, beneath over agreeing-or-beneath passes at MaxOccludedBeneathOfDeskShare and blocks just past
        # it (600 agreeing: 114 beneath is 0.1597, 115 is 0.1608), while the share of the desk in view stays far under 0.2.
        At = math.floor(CD.MaxOccludedBeneathOfDeskShare * 600 / (1 - CD.MaxOccludedBeneathOfDeskShare))
        for Under, Passes in ((At, True), (At + 1, False)):
            with Self.subTest(beneath=Under):
                Result = Self.Gate(Under)
                Self.assertEqual((Result["ok"], bool(Result.get("desk_occluded"))), (Passes, Passes), Result)
                Self.assertLess(Result["desk_beneath_share"], CD.MaxOccludedBeneathShare - 0.05)
                Self.assertEqual(Result["desk_beneath_of_desk_share"], round(Under / (Under + 600), 3))
                if not Passes:
                    Self.assertEqual(Result["error_code"], "camera_check_failed")
                    Self.assertIn("of what could be desk lies beneath the fit", Result["reason"])

    def TestTheClearViewDecidesWhenItCan(Self):
        # The clear view only adds a block: one that agrees on a move blocks however the desk in view reads; one that agrees
        # on no change still needs the share of what could be desk (a clear view taken before a mat was laid reads "still"
        # when the mat cancels a real move); an inconclusive one leaves it to that share, as with none.
        Still = {"session_check": "still", "age_s": 60.0, "cells": 2500, "agreeing_share": 0.7, "angle_deg": 0.05, "offset_cm": 0.01}
        Moved = dict(Still, session_check="moved", offset_cm=0.4)
        Unsure = dict(Still, session_check="inconclusive", agreeing_share=0.3)
        for Name, Under, Session, Passes in (("still, past the share", 150, Still, False), ("still, within it", 50, Still, True), ("moved, nothing beneath", 0, Moved, False),
                                             ("inconclusive, past the share", 150, Unsure, False), ("inconclusive, within it", 50, Unsure, True)):
            with Self.subTest(Name):
                Result = Self.Gate(Under, Session)
                Self.assertEqual((Result["ok"], bool(Result.get("desk_occluded"))), (Passes, Passes), Result)
                Self.assertEqual(Result["desk_session"], Session)
                if Session is Moved:
                    Self.assertIn("against the clear view of the desk 60 s before, what is in view moved 0.05 deg / +0.40 cm", Result["reason"])
                if Passes and Session is Still:
                    Self.assertIn("unmoved since the clear view 60 s before", Result["reason"])


class MatMajorityShiftTests(Fresh):
    """A real move under a mat over most of the desk in view, the arm hiding the rest (review of 124ef8e).

    The camera 0.6-0.9 cm farther (past the 0.5 cm limit), a flat mat 0.45-0.8 cm thick over the far desk, and the
    arm low over the near desk. The mat is most of the desk in view (56-66%), so its top wins the median fit, at or
    near the old desk height; the bare desk, really moved, disagrees beneath it; and the arm hides enough that the share
    would clear the floor had its cells agreed. Each of these passed as desk_occluded at 124ef8e."""

    # (camera farther cm, mat thickness cm, the arm's half-extent X, Y cm, its centre Y, where the mat starts Y)
    Scenes = ((0.6, 0.45, 18, 6, 34, 44), (0.6, 0.6, 25, 8, 34, 52), (0.7, 0.45, 18, 6, 34, 44), (0.7, 0.45, 20, 6, 34, 44),
              (0.7, 0.45, 15, 5, 34, 40), (0.7, 0.45, 25, 8, 34, 52), (0.7, 0.8, 25, 10, 36, 48), (0.9, 0.45, 18, 6, 34, 44),
              (0.9, 0.8, 25, 8, 34, 52), (0.9, 0.6, 30, 10, 36, 48))

    @classmethod
    def setUpClass(Class):
        Class.Model = dict(TDC.Verified(TDC.HeadCamera, [Desk], FloorZ=TDC.RoomFloorZ, TapePose=TapePose), marker_corners_gripper_cm=[Tape.tolist()])
        Class.Joints, _ = TDC.ArmInView(TDC.HeadCamera, Tape)

    def Scene(Self, Drop, Thick, HalfX, HalfY, CentreY, MatY):
        Lowered = (Desk[0], Desk[1] - np.array([0.0, 0.0, Drop]))
        Over = (np.array([-60.0, MatY, TableZ - Drop]), np.array([40.0, 90.0, TableZ - Drop + Thick]))
        Centre, Half = np.array([-10.0, CentreY, TableZ + 5.0]), np.array([HalfX, HalfY, 2.0])
        Arm = {"ok": True, "joints": np.asarray(Self.Joints).tolist(), "servos_raw": dict(wrist_roll=2048, gripper=2100),
               "link_boxes": [{"name": "forearm", "centre": Centre.tolist(), "axes": np.eye(3).tolist(), "half": Half.tolist()}]}
        return Noisy(TDC.RenderDepth(TDC.HeadCamera, [Lowered, Over, (Centre - Half, Centre + Half)], FloorZ=TDC.RoomFloorZ)), Arm

    def TestAMatMajorityShiftBlocks(Self):
        Shares = []
        for Scene in Self.Scenes:
            with Self.subTest(scene=Scene):
                Depth, Arm = Self.Scene(*Scene)
                Result = CD.CheckDesk(Depth, TDC.Intrinsics, Self.Model, 3, Arm)
                Self.assertEqual((Result["ok"], Result["error_code"]), (False, "camera_check_failed"), Result)
                Self.assertNotIn("desk_occluded", Result)
                # The old gate would have passed it: the visible floor and the hidden cells' share both cleared, no change
                # past the block limits read.
                Share, Hidden, Cells = Result["desk_agreeing_share"], Result["desk_hidden_cells"], Result["desk_cells"]
                Self.assertTrue(CD.MinOccludedAgreeingShare <= Share < CD.MinAgreeingShare, Share)
                Self.assertGreaterEqual((Share * Cells + Hidden) / (Cells + Hidden), CD.MinAgreeingShare)
                Self.assertLess(Result["angle_deg"], CD.MaxNormalChangeDeg)
                Self.assertLess(abs(Result["offset_cm"]), CD.MaxOffsetChangeCm)
                # What stops it: the bare desk beneath the fit, by a wide margin.
                Self.assertGreater(Result["desk_beneath_share"], CD.MaxOccludedBeneathShare + 0.1)
                Self.assertIn("beneath the fit", Result["reason"])
                Shares.append(Share)
                with patch.object(CD, "MaxOccludedBeneathShare", 1.0), patch.object(CD, "MaxOccludedNormalChangeDeg", 9.0), \
                        patch.object(CD, "MaxOccludedOffsetChangeCm", 9.0), patch.object(CD, "MaxOccludedBeneathOfDeskShare", 1.0):
                    Self.assertTrue(CD.CheckDesk(Depth, TDC.Intrinsics, Self.Model, 3, Arm).get("desk_occluded"), "as at 124ef8e")
                with patch.object(CD, "MaxOccludedBeneathShare", 1.0), patch.object(CD, "MaxOccludedBeneathOfDeskShare", 1.0):
                    Alone = CD.CheckDesk(Depth, TDC.Intrinsics, Self.Model, 3, Arm)  # without either, only the near-limit gate is left
                    Self.assertEqual(bool(Alone.get("desk_occluded")), abs(Result["offset_cm"]) <= CD.MaxOccludedOffsetChangeCm, Alone)
                # The share beneath of what could be desk stops each of them on its own, too.
                Self.assertGreater(Result["desk_beneath_of_desk_share"], CD.MaxOccludedBeneathOfDeskShare + 0.15)
                with patch.object(CD, "MaxOccludedBeneathShare", 1.0):
                    Self.assertNotIn("desk_occluded", CD.CheckDesk(Depth, TDC.Intrinsics, Self.Model, 3, Arm))
        Self.assertLess(min(Shares), 0.57)
        Self.assertGreater(max(Shares), 0.65)

    def TestTheProbedCaptureBlocksEndToEnd(Self):
        # The reviewer's probe (0.7 cm, 0.45 cm mat from Y 44, arm 18 x 6 and 20 x 6), through CheckCapture: no tape in view,
        # so the cart-move path cannot re-measure, and nothing passes.
        for Scene in ((0.7, 0.45, 18, 6, 34, 44), (0.7, 0.45, 20, 6, 34, 44)):
            with Self.subTest(scene=Scene):
                Depth, Arm = Self.Scene(*Scene)
                Image = np.full((TDC.Height, TDC.Width, 3), 90, np.uint8)
                Result, New = Without(CD.CheckCapture(Depth, TDC.Intrinsics, Self.Model, 3, Image, Arm))
                Self.assertFalse(Result["ok"], Result)
                Self.assertIsNone(New)
                Self.assertNotIn("desk_occluded", Result)


class ClutterOnMatTests(Fresh):
    """Items on the mat dilute the share beneath the fit (review of b1edc92).

    The camera 0.6-0.7 cm farther, a mat as thick as that over the far desk (from Y 34 or 36: its top at the old desk
    height), a thin item 0.3-0.7 cm proud of the mat (a notebook, a paper stack, a phone, at about X -45..15, Y 56..84)
    and a long arm low over the near desk. The item's cells are neither agreeing nor beneath, so the bare desk is only
    15-21% of the desk in view and b1edc92's gate (MaxOccludedBeneathShare 0.2) let most of these pass as desk_occluded,
    reading -0.05 cm / 0.02 deg. Against what could be desk (agreeing or beneath) the bare desk is 0.19-0.26."""

    Arms = ((30, 8, 33), (35, 8, 33), (30, 9, 34))  # the arm's half-extent X, Y cm and its centre Y
    Items = (((-45, 10), (58, 82)), ((-45, 15), (60, 84)), ((-40, 10), (56, 80)))  # X and Y ranges, cm
    Scenes = list(itertools.product((0.6, 0.7), (34, 36), Arms, Items, (0.3, 0.5, 0.7)))  # camera farther, mat from Y, arm, item, lift

    @classmethod
    def setUpClass(Class):
        Class.Model = dict(TDC.Verified(TDC.HeadCamera, [Desk], FloorZ=TDC.RoomFloorZ, TapePose=TapePose), marker_corners_gripper_cm=[Tape.tolist()])
        Class.Joints, _ = TDC.ArmInView(TDC.HeadCamera, Tape)

    def Scene(Self, Drop, MatY, ArmSize, Item, Lift, Moved=True, WithArm=True):
        """The depth frame and the arm pose: Moved, the camera Drop farther (the world Drop lower, the mat's top at the old
        desk height); else the same desk before the move (everything Drop higher)."""
        Base = TableZ - (Drop if Moved else 0.0)
        (X0, X1), (Y0, Y1) = Item
        Objects = [(Desk[0], Desk[1] - np.array([0.0, 0.0, TableZ - Base])),
                   (np.array([-60.0, MatY, Base]), np.array([40.0, 90.0, Base + Drop])),
                   (np.array([X0, Y0, Base + Drop]), np.array([X1, Y1, Base + Drop + Lift]))]
        HalfX, HalfY, CentreY = ArmSize
        Centre, Half = np.array([-10.0, CentreY, TableZ + 5.0]), np.array([HalfX, HalfY, 2.0])
        Arm = {"ok": True, "joints": np.asarray(Self.Joints).tolist(), "servos_raw": dict(wrist_roll=2048, gripper=2100),
               "link_boxes": [{"name": "forearm", "centre": Centre.tolist(), "axes": np.eye(3).tolist(), "half": Half.tolist()}]}
        if WithArm:
            Objects.append((Centre - Half, Centre + Half))
        return Noisy(TDC.RenderDepth(TDC.HeadCamera, Objects, FloorZ=TDC.RoomFloorZ)), Arm

    def TestABareDeskClearViewBeforeTheMatNeverWaivesTheGate(Self):
        # Review round 5 of beffdc5: a clear view of the bare desk, taken before the mat and items were laid, reads "still"
        # against a moved camera when the mat's thickness cancels the move. The clear view may only add a block, so none of
        # these scenes passes as a desk the arm hides even with that view remembered.
        Bare = Noisy(TDC.RenderDepth(TDC.HeadCamera, [Desk], FloorZ=TDC.RoomFloorZ))
        Checked = 0
        for Scene in Self.Scenes:
            with Self.subTest(scene=Scene):
                CD.ForgetSessionFrames()
                Self.assertTrue(CD.RememberSessionFrame(Bare, TDC.Intrinsics, Self.Model, None), "the bare desk is remembered")
                Depth, Arm = Self.Scene(*Scene)
                Result = CD.CheckDesk(Depth, TDC.Intrinsics, Self.Model, 3, Arm)
                Self.assertNotIn("desk_occluded", Result, "a pre-mat clear view never lets a moved camera pass as occluded")
                Checked += 1
        CD.ForgetSessionFrames()
        Self.assertEqual(Checked, len(Self.Scenes))

    def TestClutterOnAMatAtTheOldDeskHeightBlocks(Self):
        Before, Ratios, Plain = 0, [], 0
        for Scene in Self.Scenes:
            with Self.subTest(scene=Scene):
                Depth, Arm = Self.Scene(*Scene)
                Result = CD.CheckDesk(Depth, TDC.Intrinsics, Self.Model, 3, Arm)
                Self.assertNotIn("desk_occluded", Result, "no clutter scene passes as a desk the arm hides")
                Self.assertLess(abs(Result["offset_cm"]), CD.MaxOccludedOffsetChangeCm, "the mat's top won the fit: no move read")
                if Result["desk_agreeing_share"] >= CD.MinAgreeingShare:
                    # Some 0.3 cm items agree with the fit through the depth noise, lifting the share past the plain floor: the
                    # plain desk path passes them with or without the arm, as it did at 8eefe77 (no occluded gate is reached).
                    Self.assertEqual(Scene[-1], 0.3, Result)
                    Self.assertEqual(Result["ok"], CD.CheckDesk(Depth, TDC.Intrinsics, Self.Model, 3)["ok"])
                    Plain += 1
                    continue
                Self.assertEqual((Result["ok"], Result["error_code"]), (False, "camera_check_failed"), Result)
                with patch.object(CD, "MaxOccludedBeneathOfDeskShare", 1.0):  # as at b1edc92
                    Old = bool(CD.CheckDesk(Depth, TDC.Intrinsics, Self.Model, 3, Arm).get("desk_occluded"))
                if Old:
                    Before += 1
                    Ratios.append(Result["desk_beneath_of_desk_share"])
                    Self.assertLessEqual(Result["desk_beneath_share"], CD.MaxOccludedBeneathShare, "b1edc92's gate did not see it")
                    Self.assertIn("of what could be desk lies beneath the fit", Result["reason"])
        Self.assertGreaterEqual(Before, 50, "most of these passed at b1edc92")
        Self.assertGreater(min(Ratios), CD.MaxOccludedBeneathOfDeskShare + 0.025)
        Self.assertLess(Plain, 25)
        print(f"  {Before} of {len(Self.Scenes)} clutter scenes passed at b1edc92, now blocked (beneath of what could be desk "
              f"{min(Ratios):.3f}-{max(Ratios):.3f}); {Plain} pass the plain desk path as without the arm")

    def TestTheProbedCapturesBlockEndToEnd(Self):
        # The reviewer's probe scenes (a 0.5 cm item) through CheckCapture: no tape in view, so nothing re-measures or passes.
        Image = np.full((TDC.Height, TDC.Width, 3), 90, np.uint8)
        for Scene in ((0.6, 34, (30, 8, 33), ((-45, 10), (58, 82)), 0.5), (0.7, 34, (35, 8, 33), ((-45, 15), (60, 84)), 0.5),
                      (0.7, 36, (35, 8, 33), ((-40, 10), (56, 80)), 0.7), (0.6, 36, (35, 8, 33), ((-45, 15), (60, 84)), 0.3)):
            with Self.subTest(scene=Scene):
                Depth, Arm = Self.Scene(*Scene)
                Result, New = Without(CD.CheckCapture(Depth, TDC.Intrinsics, Self.Model, 3, Image, Arm))
                Self.assertFalse(Result["ok"], Result)
                Self.assertIsNone(New)
                Self.assertNotIn("desk_occluded", Result)

    def TestAClearViewBeforeTheMoveCatchesItAlone(Self):
        # Had the session seen this desk clear before the move (seeded here: with a mat over most of it the desk path would not
        # pass it by itself), every cell seen in both, mat, item or bare desk, moved by the move: with the share of what could
        # be desk switched off the session check alone blocks it, and the same desk unmoved with the arm over it reads still.
        for Scene in ((0.6, 34, (30, 8, 33), ((-45, 10), (58, 82)), 0.5), (0.7, 36, (30, 9, 34), ((-45, 15), (60, 84)), 0.3),
                      (0.7, 34, (35, 8, 33), ((-40, 10), (56, 80)), 0.7)):
            with Self.subTest(scene=Scene):
                CD.ForgetSessionFrames()
                Clear, _ = Self.Scene(*Scene, Moved=False, WithArm=False)
                Self.assertTrue(CD.RememberSessionFrame(Clear, TDC.Intrinsics, Self.Model, None))
                Depth, Arm = Self.Scene(*Scene)
                with patch.object(CD, "MaxOccludedBeneathOfDeskShare", 1.0), patch.object(CD, "MaxOccludedBeneathShare", 1.0):
                    Result = CD.CheckDesk(Depth, TDC.Intrinsics, Self.Model, 3, Arm)
                    Still = CD.SessionChange(Self.Scene(*Scene, Moved=False)[0], TDC.Intrinsics, CD.VerifiedReference(Self.Model)[0],
                                             CD.ArmOcclusion(Depth, TDC.Intrinsics, Self.Model, Arm))
                Self.assertNotIn("desk_occluded", Result)
                Self.assertEqual(Result["desk_session"]["session_check"], "moved", Result["desk_session"])
                Self.assertAlmostEqual(Result["desk_session"]["offset_cm"], Scene[0], delta=0.1)
                Self.assertGreater(Result["desk_session"]["agreeing_share"], 0.8)
                Self.assertEqual(Still["session_check"], "still", Still)
                Self.assertLess(abs(Still["offset_cm"]), 0.05)


# --------------------------------------------------------------------------
# The cart-move fallback with the wrist rolled mid-task
# --------------------------------------------------------------------------

class MidTaskRollTests(Fresh):
    @classmethod
    def setUpClass(Class):
        Class.Model = dict(TDC.Verified(TDC.HeadCamera, [Desk], FloorZ=TDC.RoomFloorZ, TapePose=TapePose), marker_corners_gripper_cm=[Tape.tolist()])
        Class.Joints, Class.Point = TDC.ArmInView(TDC.HeadCamera, Tape)
        Tip, Frame = SC.GripperFrame(Class.Joints)
        Class.Rolled = Tip + Frame @ cv2.Rodrigues(np.array([0.0, 0.0, math.radians(60.0)]))[0] @ Tape
        Class.Arm = SyntheticArm(Class.Joints, Class.Point, 60.0)  # the wrist 683 steps from the calibration's
        Class.Moved = CartMoved(TDC.HeadCamera, **CartMove)

    def TestARolledWristNoLongerRefusesTheTape(Self):
        # The cart moved and the wrist rolled 60 deg: the roll rule alone would refuse the tape; the roll-aware tape finds it
        # where the rolled gripper puts it, so the camera is still where it was on the arm and the desk is re-measured.
        Gated = CD.TapeCheck(*CartView(TDC.HeadCamera, Self.Moved, Self.Rolled), TDC.Intrinsics, Self.Model, Self.Arm)
        Self.assertEqual(Gated["tape_check"], "pose_differs", Gated)
        Image, Depth = CartView(TDC.HeadCamera, Self.Moved, Self.Rolled)
        Result, New = Without(CD.CheckCapture(Depth, TDC.Intrinsics, Self.Model, 9, Image, Self.Arm))
        Self.assertTrue(Result["ok"], Result)
        Self.assertTrue(Result["desk_remeasured"])
        Self.assertIsNotNone(New)
        Self.assertEqual((Result["tape_check"], Result["tape_model"]), ("ok", "roll_aware"))
        Self.assertLess(Result["tape_error_cm"], 0.3)
        Self.assertAlmostEqual(Result["tape_roll_change_deg"], 60.0, delta=0.05)
        Self.assertIn("wrist_roll differs", Result["tape_roll_gated_reason"])

    def TestATapeOffTheRolledGripperStillBlocks(Self):
        # The tape where the unrolled gripper would have it: 1.34 cm from the rolled gripper's, past the cart limit.
        Image, Depth = CartView(TDC.HeadCamera, Self.Moved, Self.Point)
        Result, New = Without(CD.CheckCapture(Depth, TDC.Intrinsics, Self.Model, 9, Image, Self.Arm))
        Self.assertFalse(Result["ok"], Result)
        Self.assertIsNone(New)
        Self.assertEqual((Result["error_code"], Result["tape_check"], Result["tape_model"]), ("camera_moved", "moved", "roll_aware"), Result)
        Self.assertGreater(Result["tape_error_cm"], CD.CartTapeMaxErrorCm)

    def TestNoGripperFrameKeepsTheRollAdvice(Self):
        # An arm pose without the gripper link (as before the bridge reported it): the roll rule's words stand.
        Arm = {Key: Value for Key, Value in Self.Arm.items() if Key not in ("gripper_link", "reference")}
        Image, Depth = CartView(TDC.HeadCamera, Self.Moved, Self.Rolled)
        Result, New = Without(CD.CheckCapture(Depth, TDC.Intrinsics, Self.Model, 9, Image, Arm))
        Self.assertEqual((Result["ok"], Result["error_code"]), (False, "desk_changed_unverified"), Result)
        Self.assertIsNone(New)
        Self.assertIn("wrist roll or jaw", Result["error"])


Cases = (RealOccludedDeskTests, SyntheticOccludedDeskTests, VisibleShareFloorTests, MatMajorityShiftTests, ClutterOnMatTests, MidTaskRollTests)


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestOccludedDesk` runs the Test* methods too (unittest names this hook)."""
    Prefixed = unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return unittest.TestSuite(Prefixed.loadTestsFromTestCase(Case) for Case in Cases)


if __name__ == "__main__":
    Started = time.time()
    Loader = unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Suite = unittest.TestSuite(Loader.loadTestsFromTestCase(Case) for Case in Cases)
    Result = unittest.TextTestRunner(verbosity=2).run(Suite)
    print(f"{time.time() - Started:.0f} s")
    raise SystemExit(not Result.wasSuccessful())
