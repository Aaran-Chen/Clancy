"""Rolling the cart against the desk: the desk is re-measured, a camera that moved on the arm still blocks.

The arm, the depth camera and the basket ride one cart; the desk does not (CameraDrift's module
notes). Synthetic scenes use TestDepthCamera's ray-caster: a cart move is a rigid motion of the
world under the arm and camera together, so the desk is rendered from the moved pose while the
gripper tape stays where the calibrated camera sees the arm; a droop turns the camera on the arm.
The cart-move replays run on the 2026-10-03 captures and the 2026-10-02 21:09 calibration as fitted, kept in git
(TestFixtures/CartMoved-2026-10-03); the calibration run's frames and the 2026-09-28 droops are read from the
recordings (RecordedCaptures.py), and a recording that is not on disk fails the test, RECORDING MISSING, instead of
skipping it. The real camera_calibration.json is never written: saves go to a temp copy.

Run:  python TestCartMove.py
"""
import base64
import json
import math
import os
os.environ.setdefault("SO101_LOCATE_MODE", "vlm")  # this suite tests the voted multi-call locate; TestSingleCallLocate.py the default
import shutil
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

import cv2
import numpy as np

# The desk-remeasure archive and /locate's record go to the test captures root, never the real one.
os.environ["SO101_CAPTURES_ROOT"] = str(Path(__file__).resolve().parent / "TestArtifacts" / "SessionCaptures")
# The surfaces declared for the real arm (DeclaredSurfaces.json) are no part of these scenes.
os.environ["SO101_DECLARED_SURFACES"] = str(Path(__file__).resolve().parent / "TestArtifacts" / "NoDeclaredSurfaces.json")

import TestDepthCamera as TDC  # noqa: E402  (the ray-caster, the camera models and the tape scene)
import arm_geometry as Geo  # noqa: E402
import bridge_server as Bridge  # noqa: E402
import calibration  # noqa: E402
import CameraDrift as CD  # noqa: E402
import DepthGeometry as DG  # noqa: E402
import object_locator  # noqa: E402
import RecordedCaptures as RC  # noqa: E402
import RobotReference as RR  # noqa: E402
import SelfCalibration as SC  # noqa: E402

Root = Path(__file__).resolve().parent
Intrinsics, Width, Height, TableZ = TDC.Intrinsics, TDC.Width, TDC.Height, TDC.TableZ
Desk = TDC.Workspace()[0]  # the desk, standing on the room floor
Tape = np.array([1.2, -0.6, 0.8])
TapePose = {"wrist_roll_raw": 2048, "gripper_raw": 2100}
Recalibrating = ("recalibrat", "Automatic calibration", "Lock the camera head")


def CartMoved(Model, TiltDeg=0.0, TiltAxis=(1.0, 0.0, 0.0), YawDeg=0.0, Shift=(0.0, 0.0, 0.0), Pivot=(0.0, 30.0, TableZ)):
    """The camera Model's view of the world after the cart (arm and camera together) moved: a world point
    p sits at Turn (p - Pivot) + Pivot + Shift in the arm's display frame, Turn a tilt then a yaw."""
    Axis = np.asarray(TiltAxis, dtype=float) / np.linalg.norm(TiltAxis)
    Turn = cv2.Rodrigues(Axis * math.radians(TiltDeg))[0] @ cv2.Rodrigues(np.array([0.0, 0.0, math.radians(YawDeg)]))[0]
    Rotation, Translation, Mirror = DG.PoseFromModel(Model)
    Flip = np.diag(DG.MirrorVector if Mirror else np.ones(3))
    Anchor = np.asarray(Pivot, dtype=float)
    New = Rotation @ Flip @ Turn @ Flip
    return dict(Model, rvec=cv2.Rodrigues(New)[0].ravel().tolist(),
                tvec=(Translation + Rotation @ Flip @ (Anchor + np.asarray(Shift, dtype=float) - Turn @ Anchor)).tolist())


def TurnedOnArm(Model, Degrees, Axis):
    """The camera turned on the arm about its own centre and its own axis (camera frame: x right, y down,
    z forward); "x" is a droop or lift, "y" a turn, "z" a roll."""
    Rotation, Translation, _ = DG.PoseFromModel(Model)
    Centre = -Rotation.T @ Translation
    Turn = cv2.Rodrigues(np.eye(3)["xyz".index(Axis)] * math.radians(Degrees))[0]
    New = Turn @ Rotation
    return dict(Model, rvec=cv2.Rodrigues(New)[0].ravel().tolist(), tvec=(-New @ Centre).tolist())


def CartView(OnArm, Truth, Point, Items=(), Plate=60, Red=True):
    """Native image and depth: the desk (and Items) as Truth sees the world, the white gripper and its red
    tape where OnArm (the camera as it sits on the arm) sees the arm's tape Point."""
    Camera = DG.DisplayToCamera(Point, OnArm)[0]
    U, V = np.round(DG.Project(Camera, Intrinsics)[0]).astype(int)
    Image = np.full((Height, Width, 3), 90, np.uint8)
    Depth = TDC.RenderDepth(Truth, [Desk, *Items], FloorZ=TDC.RoomFloorZ)
    cv2.rectangle(Image, (U - Plate, V - Plate), (U + Plate, V + Plate), (235, 235, 235), -1)
    if Red:
        cv2.rectangle(Image, (U - 10, V - 10), (U + 10, V + 10), (30, 30, 215), -1)
    Depth[V - Plate:V + Plate + 1, U - Plate:U + Plate + 1] = round(Camera[2] * 10.0)
    return Image, Depth


def Without(Result):
    """A CheckCapture result with its re-measured baseline taken off: (result, baseline or None)."""
    Remeasured = CD.TakeRemeasuredReference(Result)
    return Result, Remeasured


# The cart rolled along the desk and settled 2 deg tilted on an uneven floor (2026-10-02: 1.7 deg).
CartMove = {"TiltDeg": 2.0, "TiltAxis": (1.0, 0.4, 0.0), "YawDeg": 8.0, "Shift": (5.0, -4.0, 0.4)}


class CartMoveTests(unittest.TestCase):
    """CheckCapture when the desk disagrees with its baseline: the gripper tape decides."""

    @classmethod
    def setUpClass(Class):
        Class.Model = dict(TDC.Verified(TDC.HeadCamera, [Desk], FloorZ=TDC.RoomFloorZ, TapePose=TapePose),
                           marker_corners_gripper_cm=[Tape.tolist()])
        Joints, Class.Point = TDC.ArmInView(TDC.HeadCamera, Tape)
        Class.Arm = {"ok": True, "joints": Joints.tolist(), "servos_raw": {"wrist_roll": 2048, "gripper": 2100}}
        Class.Moved = CartMoved(TDC.HeadCamera, **CartMove)

    def Check(Self, OnArm, Truth, Arm=None, Model=None, **View):
        Image, Depth = CartView(OnArm, Truth, Self.Point, **View)
        return CD.CheckCapture(Depth, Intrinsics, Model or Self.Model, 9, Image, Self.Arm if Arm is None else Arm)

    def TestStillCartPassesAsBefore(Self):
        Result = Self.Check(TDC.HeadCamera, TDC.HeadCamera)
        Self.assertTrue(Result["ok"], Result)
        Self.assertEqual(Result["tape_check"], "ok")
        Self.assertNotIn("desk_remeasured", Result)
        Self.assertNotIn(CD.RemeasuredKey, Result)

    def TestMovedCartIsReMeasuredWhenTheTapeAgrees(Self):
        Plain = CD.CheckDesk(CartView(TDC.HeadCamera, Self.Moved, Self.Point)[1], Intrinsics, Self.Model)
        Self.assertFalse(Plain["ok"], "the desk alone blocks a moved cart")
        Result, New = Without(Self.Check(TDC.HeadCamera, Self.Moved))
        Self.assertTrue(Result["ok"], Result)
        Self.assertTrue(Result["desk_remeasured"])
        Self.assertEqual(Result["tape_check"], "ok")
        Self.assertLess(Result["tape_error_cm"], 0.2)
        Self.assertLess(Result["angle_deg"], 0.05, "judged against the desk measured on this frame")
        Change = Result["desk_change"]
        Self.assertEqual(Change["error_code"], Plain["error_code"])
        Self.assertAlmostEqual(Change["plane_angle_deg"], 2.0, delta=0.1)
        Self.assertIn("desk re-measured on this capture", Result["reason"])
        Self.assertNotIn(CD.RemeasuredKey, Result)
        # The new baseline: this frame's desk, checksummed to the same camera, the old desk kept.
        Old = Self.Model["drift_reference"]
        Self.assertEqual(CD.VerifiedReference(dict(Self.Model, drift_reference=New))[1], "")
        for Key in ("version", "method", "fitted_at", "tape_pose", "tape_area_cm2", "keyframe"):
            Self.assertEqual(New[Key], Old.get(Key), Key)
        Self.assertEqual((New["frames"], New["desk_remeasured_from"]["seq"]), (1, 9))
        Self.assertEqual(New["desk_remeasured_from"]["tape_error_cm"], Result["tape_error_cm"])
        [Replaced] = New["desk_history"]
        Self.assertEqual((Replaced["desk_n"], Replaced["desk_d"], Replaced["source"]), (Old["desk_n"], Old["desk_d"], "calibration"))
        Self.assertEqual(Replaced["desk_map"], Old["desk_map"])
        # The desk the supports are judged on is the moved desk, in the arm's frame.
        Normal, Offset = CD.DisplayDeskPlane(dict(Self.Model, drift_reference=New))
        Moved = CartMoved(TDC.HeadCamera, **CartMove)
        for X, Y in ((-20.0, 35.0), (10.0, 50.0), (-40.0, 70.0)):
            World = np.array([X, Y, TableZ])
            Arm = DG.CameraToDisplay(DG.DisplayToCamera(World, Moved), TDC.HeadCamera)[0]
            Self.assertAlmostEqual(float(Normal @ Arm), Offset, delta=0.15, msg=(X, Y))
        # Captures after it pass the plain check; another re-measure starts the history.
        Saved = dict(Self.Model, drift_reference=New)
        Cup = (np.array([-18.0, 40.0, TableZ]), np.array([-12.0, 46.0, TableZ + 9.0]))
        Later = Self.Check(TDC.HeadCamera, Self.Moved, Model=Saved, Items=[Cup])
        Self.assertTrue(Later["ok"], Later)
        Self.assertNotIn("desk_remeasured", Later)
        Self.assertLess(Later["angle_deg"], 0.05)
        Again, Newer = Without(Self.Check(TDC.HeadCamera, CartMoved(Self.Moved, TiltDeg=-1.5), Model=Saved))
        Self.assertTrue(Again["desk_remeasured"], Again)
        Self.assertEqual([Entry["source"] for Entry in Newer["desk_history"]], ["remeasure", "calibration"])
        Self.assertEqual(Newer["desk_history"][0]["measured_at"], New["desk_remeasured_at"])

    def TestDroopOrTurnOnTheArmStillBlocks(Self):
        # A camera that drooped or turned 2-5 deg on the arm moves the tape 1.5-3.8 cm here, with or without
        # the cart moved too: the tape, not only the desk, calls it a moved camera.
        for Axis, Name in (("x", "droop"), ("y", "turn")):
            for Degrees in (2.0, -2.0, 5.0, -5.0):
                OnArm = TurnedOnArm(TDC.HeadCamera, Degrees, Axis)
                for Cart, Truth in (("cart still", OnArm), ("cart moved", CartMoved(OnArm, **CartMove))):
                    with Self.subTest(motion=Name, deg=Degrees, cart=Cart):
                        Result, New = Without(Self.Check(OnArm, Truth))
                        Self.assertFalse(Result["ok"], Result)
                        Self.assertEqual((Result["error_code"], Result["tape_check"]), ("camera_moved", "moved"), Result)
                        Self.assertGreater(Result["tape_error_cm"], 1.4)
                        Self.assertIn("Lock the camera head, then run Automatic calibration.", Result["error"])
                        Self.assertIsNone(New)
                        Self.assertNotIn("desk_remeasured", Result)
        # Once the desk moved, a tape within the general limit but past the cart's still blocks.
        Image, Depth = CartView(TDC.HeadCamera, Self.Moved, Self.Point)
        Distance = float(DG.DisplayToCamera(Self.Point, TDC.HeadCamera)[0][2])
        Off = np.roll(Image, round(0.88 * Intrinsics["fx"] / Distance), axis=1)  # the tape 0.88 cm from where the arm puts it
        Result, New = Without(CD.CheckCapture(Depth, Intrinsics, Self.Model, 9, Off, Self.Arm))
        Self.assertTrue(CD.CartTapeMaxErrorCm < Result["tape_error_cm"] <= CD.TapeMaxErrorCm, Result)
        Self.assertEqual((Result["ok"], Result["error_code"], Result["tape_check"]), (False, "camera_moved", "moved"))
        Self.assertIn(f"(limit {CD.CartTapeMaxErrorCm} cm)", Result["reason"])
        Self.assertIsNone(New)
        Still = CD.CheckCapture(CartView(TDC.HeadCamera, TDC.HeadCamera, Self.Point)[1], Intrinsics, Self.Model, 9, Off, Self.Arm)
        Self.assertEqual((Still["ok"], Still["tape_check"]), (True, "ok"), "a still desk keeps the general limit")

    def TestUnjudgeableTapeSaysWhatLetsItJudge(Self):
        Cases = [("torque off", {"ok": False, "reason": "torque off"}, {}, "torque is off", "Turn the torque on"),
                 ("moving", {"ok": False, "reason": "arm moved since the capture"}, {}, "was moving", "Hold the arm still"),
                 ("disconnected", {"ok": False, "reason": "arm not connected"}, {}, "not connected", "Connect the arm"),
                 ("roll turned", dict(Self.Arm, servos_raw={"wrist_roll": 2148, "gripper": 2100}), {}, "wrist roll or jaw", "Park the arm"),
                 ("tape hidden", None, {"Red": False}, "not seen where the arm puts it", "Park the arm")]
        for Name, Arm, View, Why, Fix in Cases:
            with Self.subTest(Name):
                Result, New = Without(Self.Check(TDC.HeadCamera, Self.Moved, Arm=Arm, **View))
                Self.assertFalse(Result["ok"])
                Self.assertIsNone(New)
                Self.assertEqual(Result["error_code"], "desk_changed_unverified", Result)
                Self.assertIn("The desk moved against the arm since calibration at 15:04 (was the cart moved?)", Result["error"])
                Self.assertIn(Why, Result["error"])
                Self.assertIn(Fix, Result["error"])
                Self.assertTrue(Result["error"].endswith("then send again."), Result["error"])
                for Words in Recalibrating:
                    Self.assertNotIn(Words, Result["error"])
                Self.assertAlmostEqual(Result["desk_change"]["plane_angle_deg"], 2.0, delta=0.1)

    def TestMoreThanACartMovesIsTheDesksBlock(Self):
        # A 6 deg tilt, or the desk 4 cm off, is not a cart on the floor: the desk check's own block stands.
        for Name, Truth in (("tilt 6", CartMoved(TDC.HeadCamera, TiltDeg=6.0)), ("4 cm", CartMoved(TDC.HeadCamera, Shift=(0.0, 0.0, 4.0)))):
            with Self.subTest(Name):
                Plain = CD.CheckDesk(CartView(TDC.HeadCamera, Truth, Self.Point)[1], Intrinsics, Self.Model)
                Result, New = Without(Self.Check(TDC.HeadCamera, Truth))
                Self.assertFalse(Result["ok"])
                Self.assertIsNone(New)
                Self.assertEqual((Result["error_code"], Result["error"]), (Plain["error_code"], Plain["error"]))
                Self.assertIn("more than moving the cart does", Result["reason"])
                Self.assertEqual(Result["tape_check"], "ok")

    def TestNoTapeOrNoDepthKeepsTheOldBlocks(Self):
        # A calibration without a tape offset has nothing to tell a moved cart from a moved camera.
        Bare = {Key: Value for Key, Value in Self.Model.items() if Key != "marker_corners_gripper_cm"}
        Image, Depth = CartView(TDC.HeadCamera, Self.Moved, Self.Point)
        Plain = CD.CheckDesk(Depth, Intrinsics, Bare, 9, Self.Arm)  # the cells the arm covers left out, as CheckCapture does
        Result = CD.CheckCapture(Depth, Intrinsics, Bare, 9, Image, Self.Arm)
        # The robot's reading rides beside a desk result (RobotReference.ShadowKey) and decides nothing.
        Self.assertEqual({Key: Value for Key, Value in Result.items() if Key not in ("check_ms", RR.ShadowKey)},
                         {Key: Value for Key, Value in Plain.items() if Key != "check_ms"})
        Self.assertEqual(CD.CheckCapture(None, Intrinsics, Self.Model, 9, Image, Self.Arm)["error_code"], "camera_check_failed")
        Self.assertEqual(CD.CheckCapture(Depth, Intrinsics, TDC.HeadCamera, 9, Image, Self.Arm)["error_code"], "camera_not_verified")
        # No desk in view at all is not a moved desk either: the robot checks the camera instead (this
        # arm pose carries no gripper frame, so the tape cannot judge and the camera is assumed; RobotReference).
        Floor = TDC.RenderDepth(TDC.HeadCamera, [], FloorZ=TDC.RoomFloorZ)
        Robot = CD.CheckCapture(Floor, Intrinsics, Self.Model, 9, Image, Self.Arm)
        Self.assertNotEqual(Robot.get("error_code"), "desk_changed_unverified")
        Self.assertEqual((Robot["ok"], Robot["alignment_source"], Robot["robot_check"]), (True, "robot", "assumed"), Robot)
        Self.assertEqual(Robot["desk_check"]["error_code"], "camera_check_failed")


class SaveTests(unittest.TestCase):
    """calibration.save_remeasured_desk and /locate: the re-measured desk is saved, and only it."""

    @classmethod
    def setUpClass(Class):
        Class.Model = dict(TDC.Verified(TDC.HeadCamera, [Desk], FloorZ=TDC.RoomFloorZ, TapePose=TapePose),
                           marker_corners_gripper_cm=[Tape.tolist()], fit_rms_px=0.8, observations_used=22)
        Joints, Class.Point = TDC.ArmInView(TDC.HeadCamera, Tape)
        Class.Arm = {"ok": True, "joints": Joints.tolist(), "servos_raw": {"wrist_roll": 2048, "gripper": 2100}}
        Class.Moved = CartMoved(TDC.HeadCamera, **CartMove)

    def setUp(Self):
        Self.Folder = Path(tempfile.mkdtemp(prefix="CartMove", dir=TDC.Artifacts))
        Self.Archive = Path(os.environ["SO101_CAPTURES_ROOT"])
        Self.Patch = patch.object(calibration, "CALIBRATION_PATH", Self.Folder / "camera_calibration.json")
        Self.Patch.start()
        calibration.save_camera_model(Self.Model)

    def tearDown(Self):
        Self.Patch.stop()
        shutil.rmtree(Self.Folder, ignore_errors=True)

    def Saved(Self):
        return json.loads((Self.Folder / "camera_calibration.json").read_text(encoding="utf-8"))

    def Remeasure(Self, Model=None):
        Image, Depth = CartView(TDC.HeadCamera, Self.Moved, Self.Point)
        return Without(CD.CheckCapture(Depth, Intrinsics, Model or Self.Model, 9, Image, Self.Arm))

    def TestSaveReplacesOnlyTheDeskBaseline(Self):
        Before = Self.Saved()
        Result, New = Self.Remeasure()
        Archived = set(Self.Archive.rglob("desk_remeasure/*.json"))
        Self.assertEqual(calibration.save_remeasured_desk(New, Self.Model["drift_reference"]["sha256"], Result), (True, ""))
        After = Self.Saved()
        Self.assertEqual(After["camera_model"]["drift_reference"], New)
        Self.assertEqual({Key: Value for Key, Value in After["camera_model"].items() if Key != "drift_reference"},
                         {Key: Value for Key, Value in Before["camera_model"].items() if Key != "drift_reference"})
        Self.assertEqual({Key: Value for Key, Value in After.items() if Key != "camera_model"},
                         {Key: Value for Key, Value in Before.items() if Key != "camera_model"})
        Self.assertEqual(CD.VerifiedReference(After["camera_model"])[1], "")
        Self.assertEqual(calibration.camera_model(), After["camera_model"])
        Self.assertEqual(list(Self.Folder.glob("*.tmp")), [], "written whole: no temporary file left")
        # The file as it was, archived before the write.
        [Record] = set(Self.Archive.rglob("desk_remeasure/*.json")) - Archived
        Stored = json.loads(Record.read_text(encoding="utf-8"))
        Self.assertEqual((Stored["kind"], Stored["previous_calibration"], Stored["drift_reference"]), ("DeskRemeasured", Before, New))
        # A capture checked against the replaced baseline does not overwrite the newer one.
        _, Stale = Self.Remeasure()
        Self.assertEqual(calibration.save_remeasured_desk(Stale, Self.Model["drift_reference"]["sha256"]),
                         (False, "the desk baseline changed since this capture was checked"))
        Self.assertEqual(Self.Saved(), After)

    def TestSaveNeverLandsOnAnotherCalibration(Self):
        _, New = Self.Remeasure()
        Refit = dict(TDC.Turned(Self.Model, 0.3), fitted_at="2026-09-28T16:00:00")
        calibration.save_camera_model(Refit)
        Before = Self.Saved()
        Self.assertEqual(calibration.save_remeasured_desk(New, Self.Model["drift_reference"]["sha256"]),
                         (False, "the camera calibration changed since this capture was checked"))
        Self.assertEqual(Self.Saved(), Before)
        calibration.clear_probe_points()
        Self.assertEqual(calibration.save_remeasured_desk(New, Self.Model["drift_reference"]["sha256"]),
                         (False, "the camera is no longer calibrated"))

    def Locate(Self, Image, Depth, Boxes):
        Pick, Detect = MagicMock(return_value="test-model"), MagicMock(return_value=Boxes)
        Capture = cv2.flip(Image, 1)  # the browser sends mirrored frames
        with patch.object(calibration, "pixel_to_plane", side_effect=lambda U, V, Mirrored=None: SC.PixelToPlane(Self.Model, U, V, Mirrored)), \
                patch.object(calibration, "status", return_value={"calibrated": True, "rms_cm": 0.3}), \
                patch.object(object_locator, "_pick_model", Pick), patch.object(object_locator, "_detect_once", Detect), \
                patch.object(object_locator, "_refine_detection", return_value=None):
            return object_locator.locate_objects(TDC.Encode(Capture), depth={"depth_mm": Depth, "intrinsics": dict(Intrinsics), "mirrored": True,
                                                                            "seq": 77}, mirrored=True, arm=Self.Arm), Detect

    def TestLocateSavesTheDeskAndMeasuresOnIt(Self):
        # A cup on the moved desk: /locate re-measures and saves the desk, and the cup stands on it.
        Cup = (np.array([-18.0, 40.0, TableZ]), np.array([-12.0, 46.0, TableZ + 9.0]))
        Image, Depth = CartView(TDC.HeadCamera, Self.Moved, Self.Point, Items=[Cup])
        Box = TDC.CaptureBox(Self.Moved, *Cup)
        Result, Detect = Self.Locate(Image, Depth, [TDC.Detected("cup", Box)])
        Self.assertTrue(Result["ok"], Result)
        Detect.assert_called()
        Check = Result["camera_check"]
        Self.assertEqual((Check["desk_remeasured"], Check["desk_saved"]), (True, True), Check)
        Self.assertNotIn(CD.RemeasuredKey, Check)
        Saved = Self.Saved()["camera_model"]["drift_reference"]
        Self.assertEqual(Saved["desk_remeasured_from"]["seq"], 77)
        [Measured] = Result["objects"]
        # The cup's base, in the arm's frame: the moved desk under it.
        Base = DG.CameraToDisplay(DG.DisplayToCamera(np.array([-15.0, 43.0, TableZ]), Self.Moved), TDC.HeadCamera)[0]
        Self.assertAlmostEqual(Measured["z"], Base[2], delta=0.4, msg=Measured)
        Self.assertAlmostEqual(Measured["height_cm"], 9.0, delta=0.5)
        Self.assertEqual(Measured["support_branch"], "desk")
        # The next capture passes the plain check against the saved desk.
        Again, _ = Self.Locate(Image, Depth, [TDC.Detected("cup", Box)])
        Self.assertTrue(Again["ok"], Again)
        Self.assertNotIn("desk_remeasured", Again["camera_check"])
        Self.assertEqual(Self.Saved()["camera_model"]["drift_reference"], Saved)

    def TestLocateBlocksAnUnjudgedTapeBeforeAnyModelCall(Self):
        Image, Depth = CartView(TDC.HeadCamera, Self.Moved, Self.Point, Red=False)
        Before = Self.Saved()
        Result, Detect = Self.Locate(Image, Depth, [])
        Detect.assert_not_called()
        Self.assertEqual((Result["ok"], Result["error_code"]), (False, "desk_changed_unverified"), Result)
        Self.assertEqual(Self.Saved(), Before)

    def TestDepthCheckEndpointNeverSaves(Self):
        Image, Depth = CartView(TDC.HeadCamera, Self.Moved, Self.Point)
        Camera = TDC.OC.DepthCamera(Opener=TDC.FakeSource)
        Camera.IntrinsicsValue = dict(Intrinsics)
        for _ in range(TDC.OC.PinMedianFrames):
            Camera.Push(Image, Depth)
        Before = Self.Saved()
        with patch.object(Bridge, "depth_camera", Camera), patch.object(Bridge.controller, "CaptureArmPose", return_value=Self.Arm):
            Check = Bridge.depthcam_check()["camera_check"]
        Self.assertEqual((Check["ok"], Check.get("desk_remeasured"), Check.get("desk_saved")), (True, True, False), Check)
        Self.assertNotIn(CD.RemeasuredKey, Check)
        Self.assertEqual(Self.Saved(), Before)


# --------------------------------------------------------------------------
# 2026-10-02: the 21:09 calibration, then the cart rolled toward the desk
# --------------------------------------------------------------------------
# The 21:15 and 21:46 tasks of that night were blocked, and these replays were written on their captures; those were
# deleted on 2026-10-03 (audit 2 H3), so they skipped from then on. Rebuilt on the next morning's two captures of the
# same cart position, kept in git (TestFixtures/CartMoved-2026-10-03, with the calibration as fitted): 07:46:34, where
# the live bridge re-measured the desk on the tape, and 08:07:54. The calibration run's own frames are read from the
# recordings (RecordedCaptures; a missing recording fails, it no longer skips).
Fixture = json.loads(RC.Fixture("CartMoved-2026-10-03", "Captures.json").read_text(encoding="utf-8"))
Run = RC.Day("2026-10-02") / "selfcal" / "runs" / "20261002_210623_5e44514af6d7"
# The arm parked through both: its raw steps (wrist roll and jaw as the calibration's tape pose).
Parked = Fixture["captures"][0]["raws"]


def ArmFromRaw(Raw):
    """CaptureArmPose's joints for raw servo steps, by the bridge's own conversion (a fresh, unconnected controller)."""
    Controller = Bridge.BridgeController()
    for Name, Value in Raw.items():
        Controller.servos[Name].present_raw, Controller.servos[Name].gui_angle_raw = Value, None
    Offset = Controller.WorkspaceOffset()
    Joints = Controller._display_joints(Geo.forward_joint_positions(Controller.arm_geometry, Controller._current_ik_angles()))
    return {"ok": True, "joints": [[P["x"] + Offset["x"], P["y"] + Offset["y"], P["z"] + Offset["z"]] for P in Joints], "servos_raw": dict(Raw)}


def TurnedFrame(Depth, Image, Camera, Rotation):
    """The depth and colour a camera turned by Rotation about its own centre would have seen."""
    Rays = np.concatenate([DG.RayMap(Camera).astype(np.float64), np.ones((Height, Width, 1))], axis=-1)
    Moved = (Rays * (Depth / 10.0)[..., None])[Depth > 0] @ Rotation.T
    U, V = np.round(DG.Project(Moved, Camera)).astype(int).T
    Inside = (U >= 0) & (U < Width) & (V >= 0) & (V < Height) & (Moved[:, 2] > 1)
    Out = np.full((Height, Width), 65535, np.uint16)
    np.minimum.at(Out, (V[Inside], U[Inside]), np.round(Moved[Inside, 2] * 10.0).astype(np.uint16))
    Source = DG.Project(Rays.reshape(-1, 3) @ Rotation, Camera).reshape(Height, Width, 2).astype(np.float32)
    return np.where(Out == 65535, 0, Out).astype(np.uint16), cv2.remap(Image, Source[..., 0], Source[..., 1], cv2.INTER_LINEAR)


class RealCartMoveTests(unittest.TestCase):
    """The tasks after the cart was rolled toward the desk, replayed on the recorded frames."""

    @classmethod
    def setUpClass(Class):
        Class.Model = Fixture["model"]
        Class.Camera = {Key: Class.Model[Key] for Key in ("fx", "fy", "cx", "cy", "dist")} | {"width": Width, "height": Height}
        Class.Arm = ArmFromRaw(Parked)
        Folder = RC.Fixture("CartMoved-2026-10-03")
        Class.Frames = {Capture["clock"]: (cv2.imread(str(Folder / Capture["depth"]), cv2.IMREAD_UNCHANGED), cv2.imread(str(Folder / Capture["image"])),
                                           Capture["seq"]) for Capture in Fixture["captures"]}

    def TestCartMovedTasksPassOnTheGripperTape(Self):
        Remeasured = {}
        for Clock, (Depth, Image, Seq) in Self.Frames.items():
            with Self.subTest(capture=Clock):
                # On the desk alone, as the 2026-10-02 tasks were blocked: 51-52% of the calibrated desk agreeing, 2.2-2.4 deg.
                Plain = CD.CheckDesk(Depth, Self.Camera, Self.Model, Seq)
                Self.assertEqual((Plain["ok"], Plain["error_code"]), (False, "camera_check_failed"))
                Self.assertLess(Plain["desk_agreeing_share"], CD.MinAgreeingShare)
                Result, New = Without(CD.CheckCapture(Depth, Self.Camera, Self.Model, Seq, Image, Self.Arm))
                Self.assertTrue(Result["ok"], Result)
                Self.assertTrue(Result["desk_remeasured"])
                Self.assertEqual(Result["tape_check"], "ok")
                Self.assertLess(Result["tape_error_cm"], 0.6)  # 0.31 and 0.34 cm: the camera sits on the arm as calibrated
                Self.assertLess(Result["angle_deg"], 0.05)
                Self.assertTrue(1.5 < Result["desk_change"]["plane_angle_deg"] < 2.0, Result["desk_change"])
                Self.assertLess(abs(Result["desk_change"]["plane_offset_cm"]), 0.2)
                Self.assertEqual(CD.VerifiedReference(dict(Self.Model, drift_reference=New))[1], "")
                Self.assertEqual(New["fitted_at"], "2026-10-02T21:09:22")
                Remeasured[Clock] = New
        # As live at 07:46:34 (its recorded camera_check): re-measured, the tape 0.31 cm off, the desk 1.73 deg / +0.10 cm.
        Recorded = Fixture["captures"][0]["recorded_camera_check"]
        Self.assertEqual((Recorded["desk_remeasured"], Recorded["tape_check"], Recorded["tape_error_cm"]), (True, "ok", 0.31))
        # The cart did not move between the two tasks: each frame passes the plain check on the other's desk.
        for Clock, (Depth, Image, Seq) in Self.Frames.items():
            for Other, New in Remeasured.items():
                if Other != Clock:
                    Result = CD.CheckCapture(Depth, Self.Camera, dict(Self.Model, drift_reference=New), Seq, Image, Self.Arm)
                    Self.assertTrue(Result["ok"], Result)
                    Self.assertNotIn("desk_remeasured", Result)
                    Self.assertLess(Result["angle_deg"], 0.1)

    def TestCartMovedTasksWithoutAJudgeableTapeSayWhy(Self):
        Depth, Image, Seq = Self.Frames["07:46:34"]
        U, V = (int(round(Value)) for Value in Fixture["captures"][0]["recorded_camera_check"]["tape_predicted_px"])
        Covered = Image.copy()
        Covered[V - 35:V + 35, U - 40:U + 40] = (200, 200, 200)  # a hand over the gripper's tape
        for Name, Arm, Frame, Why in (("torque off", {"ok": False, "reason": "torque off"}, Image, "torque is off"),
                                      ("roll turned", dict(Self.Arm, servos_raw=dict(Parked, wrist_roll=1700)), Image, "wrist roll or jaw"),
                                      ("tape covered", Self.Arm, Covered, "not seen where the arm puts it")):
            with Self.subTest(Name):
                Result, New = Without(CD.CheckCapture(Depth, Self.Camera, Self.Model, Seq, Frame, Arm))
                Self.assertIsNone(New)
                Self.assertEqual((Result["ok"], Result["error_code"]), (False, "desk_changed_unverified"), Result)
                Self.assertIn("since calibration at 21:09 (was the cart moved?)", Result["error"])
                Self.assertIn(Why, Result["error"])
                for Words in Recalibrating:
                    Self.assertNotIn(Words, Result["error"])

    def TestCalibrationFramesPassThePlainDeskCheck(Self):
        RC.Need(Run / "observations", What="the 2026-10-02 21:09 calibration run's frames")
        Frames = sorted((Run / "observations").glob("*_pose*_depth*.png"))
        Self.assertEqual(len(Frames), 75)
        for Path_ in Frames:
            Result = CD.CheckDesk(cv2.imread(str(Path_), cv2.IMREAD_UNCHANGED), Self.Camera, Self.Model)
            Self.assertTrue(Result["ok"], (Path_.name, Result))
            Self.assertLess(Result["angle_deg"], 0.2)

    def TestTheRealCameraDroopingOrTurningOnTheArmBlocks(Self):
        # The 07:46:34 frame as the camera would have seen it turned on the arm (its tape 0.31 cm off at rest). Droops and
        # turns of 2-5 deg read 1.21-3.89 cm off and rolls about the optical axis of 5 deg 1.07-1.27 cm: all block. Smaller
        # turns stay near the 0.75 cm limit on this frame (a 1 deg droop 0.70-0.79 cm, rolls of 1-3 deg 0.31-0.82 cm), as
        # CameraDrift's module notes say of small rolls; they are not asserted here.
        Depth, Image, Seq = Self.Frames["07:46:34"]
        for Axis, Name, Turns in ((0, "droop", (2.0, -2.0, 3.0, -3.0, 5.0, -5.0)), (1, "turn", (2.0, -2.0, 3.0, -3.0, 5.0, -5.0)),
                                  (2, "roll", (5.0, -5.0))):
            for Degrees in Turns:
                with Self.subTest(motion=Name, deg=Degrees):
                    Turned, Colour = TurnedFrame(Depth, Image, Self.Camera, cv2.Rodrigues(np.eye(3)[Axis] * math.radians(Degrees))[0])
                    Result, New = Without(CD.CheckCapture(Turned, Self.Camera, Self.Model, Seq, Colour, Self.Arm))
                    Self.assertIsNone(New)
                    Self.assertEqual((Result["ok"], Result["error_code"], Result["tape_check"]), (False, "camera_moved", "moved"), Result)
                    Self.assertGreater(Result["tape_error_cm"], CD.CartTapeMaxErrorCm)

    def TestATurnedCameraWithNoArmPoseNeverPasses(Self):
        # The 2026-09-28 droops (RealDroopTests) had no arm pose or colour frame kept, so only the desk could judge them:
        # none passed. The same on this frame turned 1-5 deg every way, with the arm's torque off: every one blocks, as
        # camera_moved past the cart's limits and as desk_changed_unverified within them, and none re-measures the desk.
        Depth, Image, Seq = Self.Frames["07:46:34"]
        Codes = set()
        for Axis, Name in ((0, "droop"), (1, "turn"), (2, "roll")):
            for Degrees in (1.0, -1.0, 2.0, -2.0, 3.0, -3.0, 5.0, -5.0):
                with Self.subTest(motion=Name, deg=Degrees):
                    Turned, _ = TurnedFrame(Depth, Image, Self.Camera, cv2.Rodrigues(np.eye(3)[Axis] * math.radians(Degrees))[0])
                    Result, New = Without(CD.CheckCapture(Turned, Self.Camera, Self.Model, Seq, None, {"ok": False, "reason": "torque off"}))
                    Self.assertFalse(Result["ok"], Result)
                    Self.assertIsNone(New)
                    Self.assertIn(Result["error_code"], ("camera_moved", "desk_changed_unverified"), Result)
                    Codes.add(Result["error_code"])
        Self.assertEqual(Codes, {"camera_moved", "desk_changed_unverified"})


Droop0928 = RC.Day("2026-09-28")


class RealDroopTests(unittest.TestCase):
    def TestTheDroopsOfSeptember28StayBlocked(Self):
        # The head drooped 1.3-13 deg after the 15:04 fit. No arm pose or colour frame was kept with those
        # captures, so the tape cannot judge them: past the cart limits the desk's camera_moved stands,
        # within them (15:19, 19:02: 1.6 and 2.4 deg) they block as desk_changed_unverified. None passes.
        # The captures (recording/images) were deleted on 2026-10-03: this fails, RECORDING MISSING, until they are
        # restored. RealCartMoveTests.TestATurnedCameraWithNoArmPoseNeverPasses checks the rule on a re-rendered frame.
        RC.Need(Droop0928 / "selfcal" / "runs" / TDC.RealRun.name, TDC.RealModel,
                *(Droop0928 / "recording" / "images" / f"{Sha}.png" for _, Sha, _, _ in TDC.RealCaptures),
                What="the 2026-09-28 droop captures (recording/images) and the 15:04 fit's run")
        Model = json.loads(TDC.RealModel.read_text(encoding="utf-8"))["camera_model"]
        Camera = {Key: Model[Key] for Key in ("fx", "fy", "cx", "cy", "dist")} | {"width": Width, "height": Height}
        Planes = []
        for Capture in sorted((Droop0928 / "selfcal" / "runs" / TDC.RealRun.name / "observations").glob("*_pose*.json")):
            Record = json.loads(Capture.read_text(encoding="utf-8"))
            for Name in Record["depth_frames_png"] or []:
                Plane = CD.DeskPlane(cv2.imread(str(Capture.parent / Name), cv2.IMREAD_UNCHANGED), Record["depth_intrinsics"])
                if Plane is not None:
                    Planes.append(dict(Plane, pose_index=Record["pose_index"], received_at=Record["received_at"], file=str(Capture.parent / Name)))
        Reference, Problem = CD.BuildReference(Planes, Model, Intrinsics=Camera)
        Self.assertIsNone(Problem)
        Model = dict(Model, drift_reference=Reference)
        Codes = {}
        for Clock, Sha, Near, Far in TDC.RealCaptures:
            Depth = TDC.DecodeTurbo(Droop0928 / "recording" / "images" / f"{Sha}.png", Near, Far)
            Result, New = Without(CD.CheckCapture(Depth, Camera, Model, 1, None, {"ok": False, "reason": "torque off"}))
            Self.assertFalse(Result["ok"], (Clock, Result))
            Self.assertIsNone(New)
            Codes[Clock] = Result["error_code"]
        Self.assertEqual({Clock for Clock, Code in Codes.items() if Code == "desk_changed_unverified"}, {"15:19:15", "19:02:02"}, Codes)
        Self.assertEqual(set(Codes.values()), {"camera_moved", "desk_changed_unverified"})


Cases = (CartMoveTests, SaveTests, RealCartMoveTests, RealDroopTests)


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestCartMove` runs the Test* methods too (unittest names this hook; audit 2 L7: they ran only
    through this file's own __main__)."""
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
