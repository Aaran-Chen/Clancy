"""Grab from anywhere: a desk in view that disagrees is decided by the robot's two witnesses, and the basket rim they need
is recorded from verified captures (CameraDrift.CheckCapture, RobotReference). No hardware.

On 2026-10-25 at 22:15:25Z "grab the pille bottle of the shelf" was refused at the bookshelf cabinet as camera_check_failed
("much of the desk looks different from calibration ... lock the camera head and recalibrate"): the plane the desk search
found there (201 calibrated desk cells) agreed 55%, 4.17 deg / +3.77 cm off the calibrated desk, more than a cart move, while
the roll-aware gripper tape sat 0.58 cm from where the arm puts it and the basket rim had nothing to compare with. Here
(TestFixtures/ShelfCapture-2026-10-25, kept in git, with the calibration as fitted and the desk capture of 22:08:09Z):

- with no rim known, the shelf capture blocks with what lets the rim judge, never "recalibrate"; the desk capture of 22:08
  (desk verified, the roll-aware tape 0.21 cm) carries its rim, /locate records it, and the shelf capture then passes as
  verified by the robot (tape and rim), the old desk no support; through /locate end to end the pill bottle is measured on
  the cabinet top (its reply kept in TestArtifacts/ShelfCapture for the app-side planner), the books against its back no
  longer withholding it (DepthGeometry.SupportSides);
- the same request at 02:10:18Z (2026-10-26 UTC, the bottle elsewhere on the cabinet top, books against its back and left)
  was withheld live as "no level surface measured around the object to stand on"; replayed through the single call as
  live, it is measured now (Capture.json shelf_0210; its reply kept as TestArtifacts/ShelfCapture/Locate0210.json);
- the calibration's own frames show enough of the rim too (every pose of its run, RobotReference._RimFromEveryPose), so the
  shelf capture passes on them alone (read from the recordings: RecordedCaptures; missing, it fails RECORDING MISSING);
- real camera moves still block: the shelf frame re-rendered as the camera panned 1.5-2 deg, turned 4.16 deg about its own
  y axis (the 2026-10-06 15:02 hand on the lens), tilted or rolled 2-4 deg, or slid 1-2 cm on the cart; with no desk in view
  (the 2026-10-06 15:01 cabinet frame, from the recordings) the same, pans of 2 deg there. (A 1-1.5 deg pan, a 1 deg tilt
  or a 1 cm slide toward the tape's standing offset can pass: about what tape and rim resolve, as the cart-move path's
  tape alone; the notes in RobotReference.)
- a rim is recorded only from a capture that verified the camera in full, only while it agrees with what is known of the
  rim, and only for the calibration it was checked against: a new calibration starts fresh, a desk re-measure does not.

Run:  python TestShelfCapture.py
"""
import json
import math
import os
import shutil
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

import cv2
import numpy as np

Here = Path(__file__).resolve().parent
# Records go to the test captures root, never the real one; no declared surfaces; the rim records to a folder per test.
os.environ["SO101_CAPTURES_ROOT"] = str(Here / "TestArtifacts" / "SessionCaptures")
os.environ["SO101_DECLARED_SURFACES"] = str(Here / "TestArtifacts" / "NoDeclaredSurfaces.json")
os.environ.setdefault("SO101_LOCATE_MODE", "vlm")

import TestDepthCamera as TDC  # noqa: E402  (Encode, Detected)
from TestOccludedDesk import ArmFromRaw  # noqa: E402  (CaptureArmPose from raw steps, with the link boxes)
import calibration  # noqa: E402
import CameraDrift as CD  # noqa: E402
import DepthGeometry as DG  # noqa: E402
import object_locator  # noqa: E402
import RecordedCaptures as RC  # noqa: E402
import RobotReference as RR  # noqa: E402
import SelfCalibration as SC  # noqa: E402

Fixture = json.loads(RC.Fixture("ShelfCapture-2026-10-25", "Capture.json").read_text(encoding="utf-8"))
Folder = RC.Fixture("ShelfCapture-2026-10-25")
Model = Fixture["model"]
Camera = {Key: Model[Key] for Key in ("fx", "fy", "cx", "cy", "dist")} | {"width": 1280, "height": 720}
Width, Height = 1280, 720
Artifacts = Here / "TestArtifacts" / "ShelfCapture"
# The 2026-10-14 calibration's own run (its frames hold the rim) and the 2026-10-06 cabinet frame, from the recordings.
CalibrationRun = Path(Model["drift_reference"]["keyframe"]).parent
Cabinet = RC.Day("2026-10-06")
Cabinet1501 = {"depth": Cabinet / "recording" / "depth" / "c1e55020492ffe59b3a27f09a5f24af9e3b1491e.png", "image": Cabinet / "080101_173_vision.jpg",
               "raws": {"shoulder_pan": 3487, "shoulder_lift": 3173, "elbow_flex": 2686, "wrist_flex": 864, "wrist_roll": 1420, "gripper": 699}}


def Load(Capture):
    return (cv2.imread(str(Folder / Capture["depth"]), cv2.IMREAD_UNCHANGED), cv2.imread(str(Folder / Capture["image"])),
            ArmFromRaw(Capture["raws"], RR.ReferenceRaw(Model)))


def MovedFrame(Depth, Image, Intrinsics, Rotation, Shift=(0.0, 0.0, 0.0)):
    """The depth and colour a camera moved on the robot would have seen: every depth pixel moved rigidly (P' = R P + s,
    camera frame, cm) and splatted 2 x 2 with its colour, the nearest winning; a pixel nothing lands on has no depth and
    is grey (TestCartMove.TurnedFrame keeps the colour of pure turns; a slide moves near and far apart)."""
    Rays = np.concatenate([DG.RayMap(Intrinsics).astype(np.float64), np.ones((Height, Width, 1))], axis=-1)
    Valid = Depth > 0
    Points = (Rays * (Depth / 10.0)[..., None])[Valid] @ np.asarray(Rotation).T + np.asarray(Shift, dtype=np.float64)
    Colour = Image[Valid]
    U, V = np.floor(DG.Project(Points, Intrinsics)).astype(int).T
    Order = np.argsort(-Points[:, 2])  # far first: the nearer overwrite
    Z = np.round(Points[:, 2] * 10.0).astype(np.int64)
    OutDepth, OutImage = np.zeros((Height, Width), np.uint16), np.full_like(Image, 128)
    for DU in (0, 1):
        for DV in (0, 1):
            UU, VV = U + DU, V + DV
            Inside = (UU >= 0) & (UU < Width) & (VV >= 0) & (VV < Height) & (Points[:, 2] > 1)
            Pick = Order[Inside[Order]]
            OutDepth[VV[Pick], UU[Pick]] = Z[Pick]
            OutImage[VV[Pick], UU[Pick]] = Colour[Pick]
    return OutDepth, OutImage


def Moves(Model_, Pans=(1.5, -1.5, 2.0, -2.0)):
    """Camera motions on the robot (name, rotation, shift cm in the camera frame) that must block."""
    Rotation, _, _ = DG.PoseFromModel(Model_)
    Up = Rotation[:, 2] / np.linalg.norm(Rotation[:, 2])  # display up, in the camera frame: a pan turns about it
    Turn = lambda Axis, Degrees: cv2.Rodrigues(np.asarray(Axis, dtype=np.float64) * math.radians(Degrees))[0]
    Rows = [(f"pan {Degrees:+g} deg", Turn(Up, Degrees), (0, 0, 0)) for Degrees in Pans]
    Rows += [(f"head turn {Degrees:+g} deg about the camera's y", Turn((0, 1, 0), Degrees), (0, 0, 0)) for Degrees in (4.16, -4.16)]
    Rows += [(f"tilt {Degrees:+g} deg", Turn((1, 0, 0), Degrees), (0, 0, 0)) for Degrees in (2.0, -2.0)]
    Rows += [(f"roll {Degrees:+g} deg", Turn((0, 0, 1), Degrees), (0, 0, 0)) for Degrees in (2.0, -2.0, 4.0, -4.0)]
    Rows += [(f"slide {Shift} cm", np.eye(3), Shift) for Shift in ((2.0, 0, 0), (-2.0, 0, 0), (0, 2.0, 0), (0, -2.0, 0), (0, 0, 2.0), (0, 0, -2.0))]
    return Rows


class RimFolder(unittest.TestCase):
    """Each test records rims to a folder of its own (SO101_RIM_BASELINE), and remembers no clear view of the desk."""

    def setUp(Self):
        Artifacts.mkdir(parents=True, exist_ok=True)
        Self.RimFolder = Path(tempfile.mkdtemp(prefix="Rim", dir=Artifacts))
        Self.Env = patch.dict(os.environ, {"SO101_RIM_BASELINE": str(Self.RimFolder / "rim_baseline.json")})
        Self.Env.start()
        CD.ForgetSessionFrames()

    def tearDown(Self):
        Self.Env.stop()
        shutil.rmtree(Self.RimFolder, ignore_errors=True)
        CD.ForgetSessionFrames()

    def Check(Self, Depth, Image, Arm, Seq=1, Model_=None):
        Result = CD.CheckCapture(Depth, Camera, Model_ or Model, Seq, Image, Arm)
        return Result, CD.TakeRemeasuredReference(Result), RR.TakeRimRecord(Result)


class ShelfReplayTests(RimFolder):
    """The 22:15:25 shelf capture, and the 22:08:09 desk capture before it."""

    @classmethod
    def setUpClass(Class):
        Class.Shelf, Class.Desk = Load(Fixture["shelf"]), Load(Fixture["desk_view"])

    def RecordTheDeskView(Self):
        """The 22:08 capture through CheckCapture as live: the desk verifies it, the roll-aware tape too; its rim is recorded."""
        Result, New, Record = Self.Check(*Self.Desk, Seq=Fixture["desk_view"]["seq"])
        Self.assertEqual((Result["ok"], Result.get("error_code"), Result.get("alignment_source")), (True, None, None), Result)
        Self.assertIsNone(New)
        Self.assertEqual(Result["reason"], Fixture["desk_view"]["recorded_camera_check"]["reason"])
        Shadow = Result[RR.ShadowKey]
        Self.assertEqual(Shadow["tape_check"], "ok")
        Self.assertAlmostEqual(Shadow["tape_error_cm"], Fixture["desk_view"]["recorded_camera_check"][RR.ShadowKey]["tape_error_cm"], delta=0.05)
        Self.assertLessEqual(Shadow["tape_error_cm"], RR.RimRecordTapeMaxCm)
        Self.assertIsNotNone(Record, "a desk pass with the tape within the record limit carries its rim")
        Self.assertEqual(Record["seq"], Fixture["desk_view"]["seq"])
        Self.assertGreaterEqual(sum(Bin[0] is not None for Bin in Record["profile"]), RR.RimMinBins)
        Self.assertEqual(RR.RecordRim(Record, Model), (True, ""))
        return Record

    def TestAsRecordedTheShelfCaptureWasRefused(Self):
        Recorded = Fixture["shelf"]["recorded_camera_check"]
        Self.assertEqual((Recorded["ok"], Recorded["error_code"], Recorded["desk_agreeing_share"]), (False, "camera_check_failed", 0.552))
        Self.assertIn("more than moving the cart does", Recorded["reason"])
        Self.assertIn("recalibrate", Fixture["shelf"]["recorded_error"])
        Self.assertEqual((Recorded[RR.ShadowKey]["robot_check"], Recorded[RR.ShadowKey]["rim_check"]), ("verified", "unavailable"))
        # Replayed, the desk path decides as it did live (the arm reconstructed: the tape where the live check predicted it).
        Desk = CD._DeskCapture(Self.Shelf[0], Camera, Model, Fixture["shelf"]["seq"], Self.Shelf[1], Self.Shelf[2])[0]
        Self.assertEqual({Key: Desk.get(Key) for Key in ("ok", "error_code", "desk_cells", "desk_agreeing_share", "plane_angle_deg")},
                         {Key: Recorded.get(Key) for Key in ("ok", "error_code", "desk_cells", "desk_agreeing_share", "plane_angle_deg")})
        Self.assertEqual(Desk["tape_predicted_px"], Recorded["tape_predicted_px"])
        Self.assertAlmostEqual(Desk["tape_error_cm"], Recorded["tape_error_cm"], delta=0.03)
        Self.assertTrue(CD._DeskDiffers(Desk))

    def TestWithNoRimKnownItSaysHowToRecordItNotRecalibrate(Self):
        with patch.object(RR, "CalibrationRim", return_value=None):
            Result, New, Record = Self.Check(*Self.Shelf, Seq=Fixture["shelf"]["seq"])
        Self.assertEqual((Result["ok"], Result["error_code"], Result["robot_check"], Result["robot_mode"]),
                         (False, "camera_check_failed", "unverified", RR.DeskDiffers), Result)
        Self.assertEqual((Result["tape_check"], Result["rim_check"], Result["rim_reason"]), ("ok", "unavailable", RR.NoRimReason))
        Self.assertIsNone(New)
        Self.assertIsNone(Record)
        Self.assertIn("the gripper tape agrees (0.59 cm)", Result["error"])
        Self.assertIn("the basket's front rim has not been recorded for this calibration yet", Result["error"])
        Self.assertIn("Send one task with the cart back at the calibrated desk", Result["error"])
        Self.assertTrue(Result["error"].endswith("No recalibration is needed."), Result["error"])
        Self.assertNotIn("Automatic calibration", Result["error"])
        Self.assertEqual(Result["desk_check"]["error_code"], "camera_check_failed", "the desk path's own result rides along")

    def TestTheDeskViewsRimJudgesTheShelfCapture(Self):
        # The fixture alone, the calibration's frames left out: the 22:08 capture recorded 20 cm of the rim (a cup on its left,
        # a gap in its depth), the shelf capture shows 22, 19 of them the same: too little to judge, and it says to record more.
        with patch.object(RR, "CalibrationRim", return_value=None):
            Self.RecordTheDeskView()
            Short, _, _ = Self.Check(*Self.Shelf, Seq=Fixture["shelf"]["seq"])
        Self.assertEqual((Short["ok"], Short["error_code"], Short["robot_check"], Short["rim_check"], Short["rim_reference_short"]),
                         (False, "camera_check_failed", "unverified", "not_judgeable", True), Short)
        Self.assertIn("too little of the basket's front rim has been recorded yet", Short["error"])
        Self.assertIn("that capture records the rim. No recalibration is needed.", Short["error"])
        # With the calibration's own frames as well (as live): verified by tape and rim, the 22:08 rim merged in.
        RC.Need(CalibrationRun, What="the 2026-10-14 09:33 calibration run (its frames of the basket rim)")
        Result, New, Record = Self.Check(*Self.Shelf, Seq=Fixture["shelf"]["seq"])
        Self.assertEqual((Result["ok"], Result.get("error_code"), Result["alignment_source"], Result["robot_mode"], Result["robot_check"]),
                         (True, None, "robot", RR.DeskDiffers, "verified"), Result)
        Self.assertEqual(Result["robot_witnesses"], ["tape", "rim"])
        Self.assertAlmostEqual(Result["tape_error_cm"], 0.58, delta=0.05)
        Self.assertEqual((Result["rim_check"], Result["rim_baseline"]), ("ok", {"calibration": True, "recorded": 1}))
        Self.assertLess(abs(Result["rim_yaw_deg"]), 0.5)
        Self.assertLess(max(map(abs, Result["rim_shift_cm"])), 0.3)
        Self.assertIsNone(New, "a desk that is not the calibrated desk is not re-measured")
        Self.assertIsNone(Record, "a robot-verified capture records no rim")
        Self.assertEqual(Result["desk_check"]["error_code"], "camera_check_failed")
        Self.assertEqual(Result["desk_check"]["desk_change"]["plane_angle_deg"], 4.17)
        Self.assertIn("the desk in view does not match the calibrated desk (it moved 4.17 deg / +3.77 cm", Result["reason"])
        Self.assertEqual(CD.Verdict(Result), "verified")
        print(f"  22:15:25 on the calibration's frames and the 22:08 rim: tape {Result['tape_error_cm']} cm, "
              f"rim {Result['rim_yaw_deg']} deg / {Result['rim_shift_cm']} cm")

    def TestTheCalibrationsOwnFramesAreEnoughToo(Self):
        RC.Need(CalibrationRun, What="the 2026-10-14 09:33 calibration run (its frames of the basket rim)")
        Rim = RR.CalibrationRim(Model)
        Self.assertIsNotNone(Rim)
        Self.assertTrue(Rim["every_pose"], "no early pose shows 24 bins of it clear; every pose of the run together does")
        Self.assertGreaterEqual(int(np.isfinite(Rim["profile"]).all(axis=1).sum()), RR.RimMinBins)
        Result, _, _ = Self.Check(*Self.Shelf, Seq=Fixture["shelf"]["seq"])
        Self.assertEqual((Result["ok"], Result["robot_check"], Result["rim_check"], Result["rim_baseline"]),
                         (True, "verified", "ok", {"calibration": True, "recorded": 0}), Result)
        Self.assertLess(abs(Result["rim_yaw_deg"]), 0.6)
        # The desk view agrees with them (that is how it may record): within the closer limits.
        Desk, _, Record = Self.Check(*Self.Desk)
        Self.assertEqual(Desk[RR.ShadowKey]["rim_check"], "ok")
        Self.assertLessEqual(abs(Desk[RR.ShadowKey]["rim_yaw_deg"]), RR.RimDiffersMaxYawDeg)
        Self.assertIsNotNone(Record)
        print(f"  22:15:25 on the calibration's frames: rim {Result['rim_yaw_deg']} deg / {Result['rim_shift_cm']} cm; "
              f"22:08 against them {Desk[RR.ShadowKey]['rim_yaw_deg']} deg")

    def Locate(Self, Capture, Arm, Boxes, Task):
        Depth, Image = cv2.imread(str(Folder / Capture["depth"]), cv2.IMREAD_UNCHANGED), cv2.imread(str(Folder / Capture["image"]))
        Detect = MagicMock(return_value=Boxes)
        with patch.object(calibration, "camera_model", return_value=Model), \
                patch.object(calibration, "pixel_to_plane", side_effect=lambda U, V, Mirrored=None: SC.PixelToPlane(Model, U, V, Mirrored)), \
                patch.object(calibration, "status", return_value={"calibrated": True, "rms_cm": 0.55}), \
                patch.object(calibration, "save_remeasured_desk", side_effect=AssertionError("no desk is re-measured here")), \
                patch.object(object_locator, "_pick_model", MagicMock(return_value="test-model")), \
                patch.object(object_locator, "_detect_once", Detect), patch.object(object_locator, "_refine_detection", return_value=None):
            return object_locator.locate_objects(TDC.Encode(Image), depth={"depth_mm": Depth, "intrinsics": dict(Camera), "mirrored": False,
                                                                           "seq": Capture["seq"]},
                                                 mirrored=False, arm=Arm, task_object=Task, locate_mode="vlm"), Detect

    def TestEndToEndThroughLocate(Self):
        RC.Need(CalibrationRun, What="the 2026-10-14 09:33 calibration run (its frames of the basket rim)")
        # 22:08, the desk view: /locate records its rim (no object asked for).
        Desk, _ = Self.Locate(Fixture["desk_view"], Self.Desk[2], [], None)
        Self.assertTrue(Desk["ok"], Desk)
        Self.assertTrue(Desk["camera_check"]["rim_recorded"], Desk["camera_check"])
        Self.assertNotIn(RR.RimRecordKey, Desk["camera_check"])
        Self.assertEqual(len(RR.RecordedRims(Model)), 1)
        # 22:15, the shelf: the camera checked against the robot, then the pill bottle measured on the cabinet top.
        U0, V0, U1, V1 = Fixture["shelf"]["bottle_box_native_px"]
        Box = [U0 / Width, V0 / Height, U1 / Width, V1 / Height]
        Result, Detect = Self.Locate(Fixture["shelf"], Self.Shelf[2], [TDC.Detected("pill bottle", Box)], Fixture["shelf"]["task_object"])
        Self.assertTrue(Result["ok"], {Key: Result.get(Key) for Key in ("error_code", "error", "camera_check")})
        Detect.assert_called()
        Check = Result["camera_check"]
        Self.assertEqual((Check["alignment_source"], Check["robot_mode"], Check["robot_check"], Check["robot_witnesses"]),
                         ("robot", RR.DeskDiffers, "verified", ["tape", "rim"]))
        Self.assertEqual(Check["rim_baseline"], {"calibration": True, "recorded": 1})
        Self.assertNotIn("rim_recorded", Check)
        Self.assertEqual(Result["no_desk"], {"robot_check": "verified", "supports": "measured"})
        Artifacts.mkdir(parents=True, exist_ok=True)
        Record = {"kind": "locate", "task_object": Fixture["shelf"]["task_object"], **Result}
        (Artifacts / "Locate.json").write_text(json.dumps([Record], default=lambda Value: Value.tolist() if hasattr(Value, "tolist") else str(Value)),
                                               encoding="utf-8")
        # The cabinet top around the bottle is a level patch at Z 3.3, but the books stand against the bottle's back, so the
        # patch does not surround its footprint there (it reaches Y 42.38, the footprint 42.45): it was withheld as "not
        # measurable". With no desk in view the patch beside its front and sides holds it (DepthGeometry.SupportSides).
        AssertShelfBottle(Self, Result, (-18.0, 41.05), ("pill bottle", Fixture["shelf"]["task_object"]["label"]))
        print(f"  22:15:25 {Describe(Result)}")


def AssertShelfBottle(Self, Result, Where, Labels):
    """The task's pill bottle (detected as Labels[0], asked for under any of Labels) measured on the cabinet top at Where
    (display X, Y), its support and scene with it."""
    Self.assertNotIn("scene_error", Result)
    [Bottle] = [Item for Item in Result["objects"] if Item.get("task_match")]
    Self.assertEqual(Bottle["label"], Labels[0])
    Self.assertLess(math.hypot(Bottle["x"] - Where[0], Bottle["y"] - Where[1]), 0.5, Bottle)
    Self.assertAlmostEqual(Bottle["z"], 3.3, delta=0.2)
    Self.assertAlmostEqual(Bottle["height_cm"], 8.0, delta=0.3)
    Self.assertLess(Bottle["size_cm"][0], 5.0, Bottle)
    Self.assertEqual((Bottle["support_branch"], Bottle["support_surface"]), ("surface", "measured support at Z 3.3"))
    Support = Bottle["support_geometry"]
    Self.assertEqual((Support["Source"], Support["ObservedOnly"]), ("depth", True))
    Self.assertGreaterEqual(len(Support["Evidence"]["sides"]), DG.SupportMinSides)
    Self.assertNotIn("+y", Support["Evidence"]["sides"], "the books stand behind it")
    Self.assertLess(Support["Evidence"]["height_spread_cm"], 0.7)
    Self.assertEqual(Result["scene"]["task_object"]["label"], Bottle["label"])
    for Label in Labels:
        Self.assertNotIn(Label, [Item["label"] for Item in Result["withheld"]])
    return Bottle


def Describe(Result):
    [Bottle] = [Item for Item in Result["objects"] if Item.get("task_match")]
    Support, Contact = Bottle["support_geometry"], Result["scene"].get("task_contact")
    return (f"{Bottle['label']}: X {Bottle['x']} Y {Bottle['y']} base Z {Bottle['z']} top {Bottle['top_z']} "
            f"(height {Bottle['height_cm']}, footprint {Bottle['size_cm']} cm) on '{Bottle['support_surface']}' "
            f"(X {Support['XRangeCm']}, Y {Support['YRangeCm']}, sides {Support['Evidence']['sides']}); "
            + "against it: " + ("none" if not Contact else f"{Contact['points']} points"))


class ShelfRefusal0210Tests(RimFolder):
    """The same request's refusal at 02:10:18Z (shelf_0210): the bottle elsewhere on the cabinet top, books against its back
    and left, through the single call as live (its recorded reply), the arm at Home."""

    def TestTheBottleIsMeasuredNow(Self):
        RC.Need(CalibrationRun, What="the 2026-10-14 09:33 calibration run (its frames of the basket rim)")
        from TestSingleCallLocate import FakeOllama
        Capture = Fixture["shelf_0210"]
        Self.assertEqual(Capture["recorded_withheld"][0], {"label": "pill bottle", "reason": f"not measurable: {DG.NoDeskSupportError}"})
        Depth, Image = cv2.imread(str(Folder / Capture["depth"]), cv2.IMREAD_UNCHANGED), cv2.imread(str(Folder / Capture["image"]))
        Fake = FakeOllama(Capture["reply"])
        with patch.object(calibration, "camera_model", return_value=Model), \
                patch.object(calibration, "pixel_to_plane", side_effect=lambda U, V, Mirrored=None: SC.PixelToPlane(Model, U, V, Mirrored)), \
                patch.object(calibration, "status", return_value={"calibrated": True, "rms_cm": 0.68}), \
                patch.object(calibration, "save_remeasured_desk", side_effect=AssertionError("no desk is re-measured here")), \
                patch.object(object_locator, "_pick_model", MagicMock(return_value="test-model")), \
                patch.object(object_locator, "_http_json", side_effect=Fake):
            Result = object_locator.locate_objects(TDC.Encode(Image), depth={"depth_mm": Depth, "intrinsics": dict(Camera), "mirrored": False,
                                                                             "seq": Capture["seq"]},
                                                   mirrored=False, arm=ArmFromRaw(Capture["raws"], RR.ReferenceRaw(Model)),
                                                   task_object=Capture["task_object"], locate_mode="single_call")
        Self.assertTrue(Result["ok"], {Key: Result.get(Key) for Key in ("error_code", "error", "camera_check")})
        Self.assertEqual(Result["model_calls"], 1)
        Check = Result["camera_check"]
        Self.assertEqual((Check["alignment_source"], Check["robot_mode"], Check["robot_check"]), ("robot", RR.DeskAbsent, "verified"))
        Self.assertEqual(Result["no_desk"], {"robot_check": "verified", "supports": "measured"})
        Bottle = AssertShelfBottle(Self, Result, (-25.8, 41.66), ("pill bottle",))
        # Its outline takes in the foot of the book leaning against its back left (Y to 54 cm, rising 0.6-3.5 cm off the top):
        # its footprint is long in Y (9.3 cm), its width across X the bottle's (3.6 cm).
        Self.assertGreater(Bottle["size_cm"][1], 8.0)
        # The other objects as recorded (the cardboard box has no depth, as then).
        Recorded = {Item["label"]: Item for Item in Capture["recorded_objects"]}
        for Item in Result["objects"]:
            if Item["label"] in Recorded:
                Self.assertEqual({Key: Item.get(Key) for Key in Recorded[Item["label"]]}, Recorded[Item["label"]])
        Self.assertEqual(sorted(Recorded), sorted(Item["label"] for Item in Result["objects"] if not Item.get("task_match")))
        Self.assertEqual([Item["label"] for Item in Result["withheld"]], ["cardboard box"])
        Artifacts.mkdir(parents=True, exist_ok=True)
        (Artifacts / "Locate0210.json").write_text(json.dumps({"kind": "locate", "task_object": Capture["task_object"], **Result},
                                                              default=lambda Value: Value.tolist() if hasattr(Value, "tolist") else str(Value)),
                                                   encoding="utf-8")
        print(f"  02:10:18 {Describe(Result)}")


class RealMoveTests(RimFolder):
    """Camera moves on the robot still block where the robot decides: the desk in view but disagreeing, and no desk."""

    def AssertEveryMoveBlocks(Self, Depth, Image, Arm, Model_, Mode, Seq, Pans=(1.5, -1.5, 2.0, -2.0)):
        Still, _, _ = Self.Check(*MovedFrame(Depth, Image, Camera, np.eye(3)), Arm, Seq, Model_)
        Self.assertEqual((Still["ok"], Still.get("robot_mode"), Still.get("robot_check")), (True, Mode, "verified"),
                         f"the frame re-rendered unmoved passes: {Still}")
        Verdicts = {}
        for Name, Rotation, Shift in Moves(Model_, Pans):
            with Self.subTest(Name):
                Result, New, Record = Self.Check(*MovedFrame(Depth, Image, Camera, Rotation, Shift), Arm, Seq, Model_)
                Verdicts[Name] = (Result.get("error_code"), Result.get("tape_error_cm"), Result.get("rim_yaw_deg"), Result.get("rim_shift_cm"))
                Self.assertFalse(Result["ok"], Result)
                Self.assertIn(Result["error_code"], ("camera_moved", "camera_check_failed"), Result)
                Self.assertIsNone(New)
                Self.assertIsNone(Record)
        Moved = sum(Code == "camera_moved" for Code, *_ in Verdicts.values())
        print(f"  {Mode}: {len(Verdicts)} moves, {Moved} camera_moved, the rest not checked")
        return Verdicts

    def TestTheShelfFrameMovedOnTheRobotBlocks(Self):
        RC.Need(CalibrationRun, What="the 2026-10-14 09:33 calibration run (its frames of the basket rim)")
        Depth, Image, Arm = Load(Fixture["shelf"])
        Verdicts = Self.AssertEveryMoveBlocks(Depth, Image, Arm, Model, RR.DeskDiffers, Fixture["shelf"]["seq"])
        # The real head turn of 2026-10-06 15:02 (4.16 deg) moves the tape past its limit either way.
        for Name in ("head turn +4.16 deg about the camera's y", "head turn -4.16 deg about the camera's y"):
            Self.assertEqual(Verdicts[Name][0], "camera_moved")

    def TestTheCabinetFrameWithNoDeskMovedOnTheRobotBlocks(Self):
        RC.Need(Cabinet1501["depth"], Cabinet1501["image"], What="the 2026-10-06 15:01 cabinet capture")
        Model1002 = RC.CalibrationOf20261002()
        RC.Need(Path(Model1002["drift_reference"]["keyframe"]).parent, What="the 2026-10-02 21:09 calibration run")
        Depth, Image = cv2.imread(str(Cabinet1501["depth"]), cv2.IMREAD_UNCHANGED), cv2.imread(str(Cabinet1501["image"]))
        # Pans of 2 deg here: this frame already reads 0.4 deg and 0.4-0.6 cm off the 2026-10-02 calibration on the rim (the
        # camera kept about 0.7 deg of the 2026-10-03 pan, .claude/NoDeskDesign.md 11.4), so a 1.5 deg pan the other way is
        # about 1.1 deg from where it was calibrated: tape 0.48 cm, rim 1.00 deg, both within their limits.
        Self.AssertEveryMoveBlocks(Depth, Image, ArmFromRaw(Cabinet1501["raws"], RR.ReferenceRaw(Model1002)), Model1002, RR.DeskAbsent, 516698,
                                   Pans=(2.0, -2.0))


class RimRecordTests(RimFolder):
    """What may record the rim, and for which calibration."""

    @classmethod
    def setUpClass(Class):
        Class.Desk, Class.Shelf = Load(Fixture["desk_view"]), Load(Fixture["shelf"])

    def TestOnlyAFullyVerifiedDeskPassRecords(Self):
        Depth, Image, Arm = Self.Desk
        with patch.object(RR, "CalibrationRim", return_value=None):
            _, _, Record = Self.Check(Depth, Image, Arm)
            Self.assertIsNotNone(Record)
            # The arm not still (no pose to trust): the tape cannot verify it, nothing is recorded.
            Result, _, Record = Self.Check(Depth, Image, {"ok": False, "reason": "arm moving"})
            Self.assertEqual((Result["ok"], Record), (True, None))
            # The jaw open: the tape does not judge, nothing is recorded.
            Open = dict(Arm, servos_raw=dict(Arm["servos_raw"], gripper=1900))
            Result, _, Record = Self.Check(Depth, Image, Open)
            Self.assertEqual((Result["ok"], Result[RR.ShadowKey]["tape_check"], Record), (True, "pose_differs", None))
            # The camera panned 1.5 deg at the desk (which cannot see a pan): the desk still passes, the tape is off.
            Rotation, _, _ = DG.PoseFromModel(Model)
            Panned = MovedFrame(Depth, Image, Camera, cv2.Rodrigues(Rotation[:, 2] / np.linalg.norm(Rotation[:, 2]) * math.radians(1.5))[0])
            Result, _, Record = Self.Check(*Panned, Arm)
            Self.assertTrue(Result["ok"], Result)
            Self.assertIsNone(Record, Result.get(RR.ShadowKey))
            Shadow = Result[RR.ShadowKey]
            Self.assertTrue(Shadow.get("tape_check") != "ok" or Shadow["tape_error_cm"] > RR.RimRecordTapeMaxCm, Shadow)
            # Nor does the shelf capture, which the robot verified, or a blocked one.
            Shelf, _, Record = Self.Check(*Self.Shelf)
            Self.assertEqual((Shelf["ok"], Record), (False, None))
        Self.assertEqual(RR.RecordedRims(Model), [])

    def TestTheRuleItself(Self):
        Profile = np.tile([21.9, -0.8], (len(RR._RimBins()), 1))
        Pass = {"ok": True, "reason": "desk matches the calibration", "angle_deg": 0.1, "offset_cm": 0.0}
        Shadow = {"tape_check": "ok", "tape_error_cm": 0.3, "rim_check": "ok", "rim_yaw_deg": 0.2, "rim_shift_cm": [0.1, -0.1]}
        Self.assertIsNotNone(RR.RimRecordCandidate(Pass, Shadow, Profile))
        Self.assertIsNotNone(RR.RimRecordCandidate(Pass, {**Shadow, "rim_check": "unavailable", "rim_reason": RR.NoRimReason}, Profile))
        for Name, Result, Shadow_, Profile_ in (
                ("desk blocked", {**Pass, "ok": False}, Shadow, Profile),
                ("robot-verified", {**Pass, "alignment_source": "robot"}, Shadow, Profile),
                ("desk occluded", {**Pass, "desk_occluded": True}, Shadow, Profile),
                ("desk re-measured", {**Pass, "desk_remeasured": True}, Shadow, Profile),
                ("tape past the record limit", Pass, {**Shadow, "tape_error_cm": RR.RimRecordTapeMaxCm + 0.01}, Profile),
                ("tape not judged", Pass, {**Shadow, "tape_check": "pose_differs", "tape_error_cm": None}, Profile),
                ("rim moved against what is known", Pass, {**Shadow, "rim_check": "moved", "rim_yaw_deg": 1.6}, Profile),
                ("rim past the closer limits", Pass, {**Shadow, "rim_yaw_deg": RR.RimDiffersMaxYawDeg + 0.05}, Profile),
                ("rim shifted past them", Pass, {**Shadow, "rim_shift_cm": [RR.RimDiffersMaxShiftCm + 0.05, 0.0]}, Profile),
                ("rim not judgeable against what is known", Pass, {**Shadow, "rim_check": "not_judgeable"}, Profile),
                ("rim unavailable for another reason", Pass, {**Shadow, "rim_check": "unavailable", "rim_reason": "no depth frame"}, Profile),
                ("too little rim", Pass, Shadow, np.where(np.arange(len(Profile))[:, None] < RR.RimMinBins - 1, Profile, np.nan)),
                ("no profile", Pass, Shadow, None)):
            with Self.subTest(Name):
                Self.assertIsNone(RR.RimRecordCandidate(Result, Shadow_, Profile_))

    def TestANewCalibrationStartsFreshAndADeskRemeasureDoesNot(Self):
        Record = RR.RimRecordCandidate({"ok": True}, {"tape_check": "ok", "tape_error_cm": 0.2, "rim_check": "unavailable", "rim_reason": RR.NoRimReason},
                                       np.tile([21.9, -0.8], (len(RR._RimBins()), 1)))
        Self.assertEqual(RR.RecordRim(Record, Model), (True, ""))
        Self.assertEqual(RR.RecordRim(Record, Model), (True, ""))
        Self.assertEqual(len(RR.RecordedRims(Model)), 2)
        # The desk re-measured after a cart move: the same calibration, the same rim.
        Remeasured = dict(Model["drift_reference"], desk_remeasured_at="2026-10-25T15:20:00", desk_d=Model["drift_reference"]["desk_d"] - 1.0)
        Remeasured["sha256"] = CD.Checksum(Remeasured, Model)
        Moved = dict(Model, drift_reference=Remeasured)
        Self.assertEqual(CD.VerifiedReference(Moved)[1], "")
        Self.assertEqual(RR.RimIdentity(Moved), RR.RimIdentity(Model))
        Self.assertEqual(len(RR.RecordedRims(Moved)), 2)
        # A new calibration (another fit, another camera pose): nothing recorded for it, and its first record replaces the old.
        Reference = {Key: Value for Key, Value in Model["drift_reference"].items() if Key != "sha256"}
        Refit = dict(Model, fitted_at="2026-10-26T10:00:00", tvec=[Model["tvec"][0] + 0.4, *Model["tvec"][1:]])
        Reference["fitted_at"] = Refit["fitted_at"]
        Reference["sha256"] = CD.Checksum(Reference, Refit)
        Refit["drift_reference"] = Reference
        Self.assertEqual(CD.VerifiedReference(Refit)[1], "")
        Self.assertNotEqual(RR.RimIdentity(Refit), RR.RimIdentity(Model))
        Self.assertEqual(RR.RecordedRims(Refit), [])
        Self.assertEqual(RR.RecordRim(Record, Refit), (True, ""))
        Self.assertEqual(len(RR.RecordedRims(Refit)), 1)
        Self.assertEqual(RR.RecordedRims(Model), [], "the old calibration's records are gone with it")
        Stored = json.loads(Path(os.environ["SO101_RIM_BASELINE"]).read_text(encoding="utf-8"))
        Self.assertEqual((Stored["identity"], Stored["fitted_at"]), (RR.RimIdentity(Refit), "2026-10-26T10:00:00"))
        # Only the latest RimRecordKeep are kept; an uncalibrated camera records nothing.
        for _ in range(RR.RimRecordKeep + 2):
            RR.RecordRim(Record, Refit)
        Self.assertEqual(len(RR.RecordedRims(Refit)), RR.RimRecordKeep)
        Self.assertEqual(RR.RecordRim(Record, None)[0], False)
        Self.assertEqual(list(Self.RimFolder.glob("*.tmp")), [], "written whole")


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestShelfCapture` runs the Test* methods too (unittest names this hook)."""
    Prefixed = unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return unittest.TestSuite(Prefixed.loadTestsFromTestCase(Case) for Case in (ShelfReplayTests, ShelfRefusal0210Tests, RealMoveTests, RimRecordTests))


if __name__ == "__main__":
    Loader = unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Suite = unittest.TestSuite(Loader.loadTestsFromTestCase(Case) for Case in (ShelfReplayTests, ShelfRefusal0210Tests, RealMoveTests, RimRecordTests))
    Result = unittest.TextTestRunner(verbosity=2).run(Suite)
    raise SystemExit(not Result.wasSuccessful())
