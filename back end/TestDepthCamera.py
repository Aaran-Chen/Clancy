"""Depth camera service, depth geometry, depth-aware calibration and the bridge endpoints.

Scenes are ray-cast: a table plane and boxes seen by an Orbbec-like camera with
its real factory intrinsics. No camera, arm or Ollama is needed.

Run:  python TestDepthCamera.py
"""
import base64
import hashlib
import json
import math
import os
os.environ.setdefault("SO101_LOCATE_MODE", "vlm")  # this suite tests the voted multi-call locate; TestSingleCallLocate.py the default
import sys
import time
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

import cv2
import numpy as np

# /locate is recorded (SessionLog); keep test runs out of the real record.
os.environ["SO101_CAPTURES_ROOT"] = str(Path(__file__).resolve().parent / "TestArtifacts" / "SessionCaptures")
# The surfaces declared for the real arm (DeclaredSurfaces.json) are no part of these scenes.
os.environ["SO101_DECLARED_SURFACES"] = str(Path(__file__).resolve().parent / "TestArtifacts" / "NoDeclaredSurfaces.json")

import arm_geometry as Geo  # noqa: E402
import bridge_server as Bridge
import calibration
import CameraDrift as CD
import DepthGeometry as DG
import object_locator
import OrbbecCamera as OC
import LoadScale
import RecordedCaptures as RC
import SelfCalibration as SC

Root = Path(__file__).resolve().parent
Artifacts = Root / "TestArtifacts"
Artifacts.mkdir(exist_ok=True)
Width, Height = 1280, 720
TableZ = SC.DefaultTableZCm
# The Gemini 335L's factory color intrinsics at 1280x720 (read from the device).
Intrinsics = {"fx": 609.747864, "fy": 609.727783, "cx": 640.365051, "cy": 362.179260, "width": Width, "height": Height,
              "dist": [-0.029167, 0.033867, -0.000119, -0.000061, -0.012104], "source": "test"}
DisplayOffset = np.array(list(Bridge.controller.WorkspaceOffset().values()))


def LookAt(Eye, Target):
    """World-to-camera rotation and translation (OpenCV: +z forward, +y down)."""
    Forward = np.asarray(Target, dtype=float) - Eye
    Forward /= np.linalg.norm(Forward)
    Right = np.cross(Forward, [0.0, 0.0, 1.0])
    Right /= np.linalg.norm(Right)
    Rotation = np.vstack([Right, np.cross(Forward, Right), Forward])
    return Rotation, -Rotation @ Eye


def CameraModel(EyeDisplay, TargetDisplay, Mirror=True):
    """A depth-camera model looking from a display-frame eye at a display-frame target."""
    Scale = DG.MirrorVector if Mirror else np.ones(3)
    Rotation, Translation = LookAt(np.asarray(EyeDisplay, float) * Scale, np.asarray(TargetDisplay, float) * Scale)
    Rvec, _ = cv2.Rodrigues(Rotation)
    return {"type": "pinhole_depth_v1", "rvec": Rvec.ravel().tolist(), "tvec": Translation.tolist(), "mirror_x": Mirror,
            "image_size": [Width, Height], "table_z": TableZ, "input_mirrored": True,
            "f": 0.5 * (Intrinsics["fx"] + Intrinsics["fy"]), "fx": Intrinsics["fx"], "fy": Intrinsics["fy"],
            "cx": Intrinsics["cx"], "cy": Intrinsics["cy"], "dist": list(Intrinsics["dist"])}


# A camera on the far side of the workspace facing the arm (mirrored image, as
# the app assumes) and one on the robot's head looking forward and down.
FrontCamera = CameraModel([-15.0, 72.0, 48.0], [-15.0, 24.0, TableZ])
HeadCamera = CameraModel([0.0, -4.0, 45.0], [-15.0, 30.0, TableZ])


def RenderDepth(Model, Boxes, FloorZ=TableZ):
    """Z-depth (mm) of a floor plane and axis-aligned display-frame boxes."""
    Rays = DG.RayMap(Intrinsics).astype(np.float64).reshape(-1, 2)
    Directions = np.column_stack([Rays, np.ones(len(Rays))])  # z = 1: the ray parameter is the depth
    Rotation, _, Mirror = DG.PoseFromModel(Model)
    World = Directions @ Rotation
    if Mirror:
        World = World * DG.MirrorVector
    Centre = DG.CameraToDisplay(np.zeros(3), Model)[0]
    with np.errstate(divide="ignore", invalid="ignore"):
        Hit = (FloorZ - Centre[2]) / World[:, 2]
        Hit[~(Hit > 0)] = np.inf
        for Low, High in Boxes:
            First = (np.asarray(Low) - Centre) / World
            Second = (np.asarray(High) - Centre) / World
            Near = np.nanmax(np.minimum(First, Second), axis=1)
            Far = np.nanmin(np.maximum(First, Second), axis=1)
            Inside = (Far >= Near) & (Near > 0) & (Near < Hit)
            Hit = np.where(Inside, Near, Hit)
    return np.where(np.isfinite(Hit), np.round(Hit * 10.0), 0).astype(np.uint16).reshape(Height, Width)


def CaptureBox(Model, Low, High, Pad=0.15):
    """Normalized mirrored-capture box around a display-frame box, like a VLM would draw."""
    Corners = np.array([[X, Y, Z] for X in (Low[0], High[0]) for Y in (Low[1], High[1]) for Z in (Low[2], High[2])])
    Pixels = DG.Project(DG.DisplayToCamera(Corners, Model), Intrinsics) + 0.5  # continuous native
    U0, V0 = Pixels.min(axis=0)
    U1, V1 = Pixels.max(axis=0)
    PadU, PadV = (U1 - U0) * Pad, (V1 - V0) * Pad
    U0, U1, V0, V1 = U0 - PadU, U1 + PadU, V0 - PadV, V1 + PadV
    return [(Width - U1) / Width, V0 / Height, (Width - U0) / Width, V1 / Height]


def Encode(Image):
    _, Data = cv2.imencode(".jpg", Image, [cv2.IMWRITE_JPEG_QUALITY, 95])
    return "data:image/jpeg;base64," + base64.b64encode(Data).decode("ascii")


def DepthFile(Depth):
    """A depth frame saved as the fit archives it (16-bit PNG), for CameraDrift.PlaneDepth."""
    Encoded = cv2.imencode(".png", Depth.astype(np.uint16))[1].tobytes()
    Target = Artifacts / "DriftDepth" / f"{hashlib.sha1(Encoded).hexdigest()}.png"
    if not Target.exists():
        Target.parent.mkdir(exist_ok=True)
        Target.write_bytes(Encoded)
    return str(Target)


def Verified(Model, Boxes=(), FloorZ=TableZ, TapePose=None, Truth=None, TapeAreas=None):
    """The model with the drift baseline a fit saves: the desk the true camera (Model itself
    unless given) sees, measured in depth, and the tape's area at the fit's poses."""
    Model = dict(Model, fitted_at="2026-09-28T15:04:00")
    Depth = RenderDepth(Truth or Model, list(Boxes), FloorZ=FloorZ)
    Plane = dict(CD.DeskPlane(Depth, Intrinsics), file=DepthFile(Depth))
    Reference, Problem = CD.BuildReference([dict(Plane, pose_index=Index) for Index in range(CD.MinReferenceFrames)],
                                           Model, TapePose, Intrinsics=Intrinsics, TapeAreasCm2=TapeAreas)
    assert Reference is not None, Problem
    return dict(Model, drift_reference=Reference)


def Turned(Model, Degrees, Axis="pitch"):
    """The same camera turned about its own centre: "pitch" about the camera's x axis
    (positive tips the view down) or "pan" about the display vertical, the desk's normal."""
    Rotation, Translation, _ = DG.PoseFromModel(Model)
    Centre = -Rotation.T @ Translation
    Turn, _ = cv2.Rodrigues(np.array([math.radians(Degrees), 0.0, 0.0] if Axis == "pitch" else [0.0, 0.0, math.radians(Degrees)]))
    New = Turn @ Rotation if Axis == "pitch" else Rotation @ Turn
    return dict(Model, rvec=cv2.Rodrigues(New)[0].ravel().tolist(), tvec=(-New @ Centre).tolist())


def Shifted(Model, Display):
    """The same camera moved by a display-frame offset (cm) without turning."""
    Rotation, Translation, Mirror = DG.PoseFromModel(Model)
    Step = np.asarray(Display, dtype=float) * (DG.MirrorVector if Mirror else 1.0)
    return dict(Model, tvec=(Translation - Rotation @ Step).tolist())


class GeometryTests(unittest.TestCase):
    def TestProjectAndDeprojectRoundTrip(Self):
        Rng = np.random.default_rng(1)
        Points = np.column_stack([Rng.uniform(-30, 30, 50), Rng.uniform(-15, 15, 50), Rng.uniform(30, 120, 50)])
        Pixels = DG.Project(Points, Intrinsics)
        Back = DG.Deproject(Pixels[:, 0], Pixels[:, 1], Points[:, 2], Intrinsics)
        Self.assertLess(float(np.abs(Back - Points).max()), 1e-3)
        Rays = DG.RayMap(Intrinsics)
        U, V = 1000, 100
        Self.assertTrue(np.allclose(Rays[V, U], DG.Deproject(U, V, 1.0, Intrinsics)[0, :2], atol=1e-5))

    def TestMirroredCaptureConversions(Self):
        U, V = DG.NativeFromCapture(Width - (300 + 0.5), 200 + 0.5, Width, True)
        Self.assertAlmostEqual(float(U), 300.0)
        Self.assertAlmostEqual(float(V), 200.0)
        # A box on the capture's left edge is on the native image's right edge.
        Self.assertEqual(DG.BoxToNative([0.0, 0.0, 0.25, 0.5], Width, Height, True), (960, 0, 1280, 360))
        Self.assertEqual(DG.BoxToNative([0.0, 0.0, 0.25, 0.5], Width, Height, False), (0, 0, 320, 360))

    def TestDisplayCameraRoundTripKeepsHandedness(Self):
        Rng = np.random.default_rng(2)
        Points = Rng.uniform(-40, 40, (20, 3))
        for Model in (FrontCamera, HeadCamera, CameraModel([5, 60, 40], [0, 20, 5], Mirror=False)):
            Self.assertLess(float(np.abs(DG.CameraToDisplay(DG.DisplayToCamera(Points, Model), Model) - Points).max()), 1e-9)
        # The table is below the front camera: +Z up survives the mirror.
        Self.assertGreater(DG.CameraToDisplay(np.zeros(3), FrontCamera)[0, 2], TableZ + 30)

    def TestForegroundDepthIgnoresBackground(Self):
        Values = np.concatenate([np.full(30, 60.0) + np.linspace(-0.3, 0.3, 30), np.full(40, 120.0), np.zeros(10)])
        Self.assertAlmostEqual(DG.ForegroundDepthCm(Values), 60.0, delta=0.2)
        Self.assertIsNone(DG.ForegroundDepthCm(np.zeros(20)))

    def TestMeasureBoxFindsCentreTopAndSize(Self):
        Low, High = np.array([-18.0, 25.0, TableZ]), np.array([-12.0, 31.0, TableZ + 8.0])
        # A taller box standing behind and to the side, partly inside the drawn box.
        Behind = (np.array([-11.0, 33.0, TableZ]), np.array([-5.0, 38.0, TableZ + 15.0]))
        for Name, Model in (("front", FrontCamera), ("head", HeadCamera)):
            with Self.subTest(camera=Name):
                Depth = RenderDepth(Model, [(Low, High), Behind])
                Result = DG.MeasureBox(Depth, CaptureBox(Model, Low, High), Intrinsics, Model, TableZ, True)
                Self.assertTrue(Result["ok"], Result)
                Self.assertLess(math.hypot(Result["x"] + 15.0, Result["y"] - 28.0), 0.6, Result)
                Self.assertAlmostEqual(Result["top_z"], TableZ + 8.0, delta=0.3)
                Self.assertAlmostEqual(Result["height_cm"], 8.0, delta=0.3)
                Self.assertAlmostEqual(Result["z"], TableZ, delta=0.01)
                for Size in Result["size_cm"]:
                    Self.assertTrue(4.5 < Size < 6.6, Result["size_cm"])

    def TestMeasureBoxOnlyTableFails(Self):
        Depth = RenderDepth(FrontCamera, [])
        Result = DG.MeasureBox(Depth, [0.4, 0.4, 0.6, 0.6], Intrinsics, FrontCamera, TableZ, True)
        Self.assertFalse(Result["ok"])
        Self.assertIn("above the table", Result["error"])
        Self.assertAlmostEqual(Result["support_z_measured"], TableZ, delta=0.05)

    def TestThinObjectOnSlightlyMiscalibratedTable(Self):
        # The real desk is 4 mm above the calibrated height; a 1 cm keyboard must
        # still stand out in full, measured from the desk around it.
        Desk = TableZ + 0.4
        Low, High = np.array([-25.0, 24.0, Desk]), np.array([-5.0, 34.0, Desk + 1.0])
        Depth = RenderDepth(FrontCamera, [(Low, High)], FloorZ=Desk)
        Result = DG.MeasureBox(Depth, CaptureBox(FrontCamera, Low, High), Intrinsics, FrontCamera, TableZ, True)
        Self.assertTrue(Result["ok"], Result)
        Self.assertTrue(Result["support_measured"])
        Self.assertAlmostEqual(Result["z"], Desk, delta=0.1)
        Self.assertAlmostEqual(Result["height_cm"], 1.0, delta=0.2)
        Self.assertGreater(Result["size_cm"][0], 16.0, Result)
        Self.assertGreater(Result["size_cm"][1], 8.0, Result)
        Self.assertLess(math.hypot(Result["x"] + 15.0, Result["y"] - 29.0), 0.8, Result)

    def TestBareTableOffCalibrationIsNotAnObject(Self):
        Depth = RenderDepth(FrontCamera, [], FloorZ=TableZ + 0.6)
        Result = DG.MeasureBox(Depth, [0.4, 0.4, 0.6, 0.6], Intrinsics, FrontCamera, TableZ, True)
        Self.assertFalse(Result["ok"], Result)

    def TestMeasureBoxInsideLowerBasket(Self):
        Floor = TableZ - 10.0
        Low, High = np.array([-17.0, 26.0, Floor]), np.array([-13.0, 30.0, Floor + 5.0])
        Depth = RenderDepth(FrontCamera, [(Low, High)], FloorZ=Floor)
        Result = DG.MeasureBox(Depth, CaptureBox(FrontCamera, Low, High), Intrinsics, FrontCamera, TableZ, True)
        Self.assertTrue(Result["ok"], Result)
        Self.assertAlmostEqual(Result["z"], Floor, delta=0.3)
        Self.assertAlmostEqual(Result["height_cm"], 5.0, delta=0.4)
        Self.assertLess(math.hypot(Result["x"] + 15.0, Result["y"] - 28.0), 0.6)

    def TestCheckPlaneAgreesWithCalibration(Self):
        Result = DG.CheckPlane(RenderDepth(HeadCamera, []), Intrinsics, HeadCamera)
        Self.assertTrue(Result["ok"])
        Self.assertLess(abs(Result["height_error_cm"]), 0.05)
        Self.assertLess(Result["plane_tilt_deg"], 0.2)
        Moved = CameraModel([0.0, -4.0, 47.0], [-15.0, 30.0, TableZ])  # camera 2 cm higher than calibrated
        Result = DG.CheckPlane(RenderDepth(Moved, []), Intrinsics, HeadCamera)
        Self.assertLess(Result["height_error_cm"], -1.0)

    def TestPointAtPixelMatchesScene(Self):
        Point = np.array([-15.0, 28.0, TableZ])
        Depth = RenderDepth(FrontCamera, [])
        U, V = DG.Project(DG.DisplayToCamera(Point, FrontCamera), Intrinsics)[0]
        Result = DG.PointAtPixel(Depth, Width - (U + 0.5), V + 0.5, Intrinsics, True)
        Self.assertTrue(Result["ok"])
        Self.assertLess(float(np.linalg.norm(DG.CameraToDisplay(np.array(Result["camera_cm"]), FrontCamera)[0] - Point)), 0.3)


def ArmInView(Model, Tape, Seed=4, TapeHeight=(-np.inf, np.inf)):
    """Display-frame joints of a reachable arm pose whose tip tape Model sees, as the bridge reports them,
    with the tape TapeHeight (cm, low-high) above the desk."""
    Controller = Bridge.controller
    Rng = np.random.default_rng(Seed)
    Limits = {Name: sorted((Controller._ik_angle_from_raw(Servo, Servo.range_min), Controller._ik_angle_from_raw(Servo, Servo.range_max)))
              for Name, Servo in Controller.servos.items()}
    Current = Controller._current_ik_angles()
    for _ in range(20000):
        Angles = {Name: float(Rng.uniform(*Limits[Name])) if Name in Bridge.IK_JOINTS else Current[Name] for Name in Bridge.MOTOR_NAMES}
        Joints = np.array([[P["x"], P["y"], P["z"]] for P in Geo.forward_joint_positions(Controller.arm_geometry, Angles)])
        Display = Joints * DG.MirrorVector + DisplayOffset
        Tip, Frame = SC.GripperFrame(Display)
        Point = Tip + Frame @ Tape
        Camera = DG.DisplayToCamera(Point, Model)[0]
        Pixel = DG.Project(Camera, Intrinsics)[0]
        if (Tip[2] > TableZ + 4.0 and TapeHeight[0] <= Point[2] - TableZ <= TapeHeight[1] and Camera[2] > 20
                and 150 < Pixel[0] < Width - 150 and 150 < Pixel[1] < Height - 150):
            return Display, Point
    raise AssertionError("no arm pose shows the tape")


def TapeView(Model, Point, Plate=60, Items=()):
    """Native image and depth of the desk with the red tip tape on a white gripper (a plate Plate px
    around it at the tape's depth) where Model sees Point; Items are red display-frame boxes on the desk."""
    Camera = DG.DisplayToCamera(Point, Model)[0]
    U, V = np.round(DG.Project(Camera, Intrinsics)[0]).astype(int)
    Image = np.full((Height, Width, 3), 90, np.uint8)
    Depth = RenderDepth(Model, list(Items))
    Image[Depth != RenderDepth(Model, [])] = (30, 30, 215)
    cv2.rectangle(Image, (U - Plate, V - Plate), (U + Plate, V + Plate), (235, 235, 235), -1)
    cv2.rectangle(Image, (U - 10, V - 10), (U + 10, V + 10), (30, 30, 215), -1)
    Depth[V - Plate:V + Plate + 1, U - Plate:U + Plate + 1] = round(Camera[2] * 10.0)
    return Image, Depth


class DriftTests(unittest.TestCase):
    """The camera check /locate runs before any model call (CameraDrift)."""

    @classmethod
    def setUpClass(Class):
        Class.Model = Verified(HeadCamera)

    def TestDeskCheckPassesSmallTurnsAndBlocksLargeOnes(Self):
        # Fit-time noise was 0.06-0.08 deg; 0.6 deg and more is a moved camera, down or up.
        for Degrees, Passes in ((0.0, True), (0.2, True), (-0.2, True), (0.6, False), (1.5, False), (3.0, False), (8.0, False),
                                (-10.0, False), (-11.0, False), (-12.0, False), (-13.0, False)):
            with Self.subTest(pitch=Degrees):
                Result = CD.CheckDesk(RenderDepth(Turned(HeadCamera, Degrees), []), Intrinsics, Self.Model, Seq=7)
                Self.assertEqual(Result["ok"], Passes, Result)
                Self.assertAlmostEqual(Result["angle_deg"], abs(Degrees), delta=0.05)
                Self.assertEqual((Result["seq"], Result["reference_fitted_at"]), (7, "2026-09-28T15:04:00"))
                if not Passes:
                    Self.assertEqual(Result["error_code"], "camera_moved")
                    Self.assertIn("moved since calibration at 15:04", Result["error"])
        # Sagging 1 cm on its pivot without turning also blocks; 0.3 cm does not.
        Self.assertEqual(CD.CheckDesk(RenderDepth(Shifted(HeadCamera, [0, 0, -1.0]), []), Intrinsics, Self.Model)["error_code"],
                         "camera_moved")
        Self.assertTrue(CD.CheckDesk(RenderDepth(Shifted(HeadCamera, [0, 0, -0.3]), []), Intrinsics, Self.Model)["ok"])

    def TestDeskCheckIsCheap(Self):
        # 0.11 s on this PC idle. The fastest of five runs, against a bound that scales with how busy the PC is
        # (LoadScale): the mean of five read 0.35 s while other suites ran (audit 2's test table).
        Depth = RenderDepth(HeadCamera, [])
        CD.CheckDesk(Depth, Intrinsics, Self.Model)
        Scale = LoadScale.Factor()
        Took = LoadScale.Best(lambda: CD.CheckDesk(Depth, Intrinsics, Self.Model))
        Self.assertLess(Took, 0.25 * Scale, f"runs in the bridge beside the 30 Hz servo loop: {Took:.3f} s (load x{Scale:.2f})")

    def TestFloorDominantSceneStillFindsTheDesk(Self):
        # A small desk over the room floor: the floor is the dominant plane (it won from 19:06 on).
        Desk = (np.array([-35.0, 25.5, -78.0]), np.array([5.0, 60.0, TableZ]))
        Depth = RenderDepth(HeadCamera, [Desk], FloorZ=-78.0)
        Self.assertLess(DG.CheckPlane(Depth, Intrinsics, HeadCamera)["plane_z_cm"], -70.0)
        Model = Verified(HeadCamera, [Desk], FloorZ=-78.0)
        Self.assertTrue(CD.CheckDesk(Depth, Intrinsics, Model)["ok"])
        Moved = CD.CheckDesk(RenderDepth(Turned(HeadCamera, 3.0), [Desk], FloorZ=-78.0), Intrinsics, Model)
        Self.assertEqual(Moved["error_code"], "camera_moved")
        Self.assertAlmostEqual(Moved["angle_deg"], 3.0, delta=0.1)

    def TestThinItemsOnTheDeskAreNotATurn(Self):
        # The real desk is a small part of the view. A 0.6-0.8 cm mat over a third of it, laid down
        # after calibration, read 1.4-1.8 deg to a plane fit on a still camera; against the desk map
        # it is a separate surface. A real 0.6 deg turn still blocks with the mat there, and a mat
        # over most of the desk blocks rather than pass for the desk.
        Desk = (np.array([-35.0, 25.5, -78.0]), np.array([5.0, 60.0, TableZ]))
        Model = Verified(HeadCamera, [Desk], FloorZ=-78.0)
        Rng = np.random.default_rng(1)

        def Seen(Camera, Items):  # 1 mm depth noise
            Depth = RenderDepth(Camera, [Desk, *Items], FloorZ=-78.0)
            return np.where(Depth > 0, np.clip(Depth + Rng.normal(0.0, 1.0, Depth.shape), 1, None), 0).astype(np.uint16)

        for Thick in (0.6, 0.8):
            with Self.subTest(mat_cm=Thick):
                Mat = (np.array([-35.0, 45.0, TableZ]), np.array([5.0, 60.0, TableZ + Thick]))
                Still = CD.CheckDesk(Seen(HeadCamera, [Mat]), Intrinsics, Model)
                Self.assertTrue(Still["ok"], Still)
                Self.assertLess(Still["angle_deg"], 0.1)
                Self.assertGreater(Still["plane_angle_deg"], 1.0, "a plane fit alone takes the mat for a turn")
                Turned_ = CD.CheckDesk(Seen(Turned(HeadCamera, 0.6), [Mat]), Intrinsics, Model)
                Self.assertEqual((Turned_["ok"], Turned_["error_code"]), (False, "camera_moved"), Turned_)
                Self.assertAlmostEqual(Turned_["angle_deg"], 0.6, delta=0.1)
                Covered = CD.CheckDesk(Seen(HeadCamera, [(np.array([-35.0, 25.5, TableZ]), np.array([5.0, 50.0, TableZ + Thick]))]),
                                       Intrinsics, Model)
                Self.assertFalse(Covered["ok"], Covered)

    def TestHalfCoveredDeskIsUncheckedNotMoved(Self):
        # A 0.8 cm mat over the near part of the desk (to Y 42) on a still camera: 59% of the desk agrees
        # with one pose, under MinAgreeingShare, and the fit reads only the mat's height (-0.9 cm).
        # That is an unchecked camera, not a moved one.
        Desk = (np.array([-35.0, 25.5, -78.0]), np.array([5.0, 60.0, TableZ]))
        Model = Verified(HeadCamera, [Desk], FloorZ=-78.0)
        Mat = (np.array([-35.0, 25.5, TableZ]), np.array([5.0, 42.0, TableZ + 0.8]))
        Depth = RenderDepth(HeadCamera, [Desk, Mat], FloorZ=-78.0)
        Noisy = np.where(Depth > 0, np.clip(Depth + np.random.default_rng(1).normal(0.0, 1.0, Depth.shape), 1, None), 0).astype(np.uint16)
        Result = CD.CheckDesk(Noisy, Intrinsics, Model)
        Self.assertEqual((Result["ok"], Result["error_code"]), (False, "camera_check_failed"), Result)
        Self.assertLess(Result["desk_agreeing_share"], CD.MinAgreeingShare)
        Self.assertIn("could not be checked", Result["error"])

    def TestUnverifiedOrUnmeasurableCamerasAreBlocked(Self):
        Depth = RenderDepth(HeadCamera, [])
        Self.assertEqual(CD.CheckDesk(Depth, Intrinsics, HeadCamera)["error_code"], "camera_not_verified")
        Self.assertEqual(CD.CheckDesk(Depth, Intrinsics, None)["error_code"], "camera_not_verified")
        # A baseline edited, or carried over to a re-fitted pose, no longer matches its checksum.
        Edited = dict(Self.Model, drift_reference=dict(Self.Model["drift_reference"], desk_d=-40.0))
        Refitted = dict(Turned(Self.Model, 1.0), drift_reference=Self.Model["drift_reference"])
        for Model in (Edited, Refitted):
            Result = CD.CheckDesk(Depth, Intrinsics, Model)
            Self.assertEqual(Result["error_code"], "camera_not_verified")
            Self.assertIn("checksum", Result["reason"])
        Self.assertEqual(CD.CheckDesk(None, Intrinsics, Self.Model)["error_code"], "camera_check_failed")
        Self.assertEqual(CD.CheckDesk(np.zeros((Height, Width), np.uint16), Intrinsics, Self.Model)["error_code"],
                         "camera_check_failed")
        Floor = CD.CheckDesk(RenderDepth(HeadCamera, [], FloorZ=-78.0), Intrinsics, Self.Model)  # only the room floor
        Self.assertEqual(Floor["error_code"], "camera_check_failed")
        Self.assertIsNone(Floor["angle_deg"])

    def TestTapeCheckCatchesAPanTheDeskCannotSee(Self):
        Tape = np.array([1.2, -0.6, 0.8])
        Model = dict(Self.Model, marker_corners_gripper_cm=[Tape.tolist()])
        Joints, Point = ArmInView(HeadCamera, Tape)
        Arm = {"ok": True, "joints": Joints.tolist(), "servos_raw": {"wrist_roll": 2048, "gripper": 2100}}
        Image, Depth = TapeView(HeadCamera, Point)
        Still = CD.CheckCapture(Depth, Intrinsics, Model, 3, Image, Arm)
        Self.assertTrue(Still["ok"], Still)
        Self.assertEqual(Still["tape_check"], "ok")
        Self.assertLess(Still["tape_error_cm"], 0.2)
        # The residual the grasp planner corrects by: display frame, plain floats, as long as the error.
        Self.assertEqual(len(Still["tape_residual_cm"]), 3)
        Self.assertTrue(all(type(Value) is float for Value in Still["tape_residual_cm"]))
        Self.assertAlmostEqual(float(np.linalg.norm(Still["tape_residual_cm"])), Still["tape_error_cm"], delta=0.02)
        # A pan about the desk's normal leaves the desk exactly where it was; so does a slide along
        # it. Pans of 8 deg and more put the tape over 5 cm from where the arm is (the head yawed
        # 9-13 deg on 2026-09-28): still the tape, and still a moved camera.
        for Name, Camera, Least in (("pan 3", Turned(HeadCamera, 3.0, "pan"), 1.0), ("pan 8", Turned(HeadCamera, 8.0, "pan"), 5.0),
                                    ("pan 12", Turned(HeadCamera, 12.0, "pan"), 7.5), ("pan -8", Turned(HeadCamera, -8.0, "pan"), 5.0),
                                    ("slide 6 cm", Shifted(HeadCamera, [6.0, 0.0, 0.0]), 5.5)):
            with Self.subTest(camera=Name):
                Image, Depth = TapeView(Camera, Point)
                Panned = CD.CheckCapture(Depth, Intrinsics, Model, 3, Image, Arm)
                Self.assertLess(Panned["angle_deg"], 0.05)
                Self.assertLess(abs(Panned["offset_cm"]), 0.05)
                Self.assertFalse(Panned["ok"], Panned)
                Self.assertEqual((Panned["error_code"], Panned["tape_check"]), ("camera_moved", "moved"))
                Self.assertGreater(Panned["tape_error_cm"], Least)
                if "tape_residual_cm" in Panned:
                    Self.assertAlmostEqual(float(np.linalg.norm(Panned["tape_residual_cm"])), Panned["tape_error_cm"], delta=0.05)
        Image, Depth = TapeView(Turned(HeadCamera, 3.0, "pan"), Point)
        # Without a visible tape, or an arm pose, the tape cannot judge: reported, never blocking.
        Plain = CD.CheckCapture(Depth, Intrinsics, Model, 3, np.full((Height, Width, 3), 90, np.uint8), Arm)
        Self.assertTrue(Plain["ok"])
        Self.assertEqual(Plain["tape_check"], "not_visible")
        Unknown = CD.CheckCapture(Depth, Intrinsics, Model, 3, Image, {"ok": False, "reason": "arm moving"})
        Self.assertEqual((Unknown["ok"], Unknown["tape_check"]), (True, "unavailable"))
        # The offset holds only at the calibration's wrist roll and jaw.
        Posed = dict(Verified(HeadCamera, TapePose={"wrist_roll_raw": 2048, "gripper_raw": 2100}),
                     marker_corners_gripper_cm=[Tape.tolist()])
        Opened = CD.CheckCapture(Depth, Intrinsics, Posed, 3, Image, dict(Arm, servos_raw={"wrist_roll": 2048, "gripper": 2400}))
        Self.assertEqual((Opened["ok"], Opened["tape_check"]), (True, "pose_differs"))
        Self.assertEqual(CD.CheckCapture(Depth, Intrinsics, Posed, 3, Image, Arm)["tape_check"], "moved")

    def TestTapeCheckIgnoresRedThatIsNotTheTape(Self):
        # A still camera and the gripper low beside the red eraser: with the tape turned away, the
        # eraser (4 cm from where the arm puts the tape, 3 cm lower, 9 cm^2) and a sliver of red
        # wiring at the tape's height (0.3 cm^2) are not the tape and must not block as a moved camera.
        Tape = np.array([1.2, -0.6, 0.8])
        Joints, Point = ArmInView(HeadCamera, Tape, TapeHeight=(3.5, 4.5))
        Arm = {"ok": True, "joints": Joints.tolist(), "servos_raw": {"wrist_roll": 2048, "gripper": 2100}}
        U, V = np.round(DG.Project(DG.DisplayToCamera(Point, HeadCamera), Intrinsics)[0]).astype(int)
        Image, Depth = TapeView(HeadCamera, Point, Plate=30)
        [Seen] = SC.DetectRedDetailed(Image)["options"]
        Area = Seen["area"] * (SC._BlobCameraPoint(Depth, Seen, Intrinsics)[2] / Intrinsics["fx"]) ** 2
        Model = dict(Verified(HeadCamera, TapeAreas=[Area] * CD.MinTapeAreas), marker_corners_gripper_cm=[Tape.tolist()])
        HeightOnly = dict(Self.Model, marker_corners_gripper_cm=[Tape.tolist()])  # a baseline without tape areas
        Self.assertAlmostEqual(Model["drift_reference"]["tape_area_cm2"][0], Area, delta=0.01)
        Erasers = [(np.array([Point[0] + DX - 2.25, Point[1] + DY - 1.0, TableZ]), np.array([Point[0] + DX + 2.25, Point[1] + DY + 1.0, TableZ + 1.0]))
                   for DX, DY in ((0.0, -2.5), (3.5, 0.0))]

        def Sliver(Image):
            # Clear of the tape's edge: the detector drops a sliver touching the tape as its side fragment.
            cv2.rectangle(Image, (U + 24, V - 6), (U + 27, V + 6), (30, 30, 215), -1)
            return Image

        for Name, Eraser in zip(("in front", "beside"), Erasers):
            with Self.subTest(eraser=Name):
                Image, Depth = TapeView(HeadCamera, Point, Plate=30, Items=[Eraser])
                Visible = CD.CheckCapture(Depth, Intrinsics, Model, 3, Sliver(Image.copy()), Arm)
                Self.assertEqual((Visible["ok"], Visible["tape_check"], Visible["tape_other_blobs"]), (True, "ok", 2), Visible)
                Image[V - 10:V + 11, U - 10:U + 11] = 235  # the tape turned away
                for Baseline in (Model, HeightOnly):  # the eraser's height alone rules it out
                    Hidden = CD.CheckCapture(Depth, Intrinsics, Baseline, 3, Image, Arm)
                    Self.assertEqual((Hidden["ok"], Hidden["tape_check"]), (True, "not_visible"), Hidden)
                Wiring = CD.CheckCapture(Depth, Intrinsics, Model, 3, Sliver(Image.copy()), Arm)
                Self.assertEqual((Wiring["ok"], Wiring["tape_check"], Wiring["tape_other_blobs"]), (True, "not_visible", 2), Wiring)
        # A red blob of the tape's size, at the tape's height, beside the tip is the tape as far as
        # anything can tell: 1.4 cm off still blocks.
        Image, Depth = TapeView(HeadCamera, Point, Plate=30)
        Image[V - 10:V + 11, U - 10:U + 11] = 235
        cv2.rectangle(Image, (U + 4, V - 10), (U + 24, V + 10), (30, 30, 215), -1)
        Beside = CD.CheckCapture(Depth, Intrinsics, Model, 3, Image, Arm)
        Self.assertEqual((Beside["ok"], Beside["tape_check"]), (False, "moved"), Beside)


RoomFloorZ = -78.0


def Workspace(Items=(), DeskZ=TableZ, DeskRightX=40.0):
    """The real workspace's shape as boxes: the desk (front edge at Y 25.5) and the bin floor in
    front of it, both standing on the room floor (render with FloorZ=RoomFloorZ)."""
    Desk = (np.array([-60.0, DG.DeskFrontEdgeYCm, RoomFloorZ]), np.array([DeskRightX, 90.0, DeskZ]))
    Bin = (np.array([-DG.BinHalfWidthCm, 0.0, RoomFloorZ]), np.array([DG.BinHalfWidthCm, DG.BinRimYCm, DG.BinFloorZCm]))
    return [Desk, Bin, *Items]


def Sloped(Model, Degrees, Pivot=(0.0, 35.0, TableZ)):
    """Model whose display frame is tilted against the rendered world, so the flat rendered desk
    rises Degrees toward +Y in it, as the real desk does (-2.7 at Y 30, -1.9 at Y 40)."""
    Rotation, Translation, Mirror = DG.PoseFromModel(Model)
    Tilt, _ = cv2.Rodrigues(np.array([-math.radians(Degrees), 0.0, 0.0]))
    Anchor = np.asarray(Pivot, dtype=float) * (DG.MirrorVector if Mirror else 1.0)
    New = Rotation @ Tilt
    return dict(Model, rvec=cv2.Rodrigues(New)[0].ravel().tolist(), tvec=(Translation + Rotation @ Anchor - New @ Anchor).tolist())


class SupportTests(unittest.TestCase):
    """MeasureBox's support choice, validity limits and arm masking on the workspace's real shape."""

    @classmethod
    def setUpClass(Class):
        Class.Model = Verified(HeadCamera, Workspace(), FloorZ=RoomFloorZ)
        Class.Surfaces = DG.WorkspaceSurfaces(CD.DisplayDeskPlane(Class.Model), TableZ)

    def Measure(Self, Depth, Box, Model=None, Surfaces=None, Exclude=None):
        return DG.MeasureBox(Depth, Box, Intrinsics, Model or Self.Model, TableZ, True,
                             Surfaces=Surfaces or Self.Surfaces, Exclude=Exclude)

    def Eraser(Self, Y, X=-10.0):
        return np.array([X, Y, TableZ]), np.array([X + 5.0, Y + 2.0, TableZ + 1.1])

    def TestObjectsNearTheDeskEdgeMeasureRightOrFail(Self):
        # 19:00:57's box reached past the desk edge; the basket rule took the bin floor as support.
        for Gap in (0.3, 1.0, 2.0, 4.0, 7.0):
            with Self.subTest(gap=Gap):
                Low, High = Self.Eraser(DG.DeskFrontEdgeYCm + Gap)
                Box = CaptureBox(HeadCamera, Low, High)
                Box[3] += 25.0 / Height  # the box reaches 25 px past the eraser, toward the bin
                Result = Self.Measure(RenderDepth(HeadCamera, Workspace([(Low, High)]), FloorZ=RoomFloorZ), Box)
                if Result["ok"]:
                    Self.assertAlmostEqual(Result["z"], TableZ, delta=0.3, msg=Result)
                    Self.assertAlmostEqual(Result["height_cm"], 1.1, delta=0.3, msg=Result)
                    Self.assertEqual(Result["support_branch"], "desk", Result)
                Self.assertTrue(Result["ok"] or Gap < 4.0, Result)

    def TestObjectInTheBinStandsOnTheBinFloor(Self):
        Low, High = np.array([-12.0, 10.0, DG.BinFloorZCm]), np.array([-8.0, 14.0, DG.BinFloorZCm + 5.0])
        Result = Self.Measure(RenderDepth(HeadCamera, Workspace([(Low, High)]), FloorZ=RoomFloorZ), CaptureBox(HeadCamera, Low, High))
        Self.assertTrue(Result["ok"], Result)
        Self.assertEqual(Result["support_branch"], "bin")
        Self.assertAlmostEqual(Result["z"], DG.BinFloorZCm, delta=0.3)
        Self.assertAlmostEqual(Result["height_cm"], 5.0, delta=0.4)
        # Standing on a wide object in the bin, with no bin floor in view around it.
        Under = (np.array([-21.0, 1.0, DG.BinFloorZCm]), np.array([1.0, 21.0, DG.BinFloorZCm + 3.0]))
        Low, High = np.array([-12.0, 10.0, DG.BinFloorZCm + 3.0]), np.array([-8.0, 14.0, DG.BinFloorZCm + 7.0])
        Result = Self.Measure(RenderDepth(HeadCamera, Workspace([Under, (Low, High)]), FloorZ=RoomFloorZ),
                              CaptureBox(HeadCamera, Low, High))
        Self.assertTrue(Result["ok"], Result)
        Self.assertEqual(Result["support_branch"], "object")
        Self.assertAlmostEqual(Result["z"], DG.BinFloorZCm + 3.0, delta=0.3)
        Self.assertAlmostEqual(Result["height_cm"], 4.0, delta=0.4)

    def TestMisplacedDeskOrCameraIsNotMeasured(Self):
        # The desk 4.6 cm low (19:00:57) or 6 cm high (19:04:11, the 7.5 cm eraser), or the
        # camera pitched 10-13 deg up (19:39:37): never a measurement.
        Low, High = Self.Eraser(33.0)
        for Name, DeskZ in (("desk 4.6 cm low", TableZ - 4.6), ("desk 6 cm high", TableZ + 6.0)):
            with Self.subTest(Name):
                Shift = DeskZ - TableZ
                Item = (Low + [0, 0, Shift], High + [0, 0, Shift])
                Depth = RenderDepth(HeadCamera, Workspace([Item], DeskZ=DeskZ), FloorZ=RoomFloorZ)
                Result = Self.Measure(Depth, CaptureBox(HeadCamera, *Item))
                Self.assertFalse(Result["ok"], Result)
        for Degrees in (10.0, 11.0, 12.0, 13.0):
            with Self.subTest(pitch_up=Degrees):
                Truth = Turned(HeadCamera, -Degrees)
                Depth = RenderDepth(Truth, Workspace([(Low, High)]), FloorZ=RoomFloorZ)
                Result = Self.Measure(Depth, CaptureBox(Truth, Low, High))
                Self.assertFalse(Result["ok"], Result)

    def TestFarObjectOnTheSlopedDeskHasAMeasuredSupport(Self):
        # The desk rises 4.6 deg toward the far side; at Y 65 a flat table_z prior misses it by 2.4 cm.
        Model = Verified(Sloped(HeadCamera, 4.6), Workspace(), FloorZ=RoomFloorZ, Truth=HeadCamera)
        Surfaces = DG.WorkspaceSurfaces(CD.DisplayDeskPlane(Model), TableZ)
        Self.assertAlmostEqual(float(DG.DeskHeight(Surfaces, 0.0, 30.0)), TableZ - 0.4, delta=0.05)
        Self.assertAlmostEqual(float(DG.DeskHeight(Surfaces, 0.0, 40.0)), TableZ + 0.4, delta=0.05)
        Low, High = np.array([-18.0, 63.0, TableZ]), np.array([-12.0, 67.0, TableZ + 6.0])
        Depth = RenderDepth(HeadCamera, Workspace([(Low, High)]), FloorZ=RoomFloorZ)
        Box = CaptureBox(HeadCamera, Low, High)
        Result = Self.Measure(Depth, Box, Model, Surfaces)
        Self.assertTrue(Result["ok"], Result)
        Self.assertTrue(Result["support_measured"])
        Self.assertAlmostEqual(Result["height_cm"], 6.0, delta=0.4)
        Self.assertLess(max(Result["size_cm"]), 8.0, Result)
        Self.assertAlmostEqual(Result["z"], float(DG.DeskHeight(Surfaces, Result["contact_cm"][0], Result["contact_cm"][1])), delta=0.3)
        # On the flat prior the rising desk around it joins the object (or no support is found).
        Flat = Self.Measure(Depth, Box, Model, DG.WorkspaceSurfaces(None, TableZ))
        Self.assertTrue(not Flat["ok"] or max(Flat["size_cm"]) > 10.0, Flat)

    def TestGarbageDepthIsNotAnObject(Self):
        Depth = RenderDepth(HeadCamera, Workspace(DeskRightX=10.0), FloorZ=RoomFloorZ)
        Box = [0.3, 0.3, 0.4, 0.4]
        U0, V0, U1, V1 = DG.BoxToNative(Box, Width, Height, True)
        Sparse = Depth.copy()
        Sparse[V0:V1, U0:U1] = 0  # the 19:06 "white box": 1% depth, the rest missing or far background
        Sparse[V0:V1:10, U0:U1:10] = Depth[V0:V1:10, U0:U1:10]
        Sparse[V0 + 5:V1:10, U0 + 5:U1:10] = 5500
        Result = Self.Measure(Sparse, Box)
        Self.assertFalse(Result["ok"])
        Self.assertIn("only 1% of the box has depth within 120 cm", Result["error"])
        # A box straddling the desk's right edge over the room floor 75 cm below.
        Edge = CaptureBox(HeadCamera, np.array([5.0, 38.0, TableZ]), np.array([15.0, 44.0, TableZ + 3.0]))
        Result = Self.Measure(Depth, Edge)
        Self.assertFalse(Result["ok"], Result)
        Self.assertFalse(DG.MeasureBox(Depth, Edge, Intrinsics, Self.Model, TableZ, True)["ok"])  # without surfaces too

    def TestArmIsMaskedOutOfTheBox(Self):
        # The forearm in front of the 15:19 cup made it read 17.5 cm tall.
        Cup = (np.array([-18.0, 32.0, TableZ]), np.array([-12.0, 38.0, TableZ + 9.0]))
        # The forearm reaching past the cup's left side, from above and in front.
        Link = (np.array([-21.0, 24.0, TableZ + 5.0]), np.array([-19.0, 40.0, TableZ + 14.0]))
        Joints = [[-20.0, 10.0, TableZ + 9.5], [-20.0, 24.0, TableZ + 9.5], [-20.0, 40.0, TableZ + 9.5], [-20.0, 44.0, TableZ + 9.5]]
        Depth = RenderDepth(HeadCamera, Workspace([Cup, Link]), FloorZ=RoomFloorZ)
        Box = CaptureBox(HeadCamera, *Cup)
        Plain = Self.Measure(Depth, Box)
        Self.assertGreater(Plain["top_z"], TableZ + 11.0, "the link inflates the cup without the mask")
        Masked = Self.Measure(Depth, Box, Exclude=DG.ArmExclusion(Joints))
        Self.assertTrue(Masked["ok"], Masked)
        Self.assertAlmostEqual(Masked["top_z"], TableZ + 9.0, delta=0.5)
        Self.assertGreater(Masked["arm_fraction"], 0.0)
        ArmOnly = Self.Measure(Depth, CaptureBox(HeadCamera, *Link), Exclude=DG.ArmExclusion(Joints))
        Self.assertFalse(ArmOnly["ok"])
        Self.assertTrue(ArmOnly["robot_arm"])


# The day it was written for, from the recordings (RecordedCaptures: a missing one fails the test, RECORDING MISSING).
# The fit's run survived the deletion of 2026-10-03; the captures of the failed task (recording/images) did not.
RealRun = RC.Day("2026-09-28") / "selfcal" / "runs" / "20260928_150208_0a49ed111d2b"
RealRecording = RC.Day("2026-09-28") / "recording" / "images"
RealModel = Root / "camera_calibration.stale-camera-moved-2026-09-28.bak.json"
# The depth PNGs the model saw on 2026-09-28 (turbo, clipped to the near/far quoted in its prompt):
# the 15:19 task (1.3 deg after the 15:04 fit), the eraser task (19:00-19:09) and its 19:39 rerun.
RealCaptures = [("15:19:15", "1fba70df06522d7a37f90405b4bac19e291805e3", 29.5, 176.8),
                ("19:00:57", "6f821598dc770002ff9f2e06da086d214d9376b9", 30.6, 154.0),
                ("19:02:02", "520c61a3d7a7213fa0ae05baad1f42d733f20145", 29.6, 151.0),
                ("19:03:18", "93e374042dbdc968256c3815d171e6f6edb6c0dc", 29.9, 148.7),
                ("19:04:12", "bf5d1f3b2fa6a0560df2ca2d2a22f77367445f94", 29.7, 146.4),
                ("19:05:14", "a18b80fb81120167df078d7ae4600c6ea9285aa2", 29.6, 151.1),
                ("19:06:25", "f8635a08aef8ba153b3a511d98ada8246d41751f", 29.3, 148.8),
                ("19:07:49", "1696e5655b1a98aa8341c0bfc322f1e9f19f5e75", 29.4, 148.5),
                ("19:08:54", "781904d4ff6d81697de9d4252c7490fdc16f9d4e", 29.4, 148.2),
                ("19:39:37", "72e63845a4a3bfb57b810e923578af5babc996b8", 31.5, 148.5)]


def DecodeTurbo(Path_, NearCm, FarCm):
    """Depth (mm) back from OrbbecCamera.ColorizeDepth's turbo PNG: every colour is one of the
    256 levels, so depth is exact to (far - near) / 255, about 0.47 cm."""
    Lut = cv2.applyColorMap(np.arange(256, dtype=np.uint8).reshape(-1, 1), cv2.COLORMAP_TURBO).reshape(256, 3).astype(np.int64)
    Keys = (Lut[:, 0] << 16) | (Lut[:, 1] << 8) | Lut[:, 2]
    Order = np.argsort(Keys)
    Image = cv2.imread(str(Path_)).astype(np.int64)
    Code = (Image[..., 0] << 16) | (Image[..., 1] << 8) | Image[..., 2]
    Position = np.clip(np.searchsorted(Keys[Order], Code), 0, 255)
    Level = Order[Position]
    Exact = (Keys[Order][Position] == Code) & (Level > 0) & (Level < 255)  # 0 and 255 are clipped
    return np.where(Exact, np.round((NearCm + (Level + 0.5) / 255.0 * (FarCm - NearCm)) * 10.0), 0).astype(np.uint16)


def RealFitPlanes():
    """Desk planes of the 15:04 fit's own raw depth (72 frames), as a fit records them."""
    Planes = []
    for Capture in sorted((RealRun / "observations").glob("*_pose*.json")):
        Record = json.loads(Capture.read_text(encoding="utf-8"))
        for Index, Name in enumerate(Record["depth_frames_png"] or []):
            Plane = CD.DeskPlane(cv2.imread(str(Capture.parent / Name), cv2.IMREAD_UNCHANGED), Record["depth_intrinsics"])
            if Plane is not None:
                Planes.append(dict(Plane, pose_index=Record["pose_index"], received_at=Record["received_at"], frame=Index,
                                   file=str(Capture.parent / Name)))
    return Planes


def RealTapeAreas(Model):
    """The tape's area (cm^2) at each 15:04 pose the fit kept, as the solve hands them to the baseline:
    tracked blobs whose depth agrees with the fitted camera (pose 13's was a far red blob)."""
    Session = json.loads((RealRun / "Session.json").read_text(encoding="utf-8"))
    Tape = np.asarray(Model["marker_corners_gripper_cm"][0])
    Areas = []
    for Observation in Session["observations"]:
        if Observation.get("camera_cm") is None:
            continue
        Predicted = DG.DisplayToCamera(np.asarray(Observation["tip"]) + np.asarray(Observation["frame"]) @ Tape, Model)[0]
        if np.linalg.norm(np.asarray(Observation["camera_cm"]) - Predicted) < 2.0:
            Areas.append(Observation["area_px"] * (Observation["camera_cm"][2] / Session["depth_intrinsics"]["fx"]) ** 2)
    return Areas


class RealDataTests(unittest.TestCase):
    """The camera gate on the day it was written for."""

    @classmethod
    def setUpClass(Class):
        RC.Need(RealRun / "observations", RealRun / "Session.json", RealModel, What="the 2026-09-28 15:04 fit's run")
        Class.Planes = RealFitPlanes()
        Class.Model = json.loads(RealModel.read_text(encoding="utf-8"))["camera_model"]
        Class.Camera = {Key: Class.Model[Key] for Key in ("fx", "fy", "cx", "cy", "dist")} | {"width": 1280, "height": 720}

    def TestTheFitsOwnFramesPassTheGate(Self):
        Planes, Camera = Self.Planes, Self.Camera
        Self.assertEqual(len(Planes), 72)
        # A live fit's planes name their pose archive and frame (SelfCalibration._RecordDeskPlanes), not a file.
        Capture = sorted((RealRun / "observations").glob("*_pose*.json"))[0]
        Self.assertTrue(np.array_equal(CD.PlaneDepth({"archive": str(Capture), "frame": 1}),
                                       cv2.imread(Planes[1]["file"], cv2.IMREAD_UNCHANGED)))
        Reference, Problem = CD.BuildReference(Planes, Self.Model, Intrinsics=Camera)
        Self.assertIsNone(Problem)
        Model = dict(Self.Model, drift_reference=Reference)
        for Plane in Planes:
            Result = CD.CheckDesk(cv2.imread(Plane["file"], cv2.IMREAD_UNCHANGED), Camera, Model)
            Self.assertTrue(Result["ok"], (Plane["file"], Result))

    def TestGateBlocksEveryCaptureOfTheFailedTask(Self):
        # The task's own captures were deleted on 2026-10-03: until they are restored this fails, RECORDING MISSING
        # (TestThinItemsOnTheRealDeskAreNotATurn still turns the fit's own frame 0.6 and 1 deg and sees it blocked).
        RC.Need(*(RealRecording / f"{Sha}.png" for _, Sha, _, _ in RealCaptures), What="the 2026-09-28 failed task's captures (recording/images)")
        Camera = Self.Camera
        Reference, Problem = CD.BuildReference(Self.Planes, Self.Model, Intrinsics=Camera)
        Self.assertIsNone(Problem)
        Model = dict(Self.Model, drift_reference=Reference)
        for Clock, Sha, Near, Far in RealCaptures:
            with Self.subTest(capture=Clock):
                Result = CD.CheckDesk(DecodeTurbo(RealRecording / f"{Sha}.png", Near, Far), Camera, Model)
                Self.assertEqual(Result["error_code"], "camera_moved", Result)
                Self.assertGreater(Result["angle_deg"], 1.0)

    def TestThinItemsOnTheRealDeskAreNotATurn(Self):
        # A still camera and a notebook, mat or sheet (0.4-0.8 cm) laid on the real desk after the
        # fit: a plane fit read up to 0.64 deg (the review's probe). A real frame re-rendered from a
        # camera turned 0.6 or 1 deg still blocks, with or without a mat. The 1 deg turn with the mat
        # leaves 62% of the desk agreeing, where a mat alone reads up to 2.4 deg: it blocks as an
        # unchecked camera rather than a moved one.
        Reference, _ = CD.BuildReference(Self.Planes, Self.Model, Intrinsics=Self.Camera)
        Model = dict(Self.Model, drift_reference=Reference)
        Normal = np.asarray(Reference["desk_n"])
        Depth = cv2.imread(Self.Planes[36]["file"], cv2.IMREAD_UNCHANGED)
        Points = (np.concatenate([DG.RayMap(Self.Camera), np.ones((Height, Width, 1))], axis=-1) * (Depth / 10.0)[..., None]).reshape(-1, 3)
        World = DG.CameraToDisplay(Points, Model).reshape(Height, Width, 3)
        Surfaces = DG.WorkspaceSurfaces(CD.DisplayDeskPlane(Model), Model["table_z"])
        OnDesk = (Depth > 0) & (np.abs(World[..., 2] - DG.DeskHeight(Surfaces, World[..., 0], World[..., 1])) < 0.3) & (World[..., 1] >= 27)

        def Region(X0, X1, Y0, Y1):
            return OnDesk & (World[..., 0] >= X0) & (World[..., 0] < X1) & (World[..., 1] >= Y0) & (World[..., 1] < Y1)

        def Rendered(Raised=None, Thick=0.0, PitchDeg=0.0):
            """The frame from a camera turned PitchDeg, with Raised desk pixels lifted Thick along the desk normal."""
            Lifted = Points + (Thick * Normal * Raised.reshape(-1, 1) if Raised is not None else 0.0)
            Moved = Lifted[Depth.reshape(-1) > 0] @ cv2.Rodrigues(np.array([math.radians(PitchDeg), 0.0, 0.0]))[0].T
            U, V = np.round(DG.Project(Moved, Self.Camera)).astype(int).T
            Inside = (U >= 0) & (U < Width) & (V >= 0) & (V < Height)
            Out = np.full((Height, Width), 65535, np.uint16)
            np.minimum.at(Out, (V[Inside], U[Inside]), np.round(Moved[Inside, 2] * 10.0).astype(np.uint16))
            return np.where(Out == 65535, 0, Out).astype(np.uint16)

        Mat = Region(-30, 10, 28, 60)
        for Name, Raised in (("notebook", Region(0, 21, 30, 60)), ("mat", Mat), ("sheet", Region(10, 40, 28, 45))):
            for Thick in (0.4, 0.6, 0.8):
                with Self.subTest(item=Name, cm=Thick):
                    Lifted = Points.reshape(Height, Width, 3)[Raised] + Thick * Normal
                    Scene = Depth.copy()
                    Scene[Raised] = np.round(Lifted[:, 2] * 10.0).astype(np.uint16)  # the review's probe
                    Result = CD.CheckDesk(Scene, Self.Camera, Model)
                    Self.assertTrue(Result["ok"], Result)
                    Self.assertLess(Result["angle_deg"], 0.05)
        for Raised, Pitch, Code in ((None, 0.6, "camera_moved"), (None, 1.0, "camera_moved"), (Mat, 0.6, "camera_moved"),
                                    (Mat, 1.0, "camera_check_failed")):
            with Self.subTest(mat=Raised is not None, pitch=Pitch):
                Result = CD.CheckDesk(Rendered(Raised, 0.8, Pitch), Self.Camera, Model)
                Self.assertEqual((Result["ok"], Result["error_code"]), (False, Code), Result)
                Self.assertAlmostEqual(Result["angle_deg"], Pitch, delta=0.12)

    def TestHalfCoveredRealDeskIsUncheckedNotMoved(Self):
        # A still camera and a 0.7-0.8 cm mat over about half of the real desk: too little of the desk
        # agrees with one camera pose to trust the fit, and what it reads (the mat's own height, or a
        # ramp from the desk onto it) a thin item fakes. The check fails closed as unchecked; it used
        # to read "camera moved, recalibrate" (-0.7/-0.8 cm, and 1.3 deg, at shares 0.51-0.62 here).
        Reference, _ = CD.BuildReference(Self.Planes, Self.Model, Intrinsics=Self.Camera)
        Model = dict(Self.Model, drift_reference=Reference)
        Normal, Offset = np.asarray(Reference["desk_n"]), float(Reference["desk_d"])
        Depth = cv2.imread(Self.Planes[36]["file"], cv2.IMREAD_UNCHANGED)
        Rays = np.concatenate([DG.RayMap(Self.Camera), np.ones((Height, Width, 1))], axis=-1).astype(np.float64)
        Desk = (Depth > 0) & (np.abs((Rays * (Depth / 10.0)[..., None]) @ Normal - Offset) < 1.0)
        Rows, Columns = np.nonzero(Desk)
        U, V = np.arange(Width)[None, :], np.arange(Height)[:, None]

        def Half(Values):  # half the side of a box over 45% of the desk's 3-97 percentile span
            return (np.percentile(Values, 97) - np.percentile(Values, 3)) * math.sqrt(0.45) / 2

        def Covered(Mask, Thick):
            """The frame with the desk under Mask raised Thick cm, each pixel along its own ray."""
            Hit = Mask & Desk
            Scene = Depth.astype(np.float64)
            Scene[Hit] += Thick * 10.0 / (Rays[Hit] @ Normal)
            return np.round(Scene).astype(np.uint16)

        Right = np.broadcast_to(U >= np.median(Columns), (Height, Width))
        Middle = (np.abs(U - np.median(Columns)) <= Half(Columns)) & (np.abs(V - np.median(Rows)) <= Half(Rows))
        for Name, Mask, Thick in (("right half", Right, 0.7), ("right half", Right, 0.8), ("middle", Middle, 0.8)):
            with Self.subTest(mat=Name, cm=Thick):
                Result = CD.CheckDesk(Covered(Mask, Thick), Self.Camera, Model)
                Self.assertEqual((Result["ok"], Result["error_code"]), (False, "camera_check_failed"), Result)
                Self.assertLess(Result["desk_agreeing_share"], CD.MinAgreeingShare)

    def TestTapeCheckAgreesWithTheFitsOwnPoses(Self):
        # No false alarm: each 15:04 pose's tape, measured in its own raw depth, sits where the
        # arm's joints and the fitted tape offset put it (0.24 cm median, 0.73 worst). The eraser
        # on the desk (9 cm^2, 9-17 cm below the tape) and slivers of arm wiring (0.05-0.2 cm^2,
        # 15-23 cm away at the tape's height) are in view too, and must not count as the tape.
        Captures = [json.loads(Path_.read_text(encoding="utf-8")) for Path_ in sorted((RealRun / "observations").glob("*_pose*.json"))]
        Servos = Captures[0]["metadata"]["bridge_state_at_receive"]["servos"]
        Reference, _ = CD.BuildReference(Self.Planes, Self.Model, {"wrist_roll_raw": Servos["wrist_roll"]["present_raw"],
                                                                   "gripper_raw": Servos["gripper"]["present_raw"]},
                                         Intrinsics=Self.Camera, TapeAreasCm2=RealTapeAreas(Self.Model))
        Self.assertIsNotNone(Reference["tape_area_cm2"])
        Model = dict(Self.Model, drift_reference=Reference)
        Camera = Self.Camera
        States = []
        for Capture in Captures:
            Arm = {"ok": True, "joints": [[Point["x"], Point["y"], Point["z"]] for Point in Capture["joints"]],
                   "servos_raw": {Name: Servo["present_raw"] for Name, Servo in Capture["metadata"]["bridge_state_at_receive"]["servos"].items()}}
            Image = SC.DecodeDataUrl(Capture["images"][0])
            Depth = cv2.imread(str(RealRun / "observations" / Capture["depth_frames_png"][0]), cv2.IMREAD_UNCHANGED)
            Result = CD.CheckCapture(Depth, Camera, Model, 1, cv2.flip(Image, 1) if Capture["mirrored"] else Image, Arm)
            Self.assertTrue(Result["ok"], Result)
            States.append(Result["tape_check"])
        Self.assertGreaterEqual(States.count("ok"), 20, States)
        Self.assertNotIn("ambiguous", States)


def CalibrationPoses(Model, Tape, Count=22, Seed=3, AngleNoiseDeg=0.1):
    """Random reachable arm poses whose tip tape the camera sees: (tips, frames, pixels, camera points)."""
    Controller = Bridge.controller
    Rng = np.random.default_rng(Seed)
    Limits = {Name: sorted((Controller._ik_angle_from_raw(Servo, Servo.range_min), Controller._ik_angle_from_raw(Servo, Servo.range_max)))
              for Name, Servo in Controller.servos.items()}
    Current = Controller._current_ik_angles()
    Rows = []
    for _ in range(4000):
        Angles = {Name: float(Rng.uniform(*Limits[Name])) if Name in Bridge.IK_JOINTS else Current[Name]
                  for Name in Bridge.MOTOR_NAMES}
        Joints = np.array([[P["x"], P["y"], P["z"]] for P in Geo.forward_joint_positions(Controller.arm_geometry, Angles)])
        Display = Joints * DG.MirrorVector + DisplayOffset
        Tip, Frame = SC.GripperFrame(Display)
        if Tip[2] < TableZ + 2.0 or Tip[1] < 8.0:
            continue
        Camera = DG.DisplayToCamera(Tip + Frame @ Tape, Model)[0]
        Pixel = DG.Project(Camera, Intrinsics)[0]
        if Camera[2] <= 5 or not (40 < Pixel[0] < Width - 40 and 40 < Pixel[1] < Height - 40):
            continue
        Noisy = {Name: Value + Rng.normal(0, AngleNoiseDeg) for Name, Value in Angles.items()}
        Reported = np.array([[P["x"], P["y"], P["z"]] for P in Geo.forward_joint_positions(Controller.arm_geometry, Noisy)])
        Tip, Frame = SC.GripperFrame(Reported * DG.MirrorVector + DisplayOffset)
        Rows.append((Tip, Frame, Pixel + Rng.normal(0, 0.5, 2),
                     Camera + np.array([Rng.normal(0, 0.15), Rng.normal(0, 0.15), Rng.normal(0, 0.25)])))
        if len(Rows) == Count:
            break
    return Rows


class DepthCalibrationTests(unittest.TestCase):
    def setUp(Self):
        Self.Patches = [patch.object(SC, "SessionPath", Artifacts / "DepthSession.json"),
                        patch.object(SC, "CapturesDir", Artifacts / "DepthCaptures")]
        for Patch in Self.Patches:
            Patch.start()
        SC.ResetSession()
        SC._RedTracker.clear()

    def tearDown(Self):
        for Patch in Self.Patches:
            Patch.stop()
        SC._Session.clear()
        SC._Session.update({"image_size": None, "mirrored": None, "observations": []})
        SC._RedTracker.clear()

    def Inject(Self, Rows, WithDepth=True, DepthScale=1.0, Desk=None, DeskTiltDeg=0.0, Scene=(), FloorZ=TableZ):
        """Desk: the true camera, whose rendered desk gives each pose's desk plane as a real run
        records it; DeskTiltDeg turns that camera steadily across the run (a creeping camera), so
        each pose's depth and its plane come from the camera as it then stood. Scene: boxes
        rendered with the desk, whose plane sits at FloorZ (RenderDepth)."""
        Static = {"centroid": [180.0, 610.0], "area": 520.0, "camera_cm": [-40.0, 30.0, 90.0]}
        with SC._Lock:
            SC._Session.update(image_size=[Width, Height], mirrored=True, red_poses=[], observations=[], desk_planes=[])
            if WithDepth:
                SC._Session["depth_intrinsics"] = dict(Intrinsics)
            if Desk is not None:
                Depth = RenderDepth(Desk, list(Scene), FloorZ=FloorZ)
                Plane = dict(CD.DeskPlane(Depth, Intrinsics), file=DepthFile(Depth))
                for Index in range(len(Rows)):
                    if DeskTiltDeg:
                        Depth = RenderDepth(Turned(Desk, DeskTiltDeg * Index / max(len(Rows) - 1, 1)), list(Scene), FloorZ=FloorZ)
                        Plane = dict(CD.DeskPlane(Depth, Intrinsics), file=DepthFile(Depth))
                    SC._Session["desk_planes"].append(dict(Plane, pose_index=Index, received_at=float(Index),
                                                           wrist_roll_raw=2048, gripper_raw=2100))
            for Index, (Tip, Frame, Pixel, Camera) in enumerate(Rows):
                Point = (Camera * DepthScale).tolist() if WithDepth else None
                Tape = {"centroid": Pixel.tolist(), "area": 300.0, "camera_cm": Point}
                SC._Session["red_poses"].append({"pose_index": Index, "tip": Tip.tolist(), "frame": Frame.tolist(),
                                                 "candidates": [Tape, dict(Static)]})
                SC._Session["observations"].append({"pose_index": Index, "tip": Tip.tolist(), "frame": Frame.tolist(),
                                                    "corners": [Pixel.tolist()], "method": "red", "camera_cm": Point})

    def TestDepthSolveRecoversCameraAndMeasuresObjects(Self):
        Tape = np.array([1.2, -0.6, 0.8])
        for Name, Truth in (("front", FrontCamera), ("head", HeadCamera)):
            with Self.subTest(camera=Name):
                Rows = CalibrationPoses(Truth, Tape)
                Self.assertGreaterEqual(len(Rows), 15, "not enough visible poses")
                Self.Inject(Rows, Desk=Truth)
                Report = SC.Solve(TableZ=TableZ, Save=False)
                Self.assertTrue(Report["ok"] and Report["quality_ok"], Report.get("problems"))
                Self.assertEqual(Report["method"], "depth")
                # The baseline is saved, intact, and the camera's own desk passes its check.
                Self.assertTrue(Report["drift_baseline"])
                Self.assertTrue(CD.CheckDesk(RenderDepth(Truth, []), Intrinsics, Report["model"])["ok"])
                Self.assertEqual(Report["model"]["drift_reference"]["tape_pose"], {"wrist_roll_raw": 2048, "gripper_raw": 2100})
                # The tape's area at its depth (300 px) at each pose the fit used, which tells the tape from other red at
                # /locate. The robust assignment may leave a pose out of its consensus (for the head camera its nearest, the
                # tape 14 cm away, where the joints' noise is the most pixels), and the fit keeps the band of the poses it
                # used (SelfCalibration._SolveDepth). The band used to be checked over every pose, and failed for the head
                # camera from the day it was written (0a5879a): the "known" failure of both audits.
                Used = [Pose["pose_index"] for Pose in Report["per_pose"] if not Pose["dropped"]]
                Self.assertGreaterEqual(len(Used), len(Rows) - 1, Report["per_pose"])
                Areas = [300.0 * (Rows[Index][3][2] / Intrinsics["fx"]) ** 2 for Index in Used]
                Low, High = Report["model"]["drift_reference"]["tape_area_cm2"]
                Self.assertAlmostEqual(Low, float(np.percentile(Areas, 10)), delta=0.02)
                Self.assertAlmostEqual(High, float(np.percentile(Areas, 90)), delta=0.02)
                Self.assertTrue(Report["mirror_x"])
                TrueEye = DG.CameraToDisplay(np.zeros(3), Truth)[0]
                Self.assertLess(float(np.linalg.norm(np.array(Report["camera_position_cm"]) - TrueEye)), 0.8, Report)
                Self.assertLess(float(np.linalg.norm(np.array(Report["tape_offset_cm"]) - Tape)), 0.6, Report)
                Self.assertLess(Report["depth_rms_cm"], 0.6)
                Self.assertEqual(Report["tape_assignment"]["assigned"], len(Report["per_pose"]))
                Model = Report["model"]
                Self.assertTrue(DG.ModelUsesDepthCamera(Model, Intrinsics))
                # Objects measured through the solved camera land where they are.
                Low, High = np.array([-19.0, 24.0, TableZ]), np.array([-13.0, 29.0, TableZ + 6.0])
                Result = DG.MeasureBox(RenderDepth(Truth, [(Low, High)]), CaptureBox(Truth, Low, High), Intrinsics,
                                       Model, TableZ, True)
                Self.assertTrue(Result["ok"], Result)
                Self.assertLess(math.hypot(Result["x"] + 16.0, Result["y"] - 26.5), 0.8, Result)
                Self.assertAlmostEqual(Result["height_cm"], 6.0, delta=0.5)
                # The table-plane fallback (/locate without depth) works with the 5-term distortion.
                Point = np.array([-12.0, 30.0, TableZ])
                U, V = DG.Project(DG.DisplayToCamera(Point, Truth), Intrinsics)[0]
                Mapped = SC.PixelToPlane(Model, Width - (U + 0.5), V + 0.5)
                Self.assertLess(math.hypot(Mapped["x"] - Point[0], Mapped["y"] - Point[1]), 0.6, Mapped)

    def AssertKeptWithWarning(Self, Report, Text):
        """A fit that used to be rejected: computed and kept (saved by the bridge), the problem a warning with its numbers."""
        Self.assertTrue(Report["ok"] and Report["computed"], Report.get("error"))
        Self.assertFalse(Report["quality_ok"])
        Self.assertTrue(any(Text in Problem for Problem in Report["problems"]), Report["problems"])
        Self.assertEqual(Report["model"]["quality_problems"], Report["problems"], "the warnings are saved with the calibration")
        Self.assertIn("Reject calibration", Report["quality_warning"])
        Self.assertNotIn("error", Report)

    def TestCameraMovingDuringTheFitIsAWarning(Self):
        # 15:04 on 2026-09-28: the camera turned 0.3 deg during the fit, and nothing noticed. It is
        # flagged now, as a warning on the saved calibration: only the user rejects one (2026-10-10).
        Rows = CalibrationPoses(HeadCamera, np.array([1.2, -0.6, 0.8]))
        Self.Inject(Rows, Desk=HeadCamera, DeskTiltDeg=0.3)
        Report = SC.Solve(TableZ=TableZ, Save=False)
        Self.AssertKeptWithWarning(Report, "the camera moved 0.2")
        Self.AssertKeptWithWarning(Report, "lock the camera head")
        Self.assertTrue(Report["drift_baseline"], "the baseline still checks the camera later")
        Self.Inject(Rows, Desk=HeadCamera, DeskTiltDeg=0.05)  # noise-sized creep
        Report = SC.Solve(TableZ=TableZ, Save=False)
        Self.assertTrue(Report["ok"] and Report["quality_ok"], Report["problems"])
        Self.assertLess(Report["fit_camera_motion"]["angle_deg"], 0.05)
        Self.assertEqual(len(Report["model"]["fit_desk_planes"]), len(Rows))
        # A depth session whose desk was never measured cannot be checked later: kept, without a
        # baseline, and the warning says what that means for the camera check.
        Self.Inject(Rows)
        Report = SC.Solve(TableZ=TableZ, Save=False)
        Self.AssertKeptWithWarning(Report, "desk plane was found in depth at only 0")
        Self.AssertKeptWithWarning(Report, "every task that acts on an object stops (camera not verified)")
        Self.assertFalse(Report["drift_baseline"])
        Self.assertNotIn("drift_reference", Report["model"])
        Self.assertEqual(CD.CheckDesk(RenderDepth(HeadCamera, []), Intrinsics, Report["model"])["error_code"], "camera_not_verified",
                         "the camera check is unchanged: no baseline, not verified")
        # Nor can one whose desk was found at only 3 poses, though their 9 frames make a baseline:
        # a camera creeping during the run would go unseen.
        Self.Inject(Rows, Desk=HeadCamera)
        with SC._Lock:
            SC._Session["desk_planes"] = [dict(Plane, frame=Frame) for Plane in SC._Session["desk_planes"][:3] for Frame in range(3)]
        Report = SC.Solve(TableZ=TableZ, Save=False)
        Self.AssertKeptWithWarning(Report, "camera stillness could not be verified: the desk was found in depth at only 3 poses")
        Self.assertTrue(Report["drift_baseline"])

    def TestDeskBesideAWallIsFoundNearTheFittedCamera(Self):
        # 2026-10-01: the camera looked down at a bin under the arm (the only surface at desk height,
        # 7% of the view) beside a bookshelf, over a floor too far away to count. Every pose recorded
        # the bookshelf's side as "the desk" (90 deg from it), so the baseline found 0-5 usable frames
        # and the planes flipping between vertical surfaces read 7-15 deg of motion: four good tape
        # fits were rejected. Here a desk patch at the table's height sits beside a wall that holds
        # far more depth pixels, over a floor 80 cm down.
        Rows = CalibrationPoses(HeadCamera, np.array([1.2, -0.6, 0.8]))
        Patch = ([-30.0, 18.0, TableZ - 3.0], [0.0, 38.0, TableZ])
        Wall = ([-80.0, 48.0, TableZ - 80.0], [40.0, 50.0, TableZ + 70.0])
        Scene, Floor = [Patch, Wall], TableZ - 80.0
        Self.Inject(Rows, Desk=HeadCamera, Scene=Scene, FloorZ=Floor)
        Predicted, PredictedOffset = CD.PredictedDeskPlane(HeadCamera)
        with SC._Lock:
            Recorded = list(SC._Session["desk_planes"])
        Self.assertTrue(all(CD.AngleDeg(Plane["n"], Predicted) > 80 for Plane in Recorded), "the wall should win unaided")
        Report = SC.Solve(TableZ=TableZ, Save=False)
        Self.assertTrue(Report["ok"] and Report["quality_ok"], Report.get("problems"))
        Model = Report["model"]
        Self.assertEqual(Model["fit_desk_planes_redetected"], {"recorded": len(Rows), "kept": 0, "found_again": len(Rows), "dropped": 0})
        Self.assertEqual(Model["fit_camera_motion"]["method"], "desk_map")
        Self.assertFalse(Model["fit_camera_motion"]["moved"])
        # The baseline is the desk patch, not the wall: near the table's plane and its distance.
        Reference = Model["drift_reference"]
        Self.assertLess(CD.AngleDeg(Reference["desk_n"], Predicted), 5.0, Reference["desk_n"])
        Self.assertLess(abs(Reference["desk_d"] - PredictedOffset), 2.0)
        # /locate's check: the same scene passes; the camera tipped 2 deg is blocked as moved.
        Self.assertTrue(CD.CheckDesk(RenderDepth(HeadCamera, Scene, FloorZ=Floor), Intrinsics, Model)["ok"])
        Moved = CD.CheckDesk(RenderDepth(Turned(HeadCamera, 2.0), Scene, FloorZ=Floor), Intrinsics, Model)
        Self.assertFalse(Moved["ok"])
        Self.assertEqual(Moved["error_code"], "camera_moved", Moved)
        # The same scene while the camera creeps 0.3 deg through the run is still flagged.
        Self.Inject(Rows, Desk=HeadCamera, Scene=Scene, FloorZ=Floor, DeskTiltDeg=0.3)
        Report = SC.Solve(TableZ=TableZ, Save=False)
        Self.AssertKeptWithWarning(Report, "the camera moved 0.2")

    def TestSingleGripperTiltIsAWarning(Self):
        # Same tips and yaw as a good set, but the distal link always at -12 deg: the camera
        # height and the tape offset's vertical part become interchangeable, so no fit is trusted.
        Tape, Truth, Rng = np.array([1.2, -0.6, 0.8]), HeadCamera, np.random.default_rng(5)
        Rows = []
        for Tip, Frame, _, _ in CalibrationPoses(Truth, Tape, Count=40):
            Yaw = Frame[:, 0].copy(); Yaw[2] = 0.0
            if np.linalg.norm(Yaw) < 1e-6:
                continue
            Yaw /= np.linalg.norm(Yaw)
            X = np.append(Yaw[:2] * math.cos(math.radians(-12)), math.sin(math.radians(-12)))
            Y = np.cross([0.0, 0.0, 1.0], Yaw); Y /= np.linalg.norm(Y)
            Level = np.column_stack([X, Y, np.cross(X, Y)])
            Camera = DG.DisplayToCamera(Tip + Level @ Tape, Truth)[0]
            Pixel = DG.Project(Camera, Intrinsics)[0]
            if Camera[2] > 5 and 40 < Pixel[0] < Width - 40 and 40 < Pixel[1] < Height - 40:
                Rows.append((Tip, Level, Pixel + Rng.normal(0, 0.5, 2), Camera + Rng.normal(0, 0.2, 3)))
        Self.assertGreaterEqual(len(Rows), 15)
        Self.Inject(Rows[:22])
        Report = SC.Solve(TableZ=TableZ, Save=False)
        Self.AssertKeptWithWarning(Report, "gripper tilt barely varied")

    def TestWrongDepthScaleIsAWarning(Self):
        Self.Inject(CalibrationPoses(FrontCamera, np.array([1.0, 0.0, 0.5])), DepthScale=1.08)
        Report = SC.Solve(TableZ=TableZ, Save=False)
        Self.assertTrue(Report["ok"] and not Report["quality_ok"], Report.get("error"))
        Self.assertTrue(any(Name in Problem for Problem in Report["problems"] for Name in ("depth disagreement", "fit error", "hold-out error")),
                        Report["problems"])

    def TestWithoutDepthUsesColourFit(Self):
        Self.Inject(CalibrationPoses(FrontCamera, np.array([1.0, 0.0, 0.5])), WithDepth=False)
        Report = SC.Solve(TableZ=TableZ, Save=False)
        Self.assertNotEqual(Report.get("method"), "depth")


class FakeSource:
    Opened = 0

    def __init__(Self, FailAfter=None):
        FakeSource.Opened += 1
        Self.Info = {"name": "Fake Gemini", "connection": "USB2.1"}
        Self.Intrinsics = dict(Intrinsics)
        Self.Count = 0
        Self.FailAfter = FailAfter

    def Read(Self, TimeoutMs=500):
        Self.Count += 1
        if Self.FailAfter is not None and Self.Count > Self.FailAfter:
            raise RuntimeError("unplugged")
        time.sleep(0.004)
        Image = np.full((Height, Width, 3), Self.Count % 255, np.uint8)
        return Image, np.full((Height, Width), 500 + Self.Count, np.uint16)

    def Close(Self):
        pass


def WaitFor(Condition, TimeoutS=5.0):
    Start = time.time()
    while time.time() - Start < TimeoutS:
        if Condition():
            return True
        time.sleep(0.01)
    return False


class ServiceTests(unittest.TestCase):
    """The camera service on FakeSource frames. SO101_DEPTH_CAMERA=0, the documented way to keep the real camera closed,
    makes DepthCamera.Start a no-op, and three of these failed under it (audit 2 L7) though they never use the real
    camera: it is lifted for them only, and the Orbbec SDK cannot even be imported while they run."""

    def setUp(Self):
        for Item in (patch.dict(os.environ, {"SO101_DEPTH_CAMERA": "1"}), patch.dict(sys.modules, {"pyorbbecsdk": None})):
            Item.start()
            Self.addCleanup(Item.stop)

    def TestStreamsPinsAndReportsUsb2(Self):
        Camera = OC.DepthCamera(Opener=FakeSource)
        Camera.Start()
        try:
            Self.assertTrue(WaitFor(lambda: Camera.Status()["seq"] >= 10))
            Status = Camera.Status()
            Self.assertTrue(Status["available"])
            Self.assertGreater(Status["fps"], 1)
            Self.assertIn("USB 3", Status["warnings"][0])
            Seq, Jpeg = Camera.WaitJpeg(0)
            Self.assertGreater(len(Jpeg), 100)
            Newer = Camera.WaitJpeg(Seq)
            Self.assertGreater(Newer[0], Seq)
            Pin = Camera.Depth(Seq)
            Self.assertEqual(Pin["seq"], Seq)
            # Median of the frames ending at Seq: counts Seq-2..Seq give Seq-1.
            Self.assertEqual(int(Pin["depth_mm"][0, 0]), 500 + Seq - 1)
            Self.assertIs(Camera.Depth(Seq), Pin)
        finally:
            Camera.Stop()

    def TestPinOutlivesHistory(Self):
        Camera = OC.DepthCamera(Opener=FakeSource, History=5)
        Image = np.zeros((4, 4, 3), np.uint8)
        for Value in range(1, 4):
            Camera.Push(Image, np.full((4, 4), Value * 100, np.uint16))
        Pin = Camera.Depth(3)
        for Value in range(10):
            Camera.Push(Image, np.full((4, 4), 7, np.uint16))
        Self.assertIsNone(Camera.Jpeg(3))
        Self.assertIs(Camera.Depth(3), Pin)
        Self.assertEqual(int(Pin["depth_mm"][0, 0]), 200)
        Self.assertIsNone(Camera.Depth(2))  # gone before anyone asked for it

    def TestReconnectsAfterUnplug(Self):
        FakeSource.Opened = 0
        Camera = OC.DepthCamera(Opener=lambda: FakeSource(FailAfter=5))
        with patch.object(OC, "ReconnectDelayS", 0.05):
            Camera.Start()
            try:
                Self.assertTrue(WaitFor(lambda: FakeSource.Opened >= 2))
                Self.assertTrue(WaitFor(lambda: Camera.Error is not None and "unplugged" in Camera.Error))
            finally:
                Camera.Stop()

    def TestMissingCameraIsReportedNotRaised(Self):
        def Missing():
            raise RuntimeError("no Orbbec camera connected")
        Camera = OC.DepthCamera(Opener=Missing)
        Camera.Start()
        try:
            Self.assertTrue(WaitFor(lambda: Camera.Error is not None))
            Status = Camera.Status()
            Self.assertFalse(Status["available"])
            Self.assertIn("no Orbbec", Status["error"])
            Self.assertIsNone(Camera.Depth())
        finally:
            Camera.Stop()

    def TestMissingCameraBacksOff(Self):
        """Each failed open holds the interpreter lock for 1-2 s on the real SDK (2026-10-10: the servo loop froze for
        1-2 s of every 4 with the camera unplugged), so failed opens wait longer each time, up to ReconnectMaxDelayS."""
        Times = []
        def Missing():
            Times.append(time.monotonic())
            raise RuntimeError("no Orbbec camera connected")
        Camera = OC.DepthCamera(Opener=Missing)
        with patch.object(OC, "ReconnectDelayS", 0.05), patch.object(OC, "ReconnectMaxDelayS", 0.2):
            Camera.Start()
            try:
                Self.assertTrue(WaitFor(lambda: len(Times) >= 5, 5.0))
            finally:
                Camera.Stop()
        Gaps = [B - A for A, B in zip(Times, Times[1:])]
        Self.assertLess(Gaps[0], 0.09)                      # 0.05 s first
        Self.assertGreater(Gaps[1], 0.08)                   # then 0.1 s
        Self.assertGreater(Gaps[3], 0.17)                   # then capped at 0.2 s
        Self.assertLess(max(Gaps), 0.35)

    def TestNoOpenWhileQuiet(Self):
        """While the arm needs the control loop (Quiet), the camera is not opened; it opens once Quiet clears."""
        Opens = []
        def Opener():
            Opens.append(time.monotonic())
            return FakeSource()
        Quiet = {"on": True}
        Camera = OC.DepthCamera(Opener=Opener)
        Camera.Quiet = lambda: Quiet["on"]
        with patch.object(OC, "QuietPollS", 0.02):
            Camera.Start()
            try:
                time.sleep(0.3)
                Self.assertEqual(Opens, [], "no open attempt while quiet")
                Quiet["on"] = False
                Self.assertTrue(WaitFor(lambda: Camera.Status()["seq"] >= 3))
                Self.assertEqual(Camera.FailedOpens, 0)
            finally:
                Camera.Stop()

    def TestQuietThatRaisesDoesNotStopTheCamera(Self):
        Camera = OC.DepthCamera(Opener=FakeSource)
        def Broken():
            raise RuntimeError("controller gone")
        Camera.Quiet = Broken
        Camera.Start()
        try:
            Self.assertTrue(WaitFor(lambda: Camera.Status()["seq"] >= 3))
        finally:
            Camera.Stop()

    def TestMedianIgnoresMissingDepthAndColorizeMirrors(Self):
        Frames = [np.array([[0, 100]], np.uint16), np.array([[100, 300]], np.uint16), np.array([[200, 0]], np.uint16)]
        Self.assertEqual(OC.MedianDepth(Frames).tolist(), [[150, 200]])
        Depth = np.zeros((10, 20), np.uint16)
        Depth[:, :5] = 600
        Depth[:, 5:] = 1200
        Png, Stats = OC.ColorizeDepth(Depth, True)
        Image = cv2.imdecode(np.frombuffer(Png, np.uint8), cv2.IMREAD_COLOR)
        Self.assertEqual(Image.shape[:2], (10, 20))
        Plain = cv2.imdecode(np.frombuffer(OC.ColorizeDepth(Depth, False)[0], np.uint8), cv2.IMREAD_COLOR)
        Self.assertTrue(np.array_equal(Image, cv2.flip(Plain, 1)))
        Self.assertEqual(Stats["median_cm"], 120.0)


def Detected(Label, Box):
    """A VLM detection of a normalized capture box, as _detect_once returns it."""
    return {"label": Label, "box": tuple(Box), "u": (Box[0] + Box[2]) / 2 * Width, "v": Box[3] * Height}


def TapeScene():
    """Native frame with the red tip tape on a white gripper, and its depth (tape at 55 cm)."""
    Image = np.full((Height, Width, 3), 90, np.uint8)
    cv2.rectangle(Image, (560, 300), (680, 420), (235, 235, 235), -1)
    cv2.rectangle(Image, (612, 352), (632, 372), (30, 30, 215), -1)
    Depth = np.full((Height, Width), 900, np.uint16)
    Depth[300:421, 560:681] = 550
    return Image, Depth


class BridgeTests(unittest.TestCase):
    def setUp(Self):
        Self.Camera = OC.DepthCamera(Opener=FakeSource)
        Self.Camera.IntrinsicsValue = dict(Intrinsics)
        Self.Patches = [patch.object(Bridge, "depth_camera", Self.Camera),
                        patch.object(SC, "SessionPath", Artifacts / "DepthBridgeSession.json"),
                        patch.object(SC, "CapturesDir", Artifacts / "DepthBridgeCaptures")]
        for Patch in Self.Patches:
            Patch.start()
        SC.ResetSession()
        SC._RedTracker.clear()

    def tearDown(Self):
        for Patch in Self.Patches:
            Patch.stop()
        SC._Session.clear()
        SC._Session.update({"image_size": None, "mirrored": None, "observations": []})
        SC._RedTracker.clear()

    def TestFrameDepthAndPointEndpoints(Self):
        Scene = RenderDepth(FrontCamera, [])
        Seq = Self.Camera.Push(np.zeros((Height, Width, 3), np.uint8), Scene)
        Frame = Bridge.depthcam_frame(after=0)
        Self.assertEqual(Frame.headers["X-Frame-Seq"], str(Seq))
        Self.assertEqual(Frame.media_type, "image/jpeg")
        Self.assertEqual(Bridge.depthcam_frame(after=Seq).status_code, 204)
        # A client still holding a previous bridge process's (larger) sequence resyncs at once, not after the 1.5 s
        # long-poll. The camera counts as running here (no thread, no device): a stopped one never waits at all, which
        # made this check pass whatever the code did. 1.0 s tells the two apart even on a busy PC.
        Self.Camera.Running = True
        try:
            Started = time.perf_counter()
            Stale = Bridge.depthcam_frame(after=Seq + 100000)
            Took = time.perf_counter() - Started
        finally:
            Self.Camera.Running = False
        Self.assertEqual(Stale.headers["X-Frame-Seq"], str(Seq))
        Self.assertLess(Took, 1.0)
        Self.assertEqual(Bridge.depthcam_overlay(after=Seq + 100000).headers["X-Frame-Seq"], str(Seq))
        Depth = Bridge.depthcam_depth({"seq": Seq, "mirrored": True})
        Self.assertTrue(Depth["ok"])
        Self.assertEqual((Depth["width"], Depth["height"]), (Width, Height))
        Self.assertTrue(Depth["depth_image"].startswith("data:image/png;base64,"))
        Point = np.array([-15.0, 28.0, TableZ])
        U, V = DG.Project(DG.DisplayToCamera(Point, FrontCamera), Intrinsics)[0]
        with patch.object(calibration, "camera_model", return_value=FrontCamera):
            Result = Bridge.depthcam_point({"seq": Seq, "u": Width - (U + 0.5), "v": V + 0.5, "mirrored": True})
            Status = Bridge.depthcam_status()
            Check = Bridge.depthcam_check()
        Self.assertTrue(Result["ok"])
        Self.assertLess(math.dist([Result["display_cm"][Axis] for Axis in "xyz"], Point), 0.3)
        Self.assertTrue(Status["calibrated"])
        Self.assertLess(abs(Check["height_error_cm"]), 0.1)
        Self.assertFalse(Bridge.depthcam_depth({"seq": Seq + 50})["ok"])

    def Show(Self, Depth):
        """Stream a still scene: its pin is the median of the last frames, all of this scene."""
        for _ in range(OC.PinMedianFrames):
            Seq = Self.Camera.Push(np.zeros((Height, Width, 3), np.uint8), Depth)
        return Seq

    def TestDepthCheckReportsTheGateLocateRuns(Self):
        # GET /depthcam/check: the camera_check /locate would return for the newest frame, beside
        # the dominant-plane fields the tools read.
        Model = Verified(FrontCamera)
        Still = Self.Show(RenderDepth(FrontCamera, []))
        with patch.object(calibration, "camera_model", return_value=Model):
            Check = Bridge.depthcam_check()
        Self.assertLess(abs(Check["height_error_cm"]), 0.1)
        Gate = Check["camera_check"]
        Self.assertTrue(Gate["ok"], Gate)
        Self.assertEqual((Check["seq"], Gate["seq"], Gate["reference_fitted_at"]), (Still, Still, "2026-09-28T15:04:00"))
        Self.assertLess(Gate["angle_deg"], 0.05)
        Self.assertEqual(Gate["tape_check"], "unavailable")  # this model has no tape offset
        Moved = Self.Show(RenderDepth(Turned(FrontCamera, 2.0), []))
        with patch.object(calibration, "camera_model", return_value=Model):
            Blocked = Bridge.depthcam_check()["camera_check"]
        Self.assertEqual((Blocked["ok"], Blocked["error_code"], Blocked["seq"]), (False, "camera_moved", Moved))
        Self.assertAlmostEqual(Blocked["angle_deg"], 2.0, delta=0.1)
        with patch.object(calibration, "camera_model", return_value=FrontCamera):
            Unverified = Bridge.depthcam_check()
        Self.assertIn("height_error_cm", Unverified)
        Self.assertEqual((Unverified["camera_check"]["ok"], Unverified["camera_check"]["error_code"]), (False, "camera_not_verified"))

    def Locate(Self, Model, Payloads, Detection):
        """Bridge /locate for each payload with Model saved and the VLM mocked; (results, pick mock, detect mock)."""
        Pick, Detect = MagicMock(return_value="test-model"), MagicMock(return_value=Detection)
        with patch.object(calibration, "camera_model", return_value=Model), \
                patch.object(calibration, "pixel_to_plane",
                             side_effect=lambda U, V, Mirrored=None: SC.PixelToPlane(Model, U, V, Mirrored)), \
                patch.object(calibration, "status", return_value={"calibrated": True, "rms_cm": 0.3}), \
                patch.object(object_locator, "_pick_model", Pick), \
                patch.object(object_locator, "_detect_once", Detect), \
                patch.object(object_locator, "_refine_detection", return_value=None):
            return [Bridge.locate(Payload) for Payload in Payloads], Pick, Detect

    def TestLocateMeasuresInThreeDimensions(Self):
        Low, High = np.array([-18.0, 25.0, TableZ]), np.array([-12.0, 31.0, TableZ + 8.0])
        Seq = Self.Camera.Push(np.zeros((Height, Width, 3), np.uint8), RenderDepth(FrontCamera, [(Low, High)]))
        Image = Encode(np.zeros((Height, Width, 3), np.uint8))
        (Result, Expired, Webcam), _, _ = Self.Locate(Verified(FrontCamera), [
            {"image": Image, "depth_seq": Seq, "mirrored": True},
            # A depth-camera capture whose depth frame has expired: the camera cannot be checked.
            {"image": Image, "depth_seq": Seq + 1000, "mirrored": True},
            # A webcam capture against the depth camera's calibration.
            {"image": Image, "mirrored": True}], [Detected("block", CaptureBox(FrontCamera, Low, High))])
        with patch.object(calibration, "camera_model", return_value={**FrontCamera, "type": "pinhole_selfcal_v1"}):
            DepthOnWebcamModel = Bridge.locate({"image": Image, "depth_seq": Seq, "mirrored": True})
        Self.assertTrue(Result["ok"], Result)
        Self.assertTrue(Result["depth"])
        Self.assertTrue(Result["camera_check"]["ok"])
        Self.assertEqual(Result["camera_check"]["seq"], Seq)
        Self.assertEqual(Result["withheld"], [])
        Object = Result["objects"][0]
        Self.assertEqual(Object["method"], "depth")
        Self.assertLess(math.hypot(Object["x"] + 15.0, Object["y"] - 28.0), 0.6, Object)
        Self.assertAlmostEqual(Object["height_cm"], 8.0, delta=0.3)
        Self.assertEqual((Object["support_measured"], Object["support_branch"]), (True, "desk"))
        Self.assertAlmostEqual(Object["support_z_measured"], TableZ, delta=0.1)
        Self.assertGreater(Object["valid_fraction"], 0.9)
        Self.assertEqual((Expired["ok"], Expired["error_code"]), (False, "camera_check_failed"))
        for Refused in (Webcam, DepthOnWebcamModel):
            Self.assertFalse(Refused["ok"])
            Self.assertIn("other camera", Refused["error"])

    def TestLocateChecksTheCameraBeforeAnyModelCall(Self):
        Image = Encode(np.zeros((Height, Width, 3), np.uint8))
        Moved = Self.Show(RenderDepth(Turned(FrontCamera, 2.0), []))
        Still = Self.Show(RenderDepth(FrontCamera, []))
        Box = Detected("block", [0.4, 0.4, 0.6, 0.6])
        (Blocked,), Pick, Detect = Self.Locate(Verified(FrontCamera), [{"image": Image, "depth_seq": Moved, "mirrored": True}], [Box])
        (Unverified,), PickAgain, DetectAgain = Self.Locate(FrontCamera, [{"image": Image, "depth_seq": Still, "mirrored": True}], [Box])
        (Uncalibrated,), PickNone, DetectNone = Self.Locate(None, [{"image": Image, "depth_seq": Still, "mirrored": True}], [Box])
        for Mock in (Pick, Detect, PickAgain, DetectAgain, PickNone, DetectNone):
            Mock.assert_not_called()
        Self.assertEqual(Uncalibrated["error_code"], "camera_not_verified")
        Self.assertIn("not calibrated", Uncalibrated["error"])
        Self.assertEqual((Blocked["ok"], Blocked["error_code"]), (False, "camera_moved"))
        Check = Blocked["camera_check"]
        Self.assertFalse(Check["ok"])
        Self.assertAlmostEqual(Check["angle_deg"], 2.0, delta=0.1)
        Self.assertIsNotNone(Check["offset_cm"])
        Self.assertEqual((Check["seq"], Check["reference_fitted_at"]), (Moved, "2026-09-28T15:04:00"))
        Self.assertIn("desk plane changed", Check["reason"])
        Self.assertIn("Lock the camera head", Blocked["error"])
        Self.assertEqual((Unverified["error_code"], Unverified["camera_check"]["reference_fitted_at"]), ("camera_not_verified", None))
        Self.assertIn("Automatic calibration", Unverified["error"])

    def TestLocateWithholdsTheRobotAndGarbage(Self):
        Block = (np.array([-18.0, 25.0, TableZ]), np.array([-12.0, 31.0, TableZ + 8.0]))
        Binder = (np.array([-8.0, 34.0, TableZ]), np.array([-2.0, 40.0, TableZ + 4.0]))
        Depth = RenderDepth(FrontCamera, [Block, Binder])
        Far = CaptureBox(FrontCamera, np.array([-30.0, 45.0, TableZ]), np.array([-24.0, 50.0, TableZ + 5.0]))
        U0, V0, U1, V1 = DG.BoxToNative(Far, Width, Height, True)
        Depth[V0:V1, U0:U1] = 0  # the 19:06 "white box": 1% depth, the rest far background
        Depth[V0:V1:10, U0:U1:10] = 5500
        Seq = Self.Camera.Push(np.zeros((Height, Width, 3), np.uint8), Depth)
        Detection = [Detected("white robotic arm", CaptureBox(FrontCamera, *Block)), Detected("binder", CaptureBox(FrontCamera, *Binder)),
                     Detected("white box", Far), Detected("black bin", CaptureBox(FrontCamera, *Block))]
        (Result,), _, _ = Self.Locate(Verified(FrontCamera), [{"image": Encode(np.zeros((Height, Width, 3), np.uint8)),
                                                              "depth_seq": Seq, "mirrored": True}], Detection)
        Self.assertTrue(Result["ok"], Result)
        Self.assertEqual([Object["label"] for Object in Result["objects"]], ["binder"])
        Self.assertAlmostEqual(Result["objects"][0]["height_cm"], 4.0, delta=0.3)
        Reasons = {Item["label"]: Item["reason"] for Item in Result["withheld"]}
        Self.assertIn("robot's own parts", Reasons["white robotic arm"])
        Self.assertIn("robot's own bin", Reasons["black bin"])
        Self.assertIn("not measurable", Reasons["white box"])

    def TestSceneTripwireBlocksWhenDeskObjectsDisagreeWithTheDesk(Self):
        # The desk plane matches, yet the surfaces around desk objects do not: at 19:00:57
        # seven desk objects sat a shared 4.6 cm low.
        Image = Encode(np.zeros((Height, Width, 3), np.uint8))
        Model = Verified(FrontCamera)
        Pad = (np.array([-24.0, 28.0, TableZ]), np.array([-6.0, 44.0, TableZ + 1.2]))
        Cup = (np.array([-17.0, 34.0, TableZ + 1.2]), np.array([-13.0, 38.0, TableZ + 10.0]))
        Single = Self.Show(RenderDepth(FrontCamera, [Pad, Cup]))
        # The same cup, with the camera turned 0.3 deg: under the desk limit, over half of it.
        SingleNearLimit = Self.Show(RenderDepth(Turned(FrontCamera, 0.3), [Pad, Cup]))
        Platforms = [(np.array([X - 5.0, 31.0, TableZ]), np.array([X + 5.0, 41.0, TableZ + 2.5])) for X in (-30.0, -15.0, 0.0)]
        Items = [(np.array([X - 1.5, 34.5, TableZ + 2.5]), np.array([X + 1.5, 37.5, TableZ + 6.0])) for X in (-30.0, -15.0, 0.0)]
        Shared = Self.Show(RenderDepth(FrontCamera, Platforms + Items))
        Cups = [Detected("cup", CaptureBox(FrontCamera, *Cup))]
        (OnPad,), _, _ = Self.Locate(Model, [{"image": Image, "depth_seq": Single, "mirrored": True}], Cups)
        (OnPadNearLimit,), _, _ = Self.Locate(Model, [{"image": Image, "depth_seq": SingleNearLimit, "mirrored": True}], Cups)
        (Three,), _, _ = Self.Locate(Model, [{"image": Image, "depth_seq": Shared, "mirrored": True}],
                                     [Detected(f"item {Index}", CaptureBox(FrontCamera, *Item)) for Index, Item in enumerate(Items)])
        # One cup on a 1.2 cm pad under a still camera is a cup on a pad: measured on the pad, not a
        # moved camera (recalibrating could never clear it).
        Self.assertTrue(OnPad["ok"], OnPad)
        [Measured] = OnPad["objects"]
        Self.assertTrue(Measured["support_measured"])
        Self.assertAlmostEqual(Measured["z"], TableZ + 1.2, delta=0.2)
        Self.assertAlmostEqual(Measured["height_cm"], 8.8, delta=0.4)
        # These three separately observed platforms are now valid automatic
        # supports after the camera gate. They no longer impersonate desk drift.
        Self.assertTrue(Three["ok"], Three)
        Self.assertEqual(len(Three["objects"]), 3)
        Self.assertTrue(all(Object["support_geometry"]["Source"] == "depth" for Object in Three["objects"]))
        for Result, Words in ((OnPadNearLimit, "stands on a desk surface +1."),):
            Self.assertEqual((Result["ok"], Result["error_code"]), (False, "camera_moved"), Result)
            Self.assertIn(Words, Result["camera_check"]["reason"])
            Self.assertNotIn("objects", Result)
        Self.assertIn("If it is not standing on something", OnPadNearLimit["error"])
        Self.assertGreaterEqual(OnPadNearLimit["camera_check"]["angle_deg"], 0.25)

    def TestObservationRecordsTapeDepth(Self):
        Image, Depth = TapeScene()
        Seqs = [Self.Camera.Push(Image, Depth) for _ in range(3)]
        Capture = Encode(cv2.flip(Image, 1))  # the browser sends mirrored frames
        Check = Bridge.selfcal_check({"images": [Capture] * 3, "mirrored": True, "depth_seqs": Seqs,
                                      "hint": {"u": 622 / Width, "v": 362 / Height}, "initialize": True})
        Self.assertTrue(Check["detected"], Check.get("reason"))
        Self.assertAlmostEqual(Check["tape_distance_cm"], 55.0 * math.sqrt(1 + ((622 - 640.4) / 609.7) ** 2), delta=0.3)
        Joints = [{"x": 0.0, "y": 0.0, "z": 0.0}, {"x": 0.0, "y": 0.0, "z": 10.0}, {"x": 0.0, "y": 10.0, "z": 20.0},
                  {"x": 0.0, "y": 20.0, "z": 20.0}]
        Result = Bridge.selfcal_observe({"images": [Capture] * 3, "joints": Joints, "mirrored": True, "pose_index": 0,
                                         "depth_seqs": Seqs})
        Self.assertTrue(Result["ok"], Result)
        Self.assertTrue(Result["detected"], Result)
        Candidate = SC._Session["red_poses"][-1]["candidates"][0]
        Self.assertAlmostEqual(Candidate["camera_cm"][2], 55.0, delta=0.2)
        Self.assertEqual(SC._Session["depth_intrinsics"]["fx"], Intrinsics["fx"])
        Self.assertAlmostEqual(SC._Session["observations"][-1]["camera_cm"][2], 55.0, delta=0.2)
        Archive = Path(Result["capture_archive"])
        Saved = cv2.imread(str(Archive.with_name(f"{Archive.stem}_depth0.png")), cv2.IMREAD_UNCHANGED)
        Self.assertEqual(int(Saved[360, 620]), 550)

    def TestDepthIgnoredWithoutCamera(Self):
        Self.Camera.IntrinsicsValue = None
        Self.assertEqual(Bridge._depth_frames([1, 2]), (None, None))
        Self.assertEqual(Bridge._depth_frames([True]), (None, None))


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestDepthCamera` runs the Test* methods too (unittest names this hook)."""
    Prefixed = unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return unittest.TestSuite(Prefixed.loadTestsFromTestCase(Case) for Case in
                              (GeometryTests, DriftTests, SupportTests, RealDataTests, DepthCalibrationTests, ServiceTests,
                               BridgeTests))


if __name__ == "__main__":
    Loader = unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Suite = unittest.TestSuite(Loader.loadTestsFromTestCase(Case) for Case in
                               (GeometryTests, DriftTests, SupportTests, RealDataTests, DepthCalibrationTests, ServiceTests,
                                BridgeTests))
    Result = unittest.TextTestRunner(verbosity=2).run(Suite)
    raise SystemExit(not Result.wasSuccessful())
