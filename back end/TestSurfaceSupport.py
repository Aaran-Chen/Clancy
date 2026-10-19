"""Objects on surfaces besides the desk: a declared platform or low shelf is measured, an undeclared one withheld.

Scenes are TestDepthCamera's ray-cast workspace (the desk with its front edge at Y 25.5 and the bin, on
the room floor) seen by the head camera, with a box 15 cm tall on the desk (a platform), or a surface
5.7 cm under the desk line beside it (a low shelf), declared in DeclaredSurfaces' format or not. The
desk's own results are TestDepthCamera's, TestObjectLocator's and TestCartMove's, unchanged.

Run:  python TestSurfaceSupport.py
"""
import os
os.environ.setdefault("SO101_LOCATE_MODE", "vlm")  # this suite tests the voted multi-call locate; TestSingleCallLocate.py the default
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

import numpy as np

os.environ["SO101_CAPTURES_ROOT"] = str(Path(__file__).resolve().parent / "TestArtifacts" / "SessionCaptures")
os.environ["SO101_DECLARED_SURFACES"] = str(Path(__file__).resolve().parent / "TestArtifacts" / "NoDeclaredSurfaces.json")

import TestDepthCamera as TDC  # noqa: E402  (the ray-caster, the camera models)
import calibration  # noqa: E402
import CameraDrift as CD  # noqa: E402
import DeclaredSurfaces  # noqa: E402
import DepthGeometry as DG  # noqa: E402
import object_locator  # noqa: E402
import SelfCalibration as SC  # noqa: E402

TableZ, Height, Width = TDC.TableZ, TDC.Height, TDC.Width
Camera = TDC.HeadCamera
PlatformTop = TableZ + 15.0
# A box 15 cm tall on the desk, and a can (6.6 cm across, 12 tall) on it.
Platform = (np.array([-25.0, 30.0, TableZ]), np.array([-3.0, 44.0, PlatformTop]))
Can = (np.array([-17.0, 34.0, PlatformTop]), np.array([-10.4, 40.6, PlatformTop + 12.0]))
Declared = {"Name": "box platform", "Kind": "platform", "TopZCm": PlatformTop, "XRangeCm": [-25.0, -3.0], "YRangeCm": [30.0, 44.0],
            "BoardAboveZCm": None, "Frame": "cart"}
# A shelf 5.7 cm under the desk line set into the desk (a gap from X -5 to 29, the desk on both sides),
# and three cans in its middle.
ShelfTop = TableZ - 5.7
Shelf = (np.array([-4.0, 28.0, TDC.RoomFloorZ]), np.array([28.0, 56.0, ShelfTop]))
ShelfSpec = {**Declared, "Name": "low shelf", "TopZCm": ShelfTop, "XRangeCm": [-4.0, 28.0], "YRangeCm": [28.0, 56.0]}
ShelfCans = [(np.array([X, 39.0, ShelfTop]), np.array([X + 4.0, 43.0, ShelfTop + 7.0])) for X in (5.0, 12.0, 19.0)]
# A can on that shelf by the desk's end: the desk around it.
EdgeCan = (np.array([-3.5, 39.0, ShelfTop]), np.array([0.5, 43.0, ShelfTop + 7.0]))
DeskEraser = (np.array([-14.0, 34.0, TableZ]), np.array([-9.0, 36.0, TableZ + 1.1]))


def ShelfWorkspace(Items=()):
    """The workspace with the low shelf set into the desk: the desk left of X -5 and right of X 29."""
    Desk, Bin = TDC.Workspace()[:2]
    Left = (Desk[0], np.array([-5.0, Desk[1][1], Desk[1][2]]))
    Right = (np.array([29.0, Desk[0][1], Desk[0][2]]), Desk[1])
    Back = (np.array([-5.0, 57.0, Desk[0][2]]), np.array([29.0, Desk[1][1], Desk[1][2]]))
    Front = (np.array([-5.0, Desk[0][1], Desk[0][2]]), np.array([29.0, 27.0, Desk[1][2]]))
    return [Left, Right, Back, Front, Bin, Shelf, *Items]


def Valid(*Specs):
    Surfaces, Problems = DeclaredSurfaces.Validate({"Surfaces": list(Specs)})
    assert not Problems, Problems
    return Surfaces


class SurfaceMeasureTests(unittest.TestCase):
    @classmethod
    def setUpClass(Class):
        Class.Model = TDC.Verified(Camera, TDC.Workspace(), FloorZ=TDC.RoomFloorZ)
        Class.Desk = CD.DisplayDeskPlane(Class.Model)
        Class.Depth = TDC.RenderDepth(Camera, TDC.Workspace([Platform, Can]), FloorZ=TDC.RoomFloorZ)

    def Measure(Self, Depth, Box, *Specs):
        return DG.MeasureBox(Depth, Box, TDC.Intrinsics, Self.Model, TableZ, True,
                             Surfaces=DG.WorkspaceSurfaces(Self.Desk, TableZ, Declared=Valid(*Specs)))

    def TestACanOnADeclaredPlatformIsMeasuredOnIt(Self):
        Box = TDC.CaptureBox(Camera, *Can)
        Padded = list(Box)
        Padded[3] += 25.0 / Height  # the box reaches 25 px past the can's base, toward the platform's front
        for Name, Drawn in (("tight", Box), ("padded", Padded)):
            with Self.subTest(Name):
                Result = Self.Measure(Self.Depth, Drawn, Declared)
                Self.assertTrue(Result["ok"], Result)
                Self.assertEqual((Result["support_branch"], Result["support_surface"], Result["support_kind"]), ("surface", "box platform", "platform"))
                Self.assertTrue(Result["support_measured"])
                Self.assertAlmostEqual(Result["z"], PlatformTop, delta=0.3)
                Self.assertAlmostEqual(Result["support_z_measured"], PlatformTop, delta=0.3)
                Self.assertAlmostEqual(Result["height_cm"], 12.0, delta=0.4)
                Self.assertLess(np.hypot(Result["x"] + 13.7, Result["y"] - 37.3), 0.8, Result)
                Self.assertEqual((Result["declared_surface"], Result["desk_area"]), ("box platform", False))

    def TestWithoutTheDeclarationItIsNeverMeasuredOnThePlatform(Self):
        # As before: with the desk behind the box in view, the box and the can measure as one object on the desk,
        # 27 cm tall (the locate withholds it: over 25 cm; on a 10 cm box, 22 cm: the planner finds it too wide).
        Result = Self.Measure(Self.Depth, TDC.CaptureBox(Camera, *Can))
        Self.assertEqual((Result["ok"], Result["support_branch"]), (True, "desk"), Result)
        Self.assertAlmostEqual(Result["height_cm"], 27.0, delta=0.5)
        # On a wide box (no desk around the can) it is withheld as before, with the box's height as a hint.
        Wide = (np.array([-40.0, 28.0, TableZ]), np.array([12.0, 62.0, PlatformTop]))
        Depth = TDC.RenderDepth(Camera, TDC.Workspace([Wide, Can]), FloorZ=TDC.RoomFloorZ)
        Result = Self.Measure(Depth, TDC.CaptureBox(Camera, *Can))
        Self.assertFalse(Result["ok"], Result)
        Self.assertEqual(Result["error"], "the whole box stands above the support: the support is not where the calibration expects it")
        Self.assertAlmostEqual(Result["undeclared_support_z"], PlatformTop, delta=0.3)
        Self.assertNotEqual(Result["support_branch"], "surface")

    def TestTheDeclaredHeightMayBeAFewMillimetresOffButNotCentimetres(Self):
        Box = TDC.CaptureBox(Camera, *Can)
        Near = Self.Measure(Self.Depth, Box, {**Declared, "TopZCm": PlatformTop + 1.2})
        Self.assertTrue(Near["ok"], Near)
        Self.assertAlmostEqual(Near["z"], PlatformTop, delta=0.3, msg="the camera's height, not the hand-measured one")
        for Off in (3.0, -3.0):
            with Self.subTest(off=Off):
                Far = Self.Measure(Self.Depth, Box, {**Declared, "TopZCm": PlatformTop + Off})
                Self.assertFalse(Far["ok"], Far)
                Self.assertIn("declared platform 'box platform'", Far["error"])
                Self.assertNotEqual(Far["support_branch"], "desk")

    def TestADeclaredExtentMustCoverTheObject(Self):
        # Declared 6 cm too far left, the can is not over it and is measured as without one; an object that meets
        # the surface outside its extent is not on it.
        Box = TDC.CaptureBox(Camera, *Can)
        Missed = Self.Measure(Self.Depth, Box, {**Declared, "XRangeCm": [-31.0, -15.5]})
        Self.assertEqual(Missed, Self.Measure(Self.Depth, Box))
        [Surface] = Valid(Declared)
        Self.assertIsNone(DG._ContactMismatch("surface", -4.0, 43.0, Surface))
        Self.assertIn("outside the declared platform 'box platform' (Z 12.7), which covers X -25..-3, Y 30..44",
                      DG._ContactMismatch("surface", -0.5, 43.0, Surface))

    def TestDeskObjectsBesideADeclaredPlatformAreMeasuredAsBefore(Self):
        Eraser = (np.array([5.0, 34.0, TableZ]), np.array([10.0, 36.0, TableZ + 1.1]))
        Cup = (np.array([6.0, 40.0, TableZ]), np.array([11.0, 45.0, TableZ + 9.0]))
        Depth = TDC.RenderDepth(Camera, TDC.Workspace([Platform, Can, Eraser, Cup]), FloorZ=TDC.RoomFloorZ)
        for Item, Tall in ((Eraser, 1.1), (Cup, 9.0)):
            with Self.subTest(height=Tall):
                Box = TDC.CaptureBox(Camera, *Item)
                Plain, WithPlatform = Self.Measure(Depth, Box), Self.Measure(Depth, Box, Declared)
                Self.assertTrue(Plain["ok"], Plain)
                Self.assertEqual(WithPlatform, Plain, "a declared platform elsewhere changes nothing")
                Self.assertEqual(Plain["support_branch"], "desk")
                Self.assertAlmostEqual(Plain["height_cm"], Tall, delta=0.4)

    def TestACanOnALowShelfIsMeasuredOnItAndDeskObjectsBesideItAreNot(Self):
        Model = TDC.Verified(Camera, ShelfWorkspace(), FloorZ=TDC.RoomFloorZ)
        Desk = CD.DisplayDeskPlane(Model)
        Self.assertAlmostEqual(float(DG.DeskHeight(DG.WorkspaceSurfaces(Desk, TableZ), -20.0, 35.0)), TableZ, delta=0.1)
        Eraser = DeskEraser
        Depth = TDC.RenderDepth(Camera, ShelfWorkspace([*ShelfCans, EdgeCan, Eraser]), FloorZ=TDC.RoomFloorZ)
        Measure = lambda Box, *Specs: DG.MeasureBox(Depth, Box, TDC.Intrinsics, Model, TableZ, True,
                                                   Surfaces=DG.WorkspaceSurfaces(Desk, TableZ, Declared=Valid(*Specs)))
        for Item in [*ShelfCans, EdgeCan]:
            with Self.subTest(x=float(Item[0][0])):
                Result = Measure(TDC.CaptureBox(Camera, *Item), ShelfSpec)
                Self.assertTrue(Result["ok"], Result)
                Self.assertEqual((Result["support_branch"], Result["support_surface"]), ("surface", "low shelf"))
                Self.assertAlmostEqual(Result["z"], ShelfTop, delta=0.3)
                Self.assertAlmostEqual(Result["height_cm"], 7.0, delta=0.4)
        for Item in ShelfCans:
            Undeclared = Measure(TDC.CaptureBox(Camera, *Item))
            Self.assertFalse(Undeclared["ok"], Undeclared)
            Self.assertAlmostEqual(Undeclared["undeclared_support_z"], ShelfTop, delta=0.3)
        # Undeclared by the desk's end, the can reads as its top above the desk line (as before): never lower than it stands.
        Edge = Measure(TDC.CaptureBox(Camera, *EdgeCan))
        Self.assertTrue(not Edge["ok"] or Edge["z"] >= TableZ - 0.3, Edge)
        Box = TDC.CaptureBox(Camera, *Eraser)
        Plain, WithShelf = Measure(Box), Measure(Box, ShelfSpec)
        Self.assertTrue(Plain["ok"], Plain)
        Self.assertEqual({Key: Value for Key, Value in WithShelf.items() if Key != "declared_surface"}, Plain)

    def TestACameraPitchedUpIsStillNeverMeasuredOnAGuessedSurface(Self):
        # TestDepthCamera's misplaced desk and camera: no declared surface, so never an object, whatever surface it finds.
        Low, High = np.array([-10.0, 33.0, TableZ]), np.array([-5.0, 35.0, TableZ + 1.1])
        for Degrees in (10.0, 13.0):
            Truth = TDC.Turned(Camera, -Degrees)
            Depth = TDC.RenderDepth(Truth, TDC.Workspace([(Low, High)]), FloorZ=TDC.RoomFloorZ)
            Result = Self.Measure(Depth, TDC.CaptureBox(Truth, Low, High))
            Self.assertFalse(Result["ok"], Result)


class SurfaceLocateTests(unittest.TestCase):
    """locate_objects with the declared surfaces it reads (DeclaredSurfaces.Load), the camera gate and the tripwire."""

    def Locate(Self, Model, Depth, Boxes, Specs):
        Image = np.zeros((Height, Width, 3), np.uint8)
        with patch.object(DeclaredSurfaces, "Load", return_value=Valid(*Specs)), \
                patch.object(calibration, "camera_model", return_value=Model), \
                patch.object(calibration, "pixel_to_plane", side_effect=lambda U, V, Mirrored=None: SC.PixelToPlane(Model, U, V, Mirrored)), \
                patch.object(calibration, "status", return_value={"calibrated": True, "rms_cm": 0.3}), \
                patch.object(object_locator, "_pick_model", MagicMock(return_value="test-model")), \
                patch.object(object_locator, "_detect_once", MagicMock(return_value=Boxes)), \
                patch.object(object_locator, "_refine_detection", return_value=None):
            return object_locator.locate_objects(TDC.Encode(Image), depth={"depth_mm": Depth, "intrinsics": dict(TDC.Intrinsics),
                                                                           "mirrored": True, "seq": 5}, mirrored=True)

    def TestTheCanOnThePlatformComesBackWithItsSurface(Self):
        # The platform was not there at calibration: the camera gate still passes with it in view.
        Model = dict(TDC.Verified(Camera, TDC.Workspace(), FloorZ=TDC.RoomFloorZ), observed_tip_bounds_cm={"z": [8.3, 14.5]})
        Depth = TDC.RenderDepth(Camera, TDC.Workspace([Platform, Can]), FloorZ=TDC.RoomFloorZ)
        Boxes = [TDC.Detected("can", TDC.CaptureBox(Camera, *Can))]
        Result = Self.Locate(Model, Depth, Boxes, [Declared])
        Self.assertTrue(Result["ok"], Result)
        Self.assertTrue(Result["camera_check"]["ok"], Result["camera_check"])
        [Object] = Result["objects"]
        Self.assertEqual((Object["support_branch"], Object["support_surface"], Object["support_kind"]), ("surface", "box platform", "platform"))
        Self.assertAlmostEqual(Object["z"], PlatformTop, delta=0.3)
        Self.assertAlmostEqual(Object["top_z"], PlatformTop + 12.0, delta=0.4)
        # Its top is 10.2 cm above the highest calibration tip (Z 14.5): said, not refused.
        Self.assertAlmostEqual(Object["outside_calibration_z_cm"], PlatformTop + 12.0 - 14.5, delta=0.5)
        Undeclared = Self.Locate(Model, Depth, Boxes, [])
        Self.assertTrue(Undeclared["ok"], Undeclared)
        [Automatic] = Undeclared["objects"]
        Self.assertEqual(Automatic["support_geometry"]["Source"], "depth")
        Self.assertAlmostEqual(Automatic["z"], PlatformTop, delta=0.3)
        # A wider support is measured locally too, after this capture's camera gate.
        Wide = (np.array([-40.0, 28.0, TableZ]), np.array([12.0, 62.0, PlatformTop]))
        Depth = TDC.RenderDepth(Camera, TDC.Workspace([Wide, Can]), FloorZ=TDC.RoomFloorZ)
        Hinted = Self.Locate(dict(TDC.Verified(Camera, TDC.Workspace([Wide]), FloorZ=TDC.RoomFloorZ)), Depth, Boxes, [])
        Self.assertTrue(Hinted["ok"], Hinted)
        [Automatic] = Hinted["objects"]
        Self.assertEqual(Automatic["support_geometry"]["Source"], "depth")
        Self.assertAlmostEqual(Automatic["z"], PlatformTop, delta=0.3)

    def TestCansOnADeclaredLowShelfDoNotTripTheSceneTripwire(Self):
        # Three objects on a surface 5.7 cm under the desk line read a shared -5.7 cm desk offset: undeclared,
        # the tripwire blocks the request as a moved camera (as it must for a moved camera); declared, they are measured.
        Model = TDC.Verified(Camera, ShelfWorkspace(), FloorZ=TDC.RoomFloorZ)
        Depth = TDC.RenderDepth(Camera, ShelfWorkspace(ShelfCans), FloorZ=TDC.RoomFloorZ)
        Boxes = [TDC.Detected(f"can {Index}", TDC.CaptureBox(Camera, *Item)) for Index, Item in enumerate(ShelfCans)]
        Automatic = Self.Locate(Model, Depth, Boxes, [])
        Self.assertTrue(Automatic["ok"], Automatic)
        Self.assertEqual(len(Automatic["objects"]), 3)
        for Object in Automatic["objects"]:
            Self.assertEqual(Object["support_geometry"]["Source"], "depth")
            Self.assertAlmostEqual(Object["z"], ShelfTop, delta=0.3)
        Measured = Self.Locate(Model, Depth, Boxes, [ShelfSpec])
        Self.assertTrue(Measured["ok"], Measured)
        Self.assertEqual(sorted(Object["label"] for Object in Measured["objects"]), ["can 0", "can 1", "can 2"])
        for Object in Measured["objects"]:
            Self.assertEqual(Object["support_surface"], "low shelf")
            Self.assertAlmostEqual(Object["z"], ShelfTop, delta=0.3)


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestSurfaceSupport` runs the Test* methods too (unittest names this hook)."""
    Prefixed = unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return unittest.TestSuite(Prefixed.loadTestsFromTestCase(Case) for Case in (SurfaceMeasureTests, SurfaceLocateTests))


if __name__ == "__main__":
    Loader = unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Suite = unittest.TestSuite(Loader.loadTestsFromTestCase(Case) for Case in (SurfaceMeasureTests, SurfaceLocateTests))
    Result = unittest.TextTestRunner(verbosity=2).run(Suite)
    raise SystemExit(not Result.wasSuccessful())
