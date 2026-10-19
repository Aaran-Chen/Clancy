"""Surface/contact regressions against analytic solids and the actual cart CAD."""
import hashlib as Hashlib
import importlib.util as ImportUtil
import itertools as Itertools
import json as Json
import subprocess as Subprocess
import sys as Sys
import tempfile as Tempfile
import unittest as Unittest
from pathlib import Path

import numpy as Np
import BasketSurface as Mesh
import BasketCollision as Collision
from TestBasketCollision import MakeController


def CubeSource():
    Vertices = list(Itertools.product((-1., 1.), repeat=3))
    Faces = []
    for Axis in range(3):
        Other = [Index for Index in range(3) if Index != Axis]
        for Side in (-1, 1):
            Points = []
            for A, B in ((-1, -1), (-1, 1), (1, 1), (1, -1)):
                Value = [0., 0., 0.]
                Value[Axis], Value[Other[0]], Value[Other[1]] = Side, A, B
                Points.append(Vertices.index(tuple(Value)))
            Faces.extend(([Points[0], Points[1], Points[2]], [Points[0], Points[2], Points[3]]))
    return {"Vertices": Vertices, "Faces": Faces, "Nodes": [{"Bounds": [[-1, -1, -1], [1, 1, 1]], "Faces": list(range(12))}],
            "Components": [0] * 12, "Solids": [True]}


class SurfaceTests(Unittest.TestCase):
    def TestTouchAndContainmentAgainstAnalyticCube(Self):
        Surface = Mesh.Surface([0, 0, 0], CubeSource())
        for Point, Expected in (([0, 0, 0], True), ([1, 0, 0], True), ([1, 1, 1], True),
                                ([1.0001, 0, 0], False), ([-1.0001, 0, 0], False)):
            Self.assertEqual(Surface.Overlaps(Np.array(Point), Np.eye(3), Np.zeros(3)), Expected)
        Self.assertTrue(Surface.Overlaps(Np.array([2., 0, 0]), Np.eye(3), Np.ones(3)))
        Self.assertFalse(Surface.Overlaps(Np.array([2.001, 0, 0]), Np.eye(3), Np.ones(3)))
        Self.assertTrue(Surface.Overlaps(Np.zeros(3), Np.eye(3), Np.ones(3) * 0.01), "Fully contained box has no surface crossing")
        Self.assertTrue(Surface.Overlaps(Np.zeros(3), Np.eye(3), Np.ones(3) * 5), "Box encloses an entire solid")

    def TestRotatedBoxesMatchIndependentAnalyticSat(Self):
        Surface = Mesh.Surface([0, 0, 0], CubeSource())
        Random = Np.random.default_rng(41)
        Bounds = Np.array([[-1., -1, -1], [1., 1, 1]])
        for _ in range(500):
            Center = Random.uniform(-3, 3, 3)
            Rotation, _ = Np.linalg.qr(Random.normal(size=(3, 3)))
            Half = Random.uniform(.001, 1, 3)
            Self.assertEqual(Surface.Overlaps(Center, Rotation, Half), Collision.BoxesOverlap(Center, Rotation, Half, Bounds))

    def TestOpenTrayAndGapsAreUsableButFloorIsBlocked(Self):
        Controller, _ = MakeController()
        World = Controller.CollisionWorld()
        Offset = Np.array(Collision.Model["Mounts"]["Right"]["PanCm"])
        for Point in ([0, 0, 73], [0, 0, 76], [0, 0, 60]):
            Self.assertIsNone(World.TargetReason(dict(zip(("x", "y", "z"), Np.array(Point) - Offset))))
        Self.assertIsNotNone(World.TargetReason(dict(zip(("x", "y", "z"), Np.array([0, 0, 68.5]) - Offset))))
        Outside = World.Bounds.mean(axis=0)
        Outside[0] = World.Bounds[1, 0] + 0.001
        Self.assertIsNone(World.TargetReason(dict(zip(("x", "y", "z"), Outside))))
        Self.assertEqual(Collision.Model["ClearanceCm"], 0)
        Self.assertEqual(Collision.Model["InteriorPolicy"], "SurfaceOnly")

    def TestFormerlyBlockedArmPoseIsClearIncludingTracking(Self):
        Controller, _ = MakeController()
        World = Controller.CollisionWorld()
        Angles = dict(zip(Collision.Names, (-96.01400151436108, -13.181901323548914, -73.94974264632789,
                                          58.93324029461547, 94.39453125, 0)))
        Links, _ = Collision.Cad.Forward(Controller.arm_geometry, Angles)
        LegacyBlocked = False
        for _, Link, Center, Half, Radii in World.Boxes:
            Transform = Links[Link]
            Padding = 1 + float(Radii.sum() * World.Slack * Collision.Degrees)
            LegacyBlocked |= Collision.BoxesOverlap(Transform[:3, :3] @ Center + Transform[:3, 3],
                                                   Transform[:3, :3], Half, World.Bounds, Padding)
        Self.assertTrue(LegacyBlocked)
        # This side reach clears all surfaces and the rearward limit. The former
        # center-of-mass blocker must no longer reject it.
        Self.assertIsNone(World.PoseReason(Angles))

    def TestSurfaceTreeContainsEveryTriangleExactlyOnce(Self):
        Surface = Mesh.Surface([0, 0, 0])
        Indices = []
        for Index, Node in enumerate(Surface.Nodes):
            if "Children" in Node:
                for Child in Node["Children"]:
                    Self.assertTrue(Np.all(Surface.Bounds[Child, 0] >= Surface.Bounds[Index, 0]))
                    Self.assertTrue(Np.all(Surface.Bounds[Child, 1] <= Surface.Bounds[Index, 1]))
            else:
                Indices.extend(Node["Faces"])
                Triangles = Surface.Triangles[Node["Faces"]]
                Self.assertTrue(Np.all(Triangles >= Surface.Bounds[Index, 0]))
                Self.assertTrue(Np.all(Triangles <= Surface.Bounds[Index, 1]))
        Self.assertEqual(sorted(Indices), list(range(len(Surface.Triangles))))
        File = Path(__file__).resolve().parent / "BasketSurface.json"
        Self.assertEqual(Hashlib.sha256(File.read_bytes()).hexdigest(), Collision.Model["BasketSurfaceSha256"])

    def TestActualCadContactsAndArmMountTransforms(Self):
        for Side, Surface in Collision.Surfaces.items():
            Offset = Np.array(Collision.Model["Mounts"][Side]["PanCm"])
            for Triangle in Surface.Triangles[::350]:
                Self.assertTrue(Surface.Overlaps(Triangle.mean(axis=0), Np.eye(3), Np.zeros(3)), Side)
            Np.testing.assert_allclose(Surface.Bounds[0] + Offset, Collision.Model["BasketAssemblyBoundsCm"], atol=1e-4)

    def TestTrayBoundsFollowTheTopTraySurface(Self):
        # The open top tray, the footprint over which a target may go below the
        # desk: unlike the whole-cart bounds it ends at the tray's front rim.
        Tray = Mesh.Surface([0, 0, 0])
        Triangles = Tray.Triangles[Tray.Components == Mesh.Data["FrontWallCorrection"]["Component"]].reshape(-1, 3)
        Np.testing.assert_allclose([Triangles.min(axis=0), Triangles.max(axis=0)], Collision.Model["BasketTrayBoundsCm"], atol=1e-9)
        Frame = Json.loads((Path(__file__).resolve().parent / "WorkspaceFrame.json").read_text())["RobotFrame"]
        FrontRim = Collision.Model["BasketTrayBoundsCm"][1][1] - Frame["OriginAssemblyCm"][1]
        Self.assertAlmostEqual(FrontRim, Mesh.Data["FrontWallCorrection"]["FrontRimYCm"], delta=1e-6)
        Self.assertLess(FrontRim, Collision.Model["BasketAssemblyBoundsCm"][1][1] - Frame["OriginAssemblyCm"][1] - 5)

    @Unittest.skipUnless(ImportUtil.find_spec("trimesh"), "rebuilding from the CAD needs trimesh")
    def TestGeneratorRebuildsTheCommittedModel(Self):
        # Cad/PrepareCollisionModel.py rebuilds these files from the GLBs. It used to write the CAD's
        # front wall back and drop BasketTrayBoundsCm, silently undoing the measured correction.
        Backend = Path(__file__).resolve().parent
        with Tempfile.TemporaryDirectory() as Folder:
            Subprocess.run([Sys.executable, str(Backend.parent / "Cad" / "PrepareCollisionModel.py"), Folder],
                           check=True, capture_output=True)
            Self.assertEqual((Path(Folder) / "CollisionModel.json").read_text(), (Backend / "CollisionModel.json").read_text())
            for Name in ("BasketSurface.json", "RobotBodySurface.json"):
                Self.assertEqual(Hashlib.sha256((Path(Folder) / Name).read_bytes()).hexdigest(),
                                 Hashlib.sha256((Backend / Name).read_bytes()).hexdigest(), Name)

    def TestTreeCandidatesMatchIndependentLeafOverlap(Self):
        Random = Np.random.default_rng(24)
        for Surface in Collision.Surfaces.values():
            Leaves = [Index for Index, Node in enumerate(Surface.Nodes) if "Faces" in Node]
            Bounds = Surface.Bounds[Leaves]
            for _ in range(100):
                Center = Random.uniform(Surface.Bounds[0, 0] - 10, Surface.Bounds[0, 1] + 10)
                Half = Random.uniform(0, 12, 3)
                Low, High = Center - Half, Center + Half
                Overlap = Np.all(High >= Bounds[:, 0] - Mesh.Epsilon, axis=1) & Np.all(Low <= Bounds[:, 1] + Mesh.Epsilon, axis=1)
                Expected = [Face for Index, Hit in zip(Leaves, Overlap) if Hit for Face in Surface.Nodes[Index]["Faces"]]
                Self.assertEqual(sorted(Surface.Candidates(Low, High)), sorted(Expected))

    def TestTrackingEnvelopeStillRejectsPossibleContact(Self):
        Surface = Mesh.Surface([0, 0, 0], CubeSource())
        Center, Half = Np.array([1.2, 0, 0]), Np.ones(3) * .1
        Self.assertFalse(Surface.Overlaps(Center, Np.eye(3), Half))
        Self.assertTrue(Surface.Overlaps(Center, Np.eye(3), Half, Padding=.11))


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestBasketSurface` runs the Test* methods too (unittest names this hook)."""
    Prefixed = Unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return Prefixed.loadTestsFromTestCase(SurfaceTests)


if __name__ == "__main__":
    Loader = Unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Result = Unittest.TextTestRunner(verbosity=2).run(Loader.loadTestsFromTestCase(SurfaceTests))
    raise SystemExit(not Result.wasSuccessful())
