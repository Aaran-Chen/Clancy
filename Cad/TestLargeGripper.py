"""Check the large gripper deliverable: files, unchanged mounting features and a collision-free sweep.

Run after PrepareLargeGripper.py:
  & Cad\\.venv\\Scripts\\python.exe Cad\\TestLargeGripper.py
"""
import hashlib as Hashlib
import json as Json
import math as Math
import os as Os
import sys as Sys
import unittest as Unittest

import numpy as Np
import trimesh as Trimesh
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.GeomAbs import GeomAbs_Plane

from PrepareLargeGripper import (Box, CommonVolume, Design, Distance, JawToBody, LoadStep, MountingFaces, Output, SourceRoot,
                                 Solids, StockBodyStep, StockJawStep, Transform, Volume)

Files = ["LargeGripperFixedJawBody.step", "LargeGripperFixedJawBody.stl", "LargeGripperMovingJaw.step",
         "LargeGripperMovingJaw.stl", "LargeGripperClosed.png", "LargeGripperOpen.png", "LargeGripperCan66.png",
         "LargeGripperGrasps.png", "LargeGripperVsStock.png", "Verification.json", "License.txt"]


class LargeGripperTests(Unittest.TestCase):
    @classmethod
    def setUpClass(Class):
        Class.Report = Json.loads((Output / "Verification.json").read_text())
        Class.Body = LoadStep(Output / "LargeGripperFixedJawBody.step")
        Class.Jaw = LoadStep(Output / "LargeGripperMovingJaw.step")
        Class.StockBody = LoadStep(SourceRoot / StockBodyStep)
        Class.StockJaw = LoadStep(SourceRoot / StockJawStep)
        Class.Travel = Class.Report["Travel"]

    def TestOutputsExist(Self):
        for Name in Files:
            Self.assertGreater((Output / Name).stat().st_size, 1000, Name)
        Self.assertEqual((Output / "License.txt").read_bytes(), (SourceRoot / "Original/LICENSE").read_bytes())
        for Name, Digest in Self.Report["Files"].items():
            Self.assertEqual(Hashlib.sha256((Output / Name).read_bytes()).hexdigest(), Digest, Name)

    def TestPartsArePrintableSolids(Self):
        for Shape in (Self.Body, Self.Jaw):
            Self.assertEqual(Solids(Shape), 1)
            Self.assertTrue(BRepCheck_Analyzer(Shape.wrapped).IsValid())
        for Name in ("LargeGripperFixedJawBody.stl", "LargeGripperMovingJaw.stl"):
            Mesh = Trimesh.load(Output / Name, force="mesh")
            Self.assertTrue(Mesh.is_watertight, Name)
            Self.assertLess(max(Mesh.extents), 250, Name)   # Kobra S1 bed

    def TestMountingFeaturesMatchUpstream(Self):
        """Everything below the finger cut is the upstream solid: servo pocket, screw holes, horn interfaces."""
        PalmZ = Self.Report["Dimensions"]["PalmZ"]
        Placed = JawToBody(0)
        Checks = (MountingFaces(Self.Body, Self.StockBody, PalmZ),
                  MountingFaces(Transform(Self.Jaw, Placed), Transform(Self.StockJaw, Placed), Design["MovingCutZ"]))
        for Record in Checks:
            Self.assertGreater(Record["UpstreamFacesBelowCut"], 100)
            Self.assertGreater(Record["CylindricalFacesBelowCut"], 30)   # screw holes, horn and pocket features
            Self.assertEqual(Record["Missing"], [])
        # Negative control: a 0.01 mm shift must be detected.
        Shifted = Transform(Self.Body, [[1, 0, 0, 0.01], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        Self.assertTrue(MountingFaces(Shifted, Self.StockBody, PalmZ)["Missing"])
        # Same material below the cut (a box intersection, which Open Cascade handles reliably).
        Region = Box(-100, 100, -100, 100, -50, PalmZ - 0.05)
        Self.assertAlmostEqual(Volume(Self.Body.intersect(Region)), Volume(Self.StockBody.intersect(Region)), delta=0.5)   # 1e-5 relative: integration noise

    def TestFixedJawIsStraight(Self):
        """One flat inner face, normal +x toward the moving jaw, along the whole finger."""
        InnerX = Design["FixedOuterX"] + Design["FixedThickness"]
        Faces = []
        for Face in Self.Body.Faces():
            Surface = BRepAdaptor_Surface(Face.wrapped)
            if Surface.GetType() != GeomAbs_Plane:
                continue
            Normal = Surface.Plane().Axis().Direction()
            Bounds = Face.BoundingBox()
            if abs(Bounds.xmin - InnerX) < 1e-3 and abs(Bounds.xmax - InnerX) < 1e-3 and abs(abs(Normal.X()) - 1) < 1e-9:
                Faces.append(Bounds)
        Self.assertEqual(len(Faces), 1)
        Dimensions = Self.Report["Dimensions"]
        Self.assertAlmostEqual(Faces[0].zmin, Dimensions["PalmZ"], places=2)
        Self.assertGreater(Faces[0].zmax - Faces[0].zmin, 90)
        Self.assertGreater(Faces[0].ymax - Faces[0].ymin, 40)

    def TestClosesWhereStockCloses(Self):
        Closing = Self.Travel["StockClosingContactDeg"]
        for Fixed, Moving in ((Self.Body, Self.Jaw), (Self.StockBody, Self.StockJaw)):
            Self.assertGreater(CommonVolume(Fixed, Transform(Moving, JawToBody(Closing - 0.3))), 1e-3)
            Self.assertGreater(Distance(Fixed, Transform(Moving, JawToBody(Closing + 0.3))), 0)

    def TestSweepHasNoCollisions(Self):
        """Step the jaw about the real servo axis from just open of the closing contact to the open stop."""
        Start, Stop = Self.Travel["StockClosingContactDeg"] + 0.5, Self.Travel["NewOpenStopDeg"] - 0.25
        Self.assertAlmostEqual(Self.Travel["NewOpenStopDeg"], Self.Travel["StockOpenStopDeg"], places=1)
        for Angle in list(Np.arange(Start, Stop, 4.0)) + [Stop]:
            Moved = Transform(Self.Jaw, JawToBody(float(Angle)))
            Self.assertLess(CommonVolume(Self.Body, Moved), 1e-3, f"overlap at {Angle:.2f} deg")
            Clearance = Distance(Self.Body, Moved)
            Self.assertGreater(Clearance, 0, f"contact at {Angle:.2f} deg")
            if Angle <= 110:   # beyond this the stock yoke closes in on the stock body flange (the open stop)
                Self.assertGreater(Clearance, 0.5, f"clearance at {Angle:.2f} deg")
        Sweep = Self.Report["Sweep"]
        Self.assertEqual(Sweep["CollisionsBeforeOpenStop"], [])
        Self.assertEqual(Sweep["NewFingerContacts"], [])
        Self.assertGreaterEqual(Sweep["RangeDeg"][1], 135.5)

    def TestGraspTargets(Self):
        Grasps = Self.Report["Grasps"]["New"]
        Self.assertEqual(Grasps["Cylinder66mm"]["Kind"], "Caged")
        Self.assertEqual(Grasps["Cylinder40mm"]["Kind"], "Caged")
        Self.assertIsNotNone(Grasps["Cylinder20mm"])
        Self.assertGreaterEqual(Grasps["LargestCagedCylinder"]["DiameterMm"], 66)
        Stock = Self.Report["Grasps"]["Stock"]
        Self.assertGreater(Grasps["LargestHeldCylinder"]["DiameterMm"], Stock["LargestHeldCylinder"]["DiameterMm"])
        for Grasp in (Grasps["Cylinder66mm"], Grasps["Cylinder40mm"], Grasps["Cylinder20mm"]):
            Self.assertLess(Grasp["JawAngleDeg"], Self.Travel["UrdfLimitsDeg"][1])


if __name__ == "__main__":
    Loader = Unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Result = Unittest.TextTestRunner(verbosity=2).run(Loader.loadTestsFromTestCase(LargeGripperTests))
    Sys.stdout.flush()
    Sys.stderr.flush()
    Os._exit(0 if Result.wasSuccessful() else 1)
