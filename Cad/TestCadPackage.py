"""Check deliverable provenance, dependencies and the exported 3D scene."""

import hashlib as Hashlib
import json as Json
from pathlib import Path
import struct as Struct
import unittest as Unittest
import xml.etree.ElementTree as Xml

import numpy as Np
import trimesh as Trimesh

from ReferenceKinematics import ForwardKinematics, JointOrder, LoadModel, OriginTransform, OutputRoot

Verification = {}

class CadPackageTests(Unittest.TestCase):
    def TestOriginalSourceIntegrity(Self):
        Manifest = Json.loads((OutputRoot / "SourceManifest.json").read_text())
        for Record in Manifest["Files"]:
            Data = (OutputRoot / Record["Path"]).read_bytes()
            Self.assertEqual(Hashlib.sha256(Data).hexdigest(), Record["Sha256"])
            Self.assertEqual(len(Data), Record["Bytes"])

    def TestUrdfMeshDependencies(Self):
        for File in (OutputRoot / "Original/Simulation/SO101").glob("*.urdf"):
            for Mesh in Xml.parse(File).getroot().iter("mesh"):
                Self.assertTrue((File.parent / Mesh.get("filename")).is_file(), Mesh.get("filename"))

    def TestExportedAssembly(Self):
        Data = (OutputRoot / "SO101Reference.glb").read_bytes()
        Magic, Version, Length = Struct.unpack_from("<4sII", Data)
        Self.assertEqual((Magic, Version, Length), (b"glTF", 2, len(Data)))
        ChunkLength, ChunkType = Struct.unpack_from("<II", Data, 12)
        Document = Json.loads(Data[20:20 + ChunkLength])
        Names = {Node.get("name") for Node in Document["nodes"]}
        Self.assertTrue(set(JointOrder).issubset(Names))
        Self.assertEqual(len(Document["meshes"]), 17)
        Scene = Trimesh.load(OutputRoot / "SO101Reference.glb", force="scene")
        Np.testing.assert_array_less(Scene.bounds[0], Scene.bounds[1])
        # Compare real transformed source vertices, not the looser rotated
        # object bounding boxes used for camera framing in Blender.
        Model = LoadModel()
        Links, Frames = ForwardKinematics(Model)
        SourceBounds = []
        for Link in Model["Xml"].findall("link"):
            for Visual in Link.findall("visual"):
                Element = Visual.find("geometry/mesh")
                if Element is None:
                    continue
                Mesh = Trimesh.load(Model["Path"].parent / Element.get("filename"), force="mesh")
                Mesh.apply_scale([float(Value) for Value in Element.get("scale", "1 1 1").split()])
                Mesh.apply_transform(Links[Link.get("name")] @ OriginTransform(Visual.find("origin")))
                SourceBounds.append(Mesh.bounds)
        Minimum = Np.min([Bounds[0] for Bounds in SourceBounds], axis=0)
        Maximum = Np.max([Bounds[1] for Bounds in SourceBounds], axis=0)
        # Blender writes glTF Y-up: (X,Y,Z) -> (X,Z,-Y).
        Target = Np.array([[Minimum[0], Minimum[2], -Maximum[1]], [Maximum[0], Maximum[2], -Minimum[1]]])
        Verification["ExportBoundsMaximumErrorMm"] = float(Np.max(Np.abs(Scene.bounds - Target)) * 1000)
        # Blender/glTF uses float32 transforms and Euler decomposition.
        # Permit five micrometres for the inspection mesh, not the FK model.
        Np.testing.assert_allclose(Scene.bounds, Target, atol=5e-6)
        Self.assertGreater((OutputRoot / "SO101Reference.blend").stat().st_size, 100000)
        Self.assertGreater((OutputRoot / "SO101Preview.png").stat().st_size, 10000)


if __name__ == "__main__":
    Loader = Unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Result = Unittest.TextTestRunner(verbosity=2).run(Loader.loadTestsFromTestCase(CadPackageTests))
    Verification.update({"Successful": Result.wasSuccessful(), "TestsRun": Result.testsRun})
    (OutputRoot / "PackageVerification.json").write_text(Json.dumps(Verification, indent=2), encoding="utf-8")
    raise SystemExit(0 if Result.wasSuccessful() else 1)
