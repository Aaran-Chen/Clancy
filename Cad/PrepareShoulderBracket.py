"""Copy and verify the original stock SO-101 shoulder motor retaining bracket."""
import hashlib as Hashlib
import json as Json
from pathlib import Path
import shutil as Shutil

import numpy as Np
import trimesh as Trimesh
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer

SourceRoot = Path.home() / "Downloads/SO101Cad"
Output = Path.home() / "Downloads/SO101ShoulderBracket"
Output.mkdir(exist_ok=True)
Manifest = Json.loads((SourceRoot / "SourceManifest.json").read_text())
Records = []
for Relative, Name in (
    ("Original/STEP/SO101/Motor_holder_SO101_Base.step", "ShoulderMotorBracket.step"),
    ("Original/STL/SO101/Individual/Motor_holder_SO101_Base.stl", "ShoulderMotorBracket.stl"),
):
    Source = SourceRoot / Relative
    Entry = next(Item for Item in Manifest["Files"] if Item["Path"] == Relative)
    Digest = Hashlib.sha256(Source.read_bytes()).hexdigest()
    assert Digest == Entry["Sha256"], "Original file differs from recorded source"
    Destination = Output / Name
    Shutil.copy2(Source, Destination)
    assert Hashlib.sha256(Destination.read_bytes()).hexdigest() == Digest
    Records.append({"File": Name, "Source": Entry["SourceUrl"], "Sha256": Digest})

Reader = STEPControl_Reader()
assert Reader.ReadFile(str(Output / "ShoulderMotorBracket.step")) == IFSelect_RetDone
assert Reader.TransferRoots() > 0
Shape = Reader.OneShape()
assert BRepCheck_Analyzer(Shape).IsValid()
Explorer = TopExp_Explorer(Shape, TopAbs_SOLID)
SolidCount = 0
while Explorer.More():
    SolidCount += 1
    Explorer.Next()
assert SolidCount == 1
Mesh = Trimesh.load(Output / "ShoulderMotorBracket.stl", force="mesh")
assert Np.isfinite(Mesh.vertices).all() and len(Mesh.faces) > 0 and Mesh.is_watertight
Report = {"Part": "Motor_holder_SO101_Base", "Role": "Retainer for shoulder lift motor (motor 2)",
          "SourceCommit": Manifest["Commit"], "Files": Records,
          "StepValid": True, "SolidCount": SolidCount, "StlWatertight": bool(Mesh.is_watertight),
          "StlExtentsMm": Mesh.extents.tolist()}
(Output / "Verification.json").write_text(Json.dumps(Report, indent=2) + "\n")
Shutil.copy2(SourceRoot / "Original/LICENSE", Output / "License.txt")
print(Json.dumps(Report, indent=2))
