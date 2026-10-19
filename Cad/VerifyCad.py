"""Check the original STEP solids with Open CASCADE, without changing geometry."""

import json as Json
from pathlib import Path

from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer

OutputRoot = Path.home() / "Downloads" / "SO101Cad"
Records = []
for File in sorted((OutputRoot / "Original" / "STEP" / "SO101").rglob("*.step")):
    Reader = STEPControl_Reader()
    if Reader.ReadFile(str(File)) != IFSelect_RetDone:
        raise ValueError(f"Cannot read STEP file: {File}")
    if Reader.TransferRoots() <= 0:
        raise ValueError(f"No transferred STEP roots: {File}")
    Shape = Reader.OneShape()
    Explorer = TopExp_Explorer(Shape, TopAbs_SOLID)
    Invalid = []
    Count = 0
    while Explorer.More():
        if not BRepCheck_Analyzer(Explorer.Current()).IsValid():
            Invalid.append(Count)
        Count += 1
        Explorer.Next()
    Record = {"File": str(File.relative_to(OutputRoot)).replace("\\", "/"),
              "SolidCount": Count, "InvalidSolidIndices": Invalid,
              "Valid": bool(BRepCheck_Analyzer(Shape).IsValid())}
    Records.append(Record)
    if File.name != "SO101 Assembly.step":
        assert Count > 0 and Record["Valid"], f"Invalid individual printed part: {File}"
    print(Record, flush=True)
(OutputRoot / "CadVerification.json").write_text(Json.dumps(Records, indent=2), encoding="utf-8")
print("Individual printed-part STEP solids validated. Assembly issues are reported, not repaired.", flush=True)
