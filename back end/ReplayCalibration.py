"""Recover and validate an archived run offline. Never imports motor control or applies a camera model.

Usage: python ReplayCalibration.py RunDirectory --hints Hints.json
Hints map pose indices (or an observation filename for repeated poses) to native-image {"u": ..., "v": ...}.
"""
import argparse
import copy
import json
from pathlib import Path
from unittest.mock import patch

import CalibrationArchive as Archive
import SelfCalibration as SC


def Replay(RunDirectory, Hints=None):
    RunDirectory = Path(RunDirectory).resolve()
    Manifest = json.loads((RunDirectory / "Manifest.json").read_text(encoding="utf-8"))
    Workspace = Manifest.get("context", {}).get("workspace_frame", {})
    TableZ = Workspace.get("TableZCm", SC.DefaultTableZCm)
    Hints = Hints or {}
    # Keep replays beside source runs, avoiding deeply nested Windows paths.
    CapturesRoot = RunDirectory.parents[3]
    Output = Archive.CreateRun(CapturesRoot, {"replay_of": str(RunDirectory), "workspace_frame": Workspace})
    Archive.WriteJson(Output / "ReplayRequest.json", {"source_run": str(RunDirectory), "hints": Hints, "table_z": TableZ})
    OriginalSession, OriginalTracker = copy.deepcopy(SC._Session), copy.deepcopy(SC._RedTracker)
    Results = []
    try:
        with patch.object(SC, "SessionPath", Output / "RecoveredSession.json"), patch.object(SC, "CapturesDir", CapturesRoot):
            SC._Session.clear()
            SC._Session.update(image_size=None, mirrored=None, observations=[], archive_directory=str(Output), run_id=Output.name)
            SC._RedTracker.clear()
            for InputPath in sorted((RunDirectory / "observations").glob("*.json")):
                Capture = json.loads(InputPath.read_text(encoding="utf-8"))
                if Capture.get("metadata", {}).get("capture_valid") is False:
                    Results.append({"input": InputPath.name, "detected": False, "reason": "Capture was marked invalid; retained for manual review."})
                    continue
                Hint = Hints.get(InputPath.name, Hints.get(str(Capture["pose_index"])))
                if Hint is not None:
                    Selection = SC.CheckFrames(Capture["images"], Capture["mirrored"], Hint=Hint, Initialize=True)
                    if not Selection["detected"]:
                        Results.append({"input": InputPath.name, "detected": False, "reason": Selection["reason"]})
                        continue
                elif not SC._RedTracker and Capture.get("tracker_before"):
                    SC._RedTracker.update(Capture["tracker_before"])
                Result = SC.AddObservation(Capture["images"], Capture["joints"], Capture["mirrored"],
                                           Capture["pose_index"], Method=Capture["marker_method"], Metadata=Capture.get("metadata"))
                Results.append({"input": InputPath.name, **Result})
                if not Result["ok"]:
                    raise RuntimeError(Result.get("error", "Offline observation failed"))
            Report = SC.Solve(TableZ=TableZ, Save=False)
            Report["replay_results"] = Results
            Archive.WriteJson(Output / "FitReport.json", Report)
            Archive.WriteJson(Output / "RecoveredSession.json", SC._Session)
    finally:
        SC._Session.clear()
        SC._Session.update(OriginalSession)
        SC._RedTracker.clear()
        SC._RedTracker.update(OriginalTracker)
    return {"output_directory": str(Output), "report": Report}


if __name__ == "__main__":
    Parser = argparse.ArgumentParser(description=__doc__)
    Parser.add_argument("RunDirectory", type=Path)
    Parser.add_argument("--hints", dest="Hints", type=Path)
    Arguments = Parser.parse_args()
    Hints = json.loads(Arguments.Hints.read_text(encoding="utf-8")) if Arguments.Hints else None
    Result = Replay(Arguments.RunDirectory, Hints)
    print(json.dumps({"output_directory": Result["output_directory"],
                      **{Key: Value for Key, Value in Result["report"].items() if Key not in ("model", "replay_results")}}, indent=2))
