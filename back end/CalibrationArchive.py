"""Durable, independent calibration run archives. No motor or bridge imports."""
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import time
import uuid


def WriteJson(PathValue, Payload):
    Destination = Path(PathValue)
    Temporary = Destination.parent / (uuid.uuid4().hex[:12] + ".tmp")
    try:
        with Temporary.open("x", encoding="utf-8") as Output:
            json.dump(Payload, Output, indent=1, allow_nan=False)
            Output.flush()
            os.fsync(Output.fileno())
        Temporary.replace(Destination)
    finally:
        Temporary.unlink(missing_ok=True)


def CreateRun(CapturesRoot, Context=None):
    if Context is None:
        Context = {}
    Backend = Path(__file__).resolve().parent
    Sources = [Backend / Name for Name in ("CalibrationArchive.py", "ReplayCalibration.py", "SelfCalibration.py", "SelfCalClicks.py",
                "bridge_server.py", "arm_geometry.py", "feetech_simple.py", "calibration.py",
                "CadKinematics.py", "CadModel.json", "CadAlignment.json", "WorkspaceFrame.json",
                "BasketCollision.py", "ArmMotionEnvelope.py", "CollisionModel.json", "BasketSurface.py", "BasketSurface.json", "RobotBodySurface.json",
                "DepthGeometry.py", "OrbbecCamera.py", "object_locator.py")]
    Sources += [Backend.parent / "AI Controlled WebApp" / "src" / Name for Name in (
        "App.tsx", "components/MotionThrottle.tsx", "components/MotionThrottle.css",
        "lib/robotModel.ts", "lib/CadKinematics.ts", "lib/BasketCollision.ts", "lib/ArmMotionEnvelope.ts", "lib/BasketSurface.ts", "lib/XLeRobotScene.json")]
    Missing = [str(Source) for Source in Sources if not Source.is_file()]
    if Missing:
        raise FileNotFoundError(f"Missing archive sources: {Missing}")
    RunId = time.strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:12]
    Directory = Path(CapturesRoot) / time.strftime("%Y-%m-%d") / "selfcal" / "runs" / RunId
    Directory.mkdir(parents=True, exist_ok=False)
    (Directory / "observations").mkdir()
    (Directory / "records").mkdir()
    Manifest = {"format_version": 2, "run_id": RunId, "started_at": time.time(),
                "context": Context, "python": platform.python_version(), "packages": {}, "sources": {}}
    for Package in ("numpy", "scipy", "opencv-contrib-python", "fastapi", "pyserial"):
        try:
            Manifest["packages"][Package] = importlib.metadata.version(Package)
        except importlib.metadata.PackageNotFoundError:
            Manifest["packages"][Package] = None
    (Directory / "sources").mkdir()
    for Source in Sources:
        Data = Source.read_bytes()
        (Directory / "sources" / Source.name).write_bytes(Data)
        Manifest["sources"][Source.name] = hashlib.sha256(Data).hexdigest()
    WriteJson(Directory / "Manifest.json", Manifest)
    return Directory


def Record(Directory, Kind, Payload):
    Destination = Path(Directory) / "records" / f"{time.time_ns()}_{Kind}_{uuid.uuid4().hex[:12]}.json"
    WriteJson(Destination, {"kind": Kind, "recorded_at": time.time(), "payload": Payload})
    return str(Destination)
