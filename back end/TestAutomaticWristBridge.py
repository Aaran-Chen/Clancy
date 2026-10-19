"""Verify the browser's recovered wrist pose against the strict bridge solver.

Run TestAutomaticWristUi.mjs first (RunAllTests.py runs this last, after it): it deletes the old fixture when it starts
and writes a new one when it passes, stamped with a hash of the app's src/. A missing fixture (that run failed or never
ran) or one from other app code (stale) fails here, saying which: this used to pass on whatever an earlier run had left
(audit 2 L7). No serial connection or real motor command is used.
"""
import hashlib as Hashlib
import json as Json
import math as Math
from pathlib import Path
import bridge_server as Bridge

Root = Path(__file__).resolve().parent
App = Root.parent / "AI Controlled WebApp"
FixturePath = Root / "TestArtifacts" / "AutomaticWristUiResults.json"


def SourceHash(Folder: Path) -> str:
    """sha256 over every file of Folder (path, then bytes), as TestAutomaticWristUi.mjs stamps its fixture."""
    Digest = Hashlib.sha256()
    for File in sorted((Item for Item in Folder.rglob("*") if Item.is_file()), key=lambda Item: Item.relative_to(Folder).as_posix()):
        Digest.update(File.relative_to(Folder).as_posix().encode("utf-8") + b"\n" + File.read_bytes() + b"\n")
    return Digest.hexdigest()


if not FixturePath.is_file():
    raise SystemExit(f"FAIL: {FixturePath} is missing: TestAutomaticWristUi.mjs writes it when it passes (run it first; if it ran, it failed).")
Fixture = Json.loads(FixturePath.read_text(encoding="utf-8"))
if Fixture.get("Passed") is not True or Fixture.get("SourceHash") != SourceHash(App / "src"):
    raise SystemExit(f"FAIL: {FixturePath} is stale (written {Fixture.get('WrittenAt', 'before fixtures were stamped')} from other app code): "
                     "run TestAutomaticWristUi.mjs again.")
Controller = Bridge.BridgeController()
Target = Fixture["Resolved"]
Requested = Fixture["Requested"]
assert all(Target[Axis] == Requested[Axis] for Axis in ("x", "y", "z"))
Preview = Controller._solve_preview(Target, Target["wristPitch"], allow_unreachable=False, max_iterations=70)
assert Preview["reachable"], Preview
assert Controller.bus is None
Quantized = Controller._current_ik_angles()
for Name in Bridge.IK_JOINTS:
    Servo = Controller.servos[Name]
    Raw = Controller._ik_raw_from_angle(Servo, Preview["angles"][Name])
    Quantized[Name] = Controller._ik_angle_from_raw(Servo, Raw)
Tip = Bridge.forward_tip_position(Controller.arm_geometry, Quantized)
Error = Math.dist(tuple(Target[Axis] for Axis in ("x", "y", "z")), tuple(Tip[Axis] for Axis in ("x", "y", "z")))
PitchError = Controller._angle_delta_abs(Bridge.wrist_tip_pitch_degrees(Controller.arm_geometry, Quantized), Target["wristPitch"])
assert Error <= Bridge.IK_SEND_MAX_ERROR_CM, Error
assert PitchError <= .25, PitchError
assert Controller.CollisionWorld().PoseReason(Quantized) is None
print(Json.dumps({"Result": "PASS", "QuantizedPositionErrorCm": Error, "QuantizedPitchErrorDegrees": PitchError, "HardwareConnected": False}))
