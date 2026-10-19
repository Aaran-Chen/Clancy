"""Extract nominal dimensions and explain the migration from the current model."""

import csv as Csv
import json as Json
import math as Math
from pathlib import Path
import shutil as Shutil

import numpy as Np

from ReferenceKinematics import ForwardKinematics, JointOrder, LoadModel, OutputRoot

ProjectRoot = Path(__file__).resolve().parents[1]
Model = LoadModel()
Manifest = Json.loads((OutputRoot / "SourceManifest.json").read_text())
Current = Json.loads((ProjectRoot / "back end" / "arm_geometry.json").read_text())
Segments = {Segment["name"]: Segment for Segment in Current["segments"]}
Links, Frames = ForwardKinematics(Model)
Joints = {Joint["Name"]: Joint for Joint in Model["Joints"]}
Upper = float(Np.linalg.norm(Joints["elbow_flex"]["Origin"][:3, 3]) * 1000)
Forearm = float(Np.linalg.norm(Joints["wrist_flex"]["Origin"][:3, 3]) * 1000)
WristVector = Joints["wrist_roll"]["Origin"][:3, 3] * 1000
ToolVector = Joints["gripper_frame_joint"]["Origin"][:3, 3] * 1000
ShoulderDelta = (Frames["shoulder_lift"][:3, 3] - Frames["shoulder_pan"][:3, 3]) * 1000
ToolRadius = float(Np.linalg.norm(ToolVector[:2]))
CurrentUpper = Segments["shoulder_lift_to_elbow"]["length_cm"] * 10
CurrentForearm = Segments["elbow_to_wrist_flex"]["length_cm"] * 10
CurrentShoulderHeight = Segments["shoulder_pan_to_shoulder_lift"]["offset_cm"]["up"] * 10
CurrentShoulderRadial = Segments["shoulder_pan_to_shoulder_lift"]["offset_cm"]["right"] * 10
Verification = Json.loads((OutputRoot / "KinematicsVerification.json").read_text())
CadChecks = Json.loads((OutputRoot / "CadVerification.json").read_text())
PartChecks = [Record for Record in CadChecks if not Record["File"].endswith("SO101 Assembly.step")]
AssemblyCheck = next(Record for Record in CadChecks if Record["File"].endswith("SO101 Assembly.step"))


def Values(VectorValue):
    return ", ".join(f"{Value:.6f}" for Value in VectorValue)


Records = []
for Name in (*JointOrder[:5], "gripper_frame_joint", "gripper"):
    Joint = Joints[Name]
    Element = next(Element for Element in Model["Xml"].findall("joint") if Element.get("name") == Name)
    Rpy = [float(Value) for Value in Element.find("origin").get("rpy", "0 0 0").split()]
    Records.append({
        "Name": Name,
        "Parent": Joint["Parent"],
        "Child": Joint["Child"],
        "Type": Joint["Type"],
        "ParentFrameTranslationMm": (Joint["Origin"][:3, 3] * 1000).tolist(),
        "ParentFrameRpyRadians": Rpy,
        "LocalAxis": Joint["Axis"].tolist(),
        "OriginTransformMetres": Joint["Origin"].tolist(),
        "ZeroPoseOriginInBaseMm": (Frames[Name][:3, 3] * 1000).tolist(),
        "ZeroPoseAxisInBase": (Frames[Name][:3, :3] @ Joint["Axis"]).tolist(),
        "Limits": Joint["Limits"],
    })

Measurements = {
    "SourceCommit": Manifest["Commit"],
    "SourceUrdf": str(Model["Path"].relative_to(OutputRoot)).replace("\\", "/"),
    "Units": "Translations explicitly labelled mm or metres; rotations radians",
    "Calibration": "Official new-calibration URDF; zero is NOT the project's saved home",
    "UpperArmAxisSpacingMm": Upper,
    "ForearmAxisSpacingMm": Forearm,
    "ShoulderPanToLiftOriginDeltaBaseMm": ShoulderDelta.tolist(),
    "WristFlexToRollParentOffsetMm": WristVector.tolist(),
    "RollToToolParentOffsetMm": ToolVector.tolist(),
    "ToolOffsetFromRollAxisMm": ToolRadius,
    "ToolDisplacementOverHalfRollMm": 2 * ToolRadius,
    "Joints": Records,
}
(OutputRoot / "JointMeasurements.json").write_text(Json.dumps(Measurements, indent=2), encoding="utf-8")
with (OutputRoot / "JointTransforms.csv").open("w", encoding="utf-8", newline="") as File:
    Writer = Csv.writer(File)
    Writer.writerow(["Joint", "Parent", "Child", "Xmm", "Ymm", "Zmm", "RollRad", "PitchRad", "YawRad", "AxisX", "AxisY", "AxisZ"])
    for Record in Records:
        Writer.writerow([Record["Name"], Record["Parent"], Record["Child"], *Record["ParentFrameTranslationMm"], *Record["ParentFrameRpyRadians"], *Record["LocalAxis"]])

OriginRows = "\n".join(f"| {Record['Name']} | {Record['Parent']} | {Values(Record['ParentFrameTranslationMm'])} | {Values(Record['ParentFrameRpyRadians'])} |" for Record in Records)
SourceBase = f"https://github.com/TheRobotStudio/SO-ARM100/blob/{Manifest['Commit']}"
Report = f"""# SO-101 CAD and inverse kinematics reference

## What was found

The local project identifies the controlled arm as an SO-101 follower with STS3215 servos. TheRobotStudio publishes the original solid STEP assembly, separate printable part STEP/STL files, and CAD-derived URDF/MuJoCo models. This package preserves those sources and reconstructs the follower mesh assembly from the official URDF. It is a nominal SO-101 design reference; the physical arm's vendor revision and any modified fingers have not been independently confirmed.

![Reconstructed SO-101 follower](SO101Preview.png)

## Open these first

- [Original solid CAD assembly](Original/STEP/SO101/SO101%20Assembly.step): original designer's placement and assembly structure, for a STEP-capable CAD application.
- [Individual STEP parts](Original/STEP/SO101): editable solid bodies for measurements and modifications. Follower jaws are in Follower_Specific. STEP exports do not retain the original sketch/feature history.
- [Individual printable STLs](Original/STL/SO101/Individual): original print meshes, preserved without rescaling.
- [Reconstructed Blender assembly](SO101Reference.blend): separate meshes with the actual URDF joint hierarchy. Select a joint empty and rotate its local Z axis to pose it. Length display is millimetres; internal units are metres. This mesh scene is for inspection and simulation, not a replacement for the solid STEP parts.
- [Portable 3D assembly](SO101Reference.glb): the same URDF reference assembly, in glTF format.
- [Official simulation folder](Original/Simulation/SO101): URDF, MuJoCo XML, and their relative-path mesh dependencies, including old/new calibration variants.
- [Full extracted geometry](JointMeasurements.json) and [joint transform table](JointTransforms.csv).

## Dimensions that matter for this project's IK

All lengths below are millimetres. They are computed from the official URDF transforms, not from an image or a mesh bounding box. Decimal precision reflects the upstream export, not a claim about the manufactured arm's tolerances.

| Quantity | Current project | Official reference | Interpretation |
|---|---:|---:|---|
| Shoulder-lift axis to elbow axis | {CurrentUpper:.3f} | {Upper:.6f} | Official minus current: {Upper - CurrentUpper:+.6f} mm. |
| Elbow axis to wrist-flex axis | {CurrentForearm:.3f} | {Forearm:.6f} | Official minus current: {Forearm - CurrentForearm:+.6f} mm. |
| Shoulder lift height above pan-frame origin | {CurrentShoulderHeight:.3f} | {ShoulderDelta[2]:.6f} | Compare only after establishing the same point along the pan axis. |
| Shoulder offset perpendicular to the lift axis, horizontal at zero | {CurrentShoulderRadial:.3f} | {abs(Joints['shoulder_lift']['Origin'][0, 3]) * 1000:.6f} | The full transform also includes a displacement along the lift axis. |
| Wrist-flex origin to roll origin, point distance | {Segments['wrist_flex_to_wrist_roll']['length_cm'] * 10:.3f} | {Np.linalg.norm(WristVector):.6f} | NOT a replacement scalar link length; includes an offset along the flex axis. |
| Roll origin to official tool frame, point distance | {Segments['wrist_roll_to_gripper_tip']['length_cm'] * 10:.3f} | {Np.linalg.norm(ToolVector):.6f} | The official virtual tool point may differ from the physical fingertip you intend to track. |

The current upper-arm and forearm lengths are already close to nominal. Merely replacing those values will not fix a joint-zero, reference-frame, or tool-point mismatch.

The roll-to-tool translation in the gripper link's own frame is ({Values(ToolVector)}) mm. Its transverse distance from the roll axis is {ToolRadius:.6f} mm. Rotating the wrist through half a turn therefore moves this particular reference point by {2 * ToolRadius:.6f} mm. The current backend forward_joint_positions and frontend forwardKinematics ignore wrist roll in tip position, so they cannot reproduce that motion for the official tool frame.

The wrist-flex-to-roll translation in the wrist link frame is ({Values(WristVector)}) mm. Preserve this vector and the associated rotation. Do not sum its Euclidean norm and the tool-point norm as if both were collinear.

## Exact transforms to use

These translations are expressed in each joint's PARENT link frame. They are not coordinates in one common world frame. Roll/pitch/yaw are radians in the URDF fixed-axis XYZ convention, with rotation matrix Rz(yaw) Ry(pitch) Rx(roll). Every moving joint uses its local Z axis. The fixed tool joint has no moving axis.

| Joint/frame | Parent | Translation XYZ, mm | RPY, radians |
|---|---|---|---|
{OriginRows}

For each moving joint, compose parent-to-world, joint-origin transform, then rotation about the local joint axis. Use the fixed gripper_frame_joint transform at the end. JointMeasurements.json includes full homogeneous origin matrices in metres as well as all joint origins and axes expressed in the common base frame at zero pose.

## Applying this to the existing controller

1. Use the official new-calibration URDF chain as the geometry source for both frontend and backend. The current parameters are duplicated in back end/arm_geometry.json and AI Controlled WebApp/src/lib/robotModel.ts.
2. Determine a signed angle offset for each physical servo against the URDF reference pose. Your saved home_preset.json is an arbitrary home, and the firmware position corrections have changed locally. A zero GUI angle is not necessarily a zero URDF angle. Preserve joint limits and mechanical angle branches when unwrapping raw encoder readings.
3. Establish one rigid base-to-table transform. The URDF base_link and the project's shoulder-pan/table/display frames are different. The explicit pan-origin-to-lift-origin height above is not directly comparable until that origin is aligned.
4. Choose the actual tool center point. The official gripper_frame_link is fixed to the gripper body. It is not guaranteed to be the contact point of a modified finger, a camera marker, or the moving midpoint between jaws. If the desired point depends on jaw opening, model that dependency.
5. Include wrist roll in forward kinematics, because the official tool point is off the roll axis. A position-plus-pitch IK target also needs an explicit roll policy, such as retaining measured wrist roll.
6. After replacing geometry, re-check camera/table calibration and verify positions over several separated poses. Nominal CAD does not remove encoder-zero error, printed-part tolerances, backlash, servo compliance, or physical TCP mismatch.

The active controller and its calibration files were not modified by this CAD task. The scripts supplied here are offline and do not connect to the arm.

## Verification

- Downloaded source files were checked against the pinned Git blob hashes. SourceManifest.json also records SHA-256 checksums.
- Reference FK was checked at {Verification['PoseCount']} poses, including zero, limit corners, and seeded random poses, against MuJoCo's independently imported URDF.
- Maximum joint-origin disagreement with MuJoCo's URDF importer: {Verification['MaximumUrdfJointErrorMm']:.3e} mm.
- Maximum disagreement with the separately exported official MuJoCo XML: {Verification['MaximumMjcfJointErrorMm']:.6f} mm. This small difference is consistent with the rounded Euler angles in the URDF versus quaternion rotations in the XML.
- Rotation convention was checked against SciPy. Tests cover the tool transform, constant link lengths, wrist-roll tool movement, gripper opening independence of the fixed tool frame, and invalid input handling.
- The reconstructed mesh assembly was rendered and visually inspected. The Blender importer removed one duplicate triangle; original source meshes remain unchanged.
- All {len(PartChecks)} individual printed-part STEP files pass Open CASCADE solid validity checks. The original full assembly imports with {AssemblyCheck['SolidCount']} solids, of which {len(AssemblyCheck['InvalidSolidIndices'])} fail solid validity checks. Those source defects remain unchanged; use the individually validated part files for part editing. CadVerification.json records the details. Original assembly data and the simulation URDF are separate upstream exports and may use different part revisions. The validated FK result applies to the URDF model; it does not certify the physical arm or prove exact agreement between every STEP part and the URDF.

## Reproduce locally

The build scripts are also in the project's Cad folder. ReferenceKinematics.py depends only on NumPy; its default invocation reports the official zero pose. Optional JSON input maps the original joint names to angles in radians. TestReferenceKinematics.py additionally requires MuJoCo and SciPy. BuildReferenceScene.py runs with Blender. VerifyCad.py uses Open CASCADE directly. Run build scripts from the original project, since the measurement comparison reads its current geometry file. Copies under Scripts are included for reference.

Script variables and functions use PascalCase. Original upstream file and joint names, and external API names such as bpy and NumPy methods, retain their required spelling so the source files and simulators remain compatible.

## Sources

- [Original SO-101 project]({SourceBase}/README.md)
- [Original STEP assembly]({SourceBase}/STEP/SO101/SO101%20Assembly.step)
- [CAD-derived URDF]({SourceBase}/Simulation/SO101/so101_new_calib.urdf)
- [Calibration and simulation documentation]({SourceBase}/Simulation/SO101/README.md)
- [Upstream Onshape source linked by the simulation export](https://cad.onshape.com/documents/7715cc284bb430fe6dab4ffd/w/4fd0791b683777b02f8d975a/e/826c553ede3b7592eb9ca800)

Pinned repository commit: {Manifest['Commit']}. See Original/LICENSE for the upstream Apache 2.0 license. The original downloaded files are byte-preserved. The Blender/GLB assemblies, reports, and helper scripts are generated additions to this package.
"""
(OutputRoot / "ReadMe.md").write_text(Report, encoding="utf-8")
ScriptsRoot = OutputRoot / "Scripts"
ScriptsRoot.mkdir(exist_ok=True)
for File in (ProjectRoot / "Cad").glob("*.py"):
    Shutil.copy2(File, ScriptsRoot / File.name)
Shutil.copy2(ProjectRoot / "Cad" / "Requirements.txt", ScriptsRoot / "Requirements.txt")
Shutil.copy2(ProjectRoot / "Cad" / "UpstreamTree.json", ScriptsRoot / "UpstreamTree.json")
print(Json.dumps({Key: Value for Key, Value in Measurements.items() if Key != "Joints"}, indent=2))
