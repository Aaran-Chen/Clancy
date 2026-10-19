"""Build local, joint-local CAD meshes and one shared kinematic description."""
import json as Json
from pathlib import Path
import shutil as Shutil

import numpy as Np
import trimesh as Trimesh

from ReferenceKinematics import ForwardKinematics, JointOrder, LoadModel, OriginTransform, OutputRoot

Root = Path(__file__).resolve().parents[1]
Model = LoadModel()
AssetRoot = Root / "AI Controlled WebApp" / "public" / "Cad"
AssetRoot.mkdir(parents=True, exist_ok=True)
Scene = Trimesh.Scene()
Visuals = []
for Link in Model["Xml"].findall("link"):
    for Index, Visual in enumerate(Link.findall("visual")):
        Source = Model["Path"].parent / Visual.find("geometry/mesh").get("filename")
        Mesh = Trimesh.load(Source, force="mesh")
        Mesh.apply_transform(OriginTransform(Visual.find("origin")))
        Name = Link.get("name") + "__" + str(Index)
        Scene.add_geometry(Mesh, node_name=Name, geom_name=Name)
        Visuals.append({"Name": Name, "Link": Link.get("name"), "Servo": "sts3215" in Source.name})
Scene.export(AssetRoot / "SO101Parts.glb")
Shutil.copy2(OutputRoot / "Original/LICENSE", AssetRoot / "LICENSE.txt")
Shutil.copy2(Model["Path"], AssetRoot / "SO101.urdf")
CadToModel = Np.eye(4)
CadToModel[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
Pan = next(Joint for Joint in Model["Joints"] if Joint["Name"] == "shoulder_pan")
CadToModel[:3, 3] = -(CadToModel[:3, :3] @ Pan["Origin"][:3, 3]) * 100
Data = {"Name": "SO-101 follower", "SourceCommit": Json.loads((OutputRoot / "SourceManifest.json").read_text())["Commit"],
        "Source": "https://github.com/TheRobotStudio/SO-ARM100", "Units": "cm", "Root": Model["Root"],
        "ToolLink": "gripper_frame_link", "CadToModel": CadToModel.tolist(), "Visuals": Visuals, "Joints": []}
for Name in (*JointOrder, "gripper_frame_joint"):
    Joint = next(Joint for Joint in Model["Joints"] if Joint["Name"] == Name)
    Origin = Joint["Origin"].copy()
    Origin[:3, 3] *= 100
    Data["Joints"].append({"Name": Name, "Parent": Joint["Parent"], "Child": Joint["Child"], "Type": Joint["Type"],
                           "Origin": Origin.tolist(), "Axis": Joint["Axis"].tolist(), "Limits": Joint["Limits"]})
(Root / "back end/CadModel.json").write_text(Json.dumps(Data, indent=2) + "\n")
AlignmentPath = Root / "back end/CadAlignment.json"
if not AlignmentPath.exists():
    Links, Frames = ForwardKinematics(Model)
    Upper = Frames["elbow_flex"][:3, 3] - Frames["shoulder_lift"][:3, 3]
    Forearm = Frames["wrist_flex"][:3, 3] - Frames["elbow_flex"][:3, 3]
    UpperAngle = Np.degrees(Np.arctan2(Upper[2], Upper[0]))
    ForearmAngle = Np.degrees(Np.arctan2(Forearm[2], Forearm[0]))
    Direction = Links["gripper_frame_link"][:3, 2]
    Pitch = Np.degrees(Np.arctan2(Direction[2], Direction[0]))
    Old = Json.loads((Root / "back end/arm_geometry.json").read_text())
    Segments = {Segment["name"]: Segment for Segment in Old["segments"]}
    Shoulder = float(UpperAngle - Segments["shoulder_lift_to_elbow"]["angle_degrees_up"])
    Elbow = float(ForearmAngle - Segments["elbow_to_wrist_flex"]["angle_degrees_up"] - Shoulder)
    Wrist = float(Pitch - Segments["wrist_flex_to_wrist_roll"]["angle_degrees_up"] - Shoulder - Elbow)
    Raw = Json.loads((Root / "back end/home_preset.json").read_text())
    Alignment = {"Version": 1, "Verified": False,
                 "Note": "CAD dimensions are original. Initial servo alignment preserves the previous home link orientations and remains physically unverified.",
                 "ReferenceRaw": Raw, "AnchorRaw": Raw,
                 "UrdfAtReferenceDegrees": dict(zip(JointOrder, (0, Shoulder, Elbow, Wrist, 0, 0))),
                 "Direction": dict(zip(JointOrder, (-1, -1, -1, -1, -1, 1)))}
    AlignmentPath.write_text(Json.dumps(Alignment, indent=2) + "\n")
    print(Json.dumps(Alignment, indent=2))
print(f"Prepared {len(Visuals)} exact CAD mesh instances and shared transforms")
