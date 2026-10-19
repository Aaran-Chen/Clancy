"""Bundle the local XLeRobot cart/head CAD and mobile base for the GUI.

This is display geometry only. It never imports the hardware bridge or changes IK.
"""
import argparse as Argparse
import hashlib as Hashlib
import json as Json
from pathlib import Path
import shutil as Shutil
import xml.etree.ElementTree as Xml

import mujoco as Mj
import numpy as Np
import trimesh as Trimesh

from ReferenceKinematics import OriginTransform

Root = Path(__file__).resolve().parents[1]
Parser = Argparse.ArgumentParser(description=__doc__)
Parser.add_argument("--source", dest="Source", type=Path, default=Root.parent / "XLeRobot-main")
Arguments = Parser.parse_args()
SourceRoot = Arguments.Source.resolve()
UrdfPath = SourceRoot / "simulation/Maniskill/assets/xlerobot/xlerobot.urdf"
MobilePath = SourceRoot / "simulation/mujoco/xlerobot.xml"
AssetRoot = Root / "AI Controlled WebApp/public/Cad"
AssetRoot.mkdir(parents=True, exist_ok=True)
Scene = Trimesh.Scene()
Parts, Sources = [], {}
Rotation = Np.eye(4)
Rotation[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]


def RecordSource(File):
    Sources[str(File.relative_to(SourceRoot)).replace("\\", "/")] = Hashlib.sha256(File.read_bytes()).hexdigest()


def AddMesh(Name, Mesh, Material, Category, Source):
    Mesh.apply_transform(Rotation)
    assert Np.isfinite(Mesh.vertices).all() and len(Mesh.faces) > 0
    Scene.add_geometry(Mesh, node_name=Name, geom_name=Name)
    Parts.append({"Name": Name, "Material": Material, "Category": Category, "Source": Source,
                  "BoundsCm": Mesh.bounds.tolist(), "Triangles": len(Mesh.faces)})


Tree = Xml.parse(UrdfPath).getroot()
RecordSource(UrdfPath)
Links = {"root": Np.eye(4)}
Pending = list(Tree.findall("joint"))
while Pending:
    Count = len(Pending)
    for Joint in Pending[:]:
        Parent = Joint.find("parent").get("link")
        if Parent in Links:
            Links[Joint.find("child").get("link")] = Links[Parent] @ OriginTransform(Joint.find("origin"))
            Pending.remove(Joint)
    assert len(Pending) < Count, "Unresolved XLeRobot joint hierarchy"

for LinkName in ("base_link", "top_base_link", "head_pan_link", "head_tilt_link"):
    Link = next(Link for Link in Tree.findall("link") if Link.get("name") == LinkName)
    for Index, Visual in enumerate(Link.findall("visual")):
        MeshElement = Visual.find("geometry/mesh")
        Source = UrdfPath.parent / MeshElement.get("filename")
        RecordSource(Source)
        Mesh = Trimesh.load(Source, force="mesh")
        Mesh.apply_scale([float(Value) for Value in MeshElement.get("scale", "1 1 1").split()])
        Mesh.apply_transform(Links[LinkName] @ OriginTransform(Visual.find("origin")))
        Mesh.apply_scale(100)
        Category = "Cart" if LinkName == "base_link" else "Mast" if LinkName == "top_base_link" else "Head"
        Material = "Cart" if LinkName == "base_link" and Index == 0 else "Metal" if LinkName == "base_link" and Index == 1 else "Dark" if Visual.find("material").get("name") == "motor" else "Shell"
        AddMesh("XLe" + LinkName + str(Index), Mesh, Material, Category, str(Source.relative_to(SourceRoot)).replace("\\", "/"))

# The companion MuJoCo model supplies the original Lekiwi drive platform.
# Compiled mesh vertices and geom transforms preserve MuJoCo's mesh recentering.
RecordSource(MobilePath)
MobileTree = Xml.parse(MobilePath).getroot()
for Mesh in MobileTree.findall("asset/mesh"):
    if Mesh.get("file"):
        RecordSource(MobilePath.parent / "assets" / Mesh.get("file"))
Model = Mj.MjModel.from_xml_path(str(MobilePath))
Data = Mj.MjData(Model)
Mj.mj_forward(Model, Data)
BaseId = Mj.mj_name2id(Model, Mj.mjtObj.mjOBJ_BODY, "base_plate_layer1-v5-1")
for Index in range(Model.ngeom):
    BodyId = int(Model.geom_bodyid[Index])
    while BodyId and BodyId != BaseId:
        BodyId = int(Model.body_parentid[BodyId])
    if BodyId != BaseId or Model.geom_type[Index] != Mj.mjtGeom.mjGEOM_MESH:
        continue
    MeshId = int(Model.geom_dataid[Index])
    VertexStart, FaceStart = int(Model.mesh_vertadr[MeshId]), int(Model.mesh_faceadr[MeshId])
    Vertices = Model.mesh_vert[VertexStart:VertexStart + Model.mesh_vertnum[MeshId]].copy()
    Faces = Model.mesh_face[FaceStart:FaceStart + Model.mesh_facenum[MeshId]].copy()
    Mesh = Trimesh.Trimesh(vertices=Vertices, faces=Faces, process=False)
    Transform = Np.eye(4)
    Transform[:3, :3] = Data.geom_xmat[Index].reshape(3, 3)
    Transform[:3, 3] = Data.geom_xpos[Index]
    Mesh.apply_transform(Transform)
    Mesh.apply_scale(100)
    Name = Mj.mj_id2name(Model, Mj.mjtObj.mjOBJ_GEOM, Index)
    Material = "Metal" if "Hub" in Name or "Standoff" in Name else "Rubber" if "Wheel" in Name else "Dark"
    AddMesh("XLeDrive" + str(Index), Mesh, Material, "Drive", "simulation/mujoco/xlerobot.xml#" + Name)

# A generic power pack and computer indicate onboard equipment. These are
# intentionally schematic, not dimensions attributed to a particular product.
for Name, Size, Center, Material in (
    ("PowerPack", (16, 18, 19), (0, 0, 32), "Power"),
    ("PowerHandle", (10, 1.8, 1.8), (0, 0, 43), "Dark"),
    ("PowerHandleLeft", (1.8, 1.8, 3), (-4.1, 0, 41.5), "Dark"),
    ("PowerHandleRight", (1.8, 1.8, 3), (4.1, 0, 41.5), "Dark"),
    ("Computer", (9, 6, 3), (0, 7, 53), "Dark"),
):
    Mesh = Trimesh.creation.box(extents=Size)
    Mesh.apply_translation(Center)
    AddMesh("XLe" + Name, Mesh, Material, "SchematicEquipment", "Schematic placeholder")

CadModel = Json.loads((Root / "back end/CadModel.json").read_text())
PanOrigin = next(Joint for Joint in CadModel["Joints"] if Joint["Name"] == "shoulder_pan")
PanVector = Np.array([PanOrigin["Origin"][Index][3] for Index in range(3)])
Mounts = {}
for Side, LinkName in (("Right", "Base"), ("Left", "Base_2")):
    # SO-101 forward is +X in its original CAD frame. The old SO-100 Base
    # frame uses another orientation, so retain the upstream mount location
    # and align the replacement SO-101 forward with the cart's +X direction.
    Origin = Links[LinkName][:3, 3] * 100
    Mounts[Side] = {"RootCm": (Rotation[:3, :3] @ Origin).tolist(),
                    "PanCm": (Rotation[:3, :3] @ (Origin + PanVector)).tolist()}
Scene.export(AssetRoot / "XLeRobotBody.glb")
Shutil.copy2(SourceRoot / "LICENSE", AssetRoot / "XLeRobotLicense.txt")
Description = {
    "Name": "XLeRobot with IKEA RASKOG cart", "Units": "cm", "DefaultLiveArm": "Right",
    "SourceRepository": "https://github.com/Vector-Wangel/XLeRobot",
    "SourceDirectory": str(SourceRoot), "SourceHashes": Sources, "Parts": Parts,
    "Mounts": Mounts, "BoundsCm": Scene.bounds.tolist(), "FloorZCm": float(Scene.bounds[0, 2]),
    "ParkedUrdfRadians": {"shoulder_pan": 0, "shoulder_lift": -1.3, "elbow_flex": 1.45,
                           "wrist_flex": 0.8, "wrist_roll": 0, "gripper": 0.15},
    "Note": "Body, head, cart and drive meshes come from local upstream simulation assets. SO-101 mounting is a visual adaptation. Power pack/computer are schematic. The other arm and head are parked references, not live telemetry. No IK or hardware frame changes."
}
(Root / "AI Controlled WebApp/src/lib/XLeRobotScene.json").write_text(Json.dumps(Description, indent=2) + "\n")
(AssetRoot / "XLeRobotSource.json").write_text(Json.dumps(Description, indent=2) + "\n")
print(Json.dumps({"Parts": len(Parts), "BoundsCm": Scene.bounds.tolist(), "Mounts": Mounts}, indent=2))
