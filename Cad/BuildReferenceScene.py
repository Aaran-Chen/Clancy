"""Run with Blender in background to reconstruct the original URDF assembly."""

import json as Json
from pathlib import Path
import sys as Sys
import math as Math

import bpy as Bpy
from mathutils import Matrix, Vector

Sys.path.insert(0, str(Path(__file__).resolve().parent))
from ReferenceKinematics import LoadModel, OriginTransform, OutputRoot

Model = LoadModel()
Bpy.ops.object.select_all(action="SELECT")
Bpy.ops.object.delete(use_global=False)
Scene = Bpy.context.scene
Scene.unit_settings.system = "METRIC"
Scene.unit_settings.length_unit = "MILLIMETERS"
Scene.unit_settings.scale_length = 1.0


def Material(Name, Color, Metallic=0.0):
    Result = Bpy.data.materials.new(Name)
    Result.diffuse_color = (*Color, 1.0)
    Result.use_nodes = True
    Shader = Result.node_tree.nodes.get("Principled BSDF")
    Shader.inputs["Base Color"].default_value = (*Color, 1.0)
    Shader.inputs["Metallic"].default_value = Metallic
    Shader.inputs["Roughness"].default_value = 0.35
    return Result


Printed = Material("PrintedParts", (0.035, 0.42, 0.36))
Servo = Material("ServoHousings", (0.018, 0.025, 0.038), 0.18)
Controller = Material("ControllerMount", (0.10, 0.15, 0.23))


def Empty(Name, Parent=None, Transform=None):
    Result = Bpy.data.objects.new(Name, None)
    Scene.collection.objects.link(Result)
    Result.empty_display_type = "PLAIN_AXES"
    Result.empty_display_size = 0.018
    Result.parent = Parent
    if Transform is not None:
        Result.matrix_local = Matrix(Transform.tolist())
    return Result


Links = {Model["Root"]: Empty("BaseFrame")}
Links[Model["Root"]]["Reference"] = "Official URDF base_link; metres internally, millimetres displayed"
Pending = list(Model["Joints"])
while Pending:
    Ready = [Joint for Joint in Pending if Joint["Parent"] in Links]
    if not Ready:
        raise ValueError("Broken joint hierarchy")
    for Joint in Ready:
        JointFrame = Empty(Joint["Name"] + "Origin", Links[Joint["Parent"]], Joint["Origin"])
        Motion = Empty(Joint["Name"], JointFrame)
        Motion["JointType"] = Joint["Type"]
        Motion["UrdfChildLink"] = Joint["Child"]
        Motion["Usage"] = "Rotate local Z for URDF joint angle; do not use app GUI or raw servo angles"
        Motion.rotation_mode = "XYZ"
        for Key, Value in Joint["Limits"].items():
            Motion[Key] = float(Value)
        Links[Joint["Child"]] = Motion
        Pending.remove(Joint)

MeshCount = 0
for Link in Model["Xml"].findall("link"):
    for Index, Visual in enumerate(Link.findall("visual")):
        Mesh = Visual.find("geometry/mesh")
        if Mesh is None:
            continue
        Source = Model["Path"].parent / Mesh.get("filename")
        Bpy.ops.wm.stl_import(filepath=str(Source))
        Object = Bpy.context.object
        Object.name = Source.stem + "_" + Link.get("name")
        Object.parent = Links[Link.get("name")]
        Object.matrix_local = Matrix(OriginTransform(Visual.find("origin")).tolist())
        Scale = [float(Value) for Value in Mesh.get("scale", "1 1 1").split()]
        Object.scale = Scale
        Object["SourceFile"] = str(Source.relative_to(OutputRoot))
        Object.data.materials.clear()
        Object.data.materials.append(Servo if "sts3215" in Source.name else Controller if "waveshare" in Source.name else Printed)
        MeshCount += 1

RobotObjects = list(Scene.objects)
for Object in RobotObjects:
    Object.select_set(True)
Bpy.ops.export_scene.gltf(filepath=str(OutputRoot / "SO101Reference.glb"), export_format="GLB", use_selection=True)

Bpy.context.view_layer.update()
Corners = [Object.matrix_world @ Vector(Corner) for Object in RobotObjects if Object.type == "MESH" for Corner in Object.bound_box]
Minimum = Vector(tuple(min(Corner[Axis] for Corner in Corners) for Axis in range(3)))
Maximum = Vector(tuple(max(Corner[Axis] for Corner in Corners) for Axis in range(3)))
Center = (Minimum + Maximum) / 2

Bpy.ops.mesh.primitive_plane_add(size=200, location=(0, 0, Minimum.z - 0.0005))
Floor = Bpy.context.object
Floor.name = "PreviewFloor"
Floor.data.materials.append(Material("Floor", (0.12, 0.15, 0.20)))

Bpy.ops.object.camera_add(location=Center + Vector((0.42, -0.68, 0.34)))
Camera = Bpy.context.object
Camera.name = "PreviewCamera"
Camera.rotation_euler = (Center - Camera.location).to_track_quat("-Z", "Y").to_euler()
Camera.data.type = "ORTHO"
Camera.data.ortho_scale = max(Maximum - Minimum) * 1.42
Scene.camera = Camera

for Name, Offset, Energy, Size in (
    ("KeyLight", (0.1, -0.45, 0.75), 55, 0.5),
    ("FillLight", (-0.3, -0.15, 0.4), 30, 0.4),
    ("RimLight", (0.25, 0.5, 0.5), 65, 0.4),
):
    Bpy.ops.object.light_add(type="AREA", location=Center + Vector(Offset))
    Light = Bpy.context.object
    Light.name = Name
    Light.data.energy = Energy
    Light.data.shape = "DISK"
    Light.data.size = Size
    Light.rotation_euler = (Center - Light.location).to_track_quat("-Z", "Y").to_euler()

Scene.world.color = (0.25, 0.25, 0.25)
Scene.render.engine = "CYCLES"
Scene.cycles.samples = 48
Scene.cycles.use_denoising = True
Scene.render.resolution_x = 1600
Scene.render.resolution_y = 1100
Scene.render.resolution_percentage = 100
Scene.render.image_settings.file_format = "PNG"
Scene.render.filepath = str(OutputRoot / "SO101Preview.png")
Scene.view_settings.view_transform = "AgX"
for Screen in Bpy.data.screens:
    for Area in Screen.areas:
        if Area.type == "VIEW_3D":
            Area.spaces.active.region_3d.view_distance = 0.7
            Area.spaces.active.region_3d.view_location = Center
            Area.spaces.active.clip_start = 0.001

Bpy.ops.object.select_all(action="DESELECT")
Bpy.ops.wm.save_as_mainfile(filepath=str(OutputRoot / "SO101Reference.blend"))
Bpy.ops.render.render(write_still=True)
(OutputRoot / "SceneVerification.json").write_text(Json.dumps({
    "VisualMeshCount": MeshCount,
    "JointNames": [Joint["Name"] for Joint in Model["Joints"]],
    "Pose": "Official new-calibration URDF, all joint angles zero",
    "BoundsMm": {"Minimum": [Value * 1000 for Value in Minimum], "Maximum": [Value * 1000 for Value in Maximum]},
    "Purpose": "Reconstructed mesh assembly for inspection; original STEP supplies solid CAD",
}, indent=2), encoding="utf-8")
print(f"Verified {MeshCount} URDF visuals; saved Blender, GLB and rendered preview.", flush=True)
