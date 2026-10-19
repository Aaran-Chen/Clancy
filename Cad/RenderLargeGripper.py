"""Render review images of the large gripper with Blender (run by PrepareLargeGripper.py).

  & "C:\\Program Files\\Blender Foundation\\Blender 5.1\\blender.exe" -b --python Cad\\RenderLargeGripper.py
Reads the STLs and RenderPoses.json that PrepareLargeGripper.py wrote; writes PNGs next to them.
"""
import json as Json
import math as Math
from pathlib import Path

import bpy as Bpy
from mathutils import Matrix, Vector

Output = Path.home() / "Aaran 3D Prints/SO101/Large Gripper"
SourceRoot = Path.home() / "Downloads/SO101Cad/Original"
Poses = Json.loads((Output / "RenderPoses.json").read_text())
Colours = {"Body": (0.86, 0.86, 0.84, 1), "Jaw": (0.95, 0.45, 0.12, 1), "Servo": (0.12, 0.12, 0.13, 1),
           "StockBody": (0.35, 0.55, 0.85, 1), "StockJaw": (0.35, 0.55, 0.85, 1), "Object": (0.15, 0.65, 0.35, 1)}


def Clear():
    for Object in list(Bpy.data.objects):
        Bpy.data.objects.remove(Object, do_unlink=True)


def Material(Name):
    Item = Bpy.data.materials.get(Name) or Bpy.data.materials.new(Name)
    Item.diffuse_color = Colours[Name]
    return Item


def Import(File, Name, Placement=None):
    Bpy.ops.wm.stl_import(filepath=str(File))
    Object = Bpy.context.object
    Object.data.materials.clear()
    Object.data.materials.append(Material(Name))
    if Placement is not None:
        Object.matrix_world = Matrix(Placement)
    return Object


def Cylinder(Diameter, CentreXZ, Length=110):
    Bpy.ops.mesh.primitive_cylinder_add(vertices=96, radius=Diameter / 2, depth=Length,
                                        location=(CentreXZ[0], 0, CentreXZ[1]), rotation=(Math.pi / 2, 0, 0))
    Object = Bpy.context.object
    Object.data.materials.append(Material("Object"))
    return Object


def Scene(Size=(1100, 1100)):
    Settings = Bpy.context.scene
    Settings.render.engine = "BLENDER_WORKBENCH"
    Settings.display.shading.light = "STUDIO"
    Settings.display.shading.color_type = "MATERIAL"
    Settings.display.shading.show_cavity = True
    Settings.display.shading.show_shadows = False
    Settings.display.shading.background_type = "WORLD"
    Settings.world.color = (0.045, 0.055, 0.07)
    Settings.render.resolution_x, Settings.render.resolution_y = Size
    Settings.render.resolution_percentage = 100
    Settings.render.image_settings.file_format = "PNG"
    return Settings


def Shoot(File, Direction, Centre=(10, 0, 65), Scale=230, XRay=False, Up="Y"):
    Settings = Bpy.context.scene
    Settings.display.shading.show_xray = XRay
    Settings.display.shading.xray_alpha = 0.55
    Bpy.ops.object.camera_add()
    Camera = Bpy.context.object
    Camera.data.type = "ORTHO"
    Camera.data.ortho_scale = Scale
    Camera.data.clip_end = 5000
    Camera.location = Vector(Centre) + Vector(Direction).normalized() * 800
    Camera.rotation_euler = (Vector(Centre) - Camera.location).to_track_quat("-Z", Up).to_euler()
    Settings.camera = Camera
    Settings.render.filepath = str(Output / File)
    Bpy.ops.render.render(write_still=True)
    Bpy.data.objects.remove(Camera, do_unlink=True)


def Gripper(Angle, Prefix="", Body="LargeGripperFixedJawBody.stl", Jaw="LargeGripperMovingJaw.stl", Folder=Output):
    Import(Folder / Body, Prefix + "Body")
    Import(Folder / Jaw, Prefix + "Jaw", Poses["JawToBody"][Angle])
    Import(SourceRoot / "Simulation/SO101/assets/sts3215_03a_v1.stl", "Servo", Poses["ServoInBodyMetres"])


Scene()
Closed, Open = str(Poses["Closed"]), str(Poses["Open"])
Front, Iso = (0, -1, 0), (0.9, -1.5, 0.7)
Clear(); Gripper(Closed); Shoot("LargeGripperClosed.png", Iso); Shoot("LargeGripperClosedFront.png", Front)
Clear(); Gripper(Open); Shoot("LargeGripperOpen.png", Iso, Centre=(30, 0, 55), Scale=260)
Shoot("LargeGripperOpenFront.png", Front, Centre=(30, 0, 55), Scale=260)
for Diameter, Grasp in Poses["Grasps"].items():
    Clear(); Gripper(str(Grasp["Angle"])); Cylinder(float(Diameter), Grasp["CentreXZ"])
    Shoot(f"Grasp{Diameter}mmFront.png", Front)
    if Diameter == "66":
        Shoot("LargeGripperCan66.png", Iso)
# Stock comparison: same scale, front view; then an x-ray overlay of both.
Clear(); Gripper(Closed, "Stock", "Wrist_Roll_Follower_SO101.stl", "Moving_Jaw_SO101.stl", SourceRoot / "STL/SO101/Individual")
Shoot("StockClosedFront.png", Front)
Gripper(Closed)
Shoot("OverlayStockFront.png", Front, XRay=True)
