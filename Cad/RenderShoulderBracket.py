"""Render a recognition preview of the original bracket with Blender."""
from pathlib import Path
import bpy as Bpy
from mathutils import Vector

Output = Path.home() / "Downloads/SO101ShoulderBracket"
Bpy.ops.object.select_all(action="SELECT")
Bpy.ops.object.delete(use_global=False)
Bpy.ops.wm.stl_import(filepath=str(Output / "ShoulderMotorBracket.stl"), global_scale=0.001)
Part = Bpy.context.object
Part.name = "ShoulderMotorBracket"
Corners = [Part.matrix_world @ Vector(Corner) for Corner in Part.bound_box]
Minimum = Vector(tuple(min(Corner[Axis] for Corner in Corners) for Axis in range(3)))
Maximum = Vector(tuple(max(Corner[Axis] for Corner in Corners) for Axis in range(3)))
Center = (Minimum + Maximum) / 2
Size = max(Maximum - Minimum)

Material = Bpy.data.materials.new("Bracket")
Material.diffuse_color = (0.06, 0.48, 0.35, 1)
Part.data.materials.append(Material)
Bpy.ops.object.camera_add(location=Center + Vector((1.2, -1.4, 1.1)) * Size)
Camera = Bpy.context.object
Camera.rotation_euler = (Center - Camera.location).to_track_quat("-Z", "Y").to_euler()
Camera.data.type = "ORTHO"
Camera.data.ortho_scale = Size * 1.95
Camera.data.clip_start = 0.0001
Camera.data.clip_end = 10
Scene = Bpy.context.scene
Scene.camera = Camera
Scene.render.engine = "BLENDER_WORKBENCH"
Scene.display.shading.light = "STUDIO"
Scene.display.shading.studiolight_rotate_z = 0.4
Scene.display.shading.color_type = "MATERIAL"
Scene.display.shading.show_shadows = False
Scene.display.shading.show_cavity = False
Scene.display.shading.cavity_type = "BOTH"
Scene.display.shading.background_type = "WORLD"
Scene.world.color = (0.045, 0.055, 0.07)
Scene.render.resolution_x = 1000
Scene.render.resolution_y = 800
Scene.render.resolution_percentage = 100
Scene.render.image_settings.file_format = "PNG"
Scene.render.filepath = str(Output / "ShoulderMotorBracket.png")
Bpy.ops.render.render(write_still=True)
