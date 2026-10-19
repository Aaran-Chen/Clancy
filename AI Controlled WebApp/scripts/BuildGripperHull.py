"""Write src/lib/GripperHull.json: the convex hull of the gripper and moving jaw.

The AI floor rule (src/lib/AiTableFloor.ts) holds the lowest point of the
gripper above the desk. The collision boxes in back end/CollisionModel.json
reach up to 2 cm below the real jaw at a flat wrist, which would refuse most
desk grasps, so the rule uses the mesh itself. The lowest point of a rigid
part in any pose is a vertex of its convex hull, so the hull is exact.

Run after the CAD assets change:  python scripts/BuildGripperHull.py
"""
import hashlib as Hashlib
import json as Json
from pathlib import Path

import numpy as Np
import trimesh as Trimesh

Root = Path(__file__).resolve().parents[2]
Asset = Root / "AI Controlled WebApp/public/Cad/SO101Parts.glb"
Links = ("gripper_link", "moving_jaw_so101_v1_link")
Scene = Trimesh.load(Asset, force="scene")
Model = Json.loads((Root / "back end/CadModel.json").read_text())
Collision = Json.loads((Root / "back end/CollisionModel.json").read_text())
Points = {Link: [] for Link in Links}
for Visual in Model["Visuals"]:
    if Visual["Link"] not in Links:
        continue
    # Same placement as Cad/PrepareCollisionModel.py: each visual in its link's frame, in cm.
    Transform, Geometry = Scene.graph[Visual["Name"]]
    Mesh = Scene.geometry[Geometry].copy()
    Mesh.apply_transform(Transform)
    Points[Visual["Link"]].append(Np.asarray(Mesh.vertices) * 100)
Hulls = {}
for Link, Parts in Points.items():
    # A few thousand vertices (the parts are curved); checking them all takes well under a millisecond.
    Vertices = Trimesh.convex.convex_hull(Np.vstack(Parts)).vertices
    # The link's collision boxes were built from the same meshes: the hull must lie inside them.
    Boxes = [Box["BoundsCm"] for Box in Collision["ArmBoxes"] if Box["Link"] == Link]
    Low = Np.min([Box[0] for Box in Boxes], axis=0)
    High = Np.max([Box[1] for Box in Boxes], axis=0)
    if (Vertices < Low - 1e-3).any() or (Vertices > High + 1e-3).any():
        raise ValueError(f"{Link} hull falls outside its collision boxes: the mesh placement changed")
    Hulls[Link] = Np.round(Vertices, 3).tolist()
Output = {"Version": "gripper-hull-v1", "Units": "cm", "Frame": "link",
          "SourceSha256": Hashlib.sha256(Asset.read_bytes()).hexdigest(), "Links": Hulls}
Path(__file__).resolve().parents[1].joinpath("src/lib/GripperHull.json").write_text(Json.dumps(Output, separators=(",", ":")) + "\n")
print({Link: len(Vertices) for Link, Vertices in Hulls.items()})
