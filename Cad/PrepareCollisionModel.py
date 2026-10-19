"""Build arm envelopes, fixed-obstacle surfaces and the rearward workspace limit.

Run:  python Cad/PrepareCollisionModel.py [output folder]   (default: back end)
"""
import hashlib as Hashlib
import json as Json
import sys as Sys
from pathlib import Path

import numpy as Np
import trimesh as Trimesh

Root = Path(__file__).resolve().parents[1]
# Another folder rebuilds without touching the live files (TestBasketSurface compares them).
Output = Path(Sys.argv[1]) if len(Sys.argv) > 1 else Root / "back end"
Asset = Root / "AI Controlled WebApp/public/Cad/SO101Parts.glb"
Scene = Trimesh.load(Asset, force="scene")
Model = Json.loads((Root / "back end/CadModel.json").read_text())
Assembly = Json.loads((Root / "AI Controlled WebApp/src/lib/XLeRobotScene.json").read_text())
Boxes = []
for Visual in Model["Visuals"]:
    if Visual["Link"] == Model["Root"]:
        continue  # Fixed mounting bracket intentionally attaches to the cart.
    Transform, Geometry = Scene.graph[Visual["Name"]]
    Mesh = Scene.geometry[Geometry].copy()
    Mesh.apply_transform(Transform)
    Boxes.append({"Name": Visual["Name"], "Link": Visual["Link"], "BoundsCm": (Mesh.bounds * 100).tolist()})
Cart = next(Part for Part in Assembly["Parts"] if Part["Name"] == "XLebase_link0")
BasketAsset = Root / "AI Controlled WebApp/public/Cad/XLeRobotBody.glb"
BasketScene = Trimesh.load(BasketAsset, force="scene")
Transform, Geometry = BasketScene.graph[Cart["Name"]]
Basket = BasketScene.geometry[Geometry].copy()
Basket.apply_transform(Transform)  # XLeRobotBody.glb is already in assembly cm.
Np.testing.assert_allclose(Basket.bounds, Cart["BoundsCm"], atol=1e-4)
Parts = Basket.split(only_watertight=False)
Vertices, Faces, Components, Solids = [], [], [], []
for Index, Part in enumerate(Parts):
    Offset = len(Vertices)
    Vertices.extend(Part.vertices.tolist())
    Faces.extend((Part.faces + Offset).tolist())
    Components.extend([Index] * len(Part.faces))
    # Degenerate zero-volume sheets still block contact, but have no inside.
    Solid = bool(abs(Np.einsum("ij,ij->", Part.triangles[:, 0],
                              Np.cross(Part.triangles[:, 1], Part.triangles[:, 2])) / 6) > 1e-6)
    if Solid and not Part.is_watertight:
        raise ValueError("Basket solid is not watertight; cannot certify its interior")
    Solids.append(Solid)

# The top tray's front wall, moved to where the head depth camera sees it (six frames,
# 2026-09-29): its rim ridge sits at display Y 22.1/22.7/23.3 (5th/50th/95th percentile),
# 2.39 cm behind the CAD's. Along display Y the tray is mapped by a continuous, strictly
# increasing piecewise-linear function: identity behind Y0 (back wall, most of the side walls
# and floor), a uniform compression of the straight side walls and floor between Y0 and Y1, and
# a rigid shift from Y1 (just behind the front corners' fillet row) forward, so the front wall,
# its lip and corners keep their CAD shape. Faces crossing Y0 or Y1 are split there first, so
# the result is the exact image of the CAD solid: still closed and consistently wound.
Frame = Json.loads((Root / "back end/WorkspaceFrame.json").read_text())["RobotFrame"]
ToDisplay = -Assembly["Mounts"][Frame["ControlledArm"]]["PanCm"][1] + Frame["ArmBasePositionCm"]["y"]
TopTray, Y0, Y1, FrontRimY, CadFrontRimY = 2, 12.0, 20.57, 23.3, 25.6928
Tray = sorted({Vertex for Face, Component in zip(Faces, Components) if Component == TopTray for Vertex in Face})
TrayY = Np.array(Vertices)[Tray, 1] + ToDisplay
if (set(Tray) & {Vertex for Face, Component in zip(Faces, Components) if Component != TopTray for Vertex in Face}
        or abs(float(TrayY.max()) - CadFrontRimY) > 1e-3 or min(Np.abs(TrayY - Y0).min(), Np.abs(TrayY - Y1).min()) <= 0.1):
    raise ValueError("The CAD's top tray changed; measure its front rim again before correcting it")
Shift = FrontRimY - float(TrayY.max())
Scale = (Y1 + Shift - Y0) / (Y1 - Y0)


def Split(Plane):
    """Split top-tray faces crossing display Y = Plane; new vertices are shared per edge."""
    Level = Plane - ToDisplay
    Cache, NewFaces, NewComponents = {}, [], []
    for Face, Component in zip(Faces, Components):
        Side = [0 if abs(Vertices[Index][1] - Level) < 1e-7 else (1 if Vertices[Index][1] > Level else -1) for Index in Face]
        if Component != TopTray or min(Side) >= 0 or max(Side) <= 0:
            NewFaces.append(Face)
            NewComponents.append(Component)
            continue
        Ring = []
        for Corner in range(3):
            A, B = Face[Corner], Face[(Corner + 1) % 3]
            Ring.append((A, Side[Corner]))
            if Side[Corner] * Side[(Corner + 1) % 3] < 0:
                Key = (min(A, B), max(A, B))
                if Key not in Cache:
                    PointA, PointB = Np.array(Vertices[A]), Np.array(Vertices[B])
                    Point = (PointA + (Level - PointA[1]) / (PointB[1] - PointA[1]) * (PointB - PointA)).tolist()
                    Point[1] = Level
                    Cache[Key] = len(Vertices)
                    Vertices.append(Point)
                Ring.append((Cache[Key], 0))
        for Keep in ((-1, 0), (0, 1)):
            Polygon = [Index for Index, Where in Ring if Where in Keep]
            for Next in range(1, len(Polygon) - 1):
                NewFaces.append([Polygon[0], Polygon[Next], Polygon[Next + 1]])
                NewComponents.append(Component)
    Faces[:], Components[:] = NewFaces, NewComponents


Split(Y0)
Split(Y1)
Tray = sorted({Vertex for Face, Component in zip(Faces, Components) if Component == TopTray for Vertex in Face})
for Index in Tray:
    Y = Vertices[Index][1] + ToDisplay
    if Y >= Y1:
        Y += Shift
    elif Y > Y0:
        Y = Y0 + (Y - Y0) * Scale
    Vertices[Index][1] = Y - ToDisplay
# The open tray's footprint, where a target may go below the desk; the whole-cart bounds reach further.
TrayBounds = [Np.array(Vertices)[Tray].min(axis=0).tolist(), Np.array(Vertices)[Tray].max(axis=0).tolist()]
FrontWallCorrection = {
    "Component": TopTray, "Frame": f"display Y (robot front), X mirrored; assembly Y = display Y - {ToDisplay:g}",
    "ShiftYCm": round(Shift, 4), "CompressedYCm": [Y0, round(Y1, 4)], "FrontRimYCm": FrontRimY,
    "MeasuredRimYCm": [22.1, 22.7, 23.3],
    "Reason": "The head depth camera (six frames, 2026-09-29) puts the top tray's front rim ridge at Y 22.1/22.7/23.3 "
              "(5th/50th/95th percentile); the CAD put it 2.39 cm further forward. Only the front wall, its corners, "
              "lip and floor edge moved, to the far edge of the measurement; the back wall, side-wall positions "
              "and floor height are the CAD's."}

Triangles = Np.array(Vertices)[Np.array(Faces)]
TriangleLow, TriangleHigh = Triangles.min(axis=1), Triangles.max(axis=1)
Centers = (TriangleLow + TriangleHigh) / 2
Nodes = []


def BuildNode(Indices):
    Index = len(Nodes)
    Low, High = TriangleLow[Indices].min(axis=0), TriangleHigh[Indices].max(axis=0)
    Node = {"Bounds": [Low.tolist(), High.tolist()]}
    Nodes.append(Node)
    if len(Indices) <= 12:
        Node["Faces"] = Indices.tolist()
    else:
        Axis = int(Np.argmax(High - Low))
        Ordered = Indices[Np.argsort(Centers[Indices, Axis], kind="stable")]
        Middle = len(Ordered) // 2
        Node["Children"] = [BuildNode(Ordered[:Middle]), BuildNode(Ordered[Middle:])]
    return Index


BuildNode(Np.arange(len(Faces)))
Surface = {"Version": "basket-surface-v2", "Units": "cm", "Vertices": Vertices, "Faces": Faces,
           "Components": Components, "Solids": Solids, "Nodes": Nodes,
           "SourceSha256": Hashlib.sha256(BasketAsset.read_bytes()).hexdigest(),
           "FrontWallCorrection": FrontWallCorrection}
SurfacePath = Output / "BasketSurface.json"
SurfacePath.write_text(Json.dumps(Surface, separators=(",", ":")) + "\n")

# The cart surface is kept separate: its bounds still define the rim-entry
# planner. The mast and parked camera head are obstacles, not a taller basket.
Vertices, Faces, Components, Solids, BodyParts, Repairs = [], [], [], [], [], []
for Info in Assembly["Parts"]:
    if Info["Category"] not in ("Mast", "Head"):
        continue
    Transform, Geometry = BasketScene.graph[Info["Name"]]
    Mesh = BasketScene.geometry[Geometry].copy()
    Mesh.apply_transform(Transform)
    Np.testing.assert_allclose(Mesh.bounds, Info["BoundsCm"], atol=1e-4)
    FirstFace = len(Faces)
    for Part in Mesh.split(only_watertight=False):
        Solid = bool(abs(Np.einsum("ij,ij->", Part.triangles[:, 0],
                                  Np.cross(Part.triangles[:, 1], Part.triangles[:, 2])) / 6) > 1e-6)
        if not Part.is_watertight:
            # Two source mounting fragments are open meshes. Enclose just that
            # connected fragment so its undefined inside cannot become a hole
            # in collision detection. Never hull the whole mast or basket.
            if Np.linalg.matrix_rank(Part.vertices - Part.vertices.mean(axis=0)) == 3:
                Repairs.append({"Part": Info["Name"], "SourceTriangles": len(Part.faces),
                                "BoundsCm": Part.bounds.tolist(), "Method": "ConnectedFragmentConvexHull"})
                Part = Part.convex_hull
                Solid = True
            else:
                Solid = False
        Index, Offset = len(Solids), len(Vertices)
        Vertices.extend(Part.vertices.tolist())
        Faces.extend((Part.faces + Offset).tolist())
        Components.extend([Index] * len(Part.faces))
        Solids.append(Solid)
    BodyParts.append({"Name": Info["Name"], "Category": Info["Category"], "BoundsCm": Info["BoundsCm"],
                      "FirstFace": FirstFace, "FaceCount": len(Faces) - FirstFace})
Triangles = Np.array(Vertices)[Np.array(Faces)]
TriangleLow, TriangleHigh = Triangles.min(axis=1), Triangles.max(axis=1)
Centers, Nodes = (TriangleLow + TriangleHigh) / 2, []
BuildNode(Np.arange(len(Faces)))
BodySurface = {"Version": "robot-body-surface-v1", "Units": "cm", "Vertices": Vertices, "Faces": Faces,
               "Components": Components, "Solids": Solids, "Nodes": Nodes, "Parts": BodyParts,
               "Repairs": Repairs, "Pose": "FixedAsRendered",
               "SourceSha256": Hashlib.sha256(BasketAsset.read_bytes()).hexdigest()}
BodyPath = Output / "RobotBodySurface.json"
BodyPath.write_text(Json.dumps(BodySurface, separators=(",", ":")) + "\n")
# Version: the app and the bridge refuse to work together unless both load the same one
# (also pinned in scripts/RestartBridge.ps1 and TestRobotBodyCollision). v7: the corrected tray.
# v8: CAD-exact contact (Aaran, 2026-10-04: "If it won't hit the basket in the CAD view, just
# assume it won't in real life"). The basket, cart and body contact checks add no tracking
# allowance (ContactTrackingAllowanceRaw 0): a pose, or the region a route commands, is refused only
# where a link's CAD box meets the surface. JointTrackingAllowanceRaw stays for what the arm itself
# does: the route monitor, braking room, the grasp squeeze and the rearward limit.
Data = {"Version": "robot-body-cad-exact-v8", "Units": "cm", "ClearanceCm": 0.0, "MountClearanceCm": 0.0,
        "JointTrackingAllowanceRaw": 16, "ContactTrackingAllowanceRaw": 0, "InteriorPolicy": "SurfaceOnly",
        "SourceSha256": Hashlib.sha256(Asset.read_bytes()).hexdigest(),
        "BasketSurfaceSha256": Hashlib.sha256(SurfacePath.read_bytes()).hexdigest(),
        "RobotBodySurfaceSha256": Hashlib.sha256(BodyPath.read_bytes()).hexdigest(),
        "RobotBodyAssemblyBoundsCm": Nodes[0]["Bounds"],
        "BasketAssemblyBoundsCm": Cart["BoundsCm"], "BasketTrayBoundsCm": TrayBounds, "Mounts": Assembly["Mounts"],
        "ArmBoxes": Boxes,
        "RearwardLimit": {"MinimumDisplayYCm": -30.0}}
(Output / "CollisionModel.json").write_text(Json.dumps(Data, indent=2) + "\n")
print(f"Built {len(Boxes)} arm envelopes, {len(Surface['Faces'])} basket and {len(Faces)} mast/head triangles; "
      f"{len(Repairs)} fragment repairs; top tray front rim {Shift:+.2f} cm to Y {FrontRimY:g}; "
      f"rearward limit Y = {Data['RearwardLimit']['MinimumDisplayYCm']:g} cm")
