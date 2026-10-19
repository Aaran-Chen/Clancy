"""Write src/lib/GripperParts.json: the gripper as three convex parts, sampled.

The grasp planner and its guard (src/lib/GraspPlan.ts) keep every part of the
gripper clear of the measured objects. The tip the IK and the AI call "the
tip" (gripper_frame_link) sits on the FIXED jaw's inner pad, so the fixed
finger is its own part: it goes down beside an object, while the palm and the
open moving jaw must stay above it or beside it.

Parts (each the convex hull of its mesh points, in its link's frame, cm):
  FixedFinger  = gripper_link__1 vertices with z < PalmZ
  PalmAndServo = gripper_link__1 vertices with z >= PalmZ - 0.05, plus gripper_link__0 (the servo)
  MovingJaw    = moving_jaw_so101_v1_link__0

Hulls (the deep grasp, src/lib/GraspPlan.ts PalmGap): PalmAndServo's full convex hull
vertices (exact, not decimated), and the STS3215 servo's own (gripper_link__0, ServoHull).
The planner measures how near an object stands to the palm and servo on them (GJK), not
on the samples: a hull vertex is a CAD mesh vertex, so the gap is the CAD geometry's
(its hull's, which only fills the palm's few concave spots).

Silhouettes (grasp milestone M3, src/lib/JawSweep.ts): the moving jaw and the
fixed finger are not convex in the jaw plane (the moving jaw's inner face
steps back 0.2 and 0.4 cm above its pad, and the can touches those steps), so
the jaw-plane sweep uses their exact outlines: the union of their mesh
triangles projected onto the plane the jaw swings in, simplified to
SilhouetteToleranceCm. MovingJaw in its link's (x, y) (the hinge is its z),
FixedFinger in gripper_link (x, z) (the hinge is gripper_link y).

Each hull is decimated to the support vertices of 400 Fibonacci directions,
then sampled densely and deterministically and thinned to a SpacingCm
Poisson-disk set (hull vertices first, so corners and edges are kept). Every
point of the hull surface lies within CoverRadiusCm of a sample, and the
decimated hull is at most DecimationLossCm inside the full one: a part whose
samples all stay ContactMarginCm (0.4) clear of a prism does not touch it.

Run after the CAD assets change:  python scripts/BuildGripperParts.py
Check the committed file:         python scripts/BuildGripperParts.py --check
"""
import hashlib as Hashlib
import json as Json
import sys as Sys
from pathlib import Path

import numpy as Np
import trimesh as Trimesh
from scipy.spatial import cKDTree as KdTree
from shapely.geometry import Point as ShapelyPoint, Polygon
from shapely.geometry.polygon import orient
from shapely.ops import unary_union

Root = Path(__file__).resolve().parents[2]
Asset = Root / "AI Controlled WebApp/public/Cad/SO101Parts.glb"
Output = Path(__file__).resolve().parents[1] / "src/lib/GripperParts.json"
HullFile = Path(__file__).resolve().parents[1] / "src/lib/GripperHull.json"
PalmZ, SpacingCm = -3.72, 0.3
# GraspRules.ContactMarginCm in GraspPlan.ts: cover radius plus decimation loss must stay within it.
ContactMarginCm = 0.4
# The fixed pad (report C 2a): its inner face is at x -0.79 between these depths, where the tool point sits.
PadDepth, PadX = (-10.36, -9.44), -0.79
# The silhouettes' outline tolerance (Douglas-Peucker), and how far outside them every mesh vertex may fall.
SilhouetteToleranceCm = 0.02


def Silhouette(Vertices, Faces, Axes):
    """The union of the triangles projected onto two axes: one counter-clockwise outline (cm, 3 decimals)."""
    Flat = Vertices[:, Axes]
    Shapes = [Polygon(Flat[Face]) for Face in Faces]
    Union = unary_union([Shape for Shape in Shapes if Shape.area > 1e-9])
    if Union.geom_type == "MultiPolygon":
        Union = max(Union.geoms, key=lambda Shape: Shape.area)
    # Fill pinholes: only the outer outline can touch an object.
    Outline = orient(Polygon(Union.exterior).simplify(SilhouetteToleranceCm), 1.0)
    Points = Np.round(Np.asarray(Outline.exterior.coords)[:-1], 3)
    # Every vertex of those triangles within the tolerance of the outline (or inside it).
    Grown = Polygon(Points).buffer(SilhouetteToleranceCm + 2e-3)
    Outside = [Index for Index in Np.unique(Faces) if not Grown.contains(ShapelyPoint(Flat[Index]))]
    if Outside:
        raise ValueError(f"{len(Outside)} mesh vertices fall outside the simplified silhouette")
    return Points.tolist()


def Build():
    Scene = Trimesh.load(Asset, force="scene")
    Model = Json.loads((Root / "back end/CadModel.json").read_text())
    Collision = Json.loads((Root / "back end/CollisionModel.json").read_text())
    Vertices, Faces = {}, {}
    for Visual in Model["Visuals"]:
        if Visual["Link"] not in ("gripper_link", "moving_jaw_so101_v1_link"):
            continue
        # Same placement as BuildGripperHull.py: each visual in its link's frame, in cm.
        Transform, Geometry = Scene.graph[Visual["Name"]]
        Mesh = Scene.geometry[Geometry].copy()
        Mesh.apply_transform(Transform)
        Vertices[Visual["Name"]] = Np.asarray(Mesh.vertices) * 100
        Faces[Visual["Name"]] = Np.asarray(Mesh.faces)
    Body, Servo, Jaw = Vertices["gripper_link__1"], Vertices["gripper_link__0"], Vertices["moving_jaw_so101_v1_link__0"]
    Sources = {"FixedFinger": ("gripper_link", Body[Body[:, 2] < PalmZ]),
               "PalmAndServo": ("gripper_link", Np.vstack([Body[Body[:, 2] >= PalmZ - 0.05], Servo])),
               "MovingJaw": ("moving_jaw_so101_v1_link", Jaw)}
    Parts = {}
    for Name, (Link, Points) in Sources.items():
        Full = Trimesh.convex.convex_hull(Points)
        # Decimate: keep the support vertex of each of 400 Fibonacci directions.
        Count = 400
        Index = Np.arange(Count) + 0.5
        Phi = Np.arccos(1 - 2 * Index / Count)
        Theta = Np.pi * (1 + 5 ** 0.5) * Index
        Directions = Np.stack([Np.cos(Theta) * Np.sin(Phi), Np.sin(Theta) * Np.sin(Phi), Np.cos(Phi)], 1)
        Kept = Full.vertices[sorted(set(int(Np.argmax(Full.vertices @ Direction)) for Direction in Directions))]
        # How far the decimated hull falls inside the full one, over many directions.
        Probe = Np.random.default_rng(1).normal(size=(20000, 3))
        Probe /= Np.linalg.norm(Probe, axis=1, keepdims=True)
        Loss = float(Np.max((Full.vertices @ Probe.T).max(axis=0) - (Kept @ Probe.T).max(axis=0)))
        Hull = Trimesh.convex.convex_hull(Kept)
        # Greedy Poisson-disk thinning of a dense deterministic sample: no two kept
        # points closer than SpacingCm, every dense point within SpacingCm of a kept one.
        Dense, _ = Trimesh.sample.sample_surface(Hull, int(Hull.area * 400), seed=3)
        Candidates = Np.vstack([Hull.vertices, Dense])
        Tree = KdTree(Candidates)
        Taken = Np.zeros(len(Candidates), bool)
        Chosen = []
        for Candidate in range(len(Candidates)):
            if Taken[Candidate]:
                continue
            Chosen.append(Candidate)
            Taken[Tree.query_ball_point(Candidates[Candidate], SpacingCm)] = True
        Samples = Np.round(Candidates[Chosen], 3)
        # The cover radius, on random surface points of the hull.
        Test, _ = Trimesh.sample.sample_surface(Hull, 40000, seed=1)
        Cover = float(KdTree(Samples).query(Test)[0].max())
        Bounds = [Points.min(0).round(3).tolist(), Points.max(0).round(3).tolist()]
        # The link's collision boxes were built from the same meshes: the part must lie inside them.
        Boxes = [Box["BoundsCm"] for Box in Collision["ArmBoxes"] if Box["Link"] == Link]
        Low, High = Np.min([Box[0] for Box in Boxes], axis=0), Np.max([Box[1] for Box in Boxes], axis=0)
        if (Np.array(Bounds[0]) < Low - 1e-3).any() or (Np.array(Bounds[1]) > High + 1e-3).any():
            raise ValueError(f"{Name} falls outside the {Link} collision boxes: the mesh placement changed")
        if Cover + Loss > ContactMarginCm:
            raise ValueError(f"{Name}: cover {Cover:.3f} + loss {Loss:.3f} cm exceeds the {ContactMarginCm} cm contact margin")
        Parts[Name] = {"Link": Link, "Samples": Samples.tolist(), "Count": len(Samples), "HullVertices": len(Hull.vertices),
                       "CoverRadiusCm": round(Cover, 4), "DecimationLossCm": round(Loss, 4), "Bounds": Bounds}
        if Name == "PalmAndServo":
            Parts[Name]["Hull"] = Np.round(Full.vertices, 3).tolist()
    Finger = Sources["FixedFinger"][1]
    Pad = Finger[(Finger[:, 2] > PadDepth[0]) & (Finger[:, 2] < PadDepth[1])]
    if abs(Pad[:, 0].max() - PadX) > 0.02:
        raise ValueError(f"the fixed finger's inner face is at x {Pad[:, 0].max():.3f}, not {PadX}: the tool point is no longer on its pad")
    Sha = Hashlib.sha256(Asset.read_bytes()).hexdigest()
    if Sha != Json.loads(HullFile.read_text())["SourceSha256"]:
        raise ValueError("GripperHull.json was built from another GLB: run BuildGripperHull.py first")
    # The jaw-plane outlines: the moving jaw in its link's x-y, the fixed finger (below the palm) in gripper_link x-z.
    JawOutline = Silhouette(Jaw, Faces["moving_jaw_so101_v1_link__0"], [0, 1])
    FingerFaces = Faces["gripper_link__1"][(Body[Faces["gripper_link__1"]][:, :, 2] < PalmZ).all(axis=1)]
    FingerOutline = Silhouette(Body, FingerFaces, [0, 2])
    # The pad face the tool point sits on is the finger outline's inner side there.
    PadSide = [X for X, Z in FingerOutline if PadDepth[0] <= Z <= PadDepth[1]]
    if not PadSide or abs(max(PadSide) - PadX) > 0.03:
        raise ValueError(f"the fixed finger outline's inner face is at x {max(PadSide or [Np.nan]):.3f}, not {PadX}")
    ServoHull = Np.round(Trimesh.convex.convex_hull(Servo).vertices, 3).tolist()
    return {"Version": "gripper-parts-v1", "Units": "cm", "Frame": "link", "PalmZ": PalmZ, "SpacingCm": SpacingCm,
            "SourceSha256": Sha, "Parts": Parts, "ServoHull": ServoHull,
            "Silhouettes": {"ToleranceCm": SilhouetteToleranceCm, "MovingJaw": JawOutline, "FixedFinger": FingerOutline}}


def Check(Built):
    Saved = Json.loads(Output.read_text())
    if Saved["SourceSha256"] != Built["SourceSha256"]:
        raise SystemExit("GripperParts.json was built from another GLB: run scripts/BuildGripperParts.py")
    for Name, Part in Built["Parts"].items():
        Old = Np.array(Saved["Parts"][Name]["Samples"])
        New = Np.array(Part["Samples"])
        if Old.shape != New.shape or Np.abs(Old - New).max() > 0.01:
            raise SystemExit(f"GripperParts.json {Name} differs from a rebuild: run scripts/BuildGripperParts.py")
    for Name, Old, New in (("PalmAndServo hull", Saved["Parts"]["PalmAndServo"].get("Hull", []), Built["Parts"]["PalmAndServo"]["Hull"]),
                           ("servo hull", Saved.get("ServoHull", []), Built["ServoHull"])):
        Old, New = Np.array(Old), Np.array(New)
        if Old.shape != New.shape or Np.abs(Old - New).max() > 0.01:
            raise SystemExit(f"GripperParts.json's {Name} differs from a rebuild: run scripts/BuildGripperParts.py")
    for Name in ("MovingJaw", "FixedFinger"):
        Old = Np.array(Saved.get("Silhouettes", {}).get(Name, []))
        New = Np.array(Built["Silhouettes"][Name])
        if Old.shape != New.shape or Np.abs(Old - New).max() > 0.01:
            raise SystemExit(f"GripperParts.json's {Name} silhouette differs from a rebuild: run scripts/BuildGripperParts.py")
    print("GripperParts.json matches a rebuild:", {Name: Part["Count"] for Name, Part in Built["Parts"].items()},
          {Name: len(Built["Silhouettes"][Name]) for Name in ("MovingJaw", "FixedFinger")})


if __name__ == "__main__":
    Built = Build()
    if "--check" in Sys.argv[1:]:
        Check(Built)
    else:
        Output.write_text(Json.dumps(Built, separators=(",", ":")) + "\n")
        for Name, Part in Built["Parts"].items():
            print(Name, Part["Link"], "samples", Part["Count"], "cover", Part["CoverRadiusCm"], "loss", Part["DecimationLossCm"])
        print("bytes", Output.stat().st_size)
