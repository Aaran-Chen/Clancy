"""Build larger SO-101 follower gripper jaws from the pinned upstream parts.

Two printed parts are derived:
  LargeGripperFixedJawBody: Wrist_Roll_Follower_SO101 with its inward-leaning fixed finger replaced by a
    longer, wider, straight finger. Everything below the finger (gripper servo pocket, screw holes, wrist
    roll horn interface) is the upstream solid, untouched.
  LargeGripperMovingJaw: Moving_Jaw_SO101 with its finger replaced by a longer two-phalanx finger that
    curls toward the fixed jaw, so its whole inner length wraps around an object. The hub, servo horn
    interface, idler side and crossbar are the upstream solid, untouched.

Geometry is built in the wrist roll follower part frame (called the body frame here), where the jaw
rotates about an axis parallel to +Y. Positive gripper angles open the jaw (URDF "gripper" joint).
The moving fingertip ends in a flat pad that lies on the fixed finger's inner face at exactly the angle
the stock jaws first touch, so the new jaws close at the same servo position as the stock ones.

Run from the repository root with the Cad virtual environment:
  & Cad\\.venv\\Scripts\\python.exe Cad\\PrepareLargeGripper.py
It writes STEP/STL/PNG/Verification.json/License.txt to Aaran 3D Prints\\SO101\\Large Gripper.
"""
import hashlib as Hashlib
import json as Json
import math as Math
import os as Os
from pathlib import Path
import shutil as Shutil
import subprocess as Subprocess
import sys as Sys
import xml.etree.ElementTree as Xml

import cadquery as Cq
import numpy as Np
import trimesh as Trimesh
from matplotlib.path import Path as PolygonPath
from scipy.spatial import cKDTree as KdTree
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.gp import gp_Trsf

SourceRoot = Path.home() / "Downloads/SO101Cad"
Output = Path.home() / "Aaran 3D Prints/SO101/Large Gripper"
BlenderExe = Path("C:/Program Files/Blender Foundation/Blender 5.1/blender.exe")
StockBodyStep = "Original/STEP/SO101/Follower_Specific/Wrist_Roll_Follower_SO101.step"
StockJawStep = "Original/STEP/SO101/Follower_Specific/Moving_Jaw_SO101.step"
StockBodyStl = "Original/STL/SO101/Individual/Wrist_Roll_Follower_SO101.stl"
StockJawStl = "Original/STL/SO101/Individual/Moving_Jaw_SO101.stl"
UrdfFile = "Original/Simulation/SO101/so101_new_calib.urdf"
ServoStl = "Original/Simulation/SO101/assets/sts3215_03a_v1.stl"

# Design parameters, millimetres and degrees, body frame with the moving jaw at gripper angle 0.
Design = {
    "JawWidthY": (-24.12, 23.88),   # both fingers span the moving jaw's full yoke width (48 mm)
    "FixedOuterX": -35.2,           # flush with the body's outer side
    "FixedThickness": 8.0,          # straight finger, inner face at FixedOuterX + FixedThickness
    "FixedBaseHalfWidth": 16.0,     # the body below the stock finger is 32 mm wide
    "FixedBaseFlare": 9.0,          # height over which the finger widens to JawWidthY (48 deg from horizontal)
    "FixedTipChamfer": 3.0,
    "MovingCutZ": 46.0,             # stock moving jaw kept below this (hub, yoke, crossbar)
    "MovingInnerX": 11.0,           # inner edge of the new finger where it leaves the crossbar
    "MovingBaseOuterX": 28.0,       # outer edge of the crossbar top
    "ProximalTiltDeg": 20.0,        # proximal phalanx leans outward (away from the fixed jaw)
    "ProximalLength": 65.0,         # inner face, crossbar to knuckle
    "KnuckleBendDeg": 75.0,         # distal phalanx turns this far toward the fixed jaw
    "MovingThickness": 8.0,
    "BaseTransition": 14.0,         # proximal outer face widens to the crossbar over this length
    "EdgeRound": 1.0,               # rounding of the finger profile corners
}
TorqueNm = 30 * 9.80665 / 100      # assumed STS3215 stall torque on the 12 V SO-101 follower: 30 kg-cm
PlaDensity = 1.24                  # g/cm3
SweepMaxDeg = 140.0                # beyond the 135.5 deg gui_limits maximum
PalmZ = 37.5                       # replaced by the measured body plate top


def Sha256(File):
    return Hashlib.sha256(Path(File).read_bytes()).hexdigest()


def LoadStep(File):
    Reader = STEPControl_Reader()
    assert Reader.ReadFile(str(File)) == IFSelect_RetDone, File
    assert Reader.TransferRoots() > 0, File
    return Cq.Shape.cast(Reader.OneShape())


def Solids(Shape):
    Explorer = TopExp_Explorer(Shape.wrapped, TopAbs_SOLID)
    Count = 0
    while Explorer.More():
        Count += 1
        Explorer.Next()
    return Count


def Volume(Shape):
    Props = GProp_GProps()
    BRepGProp.VolumeProperties_s(Shape.wrapped, Props)
    return Props.Mass()


def Area(Shape):
    Props = GProp_GProps()
    BRepGProp.SurfaceProperties_s(Shape.wrapped, Props)
    return Props.Mass()


def Transform(Shape, Matrix):
    Trsf = gp_Trsf()
    Trsf.SetValues(*[float(Value) for Value in Np.asarray(Matrix)[:3].ravel()])
    return Cq.Shape.cast(Shape.wrapped.Moved(TopLoc_Location(Trsf)))


def Origin(Element):
    Matrix = Trimesh.transformations.euler_matrix(*[float(V) for V in Element.get("rpy").split()], "sxyz")
    Matrix[:3, 3] = [float(V) * 1000 for V in Element.get("xyz").split()]
    return Matrix


def LoadFrames():
    """Body-frame placement of the moving jaw and servo, from the upstream URDF (mm)."""
    Root = Xml.parse(SourceRoot / UrdfFile).getroot()
    Link = next(L for L in Root.iter("link") if L.get("name") == "gripper_link")
    Visuals = {V.find("geometry/mesh").get("filename"): Origin(V.find("origin")) for V in Link.findall("visual")}
    BodyInLink = Visuals["assets/wrist_roll_follower_so101_v1.stl"]
    ServoInLink = Visuals["assets/sts3215_03a_v1.stl"]
    Joint = next(J for J in Root.iter("joint") if J.get("name") == "gripper")
    JawLink = next(L for L in Root.iter("link") if L.get("name") == "moving_jaw_so101_v1_link")
    JawVisual = Origin(JawLink.find("visual/origin"))
    Inverse = Np.linalg.inv(BodyInLink)
    return {"JointInBody": Inverse @ Origin(Joint.find("origin")), "JawVisual": JawVisual,
            "ServoInBody": Inverse @ ServoInLink,
            "UrdfLimitsDeg": [Math.degrees(float(Joint.find("limit").get(K))) for K in ("lower", "upper")]}


Frames = LoadFrames()
PivotX, PivotZ = Frames["JointInBody"][0, 3], Frames["JointInBody"][2, 3]
assert Np.allclose(Frames["JointInBody"][:3, 2], [0, 1, 0], atol=1e-4), "gripper axis should be body +Y"


def JawToBody(AngleDeg):
    Rotation = Trimesh.transformations.rotation_matrix(Math.radians(AngleDeg), [0, 0, 1])
    return Frames["JointInBody"] @ Rotation @ Frames["JawVisual"]


def RotateXz(Points, AngleDeg):
    """Rotate body-frame (x, z) points about the jaw axis by the gripper angle (positive opens)."""
    Cos, Sin = Math.cos(Math.radians(AngleDeg)), Math.sin(Math.radians(AngleDeg))
    Delta = Np.asarray(Points, float) - [PivotX, PivotZ]
    return Np.stack([Delta[:, 0] * Cos + Delta[:, 1] * Sin, -Delta[:, 0] * Sin + Delta[:, 1] * Cos], 1) + [PivotX, PivotZ]


def Distance(ShapeA, ShapeB):
    Extrema = BRepExtrema_DistShapeShape(ShapeA.wrapped, ShapeB.wrapped)
    Extrema.Perform()
    return Extrema.Value()


def CommonVolume(ShapeA, ShapeB):
    return Volume(Cq.Shape.cast(BRepAlgoAPI_Common(ShapeA.wrapped, ShapeB.wrapped).Shape()))


def Interferes(Fixed, Jaw, AngleDeg):
    Moved = Transform(Jaw, JawToBody(AngleDeg))
    return CommonVolume(Fixed, Moved) > 1e-3 or Distance(Fixed, Moved) < 1e-6


def Bisect(Fixed, Jaw, Clear, Blocked, Steps=13):
    """Angle between a clear and a blocked angle where the jaw first touches."""
    for _ in range(Steps):
        Middle = (Clear + Blocked) / 2
        if Interferes(Fixed, Jaw, Middle):
            Blocked = Middle
        else:
            Clear = Middle
    return Clear


def Box(X0, X1, Y0, Y1, Z0, Z1):
    return Cq.Solid.makeBox(X1 - X0, Y1 - Y0, Z1 - Z0, Cq.Vector(X0, Y0, Z0))


def BodyPlateTop(StockBody):
    """Top of the servo cage beside the stock fixed finger; only the finger rises above it."""
    Top = Cq.Shape.cast(BRepAlgoAPI_Common(StockBody.wrapped, Box(-5, 100, -100, 100, -50, 300).wrapped).Shape())
    Height = round(Top.BoundingBox().zmax, 3)
    Above = Volume(Cq.Shape.cast(BRepAlgoAPI_Common(StockBody.wrapped, Box(-5, 100, -100, 100, Height + 0.01, 300).wrapped).Shape()))
    assert Above < 1e-6, "material above the cage top beside the finger"
    return Height


def FaceSignature(Face):
    Centre = Face.Center()
    return (Face.geomType(), round(Face.Area(), 3), round(Centre.x, 3), round(Centre.y, 3), round(Centre.z, 3))


def SameFace(A, B):
    """Same type, area within 0.05 % (numerical integration noise after a STEP round trip) and centre
    within 5 micrometres."""
    return A[0] == B[0] and abs(A[1] - B[1]) <= max(0.02, 5e-4 * A[1]) and max(abs(P - Q) for P, Q in zip(A[2:], B[2:])) <= 0.005


def MountingFaces(New, Stock, CutZ):
    """Upstream faces lying wholly below CutZ (body frame) must reappear unchanged in the new part.

    These are the servo pocket, screw holes and horn interfaces. Face-by-face comparison avoids booleans
    between nearly identical solids, which Open Cascade does not handle reliably."""
    NewFaces = [FaceSignature(Face) for Face in New.Faces()]
    Below = [Face for Face in Stock.Faces() if Face.BoundingBox().zmax < CutZ - 0.05]
    Missing = [FaceSignature(Face) for Face in Below if not any(SameFace(FaceSignature(Face), Other) for Other in NewFaces)]
    Holes = [Face for Face in Below if Face.geomType() == "CYLINDER"]
    return {"CutZ": CutZ, "UpstreamFacesBelowCut": len(Below), "CylindricalFacesBelowCut": len(Holes),
            "Missing": Missing, "UpstreamFacesTotal": len(Stock.Faces()), "NewFacesTotal": len(New.Faces())}


def MovingFingerProfile(ClosingDeg, FixedInnerX):
    """Closed (x, z) outline of the new moving finger, body frame, jaw at 0 deg; plus key points."""
    D = Design
    Inner = Np.array([D["MovingInnerX"], D["MovingCutZ"]])
    Up = Np.array([Math.sin(Math.radians(D["ProximalTiltDeg"])), Math.cos(Math.radians(D["ProximalTiltDeg"]))])
    Knuckle = Inner + Up * D["ProximalLength"]
    Angle = Math.radians(D["ProximalTiltDeg"] - D["KnuckleBendDeg"])
    Down = Np.array([Math.sin(Angle), Math.cos(Angle)])
    OutUp, OutDown = Np.array([Up[1], -Up[0]]), Np.array([Down[1], -Down[0]])
    Thick = D["MovingThickness"]
    # The pad: the fixed finger's inner face seen from the jaw when it is at the closing angle.
    Pad = RotateXz([[FixedInnerX, 0.0], [FixedInnerX, 250.0]], -ClosingDeg)
    PadDir = (Pad[1] - Pad[0]) / Np.linalg.norm(Pad[1] - Pad[0])

    def Hit(Point, Direction):
        Scale = Np.linalg.solve(Np.array([Direction, -PadDir]).T, Pad[0] - Point)
        return Point + Direction * Scale[0], Scale[0]

    TipInner, DistalLength = Hit(Knuckle, Down)
    TipOuter, _ = Hit(Knuckle + OutDown * Thick, Down)
    Start = Math.atan2(OutDown[1], OutDown[0])   # outer knuckle: turn clockwise from the distal to the proximal side
    Sweep = Np.linspace(Start, Start - Math.radians(D["KnuckleBendDeg"]), 10)
    KnuckleArc = Knuckle + Thick * Np.stack([Np.cos(Sweep), Np.sin(Sweep)], 1)
    BaseOuter = Inner + Up * D["BaseTransition"] + OutUp * Thick
    Outline = Np.concatenate([[[D["MovingInnerX"], D["MovingCutZ"]]], [Knuckle], [TipInner], [TipOuter],
                              KnuckleArc, [BaseOuter], [[D["MovingBaseOuterX"], D["MovingCutZ"]]]])
    return Outline, {"Knuckle": Knuckle, "TipInner": TipInner, "TipOuter": TipOuter, "DistalLength": DistalLength,
                     "PadDirection": PadDir}


def RoundedPrism(Outline, Y0, Y1, Radius):
    """Extrude a body-frame (x, z) outline along y from Y0 to Y1, all profile corners rounded by Radius."""
    Wire = Cq.Wire.makePolygon([Cq.Vector(float(X), Y0, float(Z)) for X, Z in Outline], close=True)
    if Radius > 0:
        for Sign in (1, -1):   # one order rounds the convex corners, the other the concave ones
            Wire = Wire.offset2D(Sign * Radius, "arc")[0].offset2D(-Sign * Radius, "arc")[0]
    return Cq.Solid.extrudeLinear(Cq.Face.makeFromWires(Wire), Cq.Vector(0, Y1 - Y0, 0))


def BuildFixedBody(StockBody, PlateTop, Top):
    D = Design
    Y0, Y1 = D["JawWidthY"]
    InnerX = D["FixedOuterX"] + D["FixedThickness"]
    Kept = StockBody.cut(Box(-100, 100, -100, 100, PlateTop, 300))
    # Front profile in the y-z plane: flares from the body top width to the full jaw width, chamfered tip.
    BodyHalfWidth = D["FixedBaseHalfWidth"]
    Flare, Chamfer = D["FixedBaseFlare"], D["FixedTipChamfer"]
    Profile = [(-BodyHalfWidth, PlateTop), (BodyHalfWidth, PlateTop), (Y1, PlateTop + Flare), (Y1, Top - Chamfer),
               (Y1 - Chamfer, Top), (Y0 + Chamfer, Top), (Y0, Top - Chamfer), (Y0, PlateTop + Flare)]
    Finger = (Cq.Workplane("YZ", origin=(D["FixedOuterX"], 0, 0)).polyline(Profile).close()
              .extrude(InnerX - D["FixedOuterX"]).val())
    Body = Kept.fuse(Finger)   # no clean(): keep the upstream faces exactly as they are
    return Body, Kept, Finger


def BuildMovingJaw(StockJaw, ClosingDeg, FixedInnerX):
    D = Design
    ToBody = JawToBody(0)
    ToJaw = Np.linalg.inv(ToBody)
    # Keep the stock jaw below MovingCutZ (body frame at 0 deg) = above local y = PivotZ - MovingCutZ.
    CutBody = Transform(Box(-200, 200, -200, 200, D["MovingCutZ"], 400), ToJaw)
    Kept = StockJaw.cut(CutBody)
    Outline, Points = MovingFingerProfile(ClosingDeg, FixedInnerX)
    Y0, Y1 = D["JawWidthY"]
    # Workplane XZ has normal -Y, so extrude from y=Y1 toward -Y by the width.
    Finger = RoundedPrism(Outline, Y0, Y1, D["EdgeRound"])
    Finger = Transform(Finger, ToJaw)
    Jaw = Kept.fuse(Finger)
    return Jaw, Kept, Finger, Points


def Tessellate(Shape, File):
    Cq.exporters.export(Cq.Workplane().add(Shape), str(File), tolerance=0.02, angularTolerance=0.1)
    Mesh = Trimesh.load(File, force="mesh")
    if not Mesh.is_watertight:
        # Open Cascade can leave a zero-area sliver triangle; drop it and rewrite the file.
        Mesh.update_faces(Mesh.nondegenerate_faces(height=1e-6))
        Mesh.remove_unreferenced_vertices()
        Mesh.merge_vertices()
        Mesh.export(File)
    return Mesh


def PrintedMass(Shape, Perimeters=3, LineWidth=0.42, Infill=0.15):
    VolumeMm3 = Volume(Shape)
    Shell = min(VolumeMm3, Area(Shape) * Perimeters * LineWidth)
    return round(VolumeMm3 * PlaDensity / 1000, 1), round((Shell + Infill * (VolumeMm3 - Shell)) * PlaDensity / 1000, 1)


def OverhangArea(Mesh, Up=2):
    """Downward faces flatter than 45 deg from horizontal and above the bed, in mm2."""
    Bed = Mesh.bounds[0][Up]
    Down = (Mesh.face_normals[:, Up] < -Math.cos(Math.radians(45))) & (Mesh.triangles_center[:, Up] > Bed + 0.3)
    return round(float(Mesh.area_faces[Down].sum()), 1)


# ---------- 2D grasp analysis on the y = 0 section (the plane the jaw moves in) ----------

def SectionPoints(Mesh, Spacing=0.3):
    Segments = Trimesh.intersections.mesh_plane(Mesh, [0, 1, 0], [0, 0, 0])[:, :, [0, 2]]
    Points = []
    for Start, Stop in Segments:
        Count = max(1, int(Np.linalg.norm(Stop - Start) / Spacing))
        Points.append(Start + (Stop - Start) * Np.linspace(0, 1, Count + 1)[:, None])
    return Np.concatenate(Points), Segments


def InsideSegments(Point, Segments):
    A, B = Segments[:, 0], Segments[:, 1]
    Straddle = (A[:, 1] > Point[1]) != (B[:, 1] > Point[1])
    Cross = A[:, 0] + (Point[1] - A[:, 1]) * (B[:, 0] - A[:, 0]) / Np.where(Straddle, B[:, 1] - A[:, 1], 1)
    return bool(Np.count_nonzero(Straddle & (Cross > Point[0])) % 2)


def GraspAt(FixedSection, JawSection, Radius, AngleDeg, Tolerance=0.6):
    """Positions where a cylinder of this radius touches both jaws without overlap, at one jaw angle."""
    FixedPoints, FixedSegments = FixedSection
    JawPoints = RotateXz(JawSection[0], AngleDeg)
    JawSegments = Np.stack([RotateXz(JawSection[1][:, 0], AngleDeg), RotateXz(JawSection[1][:, 1], AngleDeg)], 1)
    FixedTree, JawTree = KdTree(FixedPoints), KdTree(JawPoints)
    X, Z = Np.meshgrid(Np.arange(-40, 130, 0.5), Np.arange(PalmZ + Radius - 0.5, 190, 0.5))
    Centres = Np.stack([X.ravel(), Z.ravel()], 1)
    FixedGap = FixedTree.query(Centres)[0] - Radius
    JawGap = JawTree.query(Centres)[0] - Radius
    Valid = (FixedGap >= 0) & (JawGap >= 0) & (FixedGap < Tolerance) & (JawGap < Tolerance)
    Results = []
    for Centre in Centres[Valid]:
        if InsideSegments(Centre, FixedSegments) or InsideSegments(Centre, JawSegments):
            continue
        Contacts = Np.concatenate([FixedPoints[FixedTree.query_ball_point(Centre, Radius + Tolerance)],
                                   JawPoints[JawTree.query_ball_point(Centre, Radius + Tolerance)]])
        Angles = Np.sort(Np.degrees(Np.arctan2(Contacts[:, 1] - Centre[1], Contacts[:, 0] - Centre[0])) % 360)
        Gap = float(Np.max(Np.diff(Np.concatenate([Angles, [Angles[0] + 360]]))))
        Results.append({"Centre": Centre, "LargestContactGapDeg": Gap,
                        "FixedSide": FixedPoints[FixedTree.query_ball_point(Centre, Radius + Tolerance)],
                        "JawSide": JawPoints[JawTree.query_ball_point(Centre, Radius + Tolerance)]})
    return Results


def Cross2(A, B):
    return float(A[0] * B[1] - A[1] * B[0])


def Classify(Result, Radius):
    """Caged: the contacts leave no gap of 180 deg or more around the object, so it cannot slip out of the
    jaw plane. Pinch: held by two opposed contacts (normals within 20 deg of opposite), relying on friction."""
    if Result["LargestContactGapDeg"] < 180:
        return "Caged"
    Centre = Result["Centre"]
    FixedNormal = Np.mean(Result["FixedSide"], 0) - Centre
    JawNormal = Np.mean(Result["JawSide"], 0) - Centre
    Opposed = Math.degrees(Math.acos(Np.clip(FixedNormal @ JawNormal / Np.linalg.norm(FixedNormal) / Np.linalg.norm(JawNormal), -1, 1)))
    return "Pinch" if Opposed >= 160 else None


def Grasp(FixedSection, JawSection, Diameter, ClosingDeg, MaxDeg=100.0, Step=0.5):
    """Most-closed jaw angle at which a cylinder (axis along the jaw axis) is held: caged or pinched."""
    Radius = Diameter / 2
    for AngleDeg in Np.arange(ClosingDeg + 0.25, MaxDeg + 1e-6, Step):
        Held = [(Classify(R, Radius), R) for R in GraspAt(FixedSection, JawSection, Radius, AngleDeg)]
        Held = [(Kind, R) for Kind, R in Held if Kind]
        if Held:
            Kind, Best = min(Held, key=lambda Item: (Item[0] != "Caged", Item[1]["LargestContactGapDeg"]))
            Lever = [abs(Cross2(P - [PivotX, PivotZ], (Best["Centre"] - P) / Np.linalg.norm(Best["Centre"] - P)))
                     for P in Best["JawSide"]]
            return {"DiameterMm": Diameter, "JawAngleDeg": round(float(AngleDeg), 2), "Kind": Kind,
                    "CentreXZ": [round(float(V), 1) for V in Best["Centre"]],
                    "LargestContactGapDeg": round(Best["LargestContactGapDeg"], 1),
                    "ContactsSpanDeg": round(360 - Best["LargestContactGapDeg"], 1),
                    "JawContactLeverMm": round(float(Np.mean(Lever)), 1),
                    "ForceAtStallN": round(TorqueNm * 1000 / float(Np.mean(Lever)), 1)}
    return None


def LargestGrasps(FixedSection, JawSection, ClosingDeg, MaxDeg=100.0):
    """Largest cylinders (4 mm steps) caged, and held at all, with the jaw opened at most MaxDeg.
    Stops after three diameters in a row that cannot be held."""
    Caged = Held = None
    Misses = 0
    for Diameter in range(40, 201, 4):
        Result = Grasp(FixedSection, JawSection, Diameter, ClosingDeg, MaxDeg, Step=0.5)
        if Result is None:
            Misses += 1
            if Misses == 3:
                break
            continue
        Misses = 0
        Held = Result
        if Result["Kind"] == "Caged":
            Caged = Result
    return {"LargestCagedCylinder": Caged, "LargestHeldCylinder": Held, "MaxJawAngleDeg": MaxDeg}


def ComposeRow(Files, Name):
    """Place same-size renders side by side and delete the parts."""
    from PIL import Image
    Images = [Image.open(Output / File).convert("RGB") for File in Files]
    Row = Image.new("RGB", (sum(I.width for I in Images), max(I.height for I in Images)))
    Left = 0
    for Item in Images:
        Row.paste(Item, (Left, 0))
        Left += Item.width
    Row.save(Output / Name)
    for File in Files:
        if File not in ("LargeGripperClosedFront.png",):
            (Output / File).unlink()


def CalibratedRange():
    """The gripper's software limits in URDF degrees, via the app's CadAlignment mapping (read only)."""
    BackEnd = Path(__file__).resolve().parents[1] / "back end"
    Limits = Json.loads((BackEnd / "gui_limits.json").read_text())
    Alignment = Json.loads((BackEnd / "CadAlignment.json").read_text())
    Gripper = next(V for V in Limits.values() if isinstance(V, dict) and "gripper" in V)["gripper"]
    Zero, Sign = Alignment["ReferenceRaw"]["gripper"], Alignment["Direction"]["gripper"]
    Degrees = sorted(Sign * (Raw - Zero) * 360 / 4096 for Raw in (Gripper["min"], Gripper["max"]))
    return {"RawMin": Gripper["min"], "RawMax": Gripper["max"], "ReferenceRaw": Zero,
            "UrdfDegrees": [round(V, 2) for V in Degrees]}


if __name__ == "__main__":
    Output.mkdir(parents=True, exist_ok=True)
    Manifest = Json.loads((SourceRoot / "SourceManifest.json").read_text())
    Sources = []
    for Relative in (StockBodyStep, StockJawStep, StockBodyStl, StockJawStl, UrdfFile, ServoStl):
        Entry = next(Item for Item in Manifest["Files"] if Item["Path"] == Relative)
        assert Sha256(SourceRoot / Relative) == Entry["Sha256"], f"Original file differs from recorded source: {Relative}"
        Sources.append({"File": Relative, "Source": Entry["SourceUrl"], "Sha256": Entry["Sha256"]})

    StockBody = LoadStep(SourceRoot / StockBodyStep)
    StockJaw = LoadStep(SourceRoot / StockJawStep)
    Calibrated = CalibratedRange()
    print("Calibrated gripper range (URDF deg):", Calibrated["UrdfDegrees"], flush=True)

    # Stock jaw-on-jaw closing angle and yoke-on-body open stop.
    StockClosing = Bisect(StockBody, StockJaw, -8.0, -16.0)
    StockOpenStop = Bisect(StockBody, StockJaw, 108.0, 120.0)
    print(f"Stock closes at {StockClosing:.3f} deg, open stop {StockOpenStop:.3f} deg", flush=True)

    PlateTop = BodyPlateTop(StockBody)
    PalmZ = PlateTop
    Extrema = BRepExtrema_DistShapeShape(StockBody.wrapped, Transform(StockJaw, JawToBody(StockClosing + 0.05)).wrapped)
    Extrema.Perform()
    StockTipLever = float(Extrema.PointOnShape1(1).Z() - PivotZ)
    FixedInnerX = Design["FixedOuterX"] + Design["FixedThickness"]
    Outline, Points = MovingFingerProfile(StockClosing, FixedInnerX)
    ClosedTips = RotateXz(Np.array([Points["TipInner"], Points["TipOuter"]]), StockClosing)
    FixedTopZ = float(round(ClosedTips[:, 1].max() + 1.0, 1))   # fixed finger ends level with the closed pad
    print(f"Plate top {PlateTop}, fixed finger top {FixedTopZ}, distal length {Points['DistalLength']:.1f}", flush=True)

    Body, BodyKept, FixedFinger = BuildFixedBody(StockBody, PlateTop, FixedTopZ)
    Jaw, JawKept, MovingFinger, _ = BuildMovingJaw(StockJaw, StockClosing, FixedInnerX)
    for Name, Shape in (("Body", Body), ("Jaw", Jaw)):
        assert Solids(Shape) == 1 and BRepCheck_Analyzer(Shape.wrapped).IsValid(), f"{Name} is not one valid solid"

    BodyStep, JawStep = Output / "LargeGripperFixedJawBody.step", Output / "LargeGripperMovingJaw.step"
    Cq.exporters.export(Cq.Workplane().add(Body), str(BodyStep))
    Cq.exporters.export(Cq.Workplane().add(Jaw), str(JawStep))
    # Check what was delivered: reload the STEP files (in-memory results share topology with the
    # stock shapes, which makes booleans against the stock parts unreliable).
    Body, Jaw = LoadStep(BodyStep), LoadStep(JawStep)
    BodyMesh = Tessellate(Body, Output / "LargeGripperFixedJawBody.stl")
    JawMesh = Tessellate(Jaw, Output / "LargeGripperMovingJaw.stl")
    assert BodyMesh.is_watertight and JawMesh.is_watertight

    # Mounting regions: every upstream face below the cut is present, unchanged, in the new part.
    Mounting = {"FixedJawBody": MountingFaces(Body, StockBody, PlateTop),
                "MovingJaw": MountingFaces(Transform(Jaw, JawToBody(0)), Transform(StockJaw, JawToBody(0)), Design["MovingCutZ"])}
    for Name, Record in Mounting.items():
        print(Name, Record, flush=True)
        assert not Record["Missing"], f"{Name} mounting region changed: {Record['Missing'][:5]}"

    # Collision sweep about the real servo axis (B-rep distance), new jaw vs new body and the servo envelope.
    ServoMesh = Trimesh.load(SourceRoot / ServoStl, force="mesh")
    ServoMesh.apply_scale(1000)
    ServoMesh.apply_transform(Frames["ServoInBody"])
    (S0, S1) = ServoMesh.bounds
    ServoBox = Box(S0[0], S1[0], S0[1], S1[1], S0[2], S1[2])
    NewClosing = Bisect(Body, Jaw, StockClosing + 2.0, StockClosing - 2.0)
    NewOpenStop = Bisect(Body, Jaw, 108.0, 120.0)
    print(f"New jaws close at {NewClosing:.3f} deg, open stop {NewOpenStop:.3f} deg", flush=True)
    Sweep = []
    Angles = sorted(set(Np.round(Np.concatenate([Np.arange(StockClosing + 0.5, StockClosing + 5, 0.5),
                                                  Np.arange(Math.ceil(StockClosing + 5), SweepMaxDeg + 0.01, 2.0)]), 2)))
    for AngleDeg in Angles:
        Moved = Transform(Jaw, JawToBody(AngleDeg))
        FingerMoved = Transform(MovingFinger, JawToBody(AngleDeg))
        Whole = Distance(Body, Moved)
        Record = {"AngleDeg": float(AngleDeg), "JawToBodyMm": round(Whole, 3),
                  "NewFingerToBodyMm": round(Distance(Body, FingerMoved), 3),
                  "NewFingerToServoEnvelopeMm": round(Distance(ServoBox, FingerMoved), 3),
                  "NewFixedFingerToJawMm": round(Distance(FixedFinger, Moved), 3)}
        if Whole < 1e-6:
            Record["OverlapMm3"] = round(CommonVolume(Body, Moved), 3)
        Sweep.append(Record)
        print(Record, flush=True)
    InTravel = [R for R in Sweep if R["AngleDeg"] < NewOpenStop]
    Collisions = [R["AngleDeg"] for R in InTravel if R.get("OverlapMm3", 0) > 1e-3]
    FingerHits = [R["AngleDeg"] for R in Sweep if R["NewFingerToBodyMm"] < 1e-6 or R["NewFingerToServoEnvelopeMm"] < 1e-6]
    assert not Collisions, f"Moving jaw collides with the body at {Collisions}"
    assert not FingerHits, f"New finger touches the body or servo at {FingerHits}"

    # Grasp analysis in the jaw plane (y = 0), new versus stock, same method.
    StockBodyMesh = Trimesh.load(SourceRoot / StockBodyStl, force="mesh")
    StockJawMesh = Trimesh.load(SourceRoot / StockJawStl, force="mesh")
    Sections = {}
    for Name, BodyM, JawM in (("New", BodyMesh, JawMesh), ("Stock", StockBodyMesh, StockJawMesh)):
        Placed = JawM.copy()
        Placed.apply_transform(JawToBody(0))
        Sections[Name] = (SectionPoints(BodyM), SectionPoints(Placed))
    Grasps = {}
    for Name in ("New", "Stock"):
        Closing = StockClosing
        Grasps[Name] = {f"Cylinder{D}mm": Grasp(*Sections[Name], D, Closing) for D in (66, 40, 20)}
        Grasps[Name].update(LargestGrasps(*Sections[Name], Closing))
        print(Name, Json.dumps(Grasps[Name]), flush=True)

    # Closed-pad pinch lever and force; stock uses its tip contact point.
    PadCentre = RotateXz(Np.array([(Points["TipInner"] + Points["TipOuter"]) / 2]), StockClosing)[0]
    PadLever = float(PadCentre[1] - PivotZ)

    def Extent(Shape):
        Bounds = Shape.BoundingBox()
        return [round(V, 2) for V in (Bounds.xmin, Bounds.xmax, Bounds.ymin, Bounds.ymax, Bounds.zmin, Bounds.zmax)]

    Masses = {}
    for Name, Shape in (("FixedJawBody", Body), ("MovingJaw", Jaw), ("StockWristRollFollower", StockBody), ("StockMovingJaw", StockJaw)):
        Solid, Printed = PrintedMass(Shape)
        Masses[Name] = {"VolumeCm3": round(Volume(Shape) / 1000, 2), "SolidMassG": Solid, "Estimated15PctGyroid3PerimetersG": Printed}

    StockJawPlaced = Transform(StockJaw, JawToBody(0))
    Report = {
        "Part": "SO-101 follower large gripper (fixed jaw body + moving jaw)",
        "SourceCommit": Manifest["Commit"], "Sources": Sources,
        "Files": {Path(F).name: Sha256(F) for F in (BodyStep, JawStep, Output / "LargeGripperFixedJawBody.stl", Output / "LargeGripperMovingJaw.stl")},
        "Frames": "STEP/STL use the upstream part frames, so the parts drop into the upstream URDF/CAD unchanged.",
        "Design": Design,
        "Dimensions": {
            "PalmZ": PlateTop, "FixedFingerTopZ": FixedTopZ,
            "FixedFingerLengthAbovePalmMm": round(FixedTopZ - PlateTop, 1),
            "StockFixedFingerLengthAbovePalmMm": round(StockBody.BoundingBox().zmax - PlateTop, 1),
            "FixedInnerFaceX": FixedInnerX, "FixedInnerFace": "straight (vertical), no inward bend",
            "JawWidthMm": round(Design["JawWidthY"][1] - Design["JawWidthY"][0], 2),
            "StockFingerWidthMm": {"FixedAtBase": 32.0, "FixedAtTip": 8.0, "MovingFinger": 17.4},
            "MovingDistalLengthMm": round(float(Points["DistalLength"]), 1),
            "MovingFingerReachFromAxisMm": round(float(max(Np.linalg.norm(P - [PivotX, PivotZ]) for P in (Points["TipInner"], Points["TipOuter"]))), 1),
            "StockMovingFingerReachFromAxisMm": round(-StockJaw.BoundingBox().ymin, 1),
            "GripperLengthBodyBottomToTipMm": round(FixedTopZ - StockBody.BoundingBox().zmin, 1),
            "StockGripperLengthBodyBottomToTipMm": round(StockBody.BoundingBox().zmax - StockBody.BoundingBox().zmin, 1),
            "FixedJawBodyExtentsMm": Extent(Body), "MovingJawExtentsMm": Extent(Jaw),
            "PrintBedFitKobraS1": bool(max(BodyMesh.extents) < 250 and max(JawMesh.extents) < 250),
        },
        "Travel": {
            "CalibratedRange": Calibrated, "UrdfLimitsDeg": [round(V, 2) for V in Frames["UrdfLimitsDeg"]],
            "StockClosingContactDeg": round(StockClosing, 3), "NewClosingContactDeg": round(NewClosing, 3),
            "StockOpenStopDeg": round(StockOpenStop, 3), "NewOpenStopDeg": round(NewOpenStop, 3),
            "OpenStopCause": "the stock yoke (kept unchanged) meets the stock body flange below the servo; same for stock and new",
        },
        "Sweep": {
            "Method": "OpenCascade B-rep distance and boolean common, jaw rotated about the URDF gripper axis",
            "RangeDeg": [Angles[0], Angles[-1]], "Steps": len(Angles),
            "CollisionsBeforeOpenStop": Collisions, "NewFingerContacts": FingerHits,
            "MinJawToBodyClearanceUpTo110DegMm": min(R["JawToBodyMm"] for R in InTravel if R["AngleDeg"] <= 110),
            "MinJawToBodyClearanceNote": ("0.51 mm, constant, between the stock hub and stock body (unchanged from upstream); "
                                          "above 110 deg it closes to zero at the open stop, where the stock yoke meets the body flange"),
            "MinNewFingerToBodyMm": min(R["NewFingerToBodyMm"] for R in Sweep),
            "MinNewFingerToServoEnvelopeMm": min(R["NewFingerToServoEnvelopeMm"] for R in Sweep),
            "FixedFingerToJawAtClosingPlus1DegMm": next(R["NewFixedFingerToJawMm"] for R in Sweep if R["AngleDeg"] >= StockClosing + 1),
            "Samples": Sweep,
        },
        "Grasps": Grasps,
        "GripForce": {
            "AssumedStallTorqueNm": round(TorqueNm, 3), "Assumption": "STS3215 12 V, 30 kg-cm stall at the jaw (1:1)",
            "NewPadLeverMm": round(PadLever, 1), "NewPadForceAtStallN": round(TorqueNm * 1000 / PadLever, 1),
            "StockTipLeverMm": round(StockTipLever, 1), "StockTipForceAtStallN": round(TorqueNm * 1000 / StockTipLever, 1),
        },
        "Masses": Masses,
        "MountingRegionsIdenticalToUpstream": Mounting,
        "Print": {
            "FixedJawBody": {"Orientation": "as exported: base (z = 0) on the bed, fingers up, same as stock",
                             "OverhangMm2": OverhangArea(BodyMesh), "StockOverhangMm2": OverhangArea(StockBodyMesh)},
            "MovingJaw": {"Orientation": "as exported: horn-side plate (local z = -24) on the bed; the finger lies flat",
                          "OverhangMm2": OverhangArea(JawMesh), "StockOverhangMm2": OverhangArea(StockJawMesh)},
            "Note": "Overhang = downward faces flatter than 45 deg above the bed; upstream prints with supports everywhere.",
        },
    }
    (Output / "Verification.json").write_text(Json.dumps(Report, indent=2, default=float) + "\n")
    Shutil.copy2(SourceRoot / "Original/LICENSE", Output / "License.txt")

    # Poses for the renders.
    Poses = {"Closed": StockClosing, "Open": 90.0, "Grasps": {}}
    for D in (66, 40, 20):
        G = Grasps["New"][f"Cylinder{D}mm"]
        if G:
            Poses["Grasps"][str(D)] = {"Angle": G["JawAngleDeg"], "CentreXZ": G["CentreXZ"]}
    Poses["JawToBody"] = {str(A): JawToBody(A).tolist() for A in [StockClosing, 90.0] + [G["Angle"] for G in Poses["Grasps"].values()]}
    Poses["ServoInBodyMetres"] = (Frames["ServoInBody"] @ Np.diag([1000.0, 1000.0, 1000.0, 1.0])).tolist()
    (Output / "RenderPoses.json").write_text(Json.dumps(Poses, indent=2))
    if BlenderExe.exists() and "--no-render" not in Sys.argv:
        Subprocess.run([str(BlenderExe), "-b", "--python", str(Path(__file__).with_name("RenderLargeGripper.py"))], check=True,
                       stdout=Subprocess.DEVNULL)
        ComposeRow([f"Grasp{D}mmFront.png" for D in Poses["Grasps"]], "LargeGripperGrasps.png")
        ComposeRow(["StockClosedFront.png", "LargeGripperClosedFront.png", "OverlayStockFront.png"], "LargeGripperVsStock.png")
    print(Json.dumps({K: Report[K] for K in ("Dimensions", "Travel", "GripForce", "Masses", "Print")}, indent=2), flush=True)
    Sys.stdout.flush()
    Os._exit(0)
