"""Depth-camera geometry: pixels plus metric depth to 3D, objects, and the camera pose.

Frames (all cm)
---------------
Camera: OpenCV, +X right, +Y down, +Z forward.
Display: the AI's frame, +X the arm's left, +Y farther, +Z up. It is a mirror
image of the physical world, so a calibrated camera model stores the pose as
    Pc = R @ (S @ Pw) + t,   S = diag(-1, 1, 1) when mirror_x else identity,
exactly like SelfCalibration's pinhole models, and Pw = S @ R.T @ (Pc - t).

Pixels
------
Depth arrives aligned to the color image, so one intrinsic set serves both.
Native pixels are OpenCV's (pixel centres on integers). Browser captures are
continuous and may be mirrored left-right; NativeFromCapture converts.
"""

from __future__ import annotations

import math
from typing import Any

import cv2
import numpy as np
from scipy.optimize import least_squares

MirrorVector = np.array([-1.0, 1.0, 1.0])

# Object segmentation: points this far above the (locally measured) support
# belong to an object; about three times the sensor's noise at arm's length.
ObjectMinHeightCm = 0.5
# The support is measured in a ring this fraction of the box size around it,
# among points within RingSearchCm of the expected height.
RingPadFraction = 0.35
RingSearchCm = 1.5
RingMinPoints = 50
ObjectMaxHeightCm = 40.0
ObjectMinPoints = 25
# Depth band (cm) under an object's top that counts as its top surface: about
# the sensor's noise at arm's-length range.
TopBandCm = 0.5
# Depth at a small blob: values within this band of the near cluster count.
ForegroundBandCm = 1.5
# Depth beyond this is background (the room floor, far walls), never a workspace object.
MaxDepthCm = 120.0
# A box needs usable depth over this share of its pixels: the 1-5% boxes of 2026-09-28
# became objects 225-553 cm away. The tape in the basket read 0.23-0.24; tune from logs.
MinValidFraction = 0.3
# Points this close to the arm's link centre lines (forward kinematics) are the arm; a
# detection whose points are mostly arm is the arm (the robot measured itself as "robot
# arm" at 130-147 cm, and its forearm made a cup 28 cm tall).
ArmRadiusCm = 3.0
MaxArmFraction = 0.5
# Never a support: this far below the table is the room floor.
MaxSupportDropCm = 20.0
# One component filling this much of a box with no support pixels in it: the box saw a
# misplaced desk as the object (the 7.5 cm tall eraser of 19:04:11).
WholeBoxFraction = 0.85

# Workspace surfaces, display frame (cm), measured on 2026-09-28: the desk's front edge
# at Y 25.5, the bin's far rim at Y 22.5 (top about Z 1), its floor about -8.5, |X| < 23.
DeskFrontEdgeYCm = 25.5
BinRimYCm = 22.5
BinRimTopZCm = 1.0
BinHalfWidthCm = 23.0
BinFloorZCm = -8.5
# Contacts between these Y values are near the rim and the edge, where a small pose error
# swaps desk and bin: there no stacked support is accepted, only the expected surface.
AmbiguousContactYCm = (21.0, 27.0)
DeskSupportToleranceCm = 1.5
BinSupportToleranceCm = 1.0
DeskOffsetSearchCm = 8.0

# Surfaces besides the desk and the bin count only where they are declared (DeclaredSurfaces.json):
# depth alone cannot tell a platform from a desk that moved or a camera that drooped (the 19:00:57 desk
# read 4.6 cm low). A box is over a declared surface when the ray through its bottom-centre or its
# centre pixel meets the surface's plane within DeclaredRayMarginCm of its extent: over a solid platform
# that pixel shows the platform or what stands on it, not the desk below.
DeclaredRayMarginCm = 2.0
# The surface is searched this far from its declared (hand-measured) height, in the ring over its
# extent grown by DeclaredRingMarginCm, and the object must touch it within DeclaredContactMarginCm.
DeclaredToleranceCm = 2.0
DeclaredRingMarginCm = 1.0
DeclaredContactMarginCm = 2.0
# Over no declared surface, an object no desk or bin supports may stand on a surface nobody declared:
# withheld as before, that surface's height beside the reason. A level peak of the ring's heights is such
# a surface unless more than UndeclaredBelowShare of the depth just under the object's bottom edge lies
# lower (a cut through it).
UndeclaredBelowShare = 0.5
UndeclaredBelowCm = 1.0
# A surface is level points covering an area: on a 1 cm grid, at least this many cells inside it.
LevelCellCm = 1.0
LevelMinInteriorCells = 4
# With no calibrated desk in view a measured support need not surround the object's footprint: books, a wall or a
# neighbour standing against it hide the support on that side (the pill bottle on the bookshelf cabinet of 2026-10-25,
# the cabinet top at Z 3.3 reaching Y 42.38 behind it, its footprint 42.45). It is enough that the level patch is
# measured right beside the footprint (within SupportSideBandCm outside it, or SupportSideInsideCm under its edge) on at
# least SupportMinSides of its four sides, each side's band holding SupportSideMinPoints of the patch over
# SupportSideCover of that side's length (in SupportSideBinCm bins) and SupportSideDepthCm outward (an area; the surface
# seen inside the box counts from SupportSideClearCm outside the footprint, never the object's own side), and that the
# object reaches down to it: the lowest 5% of its points within SupportBaseGapCm of the surface (otherwise it floats, is
# held up, or its base is hidden). At a calibrated desk the patch must still lie around the whole footprint.
SupportSideBandCm = 2.0
SupportSideInsideCm = 0.5
SupportSideMinPoints = 25
SupportSideCover = 0.5
SupportSideBinCm = 0.5
SupportSideDepthCm = 1.0
SupportSideClearCm = 0.5
SupportMinSides = 2
SupportBaseGapCm = 1.5

# Depth-aware camera fit noise model. Pixels are the tape centroid; depth adds
# stereo noise that grows with distance.
FitSigmaPx = 1.5
FitSigmaLateralCm = 0.5
FitSigmaDepthBaseCm = 0.2
FitSigmaDepthPerCm = 0.005


def CameraMatrix(Intrinsics: dict[str, Any]) -> np.ndarray:
    return np.array([[Intrinsics["fx"], 0.0, Intrinsics["cx"]],
                     [0.0, Intrinsics["fy"], Intrinsics["cy"]],
                     [0.0, 0.0, 1.0]], dtype=np.float64)


def Distortion(Intrinsics: dict[str, Any]) -> np.ndarray:
    return np.array(Intrinsics.get("dist") or [0.0] * 5, dtype=np.float64)


def NativeFromCapture(U, V, Width: int, Mirrored: bool):
    """Continuous capture pixel (maybe mirrored) to native OpenCV pixel."""
    U = np.asarray(U, dtype=np.float64)
    V = np.asarray(V, dtype=np.float64)
    return (Width - U if Mirrored else U) - 0.5, V - 0.5


def BoxToNative(Box, Width: int, Height: int, Mirrored: bool) -> tuple[int, int, int, int]:
    """Normalized capture box (x1, y1, x2, y2) to an inclusive-exclusive native pixel box."""
    X1, Y1, X2, Y2 = (float(Value) for Value in Box)
    if Mirrored:
        X1, X2 = 1.0 - X2, 1.0 - X1
    U0 = int(np.clip(math.floor(min(X1, X2) * Width), 0, Width - 1))
    U1 = int(np.clip(math.ceil(max(X1, X2) * Width), U0 + 1, Width))
    V0 = int(np.clip(math.floor(min(Y1, Y2) * Height), 0, Height - 1))
    V1 = int(np.clip(math.ceil(max(Y1, Y2) * Height), V0 + 1, Height))
    return U0, V0, U1, V1


_RayCache: dict[tuple, np.ndarray] = {}


def RayMap(Intrinsics: dict[str, Any]) -> np.ndarray:
    """Undistorted normalized ray (x/z, y/z) for every native pixel, cached per camera."""
    Key = (Intrinsics["width"], Intrinsics["height"], Intrinsics["fx"], Intrinsics["fy"],
           Intrinsics["cx"], Intrinsics["cy"], tuple(Distortion(Intrinsics)))
    Rays = _RayCache.get(Key)
    if Rays is None:
        Width, Height = int(Intrinsics["width"]), int(Intrinsics["height"])
        Us, Vs = np.meshgrid(np.arange(Width, dtype=np.float64), np.arange(Height, dtype=np.float64))
        Pixels = np.stack([Us.ravel(), Vs.ravel()], axis=-1).reshape(-1, 1, 2)
        Rays = cv2.undistortPoints(Pixels, CameraMatrix(Intrinsics), Distortion(Intrinsics)).reshape(Height, Width, 2)
        _RayCache.clear()
        _RayCache[Key] = Rays.astype(np.float32)
        Rays = _RayCache[Key]
    return Rays


def Deproject(U, V, DepthCm, Intrinsics: dict[str, Any]) -> np.ndarray:
    """Native pixels with depth (cm, along the optical axis) to camera points (N x 3, cm)."""
    Pixels = np.stack([np.atleast_1d(np.asarray(U, dtype=np.float64)),
                       np.atleast_1d(np.asarray(V, dtype=np.float64))], axis=-1).reshape(-1, 1, 2)
    Rays = cv2.undistortPoints(Pixels, CameraMatrix(Intrinsics), Distortion(Intrinsics)).reshape(-1, 2)
    Depth = np.atleast_1d(np.asarray(DepthCm, dtype=np.float64))
    return np.column_stack([Rays * Depth[:, None], Depth])


def Project(Points: np.ndarray, Intrinsics: dict[str, Any]) -> np.ndarray:
    """Camera points (N x 3, cm) to native pixels (N x 2)."""
    Points = np.asarray(Points, dtype=np.float64).reshape(-1, 3)
    Pixels, _ = cv2.projectPoints(Points, np.zeros(3), np.zeros(3), CameraMatrix(Intrinsics), Distortion(Intrinsics))
    return Pixels.reshape(-1, 2)


def ForegroundDepthCm(Values: np.ndarray) -> float | None:
    """Robust depth of the nearest surface among valid samples (cm), or None.

    A small blob's window mixes its own surface with whatever lies behind it.
    The near cluster, anchored at the 20th percentile, is the blob.
    """
    Values = np.asarray(Values, dtype=np.float64)
    Values = Values[np.isfinite(Values) & (Values > 0)]
    if Values.size < 3:
        return None
    Anchor = float(np.percentile(Values, 20))
    Near = Values[np.abs(Values - Anchor) <= ForegroundBandCm + 0.01 * Anchor]
    return float(np.median(Near)) if Near.size else None


def DepthAtBlob(DepthMm: np.ndarray, Contour: np.ndarray | None, Centroid, AreaPx: float | None = None) -> float | None:
    """Depth (cm) of a detected blob: its contour pixels, else a window at the centroid."""
    Height, Width = DepthMm.shape[:2]
    if Contour is not None and len(Contour) >= 3:
        Mask = np.zeros((Height, Width), dtype=np.uint8)
        cv2.drawContours(Mask, [np.asarray(Contour, dtype=np.int32)], -1, 1, thickness=-1)
        Eroded = cv2.erode(Mask, np.ones((3, 3), np.uint8))
        if Eroded.sum() >= 6:
            Mask = Eroded
        Result = ForegroundDepthCm(DepthMm[Mask.astype(bool)] / 10.0)
        if Result is not None:
            return Result
    U, V = (float(Value) for Value in Centroid)
    Radius = int(max(3, min(15, 0.5 * math.sqrt(max(AreaPx or 50.0, 1.0) / math.pi))))
    U0, U1 = max(0, int(round(U)) - Radius), min(Width, int(round(U)) + Radius + 1)
    V0, V1 = max(0, int(round(V)) - Radius), min(Height, int(round(V)) + Radius + 1)
    if U0 >= U1 or V0 >= V1:
        return None
    return ForegroundDepthCm(DepthMm[V0:V1, U0:U1].ravel() / 10.0)


def PoseFromModel(Model: dict[str, Any]) -> tuple[np.ndarray, np.ndarray, bool]:
    Rotation, _ = cv2.Rodrigues(np.asarray(Model["rvec"], dtype=np.float64))
    return Rotation, np.asarray(Model["tvec"], dtype=np.float64), bool(Model.get("mirror_x"))


def CameraToDisplay(Points: np.ndarray, Model: dict[str, Any]) -> np.ndarray:
    Rotation, Translation, Mirror = PoseFromModel(Model)
    World = (np.asarray(Points, dtype=np.float64).reshape(-1, 3) - Translation) @ Rotation
    return World * MirrorVector if Mirror else World


def DisplayToCamera(Points: np.ndarray, Model: dict[str, Any]) -> np.ndarray:
    Rotation, Translation, Mirror = PoseFromModel(Model)
    World = np.asarray(Points, dtype=np.float64).reshape(-1, 3)
    if Mirror:
        World = World * MirrorVector
    return World @ Rotation.T + Translation


def ModelUsesDepthCamera(Model: dict[str, Any] | None, Intrinsics: dict[str, Any] | None) -> bool:
    """True when the saved camera model was fitted to this depth camera's images."""
    if not Model or not Intrinsics or Model.get("type") != "pinhole_depth_v1":
        return False
    return list(Model.get("image_size") or []) == [int(Intrinsics["width"]), int(Intrinsics["height"])]


def WorkspaceSurfaces(DeskPlane=None, TableZ: float | None = None, Declared=None, Automatic: bool = False,
                      Desk: bool = True, DeskDiagnostics: bool | None = None) -> dict[str, Any]:
    """The surfaces a support may be: the desk, as the display plane m.P = e with m up (the
    calibrated, sloped desk; flat at TableZ when none is known), the bin floor, and the declared
    surfaces (DeclaredSurfaces.Load: horizontal at TopZCm over their X and Y range, display frame).

    Desk False: the capture's camera check found no calibrated desk in view and checked the camera
    against the robot (RobotReference). The desk is then no support and has no diagnostics (MeasureBox):
    objects stand on the bin floor, a declared surface or, with Automatic, the level surface measured
    around them. Its plane stays as the cart-fixed height the room floor is bounded by.

    DeskDiagnostics (default: Desk): measure the scene tripwire's desk readings (desk_area, the ring's
    offset from the desk) even where the desk is no support. True for a capture whose camera the gripper
    tip could not check ("assumed"): it is not verified, so the tripwire still runs on it (Aaran's
    decision, 2026-10-07 audit IH1)."""
    Normal, Offset = DeskPlane if DeskPlane is not None else (np.array([0.0, 0.0, 1.0]), float(TableZ))
    return {"desk": (np.asarray(Normal, dtype=np.float64), float(Offset)), "bin_floor_z": BinFloorZCm,
            "declared": [dict(Surface) for Surface in (Declared or [])], "automatic": bool(Automatic), "desk_support": bool(Desk),
            "desk_diagnostics": bool(Desk if DeskDiagnostics is None else DeskDiagnostics)}


def DeskSupport(Surfaces: dict[str, Any] | None) -> bool:
    """Whether the calibrated desk is a support in these surfaces (WorkspaceSurfaces Desk)."""
    return Surfaces is None or bool(Surfaces.get("desk_support", True))


def DeskDiagnostics(Surfaces: dict[str, Any] | None) -> bool:
    """Whether MeasureBox takes the scene tripwire's desk readings (WorkspaceSurfaces DeskDiagnostics)."""
    return Surfaces is None or bool(Surfaces.get("desk_diagnostics", DeskSupport(Surfaces)))


def _Over(Surface: dict[str, Any], X, Y, MarginCm: float):
    """Whether display X, Y lies over a declared surface's extent grown by MarginCm (arrays alike)."""
    (X0, X1), (Y0, Y1) = Surface["XRangeCm"], Surface["YRangeCm"]
    X, Y = np.asarray(X, dtype=np.float64), np.asarray(Y, dtype=np.float64)
    return (X >= X0 - MarginCm) & (X <= X1 + MarginCm) & (Y >= Y0 - MarginCm) & (Y <= Y1 + MarginCm)


def SurfaceText(Surface: dict[str, Any]) -> str:
    """"platform 'box platform' (Z 12.7)", as the reasons name a declared surface."""
    return f"{Surface['Kind']} '{Surface['Name']}' (Z {Surface['TopZCm']:.1f})"


def DeskHeight(Surfaces: dict[str, Any], X, Y):
    """Desk Z under display X, Y."""
    Normal, Offset = Surfaces["desk"]
    return (Offset - Normal[0] * np.asarray(X, dtype=np.float64) - Normal[1] * np.asarray(Y, dtype=np.float64)) / Normal[2]


def ArmExclusion(Joints, RadiusCm: float = ArmRadiusCm):
    """Mask function over display points: True within RadiusCm of the arm's link centre lines.

    Joints are the display-frame joint positions [base, shoulder, elbow, wrist, roll, tip].
    """
    Joints = np.asarray(Joints, dtype=np.float64).reshape(-1, 3)

    def Mask(Points):
        Points = np.asarray(Points, dtype=np.float64).reshape(-1, 3)
        Near = np.zeros(len(Points), dtype=bool)
        for Start, End in zip(Joints[:-1], Joints[1:]):
            Segment = End - Start
            Length = float(Segment @ Segment)
            Along = np.clip((Points - Start) @ Segment / Length, 0.0, 1.0) if Length > 1e-9 else np.zeros(len(Points))
            Gap = Points - Start - Along[:, None] * Segment
            Near |= np.einsum("ij,ij->i", Gap, Gap) <= RadiusCm ** 2
        return Near
    return Mask


def _Flat(Level: float):
    return lambda X, Y: np.full(np.shape(X), float(Level))


def MeasureBox(DepthMm: np.ndarray, Box, Intrinsics: dict[str, Any], Model: dict[str, Any],
               TableZ: float, Mirrored: bool, Surfaces: dict[str, Any] | None = None, Exclude=None,
               Only: np.ndarray | None = None, Trace: dict[str, Any] | None = None) -> dict[str, Any]:
    """Measure the object inside a normalized capture box with metric depth (_MeasureBox).

    Only (a native, full-frame boolean mask: the model's outline of the object) keeps the object's
    pixels inside it; the support is still measured around and inside the whole box. Trace, a dict,
    receives "component_px": the measured object's pixels' native bounding box [u0, v0, u1, v1]
    (exclusive ends), for the single-call locate's depth refine (object_locator). Without them the
    result is exactly as before."""
    Result = _MeasureBox(DepthMm, Box, Intrinsics, Model, TableZ, Mirrored, Surfaces, Exclude,
                         None if Only is None else np.asarray(Only, dtype=bool))
    Component = Result.pop("_component", None)
    if Trace is not None:
        Trace.pop("component_px", None)
        if Component is not None and Result.get("ok"):
            U0, V0, _, _ = BoxToNative(Box, DepthMm.shape[1], DepthMm.shape[0], Mirrored)
            Trace["component_px"] = [Component[0] + U0, Component[1] + V0, Component[2] + U0, Component[3] + V0]
    return Result


def _MeasureBox(DepthMm: np.ndarray, Box, Intrinsics: dict[str, Any], Model: dict[str, Any],
                TableZ: float, Mirrored: bool, Surfaces: dict[str, Any] | None = None, Exclude=None,
                Only: np.ndarray | None = None) -> dict[str, Any]:
    """Measure the object inside a normalized capture box with metric depth.

    Pixels above the support form candidate object points; the connected region
    nearest the box centre is the object (a box also catches table and
    whatever stands behind). The centre comes from the object's top surface,
    which an elevated camera sees whole, so it is not biased toward the camera
    the way the visible side faces are.

    The support is measured, never assumed: in a ring around the box, or inside
    it. With Surfaces (WorkspaceSurfaces) it is chosen by where the object touches
    it: the calibrated, sloped desk beyond the desk's front edge; the bin floor, or
    an object standing on it, inside the bin; over a declared surface (a platform,
    a box top, a shelf board: DeclaredSurfaces.json) that surface at the height the
    camera measures there, support_branch "surface" and support_surface its name.
    With automatic surfaces enabled after the camera gate, a unique level patch
    surrounding the object's footprint supplies its local support geometry.
    Otherwise undeclared_support_z is only a hint. Without Surfaces, the table, or the
    lowest surface when the box reaches well below it, never the room floor.
    Exclude (ArmExclusion) removes the arm's own points first. A failure says why.
    """
    Height, Width = DepthMm.shape[:2]
    U0, V0, U1, V1 = BoxToNative(Box, Width, Height, Mirrored)
    Crop = DepthMm[V0:V1, U0:U1].astype(np.float64) / 10.0
    Valid = (Crop > 0) & (Crop <= MaxDepthCm)
    Only = Only[V0:V1, U0:U1] if Only is not None else None
    Result: dict[str, Any] = {"ok": False, "valid_fraction": round(float(Valid.mean()) if Crop.size else 0.0, 3),
                              "support_measured": False, "support_z_measured": None, "support_branch": None}
    if Valid.sum() < ObjectMinPoints:
        Result["error"] = "no depth inside the box"
        return Result
    if Result["valid_fraction"] < MinValidFraction:
        Result["error"] = f"only {Result['valid_fraction']:.0%} of the box has depth within {MaxDepthCm:g} cm"
        return Result
    Rays = RayMap(Intrinsics)[V0:V1, U0:U1]
    Camera = np.concatenate([Rays * Crop[..., None], Crop[..., None]], axis=-1)
    World = CameraToDisplay(Camera.reshape(-1, 3), Model).reshape(Crop.shape + (3,))
    ArmPoints = 0
    if Exclude is not None:
        Arm = Valid & Exclude(World.reshape(-1, 3)).reshape(Crop.shape)
        ArmPoints = int(Arm.sum())
        Valid &= ~Arm
        if Valid.sum() < ObjectMinPoints:
            Result.update(error="the box shows only the robot arm", robot_arm=True, arm_fraction=1.0)
            return Result
    Ring = _RingPoints(DepthMm, (U0, V0, U1, V1), Intrinsics, Model, Exclude)
    Lowest = float(np.percentile(World[..., 2][Valid], 5))
    Defaults: list[tuple[str, Any, dict[str, Any] | None]] = []
    Declared: list[tuple[str, Any, dict[str, Any] | None]] = []
    if Surfaces is None:
        # Objects stand on the table, unless the box reaches well below it (the basket).
        Basket = TableZ - MaxSupportDropCm <= Lowest < TableZ - 1.5
        Defaults = [("bin", _Flat(Lowest), None)] if Basket else [("desk", _Flat(TableZ), None)]
    else:
        # No calibrated desk in view (WorkspaceSurfaces Desk False): no desk branch, and no desk diagnostics
        # unless the tripwire still runs (an "assumed" camera, WorkspaceSurfaces DeskDiagnostics).
        OnDesk = DeskSupport(Surfaces)
        Diagnosed = OnDesk or DeskDiagnostics(Surfaces)
        if Diagnosed:
            Result.update(_DeskDiagnostics(Ring, Surfaces, (U0, V0, U1, V1), Intrinsics, Model))
        Defaults = ([("desk", lambda X, Y: DeskHeight(Surfaces, X, Y), None)] if OnDesk else []) + [("bin", _Flat(Surfaces["bin_floor_z"]), None)]
        if Surfaces["bin_floor_z"] + BinSupportToleranceCm < Lowest <= BinRimTopZCm:
            Defaults.append(("object", _Flat(Lowest), None))
        Under = _DeclaredUnder(Surfaces, (U0, V0, U1, V1), Intrinsics, Model)
        DeskArea = Result.get("desk_area")
        if Under:
            # Not the desk's area: the scene tripwire counts only objects on the desk.
            Result.update(declared_surface=Under[0][0]["Name"], **({"desk_area": False} if Diagnosed else {}))
            Declared = [("surface", _Flat(_SurfaceLevel(Ring, Surface)), Surface) for Surface, _ in Under]
        elif Surfaces.get("automatic"):
            # Try a measured support before the desk: otherwise a can on a box
            # can become one tall object including the box beneath it. Multiple
            # plausible support levels are ambiguous, never resolved by guessing.
            Candidates = _SupportCandidates(World, Valid, Crop, Ring, Surfaces, Only)
            if len(Candidates) > 1:
                Result.update(error="more than one measured surface could support this object", support_branch="surface")
                return Result
            if Candidates:
                Candidate = Candidates[0]
                Result.update(Candidate["measured"])
                Result["support_geometry"] = Candidate["support_geometry"]
                Result["support_surface"] = Candidate["support_geometry"]["Name"]
                Result["support_kind"] = Candidate["support_geometry"]["Kind"]
                Result["automatic_surface"] = True
                if Diagnosed:
                    Result["desk_area"] = False
                if Exclude is not None:
                    Result["arm_fraction"] = round(ArmPoints / max(ArmPoints + Result.get("points", 0), 1), 3)
                    if ArmPoints >= ObjectMinPoints and Result["arm_fraction"] > MaxArmFraction:
                        Result.update(ok=False, error="the robot's arm makes up most of the box", robot_arm=True)
                return Result
    Failures = []
    for Branch, Prior, Surface in Declared:
        # Its ring is the part over its extent: the desk beside a low platform is no part of it.
        Over = Ring[_Over(Surface, Ring[:, 0], Ring[:, 1], DeclaredRingMarginCm)] if len(Ring) else Ring
        Outcome = _MeasureOn(World, Valid, Crop, Over, Branch, Prior, Surfaces, Surface, Only)
        if Outcome["ok"]:
            Result.update(Outcome)
            break
        Failures.append(Outcome)
    else:
        # Over a raised declared surface only that surface is tried: measured on the desk, a can on a 15 cm
        # box was the box and the can. A ray meets the plane of a surface below the desk beyond the desk's
        # objects too, so there the desk and the bin are tried next (the reason stays the declared surface's
        # when it was seen): a desk object measured short over it is never measured lower than it stands.
        Raised = any(Raises for _, Raises in Under) if Declared else False
        if not Raised:
            Seen = [Failure for Failure in Failures if Failure["support_found"]]
            Failures = []
            for Branch, Prior, Surface in Defaults:
                Outcome = _MeasureOn(World, Valid, Crop, Ring, Branch, Prior, Surfaces, Surface, Only)
                if Outcome["ok"]:
                    Result.update(Outcome)
                    if Declared:  # on the desk after all: the tripwire counts it again
                        Result.pop("declared_surface", None)
                        if DeskArea is not None:
                            Result["desk_area"] = DeskArea
                    break
                Failures.append(Outcome)
            Failures = Seen + Failures
        if not Result["ok"]:
            # One that found its support says more than "no support" from the others.
            Result.update(next((Failure for Failure in Failures if Failure["support_found"]), Failures[0]))
            Result.pop("support_found", None)
            if Surfaces is not None and not DeskSupport(Surfaces) and Result["support_branch"] is None and not Declared:
                # No calibrated desk in view and no level surface measured around it: "no bin floor" would name the wrong one.
                Result["error"] = NoDeskSupportError
            # Over no declared surface: the height of a level surface that holds it, if one does (still withheld,
            # its reason and branch as they were: on the recorded desks of 2026-10-03/04 such a "surface" was as
            # often a neighbour's top or the arm, so it only goes with the record and a task object's refusal).
            Elsewhere = _UndeclaredSupport(World, Valid, Crop, Ring, Surfaces, Only) if Surfaces is not None and not Declared else None
            if Elsewhere:
                Result["undeclared_support_z"] = Elsewhere["undeclared_support_z"]
    if Exclude is not None:
        # A detection whose points are mostly on the arm is the arm (or hidden behind it);
        # a failure to find the support keeps its own reason.
        Result["arm_fraction"] = round(ArmPoints / max(ArmPoints + Result.get("points", 0), 1), 3)
        if (ArmPoints >= ObjectMinPoints and Result["arm_fraction"] > MaxArmFraction
                and (Result["ok"] or Result["support_branch"] is not None)):
            Result.update(ok=False, error="the robot's arm makes up most of the box", robot_arm=True)
    return Result


_NoSupport = {"desk": "no desk surface around the object at the calibrated height",
              "bin": "no bin floor around the object at the calibrated height",
              "object": "no measured surface under the object in the bin",
              "surface": "no surface around the object at a height the camera measures"}
NoDeskSupportError = "no level surface measured around the object to stand on (no calibrated desk in view)"


def _MeasureOn(World, Valid, Crop, Ring, Branch: str, Prior, Surfaces, Surface: dict[str, Any] | None = None,
               Only=None) -> dict[str, Any]:
    """The object above one support hypothesis, or why not ({"ok": False, "error"}).

    Branch "surface": a declared surface (Surface), or with none a surface nobody declared,
    which must not cut through the object (_UndeclaredSupport)."""
    Outcome: dict[str, Any] = {"ok": False, "support_found": False}
    Local = RingZ = None
    if len(Ring):
        Residual = Ring[:, 2] - Prior(Ring[:, 0], Ring[:, 1])
        Near = np.abs(Residual) <= RingSearchCm
        # A surface besides the desk and bin is a level patch around the object, not a band across a wall.
        if Near.sum() >= RingMinPoints and (Branch != "surface" or _Level(Ring[Near])):
            Local, RingZ = float(np.median(Residual[Near])), float(np.median(Ring[Near, 2]))
    Above = World[..., 2] - Prior(World[..., 0], World[..., 1]) - (Local or 0.0)
    TableMask = Valid & (np.abs(Above) <= ObjectMinHeightCm)
    # Measure the support around the box, or at least inside it: the calibrated height can
    # be off by millimetres, which decides whether a thin object (a keyboard) stands out
    # or the bare table does. A support nobody measured is never assumed; a surface besides
    # the desk is measured around the box (inside it, a can's side is level with any height).
    if (Local is None and TableMask.sum() < ObjectMinPoints) or (Branch in ("object", "surface") and Local is None):
        Outcome["error"] = (f"no surface around the object at the height of the declared {SurfaceText(Surface)}: the camera sees none "
                            f"within {DeclaredToleranceCm:g} cm of it there" if Surface else _NoSupport[Branch])
        return Outcome
    if Surfaces is not None and Branch == "bin" and abs(Local or 0.0) > BinSupportToleranceCm:
        Outcome["error"] = f"the surface under the object is {Local:+.1f} cm from the calibrated bin floor"
        return Outcome
    Level = float(Prior(0.0, 0.0)) + (Local or 0.0) if Branch == "surface" else None
    if Surface and abs(Level - Surface["TopZCm"]) > DeclaredToleranceCm:
        Outcome["error"] = f"the surface under the object is {Level - Surface['TopZCm']:+.1f} cm from the declared {SurfaceText(Surface)}"
        return Outcome
    Outcome.update(support_found=True, support_measured=Local is not None, support_branch=Branch,
                   support_z_measured=round(float(np.median(World[..., 2][TableMask])), 2)
                   if TableMask.sum() >= ObjectMinPoints else round(RingZ, 2))
    if Branch == "surface":
        Outcome.update(support_surface=Surface["Name"] if Surface else None, support_kind=Surface["Kind"] if Surface else None)
    if Branch == "desk" and Local is not None:
        Outcome["desk_offset_cm"] = round(Local, 2)
    ObjectMask = Valid & (Above > ObjectMinHeightCm) & (Above < ObjectMaxHeightCm)
    if Branch == "surface" and Surface is None:
        # A distant shelf back can touch the target in image space. Break those
        # depth discontinuities before choosing a component; they are not one
        # object merely because both project above its supporting plane.
        Discontinuous = np.zeros_like(ObjectMask)
        Jump = (np.abs(np.diff(Crop, axis=0)) > 2.0) & Valid[:-1] & Valid[1:]
        Discontinuous[:-1] |= Jump
        Discontinuous[1:] |= Jump
        Jump = (np.abs(np.diff(Crop, axis=1)) > 2.0) & Valid[:, :-1] & Valid[:, 1:]
        Discontinuous[:, :-1] |= Jump
        Discontinuous[:, 1:] |= Jump
        ObjectMask &= ~Discontinuous
    if Only is not None:
        ObjectMask &= Only  # the model's outline of the object (MeasureBox Only)
    Count, Labels, Stats, Centroids = cv2.connectedComponentsWithStats(ObjectMask.astype(np.uint8), connectivity=8)
    if Count <= 1:
        Outcome["error"] = f"nothing stands above the {'surface' if Branch == 'surface' else 'table'} inside the box"
        return Outcome
    BoxCentre = np.array([Crop.shape[1] / 2.0, Crop.shape[0] / 2.0])
    Diagonal = float(np.hypot(*Crop.shape))
    Best, BestScore = None, -np.inf
    for Label in range(1, Count):
        Area = float(Stats[Label, cv2.CC_STAT_AREA])
        if Area < ObjectMinPoints:
            continue
        Offset = float(np.linalg.norm(Centroids[Label] - BoxCentre)) / max(Diagonal, 1.0)
        Score = math.log(Area) - 4.0 * Offset
        if Score > BestScore:
            Best, BestScore = Label, Score
    if Best is None:
        Outcome["error"] = "object region too small in depth"
        return Outcome
    Component = Labels == Best
    if Component.sum() > WholeBoxFraction * Valid.sum() and TableMask.sum() < ObjectMinPoints:
        Outcome["error"] = "the whole box stands above the support: the support is not where the calibration expects it"
        return Outcome
    Points, Rises = World[Component], Above[Component]
    # Where the object touches its support must be where that support is.
    Bottom = Rises <= np.percentile(Rises, 10) + 0.5
    ContactX, ContactY = float(np.median(Points[Bottom, 0])), float(np.median(Points[Bottom, 1]))
    Outcome["contact_cm"] = [round(ContactX, 2), round(ContactY, 2)]
    Mismatch = _ContactMismatch(Branch, ContactX, ContactY, Surface) if Surfaces is not None else None
    if Mismatch:
        Outcome["error"] = Mismatch
        return Outcome
    if Branch == "surface" and not Surface and _CutsThrough(Component, Valid, Above):
        Outcome["error"] = "the surface found cuts through the object: the object goes on below it"
        return Outcome
    if (Branch == "surface" and not Surface and Surfaces is not None and not DeskSupport(Surfaces)
            and float(np.percentile(Rises, 5)) > SupportBaseGapCm):
        # No desk (SupportSides): the object must reach down to the surface that holds it.
        Outcome["error"] = (f"the object's lowest points are {float(np.percentile(Rises, 5)):.1f} cm above the surface measured under it: "
                            "it does not stand on it, or its base is hidden")
        return Outcome
    Rise = float(np.percentile(Rises, 95))
    Base = float(Prior(ContactX, ContactY)) + (Local or 0.0)
    # A thicker band would take in the upper strip of the side faces the
    # camera sees and pull the centre toward the camera.
    TopSurface = Points[Rises >= Rise - TopBandCm]
    if len(TopSurface) < ObjectMinPoints:
        TopSurface = Points[Rises >= Rise - max(1.0, 0.25 * Rise)]
    Centre = np.median(TopSurface[:, :2], axis=0)
    Low, High = np.percentile(Points[:, :2], 5, axis=0), np.percentile(Points[:, :2], 95, axis=0)
    Outcome.update({
        "ok": True,
        "x": round(float(Centre[0]), 2),
        "y": round(float(Centre[1]), 2),
        "z": round(Base, 2),
        "top_z": round(Base + Rise, 2),
        "height_cm": round(Rise, 2),
        "size_cm": [round(float(High[0] - Low[0]), 1), round(float(High[1] - Low[1]), 1)],
        "points": int(len(Points)),
        "distance_cm": round(float(np.median(Crop[Component])), 1),
        "bounds_cm": {"min": np.round(Points.min(axis=0), 3).tolist(),
                      "max": np.round(Points.max(axis=0), 3).tolist()},
    })
    # The component's pixel bounds in the crop, for MeasureBox's Trace (popped there, never in a result).
    Rows, Columns = np.nonzero(Component.any(axis=1))[0], np.nonzero(Component.any(axis=0))[0]
    Outcome["_component"] = [int(Columns[0]), int(Rows[0]), int(Columns[-1]) + 1, int(Rows[-1]) + 1]
    Outcome.pop("support_found")
    return Outcome


def _ContactMismatch(Branch: str, X: float, Y: float, Surface: dict[str, Any] | None = None) -> str | None:
    """Why a support cannot be the one under a contact at display X, Y, or None."""
    if Branch == "surface":
        if Surface and not bool(_Over(Surface, X, Y, DeclaredContactMarginCm)):
            (X0, X1), (Y0, Y1) = Surface["XRangeCm"], Surface["YRangeCm"]
            return (f"the object meets the surface at X {X:.1f} Y {Y:.1f}, outside the declared {SurfaceText(Surface)}, "
                    f"which covers X {X0:g}..{X1:g}, Y {Y0:g}..{Y1:g}")
        return None
    InBin = Y <= BinRimYCm and abs(X) <= BinHalfWidthCm
    if Branch == "desk" and Y < DeskFrontEdgeYCm:
        return f"the object meets the desk surface at Y {Y:.1f}, in front of the desk's edge (Y {DeskFrontEdgeYCm:g})"
    if Branch in ("bin", "object") and not InBin:
        return f"the object meets the bin floor at X {X:.1f} Y {Y:.1f}, outside the bin"
    if Branch == "object" and Y > AmbiguousContactYCm[0]:
        return f"the object stands on something at the bin rim (Y {Y:.1f}), where desk and bin are ambiguous"
    return None


def _RingPoints(DepthMm, Box, Intrinsics, Model, Exclude=None) -> np.ndarray:
    """Display points of the depth in a ring around a native box (the object's surroundings)."""
    Height, Width = DepthMm.shape[:2]
    U0, V0, U1, V1 = Box
    PadU, PadV = max(8, int(RingPadFraction * (U1 - U0))), max(8, int(RingPadFraction * (V1 - V0)))
    R0, R1 = max(0, U0 - PadU), min(Width, U1 + PadU)
    S0, S1 = max(0, V0 - PadV), min(Height, V1 + PadV)
    Crop = DepthMm[S0:S1, R0:R1].astype(np.float64) / 10.0
    Ring = (Crop > 0) & (Crop <= MaxDepthCm)
    Ring[V0 - S0:V1 - S0, U0 - R0:U1 - R0] = False
    if Ring.sum() < RingMinPoints:
        return np.empty((0, 3))
    Rays = RayMap(Intrinsics)[S0:S1, R0:R1][Ring]
    Points = CameraToDisplay(np.column_stack([Rays * Crop[Ring][:, None], Crop[Ring]]), Model)
    return Points[~Exclude(Points)] if Exclude is not None else Points


def _PixelRay(U: float, V: float, Intrinsics, Model) -> tuple[np.ndarray, np.ndarray]:
    """The camera centre and the ray through a native pixel, display frame (a unit-depth step, not normalized)."""
    Centre = CameraToDisplay(np.zeros(3), Model)[0]
    return Centre, CameraToDisplay(Deproject(U, V, 1.0, Intrinsics), Model)[0] - Centre


def _DeclaredUnder(Surfaces, Box, Intrinsics, Model) -> list[tuple[dict[str, Any], bool]]:
    """The declared surfaces a native box is over, highest first, each with whether it is above the desk:
    the ray through the box's bottom-centre or its centre pixel meets the surface's plane over its extent."""
    U0, V0, U1, V1 = Box
    Rays = [_PixelRay((U0 + U1 - 1) / 2.0, V1 - 1.0, Intrinsics, Model), _PixelRay((U0 + U1 - 1) / 2.0, (V0 + V1 - 1) / 2.0, Intrinsics, Model)]
    Found = []
    for Surface in sorted(Surfaces.get("declared") or [], key=lambda Item: -float(Item["TopZCm"])):
        for Centre, Direction in Rays:
            Reach = (float(Surface["TopZCm"]) - Centre[2]) / Direction[2] if abs(Direction[2]) > 1e-9 else -1.0
            Hit = Centre + Reach * Direction
            if Reach > 0 and bool(_Over(Surface, Hit[0], Hit[1], DeclaredRayMarginCm)):
                Found.append((Surface, bool(float(Surface["TopZCm"]) > float(DeskHeight(Surfaces, Hit[0], Hit[1])))))
                break
    return Found


def _Level(Points) -> bool:
    """Whether display points (N x 3, at about one height) cover an area: a level surface around an object,
    not the bands a height cuts across walls and an object's sides (lines seen from above, an L at a corner).
    Counted on a grid of LevelCellCm cells: at least LevelMinInteriorCells with all four neighbours covered."""
    if len(Points) < RingMinPoints:
        return False
    Cells = {(int(X), int(Y)) for X, Y in np.floor(Points[:, :2] / LevelCellCm)}
    Interior = sum(1 for X, Y in Cells if {(X - 1, Y), (X + 1, Y), (X, Y - 1), (X, Y + 1)} <= Cells)
    return Interior >= LevelMinInteriorCells


def _SurfaceLevel(Ring, Surface: dict[str, Any]) -> float:
    """The height of a declared surface as the ring over its extent measures it near the declared height
    (the dominant level 0.5 cm band within DeclaredToleranceCm), or the declared height where the ring has none."""
    Top = float(Surface["TopZCm"])
    if len(Ring):
        Over = Ring[_Over(Surface, Ring[:, 0], Ring[:, 1], DeclaredRingMarginCm)]
        Near = Over[np.abs(Over[:, 2] - Top) <= DeclaredToleranceCm]
        if len(Near) >= RingMinPoints:
            Counts, Edges = np.histogram(Near[:, 2], bins=np.arange(Top - DeclaredToleranceCm, Top + DeclaredToleranceCm + 0.25, 0.5))
            Mode = float(Edges[int(np.argmax(Counts))]) + 0.25
            Around = Near[np.abs(Near[:, 2] - Mode) <= 1.0]
            if _Level(Around):
                return float(np.median(Around[:, 2]))
    return Top


def _Surrounds(Patch, Contact, MarginCm: float = 0.5) -> bool:
    """Whether a support patch's points (display, N x 3) lie around a contact: within their 5-95% X and Y
    span. A box in front of a cup hides its lower part, and its top would read as the cup's support."""
    Low, High = np.percentile(Patch[:, :2], 5, axis=0) - MarginCm, np.percentile(Patch[:, :2], 95, axis=0) + MarginCm
    return bool(np.all(Contact >= Low) and np.all(Contact <= High))


def _CutsThrough(Component, Valid, Above) -> bool:
    """Whether the depth just under the object's bottom edge (the 4 pixels below it in each column of the
    box) mostly lies lower than the support: the support cuts through the object, it does not hold it."""
    Rows = np.arange(Component.shape[0])[:, None]
    Bottom = np.where(Component, Rows, -1).max(axis=0)
    Columns = np.nonzero(Bottom >= 0)[0]
    On = Below = 0
    for Step in range(1, 5):
        Row = Bottom[Columns] + Step
        Inside = Row < Component.shape[0]
        R, C = Row[Inside], Columns[Inside]
        Seen = Valid[R, C] & ~Component[R, C]
        On += int((Seen & (np.abs(Above[R, C]) <= UndeclaredBelowCm)).sum())
        Below += int((Seen & (Above[R, C] < -UndeclaredBelowCm)).sum())
    return Below >= ObjectMinPoints and Below > UndeclaredBelowShare * (On + Below)


def _SupportCandidates(World, Valid, Crop, Ring, Surfaces, Only=None) -> list[dict[str, Any]]:
    """Candidates where an object may stand, on a measured surface nobody declared.
    Each has the measured height and object. The surface is a level peak of
    the ring's heights (not the desk's or the bin floor's, never the room floor) that holds the object
    without cutting through it and lies around where it touches.

    With no calibrated desk in view (DeskSupport False) the desk's height is no exclusion (the old desk
    is not there; the room floor is still bounded by its cart-fixed plane), and the peaks are tried
    largest first: at the bookshelf cabinet of 2026-10-06 15:01 the cabinet top under the pill bottle
    was the lowest of five peaks around it, with 2675 of the ring's 11035 points (the others the book
    spines behind it, 260-404 points each), so a cap on the peaks tried keeps it."""
    if len(Ring) < RingMinPoints:
        return []
    Desk = DeskHeight(Surfaces, Ring[:, 0], Ring[:, 1])
    InBin = (Ring[:, 1] <= BinRimYCm) & (np.abs(Ring[:, 0]) <= BinHalfWidthCm)
    OnDesk = DeskSupport(Surfaces)
    Keep = (Ring[:, 2] >= Desk - MaxSupportDropCm) & ~(InBin & (np.abs(Ring[:, 2] - Surfaces["bin_floor_z"]) <= DeskSupportToleranceCm))
    if OnDesk:
        Keep &= np.abs(Ring[:, 2] - Desk) > DeskSupportToleranceCm
    Points = Ring[Keep]
    if len(Points) < RingMinPoints:
        return []
    Heights = Points[:, 2]
    Counts, Edges = np.histogram(Heights, bins=np.arange(math.floor(Heights.min()), Heights.max() + 0.75, 0.5))
    Peaks = [Index for Index in range(len(Counts)) if Counts[Index] >= RingMinPoints
             and Counts[Index] >= max(Counts[max(Index - 1, 0)], Counts[min(Index + 1, len(Counts) - 1)])]
    Found = []
    # At a desk, highest first as before (its recorded measurements are unchanged); with none, largest first.
    for Index in sorted(Peaks, key=(lambda Index: -Edges[Index]) if OnDesk else (lambda Index: (-Counts[Index], -Edges[Index])))[:8]:
        Patch = Points[np.abs(Heights - (Edges[Index] + 0.25)) <= 1.0]
        if not _Level(Patch):
            continue
        Level = float(np.median(Patch[:, 2]))
        Outcome = _MeasureOn(World, Valid, Crop, Ring, "surface", _Flat(Level), Surfaces, None, **({"Only": Only} if Only is not None else {}))
        # At a desk the patch must lie around the contact and the footprint; with none, beside enough of its sides (SupportSides).
        if not Outcome["ok"] or (OnDesk and not _Surrounds(Patch, np.asarray(Outcome["contact_cm"]))):
            continue
        # With no desk, its sides are judged on the ring and on the surface seen inside the box (beside the object).
        Inside = None if OnDesk else World[Valid & (np.abs(World[..., 2] - Outcome["z"]) <= 0.35)]
        Geometry = _MeasuredSupportGeometry(Patch, Outcome, Sides=not OnDesk, Inside=Inside)
        if Geometry is None:
            continue
        # A wide noisy peak may occupy adjacent histogram bins; it is one plane.
        if any(abs(Item["undeclared_support_z"] - Outcome["z"]) < 1.0 for Item in Found):
            continue
        Found.append({"undeclared_support_z": Outcome["z"], "measured": Outcome, "support_geometry": Geometry})
    return Found


def _UndeclaredSupport(World, Valid, Crop, Ring, Surfaces, Only=None) -> dict[str, Any] | None:
    """Legacy hint: an undeclared candidate is not itself an authorized support."""
    Found = _SupportCandidates(World, Valid, Crop, Ring, Surfaces, Only)
    return Found[0] if len(Found) == 1 else None


def SupportSides(Patch, Centre, Half, Inside=None) -> list[str]:
    """The sides of a footprint (display Centre +- Half, cm) a level patch (N x 3 display points) is measured right
    beside: "-x", "+x", "-y" (toward the arm), "+y". A side's band reaches SupportSideBandCm out from the footprint's
    edge and SupportSideInsideCm under it, along the side's length; it counts with SupportSideMinPoints of the patch in it
    covering SupportSideCover of that length and SupportSideDepthCm outward (an area, not the line a wall or the object's
    own side cuts at that height). Inside (the box's points at the surface's height) counts only from SupportSideClearCm
    outside the footprint: nearer, the object's own side is level with any height."""
    Centre, Half = np.asarray(Centre, dtype=np.float64), np.asarray(Half, dtype=np.float64)
    Inside = np.empty((0, 3)) if Inside is None else np.asarray(Inside, dtype=np.float64).reshape(-1, 3)
    Sides = []
    for Axis, Sign, Name in ((0, -1, "-x"), (0, 1, "+x"), (1, -1, "-y"), (1, 1, "+y")):
        Along = 1 - Axis
        Edge = Centre[Axis] + Sign * Half[Axis]
        Low, High = Centre[Along] - Half[Along], Centre[Along] + Half[Along]
        Band = []
        for Points, Nearest in ((Patch, -SupportSideInsideCm), (Inside, SupportSideClearCm)):
            Out = Sign * (Points[:, Axis] - Edge)
            Band.append(np.column_stack([Points[:, Along], Out])[(Out >= Nearest) & (Out <= SupportSideBandCm)
                                                               & (Points[:, Along] >= Low) & (Points[:, Along] <= High)])
        Band = np.vstack(Band)
        if len(Band) < SupportSideMinPoints or float(np.percentile(Band[:, 1], 90) - np.percentile(Band[:, 1], 10)) < SupportSideDepthCm:
            continue
        Bins = max(1, int(math.ceil((High - Low) / SupportSideBinCm)))
        Covered = len(np.unique(np.clip(((Band[:, 0] - Low) / SupportSideBinCm).astype(int), 0, Bins - 1)))
        if Covered >= SupportSideCover * Bins:
            Sides.append(Name)
    return Sides


def _MeasuredSupportGeometry(Patch, Outcome, Sides: bool = False, Inside=None) -> dict[str, Any] | None:
    """Observed local extent, not an extrapolated board or a claim of empty air.

    Sides (no calibrated desk in view): a patch that does not lie around the whole footprint still holds it when it is
    measured beside SupportMinSides of its sides (SupportSides, on the patch and Inside: the surface's points inside the
    box, display N x 3); its extent is then what it measured with the footprint."""
    Level = float(Outcome["z"])
    Patch = Patch[np.abs(Patch[:, 2] - Level) <= 0.35]
    if not _Level(Patch):
        return None
    Low, High = np.percentile(Patch[:, :2], 5, axis=0), np.percentile(Patch[:, :2], 95, axis=0)
    Centre = np.array([Outcome["x"], Outcome["y"]])
    Half = np.asarray(Outcome["size_cm"]) / 2
    Beside = None
    # Both the measured footprint and its contact must be surrounded by the
    # patch. A neighbour's top beside or in front of an object is not its base.
    if np.any(Low > Centre - Half - 0.25) or np.any(High < Centre + Half + 0.25):
        if not Sides:
            return None
        # No desk: books, a wall or a neighbour against the object hide its support there (they stay solids of their
        # own); the patch beside enough of its sides holds it, and its extent takes in the footprint it holds.
        Beside = SupportSides(Patch, Centre, Half, Inside)
        if len(Beside) < SupportMinSides:
            return None
        Low, High = np.minimum(Low, Centre - Half), np.maximum(High, Centre + Half)
    if not -20.0 <= Level <= 50.0 or np.any(High - Low < 2.0):
        return None
    Evidence = {"points": int(len(Patch)), "height_spread_cm": round(float(np.percentile(Patch[:, 2], 95) - np.percentile(Patch[:, 2], 5)), 3)}
    if Beside is not None:
        Evidence["sides"] = Beside
    return {"Name": f"measured support at Z {Level:.1f}", "Kind": "platform", "Enabled": True,
            "TopZCm": Level, "XRangeCm": np.round([Low[0], High[0]], 2).tolist(),
            "YRangeCm": np.round([Low[1], High[1]], 2).tolist(), "BoardAboveZCm": None,
            "Frame": "cart", "Source": "depth", "ObservedOnly": True, "Evidence": Evidence}


def LocalSupportGeometry(DepthMm, Box, Intrinsics, Model, Mirrored, Measured, Exclude=None):
    """Observed local support extent for a legacy desk/bin/declared measurement."""
    Native = BoxToNative(Box, DepthMm.shape[1], DepthMm.shape[0], Mirrored)
    Ring = _RingPoints(DepthMm, Native, Intrinsics, Model, Exclude)
    Patch = Ring[np.abs(Ring[:, 2] - Measured["z"]) <= 1.0]
    return _MeasuredSupportGeometry(Patch, Measured) if _Level(Patch) else None


def _DeskDiagnostics(Ring, Surfaces, Box, Intrinsics, Model) -> dict[str, Any]:
    """For the scene tripwire: where the box's bottom meets the calibrated desk, and the ring's
    dominant surface against that desk, searched well beyond the support window (+-8 cm)."""
    U0, V0, U1, V1 = Box
    Normal, Offset = Surfaces["desk"]
    Centre = CameraToDisplay(np.zeros(3), Model)[0]
    Direction = CameraToDisplay(Deproject((U0 + U1 - 1) / 2.0, V1 - 1.0, 1.0, Intrinsics), Model)[0] - Centre
    Diagnostics: dict[str, Any] = {"desk_area": False, "desk_ring_offset_cm": None}
    Along = float(Normal @ Direction)
    Reach = (Offset - float(Normal @ Centre)) / Along if abs(Along) > 1e-9 else -1.0
    if Reach > 0:
        Contact = Centre + Reach * Direction
        Diagnostics.update(nominal_contact_cm=[round(float(Contact[0]), 2), round(float(Contact[1]), 2)],
                           desk_area=bool(Contact[1] >= AmbiguousContactYCm[1]))
    if len(Ring):
        Residual = Ring[:, 2] - DeskHeight(Surfaces, Ring[:, 0], Ring[:, 1])
        Residual = Residual[np.abs(Residual) <= DeskOffsetSearchCm]
        if Residual.size >= RingMinPoints:
            Counts, Edges = np.histogram(Residual, bins=np.arange(-DeskOffsetSearchCm, DeskOffsetSearchCm + 0.25, 0.5))
            Mode = float(Edges[int(np.argmax(Counts))]) + 0.25
            Around = Residual[np.abs(Residual - Mode) <= 1.0]
            if Around.size >= RingMinPoints:
                Diagnostics["desk_ring_offset_cm"] = round(float(np.median(Around)), 2)
    return Diagnostics


def PointAtPixel(DepthMm: np.ndarray, U: float, V: float, Intrinsics: dict[str, Any], Mirrored: bool,
                 Radius: int = 3) -> dict[str, Any]:
    """Camera point at a capture pixel from the near cluster of a small window."""
    Height, Width = DepthMm.shape[:2]
    UNative, VNative = (float(Value) for Value in NativeFromCapture(U, V, Width, Mirrored))
    Iu, Iv = int(round(UNative)), int(round(VNative))
    if not (0 <= Iu < Width and 0 <= Iv < Height):
        return {"ok": False, "error": "pixel outside the depth image"}
    Window = DepthMm[max(0, Iv - Radius):Iv + Radius + 1, max(0, Iu - Radius):Iu + Radius + 1]
    Depth = ForegroundDepthCm(Window.ravel() / 10.0)
    if Depth is None:
        return {"ok": False, "error": "no depth at that pixel"}
    return {"ok": True, "depth_cm": round(Depth, 2), "native_px": [UNative, VNative],
            "camera_cm": Deproject(UNative, VNative, Depth, Intrinsics)[0].tolist()}


def FitPlane(Points: np.ndarray, Iterations: int = 300, ToleranceCm: float = 0.6, Seed: int = 0):
    """RANSAC plane (unit normal n, offset d with n.p = d) and its inlier mask."""
    Points = np.asarray(Points, dtype=np.float64)
    if len(Points) < 3:
        return None
    Rng = np.random.default_rng(Seed)
    # All hypotheses at once: a Python loop of 300 full-cloud passes held the interpreter long
    # enough in the bridge process to starve the servo feedback loop.
    Samples = Points[Rng.integers(0, len(Points), size=(Iterations, 3))]
    Normals = np.cross(Samples[:, 1] - Samples[:, 0], Samples[:, 2] - Samples[:, 0])
    Norms = np.linalg.norm(Normals, axis=1)
    Keep = Norms > 1e-9
    if not Keep.any():
        return None
    Normals = Normals[Keep] / Norms[Keep, None]
    Offsets = np.einsum("ij,ij->i", Normals, Samples[Keep, 0])
    Counts = np.zeros(len(Normals), dtype=np.int64)
    for Start in range(0, len(Normals), 64):
        Block = slice(Start, Start + 64)
        Counts[Block] = (np.abs(Points @ Normals[Block].T - Offsets[Block]) <= ToleranceCm).sum(axis=0)
    Winner = int(np.argmax(Counts))
    Best = np.abs(Points @ Normals[Winner] - Offsets[Winner]) <= ToleranceCm
    if Best.sum() < 3:
        return None
    Inlier = Points[Best]
    Centre = Inlier.mean(axis=0)
    _, _, Vt = np.linalg.svd(Inlier - Centre, full_matrices=False)  # full U would be N x N (~1 GB)
    Normal = Vt[2]
    Final = np.abs((Points - Centre) @ Normal) <= ToleranceCm
    return Normal, float(Normal @ Centre), Final


def CheckPlane(DepthMm: np.ndarray, Intrinsics: dict[str, Any], Model: dict[str, Any] | None,
               Step: int = 8) -> dict[str, Any]:
    """Dominant plane in view; with a depth calibration, its height and tilt in the AI frame."""
    Depth = DepthMm[::Step, ::Step].astype(np.float64) / 10.0
    Rays = RayMap(Intrinsics)[::Step, ::Step]
    Valid = Depth > 0
    Points = np.concatenate([Rays * Depth[..., None], Depth[..., None]], axis=-1)[Valid]
    Plane = FitPlane(Points)
    if Plane is None:
        return {"ok": False, "error": "Not enough depth in view to fit a plane."}
    Normal, Offset, Inliers = Plane
    Result: dict[str, Any] = {"ok": True, "plane_inlier_fraction": round(float(Inliers.mean()), 3),
                              "camera_distance_to_plane_cm": round(abs(Offset), 1)}
    if ModelUsesDepthCamera(Model, Intrinsics):
        World = CameraToDisplay(Points[Inliers], Model)
        Rotation, _, _ = PoseFromModel(Model)
        Up = abs(float((Rotation.T @ Normal)[2]))  # a mirror in X leaves the vertical component alone
        PlaneZ = float(np.median(World[:, 2]))
        Result.update(plane_z_cm=round(PlaneZ, 2), table_z_cm=Model.get("table_z"),
                      plane_tilt_deg=round(math.degrees(math.acos(min(1.0, Up))), 2),
                      height_error_cm=round(PlaneZ - float(Model.get("table_z", 0.0)), 2))
    return Result


# --------------------------------------------------------------------------
# Camera pose from the arm's tip tape, with metric depth and known intrinsics
# --------------------------------------------------------------------------

def Kabsch(Source: np.ndarray, Target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Proper rotation R and translation t minimizing |R @ Source + t - Target|."""
    SourceCentre, TargetCentre = Source.mean(axis=0), Target.mean(axis=0)
    U, _, Vt = np.linalg.svd((Source - SourceCentre).T @ (Target - TargetCentre))
    Sign = np.sign(np.linalg.det(Vt.T @ U.T)) or 1.0
    Rotation = Vt.T @ np.diag([1.0, 1.0, Sign]) @ U.T
    return Rotation, TargetCentre - Rotation @ SourceCentre


def _Predict(Params, Tips, Frames, Mirror):
    Rotation, _ = cv2.Rodrigues(Params[:3])
    World = Tips + np.einsum("nij,j->ni", Frames, Params[6:9])
    if Mirror:
        World = World * MirrorVector
    return World @ Rotation.T + Params[3:6]


def _DepthSigma(Depth):
    return FitSigmaDepthBaseCm + FitSigmaDepthPerCm * np.abs(Depth)


def _FitResiduals(Params, Tips, Frames, Pixels, CameraPoints, HasDepth, Intrinsics, Mirror):
    Predicted = _Predict(Params, Tips, Frames, Mirror)
    Safe = np.where(Predicted[:, 2:3] > 1e-3, Predicted, np.array([0.0, 0.0, 1e-3]))
    Projected = Project(Safe, Intrinsics)
    Residual = [((Projected - Pixels) / FitSigmaPx).ravel()]
    Behind = np.clip(1.0 - Predicted[:, 2], 0.0, None) * 100.0
    Residual.append(Behind)
    if HasDepth.any():
        Delta = (Predicted - CameraPoints)[HasDepth]
        Sigma = np.column_stack([np.full(HasDepth.sum(), FitSigmaLateralCm), np.full(HasDepth.sum(), FitSigmaLateralCm),
                                 _DepthSigma(CameraPoints[HasDepth, 2])])
        Residual.append((Delta / Sigma).ravel())
    return np.concatenate(Residual)


def _Errors(Params, Tips, Frames, Pixels, CameraPoints, HasDepth, Intrinsics, Mirror):
    Predicted = _Predict(Params, Tips, Frames, Mirror)
    PixelErrors = np.linalg.norm(Project(Predicted, Intrinsics) - Pixels, axis=1)
    FocalMean = 0.5 * (Intrinsics["fx"] + Intrinsics["fy"])
    PixelErrorsCm = PixelErrors * np.abs(Predicted[:, 2]) / FocalMean
    DepthErrors = np.where(HasDepth, np.linalg.norm(Predicted - np.nan_to_num(CameraPoints), axis=1), np.nan)
    return PixelErrors, PixelErrorsCm, DepthErrors, Predicted


def FitDepthCamera(Tips, Frames, Pixels, CameraPoints, Intrinsics: dict[str, Any], Mirror: bool,
                   Initial: np.ndarray | None = None, MaxEvaluations: int = 400) -> dict[str, Any] | None:
    """Camera pose and tape offset from tip poses, tape pixels and depth points.

    Tips, Frames: display-frame tip positions (N x 3) and gripper frames (N x 3 x 3).
    Pixels: native tape centroids (N x 2). CameraPoints: depth-measured tape
    points in the camera frame (N x 3, NaN rows where depth was missing).
    Intrinsics stay fixed at the camera's factory values.
    """
    Tips = np.asarray(Tips, dtype=np.float64)
    Frames = np.asarray(Frames, dtype=np.float64)
    Pixels = np.asarray(Pixels, dtype=np.float64)
    CameraPoints = np.asarray(CameraPoints, dtype=np.float64)
    HasDepth = np.isfinite(CameraPoints).all(axis=1)
    if Initial is None:
        World = Tips * MirrorVector if Mirror else Tips
        if HasDepth.sum() >= 3:
            Rotation, Translation = Kabsch(World[HasDepth], CameraPoints[HasDepth])
            Rvec, _ = cv2.Rodrigues(Rotation)
            Rvec, Translation = Rvec.ravel(), Translation
        else:
            Ok, Rvec, Tvec = cv2.solvePnP(World, Pixels, CameraMatrix(Intrinsics), Distortion(Intrinsics),
                                          flags=cv2.SOLVEPNP_SQPNP)
            if not Ok:
                return None
            Rvec, Translation = Rvec.ravel(), Tvec.ravel()
        Initial = np.concatenate([Rvec, Translation, np.zeros(3)])
    Lower = np.full(9, -np.inf)
    Upper = np.full(9, np.inf)
    Lower[6:], Upper[6:] = -15.0, 15.0  # the tape sits within 15 cm of the tip
    Start = np.clip(Initial, Lower + 1e-6, Upper - 1e-6)
    Result = least_squares(_FitResiduals, Start, args=(Tips, Frames, Pixels, CameraPoints, HasDepth, Intrinsics, Mirror),
                           bounds=(Lower, Upper), method="trf", loss="soft_l1", f_scale=2.0, x_scale="jac",
                           max_nfev=MaxEvaluations)
    Params = Result.x
    PixelErrors, PixelErrorsCm, DepthErrors, Predicted = _Errors(Params, Tips, Frames, Pixels, CameraPoints, HasDepth,
                                                                 Intrinsics, Mirror)
    if (Predicted[:, 2] <= 0).any():
        return None
    return {
        "params": Params,
        "mirror": bool(Mirror),
        "rms_px": float(np.sqrt(np.mean(PixelErrors ** 2))),
        "rms_cm": float(np.sqrt(np.mean(PixelErrorsCm ** 2))),
        "depth_rms_cm": float(np.sqrt(np.nanmean(DepthErrors ** 2))) if HasDepth.any() else None,
        "pixel_errors": PixelErrors,
        "pixel_errors_cm": PixelErrorsCm,
        "depth_errors_cm": DepthErrors,
        "cost": float(Result.cost),
    }


def PoseErrors(Params, Tips, Frames, Pixels, CameraPoints, Intrinsics, Mirror):
    CameraPoints = np.asarray(CameraPoints, dtype=np.float64)
    return _Errors(np.asarray(Params, dtype=np.float64), np.asarray(Tips, dtype=np.float64),
                   np.asarray(Frames, dtype=np.float64), np.asarray(Pixels, dtype=np.float64), CameraPoints,
                   np.isfinite(CameraPoints).all(axis=1), Intrinsics, Mirror)
