"""Observed occupied and free space from one calibrated, aligned depth frame.

No absence-of-points inference: a free voxel's whole projected footprint must
have valid depth behind its farthest camera-Z corner. Occlusions and missing
depth remain unknown. Boxes are unions of adjacent voxels, never convex hulls
over gaps. All outputs use the existing display frame, in centimetres.
"""
from __future__ import annotations

import json
from collections import defaultdict
from itertools import product
from pathlib import Path
from typing import Any

import cv2
import numpy as np

import DepthGeometry as DG

BoundsMin = np.array([-65.0, -25.0, -20.0])
BoundsMax = np.array([65.0, 80.0, 65.0])
OccupiedCellCm = 1.0
FreeCellCm = 2.0
DepthUncertaintyCm = 0.5
MaxObstacleBoxes = 2048
# The robot is no obstacle of its own scene: the cart with its basket and the mast with the camera head (the
# collision model's own surfaces, which the route planner checks every arm box against already), and the arm
# where it stood for the capture (its CAD link boxes, CaptureArmPose "link_boxes"). On 2026-10-06 at 15:01, 238
# of the 1419 obstacle boxes were the robot (188 the basket, 24 the mast, 26 the arm) and every route was refused,
# "Shoulder mounting collar intersects the observed environment": the collar's whole pan revolution is checked
# against the scene. The robot's observed points lie up to RobotMarginCm from its CAD surfaces (the calibration's
# and the CAD's errors); the arm's within ArmMarginCm of its boxes, which enclose its parts loosely already.
RobotMarginCm = 2.0
ArmMarginCm = 2.0
# Where the wrist and the gripper stood is known space (scene "known_space", which the bridge's gripper coverage check
# takes as free), each link box grown KnownArmMarginCm, not ArmMarginCm: the 2 cm cut-out above keeps the arm's own
# points out of the scene, but grown as far as known space it also declared a board 0.5-1.9 cm beside the gripper,
# jaw or wrist free, its points cut out and 100% of them "known" (2026-10-08 review, audit 1 IM1). The capture pose's
# own check (the pose with its tracking allowance, StartRelief) needs 0.25 cm at Home and all 24 calibration poses,
# with nothing else observed free (.claude/Stage5/scratch/KnownGrowth.py); 0.5 keeps twice that. The gripper's first
# move farther out needs observed free space now, as everywhere else (2 cm of known space let a 3 deg first move go
# with none). The space cut out beyond it is unknown, not free: the gripper is not certified into it.
KnownArmMarginCm = 0.5
_Robot: dict[str, Any] = {}
# Unobserved space around the task object (H2: a policy, NoDeskDesign.md open question 3, for Aaran to decide;
# off). A grasp's gripper comes within a centimetre of its object and of the surface under it, where no 2 cm cube
# is ever certified free (the depth there is the object or the surface itself), and its moving jaw closes behind
# the object, which hides that space from the camera: at the cabinet of 2026-10-06 15:01 the gripper boxes of the
# hover, the descent and the lift were 90% unobserved, and every grasp was refused. With AssumeTaskReachFree the
# space within TaskReachCm of the task object's envelope, from TaskReachBelowCm under its support's top (the solid
# support, an obstacle, under the jaws' boxes) to TaskReachAboveCm over its top (the gripper over the hover point),
# counts as known (scene "known_space"): observed solids there stay obstacles, and so does the unobserved space under
# them down to the support (HiddenUnder), so only what the camera could not see and nothing it saw stands over is
# taken to be empty. A near-level wrist (the bottle 39-42 cm out) lays the gripper's body 8 cm behind the object: 6 cm
# left the descent unobserved (the bottle moved 3 cm clear of the books). Everywhere else free space must be observed.
AssumeTaskReachFree = False
TaskReachCm = 8.0
TaskReachAboveCm = 12.0
TaskReachBelowCm = 2.0
# The task object's measurement leaves out its own rim where the depth jumps (DepthGeometry._MeasureOn breaks the
# object from what stands behind it there, both pixels of the jump): left in the scene, that rim was "observed
# environment" hugging the object, 0.0-0.2 cm outside its measured footprint (the pill bottle moved clear of the
# books on the 2026-10-06 15:01 cabinet frame: 5-8 points, every grasp refused as "running into observed
# environment beside it"). The scene removes the footprint grown by TargetRimCm; a neighbour touching the object
# loses at most that much of its near face, which its obstacle padding (DepthUncertaintyCm) covers.
TargetRimCm = 0.3
# A support is level in the display frame only when the cart stands level: the arm's frame leans (about 4 deg at the
# cabinet of 2026-10-06, the cabinet top rising 0.074 cm per cm away from the arm), so around the object its surface
# leaves the 0.35 cm band of its measured level within 5 cm, and was "observed environment" beside the object's base
# (its jaws and the lifted object "would touch" it). Within the support's measured extent, its surface within the band
# of a plane fitted to it (_SupportPlane) is the support too, where that plane stays within SupportBandCm of the level.
SupportBandCm = 1.0
# The measured extent is the part within the 0.35 cm band of the level, narrower than the support where it tilts: its
# plane holds this far beyond it. Within SupportMinTiltDeg of level a support is level (its band holds it over 20 cm),
# and a level scene is built as before.
SupportPlaneReachCm = 4.0
SupportMinTiltDeg = 1.0
SupportMaxTiltDeg = 10.0
# A neighbour touching the task object (TaskContact): this many points within TouchCm of its measured footprint.
TouchCm = 0.5
TouchMinPoints = 25


def _MergeCells(Cells: np.ndarray, Step: float, Origin: np.ndarray, Label: str | None = None) -> list[dict[str, Any]]:
    """Merge exact cell unions along X, then Y, then Z; never bridge a hole."""
    if not len(Cells):
        return []
    Rows: dict[tuple[int, int], list[int]] = defaultdict(list)
    for X, Y, Z in np.unique(Cells.astype(np.int32), axis=0):
        Rows[(int(Y), int(Z))].append(int(X))
    Boxes = []
    for (Y, Z), Xs in Rows.items():
        Xs.sort()
        Start = Last = Xs[0]
        for X in Xs[1:]:
            if X != Last + 1:
                Boxes.append([Start, Y, Z, Last + 1, Y + 1, Z + 1])
                Start = X
            Last = X
        Boxes.append([Start, Y, Z, Last + 1, Y + 1, Z + 1])
    for Axis in (1, 2):
        Groups: dict[tuple[int, ...], list[list[int]]] = defaultdict(list)
        for Box in Boxes:
            Key = tuple(Box[I] for I in range(6) if I not in (Axis, Axis + 3))
            Groups[Key].append(Box)
        Joined = []
        for Group in Groups.values():
            Group.sort(key=lambda Box: Box[Axis])
            Current = Group[0].copy()
            for Box in Group[1:]:
                if Current[Axis + 3] == Box[Axis]:
                    Current[Axis + 3] = Box[Axis + 3]
                else:
                    Joined.append(Current)
                    Current = Box.copy()
            Joined.append(Current)
        Boxes = Joined
    return [{"min": np.round(Origin + Step * np.array(Box[:3]), 3).tolist(),
             "max": np.round(Origin + Step * np.array(Box[3:]), 3).tolist(),
             **({"label": Label} if Label else {})} for Box in Boxes]


def _RobotStatic() -> dict[str, Any]:
    """The collision model's cart, basket and mast surfaces (arm model frame), and where they lie in the display frame."""
    if not _Robot:
        import BasketCollision  # its surfaces are built once, at import
        Frame = json.loads((Path(__file__).resolve().parent / "WorkspaceFrame.json").read_text(encoding="utf-8"))
        Side = Frame.get("RobotFrame", {}).get("ControlledArm", "Right")
        Base = Frame["RobotFrame"]["ArmBasePositionCm"]
        Origin = np.asarray(Frame["RobotFrame"]["OriginAssemblyCm"], dtype=np.float64)
        Bounds = [np.asarray(BasketCollision.Model["BasketAssemblyBoundsCm"]), np.asarray(BasketCollision.Model["RobotBodyAssemblyBoundsCm"])]
        # Assembly to display: X mirrored, about the arm midpoint at the bin rim.
        Low = np.min([[-High[0], Low_[1] - Origin[1], Low_[2] - Origin[2]] for Low_, High in Bounds], axis=0)
        High = np.max([[-Low_[0], High[1] - Origin[1], High[2] - Origin[2]] for Low_, High in Bounds], axis=0)
        _Robot.update(Surfaces=(BasketCollision.Surfaces[Side], BasketCollision.BodySurfaces[Side]),
                      Base=np.array([Base["x"], Base["y"], Base["z"]], dtype=np.float64), Low=Low, High=High)
    return _Robot


def RobotCells(Cells: np.ndarray, Step: float = OccupiedCellCm, Origin: np.ndarray = BoundsMin) -> np.ndarray:
    """Which occupied cells (integer indices, N x 3) touch the cart, its basket or the mast within RobotMarginCm."""
    Robot = _RobotStatic()
    Low = Origin + Step * Cells.astype(np.float64)
    Near = np.all((Low + Step >= Robot["Low"] - RobotMarginCm) & (Low <= Robot["High"] + RobotMarginCm), axis=1)
    On = np.zeros(len(Cells), dtype=bool)
    Half = np.full(3, Step / 2)
    for Index in np.nonzero(Near)[0]:
        Centre = Low[Index] + Step / 2
        # Display to the arm's model frame (SceneSafety.ModelBounds): X mirrored about the arm base.
        Model = np.array([-Centre[0] - Robot["Base"][0], Centre[1] - Robot["Base"][1], Centre[2] - Robot["Base"][2]])
        On[Index] = any(Surface.Overlaps(Model, np.eye(3), Half, RobotMarginCm) for Surface in Robot["Surfaces"])
    return On


def ArmPoints(Points: np.ndarray, Boxes, MarginCm: float = ArmMarginCm) -> np.ndarray:
    """Which display points lie in the arm's link boxes (CaptureArmPose "link_boxes") grown by MarginCm; its fixed
    mount (the collar's revolution, "fixed") by RobotMarginCm, as the cart: the servo cables hang 1.1-1.3 cm
    outside it at Home (2026-10-06 15:01)."""
    Inside = np.zeros(len(Points), dtype=bool)
    for Box in Boxes or []:
        Local = (Points - np.asarray(Box["centre"], dtype=np.float64)) @ np.asarray(Box["axes"], dtype=np.float64)
        Grown = np.asarray(Box["half"], dtype=np.float64) + (RobotMarginCm if Box.get("fixed") else MarginCm)
        Inside |= np.all(np.abs(Local) <= Grown, axis=1)
    return Inside


def _SupportPlane(Points: np.ndarray, Support: dict[str, Any], Low, High) -> np.ndarray | None:
    """The support's surface as a plane Z = a X + b Y + c (display frame, [a, b, c]) fitted to the points over its
    measured extent within SupportBandCm of its level, outside the task object's footprint; None when too few points
    cover an area, it tilts less than SupportMinTiltDeg (level) or more than SupportMaxTiltDeg, or it misses the
    measured level under the object."""
    Level = float(Support["TopZCm"])
    Outside = ~np.all((Points[:, :2] >= np.asarray(Low)[:2]) & (Points[:, :2] <= np.asarray(High)[:2]), axis=1)
    Near = Points[Outside & (np.abs(Points[:, 2] - Level) <= SupportBandCm)]
    for _ in range(3):
        if not DG._Level(Near):
            return None
        Plane = np.linalg.lstsq(np.column_stack([Near[:, 0], Near[:, 1], np.ones(len(Near))]), Near[:, 2], rcond=None)[0]
        Near = Near[np.abs(Near[:, 2] - Near[:, :2] @ Plane[:2] - Plane[2]) <= 0.35]
    Centre = (np.asarray(Low)[:2] + np.asarray(High)[:2]) / 2
    if (not DG._Level(Near) or not SupportMinTiltDeg <= np.degrees(np.arctan(np.hypot(*Plane[:2]))) <= SupportMaxTiltDeg
            or abs(float(Centre @ Plane[:2] + Plane[2]) - Level) > 0.35):
        return None
    return Plane


def TaskBounds(Object: dict[str, Any]) -> dict[str, Any] | None:
    try:
        Centre = np.array([Object["x"], Object["y"]], dtype=float)
        Size = np.asarray(Object["size_cm"], dtype=float)
        Base, Top = float(Object["z"]), float(Object["top_z"])
        if Size.shape != (2,) or not np.all(np.isfinite([*Centre, *Size, Base, Top])) or np.any(Size <= 0) or Top <= Base:
            return None
        # Account for the footprint's percentile trimming. This same envelope
        # is supplied to the carried-object check, not just removed from depth.
        Half = Size / 2 + DepthUncertaintyCm
        if Object.get("bounds_cm"):
            Low = np.asarray(Object["bounds_cm"]["min"], dtype=float)
            High = np.asarray(Object["bounds_cm"]["max"], dtype=float)
            if Low.shape != (3,) or High.shape != (3,) or not np.all(np.isfinite([Low, High])) or np.any(High <= Low):
                return None
            return {"label": str(Object["label"]), "min": [*(Low[:2] - DepthUncertaintyCm).tolist(), Base],
                    "max": [*(High[:2] + DepthUncertaintyCm).tolist(), max(Top, High[2]) + DepthUncertaintyCm]}
        return {"label": str(Object["label"]), "min": [*(Centre - Half).tolist(), Base],
                "max": [*(Centre + Half).tolist(), Top + DepthUncertaintyCm]}
    except (KeyError, TypeError, ValueError):
        return None


def _FreeBoxes(DepthMm, Intrinsics, Model) -> list[dict[str, Any]]:
    """Known-empty cubes; a zero/invalid pixel anywhere in the footprint rejects it."""
    Counts = np.floor((BoundsMax - BoundsMin) / FreeCellCm).astype(int)
    Indices = np.indices(Counts).reshape(3, -1).T
    Low = BoundsMin + Indices * FreeCellCm
    Corners = Low[:, None, :] + np.array(list(product((0.0, FreeCellCm), repeat=3)))
    Camera = DG.DisplayToCamera(Corners.reshape(-1, 3), Model).reshape(-1, 8, 3)
    Positive = Camera[:, :, 2].min(axis=1) > 1.0
    Candidates = np.nonzero(Positive)[0]
    PixelLow, PixelHigh = _ProjectedBounds(Camera[Positive], Intrinsics)
    Min = np.floor(PixelLow - 1).astype(int)
    Max = np.ceil(PixelHigh + 1).astype(int)
    Centre = (Min + Max) // 2
    Radius = np.maximum(Centre - Min, Max - Centre).max(axis=1)
    Height, Width = DepthMm.shape
    Inside = (Min[:, 0] >= 0) & (Min[:, 1] >= 0) & (Max[:, 0] < Width) & (Max[:, 1] < Height)
    SafeDepth = DepthMm.astype(np.float32) / 10.0
    SafeDepth[~np.isfinite(SafeDepth) | (SafeDepth <= 0) | (SafeDepth > DG.MaxDepthCm)] = 0
    Free = np.zeros(len(Candidates), dtype=bool)
    Before = 0
    for R in (4, 8, 16, 32, 64, 128):
        Use = Inside & (Radius > Before) & (Radius <= R)
        if np.any(Use):
            Minimum = cv2.erode(SafeDepth, np.ones((2 * R + 1, 2 * R + 1), np.uint8),
                                borderType=cv2.BORDER_CONSTANT, borderValue=0)
            Seen = Minimum[Centre[Use, 1], Centre[Use, 0]]
            Far = Camera[Candidates[Use], :, 2].max(axis=1)
            Free[Use] = Seen > Far + DepthUncertaintyCm
        Before = R
    return _MergeCells(Indices[Candidates[Free]], FreeCellCm, BoundsMin)


def _ProjectedBounds(Camera, Intrinsics):
    """Bound the whole cube projection, including distortion between corners.

    Positive-Z projective coordinates attain their extrema at vertices. Interval
    arithmetic then bounds the Brown-Conrady distortion over that rectangle.
    Unsupported lens models stay unknown, rather than using a guessed pixel pad.
    """
    Coefficients = DG.Distortion(Intrinsics).ravel()
    if len(Coefficients) not in (4, 5) or not np.all(np.isfinite(Coefficients)):
        raise ValueError("this lens distortion model cannot verify free space")
    K1, K2, P1, P2 = Coefficients[:4]
    K3 = Coefficients[4] if len(Coefficients) == 5 else 0.0
    Normalized = Camera[..., :2] / Camera[..., 2, None]
    Low, High = Normalized.min(axis=1), Normalized.max(axis=1)

    def Multiply(A, B):
        Products = np.stack([A[0] * B[0], A[0] * B[1], A[1] * B[0], A[1] * B[1]])
        return Products.min(axis=0), Products.max(axis=0)

    def Scale(A, Factor):
        return (A[0] * Factor, A[1] * Factor) if Factor >= 0 else (A[1] * Factor, A[0] * Factor)

    def Square(A):
        return np.where((A[0] <= 0) & (A[1] >= 0), 0, np.minimum(A[0] ** 2, A[1] ** 2)), np.maximum(A[0] ** 2, A[1] ** 2)

    X, Y = (Low[:, 0], High[:, 0]), (Low[:, 1], High[:, 1])
    X2, Y2 = Square(X), Square(Y)
    Radius = (X2[0] + Y2[0], X2[1] + Y2[1])
    RadialLow, RadialHigh = np.ones(len(Camera)), np.ones(len(Camera))
    for Power, Coefficient in ((1, K1), (2, K2), (3, K3)):
        Term = Scale((Radius[0] ** Power, Radius[1] ** Power), Coefficient)
        RadialLow += Term[0]
        RadialHigh += Term[1]
    XY = Multiply(X, Y)
    ResultLow, ResultHigh = [], []
    for Axis, Coordinate, Squared, Cross, Tangent in ((0, X, X2, P1, P2), (1, Y, Y2, P2, P1)):
        A = Multiply(Coordinate, (RadialLow, RadialHigh))
        B = Scale(XY, 2 * Cross)
        C = Scale((Radius[0] + 2 * Squared[0], Radius[1] + 2 * Squared[1]), Tangent)
        Focal, Centre = Intrinsics[("fx", "fy")[Axis]], Intrinsics[("cx", "cy")[Axis]]
        ResultLow.append((A[0] + B[0] + C[0]) * Focal + Centre)
        ResultHigh.append((A[1] + B[1] + C[1]) * Focal + Centre)
    return np.column_stack(ResultLow), np.column_stack(ResultHigh)


def Build(DepthMm, Intrinsics, Model, Object: dict[str, Any], Support: dict[str, Any],
          Exclude=None, CaptureSeq=None, TaskEnvelope=None, RemoveTarget=True, ArmBoxes=None) -> dict[str, Any]:
    """A candidate scene, not a motion authorization; the registry stamps it.

    The task object is deliberately separate from obstacles for grasp contact.
    Its supporting plane is kept, including points beneath the object footprint.
    Other observed objects, shelf sides, undersides and back walls remain solid.
    The robot's own points are left out: the arm in ArmBoxes (CaptureArmPose
    "link_boxes", where it stood for this capture), and the cart, basket and mast
    the collision model knows (RobotCells).
    """
    Target = TaskEnvelope if TaskEnvelope is not None else TaskBounds(Object)
    if Target is None:
        raise ValueError("the task object's depth envelope is missing")
    Depth = DepthMm.astype(np.float64) / 10.0
    Valid = (Depth > 0) & (Depth <= DG.MaxDepthCm)
    Rays = DG.RayMap(Intrinsics)[Valid]
    Cloud = DG.CameraToDisplay(np.column_stack((Rays * Depth[Valid, None], Depth[Valid])), Model)
    Cloud = Cloud[np.all((Cloud >= BoundsMin) & (Cloud < BoundsMax), axis=1)]
    if Exclude is not None:
        Cloud = Cloud[~Exclude(Cloud)]
    if ArmBoxes:
        Cloud = Cloud[~ArmPoints(Cloud, ArmBoxes)]
    # The cart, basket and mast: by the occupied cell each point falls in.
    Cells = np.floor((Cloud - BoundsMin) / OccupiedCellCm).astype(np.int32)
    Unique, Inverse = np.unique(Cells, axis=0, return_inverse=True)
    Cloud = Cloud[~RobotCells(Unique)[Inverse.ravel()]]
    Low, High = np.asarray(Target["min"]), np.asarray(Target["max"])
    RemovalLow, RemovalHigh = Low.copy(), High.copy()
    if Object.get("bounds_cm"):
        # The carried envelope has uncertainty padding; that padding must not
        # erase a touching neighbour. Remove only the observed target footprint,
        # with the rim its measurement leaves out (TargetRimCm).
        RemovalLow[:2] = np.asarray(Object["bounds_cm"]["min"])[:2] - TargetRimCm
        RemovalHigh[:2] = np.asarray(Object["bounds_cm"]["max"])[:2] + TargetRimCm
    IsTarget = (np.all((Cloud >= RemovalLow) & (Cloud <= RemovalHigh), axis=1)
                & (Cloud[:, 2] > Low[2] + 0.35) & bool(RemoveTarget))
    Environment = Cloud[~IsTarget]
    Support = dict(Support)
    Above = Environment[(Environment[:, 2] > High[2] + 1.0)
                        & (np.abs(Environment[:, 0] - (Low[0] + High[0]) / 2) < 10.0)
                        & (np.abs(Environment[:, 1] - (Low[1] + High[1]) / 2) < 10.0)]
    if len(Above) >= DG.RingMinPoints:
        Counts, Edges = np.histogram(Above[:, 2], bins=np.arange(np.floor(Above[:, 2].min()), Above[:, 2].max() + 1.0, 0.5))
        for Index in np.argsort(Counts)[::-1][:4]:
            Patch = Above[np.abs(Above[:, 2] - (Edges[Index] + 0.25)) <= 0.5]
            if DG._Level(Patch):
                Level = float(np.percentile(Patch[:, 2], 5))
                Support["OverheadObserved"] = True
                # Looking at a board's top does not measure its underside. Its
                # occupied voxels still count, and occluded space stays unknown.
                if DG.CameraToDisplay(np.zeros(3), Model)[0, 2] < Level:
                    Support.update(Kind="shelf", BoardAboveZCm=round(Level, 2))
                break
    Around = DG._Over(Support, Environment[:, 0], Environment[:, 1], OccupiedCellCm + DepthUncertaintyCm)
    IsSupport = (np.abs(Environment[:, 2] - Support["TopZCm"]) <= 0.35) & Around
    Plane = _SupportPlane(Environment[Around], Support, Low, High)
    Tilt = None
    if Plane is not None:
        # A tilted support (_SupportPlane): its surface within the band of its fitted plane, where that stays within
        # SupportBandCm of the measured level, is the support too, its top that plane's.
        Surface = Environment[:, :2] @ Plane[:2] + Plane[2]
        Near = DG._Over(Support, Environment[:, 0], Environment[:, 1], SupportPlaneReachCm)
        IsSupport |= Near & (np.abs(Environment[:, 2] - Surface) <= 0.35) & (np.abs(Surface - Support["TopZCm"]) <= SupportBandCm)
        Tilt = round(float(np.degrees(np.arctan(np.hypot(*Plane[:2])))), 2)
    # Under the task object nothing lies between it and its support: what is left there at or below the support's
    # level band is the support, seen with its noise and tilt (the cabinet top of 2026-10-06 spread 0.55 cm under
    # the pill bottle), capped at its top like it; as obstacle boxes 1 cm under the object's base they overlapped it.
    IsSupport |= (np.all((Environment[:, :2] >= Low[:2]) & (Environment[:, :2] <= High[:2]), axis=1)
                  & (Environment[:, 2] <= Support["TopZCm"] + 0.35))
    SupportPoints = Environment[IsSupport]
    if not DG._Level(SupportPoints):
        raise ValueError("the supporting surface is not visible in this capture")
    Contact = TaskContact(Environment[~IsSupport], RemovalLow, RemovalHigh, Low[2], High[2]) if RemoveTarget and Object.get("bounds_cm") else None
    Cells = np.floor((Environment[~IsSupport] - BoundsMin) / OccupiedCellCm).astype(np.int32)
    Obstacles = _MergeCells(Cells, OccupiedCellCm, BoundsMin, "observed environment")
    SupportCells = np.floor((SupportPoints - BoundsMin) / OccupiedCellCm).astype(np.int32)
    Obstacles += _MergeCells(SupportCells, OccupiedCellCm, BoundsMin, "observed support")
    # Inflate occupied observations, never free space. Sensor uncertainty is
    # not a reason to pretend gaps beside a board are empty.
    for Box in Obstacles:
        Box["min"] = (np.asarray(Box["min"]) - DepthUncertaintyCm).tolist()
        Box["max"] = (np.asarray(Box["max"]) + DepthUncertaintyCm).tolist()
        if Box["label"] == "observed support":
            # This independently fitted level is an intended contact surface.
            # Its nominal top remains the measured top, just as the planner's
            # support plane does; uncertainty is carried separately in scene.
            # A tilted support's is its plane's over the box (where it rises).
            Cap = Support["TopZCm"]
            if Plane is not None:
                Corners = np.array([[X, Y] for X in (Box["min"][0], Box["max"][0]) for Y in (Box["min"][1], Box["max"][1])])
                Cap = max(Cap, round(float(np.max(Corners @ Plane[:2] + Plane[2])), 3))
            Box["max"][2] = min(Box["max"][2], Cap)
    if not Obstacles or len(Obstacles) > MaxObstacleBoxes:
        raise ValueError("the observed environment is too incomplete or fragmented for a checked route")
    Free = _FreeBoxes(DepthMm, Intrinsics, Model)
    if not Free:
        raise ValueError("no approach space is verified clear in this depth frame")
    Scene = {"frame": "display_cm", "capture_seq": CaptureSeq, "support": Support,
             "task_object": Target, "obstacles": Obstacles, "free_space": Free,
             "observed_bounds": {"min": BoundsMin.tolist(), "max": BoundsMax.tolist()},
             "coverage": "observed_only", "depth_uncertainty_cm": DepthUncertaintyCm}
    if Tilt is not None:
        Scene["support_tilt_deg"] = Tilt
    if RemoveTarget and Object.get("bounds_cm"):
        # Observed solid touching the task object's measured footprint (TaskContact), or None: the app refuses its grasp.
        Scene["task_contact"] = Contact
    # The arm where it stood for the capture: the camera saw the arm there, not through it, and its points (within
    # ArmMarginCm of its boxes) are left out above, so the arm can leave its own place: 2026-10-06 15:01 every move from
    # Home was refused, "Current arm pose: gripper ... enters space not observed by the depth camera". Only
    # KnownArmMarginCm around its boxes is known: the cut-out's 2 cm hid real boards beside the gripper and declared
    # them free.
    Known = ArmSpace(ArmBoxes)
    if AssumeTaskReachFree:
        Reach = TaskReach(Target, Support)
        # What stands there stands on the support: the unobserved space under what the camera saw is solid too.
        for Box in HiddenUnder(Environment[~IsSupport], Free, Reach, Target, Support["TopZCm"]):
            Box["min"] = (np.asarray(Box["min"]) - DepthUncertaintyCm).tolist()
            Box["max"] = (np.asarray(Box["max"]) + DepthUncertaintyCm).tolist()
            Obstacles.append(Box)
        if len(Obstacles) > MaxObstacleBoxes:
            raise ValueError("the observed environment is too incomplete or fragmented for a checked route")
        Known.append(Reach)
        Scene["coverage"] = "observed_and_task_reach"
    if Known:
        Scene["known_space"] = Known
    return Scene


def HiddenUnder(Points: np.ndarray, Free, Reach: dict[str, Any], Target: dict[str, Any], SupportZ: float) -> list[dict[str, Any]]:
    """Within the task's reach (AssumeTaskReachFree), the unobserved cells under an observed solid down to the
    support's top, as obstacle boxes (unpadded): things stand on their support, and the camera sees their tops
    over what hides them. Without this, the task reach took the hidden lower part of whatever stands behind the
    task object as empty (the books behind the pill bottle at the cabinet of 2026-10-06, which the bottle hides
    below Z 11). A cell certified free (Free) ends a column's solid; the task object's envelope is the object, the
    support's level the support (Points: the environment's points, not the support's)."""
    Origin = np.floor((np.asarray(Reach["min"]) - BoundsMin) / OccupiedCellCm).astype(int)
    Shape = np.ceil((np.asarray(Reach["max"]) - BoundsMin) / OccupiedCellCm).astype(int) - Origin
    if np.any(Shape <= 0):
        return []
    Occupied = np.zeros(Shape, dtype=bool)
    Cells = np.floor((Points - BoundsMin) / OccupiedCellCm).astype(int) - Origin
    Cells = Cells[np.all((Cells >= 0) & (Cells < Shape), axis=1)]
    Occupied[tuple(Cells.T)] = True
    Low = BoundsMin + Origin * OccupiedCellCm
    Centres = [Low[Axis] + (np.arange(Shape[Axis]) + 0.5) * OccupiedCellCm for Axis in range(3)]
    Seen = np.zeros(Shape, dtype=bool)
    for Box in [*Free, Target]:
        Spans = [(Centres[Axis] >= Box["min"][Axis]) & (Centres[Axis] <= Box["max"][Axis]) for Axis in range(3)]
        Seen |= Spans[0][:, None, None] & Spans[1][None, :, None] & Spans[2][None, None, :]
    Hidden = np.zeros(Shape, dtype=bool)
    Under = np.zeros(Shape[:2], dtype=bool)
    for Z in range(Shape[2] - 1, -1, -1):
        Under = (Under | Occupied[:, :, Z]) & ~Seen[:, :, Z]
        if Centres[2][Z] > SupportZ:
            Hidden[:, :, Z] = Under & ~Occupied[:, :, Z]
    return _MergeCells(np.argwhere(Hidden) + Origin, OccupiedCellCm, BoundsMin, "hidden under observed environment")


def TaskContact(Points: np.ndarray, Low, High, BaseZ: float, TopZ: float) -> dict[str, Any] | None:
    """Observed solid touching the task object's measured footprint (Low..High, as Build removes it), or None.

    A measurement that took in part of a neighbour (a shelf side, the books behind a bottle) leaves the rest of that
    neighbour against its footprint, where its taken part is no obstacle any more: the object and it were not
    measured apart. TouchMinPoints or more points (not the support) within TouchCm outside the footprint, between
    SupportBandCm over the object's base (the support's level, its tilt and edge) and its top, are such a neighbour:
    a stray pixel is not. (The app compared padded boxes at first, then the 1 cm cells unpadded: on the cabinet of
    2026-10-06 15:01 either met the pill bottle's own rim and, with the bottle moved clear, still refused it; on the
    recorded desks of 2026-10-03/04 the cells met the tilted desk beside all 6 task objects that had a scene.) It still
    takes the object's own edge for a neighbour where the detection box cut it off (a can's lid, 171 points) or its
    silhouette's mixed pixels are many (a pill container, 48): 2 of those 6, refusals on the safe side."""
    Near = (np.all((Points[:, :2] >= np.asarray(Low)[:2] - TouchCm) & (Points[:, :2] <= np.asarray(High)[:2] + TouchCm), axis=1)
            & (Points[:, 2] > BaseZ + SupportBandCm) & (Points[:, 2] <= TopZ))
    if int(Near.sum()) < TouchMinPoints:
        return None
    Touching = Points[Near]
    Lowest = np.round(Touching.min(axis=0), 2)
    return {"points": int(Near.sum()), "min": Lowest.tolist(), "max": np.maximum(np.round(Touching.max(axis=0), 2), Lowest + 0.01).tolist()}


def ArmSpace(Boxes, MarginCm: float | None = None) -> list[dict[str, Any]]:
    """Where the wrist and the gripper stood for the capture: each link box (CaptureArmPose "link_boxes") grown by
    MarginCm (KnownArmMarginCm), as the display-frame box around it (the coverage checks bound a rotated part the
    same way, so a tighter shape would not cover the gripper's own envelope there)."""
    Grow = KnownArmMarginCm if MarginCm is None else MarginCm
    Space = []
    for Box in Boxes or []:
        if Box.get("fixed") or not str(Box.get("name", "")).startswith(("wrist_link", "gripper_link", "moving_jaw")):
            continue
        Extent = np.abs(np.asarray(Box["axes"], dtype=np.float64)) @ (np.asarray(Box["half"], dtype=np.float64) + Grow)
        Centre = np.asarray(Box["centre"], dtype=np.float64)
        Space.append({"min": np.round(Centre - Extent, 3).tolist(), "max": np.round(Centre + Extent, 3).tolist(), "label": f"the arm at the capture ({Box['name']})"})
    return Space


def TaskReach(Target: dict[str, Any], Support: dict[str, Any]) -> dict[str, Any]:
    """The space around the task object whose unobserved part counts as known (AssumeTaskReachFree): its envelope
    grown TaskReachCm sideways, from TaskReachBelowCm under its support's top to TaskReachAboveCm over its top."""
    Low, High = np.asarray(Target["min"], dtype=np.float64), np.asarray(Target["max"], dtype=np.float64)
    Bottom = float(Support["TopZCm"]) - TaskReachBelowCm
    return {"min": [round(float(Low[0] - TaskReachCm), 3), round(float(Low[1] - TaskReachCm), 3), round(Bottom, 3)],
            "max": [round(float(High[0] + TaskReachCm), 3), round(float(High[1] + TaskReachCm), 3), round(float(High[2] + TaskReachAboveCm), 3)],
            "label": "task reach (unobserved space taken as empty)"}
