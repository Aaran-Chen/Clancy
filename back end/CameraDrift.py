"""Is the camera still where it was calibrated? Checked on each capture's own depth.

Why
---
On 2026-09-28 the camera head drooped and slipped 5-18 deg after the 15:04
calibration, and every object position the AI received was 6-11 cm wrong;
nothing compared the camera with its calibration. Self-calibration now saves a
baseline (drift_reference) inside the camera model, and /locate checks every
capture against it before any model call, failing closed.

Desk check (3 of 6 degrees of freedom)
--------------------------------------
The desk plane in CAMERA coordinates, n.p = d with n pointing from the desk
toward the camera (so d < 0 and -d is the camera's distance to the desk). At
fit time it is measured on the calibration's own raw depth; at check time on
the capture's depth, searching only planes within ReferenceConeDeg of the
baseline normal and 30-70 cm away, so the room floor (about 120 cm) can never
win as it did at 19:06. Fit-time noise was 0.06-0.08 deg (max 0.22) and
0.04 cm, so the block limits sit well above it. Absolute tilt is never used: a
fresh calibration already reads 4-6 deg because the desk really slopes.

The change itself is measured on the desk height map the fit saves (DeskChange),
not by comparing two fitted planes: that keeps a mat or notebook laid on the
desk after calibration, or the arm covering part of it, from reading as a turn.

Tape check (the arm's own reference)
------------------------------------
The desk cannot see a pan about its own normal or a slide along it. When the
gripper's red tip tape is in view, its depth-measured position is compared with
forward kinematics plus the calibrated tape offset. Those two motions keep every
point's height above the desk, so a red blob at another height (the eraser on the
desk) or of another size than the fit's tape (arm wiring) is not the tape; a lone
blob that is the tape's size at the tape's height is the tape even far from where
the arm puts it, since that distance is how far the camera turned.

Cart moves (the desk moved, not the camera)
-------------------------------------------
The arm, the camera and the basket ride one wheeled cart; the desk does not. On
2026-10-02 the cart was rolled toward the desk after the 21:09 calibration: the
desk crossed the image and read 1.7 deg / 0.1 cm off its baseline (2.3 deg on the
map, with under half of it agreeing), while the gripper tape sat 0.43-0.44 cm from
where the arm puts it, as at calibration, and every task was blocked. The desk
cannot tell a moved cart from a moved camera; the tape, rigid with the arm, can.
So when the desk check fails only because the desk disagrees with its baseline
(CheckCapture), the tape is judged anyway, its height filter taking this frame's
desk. Tape within CartTapeMaxErrorCm of where the arm puts it and a desk change no
larger than a cart makes (CartTiltMaxDeg, CartShiftMaxCm): the camera has not moved
against the arm, the capture is accepted and the desk is measured again on it
(RemeasuredReference), for the caller to save as the new baseline. Tape farther: a
moved camera, as before. Tape not judgeable (arm off or moving, jaw or roll
changed, tape hidden): blocked as desk_changed_unverified, which names what lets
the tape judge. A desk change beyond a cart's keeps the desk check's block.

What the tape cannot see: a turn about its own line of sight. The 21:46 frame,
re-rendered as a camera turned on the arm would have seen it, reads the parked tape
(41.6 cm away, 0.43 cm off at rest) 0.82-3.5 cm off for droops of 1-5 deg and
1.0-4.1 cm for turns about the camera's vertical of 2-5 deg, and blocks. A roll
about the optical axis moves the tape 0.23 cm per degree: rolls up to 2 deg
(0.57-0.67 cm) pass and the desk is re-measured, leaving an object 20 cm from the
tape's line of sight up to 0.7 cm off; from 3 deg they block. A turn exactly about
the tape's line is bounded only by CartTiltMaxDeg, and a pan about the desk's normal
leaves the desk as it was, so the tape alone judges it, as before.

No desk in view (RobotReference)
--------------------------------
On 2026-10-06 the cart stood at a bookshelf cabinet, no calibrated desk anywhere in view,
and "grab the pill bottle" was refused as unchecked while the gripper tape sat 0.52 cm
from where the arm puts it. With the calibrated desk out of view (_CheckDesk) and the desk
path neither passing nor finding a moved camera, CheckCapture no longer requires the desk:
the camera is assumed where it was calibrated and the gripper tape checks it through every
joint, wrist roll included (within 0.75 cm verified, farther a moved camera, not judgeable
an assumed pass with a warning). The desk path above is unchanged and decides whenever the
calibrated desk is in view.
"""

from __future__ import annotations

import base64
import hashlib
import json
import math
import time
import zlib
from pathlib import Path
from typing import Any

import cv2
import numpy as np

import DepthGeometry

ReferenceVersion = 2
# Block limits against the baseline (see the module notes for the noise they sit above).
MaxNormalChangeDeg = 0.5
MaxOffsetChangeCm = 0.5
# Plane search: normals within this cone of the reference, planes this far from the camera.
ReferenceConeDeg = 35.0
DeskRangeCm = (30.0, 70.0)
InlierCm = 0.8
Hypotheses = 1500
CoarseStep = 8
FineStep = 4
MinDeskPoints = 1500
# Fit-time planes must agree with the fitted camera's own (flat) table before they become the
# baseline. The desk's slope alone put the 15:04 fit's desk 2.8 cm from it, so the gate is loose;
# agreement between the frames then removes the odd plane.
FitPlaneOffsetGateCm = 8.0
FitPlaneAgreeDeg = 1.0
FitPlaneAgreeCm = 1.0
MinReferenceFrames = 6
# Desk height map (DeskChange): one cell per 8 pixels; cells within 1 cm of the baseline plane
# in a quarter of the fit's frames. The change is refined within 0.25 cm, about 2.5 sigma of
# single-frame depth noise at the desk (0.08-0.1 cm at 47 cm on 2026-09-28), so a 0.6 cm mat
# cannot join the desk.
MapStep = 8
MapBuildBandCm = 1.0
MapMinFrameFraction = 0.25
MinMapCells = 200
# At least this share of the calibrated desk cells in view must agree with the fitted change
# (still camera on 2026-09-28: 98.6% or more; 2026-09-29: 76-86%). Below it an item covers so
# much of the desk that it could pass for the desk itself, and the check fails closed.
MinAgreeingShare = 0.67
# Below that share the fitted change is not trusted: 0.4-0.8 cm mats over 30-50% of the desk read
# up to 2.4 deg and 1.6 cm there (replays of the 2026-09-28 and 2026-09-29 frames), while real
# turns fell below it only from 6 deg (the 2026-09-28 droops: 7-13 deg at shares 0.48-0.66).
# There only a turn no thin item can fake is a moved camera; anything less is unchecked.
ThinItemMaxDeg = 3.0
MapUnitCm = 0.01
MapMissing = -32768
MapHypotheses = 200
MapBandCm = 0.25
MapRefineRounds = 4
# Camera motion during the fit: first third of the poses against the last third. With
# 8 poses a third, per-pose noise of 0.08 deg averages to 0.03 deg per third.
MaxFitMotionDeg = 0.15
MaxFitMotionCm = 0.3
MinFitMotionPoses = 6
# A pose whose desk plane is this far from its third's median locked onto another surface
# (the bin floor, 6 cm lower) and is left out of that third's mean.
FitOutlierDeg = 0.5
FitOutlierCm = 0.5
# Tape check. Within TapeNearCm the nearest tape-like blob is the tape; beyond, out to
# TapeSearchCm (a 20 deg pan at 55 cm), a lone tape-like blob is the tape seen elsewhere.
TapeMaxErrorCm = 1.0
TapeNearCm = 5.0
TapeSearchCm = 20.0
TapeRivalMarginCm = 1.5
# Tape-like: height above the desk within this of the arm's tape (a pan or slide along the
# desk keeps it; 15:04 poses -0.3..+0.6 cm), and area within this factor of the fit's
# 10th-90th percentile (15:04: 1.0-2.4 cm^2; the eraser 9, arm wiring 0.05-0.2).
TapeHeightToleranceCm = 2.0
TapeAreaFactor = 2.5
TapeAreaPercentiles = (10, 90)
MinTapeAreas = 5
# Calibrations never command wrist roll or the jaw, so the tape offset holds only near
# the roll and jaw the calibration used (raw steps).
TapeRollToleranceRaw = 25
TapeGripperToleranceRaw = 40
# Cart moves (CheckCapture): the largest desk change, this frame's plane against the baseline's,
# that a cart rolled on the floor makes. 2026-10-02: 1.7 deg / 0.1 cm; a cart with a 50 cm wheelbase
# needs 3.5 cm under one side for 4 deg. Beyond it the desk is another surface or the camera drooped
# (5-18 deg on 2026-09-28), and the desk check's own block stands.
CartTiltMaxDeg = 4.0
CartShiftMaxCm = 3.0
# Once the desk moved, the tape must sit this close to where the arm puts it for the desk to be
# re-measured; farther, both say the camera moved and it blocks as moved. At TapeMaxErrorCm a 2 deg
# turn of the real 21:46 frame passed by 0.03 cm (1.03 cm: the turn and the tape's 0.43 cm standing
# offset partly cancel); here it blocks with 0.28 cm to spare, as does a 1 deg droop (0.82-0.83 cm).
CartTapeMaxErrorCm = 0.75
# Desks a re-measure replaced, newest first, kept in the baseline (each map is about 4 KB).
DeskHistoryLength = 3
# The re-measured baseline rides on the CheckCapture result under this key until the caller takes it.
RemeasuredKey = "remeasured_reference"


def _Unit(Vector) -> np.ndarray:
    Vector = np.asarray(Vector, dtype=np.float64)
    return Vector / np.linalg.norm(Vector)


def AngleDeg(A, B) -> float:
    return math.degrees(math.acos(float(np.clip(np.dot(_Unit(A), _Unit(B)), -1.0, 1.0))))


def CameraPoints(DepthMm: np.ndarray, Intrinsics: dict[str, Any], Step: int) -> np.ndarray:
    """Camera-frame points (cm) of every Step-th valid depth pixel."""
    Depth = DepthMm[::Step, ::Step].astype(np.float32) / 10.0
    Rays = DepthGeometry.RayMap(Intrinsics)[::Step, ::Step]
    Valid = Depth > 0
    return np.concatenate([Rays * Depth[..., None], Depth[..., None]], axis=-1)[Valid].astype(np.float64)


def DeskPlane(DepthMm: np.ndarray, Intrinsics: dict[str, Any], ReferenceNormal=None,
              Seed: int = 0, ReferenceOffset: float | None = None, OffsetGateCm: float | None = None) -> dict[str, Any] | None:
    """The best-supported plane 30-70 cm from the camera (and within the cone of
    ReferenceNormal when given), as {"n", "d", "inliers", "inlier_fraction", "rms_cm"}.

    With ReferenceOffset and OffsetGateCm as well, the plane must also lie within the gate
    of the reference plane n.p = ReferenceOffset, and hypotheses are drawn only from points
    within the gate: where the desk is a small part of the view (2026-10-01: a bin under the
    arm, 7% of the pixels, beside a bookshelf and the floor), triples drawn from the whole
    view rarely land on it, and the winner flipped between cluttered planes 10 deg apart.

    Hypotheses are scored on a coarse grid and the winner is refined on a finer one;
    everything is vectorized because this runs in the bridge beside the servo loop.
    """
    Coarse = CameraPoints(DepthMm, Intrinsics, CoarseStep)
    Fine = CameraPoints(DepthMm, Intrinsics, FineStep)
    if len(Coarse) < 100 or len(Fine) < MinDeskPoints:
        return None
    Gated = ReferenceNormal is not None and ReferenceOffset is not None and OffsetGateCm is not None
    Pool = Coarse
    if Gated:
        Pool = Coarse[np.abs(Coarse @ _Unit(ReferenceNormal) - float(ReferenceOffset)) <= float(OffsetGateCm)]
        if len(Pool) < 50:
            return None
    Rng = np.random.default_rng(Seed)
    Samples = Pool[Rng.integers(0, len(Pool), size=(Hypotheses, 3))]
    Normals = np.cross(Samples[:, 1] - Samples[:, 0], Samples[:, 2] - Samples[:, 0])
    Norms = np.linalg.norm(Normals, axis=1)
    Keep = Norms > 1e-6
    Normals, Samples = Normals[Keep] / Norms[Keep, None], Samples[Keep]
    Offsets = np.einsum("ij,ij->i", Normals, Samples[:, 0])
    Flip = np.where(Offsets > 0, -1.0, 1.0)  # face the camera: d < 0
    Normals, Offsets = Normals * Flip[:, None], Offsets * Flip
    Keep = (-Offsets >= DeskRangeCm[0]) & (-Offsets <= DeskRangeCm[1])
    Cone = math.cos(math.radians(ReferenceConeDeg))
    if ReferenceNormal is not None:
        Keep &= Normals @ _Unit(ReferenceNormal) >= Cone
    if Gated:
        Keep &= np.abs(Offsets - float(ReferenceOffset)) <= float(OffsetGateCm)
    Normals, Offsets = Normals[Keep], Offsets[Keep]
    if not len(Normals):
        return None
    Counts = np.zeros(len(Normals), dtype=np.int64)
    for Start in range(0, len(Normals), 64):
        Block = slice(Start, Start + 64)
        Counts[Block] = (np.abs(Coarse @ Normals[Block].T - Offsets[Block]) <= InlierCm).sum(axis=0)
    Normal, Offset = Normals[int(np.argmax(Counts))], float(Offsets[int(np.argmax(Counts))])
    for _ in range(3):
        Inliers = np.abs(Fine @ Normal - Offset) <= InlierCm
        if Inliers.sum() < MinDeskPoints:
            return None
        Centre = Fine[Inliers].mean(axis=0)
        _, _, Vt = np.linalg.svd(Fine[Inliers] - Centre, full_matrices=False)
        Normal = Vt[2] if Vt[2] @ Centre < 0 else -Vt[2]
        Offset = float(Normal @ Centre)
    Inliers = np.abs(Fine @ Normal - Offset) <= InlierCm
    if not DeskRangeCm[0] <= -Offset <= DeskRangeCm[1] or Inliers.sum() < MinDeskPoints:
        return None
    if ReferenceNormal is not None and float(Normal @ _Unit(ReferenceNormal)) < Cone:
        return None
    if Gated and abs(Offset - float(ReferenceOffset)) > float(OffsetGateCm):
        return None
    Residual = Fine[Inliers] @ Normal - Offset
    return {"n": [float(Value) for Value in Normal], "d": Offset, "inliers": int(Inliers.sum()),
            "inlier_fraction": round(float(Inliers.mean()), 3), "rms_cm": round(float(np.sqrt(np.mean(Residual ** 2))), 3)}


# --------------------------------------------------------------------------
# Desk height map: the change is measured against the calibration's own desk
# --------------------------------------------------------------------------
# A plane fitted to the desk alone could not tell a 0.6-0.8 cm mat or notebook covering part
# of the desk from a turned camera (a plane tilted between them collects both; 0.6-1.4 deg on
# a still camera, at any band). The fit keeps the desk's height, cell by cell, above its own
# baseline plane; a check fits the change of those heights robustly, so the desk's own shape
# cancels, a thin item is a separate offset that loses the vote, and occluded cells drop out.

def GridPoints(DepthMm: np.ndarray, Intrinsics: dict[str, Any], Step: int = MapStep) -> np.ndarray:
    """Camera-frame points (cm) on every Step-th pixel, as a grid; NaN where there is no depth."""
    Depth = DepthMm[::Step, ::Step].astype(np.float64) / 10.0
    Rays = DepthGeometry.RayMap(Intrinsics)[::Step, ::Step].astype(np.float64)
    Points = np.concatenate([Rays * Depth[..., None], Depth[..., None]], axis=-1)
    Points[Depth <= 0] = np.nan
    return Points


def PlaneDepth(Plane: dict[str, Any]) -> np.ndarray | None:
    """The raw depth a fit-time plane was measured on: its file, or the pose archive's 16-bit PNG."""
    Path_ = Plane.get("file")
    if not Path_ and Plane.get("archive") and Plane.get("frame") is not None:
        Archive = Path(Plane["archive"])
        Path_ = Archive.with_name(f"{Archive.stem}_depth{int(Plane['frame'])}.png")
    if not Path_:
        return None
    Depth = cv2.imread(str(Path_), cv2.IMREAD_UNCHANGED)
    return Depth if Depth is not None and Depth.dtype == np.uint16 else None


def BuildDeskMap(Planes: list[dict[str, Any]], Normal: np.ndarray, Offset: float,
                 Intrinsics: dict[str, Any]) -> dict[str, Any] | None:
    """Each cell's desk height above the baseline plane, the median over the fit's frames."""
    return DeskMapFromDepths((Depth for Depth in map(PlaneDepth, Planes) if Depth is not None), Normal, Offset, Intrinsics)


def DeskMapFromDepths(Depths, Normal: np.ndarray, Offset: float, Intrinsics: dict[str, Any]) -> dict[str, Any] | None:
    """BuildDeskMap on depth frames in hand (any iterable, read once); with one frame (a re-measure),
    the cells it sees as desk."""
    Rows = []
    for Depth in Depths:
        Height = GridPoints(Depth, Intrinsics) @ Normal - Offset
        Height[~(np.abs(Height) <= MapBuildBandCm)] = np.nan  # objects, the arm, the floor
        Rows.append(Height)
    if not Rows:
        return None
    Stack = np.stack(Rows)
    Seen = np.isfinite(Stack).sum(axis=0)
    Map = np.full(Stack.shape[1:], np.nan)
    Kept = Seen >= max(3, MapMinFrameFraction * len(Rows)) if len(Rows) >= 3 else Seen == len(Rows)
    Map[Kept] = np.nanmedian(Stack[:, Kept], axis=0)
    if int(Kept.sum()) < MinMapCells:
        return None
    Units = np.where(np.isfinite(Map), np.round(Map / MapUnitCm), MapMissing).astype("<i2")
    return {"step": MapStep, "shape": list(Map.shape), "unit_cm": MapUnitCm, "frames": len(Rows), "cells": int(Kept.sum()),
            "data": base64.b64encode(zlib.compress(Units.tobytes(), 9)).decode("ascii")}


def DeskMap(Reference: dict[str, Any]) -> np.ndarray | None:
    """The baseline's desk heights (cm; NaN off the desk), or None."""
    Stored = Reference.get("desk_map")
    try:
        Units = np.frombuffer(zlib.decompress(base64.b64decode(Stored["data"])), dtype="<i2").reshape(Stored["shape"])
    except (TypeError, KeyError, ValueError, zlib.error):
        return None
    return np.where(Units == MapMissing, np.nan, Units * float(Stored["unit_cm"]))


def DeskChange(DepthMm: np.ndarray, Intrinsics: dict[str, Any], Reference: dict[str, Any],
               Plane: dict[str, Any]) -> dict[str, Any] | None:
    """How the desk moved against the calibration's, from the calibrated desk cells still seen
    as desk: {"angle_deg", "offset_cm" (positive: the camera is farther), "cells", "agreeing_cells"},
    or None when too little of it is visible."""
    Map = DeskMap(Reference)
    if Map is None:
        return None
    Points = GridPoints(DepthMm, Intrinsics, int(Reference["desk_map"]["step"]))
    if Points.shape[:2] != Map.shape:
        return None
    Normal, Offset = np.asarray(Reference["desk_n"], dtype=np.float64), float(Reference["desk_d"])
    with np.errstate(invalid="ignore"):
        Desk = np.abs(Points @ np.asarray(Plane["n"]) - float(Plane["d"])) <= InlierCm
    Cells = Desk & np.isfinite(Map)
    if int(Cells.sum()) < MinMapCells:
        return None
    Seen = Points[Cells]
    Rise = Seen @ Normal - Offset - Map[Cells]
    # The change as a plane over the baseline's own axes: rise = a E1.p + b E2.p + k.
    First = _Unit(np.cross(Normal, [1.0, 0.0, 0.0]))
    Design = np.column_stack([Seen @ First, Seen @ np.cross(Normal, First), np.ones(len(Seen))])
    # Hypotheses ("nothing moved", and planes through random cell triples) are judged by the
    # median of their residuals, not by a vote within a band: a mat is a separate step, and a
    # vote joins it to the desk with a ramp (0.5-1 deg on a still camera). Nor is the start a fit
    # to the cells near zero: when a turn lifts the whole desk, those are its low edge, and their
    # fit flattened a 1 deg turn to 0.33. The median holds only while most cells seen are desk
    # that agrees: with 14-24% of a still camera's cells already off (2026-09-29), a mat over 30-45%
    # of the desk can win it with a ramp. CheckDesk's agreeing-share floor then fails closed.
    Picks = np.random.default_rng(0).integers(0, len(Seen), size=(MapHypotheses, 3))
    Triples = Design[Picks]
    Good = np.abs(np.linalg.det(Triples)) > 1e-6
    Solutions = np.vstack([np.zeros((1, 3)), np.linalg.solve(Triples[Good], Rise[Picks][Good][..., None])[..., 0]])
    Spread = np.empty(len(Solutions))
    for Start in range(0, len(Solutions), 64):
        Block = Solutions[Start:Start + 64]
        Spread[Start:Start + 64] = np.median(np.abs(Design @ Block.T - Rise[:, None]), axis=0)
    Solution = Solutions[int(np.argmin(Spread))]
    # Tukey reweighting, bounded by MapBandCm, so a thin item's cells never join the desk's.
    for _ in range(MapRefineRounds):
        # Rows scaled by the square root of Tukey's biweight.
        Root = np.clip(1.0 - ((Design @ Solution - Rise) / MapBandCm) ** 2, 0.0, None)
        if int((Root > 0).sum()) < MinMapCells // 2:
            break
        Solution = np.linalg.lstsq(Design * Root[:, None], Rise * Root, rcond=None)[0]
    Agreeing = int((np.abs(Design @ Solution - Rise) <= MapBandCm).sum())
    Turned = Normal - Solution[0] * First - Solution[1] * np.cross(Normal, First)
    return {"angle_deg": AngleDeg(Turned, Normal), "offset_cm": Offset - (Offset + float(Solution[2])) / float(np.linalg.norm(Turned)),
            "cells": int(Cells.sum()), "agreeing_cells": Agreeing, "normal": _Unit(Turned).tolist()}


# --------------------------------------------------------------------------
# Baseline, saved in the camera model at fit time
# --------------------------------------------------------------------------

def PredictedDeskPlane(Model: dict[str, Any]) -> tuple[np.ndarray, float]:
    """The fitted camera's own table (display Z = table_z) in camera coordinates."""
    Rotation, Translation, _ = DepthGeometry.PoseFromModel(Model)
    Normal = Rotation[:, 2]  # display up; a mirror in X leaves it alone
    return Normal, float(Normal @ Translation + float(Model["table_z"]))


def _ChecksumPayload(Reference: dict[str, Any], Model: dict[str, Any]) -> bytes:
    """The baseline plus the pose it belongs to: a baseline copied into another calibration fails."""
    Fields = {Key: Value for Key, Value in Reference.items() if Key != "sha256"}
    Fields["model"] = {Key: Model.get(Key) for Key in ("type", "rvec", "tvec", "mirror_x", "table_z", "image_size", "fitted_at")}
    return json.dumps(Fields, sort_keys=True).encode("utf-8")


def Checksum(Reference: dict[str, Any], Model: dict[str, Any]) -> str:
    return hashlib.sha256(_ChecksumPayload(Reference, Model)).hexdigest()


def PoseAverages(Planes: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Per-pose mean plane, in capture order."""
    Poses: dict[Any, list[dict[str, Any]]] = {}
    for Plane in Planes:
        Poses.setdefault(Plane.get("pose_index", len(Poses)), []).append(Plane)
    Rows = []
    for PoseIndex, Frames in Poses.items():
        Rows.append({"pose_index": PoseIndex, "received_at": min(float(F.get("received_at", 0.0)) for F in Frames),
                     "n": _Unit(np.mean([F["n"] for F in Frames], axis=0)).tolist(),
                     "d": float(np.mean([F["d"] for F in Frames])), "frames": len(Frames)})
    return sorted(Rows, key=lambda Row: (Row["received_at"], str(Row["pose_index"])))


def _ThirdMean(Poses: list[dict[str, Any]]) -> tuple[np.ndarray, float, int]:
    """Mean plane of one third of the poses without those far from its median: (n, d, left out)."""
    Median = _Unit(np.median([P["n"] for P in Poses], axis=0))
    MedianOffset = float(np.median([P["d"] for P in Poses]))
    Kept = [P for P in Poses if AngleDeg(P["n"], Median) <= FitOutlierDeg and abs(P["d"] - MedianOffset) <= FitOutlierCm] or Poses
    return _Unit(np.mean([P["n"] for P in Kept], axis=0)), float(np.mean([P["d"] for P in Kept])), len(Poses) - len(Kept)


def MotionDuringFit(Planes: list[dict[str, Any]]) -> dict[str, Any] | None:
    """Desk-normal change between the first and last third of the poses (deg, cm), or None with too few poses.

    Thirds rather than the extremes: a single noisy pose (max 0.22 deg on the 15:04 run)
    must not reject a still camera, while steady creep shows up in full. A pose whose plane
    locked onto another surface is left out of its third, so it cannot pass for motion.
    """
    Poses = PoseAverages(Planes)
    if len(Poses) < MinFitMotionPoses:
        return None
    Third = len(Poses) // 3
    (FirstN, FirstD, FirstOut), (LastN, LastD, LastOut) = _ThirdMean(Poses[:Third]), _ThirdMean(Poses[-Third:])
    Angle = AngleDeg(FirstN, LastN)
    Offset = LastD - FirstD
    return {"angle_deg": round(Angle, 3), "offset_cm": round(Offset, 3), "poses": len(Poses), "poses_per_third": Third,
            "outlier_poses": FirstOut + LastOut, "moved": bool(Angle > MaxFitMotionDeg or abs(Offset) > MaxFitMotionCm)}


# --------------------------------------------------------------------------
# Fit-time planes judged against the fitted camera
# --------------------------------------------------------------------------
# Fit-time planes are recorded at each pose, before any camera is fitted, so each depth frame
# keeps its best-supported plane 30-70 cm away, whatever surface that is. On 2026-10-01 the
# camera looked down at a bin under the arm beside a bookshelf and the floor: 72 of 72 frames
# recorded the bookshelf's side (camera normal about (-0.98, 0.16, -0.10), 62 cm away, 90 deg
# from the desk, 8% inliers), BuildReference kept 0-5 of them and MotionDuringFit read 7-15
# deg of "motion" from planes flipping between vertical surfaces: four tape fits of
# 0.45-0.51 cm (24 of 24 poses) were rejected. Once the camera is fitted, its own table says
# where the desk is (PredictedDeskPlane), so a plane far from it is searched for again there.

def DeskPlanesNearFit(Planes: list[dict[str, Any]], Model: dict[str, Any],
                      Intrinsics: dict[str, Any] | None) -> tuple[list[dict[str, Any]], int]:
    """The fit-time planes on the fitted camera's desk, and how many were searched for again.

    A recorded plane within ReferenceConeDeg and FitPlaneOffsetGateCm of PredictedDeskPlane(Model)
    is kept as recorded (a clear desk keeps the very baseline it always had). Any other is searched
    for again on its archived raw depth (PlaneDepth) within that cone and gate, and dropped when
    the desk is not found there.
    """
    Predicted, PredictedOffset = PredictedDeskPlane(Model)
    Kept, Redetected = [], 0
    for Plane in Planes:
        if AngleDeg(Plane["n"], Predicted) <= ReferenceConeDeg and abs(float(Plane["d"]) - PredictedOffset) <= FitPlaneOffsetGateCm:
            Kept.append(Plane)
            continue
        Depth = PlaneDepth(Plane) if Intrinsics else None
        Found = None if Depth is None else DeskPlane(Depth, Intrinsics, Predicted, ReferenceOffset=PredictedOffset,
                                                      OffsetGateCm=FitPlaneOffsetGateCm)
        if Found is not None:
            Kept.append({**Plane, **Found, "redetected": True,
                         "recorded": {"n": Plane["n"], "d": Plane["d"], "inlier_fraction": Plane.get("inlier_fraction")}})
            Redetected += 1
    return Kept, Redetected


def MotionFromDeskMap(Planes: list[dict[str, Any]], Normal, Offset: float,
                      Intrinsics: dict[str, Any] | None) -> dict[str, Any] | None:
    """Camera motion during the fit measured on the desk's own cells, as /locate measures a capture.

    The first third of the poses maps the desk's height above the baseline plane (BuildDeskMap);
    each frame of the last third is compared with that map (DeskChange), and their turned normals
    and offsets are averaged, leaving out frames FitOutlierDeg / FitOutlierCm from the median.
    MotionDuringFit fits a plane per frame instead, and on a small or cluttered desk those planes
    move with what the arm hides: the 2026-10-01 bin read 0.26-0.34 deg and 0.36-0.50 cm between
    thirds per frame, but 0.01-0.03 deg and 0.02 cm here; a clear desk reads the same either way
    (2026-09-28 22:43: 0.025 against 0.026 deg) and the 0.3 deg turn of 2026-09-28 15:04 reads 0.20
    against 0.21. Same result shape as MotionDuringFit, or None when the raw depth is not on disk,
    the desk cannot be mapped, or fewer than half of the last third's poses are read.
    """
    Poses = PoseAverages(Planes)
    if len(Poses) < MinFitMotionPoses or not Intrinsics:
        return None
    Third = len(Poses) // 3
    FirstPoses, LastPoses = {P["pose_index"] for P in Poses[:Third]}, {P["pose_index"] for P in Poses[-Third:]}
    Normal = _Unit(Normal)
    Map = BuildDeskMap([P for P in Planes if P.get("pose_index", None) in FirstPoses], Normal, float(Offset), Intrinsics)
    if Map is None:
        return None
    Reference = {"desk_n": Normal.tolist(), "desk_d": float(Offset), "desk_map": Map}
    Rows, Unread = [], 0
    for Plane in Planes:
        if Plane.get("pose_index", None) not in LastPoses:
            continue
        Depth = PlaneDepth(Plane)
        Change = None if Depth is None else DeskChange(Depth, Intrinsics, Reference, Plane)
        if Change is None or Change["agreeing_cells"] < MinAgreeingShare * Change["cells"]:
            Unread += 1
            continue
        Rows.append((Plane["pose_index"], np.asarray(Change["normal"]), float(Change["offset_cm"])))
    if len({Pose for Pose, _, _ in Rows}) * 2 < Third:
        return None
    Median = _Unit(np.median([Turned for _, Turned, _ in Rows], axis=0))
    MedianOffset = float(np.median([Shift for _, _, Shift in Rows]))
    Kept = [Row for Row in Rows if AngleDeg(Row[1], Median) <= FitOutlierDeg and abs(Row[2] - MedianOffset) <= FitOutlierCm] or Rows
    Angle = AngleDeg(np.mean([Turned for _, Turned, _ in Kept], axis=0), Normal)
    Shift = float(np.mean([Value for _, _, Value in Kept]))
    return {"angle_deg": round(Angle, 3), "offset_cm": round(Shift, 3), "poses": len(Poses), "poses_per_third": Third,
            "outlier_poses": len({Row[0] for Row in Rows} - {Row[0] for Row in Kept}), "method": "desk_map",
            "frames_read": len(Rows), "frames_left_out": len(Rows) - len(Kept), "frames_unread": Unread, "map_cells": Map["cells"],
            "moved": bool(Angle > MaxFitMotionDeg or abs(Shift) > MaxFitMotionCm)}


def TapeAreaRange(AreasCm2) -> list[float] | None:
    """The fit's tape area (cm^2), 10th to 90th percentile over its poses, or None with too few."""
    Areas = np.asarray([Value for Value in (AreasCm2 if AreasCm2 is not None else []) if np.isfinite(Value) and Value > 0])
    if len(Areas) < MinTapeAreas:
        return None
    return [round(float(Value), 3) for Value in np.percentile(Areas, TapeAreaPercentiles)]


def BuildReference(Planes: list[dict[str, Any]], Model: dict[str, Any], TapePose: dict[str, Any] | None = None,
                   Keyframe: str | None = None, Intrinsics: dict[str, Any] | None = None,
                   TapeAreasCm2=None) -> tuple[dict[str, Any] | None, str | None]:
    """Baseline from fit-time desk planes that agree with the fitted camera: (reference, problem).

    Each plane's raw depth (PlaneDepth) with Intrinsics gives the desk height map;
    TapeAreasCm2, the tape's area at each fitted pose, tells the tape from other red.
    """
    Predicted, PredictedOffset = PredictedDeskPlane(Model)
    Usable = [P for P in Planes if AngleDeg(P["n"], Predicted) <= ReferenceConeDeg
              and abs(float(P["d"]) - PredictedOffset) <= FitPlaneOffsetGateCm]
    if len(Usable) < MinReferenceFrames:
        return None, (f"the desk plane was found in depth at only {len(Usable)} of {len(Planes)} frames "
                      f"(need {MinReferenceFrames}), so later captures could not be checked against this calibration")
    Median = _Unit(np.median([P["n"] for P in Usable], axis=0))
    MedianOffset = float(np.median([P["d"] for P in Usable]))
    Agree = [P for P in Usable if AngleDeg(P["n"], Median) <= FitPlaneAgreeDeg and abs(float(P["d"]) - MedianOffset) <= FitPlaneAgreeCm]
    if len(Agree) < MinReferenceFrames:
        return None, f"only {len(Agree)} fit-time desk planes agree with each other; the desk or the camera moved"
    Normal = _Unit(np.mean([P["n"] for P in Agree], axis=0))
    Offset = float(np.mean([P["d"] for P in Agree]))
    Map = BuildDeskMap(Agree, Normal, Offset, Intrinsics) if Intrinsics else None
    if Map is None:
        return None, ("the desk could not be mapped from the calibration's raw depth, so later captures "
                      "could not be checked against this calibration")
    Reference = {
        "version": ReferenceVersion,
        "method": "desk_plane",
        "desk_n": [float(Value) for Value in Normal],
        "desk_d": Offset,
        "frames": len(Agree),
        "normal_noise_deg": round(float(np.std([AngleDeg(P["n"], Normal) for P in Agree])), 4),
        "offset_noise_cm": round(float(np.std([float(P["d"]) for P in Agree])), 4),
        "fitted_at": Model.get("fitted_at"),
        "tape_pose": TapePose,
        "tape_area_cm2": TapeAreaRange(TapeAreasCm2),
        "keyframe": Keyframe,
        "desk_map": Map,
    }
    Reference["sha256"] = Checksum(Reference, Model)
    return Reference, None


def VerifiedReference(Model: dict[str, Any] | None) -> tuple[dict[str, Any] | None, str]:
    """The model's baseline when present and intact, else (None, why)."""
    if not Model:
        return None, "the camera is not calibrated"
    Reference = Model.get("drift_reference")
    if not isinstance(Reference, dict) or Reference.get("method") != "desk_plane":
        return None, "the calibration has no camera-position baseline"
    try:
        Intact = Reference.get("sha256") == Checksum(Reference, Model)
    except (TypeError, ValueError):
        Intact = False
    if not Intact:
        return None, "the calibration's camera-position baseline does not match its checksum"
    if DeskMap(Reference) is None:
        return None, "the calibration's camera-position baseline has no desk map (it predates the map)"
    return Reference, ""


def DisplayDeskPlane(Model: dict[str, Any] | None) -> tuple[np.ndarray, float] | None:
    """The calibrated desk as measured at fit time, in the display frame: m.P = e (m up).

    This is the sloped desk the supports are judged against (about -2.7 at Y 30 and
    -1.9 at Y 40 on 2026-09-28), not the flat table_z.
    """
    Reference, _ = VerifiedReference(Model)
    if Reference is None:
        return None
    Rotation, Translation, Mirror = DepthGeometry.PoseFromModel(Model)
    Normal = np.asarray(Reference["desk_n"], dtype=np.float64)
    Display = Rotation.T @ Normal
    if Mirror:
        Display = Display * DepthGeometry.MirrorVector
    return Display, float(Reference["desk_d"] - Normal @ Translation)


def RemeasuredReference(DepthMm: np.ndarray, Intrinsics: dict[str, Any], Model: dict[str, Any], Plane: dict[str, Any],
                        Source: dict[str, Any], MeasuredAt: str | None = None) -> dict[str, Any] | None:
    """The model's baseline with its desk measured again on one capture, after a cart move.

    desk_n, desk_d and desk_map come from this frame (Plane, as DeskPlane found it); the tape pose
    and areas, the keyframe and the fit's time are kept, and the camera fit is never touched. The
    desk it replaces goes to desk_history (newest first); desk_remeasured_at and Source (seq, tape
    error, desk change) say where the new one came from. None without a verified baseline, or when
    too little desk is in view to map.
    """
    Reference, _ = VerifiedReference(Model)
    if Reference is None:
        return None
    Normal, Offset = _Unit(Plane["n"]), float(Plane["d"])
    Map = DeskMapFromDepths([DepthMm], Normal, Offset, Intrinsics)
    if Map is None:
        return None
    Replaced = {Key: Reference.get(Key) for Key in ("desk_n", "desk_d", "desk_map", "frames")}
    Replaced["measured_at"] = Reference.get("desk_remeasured_at") or Reference.get("fitted_at")
    Replaced["source"] = "remeasure" if Reference.get("desk_remeasured_at") else "calibration"
    New = {Key: Value for Key, Value in Reference.items() if Key != "sha256"}
    New.update({"desk_n": [float(Value) for Value in Normal], "desk_d": Offset, "desk_map": Map, "frames": 1,
                "normal_noise_deg": None, "offset_noise_cm": None,
                "desk_remeasured_at": MeasuredAt or time.strftime("%Y-%m-%dT%H:%M:%S"), "desk_remeasured_from": Source,
                "desk_history": [Replaced, *(Reference.get("desk_history") or [])][:DeskHistoryLength]})
    New["sha256"] = Checksum(New, Model)
    return New


def TakeRemeasuredReference(Result: dict[str, Any]) -> dict[str, Any] | None:
    """Remove and return the re-measured baseline a CheckCapture result carries, if any: the
    caller saves it (calibration.save_remeasured_desk) and it never leaves the bridge."""
    return Result.pop(RemeasuredKey, None)


# --------------------------------------------------------------------------
# Checks on a capture
# --------------------------------------------------------------------------

def _Clock(FittedAt: Any) -> str:
    Text = str(FittedAt or "")
    return Text[11:16] if len(Text) >= 16 else Text or "the last calibration"


def CheckDesk(DepthMm: np.ndarray | None, Intrinsics: dict[str, Any] | None, Model: dict[str, Any] | None,
              Seq: int | None = None) -> dict[str, Any]:
    """Compare the desk in this depth frame with the calibration's baseline.

    Returns the camera_check contract: {"ok", "angle_deg", "offset_cm", "reason",
    "reference_fitted_at", "seq"} plus "error_code" and "error" (one plain sentence)
    when blocked: camera_not_verified, camera_check_failed or camera_moved.
    """
    return _CheckDesk(DepthMm, Intrinsics, Model, Seq)[0]


def _CheckDesk(DepthMm, Intrinsics, Model, Seq) -> tuple[dict[str, Any], dict[str, Any] | None, bool, bool]:
    """CheckDesk's result, the desk plane found in this frame, whether the check failed only
    because that desk disagrees with the baseline (what a cart rolled against the desk does too):
    the plane is off the calibrated cells, too little of the desk agrees, or it turned or shifted;
    and whether the calibrated desk is out of view: no desk plane near it, or fewer than
    MinMapCells of its cells seen as desk (CheckCapture then asks the robot)."""
    Started = time.perf_counter()
    Reference, Why = VerifiedReference(Model)
    Result: dict[str, Any] = {"ok": False, "angle_deg": None, "offset_cm": None, "reason": "",
                              "reference_fitted_at": Reference.get("fitted_at") if Reference else None, "seq": Seq}
    Plane = None

    def Blocked(Code: str, Reason: str, Error: str, Changed: bool = False, Absent: bool = False):
        Result.update(error_code=Code, reason=Reason, error=Error, check_ms=round((time.perf_counter() - Started) * 1000, 1))
        return Result, Plane, Changed, Absent

    if Reference is None:
        return Blocked("camera_not_verified", Why,
                       f"Object positions are blocked: {Why}. Run Automatic calibration with the depth camera.")
    if DepthMm is None or Intrinsics is None:
        return Blocked("camera_check_failed", "no depth frame for this capture",
                       "The camera position could not be checked: this capture has no depth frame. Try again.")
    # Near the calibrated desk first (a small desk, like the 2026-10-01 bin, was found in 21 of 36 fit
    # frames by the cone search alone and in 36 of 36 this way), then anywhere in the cone.
    Plane = (DeskPlane(DepthMm, Intrinsics, Reference["desk_n"], ReferenceOffset=Reference["desk_d"],
                       OffsetGateCm=FitPlaneOffsetGateCm)
             or DeskPlane(DepthMm, Intrinsics, Reference["desk_n"]))
    if Plane is None:
        return Blocked("camera_check_failed",
                       f"no desk plane within {ReferenceConeDeg:g} deg of the calibrated desk, {DeskRangeCm[0]:g}-{DeskRangeCm[1]:g} cm away",
                       "The camera position could not be checked: the desk is not visible in depth. "
                       "Clear the view of the desk, or lock the camera head and recalibrate.", Absent=True)
    PlaneAngle = round(AngleDeg(Plane["n"], Reference["desk_n"]), 3)
    Change = DeskChange(DepthMm, Intrinsics, Reference, Plane)
    if Change is None:
        Result["plane_angle_deg"] = PlaneAngle
        return Blocked("camera_check_failed", f"fewer than {MinMapCells} cells of the calibrated desk are visible as desk",
                       "The camera position could not be checked: too little of the calibrated desk is visible in depth. "
                       "Clear the view of the desk, or lock the camera head and recalibrate.", Changed=True, Absent=True)
    # Positive offset: the camera is now farther from the desk than at calibration.
    Angle, Offset = Change["angle_deg"], Change["offset_cm"]
    Share = Change["agreeing_cells"] / Change["cells"]
    Result.update(angle_deg=round(Angle, 3), offset_cm=round(Offset, 3), desk_inlier_fraction=Plane["inlier_fraction"],
                  desk_cells=Change["cells"], desk_agreeing_share=round(Share, 3), plane_angle_deg=PlaneAngle)
    Moved = Angle > MaxNormalChangeDeg or abs(Offset) > MaxOffsetChangeCm
    # Too little agreement: the fit may be a thin item's ramp or height, so only a turn beyond
    # what one fakes is reported as a moved camera (below).
    if Share < MinAgreeingShare and Angle <= ThinItemMaxDeg:
        return Blocked("camera_check_failed",
                       f"only {Share:.0%} of the calibrated desk in view agrees with one camera pose (need {MinAgreeingShare:.0%})"
                       + (f"; the {Angle:.2f} deg / {Offset:+.2f} cm it reads is within what an item on the desk can fake" if Moved else ""),
                       "The camera position could not be checked: much of the desk looks different from calibration. "
                       "Clear the desk, or lock the camera head and recalibrate.", Changed=True)
    if Moved:
        return Blocked("camera_moved",
                       f"desk plane changed by {Angle:.2f} deg / {Offset:+.2f} cm (limits {MaxNormalChangeDeg} deg / {MaxOffsetChangeCm} cm)",
                       f"The camera or the cart moved since calibration at {_Clock(Reference.get('fitted_at'))}: "
                       f"the desk looks {Angle:.1f} deg / {abs(Offset):.1f} cm different. "
                       "Lock the camera head, then run Automatic calibration.", Changed=True)
    Result.update(ok=True, reason="desk matches the calibration", check_ms=round((time.perf_counter() - Started) * 1000, 1))
    return Result, Plane, False, False


def TapeCheck(ImageBgr: np.ndarray | None, DepthMm: np.ndarray | None, Intrinsics: dict[str, Any] | None,
              Model: dict[str, Any] | None, Arm: dict[str, Any] | None, Desk: dict[str, Any] | None = None) -> dict[str, Any]:
    """Compare the gripper's red tip tape, measured in depth, with forward kinematics.

    ImageBgr is the capture in native (unmirrored) orientation. Arm is the bridge's
    CaptureArmPose: {"ok", "joints": display-frame joints, "servos_raw": {...}}.
    tape_check: "ok", "moved" (blocks), or a reason it could not be judged
    ("not_visible", "ambiguous", "unavailable", "pose_differs"), which never blocks
    on its own (CheckCapture). Desk, the desk plane found in this frame ({"n", "d"}),
    sets the height filter after a cart move; else the baseline's.
    """
    import SelfCalibration  # local import: SelfCalibration imports this module at fit time

    Result: dict[str, Any] = {"tape_check": "unavailable", "tape_error_cm": None}
    Reference, _ = VerifiedReference(Model)
    Tape = (Model or {}).get("marker_corners_gripper_cm") or []
    if Reference is None or len(Tape) != 1 or ImageBgr is None or DepthMm is None or Intrinsics is None:
        return Result
    if not Arm or not Arm.get("ok"):
        Result["tape_reason"] = (Arm or {}).get("reason", "no arm pose for this capture")
        return Result
    Pose = Reference.get("tape_pose") or {}
    Raw = Arm.get("servos_raw") or {}
    for Name, Tolerance in (("wrist_roll", TapeRollToleranceRaw), ("gripper", TapeGripperToleranceRaw)):
        Expected = Pose.get(f"{Name}_raw")
        if Expected is None or Name not in Raw:
            continue
        Difference = abs(int(Raw[Name]) - int(Expected)) % 4096
        if min(Difference, 4096 - Difference) > Tolerance:
            Result.update(tape_check="pose_differs", tape_reason=f"{Name} differs from the calibration's by {min(Difference, 4096 - Difference)} steps")
            return Result
    Tip, Frame = SelfCalibration.GripperFrame(np.asarray(Arm["joints"], dtype=np.float64))
    Predicted = DepthGeometry.DisplayToCamera(Tip + Frame @ np.asarray(Tape[0], dtype=np.float64), Model)[0]
    Height, Width = DepthMm.shape[:2]
    Pixel = DepthGeometry.Project(Predicted, Intrinsics)[0]
    Result["tape_predicted_px"] = [round(float(Value), 1) for Value in Pixel]
    if Predicted[2] <= 5.0 or not (10 <= Pixel[0] < Width - 10 and 10 <= Pixel[1] < Height - 10):
        Result.update(tape_check="not_visible", tape_reason="the tape is predicted outside the image")
        return Result
    # The basket rim's red strip is never the tape (SelfCalibration.BasketStripExclusion).
    Exclude = SelfCalibration.BasketStripExclusion(Model, Pixel, Intrinsics, (Width, Height))
    Detection = SelfCalibration.DetectRedDetailed(ImageBgr, Exclude=Exclude)
    if Detection["excluded"]:
        Result["tape_basket_strip_excluded"] = Detection["excluded"]
    Normal = _Unit(Reference["desk_n"])
    Areas = Reference.get("tape_area_cm2")
    Scored, Others = [], 0
    for Candidate in Detection.get("options", []):
        Point = SelfCalibration._BlobCameraPoint(DepthMm, Candidate, Intrinsics)
        Depth = float(Point[2]) if Point is not None else float(Predicted[2])
        Area = float(Candidate["area"]) * (Depth / float(Intrinsics["fx"])) ** 2
        # Height above the desk against the arm's tape (the desk check has just held the plane).
        Rise = float(Normal @ (Point - Predicted)) if Point is not None else None
        if Desk is not None and Point is not None:
            # After a cart move this frame's desk sets it: the blob is tape-like where the arm puts the
            # tape, or at the arm's tape height above the desk, each measured in its own frame. That
            # second test keeps a camera that drooped 5 deg on the arm (the tape 3.6 cm off, 2-3 cm of
            # it along the desk's normal) reading "moved"; the eraser on the desk still fails both.
            Seen = _Unit(Desk["n"])
            Rise = min(abs(float(Seen @ (Point - Predicted))),
                       abs(float(Seen @ Point - float(Desk["d"])) - float(Normal @ Predicted - float(Reference["desk_d"]))))
        if ((Areas and not Areas[0] / TapeAreaFactor <= Area <= Areas[1] * TapeAreaFactor)
                or (Rise is not None and abs(Rise) > TapeHeightToleranceCm)):
            Others += 1  # the eraser on the desk, or a sliver of wiring: not the tape
            continue
        if Point is not None:
            Error = float(np.linalg.norm(Point - Predicted))
        else:  # no depth on the blob: lateral error only
            Error = float(np.linalg.norm(np.asarray(Candidate["centroid"]) - Pixel)) * float(Predicted[2]) / float(Intrinsics["fx"])
        Scored.append((Error, Point is not None, Candidate))
    Scored.sort(key=lambda Item: Item[0])
    Result["tape_other_blobs"] = Others
    Near = [Item for Item in Scored if Item[0] <= TapeNearCm]
    # Beyond the near search, only a blob measured in depth (so of the tape's height) can be the tape.
    Far = [Item for Item in Scored if Item[1] and Item[0] <= TapeSearchCm]
    if Near:
        Error, _, Best = Near[0]
    elif len(Far) == 1:
        Error, _, Best = Far[0]  # one tape, seen where the arm is not: the camera turned or slid
    else:
        Result.update(tape_check="ambiguous" if Far else "not_visible",
                      tape_reason=("several tape-like red blobs, none where the arm puts the tape" if Far else
                                   "no red blob of the tape's size at the tape's height near where the arm puts it"),
                      tape_nearest_cm=round(Scored[0][0], 2) if Scored else None)
        return Result
    Result.update(tape_error_cm=round(Error, 2), tape_measured_px=[round(float(Value), 1) for Value in Best["centroid"]])
    # Where the tape is against where the arm puts it, in the display frame (measured - predicted):
    # the grasp planner moves the fixed jaw's grasp point by it (frontend GraspPlan.ts TapeCorrection).
    Measured = SelfCalibration._BlobCameraPoint(DepthMm, Best, Intrinsics)
    if Measured is not None:
        Residual = DepthGeometry.CameraToDisplay(Measured, Model)[0] - DepthGeometry.CameraToDisplay(Predicted, Model)[0]
        Result["tape_residual_cm"] = [round(float(Value), 2) for Value in Residual]
    if len(Near) > 1 and Near[1][0] - Error < TapeRivalMarginCm:
        Result.update(tape_check="ambiguous", tape_reason="more than one red blob near the predicted tape")
        return Result
    Result["tape_check"] = "moved" if Error > TapeMaxErrorCm else "ok"
    return Result


def CheckCapture(DepthMm, Intrinsics, Model, Seq=None, ImageBgr=None, Arm=None) -> dict[str, Any]:
    """Desk check, then the tape check; one camera_check result.

    When the desk passes, a tape off where the arm puts it blocks as before. When the desk fails
    only because it disagrees with its baseline and the calibration has a tape offset, the tape
    tells a cart moved against the desk from a moved camera (_DeskChanged): an accepted capture
    carries desk_remeasured, desk_change (the desk against the old baseline) and, under
    RemeasuredKey, the re-measured baseline for the caller to take (TakeRemeasuredReference).

    That desk path runs first and decides whenever the calibrated desk is in view, unchanged. With
    the calibrated desk out of view (no desk plane near it, or too few of its cells seen as desk) and
    the desk path neither passing nor finding a moved camera, the desk is not required: the camera is
    assumed where it was calibrated and the gripper tape checks it through every joint
    (RobotReference.Check: alignment_source "robot", robot_check "verified", "assumed" or "moved"; the
    desk path's own result under "desk_check"). Otherwise the robot's reading rides beside the desk
    path's result under RobotReference.ShadowKey, logged and never deciding.
    """
    import RobotReference  # local import: RobotReference imports this module

    Result, Absent = _DeskCapture(DepthMm, Intrinsics, Model, Seq, ImageBgr, Arm)
    if Result["ok"] or Result.get("error_code") == "camera_moved" or not Absent:
        return RobotReference.WithShadow(Result, ImageBgr, DepthMm, Intrinsics, Model, Arm)
    return RobotReference.Check(ImageBgr, DepthMm, Intrinsics, Model, Arm, Seq, DeskCheck=Result)


def _DeskCapture(DepthMm, Intrinsics, Model, Seq, ImageBgr, Arm) -> tuple[dict[str, Any], bool]:
    """CheckCapture's desk path (the desk, then the roll-gated tape and the cart move, as at 8f4d27a):
    its result, and whether the calibrated desk was out of view (_CheckDesk)."""
    Result, Plane, Changed, Absent = _CheckDesk(DepthMm, Intrinsics, Model, Seq)
    # Without a tape offset nothing can tell a moved cart from a moved camera: the desk's block stands.
    Changed = Changed and len((Model or {}).get("marker_corners_gripper_cm") or []) == 1
    if not Result["ok"] and not Changed:
        return Result, Absent
    Started = time.perf_counter()
    Tape = TapeCheck(ImageBgr, DepthMm, Intrinsics, Model, Arm, Desk=Plane if Changed else None)
    Tape["tape_ms"] = round((time.perf_counter() - Started) * 1000, 1)
    if Changed:
        return _DeskChanged(Result, Plane, Tape, DepthMm, Intrinsics, Model, Seq), Absent
    Result.update(Tape)
    if Result["tape_check"] == "moved":
        Result.update(ok=False, error_code="camera_moved", **_TapeMovedText(Result, Model))
    return Result, Absent


def Verdict(Check: dict[str, Any] | None) -> str:
    """One word for a camera_check (GET /depthcam/check): "desk_verified", "desk_remeasured", or with no
    calibrated desk in view "verified" or "assumed"; else its error_code (camera_moved and the rest)."""
    if not Check:
        return "camera_not_verified"
    if Check.get("ok"):
        if Check.get("alignment_source") == "robot":
            return str(Check.get("robot_check") or "verified")
        return "desk_remeasured" if Check.get("desk_remeasured") else "desk_verified"
    return str(Check.get("error_code") or "camera_check_failed")


def _TapeMovedText(Tape: dict[str, Any], Model: dict[str, Any], Limit: float = TapeMaxErrorCm) -> dict[str, str]:
    Reference, _ = VerifiedReference(Model)
    return {"reason": f"gripper tape is {Tape['tape_error_cm']:.2f} cm from where the arm puts it (limit {Limit} cm)",
            "error": f"The camera moved since calibration at {_Clock(Reference.get('fitted_at'))}: the gripper appears "
                     f"{Tape['tape_error_cm']:.1f} cm from where the arm is. Lock the camera head, then run Automatic calibration."}


def _TapeAdvice(Tape: dict[str, Any]) -> tuple[str, str]:
    """Why the tape could not judge the camera, and what lets it: (a clause, an instruction)."""
    State, Reason = Tape.get("tape_check"), str(Tape.get("tape_reason") or "")
    Park = "Park the arm with its wrist roll and jaw as at calibration, torque on and the red tape in view, and hold it still"
    if State == "pose_differs":
        return f"the wrist roll or jaw is not where it was at calibration ({Reason})", Park
    if State == "not_visible":
        return ("the tape is outside the camera's view at this arm pose" if "outside the image" in Reason
                else "the red tape is not seen where the arm puts it (covered, turned away or out of view)"), Park
    if State == "ambiguous":
        return "more than one red thing near the gripper looks like the tape", "Move red things away from the gripper and keep the arm parked"
    if Reason == "arm not connected":
        return "the arm is not connected", "Connect the arm, then park it with torque on and the red tape in view"
    if Reason == "torque off":
        return "the arm's torque is off, so its pose cannot be trusted", "Turn the torque on with the arm parked and the red tape in view"
    if "mov" in Reason or "joint record" in Reason:
        return "the arm was moving around the capture", "Hold the arm still in its parked pose with torque on"
    return f"the tape could not be looked for ({Reason or 'no colour image or arm pose for this capture'})", Park


def _DeskChanged(Desk: dict[str, Any], Plane: dict[str, Any], Tape: dict[str, Any], DepthMm: np.ndarray,
                 Intrinsics: dict[str, Any], Model: dict[str, Any], Seq) -> dict[str, Any]:
    """The desk disagrees with its baseline (Desk, CheckDesk's block); the gripper tape tells a cart
    rolled against the desk (the camera still where it was on the arm: the desk is re-measured on this
    frame) from a moved camera. The change judged is this frame's plane against the baseline's."""
    Reference, _ = VerifiedReference(Model)
    Tilt, Shift = AngleDeg(Plane["n"], Reference["desk_n"]), float(Plane["d"]) - float(Reference["desk_d"])
    Change = {"angle_deg": Desk["angle_deg"], "offset_cm": Desk["offset_cm"], "agreeing_share": Desk.get("desk_agreeing_share"),
              "plane_angle_deg": round(Tilt, 3), "plane_offset_cm": round(Shift, 3), "error_code": Desk["error_code"],
              "reason": Desk["reason"]}
    Moved = f"the desk moved {Tilt:.2f} deg / {Shift:+.2f} cm against the camera since calibration"
    if Tape["tape_check"] == "ok" and Tape["tape_error_cm"] > CartTapeMaxErrorCm:
        Tape = {**Tape, "tape_check": "moved"}  # both the desk and the tape say the camera moved
    Blocked = {**Desk, **Tape, "desk_change": Change}
    if Tape["tape_check"] == "moved":
        Text = _TapeMovedText(Tape, Model, CartTapeMaxErrorCm)
        return {**Blocked, "error_code": "camera_moved", "reason": f"{Text['reason']}; {Moved}", "error": Text["error"]}
    if Tilt > CartTiltMaxDeg or abs(Shift) > CartShiftMaxCm:
        return {**Blocked, "reason": f"{Desk['reason']}; {Moved}, more than moving the cart does "
                                     f"(limits {CartTiltMaxDeg:g} deg / {CartShiftMaxCm:g} cm)"}
    if Tape["tape_check"] != "ok":
        Why, Fix = _TapeAdvice(Tape)
        return {**Blocked, "error_code": "desk_changed_unverified",
                "reason": f"{Moved}, and the gripper tape could not check the camera: {Tape.get('tape_reason') or Tape['tape_check']}",
                "error": f"The desk moved against the arm since calibration at {_Clock(Reference.get('fitted_at'))} (was the cart "
                         f"moved?), so the camera is checked against the red tape on the gripper instead, and {Why}. {Fix}, then send again."}
    # The tape is where the arm puts it: the camera has not moved on the arm, the desk has.
    Started = time.perf_counter()
    New = RemeasuredReference(DepthMm, Intrinsics, Model, Plane, {
        "seq": Seq, "tape_error_cm": Tape["tape_error_cm"], "tape_residual_cm": Tape.get("tape_residual_cm"), "desk_change": Change})
    # The new baseline must hold on the very frame it came from.
    Checked = _CheckDesk(DepthMm, Intrinsics, dict(Model, drift_reference=New), Seq)[0] if New is not None else None
    if Checked is None or not Checked["ok"]:
        return {**Blocked, "error_code": "camera_check_failed",
                "reason": f"{Moved}; the gripper tape agrees with the arm, but the desk could not be re-measured on this frame"
                          + (f" ({Checked['reason']})" if Checked else ""),
                "error": "The desk moved against the arm since calibration (was the cart moved?) and the camera is still where it was "
                         "on the arm, but too little of the desk is in view to measure it again. Clear the view of the desk, then send again."}
    return {**Checked, **Tape, "desk_remeasured": True, "desk_change": Change,
            "remeasure_ms": round((time.perf_counter() - Started) * 1000, 1),
            "reason": f"desk re-measured on this capture: {Moved} (the cart moved), and the gripper tape is "
                      f"{Tape['tape_error_cm']:.2f} cm from where the arm puts it (limit {CartTapeMaxErrorCm} cm)",
            RemeasuredKey: New}
