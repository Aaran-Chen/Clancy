"""Independent camera-to-cart alignment from archived RGB + metric depth.

Only fixed basket rim features from the signed calibration keyframe are anchors.
The basket floor is deliberately excluded: the real archive contains loose paper
there. A current shelf, desk, or arm never becomes a reference. Missing, repeated,
occluded or narrowly clustered features return None, never an alignment pass.
No image is captured here, no calibration is changed, and no hardware is used.
"""
from __future__ import annotations

import base64
import json
import math
import time
from functools import lru_cache
from pathlib import Path

import cv2
import numpy as np

import DepthGeometry

MaxRotationDeg = 1.0
MaxTranslationCm = 1.0
MinMatches = 18
MinInliers = 15
MinInlierShare = 0.70
FitBandCm = 0.35
MaxRmsCm = 0.20
MinLongSpreadCm = 3.0
MinSecondSpreadCm = 1.0
MinSpatialCells = 8
DescriptorRatio = 0.70
MaxDescriptorDistance = 48
ArmExclusionCm = 6.0


def _Joints(Arm):
    if not isinstance(Arm, dict) or not Arm.get("ok"):
        return None
    try:
        Rows = Arm["joints"]
        Points = np.asarray([[P["x"], P["y"], P["z"]] if isinstance(P, dict) else P for P in Rows], dtype=np.float64)
        return Points if Points.ndim == 2 and Points.shape[1] == 3 and len(Points) >= 2 and np.isfinite(Points).all() else None
    except (KeyError, TypeError, ValueError):
        return None


def _Mask(Depth, Intrinsics, Model, Joints, Reference):
    Camera = np.dstack((DepthGeometry.RayMap(Intrinsics) * (Depth / 10.0)[..., None], Depth / 10.0))
    World = DepthGeometry.CameraToDisplay(Camera.reshape(-1, 3), Model).reshape(*Depth.shape, 3)
    X, Y, Z = np.moveaxis(World, -1, 0)
    if Reference:
        # The upper rim, not the floor/liner or loose cables in the basket's interior.
        Keep = (np.abs(X) < 23) & (Y > -5) & (Y < 22) & (Z > -3) & (Z < 0.8)
        Keep &= (np.abs(X) > 18) | (Y < -2) | (Y > 20)
    else:
        # The search can follow a displaced camera. Only archived rim descriptors anchor the fit.
        Keep = (np.abs(X) < 28) & (Y > -10) & (Y < 27) & (Z > -8) & (Z < 6)
    Keep &= Depth > 0
    for Start, End in zip(Joints[:-1], Joints[1:]):
        Vector = End - Start
        Along = np.clip(np.sum((World - Start) * Vector, axis=-1) / max(float(Vector @ Vector), 1e-9), 0, 1)
        Keep &= np.sum((World - Start - Along[..., None] * Vector) ** 2, axis=-1) > ArmExclusionCm ** 2
    return cv2.erode(Keep.astype(np.uint8) * 255, np.ones((5, 5), np.uint8))


def _Features(Image, Depth, Intrinsics, Model, Joints, Reference):
    Mask = _Mask(Depth, Intrinsics, Model, Joints, Reference)
    Orb = cv2.ORB_create(nfeatures=2400, fastThreshold=10, edgeThreshold=10, patchSize=21)
    Keys, Descriptors = Orb.detectAndCompute(Image, Mask)
    if Descriptors is None:
        return None
    Pixels, Points, Kept, Cells = [], [], [], set()
    # ORB repeats a corner at several scales. Count a physical patch once.
    for Index in sorted(range(len(Keys)), key=lambda I: Keys[I].response, reverse=True):
        U, V = Keys[Index].pt
        Cell = (int(U // 4), int(V // 4))
        if Cell in Cells:
            continue
        X, Y = int(round(U)), int(round(V))
        if not 1 <= X < Depth.shape[1] - 1 or not 1 <= Y < Depth.shape[0] - 1:
            continue
        Patch = Depth[Y - 1:Y + 2, X - 1:X + 2]
        Valid = Patch[Patch > 0].astype(float)
        if len(Valid) < 7:
            continue
        Z = float(np.median(Valid)) / 10.0
        # Depth boundaries are not dependable RGB/depth correspondences.
        if abs(float(Depth[Y, X]) / 10.0 - Z) > 0.3 or np.percentile(Valid, 90) - np.percentile(Valid, 10) > 8:
            continue
        Points.append(DepthGeometry.Deproject(U, V, Z, Intrinsics).reshape(3))
        Pixels.append([U, V]); Kept.append(Index); Cells.add(Cell)
    if len(Points) < MinMatches:
        return None
    return {"points": np.asarray(Points), "pixels": np.asarray(Pixels), "descriptors": Descriptors[Kept]}


def _SameIntrinsics(A, B):
    try:
        return (int(A["width"]) == int(B["width"]) and int(A["height"]) == int(B["height"])
                and all(abs(float(A[K]) - float(B[K])) < 1e-4 for K in ("fx", "fy", "cx", "cy"))
                and np.array_equal(DepthGeometry.Distortion(A), DepthGeometry.Distortion(B)))
    except (KeyError, TypeError, ValueError):
        return False


@lru_cache(maxsize=2)
def _LoadReference(ModelJson, Keyframe, Stamp):
    del Stamp  # File identity is part of the cache key; never update from a live frame.
    Model = json.loads(ModelJson)
    Path_ = Path(Keyframe)
    try:
        Record = json.loads(Path_.read_text(encoding="utf-8"))
        Intrinsics = Record["depth_intrinsics"]
        ImageData = Record["images"][0]
        if not isinstance(ImageData, str) or not ImageData.startswith("data:image/") or len(ImageData) > 8_000_000:
            return None
        Image = cv2.imdecode(np.frombuffer(base64.b64decode(ImageData.split(",", 1)[1], validate=True), np.uint8), cv2.IMREAD_COLOR)
        if Record.get("mirrored"):
            Image = cv2.flip(Image, 1)
        DepthPath = (Path_.parent / Record["depth_frames_png"][0]).resolve()
        if DepthPath.parent != Path_.parent.resolve():
            return None
        Depth = cv2.imread(str(DepthPath), cv2.IMREAD_UNCHANGED)
        Joints = _Joints({"ok": True, "joints": Record["joints"]})
        if Image is None or Depth is None or Joints is None or Image.shape[:2] != Depth.shape or Depth.dtype != np.uint16:
            return None
        if list(Model.get("image_size", [])) != [int(Intrinsics["width"]), int(Intrinsics["height"])]:
            return None
        Features = _Features(Image, Depth, Intrinsics, Model, Joints, True)
        return {"features": Features, "intrinsics": Intrinsics, "keyframe": str(Path_)} if Features is not None else None
    except (OSError, KeyError, IndexError, ValueError, TypeError, cv2.error):
        return None


def _Reference(Model):
    if not isinstance(Model, dict) or Model.get("type") != "pinhole_depth_v1":
        return None
    Reference = Model.get("drift_reference")
    if not isinstance(Reference, dict) or Reference.get("method") != "desk_plane":
        return None
    try:
        # Local import avoids a cycle when CameraDrift calls this module.
        from CameraDrift import Checksum
        if Reference.get("sha256") != Checksum(Reference, Model):
            return None
        Keyframe = Path(Reference["keyframe"])
        Stat = Keyframe.stat()
        return _LoadReference(json.dumps(Model, sort_keys=True), str(Keyframe), (Stat.st_mtime_ns, Stat.st_size))
    except (KeyError, OSError, TypeError, ValueError):
        return None


def _Pairs(Reference, Current):
    Matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    def Good(A, B):
        return {Pair[0].queryIdx: Pair[0].trainIdx for Pair in Matcher.knnMatch(A, B, k=2)
                if len(Pair) == 2 and Pair[0].distance < DescriptorRatio * Pair[1].distance and Pair[0].distance <= MaxDescriptorDistance}
    Forward = Good(Reference["descriptors"], Current["descriptors"])
    Reverse = Good(Current["descriptors"], Reference["descriptors"])
    Pairs = [(A, B) for A, B in Forward.items() if Reverse.get(B) == A]
    if len(Pairs) < MinMatches:
        return None
    return Reference["points"][[A for A, _ in Pairs]], Current["points"][[B for _, B in Pairs]]


def _Rigid(A, B):
    CentreA, CentreB = A.mean(axis=0), B.mean(axis=0)
    U, _, Vt = np.linalg.svd((A - CentreA).T @ (B - CentreB))
    Rotation = Vt.T @ U.T
    if np.linalg.det(Rotation) < 0:
        Vt[-1] *= -1
        Rotation = Vt.T @ U.T
    return Rotation, CentreB - Rotation @ CentreA


def _Angle(Rotation):
    return math.degrees(math.acos(float(np.clip((np.trace(Rotation) - 1) / 2, -1, 1))))


def FitCorrespondences(A, B):
    """Full rigid six-DOF fit. Public for deterministic offline geometry tests."""
    A, B = np.asarray(A, dtype=float), np.asarray(B, dtype=float)
    if A.shape != B.shape or A.ndim != 2 or A.shape[1] != 3 or len(A) < MinMatches or not np.isfinite([A, B]).all():
        return None
    Rng = np.random.default_rng(7401)
    Best = np.zeros(len(A), dtype=bool)
    for _ in range(300):
        Choice = Rng.choice(len(A), 3, replace=False)
        if np.linalg.norm(np.cross(A[Choice[1]] - A[Choice[0]], A[Choice[2]] - A[Choice[0]])) < 2:
            continue
        Rotation, Translation = _Rigid(A[Choice], B[Choice])
        Good = np.linalg.norm(A @ Rotation.T + Translation - B, axis=1) <= FitBandCm
        if Good.sum() > Best.sum():
            Best = Good
    if Best.sum() < max(MinInliers, MinInlierShare * len(A)):
        return None
    for _ in range(3):
        Rotation, Translation = _Rigid(A[Best], B[Best])
        Best = np.linalg.norm(A @ Rotation.T + Translation - B, axis=1) <= FitBandCm
    if Best.sum() < max(MinInliers, MinInlierShare * len(A)):
        return None
    Rotation, Translation = _Rigid(A[Best], B[Best])
    Residual = np.linalg.norm(A[Best] @ Rotation.T + Translation - B[Best], axis=1)
    Rms = float(np.sqrt(np.mean(Residual ** 2)))
    Spread = np.linalg.svd(A[Best] - A[Best].mean(axis=0), compute_uv=False) / math.sqrt(int(Best.sum()))
    Cells = len(np.unique(np.floor(A[Best] / 2).astype(int), axis=0))
    if Rms > MaxRmsCm or Spread[0] < MinLongSpreadCm or Spread[1] < MinSecondSpreadCm or Cells < MinSpatialCells:
        return None
    # Resampling independent image patches exposes weak axes and depth-fit uncertainty.
    Rotations, Translations = [], []
    Source, Target = A[Best], B[Best]
    for _ in range(60):
        Indices = Rng.choice(len(Source), len(Source), replace=True)
        R, T = _Rigid(Source[Indices], Target[Indices])
        Rotations.append(_Angle(R @ Rotation.T))
        Translations.append(float(np.linalg.norm(T - Translation)))
    RotationBound = float(np.percentile(Rotations, 95))
    TranslationBound = float(np.percentile(Translations, 95))
    Angle, Shift = _Angle(Rotation), float(np.linalg.norm(Translation))
    Passed = Angle + RotationBound <= MaxRotationDeg and Shift + TranslationBound <= MaxTranslationCm
    Moved = Angle - RotationBound > MaxRotationDeg or Shift - TranslationBound > MaxTranslationCm
    if not Passed and not Moved:
        return None
    return {"ok": bool(Passed), "alignment_source": "robot", "error_code": None if Passed else "camera_moved",
            "error": None if Passed else "The camera moved relative to the fixed robot rim. Lock the mount and recalibrate.",
            "reason": "fixed robot rim matches the calibrated camera" if Passed else "fixed robot rim proves camera movement",
            "robot_rotation_deg": round(Angle, 4), "robot_translation_cm": round(Shift, 4),
            "robot_rotation_bound_deg": round(RotationBound, 4), "robot_translation_bound_cm": round(TranslationBound, 4),
            "robot_rms_cm": round(Rms, 4), "robot_matches": len(A), "robot_inliers": int(Best.sum()),
            "robot_spatial_cells": Cells, "robot_spread_cm": Spread.round(4).tolist()}


def Check(ImageBgr, DepthMm, Intrinsics, Model, Arm):
    """Check one native synchronized RGB/depth capture; None means unavailable, never safe."""
    Started = time.perf_counter()
    Joints = _Joints(Arm)
    if ImageBgr is None or DepthMm is None or not Intrinsics or Joints is None:
        return None
    if ImageBgr.shape[:2] != DepthMm.shape or DepthMm.dtype != np.uint16:
        return None
    Reference = _Reference(Model)
    if Reference is None or not _SameIntrinsics(Reference["intrinsics"], Intrinsics):
        return None
    try:
        Current = _Features(ImageBgr, DepthMm, Intrinsics, Model, Joints, False)
        if Current is None:
            return None
        Pairs = _Pairs(Reference["features"], Current)
        Result = FitCorrespondences(*Pairs) if Pairs is not None else None
        if Result is not None:
            Result.update(robot_reference=Reference["keyframe"], check_ms=round((time.perf_counter() - Started) * 1000, 1))
        return Result
    except (ValueError, TypeError, KeyError, cv2.error, np.linalg.LinAlgError):
        return None
