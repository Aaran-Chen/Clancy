"""Camera-to-workspace calibration for the SO-101 bridge.

Maps camera pixels to arm coordinates on the table plane with ~mm accuracy.

How it works
------------
1. (Optional but recommended) `calibrate_intrinsics.py` writes
   `camera_intrinsics.json` — the camera matrix + lens distortion. With it,
   clicked pixels are undistorted before fitting, which matters near the
   image edges on a webcam like the C920.
2. The web app records PROBE POINTS: the user touches the gripper tip to a
   spot on the table, then clicks that tip in a frozen camera frame. Each
   probe pairs a pixel (u, v) with the tip's arm-frame position (x, y, z).
   The arm's own forward kinematics is the ground truth — no ruler needed.
3. `fit()` computes a homography from undistorted pixels to the arm-frame
   table plane (X, Y), and records the table height Z as the mean probe Z.
4. `pixel_to_plane(u, v)` then maps ANY pixel of something resting on the
   table to arm-frame centimeters.

All arm-frame coordinates here are in the same frame the web app's AI uses
(X positive = left, Y positive = farther, Z positive = up). The bridge does
not interpret them — it only does geometry between "pixel" and "that frame".

Automatic calibration
---------------------
`SelfCalibration.py` can replace steps 2-3 entirely: the arm shows a marker on
its gripper to the camera and a full pinhole camera model is fitted. When a
`camera_model` is saved here it takes precedence over the homography in
`pixel_to_plane`, and no probe points are needed.

Every automatic calibration that computes a camera is saved and applied, its
quality problems (`quality_problems`) shown to the user as warnings: only the
user rejects a calibration (`reject_camera_model`, the app's Reject calibration),
which writes back the file the calibration replaced. `save_camera_model` archives
that file first, in the calibration run's records (CalibrationReplaced), and the
new file names it under "replaced"; a rejection is archived there too
(UserRejected), so no calibration is ever lost.

After the cart is rolled against the desk, /locate may replace the model's desk
baseline with one measured on a capture the gripper tape verified
(`save_remeasured_desk`, see CameraDrift); the camera fit itself is only ever
written by a calibration.
"""

from __future__ import annotations

import json
import os
import threading
import time
import uuid
from pathlib import Path
from typing import Any

import cv2
import numpy as np

import CalibrationArchive
import CameraDrift
import RobotReference
import SelfCalibration
import SessionLog

BACKEND_ROOT = Path(__file__).resolve().parent
CALIBRATION_PATH = BACKEND_ROOT / "camera_calibration.json"
INTRINSICS_PATH = BACKEND_ROOT / "camera_intrinsics.json"

_LOCK = threading.Lock()

MIN_PROBE_POINTS = 4
RECOMMENDED_PROBE_POINTS = 6


def _default_state() -> dict[str, Any]:
    return {
        "points": [],
        "homography": None,
        "table_z": None,
        "rms_cm": None,
        "max_err_cm": None,
        "image_size": None,
        "used_intrinsics": False,
        "fitted_at": None,
        "mode": None,
        "camera_model": None,
        # Probe clicks arrive in the mirrored capture space the web app uses;
        # None marks probes saved before this was recorded.
        "input_mirrored": None,
        # An automatic calibration: the archived file it replaced, which Reject calibration restores
        # ({"archive", "fitted_at", "mode", "calibrated"}), and the calibration run it came from.
        "replaced": None,
        "run_directory": None,
        # A calibration a rejection restored: what the user rejected, when and why.
        "last_rejected": None,
    }


def _load_state() -> dict[str, Any]:
    try:
        if CALIBRATION_PATH.exists():
            data = json.loads(CALIBRATION_PATH.read_text(encoding="utf-8"))
            if isinstance(data, dict):
                state = _default_state()
                state.update(data)
                return state
    except (OSError, json.JSONDecodeError):
        pass
    return _default_state()


def _save_state(state: dict[str, Any]) -> None:
    """Write the calibration whole or not at all: a temporary file beside it, flushed, then renamed
    over it, so a crash mid-write (or the bridge reading it meanwhile) never sees half a file."""
    temporary = CALIBRATION_PATH.with_name(f"{CALIBRATION_PATH.name}.{uuid.uuid4().hex[:12]}.tmp")
    try:
        with temporary.open("x", encoding="utf-8") as output:
            output.write(json.dumps(state, indent=2))
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, CALIBRATION_PATH)
    finally:
        temporary.unlink(missing_ok=True)


def _load_intrinsics() -> dict[str, Any] | None:
    try:
        if INTRINSICS_PATH.exists():
            data = json.loads(INTRINSICS_PATH.read_text(encoding="utf-8"))
            if (
                isinstance(data, dict)
                and isinstance(data.get("camera_matrix"), list)
                and isinstance(data.get("dist_coeffs"), list)
            ):
                return data
    except (OSError, json.JSONDecodeError):
        pass
    return None


def _scaled_camera_matrix(
    intrinsics: dict[str, Any], image_size: tuple[float, float] | None
) -> np.ndarray:
    """Scale the calibrated camera matrix to the capture resolution in use.

    Intrinsics are resolution-specific; if the web app captures at a different
    resolution than the calibration ran at (same aspect), fx/fy/cx/cy scale
    linearly with the frame size.
    """
    matrix = np.array(intrinsics["camera_matrix"], dtype=np.float64)
    calibrated_size = intrinsics.get("image_size")
    if image_size and isinstance(calibrated_size, list) and len(calibrated_size) == 2:
        sx = image_size[0] / float(calibrated_size[0])
        sy = image_size[1] / float(calibrated_size[1])
        scaled = matrix.copy()
        scaled[0, 0] *= sx
        scaled[0, 2] *= sx
        scaled[1, 1] *= sy
        scaled[1, 2] *= sy
        return scaled
    return matrix


def _undistort(
    pixels: np.ndarray, image_size: tuple[float, float] | None, mirrored: bool = False
) -> tuple[np.ndarray, bool]:
    """Undistort Nx2 pixel coords using saved intrinsics.

    Returns (pixels, used_intrinsics). Output stays in pixel units (P=K), so
    the homography fit/query space is consistent whether or not intrinsics
    exist — only lens distortion is removed. calibrate_intrinsics.py measures
    un-flipped frames, so mirrored capture pixels are flipped into that space
    for the lens model and flipped back afterwards.
    """
    intrinsics = _load_intrinsics()
    if intrinsics is None:
        return pixels, False
    matrix = _scaled_camera_matrix(intrinsics, image_size)
    dist = np.array(intrinsics["dist_coeffs"], dtype=np.float64)
    shaped = pixels.reshape(-1, 1, 2).astype(np.float64).copy()
    width = float(image_size[0]) if image_size else None
    if mirrored and width:
        shaped[..., 0] = width - shaped[..., 0]
    undistorted = cv2.undistortPoints(shaped, matrix, dist, P=matrix).reshape(-1, 2)
    if mirrored and width:
        undistorted[:, 0] = width - undistorted[:, 0]
    return undistorted, True


def status() -> dict[str, Any]:
    with _LOCK:
        state = _load_state()
    model = state.get("camera_model")
    return {
        "calibrated": state["homography"] is not None or model is not None,
        "mode": "auto" if model is not None else ("manual" if state["homography"] is not None else None),
        "reprojection_px": model.get("fit_rms_px") if model else None,
        "holdout_px": model.get("holdout_rms_px") if model else None,
        "plane_rms_cm": model.get("plane_rms_cm") if model else None,
        "observations_used": model.get("observations_used") if model else None,
        "points": state["points"],
        "point_count": len(state["points"]),
        "min_points": MIN_PROBE_POINTS,
        "recommended_points": RECOMMENDED_PROBE_POINTS,
        "rms_cm": state["rms_cm"] if model is None else model.get("plane_rms_cm"),
        "max_err_cm": state["max_err_cm"],
        "table_z": state["table_z"] if model is None else model.get("table_z"),
        "image_size": state["image_size"],
        "has_intrinsics": _load_intrinsics() is not None,
        "used_intrinsics": state["used_intrinsics"],
        "fitted_at": state["fitted_at"],
        # The automatic calibration's quality warnings (what used to reject it), whether it carries the
        # drift baseline the camera check needs, and whether the user can reject it (reject_camera_model).
        "quality_problems": list(model.get("quality_problems") or []) if model else [],
        "drift_baseline": isinstance(model.get("drift_reference"), dict) if model else None,
        "observation_source": model.get("observation_source", "auto") if model else None,
        "can_reject": bool(model) and _replaced_archive(state) is not None,
        "replaced_fitted_at": (state.get("replaced") or {}).get("fitted_at") if isinstance(state.get("replaced"), dict) else None,
        "last_rejected": state.get("last_rejected"),
        # With no calibrated desk in view the gripper tape checks the camera (RobotReference).
        **RobotReference.Status(model),
    }


def add_probe_point(
    u: float, v: float, x: float, y: float, z: float, image_size: list[float] | None,
    mirrored: bool | None = None,
) -> dict[str, Any]:
    with _LOCK:
        state = _load_state()
        if mirrored is not None and state["points"] and state["input_mirrored"] not in (None, bool(mirrored)):
            return {"ok": False, "error": "Probe mirroring changed since earlier probes. Clear calibration points first."}
        if mirrored is not None:
            state["input_mirrored"] = bool(mirrored)
        if state["image_size"] and image_size and list(image_size) != list(state["image_size"]):
            return {
                "ok": False,
                "error": (
                    f"Probe image size {image_size} differs from earlier probes "
                    f"{state['image_size']}. Clear calibration points first."
                ),
            }
        state["points"].append({"u": u, "v": v, "x": x, "y": y, "z": z})
        if image_size:
            state["image_size"] = list(image_size)
        _save_state(state)
        return {"ok": True, "point_count": len(state["points"])}


def undo_probe_point() -> dict[str, Any]:
    with _LOCK:
        state = _load_state()
        if state["points"]:
            state["points"].pop()
            _save_state(state)
        return {"ok": True, "point_count": len(state["points"])}


def clear_probe_points() -> dict[str, Any]:
    with _LOCK:
        state = _default_state()
        _save_state(state)
        return {"ok": True, "point_count": 0}


def fit() -> dict[str, Any]:
    with _LOCK:
        state = _load_state()
        points = state["points"]
        if len(points) < MIN_PROBE_POINTS:
            return {
                "ok": False,
                "error": f"Need at least {MIN_PROBE_POINTS} probe points, have {len(points)}.",
            }

        image_size = tuple(state["image_size"]) if state["image_size"] else None
        pixels = np.array([[p["u"], p["v"]] for p in points], dtype=np.float64)
        plane = np.array([[p["x"], p["y"]] for p in points], dtype=np.float64)
        undistorted, used_intrinsics = _undistort(pixels, image_size, bool(state["input_mirrored"]))

        # Hand-clicked probe points have no gross outliers; plain least-squares
        # (method=0) uses every point and works from exactly 4 upward.
        homography, _ = cv2.findHomography(undistorted, plane, method=0)
        if homography is None:
            return {"ok": False, "error": "Homography fit failed — are the points collinear?"}

        projected = cv2.perspectiveTransform(
            undistorted.reshape(-1, 1, 2), homography
        ).reshape(-1, 2)
        errors = np.linalg.norm(projected - plane, axis=1)
        rms = float(np.sqrt(np.mean(errors**2)))
        max_err = float(np.max(errors))

        state["homography"] = homography.tolist()
        state["camera_model"] = None
        state["mode"] = "manual"
        state["table_z"] = float(np.mean([p["z"] for p in points]))
        state["rms_cm"] = rms
        state["max_err_cm"] = max_err
        state["used_intrinsics"] = used_intrinsics
        state["fitted_at"] = time.strftime("%Y-%m-%dT%H:%M:%S")
        _save_state(state)

        return {
            "ok": True,
            "rms_cm": rms,
            "max_err_cm": max_err,
            "per_point_err_cm": [float(e) for e in errors],
            "table_z": state["table_z"],
            "point_count": len(points),
            "used_intrinsics": used_intrinsics,
        }


def _read_file() -> dict[str, Any] | None:
    """camera_calibration.json as it is on disk, or None when there is none (or it cannot be read)."""
    try:
        data = json.loads(CALIBRATION_PATH.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return data if isinstance(data, dict) else None


def _record_change(run_directory: Any, kind: str, payload: dict[str, Any]) -> str:
    """Archive a change of the calibration file in the calibration run's records (CalibrationArchive.Record),
    or, without that run on disk, in captures/YYYY-MM-DD/calibration_changes/; returns the record's path."""
    if run_directory and Path(run_directory).is_dir():
        return CalibrationArchive.Record(run_directory, kind, payload)
    when = time.time()
    directory = SessionLog.CapturesRoot() / time.strftime("%Y-%m-%d", time.localtime(when)) / "calibration_changes"
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / f"{time.strftime('%H%M%S', time.localtime(when))}_{kind}_{uuid.uuid4().hex[:8]}.json"
    CalibrationArchive.WriteJson(path, {"kind": kind, "recorded_at": when, "payload": payload})
    return str(path)


def _replaced_archive(state: dict[str, Any]) -> str | None:
    """The archived file this state's calibration replaced, when it is on disk (what a rejection restores)."""
    replaced = state.get("replaced")
    archive = replaced.get("archive") if isinstance(replaced, dict) else None
    return archive if isinstance(archive, str) and Path(archive).is_file() else None


def save_camera_model(model: dict[str, Any], run_directory: Any = None) -> dict[str, Any]:
    """Store an automatic calibration; it replaces any manual homography.

    Every calibration that computed a camera is saved and applied here, whatever its quality problems
    (model["quality_problems"], shown to the user as warnings): only the user rejects one
    (reject_camera_model). The file it replaces is archived first, a CalibrationReplaced record in the
    calibration run's archive (run_directory; else captures/YYYY-MM-DD/calibration_changes/), and the
    new file names that record under "replaced": a rejection writes it back. When that record cannot be
    written nothing is saved (the file it replaces would be lost), and the error reaches the caller.
    Returns {"replaced_archive", "replaced_fitted_at", "can_reject"}.
    """
    with _LOCK:
        previous = _read_file()
        summary = {"fitted_at": (previous or {}).get("fitted_at"), "mode": (previous or {}).get("mode"),
                   "calibrated": bool(previous and (previous.get("camera_model") or previous.get("homography")))}
        payload = {"replaced_at": time.time(), "by_fitted_at": model.get("fitted_at"), **summary, "previous_calibration": previous}
        try:
            archive = _record_change(run_directory, "CalibrationReplaced", payload)
        except ValueError:
            # A value JSON cannot hold strictly (a NaN in the old file): keep its exact text instead.
            archive = _record_change(run_directory, "CalibrationReplaced",
                                     {**payload, "previous_calibration": None, "previous_file_text": CALIBRATION_PATH.read_text(encoding="utf-8")})
        state = _default_state()
        state["mode"] = "auto"
        state["camera_model"] = model
        state["table_z"] = model.get("table_z")
        state["rms_cm"] = model.get("plane_rms_cm")
        state["image_size"] = model.get("image_size")
        state["fitted_at"] = model.get("fitted_at")
        state["replaced"] = {"archive": archive, **summary}
        state["run_directory"] = str(run_directory) if run_directory else None
        _save_state(state)
        return {"replaced_archive": archive, "replaced_fitted_at": summary["fitted_at"], "can_reject": True}


def reject_camera_model(fitted_at: str | None = None, reason: str | None = None) -> dict[str, Any]:
    """The user's Reject calibration (/selfcal/reject): the calibration the current one replaced is used again.

    Nothing else ever rejects a calibration. fitted_at, when given, must be the current calibration's (the one
    the user was shown), else nothing changes. The rejected file is archived first (a UserRejected record in its
    run's archive, beside the CalibrationReplaced record it names), then the replaced file is written back whole,
    with last_rejected saying what was rejected, when and why. A restored automatic calibration keeps its own
    "replaced", so it can be rejected in turn. Returns what is in use now: {"ok", "rejected_fitted_at",
    "restored_fitted_at", "restored_mode", "calibrated", "drift_baseline", "archive"}, or {"ok": False, "error"}.
    """
    with _LOCK:
        state = _load_state()
        model = state.get("camera_model")
        if not isinstance(model, dict):
            return {"ok": False, "error": "There is no automatic calibration to reject; nothing was changed."}
        if fitted_at is not None and model.get("fitted_at") != fitted_at:
            return {"ok": False, "error": f"The calibration in use changed since it was shown (it was fitted at {model.get('fitted_at')}); "
                                          "nothing was rejected."}
        source = _replaced_archive(state)
        if source is None:
            return {"ok": False, "error": "This calibration names no archived calibration to restore (it was saved before "
                                          "calibrations could be rejected, or its archive is gone); nothing was rejected."}
        try:
            record = json.loads(Path(source).read_text(encoding="utf-8"))
            payload = record.get("payload", record)
            previous = payload.get("previous_calibration")
            if previous is None and isinstance(payload.get("previous_file_text"), str):
                previous = json.loads(payload["previous_file_text"])
        except (OSError, json.JSONDecodeError, AttributeError) as exc:
            return {"ok": False, "error": f"The calibration it replaced could not be read from {source} ({exc}); nothing was rejected."}
        restored = _default_state()
        if isinstance(previous, dict):
            restored.update(previous)
        when = time.time()
        try:
            archive = _record_change(state.get("run_directory"), "UserRejected", {
                "rejected_at": when, "reason": reason, "restored_from": source,
                "restored_fitted_at": restored.get("fitted_at"), "rejected_calibration": _read_file() or state})
            restored["last_rejected"] = {"fitted_at": model.get("fitted_at"), "reason": reason, "archive": archive,
                                         "rejected_at": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(when)),
                                         "run_directory": state.get("run_directory")}
            _save_state(restored)
        except (OSError, TypeError, ValueError) as exc:
            return {"ok": False, "error": f"The rejection could not be written ({exc}); the calibration in use is unchanged."}
    restored_model = restored.get("camera_model")
    calibrated = restored_model is not None or restored.get("homography") is not None
    return {"ok": True, "rejected_fitted_at": model.get("fitted_at"), "restored_fitted_at": restored.get("fitted_at"),
            "restored_mode": ("auto" if restored_model is not None else "manual") if calibrated else None, "calibrated": calibrated,
            "drift_baseline": isinstance((restored_model or {}).get("drift_reference"), dict), "archive": archive}


def camera_model() -> dict[str, Any] | None:
    """The saved automatic camera model, if any (pinhole_selfcal_v1 or pinhole_depth_v1)."""
    with _LOCK:
        return _load_state().get("camera_model")


def save_remeasured_desk(reference: dict[str, Any], replaced_sha256: str | None,
                         check: dict[str, Any] | None = None) -> tuple[bool, str]:
    """Save a desk re-measured after a cart move (CameraDrift.CheckCapture) as the camera model's
    drift baseline; (saved, why not).

    Only drift_reference changes: the camera fit and every other field are written back as read,
    and the new baseline's checksum must hold against the saved model, so a re-measure never lands
    on a calibration fitted since its capture was checked. replaced_sha256 names the baseline that
    capture was checked against; one replaced meanwhile (another capture re-measured it) is kept.
    The file as it was goes first to captures/YYYY-MM-DD/desk_remeasure/ (the calibration
    archive's atomic JSON, with check, the capture's camera_check), then it is rewritten atomically.
    """
    with _LOCK:
        state = _load_state()
        model = state.get("camera_model")
        if not isinstance(model, dict):
            return False, "the camera is no longer calibrated"
        if reference.get("sha256") != CameraDrift.Checksum(reference, model):
            return False, "the camera calibration changed since this capture was checked"
        current = model.get("drift_reference")
        if not isinstance(current, dict) or current.get("sha256") != replaced_sha256:
            return False, "the desk baseline changed since this capture was checked"
        when = time.time()
        directory = SessionLog.CapturesRoot() / time.strftime("%Y-%m-%d", time.localtime(when)) / "desk_remeasure"
        try:
            directory.mkdir(parents=True, exist_ok=True)
            CalibrationArchive.WriteJson(directory / f"{time.strftime('%H%M%S', time.localtime(when))}_{uuid.uuid4().hex[:8]}.json",
                                         {"kind": "DeskRemeasured", "saved_at": when, "camera_check": check,
                                          "previous_calibration": state, "drift_reference": reference})
            model["drift_reference"] = reference
            _save_state(state)
        except (OSError, TypeError, ValueError) as exc:
            return False, f"the calibration could not be written: {exc}"
        return True, ""


def depth_point_extrapolated(model: dict[str, Any] | None, x: float, y: float) -> dict[str, Any] | None:
    """Judge a depth-measured point (display frame) against where the calibration tips were.

    {"extrapolated", "outside_calibration_cm"}: more than SelfCalibration.TipRegionMarginCm
    outside the tips' hull is extrapolation. None when the model stores no tip region, and the
    caller keeps the table-plane pixel test.
    """
    outside = SelfCalibration.TipRegionDistance(model, x, y)
    if outside is None:
        return None
    # A non-finite point is extrapolated; JSON responses cannot carry infinity.
    return {"extrapolated": bool(outside > SelfCalibration.TipRegionMarginCm),
            "outside_calibration_cm": round(outside, 1) if np.isfinite(outside) else None}


def pixel_to_plane(u: float, v: float, mirrored: bool | None = None) -> dict[str, Any]:
    """Map one capture-resolution pixel to arm-frame cm on the table plane.

    mirrored says whether this capture was flipped left-right; when it differs
    from the probes' or the calibration's, the pixel is flipped to match.
    """
    with _LOCK:
        state = _load_state()
    if state.get("camera_model"):
        return SelfCalibration.PixelToPlane(state["camera_model"], u, v, mirrored)
    if state["homography"] is None:
        return {"ok": False, "error": "Camera is not calibrated yet."}

    if (mirrored is not None and state["input_mirrored"] is not None
            and bool(mirrored) != bool(state["input_mirrored"]) and state["image_size"]):
        u = float(state["image_size"][0]) - u
    pixels = np.array([[u, v]], dtype=np.float64)
    if state["used_intrinsics"]:
        image_size = tuple(state["image_size"]) if state["image_size"] else None
        pixels, _ = _undistort(pixels, image_size, bool(state["input_mirrored"]))

    homography = np.array(state["homography"], dtype=np.float64)
    projected = cv2.perspectiveTransform(pixels.reshape(-1, 1, 2), homography).reshape(-1, 2)

    # The homography is only validated inside the region the probe points cover.
    # Outside that region (plus a small margin) the mapping is extrapolation and
    # can be off by several cm, so flag it for downstream consumers.
    extrapolated = False
    points = state["points"]
    if points:
        us = [float(p["u"]) for p in points]
        vs = [float(p["v"]) for p in points]
        margin_u = max(40.0, (max(us) - min(us)) * 0.15)
        margin_v = max(40.0, (max(vs) - min(vs)) * 0.15)
        extrapolated = not (
            min(us) - margin_u <= u <= max(us) + margin_u
            and min(vs) - margin_v <= v <= max(vs) + margin_v
        )

    return {
        "ok": True,
        "x": float(projected[0][0]),
        "y": float(projected[0][1]),
        "z": float(state["table_z"]),
        "extrapolated": extrapolated,
    }
