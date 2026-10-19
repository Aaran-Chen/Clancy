"""Immutable, short-lived depth scenes used by the motor route planner. No hardware IO.

Only /locate registers scenes. Commands name a snapshot, never supply obstacle geometry.
Camera registration changes and a newer scene revoke old approvals.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
import threading
import time
import uuid

import numpy as np

MaximumAgeSeconds = 180.0
MaximumObstacles = 2048
_lock = threading.RLock()
_scenes: dict[str, dict] = {}
_latest: str | None = None
_generation = 0
# Per scene id: (time.monotonic() at registration, the capture's age then). A scene's age after registration runs on
# the monotonic clock, so a wall-clock step (a time sync) neither expires a scene mid-route nor keeps an old one
# alive (2026-10-07 audit H1). Kept apart from the snapshot, which goes to the app as it is.
_clock: dict[str, tuple[float, float]] = {}


def CameraRevision(model: dict | None) -> str:
    if not model:
        raise ValueError("Camera alignment is unavailable; measure the scene again after alignment.")
    # A changed desk baseline does not change camera-to-arm registration.
    fields = {key: model.get(key) for key in (
        "type", "rvec", "tvec", "mirror_x", "image_size", "fitted_at", "fx", "fy", "cx", "cy", "dist")}
    return hashlib.sha256(json.dumps(fields, sort_keys=True, allow_nan=False).encode()).hexdigest()[:24]


def Bounds(item: dict) -> np.ndarray:
    try:
        value = np.asarray([item["min"], item["max"]], dtype=float)
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("Measured scene contains invalid obstacle bounds.") from exc
    if value.shape != (2, 3) or not np.isfinite(value).all() or np.any(value[1] <= value[0]) or np.max(np.abs(value)) > 300:
        raise ValueError("Measured scene contains invalid obstacle bounds.")
    return value


def Register(scene: dict, model: dict, capture_time: float | None = None, generation: int | None = None) -> dict:
    global _latest
    if scene.get("frame") != "display_cm":
        raise ValueError("Measured scene has an unknown coordinate frame.")
    obstacles = scene.get("obstacles")
    if not isinstance(obstacles, list) or not obstacles or len(obstacles) > MaximumObstacles:
        raise ValueError("Measured scene has no usable obstacle map.")
    for obstacle in obstacles:
        Bounds(obstacle)
    free_space = scene.get("free_space", [])
    if not isinstance(free_space, list):
        raise ValueError("Measured scene contains invalid observed space.")
    for volume in free_space:
        Bounds(volume)
    # Space known to hold nothing to avoid though the camera did not see through it (SceneGeometry.Build: the arm
    # where it stood, and with AssumeTaskReachFree the task object's reach).
    known_space = scene.get("known_space", [])
    if not isinstance(known_space, list):
        raise ValueError("Measured scene contains invalid known space.")
    for volume in known_space:
        Bounds(volume)
    if scene.get("task_object") is not None:
        Bounds(scene["task_object"])
    # An observed solid against the task object (SceneGeometry.TaskContact): the app refuses its grasp on it.
    contact = scene.get("task_contact")
    if contact is not None:
        Bounds(contact)
        if not isinstance(contact.get("points"), int) or contact["points"] <= 0:
            raise ValueError("Measured scene contains an invalid task contact.")
    now = time.time()
    captured = now if capture_time is None else float(capture_time)
    if not math.isfinite(captured) or now - captured > MaximumAgeSeconds or captured - now > 2:
        raise ValueError("Depth capture is stale; measure the scene again.")
    snapshot = copy.deepcopy(scene)
    snapshot["camera_revision"] = CameraRevision(model)
    geometry = {key: snapshot.get(key) for key in ("frame", "support", "obstacles", "free_space", "task_object", "camera_revision")}
    for key in ("known_space", "task_contact"):
        if key in snapshot:
            geometry[key] = snapshot[key]
    snapshot["revision"] = hashlib.sha256(json.dumps(geometry, sort_keys=True, allow_nan=False).encode()).hexdigest()[:24]
    snapshot["id"] = uuid.uuid4().hex
    snapshot["captured_at"] = captured
    with _lock:
        if generation is not None and generation != _generation:
            raise ValueError("A newer capture replaced this scene; use the latest measurement.")
        _scenes.clear()
        _clock.clear()
        _scenes[snapshot["id"]] = snapshot
        _clock[snapshot["id"]] = (time.monotonic(), max(0.0, now - captured))
        _latest = snapshot["id"]
    return copy.deepcopy(snapshot)


def _Age(scene_id: str) -> float:
    """Seconds since the scene's depth capture: its age when registered, plus monotonic time since."""
    registered, age = _clock[scene_id]
    return age + (time.monotonic() - registered)


def Check(scene_id: str | None, model: dict | None) -> None:
    """GetValid's checks without the copy: raises ValueError when the scene may not be used (cheap enough for the
    motor lock, which GetValid's deep copy of a 1400-box scene is not)."""
    with _lock:
        scene = _scenes.get(scene_id) if isinstance(scene_id, str) else None
        if scene is None or scene_id != _latest:
            raise ValueError("The measured scene changed or is missing; capture and approve a new plan.")
        if _Age(scene_id) > MaximumAgeSeconds:
            raise ValueError("The measured scene is stale; capture and approve a new plan.")
        if scene["camera_revision"] != CameraRevision(model):
            raise ValueError("Camera alignment changed; capture and approve a new plan.")


def GetValid(scene_id: str | None, model: dict | None) -> dict:
    with _lock:
        Check(scene_id, model)
        return copy.deepcopy(_scenes[scene_id])


def RemainingSeconds(scene_id: str | None) -> float | None:
    """How long a current scene stays usable (MaximumAgeSeconds minus its age), or None when it is not current."""
    with _lock:
        if not isinstance(scene_id, str) or scene_id not in _scenes or scene_id != _latest:
            return None
        return MaximumAgeSeconds - _Age(scene_id)


def Previous(scene_id: str | None, model: dict | None) -> dict:
    """Source geometry for a held/occluded target, before replacing its capture.

    The caller must remeasure obstacles and free space; this is never a freshness extension.
    """
    return GetValid(scene_id, model)


def Invalidate() -> int:
    global _latest, _generation
    with _lock:
        _scenes.clear()
        _clock.clear()
        _latest = None
        _generation += 1
        return _generation


def IsCurrent(scene_id: str) -> bool:
    with _lock:
        scene = _scenes.get(scene_id)
        return bool(scene and scene_id == _latest and _Age(scene_id) <= MaximumAgeSeconds)


def ModelBounds(item: dict, frame: dict) -> np.ndarray:
    """Display (X mirrored, arm midpoint origin) to the CAD model's arm origin."""
    bounds = Bounds(item).copy()
    base = frame.get("RobotFrame", {}).get("ArmBasePositionCm")
    if not base:
        raise ValueError("Arm workspace frame is missing.")
    bounds[:, 0] = -bounds[::-1, 0] - base["x"]
    bounds[:, 1] -= base["y"]
    bounds[:, 2] -= base["z"]
    return bounds


def Covered(bounds: np.ndarray, free: list[np.ndarray]) -> bool:
    """Exact coverage by a union of observed boxes; gaps never become free space."""
    missing = [bounds]
    for seen in free:
        remaining = []
        for part in missing:
            low, high = np.maximum(part[0], seen[0]), np.minimum(part[1], seen[1])
            if np.any(low >= high):
                remaining.append(part)
                continue
            core = part.copy()
            for axis in range(3):
                if core[0, axis] < low[axis]:
                    piece = core.copy()
                    piece[1, axis] = low[axis]
                    remaining.append(piece)
                    core[0, axis] = low[axis]
                if core[1, axis] > high[axis]:
                    piece = core.copy()
                    piece[0, axis] = high[axis]
                    remaining.append(piece)
                    core[1, axis] = high[axis]
        missing = remaining
        if not missing:
            return True
        if len(missing) > 512:
            return False
    return not missing
