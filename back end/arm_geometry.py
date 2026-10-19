"""Arm geometry loader and IK solver for the SO-101 robotic arm."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import CadKinematics as Cad

DEFAULT_ARM_GEOMETRY = Path(__file__).with_name("arm_geometry.json")

# Joints solved by the IK system
IK_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex")

IK_DEFAULT_LIMITS: dict[str, tuple[float, float]] = {
    "shoulder_pan":  (-180.0, 180.0),
    "shoulder_lift": (-180.0, 180.0),
    "elbow_flex":    (-180.0, 180.0),
    "wrist_flex":    (-180.0, 180.0),
}

# Weight of wrist-pitch error against tip-position error in the IK cost.
# At 0.10 a 90-degree pitch error costs only ~9 "cm", so for hard targets the
# solver would fold the wrist backward/up to squeeze the tip onto the XYZ —
# mathematically optimal, useless for grasping (observed as "wrist maxes out
# and points up" at the end of a descend). 0.50 makes big orientation
# sacrifices uneconomical: the solver keeps the pitch and reports the position
# error instead, which the task loop can see and correct.
WRIST_PITCH_SCALE_CM_PER_DEG = 0.50


# ---------------------------------------------------------------------------
# Geometry loading
# ---------------------------------------------------------------------------

def load_arm_geometry(path: Path = DEFAULT_ARM_GEOMETRY) -> dict[str, Any]:
    if not path.is_file():
        return {"status": "missing", "segments": [], "units": "cm"}
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return {"status": "invalid", "segments": [], "units": "cm", "error": str(exc)}
    if isinstance(data, dict) and data.get("model") == "SO101Cad":
        data["CadAlignment"] = json.loads(Path(__file__).with_name("CadAlignment.json").read_text(encoding="utf-8"))
    return data if isinstance(data, dict) else {"status": "invalid", "segments": [], "units": "cm"}


def arm_geometry_summary(geometry: dict[str, Any]) -> list[str]:
    if geometry.get("model") == "SO101Cad":
        return ["SO-101 CAD geometry", "Physical alignment " + ("recorded" if geometry["CadAlignment"]["Verified"] else "needed")]
    units    = str(geometry.get("units", "cm"))
    segments = geometry.get("segments", [])
    if not isinstance(segments, list) or not segments:
        return ["Arm model: missing"]

    lines = [f"Arm model: {len(segments)} segments, {units}"]
    short = {
        "shoulder_pan_to_shoulder_lift": "pan → lift",
        "shoulder_lift_to_elbow":        "lift → elbow",
        "elbow_to_wrist_flex":           "elbow → wrist",
        "wrist_flex_to_wrist_roll":      "wrist → roll",
        "wrist_roll_to_gripper_tip":     "roll → tip",
    }
    for seg in segments[:5]:
        if not isinstance(seg, dict):
            continue
        name  = str(seg.get("name", "segment"))
        label = short.get(name, name.replace("_", " "))
        length = seg.get("length_cm")
        if length is not None:
            angle = seg.get("angle_degrees_up")
            lines.append(f"{label}: {length} {units}" + (f" @ {angle}°" if angle is not None else ""))
        else:
            offset = seg.get("offset_cm", {})
            if isinstance(offset, dict):
                lines.append(f"{label}: R {offset.get('right')} U {offset.get('up')}")
    return lines


# ---------------------------------------------------------------------------
# Internal geometry helpers
# ---------------------------------------------------------------------------

def _get_segment(geometry: dict[str, Any], name: str) -> dict[str, Any]:
    for seg in geometry.get("segments", []):
        if isinstance(seg, dict) and seg.get("name") == name:
            return seg
    return {}


def _float(d: dict[str, Any], key: str, default: float = 0.0) -> float:
    try:
        return float(d.get(key, default))
    except (TypeError, ValueError):
        return default


def _segment_vector(seg: dict[str, Any]) -> tuple[float, float]:
    length = _float(seg, "length_cm")
    if length:
        rad = math.radians(_float(seg, "angle_degrees_up"))
        return length * math.cos(rad), length * math.sin(rad)
    offset = seg.get("offset_cm", {})
    if isinstance(offset, dict):
        return _float(offset, "right"), _float(offset, "up")
    return 0.0, 0.0


def _normalize_deg(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


# ---------------------------------------------------------------------------
# IK dimensions — derived from the geometry file
# ---------------------------------------------------------------------------

def arm_ik_dimensions(geometry: dict[str, Any]) -> dict[str, float]:
    if geometry.get("model") == "SO101Cad":
        return Cad.Dimensions(geometry)
    s_pan_lift   = _get_segment(geometry, "shoulder_pan_to_shoulder_lift")
    s_lift_elbow = _get_segment(geometry, "shoulder_lift_to_elbow")
    s_elbow_wrist = _get_segment(geometry, "elbow_to_wrist_flex")
    s_wrist_roll  = _get_segment(geometry, "wrist_flex_to_wrist_roll")
    s_roll_tip    = _get_segment(geometry, "wrist_roll_to_gripper_tip")

    base_r, base_z = _segment_vector(s_pan_lift)

    upper_r, upper_z = _segment_vector(s_lift_elbow)
    upper_len = math.hypot(upper_r, upper_z)
    upper_zero = math.degrees(math.atan2(upper_z, upper_r)) if upper_len else 0.0

    forearm_len   = _float(s_elbow_wrist, "length_cm")
    wrist_len     = _float(s_wrist_roll,  "length_cm")
    tip_len       = _float(s_roll_tip,    "length_cm")
    forearm_zero  = _float(s_elbow_wrist, "angle_degrees_up")
    wrist_zero    = _float(s_wrist_roll,  "angle_degrees_up")

    lower_r = (forearm_len * math.cos(math.radians(forearm_zero))
               + (wrist_len + tip_len) * math.cos(math.radians(wrist_zero)))
    lower_z = (forearm_len * math.sin(math.radians(forearm_zero))
               + (wrist_len + tip_len) * math.sin(math.radians(wrist_zero)))
    lower_eff = math.hypot(lower_r, lower_z)
    lower_zero = math.degrees(math.atan2(lower_z, lower_r)) if lower_eff else 0.0

    return {
        "base_radial_cm":        base_r,
        "base_z_cm":             base_z,
        "upper_len_cm":          upper_len,
        "upper_zero_deg":        upper_zero,
        "forearm_len_cm":        forearm_len,
        "wrist_len_cm":          wrist_len,
        "tip_len_cm":            tip_len,
        "forearm_zero_deg":      forearm_zero,
        "wrist_zero_deg":        wrist_zero,
        "lower_len_cm":          forearm_len + wrist_len + tip_len,
        "lower_effective_len_cm": lower_eff,
        "lower_zero_deg":        lower_zero,
        "max_reach_cm":          base_r + upper_len + forearm_len + wrist_len + tip_len,
        "max_z_cm":              base_z + upper_len + forearm_len + wrist_len + tip_len,
    }


# ---------------------------------------------------------------------------
# Forward kinematics
# ---------------------------------------------------------------------------

def forward_joint_positions(geometry: dict[str, Any],
                            joint_angles_deg: dict[str, float]
                            ) -> list[dict[str, float]]:
    """Return base, shoulder, elbow, wrist, roll, and tip positions."""
    if geometry.get("model") == "SO101Cad":
        return Cad.Positions(geometry, joint_angles_deg)
    dims = arm_ik_dimensions(geometry)
    pan  = float(joint_angles_deg.get("shoulder_pan",  0.0))
    sh   = float(joint_angles_deg.get("shoulder_lift", 0.0))
    el   = float(joint_angles_deg.get("elbow_flex",    0.0))
    wr   = float(joint_angles_deg.get("wrist_flex",    0.0))

    t1 = math.radians(dims["upper_zero_deg"]   + sh)
    t2 = math.radians(dims["forearm_zero_deg"] + sh + el)
    t3 = math.radians(dims["wrist_zero_deg"]   + sh + el + wr)

    fl = dims["forearm_len_cm"]
    wl = dims["wrist_len_cm"]
    tl = dims["tip_len_cm"]
    if fl <= 0 and wl <= 0 and tl <= 0:
        fl = dims["lower_len_cm"]

    pan_rad = math.radians(pan)

    def radial_point(radial: float, z: float) -> dict[str, float]:
        return {
            "x": radial * math.sin(pan_rad),
            "y": radial * math.cos(pan_rad),
            "z": z,
        }

    shoulder_r = dims["base_radial_cm"]
    shoulder_z = dims["base_z_cm"]
    elbow_r = shoulder_r + dims["upper_len_cm"] * math.cos(t1)
    elbow_z = shoulder_z + dims["upper_len_cm"] * math.sin(t1)
    wrist_r = elbow_r + fl * math.cos(t2)
    wrist_z = elbow_z + fl * math.sin(t2)
    roll_r = wrist_r + wl * math.cos(t3)
    roll_z = wrist_z + wl * math.sin(t3)
    tip_r = roll_r + tl * math.cos(t3)
    tip_z = roll_z + tl * math.sin(t3)

    return [
        {"x": 0.0, "y": 0.0, "z": 0.0},
        radial_point(shoulder_r, shoulder_z),
        radial_point(elbow_r, elbow_z),
        radial_point(wrist_r, wrist_z),
        radial_point(roll_r, roll_z),
        radial_point(tip_r, tip_z),
    ]


def forward_tip_position(geometry: dict[str, Any],
                          joint_angles_deg: dict[str, float]) -> dict[str, float]:
    return forward_joint_positions(geometry, joint_angles_deg)[-1]


def wrist_tip_pitch_degrees(geometry: dict[str, Any],
                            joint_angles_deg: dict[str, float]) -> float:
    """Return the absolute vertical-plane pitch of the wrist-to-tip segment."""
    if geometry.get("model") == "SO101Cad":
        return Cad.Pitch(geometry, joint_angles_deg)
    dims = arm_ik_dimensions(geometry)
    sh = float(joint_angles_deg.get("shoulder_lift", 0.0))
    el = float(joint_angles_deg.get("elbow_flex", 0.0))
    wr = float(joint_angles_deg.get("wrist_flex", 0.0))
    return _normalize_deg(dims["wrist_zero_deg"] + sh + el + wr)


# ---------------------------------------------------------------------------
# IK solver internals
# ---------------------------------------------------------------------------

def _point_error(a: dict[str, float], b: dict[str, float]) -> float:
    return math.sqrt((a["x"]-b["x"])**2 + (a["y"]-b["y"])**2 + (a["z"]-b["z"])**2)


def _angle_delta(a: float, b: float, *, wrapped: bool = False) -> float:
    d = float(a) - float(b)
    return _normalize_deg(d) if wrapped else d


def _build_limits(joint_limits: dict[str, tuple[float, float]] | None
                  ) -> dict[str, tuple[float, float]]:
    limits = dict(IK_DEFAULT_LIMITS)
    for name, bounds in (joint_limits or {}).items():
        if name not in limits:
            continue
        try:
            lo, hi = float(bounds[0]), float(bounds[1])
            limits[name] = (min(lo, hi), max(lo, hi))
        except (IndexError, TypeError, ValueError):
            pass
    return limits


def _clamp_joint(name: str, angle: float, limits: dict[str, tuple[float, float]]) -> float:
    lo, hi = limits[name]
    v = float(angle)
    k_lo = math.floor((lo - v) / 360.0) - 1
    k_hi = math.ceil( (hi - v) / 360.0) + 1
    candidates = [v + 360.0 * k for k in range(k_lo, k_hi + 1)]
    in_range   = [c for c in candidates if lo <= c <= hi]
    if in_range:
        return min(in_range, key=lambda c: abs(c - v))
    best = min(candidates, key=lambda c: min(abs(c - lo), abs(c - hi)))
    return max(lo, min(hi, best))


def _clamp_pose(pose: dict[str, float], limits: dict[str, tuple[float, float]]
                ) -> dict[str, float]:
    return {n: _clamp_joint(n, pose.get(n, 0.0), limits) for n in IK_JOINTS}


def _limit_mid(name: str, limits: dict[str, tuple[float, float]]) -> float:
    lo, hi = limits[name]
    return _clamp_joint(name, (lo + hi) / 2.0, limits)


def _limit_samples(name: str, limits: dict[str, tuple[float, float]]) -> list[float]:
    lo, hi = limits[name]
    seen: set[int] = set()
    result: list[float] = []
    for v in (lo, _limit_mid(name, limits), hi):
        c = _clamp_joint(name, v, limits)
        key = round(c * 10)
        if key not in seen:
            seen.add(key)
            result.append(c)
    return result


def _solve_3x3(mat: list[list[float]], rhs: list[float]) -> list[float] | None:
    rows = [list(row) + [float(rhs[i])] for i, row in enumerate(mat)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda r: abs(rows[r][col]))
        if abs(rows[pivot][col]) < 1e-9:
            return None
        rows[col], rows[pivot] = rows[pivot], rows[col]
        pv = rows[col][col]
        for k in range(col, 4):
            rows[col][k] /= pv
        for row in range(3):
            if row == col:
                continue
            f = rows[row][col]
            if abs(f) < 1e-12:
                continue
            for k in range(col, 4):
                rows[row][k] -= f * rows[col][k]
    return [rows[r][3] for r in range(3)]


def _solve_linear(mat: list[list[float]], rhs: list[float]) -> list[float] | None:
    n = len(rhs)
    if n <= 0 or len(mat) != n or any(len(row) != n for row in mat):
        return None
    rows = [list(mat[i]) + [float(rhs[i])] for i in range(n)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(rows[r][col]))
        if abs(rows[pivot][col]) < 1e-9:
            return None
        rows[col], rows[pivot] = rows[pivot], rows[col]
        pv = rows[col][col]
        for k in range(col, n + 1):
            rows[col][k] /= pv
        for row in range(n):
            if row == col:
                continue
            f = rows[row][col]
            if abs(f) < 1e-12:
                continue
            for k in range(col, n + 1):
                rows[row][k] -= f * rows[col][k]
    return [rows[r][n] for r in range(n)]


def _ik_cost(geometry: dict[str, Any], target: dict[str, float],
             pose: dict[str, float], preferred: dict[str, float] | None,
             pref_weight: float,
             wrist_pitch_deg: float | None = None) -> float:
    tip  = forward_tip_position(geometry, pose)
    cost = ((tip["x"]-target["x"])**2 + (tip["y"]-target["y"])**2
            + (tip["z"]-target["z"])**2)
    cost += _posture_bias_cost(geometry, target, pose)
    if wrist_pitch_deg is not None:
        d = _angle_delta(wrist_tip_pitch_degrees(geometry, pose),
                         wrist_pitch_deg, wrapped=True)
        scaled = d * WRIST_PITCH_SCALE_CM_PER_DEG
        cost += scaled * scaled
    if preferred and pref_weight > 0:
        for n in IK_JOINTS:
            d = _angle_delta(pose[n], preferred.get(n, 0.0),
                             wrapped=(n == "shoulder_pan"))
            cost += pref_weight * d * d
    return cost


def _posture_bias_cost(geometry: dict[str, Any], target: dict[str, float],
                       pose: dict[str, float]) -> float:
    joints = forward_joint_positions(geometry, pose)
    if len(joints) < 6:
        return 0.0

    shoulder, elbow, wrist, roll, tip = joints[1], joints[2], joints[3], joints[4], joints[5]
    target_r = math.hypot(target["x"], target["y"])
    cost = 0.0

    # Prefer not to reach past the target horizontally and then fold back.
    if target_r > max(1.0, arm_ik_dimensions(geometry)["base_radial_cm"] + 0.5):
        ux = target["x"] / target_r
        uy = target["y"] / target_r
        max_forward = target_r + 0.75
        for pt in (elbow, wrist, roll):
            along_target = pt["x"] * ux + pt["y"] * uy
            overshoot = max(0.0, along_target - max_forward)
            cost += 0.120 * overshoot * overshoot

    # Prefer an arched path: rise above the target, then descend into the tip.
    target_z = target["z"]
    clearance = min(4.0, max(1.5, target_r * 0.10))
    desired_peak_z = max(target_z, shoulder["z"]) + clearance
    peak_z = max(elbow["z"], wrist["z"], roll["z"])
    low_peak = max(0.0, desired_peak_z - peak_z)
    cost += 0.006 * low_peak * low_peak

    low_elbow = max(0.0, shoulder["z"] + 0.75 - elbow["z"])
    cost += 0.006 * low_elbow * low_elbow

    final_clearance = max(0.75, clearance * 0.45)
    low_final_approach = max(0.0, target_z + final_clearance - roll["z"])
    cost += 0.018 * low_final_approach * low_final_approach

    upward_finish = max(0.0, tip["z"] - roll["z"])
    cost += 0.010 * upward_finish * upward_finish

    return cost


def _elbow_branch(geometry: dict[str, Any], pose: dict[str, float]) -> str:
    """Up/down label of the elbow bend between the upper arm and the FOREARM.

    Must use the same link pair as the analytic seed (upper + forearm, wrist
    pitch handled separately); it previously used the folded forearm+wrist+tip
    link, which disagreed with the seed near straight elbows.
    """
    dims = arm_ik_dimensions(geometry)
    lower_zero = dims["forearm_zero_deg"] if dims["forearm_len_cm"] > 0.0 else dims.get("lower_zero_deg", dims["upper_zero_deg"])
    bend = _normalize_deg(
        lower_zero
        - dims["upper_zero_deg"]
        + float(pose.get("elbow_flex", 0.0))
    )
    if abs(bend) < 1e-6:
        return "straight"
    return "up" if bend > 0.0 else "down"


def _matches_elbow_branch(geometry: dict[str, Any],
                          pose: dict[str, float],
                          elbow: str) -> bool:
    branch = _elbow_branch(geometry, pose)
    return branch == "straight" or branch == elbow


def _dls_delta(geometry: dict[str, Any], target: dict[str, float],
               pose: dict[str, float],
               limits: dict[str, tuple[float, float]],
               wrist_pitch_deg: float | None = None) -> dict[str, float] | None:
    base_tip = forward_tip_position(geometry, pose)
    err = [target["x"]-base_tip["x"], target["y"]-base_tip["y"], target["z"]-base_tip["z"]]
    if wrist_pitch_deg is not None:
        pitch_err = _angle_delta(wrist_pitch_deg,
                                 wrist_tip_pitch_degrees(geometry, pose),
                                 wrapped=True)
        err.append(pitch_err * WRIST_PITCH_SCALE_CM_PER_DEG)
    if math.sqrt(sum(e*e for e in err)) < 1e-4:
        return {n: 0.0 for n in IK_JOINTS}

    step = 0.35
    cols: list[list[float]] = []
    for n in IK_JOINTS:
        hi_p = {**pose, n: _clamp_joint(n, pose[n] + step, limits)}
        lo_p = {**pose, n: _clamp_joint(n, pose[n] - step, limits)}
        denom = _angle_delta(hi_p[n], lo_p[n], wrapped=(n == "shoulder_pan"))
        if abs(denom) < 1e-9:
            cols.append([0.0] * len(err))
            continue
        hi_tip = forward_tip_position(geometry, hi_p)
        lo_tip = forward_tip_position(geometry, lo_p)
        col = [(hi_tip[ax]-lo_tip[ax]) / denom for ax in ("x", "y", "z")]
        if wrist_pitch_deg is not None:
            pitch_delta = _angle_delta(
                wrist_tip_pitch_degrees(geometry, hi_p),
                wrist_tip_pitch_degrees(geometry, lo_p),
                wrapped=True,
            )
            col.append((pitch_delta / denom) * WRIST_PITCH_SCALE_CM_PER_DEG)
        cols.append(col)

    damping = 0.18
    n_joints = len(IK_JOINTS)
    normal = [[0.0] * n_joints for _ in range(n_joints)]
    rhs = [0.0] * n_joints
    for row in range(n_joints):
        for col in range(n_joints):
            normal[row][col] = sum(cols[row][task] * cols[col][task]
                                   for task in range(len(err)))
        normal[row][row] += damping * damping
        rhs[row] = sum(cols[row][task] * err[task] for task in range(len(err)))

    solved = _solve_linear(normal, rhs)
    if solved is None:
        return None

    raw = {n: solved[i] for i, n in enumerate(IK_JOINTS)}
    maxstep = max(abs(v) for v in raw.values())
    if maxstep > 18.0:
        scale = 18.0 / maxstep
        raw = {n: v*scale for n, v in raw.items()}
    return raw


def _analytic_seed(geometry: dict[str, Any], target: dict[str, float],
                   elbow: str, limits: dict[str, tuple[float, float]],
                   initial: dict[str, float] | None,
                   wrist_pitch_deg: float | None = None) -> dict[str, float] | None:
    dims  = arm_ik_dimensions(geometry)
    upper = dims["upper_len_cm"]
    lower = dims.get("lower_effective_len_cm", dims["lower_len_cm"])
    if upper <= 0 or lower <= 0:
        return None

    horiz = math.hypot(target["x"], target["y"])
    if horiz <= 1e-6 and initial:
        pan = float(initial.get("shoulder_pan", 0.0))
    else:
        pan = math.degrees(math.atan2(target["x"], target["y"]))

    r = horiz - dims["base_radial_cm"]
    z = target["z"] - dims["base_z_cm"]
    fixed_wrist_pitch = (
        wrist_pitch_deg is not None
        and dims["forearm_len_cm"] > 0.0
        and dims["wrist_len_cm"] + dims["tip_len_cm"] > 0.0
    )
    if fixed_wrist_pitch:
        phi = math.radians(float(wrist_pitch_deg))
        distal = dims["wrist_len_cm"] + dims["tip_len_cm"]
        r -= distal * math.cos(phi)
        z -= distal * math.sin(phi)
        lower = dims["forearm_len_cm"]

    reach = math.hypot(r, z)
    min_r = abs(upper - lower) + 1e-6
    max_r = upper + lower - 1e-6
    if max_r <= min_r:
        return None
    if reach < 1e-6:
        r, z = min_r, 0.0
    elif reach < min_r or reach > max_r:
        s = max(min_r, min(max_r, reach)) / reach
        r, z = r*s, z*s

    cos_el = (r*r + z*z - upper*upper - lower*lower) / (2*upper*lower)
    cos_el = max(-1.0, min(1.0, cos_el))
    sign   = 1.0 if elbow == "up" else -1.0
    el_rad = math.atan2(sign * math.sqrt(max(0.0, 1 - cos_el*cos_el)), cos_el)
    sh_rad = math.atan2(z, r) - math.atan2(lower*math.sin(el_rad),
                                             upper + lower*math.cos(el_rad))

    lower_zero = dims["forearm_zero_deg"] if fixed_wrist_pitch else dims.get("lower_zero_deg", dims["upper_zero_deg"])
    lower_delta = _normalize_deg(lower_zero - dims["upper_zero_deg"])
    shoulder_lift = _normalize_deg(math.degrees(sh_rad) - dims["upper_zero_deg"])
    elbow_flex = _normalize_deg(math.degrees(el_rad) - lower_delta)
    wrist_flex = 0.0
    if fixed_wrist_pitch:
        wrist_flex = _normalize_deg(float(wrist_pitch_deg)
                                    - dims["wrist_zero_deg"]
                                    - shoulder_lift
                                    - elbow_flex)
    pose = {
        "shoulder_pan":  pan,
        "shoulder_lift": shoulder_lift,
        "elbow_flex":    elbow_flex,
        "wrist_flex":    wrist_flex,
    }
    return _clamp_pose(pose, limits)


def _deduplicate(seeds: list[dict[str, float]],
                 limits: dict[str, tuple[float, float]]) -> list[dict[str, float]]:
    seen: set[tuple[int, ...]] = set()
    result: list[dict[str, float]] = []
    for seed in seeds:
        p = _clamp_pose(seed, limits)
        key = tuple(round(p[n]*10) for n in IK_JOINTS)
        if key not in seen:
            seen.add(key)
            result.append(p)
    return result


def _build_seeds(geometry: dict[str, Any], target: dict[str, float],
                 elbow: str, limits: dict[str, tuple[float, float]],
                 initial: dict[str, float] | None,
                 preferred: dict[str, float] | None,
                 wrist_pitch_deg: float | None = None,
                 quick: bool = False) -> list[dict[str, float]]:
    horiz = math.hypot(target["x"], target["y"])
    if horiz <= 1e-6 and initial:
        target_pan = float(initial.get("shoulder_pan", 0.0))
    else:
        target_pan = math.degrees(math.atan2(target["x"], target["y"]))

    seeds: list[dict[str, float]] = []
    for src in (initial, preferred):
        if src:
            seeds.append({n: float(src.get(n, 0.0)) for n in IK_JOINTS})

    for mode in (elbow, "up" if elbow != "up" else "down"):
        s = _analytic_seed(geometry, target, mode, limits, initial, wrist_pitch_deg)
        if s is not None:
            seeds.append(s)

    dims = arm_ik_dimensions(geometry)
    planar = math.degrees(math.atan2(target["z"]-dims["base_z_cm"],
                                      horiz - dims["base_radial_cm"]))
    planar_lift = planar - dims["upper_zero_deg"]
    planar_wrist = 0.0
    if wrist_pitch_deg is not None:
        planar_wrist = _normalize_deg(float(wrist_pitch_deg)
                                      - dims["wrist_zero_deg"]
                                      - planar_lift)
    seeds.append({"shoulder_pan":  target_pan,
                  "shoulder_lift": planar_lift,
                  "elbow_flex":    0.0, "wrist_flex": planar_wrist})
    seeds.append({n: 0.0 for n in IK_JOINTS} | {"shoulder_pan": target_pan})
    seeds.append({n: _limit_mid(n, limits) for n in IK_JOINTS}
                 | {"shoulder_pan": target_pan})

    el_lo, el_hi = limits["elbow_flex"]
    wr_lo, wr_hi = limits["wrist_flex"]
    sh_mid = _limit_mid("shoulder_lift", limits)
    if elbow == "up":
        seeds.append({"shoulder_pan": target_pan, "shoulder_lift": sh_mid,
                       "elbow_flex": el_hi, "wrist_flex": wr_lo})
    else:
        seeds.append({"shoulder_pan": target_pan, "shoulder_lift": sh_mid,
                       "elbow_flex": el_lo, "wrist_flex": wr_hi})

    # The limit grid (27 seeds) is what makes a full sweep cost ~160 ms. Drag
    # previews that warm-start from the previous solution skip it (quick=True);
    # cold solves and Send keep it.
    if not quick:
        for sh in _limit_samples("shoulder_lift", limits):
            for el in _limit_samples("elbow_flex", limits):
                for wr in _limit_samples("wrist_flex", limits):
                    seeds.append({"shoulder_pan": target_pan, "shoulder_lift": sh,
                                   "elbow_flex": el, "wrist_flex": wr})

    return _deduplicate(seeds, limits)


def _refine_seed(geometry: dict[str, Any], target: dict[str, float],
                 seed: dict[str, float], limits: dict[str, tuple[float, float]],
                 preferred: dict[str, float] | None, pref_weight: float,
                 max_iter: int,
                 wrist_pitch_deg: float | None = None) -> dict[str, Any]:
    pose = _clamp_pose(seed, limits)
    cost = _ik_cost(geometry, target, pose, preferred, pref_weight, wrist_pitch_deg)
    step = 14.0

    for _ in range(max_iter):
        improved = False

        delta = _dls_delta(geometry, target, pose, limits, wrist_pitch_deg)
        if delta is not None:
            for scale in (1.0, 0.55, 0.25, 0.1):
                candidate = {n: _clamp_joint(n, pose[n]+delta[n]*scale, limits)
                             for n in IK_JOINTS}
                c_cost = _ik_cost(geometry, target, candidate, preferred, pref_weight, wrist_pitch_deg)
                if c_cost + 1e-9 < cost:
                    pose, cost, improved = candidate, c_cost, True
                    break

        for n in IK_JOINTS:
            for direction in (1.0, -1.0):
                candidate = {**pose, n: _clamp_joint(n, pose[n]+direction*step, limits)}
                c_cost = _ik_cost(geometry, target, candidate, preferred, pref_weight, wrist_pitch_deg)
                if c_cost + 1e-9 < cost:
                    pose, cost, improved = candidate, c_cost, True

        step = min(18.0, step*1.04) if improved else step*0.5
        if step < 0.035:
            break
        tip = forward_tip_position(geometry, pose)
        pitch_err = (
            abs(_angle_delta(wrist_tip_pitch_degrees(geometry, pose),
                             wrist_pitch_deg, wrapped=True))
            if wrist_pitch_deg is not None else 0.0
        )
        if _point_error(tip, target) < 0.025 and pitch_err < 0.25 and step < 0.2:
            break

    tip = forward_tip_position(geometry, pose)
    pitch_err = (
        abs(_angle_delta(wrist_tip_pitch_degrees(geometry, pose),
                         wrist_pitch_deg, wrapped=True))
        if wrist_pitch_deg is not None else 0.0
    )
    return {"joint_angles_degrees": pose, "solved_tip_cm": tip,
            "error_cm": _point_error(tip, target), "cost": cost,
            "wrist_pitch_error_deg": pitch_err}


# ---------------------------------------------------------------------------
# Public IK entry point
# ---------------------------------------------------------------------------

def solve_tip_ik(geometry: dict[str, Any], x_cm: float, y_cm: float, z_cm: float,
                 elbow: str = "down",
                 joint_limits: dict[str, tuple[float, float]] | None = None,
                 initial_angles: dict[str, float] | None = None,
                 preferred_angles: dict[str, float] | None = None,
                 wrist_pitch_deg: float | None = None,
                 wrist_pitch_tolerance_deg: float = 6.0,
                 allow_unreachable: bool = False,
                 tolerance_cm: float = 0.75,
                 max_iterations: int = 70,
                 quick: bool = False,
                 continuity_weight: float | None = None, PoseAllowed=None, Scan: bool = False) -> dict[str, Any]:
    if geometry.get("model") == "SO101Cad":
        # Scan: many targets in a grid (calibration sampling); see Cad.SolveCandidates(Quick).
        return Cad.Solve(geometry, x_cm, y_cm, z_cm, elbow=elbow,
                         joint_limits=joint_limits, initial_angles=initial_angles,
                         preferred_angles=preferred_angles, wrist_pitch_deg=wrist_pitch_deg,
                         tolerance_cm=tolerance_cm, max_iterations=max_iterations, PoseAllowed=PoseAllowed, Quick=Scan)
    dims  = arm_ik_dimensions(geometry)
    upper = dims["upper_len_cm"]
    lower = dims["lower_len_cm"]
    if upper <= 0 or lower <= 0:
        raise ValueError("Arm geometry needs nonzero upper and lower link lengths.")

    target  = {"x": float(x_cm), "y": float(y_cm), "z": float(z_cm)}
    limits  = _build_limits(joint_limits)
    preferred = _clamp_pose(preferred_angles, limits) if preferred_angles else None
    wrist_pitch = _normalize_deg(float(wrist_pitch_deg)) if wrist_pitch_deg is not None else None
    seeds   = _build_seeds(geometry, target, elbow, limits, initial_angles,
                           preferred_angles, wrist_pitch, quick)
    if not seeds:
        raise ValueError("No valid IK seed poses could be built.")

    best: dict[str, Any] | None = None
    best_matching: dict[str, Any] | None = None
    # continuity_weight lets a caller (drag previews) pull the numeric sweep
    # toward the previous solution so marginal targets do not hop between
    # distant poses from one step to the next.
    pref_weight = (continuity_weight if continuity_weight is not None else 2e-5) if preferred else 0.0

    # Fast path (closed form). With the wrist pitch fixed the pose is fully
    # determined by the target and the elbow branch, so the analytic solution is
    # the answer whenever it sits inside the joint limits. Both branches are
    # solved (~1 ms each) and chosen in this order:
    #   1. requested branch, exact (no limit hit)          -> honour the toggle
    #   2. other branch, exact                             -> continuity beats
    #      a limit-pinned approximation of the requested branch, which is what
    #      used to snap the pose back and forth as its error crossed tolerance
    #   3. neither exact: the reachable refined candidate closest to the
    #      previous pose (continuity), else the seed sweep below
    # A target beyond the arm's geometric reach (from the shoulder, distal link
    # included) can never be reached whatever the limits, so the best-effort
    # pose is returned as unreachable at once.
    if wrist_pitch is not None:
        other = "up" if elbow != "up" else "down"
        horiz = math.hypot(target["x"], target["y"])
        from_shoulder = math.hypot(horiz - dims["base_radial_cm"], target["z"] - dims["base_z_cm"])
        beyond_reach = from_shoulder > upper + dims["forearm_len_cm"] + dims["wrist_len_cm"] + dims["tip_len_cm"] + 0.5
        # Rule 0 (continuity): along a drag the previous solution, nudged onto
        # the new target, is the pose the user expects to see. When that nudge
        # lands within 0.75 cm it wins outright; a limit-clamped closed-form
        # seed can otherwise refine into a different wrist fold and make the
        # preview snap 100+ degrees for a half-centimetre move.
        if initial_angles:
            # Fixed 24-iteration budget regardless of the caller's cap: near the
            # elbow-limit fold the true solution swings 30-60 deg per cm of tip
            # travel and a short refine loses it, which showed as pose jumps.
            cont = _refine_seed(geometry, target, _clamp_pose(initial_angles, limits), limits,
                                preferred, pref_weight, max(24, min(max_iterations, 30)), wrist_pitch)
            cont_pitch_ok = float(cont["wrist_pitch_error_deg"]) <= wrist_pitch_tolerance_deg
            if cont_pitch_ok and float(cont["error_cm"]) <= min(tolerance_cm, 0.75):
                return {
                    "joint_angles_degrees": cont["joint_angles_degrees"],
                    "target_cm":     target,
                    "target_wrist_pitch_deg": wrist_pitch,
                    "solved_tip_cm": cont["solved_tip_cm"],
                    "error_cm":      cont["error_cm"],
                    "wrist_pitch_error_deg": cont["wrist_pitch_error_deg"],
                    "reachable":     True,
                    "elbow":         _elbow_branch(geometry, cont["joint_angles_degrees"]),
                    "fast_path":     True,
                    "path":          "continue",
                }
        candidates: list[dict[str, Any]] = []
        for branch in (elbow, other):
            seed = _analytic_seed(geometry, target, branch, limits, initial_angles, wrist_pitch)
            if seed is None:
                continue
            seed_tip = forward_tip_position(geometry, seed)
            seed_pitch_err = abs(_angle_delta(wrist_tip_pitch_degrees(geometry, seed), wrist_pitch, wrapped=True))
            exact = _point_error(seed_tip, target) <= 0.05 and seed_pitch_err <= 0.5
            fast = _refine_seed(geometry, target, seed, limits, preferred,
                                pref_weight, min(max_iterations, 12), wrist_pitch)
            fast_pose = fast["joint_angles_degrees"]
            pitch_ok = float(fast["wrist_pitch_error_deg"]) <= wrist_pitch_tolerance_deg
            reachable_c = float(fast["error_cm"]) <= tolerance_cm and pitch_ok
            drift = 0.0
            if initial_angles:
                drift = sum(abs(_angle_delta(fast_pose[n], float(initial_angles.get(n, fast_pose[n])),
                                             wrapped=(n == "shoulder_pan"))) for n in IK_JOINTS)
            candidates.append({"branch": branch, "exact": exact and _matches_elbow_branch(geometry, fast_pose, branch),
                               "reachable": reachable_c, "drift": drift, "result": fast})

        chosen: dict[str, Any] | None = None
        for cand in candidates:              # rule 1 then 2 (requested first)
            if cand["exact"]:
                chosen = cand
                break
        if chosen is None:                   # rule 3
            reachable_c = [cand for cand in candidates if cand["reachable"]]
            if reachable_c:
                chosen = min(reachable_c, key=lambda cand: float(cand["result"]["error_cm"]) + 0.004 * cand["drift"])
            elif beyond_reach and candidates:
                chosen = min(candidates, key=lambda cand: float(cand["result"]["error_cm"]))
        if chosen is not None:
            fast = chosen["result"]
            fast_reachable = bool(chosen["reachable"])
            if not fast_reachable and not allow_unreachable:
                raise ValueError(
                    f"Target outside IK reach: best error {fast['error_cm']:.2f} cm.")
            return {
                "joint_angles_degrees": fast["joint_angles_degrees"],
                "target_cm":     target,
                "target_wrist_pitch_deg": wrist_pitch,
                "solved_tip_cm": fast["solved_tip_cm"],
                "error_cm":      fast["error_cm"],
                "wrist_pitch_error_deg": fast["wrist_pitch_error_deg"],
                "reachable":     fast_reachable,
                "elbow":         _elbow_branch(geometry, fast["joint_angles_degrees"]),
                "fast_path":     True,
                "path":          ("exact-" + chosen["branch"]) if chosen["exact"] else ("best-" + chosen["branch"]),
            }

    for seed in seeds:
        result = _refine_seed(geometry, target, seed, limits, preferred,
                              pref_weight, max_iterations, wrist_pitch)
        if best is None or result["cost"] < best["cost"]:
            best = result
        if _matches_elbow_branch(geometry, result["joint_angles_degrees"], elbow):
            if best_matching is None or result["cost"] < best_matching["cost"]:
                best_matching = result
            pitch_ok = (
                wrist_pitch is None
                or float(best_matching["wrist_pitch_error_deg"]) <= min(
                    0.5, max(0.1, wrist_pitch_tolerance_deg * 0.1)
                )
            )
            if (float(best_matching["error_cm"]) <= min(0.05, max(0.01, tolerance_cm*0.1))
                    and pitch_ok):
                break

    if best_matching is not None:
        matching_pitch_reachable = (
            wrist_pitch is None
            or float(best_matching["wrist_pitch_error_deg"]) <= wrist_pitch_tolerance_deg
        )
        matching_reachable = (
            float(best_matching["error_cm"]) <= tolerance_cm
            and matching_pitch_reachable
        )
        best_pitch_reachable = (
            wrist_pitch is None
            or float(best["wrist_pitch_error_deg"]) <= wrist_pitch_tolerance_deg
        )
        best_reachable = (
            float(best["error_cm"]) <= tolerance_cm
            and best_pitch_reachable
        )
        matching_is_competitive = (
            float(best_matching["cost"]) <= float(best["cost"]) * 1.35 + 0.25
        )
        if matching_reachable or (not best_reachable and matching_is_competitive):
            best = best_matching
    if best is None:
        raise ValueError("IK failed to produce any candidate pose.")

    pitch_reachable = (
        wrist_pitch is None
        or float(best["wrist_pitch_error_deg"]) <= wrist_pitch_tolerance_deg
    )
    reachable = float(best["error_cm"]) <= tolerance_cm and pitch_reachable
    if not reachable and not allow_unreachable:
        if wrist_pitch is not None and not pitch_reachable:
            raise ValueError(
                f"Target wrist angle outside IK reach: tip error {best['error_cm']:.2f} cm, "
                f"wrist error {best['wrist_pitch_error_deg']:.1f} deg.")
        raise ValueError(
            f"Target outside IK reach: best error {best['error_cm']:.2f} cm.")

    return {
        "joint_angles_degrees": best["joint_angles_degrees"],
        "target_cm":     target,
        "target_wrist_pitch_deg": wrist_pitch,
        "solved_tip_cm": best["solved_tip_cm"],
        "error_cm":      best["error_cm"],
        "wrist_pitch_error_deg": best["wrist_pitch_error_deg"],
        "reachable":     reachable,
        "elbow":         _elbow_branch(geometry, best["joint_angles_degrees"]),
    }
