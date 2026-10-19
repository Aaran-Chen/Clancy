"""Controller-ready gravity model for the SO-101 follower (offline; no hardware access).

g(q) is a sum of cos/sin terms of the cumulative joint angles, exact for the planar chain lift-elbow-wrist_flex:

    T2 = q_lift
    T3 = q_lift + q_elbow
    T4 = q_lift + q_elbow + q_wrist_flex            (URDF angles, radians)

    tau_lift  = A2 cos T2 + B2 sin T2 + A3 cos T3 + B3 sin T3 + A4 cos T4 + B4 sin T4
    tau_elbow =                         A3 cos T3 + B3 sin T3 + A4 cos T4 + B4 sin T4
    tau_wrist =                                                 A4 cos T4 + B4 sin T4
    tau_pan = 0 (vertical axis), tau_roll ~ 0 (<0.06 kg*cm), tau_gripper ~ 0 (weight of a gripped object is carried by the grasp)

tau is the torque (N*m) the actuator must apply along the joint's +z axis to hold the pose; positive = toward increasing raw ticks,
because for every joint of this arm  q_urdf = wrap(raw - ReferenceRaw) * 2*pi/4096  (verified against the bridge's arc/sign chain).
The constants below were fitted to the repo's SO101.urdf inertials (fit residual 1e-15 N*m).  A payload of m kg in the jaws adds
m/0.1 * DP100 (point mass at the tool point).  Uncalibrated, expect +/-1 kg*cm on the lift (mass, COM, cabling and camera
uncertainty); calibrate in duty units with fit_from_loads().
"""
from __future__ import annotations

import math
from typing import Dict, Iterable, Sequence

import numpy as np

REFERENCE_RAW = {"shoulder_pan": 3453, "shoulder_lift": 256, "elbow_flex": 1671, "wrist_flex": 605, "wrist_roll": 2588, "gripper": 700}
G = 9.80665
KGCM_PER_NM = 100.0 / G

# [A2, B2, A3, B3, A4, B4] in N*m, no payload (SO101.urdf, 485 g moving mass)
P0 = np.array([-0.085899, -0.402063, -0.335500, -0.012385, -0.117011, -0.002459])
# increment per 100 g held at the tool point (gripper_frame_link origin)
DP100 = np.array([-0.027458, -0.110394, -0.132292, -0.005099, -0.156149, 0.007748])

# duty-to-torque scale: full-duty (stall) torque of the servo in N*m = STALL_KGCM_PER_VOLT * supply volts / KGCM_PER_NM
# 12 V-class STS3215: 30 kg*cm at 12 V (datasheet); the arm's servos report ~8.9 V -> ~22 kg*cm.
STALL_KGCM_PER_VOLT = 30.0 / 12.0


def urdf_angle(joint: str, raw: float) -> float:
    d = (raw - REFERENCE_RAW[joint] + 2048) % 4096 - 2048
    return d * 2.0 * math.pi / 4096.0


def _basis(q2, q3, q4):
    T2, T3, T4 = q2, q2 + q3, q2 + q3 + q4
    return np.array([math.cos(T2), math.sin(T2), math.cos(T3), math.sin(T3), math.cos(T4), math.sin(T4)])


def params(payload_kg: float = 0.0) -> np.ndarray:
    return P0 + DP100 * (payload_kg / 0.1)


def gravity_torque_nm(raw: Dict[str, float], payload_kg: float = 0.0, P: np.ndarray | None = None) -> Dict[str, float]:
    """Holding torque (N*m, sign = toward increasing raw ticks) for lift, elbow and wrist_flex from raw servo ticks."""
    P = params(payload_kg) if P is None else P
    X = _basis(urdf_angle("shoulder_lift", raw["shoulder_lift"]), urdf_angle("elbow_flex", raw["elbow_flex"]),
               urdf_angle("wrist_flex", raw["wrist_flex"]))
    return {"shoulder_lift": float(X @ P), "elbow_flex": float(X[2:] @ P[2:]), "wrist_flex": float(X[4:] @ P[4:])}


def gravity_duty_permille(raw: Dict[str, float], supply_volts: float = 8.9, payload_kg: float = 0.0, gain: float = 1.0,
                          P: np.ndarray | None = None) -> Dict[str, int]:
    """PWM duty (per mille, signed, same units as Goal_Time in mode 2 / Present_Load) that balances gravity at this pose."""
    stall_nm = STALL_KGCM_PER_VOLT * supply_volts / KGCM_PER_NM
    t = gravity_torque_nm(raw, payload_kg, P)
    return {j: int(round(max(-1000.0, min(1000.0, gain * v / stall_nm * 1000.0)))) for j, v in t.items()}


def fit_from_loads(samples: Iterable[Dict], prior_rel_sigma: float = 0.25, noise_permille: float = 20.0,
                   prior: np.ndarray | None = None, supply_volts: float = 8.9):
    """Fit g(q) in duty units from hold/sweep loads.  Each sample: {"raw": {lift, elbow_flex, wrist_flex}, "load": {lift, elbow_flex, wrist_flex}}
    with load = (up-approach + down-approach)/2 of Present_Load (per mille, sign bit 10 decoded).  Returns a dict of per-joint
    coefficient vectors (duty units) usable as  duty_j = X[cols_j] @ coef_j.  MAP regression toward the URDF prior (diagonal prior sd
    prior_rel_sigma*|p0|, floor 20 per mille) plus a free offset per joint."""
    prior = P0 if prior is None else prior
    stall_nm = STALL_KGCM_PER_VOLT * supply_volts / KGCM_PER_NM
    to_duty = 1000.0 / stall_nm
    rows, ys = [], {"shoulder_lift": [], "elbow_flex": [], "wrist_flex": []}
    for s in samples:
        rows.append(_basis(urdf_angle("shoulder_lift", s["raw"]["shoulder_lift"]), urdf_angle("elbow_flex", s["raw"]["elbow_flex"]),
                           urdf_angle("wrist_flex", s["raw"]["wrist_flex"])))
        for j in ys:
            ys[j].append(s["load"][j])
    X = np.array(rows)
    out = {}
    for j, cols in (("shoulder_lift", slice(0, 6)), ("elbow_flex", slice(2, 6)), ("wrist_flex", slice(4, 6))):
        p0 = prior[cols] * to_duty
        sd = np.maximum(prior_rel_sigma * np.abs(p0), 20.0)
        A = np.hstack([X[:, cols], np.ones((len(X), 1))])
        P0v = np.concatenate([p0, [0.0]]); SD = np.concatenate([sd, [30.0]])
        H = A.T @ A / noise_permille ** 2 + np.diag(1.0 / SD ** 2)
        g = A.T @ np.array(ys[j]) / noise_permille ** 2 + P0v / SD ** 2
        out[j] = np.linalg.solve(H, g)           # last element = constant offset (drop it when commanding)
    return out


def predict_duty(coefs, raw: Dict[str, float]) -> Dict[str, float]:
    X = _basis(urdf_angle("shoulder_lift", raw["shoulder_lift"]), urdf_angle("elbow_flex", raw["elbow_flex"]), urdf_angle("wrist_flex", raw["wrist_flex"]))
    return {"shoulder_lift": float(X[0:6] @ coefs["shoulder_lift"][:-1]), "elbow_flex": float(X[2:6] @ coefs["elbow_flex"][:-1]),
            "wrist_flex": float(X[4:6] @ coefs["wrist_flex"][:-1])}


def decode_present_load(raw16: int) -> int:
    """Present_Load (reg 60, 2 bytes): bits 0-9 magnitude (per mille of full duty), bit 10 = direction (LeRobot encodings table, Feetech SDK)."""
    mag = raw16 & 0x3FF
    return -mag if raw16 & 0x400 else mag


def encode_pwm(duty_permille: int) -> int:
    """Goal_Time (reg 44) in Operating_Mode 2: magnitude 0..1000, bit 10 = direction (Waveshare ST3215 memory map, reg 0x2C)."""
    d = int(max(-1000, min(1000, duty_permille)))
    return (-d | 0x400) if d < 0 else d


if __name__ == "__main__":
    # ---- self-test against the exact URDF forward kinematics, from raw ticks ----
    import sys
    sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))
    from gravity_model import default_model, RawMap, JOINTS
    rm = RawMap()
    rng = np.random.default_rng(5)
    worst = {0.0: 0.0, 0.05: 0.0, 0.1: 0.0}
    for pay in worst:
        m = default_model(pay)
        for _ in range(500):
            raw = {}
            for n in JOINTS:
                lo, hi, inv, span = rm.arc(n)
                k = int(rng.integers(0, span + 1))
                raw[n] = (lo - k) % 4096 if inv else (lo + k) % 4096
            q = rm.urdf_pose_from_raw(raw)
            ex = m.hold_torques(q)
            t = gravity_torque_nm(raw, pay)
            for j in t:
                worst[pay] = max(worst[pay], abs(t[j] - ex[j]))
    for pay, w in worst.items():
        print(f"payload {pay*1000:3.0f} g: max |closed form from raw ticks - exact FK| = {w*KGCM_PER_NM:.3f} kg*cm (includes random roll/gripper)")
    assert max(worst.values()) * KGCM_PER_NM < 0.12
    print("Home duty at 8.9 V (per mille):", gravity_duty_permille(rm.home_raw))
    print("encode/decode round trip:", [decode_present_load(encode_pwm(d)) == d for d in (-1000, -50, 0, 50, 580)])
    print("self-test OK")
