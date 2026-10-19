"""Static gravity torque of the SO-101 arm from the repo's URDF (link masses and centres of mass).

Offline only: no bridge, no serial port. Output goes to stdout (and optional JSON).

Method: forward kinematics of the URDF chain, potential energy U(q) = sum m_k * g * z_k(q)
(z = base_link +z, vertical when the arm stands on the cart), gravity torque
tau_i = dU/dq_i by central differences. Torques are on the JOINT (output shaft), N*m.
"""
from __future__ import annotations

import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

URDF = Path(__file__).resolve().parents[2] / "AI Controlled WebApp" / "public" / "Cad" / "SO101.urdf"
G = 9.81
CHAIN = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]  # base -> gripper_link


def rpy_matrix(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def tf(xyz, rpy) -> np.ndarray:
    t = np.eye(4)
    t[:3, :3] = rpy_matrix(*rpy)
    t[:3, 3] = xyz
    return t


def rotz(q: float) -> np.ndarray:
    t = np.eye(4)
    c, s = math.cos(q), math.sin(q)
    t[:2, :2] = [[c, -s], [s, c]]
    return t


def floats(text: str) -> list[float]:
    return [float(v) for v in text.split()]


class Arm:
    def __init__(self, urdf: Path = URDF, extra: dict[str, tuple[float, tuple[float, float, float]]] | None = None):
        root = ET.parse(urdf).getroot()
        self.links: dict[str, dict] = {}
        for link in root.findall("link"):
            inertial = link.find("inertial")
            if inertial is None:
                continue
            o = inertial.find("origin")
            self.links[link.get("name")] = {
                "m": float(inertial.find("mass").get("value")),
                "com": floats(o.get("xyz")) if o is not None else [0, 0, 0],
            }
        self.joints: dict[str, dict] = {}
        for j in root.findall("joint"):
            o = j.find("origin")
            self.joints[j.get("name")] = {
                "parent": j.find("parent").get("link"), "child": j.find("child").get("link"),
                "xyz": floats(o.get("xyz")), "rpy": floats(o.get("rpy")),
                "limit": (float(j.find("limit").get("lower")), float(j.find("limit").get("upper")))
                if j.find("limit") is not None else (0, 0),
            }
        # moving jaw hangs off gripper_link through the "gripper" joint (it is a mass: keep it, at its zero angle)
        self.extra = extra or {}

    def com_positions(self, q: dict[str, float]) -> dict[str, np.ndarray]:
        """World (base_link) position of every link's centre of mass."""
        T = np.eye(4)
        out = {"base_link": T[:3, :3] @ np.array(self.links["base_link"]["com"]) + T[:3, 3]}
        for name in CHAIN:
            j = self.joints[name]
            T = T @ tf(j["xyz"], j["rpy"]) @ rotz(q.get(name, 0.0))
            child = j["child"]
            out[child] = (T @ np.append(self.links[child]["com"], 1.0))[:3]
            if child == "gripper_link":
                Tg = T @ tf(self.joints["gripper"]["xyz"], self.joints["gripper"]["rpy"])
                out["moving_jaw"] = (Tg @ np.append(self.links["moving_jaw_so101_v1_link"]["com"], 1.0))[:3]
                self._gripper_T = T
        for name, (mass, local) in self.extra.items():  # payloads fixed in the gripper_link frame
            out[name] = (self._gripper_T @ np.append(local, 1.0))[:3]
        return out

    def masses(self) -> dict[str, float]:
        m = {k: v["m"] for k, v in self.links.items() if k in ("base_link", "shoulder_link", "upper_arm_link", "lower_arm_link", "wrist_link", "gripper_link")}
        m["moving_jaw"] = self.links["moving_jaw_so101_v1_link"]["m"]
        for name, (mass, _) in self.extra.items():
            m[name] = mass
        return m

    def potential(self, q: dict[str, float]) -> float:
        pos = self.com_positions(q)
        m = self.masses()
        return sum(m[k] * G * pos[k][2] for k in pos if k in m and k != "base_link")

    def gravity_torque(self, q: dict[str, float]) -> dict[str, float]:
        tau = {}
        eps = 1e-5
        for name in CHAIN:
            qp, qm = dict(q), dict(q)
            qp[name] = q.get(name, 0.0) + eps
            qm[name] = q.get(name, 0.0) - eps
            tau[name] = (self.potential(qp) - self.potential(qm)) / (2 * eps)
        return tau


def main() -> None:
    payload_kg = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
    extra = {"payload": (payload_kg, (0.0, 0.0, -0.1))} if payload_kg > 0 else {}
    arm = Arm(extra=extra)
    m = arm.masses()
    moving = sum(v for k, v in m.items() if k != "base_link")
    print(f"payload {payload_kg:.3f} kg; moving mass {moving:.3f} kg (links: {', '.join(f'{k} {v:.3f}' for k, v in m.items())})")
    print("zero pose COM positions (m, base frame):")
    for k, v in arm.com_positions({}).items():
        print(f"  {k:16s} {np.round(v, 4)}")
    tau0 = arm.gravity_torque({})
    print("gravity torque at q = 0:", {k: round(v, 3) for k, v in tau0.items()})

    lim = {n: arm.joints[n]["limit"] for n in CHAIN}
    print("joint limits (rad):", {n: tuple(round(x, 3) for x in l) for n, l in lim.items()})

    # worst case over the joint box for the three loaded joints (grid), pan and roll fixed at 0
    names = ["shoulder_lift", "elbow_flex", "wrist_flex"]
    grids = [np.linspace(lim[n][0], lim[n][1], 41) for n in names]
    worst = {n: (0.0, None) for n in CHAIN}
    rows = []
    for a in grids[0]:
        for b in grids[1]:
            for c in grids[2]:
                q = {"shoulder_lift": a, "elbow_flex": b, "wrist_flex": c}
                tau = arm.gravity_torque(q)
                for n in CHAIN:
                    if abs(tau[n]) > abs(worst[n][0]):
                        worst[n] = (tau[n], q)
    print("\nworst-case |gravity torque| over the joint box (N*m and kg*cm):")
    for n in CHAIN:
        t, q = worst[n]
        qd = None if q is None else {k: round(math.degrees(v), 1) for k, v in q.items()}
        print(f"  {n:14s} {t:+.3f} N*m = {abs(t) * 10.197:.2f} kg*cm at {qd}")

    # a mostly-horizontal reach pose: lift forward, elbow bent 90 deg, wrist level
    out = {"payload_kg": payload_kg, "worst": {n: worst[n][0] for n in CHAIN}}
    json.dump(out, open(Path(__file__).with_name(f"GravityTorqueFromUrdf_{payload_kg:.2f}kg.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
