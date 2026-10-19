"""Gravity-load model of the SO-101 follower arm, built straight from the repo's URDF.

Scratch tool for the gravity-compensation feasibility study (offline only: it never touches the bridge,
a serial port or a camera).  Everything is SI inside; helpers convert to kg*cm.

Sources used (all inside this repo, found relative to this file):
  AI Controlled WebApp/public/Cad/SO101.urdf      link masses, centres of mass, joint origins/axes
  back end/CadAlignment.json                      raw servo ticks at the URDF zero pose + sign per joint
  back end/arm_geometry.json                      servo_signs
  back end/gui_limits.json, home_preset.json      slider arcs (software limits) and Home
  back end/bridge_server.py (_ik_angle_from_raw)  raw -> angle conversion, replicated here (pure arithmetic)
"""
from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

G = 9.80665  # m/s^2
KGCM_PER_NM = 100.0 / G  # 1 N*m = 10.197 kg*cm

# This folder is "back end/HandGuideSim" in the repo: the repo root is two levels up.
WORKTREE = Path(__file__).resolve().parents[2]
URDF_PATH = WORKTREE / "AI Controlled WebApp" / "public" / "Cad" / "SO101.urdf"
BACKEND = WORKTREE / "back end"

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")


def rpy_to_matrix(r: float, p: float, y: float) -> np.ndarray:
    """URDF rpy: R = Rz(y) Ry(p) Rx(r)."""
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def transform(xyz, rpy) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = rpy_to_matrix(*rpy)
    T[:3, 3] = xyz
    return T


def rotz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    T = np.eye(4)
    T[0, 0], T[0, 1], T[1, 0], T[1, 1] = c, -s, s, c
    return T


@dataclass
class Link:
    name: str
    mass: float
    com: np.ndarray  # in the link frame, metres
    inertia: np.ndarray | None = None


@dataclass
class Joint:
    name: str
    parent: str
    child: str
    origin: np.ndarray  # 4x4
    axis: np.ndarray
    kind: str
    lower: float | None = None
    upper: float | None = None


def load_urdf(path: Path = URDF_PATH) -> tuple[dict[str, Link], dict[str, Joint]]:
    root = ET.parse(path).getroot()
    links: dict[str, Link] = {}
    for el in root.findall("link"):
        inertial = el.find("inertial")
        if inertial is None:
            continue
        mass = float(inertial.find("mass").get("value"))
        com = np.array([float(v) for v in inertial.find("origin").get("xyz").split()])
        i = inertial.find("inertia")
        inertia = None
        if i is not None:
            ixx, ixy, ixz, iyy, iyz, izz = (float(i.get(k)) for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"))
            inertia = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
        links[el.get("name")] = Link(el.get("name"), mass, com, inertia)
    joints: dict[str, Joint] = {}
    for el in root.findall("joint"):
        o = el.find("origin")
        xyz = [float(v) for v in o.get("xyz").split()]
        rpy = [float(v) for v in o.get("rpy").split()]
        axis_el = el.find("axis")
        axis = np.array([float(v) for v in axis_el.get("xyz").split()]) if axis_el is not None else np.zeros(3)
        lim = el.find("limit")
        joints[el.get("name")] = Joint(
            el.get("name"), el.find("parent").get("link"), el.find("child").get("link"),
            transform(xyz, rpy), axis, el.get("type"),
            float(lim.get("lower")) if lim is not None else None,
            float(lim.get("upper")) if lim is not None else None)
    return links, joints


@dataclass
class ArmModel:
    links: dict[str, Link]
    joints: dict[str, Joint]
    payload_kg: float = 0.0
    payload_point: str = "tcp"  # "tcp" = gripper_frame_link origin
    payload_extra_m: float = 0.0  # extra distance past the TCP along the finger direction
    joint_order: tuple[str, ...] = JOINTS

    # ----- forward kinematics -----
    def fk(self, q: dict[str, float]):
        """Return (joint_frames, link_frames): world 4x4 poses.  Base link frame = world (z up)."""
        link_T = {"base_link": np.eye(4)}
        joint_T = {}
        # joints in URDF are listed child-first; resolve by parent availability
        pending = list(self.joints.values())
        while pending:
            progress = False
            for j in list(pending):
                if j.parent in link_T:
                    Tj = link_T[j.parent] @ j.origin
                    joint_T[j.name] = Tj
                    ang = float(q.get(j.name, 0.0)) if j.kind != "fixed" else 0.0
                    link_T[j.child] = Tj @ rotz(ang) if j.kind != "fixed" else Tj
                    pending.remove(j)
                    progress = True
            if not progress:
                raise RuntimeError("URDF tree not resolvable")
        return joint_T, link_T

    def tcp(self, q: dict[str, float]) -> np.ndarray:
        _, link_T = self.fk(q)
        return link_T["gripper_frame_link"][:3, 3]

    # ----- gravity -----
    def distal_links(self, joint_name: str) -> list[str]:
        """Names of links that move when this joint moves."""
        out, frontier = [], [self.joints[joint_name].child]
        while frontier:
            c = frontier.pop()
            out.append(c)
            frontier += [j.child for j in self.joints.values() if j.parent == c]
        return out

    def masses_and_coms(self, q: dict[str, float]):
        """World COM positions and masses of every moving link (plus the payload)."""
        joint_T, link_T = self.fk(q)
        items = []
        for name, link in self.links.items():
            if name == "base_link" or link.mass < 1e-6:
                continue
            c = (link_T[name] @ np.append(link.com, 1.0))[:3]
            items.append((name, link.mass, c))
        if self.payload_kg > 0:
            T = link_T["gripper_frame_link"]
            local = np.array([0.0, 0.0, self.payload_extra_m, 1.0])
            items.append(("payload", self.payload_kg, (T @ local)[:3]))
        return joint_T, link_T, items

    def hold_torques(self, q: dict[str, float]) -> dict[str, float]:
        """Torque (N*m) the actuator of each joint must apply along its axis to hold the pose against gravity.

        tau_j = dU/dq_j with U = sum m g z.  Positive = about the joint's +z axis (URDF sense).
        """
        joint_T, link_T, items = self.masses_and_coms(q)
        out = {}
        for jn in self.joint_order:
            j = self.joints[jn]
            Tj = joint_T[jn]
            a = Tj[:3, :3] @ j.axis
            p = Tj[:3, 3]
            moving = set(self.distal_links(jn))
            total = 0.0
            for name, m, c in items:
                # The payload sits between the jaws: it moves with every joint up to wrist_roll, but the weight of a
                # gripped object is carried by the grasp, not by the jaw servo, so it never loads the 'gripper' joint.
                if name in moving or (name == "payload" and "gripper_link" in moving):
                    total += m * G * float(np.array([0, 0, 1.0]) @ np.cross(a, c - p))
            out[jn] = total
        return out

    def hold_torques_numeric(self, q: dict[str, float], h: float = 1e-6) -> dict[str, float]:
        """Independent check: tau_j = dU/dq_j by central differences of the potential energy."""
        def U(qq):
            _, _, items = self.masses_and_coms(qq)
            return sum(m * G * c[2] for _, m, c in items)
        out = {}
        for jn in self.joint_order:
            qp, qm = dict(q), dict(q)
            qp[jn] = q.get(jn, 0.0) + h
            qm[jn] = q.get(jn, 0.0) - h
            out[jn] = (U(qp) - U(qm)) / (2 * h)
        return out


# ---------------------------------------------------------------------------------------------
# Raw servo ticks <-> URDF joint angle (replicates BridgeController._ik_angle_from_raw + CadKinematics.ToUrdf)
# ---------------------------------------------------------------------------------------------
RES = 4096


def _load(name: str) -> dict:
    return json.loads((BACKEND / name).read_text(encoding="utf-8"))


class RawMap:
    def __init__(self):
        self.limits = _load("gui_limits.json")["motor_raw_limits"]
        self.align = _load("CadAlignment.json")
        self.geometry = _load("arm_geometry.json")
        self.servo_sign = {n: int(self.geometry["ik"]["servo_signs"].get(n, 1)) for n in JOINTS}
        self.direction = self.align["Direction"]
        self.zero_raw = self.align["ReferenceRaw"]
        self.home_raw = _load("home_preset.json")

    def arc(self, name: str):
        lim = self.limits[name]
        lo, hi, inv = lim["min"], lim["max"], bool(lim["arc_inverted"])
        span = ((lo - hi) if inv else (hi - lo)) % RES
        return lo, hi, inv, span

    def dist_from_left(self, name: str, raw: int) -> int:
        lo, hi, inv, _ = self.arc(name)
        r = int(raw) % RES
        return (lo - r) % RES if inv else (r - lo) % RES

    def contains(self, name: str, raw: int) -> bool:
        return self.dist_from_left(name, raw) <= self.arc(name)[3]

    def track_pos(self, name: str, raw: int) -> float:
        lo, hi, inv, span = self.arc(name)
        if self.contains(name, raw):
            return float(self.dist_from_left(name, raw))
        circ = lambda a, b: min((a - b) % RES, (b - a) % RES)
        ld, rd = circ(raw % RES, lo), circ(raw % RES, hi)
        return -float(ld) if ld <= rd else float(span + rd)

    def ik_angle_deg(self, name: str, raw: int) -> float:
        zero_off = float(self.dist_from_left(name, self.zero_raw[name] % RES))
        return self.servo_sign[name] * (self.track_pos(name, raw) - zero_off) * 360.0 / RES

    def urdf_angle_rad(self, name: str, raw: int) -> float:
        # CadKinematics.ToUrdf: urdf = UrdfAtReference + Direction * app_angle
        return math.radians(self.align["UrdfAtReferenceDegrees"][name] + self.direction[name] * self.ik_angle_deg(name, raw))

    def raw_from_urdf(self, name: str, urdf_rad: float) -> int:
        app = (math.degrees(urdf_rad) - self.align["UrdfAtReferenceDegrees"][name]) / self.direction[name]
        lo, hi, inv, span = self.arc(name)
        zero_off = float(self.dist_from_left(name, self.zero_raw[name] % RES))
        track = zero_off + app * RES / (360.0 * self.servo_sign[name])
        return int(round(lo - track if inv else lo + track)) % RES

    def urdf_limits_rad(self, name: str) -> tuple[float, float]:
        """Software (slider arc) limits converted to URDF angles."""
        lo, hi, inv, span = self.arc(name)
        a0 = self.urdf_angle_rad(name, lo)
        a1 = self.urdf_angle_rad(name, hi)
        return (min(a0, a1), max(a0, a1))

    def urdf_pose_from_raw(self, raw: dict[str, int]) -> dict[str, float]:
        return {n: self.urdf_angle_rad(n, raw[n]) for n in JOINTS if n in raw}


def default_model(payload_kg: float = 0.0, payload_extra_m: float = 0.0) -> ArmModel:
    links, joints = load_urdf()
    return ArmModel(links, joints, payload_kg=payload_kg, payload_extra_m=payload_extra_m)


if __name__ == "__main__":
    m = default_model()
    rm = RawMap()
    print("moving mass (kg):", {k: v.mass for k, v in m.links.items()})
    print("total moving mass:", sum(v.mass for k, v in m.links.items() if k != "base_link"))
    q_home = rm.urdf_pose_from_raw(rm.home_raw)
    print("Home raw:", rm.home_raw)
    print("Home URDF angles (deg):", {k: round(math.degrees(v), 2) for k, v in q_home.items()})
    tau = m.hold_torques(q_home)
    tau_n = m.hold_torques_numeric(q_home)
    print("Home hold torques (N*m):", {k: round(v, 4) for k, v in tau.items()})
    print("numeric check        :", {k: round(v, 4) for k, v in tau_n.items()})
    print("Home hold torques (kg*cm):", {k: round(v * KGCM_PER_NM, 3) for k, v in tau.items()})
    tcp = m.tcp(q_home)
    print("TCP at Home in base_link frame (cm):", np.round(tcp * 100, 2))
