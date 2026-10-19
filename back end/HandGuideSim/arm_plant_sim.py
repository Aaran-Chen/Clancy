"""Offline physics plant for hand guiding: SO-101 lift / elbow / wrist_flex (gravity) and pan / wrist_roll / gripper.

Nothing here touches hardware. Models (docs/HandGuide.md, "Offline simulation"):
  * arm: 3-link coupled rigid-body dynamics of lift, elbow and wrist_flex from the repo URDF (mass matrix from link
    masses, COMs and inertias at the current pose; gravity from the exact closed form, with a mass-scale and payload
    error vs the controller's model);
  * pan, wrist_roll, gripper: one geared joint each with no gravity load; the link inertia about the joint axis from
    the URDF at the start pose (pan) or a floor for the fingers that hold it (roll, gripper: their own links are 4e-5
    and 1.6e-5 kg m^2, far below the reflected rotor); a small output-bearing friction (pan 0.01, others 0.002 N m);
  * gear (every joint): two masses per servo (reflected rotor inertia on the gear side, the link on the output side,
    a backlash gap with a stiff, damped contact), gear-side stick-slip friction F_s = F0 + f_load * |transmitted
    torque|, kinetic 0.8 F_s, a small viscous term;
  * motor: duty -> output torque kt (d V / 1000 - ke w) / R (12 V-class STS3215, datasheet derived): energised at duty
    0 it brakes (slow-decay drive, the back-EMF drag ke kt / R = 0.57 N m s/rad at the output); torque off floats;
  * servo firmware (clean-room 3.10 semantics): 12-bit output-shaft encoder with its 0.2/0.8 IIR, position loop
    duty = P/4 e + D/8 de(20 ticks) + punch, dead zone, clamp +-Torque_Limit and 970, ~1.4 kHz tick; goal writes
    energise; overload (bit 5: >80 % duty while stalled for 2 s -> output 20 %) and overcurrent (bit 3) protection;
    Present_Load = applied duty; a first-order thermal model;
  * bus and host: host cycles at a set period with jitter and injectable stalls, sync read sampled 0.5-1 ms into the
    cycle and returned 1.6-3.1 ms after it, goal writes landing 1.9-3.4 ms later, per-reply drops, port outages;
  * faults: every fault exit coasts (Aaran, 2026-10-10): torque off on every servo that answers; a silent servo keeps
    its last goal until it answers again (the bridge's AutoCoast retries the torque-off every frame); a USB drop while
    guiding latches at once and coasts the moment the link is back (the existing auto-coast-v1 rule for a move).
"""
from __future__ import annotations

import math
import random
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, Optional

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import hand_guide_law as L  # noqa: E402
from gravity_model import default_model  # noqa: E402

STEP = L.STEP_RAD
COMP = L.COMP_JOINTS
FREE = L.FREE_JOINTS
ALL = L.ALL_JOINTS
KGCM = 100.0 / 9.80665
TRUE_P0 = np.array([-0.085899, -0.402063, -0.335500, -0.012385, -0.117011, -0.002459])
DP100 = np.array([-0.027458, -0.110394, -0.132292, -0.005099, -0.156149, 0.007748])
START_RAW = {"shoulder_pan": 3479, "shoulder_lift": 2700, "elbow_flex": 1200, "wrist_flex": 700, "wrist_roll": 1513,
             "gripper": 900}
FREE_LINK_FLOOR = {"shoulder_pan": 0.0, "wrist_roll": 3e-4, "gripper": 2e-4}   # kg m^2: the fingers on it
FREE_LINK_FRICTION = {"shoulder_pan": 0.01, "wrist_roll": 0.002, "gripper": 0.002}  # N m: output bearings


def hold_torques(theta: np.ndarray, params: np.ndarray) -> np.ndarray:
    A2, B2, A3, B3, A4, B4 = params
    T2, T3, T4 = theta[0], theta[0] + theta[1], theta[0] + theta[1] + theta[2]
    w = A4 * math.cos(T4) + B4 * math.sin(T4)
    e = A3 * math.cos(T3) + B3 * math.sin(T3) + w
    l = A2 * math.cos(T2) + B2 * math.sin(T2) + e
    return np.array([l, e, w])


class MassMatrix:
    """M(q) of the lift-elbow-wrist chain (pan, roll, gripper at 0) from the URDF; payload as a point at the TCP."""

    def __init__(self, payload_kg: float = 0.0, mass_scale: float = 1.0):
        self.m = default_model(payload_kg)
        self.scale = mass_scale
        self.distal = {j: set(self.m.distal_links(j)) for j in ALL}

    def __call__(self, theta: np.ndarray) -> np.ndarray:
        q = {"shoulder_pan": 0.0, "shoulder_lift": theta[0], "elbow_flex": theta[1], "wrist_flex": theta[2],
             "wrist_roll": 0.0, "gripper": 0.0}
        joint_T, link_T = self.m.fk(q)
        axes, origins = [], []
        for j in COMP:
            T = joint_T[j]
            axes.append(T[:3, :3] @ self.m.joints[j].axis)
            origins.append(T[:3, 3])
        M = np.zeros((3, 3))
        items = []
        for name, link in self.m.links.items():
            if name in ("base_link", "shoulder_link") or link.mass < 1e-6:
                continue
            Tl = link_T[name]
            c = (Tl @ np.append(link.com, 1.0))[:3]
            Iw = Tl[:3, :3] @ link.inertia @ Tl[:3, :3].T if link.inertia is not None else np.zeros((3, 3))
            items.append((name, link.mass * self.scale, c, Iw * self.scale))
        if self.m.payload_kg > 0:
            items.append(("payload", self.m.payload_kg, link_T["gripper_frame_link"][:3, 3], np.zeros((3, 3))))
        for name, mass, c, Iw in items:
            Jv = np.zeros((3, 3))
            Jw = np.zeros((3, 3))
            for i, j in enumerate(COMP):
                if name in self.distal[j] or (name == "payload" and "gripper_link" in self.distal[j]):
                    Jv[:, i] = np.cross(axes[i], c - origins[i])
                    Jw[:, i] = axes[i]
            M += mass * Jv.T @ Jv + Jw.T @ Iw @ Jw
        return M

    def axis_inertia(self, theta: np.ndarray, joint: str) -> float:
        """Inertia of everything distal to `joint` about its axis (pan, roll, gripper), at this lift/elbow/wrist pose."""
        q = {"shoulder_pan": 0.0, "shoulder_lift": theta[0], "elbow_flex": theta[1], "wrist_flex": theta[2],
             "wrist_roll": 0.0, "gripper": 0.0}
        joint_T, link_T = self.m.fk(q)
        T = joint_T[joint]
        a, o = T[:3, :3] @ self.m.joints[joint].axis, T[:3, 3]
        J = 0.0
        for name, link in self.m.links.items():
            if name not in self.distal[joint] or link.mass < 1e-6:
                continue
            Tl = link_T[name]
            c = (Tl @ np.append(link.com, 1.0))[:3]
            r = np.cross(a, c - o)
            Iw = Tl[:3, :3] @ link.inertia @ Tl[:3, :3].T if link.inertia is not None else np.zeros((3, 3))
            J += link.mass * self.scale * float(r @ r) + float(a @ Iw @ a) * self.scale
        return J


@dataclass
class GearParams:
    j_motor: float = 0.012        # reflected rotor + gear inertia at the output, kg m^2 (0.005-0.028)
    backlash_deg: float = 0.87    # total (Robonine bench: 0.77-0.87 deg)
    k_contact: float = 400.0      # N m/rad
    zeta: float = 0.5
    f0: float = 0.15              # N m, unloaded breakaway at the output
    f_load: float = 0.5           # share of the transmitted torque added to breakaway
    kinetic: float = 0.8
    viscous: float = 0.02         # N m s/rad


def free_gear(f0: float = 0.08, f_load: float = 0.3, j_motor: float = 0.008, **kw) -> GearParams:
    return GearParams(j_motor=j_motor, f0=f0, f_load=f_load, **kw)


@dataclass
class ServoRegs:
    mode: int = 0
    torque: bool = True
    goal: float = 0.0             # steps
    pwm: float = 0.0
    limit: float = 1000.0
    p: int = 32
    d: int = 32
    punch: int = 16
    dz: int = 1
    status: int = 0
    lock: int = 1


class Servo:
    TICK = 0.0007

    def __init__(self, name: str, regs: ServoRegs, ref_raw: float, volts: Callable[[float], float]):
        self.name = name
        self.r = regs
        self.ref = ref_raw
        self.enc_f: Optional[float] = None
        self.e_hist = []
        self.dterm = 0.0
        self.tick_n = 0
        self.duty = 0.0
        self.current = 0.0
        self.volts = volts
        self.over_t = 0.0
        self.oc_t = 0.0
        self.overloaded = False
        self.temp = 30.0
        self.t_next = 0.0
        self.turn = 0.0            # multi-turn frame offset: the reading starts in 0..4095 at power-up / reset
        self.temp_add = 0.0        # injected temperature offset (tests of the heat exit)
        self.coast_brakes = False  # torque off shorts the motor (a braking driver) instead of floating

    def power_up(self, theta_l: float) -> None:
        raw = self.ref + theta_l / STEP
        self.turn = -math.floor(raw / 4096.0) * 4096.0
        self.enc_f = None

    def tick(self, t: float, theta_l: float, omega_m: float) -> None:
        r = self.r
        raw = self.ref + theta_l / STEP + self.turn
        q = round(raw)
        self.enc_f = q if self.enc_f is None else 0.2 * self.enc_f + 0.8 * q
        V = self.volts(t)
        if not r.torque:
            self.duty, self.e_hist, self.dterm = 0.0, [], 0.0
        elif r.mode == 0:
            e = r.goal - self.enc_f
            e_eff = 0.0 if abs(e) <= r.dz else e - math.copysign(r.dz, e)
            self.tick_n += 1
            self.e_hist.append(e_eff)
            if len(self.e_hist) > 21:
                self.e_hist.pop(0)
            if self.tick_n % 20 == 0 and len(self.e_hist) >= 21:
                self.dterm = r.d / 8.0 * (self.e_hist[-1] - self.e_hist[0])
            out = r.p / 4.0 * e_eff + self.dterm
            if out > 0:
                out += r.punch
            elif out < 0:
                out -= r.punch
            self.duty = max(-r.limit, min(r.limit, out))
        else:
            self.duty = max(-r.limit, min(r.limit, r.pwm))
        self.duty = max(-970.0, min(970.0, self.duty))
        if self.overloaded:
            self.duty = max(-200.0, min(200.0, self.duty))
        # firmware overload: >80 % output with the joint not moving for 2 s
        if abs(self.duty) > 800 and abs(omega_m) < 0.05:
            self.over_t += self.TICK
            if self.over_t > 2.0:
                self.overloaded, r.status = True, r.status | (1 << 5)
        else:
            self.over_t = 0.0
        self.current = (self.duty / 1000.0 * V - L.KE * omega_m) / L.R_OHM if r.torque else 0.0
        if abs(self.current) > 2.0:
            self.oc_t += self.TICK
            if self.oc_t > 2.0:
                r.torque, r.status = False, r.status | (1 << 3)
        else:
            self.oc_t = 0.0
        # thermal: 5 K/W, 300 s
        self.temp += ((self.current ** 2) * L.R_OHM - (self.temp - 25.0) / 5.0) / 60.0 * self.TICK

    def motor_torque(self, t: float, omega_m: float) -> float:
        if not self.r.torque:
            return -L.KT * L.KE * omega_m / L.R_OHM if self.coast_brakes else 0.0
        return L.KT * (self.duty / 1000.0 * self.volts(t) - L.KE * omega_m) / L.R_OHM

    # register writes as the firmware answers them
    def write_goal(self, goal: float, limit: Optional[float] = None) -> None:
        if limit is not None:
            self.r.limit = float(max(0.0, min(1000.0, limit)))
        self.r.goal = float(goal)
        if self.r.mode == 0:
            self.r.torque = True       # a goal write energises (position mode)

    def write_torque(self, on: bool) -> None:
        self.r.torque = on
        if not on:
            self.overloaded = False
            self.r.status &= ~(1 << 5)

    def present(self) -> float:
        return float(round(self.enc_f)) if self.enc_f is not None else 0.0


@dataclass
class Scenario:
    name: str
    seconds: float = 6.0
    start_raw: Dict[str, float] = field(default_factory=dict)     # any joints, steps
    gear: Dict[str, GearParams] = field(default_factory=dict)     # any joints
    payload_kg: float = 0.0
    mass_scale: float = 1.0                                        # plant vs model
    model_scale: Dict[str, float] = field(default_factory=dict)    # controller model error per joint (sign errors: -1)
    model_flip_at: Optional[float] = None                          # a runtime sign error on the lift at this time
    model_flip_joint: str = "shoulder_lift"
    host_period: float = 0.01
    host_jitter: float = 0.0008
    stalls: list = field(default_factory=list)                     # (start_s, duration_s)
    port_loss: list = field(default_factory=list)                  # (start_s, duration_s)
    drop_p: float = 0.0
    drop_burst: Optional[tuple] = None                             # (start_s, cycles, joint)
    volts: Callable[[float], float] = lambda t: 8.9
    volt_comp: bool = True
    p_gain: int = 32
    enter_at: float = 1.0
    hand: Optional[Callable] = None                                # (t, sim) -> {joint: torque}
    exit_at: Optional[float] = None                                # Hold here (preloaded exit) at this time
    fault_at: Optional[float] = None                               # a fault exit (coast) at this time: the E-stop drill
    heat_at: Optional[tuple] = None                                # (t, joint, +deg C): a servo's reading jumps
    params: Dict = field(default_factory=dict)                     # GuideParams overrides
    cfg_over: Dict[str, Dict] = field(default_factory=dict)        # JointConfig / FreeConfig overrides per joint
    free_joints: tuple = FREE                                      # the gripper is left out while it squeezes
    ident: Dict[str, Dict] = field(default_factory=dict)           # calibration error per joint: {"f": x, "kv": x}
    assist: bool = True                                            # the identified friction feed-forward (k_f) on
    band: Optional[Callable] = None                                # (theta dict) -> bool (in the guard band)
    brownout: Optional[tuple] = None                               # (t, joint): servo resets
    band_lookahead_s: float = 0.0
    coast_reference: bool = False                                  # servos torque off from t=0, no guiding (feel baseline)
    coast_brakes: bool = False                                     # the driver brakes when torque is off (bring-up S2 decides)
    hold_reference: bool = False                                   # the ordinary Hold, no guiding (feel baseline)
    record: bool = False
    seed: int = 1


class ArmSim:
    DT = 0.0002

    def __init__(self, sc: Scenario):
        self.sc = sc
        self.rng = random.Random(sc.seed)
        self.t = 0.0
        self.M = MassMatrix(sc.payload_kg, sc.mass_scale)
        self.true_p = TRUE_P0 * sc.mass_scale + DP100 * (sc.payload_kg / 0.1)
        self.gear = {j: sc.gear.get(j, GearParams()) for j in COMP}
        if "wrist_flex" not in sc.gear:
            self.gear["wrist_flex"] = GearParams(j_motor=0.008, f0=0.10)
        for j in FREE:
            self.gear[j] = sc.gear.get(j, free_gear())
        start = dict(START_RAW)
        start.update(sc.start_raw)
        self.theta_l = np.array([(start[j] - L.REFERENCE_RAW[j]) * STEP for j in COMP], dtype=float)
        self.omega_l = np.zeros(3)
        self.theta_m = self.theta_l.copy()
        self.omega_m = np.zeros(3)
        self.stuck = [True, True, True]
        # free joints: two masses each, no gravity
        self.f_th = {j: (start[j] - L.REFERENCE_RAW[j]) * STEP for j in FREE}
        self.f_om = {j: 0.0 for j in FREE}
        self.f_thm = dict(self.f_th)
        self.f_omm = {j: 0.0 for j in FREE}
        self.f_stuck = {j: True for j in FREE}
        self.f_J = {j: max(self.M.axis_inertia(self.theta_l, j), FREE_LINK_FLOOR[j]) for j in FREE}
        self.f_dc = {}
        for j in FREE:
            g = self.gear[j]
            m_red = g.j_motor * self.f_J[j] / (g.j_motor + self.f_J[j])
            self.f_dc[j] = 2 * g.zeta * math.sqrt(g.k_contact * m_red)
        self.servos: Dict[str, Servo] = {}
        for i, j in enumerate(ALL):
            th = self.link_theta(j)
            raw = L.REFERENCE_RAW[j] + th / STEP
            regs = ServoRegs(goal=round(raw), p=sc.p_gain)
            self.servos[j] = Servo(j, regs, L.REFERENCE_RAW[j], sc.volts)
            self.servos[j].t_next = i * 0.0001
            self.servos[j].coast_brakes = sc.coast_brakes
        self.dc = [0.0, 0.0, 0.0]
        self.update_mass()
        for j in FREE:
            sv = self.servos[j]
            sv.power_up(self.f_th[j])
            sv.r.goal = round(L.REFERENCE_RAW[j] + self.f_th[j] / STEP + sv.turn)
            sv.enc_f = sv.r.goal
        self.settle_in_hold()
        if sc.coast_reference:
            for sv in self.servos.values():
                sv.write_torque(False)
        self.law: Optional[L.HandGuideLaw] = None
        self.phase = "hold"
        self.exit_kind = None
        self.exit_reason = ""
        self.exit_t = None
        self.events = []
        self.log = []
        self.max_speed = {j: 0.0 for j in ALL}
        self.pending_writes = []       # (t_apply, cmd)
        self.wall_verdict = (False, 0.0)
        self.next_wall = 0.0
        self.cycle_k = 0
        self.next_host = sc.enter_at
        self.flipped = False
        self.link_down_until = None
        self.coast_pending: set = set()   # silent servos the bridge keeps trying to coast
        self.free_torque_log = {j: [] for j in FREE}

    # ----------------------------------------------------------------- physics
    def update_mass(self) -> None:
        self.Mc = self.M(self.theta_l)
        self.Mc_t = self.t
        eff = 1.0 / np.diag(np.linalg.inv(self.Mc))
        for i, j in enumerate(COMP):
            g = self.gear[j]
            m_red = g.j_motor * eff[i] / (g.j_motor + eff[i])
            self.dc[i] = 2 * g.zeta * math.sqrt(g.k_contact * m_red)

    def settle_in_hold(self) -> None:
        """Start as the bridge's Hold leaves an arm: gears pressed on the side that carries gravity and each goal where
        the position loop alone holds the pose (what the hold correction converges to). Free joints: gear centred."""
        tau = self.gravity_hold()
        V = self.sc.volts(0.0)
        for i, j in enumerate(COMP):
            g = self.gear[j]
            bh = math.radians(g.backlash_deg) / 2
            self.theta_m[i] = self.theta_l[i] + math.copysign(bh + abs(tau[i]) / g.k_contact, tau[i])
            s = self.servos[j]
            s.power_up(self.theta_l[i])
            raw = L.REFERENCE_RAW[j] + self.theta_l[i] / STEP + s.turn
            duty = tau[i] / L.kd_nm_per_unit(V)
            K = s.r.p / 4.0
            off = math.copysign(max(abs(duty) - s.r.punch, 0.0) / K + s.r.dz, duty) if abs(duty) > s.r.punch else 0.0
            s.r.goal = round(raw + off)
            s.enc_f = round(raw)

    def gravity_hold(self) -> np.ndarray:
        return hold_torques(self.theta_l, self.true_p)

    @staticmethod
    def _gear_side(g: GearParams, stuck: bool, w: float, tm: float, tc: float, dt: float):
        """One step of the gear (motor) side: stick-slip friction F0 + f_load |tc|, kinetic 0.8 x."""
        net = tm - tc
        fs = g.f0 + g.f_load * abs(tc)
        fk = g.kinetic * fs
        if stuck:
            if abs(net) > fs:
                return False, (net - math.copysign(fk, net)) / g.j_motor * dt
            return True, 0.0
        a = (net - math.copysign(fk, w if w != 0 else net) - g.viscous * w) / g.j_motor
        w2 = w + a * dt
        if w * w2 <= 0 and abs(net) <= fs:
            return True, 0.0
        return False, w2

    @staticmethod
    def _contact(g: GearParams, dlt: float, dw: float, dc: float) -> float:
        bh = math.radians(g.backlash_deg) / 2
        if dlt > bh:
            return max(0.0, g.k_contact * (dlt - bh) + dc * dw)
        if dlt < -bh:
            return min(0.0, g.k_contact * (dlt + bh) + dc * dw)
        return 0.0

    def physics_step(self, tau_hand: Dict[str, float]) -> None:
        dt = self.DT
        if self.t - self.Mc_t >= 0.002:
            self.update_mass()
        tc = np.zeros(3)
        for i, j in enumerate(COMP):
            g = self.gear[j]
            tc[i] = self._contact(g, self.theta_m[i] - self.theta_l[i], self.omega_m[i] - self.omega_l[i], self.dc[i])
            tm = self.servos[j].motor_torque(self.t, self.omega_m[i])
            self.stuck[i], self.omega_m[i] = self._gear_side(g, self.stuck[i], self.omega_m[i], tm, tc[i], dt)
            self.theta_m[i] += self.omega_m[i] * dt
        th = np.array([tau_hand.get(j, 0.0) for j in COMP])
        acc = np.linalg.solve(self.Mc, tc - self.gravity_hold() + th - 0.003 * self.omega_l)
        self.omega_l += acc * dt
        self.theta_l += self.omega_l * dt
        for j in FREE:
            g = self.gear[j]
            c = self._contact(g, self.f_thm[j] - self.f_th[j], self.f_omm[j] - self.f_om[j], self.f_dc[j])
            tm = self.servos[j].motor_torque(self.t, self.f_omm[j])
            self.f_stuck[j], self.f_omm[j] = self._gear_side(g, self.f_stuck[j], self.f_omm[j], tm, c, dt)
            self.f_thm[j] += self.f_omm[j] * dt
            # output side: the link on its bearing (Coulomb, sticks when it stops)
            w, drive, fb = self.f_om[j], c + tau_hand.get(j, 0.0), FREE_LINK_FRICTION[j]
            if w == 0.0 and abs(drive) <= fb:
                w2 = 0.0
            else:
                w2 = w + (drive - math.copysign(fb, w if w != 0 else drive) - 0.0005 * w) / self.f_J[j] * dt
                if w * w2 < 0 and abs(drive) <= fb:
                    w2 = 0.0
            self.f_om[j] = w2
            self.f_th[j] += self.f_om[j] * dt
        self.t += dt

    # ----------------------------------------------------------------- readings and writes
    def link_theta(self, j: str) -> float:
        return self.theta_l[COMP.index(j)] if j in COMP else self.f_th[j]

    def link_omega(self, j: str) -> float:
        return self.omega_l[COMP.index(j)] if j in COMP else self.f_om[j]

    def gear_omega(self, j: str) -> float:
        return self.omega_m[COMP.index(j)] if j in COMP else self.f_omm[j]

    def silent_now(self) -> set:
        b = self.sc.drop_burst
        if b and b[0] <= self.t < b[0] + b[1] * self.sc.host_period:
            return {b[2]}
        return set()

    def sample(self, read_torque: bool) -> Dict[str, L.Reading]:
        out = {}
        silent = self.silent_now()
        for j, s in self.servos.items():
            ok = self.rng.random() >= self.sc.drop_p and j not in silent
            out[j] = L.Reading(ok=ok, pos=s.present(), load=s.duty, volts=s.volts(self.t), temp_c=s.temp + s.temp_add,
                               status=s.r.status, torque_on=s.r.torque if read_torque else None,
                               lock=s.r.lock if read_torque else None)
        return out

    def apply(self, cmd: L.Command) -> None:
        for j, w in cmd.writes.items():
            self.servos[j].write_goal(w["goal"], w["limit"])
        for j in cmd.release:
            self.servos[j].write_torque(False)

    def schedule(self, delay: float, cmd: L.Command) -> None:
        self.pending_writes.append((self.t + delay, cmd))

    def coast_now(self) -> None:
        """The bridge's AutoCoast: Torque_Enable 0 to every joint; a silent one is retried until it answers."""
        silent = self.silent_now()
        for j, s in self.servos.items():
            if j in silent:
                self.coast_pending.add(j)
            else:
                s.write_torque(False)

    # ----------------------------------------------------------------- host
    def host_cycle(self) -> None:
        sc = self.sc
        if self.link_down_until is not None:
            if self.t < self.link_down_until:
                return
            # the link is back: guiding was a move in progress, so the drop latched at once; torque-off goes out now
            self.link_down_until = None
            self._finish(L.Command(exit_kind="coast", reason="USB serial link dropped while hand guiding: coasted "
                                                             "the moment it returned"))
            return
        if self.phase in ("guide",) and self.law is not None and self.law.phase != "done":
            for t0, d in sc.port_loss:
                if t0 <= self.t < t0 + d:
                    self.link_down_until = t0 + d
                    self.events.append((round(self.t, 3), f"port lost {d:.1f} s: latched, the servos keep their last goals"))
                    self.pending_writes = []
                    return
        # every 4th cycle, and (as the bridge does) on the next one while the law holds a torque or Lock read-back suspect
        read_torque = self.cycle_k % 4 == 0 or bool(getattr(self.law, "status_recheck", False))
        readings = self.sample(read_torque)
        self.cycle_k += 1
        if self.phase == "hold":
            hold_goals = {j: self.servos[j].r.goal for j in COMP}
            self.law = self.make_law()
            cmd = self.law.begin(self.t, readings, hold_goals)
            self.phase = "guide"
            self.entry_t = self.t
            if cmd.exit_kind:
                self._finish(cmd)
                return
            self.schedule(self.rng.uniform(0.0019, 0.0034), cmd)
            return
        if self.phase != "guide":
            return
        if sc.model_flip_at is not None and not self.flipped and self.t >= sc.model_flip_at:
            self.law.model.scale[sc.model_flip_joint] *= -1
            self.flipped = True
            self.events.append((round(self.t, 3), f"model sign flipped on {sc.model_flip_joint}"))
        if self.t >= self.next_wall - 1e-9:
            th = self.measured_theta()
            if sc.band and sc.band_lookahead_s and self.law is not None:
                th = {j: v + self.law.v.get(j, 0.0) * sc.band_lookahead_s for j, v in th.items()}
            band = sc.band(th) if sc.band else False
            self.wall_verdict = (band, self.t)
            self.next_wall = self.t + 1 / 30
        req = "preloaded" if (sc.exit_at is not None and self.t >= sc.exit_at) else None
        if sc.fault_at is not None and self.t >= sc.fault_at and self.law.phase == "guide":
            self.law.phase = "done"
            self._finish(L.Command(exit_kind="coast", reason="fault (E-stop drill): every joint goes limp"))
            return
        cmd = self.law.step(self.t, readings, wall_band=self.wall_verdict[0], wall_age=self.t - self.wall_verdict[1],
                            exit_request=req)
        for e in cmd.events:
            self.events.append((round(self.t, 3), e))
        if cmd.exit_kind:
            self._finish(cmd)
            return
        self.schedule(self.rng.uniform(0.0019, 0.0034), cmd)

    def _finish(self, cmd: L.Command) -> None:
        self.phase = "coast" if cmd.exit_kind == "coast" else ("refused" if cmd.exit_kind == "refused" else "held")
        self.exit_kind, self.exit_reason, self.exit_t = cmd.exit_kind, cmd.reason, self.t
        self.events.append((round(self.t, 3), f"exit {cmd.exit_kind}: {cmd.reason}"))
        if cmd.exit_kind == "coast":
            self.pending_writes = []
            self.coast_now()
            return
        if cmd.exit_kind == "refused":
            return
        self.schedule(self.rng.uniform(0.0019, 0.0034), cmd)

    def measured_theta(self) -> Dict[str, float]:
        return {j: (self.servos[j].present() - L.REFERENCE_RAW[j]) * STEP for j in ALL}

    def make_law(self) -> L.HandGuideLaw:
        from gravity_model import RawMap
        rm = RawMap()
        sc = self.sc
        V = 8.9
        kd = L.kd_nm_per_unit(V)
        kv_true = L.kv_prior(V)
        cfgs = {}
        defaults = {"shoulder_lift": dict(cap_max=650.0, cap_min=150.0, v_cap=math.radians(45)),
                    "elbow_flex": dict(cap_max=450.0, cap_min=120.0, v_cap=math.radians(50)),
                    "wrist_flex": dict(cap_max=250.0, cap_min=100.0, v_cap=math.radians(80))}
        for j in COMP:
            lo, hi = rm.urdf_limits_rad(j)
            g = self.gear[j]
            idf = sc.ident.get(j, {})
            kw = dict(defaults[j])
            # what the calibration identifies: the kinetic friction (unloaded part and load share) and the slope
            kw.update(dict(k_servo=sc.p_gain / 4.0, lim_lo=lo, lim_hi=hi,
                           f_kin0=g.kinetic * g.f0 / kd * idf.get("f", 1.0),
                           f_kin_load=g.kinetic * g.f_load * idf.get("f", 1.0),
                           k_v=(kv_true + g.viscous / kd) * idf.get("kv", 1.0),
                           k_f=0.5 if sc.assist else 0.0))
            kw.update(sc.cfg_over.get(j, {}))
            cfgs[j] = L.JointConfig(j, **kw)
        fcfgs = {}
        for j in sc.free_joints:
            lo, hi = rm.urdf_limits_rad(j)
            g = self.gear[j]
            idf = sc.ident.get(j, {})
            kw = dict(L.FREE_DEFAULTS[j])
            kw.update(dict(k_servo=sc.p_gain / 4.0, lim_lo=lo, lim_hi=hi,
                           f_kin=g.kinetic * g.f0 / kd * idf.get("f", 1.0),
                           k_v=(kv_true + g.viscous / kd) * idf.get("kv", 1.0),
                           k_f=0.5 if sc.assist else 0.0, coast_brakes=sc.coast_brakes))
            kw.update(sc.cfg_over.get(j, {}))
            fcfgs[j] = L.FreeConfig(j, **kw)
        model = L.GravityModel()
        for j, s in sc.model_scale.items():
            model.scale[j] = s
        params = L.GuideParams(**sc.params)
        if not sc.volt_comp:
            params.v_cal = 8.9
        law = L.HandGuideLaw(cfgs, model, params, free_configs=fcfgs)
        if not sc.volt_comp:
            orig = law.model_effort
            law.model_effort = lambda raw, volts, gain: orig(raw, 8.9, gain)
        return law

    # ----------------------------------------------------------------- run
    def run(self) -> dict:
        sc = self.sc
        if sc.coast_reference or sc.hold_reference:
            self.next_host = math.inf
        brown_done = False
        heat_done = False
        while self.t < sc.seconds:
            # host
            if self.t >= self.next_host - 1e-12:
                stalled = next(((a, d) for a, d in sc.stalls if a <= self.t < a + d), None)
                if stalled:
                    self.next_host = stalled[0] + stalled[1]
                else:
                    if self.t >= sc.enter_at or self.phase != "hold":
                        self.host_cycle()
                    self.next_host = self.t + sc.host_period + self.rng.gauss(0, sc.host_jitter) * (self.phase == "guide")
            # writes landing at the servos
            if self.pending_writes:
                keep = []
                for ta, cmd in self.pending_writes:
                    if self.t >= ta:
                        if self.link_down_until is None:
                            self.apply(cmd)
                    else:
                        keep.append((ta, cmd))
                self.pending_writes = keep
            if self.coast_pending and not self.silent_now():
                for j in self.coast_pending:
                    self.servos[j].write_torque(False)
                self.events.append((round(self.t, 3), f"torque-off confirmed on {', '.join(sorted(self.coast_pending))}"))
                self.coast_pending = set()
            if sc.brownout and not brown_done and self.t >= sc.brownout[0]:
                s = self.servos[sc.brownout[1]]
                s.power_up(self.link_theta(sc.brownout[1]))
                s.r.torque = False
                s.tick(self.t, self.link_theta(sc.brownout[1]), 0.0)
                s.r.limit, s.r.goal, s.r.status, s.r.lock = 1000.0, s.present(), 0, 0
                brown_done = True
                self.events.append((round(self.t, 3), f"{sc.brownout[1]} reset (brown-out)"))
            if sc.heat_at and not heat_done and self.t >= sc.heat_at[0]:
                self.servos[sc.heat_at[1]].temp_add = sc.heat_at[2]
                heat_done = True
            # servo ticks
            for j, s in self.servos.items():
                if self.t >= s.t_next:
                    s.tick(self.t, self.link_theta(j), self.gear_omega(j))
                    s.t_next += Servo.TICK
            tau_hand = sc.hand(self.t, self) if sc.hand else {}
            self.physics_step(tau_hand)
            if self.t > sc.enter_at:
                for j in ALL:
                    self.max_speed[j] = max(self.max_speed[j], abs(self.link_omega(j)))
            if sc.record and int(self.t / self.DT) % 40 == 0:
                self.log.append((round(self.t, 4), *[round(math.degrees(self.link_theta(j)), 3) for j in ALL],
                                 *[round(self.servos[j].duty) for j in ALL], self.phase,
                                 *[round(tau_hand.get(j, 0.0), 4) for j in ALL],
                                 *[int(self.servos[j].r.torque) for j in ALL]))
        return self.summary()

    def summary(self) -> dict:
        law = self.law
        return {"name": self.sc.name, "exit": self.exit_kind, "reason": self.exit_reason,
                "exit_t": None if self.exit_t is None else round(self.exit_t, 3),
                "phase": self.phase, "law_phase": None if law is None else law.phase,
                "ramp_max_dev_steps": None if law is None else round(law.max_dev, 1),
                "final_deg": {j: round(math.degrees(self.link_theta(j)), 2) for j in ALL},
                "max_speed_dps": {j: round(math.degrees(v), 1) for j, v in self.max_speed.items()},
                "temps": {j: round(self.servos[j].temp, 1) for j in ALL},
                "status": {j: self.servos[j].r.status for j in ALL},
                "ff_on": None if law is None else {j: v for j, v in law.ff_on.items()},
                "assist_counts": None if law is None else law.assist_counts,
                "events": self.events[-14:]}


# log columns
COL_T = 0
COL_DEG = {j: 1 + i for i, j in enumerate(ALL)}
COL_DUTY = {j: 1 + len(ALL) + i for i, j in enumerate(ALL)}
COL_PHASE = 1 + 2 * len(ALL)
COL_HAND = {j: 2 + 2 * len(ALL) + i for i, j in enumerate(ALL)}
COL_TORQUE = {j: 2 + 3 * len(ALL) + i for i, j in enumerate(ALL)}
