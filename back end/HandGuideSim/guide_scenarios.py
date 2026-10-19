"""Scenario suite for hand guiding on the offline plant (arm_plant_sim.py + hand_guide_law.py).

Each scenario states the design claim it checks; run() prints a line per scenario and writes out_guide_scenarios.json
(or out_guide_<groups>.json for a subset). Offline only: no serial port, no bridge, no camera.

    python guide_scenarios.py                 (all groups, up to 24 worker processes)
    python guide_scenarios.py free_feel       (one group; several: free_feel free_release)
    python guide_scenarios.py --workers 8 ...
"""
from __future__ import annotations

import json
import math
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import arm_plant_sim as S  # noqa: E402
import hand_guide_law as L  # noqa: E402

DEG = math.pi / 180
REF = L.REFERENCE_RAW
GUIDE_FROM = 3.6        # entry at 1.0 s + preload 0.6 + ramp 1.2 + probe 0.4 = 3.2 s, plus a margin: the hand starts here


def raw(lift_deg, elbow_deg, wrist_deg):
    return {"shoulder_lift": REF["shoulder_lift"] + lift_deg * 4096 / 360,
            "elbow_flex": REF["elbow_flex"] + elbow_deg * 4096 / 360,
            "wrist_flex": REF["wrist_flex"] + wrist_deg * 4096 / 360}


def free_raw(joint, deg):
    return {joint: REF[joint] + deg * 4096 / 360}


POSES = {"extended": raw(77.5, -75.0, -1.0), "L": raw(0.0, 0.0, 0.0), "mid": raw(40.0, -40.0, -20.0),
         "home": {"shoulder_lift": 3161 - 4096, "elbow_flex": 2686, "wrist_flex": 856}}
WORK = raw(-10.0, -40.0, -20.0)      # the feel tests' pose (tip at 22 cm reach, 49 cm up)
FRICTION = {"low": dict(f0=0.05, f_load=0.2), "mid": dict(f0=0.15, f_load=0.5), "high": dict(f0=0.35, f_load=0.9),
            "ultralow": dict(f0=0.02, f_load=0.05)}
# free joints (no gravity on the gear): output breakaway in Coast = F0 / (1 - f_load) = 0.05 / 0.11 / 0.25 N m
FREE_FRICTION = {"low": dict(f0=0.04, f_load=0.2), "mid": dict(f0=0.08, f_load=0.3), "high": dict(f0=0.15, f_load=0.4)}
# where a hand pushes: lift/elbow/wrist at the gripper; pan at the gripper (30 cm out); roll on the gripper body (3 cm
# from its axis); gripper on the moving jaw's tip (5 cm)
LEVER = {"shoulder_lift": 0.35, "elbow_flex": 0.25, "wrist_flex": 0.12, "shoulder_pan": 0.30, "wrist_roll": 0.03,
         "gripper": 0.05}
HAND_GAINS = {"shoulder_pan": (1.0, 3.0), "wrist_roll": (0.3, 1.0), "gripper": (0.3, 1.0)}


def gears(level: str, j_motor: float = 0.012, free: str = "mid"):
    f = FRICTION[level]
    g = {"shoulder_lift": S.GearParams(j_motor=j_motor, **f), "elbow_flex": S.GearParams(j_motor=j_motor, **f),
         "wrist_flex": S.GearParams(j_motor=0.008, f0=f["f0"] * 0.6, f_load=f["f_load"])}
    g.update(free_gears(free))
    return g


def free_gears(level: str, scale: float = 1.0):
    f = FREE_FRICTION[level]
    return {j: S.free_gear(f0=f["f0"] * scale, f_load=f["f_load"] * scale) for j in L.FREE_JOINTS}


# --------------------------------------------------------------------------------------------- hands
class SpeedHand:
    """A hand that drives one joint at a target speed (PI on velocity, torque-limited) between t0 and t1, then lets go.
    Records the torque it needed, for the feel numbers."""

    def __init__(self, joint, deg_s, t0, t1, kp=None, ki=None, tau_max=4.0):
        dkp, dki = HAND_GAINS.get(joint, (3.0, 8.0))
        self.j, self.w, self.t0, self.t1 = joint, deg_s * DEG, t0, t1
        self.kp, self.ki, self.tmax = kp if kp is not None else dkp, ki if ki is not None else dki, tau_max
        self.integ = 0.0
        self.samples = []
        self.speeds = []
        self.last_t = None
        self.breakaway = 0.0       # largest torque before the joint had moved 1.5 deg (past the 0.87 deg backlash)
        self.broke = False
        self.th0 = None

    def __call__(self, t, sim):
        if not (self.t0 <= t < self.t1):
            return {}
        w = sim.link_omega(self.j)
        th = sim.link_theta(self.j)
        if self.th0 is None:
            self.th0 = th
        ramp = min(1.0, (t - self.t0) / 0.3)
        err = self.w * ramp - w
        dt = 0.0 if self.last_t is None else t - self.last_t
        self.last_t = t
        self.integ += err * dt
        tau = max(-self.tmax, min(self.tmax, self.kp * err + self.ki * self.integ))
        if not self.broke:
            self.breakaway = max(self.breakaway, abs(tau))
            if abs(th - self.th0) >= 1.5 * DEG:
                self.broke = True
        if t > self.t0 + 0.6 * (self.t1 - self.t0) * 0.5 + 0.3:
            self.samples.append(tau)
            self.speeds.append(w)
        return {self.j: tau}

    def mean_force(self):
        if not self.samples:
            return float("nan")
        n = len(self.samples)
        tail = self.samples[n // 3:]
        return abs(sum(tail) / len(tail)) / LEVER[self.j]

    def mean_speed(self):
        if not self.speeds:
            return float("nan")
        tail = self.speeds[len(self.speeds) // 3:]
        return sum(tail) / len(tail)


class PdHand:
    """A hand holding joint j and moving it along a straight line to `to_deg` (relative) at deg_s, from t0;
    releases at t_release. Stiffness of a light grip, torque limited."""

    def __init__(self, joint, to_deg, deg_s, t0, t_release, k=6.0, b=0.6, tau_max=3.0):
        self.j, self.to, self.w, self.t0, self.tr, self.k, self.b, self.tmax = joint, to_deg * DEG, deg_s * DEG, t0, t_release, k, b, tau_max
        self.start = None
        self.max_tau = 0.0

    def __call__(self, t, sim):
        th, om = sim.link_theta(self.j), sim.link_omega(self.j)
        if t < self.t0 or t >= self.tr:
            if t < self.t0:
                self.start = th
            return {}
        if self.start is None:
            self.start = th
        travel = min(abs(self.to), self.w * (t - self.t0))
        target = self.start + math.copysign(travel, self.to)
        moving = travel < abs(self.to)
        tau = self.k * (target - th) + self.b * ((math.copysign(self.w, self.to) if moving else 0.0) - om)
        tau = max(-self.tmax, min(self.tmax, tau))
        self.max_tau = max(self.max_tau, abs(tau))
        return {self.j: tau}


class FlickHand:
    """A quick hand move: joint j follows a minimum-jerk path of `deg` degrees in `T` seconds from t0 (a stiff grip,
    PD on the path), then holds the end for 0.3 s and lets go. Records the peak and RMS torque the move took."""

    def __init__(self, joint, deg, T, t0, k=None, b=None):
        self.j, self.D, self.T, self.t0 = joint, deg * DEG, T, t0
        self.k = k if k is not None else {"shoulder_pan": 40.0}.get(joint, 12.0)
        self.b = b if b is not None else {"shoulder_pan": 1.5}.get(joint, 0.25)
        self.start = None
        self.peak = 0.0
        self.sq, self.n = 0.0, 0
        self.track_err = 0.0

    def __call__(self, t, sim):
        th, om = sim.link_theta(self.j), sim.link_omega(self.j)
        if t < self.t0:
            self.start = th
            return {}
        if t >= self.t0 + self.T + 0.3:
            return {}
        s = min(1.0, (t - self.t0) / self.T)
        x = self.start + self.D * (10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5)
        xd = self.D / self.T * (30 * s ** 2 - 60 * s ** 3 + 30 * s ** 4) if s < 1 else 0.0
        tau = self.k * (x - th) + self.b * (xd - om)
        if s < 1:
            self.peak = max(self.peak, abs(tau))
            self.sq += tau * tau
            self.n += 1
            self.track_err = max(self.track_err, abs(x - th))
        return {self.j: tau}


def combine(*hands):
    def h(t, sim):
        out = {}
        for x in hands:
            for k, v in x(t, sim).items():
                out[k] = out.get(k, 0.0) + v
        return out
    h.parts = hands
    return h


# --------------------------------------------------------------------------------------------- scenarios
def sc_start(pose, fric, scale=None, seconds=5.5, name=None, **kw):
    return S.Scenario(name or f"start {pose} {fric} model {scale or 1.0}", seconds=seconds, start_raw=POSES[pose],
                      gear=gears(fric), model_scale=scale or {}, **kw)


def run_one(sc: S.Scenario, extra=None):
    sim = S.ArmSim(sc)
    t0 = time.perf_counter()
    out = sim.run()
    out["wall_s"] = round(time.perf_counter() - t0, 1)
    if extra:
        out.update(extra(sim))
    return out


def comp_c(c):
    return {j: {"c_lead": c} for j in L.COMP_JOINTS}


def build():
    groups = {}

    # A. starts that must succeed (model right or within 10 %), then hands off: hover; the probe passes
    A = []
    for pose in ("extended", "mid", "L", "home"):
        for fric in ("low", "mid", "high"):
            A.append(sc_start(pose, fric, seconds=6.0, record=True))
    for s in (0.9, 1.1):
        A.append(sc_start("extended", "low", {"shoulder_lift": s, "elbow_flex": s, "wrist_flex": s}, seconds=6.0, record=True))
    groups["start_ok"] = A

    # B. starts that must abort: a sign error on one joint (the arm leaves the entry pose; back to the entry hold)
    B = []
    for pose in ("extended", "mid"):
        for j in L.COMP_JOINTS:
            for fric in ("low", "high"):
                B.append(sc_start(pose, fric, {j: -1.0}, seconds=4.0, name=f"sign error {j} at {pose} {fric}"))
    groups["start_sign"] = B

    # C. gross gain errors at the start
    C = []
    for s in (0.0, 0.5, 1.5, 2.0):
        for fric in ("low", "mid"):
            C.append(sc_start("extended", fric, {"shoulder_lift": s, "elbow_flex": s, "wrist_flex": s}, seconds=4.5,
                              name=f"gain {s} at extended {fric}"))
    groups["start_gain"] = C

    # D. feel of the lift: the hand drives it at 15 or 30 deg/s; lead c and the friction feed-forward (k_f) on or off
    D = []
    for fric in ("low", "mid", "high"):
        for w in (15, 30):
            for c, ff in ((0.0, False), (0.6, False), (0.6, True)):
                for period, P in ((0.01, 32), (0.0333, 32), (0.01, 16)):
                    if (period, P) != (0.01, 32) and (c != 0.6 or fric != "mid" or not ff):
                        continue
                    sc = S.Scenario(f"feel lift {w} deg/s {fric} c={c} {'FF 0.5' if ff else 'no FF'} host "
                                    f"{round(1 / period)} Hz P{P}", seconds=7.0, start_raw=WORK, gear=gears(fric),
                                    host_period=period, p_gain=P, cfg_over=comp_c(c), assist=ff)
                    sc.hand = SpeedHand("shoulder_lift", w, GUIDE_FROM, GUIDE_FROM + 2.2)
                    D.append(sc)
    groups["feel"] = D

    # D2. elbow and wrist feel, and the baselines: the same pushes in the ordinary Hold
    D2 = []
    for joint, w in (("elbow_flex", 30), ("wrist_flex", 45)):
        for fric in ("low", "mid"):
            for c, ff in ((0.0, False), (0.6, False), (0.6, True)):
                sc = S.Scenario(f"feel {joint} {w} deg/s {fric} c={c} {'FF 0.5' if ff else 'no FF'}", seconds=7.0,
                                start_raw=WORK, gear=gears(fric), cfg_over=comp_c(c), assist=ff)
                sc.hand = SpeedHand(joint, w, GUIDE_FROM, GUIDE_FROM + 2.2)
                if joint == "wrist_flex":
                    hold = [PdHand(o, 0.0, 1.0, 3.0, 7.0, k=40.0, b=2.0, tau_max=4.0) for o in ("shoulder_lift", "elbow_flex")]
                    sc.hand = combine(sc.hand, *hold)
                    sc.name += " (forearm held)"
                D2.append(sc)
    for fric in ("low", "mid"):
        for joint, w in (("shoulder_lift", 15), ("elbow_flex", 30)):
            sc = S.Scenario(f"reference hold: {joint} {w} deg/s {fric}", seconds=4.0, start_raw=WORK,
                            gear=gears(fric), hold_reference=True)
            sc.hand = SpeedHand(joint, w, 1.2, 4.0, tau_max=4.0)
            D2.append(sc)
    groups["feel_joints"] = D2

    # E. release after a push: it stops (no runaway, no oscillation) at the lead shares the design allows, with the
    #    feed-forward on; an over-tuned lead shows what the speed cap and friction do
    E = []
    for c in (0.6, 0.8, 1.0, 1.3):
        for fric in ("low", "mid"):
            sc = S.Scenario(f"release lift c={c} {fric} FF 0.5", seconds=8.0, start_raw=WORK, gear=gears(fric),
                            cfg_over=comp_c(c), params={"c_comp_max": 1.5}, record=True)
            sc.hand = SpeedHand("shoulder_lift", 30, GUIDE_FROM, 4.8)
            E.append(sc)
    groups["release"] = E

    # F. a sign error appearing mid-session (a bug, a wrong calibration loaded): caught, and the arm coasts
    F = []
    for pose in ("extended", "mid"):
        for fric in ("low", "mid", "high"):
            F.append(S.Scenario(f"runtime sign flip lift at {pose} {fric}", seconds=6.5, start_raw=POSES[pose],
                                gear=gears(fric), model_flip_at=4.5, record=True))
    F.append(S.Scenario("runtime sign flip elbow at extended low", seconds=6.5, start_raw=POSES["extended"], gear=gears("low"),
                        model_flip_at=4.5, model_flip_joint="elbow_flex", record=True))
    for fric in ("low", "mid", "high"):
        F.append(S.Scenario(f"runtime sign flip lift at mid {fric}, prior check OFF (physical backstops only)", seconds=6.5,
                            start_raw=POSES["mid"], gear=gears(fric), model_flip_at=4.5, record=True,
                            params={"prior_check": False}))
    F.append(S.Scenario("runtime sign flip elbow at extended low, prior check OFF", seconds=6.5, start_raw=POSES["extended"],
                        gear=gears("low"), model_flip_at=4.5, model_flip_joint="elbow_flex", record=True,
                        params={"prior_check": False}))
    groups["runtime_sign"] = F
    # F2. the servo's real P differs from the one the law assumes (P register not as read): Present_Load check
    F2 = []
    for fric in ("low", "mid"):
        F2.append(S.Scenario(f"servo P16 while the law assumes P32, mid pose {fric}", seconds=6.5, start_raw=POSES["mid"],
                             gear=gears(fric), p_gain=16, cfg_over={j: {"k_servo": 8.0} for j in L.COMP_JOINTS}))
    groups["p_mismatch"] = F2

    # G. a 1.7 s host stall: the servos hold their last goals meanwhile; on resume, coast
    G = []
    for fric in ("low", "mid"):
        G.append(S.Scenario(f"stall 1.7 s hands off {fric}", seconds=7.5, start_raw=POSES["extended"], gear=gears(fric),
                            stalls=[(4.5, 1.7)], record=True))
        sc = S.Scenario(f"stall 1.7 s, hand moves lift +10 deg, lets go in the stall {fric}", seconds=7.5,
                        start_raw=POSES["mid"], gear=gears(fric), stalls=[(4.5, 1.7)], record=True)
        sc.hand = PdHand("shoulder_lift", 10.0, 20.0, 4.6, 5.7)
        G.append(sc)
    groups["stall"] = G

    # H. Hold here (the preloaded hold) at the worst pose: sag; and a fault there (the E-stop drill): the drop
    H = []
    for fric in ("low", "mid"):
        for P in (32, 16):
            H.append(S.Scenario(f"Hold here at extended {fric} P{P}", seconds=6.5, start_raw=POSES["extended"],
                                gear=gears(fric), exit_at=4.5, p_gain=P, record=True))
        for pose in ("extended", "mid"):
            H.append(S.Scenario(f"fault coast (E-stop drill) hands off at {pose} {fric}", seconds=6.5, start_raw=POSES[pose],
                                gear=gears(fric), fault_at=4.5, record=True))
    groups["exit"] = H

    # I. speed cap: a yank on the lift at 90 deg/s (coast), a brisk 40 deg/s under the 45 cap (runs on)
    I = []
    for fric in ("low", "mid"):
        sc = S.Scenario(f"yank lift 90 deg/s {fric}", seconds=6.5, start_raw=WORK, gear=gears(fric))
        sc.hand = SpeedHand("shoulder_lift", 90, GUIDE_FROM, 5.5, kp=8.0, ki=30.0, tau_max=4.0)
        I.append(sc)
        sc = S.Scenario(f"brisk lift 40 deg/s (under the 45 cap) {fric}", seconds=6.5, start_raw=WORK, gear=gears(fric))
        sc.hand = SpeedHand("shoulder_lift", 40, GUIDE_FROM, 5.5)
        I.append(sc)
    groups["speed"] = I

    # J. soft limit: push the elbow past its upper limit (+90.3 deg) and let go
    J = []
    for fric in ("low", "mid"):
        sc = S.Scenario(f"elbow into its limit {fric}", seconds=8.5, start_raw=raw(-60.0, 70.0, 0.0), gear=gears(fric), record=True)
        sc.hand = PdHand("elbow_flex", 30.0, 12.0, GUIDE_FROM, 7.0, k=8.0, tau_max=2.5)
        J.append(sc)
    groups["limits"] = J

    # K. supply sag 8.9 -> 7.6 V during a hands-off hover, with and without voltage scaling of the duty
    K = []
    sag = lambda t: 8.9 - 1.3 * min(1.0, max(0.0, (t - 4.0) / 1.0))
    for comp in (True, False):
        for fric in ("low", "mid"):
            K.append(S.Scenario(f"supply sag to 7.6 V, voltage scaling {'on' if comp else 'off'} {fric}", seconds=8.5,
                                start_raw=POSES["extended"], gear=gears(fric), volts=sag, volt_comp=comp))
    for comp in (True, False):
        K.append(S.Scenario(f"supply sag to 7.6 V, voltage scaling {'on' if comp else 'off'} ultralow friction", seconds=8.5,
                            start_raw=POSES["extended"], gear=gears("ultralow"), volts=sag, volt_comp=comp, record=True))
    K.append(S.Scenario("supply 8.2 V at entry: refused", seconds=2.0, start_raw=POSES["extended"], gear=gears("mid"),
                        volts=lambda t: 8.2))
    K.append(S.Scenario("supply sags to 6.8 V while guiding: coast", seconds=7.0, start_raw=POSES["mid"], gear=gears("mid"),
                        volts=lambda t: 8.9 - 2.1 * min(1.0, max(0.0, (t - 4.5) / 0.5)), record=True))
    groups["supply"] = K

    # L. bus: random drops 1 % per reply (no false exit), a short burst (rides through), a long burst (coast)
    Lg = [S.Scenario("drops 1 % per reply, hover at extended, low friction", seconds=8.5, start_raw=POSES["extended"],
                     gear=gears("low"), drop_p=0.01)]
    for n in (3, 8):
        Lg.append(S.Scenario(f"burst of {n} missed lift replies", seconds=6.5, start_raw=POSES["extended"], gear=gears("low"),
                             drop_burst=(4.5, n, "shoulder_lift"), record=True))
    Lg.append(S.Scenario("burst of 8 missed wrist_roll replies", seconds=6.5, start_raw=POSES["extended"], gear=gears("low"),
                         drop_burst=(4.5, 8, "wrist_roll"), record=True))
    groups["bus"] = Lg

    # M. USB outages: guiding is a move in progress, so the drop latches; the servos keep their last goals while the
    #    port is gone and every joint coasts the moment it is back
    M = []
    for d in (0.3, 2.5):
        M.append(S.Scenario(f"USB outage {d} s hands off at extended", seconds=8.0, start_raw=POSES["extended"],
                            gear=gears("low"), port_loss=[(4.5, d)], record=True))
    sc = S.Scenario("USB outage 0.3 s while the hand moves the lift", seconds=7.0, start_raw=POSES["mid"], gear=gears("low"),
                    port_loss=[(4.5, 0.3)], record=True)
    sc.hand = PdHand("shoulder_lift", 8.0, 20.0, 4.4, 5.1)
    M.append(sc)
    sc = S.Scenario("USB outage 0.5 s while the hand turns the roll (assisted, 12 deg/s)", seconds=7.5, start_raw=POSES["mid"],
                    gear=gears("low", free="mid"), port_loss=[(4.5, 0.5)], record=True)
    sc.hand = SpeedHand("wrist_roll", 12, GUIDE_FROM, 4.8)
    M.append(sc)
    groups["usb"] = M

    # N. brown-out reset of the lift servo while hovering extended: caught, coast
    N = []
    for fric in ("low", "mid"):
        N.append(S.Scenario(f"lift servo brown-out reset at extended {fric}", seconds=6.5, start_raw=POSES["extended"],
                            gear=gears(fric), brownout=(4.5, "shoulder_lift"), record=True))
    sc = S.Scenario("lift brown-out after crossing encoder zero (frame jump)", seconds=10.0, start_raw=POSES["home"],
                    gear=gears("mid"), brownout=(8.5, "shoulder_lift"), record=True)
    sc.hand = PdHand("shoulder_lift", 100.0, 25.0, GUIDE_FROM, 7.9, k=10.0, tau_max=3.0)
    N.append(sc)
    groups["brownout"] = N

    # O. the lift's friction feed-forward: k_f 0.5 and 0.7 of the calibrated friction; the real friction lower than
    #    calibrated (x0.7: passes the probe; x0.4: the probe switches the feed-forward off; x0.4 with the probe off)
    O = []
    for k_f, scale, probe in ((0.5, 1.0, True), (0.7, 1.0, True), (0.5, 0.7, True), (0.7, 0.7, True), (0.5, 0.4, True),
                              (0.7, 0.4, True), (0.7, 0.4, False)):
        g = gears("mid")
        for j in L.COMP_JOINTS:
            g[j].f0 *= scale
            g[j].f_load *= scale
        sc = S.Scenario(f"lift FF k_f {k_f}, real friction x{scale} of calibrated{'' if probe else ', probe OFF'}",
                        seconds=8.0, start_raw=WORK, record=True, gear=g,
                        ident={j: {"f": 1.0 / scale} for j in L.COMP_JOINTS},
                        cfg_over={j: {"k_f": k_f} for j in L.COMP_JOINTS}, params={"probe": probe})
        sc.hand = SpeedHand("shoulder_lift", 20, GUIDE_FROM, 4.8)
        O.append(sc)
    groups["friction_ff"] = O

    # P. hover endurance: 20 s hands off at the worst pose, low friction (creep?)
    groups["hover"] = [S.Scenario("hover 20 s at extended, low friction", seconds=25.0, start_raw=POSES["extended"],
                                  gear=gears("low"), record=True),
                       S.Scenario("hover 20 s at extended, low friction, model 0.9", seconds=25.0, start_raw=POSES["extended"],
                                  gear=gears("low"), model_scale={"shoulder_lift": 0.9, "elbow_flex": 0.9, "wrist_flex": 0.9},
                                  record=True)]

    # Q. collision guard band: a floor 6 cm below the start tip height; the hand pushes the lift down into it
    Q = []
    from gravity_model import default_model
    m = default_model()

    def tip_z(th):
        q = {"shoulder_pan": 0.0, "shoulder_lift": th["shoulder_lift"], "elbow_flex": th["elbow_flex"],
             "wrist_flex": th["wrist_flex"], "wrist_roll": 0.0, "gripper": 0.0}
        return float(m.tcp(q)[2])
    start = POSES["mid"]
    z0 = tip_z({j: (start[j] - REF[j]) * S.STEP for j in L.COMP_JOINTS})
    for fric in ("low", "mid"):
        sc = S.Scenario(f"guard band: hand pushes the tip down into a floor band {fric}", seconds=8.5, start_raw=start,
                        gear=gears(fric), band=lambda th, z0=z0: tip_z(th) < z0 - 0.06, record=True)
        sc.hand = PdHand("shoulder_lift", 25.0, 10.0, GUIDE_FROM, 6.5, k=8.0, tau_max=2.0)
        sc.meta = {"band_edge_z": z0 - 0.06}
        Q.append(sc)
    groups["band"] = Q

    # ------------------------------------------------------------------------------ the assist on pan, roll, gripper
    # R. feel: the hand turns each free joint at a steady speed; plain Coast (torque off, no guiding) vs hand guiding
    R = []
    speeds = {"shoulder_pan": (10, 30, 50, 80), "wrist_roll": (10, 30, 60, 120), "gripper": (10, 30, 60, 120)}
    for joint in L.FREE_JOINTS:
        for fric in ("low", "mid", "high"):
            for w in speeds[joint]:
                push = min(2.5, 90.0 / w)
                for mode in ("coast", "assist"):
                    sc = S.Scenario(f"free feel {joint} {w} deg/s {fric} {mode}", seconds=GUIDE_FROM + push + 0.2,
                                    start_raw=WORK, gear=gears("mid", free=fric), coast_reference=mode == "coast", record=True)
                    sc.hand = SpeedHand(joint, w, GUIDE_FROM, GUIDE_FROM + push, tau_max=2.0)
                    sc.meta = {"joint": joint, "fric": fric, "speed": w, "mode": mode}
                    R.append(sc)
    groups["free_feel"] = R

    # R1. why the band is computed: the roll forced energised at every speed (v_top_fixed), with and without the lead
    #     and the feed-forward, against plain Coast; gives the drag of the energised motor and of the goal scheme
    R1 = []
    for w in (10, 20, 30, 60):
        for label, over, coast in (("coast", {}, True),
                                   ("energised c=0 no FF", {"c_lead": 0.0, "k_f": 0.0}, False),
                                   ("energised c=0.9 no FF", {"c_lead": 0.9, "k_f": 0.0}, False),
                                   ("energised c=0.9 FF 0.5", {"c_lead": 0.9, "k_f": 0.5}, False),
                                   ("energised c=0.9 FF 0.7", {"c_lead": 0.9, "k_f": 0.7}, False)):
            push = min(2.0, 90.0 / w)
            sc = S.Scenario(f"free drag wrist_roll {w} deg/s mid {label}", seconds=GUIDE_FROM + push + 0.2, start_raw=WORK,
                            gear=gears("mid", free="mid"), coast_reference=coast,
                            cfg_over={"wrist_roll": {**over, "v_top_fixed": 200 * DEG, "v_hi": 120 * DEG,
                                                     "a_on": 1e6, "a_off": 1e6, "steady_s": 0.0}})
            sc.hand = SpeedHand("wrist_roll", w, GUIDE_FROM, GUIDE_FROM + push, tau_max=2.0)
            sc.meta = {"joint": "wrist_roll", "speed": w, "mode": label}
            R1.append(sc)
    groups["free_drag"] = R1

    # R1b. if bring-up finds that torque off BRAKES the motor (S2), Coast drags 0.57 N m s/rad and the band runs to v_hi
    R1b = []
    for joint in ("wrist_roll", "gripper"):
        for w in (10, 30, 60):
            for mode in ("coast", "assist"):
                push = min(2.5, 90.0 / w)
                sc = S.Scenario(f"braking driver: free feel {joint} {w} deg/s mid {mode}", seconds=GUIDE_FROM + push + 0.2,
                                start_raw=WORK, gear=gears("mid", free="mid"), coast_reference=mode == "coast",
                                coast_brakes=True, record=True)
                sc.hand = SpeedHand(joint, w, GUIDE_FROM, GUIDE_FROM + push, tau_max=3.0)
                sc.meta = {"joint": joint, "fric": "mid", "speed": w, "mode": mode, "coast_brakes": True}
                R1b.append(sc)
    groups["free_brake"] = R1b

    # R2. quick moves (the inertia the hand feels): 30 deg in 0.4 s (peak 140 deg/s) and in 1.0 s (peak 56 deg/s)
    R2 = []
    for joint in L.FREE_JOINTS:
        for T in (0.4, 1.0):
            for mode in ("coast", "assist"):
                sc = S.Scenario(f"free flick {joint} 30 deg in {T} s mid {mode}", seconds=GUIDE_FROM + T + 1.0,
                                start_raw=WORK, gear=gears("mid", free="mid"), coast_reference=mode == "coast", record=True)
                sc.hand = FlickHand(joint, 30.0, T, GUIDE_FROM)
                sc.meta = {"joint": joint, "T": T, "mode": mode}
                R2.append(sc)
    groups["free_flick"] = R2

    # R3. release: the hand turns the joint at 30 deg/s (and the roll and gripper at 80 deg/s) for 1.2 s and lets go:
    #     how far it glides, how soon it stops, and that it stays stopped (no creep) for 3 s
    R3 = []
    for joint in L.FREE_JOINTS:
        for fric in ("low", "mid", "high"):
            for w in ((15, 30, 80) if joint != "shoulder_pan" else (15, 30, 50)):
                for mode in ("coast", "assist"):
                    if mode == "coast" and fric != "low":
                        continue
                    sc = S.Scenario(f"free release {joint} from {w} deg/s {fric} {mode}", seconds=GUIDE_FROM + 1.2 + 3.0,
                                    start_raw=WORK, gear=gears("mid", free=fric), coast_reference=mode == "coast", record=True)
                    sc.hand = SpeedHand(joint, w, GUIDE_FROM, GUIDE_FROM + 1.2, tau_max=2.0)
                    sc.meta = {"joint": joint, "fric": fric, "speed": w, "mode": mode}
                    R3.append(sc)
    # every joint at once: the hand swings the pan while turning the roll and opening the gripper, then lets go
    sc = S.Scenario("free release: pan, roll and gripper together from 30 deg/s, low friction", seconds=GUIDE_FROM + 1.2 + 3.0,
                    start_raw=WORK, gear=gears("low", free="low"), record=True)
    sc.hand = combine(*[SpeedHand(j, 30, GUIDE_FROM, GUIDE_FROM + 1.2, tau_max=2.0) for j in L.FREE_JOINTS])
    R3.append(sc)
    groups["free_release"] = R3

    # R4. assist settings that are wrong
    R4 = []
    for what, over in (("k_f 0.9 on the roll", {"wrist_roll": {"k_f": 0.9}}),
                       ("lead c 1.1 on the roll", {"wrist_roll": {"c_lead": 1.1}}),
                       ("lead c 0.95 on the lift", {"shoulder_lift": {"c_lead": 0.95}}),
                       ("velocity slope 1.5 x the motor's on the pan", {"shoulder_pan": {"k_v": 1.5 * 274.0}}),
                       ("assist top speed 200 deg/s on the gripper", {"gripper": {"v_hi": 200 * DEG}})):
        R4.append(S.Scenario(f"settings refused: {what}", seconds=2.0, start_raw=WORK, gear=gears("mid"), cfg_over=over))
    # the real friction is a fraction of what the calibration measured, so the feed-forward (k_f 0.5 and 0.7 of the
    # calibrated friction) is MORE than the real friction: the hand moves the joint steadily in the band at 15 deg/s
    # and lets go. The joint must not run on: past the band's top it floats, and a stroke ends after 3 s
    for joint in L.FREE_JOINTS:
        for share, k_f in ((0.4, 0.5), (0.25, 0.7)):
            g = FREE_FRICTION["mid"]
            sc = S.Scenario(f"{joint} friction x{share} of calibrated, k_f {k_f} ({k_f / share:.2f} x the real friction): "
                            f"release from 15 deg/s", seconds=GUIDE_FROM + 2.0 + 6.0, start_raw=WORK, record=True,
                            gear={**gears("mid"), **{joint: S.free_gear(f0=g["f0"] * share, f_load=g["f_load"] * share)}},
                            ident={joint: {"f": 1.0 / share}}, cfg_over={joint: {"k_f": k_f}})
            sc.hand = SpeedHand(joint, 15, GUIDE_FROM, GUIDE_FROM + 2.0, tau_max=2.0)
            sc.meta = {"joint": joint}
            R4.append(sc)
    # the most the bounds allow: k_f 0.7, c 0.9, k_v at the 1.10 x prior limit (1.06 x the plant's true slope); and the
    # same with validation bypassed and k_v 1.25 x true (lead 1.12 x the whole drag: negative damping)
    for joint in ("wrist_roll", "shoulder_pan", "gripper"):
        for kv, bypass in ((1.09 * L.kv_prior(8.9) / (L.kv_prior(8.9) + 0.02 / L.kd_nm_per_unit(8.9)), False), (1.25, True)):
            sc = S.Scenario(f"{joint} at the bounds: k_f 0.7, c 0.9, k_v {kv:.2f} x true{', validation bypassed' if bypass else ''}"
                            f", mid friction: release from 15 deg/s", seconds=GUIDE_FROM + 2.0 + 6.0, start_raw=WORK,
                            gear=gears("mid", free="mid"), record=True, ident={joint: {"kv": kv}},
                            cfg_over={joint: {"k_f": 0.7, "c_lead": 0.9}},
                            params={"kv_prior_band": (0.5, 2.0)} if bypass else {})
            sc.hand = SpeedHand(joint, 15, GUIDE_FROM, GUIDE_FROM + 2.0, tau_max=2.0)
            sc.meta = {"joint": joint}
            R4.append(sc)
    # the lift with its friction at 40 % of calibrated: the probe switches its feed-forward off
    g = gears("mid")
    for j in L.COMP_JOINTS:
        g[j].f0 *= 0.4
        g[j].f_load *= 0.4
    for probe in (True, False):
        sc = S.Scenario(f"lift, elbow, wrist friction x0.4 of calibrated, k_f 0.7, probe {'on' if probe else 'OFF'}: "
                        f"release from 20 deg/s", seconds=8.0, start_raw=WORK, gear=g, record=True,
                        ident={j: {"f": 1.0 / 0.4} for j in L.COMP_JOINTS}, cfg_over={j: {"k_f": 0.7} for j in L.COMP_JOINTS},
                        params={"probe": probe})
        sc.hand = SpeedHand("shoulder_lift", 20, GUIDE_FROM, 4.8)
        R4.append(sc)
    groups["assist_params"] = R4

    # R5. faults coast (go limp): what the arm does hands off, and with a hand supporting it
    R5 = []
    R5.append(S.Scenario("wrist_roll stops answering while hovering at mid: coast", seconds=6.5, start_raw=POSES["mid"],
                         gear=gears("mid"), drop_burst=(4.5, 30, "wrist_roll"), record=True))
    R5.append(S.Scenario("lift stops answering while hovering at mid: coast (the lift keeps its goal until it answers)",
                         seconds=6.5, start_raw=POSES["mid"], gear=gears("mid"), drop_burst=(4.5, 60, "shoulder_lift"),
                         record=True))
    R5.append(S.Scenario("lift at 55 C while hovering at mid: coast", seconds=6.5, start_raw=POSES["mid"], gear=gears("mid"),
                         heat_at=(4.5, "shoulder_lift", 26.0), record=True))
    for fric in ("low", "mid"):
        sc = S.Scenario(f"fault coast at mid {fric} with a hand supporting the forearm", seconds=6.5, start_raw=POSES["mid"],
                        gear=gears(fric), fault_at=4.5, record=True)
        sc.hand = combine(*[PdHand(o, 0.0, 1.0, 4.0, 6.5, k=8.0, b=0.8, tau_max=3.0) for o in L.COMP_JOINTS])
        R5.append(sc)
    sc = S.Scenario("fault coast while the hand turns the roll (assisted, 12 deg/s)", seconds=7.5, start_raw=POSES["mid"],
                    gear=gears("mid"), fault_at=4.5, record=True)
    sc.hand = SpeedHand("wrist_roll", 12, GUIDE_FROM, 4.8)
    R5.append(sc)
    groups["faults_coast"] = R5

    # R6. free-joint walls: the gripper closed into its slider limit, the roll turned into its limit
    R6 = []
    for fric in ("low", "mid"):
        sc = S.Scenario(f"gripper closed into its limit {fric}", seconds=8.0, start_raw={**WORK, **free_raw("gripper", 15.0)},
                        gear=gears("mid", free=fric), record=True)
        sc.hand = PdHand("gripper", -30.0, 20.0, GUIDE_FROM, 6.5, k=0.6, b=0.03, tau_max=0.4)
        R6.append(sc)
        sc = S.Scenario(f"roll turned into its limit {fric}", seconds=8.0, start_raw={**WORK, **free_raw("wrist_roll", 150.0)},
                        gear=gears("mid", free=fric), record=True)
        sc.hand = PdHand("wrist_roll", 30.0, 20.0, GUIDE_FROM, 6.5, k=0.6, b=0.03, tau_max=0.4)
        R6.append(sc)
    groups["free_walls"] = R6
    return groups


def window(sim, t_from, t_to, col):
    return [r[col] for r in sim.log if t_from <= r[0] <= t_to]


def metrics(sc: S.Scenario):
    """Per-scenario extra numbers from the simulation object."""
    def f(sim: S.ArmSim):
        out = {}
        hand = sc.hand
        if hasattr(hand, "parts"):
            hand = next((x for x in hand.parts if isinstance(x, SpeedHand)), hand)
        if isinstance(hand, SpeedHand):
            out["hand_force_N"] = round(hand.mean_force(), 2)
            out["hand_torque_Nm"] = round(hand.mean_force() * LEVER[hand.j], 4)
            out["breakaway_N"] = round(hand.breakaway / LEVER[hand.j], 2)
            out["breakaway_Nm"] = round(hand.breakaway, 4)
            out["mean_speed_dps"] = round(math.degrees(hand.mean_speed()), 1)
            if sim.log and hand.j in L.FREE_JOINTS and not sc.coast_reference:
                # the force while the joint was actually assisted (0.2 s after each switch-on), and how much of the push
                # (after its first 0.5 s) was assisted
                ct, ch = S.COL_TORQUE[hand.j], S.COL_HAND[hand.j]
                rows = [r for r in sim.log if hand.t0 + 0.5 <= r[0] < hand.t1]
                on_since, vals, n_on = None, [], 0
                for r in sim.log:
                    if r[0] >= hand.t1:
                        break
                    if r[ct]:
                        on_since = r[0] if on_since is None else on_since
                    else:
                        on_since = None
                    if r[0] >= hand.t0 + 0.5:
                        n_on += 1 if r[ct] else 0
                        if r[ct] and r[0] - on_since >= 0.2:
                            vals.append(r[ch])
                out["assisted_share"] = round(n_on / max(1, len(rows)), 2)
                if vals:
                    out["assisted_force_N"] = round(abs(sum(vals) / len(vals)) / LEVER[hand.j], 2)
            if sim.log and hand.t1 < sc.seconds - 0.5:
                c = S.COL_DEG[hand.j]
                at = [r for r in sim.log if r[0] >= hand.t1]
                if at:
                    th0 = at[0][c]
                    out["after_release_deg"] = round(at[-1][c] - th0, 2)
                    # stop time: the first moment after which the joint never moves more than 0.2 deg again
                    t_stop = None
                    for k in range(len(at)):
                        rest = [r[c] for r in at[k:]]
                        if max(rest) - min(rest) <= 0.2:
                            t_stop = at[k][0] - hand.t1
                            break
                    out["stop_s"] = None if t_stop is None else round(t_stop, 2)
                    late = [r for r in at if r[0] >= sc.seconds - 2.0]
                    if len(late) >= 2:
                        out["creep_last_2s_deg"] = round(late[-1][c] - late[0][c], 3)
                late = [r for r in sim.log if r[0] >= sc.seconds - 0.3]
                if len(late) >= 2:
                    out["end_speed_dps"] = round((late[-1][c] - late[0][c]) / (late[-1][0] - late[0][0]), 1)
        if isinstance(sc.hand, FlickHand):
            out["flick_peak_Nm"] = round(sc.hand.peak, 4)
            out["flick_rms_Nm"] = round(math.sqrt(sc.hand.sq / max(1, sc.hand.n)), 4)
            out["flick_peak_N"] = round(sc.hand.peak / LEVER[sc.hand.j], 2)
            out["flick_track_err_deg"] = round(math.degrees(sc.hand.track_err), 2)
        if isinstance(sc.hand, PdHand):
            out["hand_max_torque_Nm"] = round(sc.hand.max_tau, 3)
            if sim.log and ("limit" in sc.name):
                from gravity_model import RawMap
                lo, hi = (math.degrees(x) for x in RawMap().urdf_limits_rad(sc.hand.j))
                c = S.COL_DEG[sc.hand.j]
                past = [max(r[c] - hi, lo - r[c], 0.0) for r in sim.log if r[0] >= GUIDE_FROM]
                out["past_limit_max_deg"] = round(max(past), 2)
                out["past_limit_end_deg"] = round(past[-1], 2)
                inside = [min(hi - r[c], r[c] - lo) for r in sim.log if r[0] >= sc.hand.tr]
                out["after_release_inside_limit_deg"] = round(inside[-1], 2) if inside else None
        meta = getattr(sc, "meta", None) or {}
        if sim.log and "band_edge_z" in meta:
            from gravity_model import default_model
            m = default_model()
            depth = []
            for r in sim.log[::5]:
                if r[0] < GUIDE_FROM:
                    continue
                q = {"shoulder_pan": 0.0, "wrist_roll": 0.0, "gripper": 0.0,
                     **{j: math.radians(r[S.COL_DEG[j]]) for j in L.COMP_JOINTS}}
                depth.append((r[0], (meta["band_edge_z"] - float(m.tcp(q)[2])) * 100))
            out["band_depth_max_cm"] = round(max(d for _, d in depth), 2)
            out["band_depth_end_cm"] = round(depth[-1][1], 2)
        if sim.log:
            t_ref = (sc.exit_at or sc.fault_at or sc.model_flip_at or (sc.stalls[0][0] if sc.stalls else None)
                     or (sc.brownout[0] if sc.brownout else None) or (sc.port_loss[0][0] if sc.port_loss else None)
                     or (sc.drop_burst[0] if sc.drop_burst else None) or (sc.heat_at[0] if sc.heat_at else None))
            if t_ref:
                before = [r for r in sim.log if r[0] <= t_ref]
                after = [r for r in sim.log if r[0] > t_ref]
                if before and after:
                    b = before[-1]
                    out["moved_after_deg"] = {j: round(max(abs(r[S.COL_DEG[j]] - b[S.COL_DEG[j]]) for r in after), 2)
                                              for j in L.COMP_JOINTS}
                    out["end_minus_ref_deg"] = {j: round(after[-1][S.COL_DEG[j]] - b[S.COL_DEG[j]], 2) for j in L.COMP_JOINTS}
            if sim.exit_kind == "coast" and sim.exit_t is not None:
                at = [r for r in sim.log if r[0] >= sim.exit_t]
                if at:
                    a0 = at[0]
                    for span in (0.25, 0.5, 1.0):
                        seg = [r for r in at if r[0] <= sim.exit_t + span]
                        out[f"drop_{span}s_deg"] = {j: round(max(abs(r[S.COL_DEG[j]] - a0[S.COL_DEG[j]]) for r in seg), 1)
                                                    for j in L.COMP_JOINTS}
                    out["torque_after_exit"] = {j: at[-1][S.COL_TORQUE[j]] for j in L.ALL_JOINTS}
            # hover drift after the start (hands off), for the start and hover groups
            if sc.hand is None and not sc.exit_at and not sc.fault_at and sim.law is not None:
                g0 = sc.enter_at + 2.4
                seg = [r for r in sim.log if r[0] >= g0]
                if seg and (sim.exit_t is None or sim.exit_t > sc.seconds - 0.05):
                    out["hover_drift_deg"] = {j: round(max(abs(r[S.COL_DEG[j]] - seg[0][S.COL_DEG[j]]) for r in seg), 2)
                                              for j in L.ALL_JOINTS}
        return out
    return f


def run_group(args):
    name, idx = args
    groups = build()
    sc = groups[name][idx]
    try:
        r = run_one(sc, metrics(sc))
    except Exception as Exc:  # a scenario bug must not hide the others
        import traceback
        r = {"name": sc.name, "error": f"{Exc}", "trace": traceback.format_exc()[-1500:]}
    r["group"] = name
    r["meta"] = getattr(sc, "meta", None)
    return r


def main():
    argv = sys.argv[1:]
    workers = min(24, max(1, (os.cpu_count() or 4) - 4))
    if "--workers" in argv:
        k = argv.index("--workers")
        workers = int(argv[k + 1])
        argv = argv[:k] + argv[k + 2:]
    groups = build()
    pick = argv or list(groups)
    jobs = [(g, i) for g in pick for i in range(len(groups[g]))]
    print(f"{len(jobs)} scenarios, {workers} workers", flush=True)
    results = []
    t0 = time.perf_counter()
    with ProcessPoolExecutor(max_workers=workers) as ex:
        for r in ex.map(run_group, jobs):
            results.append(r)
            short = {k: r.get(k) for k in ("exit", "exit_t", "hand_force_N", "breakaway_N", "after_release_deg", "stop_s",
                                            "creep_last_2s_deg", "flick_peak_N", "drop_0.5s_deg", "moved_after_deg",
                                            ) if r.get(k) is not None}
            off = [j for j, v in (r.get("ff_on") or {}).items() if not v]
            if off:
                short["ff_off"] = off
            print(f"[{r['group']}] {r['name']}: {(r.get('reason') or r.get('error', ''))[:90]} | {short}", flush=True)
    out = Path(__file__).resolve().parent / ("out_guide_scenarios.json" if not argv else f"out_guide_{'_'.join(pick)}.json")
    out.write_text(json.dumps(results, indent=1, default=str), encoding="utf-8")
    print(f"wrote {out} in {time.perf_counter() - t0:.0f} s")


if __name__ == "__main__":
    main()
