"""The hand-guiding law and supervisor the bridge runs (docs/HandGuide.md): gravity compensation plus a free-motion assist.

The bridge port of HandGuideSim/hand_guide_law.py, kept line for line with it (same names, so the offline plant in
HandGuideSim/arm_plant_sim.py drives either; HandGuidePlant.py and TestHandGuideLaw.py run the plant on this file).
What the port adds: the ReferenceRaw of the arm's own CadAlignment per law (reference_raw), and the supply spread, supply
against V_cal and temperature checks of the entry (begin). Since the 2026-10-10 safety review it also:
  * acts on a bad reading only once it persists: firmware status bits, torque or Lock read back, the supply under
    7.0 V (the lowest of all six servos) need persist_reads (2) readings in a row (the host reads the status registers
    again on the next cycle: status_recheck), heat temp_exit_hold_s; a position that implies more than 3 x the joint's
    speed cap is a missed reply (that joint keeps its last goal), and two in a row end guiding. One corrupted reply
    (2026-10-10: torque off, 0 V, 90 C and load 0xF501 in one packet, the servo fine) no longer drops the arm;
  * is frame independent: a reading about a turn from the last is the servo's frame wrapping (single-turn reporting
    through raw 0, or a restart into another turn), unwrapped; every position the law keeps is in its own continuous
    frame, and every goal it writes is the servo's reported position plus an offset, never an absolute goal carried
    across a wrap;
  * watches a joint it leaves alone (the gripper while it squeezes an object, "keep"): its replies, status bits,
    torque, Lock, heat and supply end guiding like the others';
  * arms the walls from the entry pose (a start inside the collision guard band holds there).
The calibration file, its fit and building a law from it are HandGuideCalibration.py; the bus is GuideBus.py; the guide
loop is BridgeController's (bridge_server.py).

Pure logic: no serial port, no bridge, no clock of its own. The guide loop calls it once per host cycle with that
cycle's servo readings, and it answers with the registers to write.

Actuation: POSITION MODE, no operating-mode switch. Each STS3215 keeps running its own position loop at about
1.4 kHz; the host turns an effort u (duty units = 0.1 % of full duty, + = toward increasing raw ticks) into a goal a
little past the measured position:  goal = q + steps(u),  steps(u) = sign(u) ((|u| - punch) / K + dz), with
K = P/4 duty units per step (reg 21), punch = Minimum_Startup_Force (reg 24), dz = dead zone (regs 26/27). The
firmware then applies duty = K (goal - q -+ dz) + punch, clamped to +-Torque_Limit (reg 48, written with the goal
every cycle as a cap). A host that stops leaves each energised servo holding its last goal at that cap.

Joints (Aaran's decisions of 2026-10-10: everything moves freely, with a little help):
  * lift, elbow, wrist_flex ("comp"): always energised; u = gravity + lead + friction feed-forward (+ brakes).
  * pan, wrist_roll, gripper ("free"): torque OFF (floating, exactly Coast) at rest, when moved quickly and above the
    assist band; while the hand moves them slowly and steadily inside the band they are energised with an assist
    u = lead + friction feed-forward. The band's top is computed (band_top): an energised motor brakes with its
    back-EMF, so the assist beats floating only while the feed-forward outweighs the drag the lead leaves.
  The assist never self-propels: the feed-forward is at most 0.7 (default 0.5) of the identified kinetic friction it
  cancels, it is zero inside a velocity deadband, it fades out at speed, one stroke of help lasts 2 s (comp: the
  feed-forward fades; free: the joint floats until it has stopped), the lead cancels less than all of the back-EMF
  drag (c < 1: passive whatever the delay), the start probes the comp joints with 1.4 x their feed-forward (a joint
  that moves under the probe gets none), and a free joint above its band floats.

Phases: PRELOAD (the effort at the entry hold moves from the hold's own error to the model; anchor stiff) -> RAMP
(the anchor spring to the entry pose fades out; any movement aborts) -> PROBE (the comp joints pushed with +-1.4 x
their friction feed-forward: a joint that moves loses its feed-forward) -> GUIDE.
Exits: "preloaded" hold (Hold here / Stop and hold / Hold / AI mode / page gone: goal = q + model effort, normal
limit), "anchor" (a failed start: back to the entry hold goals; the arm was never handed over), "refused" (entry
refused before any write), "coast" (EVERY fault while guiding: torque off on every joint, the bridge's AutoCoast).
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, Optional

STEP_RAD = 2.0 * math.pi / 4096.0
COMP_JOINTS = ("shoulder_lift", "elbow_flex", "wrist_flex")
FREE_JOINTS = ("shoulder_pan", "wrist_roll", "gripper")
ALL_JOINTS = COMP_JOINTS + FREE_JOINTS
REFERENCE_RAW = {"shoulder_pan": 3453, "shoulder_lift": 256, "elbow_flex": 1671, "wrist_flex": 605,
                 "wrist_roll": 2588, "gripper": 700}
UNLOAD_BITS = (1 << 2) | (1 << 3) | (1 << 5)   # overheat, overcurrent, overload (FAULT_UNLOAD_BITS)
DEG = math.pi / 180.0

# Motor: 12 V-class STS3215 (C018/C047), output-referred, datasheet derived (DerivedNumbers.py)
KT = 11.0 * 0.0980665              # N*m/A
R_OHM = 12.0 / 2.7                 # ohm
KE = (12.0 - 0.20 * R_OHM) / (45.0 * 2 * math.pi / 60)   # V/(rad/s)
URDF_P0 = (-0.085899, -0.402063, -0.335500, -0.012385, -0.117011, -0.002459)


def kd_nm_per_unit(volts: float) -> float:
    """Stall torque per duty unit (0.1 %) at this supply: kt * V / (1000 R)."""
    return KT * volts / (1000.0 * R_OHM)


def kv_prior(volts: float) -> float:
    """Back-EMF duty per rad/s at the output (what a driven motor needs to keep its speed): ke / V * 1000."""
    return KE / volts * 1000.0


def urdf_rad(joint: str, raw: float) -> float:
    d = (raw - REFERENCE_RAW[joint] + 2048.0) % 4096.0 - 2048.0
    return d * STEP_RAD


def wrap_steps(d: float) -> float:
    """A step difference the short way round one turn (-2048..2048): readings a turn apart are the same angle."""
    return (d + 2048.0) % 4096.0 - 2048.0


def effort_steps(u: float, k_servo: float, punch: float, dead_zone: float) -> float:
    """steps(u): how far past the measured position a goal makes the servo's position loop apply effort u (duty units):
    sign(u) ((|u| - punch) / K + dead zone), none for an effort under half the punch."""
    if abs(u) < 0.5 * punch:
        return 0.0
    return math.copysign(max(abs(u) - punch, 0.0) / k_servo + dead_zone, u)


def ramp01(x: float, a: float, b: float) -> float:
    """0 at or below a, 1 at or above b, linear between."""
    if x <= a:
        return 0.0
    if x >= b:
        return 1.0
    return (x - a) / (b - a)


@dataclass
class GravityModel:
    """g(q) in N*m (holding torque, + = toward increasing raw) for lift, elbow, wrist_flex.

    Closed form exact for the planar chain (gravity_comp_model.py): six constants [A2 B2 A3 B3 A4 B4]. The
    identification fits them (in duty units at V_cal) from up/down load sweeps; scale scales a joint (tests)."""
    p: tuple = URDF_P0
    scale: Dict[str, float] = field(default_factory=lambda: {j: 1.0 for j in COMP_JOINTS})

    def torques(self, q2: float, q3: float, q4: float) -> Dict[str, float]:
        A2, B2, A3, B3, A4, B4 = self.p
        T2, T3, T4 = q2, q2 + q3, q2 + q3 + q4
        w = A4 * math.cos(T4) + B4 * math.sin(T4)
        e = A3 * math.cos(T3) + B3 * math.sin(T3) + w
        l = A2 * math.cos(T2) + B2 * math.sin(T2) + e
        return {"shoulder_lift": l * self.scale["shoulder_lift"], "elbow_flex": e * self.scale["elbow_flex"],
                "wrist_flex": w * self.scale["wrist_flex"]}


@dataclass
class JointConfig:
    """A gravity-compensated joint (lift, elbow, wrist_flex)."""
    name: str
    k_servo: float = 8.0          # P/4 duty units per step (P read from reg 21 at entry; 32 -> 8)
    punch: float = 16.0           # reg 24
    dead_zone: float = 1.0        # regs 26/27
    cap_min: float = 150.0        # Torque_Limit floor while guiding
    cap_max: float = 650.0        # Torque_Limit ceiling while guiding (well under the 800 overload threshold)
    cap_margin: float = 150.0     # headroom of the dynamic cap above the commanded effort
    k_v: float = 274.0            # identified velocity duty slope at V_cal (back-EMF + viscous), duty units per rad/s
    c_lead: float = 0.6           # share of k_v the lead cancels (0..0.8; 0.6 default)
    v_cap: float = math.radians(45)   # joint speed cap, rad/s
    lim_lo: float = -math.inf     # soft limits, URDF rad (continuous)
    lim_hi: float = math.inf
    limit_zone: float = math.radians(6)  # resistance starts this far inside a limit
    # friction feed-forward (the "little help"): identified kinetic friction F_k(q) = f_kin0 + f_kin_load |gravity|
    f_kin0: float = 0.0           # duty units at V_cal, the unloaded part (0 = not identified: no feed-forward)
    f_kin_load: float = 0.0       # share of the gravity effort the gear adds to the friction
    k_f: float = 0.5              # share of F_k the feed-forward cancels (0..0.7)
    v_db: float = 2.0 * DEG       # no feed-forward below this speed (a joint at rest gets none)
    v_full: float = 6.0 * DEG     # full feed-forward from this speed


@dataclass
class FreeConfig:
    """A joint without gravity load (pan, wrist_roll, gripper): floating at rest, assisted while the hand moves it."""
    name: str
    k_servo: float = 8.0
    punch: float = 16.0
    dead_zone: float = 1.0
    k_v: float = 274.0            # identified velocity duty slope at V_cal (back-EMF + viscous), duty units per rad/s
    c_lead: float = 0.9           # share of k_v the lead cancels (0..0.9): the back-EMF drag of an energised motor
    f_kin: float = 0.0            # identified kinetic friction, duty units at V_cal (0 = not identified: no assist)
    k_f: float = 0.5              # share of f_kin the feed-forward cancels (0..0.7)
    coast_brakes: bool = False    # identified: torque off brakes the motor (True) or lets it float (False, expected)
    k_loop: float = 2.0           # drag of the per-cycle goal scheme itself, duty units per rad/s (plant: 1.5)
    v_db: float = 2.0 * DEG
    v_full: float = 6.0 * DEG
    v_on: float = 4.0 * DEG       # a floating joint the hand moves steadily faster than this is energised...
    start_steps: float = 12.0     # ...once it has moved more than the gear's backlash (0.87 deg = 10 steps)...
    steady_s: float = 0.15        # ...and its speed has stayed in the band this long (a flick passes through faster)
    v_off: float = 2.0 * DEG      # an assisted joint slower than this for rest_s floats again
    rest_s: float = 0.12
    v_hi: float = 90.0 * DEG      # the assist's top speed at most (the band's top is computed: band_top())
    v_hi_back: float = 0.85       # share of the band's top below which a fast floating joint may be energised again
    v_top_fixed: float = 0.0      # tests only: force the band's top (rad/s), whatever the assist is worth
    a_on: float = 60.0 * DEG      # energised only while the hand moves it steadily: |acceleration| below this...
    a_off: float = 150.0 * DEG    # ...and floating again above this (the lead lags 25-30 ms: while the hand speeds a
                                  # joint up, an energised motor drags more than a floating one)
    band_min: float = 3.0         # a band whose top would be under band_min x v_on is not worth it: the joint floats
    cap_min: float = 60.0
    cap_margin: float = 60.0
    cap_max: float = 400.0        # Torque_Limit ceiling of the assist (half the 800 overload threshold)
    lim_lo: float = -math.inf
    lim_hi: float = math.inf
    limit_zone: float = math.radians(6)
    catch_limit: float = 400.0    # Torque_Limit when caught at a wall or a slider limit


FREE_DEFAULTS = {"shoulder_pan": dict(v_hi=60.0 * DEG, cap_max=350.0),
                 "wrist_roll": dict(v_hi=90.0 * DEG, cap_max=400.0),
                 "gripper": dict(v_hi=90.0 * DEG, cap_max=400.0)}


@dataclass
class GuideParams:
    alpha: float = 0.95           # model gain while guiding (errors sink slowly rather than rise; 0.9-1.0 from the hover test)
    alpha_hold: float = 1.0       # model gain of the preloaded exit hold
    v_cal: float = 8.9            # supply voltage the duty-unit model was identified at
    preload_s: float = 0.6
    ramp_s: float = 1.2
    ramp_dev_steps: float = 24.0  # a start whose arm leaves the entry pose by more than this aborts (2.1 deg)
    ramp_speed: float = math.radians(8)
    probe: bool = True            # the start's friction probe (off only in tests of what the other guards do)
    probe_s: float = 0.15         # each direction (ramped)
    probe_settle_s: float = 0.1
    probe_gain: float = 1.4       # the probe pushes 1.4 x the feed-forward: kinetic friction is at least 0.7 x static
    probe_steps: float = 3.0      # a joint that moves more than this under the probe gets no feed-forward
    lag_share: float = 0.8        # share of the sample-and-hold lag the goal prediction cancels
    fall_accel: float = 10.0      # rad/s^2 toward gravity (with fall_accel_speed) that ends guiding (coast)
    fall_accel_speed: float = math.radians(12)
    fall_min_effort: float = 60.0 # duty units: below this the model carries too little for a fall to be its fault
    fall_cycles: int = 3          # ...on this many cycles in a row: a gear springing back when a hand lets go of a
                                  # self-locking joint lasts one or two (a false fall now drops the arm)
    prior_check: bool = True      # the identified model must agree with the URDF prior every cycle
    prior_ratio: tuple = (0.5, 1.8)
    prior_floor: float = 40.0     # duty units: above this the ratio applies, below it the absolute difference
    prior_abs: float = 60.0
    brake_gain: float = 600.0     # duty units per rad/s above the soft speed
    soft_speed_share: float = 0.7
    overspeed_s: float = 0.06
    overspeed_hard_share: float = 1.15   # over this share of the cap on 2 cycles in a row: at once
    stall_exit_s: float = 0.15    # a cycle this late ends guiding (coast; during the start: the entry hold)
    miss_limit: int = 5           # consecutive missed readings of one joint -> coast
    # A position reading further from the last one than jump_speed_share x the joint's speed cap allows (plus
    # jump_slack_steps) is a missed reply: that joint keeps its last goal. Two in a row end guiding (a restart?). A
    # free joint's cap here is free_speed_max, the motor's own no-load speed (45 rpm at 12 V).
    jump_speed_share: float = 3.0
    jump_slack_steps: float = 4.0
    free_speed_max: float = math.radians(270)
    # Firmware status bits, the torque and Lock read-back and the supply under volt_min end guiding only once read this
    # many times in a row (one corrupted reply must not drop the arm; a real fault persists). Worst added delay: one
    # cycle (10 ms): the host reads the status registers again on the next cycle while one is suspect (status_recheck).
    persist_reads: int = 2
    temp_warn_c: float = 50.0
    temp_exit_c: float = 55.0
    # A reading at or over temp_exit_c ends guiding only after it has lasted this long: one corrupted status packet read
    # 57-62 C on a 27 C wrist (2026-10-10), and real heating takes minutes.
    temp_exit_hold_s: float = 0.25
    volt_entry_min: float = 8.5   # entry refused below this (the supply has browned out; the servos read 8.9 V once)
    volt_min: float = 7.0         # guiding: below this, coast
    volt_spread_max: float = 0.5  # entry refused when the servos' supply readings differ by more (a connector dropping it)
    volt_cal_share: float = 0.15  # entry refused when the supply is further than this share from V_cal
    temp_entry_max_c: float = 50.0    # entry refused at or above this
    load_check_units: float = 140.0   # Present_Load vs expected duty mismatch...
    load_check_s: float = 0.25        # ...for this long ends guiding (coast)
    wall_age_s: float = 0.15      # collision verdict older than this: treated as unknown -> coast
    vel_window_s: float = 0.04
    wall_limit: float = 750.0     # Torque_Limit while a wall or limit holds (just under the 800 firmware overload trip)
    zone_gain: float = 900.0      # duty units per rad/s at full depth of a limit zone, moving outward
    stroke_s: float = 2.0         # the friction feed-forward of one continuous stroke fades out after this...
    stroke_fade_s: float = 0.5    # ...over this
    # bounds the bridge refuses settings outside of (validate())
    k_f_max: float = 0.7
    c_comp_max: float = 0.8
    c_free_max: float = 0.9
    kv_prior_band: tuple = (0.75, 1.10)   # identified k_v / the motor's back-EMF prior at V_cal: an overestimate
                                          # times the largest lead (0.9) must stay under the whole drag (1.0)
    v_hi_max: float = 120.0 * DEG
    free_cap_max: float = 500.0
    alpha_range: tuple = (0.25, 1.0)      # the model gain: at most 1 (errors sink rather than rise); 0.25 is bring-up S5's
    v_cal_range: tuple = (8.2, 12.6)      # V_cal: the calibration starts at 8.5 V or more and holds within 0.3 V; 12.6 V
                                          # is the 12 V-class servo's top


@dataclass
class Reading:
    ok: bool
    pos: float = 0.0              # Present_Position, steps, in whatever frame the servo reports (multi-turn or wrapping)
    load: float = 0.0             # Present_Load decoded with sign bit 10, duty units
    volts: float = 8.9
    temp_c: float = 30.0
    status: int = 0               # reply ERROR byte (Servo_Status reg 65)
    torque_on: Optional[bool] = None  # from the periodic reg-40 read; None when not read this cycle
    lock: Optional[int] = None    # Lock (reg 55) from the same periodic read: 0 after a servo restart (the bridge keeps 1)


@dataclass
class Command:
    writes: Dict[str, dict] = field(default_factory=dict)   # joint -> {"goal": steps, "limit": units} (energises)
    release: tuple = ()           # joints to write Torque_Enable 0 (Lock stays 1): floating
    exit_kind: Optional[str] = None
    reason: str = ""
    phase: str = ""
    events: list = field(default_factory=list)


class VelocityEstimator:
    """Least-squares slope of the last vel_window_s of positions (steps -> rad/s), positions unwrapped circularly."""

    def __init__(self, window_s: float):
        self.window = window_s
        self.hist: deque = deque(maxlen=16)
        self.unwrapped: Optional[float] = None
        self.last_raw: Optional[float] = None

    def unwrap(self, raw: float) -> float:
        if self.last_raw is None:
            self.unwrapped, self.last_raw = raw, raw
            return raw
        d = (raw - self.last_raw + 2048.0) % 4096.0 - 2048.0
        self.unwrapped += d
        self.last_raw = raw
        return self.unwrapped

    def update(self, t: float, raw: float) -> float:
        pos = self.unwrap(raw)
        self.hist.append((t, pos))
        while len(self.hist) > 2 and t - self.hist[0][0] > self.window:
            self.hist.popleft()
        n = len(self.hist)
        if n < 2:
            return 0.0
        mt = sum(a for a, _ in self.hist) / n
        mp = sum(b for _, b in self.hist) / n
        sxx = sum((a - mt) ** 2 for a, _ in self.hist)
        if sxx <= 0:
            return 0.0
        sxy = sum((a - mt) * (b - mp) for a, b in self.hist)
        return sxy / sxx * STEP_RAD


def validate(configs: Dict[str, JointConfig], free_configs: Dict[str, FreeConfig], p: GuideParams) -> list:
    """Assist settings the bridge refuses to start with (each a plain-words reason). The bounds are what the release
    analysis and the plant support: a feed-forward above 0.7 of the friction it cancels, or a lead at or above the
    whole back-EMF drag, could keep a released joint moving."""
    out = []
    if not p.alpha_range[0] <= p.alpha <= p.alpha_range[1]:
        out.append(f"model gain (alpha) {p.alpha:g} is outside {p.alpha_range[0]:g}..{p.alpha_range[1]:g}")
    if not p.v_cal_range[0] <= p.v_cal <= p.v_cal_range[1]:
        out.append(f"the calibration's supply (V_cal) {p.v_cal:g} V is outside {p.v_cal_range[0]:g}..{p.v_cal_range[1]:g} V")
        return out   # the bounds below are relative to the motor's back-EMF at V_cal
    lo, hi = p.kv_prior_band
    prior = kv_prior(p.v_cal)
    for j, c in configs.items():
        if not 0.0 <= c.k_f <= p.k_f_max:
            out.append(f"{j}: friction assist {c.k_f:g} is outside 0..{p.k_f_max:g} of the calibrated friction")
        if not 0.0 <= c.c_lead <= p.c_comp_max:
            out.append(f"{j}: lead {c.c_lead:g} is outside 0..{p.c_comp_max:g} of the back-EMF drag")
        if c.k_f > 0 and c.f_kin0 < 0:
            out.append(f"{j}: calibrated friction is negative")
        if not lo <= c.k_v / prior <= hi:
            out.append(f"{j}: calibrated velocity slope {c.k_v:.0f} is not within {lo:g}-{hi:g} of the motor's {prior:.0f}")
        if c.v_db < 1.0 * DEG or c.v_full <= c.v_db:
            out.append(f"{j}: friction assist deadband {math.degrees(c.v_db):.1f} deg/s is too small")
    for j, f in free_configs.items():
        if not 0.0 <= f.k_f <= p.k_f_max:
            out.append(f"{j}: friction assist {f.k_f:g} is outside 0..{p.k_f_max:g} of the calibrated friction")
        if not 0.0 <= f.c_lead <= p.c_free_max:
            out.append(f"{j}: lead {f.c_lead:g} is outside 0..{p.c_free_max:g} of the back-EMF drag")
        if f.k_f > 0 and f.f_kin < 0:
            out.append(f"{j}: calibrated friction is negative")
        if not lo <= f.k_v / prior <= hi:
            out.append(f"{j}: calibrated velocity slope {f.k_v:.0f} is not within {lo:g}-{hi:g} of the motor's {prior:.0f}")
        if f.v_db < 1.0 * DEG or f.v_full <= f.v_db or f.v_on <= f.v_off or f.v_off < f.v_db:
            out.append(f"{j}: assist speeds out of order (deadband {math.degrees(f.v_db):.1f}, off "
                       f"{math.degrees(f.v_off):.1f}, on {math.degrees(f.v_on):.1f} deg/s)")
        if not f.v_on < f.v_hi <= p.v_hi_max:
            out.append(f"{j}: assist top speed {math.degrees(f.v_hi):.0f} deg/s is outside "
                       f"{math.degrees(f.v_on):.0f}..{math.degrees(p.v_hi_max):.0f}")
        if f.cap_max > p.free_cap_max or f.catch_limit > p.free_cap_max:
            out.append(f"{j}: assist torque ceiling over {p.free_cap_max:.0f}")
    return out


class HandGuideLaw:
    def __init__(self, configs: Dict[str, JointConfig], model: GravityModel, params: GuideParams | None = None,
                 free_configs: Dict[str, FreeConfig] | None = None, reference_raw: Dict[str, float] | None = None):
        self.ref = dict(REFERENCE_RAW if reference_raw is None else reference_raw)   # raw steps at the URDF zero pose
        self.cfg = configs
        self.model = model
        self.prior = GravityModel()
        self.p = params or GuideParams()
        self.fcfg = free_configs if free_configs is not None else {j: FreeConfig(j, **FREE_DEFAULTS[j]) for j in FREE_JOINTS}
        self.free = tuple(self.fcfg)          # the bridge leaves the gripper out while it squeezes an object
        self.joints = (*COMP_JOINTS, *self.free)
        # a joint left out (the gripper squeezing an object: "keep") is not driven, but its faults end guiding too
        self.kept = tuple(j for j in ALL_JOINTS if j not in self.joints)
        self.watched = (*self.joints, *self.kept)
        self.phase = "idle"
        self.t0 = 0.0
        self.t_last: Optional[float] = None
        self.anchor: Dict[str, float] = {}
        self.hold_goal: Dict[str, float] = {}
        self.vel = {j: VelocityEstimator(self.p.vel_window_s) for j in self.joints}
        self.v: Dict[str, float] = {j: 0.0 for j in self.joints}
        self.v_hist: Dict[str, deque] = {j: deque(maxlen=12) for j in self.joints}
        # positions: raw as the servo last reported it (its frame), unw the same in the session's continuous frame (every
        # position the law keeps is in this one); a goal g goes out as g + raw - unw (_out)
        self.raw: Dict[str, float] = {}
        self.unw: Dict[str, float] = {}
        self.t_pos: Dict[str, float] = {}       # when each joint's last accepted position was read
        self.last_goal: Dict[str, float] = {}
        self.last_limit: Dict[str, float] = {}
        self.misses = {j: 0 for j in self.watched}
        self.jumps = {j: 0 for j in self.watched}  # consecutive position readings rejected as implausible
        self.jump_raw: Dict[str, float] = {}       # the last of them
        self.suspect: Dict[tuple, int] = {}       # (check, joint) -> consecutive bad readings (persist_reads)
        self.status_recheck = False               # a torque or Lock read-back is suspect: read the status again next cycle
        self.held_raw: Dict[str, float] = {}      # a holding exit's positions, as reported (_exit)
        self.over_since: Dict[str, Optional[float]] = {j: None for j in COMP_JOINTS}
        self.fall_count = {j: 0 for j in COMP_JOINTS}
        self.hard_count = {j: 0 for j in COMP_JOINTS}
        self.load_bad_since: Dict[str, Optional[float]] = {j: None for j in COMP_JOINTS}
        self.limit_ratchet: Dict[str, Dict[str, float]] = {}
        self.wall_armed = False
        self.q_clear: Dict[str, float] | None = None
        self.max_dev = 0.0
        self.volts = self.p.v_cal
        self.warnings: list = []
        self.hot_since: Dict[str, float] = {}   # when each joint's reading first reached temp_exit_c (this run)
        self.cycle_dt = 0.01
        self.latency_s = 0.0025
        self.answering: set = set()
        # the assist
        self.ff_on = {j: True for j in self.joints}           # cleared by the start probe
        self.probe_ref: Dict[str, float] = {}
        self.probe_moved: Dict[str, float] = {}
        self.stroke: Dict[str, Optional[tuple]] = {j: None for j in self.joints}   # (t0, direction)
        self.fstate = {j: "hold" for j in self.free}          # hold (start) -> float / assist / caught
        self.rest_q: Dict[str, float] = {}
        self.rest_since: Dict[str, Optional[float]] = {j: None for j in self.free}
        self.on_count = {j: 0 for j in self.free}
        self.torque_cmd = {j: True for j in self.watched}     # what the last command left: the reg-40 read must agree
        self.assist_counts = {j: {"on": 0, "float_rest": 0, "float_fast": 0, "float_stroke": 0, "caught": 0}
                              for j in self.free}
        self.spent = {j: False for j in self.free}            # a stroke's help used up: float until rest
        self.band_since: Dict[str, Optional[float]] = {j: None for j in self.free}
        self.problems = validate(self.cfg, self.fcfg, self.p)

    # ------------------------------------------------------------------ helpers
    def urdf(self, joint: str, raw: float) -> float:
        """URDF angle (rad) of joint at raw steps: wrap(raw - ReferenceRaw) x step, as urdf_rad but with this arm's own
        ReferenceRaw (every joint of this arm reads + toward increasing raw: gravity_comp_model.py)."""
        d = (raw - self.ref[joint] + 2048.0) % 4096.0 - 2048.0
        return d * STEP_RAD

    def _c(self, joint: str):
        return self.cfg[joint] if joint in self.cfg else self.fcfg[joint]

    def jump_allowed(self, j: str, dt: float) -> float:
        """The furthest joint j can plausibly move in dt seconds, steps: jump_speed_share x its speed cap (a free or kept
        joint's: free_speed_max) over at least one 10 ms cycle, plus jump_slack_steps."""
        cap = self.cfg[j].v_cap if j in self.cfg else self.p.free_speed_max
        return self.p.jump_speed_share * cap * max(dt, 0.01) / STEP_RAD + self.p.jump_slack_steps

    def _accept(self, j: str, t: float, raw: float) -> bool:
        """Take joint j's position reading (read at t) into the continuous frame; False: it is implausible (left out). A
        reading about a turn from the last is the servo's frame wrapping, not a move: unwrapped."""
        last = self.raw.get(j)
        if last is None:
            self.raw[j], self.unw[j], self.t_pos[j] = raw, raw, t
            return True
        d = wrap_steps(raw - last)
        if abs(d) > self.jump_allowed(j, t - self.t_pos[j]):
            return False
        self.raw[j], self.unw[j], self.t_pos[j] = raw, self.unw[j] + d, t
        return True

    def _out(self, j: str, goal: float) -> float:
        """A goal in the session's continuous frame as the servo must get it: its own last reported position plus the
        offset (frame independent: never an absolute goal carried across a wrap)."""
        return goal + self.raw[j] - self.unw[j]

    def _persists(self, check: str, j: str, bad: bool) -> bool:
        """Count joint j's bad readings of check in a row; True once there are persist_reads of them."""
        n = self.suspect.get((check, j), 0) + 1 if bad else 0
        self.suspect[(check, j)] = n
        return n >= self.p.persist_reads

    def steps_for(self, joint: str, u: float) -> float:
        c = self._c(joint)
        return effort_steps(u, c.k_servo, c.punch, c.dead_zone)

    def effort(self, model: GravityModel, raw: Dict[str, float], volts: float, gain: float) -> Dict[str, float]:
        t = model.torques(self.urdf("shoulder_lift", raw["shoulder_lift"]), self.urdf("elbow_flex", raw["elbow_flex"]),
                          self.urdf("wrist_flex", raw["wrist_flex"]))
        kd = kd_nm_per_unit(volts)
        return {j: gain * t[j] / kd for j in COMP_JOINTS}

    def model_effort(self, raw: Dict[str, float], volts: float, gain: float) -> Dict[str, float]:
        return self.effort(self.model, raw, volts, gain)

    def prior_disagrees(self, raw: Dict[str, float], volts: float) -> Optional[str]:
        """The identified model against the URDF prior, computed independently: the commanded gravity effort must
        point against gravity as the prior computes it, and be of a plausible size."""
        m = self.model_effort(raw, volts, 1.0)
        pr = self.effort(self.prior, raw, volts, 1.0)
        lo, hi = self.p.prior_ratio
        for j in COMP_JOINTS:
            if abs(pr[j]) > self.p.prior_floor:
                r = m[j] / pr[j]
                if not lo <= r <= hi:
                    return f"{j}: model {m[j]:.0f} vs URDF {pr[j]:.0f} duty units"
            elif abs(m[j] - pr[j]) > self.p.prior_abs:
                return f"{j}: model {m[j]:.0f} vs URDF {pr[j]:.0f} duty units"
        return None

    def cap_for(self, joint: str, u: float) -> float:
        c = self._c(joint)
        return max(c.cap_min, min(c.cap_max, abs(u) + c.cap_margin))

    def ff_size(self, j: str, u_g1: Dict[str, float], volts: float) -> float:
        """Magnitude of joint j's friction feed-forward at full gain, duty units at the present supply."""
        if not self.ff_on.get(j, False):
            return 0.0
        scale = self.p.v_cal / volts
        if j in self.cfg:
            c = self.cfg[j]
            return c.k_f * (c.f_kin0 * scale + c.f_kin_load * abs(u_g1[j]))
        f = self.fcfg[j]
        return f.k_f * f.f_kin * scale

    def band_top(self, j: str, volts: float) -> float:
        """The top speed of free joint j's assist band, rad/s. An energised motor brakes with its back-EMF; the lead
        cancels c of it, so the assist beats a floating (torque-off) joint only while the friction feed-forward is
        larger than the drag left over: v < ff / ((1 - c) k_v + k_loop). The band ends at 0.8 of that crossover (or at
        v_hi). If torque off brakes as well (coast_brakes), energised is never worse: the band runs to v_hi. A band
        that would end below band_min x v_on is not worth switching the motor on for: 0 (the joint just floats)."""
        f = self.fcfg[j]
        if f.v_top_fixed:
            return f.v_top_fixed
        if f.coast_brakes:
            return f.v_hi
        ff = self.ff_size(j, {}, volts)
        if ff <= 0:
            return 0.0
        scale = self.p.v_cal / volts
        top = min(f.v_hi, 0.8 * ff / ((1.0 - f.c_lead) * f.k_v * scale + f.k_loop))
        return top if top >= f.band_min * f.v_on else 0.0

    def stroke_gain(self, j: str, t: float, v: float, v_db: float, v_full: float) -> float:
        """The feed-forward of one continuous stroke in one direction lasts stroke_s, then fades out; it re-arms when
        the joint comes to rest or reverses. A joint that keeps moving on its own (friction lower than calibrated
        and the probe passed anyway) loses the push that keeps it going within stroke_s + stroke_fade_s."""
        if abs(v) < v_db:
            self.stroke[j] = None
            return 0.0
        st = self.stroke[j]
        d = 1 if v > 0 else -1
        if st is None or (st[1] != d and abs(v) >= v_full):
            self.stroke[j] = st = (t, d)
        if st[1] != d:
            return 0.0
        age = t - st[0]
        if age <= self.p.stroke_s:
            return 1.0
        return max(0.0, 1.0 - (age - self.p.stroke_s) / self.p.stroke_fade_s)

    def ff_shape(self, j: str, t: float, v: float, v_db: float, v_full: float, v_top_lo: float, v_top_hi: float) -> float:
        """sign(v) x deadband ramp x high-speed fade x stroke gain, in [-1, 1]."""
        g = ramp01(abs(v), v_db, v_full) * (1.0 - ramp01(abs(v), v_top_lo, v_top_hi))
        g *= self.stroke_gain(j, t, v, v_db, v_full)
        return math.copysign(g, v) if g else 0.0

    # ------------------------------------------------------------------ entry
    def begin(self, t: float, readings: Dict[str, Reading], hold_goals: Dict[str, float]) -> Command:
        """Called once with the arm holding (position mode, normal limit) and still. hold_goals: the Goal_Position
        registers of the comp joints (read at entry). The bridge's own refusals (explicit, Manual mode, still, a
        page connected, calibration fingerprint...) come first; these are the ones the law can check itself."""
        self.phase, self.t0, self.t_last = "preload", t, t
        cmd = Command(phase=self.phase)
        if self.problems:
            return self._exit(cmd, "refused", "start refused: " + "; ".join(self.problems))
        volts = [readings[j].volts for j in self.watched if readings[j].ok]
        if not volts or min(volts) < self.p.volt_entry_min:
            low = min(volts) if volts else 0.0
            return self._exit(cmd, "refused", f"start refused: the servo supply reads {low:.1f} V (at least "
                                              f"{self.p.volt_entry_min:.1f} V needed); check the 12 V supply and its cable")
        silent = [j for j in self.watched if not readings[j].ok]
        if silent:
            return self._exit(cmd, "refused", f"start refused: {', '.join(silent)} did not answer")
        if max(volts) - min(volts) > self.p.volt_spread_max:
            return self._exit(cmd, "refused", f"start refused: the servos read the supply between {min(volts):.1f} and "
                                              f"{max(volts):.1f} V (more than {self.p.volt_spread_max:.1f} V apart): check "
                                              "the supply cable and its connectors")
        if abs(min(volts) - self.p.v_cal) > self.p.volt_cal_share * self.p.v_cal:
            return self._exit(cmd, "refused", f"start refused: the servo supply reads {min(volts):.1f} V, more than "
                                              f"{self.p.volt_cal_share * 100:.0f} % from the {self.p.v_cal:.1f} V hand guiding "
                                              "was calibrated at: check the supply, or calibrate hand guiding again")
        hot = [f"{j} {readings[j].temp_c:.0f} C" for j in self.watched if readings[j].temp_c >= self.p.temp_entry_max_c]
        if hot:
            return self._exit(cmd, "refused", f"start refused: {', '.join(hot)} (under {self.p.temp_entry_max_c:.0f} C "
                                              "needed); let it cool")
        self.raw, self.unw, self.t_pos = {}, {}, {}
        for j in self.watched:
            self._accept(j, t, readings[j].pos)
        self.anchor = {j: readings[j].pos for j in self.joints}
        # the entry hold goals, each taken the short way from where its joint reads (a goal register a turn away from a
        # wrapping servo's reading is the same angle)
        self.hold_goal = {j: readings[j].pos + wrap_steps(g - readings[j].pos) for j, g in hold_goals.items()}
        for j in self.joints:
            self.vel[j].hist.clear()
            self.vel[j].update(t, readings[j].pos)
        self.volts = self._scale_volts(readings, self.joints)
        # the walls hold from the entry pose: a start inside the collision guard band is held there, not left unguarded
        # until it first reads clear
        self.wall_armed, self.q_clear = True, dict(self.anchor)
        for j in COMP_JOINTS:
            c = self.cfg[j]
            self.limit_ratchet[j] = {"lo": c.lim_lo, "hi": c.lim_hi}
            q = self.urdf(j, readings[j].pos)
            # one-way: a joint already past its limit keeps its own position as the limit
            self.limit_ratchet[j]["lo"] = min(c.lim_lo, q)
            self.limit_ratchet[j]["hi"] = max(c.lim_hi, q)
        for j in self.free:
            f = self.fcfg[j]
            q = self.urdf(j, readings[j].pos)
            self.limit_ratchet[j] = {"lo": min(f.lim_lo, q), "hi": max(f.lim_hi, q)}
        Why = self.prior_disagrees(self.anchor, self.volts) if self.p.prior_check else None
        if Why:
            return self._exit(cmd, "anchor", f"start refused: the calibrated model disagrees with the URDF ({Why})")
        for j in COMP_JOINTS:
            self._write(cmd, j, self.hold_goal[j], 0.0, limit=self.cfg[j].cap_max)
        cmd.events.append("preload")
        return cmd

    def _scale_volts(self, readings: Dict[str, Reading], answering) -> float:
        """The supply the efforts are scaled with: the median of the comp joints' plausible readings (one corrupted reply
        cannot change it; of two, the higher: an effort a little low sinks, never rises), else the last one."""
        v = sorted(readings[j].volts for j in COMP_JOINTS if j in answering and readings[j].ok
                   and readings[j].volts >= self.p.volt_min)
        return v[len(v) // 2] if v else self.volts

    # ------------------------------------------------------------------ one cycle
    def step(self, t: float, readings: Dict[str, Reading], wall_band: Optional[bool] = None, wall_age: float = 0.0,
             exit_request: Optional[str] = None) -> Command:
        p = self.p
        cmd = Command(phase=self.phase)
        if self.phase in ("idle", "done"):
            return cmd
        gap = t - (self.t_last if self.t_last is not None else t)
        self.t_last = t
        if gap > 0:
            self.cycle_dt = 0.9 * self.cycle_dt + 0.1 * min(gap, 0.05)
        starting = self.phase in ("preload", "ramp", "probe")

        # --- feedback health: a silent joint keeps its last goal for a few cycles, then the whole arm coasts. A position
        #     reading that implies more than 3 x the joint's speed cap is a missed reply too (the whole reply is left
        #     out: a corrupted packet), and two in a row coast; a reading a turn away is the frame wrapping (_accept).
        self.answering = set()
        for j in self.watched:
            r = readings.get(j)
            if r is None or not r.ok:
                self.misses[j] += 1
            elif not self._accept(j, t, r.pos):
                self.misses[j] += 1
                self.jumps[j] += 1
                self.jump_raw[j] = r.pos
            else:
                self.misses[j] = 0
                self.jumps[j] = 0
                self.answering.add(j)
        jumped = [j for j in self.watched if self.jumps[j] >= p.persist_reads]
        if jumped:
            j = jumped[0]
            return self._exit(cmd, "coast", f"{j} position jumped from {self.raw[j]:.0f} to {self.jump_raw[j]:.0f} steps "
                                            f"on {self.jumps[j]} readings in a row (the servo restarted?): every joint "
                                            "goes limp; check the supply")
        silent = [j for j in self.watched if self.misses[j] >= p.miss_limit]
        if silent:
            return self._exit(cmd, "coast", f"{', '.join(silent)} stopped answering: every joint goes limp; check "
                                            "the servo, its cable and the supply")
        pos = {j: self.unw[j] for j in self.joints}
        volts = self._scale_volts(readings, self.answering)
        self.volts = volts

        # --- servo-reported faults (firmware protection, a restarted servo, heat, supply): coast, once each persists
        #     (persist_reads readings in a row; heat temp_exit_hold_s): one corrupted reply must not drop the arm
        for j in self.answering:
            r = readings[j]
            if self._persists("status", j, bool(r.status & UNLOAD_BITS)):
                return self._exit(cmd, "coast", f"{j} firmware protection bit {r.status}")
            if r.torque_on is not None and self._persists("torque", j, r.torque_on != self.torque_cmd.get(j, True)):
                return self._exit(cmd, "coast", f"{j} torque unexpectedly {'on' if r.torque_on else 'off'} "
                                                "(a servo restart?): every joint goes limp; check the supply")
            if r.lock is not None and self._persists("lock", j, r.lock == 0):
                return self._exit(cmd, "coast", f"{j} restarted (its Lock register reads 0): every joint goes limp; "
                                                "check the supply")
            # the supply: the lowest of every servo's reading, each low reading confirmed by the next from that servo
            if self._persists("volts", j, r.volts < p.volt_min):
                return self._exit(cmd, "coast", f"supply {r.volts:.1f} V below {p.volt_min} V: every joint goes limp")
            if r.temp_c >= p.temp_exit_c:
                if t - self.hot_since.setdefault(j, t) > p.temp_exit_hold_s:
                    return self._exit(cmd, "coast", f"{j} over {p.temp_exit_hold_s * 1000:.0f} ms at "
                                                    f"{r.temp_c:.0f} C: every joint goes limp")
            else:
                self.hot_since.pop(j, None)
            if r.temp_c >= p.temp_warn_c and f"warm:{j}" not in self.warnings:
                self.warnings.append(f"warm:{j}")
                cmd.events.append(f"warm:{j}")
        self.status_recheck = any(n for (check, _), n in self.suspect.items() if check in ("torque", "lock"))

        # --- a late cycle (thread stall): the servos held their last goals meanwhile
        if gap > p.stall_exit_s:
            if starting:
                return self._exit(cmd, "anchor", f"control loop paused {gap * 1000:.0f} ms during the start")
            return self._exit(cmd, "coast", f"control loop paused {gap * 1000:.0f} ms: every joint goes limp")

        # a joint whose reply is missing (or left out) keeps its last goal: nothing below acts on it this cycle
        for j in self.joints:
            if j in self.answering:
                self.v[j] = self.vel[j].update(t, pos[j])
                self.v_hist[j].append((t, self.v[j]))

        if exit_request:
            if starting:
                return self._exit(cmd, "anchor", "stopped during the start")
            return self._exit(cmd, exit_request, "exit requested")

        # --- the model against the URDF prior, every cycle (a bug, a stale or wrong calibration)
        if p.prior_check:
            Why = self.prior_disagrees(pos, volts)
            if Why:
                if starting:
                    return self._exit(cmd, "anchor", f"start aborted: model disagrees with the URDF ({Why})")
                return self._exit(cmd, "coast", f"model disagrees with the URDF ({Why}): every joint goes limp")

        u_model = self.model_effort(pos, volts, p.alpha)
        u_g1 = self.model_effort(pos, volts, 1.0)
        u_prior = self.effort(self.prior, pos, volts, 1.0)
        tau = t - self.t0

        # ---------------- PRELOAD, RAMP, PROBE: the anchored start, hands off, which proves the model holds this arm
        if starting:
            dev = max(abs(pos[j] - self.anchor[j]) for j in COMP_JOINTS)
            spd = max(abs(self.v[j]) for j in COMP_JOINTS)
            self.max_dev = max(self.max_dev, dev)
            if dev > p.ramp_dev_steps or (spd > p.ramp_speed and self.phase != "probe"):
                return self._exit(cmd, "anchor", f"start aborted: the arm moved {dev:.0f} steps from where it held "
                                                 f"(model or sign check failed, or a hand was on it)")
            if self.phase == "preload":
                mu = _smooth(tau / p.preload_s)
                for j in self._answered(COMP_JOINTS):
                    target = self.anchor[j] + self.steps_for(j, u_model[j])
                    goal = (1 - mu) * self.hold_goal[j] + mu * target
                    self._write(cmd, j, goal, u_model[j], limit=self.cfg[j].cap_max)
                if tau >= p.preload_s:
                    self.phase, self.t0 = "ramp", t
                    cmd.events.append("ramp")
                return cmd
            if self.phase == "ramp":
                h = 1.0 - _smooth(tau / p.ramp_s)
                for j in self._answered(COMP_JOINTS):
                    goal = pos[j] + self.steps_for(j, u_model[j]) + h * (self.anchor[j] - pos[j])
                    c = self.cfg[j]
                    lim = h * c.cap_max + (1 - h) * self.cap_for(j, u_model[j])
                    self._write(cmd, j, goal, u_model[j], limit=lim)
                if tau >= p.ramp_s:
                    if p.probe and any(self.ff_size(j, u_g1, volts) > 0 for j in COMP_JOINTS):
                        self.phase, self.t0 = "probe", t
                        self.probe_ref = {j: pos[j] for j in self.joints}
                        self.probe_moved = {j: 0.0 for j in self.joints}
                        cmd.events.append("probe")
                    else:
                        self._enter_guide(cmd, t, pos)
                return cmd
            # PROBE (lift, elbow, wrist_flex): the effort ramps from the model's to the model's +1.4 x the friction
            # feed-forward over probe_s, then to -1.4 x over probe_s, then back. Gravity keeps these gears pressed on
            # one side, so a feed-forward larger than the friction left over shows as output motion at once; a joint
            # that moves more than probe_steps gets no feed-forward, and its push stops there (a few steps, never an
            # abort). The free joints are not probed: their backlash and the energised motor's own back-EMF brake
            # hide a short probe (bench: the motor side creeps at 2 deg/s inside the 0.87 deg gap), and they cannot
            # run away anyway: above the band's top they float.
            if tau < p.probe_s:
                sgn = _smooth(tau / p.probe_s)
            elif tau < 2 * p.probe_s:
                sgn = 1.0 - 2.0 * _smooth((tau - p.probe_s) / p.probe_s)
            else:
                sgn = -1.0 + _smooth((tau - 2 * p.probe_s) / p.probe_settle_s)
            for j in self._answered(COMP_JOINTS):
                moved = abs(pos[j] - self.probe_ref[j])
                self.probe_moved[j] = max(self.probe_moved[j], moved)
                if self.ff_on[j] and self.ff_size(j, u_g1, volts) > 0 and moved > p.probe_steps:
                    self.ff_on[j] = False
                    cmd.events.append(f"probe:{j} moved {moved:.0f} steps under 1.4 x its friction assist: friction "
                                      f"lower than calibrated (or the model is off), assist off for {j}")
                push = sgn * p.probe_gain * self.ff_size(j, u_g1, volts)
                u = u_model[j] + push
                self._write(cmd, j, pos[j] + self.steps_for(j, u), u)
            if tau >= 2 * p.probe_s + p.probe_settle_s:
                self._enter_guide(cmd, t, pos)
            return cmd

        # ---------------- GUIDE
        # 1. fall: accelerating the way gravity pulls harder than a guiding hand does, while the model carries a real
        #    load. The model is not trusted: coast.
        for j in self._answered(COMP_JOINTS):
            if abs(u_model[j]) < p.fall_min_effort:
                continue
            gdir = -math.copysign(1.0, u_model[j])
            vg = self.v[j] * gdir
            ag = self._accel(j) * gdir
            self.fall_count[j] = self.fall_count[j] + 1 if (vg > p.fall_accel_speed and ag > p.fall_accel) else 0
            if self.fall_count[j] >= p.fall_cycles:
                return self._exit(cmd, "coast", f"{j} fell ({math.degrees(vg):.0f} deg/s, {ag:.0f} rad/s^2 toward "
                                                f"gravity): every joint goes limp")
        # 2. collision guard band (the bridge's 30 Hz padded collision check of the measured pose)
        if wall_band is None or wall_age > p.wall_age_s:
            return self._exit(cmd, "coast", "collision check unavailable: every joint goes limp")
        if not wall_band:
            self.wall_armed = True
            self.q_clear = {j: pos[j] for j in self.joints}
        in_band = bool(wall_band) and self.wall_armed and self.q_clear is not None
        # 3. per comp joint effort: gravity + lead + friction feed-forward, brakes
        for j in self._answered(COMP_JOINTS):
            c = self.cfg[j]
            v = self.v[j]
            u = u_model[j]
            soft = p.soft_speed_share * c.v_cap
            if abs(v) <= soft:
                u += c.c_lead * c.k_v * v * (p.v_cal / volts)
                ff = self.ff_size(j, u_g1, volts)
                if ff:
                    u += ff * self.ff_shape(j, t, v, c.v_db, c.v_full, 0.5 * soft, soft)
            else:
                u -= p.brake_gain * (abs(v) - soft) * math.copysign(1.0, v)
                self.stroke[j] = None
            # speed cap: brake above soft; end guiding if it stays over the cap or runs well over it
            self.hard_count[j] = self.hard_count[j] + 1 if abs(v) > p.overspeed_hard_share * c.v_cap else 0
            if abs(v) > c.v_cap:
                self.over_since[j] = self.over_since[j] if self.over_since[j] is not None else t
                if t - self.over_since[j] >= p.overspeed_s or self.hard_count[j] >= 2:
                    falling = abs(u_model[j]) >= p.fall_min_effort and v * -math.copysign(1.0, u_model[j]) > 0
                    return self._exit(cmd, "coast", f"{j} over its hand-guiding speed ({math.degrees(abs(v)):.0f} deg/s"
                                                    f"{', toward gravity' if falling else ''}): every joint goes limp")
            else:
                self.over_since[j] = None
            # soft joint limits: resistance in the zone while moving outward, a one-way spring at the limit
            q = self.urdf(j, pos[j])
            lim = self.limit_ratchet[j]
            if q < lim["hi"]:
                lim["hi"] = max(c.lim_hi, q)      # an effective limit a joint started past moves in with it
            if q > lim["lo"]:
                lim["lo"] = min(c.lim_lo, q)
            anchor_steps = None
            if q > lim["hi"]:
                anchor_steps = pos[j] - (q - lim["hi"]) / STEP_RAD
            elif q < lim["lo"]:
                anchor_steps = pos[j] + (lim["lo"] - q) / STEP_RAD
            else:
                depth_hi = (q - (lim["hi"] - c.limit_zone)) / c.limit_zone
                depth_lo = ((lim["lo"] + c.limit_zone) - q) / c.limit_zone
                if depth_hi > 0 and v > 0:
                    u -= p.zone_gain * depth_hi * v
                if depth_lo > 0 and v < 0:
                    u -= p.zone_gain * depth_lo * v
            prev_goal, prev_limit = self.last_goal.get(j), self.last_limit.get(j)
            lead_steps = p.lag_share * v * (self.latency_s + 0.5 * self.cycle_dt) / STEP_RAD if abs(v) <= soft else 0.0
            if in_band:
                self._write(cmd, j, self.q_clear[j] + self.steps_for(j, u_prior[j]), u_prior[j], limit=p.wall_limit)
            elif anchor_steps is not None:
                self._write(cmd, j, anchor_steps + self.steps_for(j, u_prior[j]), u_prior[j], limit=p.wall_limit)
            else:
                self._write(cmd, j, pos[j] + lead_steps + self.steps_for(j, u), u)
            # 4. what the servo actually applies (Present_Load) must match what its last goal asked for
            r = readings[j]
            if prev_goal is not None:
                expect = self._expected_duty(j, prev_goal, pos[j], prev_limit)
                bad = abs(r.load - expect) > p.load_check_units and abs(v) < 0.5 * c.v_cap
                if bad:
                    self.load_bad_since[j] = self.load_bad_since[j] if self.load_bad_since[j] is not None else t
                    if t - self.load_bad_since[j] > p.load_check_s:
                        return self._exit(cmd, "coast", f"{j} applies {r.load:.0f} where {expect:.0f} was asked for")
                else:
                    self.load_bad_since[j] = None
        # 5. free joints: floating at rest and when fast, assisted while the hand moves them, caught at walls/limits
        release = []
        for j in self._answered(self.free):
            self._free_joint(cmd, release, j, t, pos, volts, in_band)
        if release:
            cmd.release = tuple(release)
        for j in cmd.writes:
            self.torque_cmd[j] = True
        for j in cmd.release:
            self.torque_cmd[j] = False
        return cmd

    # ------------------------------------------------------------------ free joints
    def _free_joint(self, cmd: Command, release: list, j: str, t: float, pos: Dict[str, float], volts: float,
                    in_band: bool) -> None:
        p, f = self.p, self.fcfg[j]
        v = self.v[j]
        q = self.urdf(j, pos[j])
        lim = self.limit_ratchet[j]
        if q < lim["hi"]:
            lim["hi"] = max(f.lim_hi, q)
        if q > lim["lo"]:
            lim["lo"] = min(f.lim_lo, q)
        at_limit = (q >= lim["hi"] and v >= 0) or (q <= lim["lo"] and v <= 0)
        depth = 0.0
        if v > 0:
            depth = (q - (lim["hi"] - f.limit_zone)) / f.limit_zone
        elif v < 0:
            depth = ((lim["lo"] + f.limit_zone) - q) / f.limit_zone
        st = self.fstate[j]
        top = self.band_top(j, volts)
        # caught: in the collision guard band, or at a slider limit moving outward
        if in_band or at_limit:
            if st != "caught":
                if in_band and self.q_clear:
                    anchor = self.q_clear[j]
                else:
                    edge = lim["hi"] if q >= lim["hi"] else lim["lo"]
                    anchor = pos[j] - (q - edge) / STEP_RAD
                cmd.writes[j] = {"goal": self._out(j, anchor), "limit": f.catch_limit}
                cmd.events.append(f"catch:{j}")
                self.fstate[j] = "caught"
                self.assist_counts[j]["caught"] += 1
            return
        if st == "caught":
            release.append(j)
            cmd.events.append(f"release:{j}")
            self.fstate[j] = "float"
            self.rest_q[j] = pos[j]
            self.stroke[j] = None
            return
        if st == "float":
            if abs(v) < f.v_off:
                self.rest_q[j] = pos[j]
                self.on_count[j] = 0
                self.spent[j] = False
                return
            moved = abs(pos[j] - self.rest_q.get(j, pos[j])) >= f.start_steps
            zone = depth > 0
            in_band = moved and not self.spent[j] and f.v_on <= abs(v) < f.v_hi_back * top
            if in_band:
                self.band_since[j] = self.band_since[j] if self.band_since[j] is not None else t
            else:
                self.band_since[j] = None
            # floating driver: only a steady move in the band (a flick passes through); braking driver: at once
            steady = f.coast_brakes or (abs(self._accel(j)) <= f.a_on and t - self.band_since[j] >= f.steady_s
                                        if in_band else False)
            if zone or (in_band and steady):
                self.on_count[j] += 1
            else:
                self.on_count[j] = 0
            if self.on_count[j] < 2:
                return
            self.fstate[j] = st = "assist"
            self.rest_since[j] = None
            self.assist_counts[j]["on"] += 1
        # assist: above the band's top it floats (Coast is lighter there than any energised drive), and while the
        # hand speeds it up or slows it down hard (only where floating is lighter: not with a braking driver)
        if depth <= 0 and (abs(v) > top or (not f.coast_brakes and abs(self._accel(j)) > f.a_off)):
            release.append(j)
            self.fstate[j] = "float"
            self.on_count[j] = 0
            self.stroke[j] = None
            self.assist_counts[j]["float_fast"] += 1
            return
        # one stroke of help lasts stroke_s; then the joint floats until it has come to rest (the decay: a joint that
        # kept moving on its own, against every bound, is let go within stroke_s). With a braking driver the
        # feed-forward fades out instead (ff_shape) and the passive lead stays.
        st0 = self.stroke[j]
        if st0 is not None and t - st0[0] > p.stroke_s and depth <= 0 and not f.coast_brakes:
            release.append(j)
            self.fstate[j] = "float"
            self.on_count[j] = 0
            self.spent[j] = True
            self.stroke[j] = None
            self.assist_counts[j]["float_stroke"] += 1
            return
        if abs(v) < f.v_off:
            self.rest_since[j] = self.rest_since[j] if self.rest_since[j] is not None else t
            if t - self.rest_since[j] >= f.rest_s:
                release.append(j)
                self.fstate[j] = "float"
                self.rest_q[j] = pos[j]
                self.on_count[j] = 0
                self.stroke[j] = None
                self.assist_counts[j]["float_rest"] += 1
                return
        else:
            self.rest_since[j] = None
        scale = p.v_cal / volts
        if depth > 0:
            u = -p.zone_gain * min(depth, 1.0) * v            # the last 6 deg moving outward: brake, no assist
            self.stroke[j] = None
            lead_steps = 0.0
        else:
            u = f.c_lead * f.k_v * v * scale
            ff = self.ff_size(j, {}, volts)
            if ff:
                u += ff * self.ff_shape(j, t, v, f.v_db, f.v_full, 0.8 * top, top)
            lead_steps = p.lag_share * v * (self.latency_s + 0.5 * self.cycle_dt) / STEP_RAD
        cmd.writes[j] = {"goal": self._out(j, pos[j] + lead_steps + self.steps_for(j, u)), "limit": self.cap_for(j, u)}

    def _enter_guide(self, cmd: Command, t: float, pos: Dict[str, float]) -> None:
        self.phase, self.t0 = "guide", t
        cmd.release = tuple(self.free)
        for j in self.free:
            self.fstate[j] = "float"
            self.rest_q[j] = pos[j]
            self.torque_cmd[j] = False
            cmd.writes.pop(j, None)
        cmd.events.append("guide")

    # ------------------------------------------------------------------ pieces
    def _accel(self, j: str) -> float:
        h = self.v_hist[j]
        if len(h) < 2:
            return 0.0
        t1, v1 = h[-1]
        for t0, v0 in h:
            if t1 - t0 <= 0.08:
                return (v1 - v0) / (t1 - t0) if t1 > t0 else 0.0
        return 0.0

    def _expected_duty(self, j: str, goal: float, pos: float, limit: Optional[float]) -> float:
        c = self.cfg[j]
        e = goal - pos
        if abs(e) <= c.dead_zone:
            return 0.0
        d = c.k_servo * (e - math.copysign(c.dead_zone, e))
        d += math.copysign(c.punch, d) if d else 0.0
        lim = c.cap_max if limit is None else limit
        return max(-lim, min(lim, d))

    def _answered(self, joints) -> list:
        """The joints of joints whose reply this cycle was taken (the others keep their last goal)."""
        return [j for j in joints if j in self.answering]

    def _write(self, cmd: Command, j: str, goal: float, u_for_cap: float, limit: Optional[float] = None):
        """Goal (the continuous frame) to joint j, as an offset from the position it reported (_out)."""
        lim = self.cap_for(j, u_for_cap) if limit is None else limit
        cmd.writes[j] = {"goal": self._out(j, goal), "limit": lim}
        self.last_goal[j], self.last_limit[j] = goal, lim

    def _exit(self, cmd: Command, kind: str, reason: str, fresh: Optional[Dict[str, Reading]] = None,
              t: Optional[float] = None) -> Command:
        """The final writes. "coast" (every fault): none here; the bridge's AutoCoast writes Torque_Enable 0 to every
        joint and reads it back. "refused": none (nothing was written). "preloaded": comp joints held at the present
        pose with the model's effort, free joints energised where they are, normal limit. "anchor": back to the entry
        hold goals (a failed start: nothing was handed to the hand). fresh: readings taken for the exit (at t), used
        only where plausible (a corrupted one would be held at full limit): else each joint's last accepted reading.
        Every goal is an offset from the position the servo reported (frame independent)."""
        cmd.exit_kind, cmd.reason = kind, reason
        self.phase = "done"
        cmd.phase = "done"
        if kind in ("coast", "refused"):
            return cmd
        raw, pos = dict(self.raw), dict(self.unw)
        when = t if t is not None else (self.t_last if self.t_last is not None else 0.0)
        for j, r in (fresh or {}).items():
            if j in self.joints and r.ok and j in self.raw:
                d = wrap_steps(r.pos - self.raw[j])
                if abs(d) <= self.jump_allowed(j, when - self.t_pos[j]):
                    raw[j], pos[j] = r.pos, self.unw[j] + d
        self.held_raw = raw             # the positions the hold was taken at, as reported (the host's measured pose)
        u = self.model_effort(pos, self.volts, self.p.alpha_hold) if kind == "preloaded" else {j: 0.0 for j in COMP_JOINTS}
        for j in COMP_JOINTS:
            goal = self.hold_goal[j] if kind == "anchor" else pos[j] + self.steps_for(j, u[j])
            cmd.writes[j] = {"goal": goal + raw[j] - pos[j], "limit": 1000.0}
        for j in self.free:
            goal = self.anchor.get(j, pos[j]) if kind == "anchor" else pos[j]
            cmd.writes[j] = {"goal": goal + raw[j] - pos[j], "limit": 1000.0}
        return cmd


def _smooth(x: float) -> float:
    x = max(0.0, min(1.0, x))
    return x * x * (3 - 2 * x)
