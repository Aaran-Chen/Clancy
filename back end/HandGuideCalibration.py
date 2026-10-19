"""Hand-guiding calibration (docs/HandGuide.md 5): the sweeps, the fit, the acceptance checks, the file, and building
the law the bridge runs from it. Pure logic: no serial port, no bridge, no clock of its own.

The calibration ("Calibrate hand guiding") sweeps each joint slowly through a stretch of its range in position mode, a
goal ramp at 10 and then 30 deg/s (the lift 8 and 20), twice each, while the bridge's 100 Hz guide loop records Present_Load (sign bit 10),
the pose and the supply. Then the roll and the gripper are spun at 45 and 90 deg/s and let go (Torque_Enable 0) for
0.4 s: how far they run on tells whether a servo with its torque off floats (the run-on grows with the speed squared)
or brakes (it grows with the speed).
  * lift, elbow, wrist: the band centre (L_up + L_down) / 2 is the holding effort: fitted by MAP regression to the
    closed form with the URDF as the prior (25 %, at least 20 units). The half-band (L_up - L_down) / 2 is friction plus
    the velocity slope: its growth from the slow to the fast speed gives k_v, and what is left at the slow one the kinetic friction,
    F(q) = f_kin0 + f_kin_load |G(q)|.
  * pan, roll, gripper: the half-band in the middle of each pass gives k_v (growth) and f_kin.
Accepted only if: the fitted model agrees in sign with the URDF wherever the prior exceeds 40 units and lies within
0.6-1.6 of it (tighter than the runtime 0.5-1.8); the fit residual is at most 25 units RMS; every friction is positive;
every k_v is within 0.75-1.10 of the motor's back-EMF prior; the two runs agree (half-bands within 25 % on the comp
joints and 20 % on the free ones, k_v within 10 %); the supply held within 0.3 V; the spin-down verdict is the same on
the roll and the gripper. Otherwise nothing is saved and the result says what failed.

Safety (2026-10-10 review): every goal is written with a Torque_Limit of what that joint is expected to need plus a
margin (CalibrationRun.Limit; the gripper's opening and closing at most GripperSweepUnits), never 1000; every energised
joint is checked against its goal's path on every segment, the spins too, and a joint off it on two readings in a row
stops the calibration; a stop holds every joint where it is (HoldHere: the lift, elbow and wrist with the URDF prior's
gravity effort), never at the sweep's goal, which kept a blocked joint pushing into what stopped it. Positions are taken
into the run's own continuous frame (a servo reporting wrapping positions crosses raw 0 mid-sweep) and every goal is an
offset from the position its servo reported; a reading implying more than JumpDegS is a missed reply.

    Run = CalibrationRun(StartRaw, HoldGoals, Room)       # the sweeps from where the arm holds
    Command = Run.Step(t, Readings)                         # every 10 ms: the goals to write, Done, Error
    Data, Problems = Fit(Run.Samples, Run.SpinDowns, Fingerprint, ReferenceRaw)
    Law = BuildLaw(Data, Settings, ReferenceRaw, Limits)  # at entry, with the servos' own loop settings
"""
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

import HandGuide as L

CalibrationVersion = 1
DEG = math.pi / 180.0
SweepSpeeds = (10.0, 30.0)                        # deg/s (5 and 20 until 2026-10-11: about 6 minutes, now about 3.5)
# The lift slower: above 20 deg/s its load-dependent gear friction inflates its velocity slope past the 1.10 x prior
# check, and on the high-friction plant it falls 82 steps behind a 30 deg/s goal (TrackLimitSteps stops it)
SweepSpeedsByJoint = {"shoulder_lift": (8.0, 20.0)}
Runs = ("A", "B")
# Half the stretch each joint sweeps (deg either side of where it holds), within its slider arc less ArcMarginDeg
SweepHalfSpanDeg = {"wrist_flex": 30.0, "elbow_flex": 25.0, "shoulder_lift": 20.0, "shoulder_pan": 20.0,
                    "wrist_roll": 30.0, "gripper": 30.0}
SweepOrder = ("wrist_flex", "elbow_flex", "shoulder_lift", "shoulder_pan", "wrist_roll", "gripper")
MinimumSweepDeg = 15.0                            # a joint with less room than this in total is refused
ArcMarginDeg = 3.0
SettleS = 0.5                                     # a pause at each turn
SkipAfterTurnDeg = 4.0                            # samples within this of a turn are left out (backlash, the loop catching up)
TrackLimitSteps = 80                              # a joint further than this from its goal's path (two readings in a row)
                                                  # stops the calibration, holding where it is
# Torque_Limit while calibrating: (the URDF prior's gravity + the motor's back-EMF at the goal's speed) x (1 +
# SweepLoadShare) + SweepMarginUnits, at most SweepCeilingUnits (under the 800 overload trip). The gear's friction grows
# with the torque it carries (the plant's high-friction gear adds 0.9 x it, the 25 % model error on top) and has an
# unloaded part and the position loop's punch: on the plant the most any joint needed was the lift's 645 units (high
# friction, 10 % heavier) where this allows 700. The gripper's opening, closing and return at most GripperSweepUnits
# (about 7 N at the jaw where 1000 squeezed about 28 N).
SweepLoadShare = 1.0
SweepMarginUnits = 200.0
SweepCeilingUnits = 750.0
GripperSweepUnits = 250.0
JumpDegS = 270.0                                  # a reading implying more than this (3 x the fastest spin) is a missed reply
SpinSpeeds = (45.0, 90.0)
SpinTravelDeg = 30.0
SpinWatchS = 0.4
SpinJoints = ("wrist_roll", "gripper")
FloatRatio, FloatRunOnDeg = 2.2, 3.0              # floats if run-on(90) / run-on(45) >= 2.2 and run-on(90) >= 3 deg
BinDeg = 2.0
PriorShare, PriorFloorUnits, SampleSdUnits = 0.25, 20.0, 8.0
AcceptRatio, AcceptFloorUnits, AcceptAbsUnits = (0.6, 1.6), 40.0, 60.0
ResidualMaxUnits = 25.0
RunsAgreeComp, RunsAgreeFree, KvRunsAgree = 0.25, 0.20, 0.10
SupplyHeldV = 0.3
CompDefaults = {"shoulder_lift": dict(cap_min=150.0, cap_max=650.0, v_cap=45 * DEG),
                "elbow_flex": dict(cap_min=120.0, cap_max=450.0, v_cap=50 * DEG),
                "wrist_flex": dict(cap_min=100.0, cap_max=250.0, v_cap=80 * DEG)}
LeadDefault = {"comp": 0.6, "free": 0.9}
FrictionAssistDefault = 0.5
AlphaDefault = 0.95


# --------------------------------------------------------------------------------------------- the file
def Load(Path_: Path) -> dict | None:
    try:
        Data = json.loads(Path(Path_).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    return Data if isinstance(Data, dict) else None


def Save(Path_: Path, Data: dict) -> None:
    Target = Path(Path_)
    Temporary = Target.with_suffix(Target.suffix + ".tmp")
    Temporary.write_text(json.dumps(Data, indent=1) + "\n", encoding="utf-8")
    Temporary.replace(Target)


def FingerprintProblems(Data: dict | None, Fingerprint: dict) -> list[str]:
    """Why a calibration does not fit the arm as it is now (each a plain-words reason), or []."""
    if not Data:
        return ["hand guiding is not calibrated: press Calibrate hand guiding first"]
    if Data.get("Version") != CalibrationVersion:
        return [f"the hand-guiding calibration is from another version ({Data.get('Version')}): calibrate again"]
    Was = Data.get("Fingerprint") or {}
    Out = []
    if Was.get("CadAlignmentVersion") != Fingerprint.get("CadAlignmentVersion") or Was.get("ReferenceRaw") != Fingerprint.get("ReferenceRaw"):
        Out.append("the arm's zero pose (CadAlignment) changed since hand guiding was calibrated")
    for Name, Now in (Fingerprint.get("Servos") or {}).items():
        Then = (Was.get("Servos") or {}).get(Name)
        if Then != Now:
            Changed = sorted(K for K in set(Now or {}) | set(Then or {}) if (Now or {}).get(K) != (Then or {}).get(K))
            Out.append(f"{Name}'s settings changed since hand guiding was calibrated ({', '.join(Changed) or 'unread'})")
    if Out:
        Out[-1] += ": calibrate hand guiding again"
    return Out


# --------------------------------------------------------------------------------------------- the law from the file
def BuildLaw(Data: dict, Settings: dict[str, dict], ReferenceRaw: dict[str, int], Limits: dict[str, tuple[float, float]],
             FreeJoints: tuple = L.FREE_JOINTS, Params: dict | None = None) -> L.HandGuideLaw:
    """The law for this calibration. Settings: each servo's loop settings as read now (GuideBus.ReadSettings: p, punch,
    dead_zone); Limits: each joint's slider arc as URDF angles (rad); FreeJoints: the free joints (the gripper is left
    out while it squeezes an object). Its validate() problems are law.problems (begin refuses with them)."""
    Joints = Data["Joints"]

    def Loop(Name):
        S = Settings[Name]
        return dict(k_servo=S["p"] / 4.0, punch=float(S["punch"]), dead_zone=sum(S["dead_zone"]) / 2.0)

    Configs = {}
    for Name in L.COMP_JOINTS:
        J = Joints[Name]
        Configs[Name] = L.JointConfig(Name, **CompDefaults[Name], **Loop(Name), lim_lo=Limits[Name][0], lim_hi=Limits[Name][1],
                                      k_v=float(J["k_v"]), c_lead=float(J.get("c_lead", LeadDefault["comp"])),
                                      f_kin0=float(J["f_kin0"]), f_kin_load=float(J["f_kin_load"]),
                                      k_f=float(J.get("k_f", FrictionAssistDefault)))
    Frees = {}
    for Name in FreeJoints:
        J = Joints[Name]
        Defaults = dict(L.FREE_DEFAULTS[Name])
        if "v_hi_deg_s" in J:
            Defaults["v_hi"] = float(J["v_hi_deg_s"]) * DEG
        Frees[Name] = L.FreeConfig(Name, **Defaults, **Loop(Name), lim_lo=Limits[Name][0], lim_hi=Limits[Name][1],
                                   k_v=float(J["k_v"]), c_lead=float(J.get("c_lead", LeadDefault["free"])),
                                   f_kin=float(J["f_kin"]), k_f=float(J.get("k_f", FrictionAssistDefault)),
                                   coast_brakes=bool(Data.get("CoastBrakes", False)))
    Guide = L.GuideParams(alpha=float(Data.get("Alpha", AlphaDefault)), v_cal=float(Data["VCal"]), **(Params or {}))
    return L.HandGuideLaw(Configs, L.GravityModel(p=tuple(float(V) for V in Data["Model"])), Guide, free_configs=Frees,
                          reference_raw=ReferenceRaw)


# --------------------------------------------------------------------------------------------- the sweeps
@dataclass
class Segment:
    Kind: str                     # "sweep", "pause", "spin", "watch", "return"
    Joint: str | None = None
    FromDeg: float = 0.0          # relative to the joint's hold goal
    ToDeg: float = 0.0
    Speed: float = 0.0            # deg/s
    Run: str = ""
    Seconds: float = 0.0


def JointSpeeds(Joint: str) -> tuple[float, float]:
    """The joint's slow and fast sweep speeds, deg/s."""
    return SweepSpeedsByJoint.get(Joint, SweepSpeeds)


def JointRoom(Joint: str, PresentRad: float, Limits: dict[str, tuple[float, float]]) -> tuple[float, float]:
    """Degrees the joint can sweep below and above where it is, inside its slider arc less ArcMarginDeg."""
    Low, High = Limits[Joint]
    return (max(0.0, math.degrees(PresentRad - Low) - ArcMarginDeg), max(0.0, math.degrees(High - PresentRad) - ArcMarginDeg))


def PlanSweeps(PresentRad: dict[str, float], Limits: dict[str, tuple[float, float]], Joints=SweepOrder,
               HalfSpan: dict[str, float] | None = None, Speeds: tuple | None = None, RunNames=Runs,
               Spin=SpinJoints) -> tuple[list[Segment], list[str]]:
    """The calibration's segments from where the arm holds, and the joints refused for want of room (plain words).
    Speeds: every joint's sweep speeds (default: each joint's JointSpeeds)."""
    HalfSpan = {**SweepHalfSpanDeg, **(HalfSpan or {})}
    Segments, Problems = [], []
    for Joint in Joints:
        Below, Above = JointRoom(Joint, PresentRad[Joint], Limits)
        if Joint == "gripper":
            # open and close from where it is: toward the side with more room only
            Down, Up = (0.0, min(HalfSpan[Joint], Above)) if Above >= Below else (min(HalfSpan[Joint], Below), 0.0)
        else:
            Down, Up = min(HalfSpan[Joint], Below), min(HalfSpan[Joint], Above)
        if Down + Up < MinimumSweepDeg:
            Problems.append(f"{Joint.replace('_', ' ')} has only {Down + Up:.0f} deg of room in its slider range here "
                            f"(at least {MinimumSweepDeg:.0f} needed): move it toward the middle of its range first")
            continue
        for Speed in Speeds or JointSpeeds(Joint):
            for Run in RunNames:
                Points = [0.0, Up, -Down, 0.0] if Up and Down else ([0.0, Up, 0.0] if Up else [0.0, -Down, 0.0])
                for A, B in zip(Points, Points[1:]):
                    if A != B:
                        Segments.append(Segment("sweep", Joint, A, B, Speed, Run))
                        Segments.append(Segment("pause", Joint, B, B, Seconds=SettleS))
    for Joint in Spin:
        if Joint not in Joints:
            continue
        Below, Above = JointRoom(Joint, PresentRad[Joint], Limits)
        Sign = 1.0 if Above >= Below else -1.0
        if max(Above, Below) < SpinTravelDeg + 15.0:
            Problems.append(f"{Joint.replace('_', ' ')} needs {SpinTravelDeg + 15:.0f} deg of room on one side for its spin-down")
            continue
        for Speed in SpinSpeeds:
            Segments.append(Segment("spin", Joint, 0.0, Sign * SpinTravelDeg, Speed))
            Segments.append(Segment("watch", Joint, Seconds=SpinWatchS))
            Segments.append(Segment("return", Joint, Speed=20.0))
            Segments.append(Segment("pause", Joint, Seconds=SettleS))
    return Segments, Problems


def PlanExtremes(Segments: list[Segment]) -> dict[str, tuple[float, float]]:
    """Each joint's furthest excursion below and above its hold goal over the plan (deg; the spin's run-on included),
    for the collision check of every segment before any moves."""
    Out: dict[str, list[float]] = {}
    for S in Segments:
        if S.Joint is None or S.Kind not in ("sweep", "spin"):
            continue
        Low, High = Out.setdefault(S.Joint, [0.0, 0.0])
        Ends = [S.FromDeg, S.ToDeg + (math.copysign(15.0, S.ToDeg) if S.Kind == "spin" else 0.0)]
        Out[S.Joint] = [min(Low, *Ends), max(High, *Ends)]
    return {Joint: (Low, High) for Joint, (Low, High) in Out.items()}


@dataclass
class Sample:
    Joint: str
    Speed: float
    Run: str
    Direction: int                # +1: goal moving toward increasing raw
    Pos: dict                     # every joint's position, steps
    Load: float                   # the swept joint's Present_Load, duty units
    Volts: float


def PriorEffort(Raw: dict[str, float], Volts: float, ReferenceRaw: dict[str, int]) -> dict[str, float]:
    """The URDF prior's holding effort of lift, elbow and wrist at Raw: duty units at Volts, + toward increasing raw."""
    T = L.GravityModel().torques(*(Urdf(J, Raw[J], ReferenceRaw) for J in L.COMP_JOINTS))
    Kd = L.kd_nm_per_unit(Volts)
    return {J: T[J] / Kd for J in L.COMP_JOINTS}


class CalibrationRun:
    """The sweeps as a state machine the guide loop steps every cycle (10 ms): Step(t, readings) returns
    {"writes": {joint: (goal, limit)}, "release": [joints], "done": bool, "error": str | None}. Every joint not being
    swept keeps its hold goal; the swept one's goal ramps from its hold goal. Readings: {joint: {"pos", "load", "volts",
    "temp_c", "status"} or None} (GuideBus.ReadPresent). ReferenceRaw: the arm's zero pose (the prior's gravity, for the
    limits and the stop's hold); Settings: each servo's loop settings (GuideBus.ReadSettings: p, punch, dead_zone).
    Positions are kept in the run's own continuous frame (Unw; it starts at StartPos); a goal G goes out as the servo's
    last reported position plus G - Unw (frame independent). A stop holds where the arm is: HoldHere."""

    def __init__(Self, Segments: list[Segment], HoldGoals: dict[str, float], StartPos: dict[str, float],
                 ReferenceRaw: dict[str, int] | None = None, Settings: dict[str, dict] | None = None):
        Self.Segments = list(Segments)
        Self.Start = dict(StartPos)
        # the entry hold goals, each the short way from where its joint reads (a goal register a turn away from a
        # wrapping servo's reading is the same angle)
        Self.Hold = {N: Self.Start[N] + L.wrap_steps(G - Self.Start[N]) for N, G in HoldGoals.items()}
        Self.ReferenceRaw = dict(L.REFERENCE_RAW if ReferenceRaw is None else ReferenceRaw)
        Self.Settings = dict(Settings or {})
        Self.Raw, Self.Unw = dict(StartPos), dict(StartPos)   # as last reported; the same in the run's frame
        Self.SeenAt: dict[str, float] = {}
        Self.Jumps = 0                                        # cycles in a row with a position reading left out
        Self.OffPath: dict[str, int] = {}                     # readings in a row each joint was off its goal's path
        Self.HeldRaw: dict[str, float] = {}                   # the positions a stop held at (HoldHere), as reported
        Self.LastVolts = 8.9
        Self.Index = -1
        Self.SegmentStart = 0.0
        Self.Goals = dict(Self.Hold)
        Self.Samples: list[Sample] = []
        Self.SpinDowns: dict[str, dict[float, float]] = {}
        Self.Volts: list[float] = []
        Self.Released: set = set()
        Self.SpinFrom: float | None = None
        Self.TurnPos: float | None = None
        Self.Done = False
        Self.Error: str | None = None
        Self.Total = sum(Self.Duration(S) for S in Self.Segments)
        Self.Elapsed = 0.0

    # -- frame, limits, the stop's hold ------------------------------------------------------------------------------------
    @staticmethod
    def JumpSteps(Seconds: float) -> float:
        """The furthest any joint can plausibly move in Seconds (at least a 10 ms cycle), steps."""
        return JumpDegS * 4096.0 / 360.0 * max(Seconds, 0.01) + 4.0

    def _Accept(Self, Name: str, t: float, Raw: float) -> bool:
        """Take a position reading into the run's frame (a reading a turn away is the frame wrapping); False: implausible."""
        D = L.wrap_steps(Raw - Self.Raw[Name])
        if abs(D) > Self.JumpSteps(t - Self.SeenAt.get(Name, t - 0.05)):
            return False
        Self.Raw[Name], Self.Unw[Name], Self.SeenAt[Name] = Raw, Self.Unw[Name] + D, t
        return True

    def _Out(Self, Name: str, Goal: float) -> float:
        return Goal + Self.Raw[Name] - Self.Unw[Name]

    def _SupplyVolts(Self, Readings: dict) -> float | None:
        """The comp joints' median plausible supply reading (one corrupted reply cannot move it)."""
        V = sorted(R["volts"] for N, R in Readings.items() if N in L.COMP_JOINTS and R is not None
                   and R["volts"] >= L.GuideParams().volt_min)
        return V[len(V) // 2] if V else None

    def Limit(Self, Name: str, S: Segment | None, Gravity: dict[str, float]) -> float:
        """Torque_Limit for Name on segment S: what it is expected to need, the URDF prior's gravity and the motor's
        back-EMF at its goal's speed, with SweepLoadShare over for the friction that grows with it and SweepMarginUnits on
        top, at most SweepCeilingUnits (the gripper GripperSweepUnits, except in its fast, tracking-checked spin).
        Whatever pushes back harder stops the joint, and the tracking check then stops the calibration."""
        Speed = S.Speed if S is not None and S.Joint == Name and S.Kind in ("sweep", "spin", "return") else 0.0
        Expected = abs(Gravity.get(Name, 0.0)) + L.kv_prior(Self.LastVolts) * Speed * DEG
        Spinning = S is not None and S.Joint == Name and S.Kind == "spin"
        Ceiling = GripperSweepUnits if Name == "gripper" and not Spinning else SweepCeilingUnits
        return round(min(Ceiling, Expected * (1.0 + SweepLoadShare) + SweepMarginUnits), 1)

    def HoldHere(Self, t: float, Fresh: dict[str, dict | None] | None = None) -> dict[str, float]:
        """The goals of any stop (off the path, too hot, Stop and hold, the page gone, done): every joint where it is
        (Fresh where plausible, else its last reading), lift, elbow and wrist with the URDF prior's gravity effort on top
        so nothing sags, each an offset from the position its servo reported. Not the sweep's goal: a blocked joint held
        there kept pushing into what stopped it, at full torque, for as long as it held. HeldRaw: the positions used."""
        Raw, Unw = dict(Self.Raw), dict(Self.Unw)
        for Name, R in (Fresh or {}).items():
            if R is not None and Name in Raw:
                D = L.wrap_steps(R["pos"] - Self.Raw[Name])
                if abs(D) <= Self.JumpSteps(t - Self.SeenAt.get(Name, t - 0.05)):
                    Raw[Name], Unw[Name] = R["pos"], Self.Unw[Name] + D
        Volts = Self._SupplyVolts(Fresh or {}) or Self.LastVolts
        Effort = PriorEffort(Unw, Volts, Self.ReferenceRaw)
        Self.HeldRaw = Raw
        Out = {}
        for Name in Raw:
            Loop = Self.Settings.get(Name) or {"p": 32, "punch": 16, "dead_zone": [1, 1]}
            Steps = L.effort_steps(Effort.get(Name, 0.0), Loop["p"] / 4.0, float(Loop["punch"]), sum(Loop["dead_zone"]) / 2.0)
            Out[Name] = Raw[Name] + Steps
        return Out

    @staticmethod
    def Duration(S: Segment) -> float:
        if S.Kind in ("sweep", "spin"):
            return abs(S.ToDeg - S.FromDeg) / S.Speed
        if S.Kind == "return":
            return SpinTravelDeg / S.Speed
        return S.Seconds

    def Progress(Self) -> float:
        return 1.0 if Self.Done else min(0.99, Self.Elapsed / max(1e-6, Self.Total))

    def _Next(Self, t: float) -> None:
        if Self.Index >= 0:
            Self.Elapsed += Self.Duration(Self.Segments[Self.Index])
        Self.Index += 1
        Self.SegmentStart = t
        Self.TurnPos = None
        if Self.Index >= len(Self.Segments):
            Self.Done = True

    def Step(Self, t: float, Readings: dict[str, dict | None]) -> dict:
        Out = {"writes": {}, "release": [], "done": False, "error": None}
        if Self.Done or Self.Error:
            Out.update(done=Self.Done, error=Self.Error)
            return Out
        Silent = [J for J, R in Readings.items() if R is None]
        if Silent:
            Self.Error = f"{', '.join(Silent)} did not answer during the calibration"
            Out["error"] = Self.Error
            return Out
        # positions into the run's frame; a reading that implies more than JumpDegS is a missed reply (every servo keeps
        # its goal), and two such cycles in a row stop it
        Jumped = [J for J, R in Readings.items() if not Self._Accept(J, t, float(R["pos"]))]
        if Jumped:
            Self.Jumps += 1
            if Self.Jumps >= 2:
                Self.Error = (f"{', '.join(J.replace('_', ' ') for J in Jumped)} position jumped on two readings in a row "
                              "(a restart, or a loose cable?)")
                Out["error"] = Self.Error
            return Out
        Self.Jumps = 0
        # the supply (a reading under the 7.0 V floor is left out: the bridge goes limp if it persists)
        Plausible = [R["volts"] for R in Readings.values() if R["volts"] >= L.GuideParams().volt_min]
        if Plausible:
            Self.Volts.append(min(Plausible))
        Self.LastVolts = Self._SupplyVolts(Readings) or Self.LastVolts
        if Self.Index < 0:
            Self._Next(t)
        S = Self.Segments[Self.Index] if not Self.Done else None
        Tau = t - Self.SegmentStart
        if S is not None and Tau >= Self.Duration(S) and S.Kind != "watch":
            Self._Finish(S, Readings)
            Self._Next(t)
            S = Self.Segments[Self.Index] if not Self.Done else None
            Tau = 0.0
        elif S is not None and S.Kind == "watch" and Tau >= S.Seconds:
            Self._Finish(S, Readings)
            Self._Next(t)
            S = Self.Segments[Self.Index] if not Self.Done else None
            Tau = 0.0
        if Self.Done:
            Out["done"] = True
            return Out
        J = S.Joint
        Steps = 4096.0 / 360.0
        Sweeping = None
        if S.Kind == "sweep":
            Deg = S.FromDeg + math.copysign(min(abs(S.ToDeg - S.FromDeg), S.Speed * Tau), S.ToDeg - S.FromDeg)
            Self.Goals[J] = Self.Hold[J] + Deg * Steps
            Sweeping = Deg
        elif S.Kind == "spin":
            Deg = math.copysign(min(abs(S.ToDeg), S.Speed * Tau), S.ToDeg)
            Self.Goals[J] = Self.Hold[J] + Deg * Steps
        elif S.Kind == "watch":
            if J not in Self.Released:
                Self.Released.add(J)
                Self.SpinFrom = Self.Unw[J]
                Out["release"] = [J]
        elif S.Kind == "return":
            if J in Self.Released:
                # energised again where it stopped, then back to its hold goal at 20 deg/s
                Self.Released.discard(J)
                Self.Goals[J] = Self.Unw[J]
                Self.ReturnFrom = Self.Unw[J]
            Frac = min(1.0, Tau / Self.Duration(S))
            Self.Goals[J] = Self.ReturnFrom + (Self.Hold[J] - Self.ReturnFrom) * Frac
        # the tracking of every energised joint, on every segment (the spins too): how far it is from where its goal
        # has been taken (from the start's offset). Two readings in a row off it: blocked, or a hand is on it.
        Gravity = PriorEffort(Self.Unw, Self.LastVolts, Self.ReferenceRaw)
        for N, Goal in Self.Goals.items():
            if N in Self.Released:
                Self.OffPath.pop(N, None)
                continue
            Off = (Self.Unw[N] - Self.Start[N]) - (Goal - Self.Hold[N])
            Self.OffPath[N] = Self.OffPath.get(N, 0) + 1 if abs(Off) > TrackLimitSteps else 0
            if Self.OffPath[N] >= 2:
                Where = "its calibration sweep" if N == J and S.Kind in ("sweep", "spin", "return") else "where the calibration holds it"
                Self.Error = (f"{N.replace('_', ' ')} is {abs(Off):.0f} steps off {Where} (more than {TrackLimitSteps}): "
                              f"something blocks it, a hand is on it, or it needs more than the {Self.Limit(N, S, Gravity):.0f} "
                              "units of effort the calibration allows it")
                Out["error"] = Self.Error
                return Out
        if Sweeping is not None:
            Travelled = abs(Sweeping - S.FromDeg)
            if Travelled >= SkipAfterTurnDeg and Travelled <= abs(S.ToDeg - S.FromDeg):
                Self.Samples.append(Sample(J, S.Speed, S.Run, 1 if S.ToDeg > S.FromDeg else -1,
                                           {N: R["pos"] for N, R in Readings.items()}, float(Readings[J]["load"]),
                                           float(Readings[J]["volts"])))
        Out["writes"] = {N: (Self._Out(N, G), Self.Limit(N, S, Gravity)) for N, G in Self.Goals.items()
                         if N not in Self.Released}
        return Out

    def _Finish(Self, S: Segment, Readings: dict[str, dict]) -> None:
        if S.Kind == "watch" and Self.SpinFrom is not None:
            Speed = Self.Segments[Self.Index - 1].Speed
            Self.SpinDowns.setdefault(S.Joint, {})[Speed] = abs(Self.Unw[S.Joint] - Self.SpinFrom) * 360.0 / 4096.0
            Self.SpinFrom = None
        if S.Kind == "return":
            Self.Goals[S.Joint] = Self.Hold[S.Joint]


# --------------------------------------------------------------------------------------------- the fit
def Finite(Value):
    """Value with every NaN or infinity as None: the report goes into JSON (the state, the file, the events log)."""
    if isinstance(Value, float):
        return Value if math.isfinite(Value) else None
    if isinstance(Value, dict):
        return {Key: Finite(Item) for Key, Item in Value.items()}
    if isinstance(Value, (list, tuple)):
        return [Finite(Item) for Item in Value]
    return Value


def Basis(Q2: float, Q3: float, Q4: float) -> np.ndarray:
    T2, T3, T4 = Q2, Q2 + Q3, Q2 + Q3 + Q4
    return np.array([math.cos(T2), math.sin(T2), math.cos(T3), math.sin(T3), math.cos(T4), math.sin(T4)])


def JointRow(Joint: str, X: np.ndarray) -> np.ndarray:
    """The basis row of Joint's holding torque: lift all six terms, elbow the last four, wrist the last two."""
    Row = X.copy()
    if Joint == "elbow_flex":
        Row[:2] = 0.0
    elif Joint == "wrist_flex":
        Row[:4] = 0.0
    return Row


def Urdf(Joint: str, Raw: float, ReferenceRaw: dict[str, int]) -> float:
    return ((Raw - ReferenceRaw[Joint] + 2048.0) % 4096.0 - 2048.0) * L.STEP_RAD


def Bands(Samples: list[Sample], Joint: str, VCal: float, ReferenceRaw: dict[str, int]) -> dict:
    """Per (speed, run): per 2-deg bin of the joint's angle, the median load moving up and down (duty units at VCal) and
    the bin's mean comp-joint angles. {(speed, run): {bin: {"up", "down", "q"}}}."""
    Groups: dict = {}
    for S in Samples:
        if S.Joint != Joint:
            continue
        Q = {N: Urdf(N, S.Pos[N], ReferenceRaw) for N in L.COMP_JOINTS}
        Bin = round(math.degrees(Urdf(Joint, S.Pos[Joint], ReferenceRaw)) / BinDeg)
        Cell = Groups.setdefault((S.Speed, S.Run), {}).setdefault(Bin, {"up": [], "down": [], "q": []})
        Cell["up" if S.Direction > 0 else "down"].append(S.Load * S.Volts / VCal)
        Cell["q"].append([Q[N] for N in L.COMP_JOINTS])
    Out: dict = {}
    for Key, Bins in Groups.items():
        for Bin, Cell in Bins.items():
            if len(Cell["up"]) >= 2 and len(Cell["down"]) >= 2:
                Out.setdefault(Key, {})[Bin] = {"up": float(np.median(Cell["up"])), "down": float(np.median(Cell["down"])),
                                                 "q": np.mean(Cell["q"], axis=0)}
    return Out


def Fit(Samples: list[Sample], SpinDowns: dict[str, dict[float, float]], Volts: list[float], Fingerprint: dict,
        ReferenceRaw: dict[str, int], Joints=SweepOrder) -> tuple[dict | None, list[str], dict]:
    """(the calibration data to save, or None; the reasons it was not accepted; the measurements, for the report)."""
    Problems: list[str] = []
    Report: dict[str, Any] = {"joints": {}}
    if not Samples:
        return None, ["the calibration recorded nothing"], Report
    VCal = float(np.mean([S.Volts for S in Samples]))
    Kd = L.kd_nm_per_unit(VCal)
    Prior = L.kv_prior(VCal)
    Report["v_cal"] = round(VCal, 2)
    Spread = (max(Volts) - min(Volts)) if Volts else 0.0
    Report["supply_spread_v"] = round(Spread, 2)
    if Spread > SupplyHeldV:
        Problems.append(f"the servo supply moved {Spread:.1f} V during the calibration (at most {SupplyHeldV:.1f} V): "
                        "check the 12 V supply and its cable, then calibrate again")
    Joints_: dict[str, dict] = {}
    Rows, Ys, RowPose = [], [], []
    BandsBy: dict[str, dict] = {}
    for Joint in [J for J in Joints if J in L.COMP_JOINTS + L.FREE_JOINTS]:
        Slow, Fast = JointSpeeds(Joint)
        Dw = (Fast - Slow) * DEG
        B = Bands(Samples, Joint, VCal, ReferenceRaw)
        BandsBy[Joint] = B
        Half = {Key: {Bin: (C["up"] - C["down"]) / 2 for Bin, C in Bins.items()} for Key, Bins in B.items()}
        Med = {Key: float(np.median(list(V.values()))) if V else float("nan") for Key, V in Half.items()}
        Kv = {}
        for Run in Runs:
            if (Slow, Run) in Med and (Fast, Run) in Med:
                Kv[Run] = (Med[(Fast, Run)] - Med[(Slow, Run)]) / Dw
        Entry = {"half_band_slow": {Run: round(Med.get((Slow, Run), float("nan")), 1) for Run in Runs},
                 "half_band_fast": {Run: round(Med.get((Fast, Run), float("nan")), 1) for Run in Runs},
                 "k_v_runs": {Run: round(V, 1) for Run, V in Kv.items()}}
        Report["joints"][Joint] = Entry
        Name = Joint.replace("_", " ")
        if len(Kv) < 2:
            Problems.append(f"{Name}: not enough of its sweeps were recorded to measure it")
            continue
        K = float(np.mean(list(Kv.values())))
        Entry["k_v"] = round(K, 1)
        if abs(Kv["A"] - Kv["B"]) > KvRunsAgree * abs(K):
            Problems.append(f"{Name}: the two runs' velocity slopes differ ({Kv['A']:.0f} and {Kv['B']:.0f}, more than "
                            f"{KvRunsAgree * 100:.0f} % apart)")
        if not L.GuideParams().kv_prior_band[0] <= K / Prior <= L.GuideParams().kv_prior_band[1]:
            Problems.append(f"{Name}: velocity slope {K:.0f} is not within 0.75-1.10 of the motor's {Prior:.0f}")
        Agree = RunsAgreeComp if Joint in L.COMP_JOINTS else RunsAgreeFree
        SlowA, SlowB = Med[(Slow, "A")], Med[(Slow, "B")]
        if abs(SlowA - SlowB) > Agree * max(abs(SlowA), abs(SlowB), 1e-6):
            Problems.append(f"{Name}: the two runs' frictions differ ({SlowA:.0f} and {SlowB:.0f} units, more than "
                            f"{Agree * 100:.0f} % apart)")
        Joints_[Joint] = {"k_v": round(K, 1)}
        if Joint in L.FREE_JOINTS:
            Fk = (SlowA + SlowB) / 2 - K * Slow * DEG
            Entry["f_kin"] = round(Fk, 1)
            if Fk <= 0:
                Problems.append(f"{Name}: no friction measured ({Fk:.0f} units)")
            Joints_[Joint].update(f_kin=round(max(Fk, 0.0), 2), c_lead=LeadDefault["free"], k_f=FrictionAssistDefault,
                                  v_hi_deg_s=round(math.degrees(L.FREE_DEFAULTS[Joint]["v_hi"])))
        else:
            for (Speed, Run), Bins in B.items():
                for Bin, C in Bins.items():
                    Rows.append(JointRow(Joint, Basis(*C["q"])))
                    Ys.append((C["up"] + C["down"]) / 2 * Kd)
                    RowPose.append((Joint, C["q"]))
    # the gravity model: MAP regression of the band centres, the URDF as the prior
    P0 = np.array(L.URDF_P0)
    if all(J in Joints_ for J in L.COMP_JOINTS) and Rows:
        X, Y = np.array(Rows), np.array(Ys)
        Sd = SampleSdUnits * Kd
        PriorSd = np.maximum(PriorShare * np.abs(P0), PriorFloorUnits * Kd)
        A = X.T @ X / Sd ** 2 + np.diag(1.0 / PriorSd ** 2)
        Rhs = X.T @ Y / Sd ** 2 + P0 / PriorSd ** 2
        P = np.linalg.solve(A, Rhs)
        Residual = float(np.sqrt(np.mean((X @ P - Y) ** 2)) / Kd)
        Report["model"] = [round(float(V), 6) for V in P]
        Report["residual_units"] = round(Residual, 1)
        if Residual > ResidualMaxUnits:
            Problems.append(f"the gravity fit leaves {Residual:.0f} units RMS (at most {ResidualMaxUnits:.0f}): something "
                            "pushed on the arm, or its load is not what the model can describe")
        # the fitted model against the URDF at every pose the sweeps visited
        Worst = None
        for _, Q in RowPose:
            Xq = Basis(*Q)
            for J in L.COMP_JOINTS:
                M, Pr = JointRow(J, Xq) @ P / Kd, JointRow(J, Xq) @ P0 / Kd
                Bad = (not AcceptRatio[0] <= M / Pr <= AcceptRatio[1]) if abs(Pr) > AcceptFloorUnits else abs(M - Pr) > AcceptAbsUnits
                if Bad and Worst is None:
                    Worst = f"{J.replace('_', ' ')} needs {M:.0f} units where the URDF says {Pr:.0f}"
        if Worst:
            Problems.append(f"the fitted model disagrees with the URDF ({Worst}): check that nothing pushed on the arm, "
                            "that Present_Load reads with the servos' sign (bring-up S3), and calibrate again")
        # friction of the comp joints: what is left of the slow half-band, against the carried load
        for J in L.COMP_JOINTS:
            Fk, Load = [], []
            Slow = JointSpeeds(J)[0]
            for (Speed, Run), Bins in BandsBy[J].items():
                if Speed != Slow:
                    continue
                for Bin, C in Bins.items():
                    Fk.append((C["up"] - C["down"]) / 2 - Joints_[J]["k_v"] * Slow * DEG)
                    Load.append(abs(JointRow(J, Basis(*C["q"])) @ P / Kd))
            Fk_, Load_ = np.array(Fk), np.array(Load)
            if len(Fk_) >= 2 and np.ptp(Load_) > 1.0:
                F, F0 = np.polyfit(Load_, Fk_, 1)
            else:
                F, F0 = 0.0, float(np.mean(Fk_)) if len(Fk_) else 0.0
            if F < 0:
                F, F0 = 0.0, float(np.mean(Fk_))
            if F0 < 0:
                F0, F = 0.0, float(Fk_ @ Load_ / max(1e-9, Load_ @ Load_))
            Report["joints"][J].update(f_kin0=round(float(F0), 1), f_kin_load=round(float(F), 3))
            if float(np.median(Fk_)) <= 0:
                Problems.append(f"{J.replace('_', ' ')}: no friction measured")
            Joints_[J].update(f_kin0=round(float(F0), 2), f_kin_load=round(float(F), 4), c_lead=LeadDefault["comp"],
                              k_f=FrictionAssistDefault)
    elif any(J in L.COMP_JOINTS for J in Joints):
        Problems.append("the shoulder, elbow and wrist were not all measured: the gravity model needs all three")
        P = None
    else:
        P = None
    # the spin-down: does a servo with its torque off float or brake?
    Verdicts = {}
    for J, RunOn in SpinDowns.items():
        Low, High = RunOn.get(SpinSpeeds[0]), RunOn.get(SpinSpeeds[1])
        if Low is None or High is None:
            continue
        Ratio = High / max(Low, 0.05)
        Verdicts[J] = "floats" if Ratio >= FloatRatio and High >= FloatRunOnDeg else "brakes"
        Report.setdefault("spin_down", {})[J] = {"run_on_deg": {str(int(K)): round(V, 2) for K, V in RunOn.items()},
                                                 "ratio": round(Ratio, 2), "verdict": Verdicts[J]}
    if len(set(Verdicts.values())) > 1:
        Problems.append("the roll and the gripper disagree on whether a servo with its torque off floats or brakes "
                        f"({Verdicts}): calibrate again")
    if any(J in Joints for J in SpinJoints) and not Verdicts:
        Problems.append("the spin-down was not measured")
    Report = Finite(Report)
    if Problems or P is None:
        return None, Problems, Report
    Data = {"Version": CalibrationVersion, "HandGuideVersion": "guide-v1",
            "CalibratedAt": time.strftime("%Y-%m-%dT%H:%M:%S"), "VCal": round(VCal, 3),
            "Model": [float(V) for V in P], "Alpha": AlphaDefault, "Joints": Joints_,
            "CoastBrakes": next(iter(Verdicts.values())) == "brakes" if Verdicts else False,
            "Fingerprint": Fingerprint, "Report": Report}
    return Data, [], Report
