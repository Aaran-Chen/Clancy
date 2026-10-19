"""Plan real arm requests with the bridge's own planner and fly them on simulated servos. Offline only.

Never opens a serial port: a FakeBus (test_bridge_home) stands in for the servos, and the depth
camera opener and the priority raise are guarded the way TestSafety guards them. Each case starts
from a raw pose measured on the arm and sends the request the app (or calibration) sent, taken
from the session records in captures/ (2026-10-03 recording, 2026-10-02 21:06 calibration run).

    python MeasureRouteSpeed.py                   # the table at the default 18% and at 100%
    python MeasureRouteSpeed.py --case hover      # only cases whose name contains "hover"
    python MeasureRouteSpeed.py --json Out.json   # also write every number
    python MeasureRouteSpeed.py --trace hover     # per-frame stage and lead-joint command

Columns
  plan s     wall clock of the bridge call (send_ik / SendJointMotion / go_home): IK, planning,
             certification and reach assignment
  stages     route waypoints after the start; in brackets the tight ones (no waypoint reach, approached
             slowly) and the arrive ones (approached with the reach, every joint arrives before the next stage)
  joint x    arm-joint path length (Euclidean degrees, wrist roll included) over the straight
             joint-space line between the same ends
  tip x      tip path length along the stages (sampled) over the straight tip distance; the
             straight joint-space line's own value in brackets (no route can do better than that
             and still be a joint-space line)
  flown      the tip path the simulated arm actually flew at 100%, over the straight tip distance:
             joints that start, stop and wait out of step make it wander off the planned line
  run s      simulated execution, route start to settled at the end, at each speed, on the
             nominal servo model; "trip" if the route monitor or protection stopped it
  harsh      one mark per harsh servo model at 100%: "." completed, "T" the route monitor or
             protection stopped it (a regression check against the same cases before a change)
  peak/ceil  highest commanded Goal_Speed over that joint's ceiling at the speed, worst joint

Servo models (30 Hz frames, like the bridge):
  nominal    commands take effect 2 frames late; 5100 steps/s^2 (the Acceleration register the
             bridge writes, 51); each joint at most its spec speed at 8.9 V (Cad/MotionSpeed.md:
             lift 950, elbow 1500, wrist flex 1970, the rest 2280 steps/s)
  late       5 frames late, braking only 1000 steps/s^2
  sticky     3 frames late, braking 1000 steps/s^2, the shoulder lift 15% under its spec speed;
             from rest the lift sticks 0.4 s and the elbow 0.2 s, then each lurches to 1.5x its
             command for 0.1 s
  fast       every velocity command overshot by 30%
  late+fast  4 frames late, braking 1000 steps/s^2, 20% over every velocity command
"""
from __future__ import annotations

import argparse
import collections
import json
import math
import sys
import time
from pathlib import Path
from unittest.mock import patch

import numpy as np

import BasketCollision as Collision
import bridge_server as Bridge
from feetech_simple import OperatingMode
from test_bridge_home import FakeBus

FrameSeconds = 1 / 30
Names = Bridge.MOTOR_NAMES
ArmNames = Names[:5]
# Measured on the arm: Home with the jaws closed (2026-10-03 bridge fault records, 14:47 and 15:57).
Home = {"shoulder_pan": 3474, "shoulder_lift": 3171, "elbow_flex": 2688, "wrist_flex": 855, "wrist_roll": 1570, "gripper": 703}
# Where the planner (flyby-directional-v6) put the arm for those hover and grasp poses from Home.
HoverCan = {"shoulder_pan": 3157, "shoulder_lift": 188, "elbow_flex": 1989, "wrist_flex": 597, "wrist_roll": 1570, "gripper": 1919}
HoverCanLower = {"shoulder_pan": 3146, "shoulder_lift": 201, "elbow_flex": 2011, "wrist_flex": 563, "wrist_roll": 1570, "gripper": 1919}
AtCan = {"shoulder_pan": 3146, "shoulder_lift": 433, "elbow_flex": 2089, "wrist_flex": 251, "wrist_roll": 1570, "gripper": 648}
Lifted = {"shoulder_pan": 3146, "shoulder_lift": 185, "elbow_flex": 1930, "wrist_flex": 710, "wrist_roll": 1570, "gripper": 648}
# Upright beside the mast, inside the mast's tracking allowance (TestFlyByRoutes.BesideMast, in raw steps).
BesideMastAngles = {"shoulder_pan": -26.54, "shoulder_lift": 103.54, "elbow_flex": 5.19, "wrist_flex": -37.62,
                    "wrist_roll": 100.9, "gripper": 0.09}


def Raw(Values):
    return dict(zip(Names, Values))


# (name, start raw, request). A tip request is ("tip", AI display X, Y, Z, wrist pitch, gripper raw or None);
# the AI display frame mirrors X (TestBasketCollision.DisplayToModel); ("model", ...) is in the bridge's model
# frame. A joint request is ("joints", goal raw), as calibration sends its planned poses; ("home",) is Home.
Cases = [
    ("hover from home (can, 14:49)", {**Home, "gripper": 1919}, ("tip", 1.9, 28.6, 11.3, -21.3, None)),
    ("hover from home (brief, 14:46)", {**Home, "gripper": 1919}, ("tip", 1.9, 28.9, 11.5, -21.3, None)),
    ("hover from home opening jaws (pill, 15:46)", Home, ("tip", 2.8, 31.1, 9.0, -32.4, 1602)),
    ("nudge at hover (14:50:00)", HoverCan, ("tip", 2.4, 28.3, 11.3, -21.3, None)),
    ("descent to can (14:50:27)", HoverCanLower, ("tip", 1.899, 28.646, 4.549, -21.3, None)),
    ("lift with can (14:52)", AtCan, ("tip", 1.986, 28.429, 10.137, -22.412, None)),
    ("home from hover", Lifted, ("home",)),
    ("table target from beside mast (model frame)", "BesideMast", ("model", 3.7339702738078357, 43.8955547489971, 4.261681908293165,
                                                                   -2.096338539012045, None)),
    # Into the tray through its rim, from the saved Home (TestCartesianReachability.InteriorTarget, model frame).
    ("into the tray over the rim (test target)", {}, ("model", -1.9532211879722914, 14.308657609320962, -5.741477864378066,
                                                      -108.0, None)),
    # 2026-10-02 21:06 calibration run: start_raw and the route's final waypoint (BasketRoute records).
    ("calib 1: home to first pose (8 stages)", Raw((3473, 3172, 2688, 862, 1570, 704)), ("joints", Raw((3501, 119, 1995, 555, 1570, 704)))),
    ("calib 2: 8 cm sideways", Raw((3494, 121, 2006, 563, 1570, 704)), ("joints", Raw((3340, 152, 1960, 557, 1570, 704)))),
    ("calib 6: 4 cm down (2 stages)", Raw((3492, 274, 1937, 482, 1570, 704)), ("joints", Raw((3501, 211, 2104, 353, 1570, 704)))),
    ("calib 7: 5 cm out, wrist +20 (3 stages)", Raw((3492, 219, 2107, 361, 1570, 704)), ("joints", Raw((3495, 492, 1803, 146, 1570, 704)))),
    ("calib 12: 4 cm down by the rim (3 stages)", Raw((3123, 575, 1798, 84, 1570, 704)), ("joints", Raw((3112, 722, 1768, 4048, 1570, 704)))),
    ("calib 13: 4 cm sideways low (2 stages)", Raw((3123, 727, 1777, 4055, 1570, 704)), ("joints", Raw((3035, 746, 1718, 4073, 1570, 704)))),
    ("calib 16: up and out (2 stages)", Raw((2844, 765, 1702, 4084, 1570, 704)), ("joints", Raw((2868, 737, 1610, 94, 1570, 704)))),
    ("calib 18: 4 cm up (2 stages)", Raw((2952, 727, 1644, 86, 1570, 704)), ("joints", Raw((2962, 601, 1627, 213, 1570, 704)))),
]

SpecSpeed = {"shoulder_pan": 2280, "shoulder_lift": 950, "elbow_flex": 1500, "wrist_flex": 1970, "wrist_roll": 2280, "gripper": 2280}


class ServoModel:
    """A FakeBus's servos, stepped once per 30 Hz frame: command latency, acceleration and braking, caps, stiction."""

    def __init__(Self, Bus, Caps, LatencyFrames=2, Acceleration=5100.0, Braking=5100.0, Over=1.0, Stick=None):
        Self.Bus, Self.Caps, Self.Latency = Bus, Caps, LatencyFrames
        Self.Acceleration, Self.Braking, Self.Over = Acceleration, Braking, Over
        # Joint -> (seconds stuck when starting from rest, lurch factor over the command, lurch seconds).
        Self.Stick = Stick or {}
        Self.Exact, Self.Velocity, Self.Queue, Self.Stuck = {}, {}, {}, {}

    def Step(Self):
        Bus = Self.Bus
        for Name in Names:
            Command = (Bus.reg(Name, "Operating_Mode"), Bus.reg(Name, "Goal_Speed"), Bus.reg(Name, "Goal_Position"))
            Queue = Self.Queue.setdefault(Name, collections.deque([Command] * (Self.Latency + 1), maxlen=Self.Latency + 1))
            Queue.append(Command)
            Mode, Speed, Goal = Queue[0]
            Exact = Self.Exact.get(Name, float(Bus.position[Name]))
            if round(Exact) % 4096 != Bus.position[Name]:
                Exact = float(Bus.position[Name])
            Velocity = Self.Velocity.get(Name, 0.0)
            if Mode == OperatingMode.VELOCITY.value:
                Wanted = Speed * Self.Over
            else:
                Error = (Goal - Exact + 2048) % 4096 - 2048
                Cap = abs(Speed) or 3000
                Wanted = max(-Cap, min(Cap, Error * 6.0))
            if Name in Self.Stick:
                # Starting from rest the joint sticks, then breaks away above its command.
                Seconds, Lurch, LurchSeconds = Self.Stick[Name]
                Stuck = Self.Stuck.get(Name)
                if abs(Wanted) < 1:
                    Stuck = Self.Stuck[Name] = None
                elif abs(Velocity) < 1 and Stuck is None:
                    Stuck = Self.Stuck[Name] = [0.0]
                if Stuck is not None:
                    Stuck[0] += FrameSeconds
                    if Stuck[0] <= Seconds:
                        Wanted = 0.0
                    elif Stuck[0] <= Seconds + LurchSeconds:
                        Wanted *= Lurch
                    elif abs(Velocity) >= 1:
                        Self.Stuck[Name] = None
            Wanted = max(-Self.Caps[Name], min(Self.Caps[Name], Wanted))
            Speeding = abs(Wanted) > abs(Velocity) and Wanted * Velocity >= 0
            Limit = (Self.Acceleration if Speeding else Self.Braking) * FrameSeconds
            Velocity += max(-Limit, min(Limit, Wanted - Velocity))
            Exact += Velocity * FrameSeconds
            Self.Exact[Name], Self.Velocity[Name] = Exact, Velocity
            Bus.position[Name] = int(round(Exact)) % 4096


def NominalModel(Bus):
    return ServoModel(Bus, SpecSpeed)


# Each stresses the route rules on its own (Cad/BasketAvoidance.md, Execution), at 100%.
HarshModels = {
    "late": lambda Bus: ServoModel(Bus, SpecSpeed, LatencyFrames=5, Braking=1000.0),
    "sticky": lambda Bus: ServoModel(Bus, {**SpecSpeed, "shoulder_lift": SpecSpeed["shoulder_lift"] * 0.85}, LatencyFrames=3,
                                     Braking=1000.0, Stick={"shoulder_lift": (0.4, 1.5, 0.1), "elbow_flex": (0.2, 1.5, 0.1)}),
    "fast": lambda Bus: ServoModel(Bus, SpecSpeed, Over=1.3),
    "late+fast": lambda Bus: ServoModel(Bus, SpecSpeed, LatencyFrames=4, Braking=1000.0, Over=1.2),
}


def MakeController(Start):
    Controller = Bridge.BridgeController()
    Bus = FakeBus()
    Controller.bus, Controller.port, Controller.torque_enabled = Bus, "FAKE", True
    Bus.torque_enabled = True
    if Start == "BesideMast":
        Start = {Name: Controller._ik_raw_from_angle(Controller.servos[Name], Angle) for Name, Angle in BesideMastAngles.items()}
    for Name, Servo in Controller.servos.items():
        Value = Start.get(Name, Servo.present_raw)
        Servo.present_raw = Servo.target_raw = Servo.HardwareRaw = Bus.position[Name] = Value
    Bus.configure_motors()
    Controller.refresh()
    Bus.writes.clear()
    return Controller, Bus


def Send(Controller, Request):
    if Request[0] == "home":
        return Controller.go_home()
    if Request[0] == "joints":
        return Controller.SendJointMotion(Request[1], "Measure")
    Kind, X, Y, Z, Pitch, Gripper = Request
    Point = {"x": X, "y": Y, "z": Z} if Kind == "model" else Controller.ModelPointFromVisualPoint({"x": -X, "y": Y, "z": Z})
    Target = {**Point, "wrist_pitch": Pitch}
    return Controller.send_ik(Target=Target, GripperRaw=Gripper)


def AnglesOf(Controller, Point):
    return np.array([Controller._ik_angle_from_raw(Controller.servos[Name], Point[Name]) for Name in Names])


def Tip(Controller, Angles):
    Point = Bridge.forward_tip_position(Controller.arm_geometry, dict(zip(Names, Angles)))
    return np.array([Point["x"], Point["y"], Point["z"]])


def TipLength(Controller, Points, Samples=24):
    Length = 0.0
    for A, B in zip(Points, Points[1:]):
        Tips = [Tip(Controller, A + (B - A) * Fraction) for Fraction in np.linspace(0, 1, Samples + 1)]
        Length += sum(float(np.linalg.norm(Q - P)) for P, Q in zip(Tips, Tips[1:]))
    return Length


def Geometry(Controller, Start, Path):
    Points = [AnglesOf(Controller, Start)] + [AnglesOf(Controller, Point) for Point in Path]
    Arm = [Point[:5] for Point in Points]
    Joint = sum(float(np.linalg.norm(B - A)) for A, B in zip(Arm, Arm[1:]))
    Line = float(np.linalg.norm(Arm[-1] - Arm[0]))
    Straight = float(np.linalg.norm(Tip(Controller, Points[-1]) - Tip(Controller, Points[0])))
    Along = TipLength(Controller, Points)
    Direct = TipLength(Controller, [Points[0], Points[-1]])
    return {"joint_path_deg": Joint, "joint_line_deg": Line, "joint_ratio": Joint / Line if Line > 1e-9 else 1.0,
            "tip_path_cm": Along, "tip_line_cm": Straight, "tip_ratio": Along / Straight if Straight > 0.05 else None,
            "direct_tip_ratio": Direct / Straight if Straight > 0.05 else None}


def Execute(Controller, Bus, Model, Frames=30 * 120, Trace=None, Tips=None):
    """Frames from the route start until it completes, or (None, reason) when it stopped.

    Tips, if a list, collects the tip position (model frame, cm) at every frame."""
    Peaks = {Name: 0 for Name in Names}
    for Frame in range(Frames):
        Controller.last_refresh_s = time.monotonic()  # Simulated frames run faster than real time.
        Model.Step()
        Bus.writes.clear()
        Controller.refresh()
        for Register, Name, Value in Bus.writes:
            if Register == "Goal_Speed" and Bus.reg(Name, "Operating_Mode") == OperatingMode.VELOCITY.value:
                Peaks[Name] = max(Peaks[Name], abs(Value))
        if Trace is not None:
            Route = Controller.MotionRoute
            Trace.append((Frame, None if Route is None else Route["Index"],
                          {Name: Servo.command_speed for Name, Servo in Controller.servos.items()},
                          {Name: Bus.position[Name] for Name in Names}))
        if Tips is not None:
            Tips.append(Tip(Controller, AnglesOf(Controller, {Name: Servo.present_raw for Name, Servo in Controller.servos.items()})))
        if Controller.MotionFault or not Bus.torque_enabled:
            return None, Controller.MotionFault or "torque off", Peaks
        if Controller.MotionRoute is None:
            return Frame + 1, None, Peaks
    return None, "did not finish", Peaks


def Measure(Name, Start, Request, Percents=(18, 100), Harsh=True, TraceAt=None):
    Result = {"case": Name, "planner": Collision.PlannerVersion}
    for Percent in Percents:
        Controller, Bus = MakeController(Start)
        Controller.SetMotionSpeed(Percent)
        Began = time.perf_counter()
        Ok, Message = Send(Controller, Request)
        Planning = time.perf_counter() - Began
        if not Ok:
            Result[f"refused_{Percent}"] = Message
            continue
        Route = Controller.MotionRoute
        Path, Tight = list(Route["Path"]), sorted(Route["Tight"])
        if "stages" not in Result:
            Result.update({"plan_s": Planning, "stages": len(Path), "tight": len(Tight), "arrive": len(Route.get("Arrive", ())),
                           **Geometry(Controller, Route["Start"], Path)})
            Result["waypoints_raw"] = Path
        Result[f"plan_s_{Percent}"] = Planning
        Trace = [] if TraceAt == Percent else None
        Tips = [Tip(Controller, AnglesOf(Controller, Route["Start"]))]
        Frames, Stopped, Peaks = Execute(Controller, Bus, NominalModel(Bus), Trace=Trace, Tips=Tips)
        Ceilings = Controller.MotionSpeedCeilings(Percent)
        Result[f"run_s_{Percent}"] = None if Frames is None else Frames * FrameSeconds
        # The tip path the simulated arm actually flew, over the straight tip distance (route start to end).
        Flown = sum(float(np.linalg.norm(B - A)) for A, B in zip(Tips, Tips[1:]))
        Result[f"flown_tip_ratio_{Percent}"] = Flown / Result["tip_line_cm"] if Result["tip_line_cm"] > 0.05 else None
        Result[f"stopped_{Percent}"] = Stopped
        Result[f"peak_over_ceiling_{Percent}"] = max(Peaks[Name] / Ceilings[Name] for Name in Names)
        Result[f"peaks_{Percent}"] = Peaks
        if Trace is not None:
            Result["trace"] = Trace
    if Harsh:
        Result["harsh"] = {}
        for Label, Make in HarshModels.items():
            Controller, Bus = MakeController(Start)
            Controller.SetMotionSpeed(100)
            Ok, Message = Send(Controller, Request)
            if Ok:
                Frames, Stopped, _ = Execute(Controller, Bus, Make(Bus))
                Result["harsh"][Label] = {"run_s": None if Frames is None else Frames * FrameSeconds, "stopped": Stopped}
    return Result


def Row(Result):
    if "stages" not in Result:
        return f"{Result['case'][:44]:44s}  refused: {Result.get('refused_18') or Result.get('refused_100')}"
    Tip = "-" if Result["tip_ratio"] is None else f"{Result['tip_ratio']:.2f} ({Result['direct_tip_ratio']:.2f})"
    Runs = []
    for Percent in (18, 100):
        Value = Result.get(f"run_s_{Percent}")
        Runs.append(f"{Value:6.2f}" if Value is not None else "  trip")
    Models = Result.get("harsh", {})
    Harsh = "".join("." if Models[Label]["stopped"] is None else "T" for Label in HarshModels if Label in Models) or "-"
    Peak = max(Result.get("peak_over_ceiling_18", 0), Result.get("peak_over_ceiling_100", 0))
    Flown = Result.get("flown_tip_ratio_100")
    Flown = "  -  " if Flown is None else f"{Flown:5.2f}"
    Waits = f"({Result['tight']}/{Result.get('arrive', 0)})"
    return (f"{Result['case'][:44]:44s} {Result['plan_s']:6.2f} {Result['stages']:3d} {Waits:5s}"
            f"{Result['joint_ratio']:6.2f}  {Tip:12s} {Flown} {Runs[0]} {Runs[1]}  {Harsh:5s} {Peak:5.2f}")


def Main():
    Parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    Parser.add_argument("--case", default="", help="only cases whose name contains this")
    Parser.add_argument("--json", help="write every number to this file")
    Parser.add_argument("--trace", help="print the per-frame stage and commands for the first matching case (at 100%%)")
    Parser.add_argument("--no-harsh", action="store_true", help="skip the harsh-model completion check")
    Options = Parser.parse_args()
    Guards = (patch.object(Bridge.depth_camera, "Opener", lambda: (_ for _ in ()).throw(RuntimeError("offline: no camera"))),
              patch.object(Bridge, "RaiseProcessPriority", lambda: "normal"))
    for Guard in Guards:
        Guard.start()
    Results = []
    print(f"planner {Collision.PlannerVersion}, speed control {Bridge.SpeedControlVersion}")
    print(f"{'case':44s} {'plan s':>6s} {'stages':>8s} {'joint x':>7s}  {'tip x (line)':12s} {'flown':>5s} {'18%':>6s} {'100%':>6s}  harsh peak/ceil")
    for Name, Start, Request in Cases:
        Match = Options.trace or Options.case
        if Match and Match not in Name:
            continue
        Result = Measure(Name, Start, Request, Harsh=not Options.no_harsh, TraceAt=100 if Options.trace else None)
        Results.append(Result)
        print(Row(Result), flush=True)
        if Options.trace:
            for Frame, Stage, Commands, _ in Result.pop("trace", []):
                Lead = max(Commands, key=lambda Joint: abs(Commands[Joint]))
                print(f"  {Frame * FrameSeconds:6.2f}s stage {Stage}  {Lead} {Commands[Lead]:6d}  "
                      + " ".join(f"{Value:5d}" for Value in Commands.values()))
            break
    Planned = [Result for Result in Results if "stages" in Result]
    if len(Planned) > 1:
        Mean = lambda Key: sum(Result[Key] for Result in Planned if Result.get(Key) is not None) / max(
            1, sum(1 for Result in Planned if Result.get(Key) is not None))
        print(f"{'mean':44s} {Mean('plan_s'):6.2f} {Mean('stages'):6.1f}   {Mean('joint_ratio'):6.2f}  "
              f"{Mean('tip_ratio'):.2f}         {Mean('flown_tip_ratio_100'):5.2f} {Mean('run_s_18'):6.2f} {Mean('run_s_100'):6.2f}")
    if Options.json:
        for Result in Results:
            Result.pop("trace", None)
        Path(Options.json).write_text(json.dumps(Results, indent=1, default=str))


if __name__ == "__main__":
    sys.exit(Main())
