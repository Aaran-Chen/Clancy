"""FastAPI bridge for the SO-101 web control app.

This module is intentionally backend-only. It reads calibration files from this
directory and does not import the old desktop GUI package.
"""

from __future__ import annotations

import asyncio
import base64
import collections
import copy
import faulthandler
import hashlib
import itertools
import queue
import uuid
import json
import logging
import math
import os
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

import cv2
import numpy as np
import serial.tools.list_ports
from contextlib import asynccontextmanager

from fastapi import FastAPI, Request, Response, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware

import calibration
import CameraDrift
import RobotReference
import RemoteAccess
import SceneSafety
import DeclaredSurfaces
import SelfCalibration
import SelfCalClicks
import object_locator
import DepthGeometry
import OrbbecCamera
import SessionLog
import CadKinematics as Cad
import BasketCollision as Collision
import HandGuide
import HandGuideCalibration
from GuideBus import GuideBus, DecodeSigned as GuideBusDecode
# Right after BasketCollision: its fingerprint must be of the collision code this process loaded.
import RouteCheckPool as RouteChecks

from arm_geometry import (
    DEFAULT_ARM_GEOMETRY,
    IK_JOINTS,
    arm_ik_dimensions,
    forward_joint_positions,
    forward_tip_position,
    load_arm_geometry,
    solve_tip_ik,
    wrist_tip_pitch_degrees,
)
from feetech_simple import (
    CONTROL_TABLE,
    MODEL_RESOLUTION,
    FeetechMotorsBus,
    JOINT_GOAL_SPEED_LIMITS,
    Motor,
    MotorNormMode,
    NORMAL_ACCELERATION,
    NORMAL_GOAL_SPEED,
    OperatingMode,
    PortLostError,
)


BACKEND_ROOT = Path(__file__).resolve().parent
DEFAULT_HOME = BACKEND_ROOT / "home_preset.json"
DEFAULT_LIMITS = BACKEND_ROOT / "gui_limits.json"
NOTEPAD_PATH = BACKEND_ROOT / "qwen_notepad.md"


class RoutinePollFilter(logging.Filter):
    """Keeps uvicorn's access log to what happened (2026-10-07 audit 2 M4: 337 MB of api-output.log in 26 h): the
    camera frame and status polls of the live view and the session recorder, some twenty a second, and /health are
    left out. Every other request is logged as before; the launchers also cap the log (LogRotation.py)."""
    Paths = frozenset({"/depthcam/frame.jpg", "/depthcam/overlay.png", "/depthcam/status", "/depthcam/depth-at", "/health"})

    def filter(Self, record: logging.LogRecord) -> bool:  # logging's own name and argument
        # uvicorn's access record: (client, method, path with query, HTTP version, status).
        Args = record.args if isinstance(record.args, tuple) else ()
        return not (len(Args) >= 3 and Args[1] == "GET" and str(Args[2]).split("?", 1)[0] in Self.Paths)


logging.getLogger("uvicorn.access").addFilter(RoutinePollFilter())

MOTOR_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)
# Home brings every joint to its saved value (home_preset.json), the wrist roll and the gripper too: the arm and the
# roll on one certified route, the gripper opening on the way or closing once the arm has arrived, and left alone while
# it holds an object (HomeGripperPlan, PlanHomeRoute). Until 2026-10-12 Home left the roll where it was and moved the
# gripper with the arm, a close included, so a jaw stopped by something tripped the stall protection mid-route.
HOME_MOTOR_NAMES = MOTOR_NAMES
GRIPPER_NAME = "gripper"
HOME_GRIPPER_HELD_NOTE = "Home: gripper left holding the object"
RESOLUTION = MODEL_RESOLUTION["sts3215"]

BRIDGE_FRAME_S = 1.0 / 30.0

# Servo protection monitoring. The STS3215 cuts its own torque ("unloads") when
# a condition in its Unload register fires; on this arm that register is 44 =
# overheat | overcurrent | overload. When that happens mid-move the arm drops,
# and the velocity stream would drive it straight back into the same overload
# once the servo recovers. So faults are polled a few times a second and any
# unload condition stops every motion stream immediately.
FAULT_CHECK_EVERY_FRAMES = 8  # about 4 Hz at the 30 Hz control rate
FAULT_UNLOAD_BITS = {2: "overheat", 3: "overcurrent", 5: "overload"}
FAULT_WARN_BITS = {0: "voltage", 1: "encoder"}
FAULT_HISTORY = 20
IK_SEND_MAX_ERROR_CM = 0.1
IK_SEND_MAX_PITCH_ERROR_DEG = 0.25
# A refusal names a wrist pitch that does reach the point; it stands back this far
# from the edge of what reaches, where rounding and servo noise flip the verdict.
IK_PITCH_HINT_MARGIN_DEG = 0.5
IK_PITCH_HINT_SECONDS = 0.6
SLIDER_GOAL_TOLERANCE_RAW = 12
# Tighter than the other joints so the gripper still closes convincingly, but
# not zero: the velocity controller's floor speed is 24 raw steps, so an exact
# match is unreachable and the gripper hunted around its target forever instead
# of parking. Holding torque toward the goal is applied either way, so grip
# strength is unaffected. 8 steps is 0.7% of the gripper's travel.
GRIPPER_GOAL_TOLERANCE_RAW = 8.0
SLIDER_VELOCITY_MIN_SPEED = 24
SLIDER_VELOCITY_MAX_SPEED = 160
SLIDER_VELOCITY_SLOW_ZONE_RAW = 320
SLIDER_VELOCITY_SPEED_DEADBAND = 10
# A route waypoint before the end is passed, without stopping, once every joint
# is this close to it or beyond it: within the waypoint reach the next leg's
# checked region extends behind it.
ROUTE_FLY_BY_TOLERANCE_RAW = 24
# Distance over which a route's final approach ramps down to a stop. Shorter
# than a slider drag's: the route's own braking model already limits the
# approach to a waypoint, so this only shapes the last settle.
ROUTE_SLOW_ZONE_RAW = 180
# Between route waypoints a joint more than this far ahead of the slowest
# joint's progress is eased off, stopping entirely at the waypoint reach.
ROUTE_LEASH_SLACK_RAW = 8
ROUTE_LEASH_MINIMUM_SPAN_RAW = 16
# Braking model for approaching a waypoint the arm may not be allowed past yet:
# at most this long from feedback to a changed command taking effect, at least
# this much braking under load, and at most this much speed over the command.
# Sized against servo models with 4-5 frames of latency, 1000-1200 steps/s^2
# braking and a loaded shoulder that sticks and then lurches.
ROUTE_BRAKE_LATENCY_S = 0.18
ROUTE_BRAKE_ACCELERATION = 1000.0
ROUTE_BRAKE_OVERSHOOT = 1.2
ROUTE_BRAKE_GUARD_RAW = 4
# A loaded joint breaking out of stiction can briefly run at about twice its
# command (logged on the hardware: the elbow at roughly 300 steps/s under a
# 150 steps/s cap, overshooting a tight waypoint by 13 of its 16 steps). Tight
# waypoints have no reach to absorb that, so their approach assumes it.
ROUTE_TIGHT_BRAKE_OVERSHOOT = 2.0
# Subdivision budget for trying the waypoint reach on an already certified
# stage. Most stages that look blocked at a cheap budget are only under-split:
# at 64 nearly every stage of a normal route certifies its reach and is flown
# past instead of stopped at. A stage that still misses stays tight, so the
# deeper search only costs time, and ROUTE_REACH_ASSIGN_SECONDS caps that: once
# a route has spent it, the rest of the waypoints fall back to the cheap budget.
ROUTE_REACH_CHECK_BUDGET = 64
ROUTE_REACH_CHEAP_BUDGET = 8
ROUTE_REACH_ASSIGN_SECONDS = 1.2
# Where the plain waypoint reach does not certify, this smaller one is tried before the waypoint
# becomes tight. The arm still flies past (a joint trailing within the fly-by tolerance, 24, is
# inside the next stage's start: this plus the allowance), but approaches it as a tight waypoint,
# allowing a lurch to twice the command (ROUTE_TIGHT_BRAKE_OVERSHOOT).
ROUTE_REDUCED_REACH_RAW = 16
# The wider reach a joint carrying on through a waypoint may get (WideWaypointReach): this share of its
# travel on the stage before the waypoint. It costs planning time: a check that passes about 0.03 s more
# than the plain reach's, one that fails 0.1-0.2 s. It buys most where braking, not the ceiling, sets
# the pace: at 100% it took 0.4-0.9 s off 3.6-3.8 s routes from Home, at 18% 0.2 s, less than it cost.
# So it is tried only at this throttle and above, at this subdivision budget, and a route stops trying
# it after this many misses. Half the travel beat the whole of it, plan and run together (MeasureRouteSpeed).
ROUTE_WIDE_REACH_FRACTION = 0.5
ROUTE_WIDE_REACH_MINIMUM_PERCENT = 50
ROUTE_WIDE_REACH_BUDGET = 64
ROUTE_WIDE_REACH_MISSES = 2
# Wall clock for the last planning attempt, and the share of it the preferred
# arm configuration gets. Splitting it evenly gave every configuration too
# little: routes that need a detour are found in the first seconds or not at all.
ROUTE_PLAN_SECONDS = 9.0
ROUTE_PLAN_FIRST_SHARE = 0.6
HOLD_PAUSED_STATUS = "Hold correction paused near an obstacle; the servos are still holding"
# A capture's arm pose is trusted only while every joint stayed within this many raw
# steps (0.7 deg) of it since the frame: about 0.4 cm at the tip tape.
CAPTURE_STILL_RAW = 8
# ...and only while the joint record (30 Hz) has no hole this long from the capture to now,
# where a move by hand could hide.
CAPTURE_TRACE_GAP_S = 0.3

# Homing drives every joint at once, so it runs slower than a single slider
# drag. Mirrors the old desktop GUI, which switched to a reduced motion profile
# for the duration of a home move.
HOME_VELOCITY_MAX_SPEED = 70
# 100% is a top route ceiling of 70 x 25 = 1750 encoder steps/s; joints with a lower
# ceiling of their own (JOINT_GOAL_SPEED_LIMITS) stop there. See Cad/MotionSpeed.md.
MotionSpeedMaximumMultiplier = 25
# 18% of 1750 is 315 steps/s: the same startup speed as the old 30% of 1050.
DefaultMotionSpeedPercent = 18
# The app enables its fader only for exactly this value: it names the throttle contract, not how routes fly.
SpeedControlVersion = "throttle-v2"
# How a route is flown (RouteSpeeds): joints timed together at their braking bounds and carried through
# waypoints. Reported in /health and the state so a bridge still running older rules can be told apart.
# v2: the route end accepts what a joint carrying on into the last stage may already have overshot (EndReach).
RouteExecutorVersion = "synchronized-v2"
# Closed-loop hold only auto-corrects modest drift on its own. Beyond ~45 deg
# something is very wrong (clamp snap, slipped coupling, stale target), so
# latch a fault for a human instead of driving full span unattended.
HOLD_REARM_MAX_ERROR_RAW = 512
# v2-confirmed (2026-10-12): a protection reading acts only once it persists. One missed temperature reply, or one status
# reply reading torque off, a fault bit or telemetry no servo can report, coasted a healthy arm (16:17:25, 16:32:43);
# each is now read again first, and the tolerated ones go to bridge_events.jsonl as protection_glitch (MotorProtection.md).
ProtectionVersion = "auto-coast-v2-confirmed"
# Conservative software limits, additional to the servo's firmware protection.
ProtectionTemperatureC = 60
ProtectionRearmTemperatureC = 55
# 3.0 s since 2026-10-12 (was 2.0). The servos' own overload (over 80 % output while not moving for Protection_Time,
# then 20 % output) and overcurrent (2.0 A for 2 s, torque off) act on a hard stall about 2 s in either way, and the
# 60 C limit is polled throughout; see MotorProtection.md, Confirmed readings.
ProtectionStallSeconds = 3.0
# Present_Temperature polls of one joint that may fail in a row before protection trips, while its position still reads:
# 2 at the 0.25 s poll is at most about 0.5 s without a temperature, which takes minutes to change. A joint whose
# position fails too trips at once, as before.
ProtectionFeedbackMisses = 2
# A status reply whose telemetry no servo can report is a corrupted reply, read as a failed one (ServoTelemetryImpossible):
# Present_Load beyond its 0-1000 magnitude, a supply under 6.0 V with 85 C or more, or over 14.0 V (the STS3215's rating).
ServoLoadMaximum = 1000
ServoVoltageRangeDv = (60, 140)
ServoHotC = 85
# A control frame that starts this long after the last one ended trips protection while anything moves:
# a joint in velocity mode keeps its last speed meanwhile, unsupervised (the servos have no host timeout).
# An arm holding still (every joint in its servo's own position hold) is read again instead, and goes on
# holding when every joint agrees (ResumeAfterControlDelay): 2026-10-04 23:11:14 a 1.56 s delay with
# nothing moving coasted the arm, extended over the desk, and it dropped.
ControlHeartbeatLimitS = 0.75
# How long the plain-words note about such a delay stays as the bridge status.
ControlDelayNoteSeconds = 20.0
# Reported in /health and the state, so a bridge still coasting a still arm on a late frame can be told apart.
HeartbeatVersion = "held-still-v1"
# Control-loop stalls (2026-10-07 audit L3/S1: 1.1-1.7 s stalls recurred with nothing moving, cause unknown).
# A frame that starts this long after the last one ended, or that itself runs this long, is recorded as
# control_stall in bridge_events.jsonl with what it waited for: the motor lock (and who held it, how long),
# the frame's own steps, and the CPU or interpreter otherwise. Diagnostics only: no command changes.
ControlStallLogS = 0.3
# A motor-lock hold longer than this outside the control thread is remembered (InstrumentedLock.Holds).
ControlLockHoldNoteS = 0.05
# With no frame finished for this long, the running server dumps every thread's stack (faulthandler, which
# works even while another thread holds the interpreter lock) to captures/<date>/recording/control_stalls.log.
ControlStallDumpS = 0.5
# hold-v3: Hold holds every joint where it is, also outside its slider limits; a limit edit never moves a joint;
# a route stopped by a changed scene or configuration holds instead of coasting (2026-10-07 audit H1-H3, M8).
# hold-v4: a joint held in the middle of its excluded window reads its jitter there as jitter (not a turn); a stop that
# holds where it is keeps a grasp's squeeze; a POST /motion command that arrived before a Stop is cancelled by it
# (2026-10-08 safety review).
SafetyVersion = "hold-v4"
# Scene-bound motion: a measured scene that changes or expires mid-route stops the route holding; Home and manual
# moves are never scene-bound (2026-10-07 audit H1, M1, M10).
SceneVersion = "scene-hold-v1"
# The camera checked against the gripper tip with no calibrated desk in view (RobotReference.py); an "assumed"
# camera keeps the [UNRELIABLE] tag and the scene tripwire (Aaran's decision, 2026-10-07 audit IH1).
RobotReferenceVersion = "tip-check-assumed-unverified-v1"
# Who may use the bridge (RemoteAccess.py): this PC as before, other devices with the remote password, other sites'
# pages never; a connect from another device, or after a deliberate disconnect, only on a Connect click (2026-10-07
# audit 2 H2, M2; Aaran's decision 2).
AccessVersion = "remote-password-v1"
# Hand guiding (docs/HandGuide.md): the shoulder, elbow and wrist carry their own weight and every joint gets a little
# help in the direction the hand moves it, from a 100 Hz guide loop of its own (HandGuide.py, GuideBus.py). Any fault
# while guiding makes the whole arm go limp (Aaran's decision D1, 2026-10-10); Hold here holds it where it is.
HandGuideVersion = "guide-v1"
HandGuidePeriodS = 0.01
HandGuideStatusEvery = 4          # the torque and Lock read-back (regs 40..55), every 4th cycle
HandGuideOwnerGoneS = 3.0         # the page that started it silent this long (closed, or a phone asleep): Hold here
HandGuideHeartbeatS = 0.15        # the supervisor: the guide loop's last cycle older than this, or the loop gone: limp
HandGuideStillSteps = 2           # entry: every joint within this many steps...
HandGuideStillS = 0.5             # ...over this long (the motion trace)
HandGuideGuardPadDeg = 3.0        # the collision guard band: the measured pose, every joint padded by this
HandGuideNoStallS = 10.0          # entry refused within this long of a control stall or delay
HandGuideProfileRestoreS = 0.1    # a holding exit writes Goal_Speed 0 (the preload lands at once); the profile after this
HandGuideCalibrationFile = "hand_guide_calibration.json"
# The bridge's code contract with the page as a whole (BridgeVersion.json, which the page and RestartBridge.ps1 read
# too): 2026-10-06 the bridge ran 8f4d27a for a day under HEAD's page, and every older version string matched.
BridgeVersion = str(json.loads((Path(__file__).resolve().parent / "BridgeVersion.json").read_text(encoding="utf-8"))["Version"])
# A gripper closing on an object stalls there by design: that is the grasp. A
# commanded close that moves nothing else becomes a grasp hold after this long
# without progress, instead of a stall trip that coasts the whole arm: the jaws
# hold in position mode GRIPPER_GRASP_SQUEEZE_RAW steps past where the object
# stopped them, a bounded squeeze rather than a velocity stream at full torque.
# The squeeze stays inside the tracking allowance every route certifies
# (JointTrackingAllowanceRaw, 16), so an object slipping out mid-route never
# takes the jaws off the checked route. Fraction 0 is closed (the app's
# target_fraction 0.0).
GRIPPER_GRASP_STALL_SECONDS = 0.5
GRIPPER_GRASP_SQUEEZE_RAW = 8
# A tip move (send_ik) may carry a gripper target that opens the jaws: the app's
# chained "gripper_before" open. One route then opens the gripper while the arm
# moves (BlendGripperIntoRoute) instead of a gripper move, a settle, and a second
# route that the still-settling gripper could refuse (2026-10-03 14:46: 3 steps of
# gripper creep during planning refused the tip move). The jaws reach the target
# this far through the arm's route time, later only when the gripper's own speed
# ceiling needs longer, at the latest on arrival. The app reads the version.
GRIPPER_WITH_TIP_OPEN_BY = 0.5
GripperWithTipVersion = "open-while-moving-v1"
# USB serial link supervision (ServiceSerialLink; MotorProtection.md "Serial link drops").
# 2026-10-01 19:07 the arm's USB adapter dropped off the bus for 23 s; the bridge never
# reopened the port and every bus call failed for 15 minutes, emergency torque-off included.
SerialLinkVersion = "reopen-v1"
# A drop shorter than this while nothing moves ends with the arm holding where it was, once
# fresh reads of every joint pass. A longer one latches a fault and the arm coasts as soon as
# the link returns. The servos keep their own position loop and firmware protection meanwhile.
SerialLinkHoldWindowS = 2.0
SerialLinkReopenIntervalS = 0.1  # reopen attempts inside the hold window
SerialLinkRetryIntervalS = 0.5   # and after it, until the port comes back
# This many drops within the window trip protection even if each one recovered: a flapping link.
SerialLinkDropLimit = 3
SerialLinkDropWindowS = 60.0
# A holding joint found farther than this from where it was before the drop did not hold still.
SerialLinkHoldDriftRaw = 30
# How long the plain-words note about a drop stays as the bridge status.
SerialLinkNoteSeconds = 20.0
# A scene-bound route starts only while its measured scene stays usable this long: twice the route's estimated
# time (RouteDuration) plus this margin. Otherwise it would stop holding part way (2026-10-07 audit H1).
SceneDispatchMarginS = 10.0
BRIDGE_EVENTS_LOG = "bridge_events.jsonl"
CONTROL_STALLS_LOG = "control_stalls.log"
# Every completed route's plan time, flying time and control frames (RecordRouteTiming), beside it.
ROUTE_TIMING_LOG = "route_timing.jsonl"
TASK_OBJECT_PHRASE_MAX_CHARS = 80


def SourceHash(File: Path) -> str:
    return hashlib.sha256(File.read_bytes()).hexdigest()[:12]


# The IK solver code this process loaded. ik_solver_version is the app's
# compatibility contract and changes only with the solver's behaviour; this names
# the exact source, so /health shows a bridge still running an older
# CadKinematics.py (2026-09-29: the bridge ran HEAD's solver for hours after the
# rewrite, and nothing could tell).
IkSolverSource = SourceHash(Path(Cad.__file__))


def IkSolverSourceStale() -> bool:
    try:
        return SourceHash(Path(Cad.__file__)) != IkSolverSource
    except OSError:
        return True


def RouteCheckAnswered(Future, Deadline=None, Certified=True) -> bool:
    """Whether a check worker's answer (RouteCheckPool.Pool.Submit) came by Deadline (default the request's own)
    and certified the stage (Certified) or refused it (not Certified). Never true without an answer."""
    if Future is None:
        return False
    try:
        Answer = Future.result(timeout=None if Deadline is None else max(0.0, Deadline - time.monotonic()))
    except Exception:
        Future.cancel()
        return False
    return Answer is not None and (Answer[1] is None) == Certified


def RouteReachKey(Reach) -> tuple:
    """A waypoint reach (joint -> encoder steps; empty for none) as a key: the same reach, the same key."""
    return tuple(sorted((Name, int(Steps)) for Name, Steps in (Reach or {}).items() if Steps))


def RouteNote(Route) -> str:
    """A route's note (StartMotionRoute Note) as the end of its status, or nothing."""
    return f"; {Route['Note']}" if Route.get("Note") else ""


# Motion faults, protection trips and motion aborts, one JSON line each in
# captures/<date>/recording/bridge_events.jsonl beside the app's ai.jsonl. The
# state message only carries the latest, and a page reload loses its log: the
# 2026-09-29 07:57 fault ended a task and its reason was gone. The control loop
# only queues a record; a writer thread does the disk work, so a slow disk never
# delays a frame, and recording never raises.
_BridgeEvents: queue.Queue = queue.Queue()
_BridgeEventWriter: threading.Thread | None = None
_BridgeEventWriterLock = threading.Lock()


def AppendBridgeEvent(Record: dict[str, Any]) -> None:
    global _BridgeEventWriter
    try:
        with _BridgeEventWriterLock:
            if _BridgeEventWriter is None or not _BridgeEventWriter.is_alive():
                _BridgeEventWriter = threading.Thread(target=_WriteBridgeEvents, name="so101-bridge-events", daemon=True)
                _BridgeEventWriter.start()
        _BridgeEvents.put(Record)
    except Exception as Exc:
        print(f"[bridge-events] could not queue {Record.get('kind')}: {Exc}", file=sys.stderr)


def _WriteBridgeEvents() -> None:
    while True:
        Record = _BridgeEvents.get()
        try:
            Directory = SessionLog.RecordingDirectory(Record.get("t"))
            Directory.mkdir(parents=True, exist_ok=True)
            # A record may name its own log (RecordRouteTiming); faults, trips and aborts go to the events log.
            with open(Directory / Record.pop("log", BRIDGE_EVENTS_LOG), "a", encoding="utf-8") as File:
                File.write(json.dumps(Record, default=str) + "\n")
        except Exception as Exc:
            print(f"[bridge-events] could not record {Record.get('kind')}: {Exc}", file=sys.stderr)
        finally:
            _BridgeEvents.task_done()


def FlushBridgeEvents(Seconds: float = 2.0) -> bool:
    """Wait until every queued record is written (or failed); False on timeout."""
    Deadline = time.monotonic() + Seconds
    while _BridgeEvents.unfinished_tasks and time.monotonic() < Deadline:
        time.sleep(0.01)
    return not _BridgeEvents.unfinished_tasks


def _make_bus(port: str) -> FeetechMotorsBus:
    deg = MotorNormMode.DEGREES
    pct = MotorNormMode.RANGE_0_100
    return FeetechMotorsBus(
        port=port,
        motors={
            "shoulder_pan": Motor(1, "sts3215", deg),
            "shoulder_lift": Motor(2, "sts3215", deg),
            "elbow_flex": Motor(3, "sts3215", deg),
            "wrist_flex": Motor(4, "sts3215", deg),
            "wrist_roll": Motor(5, "sts3215", deg),
            "gripper": Motor(6, "sts3215", pct),
        },
    )


def _load_json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return data if isinstance(data, dict) else {}


def _normalize_deg(angle: float) -> float:
    return (float(angle) + 180.0) % 360.0 - 180.0


def _as_int(value: Any, default: int) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


@dataclass
class ServoRuntime:
    name: str
    present_raw: int
    target_raw: int
    range_min: int
    range_max: int
    arc_inverted: bool = False
    gui_angle_raw: int | None = None
    command_raw: int | None = None
    command_speed: int = 0
    AppliedSpeedPercent: int = 100
    last_write_s: float = 0.0
    homing: bool = False
    FeedbackOk: bool = False
    HardwareRaw: int | None = None
    Holding: bool = False
    # Squeezing an object it closed on (HoldGrasp); any new goal for this servo ends it.
    GraspHold: bool = False


@dataclass
class SerialLinkState:
    """The USB serial link to the servo bus: the current drop, if any, and counts (ServiceSerialLink)."""
    Down: bool = False
    LostAt: float = 0.0  # time.monotonic() of the drop
    Error: str = ""
    MovingAtLoss: bool = False
    PositionsAtLoss: dict[str, int] = field(default_factory=dict)
    Attempts: int = 0  # reopen attempts in this drop
    LastAttempt: float = 0.0
    WindowExpired: bool = False
    Drops: collections.deque = field(default_factory=lambda: collections.deque(maxlen=50))
    DropsTotal: int = 0
    Recoveries: int = 0
    Failures: int = 0  # drops that outlasted the hold window
    LastOutageS: float | None = None
    Note: str | None = None  # what happened, in plain words (LiveStatus)
    NoteAt: float = 0.0


def _NormalizePort(Port: Any) -> str:
    """Compare serial ports by substance, not spelling.

    A page refresh re-sends connect on every boot; tearing down a live,
    holding arm because of COM4 vs com4 vs an extended path spelling would
    coast the arm for nothing. Only a genuinely different port may tear down.
    """
    Text = str(Port or "").strip().upper()
    for Prefix in ("\\\\.\\", "\\??\\"):
        if Text.startswith(Prefix):
            Text = Text[len(Prefix):]
    return Text.rstrip("\\")


class InstrumentedLock:
    """The motor lock (a re-entrant lock), keeping who holds it and since when, and the long holds of late.

    For the control-stall record (BridgeController.NoteControlStall): a control frame that waited for the
    lock can say which thread held it and for how long. A hold longer than ControlLockHoldNoteS is kept in
    Holds (thread name, start and end on time.monotonic()). Costs a few microseconds per acquire.
    """

    def __init__(Self) -> None:
        Self._Lock = threading.RLock()
        Self._Depth = 0
        Self.Owner: str | None = None
        Self.Since = 0.0
        Self.Holds: collections.deque = collections.deque(maxlen=50)

    def acquire(Self, blocking: bool = True, timeout: float = -1) -> bool:
        if not Self._Lock.acquire(blocking, timeout):
            return False
        Self._Depth += 1
        if Self._Depth == 1:
            Self.Owner, Self.Since = threading.current_thread().name, time.monotonic()
        return True

    def release(Self) -> None:
        Self._Depth -= 1
        if Self._Depth == 0:
            Now = time.monotonic()
            if Now - Self.Since > ControlLockHoldNoteS:
                Self.Holds.append((Self.Owner, Self.Since, Now))
            Self.Owner = None
        Self._Lock.release()

    __enter__ = acquire

    def __exit__(Self, *Exc) -> None:
        Self.release()

    def HoldsSince(Self, Start: float) -> list[dict[str, Any]]:
        """The long holds (and the current one) overlapping [Start, now], newest last, in plain fields."""
        Now = time.monotonic()
        Found = [(Name, Begin, End) for Name, Begin, End in list(Self.Holds) if End >= Start]
        if Self.Owner is not None:
            Found.append((Self.Owner, Self.Since, None))
        return [{"thread": Name, "held_ms": round(((End or Now) - Begin) * 1000), "ended_ms_ago": None if End is None
                 else round((Now - End) * 1000)} for Name, Begin, End in Found]


@dataclass
class HandGuideSession:
    """One hand-guiding session, or one hand-guiding calibration, while it runs (BridgeController.HandGuide)."""
    Mode: str                       # "guide", or "calibrate" (HandGuideCalibration's sweeps)
    Law: Any                        # HandGuide.HandGuideLaw, or HandGuideCalibration.CalibrationRun
    Bus: Any                        # GuideBus on the bridge's own bus
    Joints: tuple                   # the joints it drives (the gripper is kept out while it squeezes an object)
    Owner: str | None               # the page connection that started it: its heartbeats keep it going
    Access: str                     # "local" (this PC) or "password" (another device)
    Since: float                    # BridgeController.HandGuideNow() (perf_counter)
    HoldGoals: dict                 # every Goal_Position register at entry (the entry hold)
    World: Any = None               # the guard band's collision world (basket, body, camera head, rearward limit)
    Fingerprint: dict = field(default_factory=dict)
    ReferenceRaw: dict = field(default_factory=dict)
    Cycle: int = 0
    LastCycle: float = 0.0
    Misses: int = 0
    WallVerdict: tuple = (None, 0.0)
    Readings: dict = field(default_factory=dict)
    Phase: str = ""
    Notes: list = field(default_factory=list)   # what the page shows: a joint's friction help off, a warm servo
    HotSince: dict = field(default_factory=dict)  # calibration: when each joint first read at or over temp_exit_c
    Suspect: dict = field(default_factory=dict)   # calibration: (check, joint) -> bad readings in a row (persist_reads)
    StatusRecheck: bool = False                   # a torque or Lock read-back is suspect: read regs 40..55 next cycle
    Stop: threading.Event = field(default_factory=threading.Event)
    Thread: threading.Thread | None = None


class RouteHeld(Exception):
    """The route was stopped and the arm holds where it is, torque on: no fault (StopMotion, not latched)."""


class CorruptStatusReply(ValueError):
    """A servo status reading whose telemetry no servo can report (ServoTelemetryImpossible): read as a failed read."""

    def __init__(self, Text: str, Bits: int, Telemetry: dict[str, Any]) -> None:
        super().__init__(Text)
        self.Bits, self.Telemetry = Bits, Telemetry


class BridgeController:
    def __init__(
        self,
        home_path: Path = DEFAULT_HOME,
        limits_path: Path = DEFAULT_LIMITS,
        geometry_path: Path = DEFAULT_ARM_GEOMETRY,
    ) -> None:
        self.home_path = home_path
        self.limits_path = limits_path
        self.geometry_path = geometry_path
        self.lock = InstrumentedLock()
        # The measured scene the running route was certified against (BindScene/AdoptScene), only while that
        # route runs: Home, manual moves and hold corrections never use it (2026-10-07 audit M1, M10).
        self.ActiveScene: dict[str, Any] | None = None
        # A held object's envelope in gripper coordinates, kept for the next scene-bound move of a carry.
        self.SceneCarried: dict[str, Any] | None = None
        # Control-stall diagnostics (NoteControlStall): the steps of the frame running now (ms) and stalls seen.
        self.FramePhases: dict[str, float] = {}
        self.ControlStalls: collections.deque[dict[str, Any]] = collections.deque(maxlen=20)
        self._last_preview: dict[str, Any] | None = None
        self.faults: dict[str, dict[str, Any]] = {}
        self.fault_events: list[dict[str, Any]] = []
        self._refresh_count = 0
        self.last_refresh_s = 0.0
        # Each joint's position when the last control frame ended with the arm holding still, else None.
        self.HeldStillAtFrameEnd: dict[str, int] | None = None
        # Delayed control frames the arm held still through (ResumeAfterControlDelay), for review.
        self.ControlDelays: collections.deque[dict[str, Any]] = collections.deque(maxlen=20)
        self.ControlDelayNote: str | None = None
        self.ControlDelayNoteAt = 0.0
        self.bus: FeetechMotorsBus | None = None
        self.port = ""
        self.torque_enabled = False
        self.status = "Bridge ready"
        self.MotionFault: str | None = None
        self.ProtectionTrip: dict[str, Any] | None = None
        self.ProtectionTelemetry: dict[str, dict[str, Any]] = {}
        # Protection readings tolerated until confirmed (NoteProtectionGlitch): an over-limit temperature or a status reply
        # that an immediate re-read contradicted, a missed temperature reply (kept for review).
        self.ProtectionGlitches: collections.deque[dict[str, Any]] = collections.deque(maxlen=20)
        # Per joint: Present_Temperature polls failed in a row (ProtectionFeedbackMisses).
        self.TemperatureMisses: dict[str, int] = {}
        # Per joint: (error at the last progress, when, the target it was measured against).
        self.ProtectionProgress: dict[str, tuple[float, float, int]] = {}
        self.LastProtectionPoll = 0.0
        self.LastCoastAttempt = 0.0
        # Where RecordBridgeEvent sends each fault, trip and abort. Only the running
        # server points it at bridge_events.jsonl (_lifespan): offline imports and
        # tests trip faults on fake motors and must not write the session record.
        self.EventSink: Callable[[dict[str, Any]], None] | None = None
        # Where completed routes' timing records go (RecordRouteTiming); like EventSink, only the running server sets it.
        self.TimingSink: Callable[[dict[str, Any]], None] | None = None
        # Set by Disconnect or a page's Emergency Shutoff ({"at", "by"}), cleared only by an explicit Connect: until
        # then no page connects the arm by itself (ConnectRefusal; 2026-10-07 audit 2 M2). The running server keeps it
        # on disk (IntentSink, RemoteAccess.SaveIntent), so a restarted bridge still knows. Never under the motor lock:
        # IntentLock only keeps two disk writes apart.
        self.DisconnectIntent: dict[str, Any] | None = None
        self.IntentSink: Callable[[dict[str, Any] | None], None] | None = None
        self.IntentLock = threading.Lock()
        self.LastBridgeEvent: dict[str, Any] | None = None
        self.MotionEpoch = 0
        self.CalibrationPlans: dict[str, dict[str, Any]] = {}
        self.CalibrationActive = False
        self.CalibrationPrepareToken: str | None = None
        self.MotionDeadline = 0.0
        self.MotionTimeoutMs = 12000
        self.MotionStartedAt = 0.0
        self.MotionSpeedPercent = DefaultMotionSpeedPercent
        self.MotionRoute = None
        # Diagnostics: the last 40 seconds of control frames (RecordMotionTrace).
        self.MotionTrace: collections.deque = collections.deque(maxlen=1200)
        self.MotionTraceStart = time.monotonic()
        self.HoldReturnCache = None
        self.HoldCorrectionPaused = False
        self.SerialLink = SerialLinkState()
        # Hand guiding (HandGuideStart): the session while it runs, how the last one ended, the pages' heartbeats
        # (connection id -> HandGuideNow), and the calibration file. HandGuideThreaded: the guide loop runs on its own
        # thread (tests step HandGuideCycle themselves on a simulated clock).
        self.HandGuide: HandGuideSession | None = None
        self.HandGuideLast: dict[str, Any] | None = None
        self.HandGuideOwners: dict[str, float] = {}
        self.HandGuideThreaded = True
        self.HandGuideCalibrationPath = BACKEND_ROOT / HandGuideCalibrationFile
        self.HandGuideCalibrationCache: tuple[float, dict | None] | None = None
        self.HandGuideCalibrationResult: dict[str, Any] | None = None
        self.HandGuideRestoreSpeedAt = 0.0
        # A session wrote Acceleration 0 and its coast left it so: the next Hold writes the normal profile first.
        self.HandGuideAccelerationDirty = False

        self.home_preset = self._load_home()
        self.gui_limits = _load_json(self.limits_path)
        self.arm_geometry = load_arm_geometry(self.geometry_path)
        self.arm_dims = arm_ik_dimensions(self.arm_geometry)
        ik_settings = self.arm_geometry.get("ik", {})
        self.ik_settings = ik_settings if isinstance(ik_settings, dict) else {}
        self.elbow_mode = str(self.ik_settings.get("default_elbow", "down"))
        if self.elbow_mode not in ("up", "down"):
            self.elbow_mode = "down"

        self.servos = self._build_initial_servos()
        current_angles = self._current_ik_angles()
        self.elbow_mode = self._current_elbow_branch(current_angles)
        zero_angles = current_angles
        self.target_cm = forward_tip_position(self.arm_geometry, zero_angles)
        self.target_wrist_pitch_deg = wrist_tip_pitch_degrees(
            self.arm_geometry,
            zero_angles,
        )
        self._target_position_changed = False

    @property
    def arm_connected(self) -> bool:
        bus = self.bus
        return bool(bus is not None and bus.is_connected)

    def _load_home(self) -> dict[str, int]:
        data = _load_json(self.home_path)
        return {name: _as_int(data.get(name), RESOLUTION // 2) % RESOLUTION for name in MOTOR_NAMES}

    def _saved_limit(self, name: str) -> tuple[int, int, bool]:
        limits = self.gui_limits.get("motor_raw_limits", {})
        saved = limits.get(name) if isinstance(limits, dict) else None
        if not isinstance(saved, dict):
            return 0, RESOLUTION - 1, False
        lo = max(0, min(RESOLUTION - 1, _as_int(saved.get("min"), 0)))
        hi = max(0, min(RESOLUTION - 1, _as_int(saved.get("max"), RESOLUTION - 1)))
        if lo == hi:
            return 0, RESOLUTION - 1, False
        return lo, hi, bool(saved.get("arc_inverted", False))

    def _build_initial_servos(self) -> dict[str, ServoRuntime]:
        servos: dict[str, ServoRuntime] = {}
        for name in MOTOR_NAMES:
            lo, hi, inverted = self._saved_limit(name)
            raw = self.home_preset.get(name, RESOLUTION // 2) % RESOLUTION
            servos[name] = ServoRuntime(
                name=name,
                present_raw=raw,
                target_raw=raw,
                range_min=lo,
                range_max=hi,
                arc_inverted=inverted,
            )
        for servo in servos.values():
            servo.present_raw = self._clamp_raw(servo, servo.present_raw)
            servo.target_raw = self._clamp_raw(servo, servo.target_raw)
        return servos

    def _reload_persistent_data(self) -> None:
        self.home_preset = self._load_home()
        self.gui_limits = _load_json(self.limits_path)
        for name, servo in self.servos.items():
            lo, hi, inverted = self._saved_limit(name)
            servo.range_min = lo
            servo.range_max = hi
            servo.arc_inverted = inverted
            # A target is never pulled into new limits: a holding joint outside them would be driven there,
            # unchecked (2026-10-07 audit M8). Limits apply to the next commanded move.
            if servo.gui_angle_raw is not None:
                servo.gui_angle_raw = self._clamp_raw(servo, servo.gui_angle_raw)

    @staticmethod
    def _arc_span(servo: ServoRuntime) -> int:
        if servo.arc_inverted:
            return (servo.range_min - servo.range_max) % RESOLUTION
        return (servo.range_max - servo.range_min) % RESOLUTION

    @staticmethod
    def _arc_distance_from_left(servo: ServoRuntime, raw: int) -> int:
        r = int(raw) % RESOLUTION
        if servo.arc_inverted:
            return (servo.range_min - r) % RESOLUTION
        return (r - servo.range_min) % RESOLUTION

    @classmethod
    def _contains_raw(cls, servo: ServoRuntime, raw: int) -> bool:
        span = cls._arc_span(servo)
        return span > 0 and cls._arc_distance_from_left(servo, raw) <= span

    @staticmethod
    def _circular_raw_distance(a: int, b: int) -> int:
        fwd = (int(a) % RESOLUTION - int(b) % RESOLUTION) % RESOLUTION
        bwd = (int(b) % RESOLUTION - int(a) % RESOLUTION) % RESOLUTION
        return min(fwd, bwd)

    @classmethod
    def _clamp_raw(cls, servo: ServoRuntime, raw: int) -> int:
        r = int(raw) % RESOLUTION
        if cls._contains_raw(servo, r):
            return r
        left_dist = cls._circular_raw_distance(r, servo.range_min)
        right_dist = cls._circular_raw_distance(r, servo.range_max)
        return servo.range_min if left_dist <= right_dist else servo.range_max

    @classmethod
    def _raw_to_track_fraction(cls, servo: ServoRuntime, raw: int) -> float:
        span = cls._arc_span(servo)
        if span == 0:
            return 0.0
        if cls._contains_raw(servo, raw):
            return cls._arc_distance_from_left(servo, raw) / span
        left_dist = cls._circular_raw_distance(raw, servo.range_min)
        right_dist = cls._circular_raw_distance(raw, servo.range_max)
        return 0.0 if left_dist <= right_dist else 1.0

    @classmethod
    def _fraction_to_raw(cls, servo: ServoRuntime, fraction: float) -> int:
        frac = max(0.0, min(1.0, float(fraction)))
        span = cls._arc_span(servo)
        step = round(frac * span)
        raw = servo.range_min - step if servo.arc_inverted else servo.range_min + step
        return cls._clamp_raw(servo, raw)

    @classmethod
    def _slider_track_pos(cls, servo: ServoRuntime, raw: int) -> float:
        """Position along the slider track, in raw steps from the left end.

        Feeds the velocity controller only. Unlike the display fraction, a raw
        outside the arc reports a signed overshoot past the nearer end rather
        than pinning to 0 or span — otherwise a joint parked beyond its limit
        reads as sitting exactly on the boundary and nothing ever drives it
        back into range.
        """
        span = cls._arc_span(servo)
        if span == 0:
            return 0.0
        if cls._contains_raw(servo, raw):
            return float(cls._arc_distance_from_left(servo, raw))
        left_dist = cls._circular_raw_distance(raw, servo.range_min)
        right_dist = cls._circular_raw_distance(raw, servo.range_max)
        if left_dist <= right_dist:
            return -float(left_dist)
        return float(span + right_dist)

    def _slider_track_near(self, servo: ServoRuntime, raw: int, Near: float) -> float:
        """raw's slider track position, a raw outside the arc read on whichever side of the excluded window is nearer
        the track position Near (its two readings, past either end, are a turn apart)."""
        Track = self._slider_track_pos(servo, raw)
        if self._arc_span(servo) == 0 or self._contains_raw(servo, raw):
            return Track
        Other = Track + RESOLUTION if Track < 0 else Track - RESOLUTION
        return Other if abs(Other - Near) < abs(Track - Near) else Track

    def _slider_track_present(self, servo: ServoRuntime) -> tuple[float, float]:
        """(present, target) track positions, for comparing a joint with where it is meant to be.

        _slider_track_pos reads a raw outside the arc past the NEARER end, so across the middle of the excluded window
        two raws a step apart read about a turn apart. Hold holds a hand-posed joint where it stands, also there
        (hold-v3, audit H3): a step or two of jitter then read 4094 steps off its hold point or its route and coasted
        every joint (2026-10-08 safety review, the wrist roll's 196-step window). Outside the arc the present is read
        on the side of what it is compared with: on a route, its stage's start (the route was planned from that
        reading, which keeps a route back into range going the way it was planned); holding, its hold point, which
        is read on the present's side instead when only the hold point is outside. Inside the arc nothing changes.
        """
        Route = self.MotionRoute
        Target = self._slider_track_pos(servo, servo.target_raw)
        if Route is not None and servo.name in Route["Start"]:
            Origin = self._slider_track_pos(servo, Route["Start"][servo.name])
            return self._slider_track_near(servo, servo.present_raw, Origin), Target
        if self._contains_raw(servo, servo.present_raw):
            Present = self._slider_track_pos(servo, servo.present_raw)
            return Present, self._slider_track_near(servo, servo.target_raw, Present)
        return self._slider_track_near(servo, servo.present_raw, Target), Target

    @staticmethod
    def _slider_goal_tolerance_raw(name: str) -> float:
        return GRIPPER_GOAL_TOLERANCE_RAW if name == GRIPPER_NAME else SLIDER_GOAL_TOLERANCE_RAW

    def _slider_target_reached(self, servo: ServoRuntime) -> bool:
        return abs(self._slider_track_error(servo)) <= self._slider_goal_tolerance_raw(servo.name)

    def _slider_track_error(self, servo: ServoRuntime) -> float:
        if self._arc_span(servo) == 0:
            return 0.0
        Present, Target = self._slider_track_present(servo)
        return Target - Present

    def _slider_velocity_speed(self, servo: ServoRuntime, SpeedPercent: int = 100, MaximumMultiplier: int = 1) -> int:
        error = self._slider_track_error(servo)
        if abs(error) <= self._slider_goal_tolerance_raw(servo.name):
            return 0
        direction = 1 if error > 0 else -1
        max_speed = (HOME_VELOCITY_MAX_SPEED if servo.homing else SLIDER_VELOCITY_MAX_SPEED) * MaximumMultiplier
        # Raise the ceiling, keeping the fine-positioning floor unchanged.
        min_speed = min(SLIDER_VELOCITY_MIN_SPEED, max_speed)
        scale = min(1.0, abs(error) / SLIDER_VELOCITY_SLOW_ZONE_RAW)
        speed = round(max_speed * scale)
        speed = max(min_speed, min(max_speed, speed))
        return direction * max(1, round(speed * SpeedPercent / 100))

    def ActiveMotionSpeedPercent(self) -> int:
        # Idle holding corrections keep their existing response. The throttle
        # scales commanded routes without changing holding torque or strength.
        return self.MotionSpeedPercent if self.MotionRoute is not None else 100

    @staticmethod
    def MotionSpeedCeilings(Percent: float) -> dict[str, float]:
        """Route cruise ceiling per joint, in encoder steps/s, at Percent of full speed.

        The throttle takes its share of the top ceiling, and a joint with a lower
        100% ceiling of its own (JOINT_GOAL_SPEED_LIMITS) never goes past it, so no
        joint is asked for more than it can follow and the driver never clips a route.
        """
        Top = HOME_VELOCITY_MAX_SPEED * MotionSpeedMaximumMultiplier
        return {Name: min(Top * Percent / 100, JOINT_GOAL_SPEED_LIMITS.get(Name, Top)) for Name in MOTOR_NAMES}

    def SetMotionSpeed(self, Percent: Any) -> tuple[bool, str]:
        if isinstance(Percent, bool) or not isinstance(Percent, (int, float)) or not 10 <= Percent <= 100:
            return False, "Motion speed must be a number from 10 to 100 percent"
        with self.lock:
            Previous = self.MotionSpeedPercent
            self.MotionSpeedPercent = round(Percent)
            if self.MotionRoute is not None and Previous != self.MotionSpeedPercent:
                Now = time.monotonic()
                Ratio = Previous / self.MotionSpeedPercent
                # Rescale only unspent timeout budget. Moving the throttle back
                # and forth cannot reset a stalled motion's watchdog.
                if self.MotionDeadline > Now:
                    self.MotionDeadline = Now + (self.MotionDeadline - Now) * Ratio
                Elapsed = max(0, Now - self.MotionStartedAt)
                Remaining = max(0, self.MotionTimeoutMs / 1000 - Elapsed)
                self.MotionTimeoutMs = math.ceil((Elapsed + Remaining * Ratio) * 1000)
            # The control loop applies this on its next guarded frame. Changing
            # speed alone never starts a stream, clears a fault, or toggles torque.
            return True, f"Motion speed set to {self.MotionSpeedPercent}%"

    def _servo_sign(self, name: str) -> int:
        try:
            value = int(self.ik_settings.get("servo_signs", {}).get(name, 1))
        except (AttributeError, TypeError, ValueError):
            value = 1
        return 1 if value >= 0 else -1

    def _display_sign(self, name: str) -> int:
        try:
            value = int(self.ik_settings.get("display_signs", {}).get(name, 1))
        except (AttributeError, TypeError, ValueError):
            value = 1
        return 1 if value >= 0 else -1

    def _display_angles(self, angles: dict[str, float]) -> dict[str, float]:
        return {
            name: float(value) * self._display_sign(name)
            for name, value in angles.items()
        }

    @staticmethod
    def _display_point(point: dict[str, float]) -> dict[str, float]:
        return {"x": -float(point["x"]), "y": float(point["y"]), "z": float(point["z"])}

    @classmethod
    def _display_joints(cls, joints: list[dict[str, float]]) -> list[dict[str, float]]:
        return [cls._display_point(point) for point in joints]

    def _raw_velocity_sign(self, name: str) -> int:
        try:
            signs = self.ik_settings.get("raw_velocity_signs", {})
            value = int(signs.get(name, 1))
        except (AttributeError, TypeError, ValueError):
            value = 1
        return 1 if value >= 0 else -1

    def _motor_velocity_speed(self, servo: ServoRuntime, track_speed: int) -> int:
        raw_direction = -1 if servo.arc_inverted else 1
        return int(track_speed) * raw_direction * self._raw_velocity_sign(servo.name)

    def _ik_zero_raw(self, name: str) -> int:
        Reference = self.arm_geometry.get("CadAlignment", {}).get("ReferenceRaw", self.home_preset)
        return Reference.get(name, self.servos[name].range_min) % RESOLUTION

    def _ik_zero_offset(self, servo: ServoRuntime) -> float:
        return float(self._arc_distance_from_left(servo, self._ik_zero_raw(servo.name)))

    def _ik_angle_from_raw(self, servo: ServoRuntime, raw: int) -> float:
        return self._ik_angle_from_track(servo, self._slider_track_pos(servo, raw))

    def _ik_angle_from_track(self, servo: ServoRuntime, offset: float) -> float:
        """The IK angle of a slider track position (_slider_track_pos, or a reading of it a turn away)."""
        if self._arc_span(servo) == 0:
            return 0.0
        return (
            self._servo_sign(servo.name)
            * (offset - self._ik_zero_offset(servo))
            * 360.0
            / RESOLUTION
        )

    def _ik_raw_from_angle(self, servo: ServoRuntime, angle_deg: float) -> int:
        span = self._arc_span(servo)
        if span == 0:
            return self._clamp_raw(servo, self._ik_zero_raw(servo.name))
        sign = self._servo_sign(servo.name)
        offset = self._ik_zero_offset(servo) + float(angle_deg) * RESOLUTION / (360.0 * sign)
        return self._fraction_to_raw(servo, offset / span)

    def _current_raw_for_ik(self, servo: ServoRuntime) -> int:
        """Where a joint is for every planning and safety input: its measured position. The GUI's "current angle"
        override (gui_angle_raw) is display only: it moved the reported tip 5.3 cm, the tape check and the carried
        envelope with it, while every route still left from the measured pose (2026-10-07 audit M7)."""
        return servo.present_raw

    def _current_ik_angles(self) -> dict[str, float]:
        return {
            name: self._ik_angle_from_raw(
                self.servos[name],
                self._current_raw_for_ik(self.servos[name]),
            )
            for name in MOTOR_NAMES
        }

    def _ik_joint_limits(self) -> dict[str, tuple[float, float]]:
        limits: dict[str, tuple[float, float]] = {}
        for name in IK_JOINTS:
            servo = self.servos[name]
            a_lo = self._ik_angle_from_raw(servo, servo.range_min)
            a_hi = self._ik_angle_from_raw(servo, servo.range_max)
            limits[name] = (min(a_lo, a_hi), max(a_lo, a_hi))
        return limits

    def _current_elbow_branch(self, angles: dict[str, float]) -> str:
        if self.arm_geometry.get("model") == "SO101Cad":
            return Cad.Branch(self.arm_geometry, angles)
        upper_zero = float(self.arm_dims.get("upper_zero_deg", 0.0))
        lower_zero = float(self.arm_dims.get("lower_zero_deg", upper_zero))
        bend = _normalize_deg(lower_zero - upper_zero + float(angles.get("elbow_flex", 0.0)))
        if abs(bend) < 1e-6:
            return self.elbow_mode
        return "up" if bend > 0.0 else "down"

    def RecordBridgeEvent(self, Kind: str, Reason: str, Joint: str | None = None,
                          Details: dict[str, Any] | None = None, Diagnostic: bool = False) -> None:
        """Keep one motion fault, protection trip, motion abort or serial link event (see AppendBridgeEvent).

        Called before the stop resets anything, so the servo record shows the
        commands and route stage the arm had when it happened. Never raises.
        Diagnostic (a control stall): recorded alike, but it is not the latest
        fault, trip or abort the state reports (last_bridge_event), which the page
        reads a latched fault's details from.
        """
        try:
            Now = time.time()
            Last = self.LastBridgeEvent
            # One record per occurrence. LatchMotionFault and TripProtection record
            # with the joint and measurements, then the stop they call reports the
            # same trip again without them. Every serial link drop and recovery is its own.
            if (not Diagnostic and Last and (Last["kind"], Last["reason"]) == (Kind, Reason) and Now - Last["t"] < 1.0
                    and not Kind.startswith("serial_link")):
                return
            Route = self.MotionRoute
            Record = {"kind": Kind, "reason": Reason, "joint": Joint,
                      "details": {"servos": {Name: {"present_raw": Servo.present_raw, "target_raw": Servo.target_raw,
                                                    "command_raw": Servo.command_raw, "command_speed": Servo.command_speed,
                                                    "feedback_ok": Servo.FeedbackOk}
                                             for Name, Servo in self.servos.items()},
                                  "route_stage": None if Route is None else Route["Index"] + 1,
                                  "route_stages": None if Route is None else len(Route["Path"]),
                                  "torque": self.torque_enabled, "calibration_active": self.CalibrationActive,
                                  **(Details or {})},
                      "t": round(Now, 3),
                      "time": time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(Now)) + f".{int(Now % 1 * 1000):03d}Z"}
            if not Diagnostic:
                self.LastBridgeEvent = Record
            if self.EventSink is not None:
                self.EventSink(Record)
        except Exception as Exc:
            print(f"[bridge-events] could not record {Kind}: {Exc}", file=sys.stderr)

    def LatchMotionFault(self, Reason: str, Joint: str | None = None, Details: dict[str, Any] | None = None) -> tuple[bool, str]:
        """StopMotion(Reason, Latch=True), recorded with the joint and the values that tripped it."""
        with self.lock:
            self.RecordBridgeEvent("motion_fault", Reason, Joint, Details)
            return self.StopMotion(Reason, Latch=True)

    def TripProtection(self, Reason: str, Joint: str | None = None, Details: dict[str, Any] | None = None) -> tuple[bool, str]:
        """AutoCoast(Reason), recorded with the joint and the values that tripped it."""
        with self.lock:
            self.RecordBridgeEvent("protection_trip", Reason, Joint, Details)
            return self.AutoCoast(Reason)

    def NoteProtectionGlitch(self, Joint: str, Register: str, Value: Any, Note: str, Details: dict[str, Any] | None = None) -> None:
        """A protection reading tolerated until it persists (ProtectionVersion v2-confirmed): kept in state.protection.glitches
        and recorded as protection_glitch in bridge_events.jsonl, so every one stays visible. A diagnostic record: it is
        never the latched fault's record the page reads (last_bridge_event)."""
        self.ProtectionGlitches.append({"servo": Joint, "register": Register, "value": Value, "note": Note, **(Details or {}),
                                        "at_s": time.time()})
        self.RecordBridgeEvent("protection_glitch", f"{Joint} {Register}: {Note}", Joint,
                               {"register": Register, "value": Value, **(Details or {})}, Diagnostic=True)

    def StopMotion(self, Reason: str = "Motion stopped", Latch: bool = False, KeepGrasp: bool = True) -> tuple[bool, str]:
        """Stop any motion and hold every joint where it is (torque on), or with Latch coast every joint (AutoCoast).

        Holding where it is includes the grip: a gripper holding an object it closed on (GraspHold) keeps squeezing it
        (SqueezeGrasp, from where its jaws rest), unless KeepGrasp is False (Coast). Re-held at its present position
        like the other joints, its 8-step squeeze, which is the grip force, went to 0 and a carried object could slip
        out while the arm held: on Stop and hold, and since the audit H1 fix also when a route's measured scene changed
        or expired mid-carry (HoldRoute) (2026-10-08 safety review)."""
        with self.lock:
            if Latch:
                return self.AutoCoast(Reason, Kind="motion_fault")
            if self.HandGuide is not None and not self.SerialLinkLost():
                # Hold here: every joint held where it is, the comp joints preloaded with the model's effort.
                return self.EndHandGuide("preloaded", Reason if Reason != "Motion stopped" else
                                         "Stop and hold: hand guiding ended, holding where it is" if self.HandGuide.Mode == "guide"
                                         else "Stop and hold: the hand-guiding calibration stopped, holding where it is")
            if self.ProtectionTrip:
                return False, self.status
            if self.SerialLinkLost():
                # Nothing reaches the servos. A move is stopped by latching its coast, sent the
                # moment the link returns; a still arm has nothing to stop.
                if self.MotionInProgress():
                    return self.AutoCoast(f"{Reason}: the USB serial link to the arm is down", Kind="motion_fault")
                self.status = f"{Reason}: nothing was moving. {self.SerialLinkDownText()}"
                return False, self.status
            if self.MotionRoute is not None:
                self.RecordBridgeEvent("motion_abort", Reason)
            self.MotionEpoch += 1
            self.CalibrationPlans.clear()
            self.CalibrationActive = False
            self.CalibrationPrepareToken = None
            self.MotionDeadline = 0.0
            self.MotionRoute = None
            self.ClearRouteScene()
            if Latch:
                self.MotionFault = Reason
            Errors = []
            for Servo in self.servos.values():
                Gripping, Squeezed = KeepGrasp and Servo.GraspHold, False
                if self.bus is not None:
                    try:
                        Mode = self.bus.read("Operating_Mode", Servo.name, normalize=False, num_retry=1)
                        # Zero stops velocity mode but means maximum speed in
                        # position mode. Limit position speed while parking.
                        self.bus.write("Goal_Speed", Servo.name, 0 if Mode == OperatingMode.VELOCITY.value else 1,
                                       normalize=False, num_retry=1)
                        # A new position command clears firmware protection. Never
                        # write one to a faulted joint or use stale feedback to hold.
                        if (Servo.FeedbackOk and not self.faults.get(Servo.name)
                                and not getattr(self.bus, "PacketFaults", {}).get(Servo.name)):
                            if Gripping:
                                self.SqueezeGrasp(Servo, Servo.present_raw)
                                Squeezed = True
                            else:
                                self._stop_slider_velocity_locked(Servo, hold_target=False, num_retry=1)
                    except Exception as Exc:
                        Errors.append(f"{Servo.name}: {Exc}")
                Servo.command_raw = None
                Servo.command_speed = 0
                Servo.homing = False
                Servo.Holding = False
                Servo.GraspHold = Squeezed
                Servo.target_raw = Servo.present_raw
            self.status = Reason + ("; stop unconfirmed: " + "; ".join(Errors) if Errors else "")
            if Errors:
                return self.AutoCoast(self.status, Kind="motion_fault")
            elif self.MotionFault and self.MotionFault not in self.status:
                self.status += ": " + self.MotionFault
            return not Errors, self.status

    def AutoCoast(self, Reason: str, Kind: str = "protection_trip") -> tuple[bool, str]:
        """Latch a trip and remove torque without writing fresh position goals.

        Kind is how the trip is recorded: a latched StopMotion is a motion fault,
        a direct call a protection trip.
        """
        with self.lock:
            self.EndHandGuide("teardown", Reason)
            self.RecordBridgeEvent(Kind, Reason)
            if self.MotionRoute is not None:
                # A route that faults keeps its frames too: 2026-10-04 23:50:01's left none, and the
                # pan's overshoot that tripped it could not be measured afterwards.
                self.RecordRouteTiming(self.MotionRoute, Fault=Reason)
            self.MotionEpoch += 1
            self.MotionFault = Reason
            self.CalibrationPlans.clear()
            self.CalibrationActive = False
            self.CalibrationPrepareToken = None
            self.MotionDeadline = 0.0
            self.MotionRoute = None
            self.ClearRouteScene()
            self.ProtectionProgress.clear()
            for Servo in self.servos.values():
                Servo.command_raw = None
                Servo.command_speed = 0
                Servo.homing = Servo.Holding = Servo.GraspHold = False
                Servo.target_raw = Servo.present_raw
            if self.ProtectionTrip is None:
                self.ProtectionTrip = {"reason": Reason, "at_s": time.time(), "coast_confirmed": False, "unconfirmed": []}
            self.VerifyProtectionCoast()
            Confirmed = self.ProtectionTrip["coast_confirmed"]
            if not self.SerialLinkLost():
                self.status = "Protection tripped: " + Reason + ("; all motors coasting" if Confirmed else "; TORQUE-OFF UNCONFIRMED: " + ", ".join(self.ProtectionTrip["unconfirmed"]))
            return Confirmed, self.status

    def ClearRouteScene(self) -> None:
        """The route that bound a measured scene is over (finished, stopped, coasted): drop the scene, so Home,
        manual moves and hold corrections never use it once it is stale (2026-10-07 audit M1, M10). A held
        object's envelope (SceneCarried) stays for the carry's next scene-bound move."""
        self.ActiveScene = None
        self.HoldReturnCache = None

    def HoldRoute(self, Why: str, Details: dict[str, Any] | None = None) -> tuple[bool, str]:
        """Stop the route and hold the arm where it is, torque on, for a cause that does not make holding unsafe:
        the measured scene changed or expired, or the configuration changed. These used to latch a fault and
        coast every joint, so the arm dropped mid-route, perhaps with an object over the desk (2026-10-07 audit
        H1, H2). Recorded as a motion abort; never latched."""
        with self.lock:
            self.RecordBridgeEvent("motion_abort", f"Route stopped, holding: {Why}", Details=Details)
            return self.StopMotion(f"Motion stopped and holding where it is: {Why}")

    def ProtectionStatus(self) -> str:
        Trip = self.ProtectionTrip
        if self.SerialLinkLost():
            return self.SerialLinkDownText()
        return "Protection tripped: " + Trip["reason"] + ("; all motors coasting" if Trip["coast_confirmed"]
                                                         else "; TORQUE-OFF UNCONFIRMED: " + ", ".join(Trip["unconfirmed"]))

    def VerifyProtectionCoast(self) -> None:
        self.LastCoastAttempt = time.monotonic()
        Unconfirmed = []
        # A lost serial port reaches no servo: the link supervisor (ServiceSerialLink) retries
        # this the moment the port is open again.
        if self.bus is not None and self.arm_connected and not self.SerialLinkLost():
            # Stop velocity registers first, in case torque-off cannot get
            # through, and remove torque LAST: a goal write re-enables torque on
            # these servos, so a speed write after torque-off re-energized the
            # arm on every retry. Never send a position goal during fault recovery.
            self.StopSpeedsBeforeTorqueOff(self.bus)
            # The driver attempts every motor even if an earlier motor fails.
            try:
                self.bus.disable_torque(num_retry=0)
            except Exception:
                pass
            for Name in MOTOR_NAMES:
                try:
                    if int(self.bus.read("Torque_Enable", Name, normalize=False, num_retry=0)):
                        Unconfirmed.append(Name)
                except Exception:
                    Unconfirmed.append(Name)
        else:
            Unconfirmed = list(MOTOR_NAMES)
        # Software commands are disabled even if a broken cable prevents
        # physical torque-off confirmation. Never claim confirmation from that.
        self.torque_enabled = False
        if self.ProtectionTrip is not None:
            self.ProtectionTrip.update(coast_confirmed=not Unconfirmed, unconfirmed=Unconfirmed)
            self.status = self.ProtectionStatus()

    @staticmethod
    def StopSpeedsBeforeTorqueOff(Bus) -> None:
        """Before a torque-off, in case it cannot get through: a velocity-mode joint's speed to 0, a position-mode joint's
        profile speed to 1 (parked slowly). A position-mode joint that already reads torque off is left alone: on these
        servos a Goal_Speed write energises it again, and hand guiding's floating joints were powered toward their stale
        goals for 10-25 ms before the torque-off that followed (2026-10-10 review M2)."""
        for Name in MOTOR_NAMES:
            try:
                Mode = Bus.read("Operating_Mode", Name, normalize=False, num_retry=0)
                if Mode != OperatingMode.VELOCITY.value and not int(Bus.read("Torque_Enable", Name, normalize=False, num_retry=0)):
                    continue
                Bus.write("Goal_Speed", Name, 0 if Mode == OperatingMode.VELOCITY.value else 1, normalize=False, num_retry=0)
            except Exception:
                pass

    # ------------------------------------------------------------------
    # USB serial link supervision
    # ------------------------------------------------------------------

    def SerialLinkLost(self) -> bool:
        """The bus's serial port itself failed and is not open again yet (FeetechMotorsBus.Reopen)."""
        return bool(self.bus is not None and getattr(self.bus, "PortLost", None))

    def RaiseIfSerialLinkLost(self, Exc: Exception) -> None:
        """A bus call failed because the port is gone: abort this frame for ServiceSerialLink.

        Called first in the handlers that latch a fault on a failed read or write. Without it a
        lost port latched "Cannot read <joint> status" and then failed every later call.
        """
        if self.SerialLinkLost():
            raise Exc if isinstance(Exc, PortLostError) else PortLostError(str(Exc))

    def MotionInProgress(self) -> bool:
        return (self.MotionRoute is not None or self.CalibrationActive or self.HandGuide is not None
                or any(S.command_raw is not None or S.homing for S in self.servos.values()))

    def MovingRefusal(self, What: str) -> str | None:
        """Why What may not happen now: the arm is moving (a route, a stream, a hold correction). Changing Home,
        a joint limit or a joint reading mid-route coasted every joint (2026-10-07 audit H2); now it waits."""
        if self.HandGuide is not None:
            return f"Hand guiding is on: press Hold here before {What}"
        if self.MotionInProgress():
            return f"Wait for the arm to stop moving (or press Stop and hold) before {What}"
        return None

    def SerialLinkDownSeconds(self) -> float:
        Link, Now = self.SerialLink, time.monotonic()
        LostAt = Link.LostAt if Link.Down else (getattr(self.bus, "PortLostAt", None) or Now)
        return max(0.0, Now - LostAt)

    def SerialLinkCounts(self) -> dict[str, Any]:
        Link, Now = self.SerialLink, time.monotonic()
        return {"port": self.port, "drops_total": Link.DropsTotal,
                "drops_recent": sum(Now - At <= SerialLinkDropWindowS for At in Link.Drops),
                "drop_window_s": SerialLinkDropWindowS, "recoveries": Link.Recoveries, "failures": Link.Failures,
                "reopen_attempts": Link.Attempts}

    def SerialLinkDownText(self) -> str:
        """What is happening while the link is down, in plain words (the bridge status)."""
        Down, Trip = self.SerialLinkDownSeconds(), self.ProtectionTrip
        if Trip and not Trip["coast_confirmed"]:
            return (f"{self.MotionFault or Trip['reason']}. The servos keep their last command until the USB serial link returns "
                    f"(down {Down:.1f} s so far); torque-off is sent the moment it does. "
                    "Support the arm and check the USB cable.")
        if Down >= SerialLinkHoldWindowS:
            return f"USB serial link to the arm down for {Down:.1f} s; still trying to reconnect: check the USB cable"
        if self.torque_enabled:
            return (f"USB serial link to the arm dropped {Down:.1f} s ago; reconnecting "
                    "(the servos keep holding their last position)")
        return f"USB serial link to the arm dropped {Down:.1f} s ago; reconnecting"

    def LiveStatus(self) -> str:
        """The status of a healthy frame: a recent serial link or control delay note stays readable for a while."""
        Link = self.SerialLink
        if Link.Note and time.monotonic() - Link.NoteAt < SerialLinkNoteSeconds:
            return Link.Note
        if self.ControlDelayNote and time.monotonic() - self.ControlDelayNoteAt < ControlDelayNoteSeconds:
            return self.ControlDelayNote
        return "Bridge live"

    def SerialLinkSnapshot(self) -> dict[str, Any]:
        Link, Lost = self.SerialLink, self.SerialLinkLost()
        Recent = Link.Note and time.monotonic() - Link.NoteAt < SerialLinkNoteSeconds
        return {"state": "down" if Lost else "ok", "version": SerialLinkVersion,
                "down_s": round(self.SerialLinkDownSeconds(), 2) if Lost else None,
                "error": str(self.bus.PortLost) if Lost else None,
                "reopen_error": getattr(self.bus, "ReopenError", None) if Lost else None,
                "hold_window_s": SerialLinkHoldWindowS, **self.SerialLinkCounts(),
                "last_outage_s": Link.LastOutageS, "note": Link.Note if Recent else None,
                "note_age_s": round(time.monotonic() - Link.NoteAt, 1) if Recent else None}

    def ServiceSerialLink(self) -> bool:
        """Supervise the USB serial link; True when the bus can be used this frame.

        A drop is recorded once (BeginSerialOutage), and a move in progress is latched at once:
        its velocity streams run on unsupervised. The same port is reopened every
        SerialLinkReopenIntervalS, then every SerialLinkRetryIntervalS once the hold window has
        passed, which latches a fault while holding. A reopen ends in FinishSerialOutage.
        """
        if not self.SerialLinkLost():
            return True
        Link, Now = self.SerialLink, time.monotonic()
        if not Link.Down:
            self.BeginSerialOutage()
        Down = Now - Link.LostAt
        if Now - Link.LastAttempt >= (SerialLinkReopenIntervalS if Down < SerialLinkHoldWindowS else SerialLinkRetryIntervalS):
            Link.LastAttempt = Now
            Link.Attempts += 1
            if self.bus.Reopen():
                return self.FinishSerialOutage()
        if not Link.WindowExpired and Down >= SerialLinkHoldWindowS:
            Link.WindowExpired = True
            Link.Failures += 1
            Reason = (f"USB serial link to the arm dropped and could not reconnect within {SerialLinkHoldWindowS:g} s: "
                      "check the USB cable")
            Details = {"error": Link.Error, "reopen_error": getattr(self.bus, "ReopenError", None),
                       "down_s": round(Down, 2), "serial_link": self.SerialLinkCounts()}
            if self.torque_enabled and not self.ProtectionTrip:
                self.LatchMotionFault(Reason, Details=Details)
            else:
                self.RecordBridgeEvent("serial_link_failed", Reason, Details=Details)
        self.status = self.SerialLinkDownText()
        return False

    def BeginSerialOutage(self, Latch: bool = True) -> None:
        """Record a new drop. Latch: a move in progress, or the drop limit reached, trips protection."""
        Link, Bus, Now = self.SerialLink, self.bus, time.monotonic()
        Link.Down, Link.WindowExpired = True, False
        Link.LostAt = getattr(Bus, "PortLostAt", None) or Now
        Link.Error = str(getattr(Bus, "PortLost", "") or "")
        Link.MovingAtLoss = self.MotionInProgress()
        # The last fresh positions: a holding joint must still be there after the drop.
        Link.PositionsAtLoss = {Name: Servo.present_raw for Name, Servo in self.servos.items()}
        for Servo in self.servos.values():
            Servo.FeedbackOk = False  # nothing is read until the port is back
        Link.Attempts, Link.LastAttempt = 0, 0.0
        Link.DropsTotal += 1
        Link.Drops.append(Now)
        Counts = self.SerialLinkCounts()
        self.RecordBridgeEvent("serial_link_lost", f"USB serial link to the arm dropped: {Link.Error}",
                               Details={"error": Link.Error, "moving": Link.MovingAtLoss, "serial_link": Counts})
        if not Latch or self.ProtectionTrip:
            return  # an existing trip only waits for the link to send its torque-off
        Details = {"error": Link.Error, "serial_link": Counts}
        if Link.MovingAtLoss:
            self.LatchMotionFault("USB serial link to the arm dropped during a move; the move is cancelled", Details=Details)
        elif self.torque_enabled and Counts["drops_recent"] >= SerialLinkDropLimit:
            self.LatchMotionFault(f"USB serial link to the arm dropped {Counts['drops_recent']} times in "
                                  f"{SerialLinkDropWindowS:g} s: check the USB cable and port", Details=Details)

    def FinishSerialOutage(self) -> bool:
        """The port is open again: send a pending torque-off, read every joint, then say what happened.

        True when this frame may go on with its normal reads and writes: the arm held still
        through the drop and every fresh read agrees. False when the frame ends here.
        """
        Link = self.SerialLink
        Outage = time.monotonic() - Link.LostAt
        Base = f"USB serial link dropped for {Outage:.1f} s; reconnected"
        self.LastProtectionPoll = 0.0  # temperatures are stale after a drop: read in the next frame
        Trip = self.ProtectionTrip
        if Trip and not Trip["coast_confirmed"]:
            # A latched fault (a move stopped, the hold window passed, a coast or stop requested)
            # sends its torque-off first. Stop and torque-off writes only take motion away.
            self.VerifyProtectionCoast()
            if self.SerialLinkLost():
                return False  # dropped again: the same outage goes on
            self._read_positions_locked()  # fresh positions for the display; the fault is already latched
            if self.SerialLinkLost():
                return False
            Confirmed = self.ProtectionTrip["coast_confirmed"]
            self.EndSerialOutage(Base + ("; all motors coasting. Support the arm, then press Hold." if Confirmed else
                                         "; torque-off still UNCONFIRMED on " + ", ".join(self.ProtectionTrip["unconfirmed"])
                                         + ". Support the arm."),
                                 Details={"coast_confirmed": Confirmed, "fault": self.MotionFault})
            return False
        Problems, Energised, Silent = self.ReadAfterSerialOutage()
        if self.SerialLinkLost():
            return False
        Holding = self.torque_enabled
        if (Problems and Holding) or Energised:
            # Holding: anything off. Coasting: a joint that holds torque all the same (a goal
            # write that went out as the link dropped, e.g. connect's Goal_Position seed) is
            # coasted now, never left energised and unsupervised behind "torque off".
            Reason = Base + ", but " + "; ".join(Problems) + ("" if Holding else " although the arm should be coasting")
            self.EndSerialOutage(Reason + ": coasting", Details={"problems": Problems})
            self.LatchMotionFault(Reason, Details={"problems": Problems, "outage_s": round(Outage, 2),
                                                   "serial_link": self.SerialLinkCounts()})
            return False
        # "Torque off" only for joints read as off: one that did not answer is unknown.
        Note = Base + ("; holding at the last position" if Holding else
                       "; torque is off on every joint that answered" if Silent else "; the arm is coasting (torque off)")
        self.EndSerialOutage(Note + (f" ({'; '.join(Problems)})" if Problems else ""), Details={"problems": Problems})
        return True

    def ReadAfterSerialOutage(self) -> tuple[list[str], list[str], list[str]]:
        """Fresh reads of every joint after a drop, before anything commands motion.

        Returns what disagrees, the joints holding torque while the arm should coast, and the
        joints that did not answer. Position, torque, hardware status and mode, each retried
        once. A holding joint must be torqued, fault-free, in position mode and within
        SerialLinkHoldDriftRaw of where it was before the drop; holding stops at the first
        joint that does not answer, coasting reads on to find any joint still energised.
        """
        return self.ReadEveryJoint(self.SerialLink.PositionsAtLoss, "while the link was down")

    def ReadEveryJoint(self, Before: dict[str, int], While: str) -> tuple[list[str], list[str], list[str]]:
        """ReadAfterSerialOutage's reads and checks, against Before (raw positions) for a time the bridge did
        not watch the arm (While, for the drift message): after a serial link drop or a delayed control frame."""
        Problems, Holding = [], self.torque_enabled
        Energised, Silent = [], []
        for Servo in self.servos.values():
            Servo.FeedbackOk = False
        for Name in MOTOR_NAMES:
            Servo = self.servos[Name]
            try:
                Raw = int(self.bus.read("Present_Position", Name, normalize=False, num_retry=1))
                Torque = bool(self.bus.read("Torque_Enable", Name, normalize=False, num_retry=1))
                Bits = int(self.bus.read("Hardware_Error_Status", Name, normalize=False, num_retry=1))
                Mode = int(self.bus.read("Operating_Mode", Name, normalize=False, num_retry=1))
            except Exception as Exc:
                if self.SerialLinkLost():
                    break
                Problems.append(f"{Name} did not answer ({Exc})")
                Silent.append(Name)
                if Holding:
                    break
                continue
            Servo.present_raw, Servo.HardwareRaw, Servo.FeedbackOk = Raw % RESOLUTION, Raw, True
            if Torque and not Holding:
                Energised.append(Name)
            if Torque != Holding:
                Problems.append(f"{Name} torque is {'on' if Torque else 'off'}")
            if Bits:
                Problems.append(f"{Name} reports hardware status {Bits}")
            if Holding and Mode != OperatingMode.POSITION.value:
                Problems.append(f"{Name} is not in position mode")
            Moved = self._circular_raw_distance(Servo.present_raw, Before.get(Name, Servo.present_raw))
            if Holding and Moved > SerialLinkHoldDriftRaw:
                Problems.append(f"{Name} moved {Moved} steps {While}")
        return Problems, Energised, Silent

    def EndSerialOutage(self, Note: str, Details: dict[str, Any] | None = None) -> None:
        """Close the current drop: count it, record it and show Note as the status."""
        Link = self.SerialLink
        Outage = self.SerialLinkDownSeconds()
        Link.Down = False
        Link.Recoveries += 1
        Link.LastOutageS = round(Outage, 2)
        Link.Note, Link.NoteAt = Note, time.monotonic()
        self.RecordBridgeEvent("serial_link_recovered", Note,
                               Details={"outage_s": round(Outage, 2), "error": Link.Error,
                                        "serial_link": self.SerialLinkCounts(), **(Details or {})})
        self.status = Note

    def CheckMotorProtection(self, BeforeArm: bool = False) -> None:
        Now = time.monotonic()
        if not self.arm_connected or self.bus is None:
            return
        if BeforeArm or Now - self.LastProtectionPoll >= .25:
            self.LastProtectionPoll = Now
            for Name in MOTOR_NAMES:
                try:
                    Temperature = int(self.bus.read("Present_Temperature", Name, normalize=False, num_retry=1))
                    if not 0 <= Temperature <= 150:
                        raise ValueError("invalid temperature feedback")
                except Exception as Exc:
                    self.RaiseIfSerialLinkLost(Exc)
                    if BeforeArm:
                        raise RuntimeError(f"Cannot verify {Name} temperature: {Exc}")
                    # One missed reply is not lost feedback: 2026-10-12 16:17:25 one coasted a healthy arm, every
                    # position reading fine. Trip once ProtectionFeedbackMisses polls in a row failed, or at once when
                    # this joint's position did not read either (the servo itself silent).
                    Misses = self.TemperatureMisses[Name] = self.TemperatureMisses.get(Name, 0) + 1
                    if Misses >= ProtectionFeedbackMisses or not self.servos[Name].FeedbackOk:
                        self.TripProtection(f"Lost {Name} protection feedback: {Exc}" + (f" ({Misses} polls in a row)" if Misses > 1 else ""),
                                            Joint=Name, Details={"error": str(Exc), "misses": Misses,
                                                                 "feedback_ok": self.servos[Name].FeedbackOk})
                        return
                    self.NoteProtectionGlitch(Name, "Present_Temperature", None, "missed reply; read again at the next poll",
                                              {"error": str(Exc), "misses": Misses, "limit": ProtectionFeedbackMisses})
                    continue
                self.TemperatureMisses[Name] = 0
                Limit = ProtectionRearmTemperatureC if BeforeArm else ProtectionTemperatureC
                if Temperature >= Limit:
                    # Confirm before acting: a lone corrupted byte read 83 C on a 28 C servo and
                    # dropped the arm mid-route. Real heating takes minutes, a re-read milliseconds.
                    try:
                        Confirm = int(self.bus.read("Present_Temperature", Name, normalize=False, num_retry=1))
                    except Exception:
                        Confirm = Temperature
                    if 0 <= Confirm < Limit:
                        self.NoteProtectionGlitch(Name, "Present_Temperature", Temperature, f"read {Temperature} C, re-read {Confirm} C",
                                                  {"read_c": Temperature, "confirmed_c": Confirm})
                        Temperature = Confirm
                self.ProtectionTelemetry[Name] = {"temperature_c": Temperature, "at_s": time.time()}
                if Temperature >= Limit:
                    if BeforeArm:
                        raise RuntimeError(f"{Name} is {Temperature} C; cool below {Limit} C before Hold")
                    self.TripProtection(f"{Name} temperature {Temperature} C reached software limit {Limit} C", Joint=Name,
                                        Details={"temperature_c": Temperature, "limit_c": Limit})
                    return
        if BeforeArm or not self.torque_enabled or self.MotionFault:
            return
        for Name, Servo in self.servos.items():
            Error = abs(self._slider_track_error(Servo))
            # A grasp can stop the jaws anywhere short of the target, a thin object only a few steps short.
            Grasp = Servo.command_raw is not None and self.GraspClosing(Servo)
            if Servo.command_raw is None or Error <= (self._slider_goal_tolerance_raw(Name) if Grasp
                                                      else max(30, self._slider_goal_tolerance_raw(Name) * 2)):
                self.ProtectionProgress.pop(Name, None)
                continue
            Previous = self.ProtectionProgress.get(Name)
            # Under the grasp's lower threshold a record survives the switch to the next
            # route stage, whose farther target reads as lost progress: it starts afresh.
            if Previous is None or Error <= Previous[0] - 2 or (Grasp and Previous[2] != Servo.target_raw):
                self.ProtectionProgress[Name] = (Error, Now, Servo.target_raw)
            elif Grasp and Now - Previous[1] >= GRIPPER_GRASP_STALL_SECONDS:
                if not self.HoldGrasp(Servo, Error, Now - Previous[1]):
                    return
            elif Now - Previous[1] >= ProtectionStallSeconds:
                self.TripProtection(f"{Name} stalled: no progress toward its target for {ProtectionStallSeconds:g} seconds",
                                    Joint=Name, Details={"error_raw": round(Error), "error_at_last_progress_raw": round(Previous[0]),
                                                         "stalled_s": round(Now - Previous[1], 3),
                                                         "present_raw": Servo.present_raw, "target_raw": Servo.target_raw})
                return

    def GraspClosing(self, Servo: ServoRuntime) -> bool:
        """The gripper is closing on a route that moves nothing else from this stage on.

        Only such a commanded close becomes a grasp when it stalls (GRIPPER_GRASP_STALL_SECONDS).
        Opening into something, or closing while arm joints also move, keeps the stall trip. Home's close
        is such a stage: the last of its route, once the arm has arrived (PlanHomeRoute).
        """
        Route = self.MotionRoute
        if Servo.name != GRIPPER_NAME or Route is None or self._slider_track_error(Servo) >= 0:
            return False
        return all(Point[Name] == Route["Start"][Name] for Point in Route["Path"][Route["Index"]:]
                   for Name in MOTOR_NAMES if Name != GRIPPER_NAME)

    def HoldGrasp(self, Servo: ServoRuntime, Error: float, StalledFor: float) -> bool:
        """The gripper stopped on something while closing: hold it as a grasp instead of coasting the arm.

        The jaws stay squeezed on the object (SqueezeGrasp) and the gripper-only route ends
        there, as a completed one does. False after tripping protection, as the stall would
        have, when the hold could not be written.
        """
        Route, Present = self.MotionRoute, Servo.present_raw
        try:
            Goal = self.SqueezeGrasp(Servo, Present)
        except Exception as Exc:
            self.RaiseIfSerialLinkLost(Exc)
            self.TripProtection(f"{Servo.name} stalled while closing and its grasp hold could not be written: {Exc}",
                                Joint=Servo.name, Details={"error": str(Exc), "error_raw": round(Error), "present_raw": Present})
            return False
        self.RecordBridgeEvent("grasp_hold", "Gripper stopped on an object while closing; holding it", Servo.name,
                               {"present_raw": Present, "goal_raw": Goal, "commanded_raw": Route["Path"][-1][Servo.name],
                                "squeeze_raw": GRIPPER_GRASP_SQUEEZE_RAW, "error_raw": round(Error),
                                "stalled_s": round(StalledFor, 3)})
        self.ProtectionProgress.pop(Servo.name, None)
        self.MotionRoute = None
        self.ClearRouteScene()
        self.CalibrationActive = False
        self.MotionDeadline = 0.0
        for Other in self.servos.values():
            Other.homing = False
        self.status = f"Gripper holding an object at raw {Present}"
        return True

    def SqueezeGrasp(self, Servo: ServoRuntime, FromRaw: int) -> int:
        """Hold the gripper in position mode GRIPPER_GRASP_SQUEEZE_RAW steps more closed than FromRaw.

        The servo's own position loop then presses on the object with a force bounded by
        that offset. Its target stays on the jaws (_write_active_slider_commands_locked keeps
        it there), so no correction drives them further into the object. Returns the goal.
        """
        Closer = GRIPPER_GRASP_SQUEEZE_RAW if Servo.arc_inverted else -GRIPPER_GRASP_SQUEEZE_RAW
        Servo.target_raw = self._clamp_raw(Servo, FromRaw + Closer)
        self._stop_slider_velocity_locked(Servo, hold_target=True, num_retry=1)
        Goal = Servo.target_raw
        Servo.target_raw = self._clamp_raw(Servo, Servo.present_raw)
        Servo.GraspHold = True
        return Goal

    def PrepareCalibration(self) -> dict[str, Any]:
        """Explicit retry of a stopped software timeout; never starts motion."""
        with self.lock:
            if self.HandGuide is not None:
                return {"ok": False, "error": "Hand guiding is on: press Hold here before calibration.", "state": self.state_snapshot()}
            if self.ProtectionTrip:
                return {"ok": False, "error": "Protection is latched. Resolve the cause, support the arm, then explicitly enable Hold.", "state": self.state_snapshot()}
            try:
                if not self.arm_connected or self.bus is None:
                    raise RuntimeError("Connect the arm first.")
                Recovering = bool(self.MotionFault and self.MotionFault.startswith("Motion settling timed out"))
                if self.MotionFault and not Recovering:
                    raise RuntimeError(self.MotionFault + ". Resolve the fault before calibration.")
                # A route parked between waypoints has no stream running, and the epoch bumped below ends it.
                if self.MotionRoute is not None or any(Servo.command_raw is not None for Servo in self.servos.values()):
                    raise RuntimeError("Wait for the arm to finish moving before calibration.")
                if Recovering and not self.torque_enabled:
                    raise RuntimeError("Enable Hold before retrying the motion timeout.")
                if not self._read_positions_locked():
                    raise RuntimeError("Joint feedback unavailable; calibration cannot start.")
                if not self.last_refresh_s or time.monotonic() - self.last_refresh_s > 0.5:
                    raise RuntimeError("Bridge control heartbeat is stale; calibration cannot start.")
                for Name, Servo in self.servos.items():
                    Bits = self.bus.read("Hardware_Error_Status", Name, normalize=False, num_retry=0)
                    Torque = bool(self.bus.read("Torque_Enable", Name, normalize=False, num_retry=0))
                    if Bits or getattr(self.bus, "PacketFaults", {}).get(Name):
                        raise RuntimeError(f"{Name} reports a hardware fault; resolve it before calibration.")
                    if Torque != self.torque_enabled:
                        raise RuntimeError(f"{Name} torque state is unexpected; resolve it before calibration.")
                    if Recovering:
                        Mode = self.bus.read("Operating_Mode", Name, normalize=False, num_retry=0)
                        Goal = self.bus.read("Goal_Position", Name, normalize=False, num_retry=0)
                        Moving = self.bus.read("Moving", Name, normalize=False, num_retry=0)
                        if (Mode != OperatingMode.POSITION.value or Moving
                                or abs(Goal - Servo.HardwareRaw) > self._slider_goal_tolerance_raw(Name)):
                            raise RuntimeError(f"{Name} is not settled in Hold; cannot clear the motion timeout.")
                # Do this only after every read succeeds. No torque cycle, goal
                # write, or automatic resumption of the interrupted motion.
                self.MotionFault = None
                self.MotionDeadline = 0.0
                self.CalibrationPlans.clear()
                self.MotionEpoch += 1
                self.CalibrationPrepareToken = uuid.uuid4().hex
                for Name in IK_JOINTS:
                    self.servos[Name].gui_angle_raw = None
                self.status = "Ready for calibration"
                return {"ok": True, "state": self.state_snapshot()}
            except Exception as Exc:
                return {"ok": False, "error": str(Exc), "state": self.state_snapshot()}

    def MotionPreflight(self) -> None:
        if self.HandGuide is not None:
            raise RuntimeError("Hand guiding is on: press Hold here first")
        if self.SerialLinkLost():
            raise RuntimeError("Cannot move: " + self.SerialLinkDownText())
        if self.MotionFault:
            raise RuntimeError(self.MotionFault + ". Resolve the cause, then use Coast and Hold to recover.")
        if not self.arm_connected or not self.torque_enabled:
            raise RuntimeError("Connect the arm and enable Hold before moving")
        if not self._read_positions_locked():
            if self.SerialLinkLost():
                raise RuntimeError("Cannot move: " + self.SerialLinkDownText())
            raise RuntimeError(self.MotionFault or "Joint feedback unavailable")
        self._check_servo_faults_locked()
        if self.MotionFault:
            raise RuntimeError(self.MotionFault)

    def ConfigurationKey(self) -> str:
        return json.dumps([self.home_preset, self.arm_geometry, _load_json(BACKEND_ROOT / "WorkspaceFrame.json"), Collision.Model,
                           [(S.name, S.range_min, S.range_max, S.arc_inverted, S.gui_angle_raw)
                            for S in self.servos.values()]], sort_keys=True)

    def SolverSnapshot(self):
        Snapshot = copy.copy(self)
        Snapshot.servos = copy.deepcopy(self.servos)
        Snapshot.home_preset = dict(self.home_preset)
        Snapshot.arm_geometry = copy.deepcopy(self.arm_geometry)
        Snapshot.target_cm = dict(self.target_cm)
        Snapshot.bus = None
        return Snapshot

    def PlanningSnapshot(self):
        """A solver snapshot for planning a motion, called with the lock: its collision worlds end the planning
        (BasketCollision.PlanningCancelled) as soon as this controller's MotionEpoch moves on, i.e. a Stop, Hold,
        Coast, emergency stop or another route came in meanwhile (2026-10-07 audit M2). Snapshots taken from it
        keep the hook."""
        Snapshot = self.SolverSnapshot()
        Live, Epoch = self, self.MotionEpoch
        Snapshot.PlanningCancel = lambda: Live.MotionEpoch != Epoch
        return Snapshot

    def BindScene(self, SceneId=None, Required=False, Carry=None):
        """Bind server-owned geometry before solving. Called on a detached solver snapshot, outside the lock.

        Only a motion that names its scene is bound to it. Home, jogs and every other manual move name none
        and plan without a scene, however recent the last AI move's was: they used to inherit it and were
        refused once it was stale (2026-10-07 audit M1).
        """
        if Required and not SceneId:
            raise ValueError("This motion needs its measured scene; capture and approve a new plan.")
        if SceneId is None:
            self.ActiveScene = None
            return
        Scene = SceneSafety.GetValid(SceneId, calibration.camera_model())
        self.ActiveScene = Scene
        Held = copy.deepcopy(getattr(self, "SceneCarried", None))
        Jaw = self.servos["gripper"]
        if Carry is False and self._raw_to_track_fraction(Jaw, Jaw.present_raw) >= 0.8:
            Held = None  # Only discard an envelope once the jaws actually opened.
        if Carry is True and Held is None:
            Item = Scene.get("task_object")
            if not Item:
                raise ValueError("The held object's size is unknown; measure it before carrying it.")
            Frame = _load_json(BACKEND_ROOT / "WorkspaceFrame.json")
            Bounds = SceneSafety.ModelBounds(Item, Frame)
            Links, _ = Cad.Forward(self.arm_geometry, self._current_ik_angles())
            Transform = Links["gripper_link"]
            Tip = np.array(list(forward_tip_position(self.arm_geometry, self._current_ik_angles()).values()))
            if np.linalg.norm(np.maximum(np.maximum(Bounds[0] - Tip, Tip - Bounds[1]), 0)) > 8:
                raise ValueError("The gripper is too far from the measured object to carry it.")
            Corners = np.array(list(itertools.product(*zip(Bounds[0], Bounds[1]))))
            Local = (Corners - Transform[:3, 3]) @ Transform[:3, :3]
            Held = {"bounds": [Local.min(axis=0).tolist(), Local.max(axis=0).tolist()],
                    "label": Item.get("label"), "source_revision": Scene["revision"]}
        self.SceneCarried = Held

    def CollisionFrame(self):
        Frame = _load_json(BACKEND_ROOT / "WorkspaceFrame.json")
        Scene = getattr(self, "ActiveScene", None)
        if Scene:
            Frame["SceneObstacles"] = Scene["obstacles"]
            Frame["SceneCarried"] = getattr(self, "SceneCarried", None)
            # Observed free space, and the space known to hold nothing (SceneGeometry.Build: the arm at the capture, the task's reach).
            Frame["SceneFree"] = list(Scene.get("free_space", [])) + list(Scene.get("known_space", []))
            if Scene.get("task_object"):
                Frame["SceneFree"].append(Scene["task_object"])
        return Frame

    def SceneDispatchRefusal(self, Snapshot, Start, Path) -> str | None:
        """Why a route planned on Snapshot may not start now, or None. Called with the lock, before anything is written.

        Its measured scene must still be current (SceneSafety.Check: no deep copy under the lock), and stay usable
        until the route should have finished: twice its estimated time plus SceneDispatchMarginS. A scene that
        expired mid-route used to coast the arm (2026-10-07 audit H1); it now stops the route holding, and this
        keeps that from happening on an ordinary route.
        """
        Scene = getattr(Snapshot, "ActiveScene", None)
        if not Scene:
            return None
        try:
            SceneSafety.Check(Scene["id"], calibration.camera_model())
        except ValueError as Error:
            return str(Error)
        Remaining = SceneSafety.RemainingSeconds(Scene["id"]) or 0.0
        Needed = 2 * self.RouteDuration(Start, Path) + SceneDispatchMarginS
        if Remaining < Needed:
            return (f"The measured scene expires in {max(0.0, Remaining):.0f} s, before this route would safely finish "
                    f"(about {Needed:.0f} s with margin): capture and approve a new plan.")
        return None

    def AdoptScene(self, Snapshot):
        """Take the scene and held-object envelope the route was planned with (SceneDispatchRefusal checked them)."""
        self.ActiveScene = getattr(Snapshot, "ActiveScene", None)
        self.SceneCarried = getattr(Snapshot, "SceneCarried", None)
        self.HoldReturnCache = None

    def SceneBoundRouteActive(self) -> bool:
        """A route certified against the current measured scene is running (a /locate now would revoke it)."""
        return self.MotionRoute is not None and bool(self.ActiveScene)

    @staticmethod
    def RouteReachRaw(Tight, Count, Index, Displaced, Arrive=frozenset(), Reach=None) -> tuple[dict[str, int], dict[str, int]]:
        """Waypoint reach (encoder steps, per joint) at the start and goal ends of stage Index of a Count-waypoint route.

        Only waypoints the arm flies past get it; the route start and tight
        waypoints do not (an empty goal reach), and the route end only its end
        reach (EndReach: what a joint carrying on into the last stage may already
        have overshot), where that certified. At the start end it goes
        only to joints that may still be settling from a waypoint (Displaced:
        joint -> reach, from RouteDisplaced; a plain set means WaypointReachRaw
        each), and not after an arrive waypoint, where every joint has arrived
        before the stage starts. At the goal end it is the route's reach for that
        waypoint (Reach, from AssignRouteReach), else WaypointReachRaw for every
        joint. A goal reach beyond WaypointReachRaw is only ever given to a joint
        that carries on the same way after the waypoint, within the next stage's
        own travel: its overshoot lies inside the next stage, so the start end
        never needs more than WaypointReachRaw.
        """
        Flown = Index > 0 and Index - 1 not in Tight and Index - 1 not in Arrive
        Settling = Displaced if isinstance(Displaced, dict) else dict.fromkeys(Displaced, Collision.WaypointReachRaw)
        Start = {Name: Settling[Name] if Flown and Name in Settling else 0 for Name in MOTOR_NAMES}
        if Index in Tight:
            return Start, {}
        if Index >= Count - 1:
            # The route end: only the end reach its last stage was certified with (EndReach), if any.
            return Start, dict((Reach or {}).get(Index) or {})
        return Start, dict((Reach or {}).get(Index) or dict.fromkeys(MOTOR_NAMES, Collision.WaypointReachRaw))

    @staticmethod
    def RouteDisplaced(Displaced, Previous, Point, Tight, Reach=None) -> dict[str, int]:
        """Joints that may still be off waypoint Point, beyond the plain allowance, when the stage after it begins,
        each with the reach (encoder steps) the next stage needs at its start for it.

        A joint that moved into a waypoint the arm flies past can trail it within
        the fly-by tolerance or overshoot it within the waypoint reach (Reach,
        that waypoint's goal reach per joint; WaypointReachRaw when not given),
        and a displaced joint that then holds still stays displaced. Beyond
        WaypointReachRaw a goal reach only goes to joints carrying on, whose
        overshoot lies ahead inside the next stage, so the start needs at most
        that. At a tight (or arrive) waypoint, Tight here, every joint is within
        the allowance before the next stage starts.
        """
        if Tight:
            return {}
        Settling = dict(Displaced) if isinstance(Displaced, dict) else dict.fromkeys(Displaced, Collision.WaypointReachRaw)
        for Name in MOTOR_NAMES:
            if Point[Name] != Previous[Name]:
                Settling[Name] = min(Collision.WaypointReachRaw, (Reach or {}).get(Name, Collision.WaypointReachRaw))
        return Settling

    def ValidateMotionPath(self, Targets: dict[str, int], Relief=None,
                           Reach: tuple[dict[str, int], dict[str, int] | int] = ({}, 0), Budget: int = 256) -> None:
        """Certify the stage from where the arm is to Targets. Reach: waypoint reach (encoder steps) at the start, per
        joint, and at the goal, per joint or one value for every joint (RouteReachRaw)."""
        Present = {Name: Servo.present_raw for Name, Servo in self.servos.items()}
        Start, Goal, Budget, Relief, Reach = self.EdgeCheckArguments(Present, Targets, Relief, Reach, Budget)
        Reason = self.CollisionWorld().EdgeReason(Start, Goal, Budget=Budget, Relief=Relief, Reach=Reach)
        if Reason:
            raise ValueError(Reason)

    def EdgeCheckArguments(self, From, Targets, Relief=None, Reach=({}, 0), Budget=256):
        """World.EdgeReason's arguments (start and goal angles, budget, relief, reach in degrees) for the stage from raw
        pose From to Targets, as ValidateMotionPath checks it: the same question for a check worker (RouteCheckPool)."""
        Current = {Name: self._ik_angle_from_raw(self.servos[Name], From[Name]) for Name in MOTOR_NAMES}
        Goal = {Name: self._ik_angle_from_raw(self.servos[Name], Targets.get(Name, From[Name])) for Name in MOTOR_NAMES}
        StartReach = Collision.Np.array([Reach[0].get(Name, 0) for Name in MOTOR_NAMES]) * 360 / RESOLUTION
        GoalReach = (Collision.Np.array([Reach[1].get(Name, 0) for Name in MOTOR_NAMES]) if isinstance(Reach[1], dict)
                     else Reach[1]) * 360 / RESOLUTION
        return (Collision.Np.array([Current[N] for N in MOTOR_NAMES]), Collision.Np.array([Goal[N] for N in MOTOR_NAMES]),
                Budget, Relief, (StartReach, GoalReach))

    def HoldReturnClear(self) -> bool:
        """Whether the velocity follower may drive drifted holding joints back.

        Both ends are measured poses: where the arm is and where it was holding.
        Each end keeps its own clearance instead of the full tracking allowance,
        which a pose resting beside the mast does not have. The result is cached
        per drift state so a blocked return is not re-solved at 30 Hz.

        It runs inside the control frame, under the motor lock, so it checks the
        basket, body and rearward limit only, never a measured scene: with a
        scene bound it took 121 ms on a synthetic shelf (and 5x the obstacles
        were measured live), meanwhile no feedback was read and an emergency
        stop waited for the lock (2026-10-07 audit M10). The hold point is where
        the arm just was.
        """
        Targets = {Name: Joint.target_raw for Name, Joint in self.servos.items() if Joint.Holding}
        Key = tuple((Name, Servo.present_raw // 4, Targets.get(Name)) for Name, Servo in self.servos.items())
        if self.HoldReturnCache and self.HoldReturnCache[0] == Key:
            self.HoldCorrectionPaused = not self.HoldReturnCache[1]
            return self.HoldReturnCache[1]
        World = self.CollisionWorld(Scene=False)
        # Each joint read as the follower drives it (_slider_track_present): a hold point outside the arc is
        # returned to the short way, not around a turn.
        Tracks = {Name: self._slider_track_present(S) for Name, S in self.servos.items()}
        Present = Collision.Np.array([self._ik_angle_from_track(S, Tracks[Name][0]) for Name, S in self.servos.items()])
        Hold = Collision.Np.array([self._ik_angle_from_track(S, Tracks[Name][1] if Name in Targets else Tracks[Name][0])
                                   for Name, S in self.servos.items()])
        try:
            Near, Far = World.StartRelief(Present), World.StartRelief(Hold)
            Insets = [Relief["InsetCm"] for Relief in (Near, Far) if Relief]
            Relief = None if Near is None and Far is None else {
                "SlackDegrees": World.Slack if Near is None else Near["SlackDegrees"],
                "FarSlackDegrees": World.Slack if Far is None else Far["SlackDegrees"],
                "RearwardSlackCm": max(0.0, *(Relief["RearwardSlackCm"] for Relief in (Near, Far) if Relief)),
                "InsetCm": {Index: max(Inset.get(Index, 0.0) for Inset in Insets) for Inset in Insets for Index in Inset}}
            Clear = World.EdgeReason(Present, Hold, Budget=64, Relief=Relief) is None
        except ValueError:
            Clear = False
        self.HoldCorrectionPaused = not Clear
        self.HoldReturnCache = (Key, Clear)
        return Clear

    def CollisionWorld(self, Scene: bool = True):
        """The collision world: basket, body and rearward limit, plus the bound measured scene unless Scene is False.
        On a planning snapshot (PlanningSnapshot) it stops the planning once the motion was cancelled."""
        Frame = self.CollisionFrame() if Scene else _load_json(BACKEND_ROOT / "WorkspaceFrame.json")
        World = Collision.CollisionWorld(self.arm_geometry, Frame)
        World.Cancel = getattr(self, "PlanningCancel", None)
        return World

    def MotionRouteCost(self):
        """Estimate leg duration from commanded speeds, not a hardware ETA.

        Intermediate waypoints are passed without stopping and every joint is
        timed to the slowest, so a leg costs its largest joint travel at cruise
        speed plus a feedback frame. The final slowing approach is common to
        every candidate route and is left out.
        """
        Ceilings = self.MotionSpeedCeilings(self.MotionSpeedPercent)
        Cruise = np.array([max(1, round(Ceilings[Name])) for Name in MOTOR_NAMES])

        def Cost(Start, Goal):
            return BRIDGE_FRAME_S + float(np.max(np.abs(Goal - Start) * RESOLUTION / 360 / Cruise))
        return Cost

    def RouteDuration(self, Start, Path):
        """Relative time of a raw route: fly-by legs plus one slowing approach at the end."""
        Cost = self.MotionRouteCost()
        Angles = [np.array([self._ik_angle_from_raw(self.servos[N], Point[N]) for N in MOTOR_NAMES])
                  for Point in (Start, *Path)]
        Approach = ROUTE_SLOW_ZONE_RAW / max(1, HOME_VELOCITY_MAX_SPEED * MotionSpeedMaximumMultiplier
                                                        * self.MotionSpeedPercent / 100)
        return sum(Cost(A, B) for A, B in zip(Angles, Angles[1:])) + Approach

    def SnapAngles(self, Values):
        """Nearest joint angles (MOTOR_NAMES order) the servos can hold exactly. Measured angles stay as they are."""
        Result = []
        for Name, Value in zip(MOTOR_NAMES, Values):
            Servo = self.servos[Name]
            Measured = self._ik_angle_from_raw(Servo, Servo.present_raw)
            Result.append(Measured if Value == Measured else self._ik_angle_from_raw(Servo, self._ik_raw_from_angle(Servo, Value)))
        return np.array(Result)

    def PlanMotionRoute(self, Targets, Seconds=5.0, LocalOnly=False, AssignReach=True):
        Current = {Name: self._ik_angle_from_raw(Servo, Servo.present_raw) for Name, Servo in self.servos.items()}
        Goal = {Name: self._ik_angle_from_raw(Servo, Targets.get(Name, Servo.present_raw)) for Name, Servo in self.servos.items()}
        Limits = {Name: sorted((self._ik_angle_from_raw(Servo, Servo.range_min),
                               self._ik_angle_from_raw(Servo, Servo.range_max))) for Name, Servo in self.servos.items()}
        World = self.CollisionWorld()
        Path = World.Plan(Current, Goal, Limits, Seconds=Seconds, EdgeCost=self.MotionRouteCost(), LocalOnly=LocalOnly,
                          Snap=self.SnapAngles)
        # Only the leading escape waypoints may leave a measured pose that lacks
        # the full tracking allowance. Once a stage passes the full check, every
        # later stage must too.
        Relief = World.StartRelief(Collision.Np.array([Current[Name] for Name in MOTOR_NAMES]))
        Result = []
        Check = self.SolverSnapshot()
        # The planner certified its snapped waypoints. If every one converts to
        # servo steps and back exactly, those certificates are for exactly this
        # route; otherwise every quantized stage is checked again.
        Exact = True
        for Angles in Path:
            Raw = {Name: Check._ik_raw_from_angle(Check.servos[Name], Angles[Name]) for Name in MOTOR_NAMES}
            # Preserve exact unchanged/returned coordinates, including a held
            # wrist outside GUI ranges. Other arm joints may temporarily move
            # away and return to the same final angle to clear the basket.
            for Name in MOTOR_NAMES:
                if Angles[Name] == Current[Name]:
                    Raw[Name] = self.servos[Name].present_raw
            Exact = Exact and all(self._ik_angle_from_raw(self.servos[Name], Raw[Name]) == Angles[Name] for Name in MOTOR_NAMES)
            Result.append(Raw)
        Result = Collision.RoutePath(Result, getattr(Path, "Tight", ()))
        if not Exact:
            self.CertifyRoutePath(Result, Relief)
        return self.AssignRouteReach(Result) if AssignReach else Result

    def CertifyRoutePath(self, Path, Relief=None):
        """Check every stage of a raw route (a RoutePath) from where the arm is, without waypoint reach.

        Only the leading stages may use the start pose's own clearance (Relief,
        World.StartRelief of where the arm is): once a stage passes the full
        check, every later stage must too, and a relieved stage's waypoints
        become tight. Raises ValueError with the failing stage's reason.

        With the check workers running (RouteCheckPool), every stage's full
        check is asked of them at once first; a stage they certified is not
        checked again, anything else is checked here as before.
        """
        Check = self.SolverSnapshot()
        Asked = self.PrefetchStageChecks(Path, Check)
        try:
            for Index, Point in enumerate(Path):
                try:
                    if not RouteCheckAnswered(Asked.get(Index), Certified=True):
                        Check.ValidateMotionPath(Point)
                    Relief = None
                except ValueError:
                    if Relief is None:
                        raise
                    Check.ValidateMotionPath(Point, Relief=Relief)
                    # The relief leaves no room for waypoint reach at either end.
                    Path.Tight.update(Waypoint for Waypoint in (Index - 1, Index) if Waypoint >= 0)
                for Name in MOTOR_NAMES:
                    Check.servos[Name].present_raw = Point[Name]
        finally:
            for Future in Asked.values():
                Future.cancel()
        return Path

    def PrefetchStageChecks(self, Path, Check) -> dict:
        """Each stage of a raw route checked in full (no relief, no reach), side by side in the check workers:
        {stage: future} (RouteCheckAnswered reads one), answered within ROUTE_REACH_ASSIGN_SECONDS; empty without
        the workers or for a route of fewer than three stages, where the round trip buys nothing."""
        Workers = RouteChecks.Running()
        if Workers is None or len(Path) < 3:
            return {}
        Geometry, Frame = Check.arm_geometry, Check.CollisionFrame()
        Deadline = time.monotonic() + ROUTE_REACH_ASSIGN_SECONDS
        Previous, Asked = {Name: Servo.present_raw for Name, Servo in Check.servos.items()}, {}
        for Index, Point in enumerate(Path):
            Asked[Index] = Workers.Submit((Geometry, Frame, *Check.EdgeCheckArguments(Previous, Point)), Deadline)
            Previous = Point
        return Asked

    def StartRelief(self):
        """World.StartRelief at where the arm is: None when the pose has the full tracking allowance."""
        return self.CollisionWorld().StartRelief(Collision.Np.array(
            [self._ik_angle_from_raw(self.servos[Name], self.servos[Name].present_raw) for Name in MOTOR_NAMES]))

    def StartHasAllowance(self) -> bool:
        """Whether the pose a route leaves from has the full tracking allowance: every stage from it is
        then certified that far around it on every joint (no relief), and the route monitor allows the same."""
        try:
            return self.StartRelief() is None
        except ValueError:
            return False

    def MovedDuringPlanning(self, StartRaw, FullAllowance) -> bool:
        """Whether the arm left the pose a route was planned from (StartRaw) while it planned. Called with the lock.

        Every arm joint must be within 2 steps. The gripper may creep up to
        GRIPPER_GOAL_TOLERANCE_RAW while it settles on its last goal or squeezes
        a grasp, when the planned start had the full tracking allowance (16
        steps, StartHasAllowance): 2026-10-03 14:46 the gripper, 3 steps short of
        an open it had just made, crept on during planning and refused a tip move.
        """
        for Name in MOTOR_NAMES:
            Drift = self._circular_raw_distance(StartRaw[Name], self.servos[Name].present_raw)
            if Drift > (GRIPPER_GOAL_TOLERANCE_RAW if Name == GRIPPER_NAME and FullAllowance else 2):
                return True
        return False

    def AssignRouteReach(self, Path):
        """Give waypoints the waypoint reach wherever the quantized route still certifies with it.

        Every stage of Path already passed without reach (or with the start
        relief, whose waypoints are tight), so this never rejects a route. A
        waypoint whose reach does not certify within the subdivision budget stays
        tight: the arm passes it within the plain allowance, slowly enough to
        stop there. When a stage does not certify even without its own reach,
        it is the reach at its start, the previous waypoint's, that does not fit:
        that waypoint becomes an arrive waypoint (its own approach keeps the
        reach it certified with; every joint arrives within the allowance before
        this stage starts, so this stage needs none at its start) and this
        waypoint's own reach is tried again. Both used to become tight: 2026-10-03,
        four of the five waypoints of the lift off the can beside the basket rim.

        Before the plain reach, each waypoint tries a wider one for the joints
        that carry on the same way after it (WideWaypointReach): their overshoot
        lies inside the next stage, so only this stage's box grows, forward along
        the leg. At 100% the braking room before each waypoint, not the ceiling,
        set the pace: with 32 steps the lift slowed from 400 to 200 steps/s at
        every one of the nine waypoints from Home to the hover over the can.
        That is tried at ROUTE_WIDE_REACH_MINIMUM_PERCENT and above, until it
        misses ROUTE_WIDE_REACH_MISSES times on the route.

        Where the plain reach does not certify, ROUTE_REDUCED_REACH_RAW is tried
        before the waypoint becomes tight (on 2026-10-03's descent to the can,
        two of its three tight waypoints took it).

        The route end tries an end reach (EndReach) for the joints that carry on
        into the last stage from a waypoint with more reach than their last
        travel: the route monitor then accepts in the last stage what it accepted
        in the stage before (2026-10-04 23:50:01 the switch between them tripped).
        Where it does not certify, the end keeps the plain allowance, as before.

        Long routes fall back to the cheap budget, and stop trying the wider
        reach, once ROUTE_REACH_ASSIGN_SECONDS is spent, which costs speed,
        never clearance.

        With the check workers running (RouteCheckPool, the server starts them),
        the checks this pass is likely to ask are certified ahead, side by side
        (PrefetchRouteReach), and it reads their answers instead of checking
        again. It still decides exactly as it would alone; only its time changes.
        """
        Check = self.SolverSnapshot()
        Deadline = time.monotonic() + ROUTE_REACH_ASSIGN_SECONDS
        Misses = 0 if self.MotionSpeedPercent >= ROUTE_WIDE_REACH_MINIMUM_PERCENT else ROUTE_WIDE_REACH_MISSES
        Plain = dict.fromkeys(MOTOR_NAMES, Collision.WaypointReachRaw)
        Reduced = dict.fromkeys(MOTOR_NAMES, ROUTE_REDUCED_REACH_RAW)
        Displaced, Previous = {}, {Name: Servo.present_raw for Name, Servo in self.servos.items()}
        Before = None
        Path.Reach = {}
        Workers = RouteChecks.Running()
        Known, Asked = {}, set()

        def Certifies(Point, Goal, Budget):
            Start = self.RouteReachRaw(Path.Tight, len(Path), Index, Displaced, Path.Arrive)[0]
            if not Goal and not any(Start.values()):
                return True  # Certified without reach when it was planned.
            Key = (Index, RouteReachKey(Start), RouteReachKey(Goal))
            if Key not in Known and Key[:2] not in Asked and Workers is not None and time.monotonic() < Deadline:
                # A waypoint before this came out otherwise than assumed: ask ahead again, from how it did,
                # and drop what was asked from the old assumption and has not started.
                for Stale, (_, Future) in Known.items():
                    if Stale[0] >= Index:
                        Future.cancel()
                Known.update(self.PrefetchRouteReach(Path, Check, Workers, Deadline, Index, Displaced, Previous,
                                                     Misses < ROUTE_WIDE_REACH_MISSES, Asked))
            if Key in Known:
                Found, Future = Known[Key]
                # A certificate holds whatever budget found it; a miss counts only at a budget at least this
                # one's (EdgeReason splits in a fixed order, so a larger budget only ever certifies more).
                if RouteCheckAnswered(Future, Deadline, Certified=True):
                    return True
                if Found >= Budget and RouteCheckAnswered(Future, Deadline, Certified=False):
                    return False
            try:
                Check.ValidateMotionPath(Point, Reach=(Start, Goal), Budget=Budget)
                return True
            except ValueError:
                return False

        for Index, Point in enumerate(Path):
            for Name in MOTOR_NAMES:
                Check.servos[Name].present_raw = Previous[Name]
            # Tight before any reach (a stage leaving with the start pose's own clearance): it stays tight.
            Fixed, Last = Index in Path.Tight, Index >= len(Path) - 1
            Wide = (self.WideWaypointReach(Previous, Point, Path[Index + 1])
                    if not Last and not Fixed and time.monotonic() < Deadline else None)
            # The route end takes what joints carrying on into the last stage may already have overshot.
            End = (self.EndReach(Before, Previous, Point, {} if Index - 1 in Path.Tight else Path.Reach.get(Index - 1) or Plain)
                   or None) if Last and not Fixed and Index > 0 else None
            while True:
                Chosen = None
                Tries = [End] if Last else [Wide if Misses < ROUTE_WIDE_REACH_MISSES else None, Plain, Reduced]
                for Goal in ([] if Fixed else Tries) + [{}]:
                    if Goal is None:
                        continue
                    Budget = (ROUTE_WIDE_REACH_BUDGET if Goal is Wide else
                              ROUTE_REACH_CHECK_BUDGET if time.monotonic() < Deadline else ROUTE_REACH_CHEAP_BUDGET)
                    if Certifies(Point, Goal, Budget):
                        Chosen = Goal
                        break
                    Misses += Goal is Wide
                if Chosen is not None:
                    break
                # Not even without its own reach: the reach at its start, the previous waypoint's, does not fit.
                # Arrived at, that waypoint needs none at this stage's start, and its own approach keeps the reach
                # it certified with. (Shrinking it to the reduced reach instead was slower on every measured route:
                # a fast approach and a short wait beat a slow approach flown past.)
                Path.Arrive.add(Index - 1)
                Displaced = {}
            if Chosen is Wide or Chosen is Reduced or (Chosen and Chosen is End):
                Path.Reach[Index] = Chosen
            elif not Chosen and not Last:
                Path.Tight.add(Index)
            Displaced = self.RouteDisplaced(Displaced, Previous, Point, Index in Path.Tight or Index in Path.Arrive,
                                            Path.Reach.get(Index))
            Before, Previous = Previous, Point
        return Path

    def EndReach(self, Before, Waypoint, End, Approach) -> dict[str, int]:
        """Goal reach past the route end for the joints carrying on into its last stage (Waypoint -> End, raw).

        The stage before accepts a joint that carries on through Waypoint (Before -> Waypoint -> End, the
        same way) up to its reach there (Approach, per joint: that waypoint's reach, empty when it is
        tight) plus the allowance beyond it. The last stage used to accept it only its own travel plus the
        allowance beyond Waypoint, so where the reach was the larger, a joint the stage before rightly let
        overshoot left the checked route the moment the last stage began: 2026-10-04 23:50:01 the pan 36
        steps past its last waypoint, 19 past the route end, with a reach of 32 against 17 steps of travel.
        Each such joint gets the difference past the route end, certified with the last stage like any
        waypoint reach (AssignRouteReach), so the last stage accepts what the stage before did. Braking
        still plans every stop at the end inside the plain allowance (RouteStopRoom). Empty when none.
        """
        Reach = {}
        for Name in MOTOR_NAMES:
            Servo = self.servos[Name]
            Here = self._slider_track_pos(Servo, Waypoint[Name])
            Into, On = Here - self._slider_track_pos(Servo, Before[Name]), self._slider_track_pos(Servo, End[Name]) - Here
            Steps = math.ceil(Approach.get(Name, 0) - abs(On)) if Into * On > 0 else 0
            if Steps > 0:
                Reach[Name] = Steps
        return Reach

    def PrefetchRouteReach(self, Path, Check, Workers, Deadline, From=0, Displaced=None, Previous=None, Wide=True,
                           Asked=None) -> dict:
        """AssignRouteReach's likely checks from waypoint From on, certified side by side in the check workers.

        Its pass is serial: a waypoint's start reach depends on how the waypoints
        before it came out. Most are flown past with a reach, so each later
        waypoint's start reach is taken as that leaves it, and its tries are
        asked in AssignRouteReach's order and budgets (wide when Wide, plain,
        reduced, none; at the route end its end reach, then none), every
        waypoint's first try ahead of any second one. A
        waypoint's later tries are dropped once an earlier one certifies, as the
        pass would never ask them. Displaced and Previous: the pass's state at
        From. Returns at once {(waypoint, start reach, goal reach): (budget,
        future)}, each future answering ("ok", reason or None) or None (no
        worker answered by Deadline); Asked collects the (waypoint, start reach)
        pairs sent. Check: AssignRouteReach's snapshot.
        """
        Plain = dict.fromkeys(MOTOR_NAMES, Collision.WaypointReachRaw)
        Reduced = dict.fromkeys(MOTOR_NAMES, ROUTE_REDUCED_REACH_RAW)
        Displaced = dict(Displaced or {})
        Previous = Previous or {Name: Servo.present_raw for Name, Servo in self.servos.items()}
        Geometry, Frame = Check.arm_geometry, Check.CollisionFrame()
        Before = Path[From - 2] if From >= 2 else {Name: Servo.present_raw for Name, Servo in self.servos.items()} if From == 1 else None
        Stages = []
        for Index in range(From, len(Path)):
            Point, Fixed, Last = Path[Index], Index in Path.Tight, Index >= len(Path) - 1
            Start = self.RouteReachRaw(Path.Tight, len(Path), Index, Displaced, Path.Arrive)[0]
            Wider = self.WideWaypointReach(Previous, Point, Path[Index + 1]) if Wide and not Last and not Fixed else None
            # The end reach as AssignRouteReach works it out, the waypoint before taken as flown past with its reach.
            End = (self.EndReach(Before, Previous, Point, {} if Index - 1 in Path.Tight else Path.Reach.get(Index - 1) or Plain)
                   or None) if Last and not Fixed and Index > 0 else None
            Tries = []
            for Goal in [Goal for Goal in ([] if Fixed else [End] if Last else [Wider, Plain, Reduced]) if Goal is not None] + [{}]:
                if not Goal and not any(Start.values()):
                    break  # Certified without reach when it was planned.
                Budget = ROUTE_WIDE_REACH_BUDGET if Goal is Wider else ROUTE_REACH_CHECK_BUDGET
                Tries.append(((Index, RouteReachKey(Start), RouteReachKey(Goal)), Budget,
                              Check.EdgeCheckArguments(Previous, Point, None, (Start, Goal), Budget)))
            if Asked is not None:
                Asked.add((Index, RouteReachKey(Start)))
            Stages.append(Tries)
            Displaced = self.RouteDisplaced(Displaced, Previous, Point, Fixed or Index in Path.Arrive)
            Before, Previous = Previous, Point
        Futures = [[None] * len(Tries) for Tries in Stages]

        def Settled(Stage, Rank, Future):
            if not Future.cancelled() and Future.result() is not None and Future.result()[1] is None:
                for Later in Futures[Stage][Rank + 1:]:
                    Later.cancel()

        Known = {}
        for Rank in range(max((len(Tries) for Tries in Stages), default=0)):
            for Stage, Tries in enumerate(Stages):
                if Rank < len(Tries):
                    Key, Budget, Arguments = Tries[Rank]
                    Futures[Stage][Rank] = Known[Key] = (Budget, Workers.Submit((Geometry, Frame, *Arguments), Deadline))
        for Stage, Row in enumerate(Futures):
            Futures[Stage] = [Future for _, Future in Row]
            for Rank, Future in enumerate(Futures[Stage]):
                Future.add_done_callback(lambda Done, Stage=Stage, Rank=Rank: Settled(Stage, Rank, Done))
        return Known

    def WideWaypointReach(self, Before, Point, After) -> dict[str, int] | None:
        """A wider goal reach for waypoint Point (raw) of a route going Before -> Point -> After, or None.

        A joint that carries on the same way after Point gets ROUTE_WIDE_REACH_FRACTION
        of its travel on this stage, no more than its travel on the next: its
        overshoot then stays inside the next stage's own range, whatever that
        stage's goal reach, and the next stage needs no more reach at its start
        (a joint trailing Point is within the fly-by tolerance). Every other
        joint, and the gripper, keeps WaypointReachRaw. Proportional to each
        joint's travel, so it grows the box along the leg, where the arm goes
        next anyway, as the joints' timed speeds need it.
        """
        Reach, Wider = {}, False
        for Name in MOTOR_NAMES:
            Servo = self.servos[Name]
            Here = self._slider_track_pos(Servo, Point[Name])
            Into, On = Here - self._slider_track_pos(Servo, Before[Name]), self._slider_track_pos(Servo, After[Name]) - Here
            Steps = Collision.WaypointReachRaw
            if Into * On > 0 and Name != GRIPPER_NAME:
                Steps = max(Steps, math.floor(min(abs(Into) * ROUTE_WIDE_REACH_FRACTION, abs(On))))
            Reach[Name] = Steps
            Wider = Wider or Steps > Collision.WaypointReachRaw
        return Reach if Wider else None

    def QuantizeCartesianPose(self, Angles, Requested, Pitch):
        """Servo targets for IK joint angles, and the angles they hold, within the CAD IK tolerances.

        Rounding each joint to its nearest servo step on its own can miss a
        tolerance the pose itself meets: 2026-09-28 22:49 a 0.236 deg solve
        rounded to 0.296 deg and was refused, while one step over on two joints
        held 0.063 cm / 0.120 deg. So when the nearest rounding misses, the
        neighbouring step of each IK joint (inside its arc) is tried, and the
        combination closest to the request within both tolerances wins. The
        nearest rounding is kept whenever it fits, so a target that planned
        before plans to the same servo target. Callers check collisions on the
        pose returned here, whichever it is.
        """
        Current = self._current_ik_angles()
        Nearest = {Name: self._ik_raw_from_angle(self.servos[Name], Angles[Name]) for Name in IK_JOINTS}

        def Measure(Targets):
            Quantized = {**Current, **{Name: self._ik_angle_from_raw(self.servos[Name], Raw) for Name, Raw in Targets.items()}}
            Tip = forward_tip_position(self.arm_geometry, Quantized)
            return (Quantized, math.dist(tuple(Tip[Axis] for Axis in ("x", "y", "z")), tuple(Requested[Axis] for Axis in ("x", "y", "z"))),
                    self._angle_delta_abs(wrist_tip_pitch_degrees(self.arm_geometry, Quantized), Pitch))

        Quantized, Error, PitchError = Measure(Nearest)
        if Error <= IK_SEND_MAX_ERROR_CM and PitchError <= IK_SEND_MAX_PITCH_ERROR_DEG:
            return Nearest, Quantized
        Best = None
        for Offsets in itertools.product((-1, 0, 1), repeat=len(IK_JOINTS)):
            Targets = {Name: (Nearest[Name] + Offset) % RESOLUTION for Name, Offset in zip(IK_JOINTS, Offsets)}
            if not any(Offsets) or not all(self._contains_raw(self.servos[Name], Targets[Name])
                                           for Name, Offset in zip(IK_JOINTS, Offsets) if Offset):
                continue
            Pose, NeighbourError, NeighbourPitchError = Measure(Targets)
            if NeighbourError > IK_SEND_MAX_ERROR_CM or NeighbourPitchError > IK_SEND_MAX_PITCH_ERROR_DEG:
                continue
            Score = NeighbourError / IK_SEND_MAX_ERROR_CM + NeighbourPitchError / IK_SEND_MAX_PITCH_ERROR_DEG
            if Best is None or Score < Best[0]:
                Best = (Score, Targets, Pose)
        if Best is not None:
            return Best[1], Best[2]
        if Error > IK_SEND_MAX_ERROR_CM:
            raise ValueError(f"Quantized servo target exceeds the CAD IK position tolerance ({Error:.3f} cm off "
                             f"at the nearest servo steps; limit {IK_SEND_MAX_ERROR_CM:g} cm)")
        raise ValueError(f"Quantized servo target exceeds the CAD IK pitch tolerance ({PitchError:.2f} deg off "
                         f"at the nearest servo steps; limit {IK_SEND_MAX_PITCH_ERROR_DEG:g} deg)")

    def BasketEntryRoute(self, Requested, Pitch, GoalTargets, Seconds=6.0, AssignReach=True):
        """Build a checked descent by lifting the final pose out of the tray.

        This computes only. The reverse of every independently certified edge
        is equally safe, even when the servos make different relative progress.
        AssignReach False leaves the waypoint reach to the caller.
        """
        World = self.CollisionWorld()
        Bounds = World.Bounds
        if not (Bounds[0, 0] <= Requested["x"] <= Bounds[1, 0]
                and Bounds[0, 1] <= Requested["y"] <= Bounds[1, 1]
                and Requested["z"] < Bounds[1, 2]):
            raise ValueError("Target does not need an entry through the basket rim")
        Deadline = time.monotonic() + Seconds
        Lift = self.SolverSnapshot()
        Goal = {Name: GoalTargets.get(Name, Servo.present_raw) for Name, Servo in Lift.servos.items()}
        for Name, Raw in Goal.items():
            Lift.servos[Name].present_raw = Raw
        Escape = [Goal]
        Height = Requested["z"]
        LastConnectionHeight = -math.inf
        # Start just above the rim, going higher only if approaching that pose
        # is still blocked. The final XYZ and wrist angle never change.
        while Height < Bounds[1, 2] + 12 and time.monotonic() < Deadline:
            Progress = False
            for Step in (2.0, 1.0, 0.5, 0.25):
                Raised = {**Requested, "z": min(Height + Step, Bounds[1, 2] + 12)}
                Current = Lift._current_ik_angles()
                Candidates = Cad.SolveCandidates(self.arm_geometry, Raised["x"], Raised["y"], Raised["z"],
                    elbow=Cad.Branch(self.arm_geometry, Current), joint_limits=self._ik_joint_limits(),
                    initial_angles=Current, preferred_angles=Current, wrist_pitch_deg=Pitch, max_iterations=30)
                Tested = set()
                for Candidate in Candidates:
                    if time.monotonic() >= Deadline:
                        break
                    if not Candidate["reachable"]:
                        continue
                    try:
                        Targets, _ = Lift.QuantizeCartesianPose(Candidate["joint_angles_degrees"], Raised, Pitch)
                        Key = tuple(Targets[Name] for Name in IK_JOINTS)
                        if Key in Tested:
                            continue
                        Tested.add(Key)
                        Lift.ValidateMotionPath(Targets)
                    except ValueError:
                        continue
                    Next = {**Escape[-1], **Targets}
                    Escape.append(Next)
                    for Name, Raw in Next.items():
                        Lift.servos[Name].present_raw = Raw
                    Height, Progress = Raised["z"], True
                    break
                if Progress:
                    break
            if not Progress:
                break
            # Start checking as soon as the tip clears the rim. A fixed tall
            # approach height can itself exceed IK reach near the mast; the
            # full arm/sweep certificate decides whether this approach is safe.
            if Height < Bounds[1, 2] + 0.25 or Height < LastConnectionHeight + 2:
                continue
            LastConnectionHeight = Height
            try:
                Approach = self.PlanMotionRoute(Escape[-1], AssignReach=False,
                                               Seconds=min(3.0, max(0.01, Deadline - time.monotonic())))
            except ValueError:
                continue
            Path = Approach + Escape[-2::-1]
            # Remove intermediate stops only after validating the exact raw
            # shortcut. This cannot reintroduce a quantization collision.
            Check, Result, Begin = self.SolverSnapshot(), [], 0
            ShortcutDeadline = min(Deadline, time.monotonic() + 0.35)
            while Begin < len(Path):
                if time.monotonic() >= ShortcutDeadline:
                    Result.extend(Path[Begin:])
                    break
                End = len(Path) - 1
                StartAngles = np.array([Check._ik_angle_from_raw(Check.servos[Name], Check.servos[Name].present_raw) for Name in MOTOR_NAMES])
                while End > Begin:
                    if time.monotonic() >= ShortcutDeadline:
                        End = Begin
                        break
                    try:
                        EndAngles = np.array([Check._ik_angle_from_raw(Check.servos[Name], Path[End][Name]) for Name in MOTOR_NAMES])
                        if World.EdgeReason(StartAngles, EndAngles, Budget=16):
                            End -= 1
                            continue
                        Check.ValidateMotionPath(Path[End])
                        break
                    except ValueError:
                        End -= 1
                # Adjacent edges were already certified during the approach
                # or reverse lift; no need to recalculate them on timeout.
                Result.append(Path[End])
                for Name in MOTOR_NAMES:
                    Check.servos[Name].present_raw = Path[End][Name]
                Begin = End + 1
            # The shortcuts above were checked without waypoint reach. Certify the exact route once more from
            # where the arm is (a leading stage that needs the start pose's own clearance leaves its waypoints
            # tight), then give each waypoint the reach wherever it certifies, as every other route gets: every
            # waypoint used to stay tight, approached slowly enough to stop inside the plain allowance (the
            # 2026-10-03 lift off the can stopped at four of five). A route that does not certify stays all tight.
            try:
                Certified = self.CertifyRoutePath(Collision.RoutePath(Result), self.StartRelief())
            except ValueError:
                return Collision.RoutePath(Result, range(len(Result)))
            return self.AssignRouteReach(Certified) if AssignReach else Certified
        raise ValueError("No checked entry over the basket rim was found at this wrist angle")

    def EndpointRefusal(self, Requested, Pitch, Candidates, Failures, World, SolveOptions) -> str:
        """Why no servo target passed PlanCartesianMotion's endpoint check, with what the app can correct.

        The bare "IK target is unreachable at this wrist angle" left the app
        nothing to do but stop the task (2026-09-29 07:58, 0.043 cm / 0.32 deg
        off). An IK or rounding refusal now says how far the closest pose misses
        and names a wrist pitch whose servo target passes this same check. A
        collision refusal is returned as it was.
        """
        if any(not Text.startswith("Quantized servo target") for Text in Failures):
            return Failures[-1]
        if Failures:
            Text = Failures[-1]
        else:
            Text = "IK target is unreachable at this wrist angle"
            if SolveOptions is None:
                return Text
            # Out of reach, SolveCandidates also offers the nearest tip at any pitch.
            Closest = min(Candidate["error_cm"] for Candidate in Candidates)
            if Closest > IK_SEND_MAX_ERROR_CM:
                return f"{Text}: the tip gets no closer than {Closest:.2f} cm to it"
            Best = min(Candidates, key=lambda Candidate: max(Candidate["error_cm"] / IK_SEND_MAX_ERROR_CM,
                                                             Candidate["wrist_pitch_error_deg"] / IK_SEND_MAX_PITCH_ERROR_DEG))
            Text += (f": the closest pose misses by {Best['error_cm']:.3f} cm and {Best['wrist_pitch_error_deg']:.2f} deg"
                     f" (limits {IK_SEND_MAX_ERROR_CM:g} cm and {IK_SEND_MAX_PITCH_ERROR_DEG:g} deg)")
        if SolveOptions is None:
            return Text
        Hint = self.NearestReachablePitch(Requested, Pitch, World, SolveOptions)
        return Text + (f"; wrist {Hint:g} deg reaches this point" if Hint is not None
                       else "; no reachable wrist angle was found near it")

    def NearestReachablePitch(self, Requested, Pitch, World, SolveOptions, Seconds=IK_PITCH_HINT_SECONDS):
        """The wrist pitch nearest Pitch whose servo target passes the endpoint check, or None.

        Steps outward in growing steps, bisects back toward the request for the
        edge, then stands IK_PITCH_HINT_MARGIN_DEG off it, never past the step
        that passed: at the edge, rounding and servo noise flip the verdict. The
        value returned is the one checked. Runs only on a refusal, off the motor
        lock, within Seconds.
        """
        Deadline = time.monotonic() + Seconds

        def Passes(Candidate):
            for Solution in Cad.SolveCandidates(self.arm_geometry, Requested["x"], Requested["y"], Requested["z"],
                                                wrist_pitch_deg=Candidate, **SolveOptions):
                if not Solution["reachable"]:
                    continue
                try:
                    _, Quantized = self.QuantizeCartesianPose(Solution["joint_angles_degrees"], Requested, Candidate)
                except ValueError:
                    continue
                if World.PoseReason(Quantized) is None:
                    return True
            return False

        Previous = 0.0
        for Step in (0.5, 1.0, 2.0, 3.0, 5.0, 8.0, 12.0, 20.0, 30.0, 45.0, 60.0, 90.0):
            for Sign in (1, -1):
                if time.monotonic() > Deadline:
                    return None
                if not Passes(Pitch + Sign * Step):
                    continue
                # Previous (or the request itself) failed on this side.
                Inside, Outside = Step, Previous
                for _ in range(5):
                    if time.monotonic() > Deadline:
                        break
                    Middle = (Inside + Outside) / 2
                    if Passes(Pitch + Sign * Middle):
                        Inside = Middle
                    else:
                        Outside = Middle
                for Offset in (min(Step, Inside + IK_PITCH_HINT_MARGIN_DEG), Step):
                    Choice = round(_normalize_deg(Pitch + Sign * Offset), 1)
                    if Passes(Choice):
                        return Choice
                return round(_normalize_deg(Pitch + Sign * Step), 3)
            Previous = Step
        return None

    def PlanCartesianMotion(self, Requested, Pitch, InitialAngles=None, Elbow=None, AssignReach=True):
        """A certified route to the tip at Requested with the wrist at Pitch. AssignReach False leaves the waypoint
        reach to the caller (a route the gripper opening is blended into is certified and assigned again)."""
        World = self.CollisionWorld()
        Reason = World.TargetReason(Requested)
        if Reason:
            raise ValueError(Reason)
        Current = self._current_ik_angles()
        Initial = {**Current, **(InitialAngles or {})}
        for Name in ("wrist_roll", "gripper"):
            Initial[Name] = Current[Name]
        if self.arm_geometry.get("model") == "SO101Cad":
            SolveOptions = dict(elbow=Elbow or self.elbow_mode, joint_limits=self._ik_joint_limits(), initial_angles=Initial,
                                preferred_angles=Initial if InitialAngles else Current, max_iterations=70)
            Candidates = Cad.SolveCandidates(self.arm_geometry, Requested["x"], Requested["y"], Requested["z"],
                                             wrist_pitch_deg=Pitch, **SolveOptions)
        else:
            SolveOptions = None
            Preview = self._solve_preview(Requested, Pitch, Elbow, initial_angles=Initial)
            Candidates = [{"reachable": Preview["reachable"], "joint_angles_degrees": Preview["angles"]}]
        Goals, Seen, Failures = [], set(), []
        for Candidate in Candidates:
            if not Candidate["reachable"]:
                continue
            try:
                Targets, Quantized = self.QuantizeCartesianPose(Candidate["joint_angles_degrees"], Requested, Pitch)
                Reason = World.PoseReason(Quantized)
                if Reason:
                    raise ValueError(Reason)
            except ValueError as Error:
                Failures.append(str(Error))
                continue
            Key = tuple(Targets[Name] for Name in IK_JOINTS)
            # Numerically distinct solutions a few encoder steps apart are the
            # same arm configuration; planning each again only splits the time.
            if Key not in Seen and not any(max(self._circular_raw_distance(Targets[Name], Other[Name]) for Name in IK_JOINTS) <= 8
                                           for Other in Goals):
                Seen.add(Key)
                Goals.append(Targets)
        if not Goals:
            raise ValueError(self.EndpointRefusal(Requested, Pitch, Candidates, Failures, World, SolveOptions))
        # A clear endpoint says nothing about the route. Try the straight gated
        # line and small repairs for every feasible endpoint and keep the fastest,
        # then tray entry, then joint-space detours. One failing preferred
        # configuration cannot veto the others.
        Start = {Name: Servo.present_raw for Name, Servo in self.servos.items()}
        Routes = []
        RepairDeadline = time.monotonic() + 2.5
        for Index, Targets in enumerate(Goals):
            try:
                Path = self.PlanMotionRoute(Targets, LocalOnly=True, AssignReach=False,
                    Seconds=max(.05, (RepairDeadline - time.monotonic()) / (len(Goals) - Index)))
            except ValueError as Error:
                Failures.append(str(Error))
                continue
            Routes.append((self.RouteDuration(Start, Path), Index, Path))
        if Routes:
            Best = min(Routes, key=lambda Item: Item[:2])[2]
            return self.AssignRouteReach(Best) if AssignReach else Best
        EntryDeadline = time.monotonic() + 6.0
        for Index, Targets in enumerate(Goals):
            if self.arm_geometry.get("model") != "SO101Cad":
                break
            try:
                return self.BasketEntryRoute(Requested, Pitch, Targets, AssignReach=AssignReach,
                    Seconds=max(0.01, (EntryDeadline - time.monotonic()) / (len(Goals) - Index)))
            except ValueError:
                pass
        # An even split gave every configuration too little to find a detour.
        # The first candidate is the preferred configuration, so it takes most
        # of the clock and the rest share what it leaves.
        Deadline = time.monotonic() + ROUTE_PLAN_SECONDS
        for Index, Targets in enumerate(Goals):
            Remaining = Deadline - time.monotonic()
            Share = Remaining if Index == len(Goals) - 1 else Remaining * ROUTE_PLAN_FIRST_SHARE
            try:
                return self.PlanMotionRoute(Targets, Seconds=max(0.01, Share), AssignReach=AssignReach)
            except ValueError as Error:
                Failures.append(str(Error))
        # By here at least one goal pose was clear, so the target itself is fine
        # and the blocker is the way there. Say which: a failure about the arm's
        # own pose is the one the operator can act on, and it was previously
        # buried behind wording that sounded like the target was unreachable.
        Blocked = next((Text for Text in Failures if Text.startswith("Current arm pose")), None)
        if Blocked:
            raise ValueError(Blocked)
        raise ValueError("This target is reachable, but no checked route gets there from where the arm is now. "
                         "Home the arm, or move it clear, and try again: " + Failures[-1])

    def GripperWithTipTarget(self, Raw) -> int | None:
        """The gripper goal a tip move carries (raw, clamped to the gripper's limits as a slider move is), or None
        when it is within the gripper's goal tolerance of where the jaws are.

        Only an opening: a gripper closing while the arm moves could stop on
        something and trip the stall protection, which coasts the whole arm
        (GraspClosing holds a stopped close only when nothing else moves), and
        the app closes before a lift, never during it. Called with the lock.
        """
        if isinstance(Raw, bool) or not isinstance(Raw, (int, float)) or not math.isfinite(Raw):
            raise ValueError("The gripper target sent with a tip move must be a raw servo position")
        Servo = self.servos[GRIPPER_NAME]
        Goal = self._clamp_raw(Servo, round(Raw))
        Opening = self._slider_track_pos(Servo, Goal) - self._slider_track_pos(Servo, Servo.present_raw)
        if Opening < -GRIPPER_GOAL_TOLERANCE_RAW:
            raise ValueError("A tip move may only open the gripper on the way; close or narrow it as its own motion")
        return Goal if Opening > GRIPPER_GOAL_TOLERANCE_RAW else None

    def GripperAtSnapshot(self, Raw):
        """A solver snapshot with the jaws already at Raw: the arm's route is planned and its end pose checked
        with the gripper as an opening leaves it, its widest on the way."""
        Snapshot = self.SolverSnapshot()
        Servo = Snapshot.servos[GRIPPER_NAME]
        Servo.present_raw = Servo.target_raw = Raw
        return Snapshot

    def RawAlongTrack(self, Name, FromRaw, ToRaw, Fraction) -> int:
        """The raw position Fraction of the way from FromRaw to ToRaw along a servo's slider track (exact at the ends)."""
        if Fraction <= 0:
            return FromRaw
        if Fraction >= 1:
            return ToRaw
        Servo = self.servos[Name]
        Start = self._slider_track_pos(Servo, FromRaw)
        Step = round(Start + Fraction * (self._slider_track_pos(Servo, ToRaw) - Start))
        return (Servo.range_min - Step if Servo.arc_inverted else Servo.range_min + Step) % RESOLUTION

    def BlendGripperIntoRoute(self, Path, Goal):
        """An arm route planned with the jaws already at Goal (GripperAtSnapshot), opening them on the way.

        The gripper leaves where it is with the arm and reaches Goal
        GRIPPER_WITH_TIP_OPEN_BY of the way through the arm's route time (each
        leg timed by its slowest arm joint at cruise, as MotionRouteCost does),
        or later when the gripper's own speed ceiling needs longer, at the latest
        on arrival. A waypoint on the arm's own line marks where it gets there;
        the gripper only widens on the way, between its start and Goal, so it
        stays inside its limits. Every stage is certified again with the jaws as
        they move on it: each stage's collision box covers the moving jaw over
        its whole swing on that stage. Should that not certify, the route opens
        the gripper where the arm is and then flies the arm's route (the old
        chain, as one motion, with nothing to settle in between), when the
        opened pose has the full tracking allowance. Raises when neither
        certifies. Either way the waypoint reach is assigned here, so Path
        need not have it (PlanCartesianMotion with AssignReach False).
        """
        Start = {Name: Servo.present_raw for Name, Servo in self.servos.items()}
        Arm = [Name for Name in MOTOR_NAMES if Name != GRIPPER_NAME]
        Ceilings = self.MotionSpeedCeilings(self.MotionSpeedPercent)
        Track = lambda Name, Raw: self._slider_track_pos(self.servos[Name], Raw)
        Points, Times = [Start, *Path], [0.0]
        for Before, After in zip(Points, Points[1:]):
            Times.append(Times[-1] + max(abs(Track(Name, After[Name]) - Track(Name, Before[Name])) / max(1.0, Ceilings[Name])
                                         for Name in Arm))
        Opening = abs(Track(GRIPPER_NAME, Goal) - Track(GRIPPER_NAME, Start[GRIPPER_NAME])) / max(1.0, Ceilings[GRIPPER_NAME])
        OpenBy = min(Times[-1], max(GRIPPER_WITH_TIP_OPEN_BY * Times[-1], Opening))
        Blended, Tight = [], set()
        for Index, Point in enumerate(Path):
            Before, After = Times[Index], Times[Index + 1]
            if Before < OpenBy < After:
                Previous = Points[Index]
                Middle = {Name: self.RawAlongTrack(Name, Previous[Name], Point[Name], (OpenBy - Before) / (After - Before)) for Name in Arm}
                # Only where the arm is still well short of both ends of the leg; else the jaws reach Goal at its end.
                if all(max(abs(Track(Name, Middle[Name]) - Track(Name, End[Name])) for Name in Arm) > ROUTE_FLY_BY_TOLERANCE_RAW
                       for End in (Previous, Point)):
                    Blended.append({**Middle, GRIPPER_NAME: Goal})
            Share = 1.0 if OpenBy <= 0 else After / OpenBy
            Blended.append({**Point, GRIPPER_NAME: self.RawAlongTrack(GRIPPER_NAME, Start[GRIPPER_NAME], Goal, Share)})
            if Index in getattr(Path, "Tight", ()):
                Tight.add(len(Blended) - 1)
        Blended = Collision.RoutePath(Blended, Tight)
        Relief = self.StartRelief()
        try:
            return self.AssignRouteReach(self.CertifyRoutePath(Blended, Relief))
        except ValueError as Error:
            Blocked = Error
        # The arm's route leaves the opened pose as it was planned only with the full tracking
        # allowance there: a relieved start is certified for an exactly measured pose alone.
        if not self.GripperAtSnapshot(Goal).StartHasAllowance():
            raise Blocked
        Opened = {**Start, GRIPPER_NAME: Goal}
        self.CertifyRoutePath(Collision.RoutePath([Opened]), Relief)
        # The arm's route was certified from exactly that pose, and the gripper opens fully (a tight
        # waypoint) before the arm leaves it. Its waypoints get their reach from there.
        Path = self.GripperAtSnapshot(Goal).AssignRouteReach(Collision.RoutePath(
            Path, getattr(Path, "Tight", ()), getattr(Path, "Arrive", ()), getattr(Path, "Reach", {})))
        return Collision.RoutePath([Opened, *Path], {0, *(Index + 1 for Index in getattr(Path, "Tight", ()))},
                                   {Index + 1 for Index in getattr(Path, "Arrive", ())},
                                   {Index + 1: Reach for Index, Reach in getattr(Path, "Reach", {}).items()})

    def PlanHomeRoute(self, Targets, Gripper):
        """Home's route to Targets (every joint's saved value, limited), the gripper as HomeGripperPlan decided (Gripper),
        certified like any route. On a planning snapshot, off the lock (in preview on the controller itself).

        The arm joints and the wrist roll ride one route. An opening rides it too, as a tip move's does
        (BlendGripperIntoRoute). A close waits for the arm: the route ends with a stage that moves only the
        gripper, from the arm at Home, after an arrive waypoint there (every joint within the tight tolerance
        of Home before it starts). A jaw stopped by something on that stage becomes a grasp hold (GraspClosing:
        nothing else moves from it on) instead of the stall trip that coasted the arm when Home closed the jaws
        on the way, and the arm drags nothing it stopped on. The arm's route is planned with the jaws where
        they are, wider than where they close to, and the close is certified from Home. Held, or within the
        goal tolerance, the jaws stay where they are. Stop, Hold, Coast and the emergency stop end the route,
        the close with it. Only the gripper to move: a gripper move, as its slider makes it.
        """
        Kind, Goal = Gripper
        Arm = {Name: Raw for Name, Raw in Targets.items() if Name != GRIPPER_NAME}
        if Kind in ("open", "close") and all(Raw == self.servos[Name].present_raw for Name, Raw in Arm.items()):
            return self.PlanMotionRoute({**Arm, GRIPPER_NAME: Goal})
        if Kind == "open":
            return self.BlendGripperIntoRoute(self.GripperAtSnapshot(Goal).PlanMotionRoute(Arm, AssignReach=False), Goal)
        if Kind != "close":
            return self.PlanMotionRoute(Arm)
        Path = self.PlanMotionRoute(Arm, AssignReach=False)
        Arrived = Path[-1]
        Closed = {**Arrived, GRIPPER_NAME: Goal}
        Check = self.SolverSnapshot()
        for Name in MOTOR_NAMES:
            Check.servos[Name].present_raw = Arrived[Name]
        try:
            Check.ValidateMotionPath(Closed)
        except ValueError as Error:
            raise ValueError(f"Home: the gripper cannot close at Home: {Error}. No motion sent.") from None
        # Certified without reach, and the stage after an arrive waypoint starts with none: AssignRouteReach
        # gives the arm's approach to Home its reach and keeps the close as certified.
        return self.AssignRouteReach(Collision.RoutePath([*Path, Closed], getattr(Path, "Tight", ()),
                                                         {*getattr(Path, "Arrive", ()), len(Path) - 1}))

    def StartMotionRoute(self, Path, Calibration=False, PlannedStart=None, PlanSeconds=None, Note=None):
        """Called with the motor lock after fresh feedback and staleness checks. PlanSeconds: how long the route took
        to plan, from the command to here (RecordRouteTiming). Note: added to the route's status (Home's held gripper)."""
        self.MotionEpoch += 1
        Start = PlannedStart or {Name: Servo.present_raw for Name, Servo in self.servos.items()}
        Previous, Travel = Start, 0.0
        for Point in Path:
            Travel += max(abs(self._slider_track_pos(self.servos[N], Point[N])
                              - self._slider_track_pos(self.servos[N], Previous[N])) for N in MOTOR_NAMES)
            Previous = Point
        SpeedScale = self.MotionSpeedPercent / 100
        self.MotionStartedAt = time.monotonic()
        # The route end takes a settling approach; tight waypoints are passed
        # slowly, the arm arrives at arrive waypoints before going on, and other
        # waypoints are flown past, each with a small allowance.
        Tight = set(getattr(Path, "Tight", ())) - {len(Path) - 1}
        Arrive = set(getattr(Path, "Arrive", ())) - Tight - {len(Path) - 1}
        self.MotionTimeoutMs = math.ceil(max(12000, Travel / HOME_VELOCITY_MAX_SPEED * 1500 + 2500
                                             + len(Tight | Arrive) * 1000 + len(Path) * 250 + 5000) / SpeedScale)
        self.MotionRoute = {"Path": list(Path), "Tight": Tight, "Arrive": Arrive, "Reach": dict(getattr(Path, "Reach", {})),
                            "Displaced": {}, "Index": 0, "Start": Start, "Origin": dict(Start), "PlanSeconds": PlanSeconds,
                            "Epoch": self.MotionEpoch, "Configuration": self.ConfigurationKey(), "Calibration": Calibration,
                            "Note": Note}
        self.BeginRouteStage()

    def RouteWaypointArrives(self, Index) -> bool:
        """Every joint must be at route waypoint Index (within the tight tolerance) before the stage after it starts:
        a tight or arrive waypoint, whose next stage has no waypoint reach at its start."""
        Route = self.MotionRoute
        return Index in Route["Tight"] or Index in Route.get("Arrive", ())

    def BeginRouteStage(self):
        Route = self.MotionRoute
        Point = Route["Path"][Route["Index"]]
        # The offline certificate includes tracking slack around both endpoints.
        # Keep its original start; never widen the certificate by accumulating
        # measured settling errors across stages.
        if Route["Index"]:
            Reached = Route["Path"][Route["Index"] - 1]
            Route["Displaced"] = self.RouteDisplaced(Route["Displaced"], Route["Start"], Reached,
                                                     self.RouteWaypointArrives(Route["Index"] - 1),
                                                     Route.get("Reach", {}).get(Route["Index"] - 1))
            Route["Start"] = Reached
        self.CheckMotionRoute()
        if self.MotionRoute is None:
            if self.MotionFault is None and self.ProtectionTrip is None:
                raise RouteHeld(self.status)  # stopped holding (CheckMotionRoute said why): no fault to latch
            raise ValueError(self.MotionFault or "Arm left the checked route")
        Distance = max(abs(self._slider_track_pos(self.servos[N], Point[N])
                           - self._slider_track_pos(self.servos[N], Route["Start"][N])) for N in MOTOR_NAMES)
        self.MotionDeadline = time.monotonic() + max(12, Distance / HOME_VELOCITY_MAX_SPEED * 1.5 + 5) * 100 / self.MotionSpeedPercent
        RouteSpeeds = self.RouteSpeeds()
        for Name, Servo in self.servos.items():
            Servo.homing = True
            Servo.Holding = True
            if Servo.GraspHold and Point[Name] == Route["Start"][Name]:
                # Still holding what it grasped, and this stage leaves the gripper there: keep
                # squeezing (a new goal would let go), re-based on the planned value so the
                # squeeze stays inside this stage's tracking allowance. It still rejoins the
                # route like any parked joint if it strays past a waypoint (the object slipped).
                self.SqueezeGrasp(Servo, Point[Name])
                Servo.homing = True
            elif Point[Name] != Servo.target_raw or not self._slider_target_reached(Servo):
                self._send_slider_target_locked(Servo, Point[Name], num_retry=1, route_speeds=RouteSpeeds)
        self.status = f"Moving on basket-safe route, stage {Route['Index'] + 1}/{len(Route['Path'])}" + RouteNote(Route)

    def CheckMotionRoute(self):
        Route = self.MotionRoute
        if Route is None:
            return
        # A changed or expired scene, or a changed configuration, does not make the arm unsafe to hold: the route
        # stops and the arm holds where it is (HoldRoute). Both used to latch a fault and coast every joint.
        Scene = self.ActiveScene
        if Scene and not SceneSafety.IsCurrent(Scene["id"]):
            self.HoldRoute("the measured scene changed or expired during the route; capture and approve a new plan",
                           Details={"scene_id": Scene["id"], "scene_revision": Scene.get("revision")})
            return
        if Route["Epoch"] != self.MotionEpoch or Route["Configuration"] != self.ConfigurationKey():
            self.HoldRoute("the arm's configuration (Home, joint limits, CAD alignment, workspace frame or collision "
                           "model) changed during the route",
                           Details={"route_epoch": Route["Epoch"], "motion_epoch": self.MotionEpoch})
            return
        Point = Route["Path"][Route["Index"]]
        Allowance = Collision.Model["JointTrackingAllowanceRaw"]
        # The same region the bridge certified: the stage box, extended by the
        # waypoint reach at ends the arm flies past, plus the tracking allowance.
        # (A relieved escape stage was certified with less at its start faces;
        # tripping there would coast the arm beside the obstacle it is leaving.)
        Reach, GoalReaches = self.RouteReachRaw(Route["Tight"], len(Route["Path"]), Route["Index"], Route["Displaced"],
                                                Route.get("Arrive", ()), Route.get("Reach"))
        for Name, Servo in self.servos.items():
            Origin, Target = self._slider_track_pos(Servo, Route["Start"][Name]), self._slider_track_pos(Servo, Point[Name])
            StartReach, GoalReach = Reach[Name], GoalReaches.get(Name, 0)
            Low, High = ((Origin - StartReach, Target + GoalReach) if Target > Origin
                         else (Target - GoalReach, Origin + StartReach) if Target < Origin
                         else (Origin - StartReach, Origin + StartReach))
            Present = self._slider_track_present(Servo)[0]
            if not Servo.FeedbackOk or Present < Low - Allowance or Present > High + Allowance:
                self.LatchMotionFault(f"{Name} left the checked basket route", Joint=Name,
                                      Details={"feedback_ok": Servo.FeedbackOk, "track_raw": Present, "allowed_track_raw":
                                               [Low - Allowance, High + Allowance], "stage_start_raw": Route["Start"][Name],
                                               "stage_goal_raw": Point[Name]})
                return

    def RouteStageSettles(self) -> bool:
        """The current waypoint is the route end, where the arm settles instead of passing."""
        Route = self.MotionRoute
        return Route["Index"] >= len(Route["Path"]) - 1

    @staticmethod
    def RouteTightToleranceRaw() -> int:
        """How close every joint must be to a tight waypoint before the next stage starts.

        The next stage has only the allowance there, and the last joint to arrive
        is usually still this far short of the waypoint when the stage switches, so
        this keeps half the allowance in hand. Logged hardware runs at 12 steps
        came within 4 steps of the monitor bound.
        """
        return Collision.Model["JointTrackingAllowanceRaw"] // 2

    def RouteWaypointPassed(self) -> bool:
        """Every joint is close enough to the current waypoint for the next stage to start without stopping.

        At a waypoint the arm flies past, a joint may be within the fly-by
        tolerance or already beyond it on its leg. Either way it lies inside the
        next stage's checked region, which extends the waypoint reach around this
        waypoint for every joint that moved into it (plus the tracking allowance).
        A joint that does not move on this leg stays displaced, or not, on the next
        leg, so it is inside that region wherever the route monitor allows it to be.
        At a tight (or arrive) waypoint the next stage has only the allowance, so
        every joint, moving or not, must be within the tight tolerance.
        """
        Route = self.MotionRoute
        Point = Route["Path"][Route["Index"]]
        Tight = self.RouteWaypointArrives(Route["Index"])
        for Name, Servo in self.servos.items():
            Target = self._slider_track_pos(Servo, Point[Name])
            Error = Target - self._slider_track_present(Servo)[0]
            Travel = Target - self._slider_track_pos(Servo, Route["Start"][Name])
            if Tight:
                # Past the waypoint and carrying on the same way: the next stage's
                # box lies ahead of it, so it need not come back.
                if abs(Error) > self.RouteTightToleranceRaw() and not (Error * Travel < 0 and self.RouteJointContinues(Name)):
                    return False
            elif abs(Error) > ROUTE_FLY_BY_TOLERANCE_RAW and Error * Travel > 0:
                return False
        return True

    def RouteJointContinues(self, Name) -> bool:
        """Whether a joint keeps moving the same way from the current waypoint into the next stage."""
        Route = self.MotionRoute
        Path, Index, Servo = Route["Path"], Route["Index"], self.servos[Name]
        if Index + 1 >= len(Path):
            return False
        Here = self._slider_track_pos(Servo, Path[Index][Name])
        return (self._slider_track_pos(Servo, Path[Index + 1][Name]) - Here) * (Here - self._slider_track_pos(Servo, Route["Start"][Name])) > 0

    @staticmethod
    def RouteBrakingSpeed(Room: float, Overshoot: float = ROUTE_BRAKE_OVERSHOOT) -> float:
        """Highest speed (encoder steps/s) from which a joint can still stop within Room steps.

        Conservative for these servos under load: 0.18 s from feedback to a
        changed command taking effect, 1000 steps/s^2 of braking, and 20% speed
        overshoot by default (see ROUTE_BRAKE_LATENCY_S).
        """
        if Room <= 0:
            return 0.0
        Latency, Braking = ROUTE_BRAKE_LATENCY_S, ROUTE_BRAKE_ACCELERATION
        return Braking * (math.sqrt(Latency * Latency + 2 * Room / Braking) - Latency) / Overshoot

    def RouteStopRoom(self, Index, Name) -> tuple[float, float]:
        """Checked room (encoder steps) beyond route waypoint Index for joint Name if it has to stop there, and the
        speed overshoot its approach assumes: the route end and tight waypoints have only the tracking allowance,
        others the reach that waypoint's stage was certified with for that joint (an arrive waypoint's included)."""
        Route = self.MotionRoute
        Allowance = Collision.Model["JointTrackingAllowanceRaw"] - ROUTE_BRAKE_GUARD_RAW
        if Index >= len(Route["Path"]) - 1:
            return Allowance, ROUTE_BRAKE_OVERSHOOT
        if Index in Route["Tight"]:
            return Allowance, ROUTE_TIGHT_BRAKE_OVERSHOOT
        Reach = Route.get("Reach", {}).get(Index, {}).get(Name, Collision.WaypointReachRaw)
        # A reduced reach is approached as a tight waypoint is: it has little room for a lurch.
        return Reach + Allowance, ROUTE_TIGHT_BRAKE_OVERSHOOT if Reach < Collision.WaypointReachRaw else ROUTE_BRAKE_OVERSHOOT

    def RouteSpeeds(self) -> dict[str, int]:
        """Signed track speeds for the active route leg, timed so every moving joint arrives together.

        Each joint may go no faster than its ceiling at the throttle, nor faster
        than it could still stop inside its checked region: beyond the current
        waypoint (its reach plus the allowance, only the allowance at a tight
        waypoint or the route end), and beyond the waypoint where it must stop
        at the latest (its own direction reverses, or the route ends), where it
        also ramps down over ROUTE_SLOW_ZONE_RAW. The joint with the longest time
        to go at those limits runs at its limit and the others are slowed to
        match, so they arrive together instead of each braking on its own and
        waiting at the waypoint for the slowest. Joints cruise past waypoints.

        Real servos still drift apart, so a joint that gets ahead of the slowest
        joint's progress is eased off (a leash); a joint still finishing the last
        waypoint counts as at the start of this stage, not behind it. A joint
        that carries on the same way after this waypoint keeps pace with the
        others through the fly-by tolerance instead of stopping in it; once past
        the waypoint (or inside the tolerance and about to turn) it waits for the
        others (absent from the result). At a tight or arrive waypoint every
        joint waits within the tight tolerance, or past it if it carries on.
        """
        Route = self.MotionRoute
        Path, Index = Route["Path"], Route["Index"]
        Settle = self.RouteStageSettles()
        Tight = self.RouteWaypointArrives(Index)
        Percent = self.ActiveMotionSpeedPercent() / 100
        Ceilings = self.MotionSpeedCeilings(Percent * 100)
        Errors, Limits, Floors, Spans, Progress = {}, {}, {}, {}, {}
        for Name, Servo in self.servos.items():
            Target = self._slider_track_pos(Servo, Path[Index][Name])
            Origin = self._slider_track_pos(Servo, Route["Start"][Name])
            Present = self._slider_track_present(Servo)[0]
            Error, Spans[Name] = Target - Present, Target - Origin
            if abs(Spans[Name]) >= ROUTE_LEASH_MINIMUM_SPAN_RAW:
                Progress[Name] = max(0.0, (Present - Origin) / Spans[Name])
            Floor = SLIDER_VELOCITY_MIN_SPEED * Percent
            if Settle:
                if abs(Error) <= self._slider_goal_tolerance_raw(Name):
                    continue
            elif Tight:
                if abs(Error) <= self.RouteTightToleranceRaw() or (Error * Spans[Name] < 0 and self.RouteJointContinues(Name)):
                    continue
            elif Error * Spans[Name] < 0:
                continue
            elif abs(Error) <= ROUTE_FLY_BY_TOLERANCE_RAW:
                if not self.RouteJointContinues(Name):
                    continue
                Floor = 0.0
            # Where this joint must stop at the latest: its next reversal, or the route end.
            Remaining, Stop = abs(Error), Index
            if not Settle:
                for Next in range(Index + 1, len(Path)):
                    Step = self._slider_track_pos(Servo, Path[Next][Name]) - self._slider_track_pos(Servo, Path[Next - 1][Name])
                    if Step * Error <= 0:
                        break
                    Remaining, Stop = Remaining + abs(Step), Next
            Room, Overshoot = self.RouteStopRoom(Index, Name)
            StopRoom, StopOvershoot = self.RouteStopRoom(Stop, Name)
            Errors[Name], Floors[Name] = Error, Floor
            Ramp = max(SLIDER_VELOCITY_MIN_SPEED * Percent, Ceilings[Name] * Remaining / ROUTE_SLOW_ZONE_RAW)
            Limits[Name] = max(1e-6, min(Ceilings[Name], Ramp, self.RouteBrakingSpeed(abs(Error) + Room, Overshoot),
                                         self.RouteBrakingSpeed(Remaining + StopRoom, StopOvershoot)))
        Longest = max((abs(Error) / Limits[Name] for Name, Error in Errors.items()), default=0.0)
        # Joints already at or past the waypoint count as arrived; only joints
        # still travelling can hold the others back.
        Laggard = min((Value for Name, Value in Progress.items() if Name in Errors), default=1.0)
        Speeds = {}
        for Name, Error in Errors.items():
            Speed = min(Limits[Name], max(Floors[Name], abs(Error) / Longest if Longest > 0 else 0.0))
            if Name in Progress and not Settle:
                Lead = (Progress[Name] - Laggard) * abs(Spans[Name])
                Speed *= min(1.0, max(0.0, 1 - (Lead - ROUTE_LEASH_SLACK_RAW) / (Collision.WaypointReachRaw - ROUTE_LEASH_SLACK_RAW)))
            if Settle:
                Speed = max(1.0, Speed)
            if Speed >= 1:
                Speeds[Name] = (1 if Error > 0 else -1) * max(1, min(round(Speed), math.floor(Limits[Name])))
        return Speeds

    def AdvanceMotionRoute(self):
        Route = self.MotionRoute
        if Route is None:
            return
        if not self.RouteStageSettles():
            if not self.RouteWaypointPassed():
                return
        elif (any(S.command_raw is not None for S in self.servos.values())
                or not all(self._slider_target_reached(S) for S in self.servos.values())):
            return
        Route["Index"] += 1
        if Route["Index"] >= len(Route["Path"]):
            self.RecordRouteTiming(Route)
            self.MotionRoute = None
            self.ClearRouteScene()
            self.CalibrationActive = False
            self.MotionDeadline = 0
            for Servo in self.servos.values():
                Servo.homing = False
            self.status = "Basket-safe motion complete" + RouteNote(Route)
            return
        try:
            self.BeginRouteStage()
        except RouteHeld:
            return  # stopped holding where it is; the status says why
        except Exception as Exc:
            self.RaiseIfSerialLinkLost(Exc)
            self.StopMotion(f"Basket route stopped: {Exc}", Latch=True)

    def _read_positions_locked(self) -> bool:
        """Read every joint's position; True when all six are fresh.

        Missing feedback latches a fault while holding, except when the serial port itself is
        lost: then it returns False with every joint marked stale, and the link supervisor
        (ServiceSerialLink) decides instead of one "Lost joint feedback" per frame.
        """
        if not self.arm_connected or self.bus is None:
            return False
        if self.SerialLinkLost():
            for Servo in self.servos.values():
                Servo.FeedbackOk = False
            return False
        ok_reads = 0
        for name in MOTOR_NAMES:
            self.servos[name].FeedbackOk = False
            try:
                # One immediate retry: a single dropped status packet (seen when a heavy
                # request briefly starves this loop) must not coast the whole arm. A servo
                # that is really gone fails both attempts and still trips protection below.
                raw = int(self.bus.read("Present_Position", name, normalize=False, num_retry=1))
            except Exception:
                if self.SerialLinkLost():
                    for Servo in self.servos.values():
                        Servo.FeedbackOk = False
                    return False
                continue
            servo = self.servos[name]
            # Store the position the motor actually reports. Clamping feedback
            # to the GUI limits made the bridge believe a hand-posed arm was
            # already sitting on the boundary, so Home saw zero error and never
            # moved. Limits belong on targets, not on measurements — the track
            # helpers already cope with an out-of-range present position.
            servo.present_raw = int(raw) % RESOLUTION
            servo.HardwareRaw = int(raw)
            servo.FeedbackOk = True
            ok_reads += 1
        if ok_reads != len(MOTOR_NAMES) and self.torque_enabled and not self.MotionFault:
            Missing = [Name for Name in MOTOR_NAMES if not self.servos[Name].FeedbackOk]
            self.LatchMotionFault("Lost joint feedback: " + ", ".join(Missing), Joint=Missing[0],
                                  Details={"missing": Missing})
        elif ok_reads and not self.MotionFault and self.MotionRoute is None:
            self.status = HOLD_PAUSED_STATUS if self.HoldCorrectionPaused else self.LiveStatus()
        return ok_reads == len(MOTOR_NAMES)

    def _stop_slider_velocity_locked(
        self,
        servo: ServoRuntime,
        *,
        hold_target: bool,
        num_retry: int = 0,
    ) -> None:
        if self.bus is None:
            return
        hold_raw = servo.target_raw if hold_target else servo.present_raw
        Hardware = servo.HardwareRaw if servo.HardwareRaw is not None else servo.present_raw
        # Position mode supports multiple turns. Preserve the encoder turn count
        # when holding; modulo feedback is only for the GUI and slider arcs.
        hold_raw = Hardware + ((hold_raw - servo.present_raw + RESOLUTION // 2) % RESOLUTION - RESOLUTION // 2)
        PreviousMode = self.bus.read("Operating_Mode", servo.name, normalize=False, num_retry=num_retry)
        if PreviousMode == OperatingMode.VELOCITY.value:
            self.bus.write("Goal_Speed", servo.name, 0, normalize=False, num_retry=num_retry)
        self.bus.write("Goal_Position", servo.name, hold_raw, normalize=False, num_retry=num_retry)
        self.bus.write("Goal_Time", servo.name, 0, normalize=False, num_retry=num_retry)
        self.bus.write(
            "Operating_Mode",
            servo.name,
            OperatingMode.POSITION.value,
            normalize=False,
            num_retry=num_retry,
        )
        self.bus.write("Goal_Position", servo.name, hold_raw, normalize=False, num_retry=num_retry)
        # Verify the mode and goal before allowing a nonzero velocity command
        # to become a position profile. Zero in position mode is unlimited.
        Mode = self.bus.read("Operating_Mode", servo.name, normalize=False, num_retry=num_retry)
        Goal = self.bus.read("Goal_Position", servo.name, normalize=False, num_retry=num_retry)
        if Mode != OperatingMode.POSITION.value or Goal != hold_raw:
            raise RuntimeError(f"{servo.name} Hold read-back mismatch: mode {Mode}, goal {Goal}, expected {hold_raw}")
        ProfileSpeed = max(1, round(NORMAL_GOAL_SPEED * self.ActiveMotionSpeedPercent() / 100))
        self.bus.write("Goal_Speed", servo.name, ProfileSpeed, normalize=False, num_retry=num_retry)
        servo.command_raw = None
        servo.command_speed = 0
        servo.homing = False
        servo.GraspHold = False
        # The hold point is where it holds, inside its slider limits or not: clamped, the closed-loop hold drove a
        # hand-posed joint back to its limit, unchecked (2026-10-07 audit H3). Targets are limited where commanded.
        servo.target_raw = hold_raw % RESOLUTION
        servo.last_write_s = time.perf_counter()

    def _send_slider_velocity_locked(
        self,
        servo: ServoRuntime,
        *,
        set_mode: bool = False,
        num_retry: int = 0,
        route_speeds: dict[str, int] | None = None,
    ) -> int:
        if self.bus is None:
            return servo.target_raw
        SpeedPercent = self.ActiveMotionSpeedPercent()
        Routed = self.MotionRoute is not None and servo.homing
        # Once a joint has settled, its drift corrections keep the old Hold response.
        track_speed = ((self.RouteSpeeds() if route_speeds is None else route_speeds).get(servo.name, 0) if Routed
                       else self._slider_velocity_speed(servo, SpeedPercent, 1))
        if track_speed == 0:
            if Routed and set_mode and not self.RouteStageSettles():
                # Already close enough to an intermediate waypoint while parked:
                # keep the position hold rather than switching modes for nothing.
                servo.command_raw = None
                return servo.target_raw
            if Routed and not set_mode and not self.RouteStageSettles():
                # Wait at an intermediate waypoint in velocity mode, where zero
                # means stop. Parking in position mode would jerk the arm every waypoint.
                if servo.command_speed != 0:
                    self.bus.write("Goal_Speed", servo.name, 0, normalize=False, num_retry=num_retry)
                    servo.command_speed = 0
                servo.last_write_s = time.perf_counter()
                return servo.command_raw if servo.command_raw is not None else servo.target_raw
            self._stop_slider_velocity_locked(servo, hold_target=True, num_retry=num_retry)
            return servo.target_raw

        motor_speed = self._motor_velocity_speed(servo, track_speed)
        previous_speed = servo.command_speed
        same_direction = previous_speed == 0 or (previous_speed > 0) == (motor_speed > 0)
        speed_changed = (servo.AppliedSpeedPercent != SpeedPercent
                         or abs(motor_speed - previous_speed) >= max(1, round(SLIDER_VELOCITY_SPEED_DEADBAND * SpeedPercent / 100)))
        if not set_mode and same_direction and not speed_changed:
            servo.last_write_s = time.perf_counter()
            return servo.command_raw if servo.command_raw is not None else servo.target_raw

        if set_mode:
            # Seed a current-position goal before clearing the profile speed:
            # speed zero means maximum speed while still in position mode.
            Mode = self.bus.read("Operating_Mode", servo.name, normalize=False, num_retry=num_retry)
            if Mode == OperatingMode.POSITION.value:
                Hardware = servo.HardwareRaw if servo.HardwareRaw is not None else servo.present_raw
                self.bus.write("Goal_Position", servo.name, Hardware, normalize=False, num_retry=num_retry)
            self.bus.write("Goal_Speed", servo.name, 0, normalize=False, num_retry=num_retry)
            self.bus.write(
                "Operating_Mode",
                servo.name,
                OperatingMode.VELOCITY.value,
                normalize=False,
                num_retry=num_retry,
            )
            self.bus.write("Acceleration", servo.name, NORMAL_ACCELERATION, normalize=False, num_retry=num_retry)
            if self.bus.read("Operating_Mode", servo.name, normalize=False, num_retry=num_retry) != OperatingMode.VELOCITY.value:
                raise RuntimeError(f"{servo.name} velocity mode was not confirmed")
            self.bus.write("Goal_Position", servo.name, 0, normalize=False, num_retry=num_retry)
            self.bus.write("Goal_Time", servo.name, 0, normalize=False, num_retry=num_retry)

        self.bus.write("Goal_Speed", servo.name, motor_speed, normalize=False, num_retry=num_retry)
        servo.command_raw = servo.target_raw
        servo.command_speed = motor_speed
        servo.AppliedSpeedPercent = SpeedPercent
        servo.last_write_s = time.perf_counter()
        return servo.command_raw

    def _send_slider_target_locked(self, servo: ServoRuntime, raw: int, *, num_retry: int = 1,
                                   route_speeds: dict[str, int] | None = None) -> int:
        """Stream a joint to raw: a certified route's waypoint or a hold point, used exactly as given. Commands are
        limited where they come in (move_servo, go_home, the IK targets); clamping here moved a joint outside its
        limits, unchanged on its route or holding there, to the limit: unplanned, unchecked (2026-10-07 audit H3)."""
        starting_stream = servo.command_raw is None
        servo.GraspHold = False
        servo.target_raw = int(raw) % RESOLUTION
        servo.command_raw = servo.target_raw
        return self._send_slider_velocity_locked(servo, set_mode=starting_stream, num_retry=num_retry, route_speeds=route_speeds)

    def _write_active_slider_commands_locked(self) -> None:
        if not self.arm_connected or self.bus is None or not self.torque_enabled or self.MotionFault:
            return
        if self.MotionDeadline and time.monotonic() > self.MotionDeadline and any(S.command_raw is not None for S in self.servos.values()):
            Errors = {S.name: round(abs(self._slider_track_error(S))) for S in self.servos.values() if S.command_raw is not None}
            Summary = "; ".join(f"{Name.replace('_', ' ')} error {Error} raw steps" for Name, Error in Errors.items())
            self.LatchMotionFault("Motion settling timed out: " + Summary, Joint=max(Errors, key=Errors.get),
                                  Details={"errors_raw": Errors, "timeout_ms": self.MotionTimeoutMs})
            return
        # The lock keeps feedback, route stage and throttle fixed throughout this dispatch.
        # Calculate the complete synchronized command once, not once for each joint. Keep
        # this local: the next frame or stage must always use fresh feedback and limits.
        RouteSpeeds = self.RouteSpeeds() if self.MotionRoute is not None else None
        for servo in self.servos.values():
            if servo.command_raw is None and self.MotionRoute is not None and servo.homing and not self.RouteStageSettles():
                # Flying through: a joint parked near its waypoint rejoins the route
                # as soon as it has to move, and never re-arms a hold in between.
                if RouteSpeeds.get(servo.name, 0):
                    try:
                        self._send_slider_target_locked(servo, servo.target_raw, num_retry=0, route_speeds=RouteSpeeds)
                    except Exception as exc:
                        self.RaiseIfSerialLinkLost(exc)
                        self.LatchMotionFault(f"{servo.name} motion write failed: {exc}", Joint=servo.name,
                                              Details={"error": str(exc)})
                        break
                continue
            if servo.command_raw is None:
                # Closed-loop hold: a holding joint that drifted off its hold
                # point (gravity sag, enable inrush) gets its stream re-armed
                # so the follower drives it back. Fault stops clear Holding,
                # so this never fights a protection trip.
                if servo.Holding and not self.MotionFault and self.torque_enabled:
                    if servo.GraspHold:
                        # The object decides where the jaws rest and the servo's own position
                        # hold keeps the bounded squeeze (SqueezeGrasp); a correction toward a
                        # target would push into the object or let it go.
                        servo.target_raw = self._clamp_raw(servo, servo.present_raw)
                        continue
                    if not self._slider_target_reached(servo):
                        Error = abs(self._slider_track_error(servo))
                        if Error > HOLD_REARM_MAX_ERROR_RAW:
                            self.LatchMotionFault(
                                f"{servo.name} drifted {Error:.0f} steps past its hold point - re-hold manually",
                                Joint=servo.name,
                                Details={"drift_raw": round(Error), "limit_raw": HOLD_REARM_MAX_ERROR_RAW,
                                         "present_raw": servo.present_raw, "target_raw": servo.target_raw})
                            break
                        # The servo's own position hold keeps pulling toward the
                        # hold point either way. A return that cannot be checked
                        # only skips the stronger velocity correction: coasting
                        # here would drop an arm resting beside the mast.
                        if self.MotionRoute is None and not self.HoldReturnClear():
                            continue
                        try:
                            if self.CalibrationActive and servo.name in IK_JOINTS:
                                servo.homing = True
                            self._send_slider_target_locked(servo, servo.target_raw, num_retry=0, route_speeds=RouteSpeeds)
                        except Exception as exc:
                            self.RaiseIfSerialLinkLost(exc)
                            self.LatchMotionFault(f"{servo.name} hold write failed: {exc}", Joint=servo.name,
                                                  Details={"error": str(exc)})
                            break
                continue
            try:
                self._send_slider_velocity_locked(servo, num_retry=0, route_speeds=RouteSpeeds)
            except Exception as exc:
                self.RaiseIfSerialLinkLost(exc)
                self.LatchMotionFault(f"{servo.name} motion write failed: {exc}", Joint=servo.name,
                                      Details={"error": str(exc)})
                break
        if not any(Servo.command_raw is not None for Servo in self.servos.values()):
            self.MotionDeadline = 0.0
        if self.HoldCorrectionPaused and all(not S.Holding or S.command_raw is not None or self._slider_target_reached(S)
                                             for S in self.servos.values()):
            self.HoldCorrectionPaused = False

    def refresh(self) -> None:
        Asked = time.perf_counter()
        with self.lock:
            Waited = time.perf_counter() - Asked
            Gap = time.monotonic() - self.last_refresh_s if self.last_refresh_s else 0.0
            self.FramePhases = {}
            Delayed = self.torque_enabled and Gap > ControlHeartbeatLimitS
            # Held still from the end of the last frame to now: nothing ran unsupervised, so the arm is read
            # again (ResumeAfterControlDelay) instead of coasted. A move started or stopped meanwhile trips.
            HeldThrough = Delayed and self.HeldStillAtFrameEnd is not None and self.ArmHoldsStill()
            if Delayed and not HeldThrough:
                self.TripProtection("Motor control heartbeat was delayed",
                                    Details={"gap_ms": round(Gap * 1000), "limit_ms": round(ControlHeartbeatLimitS * 1000)})
            self._refresh_count += 1
            # While the serial port is lost this frame only works on getting it back.
            if self.ServiceSerialLink():
                try:
                    if HeldThrough:
                        self.ResumeAfterControlDelay(Gap)
                    if self.HandGuide is not None:
                        self.HandGuideSupervise()
                    else:
                        self.ControlFrame()
                except PortLostError:
                    pass  # the port failed during this frame: handled right here, not 33 ms later
                if self.SerialLinkLost():
                    self.ServiceSerialLink()
            self.last_refresh_s = time.monotonic()
            self.HeldStillAtFrameEnd = ({Name: Servo.present_raw for Name, Servo in self.servos.items()}
                                        if self.ArmHoldsStill() and not self.SerialLinkLost() else None)
            self.NoteControlStall(Gap, Waited, time.perf_counter() - Asked)

    def NoteControlStall(self, Gap: float, Waited: float, Took: float) -> None:
        """Record what a late or long control frame waited for (control_stall in bridge_events.jsonl). Called with the lock.

        Gap: from the end of the last frame to the start of this one; Waited: how long this frame waited for the
        motor lock; Took: the frame from asking for the lock to now. The record names the threads that held the
        lock for more than ControlLockHoldNoteS meanwhile (InstrumentedLock.Holds) and this frame's own steps
        (FramePhases, ms): a stall the lock does not explain was the CPU, the interpreter lock or a step of the
        frame itself (then the faulthandler dump in control_stalls.log, ControlStallDumpS, shows every thread).
        Diagnostics only, for the 1.1-1.7 s stalls of 2026-10-03..07 (audit L3/S1); never changes a command.
        """
        if Gap < ControlStallLogS and Took < ControlStallLogS:
            return
        try:
            Me = threading.current_thread().name
            Since = time.monotonic() - Gap - Took
            Holds = [Hold for Hold in self.lock.HoldsSince(Since) if not (Hold["thread"] == Me and Hold["ended_ms_ago"] is None)]
            Others = [Hold for Hold in Holds if Hold["thread"] != Me]
            Cause = ("the motor lock, held by " + ", ".join(f"{Hold['thread']} ({Hold['held_ms']} ms)" for Hold in Others)
                     if Waited >= ControlStallLogS / 2 and Others else
                     "this frame's own steps" if Took - Waited >= ControlStallLogS else
                     "the CPU or the interpreter lock (the control thread was not running)")
            Details = {"gap_ms": round(Gap * 1000), "lock_wait_ms": round(Waited * 1000), "frame_ms": round(Took * 1000),
                       "phases_ms": {Name: round(Value, 1) for Name, Value in self.FramePhases.items()},
                       "lock_holds": Holds[-8:], "moving": self.MotionInProgress(), "torque": self.torque_enabled,
                       "cause": Cause}
            Record = {**Details, "at_s": round(time.time(), 3)}
            self.ControlStalls.append(Record)
            self.RecordBridgeEvent("control_stall", f"Control frame {round(max(Gap, Took) * 1000)} ms late or long: {Cause}",
                                   Details=Details, Diagnostic=True)
        except Exception as Exc:  # a diagnostic must never stop a frame
            print(f"[control-stall] could not record: {Exc}", file=sys.stderr)

    def Phase(self, Name: str, Started: float) -> float:
        """Add the time since Started (perf_counter) to this frame's step Name (FramePhases, ms); returns now."""
        Now = time.perf_counter()
        self.FramePhases[Name] = self.FramePhases.get(Name, 0.0) + (Now - Started) * 1000
        return Now

    def ArmHoldsStill(self) -> bool:
        """Torque on and nothing commanded: every joint rests in its servo's own position hold.

        No route, stream, calibration, hold correction or nonzero speed. Each STS3215 then keeps its
        Goal_Position by itself, so a control frame that comes late leaves nothing running; a joint
        in velocity mode instead keeps its last speed until the next frame (the servos have no
        timeout for a silent host).
        """
        return (self.torque_enabled and not self.MotionFault and not self.MotionInProgress()
                and all(Servo.command_speed == 0 for Servo in self.servos.values()))

    def ResumeAfterControlDelay(self, Gap: float) -> None:
        """The control loop resumed Gap seconds after its last frame, the arm holding still throughout.

        2026-10-04 23:11:14 a 1.56 s delay with nothing moving tripped the heartbeat, which coasted the
        arm extended over the desk: the trip itself dropped it. Now every joint is read again first, as
        after a serial link drop (ReadEveryJoint): torque on, no hardware fault, position mode, and within
        SerialLinkHoldDriftRaw of where it held when the last frame ended. Only then does the arm go on
        holding, and this frame still reads temperatures and status before any write. Anything else
        trips protection as the delay did before. Either way the delay is recorded.
        """
        Problems, _, _ = self.ReadEveryJoint(self.HeldStillAtFrameEnd or {}, "while the control loop was delayed")
        if self.SerialLinkLost():
            return  # the link supervisor reads every joint again once the port is back
        self.LastProtectionPoll = 0.0  # temperatures are stale too: read in this frame
        Details = {"gap_ms": round(Gap * 1000), "limit_ms": round(ControlHeartbeatLimitS * 1000), "held_still": True,
                   "problems": Problems}
        if Problems:
            self.TripProtection("Motor control heartbeat was delayed: " + "; ".join(Problems), Details=Details)
            return
        Note = f"Control loop delayed {Gap:.1f} s while the arm held still; every joint read back holding"
        self.ControlDelays.append({"gap_ms": Details["gap_ms"], "at_s": time.time()})
        self.ControlDelayNote, self.ControlDelayNoteAt = Note, time.monotonic()
        self.RecordBridgeEvent("control_delayed", Note, Details=Details)
        self.status = Note

    def ControlFrame(self) -> None:
        """One control frame: fresh feedback and protection checks before any command is written.

        Raises PortLostError when the serial port itself fails, before anything is latched.
        Each step's time goes to FramePhases (NoteControlStall reports a long frame's steps).
        """
        Mark = time.perf_counter()
        self._read_positions_locked()
        Mark = self.Phase("read_positions", Mark)
        if self.SerialLinkLost():
            return
        if self.torque_enabled:
            self._check_servo_faults_locked()
            Mark = self.Phase("servo_status", Mark)
            if self.torque_enabled:
                self.CheckMotorProtection()
                Mark = self.Phase("protection", Mark)
        if self.ProtectionTrip and not self.ProtectionTrip["coast_confirmed"] and time.monotonic() - self.LastCoastAttempt >= .5:
            self.VerifyProtectionCoast()
            Mark = self.Phase("coast_retry", Mark)
        if self.HandGuideRestoreSpeedAt and time.monotonic() >= self.HandGuideRestoreSpeedAt:
            self.HandGuideRestoreProfile()
        self.CheckMotionRoute()
        Mark = self.Phase("route_check", Mark)
        self._write_active_slider_commands_locked()
        Mark = self.Phase("commands", Mark)
        self.RecordMotionTrace()
        self.AdvanceMotionRoute()
        self.Phase("route_advance", Mark)

    def RecordRouteTiming(self, Route, Fault: str | None = None) -> None:
        """Keep how a completed route went, one JSON line in captures/<date>/recording/route_timing.jsonl.

        Diagnostics for route speed, like RecordMotionTrace, which this copies
        from: the plan time (command to start), the flying time (start to
        settled), each stage's time, and every control frame of the route
        (seconds from its start, stage, each joint's position and command
        speed in MOTOR_NAMES order), with each waypoint's reach per joint
        (reach_raw). 2026-10-04 the time of a route on the arm could only be
        read off the camera video. A route a latched fault stopped is kept too,
        with the fault (Fault). Only the running server writes it (TimingSink);
        never raises, and changes no command.
        """
        try:
            if self.TimingSink is None:
                return
            Now, Wall = time.monotonic(), time.time()
            # Rounded as RecordMotionTrace rounds each frame's time: on Windows time.monotonic ticks every 15.6 ms, and
            # a frame in the route start's own tick rounded below an unrounded start and was dropped (audit L5/L7).
            Since = round(self.MotionStartedAt - self.MotionTraceStart, 4)
            Frames = [[round(Frame["t"] - Since, 3), Frame["stage"], [Frame["joints"][Name][0] for Name in MOTOR_NAMES],
                       [Frame["joints"][Name][1] for Name in MOTOR_NAMES]]
                      for Frame in self.MotionTrace if Frame["t"] >= Since]
            Stages: dict[int, list[float]] = {}
            for Seconds, Stage, _, _ in Frames:
                if Stage is not None:
                    Stages.setdefault(Stage, [Seconds, Seconds])[1] = Seconds
            Plan = Route.get("PlanSeconds")
            self.TimingSink({
                "kind": "route_timing", "log": ROUTE_TIMING_LOG, "t": round(Wall, 3),
                "time": time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(Wall)) + f".{int(Wall % 1 * 1000):03d}Z",
                "plan_s": None if Plan is None else round(Plan, 3), "run_s": round(Now - self.MotionStartedAt, 3),
                "speed_percent": self.MotionSpeedPercent, "stages": len(Route["Path"]), "tight": sorted(Route["Tight"]),
                "arrive": sorted(Route.get("Arrive", ())),
                "reach": {Index: max(Reach.values()) for Index, Reach in Route.get("Reach", {}).items()},
                "reach_raw": dict(Route.get("Reach", {})),
                **({"fault": Fault, "stage_at_fault": Route["Index"]} if Fault else {}),
                "stage_s": [round(Stages[Index][1] - Stages[Index][0], 3) if Index in Stages else None
                            for Index in range(len(Route["Path"]))],
                "calibration": Route.get("Calibration", False), "route_check_workers": RouteChecks.WorkerCount(),
                "route_planner_version": Collision.PlannerVersion, "route_executor_version": RouteExecutorVersion,
                "joints": list(MOTOR_NAMES), "start_raw": Route.get("Origin"), "waypoints_raw": Route["Path"], "frames": Frames})
        except Exception as Exc:
            print(f"[route-timing] could not record: {Exc}", file=sys.stderr)

    def RecordMotionTrace(self) -> None:
        """Keep the last seconds of commands and feedback, at the control rate.

        The route's speed limits are only as good as the servo model behind them
        (ROUTE_BRAKE_LATENCY_S and friends), and a 10 Hz poll from outside cannot
        measure a 0.1 s lag. Diagnostics only: nothing here changes a command.
        """
        if not self.arm_connected or not self.torque_enabled:
            return
        Route = self.MotionRoute
        self.MotionTrace.append({
            "t": round(time.monotonic() - self.MotionTraceStart, 4),
            "stage": None if Route is None else Route["Index"],
            "joints": {Name: (Servo.present_raw, Servo.command_speed,
                              None if Servo.command_raw is None else Servo.target_raw)
                       for Name, Servo in self.servos.items()},
        })

    def _check_servo_faults_locked(self) -> None:
        """Every joint's hardware status and torque (ReadServoStatus), read again at once when they show anything.

        Only a reading the next one confirms latches (ProtectionVersion v2-confirmed): a fault bit or torque off read twice
        in a row latches that fault, two failed or corrupted readings in a row "Cannot read". A bad reading the re-read
        contradicts is a protection_glitch: 2026-10-12 16:32:43 one reply read torque off with 5.6 V, 90 C and load
        0xF901 (the signature of every "torque unexpectedly off" recorded since 2026-09-28) and coasted a holding arm
        whose positions never sagged. A servo that really restarted keeps torque off on the re-read: caught one reading,
        a few milliseconds, later than before. Nothing is written between the two readings.
        """
        if not self.arm_connected or self.bus is None:
            if self.faults:
                self.faults = {}
            return
        new_unload: list[str] = []
        NewFaults: list[dict[str, Any]] = []

        def Read(Name: str) -> tuple[int, dict[str, Any]] | Exception:
            try:
                return self.ReadServoStatus(Name)
            except Exception as Exc:
                if self.SerialLinkLost():
                    # A fault another servo already reported in this frame is latched before the
                    # frame is abandoned. It is in self.faults by now, and later frames latch only
                    # bits they have not seen, so it would otherwise never latch.
                    self.LatchServoFaults(new_unload, NewFaults)
                    self.RaiseIfSerialLinkLost(Exc)
                return Exc

        for name in MOTOR_NAMES:
            Reading = Read(name)
            if isinstance(Reading, Exception) or Reading[0]:
                Again = Read(name)
                if not isinstance(Again, Exception) and not Again[0]:
                    self.NoteProtectionGlitch(name, *self.StatusGlitchDetails(Reading))
                    Reading = Again
                elif isinstance(Again, Exception) and isinstance(Reading, Exception):
                    if self.torque_enabled and not self.MotionFault:
                        self.LatchMotionFault(f"Cannot read {name} protection/torque status: {Again}", Joint=name,
                                              Details={"error": str(Again), "first_error": str(Reading),
                                                       **({"telemetry": Again.Telemetry} if isinstance(Again, CorruptStatusReply) else {})})
                    continue
                elif not isinstance(Again, Exception):
                    Reading = Again  # confirmed (or a failed reading confirmed by this one): the latest fault reading
            bits, Telemetry = Reading
            unload = [label for bit, label in FAULT_UNLOAD_BITS.items() if bits & (1 << bit)]
            if bits & (1 << 7):
                unload.append("torque unexpectedly off")
            warn = [label for bit, label in FAULT_WARN_BITS.items() if bits & (1 << bit)]
            Unknown = bits & ~((1 << 7) | sum(1 << Bit for Bit in (*FAULT_UNLOAD_BITS, *FAULT_WARN_BITS)))
            if Unknown:
                warn.append(f"unrecognized hardware status 0x{Unknown:02x}")
            if not unload and not warn:
                self.faults.pop(name, None)
                continue
            previous = self.faults.get(name)
            record = {"servo": name, "bits": bits, "unload": unload, "warn": warn, **Telemetry}
            if previous is None or previous["bits"] != bits:
                event = dict(record, at=time.strftime("%H:%M:%S"), at_s=time.time())
                self.fault_events = (self.fault_events + [event])[-FAULT_HISTORY:]
                if unload or warn:
                    new_unload.append(f"{name.replace('_', ' ')} {'/'.join(unload + warn)}")
                    NewFaults.append(record)
            self.faults[name] = record
        self.LatchServoFaults(new_unload, NewFaults)

    def ReadServoStatus(self, Name: str) -> tuple[int, dict[str, Any]]:
        """One reading of Name's status: Hardware_Error_Status, bit 7 set for torque reading off while it should be on, and,
        when that shows anything, the telemetry its fault record keeps (load, supply, temperature, Lock: 0 after a restart).
        Raises when a read fails, or CorruptStatusReply when that telemetry is one no servo reports."""
        # One retry, as for position feedback: a single dropped packet is not a lost servo.
        Bits = int(self.bus.read("Hardware_Error_Status", Name, normalize=False, num_retry=1))
        TorqueOn = bool(self.bus.read("Torque_Enable", Name, normalize=False, num_retry=1))
        if self.torque_enabled and not TorqueOn:
            Bits |= 1 << 7
        Telemetry: dict[str, Any] = {}
        if Bits:
            try:
                for Key, Register in (("load", "Present_Load"), ("voltage_dv", "Present_Voltage"),
                                      ("temperature_c", "Present_Temperature"), ("lock", "Lock")):
                    Telemetry[Key] = int(self.bus.read(Register, Name, normalize=False, num_retry=0))
            except Exception:
                pass
            Impossible = self.ServoTelemetryImpossible(Telemetry)
            if Impossible:
                raise CorruptStatusReply(f"{Name} status {Bits} came with {Impossible}", Bits, Telemetry)
        return Bits, Telemetry

    @staticmethod
    def ServoTelemetryImpossible(Telemetry: dict[str, Any]) -> str | None:
        """What in a status reading's telemetry no servo can report, or None (ServoLoadMaximum, ServoVoltageRangeDv,
        ServoHotC). The real readings on record are 27-38 C at about 8.9 V; every "torque unexpectedly off" since
        2026-09-28 came with 89-90 C and 0-5.6 V or 15.2 V, and a load with bits over the register's range."""
        Load, Volts, Celsius = Telemetry.get("load"), Telemetry.get("voltage_dv"), Telemetry.get("temperature_c")
        if Load is not None and abs(Load) > ServoLoadMaximum:
            return f"load {Load}, beyond the register's 0-{ServoLoadMaximum}"
        if Volts is not None and Volts > ServoVoltageRangeDv[1]:
            return f"supply {Volts / 10:.1f} V, over the servo's {ServoVoltageRangeDv[1] / 10:.1f} V"
        if Volts is not None and Celsius is not None and Volts < ServoVoltageRangeDv[0] and Celsius >= ServoHotC:
            return f"supply {Volts / 10:.1f} V at {Celsius} C"
        return None

    @staticmethod
    def StatusGlitchDetails(Reading: tuple[int, dict[str, Any]] | Exception) -> tuple[str, Any, str, dict[str, Any]]:
        """NoteProtectionGlitch's register, value, note and details for a status reading the re-read contradicted."""
        if isinstance(Reading, Exception) and not isinstance(Reading, CorruptStatusReply):
            Register = next((Each for Each in ("Hardware_Error_Status", "Torque_Enable") if Each in str(Reading)), "Hardware_Error_Status")
            return Register, None, "failed read; the re-read answered clean", {"error": str(Reading)}
        Bits, Telemetry = (Reading.Bits, Reading.Telemetry) if isinstance(Reading, Exception) else Reading
        Register, Value = ("Torque_Enable", 0) if Bits == 1 << 7 else ("Hardware_Error_Status", Bits & 0x7F)
        Note = f"{'torque read off' if Bits & (1 << 7) else f'status bits {Bits}'}; the re-read answered clean"
        return Register, Value, Note, {"bits": Bits, **Telemetry,
                                       **({"impossible": str(Reading)} if isinstance(Reading, Exception) else {})}

    def LatchServoFaults(self, Unload: list[str], Faults: list[dict[str, Any]]) -> None:
        if Unload and not self.MotionFault:
            # Stop every stream: a partly limp arm plus live velocity commands is
            # exactly how it fell and then lurched back into motion.
            self.LatchMotionFault(f"Servo protection/torque fault ({', '.join(Unload)})",
                                  Joint=Faults[0]["servo"], Details={"faults": Faults})

    def connect(self, port: str) -> tuple[bool, str]:
        port = str(port or "").strip()
        if not port:
            return False, "Select a serial port first"

        with self.lock:
            if self.arm_connected and port and _NormalizePort(self.port) == _NormalizePort(port):
                if self.SerialLinkLost():
                    # The supervisor is already reopening this port; try once more right now.
                    self.SerialLink.LastAttempt = 0.0
                    self.ServiceSerialLink()
                    return not self.SerialLinkLost(), self.status
                return True, f"Arm already connected on {self.port}"
            AbandonedHolding = None
            if self.arm_connected and self.SerialLinkLost():
                # The old port is gone (the adapter came back under another name, or moved).
                # Nothing can be sent through it: let its dead handle go. A latched torque-off
                # carries over and is completed on the new port below.
                Old = self.bus
                self.RecordBridgeEvent("serial_link_abandoned", f"Gave up on {self.port} for {port}: {Old.PortLost}",
                                       Details={"serial_link": self.SerialLinkCounts()})
                if self.torque_enabled and not self.ProtectionTrip:
                    AbandonedHolding = self.port  # its servos may still hold: see the connect failure below
                self.bus, self.torque_enabled, self.SerialLink.Down = None, False, False
                try:
                    Old.disconnect(disable_torque=False)
                except Exception:
                    pass
            if self.arm_connected:
                # A real port switch, not a spelling variant: coast first.
                Ok, Message = self.disconnect(disable_torque=True)
                if not Ok:
                    return False, Message
            bus = _make_bus(port)
            try:
                bus.connect()
            except Exception as exc:
                self.bus = None
                self.port = ""
                self.torque_enabled = False
                self.status = f"Connect failed: {exc}"
                if AbandonedHolding:
                    # Nothing has reached the servos behind the abandoned port since they were
                    # holding: keep their torque-off pending, so Emergency stop cannot claim a
                    # coast and the next connect sends it.
                    self.AutoCoast(f"Gave up on {AbandonedHolding} while the arm was holding, and connect on {port} "
                                   f"failed: {exc}", Kind="motion_fault")
                return False, self.status

            self.bus = bus
            self.port = port
            self.torque_enabled = False
            self.SerialLink.Down = False
            self.SerialLink.Note = None

            try:
                return self._FinishConnect(bus, port)
            finally:
                # The first control frame after connect measures its heartbeat from here. Connect holds the lock
                # for bus.connect and configure_motors (about 90 transactions, 12 of them EEPROM writes), and the arm
                # can come out of it holding (the Goal_Position seeds energise it): a connect over 0.75 s tripped the
                # heartbeat on that first frame and dropped an arm that was holding (2026-10-07 audit H4). The
                # moving-state limit is untouched: nothing is commanded at the end of a connect.
                self.last_refresh_s = time.monotonic()
                self.HeldStillAtFrameEnd = ({Name: Servo.present_raw for Name, Servo in self.servos.items()}
                                            if self.ArmHoldsStill() and not self.SerialLinkLost() else None)

    def _FinishConnect(self, bus, port: str) -> tuple[bool, str]:
        """Connect's work on its new, open bus: motor profiles, persistent data, fresh positions, the torque state
        and a pending torque-off. Called with the lock, from connect only."""
        def LinkDropped() -> bool:
            """The link dropped during connect, after the Goal_Position seeds that energise joints.

            Their torque-off is latched and sent the moment the link returns, instead of the
            joints holding unsupervised behind a failed connect.
            """
            if not self.SerialLinkLost():
                return False
            self.AutoCoast(f"Connect on {port} could not be completed: the USB serial link to the arm dropped",
                           Kind="motion_fault")
            return True

        try:
            bus.configure_motors()
        except Exception as Exc:
            if LinkDropped():
                return False, self.status
            self.MotionFault = f"Motor profile setup failed: {Exc}"
            self.RecordBridgeEvent("motion_fault", self.MotionFault, Details={"error": str(Exc), "port": port})
            self.status = self.MotionFault
            return False, self.status
        self._reload_persistent_data()
        for servo in self.servos.values():
            servo.gui_angle_raw = None
        if not self._read_positions_locked():
            if LinkDropped():
                return False, self.status
            self.MotionFault = "Connection has incomplete joint feedback"
            Missing = [Name for Name in MOTOR_NAMES if not self.servos[Name].FeedbackOk]
            self.RecordBridgeEvent("motion_fault", self.MotionFault, Missing[0] if Missing else None,
                                   {"missing": Missing, "port": port})
            return False, self.MotionFault
        # Fresh connection: bookkeeping follows the live arm, never a stale
        # target from before a refresh/coast sag. Otherwise the next Hold or
        # move would drive toward a ghost. Where it is, inside its limits or
        # not: a clamped target is a move nobody planned (audit H3).
        for Servo in self.servos.values():
            Servo.target_raw = Servo.present_raw
            Servo.command_raw = None
            Servo.command_speed = 0
            Servo.homing = False
            Servo.Holding = False
        try:
            TorqueStates = []
            LastExc: Exception | None = None
            for _ in range(3):
                try:
                    TorqueStates = [bool(bus.read("Torque_Enable", Name, normalize=False, num_retry=1))
                                    for Name in MOTOR_NAMES]
                    LastExc = None
                    break
                except Exception as Exc:
                    LastExc = Exc
            if LastExc is not None:
                raise LastExc
            self.torque_enabled = all(TorqueStates)
            if any(TorqueStates) and not all(TorqueStates):
                # Takeover after an unclean handoff (killed bridge process,
                # tripped servo): normalize by coasting everything instead of
                # latching. Re-Hold still refuses while a hardware fault bit
                # persists, so no protection is bypassed.
                try:
                    bus.disable_torque(num_retry=1)
                except Exception:
                    pass
                TorqueStates = [bool(bus.read("Torque_Enable", Name, normalize=False, num_retry=1))
                                for Name in MOTOR_NAMES]
                self.torque_enabled = all(TorqueStates)
                if any(TorqueStates):
                    self.LatchMotionFault("Joint torque state did not normalize after connection",
                                          Details={"torque_on": [Name for Name, On in zip(MOTOR_NAMES, TorqueStates) if On]})
                    return False, self.status
        except Exception as Exc:
            if LinkDropped():
                return False, self.status
            self.MotionFault = f"Cannot verify joint torque: {Exc}"
            self.RecordBridgeEvent("motion_fault", self.MotionFault, Details={"error": str(Exc), "port": port})
            return False, self.MotionFault
        angles = self._current_ik_angles()
        self.target_cm = forward_tip_position(self.arm_geometry, angles)
        self.target_wrist_pitch_deg = wrist_tip_pitch_degrees(self.arm_geometry, angles)
        self.elbow_mode = self._current_elbow_branch(angles)
        self.status = f"Connected on {port}"
        Trip = self.ProtectionTrip
        if Trip and not Trip["coast_confirmed"]:
            # A latched torque-off that never reached the servos (the serial link dropped,
            # or the arm was disconnected first) completes now, and says so. It used to
            # happen silently on the first frame after connect: the arm just dropped.
            self.VerifyProtectionCoast()
            Confirmed = self.ProtectionTrip["coast_confirmed"]
            self.status = (f"Connected on {port} and sent the torque-off pending since: {Trip['reason']}. "
                           + ("All motors coasting: support the arm, then press Hold." if Confirmed
                              else "TORQUE-OFF still UNCONFIRMED: " + ", ".join(self.ProtectionTrip["unconfirmed"])))
            self.RecordBridgeEvent("pending_coast_sent", self.status, Details={"coast_confirmed": Confirmed, "port": port})
            return Confirmed, self.status
        return True, self.status

    def disconnect(self, disable_torque: bool = True) -> tuple[bool, str]:
        if disable_torque:
            return self.emergency_stop(Reason="Arm disconnected (coast)")
        with self.lock:
            self.EndHandGuide("teardown", "Arm disconnected", Release=False)  # (disconnected holding, as asked)
            bus = self.bus
            if self.MotionRoute is not None:
                self.RecordBridgeEvent("motion_abort", "Arm disconnected without coasting")
            self.MotionRoute = None
            self.ClearRouteScene()
            self.CalibrationPrepareToken = None
            self.MotionEpoch += 1
            self.bus = None
            self.torque_enabled = False
            self.SerialLink.Down = False
            for servo in self.servos.values():
                servo.command_raw = None
                servo.command_speed = 0
                servo.homing = False
                servo.Holding = servo.GraspHold = False
            if bus is not None:
                try:
                    bus.disconnect(disable_torque=disable_torque)
                except Exception:
                    pass
            self.status = "Arm disconnected"
            return True, self.status

    def emergency_stop(self, Reason: str = "Emergency shutoff") -> tuple[bool, str]:
        """Coast and disconnect. Reason names the caller in the record of an interrupted motion.

        Never reports a coast that did not reach the servos. A lost serial port is reopened
        first; if it stays gone the arm stays attached with the coast latched, and the link
        supervisor (ServiceSerialLink) sends it the moment the port returns.
        """
        with self.lock:
            self.EndHandGuide("teardown", Reason)
            if self.MotionRoute is not None:
                self.RecordBridgeEvent("motion_abort", Reason)
            self.MotionEpoch += 1
            self.CalibrationPlans.clear()
            self.CalibrationActive = False
            self.CalibrationPrepareToken = None
            self.MotionRoute = None
            self.ClearRouteScene()
            self.MotionDeadline = 0
            Errors = []
            bus = self.bus
            self.torque_enabled = False

            for servo in self.servos.values():
                servo.command_raw = None
                servo.command_speed = 0
                servo.homing = False
                servo.Holding = servo.GraspHold = False
                servo.target_raw = servo.present_raw

            Trip = self.ProtectionTrip
            if bus is None:
                self.port = ""
                if Trip and not Trip["coast_confirmed"]:
                    # It used to answer "coast and disconnected" here although nothing reached a servo.
                    self.status = (f"No arm connection, and the torque-off pending since \"{Trip['reason']}\" never reached "
                                   "the servos: they may still be holding. Connect the arm to send it, and support the arm.")
                    return False, self.status
                self.status = "Emergency shutoff - coast and disconnected"
                return True, self.status

            def Reopened() -> bool:
                if not self.SerialLink.Down:
                    self.BeginSerialOutage(Latch=False)
                self.SerialLink.LastAttempt = time.monotonic()
                self.SerialLink.Attempts += 1
                return bus.Reopen()

            if self.SerialLinkLost():
                Reopened()
            if bus.is_connected and not self.SerialLinkLost():
                self.StopSpeedsBeforeTorqueOff(bus)
                try:
                    bus.disable_torque(num_retry=0)
                except Exception as Exc:
                    # The port failed during the torque-off itself: a fresh handle and one more try.
                    if self.SerialLinkLost() and Reopened():
                        try:
                            bus.disable_torque(num_retry=0)
                        except Exception as Again:
                            Errors.append(str(Again))
                    else:
                        Errors.append(str(Exc))
                if not Errors and not self.SerialLinkLost():
                    # Read the torque-off back, as VerifyProtectionCoast does: a write acknowledgement is not a
                    # coast (2026-10-07 audit L4). One more torque-off for any joint still on or silent.
                    Still = self.TorqueStillOn(bus)
                    if Still:
                        try:
                            bus.disable_torque(num_retry=1)
                        except Exception:
                            pass
                        Still = self.TorqueStillOn(bus)
                    if Still:
                        Errors.append("torque still on, or not read back, on " + ", ".join(Still))
            if self.SerialLinkLost():
                # Nothing reaches the servos. Stay attached with the coast latched: the link
                # supervisor reopens the port and sends it the moment the port returns.
                self.AutoCoast(f"{Reason}: torque-off not sent, the USB serial link to the arm is down", Kind="motion_fault")
                return False, self.status
            if self.SerialLink.Down:
                self.EndSerialOutage(f"USB serial link dropped for {self.SerialLinkDownSeconds():.1f} s; "
                                     "reconnected to send the emergency stop")

            self.bus = None
            self.port = ""
            try:
                bus.disconnect(disable_torque=False)
            except Exception:
                pass
            if Errors:
                self.status = "Emergency torque-off unconfirmed: " + "; ".join(Errors)
                self.MotionFault = self.status
                # Kept latched, so the next connect sends it and no later press claims a coast.
                if not Trip or Trip["coast_confirmed"]:
                    self.ProtectionTrip = {"reason": self.status, "at_s": time.time(), "coast_confirmed": False,
                                           "unconfirmed": list(MOTOR_NAMES)}
                self.RecordBridgeEvent("motion_fault", self.status, Details={"errors": Errors})
                return False, self.status
            if Trip and not Trip["coast_confirmed"]:
                # Every servo acknowledged its torque-off: the latched coast is done.
                Trip.update(coast_confirmed=True, unconfirmed=[])
            self.status = "Emergency shutoff - coast and disconnected"
            return True, self.status

    def JointsOutsideLimits(self) -> list[str]:
        """The joints whose measured position is outside their slider limits (hand-posed, or the limits moved)."""
        return [Name for Name, Servo in self.servos.items() if not self._contains_raw(Servo, Servo.present_raw)]

    def OutsideLimitsNote(self) -> str:
        """"; elbow flex is outside its joint limits, ..." for the status, or "" when every joint is inside its own."""
        Outside = [Name.replace("_", " ") for Name in self.JointsOutsideLimits()]
        if not Outside:
            return ""
        One = len(Outside) == 1
        return (f"; {', '.join(Outside)} {'is' if One else 'are'} outside {'its' if One else 'their'} joint limits, held where "
                f"{'it stands' if One else 'they stand'}: move {'it' if One else 'them'} back into range with "
                f"{'its slider' if One else 'their sliders'} (a checked move)")

    @staticmethod
    def TorqueStillOn(Bus) -> list[str]:
        """The joints whose Torque_Enable reads on, or does not read at all (each read retried once)."""
        Still = []
        for Name in MOTOR_NAMES:
            try:
                if int(Bus.read("Torque_Enable", Name, normalize=False, num_retry=1)):
                    Still.append(Name)
            except Exception:
                Still.append(Name)
        return Still

    def toggle_torque(self, Enable: Any = None) -> tuple[bool, str]:
        """Hold (torque on) or Coast (torque off). Enable, when a bool, says which the user pressed: the command
        then means that state, not a toggle, so it can be handled ahead of a queued or planning command without
        a second press undoing the first (2026-10-07 audit M2). Hold pressed while holding stops a motion and holds."""
        with self.lock:
            if isinstance(Enable, bool) and Enable == self.torque_enabled and self.arm_connected and not self.SerialLinkLost():
                if Enable and self.MotionInProgress():
                    return self.StopMotion("Hold pressed: motion stopped, holding where it is")
                self.status = "Torque already on: holding" if Enable else "Torque already off: coasting"
                return True, self.status
            if not self.arm_connected or self.bus is None:
                self.status = "Connect Arm before enabling torque"
                return False, self.status
            if self.SerialLinkLost():
                if self.torque_enabled:
                    # Coast requested: latch it, so torque-off is sent the moment the link returns.
                    self.AutoCoast("Coast requested while the USB serial link to the arm was down")
                    return False, self.status
                # Hold re-arms only from fresh reads of every joint, which need the link.
                return False, "Cannot enable Hold: " + self.SerialLinkDownText()
            Action = "Coast" if self.torque_enabled else "Hold"
            try:
                self.MotionEpoch += 1
                if self.torque_enabled:
                    Guiding = self.HandGuide is not None
                    self.EndHandGuide("teardown", "Coast pressed")
                    if Guiding:
                        # Every joint was just released: StopMotion's hold writes would energise them all again first.
                        # Guiding left nothing else to stop (it starts with no route, stream or plan, and refuses them).
                        self.CalibrationPlans.clear()
                        self.CalibrationActive = False
                        self.MotionDeadline = 0.0
                        self.MotionRoute = None
                        self.ClearRouteScene()
                        for Servo in self.servos.values():
                            Servo.homing = Servo.Holding = Servo.GraspHold = False
                            Servo.target_raw = Servo.present_raw
                    else:
                        self.StopMotion("Coasting", KeepGrasp=False)
                    self.bus.disable_torque(num_retry=1)
                    self.torque_enabled = False
                    for servo in self.servos.values():
                        servo.command_raw = None
                        servo.command_speed = 0
                    self.status = "Torque off"
                else:
                    # Point every joint at where it physically is and arm the
                    # motion profile BEFORE energising. Two reasons: torque was
                    # being enabled against whatever Goal_Position happened to
                    # be left over, which for a joint that had been in velocity
                    # mode is raw 0, and with Goal_Speed still 0 the servo would
                    # not drive to its goal at all — so Hold felt soft and the
                    # arm could be pushed around against light resistance.
                    if not self._read_positions_locked():
                        raise RuntimeError("Cannot enable Hold without fresh positions from every joint")
                    if self.ProtectionTrip:
                        self.VerifyProtectionCoast()
                        if not self.ProtectionTrip["coast_confirmed"]:
                            raise RuntimeError("Cannot rearm: torque-off is still unconfirmed")
                    self.CheckMotorProtection(BeforeArm=True)
                    for Name in MOTOR_NAMES:
                        Bits = int(self.bus.read("Hardware_Error_Status", Name, normalize=False, num_retry=1))
                        if Bits:
                            raise RuntimeError(f"Resolve {Name} hardware fault {Bits} before enabling Hold")
                    if self.HandGuideAccelerationDirty:
                        for Name in MOTOR_NAMES:
                            self.bus.write("Acceleration", Name, NORMAL_ACCELERATION, normalize=False, num_retry=1)
                        self.HandGuideAccelerationDirty = False
                    Wrist = self.servos["wrist_roll"]
                    Left = Wrist.range_max if Wrist.arc_inverted else Wrist.range_min
                    Right = Wrist.range_min if Wrist.arc_inverted else Wrist.range_max
                    # Firmware has one contiguous interval. The GUI wrist arc
                    # crosses encoder zero, so its exclusion is enforced by the
                    # bridge instead of the old, opposite narrow firmware window.
                    Minimum, Maximum = (0, RESOLUTION - 1) if Left > Right else (Left, Right)
                    self.bus.SetPositionRangeWhileCoasting("wrist_roll", Minimum, Maximum)
                    # Every joint holds exactly where it is, also one hand-posed past its slider limit: Hold used to
                    # drive such a joint back to the limit at up to 160 steps/s, with no collision check and no size
                    # limit (2026-10-07 audit H3). Bringing it back into range is an explicit, checked move.
                    for name in MOTOR_NAMES:
                        servo = self.servos[name]
                        servo.command_raw = None
                        servo.command_speed = 0
                        servo.homing = False
                        servo.target_raw = servo.present_raw
                        self._stop_slider_velocity_locked(servo, hold_target=False, num_retry=1)
                    self.bus.enable_torque(num_retry=1)
                    Off = [Name for Name in MOTOR_NAMES
                           if not int(self.bus.read("Torque_Enable", Name, normalize=False, num_retry=1))]
                    if Off:
                        try:
                            self.bus.disable_torque(num_retry=0)
                        except Exception:
                            pass
                        raise RuntimeError(f"Torque did not engage on: {', '.join(Off)}")
                    # Closed-loop hold: arm a stream at each hold point so the
                    # 30 Hz loop drives back any gravity droop (e.g. shoulder
                    # lift sag during the enable inrush) instead of leaving the
                    # joints on a one-shot goal nobody reasserts.
                    for Servo in self.servos.values():
                        Servo.Holding = True
                        self._send_slider_target_locked(Servo, Servo.target_raw, num_retry=1)
                    self.MotionDeadline = 0.0
                    self.torque_enabled = True
                    self.MotionFault = None
                    self.ProtectionTrip = None
                    self.ProtectionProgress.clear()
                    self.last_refresh_s = time.monotonic()
                    self.faults = {}
                    self.status = "Torque on" + self.OutsideLimitsNote()
            except Exception as exc:
                if self.SerialLinkLost():
                    # Some goals may have gone out and energised joints: coast them once the link returns.
                    self.AutoCoast(f"{Action} could not be completed: the USB serial link to the arm dropped")
                else:
                    self.AutoCoast(f"Torque command could not be safely completed: {exc}")
                return False, self.status
            # A note such as "...; holding at the last position" describes the torque state the
            # user just changed: it must not come back as the status of the next frames.
            self.SerialLink.Note = self.ControlDelayNote = None
            return True, self.status

    def SendJointMotion(self, Targets, Label, SceneId=None, SceneRequired=False, SceneCarry=None, Home=False):
        """A joint-space move to Targets (raw, limited). Home: Home's route (HomeGripperPlan, PlanHomeRoute)."""
        Asked = time.monotonic()
        try:
            with self.lock:
                if self.CalibrationActive:
                    raise ValueError("Stop calibration before using manual controls")
                self.MotionPreflight()
                if self.MotionRoute or any(S.command_raw is not None for S in self.servos.values()):
                    raise ValueError("Wait for this motion to finish, or press Stop before another move")
                Epoch, Bus, Configuration = self.MotionEpoch, self.bus, self.ConfigurationKey()
                Snapshot = self.PlanningSnapshot()
                Start = {Name: S.present_raw for Name, S in self.servos.items()}
                Gripper, Note = self.HomeGripperPlan(Targets) if Home else (None, None)
                Held = self.servos[GRIPPER_NAME].GraspHold
            # Off the motor lock: binding copies the measured scene (up to ~1400 boxes), planning takes seconds.
            Snapshot.BindScene(SceneId, SceneRequired, SceneCarry)
            Path = Snapshot.PlanHomeRoute(Targets, Gripper) if Home else Snapshot.PlanMotionRoute(Targets)
            FullAllowance = Snapshot.StartHasAllowance()
            with self.lock:
                if Epoch != self.MotionEpoch or Bus is not self.bus or Configuration != self.ConfigurationKey():
                    raise ValueError("Motion cancelled: arm state changed during route planning")
                self.MotionPreflight()
                if self.MovedDuringPlanning(Start, FullAllowance):
                    raise ValueError("Arm moved during planning; retry from its new position")
                # Home's gripper was planned for the grip it had: holding an object, or not.
                if Home and self.servos[GRIPPER_NAME].GraspHold != Held:
                    raise ValueError("Home cancelled: the gripper's grip changed during route planning; press Home again")
                # Refused before anything is written: a scene that changed or would expire mid-route is no fault.
                Refusal = self.SceneDispatchRefusal(Snapshot, Start, Path)
                if Refusal:
                    raise ValueError(Refusal)
                if QueuedCommandStale():
                    raise ValueError("Motion cancelled: Stop, Hold or Coast was pressed after it was sent")
                try:
                    self.AdoptScene(Snapshot)
                    self.StartMotionRoute(Path, PlannedStart=Start, PlanSeconds=time.monotonic() - Asked, Note=Note)
                except RouteHeld as Stopped:
                    return False, str(Stopped)
                except Exception as Exc:
                    self.StopMotion(f"{Label} route stopped: {Exc}", Latch=True)
                    return False, self.status
                return True, self.status
        except Exception as Exc:
            return False, str(Exc)

    def move_servo(self, name: str, raw: Any, SceneId=None, SceneRequired=False, SceneCarry=None) -> tuple[bool, str]:
        if name not in self.servos:
            return False, f"Unknown servo: {name}"
        Servo = self.servos[name]
        Target = self._clamp_raw(Servo, _as_int(raw, Servo.present_raw))
        return self.SendJointMotion({name: Target}, name.replace("_", " "), SceneId, SceneRequired, SceneCarry)

    def HomeGripperPlan(self, Targets) -> tuple[tuple[str, int | None], str | None]:
        """How Home brings the gripper to its saved value (Targets' gripper, limited), and a note for the status.
        Called with the lock. ("open", goal): on the way, with the arm; ("close", goal): once the arm has arrived
        (PlanHomeRoute); ("stay", None): the jaws are within their goal tolerance of it; ("hold", None): they hold
        an object, so Home neither opens them (it would drop it) nor closes them further (squeezing it, or a
        stall), and its route keeps their squeeze on (BeginRouteStage), with HOME_GRIPPER_HELD_NOTE.

        Holding an object is the grasp hold (GraspHold): a commanded close that the jaws could not finish because
        something stopped them, squeezed by a bounded 8 steps since. It is the bridge's one sign that something
        is between the jaws now, the one hand guiding keeps the gripper for and its calibration refuses on, and
        the page reads it too (grasp_hold). Every stop that holds keeps it; a new gripper command, a trip, Coast,
        the emergency stop or a disconnect ends it. A close that reached its target on a soft or thin object, or
        a grip left after Coast, is not seen: Home then moves the jaws to their saved value like any other time.
        """
        Servo = self.servos[GRIPPER_NAME]
        if Servo.GraspHold:
            return ("hold", None), HOME_GRIPPER_HELD_NOTE
        Goal = Targets[GRIPPER_NAME]
        Opening = self._slider_track_pos(Servo, Goal) - self._slider_track_pos(Servo, Servo.present_raw)
        if abs(Opening) <= GRIPPER_GOAL_TOLERANCE_RAW:
            return ("stay", None), None
        return ("open" if Opening > 0 else "close", Goal), None

    def go_home(self) -> tuple[bool, str]:
        """Every joint to its saved Home, the wrist roll and the gripper included (PlanHomeRoute)."""
        with self.lock:
            if self.CalibrationActive:
                return False, "Stop calibration before Home"
            # Before Home is read again from disk: a changed preset mid-route ended the route (audit H2).
            Moving = self.MovingRefusal("pressing Home")
            if Moving:
                return False, Moving
            self.home_preset = self._load_home()
            Targets = {Name: self._clamp_raw(self.servos[Name], self.home_preset.get(Name, self.servos[Name].target_raw))
                       for Name in HOME_MOTOR_NAMES}
            if not self.arm_connected or self.bus is None:
                Gripper, Note = self.HomeGripperPlan(Targets)
                try:
                    self.PlanHomeRoute(Targets, Gripper)
                except Exception as Exc:
                    return False, str(Exc)
                for Name, Raw in Targets.items():
                    if Name != GRIPPER_NAME or Gripper[0] != "hold":
                        self.servos[Name].target_raw = Raw
                return True, "Home loaded in preview" + (f"; {Note}" if Note else "")
            if not self.torque_enabled:
                if self.ProtectionTrip:
                    return False, "Protection is latched. Resolve the cause and explicitly enable Hold before Home."
                if QueuedCommandStale():
                    return False, "Home cancelled: Stop, Hold or Coast was pressed after it was sent"
                Ok, Message = self.toggle_torque()
                if not Ok:
                    return False, Message
        return self.SendJointMotion(Targets, "Home", Home=True)

    def set_home(self) -> tuple[bool, str]:
        with self.lock:
            if self.CalibrationActive:
                return False, "Stop calibration before changing Home"
            # Set Home sits between Home and Stop and hold: pressed mid-route it coasted every joint (audit H2).
            Moving = self.MovingRefusal("setting Home")
            if Moving:
                self.status = Moving
                return False, Moving
            if self.SerialLinkLost():
                return False, "Cannot save Home from stale positions: " + self.SerialLinkDownText()
            if self.arm_connected:
                self._read_positions_locked()
            data = {name: int(self.servos[name].present_raw) for name in MOTOR_NAMES}
            try:
                self.home_path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
            except OSError as exc:
                self.status = f"Home save failed: {exc}"
                return False, self.status
            self.home_preset = data
            self.status = "Home saved to backend"
            return True, self.status

    def _save_gui_limits_locked(self) -> None:
        target_limits = self.gui_limits.get("target_limits_cm", {})
        if not isinstance(target_limits, dict):
            target_limits = {}
        data = {
            "slider_limit_mode": "ordered_raw_v2",
            "target_limits_cm": target_limits,
            "motor_raw_limits": {
                name: {
                    "min": servo.range_min,
                    "max": servo.range_max,
                    "arc_inverted": servo.arc_inverted,
                }
                for name, servo in self.servos.items()
            },
        }
        self.limits_path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
        self.gui_limits = data

    def set_servo_limits(self, data: dict[str, Any]) -> tuple[bool, str]:
        with self.lock:
            if self.CalibrationActive:
                return False, "Stop calibration before changing joint limits"
            name = str(data.get("name", ""))
            if name not in self.servos:
                return False, f"Unknown servo: {name}"
            Moving = self.MovingRefusal("changing joint limits")
            if Moving:
                self.status = Moving
                return False, Moving

            servo = self.servos[name]
            lo = max(0, min(RESOLUTION - 1, _as_int(data.get("min"), servo.range_min)))
            hi = max(0, min(RESOLUTION - 1, _as_int(data.get("max"), servo.range_max)))
            if lo == hi:
                self.status = "Slider limits need two different raw values"
                return False, self.status

            servo.range_min = lo
            servo.range_max = hi
            servo.arc_inverted = bool(data.get("arc_inverted", servo.arc_inverted))
            # The hold point stays where it is: pulled into the new arc, the hold correction drove the joint
            # there on its own, unchecked (2026-10-07 audit M8: 150 steps). The next commanded move is limited.
            if servo.gui_angle_raw is not None:
                servo.gui_angle_raw = self._clamp_raw(servo, servo.gui_angle_raw)
            try:
                self._save_gui_limits_locked()
            except OSError as exc:
                self.status = f"Limit save failed: {exc}"
                return False, self.status

            self.status = f"{name.replace('_', ' ')} limits saved" + self.OutsideLimitsNote()
            return True, self.status

    def set_gui_angle(self, data: dict[str, Any]) -> tuple[bool, str]:
        with self.lock:
            if self.CalibrationActive:
                return False, "Stop calibration before overriding joint readings"
            name = str(data.get("name", ""))
            if name not in self.servos:
                return False, f"Unknown servo: {name}"
            Moving = self.MovingRefusal("overriding a joint reading")
            if Moving:
                self.status = Moving
                return False, Moving

            servo = self.servos[name]
            raw = data.get("raw")
            label = name.replace("_", " ")
            if raw is None:
                servo.gui_angle_raw = None
                self.status = f"{label} current angle cleared"
                return True, self.status

            fallback = servo.gui_angle_raw if servo.gui_angle_raw is not None else servo.present_raw
            servo.gui_angle_raw = self._clamp_raw(servo, _as_int(raw, fallback))
            self.status = f"{label} current angle set"
            return True, self.status

    def set_target(self, data: dict[str, Any]) -> tuple[bool, str]:
        with self.lock:
            # It moves the motion epoch, which ends a running route (audit H2): staging waits for the arm.
            Moving = self.MovingRefusal("staging a new target")
            if Moving:
                return False, Moving
            try:
                Values = [float(data[Key]) for Key in ("x", "y", "z", "wrist_pitch") if Key in data]
                if not all(math.isfinite(Value) for Value in Values):
                    raise ValueError("Nonfinite target")
            except (ValueError, TypeError):
                return False, "Target coordinates must be finite numbers"
            Requested = {Axis: float(data.get(Axis, self.target_cm[Axis])) for Axis in ("x", "y", "z")}
            Reason = self.CollisionWorld().TargetReason(Requested)
            if Reason:
                return False, Reason
            self.MotionEpoch += 1
            old = dict(self.target_cm)
            for axis in ("x", "y", "z"):
                try:
                    self.target_cm[axis] = float(data.get(axis, self.target_cm[axis]))
                except (TypeError, ValueError):
                    pass
            previous_wrist_pitch = self.target_wrist_pitch_deg
            try:
                self.target_wrist_pitch_deg = float(data.get("wrist_pitch", self.target_wrist_pitch_deg))
            except (TypeError, ValueError):
                pass
            requested_wrist_pitch = self.target_wrist_pitch_deg
            self.target_wrist_pitch_deg, wrist_pitch_reachable = self._closest_reachable_wrist_pitch(
                requested_wrist_pitch,
                self.target_cm,
                reference_pitch=previous_wrist_pitch,
            )
            self._target_position_changed = any(
                abs(self.target_cm[axis] - old[axis]) > 0.02 for axis in ("x", "y", "z")
            )
            self.status = "Target staged" if wrist_pitch_reachable else "Target staged; wrist pitch adjusted"
            return True, self.status

    def _wrist_pitch_reach_error(
        self,
        pitch_deg: float,
        target_cm: dict[str, float] | None = None,
    ) -> float:
        target = target_cm or self.target_cm
        upper = float(self.arm_dims.get("upper_len_cm", 0.0))
        forearm = float(self.arm_dims.get("forearm_len_cm", 0.0))
        distal = float(self.arm_dims.get("wrist_len_cm", 0.0)) + float(self.arm_dims.get("tip_len_cm", 0.0))
        if upper <= 0 or forearm <= 0 or distal <= 0:
            return 0.0

        radial = math.hypot(target["x"], target["y"]) - float(self.arm_dims.get("base_radial_cm", 0.0))
        z = target["z"] - float(self.arm_dims.get("base_z_cm", 0.0))
        pitch = math.radians(float(pitch_deg))
        wrist_r = radial - distal * math.cos(pitch)
        wrist_z = z - distal * math.sin(pitch)
        reach = math.hypot(wrist_r, wrist_z)
        min_reach = abs(upper - forearm) + 1e-6
        max_reach = upper + forearm - 1e-6
        if min_reach <= reach <= max_reach:
            return 0.0
        return min_reach - reach if reach < min_reach else reach - max_reach

    @staticmethod
    def _closest_equivalent_angle(angle_deg: float, reference_deg: float) -> float:
        return angle_deg + 360.0 * round((reference_deg - angle_deg) / 360.0)

    @staticmethod
    def _angle_delta_abs(a: float, b: float) -> float:
        return abs(_normalize_deg(float(a) - float(b)))

    def _closest_reachable_wrist_pitch(
        self,
        requested_pitch: float,
        target_cm: dict[str, float] | None = None,
        reference_pitch: float | None = None,
    ) -> tuple[float, bool]:
        requested = _normalize_deg(float(requested_pitch))
        if self.arm_geometry.get("model") == "SO101Cad":
            # The exact solver reports failure instead of silently changing pitch.
            return requested, True
        if self._wrist_pitch_reach_error(requested, target_cm) <= 1e-5:
            return requested, True

        # Continuity anchor: among (near-)equally reachable pitches, prefer the
        # one closest to the requested pitch AND to the pitch we are already at.
        # Without the continuity term, a downward pitch just outside reach can
        # resolve to a wrapped "equally close" pitch on the opposite side,
        # physically flipping the wrist up when the user asked for down.
        reference = _normalize_deg(
            float(self.target_wrist_pitch_deg if reference_pitch is None else reference_pitch)
        )

        def score(pitch: float) -> float:
            return (
                self._angle_delta_abs(pitch, requested)
                + 0.5 * self._angle_delta_abs(pitch, reference)
            )

        def pick_best(candidates: list[float]) -> float:
            errors = {pitch: self._wrist_pitch_reach_error(pitch, target_cm) for pitch in candidates}
            best_error = min(errors.values())
            viable = [pitch for pitch in candidates if errors[pitch] <= best_error + 0.5]
            return min(viable, key=score)

        coarse = [self._closest_equivalent_angle(float(p), requested) for p in range(-180, 181)]
        coarse_best = pick_best(coarse)
        fine = [
            self._closest_equivalent_angle(coarse_best + step / 10.0, requested)
            for step in range(-20, 21)
        ]
        return round(pick_best(fine), 1), False

    def _solve_preview(
        self,
        target_cm: dict[str, float],
        wrist_pitch_deg: float,
        elbow_mode: str | None = None,
        *,
        refresh_positions: bool = False,
        allow_unreachable: bool = True,
        # Preview and Send share the CAD position tolerance.
        tolerance_cm: float = IK_SEND_MAX_ERROR_CM,
        max_iterations: int = 30,
        initial_angles: dict[str, float] | None = None,
        reference_pitch: float | None = None,
        continuity: bool = False,
        quick: bool = False,
    ) -> dict[str, Any]:
        if refresh_positions:
            self._read_positions_locked()

        elbow = elbow_mode if elbow_mode in ("up", "down") else self.elbow_mode
        requested_wrist_pitch = float(wrist_pitch_deg)
        # reference_pitch anchors the "closest reachable pitch" search to the
        # pitch the previous preview settled on, so a drag along the table does
        # not flail between e.g. -51 and -61 deg and refold the wrist.
        effective_wrist_pitch, requested_wrist_pitch_reachable = self._closest_reachable_wrist_pitch(
            requested_wrist_pitch,
            target_cm,
            reference_pitch=reference_pitch,
        )
        current = self._current_ik_angles()
        initial = {**current, **(initial_angles or {})}
        for Name in ("wrist_roll", "gripper"):
            initial[Name] = current[Name]
        # continuity: pull the solution toward the previous preview instead of
        # toward the arm's current pose (which is what "preferred" means in a
        # cold solve).
        preferred = initial if (continuity and initial_angles is not None) else current
        World = self.CollisionWorld()
        result = solve_tip_ik(
            self.arm_geometry,
            target_cm["x"],
            target_cm["y"],
            target_cm["z"],
            elbow=elbow,
            joint_limits=self._ik_joint_limits(),
            initial_angles=initial,
            preferred_angles=preferred,
            wrist_pitch_deg=effective_wrist_pitch,
            allow_unreachable=allow_unreachable,
            tolerance_cm=tolerance_cm,
            max_iterations=max_iterations,
            quick=quick,
            # Legacy solver option; CAD ranks travel only after convergence.
            continuity_weight=3e-4 if continuity else None,
            PoseAllowed=lambda Angles: World.PoseReason(Angles) is None,
        )
        angles = result["joint_angles_degrees"]
        joints = self._display_joints(forward_joint_positions(self.arm_geometry, angles))
        error_cm = float(result.get("error_cm", math.nan))
        pitch_error = float(result.get("wrist_pitch_error_deg", math.nan))
        wrist_pitch_reachable = (
            requested_wrist_pitch_reachable
            and
            math.isfinite(pitch_error)
            and pitch_error <= 0.25
        )
        Reason = World.TargetReason(target_cm)
        if not Reason and result.get("reachable"):
            Reason = World.PoseReason(angles)
        return {
            "ok": True,
            "angles": angles,
            "joints": joints,
            "reachable": bool(result.get("reachable", False)) and not Reason,
            "collision_reason": Reason,
            "error_cm": error_cm,
            "requested_wrist_pitch": requested_wrist_pitch,
            "effective_wrist_pitch": wrist_tip_pitch_degrees(self.arm_geometry, angles),
            "wrist_pitch_reachable": wrist_pitch_reachable,
            "elbow_mode": elbow,
            "solved_elbow": result.get("elbow", elbow),
        }

    # Drag previews arrive every ~80 ms. A cold 30-iteration solve from the arm's
    # current pose costs 160-230 ms, so mid-drag the bridge could never keep up
    # and the UI fell back to its rougher client-side solver (which showed red
    # until the drag ended). Two fixes: (1) warm-start from the previous preview
    # when the new target is close to it, which converges in a handful of
    # iterations (~30 ms measured, same answer); (2) solve OUTSIDE the controller
    # lock so previews do not serialise behind each other or the refresh loop.
    PREVIEW_WARM_MAX_DISTANCE_CM = 8.0
    PREVIEW_WARM_MAX_PITCH_DELTA_DEG = 20.0
    PREVIEW_WARM_ITERATIONS = 8
    PREVIEW_COLD_ITERATIONS = 30

    def preview_ik(self, data: dict[str, Any]) -> dict[str, Any]:
        with self.lock:
            target_cm = dict(self.target_cm)
            for axis in ("x", "y", "z"):
                try:
                    target_cm[axis] = float(data.get(axis, target_cm[axis]))
                except (TypeError, ValueError):
                    pass
            try:
                wrist_pitch = float(data.get("wrist_pitch", self.target_wrist_pitch_deg))
            except (TypeError, ValueError):
                wrist_pitch = self.target_wrist_pitch_deg
            elbow_mode = data.get("elbow_mode")
            elbow = elbow_mode if elbow_mode in ("up", "down") else self.elbow_mode
            last = self._last_preview
            warm = None
            if last is not None and last["elbow"] == elbow:
                dist = math.sqrt(sum((target_cm[a] - last["target_cm"][a]) ** 2 for a in ("x", "y", "z")))
                pitch_delta = abs(self._angle_delta_abs(wrist_pitch, last["wrist_pitch"]))
                if dist <= self.PREVIEW_WARM_MAX_DISTANCE_CM and pitch_delta <= self.PREVIEW_WARM_MAX_PITCH_DELTA_DEG:
                    warm = dict(last["angles"])
            Snapshot = self.SolverSnapshot()
            Configuration = self.ConfigurationKey()
        try:
            result = Snapshot._solve_preview(
                target_cm,
                wrist_pitch,
                elbow,
                initial_angles=warm,
                max_iterations=self.PREVIEW_WARM_ITERATIONS if warm is not None else self.PREVIEW_COLD_ITERATIONS,
                reference_pitch=last["effective_pitch"] if warm is not None else None,
                continuity=warm is not None,
                quick=warm is not None,
            )
        except Exception as exc:
            return {
                "ok": False,
                "status": f"IK preview failed: {exc}",
                "target_cm": target_cm,
                "requested_wrist_pitch": wrist_pitch,
                "elbow_mode": self.elbow_mode,
            }
        # A warm solve that failed to converge gets one cold retry so a drag
        # cannot "drag along" a bad local minimum from the previous step. Skipped
        # when the previous preview was already unreachable: dragging around
        # outside the reach envelope would otherwise pay for a full 31-seed
        # sweep on every step just to confirm "still unreachable".
        if warm is not None and not result.get("reachable", False) and last is not None and last.get("reachable", True):
            try:
                cold = Snapshot._solve_preview(
                    target_cm, wrist_pitch, elbow,
                    max_iterations=self.PREVIEW_COLD_ITERATIONS,
                    initial_angles=warm, reference_pitch=last["effective_pitch"],
                    continuity=True, quick=True,
                )
                if cold.get("reachable", False) or float(cold.get("error_cm", math.inf)) < float(result.get("error_cm", math.inf)):
                    result = cold
            except Exception:
                pass
        with self.lock:
            if Configuration != self.ConfigurationKey():
                return {"ok": False, "status": "Configuration changed during IK preview"}
            self._last_preview = {
                "configuration": Configuration,
                "target_cm": dict(target_cm),
                "wrist_pitch": wrist_pitch,
                "elbow": elbow,
                "angles": dict(result["angles"]),
                "reachable": bool(result.get("reachable", False)),
                "effective_pitch": float(result.get("effective_wrist_pitch", wrist_pitch)),
                "solved_elbow": str(result.get("solved_elbow", elbow)),
            }
        result["warm_start"] = warm is not None
        return result

    # Sampling coordinates are the user's Target/Current Position frame.
    # Each solved pose and transition still passes the basket checks.
    # The sampling Z range selects observations; it is not a motion height limit.
    def WorkspaceOffset(self) -> dict[str, float]:
        Frame = _load_json(BACKEND_ROOT / "WorkspaceFrame.json")
        RobotFrame = Frame.get("RobotFrame")
        if RobotFrame:
            Pan = forward_joint_positions(self.arm_geometry, {})[0]
            Base = RobotFrame["ArmBasePositionCm"]
            return {"x": Pan["x"] - Base["x"], "y": Base["y"] - Pan["y"], "z": Base["z"] - Pan["z"]}
        AnchorRaw = self.arm_geometry.get("CadAlignment", {}).get("AnchorRaw", self.home_preset)
        AnchorAngles = {Name: self._ik_angle_from_raw(self.servos[Name], AnchorRaw[Name]) for Name in MOTOR_NAMES}
        Zero = forward_tip_position(self.arm_geometry, AnchorAngles)
        return {"x": float(Frame["HomeDisplayPositionCm"]["x"]) + Zero["x"],
                "y": float(Frame["HomeDisplayPositionCm"]["y"]) - Zero["y"],
                "z": float(Frame["HomeDisplayPositionCm"]["z"]) - Zero["z"]}

    def ModelPointFromVisualPoint(self, Point: dict[str, float]) -> dict[str, float]:
        Offset = self.WorkspaceOffset()
        return {"x": Point["x"] + Offset["x"], "y": Point["y"] - Offset["y"], "z": Point["z"] - Offset["z"]}

    def VisualPointFromModelPoint(self, Point: dict[str, float]) -> dict[str, float]:
        Offset = self.WorkspaceOffset()
        return {"x": Point["x"] - Offset["x"], "y": Point["y"] + Offset["y"], "z": Point["z"] + Offset["z"]}

    def SelectCalibrationCoverage(Self, Candidates, Current, PathSnapshot, Sampling):
        """Preserve spatial coverage in the final path, including preferred Y poses."""
        Remaining = set(range(len(Candidates)))
        Order = []
        Here = Current
        PreferredCount = 0
        MinimumSpacing = float(Sampling.get("MinimumSpacingCm", 0))
        PreferredRange = Sampling.get("PreferredYRangeCm", [27, 33])
        PreferredQuota = int(Sampling.get("PreferredPoseCount", 0))
        PitchCounts = {Pitch: 0 for Pitch in {Candidate.get("wrist_pitch") for Candidate in Candidates}}
        # Every pitch gets at least half of an even share, then travel decides again.
        PitchQuota = int(Sampling["PoseCount"]) // (2 * len(PitchCounts))
        while Remaining and len(Order) < int(Sampling["PoseCount"]):
            Ranked = []
            for Index in Remaining:
                Point = Candidates[Index]["actual_visual"]
                Spacing = min((math.dist(tuple(Point[Axis] for Axis in ("x", "y", "z")),
                                        tuple(Candidates[Prior]["actual_visual"][Axis] for Axis in ("x", "y", "z")))
                               for Prior in Order), default=math.inf)
                if Spacing < MinimumSpacing:
                    continue
                Preferred = PreferredRange[0] <= Point["y"] <= PreferredRange[1]
                Travel = sum(abs(float(Candidates[Index]["angles"][Name]) - float(Here.get(Name, 0)))
                             for Name in IK_JOINTS)
                # Fill the preferred band to its quota, then cover the rest of the workspace:
                # least travel alone kept every pose in whichever band the arm reached first.
                OutOfBand = not Preferred if PreferredCount < PreferredQuota else Preferred
                # Guarantee each wrist pitch a share. At one pitch the tape offset's vertical part
                # is indistinguishable from the camera height, which then biases grasp heights.
                Short = [Pitch for Pitch, Count in PitchCounts.items() if Count < PitchQuota]
                PitchSurplus = int(bool(Short) and Candidates[Index].get("wrist_pitch") not in Short)
                # Minimum spacing preserves coverage even when visiting nearby
                # joints first. Rejected transitions are retried from
                # the next accepted pose, since their starting state has changed.
                Ranked.append((OutOfBand, PitchSurplus, Travel, -Spacing if Order else 0, Index))
            Accepted = False
            for *_, Index in sorted(Ranked):
                Targets = {Name: PathSnapshot._ik_raw_from_angle(PathSnapshot.servos[Name], Candidates[Index]["angles"][Name])
                           for Name in IK_JOINTS}
                try:
                    PathSnapshot.PlanMotionRoute(Targets)
                except ValueError:
                    continue
                Order.append(Index)
                Remaining.remove(Index)
                Here = Candidates[Index]["angles"]
                PreferredCount += int(PreferredRange[0] <= Candidates[Index]["actual_visual"]["y"] <= PreferredRange[1])
                PitchCounts[Candidates[Index].get("wrist_pitch")] += 1
                for Name, Raw in Targets.items():
                    PathSnapshot.servos[Name].present_raw = Raw
                Accepted = True
                break
            if not Accepted:
                break
        return Order

    def selfcal_poses(self, View: str = "Behind") -> dict[str, Any]:
        """Plan the automatic-calibration poses for a camera looking from View.

        A camera behind the arm (the head camera, looking forward and down)
        sees the tip tape only with the gripper tilted up, WristPitchDegrees;
        a camera facing the arm uses the CalibrationSampling.Facing overrides.
        """
        with self.lock:
            current = self._current_ik_angles()
            limits = self._ik_joint_limits()
            elbow = self.elbow_mode
            Configuration = self.ConfigurationKey()
            PathSnapshot = self.SolverSnapshot()
            Sampling = _load_json(BACKEND_ROOT / "WorkspaceFrame.json")["CalibrationSampling"]
            Offset = self.WorkspaceOffset()
        if View == "Facing":
            # A camera facing the arm keeps its own plan: it sees the tape with the gripper
            # tilted down, and its colour-only fit was tuned on nearer, lighter poses.
            Sampling = {**Sampling, **Sampling.get("Facing", {})}
        Pitches = Sampling["WristPitchDegrees"]
        candidates: list[dict[str, Any]] = []
        PreferredRange = Sampling.get("PreferredYRangeCm", [27, 33])
        BasketRejected = 0
        World = PathSnapshot.CollisionWorld()
        Seeds = {}
        SeenTargets = set()
        for VisualX in Sampling["XValuesCm"]:
            if not Sampling["MinimumXCm"] <= VisualX <= Sampling["MaximumXCm"]:
                continue
            for VisualY in Sampling["YValuesCm"]:
                for VisualZ in Sampling["ZValuesCm"]:
                    for pitch in Pitches:
                        x, y, height = VisualX + Offset["x"], VisualY - Offset["y"], VisualZ - Offset["z"]
                        try:
                            result = solve_tip_ik(
                                self.arm_geometry, x, y, height,
                                elbow=elbow, joint_limits=limits,
                                initial_angles=Seeds.get(pitch, current), preferred_angles=current,
                                wrist_pitch_deg=pitch, allow_unreachable=True,
                                tolerance_cm=IK_SEND_MAX_ERROR_CM, max_iterations=40, Scan=True,
                            )
                        except Exception:
                            continue
                        if not result.get("reachable") or float(result.get("wrist_pitch_error_deg", 99.0)) > 3.0:
                            continue
                        Seeds[pitch] = result["joint_angles_degrees"]
                        RoundedAngles = {**current, **{Name: self._ik_angle_from_raw(self.servos[Name],
                                          self._ik_raw_from_angle(self.servos[Name], result["joint_angles_degrees"][Name]))
                                         for Name in IK_JOINTS}}
                        if World.PoseReason(RoundedAngles):
                            BasketRejected += 1
                            continue
                        Actual = forward_tip_position(self.arm_geometry, RoundedAngles)
                        if (math.dist((Actual["x"], Actual["y"], Actual["z"]), (x, y, height)) > IK_SEND_MAX_ERROR_CM
                                or self._angle_delta_abs(wrist_tip_pitch_degrees(self.arm_geometry, RoundedAngles), pitch) > 0.25):
                            continue
                        ActualX, ActualY, ActualZ = Actual["x"] - Offset["x"], Actual["y"] + Offset["y"], Actual["z"] + Offset["z"]
                        if (not Sampling["MinimumXCm"] <= ActualX <= Sampling["MaximumXCm"]
                                or not (ActualY > Sampling.get("MinimumYCm", 10)
                                        and min(Sampling["ZValuesCm"]) <= ActualZ <= max(Sampling["ZValuesCm"]))):
                            continue
                        Signature = tuple(self._ik_raw_from_angle(self.servos[Name], RoundedAngles[Name]) for Name in IK_JOINTS)
                        if Signature in SeenTargets:
                            continue
                        SeenTargets.add(Signature)
                        candidates.append({
                            "x": round(x, 3), "y": round(y, 3), "z": height, "wrist_pitch": pitch,
                            "angles": RoundedAngles,
                            "visual_target": {"x": VisualX, "y": VisualY, "z": VisualZ},
                            "actual_visual": {"x": ActualX, "y": ActualY, "z": ActualZ},
                        })
        if not candidates:
            return {"ok": False, "error": "No reachable calibration poses clear the basket and load limits.",
                    "basket_or_path_rejected": BasketRejected}

        order = self.SelectCalibrationCoverage(candidates, current, PathSnapshot, Sampling)
        if len(order) < SelfCalibration.MinObservations:
            return {"ok": False, "error": "Too few calibration poses have a clear basket route within the load limits.",
                    "safe_candidates": len(candidates), "routable_poses": len(order), "basket_or_path_rejected": BasketRejected}
        poses = [
            {"x": candidates[i]["x"], "y": candidates[i]["y"], "z": candidates[i]["z"],
             "wrist_pitch": candidates[i]["wrist_pitch"], "plan_id": uuid.uuid4().hex,
             "visual_target": candidates[i]["visual_target"]}
            for i in order
        ]
        with self.lock:
            if Configuration != self.ConfigurationKey():
                return {"ok": False, "error": "Workspace configuration changed during calibration planning. Start again."}
            self.CalibrationPlans = {Pose["plan_id"]: {"angles": candidates[Index]["angles"], "configuration": Configuration,
                                                     "prepare_token": self.CalibrationPrepareToken}
                                     for Pose, Index in zip(poses, order)}
        return {"ok": True, "poses": poses, "candidates": len(candidates),
                "sampling": {"minimum_spacing_cm": Sampling.get("MinimumSpacingCm", 0),
                             "preferred_y_range_cm": PreferredRange,
                             "preferred_poses": sum(PreferredRange[0] <= candidates[Index]["actual_visual"]["y"] <= PreferredRange[1] for Index in order)}}

    def toggle_elbow(self) -> tuple[bool, str]:
        with self.lock:
            self.elbow_mode = "up" if self.elbow_mode == "down" else "down"
            self._last_preview = None
            self.status = f"Elbow {self.elbow_mode}"
            return True, self.status

    def send_ik(self, slow: bool = False, Target: dict | None = None, PlanId: str | None = None,
                GripperRaw: Any = None, SceneId=None, SceneRequired=False, SceneCarry=None) -> tuple[bool, str]:
        """A Cartesian move. GripperRaw: a gripper goal that opens the jaws on the way (GRIPPER_WITH_TIP_OPEN_BY)."""
        Asked = time.monotonic()
        try:
            with self.lock:
                if self.CalibrationActive and not PlanId:
                    raise ValueError("Stop calibration before another motion")
                self.MotionPreflight()
                if self.MotionRoute or any(S.command_raw is not None for S in self.servos.values()):
                    raise ValueError("Wait for this motion to finish, or press Stop before another move")
                if GripperRaw is not None and PlanId:
                    raise ValueError("A calibration motion does not move the gripper")
                Gripper = None if GripperRaw is None else self.GripperWithTipTarget(GripperRaw)
                Epoch, Bus, Configuration = self.MotionEpoch, self.bus, self.ConfigurationKey()
                Snapshot = self.PlanningSnapshot()
                StartRaw = {Name: Servo.present_raw for Name, Servo in self.servos.items()}
                Plan = self.CalibrationPlans.get(PlanId) if PlanId else None
                if PlanId and (Plan is None or Plan["configuration"] != Configuration):
                    raise ValueError("Calibration plan is stale. Start calibration again.")
                # ponytail: stamped plans require a live prepare token; legacy white-box injections without the key bypass.
                if PlanId and Plan is not None and "prepare_token" in Plan:
                    if self.CalibrationPrepareToken is None or Plan["prepare_token"] != self.CalibrationPrepareToken:
                        raise ValueError("Start calibration with /selfcal/prepare before plan motion.")
                Requested = dict(self.target_cm) if Target is None else {Axis: float(Target[Axis]) for Axis in ("x", "y", "z")}
                Pitch = self.target_wrist_pitch_deg if Target is None else float(Target.get("wrist_pitch", self.target_wrist_pitch_deg))
                if Plan and Target is None:
                    Requested = forward_tip_position(Snapshot.arm_geometry, Plan["angles"])
                    Pitch = wrist_tip_pitch_degrees(Snapshot.arm_geometry, Plan["angles"])
                if not all(math.isfinite(Value) for Value in [*Requested.values(), Pitch]):
                    raise ValueError("Motion coordinates must be finite")
            # Numerical solving never holds the motor feedback/emergency lock, nor does binding the measured scene
            # (a deep copy of up to ~1400 boxes) or a check against it.
            Snapshot.BindScene(SceneId, SceneRequired, SceneCarry)
            Reason = Snapshot.CollisionWorld().TargetReason(Requested)
            if Reason:
                raise ValueError(Reason)
            if Plan:
                Angles = Plan["angles"]
                slow = True
                Targets, _ = Snapshot.QuantizeCartesianPose(Angles, Requested, Pitch)
                Path = Snapshot.PlanMotionRoute(Targets)
            else:
                Last = Snapshot._last_preview
                Matches = bool(Last and Last.get("configuration") == Configuration
                               and all(abs(Requested[Axis] - Last["target_cm"][Axis]) <= 0.05 for Axis in ("x", "y", "z"))
                               and self._angle_delta_abs(Pitch, Last["wrist_pitch"]) <= 0.15)
                # With a gripper goal the arm's route is planned with the jaws already open, then
                # the opening is blended into it from where they are and certified again.
                Path = (Snapshot if Gripper is None else Snapshot.GripperAtSnapshot(Gripper)).PlanCartesianMotion(
                    Requested, Pitch, InitialAngles=dict(Last["angles"]) if Matches else None,
                    Elbow=Last["solved_elbow"] if Matches else Snapshot.elbow_mode, AssignReach=Gripper is None)
                if Gripper is not None:
                    Path = Snapshot.BlendGripperIntoRoute(Path, Gripper)
            FullAllowance = Snapshot.StartHasAllowance()
            if PlanId:
                SelfCalibration.RecordEvent("BasketRoute", {"plan_id": PlanId, "start_raw": StartRaw,
                    "waypoints_raw": Path, "collision_version": Collision.Model["Version"],
                    "route_planner_version": Collision.PlannerVersion, "configuration": Configuration})
            with self.lock:
                if Epoch != self.MotionEpoch or Bus is not self.bus or Configuration != self.ConfigurationKey():
                    raise ValueError("Motion cancelled: arm state or configuration changed during planning")
                self.MotionPreflight()
                if self.MovedDuringPlanning(StartRaw, FullAllowance):
                    raise ValueError("Arm moved during planning; retry from its new position")
                # Refused before anything is written: a scene that changed or would expire mid-route is no fault.
                Refusal = self.SceneDispatchRefusal(Snapshot, StartRaw, Path)
                if Refusal:
                    raise ValueError(Refusal)
                if QueuedCommandStale():
                    raise ValueError("Motion cancelled: Stop, Hold or Coast was pressed after it was sent")
                self.CalibrationActive = bool(PlanId)
                self.target_cm, self.target_wrist_pitch_deg = Requested, Pitch
                try:
                    self.AdoptScene(Snapshot)
                    self.StartMotionRoute(Path, Calibration=slow, PlannedStart=StartRaw, PlanSeconds=time.monotonic() - Asked)
                except RouteHeld as Held:
                    return False, str(Held)
                except Exception as Exc:
                    self.StopMotion(f"Motion write failed: {Exc}", Latch=True)
                    return False, self.status
                return True, self.status

        except Exception as Exc:
            return False, str(Exc)

    def ArmFrames(self, Angles: dict[str, float]) -> tuple[list[list[float]], list[list[float]] | None]:
        """Display-frame joints (as the app draws them) for IK angles, and the gripper link's 4x4 pose
        in the same frame (None without the CAD model). Geometry only: no servo is read.

        The link's 3x3 carries the display frame's mirrored X, so a point q fixed on the gripper sits
        at R q + t: the roll-aware tape check (RobotReference) predicts the red tape that way.
        """
        Offset = self.WorkspaceOffset()
        Joints = [[Point["x"] + Offset["x"], Point["y"] + Offset["y"], Point["z"] + Offset["z"]]
                  for Point in self._display_joints(forward_joint_positions(self.arm_geometry, Angles))]
        if self.arm_geometry.get("model") != "SO101Cad":
            return Joints, None
        Display = np.diag([-1.0, 1.0, 1.0, 1.0])
        Display[:3, 3] = [Offset["x"], Offset["y"], Offset["z"]]
        return Joints, (Display @ Cad.Forward(self.arm_geometry, Angles)[0]["gripper_link"]).tolist()

    def ArmLinkBoxes(self, Angles: dict[str, float]) -> list[dict[str, Any]] | None:
        """The collision model's link boxes (CollisionModel.json ArmBoxes, the moving jaw at its angle) at IK
        angles, display frame: {"name", "centre", "axes" (columns: the box's axes), "half"} (cm), or None
        without the CAD model; and the mounting collar's whole pan revolution (BasketCollision.CollarSweep),
        where the arm's fixed base and shoulder stand ("fixed": the servo cables hang about it). The measured
        scene leaves the arm's own points out with them (SceneGeometry.Build)."""
        if self.arm_geometry.get("model") != "SO101Cad":
            return None
        Offset = self.WorkspaceOffset()
        Display = np.diag([-1.0, 1.0, 1.0, 1.0])
        Display[:3, 3] = [Offset["x"], Offset["y"], Offset["z"]]
        Links, _ = Cad.Forward(self.arm_geometry, Angles)
        Boxes = []
        for Box in Collision.Model["ArmBoxes"]:
            Transform, Bounds = Display @ Links[Box["Link"]], np.asarray(Box["BoundsCm"], dtype=np.float64)
            if Box["Link"] == "shoulder_link":
                Sweep = Collision.CollarSweep(Bounds)
                Low, High = Display[:3, :3] @ Sweep[0] + Display[:3, 3], Display[:3, :3] @ Sweep[1] + Display[:3, 3]
                Boxes.append({"name": f"{Box['Name']} revolution", "centre": ((Low + High) / 2).round(3).tolist(),
                              "axes": np.eye(3).tolist(), "half": (np.abs(High - Low) / 2).round(3).tolist(), "fixed": True})
            Boxes.append({"name": Box["Name"], "centre": (Transform[:3, :3] @ Bounds.mean(axis=0) + Transform[:3, 3]).round(3).tolist(),
                          "axes": Transform[:3, :3].round(6).tolist(), "half": ((Bounds[1] - Bounds[0]) / 2).round(3).tolist()})
        return Boxes

    def CaptureArmPose(self, CaptureTime: float | None, ReferenceRaw: dict[str, int] | None = None) -> dict[str, Any]:
        """Display-frame joints (as the app draws them) for a capture taken at CaptureTime
        (wall clock), when the arm has held still since; else {"ok": False, "reason"}.

        /locate checks the camera against the gripper's tape and masks the arm out of the
        measurements with this pose, so a pose from after the arm moved must not be used.
        gripper_link is the gripper link's pose from the same angles (ArmFrames). ReferenceRaw,
        raw steps for some servos (the calibration's wrist roll, RobotReference.ReferenceRaw),
        adds "reference": the same pose with those servos there ({"servos_raw", "joints",
        "gripper_link"}), where the roll-aware tape check takes the tape's place on the gripper.
        """
        with self.lock:
            if self.HandGuide is not None:
                # No camera capture while guiding (docs/HandGuide.md D5): a hand moves the arm between the frame and now.
                return {"ok": False, "reason": "hand guiding"}
            if not self.arm_connected or not all(Servo.FeedbackOk for Servo in self.servos.values()):
                return {"ok": False, "reason": "arm not connected"}
            # The joint record runs only while torque is on (RecordMotionTrace): a limp arm
            # moved by hand after the capture would leave no trace.
            if not self.torque_enabled:
                return {"ok": False, "reason": "torque off"}
            if self.MotionRoute is not None or any(Servo.command_raw is not None for Servo in self.servos.values()):
                return {"ok": False, "reason": "arm moving"}
            Raw = {Name: Servo.present_raw for Name, Servo in self.servos.items()}
            Age = max(0.0, time.time() - CaptureTime) if CaptureTime is not None else 0.0
            Now = time.monotonic() - self.MotionTraceStart
            Since = Now - Age - 0.2
            Trace = list(self.MotionTrace)
            Angles = self._current_ik_angles()
            Pinned = {Name: int(Value) for Name, Value in (ReferenceRaw or {}).items() if Name in self.servos}
            PinnedAngles = {**Angles, **{Name: self._ik_angle_from_raw(self.servos[Name], Value) for Name, Value in Pinned.items()}}
        if not Trace or Trace[0]["t"] > Since:
            return {"ok": False, "reason": "no joint record back to the capture"}
        # Unbroken from the last frame before the capture window up to now.
        First = max(Index for Index, Frame in enumerate(Trace) if Frame["t"] <= Since)
        Times = [Frame["t"] for Frame in Trace[First:]] + [Now]
        if max(Later - Earlier for Earlier, Later in zip(Times, Times[1:])) > CAPTURE_TRACE_GAP_S:
            return {"ok": False, "reason": "joint record interrupted since the capture"}
        if any(self._circular_raw_distance(Frame["joints"][Name][0], Raw[Name]) > CAPTURE_STILL_RAW
               for Frame in Trace if Frame["t"] >= Since for Name in MOTOR_NAMES):
            return {"ok": False, "reason": "arm moved since the capture"}
        Joints, Link = self.ArmFrames(Angles)
        Pose = {"ok": True, "joints": Joints, "servos_raw": Raw, "gripper_link": Link}
        Boxes = self.ArmLinkBoxes(Angles)
        if Boxes is not None:
            Pose["link_boxes"] = Boxes  # the arm itself, left out of the measured scene (SceneGeometry.Build)
        if Pinned:
            PinnedJoints, PinnedLink = self.ArmFrames(PinnedAngles)
            Pose["reference"] = {"servos_raw": {**Raw, **Pinned}, "joints": PinnedJoints, "gripper_link": PinnedLink}
        return Pose

    def state_snapshot(self) -> dict[str, Any]:
        with self.lock:
            ik_angles = self._current_ik_angles()
            display_angles = self._display_angles(ik_angles)
            joint_positions = self._display_joints(
                forward_joint_positions(self.arm_geometry, ik_angles)
            )
            return {
                "type": "state",
                "servos": {
                    name: {
                        "feedback_ok": servo.FeedbackOk,
                        "moving": servo.command_raw is not None or self.MotionRoute is not None,
                        "present_raw": servo.present_raw,
                        "target_raw": servo.target_raw,
                        "gui_angle_raw": servo.gui_angle_raw,
                        "range_min": servo.range_min,
                        "range_max": servo.range_max,
                        "arc_inverted": servo.arc_inverted,
                        # Squeezing an object it closed on (HoldGrasp); target_raw follows the jaws.
                        "grasp_hold": servo.GraspHold,
                        # Measured outside its slider limits (hand-posed, or the limits moved): it holds there and
                        # only a commanded, checked move brings it back (audit H3, M8).
                        "outside_limits": not self._contains_raw(servo, servo.present_raw),
                    }
                    for name, servo in self.servos.items()
                },
                "home_raw": dict(self.home_preset),
                "cad_alignment": self.arm_geometry.get("CadAlignment"),
                "cad_source_commit": Cad.Model["SourceCommit"],
                "ik_solver_version": Cad.SolverVersion,
                "ik_solver_source": IkSolverSource,
                "workspace_offset": self.WorkspaceOffset(),
                "safety_version": SafetyVersion,
                # The bridge's code contract with the page (BridgeVersion.json): the page names a bridge running other code.
                "bridge_version": BridgeVersion,
                "scene_version": SceneVersion,
                "robot_reference_version": RobotReferenceVersion,
                "protection_version": ProtectionVersion,
                # Hand guiding (docs/HandGuide.md): on, its phase and roles, how the last session ended, the calibration.
                "hand_guide_version": HandGuideVersion,
                "hand_guide": self.HandGuideSnapshot(),
                "protection": {"enabled": True, "trip": self.ProtectionTrip, "telemetry": self.ProtectionTelemetry,
                               "glitches": list(self.ProtectionGlitches),
                               "temperature_limit_c": ProtectionTemperatureC, "stall_seconds": ProtectionStallSeconds,
                               # A later frame trips while anything moves; a still arm is read again instead.
                               "heartbeat_version": HeartbeatVersion,
                               "heartbeat_limit_ms": round(ControlHeartbeatLimitS * 1000),
                               "control_delays": list(self.ControlDelays),
                               # Late or long control frames and what they waited for (NoteControlStall).
                               "control_stalls": list(self.ControlStalls)},
                # The measured scene the running route was certified against, if any (cleared when the route ends).
                "active_scene_id": (self.ActiveScene or {}).get("id"),
                # The USB serial link: "down" while the bridge reopens a lost port, drop and
                # recovery counts, and the latest drop in plain words (ServiceSerialLink).
                "serial_link": self.SerialLinkSnapshot(),
                "collision_version": Collision.Model["Version"],
                "route_planner_version": Collision.PlannerVersion,
                "route_executor_version": RouteExecutorVersion,
                "speed_control_version": SpeedControlVersion,
                # send_ik takes gripper_raw: an opening the route makes while the arm moves.
                "gripper_with_tip_version": GripperWithTipVersion,
                "motion_speed_maximum_multiplier": MotionSpeedMaximumMultiplier,
                "motion_speed_percent": self.MotionSpeedPercent,
                "minimum_tip_height_cm": None,
                "motion_route": None if self.MotionRoute is None else {"stage": self.MotionRoute["Index"] + 1,
                    "stages": len(self.MotionRoute["Path"])},
                "motion_timeout_ms": self.MotionTimeoutMs,
                "motion_fault": self.MotionFault,
                # The latest fault, trip or abort as recorded (RecordBridgeEvent), kept after a
                # Hold clears motion_fault: kind, reason, joint and the measured values.
                "last_bridge_event": self.LastBridgeEvent,
                "motion_epoch": self.MotionEpoch,
                "configuration_key": self.ConfigurationKey(),
                "torque": self.torque_enabled,
                "arm_connected": self.arm_connected,
                # {"at", "by"} after a Disconnect or Emergency Shutoff, until an explicit Connect: no page connects by
                # itself meanwhile (ConnectRefusal).
                "disconnected_on_purpose": dict(Intent) if (Intent := self.DisconnectIntent) else None,
                "status": self.status,
                "port": self.port,
                "target_cm": dict(self.target_cm),
                "wrist_pitch": self.target_wrist_pitch_deg,
                "elbow_mode": self.elbow_mode,
                "ik_angles": ik_angles,
                "display_angles": display_angles,
                "joint_positions": joint_positions,
                # Joint angle limits in the IK model frame (derived from the
                # slider ranges via the zero pose). The web app's green wrist
                # arc and wheel clamp test pitches against these, so "inside
                # the green" means the bridge will accept it.
                "ik_joint_limits": {name: [lo, hi] for name, (lo, hi) in self._ik_joint_limits().items()},
                "faults": list(self.faults.values()),
                "fault_events": self.fault_events[-5:],
                "control_age_ms": (
                    None if not self.last_refresh_s else round((time.monotonic() - self.last_refresh_s) * 1000.0)
                ),
            }

    def CaptureCadReference(self, Confirmed: bool) -> dict[str, Any]:
        with self.lock:
            if self.HandGuide is not None:
                return {"ok": False, "error": "Hand guiding is on: press Hold here, then Coast, before recording alignment"}
            if not Confirmed:
                return {"ok": False, "error": "Match the physical arm to the displayed CAD reference pose first"}
            if self.CalibrationActive or any(Servo.command_raw is not None for Servo in self.servos.values()):
                return {"ok": False, "error": "Stop motion before recording CAD alignment"}
            if self.torque_enabled:
                return {"ok": False, "error": "Switch to Coast and support the arm before recording alignment"}
            if not self.arm_connected or not self._read_positions_locked():
                return {"ok": False, "error": "Connect the arm and obtain valid joint feedback first"}
            Alignment = copy.deepcopy(self.arm_geometry["CadAlignment"])
            Alignment["ReferenceRaw"] = {Name: Servo.present_raw for Name, Servo in self.servos.items()}
            Alignment["UrdfAtReferenceDegrees"] = dict.fromkeys(MOTOR_NAMES, 0.0)
            Alignment["Verified"] = True
            Alignment["Version"] += 1
            Alignment["Note"] = "Operator recorded the official zero pose; independent Cartesian accuracy has not been measured."
            AlignmentPath = BACKEND_ROOT / "CadAlignment.json"
            BackupPath = BACKEND_ROOT / "CadAlignmentPrevious.json"
            BackupPath.write_text(json.dumps(self.arm_geometry["CadAlignment"], indent=2) + "\n", encoding="utf-8")
            AlignmentPath.write_text(json.dumps(Alignment, indent=2) + "\n", encoding="utf-8")
            self.arm_geometry["CadAlignment"] = Alignment
            self.arm_dims = arm_ik_dimensions(self.arm_geometry)
            self._last_preview = None
            self.CalibrationPlans.clear()
            self.CalibrationPrepareToken = None
            self.MotionEpoch += 1
            for Servo in self.servos.values():
                Servo.gui_angle_raw = None
            self.target_cm = forward_tip_position(self.arm_geometry, self._current_ik_angles())
            self.target_wrist_pitch_deg = wrist_tip_pitch_degrees(self.arm_geometry, self._current_ik_angles())
            self.status = "CAD reference recorded; verify workspace coordinates before motion"
            return {"ok": True, "alignment": Alignment}

    def SetDisconnectIntent(self, OnPurpose: bool, Access: str = "local", Keep: bool = True) -> None:
        """Record (OnPurpose) or clear that the arm was disconnected on purpose. It takes no lock (one reference is
        swapped), so the /ws loop can call it while a connect holds the motor lock. Keep also writes it to disk; a
        coast passes Keep=False and calls KeepDisconnectIntent once the torque is off, so the disk never delays it."""
        self.DisconnectIntent = ({"at": time.strftime("%Y-%m-%dT%H:%M:%S"),
                                  "by": "this PC" if Access == "local" else "another device"} if OnPurpose else None)
        if Keep:
            self.KeepDisconnectIntent()

    def KeepDisconnectIntent(self) -> None:
        """Write the current intent to disk (IntentSink: only the running server has one); failures are printed."""
        if self.IntentSink is None:
            return
        with self.IntentLock:  # the newest value, read under the lock, is what ends up on disk
            try:
                self.IntentSink(self.DisconnectIntent)
            except Exception as Exc:
                print(f"[connect-intent] could not keep it on disk: {Exc}", file=sys.stderr)

    def ConnectRefusal(self, Message: dict[str, Any], Access: str = "local") -> str | None:
        """Why a connect must not run, or None (2026-10-07 audit 2 M2; Aaran's decision: nothing connects the arm or
        turns its torque on by itself from another device). Connect seeds Goal_Position, which energises the arm where
        it is. A page's own Connect click sends "explicit": true; its automatic connect on opening does not."""
        if Message.get("explicit") is True:
            return None
        if Access != "local":
            return ("Another device connects the arm only when its Connect button is pressed: it never connects by "
                    "itself.")
        Intent = self.DisconnectIntent  # read once: SetDisconnectIntent swaps it without a lock
        if Intent:
            return (f"The arm was disconnected on purpose ({Intent['by']}, {Intent['at']}): "
                    "it connects again only when someone presses Connect.")
        return None

    # ------------------------------------------------------------------
    # Hand guiding (docs/HandGuide.md)
    # ------------------------------------------------------------------

    def HandGuideNow(self) -> float:
        """The guide loop's clock: time.perf_counter (time.monotonic ticks every 15.6 ms on Windows, too coarse for the
        law's 40 ms speed estimate at 100 Hz). Tests set their own."""
        return time.perf_counter()

    def HandGuideCalibrationData(self) -> dict | None:
        """The hand-guiding calibration (hand_guide_calibration.json), read again only when the file changed."""
        try:
            Stamp = self.HandGuideCalibrationPath.stat().st_mtime
        except OSError:
            self.HandGuideCalibrationCache = None
            return None
        if self.HandGuideCalibrationCache is None or self.HandGuideCalibrationCache[0] != Stamp:
            self.HandGuideCalibrationCache = (Stamp, HandGuideCalibration.Load(self.HandGuideCalibrationPath))
        return self.HandGuideCalibrationCache[1]

    def HandGuideReferenceRaw(self) -> dict[str, int]:
        return {Name: int(Raw) for Name, Raw in self.arm_geometry["CadAlignment"]["ReferenceRaw"].items()}

    def HandGuideLimits(self, ReferenceRaw: dict[str, int]) -> dict[str, tuple[float, float]]:
        """Each joint's slider arc as URDF angles (rad), its two ends wrapped as the law wraps raw steps."""
        Out = {}
        for Name, Servo in self.servos.items():
            Ends = [((Raw - ReferenceRaw[Name] + 2048) % RESOLUTION - 2048) * HandGuide.STEP_RAD
                    for Raw in (Servo.range_min, Servo.range_max)]
            Out[Name] = (min(Ends), max(Ends))
        return Out

    def HandGuideFingerprint(self, Settings: dict[str, dict], ReferenceRaw: dict[str, int]) -> dict[str, Any]:
        """What a calibration was made with and must still hold: the zero pose and every servo's loop settings."""
        return {"CadAlignmentVersion": self.arm_geometry["CadAlignment"].get("Version"), "ReferenceRaw": ReferenceRaw,
                "Servos": {Name: {Key: Settings[Name][Key] for Key in ("firmware", "model", "p", "d", "i", "punch",
                                                                        "dead_zone", "offset", "mode")}
                           for Name in MOTOR_NAMES}}

    def HandGuideHeartbeat(self, Connection: str) -> None:
        """A page's hand-guiding heartbeat (every 0.5 s while it shows guiding); no lock: one dict entry is replaced."""
        self.HandGuideOwners[Connection] = self.HandGuideNow()

    def HandGuidePageClosed(self, Connection: str) -> None:
        """A page's connection closed. The owner's last heartbeat stays: the supervisor holds 3 s after it."""
        Session = self.HandGuide
        if Session is None or Session.Owner != Connection:
            self.HandGuideOwners.pop(Connection, None)

    def HandGuideReadiness(self, Message: dict[str, Any], Access: str, Connection: str | None, Mode: str):
        """Why a session may not start now (plain words), or None and what the start needs: the bus, every servo's
        fresh readings, status and settings, the fingerprint, the URDF zero pose and the slider arcs (URDF rad). Called
        with the lock; reads only."""
        What = "Hand guiding" if Mode == "guide" else "The hand-guiding calibration"
        if Message.get("explicit") is not True:
            return f"{What} starts only on its own button's click.", None
        if Access not in ("local", "password"):
            return f"{What} needs this PC or a device with the arm password.", None
        if not Connection:
            return f"{What} starts from Clancy's page (its live connection keeps it going).", None
        if not self.arm_connected or self.bus is None:
            return "Connect the arm first.", None
        if self.SerialLinkLost():
            return self.SerialLinkDownText(), None
        if self.ProtectionTrip or self.MotionFault:
            return (self.MotionFault or self.ProtectionTrip["reason"]) + ": resolve it and press Hold first.", None
        if not self.torque_enabled:
            return "Press Hold first: it starts from an arm that holds.", None
        if self.MotionInProgress():  # (a prepared camera calibration is cancelled by the start, as by Stop and hold)
            return "Wait for the arm to stop moving (or press Stop and hold) first.", None
        if self.faults:
            return "A servo reports a fault: " + "; ".join(f"{F['servo']} {'/'.join(F['unload'] + F['warn'])}"
                                                          for F in self.faults.values()), None
        Outside = self.JointsOutsideLimits()
        if Outside:
            return (f"{', '.join(Name.replace('_', ' ') for Name in Outside)} {'is' if len(Outside) == 1 else 'are'} "
                    "outside the slider range: move it back in first."), None
        # still: every joint within HandGuideStillSteps over the last HandGuideStillS (the 30 Hz motion trace)
        Since = time.monotonic() - self.MotionTraceStart - HandGuideStillS  # (the motion trace's clock)
        Trace = [Frame for Frame in self.MotionTrace if Frame["t"] >= Since]
        if len(Trace) < HandGuideStillS * 20 or Trace[0]["t"] > Since + 0.1:
            return "Wait a moment: the arm has to hold still for half a second first.", None
        for Name in MOTOR_NAMES:
            Raws = [Frame["joints"][Name][0] for Frame in Trace]
            if max(self._circular_raw_distance(Raw, Raws[0]) for Raw in Raws) > HandGuideStillSteps:
                return f"Wait for the arm to hold still ({Name.replace('_', ' ')} is moving).", None
        Now = time.time()
        if any(Now - Record.get("at_s", 0.0) < HandGuideNoStallS for Record in (*self.ControlStalls, *self.ControlDelays)):
            return "The control loop paused in the last 10 s: try again in a few seconds.", None
        Bus = GuideBus(self.bus, MOTOR_NAMES)
        try:
            Present = Bus.ReadPresent()
            Status = Bus.ReadStatus()
            Settings = Bus.ReadSettings()
        except PortLostError:
            return "The USB serial link to the arm dropped.", None
        for Name in MOTOR_NAMES:
            Words = Name.replace("_", " ")
            if Present[Name] is None or Status[Name] is None or Settings[Name] is None:
                return f"{Words} did not answer: check its cable.", None
            if not Status[Name]["torque"]:
                return f"{Words} torque is off: press Hold first.", None
            if Status[Name]["lock"] != 1:
                return f"{Words} reads Lock {Status[Name]['lock']} (a restart?): press Coast, then Hold.", None
            if Settings[Name]["mode"] != OperatingMode.POSITION.value:
                return f"{Words} is not in position mode: wait for it to settle, or press Stop and hold.", None
            if not Settings[Name]["p"]:
                return f"{Words} reads a position gain of 0.", None
            if Present[Name]["status"] & sum(1 << Bit for Bit in FAULT_UNLOAD_BITS):
                return f"{Words} reports hardware status {Present[Name]['status']}.", None
        ReferenceRaw = self.HandGuideReferenceRaw()
        Alignment = self.arm_geometry["CadAlignment"]
        # The law reads every joint as wrap(raw - ReferenceRaw) x step (+ toward increasing raw): it must agree with the
        # bridge's own reading of the arm (CadKinematics), or the gravity model would push the wrong way.
        Angles = Cad.ToUrdf({Name: self._ik_angle_from_raw(Servo, Servo.present_raw) for Name, Servo in self.servos.items()},
                            Alignment)
        for Name in MOTOR_NAMES:
            Mine = ((self.servos[Name].present_raw - ReferenceRaw[Name] + 2048) % RESOLUTION - 2048) * HandGuide.STEP_RAD
            if abs((math.degrees(Mine - Angles[Name]) + 180.0) % 360.0 - 180.0) > 0.5:
                return (f"{Name.replace('_', ' ')} reads {math.degrees(Angles[Name]):.1f} deg in the CAD model but "
                        f"{math.degrees(Mine):.1f} deg as hand guiding reads it: its direction or zero changed."), None
        Limits = self.HandGuideLimits(ReferenceRaw)
        for Name in MOTOR_NAMES:
            # Its arc as one URDF interval: an arc across the URDF's +-180 deg would read as its own complement.
            Mine = ((self.servos[Name].present_raw - ReferenceRaw[Name] + 2048) % RESOLUTION - 2048) * HandGuide.STEP_RAD
            if not Limits[Name][0] - 0.01 <= Mine <= Limits[Name][1] + 0.01:
                return (f"{Name.replace('_', ' ')}'s slider range crosses the URDF's 180 deg: hand guiding cannot read its "
                        "limits."), None
        World = self.CollisionWorld(Scene=False)
        if World.MountReason:
            return World.MountReason, None
        return None, {"Bus": Bus, "Present": Present, "Status": Status, "Settings": Settings, "World": World,
                      "ReferenceRaw": ReferenceRaw, "Limits": Limits,
                      "Fingerprint": self.HandGuideFingerprint(Settings, ReferenceRaw)}

    def HandGuideStart(self, Message: dict[str, Any], Access: str = "local", Connection: str | None = None) -> tuple[bool, str]:
        """hand_guide {enable: true, explicit: true}: start hand guiding (docs/HandGuide.md 3.7); {enable: false}: Hold here."""
        Enable = Message.get("enable")
        with self.lock:
            if Enable is False:
                if self.HandGuide is None:
                    return True, "Hand guiding is already off"
                return self.EndHandGuide("preloaded", "Hold here: hand guiding ended, holding where it is")
            if Enable is not True:
                return False, "hand_guide needs enable: true or false"
            if self.HandGuide is not None:
                return self.HandGuide.Mode == "guide", ("Hand guiding is already on" if self.HandGuide.Mode == "guide"
                                                        else "The hand-guiding calibration is running")
            Refusal, Ready = self.HandGuideReadiness(Message, Access, Connection, "guide")
            if Refusal:
                return False, "Hand guiding not started: " + Refusal
            # The walls hold only from a clear pose: a start inside the collision guard band had none until it first
            # read clear (the law now holds such a start where it is; it is refused here first, with the reason).
            Band = self.HandGuideGuardBandIn(Ready["World"])
            if Band is not False:
                return False, ("Hand guiding not started: " + (
                    f"the arm is within {HandGuideGuardPadDeg:.0f} deg of the basket, the cart body, the camera head or the "
                    "rearward limit: move it clear first" if Band else "the collision check could not be run here"))
            Data = self.HandGuideCalibrationData()
            Problems = HandGuideCalibration.FingerprintProblems(Data, Ready["Fingerprint"])
            if Problems:
                return False, "Hand guiding not started: " + "; ".join(Problems)
            # A gripper squeezing an object keeps its squeeze (it is left out of the free joints): free, it would drop it.
            Gripping = self.servos[GRIPPER_NAME].GraspHold
            Free = tuple(Name for Name in HandGuide.FREE_JOINTS if not (Name == GRIPPER_NAME and Gripping))
            try:
                Law = HandGuideCalibration.BuildLaw(Data, Ready["Settings"], Ready["ReferenceRaw"], Ready["Limits"], Free)
            except (KeyError, TypeError, ValueError) as Exc:
                return False, f"Hand guiding not started: the calibration file is incomplete ({Exc}): calibrate again"
            Now = self.HandGuideNow()
            Readings = self.HandGuideReadings(Ready["Present"], Ready["Status"])
            HoldGoals = {Name: float(Ready["Status"][Name]["goal"]) for Name in HandGuide.COMP_JOINTS}
            Command = Law.begin(Now, Readings, HoldGoals)
            if Command.exit_kind:  # refused before any write
                return False, "Hand guiding not started: " + Command.reason.replace("start refused: ", "")
            Joints = (*HandGuide.COMP_JOINTS, *Free)
            Session = HandGuideSession("guide", Law, Ready["Bus"], Joints, Connection, Access, Now, HoldGoals,
                                       World=Ready["World"], Fingerprint=Ready["Fingerprint"], ReferenceRaw=Ready["ReferenceRaw"])
            # Every joint guided gets Acceleration 0 (no profile: the law's goals land as written) and Goal_Speed 0; the
            # free joints keep their hold goals until the start ends and they float.
            Writes = {Name: (Command.writes[Name]["goal"], Command.writes[Name]["limit"]) for Name in Command.writes}
            Writes.update({Name: (float(Ready["Status"][Name]["goal"]), 1000.0) for Name in Free})
            return self.HandGuideBegin(Session, Writes, "Hand guiding starts: keep your hands off the arm for 2 s")

    def HandGuideBegin(self, Session: HandGuideSession, Writes: dict[str, tuple[float, float]], Status: str) -> tuple[bool, str]:
        """Hand the joints of Session to the guide loop: cancel anything queued or planning, end the bridge's own hold
        corrections on them, write the first goals, start the loop. Called with the lock."""
        self.MotionEpoch += 1
        BumpPreemptGeneration()
        self.CalibrationPlans.clear()
        self.CalibrationPrepareToken = None
        for Name in Session.Joints:
            Servo = self.servos[Name]
            Servo.Holding, Servo.homing, Servo.command_raw, Servo.command_speed = False, False, None, 0
        self.HandGuide = Session
        self.HandGuideOwners[Session.Owner] = Session.Since
        self.HandGuideRestoreSpeedAt = 0.0
        self.HandGuideCalibrationResult = None if Session.Mode == "calibrate" else self.HandGuideCalibrationResult
        if Session.Mode == "guide":
            Session.WallVerdict = (self.HandGuideGuardBand(Session), Session.Since)
        try:
            Session.Bus.WriteGoals(Writes, Acceleration=0)
        except PortLostError:
            self.HandGuidePortLost()
            return False, self.status
        Session.LastCycle = self.HandGuideNow()
        Session.Phase = Session.Law.phase if Session.Mode == "guide" else "sweep"
        self.RecordBridgeEvent("hand_guide", f"{'Hand guiding' if Session.Mode == 'guide' else 'Hand-guiding calibration'} on",
                               Details={"roles": self.HandGuideRoles(Session), "owner": Session.Owner,
                                        "access": Session.Access}, Diagnostic=True)
        self.status = Status
        if self.HandGuideThreaded:
            Session.Thread = threading.Thread(target=self.HandGuideLoop, args=(Session,), name="so101-hand-guide", daemon=True)
            Session.Thread.start()
        return True, self.status

    def HandGuideLoop(self, Session: HandGuideSession) -> None:
        """The guide loop's thread: HandGuideCycle every HandGuidePeriodS on deadline scheduling, raised priority."""
        try:
            if sys.platform == "win32":
                import ctypes
                Kernel = ctypes.WinDLL("kernel32", use_last_error=True)
                Kernel.GetCurrentThread.restype = ctypes.c_void_p
                Kernel.SetThreadPriority.argtypes = (ctypes.c_void_p, ctypes.c_int)
                Kernel.SetThreadPriority(Kernel.GetCurrentThread(), 2)  # THREAD_PRIORITY_HIGHEST
        except (OSError, AttributeError):
            pass
        Deadline = time.perf_counter()
        while not Session.Stop.is_set():
            with self.lock:
                if self.HandGuide is not Session:
                    return
                try:
                    self.HandGuideCycle()
                except Exception as Exc:
                    # A cycle that fails leaves nothing it can trust: limp (and the bridge's AutoCoast reads it back).
                    try:
                        self.EndHandGuide("coast", f"The hand-guiding loop failed: {Exc}: every joint goes limp")
                    except Exception as Again:
                        print(f"[hand-guide] cycle failed ({Exc}) and so did its coast ({Again})", file=sys.stderr)
                    return
            Deadline += HandGuidePeriodS
            Remaining = Deadline - time.perf_counter()
            if Remaining > 0:
                time.sleep(Remaining)
            else:
                Deadline = time.perf_counter()  # late: the law judges the gap; never catch up in a burst

    def HandGuideReadings(self, Present: dict[str, dict | None], Status: dict[str, dict | None] | None) -> dict[str, Any]:
        """GuideBus readings as the law's Reading per joint (torque and Lock only on the cycles that read them)."""
        Out = {}
        for Name in MOTOR_NAMES:
            R, S = Present.get(Name), (Status or {}).get(Name)
            if R is None:
                Out[Name] = HandGuide.Reading(ok=False)
                continue
            Out[Name] = HandGuide.Reading(ok=True, pos=float(R["pos"]), load=float(R["load"]), volts=float(R["volts"]),
                                          temp_c=float(R["temp_c"]), status=int(R["status"]),
                                          torque_on=None if S is None else bool(S["torque"]),
                                          lock=None if S is None else int(S["lock"]))
        return Out

    def HandGuideCycle(self) -> None:
        """One cycle of the guide loop, with the motor lock: one SYNC_READ of every servo (and every 4th cycle the torque
        and Lock read-back, or on the next cycle when the last one is suspect: one corrupted reply is not a restart), the
        law (or the calibration's sweep), one SYNC_WRITE of its goals. Every fault the law sees ends it limp
        (EndHandGuide "coast"); a lost USB link latches at once (HandGuidePortLost)."""
        Session = self.HandGuide
        if Session is None:
            return
        Now = self.HandGuideNow()
        Session.Cycle += 1
        try:
            Present = Session.Bus.ReadPresent()
            Status = (Session.Bus.ReadStatus() if Session.Cycle % HandGuideStatusEvery == 0 or Session.StatusRecheck
                      else None)
        except PortLostError:
            self.HandGuidePortLost()
            return
        for Name, Reading in Present.items():
            Servo = self.servos[Name]
            Servo.FeedbackOk = Reading is not None
            if Reading is not None:
                self.ProtectionTelemetry[Name] = {"temperature_c": int(Reading["temp_c"]), "at_s": time.time()}
        Session.Readings = Present
        if Session.Mode == "calibrate":
            self.HandGuideCalibrationCycle(Session, Now, Present, Status)
            return
        Verdict, At = Session.WallVerdict
        Command = Session.Law.step(Now, self.HandGuideReadings(Present, Status), wall_band=Verdict, wall_age=Now - At)
        Session.StatusRecheck = Session.Law.status_recheck
        # the measured pose from the readings the law took (a position it left out as implausible is not the arm's: it
        # would become the guard band's pose, the page's target and, at an exit, the hold's target)
        self.HandGuideTakePositions(Session, {Name: Session.Law.raw[Name] for Name in Session.Law.answering})
        self.HandGuideNotes(Session, Command)
        if Command.exit_kind:
            Words = {"coast": "Hand guiding went limp: ", "anchor": "Hand guiding did not start, holding as before: "}
            self.EndHandGuide(Command.exit_kind, Words.get(Command.exit_kind, "Hand guiding ended: ") + Command.reason,
                              Command=Command)
            return
        try:
            if Command.writes:
                Session.Bus.WriteGoals({Name: (W["goal"], W["limit"]) for Name, W in Command.writes.items()})
            if Command.release:
                Session.Bus.Release(Command.release)
        except PortLostError:
            self.HandGuidePortLost()
            return
        Session.LastCycle = Now

    def HandGuideTakePositions(self, Session: HandGuideSession, Raw: dict[str, float]) -> None:
        """The servos' measured positions from the guide loop's accepted readings (as reported: HardwareRaw keeps the
        servo's own frame); a guided joint's page target follows it."""
        for Name, Value in Raw.items():
            Servo = self.servos[Name]
            Servo.present_raw, Servo.HardwareRaw = int(round(Value)) % RESOLUTION, int(round(Value))
            if Name in Session.Joints:
                Servo.target_raw = Servo.present_raw  # the page's target follows the guided arm

    def HandGuideNotes(self, Session: HandGuideSession, Command) -> None:
        """The law's events worth keeping: its phases, the start's probe, a warm servo (bridge events, and the page)."""
        Session.Phase = Session.Law.phase
        for Event in Command.events:
            if Event in ("ramp", "probe", "guide"):
                self.RecordBridgeEvent("hand_guide", f"Hand guiding phase: {Event}", Diagnostic=True)
            elif Event.startswith("probe:"):
                Joint = Event[6:].split(" ", 1)[0]
                Session.Notes.append(f"{Joint.replace('_', ' ')}: no friction help (it moved under the start's test)")
                self.RecordBridgeEvent("hand_guide", Event, Joint, Diagnostic=True)
            elif Event.startswith("warm:"):
                Joint = Event[5:]
                Session.Notes.append(f"{Joint.replace('_', ' ')} is at 50 C: it goes limp at 55 C. Support the arm or "
                                     "press Hold here")
                self.RecordBridgeEvent("hand_guide", f"{Joint} warm (50 C) while hand guiding", Joint, Diagnostic=True)
        del Session.Notes[:-6]

    def HandGuidePortLost(self) -> None:
        """The USB serial link dropped in a guide cycle. Guiding is a move in progress (MotionInProgress), so the link
        supervisor latches the drop at once: the servos keep their last goals while the port is gone, and every joint
        coasts the moment it is back (auto-coast-v1)."""
        self.ServiceSerialLink()
        if self.HandGuide is not None:
            self.EndHandGuide("coast", "USB serial link to the arm dropped while hand guiding: every joint goes limp")

    def HandGuideGuardBand(self, Session: HandGuideSession) -> bool | None:
        """Whether the measured pose, every joint padded by HandGuideGuardPadDeg, could touch the basket, the cart body,
        the camera head or the rearward limit (the law's guard band); None when it cannot be told (the law then coasts)."""
        return self.HandGuideGuardBandIn(Session.World)

    def HandGuideGuardBandIn(self, World) -> bool | None:
        """HandGuideGuardBand in this collision world (also at the entry, before a session exists)."""
        try:
            Angles = Collision.Np.array([self._ik_angle_from_raw(Servo, Servo.present_raw) for Servo in self.servos.values()])
            return World.IntervalReason(Angles - HandGuideGuardPadDeg, Angles + HandGuideGuardPadDeg) is not None
        except Exception:
            return None

    def HandGuideSupervise(self) -> None:
        """The 30 Hz control frame while a session runs. No bus traffic: the guide loop owns the bus. It watches the guide
        loop (gone, or no cycle for HandGuideHeartbeatS: limp) and the page that started it (silent for
        HandGuideOwnerGoneS: Hold here), judges the collision guard band, and keeps the motion trace and the status."""
        Session = self.HandGuide
        Now = self.HandGuideNow()
        if self.HandGuideThreaded and (Session.Thread is None or not Session.Thread.is_alive()):
            self.EndHandGuide("coast", "The hand-guiding loop stopped: every joint goes limp")
            return
        if Now - Session.LastCycle > HandGuideHeartbeatS:
            self.EndHandGuide("coast", f"The hand-guiding loop paused {(Now - Session.LastCycle) * 1000:.0f} ms: every "
                                       "joint goes limp")
            return
        if Session.Owner is not None and Now - self.HandGuideOwners.get(Session.Owner, 0.0) > HandGuideOwnerGoneS:
            Who = "this PC" if Session.Access == "local" else "another device"
            Reason = (f"{'Hand guiding' if Session.Mode == 'guide' else 'The hand-guiding calibration'} ended, holding "
                      f"where it is: the page that started it ({Who}) closed or went to sleep")
            self.RecordBridgeEvent("motion_abort", Reason)
            self.EndHandGuide("preloaded", Reason)
            return
        if Session.Mode == "guide":
            Session.WallVerdict = (self.HandGuideGuardBand(Session), Now)
        self.RecordMotionTrace()
        self.status = self.HandGuideStatus(Session)

    def HandGuideStatus(self, Session: HandGuideSession) -> str:
        if Session.Mode == "calibrate":
            Run = Session.Law
            Segment = Run.Segments[Run.Index] if 0 <= Run.Index < len(Run.Segments) else None
            return (f"Calibrating hand guiding: {Run.Progress() * 100:.0f} %"
                    + (f" ({Segment.Joint.replace('_', ' ')})" if Segment and Segment.Joint else "")
                    + "; keep clear of the arm. Stop and hold ends it, holding")
        if Session.Phase in ("preload", "ramp", "probe"):
            return "Hand guiding starts: keep your hands off the arm for 2 s"
        return ("Hand guiding: move the arm by hand; Hold here to stop. If anything goes wrong the arm goes limp: keep a "
                "hand on it")

    def HandGuideRoles(self, Session: HandGuideSession) -> dict[str, str]:
        if Session.Mode == "calibrate":
            return {Name: "sweep" for Name in MOTOR_NAMES}
        return {Name: "comp" if Name in HandGuide.COMP_JOINTS else "free" if Name in Session.Joints else "keep"
                for Name in MOTOR_NAMES}

    def EndHandGuide(self, Kind: str, Reason: str, Command=None, Release: bool = True) -> tuple[bool, str]:
        """End the session, if one runs. Kind:
        - "preloaded": Hold here (also Stop and hold, Hold, AI mode, the page gone): every joint held where it is, the
          shoulder, elbow and wrist preloaded with the model's effort, so nothing sags; during the start it is "anchor";
        - "anchor": a failed start: back to the entry hold goals (the arm was never handed over);
        - "coast": every fault: AutoCoast, every joint limp (read back, retried, latched with the reason);
        - "teardown": AutoCoast, emergency stop, Coast or disconnect end it, and their own torque-off follows.
        Both first send one Torque_Enable 0 to all six (GuideBus.Release, Lock stays 1): every joint limp at once, before
        the torque-off that follows, whose own stops (a Goal_Speed write energises these servos) leave alone a joint
        already off (StopSpeedsBeforeTorqueOff). Before, the floating joints were powered toward their stale goals at
        Torque_Limit up to 1000 for 10-25 ms first.
        The holding exits are one SYNC_WRITE (Acceleration 51, Goal_Speed 0, Torque_Limit 1000) read back, then the
        bridge's normal Hold; the profile speed follows HandGuideProfileRestoreS later."""
        with self.lock:
            Session = self.HandGuide
            if Session is None:
                return True, self.status
            if Kind == "preloaded" and Session.Mode == "guide" and Session.Law.phase in ("preload", "ramp", "probe"):
                Kind = "anchor"
            self.HandGuide = None
            Session.Stop.set()
            # the owner's heartbeat entry goes with its session (a heartbeat after this adds it again until that page closes)
            self.HandGuideOwners.pop(Session.Owner, None)
            if Kind in ("coast", "teardown") and Release:
                try:
                    Session.Bus.Release(MOTOR_NAMES)
                except PortLostError:
                    pass  # nothing reaches the servos: the coast is latched and sent when the link returns
            Now = self.HandGuideNow()
            self.HandGuideLast = {"kind": "coast" if Kind == "teardown" else Kind, "reason": Reason, "mode": Session.Mode,
                                  "at_s": round(time.time(), 3), "after_s": round(Now - Session.Since, 1)}
            self.HandGuideAccelerationDirty = True
            self.RecordBridgeEvent("hand_guide", f"{'Hand guiding' if Session.Mode == 'guide' else 'Hand-guiding calibration'}"
                                                 f" ended ({self.HandGuideLast['kind']}): {Reason}",
                                   Details={"after_s": self.HandGuideLast["after_s"], "cycles": Session.Cycle},
                                   Diagnostic=True)
            if Kind == "coast":
                return self.AutoCoast(Reason, Kind="motion_fault")
            if Kind == "teardown":
                return True, self.status
            try:
                Goals = self.HandGuideHoldGoals(Session, Kind, Reason, Command)
                Bad = list(Goals)
                for _ in range(2):
                    Session.Bus.WriteGoals({Name: (Goals[Name], 1000.0) for Name in Bad},
                                           Acceleration=NORMAL_ACCELERATION, GoalSpeed=0)
                    Back = Session.Bus.ReadStatus(Bad)
                    Bad = [Name for Name in Bad if Back[Name] is None or not Back[Name]["torque"]
                           or Back[Name]["goal"] != round(Goals[Name]) or Back[Name]["limit"] != 1000]
                    if not Bad:
                        break
            except PortLostError:
                # Nothing could be sent: the servos keep their guide goals. Latched like any drop while guiding.
                return self.AutoCoast(f"{Reason}; the USB serial link to the arm dropped before the hold was written",
                                      Kind="motion_fault")
            if Bad:
                return self.LatchMotionFault(f"{Reason}; Hold read-back mismatch after hand guiding: {', '.join(Bad)}",
                                             Joint=Bad[0])
            for Name in Goals:
                Servo = self.servos[Name]
                Servo.target_raw = Servo.present_raw
                Servo.Holding, Servo.homing, Servo.command_raw, Servo.command_speed = True, False, None, 0
            self.HandGuideRestoreSpeedAt = time.monotonic() + HandGuideProfileRestoreS  # (the control frame's clock)
            self.status = Reason
            return True, self.status

    def HandGuideHoldGoals(self, Session: HandGuideSession, Kind: str, Reason: str, Command=None) -> dict[str, float]:
        """The goals of a holding exit: the law's own (an exit it decided), else the law's exit from fresh readings; the
        calibration holds every joint where it is now, the lift, elbow and wrist with the URDF prior's gravity effort on
        top (CalibrationRun.HoldHere: the sweep's goal kept a blocked joint pushing into what blocked it). A fresh reading
        is used only where it is plausible; the measured pose (and so the hold's target) follows what was used."""
        Now = self.HandGuideNow()
        if Session.Mode == "calibrate":
            Run = Session.Law
            Goals = Run.HoldHere(Now, Session.Bus.ReadPresent())
            self.HandGuideTakePositions(Session, Run.HeldRaw)
            return {Name: float(Goal) for Name, Goal in Goals.items()}
        if Command is None or not Command.writes:
            Fresh = Session.Bus.ReadPresent()
            Command = HandGuide.Command()
            Session.Law._exit(Command, Kind, Reason, fresh={Name: R for Name, R in self.HandGuideReadings(Fresh, None).items()
                                                             if Name in Session.Joints}, t=Now)
        self.HandGuideTakePositions(Session, {Name: Raw for Name, Raw in Session.Law.held_raw.items() if Name in Session.Joints})
        return {Name: float(Write["goal"]) for Name, Write in Command.writes.items() if Name in Session.Joints}

    def HandGuideRestoreProfile(self) -> None:
        """HandGuideProfileRestoreS after a holding exit: the hold's profile speed, as _stop_slider_velocity_locked leaves it."""
        self.HandGuideRestoreSpeedAt = 0.0
        if not self.arm_connected or self.bus is None or not self.torque_enabled or self.MotionFault:
            return
        Speed = max(1, round(NORMAL_GOAL_SPEED * self.ActiveMotionSpeedPercent() / 100))
        for Name, Servo in self.servos.items():
            if Servo.command_raw is not None or Servo.GraspHold:
                continue
            try:
                self.bus.write("Goal_Speed", Name, Speed, normalize=False, num_retry=1)
            except Exception as Exc:
                self.RaiseIfSerialLinkLost(Exc)
                self.LatchMotionFault(f"{Name} hold profile write failed after hand guiding: {Exc}", Joint=Name,
                                      Details={"error": str(Exc)})
                return

    def HandGuideCalibrate(self, Message: dict[str, Any], Access: str = "local", Connection: str | None = None) -> tuple[bool, str]:
        """hand_guide_calibrate {explicit: true}: the sweeps of docs/HandGuide.md 5 from where the arm holds (each joint
        through a stretch of its range, its excursions collision-checked before anything moves), then the fit, which
        saves hand_guide_calibration.json only if every acceptance check passes. {enable: false} stops it, holding."""
        with self.lock:
            if Message.get("enable") is False:
                if self.HandGuide is None or self.HandGuide.Mode != "calibrate":
                    return True, "The hand-guiding calibration is not running"
                self.HandGuideCalibrationResult = {"ok": False, "error": "stopped", "at_s": round(time.time(), 3)}
                return self.EndHandGuide("preloaded", "Hand-guiding calibration stopped, holding where it is")
            if self.HandGuide is not None:
                return False, "Press Hold here first: hand guiding is on"
            Refusal, Ready = self.HandGuideReadiness(Message, Access, Connection, "calibrate")
            if Refusal:
                return False, "Hand-guiding calibration not started: " + Refusal
            if self.servos[GRIPPER_NAME].GraspHold:
                return False, "Hand-guiding calibration not started: the gripper holds an object: open it first"
            Present, Status = Ready["Present"], Ready["Status"]
            Volts = [Present[Name]["volts"] for Name in MOTOR_NAMES]
            Entry = HandGuide.GuideParams()
            if min(Volts) < Entry.volt_entry_min or max(Volts) - min(Volts) > Entry.volt_spread_max:
                return False, (f"Hand-guiding calibration not started: the servo supply reads {min(Volts):.1f}-{max(Volts):.1f} V "
                               f"(at least {Entry.volt_entry_min:.1f} V, within {Entry.volt_spread_max:.1f} V): check the 12 V "
                               "supply and its cable")
            Hot = [f"{Name.replace('_', ' ')} {Present[Name]['temp_c']:.0f} C" for Name in MOTOR_NAMES
                   if Present[Name]["temp_c"] >= Entry.temp_entry_max_c]
            if Hot:
                return False, f"Hand-guiding calibration not started: {', '.join(Hot)}: let it cool below 50 C"
            ReferenceRaw = Ready["ReferenceRaw"]
            PresentRad = {Name: ((Present[Name]["pos"] - ReferenceRaw[Name] + 2048) % RESOLUTION - 2048) * HandGuide.STEP_RAD
                          for Name in MOTOR_NAMES}
            Segments, Problems = HandGuideCalibration.PlanSweeps(PresentRad, Ready["Limits"], **self.HandGuidePlanOptions())
            if Problems:
                return False, "Hand-guiding calibration not started: " + "; ".join(Problems)
            # Every excursion collision-checked from where the arm holds, before anything moves.
            Start = {Name: Servo.present_raw for Name, Servo in self.servos.items()}
            for Joint, Ends in HandGuideCalibration.PlanExtremes(Segments).items():
                for Deg in Ends:
                    if not Deg:
                        continue
                    Target = {Joint: round(Start[Joint] + Deg * RESOLUTION / 360.0) % RESOLUTION}
                    Begin, Goal, Budget, Relief, Reach = self.EdgeCheckArguments(Start, Target, self.StartRelief(), Budget=64)
                    Why = Ready["World"].EdgeReason(Begin, Goal, Budget=Budget, Relief=Relief, Reach=Reach)
                    if Why:
                        return False, (f"Hand-guiding calibration not started: sweeping {Joint.replace('_', ' ')} {Deg:+.0f} deg "
                                       f"from here: {Why}. Move the arm to a clear, open pose first")
            HoldGoals = {Name: float(Status[Name]["goal"]) for Name in MOTOR_NAMES}
            Run = HandGuideCalibration.CalibrationRun(Segments, HoldGoals, {Name: float(Present[Name]["pos"]) for Name in MOTOR_NAMES},
                                                      ReferenceRaw=ReferenceRaw, Settings=Ready["Settings"])
            Session = HandGuideSession("calibrate", Run, Ready["Bus"], MOTOR_NAMES, Connection, Access, self.HandGuideNow(),
                                       HoldGoals, World=Ready["World"], Fingerprint=Ready["Fingerprint"],
                                       ReferenceRaw=ReferenceRaw)
            Minutes = Run.Total / 60.0
            return self.HandGuideBegin(Session, {Name: (Goal, 1000.0) for Name, Goal in HoldGoals.items()},
                                       f"Calibrating hand guiding (about {Minutes:.0f} min): keep clear of the arm")

    def HandGuidePlanOptions(self) -> dict[str, Any]:
        """PlanSweeps options (tests shorten the calibration here)."""
        return {}

    def HandGuideCalibrationCycle(self, Session: HandGuideSession, Now: float, Present: dict[str, dict | None],
                                  Status: dict[str, dict | None] | None) -> None:
        """One cycle of the calibration's sweeps. Faults as on a route: a firmware protection bit, the supply under
        7.0 V, a servo silent for 5 cycles or with its torque off (or Lock 0) unasked: limp, each once it is read twice in
        a row (one corrupted reply is not a fault; the status registers are read again on the next cycle); 55 C for over
        temp_exit_hold_s or a joint off its path: it stops, holding where it is. Done: the fit, saved only if accepted
        (HandGuideCalibrationResult says what it found)."""
        Run = Session.Law
        Answering = {Name: R for Name, R in Present.items() if R is not None}
        Session.Misses = 0 if len(Answering) == len(Present) else Session.Misses + 1
        if Session.Misses >= 5:
            Silent = [Name for Name, R in Present.items() if R is None]
            self.EndHandGuide("coast", f"Hand-guiding calibration: {', '.join(Silent)} stopped answering: every joint goes limp")
            return
        if len(Answering) < len(Present):
            Session.LastCycle = Now  # a missed reply or two: the servos keep their goals
            return
        Unload = sum(1 << Bit for Bit in FAULT_UNLOAD_BITS)
        Params = HandGuide.GuideParams()

        def Persists(Check: str, Name: str, Bad: bool) -> bool:
            Count = Session.Suspect.get((Check, Name), 0) + 1 if Bad else 0
            Session.Suspect[(Check, Name)] = Count
            return Count >= Params.persist_reads
        for Name, R in Answering.items():
            Words = Name.replace("_", " ")
            S = (Status or {}).get(Name)
            if Persists("status", Name, bool(R["status"] & Unload)):
                self.EndHandGuide("coast", f"Hand-guiding calibration: {Words} firmware protection ({R['status']}): every joint goes limp")
                return
            if S is not None and Persists("torque", Name, Name not in Run.Released and not S["torque"]):
                self.EndHandGuide("coast", f"Hand-guiding calibration: {Words} torque went off (a restart?): every joint goes limp")
                return
            if S is not None and Persists("lock", Name, S["lock"] == 0):
                self.EndHandGuide("coast", f"Hand-guiding calibration: {Words} restarted (its Lock register reads 0): every "
                                           "joint goes limp")
                return
            if Persists("volts", Name, R["volts"] < Params.volt_min):
                self.EndHandGuide("coast", f"Hand-guiding calibration: the supply fell to {R['volts']:.1f} V: every joint goes limp")
                return
            if R["temp_c"] >= Params.temp_exit_c:
                # Only a reading that lasts: one corrupted status packet read 57-62 C on a 27 C wrist (2026-10-10).
                if Now - Session.HotSince.setdefault(Name, Now) > Params.temp_exit_hold_s:
                    self.HandGuideCalibrationResult = {"ok": False, "error": f"{Words} reached {R['temp_c']:.0f} C", "at_s": round(time.time(), 3)}
                    self.EndHandGuide("preloaded", f"Hand-guiding calibration stopped, holding: {Words} reached {R['temp_c']:.0f} C "
                                                   f"for over {Params.temp_exit_hold_s * 1000:.0f} ms")
                    return
            else:
                Session.HotSince.pop(Name, None)
        Session.StatusRecheck = any(Count for (Check, _), Count in Session.Suspect.items() if Check in ("torque", "lock"))
        Command = Run.Step(Now, Answering)
        self.HandGuideTakePositions(Session, Run.Raw)
        if Command["error"]:
            self.HandGuideCalibrationResult = {"ok": False, "error": Command["error"], "at_s": round(time.time(), 3)}
            self.EndHandGuide("preloaded", "Hand-guiding calibration stopped, holding: " + Command["error"])
            return
        if Command["done"]:
            Data, Problems, Report = HandGuideCalibration.Fit(Run.Samples, Run.SpinDowns, Run.Volts, Session.Fingerprint,
                                                              Session.ReferenceRaw)
            if Data is not None:
                try:
                    HandGuideCalibration.Save(self.HandGuideCalibrationPath, Data)
                except OSError as Exc:
                    Data, Problems = None, [f"could not save {self.HandGuideCalibrationPath.name}: {Exc}"]
            self.HandGuideCalibrationResult = {"ok": Data is not None, "problems": Problems, "report": Report,
                                               "at_s": round(time.time(), 3)}
            self.RecordBridgeEvent("hand_guide", "Hand-guiding calibration " + ("saved" if Data else "not accepted: " + "; ".join(Problems)),
                                   Details={"report": Report}, Diagnostic=True)
            self.EndHandGuide("preloaded", "Hand-guiding calibration " + ("finished and saved: hand guiding is ready" if Data else
                                                                          "finished but not accepted: " + "; ".join(Problems)))
            return
        try:
            if Command["writes"]:
                Session.Bus.WriteGoals(Command["writes"])
            if Command["release"]:
                Session.Bus.Release(Command["release"])
        except PortLostError:
            self.HandGuidePortLost()
            return
        Session.LastCycle = Now

    def ServoDump(self) -> dict[str, Any]:
        """Every servo's registers 0..70, read only (docs/HandGuide.md G2, for the first hardware session's step S1): one
        SYNC_READ, decoded by name, Present_Load with its sign in bit 10. Never writes; refused while a session owns the bus."""
        with self.lock:
            if not self.arm_connected or self.bus is None or self.SerialLinkLost():
                return {"ok": False, "error": "Connect the arm first"}
            if self.HandGuide is not None:
                return {"ok": False, "error": "Hand guiding owns the servo bus: press Hold here first"}
            try:
                Replies = GuideBus(self.bus, MOTOR_NAMES).SyncRead(0, 71, DeadlineS=0.05)
            except PortLostError as Exc:
                return {"ok": False, "error": str(Exc)}
        Out = {}
        for Name, Reply in Replies.items():
            if Reply is None:
                Out[Name] = None
                continue
            Error, Data = Reply
            Word = lambda At: Data[At] | (Data[At + 1] << 8)
            Named = {Register: (Data[At] if Length == 1 else Word(At)) for Register, (At, Length) in
                     sorted(CONTROL_TABLE.items(), key=lambda Item: Item[1][0]) if At + Length <= len(Data)}
            for Register in ("Goal_Position", "Present_Position", "Goal_Speed", "Present_Speed", "Present_Current"):
                Named[Register] = GuideBusDecode(Named[Register], 15)
            Named["Present_Load"] = GuideBusDecode(Named["Present_Load"], 10)
            Named.update(P=Data[21], D=Data[22], I=Data[23], Punch=Data[24], Dead_Zone_CW=Data[26], Dead_Zone_CCW=Data[27],
                         Firmware=f"{Data[0]}.{Data[1]}", Present_Voltage_V=Data[62] / 10.0)
            Out[Name] = {"error_byte": Error, "registers": Named, "raw": list(Data)}
        return {"ok": True, "servos": Out}

    def HandGuideSnapshot(self) -> dict[str, Any]:
        """The state's hand_guide: whether a session runs and how, how the last one ended, and the calibration."""
        Session = self.HandGuide
        Data = self.HandGuideCalibrationData()
        Out: dict[str, Any] = {"version": HandGuideVersion, "on": Session is not None and Session.Mode == "guide",
                               "calibrating": Session is not None and Session.Mode == "calibrate",
                               "last_exit": self.HandGuideLast, "calibrated": Data is not None,
                               "calibrated_at": (Data or {}).get("CalibratedAt"),
                               "coast_brakes": (Data or {}).get("CoastBrakes"),
                               # what the page shows of the last calibration (its full report is in bridge_events.jsonl)
                               "calibration_result": None if self.HandGuideCalibrationResult is None else {
                                   Key: self.HandGuideCalibrationResult.get(Key) for Key in ("ok", "problems", "error", "at_s")}}
        if Session is None:
            return Out
        Now = self.HandGuideNow()
        Out.update(phase=Session.Phase, since_s=round(Now - Session.Since, 1), roles=self.HandGuideRoles(Session),
                   owner="this PC" if Session.Access == "local" else "another device",
                   notes=list(Session.Notes))
        if Session.Mode == "calibrate":
            Out["progress"] = round(Session.Law.Progress(), 3)
            return Out
        Law = Session.Law
        Out["start_left_s"] = (round(max(0.0, Law.p.preload_s + Law.p.ramp_s + 2 * Law.p.probe_s + Law.p.probe_settle_s
                                         - (Now - Session.Since)), 1) if Law.phase in ("preload", "ramp", "probe") else 0.0)
        Out["free_states"] = dict(Law.fstate)
        Out["assist"] = {Name: {"on": bool(Law.ff_on.get(Name)),
                                **({"band_top_deg_s": round(math.degrees(Law.band_top(Name, Law.volts)), 1)}
                                   if Name in Law.fcfg else {})} for Name in Law.joints}
        return Out

    def handle_command(self, message: dict[str, Any], Access: str = "local", Connection: str | None = None) -> tuple[bool, str]:
        """Run one command. Access is who sent it (RemoteAccess: "local" for this PC and the bridge's own callers,
        "password" for another device): it decides whether a connect may be automatic. Connection: the page connection
        it came on (the /ws loop's id), which a hand-guiding session keeps as its owner."""
        command = str(message.get("type", ""))
        if command == "stop_motion":
            return self.StopMotion()
        if command == "hand_guide":
            return self.HandGuideStart(message, Access, Connection or getattr(_CommandContext, "Connection", None))
        if command == "hand_guide_calibrate":
            return self.HandGuideCalibrate(message, Access, Connection or getattr(_CommandContext, "Connection", None))
        if command == "connect":
            Refusal = self.ConnectRefusal(message, Access)
            if Refusal:
                return False, Refusal
            Ok, Text = self.connect(str(message.get("port", "")))
            if Ok and message.get("explicit") is True and self.DisconnectIntent:
                self.SetDisconnectIntent(False, Access)
            return Ok, Text
        if command == "disconnect":
            # First, so an automatic connect racing this one (another page seeing the arm go) is refused already; on
            # disk once the arm coasts.
            self.SetDisconnectIntent(True, Access, Keep=False)
            try:
                return self.disconnect(disable_torque=True)
            finally:
                self.KeepDisconnectIntent()
        if command == "set_motion_speed":
            return self.SetMotionSpeed(message.get("percent"))
        if command == "move_servo":
            return self.move_servo(str(message.get("name", "")), message.get("raw"), message.get("scene_id"),
                                   message.get("scene_required") is True, message.get("scene_carry"))
        if command == "toggle_torque":
            return self.toggle_torque(message.get("enable"))
        if command == "go_home":
            return self.go_home()
        if command == "set_home":
            return self.set_home()
        if command == "set_servo_limits":
            return self.set_servo_limits(message)
        if command == "set_gui_angle":
            return self.set_gui_angle(message)
        if command == "set_target":
            return self.set_target(message)
        if command == "send_ik":
            return self.send_ik(slow=bool(message.get("slow", False)), Target=message.get("target"), PlanId=message.get("plan_id"),
                                GripperRaw=message.get("gripper_raw"), SceneId=message.get("scene_id"),
                                SceneRequired=message.get("scene_required") is True, SceneCarry=message.get("scene_carry"))
        if command == "toggle_elbow":
            return self.toggle_elbow()
        return False, f"Unknown command: {command}"


controller = BridgeController()

# Stop, Hold, Coast, disconnect and emergency stop take the arm out of motion. They run at once, ahead of any queued
# command on any connection, and every command queued before them is dropped unrun: "Stop and hold" waited behind a
# route that was still planning, and the route started, then stopped (2026-10-07 audit M2). A queued Home or move
# must not start after the user pressed Coast or Stop.
_PreemptLock = threading.Lock()
_PreemptGeneration = 0


def Preempts(Message: dict[str, Any]) -> bool:
    """Whether a command takes the arm out of motion and so runs ahead of the queue. toggle_torque only with an
    explicit enable (the new page sends which button it showed): a bare toggle keeps its place in the queue, where
    it cannot overtake the command whose result it was meant to follow."""
    Command = Message.get("type")
    return Command in ("stop_motion", "disconnect", "emergency_stop") or (
        Command == "toggle_torque" and isinstance(Message.get("enable"), bool)) or (
        Command in ("hand_guide", "hand_guide_calibrate") and Message.get("enable") is False)


def BumpPreemptGeneration() -> int:
    """A preempting command arrived: every command queued before now is stale."""
    global _PreemptGeneration
    with _PreemptLock:
        _PreemptGeneration += 1
        return _PreemptGeneration


def PreemptGeneration() -> int:
    with _PreemptLock:
        return _PreemptGeneration


# The preempt generation a queued WebSocket command was sent under, for the thread that runs it (RunQueuedCommand):
# a motion checks it once more under the motor lock before its route starts, so a Stop that lands between the queue's
# own check and the motion's start still cancels it (the motion epoch alone would not: the Stop could move it first).
_CommandContext = threading.local()


def QueuedCommandStale() -> bool:
    """Whether the command this thread runs (from the /ws queue or POST /motion) arrived before a preempting command
    (never true outside one: the bridge's own callers)."""
    Generation = getattr(_CommandContext, "Generation", None)
    return Generation is not None and Generation != PreemptGeneration()


def RunQueuedCommand(Message: dict[str, Any], Generation: int, Access: str = "local",
                     Connection: str | None = None) -> tuple[bool, str]:
    """controller.handle_command for a command from the /ws queue or POST /motion, sent under preempt Generation by Access
    on Connection (the /ws loop's id; None for POST /motion)."""
    _CommandContext.Generation = Generation
    _CommandContext.Connection = Connection  # handle_command reads it for hand guiding's owner
    try:
        return controller.handle_command(Message, Access)
    finally:
        _CommandContext.Generation = None
        _CommandContext.Connection = None


class ControlLoop:
    """Runs the closed-loop velocity controller at a fixed rate, always.

    It used to run only inside each WebSocket client's loop. With no client
    connected, or a client busy handling a command, a joint left in velocity
    mode kept its last speed until it hit a mechanical stop and the servo's
    overload protection dropped the arm. A dedicated thread removes that
    dependency; WebSocket clients now only receive state snapshots.
    """

    def __init__(self, target: BridgeController, period_s: float) -> None:
        self.target = target
        self.period_s = period_s
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        # Where a late frame's stack dump goes (StallDumpFile); None: no dumps (offline imports and tests).
        self.StallDumps: Callable[[], Any] | None = None
        # When the stall dump was last armed (time.monotonic; 0: not since this loop started).
        self.LastArmed = 0.0

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="so101-control-loop", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    def _run(self) -> None:
        # Deadline scheduling with time.sleep: on Windows, Event.wait rounds up
        # to the ~15.6 ms system tick (30 Hz became 21 Hz), while time.sleep uses
        # a high-resolution timer on Python 3.11+.
        deadline = time.perf_counter()
        self.LastArmed = 0.0
        while not self._stop.is_set():
            self.ArmStallDump()
            try:
                self.target.refresh()
            except Exception as Exc:
                # The loop itself must never die: with it gone, a joint in velocity mode keeps its last speed and
                # nothing supervises it (2026-10-07 audit S4). A coast that fails too is tried again next frame.
                try:
                    if getattr(self.target, "torque_enabled", False):
                        self.target.AutoCoast(f"Motor control loop failed: {Exc}")
                except Exception as Again:
                    print(f"[control-loop] frame failed ({Exc}) and so did its coast ({Again}); retrying", file=sys.stderr)
            deadline += self.period_s
            remaining = deadline - time.perf_counter()
            if remaining > 0:
                time.sleep(remaining)
            else:
                deadline = time.perf_counter()  # fell behind: do not try to catch up in a burst
        try:
            faulthandler.cancel_dump_traceback_later()
        except Exception:
            pass

    def ArmStallDump(self) -> None:
        """Have every thread's stack written if no frame finishes within ControlStallDumpS from now.

        faulthandler's own watchdog thread writes it, so it works while another thread holds the interpreter
        lock, which no Python-level watchdog could see past (a cause the 2026-10-04 stall's record could not rule
        out). Re-armed every frame, so it fires only on a stall. A dump has no time of its own: the first arming
        after one writes a line under it, when the stall ended and how long no frame finished, to match it with
        its control_stall record in bridge_events.jsonl."""
        if self.StallDumps is None:
            return
        try:
            File = self.StallDumps()
            Now = time.monotonic()
            if File is not None and self.LastArmed and Now - self.LastArmed >= ControlStallDumpS:
                File.write(f"# {time.strftime('%Y-%m-%dT%H:%M:%S')}: the stack dump above; no control frame finished for "
                           f"{round((Now - self.LastArmed) * 1000)} ms (its control_stall record is in bridge_events.jsonl)\n")
                File.flush()
            self.LastArmed = Now
            if File is not None:
                faulthandler.dump_traceback_later(ControlStallDumpS, repeat=False, file=File)
        except Exception as Exc:
            print(f"[control-loop] stall dump not armed: {Exc}", file=sys.stderr)


class StallDumpFile:
    """captures/<date>/recording/control_stalls.log, opened per day for faulthandler (which needs an open file):
    a header line when it opens, faulthandler's stacks, and under each dump its time (ControlLoop.ArmStallDump)."""

    def __init__(self) -> None:
        self.Day: str | None = None
        self.File = None

    def __call__(self):
        Day = time.strftime("%Y-%m-%d")
        if Day != self.Day or self.File is None:
            if self.File is not None:
                try:
                    # A dump still armed on yesterday's file must not write to its closed (perhaps reused) descriptor.
                    faulthandler.cancel_dump_traceback_later()
                    self.File.close()
                except Exception:
                    pass
            Directory = SessionLog.RecordingDirectory(time.time())
            Directory.mkdir(parents=True, exist_ok=True)
            self.File = open(Directory / CONTROL_STALLS_LOG, "a", encoding="utf-8")
            self.File.write(f"# control-stall stack dumps (bridge pid {os.getpid()}), opened {time.strftime('%Y-%m-%dT%H:%M:%S')}: "
                            f"each dump is every thread's stack {ControlStallDumpS:g} s into a frame that had not finished\n")
            self.File.flush()
            self.Day = Day
        return self.File


control_loop = ControlLoop(controller, BRIDGE_FRAME_S)
# The Orbbec depth camera belongs to the bridge (one process can stream it).
# SO101_DEPTH_CAMERA=0 leaves it closed, e.g. for another program to use it.
depth_camera = OrbbecCamera.DepthCamera()
# No camera open attempt (1-2 s with the interpreter lock held) while the arm needs the control loop.
depth_camera.Quiet = controller.MotionInProgress


def RaiseProcessPriority() -> str:
    """Run the bridge above normal work: the 30 Hz servo loop must not wait for the CPU.

    Measured 2026-09-28: with every core busy (a batch video analysis), status packets
    arrived after 37-180 ms (timeout 34 ms) and one read took 825 ms, next to the 750 ms
    heartbeat that coasts a moving arm. At High priority the same load gave no slow reads.
    """
    try:
        if sys.platform == "win32":
            import ctypes
            Kernel = ctypes.WinDLL("kernel32", use_last_error=True)
            Kernel.GetCurrentProcess.restype = ctypes.c_void_p  # a 64-bit pseudo handle
            Kernel.SetPriorityClass.argtypes = (ctypes.c_void_p, ctypes.c_uint32)
            if Kernel.SetPriorityClass(Kernel.GetCurrentProcess(), 0x00000080):  # HIGH_PRIORITY_CLASS
                return "high"
            return "normal"
        os.nice(-5)
        return "raised"
    except (OSError, AttributeError):
        return "normal"


@asynccontextmanager
async def _lifespan(_app: FastAPI):
    RaiseProcessPriority()
    # Only the running server keeps faults, trips and aborts on disk (AppendBridgeEvent), and stack dumps of
    # stalled control frames (StallDumpFile).
    controller.EventSink = AppendBridgeEvent
    controller.TimingSink = AppendBridgeEvent
    # Other devices refused or let in (RemoteAccess), and whether the arm was disconnected on purpose, kept on disk.
    RemoteAccess.EventSink = AppendBridgeEvent
    controller.DisconnectIntent = RemoteAccess.LoadIntent()
    controller.IntentSink = RemoteAccess.SaveIntent
    control_loop.StallDumps = StallDumpFile()
    control_loop.start()
    depth_camera.Start()
    # Planning's collision checks, side by side in their own processes (RouteCheckPool). Until they
    # answer, and if they never do, the bridge checks in process as before.
    RouteChecks.Start()
    try:
        yield
    finally:
        await asyncio.to_thread(controller.emergency_stop)
        control_loop.stop()
        await asyncio.to_thread(depth_camera.Stop)
        await asyncio.to_thread(FlushBridgeEvents, 2.0)


app = FastAPI(title="SO-101 Backend Bridge", lifespan=_lifespan)
# Who may use the bridge (RemoteAccess): this PC as before; another device (LAN, Tailscale) only with the remote
# password; a web page from any other site never. Added first, so CORS (added next) wraps it and another device's page
# can read a refusal's reason.
app.add_middleware(RemoteAccess.Middleware)
app.add_middleware(
    CORSMiddleware,
    # Clancy's page on whatever host the bridge is reached by (127.0.0.1, a LAN or Tailscale address), on a dashboard
    # port. It used to admit only 192.168.x and 10.x pages (not the tailnet's 100.x); RemoteAccess decides who gets in.
    allow_origin_regex=RemoteAccess.CorsOriginPattern(),
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    expose_headers=["X-Frame-Seq", "X-Depth-Range-Cm"],
)


@app.get("/motion_trace")
def motion_trace() -> dict[str, Any]:
    """Recent control frames: command speeds and feedback at the control rate (diagnostics)."""
    with controller.lock:
        Frames = list(controller.MotionTrace)
    return {"ok": True, "frame_hz": round(1 / BRIDGE_FRAME_S), "frames": Frames}


@app.get("/servo/dump")
def ServoDump() -> dict[str, Any]:
    """Every servo's registers 0..70, read only (BridgeController.ServoDump)."""
    return controller.ServoDump()


@app.get("/bus/stats")
def BusStats() -> dict[str, Any]:
    """Servo bus transaction counts and recent failures, for diagnosing dropped status packets."""
    Stats = getattr(controller.bus, "BusStats", None)
    if Stats is None:
        return {"ok": False, "error": "Arm not connected."}
    return {"ok": True, **{K: V for K, V in Stats.items() if K != "recent_failures"},
            "max_ok_ms": round(Stats["max_ok_ms"], 1), "recent_failures": list(Stats["recent_failures"])}


@app.get("/health")
def health(request: Request = None) -> dict[str, Any]:  # request is None when called directly (tests)
    return {"service": "so101-backend-bridge", "ok": True, "safety_version": SafetyVersion,
            # How this caller got in (RemoteAccess: "local", "password" or "open") and whether other devices can.
            "access": request.scope.get(RemoteAccess.AccessKey) if request else None, "access_version": AccessVersion,
            "remote_access": "password" if RemoteAccess.PasswordSet() else "off",
            "bridge_version": BridgeVersion, "scene_version": SceneVersion, "robot_reference_version": RobotReferenceVersion,
            "protection_version": ProtectionVersion, "serial_link_version": SerialLinkVersion,
            "heartbeat_version": HeartbeatVersion, "hand_guide_version": HandGuideVersion,
            "speed_control_version": SpeedControlVersion,
            "motion_speed_maximum_multiplier": MotionSpeedMaximumMultiplier,
            "collision_version": Collision.Model["Version"],
            "route_planner_version": Collision.PlannerVersion,
            "route_executor_version": RouteExecutorVersion,
            "minimum_tip_height_cm": None,
            "calibration_version": "archive-v2", "cad_version": "so101-urdf-v1",
            "depth_camera_version": DEPTH_CAMERA_VERSION,
            "cad_source_commit": Cad.Model["SourceCommit"], "ik_solver_version": Cad.SolverVersion,
            "ik_solver_source": IkSolverSource, "ik_solver_source_stale": IkSolverSourceStale(),
            "route_check_workers": RouteChecks.WorkerCount()}


@app.post("/emergency-stop")
def EmergencyStop(request: Request = None) -> dict[str, Any]:  # request is None when called directly (tests)
    BumpPreemptGeneration()
    # A page's Emergency Shutoff (a browser sends an Origin) is a deliberate disconnect: no page connects again by
    # itself. A program's (the launcher's stop on Ctrl+C) is not one: the next start connects as it always did.
    Deliberate = request is not None and bool(request.headers.get("origin"))
    if Deliberate:
        controller.SetDisconnectIntent(True, request.scope.get(RemoteAccess.AccessKey, "local"), Keep=False)
    Ok, Message = controller.emergency_stop()
    if Deliberate:
        controller.KeepDisconnectIntent()  # on disk only once the torque is off
    return {"ok": Ok, "error": None if Ok else Message, "message": Message}


@app.post("/motion")
def MotionCommand(Payload: dict[str, Any], request: Request = None) -> dict[str, Any]:  # request: None when called directly
    # The preempt generation this request arrived under, read first, as the /ws queue reads a command's: a Stop that
    # comes in while it waits (on the calibration archive below, which takes the motor lock and writes to disk, or on
    # the lock itself) cancels it before its route starts. An HTTP move that arrived before a Stop but read the motion
    # epoch after it used to start its route after the Stop (2026-10-08 safety review).
    Generation = BumpPreemptGeneration() if isinstance(Payload, dict) and Preempts(Payload) else PreemptGeneration()
    if Payload.get("plan_id"):
        try:
            SelfCalibration.RecordEvent("MotionRequest", {"request": Payload, "state": controller.state_snapshot()})
        except Exception as Exc:
            return {"ok": False, "error": f"Could not archive calibration motion: {Exc}", "state": controller.state_snapshot()}
    Ok, Message = RunQueuedCommand(Payload, Generation, request.scope.get(RemoteAccess.AccessKey, "password") if request else "local")
    Result = {"ok": Ok, "error": None if Ok else Message, "message": Message, "state": controller.state_snapshot()}
    if Payload.get("plan_id"):
        try:
            SelfCalibration.RecordEvent("MotionResult", Result)
        except Exception as Exc:
            Result.update(ok=False, error=f"Could not archive calibration motion result: {Exc}")
    return Result


@app.get("/ports")
def ports() -> list[str]:
    return [port.device for port in serial.tools.list_ports.comports()]


@app.get("/notepad")
def get_notepad() -> dict[str, Any]:
    try:
        content = NOTEPAD_PATH.read_text(encoding="utf-8") if NOTEPAD_PATH.exists() else ""
    except OSError:
        content = ""
    return {"content": content}


@app.post("/notepad")
def set_notepad(payload: dict[str, Any]) -> dict[str, Any]:
    content = str(payload.get("content", ""))
    try:
        NOTEPAD_PATH.write_text(content, encoding="utf-8")
        return {"ok": True}
    except OSError as exc:
        return {"ok": False, "error": str(exc)}


@app.post("/cad/alignment")
def CaptureCadAlignment(Payload: dict[str, Any]) -> dict[str, Any]:
    return controller.CaptureCadReference(Payload.get("confirmed") is True)


@app.post("/ik/preview")
def ik_preview(payload: dict[str, Any]) -> dict[str, Any]:
    return controller.preview_ik(payload)


# --- Camera-to-workspace calibration + calibrated object localization -------
# See calibration.py / object_locator.py. Coordinates flow through the bridge
# opaquely in the web app's AI frame (X left+, Y farther+, Z up+).

@app.get("/calibration/status")
def calibration_status() -> dict[str, Any]:
    return calibration.status()


@app.get("/surfaces")
def declared_surfaces() -> dict[str, Any]:
    """The supports besides the desk declared in DeclaredSurfaces.json (read again on every change), and what is wrong with the rest."""
    return {"surfaces": DeclaredSurfaces.Load(), "problems": DeclaredSurfaces.Problems(), "table_z_cm": DeclaredSurfaces.TableZCm}


@app.post("/calibration/probe")
def calibration_probe(payload: dict[str, Any]) -> dict[str, Any]:
    try:
        return calibration.add_probe_point(
            u=float(payload["u"]),
            v=float(payload["v"]),
            x=float(payload["x"]),
            y=float(payload["y"]),
            z=float(payload["z"]),
            image_size=payload.get("image_size"),
            mirrored=payload["mirrored"] if isinstance(payload.get("mirrored"), bool) else None,
        )
    except (KeyError, TypeError, ValueError) as exc:
        return {"ok": False, "error": f"Bad probe payload: {exc}"}


@app.post("/calibration/undo")
def calibration_undo() -> dict[str, Any]:
    return calibration.undo_probe_point()


@app.post("/calibration/clear")
def calibration_clear() -> dict[str, Any]:
    return calibration.clear_probe_points()


@app.post("/calibration/fit")
def calibration_fit() -> dict[str, Any]:
    return calibration.fit()


@app.get("/selfcal/poses")
def selfcal_poses(view: str = "Behind") -> dict[str, Any]:
    try:
        Result = controller.selfcal_poses("Facing" if view == "Facing" else "Behind")
        with controller.lock:
            Plans = copy.deepcopy(controller.CalibrationPlans)
        SelfCalibration.RecordEvent("Plan", {"result": Result, "motor_plans": Plans})
        return Result
    except Exception as Exc:
        return {"ok": False, "error": f"Could not save calibration plan: {Exc}"}


@app.post("/selfcal/prepare")
def PrepareCalibration() -> dict[str, Any]:
    Result = controller.PrepareCalibration()
    Result["archive_version"] = 2
    return Result


@app.post("/selfcal/reset")
def selfcal_reset() -> dict[str, Any]:
    try:
        with controller.lock:
            Context = {"initial_state": controller.state_snapshot(), "arm_geometry": copy.deepcopy(controller.arm_geometry),
                       "home_preset": dict(controller.home_preset), "workspace_frame": _load_json(BACKEND_ROOT / "WorkspaceFrame.json"),
                       "gui_limits": _load_json(DEFAULT_LIMITS), "previous_camera_calibration": _load_json(BACKEND_ROOT / "camera_calibration.json")}
        Result = SelfCalibration.ResetSession(Context)
        Result["archive_version"] = 2
        return Result
    except Exception as Exc:
        return {"ok": False, "error": f"Could not create the calibration archive: {Exc}"}


@app.post("/selfcal/observe")
def selfcal_observe(payload: dict[str, Any]) -> dict[str, Any]:
    images = payload.get("images")
    joints = payload.get("joints")
    pose_index = payload.get("pose_index")
    try:
        Metadata = {Key: Value for Key, Value in payload.items() if Key not in ("images", "joints")}
        if not (isinstance(images, list) and images and all(isinstance(Image, str) for Image in images)
                and isinstance(joints, list) and len(joints) >= 3):
            Metadata.update(capture_valid=False, capture_error="Camera frames or matching joint positions are missing.")
        Metadata["bridge_state_at_receive"] = controller.state_snapshot()
        DepthFrames, DepthIntrinsics = _depth_frames(payload.get("depth_seqs"))
        Metadata["depth_frames"] = 0 if DepthFrames is None else sum(Frame is not None for Frame in DepthFrames)
        return SelfCalibration.AddObservation(
            images, joints, bool(payload.get("mirrored", False)),
            int(pose_index) if isinstance(pose_index, (int, float)) else None,
            Method="red", Metadata=Metadata, DepthFrames=DepthFrames, DepthIntrinsics=DepthIntrinsics,
        )
    except Exception as Exc:
        return {"ok": False, "error": f"Could not record calibration observation: {Exc}"}


@app.post("/selfcal/check")
def selfcal_check(payload: dict[str, Any]) -> dict[str, Any]:
    """Look for the marker in live frames right now. No motion, no session change."""
    try:
        images = payload.get("images")
        if not (isinstance(images, list) and images and all(isinstance(i, str) for i in images)):
            return {"ok": False, "error": "Payload must include images: a list of data URLs."}
        InputArchive = SelfCalibration.RecordEvent("CheckInput", payload)
        DepthFrames, DepthIntrinsics = _depth_frames(payload.get("depth_seqs"))
        Result = SelfCalibration.CheckFrames(images, bool(payload.get("mirrored", False)),
                                             Hint=payload.get("hint"), Initialize=bool(payload.get("initialize", False)),
                                             DepthFrames=DepthFrames, DepthIntrinsics=DepthIntrinsics)
        SelfCalibration.RecordEvent("CheckResult", {"input_archive": InputArchive, "result": Result})
        return Result
    except Exception as exc:
        return {"ok": False, "error": f"Check failed: {type(exc).__name__}: {exc}"}


@app.post("/selfcal/solve")
def selfcal_solve(payload: dict[str, Any] | None = None) -> dict[str, Any]:
    table_z = (payload or {}).get("table_z")
    # "manual": fit the user's clicks (semi-automatic calibration, SelfCalClicks.py) instead of the red blobs.
    source = (payload or {}).get("source", "auto")
    if source not in ("auto", "manual"):
        return {"ok": False, "error": 'source must be "auto" or "manual".'}
    try:
        SelfCalibration.RecordEvent("SolveRequest", payload or {})
        report = SelfCalibration.Solve(TableZ=float(table_z) if table_z is not None else None, Save=False, Source=source)
    except Exception as exc:  # a solver crash must reach the UI, not a 500
        return {"ok": False, "computed": False, "saved": False,
                "error": f"Could not compute a calibration: the solver failed ({type(exc).__name__}: {exc}). "
                         "The current calibration is still in use."}
    try:
        # The full report, the camera model included, is archived before the camera is applied.
        SelfCalibration.RecordEvent("FitReport", report)
    except Exception as exc:
        if report.get("ok"):
            report.update(ok=False, saved=False, error=f"The calibration was computed but its fit report could not be archived "
                                                       f"({type(exc).__name__}: {exc}), so it was not saved. The current calibration "
                                                       "is still in use.")
        report.pop("model", None)
        return report
    if report.get("ok"):
        # Every computed camera is saved and applied, whatever its quality: its problems are warnings
        # (report["problems"], also in the model), and only the user rejects a calibration (/selfcal/reject).
        try:
            Saved = calibration.save_camera_model(report["model"], SelfCalibration.RunDirectory())
        except Exception as exc:  # the file it replaces could not be archived, or the new one not written
            report.update(ok=False, saved=False, error=f"The calibration was computed but could not be saved ({type(exc).__name__}: "
                                                       f"{exc}). The current calibration is still in use.")
        else:
            report.update(saved=True, can_reject=Saved["can_reject"], replaced_fitted_at=Saved["replaced_fitted_at"])
            try:
                SelfCalibration.RecordEvent("CameraSaved", {"model": report["model"], "quality_problems": report.get("problems", []),
                                                            "replaced_archive": Saved["replaced_archive"]})
            except Exception as Exc:
                report.setdefault("warnings", []).append(f"Camera saved, but its archive confirmation failed: {Exc}")
    report.pop("model", None)
    return report


@app.post("/selfcal/finish")
def FinishCalibration(payload: dict[str, Any]) -> dict[str, Any]:
    try:
        SelfCalibration.RecordEvent("Finish", {"outcome": payload, "state": controller.state_snapshot()})
        return {"ok": True}
    except Exception as Exc:
        return {"ok": False, "error": f"Could not archive the calibration outcome: {Exc}"}


@app.post("/selfcal/reject")
def RejectCalibration(payload: dict[str, Any] | None = None) -> dict[str, Any]:
    """Reject calibration: the user's explicit choice, the only way a calibration is ever rejected. The
    calibration the current one replaced is used again (calibration.reject_camera_model); fitted_at names the
    calibration the user was shown. Data only: nothing moves."""
    Payload = payload or {}
    FittedAt, Reason = Payload.get("fitted_at"), Payload.get("reason")
    if FittedAt is not None and not isinstance(FittedAt, str):
        return {"ok": False, "error": "fitted_at must be the fit time of the calibration to reject."}
    try:
        return calibration.reject_camera_model(FittedAt, str(Reason)[:200] if Reason else None)
    except Exception as Exc:
        return {"ok": False, "error": f"Could not reject the calibration: {type(Exc).__name__}: {Exc}"}


# --- Semi-automatic calibration: the user's clicks on each pose's frozen frame ----
# Data only: none of these moves the arm. Each click names the archived capture
# (/selfcal/observe's capture_archive) it was made on; see SelfCalClicks.py.

@app.get("/selfcal/clicks")
def SelfCalClickStatus() -> dict[str, Any]:
    return SelfCalClicks.Status()


@app.post("/selfcal/click")
def SelfCalClick(payload: dict[str, Any]) -> dict[str, Any]:
    try:
        return SelfCalClicks.AddClick(payload)
    except Exception as Exc:
        return {"ok": False, "error": f"Could not record the click: {type(Exc).__name__}: {Exc}"}


@app.post("/selfcal/click/undo")
def SelfCalClickUndo(payload: dict[str, Any] | None = None) -> dict[str, Any]:
    try:
        return SelfCalClicks.RemoveClick(payload or {})
    except Exception as Exc:
        return {"ok": False, "error": f"Could not undo the click: {type(Exc).__name__}: {Exc}"}


@app.post("/selfcal/click/skip")
def SelfCalClickSkip(payload: dict[str, Any]) -> dict[str, Any]:
    try:
        return SelfCalClicks.SkipPose(payload)
    except Exception as Exc:
        return {"ok": False, "error": f"Could not skip the pose: {type(Exc).__name__}: {Exc}"}


@app.post("/locate")
def locate(payload: dict[str, Any]) -> dict[str, Any]:
    image = payload.get("image")
    if not isinstance(image, str) or not image.startswith("data:"):
        return {"ok": False, "error": "Payload must include a data-URL image."}
    labels = payload.get("labels")
    if labels is not None and not (
        isinstance(labels, list) and all(isinstance(item, str) for item in labels)
    ):
        return {"ok": False, "error": "labels must be a list of strings."}
    # The task's object as the app parsed it, e.g. {"phrase": "black thing on the desk",
    # "label": "black thing"}: grounding confirms which measured object it is.
    task_object = payload.get("task_object")
    if task_object is not None:
        phrase, label = (task_object.get("phrase"), task_object.get("label")) if isinstance(task_object, dict) else (None, None)
        if not (isinstance(phrase, str) and phrase.strip() and len(phrase) <= TASK_OBJECT_PHRASE_MAX_CHARS
                and isinstance(label, str) and label.strip()):
            return {"ok": False, "error": f'task_object must be {{"phrase": text of 1-{TASK_OBJECT_PHRASE_MAX_CHARS} '
                                          'characters, "label": text}.'}
        task_object = {"phrase": phrase, "label": label}
    model = payload.get("model")
    # Every /locate revokes the measured scene before it looks. With a route certified against that scene running,
    # the route used to coast the arm mid-way (2026-10-07 audit H1); it would now stop and hold. Neither is wanted:
    # wait for the route (the page itself never looks while the arm moves; a second tab or client can).
    if controller.SceneBoundRouteActive():
        return {"ok": False, "error_code": "arm_moving",
                "error": "The arm is moving on a route checked against the current measured scene: wait for it to finish "
                         "(or press Stop and hold), then look again."}
    # A calibration fits one camera. A capture from the other camera (the
    # depth camera versus a webcam, both 1280x720) would map to wrong positions.
    saved_type = (calibration.camera_model() or {}).get("type")
    if saved_type and (saved_type == "pinhole_depth_v1") != (_sequence(payload.get("depth_seq")) is not None):
        return {"ok": False, "error": "The saved calibration belongs to the other camera. Recalibrate with the camera in use."}
    # A capture from the depth camera names its frame; measure objects in 3D. The
    # camera is checked on that same frame, before any model call (object_locator).
    depth = None
    seq = _sequence(payload.get("depth_seq"))
    pin = depth_camera.Depth(seq) if seq else None
    intrinsics = depth_camera.Intrinsics()
    if pin is not None and intrinsics is not None:
        depth = {"depth_mm": pin["depth_mm"], "intrinsics": intrinsics, "mirrored": bool(payload.get("mirrored", False)),
                 "seq": pin["seq"], "time": pin["time"]}
    # The arm's pose at the capture: the camera is checked against its tip tape (also at the
    # calibration's wrist roll, for the roll-aware check), and the arm is masked out of the measurements.
    arm = controller.CaptureArmPose(pin["time"], RobotReference.ReferenceRaw(calibration.camera_model())) if pin is not None else None
    mirrored = payload["mirrored"] if isinstance(payload.get("mirrored"), bool) else None
    return object_locator.locate_objects(image, labels, model if isinstance(model, str) else None, depth=depth,
                                         mirrored=mirrored, arm=arm, task_object=task_object)


# --- Orbbec depth camera ------------------------------------------------------
# The browser shows this camera by polling frame.jpg; each frame's X-Frame-Seq
# goes back with captures so depth lookups use the matching depth frame.

# v2: live depth overlay (/depthcam/overlay.png).
DEPTH_CAMERA_VERSION = "orbbec-v2"


def _sequence(value: Any) -> int | None:
    return int(value) if isinstance(value, (int, float)) and not isinstance(value, bool) and value > 0 else None


def _depth_frames(seqs: Any) -> tuple[list[np.ndarray | None] | None, dict[str, Any] | None]:
    """Pinned depth frames for capture sequence numbers, None where unavailable."""
    intrinsics = depth_camera.Intrinsics()
    if not isinstance(seqs, list) or not seqs or intrinsics is None:
        return None, None
    frames = []
    for seq in seqs:
        pin = depth_camera.Depth(_sequence(seq)) if _sequence(seq) else None
        frames.append(pin["depth_mm"] if pin else None)
    return (frames if any(frame is not None for frame in frames) else None), intrinsics


def _depth_pin(payload: dict[str, Any]) -> dict[str, Any] | None:
    return depth_camera.Depth(_sequence(payload.get("seq")))


@app.get("/depthcam/status")
def depthcam_status() -> dict[str, Any]:
    status = depth_camera.Status()
    status["calibrated"] = DepthGeometry.ModelUsesDepthCamera(calibration.camera_model(), status["intrinsics"])
    status["version"] = DEPTH_CAMERA_VERSION
    return status


@app.get("/depthcam/frame.jpg")
def depthcam_frame(after: int = 0) -> Response:
    """Newest color frame newer than `after` (long-polls briefly); 204 when none arrived."""
    latest = depth_camera.WaitJpeg(after, 1.5)
    headers = {"Cache-Control": "no-store"}
    # A sequence below `after` means the client's number is from a previous bridge process:
    # serve the newest frame so its long-poll resyncs instead of waiting out the old count.
    if latest is None or latest[0] == after:
        return Response(status_code=204 if latest else 503, headers=headers)
    seq, jpeg = latest
    return Response(content=jpeg, media_type="image/jpeg", headers={**headers, "X-Frame-Seq": str(seq)})


@app.get("/depthcam/overlay.png")
def depthcam_overlay(after: int = 0, near: float = 25.0, far: float = 150.0) -> Response:
    """Depth over the live view: the newest frame after `after`, colorized from
    `near` to `far` cm, transparent where there is no reading, half resolution.
    Unmirrored like the live view. Never pins a frame; 204 when none arrived."""
    headers = {"Cache-Control": "no-store"}
    latest = depth_camera.WaitDepth(after, 1.5)
    if latest is None or latest[0] == after:  # below `after`: a restarted bridge, resync as above
        return Response(status_code=204 if latest else 503, headers=headers)
    near, far = (near, far) if 0.0 <= near < far <= 2000.0 else (25.0, 150.0)
    return Response(content=OrbbecCamera.OverlayPng(latest[1], near, far), media_type="image/png",
                    headers={**headers, "X-Frame-Seq": str(latest[0]), "X-Depth-Range-Cm": f"{near:g},{far:g}"})


@app.post("/depthcam/depth")
def depthcam_depth(payload: dict[str, Any]) -> dict[str, Any]:
    """Colorized depth for a capture (same pixels and mirroring as the color image)."""
    pin = _depth_pin(payload)
    if pin is None:
        return {"ok": False, "error": "No depth frame for that capture."}
    image, stats = OrbbecCamera.ColorizeDepth(pin["depth_mm"], bool(payload.get("mirrored", False)))
    height, width = pin["depth_mm"].shape[:2]
    return {"ok": True, "seq": pin["seq"], "width": width, "height": height, "source": "orbbec",
            "depth_image": "data:image/png;base64," + base64.b64encode(image).decode("ascii"),
            "colormap": {"name": "turbo", "near_cm": stats["near_cm"], "far_cm": stats["far_cm"]},
            "depth_stats_cm": {Key: stats[Key] for Key in ("min_cm", "median_cm", "max_cm") if Key in stats},
            "valid_fraction": stats["valid_fraction"], "frames_merged": pin["frames"]}


@app.post("/depthcam/point")
def depthcam_point(payload: dict[str, Any]) -> dict[str, Any]:
    """3D point under one capture pixel: camera frame, and the AI frame when calibrated."""
    pin = _depth_pin(payload)
    intrinsics = depth_camera.Intrinsics()
    if pin is None or intrinsics is None:
        return {"ok": False, "error": "No depth frame for that capture."}
    try:
        result = DepthGeometry.PointAtPixel(pin["depth_mm"], float(payload["u"]), float(payload["v"]),
                                            intrinsics, bool(payload.get("mirrored", False)))
    except (KeyError, TypeError, ValueError) as exc:
        return {"ok": False, "error": f"Bad point payload: {exc}"}
    model = calibration.camera_model()
    if result.get("ok") and DepthGeometry.ModelUsesDepthCamera(model, intrinsics):
        x, y, z = DepthGeometry.CameraToDisplay(np.array(result["camera_cm"]), model)[0]
        result["display_cm"] = {"x": round(float(x), 2), "y": round(float(y), 2), "z": round(float(z), 2)}
    result["seq"] = pin["seq"]
    return result


@app.get("/depthcam/depth-at")
def depthcam_depth_at(u: float, v: float) -> dict[str, Any]:
    """Depth under one live-view pixel (unmirrored, like the live view) in the newest frame, for the
    hover readout. Never pins a frame, so hovering cannot evict the frames an AI capture is waiting on."""
    latest = depth_camera.WaitDepth(0, 0.0)
    intrinsics = depth_camera.Intrinsics()
    if latest is None or intrinsics is None:
        return {"ok": False, "error": "The depth camera is not streaming."}
    result = DepthGeometry.PointAtPixel(latest[1], u, v, intrinsics, False)
    model = calibration.camera_model()
    if result.get("ok") and DepthGeometry.ModelUsesDepthCamera(model, intrinsics):
        x, y, z = DepthGeometry.CameraToDisplay(np.array(result["camera_cm"]), model)[0]
        result["display_cm"] = {"x": round(float(x), 2), "y": round(float(y), 2), "z": round(float(z), 2)}
    result["seq"] = latest[0]
    return result


@app.get("/depthcam/check")
def depthcam_check() -> dict[str, Any]:
    """Fit the dominant plane in view; once calibrated, compare it with the table height.

    A calibration that disagrees with what the depth camera sees right now
    (camera bumped, table moved) shows up here as a height or tilt error.
    """
    pin = depth_camera.Depth()
    intrinsics = depth_camera.Intrinsics()
    if pin is None or intrinsics is None:
        return {"ok": False, "error": "The depth camera is not streaming."}
    model = calibration.camera_model()
    # camera_check: the drift check /locate runs (the desk plane against the calibration's
    # baseline, then the gripper tape; with no calibrated desk in view, the gripper tape through
    # every joint), with the same fields; camera_verdict says it in a word (CameraDrift.Verdict).
    # The other fields describe the dominant plane in view, kept for the tools that read them.
    jpeg = depth_camera.Jpeg(pin["seq"])
    image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR) if jpeg else None
    camera_check = CameraDrift.CheckCapture(pin["depth_mm"], intrinsics, model, pin["seq"], image,
                                            controller.CaptureArmPose(pin["time"], RobotReference.ReferenceRaw(model)))
    # A desk re-measured after a cart move is reported, never saved: only /locate writes the calibration.
    if CameraDrift.TakeRemeasuredReference(camera_check) is not None:
        camera_check["desk_saved"] = False
    return DepthGeometry.CheckPlane(pin["depth_mm"], intrinsics, model) | {
        "seq": pin["seq"], "camera_verdict": CameraDrift.Verdict(camera_check), "camera_check": camera_check}


def CapturePath(Kind: str, Extension: str, Partners: tuple[str, ...] = (), Now: float | None = None) -> Path:
    """captures/<date>/HHMMSS_mmm_<kind>.<ext>, created empty and exclusively (with any partner files of the same
    name, a clip's .json): a second capture in the same millisecond gets HHMMSS_mmm-1_<kind> instead of overwriting
    the first, as the recorder's segments did (audit 2 M5). The clock is read once; the second and the milliseconds
    came from two reads before. The captures folder follows SO101_CAPTURES_ROOT, as the session record does (tests).
    These names are what the 7-day video retention matches (VideoRetention.py)."""
    Now = time.time() if Now is None else Now
    Day = SessionLog.CapturesRoot() / time.strftime("%Y-%m-%d", time.localtime(Now))
    Day.mkdir(parents=True, exist_ok=True)
    Base = time.strftime("%H%M%S", time.localtime(Now)) + f"_{int(Now * 1000) % 1000:03d}"
    for Count in range(1000):
        Stem = f"{Base}{f'-{Count}' if Count else ''}_{Kind}"
        Made: list[Path] = []
        try:
            for Name in (f"{Stem}.{Extension}", *(f"{Stem}.{Partner}" for Partner in Partners)):
                os.close(os.open(Day / Name, os.O_CREAT | os.O_EXCL | os.O_WRONLY))
                Made.append(Day / Name)
            return Day / f"{Stem}.{Extension}"
        except FileExistsError:
            for Path_ in Made:  # ours, made just above
                Path_.unlink(missing_ok=True)
    raise OSError(f"no free capture name for {Base}_{Kind} in {Day}")


@app.post("/captures/video")
async def save_capture_video(request: Request) -> dict[str, Any]:
    """Archive a task recording to captures/YYYY-MM-DD/HHMMSS_mmm_task.webm.

    The web app records the webcam with MediaRecorder for the life of an AI
    task and posts the raw .webm bytes here when the task ends. Query params
    (task, reason, duration_s, started_at) are written to a sidecar .json so the
    clip can be matched to what was asked. The AI never sees these clips. Like
    the session video they are kept 7 days (VideoRetention.py).
    """
    body = await request.body()
    if not body:
        return {"ok": False, "error": "Empty video body."}
    content_type = request.headers.get("content-type", "")
    ext = "webm" if "webm" in content_type else "mp4" if "mp4" in content_type else "bin"
    try:
        path = CapturePath("task", ext, ("json",))
        stem = path.stem
        day_dir = path.parent
        path.write_bytes(body)
        meta = {
            "task": request.query_params.get("task", ""),
            "reason": request.query_params.get("reason", ""),
            "duration_s": request.query_params.get("duration_s", ""),
            "started_at": request.query_params.get("started_at", ""),
            "bytes": len(body),
            "content_type": content_type,
        }
        (day_dir / f"{stem}.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
        return {"ok": True, "path": str(path), "bytes": len(body)}
    except OSError as exc:
        return {"ok": False, "error": str(exc)}


@app.post("/captures")
def save_capture(payload: dict[str, Any]) -> dict[str, Any]:
    """Archive a workspace photo to captures/YYYY-MM-DD/HHMMSS_mmm_<kind>.jpg.

    The web app sends the same frame the AI sees. When `mirrored` is true (the
    capture was horizontally flipped to match the arm frame), it is flipped
    back before saving so the archive shows the desk in natural orientation.
    """
    image = payload.get("image")
    if not isinstance(image, str) or not image.startswith("data:"):
        return {"ok": False, "error": "Payload must include a data-URL image."}
    kind = str(payload.get("kind") or "capture")
    kind = "".join(ch for ch in kind if ch.isalnum() or ch in "-_")[:24] or "capture"
    try:
        _, b64 = image.split(",", 1)
        buffer = np.frombuffer(base64.b64decode(b64), dtype=np.uint8)
        decoded = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
        if decoded is None:
            return {"ok": False, "error": "Could not decode the image."}
        if bool(payload.get("mirrored")):
            decoded = cv2.flip(decoded, 1)
        path = CapturePath(kind, "jpg")
        cv2.imwrite(str(path), decoded, [cv2.IMWRITE_JPEG_QUALITY, 90])
        return {"ok": True, "path": str(path)}
    except (ValueError, OSError, cv2.error) as exc:
        return {"ok": False, "error": str(exc)}


async def AuthenticateSocket(Socket: WebSocket) -> str | None:
    """Another device's /ws (RemoteAccess rule 3): its first message must be {"type": "auth", "password": ...}, sent
    within RemoteAccess.AuthWaitS, before it gets any state. Answers {"type": "access", "ok": ...} either way; returns
    "password", or None after closing it (4401 password missing or wrong, 4403 remote access off, 4429 locked out)."""
    Client, Key = RemoteAccess.ClientName(Socket.client.host if Socket.client else "",
                                          {Name.lower(): Value for Name, Value in Socket.headers.items()})
    Password = None
    try:
        Message = json.loads(await asyncio.wait_for(Socket.receive_text(), timeout=RemoteAccess.AuthWaitS))
        if isinstance(Message, dict) and Message.get("type") == "auth" and isinstance(Message.get("password"), str):
            Password = Message["password"]
    except WebSocketDisconnect:
        return None
    except (asyncio.TimeoutError, ValueError, KeyError, TypeError, RuntimeError):
        pass  # nothing usable came: refused below as a missing password
    Ok, Code = await asyncio.to_thread(RemoteAccess.CheckPassword, Password, Key)
    try:
        if Ok:
            RemoteAccess.Note("let in", Client, "password", "WebSocket /ws")
            await Socket.send_json({"type": "access", "ok": True, "access": "password", "text": "Arm password accepted"})
            return "password"
        RemoteAccess.Note("refused", Client, Code, "WebSocket /ws")
        await Socket.send_json({"type": "access", "ok": False, "access": Code, "text": RemoteAccess.Refusals[Code][1]})
        await Socket.close(code=RemoteAccess.CloseCodes[Code], reason=Code)
    except (WebSocketDisconnect, RuntimeError):
        pass
    return None


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket) -> None:
    # Who this is (RemoteAccess.Middleware): this PC ("local"), another device that already gave the password in a
    # header ("password", programs), or another device's page, which gives it as its first message ("pending").
    Access = websocket.scope.get(RemoteAccess.AccessKey, "pending")
    await websocket.accept()
    if Access not in ("local", "password"):
        Access = await AuthenticateSocket(websocket)
        if Access is None:
            return
    # Commands run one at a time on their own task, answered in order, while this loop
    # keeps sending state. It used to wait for each command, so a client got no state
    # while its send_ik planned (7-9 s on 2026-09-28) and took the frozen, pre-motion
    # pose for "arrived".
    SendLock = asyncio.Lock()
    # This connection's id: a hand-guiding session it starts is its own, kept going by its heartbeats.
    ConnectionId = uuid.uuid4().hex

    async def Send(Message: dict[str, Any]) -> None:
        async with SendLock:
            await websocket.send_json(Message)

    async def RunCommands() -> None:
        while True:
            Generation, Message = await Commands.get()
            if Generation != PreemptGeneration():
                # Queued before a Stop, Hold, Coast, disconnect or emergency stop (from any connection): never run.
                await Send({"type": "status", "ok": False, "text": f"{str(Message.get('type', 'command')).replace('_', ' ')} not run: "
                                                                    "Stop, Hold or Coast was pressed after it was sent"})
                continue
            ok, text = await asyncio.to_thread(RunQueuedCommand, Message, Generation, Access, ConnectionId)
            await Send({"type": "status", "ok": ok, "text": text})
            await Send(await asyncio.to_thread(controller.state_snapshot))

    Commands: asyncio.Queue = asyncio.Queue()
    await Send(await asyncio.to_thread(controller.state_snapshot))
    Worker = asyncio.create_task(RunCommands())
    try:
        while True:
            if Worker.done():
                Worker.result()  # the command task failed: end this connection as a failed command used to
            try:
                raw = await asyncio.wait_for(websocket.receive_text(), timeout=BRIDGE_FRAME_S)
            except asyncio.TimeoutError:
                # The control loop runs in its own thread (ControlLoop); this
                # client only needs the latest state.
                await Send(await asyncio.to_thread(controller.state_snapshot))
                continue

            try:
                message = json.loads(raw)
            except json.JSONDecodeError:
                await Send({"type": "status", "ok": False, "text": "Unreadable bridge command"})
                continue
            if not isinstance(message, dict):
                await Send({"type": "status", "ok": False, "text": "Bridge command must be an object"})
                continue
            # A page that sends its password anyway (another device, again), or this PC's: already in.
            if message.get("type") == "auth":
                await Send({"type": "access", "ok": True, "access": Access, "text": "Already let in"})
                continue
            # The page that started hand guiding says it is still there (every 0.5 s); no answer, no queue.
            if message.get("type") == "hand_guide_heartbeat":
                Beat = getattr(controller, "HandGuideHeartbeat", None)
                if Beat is not None:
                    Beat(ConnectionId)
                continue

            # Emergency stop never waits behind a queued or planning command. It is a deliberate disconnect: no page
            # connects the arm again by itself until someone presses Connect.
            if message.get("type") == "emergency_stop":
                BumpPreemptGeneration()
                controller.SetDisconnectIntent(True, Access, Keep=False)
                ok, text = await asyncio.to_thread(controller.emergency_stop)
                await Send({"type": "status", "ok": ok, "text": text})
                await Send(controller.state_snapshot())
                await asyncio.to_thread(controller.KeepDisconnectIntent)  # on disk only once the torque is off
                await websocket.close(code=1000, reason="Emergency shutoff")
                return
            # Nor do Stop, Hold, Coast and disconnect (Preempts): they run now, the commands queued before them are
            # dropped, and a route still planning ends at its next check (its snapshot sees the motion epoch move on).
            if Preempts(message):
                BumpPreemptGeneration()
                ok, text = await asyncio.to_thread(controller.handle_command, message, Access)
                await Send({"type": "status", "ok": ok, "text": text})
                await Send(await asyncio.to_thread(controller.state_snapshot))
                continue

            Commands.put_nowait((PreemptGeneration(), message))
    except WebSocketDisconnect:
        return
    finally:
        Worker.cancel()
        Closed = getattr(controller, "HandGuidePageClosed", None)  # (test doubles of the controller have none)
        if Closed is not None:
            Closed(ConnectionId)
