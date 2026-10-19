"""Motion faults, protection trips and motion aborts are kept on disk, once each. Fake motors only.

2026-09-29 07:57 a latched bridge fault ended an AI task, and its reason was lost: the bridge
kept it only in memory (the state message and the page log). The running bridge now appends
each one to captures/<date>/recording/bridge_events.jsonl.

Run:  python TestBridgeEvents.py
"""
import asyncio
import json
import os
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

import bridge_server as Bridge
import SessionLog
from TestSafety import MakeController


def Recording(Controller):
    """Point the controller's events at a list, as the running server points them at the file."""
    Events = []
    Controller.EventSink = Events.append
    return Events


def DelayWhileMoving(Controller, Seconds=1.0):
    """A gripper stream running, then the next control frame Seconds late: the heartbeat trip.
    (An arm holding still is read again instead of tripping: TestMotorProtection.)"""
    Gripper = Controller.servos["gripper"]
    Controller._send_slider_target_locked(Gripper, Gripper.present_raw + 200)
    Controller.last_refresh_s = time.monotonic() - Seconds


class BridgeEventTests(unittest.TestCase):
    def TestStallTripIsRecordedOnceWithTheJointAndItsError(Self):
        Controller, Bus = MakeController()
        Events = Recording(Controller)
        Servo = Controller.servos["elbow_flex"]
        Controller._send_slider_target_locked(Servo, Servo.present_raw - 200)
        Controller.LastProtectionPoll = time.monotonic()
        Controller.ProtectionProgress["elbow_flex"] = (200.0, time.monotonic() - Bridge.ProtectionStallSeconds - 1)
        for _ in range(30):
            Controller.refresh()
        Self.assertEqual(len(Events), 1, Events)
        Event = Events[0]
        Self.assertEqual((Event["kind"], Event["joint"]), ("protection_trip", "elbow_flex"))
        Self.assertIn("elbow_flex stalled", Event["reason"])
        Self.assertEqual(Event["details"]["error_raw"], 200)
        Self.assertGreaterEqual(Event["details"]["stalled_s"], Bridge.ProtectionStallSeconds)
        # The servo record is from before the trip cleared the streams: it still shows the commanded target.
        Self.assertEqual(Event["details"]["servos"]["elbow_flex"]["target_raw"], Servo.present_raw - 200)
        Self.assertEqual(set(Event), {"kind", "reason", "joint", "details", "t", "time"})

    def TestHoldDriftIsRecordedAsAMotionFault(Self):
        Controller, Bus = MakeController()
        Events = Recording(Controller)
        Servo = Controller.servos["shoulder_lift"]
        Servo.Holding = True
        Held = Servo.target_raw
        Bus.position["shoulder_lift"] = Held + 600
        Controller.refresh()
        Controller.refresh()
        Self.assertEqual([(Event["kind"], Event["joint"]) for Event in Events], [("motion_fault", "shoulder_lift")])
        Details = Events[0]["details"]
        Self.assertEqual((Details["drift_raw"], Details["limit_raw"]), (600, Bridge.HOLD_REARM_MAX_ERROR_RAW))
        Self.assertEqual((Details["present_raw"], Details["target_raw"]), (Held + 600, Held))

    def TestDelayedHeartbeatRecordsTheGap(Self):
        Controller, Bus = MakeController()
        Events = Recording(Controller)
        DelayWhileMoving(Controller)
        Controller.refresh()
        Controller.refresh()
        # The trip, then the stall diagnostic of the same late frame (NoteControlStall: what it waited for).
        Self.assertEqual([Event["kind"] for Event in Events], ["protection_trip", "control_stall"])
        Self.assertEqual(Events[0]["reason"], "Motor control heartbeat was delayed")
        Self.assertIsNone(Events[0]["joint"])
        Self.assertGreaterEqual(Events[0]["details"]["gap_ms"], 1000)
        Self.assertEqual(Events[0]["details"]["limit_ms"], 750)
        Self.assertGreaterEqual(Events[1]["details"]["gap_ms"], 1000)
        Self.assertIn("cause", Events[1]["details"])

    def TestServoHardwareFaultRecordsItsRegisters(Self):
        Controller, Bus = MakeController()
        Events = Recording(Controller)
        Bus.registers[("Hardware_Error_Status", "shoulder_lift")] = 32
        Bus.registers[("Present_Load", "shoulder_lift")] = 900
        for _ in range(5):
            Controller.refresh()
        Self.assertEqual([(Event["kind"], Event["joint"]) for Event in Events], [("motion_fault", "shoulder_lift")])
        Fault = Events[0]["details"]["faults"][0]
        Self.assertEqual((Fault["servo"], Fault["bits"], Fault["unload"], Fault["load"]), ("shoulder_lift", 32, ["overload"], 900))

    def TestLostFeedbackNamesTheSilentJoint(Self):
        Controller, Bus = MakeController()
        Events = Recording(Controller)
        Read = Bus.read
        def Fail(Reg, Name, **Options):
            if Reg == "Present_Position" and Name == "wrist_flex":
                raise ConnectionError("simulated cable loss")
            return Read(Reg, Name, **Options)
        Bus.read = Fail
        for _ in range(5):
            Controller.refresh()
        Self.assertEqual([(Event["kind"], Event["joint"]) for Event in Events], [("motion_fault", "wrist_flex")])
        Self.assertEqual(Events[0]["details"]["missing"], ["wrist_flex"])

    def TestStoppingARouteIsRecordedAsAnAbort(Self):
        Controller, Bus = MakeController()
        Events = Recording(Controller)
        Gripper = Controller.servos["gripper"]
        Ok, Message = Controller.move_servo("gripper", Gripper.present_raw + 60)
        Self.assertTrue(Ok, Message)
        Self.assertEqual(Events, [])
        Controller.StopMotion()
        Controller.StopMotion()  # nothing was moving any more: not another abort
        Self.assertEqual([(Event["kind"], Event["reason"]) for Event in Events], [("motion_abort", "Motion stopped")])
        Self.assertEqual(Events[0]["details"]["route_stage"], 1)
        # An emergency stop mid-route is one too; with nothing moving it is not.
        Controller, Bus = MakeController()
        Events = Recording(Controller)
        Self.assertTrue(Controller.move_servo("gripper", Controller.servos["gripper"].present_raw + 60)[0])
        Controller.emergency_stop()
        Controller.emergency_stop()
        Self.assertEqual([Event["kind"] for Event in Events], ["motion_abort"])
        Self.assertIn("Emergency", Events[0]["reason"])

    def TestACompletedRouteKeepsItsTiming(Self):
        """A route that arrives leaves one route_timing record, in its own log (route_timing.jsonl), not among the
        faults: its plan time, flying time, each stage's time and every control frame of it."""
        from TestFlyByRoutes import Execute
        with tempfile.TemporaryDirectory() as Root, patch.dict(os.environ, {"SO101_CAPTURES_ROOT": Root}):
            Controller, Bus = MakeController()
            Controller.EventSink = Controller.TimingSink = Bridge.AppendBridgeEvent
            Gripper = Controller.servos["gripper"]
            Goal = Gripper.present_raw + 120
            Ok, Message = Controller.move_servo("gripper", Goal)
            Self.assertTrue(Ok, Message)
            Stages = len(Controller.MotionRoute["Path"])
            Execute(Self, Controller, Bus)
            Self.assertTrue(Bridge.FlushBridgeEvents(5.0))
            Directory = SessionLog.RecordingDirectory()
            Self.assertFalse((Directory / "bridge_events.jsonl").exists(), "an arrival is not a fault")
            Lines = (Directory / "route_timing.jsonl").read_text(encoding="utf-8").splitlines()
            Self.assertEqual(len(Lines), 1)
            Record = json.loads(Lines[0])
            Self.assertEqual(Record["kind"], "route_timing")
            Self.assertNotIn("log", Record)
            Self.assertGreaterEqual(Record["plan_s"], 0)
            Self.assertGreaterEqual(Record["run_s"], 0)  # simulated frames run faster than real time
            Self.assertEqual((Record["stages"], len(Record["stage_s"])), (Stages, Stages))
            Self.assertEqual(Record["joints"], list(Bridge.MOTOR_NAMES))
            Self.assertTrue(Record["frames"])
            Seconds, Stage, Positions, Commands = Record["frames"][-1]
            Self.assertEqual((len(Positions), len(Commands)), (6, 6))
            Self.assertLessEqual(abs(Positions[Bridge.MOTOR_NAMES.index("gripper")] - Goal), Bridge.GRIPPER_GOAL_TOLERANCE_RAW + 1)
            Self.assertEqual(Record["waypoints_raw"][-1]["gripper"], Goal)
        # Without a timing sink nothing is kept, and a stopped route is an abort, not a timing.
        Controller, Bus = MakeController()
        Events = Recording(Controller)
        Controller.TimingSink = Events.append
        Self.assertTrue(Controller.move_servo("gripper", Controller.servos["gripper"].present_raw + 60)[0])
        Controller.StopMotion()
        Self.assertEqual([Event["kind"] for Event in Events], ["motion_abort"])

    def TestAFaultedRouteKeepsItsFramesToo(Self):
        """2026-10-04 23:50:01 a route tripped the route monitor and left no frames, so why the pan stood 36 steps
        past its last waypoint could not be measured. A route a latched fault stops is recorded as an arrival is,
        with the fault, the stage it was on and each waypoint's reach per joint; a plain Stop is still an abort."""
        Controller, Bus = MakeController()
        Events, Timings = Recording(Controller), []
        Controller.TimingSink = Timings.append
        Gripper = Controller.servos["gripper"]
        Start = Gripper.present_raw
        Self.assertTrue(Controller.move_servo("gripper", Start + 120)[0])
        for _ in range(3):
            Bus.step("gripper", steps_per_tick=10)
            Controller.refresh()
        Self.assertEqual(Timings, [])
        Bus.position["gripper"] = Start - 100  # knocked back off its checked route
        Controller.refresh()
        Self.assertEqual(Events[0]["kind"], "motion_fault")
        Self.assertEqual(len(Timings), 1)
        Record = Timings[0]
        Self.assertEqual((Record["kind"], Record["fault"], Record["stage_at_fault"]),
                         ("route_timing", "gripper left the checked basket route", 0))
        Self.assertGreaterEqual(len(Record["frames"]), 3)
        Self.assertEqual(Record["waypoints_raw"][-1]["gripper"], Start + 120)
        Self.assertIn("reach_raw", Record)
        json.dumps(Record)

    def TestFramesInTheStartsOwnClockTickAreKept(Self):
        """Audit 2 L7, the root of the test above's flake: Python 3.12's time.monotonic on Windows ticks every 15.625 ms,
        so a route's start and its first control frames often read the same time. A frame's time is kept rounded to
        0.1 ms (RecordMotionTrace), and RecordRouteTiming compared it with the start unrounded, which dropped the start's
        own tick on about half of the clock's values (21 of 40 in the audit's reproduction; this suite's three frames,
        all in one tick, came out as 0 frames). Here the clock stands still on one tick through the start and the frames,
        for 40 clock values: every frame is kept every time, and none from before the start."""
        Tick = 0.015625
        for Index in range(40):
            Now = [round(4000.0 + Index * 37.0101 / 7.0, 6)]
            with Self.subTest(clock=Now[0]), patch.object(Bridge.time, "monotonic", lambda: Now[0]):
                Controller, Bus = MakeController()
                Recording(Controller)
                Timings = []
                Controller.TimingSink = Timings.append
                Controller.MotionTraceStart = Now[0] - 3 * Tick - Index * 0.000137
                Controller.refresh()  # a frame one tick before the route: not the route's
                Now[0] += Tick
                Gripper = Controller.servos["gripper"]
                Start = Gripper.present_raw
                Self.assertTrue(Controller.move_servo("gripper", Start + 120)[0])
                for _ in range(3):  # three frames in the start's own tick
                    Bus.step("gripper", steps_per_tick=10)
                    Controller.refresh()
                Bus.position["gripper"] = Start - 100
                Controller.refresh()  # knocked off its route: the fault records the route before this frame is kept
                Self.assertEqual(len(Timings), 1)
                Self.assertEqual(len(Timings[0]["frames"]), 3, Timings[0]["frames"])
                Self.assertTrue(all(Frame[0] == 0.0 for Frame in Timings[0]["frames"]))
                Started = Controller.MotionStartedAt - Controller.MotionTraceStart
                Self.assertTrue(any(Frame["t"] < Started - Tick / 2 for Frame in Controller.MotionTrace),
                                "the frame a tick before the route is in the trace, not in the route's record")

    def TestStateCarriesTheFaultReason(Self):
        Controller, Bus = MakeController()
        Recording(Controller)
        DelayWhileMoving(Controller)
        Controller.refresh()
        State = Controller.state_snapshot()
        Self.assertEqual(State["motion_fault"], "Motor control heartbeat was delayed")
        Self.assertEqual(State["protection"]["trip"]["reason"], "Motor control heartbeat was delayed")
        Self.assertEqual({Key: State["last_bridge_event"][Key] for Key in ("kind", "reason", "joint")},
                         {"kind": "protection_trip", "reason": "Motor control heartbeat was delayed", "joint": None})
        json.dumps(State)  # it goes out over the WebSocket every frame

    def TestEventsGoToTheDaysRecording(Self):
        with tempfile.TemporaryDirectory() as Root, patch.dict(os.environ, {"SO101_CAPTURES_ROOT": Root}):
            Controller, Bus = MakeController()
            Controller.EventSink = Bridge.AppendBridgeEvent
            DelayWhileMoving(Controller)
            Controller.refresh()
            Self.assertTrue(Bridge.FlushBridgeEvents(5.0))
            Lines = (SessionLog.RecordingDirectory() / "bridge_events.jsonl").read_text(encoding="utf-8").splitlines()
            # The trip, and the late frame's stall diagnostic beside it.
            Self.assertEqual([json.loads(Line)["kind"] for Line in Lines], ["protection_trip", "control_stall"])
            Record = json.loads(Lines[0])
            Self.assertEqual((Record["kind"], Record["reason"]), ("protection_trip", "Motor control heartbeat was delayed"))
            Self.assertAlmostEqual(Record["t"], time.time(), delta=30)
            Self.assertRegex(Record["time"], r"^\d{4}-\d\d-\d\dT\d\d:\d\d:\d\d\.\d{3}Z$")

    def TestAnUnwritableRecordNeverBreaksTheBridge(Self):
        with tempfile.TemporaryDirectory() as Root:
            Blocker = Path(Root) / "not-a-directory"
            Blocker.write_text("x")
            with patch.dict(os.environ, {"SO101_CAPTURES_ROOT": str(Blocker)}):
                Controller, Bus = MakeController()
                Controller.EventSink = Bridge.AppendBridgeEvent
                DelayWhileMoving(Controller)
                Controller.refresh()
                Self.assertTrue(Bridge.FlushBridgeEvents(5.0))
            Self.assertEqual(Controller.MotionFault, "Motor control heartbeat was delayed")
            # A sink that raises is contained too.
            Controller, Bus = MakeController()
            Controller.EventSink = lambda Record: 1 / 0
            DelayWhileMoving(Controller)
            Controller.refresh()
            Self.assertEqual(Controller.MotionFault, "Motor control heartbeat was delayed")

    def TestOnlyTheRunningServerRecords(Self):
        # Offline imports, scripts and the other test suites trip faults on fake motors all the time:
        # they must never write into the owner's session record.
        Self.assertIsNone(Bridge.BridgeController().EventSink)
        Self.assertIsNone(Bridge.controller.EventSink)
        Self.assertIsNone(Bridge.BridgeController().TimingSink)
        Self.assertIsNone(Bridge.controller.TimingSink)
        Controller, Bus = MakeController()

        class Loop:
            def start(Self): pass
            def stop(Self): pass

        class Camera:
            def Start(Self): pass
            def Stop(Self): pass

        Seen, Pools = [], []

        async def Run():
            # The server also starts the planning check workers (RouteCheckPool); not here.
            with patch.object(Bridge, "controller", Controller), patch.object(Bridge, "control_loop", Loop()), \
                    patch.object(Bridge, "depth_camera", Camera()), patch.object(Bridge, "RaiseProcessPriority", lambda: "normal"), \
                    patch.object(Bridge.RouteChecks, "Start", lambda: Pools.append("started")):
                async with Bridge._lifespan(Bridge.app):
                    Seen.append((Controller.EventSink, Controller.TimingSink))
        asyncio.run(Run())
        Self.assertEqual(Seen[0], (Bridge.AppendBridgeEvent, Bridge.AppendBridgeEvent))
        Self.assertEqual(Pools, ["started"])


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestBridgeEvents` runs the Test* methods too (unittest names this hook)."""
    Prefixed = unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return Prefixed.loadTestsFromTestCase(BridgeEventTests)


if __name__ == "__main__":
    Loader = unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Result = unittest.TextTestRunner(verbosity=2).run(Loader.loadTestsFromTestCase(BridgeEventTests))
    raise SystemExit(not Result.wasSuccessful())
