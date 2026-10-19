"""Offline motor safety regressions. Never opens a serial connection or the depth camera,
and never raises this process's priority."""
import asyncio
import threading
import time
import unittest
from unittest.mock import patch

from fastapi.testclient import TestClient

import bridge_server as Bridge
import feetech_simple as Driver
from test_bridge_home import FakeBus


def MakeController():
    Controller = Bridge.BridgeController()
    Bus = FakeBus()
    Controller.bus, Controller.port, Controller.torque_enabled = Bus, "FAKE", True
    Bus.torque_enabled = True
    for Name, Servo in Controller.servos.items():
        Bus.position[Name] = Servo.present_raw
    Bus.configure_motors()
    Controller.refresh()
    Bus.writes.clear()
    return Controller, Bus


class ClientAt(TestClient):
    """Starlette's test client, whose WebSockets also ask for its base_url's host (its own always name "testserver")."""

    def websocket_connect(Self, Url, *Args, **Options):
        if Url.startswith("/"):
            Url = "ws" + str(Self.base_url).rstrip("/")[4:] + Url
        return super().websocket_connect(Url, *Args, **Options)


def LocalClient():
    """The test client as this PC (127.0.0.1, asking for 127.0.0.1): the bridge serves anyone else only with the remote
    password (RemoteAccess.py), and the test client's default, "testclient" asking for "testserver", is not this PC."""
    return ClientAt(Bridge.app, base_url="http://127.0.0.1:8000", client=("127.0.0.1", 50000))


def Beat(Controller, Seconds):
    """Run the control loop at 30 Hz for Seconds, as the bridge does."""
    End = time.monotonic() + Seconds
    while time.monotonic() < End:
        Controller.refresh()
        time.sleep(1 / 30)


# The live bridge owns the depth camera. While this file runs, the real camera's opener and the
# priority raise only record that something reached them (the opener raises instead of opening).
HardwareCalls = []


def GuardedOpener():
    HardwareCalls.append("depth camera opened")
    raise RuntimeError("an offline test reached the real depth camera")


def setUpModule():
    for Patch in (patch.object(Bridge.depth_camera, "Opener", GuardedOpener),
                  patch.object(Bridge, "RaiseProcessPriority", lambda: HardwareCalls.append("priority raised") or "normal")):
        Patch.start()
        unittest.addModuleCleanup(Patch.stop)


class SafetyTests(unittest.TestCase):
    def TestCalibrationRetryRequiresExplicitHoldAfterProtection(Self):
        Controller, Bus = MakeController()
        Controller.StopMotion("Motion settling timed out", Latch=True)
        Controller.servos["shoulder_pan"].gui_angle_raw = 100
        Epoch = Controller.MotionEpoch
        Bus.writes.clear()
        Result = Controller.PrepareCalibration()
        Self.assertFalse(Result["ok"])
        Self.assertIsNotNone(Controller.MotionFault)
        Self.assertEqual(Result["state"]["motion_epoch"], Epoch)
        Self.assertFalse(Bus.writes, "Calibration retry must not clear a protection trip")
        Self.assertFalse(Bus.torque_enabled)
        Self.assertTrue(Controller.toggle_torque()[0])
        Self.assertTrue(Controller.PrepareCalibration()["ok"])
        Self.assertIsNone(Controller.MotionFault)

    def TestCalibrationRetryRejectsUnsafeRecovery(Self):
        Cases = ("hardware", "torque", "position", "mode", "moving", "feedback", "status-read", "heartbeat", "stream", "other-fault")
        for Case in Cases:
            with Self.subTest(Case=Case):
                Controller, Bus = MakeController()
                Controller.StopMotion("Motion settling timed out", Latch=True)
                Epoch = Controller.MotionEpoch
                Name = "wrist_roll"
                if Case == "hardware": Bus.registers[("Hardware_Error_Status", Name)] = 32
                if Case == "torque": Bus.torque_enabled = False
                if Case == "position": Bus.position[Name] += 100
                if Case == "mode": Bus.registers[("Operating_Mode", Name)] = 1
                if Case == "moving": Bus.registers[("Moving", Name)] = 1
                if Case == "heartbeat": Controller.last_refresh_s = 0
                if Case == "stream": Controller.servos[Name].command_raw = 123
                if Case == "other-fault": Controller.MotionFault = "Lost joint feedback: wrist_roll"
                if Case in ("feedback", "status-read"):
                    OriginalRead = Bus.read
                    FailedRegister = "Present_Position" if Case == "feedback" else "Torque_Enable"
                    def FailRead(Reg, Motor, **Options):
                        if Reg == FailedRegister and Motor == Name:
                            raise ConnectionError("simulated unreadable status")
                        return OriginalRead(Reg, Motor, **Options)
                    Bus.read = FailRead
                Bus.writes.clear()
                Result = Controller.PrepareCalibration()
                Self.assertFalse(Result["ok"], Result)
                Self.assertIsNotNone(Controller.MotionFault)
                Self.assertEqual(Controller.MotionEpoch, Epoch)
                Self.assertFalse(Bus.writes)

    def TestOneDroppedStatusPacketKeepsHolding(Self):
        # Every feedback read that can trip protection retries once; a servo that stays silent still trips.
        for Register, Reason in (("Present_Position", "Lost joint feedback: gripper"),
                                 ("Hardware_Error_Status", "Cannot read gripper protection/torque status"),
                                 ("Torque_Enable", "Cannot read gripper protection/torque status"),
                                 ("Present_Temperature", "Lost gripper protection feedback")):
            with Self.subTest(Register=Register):
                Controller, Bus = MakeController()
                OriginalRead = Bus.read
                Failures = {"left": 0}
                def FlakyRead(Reg, Motor, **Options):
                    if Reg == Register and Motor == "gripper":
                        for _ in range(Options.get("num_retry", 0) + 1):
                            if Failures["left"] <= 0:
                                return OriginalRead(Reg, Motor, **Options)
                            Failures["left"] -= 1
                        raise ConnectionError("simulated dropped status packet")
                    return OriginalRead(Reg, Motor, **Options)
                Bus.read = FlakyRead
                Bus.registers[("Present_Temperature", "gripper")] = 30
                Failures["left"] = 1
                Controller.LastProtectionPoll = 0
                Controller.refresh()
                Self.assertIsNone(Controller.MotionFault)
                Self.assertIsNone(Controller.ProtectionTrip)
                Self.assertTrue(Controller.torque_enabled)
                Failures["left"] = 100
                Controller.LastProtectionPoll = 0
                Controller.refresh()
                if Register == "Present_Temperature":
                    # A temperature is lost only on the second poll in a row (v2-confirmed): its position still reads.
                    Self.assertIsNone(Controller.ProtectionTrip)
                    Controller.LastProtectionPoll = 0
                    Controller.refresh()
                Fault = (Controller.MotionFault or "") + ((Controller.ProtectionTrip or {}).get("reason") or "")
                Self.assertIn(Reason, Fault)

    def TestStopKeepsLatchedFaultVisible(Self):
        Controller, _ = MakeController()
        Controller.StopMotion("Motion settling timed out", Latch=True)
        Controller.StopMotion()
        Self.assertIn("Motion settling timed out", Controller.status)
        Self.assertEqual(Controller.MotionFault, "Motion settling timed out")

    def TestStoppingPositionModeNeverSelectsUnlimitedSpeed(Self):
        Controller, Bus = MakeController()
        Bus.registers[("Goal_Position", "wrist_roll")] = 0
        Bus.writes.clear()
        Controller.StopMotion()
        Self.assertFalse(any(Reg == "Goal_Speed" and Value == 0 for Reg, _, Value in Bus.writes))
        Self.assertEqual(Bus.reg("wrist_roll", "Goal_Position"), Bus.position["wrist_roll"])

    def TestHoldPreservesWristEncoderTurnCount(Self):
        Controller, Bus = MakeController()
        Controller.toggle_torque()
        Bus.position["wrist_roll"] = 4096 + 1408
        Self.assertTrue(Controller.toggle_torque()[0])
        Self.assertEqual(Bus.reg("wrist_roll", "Goal_Position"), 4096 + 1408)
        Self.assertEqual(Controller.servos["wrist_roll"].present_raw, 1408)

    def TestHoldCorrectsOppositeWristLimitWindow(Self):
        Controller, Bus = MakeController()
        Controller.toggle_torque()
        Bus.registers[("Min_Position_Limit", "wrist_roll")] = 289
        Bus.registers[("Max_Position_Limit", "wrist_roll")] = 501
        Bus.position["wrist_roll"] = 1408
        Self.assertTrue(Controller.toggle_torque()[0])
        Self.assertEqual(Bus.reg("wrist_roll", "Min_Position_Limit"), 0)
        Self.assertEqual(Bus.reg("wrist_roll", "Max_Position_Limit"), 4095)
        Self.assertEqual(Bus.reg("wrist_roll", "Goal_Position"), 1408)

    def TestHoldCannotEnableUnconfirmedPositionMode(Self):
        Controller, Bus = MakeController()
        Controller.toggle_torque()
        Write = Bus.write
        Bus.registers[("Operating_Mode", "wrist_roll")] = 1
        def IgnoreMode(Reg, Name, Value, **Options):
            if Reg == "Operating_Mode" and Name == "wrist_roll":
                return
            Write(Reg, Name, Value, **Options)
        Bus.write = IgnoreMode
        Ok, Message = Controller.toggle_torque()
        Self.assertFalse(Ok)
        Self.assertIn("read-back", Message)
        Self.assertFalse(Bus.torque_enabled)
        Self.assertEqual(Bus.reg("wrist_roll", "Goal_Speed"), 0)

    def TestCoastHomeParksWristBeforeTorque(Self):
        Controller, Bus = MakeController()
        Controller.ValidateMotionPath = lambda Targets, Calibration=False, **Options: None
        Wrist = Controller.servos["wrist_roll"]
        Controller._send_slider_target_locked(Wrist, Wrist.present_raw + 200)
        Self.assertTrue(Controller.toggle_torque()[0])
        Bus.writes.clear()
        Original = Bus.enable_torque
        def Enable(num_retry=0):
            Self.assertEqual(Bus.reg("wrist_roll", "Operating_Mode"), 0)
            Self.assertEqual(Bus.reg("wrist_roll", "Goal_Position"), Bus.position["wrist_roll"])
            Original(num_retry)
        Bus.enable_torque = Enable
        Self.assertTrue(Controller.go_home()[0])
        Self.assertIsNone(Wrist.command_raw)

    def TestLostFeedbackStopsAllStreams(Self):
        Controller, Bus = MakeController()
        Servo = Controller.servos["elbow_flex"]
        Controller._send_slider_target_locked(Servo, Servo.present_raw - 200)
        Read = Bus.read
        def Fail(Reg, Name, **Options):
            if Reg == "Present_Position" and Name == "elbow_flex":
                raise ConnectionError("simulated cable loss")
            return Read(Reg, Name, **Options)
        Bus.read = Fail
        Bus.writes.clear()
        Controller.refresh()
        Self.assertIn("elbow_flex", Controller.MotionFault)
        Self.assertEqual(Bus.reg("elbow_flex", "Goal_Speed"), 0)
        Self.assertFalse(any(S.command_raw is not None for S in Controller.servos.values()))
        Self.assertFalse(Controller.move_servo("gripper", 800)[0])
        Self.assertFalse(Bus.torque_enabled, "User-selected protection policy coasts all joints")

    def TestProtectionDoesNotGetClearedByPositionWrite(Self):
        Controller, Bus = MakeController()
        Bus.registers[("Hardware_Error_Status", "shoulder_lift")] = 32
        Controller.refresh()
        Self.assertIn("overload", Controller.MotionFault)
        Self.assertFalse(any(Reg == "Goal_Position" and Name == "shoulder_lift" for Reg, Name, _ in Bus.writes))
        Bus.registers[("Hardware_Error_Status", "shoulder_lift")] = 0
        Controller.refresh()
        Self.assertIsNotNone(Controller.MotionFault, "A disappearing status bit must not resume motion")

    def TestUnexpectedTorqueLossDetectedEvenBeforeMotion(Self):
        Controller, Bus = MakeController()
        Bus.torque_enabled = False
        Controller.refresh()
        Self.assertIn("torque unexpectedly off", Controller.MotionFault)

    def TestEmergencyTriesEveryMotor(Self):
        Bus = Bridge._make_bus("FAKE")
        Writes = []
        def Write(Reg, Name, Value, **Options):
            Writes.append((Reg, Name, Value))
            if Name == "shoulder_pan":
                raise ConnectionError("first motor missing")
        Bus.write = Write
        with Self.assertRaises(ConnectionError):
            Bus.disable_torque()
        Self.assertEqual({Name for Reg, Name, _ in Writes if Reg == "Torque_Enable"}, set(Bridge.MOTOR_NAMES))

    def TestEmergencyReportsFailure(Self):
        Controller, Bus = MakeController()
        Bus.disable_torque = lambda **Options: (_ for _ in ()).throw(ConnectionError("unconfirmed"))
        Ok, Message = Controller.emergency_stop()
        Self.assertFalse(Ok)
        Self.assertIn("unconfirmed", Message)

    def TestFaultPacketsRemainReadable(Self):
        Bus = Bridge._make_bus("FAKE")
        Bus._raw_read = lambda *Args: (2450, Driver.scs.COMM_SUCCESS, 32)
        Self.assertEqual(Bus.read("Present_Position", "elbow_flex"), 2450)
        Bus._raw_read = lambda *Args: (0, Driver.scs.COMM_SUCCESS, 32)
        Self.assertEqual(Bus.read("Hardware_Error_Status", "elbow_flex"), 32)

    def TestEmergencyCanInterruptNumericalSolve(Self):
        Controller, Bus = MakeController()
        Entered, Release = threading.Event(), threading.Event()
        Results = []
        OriginalSolve = Bridge.Cad.SolveCandidates
        def Solve(*Args, **Options):
            Entered.set()
            Self.assertTrue(Release.wait(3))
            return OriginalSolve(*Args, **Options)
        with patch.object(Bridge.Cad, "SolveCandidates", Solve):
            Thread = threading.Thread(target=lambda: Results.append(Controller.send_ik()))
            Thread.start()
            Self.assertTrue(Entered.wait(2))
            Self.assertTrue(Controller.lock.acquire(timeout=0.3), "Solver must release control lock")
            Controller.lock.release()
            Controller.emergency_stop()
            WritesAfterStop = len(Bus.writes)
            Release.set()
            Thread.join(4)
        Self.assertFalse(Thread.is_alive())
        Self.assertFalse(Results[0][0])
        Self.assertEqual(len(Bus.writes), WritesAfterStop)

    def TestUnreachableAndNonfiniteTargetsRejected(Self):
        Controller, Bus = MakeController()
        Ok, Message = Controller.send_ik(Target={"x": 0, "y": 20, "z": -100})
        Self.assertFalse(Ok)
        Self.assertIn("unreachable", Message)
        Self.assertFalse(Controller.set_target({"z": float("nan")})[0])
        Self.assertFalse(Bus.writes)
        with Self.assertRaises(ValueError):
            Controller.ValidateMotionPath({"shoulder_lift": Controller.servos["shoulder_lift"].range_min,
                                           "elbow_flex": Controller.servos["elbow_flex"].range_min})

    def TestCalibrationUsesSlowFeedbackThenPositionHold(Self):
        Controller, Bus = MakeController()
        Angles = Controller._current_ik_angles()
        Angles["shoulder_pan"] += 3
        Controller.CalibrationPlans["test"] = {"angles": Angles, "configuration": Controller.ConfigurationKey()}
        # Load policy is independent of mode sequencing, tested below.
        with patch.object(Bridge.BridgeController, "ValidateMotionPath", lambda *Args, **Options: None):
            Ok, Message = Controller.send_ik(slow=True, PlanId="test")
        Self.assertTrue(Ok, Message)
        for _ in range(60):
            for Name in Bridge.MOTOR_NAMES:
                Bus.step(Name, steps_per_tick=8)
            Controller.refresh()
        Self.assertFalse(Controller.MotionFault)
        Self.assertFalse(any(Reg == "Torque_Enable" and Value == 0 for Reg, _, Value in Bus.writes))
        Self.assertTrue(any(Reg == "Operating_Mode" and Value == 1 for Reg, _, Value in Bus.writes))
        Self.assertTrue(all(Bus.reg(Name, "Operating_Mode") == 0 for Name in Bridge.MOTOR_NAMES))
        Self.assertFalse(any(S.command_raw is not None for S in Controller.servos.values()))
        Self.assertTrue(Bus.torque_enabled)

    def TestCalibrationElbowMovesWhenSmallPositionGoalsStall(Self):
        Controller, Bus = MakeController()
        # Reproduce the live first-pose elbow error with a servo that cannot
        # overcome load using the old tiny position-command lead. Velocity mode
        # still works, as it does for this arm's Home and manual controls.
        Name = "elbow_flex"
        Servo = Controller.servos[Name]
        Servo.present_raw = Servo.target_raw = Servo.HardwareRaw = Bus.position[Name] = 2669
        Bus.registers[("Goal_Position", Name)] = 2669
        Goal = 2364
        Angles = Controller._current_ik_angles()
        Angles[Name] = Controller._ik_angle_from_raw(Servo, Goal)
        Controller.CalibrationPlans["loaded"] = {"angles": Angles, "configuration": Controller.ConfigurationKey()}
        with patch.object(Bridge.BridgeController, "ValidateMotionPath", lambda *Args, **Options: None):
            Self.assertTrue(Controller.send_ik(slow=True, PlanId="loaded")[0])
        for _ in range(160):
            for Motor in Bridge.MOTOR_NAMES:
                if Motor == Name and Bus.reg(Motor, "Operating_Mode") == 0:
                    continue
                Bus.step(Motor, steps_per_tick=4)
            Controller.refresh()
        Self.assertLessEqual(abs(Servo.present_raw - Goal), Bridge.SLIDER_GOAL_TOLERANCE_RAW)
        Self.assertIsNone(Servo.command_raw)
        Self.assertFalse(Controller.MotionFault)
        Self.assertEqual(Bus.reg(Name, "Operating_Mode"), 0)
        Self.assertFalse(any(Reg == "Torque_Enable" and Value == 0 for Reg, _, Value in Bus.writes))
        Self.assertFalse(any(Motor in ("wrist_roll", "gripper") for _, Motor, _ in Bus.writes))
        # Only velocity-mode speed writes are relevant; position Hold uses its
        # normal profile while remaining at the reached goal.
        Modes = dict.fromkeys(Bridge.MOTOR_NAMES, 0)
        for Reg, Motor, Value in Bus.writes:
            if Reg == "Operating_Mode": Modes[Motor] = Value
            if Reg == "Goal_Speed" and Modes[Motor] == 1:
                Self.assertLessEqual(abs(Value), 315)  # 18% startup of the 1750 top ceiling (the old 30% of 1050).

    def TestCalibrationSagCorrectionStaysSlow(Self):
        Controller, Bus = MakeController()
        Controller.CalibrationActive = True
        Servo = Controller.servos["elbow_flex"]
        Servo.Holding = True
        Bus.position[Servo.name] -= 200
        Controller.refresh()
        Self.assertTrue(Servo.homing)
        Self.assertLessEqual(abs(Servo.command_speed), Bridge.HOME_VELOCITY_MAX_SPEED)

    def TestCalibrationStallNamesJointAndStopsStreams(Self):
        Controller, Bus = MakeController()
        Servo = Controller.servos["elbow_flex"]
        Servo.homing = True
        Controller._send_slider_target_locked(Servo, Servo.present_raw - 200)
        Controller.MotionDeadline = 0.001
        Controller.refresh()
        Self.assertIn("Motion settling timed out: elbow flex error 200 raw steps", Controller.MotionFault)
        Self.assertFalse(any(S.command_raw is not None for S in Controller.servos.values()))
        Self.assertFalse(Bus.torque_enabled)
        Self.assertFalse(Controller.PrepareCalibration()["ok"])

    def TestStateKeepsFlowingWhileACommandRuns(Self):
        # 2026-09-28: send_ik planned for 7-9 s, the sending client got no state meanwhile, and
        # the app took the frozen pre-motion pose for "arrived" and photographed it.
        class SlowController:
            def __init__(Self):
                Self.Snapshots, Self.Spans = 0, []
                Self.Lock = threading.Lock()

            def state_snapshot(Self):
                with Self.Lock:
                    Self.Snapshots += 1
                    return {"type": "state", "n": Self.Snapshots}

            def handle_command(Self, Message, Access="local"):
                Started = time.monotonic()
                time.sleep(0.8)
                Self.Spans.append((Started, time.monotonic()))
                return True, f"done {Message['type']}"

        Fake = SlowController()
        with patch.object(Bridge, "controller", Fake), LocalClient().websocket_connect("/ws") as Socket:
            Self.assertEqual(Socket.receive_json()["type"], "state")
            Socket.send_json({"type": "send_ik"})
            Socket.send_json({"type": "go_home"})
            Messages, Started = [], time.monotonic()
            while sum(Message.get("type") == "status" for Message in Messages) < 2 and time.monotonic() - Started < 6:
                Messages.append(Socket.receive_json())
        Replies = [Index for Index, Message in enumerate(Messages) if Message.get("type") == "status"]
        Self.assertEqual([Messages[Index]["text"] for Index in Replies], ["done send_ik", "done go_home"])
        Self.assertGreater(sum(Message.get("type") == "state" for Message in Messages[:Replies[0]]), 10,
                           "state stopped while the first command ran")
        # Still one command at a time, each reply followed by its state.
        Self.assertLessEqual(Fake.Spans[0][1], Fake.Spans[1][0])
        Self.assertEqual(Messages[Replies[0] + 1]["type"], "state")

    def TestCapturePoseOnlyWhileTheArmHeldStill(Self):
        Controller, Bus = MakeController()
        Self.assertEqual(Bridge.BridgeController().CaptureArmPose(time.time())["reason"], "arm not connected")
        Controller.refresh()
        Self.assertEqual(Controller.CaptureArmPose(time.time() - 5.0)["reason"], "no joint record back to the capture")
        Beat(Controller, 0.3)
        Captured = time.time() - 0.05
        Pose = Controller.CaptureArmPose(Captured)
        Self.assertTrue(Pose["ok"], Pose)
        Offset = Controller.WorkspaceOffset()
        Drawn = Controller.state_snapshot()["joint_positions"]  # what the app draws, before its display offset
        Self.assertEqual(len(Pose["joints"]), len(Drawn))
        for Joint, Point in zip(Pose["joints"], Drawn):
            Self.assertAlmostEqual(Joint[0], Point["x"] + Offset["x"])
            Self.assertAlmostEqual(Joint[1], Point["y"] + Offset["y"])
        Bus.position["elbow_flex"] += 40
        Controller.refresh()
        Self.assertEqual(Controller.CaptureArmPose(Captured)["reason"], "arm moved since the capture")

    def TestCapturePoseNeedsAnUnbrokenJointRecordSinceTheCapture(Self):
        # The joint record runs only while torque is on: a limp arm moved by hand after the capture
        # went unseen, and /locate masked the arm and predicted the tape at the new pose.
        Controller, Bus = MakeController()
        Beat(Controller, 0.3)
        Controller.torque_enabled = Bus.torque_enabled = False
        Beat(Controller, 0.6)
        Captured = time.time()
        Beat(Controller, 0.2)
        Bus.position["elbow_flex"] += 400
        Bus.position["shoulder_lift"] += 300
        Controller.refresh()
        Self.assertEqual(Controller.CaptureArmPose(Captured), {"ok": False, "reason": "torque off"})
        # A record with a hole across the capture (torque back on since, or a stalled loop) is not a still arm.
        Controller, Bus = MakeController()
        Beat(Controller, 0.6)
        Captured = time.time() - 0.3
        Self.assertTrue(Controller.CaptureArmPose(Captured)["ok"])
        Hole = (Controller.MotionTrace[-1]["t"] - 0.4, Controller.MotionTrace[-1]["t"] - 0.05)
        Kept = [Frame for Frame in Controller.MotionTrace if not Hole[0] < Frame["t"] < Hole[1]]
        Controller.MotionTrace.clear()
        Controller.MotionTrace.extend(Kept)
        Self.assertEqual(Controller.CaptureArmPose(Captured)["reason"], "joint record interrupted since the capture")
        # Nor is a record that stopped: no frame for the last 0.4 s.
        Controller, Bus = MakeController()
        Beat(Controller, 0.4)
        Captured = time.time() - 0.05
        time.sleep(0.4)
        Self.assertEqual(Controller.CaptureArmPose(Captured)["reason"], "joint record interrupted since the capture")

    def TestShutdownStopsMotors(Self):
        # The lifespan also raises the process priority, starts the depth camera, starts the planning
        # check workers (RouteCheckPool: up to 8 processes) and sends fault records to the session
        # record: stand-ins here, so the test reaches none of them.
        Controller, Bus = MakeController()
        Events, Records = [], []
        class Loop:
            def start(Self): Events.append("loop start")
            def stop(Self): Events.append("loop stop")
        class Camera:
            def Start(Self): Events.append("camera start")
            def Stop(Self): Events.append("camera stop")
        async def Run():
            with patch.object(Bridge, "controller", Controller), patch.object(Bridge, "control_loop", Loop()), \
                    patch.object(Bridge, "depth_camera", Camera()), patch.object(Bridge, "AppendBridgeEvent", Records.append), \
                    patch.object(Bridge, "RaiseProcessPriority", lambda: Events.append("priority") or "normal"), \
                    patch.object(Bridge.RouteChecks, "Start", lambda: Events.append("check workers")):
                async with Bridge._lifespan(Bridge.app):
                    pass
        asyncio.run(Run())
        Self.assertFalse(Bus.torque_enabled)
        Self.assertFalse(Bus.is_connected)
        Self.assertEqual(Events, ["priority", "loop start", "camera start", "check workers", "loop stop", "camera stop"])
        Self.assertEqual(HardwareCalls, [], "the lifespan reached the real depth camera or process priority")


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestSafety` runs the Test* methods too (unittest names this hook)."""
    Prefixed = unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return Prefixed.loadTestsFromTestCase(SafetyTests)


if __name__ == "__main__":
    Loader = unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Result = unittest.TextTestRunner(verbosity=2).run(Loader.loadTestsFromTestCase(SafetyTests))
    raise SystemExit(not Result.wasSuccessful())
