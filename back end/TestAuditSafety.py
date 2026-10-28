"""The 2026-10-07 safety audit's motion findings, on the fake motor bus (no serial port, no camera, no live bridge).

H1  a measured scene that changes or expires during its route stops the route holding, never coasting; a scene that
    would expire mid-route refuses at dispatch; /locate refuses while such a route runs; a wall-clock step changes nothing.
H2  Set Home, a joint-limit change, a current-angle override, staging a target or Home itself wait for the arm to stop;
    a configuration that changes mid-route anyway stops it holding.
H3  Hold holds every joint where it is, also one hand-posed past its slider limit; only a commanded move brings it back.
M8  narrowing a limit past a holding joint does not move it.
    (2026-10-08 review) a joint held in the middle of its excluded window reads its jitter there as jitter, not a turn;
    a stop that holds where it is keeps a grasp's squeeze.
H4  a slow connect to servos that come out of it holding does not trip the heartbeat on the first frame.
M1  Home and manual moves are never scene-bound; the scene is dropped when its route ends.
M2  Stop, Hold and Coast run ahead of a route still planning, which ends at once; commands queued before them never run.
M10 the hold correction in the control frame never checks a measured scene; binding one happens off the motor lock.
M7  the GUI's current-angle override changes no planning or safety input.
L4  the emergency stop reads its torque-off back.
S4  the control loop survives a coast that fails too.
L3  a late or long control frame is recorded with what it waited for; a stalled one dumps every stack, with its time.
A2  the bridge reports the code version the page and RestartBridge.ps1 require.
(2026-10-25 stalls) no file is read under the motor lock; a depth-camera open and a motion never overlap; no motion
    during the startup warm-up (the check workers' start and the camera's first open).

Run:  python TestAuditSafety.py
"""
import asyncio
import json
import os
import shutil
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

os.environ.setdefault("SO101_DEPTH_CAMERA", "0")
os.environ.setdefault("SO101_RECORDER", "0")


import BasketCollision as Collision  # noqa: E402
import SceneSafety  # noqa: E402
import bridge_server as Bridge  # noqa: E402
import calibration  # noqa: E402
import LoadScale  # noqa: E402
import OrbbecCamera  # noqa: E402
from TestSafety import LocalClient, MakeController  # noqa: E402
from test_bridge_home import FakeBus  # noqa: E402

Root = Path(__file__).resolve().parent


def Frames(Controller, Bus, Count, Step=10):
    """Count control frames, each motor stepping toward its command first (FakeBus.step)."""
    for _ in range(Count):
        for Name in Bridge.MOTOR_NAMES:
            Bus.step(Name, steps_per_tick=Step)
        Controller.refresh()


def Holding(Self, Controller, Bus):
    """The arm stopped and holds: no route, no fault, no trip, torque on in the bridge and on every servo."""
    Self.assertIsNone(Controller.MotionRoute)
    Self.assertIsNone(Controller.MotionFault, Controller.status)
    Self.assertIsNone(Controller.ProtectionTrip)
    Self.assertTrue(Controller.torque_enabled)
    Self.assertTrue(Bus.torque_enabled)


def SyntheticScene(Obstacles=None):
    """A registered scene: small obstacles far from the arm, the whole workspace observed free (as ReproBridge)."""
    Scene = {"frame": "display_cm", "capture_seq": 1, "support": {"TopZCm": -2.3},
             "obstacles": Obstacles or [{"min": [60.0, 70.0, 60.0], "max": [61.0, 71.0, 61.0], "label": "far box"}],
             "free_space": [{"min": [-65.0, -25.0, -20.0], "max": [65.0, 80.0, 65.0]}]}
    SceneSafety.Invalidate()
    return SceneSafety.Register(Scene, calibration.camera_model())


def Age(Scene, Seconds):
    """Make a registered scene Seconds old (its monotonic clock), as if it was captured that long ago."""
    SceneSafety._clock[Scene["id"]] = (time.monotonic() - Seconds, 0.0)


def ScratchController():
    """MakeController with its Home and limit files in a scratch folder (Set Home and limit edits write them)."""
    Scratch = Path(tempfile.mkdtemp(prefix="TestAuditSafety"))
    shutil.copy(Root / "home_preset.json", Scratch / "home_preset.json")
    shutil.copy(Root / "gui_limits.json", Scratch / "gui_limits.json")
    Controller, Bus = MakeController()
    Controller.home_path, Controller.limits_path = Scratch / "home_preset.json", Scratch / "gui_limits.json"
    return Controller, Bus, Scratch


def GripperRoute(Controller, Scene=None, Steps=300):
    """A gripper-only route (the jaws open Steps), bound to Scene when given."""
    Gripper = Controller.servos["gripper"]
    Target = Controller._clamp_raw(Gripper, Gripper.present_raw + Steps)
    if Scene is None:
        return Controller.move_servo("gripper", Target)
    return Controller.move_servo("gripper", Target, Scene["id"], True, False)


class SceneRouteTests(unittest.TestCase):
    """H1, M1: the measured scene of a scene-bound route."""

    def setUp(Self):
        SceneSafety.Invalidate()
        Self.addCleanup(SceneSafety.Invalidate)

    def TestSceneExpiringMidRouteHoldsInsteadOfCoasting(Self):
        Controller, Bus = MakeController()
        Scene = SyntheticScene()
        Ok, Text = GripperRoute(Controller, Scene)
        Self.assertTrue(Ok, Text)
        Frames(Controller, Bus, 3)
        Self.assertIsNotNone(Controller.MotionRoute)
        Age(Scene, SceneSafety.MaximumAgeSeconds + 1)
        Frames(Controller, Bus, 1)
        Holding(Self, Controller, Bus)
        Self.assertIn("measured scene changed or expired", Controller.status)
        # The joints were told to hold where they are (position mode), not left coasting or streaming.
        Self.assertTrue(all(Bus.reg(Name, "Operating_Mode") == 0 for Name in Bridge.MOTOR_NAMES))
        Self.assertIsNone(Controller.ActiveScene, "the scene is dropped with its route (M1)")

    def TestLocateInvalidatingMidRouteHoldsInsteadOfCoasting(Self):
        Controller, Bus = MakeController()
        Scene = SyntheticScene()
        Self.assertTrue(GripperRoute(Controller, Scene)[0])
        Frames(Controller, Bus, 3)
        SceneSafety.Invalidate()  # what every /locate does first
        Frames(Controller, Bus, 1)
        Holding(Self, Controller, Bus)
        Self.assertIn("measured scene changed or expired", Controller.status)

    def TestWallClockStepDoesNotExpireTheScene(Self):
        # A time sync stepping the clock used to expire a scene mid-route (SceneSafety aged scenes by wall clock).
        Controller, Bus = MakeController()
        Scene = SyntheticScene()
        Self.assertTrue(GripperRoute(Controller, Scene)[0])
        Frames(Controller, Bus, 2)
        Later = time.time() + SceneSafety.MaximumAgeSeconds + 1000
        with patch.object(SceneSafety.time, "time", lambda: Later):
            Frames(Controller, Bus, 2)
            Self.assertTrue(SceneSafety.IsCurrent(Scene["id"]))
        Self.assertIsNotNone(Controller.MotionRoute, Controller.status)
        Self.assertIsNone(Controller.MotionFault)

    def TestSceneThatWouldExpireMidRouteRefusesAtDispatch(Self):
        Controller, Bus = MakeController()
        Scene = SyntheticScene()
        Age(Scene, SceneSafety.MaximumAgeSeconds - 5)
        Bus.writes.clear()
        Ok, Text = GripperRoute(Controller, Scene)
        Self.assertFalse(Ok)
        Self.assertIn("expires in", Text)
        Self.assertIsNone(Controller.MotionRoute)
        Self.assertIsNone(Controller.MotionFault)
        Self.assertFalse(Bus.writes, "refused before anything was written")
        Self.assertTrue(Controller.torque_enabled and Bus.torque_enabled)

    def TestSceneGoneBeforeDispatchRefusesWithoutCoasting(Self):
        # The scene changed while the route planned: it used to latch a fault (AdoptScene raised inside the latch).
        Controller, Bus = MakeController()
        Scene = SyntheticScene()
        Original = Bridge.BridgeController.PlanMotionRoute

        def PlanThenInvalidate(Snapshot, *Args, **Options):
            Path = Original(Snapshot, *Args, **Options)
            SceneSafety.Invalidate()
            return Path
        with patch.object(Bridge.BridgeController, "PlanMotionRoute", PlanThenInvalidate):
            Ok, Text = GripperRoute(Controller, Scene)
        Self.assertFalse(Ok)
        Self.assertIn("scene changed", Text)
        Self.assertIsNone(Controller.MotionFault)
        Self.assertTrue(Controller.torque_enabled and Bus.torque_enabled)

    def TestLocateRefusesWhileASceneBoundRouteRuns(Self):
        Controller, Bus = MakeController()
        Scene = SyntheticScene()
        Self.assertTrue(GripperRoute(Controller, Scene)[0])
        Frames(Controller, Bus, 2)
        with patch.object(Bridge, "controller", Controller), patch.object(Bridge.object_locator, "locate_objects") as Locate:
            Reply = Bridge.locate({"image": "data:image/jpeg;base64,AAAA"})
        Locate.assert_not_called()
        Self.assertEqual((Reply["ok"], Reply["error_code"]), (False, "arm_moving"))
        Self.assertTrue(SceneSafety.IsCurrent(Scene["id"]), "the route's scene was not revoked")
        Self.assertIsNotNone(Controller.MotionRoute)

    def TestHomeAndManualMovesAreNeverSceneBound(Self):
        Controller, Bus = MakeController()
        Scene = SyntheticScene()
        Self.assertTrue(GripperRoute(Controller, Scene, Steps=60)[0])
        Frames(Controller, Bus, 60)
        Self.assertIsNone(Controller.MotionRoute)
        Self.assertIsNone(Controller.ActiveScene, "dropped when its route ended")
        SceneSafety.Invalidate()  # the next /locate began (or a capture failed): the old scene is gone
        Self.assertTrue(Controller.move_servo("gripper", Controller.servos["gripper"].present_raw - 40)[0], Controller.status)
        Frames(Controller, Bus, 60)
        Ok, Text = Controller.go_home()
        Self.assertTrue(Ok, Text)


class ConfigurationTests(unittest.TestCase):
    """H2, H3, M8: what may change while the arm moves, and what Hold and limit edits may move."""

    def setUp(Self):
        SceneSafety.Invalidate()
        Self.Controller, Self.Bus, Scratch = ScratchController()
        Self.addCleanup(shutil.rmtree, Scratch, True)

    def Moving(Self):
        Self.assertTrue(GripperRoute(Self.Controller)[0])
        Frames(Self.Controller, Self.Bus, 3)
        Self.assertIsNotNone(Self.Controller.MotionRoute)

    def TestSettingsWaitForTheArmToStop(Self):
        Controller = Self.Controller
        Lift = Controller.servos["shoulder_lift"]
        Changes = {"set Home": lambda: Controller.set_home(),
                   "limits": lambda: Controller.set_servo_limits({"name": "shoulder_lift", "min": Lift.range_min + 10,
                                                                  "max": Lift.range_max, "arc_inverted": Lift.arc_inverted}),
                   "current angle": lambda: Controller.set_gui_angle({"name": "shoulder_lift", "raw": Lift.present_raw + 100}),
                   "target": lambda: Controller.set_target({"x": 1.0}),
                   "Home": lambda: Controller.go_home()}
        for Name, Change in Changes.items():
            with Self.subTest(Name):
                Self.Moving()
                Home, Limits = dict(Controller.home_preset), (Lift.range_min, Lift.range_max)
                Ok, Text = Change()
                Self.assertFalse(Ok, Text)
                Self.assertIn("Wait for the arm to stop", Text)
                Self.assertEqual(Controller.home_preset, Home)
                Self.assertEqual((Lift.range_min, Lift.range_max), Limits)
                Self.assertIsNone(Lift.gui_angle_raw)
                # The route flies on.
                Frames(Controller, Self.Bus, 1)
                Self.assertIsNotNone(Controller.MotionRoute)
                Self.assertIsNone(Controller.MotionFault)
                Frames(Controller, Self.Bus, 90)
                Self.assertIsNone(Controller.MotionRoute)
        # Stopped, each is accepted.
        Self.assertTrue(Controller.set_home()[0])

    def TestConfigurationChangingMidRouteAnywayHolds(Self):
        # A configuration file edited on disk mid-route (WorkspaceFrame.json): the route stops holding, no coast.
        Controller = Self.Controller
        Self.Moving()
        with patch.object(Bridge.BridgeController, "ConfigurationKey", lambda Controller_: "changed"):
            Frames(Controller, Self.Bus, 1)
        Holding(Self, Controller, Self.Bus)
        Self.assertIn("configuration", Controller.status)

    def TestHoldHoldsAJointOutsideItsLimitsWhereItIs(Self):
        # audit S5: elbow hand-posed 150 steps past its slider limit; Hold drove it back 150 steps, unchecked.
        Controller, Bus = Self.Controller, Self.Bus
        Controller.toggle_torque()  # Coast
        Self.assertFalse(Controller.torque_enabled)
        Elbow = Controller.servos["elbow_flex"]
        Outside = (Elbow.range_max - 150) % 4096 if Elbow.arc_inverted else (Elbow.range_max + 150) % 4096
        Bus.position["elbow_flex"] = Outside
        Controller.refresh()
        Bus.writes.clear()
        Ok, Text = Controller.toggle_torque()  # Hold
        Self.assertTrue(Ok, Text)
        Self.assertIn("elbow flex is outside its joint limits, held where it stands", Text)
        Self.assertFalse([W for W in Bus.writes if W[1] == "elbow_flex" and W[0] == "Operating_Mode" and W[2] == 1],
                         "no velocity stream for the elbow")
        Self.assertEqual(Elbow.target_raw, Outside)
        Frames(Controller, Bus, 30)
        Self.assertEqual(Bus.position["elbow_flex"], Outside, "the elbow did not move")
        Self.assertTrue(Controller.state_snapshot()["servos"]["elbow_flex"]["outside_limits"])
        Self.assertIsNone(Controller.MotionFault)
        # Sagging off its hold point, it is still corrected back to where it was held (not to the limit).
        Bus.position["elbow_flex"] = (Outside + (-40 if Elbow.arc_inverted else 40)) % 4096
        Frames(Controller, Bus, 30, Step=4)
        Self.assertLessEqual(Controller._circular_raw_distance(Bus.position["elbow_flex"], Outside), Bridge.SLIDER_GOAL_TOLERANCE_RAW)
        # Back into range only by a commanded, checked move: the slider's target is within the limits.
        Ok, Text = Controller.move_servo("elbow_flex", Elbow.range_max)
        Self.assertTrue(Ok, Text)
        Frames(Controller, Bus, 120)
        Self.assertTrue(Controller._contains_raw(Elbow, Bus.position["elbow_flex"]) or
                        Controller._circular_raw_distance(Bus.position["elbow_flex"], Elbow.range_max) <= Bridge.SLIDER_GOAL_TOLERANCE_RAW)

    def TestNarrowingALimitWhileHoldingDoesNotMoveTheJoint(Self):
        # audit M8: the wrist_flex limit moved 150 steps past the joint; the hold correction moved it 150 steps.
        Controller, Bus = Self.Controller, Self.Bus
        Controller.toggle_torque()
        Controller.toggle_torque()  # Hold, every joint holding
        Frames(Controller, Bus, 10)
        Wrist = Controller.servos["wrist_flex"]
        Before = Bus.position["wrist_flex"]
        Limits = {"name": "wrist_flex", "min": Wrist.range_min, "max": Wrist.range_max, "arc_inverted": Wrist.arc_inverted}
        if Controller._arc_distance_from_left(Wrist, Wrist.present_raw) < Controller._arc_span(Wrist) / 2:
            Limits["min"] = (Wrist.present_raw + (-150 if Wrist.arc_inverted else 150)) % 4096
        else:
            Limits["max"] = (Wrist.present_raw - (-150 if Wrist.arc_inverted else 150)) % 4096
        Ok, Text = Controller.set_servo_limits(Limits)
        Self.assertTrue(Ok, Text)
        Self.assertIn("outside its joint limits", Text)
        Frames(Controller, Bus, 60)
        Self.assertLessEqual(Controller._circular_raw_distance(Bus.position["wrist_flex"], Before), 2)
        Self.assertIsNone(Controller.MotionFault)
        Self.assertIsNone(Controller.MotionRoute)


def WindowMiddle(Controller, Name):
    """The two raws a step apart in the middle of a joint's excluded window that _slider_track_pos reads a turn apart
    (past the nearer arc end: the wrist roll's 472 and 473), as (left side, right side)."""
    Servo = Controller.servos[Name]
    Span = Controller._arc_span(Servo)
    for Raw in range(4096):
        Next = (Raw + 1) % 4096
        if not Controller._contains_raw(Servo, Raw) and not Controller._contains_raw(Servo, Next):
            Here, There = Controller._slider_track_pos(Servo, Raw), Controller._slider_track_pos(Servo, Next)
            if abs(Here - There) > Span:
                return (Raw, Next) if Here < 0 else (Next, Raw)
    raise AssertionError(f"{Name} has no excluded window")


class WindowMiddleTests(unittest.TestCase):
    """2026-10-08 safety review: a joint held in the middle of its excluded window (hold-v3 holds a hand-posed joint
    where it stands) read a step of jitter there as a turn: 4094 steps off its hold point or its route, a latched fault
    and every joint coasting. The wrist roll's window is 196 steps wide (17 deg) and the wrist turns freely by hand."""

    def setUp(Self):
        SceneSafety.Invalidate()
        Self.Fresh()

    def Fresh(Self):
        Self.Controller, Self.Bus, Scratch = ScratchController()
        Self.addCleanup(shutil.rmtree, Scratch, True)

    def PoseAndHold(Self, Name, Raw):
        """Coast, turn the joint to Raw by hand, Hold: it holds there (outside its limits)."""
        Controller, Bus = Self.Controller, Self.Bus
        Self.assertTrue(Controller.toggle_torque(False)[0])
        Bus.position[Name] = Raw
        Controller.refresh()
        Ok, Text = Controller.toggle_torque(True)
        Self.assertTrue(Ok, Text)
        Self.assertIn(f"{Name.replace('_', ' ')} is outside its joint limits, held where it stands", Text)
        Frames(Controller, Bus, 3)
        Self.assertEqual(Controller.servos[Name].target_raw, Raw)
        Holding(Self, Controller, Bus)

    def Jitter(Self, Name, Raw):
        """The servo reads Raw for one control frame (jitter against its hold), then its own hold pulls it back."""
        Self.Bus.position[Name] = Raw % 4096
        Self.Controller.refresh()

    def TestAJointHeldInTheMiddleOfItsWindowJittersThere(Self):
        for Name in ("wrist_roll", "shoulder_pan"):
            Left, Right = WindowMiddle(Self.Controller, Name)
            for Held, Across in ((Left, Right), (Right, Left)):
                with Self.subTest(joint=Name, held=Held):
                    Self.Fresh()
                    Controller, Bus = Self.Controller, Self.Bus
                    Self.PoseAndHold(Name, Held)
                    Step = 1 if (Across - Held) % 4096 == 1 else -1
                    for Offset in (Step, 2 * Step, 3 * Step, -Step, 0):
                        Self.Jitter(Name, Held + Offset)
                        Holding(Self, Controller, Bus)
                    Self.assertEqual(Controller.servos[Name].target_raw, Held, "still held where it was posed")
                    Self.assertLess(abs(Controller._slider_track_error(Controller.servos[Name])), Bridge.SLIDER_GOAL_TOLERANCE_RAW)

    def TestAPushAcrossTheMiddleIsCorrectedTheShortWay(Self):
        # 28 steps across the middle is 28 steps off the hold point: the follower drives it back there, the short way,
        # not around the joint's whole range (the error is far under HOLD_REARM_MAX_ERROR_RAW).
        Controller, Bus = Self.Controller, Self.Bus
        Left, Right = WindowMiddle(Controller, "wrist_roll")
        Self.PoseAndHold("wrist_roll", Left)
        Pushed = (Left + 28 * (1 if (Right - Left) % 4096 == 1 else -1)) % 4096
        Self.assertGreater(abs(Controller._slider_track_pos(Controller.servos["wrist_roll"], Pushed)
                               - Controller._slider_track_pos(Controller.servos["wrist_roll"], Left)), 3000, "read a turn apart before")
        Self.Jitter("wrist_roll", Pushed)
        Holding(Self, Controller, Bus)
        Self.assertAlmostEqual(abs(Controller._slider_track_error(Controller.servos["wrist_roll"])), 28, delta=1)
        Visited = []
        for _ in range(40):
            Frames(Controller, Bus, 1, Step=4)
            Visited.append(Bus.position["wrist_roll"])
        Holding(Self, Controller, Bus)
        Self.assertLessEqual(Controller._circular_raw_distance(Bus.position["wrist_roll"], Left), Bridge.SLIDER_GOAL_TOLERANCE_RAW)
        Self.assertTrue(all(Controller._circular_raw_distance(Raw, Left) <= 30 for Raw in Visited), "never went the long way round")

    def TestARouteBesideAJointHeldInTheMiddleOfItsWindowFliesOn(Self):
        # The probe's route case: the wrist roll held there, a gripper slider move, the wrist jitters 2 steps mid-route.
        Controller, Bus = Self.Controller, Self.Bus
        Left, Right = WindowMiddle(Controller, "wrist_roll")
        Self.PoseAndHold("wrist_roll", Left)
        Self.assertTrue(GripperRoute(Controller)[0])
        Frames(Controller, Bus, 2)
        for Raw in (Right, (Right + (Right - Left)) % 4096, Left):
            Bus.position["wrist_roll"] = Raw
            Bus.step("gripper", steps_per_tick=10)
            Controller.refresh()
            Self.assertIsNone(Controller.MotionFault, Controller.status)
            Self.assertIsNone(Controller.ProtectionTrip)
            Self.assertIsNotNone(Controller.MotionRoute, Controller.status)
        Frames(Controller, Bus, 90)
        Self.assertIsNone(Controller.MotionRoute, Controller.status)
        Holding(Self, Controller, Bus)
        Self.assertLessEqual(Controller._circular_raw_distance(Bus.position["wrist_roll"], Left), 3, "the wrist held where it was")

    def TestARouteBackIntoRangeFromTheMiddleGoesAsPlanned(Self):
        # Read on its stage start's side, a joint leaving the middle of its window follows its route as it was
        # planned, through a jitter across the middle at the start, with no stall trip or route fault on the way.
        Controller, Bus = Self.Controller, Self.Bus
        Left, Right = WindowMiddle(Controller, "wrist_roll")
        Wrist = Controller.servos["wrist_roll"]
        Self.PoseAndHold("wrist_roll", Right)
        Target = Controller._fraction_to_raw(Wrist, 0.85)  # inside the arc, past the right end's side
        Ok, Text = Controller.move_servo("wrist_roll", Target)
        Self.assertTrue(Ok, Text)
        Self.Jitter("wrist_roll", Left)  # a step across the middle as it starts
        Self.assertIsNone(Controller.MotionFault, Controller.status)
        for _ in range(400):
            if Controller.MotionRoute is None:
                break
            Frames(Controller, Bus, 1)
            Self.assertIsNone(Controller.MotionFault, Controller.status)
        Self.assertIsNone(Controller.MotionRoute, Controller.status)
        Holding(Self, Controller, Bus)
        Self.assertLessEqual(Controller._circular_raw_distance(Bus.position["wrist_roll"], Target), Bridge.SLIDER_GOAL_TOLERANCE_RAW)


class GraspTests(unittest.TestCase):
    """2026-10-08 safety review: holding where it is includes the grip. A gripper that stopped on an object holds it
    GRIPPER_GRASP_SQUEEZE_RAW (8) steps more closed than where its jaws rest (SqueezeGrasp): that offset is the grip
    force. A stop re-held it at its present position too, squeeze 0, so a carried object could slip out while the arm
    held: Stop and hold, Hold pressed while moving, and (since the H1 fix) a scene that changed mid-carry."""

    def setUp(Self):
        SceneSafety.Invalidate()
        Self.addCleanup(SceneSafety.Invalidate)

    @staticmethod
    def CarryFrames(Controller, Bus, Count):
        """Control frames with the jaws resting on the object: every joint but the gripper steps."""
        for _ in range(Count):
            for Name in Bridge.MOTOR_NAMES:
                if Name != "gripper":
                    Bus.step(Name, steps_per_tick=10)
            Controller.refresh()

    @staticmethod
    def Squeeze(Controller, Bus):
        Gripper = Controller.servos["gripper"]
        Closed = Gripper.present_raw + (Bridge.GRIPPER_GRASP_SQUEEZE_RAW if Gripper.arc_inverted else -Bridge.GRIPPER_GRASP_SQUEEZE_RAW)
        return Bus.reg("gripper", "Goal_Position") == Controller._clamp_raw(Gripper, Closed) and Bus.reg("gripper", "Operating_Mode") == 0

    def Carrying(Self):
        """The jaws closed on an object (HoldGrasp's squeeze), then a scene-bound wrist move carries it."""
        Controller, Bus = MakeController()
        Gripper = Controller.servos["gripper"]
        with Controller.lock:
            Controller.SqueezeGrasp(Gripper, Gripper.present_raw)
        Self.CarryFrames(Controller, Bus, 2)
        Scene = SyntheticScene()
        Wrist = Controller.servos["wrist_flex"]
        Ok, Text = Controller.move_servo("wrist_flex", Controller._clamp_raw(Wrist, Wrist.present_raw + 200), Scene["id"], True, False)
        Self.assertTrue(Ok, Text)
        Self.CarryFrames(Controller, Bus, 3)
        Self.assertIsNotNone(Controller.MotionRoute)
        Self.assertTrue(Gripper.GraspHold and Self.Squeeze(Controller, Bus), "carrying keeps the squeeze (BeginRouteStage)")
        return Controller, Bus

    def TestAStopMidCarryKeepsTheGrip(Self):
        Stops = {"scene changed (HoldRoute)": lambda Controller: SceneSafety.Invalidate(),
                 "Stop and hold": lambda Controller: Controller.handle_command({"type": "stop_motion"}),
                 "Hold pressed while moving": lambda Controller: Controller.toggle_torque(True)}
        for Name, Stop in Stops.items():
            with Self.subTest(Name):
                Controller, Bus = Self.Carrying()
                Stop(Controller)
                Self.CarryFrames(Controller, Bus, 3)
                Holding(Self, Controller, Bus)
                Self.assertTrue(Controller.servos["gripper"].GraspHold, Controller.status)
                Self.assertTrue(Self.Squeeze(Controller, Bus), (Bus.reg("gripper", "Goal_Position"), Controller.servos["gripper"].present_raw))
                Self.assertTrue(Controller.state_snapshot()["servos"]["gripper"]["grasp_hold"])
                # The next carry move keeps squeezing.
                SceneSafety.Invalidate()
                Lift = Controller.servos["wrist_flex"]
                Self.assertTrue(Controller.move_servo("wrist_flex", Controller._clamp_raw(Lift, Lift.present_raw - 100))[0], Controller.status)
                Self.CarryFrames(Controller, Bus, 2)
                Self.assertTrue(Controller.servos["gripper"].GraspHold and Self.Squeeze(Controller, Bus))

    def TestCoastLetsGo(Self):
        Controller, Bus = Self.Carrying()
        Self.assertTrue(Controller.toggle_torque(False)[0])
        Self.assertFalse(Bus.torque_enabled)
        Self.assertFalse(Controller.servos["gripper"].GraspHold, "coasting holds no grip")
        Controller, Bus = Self.Carrying()
        Controller.emergency_stop()
        Self.assertFalse(Controller.servos["gripper"].GraspHold)


class SlowFakeBus(FakeBus):
    """configure_motors as on the arm: slow (EEPROM writes), and the Goal_Position seed energises (audit ReproConnect)."""
    Seconds = 0.0

    def connect(self):
        self.is_connected = True

    def configure_motors(self, acceleration: int = 0, goal_speed: int = 0) -> None:
        time.sleep(self.Seconds)
        super().configure_motors(acceleration, goal_speed)
        self.torque_enabled = True


class ConnectTests(unittest.TestCase):
    """H4: the heartbeat after a slow connect."""

    def TestSlowConnectToAHoldingArmDoesNotTripTheFirstFrame(Self):
        for Seconds in (0.5, 0.9):
            with Self.subTest(Seconds=Seconds):
                Scratch = Path(tempfile.mkdtemp(prefix="TestAuditConnect"))
                Self.addCleanup(shutil.rmtree, Scratch, True)
                shutil.copy(Root / "home_preset.json", Scratch / "home_preset.json")
                shutil.copy(Root / "gui_limits.json", Scratch / "gui_limits.json")
                Controller = Bridge.BridgeController(home_path=Scratch / "home_preset.json", limits_path=Scratch / "gui_limits.json")
                SlowFakeBus.Seconds = Seconds
                with patch.object(Bridge, "_make_bus", lambda Port: SlowFakeBus()):
                    Controller.refresh()  # the control loop's frame before connect takes the lock
                    Ok, Text = Controller.connect("FAKE")
                Self.assertTrue(Ok, Text)
                Self.assertTrue(Controller.torque_enabled, "the arm came out of connect holding")
                Controller.refresh()  # the first frame after connect
                Self.assertIsNone(Controller.ProtectionTrip)
                Self.assertTrue(Controller.torque_enabled and Controller.bus.torque_enabled)

    def TestMovingStateHeartbeatLimitIsKept(Self):
        Controller, Bus = MakeController()
        Self.assertTrue(GripperRoute(Controller)[0])
        Frames(Controller, Bus, 2)
        Controller.last_refresh_s -= Bridge.ControlHeartbeatLimitS + 0.2  # a late frame while a route flies
        Controller.refresh()
        Self.assertIn("heartbeat", (Controller.ProtectionTrip or {}).get("reason", ""))


class PreemptionTests(unittest.TestCase):
    """M2: Stop, Hold and Coast against a route still planning, and the commands queued behind it."""

    def SlowPlanner(Self, Seconds=4.0):
        """PlanMotionRoute standing in for a long plan: real pose checks (BasketCollision.IntervalReason) for
        Seconds, then the one-stage route. A cancelled plan ends at its next check (PlanningCancelled)."""
        Log = Self.Log = []

        def Plan(Snapshot, Targets, *Args, **Options):
            Log.append(("planning", time.monotonic()))
            World = Snapshot.CollisionWorld()
            Pose = Collision.Np.array([Snapshot._ik_angle_from_raw(S, S.present_raw) for S in Snapshot.servos.values()])
            End = time.monotonic() + Seconds
            while time.monotonic() < End:
                World.IntervalReason(Pose, Pose)
            Log.append(("planned", time.monotonic()))
            return Collision.RoutePath([{Name: Targets.get(Name, Servo.present_raw) for Name, Servo in Snapshot.servos.items()}])
        return patch.object(Bridge.BridgeController, "PlanMotionRoute", Plan)

    def Run(Self, Control):
        Controller, Bus = MakeController()
        Gripper = Controller.servos["gripper"]
        Target = Controller._clamp_raw(Gripper, Gripper.present_raw + 300)
        Replies = []
        with Self.SlowPlanner(), patch.object(Bridge, "controller", Controller), \
                LocalClient().websocket_connect("/ws") as Socket:
            Socket.receive_json()
            Socket.send_json({"type": "move_servo", "name": "gripper", "raw": Target})
            Socket.send_json({"type": "go_home"})  # queued behind the planning move
            time.sleep(0.4)
            Pressed = time.monotonic()
            Socket.send_json(Control)
            Deadline = time.monotonic() + 8
            while len(Replies) < 3 and time.monotonic() < Deadline:
                Message = Socket.receive_json()
                if Message.get("type") == "status":
                    Replies.append((round(time.monotonic() - Pressed, 2), Message["ok"], Message["text"]))
        return Controller, Bus, Replies, Pressed

    @staticmethod
    def Reply(Replies, Text):
        """The one reply whose text has Text in it (the stop's and the cancelled plan's come in either order)."""
        Found = [Reply for Reply in Replies if Text in Reply[2]]
        assert len(Found) == 1, (Text, Replies)
        return Found[0]

    def TestStopRunsAheadOfPlanningAndCancelsIt(Self):
        Controller, Bus, Replies, Pressed = Self.Run({"type": "stop_motion"})
        Self.assertEqual(len(Replies), 3, Replies)
        # The stop is answered at once; the planning move is cancelled at its next check, not started; Home never runs.
        Stop, Plan, Home = (Self.Reply(Replies, Text) for Text in ("Motion stopped", "planning", "go home not run"))
        Self.assertTrue(Stop[1], Replies)
        Self.assertLess(Stop[0], 0.5, Replies)
        Self.assertFalse(Plan[1])
        Self.assertLess(Plan[0], 1.0, "the plan ended at its next check, not after its 4 s")
        Self.assertFalse(Home[1])
        Self.assertIsNone(Controller.MotionRoute)
        Self.assertTrue(Bus.torque_enabled)

    def TestCoastRunsAheadOfPlanning(Self):
        Controller, Bus, Replies, _ = Self.Run({"type": "toggle_torque", "enable": False})
        Self.assertEqual(len(Replies), 3, Replies)
        Coast = Self.Reply(Replies, "Torque off")
        Self.assertTrue(Coast[1], Replies)
        Self.assertLess(Coast[0], 0.5, Replies)
        Self.assertFalse(Bus.torque_enabled, "coasting")
        Self.assertIsNone(Controller.MotionRoute)
        Self.assertTrue(all(not Ok for _, Ok, Text in Replies if Text != Coast[2]), Replies)

    def TestAStopJustBeforeTheMotionStartsStillCancelsIt(Self):
        # The window the queue's own check leaves: the move left the queue, then a Stop came in and moved the motion
        # epoch before the move took its planning snapshot, so the epoch alone would let its route start after the Stop.
        Controller, Bus = MakeController()
        Gripper = Controller.servos["gripper"]
        Move = {"type": "move_servo", "name": "gripper", "raw": Controller._clamp_raw(Gripper, Gripper.present_raw + 300)}
        with patch.object(Bridge, "controller", Controller):
            Generation = Bridge.PreemptGeneration()  # what the queue checked
            Bridge.BumpPreemptGeneration()
            Controller.StopMotion()  # the Stop, run ahead of the queue
            Ok, Text = Bridge.RunQueuedCommand(Move, Generation)
            Self.assertFalse(Ok, Text)
            Self.assertIn("pressed after it was sent", Text)
            Self.assertIsNone(Controller.MotionRoute)
            # A command sent after the Stop runs as usual.
            Ok, Text = Bridge.RunQueuedCommand(Move, Bridge.PreemptGeneration())
            Self.assertTrue(Ok, Text)
            # And outside a command (the bridge's own callers) nothing is stale.
            Self.assertFalse(Bridge.QueuedCommandStale())

    def TestAnHttpMotionThatArrivedBeforeAStopIsCancelledByIt(Self):
        # 2026-10-08 safety review: a POST /motion move that arrived before a Stop, held up before its first lock (the
        # self-calibration moves archive a state snapshot under the motor lock first), read the motion epoch after the
        # Stop and started its route after it. It now carries the preempt generation it arrived under, as /ws commands do.
        Controller, Bus = MakeController()
        Gripper = Controller.servos["gripper"]
        Target = Controller._clamp_raw(Gripper, Gripper.present_raw + 300)
        Arrived, Stopped, Result = threading.Event(), threading.Event(), {}
        Original = Controller._clamp_raw

        def HeldUp(Servo, Raw):  # move_servo's first step, before any lock: held until the Stop has run
            if threading.current_thread().name == "http" and not Arrived.is_set():
                Arrived.set()
                Stopped.wait(5)
            return Original(Servo, Raw)
        with patch.object(Bridge, "controller", Controller), patch.object(Controller, "_clamp_raw", HeldUp):
            Http = threading.Thread(name="http", target=lambda: Result.update(
                http=Bridge.MotionCommand({"type": "move_servo", "name": "gripper", "raw": Target}, None)))
            Http.start()
            Self.assertTrue(Arrived.wait(5))
            Stop = Bridge.MotionCommand({"type": "stop_motion"}, None)  # the Stop, after the move arrived
            Stopped.set()
            Http.join(30)
            Self.assertTrue(Stop["ok"], Stop)
            Self.assertFalse(Result["http"]["ok"], Result["http"])
            Self.assertIn("pressed after it was sent", Result["http"]["message"])
            Self.assertIsNone(Controller.MotionRoute)
            # A move sent after the Stop runs.
            Self.assertTrue(Bridge.MotionCommand({"type": "move_servo", "name": "gripper", "raw": Target}, None)["ok"])
            Self.assertIsNotNone(Controller.MotionRoute)

    def TestHoldPressedWhileHoldingMeansHold(Self):
        Controller, Bus = MakeController()
        Self.assertTrue(Controller.toggle_torque(True)[0])
        Self.assertTrue(Bus.torque_enabled, "Hold pressed while holding does not coast")
        Self.assertTrue(GripperRoute(Controller)[0])
        Frames(Controller, Bus, 3)
        Ok, Text = Controller.toggle_torque(True)
        Self.assertTrue(Ok, Text)
        Holding(Self, Controller, Bus)
        Self.assertTrue(Controller.toggle_torque(False)[0])
        Self.assertTrue(Controller.toggle_torque(False)[0], "Coast pressed while coasting")
        Self.assertFalse(Bus.torque_enabled)


class LockTests(unittest.TestCase):
    """M10: nothing heavy under the motor lock."""

    def setUp(Self):
        SceneSafety.Invalidate()
        Self.addCleanup(SceneSafety.Invalidate)

    def TestHoldCorrectionNeverChecksAMeasuredScene(Self):
        Controller, Bus = MakeController()
        Boxes = [{"min": [40.0 + I % 20, 60.0 + I // 20, 40.0], "max": [40.5 + I % 20, 60.5 + I // 20, 40.5], "label": f"box {I}"}
                 for I in range(1400)]
        Controller.ActiveScene = SyntheticScene(Boxes)  # as a scene-bound move would have left it before M1
        Frames_ = []
        Original = Collision.CollisionWorld

        def Recording(Geometry, Frame):
            Frames_.append(Frame)
            return Original(Geometry, Frame)
        Lift = Controller.servos["shoulder_lift"]
        Lift.Holding, Lift.target_raw = True, Lift.present_raw
        Bus.position["shoulder_lift"] = (Lift.present_raw + 20) % 4096
        Controller._read_positions_locked()
        Controller.HoldReturnCache = None
        def Check():
            Controller.HoldReturnCache = None
            Controller.HoldReturnClear()
        # The frames it checks are the proof; the time (2.1 ms here, 120 ms with the scene before M10) only backs it up:
        # the fastest of three runs against a bound scaled by how busy the PC is (LoadScale).
        with patch.object(Collision, "CollisionWorld", Recording):
            Scale = LoadScale.Factor()
            Took = LoadScale.Best(Check, 3)
        Self.assertTrue(Frames_ and all("SceneObstacles" not in Frame for Frame in Frames_))
        Self.assertLess(Took, 0.1 * Scale, f"{Took * 1000:.0f} ms under the lock (load x{Scale:.2f})")

    def TestSceneIsBoundOffTheMotorLock(Self):
        Controller, Bus = MakeController()
        Scene = SyntheticScene()
        Owners = []
        Original = SceneSafety.GetValid

        def Watch(*Args, **Options):
            Owners.append(Controller.lock.Owner)
            return Original(*Args, **Options)
        with patch.object(SceneSafety, "GetValid", Watch):
            Self.assertTrue(GripperRoute(Controller, Scene)[0])
        Self.assertTrue(Owners)
        Self.assertNotIn(threading.current_thread().name, Owners, "the deep copy ran while this thread held the motor lock")

    def ScratchFrame(Self):
        """WorkspaceFrame.json copied to a scratch folder, read by the bridge from there (its own JsonFileCache)."""
        Scratch = Path(tempfile.mkdtemp(prefix="TestAuditFrame"))
        Self.addCleanup(shutil.rmtree, Scratch, True)
        File = Scratch / "WorkspaceFrame.json"
        shutil.copy(Root / "WorkspaceFrame.json", File)
        Cache = Bridge.JsonFileCache(File)
        Patch = patch.object(Bridge, "WorkspaceFrameFile", Cache)
        Patch.start()
        Self.addCleanup(Patch.stop)
        return File, Cache

    @staticmethod
    def SlowDisk(Seconds):
        """Every stat, open and read Seconds slow (2026-10-19: one waited 1.7 s while the recorder wrote video)."""
        def Slow(Original):
            def Wrapped(*Args, **Options):
                time.sleep(Seconds)
                return Original(*Args, **Options)
            return Wrapped
        return [patch.object(Path, "stat", Slow(Path.stat)), patch.object(Path, "read_text", Slow(Path.read_text)),
                patch.object(Path, "read_bytes", Slow(Path.read_bytes)), patch("builtins.open", Slow(open))]

    def TestNoFileIsReadUnderTheMotorLock(Self):
        # 2026-10-19T00:06:44Z: a control frame waited 1719 ms for the motor lock, held by a WebSocket client's state
        # snapshot reading WorkspaceFrame.json (ConfigurationKey, WorkspaceOffset); a route read it every frame too.
        Controller, Bus = MakeController()
        File, Cache = Self.ScratchFrame()
        Controller.state_snapshot()
        Before = Controller.state_snapshot()["workspace_offset"]
        Frame = json.loads(File.read_text(encoding="utf-8"))
        Frame["RobotFrame"]["ArmBasePositionCm"]["x"] += 1.0
        File.write_text(json.dumps(Frame, indent=2) + "\n", encoding="utf-8")  # an edit the next snapshot must see
        Controller.lock.Holds.clear()
        Snapshots, Took, Stop = [], [], threading.Event()

        def Client():  # one page at 30 Hz, its snapshots on their own thread as the bridge's asyncio.to_thread
            while not Stop.is_set():
                Snapshots.append(Controller.state_snapshot())
                time.sleep(1 / 30)
        Slow = Self.SlowDisk(0.3)
        for Each in Slow:
            Each.start()
        try:
            Edited = Controller.state_snapshot()  # reads the edited file (three slow accesses), off the lock
            Controller.refresh()  # the control loop's frame meanwhile (the heartbeat counts from it)
            Thread = threading.Thread(target=Client, name="asyncio_0")
            Thread.start()
            Self.assertTrue(GripperRoute(Controller)[0])
            for _ in range(30):  # one second of control frames, a route running, against the client's snapshots
                for Name in Bridge.MOTOR_NAMES:
                    Bus.step(Name, steps_per_tick=2)
                Start = time.perf_counter()
                Controller.refresh()
                Took.append(time.perf_counter() - Start)
                time.sleep(1 / 30)
        finally:
            Stop.set()
            Thread.join()
            for Each in reversed(Slow):
                Each.stop()
        Self.assertGreater(len(Snapshots), 1)
        Self.assertAlmostEqual(Edited["workspace_offset"]["x"], Before["x"] - 1.0, msg="the edited frame was read")
        Self.assertEqual(Snapshots[-1]["configuration_key"], Edited["configuration_key"])
        Self.assertIsNotNone(Controller.MotionRoute, Controller.status)  # the route ran on: the edit came before it
        Self.assertIsNone(Controller.ProtectionTrip)
        Scale = LoadScale.Factor()
        # The motor lock's own record keeps every hold over ControlLockHoldNoteS (50 ms): there is none.
        Self.assertEqual([Hold for Hold in Controller.lock.Holds], [], "a hold of the motor lock over 50 ms")
        Self.assertLess(max(Took), 0.05 * Scale, f"a control frame took {max(Took) * 1000:.0f} ms (load x{Scale:.2f})")

    def TestFrameEditedMidRouteStillStopsTheRouteHolding(Self):
        # What CheckMotionRoute read from disk every frame, it now takes from the last read: the running server's
        # WatchFiles (and every state snapshot) reads the edit, and the route stops holding, as before.
        Controller, Bus = MakeController()
        File, Cache = Self.ScratchFrame()
        Self.assertTrue(GripperRoute(Controller)[0])
        Frames(Controller, Bus, 2, Step=2)
        Frame = json.loads(File.read_text(encoding="utf-8"))
        Frame["RobotFrame"]["ArmBasePositionCm"]["x"] += 1.0
        File.write_text(json.dumps(Frame, indent=2) + "\n", encoding="utf-8")
        Frames(Controller, Bus, 1, Step=2)
        Self.assertIsNotNone(Controller.MotionRoute, "a control frame reads no file")
        Cache.Refresh()  # as WatchFiles does every FileWatchS
        Frames(Controller, Bus, 1, Step=2)
        Holding(Self, Controller, Bus)
        Self.assertIn("configuration", Controller.status)

    def TestFrameCacheReadsAgainOnlyWhenTheFileChanged(Self):
        File, Cache = Self.ScratchFrame()
        Reads = []
        Original = Bridge._load_json
        with patch.object(Bridge, "_load_json", lambda Path_: Reads.append(Path_) or Original(Path_)):
            First = Cache.Refresh()
            Self.assertIs(Cache.Refresh(), First)
            Self.assertIs(Cache.Data(), First)
            Self.assertEqual(len(Reads), 1, "an unchanged file is not read again")
            File.write_text(File.read_text(encoding="utf-8") + " ", encoding="utf-8")  # one byte longer
            Self.assertIsNot(Cache.Refresh(), First)
            Self.assertEqual(len(Reads), 2)
            File.write_text("{ half written", encoding="utf-8")
            Self.assertEqual(Cache.Refresh(), {})
            Self.assertEqual(Cache.Refresh(), {})
            Self.assertEqual(len(Reads), 4, "a read that did not parse is tried again")
        Copy = Bridge.WorkspaceFrame()
        Copy["changed"] = True
        Self.assertNotIn("changed", Cache.Data(), "WorkspaceFrame() hands out a copy")


def WaitFor(Condition, Seconds=5.0):
    Deadline = time.monotonic() + Seconds
    while time.monotonic() < Deadline:
        if Condition():
            return True
        time.sleep(0.01)
    return False


class CameraSource:
    """A depth camera whose open (the SDK's 1-7 s with the interpreter lock held) lasts until Release is set."""

    def __init__(Self, Opens, Release):
        Opens.append(time.monotonic())
        Release.wait(5.0)
        Self.Info, Self.Intrinsics = {"name": "fake"}, {"fx": 1.0}

    def Read(Self, TimeoutMs=500):
        time.sleep(0.01)
        return np.zeros((4, 4, 3), np.uint8), np.zeros((4, 4), np.uint16)

    def Close(Self):
        pass


class CameraInterlockTests(unittest.TestCase):
    """2026-10-25: a depth-camera open and a motion never overlap (ad5a7a8 only skipped opens while the arm moved, so an
    open begun just before a route froze the control loop under it, velocity-mode joints unsupervised)."""

    def setUp(Self):
        # SO101_DEPTH_CAMERA=0 makes Start a no-op; the real SDK cannot even be imported here (as TestDepthCamera).
        for Item in (patch.dict(os.environ, {"SO101_DEPTH_CAMERA": "1"}), patch.dict(sys.modules, {"pyorbbecsdk": None}),
                     patch.object(OrbbecCamera, "QuietPollS", 0.02)):
            Item.start()
            Self.addCleanup(Item.stop)
        Self.Controller, Self.Bus = MakeController()
        Self.Opens, Self.Release = [], threading.Event()
        Self.Camera = OrbbecCamera.DepthCamera(Opener=lambda: CameraSource(Self.Opens, Self.Release))
        # As the running server wires them (bridge_server: depth_camera.Quiet, controller.Camera).
        Self.Camera.Quiet = Self.Controller.CameraMustWait
        Self.Controller.Camera = Self.Camera
        Self.addCleanup(Self.Camera.Stop)
        Self.addCleanup(Self.Release.set)  # runs first: an open still waiting ends

    def TestNoMotionStartsWhileTheCameraOpens(Self):
        Controller = Self.Controller
        Self.Camera.Start()
        Self.assertTrue(WaitFor(lambda: Self.Camera.Opening))
        with patch.object(Bridge, "CameraOpenWaitS", 0.2):
            Ok, Text = GripperRoute(Controller)
        Self.assertFalse(Ok)
        Self.assertIn("depth camera opens", Text)
        Self.assertIsNone(Controller.MotionRoute)
        Self.assertIsNone(Controller.MotionFault, "refused, not latched")
        Self.assertFalse([W for W in Self.Bus.writes if W[0] in ("Goal_Speed", "Operating_Mode")], "nothing was written")
        Ok, Text = Controller.HandGuideBegin(None, {}, "unused")
        Self.assertFalse(Ok)
        Self.assertIn("Hand guiding not started: Cannot move while the depth camera opens", Text)
        Self.assertIsNone(Controller.HandGuide)

    def TestAMotionAskedForDuringAnOpenStartsAfterIt(Self):
        Self.Camera.Start()
        Self.assertTrue(WaitFor(lambda: Self.Camera.Opening))
        Ended = []

        def Finish():
            time.sleep(0.3)
            Ended.append(time.monotonic())
            Self.Release.set()
        threading.Thread(target=Finish).start()
        Ok, Text = GripperRoute(Self.Controller)  # waits, off the motor lock, for the open (CameraOpenWaitS)
        Self.assertTrue(Ok, Text)
        Self.assertGreaterEqual(Self.Controller.MotionStartedAt, Ended[0], "the route started after the open ended")
        Self.assertEqual(len(Self.Opens), 1)

    def TestNoOpenWhileARouteRuns(Self):
        Controller, Bus = Self.Controller, Self.Bus
        Self.assertTrue(GripperRoute(Controller)[0])
        Self.Release.set()  # an open would end at once
        Self.Camera.Start()
        for _ in range(5):
            Frames(Controller, Bus, 1, Step=2)
            time.sleep(0.03)
        Self.assertIsNotNone(Controller.MotionRoute)
        Self.assertEqual(Self.Opens, [], "no open while the route runs")
        Frames(Controller, Bus, 120)
        Self.assertIsNone(Controller.MotionRoute, Controller.status)
        Self.assertTrue(WaitFor(lambda: Self.Opens), "it opens once the route ended")

    def TestNoOpenWhileARouteIsChecked(Self):
        # A route about to start: from the command, through planning (seconds on the arm), to its start.
        Self.Release.set()
        Original = Bridge.BridgeController.PlanMotionRoute
        Planning = threading.Event()

        def Slow(Snapshot, *Args, **Options):
            Planning.set()
            time.sleep(0.4)
            return Original(Snapshot, *Args, **Options)
        Result = []
        with patch.object(Bridge.BridgeController, "PlanMotionRoute", Slow):
            Thread = threading.Thread(target=lambda: Result.append(GripperRoute(Self.Controller)))
            Thread.start()
            Self.assertTrue(Planning.wait(5.0))
            Self.Camera.Start()  # its first check comes while the route is planned
            Thread.join()
        Self.assertTrue(Result and Result[0][0], Result)
        time.sleep(0.1)
        Self.assertIsNotNone(Self.Controller.MotionRoute)
        Self.assertEqual(Self.Opens, [], "no open while the route was planned, nor since it started")

    def TestAnOpenCheckingAsTheRouteStartsWaitsForIt(Self):
        # The gap ad5a7a8 left: the open's check passes, then a route starts, then the open runs under it. The camera's
        # check now waits for the bridge's MotionStartGate, held from the route's last check to its start; here alone,
        # without the planning registration (MotionsStarting) that keeps the camera quiet before it.
        Self.Release.set()
        Original = Bridge.BridgeController.StartMotionRoute
        Checked, Threads = [], []

        def Start(Controller_, *Args, **Options):
            Controller_.MotionsStarting.clear()
            Thread = threading.Thread(target=lambda: Checked.append(Self.Camera._BeginOpen()))
            Thread.start()
            Thread.join(0.2)
            Checked.append("waiting" if Thread.is_alive() else "passed")
            Threads.append(Thread)
            return Original(Controller_, *Args, **Options)
        with patch.object(Bridge.BridgeController, "StartMotionRoute", Start):
            Ok, Text = GripperRoute(Self.Controller)
        Self.assertTrue(Ok, Text)
        Threads[0].join(2.0)
        Self.assertEqual(Checked, ["waiting", False], "the open waited for the start, then saw the route and did not open")
        Self.assertFalse(Self.Camera.Opening)

    def TestAFailedOpenIsNotOpeningWhileItBacksOff(Self):
        def Missing():
            Self.Opens.append(time.monotonic())
            raise RuntimeError("no Orbbec camera connected")
        Self.Camera.Opener = Missing
        Self.Camera.Start()
        Self.assertTrue(WaitFor(lambda: Self.Opens and Self.Camera.Error))
        Self.assertFalse(Self.Camera.Opening, "a route may start during the backoff")
        Self.assertTrue(GripperRoute(Self.Controller)[0])


class StartupWarmUpTests(unittest.TestCase):
    """2026-10-25 15:07:39, 19:01:13, 19:31:52: a 6.1-6.8 s stall about 6 s after each bridge start, while RouteCheckPool
    spawned its workers and the camera made its first open. No motion starts until both have ended."""

    def setUp(Self):
        for Item in (patch.dict(os.environ, {"SO101_DEPTH_CAMERA": "1"}), patch.dict(sys.modules, {"pyorbbecsdk": None})):
            Item.start()
            Self.addCleanup(Item.stop)

    def TestNoMotionUntilTheWorkersAndTheCameraStarted(Self):
        Controller, Bus = MakeController()
        Pool, Camera = SimpleNamespace(Started=threading.Event()), OrbbecCamera.DepthCamera(Opener=lambda: None)
        Controller.WarmUp = lambda: Bridge.StartupPending(Pool, Camera)  # as the server's lifespan sets it
        Refusals = {"a joint move": lambda: GripperRoute(Controller), "Home": Controller.go_home,
                    "a Cartesian move": lambda: Controller.send_ik(Target=dict(Controller.target_cm)),
                    "hand guiding": lambda: Controller.HandGuideBegin(None, {}, "unused")}
        for Name, Move in Refusals.items():
            with Self.subTest(Name):
                Ok, Text = Move()
                Self.assertFalse(Ok)
                Self.assertIn("the bridge is still starting (route check workers and the depth camera)", Text)
        Self.assertIsNone(Controller.MotionRoute)
        Self.assertIsNone(Controller.HandGuide)
        Self.assertIsNone(Controller.MotionFault, "refused, nothing latched")
        Self.assertFalse([W for W in Bus.writes if W[0] in ("Goal_Speed", "Operating_Mode")], "nothing was written")
        Pool.Started.set()
        Ok, Text = GripperRoute(Controller)
        Self.assertFalse(Ok)
        Self.assertIn("still starting (the depth camera)", Text)
        Camera.FirstOpenDone.set()
        Ok, Text = GripperRoute(Controller)
        Self.assertTrue(Ok, Text)
        Self.assertIsNotNone(Controller.MotionRoute)

    def TestTheCameraFirstOpenEndsTheWarmUpHoweverItEnds(Self):
        Gate = threading.Event()

        def Missing():
            Gate.wait(5.0)
            raise RuntimeError("no Orbbec camera connected")
        Camera = OrbbecCamera.DepthCamera(Opener=Missing)
        Camera.Start()
        Self.addCleanup(Camera.Stop)
        time.sleep(0.1)
        Self.assertFalse(Camera.FirstOpenDone.is_set(), "still opening")
        Gate.set()
        Self.assertTrue(Camera.FirstOpenDone.wait(2.0), "a failed first open ends it too")
        with patch.dict(os.environ, {"SO101_DEPTH_CAMERA": "0"}):
            Off = OrbbecCamera.DepthCamera(Opener=Missing)
            Off.Start()
            Self.assertTrue(Off.FirstOpenDone.is_set(), "a camera that is off never holds motion back")
        Self.assertIsNone(Bridge.StartupPending(None, Off), "no check workers started (a sandbox): none to wait for")

    def TestTheServerStartsWarmingUp(Self):
        Controller, Bus = MakeController()
        Pool = SimpleNamespace(Started=threading.Event())
        Camera = SimpleNamespace(Start=lambda: None, Stop=lambda: None, FirstOpenDone=threading.Event(), Opening=False)
        Seen = []

        class Loop:
            def start(Self_): pass
            def stop(Self_): pass

        async def Run():
            with patch.object(Bridge, "controller", Controller), patch.object(Bridge, "control_loop", Loop()), \
                    patch.object(Bridge, "depth_camera", Camera), patch.object(Bridge, "AppendBridgeEvent", lambda Record: None), \
                    patch.object(Bridge, "RaiseProcessPriority", lambda: "normal"), \
                    patch.object(Bridge.RouteChecks, "Start", lambda: Pool), patch.object(Bridge, "WatchFiles", lambda: None):
                async with Bridge._lifespan(Bridge.app):
                    Seen.append(Controller.MotionStartRefusal())
                    Pool.Started.set()
                    Camera.FirstOpenDone.set()
                    Seen.append(Controller.MotionStartRefusal())
        asyncio.run(Run())
        Self.assertIn("still starting (route check workers and the depth camera)", Seen[0])
        Self.assertIsNone(Seen[1])


class InputTests(unittest.TestCase):
    """M7, L4, S4, L3 and the version handshake."""

    def TestCurrentAngleOverrideIsDisplayOnly(Self):
        Controller, Bus = MakeController()
        Frames(Controller, Bus, 5)
        Lift = Controller.servos["shoulder_lift"]
        Before = (Controller.state_snapshot()["joint_positions"], Controller._current_ik_angles())
        Self.assertTrue(Controller.set_gui_angle({"name": "shoulder_lift", "raw": (Lift.present_raw + 200) % 4096})[0])
        After = Controller.state_snapshot()
        Self.assertEqual(After["servos"]["shoulder_lift"]["gui_angle_raw"], Lift.gui_angle_raw)
        Self.assertEqual((After["joint_positions"], Controller._current_ik_angles()), Before)

    def TestEmergencyStopReadsItsTorqueOffBack(Self):
        Controller, Bus = MakeController()
        Read = Bus.read

        def StuckOn(Reg, Name, **Options):
            return 1 if Reg == "Torque_Enable" and Name == "elbow_flex" else Read(Reg, Name, **Options)
        Bus.read = StuckOn
        Ok, Text = Controller.emergency_stop()
        Self.assertFalse(Ok)
        Self.assertIn("elbow_flex", Text)
        Self.assertFalse(Controller.ProtectionTrip["coast_confirmed"], "kept pending for the next connect")

    def TestControlLoopSurvivesAFailingCoast(Self):
        class Broken:
            torque_enabled = True
            Frames = 0

            def refresh(Self_):
                Self_.Frames += 1
                raise RuntimeError("frame failed")

            def AutoCoast(Self_, Reason):
                raise RuntimeError("coast failed too")
        Target = Broken()
        Loop = Bridge.ControlLoop(Target, 1 / 30)
        Loop.start()
        time.sleep(0.4)
        Alive = Loop._thread.is_alive()
        Loop.stop()
        Self.assertTrue(Alive)
        Self.assertGreater(Target.Frames, 3)

    def TestLateFrameRecordsWhatItWaitedFor(Self):
        Controller, Bus = MakeController()
        Events = []
        Controller.EventSink = Events.append
        Controller.refresh()
        Release = threading.Event()

        def HoldTheLock():
            with Controller.lock:
                Release.wait(2)
        Holder = threading.Thread(target=HoldTheLock, name="slow-command")
        Holder.start()
        time.sleep(0.05)
        Timer = threading.Timer(0.45, Release.set)
        Timer.start()
        Controller.refresh()  # waits for the lock
        Holder.join()
        Stall = [Event for Event in Events if Event["kind"] == "control_stall"]
        Self.assertTrue(Stall, Events)
        Self.assertIn("slow-command", Stall[-1]["details"]["cause"])
        Self.assertGreaterEqual(Stall[-1]["details"]["lock_wait_ms"], 300)
        # A slow step of the frame itself names the step.
        Events.clear()
        Controller.refresh()
        Original = Controller._read_positions_locked

        def SlowRead():
            time.sleep(0.35)
            return Original()
        Controller._read_positions_locked = SlowRead
        Controller.refresh()
        Stall = [Event for Event in Events if Event["kind"] == "control_stall"]
        Self.assertTrue(Stall, Events)
        Self.assertEqual(Stall[-1]["details"]["cause"], "this frame's own steps")
        Self.assertGreaterEqual(Stall[-1]["details"]["phases_ms"]["read_positions"], 300)
        Self.assertEqual(Controller.state_snapshot()["protection"]["control_stalls"][-1]["cause"], "this frame's own steps")

    def TestStalledFrameDumpsEveryStackWithItsTime(Self):
        # The running server's diagnostic (StallDumpFile) with the real faulthandler: a frame that runs 0.8 s gets
        # every thread's stack 0.5 s in, and the next frame writes when that was under it. A scratch file only.
        Scratch = Path(tempfile.mkdtemp(prefix="TestAuditStall"))
        Self.addCleanup(shutil.rmtree, Scratch, True)
        Dumps = open(Scratch / "control_stalls.log", "a", encoding="utf-8")
        Self.addCleanup(Dumps.close)

        class Target:
            torque_enabled = False
            Frames = 0

            def refresh(Self_):
                Self_.Frames += 1
                if Self_.Frames == 3:
                    time.sleep(0.8)
        Loop = Bridge.ControlLoop(Target(), 1 / 30)
        Loop.StallDumps = lambda: Dumps
        Loop.start()
        try:
            Deadline = time.monotonic() + 5
            while Loop.target.Frames < 6 and time.monotonic() < Deadline:
                time.sleep(0.05)
        finally:
            Loop.stop()
        Text = (Scratch / "control_stalls.log").read_text(encoding="utf-8")
        # faulthandler's own format: the timeout, then each thread's stack (the control thread inside the slow frame).
        Self.assertIn("Timeout (0:00:00.500000)!", Text)
        Self.assertIn("in refresh", Text)
        Self.assertIn("in _run", Text)
        Self.assertRegex(Text, r"Timeout \(0:00:00\.500000\)!(.|\n)*# \d{4}-\d\d-\d\dT\d\d:\d\d:\d\d: the stack dump above; "
                               r"no control frame finished for \d{3,5} ms")

    def TestBridgeReportsTheVersionThePageRequires(Self):
        Version = json.loads((Root / "BridgeVersion.json").read_text(encoding="utf-8"))["Version"]
        Controller, _ = MakeController()
        Self.assertEqual(Bridge.health()["bridge_version"], Version)
        Self.assertEqual(Controller.state_snapshot()["bridge_version"], Version)
        Self.assertEqual(Bridge.health()["safety_version"], "hold-v4")
        Script = (Root.parent / "AI Controlled WebApp" / "scripts" / "RestartBridge.ps1").read_text(encoding="utf-8")
        Self.assertIn("BridgeVersion.json", Script)
        Self.assertIn('"hold-v4"', Script)


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestAuditSafety` runs the Test* methods too (unittest names this hook)."""
    Prefixed = unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return unittest.TestSuite(Prefixed.loadTestsFromTestCase(Case) for Case in
                              (SceneRouteTests, ConfigurationTests, WindowMiddleTests, GraspTests, ConnectTests, PreemptionTests, LockTests,
                                CameraInterlockTests, StartupWarmUpTests, InputTests))


if __name__ == "__main__":
    Loader = unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Suite = unittest.TestSuite(Loader.loadTestsFromTestCase(Case) for Case in
                               (SceneRouteTests, ConfigurationTests, WindowMiddleTests, GraspTests, ConnectTests, PreemptionTests, LockTests,
                                CameraInterlockTests, StartupWarmUpTests, InputTests))
    Result = unittest.TextTestRunner(verbosity=2).run(Suite)
    raise SystemExit(not Result.wasSuccessful())
