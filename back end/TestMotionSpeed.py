"""Offline throttle regressions. Uses only FakeBus, never the serial arm."""
import unittest as Unittest
from unittest.mock import patch as Patch

import bridge_server as Bridge
import feetech_simple as Driver
from test_bridge_home import make_controller as MakeController, displace as Displace, assert_at_home as AssertAtHome
from TestBasketCollision import LowController


class MotionSpeedTests(Unittest.TestCase):
    def TestValidationAndCapabilities(Self):
        Controller = Bridge.BridgeController()
        Self.assertEqual(Controller.state_snapshot()["motion_speed_percent"], 18)
        Self.assertEqual(Controller.state_snapshot()["speed_control_version"], "throttle-v2")
        Self.assertEqual(Bridge.health()["speed_control_version"], "throttle-v2")
        for Percent in (None, True, False, "50", {}, [], 0, 9, 101, float("nan"), float("inf"), -(10 ** 400), 10 ** 400):
            with Self.subTest(Percent=Percent):
                Self.assertFalse(Controller.handle_command({"type": "set_motion_speed", "percent": Percent})[0])
                Self.assertEqual(Controller.MotionSpeedPercent, 18)
        Self.assertTrue(Controller.SetMotionSpeed(37.4)[0])
        Self.assertEqual(Controller.state_snapshot()["motion_speed_percent"], 37)

    def TestFullSpeedRunsEachJointAtItsCeilingWithoutDriverClipping(Self):
        Controller, Bus = MakeController()
        Displace(Controller, Bus)
        Controller.SetMotionSpeed(100)
        Self.assertTrue(Controller.go_home()[0])
        Moving = [Servo for Servo in Controller.servos.values() if Servo.command_raw is not None]
        Ceilings = Controller.MotionSpeedCeilings(100)
        for Servo in Moving:
            Self.assertLessEqual(abs(Servo.command_speed), Ceilings[Servo.name], Servo.name)
        # This Home route is led by the shoulder lift, which runs at its own ceiling, not the top one.
        Self.assertEqual(abs(Controller.servos["shoulder_lift"].command_speed), 1050)
        DriverBus = Driver.FeetechMotorsBus.__new__(Driver.FeetechMotorsBus)
        for Servo in Moving:
            Self.assertEqual(DriverBus._clamp_write("Goal_Speed", Servo.command_speed, Servo.name), Servo.command_speed)
        for Name in Bridge.MOTOR_NAMES:
            # The driver's hard cap is the bridge's 100% ceiling, joint by joint: nothing in between is clipped.
            Self.assertEqual(DriverBus._goal_speed_limit_for(Name), Ceilings[Name], Name)
            for Speed in (-Ceilings[Name], Ceilings[Name]):
                Self.assertEqual(DriverBus._clamp_write("Goal_Speed", Speed, Name), Speed)
                Self.assertEqual(DriverBus._decode("Goal_Speed", DriverBus._encode("Goal_Speed", Speed)), Speed)
            for Speed in (-4000, 4000):
                Self.assertEqual(abs(DriverBus._clamp_write("Goal_Speed", Speed, Name)), Ceilings[Name])
        Controller.SetMotionSpeed(18)
        Controller.refresh()
        Self.assertEqual(max(abs(Servo.command_speed) for Servo in Moving), 315)
        Self.assertEqual(Driver.NORMAL_GOAL_SPEED, 200)
        Self.assertEqual(Driver.NORMAL_ACCELERATION, 51)
        Self.assertEqual(Driver.NORMAL_TORQUE_LIMIT, 1000)
        Self.assertEqual(Driver.HARD_MAX_VELOCITY_LIMIT, 51)
        Self.assertEqual(DriverBus._profile_goal_speed_for("shoulder_lift", Driver.NORMAL_GOAL_SPEED), 400)
        Self.assertEqual(Controller.state_snapshot()["motion_speed_maximum_multiplier"], 25)
        Self.assertEqual(Bridge.health()["motion_speed_maximum_multiplier"], 25)

    def TestFullSpeedCeilingsAndUnchangedStartupSpeed(Self):
        # 100%: a 1750 steps/s top ceiling, lower for the joints that lift the arm and the gripper.
        Self.assertEqual(Bridge.HOME_VELOCITY_MAX_SPEED * Bridge.MotionSpeedMaximumMultiplier, 1750)
        Self.assertEqual(Driver.HARD_GOAL_SPEED, 1750)
        Self.assertEqual(Bridge.BridgeController.MotionSpeedCeilings(100),
                         {"shoulder_pan": 1750, "shoulder_lift": 1050, "elbow_flex": 1250, "wrist_flex": 1650,
                          "wrist_roll": 1750, "gripper": 1050})
        # Startup is physically what it was: the old 30% of the old 1050 ceiling, for every joint.
        Self.assertEqual(Bridge.DefaultMotionSpeedPercent, 18)
        Self.assertEqual(30 * 1050 / 100, 315)
        Self.assertEqual(set(Bridge.BridgeController.MotionSpeedCeilings(Bridge.DefaultMotionSpeedPercent).values()), {315})
        # Below a joint's own ceiling the throttle stays linear in the top ceiling.
        Self.assertEqual(Bridge.BridgeController.MotionSpeedCeilings(50)["elbow_flex"], 875)
        Self.assertEqual(Bridge.BridgeController.MotionSpeedCeilings(50)["shoulder_lift"], 875)
        Self.assertEqual(Bridge.BridgeController.MotionSpeedCeilings(80)["shoulder_lift"], 1050)
        # Each joint leading its own long move cruises at exactly its ceiling (braking set aside).
        for Name, Ceiling in Bridge.BridgeController.MotionSpeedCeilings(100).items():
            with Self.subTest(Name=Name):
                Controller, Bus = MakeController()
                Controller.SetMotionSpeed(100)
                Servo = Controller.servos[Name]
                Fraction = Controller._raw_to_track_fraction(Servo, Servo.present_raw)
                Goal = Controller._fraction_to_raw(Servo, 0.05 if Fraction > 0.5 else 0.95)
                # Far enough to cruise outside the final slow zone.
                Self.assertGreater(abs(Controller._slider_track_pos(Servo, Goal) - Controller._slider_track_pos(Servo, Servo.present_raw)),
                                   Bridge.ROUTE_SLOW_ZONE_RAW)
                Self.assertTrue(Controller.move_servo(Name, Goal)[0])
                with Controller.lock, Patch.object(Bridge.BridgeController, "RouteBrakingSpeed", staticmethod(lambda Room, Overshoot=None: 1e9)):
                    Self.assertEqual(abs(Controller.RouteSpeeds()[Name]), Ceiling)

    def TestFinalApproachRetainsEnoughBrakingDistanceAtHigherMaximum(Self):
        Controller, Bus = MakeController()
        Displace(Controller, Bus)
        Controller.SetMotionSpeed(100)
        Self.assertTrue(Controller.go_home()[0])
        Self.assertTrue(Controller.RouteStageSettles())
        Servo = Controller.servos["elbow_flex"]
        Room = Bridge.Collision.Model["JointTrackingAllowanceRaw"] - Bridge.ROUTE_BRAKE_GUARD_RAW
        for Remaining in (13, 32, 90, 180, 360, 720, 1000, 2000):
            for Sign in (-1, 1):
                # Isolate one settling joint; real route/independent-motor
                # integration is covered by the basket and fly-by suites.
                with Patch.object(Controller, "_slider_track_pos", side_effect=lambda S, Raw:
                                  Sign * Remaining if S.name == Servo.name and Raw == Controller.MotionRoute["Path"][0][S.name] else 0), \
                        Patch.object(Controller, "ActiveMotionSpeedPercent", return_value=100):
                    Speed = abs(Controller.RouteSpeeds()[Servo.name])
                Effective = Speed * Bridge.ROUTE_BRAKE_OVERSHOOT
                Stopping = Effective * Bridge.ROUTE_BRAKE_LATENCY_S + Effective ** 2 / (2 * Bridge.ROUTE_BRAKE_ACCELERATION)
                Self.assertLessEqual(Stopping, Remaining + Room)
                Self.assertLessEqual(Speed, Controller.MotionSpeedCeilings(100)[Servo.name])

    def TestSettingSpeedNeverStartsMotionOrClearsFault(Self):
        for State in ("holding", "coasting", "fault", "calibration", "disconnected"):
            with Self.subTest(State=State):
                Controller, Bus = MakeController()
                if State == "coasting": Controller.toggle_torque()
                if State == "fault": Controller.StopMotion("Test fault", Latch=True)
                if State == "calibration": Controller.CalibrationActive = True
                if State == "disconnected": Controller.disconnect()
                Epoch, Fault, Torque = Controller.MotionEpoch, Controller.MotionFault, Bus.torque_enabled
                Targets = {Name: Servo.target_raw for Name, Servo in Controller.servos.items()}
                Bus.writes.clear()
                Self.assertTrue(Controller.SetMotionSpeed(10)[0])
                Self.assertEqual(Bus.writes, [])
                Self.assertEqual(Controller.MotionEpoch, Epoch)
                Self.assertEqual(Controller.MotionFault, Fault)
                Self.assertEqual(Bus.torque_enabled, Torque)
                Self.assertIsNone(Controller.MotionRoute)
                Self.assertEqual({Name: Servo.target_raw for Name, Servo in Controller.servos.items()}, Targets)

    def TestSlowerHomeReachesSameGoalsAndKeepsHold(Self):
        from TestFlyByRoutes import ServoModel
        Frames = []
        for Percent in (100, 25, 10):
            Controller, Bus = MakeController()
            Displace(Controller, Bus)
            Controller.SetMotionSpeed(Percent)
            Self.assertTrue(Controller.go_home()[0])
            Model = ServoModel(Bus)
            for Frame in range(2000):
                Model.Step()
                Controller.last_refresh_s = Bridge.time.monotonic()
                Controller.refresh()
                if Controller.MotionRoute is None: break
            Self.assertIsNone(Controller.MotionFault)
            Self.assertIsNone(Controller.MotionRoute)
            AssertAtHome(Controller, Bus)
            Self.assertTrue(Bus.torque_enabled)
            for Name in Bridge.MOTOR_NAMES:
                Self.assertGreater(Bus.reg(Name, "Goal_Speed"), 0, "Position-mode zero means unlimited speed")
                Self.assertEqual(Bus.reg(Name, "Operating_Mode"), Bridge.OperatingMode.POSITION.value)
            Frames.append(Frame)
        Self.assertLess(Frames[0], Frames[1])
        Self.assertLess(Frames[1], Frames[2])

    def TestLiveThrottleChangesSpeedWithoutRetargetingOrModeSwitch(Self):
        Controller, Bus = MakeController()
        Displace(Controller, Bus)
        Self.assertTrue(Controller.go_home()[0])
        Route, Epoch = Controller.MotionRoute, Controller.MotionEpoch
        Before = {Name: Bus.reg(Name, "Goal_Speed") for Name in Bridge.MOTOR_NAMES}
        Bus.writes.clear()
        Controller.SetMotionSpeed(10)
        Self.assertEqual(Bus.writes, [], "The guarded control loop applies the setting")
        Controller.refresh()
        Self.assertIs(Controller.MotionRoute, Route)
        Self.assertEqual(Controller.MotionEpoch, Epoch)
        Self.assertFalse(any(Register in ("Operating_Mode", "Torque_Enable", "Goal_Position") for Register, _, _ in Bus.writes))
        for Name, Speed in Before.items():
            if Controller.servos[Name].command_raw is not None:
                After = Bus.reg(Name, "Goal_Speed")
                Self.assertLess(abs(After), abs(Speed))
                Self.assertGreater(After * Speed, 0)
        Bus.writes.clear()
        Controller.SetMotionSpeed(99)
        Controller.refresh()
        Bus.writes.clear()
        Controller.SetMotionSpeed(100)
        Controller.refresh()
        Self.assertTrue(any(Register == "Goal_Speed" for Register, _, _ in Bus.writes), "Small throttle changes bypass the normal write deadband")
        Self.assertTrue(Bus.torque_enabled)

    def TestThirtyPercentCanSpeedUpAndSlowDownDuringSameMotion(Self):
        Controller, Bus = MakeController()
        Displace(Controller, Bus)
        Self.assertTrue(Controller.go_home()[0])
        Route, Epoch = Controller.MotionRoute, Controller.MotionEpoch
        Targets = {Name: Servo.target_raw for Name, Servo in Controller.servos.items()}
        Speeds = []
        InitialPosition = dict(Bus.position)
        for Percent in (30, 100, 30):
            for Name in Bridge.MOTOR_NAMES: Bus.step(Name)
            Controller.refresh()
            Bus.writes.clear()
            Self.assertTrue(Controller.handle_command({"type": "set_motion_speed", "percent": Percent})[0])
            Controller.refresh()
            Self.assertIs(Controller.MotionRoute, Route)
            Self.assertEqual(Controller.MotionEpoch, Epoch)
            Self.assertEqual({Name: Servo.target_raw for Name, Servo in Controller.servos.items()}, Targets)
            Self.assertFalse(any(Register in ("Operating_Mode", "Goal_Position", "Torque_Enable") for Register, _, _ in Bus.writes))
            Speeds.append(max(abs(Bus.reg(Name, "Goal_Speed")) for Name, Servo in Controller.servos.items() if Servo.command_raw is not None))
            Self.assertEqual(Controller.state_snapshot()["motion_speed_percent"], Percent)
            Self.assertIsNone(Controller.MotionFault)
            Self.assertTrue(Bus.torque_enabled)
        Self.assertNotEqual(Bus.position, InitialPosition, "The test changes speed after movement has begun")
        Self.assertLess(Speeds[0], Speeds[1])
        Self.assertGreater(Speeds[1], Speeds[2])

    def TestManualMotionStillUsesPhysicalRouteAtLowSpeed(Self):
        Controller, Bus, _ = LowController()
        Controller.SetMotionSpeed(10)
        Goal = Controller.servos["shoulder_pan"].present_raw - 34
        Ok, Message = Controller.move_servo("shoulder_pan", Goal)
        Self.assertTrue(Ok, Message)
        Self.assertIsNotNone(Controller.MotionRoute)
        # 10% of the 1750 top ceiling, ramped down over this 34-step move.
        Self.assertLessEqual(abs(Bus.reg("shoulder_pan", "Goal_Speed")), 34)
        for Frame in range(50):
            for Name in Bridge.MOTOR_NAMES: Bus.step(Name)
            Controller.refresh()
        Self.assertIsNone(Controller.MotionFault)
        Self.assertIsNone(Controller.MotionRoute)
        Self.assertTrue(Controller._slider_target_reached(Controller.servos["shoulder_pan"]))

    def TestCalibrationStagesRespectThrottle(Self):
        Controller, Bus = MakeController()
        Displace(Controller, Bus)
        Controller.refresh()
        Controller.SetMotionSpeed(10)
        Path = Controller.PlanMotionRoute(Controller.home_preset)
        with Controller.lock: Controller.StartMotionRoute(Path, Calibration=True)
        Self.assertTrue(Controller.MotionRoute["Calibration"])
        Self.assertGreaterEqual(Controller.MotionTimeoutMs, 120000)
        Self.assertTrue(any(Servo.command_raw is not None for Servo in Controller.servos.values()))
        for Name, Servo in Controller.servos.items():
            if Servo.command_raw is not None: Self.assertLessEqual(abs(Bus.reg(Name, "Goal_Speed")), 175)  # 10% of 1750

    def TestTimeoutScalesRemainingBudgetWithoutResettingWatchdog(Self):
        Controller, Bus = MakeController()
        Controller.SetMotionSpeed(100)  # Start this ratio test at its explicit baseline.
        Displace(Controller, Bus)
        with Patch.object(Bridge.time, "monotonic", return_value=100):
            Self.assertTrue(Controller.go_home()[0])
        InitialDeadline = Controller.MotionDeadline
        InitialTimeout = Controller.MotionTimeoutMs
        with Patch.object(Bridge.time, "monotonic", return_value=102):
            Controller.SetMotionSpeed(25)
            Self.assertAlmostEqual(Controller.MotionDeadline - 102, (InitialDeadline - 102) * 4)
            Self.assertAlmostEqual(Controller.MotionTimeoutMs, 2000 + (InitialTimeout - 2000) * 4, delta=1)
            Controller.SetMotionSpeed(100)
            Self.assertAlmostEqual(Controller.MotionDeadline, InitialDeadline)
        with Patch.object(Bridge.time, "monotonic", return_value=InitialDeadline + 1):
            Controller.SetMotionSpeed(10)
            Self.assertEqual(Controller.MotionDeadline, InitialDeadline, "An expired deadline must not be revived")
            Controller.refresh()
        Self.assertIn("Motion settling timed out", Controller.MotionFault)
        Self.assertIsNone(Controller.MotionRoute)
        Self.assertFalse(Bus.torque_enabled, "Protection timeout must coast all motors")

    def TestRouteAndHardwareFaultsStillStopAtMinimumSpeed(Self):
        for Cause in ("route", "hardware"):
            with Self.subTest(Cause=Cause):
                Controller, Bus, _ = LowController()
                Controller.SetMotionSpeed(10)
                Self.assertTrue(Controller.move_servo("shoulder_pan", Controller.servos["shoulder_pan"].present_raw - 34)[0])
                if Cause == "route": Bus.position["shoulder_pan"] += 200
                else: Bus.registers[("Hardware_Error_Status", "shoulder_pan")] = 32
                Controller.refresh()
                Self.assertIsNotNone(Controller.MotionFault)
                Self.assertIsNone(Controller.MotionRoute)
                Self.assertTrue(all(Servo.command_raw is None for Servo in Controller.servos.values()))

    def TestIdleHoldCorrectionIsIndependentOfThrottle(Self):
        Speeds = []
        for Percent in (10, 100):
            Controller, Bus = MakeController()
            Servo = Controller.servos["shoulder_pan"]
            Servo.Holding = True
            Bus.position[Servo.name] = (Servo.present_raw + 50) % Bridge.RESOLUTION
            Controller.SetMotionSpeed(Percent)
            Controller.refresh()
            Speeds.append(Bus.reg(Servo.name, "Goal_Speed"))
            Self.assertTrue(Bus.torque_enabled)
            Self.assertIsNone(Controller.MotionFault)
        Self.assertNotEqual(Speeds[0], 0)
        Self.assertEqual(Speeds[0], Speeds[1])


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestMotionSpeed` runs the Test* methods too (unittest names this hook)."""
    Prefixed = Unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return Prefixed.loadTestsFromTestCase(MotionSpeedTests)


if __name__ == "__main__":
    Loader = Unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Result = Unittest.TextTestRunner(verbosity=2).run(Loader.loadTestsFromTestCase(MotionSpeedTests))
    raise SystemExit(not Result.wasSuccessful())
