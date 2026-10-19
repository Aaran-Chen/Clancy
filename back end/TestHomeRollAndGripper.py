"""Home brings the wrist roll and the gripper to their saved values too (2026-10-12). Simulated motors only: never opens
a serial port or the depth camera.

The arm joints and the roll ride one certified route. The gripper opens on the way, as a tip move's opening does,
closes only once the arm has arrived (the route's last stage, which moves nothing else), and stays where it is while it
holds an object (a grasp hold). Until then Home left the roll where it was and moved the gripper with the arm, a close
included: a jaw stopped by something tripped the stall protection mid-route and coasted the arm.
"""
import unittest as Unittest
from unittest.mock import patch as Patch

import BasketCollision as Collision
import bridge_server as Bridge
from feetech_simple import OperatingMode
from TestFlyByRoutes import AboveCanRaw, Execute, HomeOpenRaw, MonitorInsideCertificates, ServoModel
from TestGripperWithTip import PlaceRaw, Track
from TestMotorProtection import GripperController, Simulate
from TestSafety import MakeController

HardwareCalls = []
Arm = [Name for Name in Bridge.MOTOR_NAMES if Name != Bridge.GRIPPER_NAME]
OpenRaw = 1919


def setUpModule():
    def GuardedOpener():
        HardwareCalls.append("depth camera opened")
        raise RuntimeError("an offline test reached the real depth camera")
    # As TestFlyByRoutes: long enough that the reach pass's wall clock never decides how a route comes out.
    for Item in (Patch.object(Bridge.depth_camera, "Opener", GuardedOpener),
                 Patch.object(Bridge, "RaiseProcessPriority", lambda: HardwareCalls.append("priority raised") or "normal"),
                 Patch.object(Bridge, "ROUTE_REACH_ASSIGN_SECONDS", 60.0)):
        Item.start()
        Unittest.addModuleCleanup(Item.stop)


def HomeRaw(Controller):
    return {Name: Controller._clamp_raw(Controller.servos[Name], Controller.home_preset[Name]) for Name in Bridge.MOTOR_NAMES}


def SavedHome(Controller, **Values):
    """Home saved with these values (raw), the rest as home_preset.json has them."""
    return Patch.object(Controller, "_load_home", return_value={**Controller.home_preset, **Values})


def Off(Controller, Name, Raw, Goal):
    return abs(Track(Controller, Name, Raw) - Track(Controller, Name, Goal))


def Run(Test, Controller, Bus, Frames, Each):
    """Execute's loop (the servo model, 30 Hz) for Frames frames, Each(frame) after every refresh; stops when Each says so."""
    Model = ServoModel(Bus)
    for Frame in range(Frames):
        Controller.last_refresh_s = Bridge.time.monotonic()
        Model.Step()
        Controller.refresh()
        Test.assertIsNone(Controller.MotionFault)
        if Each(Frame):
            return Frame
    return None


class HomeRollAndGripperTests(Unittest.TestCase):
    def TestHomeTurnsTheWristRollHome(Self):
        Controller, Bus = MakeController()
        Home = HomeRaw(Controller)
        Roll = Controller.servos["wrist_roll"]
        Away = (Home["wrist_roll"] + 400) % 4096
        Self.assertTrue(Controller._contains_raw(Roll, Away))
        PlaceRaw(Controller, Bus, {**AboveCanRaw, "wrist_roll": Away, "gripper": Home["gripper"]})
        Ok, Message = Controller.go_home()
        Self.assertTrue(Ok, Message)
        Self.assertEqual(Controller.MotionRoute["Path"][-1], Home)
        Execute(Self, Controller, Bus)
        Self.assertLessEqual(Off(Controller, "wrist_roll", Roll.present_raw, Home["wrist_roll"]), Bridge.SLIDER_GOAL_TOLERANCE_RAW)
        for Name in Arm:
            Self.assertLessEqual(Off(Controller, Name, Controller.servos[Name].present_raw, Home[Name]), Bridge.SLIDER_GOAL_TOLERANCE_RAW, Name)
        Self.assertTrue(Bus.torque_enabled)
        Self.assertFalse(HardwareCalls)

    def TestEveryStageOfHomeIsCertified(Self):
        """The route monitor's region on every stage lies in a certified box: the roll's turn, an opening on the way,
        and the close at Home (each from a pose of 2026-10-03, the jaws open over the can or closed)."""
        Cases = {"roll and close": ({**AboveCanRaw, "wrist_roll": 1900}, {}),
                 "open on the way": ({**AboveCanRaw, "gripper": 700}, {"gripper": OpenRaw}),
                 "close at Home": (HomeOpenRaw, {})}
        for Case, (Start, Saved) in Cases.items():
            with Self.subTest(Case=Case):
                Controller, Bus = MakeController()
                PlaceRaw(Controller, Bus, Start)
                with SavedHome(Controller, **Saved):
                    MonitorInsideCertificates(Self, Controller, Controller.go_home)

    def TestHomeOpensTheGripperOnTheWay(Self):
        Controller, Bus = MakeController()
        PlaceRaw(Controller, Bus, {**AboveCanRaw, "gripper": 700})
        with SavedHome(Controller, gripper=OpenRaw):
            Ok, Message = Controller.go_home()
        Self.assertTrue(Ok, Message)
        Route = Controller.MotionRoute
        Path, Start = Route["Path"], Route["Start"]
        Self.assertEqual(Path[-1], HomeRaw(Controller))
        # The jaws only widen, and they move on stages the arm moves on: not a gripper-only stage first.
        Column = [Track(Controller, "gripper", Point["gripper"]) for Point in (Start, *Path)]
        Self.assertEqual(Column, sorted(Column))
        Together = [Index for Index, (Before, After) in enumerate(zip((Start, *Path), Path))
                    if Before["gripper"] != After["gripper"] and any(Before[Name] != After[Name] for Name in Arm)]
        Self.assertIn(0, Together)
        Middle = []

        def Watch(_):
            Lift = Controller.servos["shoulder_lift"].present_raw
            if min(Off(Controller, "shoulder_lift", Lift, Point["shoulder_lift"]) for Point in (Start, Path[-1])) > 100:
                Middle.append(Controller.servos["gripper"].present_raw)
        Execute(Self, Controller, Bus, Each=Watch)
        Self.assertTrue(any(750 < Jaw < OpenRaw - 50 for Jaw in Middle), Middle)
        Self.assertLessEqual(Off(Controller, "gripper", Controller.servos["gripper"].present_raw, OpenRaw), Bridge.GRIPPER_GOAL_TOLERANCE_RAW)

    def TestHomeClosesTheGripperOnlyOnceTheArmHasArrived(Self):
        for Percent in (18, 100):
            with Self.subTest(Percent=Percent):
                Self.ClosesAfterArrival(Percent)

    def ClosesAfterArrival(Self, Percent):
        Controller, Bus = MakeController()
        Controller.SetMotionSpeed(Percent)
        PlaceRaw(Controller, Bus, {**AboveCanRaw, "wrist_roll": 1900})
        Home = HomeRaw(Controller)
        Ok, Message = Controller.go_home()
        Self.assertTrue(Ok, Message)
        Route = Controller.MotionRoute
        Path = Route["Path"]
        # The arm's route keeps the jaws open; its last stage closes them from Home, after an arrive waypoint.
        Self.assertTrue(all(Point["gripper"] == OpenRaw for Point in Path[:-1]))
        Self.assertEqual(Path[-2], {**Home, "gripper": OpenRaw})
        Self.assertEqual(Path[-1], Home)
        Self.assertTrue(len(Path) - 2 in Route["Arrive"] | Route["Tight"])
        Ceiling = Controller.MotionSpeedCeilings(Percent)
        Fastest, Began = {}, []

        def Watch(Frame):
            for Register, Name, Value in Bus.writes:
                if Register == "Goal_Speed" and Bus.reg(Name, "Operating_Mode") == OperatingMode.VELOCITY.value:
                    Fastest[Name] = max(Fastest.get(Name, 0), abs(Value))
            Bus.writes.clear()
            Now = Controller.MotionRoute
            Jaw = Controller.servos["gripper"].present_raw
            if Now is not None and Now["Index"] == len(Path) - 1 and not Began:
                Began.append({Name: Off(Controller, Name, Controller.servos[Name].present_raw, Home[Name]) for Name in Arm})
            if not Began:
                Self.assertLessEqual(Off(Controller, "gripper", Jaw, OpenRaw), Bridge.GRIPPER_GOAL_TOLERANCE_RAW, f"frame {Frame}")
        Execute(Self, Controller, Bus, Each=Watch)
        # Every arm joint had arrived (within the tight tolerance) when the close began.
        Self.assertTrue(Began)
        Self.assertLessEqual(max(Began[0].values()), Controller.RouteTightToleranceRaw(), Began[0])
        Self.assertLessEqual(Off(Controller, "gripper", Controller.servos["gripper"].present_raw, Home["gripper"]), Bridge.GRIPPER_GOAL_TOLERANCE_RAW)
        Self.assertLessEqual(Off(Controller, "wrist_roll", Controller.servos["wrist_roll"].present_raw, Home["wrist_roll"]), Bridge.SLIDER_GOAL_TOLERANCE_RAW)
        Self.assertEqual(Controller.status, "Basket-safe motion complete")
        # No joint ran past its speed ceiling, the gripper on its own stage included.
        for Name, Speed in Fastest.items():
            Self.assertLessEqual(Speed, Ceiling[Name] + 1, Name)
        Self.assertGreater(Fastest.get("gripper", 0), 0)

    def TestOnlyTheGripperToMoveIsAGripperMove(Self):
        Controller, Bus = MakeController()
        Home = HomeRaw(Controller)
        PlaceRaw(Controller, Bus, {**Home, "gripper": OpenRaw})
        Ok, Message = Controller.go_home()
        Self.assertTrue(Ok, Message)
        Self.assertEqual(Controller.MotionRoute["Path"], [Home])
        Execute(Self, Controller, Bus)
        Self.assertLessEqual(Off(Controller, "gripper", Controller.servos["gripper"].present_raw, Home["gripper"]), Bridge.GRIPPER_GOAL_TOLERANCE_RAW)

    def TestTheJawsWithinTheirToleranceStay(Self):
        Controller, Bus = MakeController()
        Home = HomeRaw(Controller)
        PlaceRaw(Controller, Bus, {**AboveCanRaw, "gripper": Home["gripper"] + 5})
        Ok, Message = Controller.go_home()
        Self.assertTrue(Ok, Message)
        Self.assertTrue(all(Point["gripper"] == Home["gripper"] + 5 for Point in Controller.MotionRoute["Path"]))
        Controller.StopMotion("test done")

    def TestHomeNeverDropsOrSqueezesAHeldObject(Self):
        """The jaws closed on an object (a grasp hold, at raw 1100): Home leaves them squeezing it, whether its saved
        gripper value is more closed (699) or more open (2000), and says so."""
        for Saved in (699, 2000):
            with Self.subTest(Saved=Saved):
                Controller, Bus, Events = GripperController()
                Gripper, Wrist = Controller.servos["gripper"], Controller.servos["wrist_flex"]
                Self.assertTrue(Controller.move_servo("gripper", 642)[0])
                Clock = {}
                Simulate(Controller, Bus, 6, Clock, Low=1100, Until=lambda: Controller.MotionRoute is None)
                Self.assertTrue(Gripper.GraspHold, Controller.status)
                Squeeze = 1100 - Bridge.GRIPPER_GRASP_SQUEEZE_RAW
                Self.assertEqual(Bus.registers[("Goal_Position", "gripper")], Squeeze)
                # The arm is a little away from Home, so Home has a route to fly.
                Away = Wrist.present_raw + 80
                Bus.position["wrist_flex"] = Wrist.present_raw = Wrist.target_raw = Away
                Bus.write("Goal_Position", "wrist_flex", Away)
                Controller.refresh()
                Bus.writes.clear()
                with SavedHome(Controller, gripper=Saved):
                    Ok, Message = Controller.go_home()
                Self.assertTrue(Ok, Message)
                Self.assertTrue(Message.endswith("; " + Bridge.HOME_GRIPPER_HELD_NOTE), Message)
                Self.assertTrue(all(Point["gripper"] == 1100 for Point in Controller.MotionRoute["Path"]))
                Simulate(Controller, Bus, 8, Clock, Low=1100, High=1100, Until=lambda: Controller.MotionRoute is None)
                Self.assertIsNone(Controller.MotionRoute)
                Self.assertIsNone(Controller.MotionFault, Controller.status)
                Self.assertEqual(Controller.status, "Basket-safe motion complete; " + Bridge.HOME_GRIPPER_HELD_NOTE)
                Self.assertTrue(Gripper.GraspHold)
                Self.assertTrue(Bus.torque_enabled)
                # The jaws were only ever told to keep the squeeze: no velocity command, no other goal.
                Writes = [(Register, Value) for Register, Name, Value in Bus.writes if Name == "gripper"]
                Self.assertTrue(Writes)
                Self.assertEqual({Value for Register, Value in Writes if Register == "Goal_Position"}, {Squeeze})
                Self.assertNotIn(("Operating_Mode", OperatingMode.VELOCITY.value), Writes)
                Self.assertEqual(Bus.registers[("Goal_Position", "gripper")], Squeeze)
                Self.assertLessEqual(abs(Wrist.present_raw - Controller.home_preset["wrist_flex"]), Bridge.SLIDER_GOAL_TOLERANCE_RAW)
                Self.assertEqual([Event["kind"] for Event in Events], ["grasp_hold"])

    def TestAGripChangingDuringPlanningRefusesHome(Self):
        Controller, Bus = MakeController()
        PlaceRaw(Controller, Bus, AboveCanRaw)
        Plan = Bridge.BridgeController.PlanHomeRoute

        def Gripping(Snapshot, *Args):
            Controller.servos["gripper"].GraspHold = True
            return Plan(Snapshot, *Args)
        with Patch.object(Bridge.BridgeController, "PlanHomeRoute", Gripping):
            Ok, Message = Controller.go_home()
        Self.assertFalse(Ok)
        Self.assertIn("the gripper's grip changed during route planning", Message)
        Self.assertIsNone(Controller.MotionRoute)
        Self.assertFalse([Write for Write in Bus.writes if Write[0] in ("Goal_Speed", "Goal_Position", "Operating_Mode")])

    def TestStopOrCoastBeforeTheCloseCancelsIt(Self):
        """Stopped while the arm arrives at Home (the close not begun), or during the close: the jaws close no further."""
        Stops = {"Stop and hold": lambda Controller: Controller.StopMotion(),
                 "Hold pressed while moving": lambda Controller: Controller.toggle_torque(True),
                 "Coast": lambda Controller: Controller.toggle_torque(False),
                 "emergency stop": lambda Controller: Controller.emergency_stop()}
        for Case, Stop in Stops.items():
            for During in (False, True):
                with Self.subTest(Case=Case, During=During):
                    Self.StopBeforeTheClose(Stop, During, Case in ("Stop and hold", "Hold pressed while moving"))

    def StopBeforeTheClose(Self, Stop, During, Holds):
        Controller, Bus = MakeController()
        PlaceRaw(Controller, Bus, AboveCanRaw)
        Home = HomeRaw(Controller)
        Ok, Message = Controller.go_home()
        Self.assertTrue(Ok, Message)
        Last = len(Controller.MotionRoute["Path"]) - 1

        def Arriving(_):
            Route = Controller.MotionRoute
            Self.assertIsNotNone(Route)
            Jaw = Controller.servos["gripper"].present_raw
            if During:
                return Route["Index"] == Last and Off(Controller, "gripper", Jaw, OpenRaw) > 150
            return Route["Index"] == Last - 1 and max(Off(Controller, Name, Controller.servos[Name].present_raw, Home[Name])
                                                      for Name in Arm) <= 40
        Self.assertIsNotNone(Run(Self, Controller, Bus, 30 * 90, Arriving))
        Stop(Controller)
        Self.assertIsNone(Controller.MotionRoute)
        Stopped = Controller.servos["gripper"].present_raw
        Bus.writes.clear()
        Model = ServoModel(Bus)
        for _ in range(60):
            Controller.last_refresh_s = Bridge.time.monotonic()
            Model.Step()
            Controller.refresh()
        Self.assertIsNone(Controller.MotionRoute)
        Jaw = Controller.servos["gripper"].present_raw
        # Closing lowers the raw: it went no further than the stop's own braking.
        Self.assertGreaterEqual(Jaw, Stopped - 40, (Stopped, Jaw))
        if not During:
            Self.assertLessEqual(Off(Controller, "gripper", Jaw, OpenRaw), 40)
        Self.assertFalse([Write for Write in Bus.writes if Write[1] == "gripper" and Write[0] == "Operating_Mode"
                          and Write[2] == OperatingMode.VELOCITY.value])
        Self.assertEqual(Bus.torque_enabled, Holds)

    def TestRefusalsMoveNothing(Self):
        Cases = {"calibration": (lambda C: setattr(C, "CalibrationActive", True), "Stop calibration before Home"),
                 "moving": (lambda C: C.move_servo("wrist_roll", C.servos["wrist_roll"].present_raw + 200),
                            "Wait for the arm to stop moving"),
                 "protection latched": (lambda C: C.AutoCoast("test trip"), "Protection is latched"),
                 "stale command": (None, "Motion cancelled: Stop, Hold or Coast was pressed after it was sent")}
        for Case, (Prepare, Expected) in Cases.items():
            with Self.subTest(Case=Case):
                Controller, Bus = MakeController()
                PlaceRaw(Controller, Bus, AboveCanRaw)
                if Prepare:
                    Prepare(Controller)
                Before = {Name: Servo.target_raw for Name, Servo in Controller.servos.items()}
                Bus.writes.clear()
                with Patch.object(Bridge, "QueuedCommandStale", lambda: Prepare is None):
                    Ok, Message = Controller.go_home()
                Self.assertFalse(Ok)
                Self.assertIn(Expected, Message)
                if Case != "moving":
                    Self.assertIsNone(Controller.MotionRoute)
                    Self.assertEqual({Name: Servo.target_raw for Name, Servo in Controller.servos.items()}, Before)
                    Self.assertFalse([Write for Write in Bus.writes if Write[0] in ("Goal_Speed", "Goal_Position", "Operating_Mode")])
                else:
                    Self.assertEqual(Controller.MotionRoute["Path"][-1]["gripper"], AboveCanRaw["gripper"])
                    Controller.StopMotion("test done")

    def TestACloseThatDoesNotCertifyAtHomeRefusesHome(Self):
        Controller, Bus = MakeController()
        PlaceRaw(Controller, Bus, AboveCanRaw)
        Edge = Collision.CollisionWorld.EdgeReason
        Index = Collision.Names.index("gripper")

        def NoCloseAtHome(World, Start, Goal, Budget=256, Relief=None, Reach=(0.0, 0.0)):
            if Goal[Index] < Start[Index] and all(Start[Joint] == Goal[Joint] for Joint in range(Index)):
                return "moving jaw could contact the basket"
            return Edge(World, Start, Goal, Budget, Relief, Reach)
        with Patch.object(Collision.CollisionWorld, "EdgeReason", NoCloseAtHome):
            Ok, Message = Controller.go_home()
        Self.assertFalse(Ok)
        Self.assertEqual(Message, "Home: the gripper cannot close at Home: moving jaw could contact the basket. No motion sent.")
        Self.assertIsNone(Controller.MotionRoute)
        Self.assertFalse([Write for Write in Bus.writes if Write[0] in ("Goal_Speed", "Goal_Position", "Operating_Mode")])

    def TestPreviewLoadsEveryJointsHome(Self):
        """No arm: Home loads the whole saved pose, the roll and the gripper too, into the preview."""
        Controller = Bridge.BridgeController()
        Controller.bus = None
        for Name, Raw in {**AboveCanRaw, "wrist_roll": 1900}.items():
            Controller.servos[Name].present_raw = Controller.servos[Name].target_raw = Raw
        Ok, Message = Controller.go_home()
        Self.assertTrue(Ok, Message)
        Self.assertEqual(Message, "Home loaded in preview")
        Self.assertEqual({Name: Servo.target_raw for Name, Servo in Controller.servos.items()}, HomeRaw(Controller))
        # A held object (not reachable in preview, but the rule is the same): the gripper's target stays.
        Controller.servos["gripper"].GraspHold = True
        Controller.servos["gripper"].target_raw = 1100
        Ok, Message = Controller.go_home()
        Self.assertEqual((Ok, Message), (True, "Home loaded in preview; " + Bridge.HOME_GRIPPER_HELD_NOTE))
        Self.assertEqual(Controller.servos["gripper"].target_raw, 1100)


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestHomeRollAndGripper` runs the Test* methods too (unittest names this hook)."""
    Prefixed = Unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return Prefixed.loadTestsFromTestCase(HomeRollAndGripperTests)


if __name__ == "__main__":
    Loader = Unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Result = Unittest.TextTestRunner(verbosity=2).run(Loader.loadTestsFromTestCase(HomeRollAndGripperTests))
    raise SystemExit(not Result.wasSuccessful())
