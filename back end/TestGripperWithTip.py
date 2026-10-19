"""A tip move that opens the gripper on the way (send_ik gripper_raw), and gripper creep during planning.
Simulated motors only: never opens a serial port or the depth camera.

2026-10-03 14:46 (back end/captures/2026-10-03/recording/ai.jsonl): the AI chained "gripper_before":0.8
with the hover point above a can. The app opened the gripper (raw 704 -> 1922), waited for it to settle,
then sent the tip move; the gripper crept from 1916 to 1919 during planning and the bridge refused the
move ("Arm moved during planning"). Only the gripper opened. Now one route opens it while the arm moves.
"""
import math as Math
import unittest as Unittest
from unittest.mock import patch as Patch

import BasketCollision as Collision
import CadKinematics as Cad
import bridge_server as Bridge
from TestBasketCollision import DisplayToModel, Place
from TestFlyByRoutes import BesideMast, Execute, MonitorInsideCertificates, V7ContactAllowance
from TestSafety import MakeController

# Every servo at 14:46:45, as the model request and the 14:47:16 bridge record give them (the gripper closed).
Raw1446 = {"shoulder_pan": 3474, "shoulder_lift": 3171, "elbow_flex": 2688, "wrist_flex": 855, "wrist_roll": 1570, "gripper": 704}
# The GRASP PLAN's hover point above the can (AI display frame) and the chained open (80%).
Hover1446 = {**DisplayToModel(1.9, 28.9, 11.5), "wrist_pitch": -21.3}
Open1446 = 1922
HardwareCalls = []


def setUpModule():
    def GuardedOpener():
        HardwareCalls.append("depth camera opened")
        raise RuntimeError("an offline test reached the real depth camera")
    for Item in (Patch.object(Bridge.depth_camera, "Opener", GuardedOpener),
                 Patch.object(Bridge, "RaiseProcessPriority", lambda: HardwareCalls.append("priority raised") or "normal")):
        Item.start()
        Unittest.addModuleCleanup(Item.stop)


def PlaceRaw(Controller, Bus, Raw):
    for Name, Value in Raw.items():
        Servo = Controller.servos[Name]
        Servo.present_raw = Servo.target_raw = Servo.HardwareRaw = Bus.position[Name] = Value
        Servo.command_raw = None
    Bus.configure_motors()
    Controller.refresh()
    Bus.writes.clear()


def Track(Controller, Name, Raw):
    return Controller._slider_track_pos(Controller.servos[Name], Raw)


def ArmMoves(Before, After):
    return any(Before[Name] != After[Name] for Name in Bridge.MOTOR_NAMES if Name != Bridge.GRIPPER_NAME)


class GripperWithTipTests(Unittest.TestCase):
    def TestOpeningRidesWithTheArmAsOneRoute(Self):
        """The 14:46 proposal: one route; the jaws widen from 704 to 1922 on the stages the arm moves."""
        Controller, Bus = MakeController()
        PlaceRaw(Controller, Bus, Raw1446)
        Gripper = Controller.servos["gripper"]
        Ok, Message = Controller.send_ik(Target=Hover1446, GripperRaw=Open1446)
        Self.assertTrue(Ok, Message)
        Route = Controller.MotionRoute
        Path, Start = Route["Path"], Route["Start"]
        Self.assertEqual(Start, Raw1446)
        Self.assertEqual(Path[-1]["gripper"], Open1446)
        # The arm ends where the same move planned with the jaws open ends.
        Plain = Controller.GripperAtSnapshot(Open1446).PlanCartesianMotion({Axis: Hover1446[Axis] for Axis in "xyz"}, Hover1446["wrist_pitch"])
        Self.assertEqual({Name: Path[-1][Name] for Name in Cad.SolveNames}, {Name: Plain[-1][Name] for Name in Cad.SolveNames})
        # The gripper column only widens, from where the jaws are to the target, inside the gripper's limits.
        Column = [Track(Controller, "gripper", Point["gripper"]) for Point in (Start, *Path)]
        Self.assertEqual(Column, sorted(Column))
        Self.assertTrue(all(Controller._contains_raw(Gripper, Point["gripper"]) for Point in Path))
        # It moves on stages the arm moves on: not a gripper-only stage first.
        Together = [Index for Index, (Before, After) in enumerate(zip((Start, *Path), Path))
                    if Before["gripper"] != After["gripper"] and ArmMoves(Before, After)]
        Self.assertIn(0, Together)
        Self.assertGreater(len(Together), 1)
        # The arm is the faster here (18%: the gripper's 1218 steps take longer than the arm's way),
        # so the jaws widen all the way and reach the target on arrival, never slowing the arm.
        Middle = []

        def Watch(_):
            Pan, Jaw = Controller.servos["shoulder_pan"].present_raw, Gripper.present_raw
            if abs(Pan - Raw1446["shoulder_pan"]) > 60 and abs(Pan - Path[-1]["shoulder_pan"]) > 60:
                Middle.append(Jaw)
        Execute(Self, Controller, Bus, Each=Watch)
        Self.assertTrue(Middle)
        # Mid-way the jaws are part open: they open while the arm moves.
        Self.assertTrue(any(800 < Jaw < 1800 for Jaw in Middle), Middle)
        Self.assertLessEqual(Controller._circular_raw_distance(Gripper.present_raw, Open1446), Bridge.GRIPPER_GOAL_TOLERANCE_RAW)
        Tip = Cad.Positions(Controller.arm_geometry, Controller._current_ik_angles())[-1]
        Self.assertLess(Math.dist(tuple(Tip.values()), tuple(Hover1446[Axis] for Axis in "xyz")), .8)
        Self.assertTrue(Bus.torque_enabled)
        Self.assertFalse(HardwareCalls)

    def TestJawsReachTheTargetHalfwayWhenTheirSpeedAllows(Self):
        """A short opening (60% to 80%) is done by GRIPPER_WITH_TIP_OPEN_BY of the arm's route time, at an added waypoint."""
        Controller, Bus = MakeController()
        Part = Controller._fraction_to_raw(Controller.servos["gripper"], 0.6)
        PlaceRaw(Controller, Bus, {**Raw1446, "gripper": Part})
        Ok, Message = Controller.send_ik(Target=Hover1446, GripperRaw=Open1446)
        Self.assertTrue(Ok, Message)
        Path, Start = Controller.MotionRoute["Path"], Controller.MotionRoute["Start"]
        Arm = [Name for Name in Bridge.MOTOR_NAMES if Name != "gripper"]
        Ceilings = Controller.MotionSpeedCeilings(Controller.MotionSpeedPercent)
        Times = [0.0]
        for Before, After in zip((Start, *Path), Path):
            Times.append(Times[-1] + max(abs(Track(Controller, Name, After[Name]) - Track(Controller, Name, Before[Name])) / Ceilings[Name]
                                         for Name in Arm))
        Reached = next(Index for Index, Point in enumerate(Path) if Point["gripper"] == Open1446)
        # Open at the waypoint half way through the arm's time, with the arm still flying on.
        Self.assertAlmostEqual(Times[Reached + 1] / Times[-1], Bridge.GRIPPER_WITH_TIP_OPEN_BY, delta=0.02)
        Self.assertLess(Reached, len(Path) - 1)
        Self.assertTrue(all(Point["gripper"] == Open1446 for Point in Path[Reached:]))
        Execute(Self, Controller, Bus)
        Self.assertLessEqual(Controller._circular_raw_distance(Controller.servos["gripper"].present_raw, Open1446), Bridge.GRIPPER_GOAL_TOLERANCE_RAW)

    def TestEveryStageIsCertifiedWithTheMovingJaw(Self):
        """The route monitor's region on every stage, the moving jaw's swing on it included, lies in a certified box."""
        Controller, Bus = MakeController()
        PlaceRaw(Controller, Bus, Raw1446)
        Boxes = []
        Region = Collision.CollisionWorld.RegionReason

        def Recording(World, Low, High, RearwardSlack=0.0, Inset=None, Check=None):
            Boxes.append((Low.copy(), High.copy()))
            return Region(World, Low, High, RearwardSlack, Inset, Check)
        with Patch.object(Collision.CollisionWorld, "RegionReason", Recording):
            MonitorInsideCertificates(Self, Controller, lambda: Controller.send_ik(Target=Hover1446, GripperRaw=Open1446))
        # Boxes over the jaw's swing were checked against the basket and the robot body: some span the
        # gripper from (or below) its closed start to past the target, with the arm moving too.
        Index = Collision.Names.index("gripper")
        Angle = lambda Raw: Controller._ik_angle_from_raw(Controller.servos["gripper"], Raw)
        Low, High = sorted((Angle(Raw1446["gripper"]), Angle(Open1446)))
        Self.assertTrue(any(Box[0][Index] <= Low + 1e-6 and Box[1][Index] > Low + 10 for Box in Boxes))
        Self.assertTrue(any(Box[1][Index] >= High - 1e-6 and Box[0][Index] < High - 10 for Box in Boxes))

    def TestAJawThatCannotOpenOnTheWayOpensFirstInTheSameRoute(Self):
        """When stages with the jaw moving do not certify, the route opens it where the arm is, then moves."""
        Controller, Bus = MakeController()
        PlaceRaw(Controller, Bus, Raw1446)
        Edge = Collision.CollisionWorld.EdgeReason
        Index = Collision.Names.index("gripper")

        def JawOnlyInPlace(World, Start, Goal, Budget=256, Relief=None, Reach=(0.0, 0.0)):
            if Start[Index] != Goal[Index] and any(Start[Joint] != Goal[Joint] for Joint in range(Index)):
                return "moving jaw could contact the basket"
            return Edge(World, Start, Goal, Budget, Relief, Reach)
        with Patch.object(Collision.CollisionWorld, "EdgeReason", JawOnlyInPlace):
            Ok, Message = Controller.send_ik(Target=Hover1446, GripperRaw=Open1446)
        Self.assertTrue(Ok, Message)
        Route = Controller.MotionRoute
        Path = Route["Path"]
        Self.assertEqual(Path[0], {**Raw1446, "gripper": Open1446})
        Self.assertIn(0, Route["Tight"])
        Self.assertTrue(all(Point["gripper"] == Open1446 for Point in Path))
        Opened = []
        Execute(Self, Controller, Bus, Each=lambda _: Opened.append(
            (Controller.servos["gripper"].present_raw, Controller.servos["shoulder_lift"].present_raw)))
        # The arm waits for the jaws (within the tight tolerance) before it leaves.
        Leaves = next(Frame for Frame, (_, Lift) in enumerate(Opened) if Controller._circular_raw_distance(Lift, Raw1446["shoulder_lift"]) > 12)
        Self.assertLessEqual(abs(Opened[Leaves][0] - Open1446), Bridge.ROUTE_FLY_BY_TOLERANCE_RAW)

    def TestNoOpenFirstRouteFromAPoseWithoutTheAllowance(Self):
        """Beside the mast the opened pose lacks the tracking allowance: no open-first route from it, a refusal.
        (With the v7 contact allowance: under v8's CAD-exact contact that pose has it, V7ContactAllowance.)"""
        with V7ContactAllowance():
            Self.NoOpenFirstRouteWithoutTheAllowance()

    def NoOpenFirstRouteWithoutTheAllowance(Self):
        Controller, Bus = MakeController()
        Place(Controller, Bus, BesideMast)
        Controller.refresh()
        Bus.writes.clear()
        Open = Controller._fraction_to_raw(Controller.servos["gripper"], 0.8)
        Self.assertFalse(Controller.GripperAtSnapshot(Open).StartHasAllowance())
        Edge = Collision.CollisionWorld.EdgeReason
        Index = Collision.Names.index("gripper")

        def JawOnlyInPlace(World, Start, Goal, Budget=256, Relief=None, Reach=(0.0, 0.0)):
            if Start[Index] != Goal[Index] and any(Start[Joint] != Goal[Joint] for Joint in range(Index)):
                return "moving jaw could contact the robot body or camera head"
            return Edge(World, Start, Goal, Budget, Relief, Reach)
        from TestFlyByRoutes import TableTarget, TablePitch
        with Patch.object(Collision.CollisionWorld, "EdgeReason", JawOnlyInPlace):
            Ok, Message = Controller.send_ik(Target={**TableTarget, "wrist_pitch": TablePitch}, GripperRaw=Open)
        Self.assertFalse(Ok)
        Self.assertIn("moving jaw could contact", Message)
        Self.assertIsNone(Controller.MotionRoute)
        Self.assertFalse([Write for Write in Bus.writes if Write[0] in ("Goal_Speed", "Goal_Position", "Operating_Mode")])

    def TestClosingOrNarrowingWithATipMoveIsRefused(Self):
        Controller, Bus = MakeController()
        PlaceRaw(Controller, Bus, {**Raw1446, "gripper": Open1446})
        for Goal in (704, Open1446 - 300):
            Ok, Message = Controller.send_ik(Target=Hover1446, GripperRaw=Goal)
            Self.assertFalse(Ok)
            Self.assertIn("may only open the gripper", Message)
        Self.assertIsNone(Controller.MotionRoute)
        Self.assertFalse(Bus.writes)
        for Bad in ("open", True, float("nan")):
            Ok, Message = Controller.send_ik(Target=Hover1446, GripperRaw=Bad)
            Self.assertFalse(Ok)
            Self.assertIn("raw servo position", Message)
        Self.assertFalse(Bus.writes)

    def TestGripperTargetsStayInsideTheGripperLimits(Self):
        Controller, Bus = MakeController()
        PlaceRaw(Controller, Bus, Raw1446)
        Gripper = Controller.servos["gripper"]
        Ok, Message = Controller.send_ik(Target=Hover1446, GripperRaw=2600)
        Self.assertTrue(Ok, Message)
        Path = Controller.MotionRoute["Path"]
        Self.assertEqual(Path[-1]["gripper"], Gripper.range_max)
        Self.assertTrue(all(Controller._contains_raw(Gripper, Point["gripper"]) for Point in Path))
        Controller.StopMotion("test done")

    def TestAGripperTargetWhereTheJawsAreIsAPlainTipMove(Self):
        Controller, Bus = MakeController()
        PlaceRaw(Controller, Bus, Raw1446)
        Ok, Message = Controller.send_ik(Target=Hover1446, GripperRaw=Raw1446["gripper"] + 5)
        Self.assertTrue(Ok, Message)
        Self.assertTrue(all(Point["gripper"] == Raw1446["gripper"] for Point in Controller.MotionRoute["Path"]))
        Controller.StopMotion("test done")

    def TestTheGripperNeverRunsPastItsSpeedCeiling(Self):
        for Percent in (18, 100):
            with Self.subTest(Percent=Percent):
                Controller, Bus = MakeController()
                Controller.SetMotionSpeed(Percent)
                PlaceRaw(Controller, Bus, Raw1446)
                Ok, Message = Controller.send_ik(Target=Hover1446, GripperRaw=Open1446)
                Self.assertTrue(Ok, Message)
                Ceiling = Controller.MotionSpeedCeilings(Percent)
                Fastest = {}

                def Watch(_):
                    for Register, Name, Value in Bus.writes:
                        if Register == "Goal_Speed":
                            Fastest[Name] = max(Fastest.get(Name, 0), abs(Value))
                    Bus.writes.clear()
                Execute(Self, Controller, Bus, Each=Watch)
                for Name, Speed in Fastest.items():
                    Self.assertLessEqual(Speed, math_ceil(Ceiling[Name]), Name)
                Self.assertGreater(Fastest.get("gripper", 0), 0)

    def TestGripperCreepDuringPlanningDoesNotRefuseTheMove(Self):
        """14:46: the gripper 3 steps short of its open, creeping on while the tip move planned."""
        Cases = {"14:46 creep": ("gripper", 3, True), "a grasp settling": ("gripper", 8, True),
                 "past the goal tolerance": ("gripper", 9, False), "an arm joint": ("shoulder_pan", 3, False)}
        for Name, (Joint, Creep, Accepted) in Cases.items():
            with Self.subTest(Case=Name):
                Controller, Bus = MakeController()
                PlaceRaw(Controller, Bus, {**Raw1446, "gripper": 1916})
                Controller.servos["gripper"].target_raw = Open1446
                Plan = Bridge.BridgeController.PlanCartesianMotion

                def Creeping(Snapshot, *Args, **Options):
                    Bus.position[Joint] += Creep
                    return Plan(Snapshot, *Args, **Options)
                with Patch.object(Bridge.BridgeController, "PlanCartesianMotion", Creeping):
                    Ok, Message = Controller.send_ik(Target=Hover1446)
                Self.assertEqual(Ok, Accepted, Message)
                if Accepted:
                    Self.assertEqual(Controller.MotionRoute["Start"]["gripper"], 1916)
                    Execute(Self, Controller, Bus)
                else:
                    Self.assertEqual(Message, "Arm moved during planning; retry from its new position")

    def TestGripperCreepNeedsTheFullAllowanceAtTheStart(Self):
        """Beside the mast the start pose lacks the tracking allowance: any creep refuses, as before.
        (With the v7 contact allowance: under v8's CAD-exact contact that pose has it, V7ContactAllowance.)"""
        with V7ContactAllowance():
            Self.CreepNeedsTheFullAllowance()

    def CreepNeedsTheFullAllowance(Self):
        Controller, Bus = MakeController()
        Place(Controller, Bus, BesideMast)
        Controller.refresh()
        Self.assertFalse(Controller.StartHasAllowance())
        Plan = Bridge.BridgeController.PlanCartesianMotion

        def Creeping(Snapshot, *Args, **Options):
            Bus.position["gripper"] += 3
            return Plan(Snapshot, *Args, **Options)
        from TestFlyByRoutes import TableTarget, TablePitch
        with Patch.object(Bridge.BridgeController, "PlanCartesianMotion", Creeping):
            Ok, Message = Controller.send_ik(Target={**TableTarget, "wrist_pitch": TablePitch})
        Self.assertFalse(Ok)
        Self.assertEqual(Message, "Arm moved during planning; retry from its new position")

    def TestTheBridgeSaysItTakesAGripperWithATipMove(Self):
        Controller, _ = MakeController()
        Self.assertEqual(Controller.state_snapshot()["gripper_with_tip_version"], "open-while-moving-v1")
        # The websocket and /motion command carry it through.
        Seen = {}
        Controller.send_ik = lambda **Options: Seen.update(Options) or (True, "ok")
        Controller.handle_command({"type": "send_ik", "target": {"x": 1, "y": 2, "z": 3, "wrist_pitch": 0}, "gripper_raw": 1922})
        Self.assertEqual(Seen["GripperRaw"], 1922)


def math_ceil(Value):
    return Math.ceil(Value)


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestGripperWithTip` runs the Test* methods too (unittest names this hook)."""
    Prefixed = Unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return Prefixed.loadTestsFromTestCase(GripperWithTipTests)


if __name__ == "__main__":
    Loader = Unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Result = Unittest.TextTestRunner(verbosity=2).run(Loader.loadTestsFromTestCase(GripperWithTipTests))
    raise SystemExit(not Result.wasSuccessful())
