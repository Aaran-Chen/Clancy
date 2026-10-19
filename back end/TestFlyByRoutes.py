"""Leaving poses beside the mast, hold drift, fly-by waypoints and synchronized joints. Simulated motors only."""
import math as Math
import unittest as Unittest
from unittest.mock import patch as Patch

import numpy as Np
import BasketCollision as Collision
import CadKinematics as Cad
import bridge_server as Bridge
from feetech_simple import OperatingMode
from TestSafety import MakeController
from TestBasketCollision import Place
from TestBasketRouteOptimization import ObstacleWorld

# Reported pose: upright beside the mast, shoulder lift at its limit. Clear of the
# model, but inside the tracking allowance of the mast, so every route was refused.
BesideMast = {"shoulder_pan": -26.54, "shoulder_lift": 103.54, "elbow_flex": 5.19, "wrist_flex": -37.62,
              "wrist_roll": 100.9, "gripper": 0.09}
TableTarget = {"x": 3.7339702738078357, "y": 43.8955547489971, "z": 4.261681908293165}
# Measured on the hardware with the arm lying on the basket rim after a coast; the
# second shows 1.8 cm of model overlap.
RestingOnBasketRaw = [
    {"shoulder_pan": 3488, "shoulder_lift": 1229, "elbow_flex": 1114, "wrist_flex": 643, "wrist_roll": 1439, "gripper": 701},
    {"shoulder_pan": 3489, "shoulder_lift": 1276, "elbow_flex": 1086, "wrist_flex": 659, "wrist_roll": 1439, "gripper": 701},
]
TablePitch = -2.096338539012045
FrameSeconds = 1 / 30
# Achievable speeds in raw steps/s; the loaded shoulder lags.
JointCaps = {"shoulder_pan": 300, "shoulder_lift": 170, "elbow_flex": 240, "wrist_flex": 330, "wrist_roll": 350, "gripper": 350}
# The STS3215 spec speeds at the 8.9 V the servos report (Cad/MotionSpeed.md): fast enough that the bridge's own
# braking bounds, not the servos, set the pace at 100%.
SpecCaps = {"shoulder_pan": 2280, "shoulder_lift": 950, "elbow_flex": 1500, "wrist_flex": 1970, "wrist_roll": 2280, "gripper": 2280}
# Measured on the arm at Home with the jaws open (2026-10-03 records), and the hover over the can it was sent to
# at 14:49 (the app's display frame: X mirrored).
HomeOpenRaw = {"shoulder_pan": 3474, "shoulder_lift": 3171, "elbow_flex": 2688, "wrist_flex": 855, "wrist_roll": 1570, "gripper": 1919}
HoverDisplay, HoverPitch = {"x": 1.9, "y": 28.6, "z": 11.3}, -21.3
# Closed on the can beside the basket rim, and the lift the app sent at 14:52 (display frame).
AtCanRaw = {"shoulder_pan": 3146, "shoulder_lift": 433, "elbow_flex": 2089, "wrist_flex": 251, "wrist_roll": 1570, "gripper": 648}
LiftDisplay, LiftPitch = {"x": 1.986, "y": 28.429, "z": 10.137}, -22.412
# Over the can, and the descent to it the app sent at 14:50:27 (display frame).
AboveCanRaw = {"shoulder_pan": 3146, "shoulder_lift": 201, "elbow_flex": 2011, "wrist_flex": 563, "wrist_roll": 1570, "gripper": 1919}
DescentDisplay, DescentPitch = {"x": 1.899, "y": 28.646, "z": 4.549}, -21.3
# 2026-10-04: Home at the end of the 23:48:48 route, the hover beside the pill bottle approved at 23:49:53 (opening
# the jaws to 1922 on the way), and the servo record of the 23:50:01 motion fault at the switch to its last stage.
Home2348Raw = {"shoulder_pan": 3479, "shoulder_lift": 3161, "elbow_flex": 2686, "wrist_flex": 856, "wrist_roll": 1422, "gripper": 699}
Hover2349Display, Hover2349Pitch, Open2349 = {"x": 13.099999999999998, "y": 32.5, "z": 8.9}, -1.2, 1922
Fault2350Raw = {"shoulder_pan": 3001, "shoulder_lift": 770, "elbow_flex": 1433, "wrist_flex": 336, "wrist_roll": 1421, "gripper": 1918}


class ServoModel:
    """Wraps a FakeBus: 30 Hz frames, 5100 steps/s^2 acceleration, per-joint speed caps."""

    def __init__(Self, Bus):
        Self.Bus, Self.Exact, Self.Velocity = Bus, {}, {}

    def Step(Self):
        for Name in Bridge.MOTOR_NAMES:
            Exact = Self.Exact.get(Name, float(Self.Bus.position[Name]))
            if round(Exact) % 4096 != Self.Bus.position[Name]:
                Exact = float(Self.Bus.position[Name])
            Velocity = Self.Velocity.get(Name, 0.0)
            if Self.Bus.reg(Name, "Operating_Mode") == OperatingMode.VELOCITY.value:
                Wanted = float(Self.Bus.reg(Name, "Goal_Speed"))
            else:
                Error = (Self.Bus.reg(Name, "Goal_Position") - Exact + 2048) % 4096 - 2048
                Cap = abs(Self.Bus.reg(Name, "Goal_Speed")) or 3000
                Wanted = max(-Cap, min(Cap, Error * 6.0))
            Wanted = max(-JointCaps[Name], min(JointCaps[Name], Wanted))
            Velocity += max(-5100 * FrameSeconds, min(5100 * FrameSeconds, Wanted - Velocity))
            Exact += Velocity * FrameSeconds
            Self.Exact[Name], Self.Velocity[Name] = Exact, Velocity
            Self.Bus.position[Name] = int(round(Exact)) % 4096


def setUpModule():
    """The routes here must not depend on how busy the PC is. The reach pass (bridge_server.AssignRouteReach) stops at
    ROUTE_REACH_ASSIGN_SECONDS (1.2 s of wall clock), so on a slower or busier PC the wider reaches went untried and a
    route came out different (TestRouteEndAcceptsWhatTheStageBeforeLetAJointOvershoot raised KeyError 11, audit 2's
    test table; it passed 1 run in 5, audit 1 L5). Long enough that the clock never decides, as in TestRouteCheckPool
    and TestMotionDispatch."""
    Clock = Patch.object(Bridge, "ROUTE_REACH_ASSIGN_SECONDS", 60.0)
    Clock.start()
    Unittest.addModuleCleanup(Clock.stop)


def V7ContactAllowance():
    """The v7 contact model: basket and body contact with the full tracking allowance. Under v8
    (CAD-exact contact) BesideMast has the allowance; with this it lacks it again, so the tests of
    leaving (or opening the jaws at) a pose without it keep exercising that path. A pose whose
    box meets the model, or that lacks the rearward allowance, takes it under v8 too."""
    return Patch.dict(Collision.Model, {"ContactTrackingAllowanceRaw": Collision.Model["JointTrackingAllowanceRaw"]})


def MonitorInsideCertificates(Test, Controller, Send):
    """Plan and start a route with Send, then check that every joint position the route
    monitor accepts on each stage lies in a box certified for that stage (by the planner
    or by the bridge). Since v8 (CAD-exact contact) a stage has two certificates: the
    rearward limit's covers every position the monitor accepts, as before; contact's covers
    them less the tracking allowance the contact checks no longer add (ContactTrackingAllowanceRaw),
    the region the route commands.

    Returns (stationary joints whose accepted range was widened by waypoint reach, relieved stages).
    """
    Certified, Edge, RegionReason = {}, Collision.CollisionWorld.EdgeReason, Collision.CollisionWorld.RegionReason

    def Recording(World, Start, Goal, Budget=256, Relief=None, Reach=(0.0, 0.0)):
        Regions = {}

        def Region(Self, Low, High, RearwardSlack=0.0, Inset=None, Check=None):
            Regions.setdefault(Check, (Low.copy(), High.copy()))
            return RegionReason(Self, Low, High, RearwardSlack, Inset, Check)
        with Patch.object(Collision.CollisionWorld, "RegionReason", Region):
            Reason = Edge(World, Start, Goal, Budget, Relief, Reach)
        if Reason is None:
            Certified.setdefault((tuple(Start), tuple(Goal)), []).append((Regions, Relief is not None))
        return Reason
    Waived = (Collision.Model["JointTrackingAllowanceRaw"] - Collision.Model["ContactTrackingAllowanceRaw"]) * 360 / 4096

    with Patch.object(Collision.CollisionWorld, "EdgeReason", Recording):
        Ok, Message = Send()
    Test.assertTrue(Ok, Message)
    Route, Trips = Controller.MotionRoute, []
    Path = Route["Path"]
    Test.assertLess(len(Route["Tight"]), len(Path) - 1)
    Controller.StopMotion = lambda Reason="", Latch=False: Trips.append(Reason)
    Controller.ConfigurationKey = lambda: Route["Configuration"]
    Widened, Relieved = 0, 0
    for Index, Point in enumerate(Path):
        with Controller.lock:
            if Index:
                for Name, Servo in Controller.servos.items():
                    Servo.present_raw = Path[Index - 1][Name]
                Route["Index"] = Index
                Controller.BeginRouteStage()
            Test.assertFalse(Trips)
            Start = dict(Route["Start"])
            Angles = lambda Raw: tuple(Controller._ik_angle_from_raw(Controller.servos[Name], Raw[Name]) for Name in Bridge.MOTOR_NAMES)
            Boxes = Certified[Angles(Start), Angles(Point)]
            Accepted, Faces = {}, {}
            for Name, Servo in Controller.servos.items():
                Values = []
                for Raw in range(min(Start[Name], Point[Name]) - 70, max(Start[Name], Point[Name]) + 71):
                    Servo.present_raw = Raw
                    Controller.CheckMotionRoute()
                    if Trips:
                        Trips.clear()
                    else:
                        Values.append(Controller._ik_angle_from_raw(Servo, Raw))
                Servo.present_raw = Start[Name]
                Accepted[Name] = (min(Values), max(Values))
                Faces[Name] = Np.sign(Controller._ik_angle_from_raw(Servo, Point[Name]) - Controller._ik_angle_from_raw(Servo, Start[Name]))
                if Start[Name] == Point[Name] and len(Values) > 2 * Collision.Model["JointTrackingAllowanceRaw"] + 1:
                    Widened += 1

            def InsideRegion(Low, High, Relief, Shrink):
                for Joint, Name in enumerate(Bridge.MOTOR_NAMES):
                    # A relieved escape certifies the faces at the measured start with that
                    # pose's own clearance, but the monitor keeps the full tracking allowance
                    # there: a trip coasts the arm, which is worse beside the mast. Only the
                    # faces toward the escape waypoint must match.
                    if (not Relief or Faces[Name] < 0) and Accepted[Name][0] + Shrink < Low[Joint] - 1e-6:
                        return False
                    if (not Relief or Faces[Name] > 0) and Accepted[Name][1] - Shrink > High[Joint] + 1e-6:
                        return False
                return True

            def Inside(Box):
                Regions, Relief = Box
                return all(InsideRegion(*Regions[Check], Relief, Waived if Check == "contact" else 0.0) for Check in Regions)
            Relieved += any(Box[1] for Box in Boxes)
            Test.assertTrue(any(Inside(Box) for Box in Boxes), f"stage {Index}: monitor accepts {Accepted} outside {Boxes}")
    return Widened, Relieved


def Execute(Test, Controller, Bus, Frames=30 * 90, Each=None):
    Model = ServoModel(Bus)
    for Frame in range(Frames):
        Controller.last_refresh_s = Bridge.time.monotonic()  # Simulated frames run faster than real time.
        Model.Step()
        Controller.refresh()
        Test.assertIsNone(Controller.MotionFault)
        if Each:
            Each(Frame)
        if Controller.MotionRoute is None:
            return Frame
    Test.fail("Route did not finish")


class FlyByRouteTests(Unittest.TestCase):
    def TestArmBesideMastReachesTableTarget(Self):
        Controller, Bus = MakeController()
        Place(Controller, Bus, BesideMast)
        # v8 (CAD-exact contact): the pose is clear of the mast, so it has the allowance.
        Self.assertIsNone(Controller.CollisionWorld().PoseReason(Controller._current_ik_angles()))
        with V7ContactAllowance():
            Self.LeaveBesideMastForTableTarget(Controller, Bus)

    def LeaveBesideMastForTableTarget(Self, Controller, Bus):
        World = Controller.CollisionWorld()
        Start = Controller._current_ik_angles()
        Self.assertIn("robot body", World.PoseReason(Start))
        Self.assertIsNone(World.PoseReason(Start, Tracking=False))
        Ok, Message = Controller.send_ik(Target={**TableTarget, "wrist_pitch": TablePitch})
        Self.assertTrue(Ok, Message)
        # Only leading stages may use the start pose's own clearance; the rest pass the full check.
        Check, Strict, Relieved = Controller.SolverSnapshot(), [], 0
        Relief = World.StartRelief(Np.array([Start[Name] for Name in Cad.JointNames]))
        Self.assertIsNotNone(Relief)
        for Raw in Controller.MotionRoute["Path"]:
            try:
                Check.ValidateMotionPath(Raw)
                Strict.append(True)
            except ValueError:
                Self.assertTrue(all(not Value for Value in Strict), "Relief after a fully checked stage")
                Check.ValidateMotionPath(Raw, Relief=Relief)
                Relieved += 1
                Strict.append(False)
            for Name, Value in Raw.items():
                Check.servos[Name].present_raw = Value
        Self.assertEqual(Relieved, 1)
        Execute(Self, Controller, Bus)
        Tip = Cad.Positions(Controller.arm_geometry, Controller._current_ik_angles())[-1]
        Self.assertLess(Math.dist(tuple(Tip.values()), tuple(TableTarget.values())), .8)
        Self.assertTrue(Bus.torque_enabled)

    def TestHoldBesideMastCorrectsDriftWithoutCoasting(Self):
        Controller, Bus = MakeController()
        Place(Controller, Bus, BesideMast)
        for Servo in Controller.servos.values():
            Servo.Holding = True
        Controller.refresh()
        Hold = Controller.servos["elbow_flex"].target_raw
        Bus.position["elbow_flex"] += 20
        Model = ServoModel(Bus)
        for _ in range(90):
            Controller.last_refresh_s = Bridge.time.monotonic()
            Model.Step()
            Controller.refresh()
            Self.assertIsNone(Controller.MotionFault)
        Self.assertTrue(Bus.torque_enabled)
        Self.assertLessEqual(Controller._circular_raw_distance(Controller.servos["elbow_flex"].present_raw, Hold),
                             Bridge.SLIDER_GOAL_TOLERANCE_RAW)

    def TestUncheckableHoldReturnKeepsServoHoldInsteadOfCoasting(Self):
        Controller, Bus = MakeController()
        Place(Controller, Bus, BesideMast)
        for Servo in Controller.servos.values():
            Servo.Holding = True
        Controller.refresh()
        Goal = Bus.reg("elbow_flex", "Goal_Position")
        Bus.position["elbow_flex"] += 20
        Blocked = Controller.CollisionWorld()
        Blocked.EdgeReason = lambda *Args, **Options: "robot body or camera head"
        Controller.CollisionWorld = lambda Calibration=False, Scene=True: Blocked
        for _ in range(3):
            Controller.refresh()
        Self.assertIsNone(Controller.MotionFault)
        Self.assertTrue(Bus.torque_enabled)
        # The servo keeps its own position hold on the original goal.
        Self.assertEqual(Bus.reg("elbow_flex", "Operating_Mode"), OperatingMode.POSITION.value)
        Self.assertEqual(Bus.reg("elbow_flex", "Goal_Position"), Goal)
        Self.assertIn("still holding", Controller.status)

    def TestDeepModelContactAtStartIsRefusedWithoutWrites(Self):
        Controller, Bus = MakeController()
        World = Controller.CollisionWorld()
        Pose = dict(BesideMast)
        while World.PoseReason(Pose, Tracking=False, RearwardSlack=Math.inf) is None:
            Pose["shoulder_pan"] -= 0.25
        # Resting against the mast is still a pose the arm can leave...
        Self.assertTrue(World.StartRelief(Np.array([Pose[Name] for Name in Collision.Names]))["InsetCm"])
        while True:
            try:
                World.StartRelief(Np.array([Pose[Name] for Name in Collision.Names]))
            except ValueError:
                break
            Pose["shoulder_pan"] -= 0.25
        # ...but not a link pressed deeper than the model's looseness allows.
        Place(Controller, Bus, Pose)
        Ok, Message = Controller.send_ik(Target={**TableTarget, "wrist_pitch": TablePitch})
        Self.assertFalse(Ok)
        Self.assertIn("move the arm clear by hand", Message)
        Self.assertFalse(Bus.writes)

    def TestArmRestingOnTheBasketLiftsOffWithoutPressingFurtherIn(Self):
        for Pose in RestingOnBasketRaw:
            with Self.subTest(Pose=Pose):
                Self.CheckLiftOff(Pose)

    def CheckLiftOff(Self, Pose):
        Controller, Bus = MakeController()
        Controller.SetMotionSpeed(100)
        for Name, Raw in Pose.items():
            Servo = Controller.servos[Name]
            Servo.present_raw = Servo.target_raw = Servo.HardwareRaw = Bus.position[Name] = Raw
        Bus.configure_motors()
        Controller.refresh()
        World = Controller.CollisionWorld()
        Start = Np.array([Controller._current_ik_angles()[Name] for Name in Collision.Names])
        Self.assertIn("basket", World.PoseReason(dict(zip(Collision.Names, Start)), Tracking=False, RearwardSlack=Math.inf))
        Inset = World.StartRelief(Start)["InsetCm"]
        Self.assertTrue(Inset)
        Ok, Message = Controller.go_home()
        Self.assertTrue(Ok, Message)
        Self.assertIn(0, Controller.MotionRoute["Tight"])
        Deepest = {}

        def Watch(_):
            Angles = Np.array([Controller._current_ik_angles()[Name] for Name in Collision.Names])
            if World.IntervalReason(Angles, Angles, False, Math.inf):
                for Index, Depth in World.ContactInset(Angles).items():
                    Deepest[Index] = max(Deepest.get(Index, 0.0), Depth - Collision.ContactReliefMarginCm)
        Execute(Self, Controller, Bus, Each=Watch)
        # Only the boxes that already overlapped ever overlap, and never by more than the margin beyond their start.
        Self.assertLessEqual(set(Deepest), set(Inset))
        for Index, Depth in Deepest.items():
            Self.assertLessEqual(Depth, Inset[Index], (World.Boxes[Index][0], Depth, Inset[Index]))
        Home = {Name: Controller._clamp_raw(Controller.servos[Name], Controller.home_preset[Name]) for Name in Bridge.HOME_MOTOR_NAMES}
        for Name, Raw in Home.items():
            Self.assertLessEqual(Controller._circular_raw_distance(Controller.servos[Name].present_raw, Raw), Bridge.SLIDER_GOAL_TOLERANCE_RAW)

    def TestIntermediateWaypointsAreFlownThroughWithoutParking(Self):
        Controller, Bus = MakeController()
        Controller.SetMotionSpeed(100)
        Place(Controller, Bus, BesideMast)
        Ok, Message = Controller.send_ik(Target={**TableTarget, "wrist_pitch": TablePitch})
        Self.assertTrue(Ok, Message)
        Stages = len(Controller.MotionRoute["Path"])
        Self.assertGreater(Stages, 5)
        Bus.writes.clear()
        Parked = []

        def Watch(_):
            Route = Controller.MotionRoute
            if Route is not None and Route["Index"] < len(Route["Path"]) - 1:
                Parked.extend(Name for Register, Name, Value in Bus.writes
                              if Register == "Operating_Mode" and Value == OperatingMode.POSITION.value)
            Bus.writes.clear()

        Execute(Self, Controller, Bus, Each=Watch)
        # Joints may start or stop streaming when their own motion begins or ends,
        # but never park at every waypoint.
        Self.assertLess(len(Parked), Stages // 2, Parked)

    def TestMovingJointsProgressTogether(Self):
        Controller, Bus = MakeController()
        Controller.SetMotionSpeed(100)
        Start = Controller._current_ik_angles()
        Goal = dict(Start, shoulder_pan=Start["shoulder_pan"] - 5, shoulder_lift=Start["shoulder_lift"] - 40,
                    elbow_flex=Start["elbow_flex"] + 30)
        Self.assertIsNone(Controller.CollisionWorld().PoseReason(Goal))
        Targets = {Name: Controller._ik_raw_from_angle(Controller.servos[Name], Goal[Name]) for Name in Cad.SolveNames}
        Origin = {Name: Controller._slider_track_pos(Controller.servos[Name], Controller.servos[Name].present_raw) for Name in Targets}
        Ok, Message = Controller.SendJointMotion(Targets, "Test")
        Self.assertTrue(Ok, Message)
        Spread = []

        def Watch(_):
            Fractions = []
            for Name in ("shoulder_lift", "elbow_flex"):
                Servo = Controller.servos[Name]
                Travel = Controller._slider_track_pos(Servo, Targets[Name]) - Origin[Name]
                Fractions.append((Controller._slider_track_pos(Servo, Servo.present_raw) - Origin[Name]) / Travel)
            if 0.2 < min(Fractions) < 0.8:
                Spread.append(max(Fractions) - min(Fractions))

        Execute(Self, Controller, Bus, Each=Watch)
        Self.assertTrue(Spread)
        # Independent full-speed joints would differ by far more: the shoulder is capped at 170 steps/s.
        Self.assertLess(max(Spread), 0.2)

    def TestSynchronizedSpeedsKeepTheLeadJointAtCruise(Self):
        Controller, Bus = MakeController()
        Controller.SetMotionSpeed(100)
        Start = Controller._current_ik_angles()
        Goal = dict(Start, shoulder_lift=Start["shoulder_lift"] - 40, elbow_flex=Start["elbow_flex"] + 30)
        Targets = {Name: Controller._ik_raw_from_angle(Controller.servos[Name], Goal[Name]) for Name in Cad.SolveNames}
        Ok, Message = Controller.SendJointMotion(Targets, "Test")
        Self.assertTrue(Ok, Message)
        # The lift leads, and cruises at its own 100% ceiling (lower than the top one).
        Cruise = Controller.MotionSpeedCeilings(100)["shoulder_lift"]
        with Controller.lock, Patch.object(Bridge.BridgeController, "RouteBrakingSpeed", staticmethod(lambda Room, Overshoot=None: Math.inf)):
            Speeds = Controller.RouteSpeeds()
        Lift, Elbow = abs(Speeds["shoulder_lift"]), abs(Speeds["elbow_flex"])
        Self.assertEqual(Lift, Cruise)
        Self.assertAlmostEqual(Elbow / Lift, 30 / 40, delta=0.05)
        # Until the next leg can start, no joint approaches faster than it could stop within its checked reach: the
        # one this waypoint's stage was certified with for that joint (RouteStopRoom).
        with Controller.lock:
            Capped = Controller.RouteSpeeds()
            Point = Controller.MotionRoute["Path"][0]
            for Name, Speed in Capped.items():
                Servo = Controller.servos[Name]
                Error = abs(Controller._slider_track_pos(Servo, Point[Name]) - Controller._slider_track_pos(Servo, Servo.present_raw))
                Room, Overshoot = Controller.RouteStopRoom(0, Name)
                Self.assertLessEqual(abs(Speed), Controller.RouteBrakingSpeed(Error + Room, Overshoot) + 1)

    def TestRouteMonitorStaysInsideEveryCertifiedStage(Self):
        # Leaving BesideMast: with the v7 contact allowance its first stage is relieved; under v8 (CAD-exact
        # contact) it has the allowance and none is.
        for Model, Expected in ((V7ContactAllowance, 1), (lambda: Patch.dict(Collision.Model, {}), 0)):
            with Model():
                Controller, Bus = MakeController()
                Controller.SetMotionSpeed(100)
                Place(Controller, Bus, BesideMast)
                Widened, Relieved = MonitorInsideCertificates(Self, Controller,
                    lambda: Controller.send_ik(Target={**TableTarget, "wrist_pitch": TablePitch}))
                Self.assertEqual(Relieved, Expected)

    def TestRouteMonitorCoversJointsSettlingAtBends(Self):
        Controller, Bus = MakeController()
        Start = {Name: Servo.present_raw for Name, Servo in Controller.servos.items()}
        Points, Point = [], dict(Start)
        for Name in ("shoulder_pan", "elbow_flex", "elbow_flex", "wrist_flex", "shoulder_pan"):
            Point = {**Point, Name: Point[Name] + 40}
            Points.append(Point)

        class ClearWorld(Collision.CollisionWorld):
            def __init__(Self):
                Self.Slack = Collision.Model["JointTrackingAllowanceRaw"] * 360 / 4096

            def IntervalReason(Self, Low, High, Tracking=True, RearwardSlack=0.0):
                return None

        def Send():
            # Every bend is flown past, so each joint that stops moving at one keeps settling room.
            Path = Controller.AssignRouteReach(Collision.RoutePath(Points))
            Check = Controller.SolverSnapshot()
            for Raw in Path:
                Check.ValidateMotionPath(Raw)
                for Name in Bridge.MOTOR_NAMES:
                    Check.servos[Name].present_raw = Raw[Name]
            with Controller.lock:
                Controller.StartMotionRoute(Path)
            return True, "started"
        with Patch.object(Controller, "CollisionWorld", return_value=ClearWorld()):
            Widened, Relieved = MonitorInsideCertificates(Self, Controller, Send)
        Self.assertEqual(Relieved, 0)
        Self.assertGreater(Widened, 0)

    def TestOnlyJointsSettlingFromAFlownWaypointKeepItsReach(Self):
        Controller, Bus = MakeController()
        Start = {Name: Servo.present_raw for Name, Servo in Controller.servos.items()}
        Moves = ("shoulder_pan", "elbow_flex", "elbow_flex", "wrist_flex", "elbow_flex")
        Points, Point = [], dict(Start)
        for Name in Moves:
            Point = {**Point, Name: Point[Name] + 40}
            Points.append(Point)
        Path = Collision.RoutePath(Points, {3})
        Allowance, Reach = Collision.Model["JointTrackingAllowanceRaw"], Collision.WaypointReachRaw
        Expected = [set(), {"shoulder_pan"}, {"shoulder_pan", "elbow_flex"}, {"shoulder_pan", "elbow_flex"}, set()]
        Trips = []
        with Controller.lock:
            Controller.StartMotionRoute(Path)
            Controller.StopMotion = lambda Reason="", Latch=False: Trips.append(Reason)
            Route = Controller.MotionRoute
            for Index, Displaced in enumerate(Expected):
                if Index:
                    for Name, Servo in Controller.servos.items():
                        Servo.present_raw = Points[Index - 1][Name]
                    Route["Index"] = Index
                    Controller.BeginRouteStage()
                Self.assertEqual(set(Route["Displaced"]), Displaced)
                for Name in ("shoulder_pan", "wrist_roll"):
                    Servo, Origin = Controller.servos[Name], Route["Start"][Name]
                    if Points[Index][Name] != Origin:
                        continue  # Moving on this stage.
                    Limit = Allowance + (Reach if Name in Displaced else 0)
                    for Offset, Trip in ((Limit, False), (-Limit, False), (Limit + 1, True), (-Limit - 1, True)):
                        Servo.present_raw = Origin + Offset
                        Controller.CheckMotionRoute()
                        Self.assertEqual(bool(Trips), Trip, (Index, Name, Offset))
                        Trips.clear()
                    Servo.present_raw = Origin
        # Reach assignment certifies with the same joints. A reach that fails at
        # waypoint 3 makes it tight, which clears the reach for the stage after it.
        Used = {}

        def Validate(Check, Targets, Calibration=False, Relief=None, Reach=({}, 0), Budget=256):
            Index = next(Index for Index, Point in enumerate(Points) if Point == Targets)
            if Index == 3 and Reach[1]:
                raise ValueError("reach blocked")
            Used[Index] = {Name for Name, Steps in Reach[0].items() if Steps}
        for Name, Servo in Controller.servos.items():
            Servo.present_raw = Start[Name]
        with Patch.object(Bridge.BridgeController, "ValidateMotionPath", Validate):
            Assigned = Controller.AssignRouteReach(Collision.RoutePath(Points))
        Self.assertEqual(Assigned.Tight, {3})
        Self.assertEqual([Used[Index] for Index in range(4)], Expected[:4])
        Self.assertNotIn(4, Used)  # Between a tight waypoint and the route end there is no reach to check.

    def TestTightWaypointsArePassedInsideTheAllowanceWithoutParking(Self):
        Controller, Bus = MakeController()
        Controller.SetMotionSpeed(100)
        Start = {Name: Servo.present_raw for Name, Servo in Controller.servos.items()}
        Angles = Controller._current_ik_angles()
        Raw = lambda Name, Angle: Controller._ik_raw_from_angle(Controller.servos[Name], Angle)
        Points = [{**Start, "shoulder_lift": Raw("shoulder_lift", Angles["shoulder_lift"] - 6 * Step),
                   "elbow_flex": Raw("elbow_flex", Angles["elbow_flex"] + 4.5 * Step)} for Step in range(1, 6)]
        Tight = {1, 2}
        Passed, Parked, Last = [], [], [0]
        with Controller.lock:
            Controller.StartMotionRoute(Collision.RoutePath(Points, Tight))
        Bus.writes.clear()

        def Watch(_):
            Route = Controller.MotionRoute
            Index = len(Points) if Route is None else Route["Index"]
            if Index != Last[0] and Last[0] < len(Points) - 1:
                Point = Points[Last[0]]
                Passed.append((Last[0], max(Controller._circular_raw_distance(Servo.present_raw, Point[Name])
                                            for Name, Servo in Controller.servos.items())))
            Last[0] = Index
            if Route is not None and Route["Index"] < len(Points) - 1:
                Parked.extend(Name for Register, Name, Value in Bus.writes
                              if Register == "Operating_Mode" and Value == OperatingMode.POSITION.value)
            Bus.writes.clear()

        Execute(Self, Controller, Bus, Each=Watch)
        Self.assertEqual([Index for Index, _ in Passed], list(range(len(Points) - 1)))
        for Index, Error in Passed:
            if Index in Tight:
                Self.assertLessEqual(Error, Controller.RouteTightToleranceRaw(), Passed)
        Self.assertFalse(Parked)

    def TestTightWaypointLetsAJointCarryOnPastIt(Self):
        Controller, Bus = MakeController()
        Start = {Name: Servo.present_raw for Name, Servo in Controller.servos.items()}
        Points = [{**Start, "elbow_flex": Start["elbow_flex"] + 60},
                  {**Start, "elbow_flex": Start["elbow_flex"] + 120},
                  {**Start, "elbow_flex": Start["elbow_flex"] + 120, "shoulder_pan": Start["shoulder_pan"] + 60}]
        with Controller.lock:
            Controller.StartMotionRoute(Collision.RoutePath(Points, {0, 1}))
            Route = Controller.MotionRoute
            Elbow = Controller.servos["elbow_flex"]
            # Overshot the first tight waypoint but keeps going the same way: no need to come back.
            Elbow.present_raw = Points[0]["elbow_flex"] + 12
            Self.assertTrue(Controller.RouteWaypointPassed())
            Self.assertNotIn("elbow_flex", Controller.RouteSpeeds())
            for Name, Servo in Controller.servos.items():
                Servo.present_raw = Points[0][Name]
            Route["Index"] = 1
            Controller.BeginRouteStage()
            # Overshot the second, where it stops: it must return within the tight tolerance.
            Elbow.present_raw = Points[1]["elbow_flex"] + 12
            Self.assertFalse(Controller.RouteWaypointPassed())
            Self.assertIn("elbow_flex", Controller.RouteSpeeds())
            Elbow.present_raw = Points[1]["elbow_flex"] + Controller.RouteTightToleranceRaw()
            Self.assertTrue(Controller.RouteWaypointPassed())

    def TestTightWaypointWaitsForJointsThatHoldStill(Self):
        Controller, Bus = MakeController()
        Start = {Name: Servo.present_raw for Name, Servo in Controller.servos.items()}
        Points = [{**Start, "shoulder_pan": Start["shoulder_pan"] + 60},
                  {**Start, "shoulder_pan": Start["shoulder_pan"] + 60, "elbow_flex": Start["elbow_flex"] + 60},
                  {**Start, "shoulder_pan": Start["shoulder_pan"] + 60, "elbow_flex": Start["elbow_flex"] + 120}]
        for Tight, Waits in ((set(), False), ({1}, True)):
            with Controller.lock:
                Controller.StartMotionRoute(Collision.RoutePath(Points, Tight))
                Route = Controller.MotionRoute
                for Name, Servo in Controller.servos.items():
                    Servo.present_raw = Points[0][Name]
                Route["Index"] = 1
                Controller.BeginRouteStage()
                # The elbow arrived; the pan, which moved into the waypoint before, is still 20 steps past it.
                Controller.servos["elbow_flex"].present_raw = Points[1]["elbow_flex"]
                Controller.servos["shoulder_pan"].present_raw = Points[1]["shoulder_pan"] + 20
                Controller.CheckMotionRoute()
                Self.assertIsNotNone(Controller.MotionRoute)
                Self.assertEqual(Controller.RouteWaypointPassed(), not Waits)
                Controller.servos["shoulder_pan"].present_raw = Points[1]["shoulder_pan"] + Controller.RouteTightToleranceRaw()
                Self.assertTrue(Controller.RouteWaypointPassed())
                Controller.MotionRoute = None

    @staticmethod
    def PlaceRaw(Percent, Raw):
        Controller, Bus = MakeController()
        for Name, Value in Raw.items():
            Servo = Controller.servos[Name]
            Servo.present_raw = Servo.target_raw = Servo.HardwareRaw = Bus.position[Name] = Value
        Bus.configure_motors()
        Controller.refresh()
        Controller.SetMotionSpeed(Percent)
        return Controller, Bus

    @staticmethod
    def DisplayTarget(Controller, Display, Pitch):
        """A send_ik target for a point in the app's display frame (X mirrored)."""
        Visual = {"x": -Display["x"], "y": Display["y"], "z": Display["z"]}
        return {**Controller.ModelPointFromVisualPoint(Visual), "wrist_pitch": Pitch}

    def SendHoverFromHome(Self, Percent):
        Controller, Bus = Self.PlaceRaw(Percent, HomeOpenRaw)
        Ok, Message = Controller.send_ik(Target=Self.DisplayTarget(Controller, HoverDisplay, HoverPitch))
        Self.assertTrue(Ok, Message)
        return Controller, Bus

    def TestArriveWaypointKeepsItsApproachReachWhenTheNextStageCannotTakeIt(Self):
        """2026-10-03, the lift off the can beside the basket rim: each stage certified its own goal reach but not the
        reach at its start (joints settling from the waypoint before it, toward the rim). Both waypoints became tight,
        four of the five. Now the waypoint before becomes an arrive waypoint and keeps its own reach."""
        Controller, Bus = MakeController()
        Start = {Name: Servo.present_raw for Name, Servo in Controller.servos.items()}
        Points = [{**Start, "elbow_flex": Start["elbow_flex"] + 40 * Step} for Step in (1, 2, 3, 4)]
        Used = []

        def Validate(Check, Targets, Calibration=False, Relief=None, Reach=({}, 0), Budget=256):
            Index = next(Index for Index, Point in enumerate(Points) if Point == Targets)
            Goal = max(Reach[1].values(), default=0) if isinstance(Reach[1], dict) else Reach[1]
            Used.append((Index, frozenset(Name for Name, Steps in Reach[0].items() if Steps), Goal))
            if Index == 2 and any(Reach[0].values()):
                raise ValueError("start reach blocked")
        with Patch.object(Bridge.BridgeController, "ValidateMotionPath", Validate):
            Assigned = Controller.AssignRouteReach(Collision.RoutePath(Points))
        Self.assertEqual(Assigned.Tight, set())
        Self.assertEqual(Assigned.Arrive, {1})
        Self.assertNotIn(1, Assigned.Reach)  # Tried smaller first, but arrived at it keeps the plain reach.
        # Stage 2 certified its own goal reach with none at its start; stage 1 kept the reach it certified with.
        Self.assertIn((2, frozenset(), Collision.WaypointReachRaw), Used)
        Self.assertIn((1, frozenset({"elbow_flex"}), Collision.WaypointReachRaw), Used)
        # A stage that cannot take its own goal reach either way still makes its waypoint tight.
        Used.clear()

        def Blocked(Check, Targets, Calibration=False, Relief=None, Reach=({}, 0), Budget=256):
            Index = next(Index for Index, Point in enumerate(Points) if Point == Targets)
            if Index == 2 and (Reach[1] or any(Reach[0].values())):
                raise ValueError("reach blocked")
        with Patch.object(Bridge.BridgeController, "ValidateMotionPath", Blocked):
            Assigned = Controller.AssignRouteReach(Collision.RoutePath(Points))
        Self.assertEqual((Assigned.Tight, Assigned.Arrive), ({2}, {1}))

    def TestArriveWaypointIsApproachedWithItsReachAndLeftWithNone(Self):
        Controller, Bus = MakeController()
        Start = {Name: Servo.present_raw for Name, Servo in Controller.servos.items()}
        Points = [{**Start, "shoulder_pan": Start["shoulder_pan"] + 60},
                  {**Start, "shoulder_pan": Start["shoulder_pan"] + 120, "elbow_flex": Start["elbow_flex"] + 60},
                  {**Start, "shoulder_pan": Start["shoulder_pan"] + 120, "elbow_flex": Start["elbow_flex"] + 120}]
        Allowance, Reach = Collision.Model["JointTrackingAllowanceRaw"], Collision.WaypointReachRaw
        Trips = []
        with Controller.lock:
            Controller.StartMotionRoute(Collision.RoutePath(Points, Arrive={1}))
            Controller.StopMotion = lambda Reason="", Latch=False: Trips.append(Reason)
            Route = Controller.MotionRoute
            for Name, Servo in Controller.servos.items():
                Servo.present_raw = Points[0][Name]
            Route["Index"] = 1
            Controller.BeginRouteStage()
            # Approached like a flown waypoint: braking room is its reach plus the allowance ...
            Self.assertEqual(Controller.RouteStopRoom(1, "elbow_flex"), (Reach + Allowance - Bridge.ROUTE_BRAKE_GUARD_RAW, Bridge.ROUTE_BRAKE_OVERSHOOT))
            # ... and the monitor allows an overshoot into it.
            Elbow = Controller.servos["elbow_flex"]
            for Offset, Trip in ((Reach + Allowance, False), (Reach + Allowance + 1, True)):
                Elbow.present_raw = Points[1]["elbow_flex"] + Offset
                Controller.CheckMotionRoute()
                Self.assertEqual(bool(Trips), Trip, Offset)
                Trips.clear()
            # But the next stage starts only once every joint has arrived: the pan, which holds still next, is 20 past.
            Elbow.present_raw = Points[1]["elbow_flex"]
            Controller.servos["shoulder_pan"].present_raw = Points[1]["shoulder_pan"] + 20
            Self.assertFalse(Controller.RouteWaypointPassed())
            Self.assertIn("shoulder_pan", Controller.RouteSpeeds())
            Controller.servos["shoulder_pan"].present_raw = Points[1]["shoulder_pan"] + Controller.RouteTightToleranceRaw()
            Self.assertTrue(Controller.RouteWaypointPassed())
            # The stage after it has no waypoint reach at its start: the holding pan keeps the plain allowance.
            Controller.servos["shoulder_pan"].present_raw = Points[1]["shoulder_pan"]
            Route["Index"] = 2
            Controller.BeginRouteStage()
            Self.assertEqual(Route["Displaced"], {})
            Pan = Controller.servos["shoulder_pan"]
            for Offset, Trip in ((Allowance, False), (-Allowance, False), (Allowance + 1, True), (-Allowance - 1, True)):
                Pan.present_raw = Points[1]["shoulder_pan"] + Offset
                Controller.CheckMotionRoute()
                Self.assertEqual(bool(Trips), Trip, Offset)
                Trips.clear()

    def TestWideReachOnlyForJointsCarryingOnWithinTheNextStage(Self):
        """The lift carries on through waypoint 0 and gets half its 132-step travel; the pan's half (19) is less than
        the plain reach, so it keeps that. Capped by the next stage's travel, and nothing for a joint turning back."""
        Controller, Bus = MakeController()
        Points = Self.StagedPanAndLift(Controller)
        Start = {Name: Servo.present_raw for Name, Servo in Controller.servos.items()}
        Plain = Collision.WaypointReachRaw
        Wide = Controller.WideWaypointReach(Start, Points[0], Points[1])
        Self.assertEqual(Wide, {**dict.fromkeys(Bridge.MOTOR_NAMES, Plain), "shoulder_lift": 66})
        Short = {**Points[1], "shoulder_lift": Controller.RawAlongTrack("shoulder_lift", Points[0]["shoulder_lift"],
                                                                         Points[1]["shoulder_lift"], 40 / 132)}
        Self.assertEqual(Controller.WideWaypointReach(Start, Points[0], Short)["shoulder_lift"], 40)
        Back = {**Points[1], "shoulder_lift": Start["shoulder_lift"]}
        Self.assertIsNone(Controller.WideWaypointReach(Start, Points[0], Back))
        Gripper = Controller.servos["gripper"]
        Opening = {Name: {**Point, "gripper": Controller.RawAlongTrack("gripper", Gripper.present_raw, Gripper.range_max, Step / 4)}
                   for Step, (Name, Point) in enumerate((("Point", Points[0]), ("After", Points[1])), start=1)}
        Self.assertEqual(Controller.WideWaypointReach(Start, Opening["Point"], Opening["After"])["gripper"], Plain)

    def TestWideReachIsAssignedWhereItCertifiesAndOnlyAtHighSpeed(Self):
        Controller, Bus = MakeController()
        Points = Self.StagedPanAndLift(Controller)
        Tried = []

        def Validate(Check, Targets, Calibration=False, Relief=None, Reach=({}, 0), Budget=256):
            Index = next(Index for Index, Point in enumerate(Points) if Point == Targets)
            Wide = isinstance(Reach[1], dict) and max(Reach[1].values(), default=0) > Collision.WaypointReachRaw
            Tried.append((Index, Wide))
            if Index == 1 and Wide:
                raise ValueError("wide reach blocked")
        Controller.SetMotionSpeed(100)
        with Patch.object(Bridge.BridgeController, "ValidateMotionPath", Validate):
            Assigned = Controller.AssignRouteReach(Collision.RoutePath(Points))
        # Waypoint 0 certified the wider reach; waypoint 1 did not and kept the plain one, flown; 2 is the end.
        Self.assertEqual(set(Assigned.Reach), {0})
        Self.assertEqual(Assigned.Reach[0]["shoulder_lift"], 66)
        Self.assertEqual((Assigned.Tight, Assigned.Arrive), (set(), set()))
        Self.assertEqual(Tried, [(0, True), (1, True), (1, False), (2, False)])
        # Below half speed it is not worth its planning time.
        Controller.SetMotionSpeed(Bridge.ROUTE_WIDE_REACH_MINIMUM_PERCENT - 1)
        Tried.clear()
        with Patch.object(Bridge.BridgeController, "ValidateMotionPath", Validate):
            Self.assertEqual(Controller.AssignRouteReach(Collision.RoutePath(Points)).Reach, {})
        Self.assertFalse(any(Wide for _, Wide in Tried))
        # A route stops trying it after its misses.
        Controller.SetMotionSpeed(100)
        Start = {Name: Servo.present_raw for Name, Servo in Controller.servos.items()}
        Longer = [{**Start, "shoulder_pan": Self.Along(Controller, "shoulder_pan", 100 * Step)} for Step in range(1, 9)]
        Tried.clear()

        def Blocked(Check, Targets, Calibration=False, Relief=None, Reach=({}, 0), Budget=256):
            Wide = isinstance(Reach[1], dict) and max(Reach[1].values(), default=0) > Collision.WaypointReachRaw
            Tried.append(Wide)
            if Wide:
                raise ValueError("wide reach blocked")
        with Patch.object(Bridge.BridgeController, "ValidateMotionPath", Blocked):
            Assigned = Controller.AssignRouteReach(Collision.RoutePath(Longer))
        Self.assertEqual(sum(Tried), Bridge.ROUTE_WIDE_REACH_MISSES)
        Self.assertEqual((Assigned.Reach, Assigned.Tight), ({}, set()))

    def TestWideReachIsMonitoredBrakedForAndFitsTheNextStage(Self):
        Controller, Bus = MakeController()
        Controller.SetMotionSpeed(100)
        Points = Self.StagedPanAndLift(Controller)
        Allowance, Plain, Guard = Collision.Model["JointTrackingAllowanceRaw"], Collision.WaypointReachRaw, Bridge.ROUTE_BRAKE_GUARD_RAW
        Path = Collision.RoutePath(Points, Reach={0: {**dict.fromkeys(Bridge.MOTOR_NAMES, Plain), "shoulder_lift": 66}})
        Past = lambda Steps: Controller.RawAlongTrack("shoulder_lift", Points[0]["shoulder_lift"], Points[1]["shoulder_lift"], Steps / 132)
        Trips = []
        with Controller.lock:
            Controller.StartMotionRoute(Path)
            Controller.StopMotion = lambda Reason="", Latch=False: Trips.append(Reason)
            # The lift brakes for its wide reach, the pan for the plain one.
            Self.assertEqual(Controller.RouteStopRoom(0, "shoulder_lift"), (66 + Allowance - Guard, Bridge.ROUTE_BRAKE_OVERSHOOT))
            Self.assertEqual(Controller.RouteStopRoom(0, "shoulder_pan"), (Plain + Allowance - Guard, Bridge.ROUTE_BRAKE_OVERSHOOT))
            # The monitor accepts the lift that far past waypoint 0, and not a step further.
            Lift = Controller.servos["shoulder_lift"]
            for Steps, Trip in ((66 + Allowance, False), (66 + Allowance + 1, True)):
                Lift.present_raw = Past(Steps)
                Controller.CheckMotionRoute()
                Self.assertEqual(bool(Trips), Trip, Steps)
                Trips.clear()
            # Where the lift may overshoot to on stage 0 lies inside stage 1 when it starts there.
            for Name, Servo in Controller.servos.items():
                Servo.present_raw = Points[0][Name]
            Lift.present_raw = Past(66 + Allowance)
            Self.assertTrue(Controller.RouteWaypointPassed())
            Controller.MotionRoute["Index"] = 1
            Controller.BeginRouteStage()
            Self.assertFalse(Trips)
            Self.assertIsNotNone(Controller.MotionRoute)

    def TestReducedReachBeforeTightAndCarriedIntoTheNextStage(Self):
        """A waypoint whose plain reach does not certify tries ROUTE_REDUCED_REACH_RAW before becoming tight; the
        joints settling from it bring that much to the next stage's start, are monitored with it, and the waypoint
        is approached allowing a lurch, as a tight one is, but flown past."""
        Controller, Bus = MakeController()
        Points = Self.StagedPanAndLift(Controller)
        Reduced, Allowance = Bridge.ROUTE_REDUCED_REACH_RAW, Collision.Model["JointTrackingAllowanceRaw"]
        Seen = []

        def Validate(Check, Targets, Calibration=False, Relief=None, Reach=({}, 0), Budget=256):
            Index = next(Index for Index, Point in enumerate(Points) if Point == Targets)
            Goal = max(Reach[1].values(), default=0) if isinstance(Reach[1], dict) else Reach[1]
            Seen.append((Index, max(Reach[0].values(), default=0), Goal))
            if Index == 0 and Goal > Reduced:
                raise ValueError("only a small reach fits")
        with Patch.object(Bridge.BridgeController, "ValidateMotionPath", Validate):
            Assigned = Controller.AssignRouteReach(Collision.RoutePath(Points))
        Self.assertEqual((Assigned.Tight, Assigned.Arrive), (set(), set()))
        Self.assertEqual(Assigned.Reach, {0: dict.fromkeys(Bridge.MOTOR_NAMES, Reduced)})
        Self.assertIn((1, Reduced, Collision.WaypointReachRaw), Seen)  # Stage 1 started with the reduced reach.
        Trips = []
        with Controller.lock:
            Controller.StartMotionRoute(Assigned)
            Controller.StopMotion = lambda Reason="", Latch=False: Trips.append(Reason)
            Self.assertEqual(Controller.RouteStopRoom(0, "shoulder_lift"),
                             (Reduced + Allowance - Bridge.ROUTE_BRAKE_GUARD_RAW, Bridge.ROUTE_TIGHT_BRAKE_OVERSHOOT))
            for Name, Servo in Controller.servos.items():
                Servo.present_raw = Points[0][Name]
            Controller.MotionRoute["Index"] = 1
            Controller.BeginRouteStage()
            Self.assertEqual(Controller.MotionRoute["Displaced"], {"shoulder_pan": Reduced, "shoulder_lift": Reduced})
            Lift = Controller.servos["shoulder_lift"]
            Track = lambda Raw: Controller._slider_track_pos(Lift, Raw)
            Ahead = 1 if Track(Points[1]["shoulder_lift"]) > Track(Points[0]["shoulder_lift"]) else -1

            def Behind(Steps):
                Position = round(Track(Points[0]["shoulder_lift"]) - Ahead * Steps)
                return (Lift.range_min - Position if Lift.arc_inverted else Lift.range_min + Position) % Bridge.RESOLUTION
            for Steps, Trip in ((Reduced + Allowance, False), (Reduced + Allowance + 1, True)):
                Lift.present_raw = Behind(Steps)
                Controller.CheckMotionRoute()
                Self.assertEqual(bool(Trips), Trip, Steps)
                Trips.clear()

    def TestDescentToTheCanFliesReducedReachInsideItsCertificates(Self):
        """The 2026-10-03 14:50:27 descent to the can beside the basket rim: three of its waypoints were tight, one is
        now; every position the monitor accepts lies inside a certified box, and it is flown without a trip."""
        Controller, Bus = Self.PlaceRaw(100, AboveCanRaw)
        Target = Self.DisplayTarget(Controller, DescentDisplay, DescentPitch)
        MonitorInsideCertificates(Self, Controller, lambda: Controller.send_ik(Target=Target))
        Self.assertLessEqual(len(Controller.MotionRoute["Tight"]), 1)
        Self.assertTrue(any(min(Reach.values()) == Bridge.ROUTE_REDUCED_REACH_RAW for Reach in Controller.MotionRoute["Reach"].values()))
        Controller, Bus = Self.PlaceRaw(100, AboveCanRaw)
        Ok, Message = Controller.send_ik(Target=Target)
        Self.assertTrue(Ok, Message)
        with Patch.dict(JointCaps, SpecCaps):
            Execute(Self, Controller, Bus)
        Tip = Cad.Positions(Controller.arm_geometry, Controller._current_ik_angles())[-1]
        Self.assertLess(Math.dist(tuple(Tip.values()), tuple(Target[Axis] for Axis in ("x", "y", "z"))), .8)

    def TestHoverFromHomeAtFullSpeedUsesWideReachInsideItsCertificates(Self):
        """At 100% the home-to-hover route gets the wider reach at some waypoints, and every position the route monitor
        accepts on every stage lies inside a box certified for that stage."""
        Controller, Bus = Self.PlaceRaw(100, HomeOpenRaw)
        MonitorInsideCertificates(Self, Controller,
            lambda: Controller.send_ik(Target=Self.DisplayTarget(Controller, HoverDisplay, HoverPitch)))
        Self.assertTrue(Controller.MotionRoute["Reach"])

    def TestLiftOffTheCanFliesItsArriveWaypointsInsideTheirCertificates(Self):
        """The 2026-10-03 14:52 lift off the can, beside the basket rim: at most one tight waypoint (it had four), every
        position the monitor accepts on every stage inside a box certified for that stage, and flown without a trip.
        Its arrive waypoints are the v7 contact model's (V7ContactAllowance); under v8 (CAD-exact contact) the
        stages certify with their reach and the route needs none."""
        with V7ContactAllowance():
            Self.LiftOffTheCan(True)
        Self.LiftOffTheCan(False)

    def LiftOffTheCan(Self, Arrives):
        Controller, Bus = Self.PlaceRaw(100, AtCanRaw)
        Target = Self.DisplayTarget(Controller, LiftDisplay, LiftPitch)
        MonitorInsideCertificates(Self, Controller, lambda: Controller.send_ik(Target=Target))
        Controller, Bus = Self.PlaceRaw(100, AtCanRaw)
        Ok, Message = Controller.send_ik(Target=Target)
        Self.assertTrue(Ok, Message)
        Route = Controller.MotionRoute
        Self.assertLessEqual(len(Route["Tight"]), 1)
        Self.assertEqual(bool(Route["Arrive"]), Arrives)
        with Patch.dict(JointCaps, SpecCaps):
            Execute(Self, Controller, Bus)
        Tip = Cad.Positions(Controller.arm_geometry, Controller._current_ik_angles())[-1]
        Self.assertLess(Math.dist(tuple(Tip.values()), tuple(Target[Axis] for Axis in ("x", "y", "z"))), .8)

    def TestJointsMoveTogetherThroughEveryWaypoint(Self):
        """The 2026-10-03 home-to-hover route: no moving joint stands at a waypoint waiting for the others, and the tip
        flies the planned line.

        Each joint used to brake toward its own waypoint on its own, so the pan and the wrist (small travel per
        stage) got there early and stood still until the lift arrived, and at every stage start the leash then held
        the lift back for the pan finishing the last waypoint. On the servo model the tip flew 1.6 times the
        straight distance on a route planned at 1.08.
        """
        Tip = lambda Angles: Np.array(list(Cad.Positions(Controller.arm_geometry, Angles)[-1].values()))
        for Percent in (18, 100):
            with Self.subTest(Percent=Percent), Patch.dict(JointCaps, SpecCaps):
                Controller, Bus = Self.SendHoverFromHome(Percent)
                Route = Controller.MotionRoute
                Self.assertGreater(len(Route["Path"]), 5)
                Points = [Route["Start"], *Route["Path"]]
                Angles = [{Name: Controller._ik_angle_from_raw(Controller.servos[Name], Point[Name]) for Name in Bridge.MOTOR_NAMES}
                          for Point in Points]
                Planned = sum(float(Np.linalg.norm(Tip({Name: A[Name] + (B[Name] - A[Name]) * (Step + 1) / 16 for Name in A})
                                                   - Tip({Name: A[Name] + (B[Name] - A[Name]) * Step / 16 for Name in A})))
                              for A, B in zip(Angles, Angles[1:]) for Step in range(16))
                Ceilings = Controller.MotionSpeedCeilings(Percent)
                Track = lambda Name, Raw: Controller._slider_track_pos(Controller.servos[Name], Raw)
                Flown, Waiting, Frames = [Tip(Controller._current_ik_angles())], [], [0]

                def Watch(_):
                    Flown.append(Tip(Controller._current_ik_angles()))
                    for Name, Servo in Controller.servos.items():
                        Self.assertLessEqual(abs(Servo.command_speed), Ceilings[Name])
                    Route = Controller.MotionRoute
                    if Route is None or Controller.RouteStageSettles():
                        return
                    Frames[0] += 1
                    Point, Origin = Route["Path"][Route["Index"]], Route["Start"]
                    Spans = {Name: Track(Name, Point[Name]) - Track(Name, Origin[Name]) for Name in Bridge.MOTOR_NAMES}
                    Left = {Name: Track(Name, Point[Name]) - Track(Name, Controller.servos[Name].present_raw) for Name in Spans}
                    Moving = [Name for Name, Span in Spans.items() if abs(Span) >= Bridge.ROUTE_LEASH_MINIMUM_SPAN_RAW]
                    if any(abs(Left[Name]) > Bridge.ROUTE_FLY_BY_TOLERANCE_RAW for Name in Moving):
                        Waiting.extend(Name for Name in Moving
                                       if Left[Name] * Spans[Name] > 0 and Controller.servos[Name].command_speed == 0)

                Execute(Self, Controller, Bus, Each=Watch)
                Self.assertLess(len(Waiting), 0.05 * Frames[0],
                                f"{len(Waiting)} joint-frames of {Frames[0]} stood waiting at a waypoint: {sorted(set(Waiting))}")
                Length = sum(float(Np.linalg.norm(B - A)) for A, B in zip(Flown, Flown[1:]))
                Self.assertLess(Length, 1.05 * Planned, (Length, Planned))

    @staticmethod
    def Along(Controller, Name, Steps):
        """The raw position Steps along a joint's track from where it is, toward the middle of its range."""
        Servo = Controller.servos[Name]
        Position = Controller._slider_track_pos(Servo, Servo.present_raw)
        Track = round(Position + (1 if Position < Controller._arc_span(Servo) / 2 else -1) * Steps)
        return (Servo.range_min - Track if Servo.arc_inverted else Servo.range_min + Track) % Bridge.RESOLUTION

    def StagedPanAndLift(Self, Controller):
        """Three stages of the pan and the lift each moving one way along its track (toward the middle of its
        range), the lift 132 steps a stage and the pan 38."""
        Start = {Name: Servo.present_raw for Name, Servo in Controller.servos.items()}
        return [{**Start, "shoulder_pan": Self.Along(Controller, "shoulder_pan", 38 * Step),
                 "shoulder_lift": Self.Along(Controller, "shoulder_lift", 132 * Step)} for Step in (1, 2, 3)]

    def TestMovingJointsAreTimedToArriveTogether(Self):
        """At 100% the braking bound, not the ceiling, sets the lead joint's speed; the others follow it, not their own."""
        Controller, Bus = MakeController()
        Controller.SetMotionSpeed(100)
        Points = Self.StagedPanAndLift(Controller)
        with Controller.lock:
            Controller.StartMotionRoute(Collision.RoutePath(Points))
            Speeds = Controller.RouteSpeeds()
            Room = Collision.WaypointReachRaw + Collision.Model["JointTrackingAllowanceRaw"] - Bridge.ROUTE_BRAKE_GUARD_RAW
            Lift, Pan = abs(Speeds["shoulder_lift"]), abs(Speeds["shoulder_pan"])
            # The lift leads at the speed it can still stop from inside its checked reach.
            Self.assertLessEqual(Lift, Controller.RouteBrakingSpeed(132 + Room))
            Self.assertGreater(Lift, Controller.RouteBrakingSpeed(132 + Room) - 2)
            # The pan is timed to the lift (it used to run at its own braking bound, 0.57 of the lift's speed).
            Self.assertAlmostEqual(Pan / Lift, 38 / 132, delta=0.03)
        Self.assertEqual(Controller.state_snapshot()["route_executor_version"], "synchronized-v2")
        Self.assertEqual(Bridge.health()["route_executor_version"], "synchronized-v2")

    def TestStageStartDoesNotHoldBackTheLeadForAJointFinishingTheLastWaypoint(Self):
        """A stage starts once every joint is within the fly-by tolerance of the waypoint. The pan, 17 steps short of
        it with only 38 steps to go per stage, measured -0.45 of the new stage, and the leash stopped the lift there."""
        Controller, Bus = MakeController()
        Controller.SetMotionSpeed(100)
        Points = Self.StagedPanAndLift(Controller)
        with Controller.lock:
            Controller.StartMotionRoute(Collision.RoutePath(Points))
            Route = Controller.MotionRoute
            Start = dict(Route["Start"])
            for Name, Servo in Controller.servos.items():
                Servo.present_raw = Points[0][Name]
            Controller.servos["shoulder_pan"].present_raw = Controller.RawAlongTrack("shoulder_pan", Start["shoulder_pan"],
                                                                                     Points[0]["shoulder_pan"], 1 - 17 / 38)
            Route["Index"] = 1
            Controller.BeginRouteStage()
            Self.assertIsNotNone(Controller.MotionRoute)
            Speeds = Controller.RouteSpeeds()
            Self.assertGreater(abs(Speeds.get("shoulder_lift", 0)), 100)
            Self.assertGreater(abs(Speeds.get("shoulder_pan", 0)), 0)

    def TestJointCarryingOnKeepsPaceThroughTheFlyByTolerance(Self):
        """A joint that keeps going the same way after the waypoint does not stop inside the fly-by tolerance while
        the lead joint is still on its way; one exactly on its waypoint with nothing left to time against stops."""
        Controller, Bus = MakeController()
        Controller.SetMotionSpeed(100)
        Points = Self.StagedPanAndLift(Controller)
        Start = {Name: Servo.present_raw for Name, Servo in Controller.servos.items()}
        Short = lambda Name, Steps: Controller.RawAlongTrack(Name, Start[Name], Points[0][Name], 1 - Steps / abs(
            Controller._slider_track_pos(Controller.servos[Name], Points[0][Name]) - Controller._slider_track_pos(Controller.servos[Name], Start[Name])))
        with Controller.lock:
            Controller.StartMotionRoute(Collision.RoutePath(Points))
            Controller.servos["shoulder_pan"].present_raw = Short("shoulder_pan", 20)
            Controller.servos["shoulder_lift"].present_raw = Short("shoulder_lift", 80)
            Speeds = Controller.RouteSpeeds()
            Self.assertGreater(abs(Speeds.get("shoulder_pan", 0)), 0)
            Self.assertAlmostEqual(abs(Speeds["shoulder_pan"]) / abs(Speeds["shoulder_lift"]), 20 / 80, delta=0.05)
            Controller.servos["shoulder_pan"].present_raw = Points[0]["shoulder_pan"]
            Controller.servos["shoulder_lift"].present_raw = Short("shoulder_lift", 10)
            Self.assertNotIn("shoulder_pan", Controller.RouteSpeeds())
            # Every joint exactly on the waypoint: nothing to time against, and nothing moves.
            Controller.servos["shoulder_lift"].present_raw = Points[0]["shoulder_lift"]
            Self.assertEqual(Controller.RouteSpeeds(), {})

    def TestRouteEndAcceptsWhatTheStageBeforeLetAJointOvershoot(Self):
        """2026-10-04 23:50:01 (bridge_events.jsonl), the hover beside the pill bottle opening the jaws on the way:
        the stage into the last waypoint accepted the pan up to its 32-step reach plus the allowance past it, but
        the pan had only 17 steps left to the route end, and the last stage accepted 16 past that. The pan stood 36
        past the waypoint (19 past the end) when the last stage began, and the monitor coasted the arm over the
        desk. The end now takes, for each joint carrying on, the reach the last stage's travel does not cover,
        certified with it, and accepts there what the stage before did; braking at the end is unchanged.

        The reach this route gets depended on ROUTE_REACH_ASSIGN_SECONDS (1.2 s of wall clock): on a slower or busier
        PC it ran out, the wider reaches went untried and the route differed (it passed 1 run in 5, audit L5; KeyError
        11 below). setUpModule gives the reach pass all the time it needs."""
        Self.assertEqual(Bridge.ROUTE_REACH_ASSIGN_SECONDS, 60.0)
        Controller, Bus = Self.PlaceRaw(100, Home2348Raw)
        Target = Self.DisplayTarget(Controller, Hover2349Display, Hover2349Pitch)
        MonitorInsideCertificates(Self, Controller, lambda: Controller.send_ik(Target=Target, GripperRaw=Open2349))
        Controller, Bus = Self.PlaceRaw(100, Home2348Raw)
        Ok, Message = Controller.send_ik(Target=Target, GripperRaw=Open2349)
        Self.assertTrue(Ok, Message)
        Route = Controller.MotionRoute
        Path, Last = Route["Path"], len(Route["Path"]) - 1
        # The logged last stage: pan 3037 -> 3020, lift 790 -> 856 (791 logged), elbow and wrist on.
        Self.assertEqual((Path[Last - 1]["shoulder_pan"], Path[Last]["shoulder_pan"]), (3037, 3020))
        Self.assertEqual(Route["Reach"][Last - 1]["shoulder_pan"], Collision.WaypointReachRaw)
        Self.assertEqual(Route["Reach"][Last], {"shoulder_pan": 32 - 17, "wrist_flex": 32 - 20})
        Self.assertEqual(Controller.RouteStopRoom(Last, "shoulder_pan"),
                         (Collision.Model["JointTrackingAllowanceRaw"] - Bridge.ROUTE_BRAKE_GUARD_RAW, Bridge.ROUTE_BRAKE_OVERSHOOT))
        Trips = []
        Controller.StopMotion = lambda Reason="", Latch=False: Trips.append(Reason)
        Pan = Controller.servos["shoulder_pan"]

        def Switch(Reach, PanRaw):
            Route["Reach"] = Reach
            Route["Index"], Route["Start"], Route["Displaced"] = Last - 1, dict(Path[Last - 2]), {}
            for Name, Servo in Controller.servos.items():
                Servo.present_raw = {**Fault2350Raw, "shoulder_pan": PanRaw}[Name]
            Controller.MotionRoute = Route
            with Controller.lock:
                Controller.CheckMotionRoute()
                Self.assertFalse(Trips, "the stage before accepts it")
                Route["Index"] = Last
                Controller.BeginRouteStage()
            Found = list(Trips)
            Trips.clear()
            return Found
        Reach = dict(Route["Reach"])
        Self.assertEqual(Switch(Reach, Fault2350Raw["shoulder_pan"]), [])
        # As far past the end as the stage before accepted it (its reach and the allowance), and no further.
        Furthest = Path[Last - 1]["shoulder_pan"] - Collision.WaypointReachRaw - Collision.Model["JointTrackingAllowanceRaw"]
        Self.assertEqual(Switch(Reach, Furthest), [])
        Pan.present_raw = Furthest - 1
        with Controller.lock:
            Controller.CheckMotionRoute()
        Self.assertTrue(Trips)
        Trips.clear()
        # Without the end reach (the rule before), the logged pose leaves the checked route at the switch.
        Old = {Index: Value for Index, Value in Reach.items() if Index != Last}
        Self.assertEqual(Switch(Old, Fault2350Raw["shoulder_pan"]), ["shoulder_pan left the checked basket route"])

    def TestEndReachOnlyForJointsCarryingOnWithLessTravelThanTheirReach(Self):
        Controller, Bus = MakeController()
        Start = {Name: Servo.present_raw for Name, Servo in Controller.servos.items()}
        Plain = dict.fromkeys(Bridge.MOTOR_NAMES, Collision.WaypointReachRaw)
        Waypoint = {**Start, "shoulder_pan": Self.Along(Controller, "shoulder_pan", 60),
                    "elbow_flex": Self.Along(Controller, "elbow_flex", 60), "wrist_flex": Self.Along(Controller, "wrist_flex", 60)}
        End = {**Waypoint, "shoulder_pan": Self.Along(Controller, "shoulder_pan", 70),    # carries on 10
               "elbow_flex": Self.Along(Controller, "elbow_flex", 100),                  # carries on 40: its travel covers it
               "wrist_flex": Self.Along(Controller, "wrist_flex", 50),                   # turns back
               "shoulder_lift": Self.Along(Controller, "shoulder_lift", 10)}             # starts moving here
        Self.assertEqual(Controller.EndReach(Start, Waypoint, End, Plain), {"shoulder_pan": 22})
        Self.assertEqual(Controller.EndReach(Start, Waypoint, End, dict.fromkeys(Bridge.MOTOR_NAMES, Bridge.ROUTE_REDUCED_REACH_RAW)),
                         {"shoulder_pan": 6})
        Self.assertEqual(Controller.EndReach(Start, Waypoint, End, {}), {})  # after a tight waypoint
        # Assigned only where it certifies: otherwise the end keeps the plain allowance, as before.
        Points = [Waypoint, End]
        Asked = []

        def Validate(Check, Targets, Calibration=False, Relief=None, Reach=({}, 0), Budget=256):
            Index = Points.index(Targets)
            Asked.append((Index, dict(Reach[1]) if isinstance(Reach[1], dict) else Reach[1]))
            if Index == 1 and Reach[1]:
                raise ValueError("end reach blocked")
        Controller.SetMotionSpeed(100)
        with Patch.object(Bridge.BridgeController, "ValidateMotionPath", Validate):
            Assigned = Controller.AssignRouteReach(Collision.RoutePath(Points))
        Self.assertIn((1, {"shoulder_pan": 22}), Asked)
        Self.assertNotIn(1, Assigned.Reach)
        Self.assertEqual((Assigned.Tight, Assigned.Arrive), (set(), set()))
        with Patch.object(Bridge.BridgeController, "ValidateMotionPath", lambda *Arguments, **Options: None):
            Self.assertEqual(Controller.AssignRouteReach(Collision.RoutePath(Points)).Reach[1], {"shoulder_pan": 22})


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestFlyByRoutes` runs the Test* methods too (unittest names this hook)."""
    Prefixed = Unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return Prefixed.loadTestsFromTestCase(FlyByRouteTests)


if __name__ == "__main__":
    Loader = Unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Result = Unittest.TextTestRunner(verbosity=2).run(Loader.loadTestsFromTestCase(FlyByRouteTests))
    raise SystemExit(not Result.wasSuccessful())
