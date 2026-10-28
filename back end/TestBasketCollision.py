"""Offline basket geometry, routing and fake-motor execution regressions."""
import json as Json
import threading as Threading
import unittest as Unittest
from pathlib import Path
from unittest.mock import patch as Patch

import numpy as Np
import BasketCollision as Collision
import bridge_server as Bridge
from TestSafety import MakeController

Root = Path(__file__).resolve().parent
# Both poses and their direct sweep clear the mast. Coarse isotropic padding
# previously reported a false mast contact along this motion.
StartValues = (26.932136638455745, 50.83968781933507, -72.48368119971147, 10.73972938871168, 94.39453125, 0)
GoalValues = (-4.72691556738674, 78.25024333298705, -20.060610250754664, 22.460831466544704, 94.39453125, 0)
# Forward and down past the cart front, below the rim and clear of the rearward limit.
LowValues = (0.34323065528329266, -87.45806454570632, 79.94271769861842, -27.1169118284174, 94.39453125, 0)
# The top tray's front rim as the head depth camera measures it (display frame,
# six recorded frames of 2026-09-29): ridge Y 5th/50th/95th percentiles, cm.
# The CAD put it about 2.4 cm further forward, over the gap before the desk.
MeasuredRimYCm = (22.1, 22.7, 23.3)
TopTray = 2  # BasketSurface component: the top tray, rim at display Z 0, floor at Z -9.


def Limits(Controller):
    return {Name: sorted((Controller._ik_angle_from_raw(S, S.range_min), Controller._ik_angle_from_raw(S, S.range_max)))
            for Name, S in Controller.servos.items()}


def Place(Controller, Bus, Angles):
    for Name, Angle in Angles.items():
        Servo = Controller.servos[Name]
        Raw = Controller._ik_raw_from_angle(Servo, Angle)
        Servo.present_raw = Servo.target_raw = Servo.HardwareRaw = Bus.position[Name] = Raw
        Servo.command_raw = None
    Bus.configure_motors()
    Bus.writes.clear()


def DisplayToModel(X, Y, Z):
    """The AI's display frame: X mirrored about the arm base; the CAD pan axis is the model origin."""
    Base = Json.loads((Root / "WorkspaceFrame.json").read_text())["RobotFrame"]["ArmBasePositionCm"]
    return {"x": -X - Base["x"], "y": Y - Base["y"], "z": Z - Base["z"]}


def TopTrayDisplay(World):
    """Top-tray triangles in the display frame (X mirrored)."""
    Base = Json.loads((Root / "WorkspaceFrame.json").read_text())["RobotFrame"]["ArmBasePositionCm"]
    Triangles = World.Surface.Triangles[World.Surface.Components == TopTray] + [Base["x"], Base["y"], Base["z"]]
    Triangles[..., 0] *= -1
    return Triangles


def LowController():
    Controller, Bus = MakeController()
    Place(Controller, Bus, dict(zip(Collision.Names, LowValues)))
    Angles = Controller._current_ik_angles()
    Controller.elbow_mode = Collision.Cad.Branch(Controller.arm_geometry, Angles)
    return Controller, Bus, Angles


class BasketTests(Unittest.TestCase):
    def TestReachableLowCartesianMotionExecutesOutsideBasket(Self):
        Controller, Bus, Angles = LowController()
        Angles["shoulder_pan"] += 3
        Target = Collision.Cad.Positions(Controller.arm_geometry, Angles)[-1]
        Self.assertLess(Controller.VisualPointFromModelPoint(Target)["z"], 0)
        Requested = {**Target, "wrist_pitch": Collision.Cad.Pitch(Controller.arm_geometry, Angles)}
        Self.assertTrue(Controller.set_target(Requested)[0])
        Self.assertFalse(Bus.writes, "Staging a low target must not move motors")
        Ok, Message = Controller.send_ik()
        Self.assertTrue(Ok, Message)
        World = Controller.CollisionWorld()
        for _ in range(200):
            for Name in Collision.Names: Bus.step(Name, steps_per_tick=4)
            Controller.refresh()
            Self.assertIsNone(Controller.MotionFault)
            Self.assertIsNone(World.PoseReason(Controller._current_ik_angles()))
            if Controller.MotionRoute is None: break
        Self.assertIsNone(Controller.MotionRoute)
        Self.assertLess(Controller.VisualPointFromModelPoint(Collision.Cad.Positions(Controller.arm_geometry, Controller._current_ik_angles())[-1])["z"], 0)
        Self.assertIsNone(Controller.state_snapshot()["minimum_tip_height_cm"])

    def TestLowManualAndHomeMotionsKeepCollisionGuards(Self):
        Controller, Bus, _ = LowController()
        Goals = {Name: Servo.present_raw for Name, Servo in Controller.servos.items()}
        Goals["shoulder_pan"] -= 34
        Ok, Message = Controller.move_servo("shoulder_pan", Goals["shoulder_pan"])
        Self.assertTrue(Ok, Message)
        Controller.StopMotion()
        with Patch.object(Controller, "_load_home", return_value=Goals):
            Ok, Message = Controller.go_home()
        Self.assertTrue(Ok, Message)
        Self.assertTrue(Bus.torque_enabled)

    def TestLegacyOrMissingHeightSettingDoesNotRestrictRoutes(Self):
        Controller, _, _ = LowController()
        Frame = Bridge._load_json(Root / "WorkspaceFrame.json")
        for Value in (None, 999):
            if Value is None: Frame.pop("MinimumTipZCm", None)
            else: Frame["MinimumTipZCm"] = Value
            with Patch.object(Bridge.WorkspaceFrameFile, "Data", return_value=Frame):
                Controller.ValidateMotionPath({"shoulder_pan": Controller.servos["shoulder_pan"].present_raw - 34})

    def TestBoundsMatchBothAssemblyMounts(Self):
        Frame = Json.loads((Root / "WorkspaceFrame.json").read_text())
        for Side in ("Left", "Right"):
            Frame["RobotFrame"]["ControlledArm"] = Side
            Bounds = Collision.BasketBounds(Frame)
            Actual = Bounds + Np.array(Collision.Model["Mounts"][Side]["PanCm"])
            Np.testing.assert_allclose(Actual, Collision.Model["BasketAssemblyBoundsCm"])

    def TestTargetOnBasketMaterialIsRejectedWithoutMotorWrites(Self):
        Controller, Bus = MakeController()
        World = Controller.CollisionWorld()
        # Actual CAD face centers, not faces of the old solid-cart envelope.
        Points = [Triangle.mean(axis=0) for Triangle in World.Surface.Triangles[::1500]]
        Previous = dict(Controller.target_cm)
        for Point in Points:
            Target = dict(zip(("x", "y", "z"), Point))
            Self.assertIsNotNone(World.TargetReason(Target))
            Self.assertFalse(Controller.set_target(Target)[0])
            Self.assertFalse(Controller.send_ik(Target=Target)[0])
        Self.assertEqual(Controller.target_cm, Previous)
        Self.assertFalse(Bus.writes)

    def TestWholeCollarRotationAndHomeAreClear(Self):
        Controller, _ = MakeController()
        World = Controller.CollisionWorld()
        Self.assertIsNone(World.MountReason)
        Self.assertIsNone(World.PoseReason(Controller._current_ik_angles()))

    def TestTopTrayFrontRimIsWhereTheDepthCameraSeesIt(Self):
        Controller, _ = MakeController()
        World = Controller.CollisionWorld()
        Tray = TopTrayDisplay(World)
        # The outermost rim edge sits at the far (conservative) edge of the measured ridge.
        Self.assertAlmostEqual(float(Tray[..., 1].max()), MeasuredRimYCm[2], delta=0.01)
        for X in (-12, -6, 0, 6, 12):
            # Inside the measured front wall: basket material, not open tray.
            Self.assertIn("basket", World.TargetReason(DisplayToModel(X, MeasuredRimYCm[1], -4)) or "", X)
            # The depth camera sees down into the gap between rim and desk here.
            Self.assertIsNone(World.TargetReason(DisplayToModel(X, 24.9, -4)), X)
            Self.assertIsNone(World.TargetReason(DisplayToModel(X, 21.5, -4)), X)

    def TestOnlyTheTopTrayFrontMoved(Self):
        Controller, _ = MakeController()
        World = Controller.CollisionWorld()
        Tray = TopTrayDisplay(World).reshape(-1, 3)
        # Back wall, side walls, floor and rim height are the CAD's.
        Np.testing.assert_allclose(Tray.min(axis=0), [-20.6957, -6.7957, -9.0078], atol=2e-4)
        Np.testing.assert_allclose(Tray.max(axis=0)[[0, 2]], [20.7719, 0.0], atol=2e-4)
        # Still one closed, consistently wound solid: every edge is shared by
        # exactly two faces in opposite directions, so containment stays exact.
        Faces = Np.array(Collision.BasketSurface.Data["Faces"])[Np.array(Collision.BasketSurface.Data["Components"]) == TopTray]
        Edges = Np.concatenate([Faces[:, [0, 1]], Faces[:, [1, 2]], Faces[:, [2, 0]]])
        Self.assertEqual(len({tuple(Edge) for Edge in Edges.tolist()}), len(Edges))
        Self.assertEqual({tuple(Edge) for Edge in Edges.tolist()}, {tuple(Edge) for Edge in Edges[:, ::-1].tolist()})
        # The open tray and the floor under the pliers in the bin are unchanged.
        Self.assertIsNone(World.TargetReason(DisplayToModel(7.5, 8.8, -7.8)))
        Self.assertIn("basket", World.TargetReason(DisplayToModel(7.5, 8.8, -8.85)) or "")

    def TestDeskGraspBeyondTheRimIsClearAndTheNearOneStaysBlocked(Self):
        Controller, _ = MakeController()
        World = Controller.CollisionWorld()
        Current, Limits = Controller._current_ik_angles(), Controller._ik_joint_limits()

        def Poses(Point, Pitch):
            Target = DisplayToModel(*Point)
            Candidates = Collision.Cad.SolveCandidates(Controller.arm_geometry, Target["x"], Target["y"], Target["z"],
                elbow="up", joint_limits=Limits, initial_angles=Current, preferred_angles=Current, wrist_pitch_deg=Pitch)
            return [Candidate["joint_angles_degrees"] for Candidate in Candidates if Candidate["reachable"]]
        # 2026-09-29 07:58: the eraser 12 cm past the real rim was refused as
        # basket contact at wrist -45, the steepest pitch the arm reaches there.
        Reachable = Poses((5.18, 34.69, -1.8), -45)
        Self.assertTrue(Reachable)
        Self.assertTrue(any(World.PoseReason(Angles) is None for Angles in Reachable))
        # 07:57: 6.7 cm past the real rim, 40 cm out, the gripper meets the rim itself.
        Blocked = [Angles for Pitch in (-90, -75, -60, -45, -30, -15, 0) for Angles in Poses((14.47, 29.34, -2.49), Pitch)]
        Self.assertTrue(Blocked)
        for Angles in Blocked:
            Self.assertIn("basket", World.PoseReason(Angles) or "")

    def TestObbTouchAndRotatedSeparation(Self):
        Bounds = Np.array([[-1., -1, -1], [1., 1, 1]])
        Self.assertTrue(Collision.BoxesOverlap(Np.array([2., 0, 0]), Np.eye(3), Np.ones(3), Bounds))
        Self.assertFalse(Collision.BoxesOverlap(Np.array([2.01, 0, 0]), Np.eye(3), Np.ones(3), Bounds))
        Self.assertTrue(Collision.BoxesOverlap(Np.array([2.01, 0, 0]), Np.eye(3), Np.ones(3), Bounds, .02))

    def TestSafeEndpointsWithBlockedSweepGetADetour(Self):
        from TestRobotBodyCollision import Start, Goal
        Controller, _ = MakeController()
        World = Controller.CollisionWorld()
        StartVector, GoalVector = (Np.array([Pose[Name] for Name in Collision.Names]) for Pose in (Start, Goal))
        Self.assertIsNone(World.PoseReason(Start))
        Self.assertIsNone(World.PoseReason(Goal))
        Self.assertIn("robot body", World.EdgeReason(StartVector, GoalVector))
        Self.assertTrue(any("robot body" in (World.PoseReason(dict(zip(Collision.Names,
            StartVector + Fraction * (GoalVector - StartVector))), Tracking=False) or "")
            for Fraction in Np.linspace(0, 1, 41)), "Fixture must have actual intermediate contact")
        Path = World.Plan(Start, Goal, Limits(Controller))
        Self.assertGreater(len(Path), 1)
        Self.assertEqual(Path[-1], Goal)
        Random = Np.random.default_rng(32)
        Current = StartVector
        for Point in Path:
            Next = Np.array([Point[N] for N in Collision.Names])
            Self.assertIsNone(World.EdgeReason(Current, Next))
            # Independent progress plus encoder tracking error, not synchronized
            # interpolation. Test the certificate with many concrete arm poses.
            for _ in range(150):
                Sample = Current + Random.uniform(size=6) * (Next - Current)
                Sample += Random.uniform(-World.Slack, World.Slack, size=6)
                # Check concrete poses without double-counting tracking slack.
                Basket = Collision.CollisionWorld(Controller.arm_geometry, Json.loads((Root / "WorkspaceFrame.json").read_text()))
                Self.assertIsNone(Basket.PoseReason(dict(zip(Collision.Names, Sample)), Tracking=False))
            Current = Next

    def TestRepairFindsExtraWaypointWhenJointOrderingCannotWork(Self):
        class SyntheticWorld(Collision.CollisionWorld):
            def __init__(Self): pass
            def IntervalReason(Self, Low, High, Tracking=True):
                return "basket" if High[0] >= 0 and Low[0] <= 40 and High[1] >= -10 and Low[1] <= 10 else None
        Start = dict.fromkeys(Collision.Names, 0.0)
        Goal = dict(Start)
        Start[Collision.Names[0]], Goal[Collision.Names[0]] = -20, 60
        Ranges = dict.fromkeys(Collision.Names, (-40, 80))
        World = SyntheticWorld()
        Path = World.Plan(Start, Goal, Ranges, Seconds=2)
        Self.assertTrue(any(abs(Point[Collision.Names[1]]) > 10 for Point in Path))
        Previous = Np.array(list(Start.values()))
        for Point in Path:
            Values = Np.array(list(Point.values()))
            Self.assertIsNone(World.EdgeReason(Previous, Values))
            Previous = Values
        Self.assertEqual(Path[-1], Goal)

    def TestBlockedPoseAndNoRouteFailClosed(Self):
        Controller, Bus = MakeController()
        World = Controller.CollisionWorld()
        Current = Controller._current_ik_angles()
        with Patch.object(World, "IntervalReason", return_value="basket"):
            with Self.assertRaisesRegex(ValueError, "No motion sent"):
                World.Plan(Current, Current, Limits(Controller))
        with Patch.object(World, "EdgeReason", return_value="basket"):
            with Self.assertRaisesRegex(ValueError, "No clear route"):
                World.Plan(dict(zip(Collision.Names, StartValues)), dict(zip(Collision.Names, GoalValues)), Limits(Controller), Seconds=.02)
        Self.assertFalse(Bus.writes)

    def TestQuantizedDetourExecutesWithIndependentJointSpeeds(Self):
        Controller, Bus = MakeController()
        Place(Controller, Bus, dict(zip(Collision.Names, StartValues)))
        Goal = {Name: Controller._ik_raw_from_angle(Controller.servos[Name], Angle) for Name, Angle in zip(Collision.Names, GoalValues)}
        Ok, Message = Controller.SendJointMotion(Goal, "Test")
        Self.assertTrue(Ok, Message)
        Self.assertGreater(len(Controller.MotionRoute["Path"]), 1)
        # This synchronous fixture has no background feedback thread during
        # planning. Resume its simulated control clock before executing frames.
        Controller.last_refresh_s = Bridge.time.monotonic()
        for _ in range(2500):
            for Index, Name in enumerate(Collision.Names):
                Bus.step(Name, steps_per_tick=3 + Index)
            Controller.refresh()
            Self.assertIsNone(Controller.MotionFault)
            if Controller.MotionRoute is None:
                break
        Self.assertIsNone(Controller.MotionRoute)
        for Name, Raw in Goal.items():
            Self.assertLessEqual(Controller._circular_raw_distance(Controller.servos[Name].present_raw, Raw), 12)
        Self.assertTrue(Bus.torque_enabled)
        Self.assertFalse(any(Reg == "Torque_Enable" and Value == 0 for Reg, _, Value in Bus.writes))

    def StagedController(Self):
        Controller, Bus = MakeController()
        First = {Name: S.present_raw for Name, S in Controller.servos.items()}
        First["shoulder_pan"] += 100
        Second = {**First, "shoulder_pan": First["shoulder_pan"] + 100}
        Controller.ValidateMotionPath(Second)
        Controller.StartMotionRoute([First, Second])
        return Controller, Bus, First, Second

    def TestWaitsForFeedbackBeforeNextStage(Self):
        Controller, Bus, First, Second = Self.StagedController()
        for _ in range(3): Controller.refresh()
        Self.assertEqual(Controller.MotionRoute["Index"], 0)
        Self.assertEqual(Controller.servos["shoulder_pan"].target_raw, First["shoulder_pan"])
        Bus.position["shoulder_pan"] = First["shoulder_pan"]
        Controller.refresh()
        Self.assertEqual(Controller.MotionRoute["Index"], 1)
        Self.assertEqual(Controller.servos["shoulder_pan"].target_raw, Second["shoulder_pan"])

    def TestStopDiscardsEveryRemainingWaypoint(Self):
        Controller, Bus, _, _ = Self.StagedController()
        Controller.StopMotion()
        Count = len(Bus.writes)
        for _ in range(4): Controller.refresh()
        Self.assertIsNone(Controller.MotionRoute)
        Self.assertEqual(len(Bus.writes), Count)
        Self.assertTrue(Bus.torque_enabled)

    def TestDriftOrConfigChangeStopsTheRoute(Self):
        for Case in ("drift", "config", "torque", "feedback"):
            Controller, Bus, _, _ = Self.StagedController()
            if Case == "drift": Bus.position["wrist_roll"] += 30
            if Case == "config": Controller.home_preset["shoulder_pan"] += 1
            if Case == "torque": Bus.torque_enabled = False
            if Case == "feedback":
                Read = Bus.read
                def Fail(Reg, Name, **Options):
                    if Reg == "Present_Position" and Name == "elbow_flex": raise ConnectionError("lost")
                    return Read(Reg, Name, **Options)
                Bus.read = Fail
            Controller.refresh()
            Self.assertIsNone(Controller.MotionRoute, Case)
            if Case == "config":
                # A changed configuration stops the route and the arm holds where it is, torque on: it latched a
                # fault and coasted every joint until 2026-10-07 (audit H2).
                Self.assertIsNone(Controller.MotionFault, Case)
                Self.assertTrue(Controller.torque_enabled and Bus.torque_enabled, Case)
                Self.assertIn("configuration", Controller.status, Case)
            else:
                Self.assertIsNotNone(Controller.MotionFault, Case)
            Self.assertFalse(any(S.command_raw is not None for S in Controller.servos.values()), Case)

    def TestConcurrentMotionCannotReplaceCheckedRoute(Self):
        Controller, Bus, _, _ = Self.StagedController()
        Count = len(Bus.writes)
        Self.assertFalse(Controller.send_ik()[0])
        Self.assertFalse(Controller.move_servo("gripper", 750)[0])
        Self.assertFalse(Controller.go_home()[0])
        Self.assertEqual(len(Bus.writes), Count)

    def TestFailedRouteStartReportsFailureEvenIfStopSucceeds(Self):
        Controller, Bus = MakeController()
        with Patch.object(Controller, "BeginRouteStage", side_effect=RuntimeError("simulated write failure")):
            Ok, Message = Controller.move_servo("shoulder_pan", Controller.servos["shoulder_pan"].present_raw + 50)
        Self.assertFalse(Ok)
        Self.assertIn("simulated write failure", Message)
        Self.assertIsNone(Controller.MotionRoute)
        Self.assertFalse(Bus.torque_enabled, "Failed route execution must coast all motors")

    def TestEmergencyInterruptsRoutePlanningWithoutMotorWrites(Self):
        Controller, Bus = MakeController()
        Entered, Release = Threading.Event(), Threading.Event()
        Results = []
        def Plan(Snapshot, *Args, **Options):
            Entered.set()
            Release.wait(2)
            return [{N: S.present_raw for N, S in Snapshot.servos.items()}]
        with Patch.object(Bridge.BridgeController, "PlanMotionRoute", Plan):
            Thread = Threading.Thread(target=lambda: Results.append(Controller.move_servo("gripper", 750)))
            Thread.start()
            Self.assertTrue(Entered.wait(2))
            Controller.emergency_stop()
            Count = len(Bus.writes)
            Release.set()
            Thread.join(2)
        Self.assertFalse(Thread.is_alive())
        Self.assertFalse(Results[0][0])
        Self.assertEqual(len(Bus.writes), Count)


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestBasketCollision` runs the Test* methods too (unittest names this hook)."""
    Prefixed = Unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return Prefixed.loadTestsFromTestCase(BasketTests)


if __name__ == "__main__":
    Loader = Unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Result = Unittest.TextTestRunner(verbosity=2).run(Loader.loadTestsFromTestCase(BasketTests))
    raise SystemExit(not Result.wasSuccessful())
