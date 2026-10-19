"""Direct-first route repair and cost regressions. Offline, never serial I/O."""
import time as Time
import unittest as Unittest
from unittest.mock import patch as Patch

import numpy as Np
import BasketCollision as Collision
import bridge_server as Bridge
from TestBasketCollision import MakeController, Limits, StartValues, GoalValues


def Angles(Values):
    return dict(zip(Collision.Names, Values))


def Vectors(Start, Path):
    return [Np.array(Start)] + [Np.array([Point[Name] for Name in Collision.Names]) for Point in Path]


class ObstacleWorld(Collision.CollisionWorld):
    def __init__(Self, Origin=None):
        Self.Origin = Np.zeros(6) if Origin is None else Origin

    def IntervalReason(Self, Low, High, Tracking=True):
        Low, High = Low - Self.Origin, High - Self.Origin
        return "basket" if High[0] >= 10 and Low[0] <= 30 and High[1] >= -5 and Low[1] <= 5 else None


class RouteOptimizationTests(Unittest.TestCase):
    def TestSafeDirectMotionDoesNotSearchOrAddStops(Self):
        Controller, Bus = MakeController()
        World = Controller.CollisionWorld()
        Start = Controller._current_ik_angles()
        Goal = {**Start, "shoulder_pan": Start["shoulder_pan"] + 3}
        with Patch.object(World, "ImprovePath", side_effect=AssertionError("direct move searched")), \
                Patch.object(World, "EdgeReason", wraps=World.EdgeReason) as Check:
            Self.assertEqual(World.Plan(Start, Goal, Limits(Controller)), [Goal])
        Self.assertEqual(Check.call_count, 1)
        Self.assertFalse(Bus.writes)

    def TestBodySafeOrderedBaselineBecomesFasterFlyByMotion(Self):
        Controller, _ = MakeController()
        World = Controller.CollisionWorld()
        Start, Goal = Np.array(StartValues), Np.array(GoalValues)
        Cost = Controller.MotionRouteCost()
        # The coarse isotropic envelope used to falsely block this direct sweep.
        # Keep a checked one-joint-at-a-time baseline to compare its travel time.
        Self.assertIsNone(World.EdgeReason(Start, Goal))
        Baseline, Current = [Start], Start.copy()
        for Index in (0, 1, 2, 3):
            Next = Current.copy()
            Next[Index] = Goal[Index]
            Gates = World.LegGates(Current, Next)
            Self.assertIsNotNone(Gates)
            Baseline.extend(Gates)
            Current = Next
        Path = World.Plan(Angles(Start), Angles(Goal), Limits(Controller), EdgeCost=Cost)
        Result = Vectors(Start, Path)
        Self.assertLess(World.PathCost(Result, Cost), World.PathCost(Baseline, Cost))
        Self.assertEqual(Path[-1], Angles(Goal))
        for A, B in zip(Result, Result[1:]):
            Self.assertIsNone(World.EdgeReason(A, B))
            Np.testing.assert_array_equal(B[4:], Start[4:])

    def TestProgressGatesKeepClearNominalLineWithoutGeometricDetour(Self):
        class CorridorWorld(Collision.CollisionWorld):
            def __init__(Self): pass
            def IntervalReason(Self, Low, High, Tracking=True):
                return "basket" if High[0] - Low[1] > 45 or High[1] - Low[0] > 45 else None
        Start, Goal = Np.zeros(6), Np.array([80., 80, 0, 0, 0, 0])
        World = CorridorWorld()
        Self.assertIsNotNone(World.EdgeReason(Start, Goal))
        Path = World.Plan(Angles(Start), Angles(Goal), dict.fromkeys(Collision.Names, (-90, 90)))
        # Fly-by gates stay on the nominal line; only the last point stops.
        Self.assertGreater(len(Path), 1)
        Self.assertEqual(Path[-1], Angles(Goal))
        for Point in Path:
            Self.assertAlmostEqual(Point["shoulder_pan"], Point["shoulder_lift"])
        for A, B in zip(Vectors(Start, Path), Vectors(Start, Path)[1:]):
            Self.assertIsNone(World.EdgeReason(A, B))

    def TestSmallDetourKeepsGoalAndToolAngles(Self):
        Start, Goal = Np.zeros(6), Np.array([40., 0, 0, 0, 0, 0])
        World = ObstacleWorld()
        Path = World.Plan(Angles(Start), Angles(Goal), dict.fromkeys(Collision.Names, (-90, 90)))
        Self.assertLessEqual(max(abs(Point["shoulder_lift"]) for Point in Path), 8)
        Self.assertEqual(Path[-1], Angles(Goal))
        for A, B in zip(Vectors(Start, Path), Vectors(Start, Path)[1:]):
            Self.assertIsNone(World.EdgeReason(A, B))
            Np.testing.assert_array_equal(B[4:], Start[4:])

    def TestQuantizationPreservesNecessaryMoveAwayAndReturn(Self):
        Controller, Bus = MakeController()
        Start = Controller._current_ik_angles()
        Origin = Np.array([Start[Name] for Name in Collision.Names])
        World = ObstacleWorld(Origin)
        Goal = {"shoulder_pan": Controller._ik_raw_from_angle(Controller.servos["shoulder_pan"], Origin[0] + 40)}
        with Patch.object(Controller, "CollisionWorld", return_value=World):
            Path = Controller.PlanMotionRoute(Goal)
        Raw = {Name: Servo.present_raw for Name, Servo in Controller.servos.items()}
        Self.assertTrue(any(Point["shoulder_lift"] != Raw["shoulder_lift"] for Point in Path[:-1]))
        Self.assertEqual(Path[-1], {**Raw, **Goal})
        for Point in Path:
            for Name in ("wrist_roll", "gripper"):
                Self.assertEqual(Point[Name], Raw[Name])
            for Name, Value in Point.items():
                Self.assertTrue(Controller._contains_raw(Controller.servos[Name], Value))
        Self.assertFalse(Bus.writes)

    def TestShortcutCannotCutThroughObstacle(Self):
        World = ObstacleWorld()
        Path = [Np.array(Point, dtype=float) for Point in (
            (0, 0, 0, 0, 0, 0), (0, 8, 0, 0, 0, 0), (40, 8, 0, 0, 0, 0), (40, 0, 0, 0, 0, 0))]
        Cost = lambda A, B: float(Np.max(Np.abs(B - A))) + 2
        Result = World.ImprovePath(Path, Cost, Time.monotonic() + 1)
        Self.assertIsNotNone(World.EdgeReason(Result[0], Result[-1]))
        Self.assertLessEqual(World.PathCost(Result, Cost), World.PathCost(Path, Cost))
        for A, B in zip(Result, Result[1:]):
            Self.assertIsNone(World.EdgeReason(A, B))

    def TestGlobalFallbackWorksWhenLocalSingleAxisRepairsCannot(Self):
        class TwoAxisWorld(Collision.CollisionWorld):
            def __init__(Self): pass
            def IntervalReason(Self, Low, High, Tracking=True):
                return "basket" if (High[0] >= 0 and Low[0] <= 40
                                    and (Low[1] <= 10 or Low[2] <= 10)) else None
        Start, Goal = Np.array([-20., 0, 0, 0, 0, 0]), Np.array([60., 0, 0, 0, 0, 0])
        World = TwoAxisWorld()
        Path = World.Plan(Angles(Start), Angles(Goal), dict.fromkeys(Collision.Names, (-40, 80)), Seconds=2)
        Self.assertTrue(any(Point["shoulder_lift"] > 10 and Point["elbow_flex"] > 10 for Point in Path))
        Self.assertEqual(Path[-1], Angles(Goal))
        for A, B in zip(Vectors(Start, Path), Vectors(Start, Path)[1:]):
            Self.assertIsNone(World.EdgeReason(A, B))

    def TestRepairsThroughARefusedRegionAreRejected(Self):
        # Any interval the world refuses (here the shoulder outside +/-5 deg) rejects the repairs through it. This
        # once stood in for the calibration shoulder-load check, which was removed on 2026-10-03 (446462d).
        class RestrictedWorld(ObstacleWorld):
            def IntervalReason(Self, Low, High, Tracking=True):
                if Low[1] < -5 or High[1] > 5:
                    return "Transition leaves the region this world allows"
                return super().IntervalReason(Low, High, Tracking)
        Start, Goal = Np.zeros(6), Np.array([40., 0, 0, 0, 0, 0])
        with Self.assertRaisesRegex(ValueError, "No clear route"):
            RestrictedWorld().Plan(Angles(Start), Angles(Goal), dict.fromkeys(Collision.Names, (-90, 90)), Seconds=0.1)

    def TestShortcutCanRemoveMoreThanEightOldWaypoints(Self):
        class EmptyWorld(ObstacleWorld):
            def IntervalReason(Self, Low, High, Tracking=True): return None
        Path = [Np.array([Index * 5., 0, 0, 0, 0, 0]) for Index in range(15)]
        Result = EmptyWorld().ImprovePath(Path, lambda A, B: float(Np.max(Np.abs(B - A))) + 2,
                                         Time.monotonic() + 1)
        Self.assertEqual(len(Result), 2)
        Np.testing.assert_array_equal(Result[-1], Path[-1])

    def TestCostAccountsForConcurrentJointsAndWaypointFrames(Self):
        Controller, _ = MakeController()
        Start = Np.zeros(6)
        First = Np.array([40., 0, 0, 0, 0, 0])
        Goal = Np.array([40., 40, 0, 0, 0, 0])
        Cost = Controller.MotionRouteCost()
        Self.assertLess(Cost(Start, Goal), Cost(Start, First) + Cost(First, Goal))
        Self.assertLess(Cost(Start, Goal), Cost(Start, Goal / 2) + Cost(Goal / 2, Goal))
        Controller.SetMotionSpeed(100)
        Self.assertLess(Controller.MotionRouteCost()(Start, Goal), Cost(Start, Goal))

    def TestMalformedOrOutOfRangeTargetFailsBeforeSearch(Self):
        World = ObstacleWorld()
        Start = Np.zeros(6)
        for Value in (float("nan"), float("inf"), 91):
            Goal = Start.copy()
            Goal[0] = Value
            with Self.assertRaisesRegex(ValueError, "No motion sent"):
                World.Plan(Angles(Start), Angles(Goal), dict.fromkeys(Collision.Names, (-90, 90)))

    def TestPlannerVersionIsExposed(Self):
        Controller, _ = MakeController()
        Self.assertEqual(Controller.state_snapshot()["route_planner_version"], "flyby-cad-exact-v8")
        Self.assertEqual(Bridge.health()["route_planner_version"], "flyby-cad-exact-v8")


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestBasketRouteOptimization` runs the Test* methods too (unittest names this hook)."""
    Prefixed = Unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return Prefixed.loadTestsFromTestCase(RouteOptimizationTests)


if __name__ == "__main__":
    Loader = Unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Result = Unittest.TextTestRunner(verbosity=2).run(Loader.loadTestsFromTestCase(RouteOptimizationTests))
    raise SystemExit(not Result.wasSuccessful())
