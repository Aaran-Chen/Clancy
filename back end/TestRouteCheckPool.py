"""Route collision checks in worker processes (RouteCheckPool): the same answers, the same routes, sooner.
Simulated motors only: never opens a serial port or the depth camera; the workers import only the collision code.
The pools here have 4 and 8 workers, as the bridge has on this PC; SO101_TEST_ROUTE_WORKERS caps them (RunAllTests.py
uses 2, so the suite and its workers stay three processes).

2026-10-04 15:31 the 27.3 cm move to the pill bottle (the jaws opening on the way) planned for about 2.3 s on
the live bridge before the arm moved; offline its waypoint reach alone took 1.4 s of 1.6 s, one check at a time.
"""
import os as Os
import time as Time
import unittest as Unittest
from unittest.mock import patch as Patch

import BasketCollision as Collision
import bridge_server as Bridge
import RouteCheckPool
from TestBasketCollision import DisplayToModel
from TestGripperWithTip import PlaceRaw
from TestSafety import MakeController

HardwareCalls = []
# 2026-10-04 15:31:57: Home with the jaws shut, then the hover beside the pill bottle, opening the jaws on the way.
Raw1531 = {"shoulder_pan": 3471, "shoulder_lift": 3173, "elbow_flex": 2688, "wrist_flex": 856, "wrist_roll": 1423, "gripper": 696}
Hover1531 = {**DisplayToModel(11.1, 32.6, 8.9), "wrist_pitch": -13.9}
Open1531 = 1442
# 15:32:21: the 4.4 cm descent from that hover (two arrive waypoints beside the bottle).
Raw1532 = {"shoulder_pan": 3045, "shoulder_lift": 722, "elbow_flex": 1462, "wrist_flex": 527, "wrist_roll": 1423, "gripper": 1438}
Descent1532 = {**DisplayToModel(11.071199543312222, 32.63640799118939, 3.378639375684835), "wrist_pitch": -13.9}


def setUpModule():
    def GuardedOpener():
        HardwareCalls.append("depth camera opened")
        raise RuntimeError("an offline test reached the real depth camera")
    for Item in (Patch.object(Bridge.depth_camera, "Opener", GuardedOpener),
                 Patch.object(Bridge, "RaiseProcessPriority", lambda: HardwareCalls.append("priority raised") or "normal"),
                 # Long enough that the reach clock never decides: both ways then decide alike.
                 Patch.object(Bridge, "ROUTE_REACH_ASSIGN_SECONDS", 60.0)):
        Item.start()
        Unittest.addModuleCleanup(Item.stop)


def PoolSize(Count):
    """Count workers, or SO101_TEST_ROUTE_WORKERS if that is fewer."""
    Cap = int(Os.environ.get("SO101_TEST_ROUTE_WORKERS") or 0)
    return min(Count, Cap) if Cap > 0 else Count


def StartedPool(Count):
    Pool = RouteCheckPool.Pool(PoolSize(Count))
    Pool._Start()
    return Pool


def Plan(Start, Target, Gripper=None):
    Controller, Bus = MakeController()
    PlaceRaw(Controller, Bus, Start)
    Controller.SetMotionSpeed(100)
    Ok, Message = Controller.send_ik(Target=Target, GripperRaw=Gripper)
    Route = Controller.MotionRoute
    return Ok, Message, Controller, (None if Route is None else
                                     (Route["Path"], sorted(Route["Tight"]), sorted(Route["Arrive"]), Route["Reach"]))


class RouteCheckPoolTests(Unittest.TestCase):
    def TestWorkersAnswerAsTheBridgeDoes(Self):
        """Every stage of the 15:31 route, with no reach, the plain reach and one far too wide: the same verdicts and
        reasons from a worker as from the bridge's own check."""
        Ok, Message, Controller, Route = Plan(Raw1531, Hover1531, Open1531)
        Self.assertTrue(Ok, Message)
        Check = Controller.SolverSnapshot()
        Geometry, Frame = Check.arm_geometry, Bridge._load_json(Bridge.BACKEND_ROOT / "WorkspaceFrame.json")
        Requests, Previous = [], Raw1531
        for Point in Route[0]:
            for Reach in (({}, 0), ({}, Collision.WaypointReachRaw), ({}, 600)):
                Requests.append((Geometry, Frame, *Check.EdgeCheckArguments(Previous, Point, None, Reach, 64)))
            Previous = Point
        Pool = StartedPool(4)
        Self.addCleanup(Pool.Close)
        Self.assertIsNone(Pool.Problem)
        Self.assertEqual(len(Pool.Workers), PoolSize(4))
        Answers = Pool.Map(Requests, 60)
        Expected = [RouteCheckPool.Check(*Request) for Request in Requests]
        Self.assertEqual([Answer[1] for Answer in Answers], Expected)
        Self.assertTrue(any(Reason is None for Reason in Expected) and any(Reason for Reason in Expected),
                        "the cases include both a certified and a refused stage")
        Self.assertEqual(HardwareCalls, [])

    def TestRoutesComeOutTheSameWithWorkers(Self):
        """15:31 and 15:32 planned with the workers: the same waypoints, tight, arrive and reach as without them,
        with the workers asked and the plan quicker."""
        Cases = [(Raw1531, Hover1531, Open1531), (Raw1532, Descent1532, None)]
        Alone = []
        # Each plan timed twice, the faster kept, alone and with the workers alike: a plan that other work on the PC
        # interrupted is not its cost (audit 1 L5), and the workers must win on a busy PC as on an idle one.
        for Case in Cases:
            Timed = []
            for _ in range(2):
                Began = Time.perf_counter()
                Result = Plan(*Case)
                Timed.append(Time.perf_counter() - Began)
            Alone.append((Result, min(Timed)))
        Pool = StartedPool(8)
        Self.addCleanup(Pool.Close)
        Asked = []
        Submit = Pool.Submit
        Pool.Submit = lambda Request, Deadline: Asked.append(Request) or Submit(Request, Deadline)
        with Patch.object(RouteCheckPool, "_Pool", Pool):
            Self.assertIs(RouteCheckPool.Running(), Pool)
            for Case, ((Ok, Message, _, Route), Seconds) in zip(Cases, Alone):
                Self.assertTrue(Ok, Message)
                Timed = []
                for _ in range(2):
                    Began = Time.perf_counter()
                    Again = Plan(*Case)
                    Timed.append(Time.perf_counter() - Began)
                Parallel = min(Timed)
                Self.assertTrue(Again[0], Again[1])
                Self.assertEqual(Again[3], Route)
                Self.assertLess(Parallel, Seconds, "planning is quicker with the workers")
        Self.assertGreater(len(Asked), 10)
        # The CAD-exact contact model (v8) left the descent no arrive waypoint (two under v7's tracking envelope).
        Self.assertEqual(Alone[1][0][3][2], [], "the descent has no arrive waypoint")

    def TestWorkersWithOtherCollisionCodeAreTurnedAway(Self):
        """A worker whose collision code or data differ from the bridge's (a file changed after the bridge
        started) is never asked: the pool reports why, and planning checks in process as before."""
        with Patch.object(RouteCheckPool, "LoadedFingerprint", "not-the-workers"):
            Pool = StartedPool(2)
        Self.addCleanup(Pool.Close)
        Self.assertEqual(Pool.Workers, [])
        Self.assertIn("different collision code", Pool.Problem)
        with Patch.object(RouteCheckPool, "_Pool", Pool):
            Self.assertIsNone(RouteCheckPool.Running())
            Self.assertEqual(RouteCheckPool.WorkerCount(), 0)
            Ok, Message, _, Route = Plan(Raw1532, Descent1532)
        Self.assertTrue(Ok, Message)
        Self.assertEqual(Route[2], [])

    def TestLostWorkersFallBackToTheBridge(Self):
        """Workers that die while the bridge plans (killed here) cost no route: the bridge checks what they did not
        answer itself, and gets the route it gets alone."""
        Ok, Message, _, Route = Plan(Raw1531, Hover1531, Open1531)
        Self.assertTrue(Ok, Message)
        Pool = StartedPool(2)
        Self.addCleanup(Pool.Close)
        Original = Pool.Submit

        def KillFirst(Request, Deadline):
            for Each in Pool.Workers:
                Each.Process.kill()
            return Original(Request, Deadline)
        Pool.Submit = KillFirst
        with Patch.object(RouteCheckPool, "_Pool", Pool):
            Began = Time.perf_counter()
            Again = Plan(Raw1531, Hover1531, Open1531)
        Self.assertTrue(Again[0], Again[1])
        Self.assertEqual(Again[3], Route)
        Self.assertLess(Time.perf_counter() - Began, 15)
        Self.assertTrue(all(not Each.Alive for Each in Pool.Workers))


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestRouteCheckPool` runs the Test* methods too (unittest names this hook)."""
    Prefixed = Unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return Prefixed.loadTestsFromTestCase(RouteCheckPoolTests)


if __name__ == "__main__":
    Loader = Unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Result = Unittest.TextTestRunner(verbosity=2).run(Loader.loadTestsFromTestCase(RouteCheckPoolTests))
    raise SystemExit(not Result.wasSuccessful())
