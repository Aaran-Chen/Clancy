"""Rear workspace boundary and formerly blocked side reaches. Simulated motors only."""
import itertools as Itertools
import json as Json
import math as Math
import unittest as Unittest
from pathlib import Path

import numpy as Np
import BasketCollision as Collision
import CadKinematics as Cad
from TestSafety import MakeController
from TestBasketCollision import Place, Limits
from TestFlyByRoutes import Execute

Root = Path(__file__).resolve().parent
SideReach = dict(zip(Cad.JointNames, (-59.80348210398985, -96.55153644855653, 87.73730229885464,
                                     46.91897851079648, 94.39453125, 0.0)))
RearReach = dict(zip(Cad.JointNames, (-11.48943451566926, 81.25358230403515, 53.76956572029036,
                                     55.605214686058275, 94.39453125, 0.0)))
SweepStart = dict(zip(Cad.JointNames, (-118.71531001612988, 29.256094890943217, 91.39187512455516,
                                      80.50800986562217, 94.39453125, 0.0)))
SweepGoal = dict(zip(Cad.JointNames, (77.9898639435574, 88.71375030567992, 101.17483259407607,
                                     43.266593158716546, 94.39453125, 0.0)))


class RearwardTests(Unittest.TestCase):
    def TestDisplayBoundaryForBothMountsAndTranslatedOrigins(Self):
        Controller, _ = MakeController()
        Frame = Json.loads((Root / "WorkspaceFrame.json").read_text())
        Self.assertNotIn("ArmMass", Collision.Model)
        Self.assertNotIn("SupportFootprint", Collision.Model)
        Self.assertEqual(Collision.Model["RearwardLimit"]["MinimumDisplayYCm"], -30)
        for Side in ("Right", "Left"):
            Frame["RobotFrame"]["ControlledArm"] = Side
            for Offset in (0, 12):
                Frame["RobotFrame"]["OriginAssemblyCm"][1] = Collision.Model["Mounts"][Side]["PanCm"][1] + Offset
                World = Collision.CollisionWorld(Controller.arm_geometry, Frame)
                for Delta in (-0.001, 0, 0.001):
                    Point = {"x": 0, "y": -30 + Offset + Delta, "z": 100}
                    Self.assertEqual(World.TargetReason(Point), Collision.RearwardReason if Delta < 0 else None)

    def TestSideReachIsAllowedByPreviewAndFakeMotorRoute(Self):
        Controller, Bus = MakeController()
        World = Controller.CollisionWorld()
        Self.assertIsNone(World.PoseReason(SideReach))
        Near = {**SideReach, "shoulder_pan": SideReach["shoulder_pan"] + 3}
        Place(Controller, Bus, Near)
        Target = Cad.Positions(Controller.arm_geometry, SideReach)[-1]
        Pitch = Cad.Pitch(Controller.arm_geometry, SideReach)
        Controller.elbow_mode = Cad.Branch(Controller.arm_geometry, SideReach)
        Self.assertTrue(Controller.preview_ik({**Target, "wrist_pitch": Pitch})["reachable"])
        Ok, Message = Controller.send_ik(Target={**Target, "wrist_pitch": Pitch})
        Self.assertTrue(Ok, Message)
        Execute(Self, Controller, Bus, Each=lambda _: Self.assertIsNone(World.PoseReason(Controller._current_ik_angles())))

    def TestRearwardTargetAndJointGoalAreRejectedWithoutWrites(Self):
        Controller, Bus = MakeController()
        World = Controller.CollisionWorld()
        Self.assertIsNone(World.PoseReason(RearReach, RearwardSlack=Math.inf))
        Self.assertEqual(World.PoseReason(RearReach), Collision.RearwardReason)
        Target = Cad.Positions(Controller.arm_geometry, RearReach)[-1]
        Requested = {**Target, "wrist_pitch": Cad.Pitch(Controller.arm_geometry, RearReach)}
        Self.assertFalse(Controller.preview_ik(Requested)["reachable"])
        for Send in (lambda: Controller.set_target(Requested), lambda: Controller.send_ik(Target=Requested),
                     lambda: Controller.SendJointMotion({Name: Controller._ik_raw_from_angle(Controller.servos[Name], RearReach[Name])
                                                         for Name in Cad.SolveNames}, "Test")):
            Ok, Message = Send()
            Self.assertFalse(Ok)
            Self.assertIn("Y = -30 cm", Message)
            Self.assertFalse(Bus.writes)

    def TestClearEndpointsCannotSweepBehindLimit(Self):
        Controller, _ = MakeController()
        World = Controller.CollisionWorld()
        Self.assertIsNone(World.PoseReason(SweepStart))
        Self.assertIsNone(World.PoseReason(SweepGoal))
        Start, Goal = [Np.array([Pose[Name] for Name in Cad.JointNames]) for Pose in (SweepStart, SweepGoal)]
        Self.assertEqual(World.IntervalReason((Start + Goal) / 2, (Start + Goal) / 2, False), Collision.RearwardReason)
        Self.assertEqual(World.EdgeReason(Start, Goal), Collision.RearwardReason)

    def TestIntervalBoundCoversEveryBoxCorner(Self):
        Controller, _ = MakeController()
        World = Controller.CollisionWorld()
        Random = Np.random.default_rng(4)
        Bounds = Limits(Controller)
        Lower = Np.array([Bounds[Name][0] for Name in Cad.JointNames])
        Upper = Np.array([Bounds[Name][1] for Name in Cad.JointNames])
        for _ in range(40):
            Mid = Random.uniform(Lower, Upper)
            Half = Random.uniform(0, 12, 6)
            MidLinks, _ = Cad.Forward(Controller.arm_geometry, dict(zip(Cad.JointNames, Mid)))
            Bound = World.RearwardExcess(MidLinks, (Half + World.Slack) * Collision.Degrees)
            for _ in range(20):
                Sample = Mid + Random.uniform(-1, 1, 6) * (Half + World.Slack)
                Links, _ = Cad.Forward(Controller.arm_geometry, dict(zip(Cad.JointNames, Sample)))
                for _, Link, Center, Size, _ in World.Boxes:
                    Corners = Center + Np.array(list(Itertools.product((-1, 1), repeat=3))) * Size
                    Minimum = min((Links[Link][:3, :3] @ Corner + Links[Link][:3, 3])[1] for Corner in Corners)
                    Self.assertLessEqual(World.MinimumY - Minimum, Bound + 1e-9)

    def TestHandPosedRearwardArmCanReturnWithoutReachingFurtherBack(Self):
        Controller, Bus = MakeController()
        World = Controller.CollisionWorld()
        Place(Controller, Bus, RearReach)
        Start = Np.array([Controller._current_ik_angles()[Name] for Name in Cad.JointNames])
        Relief = World.StartRelief(Start)
        Self.assertGreater(Relief["RearwardSlackCm"], 0)
        Goal = {**Controller._current_ik_angles(), "shoulder_lift": 45}
        Path = World.Plan(Controller._current_ik_angles(), Goal, Limits(Controller), Seconds=3)
        Self.assertTrue(Path)
        Random = Np.random.default_rng(15)
        Previous = Start
        for Index, Point in enumerate(Path):
            Next = Np.array([Point[Name] for Name in Cad.JointNames])
            Self.assertIsNone(World.EdgeReason(Previous, Next, Relief=Relief if Index == 0 else None))
            Self.assertIsNone(World.IntervalReason(Next, Next))
            for _ in range(80):
                Sample = Random.uniform(Np.minimum(Previous, Next), Np.maximum(Previous, Next))
                Links, _ = Cad.Forward(Controller.arm_geometry, dict(zip(Cad.JointNames, Sample)))
                Self.assertLessEqual(World.RearwardExcess(Links), Relief["RearwardSlackCm"] if Index == 0 else 0)
            Previous = Next


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestRearwardLimit` runs the Test* methods too (unittest names this hook)."""
    Prefixed = Unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return Prefixed.loadTestsFromTestCase(RearwardTests)


if __name__ == "__main__":
    Loader = Unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Result = Unittest.TextTestRunner(verbosity=2).run(Loader.loadTestsFromTestCase(RearwardTests))
    raise SystemExit(not Result.wasSuccessful())
