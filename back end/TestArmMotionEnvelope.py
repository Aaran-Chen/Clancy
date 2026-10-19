"""Offline containment and false-contact regressions for directional envelopes."""
import itertools as Itertools
import unittest as Unittest

import numpy as Np
import BasketCollision as Collision
from ArmMotionEnvelope import BoxEnvelope, Signs
from TestBasketCollision import MakeController, Limits, Place

# Nominally clear positions previously rejected by isotropic encoder padding.
# Includes reaches outside the front of the basket and into its open tray.
# The fourth (tip at display (-22, 27.4, 1.9), 4 cm in front of the measured
# front rim) is flagged at 75% of the padding, not at its edge, so a small rim
# correction cannot silently turn it into a pose the padded check never flags.
ClearPoses = [
    (45.13556381586096, -40.40638462172899, 21.14504552886892, -89.79602432212215, 94.39453125, 0),
    (-42.12475275031272, 49.419144335227344, -80.44961185309788, -5.841756384910141, 94.39453125, 0),
    (-30.11188779284906, -29.00415995692738, -48.35213668520457, 81.00749917310267, 94.39453125, 0),
    (-16.006408397602456, -8.225155243634065, -20.850903392747014, -25.38324122836822, 94.39453125, 0),
    (-3.649021223314236, -40.3159475786256, -6.744117888761622, -93.20921228974743, 94.39453125, 0),
]


class EnvelopeTests(Unittest.TestCase):
    def setUp(Self):
        Self.Controller, Self.Bus = MakeController()
        Self.World = Self.Controller.CollisionWorld()

    def TestFalseBasketWarningsClearWithoutReducingTrackingAllowance(Self):
        World = Self.World
        Self.assertEqual(World.Slack, 16 * 360 / 4096)
        Random = Np.random.default_rng(4096)
        for Values in ClearPoses:
            Angles = dict(zip(Collision.Names, Values))
            Links, _ = Collision.Cad.Forward(Self.Controller.arm_geometry, Angles)
            OldReasons = []
            for _, Link, Center, Half, Radii in World.Boxes:
                Transform = Links[Link]
                OldReasons.append(World.ObstacleReason(Transform[:3, :3] @ Center + Transform[:3, 3],
                    Transform[:3, :3], Half, float(Radii.sum() * World.Slack * Collision.Degrees)))
            Self.assertIn("basket", OldReasons)
            Self.assertIsNone(World.PoseReason(Angles))
            Self.assertIsNone(World.EdgeReason(Np.array(Values), Np.array(Values)))
            # Independent concrete poses provide a separate nominal-contact oracle.
            Samples = list(Itertools.product((-World.Slack, World.Slack), repeat=6))
            Samples.extend(Random.uniform(-World.Slack, World.Slack, (80, 6)))
            for Delta in Samples:
                Sample = dict(zip(Collision.Names, Np.array(Values) + Delta))
                Self.assertIsNone(World.PoseReason(Sample, Tracking=False))
        Self.assertFalse(Self.Bus.writes)

    def TestEnvelopeContainsEveryCornerAcrossIndependentJointIntervals(Self):
        World = Self.World
        Random = Np.random.default_rng(190926)
        Ranges = Np.array([Limits(Self.Controller)[Name] for Name in Collision.Names])
        for Case in range(32):
            Mid = Random.uniform(Ranges[:, 0], Ranges[:, 1])
            Width = Random.uniform(0, (2, 8, 30, 180)[Case % 4], 6)
            if Case % 5 == 0:
                Width[::2] = 0  # Fixed joints and mixed direction/width intervals.
            Links, Frames = Collision.Cad.Forward(Self.Controller.arm_geometry, dict(zip(Collision.Names, Mid)))
            Envelopes = [BoxEnvelope(Center, Half, Radii, Links[Link], Frames, Width * Collision.Degrees)
                         for _, Link, Center, Half, Radii in World.Boxes]
            Samples = list(Itertools.product((-1, 1), repeat=6))
            Samples.extend(Random.uniform(-1, 1, (20, 6)))
            for Delta in Samples:
                Actual, _ = Collision.Cad.Forward(Self.Controller.arm_geometry,
                    dict(zip(Collision.Names, Mid + Np.array(Delta) * Width)))
                for Box, (Center, Rotation, Half) in zip(World.Boxes, Envelopes):
                    _, Link, LocalCenter, LocalHalf, _ = Box
                    Corners = LocalCenter + Signs * LocalHalf
                    WorldCorners = Corners @ Actual[Link][:3, :3].T + Actual[Link][:3, 3]
                    Local = (WorldCorners - Center) @ Rotation
                    Self.assertLessEqual(float(Np.max(Np.abs(Local) - Half)), 1e-7, (Case, Link))

    def TestZeroMotionEnvelopeMatchesRenderedBox(Self):
        Links, Frames = Collision.Cad.Forward(Self.Controller.arm_geometry, Self.Controller._current_ik_angles())
        for _, Link, Center, Half, Radii in Self.World.Boxes:
            ActualCenter, Rotation, ActualHalf = BoxEnvelope(Center, Half, Radii, Links[Link], Frames, Np.zeros(6))
            Np.testing.assert_allclose(ActualCenter, Links[Link][:3, :3] @ Center + Links[Link][:3, 3], atol=1e-8)
            Np.testing.assert_allclose(Rotation, Links[Link][:3, :3])
            Np.testing.assert_allclose(ActualHalf, Half, atol=1e-8)

    def TestFormerMastPaddingWarningAlsoClears(Self):
        Angles = dict(zip(Collision.Names, (52.41018969743456, -19.77491372635835, -9.801173360715865,
                                           -59.41393702872465, 94.39453125, 0)))
        Self.assertIsNone(Self.World.PoseReason(Angles))
        Values = Np.array(list(Angles.values()))
        Self.assertIsNone(Self.World.EdgeReason(Values, Values))

    def TestRealContactsAreStillRejectedWithTracking(Self):
        World = Self.World
        Random = Np.random.default_rng(72)
        Ranges = Limits(Self.Controller)
        Hits = 0
        for _ in range(160):
            Angles = {Name: float(Random.uniform(*Ranges[Name])) for Name in Collision.Names}
            Nominal = World.PoseReason(Angles, Tracking=False)
            if Nominal and ("basket" in Nominal or "robot body" in Nominal):
                Hits += 1
                Self.assertIsNotNone(World.PoseReason(Angles))
        Self.assertGreater(Hits, 15)

    def TestTrackingOnlyContactIsTheContactAllowances(Self):
        # Place a tiny obstacle at a real corner reached only by encoder error.
        # The midpoint is clear. Since v8 (CAD-exact contact, approved 2026-10-04)
        # the pose is judged by its CAD boxes alone and passes; with the contact
        # allowance at the joint allowance (v7) the same contact is refused.
        World = Self.World
        Values = Np.array(ClearPoses[3])
        LinkBox = World.Boxes[-1]
        _, Link, Center, Half, _ = LinkBox
        MidLinks, _ = Collision.Cad.Forward(Self.Controller.arm_geometry, dict(zip(Collision.Names, Values)))
        Moved = Values + World.Slack
        MovedLinks, _ = Collision.Cad.Forward(Self.Controller.arm_geometry, dict(zip(Collision.Names, Moved)))
        Corners = Center + Signs * Half
        WorldCorners = Corners @ MovedLinks[Link][:3, :3].T + MovedLinks[Link][:3, 3]
        Local = (WorldCorners - MidLinks[Link][:3, 3]) @ MidLinks[Link][:3, :3] - Center
        Point = WorldCorners[Np.argmax(Np.max(Np.abs(Local) - Half, axis=1))]
        from BasketSurface import Surface
        from TestBasketSurface import CubeSource
        Source = CubeSource()
        Source["Vertices"] = (Np.array(Source["Vertices"]) * .001 + Point).tolist()
        Source["Nodes"][0]["Bounds"] = [(Point - .001).tolist(), (Point + .001).tolist()]
        World.Surface = Surface([0, 0, 0], Source)
        World.Boxes = [LinkBox]
        Self.assertIsNone(World.PoseReason(dict(zip(Collision.Names, Values)), Tracking=False))
        Self.assertIn("basket", World.PoseReason(dict(zip(Collision.Names, Moved)), Tracking=False))
        Self.assertEqual(Collision.Model["ContactTrackingAllowanceRaw"], 0)
        Self.assertIsNone(World.PoseReason(dict(zip(Collision.Names, Values))))
        Self.assertIsNone(World.EdgeReason(Values, Values))
        World.ContactSlack = World.Slack
        Self.assertIn("basket", World.PoseReason(dict(zip(Collision.Names, Values))))
        Self.assertIn("basket", World.EdgeReason(Values, Values))

    def TestFormerlyBlockedPoseCanExecuteThroughFakeMotors(Self):
        Controller, Bus = Self.Controller, Self.Bus
        Place(Controller, Bus, dict(zip(Collision.Names, ClearPoses[3])))
        Raw = Controller.servos["shoulder_pan"].present_raw
        Ok, Message = Controller.move_servo("shoulder_pan", Raw + 12)
        Self.assertTrue(Ok, Message)
        for _ in range(250):
            for Name in Collision.Names:
                Bus.step(Name, steps_per_tick=4)
            Controller.refresh()
            Self.assertIsNone(Controller.MotionFault)
            Self.assertIsNone(Self.World.PoseReason(Controller._current_ik_angles(), Tracking=False))
            if Controller.MotionRoute is None:
                break
        Self.assertIsNone(Controller.MotionRoute)
        Self.assertGreater(Controller.servos["shoulder_pan"].present_raw, Raw)
        Self.assertEqual(Controller.servos["shoulder_pan"].target_raw, Raw + 12)
        Self.assertLessEqual(abs(Controller.servos["shoulder_pan"].present_raw - Raw - 12),
                            Collision.Model["JointTrackingAllowanceRaw"])


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestArmMotionEnvelope` runs the Test* methods too (unittest names this hook)."""
    Prefixed = Unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return Prefixed.loadTestsFromTestCase(EnvelopeTests)


if __name__ == "__main__":
    Loader = Unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Result = Unittest.TextTestRunner(verbosity=2).run(Loader.loadTestsFromTestCase(EnvelopeTests))
    raise SystemExit(not Result.wasSuccessful())
