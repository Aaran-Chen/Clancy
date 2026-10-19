"""Alternate wrist poses and entry through the basket rim. Fake motors only."""
import hashlib as Hashlib
import math as Math
import random as Random
import re as Re
import unittest as Unittest
from pathlib import Path
from unittest.mock import patch as Patch

import numpy as Np
import CadKinematics as Cad
import bridge_server as Bridge
from TestSafety import MakeController

InteriorTarget = {"x": -1.9532211879722914, "y": 14.308657609320962, "z": -5.741477864378066}
InteriorPitch = -108.0
AlternateTarget = {"x": -12.441904167611757, "y": 6.413114025735255, "z": 20.974084794840152}
AlternatePitch = 149.67387933428267
# Two refused approvals, from the servo positions in their model requests (captures/<date>/recording/ai.jsonl).
# Targets are the recorded display targets in the model frame (display offset -13.3, 0, 4.7324).
# 2026-09-28 22:49: display (-11.583, 14.998, 12.262) under the corrected wrist-roll direction (recorded as (-13.156, 15.082, 12.233)), refused "Quantized servo target exceeds the CAD IK pitch tolerance".
Raws2249 = {"shoulder_pan": 3487, "shoulder_lift": 3168, "elbow_flex": 2685, "wrist_flex": 854, "wrist_roll": 1514, "gripper": 700}
Target2249 = {"x": -1.7172967997255948, "y": 14.997547758602197, "z": 7.529752675193479}
Pitch2249 = -26.95000000000006
# 2026-09-29 07:58: display (6.587, 33.949, 4.224) under the corrected wrist-roll direction (recorded as (5.2, 34.7, 4.2)), refused with the bare "IK target is unreachable at this wrist angle".
Raws0758 = {"shoulder_pan": 3486, "shoulder_lift": 3167, "elbow_flex": 2688, "wrist_flex": 862, "wrist_roll": 1522, "gripper": 699}
Target0758 = {"x": -19.88711733159919, "y": 33.949491801933206, "z": -0.508139173881216}
Pitch0758 = -43.575000000000024


def ControllerAt(Raws):
    """A fake-motor controller holding still at Raws, with the elbow branch connect() would pick there."""
    Controller, Bus = MakeController()
    for Name, Raw in Raws.items():
        Bus.position[Name] = Bus.registers[("Goal_Position", Name)] = Raw
    Controller.refresh()
    for Servo in Controller.servos.values():
        Servo.target_raw = Servo.present_raw
    Controller.elbow_mode = Controller._current_elbow_branch(Controller._current_ik_angles())
    Bus.writes.clear()
    return Controller, Bus


def PoseErrors(Controller, Raw, Target, Pitch):
    """Tip (cm) and wrist pitch (deg) errors of the IK joints at Raw, the other joints where they are."""
    Angles = {**Controller._current_ik_angles(),
              **{Name: Controller._ik_angle_from_raw(Controller.servos[Name], Value) for Name, Value in Raw.items()}}
    Tip = Bridge.forward_tip_position(Controller.arm_geometry, Angles)
    return (Math.dist(tuple(Tip[Axis] for Axis in "xyz"), tuple(Target[Axis] for Axis in "xyz")),
            Controller._angle_delta_abs(Bridge.wrist_tip_pitch_degrees(Controller.arm_geometry, Angles), Pitch))


class CartesianTests(Unittest.TestCase):
    def TestCollisionFreeBranchBeatsCollidingPreferredBranch(Self):
        Controller, Bus = MakeController()
        World = Controller.CollisionWorld()
        Options = dict(initial_angles=Controller._current_ik_angles(), elbow="down", wrist_pitch_deg=AlternatePitch,
                       joint_limits={Name: (-170, 170) for Name in Cad.SolveNames})
        Coordinates = tuple(AlternateTarget[Axis] for Axis in ("x", "y", "z"))
        Preferred = Cad.Solve(Controller.arm_geometry, *Coordinates, **Options)
        Self.assertTrue(Preferred["reachable"])
        Self.assertIsNotNone(World.PoseReason(Preferred["joint_angles_degrees"], Tracking=False))
        Result = Cad.Solve(Controller.arm_geometry, *Coordinates, **Options, PoseAllowed=lambda Angles: World.PoseReason(Angles) is None)
        Self.assertTrue(Result["reachable"])
        Self.assertIsNone(World.PoseReason(Result["joint_angles_degrees"]))
        Self.assertLess(Result["error_cm"], 1e-4)
        Self.assertLess(Result["wrist_pitch_error_deg"], 1e-4)
        Self.assertFalse(Bus.writes)

    def TestFixedJointRangesCanHoldAnExactlyReachablePose(Self):
        Controller, _ = MakeController()
        Current = Controller._current_ik_angles()
        Target = Cad.Positions(Controller.arm_geometry, Current)[-1]
        Options = dict(initial_angles=Current, wrist_pitch_deg=Cad.Pitch(Controller.arm_geometry, Current),
                       joint_limits={Name: (Current[Name], Current[Name]) for Name in Cad.SolveNames})
        Result = Cad.Solve(Controller.arm_geometry, *Target.values(), **Options)
        Self.assertTrue(Result["reachable"])
        Bad = {**Target, "z": Target["z"] + 1}
        Self.assertFalse(Cad.Solve(Controller.arm_geometry, *Bad.values(), **Options)["reachable"])

    def TestRimEntryPlannerProducesCheckedDescent(Self):
        Controller, Bus = MakeController()
        Preview = Controller.preview_ik({**InteriorTarget, "wrist_pitch": InteriorPitch})
        Goal = Controller.QuantizeCartesianPose(Preview["angles"], InteriorTarget, InteriorPitch)[0]
        Path = Controller.BasketEntryRoute(InteriorTarget, InteriorPitch, Goal)
        World = Controller.CollisionWorld()
        Check, Tips = Controller.SolverSnapshot(), []
        for Raw in Path:
            Check.ValidateMotionPath(Raw)
            for Name, Value in Raw.items():
                Check.servos[Name].present_raw = Value
            Tips.append(Cad.Positions(Controller.arm_geometry, Check._current_ik_angles())[-1])
        Self.assertTrue(any(Tip["z"] > World.Bounds[1, 2] + .2 for Tip in Tips))
        Self.assertEqual({Name: Path[-1][Name] for Name in Goal}, Goal)
        Self.assertFalse(Bus.writes)
        # Its waypoints get the waypoint reach wherever it certifies, as every other route's do: they all used to stay
        # tight, each approached slowly enough to stop inside the plain allowance.
        Self.assertLess(len(Path.Tight | Path.Arrive), len(Path) // 2)
        Reached, Displaced = Controller.SolverSnapshot(), set()
        Previous = {Name: Servo.present_raw for Name, Servo in Controller.servos.items()}
        for Index, Raw in enumerate(Path):
            Reached.ValidateMotionPath(Raw, Reach=Controller.RouteReachRaw(Path.Tight, len(Path), Index, Displaced, Path.Arrive, Path.Reach))
            Displaced = Controller.RouteDisplaced(Displaced, Previous, Raw, Index in Path.Tight or Index in Path.Arrive,
                                                  Path.Reach.get(Index))
            for Name, Value in Raw.items():
                Reached.servos[Name].present_raw = Value
            Previous = Raw

    def TestValidInteriorPoseIsReachedAndCompletesWithIndependentServos(Self):
        Controller, Bus = MakeController()
        Preview = Controller.preview_ik({**InteriorTarget, "wrist_pitch": InteriorPitch})
        Self.assertTrue(Preview["reachable"], Preview)
        Self.assertIsNone(Preview["collision_reason"])
        World = Controller.CollisionWorld()
        Start = Controller._current_ik_angles()
        Goal = Preview["angles"]
        # The point is clear of both the basket and the mast, but its direct
        # unsynchronized sweep is blocked. Send must still find a checked route.
        Self.assertIsNotNone(World.EdgeReason(Np.array([Start[N] for N in Cad.JointNames]), Np.array([Goal[N] for N in Cad.JointNames])))
        Ok, Message = Controller.send_ik(Target={**InteriorTarget, "wrist_pitch": InteriorPitch})
        Self.assertTrue(Ok, Message)
        Path = Controller.MotionRoute["Path"]
        Check = Controller.SolverSnapshot()
        for Raw in Path:
            Check.ValidateMotionPath(Raw)
            for Name, Value in Raw.items():
                Check.servos[Name].present_raw = Value
        Points = []
        for Raw in Path:
            Angles = {Name: Controller._ik_angle_from_raw(Controller.servos[Name], Value) for Name, Value in Raw.items()}
            Points.append(Cad.Positions(Controller.arm_geometry, Angles)[-1])
        Self.assertTrue(any(Point["z"] > World.Bounds[1, 2] + .2 for Point in Points))
        Controller.last_refresh_s = Bridge.time.monotonic()  # Resume the fixture's feedback clock after offline planning.
        for _ in range(12000):
            for Index, Name in enumerate(Cad.JointNames):
                Bus.step(Name, steps_per_tick=3 + Index)
            Controller.refresh()
            Self.assertIsNone(Controller.MotionFault)
            if Controller.MotionRoute is None:
                break
        Self.assertIsNone(Controller.MotionRoute)
        Final = Controller._current_ik_angles()
        Tip = Cad.Positions(Controller.arm_geometry, Final)[-1]
        # Servo settling tolerance is wider than the commanded IK quantization tolerance.
        Self.assertLess(Math.dist(tuple(Tip.values()), tuple(InteriorTarget.values())), .8)
        Self.assertEqual(Controller.target_cm, InteriorTarget)
        Self.assertEqual(Controller.target_wrist_pitch_deg, InteriorPitch)
        Self.assertTrue(Bus.torque_enabled)

    def TestNoMotionForMaterialContactOrImpossiblePitch(Self):
        Controller, Bus = MakeController()
        Offset = Bridge.Collision.Model["Mounts"]["Right"]["PanCm"]
        Floor = dict(zip(("x", "y", "z"), Np.array([0, 0, 68.5]) - Offset))
        Self.assertFalse(Controller.send_ik(Target={**Floor, "wrist_pitch": -90})[0])
        Self.assertFalse(Controller.send_ik(Target={"x": 500, "y": 0, "z": 0, "wrist_pitch": 0})[0])
        Self.assertFalse(Bus.writes)


    def TestOutOfReachPreviewFollowsTheTargetInsteadOfBackingAway(Self):
        """Dragging an unreachable target closer must never move the shown tip away from it.

        The pose solve may trade tip position for wrist pitch. Out of reach that
        made the preview retreat from a target being dragged toward it, which
        reads as the arm refusing to follow, so out of reach the nearest tip wins.
        """
        Controller, Bus = MakeController()
        Previous = None
        for Step in range(26):
            Target = {"x": 0.0, "y": 70.0 - Step * 1.5, "z": 12.0}
            Preview = Controller._solve_preview(Target, -20.0, None)
            Point = Cad.Positions(Controller.arm_geometry, Preview["angles"])[-1]
            Point = tuple(Point[Axis] for Axis in ("x", "y", "z"))
            Coordinates = tuple(Target[Axis] for Axis in ("x", "y", "z"))
            if Previous is not None:
                # The new pose is at least as close to the new target as the pose
                # the operator was already looking at.
                Self.assertLessEqual(Math.dist(Point, Coordinates), Math.dist(Previous, Coordinates) + 1e-6,
                                     f"Preview backed away from the target at y={Target['y']}")
            Previous = Point
        Self.assertTrue(Preview["reachable"])
        Self.assertFalse(Bus.writes)

    def TestOutOfReachPreviewGetsAsCloseAsTheArmCan(Self):
        """The shown pose is the closest the tip can get, not merely a stopped solve."""
        Controller, Bus = MakeController()
        Target = {"x": 0.0, "y": 46.0, "z": 12.0}
        Preview = Controller._solve_preview(Target, -20.0, None)
        Self.assertFalse(Preview["reachable"])
        Point = Cad.Positions(Controller.arm_geometry, Preview["angles"])[-1]
        Error = Math.dist(tuple(Point[Axis] for Axis in ("x", "y", "z")), (0.0, 46.0, 12.0))
        # Measured: the pitch-weighted solve stopped 3.35 cm out where the arm
        # can stand 2.5 cm off the request.
        Self.assertLess(Error, 2.5)
        Self.assertFalse(Bus.writes)

    def TestNeighbouringServoStepRescuesARoundingRefusal(Self):
        """22:49: the solve missed by 0.236 deg; rounding each joint on its own made that 0.296 deg and the move was
        refused, while one servo step over on two joints was 0.063 cm / 0.120 deg, inside both tolerances."""
        Controller, Bus = ControllerAt(Raws2249)
        Current = Controller._current_ik_angles()
        Candidate = next(Candidate for Candidate in Cad.SolveCandidates(
            Controller.arm_geometry, *(Target2249[Axis] for Axis in "xyz"), elbow=Controller.elbow_mode,
            joint_limits=Controller._ik_joint_limits(), initial_angles=Current, preferred_angles=Current,
            wrist_pitch_deg=Pitch2249, max_iterations=70) if Candidate["reachable"])
        Nearest = {Name: Controller._ik_raw_from_angle(Controller.servos[Name], Candidate["joint_angles_degrees"][Name])
                   for Name in Bridge.IK_JOINTS}
        Self.assertGreater(PoseErrors(Controller, Nearest, Target2249, Pitch2249)[1], 0.25,
                           "the fixture no longer reproduces the 22:49 rounding refusal")
        Targets, Quantized = Controller.QuantizeCartesianPose(Candidate["joint_angles_degrees"], Target2249, Pitch2249)
        for Name in Bridge.IK_JOINTS:
            Self.assertLessEqual(Controller._circular_raw_distance(Targets[Name], Nearest[Name]), 1, Name)
            Self.assertTrue(Controller._contains_raw(Controller.servos[Name], Targets[Name]), Name)
        Error, PitchError = PoseErrors(Controller, Targets, Target2249, Pitch2249)
        Self.assertLessEqual(Error, Bridge.IK_SEND_MAX_ERROR_CM)
        Self.assertLessEqual(PitchError, 0.25)
        Self.assertEqual(Quantized, {**Current, **{Name: Controller._ik_angle_from_raw(Controller.servos[Name], Raw)
                                                   for Name, Raw in Targets.items()}})
        # The whole send path accepts it, and the route ends on exactly that servo target.
        Path = Controller.SolverSnapshot().PlanCartesianMotion(Target2249, Pitch2249)
        Self.assertEqual({Name: Path[-1][Name] for Name in Bridge.IK_JOINTS}, Targets)
        Self.assertFalse(Bus.writes)

    def TestNearestRoundingIsKeptWhenItFits(Self):
        """The neighbour search runs only when rounding misses, so a target that plans today keeps its endpoint."""
        Controller, _ = MakeController()
        Limits = Controller._ik_joint_limits()
        Generator, Checked = Random.Random(7), 0
        for _ in range(40):
            Angles = {**Controller._current_ik_angles(), **{Name: Generator.uniform(*Limits[Name]) for Name in Bridge.IK_JOINTS}}
            Tip = Bridge.forward_tip_position(Controller.arm_geometry, Angles)
            Pitch = Bridge.wrist_tip_pitch_degrees(Controller.arm_geometry, Angles)
            Nearest = {Name: Controller._ik_raw_from_angle(Controller.servos[Name], Angles[Name]) for Name in Bridge.IK_JOINTS}
            Error, PitchError = PoseErrors(Controller, Nearest, Tip, Pitch)
            if Error <= Bridge.IK_SEND_MAX_ERROR_CM and PitchError <= 0.25:
                Self.assertEqual(Controller.QuantizeCartesianPose(Angles, Tip, Pitch)[0], Nearest)
                Checked += 1
        Self.assertGreater(Checked, 30)

    def TestTargetsNoServoStepCanHoldAreStillRefused(Self):
        Controller, _ = MakeController()
        Angles = Controller._current_ik_angles()
        Tip = Bridge.forward_tip_position(Controller.arm_geometry, Angles)
        Pitch = Bridge.wrist_tip_pitch_degrees(Controller.arm_geometry, Angles)
        with Self.assertRaisesRegex(ValueError, "Quantized servo target exceeds the CAD IK position tolerance"):
            Controller.QuantizeCartesianPose(Angles, {**Tip, "z": Tip["z"] + 0.5}, Pitch)
        with Self.assertRaisesRegex(ValueError, "Quantized servo target exceeds the CAD IK pitch tolerance"):
            Controller.QuantizeCartesianPose(Angles, Tip, Pitch + 1.0)

    def TestIkRefusalNamesTheMissAndAReachableWristPitch(Self):
        """07:58: the app forced the wrist to -43.575 deg, the edge of its own tolerance band. The bridge refused with
        the bare 'IK target is unreachable at this wrist angle', which gave the app nothing to correct with."""
        Controller, Bus = ControllerAt(Raws0758)
        Ok, Message = Controller.send_ik(Target={**Target0758, "wrist_pitch": Pitch0758})
        Self.assertFalse(Ok)
        Self.assertTrue(Message.startswith("IK target is unreachable at this wrist angle"), Message)
        Miss = Re.search(r"misses by ([\d.]+) cm and ([\d.]+) deg", Message)
        Self.assertIsNotNone(Miss, Message)
        Self.assertAlmostEqual(float(Miss[1]), 0.043, delta=0.005)
        Self.assertAlmostEqual(float(Miss[2]), 0.32, delta=0.01)
        Reachable = Re.search(r"wrist (-?[\d.]+) deg reaches this point", Message)
        Self.assertIsNotNone(Reachable, Message)
        Pitch = float(Reachable[1])
        # On the reachable side, near the request, and with room to spare: the bridge's own edge is -43.45 deg.
        Self.assertTrue(-43.2 <= Pitch <= -40.0, Message)
        Self.assertTrue(Controller.SolverSnapshot().PlanCartesianMotion(Target0758, Pitch))
        Self.assertFalse(Bus.writes)

    def TestOutOfReachRefusalSaysHowCloseTheTipGets(Self):
        Controller, Bus = MakeController()
        Ok, Message = Controller.send_ik(Target={"x": 0.0, "y": 46.0, "z": 12.0, "wrist_pitch": -20.0})
        Self.assertFalse(Ok)
        Self.assertTrue(Message.startswith("IK target is unreachable at this wrist angle"), Message)
        Self.assertRegex(Message, r"the tip gets no closer than [\d.]+ cm")
        Self.assertNotIn("reaches this point", Message)
        Self.assertFalse(Bus.writes)

    def TestHealthNamesTheLoadedSolverSource(Self):
        """2026-09-29: the bridge ran HEAD's solver for hours after CadKinematics.py was rewritten, and neither /health
        nor the app could tell. The source hash tells them apart without changing the app's compatibility check."""
        Loaded = Hashlib.sha256(Path(Cad.__file__).read_bytes()).hexdigest()[:12]
        Health = Bridge.health()
        Self.assertEqual(Health["ik_solver_source"], Loaded)
        Self.assertFalse(Health["ik_solver_source_stale"])
        Self.assertEqual(Health["ik_solver_version"], Cad.SolverVersion)
        Self.assertEqual(MakeController()[0].state_snapshot()["ik_solver_source"], Loaded)
        with Patch.object(Bridge, "IkSolverSource", "000000000000"):
            Self.assertTrue(Bridge.health()["ik_solver_source_stale"])


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestCartesianReachability` runs the Test* methods too (unittest names this hook)."""
    Prefixed = Unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return Prefixed.loadTestsFromTestCase(CartesianTests)


if __name__ == "__main__":
    Loader = Unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Result = Unittest.TextTestRunner(verbosity=2).run(Loader.loadTestsFromTestCase(CartesianTests))
    raise SystemExit(not Result.wasSuccessful())
