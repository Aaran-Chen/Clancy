"""CAD FK, IK, calibration and frontend parity fixtures; no hardware access."""
import copy as Copy
import json as Json
import math as Math
from pathlib import Path
import sys as Sys
import time as Time
import unittest as Unittest
from unittest.mock import patch as Patch

import numpy as Np

import CadKinematics as Cad
import arm_geometry as Arm
import bridge_server as Bridge

Output = Path(__file__).with_name("TestArtifacts")
Output.mkdir(exist_ok=True)
Geometry = Arm.load_arm_geometry()
Statistics = {}


class CadKinematicsTests(Unittest.TestCase):
    def TestShoulderPanEncoderDirectionAndInverse(Self):
        Controller = Bridge.BridgeController()
        Self.assertIsNone(Controller.bus)
        Servo = Controller.servos["shoulder_pan"]
        Reference = Controller.arm_geometry["CadAlignment"]["ReferenceRaw"]["shoulder_pan"]
        Initial = Controller._current_ik_angles()
        ZeroRadians = Cad.ToUrdf(Initial, Controller.arm_geometry["CadAlignment"])
        for Delta in (-128, 128, 700):
            Raw = (Reference + Delta) % Bridge.RESOLUTION
            Angles = {**Initial, "shoulder_pan": Controller._ik_angle_from_raw(Servo, Raw)}
            Radians = Cad.ToUrdf(Angles, Controller.arm_geometry["CadAlignment"])
            Expected = {**ZeroRadians, "shoulder_pan": Math.radians(
                Controller.arm_geometry["CadAlignment"]["UrdfAtReferenceDegrees"]["shoulder_pan"] + Delta * 360 / Bridge.RESOLUTION)}
            Self.assertAlmostEqual(Radians["shoulder_pan"], Expected["shoulder_pan"], places=10)
            ExpectedLinks, _ = Cad.ForwardUrdf(Expected)
            Target = Cad.Point(ExpectedLinks[Cad.Model["ToolLink"]][:3, 3])
            Np.testing.assert_allclose(list(Cad.Positions(Controller.arm_geometry, Angles)[-1].values()), list(Target.values()), atol=1e-9)
            Result = Cad.Solve(Controller.arm_geometry, Target["x"], Target["y"], Target["z"],
                               joint_limits=Controller._ik_joint_limits(), initial_angles=Angles,
                               wrist_pitch_deg=Math.degrees(Cad.PitchUrdf(Expected)))
            Self.assertTrue(Result["reachable"], Result)
            Self.assertEqual(Controller._ik_raw_from_angle(Servo, Result["joint_angles_degrees"]["shoulder_pan"]), Raw)

    def TestRandomFkIkRoundTrips(Self):
        Generator = Np.random.default_rng(101)
        Fixtures = []
        Maximum, MaximumPitch = 0.0, 0.0
        Started = Time.perf_counter()
        for Index in range(120):
            Radians = {Joint["Name"]: float(Generator.uniform(float(Joint["Limits"]["lower"]) * .9,
                                                              float(Joint["Limits"]["upper"]) * .9))
                       for Joint in Cad.OrderedJoints if Joint["Type"] != "fixed"}
            Angles = Cad.FromUrdf(Radians, Geometry["CadAlignment"])
            Target = Cad.Positions(Geometry, Angles)[-1]
            Pitch = Cad.Pitch(Geometry, Angles)
            Initial = {**dict.fromkeys(Cad.JointNames, 0.0), **{Name: Angles[Name] for Name in Cad.JointNames[4:]}}
            Result = Arm.solve_tip_ik(Geometry, Target["x"], Target["y"], Target["z"], initial_angles=Initial,
                                      wrist_pitch_deg=Pitch, elbow="up" if Index % 2 else "down")
            Self.assertTrue(Result["reachable"], (Index, Result))
            Actual = Cad.Positions(Geometry, Result["joint_angles_degrees"])[-1]
            Error = Math.dist(list(Target.values()), list(Actual.values()))
            Self.assertLess(Error, 1e-4)
            Self.assertLess(Result["wrist_pitch_error_deg"], 1e-4)
            for Name in Cad.JointNames[4:]:
                Self.assertAlmostEqual(Result["joint_angles_degrees"][Name], Initial[Name], places=10)
            Maximum = max(Maximum, Error)
            MaximumPitch = max(MaximumPitch, Result["wrist_pitch_error_deg"])
            Fixtures.append({"Angles": Angles, "Initial": Initial, "Target": Target, "Pitch": Pitch,
                             "Elbow": "up" if Index % 2 else "down", "Joints": Cad.Positions(Geometry, Angles)})
        Statistics.update(PoseCount=len(Fixtures), MaximumErrorMm=Maximum * 10, MaximumPitchErrorDegrees=MaximumPitch,
                          TotalSolveSeconds=Time.perf_counter() - Started)
        (Output / "CadFixtures.json").write_text(Json.dumps(Fixtures, indent=2))

    def TestLimitsUnreachableAndInvalidInput(Self):
        Limits = {Name: (-5, 5) for Name in Cad.SolveNames}
        Target = Cad.Positions(Geometry, {"shoulder_lift": 40})[-1]
        Result = Cad.Solve(Geometry, **dict(zip(("X", "Y", "Z"), Target.values())), joint_limits=Limits, wrist_pitch_deg=-15)
        Self.assertFalse(Result["reachable"])
        for Name in Cad.SolveNames:
            Self.assertGreaterEqual(Result["joint_angles_degrees"][Name], -5 - 1e-8)
            Self.assertLessEqual(Result["joint_angles_degrees"][Name], 5 + 1e-8)
        Self.assertFalse(Cad.Solve(Geometry, 1000, 1000, 1000, wrist_pitch_deg=0)["reachable"])
        for Bad in (float("nan"), float("inf")):
            with Self.assertRaises(ValueError): Cad.Solve(Geometry, Bad, 10, 5, wrist_pitch_deg=0)

    def TestConfiguredEncoderLimitsAndQuantization(Self):
        Controller = Bridge.BridgeController()
        Generator = Np.random.default_rng(3215)
        Limits = Controller._ik_joint_limits()
        Maximum = 0.0
        Fixtures = []
        for Index in range(60):
            Angles = {Name: float(Generator.uniform(*Limits[Name])) for Name in Cad.SolveNames}
            Angles.update(wrist_roll=float(Generator.uniform(-150, 150)), gripper=20.0)
            Target = Cad.Positions(Geometry, Angles)[-1]
            Pitch = Cad.Pitch(Geometry, Angles)
            Initial = {**dict.fromkeys(Cad.JointNames, 0.0), **{Name: Angles[Name] for Name in Cad.JointNames[4:]}}
            Result = Cad.Solve(Geometry, Target["x"], Target["y"], Target["z"], joint_limits=Limits,
                               initial_angles=Initial, wrist_pitch_deg=Pitch)
            Self.assertTrue(Result["reachable"], (Index, Result))
            Self.assertLess(Result["error_cm"], 1e-4)
            Rounded = dict(Result["joint_angles_degrees"])
            for Name in Cad.SolveNames:
                Self.assertGreaterEqual(Rounded[Name], Limits[Name][0] - 1e-8)
                Self.assertLessEqual(Rounded[Name], Limits[Name][1] + 1e-8)
                Servo = Controller.servos[Name]
                Rounded[Name] = Controller._ik_angle_from_raw(Servo, Controller._ik_raw_from_angle(Servo, Rounded[Name]))
            Error = Math.dist(tuple(Target.values()), tuple(Cad.Positions(Geometry, Rounded)[-1].values()))
            Self.assertLess(Error, Bridge.IK_SEND_MAX_ERROR_CM)
            Maximum = max(Maximum, Error)
            Fixtures.append({"Angles": Angles, "Initial": Initial, "Target": Target, "Pitch": Pitch,
                             "Elbow": "up", "Limits": Limits, "Joints": Cad.Positions(Geometry, Angles)})
        Statistics.update(EncoderLimitPoseCount=len(Fixtures), MaximumQuantizedErrorMm=Maximum * 10)
        (Output / "CadBoundedFixtures.json").write_text(Json.dumps(Fixtures, indent=2))

    def TestRecordReferenceReadsOnlyAndKeepsWorkspaceAnchor(Self):
        from TestSafety import MakeController
        Controller, Bus = MakeController()
        Controller.torque_enabled = Bus.torque_enabled = False
        Anchor = Copy.deepcopy(Controller.arm_geometry["CadAlignment"]["AnchorRaw"])
        Destination = Output / "AlignmentCapture"
        Destination.mkdir(exist_ok=True)
        with Patch.object(Bridge, "BACKEND_ROOT", Destination):
            Result = Controller.CaptureCadReference(True)
        Self.assertTrue(Result["ok"], Result)
        Self.assertFalse(Bus.writes)
        Self.assertEqual(Result["alignment"]["AnchorRaw"], Anchor)
        Self.assertTrue(Result["alignment"]["Verified"])
        Self.assertTrue((Destination / "CadAlignmentPrevious.json").is_file())
        Self.assertEqual(Result["alignment"]["ReferenceRaw"], {Name: Servo.present_raw for Name, Servo in Controller.servos.items()})
        Links, _ = Cad.Forward(Controller.arm_geometry, Controller._current_ik_angles())
        Expected, _ = Cad.ForwardUrdf({})
        Np.testing.assert_allclose(Links[Cad.Model["ToolLink"]], Expected[Cad.Model["ToolLink"]], atol=1e-9)

    def TestEveryArmJointTurnsTheCadTheWayItsServoTurns(Self):
        # One raw step up turns every arm joint's URDF angle the same way. wrist_roll's arc is
        # inverted in gui_limits.json, so its CadAlignment Direction must be -1 like the others'
        # (with +1 the drawn gripper sat ~189 deg off at Home and the fixed jaw swapped sides).
        Controller = Bridge.BridgeController()
        Alignment = Controller.arm_geometry["CadAlignment"]
        for Name in Cad.JointNames[:5]:
            Servo = Controller.servos[Name]
            Raw = Alignment["ReferenceRaw"][Name]
            Urdf = [Cad.ToUrdf({Name: Controller._ik_angle_from_raw(Servo, (Raw + Step) % Bridge.RESOLUTION)}, Alignment)[Name]
                    for Step in (0, 16)]
            Self.assertAlmostEqual(Urdf[1] - Urdf[0], Math.radians(16 * 360 / Bridge.RESOLUTION), places=9, msg=Name)

    def TestRollMovesActualToolPointAndJawDoesNot(Self):
        First = Cad.Positions(Geometry, {})[-1]
        Rolled = Cad.Positions(Geometry, {"wrist_roll": 180})[-1]
        Tool = Cad.Origins["gripper_frame_joint"][:3, 3]
        Self.assertAlmostEqual(Math.dist(list(First.values()), list(Rolled.values())), float(2 * Np.linalg.norm(Tool[:2])), places=10)
        Self.assertEqual(First, Cad.Positions(Geometry, {"gripper": 30})[-1])

    def TestSavedHomeCannotChangeCalibration(Self):
        Controller = Bridge.BridgeController()
        Before = Controller._current_ik_angles()
        Offset = Controller.WorkspaceOffset()
        Controller.home_preset = {Name: (Value + 100) % Bridge.RESOLUTION for Name, Value in Controller.home_preset.items()}
        Self.assertEqual(Before, Controller._current_ik_angles())
        Self.assertEqual(Offset, Controller.WorkspaceOffset())

    def TestBridgePreviewIncludesHeldRoll(Self):
        Controller = Bridge.BridgeController()
        Controller.servos["wrist_roll"].present_raw -= 150
        Current = Controller._current_ik_angles()
        Self.assertIsNone(Controller.CollisionWorld().PoseReason(Current))
        Target = Cad.Positions(Controller.arm_geometry, Current)[-1]
        Result = Controller.preview_ik({**Target, "wrist_pitch": Cad.Pitch(Controller.arm_geometry, Current)})
        Self.assertTrue(Result["reachable"], Result)
        Self.assertAlmostEqual(Current["wrist_roll"], Result["angles"]["wrist_roll"])

    def TestAlignmentCannotBeCapturedWithoutFeedback(Self):
        Controller = Bridge.BridgeController()
        Before = Copy.deepcopy(Controller.arm_geometry)
        Self.assertFalse(Controller.CaptureCadReference(True)["ok"])
        Self.assertFalse(Controller.CaptureCadReference(False)["ok"])
        Self.assertEqual(Before, Controller.arm_geometry)

    def TestExportedSourceMatchesUrdf(Self):
        Sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Cad"))
        from ReferenceKinematics import LoadModel, ForwardKinematics
        Original = LoadModel()
        Radians = dict(zip(Cad.JointNames, (.2, -.5, .3, -.8, .7, .4)))
        OriginalLinks, OriginalFrames = ForwardKinematics(Original, Radians)
        Links, Frames = Cad.ForwardUrdf(Radians)
        for Name, Value in OriginalLinks.items():
            Converted = Value.copy(); Converted[:3, 3] *= 100
            Np.testing.assert_allclose(Links[Name], Cad.BaseTransform @ Converted, atol=1e-10)


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestCadKinematics` runs the Test* methods too (unittest names this hook)."""
    Prefixed = Unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return Prefixed.loadTestsFromTestCase(CadKinematicsTests)


if __name__ == "__main__":
    Loader = Unittest.TestLoader(); Loader.testMethodPrefix = "Test"
    Result = Unittest.TextTestRunner(verbosity=2).run(Loader.loadTestsFromTestCase(CadKinematicsTests))
    Statistics.update(Successful=Result.wasSuccessful(), TestsRun=Result.testsRun)
    (Output / "CadVerification.json").write_text(Json.dumps(Statistics, indent=2))
    print(Json.dumps(Statistics, indent=2))
    raise SystemExit(not Result.wasSuccessful())
