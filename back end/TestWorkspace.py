"""Workspace translation and calibration sampling checks, without motor access."""
import unittest
import math
from contextlib import ExitStack
from unittest.mock import patch

import bridge_server as Bridge
import SelfCalibration as SC


class WorkspaceTests(unittest.TestCase):
    @classmethod
    def setUpClass(Class):
        Class.Controller = Bridge.BridgeController()
        Class.Result = Class.Controller.selfcal_poses()

    def TestRobotOriginAndCoordinateRoundTrip(Self):
        Controller = Self.Controller
        Frame = Bridge._load_json(Bridge.BACKEND_ROOT / "WorkspaceFrame.json")
        RobotFrame = Frame["RobotFrame"]
        Pan = Controller.VisualPointFromModelPoint(Bridge.forward_joint_positions(Controller.arm_geometry, {})[0])
        for Axis in ("x", "y", "z"):
            Self.assertAlmostEqual(Pan[Axis], RobotFrame["ArmBasePositionCm"][Axis])
        Self.assertAlmostEqual(Pan["y"], 0)
        Self.assertGreater(Pan["x"], 0)
        Self.assertGreater(Pan["z"], 0)
        Self.assertAlmostEqual(SC.DefaultTableZCm, Frame["TableZCm"])
        Forward = Controller.VisualPointFromModelPoint({"x": 0, "y": 10, "z": 0})
        Self.assertGreater(Forward["y"], Pan["y"])
        for Point in ({"x": -12, "y": 11, "z": -3}, {"x": 12, "y": 23, "z": 5}, {"x": 0, "y": 0, "z": 0}):
            RoundTrip = Controller.VisualPointFromModelPoint(Controller.ModelPointFromVisualPoint(Point))
            for Axis in Point:
                Self.assertAlmostEqual(RoundTrip[Axis], Point[Axis])

    def TestSpacedReachableSafePosesInsideRequestedRegion(Self):
        Controller = Self.Controller
        Self.assertIsNone(Controller.bus)
        Self.assertTrue(Self.Result["ok"], Self.Result)
        Poses = Self.Result["poses"]
        Sampling = Bridge._load_json(Bridge.BACKEND_ROOT / "WorkspaceFrame.json")["CalibrationSampling"]
        Self.assertEqual(len(Poses), 24)
        Path = Controller.SolverSnapshot()
        Achieved = []
        Signatures = set()
        for Pose in Poses:
            Plan = Controller.CalibrationPlans[Pose["plan_id"]]
            Targets = {Name: Controller._ik_raw_from_angle(Controller.servos[Name], Plan["angles"][Name]) for Name in Bridge.IK_JOINTS}
            Signature = tuple(Targets[Name] for Name in Bridge.IK_JOINTS)
            Self.assertNotIn(Signature, Signatures)
            Signatures.add(Signature)
            Angles = {**Controller._current_ik_angles(),
                      **{Name: Controller._ik_angle_from_raw(Controller.servos[Name], Raw) for Name, Raw in Targets.items()}}
            Actual = Controller.VisualPointFromModelPoint(Bridge.forward_tip_position(Controller.arm_geometry, Angles))
            for Point in (Actual, Pose["visual_target"]):
                Self.assertLessEqual(Point["x"], Sampling["MaximumXCm"])
                Self.assertGreaterEqual(Point["x"], Sampling["MinimumXCm"])
                Self.assertGreater(Point["y"], Sampling["MinimumYCm"])
                Self.assertGreaterEqual(Point["z"], min(Sampling["ZValuesCm"]))
                Self.assertLessEqual(Point["z"], max(Sampling["ZValuesCm"]))
            Stages = Path.PlanMotionRoute(Targets)
            for Stage in Stages:
                Path.ValidateMotionPath(Stage)
                for Name, Raw in Stage.items():
                    Path.servos[Name].present_raw = Raw
            for Name, Raw in Targets.items():
                Path.servos[Name].present_raw = Raw
            for Previous in Achieved:
                Self.assertGreaterEqual(math.dist(tuple(Actual[Axis] for Axis in ("x", "y", "z")),
                                                 tuple(Previous[Axis] for Axis in ("x", "y", "z"))), 2.5)
            Achieved.append(Actual)
        Self.assertGreaterEqual(len({Pose["visual_target"]["z"] for Pose in Poses}), 3)
        Self.assertGreaterEqual(len({Pose["visual_target"]["x"] for Pose in Poses}), 3)
        print("Verified achieved coordinate ranges:", {Axis: [min(P[Axis] for P in Achieved), max(P[Axis] for P in Achieved)] for Axis in ("x", "y", "z")})
        print("Sampling diagnostics:", Self.Result["sampling"])

    def TestFrameChangeInvalidatesOldPlans(Self):
        Controller = Self.Controller
        Original = Controller.ConfigurationKey()
        Frame = Bridge._load_json(Bridge.BACKEND_ROOT / "WorkspaceFrame.json")
        Frame["RobotFrame"]["ArmBasePositionCm"]["x"] += 1
        with patch.object(Bridge.WorkspaceFrameFile, "Data", return_value=Frame):
            Self.assertNotEqual(Controller.ConfigurationKey(), Original)

    def TestBothXCutoffsExcludeCandidatesBeforeSolving(Self):
        Controller = Self.Controller
        Frame = Bridge._load_json(Bridge.BACKEND_ROOT / "WorkspaceFrame.json")
        Frame["CalibrationSampling"].update(XValuesCm=[-26, -25, -16, -15, 0, 15, 16], YValuesCm=[11], ZValuesCm=[0], WristPitchDegrees=[-40], MinimumXCm=-25, MaximumXCm=15, MinimumYCm=10)
        Offset = Controller.WorkspaceOffset()
        with patch.object(Bridge.WorkspaceFrameFile, "Data", return_value=Frame), patch.object(Bridge, "solve_tip_ik", return_value={"reachable": False}) as Solve:
            Controller.selfcal_poses()
        Sampled = [round(Call.args[1] - Offset["x"], 6) for Call in Solve.call_args_list]
        Self.assertEqual(Sampled, [-25, -16, -15, 0, 15])

    def TestExpandedYGridReachesSolver(Self):
        Controller = Self.Controller
        Offset = Controller.WorkspaceOffset()
        with patch.object(Bridge, "solve_tip_ik", return_value={"reachable": False}) as Solve:
            Controller.selfcal_poses()
        SampledY = {round(Call.args[2] + Offset["y"], 6) for Call in Solve.call_args_list}
        Sampling = Bridge._load_json(Bridge.BACKEND_ROOT / "WorkspaceFrame.json")["CalibrationSampling"]
        Self.assertEqual(SampledY, {round(Value, 6) for Value in Sampling["YValuesCm"]})
        Self.assertTrue({round(Value, 6) for Value in Sampling["PreferredYRangeCm"]}.issubset(SampledY))

    def TestFinalSelectionPrefersFarYAndRemovesRepeatedPositions(Self):
        Controller = Bridge.BridgeController()
        Candidates = []
        # Nearby candidates are cheapest to visit, but must not crowd out the
        # requested far-Y band. A different wrist angle at the same XYZ is a repeat.
        for Index, (X, Y) in enumerate(((0, 15), (0, 15), (0, 27), (0, 27), (4, 30), (8, 33), (8, 15))):
            Candidates.append({"actual_visual": {"x": X, "y": Y, "z": 6},
                               "angles": dict.fromkeys(Bridge.IK_JOINTS, Index)})
        Sampling = {"PoseCount": 4, "MinimumSpacingCm": 2.5,
                    "PreferredYRangeCm": [27, 33], "PreferredPoseCount": 3}
        with patch.object(Controller, "PlanMotionRoute") as Validate:
            Order = Controller.SelectCalibrationCoverage(Candidates, dict.fromkeys(Bridge.IK_JOINTS, 0), Controller, Sampling)
        Self.assertEqual(len(Order), 4)
        Self.assertEqual(sum(27 <= Candidates[Index]["actual_visual"]["y"] <= 33 for Index in Order[:3]), 3)
        Self.assertEqual(len({tuple(Candidates[Index]["actual_visual"].values()) for Index in Order}), 4)
        Self.assertEqual(Validate.call_count, 4)

    def TestSelectionCoversBothBandsAndBalancesWristPitch(Self):
        Controller = Bridge.BridgeController()
        Candidates = []
        # Least travel alone would stay in the band and on pitch -12, whose angles sit next to the start.
        for Pitch, Base in ((-12, 0), (8, 100)):
            for Index, Y in enumerate((30, 32, 34, 36, 18, 20, 22, 24)):
                Candidates.append({"actual_visual": {"x": 3 * Index + (1 if Pitch == 8 else 0) * 40, "y": Y, "z": 6},
                                   "wrist_pitch": Pitch, "angles": dict.fromkeys(Bridge.IK_JOINTS, Base + Index)})
        Sampling = {"PoseCount": 8, "MinimumSpacingCm": 2.5, "PreferredYRangeCm": [28, 40], "PreferredPoseCount": 4}
        with patch.object(Controller, "PlanMotionRoute"):
            Order = Controller.SelectCalibrationCoverage(Candidates, dict.fromkeys(Bridge.IK_JOINTS, 0), Controller, Sampling)
        Chosen = [Candidates[Index] for Index in Order]
        Self.assertEqual(len(Chosen), 8)
        Self.assertTrue(all(28 <= Pose["actual_visual"]["y"] <= 40 for Pose in Chosen[:4]))
        Self.assertTrue(all(Pose["actual_visual"]["y"] < 28 for Pose in Chosen[4:]))
        # Each pitch gets at least half an even share (8 poses, 2 pitches: 2 each).
        Self.assertGreaterEqual(sum(Pose["wrist_pitch"] == 8 for Pose in Chosen), 2)

    def TestPreferredPosesCannotBypassPathGuard(Self):
        Controller = Bridge.BridgeController()
        Candidates = [{"actual_visual": {"x": 0, "y": Y, "z": 6},
                       "angles": dict.fromkeys(Bridge.IK_JOINTS, Index)} for Index, Y in enumerate((30, 15))]
        Sampling = {"PoseCount": 2, "MinimumSpacingCm": 2.5,
                    "PreferredYRangeCm": [27, 33], "PreferredPoseCount": 1}
        with patch.object(Controller, "PlanMotionRoute", side_effect=[ValueError("load"), None, ValueError("load")]):
            Order = Controller.SelectCalibrationCoverage(Candidates, dict.fromkeys(Bridge.IK_JOINTS, 0), Controller, Sampling)
        Self.assertEqual(Order, [1])

    def TestAchievedXUsesConfiguredCutoffs(Self):
        Controller = Bridge.BridgeController()
        Offset = Controller.WorkspaceOffset()
        Frame = Bridge._load_json(Bridge.BACKEND_ROOT / "WorkspaceFrame.json")
        Frame["CalibrationSampling"].update(XValuesCm=[0], YValuesCm=[17], ZValuesCm=[6], WristPitchDegrees=[-40], PoseCount=1, MinimumXCm=-25, MaximumXCm=15, MinimumYCm=10)
        for ActualX, Expected in ((-26, False), (-25, True), (-20, True), (15, True), (16, False)):
            with Self.subTest(ActualX=ActualX), ExitStack() as Stack:
                Stack.enter_context(patch.object(Bridge.WorkspaceFrameFile, "Data", return_value=Frame))
                Stack.enter_context(patch.object(Controller, "WorkspaceOffset", return_value=Offset))
                Stack.enter_context(patch.object(Bridge.BridgeController, "ValidateMotionPath", return_value=None))
                Stack.enter_context(patch.object(Bridge.BridgeController, "PlanMotionRoute", return_value=[]))
                Stack.enter_context(patch.object(SC, "MinObservations", 1))
                # Isolate achieved-coordinate filtering from the separate CAD
                # accuracy checks, since this fixture supplies synthetic FK.
                Stack.enter_context(patch.object(Bridge, "IK_SEND_MAX_ERROR_CM", 100))
                Stack.enter_context(patch.object(Bridge, "wrist_tip_pitch_degrees", return_value=-40))
                Stack.enter_context(patch.object(Bridge, "solve_tip_ik", return_value={
                    "reachable": True, "wrist_pitch_error_deg": 0,
                    "joint_angles_degrees": Controller._current_ik_angles(),
                }))
                Stack.enter_context(patch.object(Bridge, "forward_tip_position", return_value={
                    "x": ActualX + Offset["x"], "y": 17 - Offset["y"], "z": 6 - Offset["z"],
                }))
                Self.assertEqual(Controller.selfcal_poses()["ok"], Expected)


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestWorkspace` runs the Test* methods too (unittest names this hook)."""
    Prefixed = unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return Prefixed.loadTestsFromTestCase(WorkspaceTests)


if __name__ == "__main__":
    Loader = unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Result = unittest.TextTestRunner(verbosity=2).run(Loader.loadTestsFromTestCase(WorkspaceTests))
    raise SystemExit(not Result.wasSuccessful())
