"""Automatic calibration visits the desk where objects sit, whatever pose the arm starts from.

CalibrationSampling is in the bridge's visual (panel) frame, X = -AI/display X. Until 2026-09-29
its window was display X -37.9..+2.1: the arm's right side, mostly over the floor beyond the
desk's right edge (display X ~ -10). Planned from the 22:43 run's start pose it reproduced that
run: display X -34.9..-2.9, no pose over the object area, so every object was an extrapolation.
Planning is offline: no bus, no camera (SO101_DEPTH_CAMERA=0).

Run:  python TestCalibrationCoverage.py
"""
import os
import unittest
from unittest import mock

import bridge_server as Bridge

# Only while this module's tests run: set at import, it leaked into every suite loaded
# after it in one process (TestDepthCamera's fake camera then never started). The camera
# only reads it when started, never on import, so the tests here see the same setting.
Environment = mock.patch.dict(os.environ)


def setUpModule():
    Environment.start()
    os.environ.setdefault("SO101_DEPTH_CAMERA", "0")


def tearDownModule():
    Environment.stop()

# Start poses of real runs (Manifest initial_state): the tape poses follow the least-travel walk
# from wherever the arm stands, so one start is not enough. Even one raw step of wrist roll
# (1513 at home, 1514 at 22:43) changed the old window's plan from display X -30.9..2.1 to -34.9..-2.9.
Starts = {
    "home": None,
    "22:43 run": {"shoulder_pan": 3469, "shoulder_lift": 3168, "elbow_flex": 2685, "wrist_flex": 854, "wrist_roll": 1514, "gripper": 700},
    "15:02 run": {"shoulder_pan": 3341, "shoulder_lift": 3754, "elbow_flex": 2492, "wrist_flex": 539, "wrist_roll": 1554, "gripper": 701},
}
# The tape stays at least as high above the desk as before the window moved: objects stand on
# the desk and the calibration collision model does not know about them.
LowestTapeZCm = 7.139407523131164
FacingXValuesCm = [-2.1006143172419343, 2.8993856827580657, 6.899385682758066, 10.899385682758066, 14.899385682758066,
                   18.899385682758066, 22.899385682758066, 26.899385682758066, 30.899385682758066, 34.899385682758066]


class CoverageTests(unittest.TestCase):
    @classmethod
    def setUpClass(Class):
        Class.Plans = {}
        for Name, Raw in Starts.items():
            Controller = Bridge.BridgeController()
            assert Controller.bus is None
            for Joint, Value in (Raw or {}).items():
                Controller.servos[Joint].present_raw = Value
            Result = Controller.selfcal_poses("Behind")
            Tips = []
            for Pose in Result.get("poses", []):
                Angles = {**Controller._current_ik_angles(), **Controller.CalibrationPlans[Pose["plan_id"]]["angles"]}
                Visual = Controller.VisualPointFromModelPoint(Bridge.forward_tip_position(Controller.arm_geometry, Angles))
                Tips.append({"x": -Visual["x"], "y": Visual["y"], "z": Visual["z"]})  # display = visual with X negated
            Class.Plans[Name] = (Result, Tips)

    def TestEveryStartCoversTheObjectArea(Self):
        for Name, (Result, Tips) in Self.Plans.items():
            with Self.subTest(Start=Name):
                Self.assertTrue(Result["ok"], Result)
                Xs = sorted(round(Tip["x"], 1) for Tip in Tips)
                Summary = f"{Name}: display X {Xs[0]}..{Xs[-1]}"
                print("\n  " + Summary, f"({sum(X > 0 for X in Xs)} poses at X > 0, {sum(X > -10 for X in Xs)} over the desk)")
                Self.assertGreaterEqual(sum(X > 0 for X in Xs), 8, Summary)
                Self.assertGreaterEqual(sum(X > -10 for X in Xs), 12, Summary)
                Self.assertGreaterEqual(Xs[-1], 12.0, Summary)
                Self.assertGreaterEqual(len({Pose["wrist_pitch"] for Pose in Result["poses"]}), 2)

    def TestTheTapeStaysAsHighAboveTheDesk(Self):
        Sampling = Bridge._load_json(Bridge.BACKEND_ROOT / "WorkspaceFrame.json")["CalibrationSampling"]
        Self.assertGreaterEqual(min(Sampling["ZValuesCm"]), LowestTapeZCm)
        for Name, (_, Tips) in Self.Plans.items():
            with Self.subTest(Start=Name):
                Self.assertGreaterEqual(min(Tip["z"] for Tip in Tips), LowestTapeZCm - 0.1)

    def TestTheFacingViewKeepsItsWindow(Self):
        Sampling = Bridge._load_json(Bridge.BACKEND_ROOT / "WorkspaceFrame.json")["CalibrationSampling"]
        Facing = {**Sampling, **Sampling["Facing"]}
        Self.assertEqual(Facing["XValuesCm"], FacingXValuesCm)
        Self.assertEqual((Facing["MinimumXCm"], Facing["MaximumXCm"]), (-2.1006143172419343, 37.899385682758066))
        Self.assertIn("visual", Sampling.get("Frame", ""), "the window's frame is written down next to it")


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestCalibrationCoverage` runs the Test* methods too (unittest names this hook)."""
    Prefixed = unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return Prefixed.loadTestsFromTestCase(CoverageTests)


if __name__ == "__main__":
    Loader = unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Result = unittest.TextTestRunner(verbosity=2).run(Loader.loadTestsFromTestCase(CoverageTests))
    raise SystemExit(not Result.wasSuccessful())
