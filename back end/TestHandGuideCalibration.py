"""The hand-guiding calibration (HandGuideCalibration.py) on the offline plant: the sweeps the bridge runs recover the
plant's own gravity model, frictions and velocity slopes, the spin-down tells a floating driver from a braking one, a
calibration that does not fit is refused (nothing saved), and the law built from an accepted calibration hovers the
plant. Offline only: no serial port, no bridge (HandGuidePlant.py). About 35 s (one process per joint).

    python TestHandGuideCalibration.py
"""
from __future__ import annotations

import copy
import json
import math
import os
import sys
import unittest
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import HandGuide as L  # noqa: E402
import HandGuideCalibration as C  # noqa: E402
import HandGuidePlant as Plant  # noqa: E402

MassScale = 1.1          # the plant weighs 10 % more than the URDF says: the fit must find it
Runs: dict = {}
Fitted: dict = {}


def Calibrate(Args):
    Key, Joint, Spin, Brakes = Args
    return Key, Plant.CalibrateOnPlant((Joint,), Spin, MassScale=MassScale, CoastBrakes=Brakes)


def setUpModule():
    Jobs = [(Joint, Joint, (Joint,) if Joint in C.SpinJoints else (), False) for Joint in C.SweepOrder]
    Jobs += [(f"brakes {Joint}", Joint, (Joint,), True) for Joint in C.SpinJoints]
    with ProcessPoolExecutor(max_workers=max(1, min(len(Jobs), (os.cpu_count() or 4) - 2))) as Pool:
        Runs.update(dict(Pool.map(Calibrate, Jobs)))
    Samples, Spins, Volts = Merge(C.SweepOrder)
    Fitted["data"], Fitted["problems"], Fitted["report"] = C.Fit(Samples, Spins, Volts, {"test": True}, L.REFERENCE_RAW)


def Merge(Keys):
    Samples, Spins, Volts = [], {}, []
    for Key in Keys:
        Samples += Runs[Key]["samples"]
        Spins.update(Runs[Key]["spin_downs"])
        Volts += Runs[Key]["volts"]
    return Samples, Spins, Volts


class HandGuideCalibrationTests(unittest.TestCase):
    def TestTheSweepsRanWithoutStopping(Self):
        for Key, Run in Runs.items():
            Self.assertIsNone(Run["error"], Key)
            Self.assertGreater(len(Run["samples"]), 1000, Key)

    def TestTheCalibrationIsAccepted(Self):
        Self.assertEqual(Fitted["problems"], [])
        Data = Fitted["data"]
        Self.assertEqual(Data["Version"], C.CalibrationVersion)
        Self.assertAlmostEqual(Data["VCal"], 8.9, places=2)
        Self.assertFalse(Data["CoastBrakes"], "the plant's torque-off floats")
        Self.assertEqual(set(Data["Joints"]), set(L.ALL_JOINTS))
        Self.assertLessEqual(Fitted["report"]["residual_units"], 10.0)

    def TestTheFittedModelIsThePlantsWithinFivePercent(Self):
        Fit, Truth, Prior = np.array(Fitted["data"]["Model"]), np.array(Runs["wrist_flex"]["true_p"]), np.array(L.URDF_P0)
        Kd = L.kd_nm_per_unit(8.9)
        Errors, Priors = [], []
        Samples, _, _ = Merge(L.COMP_JOINTS)
        for S in Samples[::25]:
            X = C.Basis(*[C.Urdf(N, S.Pos[N], L.REFERENCE_RAW) for N in L.COMP_JOINTS])
            for J in L.COMP_JOINTS:
                Row = C.JointRow(J, X)
                Errors.append((Row @ Fit - Row @ Truth) / Kd)
                Priors.append((Row @ Prior - Row @ Truth) / Kd)
                Scale = abs(Row @ Truth) / Kd
                if Scale > 80:
                    Self.assertLess(abs(Row @ Fit - Row @ Truth) / Kd, 0.05 * Scale + 3, (J, Row @ Fit / Kd, Row @ Truth / Kd))
        Self.assertLess(np.sqrt(np.mean(np.square(Errors))), 0.5 * np.sqrt(np.mean(np.square(Priors))),
                        "the fit is closer to the plant than the URDF it started from")

    def TestFrictionsAndSlopesAreThePlants(Self):
        Joints = Fitted["data"]["Joints"]
        Kd, Prior = L.kd_nm_per_unit(8.9), L.kv_prior(8.9)
        Gear = Plant.Scenarios.gears("mid")
        for J in L.COMP_JOINTS:
            Self.assertAlmostEqual(Joints[J]["f_kin0"], 0.8 * Gear[J].f0 / Kd, delta=0.2 * 0.8 * Gear[J].f0 / Kd, msg=J)
            Self.assertAlmostEqual(Joints[J]["f_kin_load"], 0.8 * Gear[J].f_load, delta=0.25 * 0.8 * Gear[J].f_load, msg=J)
        for J in L.FREE_JOINTS:
            Truth = (0.8 * Gear[J].f0 + Plant.Plant.FREE_LINK_FRICTION[J]) / Kd
            Self.assertAlmostEqual(Joints[J]["f_kin"], Truth, delta=0.25 * Truth, msg=J)
        for J in L.ALL_JOINTS:
            Self.assertTrue(0.75 <= Joints[J]["k_v"] / Prior <= 1.10, (J, Joints[J]["k_v"]))
            Self.assertAlmostEqual(Joints[J]["k_v"], Prior + 0.02 / Kd, delta=0.15 * (Prior + 0.02 / Kd), msg=J)

    def TestTheSpinDownTellsFloatingFromBraking(Self):
        Spins = Fitted["report"]["spin_down"]
        Self.assertEqual({J: Spins[J]["verdict"] for J in C.SpinJoints}, dict.fromkeys(C.SpinJoints, "floats"))
        Samples, Spin, Volts = Merge([*C.SweepOrder[:-2], *[f"brakes {J}" for J in C.SpinJoints]])
        Data, Problems, Report = C.Fit(Samples, Spin, Volts, {}, L.REFERENCE_RAW)
        Self.assertEqual(Problems, [])
        Self.assertEqual({J: Report["spin_down"][J]["verdict"] for J in C.SpinJoints}, dict.fromkeys(C.SpinJoints, "brakes"))
        Self.assertTrue(Data["CoastBrakes"])

    def TestACalibrationThatDoesNotFitIsRefused(Self):
        Samples, Spins, Volts = Merge(C.SweepOrder)
        Flipped = [copy.copy(S) for S in Samples]
        for S in Flipped:
            S.Load = -S.Load if S.Joint == "elbow_flex" else S.Load
        Data, Problems, _ = C.Fit(Flipped, Spins, Volts, {}, L.REFERENCE_RAW)
        Self.assertIsNone(Data)
        Self.assertTrue(any("disagrees with the URDF" in P for P in Problems), Problems)
        Uneven = [copy.copy(S) for S in Samples]
        for S in Uneven:
            if S.Joint == "wrist_roll" and S.Run == "B":
                S.Load *= 1.5
        Data, Problems, _ = C.Fit(Uneven, Spins, Volts, {}, L.REFERENCE_RAW)
        Self.assertIsNone(Data)
        Self.assertTrue(any("wrist roll: the two runs' frictions differ" in P for P in Problems), Problems)
        Data, Problems, _ = C.Fit(Samples, Spins, Volts + [Volts[0] - 0.5], {}, L.REFERENCE_RAW)
        Self.assertIsNone(Data)
        Self.assertTrue(any("supply moved 0.5 V" in P for P in Problems), Problems)
        Data, Problems, _ = C.Fit([S for S in Samples if S.Joint != "shoulder_lift"], Spins, Volts, {}, L.REFERENCE_RAW)
        Self.assertIsNone(Data)
        Self.assertTrue(any("not all measured" in P or "shoulder lift" in P for P in Problems), Problems)
        Data, Problems, Report = C.Fit([S for S in Samples if S.Run == "A"], Spins, Volts, {}, L.REFERENCE_RAW)
        Self.assertIsNone(Data)
        Self.assertTrue(any("not enough of its sweeps" in P for P in Problems), Problems)
        json.dumps(Report, allow_nan=False)  # the report reaches the page's JSON.parse: no NaN
        Disagree = {**Spins, "gripper": {45.0: 2.0, 90.0: 3.5}}
        Data, Problems, _ = C.Fit(Samples, Disagree, Volts, {}, L.REFERENCE_RAW)
        Self.assertIsNone(Data)
        Self.assertTrue(any("disagree on whether" in P for P in Problems), Problems)

    def TestTheFingerprintMustMatch(Self):
        Print = {"CadAlignmentVersion": 4, "ReferenceRaw": dict(L.REFERENCE_RAW),
                 "Servos": {J: {"p": 32, "punch": 16} for J in L.ALL_JOINTS}}
        Data = {**Fitted["data"], "Fingerprint": copy.deepcopy(Print)}
        Self.assertEqual(C.FingerprintProblems(Data, Print), [])
        Changed = copy.deepcopy(Print)
        Changed["Servos"]["elbow_flex"]["p"] = 16
        Self.assertIn("elbow_flex's settings changed", C.FingerprintProblems(Data, Changed)[0])
        Changed = {**copy.deepcopy(Print), "CadAlignmentVersion": 5}
        Self.assertIn("zero pose (CadAlignment) changed", C.FingerprintProblems(Data, Changed)[0])
        Self.assertIn("not calibrated", C.FingerprintProblems(None, Print)[0])

    def TestTheLawBuiltFromTheCalibrationHoversThePlant(Self):
        Result = Plant.GuideOnPlant(Fitted["data"], Seconds=8.0, Pose="extended", MassScale=MassScale)
        Self.assertEqual(Result["problems"], [])
        Self.assertIsNone(Result["exit"], Result.get("reason"))
        Self.assertEqual(Result["law_phase"], "guide")
        Self.assertTrue(all(V <= 0.1 for V in Result["hover_drift_deg"].values()), Result["hover_drift_deg"])
        Pushed = Plant.GuideOnPlant(Fitted["data"], Seconds=7.0, Pose="mid", MassScale=MassScale,
                                    Hand=Plant.Scenarios.SpeedHand("shoulder_lift", 15, Plant.Scenarios.GUIDE_FROM, 5.6))
        Self.assertIsNone(Pushed["exit"], Pushed.get("reason"))
        Self.assertLess(abs(Pushed["after_release_deg"]), 2.5)


class Follower:
    """The calibration run on its own (no plant): every energised joint reaches the goal written to it within the cycle,
    in its servo's frame (the goal minus the position it reported); a joint let go runs on 4.4 deg (a floating spin-down)
    then stops. Positions are reported as counted (multi-turn) or modulo a turn (Wrap: a servo that wraps through raw 0).
    Blocked joints stay where they are."""

    def __init__(Self, Run, Start, Wrap=False):
        Self.Run, Self.X, Self.Wrap = Run, dict(Start), Wrap
        Self.T, Self.Released, Self.RunOn, Self.Blocked, Self.Log = 0.0, set(), {}, set(), []

    def Reported(Self):
        return {N: (X % 4096 if Self.Wrap else X) for N, X in Self.X.items()}

    def Readings(Self):
        return {N: {"pos": P, "load": 0.0, "volts": 8.9, "temp_c": 30.0, "status": 0} for N, P in Self.Reported().items()}

    def Step(Self, Corrupt=None):
        Self.T = round(Self.T + 0.01, 6)
        Readings = Self.Readings()
        if Corrupt:
            Corrupt(Readings)
        Out = Self.Run.Step(Self.T, Readings)
        Run = Self.Run
        Self.Log.append((Readings, Out, Run.Segments[Run.Index] if 0 <= Run.Index < len(Run.Segments) else None))
        for N, (Goal, Limit) in Out["writes"].items():
            Self.Released.discard(N)
            if N not in Self.Blocked:
                Self.X[N] += Goal - Readings[N]["pos"]
        for N in Out["release"]:
            Self.Released.add(N)
            Self.RunOn[N] = 5
        for N in list(Self.Released):
            if Self.RunOn.get(N):
                Self.X[N] += 10 * (1 if Self.Run.Goals[N] >= Self.Run.Hold[N] else -1)
                Self.RunOn[N] -= 1
        return Out

    def Until(Self, Done, Seconds=600.0):
        for _ in range(round(Seconds * 100)):
            Out = Self.Step()
            if Out["done"] or Out["error"] or Done(Self):
                return Out
        raise AssertionError("never got there")


class CalibrationRunTests(unittest.TestCase):
    """The 2026-10-10 safety review of the calibration's sweeps, on the run alone (no plant, a few seconds)."""
    Limits = {J: (-3.0, 3.0) for J in L.ALL_JOINTS}
    Start = {"shoulder_pan": 3453.0, "shoulder_lift": 142.0, "elbow_flex": 1216.0, "wrist_flex": 20.0,
             "wrist_roll": 4080.0, "gripper": 900.0}

    def Run(Self, Joints, Spin=(), **Plan):
        Rad = {J: C.Urdf(J, Self.Start[J], L.REFERENCE_RAW) for J in L.ALL_JOINTS}
        Segments, Problems = C.PlanSweeps(Rad, Self.Limits, Joints=Joints, Spin=Spin, **Plan)
        Self.assertEqual(Problems, [])
        return C.CalibrationRun(Segments, dict(Self.Start), dict(Self.Start), L.REFERENCE_RAW,
                                {J: {"p": 32, "punch": 16, "dead_zone": [1, 1]} for J in L.ALL_JOINTS})

    def TestTheSweepsAndTheSpinDownFollowTheReportedFrameThroughRawZero(Self):
        """The wrist sweeps 20 deg either side of raw 20 and the roll spins 30 deg on from raw 4080, crossing raw 0:
        reported as a multi-turn count or wrapping. A wrapped reading stopped the calibration (or ended it with its goal
        a turn away, at full limit), and a spin-down across raw 0 measured 357 deg."""
        for Wrap in (False, True):
            with Self.subTest("wrapping" if Wrap else "multi-turn"):
                Run = Self.Run(("wrist_flex", "wrist_roll"), Spin=("wrist_roll",), Speeds=(20.0,), RunNames=("A",),
                               HalfSpan={"wrist_flex": 20.0, "wrist_roll": 20.0})
                Arm = Follower(Run, Self.Start, Wrap)
                Out = Arm.Until(lambda A: False)
                Self.assertTrue(Out["done"], Out["error"])
                Seen = [R["wrist_flex"]["pos"] for R, _, _ in Arm.Log]
                Self.assertTrue(min(Seen) < 0 if not Wrap else (min(Seen) < 100 and max(Seen) > 4000), (min(Seen), max(Seen)))
                for Readings, Step, _ in Arm.Log:
                    for N, (Goal, Limit) in Step["writes"].items():
                        Self.assertLess(abs(Goal - Readings[N]["pos"]), 200, (N, Goal, Readings[N]["pos"]))
                Self.assertEqual(set(Run.SpinDowns["wrist_roll"]), {45.0, 90.0})
                for Speed, Deg in Run.SpinDowns["wrist_roll"].items():
                    Self.assertAlmostEqual(Deg, 50 * 360 / 4096, delta=0.5, msg=Speed)
                Hold = Run.HoldHere(Arm.T + 0.01, Arm.Readings())
                for N, P in Arm.Reported().items():
                    Self.assertLess(abs(Hold[N] - P), 15, (N, Hold[N], P))

    def TestEveryLimitIsWhatTheJointNeedsNotFullTorqueAndTheGrippersIsLow(Self):
        Run = Self.Run(C.SweepOrder, Spin=C.SpinJoints, Speeds=(5.0, 20.0), RunNames=("A",),
                       HalfSpan={J: 15.0 for J in C.SweepOrder})
        Arm = Follower(Run, Self.Start)
        Self.assertTrue(Arm.Until(lambda A: False)["done"])
        Most, Gripper, Spinning = {}, [], []
        for Readings, Step, Segment in Arm.Log:
            for N, (Goal, Limit) in Step["writes"].items():
                Most[N] = max(Most.get(N, 0.0), Limit)
                if N == "gripper":
                    (Spinning if Segment.Kind == "spin" and Segment.Joint == N else Gripper).append(Limit)
        Self.assertTrue(all(V <= 750 for V in Most.values()), f"under the 800 overload trip, never 1000: {Most}")
        Self.assertLess(Most["wrist_flex"], 500, "the wrist never more than its gravity and speed need, x 2 + 200")
        Self.assertEqual(max(Gripper), 250, "the gripper opens and closes at 250 at most (about 7 N at the jaw, not 28)")
        Self.assertGreater(max(Spinning), 250, "its fast spin needs more, and is tracking-checked")

    def TestABlockedJointStopsItAndTheStopHoldsWhereItIsWithGravityOnly(Self):
        """A blocked sweep stopped "holding" its sweep's goal: 80 steps into the obstacle at Torque_Limit 1000, for as
        long as it held. Now the stop holds every joint where it is, the comp joints with the URDF prior's gravity."""
        Run = Self.Run(("wrist_flex",), Speeds=(20.0,), RunNames=("A",))
        Arm = Follower(Run, Self.Start)
        Arm.Until(lambda A: A.T >= 0.5)
        Arm.Blocked.add("wrist_flex")
        Blocked = Arm.X["wrist_flex"]
        Out = Arm.Until(lambda A: False, Seconds=3.0)
        Self.assertIn("wrist flex is", Out["error"])
        Self.assertIn("off its calibration sweep", Out["error"])
        Hold = Run.HoldHere(Arm.T + 0.01, Arm.Readings())
        Effort = C.PriorEffort(Arm.X, 8.9, L.REFERENCE_RAW)
        for N, P in Arm.Reported().items():
            Expect = P + (L.effort_steps(Effort[N], 8.0, 16.0, 1.0) if N in Effort else 0.0)
            Self.assertAlmostEqual(Hold[N], Expect, places=6, msg=N)
        Self.assertLess(abs(Hold["wrist_flex"] - Blocked), 12, "gravity's few steps, not the sweep's 80")

    def TestOnePositionGlitchIsAMissedReplyAndTwoStopIt(Self):
        Run = Self.Run(("wrist_flex",), Speeds=(20.0,), RunNames=("A",))
        Arm = Follower(Run, Self.Start)
        Arm.Until(lambda A: A.T >= 0.5)
        Glitch = lambda R: R["elbow_flex"].update(pos=R["elbow_flex"]["pos"] + 300)
        Out = Arm.Step(Glitch)
        Self.assertEqual((Out["writes"], Out["error"]), ({}, None), "every servo keeps its goal")
        Arm.Until(lambda A: A.T >= 1.0)
        Self.assertIsNone(Run.Error)
        Arm.Step(Glitch)
        Self.assertIn("elbow flex position jumped on two readings in a row", Arm.Step(Glitch)["error"])


def load_tests(Loader, Tests, Pattern):
    Prefixed = unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return unittest.TestSuite([Prefixed.loadTestsFromTestCase(HandGuideCalibrationTests),
                               Prefixed.loadTestsFromTestCase(CalibrationRunTests)])


if __name__ == "__main__":
    Loader = unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Result = unittest.TextTestRunner(verbosity=2).run(unittest.TestSuite([Loader.loadTestsFromTestCase(HandGuideCalibrationTests),
                                                                          Loader.loadTestsFromTestCase(CalibrationRunTests)]))
    raise SystemExit(not Result.wasSuccessful())
