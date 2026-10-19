"""The hand-guiding law (HandGuide.py) on the offline plant: every claim of docs/HandGuide.md section 7.2 that the bridge
relies on, as an assertion. Offline only: no serial port, no bridge, no camera (HandGuidePlant.py).

The plant runs about 45 scenarios side by side (one process each, about 20 s); the law's own checks run without it.

    python TestHandGuideLaw.py            (the quick set)
    python TestHandGuideLaw.py --full     (also every one of the 323 scenarios runs without an error, about a minute)
"""
from __future__ import annotations

import math
import os
import sys
import unittest
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import HandGuide as L  # noqa: E402
import HandGuidePlant as Plant  # noqa: E402

DEG = math.pi / 180
FREE_ROLL = "wrist_roll"
# The scenarios this suite asserts (guide_scenarios.py names them), run once per process.
Named = [
    "start extended mid model 1.0", "start home low model 1.0", "start mid high model 1.0",
    "sign error shoulder_lift at extended low", "sign error wrist_flex at mid high", "gain 1.5 at extended low",
    "feel lift 15 deg/s mid c=0.0 no FF host 100 Hz P32", "feel lift 15 deg/s mid c=0.6 FF 0.5 host 100 Hz P32",
    "free feel wrist_roll 10 deg/s mid coast", "free feel wrist_roll 10 deg/s mid assist",
    "free feel gripper 10 deg/s high coast", "free feel gripper 10 deg/s high assist",
    "free feel shoulder_pan 10 deg/s mid coast", "free feel shoulder_pan 10 deg/s mid assist",
    "free feel wrist_roll 60 deg/s mid coast", "free feel wrist_roll 60 deg/s mid assist",
    "free feel wrist_roll 10 deg/s low coast", "free feel wrist_roll 10 deg/s low assist",
    "free flick wrist_roll 30 deg in 0.4 s mid coast", "free flick wrist_roll 30 deg in 0.4 s mid assist",
    "free release wrist_roll from 15 deg/s mid assist", "free release shoulder_pan from 15 deg/s high assist",
    "free release gripper from 15 deg/s low assist", "free release wrist_roll from 80 deg/s mid assist",
    "free release: pan, roll and gripper together from 30 deg/s, low friction",
    "braking driver: free feel wrist_roll 30 deg/s mid coast", "braking driver: free feel wrist_roll 30 deg/s mid assist",
    "wrist_roll friction x0.25 of calibrated, k_f 0.7 (2.80 x the real friction): release from 15 deg/s",
    "gripper friction x0.4 of calibrated, k_f 0.5 (1.25 x the real friction): release from 15 deg/s",
    "lift, elbow, wrist friction x0.4 of calibrated, k_f 0.7, probe on: release from 20 deg/s",
    "release lift c=0.6 mid FF 0.5", "release lift c=0.8 low FF 0.5",
    "Hold here at extended low P32", "Hold here at extended mid P16",
    "fault coast (E-stop drill) hands off at extended low", "lift at 55 C while hovering at mid: coast",
    "wrist_roll stops answering while hovering at mid: coast", "USB outage 0.3 s hands off at extended",
    "lift servo brown-out reset at extended low", "yank lift 90 deg/s low", "brisk lift 40 deg/s (under the 45 cap) low",
    "supply 8.2 V at entry: refused", "supply sags to 6.8 V while guiding: coast",
    "supply sag to 7.6 V, voltage scaling on ultralow friction", "drops 1 % per reply, hover at extended, low friction",
    "burst of 3 missed lift replies", "runtime sign flip lift at mid low", "stall 1.7 s hands off low",
    "elbow into its limit low", "gripper closed into its limit low",
    "settings refused: k_f 0.9 on the roll", "settings refused: lead c 1.1 on the roll",
    "settings refused: lead c 0.95 on the lift", "settings refused: velocity slope 1.5 x the motor's on the pan",
    "settings refused: assist top speed 200 deg/s on the gripper",
]
FaithfulTo = "free release wrist_roll from 15 deg/s mid assist"
Results: dict = {}
Reference: dict = {}


def Workers() -> int:
    return max(1, min(16, (os.cpu_count() or 4) - 4))


def setUpModule():
    with ProcessPoolExecutor(max_workers=Workers()) as Pool:
        Futures = {Name: Pool.submit(Plant.RunNamed, Name) for Name in Named}
        Faithful = Pool.submit(Plant.RunNamed, FaithfulTo, True)
        for Name, Future in Futures.items():
            Results[Name] = Future.result()
        Reference[FaithfulTo] = Faithful.result()


def Comp(Value) -> dict:
    return {Joint: Value for Joint in L.COMP_JOINTS}


Pose = {"shoulder_pan": 3453, "shoulder_lift": 256 + 455, "elbow_flex": 1671 - 455, "wrist_flex": 605 - 228,
        "wrist_roll": 2588, "gripper": 900}


class Bench:
    """The six servos for the law on its own (no plant): each joint where Where(joint, t) puts it (continuous steps;
    reported as they are, or modulo a turn with Wrap: a servo whose position wraps through raw 0), Present_Load the duty
    its last goal asks for in its own frame (P 32, punch 16, dead zone 1, clamped at its Torque_Limit), torque as the
    commands left it, and the status registers read every 4th cycle and again on the next while the law asks
    (status_recheck), as the bridge does. Each(t, readings) may corrupt a cycle's readings."""

    def __init__(Self, Law, Where=None, Wrap=False, Band=False):
        Self.Law, Self.Wrap, Self.Band = Law, Wrap, Band
        Self.Where = Where or (lambda J, T: float(Pose[J]))
        Self.Goal, Self.Limit, Self.Torque = {}, {}, dict.fromkeys(L.ALL_JOINTS, True)
        Self.T, Self.Cycle, Self.Commands, Self.Seen = 0.0, 0, [], []
        Readings = Self.Readings(True)
        for J in L.ALL_JOINTS:
            Self.Goal[J], Self.Limit[J] = Readings[J].pos, 1000.0
        Self.Apply(Law.begin(0.0, Readings, {J: Readings[J].pos for J in L.COMP_JOINTS}))

    def Raw(Self, J):
        Raw = Self.Where(J, Self.T)
        return Raw % 4096 if Self.Wrap else Raw

    def Readings(Self, Status):
        Out = {}
        for J in L.ALL_JOINTS:
            Raw, Load = Self.Raw(J), 0.0
            E = Self.Goal.get(J, Raw) - Raw if Self.Torque[J] else 0.0
            if abs(E) > 1:
                Duty = 8 * (E - math.copysign(1, E)) + math.copysign(16, E)
                Load = max(-Self.Limit[J], min(Self.Limit[J], Duty))
            Out[J] = L.Reading(ok=True, pos=float(Raw), load=Load, volts=8.9, temp_c=30.0, status=0,
                               torque_on=Self.Torque[J] if Status else None, lock=1 if Status else None)
        return Out

    def Apply(Self, Command):
        for J, W in Command.writes.items():
            Self.Goal[J], Self.Limit[J], Self.Torque[J] = W["goal"], W["limit"], True
        for J in Command.release:
            Self.Torque[J] = False
        Self.Commands.append(Command)
        return Command

    def Step(Self, Each=None):
        Self.T = round(Self.T + 0.01, 6)
        Readings = Self.Readings(Self.Cycle % 4 == 0 or getattr(Self.Law, "status_recheck", False))
        Self.Cycle += 1
        if Each:
            Each(Self.T, Readings)
        Self.Seen.append({J: R.pos for J, R in Readings.items()})
        return Self.Apply(Self.Law.step(Self.T, Readings, wall_band=Self.Band, wall_age=0.0))

    def Run(Self, Seconds, Each=None):
        for _ in range(round(Seconds * 100)):
            Command = Self.Step(Each)
            if Command.exit_kind:
                return Command
        return Self.Commands[-1]

    def Guiding(Self):
        """Through the 2.2 s hands-off start into GUIDE."""
        Command = Self.Run(2.6)
        assert Self.Law.phase == "guide", (Command.exit_kind, Command.reason)
        return Self


class HandGuideLawTests(unittest.TestCase):
    def R(Self, Name):
        return Results[Name]

    # -- the law on its own -----------------------------------------------------------------------------------------------
    def Law(Self, Free=None, Comp=None, Kept=(), **Params):
        Configs = {J: L.JointConfig(J, **{"k_v": L.kv_prior(8.9), "f_kin0": 30.0, "f_kin_load": 0.3, **(Comp or {}).get(J, {})})
                   for J in L.COMP_JOINTS}
        Frees = {J: L.FreeConfig(J, **{"k_v": L.kv_prior(8.9), "f_kin": 30.0, **L.FREE_DEFAULTS[J], **(Free or {}).get(J, {})})
                 for J in L.FREE_JOINTS if J not in Kept}
        return L.HandGuideLaw(Configs, L.GravityModel(), L.GuideParams(**Params), free_configs=Frees)

    def Readings(Self, **Over):
        Pose = {"shoulder_pan": 3453, "shoulder_lift": 256 + 455, "elbow_flex": 1671 - 455, "wrist_flex": 605 - 228,
                "wrist_roll": 2588, "gripper": 900}
        return {J: L.Reading(**{"ok": True, "pos": float(Raw), **Over.get(J, {})}) for J, Raw in Pose.items()}

    def TestOutOfBoundsAssistSettingsAreRefusedBeforeAnyWrite(Self):
        for Free, Comp, Words in (({FREE_ROLL: {"k_f": 0.9}}, None, "friction assist 0.9"),
                                  ({FREE_ROLL: {"c_lead": 1.1}}, None, "lead 1.1"),
                                  (None, {"shoulder_lift": {"c_lead": 0.95}}, "lead 0.95"),
                                  ({"shoulder_pan": {"k_v": 1.5 * L.kv_prior(8.9)}}, None, "velocity slope"),
                                  ({"gripper": {"v_hi": 200 * DEG}}, None, "top speed 200"),
                                  ({"gripper": {"cap_max": 600.0}}, None, "torque ceiling over 500"),
                                  ({FREE_ROLL: {"v_on": 1.0 * DEG}}, None, "out of order")):
            Law = Self.Law(Free, Comp)
            Command = Law.begin(0.0, Self.Readings(), {J: 1000.0 for J in L.COMP_JOINTS})
            Self.assertEqual(Command.exit_kind, "refused", Words)
            Self.assertIn(Words, Command.reason)
            Self.assertEqual(Command.writes, {}, "a refused start writes nothing")
        Self.assertEqual(Self.Law().problems, [])

    def TestEntryChecksTheSupplyItsSpreadAndTheTemperature(Self):
        Hold = {J: 1000.0 for J in L.COMP_JOINTS}
        for Over, Words in (({J: {"volts": 8.2} for J in L.ALL_JOINTS}, "reads 8.2 V (at least 8.5 V needed)"),
                            ({"gripper": {"volts": 9.5}}, "more than 0.5 V apart"),
                            ({J: {"volts": 11.0} for J in L.ALL_JOINTS}, "from the 8.9 V hand guiding was calibrated at"),
                            ({"shoulder_lift": {"temp_c": 50.0}}, "shoulder_lift 50 C"),
                            ({"wrist_roll": {"ok": False}}, "wrist_roll did not answer")):
            Command = Self.Law().begin(0.0, Self.Readings(**Over), Hold)
            Self.assertEqual(Command.exit_kind, "refused", Words)
            Self.assertIn(Words, Command.reason)
            Self.assertEqual(Command.writes, {})
        Command = Self.Law().begin(0.0, Self.Readings(), Hold)
        Self.assertIsNone(Command.exit_kind)
        Self.assertEqual(set(Command.writes), set(L.COMP_JOINTS), "the start preloads the comp joints only")
        Self.assertTrue(all(W["limit"] <= 650 for W in Command.writes.values()))

    def TestTheBandTopFollowsTheCrossoverAndFloatsWhenTheHelpIsTooSmall(Self):
        Law = Self.Law()
        F = Law.fcfg[FREE_ROLL]
        Ff = F.k_f * F.f_kin
        Expected = min(F.v_hi, 0.8 * Ff / ((1 - F.c_lead) * F.k_v + F.k_loop))
        Self.assertAlmostEqual(Law.band_top(FREE_ROLL, 8.9), Expected, places=9)
        Self.assertGreater(Expected, 12 * DEG)
        Small = Self.Law(Free={FREE_ROLL: {"f_kin": 5.0}})
        Self.assertEqual(Small.band_top(FREE_ROLL, 8.9), 0.0, "a band under 3 x v_on is not worth switching on for")
        Brakes = Self.Law(Free={FREE_ROLL: {"coast_brakes": True}})
        Self.assertEqual(Brakes.band_top(FREE_ROLL, 8.9), Brakes.fcfg[FREE_ROLL].v_hi)

    def TestTheLeadStaysPassiveAtTheBoundsAtAnyDelay(Self):
        # Re = b (1 - c cos w tau) > 0 for c kv <= 0.99 of the real slope: the largest lead the bounds allow.
        Prior = L.kv_prior(8.9)
        for Lead in (0.6, 0.8, 0.9):
            Slope = 1.10 * Prior * Lead / (Prior + 0.02 / L.kd_nm_per_unit(8.9))
            Self.assertLess(Slope, 1.0)
            for Omega in (1.0, 10.0, 60.0, 300.0):
                for Tau in (0.0, 0.025, 0.03, 0.1):
                    Self.assertGreater(1 - Slope * math.cos(Omega * Tau), 0.0)

    def TestTheFeedForwardIsZeroAtRestAndBelowTheFrictionItCancels(Self):
        Law = Self.Law(Free={FREE_ROLL: {"k_f": 0.7}})
        for Speed, Expect in ((0.0, 0.0), (1.9 * DEG, 0.0), (4.0 * DEG, 0.5), (6.0 * DEG, 1.0)):
            Law.stroke[FREE_ROLL] = None
            Self.assertAlmostEqual(Law.ff_shape(FREE_ROLL, 0.0, Speed, 2 * DEG, 6 * DEG, 1.0, 2.0), Expect, places=9)
        Self.assertLessEqual(Law.ff_size(FREE_ROLL, {}, 8.9), 0.7 * Law.fcfg[FREE_ROLL].f_kin + 1e-9)
        # one stroke of help lasts 2 s, then fades over 0.5 s
        Law.stroke[FREE_ROLL] = None
        Self.assertEqual(Law.stroke_gain(FREE_ROLL, 0.0, 10 * DEG, 2 * DEG, 6 * DEG), 1.0)
        Self.assertEqual(Law.stroke_gain(FREE_ROLL, 2.0, 10 * DEG, 2 * DEG, 6 * DEG), 1.0)
        Self.assertAlmostEqual(Law.stroke_gain(FREE_ROLL, 2.25, 10 * DEG, 2 * DEG, 6 * DEG), 0.5, places=9)
        Self.assertEqual(Law.stroke_gain(FREE_ROLL, 2.6, 10 * DEG, 2 * DEG, 6 * DEG), 0.0)

    # -- the 2026-10-10 safety review: one corrupted reply, the frame, the kept gripper, the bounds, the walls ---------------
    def TestOneCorruptedReplyDoesNotDropTheArmButAPersistingFaultDoes(Self):
        """2026-10-10, plain Hold: one packet read torque off, 0 V, 90 C and load 0xF501 on a servo that was fine. Every
        check but heat (temp_exit_hold_s) acted on the first reading and dropped the arm."""
        def Packet(T, R):  # the whole corrupted reply, on a status cycle
            R["elbow_flex"] = L.Reading(ok=True, pos=R["elbow_flex"].pos, load=-61697.0, volts=0.0, temp_c=90.0, status=32,
                                        torque_on=False, lock=0)
        B = Bench(Self.Law()).Guiding()
        while B.Cycle % 4:
            B.Step()
        First = B.Step(Packet)
        Self.assertIsNone(First.exit_kind, First.reason)
        Self.assertTrue(B.Law.status_recheck, "the status registers are read again on the next cycle")
        Self.assertTrue(all(abs(W["goal"] - Pose[J]) < 60 for J, W in First.writes.items()), First.writes)
        B.Run(0.5)
        Self.assertEqual(B.Law.phase, "guide")
        Self.assertFalse(B.Law.status_recheck)
        # each fault read twice in a row (the bridge's re-read confirms it) goes limp: 10 ms later than before
        Faults = (("firmware", lambda T, R: setattr(R["shoulder_lift"], "status", 32), "firmware protection bit 32"),
                  ("torque", lambda T, R: setattr(R["elbow_flex"], "torque_on", False), "torque unexpectedly off"),
                  ("restart", lambda T, R: setattr(R["wrist_roll"], "lock", 0), "Lock register reads 0"),
                  ("supply, lift", lambda T, R: setattr(R["shoulder_lift"], "volts", 0.0), "below 7.0 V"),
                  ("supply, gripper alone", lambda T, R: setattr(R["gripper"], "volts", 6.5), "6.5 V below 7.0 V"))
        for Name, Fault, Words in Faults:
            with Self.subTest(Name):
                B = Bench(Self.Law()).Guiding()
                while B.Cycle % 4:
                    B.Step()
                Self.assertIsNone(B.Step(Fault).exit_kind, "one reading")
                Command = B.Step(Fault)
                Self.assertEqual(Command.exit_kind, "coast", "two in a row")
                Self.assertIn(Words, Command.reason)

    def TestAPositionGlitchIsAMissedReplyAndTwoInARowGoLimp(Self):
        """A +300-step glitch made the lift's goal 351 steps away at its 650 cap for one cycle: now that reply is missed
        (the lift keeps its last goal), the next is used as usual, and only two in a row end guiding."""
        B = Bench(Self.Law()).Guiding()
        Command = B.Step(lambda T, R: setattr(R["shoulder_lift"], "pos", R["shoulder_lift"].pos + 300))
        Self.assertIsNone(Command.exit_kind, Command.reason)
        Self.assertNotIn("shoulder_lift", Command.writes, "it keeps its last goal")
        Self.assertIn("elbow_flex", Command.writes)
        Since = len(B.Commands)
        B.Run(0.5)
        Self.assertEqual(B.Law.phase, "guide")
        Goals = [C.writes["shoulder_lift"]["goal"] for C in B.Commands[Since:] if "shoulder_lift" in C.writes]
        Self.assertTrue(Goals and all(abs(G - Pose["shoulder_lift"]) < 60 for G in Goals), Goals)
        Command = B.Run(0.05, lambda T, R: setattr(R["elbow_flex"], "pos", R["elbow_flex"].pos + 300))
        Self.assertEqual(Command.exit_kind, "coast")
        Self.assertIn("elbow_flex position jumped", Command.reason)
        Self.assertIn("on 2 readings in a row", Command.reason)

    def TestTheGoalsFollowTheReportedFrameThroughRawZero(Self):
        """The wrist flexed (6 deg/s) and the roll turned (12 deg/s, assisted) by hand through raw 0, with the servos
        reporting a multi-turn count or wrapping to 0..4095: no exit, and every goal written is an offset from the
        position that servo reported that cycle (a wrapped reading went limp: "frame jumped -4095 steps")."""
        Start = {"wrist_flex": 40.0, "wrist_roll": 4040.0}
        Rate = {"wrist_flex": -6 * 4096 / 360, "wrist_roll": 12 * 4096 / 360}
        Where = lambda J, T: Start[J] + Rate[J] * max(0.0, T - 2.7) if J in Start else float(Pose[J])
        for Wrap in (False, True):
            with Self.subTest("wrapping" if Wrap else "multi-turn"):
                B = Bench(Self.Law(), Where, Wrap=Wrap).Guiding()
                Since = len(B.Commands)
                Command = B.Run(1.8)
                Self.assertIsNone(Command.exit_kind, Command.reason)
                Wrist, Roll = B.Seen[-1]["wrist_flex"], B.Seen[-1]["wrist_roll"]
                Self.assertTrue(3900 < Wrist < 4096 and 0 <= Roll < 300 if Wrap else Wrist < 0 and Roll > 4096, (Wrist, Roll))
                Self.assertGreaterEqual(B.Law.assist_counts["wrist_roll"]["on"], 1, "the roll was assisted")
                Writes = 0
                for Seen, C in zip(B.Seen[Since - 1:], B.Commands[Since:]):
                    for J in Start:
                        if J in C.writes:
                            Writes += 1
                            Self.assertLess(abs(C.writes[J]["goal"] - Seen[J]), 60, (J, Seen[J], C.writes[J]))
                Self.assertGreater(Writes, 200)

    def TestAKeptGripperIsWatchedLikeTheOthers(Self):
        """A gripper left squeezing an object ("keep") was not read for faults while guiding."""
        for Name, Fault, Words in (("firmware", lambda T, R: setattr(R["gripper"], "status", 8), "gripper firmware protection"),
                                   ("silent", lambda T, R: setattr(R["gripper"], "ok", False), "gripper stopped answering"),
                                   ("heat", lambda T, R: setattr(R["gripper"], "temp_c", 60.0), "gripper over 250 ms at 60 C"),
                                   ("restart", lambda T, R: setattr(R["gripper"], "lock", 0 if R["gripper"].lock is not None
                                                                     else None), "gripper restarted")):
            with Self.subTest(Name):
                B = Bench(Self.Law(Kept=("gripper",))).Guiding()
                Command = B.Run(0.5, Fault)
                Self.assertEqual(Command.exit_kind, "coast")
                Self.assertIn(Words, Command.reason)
                Self.assertFalse(any("gripper" in C.writes or "gripper" in C.release for C in B.Commands), "never driven")

    def TestAnExitTakesOnlyPlausibleFreshReadingsInTheirOwnFrame(Self):
        B = Bench(Self.Law()).Guiding()
        Fresh = B.Readings(False)
        Fresh["shoulder_lift"].pos += 300   # corrupted: the lift is held where it last read
        Fresh["elbow_flex"].pos -= 4096     # the same angle a turn away: held in the frame it now reports
        Command = B.Law._exit(L.Command(), "preloaded", "Hold here", fresh=Fresh)
        Self.assertLess(abs(Command.writes["shoulder_lift"]["goal"] - Pose["shoulder_lift"]), 60)
        Self.assertLess(abs(Command.writes["elbow_flex"]["goal"] - Fresh["elbow_flex"].pos), 60)
        Self.assertEqual(Command.writes["wrist_roll"]["goal"], Pose["wrist_roll"])

    def TestAStartInsideTheCollisionGuardBandIsHeldThere(Self):
        """The walls were armed only once the guard band first read clear: a start inside it had none."""
        B = Bench(Self.Law(), Band=True).Guiding()
        Command = B.Step()
        Self.assertIsNone(Command.exit_kind)
        Self.assertEqual({J: Command.writes[J]["limit"] for J in L.COMP_JOINTS}, Comp(750.0))
        Self.assertEqual(B.Law.fstate, dict.fromkeys(L.FREE_JOINTS, "caught"))

    def TestAlphaAndVCalOutsideTheDesignAreRefused(Self):
        Hold = {J: float(Pose[J]) for J in L.COMP_JOINTS}
        for Params, Words in (({"alpha": 1.2}, "model gain (alpha) 1.2 is outside 0.25..1"),
                              ({"alpha": 0.1}, "model gain (alpha) 0.1"), ({"alpha": float("nan")}, "model gain (alpha) nan"),
                              ({"v_cal": 0.0}, "supply (V_cal) 0 V is outside 8.2..12.6 V"),
                              ({"v_cal": 30.0}, "supply (V_cal) 30 V")):
            Command = Self.Law(**Params).begin(0.0, Self.Readings(), Hold)
            Self.assertEqual(Command.exit_kind, "refused", Params)
            Self.assertIn(Words, Command.reason)
            Self.assertEqual(Command.writes, {})

    def TestTheUrdfAnglesUseThisArmsReferenceRaw(Self):
        Law = L.HandGuideLaw({J: L.JointConfig(J) for J in L.COMP_JOINTS}, L.GravityModel(),
                             reference_raw={**L.REFERENCE_RAW, "shoulder_lift": 300})
        Self.assertAlmostEqual(Law.urdf("shoulder_lift", 300 + 1024), math.pi / 2, places=9)
        Self.assertAlmostEqual(Law.urdf("elbow_flex", L.REFERENCE_RAW["elbow_flex"] - 4096 - 512), -math.pi / 4, places=9)

    # -- the plant ---------------------------------------------------------------------------------------------------------
    def TestThePortRunsTheReferenceLawExactly(Self):
        Mine, Theirs = Self.R(FaithfulTo), Reference[FaithfulTo]
        for Key in ("exit", "final_deg", "max_speed_dps", "events", "assist_counts", "hand_force_N", "after_release_deg"):
            Self.assertEqual(Mine.get(Key), Theirs.get(Key), Key)

    def TestNoScenarioErrored(Self):
        Self.assertEqual([N for N, R in Results.items() if "error" in R], [])

    def TestCorrectStartsHoverWithoutMoving(Self):
        for Name in ("start extended mid model 1.0", "start home low model 1.0", "start mid high model 1.0"):
            R = Self.R(Name)
            Self.assertIsNone(R["exit"], R.get("reason"))
            Self.assertEqual(R["law_phase"], "guide")
            Self.assertTrue(all(V <= 0.05 for V in R["hover_drift_deg"].values()), (Name, R["hover_drift_deg"]))

    def TestBadModelsNeverMoveTheArmFar(Self):
        for Name in ("sign error shoulder_lift at extended low", "sign error wrist_flex at mid high"):
            R = Self.R(Name)
            Self.assertEqual(R["exit"], "anchor")
            Self.assertIn("disagrees with the URDF", R["reason"])
            Self.assertTrue(all(V == 0.0 for V in R["max_speed_dps"].values()), "refused before any write")
        R = Self.R("gain 1.5 at extended low")
        Self.assertEqual(R["exit"], "anchor")
        Self.assertLessEqual(R["ramp_max_dev_steps"], 26)

    def TestTheFrictionFeedForwardLightensTheLift(Self):
        Plain = Self.R("feel lift 15 deg/s mid c=0.0 no FF host 100 Hz P32")["hand_force_N"]
        Helped = Self.R("feel lift 15 deg/s mid c=0.6 FF 0.5 host 100 Hz P32")["hand_force_N"]
        Self.assertLess(Helped, 0.6 * Plain, (Plain, Helped))

    def TestSlowSteadyMovesOfTheFreeJointsTakeLessForceThanCoast(Self):
        for Joint, Fric in ((FREE_ROLL, "mid"), ("gripper", "high"), ("shoulder_pan", "mid")):
            Coast = Self.R(f"free feel {Joint} 10 deg/s {Fric} coast")["hand_force_N"]
            Assist = Self.R(f"free feel {Joint} 10 deg/s {Fric} assist")
            Force = Assist.get("assisted_force_N", Assist["hand_force_N"])
            Self.assertLess(Force, 0.85 * Coast, (Joint, Coast, Force))
            Self.assertIsNone(Assist["exit"])
        Braked = Self.R("braking driver: free feel wrist_roll 30 deg/s mid coast")["hand_force_N"]
        Helped = Self.R("braking driver: free feel wrist_roll 30 deg/s mid assist")["hand_force_N"]
        Self.assertLess(Helped, 0.5 * Braked, "a braking Coast is helped at every speed")

    def TestAboveTheBandAndOnFlicksItIsExactlyCoast(Self):
        for Pair in (("free feel wrist_roll 60 deg/s mid coast", "free feel wrist_roll 60 deg/s mid assist", "hand_force_N"),
                     ("free feel wrist_roll 10 deg/s low coast", "free feel wrist_roll 10 deg/s low assist", "hand_force_N"),
                     ("free flick wrist_roll 30 deg in 0.4 s mid coast", "free flick wrist_roll 30 deg in 0.4 s mid assist",
                      "flick_peak_N")):
            Coast, Assist = Self.R(Pair[0])[Pair[2]], Self.R(Pair[1])[Pair[2]]
            Self.assertLessEqual(abs(Assist - Coast), 0.05 * Coast + 0.02, Pair)

    def TestReleasedFreeJointsStopAndStayWhereTheyAreLeft(Self):
        for Name in ("free release wrist_roll from 15 deg/s mid assist", "free release shoulder_pan from 15 deg/s high assist",
                     "free release gripper from 15 deg/s low assist"):
            R = Self.R(Name)
            Self.assertIsNone(R["exit"])
            Self.assertLess(abs(R["after_release_deg"]), 1.0, (Name, R["after_release_deg"]))
            Self.assertLessEqual(R["stop_s"], 0.2, Name)
            Self.assertEqual(R["creep_last_2s_deg"], 0.0, Name)
        Fast = Self.R("free release wrist_roll from 80 deg/s mid assist")
        Self.assertLess(abs(Fast["after_release_deg"]), 9.0)
        Self.assertEqual(Fast["creep_last_2s_deg"], 0.0)
        Together = Self.R("free release: pan, roll and gripper together from 30 deg/s, low friction")
        Self.assertEqual(Together["creep_last_2s_deg"], 0.0)
        Self.assertEqual(Together["end_speed_dps"], 0.0)

    def TestTheAssistNeverSelfPropelsEvenWithFrictionFarBelowCalibrated(Self):
        for Name, Most in (("wrist_roll friction x0.25 of calibrated, k_f 0.7 (2.80 x the real friction): release from 15 deg/s", 10.0),
                           ("gripper friction x0.4 of calibrated, k_f 0.5 (1.25 x the real friction): release from 15 deg/s", 1.5)):
            R = Self.R(Name)
            Self.assertLess(abs(R["after_release_deg"]), Most, (Name, R["after_release_deg"]))
            Self.assertEqual(R["creep_last_2s_deg"], 0.0, Name)
            Self.assertEqual(R["end_speed_dps"], 0.0, Name)
        Probed = Self.R("lift, elbow, wrist friction x0.4 of calibrated, k_f 0.7, probe on: release from 20 deg/s")
        Self.assertFalse(Probed["ff_on"]["shoulder_lift"], "the start's probe takes the lift's feed-forward away")
        Self.assertLess(abs(Probed["after_release_deg"]), 2.0)

    def TestTheLiftStopsAfterAPushAtTheAllowedLeads(Self):
        for Name in ("release lift c=0.6 mid FF 0.5", "release lift c=0.8 low FF 0.5"):
            R = Self.R(Name)
            Self.assertIsNone(R["exit"], R.get("reason"))
            Self.assertLess(abs(R["after_release_deg"]), 2.5, (Name, R["after_release_deg"]))
            Self.assertEqual(R["end_speed_dps"], 0.0)

    def TestHoldHereHoldsWithoutSag(Self):
        for Name in ("Hold here at extended low P32", "Hold here at extended mid P16"):
            R = Self.R(Name)
            Self.assertEqual(R["exit"], "preloaded")
            Self.assertTrue(all(V <= 0.05 for V in R["moved_after_deg"].values()), (Name, R["moved_after_deg"]))

    def TestEveryFaultGoesLimp(Self):
        for Name, Words in (("fault coast (E-stop drill) hands off at extended low", "fault"),
                            ("lift at 55 C while hovering at mid: coast", " C: every joint goes limp"),
                            ("wrist_roll stops answering while hovering at mid: coast", "stopped answering"),
                            ("USB outage 0.3 s hands off at extended", "USB serial link dropped"),
                            ("lift servo brown-out reset at extended low", "shoulder_lift"),
                            ("yank lift 90 deg/s low", "over its hand-guiding speed"),
                            ("supply sags to 6.8 V while guiding: coast", "below 7.0 V"),
                            ("runtime sign flip lift at mid low", "disagrees with the URDF"),
                            ("stall 1.7 s hands off low", "control loop paused")):
            R = Self.R(Name)
            Self.assertEqual(R["exit"], "coast", (Name, R.get("reason")))
            Self.assertIn(Words, R["reason"], Name)
            if "torque_after_exit" in R:  # recorded scenarios only
                Self.assertEqual(set(R["torque_after_exit"].values()), {0}, f"{Name}: every joint limp")
        Brown = Self.R("lift servo brown-out reset at extended low")
        Self.assertLess(Brown["exit_t"] - 4.5, 0.05, "a restart is caught within 50 ms")
        Flip = Self.R("runtime sign flip lift at mid low")
        Self.assertLess(Flip["exit_t"] - 4.5, 0.02, "a model error is caught on the first cycle")

    def TestNoFalseExitsFromABriskMoveDropsOrASag(Self):
        for Name in ("brisk lift 40 deg/s (under the 45 cap) low", "drops 1 % per reply, hover at extended, low friction",
                     "burst of 3 missed lift replies", "supply sag to 7.6 V, voltage scaling on ultralow friction"):
            R = Self.R(Name)
            Self.assertIsNone(R["exit"], (Name, R.get("reason")))
        Sag = Self.R("supply sag to 7.6 V, voltage scaling on ultralow friction")
        Self.assertTrue(all(V <= 0.1 for V in Sag["hover_drift_deg"].values()), Sag["hover_drift_deg"])

    def TestALowSupplyAtEntryIsRefused(Self):
        R = Self.R("supply 8.2 V at entry: refused")
        Self.assertEqual(R["exit"], "refused")
        Self.assertIn("8.2 V (at least 8.5 V needed)", R["reason"])

    def TestWallsHoldTheJointsNearTheirLimits(Self):
        Self.assertLess(Self.R("elbow into its limit low")["past_limit_max_deg"], 4.0)
        Gripper = Self.R("gripper closed into its limit low")
        Self.assertLess(Gripper["past_limit_max_deg"], 1.0)
        Self.assertLess(Gripper["past_limit_end_deg"], 0.5)

    def TestWrongSettingsAreRefusedOnThePlant(Self):
        for Name in [N for N in Named if N.startswith("settings refused")]:
            R = Self.R(Name)
            Self.assertEqual(R["exit"], "refused", Name)
            Self.assertTrue(all(V == 0.0 for V in R["max_speed_dps"].values()), Name)


class FullScenarioSet(unittest.TestCase):
    def TestEveryScenarioRuns(Self):
        Names = list(Plant.AllScenarios())
        with ProcessPoolExecutor(max_workers=Workers()) as Pool:
            Out = list(Pool.map(Plant.RunNamed, Names))
        Self.assertEqual(len(Out), 323)
        Self.assertEqual([R["group"] + ": " + Name for Name, R in zip(Names, Out) if "error" in R], [])


def Suite(Full: bool):
    Loader = unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Tests = Loader.loadTestsFromTestCase(HandGuideLawTests)
    if Full:
        Tests.addTests(Loader.loadTestsFromTestCase(FullScenarioSet))
    return Tests


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestHandGuideLaw` runs the Test* methods too (unittest names this hook)."""
    return Suite(False)


if __name__ == "__main__":
    Result = unittest.TextTestRunner(verbosity=2).run(Suite("--full" in sys.argv))
    raise SystemExit(not Result.wasSuccessful())
