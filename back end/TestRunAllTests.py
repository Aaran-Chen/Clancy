"""RunAllTests.py's own rules (2026-10-08 regression review): the accepted failures are known by suite and by each
failing check, so a new failure inside the same suite still fails the run; a UI suite that fails is run once more and
reported "flaky" when it then passes. No suite is run: the logs are made up here, RunSuite is replaced.

Run:  python TestRunAllTests.py
"""
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import RunAllTests as Runner  # noqa: E402

SelfCalibration = """  [PASS] old archive replays: archived
  [FAIL] table mapping: RMS 0.610 cm, worst 0.814 cm
  [FAIL] distractor table mapping: RMS 1.018 cm, worst 1.909 cm; with only the true tape 1.018 cm

FAILED: table mapping, distractor table mapping
"""
CartMove = """======================================================================
FAIL: TestTheDroopsOfSeptember28StayBlocked (__main__.RealDroopTests.TestTheDroopsOfSeptember28StayBlocked)
----------------------------------------------------------------------
AssertionError: RECORDING MISSING: the 2026-09-28 droop captures
----------------------------------------------------------------------
Ran 17 tests in 41.1s

FAILED (failures=1)
"""
NodeFailure = """✖ failing tests:

test at TestX.mjs:10:1
✖ the plan holds (12.5ms)
  AssertionError [ERR_ASSERTION]: no
"""


class RuleTests(unittest.TestCase):
    def TestTheFailingChecksAreReadFromEachKindOfLog(Self):
        Self.assertEqual(Runner.FailedChecks(SelfCalibration), {"table mapping", "distractor table mapping"})
        Self.assertEqual(Runner.FailedChecks(CartMove), {"TestTheDroopsOfSeptember28StayBlocked"})
        Self.assertEqual(Runner.FailedChecks(NodeFailure), {"the plan holds"})

    def TestOnlyTheKnownChecksMakeAKnownFailure(Self):
        Self.assertTrue(Runner.Known("TestSelfCalibration.py", 1, SelfCalibration))
        Self.assertTrue(Runner.Known("TestCartMove.py", 1, CartMove))
        Self.assertFalse(Runner.Known("TestSelfCalibration.py", 1, SelfCalibration + "  [FAIL] tape found: 3.1 cm off\n"),
                         "a new failing check in the same suite")
        Self.assertFalse(Runner.Known("TestCartMove.py", 1, CartMove.replace("FAIL: TestThe", "FAIL: TestAnother")))
        Self.assertFalse(Runner.Known("TestCartMove.py", 1, "Traceback (most recent call last):\nImportError: x\n"), "a crash names no check")
        Self.assertFalse(Runner.Known("TestCartMove.py", "timeout", CartMove), "a timeout is no known failure")
        Self.assertFalse(Runner.Known("TestSafety.py", 1, CartMove), "another suite's check")
        Self.assertFalse(Runner.Known("TestCartMove.py", 0, CartMove), "a pass is no failure")

    def TestARunPassesWithKnownFailuresAndFlakesOnly(Self):
        Pass = {"result": "pass"}
        Self.assertTrue(Runner.Passed([Pass, {"result": "fail", "known": True}, {"result": "recording missing", "known": True},
                                       {"result": "flaky"}]))
        Self.assertFalse(Runner.Passed([Pass, {"result": "fail", "known": False}]))
        Self.assertFalse(Runner.Passed([Pass, {"result": "timeout"}]))

    def TestAFailedUiSuiteRunsOnceMoreAndAPassThenIsAFlake(Self):
        Logs = Path(tempfile.mkdtemp(prefix="TestRunAllTests"))

        def Fake(Outcomes):
            def RunSuite(Lane, Name, Argv, Cwd, Logs_, Env):
                (Logs_ / f"{Name}.log").write_text("log", encoding="utf-8")
                Result = Outcomes.pop(0)
                return {"suite": Name, "lane": Lane, "result": Result, "known": False, "seconds": 1.0,
                        "fail_lines": ["locator.waitFor: Timeout 120000ms exceeded."] if Result != "pass" else []}
            return RunSuite
        for Lane, Outcomes, Expected, Runs in (("ui", ["fail", "pass"], "flaky", 2), ("ui", ["fail", "fail"], "fail", 2),
                                               ("ui", ["pass"], "pass", 1), ("py", ["fail", "pass"], "fail", 1)):
            with Self.subTest(lane=Lane, outcomes=list(Outcomes)):
                Queue, Results = list(Outcomes), []
                with patch.object(Runner, "RunSuite", Fake(Queue)):
                    Runner.RunLane(Lane, [("TestRobotHud.mjs", ["node"], Logs)], Logs, {}, Results)
                Self.assertEqual([Row["result"] for Row in Results], [Expected])
                Self.assertEqual(len(Outcomes) - len(Queue), Runs)
                if Expected == "flaky":
                    Self.assertTrue((Logs / "TestRobotHud.mjs.first.log").is_file(), "the first run's log is kept")
                    Self.assertIn("Timeout", Results[0]["first"][0])


if __name__ == "__main__":
    Loader = unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Result = unittest.TextTestRunner(verbosity=2).run(Loader.loadTestsFromTestCase(RuleTests))
    raise SystemExit(not Result.wasSuccessful())
