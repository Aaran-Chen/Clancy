"""Size-capped logs (LogRotation.py): rotation, the cap on every kept log, the names, and a log held open elsewhere.

All in a scratch folder (TestArtifacts/LogRotation, git-ignored).

Run:  python TestLogRotation.py
"""
import shutil
import unittest
from pathlib import Path

import LogRotation

Scratch = Path(__file__).resolve().parent / "TestArtifacts" / "LogRotation"


def Lines(Count: int, Tag: str) -> bytes:
    return b"".join(f"{Tag} line {Index:06d}\n".encode() for Index in range(Count))


class RotationTests(unittest.TestCase):
    def setUp(Self):
        shutil.rmtree(Scratch, ignore_errors=True)
        Scratch.mkdir(parents=True)
        Self.Log = Scratch / "api-output.log"

    def tearDown(Self):
        shutil.rmtree(Scratch, ignore_errors=True)

    def TestALogUnderTheCapStays(Self):
        Self.Log.write_bytes(Lines(10, "a"))
        Self.assertEqual(LogRotation.RotateLog(Self.Log, 10_000, 3), "kept")
        Self.assertEqual(Self.Log.read_bytes(), Lines(10, "a"))
        Self.assertEqual(LogRotation.RotateLog(Scratch / "none.log", 10, 3), "missing")

    def TestOldLogsMoveUpAndOnlyKeepAreKept(Self):
        for Generation in range(5):
            # One launch each, under the cap: kept whole, as the PowerShell launchers do (Always).
            Self.Log.write_bytes(Lines(100, f"g{Generation}"))
            Self.assertEqual(LogRotation.RotateLog(Self.Log, 1_000_000, 3, Always=True), "rotated")
            Self.assertFalse(Self.Log.exists())
        Self.assertEqual(sorted(Path_.name for Path_ in Scratch.iterdir()), ["api-output.1.log", "api-output.2.log", "api-output.3.log"])
        # The newest generation is .1; the two before it follow; older ones are gone.
        for Number, Generation in ((1, 4), (2, 3), (3, 2)):
            Self.assertEqual((Scratch / f"api-output.{Number}.log").read_bytes(), Lines(100, f"g{Generation}"))

    def TestABigLogIsCutToItsNewestLinesSoNoKeptLogPassesTheCap(Self):
        Big = Lines(20_000, "poll")  # ~330 kB, the 337 MB access log in small
        Self.Log.write_bytes(Big)
        Self.assertEqual(LogRotation.RotateLog(Self.Log, 50_000, 3), "cut")
        Kept = (Scratch / "api-output.1.log").read_bytes()
        Self.assertLessEqual(len(Kept), 50_000)
        Self.assertGreater(len(Kept), 49_000)
        Self.assertTrue(Big.endswith(Kept), "the newest lines are kept")
        Self.assertTrue(Kept.startswith(b"poll line "), "from the start of a line")
        Self.assertFalse(Self.Log.exists())
        Self.assertFalse(any(Path_.name.endswith((".rotating", ".tmp")) for Path_ in Scratch.iterdir()))

    def TestAlwaysKeepsAnyLogThatIsNotEmpty(Self):
        Self.Log.write_bytes(b"")
        Self.assertEqual(LogRotation.RotateLog(Self.Log, 1_000, 3, Always=True), "kept", "an empty log is left alone")
        Self.Log.write_bytes(Lines(2, "small"))
        Self.assertEqual(LogRotation.RotateLog(Self.Log, 1_000, 3), "kept")
        Self.assertEqual(LogRotation.RotateLog(Self.Log, 1_000, 3, Always=True), "rotated")
        Self.assertEqual((Scratch / "api-output.1.log").read_bytes(), Lines(2, "small"))

    def TestTheSdkLogKeepsItsOwnNaming(Self):
        Sdk = Scratch / "OrbbecSDK.log.txt"
        Sdk.write_bytes(Lines(500, "sdk"))
        Self.assertEqual(LogRotation.RotateLog(Sdk, 1_000, 3), "cut")
        Self.assertTrue((Scratch / "OrbbecSDK.log.1.txt").is_file())
        Self.assertEqual(LogRotation.RotatedName(Scratch / "api-error.log", 2).name, "api-error.2.log")

    def TestALogHeldOpenElsewhereIsLeftExactlyAsItWas(Self):
        Self.Log.write_bytes(Lines(500, "live"))
        (Scratch / "api-output.1.log").write_bytes(b"one")
        (Scratch / "api-output.2.log").write_bytes(b"two")
        with open(Self.Log, "ab"):  # Windows: an open file cannot be renamed
            Result = LogRotation.RotateLog(Self.Log, 1_000, 3)
        Self.assertTrue(Result.startswith("busy"), Result)
        Self.assertEqual(Self.Log.read_bytes(), Lines(500, "live"))
        Self.assertEqual((Scratch / "api-output.1.log").read_bytes(), b"one", "older logs were not moved either")
        Self.assertEqual((Scratch / "api-output.2.log").read_bytes(), b"two")
        Self.assertFalse((Scratch / "api-output.3.log").exists())

    def TestTheCommandLine(Self):
        Self.Log.write_bytes(Lines(3000, "cli"))
        Error = Scratch / "api-error.log"
        Error.write_bytes(b"a short error log\n")
        Self.assertEqual(LogRotation.Main([str(Self.Log), str(Error), "--cap-mb", "0.01", "--keep", "2"]), 0)
        Self.assertLessEqual((Scratch / "api-output.1.log").stat().st_size, 10_000)
        Self.assertTrue(Error.is_file(), "under the cap: kept as it is")
        Self.assertEqual(LogRotation.Main([str(Error), "--always"]), 0)
        Self.assertEqual((Scratch / "api-error.1.log").read_bytes(), b"a short error log\n")


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestLogRotation` runs the Test* methods too (unittest names this hook)."""
    Prefixed = unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return Prefixed.loadTestsFromTestCase(RotationTests)


if __name__ == "__main__":
    Loader = unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Result = unittest.TextTestRunner(verbosity=2).run(Loader.loadTestsFromTestCase(RotationTests))
    raise SystemExit(not Result.wasSuccessful())
