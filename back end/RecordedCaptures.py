"""Where the recorded captures the replay tests read are, and what a replay does when its recording is gone.

The replays (TestCartMove, TestDepthCamera, TestSelfCalibration, TestCalibrationRegion, TestRedTape,
TestRobotCameraReference, TestRobotReference, TestAutomaticScene, TestDeskGateGolden, TestDeskMeasureGolden, and the
app's TestCameraGateReplay.mjs and TestCabinetGrasp.mjs) run the camera gate, the desk check and the calibration on
real recordings in back end/captures, which git does not keep. On 2026-10-03 the recordings of 09-13 to 10-02 were
deleted (their selfcal/ folders survived), the replays that needed them skipped, and "OK (skipped=9)" read as green
for days (audit 2 H3). So:

- Root(): SO101_REAL_CAPTURES (SO101_REPLAY_CAPTURES is the older name) when set; else this checkout's
  back end/captures; in a git worktree, which has no captures of its own, the main checkout's (found with git).
  Read only: nothing here writes there.
- Need(*Paths, What=...): a replay names what it reads before it starts. Anything missing FAILS the test
  (RecordingMissing, an AssertionError) with "RECORDING MISSING: <what>: <path> ..." and what to do; RunAllTests.py
  lists a suite whose only failures are these as "recording missing". SO101_MISSING_RECORDINGS=skip makes them skips
  with the same words, for a machine that never had the recordings (a fresh clone, the Mac).
- Fixtures: back end/TestFixtures holds the few recorded files a replay keeps in git, so that deleting the captures
  cannot take them (Fixture()), and CalibrationOf20261002() the calibration the 2026-10-02..10-06 captures were taken
  with, from its tracked copy: a new calibration after the camera is remounted must not turn their replays into skips.

  python RecordedCaptures.py     says where the recordings are read from and which replay inputs are missing
"""
from __future__ import annotations

import json
import os
import subprocess
import unittest
from pathlib import Path

Here = Path(__file__).resolve().parent
Marker = "RECORDING MISSING"
Fixtures = Here / "TestFixtures"
# camera_calibration.json as committed in a9164a2: the 2026-10-02 21:09 fit, its desk re-measured on 2026-10-03 07:46.
Calibration20261002 = Here / "camera_calibration.cart-desk-2026-10-02.bak.json"


class RecordingMissing(AssertionError):
    """A replay's recording is not on this machine: a failure, not a skip (the module notes)."""


def _MainCheckout() -> Path | None:
    """The main checkout's root when this file is in a git worktree of it, else None."""
    try:
        Done = subprocess.run(["git", "-C", str(Here), "rev-parse", "--path-format=absolute", "--git-dir", "--git-common-dir"],
                              capture_output=True, text=True, timeout=20)
    except (OSError, subprocess.TimeoutExpired):
        return None
    Lines = Done.stdout.split() if Done.returncode == 0 else []
    if len(Lines) != 2 or Path(Lines[0]).resolve() == Path(Lines[1]).resolve():
        return None
    return Path(Lines[1]).resolve().parent


_Found: Path | None = None


def Root() -> Path:
    """The captures folder the replays read (module notes)."""
    global _Found
    Named = os.environ.get("SO101_REAL_CAPTURES") or os.environ.get("SO101_REPLAY_CAPTURES")
    if Named:
        return Path(Named)
    if _Found is None:
        Main = _MainCheckout()
        _Found = Main / "back end" / "captures" if Main is not None else Here / "captures"
    return _Found


def Day(Name: str) -> Path:
    """A day folder of the recordings, e.g. Day("2026-10-03")."""
    return Root() / Name


def Missing(*Paths) -> list[Path]:
    return [Path(Item) for Item in Paths if not Path(Item).exists()]


def MissingText(What: str, Absent: list[Path]) -> str:
    Shown = ", ".join(str(Item) for Item in Absent[:3]) + (f" and {len(Absent) - 3} more" if len(Absent) > 3 else "")
    Verb, It = ("is", "it") if len(Absent) == 1 else ("are", "them")
    return (f"{Marker}: {What}: {Shown} {Verb} not on this machine. Restore {It} (SO101_REAL_CAPTURES names another "
            f"captures folder), or set SO101_MISSING_RECORDINGS=skip on a machine that never had {It}.")


def Need(*Paths, What: str) -> None:
    """Fail (or, with SO101_MISSING_RECORDINGS=skip, skip) a replay whose recorded inputs are missing."""
    Absent = Missing(*Paths)
    if not Absent:
        return
    Text = MissingText(What, Absent)
    if os.environ.get("SO101_MISSING_RECORDINGS") == "skip":
        raise unittest.SkipTest(Text)
    raise RecordingMissing(Text)


def Fixture(*Parts: str) -> Path:
    """A recorded file kept in git (back end/TestFixtures/...)."""
    return Fixtures.joinpath(*Parts)


def CalibrationOf20261002() -> dict:
    """The camera model the 2026-10-02..10-06 captures were taken with (camera_calibration.json at a9164a2)."""
    Need(Calibration20261002, What="the tracked copy of the 2026-10-02 21:09 calibration")
    return json.loads(Calibration20261002.read_text(encoding="utf-8"))["camera_model"]


if __name__ == "__main__":
    print(f"Recorded captures read from: {Root()}")
    for Name in sorted(Path(Root()).glob("20??-??-??")) if Root().is_dir() else []:
        Parts = [Part for Part in ("recording", "selfcal") if (Name / Part).is_dir()]
        print(f"  {Name.name}: {', '.join(Parts) or 'only the day folder'}")
