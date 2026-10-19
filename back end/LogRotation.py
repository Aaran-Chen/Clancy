"""Size-capped logs with a few old ones kept (2026-10-07 audit 2 M4: a 337 MB bridge access log, a 102 MB camera SDK log).

RotateLog(Log, CapBytes, Keep): once Log is larger than CapBytes, the old logs move up one (name.1.log becomes
name.2.log, ...; the one past Keep is deleted) and Log becomes name.1.log. A log larger than the cap is cut to its
last CapBytes, from the start of a line, so no log kept is ever larger than the cap. The numbers go before the
extension, as the Orbbec SDK names its own (OrbbecSDK.log.txt -> OrbbecSDK.log.1.txt), so .log files stay .log.

Run it before a program opens the log for writing: Windows cannot rename a log another process holds open, and
RotateLog then leaves everything as it was and says so ("busy").

  python LogRotation.py <log> [<log> ...] [--cap-mb 20] [--keep 3] [--always]
      the launchers, before they start the bridge. --always keeps any log that is not empty, whatever its size: for
      the PowerShell launchers, whose Start-Process empties the log it writes to.
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path


def RotatedName(Log: Path, Number: int) -> Path:
    """name.log -> name.<Number>.log; OrbbecSDK.log.txt -> OrbbecSDK.log.<Number>.txt."""
    return Log.with_name(f"{Log.stem}.{Number}{Log.suffix}")


def RotateLog(Log: Path | str, CapBytes: int, Keep: int = 3, Always: bool = False) -> str:
    """Rotate Log when it is over CapBytes (or, with Always, when it is not empty). Returns what happened: "missing",
    "kept" (small enough), "rotated", "cut" (rotated, and its oldest part dropped to fit the cap) or "busy: <why>"
    (another process holds it open; nothing changed). Never raises."""
    Log = Path(Log)
    Keep = max(1, int(Keep))
    try:
        Size = Log.stat().st_size
    except FileNotFoundError:
        return "missing"
    except OSError as Exc:
        return f"busy: {Exc}"
    if Size == 0 or (Size <= CapBytes and not Always):
        return "kept"
    # Moved aside first: a log another process holds open cannot be renamed, and then nothing else changes either.
    Aside = Log.with_name(Log.name + ".rotating")
    try:
        os.replace(Log, Aside)
    except OSError as Exc:
        return f"busy: {Exc}"
    try:
        RotatedName(Log, Keep).unlink(missing_ok=True)
        for Number in range(Keep - 1, 0, -1):
            Older = RotatedName(Log, Number)
            if Older.exists():
                os.replace(Older, RotatedName(Log, Number + 1))
        First = RotatedName(Log, 1)
        if Size <= CapBytes:
            os.replace(Aside, First)
            return "rotated"
        with open(Aside, "rb") as File:
            File.seek(Size - CapBytes)
            Tail = File.read()
        LineStart = Tail.find(b"\n") + 1
        if 0 < LineStart < len(Tail):
            Tail = Tail[LineStart:]
        Temporary = First.with_name(First.name + ".tmp")
        Temporary.write_bytes(Tail)
        os.replace(Temporary, First)
        Aside.unlink()
        return "cut"
    except OSError as Exc:
        # Put the log back where its program looks for it rather than lose it.
        if Aside.exists() and not Log.exists():
            try:
                os.replace(Aside, Log)
            except OSError:
                pass
        return f"busy: {Exc}"


def Main(Arguments: list[str] | None = None) -> int:
    Parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    Parser.add_argument("logs", nargs="+", type=Path)
    Parser.add_argument("--cap-mb", type=float, default=20.0)
    Parser.add_argument("--keep", type=int, default=3)
    Parser.add_argument("--always", action="store_true", help="keep any log that is not empty, whatever its size")
    Args = Parser.parse_args(Arguments)
    for Log in Args.logs:
        Result = RotateLog(Log, int(Args.cap_mb * 1e6), Args.keep, Args.always)
        if Result not in ("missing", "kept"):
            print(f"{Log}: {Result}", file=sys.stderr if Result.startswith("busy") else sys.stdout)
    return 0


if __name__ == "__main__":
    raise SystemExit(Main())
