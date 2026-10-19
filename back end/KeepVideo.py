"""Keep the session video around chosen moments: the 7-day retention never deletes what captures/video_keep.txt names.

Why: video is kept 7 days (VideoRetention.py, Aaran's decision 3), and the recorder that deletes older video starts
the moment the dev server picks up the new vite.config.ts, which a merge does by itself (Vite restarts on a changed
config). The video of the servo faults behind audit 2 M1 (2026-10-03, -05 and -06) would be gone within days of that.
This marks such video as kept, by name, and copies, moves or deletes nothing.

  python KeepVideo.py                  lists the video around every servo fault and protection trip the bridge
                                       recorded (captures/<day>/recording/bridge_events.jsonl)
  python KeepVideo.py --write          adds those files to captures/video_keep.txt
  python KeepVideo.py --at "2026-10-03 10:00" [--at ...]
                                       the video around these local times instead
      [--root <captures folder>] [--before 10] [--after 2] (minutes around each moment)

A segment covers the time from its name (when it started) to its last write; one that overlaps a moment's window is
kept with its frame index, both cameras alike, and so is a per-task clip. Remove a line from video_keep.txt (or the
file) to let retention have it again.

The recorder does the same once by itself (KeepFaultsOnce), before its first retention pass on a captures folder
deletes anything, in case the merge came before anyone ran this: video_keep_first_pass.json there says it did.
"""

from __future__ import annotations

import argparse
import datetime
import json
import sys
import time
from pathlib import Path
from typing import Any

import VideoRetention

FaultKinds = ("motion_fault", "protection_trip")
FirstPassName = "video_keep_first_pass.json"
BeforeS, AfterS = 600.0, 120.0


def Moments(Root: Path, Kinds=FaultKinds) -> list[dict[str, Any]]:
    """The bridge's recorded faults: [{"t" (unix s), "what"}], from every day's recording/bridge_events.jsonl."""
    Found = []
    for Day in sorted(Path(Root).glob("*/recording/bridge_events.jsonl")):
        if not VideoRetention.DayName.match(Day.parent.parent.name):
            continue
        try:
            Lines = Day.read_text(encoding="utf-8", errors="replace").splitlines()
        except OSError:
            continue
        for Line in Lines:
            try:
                Record = json.loads(Line)
            except ValueError:
                continue
            if not isinstance(Record, dict) or Record.get("kind") not in Kinds:
                continue
            When = Record.get("t")
            if not isinstance(When, (int, float)):
                try:
                    When = datetime.datetime.fromisoformat(str(Record.get("time")).replace("Z", "+00:00")).timestamp()
                except ValueError:
                    continue
            Found.append({"t": float(When), "what": f"{Record['kind']}: {str(Record.get('reason') or '')[:90]}"})
    return sorted(Found, key=lambda Item: Item["t"])


def Around(Root: Path, Points: list[dict[str, Any]], Before: float, After: float) -> list[dict[str, Any]]:
    """The video units (VideoRetention.VideoUnits) overlapping Before..After seconds around any point, each with the
    point it is kept for: [{"files": [relative paths], "named", "end", "why"}]."""
    Resolved = Path(Root).resolve()
    Kept = []
    for Unit in VideoRetention.VideoUnits(Root):
        try:
            End = max(File.stat(follow_symlinks=False).st_mtime for File in Unit["files"])
        except OSError:
            continue
        Start = Unit["named"]
        Hit = next((Point for Point in Points if Start <= Point["t"] + After and End >= Point["t"] - Before), None)
        if Hit:
            Kept.append({"files": sorted(File.resolve().relative_to(Resolved).as_posix() for File in Unit["files"]),
                         "named": Start, "end": End, "why": Hit["what"], "at": Hit["t"]})
    return Kept


def Write(Root: Path, Units: list[dict[str, Any]], Now: float | None = None) -> list[str]:
    """Append the units' files that video_keep.txt does not name yet, under a comment saying why; returns them."""
    Keep = Path(Root) / VideoRetention.KeepName
    Named = set(VideoRetention.KeepPatterns(Root))
    Lines, Added = [], []
    for Unit in Units:
        New = [File for File in Unit["files"] if File not in Named]
        if not New:
            continue
        Lines.append(f"# around {VideoRetention.Stamp(Unit['at'])}: {Unit['why']}")
        Lines.extend(New)
        Named.update(New)
        Added.extend(New)
    if Lines:
        Header = "" if Keep.exists() else ("# Video the 7-day retention never deletes (VideoRetention.py), one path relative to "
                                          "this folder per line (a glob allowed). Written by KeepVideo.py and by hand.\n")
        with open(Keep, "a", encoding="utf-8") as File:
            File.write(Header + f"# KeepVideo.py, {VideoRetention.Stamp(time.time() if Now is None else Now)}\n"
                       + "".join(Line + "\n" for Line in Lines))
    return Added


def KeepFaultsOnce(Root: Path, Now: float | None = None) -> list[str] | None:
    """What the recorder's supervisor runs before its first retention pass on a captures folder deletes anything: the
    video around every recorded fault goes into video_keep.txt, as --write does, once (FirstPassName records it, so a
    line removed later stays removed). The go-live steps say to run this script before the merge, which starts the new
    recorder by itself; if nobody did, the fault video (audit 2 M1's servo faults of 2026-10-03 to -07) would still be
    kept (2026-10-08, Stage 5 verification). Returns the files added; None when it ran before or there is no captures
    folder yet. Raises when it cannot finish: that pass then deletes nothing."""
    Root = Path(Root)
    Marker = Root / FirstPassName
    if Marker.exists() or not Root.is_dir():
        return None
    Points = Moments(Root)
    Added = Write(Root, Around(Root, Points, BeforeS, AfterS), Now)
    When = time.time() if Now is None else Now
    Marker.write_text(json.dumps({"at": VideoRetention.Stamp(When), "faults": len(Points), "files_added": len(Added),
                                  "note": f"the recorder's first retention pass kept the video around these faults in "
                                          f"{VideoRetention.KeepName}; it never does this again here"}, indent=2) + "\n",
                      encoding="utf-8")
    return Added


def Main(Arguments: list[str] | None = None) -> int:
    Parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    Parser.add_argument("--root", type=Path, default=None, help="the captures folder (default: back end/captures)")
    Parser.add_argument("--at", action="append", default=[], help='a local time, "YYYY-MM-DD HH:MM[:SS]" (repeatable)')
    Parser.add_argument("--before", type=float, default=BeforeS / 60, help="minutes before each moment (default 10)")
    Parser.add_argument("--after", type=float, default=AfterS / 60, help="minutes after each moment (default 2)")
    Parser.add_argument("--write", action="store_true", help=f"add them to {VideoRetention.KeepName} (without it, only list)")
    Args = Parser.parse_args(Arguments)
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    Root = Args.root or VideoRetention.DefaultRoot()
    if Args.at:
        Points = []
        for Text in Args.at:
            try:
                Points.append({"t": datetime.datetime.fromisoformat(Text.strip()).timestamp(), "what": f"--at {Text.strip()}"})
            except ValueError:
                Parser.error(f"--at {Text!r} is not a local time like 2026-10-03 10:00")
    else:
        Points = Moments(Root)
    Units = Around(Root, Points, Args.before * 60, Args.after * 60)
    Files = [File for Unit in Units for File in Unit["files"]]
    for Unit in Units:
        print(f"{VideoRetention.Stamp(Unit['named'])} to {VideoRetention.Stamp(Unit['end'])}  {', '.join(Unit['files'])}  ({Unit['why']})")
    if Args.write:
        Added = Write(Root, Units)
        print(f"{len(Points)} moments, {len(Files)} files; {len(Added)} added to {Path(Root) / VideoRetention.KeepName}")
    else:
        print(f"{len(Points)} moments, {len(Files)} files would be kept (add --write to keep them)")
    return 0


if __name__ == "__main__":
    raise SystemExit(Main())
