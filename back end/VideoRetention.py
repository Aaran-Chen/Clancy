"""Session video is kept 7 days: a pass deletes older video, and nothing else (Aaran, 2026-10-08).

Only these are deleted, once they are more than KeepDays old by both the time in their name and their last write
(so a segment still being written, a file copied in recently, or a name that does not read as a time always stays):
  captures/YYYY-MM-DD/recording/video/HHMMSS[_mmm][-n]_<camera>.mp4 and .frames.jsonl
      the session recorder's segments and their frame indexes (SessionRecorder.py)
  captures/YYYY-MM-DD/HHMMSS_mmm[-n]_task.webm (or .mp4, .mkv, .bin) and .json
      the page's per-task clips and their sidecars (the bridge's POST /captures/video)
A segment and its index (a clip and its sidecar) go together, when every file of the pair is old enough.

Never deleted either, however old: the video that captures/video_keep.txt names, one path (relative to the captures
folder, a glob allowed) per line, # for a comment. KeepVideo.py fills it with the video around the servo faults the
bridge recorded, or around times you name; a segment or clip whose name is there is kept with its partner. Moving
video into a folder that is not a day (captures/keep/, say) keeps it too: no other folder is ever looked in.

Never deleted: everything else. That is the calibration and self-calibration archives (selfcal/, desk_remeasure/),
the text logs (ai.jsonl, bridge_events.jsonl, grounding.jsonl, route_timing.jsonl, recorder_events.jsonl, ...), the
images/ and depth/ folders the logs point into, the photos the AI saw, ReviewRecording's review/ output and anything
outside the captures folder. Only the two places above are looked in (no other subfolder is ever entered), and
links and junctions are never followed. Every deletion is logged in captures/video_retention.log.

The session recorder runs a pass when it starts and every hour (SessionRecorder.py). By hand:
  python VideoRetention.py              lists what a pass would delete now (deletes nothing)
  python VideoRetention.py --delete     deletes it, and logs each file
      [--root <captures folder>] [--days 7]
"""

from __future__ import annotations

import argparse
import datetime
import fnmatch
import os
import re
import sys
import time
from pathlib import Path
from typing import Any, Callable

import LogRotation

KeepDays = 7.0
LogName = "video_retention.log"
KeepName = "video_keep.txt"
LogCapBytes = 2_000_000
LogKeep = 3
DayName = re.compile(r"^\d{4}-\d{2}-\d{2}$")
# HHMMSS (the recorder's names before 2026-10-08) or HHMMSS_mmm, "-n" when that name was taken, then the camera.
SegmentName = re.compile(r"^(?P<unit>(?P<clock>\d{6})(?:_\d{3})?(?:-\d+)?_[a-z0-9]+)\.(?:mp4|frames\.jsonl)$")
TaskClipName = re.compile(r"^(?P<unit>(?P<clock>\d{6})_\d{3}(?:-\d+)?_task)\.(?:webm|mp4|mkv|bin|json)$")


def DefaultRoot() -> Path:
    """The captures folder the recorder writes (SO101_CAPTURES_ROOT points tests elsewhere)."""
    return Path(os.environ.get("SO101_CAPTURES_ROOT") or Path(__file__).resolve().parent / "captures")


def NameTime(Day: str, Clock: str) -> float | None:
    """Local time a name gives (the day folder and HHMMSS) as unix seconds; None when it is not a time."""
    try:
        return datetime.datetime.strptime(f"{Day} {Clock}", "%Y-%m-%d %H%M%S").timestamp()
    except ValueError:
        return None


def _Plain(Entry: os.DirEntry) -> bool:
    """Neither a link nor a junction (never followed: they could lead out of the captures folder)."""
    if Entry.is_symlink():
        return False
    IsJunction = getattr(Entry, "is_junction", None)
    return not (IsJunction() if IsJunction else os.path.isjunction(Entry.path))


def _Folders(Directory: Path) -> list[os.DirEntry]:
    try:
        with os.scandir(Directory) as Entries:
            return [Entry for Entry in Entries if _Plain(Entry) and Entry.is_dir(follow_symlinks=False)]
    except OSError:
        return []


def _Child(Directory: Path, Name: str) -> Path | None:
    """Directory/Name when it is a plain folder (no link, no junction)."""
    return next((Path(Entry.path) for Entry in _Folders(Directory) if Entry.name == Name), None)


def VideoUnits(Root: Path) -> list[dict[str, Any]]:
    """Every recorded video under Root, each with the files that go with it:
    {"folder", "unit" (the shared name), "named" (unix s from its name), "files": [Path, ...]}."""
    Root = Path(Root).resolve()
    Units: dict[tuple[str, str], dict[str, Any]] = {}
    for Day in _Folders(Root):
        if not DayName.match(Day.name):
            continue
        DayPath = Path(Day.path)
        Places = [(DayPath, TaskClipName)]
        Recording = _Child(DayPath, "recording")
        Video = _Child(Recording, "video") if Recording else None
        if Video:
            Places.append((Video, SegmentName))
        for Folder, Pattern in Places:
            try:
                with os.scandir(Folder) as Entries:
                    Files = [Entry for Entry in Entries if _Plain(Entry) and Entry.is_file(follow_symlinks=False)]
            except OSError:
                continue
            for Entry in Files:
                Match = Pattern.match(Entry.name)
                Named = NameTime(Day.name, Match["clock"]) if Match else None
                File = Path(Entry.path)
                if Named is None or not File.resolve().is_relative_to(Root):
                    continue
                Unit = Units.setdefault((str(Folder), Match["unit"]),
                                        {"folder": Folder, "unit": Match["unit"], "named": Named, "files": []})
                Unit["files"].append(File)
    return sorted(Units.values(), key=lambda Unit: (Unit["named"], str(Unit["folder"]), Unit["unit"]))


def KeepPatterns(Root: Path) -> list[str]:
    """The paths and globs captures/video_keep.txt names (relative to the captures folder, "/" separated)."""
    try:
        Text = (Path(Root) / KeepName).read_text(encoding="utf-8-sig")
    except OSError:
        return []
    return [Line.strip().replace("\\", "/") for Line in Text.splitlines() if Line.strip() and not Line.strip().startswith("#")]


def IsKept(Relative: str, Patterns: list[str]) -> bool:
    """Whether a file (relative to the captures folder) is named by the keep list (fnmatch: case-insensitive here)."""
    return any(fnmatch.fnmatch(Relative, Pattern) for Pattern in Patterns)


def Stamp(When: float) -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(When))


def AppendLog(Root: Path, Lines: list[str]) -> None:
    """captures/video_retention.log, size-capped like the other logs (LogRotation)."""
    if not Lines:
        return
    Log = Path(Root) / LogName
    LogRotation.RotateLog(Log, LogCapBytes, LogKeep)
    with open(Log, "a", encoding="utf-8") as File:
        File.write("".join(Line + "\n" for Line in Lines))


def RunRetention(Root: Path, Now: float | None = None, Days: float = KeepDays, Delete: bool = True,
                 Say: Callable[[str], None] | None = None) -> dict[str, Any]:
    """One pass over Root: deletes (or, with Delete False, lists) the video older than Days, and logs each deletion
    in Root/video_retention.log. Returns {"checked_at", "kept_after", "deleted": [relative paths], "bytes",
    "errors": [...], "dry_run"}. Never raises for a file it cannot delete; that is logged and the pass goes on."""
    Now = time.time() if Now is None else float(Now)
    Root = Path(Root)
    Cutoff = Now - Days * 86400.0
    Report: dict[str, Any] = {"checked_at": Now, "kept_after": Cutoff, "deleted": [], "bytes": 0, "errors": [],
                              "dry_run": not Delete, "kept": []}
    if not Root.is_dir():
        return Report
    Resolved = Root.resolve()
    Keep = KeepPatterns(Root)
    Lines: list[str] = []
    for Unit in VideoUnits(Root):
        if Unit["named"] >= Cutoff:
            continue
        Names = [File.resolve().relative_to(Resolved).as_posix() for File in Unit["files"]]
        if any(IsKept(Name, Keep) for Name in Names):
            Report["kept"].extend(Names)  # named in video_keep.txt: kept with its partner, however old
            continue
        try:
            Stats = [(File, File.stat(follow_symlinks=False)) for File in Unit["files"]]
        except OSError:
            continue  # gone meanwhile
        if any(Stat.st_mtime >= Cutoff for _, Stat in Stats):
            continue
        for File, Stat in Stats:
            Relative = File.resolve().relative_to(Resolved).as_posix()
            Text = (f"{Relative} ({Stat.st_size / 1e6:.1f} MB, recorded {Stamp(Unit['named'])}, "
                    f"last written {Stamp(Stat.st_mtime)}, {(Now - Unit['named']) / 86400:.1f} days ago)")
            if Delete:
                try:
                    File.unlink()
                except OSError as Exc:
                    Report["errors"].append(f"{Relative}: {Exc}")
                    Lines.append(f"{Stamp(Now)}  could not delete {Relative}: {Exc}")
                    continue
                Lines.append(f"{Stamp(Now)}  deleted {Text}")
            elif Say:
                Say(f"would delete {Text}")
            Report["deleted"].append(Relative)
            Report["bytes"] += Stat.st_size
    if Delete and (Report["deleted"] or Report["errors"]):
        Lines.append(f"{Stamp(Now)}  pass: deleted {len(Report['deleted'])} files ({Report['bytes'] / 1e9:.2f} GB) "
                     f"recorded before {Stamp(Cutoff)}; {len(Report['errors'])} could not be deleted")
        try:
            AppendLog(Root, Lines)
        except OSError as Exc:
            Report["errors"].append(f"{LogName}: {Exc}")
    return Report


def Main(Arguments: list[str] | None = None) -> int:
    Parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    Parser.add_argument("--root", type=Path, default=None, help="the captures folder (default: back end/captures)")
    Parser.add_argument("--days", type=float, default=KeepDays)
    Parser.add_argument("--delete", action="store_true", help="delete (without it, only list what would go)")
    Args = Parser.parse_args(Arguments)
    Root = Args.root or DefaultRoot()
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    Report = RunRetention(Root, Days=Args.days, Delete=Args.delete, Say=print)
    Verb = "deleted" if Args.delete else "would delete"
    print(f"{Root}: {Verb} {len(Report['deleted'])} files ({Report['bytes'] / 1e9:.2f} GB) recorded before "
          f"{Stamp(Report['kept_after'])}" + (f"; {len(Report['errors'])} errors" if Report["errors"] else "")
          + (f"; {len(Report['kept'])} older files kept by {KeepName}" if Report["kept"] else ""))
    for Error in Report["errors"]:
        print(f"  could not delete {Error}")
    return 1 if Report["errors"] else 0


if __name__ == "__main__":
    raise SystemExit(Main())
