"""Video is kept 7 days (VideoRetention.py): what a pass deletes, and everything it must never touch; the video
captures/video_keep.txt names (KeepVideo.py writes it around the bridge's recorded faults) is never deleted.

Everything happens in a scratch captures folder (TestArtifacts/VideoRetention, git-ignored), never in back end/captures.
The pass runs at a fixed "now", 2026-10-15 12:00 local; video recorded 7.1 days before goes, 6.9 days before stays.

Run:  python TestVideoRetention.py
"""
import hashlib
import io
import json
import os
import shutil
import subprocess
import time
import unittest
from contextlib import redirect_stdout
from pathlib import Path

Scratch = Path(__file__).resolve().parent / "TestArtifacts" / "VideoRetention"
Root = Scratch / "captures"
Outside = Scratch / "Outside"
os.environ["SO101_CAPTURES_ROOT"] = str(Root)

import KeepVideo  # noqa: E402
import VideoRetention  # noqa: E402

Now = time.mktime((2026, 10, 15, 12, 0, 0, 0, 0, -1))
Old = Now - 7.1 * 86400
Young = Now - 6.9 * 86400
Day = lambda When: time.strftime("%Y-%m-%d", time.localtime(When))  # noqa: E731
Clock = lambda When: time.strftime("%H%M%S", time.localtime(When))  # noqa: E731


def IsJunction(Path_: Path) -> bool:
    return os.path.isjunction(Path_)


def RemoveTree(Folder: Path) -> None:
    """Delete a scratch tree without ever going through a junction (its target is left alone)."""
    if not Folder.exists():
        return
    for Current, Folders, _ in os.walk(Folder, topdown=True):
        for Name in list(Folders):
            Child = Path(Current) / Name
            if IsJunction(Child) or Child.is_symlink():
                os.rmdir(Child)
                Folders.remove(Name)
    shutil.rmtree(Folder)


def Make(Path_: Path, When: float, Content: bytes = b"x" * 64) -> Path:
    Path_.parent.mkdir(parents=True, exist_ok=True)
    Path_.write_bytes(Content)
    os.utime(Path_, (When, When))
    return Path_


def Junction(Link: Path, Target: Path) -> None:
    Link.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(["cmd", "/c", "mklink", "/J", str(Link), str(Target)], check=True, capture_output=True)
    assert IsJunction(Link)


def Snapshot() -> dict[str, str]:
    """Every file under the scratch folder (inside captures and outside it) with its content hash; junctions not followed."""
    Files = {}
    for Current, Folders, Names in os.walk(Scratch, topdown=True):
        Folders[:] = [Name for Name in Folders if not IsJunction(Path(Current) / Name)]
        for Name in Names:
            File = Path(Current) / Name
            Files[File.relative_to(Scratch).as_posix()] = hashlib.sha1(File.read_bytes()).hexdigest() + f"@{int(File.stat().st_mtime)}"
    return Files


def Build() -> set[str]:
    """The scratch captures folder; returns the files (relative to captures) a pass must delete."""
    RemoveTree(Scratch)
    D, C, Y = Day(Old), Clock(Old), Clock(Young)
    assert D == Day(Young), "both moments fall on 2026-10-08"
    Video = Root / D / "recording" / "video"
    Gone = set()

    def Expect(Path_: Path, When: float) -> None:
        Make(Path_, When)
        Gone.add(Path_.relative_to(Root).as_posix())

    # Recorder segments 7.1 days old: today's names, the names before 2026-10-08, and a "-1" taken name. All go, with their indexes.
    for Stem in (f"{C}_123_depthcam", f"{C}_webcam", f"{C}_123-1_depthcam"):
        Expect(Video / f"{Stem}.mp4", Old)
        Expect(Video / f"{Stem}.frames.jsonl", Old)
    # The page's per-task clips and their sidecars, 7.1 days old.
    for Stem, Extension in ((f"{C}_456_task", "webm"), (f"{C}_457_task", "mp4"), (f"{C}_458-1_task", "bin")):
        Expect(Root / D / f"{Stem}.{Extension}", Old)
        Expect(Root / D / f"{Stem}.json", Old)

    # 6.9 days old: kept.
    for Stem in (f"{Y}_123_depthcam", f"{Y}_124_webcam"):
        Make(Video / f"{Stem}.mp4", Young)
        Make(Video / f"{Stem}.frames.jsonl", Young)
    Make(Root / D / f"{Y}_456_task.webm", Young)
    Make(Root / D / f"{Y}_456_task.json", Young)
    # Old by its name but written 6.9 days ago (copied in, or still being appended): kept, and its partner with it.
    Make(Video / f"{C}_200_depthcam.mp4", Old)
    Make(Video / f"{C}_200_depthcam.frames.jsonl", Young)
    Make(Video / f"{C}_201_webcam.mp4", Young)
    Make(Root / D / f"{C}_202_task.webm", Young)
    # Young by its name, written long ago: kept.
    Make(Video / f"{Y}_203_depthcam.mp4", Old)
    Make(Video / f"{Y}_203_depthcam.frames.jsonl", Old)

    # Never deleted, however old: calibration and self-calibration archives (video-like names inside them too),
    # the text logs, the images and depth the logs point to, review output, the AI's photos, other files.
    for Relative in ("selfcal/runs/20261008-093000/camera_calibration.json", "selfcal/runs/20261008-093000/pose0_red_hit.jpg",
                     f"selfcal/runs/20261008-093000/{C}_123_depthcam.mp4", f"selfcal/{C}_459_task.webm",
                     "desk_remeasure/camera_calibration.json", f"desk_remeasure/{C}_460_task.json",
                     "recording/ai.jsonl", "recording/bridge_events.jsonl", "recording/grounding.jsonl",
                     "recording/route_timing.jsonl", "recording/recorder_events.jsonl", "recording/control_stalls.log",
                     "recording/images/0a1b2c.jpg", "recording/depth/0a1b2c.png", "recording/review/frame_093600.jpg",
                     f"recording/review/{C}_123_depthcam.mp4", f"recording/{C}_123_depthcam.mp4",
                     f"recording/video/notes.txt", f"recording/video/{C}_126_depthcam.mp4.bak",
                     f"recording/video/{C}_127_depthcam.avi", f"recording/video/old/{C}_128_depthcam.mp4",
                     f"{C}_461_vision.jpg", f"{C}_462_task.txt", f"{C}_task.webm", "notes.md"):
        Make(Root / D / Relative, Old)
    Make(Root / "recorder-output.log", Old)
    Make(Root / "video_retention.log", Old, b"2026-10-01 00:00:00  an earlier line\n")
    Make(Root / "not-a-day" / "recording" / "video" / f"{C}_123_depthcam.mp4", Old)
    Make(Root / "2026-13-45" / "recording" / "video" / f"{C}_123_depthcam.mp4", Old)
    Make(Root / D / "recording" / "video" / f"256199_123_depthcam.mp4", Old)  # not a time of day

    # Outside the captures folder, reached only through junctions: never followed, never touched.
    for Target in (Outside / "2026-10-01", Outside / "Elsewhere"):
        Make(Target / "recording" / "video" / f"{C}_123_depthcam.mp4", Old)
        Make(Target / "recording" / "video" / f"{C}_123_depthcam.frames.jsonl", Old)
        Make(Target / f"{C}_456_task.webm", Old)
        Make(Target / "video" / f"{C}_124_depthcam.mp4", Old)
    Junction(Root / "2026-10-01", Outside / "2026-10-01")              # a day folder that is a junction
    Junction(Root / "2026-10-02" / "recording", Outside / "Elsewhere")  # a day's recording folder
    Junction(Root / "2026-10-03" / "recording" / "video", Outside / "Elsewhere" / "video")  # a day's video folder
    Make(Outside / "2026-10-08" / "recording" / "video" / f"{C}_129_depthcam.mp4", Old)  # a sibling of captures
    return Gone


class RetentionTests(unittest.TestCase):
    def setUp(Self):
        Self.Gone = Build()

    def tearDown(Self):
        RemoveTree(Scratch)

    def TestTheDefaultIsSevenDaysAndTheRootFollowsTheEnvironment(Self):
        Self.assertEqual(VideoRetention.KeepDays, 7.0)
        Self.assertEqual(VideoRetention.DefaultRoot(), Root, "tests must never point at the real captures")

    def TestAListingDeletesNothing(Self):
        Before = Snapshot()
        Said = []
        Report = VideoRetention.RunRetention(Root, Now=Now, Delete=False, Say=Said.append)
        Self.assertEqual(set(Report["deleted"]), Self.Gone)
        Self.assertTrue(Report["dry_run"])
        Self.assertEqual(Snapshot(), Before, "a listing changes nothing, its log included")
        Self.assertEqual(len(Said), len(Self.Gone))
        Self.assertTrue(all(Line.startswith("would delete ") for Line in Said))
        # The command line lists by default too.
        Self.assertEqual(VideoRetention.Main(["--root", str(Root)]), 0)
        Self.assertEqual(Snapshot(), Before)

    def TestOnlyVideoOlderThanSevenDaysIsDeletedAndEachDeletionIsLogged(Self):
        Before = Snapshot()
        Report = VideoRetention.RunRetention(Root, Now=Now)
        Self.assertEqual(set(Report["deleted"]), Self.Gone)
        Self.assertEqual(Report["errors"], [])
        Self.assertEqual(Report["bytes"], 64 * len(Self.Gone))
        After = Snapshot()
        GoneHere = {f"captures/{Relative}" for Relative in Self.Gone}
        for Relative in GoneHere:
            Self.assertNotIn(Relative, After)
        # Everything else is exactly as it was, inside the captures folder and outside it, except the log, which grew.
        Untouched = {Key: Value for Key, Value in Before.items() if Key not in GoneHere and Key != "captures/video_retention.log"}
        Self.assertEqual({Key: After[Key] for Key in Untouched}, Untouched)
        Self.assertEqual(set(After) - set(Before), set())
        Log = (Root / "video_retention.log").read_text(encoding="utf-8").splitlines()
        Self.assertEqual(Log[0], "2026-10-01 00:00:00  an earlier line", "the log is appended to, never rewritten")
        Deleted = [Line for Line in Log if "  deleted " in Line]
        Self.assertEqual(len(Deleted), len(Self.Gone))
        for Relative in Self.Gone:
            Self.assertTrue(any(f"deleted {Relative} (" in Line for Line in Deleted), Relative)
        Self.assertTrue(all("7.1 days ago" in Line and "recorded 2026-10-08" in Line for Line in Deleted))
        Self.assertIn(f"pass: deleted {len(Self.Gone)} files", Log[-1])
        Self.assertEqual(len(Self.Gone), 12)
        # A second pass finds nothing more, and writes nothing.
        Again = VideoRetention.RunRetention(Root, Now=Now)
        Self.assertEqual(Again["deleted"], [])
        Self.assertEqual((Root / "video_retention.log").read_text(encoding="utf-8").splitlines(), Log)

    def TestSixPointNineDaysStaysAndSevenPointOneGoes(Self):
        Report = VideoRetention.RunRetention(Root, Now=Now)
        Video = f"{Day(Old)}/recording/video"
        Self.assertIn(f"{Video}/{Clock(Old)}_123_depthcam.mp4", Report["deleted"])
        Self.assertTrue((Root / Video / f"{Clock(Young)}_123_depthcam.mp4").is_file())
        Self.assertTrue((Root / Day(Young) / f"{Clock(Young)}_456_task.webm").is_file())
        # Kept by its last write while that is recent, whatever its name says, and its partner with it.
        Self.assertTrue((Root / Video / f"{Clock(Old)}_200_depthcam.mp4").is_file())
        Self.assertTrue((Root / Video / f"{Clock(Old)}_200_depthcam.frames.jsonl").is_file())
        # The same files a little later: what was 6.9 days old is now 7.1 and goes too, the pair included.
        Later = VideoRetention.RunRetention(Root, Now=Now + 0.2 * 86400)
        Self.assertIn(f"{Video}/{Clock(Young)}_123_depthcam.mp4", Later["deleted"])
        Self.assertIn(f"{Day(Young)}/{Clock(Young)}_456_task.webm", Later["deleted"])
        Self.assertIn(f"{Video}/{Clock(Old)}_200_depthcam.mp4", Later["deleted"])
        Self.assertIn(f"{Video}/{Clock(Old)}_200_depthcam.frames.jsonl", Later["deleted"])

    def TestAFileThatCannotBeDeletedIsLoggedAndThePassGoesOn(Self):
        Held = Root / Day(Old) / "recording" / "video" / f"{Clock(Old)}_123_depthcam.mp4"
        with open(Held, "rb"):  # open in this process: Windows refuses to delete it
            Report = VideoRetention.RunRetention(Root, Now=Now)
        Self.assertEqual(len(Report["errors"]), 1)
        Self.assertIn(f"{Clock(Old)}_123_depthcam.mp4", Report["errors"][0])
        Self.assertEqual(len(Report["deleted"]), len(Self.Gone) - 1)
        Self.assertTrue(Held.is_file())
        Log = (Root / "video_retention.log").read_text(encoding="utf-8")
        Self.assertIn(f"could not delete {Day(Old)}/recording/video/{Clock(Old)}_123_depthcam.mp4", Log)
        Self.assertIn("1 could not be deleted", Log)

    def TestAMissingRootIsNotAnError(Self):
        Report = VideoRetention.RunRetention(Scratch / "no-such-captures", Now=Now)
        Self.assertEqual((Report["deleted"], Report["errors"]), ([], []))


class KeepTests(unittest.TestCase):
    """2026-10-08 regression review: the merge starts the new recorder and its retention at once (Vite restarts on the
    changed vite.config.ts), and the servo-fault video of 10-03 to 10-06 (audit 2 M1) would be deleted within days.
    What captures/video_keep.txt names stays, however old; KeepVideo.py names the video around the recorded faults."""

    def setUp(Self):
        Self.Gone = Build()

    def tearDown(Self):
        RemoveTree(Scratch)

    def TestVideoTheKeepListNamesIsNeverDeleted(Self):
        D, C = Day(Old), Clock(Old)
        Lines = ["# kept for the power review", "",
                 f"{D}/recording/video/{C}_123_depthcam.mp4",  # one file: its index goes with it
                 f"{D}" + "\\" + f"{C}_45*_task.*"]  # a glob, written with a backslash
        (Root / VideoRetention.KeepName).write_text("\n".join(Lines) + "\n", encoding="utf-8")
        Kept = {f"{D}/recording/video/{C}_123_depthcam.mp4", f"{D}/recording/video/{C}_123_depthcam.frames.jsonl"}
        Kept |= {f"{D}/{C}_{Stem}_task.{Extension}" for Stem, Extensions in (("456", ("webm", "json")), ("457", ("mp4", "json")),
                                                                            ("458-1", ("bin", "json"))) for Extension in Extensions}
        Report = VideoRetention.RunRetention(Root, Now=Now)
        Self.assertEqual(set(Report["kept"]), Kept)
        Self.assertEqual(set(Report["deleted"]), Self.Gone - Kept)
        for Relative in Kept:
            Self.assertTrue((Root / Relative).is_file(), Relative)
        Self.assertTrue(Kept <= set(VideoRetention.RunRetention(Root, Now=Now + 400 * 86400)["kept"]), "a year on, still kept")
        Self.assertTrue(all((Root / Relative).is_file() for Relative in Kept))
        Self.assertEqual(set(VideoRetention.RunRetention(Root, Now=Now, Delete=False)["kept"]), Kept, "a listing reports them too")

    def TestKeepVideoNamesTheVideoAroundEveryRecordedFault(Self):
        RemoveTree(Scratch)
        Fault = Now - 9 * 86400  # 2026-10-06 12:00: retention would delete everything around it
        D = Day(Fault)
        Video = Root / D / "recording" / "video"

        def Segment(Start, End, Camera):
            Stem = f"{Clock(Start)}_000_{Camera}"
            Make(Video / f"{Stem}.mp4", End)
            Make(Video / f"{Stem}.frames.jsonl", End)
            return {f"{D}/recording/video/{Stem}.mp4", f"{D}/recording/video/{Stem}.frames.jsonl"}
        During = Segment(Fault - 300, Fault + 300, "depthcam") | Segment(Fault - 120, Fault + 480, "webcam")
        Earlier = Segment(Fault - 3600, Fault - 1800, "depthcam")   # ended half an hour before
        Later = Segment(Fault + 600, Fault + 1200, "webcam")        # began 10 minutes after (the window ends 2 after)
        Clip = {f"{D}/{Clock(Fault - 60)}_001_task.webm", f"{D}/{Clock(Fault - 60)}_001_task.json"}
        for Relative in Clip:
            Make(Root / Relative, Fault + 30)
        Events = [{"kind": "motion_fault", "reason": "Servo protection/torque fault (elbow flex torque unexpectedly off)", "t": Fault},
                  {"kind": "grasp_hold", "reason": "not a fault", "t": Fault - 2700}]
        (Root / D / "recording" / "bridge_events.jsonl").write_text("".join(json.dumps(Event) + "\n" for Event in Events), encoding="utf-8")
        Said = io.StringIO()
        with redirect_stdout(Said):
            Self.assertEqual(KeepVideo.Main(["--root", str(Root)]), 0)
        Self.assertIn(f"1 moments, {len(During | Clip)} files would be kept", Said.getvalue())
        Self.assertFalse((Root / VideoRetention.KeepName).exists(), "a listing writes nothing")
        with redirect_stdout(Said):
            Self.assertEqual(KeepVideo.Main(["--root", str(Root), "--write"]), 0)
            Self.assertEqual(KeepVideo.Main(["--root", str(Root), "--write"]), 0)
        Text = (Root / VideoRetention.KeepName).read_text(encoding="utf-8")
        Self.assertIn("elbow flex torque unexpectedly off", Text)
        Self.assertEqual(sum(1 for Line in Text.splitlines() if Line and not Line.startswith("#")), len(During | Clip), "written once")
        # A named time keeps what the faults did not: the earlier segment, around 11:15.
        At = time.strftime("%Y-%m-%d %H:%M", time.localtime(Fault - 2700))
        with redirect_stdout(Said):
            Self.assertEqual(KeepVideo.Main(["--root", str(Root), "--at", At, "--before", "1", "--after", "1", "--write"]), 0)
        Report = VideoRetention.RunRetention(Root, Now=Now)
        Self.assertEqual(set(Report["kept"]), During | Clip | Earlier)
        Self.assertEqual(set(Report["deleted"]), Later)

    def TestTheRecordersFirstPassKeepsTheFaultVideoOnce(Self):
        # What the recorder's supervisor runs before its first deletion (KeepFaultsOnce): a merge that starts the new
        # recorder before anyone ran KeepVideo.py still keeps the fault video; once only, so a line removed stays removed.
        RemoveTree(Scratch)
        Self.assertIsNone(KeepVideo.KeepFaultsOnce(Root, Now), "no captures folder yet: nothing, and no marker")
        Fault = Now - 9 * 86400
        D = Day(Fault)
        Video = Root / D / "recording" / "video"
        Stems = {Name: f"{Clock(Start)}_000_{Name}" for Name, Start in (("depthcam", Fault - 300), ("webcam", Fault - 7200))}
        for Name, Stem in Stems.items():
            End = Fault + 300 if Name == "depthcam" else Fault - 3600  # the webcam segment ended an hour before the fault
            Make(Video / f"{Stem}.mp4", End)
            Make(Video / f"{Stem}.frames.jsonl", End)
        Event = {"kind": "protection_trip", "reason": "Servo protection/torque fault (wrist flex)", "t": Fault}
        (Root / D / "recording" / "bridge_events.jsonl").write_text(json.dumps(Event) + "\n", encoding="utf-8")
        Around = {f"{D}/recording/video/{Stems['depthcam']}.mp4", f"{D}/recording/video/{Stems['depthcam']}.frames.jsonl"}
        Self.assertEqual(set(KeepVideo.KeepFaultsOnce(Root, Now)), Around)
        Self.assertTrue((Root / KeepVideo.FirstPassName).is_file())
        Self.assertIn("wrist flex", (Root / VideoRetention.KeepName).read_text(encoding="utf-8"))
        Report = VideoRetention.RunRetention(Root, Now=Now)
        Self.assertEqual(set(Report["kept"]), Around)
        Self.assertEqual({Path(Item).name for Item in Report["deleted"]}, {f"{Stems['webcam']}.mp4", f"{Stems['webcam']}.frames.jsonl"})
        # Once: with the list emptied by hand, the next start keeps nothing again and the pass may delete it.
        (Root / VideoRetention.KeepName).write_text("# emptied by hand\n", encoding="utf-8")
        Self.assertIsNone(KeepVideo.KeepFaultsOnce(Root, Now))
        Self.assertEqual(set(VideoRetention.RunRetention(Root, Now=Now)["deleted"]), Around)


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestVideoRetention` runs the Test* methods too (unittest names this hook)."""
    Prefixed = unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return unittest.TestSuite(Prefixed.loadTestsFromTestCase(Case) for Case in (RetentionTests, KeepTests))


if __name__ == "__main__":
    Loader = unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Result = unittest.TextTestRunner(verbosity=2).run(unittest.TestSuite(Loader.loadTestsFromTestCase(Case) for Case in (RetentionTests, KeepTests)))
    raise SystemExit(not Result.wasSuccessful())
