"""Every test suite of this repo, offline, with one command (audit 2 L7: there was none).

  python RunAllTests.py                        # every lane, one suite at a time (about an hour)
  python RunAllTests.py --lanes py,node        # some lanes: py, node, ui, cad
  python RunAllTests.py --only TestSafety.py,TestGraspPlan.mjs
  python RunAllTests.py --parallel             # the py and node lanes side by side (two suites at once), then ui
  python RunAllTests.py --tag before-merge     # the logs' folder name (default: date and time)

Lanes:
  node  tsc on src and on vite.config.ts + scripts, then every AI Controlled WebApp/Test*.mjs that opens no browser;
  ui    every Playwright suite, one at a time, each serving this checkout on its own port of 5191-5199 (UiTestLib.mjs);
  py    every back end/Test*.py and test_bridge_home.py;
  cad   Cad/Test*.py (the CAD package and the reference kinematics against MuJoCo), with Cad/.venv's Python (this
        checkout's, else the main checkout's; Cad/ReadMe.md says how to make it) and the package in
        Downloads/SO101Cad. Run through unittest, not their __main__, which writes its report into that package.
Order: py, cad, node, ui, and last TestAutomaticWristBridge.py, which reads what the UI suite TestAutomaticWristUi leaves.

Nothing real is touched: the depth camera, the session recorder and Ollama autostart are off, every service URL points
at a closed port, session records go to back end/TestArtifacts/RunAllCaptures, TestRouteCheckPool starts 2 workers
(SO101_TEST_ROUTE_WORKERS), and the browsers map the live ports (the app 5181, the bridge 8000, the depth server 8001,
the recorder 8002, Ollama 11434) to a closed port and use Chromium's fake camera. The replays read the recorded
captures where back end/RecordedCaptures.py finds them (read only). Suites run at below-normal priority, so the live
bridge on this PC never waits for them.

Logs: back end/TestArtifacts/RunAllTests/<tag>/<suite>.log, results.jsonl and Summary.md (the table). A suite's result
is pass, fail, timeout, "recording missing": it failed only because recordings its replays need are not on this
machine (each such failure says RECORDING MISSING and what to restore), or "not set up": a cad suite without Cad/.venv.
A failing suite is "known" when every check that failed is one KnownFailures lists for it (by its exact name): the run
passes with those, and fails on anything else, a new failing check inside the same suite included. A UI suite that
fails is run once more; passing then, it is "flaky" (both logs kept), which does not fail the run but is listed.
"""
from __future__ import annotations

import json
import os
import re
import subprocess
import sys
import threading
import time
from pathlib import Path

Repo = Path(__file__).resolve().parent
App = Repo / "AI Controlled WebApp"
Backend = Repo / "back end"
Cad = Repo / "Cad"
Closed = "http://127.0.0.1:9"
# Slow suites get more time; the rest share the default.
Timeouts = {"TestGraspPlan.mjs": 3600, "TestAiTaskSafetyUi.mjs": 2400, "TestNoScroll.mjs": 1800, "TestSemiCalibrationUi.mjs": 1800,
            "TestGraspReachRefusal.mjs": 1800, "TestGraspReplay.mjs": 1800}
DefaultTimeout = 1200
Last = "TestAutomaticWristBridge.py"
# Failures known and accepted for now, by suite and by each failing check's exact name (unittest's "FAIL: <name>", a
# script's "[FAIL] <name>:"), with why and who decides. Without this every run exited 1 on these, so a new failure inside
# the same suites looked the same as today (2026-10-08 regression review).
KnownFailures = {
    "TestSelfCalibration.py": {"checks": {"table mapping", "distractor table mapping"},
                               "why": "RMS 0.610 / 1.018 cm against 0.6 / 1.0 since 446462d moved the Facing calibration poses; "
                                      "where those poses go is Aaran's decision (PROGRESS.md, Stage 4)"},
    "TestCartMove.py": {"checks": {"TestTheDroopsOfSeptember28StayBlocked"},
                        "why": "RECORDING MISSING: the 2026-09-28 captures were deleted on 2026-10-03; restore them or retire the replay (Aaran)"},
    "TestDepthCamera.py": {"checks": {"TestGateBlocksEveryCaptureOfTheFailedTask"},
                           "why": "RECORDING MISSING: the 2026-09-28 failed task's captures were deleted on 2026-10-03; restore or retire (Aaran)"},
}


def PythonExe() -> str:
    """The interpreter run-app.ps1 found (its note, .backend-python.txt), else this one."""
    Note = Repo / ".backend-python.txt"
    if Note.is_file():
        Text = Note.read_text(encoding="utf-8-sig").strip()
        if Text:
            return Text
    return sys.executable


def Environment() -> dict:
    Env = {Key: Value for Key, Value in os.environ.items() if Key != "CadTestUrl"}  # each UI suite serves itself
    Env.update({"SO101_DEPTH_CAMERA": "0", "SO101_RECORDER": "0", "SO101_OLLAMA_AUTOSTART": "0", "SO101_WEBCAM": "",
                "SO101_BRIDGE_URL": Closed, "OLLAMA_BASE_URL": Closed, "DEPTH_BASE_URL": Closed,
                "SO101_CAPTURES_ROOT": str(Backend / "TestArtifacts" / "RunAllCaptures"), "SO101_TEST_ROUTE_WORKERS": "2",
                "PYTHONIOENCODING": "utf-8", "PYTHONUTF8": "1", "PYTHONDONTWRITEBYTECODE": "1", "NO_COLOR": "1", "FORCE_COLOR": "0"})
    return Env


def CadPython() -> str | None:
    """Cad/.venv's interpreter (mujoco, trimesh, OCP): this checkout's, else, in a git worktree, the main checkout's."""
    Folders = [Repo]
    try:
        Common = subprocess.run(["git", "-C", str(Repo), "rev-parse", "--path-format=absolute", "--git-common-dir"],
                                capture_output=True, text=True, timeout=20).stdout.strip()
        if Common:
            Folders.append(Path(Common).resolve().parent)
    except (OSError, subprocess.TimeoutExpired):
        pass
    for Folder in Folders:
        Candidate = Folder / "Cad" / ".venv" / "Scripts" / "python.exe"
        if Candidate.is_file():
            return str(Candidate)
    return None


# A Cad suite's test case through unittest (method prefix "Test"), without the module's __main__, which writes its
# verification report into the CAD package in Downloads/SO101Cad: a test run must not rewrite that deliverable.
CadRunner = (
    "import sys, unittest\n"
    "sys.path.insert(0, '.')\n"
    "Loader = unittest.TestLoader(); Loader.testMethodPrefix = 'Test'\n"
    "Result = unittest.TextTestRunner(verbosity=2).run(Loader.loadTestsFromTestCase(getattr(__import__(sys.argv[1]), sys.argv[2])))\n"
    "sys.exit(0 if Result.wasSuccessful() else 1)\n")
NotSetUp = ("import sys; print('NOT SET UP: Cad/.venv is missing (mujoco, trimesh, OCP). Make it as Cad/ReadMe.md says: "
            "python -m venv Cad/.venv, then Cad/.venv/Scripts/python.exe -m pip install -r Cad/Requirements.txt'); sys.exit(1)")


def UsesBrowser(File: Path) -> bool:
    return re.search(r"""from ["']playwright["']""", File.read_text(encoding="utf-8")) is not None


def Suites() -> dict[str, list[tuple[str, list[str], Path]]]:
    Tsc = str(App / "node_modules" / "typescript" / "bin" / "tsc")
    Scripts = sorted(App.glob("Test*.mjs"))
    Node = [("tsc", ["node", Tsc, "-p", "tsconfig.json"], App),
            ("tsc-node", ["node", Tsc, "-p", "tsconfig.node.json", "--noEmit", "--composite", "false", "--incremental", "false"], App)]
    Node += [(File.name, ["node", File.name], App) for File in Scripts if not UsesBrowser(File)]
    Ui = [(File.name, ["node", File.name], App) for File in Scripts if UsesBrowser(File)]
    # Windows globs ignore case, so "Test*.py" also finds test_bridge_home.py; it is added once, by name.
    Names = sorted({File.name for File in Backend.glob("Test*.py")} - {"test_bridge_home.py"})
    Python = PythonExe()
    Py = [(Name, [Python, "-u", Name], Backend) for Name in Names + ["test_bridge_home.py"]]
    Venv = CadPython()
    CadSuites = []
    for File in sorted(Cad.glob("Test*.py")):
        Case = re.search(r"^class (\w+)\(\w+\.TestCase\)", File.read_text(encoding="utf-8"), re.M)
        Argv = [Venv, "-u", "-c", CadRunner, File.stem, Case.group(1)] if Venv and Case else [Python, "-c", NotSetUp]
        CadSuites.append((File.name, Argv, Cad))
    return {"py": Py, "cad": CadSuites, "node": Node, "ui": Ui}


def Counts(Text: str) -> dict:
    """Tests run / failed / skipped, from unittest's or node:test's summary, or from a script's own PASS line."""
    Found = {}
    Ran = re.findall(r"^Ran (\d+) tests? in", Text, re.M)
    if Ran:
        Found["tests"] = int(Ran[-1])
        Tail = (re.findall(r"^(OK|FAILED)(?: \(([^)]*)\))?\s*$", Text, re.M) or [("", "")])[-1]
        for Key, Value in re.findall(r"(failures|errors|skipped|expected failures)=(\d+)", Tail[1]):
            Found[Key] = int(Value)
    for Key in ("tests", "pass", "fail", "skipped", "cancelled"):
        Match = re.findall(rf"^(?:ℹ|#) {Key} (\d+)\s*$", Text, re.M)
        if Match:
            Found[Key if Key != "tests" else "tests"] = int(Match[-1])
    return Found


def FailureTexts(Text: str) -> list[str]:
    """Each failure's own text: unittest's FAIL/ERROR blocks, node:test's failing tests, a script's [FAIL] lines."""
    Blocks = [Block for Block in re.split(r"^={40,}$", Text, flags=re.M) if re.match(r"\s*(FAIL|ERROR):", Block)]
    if Blocks:
        return Blocks
    # node:test's spec reporter lists each failing test again under "✖ failing tests:", with its error, after a
    # "test at <file>:<line>" line.
    if "✖ failing tests:" in Text:
        Tail = Text.split("✖ failing tests:", 1)[1]
        Blocks = [Block for Block in re.split(r"^test at .*$", Tail, flags=re.M) if Block.strip().startswith("✖")]
        if Blocks:
            return Blocks
    # Otherwise a failing test is a "✖ name" line and the lines under it, up to the next test line.
    Tests = re.split(r"^\s*(?=[✔✖﹣] )", Text, flags=re.M)
    Failed = [Block for Block in Tests if Block.startswith("✖ ") and not Block.startswith("✖ failing tests")]
    if Failed:
        return Failed
    return [Line for Line in Text.splitlines() if re.search(r"\[FAIL\]|^FAILED|Error: |AssertionError", Line)]


def FailedChecks(Text: str) -> set[str]:
    """The names of the checks that failed: unittest's FAIL/ERROR lines, node:test's failing tests, a script's [FAIL] lines."""
    Names = set(re.findall(r"^(?:FAIL|ERROR): (\S+)", Text, re.M))
    Names |= {Name.strip() for Name in re.findall(r"^\s*\[FAIL\] ([^:\n]+)", Text, re.M)}
    if "✖ failing tests:" in Text:
        Names |= {Name.strip() for Name in re.findall(r"^✖ (.+?) \(\d+(?:\.\d+)?m?s\)\s*$", Text.split("✖ failing tests:", 1)[1], re.M)}
    return Names


def Known(Name: str, Code, Text: str) -> bool:
    """A failed suite whose every failing check is one KnownFailures lists for it (and at least one failed)."""
    Entry = KnownFailures.get(Name)
    if Code in (0, "timeout") or not Entry:
        return False
    Failed = FailedChecks(Text)
    return bool(Failed) and Failed <= Entry["checks"]


def Classify(Code, Text: str) -> str:
    if Code == "timeout":
        return "timeout"
    if Code == 0:
        return "pass"
    if "NOT SET UP:" in Text:
        return "not set up"
    Failures = FailureTexts(Text)
    return "recording missing" if Failures and all("RECORDING MISSING" in Item for Item in Failures) else "fail"


Lock = threading.Lock()


def RunSuite(Lane: str, Name: str, Argv: list[str], Cwd: Path, Logs: Path, Env: dict) -> dict:
    Log = Logs / f"{Name}.log"
    Timeout = Timeouts.get(Name, DefaultTimeout)
    Started = time.time()
    with open(Log, "w", encoding="utf-8", errors="replace") as Out:
        Out.write(f"# {' '.join(Argv)}  (cwd {Cwd})\n")
        Out.flush()
        try:
            Done = subprocess.run(Argv, cwd=Cwd, env=Env, stdout=Out, stderr=subprocess.STDOUT, timeout=Timeout,
                                  creationflags=getattr(subprocess, "BELOW_NORMAL_PRIORITY_CLASS", 0))
            Code = Done.returncode
        except subprocess.TimeoutExpired:
            Code = "timeout"
    Seconds = round(time.time() - Started, 1)
    Text = Log.read_text(encoding="utf-8", errors="replace")
    Result = {"suite": Name, "lane": Lane, "exit": Code, "seconds": Seconds, "result": Classify(Code, Text), "counts": Counts(Text),
              "known": Known(Name, Code, Text), "failed_checks": sorted(FailedChecks(Text))[:12],
              "missing": sorted(set(re.findall(r"RECORDING MISSING: ([^\n]{0,160})", Text)))[:6],
              "fail_lines": [Line.strip() for Line in Text.splitlines()
                             if re.search(r"^(FAIL|ERROR):|^not ok|^\s*\[FAIL\]|AssertionError|Error: |Timed out", Line)][:12],
              "tail": Text.splitlines()[-3:]}
    with Lock:
        with open(Logs / "results.jsonl", "a", encoding="utf-8") as Results:
            Results.write(json.dumps(Result) + "\n")
        print(f"[{Lane}] {Name}: {Result['result']} (exit {Code}) in {Seconds} s", flush=True)
    return Result


def RunLane(Lane: str, Items, Logs: Path, Env: dict, Results: list) -> None:
    for Name, Argv, Cwd in Items:
        Result = RunSuite(Lane, Name, Argv, Cwd, Logs, Env)
        if Lane == "ui" and Result["result"] != "pass" and not Result["known"]:
            # Once more, to tell a flake (TestRobotHud's CAD view once never came up after a reload, 2026-10-08, and
            # passed 13 times after) from a failure. The first run's log is kept beside the second's.
            First = Logs / f"{Name}.log"
            First.replace(Logs / f"{Name}.first.log")
            Again = RunSuite(Lane, Name, Argv, Cwd, Logs, Env)
            if Again["result"] == "pass":
                Again.update(result="flaky", first=Result["fail_lines"][:3], seconds=round(Result["seconds"] + Again["seconds"], 1))
            Result = Again
        Results.append(Result)


def Describe(Counts_: dict) -> str:
    if "tests" not in Counts_ and "pass" not in Counts_:
        return ""
    Total = Counts_.get("tests", Counts_.get("pass", 0) + Counts_.get("fail", 0))
    Bad = Counts_.get("failures", 0) + Counts_.get("errors", 0) + Counts_.get("fail", 0)
    Skipped = Counts_.get("skipped", 0)
    return f"{Total - Bad - Skipped}/{Total}" + (f", {Skipped} skipped" if Skipped else "")


def WriteSummary(Results: list, Logs: Path, Seconds: float) -> None:
    Order = {"py": 0, "cad": 1, "node": 2, "ui": 3}
    Rows = sorted(Results, key=lambda Row: (Order.get(Row["lane"], 9), Row["suite"].lower()))
    Tally = {}
    for Row in Rows:
        Key = f"{Row['result']} (known)" if Row.get("known") else Row["result"]
        Tally[Key] = Tally.get(Key, 0) + 1
    Lines = [f"# Every suite: {', '.join(f'{Count} {Name}' for Name, Count in sorted(Tally.items()))} ({len(Rows)} suites, {Seconds / 60:.0f} min)", "",
             "| Suite | Lane | Result | Tests | Seconds | Notes |", "|---|---|---|---|---|---|"]
    for Row in Rows:
        Note = "; ".join(Row["missing"]) if Row["result"] == "recording missing" else ("; ".join(Row["fail_lines"][:2]) if Row["result"] != "pass" else "")
        if Row.get("known"):
            Note = f"known: {', '.join(Row['failed_checks'])} ({KnownFailures[Row['suite']]['why']})"
        elif Row["result"] == "flaky":
            Note = "failed once, passed when run again (first run: " + "; ".join(Row.get("first") or []) + ")"
        Result = f"{Row['result']} (known)" if Row.get("known") else Row["result"]
        Lines.append(f"| {Row['suite']} | {Row['lane']} | {Result} | {Describe(Row['counts'])} | {Row['seconds']} | {Note.replace('|', '/')[:300]} |")
    (Logs / "Summary.md").write_text("\n".join(Lines) + "\n", encoding="utf-8")
    print("\n".join(Lines[:1]), f"\nSummary: {Logs / 'Summary.md'}", flush=True)


def Passed(Results: list) -> bool:
    """Whether a run passed: known failures (by suite and check) and flakes (a UI suite that passed when run again)
    do not fail it; anything else does."""
    return all(Row["result"] in ("pass", "flaky") or Row.get("known") for Row in Results)


def Option(Name: str, Default=None):
    return sys.argv[sys.argv.index(Name) + 1] if Name in sys.argv else Default


def Main() -> int:
    Only = set(Option("--only").split(",")) if Option("--only") else None
    Lanes = Option("--lanes", "py,cad,node,ui").split(",")
    Logs = Backend / "TestArtifacts" / "RunAllTests" / Option("--tag", time.strftime("%Y%m%d-%H%M%S"))
    Logs.mkdir(parents=True, exist_ok=True)
    (Logs / "results.jsonl").write_text("", encoding="utf-8")
    Env = Environment()
    Picked = {Lane: [Item for Item in Items if Only is None or Item[0] in Only] for Lane, Items in Suites().items() if Lane in Lanes}
    Final = [Item for Item in Picked.get("py", []) if Item[0] == Last]
    if "py" in Picked:
        Picked["py"] = [Item for Item in Picked["py"] if Item[0] != Last]
    Results: list = []
    Started = time.time()
    if "--parallel" in sys.argv:
        Threads = [threading.Thread(target=RunLane, args=(Lane, Picked.get(Lane, []), Logs, Env, Results)) for Lane in ("py", "node")]
        for Thread in Threads:
            Thread.start()
        for Thread in Threads:
            Thread.join()
    else:
        for Lane in ("py", "cad", "node"):
            RunLane(Lane, Picked.get(Lane, []), Logs, Env, Results)
    if "--parallel" in sys.argv:
        RunLane("cad", Picked.get("cad", []), Logs, Env, Results)
    RunLane("ui", Picked.get("ui", []), Logs, Env, Results)
    RunLane("py", Final, Logs, Env, Results)
    WriteSummary(Results, Logs, time.time() - Started)
    return 0 if Passed(Results) else 1


if __name__ == "__main__":
    raise SystemExit(Main())
