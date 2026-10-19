"""Measurements with the desk verified unchanged: every recorded box, through 74097be's MeasureBox and this one.

Since the camera may now be checked against the gripper tip with no calibrated desk in view, object_locator
measures those captures without the desk (DepthGeometry.WorkspaceSurfaces Desk=False) and runs the scene
tripwire only on desk-verified ones, each object counted once there. A capture whose desk was verified must
measure exactly as before. 74097be (the code this branch started from) runs here in a process of its own, from
`git archive 74097be "back end"`, on every box the 2026-10-03 and 2026-10-04 /locate records measured (703), with
the depth, mirroring, calibration and arm pose of its capture: with the automatic supports on (as every capture
that passed the camera gate is measured) and off, with the arm masked out as recorded and not at all. Key for
key the results must be equal. So must the scene tripwire's verdict on each recorded locate, on its recorded
diagnostics and on the ones measured here.

The no-desk measurement of the same boxes is reported, not compared: without the desk an object on it is
measured on the level surface around it, or withheld.

The recorded captures stay where they are (RecordedCaptures: in a git worktree, the main checkout's) and the calibration
is its tracked copy; without the recordings this fails, RECORDING MISSING, and without git (the old commit's code) it
fails too: it no longer skips.

Run:  python TestDeskMeasureGolden.py
"""
import hashlib
import io
import json
import math
import os
import subprocess
import sys
import tarfile
import time
import unittest
from pathlib import Path

import cv2
import numpy as np

os.environ["SO101_CAPTURES_ROOT"] = str(Path(__file__).resolve().parent / "TestArtifacts" / "SessionCaptures")
os.environ["SO101_DECLARED_SURFACES"] = str(Path(__file__).resolve().parent / "TestArtifacts" / "NoDeclaredSurfaces.json")

import CameraDrift as CD  # noqa: E402
import DepthGeometry as DG  # noqa: E402
import object_locator as Locator  # noqa: E402
import RecordedCaptures as RC  # noqa: E402
from TestDeskGateGolden import AsFitted, Captures, Recorded  # noqa: E402
from TestRobotReference import ArmFromRaw  # noqa: E402

Root = Path(__file__).resolve().parent
Days = ("2026-10-03", "2026-10-04")
OldCommit = "74097be"
Work = Root / "TestArtifacts" / "DeskMeasureGolden"

# The old MeasureBox and tripwire, run against the archive's modules only.
Runner = r'''
import json, math, sys
import cv2, numpy as np
sys.path.insert(0, sys.argv[1])
import CameraDrift, DepthGeometry as DG, object_locator as Locator
Spec = json.loads(open(sys.argv[2], encoding="utf-8").read())
def Normal(Value):
    if isinstance(Value, dict):
        return {K: Normal(V) for K, V in Value.items()}
    if isinstance(Value, (list, tuple)):
        return [Normal(V) for V in Value]
    if isinstance(Value, (np.floating, float)):
        return "NaN" if math.isnan(float(Value)) else float(Value)
    if isinstance(Value, np.integer):
        return int(Value)
    if isinstance(Value, np.bool_):
        return bool(Value)
    return Value
Out = {"boxes": {}, "tripwire": {}}
Depths = {}
for Case in Spec["boxes"]:
    Depth = Depths.get(Case["depth"])
    if Depth is None:
        Depths.clear()
        Depth = Depths[Case["depth"]] = cv2.imread(Case["depth"], cv2.IMREAD_UNCHANGED)
    Model = Spec["models"][Case["model"]]
    Surfaces = DG.WorkspaceSurfaces(CameraDrift.DisplayDeskPlane(Model), Model.get("table_z"), Declared=[], Automatic=Case["automatic"])
    Exclude = DG.ArmExclusion(Case["joints"]) if Case["joints"] else None
    Out["boxes"][Case["id"]] = Normal(DG.MeasureBox(Depth, Case["box"], Spec["intrinsics"], Model, float(Model["table_z"]), Case["mirrored"],
                                                    Surfaces=Surfaces, Exclude=Exclude))
for Case in Spec["tripwire"]:
    Out["tripwire"][Case["id"]] = Locator._SceneTripwire(Case["diagnostics"], Case["camera_check"])
    # The same locate as measured here: its boxes' results under their recorded labels and notes.
    Mine = [{**Item, **Out["boxes"][Key]} for Item, Key in zip(Case["notes"], Case["measured"])]
    Out["tripwire"][Case["id"] + "|measured"] = Locator._SceneTripwire(Mine, Case["camera_check"])
open(sys.argv[3], "w", encoding="utf-8").write(json.dumps(Out))
'''


def Normal(Value):
    if isinstance(Value, dict):
        return {Key: Normal(Item) for Key, Item in Value.items()}
    if isinstance(Value, (list, tuple)):
        return [Normal(Item) for Item in Value]
    if isinstance(Value, (np.floating, float)):
        return "NaN" if math.isnan(float(Value)) else float(Value)
    if isinstance(Value, np.integer):
        return int(Value)
    if isinstance(Value, np.bool_):
        return bool(Value)
    return Value


def Git(*Arguments) -> bytes | None:
    try:
        Done = subprocess.run(["git", "-C", str(Root.parent), *Arguments], capture_output=True, timeout=120)
    except (OSError, subprocess.TimeoutExpired):
        return None
    return Done.stdout if Done.returncode == 0 else None


def OldBackend() -> Path | None:
    """74097be's back end, extracted once under TestArtifacts."""
    Target = Work / OldCommit
    if (Target / "back end" / "DepthGeometry.py").is_file():
        return Target / "back end"
    Archive = Git("archive", "--format=tar", OldCommit, "back end")
    if not Archive:
        return None
    Target.mkdir(parents=True, exist_ok=True)
    with tarfile.open(fileobj=io.BytesIO(Archive)) as Tar:
        Tar.extractall(Target, filter="data")
    return Target / "back end"


def Locates() -> list[dict]:
    """The recorded /locates of Days with their boxes: TestDeskGateGolden.Recorded's capture rows plus each record."""
    ByDepth = {Row["depth"]: Row for Row in Recorded()}
    Out = []
    for Day in Days:
        Grounding = Captures / Day / "recording" / "grounding.jsonl"
        if not Grounding.is_file():
            continue
        for Line in Grounding.read_text(encoding="utf-8").splitlines():
            Record = json.loads(Line)
            Depth = str(Captures / Day / "recording" / str(Record.get("depth_file") or ""))
            if Record.get("kind") == "locate" and Record.get("depth_file") and Depth in ByDepth:
                Out.append({**ByDepth[Depth], "record": Record})
    return Out


class DeskMeasureGoldenTests(unittest.TestCase):
    @classmethod
    def setUpClass(Class):
        RC.Need(*(Captures / Day / "recording" / "grounding.jsonl" for Day in Days), What="the recorded captures of 2026-10-03 and 10-04")
        Class.Backend = OldBackend()
        if Class.Backend is None:
            raise AssertionError(f"git archive {OldCommit} failed: this replay needs git and that commit of this repo")
        # The calibration the captures were taken with: its tracked copy, as it was live (the desk re-measured 10-03).
        Live = RC.CalibrationOf20261002()
        Class.Models = {"live": Live, "fitted": AsFitted(Live)}
        Class.Intrinsics = {Key: Live[Key] for Key in ("fx", "fy", "cx", "cy", "dist")} | {"width": 1280, "height": 720}
        Class.Locates = Locates()
        Class.Boxes, Class.Trips = [], []
        Joints = {}
        for Index, Row in enumerate(Class.Locates):
            if Row["raws"] and Index not in Joints:
                Joints[Index] = [list(map(float, Point)) for Point in ArmFromRaw(Row["raws"])["joints"]]
            Diagnostics = [Item for Item in Row["record"].get("diagnostics") or [] if Item.get("box")]
            for Number, Item in enumerate(Diagnostics):
                for ModelKey in ("live", "fitted"):
                    for Automatic in (True, False):
                        for ArmKey, Arm in (("recorded", Joints.get(Index)), ("none", None)):
                            if ArmKey == "recorded" and Arm is None:
                                continue
                            Class.Boxes.append({"id": f"{Index}|{Number}|{ModelKey}|{Automatic}|{ArmKey}", "locate": Index, "item": Number,
                                                "model": ModelKey, "automatic": Automatic, "arm": ArmKey, "joints": Arm, "depth": Row["depth"],
                                                "box": Item["box"], "mirrored": Row["mirrored"], "label": Item.get("label")})
            if Row["record"].get("camera_check") and Diagnostics:
                Arm = "recorded" if Index in Joints else "none"
                Class.Trips.append({"id": f"{Index}|recorded", "diagnostics": Diagnostics, "camera_check": Row["record"]["camera_check"],
                                    "notes": [{"label": Item.get("label"), "box": Item["box"], **({"task_query_only": True} if Item.get("task_query_only") else {})}
                                              for Item in Diagnostics],
                                    "measured": [f"{Index}|{Number}|live|True|{Arm}" for Number in range(len(Diagnostics))]})
        Spec = {"boxes": Class.Boxes, "tripwire": Class.Trips, "models": Class.Models, "intrinsics": Class.Intrinsics}
        Text = json.dumps(Spec, sort_keys=True)
        Key = hashlib.sha256((Text + Runner).encode("utf-8")).hexdigest()[:16]
        Work.mkdir(parents=True, exist_ok=True)
        Golden = Work / f"Old_{OldCommit}_{Key}.json"
        if not Golden.is_file():
            (Work / f"Spec_{Key}.json").write_text(Text, encoding="utf-8")
            (Work / "OldRunner.py").write_text(Runner, encoding="utf-8")
            Started = time.time()
            Done = subprocess.run([sys.executable, str(Work / "OldRunner.py"), str(Class.Backend), str(Work / f"Spec_{Key}.json"), str(Golden)],
                                  capture_output=True, text=True, cwd=str(Work), timeout=3600,
                                  env=dict(os.environ, PYTHONDONTWRITEBYTECODE="1", SO101_DEPTH_CAMERA="0"))
            if Done.returncode != 0:
                raise AssertionError(f"{OldCommit}'s MeasureBox failed:\n{Done.stderr[-4000:]}")
            print(f"  {OldCommit} measured {len(Class.Boxes)} cases in {time.time() - Started:.0f} s")
        Class.Old = json.loads(Golden.read_text(encoding="utf-8"))
        Class.New, Class.NoDesk = {}, {}
        Started = time.time()
        Depths: dict[str, np.ndarray] = {}
        for Case in Class.Boxes:
            Depth = Depths.get(Case["depth"])
            if Depth is None:
                Depths.clear()
                Depth = Depths[Case["depth"]] = cv2.imread(Case["depth"], cv2.IMREAD_UNCHANGED)
            Model = Class.Models[Case["model"]]
            Exclude = DG.ArmExclusion(Case["joints"]) if Case["joints"] else None
            for Desk, Into in ((True, Class.New), (False, Class.NoDesk)):
                if not Desk and (Case["model"], Case["arm"]) != ("live", "recorded"):
                    continue
                Surfaces = DG.WorkspaceSurfaces(CD.DisplayDeskPlane(Model), Model.get("table_z"), Declared=[], Automatic=Case["automatic"], Desk=Desk)
                Into[Case["id"]] = Normal(DG.MeasureBox(Depth, Case["box"], Class.Intrinsics, Model, float(Model["table_z"]), Case["mirrored"],
                                                        Surfaces=Surfaces, Exclude=Exclude))
        print(f"  this checkout measured {len(Class.Boxes)} cases (and {len(Class.NoDesk)} without the desk) in {time.time() - Started:.0f} s")

    def TestEveryRecordedBoxMeasuresAsBefore(Self):
        Measured = Withheld = 0
        Boxes = {(Case["locate"], Case["item"]) for Case in Self.Boxes}
        for Case in Self.Boxes:
            with Self.subTest(case=Case["id"], label=Case["label"]):
                Self.assertEqual(Self.New[Case["id"]], Self.Old["boxes"][Case["id"]])
                if Self.New[Case["id"]]["ok"]:
                    Measured += 1
                else:
                    Withheld += 1
        Self.assertEqual(len(Boxes), 703, "the 2026-10-03/04 records hold 293 + 410 boxes")
        print(f"  {len(Boxes)} recorded boxes, {len(Self.Boxes)} cases (2 calibrations x automatic supports on/off x arm masked/not): "
              f"all equal to {OldCommit} key for key ({Measured} measured, {Withheld} withheld)")

    def TestTheTripwireDecidesAsBefore(Self):
        # On each recorded locate's own diagnostics, and on its boxes as measured here (automatic supports on, the arm
        # masked as recorded), with each object counted once (_Distinct): the verdict is the old one.
        Tripped, Twice = 0, 0
        for Case in Self.Trips:
            Mine = [{**Note, **Self.New[Key]} for Note, Key in zip(Case["notes"], Case["measured"])]
            for Name, Diagnostics in (("recorded", Case["diagnostics"]), ("measured", Mine)):
                Old = Self.Old["tripwire"][Case["id"] if Name == "recorded" else Case["id"] + "|measured"]
                New = Locator._SceneTripwire(Diagnostics, Case["camera_check"])
                with Self.subTest(locate=Case["id"], diagnostics=Name):
                    Self.assertEqual(list(New) if New else None, Old)
                Tripped += bool(New)
                Twice += len(Locator._Distinct(Diagnostics)) < len(Diagnostics)
        print(f"  tripwire on {len(Self.Trips)} recorded locates, their diagnostics as recorded and as measured here: "
              f"{Tripped} trips, {Twice} with an object measured twice, every verdict as {OldCommit}'s")

    def TestWithoutTheDeskTheDeskIsNoSupport(Self):
        # Reported, not compared: the same boxes measured as if no calibrated desk were in view.
        Same = Lost = Gained = Changed = 0
        Branches: dict[str, int] = {}
        for Case in Self.Boxes:
            if Case["id"] not in Self.NoDesk:
                continue
            Old, New = Self.New[Case["id"]], Self.NoDesk[Case["id"]]
            Self.assertNotEqual(New.get("support_branch"), "desk")
            Self.assertNotIn("desk_area", New)
            Self.assertNotIn("desk_ring_offset_cm", New)
            if New == Old:
                Same += 1
            elif Old["ok"] and not New["ok"]:
                Lost += 1
            elif New["ok"] and not Old["ok"]:
                Gained += 1
            else:
                Changed += 1
            if New["ok"]:
                Branches[New["support_branch"]] = Branches.get(New["support_branch"], 0) + 1
        print(f"  without the desk ({len(Self.NoDesk)} cases, live calibration, arm masked): {Same} unchanged, {Lost} no longer measured, "
              f"{Gained} newly measured, {Changed} measured differently; measured on {Branches}")


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestDeskMeasureGolden` runs the Test* methods too (unittest names this hook)."""
    Prefixed = unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return Prefixed.loadTestsFromTestCase(DeskMeasureGoldenTests)


if __name__ == "__main__":
    Started = time.time()
    Loader = unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Result = unittest.TextTestRunner(verbosity=2).run(Loader.loadTestsFromTestCase(DeskMeasureGoldenTests))
    print(f"{time.time() - Started:.0f} s")
    raise SystemExit(not Result.wasSuccessful())
