"""Desk behaviour unchanged: every recorded capture with depth on disk, through 8f4d27a's CheckCapture and this one.

The live bridge of 2026-10-06 ran 8f4d27a. Its CheckCapture runs here in a process of its own, from
`git archive 8f4d27a "back end"` (its modules share this checkout's names), on the same depth, colour, arm pose
and calibration as this checkout's. Key for key the results must be equal, timing fields and the robot's shadow
(RobotReference.ShadowKey) aside, wherever the calibrated desk decided. Where it was out of view the old result
must be the new one's desk_check, and only the recorded no-desk captures may be among those: 2026-10-05 01:42Z
(in the 2026-10-04 folder) and 2026-10-06 15:01Z and 23:01Z.

Each capture runs with the calibration as it was live (its desk re-measured on 2026-10-03) and as fitted (the
calibration's own desk, from desk_history), and with the arm as recorded (the servo steps in the model request
after the capture, else the last route before it), the same arm at the calibration's wrist roll and jaw (so the
desk path's roll-gated tape judges), and no arm pose. The recorded captures stay where they are (RecordedCaptures:
in a git worktree, the main checkout's) and the calibration is its tracked copy; without the recordings this fails,
RECORDING MISSING, and without git (the old commit's code) it fails too: it no longer skips.

Since occluded-desk-check-2026-10-22 two changes are allowed, and only where they apply (AssertOnlyTheOcclusionAndRollChanges):
the cells the arm covers count neither way, which moves the desk's numbers wherever the arm hides some of it (no verdict
among these captures changes for it), and where the desk path's tape stopped at the wrist roll the roll-aware tape judges.
Each case is judged with no clear view of the desk remembered from the one before (CameraDrift.SessionChange): the arm
and model variants of one capture are no session. None of these captures reaches the occluded gates.

Since robot-verified-camera-2026-10-25 a desk in view that disagrees and that the cart-move path could not explain
(CameraDrift._DeskDiffers) is decided by the robot's witnesses (robot_mode "desk_differs"); there the desk path's result
rides along as desk_check, as before but for the two changes above. A desk pass may carry a rim record
(RobotReference.RimRecordKey), left out of the comparison like the shadow.

Run:  python TestDeskGateGolden.py
"""
import datetime
import hashlib
import io
import json
import math
import os
import re
import subprocess
import sys
import tarfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np

os.environ["SO101_CAPTURES_ROOT"] = str(Path(__file__).resolve().parent / "TestArtifacts" / "SessionCaptures")
os.environ["SO101_DECLARED_SURFACES"] = str(Path(__file__).resolve().parent / "TestArtifacts" / "NoDeclaredSurfaces.json")

import CameraDrift as CD  # noqa: E402
import RecordedCaptures as RC  # noqa: E402
import RobotCameraReference  # noqa: E402
import RobotReference as RR  # noqa: E402
from TestRobotReference import ArmFromRaw  # noqa: E402

Root = Path(__file__).resolve().parent
Captures = RC.Root()
Days = ("2026-10-03", "2026-10-04", "2026-10-06")
OldCommit = "8f4d27a"
Work = Root / "TestArtifacts" / "DeskGateGolden"
Names = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
# The captures' file names are local time; October 2026 in California is PDT.
LocalOffset = datetime.timezone(datetime.timedelta(hours=-7))
NoDeskCaptures = {"2026-10-05T01:42", "2026-10-06T15:01", "2026-10-06T23:01"}
Timing = ("check_ms", "tape_ms", "remeasure_ms", "rim_ms")
# The re-measured baseline a result carries holds the time it was made, and a checksum over that.
RemeasureClock = ("desk_remeasured_at", "sha256")

# The old CheckCapture, run against the archive's modules only.
Runner = r'''
import json, math, sys
import cv2, numpy as np
sys.path.insert(0, sys.argv[1])
import CameraDrift
Spec = json.loads(open(sys.argv[2], encoding="utf-8").read())
Timing, Clock = set(Spec["timing"]), set(Spec["remeasure_clock"])
def Normal(Value, Key=None):
    if isinstance(Value, dict):
        return {K: Normal(V, K) for K, V in Value.items() if K not in Timing and not (Key == CameraDrift.RemeasuredKey and K in Clock)}
    if isinstance(Value, (list, tuple)):
        return [Normal(V) for V in Value]
    if isinstance(Value, (np.floating, float)):
        return "NaN" if math.isnan(float(Value)) else float(Value)
    if isinstance(Value, (np.integer,)):
        return int(Value)
    if isinstance(Value, np.bool_):
        return bool(Value)
    return Value
Out = {}
for Case in Spec["cases"]:
    Depth = cv2.imread(Case["depth"], cv2.IMREAD_UNCHANGED)
    Image = cv2.imread(Case["image"]) if Case["image"] else None
    if Image is not None and Case["mirrored"]:
        Image = cv2.flip(Image, 1)
    Result = CameraDrift.CheckCapture(Depth, Spec["intrinsics"], Spec["models"][Case["model"]], Case["seq"], Image, Case["arm"])
    Out[Case["id"]] = Normal(Result)
open(sys.argv[3], "w", encoding="utf-8").write(json.dumps(Out))
'''


def Normal(Value, Key=None):
    """A camera_check as compared: timing fields and the shadow out, the re-measure's clock out, NaN as text."""
    if isinstance(Value, dict):
        return {K: Normal(V, K) for K, V in Value.items()
                if K not in Timing and K not in (RR.ShadowKey, RR.RimRecordKey) and not (Key == CD.RemeasuredKey and K in RemeasureClock)}
    if isinstance(Value, (list, tuple)):
        return [Normal(V) for V in Value]
    if isinstance(Value, (np.floating, float)):
        return "NaN" if math.isnan(float(Value)) else float(Value)
    if isinstance(Value, np.integer):
        return int(Value)
    if isinstance(Value, np.bool_):
        return bool(Value)
    return Value


def Git(*Arguments) -> bytes | None:
    """git in this checkout's root (pathspecs from there), or None when it fails."""
    try:
        Done = subprocess.run(["git", "-C", str(Root.parent), *Arguments], capture_output=True, timeout=120)
    except (OSError, subprocess.TimeoutExpired):
        return None
    return Done.stdout if Done.returncode == 0 else None


def OldBackend() -> Path | None:
    """8f4d27a's back end, extracted once under TestArtifacts."""
    Target = Work / OldCommit
    if (Target / "back end" / "CameraDrift.py").is_file():
        return Target / "back end"
    Archive = Git("archive", "--format=tar", OldCommit, "back end")
    if not Archive:
        return None
    Target.mkdir(parents=True, exist_ok=True)
    with tarfile.open(fileobj=io.BytesIO(Archive)) as Tar:
        Tar.extractall(Target, filter="data")
    return Target / "back end"


def Vision(Day: str) -> list[tuple[float, Path]]:
    """The archived captures of a day: (UTC epoch, file), from their local-time names."""
    Year, Month, Date = (int(Part) for Part in Day.split("-"))
    Rows = []
    for Path_ in sorted((Captures / Day).glob("*_vision.jpg")):
        Name = Path_.name
        Stamp = datetime.datetime(Year, Month, Date, int(Name[0:2]), int(Name[2:4]), int(Name[4:6]), int(Name[7:10]) * 1000, tzinfo=LocalOffset)
        Rows.append((Stamp.timestamp(), Path_))
    return Rows


def ArmRaws(Day: str) -> tuple[list[tuple[float, dict]], list[tuple[float, dict]]]:
    """Servo steps the session recorded: from each model request (its "Current servo positions"), and at the end of each route."""
    Requests, Routes = [], []
    for Line in (Captures / Day / "recording" / "ai.jsonl").read_text(encoding="utf-8").splitlines():
        Record = json.loads(Line)
        if Record.get("kind") == "model" and not Record.get("preload"):
            Raws = dict(re.findall(r"(shoulder_pan|shoulder_lift|elbow_flex|wrist_flex|wrist_roll|gripper): present_raw (\d+)",
                                   json.dumps(Record.get("request"))))
            if len(Raws) == len(Names):
                Requests.append((float(Record["t"]), {Name: int(Raws[Name]) for Name in Names}))
    Timings = Captures / Day / "recording" / "route_timing.jsonl"
    if Timings.is_file():
        for Line in Timings.read_text(encoding="utf-8").splitlines():
            Record = json.loads(Line)
            if Record.get("frames"):
                Routes.append((float(Record["t"]), dict(zip(Record["joints"], (int(Value) for Value in Record["frames"][-1][2])))))
    return Requests, Routes


def Recorded() -> list[dict]:
    """Every recorded /locate with its raw depth on disk: depth, colour, seq, the arm's raw steps if recorded."""
    Rows = []
    for Day in Days:
        Grounding = Captures / Day / "recording" / "grounding.jsonl"
        if not Grounding.is_file():
            continue
        Images, (Requests, Routes) = Vision(Day), ArmRaws(Day)
        Mirrored = {}
        for Line in (Captures / Day / "recording" / "ai.jsonl").read_text(encoding="utf-8").splitlines():
            Record = json.loads(Line)
            if Record.get("type") == "vision" and Record.get("depth_seq") is not None:
                Mirrored[Record["depth_seq"]] = bool(Record.get("mirrored"))
        for Line in Grounding.read_text(encoding="utf-8").splitlines():
            Record = json.loads(Line)
            Depth = Captures / Day / "recording" / str(Record.get("depth_file") or "")
            if Record.get("kind") != "locate" or not Record.get("depth_file") or not Depth.is_file():
                continue
            At = float(Record["t"])
            Before = [Path_ for Stamp, Path_ in Images if 0.0 <= At - Stamp <= 40.0]
            # The model request answers this capture: no motion between the two.
            After = [Raw for Stamp, Raw in Requests if 0.0 <= Stamp - At <= 90.0]
            Last = [Raw for Stamp, Raw in Routes if Stamp < At - 0.5]
            Rows.append({"time": Record["time"], "seq": Record.get("depth_seq"), "depth": str(Depth),
                         "image": str(Before[-1]) if Before else None, "mirrored": Mirrored.get(Record.get("depth_seq"), False),
                         "raws": After[0] if After else (Last[-1] if Last else None), "recorded": Record.get("camera_check")})
    return Rows


def AsFitted(Model: dict) -> dict:
    """The calibration with the desk it was fitted on (the oldest of desk_history), before any re-measure."""
    Reference = dict(Model["drift_reference"])
    History = Reference.pop("desk_history", None) or []
    for Key in ("desk_remeasured_at", "desk_remeasured_from", "sha256"):
        Reference.pop(Key, None)
    if History:
        Reference.update({Key: History[-1][Key] for Key in ("desk_n", "desk_d", "desk_map", "frames")})
    Reference["sha256"] = CD.Checksum(Reference, Model)
    return dict(Model, drift_reference=Reference)


class DeskGateGoldenTests(unittest.TestCase):
    @classmethod
    def setUpClass(Class):
        RC.Need(*(Captures / Day / "recording" / "grounding.jsonl" for Day in Days), What="the recorded captures of 2026-10-03, 10-04 and 10-06")
        Class.Backend = OldBackend()
        if Class.Backend is None:
            raise AssertionError(f"git archive {OldCommit} failed: this replay needs git and that commit of this repo")
        # The calibration the captures were taken with: its tracked copy, as it was live (the desk re-measured 10-03).
        Live = RC.CalibrationOf20261002()
        Class.Models = {"live": Live, "fitted": AsFitted(Live)}
        Class.Intrinsics = {Key: Live[Key] for Key in ("fx", "fy", "cx", "cy", "dist")} | {"width": 1280, "height": 720}
        Class.Rows = Recorded()
        Pinned = RR.ReferenceRaw(Live)
        Tape = Live["drift_reference"]["tape_pose"]
        Class.Cases = []
        for Index, Row in enumerate(Class.Rows):
            Arms = {"none": {"ok": False, "reason": "arm moving"}}
            if Row["raws"]:
                Arms["recorded"] = ArmFromRaw(Row["raws"], Pinned)
                Arms["calibration roll"] = ArmFromRaw(dict(Row["raws"], wrist_roll=Tape["wrist_roll_raw"], gripper=Tape["gripper_raw"]), Pinned)
            for ModelKey in Class.Models:
                for ArmKey, Arm in Arms.items():
                    Class.Cases.append({"id": f"{Index}|{ModelKey}|{ArmKey}", "row": Index, "model": ModelKey, "arm_key": ArmKey, "arm": Arm,
                                        "depth": Row["depth"], "image": Row["image"], "mirrored": Row["mirrored"], "seq": Row["seq"]})
        Spec = {"cases": [{Key: Case[Key] for Key in ("id", "model", "arm", "depth", "image", "mirrored", "seq")} for Case in Class.Cases],
                "models": Class.Models, "intrinsics": Class.Intrinsics, "timing": list(Timing), "remeasure_clock": list(RemeasureClock)}
        Text = json.dumps(Spec, sort_keys=True)
        Key = hashlib.sha256((Text + Runner).encode("utf-8")).hexdigest()[:16]
        Work.mkdir(parents=True, exist_ok=True)
        Golden = Work / f"Old_{OldCommit}_{Key}.json"
        if not Golden.is_file():
            (Work / f"Spec_{Key}.json").write_text(Text, encoding="utf-8")
            (Work / "OldRunner.py").write_text(Runner, encoding="utf-8")
            Started = time.time()
            Done = subprocess.run([sys.executable, str(Work / "OldRunner.py"), str(Class.Backend), str(Work / f"Spec_{Key}.json"), str(Golden)],
                                  capture_output=True, text=True, cwd=str(Work), timeout=1800,
                                  env=dict(os.environ, PYTHONDONTWRITEBYTECODE="1", SO101_DEPTH_CAMERA="0"))
            if Done.returncode != 0:
                raise AssertionError(f"{OldCommit}'s CheckCapture failed:\n{Done.stderr[-4000:]}")
            print(f"  {OldCommit} replayed {len(Class.Cases)} cases in {time.time() - Started:.0f} s")
        Class.Old = json.loads(Golden.read_text(encoding="utf-8"))
        Class.New = {}
        Started = time.time()
        Real = RobotCameraReference.Check
        for Case in Class.Cases:
            Depth = cv2.imread(Case["depth"], cv2.IMREAD_UNCHANGED)
            Image = cv2.imread(Case["image"]) if Case["image"] else None
            if Image is not None and Case["mirrored"]:
                Image = cv2.flip(Image, 1)
            # The RGB keyframe match (0.3 s a capture, advisory) runs on the live calibration with the recorded arm only.
            Keyframe = Real if (Case["model"], Case["arm_key"]) == ("live", "recorded") else (lambda *Arguments: None)
            # Each case judged on its own: no clear view of the desk kept from the one before (CameraDrift.SessionChange; the
            # arm and model variants of one capture are not a session).
            CD.ForgetSessionFrames()
            with patch.object(RobotCameraReference, "Check", Keyframe):
                Class.New[Case["id"]] = CD.CheckCapture(Depth, Class.Intrinsics, Class.Models[Case["model"]], Case["seq"], Image, Case["arm"])
        print(f"  this checkout replayed {len(Class.Cases)} cases in {time.time() - Started:.0f} s")

    def AssertOnlyTheOcclusionAndRollChanges(Self, Old, New):
        """Old and New equal key for key but for the two changes of occluded-desk-check-2026-10-22 (CameraDrift's
        module notes, "The arm over the desk"); which of them applied ("" for none, the result exactly as before):
        - "occlusion": the cells the arm covers stay out of the fit (desk_occluded_cells; desk_hidden_cells those not seen
          as desk), so the desk's numbers move;
          the verdict stays the old one, or the desk passes as matching where it is visible (desk_occluded);
        - "roll-aware": the old tape stopped at the wrist roll (its reason now tape_roll_gated_reason) and the roll-aware
          tape judged instead. Where the desk passed the verdict stays; in the cart-move path a desk_changed_unverified
          may become a re-measured desk (the tape where the rolled gripper puts it) or camera_moved (the tape off it)."""
        if New == Old:
            return ""
        Kinds, Desk, Tape = [], {"angle_deg", "offset_cm", "desk_cells", "desk_agreeing_share", "desk_occluded_cells", "desk_hidden_cells", "desk_beneath_share", "desk_beneath_of_desk_share", "desk_session", "desk_change", "reason", "error", CD.RemeasuredKey}, set()
        if New.get("desk_occluded_cells") or (New.get("desk_change") or {}).get("occluded_cells"):
            Kinds.append("occlusion")
        if "tape_roll_gated_reason" in New:
            Kinds.append("roll-aware")
            Self.assertEqual((Old.get("tape_check"), Old.get("tape_reason")), ("pose_differs", New["tape_roll_gated_reason"]))
            Self.assertTrue(Old["tape_reason"].startswith("wrist_roll"))
            Tape = {Key for Key in set(Old) | set(New) if Key.startswith("tape_")} | {"error", "reason"}
        Self.assertTrue(Kinds, "the result changed with neither the arm's cells nor the roll-aware tape to explain it")
        Verdict = lambda Value: (Value["ok"], Value.get("error_code"))
        if Verdict(New) != Verdict(Old):
            if New.get("desk_occluded"):
                Self.assertEqual(Verdict(New), (True, None))
            else:
                Self.assertIn("roll-aware", Kinds)
                Self.assertEqual(Old.get("error_code"), "desk_changed_unverified", (Old, New))
                Self.assertTrue((New["ok"] and New.get("desk_remeasured") and New["tape_check"] == "ok")
                                or (New.get("error_code"), New["tape_check"]) == ("camera_moved", "moved"), New)
                Tape |= {"ok", "error_code", "desk_remeasured", CD.RemeasuredKey, "plane_angle_deg", "desk_agreeing_share", "angle_deg", "offset_cm",
                         "desk_cells", "desk_change"}
        Allowed = (Desk if "occlusion" in Kinds else set()) | Tape | ({"desk_occluded", "desk_agreeing_share_unhidden", "desk_beneath_share", "desk_beneath_of_desk_share", "desk_session", "reason"} if New.get("desk_occluded") else set())
        Self.assertEqual({Key: Value for Key, Value in New.items() if Key not in Allowed},
                         {Key: Value for Key, Value in Old.items() if Key not in Allowed})
        return "+".join(Kinds) + (": verdict changed" if Verdict(New) != Verdict(Old) else "")

    def TestEveryCaptureWithTheDeskInViewIsUnchanged(Self):
        Present, Absent, Altered, Differs = 0, {}, {}, {}
        for Case in Self.Cases:
            Old, New = Self.Old[Case["id"]], Self.New[Case["id"]]
            Row = Self.Rows[Case["row"]]
            with Self.subTest(capture=Row["time"], model=Case["model"], arm=Case["arm_key"]):
                if New.get("robot_mode") == RR.DeskDiffers:
                    # A desk in view that disagrees, which the cart-move path could not explain (robot-verified-camera-2026-10-25):
                    # the robot's witnesses decided, the desk path's block (as before, but for the changes named) rides along.
                    Self.assertTrue(CD._DeskDiffers(New["desk_check"]), New["desk_check"])
                    Self.assertFalse(Old["ok"])
                    Self.assertNotEqual(Old.get("tape_check"), "moved")
                    Self.AssertOnlyTheOcclusionAndRollChanges(Old, Normal(New["desk_check"]))
                    Differs[(Row["time"][:16], Case["model"], Case["arm_key"])] = (New["robot_check"], New.get("tape_error_cm"), New.get("rim_check"))
                elif New.get("alignment_source") == "robot":
                    # The calibrated desk out of view: the old result rides along unchanged as desk_check.
                    Self.assertEqual(Normal(New["desk_check"]), Old)
                    Self.assertFalse(Old["ok"])
                    Self.assertNotEqual(Old.get("error_code"), "camera_moved")
                    Absent[(Row["time"][:16], Case["model"], Case["arm_key"])] = (New["robot_check"], New.get("tape_error_cm"))
                else:
                    Changed = Self.AssertOnlyTheOcclusionAndRollChanges(Old, Normal(New))
                    if Changed:
                        Altered[Changed] = Altered.get(Changed, 0) + 1
                    Self.assertIn(RR.ShadowKey, New)
                    Present += 1
        # As live, exactly the recorded no-desk captures go to the robot. (On the desk the calibration was fitted
        # on, before the 2026-10-03 cart move, a capture may see too little of it as well; checked the same way.)
        Self.assertEqual({Clock for Clock, ModelKey, _ in Absent if ModelKey == "live"}, NoDeskCaptures, Absent)
        print(f"  {len(Self.Rows)} captures, {len(Self.Cases)} cases: {Present} with the desk deciding, as before key for key but for the changes named; "
              f"{len(Absent)} with no calibrated desk in view; changed as occluded-desk-check-2026-10-22 means to: {Altered}")
        print("  no calibrated desk in view:")
        for (Clock, ModelKey, ArmKey), (Verdict, Error) in sorted(Absent.items()):
            print(f"    {Clock}Z  {ModelKey:6s} {ArmKey:16s} -> {Verdict}" + (f" (tape {Error} cm)" if Error is not None else ""))
        print(f"  a desk in view that disagrees, decided by the robot: {len(Differs)}")
        for (Clock, ModelKey, ArmKey), (Verdict, Error, Rim) in sorted(Differs.items()):
            print(f"    {Clock}Z  {ModelKey:6s} {ArmKey:16s} -> {Verdict} (tape {Error} cm, rim {Rim})")

    def TestTheLiveReplayMatchesWhatTheBridgeRecorded(Self):
        # With the arm as recorded, the old code reproduces the camera_check each locate recorded (where the
        # recording kept an arm pose and colour frame the bridge would have had).
        Matched = 0
        for Case in Self.Cases:
            Row = Self.Rows[Case["row"]]
            if (Case["model"], Case["arm_key"]) != ("live", "recorded") or not Row["recorded"] or not Row["image"]:
                continue
            Recorded_ = {Key: Value for Key, Value in Normal(Row["recorded"]).items() if Key not in ("desk_saved", "desk_save_error")}
            Old = {Key: Value for Key, Value in Self.Old[Case["id"]].items() if Key != CD.RemeasuredKey}
            if Recorded_.get("desk_remeasured"):
                continue  # recorded against the desk it replaced, which the live model no longer has
            if Old == Recorded_:
                Matched += 1
        Self.assertGreater(Matched, 0)
        print(f"  {Matched} replays equal to the recorded camera_check key for key")

    def TestTheNoDeskCapturesAreJudgedByTheTip(Self):
        Verdicts = {}
        for Case in Self.Cases:
            Row = Self.Rows[Case["row"]]
            if (Case["model"], Case["arm_key"]) == ("live", "recorded") and Row["time"][:16] in NoDeskCaptures:
                Verdicts[Row["time"][:16]] = (Self.New[Case["id"]]["robot_check"], Self.New[Case["id"]].get("tape_error_cm"))
        Self.assertEqual(Verdicts["2026-10-06T15:01"][0], "verified")
        Self.assertEqual(Verdicts["2026-10-06T23:01"][0], "moved")
        print(f"  no-desk captures, as recorded: {Verdicts}")


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestDeskGateGolden` runs the Test* methods too (unittest names this hook)."""
    Prefixed = unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return Prefixed.loadTestsFromTestCase(DeskGateGoldenTests)


if __name__ == "__main__":
    Started = time.time()
    Loader = unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Result = unittest.TextTestRunner(verbosity=2).run(Loader.loadTestsFromTestCase(DeskGateGoldenTests))
    print(f"{time.time() - Started:.0f} s")
    raise SystemExit(not Result.wasSuccessful())
