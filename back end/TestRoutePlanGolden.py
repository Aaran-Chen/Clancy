"""Faster route planning, the same routes: every recorded route planned by ce62ab9's planner and by this one.

On 2026-10-25 two Home presses planned for 5.7 s and 5.9 s on the live bridge before the arm moved (route_timing.jsonl
22:14:58Z and 02:05:35Z, out of the basket by the bottle). The checks behind every route are as strict as before; what
changed is how fast they are run (BasketCollision's module notes: IntervalBlocked, RegionFailure, SceneCandidates,
PathScout; BasketSurface's answer memo; RouteCheckPool's WorldRef). Here that is held to:

- the same routes: ce62ab9's planner (`git archive ce62ab9 "back end"`, in a process of its own) and this checkout's
  plan every recorded request with the clock stopped, so no deadline decides and both search to the end: Home to where
  each recorded route ended (every route_timing.jsonl record, the jaws as Home moves them), and every recorded tip move
  of the app (ai.jsonl approvals, its own gripper goal). Waypoints, tight, arrive and reach, or the refusal and its
  words, must be equal case for case. This checkout plans them with a worker pool, so the workers' answers are tested
  too;
- the same checks: on poses, intervals and edges along those routes, with and without the measured cabinet scene
  (TestFixtures/CabinetScene-2026-10-06: 1292 obstacles, 562 observed boxes, TestAutomaticScene's 2026-10-06 15:01 capture), with the start pose's relief and the
  waypoint reaches the bridge gives, the old EdgeReason and IntervalReason give the very reasons this one does, and
  IntervalBlocked and EdgeBlocked agree with them;
- the time: the two slow Home presses, on the bridge's pool of 8 workers and the real clock, plan in under 1.5 s each
  (they took 5.7 and 5.9 s live, 2.7 and 5.0 s offline before), to the very route the old planner finds with no clock.

The recordings stay where they are (RecordedCaptures: in a git worktree, the main checkout's); without them this fails,
RECORDING MISSING, and without git (the old commit's code) it fails too.

Run:  python TestRoutePlanGolden.py
"""
import hashlib
import io
import json
import os
import subprocess
import sys
import tarfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as Np

import BasketCollision as Collision
import bridge_server as Bridge
import RecordedCaptures as RC
import RouteCheckPool
from test_bridge_home import FakeBus

Root = Path(__file__).resolve().parent
Captures = RC.Root()
OldCommit = "ce62ab9"
Work = Root / "TestArtifacts" / "RoutePlanGolden"
SceneFile = RC.Fixture("CabinetScene-2026-10-06", "Scene.json")
# The two slow Home presses of 2026-10-25 (route_timing.jsonl): the plan seconds they took live.
Slow = {"2026-10-25T22:14:58.874Z": 5.672, "2026-10-26T02:05:35.224Z": 5.86}
SlowBoundSeconds = 1.5

# The planning and checking both checkouts run: this text runs in the old one's process (argv: back end, spec, out)
# and is imported here for this one, so both plan each case the same way.
Runner = r'''
import json, os, sys, time
Backend = sys.argv[1] if __name__ == "__main__" else None
if Backend:
    sys.path.insert(0, Backend)
    os.chdir(Backend)
from unittest.mock import patch
import numpy as Np
import BasketCollision as Collision
import bridge_server as Bridge
from test_bridge_home import FakeBus


def MakeController(Start, Percent):
    Controller = Bridge.BridgeController()
    Bus = FakeBus()
    Controller.bus, Controller.port, Controller.torque_enabled = Bus, "FAKE", True
    Bus.torque_enabled = True
    for Name, Servo in Controller.servos.items():
        Servo.present_raw = Servo.target_raw = Servo.HardwareRaw = Bus.position[Name] = Start.get(Name, Servo.present_raw)
    Bus.configure_motors()
    Controller.refresh()
    Bus.writes.clear()
    Controller.SetMotionSpeed(Percent)
    return Controller


def Snapshot(Controller, Scene):
    Planning = Controller.PlanningSnapshot()
    if Scene is not None:
        Planning.ActiveScene = json.loads(json.dumps(Scene))
        Planning.SceneCarried = None
    return Planning


def Plan(Case, Scenes):
    """One recorded request, as the bridge plans it (go_home's PlanHomeRoute, send_ik's PlanCartesianMotion)."""
    Controller = MakeController(Case["start"], Case["speed"])
    Planning = Snapshot(Controller, Scenes.get(Case.get("scene")))
    try:
        if Case["kind"] == "home":
            Targets = {Name: Controller._clamp_raw(Controller.servos[Name], Case["goal"][Name]) for Name in Bridge.HOME_MOTOR_NAMES}
            Path = Planning.PlanHomeRoute(Targets, Controller.HomeGripperPlan(Targets)[0])
        else:
            Target = Case["target"]
            Requested = Controller.ModelPointFromVisualPoint({"x": -Target["x"], "y": Target["y"], "z": Target["z"]})
            Reason = Planning.CollisionWorld().TargetReason(Requested)
            if Reason:
                raise ValueError(Reason)
            Gripper = None if Case.get("gripper") is None else Controller.GripperWithTipTarget(Case["gripper"])
            Path = (Planning if Gripper is None else Planning.GripperAtSnapshot(Gripper)).PlanCartesianMotion(
                Requested, Case["pitch"], InitialAngles=None, Elbow=Planning.elbow_mode, AssignReach=Gripper is None)
            if Gripper is not None:
                Path = Planning.BlendGripperIntoRoute(Path, Gripper)
    except ValueError as Error:
        return {"ok": False, "reason": str(Error)}
    return {"ok": True, "path": [dict(Point) for Point in Path], "tight": sorted(Path.Tight), "arrive": sorted(Path.Arrive),
            "reach": {str(Index): Reach for Index, Reach in sorted(Path.Reach.items())}}


def Checks(Query, Scenes, Words=True):
    """The reasons EdgeReason and IntervalReason give for one query (and, Words False, only whether there is one)."""
    Controller = MakeController(Query["start"], 100)
    Planning = Snapshot(Controller, Scenes.get(Query.get("scene")))
    World = Planning.CollisionWorld()
    Answers = {}
    try:
        Relief = Planning.StartRelief() if Query.get("relief") else None
    except ValueError as Error:
        Answers["relief"], Relief = str(Error), None
    for Index, (Goal, Reach, Budget) in enumerate(Query["edges"]):
        Reach = (Reach[0], Reach[1] if isinstance(Reach[1], int) else dict(Reach[1]))
        Arguments = Planning.EdgeCheckArguments(Query["start"], Goal, Relief, Reach, Budget)
        Answers[f"edge {Index}"] = World.EdgeReason(*Arguments[:2], Budget=Arguments[2], Relief=Arguments[3], Reach=Arguments[4])
        if not Words:
            Answers[f"edge {Index} blocked"] = World.EdgeBlocked(*Arguments[:2], Budget=Arguments[2], Relief=Arguments[3], Reach=Arguments[4])
    Pose = Np.array([Controller._ik_angle_from_raw(Controller.servos[Name], Query["start"][Name]) for Name in Bridge.MOTOR_NAMES])
    for Index, (Spread, Tracking) in enumerate(Query["intervals"]):
        Low, High = Pose - Np.array(Spread), Pose + Np.array(Spread)
        Answers[f"interval {Index}"] = World.IntervalReason(Low, High, Tracking)
        if Relief is not None:
            Answers[f"interval {Index} relieved"] = World.IntervalReason(Low, High, False, Relief["RearwardSlackCm"], Relief["InsetCm"])
        if not Words:
            Answers[f"interval {Index} blocked"] = World.IntervalBlocked(Low, High, Tracking)
    return Answers


def Run(Spec, Frozen=True):
    """Every case of Spec planned and every query checked, with the clock stopped (Frozen): no deadline decides."""
    Scenes = {Name: json.loads(open(File, encoding="utf-8").read()) for Name, File in Spec["scenes"].items()}
    Out = {"plans": {}, "checks": {}}
    with patch.object(Bridge, "ROUTE_REACH_ASSIGN_SECONDS", 3600.0):
        Now = time.monotonic()
        Clock = patch.object(time, "monotonic", lambda: Now) if Frozen else None
        if Clock:
            Clock.start()
        try:
            for Case in Spec["cases"]:
                Out["plans"][Case["name"]] = Plan(Case, Scenes)
            for Query in Spec["queries"]:
                Out["checks"][Query["name"]] = Checks(Query, Scenes)
        finally:
            if Clock:
                Clock.stop()
    return Plain(Out)


def Plain(Value):
    """Value as JSON reads it back (NumPy numbers as numbers), so both checkouts' results compare alike."""
    return json.loads(json.dumps(Value, default=lambda Item: Item.item() if hasattr(Item, "item") else str(Item)))


if __name__ == "__main__":
    Guards = (patch.object(Bridge.depth_camera, "Opener", lambda: (_ for _ in ()).throw(RuntimeError("offline: no camera"))),
              patch.object(Bridge, "RaiseProcessPriority", lambda: "normal"))
    for Guard in Guards:
        Guard.start()
    Spec = json.loads(open(sys.argv[2], encoding="utf-8").read())
    open(sys.argv[3], "w", encoding="utf-8").write(json.dumps(Run(Spec)))
'''


def Git(*Arguments) -> bytes | None:
    """git in this checkout's root (pathspecs from there), or None when it fails."""
    try:
        Done = subprocess.run(["git", "-C", str(Root.parent), *Arguments], capture_output=True, timeout=120)
    except (OSError, subprocess.TimeoutExpired):
        return None
    return Done.stdout if Done.returncode == 0 else None


def OldBackend() -> Path | None:
    """ce62ab9's back end, extracted once under TestArtifacts."""
    Target = Work / OldCommit
    if (Target / "back end" / "BasketCollision.py").is_file():
        return Target / "back end"
    Archive = Git("archive", "--format=tar", OldCommit, "back end")
    if not Archive:
        return None
    Target.mkdir(parents=True, exist_ok=True)
    with tarfile.open(fileobj=io.BytesIO(Archive)) as Tar:
        Tar.extractall(Target, filter="data")
    return Target / "back end"


def RecordedCases() -> list[dict]:
    """Every recorded route (Home to where it ended) and every recorded tip move, oldest first."""
    Cases = []
    for Day in sorted(Path(Captures).glob("20??-??-??")):
        Timing, Ai = Day / "recording" / "route_timing.jsonl", Day / "recording" / "ai.jsonl"
        Routes = [json.loads(Line) for Line in Timing.read_text(encoding="utf-8").splitlines() if Line.strip()] if Timing.is_file() else []
        for Record in Routes:
            Cases.append({"name": f"home {Record['time']}", "kind": "home", "start": Record["start_raw"],
                          "goal": Record["waypoints_raw"][-1], "speed": Record.get("speed_percent", 100)})
        if not Routes or not Ai.is_file():
            continue
        # Each tip leg the app sent (chain_leg "together" carries its gripper goal, "tip" none), from where the route it
        # started began (the route that began planning within 3 s of it), else where the last route before it ended.
        Begins = [(Record["t"] - Record["run_s"] - Record["plan_s"], Record) for Record in Routes]
        Approved = None
        for Line in Ai.read_text(encoding="utf-8").splitlines():
            try:
                Event = json.loads(Line)
            except ValueError:
                continue
            if Event.get("type") == "approve":
                Approved = Event
            if Event.get("type") != "chain_leg" or Approved is None or Event.get("event_kind") not in ("together", "tip") \
                    or not Event.get("target_cm"):
                continue
            Near = [(abs(Begin - Event["t"]), Record) for Begin, Record in Begins if -1.0 <= Begin - Event["t"] <= 3.0]
            Before = [Record for Begin, Record in Begins if Begin < Event["t"] - 1.0]
            if Near:
                Live = min(Near, key=lambda Item: Item[0])[1]
                Start = Live["start_raw"]
            elif Before:
                Live, Start = None, Before[-1]["waypoints_raw"][-1]
            else:
                continue
            Cases.append({"name": f"tip {Event['time']}", "kind": "tip", "start": Start, "target": Event["target_cm"],
                          "pitch": Approved["motion"]["wrist_pitch_degrees"],
                          "gripper": Event.get("gripper_raw") if Event["event_kind"] == "together" else None,
                          "speed": (Live or {}).get("speed_percent", 100)})
    # Every recorded request planned; requests the planner refuses, from where the 2026-10-25 22:14:58 Home began
    # (low beside the bottle): out of reach, behind the rearward limit, into the cart, a wrist it cannot hold there.
    Start = next((Case["start"] for Case in Cases if Case["name"] == "home 2026-10-25T22:14:58.874Z"), None)
    if Start is not None:
        for Name, (X, Y, Z, Pitch) in {"out of reach": (0.0, 80.0, 10.0, -15.0), "behind the limit": (0.0, -40.0, 10.0, 0.0),
                                       "into the cart": (-13.3, 2.0, -6.0, -90.0), "wrist": (3.4, 30.1, 2.4, 80.0)}.items():
            Cases.append({"name": f"refused {Name}", "kind": "tip", "start": Start, "target": {"x": X, "y": Y, "z": Z},
                          "pitch": Pitch, "gripper": None, "speed": 100})
    return Cases


def CheckQueries(Cases) -> list[dict]:
    """Edges and intervals around the recorded starts: a few hundred, half with the cabinet scene, some relieved."""
    Random = Np.random.default_rng(20261025)
    Queries = []
    Names = Bridge.MOTOR_NAMES
    for Index, Case in enumerate(Cases[::4]):
        Start = Case["start"]
        Edges = []
        for _ in range(3):
            Goal = {Name: int(Start[Name] + Random.integers(-90, 91)) % 4096 for Name in Names[:4]}
            Goal.update({Name: Start[Name] for Name in Names[4:]})
            Reach = [{Name: int(Random.choice((0, 16, 32))) for Name in Names}, int(Random.choice((0, 16, 32, 64)))]
            Edges.append((Goal, Reach, int(Random.choice((2, 8, 64)))))
        Intervals = [([0.0] * 6, True), ([float(Random.uniform(0, 3)) for _ in Names], False),
                     ([float(Random.uniform(0, 8)) for _ in Names], True)]
        Scene = "cabinet" if Index % 2 else None
        if Scene:
            Edges = [(Goal, Reach, min(Budget, 2)) for Goal, Reach, Budget in Edges[:2]]  # The old scene check: ~0.1 s a pose.
        Queries.append({"name": f"{Case['name']} {'scene' if Scene else 'plain'}", "start": Start, "edges": Edges,
                        "intervals": Intervals, "scene": Scene, "relief": Index % 3 == 0})
    return Queries


def Namespace():
    Space = {"__name__": "RoutePlanRunner"}
    exec(compile(Runner, "RoutePlanRunner", "exec"), Space)
    return Space


def StartedPool(Count):
    Pool = RouteCheckPool.Pool(Count)
    Pool._Start()
    return Pool


class RoutePlanGoldenTests(unittest.TestCase):
    @classmethod
    def setUpClass(Class):
        Timings = sorted(Path(Captures).glob("20??-??-??/recording/route_timing.jsonl"))
        RC.Need(Path(Captures) / "2026-10-25" / "recording" / "route_timing.jsonl", What="the recorded routes of 2026-10-25")
        RC.Need(SceneFile, What="the cabinet scene fixture")
        Class.Backend = OldBackend()
        if Class.Backend is None:
            raise AssertionError(f"git archive {OldCommit} failed: this replay needs git and that commit of this repo")
        Class.Cases = RecordedCases()
        Class.Spec = {"cases": Class.Cases, "queries": CheckQueries(Class.Cases), "scenes": {"cabinet": str(SceneFile)}}
        Text = json.dumps(Class.Spec, sort_keys=True)
        Key = hashlib.sha256((Text + Runner + SceneFile.read_text(encoding="utf-8")).encode("utf-8")).hexdigest()[:16]
        Work.mkdir(parents=True, exist_ok=True)
        Golden = Work / f"Old_{OldCommit}_{Key}.json"
        if not Golden.is_file():
            (Work / f"Spec_{Key}.json").write_text(Text, encoding="utf-8")
            (Work / "OldRunner.py").write_text(Runner, encoding="utf-8")
            Started = time.time()
            Done = subprocess.run([sys.executable, str(Work / "OldRunner.py"), str(Class.Backend), str(Work / f"Spec_{Key}.json"), str(Golden)],
                                  capture_output=True, text=True, cwd=str(Work), timeout=7200,
                                  env=dict(os.environ, PYTHONDONTWRITEBYTECODE="1", SO101_DEPTH_CAMERA="0"))
            if Done.returncode != 0:
                raise AssertionError(f"{OldCommit}'s planner failed:\n{Done.stderr[-4000:]}")
            print(f"  {OldCommit} planned {len(Class.Cases)} cases from {len(Timings)} days in {time.time() - Started:.0f} s")
        Class.Old = json.loads(Golden.read_text(encoding="utf-8"))
        Class.Runner = Namespace()
        Class.Scenes = {"cabinet": json.loads(SceneFile.read_text(encoding="utf-8"))}
        Class.Guards = (patch.object(Bridge.depth_camera, "Opener", lambda: (_ for _ in ()).throw(RuntimeError("offline: no camera"))),
                        patch.object(Bridge, "RaiseProcessPriority", lambda: "normal"))
        for Guard in Class.Guards:
            Guard.start()
        # This checkout plans with check workers (SO101_TEST_ROUTE_WORKERS caps them in RunAllTests.py), so the
        # candidate paths go through them (PathScout) as on the bridge.
        Cap = int(os.environ.get("SO101_TEST_ROUTE_WORKERS") or 0)
        Class.Pool = StartedPool(min(4, Cap) if Cap > 0 else 4)
        Started = time.time()
        with patch.object(RouteCheckPool, "_Pool", Class.Pool):
            Class.New = Class.Runner["Run"](Class.Spec)
        print(f"  this checkout planned {len(Class.Cases)} cases in {time.time() - Started:.0f} s "
              f"({len(Class.Pool.Workers)} workers)")

    @classmethod
    def tearDownClass(Class):
        Class.Pool.Close()
        for Guard in Class.Guards:
            Guard.stop()

    def TestEveryRecordedRequestPlansTheSameRoute(Self):
        Planned = sum(Result["ok"] for Result in Self.Old["plans"].values())
        Self.assertGreater(len(Self.Cases), 150)
        Self.assertGreater(Planned, 100)
        Self.assertTrue(any(not Result["ok"] for Result in Self.Old["plans"].values()), "some recorded requests are refused")
        for Case in Self.Cases:
            with Self.subTest(case=Case["name"]):
                Self.assertEqual(Self.New["plans"][Case["name"]], Self.Old["plans"][Case["name"]])
        print(f"  {len(Self.Cases)} recorded requests: {Planned} routes and {len(Self.Cases) - Planned} refusals, as {OldCommit}")

    def TestTheChecksGiveTheOldReasons(Self):
        Count = 0
        for Query in Self.Spec["queries"]:
            Old = Self.Old["checks"][Query["name"]]
            with patch.object(RouteCheckPool, "_Pool", None):
                New = Self.Runner["Plain"](Self.Runner["Checks"](Query, Self.Scenes, Words=False))
            for Name, Reason in Old.items():
                Count += 1
                with Self.subTest(query=Query["name"], check=Name):
                    Self.assertEqual(New[Name], Reason)
                    if f"{Name} blocked" in New:
                        Self.assertEqual(New[f"{Name} blocked"], Reason is not None)
        Self.assertTrue(any(Reason for Answers in Self.Old["checks"].values() for Reason in Answers.values()))
        Self.assertTrue(any(Reason is None for Answers in Self.Old["checks"].values() for Reason in Answers.values()))
        Self.assertTrue(any("observed" in str(Reason) or "environment" in str(Reason)
                            for Answers in Self.Old["checks"].values() for Reason in Answers.values()), "scene reasons are among them")
        print(f"  {Count} checks give {OldCommit}'s reasons")

    def TestTheSlowHomePressesPlanInUnderASecondAndAHalf(Self):
        """On the bridge's pool (RouteCheckPool.MaximumWorkers, whatever SO101_TEST_ROUTE_WORKERS says: this is the time the
        bridge takes) and the real clock, to the route the old planner finds with no clock at all. Each time as the
        request's first: no answers remembered from planning it before (BasketSurface's memo), here or in a worker."""
        Cases = [Case for Case in Self.Cases if Case["kind"] == "home" and Case["name"][5:] in Slow]
        Self.assertEqual(len(Cases), 2)
        for Case in Cases:
            Best = None
            for _ in range(2):  # The faster of two: a plan another program on the PC interrupted is not its cost.
                Pool = StartedPool(RouteCheckPool.MaximumWorkers)
                Self.addCleanup(Pool.Close)
                for Surface in [*Collision.Surfaces.values(), *Collision.BodySurfaces.values()]:
                    Surface.Memo = {}
                with patch.object(RouteCheckPool, "_Pool", Pool):
                    Began = time.perf_counter()
                    Result = Self.Runner["Plain"](Self.Runner["Plan"](Case, {}))
                    Seconds = time.perf_counter() - Began
                Pool.Close()
                Best = Seconds if Best is None else min(Best, Seconds)
                Self.assertEqual(Result, Self.Old["plans"][Case["name"]], Case["name"])
            print(f"  {Case['name']}: {Best:.2f} s (live {Slow[Case['name'][5:]]} s)")
            Self.assertLess(Best, SlowBoundSeconds, Case["name"])


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestRoutePlanGolden` runs the Test* methods too (unittest names this hook)."""
    Prefixed = unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return Prefixed.loadTestsFromTestCase(RoutePlanGoldenTests)


if __name__ == "__main__":
    Loader = unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Result = unittest.TextTestRunner(verbosity=2).run(Loader.loadTestsFromTestCase(RoutePlanGoldenTests))
    raise SystemExit(not Result.wasSuccessful())
