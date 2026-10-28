import { build as Build } from 'esbuild';
import { execFileSync as ExecFile } from 'node:child_process';
import { mkdirSync as MakeDir, mkdtempSync as MakeTemp, readFileSync as ReadFile, rmSync as Remove, writeFileSync as WriteFile } from 'node:fs';
import { createRequire as CreateRequire } from 'node:module';
import { tmpdir as TempDir } from 'node:os';
import { test as Test, after as After } from 'node:test';
import Assert from 'node:assert/strict';
import Path from 'node:path';
import { fileURLToPath as FilePath } from 'node:url';
import { Fixture } from './UiTestLib.mjs';

// "grab the white pill bottle", 2026-10-22 14:53:45 UTC: the /locate reply of 14:54:03 (back end/TestFixtures/
// PillBottle-2026-10-22/Locate.json, from that day's grounding.jsonl) with the Home the app held (HomePreset.json there:
// Aaran's Set Home, the wrist roll at the 1586 it read). The app stopped "the white pill bottle measured at X 10.3,
// Y 38.4 is out of the right arm's reach (45.1 cm from its shoulder)" in 387 ms, before the grasp planner ran: it planned
// only where a pitch reached the bottle's centre (or 3 cm above or below it, or beside it), and skipped even those scans
// with the centre beyond the straightened arm. Now the planner's own search runs (GRASP ANYTHING, 2026-10-22), and it
// agrees: the bottle is out of reach there. At its top half the finger tips reach 43.1-43.8 cm from the pan axis at
// most, the bottle's near face is at about 43.3 and its centre at 45.1: the jaws cannot get around any of it. Moved about
// 4 cm closer (where Aaran put it at 14:55:37) it plans.
const Root = Path.dirname(FilePath(import.meta.url));
const Repo = Path.join(Root, '..');
const Base = '8eefe77';
const Home = { name: 'recorded-home', setup(Builder) {
  Builder.onLoad({ filter: /home_preset\.json$/ }, () => ({ loader: 'json', contents: ReadFile(Fixture('PillBottle-2026-10-22', 'HomePreset.json'), 'utf8') }));
} };
async function Load(LibDir) {
  const Output = await Build({ stdin: { contents: `export * from './AiMeasurements'; export * from './GraspPlan'; export { INITIAL_SERVOS, modelPointFromDisplayPoint, mirrorDisplayPoint, solveIkPreview, rawFromAngle } from './robotModel';`,
    resolveDir: LibDir, loader: 'ts' }, bundle: true, platform: 'node', format: 'cjs', write: false, loader: { '.json': 'json' },
    nodePaths: [Path.join(Root, 'node_modules')], plugins: [Home] });
  const Module = { exports: {} };
  new Function('require', 'module', 'exports', Output.outputFiles[0].text)(CreateRequire(import.meta.url), Module, Module.exports);
  return Module.exports;
}
// The app's lib at Base and the back end JSON it imports, laid out as in the checkout (TestSceneUnverifiedReplay.mjs).
function CheckoutAt(Commit) {
  const Git = (...Args) => ExecFile('git', ['-C', Repo, ...Args], { maxBuffer: 256 << 20 });
  const Folder = MakeTemp(Path.join(TempDir(), 'PillBottleReplay-'));
  const Files = [...Git('ls-tree', '-r', '--name-only', Commit, 'AI Controlled WebApp/src/lib').toString().trim().split('\n'),
    ...Git('ls-tree', '--name-only', Commit, 'back end/').toString().trim().split('\n').filter(Name => Name.endsWith('.json'))];
  for (const File of Files) {
    const Target = Path.join(Folder, File);
    MakeDir(Path.dirname(Target), { recursive: true });
    WriteFile(Target, Git('show', `${Commit}:${File}`));
  }
  return Folder;
}
const Folder = CheckoutAt(Base);
After(() => Remove(Folder, { recursive: true, force: true }));
const Old = await Load(Path.join(Folder, 'AI Controlled WebApp', 'src', 'lib'));
const New = await Load(Path.join(Root, 'src', 'lib'));
const [Record] = JSON.parse(ReadFile(Fixture('PillBottle-2026-10-22', 'Locate.json'), 'utf8'));
const ReplyOf = Record_ => { const { kind, labels, task_object, diagnostics, depth_seq, depth_file, depth_bytes, t, time, ...Reply } = Record_; return Reply; };
const Query = New.ParseTaskObject('grab the white pill bottle');
// As App.tsx BindTaskObject plans it (its floor passed with a surface, or a desk object's solids).
function Plan(Api, Reply, PassFloor = Floor => !!Floor.Surface || !!Floor.Solids?.length) {
  const Now = Api.ReadLocateResponse(Reply);
  const Bottle = Api.ResolveTaskObject(Query, Now.objects).Binding.Object;
  const Support = Api.TaskSupport(Bottle, 'white pill bottle');
  const Context = { Servos: structuredClone(Api.INITIAL_SERVOS), ElbowMode: 'up', ClearanceCm: 1, ...(PassFloor(Support.Floor) ? { Floor: Support.Floor } : {}) };
  const Started = Date.now();
  const Reach = Api.TaskObjectReachability(Bottle, Context, { Others: Api.NeighbourObjects(Now.objects, Bottle), Kind: 'white pill bottle', Label: 'white pill bottle' });
  return { Now, Bottle, Support, Context, Reach, Ms: Date.now() - Started };
}

Test('as recorded: out of reach before the planner ran', () => {
  const { Bottle, Reach } = Plan(Old, ReplyOf(Record), Floor => !!Floor.Surface);
  Assert.equal(Bottle.label, 'white pill bottle');
  Assert.equal(Reach.Kind, 'out-of-reach');
  Assert.equal(Reach.BaseDistanceCm.toFixed(1), '45.1');
  Assert.equal(Reach.Plan ?? null, null);
  Assert.match(Reach.Text, /^OUT OF REACH \(45\.1 cm from the right arm's pan axis; the closest pose misses by 2\.1 cm\)/);
  Assert.doesNotMatch(Reach.Text, /grasp search/);
});

Test('now: the planner\'s own search runs, and it is out of reach there, by the arm\'s length', () => {
  const { Bottle, Support, Context, Reach } = Plan(New, ReplyOf(Record));
  // Its scene was registered on the desk: a desk object, the keyboard the bridge measured against it a solid.
  Assert.ok(Support.Floor.Solids.some(Box => Box.label === 'observed environment against it'));
  Assert.equal(New.NearestBeyondArm(Bottle), false, 'its near face is within the straightened arm');
  Assert.equal(Reach.Kind, 'out-of-reach', Reach.Text);
  // (What the search tried, not that no grasp is possible: its grasp points go through the bottle's middle or beside it.)
  Assert.match(Reach.Text, /; the grasp search, its grasp point through the middle of it or beside it, found none either: every wrist pitch and grasp height tried .*the arm does not reach a grasp point beside it at wrist -?\d/);
  Assert.doesNotMatch(Reach.Text, /physically possible/);
  // (3 cm since review round 3: 4 before, the bottle's own cap, read as a "white keyboard" inside its top, a neighbour.)
  Assert.equal(Reach.CloserCm, 3);
  Assert.match(Reach.Advice, /^Move the white pill bottle about 3 cm closer to the right arm/);
  // The arm's real reach (its IK with the joint limits, every wrist pitch): over the bottle's height the finger tips
  // (the tool point: the fingers reach 0.6 cm past it) stop short of its centre, at about its near face.
  const Centre = New.modelPointFromDisplayPoint({ x: Bottle.x, y: Bottle.y, z: 0 }), R = Math.hypot(Centre.x, Centre.y);
  const Unit = { x: Centre.x / R, y: Centre.y / R };
  const Reaches = (Radial, Z) => {
    const At = New.mirrorDisplayPoint({ x: Unit.x * Radial, y: Unit.y * Radial, z: 0 });
    const Point = New.modelPointFromDisplayPoint({ x: At.x, y: At.y, z: Z });
    for (let Pitch = -90; Pitch <= 30; Pitch += 1) if (New.solveIkPreview({ ...Point, wristPitch: Pitch }, 'up', Context.Servos).reachable) return true;
    return false;
  };
  for (const Z of [Bottle.z + 1, (Bottle.z + Bottle.topZ) / 2, Bottle.topZ]) {
    Assert.ok(Reaches(R - 3.5, Z), `a point 3.5 cm nearer than its centre at Z ${Z}`);
    Assert.ok(!Reaches(R - 1, Z), `1 cm short of its centre at Z ${Z}`);
  }
});

Test('moved about 4 cm closer, as Aaran then put it, it plans', () => {
  const Reply = structuredClone(ReplyOf(Record));
  // (The scene belongs to where it stood: left out.)
  delete Reply.scene;
  for (const Item of Reply.objects) {
    delete Item.scene_id; delete Item.support_geometry; delete Item.scene_checked;
    if (Item.label === 'white pill bottle' || Item.label === 'white keyboard') { Item.x -= 1; Item.y -= 4; }
  }
  const { Reach } = Plan(New, Reply);
  Assert.equal(Reach.Kind, 'graspable', Reach.Text);
  Assert.ok(Reach.BaseDistanceCm < 42, `${Reach.BaseDistanceCm}`);
});

Test('a grasp is not refused before the search because the object is past the reach scan, only past the arm itself', () => {
  const Reply = structuredClone(ReplyOf(Record));
  const Far = { ...Reply.objects.find(Item => Item.task_match), x: 10.25, y: 60 };
  const Now = New.ReadLocateResponse({ ...Reply, objects: [Far] });
  const Bottle = Now.objects[0];
  Assert.equal(New.NearestBeyondArm(Bottle), true, '20 cm out, beyond the straightened arm');
  const Started = Date.now();
  const Reach = New.TaskObjectReachability(Bottle, { Servos: structuredClone(New.INITIAL_SERVOS), ElbowMode: 'up' }, { Others: [], Label: 'white pill bottle' });
  Assert.equal(Reach.Kind, 'out-of-reach');
  Assert.doesNotMatch(Reach.Text, /grasp search/);
  Assert.ok(Date.now() - Started < 20000, 'no search for what no arm pose reaches');
});

// The scene's cells, the merged boxes the bridge sends (back end/SceneGeometry.py _MergeCells), less their 0.5 cm padding,
// sampled every 0.25 cm; and whether a point lies in a prism as the planner and the guard hold it (ScenePrisms).
const Pad = 0.5;
function Samples(Box, Step = 0.25) {
  const Points = [];
  for (let X = Box.min[0] + Pad + Step / 2; X < Box.max[0] - Pad; X += Step)
    for (let Y = Box.min[1] + Pad + Step / 2; Y < Box.max[1] - Pad; Y += Step)
      for (let Z = Box.min[2] + Pad + Step / 2; Z < Box.max[2] - Pad; Z += Step) Points.push({ x: X, y: Y, z: Z });
  return Points;
}
const InPrism = (Prism, P) => P.z >= Prism.BaseZ && P.z <= Prism.TopZ
  && P.x >= Math.min(...Prism.Footprint.map(C => C.x)) && P.x <= Math.max(...Prism.Footprint.map(C => C.x))
  && P.y >= Math.min(...Prism.Footprint.map(C => C.y)) && P.y <= Math.max(...Prism.Footprint.map(C => C.y));

Test('the keyboard against it stays a solid: every cell against the bottle\'s envelope, outside what the bridge removed as the bottle, is one the planner and the guard hold', () => {
  // The bridge measured the keyboard against the bottle (task_contact, 124 points). The desk route used to drop every scene
  // box overlapping the bottle's padded envelope as its rim: 20 boxes over the desk band, running up to 19 cm past it
  // (X -11.5..10.5 at Y 45.5-49.5, X 13.5..27.5 at Y 41.5-46.5), held by no solid but the contact's own box.
  const Reply = ReplyOf(Record);
  const { Support } = Plan(New, Reply);
  const Scene = Reply.scene, Task = Scene.task_object, Level = Scene.support.TopZCm;
  Assert.ok(Scene.task_contact);
  // The footprint the bridge removed as the bottle (Build: bounds_cm grown by TargetRimCm 0.3; the envelope is it padded by 0.5).
  const Bounds = Reply.objects.find(Item => Item.task_match).bounds_cm;
  const Removed = { min: [Bounds.min[0] - 0.3, Bounds.min[1] - 0.3], max: [Bounds.max[0] + 0.3, Bounds.max[1] + 0.3] };
  Assert.ok([0, 1].every(I => Math.abs(Removed.min[I] - (Task.min[I] + 0.2)) < 1e-9 && Math.abs(Removed.max[I] - (Task.max[I] - 0.2)) < 1e-9));
  const InRemoved = P => P.x >= Removed.min[0] && P.x <= Removed.max[0] && P.y >= Removed.min[1] && P.y <= Removed.max[1] && P.z >= Task.min[2] && P.z <= Task.max[2];
  const Overlaps = (A, B) => [0, 1, 2].every(I => A.min[I] < B.max[I] && A.max[I] > B.min[I]);
  const Against = Scene.obstacles.filter(Box => Box.label !== 'observed support' && Overlaps(Box, Task) && Box.max[2] - Pad > Level + 1);
  Assert.equal(Against.length, 20);
  const Prisms = New.ScenePrisms(Support.Floor);
  let Checked = 0;
  const Missed = [];
  for (const Box of Against) for (const P of Samples(Box)) {
    if (InRemoved(P)) continue;
    Checked++;
    if (!Prisms.some(Prism => InPrism(Prism, P))) Missed.push(P);
  }
  Assert.ok(Checked > 5000, `${Checked}`);
  Assert.deepEqual(Missed.slice(0, 3), [], `${Missed.length} of ${Checked} keyboard samples held by no solid`);
  for (const P of [{ x: 20, y: 44, z: -2 }, { x: 0, y: 47.5, z: -2.5 }, { x: 7.2, y: 39.5, z: -2.5 }]) Assert.ok(Prisms.some(Prism => InPrism(Prism, P)), JSON.stringify(P));
  // Every kept part lies clear of that footprint (the bottle's own space), or is the padded contact.
  const Clear = Box => Box.max[0] <= Removed.min[0] + 1e-9 || Box.min[0] >= Removed.max[0] - 1e-9 || Box.max[1] <= Removed.min[1] + 1e-9
    || Box.min[1] >= Removed.max[1] - 1e-9 || Box.min[2] >= Task.max[2] - 1e-9 || Box.max[2] <= Task.min[2] + 1e-9;
  Assert.ok(Support.Floor.Solids.every(Box => Box.label === 'observed environment against it' || Clear(Box)));
  // Without the contact (task_contact null), the boxes against the envelope are its rim, dropped as before.
  const Apart = structuredClone(Reply);
  Apart.scene.task_contact = null;
  const Rimless = Plan(New, Apart).Support.Floor.Solids;
  Assert.ok(Rimless.every(Box => !Overlaps(Box, Task)));
  Assert.ok(Rimless.length < Support.Floor.Solids.length);
});

// The bottle, the keyboard and the scene moved together (by DX, DY): the bottle within the arm's reach, the keyboard against it.
function Shifted(DX, DY) {
  const Reply = structuredClone(ReplyOf(Record));
  const Move = Box => { if (!Box) return; Box.min[0] += DX; Box.max[0] += DX; Box.min[1] += DY; Box.max[1] += DY; };
  for (const Item of Reply.objects) if (Item.label === 'white pill bottle' || Item.label === 'white keyboard') { Item.x += DX; Item.y += DY; Move(Item.bounds_cm); }
  for (const Box of [...Reply.scene.obstacles, ...(Reply.scene.free_space ?? []), ...(Reply.scene.known_space ?? []), Reply.scene.task_object, Reply.scene.task_contact]) Move(Box);
  return Reply;
}

Test('moved closer with the keyboard still against it: it plans, clear of every solid, and is told it was not measured apart', () => {
  const { Support, Context, Reach } = Plan(New, Shifted(-1, -4));
  Assert.equal(Support.Floor.Scene ?? null, null, 'a desk object');
  const Apart = /^the camera did not measure it apart from the observed environment against it \(at X -?\d+\.\d, Y \d+\.\d, Z -?\d+\.\d to -?\d+\.\d\)/;
  Assert.ok(New.FloorConflict(Support.Floor));
  // A plan, not a refusal: the solid against it is planned around (a refusal here is the one GRASP ANYTHING removed).
  Assert.equal(Reach.Kind, 'graspable', Reach.Text);
  Assert.equal(Reach.Plan.Caveats.length, 1);
  Assert.match(Reach.Plan.Caveats[0], new RegExp(`${Apart.source}: planned clear of that, but the white pill bottle may be larger than measured there$`));
  // Not a warning: those mark a best-effort plan, which the guard lets touch the bottle on its descent.
  Assert.ok(!(Reach.Plan.Warnings ?? []).some(Why => Apart.test(Why)));
  Assert.match(Reach.Text, /^graspable[^:]* \(the camera did not measure it apart from the observed environment against it /);
  Assert.match(New.BestEffortNote(Reach.Plan), /The camera did not measure it apart from the observed environment against it/);
  const Prisms = New.ScenePrisms(Context.Floor);
  for (const Pose of [Reach.Plan.Poses.Hover, Reach.Plan.Poses.Grasp]) Assert.equal(New.DeepestContact(New.GripperSamples(Pose, Reach.Plan.Poses.JawDegs), Prisms, 0.4), null);
});

Test('where it really cannot be planned (out of reach as recorded), the refusal names the solid it was not measured apart from', () => {
  const { Support, Reach } = Plan(New, ReplyOf(Record));
  Assert.ok(New.FloorConflict(Support.Floor));
  Assert.equal(Reach.Kind, 'out-of-reach');
  Assert.equal(Reach.Plan ?? null, null);
  Assert.match(Reach.Text, /; and the camera did not measure it apart from the observed environment against it \(at X 12\.0, Y 44\.9, Z -2\.2 to 0\.5\)\)/);
});

// Held, on the desk route (2026-10-22 review): 2 cm nearer and 6 cm more, the plan grasps at wrist -34.2 at (12.0, 31.2, -0.8),
// the keyboard the camera measured against the bottle at Y 36-41.8. A lift or carry toward it at up to 2 cm up (the bottle's
// base 1-3 cm under the keyboard's top) drives the held bottle into the keyboard though every gripper part stays clear of it:
// the guard checked the held bottle only against a scene's solids (its floor's Scene), and a desk object's floor has none.
let Near_ = null;
const Near = () => Near_ ??= Plan(New, Shifted(-2, -6));
function HeldGuard(Current) {
  const { Bottle, Context, Reach } = Near();
  const State = { Label: 'white pill bottle', Query, DetectedNow: true, Binding: { Object: Current ?? Bottle, Source: 'query' }, LastSeen: null, Reachability: Reach,
    Plan: { Reach, Source: Bottle, At: 0, Key: New.GraspPlanKey(Context.Servos, 'up', null), Note: null }, Intent: 'grasp' };
  return New.BuildGraspGuard([Current ?? Bottle], State, true);
}
// From the grasp point, jaws closed on it: Along cm toward the solid against it (its measured box's centre), Up cm up, at Wrist.
function HeldMove(Guard, Along, Up, Wrist) {
  const { Bottle, Context, Support, Reach } = Near();
  const Measured = Support.Floor.Solids.find(Box => Box.label === 'observed environment against it').measured;
  const DX = (Measured.min[0] + Measured.max[0]) / 2 - Bottle.x, DY = (Measured.min[1] + Measured.max[1]) / 2 - Bottle.y, L = Math.hypot(DX, DY);
  const Closed = { ...Reach.Plan.Poses.Grasp, gripper: Reach.Plan.Poses.JawDegs[0] };
  const Model = { ...New.modelPointFromDisplayPoint({ x: Reach.Plan.Grasp.x + DX / L * Along, y: Reach.Plan.Grasp.y + DY / L * Along, z: Reach.Plan.Grasp.z + Up }), wristPitch: Wrist };
  const Preview = New.solveIkPreview(Model, 'up', Context.Servos);
  return Preview.reachable ? New.GraspGuardReason(Model, { ...Preview.angles, gripper: Closed.gripper }, Closed, Up, Guard) ?? 'ALLOWED' : null;
}

Test('held on the desk route: a carry toward the keyboard against it is refused for the held bottle, a lift straight out of it runs', () => {
  const { Reach } = Near();
  Assert.equal(Reach.Kind, 'graspable', Reach.Text);
  const Guard = HeldGuard();
  Assert.ok(Guard.Floor?.Solids?.some(Box => Box.label === 'observed environment against it'), 'the guard holds the desk object\'s floor');
  let Refused = 0;
  for (const Along of [1.5, 2, 3]) for (const Up of [0.5, 1, 2]) for (const Wrist of [Reach.Plan.Pitch, -35, -15, -5]) {
    const Reason = HeldMove(Guard, Along, Up, Wrist);
    if (Reason === null) continue;
    Assert.match(Reason, /^That route is blocked: the held white pill bottle would (touch|be pushed further into) observed environment/, `${Along} cm toward it, ${Up} up, wrist ${Wrist}`);
    Refused++;
  }
  Assert.ok(Refused >= 8, `${Refused}`);
  // Straight up out of it, at the grasp wrist, runs, up to the plan's lift step (round 3: a longer one joint-linear move
  // swings the held bottle toward the keyboard's part its measurement took in, TaskContactPrisms; the plan lifts in steps).
  const Step = New.PlanLiftStepCm(Reach.Plan);
  for (const Up of [0.5, 1, Step]) Assert.equal(HeldMove(Guard, 0, Up, Reach.Plan.Pitch), 'ALLOWED', `${Up} cm up`);
  Assert.match(HeldMove(Guard, 0, 5, Reach.Plan.Pitch), /^That route is blocked: the held white pill bottle would be pushed further into observed environment against it \(inside its measured box\)/);
});

Test('a kept plan keeps the desk solids it was made clear of on a check-in whose frame registered no scene', () => {
  // The gripper over the bottle: the bridge registered no scene ("the supporting surface is not visible"). The plan is kept
  // (no scene revision on the desk route to drop it by), and its guard held only the measured objects: not the keyboard.
  const Reply = Shifted(-2, -6);
  delete Reply.scene;
  Reply.scene_error = 'the supporting surface is not visible';
  const Current = New.ResolveTaskObject(Query, New.ReadLocateResponse(Reply).objects).Binding.Object;
  Assert.equal(Current.deskSolids, undefined);
  const Guard = HeldGuard(Current);
  Assert.ok(Guard.Prisms.some(Prism => Prism.Label === 'observed environment against it'), 'the solid against it stays an obstacle');
  // (With the keyboard's part its measurement took in, beside the plan's prism: TaskContactPrisms, round 3.)
  Assert.equal(Guard.Prisms.filter(Prism => Prism.Scene).length,
    New.ScenePrisms(Near().Support.Floor).length + New.TaskContactPrisms(Near().Support.Floor, Near().Reach.Plan.Prism).length);
  Assert.match(HeldMove(Guard, 2, 1, -15) ?? '', /^That route is blocked: the held white pill bottle would/);
  Assert.equal(HeldMove(Guard, 0, New.PlanLiftStepCm(Near().Reach.Plan), Near().Reach.Plan.Pitch), 'ALLOWED');
  // Not held yet, the guard holds the keyboard too.
  const { Bottle, Context, Reach } = Near();
  const Open = New.BuildGraspGuard([Current], { Label: 'white pill bottle', Query, DetectedNow: true, Binding: { Object: Current, Source: 'query' }, LastSeen: null,
    Reachability: Reach, Plan: { Reach, Source: Bottle, At: 0, Key: New.GraspPlanKey(Context.Servos, 'up', null), Note: null }, Intent: 'grasp' }, false);
  Assert.ok(Open.Prisms.some(Prism => Prism.Label === 'observed environment against it'));
});

// --- Review round 3 (2026-10-22) -------------------------------------------------------------------------------------

// Every gripper sample of a plan's hover and grasp poses (each jaw angle it was checked at).
const PlanSamples = Plan_ => [Plan_.Poses.Hover, Plan_.Poses.Grasp].flatMap(Pose => Plan_.Poses.JawDegs.flatMap(Jaw => New.GripperSamples({ ...Pose, gripper: Jaw }, [Jaw])));
const InBox = (Box, P) => ['x', 'y', 'z'].every((Axis, I) => P[Axis] >= Box.min[I] && P[Axis] <= Box.max[I]);

Test('round 3, 1: a neighbour the measurement took into the bottle stays a solid: no gripper sample of the plan inside it', () => {
  // A box 4 cm tall against the bottle's +X side, 3 cm of it taken into the bottle's measurement (its bounds_cm to X 14.6),
  // as 6 cm of the keyboard were behind it. The bridge removes that part with the bottle, and its task_contact is only
  // what lies within TouchCm outside the removed footprint: the taken part was in no solid, and the fixed finger went
  // into it (7af0ced: 183 gripper samples inside the real box).
  const Reply = Shifted(-2, -6);
  const Item = Reply.objects.find(Object_ => Object_.task_match);
  const Real = { min: [11.6, 28.5, -3.19], max: [16.1, 35.5, 0.8] };
  Item.bounds_cm.max[0] = Real.min[0] + 3;
  Reply.scene.task_object.max[0] = Item.bounds_cm.max[0] + 0.5;
  const Edge = Item.bounds_cm.max[0] + 0.3;
  Reply.scene.obstacles.push({ label: 'observed environment', min: [Edge - 0.5, Real.min[1] - 0.5, Real.min[2] + 0.5], max: [Real.max[0] + 0.5, Real.max[1] + 0.5, Real.max[2] + 0.5] });
  Reply.scene.task_contact = { points: 200, min: [Edge, Real.min[1], -2.19], max: [Edge + 0.5, Real.max[1], Real.max[2]] };
  const { Now, Support, Reach } = Plan(New, Reply);
  Assert.equal(Reach.Kind, 'graspable', Reach.Text);
  const Inside = PlanSamples(Reach.Plan).filter(Sample => InBox(Real, Sample.P));
  Assert.deepEqual(Inside.slice(0, 3), [], `${Inside.length} gripper samples inside the box the bottle's measurement took in`);
  // The taken part is a solid now, beside the bottle's prism toward the contact (TaskContactPrisms).
  const Inner = New.TaskContactPrisms(Support.Floor, Reach.Plan.Prism);
  Assert.ok(Inner.length >= 1 && Inner.every(Prism => Prism.Against && Prism.Scene));
  Assert.ok(Inner.some(Prism => InPrism(Prism, { x: 12.5, y: 31, z: -1 })), 'the taken part of the box is held');
  // And the guard holds it too.
  const Bottle = New.ResolveTaskObject(Query, Now.objects).Binding.Object;
  const Guard = New.BuildGraspGuard(Now.objects, { Label: 'white pill bottle', Query, DetectedNow: true, Binding: { Object: Bottle, Source: 'query' }, LastSeen: null,
    Reachability: Reach, Plan: { Reach, Source: Bottle, At: 0, Key: New.GraspPlanKey(structuredClone(New.INITIAL_SERVOS), 'up', null), Note: null }, Intent: 'grasp' }, false);
  Assert.ok(Guard.Prisms.some(Prism => Prism.Label === 'observed environment against it (inside its measured box)'));
});

Test('round 3, 2: planned again on a frame with no scene, a desk object keeps the solids its last plan was made clear of', () => {
  // The jaws closed on nothing, or the object moved: the next frame registered no scene (the gripper over it), and
  // App.tsx BindTaskObject planned it again from the desk's floor alone, the keyboard against it in no solid of the
  // planner or of the new plan's guard (7af0ced: 0 floor solids, 0 scene prisms in the guard).
  const First = Near();
  Assert.ok(First.Bottle.deskSolids?.length);
  const Reply = Shifted(-2, -6);
  delete Reply.scene;
  Reply.scene_error = 'the supporting surface is not visible in this capture';
  for (const Item of Reply.objects) {
    delete Item.scene_id;
    // (Pushed 0.5 cm toward the fixed finger, as a close on nothing pushes it.)
    if (Item.task_match) { Item.x += 0.5; Item.bounds_cm.min[0] += 0.5; Item.bounds_cm.max[0] += 0.5; }
  }
  const Now = New.ReadLocateResponse(Reply);
  const Bottle = New.ResolveTaskObject(Query, Now.objects).Binding.Object;
  Assert.equal(Bottle.deskSolids, undefined);
  const Floor = New.KeptDeskFloor(New.TaskSupport(Bottle, 'white pill bottle').Floor, Bottle, First.Bottle);
  Assert.equal(Floor.Solids, First.Bottle.deskSolids, "the last plan's solids, the keyboard against it among them");
  const Context = { ...First.Context, Floor };
  const Reach = New.TaskObjectReachability(Bottle, Context, { Others: New.NeighbourObjects(Now.objects, Bottle), Kind: 'white pill bottle', Label: 'white pill bottle' });
  Assert.equal(Reach.Kind, 'graspable', Reach.Text);
  const Solids = [...New.ScenePrisms(Floor), ...New.TaskContactPrisms(Floor, Reach.Plan.Prism)];
  for (const Pose of [Reach.Plan.Poses.Hover, Reach.Plan.Poses.Grasp]) Assert.equal(New.DeepestContact(New.GripperSamples(Pose, Reach.Plan.Poses.JawDegs), Solids, 0.4), null);
  // The new plan's Source carries them, so its guard keeps the keyboard on the next frames.
  const Source = New.PlanSource(Bottle, Floor);
  Assert.equal(Source.deskSolids, Floor.Solids);
  const State = { Label: 'white pill bottle', Query, DetectedNow: true, Binding: { Object: Bottle, Source: 'query' }, LastSeen: null, Reachability: Reach,
    Plan: { Reach, Source, At: 0, Key: New.GraspPlanKey(Context.Servos, 'up', null), Note: null, AfterMiss: true }, Intent: 'grasp' };
  for (const Held of [false, true]) Assert.ok(New.BuildGraspGuard(Now.objects, State, Held).Prisms.some(Prism => Prism.Label === 'observed environment against it'));
  // A frame with its own desk scene plans with that alone, and its plan's Source is the object itself.
  Assert.equal(New.KeptDeskFloor(First.Support.Floor, First.Bottle, Source), First.Support.Floor);
  Assert.equal(New.PlanSource(First.Bottle, First.Support.Floor), First.Bottle);
});

Test('round 3, 3: the bottle\'s own cap, read as a "white keyboard" inside its top, is the bottle: 8 cm nearer it plans', () => {
  // 7af0ced refused it after 99 s, "closing at the grasp point, the moving jaw would swing within 0.2 cm of the white
  // keyboard": the cap (base Z 4.0 inside the bottle's Z -3.2 to 4.7) planned as a neighbour.
  const { Now, Bottle, Reach } = Plan(New, Shifted(-1, -8));
  const Cap = Now.objects.find(Object_ => Object_.label === 'white keyboard');
  Assert.ok(Cap.z > Bottle.z + 5 && Cap.z < Bottle.topZ, JSON.stringify([Cap.z, Bottle.z, Bottle.topZ]));
  Assert.ok(New.IsTwin(Cap, Bottle, null, New.GraspPrism(Bottle)));
  Assert.ok(!New.NeighbourObjects(Now.objects, Bottle).includes(Cap));
  Assert.equal(Reach.Kind, 'graspable', Reach.Text);
  // Something resting on it (its base at its top), or wider than it, stays a neighbour.
  Assert.ok(!New.IsTwin({ ...Cap, z: Bottle.topZ - 0.2 }, Bottle, null, New.GraspPrism(Bottle)));
  Assert.ok(!New.IsTwin({ ...Cap, sizeCm: [7, 7] }, Bottle, null, New.GraspPrism(Bottle)));
});

// The plan's lift as the app runs it: the close-and-lift's straight lift, then a lift a step at a time (NEXT STEP), each
// solved from where the last ended (as the app solves a move from the live arm) at the wrist it holds.
const IkJoints = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex'];
const ServosAt = (Servos, Angles) => Object.fromEntries(Object.entries(Servos).map(([Name, Servo]) => [Name, IkJoints.includes(Name)
  ? { ...Servo, presentRaw: New.rawFromAngle(Servo, Angles[Name]), ...(Servo.guiAngleRaw != null ? { guiAngleRaw: New.rawFromAngle(Servo, Angles[Name]) } : {}) } : Servo]));
function LiftSteps(Guard, Step, Total = 6) {
  const { Context, Reach } = Near(), Plan_ = Reach.Plan;
  let From = { ...Plan_.Poses.Grasp, gripper: Plan_.Poses.JawDegs[0] };
  const Results = [];
  for (let Up = Step; Up <= Total + 1e-9; Up += Step) {
    const Model = { ...New.modelPointFromDisplayPoint({ ...Plan_.Grasp, z: Plan_.Grasp.z + Up }), wristPitch: Plan_.Pitch };
    const Preview = New.solveIkPreview(Model, 'up', ServosAt(Context.Servos, From));
    const To = { ...Preview.angles, gripper: From.gripper };
    Results.push(Preview.reachable ? New.GraspGuardReason(Model, To, From, Step, Guard) ?? 'ALLOWED' : 'unreachable');
    From = To;
  }
  return Results;
}

Test("round 3, 4: the plan's own close-and-lift runs past the guard: in steps the joint-linear way keeps clear of the keyboard", () => {
  // 7af0ced: the plan (wrist -34.2, grasp X 12.0, Y 31.2, Z -0.8) was shipped with one 6 cm lift (GraspRules.LiftCm),
  // and the guard refused it once the jaws held the bottle ("the held white pill bottle would touch observed environment
  // against it at 20% of the way there": the joint-linear lift swung it 0.9 cm toward the keyboard): the task stalled.
  const { Reach } = Near(), Plan_ = Reach.Plan;
  Assert.equal(Reach.Kind, 'graspable', Reach.Text);
  const Guard = HeldGuard();
  // (GraspPlan LiftStepCm, else one lift of GraspRules.LiftCm: AiMeasurements PlanLiftStepCm.)
  const Step = Plan_.LiftStepCm ?? 6;
  Assert.ok(Step >= 1.5 && Step <= 6, `${Step}`);
  Assert.deepEqual(LiftSteps(Guard, Step), Array.from({ length: Math.round(6 / Step) }, () => 'ALLOWED'), `the plan's lift, ${Step} cm at a time`);
  // In one move, the lift is still refused (the steps are why), for the keyboard: nothing was loosened.
  if (Step < 6) Assert.match(LiftSteps(Guard, 6)[0], /^That route is blocked: the held white pill bottle would (touch|be pushed further into) observed environment against it/);
  // The NEXT STEP gives it so: the close chained with the first step, then a step at a time until the 6 cm.
  Assert.equal(New.PlanLiftStepCm(Plan_), Step);
  const Close = New.NextPlannedGraspStep(Plan_, Plan_.Grasp, { GripperFraction: Plan_.OpenFraction, Closed: false }, 'white pill bottle');
  Assert.ok(Close.includes(`"motion_proposal":${New.CloseAndLiftJson(Step)}`), Close);
  if (Step < 6) {
    const Next = New.NextPlannedGraspStep(Plan_, { ...Plan_.Grasp, z: Plan_.Grasp.z + Step }, { GripperFraction: 0.5, Closed: true }, 'white pill bottle');
    Assert.ok(Next.includes(`"motion_proposal":{"move_delta_cm":{"left":0,"farther":0,"up":${Step}}}`), Next);
    Assert.match(New.NextPlannedGraspStep(Plan_, { ...Plan_.Grasp, z: Plan_.Grasp.z + 6 }, { GripperFraction: 0.5, Closed: true }, 'white pill bottle'), /it is lifted/);
  }
});

Test('round 3, 5: held, a measured neighbour against it is held on a rise too: a lift runs, a rise with a carry into it does not', () => {
  // A box 1.5 cm tall 0.2 cm off the bottle's -Y side, a measured object only. 7af0ced skipped a measured object the held
  // bottle touched for any step rising more than 0.5 cm: 1.5 to 3 cm toward it and 0.6 to 2 cm up ran, into it.
  const Reply = Shifted(-2, -6);
  const Cup = Reply.objects.find(Object_ => Object_.label === 'white cup');
  Reply.objects.push({ ...structuredClone(Cup), label: 'blue box', x: 8.25, y: 28.5, z: -3.19, top_z: -1.7, size_cm: [4, 2.1],
    bounds_cm: { min: [6.25, 27.45, -3.19], max: [10.25, 29.55, -1.7] }, task_match: false });
  const P = Plan(New, Reply);
  Assert.equal(P.Reach.Kind, 'graspable', P.Reach.Text);
  const Plan_ = P.Reach.Plan;
  const State = { Label: 'white pill bottle', Query, DetectedNow: true, Binding: { Object: P.Bottle, Source: 'query' }, LastSeen: null, Reachability: P.Reach,
    Plan: { Reach: P.Reach, Source: P.Bottle, At: 0, Key: New.GraspPlanKey(P.Context.Servos, 'up', null), Note: null }, Intent: 'grasp' };
  const Guard = New.BuildGraspGuard(P.Now.objects, State, true);
  Assert.ok(Guard.Prisms.some(Prism => Prism.Label === 'blue box'));
  const Closed = { ...Plan_.Poses.Grasp, gripper: Plan_.Poses.JawDegs[0] };
  const Move = (Dy, Up, Wrist = Plan_.Pitch) => {
    const Model = { ...New.modelPointFromDisplayPoint({ x: Plan_.Grasp.x, y: Plan_.Grasp.y + Dy, z: Plan_.Grasp.z + Up }), wristPitch: Wrist };
    const Preview = New.solveIkPreview(Model, 'up', ServosAt(P.Context.Servos, Closed));
    return Preview.reachable ? New.GraspGuardReason(Model, { ...Preview.angles, gripper: Closed.gripper }, Closed, Up, Guard) ?? 'ALLOWED' : null;
  };
  let Checked = 0;
  for (const [Dy, Up] of [[-1.5, 0.6], [-2, 0.6], [-3, 2], [-2, 1], [-3, 0.6], [-3, 0.3], [-4, 0.8]]) {
    const Reason = Move(Dy, Up);
    if (Reason === null) continue;
    Checked++;
    Assert.match(Reason, /^That route is blocked: the held white pill bottle would (touch|be pushed further into) blue box/, `${-Dy} cm toward it, ${Up} up`);
  }
  Assert.ok(Checked >= 4, `${Checked}`);
  Assert.equal(Move(0, New.PlanLiftStepCm(Plan_)), 'ALLOWED', 'straight up out of it, by the plan\'s lift step');
});
