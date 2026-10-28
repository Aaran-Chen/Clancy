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

// "grab the white thing on the desk", 2026-10-18 20:24 local (03:24 UTC): the /locate reply of that task
// (back end/TestFixtures/WhiteHolder-2026-10-18/Locate.json, from that day's grounding.jsonl) with the Home the app
// held then (HomePreset.json there: Aaran's Set Home, wrist roll 1590; the arm was not connected, so the app planned
// from Home). The bridge registered a measured scene for the small white holder lying on the desk at X 2.0, Y 34.4,
// and the app refused it, "no grasp is physically possible from here". It is not impossible: as a desk object it
// plans, and the refusal came from the scene layer's cabinet rules on the desk (SceneGeometry.ts DeskSolids): the
// desk under it an obstacle the fixed finger had to keep 0.4 cm from, the flying pixels at its edge (0.5-0.7 cm over
// the desk) solids the closing jaw had to keep 0.9 cm from, and every gripper box beside it required to be observed
// free, which no box a centimetre from a surface or in the object's shadow is.
const Root = Path.dirname(FilePath(import.meta.url));
const Repo = Path.join(Root, '..');
const Base = '1d2d502';
const Home = { name: 'recorded-home', setup(Builder) {
  Builder.onLoad({ filter: /home_preset\.json$/ }, () => ({ loader: 'json', contents: ReadFile(Fixture('WhiteHolder-2026-10-18', 'HomePreset.json'), 'utf8') }));
} };

async function Load(LibDir, Extra = '') {
  const Output = await Build({ stdin: { contents: `export * from './AiMeasurements'; export * from './GraspPlan'; export { INITIAL_SERVOS } from './robotModel';${Extra}`,
    resolveDir: LibDir, loader: 'ts' }, bundle: true, platform: 'node', format: 'cjs', write: false, loader: { '.json': 'json' },
    nodePaths: [Path.join(Root, 'node_modules')], plugins: [Home] });
  const Module = { exports: {} };
  new Function('require', 'module', 'exports', Output.outputFiles[0].text)(CreateRequire(import.meta.url), Module, Module.exports);
  return Module.exports;
}
// The app's lib at Base and the back end JSON it imports, laid out as in the checkout (TestSceneUnverifiedReplay.mjs).
function CheckoutAt(Commit) {
  const Git = (...Args) => ExecFile('git', ['-C', Repo, ...Args], { maxBuffer: 256 << 20 });
  const Folder = MakeTemp(Path.join(TempDir(), 'WhiteHolderReplay-'));
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
const New = await Load(Path.join(Root, 'src', 'lib'), ` export { DeskSolids, RemovedFootprint, SceneCommandFields } from './SceneGeometry';`);
const [Record] = JSON.parse(ReadFile(Fixture('WhiteHolder-2026-10-18', 'Locate.json'), 'utf8'));
const ReplyOf = Record_ => { const { kind, labels, task_object, diagnostics, depth_seq, depth_file, depth_bytes, t, time, ...Reply } = Record_; return Reply; };
const Query = New.ParseTaskObject('grab the white thing on the desk');
// The stop reason the app recorded (ai.jsonl task_stop, 03:24:17.203Z), less its "Stopped: " and advice.
const Recorded = 'the white thing measured at X 2.0, Y 34.4 is not graspable by this gripper: no grasp is physically possible from here: every wrist pitch and grasp height tried (best effort, every grip that may slip allowed) is blocked: at wrist -50.6..-23.1 deg the arm does not reach the hover point Z 5.3 (off the desk the wrist stays one from the hover point to the lift); at wrist -43.1..-30.6 deg the fixed finger would come within 0.3 cm of the observed support; at wrist -43.1..-28.1 deg closing at the grasp point, the moving jaw would swing within 0.1 cm of the observed environment.';

// As App.tsx BindTaskObject plans it: the floor from TaskSupport (passed with a surface or a desk object's solids), the
// frame's other objects as neighbours, the arm at Home, the grasp gap of the task (1 cm).
function Plan(Api, Reply, PassFloor = Floor => !!Floor.Surface || !!Floor.Solids?.length) {
  const Now = Api.ReadLocateResponse(Reply);
  const Holder = Api.ResolveTaskObject(Query, Now.objects).Binding.Object;
  const Support = Api.TaskSupport(Holder, 'white thing');
  const Context = { Servos: structuredClone(Api.INITIAL_SERVOS), ElbowMode: 'up', ClearanceCm: 1, ...(PassFloor(Support.Floor) ? { Floor: Support.Floor } : {}) };
  const Reach = Api.TaskObjectReachability(Holder, Context, { Others: Api.NeighbourObjects(Now.objects, Holder), Kind: 'white thing', Label: 'white thing' });
  return { Now, Holder, Support, Context, Reach };
}

Test('as recorded: the scene-bound white holder was refused as physically impossible, word for word', () => {
  const { Now, Holder, Reach } = Plan(Old, ReplyOf(Record), Floor => !!Floor.Surface);
  Assert.ok(Now.scene, 'the scene was used');
  Assert.equal(Holder.label, 'white cup');
  Assert.equal(Reach.Kind, 'ungraspable');
  Assert.equal(Reach.Refusal.Code, 'path');
  Assert.equal(`the white thing measured at X 2.0, Y 34.4 is not graspable by this gripper: ${Reach.Refusal.Text}.`, Recorded);
  Assert.equal(Reach.Refusal.Advice, "Move the white thing farther from the basket and the robot's body (or closer to the arm), then send the task again.");
});

Test('now: a desk object\'s scene reads as desk objects, its solids clear of the desk kept as obstacles', () => {
  const Reply = ReplyOf(Record);
  const Now = New.ReadLocateResponse(Reply);
  Assert.equal(Now.errorCode, null);
  Assert.equal(Now.scene, null);
  Assert.match(Now.sceneNote, /^the task object lies on the desk: read as desk objects, the scene's \d+ solids clear of the desk kept as obstacles$/);
  Assert.equal(New.CameraGateFailure(Now, true), null);
  const Holder = New.ResolveTaskObject(Query, Now.objects).Binding.Object;
  Assert.equal(Holder.label, 'white cup');
  Assert.equal(Holder.scene ?? null, null);
  Assert.equal(Holder.supportGeometry ?? null, null);
  // Commands name no scene: the bridge plans them against the basket, body and rearward limit, as for every desk object.
  Assert.deepEqual(New.SceneCommandFields(Now.scene), {});
  // The solids: none of the support's own cells, none against the holder's envelope, none at the desk's level.
  const Solids = Holder.deskSolids;
  const Task = Reply.scene.task_object;
  Assert.ok(Solids.length > 100 && Solids.length < Reply.scene.obstacles.length, `${Solids.length} of ${Reply.scene.obstacles.length}`);
  Assert.ok(Solids.every(Box => Box.label !== 'observed support'));
  const Overlaps = (A, B) => [0, 1, 2].every(I => A.min[I] < B.max[I] && A.max[I] > B.min[I]);
  Assert.ok(Solids.every(Box => !Overlaps(Box, Task)));
  Assert.ok(Solids.every(Box => Box.max[2] - 0.5 > Reply.scene.support.TopZCm + 1.0));
  // The tall glass and the cart side at X -20..-10 (tops to 14.5) stay; the holder's own rim cells (X 4.5-6.5 and
  // -3.5..-1.5, Y 35.5-37.5, Z -3.5..-1.5) do not.
  Assert.ok(Solids.some(Box => Box.max[0] < -8 && Box.max[2] > 10), 'the tall solids beside the cart');
  for (const X of [4.5, -3.5]) Assert.ok(!Solids.some(Box => Box.min[0] === X && Box.min[1] === 35.5 && Box.max[2] === -1.5), `rim cell at X ${X}`);
  // Its floor is the desk's, with those solids.
  const Support = New.TaskSupport(Holder, 'white thing');
  Assert.equal(Support.Refusal, null);
  Assert.equal(Support.Floor.Surface, null);
  Assert.equal(Support.Floor.FloorZ, -3.3);
  Assert.equal(Support.Floor.Solids, Solids);
  Assert.equal(New.ScenePrisms(Support.Floor).length, Solids.length);
});

Test('now: the white holder plans, checked against those solids, and the guard holds them', () => {
  const { Holder, Context, Reach } = Plan(New, ReplyOf(Record));
  Assert.ok(Context.Floor?.Solids?.length, 'the planner gets the solids');
  Assert.equal(Reach.Kind, 'graspable', Reach.Text);
  const Plan_ = Reach.Plan;
  Assert.ok(Plan_.Pitch <= -20 && Plan_.Pitch >= -45, `wrist ${Plan_.Pitch}`);
  Assert.ok(Math.hypot(Plan_.Grasp.x - Holder.x, Plan_.Grasp.y - Holder.y) < 6, JSON.stringify(Plan_.Grasp));
  Assert.ok(Plan_.Grasp.z > Holder.z && Plan_.Grasp.z < Holder.topZ, JSON.stringify(Plan_.Grasp));
  Assert.ok(Plan_.Grasp.z >= -3.3, 'on or over the desk floor');
  // The plan's own poses keep clear of every solid (the planner's contact check, again here).
  const Prisms = New.ScenePrisms(Context.Floor);
  for (const Pose of [Plan_.Poses.Hover, Plan_.Poses.Grasp]) Assert.equal(New.DeepestContact(New.GripperSamples(Pose, Plan_.Poses.JawDegs), Prisms, 0.4), null);
  // The guard of the task holds them too (BuildGraspGuard: every prism of the desk object's floor).
  const Key = New.GraspPlanKey(Context.Servos, 'up', null);
  const State = { Label: 'white thing', Query, DetectedNow: true, Binding: { Object: Holder, Source: 'query' }, LastSeen: null, Reachability: Reach,
    Plan: { Reach, Source: Holder, At: 0, Key, Note: null }, Intent: 'grasp' };
  const Guard = New.BuildGraspGuard([Holder], State, false);
  Assert.equal(Guard.Prisms.filter(Prism => Prism.Scene).length, Prisms.length);
});

Test('a solid where the plan closed the jaws moves the plan, or refuses it: the solids are real obstacles', () => {
  const First = Plan(New, ReplyOf(Record)).Reach.Plan;
  // A tall block standing 1 cm beyond the holder's envelope, on the side the plan's fixed jaw went to.
  const Reply = structuredClone(ReplyOf(Record));
  const Task = Reply.scene.task_object;
  const Left = First.Grasp.x > Task.max[0];
  const Block = { label: 'observed environment', min: [Left ? Task.max[0] + 1 : Task.min[0] - 8, Task.min[1] - 3, -3.5], max: [Left ? Task.max[0] + 8 : Task.min[0] - 1, Task.max[1] + 3, 8] };
  Reply.scene.obstacles.push(Block);
  const { Context, Reach } = Plan(New, Reply);
  Assert.ok(Context.Floor.Solids.some(Box => Box.max[2] === 8), 'the block is a solid');
  const Inside = Point => Point.x > Block.min[0] - 1 && Point.x < Block.max[0] + 1 && Point.y > Block.min[1] - 1 && Point.y < Block.max[1] + 1;
  if (Reach.Plan) Assert.ok(!Inside(Reach.Plan.Grasp) && !Inside(Reach.Plan.Hover), JSON.stringify(Reach.Plan.Grasp));
  else Assert.match(Reach.Text, /observed environment/);
});

Test('a solid touching the object is an obstacle of its desk floor; an object off the desk keeps its scene', () => {
  // The bridge measured a solid against the holder (task_contact). Since 2026-10-22 (the pill bottle) that no longer
  // refuses the grasp before any search: the holder is a desk object and that solid, padded as the cells are, one of its
  // floor's solids. Against its +Y face it plans, clear of it, and says it was not measured apart (its Caveats).
  const Touching = structuredClone(ReplyOf(Record));
  const Task = Touching.scene.task_object;
  Touching.scene.task_contact = { min: [Task.min[0], Task.max[1] - 0.2, Task.min[2]], max: [Task.max[0], Task.max[1] + 1, Task.max[2]], points: 80 };
  const A = Plan(New, Touching);
  Assert.equal(A.Now.scene, null);
  const Contact = A.Holder.deskSolids.find(Box => Box.label === 'observed environment against it');
  // (With the footprint the bridge removed as the holder, and as a solid it stands against: review round 3.)
  Assert.deepEqual(Contact, { label: 'observed environment against it', min: Touching.scene.task_contact.min.map(V => V - 0.5), max: Touching.scene.task_contact.max.map(V => V + 0.5),
    measured: { min: Touching.scene.task_contact.min, max: Touching.scene.task_contact.max }, removed: New.RemovedFootprint(Touching.scene), against: true });
  const Apart = /the camera did not measure it apart from the observed environment against it \(at X -?\d+\.\d, Y \d+\.\d, Z -?\d+\.\d to -?\d+\.\d\)/;
  // A plan, not a refusal: the solid against it is planned around (a refusal here is the one GRASP ANYTHING removed).
  Assert.equal(A.Reach.Kind, 'graspable', A.Reach.Text);
  const Prism = New.ScenePrisms(A.Context.Floor).find(P => P.Label === 'observed environment against it');
  for (const Pose of [A.Reach.Plan.Poses.Hover, A.Reach.Plan.Poses.Grasp]) Assert.equal(New.DeepestContact(New.GripperSamples(Pose, A.Reach.Plan.Poses.JawDegs), [Prism], 0.4), null);
  Assert.ok(A.Reach.Plan.Caveats.some(Why => Apart.test(Why) && Why.endsWith('planned clear of that, but the white thing may be larger than measured there')), JSON.stringify(A.Reach.Plan.Caveats));
  Assert.ok(!(A.Reach.Plan.Warnings ?? []).some(Why => Apart.test(Why)), 'not a best-effort warning');
  Assert.match(A.Reach.Text, Apart);
  // Against its +X face, over its whole height, it cannot be planned at the wrist roll the arm holds: its one grasp there
  // (the fixed finger beside that face, at X 7.0) would put the fixed finger in the solid. The refusal says what blocks
  // it, and that it was not measured apart.
  const Wall = structuredClone(ReplyOf(Record));
  Wall.scene.task_contact = { min: [Task.max[0] - 0.2, Task.min[1], Task.min[2]], max: [Task.max[0] + 1, Task.max[1], Task.max[2]], points: 80 };
  const Held = (Make => { const Saved = New.GraspRules.FreeRoll; New.GraspRules.FreeRoll = false; try { return Make(); } finally { New.GraspRules.FreeRoll = Saved; } });
  const W = Held(() => Plan(New, Wall));
  Assert.equal(W.Reach.Kind, 'ungraspable');
  Assert.match(W.Reach.Refusal.Text, /the fixed finger would come within -?\d\.\d cm of the observed environment against it/);
  Assert.match(W.Reach.Refusal.Text, new RegExp(`; and ${Apart.source}$`));
  Assert.match(W.Reach.Refusal.Advice, / Or move the white thing 2 to 3 cm away from what stands against it\.$/);
  // With the wrist roll free (GraspRules.FreeRoll, milestone 2) the jaws turned close across it from its -Y side, the
  // solid against its +X face planned around as any (and the part of it the measurement took in, TaskContactPrisms),
  // its lift checked and stepped (DeskLift), and it says it was not measured apart.
  const R = Plan(New, Wall), Rolled = R.Reach.Plan;
  Assert.equal(R.Reach.Kind, 'graspable', R.Reach.Text);
  Assert.ok(Rolled.RollRaw != null && Rolled.RollFromRaw === 1590, `${Rolled.RollRaw} ${Rolled.RollFromRaw}`);
  const Solids = [...New.ScenePrisms(R.Context.Floor), ...New.TaskContactPrisms(R.Context.Floor, Rolled.Prism)];
  Assert.ok(Solids.some(P => P.Label === 'observed environment against it'));
  for (const Pose of [Rolled.Poses.Hover, Rolled.Poses.Grasp]) Assert.equal(New.DeepestContact(New.GripperSamples(Pose, Rolled.Poses.JawDegs), Solids, 0.4), null);
  Assert.ok(Rolled.Caveats.some(Why => Apart.test(Why)), JSON.stringify(Rolled.Caveats));
  Assert.ok(Rolled.LiftStepCm == null || Rolled.LiftStepCm < 6, `${Rolled.LiftStepCm}`);
  // On a measured surface (a cabinet top, not the desk): the scene as before, its support the floor.
  const Raised = structuredClone(ReplyOf(Record));
  for (const Item of Raised.objects) if (Item.task_match) Item.support_branch = 'surface';
  const Now = New.ReadLocateResponse(Raised);
  Assert.ok(Now.scene);
  // (Found by its task match: "on the desk" does not bind an object on another surface.)
  const Holder = Now.objects.find(Object_ => Object_.taskMatch);
  Assert.equal(Holder.deskSolids, undefined);
  Assert.ok(New.TaskSupport(Holder, 'white thing').Floor.Scene);
});

Test('a neighbour standing against the object stays a solid where its cells reach into the padded envelope', () => {
  // The bridge merges cells into long boxes: a neighbour against the holder's +Y face is one box reaching 0.3 cm into the
  // envelope, up to 10 cm over the holder. The desk route dropped every box overlapping the envelope as the object's rim,
  // so with only the contact's box kept, the planner and the guard held nothing where the neighbour stands.
  const Reply = structuredClone(ReplyOf(Record));
  const Task = Reply.scene.task_object;
  const Neighbour = { label: 'observed environment', min: [Task.min[0], Task.max[1] - 0.3, Task.min[2]], max: [Task.max[0], Task.max[1] + 4, Task.max[2] + 10] };
  Reply.scene.obstacles.push(Neighbour);
  Reply.scene.task_contact = { min: [Task.min[0] + 0.5, Task.max[1] - 0.2, Task.min[2] + 1], max: [Task.max[0] - 0.5, Task.max[1] + 0.3, Task.max[2] - 0.5], points: 60 };
  const { Holder, Support, Context, Reach } = Plan(New, Reply);
  Assert.equal(Support.Floor.Solids, Holder.deskSolids);
  // Its cells (the box less its 0.5 cm padding), every 0.25 cm: each inside a prism the planner and the guard hold.
  const Prisms = New.ScenePrisms(Support.Floor);
  const In = (Prism, P) => P.z >= Prism.BaseZ && P.z <= Prism.TopZ && P.x >= Math.min(...Prism.Footprint.map(C => C.x)) && P.x <= Math.max(...Prism.Footprint.map(C => C.x))
    && P.y >= Math.min(...Prism.Footprint.map(C => C.y)) && P.y <= Math.max(...Prism.Footprint.map(C => C.y));
  let Checked = 0;
  for (let X = Neighbour.min[0] + 0.625; X < Neighbour.max[0] - 0.5; X += 0.25)
    for (let Y = Neighbour.min[1] + 0.625; Y < Neighbour.max[1] - 0.5; Y += 0.25)
      for (let Z = Neighbour.min[2] + 0.625; Z < Neighbour.max[2] - 0.5; Z += 0.25) {
        Checked++;
        Assert.ok(Prisms.some(Prism => In(Prism, { x: X, y: Y, z: Z })), `X ${X}, Y ${Y}, Z ${Z}`);
      }
  Assert.ok(Checked > 1000, `${Checked}`);
  // The guard holds them (BuildGraspGuard: every prism of the desk object's floor); a plan keeps clear of them.
  const Key = New.GraspPlanKey(Context.Servos, 'up', null);
  const State = { Label: 'white thing', Query, DetectedNow: true, Binding: { Object: Holder, Source: 'query' }, LastSeen: null, Reachability: Reach,
    Plan: { Reach, Source: Holder, At: 0, Key, Note: null }, Intent: 'grasp' };
  // (And the part of the solid against it the holder's measurement took in, beside the plan's prism: TaskContactPrisms, review round 3.)
  Assert.equal(New.BuildGraspGuard([Holder], State, false).Prisms.filter(Prism => Prism.Scene).length,
    Prisms.length + New.TaskContactPrisms(Support.Floor, Reach.Plan.Prism).length);
  Assert.equal(Reach.Kind, 'graspable', Reach.Text);
  for (const Pose of [Reach.Plan.Poses.Hover, Reach.Plan.Poses.Grasp]) Assert.equal(New.DeepestContact(New.GripperSamples(Pose, Reach.Plan.Poses.JawDegs), Prisms, 0.4), null);
  Assert.ok(Reach.Plan.Caveats?.length, 'told it was not measured apart');
});

Test('an impossible refusal names each blocker apart, not every one under the first one\'s words', () => {
  const Fail = (Pitch, Why) => ({ Ok: false, Code: 'path', Stage: 5, Pitch, Why });
  const Refusal = New.ImpossibleRefusal({ Label: 'white thing' }, [
    Fail(-50.6, 'at wrist -50.6 deg the arm does not reach the hover point Z 5.3 (off the desk the wrist stays one from the hover point to the lift)'),
    Fail(-23.1, "the gripper's path enters space the depth camera has not observed"),
    Fail(-38.1, "the gripper's path enters space the depth camera has not observed")], { Code: 'path', Text: '', Advice: '' });
  Assert.match(Refusal.Text, /at wrist -38\.1\.\.-23\.1 deg the gripper's path enters space the depth camera has not observed/);
  Assert.match(Refusal.Text, /at wrist -50\.6 deg the arm does not reach the hover point Z 5\.3/);
});
