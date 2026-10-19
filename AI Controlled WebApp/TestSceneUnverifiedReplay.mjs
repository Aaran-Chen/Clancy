import { build as Build } from 'esbuild';
import { execFileSync as ExecFile } from 'node:child_process';
import { mkdirSync as MakeDir, mkdtempSync as MakeTemp, readFileSync as ReadFile, rmSync as Remove, writeFileSync as WriteFile } from 'node:fs';
import { createRequire as CreateRequire } from 'node:module';
import { tmpdir as TempDir } from 'node:os';
import { test as Test, after as After } from 'node:test';
import Assert from 'node:assert/strict';
import Path from 'node:path';
import { fileURLToPath as FilePath } from 'node:url';
import { Fixture, PinnedHome } from './UiTestLib.mjs';

// The two /locate replies of 2026-10-14 16:35:13 and 16:35:14 UTC ("grab the white thing on the desk", right after a
// calibration; back end/TestFixtures/SceneUnverified-2026-10-14/Locate.json, from that day's grounding.jsonl). Each
// had one object under several labels as task matches ("White holder" and "white device" 0.1 cm apart), so the
// bridge registered no scene, and the app at Base read every scene-less reply with a measured support as
// scene_unverified: every object task stopped "The camera's scene could not be verified". Scenes are optional now
// (2026-10-14, "just remove the scene verification thing"): a reply without a usable scene reads as desk objects, as
// before measured scenes, and the task goes on to its GRASP PLAN.
const Root = Path.dirname(FilePath(import.meta.url));
const Repo = Path.join(Root, '..');
const Base = '6f3dae3';

async function Load(LibDir, Extra = '') {
  const Output = await Build({ stdin: { contents: `export * from './AiMeasurements';${Extra}`, resolveDir: LibDir, loader: 'ts' },
    bundle: true, platform: 'node', format: 'cjs', write: false, loader: { '.json': 'json' }, nodePaths: [Path.join(Root, 'node_modules')], plugins: [PinnedHome] });
  const Module = { exports: {} };
  new Function('require', 'module', 'exports', Output.outputFiles[0].text)(CreateRequire(import.meta.url), Module, Module.exports);
  return Module.exports;
}

// The app's lib at Base and the back end JSON it imports, laid out as in the checkout (TestCameraGateReplay.mjs).
function CheckoutAt(Commit) {
  const Git = (...Args) => ExecFile('git', ['-C', Repo, ...Args], { maxBuffer: 256 << 20 });
  const Folder = MakeTemp(Path.join(TempDir(), 'SceneUnverifiedReplay-'));
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
const New = await Load(Path.join(Root, 'src', 'lib'),
  `export { NeighbourObjects } from './GraspPlan'; export { INITIAL_SERVOS } from './robotModel'; export { SceneCommandFields } from './SceneGeometry';`);
const Records = JSON.parse(ReadFile(Fixture('SceneUnverified-2026-10-14', 'Locate.json'), 'utf8'));
// What the bridge returned to the app (the record adds its own fields beside these).
const ReplyOf = Record => { const { kind, labels, task_object, diagnostics, depth_seq, depth_file, depth_bytes, t, time, ...Reply } = Record; return Reply; };
const Query = New.ParseTaskObject('grab the white thing on the desk');

// The same reply from the fixed bridge (object_locator.py _OneTaskObject): one task match, the twins listed
// beside it, and the scene registered on the kept one's support.
function Fixed(Record) {
  const Reply = structuredClone(ReplyOf(Record));
  const Matches = Reply.objects.filter(Item => Item.task_match);
  const Kept = Matches.find(Item => Item.support_geometry && Item.refined) ?? Matches[0];
  for (const Item of Matches) if (Item !== Kept) Object.assign(Item, { task_match: false, task_twin_of: Kept.label });
  const [Low, High] = [Kept.bounds_cm.min, Kept.bounds_cm.max];
  Reply.scene = { id: 'capture-1014', revision: 'geometry-1014', frame: 'display_cm', capture_seq: Record.depth_seq ?? 1,
    support: Kept.support_geometry, obstacles: [], free_space: [{ min: [Low[0] - 10, Low[1] - 10, High[2] + 1], max: [High[0] + 10, High[1] + 10, High[2] + 15] }],
    task_object: { min: Low, max: High } };
  Object.assign(Kept, { scene_id: 'capture-1014', scene_checked: true, localization_frame: 'calibrated_depth_3d' });
  delete Reply.scene_error;
  return { Reply, Kept };
}

Test('the recorded replies stopped every object task before; now they read as desk objects and nothing stops', () => {
  Assert.equal(Records.length, 2);
  for (const Record of Records) {
    const Reply = ReplyOf(Record);
    Assert.equal(Reply.scene, undefined, Record.time);
    Assert.ok(Reply.objects.filter(Item => Item.task_match).length > 1, Record.time);
    const Before = Old.ReadLocateResponse(Reply);
    Assert.equal(Before.errorCode, 'scene_unverified', Record.time);
    Assert.deepEqual(Before.objects, []);
    Assert.match(Old.CameraGateFailure(Before, true).Title, /scene could not be verified/);

    const Now = New.ReadLocateResponse(Reply);
    Assert.equal(Now.errorCode, null, Record.time);
    Assert.equal(Now.error, null);
    Assert.equal(Now.scene, null);
    Assert.match(Now.sceneNote, /^no scene registered \(a unique task object and its supporting surface were not measured\); read as desk objects$/);
    Assert.equal(Now.objects.length, Reply.objects.length);
    Assert.ok(Now.objects.every(Object_ => !('supportGeometry' in Object_) && !('scene' in Object_)), 'no scene geometry on any object');
    Assert.equal(New.CameraGateFailure(Now, true), null);
    Assert.equal(New.MotionSceneIssue, undefined);
    const Resolved = New.ResolveTaskObject(Query, Now.objects);
    Assert.ok(Resolved.Binding, `${Record.time}: the twins at one spot bind`);
    const Support = New.TaskSupport(Resolved.Binding.Object);
    Assert.equal(Support.Refusal, null, Record.time);
    Assert.equal(Support.Floor.Scene ?? null, null);
    Assert.equal(Support.Floor.Surface ?? null, null, 'the desk floor');
  }
});

// 16:35, "grab the white thing on the desk": the white holder at X 8.1, Y 27.9 on the desk now goes on to the grasp
// planner as App.tsx BindTaskObject calls it (its floor from TaskSupport, the frame's other objects as neighbours, the
// arm at rest). The planner's verdict is its own and the same as with the scene the fixed bridge registers: the
// holder's footprint across the jaws is too wide at 16:35:13, and at 16:35:14 the gripper link would touch the basket
// at the grasp point. Neither is a scene refusal; the task asks the user rather than stopping before the model.
Test('the 16:35 attempt replays to the grasp planner for the white holder, the same verdict as with a scene', () => {
  const Plan = (Reply) => {
    const Now = New.ReadLocateResponse(Reply);
    const Holder = New.ResolveTaskObject(Query, Now.objects).Binding.Object;
    const Support = New.TaskSupport(Holder, 'white thing');
    const Context = { Servos: structuredClone(New.INITIAL_SERVOS), ElbowMode: 'up', ...(Support.Floor.Surface ? { Floor: Support.Floor } : {}) };
    return { Now, Holder, Support, Reach: New.TaskObjectReachability(Holder, Context,
      { Others: New.NeighbourObjects(Now.objects, Holder), Kind: 'white thing', Label: 'white thing' }) };
  };
  for (const [Index, Record] of Records.entries()) {
    const { Now, Holder, Support, Reach } = Plan(ReplyOf(Record));
    Assert.equal(New.CameraGateFailure(Now, true), null, Record.time);
    Assert.equal(Support.Refusal, null, Record.time);
    Assert.ok(Math.abs(Holder.x - 8.1) < 0.3 && Math.abs(Holder.y - 27.7) < 0.3, `${Holder.label} at ${Holder.x}, ${Holder.y}`);
    Assert.match(Reach.Text, /^NOT GRASPABLE \(35\.\d cm from the right arm's pan axis\)/, Record.time);
    // GRASP ANYTHING (2026-10-18): 16:35:13 read "too wide for the jaws" from the strict passes; the best-effort pass, which
    // takes any width the open jaws clear, finds the gripper or wrist link in the cart's way at every pitch and height there,
    // and says so (a real CAD contact, not a grip prediction).
    Assert.equal(Reach.Refusal.Code, ['path', 'path'][Index], Reach.Text);
    Assert.match(Reach.Text, /no grasp is physically possible from here: .*could contact the basket/, Reach.Text);
    void Index;
    Assert.doesNotMatch(Reach.Text, /scene|supporting surface|motion path/i);
    // With the scene the fixed bridge registers on the holder's support, the planner says the same.
    const Scened = Plan(Fixed(Record).Reply);
    Assert.ok(Scened.Support.Floor.Scene, Record.time);
    Assert.equal(Scened.Reach.Kind, Reach.Kind, Record.time);
    Assert.equal(Scened.Reach.Refusal?.Code, Reach.Refusal.Code, Record.time);
    // Its commands name no scene, so the bridge plans them against the basket, body and rearward limit, as before scenes.
    Assert.deepEqual(New.SceneCommandFields(Now.scene), {});
  }
});

Test('from the fixed bridge the task object binds with its scene', () => {
  for (const Record of Records) {
    const { Reply, Kept } = Fixed(Record);
    const Before = Old.ReadLocateResponse(Reply), Now = New.ReadLocateResponse(Reply);
    for (const Located of [Before, Now]) {
      Assert.equal(Located.errorCode, null, Record.time);
      Assert.equal(Located.scene.id, 'capture-1014');
      const Resolved = New.ResolveTaskObject(Query, Located.objects);
      Assert.equal(Resolved.Binding?.Object.label, Kept.label, Record.time);
      Assert.equal(Resolved.Binding.Object.scene.id, 'capture-1014');
      Assert.equal(New.TaskSupport(Resolved.Binding.Object).Refusal, null);
    }
  }
});

Test('an unverifiable or mismatched scene is dropped: the objects read as desk objects', () => {
  const [First] = Records;
  const Base_ = ReplyOf(First);
  const OneMatch = structuredClone(Base_);
  // One task match left with a support and no scene: the bridge tried and failed to register it.
  let Seen = false;
  for (const Item of OneMatch.objects) if (Item.task_match) { if (Seen) Item.task_match = false; Seen = true; }
  const { Reply: Good } = Fixed(First);
  const Bad = [
    OneMatch,
    { ...Base_, objects: Base_.objects.map(Item => Item.task_match ? { ...Item, scene_id: 'older' } : Item) },
    { ...Good, scene: { ...Good.scene, frame: 'camera_cm' } },
    { ...Good, objects: Good.objects.map(Item => Item.scene_id ? { ...Item, scene_id: 'older' } : Item) },
    { ...Good, objects: Good.objects.map(Item => Item.task_twin_of ? { ...Item, task_match: true } : Item) },
    { ...Good, objects: Good.objects.map(Item => Item.scene_id ? { ...Item, support_geometry: { ...Item.support_geometry, ObservedOnly: false } } : Item) },
    { ...Base_, scene: { id: '' } },
  ];
  for (const [Index, Reply] of Bad.entries()) {
    const Located = New.ReadLocateResponse(Reply);
    Assert.equal(Located.errorCode, null, `case ${Index}`);
    Assert.equal(Located.scene, null, `case ${Index}`);
    Assert.match(Located.sceneNote, /read as desk objects/, `case ${Index}`);
    Assert.equal(Located.objects.length, Reply.objects.length, `case ${Index}`);
    Assert.ok(Located.objects.every(Object_ => !Object_.scene && !Object_.supportGeometry), `case ${Index}`);
    Assert.equal(New.CameraGateFailure(Located, true), null, `case ${Index}`);
    for (const Object_ of Located.objects.filter(Item => Item.taskMatch))
      Assert.equal(New.TaskSupport(Object_).Refusal, null, `case ${Index}: ${Object_.label}`);
  }
});

Test('task matches at two spots are candidates: no binding until the task names one', () => {
  const Reply = structuredClone(ReplyOf(Records[0]));
  const Device = Reply.objects.find(Item => Item.label === 'white device');
  Object.assign(Device, { x: Device.x + 20 });
  const Located = New.ReadLocateResponse(Reply);
  Assert.equal(Located.errorCode, null);
  const Resolved = New.ResolveTaskObject(Query, Located.objects);
  Assert.equal(Resolved.Binding, null);
  Assert.ok(Resolved.Candidates.length >= 2);
});
