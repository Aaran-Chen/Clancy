// "grab the pill bottle" at the bookshelf cabinet of 2026-10-06 15:01:01Z, on the bridge's /locate replies that
// back end/TestAutomaticScene.py (CabinetSceneTests) writes from the recorded frame: as placed, and with the bottle
// slid 3 cm toward the arm, clear of the books (a counterfactual render), with the task's reach (SceneGeometry
// AssumeTaskReachFree, H2) off and on. This suite has them written afresh, into a folder of its own, by this
// checkout's back end before it runs: they used to be read from whatever TestAutomaticScene.py had last left, and
// without them the tests skipped. Without the recorded frame that fails, RECORDING MISSING. No bridge, no camera.
import { build as Build } from 'esbuild';
import { execFileSync as Execute } from 'node:child_process';
import { existsSync as Exists, readFileSync as ReadFile, rmSync as Remove } from 'node:fs';
import { createRequire as CreateRequire } from 'node:module';
import { test as Test } from 'node:test';
import Assert from 'node:assert/strict';
import Path from 'node:path';
import { fileURLToPath as FilePath } from 'node:url';
import { BackendPython } from './UiTestLib.mjs';

const Root = Path.dirname(FilePath(import.meta.url));
const Backend = Path.join(Root, '..', 'back end');
const Scenes = Path.join(Backend, 'TestArtifacts', 'CabinetScenesForGrasp');
const Reply = Name => JSON.parse(ReadFile(Path.join(Scenes, `${Name}.json`), 'utf8'));
// The replies, written now (about 30 s); its failure (a RECORDING MISSING above all) is this suite's.
Remove(Scenes, { recursive: true, force: true });
const Python = BackendPython();
let Written = null;
try {
  Execute(Python, ['-m', 'unittest', 'TestAutomaticScene.CabinetSceneTests.test_moved_clear_of_the_books_nothing_stands_against_the_bottle'],
    { cwd: Backend, encoding: 'utf8', stdio: 'pipe', timeout: 600_000,
      env: { ...process.env, SO101_CABINET_SCENES: Scenes, SO101_DEPTH_CAMERA: '0', SO101_RECORDER: '0', PYTHONDONTWRITEBYTECODE: '1' } });
} catch (Failure) {
  Written = `the cabinet replies could not be written (back end/TestAutomaticScene.py CabinetSceneTests):\n${String(Failure.stderr || Failure.message).slice(-3000)}`;
}
const Missing = Written ?? (['recorded', 'moved_3cm', 'moved_3cm_reach'].find(Name => !Exists(Path.join(Scenes, `${Name}.json`))) ? 'no cabinet replies written' : null);
const Fixture = { name: 'no-declared-surfaces', setup(Builder) {
  Builder.onLoad({ filter: /DeclaredSurfaces\.json$/ }, () => ({ loader: 'json', contents: JSON.stringify({ Surfaces: [] }) }));
} };
const Bundle = await Build({ stdin: { contents: ReadFile(Path.join(Root, 'src/App.tsx'), 'utf8')
  + '\nexport * from "./lib/AiMeasurements";\nexport * from "./lib/GraspPlan";\nexport { GripperLowestZ } from "./lib/AiTableFloor";'
  + '\nexport { INITIAL_SERVOS, currentAngles, forwardKinematics, mirrorDisplayPoint, solveIkPreview, modelPointFromDisplayPoint } from "./lib/robotModel";',
  resolveDir: Path.join(Root, 'src'), loader: 'tsx' }, bundle: true, platform: 'node', format: 'cjs', write: false, loader: { '.css': 'empty' },
  plugins: [Fixture], logLevel: 'error' });
globalThis.window ??= { location: { hostname: 'localhost' } };
const AppModule = { exports: {} };
new Function('require', 'module', 'exports', Bundle.outputFiles[0].text)(CreateRequire(import.meta.url), AppModule, AppModule.exports);
const Api = AppModule.exports;

// The arm at Home for the capture (the camera gate's replay of 15:01), and the joint limits the bridge reported then.
const Servos = structuredClone(Api.INITIAL_SERVOS);
for (const [Name, Raw] of Object.entries({ shoulder_pan: 3487, shoulder_lift: 3173, elbow_flex: 2686, wrist_flex: 864, wrist_roll: 1420, gripper: 699 })) {
  Object.assign(Servos[Name], { presentRaw: Raw, targetRaw: Raw, guiAngleRaw: null });
}
const Limits = { shoulder_pan: [-126.5625, 113.37890625], shoulder_lift: [-106.083984375, 105.46875], elbow_flex: [-90.263671875, 104.0625],
  wrist_flex: [-100.458984375, 105.029296875] };
function Grab(Name) {
  const Located = Api.ReadLocateResponse(Reply(Name));
  Assert.equal(Located.error, null);
  const Query = Api.ParseTaskObject('grab the pill bottle');
  const Tip = Api.mirrorDisplayPoint(Api.forwardKinematics(Api.currentAngles(Servos)).at(-1));
  const { Binding } = Api.ResolveTaskObject(Query, Located.objects, { LastSeen: null, Track: true, Tip });
  Assert.ok(Binding, 'the task binds the measured bottle');
  const Support = Api.TaskSupport(Binding.Object, Query.Name);
  Assert.equal(Support.Refusal, null);
  Assert.ok(Support.Floor.Scene, 'with its measured scene');
  const Context = { Servos, ElbowMode: 'down', Limits, ClearanceCm: 1.0, Floor: Support.Floor };
  return { Object: Binding.Object, Reach: Api.TaskObjectReachability(Binding.Object, Context, { Others: Api.NeighbourObjects(Located.objects, Binding.Object), Kind: Query.Name, Label: Query.Name }) };
}

Test('the replies are written from the recorded frame by this checkout', () => {
  Assert.equal(Missing, null);
});

// GRASP ANYTHING (2026-10-22): the books against it no longer refuse it before any search: they are an obstacle, the
// search finds the way down through space the camera has not observed and the hover out of reach, and the refusal says
// so with the books against it, and what to do.
Test('as placed, against the books, the grasp is refused for what blocks it, the books named, with what to do', () => {
  Assert.equal(Missing, null);
  const { Object, Reach } = Grab('recorded');
  Assert.ok(Math.hypot(Object.x + 25.0, Object.y - 40.7) < 1.0 && Math.abs(Object.z - 4.2) < 0.5, JSON.stringify(Object));
  Assert.equal(Reach.Kind, 'ungraspable');
  // With the task's reach taken as empty where unobserved (H2, on since 2026-10-25) the unseen space no longer refuses
  // it: what the camera saw does, the books against the bottle (the held bottle would touch them, the open jaw meets
  // what hides under them).
  Assert.doesNotMatch(Reach.Refusal.Text, /space the depth camera has not observed/);
  Assert.match(Reach.Refusal.Text, /held pill bottle would touch observed environment|hidden under observed environment/);
  Assert.match(Reach.Refusal.Text, /; and the camera did not measure it apart from the observed environment against it \(at X -2[6-9]\.\d, Y 4[2-4]\.\d, Z 1\d\.\d to 1\d\.\d\)$/);
  Assert.match(Reach.Refusal.Advice, / Or move the pill bottle 2 to 3 cm away from what stands against it\.$/);
});

Test('moved clear of the books, the grasp needs the space the camera cannot see beside the bottle', () => {
  Assert.equal(Missing, null);
  const { Reach } = Grab('moved_3cm');
  Assert.equal(Reach.Kind, 'ungraspable');
  Assert.match(Reach.Refusal.Text, /space the depth camera has not observed/);
});

Test("moved clear, with the task's reach taken as empty where unobserved (H2), it is planned with a near-level wrist", () => {
  Assert.equal(Missing, null);
  const { Object, Reach } = Grab('moved_3cm_reach');
  Assert.equal(Reach.Kind, 'graspable', Reach.Text);
  const Plan = Reach.Plan;
  Assert.ok(Plan.Pitch >= -20 && Plan.Pitch <= 0, `wrist ${Plan.Pitch}`);
  Assert.ok(Plan.Hover.z >= Object.topZ + 2.9 && Plan.Grasp.z > Object.z + 2 && Plan.Grasp.z < Object.topZ, JSON.stringify([Plan.Grasp, Plan.Hover]));
  Assert.ok(Reach.BaseDistanceCm > 38 && Reach.BaseDistanceCm < 40, `${Reach.BaseDistanceCm} cm from the pan axis`);
  const Pose = Api.solveIkPreview({ ...Api.modelPointFromDisplayPoint(Plan.Grasp), wristPitch: Plan.Pitch }, 'down', Servos);
  Assert.ok(Api.GripperLowestZ(Pose.angles) > Object.z + 1, 'the gripper over the cabinet top at the grasp');
  console.log(JSON.stringify({ Pitch: Plan.Pitch, Hover: Plan.Hover, Grasp: Plan.Grasp, Open: Plan.OpenFraction, Side: Plan.Side, Distance: Reach.BaseDistanceCm }));
});
