import { build as Build } from 'esbuild';
import { readFileSync as ReadFile } from 'node:fs';
import { execFileSync as Execute } from 'node:child_process';
import { createRequire as CreateRequire } from 'node:module';
import { test as Test } from 'node:test';
import Assert from 'node:assert/strict';
import Path from 'node:path';
import { fileURLToPath as FilePath } from 'node:url';
import { BackendPython } from './UiTestLib.mjs';

// 2026-10-04 08:30-08:32, "Grab the pill bottle" (back end/captures/2026-10-04/recording/ai.jsonl).
// 08:30:52: the bottle, measured as a "white cup" at X 10.0, Y 32.8 (08:30:35 as a "cable"), was refused:
// "the pill bottle measured at X 10.0, Y 32.8 is not graspable by this gripper: ... the lower arm link
// could contact the basket. Move the white cup farther from the basket". 08:31:44: moved to X 7.4, Y 33.2
// ("bottle") it was planned, the arm descended to the grasp point, and the bridge refused the close
// there: "Target arm pose: gripper link could contact the basket". No bridge, no hardware, no capture files.
const Root = Path.dirname(FilePath(import.meta.url));
const Bundle = await Build({ stdin: { contents: ReadFile(Path.join(Root, 'src/App.tsx'), 'utf8')
  + '\nexport * from "./lib/GraspPlan";'
  + '\nexport { TaskObjectReachability, ReachScan, ParseTaskObject, BuildGraspGuard, TaskObjectLine, BestEffortNote } from "./lib/AiMeasurements";'
  + '\nexport { BasketCollisionVersion, BasketPoseReason, BasketSurfaceReason } from "./lib/BasketCollision";'
  + '\nexport { CadForward } from "./lib/CadKinematics";'
  + '\nexport { ConvexGap } from "./lib/ConvexGap";'
  + '\nexport { INITIAL_SERVOS, currentAngles, forwardKinematics, mirrorDisplayPoint, modelPointFromDisplayPoint, solveIkPreview } from "./lib/robotModel";',
  resolveDir: Path.join(Root, 'src'), loader: 'tsx' }, bundle: true, platform: 'node', format: 'cjs', write: false, loader: { '.css': 'empty' }, logLevel: 'error' });
globalThis.window ??= { location: { hostname: 'localhost' } };
const Module = { exports: {} };
new Function('require', 'module', 'exports', Bundle.outputFiles[0].text)(CreateRequire(import.meta.url), Module, Module.exports);
const Api = Module.exports;

// --- The recorded run -------------------------------------------------------------------------

const Snapshot = Raw => {
  const Servos = structuredClone(Api.INITIAL_SERVOS);
  for (const [Name, Value] of Object.entries(Raw)) Object.assign(Servos[Name], { presentRaw: Value, targetRaw: Value, guiAngleRaw: null });
  return Servos;
};
// The model requests' "Current servo positions": at home (08:30-08:31, the arm had not moved), and at the grasp
// point at 08:32:36 (the descent to Z 3.4 stopped at Z 2.4), where the bridge refused the close.
const Home = Snapshot({ shoulder_pan: 3471, shoulder_lift: 3173, elbow_flex: 2688, wrist_flex: 856, wrist_roll: 1423, gripper: 696 });
const AtGrasp = { shoulder_pan: 3045, shoulder_lift: 891, elbow_flex: 1448, wrist_flex: 374, wrist_roll: 1423 };
const Context = { Servos: Home, ElbowMode: 'up', ClearanceCm: 1 };
const Obj = (label, x, y, z, topZ, sizeCm, supportZMeasured, validFraction, Extra = {}) => ({ label, x, y, z, topZ, heightCm: Number((topZ - z).toFixed(2)), sizeCm,
  depthMeasured: true, extrapolated: false, unstable: false, supportMeasured: true, supportZMeasured, validFraction, supportBranch: 'desk',
  taskMatch: false, taskQueryOnly: false, refined: false, outsideCalibrationCm: 0, armFraction: 0, ...Extra });
const Far = Cm => ({ extrapolated: true, outsideCalibrationCm: Cm });
// The /locate frames (ai.jsonl vision records), every measured object.
const Frame0852 = [Obj('keyboard', 14.55, 76.46, -0.15, 5.36, [12.7, 22.7], -0.17, 0.997, Far(44.1)), Obj('mouse', 15.96, 107.94, 1.32, 5.34, [6.3, 4.6], 1.44, 1, Far(74.6)),
  Obj('white cup', 10.01, 32.84, -3.1, 4.71, [3.3, 3.2], -2.9, 0.859, { taskMatch: true, refined: true, outsideCalibrationCm: 2.5, heightCm: 7.81 }),
  Obj('brown cup', -12.45, 32.65, -3.06, -0.95, [7.6, 8.5], -3.09, 0.478, { armFraction: 0.06 })];
const Frame0844 = [Obj('keyboard', 15.32, 71.5, 0, 4.16, [12.4, 26.4], 0.39, 1, Far(39.6)), Obj('phone', 1.13, 88.19, 0.12, 1.09, [6.4, 7.8], 0.56, 1, Far(51.8)),
  Obj('white container', 7.41, 33.27, -3.07, 4.83, [3.3, 3.1], -2.79, 0.784, { taskMatch: true, refined: true, outsideCalibrationCm: 1.7 }),
  Obj('cup', -12.85, 32.77, -3.02, -0.88, [7.6, 8.6], -3.1, 0.468, { armFraction: 0.052 }),
  Obj('bottle', 7.41, 33.2, -3.08, 4.79, [3.3, 3], -2.92, 0.807, { taskMatch: true, refined: true, outsideCalibrationCm: 1.6, heightCm: 7.87 })];
const Cup = Frame0852[2], Bottle = Frame0844[4];
const Query = Api.ParseTaskObject('Grab the pill bottle');
// As App.tsx BindTaskObject plans a grasp: the task's word for it is its Kind and its Label.
const Reach = (Object_, Frame, Others = Api.NeighbourObjects(Frame, Object_), Ctx = Context) =>
  Api.TaskObjectReachability(Object_, Ctx, { Others, Kind: Query.Name, Label: Query.Name });

// The two passes before the best-effort one (GraspRules.BestEffort false, 2026-10-04): their refusals.
const Strictly = Make => { const Saved = Api.GraspRules.BestEffort; Api.GraspRules.BestEffort = false; try { return Make(); } finally { Api.GraspRules.BestEffort = Saved; } };

// --- The label ----------------------------------------------------------------------------------

Test('08:30:52: a refusal names the pill bottle as the stop\'s headline does, in its text and its advice, never the detector\'s "white cup"', () => {
  Assert.equal(Query.Name, 'pill bottle');
  // App.tsx plans the task object with its task label (the headline is "the ${TaskObject.Label} measured at ...").
  // It plans in a worker (PlanReachability, 2026-10-07 audit M4), which hands the same three on to TaskObjectReachability.
  const App = ReadFile(Path.join(Root, 'src/App.tsx'), 'utf8');
  Assert.match(App, /Planned\(\{ Others: NeighbourObjects\(Measured \?\? \[\], Binding\.Object\), Kind: State\.Label,\s*Label: State\.Label \}\)/);
  Assert.match(App, /const Planned = async \(Planned_\?: ReachPlanned\) => \{\s*const Reach = await PlanReachability\(Binding\.Object, Context, Planned_, RequestAbort\.signal\);/);
  Assert.match(App, /worker\.postMessage\(\{ Object: Object_, Context, Planned, Alignment: GetCadAlignment\(\) \}\)/);
  Assert.match(ReadFile(Path.join(Root, 'src/lib/GraspPlanWorker.ts'), 'utf8'), /TaskObjectReachability\(Event\.data\.Object, Event\.data\.Context, Event\.data\.Planned\)/);
  Assert.match(App, /StopTaskWithReason\(`the \$\{TaskObject\.Label\} measured at X \$\{Found\.x\.toFixed\(1\)\}, Y \$\{Found\.y\.toFixed\(1\)\} is not graspable by this gripper: \$\{Refusal\.Text\}\.`, Refusal\.Advice\)/);
  // Every refusal path that names the object: unmeasured, too wide (turn it), a neighbour in the fixed jaw's way.
  const Battery = Obj('battery', 7.2, 30.2, -3.1, 4, [1.6, 1.6], -2.9, 0.9);
  const Cases = [['unmeasured', { ...Cup, sizeCm: null }, Frame0852], ['wide', { ...Cup, sizeCm: [2.5, 7] }, Frame0852],
    ['neighbour', { ...Cup, x: 4, y: 31 }, [...Frame0852, Battery]], ['recorded', Cup, Frame0852]];
  // (The two passes' refusals: GraspRules.BestEffort plans the wide and the recorded ones anyway, warned.)
  for (const [Name, Object_, Frame] of Cases) {
    const Verdict = Strictly(() => Reach(Object_, Frame, Api.NeighbourObjects(Frame, Cup)));
    if (Name !== 'recorded') Assert.equal(Verdict.Kind, 'ungraspable', `${Name}: ${Verdict.Text}`);
    if (Verdict.Kind !== 'ungraspable') continue;
    const { Text, Advice } = Verdict.Refusal;
    Assert.doesNotMatch(`${Text} ${Advice} ${Verdict.Text}`, /white cup/, Name);
    Assert.match(Advice, /the pill bottle/, `${Name}: ${Advice}`);
  }
  // Without a task label (a detector-only verdict), the measurement's own label as before.
  Assert.match(Api.TaskObjectReachability({ ...Cup, sizeCm: null }, Context, { Others: [], Kind: Query.Name }).Refusal.Advice, /the white cup/);
});

// --- CAD-exact contact (CollisionModel v8) ---------------------------------------------------------

Test('08:32: the grasp pose the arm stood in beside the bottle is clear of the basket, jaws open or closed; turned into the cart it is not', () => {
  // v7 refused it ("gripper link could contact the basket"): the gripper box was 2.4 cm from the cart and only its
  // 16-step tracking envelope reached it. The app and the bridge (TestCadExactContact.py) judge the CAD boxes alone.
  Assert.equal(Api.BasketCollisionVersion, 'robot-body-cad-exact-v8');
  for (const Gripper of [1438, 642]) Assert.equal(Api.BasketPoseReason(Api.currentAngles(Snapshot({ ...AtGrasp, gripper: Gripper }))), null, `gripper raw ${Gripper}`);
  // One arm joint turned from there until a link's CAD box meets the cart: refused.
  const Start = Api.currentAngles(Snapshot({ ...AtGrasp, gripper: 1438 }));
  let Into = null;
  for (const Joint of ['wrist_flex', 'elbow_flex', 'shoulder_lift']) {
    for (const Sign of [1, -1]) {
      for (let Step = 1; Step <= 120 && !Into; Step++) {
        const Angles = { ...Start, [Joint]: Start[Joint] + Sign * Step * 0.5 };
        const Reason = Api.BasketPoseReason(Angles);
        if (Reason) { if (/basket/.test(Reason)) Into = Angles; break; }
      }
      if (Into) break;
    }
    if (Into) break;
  }
  Assert.ok(Into, 'no single joint turn met the cart');
  Assert.match(Api.BasketSurfaceReason(Into), /could contact the basket/);
});

// --- Try unless certain (GraspRules.BestEffort) --------------------------------------------------

// 2026-10-04 16:43-16:49 (ai.jsonl): a taller bottle and a black cube, every one refused, while Aaran knew the arm can
// grab them: "It won't grab the pill bottle, just let it try unless your code is totally sure it can't grab it."
// The model requests' servos then (16:45:42; the cube's plans hold the same wrist roll).
const Home1645 = Snapshot({ shoulder_pan: 3475, shoulder_lift: 3172, elbow_flex: 2688, wrist_flex: 854, wrist_roll: 1423, gripper: 701 });
const T = { taskMatch: true, refined: true }, X = { extrapolated: true };
const Frame1643 = [Obj('bottle', 9.32, 34.39, -2.94, 10.77, [5.3, 2.8], -2.08, 0.798, { ...T, outsideCalibrationCm: 3.5 }),
  Obj('box', 9.2, 32.87, -3.17, 6.12, [6.5, 4.5], -2.88, 0.902, { outsideCalibrationCm: 2.1 }), Obj('cup', -12.42, 32.52, -3.14, -0.38, [8.1, 8.2], -3.32, 0.435, { armFraction: 0.118 })];
const Frame1644 = [Obj('bottle', 9.3, 34.56, -2.94, 10.79, [5.3, 3.1], -2.05, 0.797, { ...T, outsideCalibrationCm: 3.7 }),
  Obj('box', 9.14, 32.87, -3.17, 6.02, [6.4, 4.3], -2.93, 0.917, { outsideCalibrationCm: 2.1 }), Obj('cup', -12.32, 32.61, -3.15, -0.66, [7.5, 8.2], -3.35, 0.426, { armFraction: 0.057 })];
const Frame1645 = [Obj('bottle', 6.86, 29.31, -3.26, 10.43, [4.9, 3], -2.44, 0.782, T), Obj('blue box', 6.93, 27.75, -3.48, 5.83, [6.7, 5.1], -3.35, 0.882),
  Obj('cup', -12.09, 32.68, -3.14, -0.84, [7.6, 8.4], -3.36, 0.416, { armFraction: 0.077 }), Obj('blue block', 6.85, 27.7, -3.48, 6.16, [6.7, 5.1], -3.29, 0.868)];
const Frame1649 = [Obj('black cube', 6.01, 35.26, -2.92, 1.74, [3.2, 1.2], -2.94, 1, { ...T, outsideCalibrationCm: 2.8 }),
  Obj('white bottle', 37.18, 42.5, -2.66, 5.2, [4.3, 3.5], -2.26, 0.901, { ...X, outsideCalibrationCm: 26.7 }), Obj('blue box', 35.87, 32.43, -3.32, 2.8, [6.1, 5.9], -2.94, 0.643, X),
  Obj('small red box', 28.93, 36.23, -2.8, -1.07, [2.4, 3.8], -2.67, 0.423, X), Obj('white cup', -14.81, 31.29, -3.27, -0.33, [8.7, 7.9], -3.3, 0.509, { armFraction: 0.338 }),
  Obj('cup', -12.62, 32.46, -3.27, -0.73, [8.3, 8], -3.3, 0.486, { armFraction: 0.226 }), Obj('black block', 5.89, 35.05, -2.91, 1.61, [3.3, 0.9], -2.94, 0.998, T)];
const ReachAs = (Frame, Name, Servos, Others = Api.NeighbourObjects(Frame, Frame[0])) =>
  Api.TaskObjectReachability(Frame[0], { Servos, ElbowMode: 'up', ClearanceCm: 1 }, { Others, Kind: Name, Label: Name });
// The recorded frames with the task object first (ReachAs plans Frame[0]).
const TaskFirst = (Frame, Task) => [Task, ...Frame.filter(Object_ => Object_ !== Task)];
const StateOf = (Frame, Reach, Name, Servos) => ({ Label: Name, DetectedNow: true, Binding: { Object: Frame[0], Source: 'name', DistanceCm: null },
  LastSeen: { At: 0, Object: Frame[0], Reachability: Reach }, Reachability: Reach, Intent: 'grasp',
  Plan: { Reach, Source: Frame[0], At: 0, Key: Api.GraspPlanKey(Servos, 'up'), Note: null } });
// The plan's own descent (hover to grasp) as the guard judges the model's copy of that step.
function GuardAllowsPlan(Frame, Reach, Name, Servos) {
  const Plan = Reach.Plan;
  const Guard = Api.BuildGraspGuard(Frame, StateOf(Frame, Reach, Name, Servos), false);
  const Target = { ...Api.modelPointFromDisplayPoint(Plan.Grasp), wristPitch: Plan.Pitch };
  return Api.GraspGuardReason(Target, Plan.Poses.Grasp, Plan.Poses.Hover, Plan.Grasp.z - Plan.Hover.z, Guard);
}

// (As the best-effort pass came, before the deep grasp: GraspRules.DeepGrasp false. Deep, 08:30:52 and 16:45:12 plan with every
// check and 16:44:14 is refused as too wide, not out of reach, by the passes before it: below.)
Test('16:43-16:45 and 08:30:52: the bottles refused for what only predicts a failed grip are planned best-effort, warned, and the guard lets the plan run', () => Shallow(() => {
  for (const [Name, Frame, Servos, Was] of [['16:43:49', Frame1643, Home1645, 'wide'], ['16:44:14', Frame1644, Home1645, 'reach'], ['16:45:12', Frame1645, Home1645, 'wide'],
    ['08:30:52', TaskFirst(Frame0852, Cup), Home, 'path']]) {
    // Refused as the passes before planned it (16:45:12 was "path" for the basket's tracking envelope, "wide" since CAD-exact contact).
    Assert.equal(Strictly(() => ReachAs(Frame, 'pill bottle', Servos)).Refusal?.Code, Was, Name);
    const Reach = ReachAs(Frame, 'pill bottle', Servos), Plan = Reach.Plan;
    Assert.equal(Reach.Kind, 'graspable', `${Name}: ${Reach.Text}`);
    Assert.ok(Plan.Warnings?.length, `${Name}: a best-effort plan says what it is not sure of`);
    Assert.match(Reach.Text, /^graspable \(best effort: the app is not sure it holds\): GRASP POINT/);
    // What is certain still holds: the arm reaches the printed points clear of the basket and above the desk (ExactlyReachable).
    const Context_ = { Servos, ElbowMode: 'up', Others: [], Scan: () => null };
    Assert.ok(Api.ExactlyReachable(Plan.Grasp, Plan.Pitch, Context_, Plan.Hover.z) && Api.ExactlyReachable(Plan.Hover, Plan.HoverPitch, Context_, Plan.Hover.z), Name);
    for (const Pose of [Plan.Poses.Grasp, Plan.Poses.Hover]) Assert.equal(Api.BasketPoseReason(Pose), null, Name);
    // The model is told, and the approval card says so; the guard lets the plan's own descent run.
    const Line = Api.TaskObjectLine(StateOf(Frame, Reach, 'pill bottle', Servos), Api.mirrorDisplayPoint(Api.forwardKinematics(Api.currentAngles(Servos)).at(-1)),
      { GripperFraction: 0.04, Closed: false });
    Assert.match(Line, /BEST-EFFORT GRASP: the app is not sure this grasp holds the pill bottle \(/, Name);
    Assert.match(Api.BestEffortNote(Plan), /^Best-effort grasp, may not work: /);
    Assert.equal(GuardAllowsPlan(Frame, Reach, 'pill bottle', Servos), null, Name);
  }
  // 08:31:44 (7.41, 33.2) plans with every check, as it did: no warnings.
  const Planned = ReachAs(TaskFirst(Frame0844, Bottle), 'pill bottle', Home);
  Assert.equal(Planned.Kind, 'graspable');
  Assert.equal(Planned.Plan.Warnings, undefined);
  Assert.equal(Api.BestEffortNote(Planned.Plan), null);
}));

Test('16:49:45: the black cube refused as too wide for the jaw table and too lean a grip is planned best-effort; the wrist roll cannot turn the jaws across its 1.2 cm side', () => {
  const Strict = Strictly(() => ReachAs(Frame1649, 'black thing', Home1645));
  Assert.equal(Strict.Refusal?.Code, 'wide');
  Assert.match(Strict.Text, /closing jaws would need friction for 23 deg at a pad \(at most 20\)/);
  const Reach = ReachAs(Frame1649, 'black thing', Home1645), Plan = Reach.Plan;
  Assert.equal(Reach.Kind, 'graspable', Reach.Text);
  Assert.ok(Plan.Warnings.some(Why => /^the grip may slip \(.*closing jaws would need friction for \d+ deg at a pad \(at most 20\)/.test(Why)), JSON.stringify(Plan.Warnings));
  Assert.equal(GuardAllowsPlan(Frame1649, Reach, 'black thing', Home1645), null);
  // Level jaws close only across the arm's reach: to span its 1.2 cm side (along the reach there, 29 deg off it) the wrist
  // roll would need the fingers pointing straight down, and 40 cm out the arm reaches its centre no steeper than -41 deg.
  const Cube = Frame1649[0], Pan = Api.mirrorDisplayPoint({ x: 0, y: 0, z: 0 }), Mid = { x: Cube.x, y: Cube.y, z: (Cube.z + Cube.topZ) / 2 };
  Assert.ok(Math.hypot(Cube.x - Pan.x, Cube.y - Pan.y) > 40);
  Assert.ok(Api.ReachScan(Api.modelPointFromDisplayPoint(Mid), { Servos: Home1645, ElbowMode: 'up' }).Pitch > -45);
  for (const Pitch of [-90, -80]) Assert.equal(Api.solveIkPreview({ ...Api.modelPointFromDisplayPoint(Mid), wristPitch: Pitch }, 'up', Home1645).reachable, false, `${Pitch}`);
});

Test('what is certain is still refused: an object out of IK reach (16:48:45), one the gripper meets the cart at, one thinner than the closed pads', () => {
  // 16:48:45 "the black thing measured at X 13.8, Y 36.1 is out of the right arm's reach": at no pitch does the tip reach
  // within 0.5 cm of its near face (44.1 cm out, its centre 45.1), at the bottom, middle or top of its side.
  const Block = Obj('black block', 13.8, 36.07, -2.87, 1.8, [3.5, 2], -2.7, 0.947, { ...T, outsideCalibrationCm: 7.1 });
  const Far = ReachAs([Block], 'black thing', Home1645, []);
  Assert.equal(Far.Kind, 'out-of-reach', Far.Text);
  const Pan = Api.mirrorDisplayPoint({ x: 0, y: 0, z: 0 }), Bearing = Math.atan2(Block.y - Pan.y, Block.x - Pan.x);
  const Near = Math.hypot(Block.x - Pan.x, Block.y - Pan.y) - Block.sizeCm[1] / 2 - 0.5;
  for (const Z of [-2.3, -0.5, 1.3]) {
    const P = { x: Pan.x + Near * Math.cos(Bearing), y: Pan.y + Near * Math.sin(Bearing), z: Z };
    Assert.equal(Api.ReachScan(Api.modelPointFromDisplayPoint(P), { Servos: Home1645, ElbowMode: 'up' }).Pitch, null, `the tip at Z ${Z}, 0.5 cm short of its near face`);
  }
  // Against the cart's front, every pose that reaches it puts a link's CAD box into the cart: refused, as with every check.
  const AtCart = Obj('box', 5, 27, -2.3, -0.3, [2, 2], -2.3, 0.9, T);
  const Cart = ReachAs([AtCart], 'box', Home, []);
  Assert.notEqual(Cart.Kind, 'graspable', Cart.Text);
  Assert.equal(Strictly(() => ReachAs([AtCart], 'box', Home, [])).Kind, Cart.Kind);
  // Thinner than the pads' gap when the gripper is fully closed: no grip at all.
  const Thin = ReachAs([Obj('pin', 0, 32, -2.5, -1.5, [0.4, 0.5], -2.5, 0.9, T)], 'pin', Home, []);
  Assert.equal(Thin.Refusal?.Code, 'thin', Thin.Text);
});

// --- The deep grasp (GraspRules.DeepGrasp) ---------------------------------------------------------

// Aaran, 2026-10-04: "It needs to go like one inch farther into the object. In the CAD view I want the wireframe of the
// grab object to be almost touching the servo of the gripper." A round object's plan goes as deep in the jaws as the
// palm and the STS3215 servo (their CAD hull, PalmGap) and the arm's reach let it, PalmGapCm from them where it can.
const Shallow = Make => { const Saved = Api.GraspRules.DeepGrasp; Api.GraspRules.DeepGrasp = false; try { return Make(); } finally { Api.GraspRules.DeepGrasp = Saved; } };
// The pose where a plan's jaws close: the arm stops SagCm under the printed grasp point.
const ClosePose = Plan => ({ ...Api.solveIkPreview({ ...Api.modelPointFromDisplayPoint({ ...Plan.Grasp, z: Plan.Grasp.z - (Plan.SagCm ?? 0) }), wristPitch: Plan.Pitch },
  'up', Home).angles, gripper: Plan.Poses.Grasp.gripper });
// The gripper_link frame in the display frame (GraspPlan.ts ToDisplay): rows of a 3 x 4 affine map.
const DisplayAffine = Angles => {
  const E = Api.CadForward(Angles).Links.gripper_link.elements;
  const At = P => Api.mirrorDisplayPoint({ x: E[0] * P[0] + E[4] * P[1] + E[8] * P[2] + E[12], y: E[1] * P[0] + E[5] * P[1] + E[9] * P[2] + E[13], z: E[2] * P[0] + E[6] * P[1] + E[10] * P[2] + E[14] });
  const O = At([0, 0, 0]), Axes = [[1, 0, 0], [0, 1, 0], [0, 0, 1]].map(At).map(P => [P.x - O.x, P.y - O.y, P.z - O.z]);
  return ['x', 'y', 'z'].map((K, Row) => [Axes[0][Row], Axes[1][Row], Axes[2][Row], O[K]]);
};
// The gap from a prism to the palm and servo's CAD MESHES (not the planner's hull), from the GLB with trimesh: the mesh
// surface sampled every 0.02 cm near it, each sample's exact distance to the prism. Servo: the STS3215's mesh alone.
const Python = BackendPython();
function MeshGap(Angles, Prism, Servo = false) {
  const Script = `
import json, sys, numpy as np, trimesh
A = json.loads(sys.argv[1]); M = np.array(A["Affine"]); F = np.array(A["Footprint"]); Z0, Z1 = A["BaseZ"], A["TopZ"]
S = trimesh.load(A["Glb"], force="scene"); C = json.load(open(A["Cad"]))
Parts = []
for V in C["Visuals"]:
    if V["Name"] not in ("gripper_link__0", "gripper_link__1") or (A["Servo"] and V["Name"] != "gripper_link__0"): continue
    T, G = S.graph[V["Name"]]; Mesh = S.geometry[G].copy(); Mesh.apply_transform(T); Mesh.apply_scale(100)
    if V["Name"] == "gripper_link__1":
        Keep = (Mesh.vertices[Mesh.faces][:, :, 2] >= -3.77).all(axis=1); Mesh.update_faces(Keep); Mesh.remove_unreferenced_vertices()
    Parts.append(Mesh)
Mesh = trimesh.util.concatenate(Parts)
Mesh.vertices = Mesh.vertices @ M[:, :3].T + M[:, 3]
def Gap(P):
    D = np.full(len(P), np.inf); Inside = np.ones(len(P), bool)
    for I in range(len(F)):
        A_, B_ = F[I], F[(I + 1) % len(F)]; E = B_ - A_; L = E @ E
        T_ = np.clip(((P[:, :2] - A_) @ E) / L, 0, 1); D = np.minimum(D, np.linalg.norm(P[:, :2] - (A_ + T_[:, None] * E), axis=1))
        Inside &= (E[0] * (P[:, 1] - A_[1]) - E[1] * (P[:, 0] - A_[0])) * A["Winding"] >= 0
    Flat = np.where(Inside, 0, D); Up = np.maximum(0, np.maximum(Z0 - P[:, 2], P[:, 2] - Z1))
    return np.hypot(Flat, Up)
Near = Gap(Mesh.vertices) < Gap(Mesh.vertices).min() + 1.5
Faces = Mesh.faces[Near[Mesh.faces].any(axis=1)]
Sub = trimesh.Trimesh(Mesh.vertices, Faces, process=False)
Points, _ = trimesh.sample.sample_surface_even(Sub, int(Sub.area / 0.02 ** 2), radius=0.015, seed=1)
print(float(min(Gap(np.vstack([Points, Sub.vertices])).min(), Gap(Mesh.vertices).min())))
`;
  const Signed = Prism.Footprint.reduce((Sum, P, I) => { const Q = Prism.Footprint[(I + 1) % Prism.Footprint.length]; return Sum + P.x * Q.y - Q.x * P.y; }, 0);
  const Args = JSON.stringify({ Affine: DisplayAffine(Angles), Footprint: Prism.Footprint.map(P => [P.x, P.y]), BaseZ: Prism.BaseZ, TopZ: Prism.TopZ, Servo,
    Winding: Math.sign(Signed), Glb: Path.join(Root, 'public/Cad/SO101Parts.glb'), Cad: Path.join(Root, '../back end/CadModel.json') });
  return Number(Execute(Python, ['-c', Script, Args], { encoding: 'utf8', maxBuffer: 1e6 }).trim());
}

Test('ConvexGap: the exact gap between convex shapes, from below; 0 where they meet', () => {
  const Box = (C, H) => [-1, 1].flatMap(X => [-1, 1].flatMap(Y => [-1, 1].map(Z => ({ x: C.x + X * H, y: C.y + Y * H, z: C.z + Z * H }))));
  const Support = Points => D => Points.reduce((A, B) => (B.x * D.x + B.y * D.y + B.z * D.z > A.x * D.x + A.y * D.y + A.z * D.z ? B : A));
  const Unit = Box({ x: 0, y: 0, z: 0 }, 1);
  Assert.ok(Math.abs(Api.ConvexGap(Support(Unit), Support(Box({ x: 3, y: 0.5, z: 0 }, 1)), { x: 1, y: 0, z: 0 }) - 1) < 1e-3);
  Assert.ok(Math.abs(Api.ConvexGap(Support(Unit), Support(Box({ x: 3, y: 3, z: 3 }, 1)), { x: 1, y: 0, z: 0 }) - Math.sqrt(3)) < 1e-3);
  Assert.equal(Api.ConvexGap(Support(Unit), Support(Box({ x: 1.5, y: 0, z: 0 }, 1)), { x: 1, y: 0, z: 0 }), 0);
});

Test('08:31:44: the deep grasp puts the bottle far deeper in the jaws, its prism near the palm and the STS3215 servo, as the CAD meshes measure it', () => {
  const Frame = TaskFirst(Frame0844, Bottle);
  // (Deep against shallow with the fixed jaw's gap as before; with its extra standoff, GraspRules.FixedJawExtraStandoffCm, below.)
  const OldGap = Make => { const Saved = Api.GraspRules.FixedJawExtraStandoffCm; Api.GraspRules.FixedJawExtraStandoffCm = 0; try { return Make(); } finally { Api.GraspRules.FixedJawExtraStandoffCm = Saved; } };
  const Before = OldGap(() => Shallow(() => ReachAs(Frame, 'pill bottle', Home))).Plan, After = OldGap(() => ReachAs(Frame, 'pill bottle', Home)).Plan;
  Assert.ok(Before && After && !After.Warnings, 'both plan with every check');
  Assert.equal(Before.Deep, undefined);
  Assert.equal(After.Deep, true);
  // The palm and servo where the jaws close: 2.4 cm from it before, now as near as the arm's reach lets it go at that wrist
  // (the bottle stands 39 cm from the pan axis: deeper, the tip goes farther out), and never nearer than PalmGapCm.
  const Was = Api.PalmGap(Api.CadForward(ClosePose(Before)).Links, Before.Prism);
  Assert.ok(Was > 2.2, `${Was}`);
  Assert.ok(After.PalmGapCm >= Api.GraspRules.PalmGapCm - 0.01 && After.PalmGapCm <= 0.9, `${After.PalmGapCm}`);
  Assert.ok(After.Closing.DepthOffsetCm - Before.Closing.DepthOffsetCm >= 1.5, `${Before.Closing.DepthOffsetCm} -> ${After.Closing.DepthOffsetCm}`);
  Assert.ok(Math.min(After.Straddle.FixedCm, After.Straddle.MovingCm) >= 3, JSON.stringify(After.Straddle));
  // With the fixed jaw 2.54 cm farther out (the full extra fits): deep with every check still, at wrist -26.4, the palm 1.1 cm from
  // it (the arm reaches no deeper there), the moving jaw 2.2 cm past its centre (more than StraddleCm), the fixed one 3.0.
  const Standoff = ReachAs(Frame, 'pill bottle', Home).Plan;
  Assert.ok(Standoff.Deep && !Standoff.Warnings && Standoff.ExtraStandoffCm === 2.54, JSON.stringify([Standoff.Deep, Standoff.Warnings, Standoff.ExtraStandoffCm]));
  Assert.ok(Standoff.PalmGapCm >= Api.GraspRules.PalmGapCm - 0.01 && Standoff.PalmGapCm <= 1.2, `${Standoff.PalmGapCm}`);
  Assert.ok(Math.min(Standoff.Straddle.FixedCm, Standoff.Straddle.MovingCm) >= Api.GraspRules.StraddleCm, JSON.stringify(Standoff.Straddle));
  // Measured on the CAD meshes (trimesh, the GLB) where the printed grasp point's jaws close: the planner's hull gap is never
  // more than the meshes', and within 0.12 cm of it; the plan's own (at its unrounded pose) within the rounding of it.
  const Pose = ClosePose(After);
  for (const [Servo, Planned] of [[false, After.PalmGapCm], [true, After.ServoGapCm]]) {
    const Hull = Api.PalmGap(Api.CadForward(Pose).Links, After.Prism, Servo), Mesh = MeshGap(Pose, After.Prism, Servo);
    Assert.ok(Hull <= Mesh + 0.002 && Mesh - Hull <= 0.12, `${Servo ? 'servo' : 'palm and servo'}: hull ${Hull}, mesh ${Mesh}`);
    Assert.ok(Math.abs(Hull - Planned) <= 0.15, `${Servo ? 'servo' : 'palm and servo'}: planned ${Planned}, at the printed point ${Hull}`);
  }
  // The model is told, and the session record carries the gaps (App.tsx grasp_plan palm_gap_cm, servo_gap_cm).
  Assert.match(Api.TaskObjectLine(StateOf(Frame, ReachAs(Frame, 'pill bottle', Home), 'pill bottle', Home), Api.mirrorDisplayPoint(Api.forwardKinematics(Api.currentAngles(Home)).at(-1)),
    { GripperFraction: 0.04, Closed: false }), /It sits deep in the jaws, the gripper's palm and servo \d\.\d cm from it at the GRASP POINT/);
  Assert.match(ReadFile(Path.join(Root, 'src/App.tsx'), 'utf8'), /palm_gap_cm: Plan\?\.PalmGapCm/);
});

Test('08:30:52 and 16:45:12: deep, the pill bottle refused at 08:30 plans with every check; the guard runs each deep plan\'s descent and refuses the palm into the bottle', () => {
  for (const [Name, Frame, Servos, Kind] of [['08:30:52', TaskFirst(Frame0852, Cup), Home, 'pill bottle'], ['16:45:12', Frame1645, Home1645, 'pill bottle']]) {
    const Reach = ReachAs(Frame, Kind, Servos), Plan = Reach.Plan;
    Assert.equal(Reach.Kind, 'graspable', `${Name}: ${Reach.Text}`);
    Assert.ok(Plan.Deep && !Plan.Warnings, `${Name}: ${JSON.stringify(Plan.Warnings)}`);
    Assert.ok(Plan.PalmGapCm >= Api.GraspRules.PalmGapCm - 0.01, `${Name}: ${Plan.PalmGapCm}`);
    Assert.equal(GuardAllowsPlan(Frame, Reach, Kind, Servos), null, Name);
    // 1.5 cm farther in along the fingers (at the grasp height), the palm (or the open moving jaw) meets the bottle: refused.
    const Guard = Api.BuildGraspGuard(Frame, StateOf(Frame, Reach, Kind, Servos), false);
    const F = Plan.Straddle.FingerAxis, L = Math.hypot(F.x, F.y);
    const Deeper = { ...Plan.Grasp, x: Plan.Grasp.x + 1.5 * F.x / L, y: Plan.Grasp.y + 1.5 * F.y / L };
    const Preview = Api.solveIkPreview({ ...Api.modelPointFromDisplayPoint(Deeper), wristPitch: Plan.Pitch }, 'up', Servos);
    if (!Preview.reachable) continue;
    const Why = Api.GraspGuardReason({ ...Api.modelPointFromDisplayPoint(Deeper), wristPitch: Plan.Pitch }, { ...Preview.angles, gripper: Plan.Poses.Grasp.gripper },
      Plan.Poses.Hover, Deeper.z - Plan.Hover.z, Guard);
    Assert.match(Why ?? '', /(palm|open moving jaw) would (come within \d\.\d cm of|go \d\.\d cm into) the pill bottle/, `${Name}: ${Why}`);
  }
});
