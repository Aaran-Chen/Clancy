// Offline geometry only: no browser, bridge, camera or serial device is opened.
import { build as Build } from 'esbuild';
import { createRequire as CreateRequire } from 'node:module';
import { test as Test } from 'node:test';
import Assert from 'node:assert/strict';
import Path from 'node:path';
import { fileURLToPath as FilePath } from 'node:url';
import { PinnedHome } from './UiTestLib.mjs';

const Root = Path.dirname(FilePath(import.meta.url));
const Bundle = await Build({ stdin: { contents: 'export * from "./lib/GraspPlan"; export * from "./lib/AiTableFloor"; export * from "./lib/ObservedSpace"; export * from "./lib/robotModel";',
  resolveDir: Path.join(Root, 'src'), loader: 'ts' }, bundle: true, platform: 'node', format: 'cjs', write: false, plugins: [PinnedHome], logLevel: 'error' });
const Module = { exports: {} };
new Function('require', 'module', 'exports', Bundle.outputFiles[0].text)(CreateRequire(import.meta.url), Module, Module.exports);
const Api = Module.exports;
const Box = (min, max) => ({ min, max });
const Support = { Name: 'observed shelf', Kind: 'shelf', TopZCm: 12.7, XRangeCm: [-25, -3], YRangeCm: [30, 44], BoardAboveZCm: 28,
  Frame: 'cart', Source: 'depth', ObservedOnly: true };
const Scene = { id: 'fixture', revision: 'one', frame: 'display_cm', capture_seq: 1, support: Support,
  obstacles: [{ label: 'shelf support', ...Box([-25, 30, 11.7], [-3, 44, 12.7]) }, { label: 'upper shelf', ...Box([-25, 30, 28], [-3, 44, 29]) }],
  free_space: [Box([-80, -10, -20], [80, 100, 80])], task_object: Box([-16.4, 30.7, 12.7], [-10.4, 36.3, 24.7]) };
const Floor = { ...Api.SurfaceFloor(Support), Scene };
const Home = structuredClone(Api.INITIAL_SERVOS);
const Object_ = Api.GraspPrism({ label: 'can', x: -13.4, y: 33.5, z: 12.7, topZ: 24.7, supportZMeasured: 12.7,
  sizeCm: [5, 4.6], depthMeasured: true }, 'can');
const Context = { Servos: Home, ElbowMode: 'up', Others: [], Floor, Scan: () => -30 };

Test('observed box union preserves unknown holes and accepts truly adjoining coverage', () => {
  const Target = Box([0, 0, 0], [2, 2, 2]);
  Assert.equal(Api.ObservedBoxCovered(Target, [Box([0, 0, 0], [1, 2, 2]), Box([1, 0, 0], [2, 2, 2])]), true);
  Assert.equal(Api.ObservedBoxCovered(Target, [Box([0, 0, 0], [0.99, 2, 2]), Box([1.01, 0, 0], [2, 2, 2])]), false);
  Assert.equal(Api.ObservedBoxCovered(Target, []), false);
});

Test('observed raised floor is local and unknown/lower transit retains the desk safeguard', () => {
  Assert.equal(Api.FloorZAt(Floor, { x: -14, y: 33 }), 11.7);
  Assert.equal(Api.FloorZAt(Floor, { x: 20, y: 33 }), Api.AiTableFloorCm);
  const Legacy = Api.SurfaceFloor(Support);
  Assert.equal(Api.FloorZAt(Legacy, { x: 20, y: 33 }), 11.7);
  const Lower = { ...Api.SurfaceFloor({ ...Support, TopZCm: -8 }), Scene };
  Assert.equal(Api.FloorZAt(Lower, { x: -14, y: 33 }), -9);
  Assert.equal(Api.FloorZAt(Lower, { x: 20, y: 33 }), Api.AiTableFloorCm);
});

Test('a scene without observed free space refuses before grasp search', () => {
  const Result = Api.PlanGrasp(Object_, -30, { ...Context, Floor: { ...Floor, Scene: { ...Scene, free_space: [] } } });
  Assert.equal(Result.Plan, null);
  Assert.match(Result.Refusal.Text, /free approach corridor/);
  Assert.equal(Result.Tried.length, 0);
});

let Plan;
Test('a reachable shelf uses side entry, release and retreat under a board instead of a vertical hover', () => {
  const Result = Api.PlanGrasp(Object_, -30, Context, false);
  Assert.ok(Result.Plan, JSON.stringify(Result.Refusal));
  Plan = Result.Plan;
  Assert.equal(Plan.Approach.Kind, 'side');
  Assert.equal(Plan.Hover.z, Plan.Grasp.z);
  Assert.ok(Plan.Hover.y < Plan.Grasp.y);
  Assert.ok(Math.abs(Plan.Approach.Release.z - Plan.Grasp.z - 1.2) < 1e-8);
  Assert.equal(Plan.Approach.Retreat.z, Plan.Approach.Release.z);
  Assert.equal(Plan.HoverPitch, Plan.Pitch);
  Assert.ok(Plan.Grasp.z + Api.GraspRules.HoverAboveGraspCm > Plan.Hover.z);
  console.log(JSON.stringify({ Pitch: Plan.Pitch, Grasp: Plan.Grasp, Approach: Plan.Approach }));
});

Test('carried-object sweep refuses an overhead obstacle even with the tip and gripper below it', () => {
  Assert.ok(Plan);
  const Lift = Api.solveIkPreview({ ...Api.modelPointFromDisplayPoint(Plan.Approach.Release), wristPitch: Plan.Pitch }, 'up', Home).angles;
  const Overhead = Api.ScenePrisms({ ...Floor, Scene: { ...Scene, obstacles: [{ label: 'low overhang', ...Box([-30, 28, 25.5], [0, 42, 26]) }] } });
  Assert.equal(Api.CarriedObjectPathReason(Plan.Prism, Plan.Poses.Grasp, Plan.Poses.Grasp, Plan.Poses.Grasp, Overhead), null);
  Assert.match(Api.CarriedObjectPathReason(Plan.Prism, Plan.Poses.Grasp, Plan.Poses.Grasp, Lift, Overhead), /held can.*low overhang/);
});

Test('a solid blocking the shelf corridor refuses even when free-space data overlaps it', () => {
  const Blocked = { ...Scene, obstacles: [...Scene.obstacles, { label: 'closed shelf front', ...Box([-40, 25, 13], [10, 40, 40]) }] };
  const Result = Api.PlanGrasp(Object_, -20, { ...Context, Floor: { ...Floor, Scene: Blocked } }, false);
  Assert.equal(Result.Plan, null);
  Assert.ok(Result.Refusal);
});

Test('an unreachable shelf cannot turn free-space coverage into arm reach', () => {
  const Far = { ...Object_, Centre: { x: -13.4, y: 90 }, Footprint: Object_.Footprint.map(P => ({ ...P, y: P.y + 56.5 })) };
  const Result = Api.PlanGrasp(Far, 0, Context, false);
  Assert.equal(Result.Plan, null);
});

Test('the entire withdrawal is required to remain in observed space', () => {
  Assert.ok(Plan);
  const Far = { ...Floor, Scene: { ...Scene, free_space: [Box([50, 50, 50], [60, 60, 60])] } };
  Assert.match(Api.ScenePathCoverageReason(Plan.Poses.Hover, Plan.Poses.Grasp, Far), /not observed/);
});

Test('the object meets an observed solid only when one lies inside its measured points: padding, the support and its tilt are not one', () => {
  // The scene pads the task envelope and every obstacle's 1 cm cell by 0.5 cm (back end/SceneGeometry.py DepthUncertaintyCm).
  const Pad = Api.SceneRules.DepthUncertaintyCm;
  const Task = Box([-16.4, 30.7, 12.7], [-10.4, 36.3, 24.7]);
  const With = Obstacles => ({ ...Scene, task_object: Task, obstacles: Obstacles });
  // A neighbour's cell 0.6 cm beside the object's points: their padded boxes overlap, its observed cell does not.
  const Beside = { label: 'observed environment', ...Box([-10.4 - Pad + 0.6 - Pad, 32, 14 - Pad], [-9.4 - Pad + 0.6 + Pad, 33, 15 + Pad]) };
  Assert.ok(Beside.min[0] < Task.max[0], 'the padded boxes overlap');
  Assert.equal(Api.SceneConflict(With([Beside])), null);
  // Cells of the support's level under and beside its base (a tilted top rises up to SupportBandCm), capped or not: the support.
  const Tilted = { label: 'observed environment', ...Box([-17 - Pad, 31 - Pad, 12 - Pad], [-15 + Pad, 33 + Pad, 13.6 + Pad]) };
  Assert.equal(Api.SceneConflict(With([Scene.obstacles[0], Tilted])), null);
  // A board through it, or a neighbour's side merged into its measurement: an observed cell inside its points.
  const Board = { label: 'shelf board', ...Box([-25 - Pad, 30 - Pad, 18 - Pad], [-3 + Pad, 44 + Pad, 19 + Pad]) };
  const Merged = { label: 'observed environment', ...Box([-12 - Pad, 35 - Pad, 20 - Pad], [-10 + Pad, 37 + Pad, 21 + Pad]) };
  for (const Solid of [Board, Merged]) Assert.equal(Api.SceneConflict(With([Solid])).Box, Solid);
  // The planner refuses with where it is and what to do.
  const Refused = Api.PlanGrasp(Object_, -30, { ...Context, Floor: { ...Floor, Scene: With([Scene.obstacles[0], Merged]) } });
  Assert.equal(Refused.Plan, null);
  Assert.equal(Refused.Refusal.Text, 'the camera measured the can running into observed environment beside it (at X -11.0, Y 36.0, Z 20.0 to 21.0), so the two were not measured apart');
  Assert.equal(Refused.Refusal.Advice, 'Move the can 2 to 3 cm away from what stands beside it, then send the task again.');
  Assert.equal(Api.SceneConflict(null), null);
  Assert.equal(Api.SceneConflict({ ...Scene, task_object: undefined }), null);
});

Test("the scene's known space counts as known for the gripper's way, where observed free space alone does not reach", () => {
  // The arm where it stood for the capture, and with the task's reach switched on its unobserved surroundings
  // (back end/SceneGeometry.py ArmSpace, TaskReach): what the camera saw through, plus what it saw the arm in.
  const Angles = { ...Api.currentAngles(Home), gripper: 10 };
  const Strict = { ...Floor, Scene: { ...Scene, free_space: [Box([60, 60, 60], [61, 61, 61])] } };
  Assert.match(Api.ScenePathCoverageReason(Angles, Angles, Strict) ?? '', /not observed/);
  const Known = { ...Strict, Scene: { ...Strict.Scene, known_space: [Box([-80, -40, -30], [80, 100, 80])] } };
  Assert.equal(Api.ScenePathCoverageReason(Angles, Angles, Known), null);
});

Test("the bridge's measured contact decides the conflict; the boxes only for a scene without one", () => {
  // back end/SceneGeometry.py TaskContact: points (not cells) against the object's measured footprint, or null. The 1 cm
  // cells met the pill bottle's own rim on the cabinet of 2026-10-06, and still refused it moved clear of the books.
  const Pad = Api.SceneRules.DepthUncertaintyCm;
  const Task = Box([-16.4, 30.7, 12.7], [-10.4, 36.3, 24.7]);
  const Merged = { label: 'observed environment', ...Box([-12 - Pad, 35 - Pad, 20 - Pad], [-10 + Pad, 37 + Pad, 21 + Pad]) };
  const With = (Obstacles, Contact) => ({ ...Scene, task_object: Task, obstacles: Obstacles, ...(Contact !== undefined ? { task_contact: Contact } : {}) });
  Assert.equal(Api.SceneConflict(With([Merged], null)), null);
  Assert.equal(Api.SceneConflict(With([Merged])).Box, Merged, 'an older bridge: the boxes');
  const Contact = { points: 334, min: [-29.9, 43.08, 10.32], max: [-26.2, 43.58, 13.43] };
  const Found = Api.SceneConflict(With([], Contact));
  Assert.deepEqual([Found.Low, Found.High, Found.Box.label], [Contact.min, Contact.max, 'observed environment']);
  const Refused = Api.PlanGrasp(Object_, -30, { ...Context, Floor: { ...Floor, Scene: With([Scene.obstacles[0]], Contact) } });
  Assert.equal(Refused.Plan, null);
  Assert.equal(Refused.Refusal.Text, 'the camera measured the can running into observed environment beside it (at X -28.0, Y 43.3, Z 10.3 to 13.4), so the two were not measured apart');
});

Test("lifted off its support, the object meets a cell of the support's level only where it was observed, a neighbour as before", () => {
  // A cell at the support's level beside the object (a tilted support's rise, its edge: the cabinet top's front edge
  // 1 cm under the pill bottle's base, 2026-10-06 15:01) is where the object stands: its padding is not a solid.
  Assert.ok(Plan);
  const Lift = Api.solveIkPreview({ ...Api.modelPointFromDisplayPoint(Plan.Approach.Release), wristPitch: Plan.Pitch }, 'up', Home).angles;
  const Pad = Api.SceneRules.DepthUncertaintyCm, Base = Plan.Prism.BaseZ;
  const Right = Math.max(...Plan.Prism.Footprint.map(P => P.x)), Y0 = Math.min(...Plan.Prism.Footprint.map(P => P.y));
  const Cell = (Z0, Z1) => [{ label: 'observed environment', ...Box([Right + 0.2 - Pad, Y0 - Pad, Z0 - Pad], [Right + 1.2 + Pad, Y0 + 1 + Pad, Z1 + Pad]) }];
  const Held = Cells => Api.CarriedObjectPathReason(Plan.Prism, Plan.Poses.Grasp, Plan.Poses.Grasp, Lift, Api.ScenePrisms({ ...Floor, Scene: { ...Scene, obstacles: Cells } }));
  Assert.equal(Held(Cell(Base - 1.2, Base - 0.2)), null, 'the support edge beside it, its padding over the base');
  Assert.equal(Held(Cell(Base - 0.5, Base + 0.8)), null, "a tilted support's rise beside it, observed clear of it");
  Assert.match(Held(Cell(Base + 3, Base + 5)) ?? '', /held can would touch observed environment at 0% of the way there/);
  // Observed inside the object's own box it touches whatever its level.
  const Inside = [{ label: 'observed environment', ...Box([Right - 1 - Pad, Y0 + 1 - Pad, Base - 0.6 - Pad], [Right - Pad + 0.1, Y0 + 2 + Pad, Base + 0.6 + Pad]) }];
  Assert.match(Api.CarriedObjectPathReason(Plan.Prism, Plan.Poses.Grasp, Plan.Poses.Grasp, Lift, Api.ScenePrisms({ ...Floor, Scene: { ...Scene, obstacles: Inside } })) ?? '', /would touch/);
});
