import { build as Build } from 'esbuild';
import { readFileSync as ReadFile } from 'node:fs';
import { createRequire as CreateRequire } from 'node:module';
import { test as Test } from 'node:test';
import Assert from 'node:assert/strict';
import Path from 'node:path';
import { fileURLToPath as FilePath } from 'node:url';

// The best-effort GRASP PLAN (src/lib/GraspPlan.ts GraspRules.BestEffort) after the 2026-10-07 audit:
// M3, the plan is the same however busy the PC is (no wall-clock budget; too long only refuses), and
// M9, its warnings take the task object out of the guard's path check only on the plan's own descent.
// TestGraspPlan.mjs's "tin" (6.0 x 5.5 cm, planned as measured, the grip may slip). No bridge, no hardware.
const Root = Path.dirname(FilePath(import.meta.url));
const Bundle = await Build({ stdin: { contents: ReadFile(Path.join(Root, 'src/App.tsx'), 'utf8')
  + '\nexport * from "./lib/GraspPlan";'
  + '\nexport { ReachScan, TaskObjectReachability } from "./lib/AiMeasurements";'
  + '\nexport { CadForward } from "./lib/CadKinematics";'
  + '\nexport { CloseOn } from "./lib/JawSweep";'
  + '\nexport { INITIAL_SERVOS, angleFromRawValue, fractionToRaw, mirrorDisplayPoint, modelPointFromDisplayPoint, solveIkPreview } from "./lib/robotModel";',
  resolveDir: Path.join(Root, 'src'), loader: 'tsx' }, bundle: true, platform: 'node', format: 'cjs', write: false, loader: { '.css': 'empty' }, logLevel: 'error' });
globalThis.window ??= { location: { hostname: 'localhost' } };
const AppModule = { exports: {} };
new Function('require', 'module', 'exports', Bundle.outputFiles[0].text)(CreateRequire(import.meta.url), AppModule, AppModule.exports);
const Api = AppModule.exports;
const Rules = Api.GraspRules;

const Servos = structuredClone(Api.INITIAL_SERVOS);
Object.assign(Servos.gripper, { presentRaw: 1922, targetRaw: 1922, guiAngleRaw: null });
const Context = { Servos, ElbowMode: 'up', Others: [], Scan: Point => Api.ReachScan(Point, { Servos, ElbowMode: 'up' }).Pitch };
const Tin = { label: 'tin', x: -5.44, y: 36.39, z: -1.97, topZ: 10.16, heightCm: 12.13, sizeCm: [6.0, 5.5], depthMeasured: true, extrapolated: false,
  unstable: false, supportMeasured: true, supportZMeasured: -1.77, validFraction: 0.9, supportBranch: 'desk', taskMatch: true, taskQueryOnly: false,
  refined: true, outsideCalibrationCm: 0 };
const Reach = () => Api.TaskObjectReachability(Tin, Context, { Others: [] });
// The page clock running Speed times fast: a PC Speed times slower, as far as any wall-clock budget can tell.
function WithClock(Speed, Make) {
  const RealNow = performance.now.bind(performance), Origin = RealNow();
  performance.now = () => Origin + (RealNow() - Origin) * Speed;
  try { return Make(); } finally { performance.now = RealNow; }
}
const Verdict = Result => JSON.stringify([Result.Kind, Result.Plan?.Grasp ?? null, Result.Plan?.Pitch ?? null, Result.Plan?.Warnings ?? null]);

const Idle = Reach();
const Plan = Idle.Plan;

Test('the tin gets a best-effort plan, warned', () => {
  Assert.equal(Idle.Kind, 'graspable', Idle.Text);
  Assert.ok(Plan.Warnings.length > 0, 'a best-effort plan');
  Assert.match(Plan.Warnings[0], /^planned as measured \(6\.0 x 5\.5 cm\)/);
});

Test('the best-effort plan is the same on a PC 8 times slower (audit M3: no wall-clock budget)', () => {
  // Before the fix a 6 s budget cut the search: at 4x the tin planned at wrist -44 with four other warnings.
  Assert.equal(Verdict(WithClock(8, Reach)), Verdict(Idle));
});

Test('a search that runs too long refuses, never a plan from part of the search (audit M3)', () => {
  const Saved = Rules.BestEffortRefuseMs;
  Rules.BestEffortRefuseMs = 1;
  try {
    const Slow = WithClock(1000, Reach);
    // The strict passes' own refusal stands; nothing best-effort, nothing else picked.
    Assert.notEqual(Slow.Kind, 'graspable', Slow.Text);
  } finally { Rules.BestEffortRefuseMs = Saved; }
  // And the cap only ever refuses: with it back, the same plan as before.
  Assert.equal(Verdict(Reach()), Verdict(Idle));
});

Test('the best-effort work is counted in placements, never the clock: the tin\'s whole search fits, a measured scene gets fewer (audit M3)', () => {
  // A measured scene's obstacle boxes are neighbours: the 1201 of the 2026-10-06 cabinet made a placement 4 to 7 times a
  // desk one, and the old 6 s covered 35 placements there; a desk object gets BestEffortWork.
  Assert.equal(Api.BestEffortPlacements({ Others: [] }), Rules.BestEffortWork);
  Assert.equal(Api.BestEffortPlacements({ Others: Array.from({ length: 1201 }, () => ({})) }), 37);
  Assert.equal(Api.BestEffortPlacements({ Others: Array.from({ length: 100000 }, () => ({})) }), Rules.BestEffortMinPlacements);
  // The tin's whole search (96 placements) fits in the desk's work: with no bound at all, the same plan.
  const Saved = Rules.BestEffortWork;
  Rules.BestEffortWork = 100000;
  try { Assert.equal(Verdict(Reach()), Verdict(Idle)); } finally { Rules.BestEffortWork = Saved; }
  // And a budget too small for the shape that plans it (the "as measured" box, tried second) refuses: never a pick
  // from beyond the bound, and nothing the old 6 s on an idle PC would not have planned.
  Rules.BestEffortWork = 40;
  try {
    const Short = Reach();
    Assert.ok(Short.Kind !== 'graspable' || !/planned as measured/.test(Short.Plan.Warnings[0] ?? ''), Short.Text);
  } finally { Rules.BestEffortWork = Saved; }
});

Test('the work counts each placement\'s IK solves and closing sweep, never the clock (2026-10-08 review: a round neighbour made 150 placements take 60 s)', () => {
  // One solve is one unit; the closing sweep's vertex-edge pairs count BestEffortSweepPairsPerSolve to a unit.
  const Worked = Api.BestEffortWorkDone();
  Assert.equal(Worked(), 0);
  Api.solveIkPreview({ ...Api.modelPointFromDisplayPoint({ x: 4, y: 32, z: 1 }), wristPitch: -60 }, 'up', Servos);
  Assert.equal(Worked(), 1);
  const Disc = Array.from({ length: 32 }, (_, Index) => ({ x: 1.5 + 2.5 * Math.cos(Index * Math.PI / 16), z: -6 + 2.5 * Math.sin(Index * Math.PI / 16) }));
  Api.CloseOn(Disc, 30);
  const AfterSweep = Worked();
  Assert.ok(AfterSweep > 1 && AfterSweep < 2, `a closing sweep on a 32-sided disc is a fraction of a solve: ${AfterSweep}`);
  // The tin's whole search (1108 solves) fits in the work: with no bound at all, the same plan; with a bound too small
  // for the shape that plans it (the "as measured" box, tried second, found after about 970 solves), it is not planned.
  const Saved = Rules.BestEffortSolveWork;
  Rules.BestEffortSolveWork = 1e9;
  try { Assert.equal(Verdict(Reach()), Verdict(Idle)); } finally { Rules.BestEffortSolveWork = Saved; }
  Rules.BestEffortSolveWork = 400;
  try {
    const Short = Reach();
    Assert.ok(Short.Kind !== 'graspable' || !/planned as measured/.test(Short.Plan.Warnings[0] ?? ''), Short.Text);
  } finally { Rules.BestEffortSolveWork = Saved; }
});

// The guard as the page builds it for the tin's plan (BuildGraspGuard), the tin its only prism.
const TaskPrism = { ...Api.ObjectPrism(Tin), Role: 'task' };
const Guard = Warnings => ({ Prisms: [TaskPrism], TaskPrism, Plan: { ...Plan, Warnings }, Refusal: null, Held: false, Intent: 'grasp', TaskLabel: 'tin', Step: null });
const OpenDeg = Api.angleFromRawValue(Servos.gripper, Api.fractionToRaw(Servos.gripper, Plan.OpenFraction));
const Pose = (Display, Pitch) => {
  const Model = { ...Api.modelPointFromDisplayPoint(Display), wristPitch: Pitch };
  const Preview = Api.solveIkPreview(Model, 'up', Servos);
  Assert.ok(Preview.reachable, `IK reaches ${JSON.stringify(Display)} at ${Pitch}`);
  return { Model, Angles: { ...Preview.angles, gripper: OpenDeg } };
};
const Move = (From, To, Warnings) => Api.GraspGuardReason(To.Model, To.Angles, From.Angles,
  Api.mirrorDisplayPoint(To.Model).z - Api.mirrorDisplayPoint(From.Model).z, Guard(Warnings));

Test("a best-effort plan's warnings no longer switch off the tin's path check (audit M9)", () => {
  // The audit's move: sideways at Z 4 from 9 cm one side of the tin to 9 cm the other, the wrist as planned.
  const From = Pose({ x: Tin.x + 9, y: Tin.y, z: 4 }, Plan.Pitch), To = Pose({ x: Tin.x - 9, y: Tin.y, z: 4 }, Plan.Pitch);
  for (const Warnings of [Plan.Warnings, undefined]) {
    const Reason = Move(From, To, Warnings);
    Assert.match(Reason ?? 'ALLOWED', /FIXED finger .* into the tin/, `${Warnings ? 'with' : 'without'} warnings`);
  }
});

Test("the plan's own descent, hover point to grasp point, still runs with its warnings (audit M9)", () => {
  const Hover = Pose(Plan.Hover, Plan.HoverPitch), Grasp = Pose(Plan.Grasp, Plan.Pitch);
  Assert.equal(Move(Hover, Grasp, Plan.Warnings), null);
  // Stopped half a centimetre off the hover point (the arm stops within ArrivalCm of a commanded point): still its descent.
  const Off = Pose({ ...Plan.Hover, x: Plan.Hover.x + 0.5 }, Plan.HoverPitch);
  Assert.equal(Move(Off, Grasp, Plan.Warnings), null);
  Assert.equal(Api.OnPlannedDescent(Guard(Plan.Warnings), Api.mirrorDisplayPoint(Off.Model), Plan.Grasp, Plan.Pitch), true);
  // Off the descent's line, or at another wrist, it is not: the tin is checked there.
  Assert.equal(Api.OnPlannedDescent(Guard(Plan.Warnings), { ...Plan.Hover, x: Plan.Hover.x + 6 }, Plan.Grasp, Plan.Pitch), false);
  Assert.equal(Api.OnPlannedDescent(Guard(Plan.Warnings), Plan.Hover, { ...Plan.Grasp, x: Plan.Grasp.x - 1 }, Plan.Pitch), false);
  Assert.equal(Api.OnPlannedDescent(Guard(Plan.Warnings), Plan.Hover, Plan.Grasp, Plan.Pitch + 20), false);
  // No warnings, held, or not a grasp: never (those keep their own rules).
  Assert.equal(Api.OnPlannedDescent(Guard(undefined), Plan.Hover, Plan.Grasp, Plan.Pitch), false);
  Assert.equal(Api.OnPlannedDescent({ ...Guard(Plan.Warnings), Held: true }, Plan.Hover, Plan.Grasp, Plan.Pitch), false);
});

Test("the snap's pose check keeps the tin off the descent and leaves it out only on it (audit M9)", () => {
  const Check = Api.GuardPoseCheck(Guard(Plan.Warnings), Pose(Plan.Hover, Plan.HoverPitch).Angles);
  const LinksAt = Target => Api.CadForward(Api.solveIkPreview(Target, 'up', Servos).angles).Links;
  const GraspTarget = { ...Api.modelPointFromDisplayPoint(Plan.Grasp), wristPitch: Plan.Pitch };
  Assert.equal(Check(GraspTarget, LinksAt(GraspTarget)), true, 'the plan\'s grasp pose');
  // Inside the tin, beside the descent: refused.
  const Inside = { ...Api.modelPointFromDisplayPoint({ x: Tin.x, y: Tin.y, z: 4 }), wristPitch: Plan.Pitch };
  Assert.equal(Check(Inside, LinksAt(Inside)), false, 'a pose inside the tin');
});
