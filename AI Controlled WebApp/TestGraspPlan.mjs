import { build as Build } from 'esbuild';
import { readFileSync as ReadFile } from 'node:fs';
import { createRequire as CreateRequire } from 'node:module';
import { test as Test } from 'node:test';
import Assert from 'node:assert/strict';
import Path from 'node:path';
import { fileURLToPath as FilePath } from 'node:url';
import { PinnedHome } from './UiTestLib.mjs';

// The fixed-jaw GRASP PLAN (src/lib/GraspPlan.ts): the tip is the inner pad
// of the FIXED jaw, so a grasp puts it beside the object, never over it; the
// guard holds every AI tip motion of a task to the gripper's real parts; and
// the model is told the plan step by step. App.tsx is bundled with the libs
// (one module instance), its solveIkPreview counted. No bridge, no hardware.
const Root = Path.dirname(FilePath(import.meta.url));
const CountSolves = { name: 'count-solves', setup(Builder) {
  Builder.onLoad({ filter: /robotModel\.ts$/ }, Args => {
    const Source = ReadFile(Args.path, 'utf8');
    if (!Source.includes('export function solveIkPreview(')) throw new Error('robotModel.ts no longer exports solveIkPreview');
    return { loader: 'ts', contents: Source.replace('export function solveIkPreview(', 'function CountedSolveIkPreview(')
      + '\nexport const SolveCounter = { Count: 0 };\nexport function solveIkPreview(...Args: Parameters<typeof CountedSolveIkPreview>) { SolveCounter.Count += 1; return CountedSolveIkPreview(...Args); }\n' };
  });
} };
const Bundle = await Build({ stdin: { contents: ReadFile(Path.join(Root, 'src/App.tsx'), 'utf8')
  + '\nexport { parseAiResponse as ParseResponse, validateAiMotionProposal as Validate, buildAiSystemPrompt as AiPrompt };'
  + '\nexport * from "./lib/GraspPlan";'
  + '\nexport * as Sweep from "./lib/JawSweep";'
  + '\nexport { BuildGraspGuard, MeasuredObjectsSummary, ReachScan, TaskObjectLine, TaskObjectReachability, TaskIntent, ParseTaskObject,'
  + ' ClosedJawDescentReason, UnreliableTargetReason, ReadLocateResponse, TapeCorrectedReach, GripperClosedAfter, HeldFraction, CorrectedHover, PanArrivalCm,'
  + ' NextStepOf } from "./lib/AiMeasurements";'
  + '\nexport { NoOpReason, NoOpFeedbackPrompt, NoOpCommandedCm, NoOpDeg } from "./lib/AiTaskLoop";'
  + '\nexport { SnapAiTarget, SnapRules } from "./lib/AiTargetSnap";'
  + '\nexport { AiJawFloorReason, AiTableFloorReason, GripperLowestZOfLinks } from "./lib/AiTableFloor";'
  + '\nexport { BasketPoseReason, BasketTargetReason } from "./lib/BasketCollision";'
  + '\nexport { CadForward } from "./lib/CadKinematics";'
  + '\nexport { SolveCounter, INITIAL_SERVOS, IK_JOINTS, angleFromRawValue, currentAngles, forwardKinematics, fractionToRaw, mirrorDisplayPoint,'
  + ' modelPointFromDisplayPoint, rawFromAngle, rawToFraction, solveIkPreview, wristPitchFromAngles } from "./lib/robotModel";',
  resolveDir: Path.join(Root, 'src'), loader: 'tsx' }, bundle: true, platform: 'node', format: 'cjs', write: false, loader: { '.css': 'empty' },
  plugins: [CountSolves, PinnedHome], logLevel: 'error' });
globalThis.window ??= { location: { hostname: 'localhost' } };
const AppModule = { exports: {} };
new Function('require', 'module', 'exports', Bundle.outputFiles[0].text)(CreateRequire(import.meta.url), AppModule, AppModule.exports);
const Api = AppModule.exports;
const Rules = Api.GraspRules;

// --- Fixtures ------------------------------------------------------------------------

// Home with the gripper open (raw 1922, the 80% open the steps use).
const Snapshot = (Raw, Base = Api.INITIAL_SERVOS) => {
  const Servos = structuredClone(Base);
  for (const [Name, Value] of Object.entries(Raw)) Object.assign(Servos[Name], { presentRaw: Value, targetRaw: Value, guiAngleRaw: null });
  return Servos;
};
const Home = Snapshot({ gripper: 1922 });
// 2026-09-30 23:50: the hover servos the can descent was proposed from.
const S2350 = Snapshot({ shoulder_pan: 3305, shoulder_lift: 732, elbow_flex: 931, wrist_flex: 1312, wrist_roll: 1521, gripper: 1917 });
// The servos at an IK pose (the gripper and roll kept).
const At = (Servos, Angles) => {
  const Copy = structuredClone(Servos);
  for (const Joint of Api.IK_JOINTS) Object.assign(Copy[Joint], { presentRaw: Api.rawFromAngle(Copy[Joint], Angles[Joint]), targetRaw: Api.rawFromAngle(Copy[Joint], Angles[Joint]) });
  return Copy;
};
const WithGripper = (Servos, Fraction) => { const Copy = structuredClone(Servos); const Raw = Api.fractionToRaw(Copy.gripper, Fraction); Object.assign(Copy.gripper, { presentRaw: Raw, targetRaw: Raw }); return Copy; };
const TipOf = Servos => Api.mirrorDisplayPoint(Api.forwardKinematics(Api.currentAngles(Servos)).at(-1));
const Model = (Point, Pitch) => ({ ...Api.modelPointFromDisplayPoint(Point), wristPitch: Pitch });
const Solve = (Point, Pitch, Servos) => Api.solveIkPreview(Model(Point, Pitch), 'up', Servos);
// How busy the PC is right now, as the cost of one IK solve (the median of five batches), for the three time bounds below,
// which back up counted costs (IK solves) and were set on this PC idle (about 2.5 ms a solve, 2026-10-07): they scale with
// it, as TestAiTargetSnap's do (audit 1 L5), so a loaded PC that slows every step alike does not fail them. The counts stay exact.
function LoadScale(IdleMsPerSolve = 2.5) {
  const Point = { x: 4, y: 32, z: 1 }, Batches = Array.from({ length: 5 }, () => {
    const Started = performance.now();
    for (let Index = 0; Index < 40; Index += 1) Solve(Point, -60, Api.INITIAL_SERVOS);
    return (performance.now() - Started) / 40;
  }).sort((A, B) => A - B);
  return Math.max(1, Batches[2] / IdleMsPerSolve);
}
const Obj = (label, x, y, z, top, size, Extra = {}) => ({ label, x, y, z, topZ: top, heightCm: top - z, sizeCm: size, depthMeasured: true, extrapolated: false,
  unstable: false, supportMeasured: true, supportZMeasured: z, validFraction: 0.9, supportBranch: 'desk', taskMatch: true, taskQueryOnly: false, refined: true,
  outsideCalibrationCm: 0, ...Extra });
const Usb = Obj('usb drive', 4.3, 32.9, -2.5, -0.4, [1.7, 1.8]);
const Can2 = Obj('can', -5.44, 36.39, -1.97, 10.16, [5.0, 4.5], { supportZMeasured: -1.77 });
const Can3 = Obj('can', -4.54, 34.34, -1.85, 10.08, [4.3, 2.3], { supportZMeasured: -1.67, validFraction: 0.608 });
const Spray = Obj('spray can', -4.5, 34.35, -1.84, 10.06, [4.9, 2.2], { supportZMeasured: -1.85 });
const Bottle = Obj('bottle', 16.09, 32.16, -2.33, 5.56, [3.6, 2.7], { supportZMeasured: -1.96 });
const Eraser25 = Obj('eraser', -5, 35, -2.3, -0.3, [2, 5]);
const Eraser12 = Obj('eraser', -5, 33, -2.3, -1.1, [2, 5]);
const Pen = Obj('pen', -5, 33, -2.3, -1.3, [1, 12]);
const Stick = Obj('stick', -5, 30, -2.3, 12.7, [1.5, 1.5]);
const Tape = Obj('tape', 7.62, 33.34, -2.14, 4.71, [1.1, 1.5], { supportZMeasured: -1.66 });
const Boxes = [[-10, 32], [-15, 42], [-10, 42], [0, 34], [0, 40], [-15, 30]].map(([X, Y]) => Obj('box', X, Y, -2.3, -0.3, [2, 2]));
const Context = (Servos = Home) => ({ Servos, ElbowMode: 'up', Others: [], Scan: Point => Api.ReachScan(Point, { Servos, ElbowMode: 'up' }).Pitch });
const Reach = (Object_, Others = [], Servos = Home) => Api.TaskObjectReachability(Object_, Context(Servos), { Others });
// The two passes before the best-effort one (GraspRules.BestEffort false): their refusals, as the tests of them were
// written. Since 2026-10-04 ("just let it try unless your code is totally sure it can't grab it") what only predicts a
// failed grip is planned anyway, warned (TestGraspReachRefusal.mjs).
const Strictly = Make => { const Saved = Rules.BestEffort; Rules.BestEffort = false; try { return Make(); } finally { Rules.BestEffort = Saved; } };
// With the fixed jaw's extra standoff (GraspRules.FixedJawExtraStandoffCm) set to Cm: 0, the gap as before 2026-10-25.
const WithExtra = (Cm, Make) => { const Saved = Rules.FixedJawExtraStandoffCm; Rules.FixedJawExtraStandoffCm = Cm; try { return Make(); } finally { Rules.FixedJawExtraStandoffCm = Saved; } };
const Planned = [['usb', Usb], ['eraser [2,5]', Eraser25], ['eraser 1.2', Eraser12], ['pen', Pen], ...Boxes.map(Box => [`box at (${Box.x}, ${Box.y})`, Box]),
  ['stick', Stick], ['tape', Tape]];
// Plans are deterministic: each object is planned once.
const Plans = new Map(Planned.map(([Name, Object_]) => [Name, Reach(Object_)]));
const UsbReach = Plans.get('usb'), UsbPlan = UsbReach.Plan;
const OpenDeg = Api.angleFromRawValue(Home.gripper, 1922);
const JawPair = Deg => [Deg, Math.max(Deg - Rules.JawUncertaintyDeg, Rules.PadsMeetDeg)];
// The pose the printed grasp point is reached with, the jaw open.
const GraspAngles = (Plan, Servos = Home) => ({ ...Solve(Plan.Grasp, Plan.Pitch, Servos).angles, gripper: OpenDeg });
const Dot = (A, B) => A.x * B.x + A.y * B.y;
// The footprint clipped to the finger's band across it (independent of the planner's own clip).
function Band(Footprint, Vh, Lo, Hi) {
  const Clip = (Points, Keep) => Points.flatMap((P, I) => {
    const Q = Points[(I + 1) % Points.length], Kp = Keep(P), Kq = Keep(Q);
    return [...(Kp >= 0 ? [P] : []), ...((Kp >= 0) !== (Kq >= 0) ? [{ x: P.x + Kp / (Kp - Kq) * (Q.x - P.x), y: P.y + Kp / (Kp - Kq) * (Q.y - P.y) }] : [])];
  });
  return Clip(Clip(Footprint, P => Dot(P, Vh) - Lo), P => Hi - Dot(P, Vh));
}
const PadCorners = [[-0.79, -0.53, -10.35], [-0.79, 0.49, -10.35], [-0.79, -0.53, -9.45], [-0.79, 0.49, -9.45]];
const PadZ = Angles => {
  const E = Api.CadForward(Angles).Links.gripper_link.elements;
  return PadCorners.map(([X, Y, Z]) => Api.mirrorDisplayPoint({ x: E[0] * X + E[4] * Y + E[8] * Z + E[12], y: E[1] * X + E[5] * Y + E[9] * Z + E[13], z: E[2] * X + E[6] * Y + E[10] * Z + E[14] }).z);
};
const State = (Object_, Reachability, Extra = {}) => ({ Label: Object_.label, DetectedNow: true, Binding: { Object: Object_, Source: 'name', DistanceCm: null },
  LastSeen: { At: 0, Object: Object_, Reachability }, Reachability, Intent: 'grasp',
  Plan: Reachability.Plan ? { Reach: Reachability, Source: Object_, At: 0, Key: Api.GraspPlanKey(Home, 'up'), Note: null } : null, ...Extra });
// The first motion_proposal JSON object in a line, by matching braces.
const MotionIn = Line => {
  const Start = Line.indexOf('"motion_proposal":') + '"motion_proposal":'.length;
  Assert.ok(Start > 17, `no motion_proposal in ${Line}`);
  let Depth = 0;
  for (let Index = Start; Index < Line.length; Index++) {
    Depth += Line[Index] === '{' ? 1 : Line[Index] === '}' ? -1 : 0;
    if (Depth === 0) return JSON.parse(Line.slice(Start, Index + 1));
  }
  throw new Error(`unterminated motion_proposal in ${Line}`);
};
const Guard = (Prisms, Extra = {}) => ({ Prisms, TaskPrism: Prisms.find(P => P.Role === 'task') ?? null, Plan: null, Refusal: null, Held: false, Intent: 'grasp',
  TaskLabel: 'can', ...Extra });
const AsTask = Object_ => ({ ...Api.ObjectPrism(Object_), Role: 'task' });

// --- 1. The asset ---------------------------------------------------------------------

Test('GripperParts covers the CAD gripper: the fixed pad face, the cover radius, a small sample set, the same GLB as GripperHull', () => {
  const Parts = JSON.parse(ReadFile(Path.join(Root, 'src/lib/GripperParts.json'), 'utf8'));
  const Hull = JSON.parse(ReadFile(Path.join(Root, 'src/lib/GripperHull.json'), 'utf8'));
  const Pad = Parts.Parts.FixedFinger.Samples.filter(([, , Z]) => Z >= -10.36 && Z <= -9.44);
  Assert.ok(Pad.length > 5);
  Assert.ok(Math.abs(Math.max(...Pad.map(([X]) => X)) + 0.79) <= 0.02, `finger face at x ${Math.max(...Pad.map(([X]) => X))}`);
  for (const [Name, Part] of Object.entries(Parts.Parts)) {
    Assert.ok(Part.CoverRadiusCm + Part.DecimationLossCm <= Rules.ContactMarginCm, `${Name}: ${Part.CoverRadiusCm} + ${Part.DecimationLossCm}`);
    Assert.ok(Part.Samples.length <= 1000, `${Name}: ${Part.Samples.length} samples`);
  }
  Assert.deepEqual(Object.keys(Parts.Parts).sort(), ['FixedFinger', 'MovingJaw', 'PalmAndServo']);
  Assert.equal(Parts.SourceSha256, Hull.SourceSha256);
});

// --- 2-4. Where the plan puts the fixed jaw -------------------------------------------

Test('the fixed jaw goes beside the object, never over it: c outside its padded box, centred on it, at a fixed point of the planner', () => {
  for (const [Name, Object_] of Planned) {
    const Plan = Plans.get(Name).Plan;
    Assert.ok(Plan, `${Name}: ${Plans.get(Name).Text}`);
    const Prism = Plan.Prism, Angles = GraspAngles(Plan);
    const Finger = Api.GripperSamples(Angles, []).filter(Sample => Sample.Part === 'FixedFinger');
    Assert.equal(Api.DeepestContact(Finger, [Prism], 0.5), null, `${Name}: the fixed finger at the printed grasp`);
    const Uh = Plan.JawAxis, Vh = { x: -Uh.y, y: Uh.x };
    const Across = Prism.Footprint.map(P => Dot(P, Vh)), Cv = (Math.min(...Across) + Math.max(...Across)) / 2;
    const Near = Math.min(...Band(Prism.Footprint, Vh, Cv - Rules.FingerBandHalfCm, Cv + Rules.FingerBandHalfCm).map(P => Dot(P, Uh)));
    Assert.ok(Near - Dot(Plan.Grasp, Uh) >= Api.PlanClearanceCm(Plan) - 0.08, `${Name}: pad plane ${(Near - Dot(Plan.Grasp, Uh)).toFixed(3)} cm outside`);
    // (A sweep plan's head start along the fingers, Closing.DepthOffsetCm, moves it across the band: along u only, below.)
    const Swept = Plan.Closing.Mode === 'sweep';
    if (!Swept) Assert.ok(Math.abs(Dot(Plan.Grasp, Vh) - Cv) <= 0.08, `${Name}: ${(Dot(Plan.Grasp, Vh) - Cv).toFixed(3)} cm off the band's centre`);
    // A fixed point: the formula at the unrounded T gives T again. (Placed with the fixed jaw's extra standoff the plan was
    // made with, GraspRules.FixedJawExtraStandoffCm or the smaller one the object fitted: c is the plan's ClearanceCm.)
    const Placed = Api.PlaceAtPitch(Prism, Plan.Pitch, { ...Context(), ExtraStandoffCm: Plan.ExtraStandoffCm ?? 0 }, Plan.Closing.Mode === 'sweep');
    Assert.ok(Placed.Ok, Name);
    const Frame = Api.JawFrame({ ...Api.solveIkPreview(Model(Placed.T, Plan.Pitch), 'up', Home).angles, gripper: OpenDeg });
    const N = Math.hypot(Frame.U.x, Frame.U.y), U2 = { x: Frame.U.x / N, y: Frame.U.y / N }, V2 = { x: -U2.y, y: U2.x };
    const Across2 = Prism.Footprint.map(P => Dot(P, V2)), Cv2 = (Math.min(...Across2) + Math.max(...Across2)) / 2;
    const Near2 = Math.min(...Band(Prism.Footprint, V2, Cv2 - 0.75, Cv2 + 0.75).map(P => Dot(P, U2)));
    const ToolU = Dot(Frame.Tool, U2);
    const Delta = Math.max(0, ...Api.GripperSamples({ ...Api.solveIkPreview(Model(Placed.T, Plan.Pitch), 'up', Home).angles, gripper: OpenDeg }, [])
      .filter(S => S.Part === 'FixedFinger' && S.P.z >= Prism.BaseZ - 0.5 && S.P.z <= Prism.TopZ + 0.5).map(S => Dot(S.P, U2) - ToolU));
    const Along = Near2 - Api.PlanClearanceCm(Plan) - Delta;
    // (A sweep plan's head start: out along the level part of the approach a, DepthOffsetCm of it along the fingers.)
    const Level = Math.hypot(Frame.A.x, Frame.A.y), Head = Swept ? (Placed.Info.DepthOffsetCm ?? 0) / Level / Level : 0;
    const Again = { x: Along * U2.x + Cv2 * V2.x + Head * Frame.A.x, y: Along * U2.y + Cv2 * V2.y + Head * Frame.A.y };
    const Moves = Swept ? Math.abs(Dot(Again, U2) - Dot(Placed.T, U2)) : Math.hypot(Again.x - Placed.T.x, Again.y - Placed.T.y);
    Assert.ok(Moves < 0.02, `${Name}: one more iteration moves T ${Moves.toFixed(3)} cm`);
    Assert.ok(Math.hypot(Placed.T.x - Plan.Grasp.x, Placed.T.y - Plan.Grasp.y) <= 0.08, `${Name}: printed ${JSON.stringify(Plan.Grasp)}`);
  }
});

Test('the jaw frame: -u points from the object to the fixed jaw, turns with the pan, and names the side', () => {
  const Mid = (Can2.z + Can2.topZ) / 2;
  const Frame = Api.JawFrame({ ...Solve({ x: Can2.x, y: Can2.y, z: Mid }, -45, Home).angles, gripper: OpenDeg });
  const N = Math.hypot(Frame.U.x, Frame.U.y);
  Assert.ok(Math.abs(-Frame.U.x / N - 0.995) <= 0.02 && Math.abs(-Frame.U.y / N + 0.104) <= 0.02, JSON.stringify(Frame.U));
  Assert.ok(Math.abs(-UsbPlan.JawAxis.x - 0.90) <= 0.03 && Math.abs(-UsbPlan.JawAxis.y + 0.43) <= 0.03, JSON.stringify(UsbPlan.JawAxis));
  Assert.equal(UsbPlan.Side, 'left-near');
  Assert.ok(UsbPlan.Grasp.x > Usb.x, 'The fixed jaw is on the +X (image-left) side of an object ahead of the arm');
  // The verified USB plan (spec 1.6).
  Assert.deepEqual([UsbPlan.Pitch, UsbPlan.HoverPitch, UsbPlan.Grasp, UsbPlan.Hover], [-52.8, -49.7, { x: 6.9, y: 31.7, z: -1.4 }, { x: 6.9, y: 31.7, z: 4.6 }]);
  Assert.ok(Math.abs(UsbPlan.OffsetCm - 2.86) <= 0.02 && Math.abs(UsbPlan.WidthEffCm - 4.69) <= 0.02);
});

Test('the grasp height: mid-height, raised for the closing jaws over the desk, or under the top where the moving jaw meets the side', () => {
  Assert.equal(UsbPlan.HeightBy, 'mid');
  Assert.ok(Math.abs(UsbPlan.Grasp.z + 1.45) <= 0.1);
  const Eraser = Plans.get('eraser 1.2').Plan;
  Assert.equal(Eraser.HeightBy, 'floor');
  const Angles = GraspAngles(Eraser);
  const Lowest = Deg => Api.GripperLowestZOfLinks(Api.CadForward({ ...Angles, gripper: Deg }).Links);
  const Sweep = Array.from({ length: 16 }, (_, I) => Rules.PadsMeetDeg + (OpenDeg - Rules.PadsMeetDeg) * I / 15);
  Assert.ok(Math.min(...Sweep.map(Lowest)) >= Eraser12.supportZMeasured - 0.01, `closing: ${Math.min(...Sweep.map(Lowest))}`);
  Assert.ok(Lowest(OpenDeg) >= Eraser12.supportZMeasured + 0.19, `open: ${Lowest(OpenDeg)}`);
  const StickPlan = Plans.get('stick').Plan;
  Assert.equal(StickPlan.HeightBy, 'jaw');
  Assert.ok(Api.JawSliceTopCm(StickPlan.Prism, StickPlan.Grasp, GraspAngles(StickPlan)) <= Api.MaxTopAboveTool(StickPlan.WidthEffCm) + 0.05);
  for (const [Name] of Planned) {
    const Plan = Plans.get(Name).Plan, Prism = Plan.Prism, Pose = GraspAngles(Plan);
    const Over = Api.GripperSamples(Pose, JawPair(OpenDeg)).filter(S => S.Part !== 'FixedFinger' && Api.FootprintDistance(Prism.Footprint, S.P) <= Rules.OverTopGrowCm);
    if (Over.length) Assert.ok(Math.min(...Over.map(S => S.P.z)) >= Prism.TopZ + 0.49, `${Name}: palm or jaw ${(Math.min(...Over.map(S => S.P.z)) - Prism.TopZ).toFixed(2)} cm over the top`);
    const Pad = PadZ(Pose);
    Assert.ok(Math.min(Prism.TopZ, Math.max(...Pad)) - Math.max(Prism.BaseZ, Math.min(...Pad)) >= Rules.MinPadOverlapCm, `${Name}: pad overlap`);
  }
});

// --- 5. Refusals ------------------------------------------------------------------------

Test('objects this gripper cannot grasp are refused with the reason and what the user can change', () => Strictly(() => {
  // A round tin measured 6.0 x 5.5 may be 8.0 cm across: no wrist pitch holds it (the jaw-plane sweep tried them all).
  Api.SolveCounter.Count = 0;
  const Tin = Reach(Obj('tin', -5.44, 36.39, -1.97, 10.16, [6.0, 5.5], { supportZMeasured: -1.77 }));
  Assert.ok(Api.SolveCounter.Count <= 120, `${Api.SolveCounter.Count} IK solves for the wide tin`);
  Assert.equal(Tin.Kind, 'ungraspable');
  Assert.equal(Tin.Refusal.Code, 'wide');
  Assert.match(Tin.Text, /^NOT GRASPABLE \(37\.\d cm from the right arm's pan axis\): it measured 6\.0 x 5\.5 cm \(round: up to 8\.0 cm across\); .*close \d\.\d cm.*up to 5\.0 cm from the fixed jaw.*; with the wrist nearer level the jaws do not hold it either \(at best, at wrist -?[\d.]+ deg .*squeeze it out.*\)\. Do not move toward it: reply ask and tell the user why$/);
  const Line = Api.TaskObjectLine(State(Obj('tin', -5.44, 36.39, -1.97, 10.16, [6.0, 5.5]), Tin), { x: -13.3, y: 20.1, z: 12.4 }, { GripperFraction: 0.8, Closed: false });
  Assert.match(Line, /RIGHT ARM: NOT GRASPABLE/);
  Assert.doesNotMatch(Line, /"motion_proposal"/);
  const Turnable = Reach(Obj('eraser', -5, 35, -2.3, -0.3, [5, 2]));
  Assert.equal(Turnable.Refusal.Code, 'wide');
  Assert.match(Turnable.Refusal.Advice, /2\.0 cm side/);
  Assert.match(Turnable.Text, /turned so its 2\.0 cm side spans the jaws/);
  const Cube = Reach(Obj('cube', -5, 32, -2.3, 1.7, [4, 4]));
  Assert.equal(Cube.Refusal.Code, 'wide');
  Assert.doesNotMatch(Cube.Refusal.Advice + Cube.Text, /side (faces|spans) the jaws/);
  Assert.equal(Reach(Obj('coin', -5, 32, -2.3, -2.0, [2, 2])).Refusal.Code, 'flat');
  const Unmeasured = Reach({ ...Usb, sizeCm: null });
  Assert.equal(Unmeasured.Refusal.Code, 'unmeasured');
  Assert.match(Unmeasured.Text, /its size was not measured by the depth camera/);
  const Crowded = Reach(Usb, [Obj('box', 7.9, 31.8, -2.5, -0.5, [1.5, 1.5])]);
  Assert.equal(Crowded.Refusal.Code, 'neighbour');
  Assert.match(Crowded.Text, /the gripper's fixed finger would go \d\.\d cm into the box measured at X 7\.9, Y 31\.8 beside it/);
  Assert.equal(Reach(Usb, [Obj('bottle', 9.5, 31.0, -2.3, 5.6, [3, 3])]).Refusal.Code, 'neighbour');
  const Far = Reach(Bottle);
  Assert.ok(Far.Kind === 'blocked' || Far.Kind === 'out-of-reach', Far.Text);
  Assert.match(Far.Text, /BLOCKED|OUT OF REACH/);
}));

Test('the best-effort pass plans what only predicts a failed grip, warned; what is certain stays refused', () => {
  // The tin: as measured (6.0 x 5.5), not as the 8.0 cm it may be, with the grip it would need. The eraser and the 4 x 4 cube
  // the strict passes refused hold with every check at a pitch or height they did not try.
  const Tin = Reach(Obj('tin', -5.44, 36.39, -1.97, 10.16, [6.0, 5.5], { supportZMeasured: -1.77 }));
  Assert.equal(Tin.Kind, 'graspable', Tin.Text);
  Assert.match(Tin.Plan.Warnings[0], /^planned as measured \(6\.0 x 5\.5 cm\), not as a round object up to 8\.0 cm across/);
  Assert.ok(Tin.Plan.Warnings.some(Why => /^the grip may slip/.test(Why)), JSON.stringify(Tin.Plan.Warnings));
  for (const Object_ of [Obj('eraser', -5, 35, -2.3, -0.3, [5, 2]), Obj('cube', -5, 32, -2.3, 1.7, [4, 4])]) Assert.equal(Reach(Object_).Kind, 'graspable', Object_.label);
  // The coin lying flat: the pad barely faces it (the strict passes' "flat").
  const Coin = Reach(Obj('coin', -5, 32, -2.3, -2.0, [2, 2]));
  Assert.ok(Coin.Plan?.Warnings.some(Why => /^only 0\.\d cm of the fixed pad faces it$/.test(Why)), Coin.Text);
  // Certain: unmeasured, a neighbour where the fixed finger goes down, out of reach.
  Assert.equal(Reach({ ...Usb, sizeCm: null }).Refusal.Code, 'unmeasured');
  Assert.equal(Reach(Usb, [Obj('box', 7.9, 31.8, -2.5, -0.5, [1.5, 1.5])]).Refusal.Code, 'neighbour');
  Assert.equal(Reach(Usb, [Obj('bottle', 9.5, 31.0, -2.3, 5.6, [3, 3])]).Refusal.Code, 'neighbour');
  Assert.match(Reach(Bottle).Text, /BLOCKED|OUT OF REACH/);
});

Test('a box wider than the jaws span fully open is refused as too wide at either wrist roll, never planned at its rim', () => {
  // TestAiTaskSafetyUi's 12 x 12 cm box. With Aaran's saved Home (wrist roll 1590, not the pinned 1513) the best-effort pass
  // of 2026-10-18 planned it at 0.6 cm under its top: the tilted jaws "met" a sliver of its rim past the finger tips.
  const Box = Obj('box', -5, 35, -2.3, 4.7, [12, 12]);
  for (const Roll of [Home.wrist_roll.presentRaw, 1590]) {
    const Servos = structuredClone(Home);
    Object.assign(Servos.wrist_roll, { presentRaw: Roll, targetRaw: Roll });
    const Verdict = Reach(Box, [], Servos);
    Assert.equal(Verdict.Kind, 'ungraspable', `roll ${Roll}: ${Verdict.Text}`);
    Assert.equal(Verdict.Refusal.Code, 'wide', Verdict.Text);
    Assert.match(Verdict.Text, /it measured 12\.0 x 12\.0 cm; .*more than the 11\.\d cm the moving jaw reaches across from the fixed pad fully open/);
  }
});

// --- 6. The printed points ---------------------------------------------------------------

Test('the printed grasp and hover points are the ones checked: the snap takes them as they are, and the floor and basket allow them', () => {
  const Home_ = Model(TipOf(Home), Api.wristPitchFromAngles(Api.currentAngles(Home)));
  for (const [Name] of Planned) {
    const Plan = Plans.get(Name).Plan;
    const Grasp = Model(Plan.Grasp, Plan.Pitch), Hover = Model(Plan.Hover, Plan.HoverPitch);
    const Snap = (Requested, Anchor) => Api.SnapAiTarget({ Requested, Anchor, Servos: Home, ElbowMode: 'up', Limits: undefined, AbsoluteXY: true, Unreliable: [] }, Api.SnapRules, true);
    Assert.equal(Snap(Grasp, Hover).Kind, 'exact', `${Name} grasp`);
    Assert.equal(Snap(Hover, Home_).Kind, 'exact', `${Name} hover`);
    for (const [Point, Anchor] of [[Grasp, Hover], [Hover, Home_]]) {
      Assert.equal(Api.AiTableFloorReason(Point, Point.z - Anchor.z), null, Name);
      Assert.equal(Api.BasketTargetReason(Point), null, Name);
      Assert.equal(Api.AiJawFloorReason(Point, Api.solveIkPreview(Point, 'up', Home).angles, Api.currentAngles(Home)), null, Name);
    }
  }
});

// --- 7. The incident ------------------------------------------------------------------------

Test('2026-09-30 23:50: the descent onto the can that put the fixed finger into its lid is refused', () => {
  const Target = Model({ x: -5.705, y: 36.4625, z: 3.486 }, -52);
  const Angles = Api.solveIkPreview(Target, 'up', S2350).angles, Now = Api.currentAngles(S2350);
  const Rise = Api.mirrorDisplayPoint(Target).z - TipOf(S2350).z;
  // M3 plans the can: the refusal names the plan's points beside it.
  const CanPlan = Reach(Can2, [], S2350).Plan;
  Assert.ok(CanPlan, 'the can has a GRASP PLAN');
  const Text = Api.GraspGuardReason(Target, Angles, Now, Rise, Guard([AsTask(Can2)], { Plan: CanPlan }));
  Assert.match(Text, /FIXED finger.*can/);
  Assert.match(Text, /a grasp goes BESIDE the can: use the GRASP PLAN: hover point X -?[\d.]+, Y [\d.]+, Z [\d.]+ \(wrist -?[\d.]+\)/);
  const Refused = Api.GraspGuardReason(Target, Angles, Now, Rise, Guard([AsTask(Can2)], { Refusal: { Code: 'wide', Text: 'test refusal', Advice: '' } }));
  Assert.match(Refused, /FIXED finger.*can.*NOT GRASPABLE \(test refusal\)/);
  Assert.ok(Api.GraspGuardReason(Target, Angles, Now, Rise, Guard([AsTask(Can3)])), 'The occluded 23:50:21 measurement alone refuses it too');
  const Path_ = Api.PathContact(Now, Angles, Now.gripper, [Api.ObjectPrism(Can2)], Rules.ContactMarginCm);
  Assert.equal(Path_.Part, 'FixedFinger');
  Assert.ok(Path_.FirstS <= 0.4, `first contact at s ${Path_.FirstS}`);
});

// --- 8. The guard ---------------------------------------------------------------------------

Test('the guard allows the plan and refuses deviations from it', () => {
  const Task = State(Usb, UsbReach);
  const UsbGuard = (Held = false) => Api.BuildGraspGuard([Usb], Task, Held);
  Assert.equal(UsbGuard().TaskPrism.Role, 'task');
  const Check = (From, Point, Pitch, G = UsbGuard()) => {
    const Target = Model(Point, Pitch);
    const Preview = Api.solveIkPreview(Target, 'up', From);
    return Api.GraspGuardReason(Target, Preview.angles, Api.currentAngles(From), Point.z - TipOf(From).z, G);
  };
  const Hover = UsbPlan.Hover, Grasp = UsbPlan.Grasp;
  Assert.equal(Check(Home, Hover, UsbPlan.HoverPitch), null, 'Home to the hover point');
  const AtHover = At(Home, Solve(Hover, UsbPlan.HoverPitch, Home).angles);
  Assert.equal(Check(AtHover, Grasp, UsbPlan.Pitch), null, 'the hover point to the grasp point');
  Assert.equal(Check(Home, { x: Usb.x, y: Usb.y, z: Usb.topZ + 3 }, UsbPlan.HoverPitch), null, 'a hover above the centre at top + 3');
  const AtGrasp = At(AtHover, Solve(Grasp, UsbPlan.Pitch, AtHover).angles);
  const Holding = WithGripper(AtGrasp, 0.25);
  Assert.equal(Check(Holding, { ...Grasp, z: Grasp.z + 6 }, UsbPlan.Pitch, UsbGuard(true)), null, 'lifting it 6 cm, held');
  Assert.match(Check(AtGrasp, { x: Usb.x, y: Usb.y, z: Grasp.z }, UsbPlan.Pitch), /FIXED finger.*usb drive/, 'sideways into its centre at grasp height');
  Assert.match(Check(AtHover, { x: Usb.x, y: Usb.y, z: Grasp.z }, UsbPlan.Pitch), /FIXED finger.*usb drive.*GRASP PLAN: hover point X 6\.9, Y 31\.7, Z 4\.6 \(wrist -49\.7\), then grasp point X 6\.9, Y 31\.7, Z -1\.4 \(wrist -52\.8\)/, 'down onto its centre');
  Assert.ok(Check(AtHover, Grasp, -30), 'the grasp point at another wrist pitch');
  Assert.match(Check(AtHover, { ...Grasp, z: Grasp.z - 0.6 }, UsbPlan.Pitch), /deeper/);
  const NoPlan = { ...UsbGuard(), Plan: null, Refusal: { Code: 'path', Text: 'test refusal', Advice: '' } };
  Assert.match(Check(Home, Grasp, UsbPlan.Pitch, NoPlan), /no GRASP PLAN.*NOT GRASPABLE \(test refusal\)/, 'a descent beside an object with no plan');
  // Its timing: a 10-sample path against three objects.
  const From = Api.currentAngles(AtHover), To = Solve(Grasp, UsbPlan.Pitch, AtHover).angles, Three = UsbGuard();
  Three.Prisms = [Three.Prisms[0], { ...Api.ObjectPrism(Eraser25) }, { ...Api.ObjectPrism(Pen) }];
  const Scale = LoadScale(), Started = performance.now();
  for (let Run = 0; Run < 20; Run++) Api.GraspGuardReason(Model(Grasp, UsbPlan.Pitch), To, From, -6, Three);
  const PerGuard = (performance.now() - Started) / 20;
  Assert.ok(PerGuard <= 10 * Scale, `${PerGuard.toFixed(1)} ms per guard (load x${Scale.toFixed(2)})`);
});

// --- 9-10. The plan lock and twins ------------------------------------------------------------

Test('the plan is kept while the gripper hides part of the object, and made again when it moved or the arm changed', () => {
  const Key = Api.GraspPlanKey(Home, 'up');
  const Can = { Source: Can2, At: new Date(2026, 8, 30, 23, 49, 43).getTime(), Key, Reach: { Plan: null } };
  const Kept = Api.KeepOrReplan(Can, Can3, { x: -4.2, y: 36.1, z: 12.5 }, false, Key);
  Assert.equal(Kept.Keep, true);
  Assert.match(Kept.Note, /re-measured at X -4\.5, Y 34\.3 with the gripper over it \(size 4\.3 x 2\.3, was 5\.0 x 4\.5\): the GRASP PLAN keeps the 23:49:43 measurement/);
  const Plan = { Source: Usb, At: new Date(2026, 8, 30, 23, 49, 22).getTime(), Key, Reach: UsbReach };
  const HomeTip = TipOf(Home);
  const Moved = Api.KeepOrReplan(Plan, { ...Usb, x: Usb.x + 4.8 }, HomeTip, false, Key);
  Assert.equal(Moved.Keep, false);
  Assert.match(Moved.Note, /moved 4\.8 cm since 23:49:22/);
  Assert.equal(Api.KeepOrReplan(Plan, { ...Usb, x: Usb.x + 0.6 }, HomeTip, false, Key).Keep, true);
  Assert.equal(Api.KeepOrReplan(Plan, Usb, HomeTip, false, { ...Key, RollRaw: Key.RollRaw + 20 }).Keep, false);
  Assert.equal(Api.KeepOrReplan(Plan, { ...Usb, x: Usb.x + 4.8 }, HomeTip, true, Key).Keep, true, 'After a close the plan stays');
  Assert.equal(Api.KeepOrReplan(null, Usb, HomeTip, false, Key).Keep, false);
});

Test('the task object measured twice under two labels is one object: twins are never neighbours', () => {
  const Frame = [Can3, Spray, Bottle];
  const CanPlan = { Prism: Api.ObjectPrism(Can2) };
  const Prisms = Api.GuardPrisms(Frame, Can3, CanPlan, Can2).Prisms;
  Assert.deepEqual(Prisms.map(P => P.Label).sort(), ['bottle', 'can']);
  Assert.equal(Prisms.find(P => P.Label === 'can').Role, 'task');
  Assert.equal(Prisms.find(P => P.Label === 'can').BaseZ, Can2.z, 'The plan\'s prism, not the occluded re-measure');
  Assert.deepEqual(Api.NeighbourObjects(Frame, Can3).map(O => O.label), ['bottle']);
  // Without a plan the twin stays, marked as the task object's.
  Assert.equal(Api.GuardPrisms(Frame, Can3, null, null).Prisms.find(P => P.Label === 'spray can').Role, 'twin');
  // The USB re-labelled 0.04 cm away: the plan's own descent passes the guard with this frame's prisms.
  const Twin = { ...Usb, label: 'small black object', x: 4.34 };
  const Task = State(Usb, UsbReach, { Binding: { Object: Twin, Source: 'position', DistanceCm: 0.04 } });
  const G = Api.BuildGraspGuard([Twin, Bottle], Task, false);
  Assert.deepEqual(G.Prisms.map(P => P.Label), ['usb drive', 'bottle']);
  const AtHover = At(Home, Solve(UsbPlan.Hover, UsbPlan.HoverPitch, Home).angles);
  const Target = Model(UsbPlan.Grasp, UsbPlan.Pitch);
  Assert.equal(Api.GraspGuardReason(Target, Api.solveIkPreview(Target, 'up', AtHover).angles, Api.currentAngles(AtHover), -6, G), null);
});

// --- 11-12. What the model is told, step by step -----------------------------------------------

Test('the GRASP PLAN line: the verbatim plan, then one step at a time from where the tip is', () => {
  const Task = State(Usb, UsbReach);
  const Line = (Tip, Fraction, Closed = false, S = Task) => Api.TaskObjectLine(S, Tip, { GripperFraction: Fraction, Closed });
  const HomeTip = { x: -13.3, y: 20.1, z: 12.4 };
  Assert.equal(Line(HomeTip, 0.8), 'TASK OBJECT "usb drive": detected this check-in at X 4.3, Y 32.9, base Z -2.5, top Z -0.4. RIGHT ARM: graspable: GRASP POINT X 6.9, Y 31.7, Z -1.4 with wrist -52.8 deg; HOVER POINT X 6.9, Y 31.7, Z 4.6 with wrist -49.7 deg (37.3 cm from the right arm\'s pan axis). GRASP PLAN (checked by the app against the gripper\'s shape): the tip is the inner face of the FIXED jaw (the one with the red tape) and only the other jaw moves, so the tip goes BESIDE the usb drive, 1.0 cm outside its left-near side, NOT over its centre; the open moving jaw then closes across it and pushes it against the fixed jaw. That 1.0 cm is the grasp gap the user set for the fixed jaw. The grasp height is its mid-height. NEXT STEP of the grasp (tip now X -13.3, Y 20.1, Z 12.4; gripper 80% open): go to the HOVER POINT, straight above the grasp point beside the usb drive (not above its centre): "motion_proposal":{"target_cm":{"x":6.9,"y":31.7,"z":4.6},"wrist_pitch_degrees":-49.7}. Copy it exactly (one step per motion, never skipping ahead); the next check-in gives the step after it. Coordinates in your earlier replies are stale: use only this measurement.');
  // Open and hover (ONE chained motion: the gripper opens while the tip moves), descend (its own motion), close and lift (one chained motion), done.
  const OpenHover = Line(HomeTip, 0.034);
  Assert.match(OpenHover, /NEXT STEP of the grasp \(tip now X -13\.3, Y 20\.1, Z 12\.4; gripper 3% open\): open the gripper to 80% and go to the HOVER POINT, straight above the grasp point beside the usb drive \(not above its centre\), as ONE chained motion \(the gripper opens while the tip moves\): "motion_proposal":\{"target_cm":\{"x":6\.9,"y":31\.7,"z":4\.6\},"wrist_pitch_degrees":-49\.7,"gripper_before":0\.8\}\. Copy it exactly: it is ONE motion_proposal, and the gripper opens while the tip moves; the next check-in gives the step after it\./);
  Assert.deepEqual(MotionIn(OpenHover), { target_cm: UsbPlan.Hover, wrist_pitch_degrees: UsbPlan.HoverPitch, gripper_before: 0.8 });
  Assert.deepEqual(MotionIn(Line(HomeTip, 0.8)), { target_cm: UsbPlan.Hover, wrist_pitch_degrees: UsbPlan.HoverPitch });
  const Descend = Line(UsbPlan.Hover, 0.8);
  Assert.match(Descend, /the tip is above the grasp point: descend straight down to the GRASP POINT Z -1\.4: "motion_proposal":\{"move_delta_cm":\{"left":0,"farther":0,"up":-6\.0\},"wrist_pitch_degrees":-52\.8\}\./);
  Assert.deepEqual(MotionIn(Descend), { move_delta_cm: { left: 0, farther: 0, up: -6 }, wrist_pitch_degrees: UsbPlan.Pitch });
  // Closed at the hover point it opens alone: the step after it is the descent, never chained.
  Assert.deepEqual(MotionIn(Line(UsbPlan.Hover, 0.034)), { direct_servo_targets: [{ name: 'gripper', target_fraction: 0.8 }] });
  const CloseLift = Line(UsbPlan.Grasp, 0.8);
  Assert.match(CloseLift, /the open jaws are beside it at the grasp point: close the gripper and lift as ONE chained motion \(the moving jaw pushes it against the fixed jaw and the gripper holds it, then the tip rises 6 cm straight up\): "motion_proposal":\{"move_delta_cm":\{"left":0,"farther":0,"up":6\},"gripper_before":0\.0\}\./);
  Assert.deepEqual(MotionIn(CloseLift), { move_delta_cm: { left: 0, farther: 0, up: 6 }, gripper_before: 0 });
  Assert.deepEqual(MotionIn(Line(UsbPlan.Grasp, 0.25, true)), { move_delta_cm: { left: 0, farther: 0, up: 6 } });
  Assert.match(Line({ ...UsbPlan.Grasp, z: UsbPlan.Grasp.z + 6 }, 0.25, true), /it is lifted\. If the image shows it in the jaws, reply task_status "done"/);
  // 0.5 cm toward the object above its top: back to the hover point; 1.0 cm away from it: still the descent.
  const U = UsbPlan.JawAxis, V = { x: -U.y, y: U.x };
  const Toward = { x: UsbPlan.Hover.x + 0.5 * U.x, y: UsbPlan.Hover.y + 0.5 * U.y, z: UsbPlan.Hover.z };
  Assert.deepEqual(MotionIn(Line(Toward, 0.8)), { target_cm: UsbPlan.Hover, wrist_pitch_degrees: UsbPlan.HoverPitch });
  const Away = { x: UsbPlan.Hover.x - U.x, y: UsbPlan.Hover.y - U.y, z: UsbPlan.Hover.z };
  const Down = MotionIn(Line(Away, 0.8));
  Assert.equal(Down.wrist_pitch_degrees, UsbPlan.Pitch);
  Assert.ok(Math.abs(Away.x + Down.move_delta_cm.left - UsbPlan.Grasp.x) <= 0.06 && Math.abs(Away.y + Down.move_delta_cm.farther - UsbPlan.Grasp.y) <= 0.06
    && Math.abs(Away.z + Down.move_delta_cm.up - UsbPlan.Grasp.z) <= 0.06, JSON.stringify(Down));
  // Below the top beside it but not at the grasp point: straight up first.
  const Off = { x: UsbPlan.Grasp.x + 2 * V.x, y: UsbPlan.Grasp.y + 2 * V.y, z: UsbPlan.Grasp.z };
  const Rise = Line(Off, 0.8);
  Assert.match(Rise, /below the usb drive's top but not at the grasp point: rise straight up first/);
  Assert.deepEqual(MotionIn(Rise), { move_delta_cm: { left: 0, farther: 0, up: 6 } });
  // A missed close at the grasp point: up with the jaws closed, then open.
  Assert.deepEqual(MotionIn(Line(UsbPlan.Grasp, 0.034, true)), { move_delta_cm: { left: 0, farther: 0, up: 6 } });
  Assert.deepEqual(MotionIn(Line(UsbPlan.Hover, 0.034, true)), { direct_servo_targets: [{ name: 'gripper', target_fraction: 0.8 }] });
  // Closed beside it (not after a close): up before opening, so the jaw does not swing into it.
  Assert.deepEqual(MotionIn(Line(UsbPlan.Grasp, 0.034)), { move_delta_cm: { left: 0, farther: 0, up: 6 } });
  for (const Text of [Line(HomeTip, 0.8), Descend, Rise]) for (const Snippet of Text.match(/\{"(target_cm|move_delta_cm)":\{[^}]*\}[^}]*\}/g)) Assert.doesNotThrow(() => JSON.parse(Snippet), Snippet);
  // A kept plan says why.
  const Kept = State(Usb, UsbReach);
  Kept.Plan.Note = 're-measured at X 4.5, Y 33.0 with the gripper over it (size 1.0 x 1.2, was 1.7 x 1.8): the GRASP PLAN keeps the 23:49:22 measurement';
  Assert.match(Line(HomeTip, 0.8, false, Kept), /^TASK OBJECT "usb drive": detected this check-in at X 4\.3, Y 32\.9, base Z -2\.5, top Z -0\.4 \(re-measured at X 4\.5, Y 33\.0 with the gripper over it .*\)\. RIGHT ARM: graspable/);
});

Test('with the object hidden under the gripper at the grasp point, the close is still given', () => {
  const Hidden = { ...State(Usb, UsbReach), DetectedNow: false, Reachability: null };
  Assert.ok(Math.hypot(UsbPlan.Grasp.x - Usb.x, UsbPlan.Grasp.y - Usb.y) > 2.8);
  const Line = Api.TaskObjectLine(Hidden, UsbPlan.Grasp, { GripperFraction: 0.8, Closed: false });
  Assert.match(Line, /NOT detected this check-in.*RIGHT ARM when last measured: graspable: GRASP POINT.*close the gripper and lift as ONE chained motion/);
  Assert.deepEqual(MotionIn(Line), { move_delta_cm: { left: 0, farther: 0, up: 6 }, gripper_before: 0 });
  Assert.doesNotMatch(Api.TaskObjectLine(Hidden, { x: -13.3, y: 20.1, z: 12.4 }, { GripperFraction: 0.8, Closed: false }), /NEXT STEP/, 'Not once the gripper is far away');
});

// --- 13-14. The prompt and the validator ----------------------------------------------------------

Test('the prompt says the tip is the fixed jaw and a grasp goes beside the object, never to its mid-height centre', () => {
  const Prompt = Api.AiPrompt('', true, false, 'Behind');
  for (const Text of ['GRASP RULE', 'FIXED jaw', 'NOT GRASPABLE', 'OUT OF REACH or BLOCKED, do not move toward it', 'never change wrist_roll']) Assert.ok(Prompt.includes(Text), Text);
  Assert.doesNotMatch(Prompt, /MID-HEIGHT|tip tape must end up on it/);
  Assert.doesNotMatch(Prompt, /"wrist_pitch_degrees":-?\d/);
  const Summary = Api.MeasuredObjectsSummary({ objects: [Usb], rmsCm: 0.2, error: null, errorCode: null, cameraCheck: null, withheld: [] });
  Assert.match(Summary, /centre is NOT a grasp point/);
  Assert.doesNotMatch(Summary, /MID-HEIGHT/);
});

Test('the validator refuses the incident descent and a wrist roll in an object task, and passes the plan\'s own steps', () => {
  const Reply = Motion => JSON.stringify({ response_to_user: 'x', task_status: 'continue', motion_proposal: Motion });
  const Anchor = Servos => ({ ...Api.forwardKinematics(Api.currentAngles(Servos)).at(-1), wristPitch: Api.wristPitchFromAngles(Api.currentAngles(Servos)) });
  const Parse = (Motion, Servos) => Api.ParseResponse(Reply(Motion), Anchor(Servos), Servos).motion;
  const CanGuard = Guard([AsTask(Can2)], { Refusal: Reach(Can2, [], S2350).Refusal });
  const Incident = Api.Validate(Parse({ target_cm: { x: -5.705, y: 36.4625, z: 3.486 }, wrist_pitch_degrees: -52 }, S2350), 'up', S2350, [], undefined, null, CanGuard);
  Assert.equal(Incident.possible, false);
  Assert.ok(Incident.issues.some(Issue => /FIXED finger \(the red-taped jaw; the tip is on its inside\) would go \d\.\d cm into the can/.test(Issue)), Incident.issues.join(' | '));
  const Roll = Parse({ direct_servo_targets: [{ name: 'wrist_roll', target_raw: Home.wrist_roll.targetRaw + 300 }] }, Home);
  Assert.ok(Api.Validate(Roll, 'up', Home, [], undefined, null, CanGuard).issues.some(Issue => /Wrist roll is held during an object task/.test(Issue)));
  Assert.equal(Api.Validate(Roll, 'up', Home, [], undefined, null, null).possible, true);
  const UsbGuard = Api.BuildGraspGuard([Usb], State(Usb, UsbReach), false);
  const Hover = Api.Validate(Parse({ target_cm: UsbPlan.Hover, wrist_pitch_degrees: UsbPlan.HoverPitch }, Home), 'up', Home, [], undefined, null, UsbGuard);
  Assert.deepEqual(Hover.issues, []);
  const AtHover = At(Home, Solve(UsbPlan.Hover, UsbPlan.HoverPitch, Home).angles);
  const Step = MotionIn(Api.TaskObjectLine(State(Usb, UsbReach), TipOf(AtHover), { GripperFraction: 0.8, Closed: false }));
  Assert.ok(Step.move_delta_cm, JSON.stringify(Step));
  const Descent = Api.Validate(Parse(Step, AtHover), 'up', AtHover, [], undefined, null, UsbGuard);
  Assert.deepEqual(Descent.issues, []);
  // The model's own centre descent from there: refused by the guard.
  const Centre = Api.Validate(Parse({ target_cm: { x: Usb.x, y: Usb.y, z: UsbPlan.Grasp.z }, wrist_pitch_degrees: UsbPlan.Pitch }, AtHover), 'up', AtHover, [], undefined, null, UsbGuard);
  Assert.ok(Centre.issues.some(Issue => /FIXED finger.*usb drive/.test(Issue)), Centre.issues.join(' | '));
});

// --- M1b: the snap, the corridor, a second measurement, the gripper sweep, narrow objects -----------

Test('the snap never moves a refused fixed-jaw descent, and a moved target keeps the gripper clear of measured objects', () => {
  const UsbGuard = Api.BuildGraspGuard([Usb], State(Usb, UsbReach), false);
  const Anchor = Model(TipOf(Home), Api.wristPitchFromAngles(Api.currentAngles(Home)));
  const Snap = (Point, Pitch, Guard_ = UsbGuard, ExactOnly = false) => Api.SnapAiTarget({ Requested: Model(Point, Pitch), Anchor, Servos: Home, ElbowMode: 'up',
    Limits: undefined, AbsoluteXY: true, Unreliable: [], Grasp: Guard_ }, Api.SnapRules, ExactOnly);
  const Centre = Snap({ x: Usb.x, y: Usb.y, z: UsbPlan.Grasp.z }, -90);
  Assert.equal(Centre.Code, 'grasp', JSON.stringify(Centre.Kind));
  // A hover the wrist cannot reach at -90 is moved to a pitch that keeps every part clear.
  const Hover = Snap(UsbPlan.Hover, -90);
  Assert.ok(Hover.Kind === 'pitch' || Hover.Kind === 'position', JSON.stringify(Hover));
  const Angles = Api.solveIkPreview(Hover.Target, 'up', Home).angles;
  Assert.equal(Api.GraspGuardReason(Hover.Target, Angles, Api.currentAngles(Home), Hover.Target.z - Anchor.z, UsbGuard), null);
  // The pose rule the searches use agrees with the full guard at the plan's grasp and at the centre.
  const Pose = Api.GuardPoseCheck(UsbGuard, Api.currentAngles(Home));
  const Links = (Point, Pitch) => Api.CadForward(Api.solveIkPreview(Model(Point, Pitch), 'up', Home).angles).Links;
  Assert.equal(Pose(Model(UsbPlan.Grasp, UsbPlan.Pitch), Links(UsbPlan.Grasp, UsbPlan.Pitch)), true);
  Assert.equal(Pose(Model({ x: Usb.x, y: Usb.y, z: UsbPlan.Grasp.z }, UsbPlan.Pitch), Links({ x: Usb.x, y: Usb.y, z: UsbPlan.Grasp.z }, UsbPlan.Pitch)), false);
  Assert.equal(Pose(Model({ ...UsbPlan.Grasp, z: UsbPlan.Grasp.z - 0.8 }, UsbPlan.Pitch), Links(UsbPlan.Grasp, UsbPlan.Pitch)), false, 'deeper than the plan');
});

Test('a re-measure under the gripper whose near side came toward the fixed jaw stops the descent; one that agrees confirms the plan', () => {
  const Key = Api.GraspPlanKey(Home, 'up');
  const Plan = { Source: Usb, At: 0, Key, Reach: UsbReach };
  const Over = { ...UsbPlan.Hover };
  // 1.0 cm toward the finger (along -u), measured with the gripper over it.
  const Toward = { ...Usb, x: Usb.x - 1.0 * UsbPlan.JawAxis.x, y: Usb.y - 1.0 * UsbPlan.JawAxis.y };
  const Moved = Api.KeepOrReplan(Plan, Toward, Over, false, Key);
  Assert.equal(Moved.Keep, true);
  Assert.ok(Moved.Corridor && Moved.Corridor.ShiftCm > 0.7, JSON.stringify(Moved.Corridor));
  Assert.equal(Moved.Confirmed, false);
  // Twins are ignored: another label at the same spot is not the bound one.
  Assert.equal(Api.KeepOrReplan(Plan, { ...Toward, label: 'small black object' }, Over, false, Key).Corridor, null);
  const Task = State(Usb, UsbReach);
  Object.assign(Task.Plan, { Corridor: Moved.Corridor, Confirmed: false });
  const Line = Api.TaskObjectLine(Task, Over, { GripperFraction: 0.8, Closed: false });
  Assert.match(Line, /NEXT STEP of the grasp: reply ask: the usb drive may have moved into the gripper's way \(re-measured with the gripper over it at X 5\.2, Y 32\.5 \(size 1\.7 x 1\.8\), the GRASP PLAN's fixed finger would come within 0\.\d cm of it/);
  Assert.doesNotMatch(Line, /"motion_proposal"/);
  const Guard_ = Api.BuildGraspGuard([Toward], Task, false);
  Assert.equal(Guard_.Prisms.length, 2, 'The new reading is held against the gripper too');
  // Away from the finger (the jaws covering its far side): kept, and agreeing within 1 cm it confirms the plan.
  const Away = { ...Usb, x: Usb.x + 0.4 * UsbPlan.JawAxis.x, y: Usb.y + 0.4 * UsbPlan.JawAxis.y, sizeCm: [1.2, 1.3] };
  const Agreed = Api.KeepOrReplan(Plan, Away, Over, false, Key);
  Assert.deepEqual([Agreed.Keep, Agreed.Corridor, Agreed.Confirmed], [true, null, true]);
  Assert.equal(Api.KeepOrReplan(Plan, { ...Usb, x: Usb.x + 0.3 }, TipOf(Home), false, Key).Confirmed, true, 'A clean re-measure within 1 cm');
  // Measured once: the hover step and the descent say to check the image first.
  const Once = State(Usb, UsbReach);
  Once.Plan.Confirmed = false;
  Assert.match(Api.TaskObjectLine(Once, TipOf(Home), { GripperFraction: 0.8, Closed: false }),
    /go to the HOVER POINT[^\n]*\. The usb drive was measured once so far: descend only if the image shows the open jaws beside it on its left-near side, else reply ask\./);
  const Descend = Api.TaskObjectLine(Once, UsbPlan.Hover, { GripperFraction: 0.8, Closed: false });
  Assert.match(Descend, /above the grasp point, but the usb drive was measured once so far: descend only if the image shows the open jaws beside it on its left-near side, else reply ask: descend straight down/);
  Assert.deepEqual(MotionIn(Descend), { move_delta_cm: { left: 0, farther: 0, up: -6 }, wrist_pitch_degrees: UsbPlan.Pitch });
  // The 23:50:21 can (an occluded 4.3 x 2.3 under the gripper) against a plan from the 23:49:43 reading.
  const CanPlan = { Source: Can2, At: 0, Key, Reach: { Plan: { Prism: Api.ObjectPrism(Can2), JawAxis: { x: -0.995, y: 0.104 },
    Grasp: { x: Can2.x + 4.0, y: Can2.y - 0.4, z: 4 } } } };
  // A plan without its poses (made before they were kept): its near side only, under c - 0.3: the grasp
  // goes on. Its centre jumped 2.2 cm, so it does not confirm the plan (the image decides the descent).
  const Can = Api.KeepOrReplan(CanPlan, Can3, { x: -4.2, y: 36.1, z: 12.5 }, false, Key);
  Assert.deepEqual([Can.Keep, Can.Corridor, Can.Confirmed], [true, null, false], JSON.stringify(Can));
});

Test('opening the gripper beside a measured neighbour is refused; opening clear of one is not', () => {
  const AtHover = At(Home, Solve(UsbPlan.Hover, UsbPlan.HoverPitch, Home).angles);
  const AtGrasp = WithGripper(At(AtHover, Solve(UsbPlan.Grasp, UsbPlan.Pitch, AtHover).angles), 0.05);
  const Now = Api.currentAngles(AtGrasp), Open = Api.angleFromRawValue(AtGrasp.gripper, Api.fractionToRaw(AtGrasp.gripper, 0.8));
  // A box where the open moving jaw swings, on the far side of the USB from the fixed jaw.
  const U = UsbPlan.JawAxis;
  const Neighbour = Obj('box', Usb.x + 3.2 * U.x, Usb.y + 3.2 * U.y, -2.5, 1.5, [1.5, 1.5]);
  const Crowded = Api.BuildGraspGuard([Usb, Neighbour], State(Usb, UsbReach), false);
  Assert.match(Api.GripperSweepReason(Now, Open, Crowded), /swing the moving jaw .* the box measured at/);
  Assert.equal(Api.GripperSweepReason(Now, Open, Api.BuildGraspGuard([Usb], State(Usb, UsbReach), false)), null, 'The task object itself is what the jaws close on');
  Assert.equal(Api.GripperSweepReason(Now, Open, null), null, 'No object task, no sweep check');
  // Through the validator, as a proposal.
  const Reply = Motion => JSON.stringify({ response_to_user: 'x', task_status: 'continue', motion_proposal: Motion });
  const Anchor = { ...Api.forwardKinematics(Now).at(-1), wristPitch: Api.wristPitchFromAngles(Now) };
  const Motion = Api.ParseResponse(Reply({ direct_servo_targets: [{ name: 'gripper', target_fraction: 0.8 }] }), Anchor, AtGrasp).motion;
  Assert.ok(Api.Validate(Motion, 'up', AtGrasp, [], undefined, null, Crowded).issues.some(Issue => /swing the moving jaw/.test(Issue)));
});

Test('on a narrow object a nearly full close is looked at before it is called a miss', () => {
  const Narrow = Obj('pin', -5, 33, -2.3, -1.0, [1.0, 0.8]);
  const Reach_ = Reach(Narrow);
  Assert.ok(Reach_.Plan, Reach_.Text);
  Assert.ok(Reach_.Plan.WidthAcrossCm < 1.2, `${Reach_.Plan.WidthAcrossCm} cm across`);
  const Line = Api.TaskObjectLine(State(Narrow, Reach_), Reach_.Plan.Grasp, { GripperFraction: 0.04, Closed: true });
  Assert.match(Line, /can still be a grasp\. Lift 3 cm and check the image/);
  Assert.deepEqual(MotionIn(Line), { move_delta_cm: { left: 0, farther: 0, up: 3 } });
  Assert.match(Api.TaskObjectLine(State(Usb, UsbReach), UsbPlan.Grasp, { GripperFraction: 0.04, Closed: true }), /closed all the way, so they missed it/,
    'A wider object closed all the way is a miss');
});

// --- 15-16. Cost and intent ---------------------------------------------------------------------

Test('planning costs a bounded number of IK solves, both reach scans included', () => {
  const Scale = LoadScale();
  Api.SolveCounter.Count = 0;
  const Started = performance.now();
  Assert.ok(Reach(Usb).Plan);
  Assert.ok(Api.SolveCounter.Count <= 120, `${Api.SolveCounter.Count} IK solves`);
  Assert.ok(performance.now() - Started < 2000 * Scale, `${(performance.now() - Started).toFixed(0)} ms (load x${Scale.toFixed(2)})`);
});

Test('what the task does to its object: grasp, contact or only approach; a contact task may touch it', () => {
  const Intent = Task => Api.TaskIntent(Api.ParseTaskObject(Task));
  Assert.equal(Intent('Grab the can'), 'grasp');
  Assert.equal(Intent('touch the eraser'), 'contact');
  Assert.equal(Intent('move the gripper above the eraser'), 'approach');
  Assert.equal(Intent('Pick up the pen that is on the left'), 'grasp');
  Assert.equal(Intent('Put the eraser in the bin'), 'grasp');
  Assert.equal(Intent('hover over the cup'), 'approach');
  const Target = Model({ x: Usb.x, y: Usb.y, z: Usb.topZ + 0.2 }, -60);
  const Angles = Api.solveIkPreview(Target, 'up', Home).angles;
  const Touch = Api.BuildGraspGuard([Usb], { ...State(Usb, Reach(Usb)), Intent: 'contact', Plan: null }, false);
  Assert.equal(Api.GraspGuardReason(Target, Angles, Api.currentAngles(Home), -12, Touch), null);
  Assert.ok(Api.GraspGuardReason(Target, Angles, Api.currentAngles(Home), -12, { ...Touch, Intent: 'grasp' }));
  // An approach task gets the hover move alone.
  const Approach = Api.TaskObjectLine({ ...State(Eraser25, Api.TaskObjectReachability(Eraser25, Context())), Intent: 'approach', Plan: null },
    { x: -13.3, y: 20.1, z: 12.4 }, { GripperFraction: 0.8, Closed: false });
  Assert.match(Approach, /READY-MADE MOVE from the tip now .*hover above it/);
  Assert.equal(Approach.match(/"motion_proposal"/g).length, 1, 'One move, no descent or close');
  Assert.doesNotMatch(Approach, /close the gripper|grasp height Z/);
});

// --- M3: wide or round objects at flat pitches (the can) -------------------------------------------

const Sweep = Api.Sweep;
// A round slice in the jaw plane (gripper x, z): a D cm disc stretched along the fingers by a jaw plane
// tilted Tilt deg from level (a vertical cylinder's cut), Gap cm from the fixed pad, centred on the pad.
const RoundSlice = (D, Gap, Tilt, Sides = 180) => Sweep.ConvexHull2(Array.from({ length: Sides }, (_, I) => {
  const A = 2 * Math.PI * I / Sides, R = D / 2 / Math.cos(Math.PI / Sides);
  return { x: -0.79 + Gap + D / 2 + R * Math.cos(A), z: -9.81274 + R / Math.cos(Tilt * Math.PI / 180) * Math.sin(A) };
}));
// The can along its bearing from the right arm's pan axis, R cm out (its recorded spot: 37.2).
const Pan = Api.mirrorDisplayPoint({ x: 0, y: 0, z: 0 });
const CanAt = (R, Size = [5.0, 4.5], Label = 'can') => {
  const Dx = Can2.x - Pan.x, Dy = Can2.y - Pan.y, R0 = Math.hypot(Dx, Dy);
  return Obj(Label, Pan.x + Dx * R / R0, Pan.y + Dy * R / R0, Can2.z, Can2.topZ, Size, { supportZMeasured: Can2.supportZMeasured });
};
const CanScale = LoadScale();
Api.SolveCounter.Count = 0;
const CanStarted = performance.now();
const CanReach = Reach(Can2), CanPlan = CanReach.Plan;
const CanCost = { Solves: Api.SolveCounter.Count, Ms: performance.now() - CanStarted, Scale: CanScale };
const CanOpenDeg = Plan => Api.angleFromRawValue(Home.gripper, Api.fractionToRaw(Home.gripper, Plan.OpenFraction));
// The near face along u of a footprint within the finger's band, and how far the printed grasp point is outside it.
const PadOutside = (Plan, Footprint) => {
  const Uh = Plan.JawAxis, Vh = { x: -Uh.y, y: Uh.x };
  const Across = Footprint.map(P => Dot(P, Vh)), Cv = (Math.min(...Across) + Math.max(...Across)) / 2;
  return Math.min(...Band(Footprint, Vh, Cv - Rules.FingerBandHalfCm, Cv + Rules.FingerBandHalfCm).map(P => Dot(P, Uh))) - Dot(Plan.Grasp, Uh);
};
const DiscAround = (Centre, D, Sides = 96) => Array.from({ length: Sides }, (_, I) => ({ x: Centre.x + D / 2 * Math.cos(2 * Math.PI * I / Sides), y: Centre.y + D / 2 * Math.sin(2 * Math.PI * I / Sides) }));

Test('the jaw-plane sweep reproduces the design\'s CAD sweep: a 6 cm can pushes 16 and 15 deg at wrist 0 and -15 (held), 28 and 37 at -30 and -45 (refused)', () => {
  const Final = [0, 15, 30, 45].map(Tilt => Sweep.CloseOn(RoundSlice(6.0, 1.0, Tilt), OpenDeg));
  const First = [0, 15, 30, 45].map(Tilt => Sweep.CloseOn(RoundSlice(6.0, 0.8, Tilt), OpenDeg, undefined, false));
  // The design reports: touching the pad 16.3 / 15.3 / 28 / 37; first contact from a 0.8 cm gap 23.5 / 22.1 / 34.3 / 44.3.
  [16.3, 15.3, 28, 37].forEach((Want, I) => Assert.ok(Math.abs(Final[I].PushDeg - Want) <= 1, `final push at tilt ${I * 15}: ${Final[I].PushDeg}`));
  [23.5, 22.1, 34.3, 44.3].forEach((Want, I) => Assert.ok(Math.abs(First[I].PushDeg - Want) <= 1, `first push at tilt ${I * 15}: ${First[I].PushDeg}`));
  // The rule (push at most 20 deg, mu >= 0.36): held near level, refused tilted; the jaw stops on it above where the pads meet.
  Assert.deepEqual(Final.map(R => R.Ok && R.PushDeg <= Sweep.SweepRules.MaxLeanDeg), [true, true, false, false]);
  for (const R of Final) Assert.ok(R.ContactDeg > Sweep.SweepRules.PadsMeetDeg + Sweep.SweepRules.StopAboveMeetDeg);
  // A 3 cm round slice pushes at most 8.7 deg at any pitch (the design); the pads meet at the CAD -11.41 deg.
  for (const Tilt of [0, 15, 30, 45]) Assert.ok(Sweep.CloseOn(RoundSlice(3.0, 1.0, Tilt), OpenDeg).PushDeg <= 8.7);
  let Meet = 20;
  while (Meet > -20 && !Sweep.Overlaps(Sweep.JawPolygon(Meet), Sweep.FixedFingerPolygon)) Meet -= 0.01;
  Assert.ok(Math.abs(Meet - Rules.PadsMeetDeg) <= 0.1, `pads meet at ${Meet.toFixed(2)}`);
  // The outlines are the GLB's: the fixed pad's face at x -0.79, the moving jaw's 0.2 and 0.4 cm steps above its pad.
  const Parts = JSON.parse(ReadFile(Path.join(Root, 'src/lib/GripperParts.json'), 'utf8'));
  Assert.ok(Parts.Silhouettes.FixedFinger.some(([X, Z]) => X === -0.79 && Z <= -10.3) && Parts.Silhouettes.FixedFinger.some(([X, Z]) => X === -0.79 && Z >= -9.5));
  Assert.ok([-1.23, -1.03, -0.83].every(X => Parts.Silhouettes.MovingJaw.some(([Px]) => Px === X)));
});

Test('true widths at a level wrist: 6.0 and 6.6 cm cans pass the design\'s push rule and 7.0 does not; with both pads gripping all three hold', () => {
  const At = D => Sweep.CloseOn(RoundSlice(D, 1.0, 0), OpenDeg);
  const [Six, SixSix, Seven] = [6.0, 6.6, 7.0].map(At);
  Assert.ok(Six.PushDeg <= 20 && SixSix.PushDeg <= 20 && Seven.PushDeg > 20, [Six, SixSix, Seven].map(R => R.PushDeg.toFixed(1)).join(' / '));
  for (const R of [Six, SixSix, Seven]) Assert.ok(R.GripDeg <= 20, `grip ${R.GripDeg}`);
  // Round: the grip needs half the angle between the two pads' pushes.
  for (const R of [Six, SixSix]) Assert.ok(Math.abs(R.GripDeg - R.PushDeg / 2) <= 0.3, `${R.GripDeg} vs ${R.PushDeg}`);
  // Half a cm along the fingers, a 6.6 cm can meets another jaw step: the push alone fails, the grip holds.
  for (const Depth of [-0.5, 0.5]) {
    const Off = Sweep.CloseOn(RoundSlice(6.6, 1.0, 0).map(P => ({ x: P.x, z: P.z + Depth })), OpenDeg);
    Assert.ok(Off.PushDeg > 20 && Off.GripDeg <= 20, `${Depth}: push ${Off.PushDeg} grip ${Off.GripDeg}`);
  }
});

// The builder's 16:49 plan pose (wrist -14, grasp X -1.1, Y 35.1, Z 4.1, no head start), the jaw open 80%.
const OldCanPose = () => ({ ...Solve({ x: -1.1, y: 35.1, z: 4.1 }, -14, Home).angles, gripper: OpenDeg });
const CanDisc = (D, Sides = 180) => ({ ...Api.GraspPrism(Can2), Footprint: DiscAround({ x: Can2.x, y: Can2.y }, D, Sides), Round: null });
// A can plan's pose where its jaws close: the printed grasp point less the arm's sag (GraspRules.GraspSagCm).
const ClosePose = Plan => ({ ...Solve({ ...Plan.Grasp, z: Plan.Grasp.z - (Plan.SagCm ?? 0) }, Plan.Pitch, Home).angles, gripper: OpenDeg });
const Joints = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex'];
const Between = (From, To, S) => { const Out = { ...From }; for (const Name of Joints) Out[Name] = From[Name] + S * (To[Name] - From[Name]); return Out; };

Test('the closing jaw pushes the can along the fingers before the pads grip it: the old plan left it past the pad, the plan starts it nearer the palm', () => {
  // The jaw meets the can where it stands and carries it 0.7 to 1.0 cm toward the tips on its way to the
  // fixed finger (the reviewer's 3D mesh push: 0.68 to 1.13), its first touch leaning about 31 deg.
  const Frame = Api.JawFrame(OldCanPose());
  for (const D of [5.0, 6.0, 6.6]) {
    const Pushed = Sweep.CloseOn(Api.SliceInJawPlane(CanDisc(D), Frame, 0), OpenDeg, undefined, 'push');
    Assert.ok(Pushed.Ok && Pushed.DriftCm >= 0.7 && Pushed.DriftCm <= 1.0, `${D}: drift ${Pushed.DriftCm}`);
    Assert.ok(Pushed.FirstPushDeg >= 28 && Pushed.FirstPushDeg <= 34, `${D}: first push ${Pushed.FirstPushDeg}`);
    // Slid straight (the design's model) it would not move along the fingers at all.
    Assert.equal(Sweep.CloseOn(Api.SliceInJawPlane(CanDisc(D), Frame, 0), OpenDeg).DriftCm, undefined);
  }
  // Judged where the push leaves it, that pose does not hold the can: the push leaves it at the fixed finger's tip.
  const Verdict = Api.ClosingSweep(Api.GraspPrism(Can2), Frame, Home);
  Assert.deepEqual([Verdict.Ok, Verdict.Code], [false, 'tip'], Verdict.Why);
  Assert.ok(Verdict.TipCm < Rules.FingertipMarginCm, `${Verdict.TipCm}`);
  // The plan (a round object, 2026-10-03): the head start along the fingers is the straddle's, deeper than the push's drift
  // needs (the 2026-10-01 plans took the drift alone, so the can ended on the pad's middle, 0.6 cm inside the finger's tip).
  const Closing = CanPlan.Closing;
  Assert.ok(Closing.DriftCm >= 0.6 && Closing.DriftCm <= 1.0, JSON.stringify(Closing));
  Assert.ok(Closing.DepthOffsetCm > Closing.DriftCm && Math.min(CanPlan.Straddle.FixedCm, CanPlan.Straddle.MovingCm) >= Rules.StraddleCm - 0.1,
    JSON.stringify([Closing, CanPlan.Straddle]));
});

Test('the grip is checked wherever the push may leave the can, not at three depths: the jump between the jaw\'s steps is found', () => {
  // At the old pose the grip jumps from 11 to 19 deg between 0.05 and 0.10 cm along the fingers (a 6.6 cm can against the pad).
  const Old = Api.JawFrame(OldCanPose());
  const GripAt = Depth => Sweep.CloseOn(Api.SliceInJawPlane(CanDisc(6.6), Old, Depth), OpenDeg).GripDeg;
  Assert.ok(GripAt(0.05) < 13 && GripAt(0.1) > 17.5, `${GripAt(0.05)} / ${GripAt(0.1)}`);
  // The plan's grip is at least the worst of full push simulations from every 0.05 cm of the tolerance, for its smallest and widest can,
  // where the jaws close (the printed grasp point less the arm's sag) and where the real gripper meets the can (the finger offset).
  const Frame = Api.JawFrame(ClosePose(CanPlan));
  let Worst = 0;
  for (const D of [5.0, 6.67]) {
    for (let Start = -Rules.DepthToleranceCm; Start <= Rules.DepthToleranceCm + 1e-9; Start += 0.05) {
      const Result = Sweep.CloseOn(Api.SliceInJawPlane(Api.ArmPrism(CanDisc(D, 96), CanPlan.JawAxis, CanPlan.FingerOffsetCm), Frame, Start), OpenDeg, undefined, 'push');
      Assert.ok(Result.Ok, `${D} at ${Start}: ${Result.Why}`);
      Worst = Math.max(Worst, Result.GripDeg);
    }
  }
  Assert.ok(Worst <= CanPlan.Closing.GripDeg + 0.5 && Worst <= Sweep.SweepRules.MaxLeanDeg + 0.5, `worst ${Worst} against the plan's ${CanPlan.Closing.GripDeg}`);
});

Test('the grip in 3D: jaws tilted against the can\'s standing side need more friction than their plane shows; the plan keeps within a few deg of the limit', () => {
  // The reviewer's GLB mesh sweep at the old pose, the can slid against the pad: 19.0 / 18.4 / 18.4 deg for 5.0 / 6.0 / 6.6 cm (2D: 8.0 / 10.2 / 11.6).
  const Frame = Api.JawFrame(OldCanPose());
  for (const [D, Want, Plane] of [[5.0, 19.0, 8.0], [6.0, 18.4, 10.2], [6.6, 18.4, 11.6]]) {
    const Result = Sweep.CloseOn(Api.SliceInJawPlane(CanDisc(D), Frame, 0), OpenDeg);
    Assert.ok(Math.abs(Result.GripDeg - Plane) <= 0.3, `${D}: 2D ${Result.GripDeg}`);
    const Grip = Sweep.GripIn3D(Result.Contacts, Frame.U, Frame.H, Frame.A);
    Assert.ok(Math.abs(Grip - Want) <= 1.0, `${D}: 3D ${Grip}`);
  }
  // No pitch holds this can within 20 deg in 3D with the jaws around it (the fallback plan, below): before 2026-10-03 the
  // gentlest (21.9 deg at -14); now the preferred pitch's order among the pitches the screen puts within StrictScreenMarginDeg
  // of the limit, the first whose jaws reach StraddleGoodCm around it (the screen's grip is at mid-height: placed up to 2.5
  // deg more).
  Assert.ok(CanPlan.Closing.Grip3DDeg != null && CanPlan.Closing.Grip3DDeg <= Sweep.SweepRules.MaxLeanDeg + Rules.StrictScreenMarginDeg + 2.5, JSON.stringify(CanPlan.Closing));
});

Test('the can at its recorded spot is planned with the jaw-plane sweep: beside its widest plausible side, the palm clear by the depth tolerance, clear of the desk', () => {
  Assert.equal(CanReach.Kind, 'graspable', CanReach.Text);
  const Plan = CanPlan, Prism = Plan.Prism;
  Assert.equal(Plan.Closing.Mode, 'sweep');
  Assert.deepEqual(Prism.Round, { LoCm: 5.0, HiCm: 6.67, NominalCm: 6.67 });
  Assert.deepEqual(Plan.Closing.DiameterCm, [5.0, 6.67]);
  Assert.ok(Plan.Closing.GripDeg <= 20, JSON.stringify(Plan.Closing));
  Assert.ok(Plan.Closing.ContactDeg > Rules.PadsMeetDeg + 2);
  Assert.ok([0.5, 0.6, 0.8].includes(Plan.OpenFraction));
  // THE STRADDLE (2026-10-03): where the jaws close (the printed point less the sag), both jaws reach at least StraddleCm past
  // its centre, the fixed finger along the fingers and the moving jaw along its own length as it swings over it.
  Assert.ok(Plan.Straddle && Math.min(Plan.Straddle.FixedCm, Plan.Straddle.MovingCm) >= Rules.StraddleCm - 0.1, JSON.stringify(Plan.Straddle));
  const Reach_ = Sweep.StraddleOf(Api.SliceInJawPlane(Prism, Api.JawFrame(ClosePose(Plan))));
  Assert.ok(Math.abs(Reach_.FixedCm - Plan.Straddle.FixedCm) < 0.02 && Math.abs(Reach_.MovingCm - Plan.Straddle.MovingCm) < 0.02, JSON.stringify(Reach_));
  // Its centre between the jaws: across them past the fixed pad, inside the fixed finger's tip and within the moving jaw's reach.
  Assert.ok(Reach_.Centre.x > -0.79 && Reach_.Centre.z > Sweep.FixedFingertipZ && Math.hypot(Reach_.Centre.x - Sweep.MovingJawHinge.x, Reach_.Centre.z - Sweep.MovingJawHinge.z) < Sweep.MovingJawReachCm);
  // At 37.2 cm no pitch and height the planner reached held every size from 5.0 to 6.7 cm in 3D within 20 deg with the jaws
  // around it: PlanGrasp's fallback plan, 21 to 25 deg. The deep grasp (GraspRules.DeepGrasp, 2026-10-04) does not start the
  // can nearer the palm than the palm itself, and the same grasp holds it within 20: a plan with every check.
  Assert.equal(Plan.Deep, true);
  Assert.ok(Plan.Closing.Grip3DDeg <= 20 && Plan.Closing.TipCm >= Rules.FingertipMarginCm, JSON.stringify(Plan.Closing));
  // Printed the arm's sag above where it closes, where the arm stops under it.
  Assert.equal(Plan.SagCm, Rules.GraspSagCm);
  Assert.equal(Plan.Grasp.z, Math.ceil(((Can2.z + Can2.topZ) / 2 + (Plan.RaiseCm ?? 0) + Rules.GraspSagCm) * 10 - 1e-9) / 10);
  // The fixed jaw on the can's left (image-left, +X), c outside its widest plausible side where the camera measured it: about
  // 1.0 cm from a true 6.6 cm can (the camera saw the real fixed finger where the arm model puts it, 2026-10-03: no offset).
  Assert.equal(Plan.Side, 'left');
  Assert.deepEqual([Plan.FingerOffsetCm, Api.PlanArmPrism(Plan)], [0, null]);
  const Gap = PadOutside(Plan, DiscAround(Prism.Centre, 6.6));
  // (Up to 0.2 cm more: the printed point's rounding, and the deep grasp's head start along the fingers, which lean a little across u.)
  Assert.ok(Gap >= Rules.FixedJawClearanceCm - 0.08 && Gap <= Rules.FixedJawClearanceCm + 0.2, `pad ${Gap} cm from a 6.6 cm can`);
  // Every part (the open jaw over the plan's band) clear of the true 6.6 cm can at the guard's margin, at the printed point and
  // where the jaws close.
  const Open = CanOpenDeg(Plan), Jaws = Api.PlanJawDegs(Home, Plan.OpenFraction);
  const Pose = { ...Solve(Plan.Grasp, Plan.Pitch, Home).angles, gripper: Open };
  const TrueCan = { ...Prism, Footprint: DiscAround(Prism.Centre, 6.6), Round: null };
  // (A deep plan's palm by its exact gap, below.)
  const Deep = { PalmAndServo: -Infinity };
  for (const At_ of [Pose, { ...ClosePose(Plan), gripper: Open }]) {
    Assert.equal(Api.DeepestContact(Api.GripperSamples(At_, Jaws), [TrueCan, Prism], Rules.ContactMarginCm, Deep, [TrueCan, Prism]), null);
  }
  // The palm and servo, at the grasp point and on the way down, PalmGapCm from the planned can by their exact gap (the deep
  // grasp; before, the palm's samples kept the plan's margin plus the depth tolerance, 1.0 cm, from it).
  const HoverPose = { ...Solve(Plan.Hover, Plan.HoverPitch, Home).angles, gripper: Open };
  for (let Index = 0; Index <= 10; Index++) {
    const Gap_ = Api.PalmGap(Api.CadForward(Between(HoverPose, Pose, Index / 10)).Links, Prism);
    Assert.ok(Gap_ >= Rules.PalmGapCm - 0.01, `palm ${Gap_.toFixed(2)} cm from it ${Index * 10}% of the way down`);
  }
  Assert.ok(Plan.PalmGapCm >= Rules.PalmGapCm && Plan.PalmGapCm < 1, `${Plan.PalmGapCm}`);
  // The desk: the lowest gripper point open stays 0.2 cm over the support, closing at or over it.
  const Lowest = Deg => Api.GripperLowestZOfLinks(Api.CadForward({ ...Pose, gripper: Deg }).Links);
  Assert.ok(Lowest(Open) >= Can2.supportZMeasured + 0.19, `open: ${Lowest(Open)}`);
  Assert.ok(Math.min(...Array.from({ length: 16 }, (_, I) => Lowest(Rules.PadsMeetDeg + (Open - Rules.PadsMeetDeg) * I / 15))) >= Can2.supportZMeasured - 0.01);
  // Straight down from the hover point (3 cm over its top) to the grasp point, the joint-linear path clear at 0.5 cm.
  Assert.deepEqual([Plan.Hover.x, Plan.Hover.y], [Plan.Grasp.x, Plan.Grasp.y]);
  Assert.ok(Plan.Hover.z >= Can2.topZ + 3);
  Assert.equal(Api.PathContact(HoverPose, Pose, Open, [Prism, TrueCan], Rules.PlanMarginCm, Rules.PathSamples, Jaws, Deep, [Prism, TrueCan]), null);
  // The printed points are exact for the snap; the floor and the basket allow them.
  const HomeAnchor = Model(TipOf(Home), Api.wristPitchFromAngles(Api.currentAngles(Home)));
  const Grasp = Model(Plan.Grasp, Plan.Pitch), Hover = Model(Plan.Hover, Plan.HoverPitch);
  const Snap = (Requested, Anchor) => Api.SnapAiTarget({ Requested, Anchor, Servos: Home, ElbowMode: 'up', Limits: undefined, AbsoluteXY: true, Unreliable: [] }, Api.SnapRules, true);
  Assert.equal(Snap(Grasp, Hover).Kind, 'exact');
  Assert.equal(Snap(Hover, HomeAnchor).Kind, 'exact');
  for (const [Point, Anchor] of [[Grasp, Hover], [Hover, HomeAnchor]]) {
    Assert.equal(Api.AiTableFloorReason(Point, Point.z - Anchor.z), null);
    Assert.equal(Api.BasketTargetReason(Point), null);
  }
  // Cost: the table's first pitch, the screen (no IK) and the pitches tried, raised up to 2.5 cm for the fingertip margin
  // (290 solves on 2026-10-01; the straddle's palm search, its head starts and the deeper plan looked for, StraddleLookOn, cost more).
  Assert.ok(CanCost.Solves <= 2400, `${CanCost.Solves} IK solves, ${CanCost.Ms.toFixed(0)} ms`);
  Assert.ok(CanCost.Ms < 8000 * CanCost.Scale, `${CanCost.Ms.toFixed(0)} ms (load x${CanCost.Scale.toFixed(2)})`);
  // Refused tilted (the judges: refused at -30/-45, accepted near -15..0): the grip, or the palm the head start for it brings near; the plan within the window.
  const At_ = Pitch => Api.PlaceAtPitch(Prism, Pitch, Context(), true);
  for (const Pitch of [-30, -45]) {
    const Refused = At_(Pitch);
    Assert.ok(!Refused.Ok && ['lean', 'palm', 'straddle', ...(Pitch < -40 ? ['reach'] : [])].includes(Refused.Code), `wrist ${Pitch}: ${Refused.Code} ${Refused.Why}`);
  }
  Assert.ok(Plan.Pitch >= -27.5 && Plan.Pitch <= -5, `wrist ${Plan.Pitch}`);
});

Test('what the model is told about the can, verbatim, and the grasp step by step', () => {
  const Task = State(Can2, CanReach);
  const Line = (Tip, Fraction, Closed = false) => Api.TaskObjectLine(Task, Tip, { GripperFraction: Fraction, Closed });
  const HomeTip = { x: -13.3, y: 20.1, z: 12.4 };
  const P = CanPlan, C = P.Closing, Point = Q => `X ${Q.x.toFixed(1)}, Y ${Q.y.toFixed(1)}, Z ${Q.z.toFixed(1)}`;
  // The straddle (2026-10-03): how far the jaws reach around it, and the push's drift after.
  const Around = ` The jaws reach around it before they close: the GRASP POINT is ${C.DepthOffsetCm.toFixed(1)} cm farther out along the fingers than its centre,`
    + ` so the fixed finger reaches ${P.Straddle.FixedCm.toFixed(1)} cm past its middle and the moving jaw, swinging closed, ${P.Straddle.MovingCm.toFixed(1)} cm past it`
    + ' (the app wants at least 1.5 for both: short of its middle, the close squeezes it out past the tips); do not stop the tip short of the GRASP POINT.';
  const Drift = ` Closing, the moving jaw pushes it about ${C.DriftCm.toFixed(1)} cm along the fingers toward their tips before the pads grip it.`;
  // The real-arm corrections, with their reasons: the sag only (FixedFingerOffsetCm 0 since 2026-10-03).
  const Real = ' The GRASP POINT\'s Z is 0.5 cm above where the jaws close, because the arm stops about that much lower than a low target.';
  const Height = P.RaiseCm ? `${P.RaiseCm.toFixed(1)} cm above its mid-height (at mid-height ${P.RaisedFor})` : 'its mid-height';
  // Where the push leaves it against the fixed finger (a plan with every check since the deep grasp; before, the fallback
  // plan said it needed friction for 21 deg in 3D).
  const Tip = ` That leaves its contact with the fixed finger ${C.TipCm.toFixed(1)} cm inside the finger's tip (nearer the tip, the close can squeeze it out past the tips).`;
  // Deep in the jaws (GraspRules.DeepGrasp, 2026-10-04).
  const DeepText = ` It sits deep in the jaws, the gripper's palm and servo ${P.PalmGapCm.toFixed(1)} cm from it at the GRASP POINT (the app keeps them at least 0.25 cm from it on the way down), so do not move the tip farther toward it along the fingers.`;
  // The wrist: as near the preferred -60 deg as the grip allows (GraspRules.PreferredPitchDeg, 2026-10-03).
  const Wrist = ' (the app picked it itself among the pitches the jaws hold it with and the arm reaches beside it with, as near its preferred -60 deg as that grip allows; copy it: beside the can the app holds the wrist to it)';
  Assert.equal(Line(HomeTip, 0.8), `TASK OBJECT "can": detected this check-in at X -5.4, Y 36.4, base Z -2.0, top Z 10.2. RIGHT ARM: graspable: GRASP POINT ${Point(P.Grasp)} with wrist ${P.Pitch} deg; HOVER POINT ${Point(P.Hover)} with wrist ${P.HoverPitch} deg (37.2 cm from the right arm's pan axis). GRASP PLAN (checked by the app against the gripper's shape): the tip is the inner face of the FIXED jaw (the one with the red tape) and only the other jaw moves, so the tip goes BESIDE the can, 1.0 cm outside its left side (planned as if it were 6.7 cm across: a round object measures narrower than it is), NOT over its centre; the open moving jaw then closes across it and pushes it against the fixed jaw. That 1.0 cm is the grasp gap the user set for the fixed jaw. The grasp height is ${Height}. It is too wide for the jaws to close on from straight above, so the wrist stays at ${P.Pitch} deg all the way down${Wrist} and the gripper opens to ${Math.round(P.OpenFraction * 100)}% (exactly that: wider, the open jaw swings farther out).${Around}${DeepText}${Drift}${Tip} The app checked that grip for it 5.0 to 6.7 cm across and up to 0.5 cm off along the fingers.${Real} NEXT STEP of the grasp (tip now X -13.3, Y 20.1, Z 12.4; gripper 80% open): go to the HOVER POINT, straight above the grasp point beside the can (not above its centre): "motion_proposal":{"target_cm":{"x":${P.Hover.x.toFixed(1)},"y":${P.Hover.y.toFixed(1)},"z":${P.Hover.z.toFixed(1)}},"wrist_pitch_degrees":${P.HoverPitch}}. Copy it exactly (one step per motion, never skipping ahead); the next check-in gives the step after it. Coordinates in your earlier replies are stale: use only this measurement.`);
  // Open and hover (one chained motion), straight down at the plan's wrist (its own motion), close and lift (one chained motion), done.
  Assert.deepEqual(MotionIn(Line(HomeTip, 0.034)), { target_cm: P.Hover, wrist_pitch_degrees: P.HoverPitch, gripper_before: P.OpenFraction });
  Assert.deepEqual(MotionIn(Line(HomeTip, 0.8)), { target_cm: P.Hover, wrist_pitch_degrees: P.HoverPitch });
  const Down = MotionIn(Line(P.Hover, 0.8));
  Assert.deepEqual(Down, { move_delta_cm: { left: 0, farther: 0, up: Number((P.Grasp.z - P.Hover.z).toFixed(1)) }, wrist_pitch_degrees: P.Pitch });
  Assert.deepEqual(MotionIn(Line(P.Grasp, 0.8)), { move_delta_cm: { left: 0, farther: 0, up: 6 }, gripper_before: 0 });
  // That chain passes every check at the grasp point beside the 12 cm can: the lift, solved with the jaws closed, ends
  // below the can's top beside it, at the plan's wrist; the guard does not yet know the jaws hold it.
  const AtGraspCan = WithGripper(At(Home, Solve(P.Grasp, P.Pitch, Home).angles), P.OpenFraction);
  const CloseLift = Api.ParseResponse(JSON.stringify({ response_to_user: 'x', task_status: 'continue', motion_proposal: MotionIn(Line(P.Grasp, 0.8)) }),
    Model(TipOf(AtGraspCan), Api.wristPitchFromAngles(Api.currentAngles(AtGraspCan))), AtGraspCan).motion;
  Assert.equal(CloseLift.chain.Order, 'before');
  Assert.deepEqual(Api.Validate(CloseLift, 'up', AtGraspCan, [], undefined, { Object: Can2, State: { GripperFraction: P.OpenFraction, Closed: false }, Plan: P },
    Api.BuildGraspGuard([Can2], Task, false)).issues, []);
  Assert.deepEqual(MotionIn(Line(P.Grasp, 0.28, true)), { move_delta_cm: { left: 0, farther: 0, up: 6 } });
  Assert.match(Line({ ...P.Grasp, z: P.Grasp.z + 6 }, 0.28, true), /it is lifted\. If the image shows it in the jaws, reply task_status "done"/);
  // The guard lets the plan's own moves through and refuses the 23:50 descent onto the can's centre.
  const CanGuard = Api.BuildGraspGuard([Can2], Task, false);
  const Check = (From, Point_, Pitch) => {
    const Target = Model(Point_, Pitch);
    return Api.GraspGuardReason(Target, Api.solveIkPreview(Target, 'up', From).angles, Api.currentAngles(From), Point_.z - TipOf(From).z, CanGuard);
  };
  Assert.equal(Check(Home, P.Hover, P.HoverPitch), null, 'Home to the hover point');
  const AtHover = At(Home, Solve(P.Hover, P.HoverPitch, Home).angles);
  Assert.equal(Check(AtHover, P.Grasp, P.Pitch), null, 'the hover point to the grasp point');
  Assert.match(Check(AtHover, { x: Can2.x, y: Can2.y, z: P.Grasp.z }, P.Pitch), new RegExp(`FIXED finger.*can.*GRASP PLAN: hover point ${Point(P.Hover)} \\(wrist ${P.HoverPitch}\\)`));
  Assert.ok(Check(AtHover, P.Grasp, P.Pitch - 25), 'the grasp point at another wrist pitch');
});

Test('narrowing the gripper to a plan\'s fraction is not a close, and the steps, the closed-jaw rule and the guard all hold that fraction', () => {
  // App.tsx's close rule: an opening ends a close; a move to 30% or less is one; narrowing to 50-60% is not.
  Assert.equal(Api.GripperClosedAfter(0.8, 0.6, false), false);
  Assert.equal(Api.GripperClosedAfter(0.8, 0.5, false), false);
  Assert.equal(Api.GripperClosedAfter(0.8, 0, false), true);
  Assert.equal(Api.GripperClosedAfter(0.6, 0.2, false), true);
  Assert.equal(Api.GripperClosedAfter(0.034, 0.6, true), false, 'an opening ends a close');
  Assert.equal(Api.GripperClosedAfter(0.3, 0.25, true), true, 'closing further stays closed');
  // A standing 4 x 2 x 8 box: planned 60% open. Walk the NEXT STEPs through the validator (grasp state and guard
  // as App.tsx builds them), from Home 80% open.
  const Box = Obj('box', -5, 33, -2.3, 5.7, [4, 2]);
  // (With the fixed jaw's gap as before: with GraspRules.FixedJawExtraStandoffCm the plan is 2.54 cm farther out at wrist -18, 80% open.)
  const Reach_ = WithExtra(0, () => Reach(Box)), Plan = Reach_.Plan, Task = State(Box, Reach_);
  Assert.ok(Plan && Plan.OpenFraction < 0.8, Reach_.Text);
  const Reply = Motion => JSON.stringify({ response_to_user: 'x', task_status: 'continue', motion_proposal: Motion });
  const Anchor = Servos => ({ ...Api.forwardKinematics(Api.currentAngles(Servos)).at(-1), wristPitch: Api.wristPitchFromAngles(Api.currentAngles(Servos)) });
  const Parse = (Motion, Servos) => Api.ParseResponse(Reply(Motion), Anchor(Servos), Servos).motion;
  const Fraction = Servos => Api.rawToFraction(Servos.gripper, Servos.gripper.presentRaw);
  const Run = (Servos, Closed) => {
    const Grasp = { GripperFraction: Fraction(Servos), Closed };
    const Line = Api.TaskObjectLine(Task, TipOf(Servos), Grasp);
    const Motion = MotionIn(Line);
    const Guard_ = Api.BuildGraspGuard([Box], Task, Closed && Grasp.GripperFraction >= Api.HeldFraction);
    const Verdict = Api.Validate(Parse(Motion, Servos), 'up', Servos, [], undefined, { Object: Box, State: Grasp, Plan }, Guard_);
    return { Line, Motion, Verdict, Guard_ };
  };
  let Servos = WithGripper(Home, 0.8), Closed = false;
  // 1-2. Set the gripper to the plan's fraction and go to the hover point, as ONE chained motion (lib/AiChain.ts):
  // the narrowing is not a close, and the hover move is checked with the jaws at the plan's fraction; the guard
  // still holds the box (not held).
  let Step = Run(Servos, Closed);
  Assert.match(Step.Line, new RegExp(`set the gripper to the ${Math.round(Plan.OpenFraction * 100)}% open the GRASP PLAN was checked at \\(wider, the open jaw swings farther out; this is not the close\\) and go to the HOVER POINT[^\\n]*as ONE chained motion`));
  Assert.deepEqual(Step.Motion, { target_cm: Plan.Hover, wrist_pitch_degrees: Plan.HoverPitch, gripper_before: Plan.OpenFraction });
  Assert.equal(Step.Guard_.Held, false);
  Assert.deepEqual(Parse(Step.Motion, Servos).chain, { Order: 'before', TargetRaw: Api.fractionToRaw(Servos.gripper, Plan.OpenFraction), Fraction: Plan.OpenFraction });
  Assert.deepEqual(Step.Verdict.issues, []);
  Closed = Api.GripperClosedAfter(0.8, Plan.OpenFraction, Closed);
  Assert.equal(Closed, false);
  Servos = At(WithGripper(Servos, Plan.OpenFraction), Step.Verdict.preview.angles);
  // At the hover point the 23:50-style centre descent is refused.
  Step = Run(Servos, Closed);
  const Centre = Api.Validate(Parse({ target_cm: { x: Box.x, y: Box.y, z: Plan.Grasp.z }, wrist_pitch_degrees: Plan.Pitch }, Servos), 'up', Servos, [], undefined,
    { Object: Box, State: { GripperFraction: Fraction(Servos), Closed }, Plan }, Step.Guard_);
  Assert.equal(Centre.possible, false, 'the centre descent');
  // 3. The descent, at the plan's fraction and one raw count under it; 0.1 under: open to the plan's fraction first.
  Assert.ok(Step.Motion.move_delta_cm && !('gripper_before' in Step.Motion), Step.Line);
  Assert.deepEqual(Step.Verdict.issues, []);
  // The descent is never chained with the gripper, whichever way round.
  for (const Key of ['gripper_before', 'gripper_after']) {
    const Chained = Api.Validate(Parse({ ...Step.Motion, [Key]: 0.8 }, Servos), 'up', Servos, [], undefined,
      { Object: Box, State: { GripperFraction: Fraction(Servos), Closed }, Plan }, Step.Guard_);
    Assert.ok(Chained.issues.some(Issue => /goes down beside the box[^\n]*the descent is never chained with the gripper/.test(Issue)), `${Key}: ${Chained.issues.join(' | ')}`);
  }
  const Short = structuredClone(Servos);
  Object.assign(Short.gripper, { presentRaw: Servos.gripper.presentRaw - 1 });
  Assert.deepEqual(Run(Short, Closed).Verdict.issues, [], 'one raw count short of the plan\'s fraction');
  const Under = WithGripper(Servos, Plan.OpenFraction - 0.1);
  const UnderStep = Run(Under, Closed);
  Assert.deepEqual(UnderStep.Motion, { direct_servo_targets: [{ name: 'gripper', target_fraction: Plan.OpenFraction }] });
  const Descent = Model(Plan.Grasp, Plan.Pitch);
  Assert.match(Api.ClosedJawDescentReason(Descent, -6, Box, { GripperFraction: Plan.OpenFraction - 0.1, Closed: false }, Plan),
    new RegExp(`less than the ${Math.round(Plan.OpenFraction * 100)}% the GRASP PLAN was checked at: open it before descending onto the box: "motion_proposal":\\{"direct_servo_targets":\\[\\{"name":"gripper","target_fraction":${Plan.OpenFraction.toFixed(1)}\\}\\]\\}`));
  // Wider than the plan's fraction beside the box: rise first.
  const Wide = WithGripper(At(Servos, Step.Verdict.preview.angles), 0.8);
  Assert.match(Run(Wide, Closed).Line, /wider open than the GRASP PLAN's \d+% beside the box: rise straight up/);
  Servos = At(Servos, Step.Verdict.preview.angles);
  // 4-5. Close and lift, as ONE chained motion: the lift is checked with the jaws closed, before the guard knows they hold it.
  Step = Run(Servos, Closed);
  Assert.deepEqual(Step.Motion, { move_delta_cm: { left: 0, farther: 0, up: 6 }, gripper_before: 0 });
  Assert.equal(Step.Guard_.Held, false);
  Assert.deepEqual(Step.Verdict.issues, []);
  Assert.equal(Step.Verdict.preview.angles.gripper, Api.currentAngles(WithGripper(Servos, 0)).gripper, 'the lift is solved with the jaws closed');
  // The same lift with the jaws closing after it, or sideways after the close: refused.
  const Grasped = { Object: Box, State: { GripperFraction: Fraction(Servos), Closed }, Plan };
  Assert.match(Api.Validate(Parse({ move_delta_cm: { left: 0, farther: 0, up: 6 }, gripper_after: 0 }, Servos), 'up', Servos, [], undefined, Grasped, Step.Guard_).issues.join(' '),
    /A close is never chained after a tip move/);
  Assert.match(Api.Validate(Parse({ move_delta_cm: { left: 2, farther: 0, up: 6 }, gripper_before: 0 }, Servos), 'up', Servos, [], undefined, Grasped, Step.Guard_).issues.join(' '),
    /After a close the only tip move that may be chained is a straight lift/);
  Closed = Api.GripperClosedAfter(Plan.OpenFraction, 0, Closed);
  // Closed alone (or the chain's lift not sent), the jaws stopped on it at 30%: the plain lift validates, the guard holding it.
  Step = Run(WithGripper(Servos, 0.3), Closed);
  Assert.deepEqual(Step.Motion, { move_delta_cm: { left: 0, farther: 0, up: 6 } });
  Assert.equal(Step.Guard_.Held, true);
  Assert.deepEqual(Step.Verdict.issues, []);
  // The chain's lift checked again before it is sent (App.tsx chainLegIssues): from the live pose, the jaws where they
  // stopped, and the guard holding the box once they stopped on it, as the check-in's guard has it. Stopped at 52% (a box
  // a little wider than measured), the open moving jaw is under 1 cm outside it: the proposal's guard, which does not
  // know it is held, would refuse the lift that the plain step allows.
  const Lift = { ...Parse({ move_delta_cm: { left: 0, farther: 0, up: 6 }, gripper_before: 0 }, Servos), chain: null, servoTargets: [] };
  for (const Stopped of [0.3, 0.52]) {
    const Live = WithGripper(Servos, Stopped);
    const HeldGuard = Api.BuildGraspGuard([Box], Task, Closed && Fraction(Live) >= Api.HeldFraction);
    Assert.equal(HeldGuard.Held, true);
    Assert.deepEqual(Api.Validate(Lift, 'up', Live, [], undefined, { Object: Box, State: { GripperFraction: Stopped, Closed }, Plan }, HeldGuard).issues, [], `${Stopped}`);
  }
  Assert.match(Api.Validate(Lift, 'up', WithGripper(Servos, 0.52), [], undefined, { Object: Box, State: { GripperFraction: 0.52, Closed }, Plan },
    Api.BuildGraspGuard([Box], Task, false)).issues.join(' '), /open moving jaw would come within/);
  // The jaws stopped on it (30%) and the tip rose 6 cm: held, lifted, done.
  Servos = At(WithGripper(Servos, 0.3), Api.solveIkPreview(Model({ ...TipOf(Servos), z: TipOf(Servos).z + 6 }, Api.wristPitchFromAngles(Api.currentAngles(Servos))), 'up', WithGripper(Servos, 0.3)).angles);
  Assert.equal(Api.BuildGraspGuard([Box], Task, Closed && Fraction(Servos) >= Api.HeldFraction).Held, true);
  Assert.match(Api.TaskObjectLine(Task, TipOf(Servos), { GripperFraction: Fraction(Servos), Closed }), /it is lifted\. If the image shows it in the jaws, reply task_status "done"/);
});

Test('a held object is put down and released with a chain: the rules that keep the open jaws off the task object are not about the object they hold', () => {
  const Task = State(Usb, UsbReach);
  // Held at the grasp point (the jaws stopped at 25%), then lifted 6 cm.
  const AtGrasp = WithGripper(At(Home, Solve(UsbPlan.Grasp, UsbPlan.Pitch, Home).angles), 0.25);
  const Lifted = At(AtGrasp, Solve({ ...UsbPlan.Grasp, z: UsbPlan.Grasp.z + 6 }, UsbPlan.Pitch, AtGrasp).angles);
  const Reply = Motion => JSON.stringify({ response_to_user: 'x', task_status: 'continue', motion_proposal: Motion });
  const Anchor = Servos => ({ ...Api.forwardKinematics(Api.currentAngles(Servos)).at(-1), wristPitch: Api.wristPitchFromAngles(Api.currentAngles(Servos)) });
  const Issues = (Motion, Servos, Held) => {
    const Parsed = Api.ParseResponse(Reply(Motion), Anchor(Servos), Servos).motion;
    Assert.ok(Parsed.chain, JSON.stringify(Motion));
    return Api.Validate(Parsed, 'up', Servos, [], undefined, { Object: Usb, State: { GripperFraction: 0.25, Closed: true }, Plan: UsbPlan },
      Api.BuildGraspGuard([Usb], Task, Held)).issues;
  };
  const PutBack = { move_delta_cm: { left: 0, farther: 0, up: -6 }, gripper_after: 0.8 };
  const PutAside = { move_delta_cm: { left: -3, farther: 0, up: -6 }, gripper_after: 0.8 };
  const ReleaseAndLift = { target_cm: UsbPlan.Hover, wrist_pitch_degrees: UsbPlan.HoverPitch, gripper_before: 0.8 };
  // Held: put back where it was and let go there, 3 cm aside, or let go at the desk and rise to the hover point; each as one chain.
  for (const [Motion, Servos] of [[PutBack, Lifted], [PutAside, Lifted], [ReleaseAndLift, AtGrasp]]) {
    Assert.deepEqual(Issues(Motion, Servos, true), [], JSON.stringify(Motion));
  }
  // Not held (the jaws missed it), the same chains are the descent beside it and the jaws opening beside it: refused.
  Assert.match(Issues(PutBack, Lifted, false).join(' '), /goes down beside the usb drive[^\n]*the descent is never chained with the gripper/);
  Assert.match(Issues(ReleaseAndLift, AtGrasp, false).join(' '), /The gripper would open beside the usb drive below its top/);
});

Test('a chained motion\'s legs are each checked with the gripper as it will be then: the move with the jaws its gripper action leaves, the gripper action where the arm is when it runs', () => {
  const Closed = WithGripper(Home, 0.05);
  const Start = TipOf(Closed), Pitch = Api.wristPitchFromAngles(Api.currentAngles(Closed));
  const End = { x: Start.x + 4, y: Start.y + 4, z: Start.z - 2 };
  const EndAngles = Solve(End, Pitch, Closed).angles;
  const OpenDeg = Api.angleFromRawValue(Closed.gripper, Api.fractionToRaw(Closed.gripper, 0.8)), ClosedDeg = Api.currentAngles(Closed).gripper;
  // A 1 cm box where the jaw opened to 80% would be at a pose, as far as it gets from the closed gripper there.
  const OpenJawSpot = Angles => {
    const Rest = Api.GripperSamples({ ...Angles, gripper: ClosedDeg }, [ClosedDeg, ClosedDeg - Rules.JawUncertaintyDeg]).map(S => S.P);
    const Gap = P => Math.min(...Rest.map(Q => Math.hypot(P.x - Q.x, P.y - Q.y, P.z - Q.z)));
    const Spot = Api.GripperSamples({ ...Angles, gripper: OpenDeg }, [OpenDeg]).filter(S => S.Part === 'MovingJaw').map(S => S.P)
      .reduce((Best, P) => (Gap(P) > Gap(Best) ? P : Best));
    Assert.ok(Gap(Spot) >= 1.5, `the open jaw is only ${Gap(Spot).toFixed(2)} cm from the closed gripper`);
    return Api.ObjectPrism(Obj('block', Spot.x, Spot.y, Spot.z - 0.5, Spot.z + 0.5, [1, 1]));
  };
  const Check = (Motion, Prism) => Api.Validate(Api.ParseResponse(JSON.stringify({ response_to_user: 'x', task_status: 'continue', motion_proposal: Motion }),
    Model(Start, Pitch), Closed).motion, 'up', Closed, [], undefined, null, Guard([Prism])).issues.join(' | ');
  const Move = { target_cm: End, wrist_pitch_degrees: Number(Pitch.toFixed(1)) };
  // A block where the open jaw would be at the END of the move.
  const AtEnd = OpenJawSpot(EndAngles);
  Assert.equal(Check(Move, AtEnd), '', 'the move alone, with the jaws closed');
  Assert.match(Check({ ...Move, gripper_before: 0.8 }, AtEnd), /gripper's open moving jaw would .* the block/, 'opened first, the move carries the open jaw into it');
  const After = Check({ ...Move, gripper_after: 0.8 }, AtEnd);
  Assert.match(After, /Moving the gripper there would swing the moving jaw .* the block/, 'opened at the end, the jaw swings into it there');
  Assert.doesNotMatch(After, /open moving jaw would/, 'the move itself runs with the jaws closed');
  // A block where the open jaw would be at the START: opening first swings into it; opening at the end is clear.
  const AtStart = OpenJawSpot(Api.currentAngles(Closed));
  Assert.equal(Check(Move, AtStart), '');
  Assert.match(Check({ ...Move, gripper_before: 0.8 }, AtStart), /Moving the gripper there would swing the moving jaw .* the block/);
  Assert.equal(Check({ ...Move, gripper_after: 0.8 }, AtStart), '');
  // A chain's second leg is solved as it was approved (its IK seed) and its path checked from the live joints
  // (App.tsx chainLegIssues): a block on the way from where the arm really is refuses it, though the seed's path is clear.
  const Far = { x: Start.x + 10, y: Start.y + 10, z: Start.z - 4 }, FarAngles = Solve(Far, Pitch, Closed).angles;
  const Seed = At(Closed, Solve({ ...Far, z: Far.z + 1 }, Pitch, Closed).angles);
  const Midway = Api.mirrorDisplayPoint(Api.forwardKinematics(Object.fromEntries(Object.entries(Api.currentAngles(Closed))
    .map(([Name, Angle]) => [Name, Api.IK_JOINTS.includes(Name) ? (Angle + FarAngles[Name]) / 2 : Angle]))).at(-1));
  const OnTheWay = Api.ObjectPrism(Obj('block', Midway.x, Midway.y, Midway.z - 0.5, Midway.z + 0.5, [1, 1]));
  const Leg2 = { ...Api.ParseResponse(JSON.stringify({ response_to_user: 'x', task_status: 'continue', motion_proposal: { target_cm: Far, wrist_pitch_degrees: Number(Pitch.toFixed(1)) } }),
    Model(TipOf(Seed), Pitch), Seed).motion, chain: null, servoTargets: [] };
  Assert.deepEqual(Api.Validate(Leg2, 'up', Seed, [], undefined, null, Guard([OnTheWay])).issues, [], 'from the seed the path is clear');
  Assert.match(Api.Validate(Leg2, 'up', Seed, [], undefined, null, Guard([OnTheWay]), Closed).issues.join(' | '), /the block/, 'from the live joints it is not');
});

Test('the plan checks the moving jaw\'s swing as it closes against what is beside the object, as the guard does before the close runs', () => {
  // A bottle (measured 5.0 x 4.5, so up to 6.7 cm across) 7 cm toward -X of the recorded can: where the closing jaw swings.
  for (const Dx of [7, 8, 9, 10]) {
    const BottleBy = Obj('bottle', Can2.x - Dx, Can2.y, -2.0, 12, [5.0, 4.5]);
    const Before = Api.SolveCounter.Count;
    const Reach_ = Reach(Can2, [BottleBy]);
    // Its work is bounded (2026-10-08 review: the best-effort pass after the two refusing passes ran to its 60 s cap,
    // 9663 to 9757 IK solves a call; bounded by its work 5552 to 5956; the old 6 s budget made 5520 to 5771).
    // GRASP ANYTHING (2026-10-18): with no plan found the pass searches the rest of the object's height on, up to BestEffortNoPlanWork (2) times its work more.
    console.log(`${Dx} cm: ${Api.SolveCounter.Count - Before} IK solves, ${Reach_.Plan ? 'planned' : Reach_.Refusal.Code}`);
    Assert.ok(Api.SolveCounter.Count - Before <= 6500 * (1 + Rules.BestEffortNoPlanWork), `${Dx} cm: ${Api.SolveCounter.Count - Before} IK solves`);
    if (!Reach_.Plan) {
      Assert.ok(['neighbour', 'path'].includes(Reach_.Refusal.Code) && /bottle/.test(Reach_.Text), Reach_.Text);
      continue;
    }
    // Planned: the close the guard checks at the grasp point is allowed.
    const AtGrasp = WithGripper(At(Home, Solve(Reach_.Plan.Grasp, Reach_.Plan.Pitch, Home).angles), Reach_.Plan.OpenFraction);
    const Guard_ = Api.BuildGraspGuard([Can2, BottleBy], State(Can2, Reach_), false);
    Assert.equal(Api.GripperSweepReason(Api.currentAngles(AtGrasp), Api.angleFromRawValue(Home.gripper, Api.fractionToRaw(Home.gripper, 0)), Guard_), null, `${Dx} cm`);
  }
  // Where the closing swing meets it, the refusal says so.
  const Close = Reach(Can2, [Obj('bottle', Can2.x - 7, Can2.y, -2.0, 12, [5.0, 4.5])]);
  Assert.match(Close.Text, /bottle/);
});

Test('a round neighbour is held at its widest too: a can beside the object is 6.7 cm across to the planner and the guard', () => {
  const Box = Obj('box', -5, 33, -2.3, -0.3, [2, 2]);
  for (const Dx of [5, 6]) {
    const Neighbour = Obj('can', Box.x - Dx, Box.y, -1.97, 10.16, [5.0, 4.5]);
    const Reach_ = Reach(Box, [Neighbour]);
    const Guard_ = Api.BuildGraspGuard([Box, Neighbour], State(Box, Reach_), false);
    Assert.equal(Guard_.Prisms.find(P => P.Label === 'can').Round.HiCm, 6.67);
    // Refused for the can: at the grasp point, or (since the CAD-exact contact model, v8, frees the pitches the basket's
    // tracking envelope refused) on the way down to it.
    if (!Reach_.Plan) { Assert.ok(['neighbour', 'path'].includes(Reach_.Refusal.Code) && /of the can\b/.test(Reach_.Text), Reach_.Text); continue; }
    // Planned: the descent keeps the guard's margin from a true 6.6 cm can there.
    const TrueCan = { ...Api.ObjectPrism(Neighbour), Footprint: DiscAround({ x: Neighbour.x, y: Neighbour.y }, 6.6) };
    const Open = Api.angleFromRawValue(Home.gripper, Api.fractionToRaw(Home.gripper, Reach_.Plan.OpenFraction));
    const H = { ...Solve(Reach_.Plan.Hover, Reach_.Plan.HoverPitch, Home).angles, gripper: Open }, G = { ...Solve(Reach_.Plan.Grasp, Reach_.Plan.Pitch, Home).angles, gripper: Open };
    Assert.equal(Api.PathContact(H, G, Open, [TrueCan], Rules.ContactMarginCm), null);
  }
});

Test('the printed points keep the snap\'s extra basket clearance, so the bridge\'s check of the rounded pose agrees', () => {
  let Planned = 0;
  for (const [X, Y] of [[-8, 28], [0, 30], [10, 32]]) {
    const Reach_ = Reach(Obj('can', X, Y, -1.97, 10.16, [5.0, 4.5], { supportZMeasured: -1.77 }));
    if (!Reach_.Plan) continue;
    Planned++;
    for (const [Point, Pitch] of [[Reach_.Plan.Grasp, Reach_.Plan.Pitch], [Reach_.Plan.Hover, Reach_.Plan.HoverPitch]]) {
      Assert.equal(Api.BasketPoseReason(Solve(Point, Pitch, Home).angles, Rules.BasketExtraCm), null, `(${X}, ${Y})`);
    }
  }
  Assert.ok(Planned >= 1);
});

Test('the preferred wrist pitch (-60 deg, steep so the camera sees the object): planned there when every check passes, else at the nearest pitch that passes, equally near the steeper; not the steepest', () => {
  Assert.equal(Rules.PreferredPitchDeg, -60);
  // (Before: the steepest-first order of 2026-10-02, with the fixed jaw's gap of then too: no extra standoff.)
  const Before = Make => { const Saved = Rules.PreferredPitchDeg; Rules.PreferredPitchDeg = null; try { return WithExtra(0, Make); } finally { Rules.PreferredPitchDeg = Saved; } };
  // Near the arm (the centre reached at -68 to -74): -60, where the steepest-first order planned them at -66 to -73.
  for (const [Name, Object_, Old] of [['eraser 1.2', Eraser12, -66], ['pen', Pen, -66.4], ['box at (-10, 32)', Obj('box', -10, 32, -2.3, -0.3, [2, 2]), -72.6]]) {
    Assert.equal(Reach(Object_).Plan.Pitch, -60, Name);
    Assert.equal(Before(() => Reach(Object_).Plan.Pitch), Old, `${Name} before`);
  }
  // By the basket at (-15, 30) (v7: -62.5, the gripper link's tracking envelope at the basket's rim at -57.5) the CAD-exact
  // contact model (v8) plans it at -60. Nearer the cart, at (-17.5, 27), at -60 and -57.5 the gripper link's CAD box
  // reaches the rim: -67, the nearest that passes every check (the steepest-first order went to -89.5).
  Assert.equal(Reach(Obj('box', -15, 30, -2.3, -0.3, [2, 2])).Plan.Pitch, -60);
  const ByBasket = Obj('box', -17.5, 27, -2.3, -0.3, [2, 2]), Prism = Api.GraspPrism(ByBasket);
  Assert.equal(Reach(ByBasket).Plan.Pitch, -67);
  // (Placed with the fixed jaw's gap as before, GraspRules.FixedJawExtraStandoffCm 0: the jaw table's plan there sets the wrist;
  // with the full 2.54 cm no wrist plans it by the strict closing, and at -67 it then fits 2 cm more by the jaw-plane sweep.)
  const ByBasketPlan = Reach(ByBasket).Plan;
  Assert.deepEqual([ByBasketPlan.ExtraStandoffCm, ByBasketPlan.Closing.Mode], [2, 'sweep']);
  for (const Pitch of [-60, -57.5]) Assert.match(Api.PlaceAtPitch(Prism, Pitch, { ...Context(), ExtraStandoffCm: 0 }).Why, /gripper link could contact the basket/);
  Assert.equal(Before(() => Reach(ByBasket).Plan.Pitch), -89.5);
  // Farther out the arm reaches only flatter: the steepest it reaches, as before (the box at (0, 40): -17.9).
  Assert.equal(Reach(Obj('box', 0, 40, -2.3, -0.3, [2, 2])).Plan.Pitch, -17.9);
  // Equally near -60, the steeper first.
  Assert.ok(Api.PreferredFirst(-62.5, -57.5) < 0 && Api.PreferredFirst(-57.5, -62.5) > 0 && Api.PreferredFirst(-59, -62.5) < 0);
});

Test('reach: 5 cm closer the can is planned; 46 cm out it is OUT OF REACH with the cm closer the app planned it, and planned there', () => {
  const Closer = Reach(CanAt(32.2));
  Assert.equal(Closer.Kind, 'graspable', Closer.Text);
  Assert.equal(Closer.Plan.Closing.Mode, 'sweep');
  const Far = Reach(CanAt(46));
  Assert.equal(Far.Kind, 'out-of-reach', Far.Text);
  Assert.ok(Number.isInteger(Far.CloserCm) && Far.CloserCm >= 1 && Far.CloserCm <= 6, JSON.stringify(Far.CloserCm));
  // (Since 2026-10-22 the planner's own search runs before it is called out of reach, and the verdict says what it found.)
  Assert.match(Far.Text, new RegExp(`^OUT OF REACH \\(46\\.0 cm from the right arm's pan axis; the closest pose misses by \\d\\.\\d cm; the grasp search, its grasp point through the middle of it or beside it, found none either: every wrist pitch and grasp height tried .*the arm does not reach a grasp point beside it at wrist [^;]*\\); about ${Far.CloserCm} cm closer to the arm \\(its centre about ${46 - Far.CloserCm} cm from the pan axis\\) the app plans the grasp$`));
  Assert.equal(Far.Advice, `Move the can about ${Far.CloserCm} cm closer to the right arm (straight toward its shoulder), then send the task again.`);
  Assert.equal(Reach(CanAt(46 - Far.CloserCm)).Kind, 'graspable', 'planned where the advice puts it');
  // (A cm short of it may plan too since the deep grasp, 2026-10-04: a head start the arm does not reach is shortened to one it
  // does, and the search starts at the miss distance; the best-effort pass may plan it anyway, warned.)
  // Without the grasp option the out-of-reach verdict is today's, word for word.
  Assert.equal(Api.TaskObjectReachability(CanAt(46), Context()).Text.endsWith('cm)'), true);
});

Test('a round object measures narrower than it is: planned from its larger side to the widest it may be, a drink can as one, small ones as M1 planned them', () => {
  Assert.equal(Api.RoundObject(Can2), true);
  for (const Label of ['bottle', 'cup', 'spray can', 'mug']) Assert.equal(Api.RoundObject({ ...Can2, label: Label }), true, Label);
  Assert.equal(Api.RoundObject({ ...Can2, label: 'box' }), false, 'a box: its corners are not a disc');
  Assert.equal(Api.RoundObject({ ...Can2, sizeCm: [12.1, 6.6], topZ: Can2.z + 6.6, heightCm: 6.6 }), false, 'a can lying down');
  Assert.equal(Api.RoundObject({ ...Can2, topZ: Can2.z + 3, heightCm: 3 }), false, 'a flat tin');
  // The can partly hidden by the arm at 23:50:21 read 4.3 x 2.3: still a standing can (its height says so), planned as one.
  Assert.equal(Api.RoundObject(Can3), true);
  Assert.deepEqual(Api.GraspPrism(Can3).Round, { LoCm: 4.3, HiCm: 6.6, NominalCm: 5.73 });
  // The widest each may be: a disc's top reads about 0.75 of its diameter; a "can" is a drink can (6.6 to 6.8);
  // a small one keeps its padded box (a 2 cm tube: 3 cm, as M1 planned it).
  for (const [Label, Size, Hi] of [['can', [5.0, 4.5], 6.67], ['can', [5.6, 5.0], 6.8], ['can', [4.0, 3.9], 6.6], ['can', [6.6, 6.6], 6.8],
    ['spray can', [4.9, 2.2], 6.6], ['bottle', [3.0, 2.5], 4.0], ['bottle', [5.0, 4.5], 6.67], ['tube', [2, 2], 3.0], ['cup', [7.0, 6.5], 9.33]]) {
    Assert.equal(Api.RoundSizes(Label, Size).HiCm, Hi, `${Label} ${Size}`);
  }
  // The can's prism is the 6.67 cm disc alone: it holds the measured box both ways, and no box corner pushes the fixed pad out.
  const Prism = Api.GraspPrism(Can2);
  for (const P of DiscAround(Prism.Centre, 6.66)) Assert.ok(Api.FootprintDistance(Prism.Footprint, P) <= 1e-6);
  Assert.ok(Prism.Footprint.every(P => Math.hypot(P.x - Prism.Centre.x, P.y - Prism.Centre.y) <= 6.67 / 2 / Math.cos(Math.PI / 48) + 1e-6));
  Assert.equal(Api.ObjectPrism(Can2).Round, undefined);
  Assert.equal(Api.GuardPrisms([Can2], Can2, null, null).TaskPrism.Round.HiCm, 6.67, 'the guard holds the task can at its widest without a plan too');
  // Small round objects keep M1's plan: a 2 cm tube and a 2.5 cm bottle on the jaw table at 80% open (master: wrist -56.3 and -57.7).
  for (const [Object_, Pitch] of [[Obj('tube', 2, 31, -2.3, 6.7, [2, 2]), -56.3], [Obj('bottle', -5, 33, -2.3, 7.7, [2.5, 2.5]), -57.7]]) {
    const Plan = Reach(Object_).Plan;
    Assert.ok(Plan, Object_.label);
    Assert.deepEqual([Plan.Closing.Mode, Plan.OpenFraction, Plan.Pitch], ['table', 0.8, Pitch], Object_.label);
  }
  // Every reading of the real can is planned: the 05:19 attempt (5.6 x 5.0 at X -0.7, Y 30.4), and 4.0 to 6.6 at the 16:49 spot;
  // the fixed pad c outside the widest each time, the grip held.
  for (const Object_ of [Obj('can', -0.68, 30.39, -2.3, 10.77, [5.6, 5.0], { supportZMeasured: -2.03 }), CanAt(37.2, [4.0, 3.9]), CanAt(37.2, [5.7, 5.4]), CanAt(37.2, [6.6, 6.6])]) {
    const Reach_ = Reach(Object_);
    Assert.equal(Reach_.Kind, 'graspable', `${Object_.sizeCm}: ${Reach_.Text}`);
    const Plan = Reach_.Plan;
    Assert.ok(PadOutside(Plan, DiscAround(Plan.Prism.Centre, Plan.Prism.Round.HiCm)) >= Rules.FixedJawClearanceCm - 0.08, `${Object_.sizeCm}`);
    Assert.ok(Plan.Closing.GripDeg <= 20, `${Object_.sizeCm}: ${JSON.stringify(Plan.Closing)}`);
  }
  // The same can labelled a box is planned as the box it measured.
  const Boxed = Reach({ ...Can2, label: 'box' });
  Assert.ok(!Boxed.Plan?.Prism.Round, Boxed.Text);
});

Test('wide boxes: a standing 4 x 2 x 8 box is planned with the sweep, a 4 x 4 x 10 one is refused for the grip, the 5 x 2 eraser is told to turn', () => {
  const Tall = Reach(Obj('box', -5, 33, -2.3, 5.7, [4, 2]));
  Assert.equal(Tall.Kind, 'graspable', Tall.Text);
  Assert.equal(Tall.Plan.Closing.Mode, 'sweep');
  Assert.ok(Tall.Plan.WidthEffCm > 5.3, 'the jaw table refuses it');
  // Refused by the strict passes; planned best-effort, the grip warned.
  Assert.ok(Reach(Obj('box', -5, 33, -2.3, 7.7, [4, 4])).Plan.Warnings.some(Why => /^the grip may slip/.test(Why)));
  const Wide = Strictly(() => Reach(Obj('box', -5, 33, -2.3, 7.7, [4, 4])));
  Assert.equal(Wide.Refusal.Code, 'wide');
  Assert.match(Wide.Text, /the jaws do not hold it either \(at best, at wrist -?[\d.]+ deg [^()]*closing jaws would need friction for \d+ deg at a pad \(at most 20\) and squeeze it out past the finger tips\)/);
  // Low by the basket rim: at the flat wrists that would hold it the basket's tracking envelope was in the way (v7), and
  // where the arm reached beside it the closing would squeeze it out: refused. Since the CAD-exact contact model (v8) the
  // flat wrists clear the rim: planned with the sweep, and never called out of reach.
  const Low = Reach(Obj('box', -5, 33, -2.3, 1.7, [4, 2]));
  Assert.equal(Low.Kind, 'graspable', Low.Text);
  Assert.equal(Low.Plan.Closing.Mode, 'sweep');
  Assert.doesNotMatch(Low.Text, /OUT OF REACH/);
});

Test('the tip tape at the hover point moves the plan along the jaw axis, at most 0.5 cm toward the object, which stays where the camera measured it', () => {
  const U = CanPlan.JawAxis;
  // Seen 0.8 cm nearer the can than the arm model puts it: the grasp point (and the hover) move 0.8 cm away; the can does not move.
  const Near = Api.TapeCorrection(CanPlan, { x: 0.8 * U.x, y: 0.8 * U.y, z: 0.3 }, null);
  Assert.ok(Math.abs(Near.Fix.AlongCm - 0.8) < 1e-9 && Math.abs(Near.Fix.ShiftCm + 0.8) < 1e-9 && !Near.Fix.Clamped);
  const Moved = Near.Plan;
  Assert.ok(Math.abs(Dot(Moved.Grasp, U) - Dot(CanPlan.Grasp, U) + 0.8) <= 0.08, 'along u');
  Assert.ok(Math.abs(Dot(Moved.Grasp, { x: -U.y, y: U.x }) - Dot(CanPlan.Grasp, { x: -U.y, y: U.x })) <= 0.08, 'not across');
  Assert.deepEqual([Moved.Hover.x, Moved.Hover.y, Moved.Hover.z, Moved.Grasp.z], [Moved.Grasp.x, Moved.Grasp.y, CanPlan.Hover.z, CanPlan.Grasp.z]);
  Assert.equal(Moved.Prism, CanPlan.Prism, 'the can stays where the camera measured it');
  Assert.match(Api.TapeText(Near.Fix, 'can'), /^the camera saw the tip tape 0\.8 cm nearer the can than the arm model puts it, so the GRASP POINT moved 0\.8 cm away from it$/);
  // Seen 3 cm farther: 0.5 cm toward it at most (c is 1.0, the guard keeps 0.4); 1.5 cm nearer: 1.0 away at most; 0.1 cm, or across the jaws: it stays.
  const Far = Api.TapeCorrection(CanPlan, { x: -3 * U.x, y: -3 * U.y, z: 0 }, null);
  Assert.deepEqual([Far.Fix.ShiftCm, Far.Fix.Clamped], [0.5, true]);
  Assert.match(Api.TapeText(Far.Fix, 'can'), /farther from the can .* moved 0\.5 cm toward it \(the most the app moves it that way; the rest is 2\.5 cm\)/);
  const Away = Api.TapeCorrection(CanPlan, { x: 1.5 * U.x, y: 1.5 * U.y, z: 0 }, null);
  Assert.deepEqual([Away.Fix.ShiftCm, Away.Fix.Clamped], [-1.0, true]);
  Assert.equal(Api.TapeCorrection(CanPlan, { x: 0.1 * U.x, y: 0.1 * U.y, z: 0 }, null).Fix.ShiftCm, 0);
  Assert.equal(Api.TapeCorrection(CanPlan, { x: -U.y, y: U.x, z: 0 }, null).Fix.ShiftCm, 0);
  // Checked: a box just beyond the fixed finger refuses a move into it, and says why.
  const Behind = Obj('box', CanPlan.Grasp.x - 2.0 * U.x, CanPlan.Grasp.y - 2.0 * U.y, -2.0, 10.0, [1.0, 1.0]);
  const Blocked = Api.TapeCorrection(CanPlan, { x: 1.0 * U.x, y: 1.0 * U.y, z: 0 }, { ...Context(), Others: [Api.ObjectPrism(Behind)] });
  Assert.equal(Blocked.Plan, null);
  Assert.match(Blocked.Why, /the fixed finger would come within .* of the box/);
  // The most toward the can, checked: the guard on the can where it was measured lets the moved descent through.
  const Toward = Api.TapeCorrection(CanPlan, { x: -3 * U.x, y: -3 * U.y, z: 0 }, Context());
  Assert.ok(Toward.Plan, Toward.Why);
  const TowardGuard = Api.BuildGraspGuard([Can2], State(Can2, { ...CanReach, Plan: Toward.Plan }), false);
  Assert.equal(TowardGuard.TaskPrism.Centre, CanPlan.Prism.Centre);
  const AtMovedHover = At(Home, Solve(Toward.Plan.Hover, Toward.Plan.HoverPitch, Home).angles);
  const MovedGrasp = Model(Toward.Plan.Grasp, Toward.Plan.Pitch);
  Assert.equal(Api.GraspGuardReason(MovedGrasp, Api.solveIkPreview(MovedGrasp, 'up', AtMovedHover).angles, Api.currentAngles(AtMovedHover),
    Toward.Plan.Grasp.z - Toward.Plan.Hover.z, TowardGuard), null);
  // At the hover check-in, with the tape judged: the verdict moves once, and the line says why.
  const Check = (Residual, TapeCheck = 'ok') => ({ Ok: true, AngleDeg: 0, OffsetCm: 0, Reason: '', ReferenceFittedAt: null, Seq: 1, TapeCheck, TapeResidualCm: Residual });
  const Residual = { x: 0.8 * U.x, y: 0.8 * U.y, z: 0.3 };
  const Ctx = { Servos: Home, ElbowMode: 'up' };
  Assert.equal(Api.TapeCorrectedReach(CanReach, Check(Residual), { x: -13.3, y: 20.1, z: 12.4 }, Ctx, [], 'can'), null, 'not at the hover point');
  Assert.equal(Api.TapeCorrectedReach(CanReach, Check(Residual, 'pose_differs'), CanPlan.Hover, Ctx, [], 'can'), null, 'the tape not judged');
  const Fixed = Api.TapeCorrectedReach(CanReach, Check(Residual), CanPlan.Hover, Ctx, [], 'can');
  Assert.ok(Fixed && Fixed.Moved);
  const GraspZ = CanPlan.Grasp.z.toFixed(1).replace('.', '\\.');
  Assert.match(Fixed.Note, new RegExp(`^the camera saw the tip tape 0\\.8 cm nearer the can than the arm model puts it, so the GRASP POINT moved 0\\.8 cm away from it \\(now X -?[\\d.]+, Y [\\d.]+, Z ${GraspZ}\\)$`));
  Assert.match(Fixed.Reach.Text, new RegExp(`^graspable: GRASP POINT X ${Fixed.Reach.Plan.Grasp.x.toFixed(1)}, Y ${Fixed.Reach.Plan.Grasp.y.toFixed(1)}, Z ${GraspZ}`));
  Assert.equal(Api.TapeCorrectedReach(Fixed.Reach, Check(Residual), CanPlan.Hover, Ctx, [], 'can'), null, 'once per plan');
  // The bridge judges the tape with the gripper closed (its tape pose), so on the arm it fires after a missed close, back at the
  // hover point: the steps then open the gripper to the plan's fraction, go over the moved hover point and down to the moved grasp point.
  const Task = State(Can2, Fixed.Reach);
  Task.Plan.Note = Fixed.Note;
  const Plan2 = Fixed.Reach.Plan;
  const Missed = Api.TaskObjectLine(State(Can2, CanReach), CanPlan.Grasp, { GripperFraction: 0.034, Closed: true });
  Assert.deepEqual(MotionIn(Missed), { move_delta_cm: { left: 0, farther: 0, up: Number((CanPlan.Hover.z - CanPlan.Grasp.z).toFixed(1)) } });
  Assert.deepEqual(MotionIn(Api.TaskObjectLine(Task, CanPlan.Hover, { GripperFraction: 0.034, Closed: true })), { direct_servo_targets: [{ name: 'gripper', target_fraction: Plan2.OpenFraction }] });
  const Line = Api.TaskObjectLine(Task, CanPlan.Hover, { GripperFraction: 0.8, Closed: false });
  Assert.match(Line, /\(the camera saw the tip tape 0\.8 cm nearer the can than the arm model puts it, so the GRASP POINT moved 0\.8 cm away from it \(now X/);
  // (A coordinate rounded to -0 reads back from the JSON as 0.)
  Assert.deepEqual(MotionIn(Line), { target_cm: { x: Plan2.Hover.x || 0, y: Plan2.Hover.y || 0, z: Plan2.Hover.z || 0 }, wrist_pitch_degrees: Plan2.HoverPitch });
  const Step = MotionIn(Api.TaskObjectLine(Task, Plan2.Hover, { GripperFraction: 0.8, Closed: false }));
  Assert.deepEqual(Step, { move_delta_cm: { left: 0, farther: 0, up: Number((Plan2.Grasp.z - Plan2.Hover.z).toFixed(1)) }, wrist_pitch_degrees: Plan2.Pitch });
  // The guard lets that through, held to the can where it was measured.
  const Guard_ = Api.BuildGraspGuard([Can2], Task, false);
  const AtHover = At(Home, Solve(CanPlan.Hover, CanPlan.HoverPitch, Home).angles);
  const ToHover = Model(Plan2.Hover, Plan2.HoverPitch);
  Assert.equal(Api.GraspGuardReason(ToHover, Api.solveIkPreview(ToHover, 'up', AtHover).angles, Api.currentAngles(AtHover), 0, Guard_), null);
  const AtHover2 = At(Home, Solve(Plan2.Hover, Plan2.HoverPitch, Home).angles);
  const Target = Model(Plan2.Grasp, Plan2.Pitch);
  Assert.equal(Api.GraspGuardReason(Target, Api.solveIkPreview(Target, 'up', AtHover2).angles, Api.currentAngles(AtHover2), Plan2.Grasp.z - Plan2.Hover.z, Guard_), null);
});

Test('the can\'s plan lock: a re-measure is held against the plan\'s own descent on every side; a reading of its true width is not a move', () => {
  const Key = Api.GraspPlanKey(Home, 'up');
  const Previous = { Source: Can2, At: 0, Key, Reach: CanReach };
  const Hover = CanPlan.Hover;
  // At the hover point, the can re-measured 6.0 x 5.5, 6.4 x 6.0 and 6.6 x 6.6 where it was: within the widest it was planned as.
  for (const Size of [[6.0, 5.5], [6.4, 6.0], [6.6, 6.6]]) {
    const Verdict = Api.KeepOrReplan(Previous, { ...Can2, sizeCm: Size }, Hover, false, Key);
    Assert.deepEqual([Verdict.Keep, Verdict.Corridor, Verdict.Confirmed], [true, null, true], `${Size}`);
  }
  // Re-measured 1 cm toward the arm along the approach, the gripper at the hover: the plan's palm, or its open moving jaw (the pad
  // stands the finger offset farther out along u, so the open jaw is that much nearer where the camera measured the can), would
  // meet it. Ask, and the guard holds it.
  const A = Api.JawFrame(CanPlan.Poses.Grasp).A, Level = Math.hypot(A.x, A.y);
  const Back = Cm => ({ ...Can2, x: Can2.x - Cm * A.x / Level, y: Can2.y - Cm * A.y / Level });
  const Palm = Api.KeepOrReplan(Previous, Back(1.0), Hover, false, Key);
  Assert.ok(Palm.Keep && Palm.Corridor && Palm.Confirmed === false, JSON.stringify(Palm));
  Assert.match(Palm.Corridor.Why, /the GRASP PLAN's (palm|open moving jaw) would come within 0\.\d cm of it/);
  const Task = State(Can2, CanReach);
  Object.assign(Task.Plan, { Corridor: Palm.Corridor, Confirmed: false });
  Assert.match(Api.TaskObjectLine(Task, Hover, { GripperFraction: 0.8, Closed: false }), /NEXT STEP of the grasp: reply ask: the can may have moved into the gripper's way \(re-measured with the gripper over it/);
  const Guard_ = Api.BuildGraspGuard([Back(1.0)], Task, false);
  const AtHover = At(Home, Solve(Hover, CanPlan.HoverPitch, Home).angles);
  const Target = Model(CanPlan.Grasp, CanPlan.Pitch);
  Assert.ok(Api.GraspGuardReason(Target, Api.solveIkPreview(Target, 'up', AtHover).angles, Api.currentAngles(AtHover), CanPlan.Grasp.z - Hover.z, Guard_));
  // 1.5 cm toward the finger tips (away from every part): kept, but a 1.5 cm jump does not confirm the plan.
  const Tips = Api.KeepOrReplan(Previous, Back(-1.5), Hover, false, Key);
  Assert.deepEqual([Tips.Keep, Tips.Corridor, Tips.Confirmed], [true, null, false]);
  // The occluded 23:50:21 reading (4.3 x 2.3, its centre 1.7 cm toward the arm): kept, not confirmed. The straddle takes this
  // plan's palm deeper toward the arm (2026-10-03: the jaws around the can), so what it saw is in the palm's way at the grasp
  // point: ask (the 2026-10-01 plan, its palm 0.4 cm clear of it, went on; 0.5 cm nearer the arm it asked too).
  const Occluded = Api.KeepOrReplan(Previous, Can3, Hover, false, Key);
  Assert.ok(Occluded.Keep && Occluded.Corridor && Occluded.Confirmed === false, JSON.stringify(Occluded));
  // (Deep in the jaws, GraspRules.DeepGrasp, its palm is judged by its exact gap: here its open moving jaw meets it first.)
  Assert.match(Occluded.Corridor.Why, /the GRASP PLAN's (palm|open moving jaw) would come within 0\.\d cm of it/);
  const Farther = Api.KeepOrReplan(Previous, { ...Can3, x: Can3.x + 1.5 * A.x / Level, y: Can3.y + 1.5 * A.y / Level }, Hover, false, Key);
  Assert.deepEqual([Farther.Keep, Farther.Corridor, Farther.Confirmed], [true, null, false], `1.5 cm toward the tips: ${JSON.stringify(Farther.Corridor?.Why)}`);
  // A clean re-measure from Home that moved 1.5 cm: planned again.
  Assert.equal(Api.KeepOrReplan(Previous, Back(1.5), TipOf(Home), false, Key).Keep, false);
});

Test('the tape check is read from /locate: where the tape is against where the arm puts it', () => {
  const Read = Check => Api.ReadLocateResponse({ ok: true, objects: [], camera_check: { ok: true, ...Check } }).cameraCheck;
  const Judged = Read({ tape_check: 'ok', tape_error_cm: 0.23, tape_residual_cm: [0.1, -0.2, 0.05] });
  Assert.equal(Judged.TapeCheck, 'ok');
  Assert.deepEqual(Judged.TapeResidualCm, { x: 0.1, y: -0.2, z: 0.05 });
  const Old = Read({});
  Assert.deepEqual([Old.TapeCheck ?? null, Old.TapeResidualCm ?? null], [null, null]);
  Assert.equal('TapeCheck' in Old, false, 'an older bridge\'s reply reads as before');
  Assert.equal(Read({ tape_check: 'pose_differs', tape_residual_cm: [1, 'x', 2] }).TapeResidualCm, null);
});

// --- 2026-10-01 07:56: the arm stopped short of the hover point it held --------------------------

// The recorded servos and measurements (back end/captures/2026-10-01, ai.jsonl events 12-35). The live
// app ran with the elbow 'down' (the bridge's branch at Home): with 'up' this spot is not planned.
const S0755 = Snapshot({ shoulder_pan: 3477, shoulder_lift: 3172, elbow_flex: 2688, wrist_flex: 854, wrist_roll: 1522, gripper: 701 });
const S0756 = Snapshot({ shoulder_pan: 3135, shoulder_lift: 815, elbow_flex: 1078, wrist_flex: 834, wrist_roll: 1521, gripper: 1917 });
const Can0755 = Obj('can', 3.37, 37.94, -1.78, 9.98, [5.3, 5.4], { supportZMeasured: -1.71, validFraction: 0.933, outsideCalibrationCm: 2.8 });
const Around0755 = [Obj('tape', 1.41, 30.28, -2.39, 2.45, [7.7, 4.9], { supportZMeasured: -2.21, refined: false }),
  Obj('bottle', 11.66, 31.51, -2.3, 5.5, [3.6, 2.9], { supportZMeasured: -1.99, refined: false }),
  Obj('cup', -12.49, 31.77, -2.25, -0.05, [8.4, 8.9], { supportZMeasured: -2.24, refined: false })];
// The plan as the app printed it live, before the real-arm corrections (GraspRules FixedFingerOffsetCm and GraspSagCm,
// from the 15:07 run), the straddle and the preferred wrist pitch (2026-10-03): this replay tests the steps on the poses
// recorded around that plan.
const AsLive = Make => {
  // (And no extra standoff for the fixed jaw, GraspRules.FixedJawExtraStandoffCm, 2026-10-25.)
  const Saved = [Rules.FixedFingerOffsetCm, Rules.GraspSagCm, Rules.StrictClosing, Rules.StraddleRoundObjects, Rules.PreferredPitchDeg, Rules.BestEffort, Rules.DeepGrasp, Rules.FixedJawExtraStandoffCm];
  Object.assign(Rules, { FixedFingerOffsetCm: 0, GraspSagCm: 0, StrictClosing: false, StraddleRoundObjects: false, PreferredPitchDeg: null, BestEffort: false, DeepGrasp: false, FixedJawExtraStandoffCm: 0 });
  try { return Make(); } finally { [Rules.FixedFingerOffsetCm, Rules.GraspSagCm, Rules.StrictClosing, Rules.StraddleRoundObjects, Rules.PreferredPitchDeg, Rules.BestEffort, Rules.DeepGrasp, Rules.FixedJawExtraStandoffCm] = Saved; }
};
const Reach0755 = AsLive(() => Api.TaskObjectReachability(Can0755, { Servos: S0755, ElbowMode: 'down' }, { Others: Around0755 }));
const Plan0755 = Reach0755.Plan;
// Hidden under the gripper at the check-in, as live: the plan goes on from its last measurement.
const Task0756 = (Reach_ = Reach0755) => ({ Label: 'can', DetectedNow: false, Binding: null, Candidates: [], Reachability: null, Intent: 'grasp',
  LastSeen: { At: 0, Object: Can0755, Reachability: Reach_ }, Plan: { Reach: Reach_, Source: Can0755, At: 0, Key: Api.GraspPlanKey(S0755, 'down'), Note: null, Confirmed: true } });
const Open0756 = { GripperFraction: 0.7969, Closed: false };

Test('2026-10-01 07:56: holding the hover point 0.7 cm toward the can, the step aims past it instead of repeating it, then descends', () => {
  Assert.equal(Reach0755.Kind, 'graspable', Reach0755.Text);
  const Plan = Plan0755, U = Plan.JawAxis;
  Assert.deepEqual([Plan.Pitch, Plan.Grasp, Plan.Hover, Plan.OpenFraction], [-15.2, { x: 7.7, y: 36.5, z: 4.1 }, { x: 7.7, y: 36.5, z: 13 }, 0.8], 'the plan the app printed live');
  // Planned now, with the real-arm corrections, it is still planned.
  const Corrected = Api.TaskObjectReachability(Can0755, { Servos: S0755, ElbowMode: 'down' }, { Others: Around0755 }).Plan;
  Assert.ok(Corrected && Corrected.FingerOffsetCm === Rules.FixedFingerOffsetCm && Corrected.SagCm === Rules.GraspSagCm, 'planned with the corrections');
  // The recorded check-in: the bridge stops each joint within 12 steps of its target (the pan stopped 10 short), the tip 0.67 cm toward the can.
  const Tip = TipOf(S0756);
  Assert.ok(Math.hypot(Tip.x - 7.0955, Tip.y - 36.798, Tip.z - 11.9971) < 0.01, JSON.stringify(Tip));
  const AlongOf = Point => (Point.x - Plan.Grasp.x) * U.x + (Point.y - Plan.Grasp.y) * U.y;
  Assert.ok(AlongOf(Tip) > Rules.AlignTowardCm + 0.3 && AlongOf(Tip) < 0.7, `${AlongOf(Tip)}`);
  const Task = Task0756();
  // Without the held target (as live): the hover point again, which the arm already held (two no-ops, the task stopped).
  Assert.deepEqual(MotionIn(Api.TaskObjectLine(Task, Tip, Open0756)), { target_cm: Plan.Hover, wrist_pitch_degrees: Plan.HoverPitch });
  const Line = Api.TaskObjectLine(Task, Tip, { ...Open0756, Held: { Point: Plan.Hover, Pitch: Plan.HoverPitch } });
  const Aim = Api.CorrectedHover(Plan);
  Assert.deepEqual(Aim, { x: 8.4, y: 36.1, z: 13 });
  Assert.deepEqual(MotionIn(Line), { target_cm: Aim, wrist_pitch_degrees: Plan.HoverPitch });
  Assert.match(Line, /the arm holds the HOVER POINT but its tip stopped 0\.7 cm toward the can from where the GRASP PLAN puts the fixed jaw .* Do not descend from here: go to the hover point aimed 0\.8 cm past it, away from the can, so the arm stops at it/);
  // Away from the can along the jaws, by the pan's 12 steps at that reach.
  Assert.ok(Math.abs(Api.PanArrivalCm(Plan.Hover) - 41.9 * 12 * 2 * Math.PI / 4096) < 0.05, `${Api.PanArrivalCm(Plan.Hover)}`);
  Assert.ok(AlongOf(Aim) < -0.7 && AlongOf(Aim) > -0.85, `${AlongOf(Aim)}`);
  // It moves the arm (not the held target, by the app's no-op rule), and the guard lets it through from the recorded pose.
  Assert.deepEqual([Api.NoOpCommandedCm, Api.NoOpDeg], [0.5, 2], 'the steps hold a point by the no-op rule\'s numbers');
  const ModelAim = Model(Aim, Plan.HoverPitch), HeldModel = Model(Plan.Hover, Plan.HoverPitch);
  const Live = { Tip: Api.modelPointFromDisplayPoint(Tip), WristPitch: Api.wristPitchFromAngles(Api.currentAngles(S0756)) };
  Assert.equal(Api.NoOpReason({ hasTargetMotion: true, target: ModelAim, servoTargets: [] }, Live, HeldModel, {}), null);
  Assert.match(Api.NoOpReason({ hasTargetMotion: true, target: HeldModel, servoTargets: [] }, Live, HeldModel, {}), /0\.0 cm from the target the arm is already holding/, 'the live repeat');
  const Guard_ = Api.BuildGraspGuard([Can0755, ...Around0755], Task, false);
  const AimAngles = Api.solveIkPreview(ModelAim, 'down', S0756).angles;
  Assert.equal(Api.GraspGuardReason(ModelAim, AimAngles, Api.currentAngles(S0756), 0, Guard_), null);
  // Wherever each joint stops within its 12 steps of the aim (at the edge, half way, on it), the tip is aligned and the step descends.
  for (const Short of [12, 6, 0]) {
    const Stopped = structuredClone(S0756);
    for (const Joint of Api.IK_JOINTS) {
      const Target = Api.rawFromAngle(S0756[Joint], AimAngles[Joint]), Present = S0756[Joint].presentRaw;
      const End = Math.abs(Target - Present) <= 12 ? Present : Target + Math.sign(Present - Target) * Short;
      Object.assign(Stopped[Joint], { presentRaw: End, targetRaw: End });
    }
    const At_ = TipOf(Stopped);
    Assert.ok(AlongOf(At_) <= Rules.AlignTowardCm && AlongOf(At_) >= -Rules.AlignAwayCm, `${Short} short: ${AlongOf(At_)}`);
    const Step = MotionIn(Api.TaskObjectLine(Task, At_, { ...Open0756, Held: { Point: Aim, Pitch: Plan.HoverPitch } }));
    Assert.equal(Step.wrist_pitch_degrees, Plan.Pitch, `${Short} short`);
    Assert.ok(Step.move_delta_cm.up < -7 && Math.abs(Step.move_delta_cm.left) <= 0.8, JSON.stringify(Step));
  }
  // Holding the aim and still toward the can: reply ask, never a third move to the same point.
  Assert.match(Api.TaskObjectLine(Task, Tip, { ...Open0756, Held: { Point: Aim, Pitch: Plan.HoverPitch } }),
    /NEXT STEP of the grasp \(tip now X 7\.1, Y 36\.8, Z 12\.0; gripper 80% open\): reply ask: the arm holds the HOVER POINT aimed past for the arm's stopping error but its tip stopped 0\.7 cm toward the can/);
  // A stale held target (the arm far from it, the can in view) changes nothing.
  const Seen = { ...Task, DetectedNow: true, Binding: { Object: Can0755, Source: 'query', DistanceCm: null }, Reachability: Reach0755 };
  Assert.deepEqual(MotionIn(Api.TaskObjectLine(Seen, TipOf(S0755), { ...Open0756, Held: { Point: Plan.Hover, Pitch: Plan.HoverPitch } })),
    { target_cm: Plan.Hover, wrist_pitch_degrees: Plan.HoverPitch });
  // The no-op feedback repeats the step (the live model copied its first reply, "open the gripper", instead).
  Assert.match(Api.NoOpFeedbackPrompt('gripper is already there', Api.NextStepOf(Line)),
    /\nThe TASK OBJECT line in this message gives the step for the arm as it is now: NEXT STEP of the grasp .*"target_cm":\{"x":8\.4,"y":36\.1,"z":13\.0\},"wrist_pitch_degrees":-15\.2\}\. Copy it exactly .*\nChoose a target/);
  Assert.equal(Api.NoOpFeedbackPrompt('gripper is already there').includes('NEXT STEP'), false);
});

Test('holding the grasp point, a tip the arm stopped off it closes there when that is safe, else the model asks', () => {
  const Plan = Plan0755, Task = Task0756();
  const Line = (Tip, Held) => Api.TaskObjectLine(Task, Tip, { ...Open0756, Held });
  const AtGrasp = { Point: Plan.Grasp, Pitch: Plan.Pitch };
  const Low = { ...Plan.Grasp, z: Plan.Grasp.z - 1.0 }, High = { ...Plan.Grasp, z: Plan.Grasp.z + 1.4 };
  // The close is offered with the straight lift after it, as ONE chained motion (lib/AiChain.ts).
  const Close = { move_delta_cm: { left: 0, farther: 0, up: 6 }, gripper_before: 0 };
  // Not holding it: rise or descend to it, as before.
  Assert.deepEqual(MotionIn(Line(Low, null)), { move_delta_cm: { left: 0, farther: 0, up: 1 } });
  Assert.deepEqual(MotionIn(Line(High, null)), { move_delta_cm: { left: 0, farther: 0, up: -1.4 }, wrist_pitch_degrees: Plan.Pitch });
  // Holding it (the same move would not move the arm): the jaws are around the can's mid-height, close there.
  Assert.deepEqual(MotionIn(Line(Low, AtGrasp)), Close);
  Assert.match(Line(Low, AtGrasp), /the arm holds the GRASP POINT and its tip stopped 1\.0 cm below it/);
  Assert.deepEqual(MotionIn(Line(High, AtGrasp)), Close);
  Assert.match(Line(High, AtGrasp), /stopped 1\.4 cm above it/);
  // Stuck far above it: not the stopping error, the descent as before.
  Assert.deepEqual(MotionIn(Line({ ...Plan.Grasp, z: Plan.Grasp.z + 3 }, AtGrasp)).move_delta_cm, { left: 0, farther: 0, up: -3 });
  // Where the desk set the grasp height, lower than planned is not closed on; where the jaw meets it under its top, higher is not.
  const As = HeightBy => Task0756({ ...Reach0755, Plan: { ...Plan, HeightBy } });
  Assert.match(Api.TaskObjectLine(As('floor'), Low, { ...Open0756, Held: AtGrasp }), /: reply ask: the arm holds the GRASP POINT but its tip stopped 1\.0 cm below it, too near the desk to close/);
  Assert.match(Api.TaskObjectLine(As('jaw'), High, { ...Open0756, Held: AtGrasp }), /: reply ask: the arm holds the GRASP POINT but its tip stopped 1\.4 cm above it, too high to close/);
  Assert.deepEqual(MotionIn(Api.TaskObjectLine(As('floor'), High, { ...Open0756, Held: AtGrasp })), Close);
});

// --- 2026-10-01 07:59-08:01: the can refused as too wide, then for the battery beside it ----------

// Home at 08:00 (ai.jsonl event 71) and the /locate frames (grounding.jsonl, 14:59:24Z and 15:01:40Z).
const S0800 = Snapshot({ shoulder_pan: 3472, shoulder_lift: 3172, elbow_flex: 2688, wrist_flex: 863, wrist_roll: 1521, gripper: 701 });
const Frame0759 = [Obj('can', 0.8, 35.23, -2.03, 9.69, [5.3, 7.7], { supportZMeasured: -1.67, validFraction: 0.868 }),
  Obj('bottle', 11.6, 31.46, -2.29, 5.51, [3.5, 2.8], { supportZMeasured: -1.99, validFraction: 0.912, refined: false, outsideCalibrationCm: 1.3 }),
  Obj('tape', 1.92, 29.15, -2.48, 2.08, [7.6, 5.0], { supportZMeasured: -2.14, validFraction: 0.969, refined: false }),
  Obj('cup', -12.28, 31.87, -2.2, 0.07, [8.4, 8.5], { supportZMeasured: -2.19, validFraction: 0.617, refined: false, outsideCalibrationCm: 1.0 })];
const Frame0801 = [Obj('can', 0.4, 30.28, -2.36, 9.39, [5.5, 5.1], { supportZMeasured: -2.06, validFraction: 0.916 }),
  Obj('bottle', 11.66, 31.46, -2.28, 5.52, [3.5, 2.9], { supportZMeasured: -2.05, validFraction: 0.876, refined: false, outsideCalibrationCm: 1.3 }),
  Obj('battery', 6.68, 28.39, -2.1, 2.65, [4.5, 2.6], { supportZMeasured: -1.99, validFraction: 0.906, refined: false }),
  Obj('tape', 7.25, 26.89, -2.57, 1.46, [7.6, 3.5], { supportZMeasured: -2.26, validFraction: 0.979, refined: false }),
  Obj('cup', -13.8, 31.93, -2.08, 0.02, [8.4, 4.4], { supportZMeasured: -2.19, validFraction: 0.583, refined: false, outsideCalibrationCm: 1.6 }),
  Obj('tape dispenser', 6.9, 26.52, -2.56, 1.15, [7.4, 5.8], { supportZMeasured: -2.24, validFraction: 0.988, refined: false })];
const Frame0800 = [Obj('can', 0.44, 30.24, -2.36, 9.4, [5.4, 5.1], { supportZMeasured: -2.14, validFraction: 0.924 }),
  Obj('bottle', 11.64, 31.48, -2.29, 5.53, [3.4, 2.9], { supportZMeasured: -2.17, validFraction: 0.916, outsideCalibrationCm: 1.3 }),
  Obj('battery', 6.91, 27.98, -2.0, 2.56, [3.8, 2.2], { supportZMeasured: -2.18, validFraction: 0.962 }),
  Obj('tape', 6.69, 28.06, -2.58, 2.31, [6.8, 7.0], { supportZMeasured: -2.23, validFraction: 0.953 }),
  Obj('cup', -11.37, 30.95, -2.25, -0.14, [8.4, 8.5], { supportZMeasured: -2.23, validFraction: 0.556, outsideCalibrationCm: 1.4 })];
const ReachIn = (Frame, Kind = 'can', Elbow = 'down') => {
  const Task = Frame.find(Object_ => Object_.label === 'can');
  return Api.TaskObjectReachability(Task, { Servos: S0800, ElbowMode: Elbow }, { Others: Api.NeighbourObjects(Frame, Task), Kind });
};

Test('2026-10-01 07:59: a can read 5.3 x 7.7 (the tape roll in front measured with it) is planned as a drink can where its top was, not refused at 10.3 cm', () => {
  // The rule: a can's wide side over 6.8 with its other side a can's is a merged reading; the generic round rule made it 10.3 and refused it.
  Assert.deepEqual(Api.RoundSizes('can', [5.3, 7.7]), { LoCm: 5.3, HiCm: 6.8, NominalCm: 6.8, MergedCm: 7.7 });
  Assert.equal(Api.RoundSizes('tin', [5.3, 7.7]).HiCm, 10.27, 'a tin may be that wide: as measured');
  Assert.equal(Api.RoundSizes('can', [7.7, 7.4]).HiCm, 10.27, 'wide both ways: not a drink can');
  Assert.equal(Api.RoundSizes('can', [5.3, 13.2]).HiCm, 17.6, 'two cans side by side: as measured');
  for (const Label of ['soda can', 'red can', 'soda', 'coke']) Assert.equal(Api.RoundSizes(Label, [5.3, 7.7]).MergedCm, 7.7, Label);
  const Reach_ = ReachIn(Frame0759);
  Assert.equal(Reach_.Kind, 'graspable', Reach_.Text);
  const Plan = Reach_.Plan, Can = Frame0759[0];
  Assert.equal(Plan.Prism.Round.MergedCm, 7.7);
  // The 6.8 cm disc on the measured (top) centre; the tape roll in front is its own prism, which the plan clears.
  Assert.deepEqual(Plan.Prism.Centre, { x: Can.x, y: Can.y });
  Assert.ok(PadOutside(Plan, DiscAround(Plan.Prism.Centre, 6.8)) >= Rules.FixedJawClearanceCm - 0.08);
  Assert.ok(Plan.Closing.Mode === 'sweep' && Plan.Closing.GripDeg <= 20, JSON.stringify(Plan.Closing));
  Assert.deepEqual(Plan.Closing.DiameterCm, [5.3, 6.8]);
  // The model is told why it is planned narrower than it measured.
  const Line = Api.TaskObjectLine(State(Can, Reach_), TipOf(S0800), { GripperFraction: 0.04, Closed: false });
  // (0.5 cm of the fixed jaw's extra standoff fits it, GraspRules.FixedJawExtraStandoffCm: the strict grip holds there at its wrist, not farther out.)
  Assert.deepEqual([Plan.ExtraStandoffCm, Plan.ClearanceCm], [0.5, 1.5]);
  Assert.match(Line, /1\.5 cm outside its left-near side \(it measured 5\.3 x 7\.7 cm, wider than a drink can: something beside it was measured with it, so it is planned as a 6\.8 cm can where its top was measured\), NOT over its centre/);
  // A detector label without the word: the task's own word plans it as a can; without one it is a cylinder as measured.
  const Cylinder = [{ ...Can, label: 'cylinder' }, ...Frame0759.slice(1)];
  const AsCan = Api.TaskObjectReachability(Cylinder[0], { Servos: S0800, ElbowMode: 'down' }, { Others: Api.NeighbourObjects(Cylinder, Cylinder[0]), Kind: 'can' });
  Assert.equal(AsCan.Plan?.Prism.Round.MergedCm, 7.7, AsCan.Text);
  Assert.equal(Strictly(() => Api.TaskObjectReachability(Cylinder[0], { Servos: S0800, ElbowMode: 'down' }, { Others: Api.NeighbourObjects(Cylinder, Cylinder[0]) })).Kind, 'ungraspable');
});

Test('2026-10-01 08:00-08:01: the can beside the battery, refused at mid-height, is planned higher; a taller neighbour gets advice the app checked', () => {
  // 08:01 (and the 08:00 frame "now grab it" would have measured): at mid-height the battery blocked the fixed finger at
  // the pitch the arm reached it, the grip was too lean at the steeper ones and the basket rim was in the way of the flatter.
  for (const Frame of [Frame0801, Frame0800]) {
    const Reach_ = ReachIn(Frame), Can = Frame[0];
    Assert.equal(Reach_.Kind, 'graspable', Reach_.Text);
    const Plan = Reach_.Plan, Mid = (Can.z + Can.topZ) / 2;
    // 0.5 cm higher cleared the battery (live); higher still, the push leaves it FingertipMarginCm inside the fixed finger's tip.
    Assert.deepEqual([Plan.HeightBy, Plan.Side], ['clear', 'left-near']);
    Assert.ok(Plan.RaiseCm >= 0.5 && Plan.RaiseCm <= Rules.MaxRaiseCm && Plan.Closing.TipCm >= Rules.FingertipMarginCm, JSON.stringify([Plan.RaiseCm, Plan.Closing]));
    // Live (the rules of the day), 08:01 0.5 cm higher; 08:00 at mid-height at the flatter pitch the basket rim's tracking
    // envelope was in the way of before the CAD-exact contact model (v8).
    const Live = AsLive(() => ReachIn(Frame).Plan);
    Assert.deepEqual([Live.RaiseCm, Live.HeightBy], Frame === Frame0801 ? [0.5, 'clear'] : [undefined, 'mid'], 'live');
    // Where the jaws close: the printed grasp point less the arm's sag.
    Assert.ok(Plan.Grasp.z - Plan.SagCm >= Mid + Plan.RaiseCm - 0.05 && Plan.Grasp.z - Plan.SagCm <= Mid + Plan.RaiseCm + 0.15, `${Plan.Grasp.z}`);
    // Every gripper part clear of the battery by the guard's margin, at the grasp point and on the way down to it.
    const Battery = Api.GraspPrism(Frame.find(Object_ => Object_.label === 'battery')), Poses = Plan.Poses;
    Assert.equal(Api.DeepestContact(Api.GripperSamples(Poses.Grasp, Poses.JawDegs), [Battery], Rules.ContactMarginCm), null);
    Assert.equal(Api.PathContact(Poses.Hover, Poses.Grasp, Poses.Grasp.gripper, [Battery], Rules.ContactMarginCm, Rules.PathSamples, Poses.JawDegs), null);
    Assert.ok(Plan.Closing.Mode === 'sweep' && Plan.Closing.GripDeg <= 20, JSON.stringify(Plan.Closing));
    // The model is told how much higher and what failed at mid-height there (the grip, or with the fixed pad the finger
    // offset farther out, the basket rim at the pitch it was raised at).
    // (2026-10-03: the basket at the grasp point beside it, the palm before the jaws reach around it, or a lower raise's shorter
    // straddle, as the planner found them.)
    const Raised = Plan.RaiseCm.toFixed(1).replace('.', '\\.');
    Assert.match(Plan.RaisedFor ?? '', /closing jaws would need friction for 2\d deg at a pad|at the grasp point beside it, the gripper link could contact the basket|before both jaws reach 1\.5 cm past its centre|the jaws would reach only \d\.\d cm past its centre/);
    Assert.match(Api.TaskObjectLine(State(Can, Reach_), TipOf(S0800), { GripperFraction: 0.04, Closed: false }),
      new RegExp(`The grasp height is ${Raised} cm above its mid-height \\(at mid-height ${Plan.RaisedFor.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')}\\)\\.`));
  }
  const Can = Frame0801[0];
  // Standing 6.5 cm tall, no raise clears it: the advice says how far to move it, checked by planning it there. (As live: with
  // the real-arm corrections the fixed pad stands 1.5 cm nearer the cart's front edge, which then refuses it first, below.)
  const Tall = Frame0801.map(Object_ => Object_.label === 'battery' ? { ...Object_, topZ: 6.5, heightCm: 8.6 } : Object_);
  const Refused = AsLive(() => ReachIn(Tall));
  Assert.equal(Refused.Refusal?.Code, 'neighbour', Refused.Text);
  const Cm = Number(Refused.Refusal.Advice.match(/^Move the battery at least (\d) cm farther from the can \(the fixed jaw goes down on its left-near side, where the battery is\), then send the task again\.$/)?.[1]);
  Assert.ok(Cm >= 1 && Cm <= Rules.NeighbourMoveMaxCm, Refused.Refusal.Advice);
  const Away = (Objects, D) => {
    const C = Objects[0], B = Objects.find(Object_ => Object_.label === 'battery'), R = Math.hypot(B.x - C.x, B.y - C.y);
    return Objects.map(Object_ => Object_ === B ? { ...Object_, x: B.x + D * (B.x - C.x) / R, y: B.y + D * (B.y - C.y) / R } : Object_);
  };
  Assert.equal(AsLive(() => ReachIn(Away(Tall, Cm))).Kind, 'graspable', 'planned where the advice puts it');
  // With the corrections the strict passes refuse it too (never planned into the battery), for the basket the pad would come
  // near. GRASP ANYTHING (2026-10-18): the best-effort pass now tries every height from just above the desk to under the top,
  // and plans it warned, every gripper part still clear of the battery at the grasp point and on the way down.
  Assert.ok(Strictly(() => !ReachIn(Tall).Plan));
  const Corrected = ReachIn(Tall);
  Assert.equal(Corrected.Kind, 'graspable', Corrected.Text);
  Assert.ok(Corrected.Plan.Warnings.length > 0, 'a best-effort plan');
  const TallBattery = Api.GraspPrism(Tall.find(Object_ => Object_.label === 'battery')), CPoses = Corrected.Plan.Poses;
  Assert.equal(Api.DeepestContact(Api.GripperSamples(CPoses.Grasp, CPoses.JawDegs), [TallBattery], Rules.ContactMarginCm), null);
  Assert.equal(Api.PathContact(CPoses.Hover, CPoses.Grasp, CPoses.Grasp.gripper, [TallBattery], Rules.ContactMarginCm, Rules.PathSamples, CPoses.JawDegs), null);
});

// --- "It's often saying it can't reach the can": the reach verdicts of 2026-09-30 and 10-01 ----------

Test('2026-10-01 07:55: the can 53 cm out is out of reach, and the advice names a spot the app planned, off the bottle and the basket', () => {
  // Measured at the far left behind the bottle (the image agrees). Straight toward the shoulder the spots 10-20 cm
  // closer are on the bottle or the tape, or at the basket's rim; the old search stopped there and gave no advice.
  const Far = Obj('can', 20.65, 40.84, -1.68, 10.12, [5.2, 5.5], { supportZMeasured: -1.68, extrapolated: true, outsideCalibrationCm: 14.2 });
  const Near = [Obj('bottle', 11.75, 31.49, -2.3, 5.5, [3.8, 2.9], { supportZMeasured: -1.99 }), Obj('tape', 1.48, 30.19, -2.4, 2.36, [8.1, 4.8], { refined: false }),
    Obj('cup', -12.61, 31.72, -2.2, -0.07, [8.3, 8.8], { refined: false })];
  const Context0755 = { Servos: S0755, ElbowMode: 'down' };
  const Reach_ = Api.TaskObjectReachability(Far, Context0755, { Others: Near, Kind: 'can' });
  Assert.equal(Reach_.Kind, 'out-of-reach', Reach_.Text);
  // (2026-10-03: the jaws reaching around it put the grasp point deeper, so the 42 cm spot at X 7.6, Y 36.6 the advice gave
  // was a little out of reach and it said X 18.1, Y 26.6; spots refused for reach beside them no longer end the search,
  // CloserToGrasp. Since the deep grasp, 2026-10-04, a head start the arm does not reach is shortened to one it does, and
  // X 7.6, Y 36.6 plans again.)
  Assert.match(Reach_.Text, /^OUT OF REACH \(53\.0 cm from the right arm's pan axis; the closest pose misses by 9\.3 cm\); moved about 14 cm to X 7\.6, Y 36\.6 \(its centre about 42 cm from the pan axis\) the app plans the grasp$/);
  Assert.equal(Reach_.Advice, 'Move the can about 14 cm: 13 cm to the arm\'s right (the camera image\'s right) and 4 cm nearer the arm, to about X 7.6, Y 36.6, then send the task again.');
  // Planned there among the same objects, and clear of the bottle and the tape; the user's own move, to X 3.4, Y 37.9, planned too.
  const There = { ...Far, x: 7.6, y: 36.6, extrapolated: false, outsideCalibrationCm: 0 };
  Assert.equal(Api.TaskObjectReachability(There, Context0755, { Others: Near, Kind: 'can' }).Kind, 'graspable');
  for (const Other of Near.slice(0, 2)) Assert.ok(Math.abs(Other.x - There.x) >= (5.2 + Other.sizeCm[0]) / 2 + 0.5 || Math.abs(Other.y - There.y) >= (5.5 + Other.sizeCm[1]) / 2 + 0.5, Other.label);
  Assert.equal(Api.TaskObjectReachability({ ...There, x: 3.4, y: 37.9 }, Context0755, { Others: Near, Kind: 'can' }).Kind, 'graspable');
});

Test('2026-09-30 19:10: the pill container by the basket is judged by the planner (reached above mid-height), not called BLOCKED at Z 1.3', () => {
  // BLOCKED at its mid-height Z 1.3 by 0.2 cm while Z 1.5 and up reach: the plan beside it, from the pitch that reaches
  // higher, said why it failed (the basket). Those were the basket's tracking envelope: since the CAD-exact contact model
  // (v8) the arm reaches its centre at mid-height, and the planner plans it beside it.
  const Pill = Obj('bottle', 12.27, 30.04, -2.49, 5.18, [3.6, 1.8], { supportZMeasured: -2.56, validFraction: 0.954, outsideCalibrationCm: 0.6 });
  const Around = [Obj('tape', 5.88, 33.95, -2.19, -0.88, [7.6, 5.2], { refined: false }), Obj('cup', -13.56, 30.67, -2.3, -0.03, [8.5, 4.3], { refined: false })];
  const Reach_ = Api.TaskObjectReachability(Pill, { Servos: S0800, ElbowMode: 'down' }, { Others: Around, Kind: 'pill container' });
  Assert.doesNotMatch(Reach_.Text, /BLOCKED|OUT OF REACH/, Reach_.Text);
  Assert.equal(Reach_.Kind, 'graspable', Reach_.Text);
  // Without a grasp (no plan asked for) the centre's verdict: reached at its mid-height.
  Assert.match(Api.TaskObjectReachability(Pill, { Servos: S0800, ElbowMode: 'down' }).Text, /^graspable at Z 1\.3 with wrist/);
});

Test('the grasp gap below the default moves only a round object\'s plan: anything else keeps the default, its only room for the real fixed finger', () => {
  // Review of 2026-10-01: below the default a box's plan had no real-finger correction and no check of the finger nearer still
  // (RealArmCorrections: only a round object's sweep plan), yet the setting said every safety check still ran. The 2.5 x 2.0
  // box at (-4, 32) was refused at the default and planned at 0.5 to 0.9 with the CAD fixed finger 0.5 cm from it.
  // The review's probe: the arm at the 18:53 Home raws, the box asked for as a box.
  const Home1853 = Snapshot({ shoulder_pan: 3479, shoulder_lift: 3172, elbow_flex: 2688, wrist_flex: 856, wrist_roll: 1521, gripper: 694 });
  // (The setting alone: no extra standoff for the fixed jaw, GraspRules.FixedJawExtraStandoffCm. With it the 2.5 x 2.0 box
  // below is planned with the jaw-plane sweep 2.54 cm farther out, and the usb drive at the setting's gap, nothing more fitting.)
  const At = (Object_, Cm) => WithExtra(0, () => Api.TaskObjectReachability(Object_, { ...Context(), ClearanceCm: Cm }, { Others: [] }));
  // (The strict passes' verdicts: the best-effort pass plans the box anyway, its warnings not about the gap.)
  const AtBox = (Object_, Cm) => WithExtra(0, () => Strictly(() => Api.TaskObjectReachability(Object_, { Servos: Home1853, ElbowMode: 'up', ClearanceCm: Cm }, { Others: [], Kind: 'box' })));
  const Box = Obj('box', -4, 32, -2.1, 5.0, [2.5, 2.0], { validFraction: 0.95 });
  const Default = AtBox(Box, 1.0);
  Assert.equal(Default.Refusal?.Code, 'wide', Default.Text);
  for (const Cm of [0.5, 0.8]) {
    const Small = AtBox(Box, Cm);
    Assert.equal(Small.Kind, 'ungraspable', `${Cm}: ${Small.Text}`);
    Assert.deepEqual([Small.Refusal.Code, Small.Refusal.GapCm, Small.Refusal.Text], ['wide', 1, Default.Refusal.Text], `${Cm}: refused as at the default`);
    Assert.doesNotMatch(Small.Refusal.Text + Small.Refusal.Advice, /grasp gap/, `${Cm}: the setting played no part`);
  }
  // A plannable box (the usb drive): below the default the plan is the default's, and the model is told why the setting was not used.
  for (const Cm of [0.5, 0.8]) {
    const Small = At(Usb, Cm);
    Assert.equal(Small.Kind, 'graspable', `${Cm}: ${Small.Text}`);
    Assert.deepEqual([Small.Plan.Grasp, Small.Plan.Pitch, Small.Plan.ClearanceCm, Small.Plan.GapSetCm, Small.Plan.FingerOffsetCm],
      [UsbPlan.Grasp, UsbPlan.Pitch, 1, Cm, undefined], `${Cm}`);
    const Line = Api.TaskObjectLine(State(Usb, Small), { x: -13.3, y: 20.1, z: 12.4 }, { GripperFraction: 0.8, Closed: false });
    Assert.match(Line, new RegExp(`the tip goes BESIDE the usb drive, 1\\.0 cm outside its left-near side[^\\n]* The user set the grasp gap to ${Cm.toFixed(1).replace('.', '\\.')} cm, but the fixed jaw keeps the default 1\\.0 cm beside the usb drive: only a round object's GRASP PLAN corrects for where the real fixed finger stands\\. The grasp height`));
    Assert.doesNotMatch(Line, /grasp gap the user set/);
  }
  Assert.equal(UsbPlan.GapSetCm, undefined, 'at the default: nothing to say');
  // Above the default it applies to any object: the pad goes out by it, and a box too wide for the jaws is told the gap is part of why.
  const Wider = At(Usb, 1.3);
  Assert.equal(Wider.Kind, 'graspable', Wider.Text);
  Assert.deepEqual([Wider.Plan.ClearanceCm, Wider.Plan.GapSetCm], [1.3, undefined]);
  const U = Wider.Plan.JawAxis, Along = Point => Point.x * U.x + Point.y * U.y;
  Assert.ok(Math.abs(Along(UsbPlan.Grasp) - Along(Wider.Plan.Grasp) - 0.3) <= 0.15, `${Along(UsbPlan.Grasp) - Along(Wider.Plan.Grasp)}`);
  const WideBox = AtBox(Box, 1.3);
  Assert.equal(WideBox.Refusal.Code, 'wide');
  Assert.ok(WideBox.Refusal.Text.endsWith('; the grasp gap at the fixed finger is set to 1.3 cm in Settings (the default is 1.0 cm): the wider the gap, the farther the moving jaw must close'), WideBox.Refusal.Text);
  Assert.doesNotMatch(WideBox.Refusal.Text, /may plan it/);
  // The planner's own rule: c only below the default where the plan corrects for the real finger (RealArmCorrections On:
  // a round object's sweep plan, its fixed finger checked FixedFingerToleranceCm nearer, whatever FixedFingerOffsetCm is).
  Assert.deepEqual([Api.ClearanceFor({ ClearanceCm: 0.6 }, false), Api.ClearanceFor({ ClearanceCm: 0.6 }, true),
    Api.ClearanceFor({ ClearanceCm: 1.4 }, false), Api.ClearanceFor({}, false)], [1, 0.6, 1.4, 1]);
  const Corrections = Api.RealArmCorrections(Api.GraspPrism(Can2), true);
  Assert.deepEqual([Corrections.On, Corrections.OffsetCm, Corrections.SagCm], [true, Rules.FixedFingerOffsetCm, Rules.GraspSagCm]);
});

// THE FIXED JAW'S EXTRA STANDOFF (GraspRules.FixedJawExtraStandoffCm, 2026-10-25, Aaran: the fixed jaw comes down a little
// too close to the object, "preferably 1 inch" farther). The fixed finger's inner face at the printed grasp pose: how far it
// stands from the object's near face (the footprint across the finger's band) along the jaws' closing direction u.
function FixedFaceGap(Plan) {
  const Uh = Plan.JawAxis, Vh = { x: -Uh.y, y: Uh.x }, Prism = Plan.Prism;
  const Across = Prism.Footprint.map(P => Dot(P, Vh)), Cv = (Math.min(...Across) + Math.max(...Across)) / 2;
  const Near = Math.min(...Band(Prism.Footprint, Vh, Cv - Rules.FingerBandHalfCm, Cv + Rules.FingerBandHalfCm).map(P => Dot(P, Uh)));
  const Finger = Api.GripperSamples(Plan.Poses.Grasp, []).filter(S => S.Part === 'FixedFinger' && S.P.z >= Prism.BaseZ - 0.5 && S.P.z <= Prism.TopZ + 0.5);
  return Near - Math.max(...Finger.map(S => Dot(S.P, Uh)));
}

Test('the fixed jaw stands the grasp gap + 2.54 cm from the object along the closing direction, whatever the approach', () => {
  Assert.equal(Rules.FixedJawExtraStandoffCm, 2.54);
  // A 3.5 cm cup ahead and to the left of the arm (wrist -31) and one farther out to the right (wrist -13): the jaws'
  // closing directions -u (0.79, -0.61) and (0.92, -0.38), the fixed jaw on the +X side of each.
  const Cases = [['cup', Obj('cup', 8, 32, -2.3, 6, [3.5, 3.0])], ['far cup', Obj('cup', -2, 38, -2.3, 6, [3.5, 3.0])]];
  const Axes = [];
  for (const [Name, Object_] of Cases) {
    const Before = WithExtra(0, () => Reach(Object_)).Plan, After = Reach(Object_).Plan;
    Assert.ok(Before && After, Name);
    Assert.deepEqual([Before.ExtraStandoffCm, Before.ClearanceCm, After.ExtraStandoffCm, After.ClearanceCm], [undefined, 1, 2.54, 3.54], Name);
    Assert.equal(After.Warnings, undefined, `${Name}: planned with every check`);
    const [Was, Now_] = [FixedFaceGap(Before), FixedFaceGap(After)];
    // (The finger's samples cover its surface within ContactMarginCm; a round object's disc is planned at its widest.)
    Assert.ok(Was >= 1 - 0.08 && Was <= 1 + 0.2, `${Name}: before ${Was.toFixed(2)} cm`);
    Assert.ok(Now_ >= 3.54 - 0.08 && Now_ <= 3.54 + 0.2, `${Name}: after ${Now_.toFixed(2)} cm`);
    Assert.ok(Math.abs(Now_ - Was - 2.54) <= 0.15, `${Name}: ${(Now_ - Was).toFixed(2)} cm farther`);
    // Away from the object: the fixed jaw is on the -u side of it (u points from the fixed pad into the gap).
    Assert.ok(Dot(After.Grasp, After.JawAxis) < Dot(After.Prism.Centre, After.JawAxis) - 3.54, Name);
    Axes.push([After.JawAxis, After.Pitch]);
  }
  Assert.ok(Math.acos(Dot(Axes[0][0], Axes[1][0])) * 180 / Math.PI > 10 && Math.abs(Axes[0][1] - Axes[1][1]) > 10, 'two approach directions');
  // The model is told the gap and why it is wider.
  const Cup = Reach(Cases[0][1]);
  const Line = Api.TaskObjectLine(State(Cases[0][1], Cup), { x: -13.3, y: 20.1, z: 12.4 }, { GripperFraction: 0.8, Closed: false });
  Assert.match(Line, /the tip goes BESIDE the cup, 3\.5 cm outside its left-near side[^\n]* That 3\.5 cm is the 1\.0 cm grasp gap the user set for the fixed jaw plus 2\.5 cm more the app keeps the fixed jaw from it/);
});

Test('an object that does not fit with the full 2.54 cm falls back to the largest smaller extra that plans, down to the old gap', () => {
  // The pen (1 cm across, planned with the jaw table): 2.54 cm or 2 cm more and the moving jaw would close more than the 5 cm
  // it meets the side of; 1.5 cm more fits (W_eff 4.9 cm), at the wrist the old gap planned it at.
  const Pen_ = Reach(Pen).Plan, PenBefore = WithExtra(0, () => Reach(Pen)).Plan;
  Assert.deepEqual([Pen_.ExtraStandoffCm, Pen_.ClearanceCm, Pen_.Closing.Mode, Pen_.Warnings, Pen_.Pitch], [1.5, 2.5, 'table', undefined, PenBefore.Pitch]);
  Assert.ok(Pen_.WidthEffCm <= 5.0 && Pen_.WidthEffCm + 0.5 > 5.0, `${Pen_.WidthEffCm}`);
  Assert.ok(Math.abs(FixedFaceGap(Pen_) - 2.5) <= 0.1 && Math.abs(FixedFaceGap(Pen_) - FixedFaceGap(PenBefore) - 1.5) <= 0.15,
    `${FixedFaceGap(Pen_)} - ${FixedFaceGap(PenBefore)}`);
  // A 2 cm box the old gap planned with the jaw table: 1.5 cm more with the jaw-plane sweep there, 50% open.
  const Box = Obj('box', -10, 32, -2.3, -0.3, [2, 2]), Swept = Reach(Box).Plan;
  Assert.deepEqual([Swept.ExtraStandoffCm, Swept.Closing.Mode, Swept.OpenFraction, Swept.Pitch, Swept.Warnings], [1.5, 'sweep', 0.5, -60, undefined]);
  Assert.ok(Math.abs(FixedFaceGap(Swept) - 2.5) <= 0.2, `${FixedFaceGap(Swept)}`);
  // A plan whose grip holds by the strict closing with less beats one that holds only by the old closing with more: the can
  // at 16:49's spot keeps the old gap (with 1.5 cm more the old closing planned it at wrist -9, pushed 2.7 cm along the
  // fingers and left 0.2 cm inside the fixed finger's tip: the close that squeezed it out on 2026-10-01).
  const Can = Reach(Can2).Plan;
  Assert.deepEqual([Can.ExtraStandoffCm, Can.ClearanceCm, Can.Pitch], [undefined, 1, WithExtra(0, () => Reach(Can2)).Plan.Pitch]);
  Assert.ok(Can.Closing.TipCm >= Rules.FingertipMarginCm, `${Can.Closing.TipCm}`);
  // Nothing more fits the usb drive: planned at the old gap, as before, never refused for the standoff.
  Assert.deepEqual([UsbPlan.ExtraStandoffCm, UsbPlan.ClearanceCm, UsbPlan.Grasp], [undefined, 1, WithExtra(0, () => Reach(Usb)).Plan.Grasp]);
  Assert.ok(Math.abs(FixedFaceGap(UsbPlan) - 1) <= 0.08, `${FixedFaceGap(UsbPlan)}`);
  // Half an inch: the pen fits it all.
  const Half = WithExtra(1.27, () => Reach(Pen)).Plan;
  Assert.deepEqual([Half.ExtraStandoffCm, Half.ClearanceCm], [1.27, 2.27]);
  Assert.ok(Math.abs(FixedFaceGap(Half) - 2.27) <= 0.1, `${FixedFaceGap(Half)}`);
});
