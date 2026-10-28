import { build as Build } from 'esbuild';
import { readFileSync as ReadFile } from 'node:fs';
import { createRequire as CreateRequire } from 'node:module';
import { test as Test } from 'node:test';
import Assert from 'node:assert/strict';
import Path from 'node:path';
import { fileURLToPath as FilePath } from 'node:url';

// The arm measured as an object (src/lib/ArmMadeObjects.ts): a measurement whose top is the
// gripper's underside is the gripper itself, not an obstacle, a neighbour or an object to target.
//
// 2026-10-03 16:41:36, "grab the pill bottle" (back end/captures/2026-10-03/recording): with the
// gripper hovering over the bottle, 60% open, the "phone" box (the phone lies flat at about Y 47)
// held the gripper: the bridge measured it at X -3.6, Y 27.2, 13.2 x 12.8 cm, 9.85 cm tall
// (grounding.jsonl: arm_fraction 0.356), and at 16:41:40 the guard refused the descent onto the
// bottle: "the gripper's open moving jaw would go 4.4 cm into the phone measured at X -3.6, Y 27.2".
// The same day's real objects beside or under the gripper still count. No bridge, no hardware.
const Root = Path.dirname(FilePath(import.meta.url));
const Bundle = await Build({ stdin: { contents: ReadFile(Path.join(Root, 'src/App.tsx'), 'utf8')
  + '\nexport { parseAiResponse as ParseResponse, validateAiMotionProposal as Validate, applyAiSnap as ApplySnap, aiSnapInput as SnapInput };'
  + '\nexport { SnapAiTarget } from "./lib/AiTargetSnap";'
  + '\nexport * from "./lib/ArmMadeObjects";'
  + '\nexport { JudgeArmOcclusion } from "./lib/ArmOcclusion";'
  + '\nexport { GraspPlanKey, GraspRules, NeighbourObjects } from "./lib/GraspPlan";'
  + '\nexport { BuildGraspGuard, MeasuredObjectsSummary, NamesQuery, ParseTaskObject, PlannedStepOf, ReadLocateResponse, ResolveTaskObject, TaskObjectLine, TaskObjectReachability,'
  + ' UnreliableMeasurements } from "./lib/AiMeasurements";'
  + '\nexport { currentAngles, forwardKinematics, INITIAL_SERVOS, mirrorDisplayPoint, rawFromAngle, rawToFraction, wristPitchFromAngles } from "./lib/robotModel";',
  resolveDir: Path.join(Root, 'src'), loader: 'tsx' }, bundle: true, platform: 'node', format: 'cjs', write: false, loader: { '.css': 'empty' }, logLevel: 'error' });
globalThis.window ??= { location: { hostname: 'localhost' } };
const Module = { exports: {} };
new Function('require', 'module', 'exports', Bundle.outputFiles[0].text)(CreateRequire(import.meta.url), Module, Module.exports);
const Api = Module.exports;
const Debug = process.env.ShowLines ? (...Args) => console.log(...Args) : () => {};

// --- The recorded frames ------------------------------------------------------------------------

const Snapshot = Raw => {
  const Servos = structuredClone(Api.INITIAL_SERVOS);
  for (const [Name, Value] of Object.entries(Raw)) Object.assign(Servos[Name], { presentRaw: Value, targetRaw: Value, guiAngleRaw: null });
  return Servos;
};
// The model requests' "Current servo positions" after each capture.
const Raw = (Pan, Lift, Elbow, Wrist, Gripper) => ({ shoulder_pan: Pan, shoulder_lift: Lift, elbow_flex: Elbow, wrist_flex: Wrist, wrist_roll: 1570, gripper: Gripper });
const Home = Snapshot(Raw(3475, 3172, 2688, 855, 702));
const Servos1636 = Snapshot(Raw(3154, 405, 1655, 854, 1599)), Tip1636 = { x: 2.8468, y: 30.9552, z: 9.4607 };
const Obj = (label, x, y, z, topZ, sizeCm, supportZMeasured, validFraction, Extra = {}) => ({ label, x, y, z, topZ, heightCm: Number((topZ - z).toFixed(2)), sizeCm,
  depthMeasured: true, extrapolated: false, unstable: false, supportMeasured: true, supportZMeasured, validFraction, supportBranch: 'desk',
  taskMatch: false, taskQueryOnly: false, refined: false, outsideCalibrationCm: 0, ...Extra });
const Far = Cm => ({ extrapolated: true, outsideCalibrationCm: Cm });
// 16:41:12: the bottle (bound and planned), and every object measured with the arm at home.
const Bottle = { ...Obj('pill bottle', -1.56, 32.68, -2.68, 5.87, [3.3, 3], -2.45, 0.851, { taskMatch: true, refined: true }), heightCm: 8.55 };
const Frame1612 = [Obj('can', 18.72, 58.72, -1.52, 10.15, [6.2, 5], -1.4, 0.97, Far(29.5)), Obj('keyboard', 14.52, 85.46, -0.06, 1.14, [10, 23], -0.28, 0.993, Far(52.7)),
  Obj('mouse', 16.75, 109.95, 1.39, 3.84, [8.2, 9.2], 1.44, 1, Far(76.7)), Obj('phone', -1.65, 33.71, -2.28, 6.64, [13.3, 16], -2.2, 0.958),
  Bottle, Obj('tape', -0.01, 31.91, -2.96, 5.16, [5.5, 5.4], -2.65, 0.907), Obj('black box', 0.11, 28.13, -3.39, -0.77, [4.4, 3.1], -3.11, 0.999),
  Obj('soda can', 18.72, 58.73, -1.58, 10.09, [6.2, 4.1], -1.36, 0.967, Far(29.5)), Obj('tape roll', -0.01, 31.91, -2.96, 5.16, [5.5, 5.4], -2.64, 0.903),
  Obj('black cube', 0.12, 28.11, -3.37, -0.77, [4.1, 3.1], -3.07, 0.998)];
// 16:41:36 as the app read it from the bridge then (no arm_fraction), and as a bridge that reports it gives it.
const Phone1636 = Obj('phone', -3.6, 27.21, -2.11, 7.74, [13.2, 12.8], -1.93, 0.8, { outsideCalibrationCm: 1.4 });
const Frame1636 = [Obj('can', 18.56, 58.66, -1.42, 10.23, [6.1, 5.6], -1.34, 0.97, Far(29.4)), Obj('keyboard', 14.08, 85.66, 0.06, 1.2, [9.7, 22.7], -0.17, 0.995, Far(52.7)),
  Phone1636, Obj('soda can', 18.56, 58.72, -1.41, 10.26, [6.1, 7.2], -1.28, 0.969, Far(29.4))];
const Reported = Frame => Frame.map(Object_ => ({ ...Object_, armFraction: Object_ === Phone1636 ? 0.356 : 0 }));
const LocateOf = Objects => ({ ...Api.ReadLocateResponse({ ok: true, objects: [], rms_cm: 0.75, camera_check: { ok: true, angle_deg: 0, offset_cm: 0, reason: '' }, withheld: [] }), objects: Objects });
// The same day's real objects beside or under the gripper (vision records), with the arm's pose and the bridge's arm_fraction.
const Real = [
  // 14:50:40: the jaws open beside the can at the grasp point; the can read with the jaws (5.0 x 8.6, not 5.6 x 4.9).
  ['14:50:40 can beside the open jaws', Raw(3147, 424, 2081, 282, 1918), Obj('can', -4.36, 30.08, -3.31, 8.57, [5, 8.6], -2.34, 0.839, { taskMatch: true, refined: true }), 0.172],
  // 14:52-14:59: the can pushed out of the jaws, the gripper raised over its near side, closed then open.
  ['14:52:30 can under the closed jaws', Raw(3147, 223, 2007, 569, 648), Obj('black cup', 1.39, 33.06, -3.07, 8.86, [4.9, 4.2], -2.37, 0.671, { refined: true }), 0.373],
  ['14:55:23 can under the closed jaws', Raw(3147, 197, 1968, 651, 648), Obj('can', 1.36, 32.8, -3.04, 8.83, [4.7, 4.3], -2.95, 0.575, { refined: true }), 0.382],
  ['14:56:15 can under the open jaws', Raw(3147, 202, 1972, 651, 1917), Obj('can', 1.42, 32.8, -3.06, 8.81, [5, 4.4], -2.76, 0.583, { refined: true }), 0.407],
  ['14:59:24 can under the raised jaws', Raw(3147, 181, 1921, 736, 1917), Obj('can', 1.45, 32.94, -3.03, 8.9, [4.7, 3.6], -2.9, 0.502, { refined: true }), 0.222],
  // 16:42:03: the phone itself, flat, with the gripper in its box.
  ['16:42:03 the phone itself', Raw(3132, 463, 1598, 858, 1599), Obj('phone', 4.35, 47.11, -2.12, -0.8, [13.3, 6.6], -1.9, 0.758, Far(13.1)), 0.202],
];

// The bottle's GRASP PLAN (16:41:12) and the task as the app keeps it while the gripper hides the bottle.
const Query = Api.ParseTaskObject('grab the pill bottle');
// Planned as it was then, before the deep grasp (GraspRules.DeepGrasp, 2026-10-04: deeper in the jaws, it opens to 80%):
// the recorded check-ins had the gripper 60% open at the hover, and the arm-made objects do not depend on the depth. (Nor
// with the fixed jaw's extra standoff, GraspRules.FixedJawExtraStandoffCm of 2026-10-25: the plan then had none.)
const Shallow = Make => {
  const Saved = [Api.GraspRules.DeepGrasp, Api.GraspRules.FixedJawExtraStandoffCm];
  Object.assign(Api.GraspRules, { DeepGrasp: false, FixedJawExtraStandoffCm: 0 });
  try { return Make(); } finally { [Api.GraspRules.DeepGrasp, Api.GraspRules.FixedJawExtraStandoffCm] = Saved; }
};
const Reach = Shallow(() => Api.TaskObjectReachability(Bottle, { Servos: Home, ElbowMode: 'up', ClearanceCm: 1 }, { Others: Api.NeighbourObjects(Frame1612, Bottle), Kind: Query.Name }));
const Plan = Reach.Plan;
const At1612 = Date.parse('2026-10-03T16:41:12.237Z'), At1636 = Date.parse('2026-10-03T16:41:36.063Z');
const Seen = { At: At1612, Object: Bottle, Reachability: Reach, Motions: 0 };
const TaskState = Occluded => ({ Label: Query.Name, Query, Intent: 'grasp', DetectedNow: false, Binding: null, Candidates: [], LastSeen: Seen, Reachability: null,
  Plan: { Reach, Source: Bottle, At: At1612, Key: Api.GraspPlanKey(Home, 'up'), Note: null, Confirmed: false, Corridor: null }, Occluded });
const GraspAt = Servos => ({ GripperFraction: Api.rawToFraction(Servos.gripper, Servos.gripper.presentRaw), Closed: false, Held: null });
// The arm at the plan's hover point (the pose the plan checked there), the gripper 60% open.
const IkJoints = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll'];
const ServosHover = Snapshot({ ...Object.fromEntries(IkJoints.map(Name => [Name, Math.round(Api.rawFromAngle(Api.INITIAL_SERVOS[Name], Plan.Poses.Hover[Name]))])), gripper: 1599 });
const ServosGrasp = Snapshot({ ...Object.fromEntries(IkJoints.map(Name => [Name, Math.round(Api.rawFromAngle(Api.INITIAL_SERVOS[Name], Plan.Poses.Grasp[Name]))])), gripper: 1599 });
const TipOf = Servos => Api.mirrorDisplayPoint(Api.forwardKinematics(Api.currentAngles(Servos)).at(-1));

// A reply checked as the app checks it (read from the tip, snapped, validated against this check-in's guard).
function Check(Servos, Frame, Motion) {
  const Tip = TipOf(Servos), Occluded = Api.JudgeArmOcclusion(Seen, null, Tip, { Plan, Closed: false, Missed: false, Now: At1636, Motions: 1 });
  const State = TaskState(Occluded), Grasp = GraspAt(Servos);
  const Angles = Api.currentAngles(Servos);
  const Anchor = { ...Api.forwardKinematics(Angles).at(-1), wristPitch: Api.wristPitchFromAngles(Angles) };
  const Line = Api.TaskObjectLine(State, Tip, Grasp);
  const Step = Api.PlannedStepOf(Line, Api.mirrorDisplayPoint(Anchor), Anchor.wristPitch);
  const Guard = Api.BuildGraspGuard(Frame, State, false, Step);
  const Unreliable = Api.UnreliableMeasurements(Frame, State);
  const Parsed = Api.ParseResponse(`{"response_to_user":"","task_status":"continue","motion_proposal":${Motion ?? Step.Json}}`, Anchor, Servos).motion;
  const Guarded = { ...Parsed, guard: Guard };
  const Snapped = Api.ApplySnap(Guarded, Api.SnapAiTarget(Api.SnapInput(Guarded, Servos, 'up', undefined, Unreliable)));
  return { Guard, Step, Line, ...Api.Validate(Snapped, 'up', Servos, Unreliable, undefined, { Object: Bottle, State: Grasp, Plan }, Guard) };
}
const PhoneIssue = /into the phone/;

Test('16:41:36: the "phone" with the open gripper in its box is the gripper itself, from the bridge\'s output then and from one that reports arm_fraction', () => {
  const Gripper = Api.GripperSurface(Servos1636);
  for (const [Name, Frame] of [['as recorded (no arm_fraction)', Frame1636], ['with arm_fraction 0.356', Reported(Frame1636)]]) {
    const Phone = Frame.find(Object_ => Object_.label === 'phone');
    const Made = Api.ArmMadeReason(Phone, Gripper);
    Assert.ok(Made, Name);
    Debug(`--- ${Name}: ${Made.Reason}`);
    // Its top just under the gripper (the gripper's underside), the gripper deep inside its footprint, none of it in the box.
    Assert.ok(Made.GapCm > 0 && Made.GapCm <= Api.ArmMadeRules.TopGapCm, `gap ${Made.GapCm}`);
    Assert.ok(Made.InsideCm >= Api.ArmMadeRules.InsideCm, `inside ${Made.InsideCm}`);
    Assert.match(Made.Reason, /^the robot's own gripper, measured as an object: its measured top \(Z 7\.7\) is the underside of the gripper over it \(\d\.\d cm under it, \d\.\d cm inside its 13\.2 x 12\.8 cm footprint/);
    // Never measured clear of the gripper: left out; every other object of the frame stays.
    const { Located, Dropped } = Api.WithoutArmMade(LocateOf(Frame), Servos1636);
    Assert.deepEqual(Dropped.map(Item => [Item.Object, Item.Kept]), [[Phone, null]]);
    Assert.deepEqual(Located.objects.map(Object_ => Object_.label), ['can', 'keyboard', 'soda can']);
    // Not offered to the model as an object: listed as not measurable, with why.
    const Summary = Api.MeasuredObjectsSummary(Located);
    Assert.doesNotMatch(Summary, /^- phone: centre/m);
    Assert.match(Summary, /Not measurable in this frame \(do not target these\):\n- phone: not measurable: the robot's own gripper, measured as an object[^\n]*\)$/m);
    // Neither a neighbour of the bottle nor a prism of the guard.
    Assert.ok(!Api.NeighbourObjects(Located.objects, Bottle).includes(Phone));
    Assert.ok(!Api.BuildGraspGuard(Located.objects, TaskState(null), false, null).Prisms.some(Prism => Prism.Label === 'phone'));
    // As the app runs: the 16:41:12 capture (the arm at home) read every object clear of the gripper, so the phone
    // keeps that reading, which stands where the bottle stands (a same-spot twin of it, as it was at 16:41:12).
    const Clear = new Map();
    Assert.deepEqual(Api.WithoutArmMade(LocateOf(Frame1612), Home, Clear, At1612).Dropped, []);
    Assert.deepEqual([...Clear.keys()].sort(), Frame1612.map(Object_ => Object_.label).sort());
    const Then = Api.WithoutArmMade(LocateOf(Frame), Servos1636, Clear, At1636);
    Assert.deepEqual(Then.Dropped.map(Item => [Item.Object, Item.Kept?.Object]), [[Phone, Frame1612[3]]]);
    const Kept = Then.Located.objects.find(Object_ => Object_.label === 'phone');
    Assert.deepEqual({ ...Kept, armClearAt: undefined }, { ...Frame1612[3], armClearAt: undefined });
    Assert.equal(Kept.armClearAt, At1612);
    const Listed = Api.MeasuredObjectsSummary(Then.Located);
    Debug(Listed);
    Assert.match(Listed, /^- phone: centre X=-1\.6, Y=33\.7, [^\n]* \(as measured at \d\d:\d\d:\d\d with the gripper clear of it: this check-in measured the gripper in its place\)/m);
    Assert.match(Listed, /^- phone: not measurable: the robot's own gripper, measured as an object[^\n]*; its reading at \d\d:\d\d:\d\d, with the gripper clear of it, stands$/m);
    Assert.ok(!Api.NeighbourObjects(Then.Located.objects, Bottle).includes(Kept), 'a twin of the bottle, not its neighbour');
    Assert.ok(!Api.BuildGraspGuard(Then.Located.objects, TaskState(null), false, null).Prisms.some(Prism => Prism.Label === 'phone' && !Prism.Role));
    // It is never taken for the bottle the gripper hides (16 cm long, not the 3.3 cm bottle).
    Assert.equal(Api.ResolveTaskObject(Query, Then.Located.objects, { LastSeen: Bottle, Track: true, Tip: Tip1636 }).Binding, null);
    // Were it the task's object ("grab the phone"), it is only left out: the task object's own rules keep its last measurement.
    const Task = Api.WithoutArmMade(LocateOf(Frame), Servos1636, Clear, At1636, Object_ => Api.NamesQuery(Api.ParseTaskObject('grab the phone'), Object_.label));
    Assert.deepEqual(Task.Dropped.map(Item => [Item.Object, Item.Kept]), [[Phone, null]]);
    Assert.ok(!Task.Located.objects.some(Object_ => Object_.label === 'phone'));
  }
  // A bridge that reports the arm a small part of its box: the gripper's geometry alone does not drop it.
  Assert.equal(Api.ArmMadeReason({ ...Phone1636, armFraction: 0.1 }, Gripper), null);
  // With the arm at home the same box is no gripper.
  Assert.equal(Api.ArmMadeReason(Phone1636, Api.GripperSurface(Home)), null);
});

Test('16:41:40: the descent the guard refused for the "phone" is no longer refused for it, and the plan\'s own descent passes the app\'s checks', () => {
  // The model's reply at 16:41:40 (the old plan's grasp point, straight down from where the arm stood).
  const Recorded = '{"target_cm":{"x":3.1468407575318516,"y":30.8551525825444,"z":2.0607119614234097},"wrist_pitch_degrees":-32.8}';
  const Then = Check(Servos1636, Frame1636, Recorded);
  Debug('--- 16:41:40 as recorded', Then.possible, Then.issues);
  Assert.ok(Then.issues.some(Issue => PhoneIssue.test(Issue)), Then.issues.join(' | '));
  // The phone left out (never measured clear of the gripper), or standing as read at 16:41:12 (the app's case).
  const Clear = new Map();
  Api.WithoutArmMade(LocateOf(Frame1612), Home, Clear, At1612);
  for (const [Name, Filtered] of [['left out', Api.WithoutArmMade(LocateOf(Frame1636), Servos1636).Located.objects],
    ['as read at 16:41:12', Api.WithoutArmMade(LocateOf(Frame1636), Servos1636, Clear, At1636).Located.objects]]) {
    const Now = Check(Servos1636, Filtered, Recorded);
    Debug(`--- 16:41:40, the phone ${Name}`, Now.possible, Now.issues);
    Assert.ok(!Now.issues.some(Issue => PhoneIssue.test(Issue)), `${Name}: ${Now.issues.join(' | ')}`);
    Assert.ok(!Now.Guard.Prisms.some(Prism => Prism.Label === 'phone' && Prism.Role !== 'twin'), Name);
    // The NEXT STEP from there (the plan's hover point), and from the hover point its descent to the grasp point, with this frame's objects.
    const Step = Check(Servos1636, Filtered);
    Assert.ok(!Step.issues.some(Issue => PhoneIssue.test(Issue)), `${Name}: ${Step.issues.join(' | ')}`);
    if (Name === 'left out') Assert.equal(Step.possible, true, Step.issues.join(' | '));
    // Read as at 16:41:12, its coarse box (the bottle's cap in it) holds a target_cm over it to 3 cm above its top
    // ([UNRELIABLE]), as at 16:41:12 itself: there the model went to Z 10 instead (16:41:18).
    else Assert.deepEqual(Step.issues.map(Issue => Issue.replace(/Target X .* descends onto the \[UNRELIABLE\] phone measurement.*/, 'unreliable phone')), ['unreliable phone']);
    const Down = Check(ServosHover, Filtered);
    Assert.match(Down.Line, /descend straight down to the GRASP POINT/);
    Assert.equal(Down.possible, true, `${Name}: ${Down.issues.join(' | ')}`);
  }
  // With the "phone" kept, that descent was refused for it.
  const Blocked = Check(ServosHover, Frame1636);
  Debug('--- descent with the phone kept', Blocked.possible, Blocked.issues);
  Assert.equal(Blocked.possible, false);
  Assert.ok(Blocked.issues.some(Issue => PhoneIssue.test(Issue)), Blocked.issues.join(' | '));
});

Test('real objects beside or under the gripper still count: the can beside the open jaws, the can under the raised gripper (arm_fraction up to 0.407), the flat phone, the bottle', () => {
  for (const [Name, Pose, Object_, Fraction] of Real) {
    const Servos = Snapshot(Pose), Gripper = Api.GripperSurface(Servos);
    for (const Reading of [Object_, { ...Object_, armFraction: Fraction }, { ...Object_, armFraction: 0.9 }]) {
      Assert.equal(Api.ArmMadeReason(Reading, Gripper), null, `${Name} (arm_fraction ${Reading.armFraction})`);
    }
    Assert.deepEqual(Api.WithoutArmMade(LocateOf([Object_]), Servos).Dropped, [], Name);
  }
  // The bottle and its neighbours read as at 16:41:12 with the arm at the plan's hover and grasp points: the bottle and
  // the tape stay. The 16:41:12 "phone" (its box took the bottle's cap: 13.3 x 16 cm, top Z 6.6) reads like a wide object
  // with the gripper low over its middle, which only the bridge's points could tell from the gripper: it keeps its
  // reading from before the gripper came over it, so nothing the guard holds is lost.
  for (const Servos of [ServosHover, ServosGrasp]) {
    const Clear = new Map(), At = `at ${JSON.stringify(TipOf(Servos))}`;
    Api.WithoutArmMade(LocateOf(Frame1612), Home, Clear, At1612);
    const { Located, Dropped } = Api.WithoutArmMade(LocateOf(Frame1612), Servos, Clear, At1636);
    Assert.ok(Dropped.every(Item => Item.Object.label === 'phone' && Item.Kept), At);
    Assert.deepEqual(Located.objects.map(({ armClearAt, ...Object_ }) => Object_), Frame1612, At);
  }
  // A wide real object under the gripper, with the gripper hovering higher over it than the rule's gap: it stays.
  const Board = Obj('cutting board', 0, 30, -2.4, 0.6, [20, 20], -2.4, 0.99);
  Assert.equal(Api.ArmMadeReason(Board, Api.GripperSurface(Servos1636)), null);
});

Test('the bridge\'s arm_fraction is read with each object, and is absent from a bridge that does not send it', () => {
  const Item = { label: 'phone', x: -3.6, y: 27.21, z: -2.11, top_z: 7.74, height_cm: 9.85, size_cm: [13.2, 12.8], method: 'depth', valid_fraction: 0.8 };
  const Read = Api.ReadLocateResponse({ ok: true, objects: [{ ...Item, arm_fraction: 0.356 }, Item], camera_check: { ok: true } });
  Assert.equal(Read.objects[0].armFraction, 0.356);
  Assert.ok(!('armFraction' in Read.objects[1]));
});
