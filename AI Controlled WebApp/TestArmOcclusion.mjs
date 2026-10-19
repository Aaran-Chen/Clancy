import { build as Build } from 'esbuild';
import { readFileSync as ReadFile } from 'node:fs';
import { createRequire as CreateRequire } from 'node:module';
import { test as Test } from 'node:test';
import Assert from 'node:assert/strict';
import Path from 'node:path';
import { fileURLToPath as FilePath } from 'node:url';
import { PinnedHome } from './UiTestLib.mjs';

// The task object hidden by the arm (src/lib/ArmOcclusion.ts): with the gripper over or beside
// where it was last measured, a check-in that does not measure it, or measures only part of it,
// keeps its last full measurement and GRASP PLAN, and the model is told so plainly.
//
// 2026-10-03 16:41-16:42, "grab the pill bottle" (back end/captures/2026-10-03/recording/ai.jsonl):
// bound at 16:41:12 (09:41:12 local), the chained open and hover approved; at 16:41:36 and 16:42:03
// the task query found nothing (the bottle under the open jaws); the model was told "NOT detected
// this check-in (possibly under the gripper or out of view)" and a NEXT STEP that descended "only if
// the image shows the open jaws beside it", and asked at 16:42:09. The user's "proceed" (16:42:17)
// started a task that had dropped the measurement ("NOT detected in any check-in of this task"),
// and the model asked again at 16:42:30. No bridge, no hardware, no capture files.
// The GRASP PLAN is the one the app makes from the 16:41:12 measurement NOW (not the recorded
// one: the grasp rework of 2026-10-03 moved it), so these tests read its points from the plan.
const Root = Path.dirname(FilePath(import.meta.url));
const Bundle = await Build({ stdin: { contents: ReadFile(Path.join(Root, 'src/App.tsx'), 'utf8')
  + '\nexport { buildAiSystemPrompt as AiPrompt, parseAiResponse as ParseResponse, validateAiMotionProposal as Validate, applyAiSnap as ApplySnap, aiSnapInput as SnapInput };'
  + '\nexport { SnapAiTarget } from "./lib/AiTargetSnap";'
  + '\nexport { UnreliableMeasurements } from "./lib/AiMeasurements";'
  + '\nexport { currentAngles, forwardKinematics, mirrorDisplayPoint, wristPitchFromAngles } from "./lib/robotModel";'
  + '\nexport * from "./lib/ArmOcclusion";'
  + '\nexport { FootprintDistance, GraspPlanKey, GraspRules, NeighbourObjects } from "./lib/GraspPlan";'
  + '\nexport { BuildGraspGuard, MeasuredObjectsSummary, NextStepReminder, ParseTaskObject, PlannedStepOf, ReadLocateResponse, ResolveTaskObject, TaskObjectLine,'
  + ' TaskObjectReachability } from "./lib/AiMeasurements";'
  + '\nexport { ContinueAskedTask } from "./lib/TaskObject";'
  + '\nexport { INITIAL_SERVOS, rawFromAngle, rawToFraction } from "./lib/robotModel";',
  resolveDir: Path.join(Root, 'src'), loader: 'tsx' }, bundle: true, platform: 'node', format: 'cjs', write: false, loader: { '.css': 'empty' }, plugins: [PinnedHome], logLevel: 'error' });
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
const Raw = (Pan, Lift, Elbow, Wrist, Gripper) => ({ shoulder_pan: Pan, shoulder_lift: Lift, elbow_flex: Elbow, wrist_flex: Wrist, wrist_roll: 1570, gripper: Gripper });
// The model requests' "Current servo positions" and "Current arm position" (the bridge's tip, display frame).
const Home = Snapshot(Raw(3475, 3172, 2688, 855, 702));
const Servos1636 = Snapshot(Raw(3154, 405, 1655, 854, 1599)), Tip1636 = { x: 2.8468, y: 30.9552, z: 9.4607 };
const Servos1642 = Snapshot(Raw(3132, 463, 1598, 858, 1599)), Tip1642 = { x: 4.2725, y: 31.1203, z: 8.829 };
const HomeTip = { x: -13.2082, y: 20.1789, z: 11.9369 };
const Obj = (label, x, y, z, topZ, sizeCm, supportZMeasured, validFraction, Extra = {}) => ({ label, x, y, z, topZ, heightCm: Number((topZ - z).toFixed(2)), sizeCm,
  depthMeasured: true, extrapolated: false, unstable: false, supportMeasured: true, supportZMeasured, validFraction, supportBranch: 'desk',
  taskMatch: false, taskQueryOnly: false, refined: false, outsideCalibrationCm: 0, ...Extra });
const Far = Cm => ({ extrapolated: true, outsideCalibrationCm: Cm });
// The /locate frames (ai.jsonl vision records). 16:41:12: every measured object; the bottle's height as measured (8.55).
const Bottle = { ...Obj('pill bottle', -1.56, 32.68, -2.68, 5.87, [3.3, 3], -2.45, 0.851, { taskMatch: true, refined: true }), heightCm: 8.55 };
const Frame1612 = [Obj('can', 18.72, 58.72, -1.52, 10.15, [6.2, 5], -1.4, 0.97, Far(29.5)), Obj('keyboard', 14.52, 85.46, -0.06, 1.14, [10, 23], -0.28, 0.993, Far(52.7)),
  Obj('mouse', 16.75, 109.95, 1.39, 3.84, [8.2, 9.2], 1.44, 1, Far(76.7)), Obj('phone', -1.65, 33.71, -2.28, 6.64, [13.3, 16], -2.2, 0.958),
  Bottle, Obj('tape', -0.01, 31.91, -2.96, 5.16, [5.5, 5.4], -2.65, 0.907), Obj('black box', 0.11, 28.13, -3.39, -0.77, [4.4, 3.1], -3.11, 0.999),
  Obj('soda can', 18.72, 58.73, -1.58, 10.09, [6.2, 4.1], -1.36, 0.967, Far(29.5)), Obj('tape roll', -0.01, 31.91, -2.96, 5.16, [5.5, 5.4], -2.64, 0.903),
  Obj('black cube', 0.12, 28.11, -3.37, -0.77, [4.1, 3.1], -3.07, 0.998)];
// 16:41:36 and 16:42:03: no pill bottle (the task query boxed nothing); the phone's box had the arm in it.
const Frame1636 = [Obj('can', 18.56, 58.66, -1.42, 10.23, [6.1, 5.6], -1.34, 0.97, Far(29.4)), Obj('keyboard', 14.08, 85.66, 0.06, 1.2, [9.7, 22.7], -0.17, 0.995, Far(52.7)),
  Obj('phone', -3.6, 27.21, -2.11, 7.74, [13.2, 12.8], -1.93, 0.8, { outsideCalibrationCm: 1.4 }), Obj('soda can', 18.56, 58.72, -1.41, 10.26, [6.1, 7.2], -1.28, 0.969, Far(29.4))];
const Frame1642 = [Obj('can', 18.5, 58.67, -1.45, 10.22, [6.2, 4.1], -1.41, 0.966, Far(29.4)), Obj('keyboard', 14.46, 85.57, 0.09, 1.25, [9.9, 22.7], -0.06, 0.992, Far(52.8)),
  Obj('phone', 4.35, 47.11, -2.12, -0.8, [13.3, 6.6], -1.9, 0.758, Far(13.1))];
// When it was measured (16:41:12 UTC, the recording's clock) and the check-ins after it.
const At1612 = Date.parse('2026-10-03T16:41:12.237Z'), At1636 = Date.parse('2026-10-03T16:41:36.063Z'), At1642 = Date.parse('2026-10-03T16:42:03.567Z');

const Query = Api.ParseTaskObject('grab the pill bottle');
// Planned as it was then, before the deep grasp (GraspRules.DeepGrasp, 2026-10-04: deeper in the jaws, it opens to 80%):
// the recorded check-ins had the gripper 60% open at the hover, and the occlusion rules do not depend on the depth.
const Shallow = Make => { const Saved = Api.GraspRules.DeepGrasp; Api.GraspRules.DeepGrasp = false; try { return Make(); } finally { Api.GraspRules.DeepGrasp = Saved; } };
const Reach = Shallow(() => Api.TaskObjectReachability(Bottle, { Servos: Home, ElbowMode: 'up', ClearanceCm: 1 }, { Others: Api.NeighbourObjects(Frame1612, Bottle), Kind: Query.Name }));
const Plan = Reach.Plan;
// The plan's points and moves as the TASK OBJECT line prints them (AiMeasurements.ts GraspableText, NextPlannedGraspStep).
const Escape = Text => Text.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
const Point = P => `X ${P.x.toFixed(1)}, Y ${P.y.toFixed(1)}, Z ${P.z.toFixed(1)}`;
const Deg = Pitch => Number(Pitch.toFixed(1));
const StepCm = Value => Math.abs(Value) < 0.05 ? 0 : Number(Value.toFixed(1));
const PlanText = Escape(`RIGHT ARM (from that measurement): graspable: GRASP POINT ${Point(Plan.Grasp)} with wrist ${Deg(Plan.Pitch)} deg; HOVER POINT ${Point(Plan.Hover)} with wrist ${Deg(Plan.HoverPitch)} deg`);
const Descent = Escape(`descend straight down to the GRASP POINT Z ${Plan.Grasp.z.toFixed(1)}:`);
const DescentFrom = Tip => ({ move_delta_cm: { left: StepCm(Plan.Grasp.x - Tip.x), farther: StepCm(Plan.Grasp.y - Tip.y), up: StepCm(Plan.Grasp.z - Tip.z) },
  wrist_pitch_degrees: Deg(Plan.Pitch) });
// The arm stopped at the plan's HOVER POINT: the pose the plan checked there, the gripper 60% open as recorded.
const IkJoints = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll'];
const ServosHover = Snapshot({ ...Object.fromEntries(IkJoints.map(Name => [Name, Math.round(Api.rawFromAngle(Api.INITIAL_SERVOS[Name], Plan.Poses.Hover[Name]))])), gripper: 1599 });
const TipHover = Api.mirrorDisplayPoint(Api.forwardKinematics(Api.currentAngles(ServosHover)).at(-1));
const Seen ={ At: At1612, Object: Bottle, Reachability: Reach, Motions: 0 };
const PlanState = { Reach, Source: Bottle, At: At1612, Key: Api.GraspPlanKey(Home, 'up'), Note: null, Confirmed: false, Corridor: null };
// The task object as the app keeps it after a check-in (App.tsx), with this check-in's occlusion verdict.
const TaskState = (Occluded, Extra = {}) => ({ Label: Query.Name, Query, Intent: 'grasp', DetectedNow: false, Binding: null, Candidates: [],
  LastSeen: Seen, Reachability: null, Plan: PlanState, Occluded, ...Extra });
const Judge = (Reading, Tip, Now, Motions, Options = {}) => Api.JudgeArmOcclusion(Seen, Reading, Tip, { Plan, Closed: false, Missed: false, Now, Motions, ...Options });
const GraspAt = Servos => ({ GripperFraction: Api.rawToFraction(Servos.gripper, Servos.gripper.presentRaw), Closed: false, Held: null });
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
const Debug = process.env.ShowLines ? (...Args) => console.log(...Args) : () => {};

Test('16:41:12 "grab the pill bottle": the GRASP PLAN from that measurement puts the fixed jaw beside it (not over its centre), and the arm at its hover or grasp point covers it', () => {
  Assert.equal(Reach.Kind, 'graspable', Reach.Text);
  // Straight above the grasp point, beside the bottle's measured footprint (and its planned prism), 60% open.
  Assert.deepEqual({ x: Plan.Hover.x, y: Plan.Hover.y }, { x: Plan.Grasp.x, y: Plan.Grasp.y });
  Assert.ok(Plan.Hover.z > Bottle.topZ && Plan.Grasp.z < Bottle.topZ, `hover Z ${Plan.Hover.z}, grasp Z ${Plan.Grasp.z}`);
  for (const Footprint of [Plan.Prism.Footprint, [{ x: Bottle.x - 1.65, y: Bottle.y - 1.5 }, { x: Bottle.x + 1.65, y: Bottle.y - 1.5 }, { x: Bottle.x + 1.65, y: Bottle.y + 1.5 }, { x: Bottle.x - 1.65, y: Bottle.y + 1.5 }]]) {
    const Beside = Api.FootprintDistance(Footprint, Plan.Grasp);
    Assert.ok(Beside > 0 && Beside <= Api.GraspRules.NearCm, `the grasp point ${Beside.toFixed(2)} cm from its footprint`);
  }
  Assert.equal(Plan.OpenFraction, 0.6);
  // The occlusion zone covers the plan's points: with the tip at the hover point or the grasp point, the arm is over it.
  for (const At of [Plan.Hover, Plan.Grasp, TipHover]) Assert.ok(Api.ArmOver(Bottle, At, Plan), `the arm at ${Point(At)} is over the bottle`);
  // The task query found nothing under the gripper: no measured object is the bottle at either check-in.
  for (const [Frame, Tip] of [[Frame1636, Tip1636], [Frame1642, Tip1642]]) {
    Assert.equal(Api.ResolveTaskObject(Query, Frame, { LastSeen: Bottle, Track: true, Tip }).Binding, null);
  }
});

Test('16:41:36 and 16:42:03: not measured with the open jaws over it, the bottle is HIDDEN BY THE GRIPPER, its 16:41:12 measurement and plan stand, and the NEXT STEP goes on from it', () => {
  for (const [Name, Tip, Servos, Now, Motions, Ago] of [['1636', Tip1636, Servos1636, At1636, 1, '24 s and 1 motion ago'], ['1642', Tip1642, Servos1642, At1642, 2, '51 s and 2 motions ago']]) {
    const Occluded = Judge(null, Tip, Now, Motions);
    Assert.ok(Occluded, `${Name}: the arm is over it`);
    Assert.equal(Occluded.Partial, null);
    Assert.ok(Occluded.TipCm <= Api.GraspRules.NearCm, `${Name}: ${Occluded.TipCm}`);
    const Line = Api.TaskObjectLine(TaskState(Occluded), Tip, GraspAt(Servos));
    Debug(`--- ${Name}\n${Line}`);
    Assert.match(Line, /^TASK OBJECT "pill bottle": HIDDEN BY THE GRIPPER, not lost: not measured this check-in because the gripper hides it \(the tip is \d+\.\d cm from its measured footprint, so the arm covers it in this image\)\./);
    Assert.match(Line, new RegExp(`Its last full measurement stands as its working measurement: centre X -1\\.6, Y 32\\.7, base Z -2\\.7, top Z 5\\.9 \\(8\\.6 cm tall, footprint about 3\\.3 x 3\\.0 cm\\), measured at \\d\\d:\\d\\d:\\d\\d, ${Ago}; the app keeps it and the GRASP PLAN made from it\\.`));
    Assert.match(Line, /Do not ask the user to confirm it from the image, and do not reply ask because you cannot see it: follow the NEXT STEP\. Reply ask only if the image shows it somewhere else, clear of the gripper\./);
    Assert.match(Line, new RegExp(PlanText));
    // From where the arm stopped (off the plan's points), the step goes on from the last measurement: it does not
    // wait for the image to show the jaws beside a bottle the jaws hide. Its motion ends at the plan's hover or grasp point.
    Assert.match(Line, new RegExp(`NEXT STEP of the grasp \\(tip now [^)]*; gripper 60% open\\): .*[Tt]he pill bottle was measured once so far and the gripper now hides it \\(expected with the jaws over it: the image cannot show the open jaws beside it on its ${Plan.Side} side, and that is no reason to ask; reply ask only if the image shows it somewhere else, clear of the gripper\\), so go on from its last measurement`));
    Assert.doesNotMatch(Line, /descend only if the image shows|NOT detected|possibly under the gripper/);
    const Motion = MotionIn(Line);
    const End = Motion.target_cm ?? { x: Tip.x + Motion.move_delta_cm.left, y: Tip.y + Motion.move_delta_cm.farther, z: Tip.z + Motion.move_delta_cm.up };
    Assert.ok([Plan.Hover, Plan.Grasp].some(P => Math.hypot(End.x - P.x, End.y - P.y, End.z - P.z) <= 0.15), `${Name}: the step ends at ${Point(End)}`);
    // The guard holds the moves near the bottle to that step, and the prompt ends with it.
    const Step = Api.PlannedStepOf(Line, Tip, Api.wristPitchFromAngles(Api.currentAngles(Servos)));
    const Guard = Api.BuildGraspGuard(Name === '1636' ? Frame1636 : Frame1642, TaskState(Occluded), false, Step);
    Assert.equal(Guard.Step, Step);
    Assert.match(Api.NextStepReminder(Guard.Step, 'pill bottle'), /^NEXT STEP TO COPY .*"motion_proposal":\{"(target_cm|move_delta_cm)":\{/);
  }
});

Test('the arm stopped at the plan\'s HOVER POINT with the bottle hidden: the NEXT STEP descends straight down to the GRASP POINT from its last measurement', () => {
  const Occluded = Judge(null, TipHover, At1642, 2);
  Assert.ok(Occluded, 'the arm at the hover point is over it');
  Assert.equal(Occluded.Partial, null);
  const Line = Api.TaskObjectLine(TaskState(Occluded), TipHover, GraspAt(ServosHover));
  Debug(`--- hover\n${Line}`);
  Assert.match(Line, /^TASK OBJECT "pill bottle": HIDDEN BY THE GRIPPER, not lost: /);
  Assert.match(Line, new RegExp(`NEXT STEP of the grasp \\(tip now [^)]*; gripper 60% open\\): the tip is above the grasp point, but the pill bottle was measured once so far and the gripper now hides it \\(expected with the jaws over it: the image cannot show the open jaws beside it on its ${Plan.Side} side, and that is no reason to ask; reply ask only if the image shows it somewhere else, clear of the gripper\\), so go on from its last measurement: ${Descent}`));
  Assert.doesNotMatch(Line, /descend only if the image shows|NOT detected|possibly under the gripper/);
  Assert.deepEqual(MotionIn(Line), DescentFrom(TipHover), 'straight down to the plan\'s grasp point');
});

// The check-in's NEXT STEP, copied exactly, as the app reads, snaps and checks a reply.
function CheckStep(Tip, Servos, Now, Motions, Frame) {
  const Occluded = Judge(null, Tip, Now, Motions);
  const State = TaskState(Occluded), Grasp = GraspAt(Servos);
  const Line = Api.TaskObjectLine(State, Tip, Grasp);
  const Angles = Api.currentAngles(Servos);
  const Anchor = { ...Api.forwardKinematics(Angles).at(-1), wristPitch: Api.wristPitchFromAngles(Angles) };
  const Step = Api.PlannedStepOf(Line, Api.mirrorDisplayPoint(Anchor), Anchor.wristPitch);
  const Guard = Api.BuildGraspGuard(Frame, State, false, Step);
  const Unreliable = Api.UnreliableMeasurements(Frame, State);
  const Parsed = Api.ParseResponse(`{"response_to_user":"","task_status":"continue","motion_proposal":${Step.Json}}`, Anchor, Servos).motion;
  const Guarded = { ...Parsed, guard: Guard };
  const Snapped = Api.ApplySnap(Guarded, Api.SnapAiTarget(Api.SnapInput(Guarded, Servos, 'up', undefined, Unreliable)));
  return Api.Validate(Snapped, 'up', Servos, Unreliable, undefined, { Object: Bottle, State: Grasp, Plan }, Guard);
}

Test('16:42:03, and at the hover point: the NEXT STEP copied exactly passes the app\'s checks (read from the tip, snapped, validated against the guard), so it reaches the user to approve', () => {
  const Check = CheckStep(Tip1642, Servos1642, At1642, 2, Frame1642);
  Assert.equal(Check.possible, true, Check.issues.join(' | '));
  // The descent from the hover point, with the objects measured at 16:42:03 (the 16:41:36 frame's arm-made "phone": TestArmMadeObjects.mjs).
  const Down = CheckStep(TipHover, ServosHover, At1642, 2, Frame1642);
  Assert.equal(Down.possible, true, Down.issues.join(' | '));
  if (process.env.ShowLines) { const Early = CheckStep(Tip1636, Servos1636, At1636, 1, Frame1636); Debug('--- 1636 check', Early.possible, Early.issues); }
});

Test('16:42:17 "proceed": the answer keeps the asked task\'s object, and with the arm still over it the new task goes on from its measurement', () => {
  const Asked = { Task: 'grab the pill bottle', Question: 'I cannot clearly see the pill bottle in the image to confirm the open jaws are beside it on its left-near side.', Query };
  const Answered = Api.ContinueAskedTask('proceed', Asked);
  Assert.match(Answered.Task, /^grab the pill bottle \(you asked: "I cannot clearly see the pill bottle[^"]*"; the user answered: "proceed"\)/);
  // App.tsx keeps the asked task's measurement and plan for the same object (the same query).
  Assert.equal(Answered.Query, Query);
  Assert.ok(!Answered.Stopped);
  // Taken up at the first check-in (16:42:26, the bottle not measured, the arm where it stopped): hidden, not "NOT detected in any check-in".
  const Carried = { ...Seen, Motions: 0 - 2 };
  const Occluded = Api.JudgeArmOcclusion(Carried, null, Tip1642, { Plan, Closed: false, Missed: false, Now: Date.parse('2026-10-03T16:42:26.774Z'), Motions: 0 });
  Assert.ok(Occluded);
  Assert.equal(Occluded.MotionsAgo, 2, 'the motions of the asked task count');
  const Line = Api.TaskObjectLine(TaskState(Occluded, { LastSeen: Carried }), Tip1642, GraspAt(Servos1642));
  Debug(`--- proceed\n${Line}`);
  Assert.match(Line, /^TASK OBJECT "pill bottle": HIDDEN BY THE GRIPPER, not lost: .*measured at \d\d:\d\d:\d\d, 75 s and 2 motions ago; the app keeps it and the GRASP PLAN made from it\./);
  Assert.match(Line, new RegExp(PlanText));
  Assert.match(Line, /the gripper now hides it \([^)]*\), so go on from its last measurement/);
  // Stopped at the hover point, the step from that measurement is the descent.
  const AtHover = Api.JudgeArmOcclusion(Carried, null, TipHover, { Plan, Closed: false, Missed: false, Now: Date.parse('2026-10-03T16:42:26.774Z'), Motions: 0 });
  Assert.match(Api.TaskObjectLine(TaskState(AtHover, { LastSeen: Carried }), TipHover, GraspAt(ServosHover)), new RegExp(`so go on from its last measurement: ${Descent}`));
  // With the arm away from it, an answer's carried measurement is not taken up (it may have moved): as before.
  Assert.equal(Api.JudgeArmOcclusion(Carried, null, HomeTip, { Plan, Closed: false, Missed: false, Now: At1642, Motions: 0 }), null);
  Assert.match(Api.TaskObjectLine({ ...TaskState(null), LastSeen: null, Plan: null }, HomeTip, GraspAt(Home)), /^TASK OBJECT "pill bottle": NOT detected in any check-in of this task\./);
});

Test('measured only partly with the arm over it: the partial reading is not used, the last full measurement and plan stand, and the measured list says so', () => {
  // The bottle's top and one side seen past the jaws: narrower, fewer valid depth points.
  const Part = { ...Bottle, x: -1.1, y: 32.2, sizeCm: [1.6, 2.9], validFraction: 0.52, topZ: 5.7, heightCm: 8.4, refined: true };
  Assert.ok(Api.PartlyMeasured(Part, Bottle));
  const Occluded = Judge(Part, Tip1642, At1642, 2);
  Assert.equal(Occluded.Partial, Part);
  const Note = 're-measured at X -1.1, Y 32.2 with the gripper over it (size 1.6 x 2.9, was 3.3 x 3.0): the GRASP PLAN keeps the 09:41:12 measurement';
  const State = TaskState(Occluded, { DetectedNow: true, Binding: { Object: Part, Source: 'query', DistanceCm: 0.7 }, Reachability: Reach, Plan: { ...PlanState, Note } });
  const Line = Api.TaskObjectLine(State, Tip1642, GraspAt(Servos1642));
  Debug(`--- partial\n${Line}`);
  Assert.match(Line, /^TASK OBJECT "pill bottle": HIDDEN BY THE GRIPPER, not lost: measured only partly this check-in \(as "pill bottle" at X -1\.1, Y 32\.2, top Z 5\.7, footprint about 1\.6 x 2\.9 cm\) because the gripper hides part of it, so that reading is not used \(the tip is/);
  Assert.match(Line, /Its last full measurement stands as its working measurement: centre X -1\.6, Y 32\.7, base Z -2\.7, top Z 5\.9 \(8\.6 cm tall, footprint about 3\.3 x 3\.0 cm\)/);
  Assert.match(Line, / The GRASP PLAN: re-measured at X -1\.1, Y 32\.2 with the gripper over it/);
  Assert.match(Line, new RegExp(PlanText));
  Assert.match(Line, /the gripper now hides it \([^)]*\), so go on from its last measurement/);
  // The same partial reading with the arm at the hover point: the descent, from the last full measurement's plan.
  const AtHover = Judge(Part, TipHover, At1642, 2);
  Assert.equal(AtHover.Partial, Part);
  Assert.match(Api.TaskObjectLine({ ...State, Occluded: AtHover }, TipHover, GraspAt(ServosHover)), new RegExp(`so go on from its last measurement: ${Descent}`));
  const Located = Api.ReadLocateResponse({ ok: true, objects: [], rms_cm: 0.75, camera_check: { ok: true, angle_deg: 0.1, offset_cm: 0, reason: '' }, withheld: [] });
  Located.objects = [...Frame1642, Part];
  Assert.match(Api.MeasuredObjectsSummary(Located, Part, true), /- pill bottle: centre X=-1\.1, Y=32\.2[^\n]* <- the task object, measured only partly \(the gripper hides part of it\): use its TASK OBJECT line's last full measurement/);
  Assert.match(Api.MeasuredObjectsSummary(Located, Part), /- pill bottle: [^\n]* <- the task object$/m);
  // 2026-10-03 14:50:40: the can read 5.0 x 8.6 with the jaws beside it at the grasp point (5.6 x 4.9 at 14:48:22): measured with the jaws, partly.
  const Can = Obj('can', -3.96, 30.66, -3.37, 8.3, [5.6, 4.9], -3.2, 0.829), Merged = Obj('can', -4.36, 30.08, -3.31, 8.57, [5, 8.6], -3.2, 0.839);
  Assert.ok(Api.PartlyMeasured(Merged, Can));
  Assert.ok(Api.JudgeArmOcclusion({ At: 0, Object: Can }, Merged, { x: 2.0, y: 28.4, z: 4.1 }, { Plan: null, Closed: false, Missed: false, Now: 0, Motions: 3 }).Partial);
});

Test('not seen with the arm away from its last spot, or after a close: still a lost object, as before', () => {
  // The arm at home, 17 cm from it; high above it; or the jaws closed (it may be in them), or closed on nothing.
  Assert.equal(Judge(null, HomeTip, At1642, 2), null);
  Assert.equal(Judge(null, { x: Tip1642.x, y: Tip1642.y, z: Bottle.topZ + 12 }, At1642, 2), null);
  Assert.equal(Judge(null, Tip1642, At1642, 2, { Closed: true }), null);
  Assert.equal(Judge(null, Tip1642, At1642, 2, { Missed: true }), null);
  Assert.equal(Api.JudgeArmOcclusion(null, null, Tip1642, { Plan, Closed: false, Missed: false, Now: At1642, Motions: 2 }), null, 'never measured: nothing to keep');
  // A full re-measure with the arm over it replaces the last one (the plan lock judges it); a partial one 5 cm off may have been pushed.
  Assert.equal(Judge({ ...Bottle, x: -1.4, y: 32.5, validFraction: 0.83 }, Tip1642, At1642, 2), null);
  Assert.equal(Judge({ ...Bottle, x: -1.6, y: 37.9, sizeCm: [1.5, 2.8], validFraction: 0.5 }, Tip1642, At1642, 2), null);
  // Without the verdict the line reads as before: NOT detected, last measured, no HIDDEN and no step away from it.
  const Line = Api.TaskObjectLine(TaskState(null), HomeTip, GraspAt(Home));
  Debug(`--- away\n${Line}`);
  Assert.match(Line, /^TASK OBJECT "pill bottle": NOT detected this check-in \(possibly under the gripper or out of view\); last measured at \d\d:\d\d:\d\d at X -1\.6, Y 32\.7\./);
  Assert.doesNotMatch(Line, /HIDDEN|NEXT STEP/);
});

Test('the system prompt\'s HIDDEN RULE, and the chained-motion rule\'s image check that allows for it', () => {
  const Prompt = Api.AiPrompt('', true, true, 'Behind');
  Assert.match(Prompt, /HIDDEN RULE: when the TASK OBJECT line says the object is HIDDEN BY THE GRIPPER, the arm is over or beside where the camera last measured it and covers it in the image: that is normal while grasping, not a lost object\. Its last full measurement and GRASP PLAN stand: do not ask the user to confirm it from the image, and do not reply ask because you cannot see it; follow the NEXT STEP\./);
  Assert.match(Prompt, /\(always its own motion, so the next image shows the open jaws beside it before the close, unless the gripper hides it there\)/);
});
