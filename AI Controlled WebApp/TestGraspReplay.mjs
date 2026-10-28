import { build as Build } from 'esbuild';
import { readFileSync as ReadFile } from 'node:fs';
import { createRequire as CreateRequire } from 'node:module';
import { test as Test } from 'node:test';
import Assert from 'node:assert/strict';
import Path from 'node:path';
import { fileURLToPath as FilePath } from 'node:url';
import { PinnedHome } from './UiTestLib.mjs';

// Replays of recorded grasp runs against the GRASP PLAN (src/lib/GraspPlan.ts) and its steps
// (src/lib/AiMeasurements.ts): the recorded measurements and servo raws, and where the real arm
// put the real gripper. No bridge, no hardware, no capture files (they are not in git).
//
// 2026-10-01 15:05-15:10, "Grab the can" (back end/captures/2026-10-01/recording/ai.jsonl): the
// plan put the fixed pad 1.2 cm from the can; the descent pushed the can 0.6 cm (the real fixed
// finger stood nearer it than the arm model put it) and stopped 0.47 cm under the grasp point, at
// the mid-height the plan had raised it from; the close squeezed the can 3 cm out along the
// fingers; the plan lock kept the old plan, and the second close went down where the can had been
// and closed on air. The push model calls that first close held, so the close is judged here by
// where the push leaves the can against the fixed finger's tip, not by the model's verdict.
//
// 2026-10-01 18:53-18:56, "grab the can" (the same recording): the arm held the hover point but
// stopped 0.5 cm toward the can, and the NEXT STEP re-aimed the hover point past it; the model
// descended from where the arm stopped instead, and its revision (down to the grasp height, 0.5 cm
// toward the can) was approved. App.tsx is bundled with the libs (one module instance): its parser,
// validator and snap check the replies as the app did.
const Root = Path.dirname(FilePath(import.meta.url));
const Bundle = await Build({ stdin: { contents: ReadFile(Path.join(Root, 'src/App.tsx'), 'utf8')
  + '\nexport { parseAiResponse as ParseResponse, validateAiMotionProposal as Validate, applyAiSnap as ApplySnap, aiSnapInput as SnapInput,'
  + ' HOME_MODEL_TARGET_CM as HomeTarget };'
  + '\nexport * from "./lib/GraspPlan";'
  + '\nexport * as Sweep from "./lib/JawSweep";'
  + '\nexport { BuildGraspGuard, ClosedOnNothing, NextStepOf, NextStepReminder, NoteGripperMove, PlannedStepOf, ReachScan, TaskObjectLine, TaskObjectReachability } from "./lib/AiMeasurements";'
  + '\nexport { DescribeMotionFact, NoOpReason, PlannedStepFact, PlannedStepNote, PlannedStepReply } from "./lib/AiTaskLoop";'
  + '\nexport { SnapAiTarget } from "./lib/AiTargetSnap";'
  + '\nexport { INITIAL_SERVOS, IK_JOINTS, angleFromRawValue, currentAngles, forwardKinematics, fractionToRaw, mirrorDisplayPoint,'
  + ' modelPointFromDisplayPoint, rawFromAngle, rawToFraction, solveIkPreview, wristPitchFromAngles } from "./lib/robotModel";',
  resolveDir: Path.join(Root, 'src'), loader: 'tsx' }, bundle: true, platform: 'node', format: 'cjs', write: false, loader: { '.css': 'empty' }, plugins: [PinnedHome], logLevel: 'error' });
globalThis.window ??= { location: { hostname: 'localhost' } };
const Module = { exports: {} };
new Function('require', 'module', 'exports', Bundle.outputFiles[0].text)(CreateRequire(import.meta.url), Module, Module.exports);
const Api = Module.exports, Rules = Api.GraspRules, Sweep = Api.Sweep;

// --- The recorded run -------------------------------------------------------------------------

const Snapshot = Raw => {
  const Servos = structuredClone(Api.INITIAL_SERVOS);
  for (const [Name, Value] of Object.entries(Raw)) Object.assign(Servos[Name], { presentRaw: Value, targetRaw: Value, guiAngleRaw: null });
  return Servos;
};
const Raw = (Pan, Lift, Elbow, Wrist, Gripper) => ({ shoulder_pan: Pan, shoulder_lift: Lift, elbow_flex: Elbow, wrist_flex: Wrist, wrist_roll: 1521, gripper: Gripper });
// The servo raws at each check-in (the model requests' "Current servo positions"); their FK is the logged tip.
const Raws = {
  Home: Raw(3473, 3171, 2688, 863, 701), Aimed: Raw(3101, 262, 1841, 714, 1917), Grasp1: Raw(3105, 484, 1986, 339, 1917),
  Closed1: Raw(3105, 484, 1986, 339, 650), Risen: Raw(3105, 254, 1832, 734, 650), Hover2: Raw(3105, 258, 1836, 734, 1917),
  Grasp2: Raw(3107, 484, 1987, 338, 1917), Closed2: Raw(3107, 484, 1987, 338, 648) };
// The first descent the arm was sent (approved 15:07:42, from the hover aimed past), wrist -23.1.
const Descent1 = { x: 4.602039800571507, y: 28.570478514704895, z: 4.1072343274824465 };
const TipOf = Servos => Api.mirrorDisplayPoint(Api.forwardKinematics(Api.currentAngles(Servos)).at(-1));
const Obj = (label, x, y, z, topZ, sizeCm, supportZMeasured, validFraction, Extra = {}) => ({ label, x, y, z, topZ, heightCm: topZ - z, sizeCm,
  depthMeasured: true, extrapolated: false, unstable: false, supportMeasured: true, supportZMeasured, validFraction, supportBranch: 'desk',
  taskMatch: label === 'can', taskQueryOnly: false, refined: label === 'can', outsideCalibrationCm: 0, ...Extra });
const Far = { extrapolated: true, outsideCalibrationCm: 60 }, Bin = { supportBranch: 'bin', extrapolated: true, outsideCalibrationCm: 16 };
// The /locate frames (ai.jsonl vision records), every measured object.
const Frame0536 = [Obj('keyboard', 22.61, 90.6, 2.14, 3.29, [9.8, 23.4], 2.05, 1, Far), Obj('can', 0.51, 30.47, -2.37, 9.4, [5.4, 5.3], -2.07, 0.889),
  Obj('bottle', 11.74, 31.48, -2.29, 5.61, [3.4, 3.1], -2.15, 0.836), Obj('tape', 7.63, 26.44, -1.63, 1.44, [3.2, 0.6], -1.9, 0.992),
  Obj('pencil sharpener', 6.65, 28.29, -2.18, 2.59, [4.4, 2.4], -2.03, 0.914), Obj('bucket', -12.66, 31.62, -2.1, 0.19, [8.7, 9], -2.21, 0.616),
  Obj('pliers', -4.23, 13.11, -8.11, -6.42, [19.7, 7.3], -8.27, 0.923, Bin), Obj('wrench', 1.16, 8.41, -8.18, -5.83, [17.4, 9.9], -8.31, 0.888, Bin),
  Obj('cable', 1.06, 8.44, -8.26, -5.85, [18.6, 10.2], -8.31, 0.823, Bin), Obj('pen', 22.16, 105.01, 3.43, 4.43, [8.5, 3.2], 3.77, 1, Far),
  Obj('plastic container', -12.47, 24.12, -2.11, 0.38, [8.7, 9.1], -2.25, 0.599)];
// After the first close: the can 2.9 cm out along the fingers.
const Frame0815 = [Obj('keyboard', 22.53, 90.56, 2.28, 3.39, [9.7, 23.3], 2.19, 1, Far), Obj('can', 2.35, 32.73, -2.14, 9.68, [4.7, 5.4], -0.86, 0.782),
  Obj('bottle', 11.77, 31.43, -2.24, 5.63, [3.4, 3.2], -2.08, 0.813), Obj('tape', 7.59, 26.54, -1.97, 1.25, [4.5, 1.7], -1.98, 0.961),
  Obj('battery', 7.57, 27.51, -2.03, 2.59, [4.5, 1.7], -1.96, 0.914), Obj('cup', -13.81, 31.7, -2.08, 0.13, [9.1, 4.3], -2.04, 0.765),
  Obj('tissue box', -13.58, 31.64, -2.08, 0.12, [9.1, 4.2], -2.04, 0.765), Obj('pliers', 5, 14.46, -8.14, -6.57, [10.8, 3], -8.23, 0.616, Bin),
  Obj('wrench', 1.69, 8.26, -8.42, -5.67, [10.1, 5.3], -8.44, 0.853, Bin)];
// At the hover after rising, the jaws open: the can partly hidden (4.4 x 3.5, 40% valid).
const Frame0927 = [Obj('bottle', 11.76, 31.52, -2.26, 5.6, [3.2, 3.4], -2.01, 0.815), Obj('can', 2.78, 33.29, -1.95, 9.98, [4.4, 3.5], -0.89, 0.395),
  Obj('tape', 4.85, 25.73, -2.54, 2.31, [7.2, 4.4], -2.23, 0.918), Obj('battery', 4.36, 25.77, -2.25, 3.24, [4.6, 3.3], -1.99, 0.814),
  Obj('pill bottle', 11.76, 31.45, -2.25, 5.6, [3.3, 3.2], -2.01, 0.81)];
// After the second close, the can where it stood since the first (15:09:45 read it 2.43, 33.06).
const Frame1006 = [Obj('can', 2.4, 33.09, -1.93, 9.91, [4.8, 5.3], -1.04, 0.786), Obj('bottle', 11.77, 31.46, -2.27, 5.6, [3.3, 3.5], -2.13, 0.805),
  Obj('battery', 7.51, 27.43, -1.97, 2.61, [4.5, 1.7], -1.93, 0.921), Obj('tape', 7.08, 26.62, -1.82, 0.83, [4.5, 1.6], -1.92, 0.925),
  Obj('bowl', -13.35, 31.61, -2.04, 0, [9, 4.4], -2, 0.767), Obj('pill bottle', 11.78, 31.46, -2.24, 5.63, [3.3, 3.1], -2.03, 0.797),
  Obj('tape roll', 7.25, 26.5, -1.73, 1.28, [4.5, 1.2], -1.9, 0.943)];
const CanOf = Frame => Frame.find(Object_ => Object_.label === 'can');

// --- Where the real arm puts the real gripper -----------------------------------------------------

// The diagnosis of the run (camera, depth and video of 15:07:47-15:08:05): measured against the app's
// can, the real fixed finger stood 1.69 cm nearer it along the jaw axis than the arm model puts it
// (the FK gap at the stop, 1.09 cm to a 6.6 cm can, plus the 0.6 cm the descent pushed the can), from
// the push's 0.55 to 0.7 cm and the finger's depth edge: checked at 1.69 and 0.25 cm either side, and at
// the 2.1 cm the depth edge reached (GraspRules.FixedFingerToleranceCm over the plan's offset).
const RealOffsetsCm = [1.44, 1.69, 1.94], CentralOffsetCm = 1.69, UpperOffsetCm = 2.1, TrueCanCm = 6.6;
const Disc = (Centre, D, Sides = 96) => { const R = D / 2 / Math.cos(Math.PI / Sides);
  return Array.from({ length: Sides }, (_, I) => ({ x: Centre.x + R * Math.cos(2 * Math.PI * I / Sides), y: Centre.y + R * Math.sin(2 * Math.PI * I / Sides) })); };
// The real can where the real gripper meets it (the arm's frame): the measured centre moved the offset toward the fixed jaw,
// and Along cm toward the finger tips (the camera's diagnosis put the real gripper 0.4 to 1.0 cm nearer the palm than the
// model, unconfirmed: the replay checks 0 and 0.4).
const RealCan = (Can, U, OffsetCm, Along = 0, A = { x: 0, y: 0 }) => { const Centre = { x: Can.x - OffsetCm * U.x + Along * A.x, y: Can.y - OffsetCm * U.y + Along * A.y };
  return { Label: 'can', BaseZ: Can.z, TopZ: Can.topZ, SupportZ: Can.supportZMeasured, Centre, SizeCm: [TrueCanCm, TrueCanCm], Footprint: Disc(Centre, TrueCanCm), Round: null }; };
const Model = (Point, Pitch) => ({ ...Api.modelPointFromDisplayPoint(Point), wristPitch: Pitch });
const Solve = (Point, Pitch, Seed) => Api.solveIkPreview(Model(Point, Pitch), 'up', Seed);
const RawsAt = (Angles, Servos) => Object.fromEntries(Api.IK_JOINTS.map(Joint => [Joint, Api.rawFromAngle(Servos[Joint], Angles[Joint])]));
// The first descent's joint stop errors (raw steps): where the bridge stopped each joint against its target.
const Hovered = Snapshot(Raws.Aimed);
const Wanted1 = RawsAt(Solve(Descent1, -23.1, Hovered).angles, Hovered);
const StopRaw = Object.fromEntries(Api.IK_JOINTS.map(Joint => [Joint, Raws.Grasp1[Joint] - Wanted1[Joint]]));
const OpenDeg = Api.angleFromRawValue(Api.INITIAL_SERVOS.gripper, 1917);
// The fixed finger's tip along the fingers (gripper z), and how far inside it a close leaves the can's contact with it.
const FingertipZ = Math.min(...Sweep.FixedFingerPolygon.map(P => P.z));
const Level = V => { const L = Math.hypot(V.x, V.y); return { x: V.x / L, y: V.y / L }; };
/** The push on a real can at a pose: the close (CloseOn), the contact's distance inside the fixed finger's tip, the grip in 3D, the pad's gap. */
function CloseAt(Stop, Real) {
  const Frame = Api.JawFrame(Stop), Slice = Api.SliceInJawPlane(Real, Frame), Close = Sweep.CloseOn(Slice, OpenDeg, Sweep.SweepRules, 'push');
  return { Close, Gap: Sweep.Gap(Sweep.FixedFingerPolygon, Slice),
    TipCm: Close.Contacts ? Math.min(...Close.Contacts.Fixed.Patch.map(P => P.z)) - FingertipZ : null,
    Grip3DDeg: Close.Contacts ? Sweep.GripIn3D(Close.Contacts, Frame.U, Frame.H, Frame.A) : null };
}
/**
 * A descent of a plan from its hover point to its printed grasp point on the arm: each joint stops
 * with the first descent's raw error. Where the real fixed finger, palm and open jaw came nearest
 * the real can on the way down, and the close there on the real can (CloseAt).
 */
function OnTheArm(Plan, Can, OffsetCm, Along = 0) {
  const Hover = Solve(Plan.Hover, Plan.HoverPitch, Snapshot(Raws.Home)).angles;
  const Servos = Snapshot(Raws.Home);
  const Target = RawsAt(Solve(Plan.Grasp, Plan.Pitch, Snapshot({ ...Raws.Home, ...RawsAt(Hover, Servos) })).angles, Servos);
  const Stopped = Snapshot({ ...Raws.Grasp1, ...Object.fromEntries(Api.IK_JOINTS.map(Joint => [Joint, Target[Joint] + StopRaw[Joint]])) });
  const Stop = { ...Api.currentAngles(Stopped), gripper: OpenDeg };
  const Real = RealCan(Can, Plan.JawAxis, OffsetCm, Along, Level(Api.JawFrame(Stop).A));
  let Least = Infinity;
  for (let Index = 0; Index <= 20; Index++) {
    const S = Index / 20, Angles = { ...Hover, gripper: OpenDeg };
    for (const Joint of ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex']) Angles[Joint] = Hover[Joint] + S * (Stop[Joint] - Hover[Joint]);
    const Hit = Api.DeepestContact(Api.GripperSamples(Angles, [OpenDeg, OpenDeg - Rules.JawUncertaintyDeg]), [Real], 3);
    if (Hit) Least = Math.min(Least, 3 - Hit.DepthCm);
  }
  return { Tip: TipOf(Stopped), Least, ...CloseAt(Stop, Real) };
}

// The plan as the app printed it live (no real-arm corrections, the closing as it was judged then), as this branch first
// planned it (the finger offset 1.5 and the sag, the closing as before), and as the 2026-10-01 fix planned it (As1001: that
// day's calibration put the real fixed finger 1.7 cm nearer the can than the arm model, GraspRules.FixedFingerOffsetCm; the
// calibration of 2026-10-02 21:09 shows none, and 2026-10-03 added the straddle and the preferred wrist pitch: the replays below
// hold that fix to its run).
const With = (Changes, Make) => {
  const Saved = Object.fromEntries(Object.keys(Changes).map(Name => [Name, Rules[Name]]));
  Object.assign(Rules, Changes);
  try { return Make(); } finally { Object.assign(Rules, Saved); }
};
const AsLive = Make => With({ FixedFingerOffsetCm: 0, GraspSagCm: 0, StrictClosing: false, StraddleRoundObjects: false, PreferredPitchDeg: null, BestEffort: false, DeepGrasp: false, FixedJawExtraStandoffCm: 0 }, Make);
// The two passes before the best-effort one (GraspRules.BestEffort false, 2026-10-04): their refusals.
const Strictly = Make => With({ BestEffort: false }, Make);
const AsBefore = Make => With({ FixedFingerOffsetCm: 1.5, GraspSagCm: 0.45, StrictClosing: false, StraddleRoundObjects: false, PreferredPitchDeg: null }, Make);
const As1001 = Make => With({ FixedFingerOffsetCm: 1.7, StraddleRoundObjects: false, PreferredPitchDeg: null }, Make);
const ReachIn = (Frame, Servos) => { const Can = CanOf(Frame);
  return Api.TaskObjectReachability(Can, { Servos, ElbowMode: 'up' }, { Others: Api.NeighbourObjects(Frame, Can), Kind: 'can' }); };
const Live = AsLive(() => ReachIn(Frame0536, Snapshot(Raws.Home))), LivePlan = Live.Plan;
const BeforePlan = AsBefore(() => ReachIn(Frame0536, Snapshot(Raws.Home))).Plan;
const Now = As1001(() => ReachIn(Frame0536, Snapshot(Raws.Home))), NowPlan = Now.Plan;
const Key = Api.GraspPlanKey(Snapshot(Raws.Home), 'up');
const State = (Reach, Can, Extra = {}) => ({ Label: 'can', Intent: 'grasp', DetectedNow: true, Binding: { Object: Can, Source: 'query', DistanceCm: null },
  Candidates: [], LastSeen: { At: 0, Object: Can, Reachability: Reach }, Reachability: Reach,
  Plan: { Reach, Source: Can, At: new Date(2026, 9, 1, 15, 5, 36).getTime(), Key, Note: null, Confirmed: true, Corridor: null, ...Extra } });
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
const Fraction = Servos => Api.rawToFraction(Servos.gripper, Servos.gripper.presentRaw);
const PointText = P => `X ${P.x.toFixed(1)}, Y ${P.y.toFixed(1)}, Z ${P.z.toFixed(1)}`;

// --- 15:07, the first grasp -----------------------------------------------------------------------

Test('15:05 "Grab the can": the recorded plan and stops replay as logged', () => {
  // The live plan, as the grasp_plan record printed it.
  Assert.deepEqual([LivePlan.Pitch, LivePlan.Grasp, LivePlan.Hover, LivePlan.Side, LivePlan.HeightBy, LivePlan.RaiseCm],
    [-23.1, { x: 4.6, y: 28.6, z: 4.1 }, { x: 4.6, y: 28.6, z: 12.4 }, 'left-near', 'clear', 0.5]);
  // Each joint of the first descent stopped within the bridge's 12 steps of its target, the tip 0.47 cm under it.
  for (const Joint of Api.IK_JOINTS) Assert.ok(Math.abs(StopRaw[Joint]) <= Rules.ArrivalToleranceRaw, `${Joint} ${StopRaw[Joint]}`);
  const Tip = TipOf(Snapshot(Raws.Grasp1));
  Assert.ok(Math.abs(Tip.z - Descent1.z + 0.47) < 0.02 && Math.hypot(Tip.x - 4.42, Tip.y - 28.47) < 0.02, JSON.stringify(Tip));
  // Replayed with those errors, the live plan stops where the arm did.
  const Replayed = OnTheArm(LivePlan, CanOf(Frame0536), CentralOffsetCm).Tip;
  Assert.ok(Math.hypot(Replayed.x - Tip.x, Replayed.y - Tip.y, Replayed.z - Tip.z) < 0.15, JSON.stringify(Replayed));
  // This branch's first plan (the finger offset 1.5, the closing judged as before), which the review replayed.
  Assert.deepEqual([BeforePlan.Pitch, BeforePlan.Grasp, BeforePlan.RaiseCm], [-23.1, { x: 5.8, y: 27.6, z: 4.5 }, 0.5]);
});

Test('the push model cannot tell the 15:08 close from a hold: it calls the close the arm made held, so the replay judges the close by where it leaves the can, not by that verdict', () => {
  // At the close the can stood against the real fixed finger (the descent had pushed it there): the CAD finger 0.02 to 0.3 cm
  // from it at the recorded stop. The model holds it with friction for 11 to 18 deg, its contact 0.6 to 0.8 cm inside the
  // finger's tip; the arm squeezed it 3 cm out past the tips. Only with the real gripper 0.4 cm nearer the palm (unconfirmed)
  // is the contact within 0.4 cm of the tip, and at 0.7 cm on the tip's corner.
  const Can = CanOf(Frame0536), Stop = { ...Api.currentAngles(Snapshot(Raws.Grasp1)), gripper: OpenDeg };
  const Frame = Api.JawFrame(Stop), U = Level(Frame.U), A = Level(Frame.A);
  for (const GapCm of [0.02, 0.1, 0.3]) {
    const At = Along => {
      let Lo = 0, Hi = 2.5;
      for (let Step = 0; Step < 40; Step++) { const Mid = (Lo + Hi) / 2; if (CloseAt(Stop, RealCan(Can, U, Mid, Along, A)).Gap > GapCm) Lo = Mid; else Hi = Mid; }
      return CloseAt(Stop, RealCan(Can, U, Lo, Along, A));
    };
    const [Flush, Palm4, Palm7] = [0, 0.4, 0.7].map(At);
    Assert.ok(Flush.Close.Ok && Flush.Close.GripDeg <= Sweep.SweepRules.MaxLeanDeg && Flush.TipCm >= 0.6, `gap ${GapCm}: ${Flush.Close.Code} grip ${Flush.Close.GripDeg} tip ${Flush.TipCm}`);
    Assert.ok(Palm4.Close.Ok && Palm4.TipCm < 0.4, `gap ${GapCm}, 0.4 toward the tips: tip ${Palm4.TipCm}`);
    Assert.ok(Palm7.Close.Ok && Palm7.TipCm <= 0.1, `gap ${GapCm}, 0.7 toward the tips: tip ${Palm7.TipCm}`);
  }
});

Test('the live plan put the real fixed finger into the can on the way down; the plan now keeps every part clear of the real can, with the finger 0.4 cm nearer still too', () => {
  const Can = CanOf(Frame0536);
  for (const OffsetCm of RealOffsetsCm) {
    // Live: the fixed finger came down into the can (the descent pushed it 0.6 cm), so the close started from a pushed can.
    Assert.ok(OnTheArm(LivePlan, Can, OffsetCm).Least < 0.1, `live, offset ${OffsetCm}`);
    // Now: every part clear of the real can on the way down, the real pad within the closing range of it.
    const Arm = OnTheArm(NowPlan, Can, OffsetCm);
    Assert.ok(Arm.Least >= 0.4, `offset ${OffsetCm}: nearest ${Arm.Least.toFixed(2)} cm on the way down`);
    Assert.ok(Arm.Gap >= 0.3 && Arm.Gap <= 1.6, `offset ${OffsetCm}: the real pad ${Arm.Gap.toFixed(2)} cm from the real can`);
  }
  // At the 2.1 cm the depth edge reached (the plan's offset plus FixedFingerToleranceCm): still clear by the guard's margin.
  Assert.ok(OnTheArm(NowPlan, Can, UpperOffsetCm).Least >= Rules.ContactMarginCm, 'the upper offset');
  // The plan: c outside the can where the real gripper meets it, the finger offset farther from where the camera measured it.
  Assert.deepEqual([NowPlan.FingerOffsetCm, NowPlan.SagCm, NowPlan.Closing.Mode], [1.7, Rules.GraspSagCm, 'sweep']);
  const U = NowPlan.JawAxis, Along = Point => Point.x * U.x + Point.y * U.y;
  Assert.ok(Math.abs(Along(LivePlan.Grasp) - Along(NowPlan.Grasp) - 1.7) <= 0.3, `${Along(LivePlan.Grasp) - Along(NowPlan.Grasp)}`);
});

Test('the close: the plan leaves the can deeper in the jaws than the 15:05 plans did, at the arm\'s recorded stop', () => {
  const Can = CanOf(Frame0536);
  // The plan's own check (where the real gripper meets it, at its printed point): at or behind the fixed pad's middle, the grip
  // within the friction limit in 3D too. The 15:05 plans: 0.23 and 0.30 cm inside the tip, 21 deg in 3D.
  Assert.ok(NowPlan.Closing.TipCm >= Rules.FingertipMarginCm && NowPlan.Closing.Grip3DDeg <= Sweep.SweepRules.MaxLeanDeg, JSON.stringify(NowPlan.Closing));
  for (const Then of [LivePlan, BeforePlan]) Assert.ok(Then.Closing.TipCm < 0.35 && Then.Closing.Grip3DDeg > Sweep.SweepRules.MaxLeanDeg, JSON.stringify(Then.Closing));
  // Its head start along the fingers is the push's drift (the 15:05 plans: a quarter of it, the palm in the way at that height).
  Assert.ok(NowPlan.Closing.DepthOffsetCm >= NowPlan.Closing.DriftCm - 0.1, JSON.stringify(NowPlan.Closing));
  Assert.ok(BeforePlan.Closing.DepthOffsetCm <= BeforePlan.Closing.DriftCm / 2, JSON.stringify(BeforePlan.Closing));
  // On the arm, at the recorded descent's stop errors: at the central offset the contact ends 0.7 cm inside the tip (this
  // branch's first plan: 0.27), and with the real gripper 0.4 cm nearer the palm 0.4 cm inside (its first plan: on the tip's
  // corner); 0.25 cm nearer or farther, 0.5 and 0.9 (0.1 and 0.4).
  const Inside = (Plan, OffsetCm, Along = 0) => OnTheArm(Plan, Can, OffsetCm, Along).TipCm;
  Assert.ok(Inside(NowPlan, CentralOffsetCm) >= 0.7 && Inside(BeforePlan, CentralOffsetCm) < 0.3, `${Inside(NowPlan, CentralOffsetCm)} / ${Inside(BeforePlan, CentralOffsetCm)}`);
  Assert.ok(Inside(NowPlan, CentralOffsetCm, 0.4) >= 0.35 && Inside(BeforePlan, CentralOffsetCm, 0.4) <= 0.1, `${Inside(NowPlan, CentralOffsetCm, 0.4)} / ${Inside(BeforePlan, CentralOffsetCm, 0.4)}`);
  for (const OffsetCm of RealOffsetsCm) {
    const Arm = OnTheArm(NowPlan, Can, OffsetCm);
    Assert.ok(Arm.TipCm >= 0.5 && Arm.TipCm > OnTheArm(BeforePlan, Can, OffsetCm).TipCm + 0.3, `offset ${OffsetCm}: ${Arm.TipCm}`);
    Assert.ok(Arm.Close.Ok && Arm.Grip3DDeg <= Sweep.SweepRules.MaxLeanDeg, `offset ${OffsetCm}: ${Arm.Close.Code} ${Arm.Grip3DDeg}`);
  }
});

Test('the grasp point is printed the arm\'s sag above where the jaws close: the arm stops at the planned height', () => {
  const Can = CanOf(Frame0536), Mid = (Can.z + Can.topZ) / 2;
  // Live: planned 0.5 cm over mid-height (at mid-height the grip needs more friction than the pads have); stopped 0.47 cm under it.
  Assert.ok(Math.abs(TipOf(Snapshot(Raws.Grasp1)).z - Mid) < 0.2, 'the live descent stopped at mid-height');
  // Now: raised to where the push leaves it at or behind the pad's middle, printed the sag higher; the descent with the recorded
  // errors stops within 0.1 cm of where the plan closes.
  Assert.equal(NowPlan.HeightBy, 'clear');
  Assert.match(NowPlan.RaisedFor, /the closing push would leave its contact with the fixed finger only 0\.\d cm inside the finger's tip/);
  const Closes = NowPlan.Grasp.z - NowPlan.SagCm;
  Assert.ok(Closes >= Mid + NowPlan.RaiseCm - 0.05 && Closes <= Mid + NowPlan.RaiseCm + 0.15, `${Closes}`);
  const Stop = OnTheArm(NowPlan, Can, CentralOffsetCm).Tip;
  Assert.ok(Math.abs(Stop.z - Closes) <= 0.1, `stopped at ${Stop.z.toFixed(2)}, closes at ${Closes.toFixed(2)}`);
  // Stopped there (or not sagging at all, at the printed point), the next step is the close (chained with the lift,
  // AiChain.ts); the close is offered within CloseBelowCm of where the jaws close, not of the printed point.
  const Close = { move_delta_cm: { left: 0, farther: 0, up: 6 }, gripper_before: 0 };
  const StepAt = Tip => MotionIn(Api.TaskObjectLine(State(Now, Can), Tip, { GripperFraction: 0.8, Closed: false }));
  for (const Tip of [Stop, NowPlan.Grasp, { ...NowPlan.Grasp, z: Closes - Rules.CloseBelowCm + 0.02 }]) Assert.deepEqual(StepAt(Tip), Close, JSON.stringify(Tip));
  const Low = { ...NowPlan.Grasp, z: Closes - Rules.CloseBelowCm - 0.1 };
  Assert.deepEqual(StepAt(Low), { move_delta_cm: { left: 0, farther: 0, up: Number((NowPlan.Grasp.z - Low.z).toFixed(1)) } }, 'lower than that, not holding the grasp point: rise to it');
  // The model is told the corrections, how deep the push leaves it, and why.
  const Line = Api.TaskObjectLine(State(Now, Can), TipOf(Snapshot(Raws.Home)), { GripperFraction: 0.8, Closed: false });
  Assert.match(Line, /The GRASP POINT is 2\.7 cm outside its left-near side as the camera measured it: the real fixed finger stands about 1\.7 cm nearer the can than the arm model puts it \(it pushed a can it was planned 1\.2 cm from\), so the real finger comes down 1\.0 cm from it \(the app checked the way down with it 0\.4 cm nearer still\); do not move the tip toward the can to close that gap\./);
  Assert.match(Line, /The GRASP POINT's Z is 0\.5 cm above where the jaws close, because the arm stops about that much lower than a low target\./);
  Assert.match(Line, /it ends on the middle of the fixed pad\. That leaves its contact with the fixed finger 0\.\d cm inside the finger's tip \(nearer the tip, the close can squeeze it out past the tips\)\./);
});

Test('the guard lets the plan\'s own descent through and holds the can where the real fixed finger meets it, also 1 cm farther out', () => {
  const Can = CanOf(Frame0536), Task = State(Now, Can);
  const Guard = Api.BuildGraspGuard(Frame0536, Task, false);
  Assert.ok(Guard.Prisms.some(P => P.Role === 'twin' && /where the real fixed finger meets it/.test(P.Label)), 'the arm-frame can');
  // At the plan's hover point with the jaws open to the plan's 80% (raw 1917, as the run's).
  const AtHover = Snapshot({ ...Raws.Aimed, ...RawsAt(Solve(NowPlan.Hover, NowPlan.HoverPitch, Snapshot(Raws.Home)).angles, Snapshot(Raws.Home)) });
  const Check = (Point, Pitch, With = Guard) => { const Target = Model(Point, Pitch);
    return Api.GraspGuardReason(Target, Api.solveIkPreview(Target, 'up', AtHover).angles, Api.currentAngles(AtHover), Point.z - TipOf(AtHover).z, With); };
  Assert.equal(Check(NowPlan.Grasp, NowPlan.Pitch), null, 'the plan\'s descent');
  // The live plan's grasp point (1.7 cm nearer the can): the real fixed finger would go into the can there.
  Assert.match(Check({ ...LivePlan.Grasp, z: NowPlan.Grasp.z }, NowPlan.Pitch) ?? '', /FIXED finger .* the can/);
  // The tape check moving the plan 1 cm away (over 3 cm from where the camera measured the can): below its top there the wrist is
  // still held to the plan's (the guard's 3 cm zone counts from where the real gripper meets it too).
  const Moved = Api.TapeCorrection(NowPlan, { x: NowPlan.JawAxis.x, y: NowPlan.JawAxis.y, z: 0 }, { Servos: Snapshot(Raws.Home), ElbowMode: 'up', Others: [] }).Plan;
  Assert.ok(Moved && Api.FootprintDistance(NowPlan.Prism.Footprint, Moved.Grasp) > 3, JSON.stringify(Moved?.Grasp));
  const MovedGuard = Api.BuildGraspGuard(Frame0536, State({ ...Now, Plan: Moved }, Can), false);
  Assert.equal(Check(Moved.Grasp, Moved.Pitch, MovedGuard), null, 'the moved plan\'s descent');
  Assert.match(Check(Moved.Grasp, -5, MovedGuard) ?? '', /GRASP PLAN was checked only at wrist -?[\d.]+ deg there/);
});

// --- 15:08-15:10, the miss and the second grasp ---------------------------------------------------

Test('after the miss the plan is made again from where the close pushed the can, not repeated where it was', () => {
  const Before = CanOf(Frame0536), After = CanOf(Frame0815);
  const Closed1 = Snapshot(Raws.Closed1), Tip = TipOf(Closed1), Grasp = { GripperFraction: Fraction(Closed1), Closed: true };
  const Previous = State(Live, Before).Plan;
  // The jaws closed on nothing at the plan's grasp point: the plan they missed with is stale. Anywhere the steps offer the
  // close counts (1.5 cm away along the jaws, 1 cm across, 1 cm high); risen 3 cm, or a narrow object's full close, does not.
  Assert.equal(Api.ClosedOnNothing(Grasp, LivePlan, Tip), true);
  const U = LivePlan.JawAxis, Edge = { x: LivePlan.Grasp.x - 1.5 * U.x - 1.0 * U.y, y: LivePlan.Grasp.y - 1.5 * U.y + 1.0 * U.x, z: LivePlan.Grasp.z + 1.0 };
  Assert.equal(Api.ClosedOnNothing(Grasp, LivePlan, Edge), true, 'the edge of where the close is offered');
  Assert.equal(Api.ClosedOnNothing(Grasp, LivePlan, { ...LivePlan.Grasp, z: LivePlan.Grasp.z + 3 }), false, 'risen from it');
  Assert.equal(Api.ClosedOnNothing(Grasp, { ...LivePlan, WidthAcrossCm: 1.0 }, Tip), false, 'a narrow object');
  Assert.equal(Api.ClosedOnNothing({ ...Grasp, GripperFraction: 0.2 }, LivePlan, Tip), false, 'the jaws stopped on something');
  // The plan lock as it was (no miss): kept, and the second descent went where the can had been, closing on air in front of it.
  Assert.equal(Api.KeepOrReplan(Previous, After, Tip, true, Key).Keep, true);
  const Stale = OnTheArm(LivePlan, CanOf(Frame1006), CentralOffsetCm);
  Assert.equal(Stale.Close.Ok, false, `the stale plan: ${Stale.Close.Code}`);
  // Now: planned again from the re-measure, and the model is told why.
  const Verdict = Api.KeepOrReplan(Previous, After, Tip, true, Key, true);
  Assert.deepEqual([Verdict.Keep, Verdict.AfterMiss], [false, true]);
  Assert.match(Verdict.Note, /^the jaws closed on nothing at the GRASP PLAN's grasp point \(the close may have pushed it\); re-measured at X 2\.4, Y 32\.7, 2\.9 cm from the 15:05:36 measurement the GRASP PLAN was made from: new GRASP PLAN from where it stands now$/);
  const Again = As1001(() => ReachIn(Frame0815, Closed1)), Plan = Again.Plan;
  Assert.equal(Again.Kind, 'graspable', Again.Text);
  Assert.ok(Math.hypot(Plan.Grasp.x - LivePlan.Grasp.x, Plan.Grasp.y - LivePlan.Grasp.y) >= 2.5, JSON.stringify(Plan.Grasp));
  Assert.ok(Plan.Closing.TipCm >= Rules.FingertipMarginCm && Plan.Closing.Grip3DDeg <= Sweep.SweepRules.MaxLeanDeg, JSON.stringify(Plan.Closing));
  // On the arm, against the can where it stood from then on (15:10:06, 0.4 cm from the 15:08:15 reading the plan is made from):
  // clear on the way down, at 2.1 cm too, the real pad within the closing range, the contact inside the finger's tip.
  const Stood = CanOf(Frame1006);
  for (const OffsetCm of [...RealOffsetsCm, UpperOffsetCm]) {
    const Arm = OnTheArm(Plan, Stood, OffsetCm);
    Assert.ok(Arm.Least >= 0.4 && Arm.Gap >= 0.3 && Arm.Gap <= 1.6 && Arm.TipCm >= 0.1, `offset ${OffsetCm}: nearest ${Arm.Least.toFixed(2)}, gap ${Arm.Gap.toFixed(2)}, tip ${Arm.TipCm}`);
  }
  // The step: rise with the jaws closed to the NEW plan's hover height, then open; the next hover is the new plan's.
  const Task = State(Again, After, { Note: Verdict.Note, Confirmed: false, AfterMiss: Verdict.AfterMiss });
  const Line = Api.TaskObjectLine(Task, Tip, Grasp);
  Assert.deepEqual(MotionIn(Line), { move_delta_cm: { left: 0, farther: 0, up: Number((Plan.Hover.z - Tip.z).toFixed(1)) } });
  Assert.match(Line, /the jaws closed all the way, so they missed it\. Rise straight up .* The GRASP PLAN above is made from where the camera measured the can after the close \(the close may have pushed it\), not where it was before\./);
  Assert.match(Line, /\(the jaws closed on nothing at the GRASP PLAN's grasp point/);
  const Risen = Snapshot(Raws.Risen);
  // Risen with the jaws still closed, the can partly hidden: the new plan stays (no miss at its grasp point), the step opens.
  Assert.equal(Api.ClosedOnNothing({ GripperFraction: Fraction(Risen), Closed: true }, Plan, TipOf(Risen)), false);
  Assert.equal(Api.KeepOrReplan(Task.Plan, CanOf(Frame0927), TipOf(Risen), true, Key, false).Keep, true);
  Assert.deepEqual(MotionIn(Api.TaskObjectLine(Task, TipOf(Risen), { GripperFraction: Fraction(Risen), Closed: true })),
    { direct_servo_targets: [{ name: 'gripper', target_fraction: Plan.OpenFraction }] });
  const Hover2 = Snapshot(Raws.Hover2);
  Assert.deepEqual(MotionIn(Api.TaskObjectLine(Task, TipOf(Hover2), { GripperFraction: Fraction(Hover2), Closed: false })),
    { target_cm: Plan.Hover, wrist_pitch_degrees: Plan.HoverPitch });
});

Test('a miss whose re-measure is partly hidden keeps the plan marked missed: no step descends with it, the guard refuses its stale descent, and the model is not given its points', () => {
  const Before = CanOf(Frame0536), Hidden = { ...CanOf(Frame0927) };
  const Closed1 = Snapshot(Raws.Closed1), Tip = TipOf(Closed1);
  const Previous = State(Live, Before).Plan;
  const Kept = Api.KeepOrReplan(Previous, Hidden, Tip, true, Key, true);
  Assert.deepEqual([Kept.Keep, Kept.Missed, Kept.Corridor], [true, true, null]);
  Assert.match(Kept.Note, /but only partly \(size 4\.4 x 3\.5, was 5\.4 x 5\.3\): the app plans the grasp again once the camera measures it clear of the gripper$/);
  for (const [Name, Reach, Plan] of [['live', Live, LivePlan], ['now', Now, NowPlan]]) {
    const Task = { ...State(Reach, Before, { Missed: true, Note: Kept.Note, Confirmed: false }), Binding: { Object: Hidden, Source: 'query', DistanceCm: null },
      LastSeen: { At: 0, Object: Hidden, Reachability: Reach } };
    const Step = (Servos, Closed) => Api.TaskObjectLine(Task, TipOf(Servos), { GripperFraction: Fraction(Servos), Closed });
    // Closed beside it: rise, and the model is told no plan is made for where it stands yet.
    const Low = Step(Closed1, true);
    Assert.deepEqual(MotionIn(Low), { move_delta_cm: { left: 0, farther: 0, up: Number((Plan.Hover.z - Tip.z).toFixed(1)) } }, Name);
    Assert.match(Low, /The camera has not measured the can clear of the gripper since, so the app has no GRASP PLAN for where it stands now: it plans the grasp again once the camera sees all of it\. Do not descend before that\./);
    // Opened low beside it: rise; opened at the hover: ask. Never the old descent or a close.
    const OpenLow = Snapshot({ ...Raws.Grasp1 });
    Assert.deepEqual(MotionIn(Step(OpenLow, false)), { move_delta_cm: { left: 0, farther: 0, up: Number((Plan.Hover.z - TipOf(OpenLow).z).toFixed(1)) } }, Name);
    const AtHover = Step(Snapshot(Raws.Hover2), false);
    Assert.match(AtHover, /NEXT STEP of the grasp \(tip now X 4\.4, Y 28\.4, Z 11\.6; gripper 80% open\): reply ask: the jaws missed the can and the camera has not measured it clear of the gripper since/);
    Assert.doesNotMatch(AtHover, /"up":-/);
    // The stale plan's points and its description are not given: RIGHT ARM says it is stale, no GRASP POINT anywhere.
    for (const Line of [Low, AtHover]) {
      Assert.match(Line, /RIGHT ARM: the GRASP PLAN the jaws missed the can with is stale, so the app gives no GRASP POINT or HOVER POINT for it until the camera measures the can clear of the gripper/);
      Assert.doesNotMatch(Line, new RegExp(`GRASP POINT X|${PointText(Plan.Grasp)}|"x":${Plan.Grasp.x.toFixed(1)}|GRASP PLAN \\(checked by the app`), Name);
    }
    // The guard holds the can where it was measured now, without the stale plan. The review's case: re-measured partly
    // hidden 1.2 cm toward the fixed finger along the jaws, into the stale descent's way (the real finger would go into
    // it): the stale descent and a descent onto the can are refused, and the refusal quotes no stale point. Before, the
    // guard checked the stale plan and let its descent through, and its refusals offered the stale points.
    const U = Plan.JawAxis, Pushed = { ...Before, x: Before.x - 1.2 * U.x, y: Before.y - 1.2 * U.y, sizeCm: [4.0, 3.5], validFraction: 0.5 };
    const Measured = [Pushed, ...Frame0536.filter(Object_ => Object_ !== Before)];
    const PushedTask = { ...Task, Binding: { Object: Pushed, Source: 'query', DistanceCm: null }, LastSeen: { At: 0, Object: Pushed, Reachability: Reach } };
    const Guard = Api.BuildGraspGuard(Measured, PushedTask, false);
    Assert.equal(Guard.Plan, null);
    const Above = Snapshot({ ...Raws.Hover2, ...RawsAt(Solve(Plan.Hover, Plan.HoverPitch, Snapshot(Raws.Home)).angles, Snapshot(Raws.Home)) });
    const Down = (Point, With = Guard) => { const Target = Model(Point, Plan.Pitch);
      return Api.GraspGuardReason(Target, Api.solveIkPreview(Target, 'up', Above).angles, { ...Api.currentAngles(Above), gripper: OpenDeg }, Point.z - TipOf(Above).z, With) ?? ''; };
    for (const Point of [Plan.Grasp, { x: Pushed.x, y: Pushed.y, z: Plan.Grasp.z }]) {
      const Why = Down(Point);
      Assert.match(Why, /descends beside the can, which has no GRASP PLAN|would (go|come)/, `${Name}: ${JSON.stringify(Point)}`);
      Assert.doesNotMatch(Why, /use the GRASP PLAN|hover point X/, Name);
    }
    Assert.doesNotMatch(Api.TaskObjectLine(PushedTask, TipOf(Above), { GripperFraction: Plan.OpenFraction, Closed: false }), /GRASP POINT X/, Name);
    // The stale plan's guard (as before) let that descent through.
    Assert.equal(Down(Plan.Grasp, Api.BuildGraspGuard(Measured, { ...PushedTask, Plan: { ...PushedTask.Plan, Missed: false } }, false)), '', `${Name}: as before`);
  }
  // The next clean measurement (15:10:06) plans again, the gripper near it or not.
  const Clean = Api.KeepOrReplan({ ...Previous, Missed: true }, CanOf(Frame1006), TipOf(Snapshot(Raws.Hover2)), false, Key, false);
  Assert.equal(Clean.Keep, false);
  Assert.match(Clean.Note, /new GRASP PLAN from where it stands now$/);
});

Test('a miss whose clean re-measure plans no grasp keeps the missed plan: the closed jaws rise clear of the can first, then the model asks', () => {
  // The close pushed the can out of the arm's reach (moved 20 cm farther than the 15:08:15 reading): no new plan. Without the
  // missed plan the step would be the refusal's "reply ask" with the closed jaws low beside where the can was.
  const Before = CanOf(Frame0536), After = CanOf(Frame0815), Gone = { ...After, y: After.y + 20 };
  const Closed1 = Snapshot(Raws.Closed1), Tip = TipOf(Closed1), Grasp = { GripperFraction: Fraction(Closed1), Closed: true };
  const Previous = State(Live, Before).Plan;
  Assert.equal(Api.KeepOrReplan(Previous, Gone, Tip, true, Key, true).Keep, false);
  const Reach = As1001(() => ReachIn([Gone, ...Frame0815.slice(2)], Closed1));
  Assert.ok(!Reach.Plan && ['out-of-reach', 'ungraspable'].includes(Reach.Kind), Reach.Text);
  const Kept = Api.KeepMissed(Previous, Gone, Reach);
  Assert.deepEqual([Kept.Missed, Kept.Confirmed, Kept.Corridor, Kept.Reach], [true, false, null, Live]);
  Assert.match(Kept.Note, /^the jaws closed on nothing at the GRASP PLAN's grasp point; re-measured at X 2\.4, Y 52\.7, where the app plans no grasp \(.+\): the jaws rise clear of it first$/);
  const Task = { ...State(Live, Gone), Plan: Kept };
  const Line = Api.TaskObjectLine(Task, Tip, Grasp);
  Assert.deepEqual(MotionIn(Line), { move_delta_cm: { left: 0, farther: 0, up: Number((LivePlan.Hover.z - Tip.z).toFixed(1)) } });
  Assert.doesNotMatch(Line, /GRASP POINT X/);
  // Risen: no step descends with it; at the hover height the model asks.
  Assert.match(Api.TaskObjectLine(Task, { ...Tip, z: LivePlan.Hover.z }, { GripperFraction: Fraction(Closed1), Closed: true }), /the jaws closed all the way, so they missed it\. Open the gripper/);
  Assert.match(Api.TaskObjectLine(Task, { ...Tip, z: LivePlan.Hover.z }, { GripperFraction: 0.8, Closed: false }), /reply ask: the jaws missed the can/);
});

Test('where no wrist pitch or height keeps the can inside the fixed finger\'s tip, the app plans as before and tells the model what that plan lacks', () => {
  const Can = CanOf(Frame0536);
  const Short = With({ FingertipMarginCm: 1.5, FixedFingerOffsetCm: 1.7, StraddleRoundObjects: false, PreferredPitchDeg: null }, () => ReachIn(Frame0536, Snapshot(Raws.Home)));
  const Plan = Short.Plan;
  Assert.equal(Short.Kind, 'graspable', Short.Text);
  Assert.deepEqual([Plan.Pitch, Plan.RaiseCm], [BeforePlan.Pitch, BeforePlan.RaiseCm], 'planned as before');
  Assert.ok(Plan.Closing.TipCm < 0.4 && Plan.Closing.Grip3DDeg > Sweep.SweepRules.MaxLeanDeg, JSON.stringify(Plan.Closing));
  const Line = With({ FingertipMarginCm: 1.5 }, () => Api.TaskObjectLine(State(Short, Can), TipOf(Snapshot(Raws.Home)), { GripperFraction: 0.8, Closed: false }));
  Assert.match(Line, /No wrist pitch or height the app tried holds it better, but here its contact with the fixed finger ends only 0\.\d cm inside the finger's tip \(the app wants 1\.5\) and the grip needs friction for 2\d deg with the jaws tilted against its standing side \(at most 20\): the close may squeeze it out past the finger tips; if the image after the close shows it slipping out, reply ask\./);
});

// --- 18:53-18:56, the descent from where the arm stopped off the hover point --------------------------

// The servo raws at each check-in (the model requests' "Current servo positions"): 18:53:55 at Home; 18:54:53 holding
// the hover point, stopped 0.5 cm toward the can; 18:55:06 at the bottom after the descent from there; 18:55:49 at the
// re-aimed hover point; 18:56:33 at the bottom after the aligned descent.
const Raws1853 = { Home: Raw(3479, 3172, 2688, 856, 694), Hover: Raw(3080, 464, 1619, 676, 1919), Low: Raw(3080, 581, 1740, 432, 1919),
  Aimed: Raw(3062, 465, 1618, 667, 1919), Aligned: Raw(3069, 585, 1731, 438, 1919) };
const Near1853 = (Extra = {}) => ({ taskMatch: false, refined: false, ...Extra });
// 18:53:54 (the plan's frame): the can measured as "coffee cup", picked by the task query.
const Frame1853 = [Obj('coffee cup', 2.35, 33.37, -2.1, 9.69, [5.5, 5.2], -1.84, 0.831, { taskMatch: true, refined: false }),
  Obj('small bottle', 11.57, 31.45, -2.31, 5.5, [3.5, 2.8], -2.11, 0.872, Near1853({ outsideCalibrationCm: 1.3 })),
  Obj('tape roll', 6.87, 26.68, -2.5, 1.27, [6.4, 3.5], -2.24, 0.974, Near1853()),
  Obj('paper cup', -12.59, 31.63, -2.28, -0.18, [8.6, 8.6], -2.25, 0.575, Near1853({ outsideCalibrationCm: 1.3 })),
  Obj('bottle', 11.57, 31.45, -2.31, 5.5, [3.5, 2.8], -2.09, 0.884, Near1853({ outsideCalibrationCm: 1.3 })),
  Obj('tape', 6.94, 26.74, -2.62, 1.26, [7.6, 3.6], -2.28, 0.976, Near1853()),
  Obj('small jar', 11.57, 31.45, -2.31, 5.5, [3.5, 2.8], -2.09, 0.862, Near1853({ outsideCalibrationCm: 1.3 })),
  Obj('phone', 25.35, 62.64, 0.07, 1.36, [12.9, 7.5], 0.08, 1, Near1853({ extrapolated: true, outsideCalibrationCm: 34.6 }))];
// 18:54:51, the gripper over the can: the can not measured; its neighbours were.
const Frame1854 = [Obj('bottle', 11.64, 31.42, -2.24, 5.58, [3.5, 2.9], -2.04, 0.868, Near1853({ outsideCalibrationCm: 1.3 })),
  Obj('battery', 6.64, 28.37, -2.1, 2.77, [4.7, 2.8], -2, 0.865, Near1853()),
  Obj('white bottle', 11.64, 31.39, -2.25, 5.56, [3.5, 2.8], -2.02, 0.872, Near1853({ outsideCalibrationCm: 1.3 })),
  Obj('black battery pack', 6.63, 28.36, -2.11, 2.79, [5, 3.6], -2.05, 0.888, Near1853()),
  Obj('phone', 24.96, 62.54, 0.17, 1.46, [13.5, 9.7], 0.2, 1, Near1853({ extrapolated: true, outsideCalibrationCm: 34.3 }))];
const Can1853 = Frame1853[0];
const Reach1853 = As1001(() => Api.TaskObjectReachability(Can1853, { Servos: Snapshot(Raws1853.Home), ElbowMode: 'up' },
  { Others: Api.NeighbourObjects(Frame1853, Can1853), Kind: 'can' }));
const Plan1853 = Reach1853.Plan;
const At1853 = new Date(2026, 9, 1, 18, 53, 55).getTime();
// Under the gripper from 18:54:51 on: the plan goes on from the 18:53:55 measurement (kept, measured once).
const Task1853 = { Label: 'can', Intent: 'grasp', DetectedNow: false, Binding: null, Candidates: [], Reachability: null,
  LastSeen: { At: At1853, Object: Can1853, Reachability: Reach1853 },
  Plan: { Reach: Reach1853, Source: Can1853, At: At1853, Key: Api.GraspPlanKey(Snapshot(Raws1853.Home), 'up'), Note: null, Confirmed: false, Corridor: null } };
// The tip and wrist the app reads a reply from (model frame), and the grasp as the steps see it (the target the bridge holds).
const Anchor = Servos => { const Angles = Api.currentAngles(Servos);
  return { ...Api.forwardKinematics(Angles).at(-1), wristPitch: Api.wristPitchFromAngles(Angles) }; };
const Holding = (Servos, Point, Pitch) => ({ GripperFraction: Fraction(Servos), Closed: false, Held: { Point, Pitch } });
// The check-in at a pose: its TASK OBJECT line, the line's NEXT STEP read from the reply's anchor, and the guard with it.
function CheckIn1853(Servos, Held, Frame = Frame1854) {
  const Tip = TipOf(Servos), Grasp = Holding(Servos, Held.Point, Held.Pitch);
  const Line = Api.TaskObjectLine(Task1853, Tip, Grasp);
  const Step = Api.PlannedStepOf(Line, Api.mirrorDisplayPoint(Anchor(Servos)), Anchor(Servos).wristPitch);
  return { Line, Step, Grasp, Guard: Api.BuildGraspGuard(Frame, Task1853, false, Step) };
}
// A reply as the app turns it into a proposal and checks it: parsed from the live tip, its guard attached, snapped, validated.
function Propose1853(Servos, Motion, Guard, Grasp) {
  const Parsed = Api.ParseResponse(JSON.stringify({ response_to_user: 'x', task_status: 'continue', motion_proposal: Motion }), Anchor(Servos), Servos).motion;
  const Guarded = { ...Parsed, guard: Guard };
  const Snapped = Api.ApplySnap(Guarded, Api.SnapAiTarget(Api.SnapInput(Guarded, Servos, 'up', undefined, [])));
  return { Motion: Snapped, Check: Api.Validate(Snapped, 'up', Servos, [], undefined, { Object: Can1853, State: Grasp, Plan: Plan1853 }, Guard) };
}
const AimJson = '{"target_cm":{"x":8.7,"y":30.5,"z":12.7},"wrist_pitch_degrees":-18.1}';

Test('18:53 "grab the can": the recorded plan, stops and NEXT STEP replay as logged', () => {
  Assert.equal(Reach1853.Kind, 'graspable', Reach1853.Text);
  Assert.deepEqual([Plan1853.Pitch, Plan1853.Grasp, Plan1853.Hover, Plan1853.Side, Plan1853.HeightBy, Plan1853.RaiseCm, Plan1853.OpenFraction],
    [-18.1, { x: 8.1, y: 30.9, z: 6.8 }, { x: 8.1, y: 30.9, z: 12.7 }, 'left-near', 'clear', 2.5, 0.8]);
  // The logged tips: holding the hover point 0.5 cm toward the can, then at the bottom of the descent from there.
  const Tip = TipOf(Snapshot(Raws1853.Hover));
  Assert.ok(Math.hypot(Tip.x - 7.6244, Tip.y - 31.0676, Tip.z - 11.9023) < 0.001, JSON.stringify(Tip));
  const Low = TipOf(Snapshot(Raws1853.Low));
  Assert.ok(Math.hypot(Low.x - 7.6, Low.y - 31.0, Low.z - 6.2) < 0.1, JSON.stringify(Low));
  // 18:54:53: the step re-aims the hover point past where the arm stopped, and says not to descend from there.
  const { Line, Step } = CheckIn1853(Snapshot(Raws1853.Hover), { Point: Plan1853.Hover, Pitch: Plan1853.HoverPitch });
  Assert.deepEqual(MotionIn(Line), JSON.parse(AimJson));
  Assert.match(Line, /NEXT STEP of the grasp \(tip now X 7\.6, Y 31\.1, Z 11\.9; gripper 80% open\): the arm holds the HOVER POINT but its tip stopped 0\.5 cm toward the can and 0\.1 cm across from where the GRASP PLAN puts the fixed jaw .* Do not descend from here: go to the hover point aimed 0\.7 cm past it, away from the can, so the arm stops at it: "motion_proposal":\{"target_cm":\{"x":8\.7,"y":30\.5,"z":12\.7\},"wrist_pitch_degrees":-18\.1\}\. Copy it exactly \(one step per motion, never skipping ahead\)/);
  Assert.deepEqual([Step.Json, Step.Target, Step.Pitch], [AimJson, { x: 8.7, y: 30.5, z: 12.7 }, -18.1]);
  // The prompt's last line repeats it.
  Assert.match(Api.NextStepReminder(Step, 'can'), /^NEXT STEP TO COPY \(from the TASK OBJECT line; the app refuses any other move near the can\): "motion_proposal":\{"target_cm":\{"x":8\.7,"y":30\.5,"z":12\.7\},"wrist_pitch_degrees":-18\.1\}\./);
});

Test('18:54:53: the descent from the hover point the arm stopped off is refused, its approved revision too, each with the NEXT STEP\'s JSON verbatim; the step itself passes', () => {
  const Hover = Snapshot(Raws1853.Hover);
  const { Step, Grasp, Guard } = CheckIn1853(Hover, { Point: Plan1853.Hover, Pitch: Plan1853.HoverPitch });
  const NoStep = { ...Guard, Step: null };
  // The reply (up 5.9, 0.8 cm deeper than the plan) and its revision (up 5.1, to the grasp height 0.5 cm toward the can).
  for (const Up of [-5.9, -5.1]) {
    const Reply = { move_delta_cm: { left: 0, farther: 0, up: Up }, wrist_pitch_degrees: -18.1 };
    const Now = Propose1853(Hover, Reply, Guard, Grasp);
    Assert.equal(Now.Check.possible, false, `up ${Up}`);
    Assert.equal(Now.Motion.snap.Kind, 'refused', 'refused by the snap, never moved');
    Assert.equal(Now.Check.issues.length, 1, Now.Check.issues.join(' | '));
    const [Why] = Now.Check.issues;
    Assert.ok(Why.startsWith(Api.NextStepRefusalLead), Why);
    Assert.match(Why, /^That move is not the NEXT STEP, and it ends low beside the can \(within 3 cm of it, below its top \+ 1 cm\), where the app accepts only the NEXT STEP or a move up and away from the can\. Copy the NEXT STEP's motion_proposal exactly: "motion_proposal":\{"target_cm":\{"x":8\.7,"y":30\.5,"z":12\.7\},"wrist_pitch_degrees":-18\.1\}\. The step, with why: NEXT STEP of the grasp \(tip now X 7\.6/);
    Assert.ok(Why.includes(`"motion_proposal":${Step.Json}`) && Why.includes(Step.Text), 'the step and its JSON, verbatim');
    Assert.match(Why, /If you think the NEXT STEP is wrong, reply task_status "ask" with motion_proposal null and say why\.$/);
    // Without the step (as live): the revision passed, and was approved.
    const Before = Propose1853(Hover, Reply, NoStep, Grasp);
    if (Up === -5.1) Assert.equal(Before.Check.possible, true, `as live: ${Before.Check.issues.join(' ')}`);
    else Assert.match(Before.Check.issues.join(' '), /0\.8 cm deeper than the GRASP PLAN's grasp point Z 6\.8 beside the can\. .*use the GRASP PLAN: hover point X 8\.1, Y 30\.9, Z 12\.7/);
  }
  // The step, copied exactly (and as the app would offer it), passes; the hover point the arm already holds too (it is no
  // descent: the no-op rule handles a repeat), and a move up and away.
  for (const [Name, Reply] of [['the step', JSON.parse(AimJson)], ['the step without its wrist', { target_cm: { x: 8.7, y: 30.5, z: 12.7 } }],
    ['the plain hover point', { target_cm: Plan1853.Hover, wrist_pitch_degrees: -18.1 }], ['up and away', { move_delta_cm: { left: 1, farther: -1, up: 2 } }]]) {
    const Now = Propose1853(Hover, Reply, Guard, Grasp);
    Assert.equal(Now.Check.possible, true, `${Name}: ${Now.Check.issues.join(' ')}`);
  }
  const Offered = Api.ParseResponse(`{"response_to_user":"","task_status":"continue","motion_proposal":${Step.Json}}`, Anchor(Hover), Hover);
  Assert.equal(Offered.taskStatus, 'continue');
  Assert.equal(Api.Validate({ ...Offered.motion, guard: Guard }, 'up', Hover, [], undefined, { Object: Can1853, State: Grasp, Plan: Plan1853 }, Guard).possible, true);
  // Any other descent beside the can, sideways or straight, is refused the same way; one from far off ends far off and is not held to it.
  for (const Delta of [{ left: 0.5, farther: -0.4, up: -5.1 }, { left: -1, farther: 1, up: -3 }, { left: 0, farther: 0, up: -2.5 }]) {
    const Why = Propose1853(Hover, { move_delta_cm: Delta, wrist_pitch_degrees: -18.1 }, Guard, Grasp).Check.issues.join(' ');
    Assert.ok(Why.includes(Api.NextStepRefusalLead), `${JSON.stringify(Delta)}: ${Why}`);
  }
  Assert.equal(Api.NextStepReason(Guard, { x: 20, y: 30, z: 2 }, -40, TipOf(Hover), -10), null, 'far from the can');
});

Test('the NEXT STEP rule lets the legitimate moves through: the steps, rising out, Home, a held object, a contact task, no plan', () => {
  const Hover = Snapshot(Raws1853.Hover), Low = Snapshot(Raws1853.Low);
  const { Guard } = CheckIn1853(Hover, { Point: Plan1853.Hover, Pitch: Plan1853.HoverPitch });
  const Down = { x: 7.6, y: 31.1, z: 6.8 }, Start = TipOf(Hover);
  Assert.ok(Api.NextStepReason(Guard, Down, -18.1, Start, -5.1)?.startsWith(Api.NextStepRefusalLead));
  // Held (a put-down or a release), a contact task, no step (no plan, or a reply before this rule): not held to it.
  Assert.equal(Api.NextStepReason({ ...Guard, Held: true }, Down, -18.1, Start, -5.1), null, 'held');
  Assert.equal(Api.NextStepReason({ ...Guard, Intent: 'contact' }, Down, -18.1, Start, -5.1), null, 'contact');
  Assert.equal(Api.NextStepReason({ ...Guard, Step: null }, Down, -18.1, Start, -5.1), null, 'no step');
  Assert.equal(Api.BuildGraspGuard(Frame1854, { ...Task1853, Plan: null }, false, Guard.Step).Step, null, 'no GRASP PLAN: no step held');
  // Home from the hover point: far from the can, so the step does not hold it.
  const HomeTarget = Api.mirrorDisplayPoint(Api.HomeTarget);
  Assert.equal(Api.NextStepReason(Guard, HomeTarget, Api.HomeTarget.wristPitch, Start, HomeTarget.z - Start.z), null, 'Home');
  // At the bottom after that descent (18:55:06): the step rises straight up; the rise passes, so does a part of it and a rise
  // away from the can; staying low (sideways, or deeper) is refused.
  const Bottom = CheckIn1853(Low, { Point: { x: 7.6, y: 31.1, z: 6.8 }, Pitch: -18.1 });
  Assert.deepEqual(MotionIn(Bottom.Line), { move_delta_cm: { left: 0, farther: 0, up: 6.5 } });
  Assert.match(Bottom.Line, /the tip is below the can's top but not at the grasp point: rise straight up first/);
  for (const [Name, Reply] of [['the step', { move_delta_cm: { left: 0, farther: 0, up: 6.5 } }], ['the step, the wrist added (as at 18:55:06)', { move_delta_cm: { left: 0, farther: 0, up: 6.5 }, wrist_pitch_degrees: -18.1 }],
    ['part of the rise', { move_delta_cm: { left: 0, farther: 0, up: 2 } }], ['up and away', { move_delta_cm: { left: 0.6, farther: -0.4, up: 2 } }]]) {
    const Now = Propose1853(Low, Reply, Bottom.Guard, Bottom.Grasp);
    Assert.equal(Now.Check.possible, true, `${Name}: ${Now.Check.issues.join(' ')}`);
  }
  for (const [Name, Reply] of [['sideways, low', { move_delta_cm: { left: 0.5, farther: -0.3, up: 0 } }], ['deeper', { move_delta_cm: { left: 0, farther: 0, up: -0.3 } }],
    ['up toward the can', { move_delta_cm: { left: -1, farther: 1, up: 1 } }]]) {
    Assert.ok(Propose1853(Low, Reply, Bottom.Guard, Bottom.Grasp).Check.issues.some(Issue => Issue.includes(Api.NextStepRefusalLead)), Name);
  }
  // At the re-aimed hover point (18:55:49) the step descends; the model's copy of it (as recorded) passes.
  const Aimed = Snapshot(Raws1853.Aimed);
  const AtAim = CheckIn1853(Aimed, { Point: { x: 8.7, y: 30.5, z: 12.7 }, Pitch: -18.1 });
  Assert.deepEqual(MotionIn(AtAim.Line), { move_delta_cm: { left: -0.4, farther: 0.3, up: -5.3 }, wrist_pitch_degrees: -18.1 });
  Assert.equal(Propose1853(Aimed, { move_delta_cm: { left: -0.4, farther: 0.3, up: -5.3 }, wrist_pitch_degrees: -18.1 }, AtAim.Guard, AtAim.Grasp).Check.possible, true);
  // The same point as an absolute copy (target_cm, the 0.1 cm rounding apart) passes too; 0.5 cm off it does not.
  Assert.equal(Propose1853(Aimed, { target_cm: Plan1853.Grasp, wrist_pitch_degrees: -18.1 }, AtAim.Guard, AtAim.Grasp).Check.possible, true);
  Assert.ok(Propose1853(Aimed, { target_cm: { ...Plan1853.Grasp, x: Plan1853.Grasp.x - 0.5 }, wrist_pitch_degrees: -18.1 }, AtAim.Guard, AtAim.Grasp).Check.issues
    .some(Issue => Issue.includes(Api.NextStepRefusalLead)), 'off the step');
  // At the bottom of that descent (18:56:33): the close and lift, as recorded, passes.
  const Aligned = Snapshot(Raws1853.Aligned);
  const AtGrasp = CheckIn1853(Aligned, { Point: { x: 8.1, y: 30.9, z: 6.8 }, Pitch: -18.1 });
  Assert.deepEqual(MotionIn(AtGrasp.Line), { move_delta_cm: { left: 0, farther: 0, up: 6 }, gripper_before: 0 });
  Assert.equal(Propose1853(Aligned, { move_delta_cm: { left: 0, farther: 0, up: 6 }, gripper_before: 0 }, AtGrasp.Guard, AtGrasp.Grasp).Check.possible, true);
});

Test('the app\'s planned step: what the card, the chat and the next check-in say', () => {
  const { Step } = CheckIn1853(Snapshot(Raws1853.Hover), { Point: Plan1853.Hover, Pitch: Plan1853.HoverPitch });
  Assert.equal(Api.PlannedStepNote, "This is the app's planned NEXT STEP from the GRASP PLAN, not the AI's: the AI did not follow it. Check it before approving.");
  Assert.match(Api.PlannedStepReply(4, Step.Text), /^The AI did not follow the app's NEXT STEP in 4 revisions, so the app offers its own planned step instead: the arm holds the HOVER POINT but its tip stopped 0\.5 cm toward the can .* Do not descend from here: go to the hover point aimed 0\.7 cm past it, away from the can, so the arm stops at it\. Approve it to go on, or Block to stop the task\.$/);
  const Fact = Api.DescribeMotionFact({ Step: 3, StartTip: { x: 7.6, y: 31.1, z: 11.9 }, EndTip: { x: 8.5, y: 30.6, z: 12.1 }, Target: { x: 8.7, y: 30.5, z: 12.7 }, Servos: null, Planned: true });
  Assert.match(Fact, /\. That motion was the app's own NEXT STEP from the GRASP PLAN, approved by the user because your proposals did not follow it: copy each NEXT STEP exactly from now on, or reply ask\.$/);
  Assert.doesNotMatch(Api.DescribeMotionFact({ Step: 3, StartTip: { x: 0, y: 0, z: 0 }, EndTip: { x: 1, y: 0, z: 0 }, Target: null, Servos: null }), /app's own NEXT STEP/);
});

Test('the grasp gap: the 18:53 can planned at the user\'s gap; too small for the checks, refused with why and the setting named', () => {
  const At = Cm => As1001(() => Api.TaskObjectReachability(Can1853, { Servos: Snapshot(Raws1853.Home), ElbowMode: 'up', ClearanceCm: Cm },
    { Others: Api.NeighbourObjects(Frame1853, Can1853), Kind: 'can' }));
  // The pad c outside the can where the real fixed finger meets it, measured along the jaw axis at the grasp point.
  const PadGap = Plan => { const Real = Api.PlanArmPrism(Plan), U = Plan.JawAxis;
    return Math.min(...Real.Footprint.map(P => (P.x - Plan.Grasp.x) * U.x + (P.y - Plan.Grasp.y) * U.y)); };
  // The default: the plan as without the setting, and as logged.
  const Default = At(1.0).Plan;
  Assert.deepEqual([Default.Grasp, Default.Pitch, Default.ClearanceCm], [Plan1853.Grasp, Plan1853.Pitch, 1]);
  Assert.equal(Plan1853.ClearanceCm, Rules.FixedJawClearanceCm);
  // 0.8: the pad 0.2 cm nearer the can (planned again: a flatter wrist, 0.5 cm lower), every check passed; the model is told the gap.
  const Near = At(0.8);
  Assert.equal(Near.Kind, 'graspable', Near.Text);
  Assert.deepEqual([Near.Plan.ClearanceCm, Near.Plan.Pitch, Near.Plan.Grasp], [0.8, -20.6, { x: 7.9, y: 30.9, z: 6.3 }]);
  Assert.ok(PadGap(Near.Plan) < PadGap(Default) - 0.1, `${PadGap(Near.Plan)} / ${PadGap(Default)}`);
  const Line = Api.TaskObjectLine({ ...Task1853, DetectedNow: true, Binding: { Object: Can1853, Source: 'query', DistanceCm: null }, Reachability: Near,
    LastSeen: { ...Task1853.LastSeen, Reachability: Near }, Plan: { ...Task1853.Plan, Reach: Near } },
    TipOf(Snapshot(Raws1853.Home)), { GripperFraction: 0.03, Closed: false });
  Assert.match(Line, /the tip goes BESIDE the can, 0\.8 cm outside its left-near side \(planned as if it were 6\.8 cm across[^\n]*pushes it against the fixed jaw\. That 0\.8 cm is the grasp gap the user set for the fixed jaw\./);
  Assert.match(Line, /The GRASP POINT is 2\.5 cm outside its left-near side as the camera measured it: [^\n]*so the real finger comes down 0\.8 cm from it/);
  // 0.5 and 0.7: no pitch keeps the fixed finger 0.5 cm clear on the way down if the real one stands 0.4 cm nearer than
  // planned: refused, with the reason and the setting named, for the model and for the user. (0.3, below the planner's own
  // 0.5 cm margin, is no longer a setting: GraspGap.ts MinCm.)
  for (const [Cm, Why] of [[0.5, /the fixed finger would come within 0\.4 cm of the coffee cup if the real fixed finger stood 0\.4 cm nearer it than planned/],
    [0.7, /of the way down from the hover point the fixed finger would come within 0\.\d cm of the coffee cup if the real fixed finger stood 0\.4 cm nearer it than planned/]]) {
    // (The best-effort pass plans it anyway, warned that the fixed finger may push it on the way down.)
    Assert.ok(At(Cm).Plan?.Warnings?.some(Warning => /^the fixed finger may push it/.test(Warning)), `${Cm}`);
    const Small = Strictly(() => At(Cm));
    Assert.equal(Small.Kind, 'ungraspable', `${Cm}: ${Small.Text}`);
    Assert.deepEqual([Small.Refusal.Code, Small.Refusal.GapCm], ['path', Cm], `${Cm}`);
    Assert.match(Small.Refusal.Text, Why, `${Cm}`);
    Assert.ok(Small.Refusal.Text.endsWith(`; the grasp gap at the fixed finger is set to ${Cm.toFixed(1)} cm in Settings (the default is 1.0 cm): a wider gap may plan it`), Small.Refusal.Text);
    Assert.match(Small.Refusal.Advice, new RegExp(`Or widen the grasp gap: the grasp gap at the fixed finger is set to ${Cm.toFixed(1).replace('.', '\\.')} cm in Settings`));
    Assert.match(Small.Text, /^NOT GRASPABLE \(36\.9 cm from the right arm's pan axis\): /);
  }
});

// --- 18:56, the close and lift that closed on air --------------------------------------------------------

// 18:56:43: after the chained close and lift from the aligned grasp point (18:56:33): the jaws closed all the way (raw 649),
// the lift held its target Z 12.25 and the tip stopped at Z 11.39 under it. The can was measured partly (53% valid).
const Lifted1856 = Raw(3069, 466, 1628, 677, 649);
const Can1856 = Obj('can', 7.02, 35.95, -1.68, 10.24, [4.8, 3.7], -1.63, 0.532, { taskMatch: true, refined: true });
const Lift1856 = { Point: { x: 8.127325020864319, y: 30.682676857963237, z: 12.248558855804243 }, Pitch: -19.511278933436678 };
// A step as the app reads it from the anchor, and why it would not move the arm holding Held (null: it moves it).
const NoOpOf = (Servos, Json, Held) => {
  const Motion = Api.ParseResponse(`{"response_to_user":"","task_status":"continue","motion_proposal":${Json}}`, Anchor(Servos), Servos).motion;
  return Api.NoOpReason({ hasTargetMotion: Motion.hasTargetMotion, target: Motion.target, servoTargets: Motion.servoTargets },
    { Tip: Anchor(Servos), WristPitch: Anchor(Servos).wristPitch }, { ...Api.modelPointFromDisplayPoint(Held.Point), wristPitch: Held.Pitch }, { gripper: Servos.gripper.presentRaw });
};

Test('18:56:43: the missed close and lift is caught where the jaws closed, the kept plan is not called new, and the closed jaws rise above the lift the arm holds', () => {
  const Aligned = Snapshot(Raws1853.Aligned), Lifted = Snapshot(Lifted1856), Tip = TipOf(Lifted);
  Assert.ok(Math.hypot(Tip.x - 8.08, Tip.y - 30.61, Tip.z - 11.39) < 0.01, JSON.stringify(Tip));
  const Grasp = { GripperFraction: Fraction(Lifted), Closed: true, Held: Lift1856 };
  // The chain's gripper leg ran first, at the aligned grasp point: the app notes where the close was sent from, with this plan.
  const Noted = Api.NoteGripperMove(0.8, 0, false, null, TipOf(Aligned), Plan1853);
  Assert.equal(Noted.Closed, true);
  Assert.ok(Noted.Close.Plan === Plan1853 && Math.hypot(Noted.Close.Tip.x - Plan1853.Grasp.x, Noted.Close.Tip.y - Plan1853.Grasp.y) < 0.3, JSON.stringify(Noted.Close));
  // As live: judged by the tip alone, risen 4.6 cm by the lift: no miss, and the plan lock kept the plan (grasp_plan kept, missed null).
  Assert.equal(Api.ClosedOnNothing(Grasp, Plan1853, Tip), false);
  Assert.deepEqual(Api.KeepOrReplan(Task1853.Plan, Can1856, Tip, true, Task1853.Plan.Key, false), { Keep: true, Note: null, Corridor: null });
  // Now: judged where the jaws closed. Not for a plan made again since (the close was not made with it), nor once the jaws opened.
  Assert.equal(Api.ClosedOnNothing(Grasp, Plan1853, Tip, Noted.Close), true);
  Assert.equal(Api.ClosedOnNothing(Grasp, { ...Plan1853 }, Tip, Noted.Close), false, 'another plan');
  Assert.deepEqual(Api.NoteGripperMove(0, 0.8, true, Noted.Close, Tip, Plan1853), { Closed: false, Close: null }, 'opened');
  Assert.equal(Api.NoteGripperMove(0.03, 0, true, Noted.Close, Tip, Plan1853).Close, Noted.Close, 'closed again: where the jaws first closed stands');
  Assert.equal(Api.NoteGripperMove(0.8, 0.6, false, null, Tip, Plan1853).Close, null, 'narrowing to a plan\'s fraction is no close');
  // The can measured partly with the closed jaws over it: the plan is kept, marked missed (no step descends with it).
  const Verdict = Api.KeepOrReplan(Task1853.Plan, Can1856, Tip, true, Task1853.Plan.Key, true);
  Assert.deepEqual([Verdict.Keep, Verdict.Missed, Verdict.AfterMiss], [true, true, undefined]);
  Assert.match(Verdict.Note, /^the jaws closed on nothing at the GRASP PLAN's grasp point; re-measured at X 7\.0, Y 36\.0, 5\.3 cm from the 18:53:55 measurement the GRASP PLAN was made from, but only partly \(size 4\.8 x 3\.7, was 5\.5 x 5\.2\)/);
  const Seen = { At: At1853, Object: Can1856, Reachability: Reach1853 };
  const Missed = { ...Task1853, DetectedNow: true, Binding: { Object: Can1856, Source: 'query', DistanceCm: null }, Reachability: Reach1853, LastSeen: Seen,
    Plan: { ...Task1853.Plan, Missed: true, Confirmed: false, Note: Verdict.Note } };
  const Line = Api.TaskObjectLine(Missed, Tip, Grasp);
  Assert.match(Line, /the jaws closed all the way, so they missed it\. Rise straight up with the jaws closed first, to Z 12\.9, above the HOVER POINT height Z 12\.7 \(the arm already holds a target at Z 12\.2 and its tip stopped under it, so a rise to the hover height would not move it\): "motion_proposal":\{"move_delta_cm":\{"left":0,"farther":0,"up":1\.5\}\}\. The camera has not measured the can clear of the gripper since/);
  Assert.doesNotMatch(Line, /made from where the camera measured the can after the close/);
  // As live the step rose to the hover height (up 1.3): 0.4 cm from the lift the arm held, so it would not move it, twice, and the
  // task ended. The rise now ends 0.6 cm above that target, and moves the arm.
  Assert.equal(NoOpOf(Lifted, '{"move_delta_cm":{"left":0,"farther":0,"up":1.3}}', Lift1856), 'that target is 0.4 cm from the target the arm is already holding');
  const Step = Api.PlannedStepOf(Line, Api.mirrorDisplayPoint(Anchor(Lifted)), Anchor(Lifted).wristPitch);
  Assert.equal(Step.Json, '{"move_delta_cm":{"left":0,"farther":0,"up":1.5}}');
  Assert.equal(NoOpOf(Lifted, Step.Json, Lift1856), null);
  // The guard holds the model to it (the missed plan's step), and the rise passes it.
  const Guard = Api.BuildGraspGuard([Can1856], Missed, false, Step);
  Assert.ok(Guard.Step && Guard.Plan === null);
  Assert.equal(Api.NextStepReason(Guard, Step.Target, Step.Pitch, Tip, 1.5), null);
  // Risen there, the tip again stops under the target it holds (0.85 cm, as at 18:56:43): now high enough, the step opens the gripper.
  const Risen = { ...Tip, z: Step.Target.z - 0.85 }, Held = { Point: { ...Lift1856.Point, z: Step.Target.z }, Pitch: Lift1856.Pitch };
  Assert.deepEqual(MotionIn(Api.TaskObjectLine(Missed, Risen, { ...Grasp, Held })), { direct_servo_targets: [{ name: 'gripper', target_fraction: Plan1853.OpenFraction }] });
  // The plan kept without the miss (as live) no longer says it was made again, and one made again after the miss still does.
  Assert.doesNotMatch(Api.TaskObjectLine({ ...Missed, Plan: { ...Task1853.Plan } }, Tip, Grasp), /made from where the camera measured the can after the close|has not measured the can/);
  Assert.match(Api.TaskObjectLine({ ...Missed, Plan: { ...Task1853.Plan, AfterMiss: true } }, Tip, Grasp),
    /The GRASP PLAN above is made from where the camera measured the can after the close \(the close may have pushed it\), not where it was before\./);
  // A rise far from any target the arm holds is the plain one to the hover height, and never 1.0 cm or less (the app's no-move).
  Assert.deepEqual(MotionIn(Api.TaskObjectLine(Missed, Tip, { ...Grasp, Held: null })), { move_delta_cm: { left: 0, farther: 0, up: 1.3 } });
  Assert.deepEqual(MotionIn(Api.TaskObjectLine(Missed, { ...Tip, z: Plan1853.Hover.z - 1.04 }, { ...Grasp, Held: null })), { move_delta_cm: { left: 0, farther: 0, up: 1.1 } });
});

Test('the NEXT STEP TO COPY line, and its claim that the app refuses other moves, come only with a step the guard holds', () => {
  const Hover = Snapshot(Raws1853.Hover);
  const { Step, Guard } = CheckIn1853(Hover, { Point: Plan1853.Hover, Pitch: Plan1853.HoverPitch });
  Assert.ok(Guard.Step === Step);
  Assert.match(Api.NextStepReminder(Guard.Step, 'can'), /^NEXT STEP TO COPY \(from the TASK OBJECT line; the app refuses any other move near the can\)/);
  // NextStepReason holds no step for a contact task, an object the jaws hold, or a grasp with no plan: neither does the guard.
  for (const [Name, Other] of [['a contact task', Api.BuildGraspGuard(Frame1854, { ...Task1853, Intent: 'contact' }, false, Step)],
    ['the object in the jaws', Api.BuildGraspGuard(Frame1854, Task1853, true, Step)], ['no GRASP PLAN', Api.BuildGraspGuard(Frame1854, { ...Task1853, Plan: null }, false, Step)]]) {
    Assert.equal(Other.Step, null, Name);
    Assert.equal(Api.NextStepReminder(Other.Step, 'can'), null, Name);
  }
  // The review's probe: a box with no GRASP PLAN, as a contact task and as a grasp. Each line gives a NEXT STEP with its JSON (the
  // hover), the guard holds neither, so no reminder claims a refusal. Over the box the contact task descends; the grasp asks
  // (the GRASP RULE) instead of the descent onto it that the guard refuses for a grasp with no plan.
  const Box = Obj('box', 6.5, 28.5, -2.2, 6.0, [4.0, 3.0], -2.1, 0.95, { taskMatch: true });
  const HomeTip = TipOf(Snapshot(Raws1853.Home)), Over = { x: 6.6, y: 28.4, z: 9.0 };
  for (const Intent of ['contact', 'grasp']) {
    const Task = { Label: 'box', Intent, DetectedNow: true, Binding: { Object: Box, Source: 'query', DistanceCm: null }, Candidates: [], Reachability: null,
      Plan: null, LastSeen: { At: 0, Object: Box, Reachability: null } };
    const Far = Api.TaskObjectLine(Task, HomeTip, { GripperFraction: 0.8, Closed: false });
    const FarStep = Api.PlannedStepOf(Far, HomeTip, -45);
    Assert.deepEqual(JSON.parse(FarStep.Json), { target_cm: { x: 6.5, y: 28.5, z: 9 } }, Intent);
    const BoxGuard = Api.BuildGraspGuard([Box], Task, false, FarStep);
    Assert.deepEqual([BoxGuard.Step, Api.NextStepReminder(BoxGuard.Step, 'box')], [null, null], Intent);
    const Above = Api.TaskObjectLine(Task, Over, { GripperFraction: 0.8, Closed: false });
    if (Intent === 'contact') { Assert.ok(MotionIn(Above).move_delta_cm.up < -5, Above); continue; }
    Assert.match(Above, /NEXT STEP of the grasp \(tip now X 6\.6, Y 28\.4, Z 9\.0; gripper 80% open\): reply ask: the app has no GRASP PLAN for it, checked against the gripper's shape, so the jaws do not descend beside or onto it\./);
    Assert.doesNotMatch(Above, /"motion_proposal"/);
    // The descent the step gave before: the guard refuses it (here the fixed finger, at the box's centre, would go into it).
    const Down = { x: 6.5, y: 28.5, z: 1.9 }, Seed = Snapshot(Raws1853.Home);
    const From = { ...Solve(Over, -60, Seed).angles, gripper: OpenDeg }, To = Solve(Down, -60, Seed).angles;
    Assert.match(Api.GraspGuardReason(Model(Down, -60), To, From, Down.z - Over.z, BoxGuard) ?? '', /FIXED finger .* would go [\d.]+ cm into the box|descends beside the box, which has no GRASP PLAN/);
  }
});

// --- 2026-10-03 14:46-14:52, "Grab the can": the jaws closed in front of it ------------------------------
//
// back end/captures/2026-10-03/recording (ai.jsonl, grounding.jsonl, depth/*.png). The plan of 14:48:22 (logged: wrist -21.3,
// GRASP POINT X 1.9, Y 28.6, Z 4.5, 6.21 cm from the can's centre) put the fixed pad 2.7 cm outside the can as the camera
// measured it (the 1.7 cm finger offset over the 1.0 cm gap) and the can's centre at the pad's middle plus a 0.64 cm head start.
// The arm stopped there (14:50:40, 0.3 cm from the descent's target); the close pushed the can out past the fingertips and the
// jaws closed on nothing. The depth frame at that pose (analysed offline): the real gripper stood where the arm model puts it
// (the fixed finger 0.14 cm FARTHER from the can along the jaw axis, not 1.7 nearer), and the can's top where it was measured
// (its top's centroid X -4.0, Y 30.8; the 14:48:21 measurement X -3.96, Y 30.66). After the close the depth frame put the can's
// top at X 0.6, Y 32.4 (the app read it as a "black cup" at X 1.4, Y 33.1, partly hidden). A minute earlier (14:47:36) the same
// can was refused: "the jaws hold it only with the wrist -27.6..-0.1 deg, and there the gripper link could contact the basket".
const Snapshot1003 = Raw => Snapshot({ wrist_roll: 1570, ...Raw });
const Home1003 = Snapshot1003({ shoulder_pan: 3474, shoulder_lift: 3171, elbow_flex: 2688, wrist_flex: 855, gripper: 1919 });
const Stop1003 = Snapshot1003({ shoulder_pan: 3147, shoulder_lift: 424, elbow_flex: 2081, wrist_flex: 282, gripper: 1918 });
const Desk1003 = (Extra = {}) => ({ refined: false, ...Extra });
// The /locate frames: 14:48:21 (the plan's) and 14:47:36 (the refusal's), every measured object.
const Frame1448 = [Obj('pen', 20.39, 69, -0.9, 0.08, [0.8, 1.1], -0.74, 1, Desk1003({ extrapolated: true, outsideCalibrationCm: 39.4 })),
  Obj('bottle', 11.47, 30.83, -3.22, 4.55, [3.6, 6.9], -2.99, 0.888, Desk1003({ outsideCalibrationCm: 1.3 })),
  Obj('tape', 4.63, 26.19, -3.38, -2.1, [4.7, 5.5], -3.18, 1, Desk1003()),
  Obj('can', -3.96, 30.66, -3.37, 8.3, [5.6, 4.9], -3.24, 0.829),
  Obj('cup', -12.97, 31.58, -3.25, -1.15, [7.1, 8.4], -3.18, 0.532, Desk1003())];
const Frame1447 = [Obj('mouse', 16.44, 108.96, 1.1, 3.87, [9.2, 5.1], 1.32, 0.994, Desk1003({ extrapolated: true, outsideCalibrationCm: 75.7 })),
  Obj('white bottle', 11.47, 30.82, -3.24, 4.53, [3.6, 6.7], -3.04, 0.917, Desk1003({ outsideCalibrationCm: 1.3 })),
  Obj('black box', 8.82, 36.2, -2.97, -0.32, [4, 3.7], -2.84, 0.954, Desk1003({ outsideCalibrationCm: 4.9 })),
  Obj('tape', 11.04, 29.1, -3.44, -1.32, [8.1, 5.5], -3.18, 1, Desk1003()),
  Obj('can', -3.97, 30.28, -3.36, 8.3, [5.6, 4.1], -3.37, 0.825),
  Obj('bowl', -14, 31.6, -3.18, -1.22, [7.3, 4.4], -3.2, 0.53, Desk1003({ outsideCalibrationCm: 0.2 })),
  Obj('white container', 11.47, 30.82, -3.22, 4.54, [3.6, 6.7], -2.98, 0.913, Desk1003({ outsideCalibrationCm: 1.3 })),
  Obj('tape roll', 10.77, 29.1, -3.43, -0.89, [7.9, 5.8], -3.17, 1, Desk1003()),
  Obj('phone', 21.72, 71.67, -0.89, 0.33, [9.2, 1.3], -0.78, 1, Desk1003({ extrapolated: true, outsideCalibrationCm: 42.4 })),
  Obj('white cup', 11.47, 30.82, -3.25, 4.52, [3.6, 6.7], -3.06, 0.919, Desk1003({ outsideCalibrationCm: 1.3 }))];
const Reach1003 = Frame => { const Can = CanOf(Frame);
  return Api.TaskObjectReachability(Can, { Servos: Home1003, ElbowMode: 'up' }, { Others: Api.NeighbourObjects(Frame, Can), Kind: 'can' }); };
// The real can: a 6.6 cm drink can where the depth camera saw it at the grasp pose, and after the close.
const SeenCan1003 = { x: -4.0, y: 30.8 }, AfterClose1003 = { x: 0.6, y: 32.4 };
const RealCan1003 = (Centre = SeenCan1003) => ({ Label: 'can', BaseZ: -3.37, TopZ: 8.3, SupportZ: -3.24, Centre, SizeCm: [TrueCanCm, TrueCanCm],
  Footprint: Disc(Centre, TrueCanCm), Round: null });
// The close on a can at a pose: the fixed pad's gap, how far each jaw reaches past its centre (JawSweep StraddleOf), the push,
// and how far inside the fixed finger's tip its centre ends (negative: past it, squeezed out).
function Close1003(Angles, Can) {
  const Frame = Api.JawFrame({ ...Angles, gripper: OpenDeg }), Slice = Api.SliceInJawPlane(Can, Frame);
  const Straddle = Sweep.StraddleOf(Slice), Push = Sweep.CloseOn(Slice, OpenDeg, Sweep.SweepRules, 'push');
  return { Frame, Gap: Sweep.Gap(Sweep.FixedFingerPolygon, Slice), Straddle, Push,
    EndInsideCm: Push.Ok ? Straddle.Centre.z - Push.DriftCm - Sweep.FixedFingertipZ : null };
}

Test('2026-10-03 14:48 "Grab the can": the plan was short, not the measurement: the pad 3.1 cm from the real can, the jaws 1.2 and 0.5 cm past its centre, and the close squeezes it out past the tips where the camera saw it go', () => {
  // The arm stopped where the plan sent it: the logged tip, the logged GRASP POINT less the arm's sag.
  const Tip = TipOf(Stop1003);
  Assert.ok(Math.hypot(Tip.x - 1.986, Tip.y - 28.429, Tip.z - 4.137) < 0.01, JSON.stringify(Tip));
  Assert.ok(Math.hypot(Tip.x - 1.9, Tip.y - 28.6) < 0.25 && Math.abs(Tip.z - (4.5 - Rules.GraspSagCm)) < 0.1, 'at the logged GRASP POINT');
  // The camera's can then and the plan's measurement: within 0.2 cm.
  Assert.ok(Math.hypot(SeenCan1003.x - CanOf(Frame1448).x, SeenCan1003.y - CanOf(Frame1448).y) < 0.2);
  const At = Close1003(Api.currentAngles(Stop1003), RealCan1003());
  // The pad 3.1 cm from it (the gap the moving jaw must push it across); its centre 1.2 cm inside the fixed finger's tip and
  // 0.5 cm inside the moving jaw's reach: the jaws do not surround it.
  Assert.ok(At.Gap > 3.0 && At.Gap < 3.25, `pad ${At.Gap.toFixed(2)} cm from the can`);
  Assert.ok(At.Straddle.FixedCm > 1.0 && At.Straddle.FixedCm < 1.3 && At.Straddle.MovingCm > 0.3 && At.Straddle.MovingCm < 0.6, JSON.stringify(At.Straddle));
  // The push: 3.1 cm across and over 2 cm along the fingers, its centre ending past the fixed finger's tip.
  Assert.ok(At.Push.Ok && At.Push.DriftCm > 2.3 && At.EndInsideCm < -1, `drift ${At.Push.DriftCm}, ends ${At.EndInsideCm} cm inside the tip`);
  // Where the camera saw it after the close: its centre about 2.9 cm out along the fingers from the tool point (the fixed
  // fingertip 0.6 cm out), and moved over 3 cm along the jaw axis toward the fixed finger's side (onto its line: past its tip).
  const Level = Math.hypot(At.Frame.A.x, At.Frame.A.y), Uh = Level1003(At.Frame.U);
  const Out = ((AfterClose1003.x - Tip.x) * At.Frame.A.x + (AfterClose1003.y - Tip.y) * At.Frame.A.y) / Level;
  Assert.ok(Out > 2.5 && Out < 3.3, `${Out.toFixed(2)} cm out along the fingers`);
  const Along = P => (P.x - Tip.x) * Uh.x + (P.y - Tip.y) * Uh.y;
  Assert.ok(Along(AfterClose1003) < Along(SeenCan1003) - 3 && Along(AfterClose1003) < TrueCanCm / 2, `${Along(SeenCan1003).toFixed(2)} -> ${Along(AfterClose1003).toFixed(2)} cm along the jaw axis`);
});
function Level1003(V) { const L = Math.hypot(V.x, V.y); return { x: V.x / L, y: V.y / L }; }

Test('2026-10-03 14:48: planned now, the jaws surround the can: the fixed pad c beside it, both jaws 1.9 cm past its centre, and the close holds the real can deep in the jaws', () => {
  const Now = Reach1003(Frame1448), Plan = Now.Plan;
  Assert.equal(Now.Kind, 'graspable', Now.Text);
  Assert.equal(Plan.Closing.Mode, 'sweep');
  Assert.deepEqual([Plan.FingerOffsetCm, Plan.SagCm, Plan.ClearanceCm, Plan.Side, Plan.OpenFraction], [0, Rules.GraspSagCm, 1, 'left-near', 0.8]);
  // The wrist as near the preferred -60 deg as a deep grip allows (GraspRules.PreferredPitchDeg): no pitch the jaws hold the
  // can with reaches StraddleGoodCm around it here; -13.8 the deepest of those tried nearest -60 (at -21.3 only 1.6, and 0.5 cm
  // off where the camera saw it the close left the can 0.1 cm inside the fixed finger's tip). It was raised 3 cm, where the
  // palm rose clear of its top and let it in deeper; deep (GraspRules.DeepGrasp, 2026-10-04) it goes as deep at mid-height.
  Assert.deepEqual([Plan.Pitch, Plan.Grasp, Plan.Hover, Plan.RaiseCm], [-13.8, { x: 0.6, y: 30.1, z: 3 }, { x: 0.6, y: 30.1, z: 11.3 }, undefined]);
  Assert.ok(Plan.Deep && Plan.PalmGapCm >= Rules.PalmGapCm, `${Plan.PalmGapCm}`);
  // Both jaws 1.9 cm past its centre (planned as a 6.8 cm can), where the logged plan's reached 1.35 and 0.7.
  Assert.ok(Math.min(Plan.Straddle.FixedCm, Plan.Straddle.MovingCm) >= 1.85, JSON.stringify(Plan.Straddle));
  // Where the jaws close (the printed point less the sag), on the real can where the camera saw it: the pad 1.0 to 1.4 cm from
  // it (the plan's 6.8 cm disc against the true 6.6), its centre between the jaws (past the fixed pad, inside both reaches).
  const Closes = Solve({ ...Plan.Grasp, z: Plan.Grasp.z - Plan.SagCm }, Plan.Pitch, Home1003).angles;
  const At = Close1003(Closes, RealCan1003());
  Assert.ok(At.Gap >= 1.0 && At.Gap <= 1.4, `pad ${At.Gap.toFixed(2)} cm from the can`);
  Assert.ok(At.Straddle.Centre.x > -0.79 && At.Straddle.FixedCm >= 1.7 && At.Straddle.MovingCm >= 1.7, JSON.stringify(At.Straddle));
  // The close: pushed across and about 1 cm along the fingers, its centre ends 0.8 cm inside the fixed finger's tip, the two-pad
  // grip within the friction limit (the logged plan's: 1.5 cm past the tip).
  Assert.ok(At.Push.Ok && At.Push.GripDeg <= Sweep.SweepRules.MaxLeanDeg && At.EndInsideCm >= 0.7, `grip ${At.Push.GripDeg}, ends ${At.EndInsideCm} cm inside the tip`);
  // The can 0.5 cm off where the camera saw it, each way along the jaw axis and the fingers: still held, inside the tip.
  const U = Level1003(At.Frame.U), A = Level1003(At.Frame.A);
  for (const [Name, D] of [['toward the fixed jaw', { x: -0.5 * U.x, y: -0.5 * U.y }], ['away from it', { x: 0.5 * U.x, y: 0.5 * U.y }],
    ['toward the tips', { x: 0.5 * A.x, y: 0.5 * A.y }], ['toward the palm', { x: -0.5 * A.x, y: -0.5 * A.y }]]) {
    const Off = Close1003(Closes, RealCan1003({ x: SeenCan1003.x + D.x, y: SeenCan1003.y + D.y }));
    Assert.ok(Off.Push.Ok && Off.EndInsideCm >= 0.3, `${Name}: ${Off.Push.Code}, ends ${Off.EndInsideCm} cm inside the tip`);
  }
  // The way down from the hover point (3 cm over its top) keeps every part clear of the real can (the open moving jaw never
  // lands on it), the palm and servo (deep: GraspRules.DeepGrasp; before, the depth tolerance's room, 1.0 cm) PalmGapCm by
  // their exact gap.
  Assert.ok(Plan.Hover.z >= CanOf(Frame1448).topZ + 3);
  const Poses = Plan.Poses, Real = RealCan1003();
  Assert.equal(Api.PathContact(Poses.Hover, Poses.Grasp, Poses.Grasp.gripper, [Real], Rules.PlanMarginCm, Rules.PathSamples, Poses.JawDegs), null);
  for (let Index = 0; Index <= 10; Index++) {
    const Angles = { ...Poses.Hover };
    for (const Joint of ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex']) Angles[Joint] = Poses.Hover[Joint] + Index / 10 * (Poses.Grasp[Joint] - Poses.Hover[Joint]);
    const Gap_ = Api.PalmGap(Api.JawFrame(Angles).Links, Real);
    Assert.ok(Gap_ >= Rules.PalmGapCm, `palm ${Gap_.toFixed(2)} cm from it ${Index * 10}% of the way down`);
  }
  // The model is told the jaws reach around it; the steps: hover, straight down, close and lift.
  const Task = State(Now, CanOf(Frame1448));
  const Line = Api.TaskObjectLine(Task, TipOf(Home1003), { GripperFraction: 0.8, Closed: false });
  Assert.match(Line, /The jaws reach around it before they close: the GRASP POINT is \d\.\d cm farther out along the fingers than its centre, so the fixed finger reaches 1\.9 cm past its middle and the moving jaw, swinging closed, 1\.9 cm past it/);
  Assert.doesNotMatch(Line, /real fixed finger stands about/);
  Assert.deepEqual(MotionIn(Api.TaskObjectLine(Task, Plan.Hover, { GripperFraction: 0.8, Closed: false })),
    { move_delta_cm: { left: 0, farther: 0, up: Number((Plan.Grasp.z - Plan.Hover.z).toFixed(1)) }, wrist_pitch_degrees: Plan.Pitch });
  const Close = { move_delta_cm: { left: 0, farther: 0, up: 6 }, gripper_before: 0 };
  Assert.deepEqual(MotionIn(Api.TaskObjectLine(Task, Plan.Grasp, { GripperFraction: 0.8, Closed: false })), Close);
  // Stopped 0.9 cm short along the fingers and 1.4 cm away along the jaw axis, inside the alignment's window (1.5 away, 1.0
  // across): the jaws would reach under StraddleCloseCm around it, so the step is not the close (below its top: rise first).
  const F = Level1003(Plan.Straddle.FingerAxis);
  const Off = { ...Plan.Grasp, x: Plan.Grasp.x - 0.9 * F.x - 1.4 * Plan.JawAxis.x, y: Plan.Grasp.y - 0.9 * F.y - 1.4 * Plan.JawAxis.y };
  Assert.ok(Api.StraddleAtTip(Plan, Off) < Rules.StraddleCloseCm, `${Api.StraddleAtTip(Plan, Off)}`);
  const OffLine = Api.TaskObjectLine(Task, Off, { GripperFraction: 0.8, Closed: false });
  Assert.deepEqual(MotionIn(OffLine), { move_delta_cm: { left: 0, farther: 0, up: Number((Plan.Hover.z - Off.z).toFixed(1)) } });
  Assert.match(OffLine, /the tip is below the can's top but not at the grasp point: rise straight up first/);
  // 0.5 cm short: still around it, the close.
  const Near = { ...Plan.Grasp, x: Plan.Grasp.x - 0.5 * F.x, y: Plan.Grasp.y - 0.5 * F.y };
  Assert.ok(Api.StraddleAtTip(Plan, Near) >= Rules.StraddleCloseCm);
  Assert.deepEqual(MotionIn(Api.TaskObjectLine(Task, Near, { GripperFraction: 0.8, Closed: false })), Close);
});

Test('2026-10-03 14:47:36: the can refused for the basket at its centre is planned beside it: the wrist is the planner\'s to choose', () => {
  // With the tip on the can's centre at mid-height (where the planner took its first pose), the gripper link's tracking
  // envelope met the basket's rim at the pitches the jaws hold it with: the refusal of 14:47:36. Since the CAD-exact contact
  // model (v8, 2026-10-04) only the link's CAD box counts, and the arm reaches the centre clear of the basket at each.
  const Can = CanOf(Frame1447), Mid = (Can.z + Can.topZ) / 2;
  const Blocked = [-21.3, -16.3, -11.3, -6.3].filter(Pitch => !Api.solveIkPreview(Model({ x: Can.x, y: Can.y, z: Mid }, Pitch), 'up', Home1003).reachable);
  Assert.deepEqual(Blocked, [], `the centre blocked at wrist ${Blocked.join(', ')}`);
  const Now = Reach1003(Frame1447), Plan = Now.Plan;
  Assert.equal(Now.Kind, 'graspable', Now.Text);
  // The nearest the preferred -60 deg whose jaws reach StraddleGoodCm around it (at -17.6, 3 cm up, they reached 1.85): -12.6
  // 3 cm up before the deep grasp; deep (GraspRules.DeepGrasp, 2026-10-04) -10.1 at mid-height.
  Assert.deepEqual([Plan.Pitch, Plan.Grasp, Plan.RaiseCm], [-10.1, { x: 0.7, y: 29.9, z: 3 }, undefined]);
  Assert.ok(Plan.Pitch > -27.6 && Plan.Pitch < -0.1, 'inside the pitches the jaws hold it with');
  Assert.ok(Math.min(Plan.Straddle.FixedCm, Plan.Straddle.MovingCm) >= Rules.StraddleGoodCm - 0.05, JSON.stringify(Plan.Straddle));
  Assert.ok(Api.ExactlyReachable(Plan.Grasp, Plan.Pitch, { Servos: Home1003, ElbowMode: 'up', Others: [], Scan: () => null }, Plan.Hover.z), 'the printed point passes the exact reach check');
});

// The screen of a task object's pitches as PlanGrasp makes it (its centre's steepest reached pitch to level), with the fixed
// jaw's gap of 2026-10-03 (no extra standoff, GraspRules.FixedJawExtraStandoffCm: PlanGrasp screens with none after the full one).
const Screen1003 = (Frame, Label, Kind) => {
  const Task = Frame.find(Object_ => Object_.label === Label);
  const Context = { Servos: Home1003, ElbowMode: 'up', ExtraStandoffCm: 0, Others: Api.NeighbourObjects(Frame, Task).map(Other => Api.GraspPrism(Other)).filter(Boolean),
    Scan: Point => Api.ReachScan(Point, { Servos: Home1003, ElbowMode: 'up' }).Pitch };
  const Prism = Api.GraspPrism(Task, Kind);
  return Api.ScreenPitches(Prism, Context.Scan(Api.modelPointFromDisplayPoint({ ...Prism.Centre, z: (Prism.BaseZ + Prism.TopZ) / 2 })), Context);
};
// 16:40:18 "pick up the pill bottle" (vision record 188): planned live at wrist -32.1 (GRASP POINT X 3, Y 31.1, Z 2.1).
const Frame1640 = [Obj('phone', -1.66, 33.76, -2.28, 6.6, [13.4, 15.7], -2.08, 0.963, Desk1003()),
  Obj('pill bottle', -1.58, 32.86, -2.69, 5.92, [3.3, 3.2], -2.25, 0.855, { taskMatch: true, refined: true }),
  Obj('tape', 0.12, 31.93, -2.97, 4.51, [5.1, 4.9], -2.67, 0.908, Desk1003()), Obj('battery', 0.04, 28.04, -3.37, -0.82, [4.4, 3], -3.01, 0.999, Desk1003()),
  Obj('bowl', -13.72, 31.73, -3.1, -0.99, [7.7, 4.2], -3.16, 0.56, Desk1003()), Obj('smartphone', -1.65, 33.58, -2.27, 6.63, [13.3, 16.1], -2.23, 0.956, Desk1003()),
  Obj('adapter', 0.01, 28.04, -3.39, -0.82, [4.4, 3], -3.05, 0.999, Desk1003())];

Test('2026-10-03: the wrist as near the preferred -60 deg as the grip allows: the can and the pill bottle cannot be held that steep, and why', () => {
  Assert.equal(Rules.PreferredPitchDeg, -60);
  // The can (14:48): at -60 the fixed finger or the palm is already in it in the jaws' plane; to -24 the grip needs more
  // friction than the jaws have (lean); from -21 they hold it, and the plan (-13.8) is the deepest grip nearest -60.
  const Can = Screen1003(Frame1448, 'can', 'can'), At = Pitch => Can.reduce((A, B) => (Math.abs(B.Pitch - Pitch) < Math.abs(A.Pitch - Pitch) ? B : A));
  Assert.equal(At(-60).Verdict.Code, 'finger');
  Assert.equal(At(-35).Verdict.Code, 'lean');
  Assert.ok(At(-21.4).Verdict.Ok);
  // The pill bottle (16:40): its 3D grip needs more friction as the wrist tips down (20.5 deg at -20, 34 at -45, 49 at -60;
  // the jaws hold 20). Planned at the steepest pitch whose screened grip is within StrictScreenMarginDeg of that (no pitch
  // held it with the strict closing: as the live plans, the fallback, its grip said). Deep (GraspRules.DeepGrasp, 2026-10-04),
  // its palm 0.3 cm from it, the grip there holds within 20 in 3D: a plan with every check.
  const Pill = Screen1003(Frame1640, 'pill bottle', 'pill bottle');
  const Grip = Pitch => Pill.reduce((A, B) => (Math.abs(B.Pitch - Pitch) < Math.abs(A.Pitch - Pitch) ? B : A)).Verdict.Grip3DDeg;
  Assert.ok(Grip(-60) > 45 && Grip(-45) > 30 && Grip(-20) < 21, `${Grip(-60)}, ${Grip(-45)}, ${Grip(-20)}`);
  const Reach = Api.TaskObjectReachability(Frame1640[1], { Servos: Home1003, ElbowMode: 'up' }, { Others: Api.NeighbourObjects(Frame1640, Frame1640[1]), Kind: 'pill bottle' });
  Assert.equal(Reach.Kind, 'graspable', Reach.Text);
  const Plan = Reach.Plan;
  Assert.ok(Plan.Pitch >= -30 && Plan.Pitch <= -24 && Plan.Closing.Grip3DDeg <= Sweep.SweepRules.MaxLeanDeg, `${Plan.Pitch}, ${Plan.Closing.Grip3DDeg}`);
  Assert.ok(Plan.Deep && Plan.PalmGapCm >= Rules.PalmGapCm && Plan.PalmGapCm <= 0.45, `${Plan.PalmGapCm}`);
  for (const Item of Pill) if (Item.Pitch < Plan.Pitch - 1.5) Assert.ok(Item.Verdict.Grip3DDeg > Sweep.SweepRules.MaxLeanDeg + Rules.StrictScreenMarginDeg, `${Item.Pitch}: ${Item.Verdict.Grip3DDeg}`);
  // Its jaws well around it (a 4.4 cm bottle at most).
  Assert.ok(Math.min(Plan.Straddle.FixedCm, Plan.Straddle.MovingCm) >= Rules.StraddleGoodCm, JSON.stringify(Plan.Straddle));
});

Test('2026-10-03: a narrower round object where the can stood is planned with the jaws around it too; one too wide for them to reach around is refused as too wide, never planned short', () => {
  const Open = Snapshot({ gripper: 1922 });
  const Plan_ = Object_ => Api.TaskObjectReachability(Object_, { Servos: Open, ElbowMode: 'up' }, { Others: [] });
  const Straddle = Plan => Sweep.StraddleOf(Api.SliceInJawPlane(Plan.Prism,
    Api.JawFrame({ ...Solve({ ...Plan.Grasp, z: Plan.Grasp.z - (Plan.SagCm ?? 0) }, Plan.Pitch, Open).angles, gripper: OpenDeg })));
  // A 4 cm bottle (planned as up to 5.3 cm across): both jaws 2 cm past its centre where they close, its centre between them.
  const Bottle = Obj('bottle', -3.96, 30.66, -3.37, 8.3, [4.0, 3.8], -3.37, 0.9, { taskMatch: true, refined: true });
  const Planned = Plan_(Bottle);
  Assert.equal(Planned.Kind, 'graspable', Planned.Text);
  // (With the fixed jaw 2.54 cm farther out, GraspRules.FixedJawExtraStandoffCm, 2026-10-25, its centre is that much farther
  // across the jaws: the moving jaw reaches 1.8 cm past it, still more than the StraddleCm 1.5 the plan must have; with the
  // gap as before, 2 cm.)
  const S = Straddle(Planned.Plan);
  Assert.equal(Planned.Plan.ExtraStandoffCm, 2.54);
  Assert.ok(S.FixedCm >= Rules.StraddleGoodCm - 0.05 && S.MovingCm >= Rules.StraddleCm && S.Centre.x > -0.79, JSON.stringify(S));
  const S0 = Straddle(With({ FixedJawExtraStandoffCm: 0 }, () => Plan_(Bottle)).Plan);
  Assert.ok(S0.FixedCm >= Rules.StraddleGoodCm - 0.05 && S0.MovingCm >= Rules.StraddleGoodCm - 0.05 && S0.Centre.x > -0.79, JSON.stringify(S0));
  // As planned before 2026-10-03 (no straddle): a fingertip grasp, both jaws under 1 cm past its centre.
  const Old = Straddle(With({ StraddleRoundObjects: false, FixedJawExtraStandoffCm: 0 }, () => Plan_(Bottle)).Plan);
  Assert.ok(Math.max(Old.FixedCm, Old.MovingCm) < 1.0, JSON.stringify(Old));
  // A 5.5 cm jar (planned as up to 7.3 cm across: a jar may be wider than it reads): no pitch or height let both jaws reach
  // 1.5 cm past its centre before the palm came within its margin (1.0 cm) of it: refused as too wide, with why. As before
  // the deep grasp (GraspRules.DeepGrasp false):
  const JarObject = Obj('jar', -3, 32, -2.3, 8, [5.5, 5.5], -2.3, 0.9, { taskMatch: true, refined: true });
  const Jar = With({ DeepGrasp: false, BestEffort: false }, () => Plan_(JarObject));
  Assert.equal(Jar.Kind, 'ungraspable', Jar.Text);
  Assert.equal(Jar.Refusal.Code, 'wide');
  Assert.match(Jar.Refusal.Text, /with the fixed jaw 1\.0 cm beside it the jaws cannot reach around it: at wrist -?[\d.]+ deg the palm would come within 1\.0 cm of it on the way down before both jaws reach 1\.5 cm past its centre \(the jaws cannot surround it here\); a grasp with the jaws short of its middle squeezes it out past the finger tips$/);
  Assert.equal(Jar.Refusal.Advice, 'Try a narrower object: the jar is too wide for this gripper\'s jaws to reach around before they close.');
  // Deep, the palm lets it in to PalmGapCm: the jaws reach around it, a plan with every check. A 6 cm jar (up to 8 cm) the
  // strict passes refuse as too wide, never planned short; the best-effort pass plans it as measured, warned.
  const Deep = Plan_(JarObject).Plan;
  Assert.ok(Deep?.Deep && !Deep.Warnings && Math.min(Deep.Straddle.FixedCm, Deep.Straddle.MovingCm) >= Rules.StraddleCm, JSON.stringify(Deep?.Straddle));
  const Wider = Obj('jar', -3, 32, -2.3, 8, [6, 6], -2.3, 0.9, { taskMatch: true, refined: true });
  Assert.equal(Strictly(() => Plan_(Wider)).Refusal?.Code, 'wide');
  Assert.ok(Plan_(Wider).Plan?.Warnings?.length);
});
