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

// MILESTONE 2 (GraspPlan.ts GraspRules.FreeRoll, FullOpen): a free wrist roll, and the full opening.
// The recorded cases are the grey packet of 2026-10-18 (8.4 x 4.7 cm, 1.6 cm tall, just past the basket): the "cloth" the
// 23:08 task bound to (back end/TestFixtures/GreyPacket-2026-10-18/Locate.json, record 0) and the "grey packet" of
// 00:34 UTC (record 1), with the Home the app held (WhiteHolder-2026-10-18/HomePreset.json, wrist roll 1590). Turning the
// roll does not turn the closing direction across the desk unless the wrist is near straight down: there the rolls
// that line it up with the packet's 4.7 cm side tilt it 35-40 deg toward vertical, and the moving jaw closes past a
// 1.6 cm packet at every pitch the arm reaches clear of the basket. They stay refused, the rolls tried said. A box the
// open jaws cannot span across the arm's reach (12 x 4 x 6 cm) plans with the jaws turned across its 4 cm side.
const Root = Path.dirname(FilePath(import.meta.url));
const Repo = Path.join(Root, '..');
const Base = '3bcbff6';
const RecordedHome = { name: 'recorded-home', setup(Builder) {
  Builder.onLoad({ filter: /home_preset\.json$/ }, () => ({ loader: 'json', contents: ReadFile(Fixture('WhiteHolder-2026-10-18', 'HomePreset.json'), 'utf8') }));
} };
async function Load(LibDir, Home) {
  const Output = await Build({ stdin: { contents: `export * from './AiMeasurements'; export * from './GraspPlan'; export { CadForward } from './CadKinematics'; export { GripperLowestZOfLinks } from './AiTableFloor';
    export { INITIAL_SERVOS, currentAngles, angleFromRawValue, fractionToRaw, rawFromAngle, rawToFraction, forwardKinematics, mirrorDisplayPoint, modelPointFromDisplayPoint, solveIkPreview } from './robotModel';`,
    resolveDir: LibDir, loader: 'ts' }, bundle: true, platform: 'node', format: 'cjs', write: false, loader: { '.json': 'json' },
    nodePaths: [Path.join(Root, 'node_modules')], plugins: [Home] });
  const Module = { exports: {} };
  new Function('require', 'module', 'exports', Output.outputFiles[0].text)(CreateRequire(import.meta.url), Module, Module.exports);
  return Module.exports;
}
function CheckoutAt(Commit) {
  const Git = (...Args) => ExecFile('git', ['-C', Repo, ...Args], { maxBuffer: 256 << 20 });
  const Folder = MakeTemp(Path.join(TempDir(), 'FreeRoll-'));
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
const OldLib = Path.join(Folder, 'AI Controlled WebApp', 'src', 'lib'), NewLib = Path.join(Root, 'src', 'lib');
const [OldRec, NewRec, OldPin, New] = await Promise.all([Load(OldLib, RecordedHome), Load(NewLib, RecordedHome), Load(OldLib, PinnedHome), Load(NewLib, PinnedHome)]);
const Records = JSON.parse(ReadFile(Fixture('GreyPacket-2026-10-18', 'Locate.json'), 'utf8'));
const ReplyOf = Record => { const { kind, labels, task_object, diagnostics, depth_seq, depth_file, depth_bytes, t, time, ...Reply } = Record; return Reply; };

// As App.tsx BindTaskObject plans it.
function PlanRecorded(Api, Index, Label, Pick) {
  const Now = Api.ReadLocateResponse(ReplyOf(Records[Index]));
  const Object_ = Now.objects.find(Pick);
  const Support = Api.TaskSupport(Object_, Label);
  const Context = { Servos: structuredClone(Api.INITIAL_SERVOS), ElbowMode: 'up', ClearanceCm: 1, ...(Support.Floor.Surface || Support.Floor.Solids?.length ? { Floor: Support.Floor } : {}) };
  return { Object_, Context, Reach: Api.TaskObjectReachability(Object_, Context, { Others: Api.NeighbourObjects(Now.objects, Object_), Kind: Label, Label }) };
}
const Recorded = [[0, 'cloth', Object_ => Object_.label === 'cloth'], [1, 'grey packet', Object_ => Object_.taskMatch]];

const Box = (X, Y, Size, H, Label = 'box') => ({ label: Label, x: X, y: Y, z: -2.3, topZ: -2.3 + H, heightCm: H, sizeCm: Size, depthMeasured: true, extrapolated: false,
  unstable: false, supportMeasured: true, supportZMeasured: -2.3, validFraction: 0.9, supportBranch: 'desk', taskMatch: true, refined: true, outsideCalibrationCm: 0 });
const Home = () => structuredClone(New.INITIAL_SERVOS);
const Reach = (Api, Object_, Servos = structuredClone(Api.INITIAL_SERVOS)) => Api.TaskObjectReachability(Object_, { Servos, ElbowMode: 'up' }, { Others: [], Label: Object_.label });
// A box 12 cm across the arm's reach, 4 cm along it: wider than the open jaws span (11.1 cm) at the roll the arm holds.
const Wide = Box(8, 30, [12, 4], 6);
const Rolled = Reach(New, Wide);

Test('the recorded grey packet: refused before, refused now with the rolls tried, and why roll cannot help there', () => {
  for (const [Index, Label, Pick] of Recorded) {
    const Before = PlanRecorded(OldRec, Index, Label, Pick).Reach;
    Assert.equal(Before.Kind, 'ungraspable', Label);
    const { Object_, Context, Reach: Now } = PlanRecorded(NewRec, Index, Label, Pick);
    Assert.equal(Now.Kind, 'ungraspable', Label);
    Assert.equal(Now.Plan, null);
    Assert.match(Now.Text, /; nor with the wrist roll turned (-\d+(, )?){3} deg \(the jaws closing across [\d., ]+ cm of it, not 9\.\d\)/, Label);
    // The rolls tried line the closing direction up with the packet's 4.7 cm side (about 5 cm along it, not 9.4-9.5) ...
    const Prism = New.GraspPrism(Object_, Label);
    const { FromSpan, Rolls } = New.RollCandidates(Prism, -50, { ...Context, Others: [], Scan: () => null });
    Assert.ok(FromSpan > 9 && Rolls[0].Span < 5.5, `${FromSpan} ${JSON.stringify(Rolls)}`);
    // ... tilted toward vertical (the wrist is not straight down there): the vertical part of the closing direction.
    const Servos = { ...Context.Servos, wrist_roll: { ...Context.Servos.wrist_roll, presentRaw: Rolls[0].Raw, targetRaw: Rolls[0].Raw, guiAngleRaw: null } };
    const Pose = New.solveIkPreview({ ...New.modelPointFromDisplayPoint({ x: Object_.x, y: Object_.y, z: (Object_.z + Object_.topZ) / 2 }), wristPitch: -50 }, 'up', Servos);
    const U = New.JawFrame({ ...Pose.angles, gripper: 0 }).U;
    Assert.ok(Math.abs(U.z) / Math.hypot(U.x, U.y, U.z) > 0.5, `tilt ${U.z}`);
  }
});

Test('roll choice: a box the jaws cannot span across the reach is planned with the jaws turned across its narrow side', () => {
  Assert.equal(Reach(OldPin, Wide).Kind, 'ungraspable', 'refused at the roll the arm holds before');
  Assert.equal(Rolled.Kind, 'graspable', Rolled.Text);
  const Plan = Rolled.Plan, Roll = Home().wrist_roll;
  Assert.equal(Plan.RollFromRaw, Roll.presentRaw);
  Assert.ok(Plan.RollRaw != null && Math.abs(New.RawTurn(Roll.presentRaw, Plan.RollRaw)) > 100, `${Plan.RollRaw}`);
  // Closing across the 4 cm side (display Y), not the 12 cm one.
  Assert.ok(Math.abs(Plan.JawAxis.y) > 0.9, JSON.stringify(Plan.JawAxis));
  Assert.ok(Plan.WidthAcrossCm < 7, `${Plan.WidthAcrossCm} cm across the jaws, not 12`);
  // Its poses are the turned roll's: every check of the plan ran with it.
  Assert.ok(Math.abs(Plan.Poses.Grasp.wrist_roll - New.angleFromRawValue(Roll, Plan.RollRaw)) < 0.01);
  Assert.ok(Math.abs(Plan.Poses.Hover.wrist_roll - New.angleFromRawValue(Roll, Plan.RollRaw)) < 0.01);
  Assert.match(New.TaskObjectLine({ Label: 'box', DetectedNow: true, Binding: { Object: Wide, Source: 'name' }, LastSeen: { At: Date.now(), Object: Wide, Reachability: Rolled }, Reachability: Rolled,
    Plan: { Reach: Rolled, Source: Wide, At: 0, Key: New.GraspPlanKey(Home(), 'up', null), Note: null }, Intent: 'grasp' }), /checked with the wrist roll at raw \d+, not the raw \d+ the arm held/);
});

Test('the current roll is kept whenever it plans', () => {
  // Narrow across the reach already (4 cm across, 12 along), and the 8.4 x 4.7 cloth-sized box 5 cm tall.
  for (const Object_ of [Box(0, 28, [4, 12], 6), Box(0, 30, [8.4, 4.7], 5)]) {
    const Now = Reach(New, Object_);
    Assert.equal(Now.Kind, 'graspable', Now.Text);
    Assert.equal(Now.Plan.RollRaw, undefined);
    Assert.deepEqual({ ...Now.Plan, Ms: 0 }, { ...Reach(OldPin, Object_).Plan, Ms: 0 }, 'the same plan as before');
  }
});

Test('the steps turn the roll first, out of the way, and only to the plan\'s roll', () => {
  const Plan = Rolled.Plan, From = Plan.RollFromRaw;
  const HomeTip = New.mirrorDisplayPoint(New.forwardKinematics(New.currentAngles(Home())).at(-1));
  const Step = (Tip, RollRaw, Fraction = 0.04) => New.NextPlannedGraspStep(Plan, Tip, { GripperFraction: Fraction, Closed: false, RollRaw }, 'box');
  const Turn = Step(HomeTip, From);
  Assert.match(Turn, new RegExp(`turn the wrist roll \\d+ deg \\(raw ${From} to ${Plan.RollRaw}\\)`));
  Assert.ok(Turn.includes(`"motion_proposal":{"direct_servo_targets":[{"name":"wrist_roll","target_raw":${Plan.RollRaw}}]}`), Turn);
  // Low beside it: rise first.
  const Low = { ...Plan.Grasp, x: Plan.Grasp.x + 0.5 };
  Assert.match(Step(Low, From), /rise straight up to the HOVER POINT height Z [\d.]+ before turning it/);
  // At the plan's roll (within the tolerance): the usual steps, no roll in them.
  for (const At of [Plan.RollRaw, Plan.RollRaw + 10]) {
    const Next = Step(HomeTip, At);
    Assert.doesNotMatch(Next, /wrist_roll/);
    Assert.match(Next, /open the gripper/);
  }
});

Test('the guard checks the roll turn and holds the descent to the plan\'s roll; the plan lock keeps the plan once it is there', () => {
  const Plan = Rolled.Plan, Servos = Home(), Roll = Servos.wrist_roll;
  const Key = New.GraspPlanKey(Servos, 'up', null);
  const State = { Label: 'box', DetectedNow: true, Binding: { Object: Wide, Source: 'name' }, LastSeen: null, Reachability: Rolled,
    Plan: { Reach: Rolled, Source: Wide, At: 0, Key, Note: null }, Intent: 'grasp' };
  const Guard = New.BuildGraspGuard([Wide], State, false);
  const From = New.currentAngles(Servos);
  // The plan's own turn, from Home: clear.
  Assert.equal(New.WristRollTurnReason(From, Plan.RollRaw, New.angleFromRawValue(Roll, Plan.RollRaw), Guard), null);
  // Any other turn, a turn while holding it, and a turn without a plan at another roll: refused.
  const Other = Plan.RollRaw + 200;
  Assert.match(New.WristRollTurnReason(From, Other, New.angleFromRawValue(Roll, Other), Guard), /^Wrist roll is held during an object task/);
  Assert.match(New.WristRollTurnReason(From, Plan.RollRaw, New.angleFromRawValue(Roll, Plan.RollRaw), { ...Guard, Held: true }), /^Wrist roll is held/);
  Assert.match(New.WristRollTurnReason(From, Plan.RollRaw, New.angleFromRawValue(Roll, Plan.RollRaw), { ...Guard, Plan: { ...Plan, RollRaw: undefined } }), /^Wrist roll is held/);
  // Low beside the box: refused, rise first (as the NEXT STEP says).
  const LowAngles = { ...Plan.Poses.Grasp, wrist_roll: New.angleFromRawValue(Roll, Roll.presentRaw) };
  Assert.match(New.WristRollTurnReason(LowAngles, Plan.RollRaw, New.angleFromRawValue(Roll, Plan.RollRaw), Guard) ?? '', /^Turning the wrist roll there: the tip is low beside the box: rise straight up/);
  // At the hover point: checked on its way round, every gripper part against what was measured.
  const HoverAngles = { ...Plan.Poses.Hover, wrist_roll: New.angleFromRawValue(Roll, Roll.presentRaw) };
  Assert.equal(New.WristRollTurnReason(HoverAngles, Plan.RollRaw, New.angleFromRawValue(Roll, Plan.RollRaw), Guard), null);
  // The descent to the grasp point at the roll the arm held: refused; with the roll turned: not for the roll.
  const Target = { ...New.modelPointFromDisplayPoint(Plan.Grasp), wristPitch: Plan.Pitch };
  const AtHover = (RollDeg) => ({ ...Plan.Poses.Hover, wrist_roll: RollDeg });
  const Unturned = New.GraspGuardReason(Target, { ...Plan.Poses.Grasp, wrist_roll: From.wrist_roll }, AtHover(From.wrist_roll), -6, Guard);
  Assert.match(Unturned ?? '', /with the wrist roll where the GRASP PLAN was not checked: turn it to raw \d+ first/);
  const Turned = New.GraspGuardReason(Target, Plan.Poses.Grasp, AtHover(Plan.Poses.Grasp.wrist_roll), -6, Guard);
  Assert.doesNotMatch(Turned ?? '', /wrist roll/);
  // The plan lock: kept with the roll turned to the plan's; made again at any other roll.
  const Tip = New.mirrorDisplayPoint(New.forwardKinematics(New.currentAngles(Servos)).at(-1));
  const KeyAt = Raw => New.GraspPlanKey({ ...Servos, wrist_roll: { ...Roll, presentRaw: Raw, targetRaw: Raw } }, 'up', null);
  const Previous = { Source: Wide, At: 0, Key, Reach: Rolled };
  Assert.equal(New.KeepOrReplan(Previous, Wide, Tip, false, KeyAt(Plan.RollRaw)).Keep, true);
  Assert.equal(New.KeepOrReplan(Previous, Wide, Tip, false, KeyAt(Plan.RollRaw + 60)).Keep, false);
});

Test('full open: the jaws span less across opened past 80%, so it stays off; enabled, it clears the open jaw at a level wrist, its floor bound and descent holding the wider jaw', () => {
  const Servos = Home();
  const Pose = New.solveIkPreview({ ...New.modelPointFromDisplayPoint({ x: 5, y: 30, z: 2 }), wristPitch: -40 }, 'up', Servos).angles;
  const Deg = Fraction => New.angleFromRawValue(Servos.gripper, New.fractionToRaw(Servos.gripper, Fraction));
  const Across = Fraction => {
    const Angles = { ...Pose, gripper: Deg(Fraction) }, Frame = New.JawFrame(Angles);
    return Math.max(...New.GripperSamples(Angles, [Deg(Fraction)]).filter(S => S.Part === 'MovingJaw')
      .map(S => (S.P.x - Frame.Tool.x) * Frame.U.x + (S.P.y - Frame.Tool.y) * Frame.U.y + (S.P.z - Frame.Tool.z) * Frame.U.z));
  };
  Assert.ok(Across(0.8) > 10.8 && Across(1.0) < Across(0.8) - 1, `${Across(0.8)} ${Across(1.0)}`);
  Assert.equal(New.GraspRules.FullOpen, false);
  // The descent of a full-open plan is checked with the jaw anywhere from the steps' 80% to it (a stall short of it).
  const Degs = New.PlanJawDegs(Servos, 1);
  for (const Fraction of [0.8, 0.85, 0.9, 0.95]) Assert.ok(Degs.some(D => Math.abs(D - Deg(Fraction)) < 0.01), `${Fraction}`);
  Assert.deepEqual(New.PlanJawDegs(Servos, 0.8), OldPin.PlanJawDegs(OldPin.INITIAL_SERVOS, 0.8), 'unchanged at the usual opening');
  // At wrist -30 a 5 x 5 x 6 cm box has the 80% open jaw in it; fully open, the jaw is clear of it. The desk floor with
  // the wider jaw: the placement's lowest open-jaw point (fully open and every opening short of it) over the support.
  const Object_ = New.GraspPrism(Box(0, 30, [5, 5], 6), 'box');
  const Context = { Servos, ElbowMode: 'up', Others: [], Scan: () => null };
  const Without = New.BestEffortPlacement(Object_, -30, Context, false), With = New.BestEffortPlacement(Object_, -30, Context, true);
  Assert.equal(Without.Ok, false);
  Assert.match(Without.Why, /even 80% open the moving jaw would be in it/);
  Assert.ok(With.Ok, With.Why);
  Assert.equal(With.OpenFraction, 1);
  const Lowest = Fraction => New.GripperLowestZOfLinks(New.CadForward({ ...With.Angles, gripper: Deg(Fraction) }).Links);
  for (const Fraction of [0.8, 0.9, 1]) Assert.ok(Lowest(Fraction) >= Object_.SupportZ + New.GraspRules.OpenFloorClearanceCm - 1e-6, `${Fraction}: ${Lowest(Fraction)}`);
  // At wrist -40 both place it, at the same height (the wider jaw hangs no lower there).
  const W40 = New.BestEffortPlacement(Object_, -40, Context, false), F40 = New.BestEffortPlacement(Object_, -40, Context, true);
  Assert.ok(W40.Ok && F40.Ok && F40.T.z >= W40.T.z - 1e-6);
  // A full-open plan's opening is its own step (a chained open goes to 80% at most), never chained with the hover.
  const Full = { ...Reach(New, Box(0, 28, [4, 12], 6)).Plan, OpenFraction: 1 };
  const HomeTip = New.mirrorDisplayPoint(New.forwardKinematics(New.currentAngles(Servos)).at(-1));
  const Step = New.NextPlannedGraspStep(Full, HomeTip, { GripperFraction: 0.04, Closed: false, RollRaw: Servos.wrist_roll.presentRaw }, 'box');
  Assert.ok(Step.includes('"motion_proposal":{"direct_servo_targets":[{"name":"gripper","target_fraction":1.0}]}'), Step);
  Assert.doesNotMatch(Step, /gripper_before/);
});


// The loosened grasp checks (codex/generalized-grasp, merged 2026-10-22) hold for a rolled plan as for any: a solid the
// camera measured against the box (task_contact, a desk object's scene solid) is planned around, with the part of it the
// box's measurement took in (TaskContactPrisms), the plan's own close-and-lift checked as the guard will check it and
// stepped (DeskLift, LiftStepCm), and the roll turn checked against those solids, touched or not.
Test('a rolled grasp of the long box with a solid against it gets the contact solid and a guard-checked stepped lift', () => {
  // A solid 5.3 cm tall against the box's +X end, its measurement run 1.7 cm into it (bounds_cm to X 15.7, as the pill
  // bottle's ran into the keyboard): the bridge removed that footprint (grown 0.3 cm) as the box, and measured the solid
  // against it from there (task_contact). The part between the box's prism (to X 14.5) and the contact is in no cell.
  const Removed = { min: [1.7, 27.7, -2.3], max: [16, 32.3, 3.7] }, Measured = { min: [16, 28.5, -2.3], max: [18.5, 31.5, 3] };
  const Solid = { label: 'observed environment against it', min: Measured.min.map(V => V - 0.5), max: Measured.max.map(V => V + 0.5),
    measured: Measured, removed: Removed, against: true };
  const Object_ = { ...Wide, deskSolids: [Solid] };
  const Floor = New.TaskSupport(Object_, 'box').Floor;
  Assert.deepEqual(Floor.Solids, [Solid]);
  const Servos = Home(), Roll = Servos.wrist_roll;
  const Reach_ = New.TaskObjectReachability(Object_, { Servos, ElbowMode: 'up', Floor }, { Others: [], Label: 'box' });
  Assert.equal(Reach_.Kind, 'graspable', Reach_.Text);
  const Plan = Reach_.Plan;
  // Rolled: the roll the arm holds does not plan it (the roll choice test above).
  Assert.equal(Plan.RollFromRaw, Roll.presentRaw);
  Assert.ok(Plan.RollRaw != null && Math.abs(New.RawTurn(Roll.presentRaw, Plan.RollRaw)) > 100, `${Plan.RollRaw}`);
  Assert.ok(Math.abs(Plan.Poses.Grasp.wrist_roll - New.angleFromRawValue(Roll, Plan.RollRaw)) < 0.01);
  // The contact solid, and the part of it the measurement took in, are solids the rolled plan keeps clear of.
  const Inner = New.TaskContactPrisms(Floor, Plan.Prism);
  Assert.ok(Inner.length >= 1 && Inner.every(Prism => Prism.Scene && Prism.Against), JSON.stringify(Inner));
  const Solids = [...New.ScenePrisms(Floor), ...Inner];
  Assert.ok(Solids.some(Prism => Prism.Label === 'observed environment against it'));
  for (const Pose of [Plan.Poses.Hover, Plan.Poses.Grasp]) Assert.equal(New.DeepestContact(New.GripperSamples(Pose, Plan.Poses.JawDegs), Solids, 0.4), null);
  Assert.ok(Plan.Caveats?.some(Why => /the camera did not measure it apart from the observed environment against it/.test(Why)), JSON.stringify(Plan.Caveats));
  // Its lift is stepped (a 6 cm joint-linear lift would carry the held box toward the solid), and the steps say so.
  const Step = New.PlanLiftStepCm(Plan);
  Assert.ok(Plan.LiftStepCm != null && Step === Plan.LiftStepCm && Step < 6 && Step >= 1.5, `${Plan.LiftStepCm}`);
  const AtRoll = { ...Servos, wrist_roll: { ...Roll, presentRaw: Plan.RollRaw, targetRaw: Plan.RollRaw, guiAngleRaw: null } };
  const Close = New.NextPlannedGraspStep(Plan, Plan.Grasp, { GripperFraction: Plan.OpenFraction, Closed: false, RollRaw: Plan.RollRaw }, 'box');
  Assert.ok(Close.includes(`"motion_proposal":${New.CloseAndLiftJson(Step)}`), Close);
  const Next = New.NextPlannedGraspStep(Plan, { ...Plan.Grasp, z: Plan.Grasp.z + Step }, { GripperFraction: 0.5, Closed: true, RollRaw: Plan.RollRaw }, 'box');
  Assert.ok(Next.includes(`"motion_proposal":{"move_delta_cm":{"left":0,"farther":0,"up":${Step}}}`), Next);
  // The guard, the box held: each step of the plan's lift, solved from where the last ended at the plan's roll, runs.
  const State = { Label: 'box', DetectedNow: true, Binding: { Object: Object_, Source: 'name' }, LastSeen: null, Reachability: Reach_,
    Plan: { Reach: Reach_, Source: Object_, At: 0, Key: New.GraspPlanKey(AtRoll, 'up', null), Note: null }, Intent: 'grasp' };
  const Held = New.BuildGraspGuard([Object_], State, true);
  Assert.ok(Held.Floor?.Solids?.length, 'the held guard checks the box against the desk solids');
  Assert.ok(Held.Prisms.some(Prism => Prism.Label === 'observed environment against it (inside its measured box)'));
  const Joints = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex'];
  const ServosAt = Angles => Object.fromEntries(Object.entries(AtRoll).map(([Name, Servo]) => [Name, Joints.includes(Name)
    ? { ...Servo, presentRaw: New.rawFromAngle(Servo, Angles[Name]), ...(Servo.guiAngleRaw != null ? { guiAngleRaw: New.rawFromAngle(Servo, Angles[Name]) } : {}) } : Servo]));
  const Lift = (By, Total = 6) => {
    let From = { ...Plan.Poses.Grasp, gripper: Plan.Poses.JawDegs[0] };
    const Results = [];
    for (let Up = By; Up <= Total + 1e-9; Up += By) {
      const Model = { ...New.modelPointFromDisplayPoint({ ...Plan.Grasp, z: Plan.Grasp.z + Up }), wristPitch: Plan.Pitch };
      const Preview = New.solveIkPreview(Model, 'up', ServosAt(From));
      const To = { ...Preview.angles, gripper: From.gripper };
      Assert.ok(Math.abs(To.wrist_roll - Plan.Poses.Grasp.wrist_roll) < 0.01, 'the lift holds the plan\'s roll');
      Results.push(Preview.reachable ? New.GraspGuardReason(Model, To, From, By, Held) ?? 'ALLOWED' : 'unreachable');
      From = To;
    }
    return Results;
  };
  Assert.deepEqual(Lift(Step), Array.from({ length: Math.round(6 / Step) }, () => 'ALLOWED'), `the plan's lift, ${Step} cm at a time`);
  // In one move the guard refuses it, for the solid's part the measurement took in: the steps are why it runs.
  Assert.match(Lift(6)[0], /^That route is blocked: the held box would (touch|be pushed further into) observed environment against it \(inside its measured box\)/);
  // A carry sideways into the solid, held: refused for the held box.
  const Closed = { ...Plan.Poses.Grasp, gripper: Plan.Poses.JawDegs[0] };
  const Toward = { ...New.modelPointFromDisplayPoint({ ...Plan.Grasp, x: Plan.Grasp.x + 2, z: Plan.Grasp.z + 1 }), wristPitch: Plan.Pitch };
  const Sideways = New.solveIkPreview(Toward, 'up', ServosAt(Closed));
  Assert.ok(Sideways.reachable);
  Assert.match(New.GraspGuardReason(Toward, { ...Sideways.angles, gripper: Closed.gripper }, Closed, 1, Held) ?? '',
    /^That route is blocked: the held box would (touch|be pushed further into) observed environment against it/);
  // The roll turn (before the approach, not held) is checked against the same solids: clear from Home, and a scene solid
  // the gripper already touches where it turns is not exempt (as on any guard check, ActivePrisms).
  const Open = New.BuildGraspGuard([Object_], { ...State, Plan: { ...State.Plan, Key: New.GraspPlanKey(Servos, 'up', null) } }, false);
  Assert.ok(Open.Prisms.some(Prism => Prism.Label === 'observed environment against it (inside its measured box)'));
  const TargetDeg = New.angleFromRawValue(Roll, Plan.RollRaw);
  Assert.equal(New.WristRollTurnReason(New.currentAngles(Servos), Plan.RollRaw, TargetDeg, Open), null);
  const HoverAngles = { ...Plan.Poses.Hover, wrist_roll: New.angleFromRawValue(Roll, Roll.presentRaw) };
  Assert.equal(New.WristRollTurnReason(HoverAngles, Plan.RollRaw, TargetDeg, Open), null);
  const At = New.GripperSamples(HoverAngles, [HoverAngles.gripper]).find(Sample => Sample.Part === 'MovingJaw').P;
  const Touched = { Label: 'observed environment', BaseZ: At.z - 0.6, TopZ: At.z + 0.6, SupportZ: At.z - 0.6, Scene: true, Centre: { x: At.x, y: At.y }, SizeCm: [1.2, 1.2],
    Footprint: [{ x: At.x - 0.6, y: At.y - 0.6 }, { x: At.x + 0.6, y: At.y - 0.6 }, { x: At.x + 0.6, y: At.y + 0.6 }, { x: At.x - 0.6, y: At.y + 0.6 }] };
  Assert.match(New.WristRollTurnReason(HoverAngles, Plan.RollRaw, TargetDeg, { ...Open, Prisms: [...Open.Prisms, Touched] }) ?? '',
    /^Turning the wrist roll there would bring the gripper's .* the observed environment measured at/);
});
