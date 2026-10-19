// Tasks on surfaces besides the desk (back end/DeclaredSurfaces.json): what the app reads from the bridge
// about an object on a declared platform, and what it does with it. The declared surfaces are the
// bundle's fixture (a box 15 cm tall on the desk), not the file on this machine. No bridge, no hardware.
import { build as Build } from 'esbuild';
import { readFileSync as ReadFile } from 'node:fs';
import { createRequire as CreateRequire } from 'node:module';
import { test as Test } from 'node:test';
import Assert from 'node:assert/strict';
import Path from 'node:path';
import { fileURLToPath as FilePath } from 'node:url';
import { PinnedHome } from './UiTestLib.mjs';

const Root = Path.dirname(FilePath(import.meta.url));
const Platform = { Name: 'box platform', Kind: 'platform', TopZCm: 12.7, XRangeCm: [-25, -3], YRangeCm: [30, 44], BoardAboveZCm: null, Frame: 'cart' };
const Fixture = { name: 'declared-surfaces-fixture', setup(Builder) {
  Builder.onLoad({ filter: /DeclaredSurfaces\.json$/ }, () => ({ loader: 'json', contents: JSON.stringify({ Surfaces: [Platform] }) }));
} };
const Bundle = await Build({ stdin: { contents: ReadFile(Path.join(Root, 'src/App.tsx'), 'utf8')
  + '\nexport { parseAiResponse as ParseResponse, validateAiMotionProposal as Validate, buildAiSystemPrompt as AiPrompt, applyAiSnap as ApplySnap };'
  + '\nexport * from "./lib/AiMeasurements";'
  + '\nexport * from "./lib/DeclaredSurfaces";'
  + '\nexport * from "./lib/AiTableFloor";'
  + '\nexport { SnapAiTarget, SnapRules } from "./lib/AiTargetSnap";'
  + '\nexport { GraspRules } from "./lib/GraspPlan";'
  + '\nexport { INITIAL_SERVOS, IK_JOINTS, currentAngles, forwardKinematics, fractionToRaw, mirrorDisplayPoint, modelPointFromDisplayPoint, rawFromAngle, solveIkPreview,'
  + ' wristPitchFromAngles } from "./lib/robotModel";',
  resolveDir: Path.join(Root, 'src'), loader: 'tsx' }, bundle: true, platform: 'node', format: 'cjs', write: false, loader: { '.css': 'empty' },
  plugins: [Fixture, PinnedHome], logLevel: 'error' });
globalThis.window ??= { location: { hostname: 'localhost' } };
const AppModule = { exports: {} };
new Function('require', 'module', 'exports', Bundle.outputFiles[0].text)(CreateRequire(import.meta.url), AppModule, AppModule.exports);
const Api = AppModule.exports;

// The /locate reply for a can on the platform, as object_locator sends it: straight ahead of the right arm's pan axis
// (X -13.3), 33.5 cm out. A level wrist reaches its hover point (Z 27.7, 3 cm over its top) only out to about r 36.
const CanReply = { label: 'can', x: -13.4, y: 33.5, z: 12.7, top_z: 24.7, height_cm: 12.0, size_cm: [5.0, 4.6], distance_cm: 43.5, method: 'depth',
  support_measured: true, support_z_measured: 12.7, valid_fraction: 0.82, support_branch: 'surface', support_surface: 'box platform',
  support_kind: 'platform', outside_calibration_z_cm: 10.2, extrapolated: false, outside_calibration_cm: 0, unstable: false, refined: true,
  samples: 3, box: [0.4, 0.3, 0.5, 0.45], task_match: true, task_query_only: false };
const Located = Reply => Api.ReadLocateResponse({ ok: true, objects: [Reply], withheld: [], camera_check: { ok: true }, rms_cm: 0.75 });

Test('the app reads an object on a declared surface, its name and its heights against the calibration', () => {
  Assert.deepEqual(Api.DeclaredSurfaceList.map(Surface => Surface.Name), ['box platform']);
  const [Can] = Located(CanReply).objects;
  Assert.equal(Can.supportBranch, 'surface');
  Assert.equal(Can.supportSurface, 'box platform');
  Assert.equal(Can.outsideCalibrationZCm, 10.2);
  Assert.equal(Api.DeclaredSurfaceOf(Can)?.TopZCm, 12.7);
  // A desk object reads as before: no surface fields at all.
  const [Desk] = Located({ ...CanReply, z: -2.3, top_z: 9.7, support_branch: 'desk', support_surface: undefined }).objects;
  Assert.equal(Desk.supportBranch, 'desk');
  Assert.ok(!('supportSurface' in Desk) && !('outsideCalibrationZCm' in Desk));
  const Summary = Api.MeasuredObjectsSummary(Located(CanReply));
  Assert.match(Summary, /- can: centre X=-13\.4, Y=33\.5, base Z=12\.7, top Z=24\.7 .*resting on the platform "box platform" \(Z 12\.7, 15\.0 cm above the desk\)/);
  // A withheld object keeps the bridge's hint of a surface nobody declared.
  const Withheld = Api.ReadLocateResponse({ ok: true, objects: [], camera_check: { ok: true }, withheld: [{ label: 'can', reason: 'not measurable: x', undeclared_support_z: 12.68 }] });
  Assert.deepEqual(Withheld.withheld, [{ Label: 'can', Reason: 'not measurable: x', UndeclaredSupportZ: 12.68 }]);
});

// --- Phase 2: the task's floor is the surface its object stands on ----------------------------------

const Snapshot = (Raw, Base = Api.INITIAL_SERVOS) => {
  const Servos = structuredClone(Base);
  for (const [Name, Value] of Object.entries(Raw)) Object.assign(Servos[Name], { presentRaw: Value, targetRaw: Value, guiAngleRaw: null });
  return Servos;
};
const Home = Snapshot({ gripper: 1922 });
const HomeClosed = Snapshot({ gripper: 704 });
const At = (Servos, Angles) => {
  const Copy = structuredClone(Servos);
  for (const Joint of Api.IK_JOINTS) Object.assign(Copy[Joint], { presentRaw: Api.rawFromAngle(Copy[Joint], Angles[Joint]), targetRaw: Api.rawFromAngle(Copy[Joint], Angles[Joint]) });
  return Copy;
};
const TipOf = Servos => Api.mirrorDisplayPoint(Api.forwardKinematics(Api.currentAngles(Servos)).at(-1));
const Model = (Point, Pitch) => ({ ...Api.modelPointFromDisplayPoint(Point), wristPitch: Pitch });
const Solve = (Point, Pitch, Servos) => Api.solveIkPreview(Model(Point, Pitch), 'up', Servos);
const MotionIn = Line => {
  const Start = Line.indexOf('"motion_proposal":') + '"motion_proposal":'.length;
  Assert.ok(Start > 17, `no motion_proposal in ${Line}`);
  let Depth = 0;
  for (let Index = Start; Index < Line.length; Index++) {
    Depth += Line[Index] === '{' ? 1 : Line[Index] === '}' ? -1 : 0;
    if (Depth === 0) return JSON.parse(Line.slice(Start, Index + 1));
  }
  throw new Error('unbalanced');
};
const Reply = Motion => JSON.stringify({ response_to_user: 'x', task_status: 'continue', motion_proposal: Motion });
const AnchorOf = Servos => ({ ...Api.forwardKinematics(Api.currentAngles(Servos)).at(-1), wristPitch: Api.wristPitchFromAngles(Api.currentAngles(Servos)) });
const Parse = (Motion, Servos) => Api.ParseResponse(Reply(Motion), AnchorOf(Servos), Servos).motion;
const Context = Servos => ({ Servos, ElbowMode: 'up' });
const State = (Object_, Reachability) => ({ Label: 'can', DetectedNow: true, Binding: { Object: Object_, Source: 'name', DistanceCm: null },
  LastSeen: { At: 0, Object: Object_, Reachability }, Reachability, Intent: 'grasp',
  Plan: Reachability.Plan ? { Reach: Reachability, Source: Object_, At: 0, Key: null, Note: null, Confirmed: true, Corridor: null } : null });
const [Can] = Located(CanReply).objects;
const Started = Date.now();
const CanReach = Api.TaskObjectReachability(Can, Context(Home), { Others: [], Kind: 'can', Label: 'can' });
const PlanMs = Date.now() - Started;
const Plan = CanReach.Plan;

Test('a can on the declared 15 cm platform is planned top-down, its points above the platform and its floor the platform\'s', () => {
  Assert.equal(CanReach.Kind, 'graspable', CanReach.Text);
  Assert.ok(PlanMs < 20000, `${PlanMs} ms`);
  // Top-down as on the desk: the hover straight above the grasp point, the grasp beside the can at its height, and
  // off the desk one wrist from the hover point to the lift (here near level, as the desk's grasps of 2026-10-04).
  Assert.equal(Plan.Hover.x, Plan.Grasp.x);
  Assert.equal(Plan.Hover.y, Plan.Grasp.y);
  Assert.equal(Plan.HoverPitch, Plan.Pitch);
  Assert.ok(Plan.Pitch > -30 && Plan.Pitch <= 0, `wrist ${Plan.Pitch}`);
  Assert.ok(Plan.Grasp.z > 12.7 + 3 && Plan.Grasp.z < 24.7, `grasp Z ${Plan.Grasp.z}`);
  Assert.ok(Plan.Hover.z >= 24.7 + Api.GraspRules.HoverAboveTopCm - 0.05, `hover Z ${Plan.Hover.z}`);
  const Floor = Api.TaskSupport(Can).Floor;
  Assert.equal(Floor.Surface.Name, 'box platform');
  Assert.equal(Floor.FloorZ, 11.7);
  for (const [Point, Pitch] of [[Plan.Hover, Plan.HoverPitch], [Plan.Grasp, Plan.Pitch]]) {
    const Angles = Solve(Point, Pitch, Home).angles;
    Assert.ok(Api.GripperLowestZ(Angles) > Floor.FloorZ + 1, `lowest gripper point at Z ${Api.GripperLowestZ(Angles).toFixed(2)} over the floor at ${Floor.FloorZ}`);
  }
  console.log(`can on the platform: ${CanReach.Text} (${PlanMs} ms)`);
});

Test('the plan, the guard, the snap and the page\'s checks all accept the plan\'s steps and refuse a target under the platform', () => {
  const Task = State(Can, CanReach);
  const Guard = Api.BuildGraspGuard([Can], Task, false);
  Assert.equal(Guard.Floor.Surface.Name, 'box platform');
  Assert.ok(Guard.Prisms.some(Prism => Prism.Label === 'platform "box platform" under the task object (to 1 cm under its top, Z 11.7)' && Prism.TopZ === 11.7));
  const Line = (Tip, Fraction, Closed = false) => Api.TaskObjectLine(Task, Tip, { GripperFraction: Fraction, Closed });
  Assert.match(Line(TipOf(HomeClosed), 0.034), /^TASK OBJECT "can": detected this check-in at X -13\.4, Y 33\.5, base Z 12\.7, top Z 24\.7, standing on the platform "box platform" \(Z 12\.7, 15\.0 cm above the desk\): the tip and the gripper stay above this task's floor \(Z 11\.7 cm, 1 cm under the platform "box platform" the task object stands on\)/);
  // Every NEXT STEP from Home with the gripper closed, as the page runs a reply: the snap (the guard holding the moves
  // near the can to that step), then the page's check of the snapped motion (validateAiMotionProposal), then the move
  // (the IK's pose). Kinds: the snap's verdict for each tip move.
  let Servos = HomeClosed, Fraction = 0.034, Closed = false;
  const Steps = [], Kinds = [];
  for (let Index = 0; Index < 8 && !Closed; Index++) {
    const Tip = TipOf(Servos), Text = Line(Tip, Fraction, Closed);
    const Motion = MotionIn(Text);
    const Step = Api.PlannedStepOf(Text, Tip, Api.wristPitchFromAngles(Api.currentAngles(Servos)));
    const StepGuard = Api.BuildGraspGuard([Can], Task, false, Step);
    let Parsed = { ...Parse(Motion, Servos), guard: StepGuard };
    if (Parsed.hasTargetMotion) {
      const Snap = Api.SnapAiTarget({ Requested: Parsed.requestedTarget, Anchor: Parsed.anchor, Servos, ElbowMode: 'up', AbsoluteXY: Parsed.absoluteXY,
        Unreliable: [], Grasp: StepGuard });
      Assert.notEqual(Snap.Kind, 'refused', `step ${Index}: ${JSON.stringify(Motion)}: ${Snap.Text}`);
      Kinds.push(Snap.Kind === 'exact' ? 'exact' : `${Snap.Kind} (wrist ${Snap.PitchChangeDeg.toFixed(1)} deg, tip ${Snap.DistanceCm.toFixed(2)} cm)`);
      Parsed = Api.ApplySnap(Parsed, Snap);
    }
    const Checked = Api.Validate(Parsed, 'up', Servos, [], undefined, null, StepGuard);
    Assert.deepEqual(Checked.issues, [], `step ${Index}: ${JSON.stringify(Motion)}`);
    if (Parsed.hasTargetMotion) Servos = At(Servos, Checked.preview.angles);
    const Jaw = Motion.direct_servo_targets?.[0]?.target_fraction ?? Motion.gripper_before;
    if (Jaw != null) {
      Fraction = Jaw;
      Closed = Jaw <= 0.3;
      Servos = structuredClone(Servos);
      const Raw = Api.fractionToRaw(Servos.gripper, Jaw);
      Object.assign(Servos.gripper, { presentRaw: Raw, targetRaw: Raw });
    }
    Steps.push(Motion.target_cm ? `to X ${Motion.target_cm.x} Y ${Motion.target_cm.y} Z ${Motion.target_cm.z}` : Motion.move_delta_cm
      ? `up ${Motion.move_delta_cm.up}${Motion.gripper_before != null ? ` after the gripper to ${Motion.gripper_before}` : ""}` : `gripper to ${Jaw}`);
  }
  console.log(`steps: ${Steps.join('; ')}; snaps: ${Kinds.join(', ')}`);
  // Open, rise straight up out of Home to the hover height (the can's top is above Home's tip), across to the hover point,
  // down to the grasp point, close and lift: never under the platform's top over it.
  Assert.deepEqual(Steps, ['gripper to 0.8', `up ${(Plan.Hover.z - TipOf(Home).z).toFixed(1)}`, `to X ${Plan.Hover.x} Y ${Plan.Hover.y} Z ${Plan.Hover.z}`,
    `up ${(Plan.Grasp.z - Plan.Hover.z).toFixed(1)}`, 'up 6 after the gripper to 0']);
  // Every tip move exactly as the step gives it: the lift too, at the grasp's wrist (no turn with the can in the jaws).
  Assert.deepEqual(Kinds, ['exact', 'exact', 'exact', 'exact']);
  // A target under the platform's top beside the can: refused by the page and the snap, as a desk target under the desk is.
  const AtHover = At(Home, Solve(Plan.Hover, Plan.HoverPitch, Home).angles);
  const Low = { x: Plan.Grasp.x, y: Plan.Grasp.y, z: 10.5 };
  const Under = Api.Validate(Parse({ target_cm: Low, wrist_pitch_degrees: Plan.Pitch }, AtHover), 'up', AtHover, [], undefined, null, Guard);
  Assert.ok(Under.issues.some(Issue => /below this task's floor \(Z 11\.7 cm, 1 cm under the platform "box platform"/.test(Issue)), Under.issues.join(' | '));
  const UnderSnap = Api.SnapAiTarget({ Requested: Model(Low, Plan.Pitch), Anchor: AnchorOf(AtHover), Servos: AtHover, ElbowMode: 'up',
    AbsoluteXY: true, Unreliable: [], Grasp: Guard }, Api.SnapRules, true);
  Assert.equal(UnderSnap.Code, 'floor', JSON.stringify(UnderSnap));
  // The same target in a desk task (no surface): the desk's floor as before, so only the platform's floor refuses it.
  Assert.equal(Api.AiTableFloorReason(Model(Low, Plan.Pitch), -3), null);
  // The gripper swung under the platform's top over it on the way: the guard refuses it (a pose low beside the platform).
  const Beside = { x: -14, y: 31, z: 12.5 };
  const Swept = Api.Validate(Parse({ target_cm: Beside, wrist_pitch_degrees: -60 }, AtHover), 'up', AtHover, [], undefined, null, Guard);
  Assert.ok(Swept.issues.length > 0, 'a target 0.2 cm over the platform at a steep wrist');
});

Test('the prompt names the platform and its floor; a desk task\'s prompt is word for word as before', () => {
  const Floor = Api.TaskSupport(Can).Floor;
  const Desk = Api.AiPrompt('', true, false, 'Behind');
  Assert.equal(Api.AiPrompt('', true, false, 'Behind', Api.DeskFloor), Desk);
  Assert.match(Desk, /Objects rest ON the table \(Z=-2\.3\)/);
  Assert.match(Desk, /HARD FLOOR: outside the black mesh basket, never propose a motion that would put the gripper tip below Z = -3\.3 cm \(1 cm under the table surface\)/);
  const Surface = Api.AiPrompt('', true, false, 'Behind', Floor);
  Assert.match(Surface, /The task object rests ON the platform "box platform" \(Z 12\.7, 15\.0 cm above the desk\), not on the table \(Z=-2\.3\)/);
  Assert.match(Surface, /HARD FLOOR: outside the black mesh basket, never propose a motion that would put the gripper tip below Z = 11\.7 cm \(1 cm under the platform "box platform" .*\), anywhere in this task\./);
  Assert.match(Surface, /APPROACH RULE: NEVER move directly to a grasp/, 'still top-down');
  Assert.match(Surface, /where the desk figures do not hold: from Z 15 to Z 25 the arm reaches wrist -30 deg r 15\.\.27 cm, wrist 0 deg r 20\.\.39 cm, wrist -60 deg nowhere/);
  Assert.doesNotMatch(Desk, /desk figures do not hold/);
});

Test('a surface below the desk lowers the floor over its own extent only, each gripper point held to the floor under it', () => {
  const Low = { ...Api.DeclaredSurfaceList[0], Name: 'low shelf', TopZCm: -8, XRangeCm: [-40, -20], YRangeCm: [30, 44] };
  const Floor = Api.SurfaceFloor(Low, -8);
  Assert.equal(Floor.FloorZ, -9);
  Assert.equal(Api.FloorZAt(Floor, { x: -30, y: 35 }), -9);
  Assert.equal(Api.FloorZAt(Floor, { x: -10, y: 35 }), Api.AiTableFloorCm, 'over the desk beside it, the desk\'s floor');
  Assert.equal(Api.AiTableFloorReason(Model({ x: -30, y: 35, z: -8.5 }, -45), -3, Floor), null);
  Assert.match(Api.AiTableFloorReason(Model({ x: -10, y: 35, z: -5 }, -45), -3, Floor), /below the table floor \(Z -3\.3 cm/);
  Assert.match(Api.AiTableFloorReason(Model({ x: -30, y: 35, z: -9.5 }, -45), -3, Floor), /below this task's floor \(Z -9\.0 cm, 1 cm under the platform "low shelf"/);
  // A pose whose tip is over the shelf but whose gripper reaches back over the desk below the desk's floor: the gap counts per point.
  const Pose = Solve({ x: -21, y: 33, z: -4.5 }, -45, Home);
  if (Pose.reachable) {
    const Gap = Api.GripperFloorGap(Pose.angles, Floor), Lowest = Api.GripperLowestZ(Pose.angles);
    Assert.ok(Gap <= Lowest - Floor.FloorZ + 1e-9, `${Gap} vs ${Lowest - Floor.FloorZ}`);
  }
  // Raised or desk floors: the gap is the lowest point's height above the floor, as before.
  const Angles = Solve({ x: -13, y: 33, z: 15 }, -10, Home).angles;
  Assert.ok(Math.abs(Api.GripperFloorGap(Angles, Api.DeskFloor) - (Api.GripperLowestZ(Angles) - Api.AiTableFloorCm)) < 1e-9);
  Assert.ok(Math.abs(Api.GripperFloorGap(Angles, Api.TaskSupport(Can).Floor) - (Api.GripperLowestZ(Angles) - 11.7)) < 1e-9);
});

Test('a task that says where its object is picks the one there: the platform, or the desk', () => {
  const Query = Api.ParseTaskObject('grab the can on the platform');
  Assert.equal(Query.Support, 'surface');
  Assert.equal(Query.Phrase, 'can on the platform');
  Assert.equal(Api.ParseTaskObject('take the cup from the shelf').Support, 'surface');
  Assert.equal(Api.ParseTaskObject('grab the can on the desk').Support, 'desk');
  Assert.equal(Api.ParseTaskObject('put the can on the shelf').Support, null, 'the shelf is where it goes');
  const [Desk] = Located({ ...CanReply, x: 5, y: 36, z: -2.3, top_z: 9.7, support_branch: 'desk', support_surface: undefined }).objects;
  Assert.equal(Api.ResolveTaskObject(Query, [Desk, Can]).Binding.Object, Can);
  Assert.equal(Api.ResolveTaskObject(Api.ParseTaskObject('grab the can on the desk'), [Desk, Can]).Binding.Object, Desk);
  // Saying nothing of where it is: both, as before (the model says which).
  Assert.equal(Api.ResolveTaskObject(Api.ParseTaskObject('grab the can'), [Desk, Can]).Binding, null);
  // Nothing measured on a declared surface (none declared): "on the shelf" picks among them all, as before.
  Assert.equal(Api.ResolveTaskObject(Api.ParseTaskObject('grab the can on the shelf'), [Desk]).Binding.Object, Desk);
});

Test('the app plans no grasp where it cannot hold the arm to the surface', () => {
  const Cases = [
    [{ ...CanReply, support_surface: 'some other box' }, /a surface called "some other box" that this page does not declare/],
    [{ ...CanReply, z: 9.9, top_z: 21.9 }, /measured at base Z 9\.9, more than 2 cm from the platform/],
    [{ ...CanReply, x: 5.0 }, /measured at X 5\.0, Y 33\.5, outside the platform/],
  ];
  for (const [Reply_, Why] of Cases) {
    const [Object_] = Located(Reply_).objects;
    const Reach = Api.TaskObjectReachability(Object_, Context(Home), { Others: [], Kind: 'can', Label: 'can' });
    Assert.equal(Reach.Kind, 'ungraspable');
    Assert.equal(Reach.Refusal.Code, 'surface');
    Assert.match(Reach.Text, Why);
  }
  // A shelf with a board above: refused until the board is modelled (its floor still the shelf's).
  const Shelf = { ...Api.DeclaredSurfaceList[0], Kind: 'shelf', BoardAboveZCm: 30 };
  const Support = Api.TaskSupport(Can, 'can', [Shelf]);
  Assert.match(Support.Refusal.Text, /under a board at Z 30\.0, which the app's checks do not model yet/);
  Assert.equal(Support.Floor.FloorZ, 11.7);
  // On the desk: the desk's floor, nothing new.
  const [Desk] = Located({ ...CanReply, z: -2.3, top_z: 9.7, support_branch: 'desk' }).objects;
  Assert.deepEqual(Api.TaskSupport(Desk), { Floor: Api.DeskFloor, Refusal: null });
  // A can on a 10 cm box nobody declared, with the desk behind the box in view, measures as one object on the desk
  // (the bridge as before: TestSurfaceSupport.py): 22 cm tall and as wide as the box, which no grasp plan takes.
  const [Merged] = Located({ ...CanReply, z: -2.3, top_z: 19.7, height_cm: 22.0, size_cm: [10.3, 9.7], support_branch: 'desk', support_surface: undefined }).objects;
  const MergedReach = Api.TaskObjectReachability(Merged, Context(Home), { Others: [], Kind: 'can', Label: 'can' });
  Assert.equal(MergedReach.Kind, 'ungraspable', MergedReach.Text);
  Assert.equal(MergedReach.Refusal.Code, 'wide');
});
