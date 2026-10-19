import { build as Build } from 'esbuild';
import { readFileSync as ReadFile } from 'node:fs';
import { createRequire as CreateRequire } from 'node:module';
import { test as Test } from 'node:test';
import Assert from 'node:assert/strict';
import Path from 'node:path';
import { fileURLToPath as FilePath } from 'node:url';

// The AI target snap (src/lib/AiTargetSnap.ts): an AI target the arm cannot
// reach exactly moves to the nearest one it can, within caps, the way the
// manual GUI snaps a dragged target; what the model and the human are told.
// App.tsx's own parser, validator and snap helpers are bundled with it, so the
// snap and the checks share one module instance. No bridge, no hardware.
const Root = Path.dirname(FilePath(import.meta.url));
const AppBundle = await Build({ stdin: { contents: ReadFile(Path.join(Root, 'src/App.tsx'), 'utf8')
  + '\nexport { parseAiResponse as ParseResponse, validateAiMotionProposal as Validate, INITIAL_SERVOS as Servos, applyAiSnap as ApplySnap,'
  + ' aiSnapInput as SnapInput, motionRecord as MotionRecord, approvalSnapNote as SnapNote, seededServos as SeededServos, servoSeed as ServoSeed,'
  + ' taskMotionSnap as TaskMotionSnap, buildAiSystemPrompt as AiPrompt };'
  + '\nexport { SnapAiTarget, SnapRules } from "./lib/AiTargetSnap";'
  + '\nexport { FindNearestValidTarget, FindNearestLocalTarget } from "./lib/TargetWorkspace";'
  + '\nexport { FindReachableWristTarget } from "./lib/AutomaticWrist";'
  + '\nexport { AiTableFloorCm, GripperLowestZ, OverBasket } from "./lib/AiTableFloor";'
  + '\nexport { BasketBounds, BasketCollisionVersion, BasketPoseReason, TrayBounds } from "./lib/BasketCollision";'
  + '\nexport { GetCadAlignment } from "./lib/CadKinematics";'
  + '\nexport * from "./lib/AiTaskLoop";'
  + '\nexport { currentAngles, forwardKinematics, wristPitchFromAngles, solveIkPreview, mirrorDisplayPoint, modelPointFromDisplayPoint, IK_JOINTS,'
  + ' HOME_MODEL_TARGET_CM, GetIkAngleLimits, rawFromAngle } from "./lib/robotModel";'
  + '\nexport { BuildGraspGuard, TaskObjectReachability } from "./lib/AiMeasurements";'
  + '\nexport { GraspPlanKey, GraspRules } from "./lib/GraspPlan";',
  resolveDir: Path.join(Root, 'src'), loader: 'tsx' }, bundle: true, platform: 'node', format: 'cjs', write: false, loader: { '.css': 'empty' }, logLevel: 'error' });
globalThis.window ??= { location: { hostname: 'localhost' } };
const AppModule = { exports: {} };
new Function('require', 'module', 'exports', AppBundle.outputFiles[0].text)(CreateRequire(import.meta.url), AppModule, AppModule.exports);
const App = AppModule.exports;

// Recorded arm states (present_raw from the model requests of 2026-09-29).
const S1605 = { shoulder_pan: 3471, shoulder_lift: 3168, elbow_flex: 2686, wrist_flex: 855, wrist_roll: 1522, gripper: 699 };
const S1610 = { shoulder_pan: 3244, shoulder_lift: 771, elbow_flex: 851, wrist_flex: 1742, wrist_roll: 1522, gripper: 699 };
const Home = Object.fromEntries(Object.entries(App.Servos).map(([Name, Servo]) => [Name, Servo.presentRaw]));
// The [UNRELIABLE] measurements the model aimed at (vision records 23 and 31).
const UsbDrive = { label: 'USB drive', x: 4.07, y: 32.29, z: -2.59, extrapolated: true, unstable: false, topZ: -0.38, heightCm: 2.2, sizeCm: [2, 1.4],
  depthMeasured: true, supportMeasured: true, supportZMeasured: -2.31, validFraction: 0.996, supportBranch: 'desk' };
const BlackObject = { label: 'small black object', x: 4.23, y: 32.5, z: -2.52, extrapolated: true, unstable: false, topZ: -0.18, heightCm: 2.35, sizeCm: [1.4, 2.3],
  depthMeasured: true, supportMeasured: true, supportZMeasured: -2.7, validFraction: 0.999, supportBranch: 'desk' };

const Snapshot = Raw => {
  const Servos = structuredClone(App.Servos);
  for (const [Name, Value] of Object.entries(Raw)) Object.assign(Servos[Name], { presentRaw: Value, targetRaw: Value, guiAngleRaw: null });
  return Servos;
};
const TipOf = Servos => { const Angles = App.currentAngles(Servos); return { ...App.forwardKinematics(Angles).at(-1), wristPitch: App.wristPitchFromAngles(Angles) }; };
const Reply = Motion => JSON.stringify({ response_to_user: 'x', task_status: 'continue', motion_proposal: Motion });
const Display = Pose => App.mirrorDisplayPoint(Pose);
const Distance = (A, B) => Math.hypot(A.x - B.x, A.y - B.y, A.z - B.z);
const Converged = (Target, Servos, Elbow = 'up') => {
  const Preview = App.solveIkPreview(Target, Elbow, Servos);
  return Preview.reachable && Preview.errorCm <= 0.02 && Math.abs(Preview.effectiveWristPitch - Target.wristPitch) <= 0.08;
};
// A reply as the app turns it into a proposal: parsed, snapped (here on the
// page thread; the app runs the same function in its worker) and checked.
function Propose(Raw, Target, Wrist, { Unreliable = [], Elbow = 'up', Limits, Motion: Given } = {}) {
  const Servos = Snapshot(Raw), Current = TipOf(Servos);
  const Motion = App.ParseResponse(Reply(Given ?? { target_cm: Target, ...(Wrist == null ? {} : { wrist_pitch_degrees: Wrist }) }), Current, Servos).motion;
  const Started = performance.now();
  const Snap = App.SnapAiTarget(App.SnapInput(Motion, Servos, Elbow, Limits, Unreliable));
  const Ms = performance.now() - Started;
  const Snapped = App.ApplySnap({ ...Motion, snapSeed: App.ServoSeed(Servos), unreliable: Unreliable }, Snap);
  return { Servos, Motion: Snapped, Snap, Ms, Validation: App.Validate(Snapped, Elbow, Servos, Unreliable, Limits) };
}
// A small deterministic generator for jitter.
function Random(Seed) { return () => { Seed = (1664525 * Seed + 1013904223) >>> 0; return Seed / 2 ** 32; }; }
const Jittered = (Raw, Next) => Object.fromEntries(Object.entries(Raw).map(([Name, Value]) =>
  [Name, App.IK_JOINTS.includes(Name) ? Value + Math.round(Next() * 4) - 2 : Value]));

Test('16:10: an unreachable -90 wrist becomes the nearest pitch the IK reaches strictly, which Approve accepts after 200 jittered readings', () => {
  // The old search forced -55.9, 0.5 deg past the reach edge: Approve refused it after 1 tick of jitter.
  const { Servos, Motion, Snap, Validation } = Propose(S1610, { x: 4.2, y: 32.5, z: 1.0 }, -90);
  Assert.equal(Snap.Kind, 'pitch', JSON.stringify(Snap));
  Assert.equal(Snap.DistanceCm, 0);
  Assert.ok(Snap.Target.wristPitch >= -55.55 && Snap.Target.wristPitch <= -55.0, `wrist ${Snap.Target.wristPitch}`);
  Assert.ok(Converged(Snap.Target, Servos), 'selected strictly');
  Assert.deepEqual(Validation.issues, []);
  Assert.equal(Motion.wristPitchTarget, Snap.Target.wristPitch);
  Assert.match(Snap.Reason.Text, /^at wrist -90 deg the arm is 2\.\d+ cm too short to reach that point$/);
  const Next = Random(1610);
  for (let Run = 0; Run < 200; Run += 1) {
    const Later = Snapshot(Jittered(S1610, Next));
    Assert.deepEqual(App.Validate(Motion, 'up', Later, [], undefined).issues, [], `run ${Run}`);
    // Approve's own check: the proposal-time seed with the live servos' ranges.
    Assert.deepEqual(App.Validate(Motion, 'up', App.SeededServos(Later, Motion.snapSeed), [], undefined).issues, [], `seeded run ${Run}`);
  }
});

Test('a descent onto an [UNRELIABLE] object is refused, never snapped, whether or not it is the task object', () => {
  // 16:10: the target sat on the small black object; the task noun "thingon" matched nothing.
  const Black = Propose(S1610, { x: 4.2, y: 32.5, z: 1.0 }, -90, { Unreliable: [BlackObject] });
  Assert.equal(Black.Snap.Kind, 'refused');
  Assert.equal(Black.Snap.Code, 'unreliable');
  Assert.match(Black.Snap.Text, /\[UNRELIABLE\] small black object measurement.*hover at Z 2\.9 or higher/);
  Assert.equal(Black.Snap.Nearest, null);
  Assert.equal(Black.Validation.possible, false);
  // Listed once: the validator added the snap's text again, and the 16:10 replay's feedback gave it twice.
  Assert.deepEqual(Black.Validation.issues.filter(Issue => /UNRELIABLE/.test(Issue)), [Black.Snap.Text], Black.Validation.issues.join(' | '));
  Assert.equal(Black.Motion.target, Black.Motion.requestedTarget, 'The request is kept as asked');
  // 16:05: the same for the USB drive the model aimed at.
  const Usb = Propose(S1605, { x: 4.1, y: 32.3, z: -0.4 }, -90, { Unreliable: [UsbDrive] });
  Assert.equal(Usb.Snap.Code, 'unreliable');
  Assert.match(Usb.Snap.Text, /USB drive/);
  // A hover above it is snapped like any other target.
  const Hover = Propose(S1610, { x: 4.2, y: 32.5, z: 4.0 }, -90, { Unreliable: [BlackObject] });
  Assert.notEqual(Hover.Snap.Kind, 'refused', JSON.stringify(Hover.Snap));
  Assert.deepEqual(Hover.Validation.issues, []);
});

Test('in a grasp task a descent onto the task object\'s centre is refused "grasp", never snapped; the GRASP PLAN\'s own points are exact', () => {
  // The tip is the fixed jaw's inner pad (lib/GraspPlan.ts): at the centre the fixed finger is inside the object.
  const Open = Snapshot({ ...Home, gripper: 1922 });
  const Usb = { label: 'usb drive', x: 4.3, y: 32.9, z: -2.5, topZ: -0.4, heightCm: 2.1, sizeCm: [1.7, 1.8], depthMeasured: true, extrapolated: false,
    unstable: false, supportMeasured: true, supportZMeasured: -2.5, validFraction: 0.9, supportBranch: 'desk', refined: true };
  const Reach = App.TaskObjectReachability(Usb, { Servos: Open, ElbowMode: 'up' }, { Others: [] });
  const Plan = Reach.Plan;
  Assert.ok(Plan, Reach.Text);
  const State = { Label: 'usb drive', DetectedNow: true, Binding: { Object: Usb, Source: 'name', DistanceCm: null }, LastSeen: { At: 0, Object: Usb, Reachability: Reach },
    Reachability: Reach, Intent: 'grasp', Plan: { Reach, Source: Usb, At: 0, Key: App.GraspPlanKey(Open, 'up'), Note: null } };
  const Guard = App.BuildGraspGuard([Usb], State, false);
  const Snap = (Target, Wrist, Servos = Open) => {
    const Motion = App.ParseResponse(Reply({ target_cm: Target, wrist_pitch_degrees: Wrist }), TipOf(Servos), Servos).motion;
    return App.SnapAiTarget(App.SnapInput({ ...Motion, guard: Guard }, Servos, 'up', undefined, []));
  };
  for (const Wrist of [Plan.Pitch, -90]) {
    const Centre = Snap({ x: Usb.x, y: Usb.y, z: Plan.Grasp.z }, Wrist);
    Assert.equal(Centre.Kind, 'refused', `wrist ${Wrist}: ${JSON.stringify(Centre.Kind)}`);
    Assert.equal(Centre.Code, 'grasp');
    // Into the object at the plan's wrist; at -90, below its top at a wrist the plan never checked.
    Assert.match(Centre.Text, Wrist === -90 ? /checked only at wrist -52\.8 deg/ : /FIXED finger.*usb drive/);
    Assert.match(Centre.Text, /use the GRASP PLAN: hover point X 6\.9, Y 31\.7, Z 4\.6 \(wrist -49\.7\)/);
  }
  Assert.equal(Snap(Plan.Hover, Plan.HoverPitch).Kind, 'exact');
  const AtHover = structuredClone(Open);
  const HoverAngles = App.solveIkPreview({ ...App.modelPointFromDisplayPoint(Plan.Hover), wristPitch: Plan.HoverPitch }, 'up', Open).angles;
  for (const Joint of App.IK_JOINTS) Object.assign(AtHover[Joint], { presentRaw: App.rawFromAngle(AtHover[Joint], HoverAngles[Joint]), targetRaw: App.rawFromAngle(AtHover[Joint], HoverAngles[Joint]) });
  Assert.equal(Snap(Plan.Grasp, Plan.Pitch, AtHover).Kind, 'exact');
  // Without the guard the same centre descent is taken as it is (the rows above run without one).
  const Plain = App.ParseResponse(Reply({ target_cm: { x: Usb.x, y: Usb.y, z: Plan.Grasp.z }, wrist_pitch_degrees: Plan.Pitch }), TipOf(Open), Open).motion;
  Assert.equal(App.SnapAiTarget(App.SnapInput(Plain, Open, 'up', undefined, [])).Kind, 'exact');
});

Test('16:05: the -90 grasp the arm fell 2.6 cm short of snaps to the same point or one within 0.8 cm, never lower, with the real cause', () => {
  // The old feedback said "joint slider limits would be exceeded" and the straight-line hint "8.02 cm short".
  const { Servos, Snap, Validation } = Propose(S1605, { x: 4.1, y: 32.3, z: -0.4 }, -90);
  Assert.ok(Snap.Kind === 'pitch' || Snap.Kind === 'position', JSON.stringify(Snap));
  Assert.ok(Snap.DistanceCm <= 0.8, `${Snap.DistanceCm} cm`);
  Assert.ok(Display(Snap.Target).z >= -0.9, `Z ${Display(Snap.Target).z}`);
  Assert.ok(Snap.Target.wristPitch >= -135 && Snap.Target.wristPitch <= -45, `wrist ${Snap.Target.wristPitch}`);
  Assert.ok(Converged(Snap.Target, Servos));
  Assert.match(Snap.Reason.Text, /too short/);
  if (Snap.Kind === 'position') Assert.match(Snap.Reason.Text, /basket/);
  Assert.deepEqual(Validation.issues, []);
});

Test('the floor refuses a target under it instead of snapping it up; exactly on the floor is not refused', () => {
  const Below = Propose(Home, { x: -34, y: 32, z: -3.5 }, -45);
  Assert.equal(Below.Snap.Code, 'floor');
  Assert.equal(Below.Snap.Nearest, null);
  Assert.match(Below.Snap.Text, /below the table floor/);
  const On = Propose(Home, { x: -34, y: 32, z: App.AiTableFloorCm }, -45);
  Assert.ok(!(On.Snap.Kind === 'refused' && On.Snap.Code === 'floor'), JSON.stringify(On.Snap));
  Assert.ok(!On.Validation.issues.some(Issue => /Target Z -3\.3 cm is below/.test(Issue)), On.Validation.issues.join(' | '));
});

Test('a low target on the desk in front of the tray is refused by the floor: only the tray\'s open top is exempt, not the whole cart\'s footprint', () => {
  // The cart's bounds reach display Y 29.06, the tray's front rim is at Y 23.3: (-15, 28, -5) was snapped and approvable.
  const Desk = Propose(Home, { x: -15, y: 28, z: -5 }, -90);
  Assert.equal(Desk.Snap.Code, 'floor', JSON.stringify(Desk.Snap));
  Assert.equal(Desk.Validation.possible, false);
  Assert.ok(Desk.Validation.issues.some(Issue => /Target Z -5\.0 cm is below the table floor/.test(Issue)), Desk.Validation.issues.join(' | '));
  // The model is told where the exemption ends.
  Assert.match(App.AiPrompt('', true, false, 'Behind'), /Only over the basket's open top \(Y less than 23\.3, inside its front rim\) may the tip go lower/);
});

Test('a flatter wrist never puts the gripper into the desk: every snapped low target keeps its lowest point above the floor', () => {
  let Snapped = 0;
  for (const [Raw, Target] of [[Home, { x: -30, y: 36, z: -2.0 }], [S1610, { x: 4, y: 38, z: -2.0 }], [Home, { x: -34, y: 32, z: -3.3 }], [S1610, { x: -5, y: 33, z: -2.5 }]]) {
    const { Servos, Snap } = Propose(Raw, Target, -90);
    if (Snap.Kind === 'refused') continue;
    Snapped += 1;
    const Lowest = App.GripperLowestZ(App.solveIkPreview(Snap.Target, 'up', Servos).angles);
    Assert.ok(Lowest >= App.AiTableFloorCm - 1e-6, `${JSON.stringify(Target)}: gripper at Z ${Lowest}`);
  }
  Assert.ok(Snapped >= 2);
});

Test('near the basket rim a snap stays on the side of the rim it was asked for, with clearance to spare', () => {
  // Across the top tray's front rim (display Y 23.3): a low target just in front of it is over the desk.
  const RimZ = App.BasketBounds[1].z;
  Assert.ok(Math.abs(RimZ - App.TrayBounds[1].z) < 1e-4, 'The cart\'s top is the tray\'s rim');
  let Moved = 0;
  for (const Z of [0, -1]) for (let Y = 22.8; Y <= 24.201; Y += 0.2) {
    const Point = { x: 4, y: Number(Y.toFixed(1)), z: Z };
    const { Motion, Snap } = Propose(S1610, Point, -90);
    if (Snap.Kind === 'refused') continue;
    const Request = Motion.requestedTarget;
    Assert.equal(App.OverBasket(Snap.Target), App.OverBasket(Request), `${JSON.stringify(Point)} crossed the rim's footprint`);
    if (App.OverBasket(Request)) Assert.equal(Snap.Target.z >= RimZ, Request.z >= RimZ, `${JSON.stringify(Point)} crossed the rim height`);
    Assert.ok(Snap.Target.z >= Request.z - App.SnapRules.MaxDropCm - 1e-6, `${JSON.stringify(Point)} went lower`);
    if (Snap.Kind !== 'position') continue;
    Moved += 1;
    // The nearest point lies on a contact boundary: without the margin the bridge's check of the pose
    // rounded to encoder steps refused 13 of 33 grid snaps (offline replay of PlanCartesianMotion).
    const Pose = App.solveIkPreview(Snap.Target, 'up', Snapshot(S1610)).angles;
    Assert.equal(App.BasketPoseReason(Pose, App.SnapRules.CollisionMarginCm), null, `${JSON.stringify(Point)} sits on the contact boundary`);
  }
  Assert.ok(Moved >= 4, `${Moved} position snaps`);
});

Test('a target more than 3 cm from anything reachable is refused with the nearest possible target and why it was not used', () => {
  const Far = Propose(S1610, { x: 4, y: 44, z: 0.5 }, -90);
  Assert.equal(Far.Snap.Kind, 'refused');
  Assert.equal(Far.Snap.Code, 'too-far');
  Assert.ok(Far.Snap.Nearest.DistanceCm > 3 && Far.Snap.Nearest.DistanceCm < 5, `${Far.Snap.Nearest.DistanceCm} cm`);
  Assert.match(Far.Snap.Nearest.Why, /more than 3 cm away/);
  Assert.ok(Converged(Far.Snap.Nearest.Target, Far.Servos), 'The nearest target is one the model can ask for');
  Assert.equal(Far.Validation.possible, false);
  Assert.ok(Far.Validation.issues.includes(Far.Snap.Text));
  // The wrist a preference: refused only for the distance, or snapped with whatever wrist reaches it (2026-10-03).
  const Side = Propose(S1610, { x: -12, y: 44, z: 0.5 }, -90);
  if (Side.Snap.Kind === 'refused') Assert.match(Side.Snap.Nearest?.Why ?? Side.Snap.Text, /more than 3 cm|found no reachable target/);
  else Assert.ok(Side.Snap.DistanceCm <= 3 && Converged(Side.Snap.Target, Side.Servos) && Side.Validation.issues.length === 0, `wrist ${Side.Snap.Target.wristPitch}`);
});

// The wrist is a preference (SnapRules.WristOnlyMaxChangeDeg, 2026-10-03: "it says it is too far when it definitely can").
// Capped at 45 deg the app moved the tip 0.5 cm off (4, 36, 0.5) with the wrist at -45, and refused (4, 39.6, 0.5).
const SoftWrist = Rules => ({ ...App.SnapRules, ...Rules });
Test('a point the arm holds only with a wrist 48 deg flatter than asked: the wrist turns that far, the tip stays; with no wrist it moves the tip a little', () => {
  const { Servos, Snap, Validation, Motion } = Propose(S1610, { x: 4, y: 36, z: 0.5 }, -90);
  Assert.equal(Snap.Kind, 'pitch', JSON.stringify(Snap));
  Assert.equal(Snap.DistanceCm, 0);
  Assert.ok(Snap.PitchChangeDeg > 45 && Snap.Target.wristPitch > -45, `wrist ${Snap.Target.wristPitch}`);
  Assert.ok(Converged(Snap.Target, Servos));
  Assert.deepEqual(Validation.issues, []);
  // The wrist the nearest that works: a degree steeper and the arm does not hold the point.
  Assert.equal(App.SnapAiTarget(App.SnapInput(Motion, Servos, 'up', undefined, []), SoftWrist({ WristOnlyMaxChangeDeg: Snap.PitchChangeDeg - 1 })).Kind === 'pitch', false);
  // As the app was (the wrist alone at most 45 deg): the tip moved instead.
  const Capped = App.SnapAiTarget(App.SnapInput(Motion, Servos, 'up', undefined, []), SoftWrist({ WristOnlyMaxChangeDeg: 45 }));
  Assert.equal(Capped.Kind, 'position', JSON.stringify(Capped));
  // A point no wrist holds (at the edge of reach): the tip moved under a cm, with whichever wrist reaches nearest, never refused.
  const Edge = Propose(S1610, { x: 4, y: 39.6, z: 0.5 }, -90);
  Assert.equal(Edge.Snap.Kind, 'position', JSON.stringify(Edge.Snap));
  Assert.ok(Edge.Snap.DistanceCm > 0.1 && Edge.Snap.DistanceCm < 0.8, `${Edge.Snap.DistanceCm} cm`);
  Assert.ok(Edge.Snap.PitchChangeDeg > 45, `wrist ${Edge.Snap.Target.wristPitch}`);
  Assert.ok(Converged(Edge.Snap.Target, Edge.Servos));
  Assert.match(Edge.Snap.Reason.Text, /no wrist angle reaches it/);
  Assert.deepEqual(Edge.Validation.issues, []);
  Assert.equal(App.SnapAiTarget(App.SnapInput(Edge.Motion, Edge.Servos, 'up', undefined, []), SoftWrist({ WristOnlyMaxChangeDeg: 45 })).Kind, 'refused', 'capped at 45 deg it was refused');
});

Test('a target_cm without a wrist, or "auto", asks for the preferred -60 deg and gets the nearest pitch to it the arm holds there; a step without one keeps the wrist', () => {
  // The preferred pitch (GraspRules.PreferredPitchDeg, 2026-10-03: steep, so the camera still sees the object under the gripper).
  Assert.equal(App.GraspRules.PreferredPitchDeg, -60);
  const Servos = Snapshot(S1610), Current = TipOf(Servos);
  const Parse = Motion => App.ParseResponse(Reply(Motion), Current, Servos).motion;
  const Snapped = Motion => App.SnapAiTarget(App.SnapInput(Motion, Servos, 'up', undefined, []));
  for (const Wrist of [undefined, 'auto']) {
    const With = Target => ({ target_cm: Target, ...(Wrist ? { wrist_pitch_degrees: Wrist } : {}) });
    // Where -60 holds the point: -60, exactly.
    const There = Parse(With({ x: 4, y: 30, z: 3 }));
    Assert.equal(There.requestedWristPitchTarget, -60, `${Wrist}`);
    const Exact = Snapped(There);
    Assert.deepEqual([Exact.Kind, Exact.Target.wristPitch], ['exact', -60], `${Wrist}: ${JSON.stringify(Exact)}`);
    // Where it does not (the arm reaches (4, 36, 0.5) only flatter than -50): the nearest pitch to -60 that holds it, the tip kept.
    const Far = Parse(With({ x: 4, y: 36, z: 0.5 }));
    Assert.equal(Far.requestedWristPitchTarget, -60);
    const Snap = Snapped(Far);
    Assert.deepEqual([Snap.Kind, Snap.DistanceCm], ['pitch', 0], `${Wrist}: ${JSON.stringify(Snap)}`);
    Assert.ok(Snap.Target.wristPitch > -60 && Converged(Snap.Target, Servos), `wrist ${Snap.Target.wristPitch}`);
    // None nearer -60 works.
    Assert.notEqual(App.SnapAiTarget(App.SnapInput(Far, Servos, 'up', undefined, []), SoftWrist({ WristOnlyMaxChangeDeg: Snap.PitchChangeDeg - 1 })).Kind, 'pitch', `${Wrist}`);
  }
  // A relative step without a wrist keeps it (the lift after the close: the jaws hold the object as they closed on it), and so
  // does a target_cm restating where the tip is (within a cm: with a gripper action, say); "auto" asks for -60 there too.
  Assert.equal(Parse({ move_delta_cm: { up: 2 } }).requestedWristPitchTarget, Current.wristPitch);
  const Display = App.mirrorDisplayPoint(Current);
  Assert.equal(Parse({ target_cm: { x: Display.x + 0.5, y: Display.y, z: Display.z } }).requestedWristPitchTarget, Current.wristPitch);
  Assert.equal(Parse({ move_delta_cm: { up: 2 }, wrist_pitch_degrees: 'auto' }).requestedWristPitchTarget, -60);
  // A given pitch is the pitch asked for.
  Assert.equal(Parse({ target_cm: { x: 4, y: 30, z: 3 }, wrist_pitch_degrees: -35 }).requestedWristPitchTarget, -35);
  // The prompt tells the model.
  Assert.match(App.AiPrompt('', true, false, 'Behind'), /WRIST RULE: wrist_pitch_degrees is the wrist pitch you would like, in degrees, not a hard requirement: .* With target_cm, leave it out or write "wrist_pitch_degrees":"auto" to get the app's preferred -60 deg \(steep, so the camera still sees what is under the gripper\), or the nearest pitch to it the arm can hold there; a move_delta_cm step without it keeps the wrist as it is\./);
});

Test('a joint limit that holds the tip off the point at every wrist: the tip moves and the wrist stays as asked (TestAiTaskSafetyUi\'s limit-change re-snap)', () => {
  // TestAiTaskSafetyUi (after the grasp rework, 2026-10-03): at home a move_delta_cm up 2 without a wrist (so the wrist
  // -6.6 is kept) is exact; before Approve the bridge narrows the shoulder pan's range to end 0.32 deg short of the
  // pan, so no wrist reaches the point. The re-snap turned the wrist 30 deg to -36.6 (the IK misses the point by the
  // same 0.11 cm at every pitch from -66.6 to -6.6) and asked for approval again instead of sending a 0.29 cm move.
  const Servos = Snapshot(Home), Current = TipOf(Servos);
  const Limits = App.GetIkAngleLimits(Servos), Pan = App.currentAngles(Servos).shoulder_pan;
  const Motion = App.ParseResponse(Reply({ move_delta_cm: { left: 0, farther: 0, up: 2 } }), Current, Servos).motion;
  Assert.equal(Motion.requestedTarget.wristPitch, Current.wristPitch, 'a step without a wrist keeps it');
  const Proposed = App.SnapAiTarget(App.SnapInput(Motion, Servos, 'up', Limits, []));
  Assert.equal(Proposed.Kind, 'exact', JSON.stringify(Proposed));
  const Proposal = App.ApplySnap({ ...Motion, snapSeed: App.ServoSeed(Servos), unreliable: [] }, Proposed);
  for (const [Change, Sent] of [[0.32, true], [1, false]]) {
    const Narrow = { ...Limits, shoulder_pan: [Limits.shoulder_pan[0], Pan - Change] };
    const Snap = App.SnapAiTarget(App.SnapInput(Proposal, Servos, 'up', Narrow, []));
    Assert.equal(Snap.Kind, 'position', `${Change}: ${JSON.stringify(Snap)}`);
    Assert.match(Snap.Reason.Text, /the shoulder pan would need -?\d+\.\d deg, its range is -?\d+\.\d\.\.-?\d+\.\d deg.*; no wrist angle reaches it/);
    // The wrist as asked (within the search's fine steps), the tip moved only as far as the pan's limit makes it.
    Assert.ok(Snap.PitchChangeDeg < 0.5, `${Change}: wrist ${Snap.Target.wristPitch.toFixed(2)}, asked ${Current.wristPitch.toFixed(2)}`);
    const Resnapped = App.ApplySnap({ ...Proposal, snapSeed: App.ServoSeed(Servos) }, Snap);
    Assert.deepEqual(App.Validate(Resnapped, 'up', Servos, [], Narrow).issues, []);
    // App.tsx's Approve: a re-snap within 0.3 cm and 1 deg of the proposal is sent, else shown for another approval.
    const ShiftCm = Distance(Resnapped.target, Proposal.target), TurnDeg = Math.abs(Resnapped.target.wristPitch - Proposal.target.wristPitch);
    Assert.equal(ShiftCm <= 0.3 && TurnDeg <= 1, Sent, `${Change}: ${ShiftCm.toFixed(3)} cm, ${TurnDeg.toFixed(2)} deg`);
    Assert.ok(ShiftCm < 1, `${Change}: ${ShiftCm.toFixed(3)} cm`);
    Assert.match(App.SnapNote(Resnapped), /\(0\.\d\d cm away; wrist unchanged\)\. Why: at wrist -6\.6 deg the shoulder pan would need/);
  }
});

Test('the snap is deterministic, and a jittered live reading with the proposal-time seed gives the same result', () => {
  const Strip = Snap => ({ ...Snap, Ms: null });
  const First = Propose(S1610, { x: 4, y: 36, z: 0.5 }, -90), Second = Propose(S1610, { x: 4, y: 36, z: 0.5 }, -90);
  Assert.deepEqual(Strip(First.Snap), Strip(Second.Snap));
  const Next = Random(7);
  const Live = App.SeededServos(Snapshot(Jittered(S1610, Next)), First.Motion.snapSeed);
  const Again = App.SnapAiTarget(App.SnapInput(First.Motion, Live, 'up', undefined, []));
  Assert.deepEqual(Strip(Again), Strip(First.Snap));
});

// The manual GUI's searches (FindNearestValidTarget, FindReachableWristTarget)
// without the AI options, recorded before the snap was added: unchanged.
// v7 is the v6 geometry with the tray's front wall moved; these rows hold on both, and on
// v8 (v7's geometry, contact judged on the CAD boxes alone: no tracking envelope).
const GuiFixtures = { Collision: 'robot-body-cad-exact-v8', Cad: 4, Rows: [
  ['pitch', { x: 6.699999999999999, y: 30, z: -4.732434233134427, wristPitch: -90 }, { x: 6.699999999999999, y: 30, z: -4.732434233134427, wristPitch: -78.848876953125 }],
  ['pitch', { x: -17.3, y: 36, z: -4.232434233134427, wristPitch: -90 }, { x: -17.3, y: 36, z: -4.232434233134427, wristPitch: -42.515625 }],
  ['pitch', { x: 20.7, y: 32, z: -6.232434233134427, wristPitch: -60 }, { x: 20.7, y: 32, z: -6.232434233134427, wristPitch: -51.623623092259166 }],
  ['position', { x: 80, y: 20, z: 5, wristPitch: -6.591434689133739 }, { x: 42.75309889623499, y: 10.688773024343263, z: 5.202771038165852, wristPitch: -0.5837789997172773 }],
  ['position', { x: 4, y: 44, z: 0.5, wristPitch: -90 }, { x: 3.9638600280495306, y: 43.59940334824956, z: 0.5485420033537414, wristPitch: -7.209868005166378 }],
] };
Test('the manual GUI snap is unchanged without the AI options', { skip: App.BasketCollisionVersion !== GuiFixtures.Collision || App.GetCadAlignment().Version !== GuiFixtures.Cad
  ? 'the collision model or CAD alignment changed since the fixtures were recorded' : false }, () => {
  const Context = { Servos: structuredClone(App.Servos), ElbowMode: 'up' };
  for (const [Kind, Requested, Expected] of GuiFixtures.Rows) {
    const Result = Kind === 'pitch' ? App.FindReachableWristTarget(Requested, Context)
      : App.FindNearestValidTarget(Requested, App.HOME_MODEL_TARGET_CM, Context, true);
    Assert.deepEqual(Result, Expected, `${Kind} ${JSON.stringify(Requested)}`);
  }
  // With options it is the bounded search: nothing beyond MaxCm.
  const Bounded = App.FindNearestValidTarget(GuiFixtures.Rows[4][1], App.HOME_MODEL_TARGET_CM, Context, true, { MaxCm: 3, PitchWindowDeg: 45, MaxEvaluations: 20000 });
  Assert.ok(Bounded === null || Distance(Bounded, GuiFixtures.Rows[4][1]) <= 3);
});

Test('the reason names the real cause: reach, or the joint and its range, never an untouched slider limit', () => {
  for (const [Raw, Target] of [[S1605, { x: 4.1, y: 32.3, z: -0.4 }], [S1610, { x: 4.2, y: 32.5, z: 1.0 }]]) {
    const { Snap, Motion, Servos } = Propose(Raw, Target, -90);
    const Text = App.BuildAiFeasibilityFeedbackPrompt([Snap.Reason.Text], Motion, TipOf(Servos), { Servos, ElbowMode: 'up' }, 1, 4);
    // The old line: "One or more configured joint slider limits would be exceeded", with every joint far from its stop.
    Assert.doesNotMatch(Text + Snap.Reason.Text, /limits would be exceeded|joint limit/);
  }
  // A pose blocked by a joint limit (TestAiTaskSafety's pose, with the elbow's range narrowed).
  const Servos = structuredClone(App.Servos);
  const Pose = { shoulder_pan: 0.34323065528329266, shoulder_lift: -87.45806454570632, elbow_flex: 79.94271769861842, wrist_flex: -27.1169118284174 };
  for (const [Name, Value] of Object.entries(Pose)) Servos[Name].presentRaw = Servos[Name].targetRaw = App.rawFromAngle(Servos[Name], Value);
  const Raw = Object.fromEntries(Object.entries(Servos).map(([Name, Servo]) => [Name, Servo.presentRaw]));
  const Goal = Snapshot({ ...Raw, shoulder_pan: Raw.shoulder_pan - 34 });
  const Elbow = App.currentAngles(Servos).elbow_flex;
  const Limits = { ...App.GetIkAngleLimits(Servos), elbow_flex: [Elbow + 15, Elbow + 25] };
  const { Snap } = Propose(Raw, Display(TipOf(Goal)), TipOf(Goal).wristPitch, { Elbow: 'down', Limits });
  Assert.equal(Snap.Reason.Code, 'joint-limit', JSON.stringify(Snap));
  Assert.match(Snap.Reason.Text, /the elbow flex would need -?\d+\.\d deg, its range is -?\d+\.\d\.\.-?\d+\.\d deg/);
});

Test('the model and the human are told what was asked, where the arm goes instead and why', () => {
  const { Motion, Snap } = Propose(S1610, { x: 4.2, y: 32.5, z: 1.0 }, -90);
  const Told = App.TaskMotionSnap(Motion);
  Assert.deepEqual([Told.Requested.x, Told.Requested.y, Told.Requested.z].map(Value => Number(Value.toFixed(1))), [4.2, 32.5, 1]);
  const Fact = App.DescribeMotionFact({ Step: 2, StartTip: { x: -4.8, y: 28.2, z: 7.4 }, EndTip: { x: 4.2, y: 32.5, z: 1.1 }, Target: Told.Target, Servos: null, Snap: Told });
  Assert.match(Fact, new RegExp(String.raw`You asked for X 4\.2, Y 32\.5, Z 1\.0, wrist -90\.0; that is not reachable \(at wrist -90 deg the arm is 2\.\d+ cm too short to reach that point\), so the app sent the nearest possible target X 4\.2, Y 32\.5, Z 1\.0, wrist ${Snap.Target.wristPitch.toFixed(1)} \(0\.0 cm from your target\)\.`));
  const CheckIn = App.BuildTaskCheckInPrompt({ task: 'Grab the black thing', step: 3 }, ['- Motion 2: tip to X 4.2' + App.SnapMotionNote(Told)], Fact, true);
  Assert.match(CheckIn, /do not ask for the same unreachable point again; change the approach \(a higher hover, another side\) or reply ask/);
  Assert.match(CheckIn, /- Motion 2: tip to X 4\.2 \(you asked for X 4\.2, Y 32\.5, Z 1\.0, wrist -90\.0: snapped 0\.0 cm and 3\d\.\d deg of wrist, at wrist -90 deg the arm is/);
  Assert.doesNotMatch(App.BuildTaskCheckInPrompt({ task: 'x', step: 3 }, [], 'fact', false), /unreachable point again/);
  // A snapped target that lands where the arm already is: a no-op that says why.
  Assert.match(App.SnapNoOpReason(Told), /^your target X 4\.2, Y 32\.5, Z 1\.0, wrist -90\.0 is not reachable \(at wrist -90 deg .*\); its nearest reachable point \(X 4\.2, Y 32\.5, Z 1\.0, wrist -5\d\.\d, 0\.0 cm from it\) is where the arm already is, so the arm cannot get closer to it this way$/);
  // The approval card, in the tiles' frame (X negated): asked vs. sent and why.
  const Note = App.SnapNote(Motion);
  Assert.match(Note, new RegExp(String.raw`^Asked for X -4\.2 Y 32\.5 Z 1\.0, wrist -90\.0 -> will move to X -4\.2 Y 32\.5 Z 1\.0, wrist ${Snap.Target.wristPitch.toFixed(1)} \(same point; wrist 3\d\.\d deg flatter\)\. Why: at wrist -90 deg the arm is 2\.\d+ cm too short to reach that point\.$`));
  const Position = Propose(S1610, { x: 4, y: 39.6, z: 0.5 }, -90);
  Assert.match(App.SnapNote(Position.Motion), /\(0\.\d\d cm away; wrist \d+\.\d deg flatter\)/);
  const Here = TipOf(Snapshot(Home)), Up = Display({ ...Here, y: Here.y + 2 });
  const Exact = Propose(Home, { x: Up.x, y: Up.y, z: Up.z }, Here.wristPitch);
  Assert.equal(Exact.Snap.Kind, 'exact', JSON.stringify(Exact.Snap));
  Assert.equal(App.SnapNote(Exact.Motion), null, 'An exact target has no note');
  // The record keeps the request and the snap.
  const Record = App.MotionRecord(Motion);
  Assert.equal(Record.requested_wrist_pitch_degrees, -90);
  Assert.deepEqual(Object.values(Record.requested_target_cm).map(Value => Number(Value.toFixed(1))), [4.2, 32.5, 1]);
  Assert.equal(Record.snap.kind, 'pitch');
  Assert.equal(Record.wrist_pitch_degrees, Snap.Target.wristPitch);
  // A refused one: the feedback gives the nearest possible target instead of the straight-line hint.
  const Far = Propose(S1610, { x: 4, y: 44, z: 0.5 }, -90);
  const Feedback = App.BuildAiFeasibilityFeedbackPrompt(Far.Validation.issues, Far.Motion, TipOf(Far.Servos), { Servos: Far.Servos, ElbowMode: 'up' }, 1, 4);
  Assert.match(Feedback, /Nearest possible target: X -?[\d.]+ cm, Y -?[\d.]+ cm, Z -?[\d.]+ cm, wrist -?[\d.]+ deg \([34]\.\d+ cm from yours\)\. The app did not use it because it is more than 3 cm away/);
  Assert.doesNotMatch(Feedback, /straight line from the tip/);
  Assert.equal(App.MotionRecord(Far.Motion).snap.code, 'too-far');
  // Refusals by the bridge and approvals that were not sent.
  Assert.match(App.DescribeBridgeRefusal(3, 'IK target is unreachable at this wrist angle.', Told),
    /^Your last approved motion \(motion 3\) was NOT executed: the bridge refused it: IK target is unreachable at this wrist angle\. The arm did not move\. The target sent was the app's nearest possible point X 4\.2, Y 32\.5, Z 1\.0, wrist -5\d\.\d \(you asked for X 4\.2, Y 32\.5, Z 1\.0, wrist -90\.0\)\./);
  Assert.match(App.NotSentFeedbackPrompt(['Wrist pitch -55.9 deg is not reachable there.']), /^Your last proposal was not sent: Wrist pitch -55\.9 deg is not reachable there\. The arm did not move\./);
  // The system prompt states the rule with the caps it uses.
  Assert.match(App.AiPrompt('', true, false, 'Behind'), /REACH RULE: if your target cannot be reached exactly, the app moves it to the nearest reachable target by itself and shows the human both\. It first changes only the wrist angle \(to the nearest pitch that works at your point, however far\), otherwise it moves the tip by at most 3 cm, with the wrist that reaches nearest \(never lower than you asked, never below the floor, never across the basket rim\)/);
});

// How busy the PC is right now, as the cost of one IK solve (the median of five batches): the time bounds below were set
// on this PC idle (about 2.5 ms a solve, 2026-10-07) and scale with it, so a loaded PC that slows every step alike no
// longer fails them (audit L5: the wrist stage took 1232-2002 ms against 1 s while other suites ran).
function LoadScale(Solve, IdleMsPerSolve = 2.5) {
  const Batches = Array.from({ length: 5 }, () => {
    const Started = performance.now();
    for (let Index = 0; Index < 40; Index += 1) Solve();
    return (performance.now() - Started) / 40;
  }).sort((A, B) => A - B);
  return Math.max(1, Batches[2] / IdleMsPerSolve);
}

Test('the snap costs well under the worker guard: the wrist stage under 1 s, the position stage under 6 s and its evaluation budget', () => {
  const Point = { ...App.modelPointFromDisplayPoint({ x: 4, y: 32, z: 1 }), wristPitch: -60 };
  const Scale = LoadScale(() => App.solveIkPreview(Point, 'up', App.Servos));
  const Pitch = Propose(S1605, { x: 4.1, y: 32.3, z: -0.4 }, -90);
  Assert.ok(Pitch.Snap.Ms.PitchMs <= 1000 * Scale, `wrist stage ${Pitch.Snap.Ms.PitchMs.toFixed(0)} ms (load x${Scale.toFixed(2)})`);
  const Position = Propose(S1610, { x: 4, y: 39.6, z: 0.5 }, -90);
  Assert.equal(Position.Snap.Kind, 'position');
  Assert.ok(Position.Snap.Ms.PitchMs <= 1000 * Scale, `wrist stage ${Position.Snap.Ms.PitchMs.toFixed(0)} ms (load x${Scale.toFixed(2)})`);
  Assert.ok(Position.Snap.Ms.PositionMs <= 6000 * Scale, `position stage ${Position.Snap.Ms.PositionMs.toFixed(0)} ms (load x${Scale.toFixed(2)})`);
  Assert.ok(Position.Snap.Evaluations <= App.SnapRules.MaxEvaluations);
  const Far = Propose(S1610, { x: 4, y: 44, z: 0.5 }, -90);
  Assert.ok(Far.Snap.Evaluations <= App.SnapRules.MaxEvaluations, `${Far.Snap.Evaluations} evaluations`);
  Assert.ok(Far.Ms <= 10000 * Scale, `${Far.Ms.toFixed(0)} ms for a refusal (load x${Scale.toFixed(2)})`);
});
