import { build as Build } from 'esbuild';
import { readFileSync as ReadFile } from 'node:fs';
import { createRequire as CreateRequire } from 'node:module';
import { test as Test } from 'node:test';
import Assert from 'node:assert/strict';
import Path from 'node:path';
import { fileURLToPath as FilePath } from 'node:url';
import { PinnedHome } from './UiTestLib.mjs';

// The AI task loop's pure rules: what reaches the model after a /locate reply,
// and when a task must stop. The UI side is TestAiTaskSafetyUi.mjs.
const Root = Path.dirname(FilePath(import.meta.url));
const Bundle = await Build({ stdin: { contents: `export * from './AiMeasurements'; export * from './AiTaskLoop'; export * from './robotModel'; export * from './AiChain';
  export { CadSolve, CadSolveCandidates } from './CadKinematics';`,
  resolveDir: Path.join(Root, 'src/lib'), loader: 'ts' }, bundle: true, platform: 'node', format: 'cjs', write: false, plugins: [PinnedHome] });
const Module = { exports: {} };
new Function('require', 'module', 'exports', Bundle.outputFiles[0].text)(CreateRequire(import.meta.url), Module, Module.exports);
const Api = Module.exports;
// App.tsx's own parser and validator, for the rules that live there. Its
// solveIkPreview is counted, so a search's cost is measured in IK solves,
// which do not depend on how busy this machine is.
const CountSolves = { name: 'count-solves', setup(Builder) {
  Builder.onLoad({ filter: /robotModel\.ts$/ }, Args => {
    const Source = ReadFile(Args.path, 'utf8');
    if (!Source.includes('export function solveIkPreview(')) throw new Error('robotModel.ts no longer exports solveIkPreview');
    return { loader: 'ts', contents: Source.replace('export function solveIkPreview(', 'function CountedSolveIkPreview(')
      + '\nexport const SolveCounter = { Count: 0 };\nexport function solveIkPreview(...Args: Parameters<typeof CountedSolveIkPreview>) { SolveCounter.Count += 1; return CountedSolveIkPreview(...Args); }\n' };
  });
} };
const AppBundle = await Build({ stdin: { contents: ReadFile(Path.join(Root, 'src/App.tsx'), 'utf8')
  + '\nexport { parseAiResponse as ParseResponse, validateAiMotionProposal as Validate, HOME_MODEL_TARGET_CM as HomeTarget, INITIAL_SERVOS as Servos,'
  + ' applyAiSnap as ApplySnap, aiSnapInput as SnapInput, buildAiSystemPrompt as AiPrompt, chainLegs as ChainLegs, chainCardSteps as ChainCardSteps,'
  + ' ikCommand as IkCommand };\nexport { SolveCounter } from "./lib/robotModel";'
  + '\nexport { SnapAiTarget } from "./lib/AiTargetSnap";\nexport { GripperSamples, GraspRules } from "./lib/GraspPlan";',
  resolveDir: Path.join(Root, 'src'), loader: 'tsx' }, bundle: true, platform: 'node', format: 'cjs', write: false, loader: { '.css': 'empty' }, plugins: [CountSolves, PinnedHome] });
globalThis.window ??= { location: { hostname: 'localhost' } };
const AppModule = { exports: {} };
new Function('require', 'module', 'exports', AppBundle.outputFiles[0].text)(CreateRequire(import.meta.url), AppModule, AppModule.exports);
const App = AppModule.exports;

const Check = (Fields = {}) => ({ ok: true, angle_deg: 0.12, offset_cm: 0.08, reason: '', reference_fitted_at: '2026-09-28T21:10:00', seq: 42, ...Fields });
const Eraser = { label: 'eraser', x: 16.1, y: 35.8, z: -2.3, top_z: 5.2, height_cm: 7.5, size_cm: [5, 2], method: 'depth', extrapolated: true,
  support_measured: false, support_z_measured: null, valid_fraction: 0.9, support_branch: 'desk' };
const Cup = { label: 'cup', x: 0.2, y: 40.8, z: -2.3, top_z: 6.7, height_cm: 9, size_cm: [7, 7], method: 'depth', support_measured: true,
  support_z_measured: -2.4, valid_fraction: 0.95, support_branch: 'desk' };
const Ok = Api.ReadLocateResponse({ ok: true, objects: [Eraser, Cup], rms_cm: 0.22, camera_check: Check(),
  withheld: [{ label: 'white box', reason: '553 cm away' }, { label: 'robot arm', reason: "the robot's own part" }] });

// Recorded arm states (present_raw from the model requests): 2026-09-29 07:58,
// 16:05 and 16:10, when forced or unreachable wrist pitches were refused.
const S0758 = { shoulder_pan: 3486, shoulder_lift: 3167, elbow_flex: 2688, wrist_flex: 862, wrist_roll: 1522, gripper: 699 };
const S1605 = { shoulder_pan: 3471, shoulder_lift: 3168, elbow_flex: 2686, wrist_flex: 855, wrist_roll: 1522, gripper: 699 };
const S1610 = { shoulder_pan: 3244, shoulder_lift: 771, elbow_flex: 851, wrist_flex: 1742, wrist_roll: 1522, gripper: 699 };
const Snapshot = Raw => {
  const Servos = structuredClone(App.Servos);
  for (const [Name, Value] of Object.entries(Raw)) Object.assign(Servos[Name], { presentRaw: Value, targetRaw: Value, guiAngleRaw: null });
  return Servos;
};
const TipOf = Servos => { const Angles = Api.currentAngles(Servos); return { ...Api.forwardKinematics(Angles).at(-1), wristPitch: Api.wristPitchFromAngles(Angles) }; };
const Reply = Motion => JSON.stringify({ response_to_user: 'x', task_status: 'continue', motion_proposal: Motion });
// A reply as the app turns it into a proposal: parsed, then its target snapped
// (lib/AiTargetSnap.ts, which the app runs in its worker) if it must be.
const Forced = (Servos, Motion, Elbow = 'up') => {
  const Parsed = App.ParseResponse(Reply(Motion), TipOf(Servos), Servos).motion;
  return App.ApplySnap(Parsed, App.SnapAiTarget(App.SnapInput(Parsed, Servos, Elbow, undefined, [])));
};
// Selected strictly: the IK converges there (0.02 cm, 0.08 deg), inside the 0.1 cm / 0.25 deg acceptance test.
const Converged = (Target, Servos, Elbow = 'up') => {
  const Preview = Api.solveIkPreview(Target, Elbow, Servos);
  return Preview.reachable && Preview.errorCm <= 0.02 && Math.abs(Preview.effectiveWristPitch - Target.wristPitch) <= 0.08;
};

Test('a /locate reply keeps every existing field and reads the camera check, withheld items and support fields', () => {
  Assert.equal(Ok.objects.length, 2);
  Assert.deepEqual(Ok.cameraCheck, { Ok: true, AngleDeg: 0.12, OffsetCm: 0.08, Reason: '', ReferenceFittedAt: '2026-09-28T21:10:00', Seq: 42 });
  Assert.deepEqual(Ok.withheld[0], { Label: 'white box', Reason: '553 cm away' });
  Assert.equal(Ok.objects[0].supportMeasured, false);
  Assert.equal(Ok.objects[1].supportZMeasured, -2.4);
  Assert.equal(Ok.objects[1].supportBranch, 'desk');
  Assert.equal(Ok.objects[0].topZ, 5.2);
  const Moved = Api.ReadLocateResponse({ ok: false, error_code: 'camera_moved', error: 'The camera or cart moved.', camera_check: Check({ ok: false, angle_deg: 5.2, offset_cm: 1.3 }) });
  Assert.equal(Moved.errorCode, 'camera_moved');
  Assert.equal(Moved.cameraCheck.Ok, false);
  Assert.equal(Api.ReadLocateResponse({ ok: false, error: 'Ollama is down' }).errorCode, null);
  Assert.equal(Api.ReadLocateResponse({ ok: true, calibrated: false }), null, 'A reply that is not a /locate result reads as no answer');
});

Test('the camera gate stops every request on a moved camera, with the numbers and calibration time', () => {
  const Moved = Api.ReadLocateResponse({ ok: false, error_code: 'camera_moved', error: 'The camera or cart moved since calibration.',
    camera_check: Check({ ok: false, angle_deg: 5.24, offset_cm: 1.31 }) });
  for (const Needs of [true, false]) {
    const Gate = Api.CameraGateFailure(Moved, Needs);
    Assert.equal(Gate.Code, 'camera_moved');
    Assert.match(Gate.Title, /^Camera moved 5\.2 deg \/ 1\.3 cm since calibration at 21:10: lock the head and run Automatic calibration\.$/);
    Assert.match(Gate.Detail, /camera or cart moved/);
  }
  // The desk offset is signed (nearer or farther); the user is told its size.
  const Nearer = Api.ReadLocateResponse({ ok: false, error_code: 'camera_moved', error: 'The camera or cart moved.', camera_check: Check({ ok: false, angle_deg: 0.2, offset_cm: -1.31 }) });
  Assert.match(Api.CameraGateFailure(Nearer, true).Title, /^Camera moved 0\.2 deg \/ 1\.3 cm since calibration at 21:10/);
});

Test('a block by the tape check or the scene tripwire is titled by its own reason, not by passing desk numbers', () => {
  const Tape = 'The camera moved since calibration at 21:10: the gripper appears 1.2 cm from where the arm is. Lock the camera head, then run Automatic calibration.';
  const Blocked = Api.ReadLocateResponse({ ok: false, error_code: 'camera_moved', error: Tape,
    camera_check: Check({ ok: false, angle_deg: 0.1, offset_cm: 0.02, reason: 'gripper tape is 1.20 cm from where the arm puts it (limit 0.7 cm)' }) });
  const Gate = Api.CameraGateFailure(Blocked, true);
  Assert.equal(Gate.Code, 'camera_moved');
  Assert.equal(Gate.Title, Tape);
  Assert.doesNotMatch(Gate.Title, /0\.1 deg/);
  Assert.match(Gate.Detail, /gripper tape is 1\.20 cm/);
  const Tripwire = Api.ReadLocateResponse({ ok: false, error_code: 'camera_moved', error: 'The camera or the cart moved since calibration: objects on the desk do not sit on the calibrated desk.',
    camera_check: Check({ ok: false, angle_deg: 0.3, offset_cm: -0.4, reason: 'scene tripwire: 3 supports 1.8 cm low' }) });
  Assert.match(Api.CameraGateFailure(Tripwire, false).Title, /^The camera or the cart moved since calibration: objects on the desk/);
});

Test('an unverified or unchecked camera stops tasks that act on objects, not a description or a jog', () => {
  const NotVerified = Api.ReadLocateResponse({ ok: false, error_code: 'camera_not_verified', error: 'Object positions are blocked: the camera is not calibrated.' });
  const CheckFailed = Api.ReadLocateResponse({ ok: false, error_code: 'camera_check_failed', error: 'The desk is not visible in depth.' });
  Assert.match(Api.CameraGateFailure(NotVerified, true).Title, /no camera check reference/);
  Assert.match(Api.CameraGateFailure(CheckFailed, true).Title, /could not be checked/);
  Assert.equal(Api.CameraGateFailure(NotVerified, false), null, '"Open the gripper" still runs while the calibration is cleared');
  Assert.equal(Api.CameraGateFailure(CheckFailed, false), null);
  Assert.match(Api.MeasuredObjectsSummary(NotVerified), /NO MEASURED OBJECT POSITIONS: Object positions are blocked/);
  // A webcam capture can never carry the depth baseline: say what would work.
  const Webcam = Api.CameraGateFailure(NotVerified, true, 'webcam');
  Assert.equal(Webcam.Code, 'camera_not_verified');
  Assert.match(Webcam.Title, /^Tasks that act on objects need the depth camera/);
});

Test('a cart rolled against the desk: an unjudged tape stops object tasks without recalibration wording, a re-measured desk never blocks', () => {
  // The bridge's sentence (CameraDrift._DeskChanged), as on 2026-10-02 with the arm's torque off.
  const Error = 'The desk moved against the arm since calibration at 21:09 (was the cart moved?), so the camera is checked against the red tape '
    + "on the gripper instead, and the arm's torque is off, so its pose cannot be trusted. Turn the torque on with the arm parked and the red tape in view, then send again.";
  const Unverified = Api.ReadLocateResponse({ ok: false, error_code: 'desk_changed_unverified', error: Error,
    camera_check: Check({ ok: false, angle_deg: 2.268, offset_cm: -0.83, tape_check: 'unavailable', tape_residual_cm: null,
      reason: 'the desk moved 1.69 deg / +0.09 cm against the camera since calibration, and the gripper tape could not check the camera: torque off' }) });
  Assert.ok(Api.CameraGateCodes.includes('desk_changed_unverified'));
  const Gate = Api.CameraGateFailure(Unverified, true);
  Assert.equal(Gate.Code, 'desk_changed_unverified');
  Assert.equal(Gate.Title, Error);
  Assert.doesNotMatch(Gate.Title, /recalibrat|Automatic calibration|lock the (camera )?head/i);
  Assert.doesNotMatch(Gate.Title, /2\.3 deg/, 'not titled as a moved camera by the desk numbers');
  Assert.match(Gate.Detail, /torque off/);
  // Not a moved camera: a description or a jog still runs, told that nothing was measured.
  Assert.equal(Api.CameraGateFailure(Unverified, false), null);
  Assert.match(Api.MeasuredObjectsSummary(Unverified), /NO MEASURED OBJECT POSITIONS: The desk moved against the arm/);
  // A bridge that sends the code without a sentence still gets an actionable, non-recalibrating title.
  const Bare = Api.CameraGateFailure(Api.ReadLocateResponse({ ok: false, error_code: 'desk_changed_unverified', error: '' }), true);
  Assert.match(Bare.Title, /cart moved\?\): park the arm with torque on and the red tape on the gripper in view, hold it still/);
  Assert.doesNotMatch(Bare.Title, /recalibrat|Automatic calibration/i);
  // The tape caught a camera that moved while the desk changed: the moved-camera gate as before, on every request.
  const Moved = Api.ReadLocateResponse({ ok: false, error_code: 'camera_moved', error: 'The camera moved since calibration at 21:09: the gripper appears 1.5 cm from where the arm is. Lock the camera head, then run Automatic calibration.',
    camera_check: Check({ ok: false, angle_deg: 3.9, offset_cm: -0.76, tape_check: 'moved' }) });
  for (const Needs of [true, false]) {
    Assert.equal(Api.CameraGateFailure(Moved, Needs).Code, 'camera_moved');
    Assert.match(Api.CameraGateFailure(Moved, Needs).Title, /Camera moved 3\.9 deg \/ 0\.8 cm since calibration at 21:10: lock the head and run Automatic calibration\./);
  }
  // Accepted on a re-measured desk: no block, and one log line (saved, or not saved and measured again next time).
  const Remeasured = Api.ReadLocateResponse({ ok: true, objects: [Cup], camera_check: Check({ angle_deg: 0, offset_cm: 0, tape_check: 'ok',
    tape_residual_cm: [-0.38, -0.16, 0.13], desk_remeasured: true, desk_saved: true }) });
  Assert.equal(Remeasured.cameraCheck.DeskRemeasured, true);
  Assert.equal(Remeasured.cameraCheck.DeskSaved, true);
  Assert.equal(Api.CameraGateFailure(Remeasured, true), null);
  Assert.match(Api.DeskRemeasuredNote(Remeasured.cameraCheck), /cart moved\): the gripper tape shows the camera has not, so the desk was measured again on this frame and saved as the new baseline\.$/);
  const Unsaved = Api.ReadLocateResponse({ ok: true, objects: [Cup], camera_check: Check({ desk_remeasured: true, desk_saved: false }) });
  Assert.match(Api.DeskRemeasuredNote(Unsaved.cameraCheck), /\(not saved: the next capture measures it again\)\.$/);
  Assert.equal(Api.DeskRemeasuredNote(Ok.cameraCheck), null);
  Assert.equal(Api.DeskRemeasuredNote(null), null);
  Assert.equal('DeskRemeasured' in Ok.cameraCheck, false, 'an ordinary check reads as before');
});

Test('no calibrated desk in view: the gripper tip verifies the camera, or it is assumed with a one-line warning, or a moved camera stops every request', () => {
  // The bridge's camera_check with the calibrated desk out of view (back end/RobotReference.py), as on the
  // 2026-10-06 cabinet frames: 15:01 verified (tape 0.52 cm), 23:01 after the head turn moved (2.66 cm).
  const Robot = (Fields = {}) => Check({ angle_deg: null, offset_cm: null, reference_fitted_at: '2026-10-02T21:09:22', alignment_source: 'robot',
    desk_check: { ok: false, error_code: 'camera_check_failed', reason: 'fewer than 200 cells of the calibrated desk are visible as desk' }, ...Fields });
  const Verified = Api.ReadLocateResponse({ ok: true, objects: [Cup], camera_check: Robot({ robot_check: 'verified', tape_check: 'ok', tape_error_cm: 0.52,
    tape_residual_cm: [-0.5, 0.13, -0.06], reason: 'no calibrated desk in view; the gripper tape is 0.52 cm from where the arm puts it (limit 0.75 cm, through every joint): the camera is where it was calibrated' }) });
  Assert.equal(Verified.cameraCheck.AlignmentSource, 'robot');
  Assert.equal(Verified.cameraCheck.RobotCheck, 'verified');
  Assert.equal(Verified.cameraCheck.TapeErrorCm, 0.52);
  Assert.deepEqual(Verified.cameraCheck.TapeResidualCm, { x: -0.5, y: 0.13, z: -0.06 }, 'the grasp correction gets the roll-aware residual');
  for (const Needs of [true, false]) Assert.equal(Api.CameraGateFailure(Verified, Needs), null);
  Assert.deepEqual(Api.RobotCheckNote(Verified.cameraCheck),
    { Level: 'ok', Text: 'Camera checked against the gripper tip (no desk in view): the red tape is 0.5 cm from where the calibration from 21:09 puts it.' });
  Assert.equal(Api.DeskRemeasuredNote(Verified.cameraCheck), null);
  // The tip could not be checked: the task goes on, on the calibration as it was, with the bridge's warning logged.
  const Warning = 'Camera assumed where it was calibrated at 21:09: the gripper tip could not be checked (the red tape on the gripper is not seen where the arm puts it: covered, turned away or out of view).';
  const Assumed = Api.ReadLocateResponse({ ok: true, objects: [Cup], camera_check: Robot({ robot_check: 'assumed', tape_check: 'not_visible', tape_error_cm: null,
    warning: Warning, reason: 'no calibrated desk in view; the camera is assumed where it was calibrated: the gripper tape could not check it (no tape-sized red blob near where the arm puts the tape)' }) });
  for (const Needs of [true, false]) Assert.equal(Api.CameraGateFailure(Assumed, Needs), null, 'an assumed camera does not stop the task');
  Assert.deepEqual(Api.RobotCheckNote(Assumed.cameraCheck), { Level: 'warn', Text: Warning });
  Assert.doesNotMatch(Api.RobotCheckNote(Assumed.cameraCheck).Text, /\n/);
  const Unworded = Api.ReadLocateResponse({ ok: true, objects: [Cup], camera_check: Robot({ robot_check: 'assumed', tape_check: 'unavailable' }) });
  Assert.deepEqual(Api.RobotCheckNote(Unworded.cameraCheck), { Level: 'warn', Text: 'Camera assumed where it was calibrated at 21:09: the gripper tip could not be checked.' });
  // The tip positively off: a moved camera, titled by the bridge's sentence on every request.
  const Error = 'The gripper tip is 2.7 cm from where the calibration at 21:09 puts it, so the camera moved on the robot: recalibrate (Automatic calibration).';
  const Moved = Api.ReadLocateResponse({ ok: false, error_code: 'camera_moved', error: Error, camera_check: Robot({ ok: false, robot_check: 'moved', tape_check: 'moved',
    tape_error_cm: 2.66, error_code: 'camera_moved', error: Error, reason: 'no calibrated desk in view; the gripper tape is 2.66 cm from where the arm puts it (limit 0.75 cm, through every joint)' }) });
  Assert.equal(Moved.cameraCheck.RobotCheck, 'moved');
  for (const Needs of [true, false]) {
    const Gate = Api.CameraGateFailure(Moved, Needs);
    Assert.equal(Gate.Code, 'camera_moved');
    Assert.equal(Gate.Title, Error);
    Assert.match(Gate.Detail, /2\.66 cm from where the arm puts it/);
  }
  Assert.equal(Api.RobotCheckNote(Moved.cameraCheck), null, 'a block is the gate, not a note');
  // The basket rim is advisory: read for the session record, never a gate (2026-10-05 01:42: tape 0.45 cm, rim 1.5 deg).
  const RimTurned = Api.ReadLocateResponse({ ok: true, objects: [Cup], camera_check: Robot({ robot_check: 'verified', tape_check: 'ok', tape_error_cm: 0.45,
    rim_check: 'moved', rim_yaw_deg: -1.5 }) });
  Assert.deepEqual([RimTurned.cameraCheck.RimCheck, RimTurned.cameraCheck.RimYawDeg], ['moved', -1.5]);
  for (const Needs of [true, false]) Assert.equal(Api.CameraGateFailure(RimTurned, Needs), null);
  Assert.equal(Api.RobotCheckNote(RimTurned.cameraCheck).Level, 'ok');
  // A desk check reads as before: no robot fields and no note; nor does an unknown robot verdict.
  Assert.equal('RobotCheck' in Ok.cameraCheck, false);
  Assert.equal(Api.RobotCheckNote(Ok.cameraCheck), null);
  Assert.equal(Api.RobotCheckNote(null), null);
  Assert.equal('RobotCheck' in Api.ReadLocateResponse({ ok: true, objects: [Cup], camera_check: Robot({ robot_check: 'maybe' }) }).cameraCheck, false);
});

Test('a task object seen but withheld, with nothing else placing it, stops the task with the bridge\'s reason (no unguarded targets)', () => {
  const Query = Api.ParseTaskObject('grab the pill bottle');
  const Reply = (Withheld, Objects = [Cup]) => Api.ReadLocateResponse({ ok: true, objects: Objects, camera_check: Check(), withheld: Withheld });
  // Withheld under its own label, or measured only from the task query's box under the task's label.
  const Bare = Reply([{ label: 'pill bottle', reason: 'not measurable: no level surface measured around the object to stand on (no calibrated desk in view)' }]);
  Assert.deepEqual(Api.WithheldTaskObject(Bare, Query), { Label: 'pill bottle', Reason: 'not measurable: no level surface measured around the object to stand on (no calibrated desk in view)' });
  const Stop = Api.WithheldTaskStop(Api.WithheldTaskObject(Bare, Query), 'pill bottle');
  Assert.equal(Stop.Reason, 'the pill bottle is in view but the camera could not measure it (no level surface measured around the object to stand on (no calibrated desk in view)), so no move toward it can be checked.');
  Assert.equal(Stop.Advice, 'The camera needs to see the surface the pill bottle stands on around it: clear the things right beside it, then send the task again.');
  const Queried = Reply([{ label: 'white bottle', reason: 'not measurable: more than one measured surface could support this object' },
    { label: 'pill bottle', reason: 'not measurable: more than one measured surface could support this object', task_query_only: true }]);
  Assert.equal(Api.WithheldTaskObject(Queried, Query).Label, 'white bottle', 'the samples\' own box first, named by the task\'s noun');
  Assert.equal(Api.WithheldTaskObject(Reply([{ label: 'thing', reason: 'not measurable: 3.0 cm tall', task_query_only: true }]), Query).Label, 'thing');
  Assert.match(Api.WithheldTaskStop(Api.WithheldTaskObject(Queried, Query), 'pill bottle').Advice, /^Move the pill bottle away from the edge and the other things around it/);
  // The arm in front of it, too far, an undeclared platform's hint (the automatic supports off): each says what to do.
  const Advice = Reason => Api.WithheldTaskStop({ Label: 'pill bottle', Reason }, 'pill bottle').Advice;
  Assert.match(Advice("the robot's own arm fills the box"), /^Move the arm to Home so the camera sees the pill bottle clear of it/);
  Assert.match(Advice('not measurable: 131 cm from the camera (limit 110)'), /^Move the pill bottle closer to the right arm/);
  Assert.match(Advice('not measurable: nothing stands above the table inside the box'), /^Make sure the pill bottle stands clear of other things and in view/);
  const Hinted = Reply([{ label: 'pill bottle', reason: 'not measurable: no desk surface around the object at the calibrated height', undeclared_support_z: 4.14 }]);
  Assert.match(Api.WithheldTaskStop(Api.WithheldTaskObject(Hinted, Query), 'pill bottle').Advice,
    /clear the things right beside it, then send the task again\. If it stands on a platform at Z 4\.1, declare that platform in back end\/DeclaredSurfaces\.json and reload the page\.$/);
  // Another object withheld, the robot's parts, nothing withheld, no task object or no reply: nothing to stop for.
  Assert.equal(Api.WithheldTaskObject(Ok, Query), null);
  Assert.equal(Api.WithheldTaskObject(Reply([{ label: 'book', reason: 'not measurable: 30.0 cm tall' }]), Query), null);
  Assert.equal(Api.WithheldTaskObject(Reply([]), Query), null);
  Assert.equal(Api.WithheldTaskObject(Bare, null), null);
  Assert.equal(Api.WithheldTaskObject(null, Query), null);
  // The withheld list reads as before, the task query's flag beside it.
  Assert.deepEqual(Queried.withheld[1], { Label: 'pill bottle', Reason: 'not measurable: more than one measured surface could support this object', TaskQueryOnly: true });
  Assert.deepEqual(Ok.withheld, [{ Label: 'white box', Reason: '553 cm away' }, { Label: 'robot arm', Reason: "the robot's own part" }]);
});

Test('App.tsx stops on a withheld task object only when nothing else places it: not bound, never measured in this task, not in the jaws', () => {
  // The rule's own guard, as written where the task object is bound (a source check: the stop needs a live task).
  const Source = ReadFile(Path.join(Root, 'src/App.tsx'), 'utf8');
  const At = Source.indexOf('const Withheld = !Resolution.Binding && !TaskObject.LastSeen && !TaskGripperClosedRef.current');
  Assert.ok(At > 0, 'the withheld stop is guarded by the binding, the last measurement and the jaws');
  const Block = Source.slice(At, Source.indexOf('return;', At) + 7);
  Assert.match(Block, /WithheldTaskObject\(Located, TaskObject\.Query\)/);
  Assert.match(Block, /StopTaskWithReason\(Stop\.Reason, Stop\.Advice\);\s*return;/);
  Assert.ok(At < Source.indexOf('const UnreliableNow = () => UnreliableMeasurements(Measured, TaskObject);'), 'before the model is asked');
});

Test('a missing, failed or unchecked measurement stops a grasp task but not a description', () => {
  Assert.equal(Api.CameraGateFailure(null, true).Code, 'locate_unavailable');
  Assert.equal(Api.CameraGateFailure(null, false), null);
  const Down = Api.ReadLocateResponse({ ok: false, error: 'No Ollama model available for localization.' });
  Assert.match(Api.CameraGateFailure(Down, true).Title, /No Ollama model/);
  Assert.equal(Api.CameraGateFailure(Down, false), null);
  const Unchecked = Api.ReadLocateResponse({ ok: true, objects: [Cup] });
  Assert.equal(Api.CameraGateFailure(Unchecked, true).Code, 'camera_not_verified');
  Assert.equal(Api.CameraGateFailure(Ok, true), null);
  Assert.equal(Api.TaskNeedsPositions('Grab the eraser'), true);
  Assert.equal(Api.TaskNeedsPositions('put it in the bin'), true);
  Assert.equal(Api.TaskNeedsPositions('move the cup'), true);
  for (const Text of ['Hand me the eraser', 'Give me the pen', 'Throw the paper in the bin', 'Remove the cup from the desk',
    'Press the button', 'Toss the pen', 'Flip the card', 'Clean up the desk', 'Tidy the desk', 'Open the box', 'Drag the cup closer']) {
    Assert.equal(Api.TaskNeedsPositions(Text), true, Text);
  }
  for (const Text of ['Describe the workspace; do not move.', 'Open the gripper', 'open your claw', 'Close the gripper', 'move up 3 cm',
    'Respond without moving.', 'Wave', 'What do you see?']) {
    Assert.equal(Api.TaskNeedsPositions(Text), false, Text);
  }
});

Test('the measured header quotes this frame\'s camera check and never claims ground truth or ±1 cm', () => {
  const Text = Api.MeasuredObjectsSummary(Ok);
  Assert.match(Text, /MEASURED OBJECT POSITIONS \(measured in this frame, camera check passed: 0\.1 deg \/ 0\.1 cm from the calibration fitted at 21:10;/);
  Assert.doesNotMatch(Text, /ground truth|±1 cm|calibration RMS/i);
  Assert.match(Text, /- white box: not measurable: 553 cm away/);
  Assert.match(Text, /- robot arm: not measurable: the robot's own part/);
  Assert.match(Text, /more than 3 cm, do not move toward it: reply ask/);
  Assert.match(Text, /- eraser: .*base not measured, desk height assumed\).*\[UNRELIABLE: outside the camera-calibrated area/);
  Assert.match(Api.MeasuredObjectsSummary(null), /NO MEASURED OBJECT POSITIONS: the bridge did not answer/);
  Assert.match(Api.MeasuredObjectsSummary(Api.ReadLocateResponse({ ok: true, objects: [], camera_check: Check() })), /no objects were measured in this frame/);
});

Test('an absolute descent onto an [UNRELIABLE] task object is refused; a hover above it, a relative step or a reliable one is not', () => {
  const Unreliable = Ok.objects[0];
  const Copied = Api.modelPointFromDisplayPoint({ x: 16.1, y: 35.8, z: 5 });
  Assert.match(Api.UnreliableTargetReason(Copied, true, Unreliable), /descends onto the \[UNRELIABLE\] eraser.*hover at Z 8\.2 or higher/);
  // Top Z 5.2 + 3 cm clearance: the hover is checked in the next image before any descent.
  Assert.equal(Api.UnreliableTargetReason(Api.modelPointFromDisplayPoint({ x: 16.1, y: 35.8, z: 8.2 }), true, Unreliable), null);
  Assert.match(Api.UnreliableTargetReason(Api.modelPointFromDisplayPoint({ x: 16.1, y: 35.8, z: 8.1 }), true, Unreliable), /UNRELIABLE/);
  Assert.equal(Api.UnreliableTargetReason(Copied, false, Unreliable), null);
  Assert.equal(Api.UnreliableTargetReason(Copied, true, { ...Unreliable, extrapolated: false }), null);
  Assert.equal(Api.UnreliableTargetReason(Api.modelPointFromDisplayPoint({ x: 5, y: 30, z: 5 }), true, Unreliable), null);
});

Test('the validator refuses a target_cm descent onto an [UNRELIABLE] task object, but not a hover above it or a relative step', () => {
  const Unreliable = { ...Ok.objects[0], x: -5, y: 35 };
  const Validate = Motion => {
    const Parsed = App.ParseResponse(JSON.stringify({ task_status: 'continue', motion_proposal: Motion }), App.HomeTarget, App.Servos);
    Assert.ok(Parsed.motion, JSON.stringify(Motion));
    return App.Validate(Parsed.motion, 'up', App.Servos, [Unreliable]);
  };
  for (const Target of [{ x: -5, y: 35, z: 6 }, { x: -4.2, y: 36, z: 8 }]) {
    const Result = Validate({ target_cm: Target });
    Assert.equal(Result.possible, false, JSON.stringify(Target));
    Assert.ok(Result.issues.some(Issue => /\[UNRELIABLE\] eraser/.test(Issue)), Result.issues.join('; '));
  }
  // Only Z given keeps the tip's X/Y, a relative step is checked in the next image, and X/Y alone keeps the
  // home height, a hover well above the object's top: none of them descends onto the object.
  const Home = Api.mirrorDisplayPoint(App.HomeTarget);
  Assert.ok(Home.z >= 5.2 + 3, `home Z ${Home.z} is a hover height`);
  for (const Motion of [{ target_cm: { z: 6 } }, { move_delta_cm: { left: -5 - Home.x, farther: 35 - Home.y, up: 0 } }, { target_cm: { x: -5, y: 35 } }]) {
    Assert.ok(!Validate(Motion).issues.some(Issue => /UNRELIABLE/.test(Issue)), JSON.stringify(Motion));
  }
  // As the app runs it, snapped first: no wrist pitch given, the preferred -60 deg or the nearest the arm holds there (2026-10-03).
  const Reliable = App.ParseResponse(JSON.stringify({ task_status: 'continue', motion_proposal: { target_cm: { x: -5, y: 35, z: 6 } } }), App.HomeTarget, App.Servos).motion;
  const Snapped = App.ApplySnap(Reliable, App.SnapAiTarget(App.SnapInput(Reliable, App.Servos, 'up', undefined, [])));
  Assert.equal(App.Validate(Snapped, 'up', App.Servos, [{ ...Unreliable, extrapolated: false }]).possible, true, 'A reliable measurement is a valid target');
});

Test('the [UNRELIABLE] descent rule guards every unreliable object in the frame, not only the matched task object', () => {
  // 16:10: a typo named no located object, so the rule never ran; the target descended onto the unreliable one.
  const Other = { ...Ok.objects[0], label: 'small black object', x: -5, y: 35 };
  const Motion = App.ParseResponse(Reply({ target_cm: { x: -5, y: 35, z: 1 } }), App.HomeTarget, App.Servos).motion;
  const Result = App.Validate(Motion, 'up', App.Servos, [{ ...Ok.objects[1] }, Other]);
  Assert.equal(Result.possible, false);
  Assert.ok(Result.issues.some(Issue => /descends onto the \[UNRELIABLE\] small black object measurement/.test(Issue)), Result.issues.join(' | '));
  Assert.ok(!App.Validate(Motion, 'up', App.Servos, [{ ...Other, extrapolated: false }]).issues.some(Issue => /UNRELIABLE/.test(Issue)));
});

Test('one thing changes per motion: a restated servo is ignored, two changing ones or a tip move with a gripper change are refused', () => {
  const Servos = structuredClone(App.Servos);
  const Gripper = Servos.gripper.targetRaw, Roll = Servos.wrist_roll.targetRaw;
  const Other = Raw => (Raw + 400) % 4096;
  const Validate = Motion => {
    const Parsed = App.ParseResponse(JSON.stringify({ task_status: 'continue', motion_proposal: Motion }), App.HomeTarget, Servos);
    Assert.ok(Parsed.motion, JSON.stringify(Motion));
    return App.Validate(Parsed.motion, 'up', Servos);
  };
  // The old prompt example: an approach step restating the open gripper.
  const Approach = Validate({ move_delta_cm: { left: 0, farther: 0, up: 2 }, direct_servo_targets: [{ name: 'gripper', target_raw: Gripper }] });
  Assert.equal(Approach.possible, true, Approach.issues.join('; '));
  Assert.match(Validate({ move_delta_cm: { left: 0, farther: 0, up: 2 }, direct_servo_targets: [{ name: 'gripper', target_raw: Other(Gripper) }] }).issues.join(' '),
    /separate approved steps/);
  Assert.match(Validate({ direct_servo_targets: [{ name: 'gripper', target_raw: Other(Gripper) }, { name: 'wrist_roll', target_raw: Other(Roll) }] }).issues.join(' '),
    /Change one slider per motion/);
  Assert.equal(Validate({ direct_servo_targets: [{ name: 'gripper', target_raw: Other(Gripper) }, { name: 'wrist_roll', target_raw: Roll }] }).issues
    .some(Issue => /one slider|separate/.test(Issue)), false, 'A restated wrist roll does not count');
  Assert.deepEqual(Api.ChangingServoTargets([{ name: 'gripper', targetRaw: Gripper + 10 }, { name: 'wrist_roll', targetRaw: Other(Roll) }],
    { gripper: Gripper, wrist_roll: Roll }).map(Item => Item.name), ['wrist_roll']);
});

Test('the check-in waits for a newer motion epoch, fresh still state, and the minimum time from the motion start', () => {
  const Wait = { approvedAt: 0, epoch: 7, startedAt: null };
  const Frozen = { Epoch: 7, StateAgeMs: 9000, RouteActive: false, Moving: false, WithinTolerance: true, MotionTimeoutMs: 12000 };
  Assert.equal(Api.TaskSettleVerdictFor(Wait, 1600, Frozen), 'waiting', 'Frozen pre-motion state is not arrival');
  Assert.equal(Api.TaskSettleVerdictFor(Wait, 24900, Frozen), 'waiting');
  Assert.equal(Api.TaskSettleVerdictFor(Wait, 25000, Frozen), 'not-started', 'The bridge never started the motion');
  Assert.equal(Api.TaskSettleVerdictFor(Wait, 9000, { ...Frozen, Epoch: 8 }), 'started');
  const Started = { ...Wait, startedAt: 9000 };
  const Fresh = { ...Frozen, Epoch: 8, StateAgeMs: 50 };
  Assert.equal(Api.TaskSettleVerdictFor(Started, 9000 + Api.TaskSettleMinMs - 1, Fresh), 'waiting', 'TaskSettleMinMs counts from the motion start');
  Assert.ok(Api.TaskSettleMinMs <= 300, 'A short move is not held back long after it stopped');
  for (const Busy of [{ StateAgeMs: 800 }, { RouteActive: true }, { Moving: true }, { WithinTolerance: false }]) {
    Assert.equal(Api.TaskSettleVerdictFor(Started, 10600, { ...Fresh, ...Busy }), 'waiting', JSON.stringify(Busy));
  }
  Assert.equal(Api.TaskSettleVerdictFor(Started, 9000 + Api.TaskSettleMinMs, Fresh), 'settled');
  Assert.equal(Api.TaskSettleVerdictFor(Started, 10600, Fresh), 'settled');
  Assert.equal(Api.TaskSettleVerdictFor(Started, 9000 + 11999, { ...Fresh, Moving: true }), 'waiting', 'The timeout counts from the motion start');
  Assert.equal(Api.TaskSettleVerdictFor(Started, 9000 + 12000, { ...Fresh, Moving: true }), 'timeout');
  // motion_timeout_ms is the bridge's budget for the whole route, every stage
  // included: a 16-stage route is not given 16 times as long.
  const Route = { ...Fresh, Moving: true, MotionTimeoutMs: 20000 };
  Assert.equal(Api.TaskSettleVerdictFor({ ...Started, stages: 16 }, 9000 + 19999, Route), 'waiting');
  Assert.equal(Api.TaskSettleVerdictFor({ ...Started, stages: 16 }, 9000 + 20000, Route), 'timeout');
});

Test('a move to the held target or the live tip is a no-op; a real step or a gripper change is not', () => {
  const Tip = { x: -28.3, y: 32, z: 0.5 }, Held = { x: -28.3, y: 32, z: 1.268, wristPitch: -16.85 };
  const Live = { Tip, WristPitch: -16.85 };
  const Move = Target => ({ hasTargetMotion: true, target: { wristPitch: -16.85, ...Target }, servoTargets: [] });
  Assert.match(Api.NoOpReason(Move(Held), Live, Held, {}), /already holding/, 'The incident: re-sending the held X 15 Y 32 Z 6');
  Assert.match(Api.NoOpReason(Move({ ...Tip, z: Tip.z + 0.9 }), Live, null, {}), /where the tip already is/);
  Assert.equal(Api.NoOpReason(Move({ ...Tip, y: Tip.y + 2 }), Live, Held, {}), null);
  Assert.equal(Api.NoOpReason(Move({ ...Held, wristPitch: -40 }), Live, Held, {}), null, 'A 23 deg wrist change is a motion');
  // After Home the bridge still holds the old target, far from the tip.
  Assert.equal(Api.NoOpReason(Move({ x: -28.3, y: 32, z: 6 }), { Tip: { x: -1.8, y: 20, z: 12 }, WristPitch: 0 }, { x: -28.3, y: 32, z: 6, wristPitch: -16.85 }, {}), null);
  const Gripper = Raw => ({ hasTargetMotion: false, target: Held, servoTargets: [{ name: 'gripper', targetRaw: Raw }] });
  Assert.match(Api.NoOpReason(Gripper(2240), Live, Held, { gripper: 2242 }), /gripper is already there/);
  Assert.equal(Api.NoOpReason(Gripper(700), Live, Held, { gripper: 2242 }), null);
  // Holding the tip and closing the gripper is a motion, whether the tip part
  // restates the held target or a wrist rounded 0.5 deg off the live one.
  const HoldAndClose = Target => ({ hasTargetMotion: true, target: Target, servoTargets: [{ name: 'gripper', targetRaw: 700 }] });
  Assert.equal(Api.NoOpReason(HoldAndClose(Held), Live, Held, { gripper: 2242 }), null);
  Assert.equal(Api.NoOpReason(HoldAndClose({ ...Tip, wristPitch: -17.35 }), Live, Held, { gripper: 2242 }), null);
  Assert.match(Api.NoOpReason(HoldAndClose(Held), Live, Held, { gripper: 705 }), /already holding/, 'A gripper restated where it is stays a no-op');
  Assert.equal(Api.TargetKey(Api.modelPointFromDisplayPoint({ x: 15.1, y: 31.9, z: 6.2 })), Api.TargetKey(Api.modelPointFromDisplayPoint({ x: 14.9, y: 32.1, z: 5.9 })));
  Assert.notEqual(Api.TargetKey(Api.modelPointFromDisplayPoint({ x: 15, y: 32, z: 6 })), Api.TargetKey(Api.modelPointFromDisplayPoint({ x: 15, y: 32, z: 7 })));
});

Test('a wrist-only correction at the same point is a new motion, not a repeat of the last target', () => {
  const Point = Api.modelPointFromDisplayPoint({ x: 5.2, y: 34.7, z: 4.2 });
  const First = { ...Point, wristPitch: -30.06 }, Turned = { ...Point, wristPitch: -20 };
  // The point alone cannot tell them apart: the old check stopped this task.
  Assert.equal(Api.TargetKey(First), Api.TargetKey(Turned));
  const Mark = Api.MarkFinalTarget(First);
  Assert.equal(Api.RepeatsLastTarget(Mark, Turned), false);
  Assert.equal(Api.RepeatsLastTarget(Mark, { ...First, wristPitch: -31.5 }), true, 'Within NoOpDeg it is the same target');
  Assert.equal(Api.RepeatsLastTarget(Mark, { ...Api.modelPointFromDisplayPoint({ x: 5.2, y: 36, z: 4.2 }), wristPitch: -30.06 }), false);
  Assert.equal(Api.RepeatsLastTarget(null, First), false);
});

Test('a task reads the arm pose only when the arm is still, has been for a moment, and its state is fresh', () => {
  const Still = { Live: true, StateAgeMs: 40, RouteActive: false, Moving: false, StillForMs: 5000 };
  Assert.equal(Api.ArmNotStillReason(Still), null);
  for (const Busy of [{ RouteActive: true }, { Moving: true }, { StillForMs: Api.ArmStillHoldMs - 1 }]) {
    Assert.equal(Api.ArmNotStillReason({ ...Still, ...Busy }), 'the arm is moving', JSON.stringify(Busy));
  }
  Assert.match(Api.ArmNotStillReason({ ...Still, StateAgeMs: 2500 }), /no arm state for 2\.5 s, as it does while it plans a motion/);
  Assert.equal(Api.ArmNotStillReason({ ...Still, Live: false, Moving: true, StateAgeMs: 1e9 }), null, 'An arm the bridge does not stream is not moving');
  Assert.equal(Api.ArmStillWaitMs, 10000);
  Assert.equal(Api.ArmMovedDuringCaptureCm, 1);
});

Test('a step\'s time limit rejects at the deadline even if the call ignores its signal; a stop aborts the call, not as a timeout', async () => {
  Assert.equal(Api.AiModelTimeoutMs, 120000);
  Assert.equal(Api.LocateTimeoutMs, 60000);
  const Never = () => new Promise(() => {});
  const Started = Date.now();
  await Assert.rejects(Api.WithTimeout(Never, new AbortController().signal, 60, 'the model (x)'),
    Failure => Failure instanceof Api.AiStepTimeoutError && Failure.message === 'the model (x) did not answer within 0.06 s');
  Assert.ok(Date.now() - Started < 1000);
  // The call sees its signal aborted at the deadline (a fetch rejects then).
  let Seen = null;
  await Assert.rejects(Api.WithTimeout(Signal => new Promise((_, Reject) => Signal.addEventListener('abort', () => { Seen = Signal.aborted; Reject(new Error('aborted')); })),
    new AbortController().signal, 30, 'the object measurement'), Api.AiStepTimeoutError);
  Assert.equal(Seen, true);
  // Stop task aborts the parent: the call is aborted and its own error comes through.
  const Parent = new AbortController();
  const Stopped = Api.WithTimeout(Signal => new Promise((_, Reject) => Signal.addEventListener('abort', () => Reject(new Error('stopped')))), Parent.signal, 5000, 'the model (x)');
  Parent.abort();
  await Assert.rejects(Stopped, Failure => Failure.message === 'stopped' && !(Failure instanceof Api.AiStepTimeoutError));
  Assert.equal(await Api.WithTimeout(async () => 42, new AbortController().signal, 1000, 'x'), 42);
});

Test('the task object line says whether it was detected this check-in and where it was last measured', () => {
  Assert.equal(Api.TaskObjectNoun('Grab the eraser'), 'eraser');
  Assert.equal(Api.TaskObjectNoun('pick up the red cups and put them in the bin'), 'cup');
  Assert.equal(Api.TaskObjectNoun('move up and grab the cup'), 'cup');
  for (const Text of ['move up 3 cm', 'Describe the workspace', 'Open the gripper', 'move the arm left', 'Tidy the desk']) Assert.equal(Api.TaskObjectNoun(Text), null, Text);
  // Politeness, adverbs and clauses after the object are not the object.
  for (const [Text, Noun] of [['Grab the eraser please', 'eraser'], ['Grab the eraser now', 'eraser'], ['Grab the blue eraser carefully', 'eraser'],
    ['Pick up the pen that is on the left', 'pen'], ['Grab the eraser right now', 'eraser'], ['Hand me the eraser', 'eraser'],
    ['Give me a pen', 'pen'], ['Throw the paper in the bin', 'paper'], ['Remove the cup from the desk', 'cup'], ['Open the box', 'box']]) {
    Assert.equal(Api.TaskObjectNoun(Text), Noun, Text);
  }
  // An adverb after the object is not the object: "gingerly" was the label until the eraser was matched.
  Assert.equal(Api.ParseTaskObject('Grab the eraser gingerly').Head, 'eraser');
  const Resolve = (Task, Objects, Options) => Api.ResolveTaskObject(Api.ParseTaskObject(Task), Objects, Options).Binding;
  Assert.equal(Resolve('Grab the eraser gingerly', Ok.objects).Object.label, 'eraser');
  Assert.equal(Resolve('Grab the red cup', [{ ...Ok.objects[1], label: 'red pen' }]), null,
    'An adjective of the phrase does not pick another object ("the red cup" is not the red pen)');
  Assert.equal(Resolve('Grab the cup', [Ok.objects[0]], { LastSeen: Ok.objects[1] }), null,
    'Once found, the object is not re-bound to another one far from where it was');
  const Found = Resolve('Grab the eraser', Ok.objects).Object;
  Assert.equal(Found.label, 'eraser');
  const At = new Date(2026, 8, 28, 19, 4, 11).getTime();
  const State = { Label: 'eraser', Words: [], DetectedNow: true, LastSeen: { At, Object: Found, Reachability: null },
    Reachability: { Kind: 'out-of-reach', Text: "OUT OF REACH (46.3 cm from the right arm's shoulder)", BaseDistanceCm: 46.3 } };
  Assert.match(Api.TaskObjectLine(State), /TASK OBJECT "eraser": detected this check-in at X 16\.1, Y 35\.8.*\[UNRELIABLE.*RIGHT ARM: OUT OF REACH/);
  // Out of reach means reply ask: no hover advice that would invite a move toward it.
  Assert.doesNotMatch(Api.TaskObjectLine(State), /hover waypoint/);
  Assert.match(Api.TaskObjectLine({ ...State, Reachability: null }), /tagged \[UNRELIABLE\], target_cm is accepted only for the hover waypoint/, 'The line says how to approach a tagged object');
  Assert.doesNotMatch(Api.TaskObjectLine(State, { x: 0, y: 20, z: 10 }), /READY-MADE MOVES/, 'No ready-made move toward an object out of reach');
  // The ready-made move is exact, with signs: the model flipped farther +11.5 to -11.5 when it subtracted itself.
  const Graspable_ = { Kind: 'graspable', Text: 'graspable at Z 1.5 with wrist -60 deg', BaseDistanceCm: 30 };
  const Line = Api.TaskObjectLine({ ...State, Reachability: Graspable_ }, { x: -14.0, y: 17.8, z: 5.8 });
  Assert.match(Line, /READY-MADE MOVES from the tip now \(X -14\.0, Y 17\.8, Z 5\.8\): 1\) hover above it: "motion_proposal":\{"target_cm":\{"x":16\.1,"y":35\.8,"z":8\.2\}\} \(the same as move_delta_cm \{"left":30\.1,"farther":18\.0,"up":2\.4\}\)/);
  Assert.match(Line, /descend to the grasp height Z 1\.5: "motion_proposal":\{"move_delta_cm":\{"left":0,"farther":0,"up":-6\.7\}\}/);
  for (const Snippet of Line.match(/\{"(target_cm|move_delta_cm)":\{[^}]*\}\}/g)) Assert.doesNotThrow(() => JSON.parse(Snippet), Snippet);
  // The hover it hands out is one the refusal accepts.
  Assert.equal(Api.UnreliableTargetReason(Api.modelPointFromDisplayPoint({ x: 16.1, y: 35.8, z: 8.2 }), true, Found), null);
  Assert.doesNotMatch(Api.TaskObjectLine({ ...State, Reachability: { ...Graspable_, Kind: 'blocked' } }, { x: 0, y: 20, z: 10 }), /READY-MADE MOVES/);
  const Missing = Api.TaskObjectLine({ ...State, DetectedNow: false });
  Assert.match(Missing, /TASK OBJECT "eraser": NOT detected this check-in \(possibly under the gripper or out of view\); last measured at 19:04:11 at X 16\.1, Y 35\.8/);
  Assert.match(Missing, /Coordinates in earlier replies are stale/);
  Assert.doesNotMatch(Missing, /OUT OF REACH/, 'No verdict was kept with that measurement');
  // Hidden under the gripper while grasping: the verdict from when it was last measured still stands.
  const Graspable = { Kind: 'graspable', Text: 'graspable at Z -1.3 with wrist -60 deg', BaseDistanceCm: 30 };
  const Hidden = Api.TaskObjectLine({ ...State, DetectedNow: false, Reachability: null,
    LastSeen: { At, Object: { ...Found, extrapolated: false }, Reachability: Graspable } });
  Assert.match(Hidden, /NOT detected this check-in.*RIGHT ARM when last measured: graspable at Z -1\.3 with wrist -60 deg\./);
  Assert.doesNotMatch(Hidden, /UNRELIABLE/);
  Assert.match(Api.TaskObjectLine({ ...State, DetectedNow: false, LastSeen: null }), /NOT detected in any check-in of this task/);
});

Test('stored history carries no numeric old targets and a slice never starts with an orphaned reply or feedback turn', () => {
  const Reply = JSON.stringify({ response_to_user: 'The measured position of the eraser is X=16.1, Y=35.8.', task_status: 'continue',
    motion_proposal: { target_cm: { x: 16.1, y: 35.8, z: 5 }, wrist_pitch_degrees: -24.1694 }, target_estimate_cm: { left: 16.1, farther: 35.8, up: 0 } });
  const Turn = Api.CompactTurn([{ role: 'user', content: 'AUTO CHECK-IN\n- Motion 4: tip to X 15, Y 32, Z 6 cm' }, { role: 'assistant', content: Reply },
    { role: 'user', content: 'Feasibility feedback 1: IK target misses by 2.9459 cm.' }, { role: 'assistant', content: Reply }], 11);
  Assert.equal(Turn.length, 2);
  for (const Message of Turn) {
    Assert.doesNotMatch(Message.content, /16\.1|35\.8|\b[XYZ]\s*=?\s*-?\d/, Message.content);
    Assert.doesNotMatch(Message.content, /"target_cm"\s*:\s*\{/);
  }
  Assert.match(Turn[1].content, /"target_cm":"\[stale\]"/);
  Assert.match(Turn[0].content, /rejected 1 proposal in this turn/);
  const History = [
    { role: 'assistant', content: 'orphan reply', turn: 1 }, { role: 'user', content: 'Feasibility feedback 1', turn: 2 },
    { role: 'user', content: 'a', turn: 3 }, { role: 'assistant', content: 'b', turn: 3 },
    { role: 'user', content: 'c', turn: 4 }, { role: 'assistant', content: 'd', turn: 4 },
    { role: 'user', content: 'e', turn: 5 }, { role: 'assistant', content: 'f', turn: 5 }];
  for (const Turns of [1, 2, 3, 5]) {
    const Slice = Api.SliceWholeTurns(History, Turns);
    Assert.equal(Slice[0].role, 'user');
    Assert.ok(!/orphan|Feasibility/.test(Slice.map(Message => Message.content).join()));
    Assert.equal(Slice.length, 2 * Math.min(Turns, 3));
  }
});

Test('feasibility feedback says where the arm is, gives the nearest reachable point and drops a duplicate clamp line', () => {
  const Context = { Servos: Api.INITIAL_SERVOS, ElbowMode: 'up' };
  const Current = Api.HOME_MODEL_TARGET_CM;
  // The incident's rejected proposal: X 16.1, Y 35.8, Z 5, wrist -24.1694.
  const Requested = { ...Api.modelPointFromDisplayPoint({ x: 16.1, y: 35.8, z: 5 }), wristPitch: -24.1694 };
  const Motion = { requestedTarget: Requested, target: Requested, requestedWristPitchTarget: -24.1694, servoTargets: [],
    requestedRelativeInches: { left: Current.x - Requested.x, farther: Requested.y - Current.y, up: Requested.z - Current.z } };
  const Text = Api.BuildAiFeasibilityFeedbackPrompt(['IK target misses by 2.9459 cm.'], Motion, Current, Context, 1, 4);
  // Home comes from back end/home_preset.json, which Set Home rewrites: expect wherever it puts the tip.
  const Home = Api.mirrorDisplayPoint(Current), N = Api.FormatAiPromptNumber;
  Assert.ok(Text.includes(`The arm is currently at:\nX ${N(Home.x)} cm, Y ${N(Home.y)} cm, Z ${N(Home.z)} cm, wrist ${N(Current.wristPitch)} deg.`), Text);
  Assert.doesNotMatch(Text, /Current target is still|clamped|adjusted that/);
  const Nearest = Text.match(/Nearest reachable, collision-free point on the straight line from the tip toward your target: X (-?[\d.]+) cm, Y (-?[\d.]+) cm, Z (-?[\d.]+) cm, wrist (-?[\d.]+) deg \(([\d.]+) cm short of your target\)/);
  Assert.ok(Nearest, Text);
  const [X, Y, Z, Pitch, Short] = Nearest.slice(1).map(Number);
  Assert.ok(Short > 1 && Short < 5, `${Short} cm short`);
  Assert.ok(Api.solveIkPreview({ ...Api.modelPointFromDisplayPoint({ x: X, y: Y, z: Z }), wristPitch: Pitch }, 'up', Api.INITIAL_SERVOS).reachable,
    'The suggested point is reachable');
  // A wrist the app changed is still reported, since it differs from the request.
  const Forced = Api.BuildAiFeasibilityFeedbackPrompt(['IK target misses by 2.9 cm.'], { ...Motion, target: { ...Requested, wristPitch: -16.85 } }, Current, Context, 1, 4);
  Assert.match(Forced, /The app adjusted that to this preview target:\nX 16\.1 cm, Y 35\.8 cm, Z 5 cm, wrist -16\.85 deg\./);
});

Test('a basket rejection says the lowest clear height at that X/Y', () => {
  const Context = { Servos: Api.INITIAL_SERVOS, ElbowMode: 'up' };
  // A desk point just past the rim that the basket blocks at every sampled pitch; the first that is,
  // so a change to the basket model moves the case instead of breaking it. Since the CAD-exact contact
  // model (v8) the arm reaches the desk out to Y 29 there: the last candidates are nearer the cart.
  const [X, Y] = [[5.4, 33], [5.4, 32], [10, 33], [5.4, 31], [10, 32], [5.4, 30], [5.4, 29], [10, 29], [5.4, 28], [10, 28], [5.4, 27]].find(([X_, Y_]) => {
    const Reach = Api.ReachAt(Api.modelPointFromDisplayPoint({ x: X_, y: Y_, z: -1.3 }), Context);
    return Reach.Pitch === null && Reach.Collision;
  });
  const Low = { ...Api.modelPointFromDisplayPoint({ x: X, y: Y, z: -1.3 }), wristPitch: -60 };
  const Motion = { requestedTarget: Low, target: Low, requestedWristPitchTarget: -60, servoTargets: [], requestedRelativeInches: { left: 0, farther: 0, up: 0 } };
  const Text = Api.BuildAiFeasibilityFeedbackPrompt(['gripper link could contact the basket'], Motion, Api.HOME_MODEL_TARGET_CM, Context, 1, 4);
  // Not "every wrist pitch hits the basket": steep pitches there fail on reach, not on the basket.
  Assert.doesNotMatch(Text, /every wrist pitch/);
  const Clear = Text.match(new RegExp(String.raw`At X ${X}, Y ${Y} no wrist pitch reaches below Z (-?[\d.]+) cm without touching the basket; that is the lowest clear height there \(wrist (-?[\d.]+) deg\)`));
  Assert.ok(Clear, Text);
  const [Z, Pitch] = Clear.slice(1).map(Number);
  Assert.ok(Z > -1.3 && Z <= 4.7, `clear height ${Z}`);
  Assert.ok(Api.solveIkPreview({ ...Api.modelPointFromDisplayPoint({ x: X, y: Y, z: Z }), wristPitch: Pitch }, 'up', Api.INITIAL_SERVOS).reachable);
  for (const Wrist of Api.ReachPitches) {
    Assert.equal(Api.solveIkPreview({ ...Api.modelPointFromDisplayPoint({ x: X, y: Y, z: Z - 0.5 }), wristPitch: Wrist }, 'up', Api.INITIAL_SERVOS).reachable, false,
      `0.5 cm lower is blocked at wrist ${Wrist}`);
  }
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

Test('the feedback hints stay cheap on the page thread and never blame the basket for a reach failure', () => {
  const Context = { Servos: Api.INITIAL_SERVOS, ElbowMode: 'down', Limits: Object.fromEntries(Api.IK_JOINTS.map(Name => [Name, [-170, 170]])) };
  const Tip = { ...Api.modelPointFromDisplayPoint({ x: -14.9, y: 19.1, z: 8.3 }), wristPitch: -45 };
  const Motion = Target => ({ requestedTarget: Target, target: Target, requestedWristPitchTarget: Target.wristPitch,
    requestedRelativeInches: { left: 0, farther: 0, up: 0 }, servoTargets: [] });
  const Phantom = { ...Api.modelPointFromDisplayPoint({ x: 16.1, y: 35.8, z: 5 }), wristPitch: -24.17 };
  const BasketLow = { ...Api.modelPointFromDisplayPoint({ x: 5.4, y: 25, z: -1.5 }), wristPitch: -60 };
  const Issues = ['Basket collision: the gripper hits the basket wall.'];
  const Slowest = Target => {
    Api.BuildAiFeasibilityFeedbackPrompt(Issues, Motion(Target), Tip, Context, 1, 4);
    let Worst = 0;
    for (let Run = 0; Run < 3; Run += 1) {
      const Started = performance.now();
      Api.BuildAiFeasibilityFeedbackPrompt(Issues, Motion(Target), Tip, Context, 1, 4);
      Worst = Math.max(Worst, performance.now() - Started);
    }
    return Worst;
  };
  // It was 680-740 ms per revision (up to four per request) with Stop and the state stream frozen meanwhile. The bound
  // scales with how busy the PC is (LoadScale: audit L5, 427-580 ms measured while other suites ran).
  const Scale = LoadScale(() => Api.solveIkPreview(Tip, 'up', Api.INITIAL_SERVOS));
  for (const [Name, Target] of [['phantom', Phantom], ['basket', BasketLow]]) {
    const Worst = Slowest(Target);
    Assert.ok(Worst < 400 * Scale, `${Name}: ${Worst.toFixed(0)} ms (load x${Scale.toFixed(2)})`);
  }
  const Far = Api.BuildAiFeasibilityFeedbackPrompt(Issues, Motion(Phantom), Tip, Context, 1, 4);
  Assert.doesNotMatch(Far, /clears the basket|clear of the basket|hits the basket below/, 'Out of reach there, not blocked by the basket');
  Assert.match(Far, /Nearest reachable, collision-free point/);
  Assert.match(Api.BuildAiFeasibilityFeedbackPrompt(Issues, Motion(BasketLow), Tip, Context, 1, 4), /no wrist pitch reaches below Z -?[\d.]+ cm without touching the basket/);
});

Test('reach checks apply the slider limits when the bridge has sent none', () => {
  // null means "no joint limits" to the solver; before the bridge's limits
  // arrive the reach tag must use the sliders, as the validator does.
  const Point = Api.modelPointFromDisplayPoint({ x: -30, y: -5, z: -2 });
  const Slider = Api.ReachAt(Point, { Servos: Api.INITIAL_SERVOS, ElbowMode: 'up' }, [-45]);
  Assert.equal(Slider.Pitch, null);
  Assert.equal(Api.ReachAt(Point, { Servos: Api.INITIAL_SERVOS, ElbowMode: 'up', Limits: null }, [-45]).Pitch, Slider.Pitch);
});

Test('the check-in states measured facts, not assumed arrival, and allows ask on the last step', () => {
  const Fact = Api.DescribeMotionFact({ Step: 6, StartTip: { x: 15.06, y: 31.81, z: 5.26 }, EndTip: { x: 15.06, y: 31.81, z: 5.26 },
    Target: { x: 15, y: 32, z: 6 }, Servos: null });
  Assert.match(Fact, /motion 6\) started and has stopped: the tip moved 0\.0 cm.*0\.8 cm from that motion's target\. That motion made no progress\./);
  const Prompt = Api.BuildTaskCheckInPrompt({ task: 'Grab the eraser', step: 3 }, ['- Motion 2: tip to X 14.2'], Fact);
  Assert.doesNotMatch(Prompt, /has finished|arrival position is the current position/);
  Assert.match(Prompt, /Coordinates in your earlier replies are stale/);
  Assert.doesNotMatch(Prompt, /last motion for this task/);
  Assert.match(Api.BuildTaskCheckInPrompt({ task: 'Grab the eraser', step: Api.MAX_AI_TASK_STEPS }, [], null),
    /This is your last motion for this task: if you cannot finish with it, reply ask and say what the user must change\./);
});

// --- Wrist pitches the app picks, reach verdicts and what the model is told --

Test('a forced wrist pitch is one the IK reaches exactly, inset from the edge, and still passes Approve one or two ticks later', () => {
  // 16:10: -90 was forced to -55.9, 0.5 deg past the reach edge; Approve re-checked with newer servo readings and refused it.
  // 07:58: -45 was forced to -43.575, which the bridge refused as unreachable.
  for (const [Name, Raw, Target, Wrist] of [['16:10', S1610, { x: 4.2, y: 32.5, z: 1.0 }, -90], ['07:58', S0758, { x: 5.2, y: 34.7, z: 4.2 }, -45]]) {
    const Servos = Snapshot(Raw);
    const Motion = Forced(Servos, { target_cm: Target, wrist_pitch_degrees: Wrist });
    Assert.notEqual(Motion.wristPitchTarget, Wrist, Name);
    Assert.ok(Converged(Motion.target, Servos), `${Name}: forced wrist ${Motion.wristPitchTarget} is only inside the IK tolerance`);
    Assert.equal(Motion.wristPitchTarget, Math.round(Motion.wristPitchTarget * 10) / 10, `${Name}: a clean 0.1 deg value`);
    Assert.deepEqual(App.Validate(Motion, 'up', Servos).issues, [], Name);
    for (const Joint of Api.IK_JOINTS) for (const Tick of [-2, -1, 1, 2]) {
      const Later = Snapshot({ ...Raw, [Joint]: Raw[Joint] + Tick });
      Assert.deepEqual(App.Validate(Motion, 'up', Later).issues, [], `${Name}: ${Joint} ${Tick > 0 ? '+' : ''}${Tick} tick refused wrist ${Motion.wristPitchTarget}`);
    }
  }
});

Test('the forced pose rounded to encoder ticks stays within the bridge send tolerances (0.1 cm, 0.25 deg)', () => {
  const Servos = Snapshot(S1610);
  const Motion = Forced(Servos, { target_cm: { x: 4.2, y: 32.5, z: 1.0 }, wrist_pitch_degrees: -90 });
  const Preview = Api.solveIkPreview(Motion.target, 'up', Servos);
  const Rounded = { ...Preview.angles };
  for (const Joint of Api.IK_JOINTS) Rounded[Joint] = Api.angleFromRawValue(Servos[Joint], Api.rawFromAngle(Servos[Joint], Preview.angles[Joint]));
  const Tip = Api.forwardKinematics(Rounded).at(-1);
  Assert.ok(Math.hypot(Tip.x - Motion.target.x, Tip.y - Motion.target.y, Tip.z - Motion.target.z) <= 0.1);
  Assert.ok(Math.abs(Api.wristPitchFromAngles(Rounded) - Motion.wristPitchTarget) <= 0.25);
});

Test('forcing a wrist pitch costs a bounded number of IK solves, and the page thread only checks the result', () => {
  // The snap's wrist stage (it runs in the worker): the inset search plus the exact check, the post check and the reason.
  for (const [Name, Raw, Target, Wrist, Budget] of [['07:58', S0758, { x: 5.2, y: 34.7, z: 4.2 }, -45, 65],
    ['16:10', S1610, { x: 4.2, y: 32.5, z: 1.0 }, -90, 165]]) {
    const Servos = Snapshot(Raw);
    App.SolveCounter.Count = 0;
    Forced(Servos, { target_cm: Target, wrist_pitch_degrees: Wrist });
    Assert.ok(App.SolveCounter.Count <= Budget, `${Name}: ${App.SolveCounter.Count} IK solves (budget ${Budget})`);
  }
  // The page thread (it once ran a 442-solve sweep there): one check of the snapped proposal.
  for (const Target of [{ x: 4.2, y: 32.5, z: 1.0 }, { x: 16.1, y: 45, z: 1 }]) {
    const Servos = Snapshot(S1610), Motion = Forced(Servos, { target_cm: Target, wrist_pitch_degrees: -90 });
    App.SolveCounter.Count = 0;
    App.Validate(Motion, 'up', Servos);
    Assert.ok(App.SolveCounter.Count <= 1, `${JSON.stringify(Target)}: ${App.SolveCounter.Count} IK solves on the page thread`);
  }
});

Test('an unreachable target names the real cause and never an untouched joint slider limit or an unverified pitch', () => {
  // 16:05: at wrist -90 the object is out of the arm's reach; every joint was far from its stop, yet the
  // model was told a slider limit was exceeded and answered "blocked by joint limits".
  const Servos = Snapshot(S1605);
  const Motion = App.ParseResponse(Reply({ target_cm: { x: 4.1, y: 32.3, z: -0.4 }, wrist_pitch_degrees: -90 }), TipOf(Servos), Servos).motion;
  const Issues = App.Validate(Motion, 'up', Servos).issues;
  Assert.ok(!Issues.some(Issue => /slider limits/.test(Issue)), Issues.join(' | '));
  Assert.ok(Issues.some(Issue => /At wrist -90 deg the arm is 2\.\d+ cm too short to reach that point/.test(Issue)), Issues.join(' | '));
  // The wrist is a preference (2026-10-03): 44 cm out at desk height, a nearly level wrist reaches within the snap's 3 cm of
  // the point, so the snap moves it there with that wrist instead of refusing it.
  const Turned = Forced(Servos, { target_cm: { x: -13.3, y: 44, z: -1.3 }, wrist_pitch_degrees: -90 });
  Assert.equal(Turned.snap.Kind, 'position');
  Assert.ok(Turned.snap.DistanceCm <= 3 && Turned.wristPitchTarget > -20 && Turned.wristPitchTarget < -10, `${Turned.snap.DistanceCm} cm, wrist ${Turned.wristPitchTarget}`);
  // Past the arm's reach at desk height no wrist pitch works: the snap refuses it (with the nearest
  // target it found, if any) instead of quoting a pitch that fails again.
  const Far = Forced(Servos, { target_cm: { x: -13.3, y: 50, z: -1.3 }, wrist_pitch_degrees: -90 });
  const FarIssues = App.Validate(Far, 'up', Servos).issues;
  Assert.equal(Far.wristPitchTarget, -90);
  Assert.equal(Far.snap.Kind, 'refused');
  Assert.ok(FarIssues.some(Issue => /nearest target the arm can reach is|found no reachable target/.test(Issue)), FarIssues.join(' | '));
  Assert.ok(!FarIssues.some(Issue => /closest is|slider limits/.test(Issue)), FarIssues.join(' | '));
});

Test('a pose blocked by a joint limit names the joint, the angle it needs and its range, from the limits the app sends with', () => {
  // A pose from TestFrontend: forward and down past the cart front.
  const Servos = structuredClone(App.Servos);
  const Pose = { shoulder_pan: 0.34323065528329266, shoulder_lift: -87.45806454570632, elbow_flex: 79.94271769861842, wrist_flex: -27.1169118284174 };
  for (const [Name, Value] of Object.entries(Pose)) Servos[Name].presentRaw = Servos[Name].targetRaw = Api.rawFromAngle(Servos[Name], Value);
  const Goal = Snapshot({ ...Object.fromEntries(Object.entries(Servos).map(([Name, Servo]) => [Name, Servo.presentRaw])), shoulder_pan: Servos.shoulder_pan.presentRaw - 34 });
  const GoalTip = Api.mirrorDisplayPoint(TipOf(Goal));
  const Motion = App.ParseResponse(Reply({ target_cm: GoalTip, wrist_pitch_degrees: TipOf(Goal).wristPitch }), TipOf(Servos), Servos).motion;
  Assert.equal(App.Validate(Motion, 'down', Servos, []).possible, true, 'Within the slider limits the move is possible');
  const Elbow = Api.currentAngles(Servos).elbow_flex;
  const Limits = { ...Api.GetIkAngleLimits(Servos), elbow_flex: [Elbow + 15, Elbow + 25] };
  const Result = App.Validate(Motion, 'down', Servos, [], Limits);
  Assert.equal(Result.possible, false);
  Assert.ok(Result.issues.some(Issue => /elbow flex would need -?\d+(\.\d)? deg, its range is -?\d+(\.\d)?\.\.-?\d+(\.\d)? deg/.test(Issue)), Result.issues.join(' | '));
  Assert.ok(!Result.issues.some(Issue => /too short/.test(Issue)), Result.issues.join(' | '));
});

Test('the floor holds the lowest point of the gripper, not only the tip', () => {
  const Servos = structuredClone(App.Servos);
  const Validate = (Target, Wrist, From = Servos) =>
    App.Validate(App.ParseResponse(Reply({ target_cm: Target, wrist_pitch_degrees: Wrist }), TipOf(From), From).motion, 'up', From);
  // A flat wrist hangs the jaw under the tip: a tip at Z -3.0 is above the floor (-3.3), the jaw is not.
  const Low = Validate({ x: -34, y: 32, z: -3.0 }, -20);
  Assert.ok(Low.preview.reachable, Low.issues.join(' | '));
  Assert.ok(Low.issues.some(Issue => /lowest point of the gripper .*below the table floor/.test(Issue)), Low.issues.join(' | '));
  // A grasp at an eraser's mid-height stays allowed at any wrist that reaches it.
  for (const Wrist of [-45, -30, -20]) {
    const Grasp = Validate({ x: -34, y: 32, z: -1.5 }, Wrist);
    Assert.ok(!Grasp.issues.some(Issue => /floor/.test(Issue)), `${Wrist}: ${Grasp.issues.join(' | ')}`);
  }
  // Already that low (a hand move), rising out of it is allowed.
  const There = structuredClone(Servos);
  for (const Joint of Api.IK_JOINTS) There[Joint].presentRaw = There[Joint].targetRaw = Api.rawFromAngle(There[Joint], Low.preview.angles[Joint]);
  const Up = Validate({ x: -34, y: 32, z: -2.8 }, -20, There);
  Assert.ok(!Up.issues.some(Issue => /floor/.test(Issue)), Up.issues.join(' | '));
});

Test('a graspable task object gets the wrist pitches the app verified, for the hover and for the grasp', () => {
  const Context = { Servos: Api.INITIAL_SERVOS, ElbowMode: 'up' };
  const Object_ = { label: 'eraser', x: -5, y: 35, z: -2.3, topZ: -0.3, heightCm: 2, sizeCm: [5, 2], depthMeasured: true, extrapolated: false,
    unstable: false, supportMeasured: true, supportZMeasured: -2.3, validFraction: 0.9, supportBranch: 'desk' };
  const Reach = Api.TaskObjectReachability(Object_, Context);
  Assert.equal(Reach.Kind, 'graspable', Reach.Text);
  Assert.match(Reach.Text, /^graspable at Z -1\.3 with wrist -?[\d.]+ deg; hover waypoint Z 4\.7 with wrist -?[\d.]+ deg \(3\d\.\d cm from the right arm's pan axis\)$/);
  const Line = Api.TaskObjectLine({ Label: 'eraser', Words: [], DetectedNow: true, LastSeen: { At: 0, Object: Object_, Reachability: Reach }, Reachability: Reach },
    { x: -14, y: 17.8, z: 5.8 });
  const Hover = JSON.parse(Line.match(/1\) hover above it: "motion_proposal":(\{"target_cm":\{[^}]*\},"wrist_pitch_degrees":-?[\d.]+\})/)[1]);
  const Descent = JSON.parse(Line.match(/descend to the grasp height Z -1\.3: "motion_proposal":(\{"move_delta_cm":\{[^}]*\},"wrist_pitch_degrees":-?[\d.]+\})/)[1]);
  const HoverTarget = { ...Api.modelPointFromDisplayPoint(Hover.target_cm), wristPitch: Hover.wrist_pitch_degrees };
  const GraspTarget = { ...Api.modelPointFromDisplayPoint({ x: Hover.target_cm.x + Descent.move_delta_cm.left,
    y: Hover.target_cm.y + Descent.move_delta_cm.farther, z: Hover.target_cm.z + Descent.move_delta_cm.up }), wristPitch: Descent.wrist_pitch_degrees };
  Assert.ok(Converged(HoverTarget, Api.INITIAL_SERVOS), `hover wrist ${Hover.wrist_pitch_degrees}`);
  Assert.ok(Converged(GraspTarget, Api.INITIAL_SERVOS), `grasp wrist ${Descent.wrist_pitch_degrees}`);
  Assert.equal(Descent.wrist_pitch_degrees, Reach.GraspPitch);
  Assert.equal(Hover.wrist_pitch_degrees, Reach.HoverPitch);
});

Test('a BLOCKED verdict says how each wrist pitch failed and does not invite a hover', () => {
  const F = (Pitch, Kind, Reason = null) => ({ Pitch, Kind, Reason });
  const Gripper = 'gripper link could contact the basket', Wrist = 'wrist link could contact the basket';
  Assert.equal(Api.ReachFailureText([F(-90, 'reach'), F(-87.5, 'reach'), F(-85, 'collision', Gripper), F(-82.5, 'collision', Gripper), F(-80, 'collision', Wrist)]),
    'out of reach at wrist -90..-87.5 deg; gripper link could contact the basket at wrist -85..-82.5 deg; wrist link could contact the basket at wrist -80 deg');
  Assert.equal(Api.ReachFailureText([F(-90, 'collision', Wrist), F(-87.5, 'collision', Wrist)]), 'wrist link could contact the basket at every wrist pitch (-90..-87.5 deg)');
  // 07:58 replay: steep pitches missed on reach, flatter ones touched the basket, yet the text blamed one link
  // "at every wrist pitch" and added "the waypoint above it is reachable" while the rules say BLOCKED means ask.
  const Eraser0758 = { label: 'eraser', x: 5.18, y: 34.69, z: -2.42, topZ: -1.19, heightCm: 1.2, sizeCm: [5, 2], depthMeasured: true, extrapolated: true,
    unstable: false, supportMeasured: true, supportZMeasured: -2.42, validFraction: 0.9, supportBranch: 'desk' };
  const Reach = Api.TaskObjectReachability(Eraser0758, { Servos: Snapshot(S0758), ElbowMode: 'up' });
  Assert.doesNotMatch(Reach.Text, /waypoint .*is reachable/, Reach.Text);
  if (Reach.Kind === 'blocked') {
    Assert.match(Reach.Text, /out of reach at wrist -90\.\./, Reach.Text);
    Assert.doesNotMatch(Reach.Text, /at every wrist pitch/, Reach.Text);
    Assert.match(Reach.Text, /\d+\.\d cm from the right arm's pan axis/, Reach.Text);
  } else Assert.equal(Reach.Kind, 'graspable', `a basket-model change may clear it: ${Reach.Text}`);
});

Test('the reach verdict finds wrist windows narrower than the old 15 deg samples', () => {
  const Context = { Servos: Api.INITIAL_SERVOS, ElbowMode: 'up' };
  let Checked = 0;
  for (const [X, Y] of [[-10, 32], [-15, 42], [-10, 42], [0, 34], [0, 40], [-15, 30]]) {
    const Point = Api.modelPointFromDisplayPoint({ x: X, y: Y, z: -1.3 });
    let Run = 0, Longest = 0, Steepest = null;
    for (let Pitch = -90; Pitch <= 0; Pitch += 1) {
      const Reached = Converged({ ...Point, wristPitch: Pitch }, Api.INITIAL_SERVOS);
      Run = Reached ? Run + 1 : 0; Longest = Math.max(Longest, Run);
      if (Reached && Steepest === null) Steepest = Pitch;
    }
    if (Longest < 4) continue; // a window of 3 deg or more always holds a 2.5 deg sample
    Checked += 1;
    const Object_ = { label: 'box', x: X, y: Y, z: -2.3, topZ: -0.3, heightCm: 2, sizeCm: [3, 3], depthMeasured: true, extrapolated: false,
      unstable: false, supportMeasured: true, supportZMeasured: -2.3, validFraction: 0.9, supportBranch: 'desk' };
    const Reach = Api.TaskObjectReachability(Object_, Context);
    Assert.equal(Reach.Kind, 'graspable', `(${X}, ${Y}): ${Reach.Text}`);
    Assert.ok(Converged({ ...Point, wristPitch: Reach.GraspPitch }, Api.INITIAL_SERVOS), `(${X}, ${Y}) wrist ${Reach.GraspPitch}`);
    Assert.ok(Reach.GraspPitch >= Steepest - 1 && Reach.GraspPitch <= Steepest + 3.5, `(${X}, ${Y}): wrist ${Reach.GraspPitch}, steepest ${Steepest}`);
  }
  Assert.ok(Checked >= 1);
});

Test('the READY-MADE hover over a tall [UNRELIABLE] object is rounded up, so the app accepts the move it hands out', () => {
  const Tall = Top => ({ label: 'bottle', x: -5, y: 30, z: -2.4, topZ: Top, heightCm: Top + 2.4, sizeCm: [4, 4], depthMeasured: true, extrapolated: true,
    unstable: false, supportMeasured: true, supportZMeasured: -2.4, validFraction: 0.9, supportBranch: 'desk' });
  for (const Top of [4.01, 4.04, 5.26, 7.93]) {
    const Line = Api.TaskObjectLine({ Label: 'bottle', Words: [], DetectedNow: true, LastSeen: { At: 0, Object: Tall(Top), Reachability: null }, Reachability: null },
      { x: -14, y: 17.8, z: 5.8 });
    const Hover = JSON.parse(Line.match(/"motion_proposal":\{"target_cm":(\{[^}]*\})/)[1]);
    Assert.ok(Hover.z >= Top + 3, `hover Z ${Hover.z} for top ${Top}`);
    Assert.equal(Api.UnreliableTargetReason(Api.modelPointFromDisplayPoint(Hover), true, Tall(Top)), null, `hover Z ${Hover.z} for top ${Top}`);
  }
  // The refusal quotes the same rounded-up height it accepts.
  Assert.match(Api.UnreliableTargetReason(Api.modelPointFromDisplayPoint({ x: -5, y: 30, z: 6.5 }), true, Tall(4.01)), /hover at Z 7\.1 or higher/);
});

Test('the system prompt says where the arm can reach, gives no stale anchor and no example wrist pitch to copy', () => {
  const Prompt = App.AiPrompt('', true, false, 'Behind');
  Assert.doesNotMatch(Prompt, /48\.7|FIXED ANCHOR|28\.9|17\.059/);
  Assert.match(Prompt, /pan axis \(its shoulder\) is at X -13\.3, Y 0, Z 4\.7/);
  Assert.match(Prompt, /wrist -90 deg \(straight down\) reaches only r <= 27 cm, -75 deg r <= 31\.5 cm, -60 deg r <= 35\.5 cm, -45 deg r <= 39 cm, -30 deg r <= 41\.5 cm, and nothing reaches beyond about 42\.5 cm/);
  // 2026-10-01 07:55 the model offered to "try with the left arm".
  Assert.match(Prompt, /Only this right arm is controlled: there is no other arm to use\./);
  Assert.doesNotMatch(Prompt, /"wrist_pitch_degrees":-?\d/, 'No example pitch for the model to copy');
  Assert.doesNotMatch(Prompt, /"z":-5\.0/, 'No example target below the floor');
  Assert.doesNotMatch(Api.MeasuredObjectsSummary(Ok), /refuses them as an absolute target_cm/);
  Assert.match(Api.MeasuredObjectsSummary(Ok), /\[UNRELIABLE\] may be off by several cm: near them target_cm is accepted only for a hover at least 3 cm above the object's top/);
});

Test('the desk reach figures the model is given match the CAD arm', () => {
  const Initial = Api.currentAngles(Api.INITIAL_SERVOS), Limits = Api.GetIkAngleLimits(Api.INITIAL_SERVOS);
  const Pan = Api.mirrorDisplayPoint({ x: 0, y: 0, z: 0 });
  const At = (R, Z) => Api.modelPointFromDisplayPoint({ x: Pan.x, y: Pan.y + R, z: Z });
  // Grasp heights of objects on the desk (Z -2.3).
  for (const Z of [-1.3, 0.7]) {
    for (const [Pitch, R] of Api.DeskReachCm) {
      Assert.ok(Api.CadSolveCandidates(At(R, Z), Pitch, 'up', Initial, Limits).some(Candidate => Candidate.Reachable), `wrist ${Pitch} at r ${R}, Z ${Z}`);
      Assert.ok(!Api.CadSolveCandidates(At(R + 1, Z), Pitch, 'up', Initial, Limits).some(Candidate => Candidate.Reachable), `wrist ${Pitch} at r ${R + 1}, Z ${Z}`);
    }
    Assert.equal(Api.CadSolve(At(Api.DeskReachMaxCm + 1, Z), 0, 'up', Initial, Limits, true).Reachable, false, `r ${Api.DeskReachMaxCm + 1}, Z ${Z}`);
  }
  Assert.equal(Api.CadSolve(At(Api.DeskReachMaxCm, -1.3), 0, 'up', Initial, Limits, true).Reachable, true);
});

// --- The task's object: read from the task, found among the measured objects --

// Measured objects of the recorded requests (back end/captures/2026-09-28 and
// 2026-09-29): label, X, Y, base Z, top Z, footprint, extrapolated, support.
const Recorded = {
  // 2026-09-28 22:19:01 UTC, "Move the gripper tip to about 5 cm directly above the red eraser on the desk." (a bridge without support_branch)
  F0928_2219: [['keyboard', 24.5, 65.77, -1.01, 2.34, [19.4, 22.5], true, null], ['eraser', 4.85, 33.31, -1.71, -0.56, [4.1, 3.4], true, null],
    ['paper', 41.04, 44.96, -1.75, 7.87, [15.3, 20], true, null], ['book', 39.23, 35.84, -1.98, 7.16, [13.7, 13.9], true, null],
    ['plastic cup', -10.88, 18.7, -8.46, 9.04, [9, 14.8], true, null], ['pliers', 9.65, 7.87, -8.1, -5.51, [20.1, 9.8], true, null],
    ['white object', 0.25, 3.11, -1.24, 4.02, [10.9, 2.7], true, null], ['black box', 10.38, 0.65, -2.17, 10.23, [2.5, 3.4], true, null],
    ['tape', 34.64, 1.5, -13.85, -8.47, [3.4, 7.3], true, null], ['mouse', 4.85, 33.3, -1.74, -0.56, [4.1, 3.1], true, null],
    ['notebook', 41.1, 45.09, -1.64, 7.89, [15.3, 20.2], true, null], ['binder', 167.87, 52.41, -175.17, -169.23, [8, 3.2], true, null],
    ['cup', -10.98, 18.82, -8.43, 9.02, [9.4, 14.8], true, null], ['rubber eraser', 4.85, 33.3, -1.72, -0.56, [4.1, 3.2], true, null],
    ['puncher', 47.37, 51.25, -2.3, 10.99, [5.8, 4.5], true, null]],
  // 2026-09-28 02:03:00 UTC, "Grab the eraser": the model's target_estimate_cm was X 36.9, Y -5.1, on the binder.
  F0928_0203: [['paper', 41.95, 43.43, -2.3, 13.3, [4, 15.2], true, null], ['book', 41.58, 42.6, -2.5, 12.89, [7.2, 26.8], true, null],
    ['cup', -5.84, 32.85, -2.3, null, null, false, null], ['cylinder', -1.75, 38.38, -5.14, 3.67, [9.6, 7.7], false, null],
    ['binder', 36.93, -5.14, -2.3, null, null, true, null], ['cable', 36.39, -0.48, -9.28, -7.48, [2.6, 4.2], true, null]],
  // 2026-09-28 02:04:52 UTC, "Grab the eraser": a stale estimate X 16.1, Y 35.8, 16 cm or more from every object.
  F0928_0204: [['robot arm', -36.93, 113.45, -79.48, -56.38, [35.9, 25.1], true, null], ['paper', 48.23, 48.16, -2.3, 11.47, [0.4, 2.1], true, null],
    ['book', 44.21, 43.38, -2, 9.58, [7.4, 20.2], true, null], ['cup', -0.14, 40.46, -2.3, 6.44, [8.7, 6.7], false, null],
    ['paper cup', 0.69, 40.14, -2.3, 6.3, [9.1, 7], false, null], ['wooden cup', 0.07, 40.35, -2.3, 6.4, [8.9, 6.9], false, null]],
  // 2026-09-28 05:48:34 UTC, "Grab the eraser".
  F0928_0548: [['keyboard', 19.88, 84.27, 2.52, 3.55, [10, 23.2], true, 'desk'], ['eraser', 3.46, 32.9, -2.7, -1.5, [4.6, 2.2], true, 'desk'],
    ['cup', -11.02, 30.95, -3.21, -1.16, [5.3, 8], true, 'desk'], ['pliers', 4.35, 9.26, -8.74, -6.92, [15.7, 7.5], true, 'bin'],
    ['notebook', 40.43, 42.19, -3.13, 7.02, [14, 18], true, 'desk']],
  // 2026-09-29 23:01:53 UTC, "Grab the black thing": the model replied ask, 'The task object "thing" is not detected'.
  F0929_2301: [['keyboard', 18.4, 84.32, 3.43, 8.91, [13.4, 23.8], true, 'desk'], ['phone', 2.41, 95.49, 4.19, 5.49, [10.5, 5.3], true, 'desk'],
    ['usb drive', 4.11, 32.33, -2.58, -0.42, [2, 1.6], true, 'desk'], ['cup', -9.15, 22.16, -2.94, 0.41, [4.5, 9.8], true, 'desk'],
    ['pliers', 5.8, 9.04, -8.72, -6.84, [17.1, 7.9], true, 'bin'], ['cable', 6.14, 8.98, -8.66, -6.86, [17.6, 8.1], true, 'bin'],
    ['box', 37.26, 32.84, -3.45, 4.22, [8.5, 13.3], true, 'desk'], ['bowl', -9.56, 22.25, -2.89, 0.44, [4.9, 10.1], true, 'desk'],
    ['wires', 5.89, 9.04, -8.63, -6.83, [17.3, 8], true, 'bin'], ['blue box', 37.22, 32.78, -3.45, 4.2, [8.5, 13.2], true, 'desk'],
    ['cables', 6.32, 8.95, -8.58, -6.83, [17.9, 8.1], true, 'bin'], ['monitor stand', 38.96, 79.19, 1.76, 12.37, [14.5, 16.3], true, 'desk']],
  // 2026-09-29 23:05:19 UTC, "Grab the black thingon the desk": the estimate X 4.1, Y 32.3 is 0.03 cm from the USB drive.
  F0929_2305: [['keyboard', 26.25, 91.31, 3.42, 4.55, [10.2, 23.8], true, 'desk'], ['USB drive', 4.07, 32.29, -2.59, -0.38, [2, 1.4], true, 'desk'],
    ['cup', -8.6, 22.17, -3.04, 0.33, [3.5, 9], true, 'desk'], ['pliers', 6.88, 8.95, -8.65, -6.79, [18.3, 8], true, 'bin'],
    ['cable', 6.33, 9.01, -8.63, -6.84, [17.7, 8], true, 'bin'], ['box', 37.25, 32.76, -3.43, 4.22, [8.5, 13.1], true, 'desk']],
  // 2026-09-29 23:09:51 UTC, the same task's check-in: the USB drive is now "small black object".
  F0929_2309: [['keyboard', 26.21, 91.53, 3.56, 4.67, [10.4, 23.9], true, 'desk'], ['notebook', 26.22, 91.55, 3.57, 4.68, [10.4, 24], true, 'desk'],
    ['cup', 7.22, 95.88, 4.34, 16.07, [7.4, 6.3], true, 'desk'], ['wrench', 7.41, 8.87, -8.7, -6.72, [17.3, 8.1], true, 'bin'],
    ['cable', 6.47, 9.01, -8.63, -6.8, [16, 7.3], true, 'bin'], ['box', 37.24, 32.71, -3.34, 4.29, [8.4, 13.1], true, 'desk'],
    ['pliers', 6.99, 8.93, -8.71, -6.75, [16.6, 8], true, 'bin'], ['mug', 7.32, 95.88, 4.42, 16.19, [6.9, 6.9], true, 'desk'],
    ['small black object', 4.23, 32.5, -2.52, -0.18, [1.4, 2.3], true, 'desk']],
};
// A recorded frame as the app reads it from /locate; Matched: the labels the bridge's strict task query landed on.
const Frame = (Rows, Matched = []) => Api.ReadLocateResponse({ ok: true, camera_check: Check(), objects: Rows.map(([label, x, y, z, top_z, size_cm, extrapolated, support_branch]) =>
  ({ label, x, y, z, top_z, height_cm: top_z == null ? null : top_z - z, size_cm, method: top_z == null ? 'table' : 'depth', extrapolated, support_branch,
    task_match: Matched.includes(label), task_query_only: false })) }).objects;
const Parse = Task => Api.ParseTaskObject(Task);
const Bind = (Task, Objects, Options) => Api.ResolveTaskObject(Parse(Task), Objects, Options).Binding;

Test('the task object is read through verb typos, glued words, generic nouns and arm-motion phrasing', () => {
  // The 10 distinct real phrases of 09-28/29 among them: "thingon" was the label, and "Grabe" and the hover task had none.
  const Cases = [
    ['Grab the eraser', { Head: 'eraser', Generic: false, Name: 'eraser', Phrase: 'eraser', Support: null, VerbFuzzy: false }],
    ['Grabe the eraser', { Head: 'eraser', VerbFuzzy: true }],
    ['Move the gripper tip to about 5 cm directly above the red eraser on the desk.',
      { Head: 'eraser', Colours: ['red'], Support: 'desk', Name: 'red eraser', Phrase: 'red eraser on the desk' }],
    ['Grab the black thing', { Head: null, Generic: true, Colours: ['black'], Name: 'black thing', Phrase: 'black thing' }],
    ['Grab the black thingon the desk', { Head: null, Generic: true, Colours: ['black'], Support: 'desk', Name: 'black thing',
      Phrase: 'black thing on the desk', Repaired: ['thingon -> thing on'] }],
    ['grab the black thing on the desk', { Head: null, Generic: true, Phrase: 'black thing on the desk' }],
    ['grab the black one', { Head: null, Generic: true, Colours: ['black'], Phrase: 'black thing' }],
    ['grab the small black object', { Head: null, Generic: true, Sizes: ['small'], Colours: ['black'], Phrase: 'small black thing' }],
    ['grab theeraser', { Head: 'eraser', Repaired: ['theeraser -> the eraser'] }],
    ['Grab the eraseron the desk', { Head: 'eraser', Support: 'desk' }],
    // Real words are never split: "cart on", "butt on".
    ['grab the carton', { Head: 'carton', Repaired: [] }],
    ['Press the button', { Head: 'button', Repaired: [] }],
    ['grb the eraser', { Head: 'eraser', VerbFuzzy: true }],
    ['pickk up the cup', { Head: 'cup', VerbFuzzy: true }],
    ['hover over the eraser', { Head: 'eraser' }],
    ['go 5 cm above the cup', { Head: 'cup' }],
    ['Position the gripper above the eraser', { Head: 'eraser' }],
    ['point the gripper at the cup', { Head: 'cup' }],
    ['move the cup to the left', { Head: 'cup' }],
    ['open your jaws and grab the eraser', { Head: 'eraser' }],
    ['open the gripper around the eraser', { Head: 'eraser' }],
    ['grab the eraser in the bin', { Head: 'eraser', Support: 'bin', Phrase: 'eraser in the bin' }],
    ['Take the pen from the bin', { Head: 'pen', Support: 'bin' }],
    // Where it goes is not where it is: the desk eraser must stay bindable.
    ['Put the eraser in the bin', { Head: 'eraser', Support: null, Phrase: 'eraser' }],
    ['Drop the pen in the basket', { Head: 'pen', Support: null }],
    ['Grab the red block', { Head: 'block', Generic: false }],
    ['grab the usb thing', { Head: 'usb', Generic: false, Name: 'usb thing' }],
  ];
  for (const [Task, Expected] of Cases) {
    const Query = Parse(Task);
    Assert.ok(Query, Task);
    for (const [Key, Value] of Object.entries(Expected)) Assert.deepEqual(Query[Key], Value, `${Task}: ${Key}`);
    Assert.equal(Api.TaskNeedsPositions(Task), true, Task);
  }
  for (const Task of ['move the gripper to the left', 'move up 3 cm', 'move 3 cm up', 'lower the arm 2 cm', 'open the gripper wider', 'go home',
    'move to the center', 'grab this', 'touch this', 'take this', 'Take a picture', 'put it in the bin']) {
    Assert.equal(Parse(Task), null, Task);
  }
  Assert.equal(Api.TaskObjectNoun('Grab the black thingon the desk'), 'black thing on the desk');
  Assert.equal(Api.TaskObjectNoun('Grabe the eraser'), 'eraser');
  // The bridge takes a phrase of at most 80 characters.
  Assert.equal(Parse(`grab the ${'q'.repeat(100)}`).Phrase.length, 80);
});

Test('gripper-only and description tasks are not positional, and a task that names no object stays ungated', () => {
  // Review: plural gripper words escaped the filter, and "open" made gripper questions need measured positions.
  for (const Task of ['open your jaws', 'open the jaws', 'open the claws', 'open your claw', 'is the gripper open?', 'gripper open',
    'open and close the gripper', 'open the gripper wider', 'open wide', 'raise your hand', 'wave your hand',
    'tell me whether the gripper is open or closed', 'Open the gripper', 'Close the gripper', 'is your hand open?']) {
    Assert.equal(Api.TaskNeedsPositions(Task), false, Task);
    Assert.equal(Api.TaskObjectNoun(Task), null, Task);
  }
  // A word one letter from a verb reads as that verb only when the task has no verb, the word is not an
  // ordinary one in its place ("the plate", "told"), and the task is not a question or a description.
  for (const Task of ['Describe the plate next to the cup', 'How tall is the plate cup', 'I told you the cup is red', 'Describe the oven mitt',
    'Describe the workspace', 'Describe the objects you see on the desk and how tall they are. Do not move the arm.', 'What do you see?', 'Wave',
    'Hello', 'How tall is the cup?', 'Where is the eraser?', 'Is the black thing reachable?', 'Open the gripper', 'Close the gripper', 'go home',
    'move up 3 cm', 'move left 2 cm', 'rotate the wrist', 'tilt the wrist down', 'count the objects', 'Descirbe the workspace', 'Wht do you see',
    'mvoe up 3 cm', 'opne the gripper', 'clsoe the gripper', 'Respond without moving.', 'Tell me a joke', 'lower the arm 2 cm', 'raise the gripper',
    'Look at the desk', 'I told you to stop', 'Stop', 'Relax the arm', 'Say hi', 'what is the tallest thing', 'thanks']) {
    Assert.equal(Api.TaskNeedsPositions(Task), false, Task);
  }
  // The same 34 tasks gated before and after (these two by their verb, as before).
  for (const Task of ['Take a picture', 'take a photo of the desk']) Assert.equal(Api.TaskNeedsPositions(Task), true, Task);
});

Test('a /locate reply carries the task query\'s matches; an object only the task query found is [UNRELIABLE]', () => {
  const Reply_ = Api.ReadLocateResponse({ ok: true, camera_check: Check(), objects: [{ ...Cup, task_match: true },
    { ...Cup, label: 'black thing', x: 4, y: 32, task_match: true, task_query_only: true }, Eraser] });
  Assert.deepEqual(Reply_.objects.map(Object_ => [Object_.taskMatch, Object_.taskQueryOnly]), [[true, false], [true, true], [false, false]]);
  Assert.equal(Api.IsUnreliable(Reply_.objects[0]), false);
  // One detection at one temperature, not confirmed across the samples: a hover above it, never a descent onto it.
  Assert.equal(Api.IsUnreliable(Reply_.objects[1]), true);
  Assert.match(Api.MeasuredObjectsSummary(Reply_), /- black thing: [^\n]*\[UNRELIABLE: found only by the task query, in one detection/);
  Assert.match(Api.UnreliableTargetReason(Api.modelPointFromDisplayPoint({ x: 4, y: 32, z: 2 }), true, Reply_.objects[1]), /\[UNRELIABLE\] black thing/);
});

Test('an object the bridge did not zoom-refine is [UNRELIABLE] for descents unless it is the task object; the calibrated-area tag says how far out', () => {
  // Told the task object, the bridge refines only its boxes: the others keep the first pass's box, which can be several cm off.
  const Located = Api.ReadLocateResponse({ ok: true, camera_check: Check(), objects: [
    { ...Cup, label: 'usb drive', x: 4.1, y: 32.3, refined: true, task_match: true }, { ...Cup, refined: false },
    { ...Eraser, label: 'pen', x: -5, y: 30, outside_calibration_cm: 11.2, refined: true },
    { ...Cup, label: 'mug', x: -12, y: 28, extrapolated: false, outside_calibration_cm: 7.6 }] });
  const [Usb, CupNow, Pen, Mug] = Located.objects;
  Assert.deepEqual(Located.objects.map(Object_ => Object_.refined), [true, false, true, null], 'null: a bridge that does not say');
  Assert.equal(Api.IsUnreliable(CupNow), true);
  Assert.equal(Api.IsUnreliable(CupNow, CupNow), false, 'The task object is judged by its own measurement');
  Assert.equal(Api.IsUnreliable(Usb), false);
  Assert.equal(Api.IsUnreliable(Mug), false, 'Within the 10 cm margin of the calibration');
  const Summary = Api.MeasuredObjectsSummary(Located, Usb);
  Assert.match(Summary, /- cup: [^\n]*\[UNRELIABLE: coarse box, not refined: may be off by several cm\]/);
  Assert.match(Summary, /- pen: [^\n]*\[UNRELIABLE: 11\.2 cm outside the camera-calibrated area: may be off by several cm\]/);
  Assert.doesNotMatch(Summary, /- (usb drive|mug): [^\n]*UNRELIABLE/);
  // Without a distance (an older bridge, or a fit with no tip region) the tag reads as before.
  Assert.match(Api.MeasuredObjectsSummary(Ok), /- eraser: [^\n]*\[UNRELIABLE: outside the camera-calibrated area: may be off/);
  Assert.doesNotMatch(Api.MeasuredObjectsSummary(Located, CupNow), /- cup: [^\n]*UNRELIABLE/);
  const Line = (Object_, Label) => Api.TaskObjectLine({ Label, DetectedNow: true, LastSeen: { At: 0, Object: Object_, Reachability: null }, Reachability: null });
  Assert.doesNotMatch(Line(CupNow, 'cup'), /UNRELIABLE/);
  Assert.match(Line(Pen, 'pen'), /\[UNRELIABLE: 11\.2 cm outside the camera-calibrated area\]/);
  // Like every [UNRELIABLE] measurement: a target_cm hover above it is accepted, a descent onto it is not.
  const Validate = (Target, Bound) => {
    const Motion = App.ParseResponse(Reply({ target_cm: Target }), App.HomeTarget, App.Servos).motion;
    const State = { Label: Bound.label, DetectedNow: true, Binding: { Object: Bound, Source: 'name', DistanceCm: null },
      LastSeen: { At: 0, Object: Bound, Reachability: null }, Reachability: null };
    return App.Validate(Motion, 'up', App.Servos, Api.UnreliableMeasurements(Located.objects, State)).issues.filter(Issue => /UNRELIABLE/.test(Issue));
  };
  Assert.match(Validate({ x: 0.2, y: 40.8, z: 2 }, Usb).join(' | '), /descends onto the \[UNRELIABLE\] cup measurement/);
  Assert.deepEqual(Validate({ x: 0.2, y: 40.8, z: 9.7 }, Usb), [], 'A hover 3 cm above its top');
  Assert.deepEqual(Validate({ x: 0.2, y: 40.8, z: 2 }, CupNow), [], 'The task object\'s own refine is not held against it');
  Assert.deepEqual(Api.UnreliableMeasurements(Located.objects, null), [CupNow, Pen]);
  // Of two labels at one spot, the refined one is the task object.
  const Pair = Api.ReadLocateResponse({ ok: true, camera_check: Check(), objects: [{ ...Cup, refined: false }, { ...Cup, label: 'white cup', x: 0.6, refined: true }] }).objects;
  Assert.equal(Api.ResolveTaskObject(Api.ParseTaskObject('Grab the cup'), Pair).Binding.Object, Pair[1]);
});

Test('recorded frames: the task object binds by name, synonym, typo, the task query, position or its colour, never to a clashing or distant object', () => {
  const F21 = Frame(Recorded.F0928_0548), F27 = Frame(Recorded.F0929_2301), F28 = Frame(Recorded.F0929_2305), F29 = Frame(Recorded.F0929_2309);
  Assert.equal(Bind('Grab the eraser', F21).Object.label, 'eraser');
  Assert.equal(Bind('Grab the eraser', F21).Source, 'name');
  Assert.equal(Bind('pick up the rubber', F21).Object.label, 'eraser', 'a synonym');
  Assert.equal(Bind('grab the earser', F21).Object.label, 'eraser', 'a one-letter typo');
  Assert.equal(Bind('Grabe the eraser', F21).Object.label, 'eraser', '05:54: a typo in the verb');
  Assert.equal(Bind('Grab the eraser', Frame(Recorded.F0928_0548, ['eraser'])).Source, 'query');
  Assert.equal(Bind('Move the gripper tip to about 5 cm directly above the red eraser on the desk.', Frame(Recorded.F0928_2219)).Object.label, 'eraser');
  // "the black thing": no label says black, so it is unbound with the candidates listed; the task query picks the USB drive.
  const Black = Api.ResolveTaskObject(Parse('Grab the black thing'), F27);
  Assert.equal(Black.Binding, null);
  Assert.ok(Black.Candidates.length > 1);
  Assert.ok(Black.Candidates.some(Object_ => Object_.label === 'usb drive'));
  Assert.ok(!Black.Candidates.some(Object_ => /box/.test(Object_.label)), 'The blue box, and the box measured at its spot, are not black');
  const Queried = Bind('Grab the black thing', Frame(Recorded.F0929_2301, ['usb drive']));
  Assert.equal(Queried.Object.label, 'usb drive');
  Assert.equal(Queried.Source, 'query');
  // "the black box": the only box is the blue one (the plain "box" is measured at the same spot), so no name bind.
  Assert.equal(Bind('grab the black box', F27), null);
  Assert.equal(Bind('grab the black box', Frame(Recorded.F0929_2301, ['usb drive'])).Object.label, 'usb drive');
  // 23:05 with the task query; 23:09 by the one label that says black.
  Assert.equal(Bind('Grab the black thingon the desk', Frame(Recorded.F0929_2305, ['USB drive'])).Object.label, 'USB drive');
  const Coloured = Bind('Grab the black thingon the desk', F29);
  Assert.equal(Coloured.Object.label, 'small black object');
  Assert.equal(Coloured.Source, 'colour');
  // Renamed between check-ins ("USB drive" -> "small black object"): the same spot and size is the same object.
  const Usb = F28.find(Object_ => Object_.label === 'USB drive');
  Assert.equal(Bind('grab the usb drive', F28).Object, Usb);
  const Tracked = Bind('grab the usb drive', F29, { LastSeen: Usb });
  Assert.equal(Tracked.Object.label, 'small black object');
  Assert.equal(Tracked.Source, 'position');
  Assert.ok(Tracked.DistanceCm < 0.3, `${Tracked.DistanceCm} cm`);
  Assert.equal(Bind('grab the usb drive', F29, { LastSeen: { ...Usb, x: Usb.x + 6.2 } }), null, 'Nothing within 3 cm: not tracked');
  Assert.equal(Bind('grab the usb drive', F29, { LastSeen: Usb, Track: false }), null, 'Not tracked once the gripper closed (it may be in the jaws)');
  Assert.equal(Bind('grab the usb drive', F29, { LastSeen: { ...Usb, z: Usb.z + 6 } }), null, 'A box 6 cm higher (on the jaws) is not it');
  Assert.equal(Bind('grab the usb drive', F29, { LastSeen: { ...Usb, sizeCm: [0.2, 0.2] } }).Object.label, 'small black object');
  Assert.equal(Bind('grab the usb drive', [{ ...F29.at(-1), sizeCm: [14, 20] }], { LastSeen: Usb }), null, 'A far larger footprint is another object');
  // The bin is not the desk.
  Assert.equal(Bind('grab the eraser in the bin', F21), null);
  Assert.equal(Bind('grab the pliers in the bin', F21).Object.label, 'pliers');
  Assert.equal(Bind('grab the pliers on the desk', F21), null);
  // A synonym or typo never drops a word of the phrase: a glue stick is not the USB drive.
  Assert.equal(Bind('grab the glue stick', F27), null);
  Assert.equal(Bind('grab the usb stick', F27).Object.label, 'usb drive');
});

Test('the task query does not override a named object with a differently named one, and two spots are ambiguous', () => {
  const Eraser_ = { ...Ok.objects[0], extrapolated: false }, Mouse = { ...Ok.objects[1], label: 'mouse', x: -12, y: 30, taskMatch: true };
  // The strict query landed on the "mouse" for "red eraser on the desk" (09-28, bea44218).
  Assert.equal(Bind('Grab the red eraser', [Eraser_, Mouse]).Object.label, 'eraser');
  // Named at two places: ambiguous until one was seen, then the nearer one.
  const Other = { ...Eraser_, x: -10, y: 25 };
  const Two = Api.ResolveTaskObject(Parse('Grab the eraser'), [Eraser_, Other]);
  Assert.equal(Two.Binding, null);
  Assert.equal(Two.Candidates.length, 2);
  Assert.equal(Bind('Grab the eraser', [Eraser_, Other], { LastSeen: { ...Other, x: -9.5 } }).Object, Other);
  // The query on two separate spots picks neither.
  Assert.equal(Bind('Grab the black thing', [{ ...Eraser_, label: 'thing', taskMatch: true }, { ...Other, label: 'object', taskMatch: true }]), null);
  // A colour label in the bin is not the desk task's black thing; nor is the robot's own arm.
  const Bin = [{ ...Eraser_, label: 'usb drive' }, { ...Other, label: 'black cable', supportBranch: 'bin' }];
  Assert.equal(Bind('Grab the black thing', Bin), null);
  Assert.equal(Bind('Grab the black thing in the bin', Bin).Object.label, 'black cable');
  const Arm = Api.ResolveTaskObject(Parse('Grab the black thing'), Frame(Recorded.F0928_0204));
  Assert.ok(!Arm.Candidates.some(Object_ => /robot/.test(Object_.label)), Arm.Candidates.map(Object_ => Object_.label).join());
  Assert.equal(Bind('grab the robot arm', Frame(Recorded.F0928_0204)), null);
});

Test('the model\'s target_estimate_cm binds the one measured object within 3 cm that the task could mean', () => {
  const Generic = Parse('Grab the black thingon the desk');
  const Estimated = Api.BindFromEstimate(Generic, Frame(Recorded.F0929_2305), { x: 4.1, y: 32.3, z: -0.4 });
  Assert.equal(Estimated.Object.label, 'USB drive');
  Assert.equal(Estimated.Source, 'estimate');
  Assert.ok(Estimated.DistanceCm < 0.1, `${Estimated.DistanceCm} cm`);
  Assert.equal(Api.BindFromEstimate(Generic, Frame(Recorded.F0929_2309), { x: 4.2, y: 32.5, z: -1.35 }).Object.label, 'small black object');
  // 02:03: an eraser task never binds the binder the model pointed at; 02:04: a stale estimate far from everything.
  Assert.equal(Api.BindFromEstimate(Parse('Grab the eraser'), Frame(Recorded.F0928_0203), { x: 36.9, y: -5.1, z: -2.3 }), null);
  Assert.equal(Api.BindFromEstimate(Parse('Grab the eraser'), Frame(Recorded.F0928_0204), { x: 16.1, y: 35.8, z: -2.3 }), null);
  // A label that names it, or a generic one, is fine for a concrete noun.
  Assert.equal(Api.BindFromEstimate(Parse('grab the usb drive'), Frame(Recorded.F0929_2309), { x: 4.2, y: 32.5, z: 0 }).Object.label, 'small black object');
  Assert.equal(Api.BindFromEstimate(Parse('Grab the eraser'), Frame(Recorded.F0928_0548), { x: 3.5, y: 32.9, z: -2.1 }).Object.label, 'eraser');
  // A clashing colour, two objects within the gate, or the bin for a desk task: no bind.
  Assert.equal(Api.BindFromEstimate(Parse('grab the red thing'), Frame(Recorded.F0929_2309), { x: 4.2, y: 32.5, z: 0 }), null);
  Assert.equal(Api.BindFromEstimate(Generic, [...Frame(Recorded.F0929_2305), { ...Frame(Recorded.F0929_2305)[1], label: 'pen', x: 6.5, y: 32.3 }], { x: 5.2, y: 32.3, z: 0 }), null);
  Assert.equal(Api.BindFromEstimate(Generic, Frame(Recorded.F0929_2305), { x: 6.3, y: 9, z: -7 }), null);
  Assert.equal(Api.BindFromEstimate(Generic, Frame(Recorded.F0929_2305), null), null);
});

Test('the TASK OBJECT line says how the object was picked, or lists the candidates when no label names it', () => {
  const Query = Parse('Grab the black thing');
  const F27 = Frame(Recorded.F0929_2301);
  const Unbound = Api.ResolveTaskObject(Query, F27);
  const State = { Label: Query.Name, Query, DetectedNow: false, Binding: null, Candidates: Unbound.Candidates, LastSeen: null, Reachability: null };
  const Line = Api.TaskObjectLine(State);
  Assert.match(Line, /^TASK OBJECT "black thing": no measured label names it\. Candidates measured this check-in: [^\n]*usb drive \(X 4\.1, Y 32\.3\)/);
  Assert.match(Line, /cup \(X -9\.2, Y 22\.2\)/);
  Assert.match(Line, /If one of them is it, set target_estimate_cm to its measured centre and the app will track it and check its reach; otherwise reply ask\.$/);
  Assert.doesNotMatch(Line, /box/);
  // A concrete noun with nothing measured keeps the plain instruction.
  Assert.match(Api.TaskObjectLine({ ...State, Label: 'eraser', Query: Parse('Grab the eraser'), Candidates: [] }),
    /^TASK OBJECT "eraser": NOT detected in any check-in of this task\. Do not guess its coordinates/);
  // Named at two places.
  const Two = Api.TaskObjectLine({ ...State, Label: 'eraser', Query: Parse('Grab the eraser'),
    Candidates: [{ ...Ok.objects[0], extrapolated: false }, { ...Ok.objects[0], extrapolated: false, x: -10, y: 25 }] });
  Assert.match(Two, /^TASK OBJECT "eraser": more than one measured object could be it\. Candidates measured this check-in: eraser \(X -10\.0, Y 25\.0\), eraser \(X 16\.1, Y 35\.8\)\./);
  // Picked by the task query, tracked at the same spot, and tagged in the measured list.
  const Matched = Frame(Recorded.F0929_2301, ['usb drive']);
  const Picked = Api.ResolveTaskObject(Query, Matched).Binding;
  const Detected = Api.TaskObjectLine({ ...State, DetectedNow: true, Binding: Picked, Candidates: [], LastSeen: { At: 0, Object: Picked.Object, Reachability: null } });
  Assert.match(Detected, /^TASK OBJECT "black thing": detected this check-in \(measured as "usb drive", picked by the task query\) at X 4\.1, Y 32\.3, base Z -2\.6/);
  const Track = Bind('grab the usb drive', Frame(Recorded.F0929_2309), { LastSeen: Frame(Recorded.F0929_2305)[1] });
  Assert.match(Api.TaskObjectLine({ ...State, Label: 'usb drive', DetectedNow: true, Binding: Track, LastSeen: { At: 0, Object: Track.Object, Reachability: null } }),
    /^TASK OBJECT "usb drive": detected this check-in \(measured as "small black object", tracked at the same spot\) at X 4\.2, Y 32\.5/);
  const Named = Bind('Grab the eraser', Ok.objects);
  Assert.match(Api.TaskObjectLine({ ...State, Label: 'eraser', DetectedNow: true, Binding: Named, LastSeen: { At: 0, Object: Named.Object, Reachability: null } }),
    /^TASK OBJECT "eraser": detected this check-in at X 16\.1/, 'Its own name needs no note');
  const Located = Api.ReadLocateResponse({ ok: true, camera_check: Check(), objects: [{ ...Cup, label: 'usb drive', x: 4.11, y: 32.33, task_match: true }, Cup] });
  const Summary = Api.MeasuredObjectsSummary(Located, Located.objects[0]);
  Assert.match(Summary, /- usb drive: centre X=4\.1, Y=32\.3[^\n]* <- the task object\n/);
  Assert.equal(Summary.match(/<- the task object/g).length, 1);
  Assert.doesNotMatch(Api.MeasuredObjectsSummary(Located), /<- the task object/);
});

// 2026-09-29 22:46-22:47: the model hovered over the USB drive, descended with
// the jaws closed onto it, and the task query then re-bound the task to the
// mouse 72 cm away. The grasp is now driven one step at a time.
const Usb = { label: 'usb drive', x: 4.3, y: 32.9, z: -2.5, topZ: -0.4, heightCm: 2.1, sizeCm: [1.7, 1.8], method: 'depth',
  extrapolated: false, unstable: false, refined: true, taskMatch: true, taskQueryOnly: false };
const UsbReach = { Kind: 'graspable', Text: 'graspable at Z -1.4 with wrist -54.6 deg', BaseDistanceCm: 37.3, GraspPitch: -54.6, HoverPitch: -50.6 };
const UsbState = { Label: 'black thing', DetectedNow: true, LastSeen: { At: 0, Object: Usb, Reachability: UsbReach }, Reachability: UsbReach };
const Step = (Tip, Fraction, Closed = false, State = UsbState) =>
  Api.TaskObjectLine(State, Tip, { GripperFraction: Fraction, Closed });
// The first motion_proposal JSON object in a line, by matching braces.
const MotionIn = Line => {
  const Start = Line.indexOf('"motion_proposal":') + '"motion_proposal":'.length;
  let Depth = 0;
  for (let Index = Start; Index < Line.length; Index++) {
    Depth += Line[Index] === '{' ? 1 : Line[Index] === '}' ? -1 : 0;
    if (Depth === 0) return JSON.parse(Line.slice(Start, Index + 1));
  }
  throw new Error(`no motion_proposal in ${Line}`);
};

Test('the grasp is driven one step at a time: hover, open, descend, close, lift, done', () => {
  // Closed jaws open first, so they are open on the way in: far away, the opening and the hover are ONE
  // chained motion (lib/AiChain.ts); over it already, the opening alone.
  const OpenAndHover = Step({ x: -14.8, y: 20.1, z: 12.1 }, 0.034);
  Assert.match(OpenAndHover, /NEXT STEP of the grasp .*gripper 3% open\): open the gripper to 80% and hover above it as ONE chained motion \(the gripper opens while the tip moves\)/);
  Assert.deepEqual(MotionIn(OpenAndHover), { target_cm: { x: 4.3, y: 32.9, z: 4.6 }, wrist_pitch_degrees: -50.6, gripper_before: 0.8 });
  Assert.match(OpenAndHover, /Copy it exactly: it is ONE motion_proposal, and the gripper opens while the tip moves/);
  const Open = Step({ x: 3.7, y: 33.0, z: 3.9 }, 0.034);
  Assert.match(Open, /NEXT STEP of the grasp .*gripper 3% open\): open the gripper before approaching it/);
  Assert.deepEqual(MotionIn(Open), { direct_servo_targets: [{ name: 'gripper', target_fraction: 0.8 }] });
  // Low beside it, the opening is never chained (the moving jaw would swing toward it).
  Assert.deepEqual(MotionIn(Step({ x: 7.3, y: 32.9, z: -1 }, 0.034)), { direct_servo_targets: [{ name: 'gripper', target_fraction: 0.8 }] });
  const Hover = Step({ x: -14.8, y: 20.1, z: 12.1 }, 0.8);
  Assert.deepEqual(MotionIn(Hover), { target_cm: { x: 4.3, y: 32.9, z: 4.6 }, wrist_pitch_degrees: -50.6 });
  // Open over it, a grasp with no GRASP PLAN (this state has none) does not descend: the GRASP RULE says reply ask, and the
  // guard refuses that descent (2026-10-01 review: the step offered it, and the NEXT STEP RULE now outranks the GRASP RULE).
  const NoPlan = Step({ x: 3.7, y: 33.0, z: 3.9 }, 0.8);
  Assert.match(NoPlan, /NEXT STEP of the grasp \(tip now X 3\.7, Y 33\.0, Z 3\.9; gripper 80% open\): reply ask: the app has no GRASP PLAN for it, checked against the gripper's shape, so the jaws do not descend beside or onto it\./);
  Assert.doesNotMatch(NoPlan, /"motion_proposal"/);
  // A contact task (push, slide, touch) descends from where the tip is, not from the printed hover height it missed.
  const Descend = Step({ x: 3.7, y: 33.0, z: 3.9 }, 0.8, false, { ...UsbState, Intent: 'contact' });
  const Down = MotionIn(Descend);
  Assert.equal(Down.wrist_pitch_degrees, -54.6);
  Assert.ok(Math.abs(3.9 + Down.move_delta_cm.up - -1.4) <= 0.11, JSON.stringify(Down));
  Assert.equal(Down.move_delta_cm.left, 0.6);
  // A descent that stopped above the grasp height but below the object's top: the jaws are around it, close.
  Assert.match(Step({ x: 4.2, y: 33.0, z: -0.8 }, 0.8), /close the gripper/);
  const Close = Step({ x: 4.2, y: 33.0, z: -1.1 }, 0.8);
  // The close and the straight lift after it are ONE chained motion: the gripper closes first, then the tip rises.
  Assert.match(Close, /close the gripper and lift as ONE chained motion/);
  Assert.deepEqual(MotionIn(Close), { move_delta_cm: { left: 0, farther: 0, up: 6 }, gripper_before: 0 });
  // A contact task (push, slide, touch) only closes: chained with the lift, the close would pick the object up.
  const Push = Step({ x: 4.2, y: 33.0, z: -1.1 }, 0.8, false, { ...UsbState, Intent: 'contact' });
  Assert.match(Push, /the jaws are around it at the grasp height; close the gripper: "motion_proposal"/);
  Assert.doesNotMatch(Push, /chained|gripper_before|lift/);
  Assert.deepEqual(MotionIn(Push), { direct_servo_targets: [{ name: 'gripper', target_fraction: 0 }] });
  // After the close: stopped partway holds it; lift, then done. Closed all the way missed it.
  Assert.deepEqual(MotionIn(Step({ x: 4.2, y: 33.0, z: -1.1 }, 0.25, true)), { move_delta_cm: { left: 0, farther: 0, up: 6 } });
  Assert.match(Step({ x: 4.2, y: 33.0, z: 4.9 }, 0.25, true), /it is lifted\. If the image shows it in the jaws, reply task_status "done"/);
  Assert.match(Step({ x: 4.2, y: 33.0, z: -1.1 }, 0.034, true), /closed all the way, so they missed it\. Open the gripper/);
});

Test('the grasp steps go on while the object is hidden under the gripper, not once the gripper moved away', () => {
  const Hidden = { ...UsbState, DetectedNow: false, Reachability: null };
  Assert.match(Step({ x: 3.7, y: 33.0, z: 3.9 }, 0.034, false, Hidden), /NOT detected this check-in .*NEXT STEP of the grasp .*open the gripper before approaching it/);
  Assert.doesNotMatch(Step({ x: -10, y: 20, z: 10 }, 0.034, false, Hidden), /NEXT STEP/);
  // Carried in the jaws after a close, wherever the tip is.
  Assert.match(Step({ x: -10, y: 20, z: 10 }, 0.25, true, Hidden), /it is lifted/);
});

Test('a descent onto the task object with the jaws closed is refused with the open step; lifts, moves away and open jaws are not', () => {
  const Model = Point => Api.modelPointFromDisplayPoint(Point);
  const Closed = { GripperFraction: 0.034, Closed: false };
  Assert.match(Api.ClosedJawDescentReason(Model({ x: 3.7, y: 33.0, z: -1.4 }), -5.3, Usb, Closed),
    /gripper is closed \(3% open\): open it before descending onto the usb drive: "motion_proposal":\{"direct_servo_targets":\[\{"name":"gripper","target_fraction":0\.8\}\]\}/);
  Assert.equal(Api.ClosedJawDescentReason(Model({ x: 3.7, y: 33.0, z: -1.4 }), -5.3, Usb, { GripperFraction: 0.8, Closed: false }), null);
  Assert.equal(Api.ClosedJawDescentReason(Model({ x: 3.7, y: 33.0, z: 9.9 }), 6, Usb, Closed), null, 'lifting');
  Assert.equal(Api.ClosedJawDescentReason(Model({ x: -8, y: 30, z: -1.4 }), -5.3, Usb, Closed), null, 'away from the object');
  Assert.equal(Api.ClosedJawDescentReason(Model({ x: 3.7, y: 33.0, z: 4.6 }), -1, Usb, Closed), null, 'a hover over it');
  Assert.equal(Api.ClosedJawDescentReason(Model({ x: 3.7, y: 33.0, z: -1.4 }), -5.3, Usb, { GripperFraction: 0.25, Closed: true }), null, 'carrying it after a close');
});

Test('a measured task object is never re-bound to a match far from where it was last measured (the mouse at 72 cm)', () => {
  const Query = Api.ParseTaskObject('grab the black thing on the desk');
  const Mouse = { label: 'mouse', x: 7.0, y: 105.1, z: 5.3, topZ: 8.7, sizeCm: [7.7, 4.3], method: 'depth', taskMatch: true, refined: true };
  Assert.equal(Api.ResolveTaskObject(Query, [Mouse], { LastSeen: Usb }).Binding, null);
  Assert.equal(Api.ResolveTaskObject(Query, [Mouse], { LastSeen: Usb, Track: false, Tip: { x: 4.1, y: 33.3 } }).Binding, null);
  const Relabelled = { ...Usb, label: 'small black object', x: 4.2, y: 33.1, taskMatch: false };
  Assert.equal(Api.ResolveTaskObject(Query, [Mouse, Relabelled], { LastSeen: Usb }).Binding.Object, Relabelled);
  // Never measured yet: the query still finds it anywhere.
  Assert.equal(Api.ResolveTaskObject(Query, [Mouse]).Binding.Object, Mouse);
});

// 2026-09-30 05:24-05:31 (back end/captures/2026-09-30/recording/ai.jsonl): six tasks in a row for the black servo
// sitting on the clear tape roll ended in a question. The detector never said "servo": it called it "small black
// object", "small black box", "black object on tape", "usb drive", "battery"; the strict query answered [] for
// "black servo motor thing" and "black servo thing", so the model was told it was NOT detected, while the object
// itself was [UNRELIABLE: coarse box, not refined] (the bridge refines only boxes it takes for the task object).
// Rows: label, X, Y, base Z, top Z, footprint, extrapolated, support, refined, task_match, task_query_only,
// valid_fraction, outside_calibration_cm.
const Servo = {
  // 05:24:55, "grab the black servo motor thing" (reply: "not detected ... marked as [UNRELIABLE]. Could you confirm").
  F0524: [['can', 19.47, 41.31, -1.61, 10.12, [5.6, 5.4], true, 'desk', false, false, false, 0.918, 13.9],
    ['bottle', 11.13, 31.63, -2.28, 5.58, [3.5, 3], false, 'desk', true, false, false, 0.876, 1.2],
    ['tape', 0.75, 30.71, -2.38, 2.54, [7.9, 4.7], false, 'desk', false, false, false, 0.988, 0],
    ['keyboard', 3.07, 61.09, 0.12, 1.28, [9.7, 21.7], true, 'desk', false, false, false, 1, 24.8],
    ['mouse', 12.82, 107.5, 3.86, 10.08, [10.9, 9.8], true, 'desk', false, false, false, 0.999, 72.2],
    ['cup', -12.66, 31.86, -2.02, -0.19, [7.8, 8.5], false, 'desk', true, false, false, 0.562, 1.2],
    ['pen', 16, 60.28, 0.17, 2.08, [2, 4.5], true, 'desk', false, false, false, 0.999, 28],
    ['tape roll', 0.88, 29.84, -2.39, 2.05, [7.9, 4.6], false, 'desk', false, false, false, 0.997, 0],
    ['pliers', 3.08, 7.44, -8.21, -5.51, [18.6, 8.2], true, 'bin', false, false, false, 0.862, 16.2],
    ['small black object', 1.01, 30.84, -2.18, 2.88, [4.5, 2.4], false, 'desk', false, false, false, 0.963, 0]],
  // 05:26:53, "grab the black servo thing": the same object as "usb drive" and "small black box".
  F0526: [['travel mug', 18.6, 41.35, -1.68, 10.05, [5.7, 5.5], true, 'desk', false, false, false, 0.927, 13.4],
    ['bottle', 10.45, 31.34, -2.3, 5.49, [3.7, 3], false, 'desk', false, false, false, 0.872, 0.5],
    ['tape roll', 0.3, 29.47, -2.44, 2.06, [7.6, 4.5], false, 'desk', false, false, false, 0.992, 0],
    ['usb drive', 0.11, 30.51, -2.23, 2.82, [4.3, 2.1], false, 'desk', false, false, false, 0.936, 0],
    ['keyboard', 2.02, 60.95, 0.05, 1.17, [9.6, 22.6], true, 'desk', false, false, false, 1, 24.5],
    ['laptop', 35.93, 85.13, 1.12, 13.78, [14, 29.3], true, 'desk', false, false, false, 0.779, 59.2],
    ['cup', -14, 31.59, -2.23, 0.18, [8.1, 8.4], false, 'desk', false, false, false, 0.554, 2],
    ['small bottle', 10.45, 31.37, -2.3, 5.51, [3.6, 3.1], false, 'desk', false, false, false, 0.861, 0.5],
    ['small black box', 0.09, 30.51, -2.23, 2.81, [4.2, 2.1], false, 'desk', false, false, false, 0.935, 0],
    ['pen (small)', 14.67, 59.57, 0.09, 2.32, [2.1, 5.2], true, 'desk', false, false, false, 1, 26.8],
    ['paper cup', -13.97, 31.59, -2.27, 0.15, [8, 8.5], false, 'desk', false, false, false, 0.546, 2]],
  // 05:28:49, "grab the black thing on the desk thats next to the pill bottle": the query boxed something 42 cm
  // outside the calibrated area (34 x 21 cm), and the task ended OUT OF REACH.
  F0528: [['travel mug', 18.62, 41.3, -1.69, 10.08, [5.6, 5.5], true, 'desk', false, false, false, 0.947, 13.4],
    ['small bottle', 10.44, 31.35, -2.28, 5.5, [3.7, 3], false, 'desk', false, false, false, 0.886, 0.5],
    ['tape roll', 0.15, 29.88, -2.43, 2.29, [8, 4.4], false, 'desk', false, false, false, 0.979, 0],
    ['black object on tape', 0.04, 30.57, -2.23, 2.81, [4.2, 2.2], false, 'desk', false, false, false, 0.929, 0],
    ['keyboard', 2.78, 63, 0.16, 1.32, [12.4, 23.7], true, 'desk', false, false, false, 1, 26.6],
    ['cup', -13.54, 31.48, -2.26, 0.44, [8, 8.7], false, 'desk', false, false, false, 0.563, 1.9],
    ['laptop', 35.92, 84.97, 1.1, 13.64, [14.6, 30.5], true, 'desk', false, false, false, 0.764, 59.1],
    ['wires', 2.31, 7.83, -8.03, -5.69, [16, 10.7], true, 'bin', false, false, false, 0.866, 16.2],
    ['tumbler', 18.62, 41.3, -1.69, 10.08, [5.7, 5.5], true, 'desk', false, false, false, 0.94, 13.4],
    ['USB drive', 0.08, 30.55, -2.23, 2.81, [4.3, 2.1], false, 'desk', false, false, false, 0.932, 0],
    ['potted plant', -13.67, 31.52, -2.24, 0.45, [8, 8.7], false, 'desk', false, false, false, 0.58, 1.9],
    ['roll of tape', -0.09, 30.48, -2.43, 2.55, [8.1, 4.7], false, 'desk', false, false, false, 0.974, 0],
    ['paper cup', -13.46, 31.42, -2.28, 0.42, [8, 8.8], false, 'desk', false, false, false, 0.564, 1.9],
    ['black thing', -28.2, 75.76, 1.36, 14.38, [34.1, 20.8], true, 'desk', false, true, true, 0.703, 42.2]],
};
const ServoFrame = Rows => Api.ReadLocateResponse({ ok: true, camera_check: Check(), objects: Rows.map(([label, x, y, z, top_z, size_cm, extrapolated,
  support_branch, refined, task_match, task_query_only, valid_fraction, outside_calibration_cm]) => ({ label, x, y, z, top_z, height_cm: top_z - z, size_cm,
  method: 'depth', extrapolated, support_branch, support_measured: true, refined, task_match, task_query_only, valid_fraction, outside_calibration_cm })) });
const ServoState = (Query, Resolution) => ({ Label: Query.Name, Query, DetectedNow: !!Resolution.Binding, Binding: Resolution.Binding,
  Candidates: Resolution.Candidates, LastSeen: Resolution.Binding ? { At: 0, Object: Resolution.Binding.Object, Reachability: null } : null, Reachability: null });

Test('a noun the detector never uses binds the one label with the task\'s colour, and the model is told how (the servo, 2026-09-30 05:24)', () => {
  const Located = ServoFrame(Servo.F0524);
  const Query = Parse('grab the black servo motor thing');
  Assert.deepEqual([Query.Head, Query.Colours, Query.Name], ['motor', ['black'], 'black servo motor thing']);
  const Resolution = Api.ResolveTaskObject(Query, Located.objects);
  Assert.equal(Resolution.Binding.Object.label, 'small black object');
  Assert.equal(Resolution.Binding.Source, 'colour');
  const Line = Api.TaskObjectLine(ServoState(Query, Resolution));
  Assert.match(Line, /^TASK OBJECT "black servo motor thing": detected this check-in \(measured as "small black object", the only label with its colour; no measured label names a "motor": check in the image that it is the black servo motor thing, else reply ask\) at X 1\.0, Y 30\.8, base Z -2\.2, top Z 2\.9\./);
  // Its depth was dense (96%) and inside the calibrated area: only the skipped refine tagged it, which the task object is spared.
  Assert.doesNotMatch(Line, /UNRELIABLE|NOT detected/);
  const Summary = Api.MeasuredObjectsSummary(Located, Resolution.Binding.Object);
  Assert.match(Summary, /- small black object: [^\n]*measured by the depth camera\) <- the task object\n/);
  Assert.match(Summary, /- tape roll: [^\n]*\[UNRELIABLE: coarse box, not refined/, 'The others keep their tag');
  Assert.equal(Api.UnreliableMeasurements(Located.objects, ServoState(Query, Resolution)).some(Object_ => Object_.label === 'small black object'), false);
  // 05:26 "grab the black servo thing": "small black box" says the colour; the "usb drive" at its spot does not.
  const Box = Bind('grab the black servo thing', ServoFrame(Servo.F0526).objects);
  Assert.deepEqual([Box.Object.label, Box.Source], ['small black box', 'colour']);
  // Without a colour the labels that could be it are listed, the generic one for the servo's spot (not "tape"), and the model's estimate binds it.
  const Plain = Api.ResolveTaskObject(Parse('grab the servo motor'), Located.objects);
  Assert.equal(Plain.Binding, null);
  const PlainLine = Api.TaskObjectLine(ServoState(Parse('grab the servo motor'), Plain));
  Assert.match(PlainLine, /^TASK OBJECT "servo motor": no measured label names it\. Candidates measured this check-in: [^\n]*small black object \(X 1\.0, Y 30\.8\)/);
  Assert.doesNotMatch(PlainLine, /pliers/, 'Not the bin for a desk task');
  Assert.equal(Api.BindFromEstimate(Parse('grab the servo motor'), Located.objects, { x: 1.0, y: 30.8, z: 0 }).Object.label, 'small black object');
  Assert.equal(Api.BindFromEstimate(Parse('grab the black servo thing'), ServoFrame(Servo.F0526).objects, { x: 0.1, y: 30.5, z: 0 }).Object.label, 'small black box');
});

Test('stand-ins stay safe: a known noun only takes a label naming no kind of thing, and a clashing colour, the bin or the robot never stand in', () => {
  const F0524 = ServoFrame(Servo.F0524).objects, F0526 = ServoFrame(Servo.F0526).objects;
  // "small black box" names a kind (a box): a stapler task is not bound to it, but "small black object" is a stand-in.
  Assert.equal(Bind('grab the black stapler', F0526), null);
  Assert.equal(Api.ResolveTaskObject(Parse('grab the black stapler'), F0526).Candidates.length, 0);
  Assert.equal(Bind('grab the black stapler', F0524).Object.label, 'small black object');
  // A known noun with nothing that could be it keeps the plain instruction.
  Assert.match(Api.TaskObjectLine(ServoState(Parse('grab the eraser'), Api.ResolveTaskObject(Parse('grab the eraser'), F0526))),
    /^TASK OBJECT "eraser": NOT detected in any check-in of this task/);
  // A clashing colour is never bound; the bin only for a bin task; the robot never.
  Assert.equal(Bind('grab the white servo', F0524), null);
  Assert.equal(Bind('grab the black servo in the bin', F0524), null);
  Assert.equal(Bind('grab the black robot', F0524), null);
  Assert.equal(Api.BindFromEstimate(Parse('grab the black robot'), F0524, { x: 1.0, y: 30.8, z: 0 }), null);
  // Two black stand-ins at two spots are listed, not guessed.
  const Two = Api.ResolveTaskObject(Parse('grab the black servo'), [...F0524, { ...F0524.at(-1), label: 'black block', x: -6, y: 28 }]);
  Assert.equal(Two.Binding, null);
  Assert.deepEqual(Two.Candidates.map(Object_ => Object_.label).sort(), ['black block', 'small black object']);
  // A known noun still never binds a label naming another kind from the model's estimate (the binder, 2026-09-28 02:03).
  Assert.equal(Api.BindFromEstimate(Parse('grab the black stapler'), F0526, { x: 0.1, y: 30.5, z: 0 }), null);
});

Test('a box only the task query found far outside the calibrated area yields to the one label inside it that says the colour (05:28)', () => {
  const F0528 = ServoFrame(Servo.F0528).objects;
  const Query = Parse('grab the black thing on the desk thats next to the pill bottle');
  Assert.equal(Query.Phrase, 'black thing on the desk');
  const Resolution = Api.ResolveTaskObject(Query, F0528);
  Assert.deepEqual([Resolution.Binding.Object.label, Resolution.Binding.Source], ['black object on tape', 'colour']);
  // Inside the calibrated area, or found by the samples too, the query's pick stands.
  const Near = F0528.map(Object_ => Object_.label === 'black thing' ? { ...Object_, extrapolated: false, outsideCalibrationCm: 0 } : Object_);
  Assert.equal(Bind('grab the black thing on the desk', Near).Object.label, 'black thing');
  const Sampled = F0528.map(Object_ => Object_.label === 'black thing' ? { ...Object_, taskQueryOnly: false } : Object_);
  Assert.equal(Bind('grab the black thing on the desk', Sampled).Object.label, 'black thing');
  // No coloured label inside it: the far pick stays (and is OUT OF REACH, as before).
  Assert.equal(Bind('grab the black thing on the desk', F0528.filter(Object_ => Object_.label !== 'black object on tape')).Object.label, 'black thing');
  // Once measured, tracking decides (RebindGateCm), not this rule.
  const Seen = F0528.find(Object_ => Object_.label === 'black object on tape');
  Assert.equal(Bind('grab the black thing on the desk', F0528, { LastSeen: Seen }).Object.label, 'black object on tape');
});

// An answer to the model's question (lib/TaskObject.ts ContinueAskedTask), bundled on its own.
const TaskObjectBundle = await Build({ stdin: { contents: `export * from './TaskObject';`, resolveDir: Path.join(Root, 'src/lib'), loader: 'ts' },
  bundle: true, platform: 'node', format: 'cjs', write: false });
const TaskObjectModule = { exports: {} };
new Function('require', 'module', 'exports', TaskObjectBundle.outputFiles[0].text)(CreateRequire(import.meta.url), TaskObjectModule, TaskObjectModule.exports);
const { ContinueAskedTask } = TaskObjectModule.exports;

Test('an answer to the model\'s question goes on with the asked task and its object (2026-09-30 05:25-05:28)', () => {
  const Question = "The 'black servo motor thing' was not detected by the system. I see a small black object on the desk, but it's marked as [UNRELIABLE]. Could you please confirm if that is the correct object to grab?";
  const Asked = { Task: 'grab the black servo motor thing', Question, Query: Parse('grab the black servo motor thing') };
  // 05:25:28: the answer describes the object; the asked verb grabs it, and the model sees its own question.
  const Yes = ContinueAskedTask('yes, its the small black object on the desk', Asked);
  Assert.deepEqual([Yes.Query.Name, Yes.Query.Phrase, Yes.Query.Verb, Yes.Query.Colours, Yes.Query.Sizes],
    ['small black thing', 'small black thing on the desk', 'grab', ['black'], ['small']]);
  Assert.equal(Yes.Task, `grab the black servo motor thing (you asked: "${Question}"; the user answered: "yes, its the small black object on the desk")`);
  Assert.equal(Api.TaskNeedsPositions(Yes.Task), true);
  Assert.equal(Bind('grab the small black object', ServoFrame(Servo.F0524).objects).Object.label, 'small black object');
  Assert.equal(Api.ResolveTaskObject(Yes.Query, ServoFrame(Servo.F0524).objects).Binding.Object.label, 'small black object');
  // 05:27:12: a new description; 05:27:55 and 09-29 05:56 "try again": the asked object.
  Assert.equal(ContinueAskedTask('the black box near the tape roll', Asked).Query.Name, 'black box');
  for (const Answer of ['yes grab that', 'yes', 'try again', 'do it', 'pick it up', 'no', 'the left one', 'ok go ahead', 'carry on']) {
    const Continued = ContinueAskedTask(Answer, Asked);
    Assert.ok(Continued, Answer);
    Assert.equal(Continued.Query, Asked.Query, Answer);
    Assert.match(Continued.Task, /^grab the black servo motor thing \(you asked: /, Answer);
  }
  // A task of its own, a question, or another command is no answer.
  for (const Prompt of ['grab the can', 'yes, grab the small black object', 'describe the workspace', 'is it the black one?', 'move up 3 cm',
    'open the gripper', 'Close the gripper', 'what do you see', 'up 3 cm', 'go home', 'stop', 'rotate the wrist', 'raise your hand']) {
    Assert.equal(ContinueAskedTask(Prompt, Asked), null, Prompt);
  }
  Assert.equal(ContinueAskedTask('yes', null), null, 'Nothing was asked');
  // A second answer replaces the first one's context instead of nesting it; an asked task without an object takes its verb.
  const Again = ContinueAskedTask('the small one', { ...Asked, Task: Yes.Task, Question: 'Which black object?' });
  Assert.equal(Again.Task, 'grab the black servo motor thing (you asked: "Which black object?"; the user answered: "the small one")');
  Assert.equal(Again.Query.Name, 'small thing');
  const Loose = ContinueAskedTask('the pill bottle', { Task: 'grab it and drop it into the basket', Question: 'Which object?', Query: null });
  Assert.deepEqual([Loose.Query.Name, Loose.Query.Verb], ['pill bottle', 'grab']);
  Assert.equal(ContinueAskedTask('yes', { Task: 'grab it', Question: 'This one?', Query: null }).Query, null);
});

Test('a go-ahead after the app stopped a task goes on with it and its object (2026-10-01 07:56 "ok i did that", 08:00 "now grab it")', () => {
  // 07:56:23: the app stopped the continued can task; the user did what it said and typed "ok i did that".
  const Can = Parse('grab the can');
  const Reason = 'the AI twice proposed a move that would not move the arm (that target is 0.0 cm from the target the arm is already holding).';
  const Advice = 'Move the can closer to the right arm, clear of the basket rim, or rephrase the task, then send it again.';
  const Stopped = { Task: 'grab the can (you asked: "The can is marked as OUT OF REACH for the right arm (53 cm from the shoulder)."; the user answered: "try again")',
    Question: `${Reason} ${Advice}`, Query: Can, Stopped: true };
  const Done = ContinueAskedTask('ok i did that', Stopped);
  Assert.equal(Done.Task, `grab the can (the app stopped it: "${Reason} ${Advice}"; the user answered: "ok i did that")`);
  Assert.equal(Done.Query, Can, 'the can stays the task object');
  Assert.equal(Api.TaskNeedsPositions(Done.Task), true);
  // 07:59:24 refused the can (5.3 x 7.7 cm), 08:00:21 "now grab it"; 08:01:40 refused it for the battery beside it.
  const Refused = { Task: 'Grab the can', Question: 'the can measured at X 0.8, Y 35.2 is not graspable by this gripper: it measured 5.3 x 7.7 cm. Try a narrower object.',
    Query: Parse('Grab the can'), Stopped: true };
  Assert.match(ContinueAskedTask('now grab it', Refused).Task, /^Grab the can \(the app stopped it: "the can measured at X 0\.8.*"; the user answered: "now grab it"\)$/);
  for (const Answer of ['done', 'try again', 'go', 'ok', 'OK, go ahead', 'retry', 'did it', "ok I've moved it", 'i moved the can closer', 'ok i moved the battery away',
    'moved it', 'yes', 'do it again', 'pick it up now', 'continue', 'all set']) {
    const Continued = ContinueAskedTask(Answer, Refused);
    Assert.ok(Continued, Answer);
    Assert.equal(Continued.Query, Refused.Query, Answer);
    Assert.match(Continued.Task, /^Grab the can \(the app stopped it: /, Answer);
  }
  // A task of its own (the can again, another object, a misspelt verb on one), a question or another command starts a new task.
  for (const Prompt of ['grab the can', 'grab the bottle', 'move the battery away', 'ok grabe the cup', 'describe the scene', 'what happened?',
    'why did it stop', 'move up 3 cm', 'open the gripper', 'go home', 'stop', 'the red one', 'put the tape in the basket']) {
    Assert.equal(ContinueAskedTask(Prompt, Refused), null, Prompt);
  }
  // A model's question is still answered as before.
  Assert.match(ContinueAskedTask('the black box near the tape roll', { ...Refused, Stopped: false }).Task, /\(you asked: /);
  // The stop's advice no longer sends the user to move a can that was planned (the arm held its hover point).
  Assert.match(Api.TaskStopAdvice(true, 'can', true), /^The can is within reach and its grasp was planned; .*Say "try again" to go on from where the arm is/);
  Assert.equal(Api.TaskStopAdvice(true, 'can'), Advice);
  Assert.match(Api.TaskStopAdvice(false, 'can', true), /camera was not verified/);
  // A task with no object is told to ask which one rather than pick one (07:56 the model went for the tape roll).
  Assert.match(Api.NoTaskObjectLine, /^TASK OBJECT: none: the task names no object\. Do not choose an object .* reply ask and ask the user which object\.$/);
});

// --- CHAINED MOTION (lib/AiChain.ts): one gripper action and one tip move on one approval ------------

// The home pose with the gripper at a fraction (present and target).
const GripperAt = Fraction => {
  const Servos = structuredClone(App.Servos);
  const Raw = Api.fractionToRaw(Servos.gripper, Fraction);
  Object.assign(Servos.gripper, { presentRaw: Raw, targetRaw: Raw });
  return Servos;
};
const ChainOf = (Servos, Motion) => App.ParseResponse(Reply(Motion), TipOf(Servos), Servos).motion;

Test('a chained motion is read from gripper_before or gripper_after; a gripper action that changes nothing is dropped, one without a move is a gripper step', () => {
  Assert.deepEqual(Api.ReadChainGripper({ gripper_before: 0.8 }), { Order: 'before', Fraction: 0.8 });
  Assert.deepEqual(Api.ReadChainGripper({ gripper_after: '0.0' }), { Order: 'after', Fraction: 0 });
  Assert.deepEqual(Api.ReadChainGripper({ gripper_before: { target_fraction: 0.6 } }), { Order: 'before', Fraction: 0.6 });
  Assert.equal(Api.ReadChainGripper({ target_cm: { x: 1 } }), null);
  Assert.match(Api.ReadChainGripper({ gripper_before: 0.8, gripper_after: 0 }).Error, /ONE gripper action: give gripper_before or gripper_after, not both/);
  Assert.match(Api.ReadChainGripper({ gripper_before: 1.5 }).Error, /gripper_before must be a gripper fraction from 0\.0 \(closed\) to 1\.0 \(open\)/);
  Assert.match(Api.ReadChainGripper({ gripper_after: 'open' }).Error, /gripper_after must be a gripper fraction/);
  // Open, then move up 4 cm: one proposal, the gripper opening while the tip moves (Together).
  const Closed = GripperAt(0.03);
  const Open = ChainOf(Closed, { move_delta_cm: { left: 0, farther: 0, up: 4 }, gripper_before: 0.8 });
  Assert.deepEqual(Open.chain, { Order: 'before', TargetRaw: Api.fractionToRaw(Closed.gripper, 0.8), Fraction: 0.8, Together: true });
  Assert.equal(Open.hasTargetMotion, true);
  Assert.deepEqual(Open.servoTargets, []);
  Assert.deepEqual(App.Validate(Open, 'up', Closed).issues, []);
  // The move first, then the gripper.
  Assert.equal(ChainOf(Closed, { move_delta_cm: { left: 0, farther: 0, up: 4 }, gripper_after: 0.8 }).chain.Order, 'after');
  // Already 80% open: the gripper action is dropped and the move is a single motion.
  const Restated = ChainOf(GripperAt(0.8), { move_delta_cm: { left: 0, farther: 0, up: 4 }, gripper_before: 0.8 });
  Assert.equal(Restated.chain, null);
  Assert.equal(Restated.hasTargetMotion, true);
  // No tip move: a gripper step, as direct_servo_targets would give it.
  const Alone = ChainOf(Closed, { gripper_before: 0.8 });
  Assert.equal(Alone.chain, null);
  Assert.equal(Alone.hasTargetMotion, false);
  Assert.deepEqual(Alone.servoTargets, [{ name: 'gripper', targetRaw: Api.fractionToRaw(Closed.gripper, 0.8) }]);
  // Both orders at once, or the gripper in direct_servo_targets beside a tip move: refused with how to chain.
  const Both = ChainOf(Closed, { move_delta_cm: { left: 0, farther: 0, up: 4 }, gripper_before: 0.8, gripper_after: 0 });
  Assert.equal(Both.chain, null);
  Assert.match(App.Validate(Both, 'up', Closed).issues.join(' '), /ONE gripper action/);
  const Direct = ChainOf(Closed, { move_delta_cm: { left: 0, farther: 0, up: 4 }, direct_servo_targets: [{ name: 'gripper', target_fraction: 0.8 }] });
  Assert.match(App.Validate(Direct, 'up', Closed).issues.join(' '), /chain the gripper with the tip move through gripper_before or gripper_after \(not direct_servo_targets\)/);
  // A close, then lift straight up: allowed; a close after the lift: refused.
  const Open80 = GripperAt(0.8);
  Assert.deepEqual(App.Validate(ChainOf(Open80, { move_delta_cm: { left: 0, farther: 0, up: 4 }, gripper_before: 0 }), 'up', Open80).issues, []);
  Assert.match(App.Validate(ChainOf(Open80, { move_delta_cm: { left: 0, farther: 0, up: 4 }, gripper_after: 0 }), 'up', Open80).issues.join(' '), /A close is never chained after a tip move/);
  // "Open the gripper and move" with the open asked as 1.0 (the prompt's "1.0 means open"): chained at 0.8, which the
  // card and the check-in say, instead of a refusal and another model call. Alone, the gripper step keeps 1.0.
  const Full = ChainOf(Closed, { move_delta_cm: { left: 0, farther: 0, up: 4 }, gripper_before: 1 });
  Assert.deepEqual(Full.chain, { Order: 'before', TargetRaw: Api.fractionToRaw(Closed.gripper, 0.8), Fraction: 0.8, Asked: 1, Together: true });
  Assert.deepEqual(App.Validate(Full, 'up', Closed).issues, []);
  Assert.equal(Api.ChainGripperText(0.03, 0.8, 1), 'open the gripper to 80% (100% was asked for; a chained open goes to at most 80%)');
  Assert.equal(Api.ChainGripperText(0.03, 0.8), 'open the gripper to 80%');
  Assert.deepEqual(ChainOf(Closed, { gripper_before: 1 }).servoTargets, [{ name: 'gripper', targetRaw: Api.fractionToRaw(Closed.gripper, 1) }]);
  // Already 80% open or wider, a chained 1.0 would not open it: dropped, the move runs alone (never narrowed to 80%).
  for (const Already of [0.8, 0.9, 1]) {
    const Wide = ChainOf(GripperAt(Already), { move_delta_cm: { left: 0, farther: 0, up: 4 }, gripper_before: 1 });
    Assert.equal(Wide.chain, null, `${Already}`);
    Assert.equal(Wide.hasTargetMotion, true);
    Assert.deepEqual(Wide.servoTargets, []);
  }
  // The rule itself still refuses a wider chained open, should one ever reach it.
  Assert.match(Api.ChainRefusalReason({ Order: 'before', FromFraction: 0.03, ToFraction: 1, Start: { x: 0, y: 20, z: 10 }, End: { x: 0, y: 20, z: 14 }, WristTurnDeg: 0, Task: null }),
    /may open the gripper to at most 0\.8/);
});

Test('which chains run as one motion: open or narrow then move, close then a straight lift, move then open; never the descent, a close after a move or before anything but a lift, nor the jaws beside the object under its top', () => {
  // The eraser: a 2 x 2 cm footprint, its top at Z 0 (display frame).
  const Eraser = { Label: 'eraser', Footprint: [{ x: -6, y: 34 }, { x: -4, y: 34 }, { x: -4, y: 36 }, { x: -6, y: 36 }], BaseZ: -2.3, TopZ: 0 };
  const Task = { Prism: Eraser, Label: 'eraser' };
  const Far = { x: -14, y: 20, z: 12 }, Hover = { x: -3, y: 35, z: 4 }, Grasp = { x: -3.5, y: 35, z: -1.3 };
  const Shape = (Order, FromFraction, ToFraction, Start, End, Extra = {}) => ({ Order, FromFraction, ToFraction, Start, End, WristTurnDeg: 0, Task, ...Extra });
  const Allowed = {
    'open, then the hover point': Shape('before', 0.03, 0.8, Far, Hover),
    'narrow to a plan\'s 60%, then the hover point': Shape('before', 0.8, 0.6, Far, Hover),
    'close, then lift straight up': Shape('before', 0.8, 0, Grasp, { ...Grasp, z: Grasp.z + 6 }),
    'close, then lift with a wrist change of 1 deg': Shape('before', 0.8, 0, Grasp, { ...Grasp, z: Grasp.z + 6 }, { WristTurnDeg: 1 }),
    'move above the basket, then open (a release)': Shape('after', 0, 0.8, { x: 0, y: 30, z: 10 }, { x: 5, y: 10, z: 8 }),
    'rise out from beside it, then open above it': Shape('after', 0.03, 0.8, Grasp, { ...Grasp, z: 4 }),
    'open, then move down with no task object': Shape('before', 0.03, 0.8, Far, { x: -3, y: 35, z: -1.3 }, { Task: null }),
  };
  for (const [Name, Case] of Object.entries(Allowed)) Assert.equal(Api.ChainRefusalReason(Case), null, Name);
  const Refused = [
    ['close after a move', Shape('after', 0.8, 0, Hover, Grasp), /A close is never chained after a tip move: the jaws would close wherever the move ends/],
    ['close, then sideways', Shape('before', 0.8, 0, Grasp, { x: -1.5, y: 35, z: 4.7 }), /only tip move that may be chained is a straight lift .* this one goes 6\.0 cm up and 2\.0 cm sideways/],
    ['close, then down', Shape('before', 0.8, 0, Hover, { ...Hover, z: 2 }), /straight lift/],
    ['close, then a lift turning the wrist', Shape('before', 0.8, 0, Grasp, { ...Grasp, z: Grasp.z + 6 }, { WristTurnDeg: 15 }), /turns the wrist 15\.0 deg/],
    ['open, then the descent', Shape('before', 0.6, 0.8, Hover, Grasp), /goes down beside the eraser \(it ends 0\.5 cm from it, below its top\), which is the descent onto it: the descent is never chained/],
    ['the descent, then open', Shape('after', 0.6, 0.8, Hover, Grasp), /the descent is never chained/],
    ['the descent, then narrow', Shape('after', 0.8, 0.6, Hover, Grasp), /the descent is never chained/],
    ['open beside it under its top, then rise', Shape('before', 0.03, 0.8, Grasp, { ...Grasp, z: 4 }), /The gripper would open beside the eraser below its top .* Rise straight up alone first/],
    ['rise, then open still beside it under its top', Shape('after', 0.03, 0.8, { ...Grasp, z: -2 }, { ...Grasp, z: -0.5 }), /The gripper would open where that move ends, beside the eraser below its top/],
    ['open wider than 80%', Shape('before', 0.03, 1, Far, Hover), /at most 0\.8 \(gripper_before 1 was proposed\)/],
    // The snap moved it above the object, but the model asked for the descent: refused all the same.
    ['a snapped descent', Shape('before', 0.6, 0.8, Hover, Hover, { Requested: Grasp }), /the descent is never chained/],
  ];
  for (const [Name, Case, Pattern] of Refused) Assert.match(Api.ChainRefusalReason(Case) ?? '(allowed)', Pattern, Name);
  Assert.deepEqual([Api.ChainGripperKind(0.03, 0.8), Api.ChainGripperKind(0.8, 0.6), Api.ChainGripperKind(0.8, 0.3), Api.ChainGripperKind(0.6, 0)], ['open', 'narrow', 'close', 'close']);
});

Test('a chained motion is told to the model as one: both legs in their order, and which of them ran', () => {
  const Run = (Results, Why = null, Order = 'before') => Api.DescribeChainRun({ Order, Gripper: 'open the gripper to 80%', Tip: 'move the tip to X 6.9, Y 31.7, Z 4.6 cm, wrist -49.7 deg', Results, Why });
  Assert.equal(Run(['sent', 'sent']), 'It was a chained motion: first open the gripper to 80%, then move the tip to X 6.9, Y 31.7, Z 4.6 cm, wrist -49.7 deg; both parts ran, in that order.');
  Assert.equal(Run(['sent', 'sent'], null, 'after'), 'It was a chained motion: first move the tip to X 6.9, Y 31.7, Z 4.6 cm, wrist -49.7 deg, then open the gripper to 80%; both parts ran, in that order.');
  Assert.equal(Run(['sent', 'not-sent'], 'Stop task was pressed.'), 'It was a chained motion: first open the gripper to 80%, then move the tip to X 6.9, Y 31.7, Z 4.6 cm, wrist -49.7 deg; only the first part ran (open the gripper to 80%): the second (move the tip to X 6.9, Y 31.7, Z 4.6 cm, wrist -49.7 deg) was not sent: Stop task was pressed.');
  Assert.match(Run(['sent', 'refused'], 'the bridge refused it: busy'), /only the first part ran \(open the gripper to 80%\): the second \(move the tip .*\) was refused by the bridge: the bridge refused it: busy\.$/);
  Assert.match(Run(['refused', 'not-sent']), /; neither part ran: the first \(open the gripper to 80%\) was refused by the bridge\.$/);
  Assert.match(Run(['skipped', 'sent']), /; the first part had nothing to do \(the gripper was already there\), and the second ran\.$/);
  Assert.match(Run(['sent', 'skipped']), /; the first part ran, and the second had nothing to do \(the tip was already there\)\.$/);
  // The check-in states what the motion measurably did, then how the chain ran.
  const Fact = Api.DescribeMotionFact({ Step: 2, StartTip: { x: -13.3, y: 20.1, z: 12.4 }, EndTip: { x: 6.9, y: 31.7, z: 4.6 }, Target: { x: 6.9, y: 31.7, z: 4.6 },
    Servos: 'gripper now at raw 1922 (commanded 1922)', Chain: Run(['sent', 'sent']) });
  Assert.match(Fact, /^Your last approved motion \(motion 2\) started and has stopped: the tip moved 24\.6 cm, .*; gripper now at raw 1922 \(commanded 1922\)\. It was a chained motion: first open the gripper to 80%, then move the tip .*; both parts ran, in that order\.$/);
  Assert.match(Api.DescribeBridgeRefusal(2, 'Busy.', null, Run(['refused', 'not-sent'])), /was NOT executed: the bridge refused it: Busy\. The arm did not move\. It was a chained motion: .*neither part ran.* Do not propose the same target again/);
});

Test("a chain's second part follows only a first part nothing else cut across, and its gripper action never runs where the tip stopped low beside the object", () => {
  // The bridge's epoch moves at each route start and at every stop, Coast, Hold or Home; arriving and a grasp hold leave it.
  const Leg = { Kind: 'tip', Target: { x: 0, y: 20, z: 10, wristPitch: -40 } };
  const Wait = (Extra = {}) => ({ approvedAt: 0, epoch: 5, startedAt: 100, StartedEpoch: 6, Chain: { Part: 1, Next: Leg, CheckIn: true, EndsTask: false }, ...Extra });
  Assert.equal(Api.ChainLegInterrupted(Wait(), 6), false, 'the first part running or settled');
  Assert.equal(Api.ChainLegInterrupted(Wait(), 7), true, 'Hold, Home, a stop or a slider after it started');
  Assert.equal(Api.ChainLegInterrupted(Wait(), 9), true);
  Assert.equal(Api.ChainLegInterrupted(Wait({ startedAt: null, StartedEpoch: null }), 7), false, 'not started yet: the start itself moves it');
  Assert.equal(Api.ChainLegInterrupted(Wait({ Chain: { Part: 2, Next: null, CheckIn: true, EndsTask: false } }), 7), false, 'the last part: a check-in follows as for any motion');
  Assert.equal(Api.ChainLegInterrupted({ approvedAt: 0, epoch: 5, startedAt: 100, StartedEpoch: 6 }, 7), false, 'no chain');
  // Where the tip stopped (display frame), beside the eraser (2 x 2 cm, top Z 0): low beside it, an open or a narrowing is not sent.
  const Task = { Prism: { Label: 'eraser', Footprint: [{ x: -6, y: 34 }, { x: -4, y: 34 }, { x: -4, y: 36 }, { x: -6, y: 36 }], BaseZ: -2.3, TopZ: 0 }, Label: 'eraser' };
  Assert.equal(Api.ChainGripperHereReason(0.03, 0.8, { x: -3, y: 35, z: 0.7 }, Task), 'the tip stopped beside the eraser below its top (1.0 cm from it), where the moving jaw would swing toward it as the gripper opens');
  Assert.match(Api.ChainGripperHereReason(0.8, 0.6, { x: -2, y: 35, z: -1 }, Task), /2\.0 cm from it\), where the moving jaw would swing toward it as the gripper narrows$/);
  Assert.equal(Api.ChainGripperHereReason(0.03, 0.8, { x: -3, y: 35, z: 1.2 }, Task), null, 'above its top + 1 cm');
  Assert.equal(Api.ChainGripperHereReason(0.03, 0.8, { x: 0, y: 35, z: -1 }, Task), null, 'more than 3 cm from it');
  Assert.equal(Api.ChainGripperHereReason(0.8, 0, { x: -3, y: 35, z: -1 }, Task), null, 'a close (its own rule: only before a straight lift)');
  Assert.equal(Api.ChainGripperHereReason(0.03, 0.8, { x: -3, y: 35, z: -1 }, null), null, 'no task object, or one the jaws hold');
  Assert.equal(Api.ChainLowBeside(Task, { x: -3, y: 35, z: 0.9 }), 1);
  Assert.equal(Api.ChainLowBeside(Task, { x: -3, y: 35, z: 1 }), null);
  // How the check-in says the second part was not sent because the first did not get there.
  Assert.match(Api.DescribeChainRun({ Order: 'after', Gripper: 'open the gripper to 80%', Tip: 'move the tip to X 5.0, Y 10.0, Z 8.0 cm, wrist -40.0 deg',
    Results: ['sent', 'not-sent'], Why: "the tip stopped 2.4 cm from that part's target (more than 1.5 cm)" }),
  /only the first part ran \(move the tip to X 5\.0, Y 10\.0, Z 8\.0 cm, wrist -40\.0 deg\): the second \(open the gripper to 80%\) was not sent: the tip stopped 2\.4 cm from that part's target \(more than 1\.5 cm\)\.$/);
  Assert.equal(Api.ChainRules.ArrivedCm, 1.5);
});

Test('the system prompt gives the chained motion: its JSON, its order, that it counts as one motion, and what is never chained', () => {
  const Prompt = App.AiPrompt('', false, false, 'Behind');
  Assert.match(Prompt, /never both\. The one exception is a CHAINED MOTION: one gripper action chained with one tip move\./);
  Assert.match(Prompt, /CHAINED MOTION: to move the gripper and the tip with ONE motion_proposal, add "gripper_before" \(the gripper moves first, then the tip; a gripper_before that OPENS the gripper opens it WHILE the tip moves, as one motion\) or "gripper_after" \(the tip moves first, then the gripper\)/);
  Assert.match(Prompt, /the second only once the first has finished \(an opening gripper_before: both at once\)/);
  Assert.match(Prompt, /So "gripper_before":0\.8 added to a target_cm move opens the gripper to 80% while the tip moves there;/);
  Assert.match(Prompt, /\{"move_delta_cm":\{"left":0,"farther":0,"up":6\},"gripper_before":0\.0\} closes the gripper and then lifts straight up/);
  Assert.match(Prompt, /Never chained \(the app refuses them\): the descent to an object's grasp point \(always its own motion, so the next image shows the open jaws beside it before the close, unless the gripper hides it there\), a close after a tip move, a close followed by anything but a straight lift/);
  Assert.match(Prompt, /and opening or narrowing the gripper beside an object below its top \(not one the jaws hold: putting a held object down and letting go may be chained\)\. A chained open goes to at most 0\.8: a chained fraction above 0\.8 runs to 0\.8\./);
  Assert.match(Prompt, /A chained motion counts as ONE of your motions\./);
  Assert.match(Prompt, /GRASP SEQUENCE: open the gripper and go to the GRASP PLAN's hover point \(one chained motion\), descend straight down to its grasp point with the jaws open \(always its own motion\), then close the gripper .* and lift \(one chained motion/);
  Assert.match(Prompt, /move above that place and open the gripper there in one chained motion \("gripper_after":0\.8 with that move\)/);
  Assert.match(Prompt, /You have at most 8 motions per task \(a chained motion counts as one\)/);
  // The prompt's own close-and-lift JSON is what the NEXT STEP offers, and parses as a chain.
  Assert.equal(Api.CloseAndLiftJson(6), '{"move_delta_cm":{"left":0,"farther":0,"up":6},"gripper_before":0.0}');
  const Open80 = GripperAt(0.8);
  Assert.equal(ChainOf(Open80, JSON.parse(Api.CloseAndLiftJson(6))).chain.Order, 'before');
});

Test('2026-10-01 18:54: the prompt puts the NEXT STEP first: copied exactly, no skipping, no descent it does not give, ask to disagree', () => {
  const Prompt = App.AiPrompt('', true, true, 'Behind');
  // The rule, right after the GRASP RULE that introduces the GRASP PLAN.
  // The refusal is claimed only for a step the app holds the arm to: the one the NEXT STEP TO COPY line repeats.
  Assert.match(Prompt, /\nNEXT STEP RULE \(the most important rule\): when the TASK OBJECT line gives a NEXT STEP with a "motion_proposal", your motion_proposal is that JSON copied exactly, every key and number\. Do not skip it, reorder it, merge it with another step or improve it, and never descend unless the NEXT STEP says descend\. When the message ends with a NEXT STEP TO COPY line \(a grasp the app planned\), the app refuses any other move that ends low beside the task object\. If you think the NEXT STEP is wrong, reply task_status "ask" with motion_proposal null and say why\.\n/);
  Assert.doesNotMatch(Prompt, /the app refuses any other move near the task object/);
  Assert.ok(Prompt.indexOf('NEXT STEP RULE') > Prompt.indexOf('GRASP RULE:') && Prompt.indexOf('NEXT STEP RULE') < Prompt.indexOf('HARD FLOOR:'));
  // The approach no longer tells it to descend at the next check-in on its own (the 18:54 model descended from a hover the step was correcting).
  Assert.doesNotMatch(Prompt, /At the next check-in, check in the image that the open gripper is above the side|only THEN descend/);
  Assert.match(Prompt, /For a grasp, the NEXT STEP takes you there one motion at a time: first the GRASP PLAN's HOVER POINT \(straight above the GRASP POINT, which is beside the object, not above its centre\), then, only when the NEXT STEP says descend, straight down to the GRASP POINT as its own separate motion\./);
  Assert.match(Prompt, /You do not pick these steps: the NEXT STEP gives the one for the arm as it is now, chained or not, and repeats or corrects a step when the arm stopped off its point \(the hover point aimed past it, a rise first\)\./);
  // The check-in: "change your approach" is not leave to skip the NEXT STEP.
  Assert.match(Api.BuildTaskCheckInPrompt({ task: 'grab the can', step: 2 }, [], null), /change your approach or reply ask; when the TASK OBJECT line gives a NEXT STEP, that step is the approach: copy it\./);
  // The prompt's last line repeats the step's JSON verbatim (read from the TASK OBJECT line as a reply would be).
  const Line = 'TASK OBJECT "can": NOT detected this check-in. NEXT STEP of the grasp (tip now X 7.6, Y 31.1, Z 11.9; gripper 80% open): the arm holds the HOVER POINT but its tip stopped 0.5 cm toward the can. Do not descend from here: go to the hover point aimed 0.7 cm past it, away from the can, so the arm stops at it: "motion_proposal":{"target_cm":{"x":8.7,"y":30.5,"z":12.7},"wrist_pitch_degrees":-18.1}. Copy it exactly (one step per motion, never skipping ahead); the next check-in gives the step after it. Coordinates in earlier replies are stale: do not reuse them.';
  const Step = Api.PlannedStepOf(Line, { x: 7.62, y: 31.07, z: 11.9 }, -19.95);
  Assert.deepEqual([Step.Json, Step.Target, Step.Pitch], ['{"target_cm":{"x":8.7,"y":30.5,"z":12.7},"wrist_pitch_degrees":-18.1}', { x: 8.7, y: 30.5, z: 12.7 }, -18.1]);
  Assert.ok(Step.Text.startsWith('NEXT STEP of the grasp (tip now X 7.6') && Step.Text.endsWith('the next check-in gives the step after it.'));
  Assert.equal(Api.NextStepReminder(Step, 'can'), 'NEXT STEP TO COPY (from the TASK OBJECT line; the app refuses any other move near the can): "motion_proposal":{"target_cm":{"x":8.7,"y":30.5,"z":12.7},"wrist_pitch_degrees":-18.1}. Copy it exactly as your motion_proposal, or reply task_status "ask" with motion_proposal null if you think it is wrong.');
  // A relative step ends where the same reply would (the anchor plus its deltas); a step without its wrist keeps the tip's.
  const Rise = Api.PlannedStepOf('NEXT STEP of the grasp (x): rise straight up first: "motion_proposal":{"move_delta_cm":{"left":0,"farther":0,"up":6.5}}. Copy it exactly.', { x: 7.6, y: 31, z: 6.2 }, -19.5);
  Assert.deepEqual([Rise.Target, Rise.Pitch], [{ x: 7.6, y: 31, z: 12.7 }, -19.5]);
  // No step, or one that gives no single JSON (reply ask; a narrow object's lift-or-open): no reminder, the sentence kept.
  Assert.equal(Api.PlannedStepOf('TASK OBJECT "can": NOT detected in any check-in of this task.', { x: 0, y: 0, z: 0 }, 0), null);
  const Ask = Api.PlannedStepOf('NEXT STEP of the grasp (x): reply ask: the can may have moved into the gripper\'s way.', { x: 0, y: 0, z: 0 }, 0);
  Assert.deepEqual([Ask.Json, Ask.Target, Api.NextStepReminder(Ask, 'can')], [null, null, null]);
  const Two = Api.PlannedStepOf('NEXT STEP of the grasp (x): lift it on: "motion_proposal":{"move_delta_cm":{"left":0,"farther":0,"up":3}}, or open: "motion_proposal":{"direct_servo_targets":[{"name":"gripper","target_fraction":0.8}]}.', { x: 0, y: 0, z: 0 }, 0);
  Assert.equal(Two.Json, null);
  // A gripper-only step has no tip move.
  const Open = Api.PlannedStepOf('NEXT STEP of the grasp (x): open the gripper: "motion_proposal":{"direct_servo_targets":[{"name":"gripper","target_fraction":0.8}]}. Copy it exactly.', { x: 1, y: 2, z: 3 }, -10);
  Assert.deepEqual([Open.Json, Open.Target, Open.Pitch], ['{"direct_servo_targets":[{"name":"gripper","target_fraction":0.8}]}', null, null]);
});

// --- TOGETHER (lib/AiChain.ts): an opening gripper_before runs as ONE motion with its tip move ------------

Test('an opening gripper_before is ONE send_ik carrying the gripper target; a close, a narrowing or a gripper_after stays two legs', () => {
  const Closed = GripperAt(0.03), Open80 = GripperAt(0.8);
  const Open = ChainOf(Closed, { move_delta_cm: { left: 0, farther: 0, up: 4 }, gripper_before: 0.8 });
  const OpenRaw = Api.fractionToRaw(Closed.gripper, 0.8);
  // With a bridge that takes the gripper with a tip move: one leg, the tip move carrying the opening.
  Assert.deepEqual(App.ChainLegs(Open, true), [{ Kind: 'together', Target: Open.target, TargetRaw: OpenRaw, Fraction: 0.8 }]);
  Assert.deepEqual(App.IkCommand(Open.target, OpenRaw), { type: 'send_ik', gripper_raw: OpenRaw,
    target: { x: Open.target.x, y: Open.target.y, z: Open.target.z, wrist_pitch: Open.target.wristPitch } });
  Assert.equal('gripper_raw' in App.IkCommand(Open.target), false);
  // An older bridge (no gripper_with_tip_version): the two legs, as before.
  Assert.deepEqual(App.ChainLegs(Open, false).map(Leg => Leg.Kind), ['gripper', 'tip']);
  // The close before a straight lift: closing while the tip rises would lift before the jaws hold anything.
  const CloseLift = ChainOf(Open80, JSON.parse(Api.CloseAndLiftJson(6)));
  Assert.equal(CloseLift.chain.Together, undefined);
  Assert.deepEqual(App.ChainLegs(CloseLift, true).map(Leg => Leg.Kind), ['gripper', 'tip']);
  // Narrowing to a plan's 60% and an open after the move stay in their order too.
  const Narrow = ChainOf(Open80, { move_delta_cm: { left: 0, farther: 0, up: 4 }, gripper_before: 0.6 });
  Assert.deepEqual([Narrow.chain.Together, App.ChainLegs(Narrow, true).map(Leg => Leg.Kind)], [undefined, ['gripper', 'tip']]);
  const After = ChainOf(Closed, { move_delta_cm: { left: 0, farther: 0, up: 4 }, gripper_after: 0.8 });
  Assert.deepEqual([After.chain.Together, App.ChainLegs(After, true).map(Leg => Leg.Kind)], [undefined, ['tip', 'gripper']]);
  Assert.deepEqual([Api.ChainRunsTogether('before', 0.03, 0.8), Api.ChainRunsTogether('before', 0.8, 0), Api.ChainRunsTogether('before', 0.8, 0.6),
    Api.ChainRunsTogether('after', 0.03, 0.8)], [true, false, false, false]);
  // The card: one step; with an older bridge, the two.
  const Card = App.ChainCardSteps(Open, Closed, true);
  Assert.equal(Card.length, 1);
  Assert.match(Card[0], /^Open the gripper to 80% \(raw \d+\) while moving the tip to X -?[\d.]+ Y [\d.]+ Z [\d.]+ cm, wrist -?[\d.]+ deg$/);
  Assert.deepEqual(App.ChainCardSteps(Open, Closed, false).map(Step => Step.split(' ')[0]), ['Open', 'Move']);
  Assert.deepEqual(App.ChainCardSteps(CloseLift, Open80, true).map(Step => Step.split(' ')[0]), ['Close', 'Move']);
});

Test('a chain run as one motion is told to the model as one: what ran together, or that it did not run', () => {
  const Tip = 'move the tip to X 1.9, Y 28.9, Z 11.5 cm, wrist -21.3 deg';
  Assert.equal(Api.ChainOrderText('before', 'open the gripper to 80%', Tip, true), 'open the gripper to 80% while moving the tip to X 1.9, Y 28.9, Z 11.5 cm, wrist -21.3 deg');
  Assert.equal(Api.ChainOrderText('before', 'open the gripper to 80%', Tip), `first open the gripper to 80%, then ${Tip}`);
  const Run = (Results, Why = null) => Api.DescribeChainRun({ Order: 'before', Gripper: 'open the gripper to 80%', Tip, Results, Why, Together: true });
  Assert.equal(Run(['sent', null]), 'It was a chained motion, run as ONE motion: open the gripper to 80% while moving the tip to X 1.9, Y 28.9, Z 11.5 cm, wrist -21.3 deg; the gripper opened on the way.');
  Assert.equal(Run(['refused', null], 'the bridge refused it: No route.'), 'It was a chained motion, run as ONE motion: open the gripper to 80% while moving the tip to X 1.9, Y 28.9, Z 11.5 cm, wrist -21.3 deg; it did not run: it was refused by the bridge: the bridge refused it: No route.');
  Assert.match(Api.DescribeBridgeRefusal(1, 'No route.', null, Run(['refused', null])), /was NOT executed: the bridge refused it: No route\. The arm did not move\. It was a chained motion, run as ONE motion: .*it did not run/);
  const Fact = Api.DescribeMotionFact({ Step: 1, StartTip: { x: -13.2, y: 20.2, z: 12 }, EndTip: { x: 1.9, y: 28.7, z: 11 }, Target: { x: 1.9, y: 28.9, z: 11.5 },
    Servos: 'gripper now at raw 1914 (commanded 1922)', Chain: Run(['sent', null]) });
  Assert.match(Fact, /gripper now at raw 1914 \(commanded 1922\)\. It was a chained motion, run as ONE motion: open the gripper to 80% while moving the tip .*; the gripper opened on the way\.$/);
});

Test('2026-10-03 14:46: "gripper_before":0.8 with the hover point above the can replays as ONE motion: one send_ik that opens the gripper to raw 1922 on the way', () => {
  // Every servo when it was approved (the model request; the gripper closed at 704).
  const S1446 = Snapshot({ shoulder_pan: 3474, shoulder_lift: 3171, elbow_flex: 2688, wrist_flex: 855, wrist_roll: 1570, gripper: 704 });
  const Motion = ChainOf(S1446, { target_cm: { x: 1.9, y: 28.9, z: 11.5 }, wrist_pitch_degrees: -21.3, gripper_before: 0.8 });
  Assert.deepEqual(Motion.chain, { Order: 'before', TargetRaw: 1922, Fraction: 0.8, Together: true });
  Assert.deepEqual(App.Validate(Motion, 'up', S1446).issues, []);
  const Legs = App.ChainLegs(Motion, true);
  Assert.deepEqual(Legs.map(Leg => Leg.Kind), ['together']);
  const Command = App.IkCommand(Legs[0].Target, Legs[0].TargetRaw);
  Assert.equal(Command.type, 'send_ik');
  Assert.equal(Command.gripper_raw, 1922);
  const Display = Api.mirrorDisplayPoint({ x: Command.target.x, y: Command.target.y, z: Command.target.z });
  Assert.ok(Math.hypot(Display.x - 1.9, Display.y - 28.9, Display.z - 11.5) < 1e-6, JSON.stringify(Display));
  Assert.equal(Command.target.wrist_pitch, -21.3);
  // What ran at 14:46 (an older bridge): the open, then the tip move the bridge refused.
  Assert.deepEqual(App.ChainLegs(Motion, false).map(Leg => Leg.Kind), ['gripper', 'tip']);
});

Test('opening on the way, the moving jaw is checked at every opening along the move: an object only the half-open jaw would reach refuses it as one motion', () => {
  const Closed = GripperAt(0.03);
  const Open = ChainOf(Closed, { move_delta_cm: { left: 0, farther: 0, up: 4 }, gripper_before: 0.8 });
  const Plain = { ...Open, chain: { ...Open.chain, Together: undefined } };
  const End = App.Validate(Open, 'up', Closed).preview.angles;
  const From = Api.angleFromRawValue(Closed.gripper, Closed.gripper.presentRaw), To = Api.angleFromRawValue(Closed.gripper, Open.chain.TargetRaw);
  // Small objects where the moving jaw is a third of the way open at the move's end, farthest from the open jaw first.
  const Mid = From + 0.3 * (To - From);
  const OpenJaw = App.GripperSamples({ ...End, gripper: To }, [To, To - App.GraspRules.JawUncertaintyDeg]).filter(Sample => Sample.Part === 'MovingJaw').map(Sample => Sample.P);
  const Far = P => Math.min(...OpenJaw.map(Q => Math.hypot(P.x - Q.x, P.y - Q.y, P.z - Q.z)));
  const Candidates = App.GripperSamples({ ...End, gripper: Mid }, [Mid]).filter(Sample => Sample.Part === 'MovingJaw').map(Sample => Sample.P)
    .sort((A, B) => Far(B) - Far(A)).slice(0, 12);
  const Guard = P => ({ Prisms: [{ Label: 'pen', Footprint: [{ x: P.x - 0.1, y: P.y - 0.1 }, { x: P.x + 0.1, y: P.y - 0.1 }, { x: P.x + 0.1, y: P.y + 0.1 }, { x: P.x - 0.1, y: P.y + 0.1 }],
    BaseZ: P.z - 0.1, TopZ: P.z + 0.1, SupportZ: P.z - 0.1, Centre: { x: P.x, y: P.y }, SizeCm: [0.2, 0.2] }],
    TaskPrism: null, Plan: null, Refusal: null, Held: false, Intent: 'grasp', TaskLabel: 'can', Step: null });
  const Found = Candidates.find(P => App.Validate(Plain, 'up', Closed, [], undefined, null, Guard(P)).issues.length === 0);
  Assert.ok(Found, 'an object the open jaw clears on the way');
  // Sent as two legs (the jaws open first) it passes; opening on the way, the half-open jaw would reach it.
  const Issues = App.Validate(Open, 'up', Closed, [], undefined, null, Guard(Found)).issues;
  Assert.equal(Issues.length, 1, Issues.join(' '));
  Assert.match(Issues[0], /^Opening the gripper on the way there would swing its moving jaw (within [\d.]+ cm of|[\d.]+ cm into) the pen measured at X -?[\d.]+, Y -?[\d.]+ \(from \d+% of the way there\): open the gripper alone first, then move\.$/);
  // No object in its way: nothing to refuse.
  Assert.deepEqual(App.Validate(Open, 'up', Closed, [], undefined, null, { ...Guard(Found), Prisms: [] }).issues, []);
});

Test('a closing chain stays two legs, and its tip move waits until the jaws hold still (2026-10-03 14:46: 3 steps of creep refused it)', () => {
  const Frames = (From, Rate, Start, End) => Array.from({ length: Math.round((End - Start) / 33) + 1 },
    (_, Index) => ({ At: Start + Index * 33, Raw: Math.round(From + Rate * Index * 33 / 1000) }));
  // Creeping 4 steps/s (1916 to 1919 in 0.8 s, as at 14:46): not still yet.
  const Creep = Frames(1916, 4, 0, 1000);
  Assert.equal(Api.ChainGripperStill(Creep, 1000, 900), false);
  // ...until ChainGripperStillWaitMs after the leg settled (the bridge allows a little creep itself).
  Assert.equal(Api.ChainGripperStill(Creep, 1000, 1000 - Api.ChainGripperStillWaitMs), true);
  // Still, within a step of noise for the whole window: the tip move goes.
  const Still = Frames(1919, 0, 0, 1000).map((Sample, Index) => ({ ...Sample, Raw: Sample.Raw + (Index % 2) }));
  Assert.equal(Api.ChainGripperStill(Still, 1000, 900), true);
  // No reading from before the window, or none in it (the state stream stalled): not yet.
  Assert.equal(Api.ChainGripperStill(Frames(1919, 0, 700, 1000), 1000, 900), false);
  Assert.equal(Api.ChainGripperStill(Frames(1919, 0, 0, 500), 1000, 900), false);
  Assert.deepEqual([Api.ChainGripperStillMs, Api.ChainGripperStillRaw], [400, 1]);
});
