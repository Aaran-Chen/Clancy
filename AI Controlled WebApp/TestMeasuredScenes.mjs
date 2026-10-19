import { build } from 'esbuild';
import { readFileSync } from 'node:fs';
import { createRequire } from 'node:module';
import { test } from 'node:test';
import assert from 'node:assert/strict';
import Path from 'node:path';
import { fileURLToPath } from 'node:url';
import { PinnedHome } from './UiTestLib.mjs';

const root = Path.dirname(fileURLToPath(import.meta.url));
const bundle = await build({ stdin: { contents: readFileSync(Path.join(root, 'src/App.tsx'), 'utf8')
  + '\nexport { ikCommand, buildAiSystemPrompt, parseAiResponse, validateAiMotionProposal, locateObjects };\nexport * from "./lib/AiMeasurements";\nexport * from "./lib/SceneGeometry";\nexport {INITIAL_SERVOS} from "./lib/robotModel";',
  resolveDir: Path.join(root, 'src'), loader: 'tsx' }, bundle: true, platform: 'node', format: 'cjs', write: false,
  loader: { '.css': 'empty' }, plugins: [PinnedHome], logLevel: 'error' });
globalThis.window ??= { location: { hostname: 'localhost' } };
const mod = { exports: {} };
new Function('require', 'module', 'exports', bundle.outputFiles[0].text)(createRequire(import.meta.url), mod, mod.exports);
export const api = mod.exports;
const support = { Name: 'observed shelf', Kind: 'shelf', TopZCm: 12, XRangeCm: [-25, -3], YRangeCm: [25, 45],
  BoardAboveZCm: 33, Frame: 'cart', Source: 'depth', ObservedOnly: true };
const scene = { id: 'capture-a', revision: 'geometry-a', frame: 'display_cm', capture_seq: 17, support,
  obstacles: [{ label: 'shelf board', min: [-25, 25, 10], max: [-3, 45, 12] }],
  free_space: [{ min: [-30, 10, 12.1], max: [0, 48, 32] }], task_object: { min: [-15, 32, 12], max: [-11, 36, 19] } };
const object = { label: 'bottle', x: -13, y: 34, z: 12, top_z: 19, height_cm: 7, size_cm: [4, 4], method: 'depth',
  support_measured: true, task_match: true, support_branch: 'surface', support_surface: 'observed shelf', support_geometry: support, scene_id: scene.id };
const reply = (changes = {}) => ({ ok: true, objects: [object], scene, camera_check: { ok: true }, ...changes });

test('capture geometry and depth provenance replace per-shelf declarations', () => {
  const decoded = api.ReadLocateResponse(reply());
  assert.equal(decoded.error, null);
  assert.equal(decoded.scene.id, scene.id);
  const measured = decoded.objects[0];
  const result = api.TaskSupport(measured, 'bottle', []);
  assert.equal(result.Refusal, null);
  assert.equal(result.Floor.Scene.id, scene.id);
  assert.equal(result.Floor.Surface.TopZCm, 12);
  assert.match(api.MeasuredObjectsSummary(decoded), /shelf measured in this capture/);
  assert.match(api.buildAiSystemPrompt('', true, true, 'front', result.Floor), /SCENE LIMITS/);
  assert.match(api.buildAiSystemPrompt('', true, true, 'front', result.Floor), /Close alone/);
});

// Scenes are optional (2026-10-14, "just remove the scene verification thing"): a scene that does not decode or bind
// is dropped, and its objects read as they did before measured scenes, desk objects with the desk's rules.
test('malformed, cross-capture or partial geometry is dropped: the objects read as desk objects, nothing stops', () => {
  for (const bad of [reply({ scene: { ...scene, frame: 'camera_cm' } }), reply({ scene: { ...scene, revision: '' } }),
    reply({ scene: { ...scene, obstacles: [{ label: 'bad', min: [0, 0, 0], max: [1, 1, NaN] }] } }),
    reply({ objects: [{ ...object, scene_id: 'older' }] }), reply({ objects: [{ ...object, scene_id: undefined }] }),
    reply({ objects: [{ ...object, support_geometry: { ...support, ObservedOnly: false } }] }),
    reply({ scene: undefined }), reply({ scene: { ...scene, free_space: [{ min: [0, 0, 0], max: [-1, 1, 1] }] } })]) {
    const decoded = api.ReadLocateResponse(bad);
    assert.equal(decoded.errorCode, null);
    assert.equal(decoded.error, null);
    assert.equal(decoded.scene, null);
    assert.match(decoded.sceneNote, /read as desk objects/);
    assert.equal(decoded.objects.length, 1);
    const [bottle] = decoded.objects;
    assert.equal('supportGeometry' in bottle || 'scene' in bottle, false, 'no scene geometry is kept from it');
    assert.equal(bottle.supportBranch, 'object', 'its observed-only support reads as resting on something');
    assert.equal(api.CameraGateFailure(decoded, true), null);
    const support = api.TaskSupport(bottle, 'bottle', []);
    assert.equal(support.Refusal, null);
    assert.equal(support.Floor.Scene ?? null, null);
    assert.equal(support.Floor.Surface ?? null, null, 'the desk floor');
  }
});

test('unknown free space and unmeasured depth remain explicit refusals', () => {
  const decoded = api.ReadLocateResponse(reply({ scene: { ...scene, free_space: [] } }));
  assert.match(api.TaskSupport(decoded.objects[0], 'bottle', []).Refusal.Text, /clear approach/);
  const measured = api.ReadLocateResponse(reply()).objects[0];
  assert.ok(api.TaskSupport({ ...measured, depthMeasured: false }, 'bottle', []).Refusal);
  assert.ok(api.TaskSupport({ ...measured, supportMeasured: false }, 'bottle', []).Refusal);
  assert.ok(api.TaskSupport({ ...measured, z: 5 }, 'bottle', []).Refusal);
});

test('scene identity is required on IK and servo command payloads, with explicit release', () => {
  const fields = api.SceneCommandFields(scene, true);
  assert.deepEqual(fields, { scene_id: 'capture-a', scene_required: true, scene_carry: true });
  const command = api.ikCommand({ x: 1, y: 2, z: 3, wristPitch: -20 }, null, scene, true);
  assert.equal(command.scene_id, scene.id);
  assert.equal(command.scene_required, true);
  assert.equal(command.scene_carry, true);
  assert.equal(api.SceneCommandFields(scene, false).scene_carry, false);
  assert.deepEqual(api.SceneCommandFields(null), {});
  assert.equal(api.SceneFreshnessIssue(scene, { ...scene }), null);
  assert.ok(api.SceneFreshnessIssue(scene, { ...scene, revision: 'geometry-b' }));
  assert.ok(api.SceneFreshnessIssue(scene, { ...scene, id: 'capture-b' }));
  assert.ok(api.SceneFreshnessIssue(scene, null));
  assert.equal(api.SceneFreshnessIssue(null, null), null);
});

const plan = { Grasp: { x: -13, y: 34, z: 15 }, Hover: { x: -13, y: 24, z: 15 }, HoverPitch: -20, Pitch: -20, OpenFraction: 0.8,
  WidthAcrossCm: 4, Approach: { Kind: 'side', Pitch: -20, Pregrasp: { x: -13, y: 24, z: 15 },
    Release: { x: -13, y: 34, z: 16 }, Retreat: { x: -13, y: 24, z: 16 } } };
const stateAt = (point, closed = false, fraction = 0.8) => ({ Closed: closed, GripperFraction: fraction, Held: { Point: point, Pitch: -20 } });
const step = (point, closed = false, fraction = 0.8) => api.NextPlannedGraspStep(plan, point, stateAt(point, closed, fraction), 'bottle');
test('shelf steps close alone, then release clearance, then withdraw at the held pitch', () => {
  assert.match(step(plan.Approach.Pregrasp), /enter from the open side/);
  const close = step(plan.Grasp);
  assert.match(close, /close the gripper only/);
  assert.doesNotMatch(close, /gripper_before|gripper_after|move_delta_cm/);
  const release = step(plan.Grasp, true, 0.4);
  assert.match(release, /release clearance/);
  assert.match(release, /"z":16.0/);
  assert.doesNotMatch(release, /withdraw/);
  assert.match(step(plan.Approach.Release, true, 0.4), /withdraw along the checked path/);
  assert.match(step(plan.Approach.Retreat, true, 0.4), /task_status "done"/);
  assert.match(step(plan.Grasp, true, 0), /reply ask/);
  assert.doesNotMatch(step(plan.Grasp, true, 0), /"motion_proposal"/);
});

test('legacy desk replies and manual command payloads retain their contract', () => {
  const decoded = api.ReadLocateResponse({ ok: true, objects: [{ ...object, scene_id: undefined, support_geometry: undefined,
    support_branch: 'desk', task_match: false }], camera_check: { ok: true } });
  assert.equal(decoded.error, null);
  assert.equal(decoded.scene, null);
  assert.equal(api.TaskSupport(decoded.objects[0]).Refusal, null);
  assert.ok(!('scene_id' in api.ikCommand({ x: 1, y: 2, z: 3, wristPitch: 0 })));
});

test('robot-referenced alignment failure explains the visible reference, not a desk calibration', () => {
  const response = api.ReadLocateResponse({ ok: false, error_code: 'camera_not_verified', error: 'Keep the robot reference clearly in view.',
    camera_check: { ok: false, alignment_source: 'robot', reason: 'Robot reference is hidden.' } });
  assert.equal(api.CameraGateFailure(response, true).Title, 'Keep the robot reference clearly in view.');
  assert.equal(api.CameraGateFailure(response, false), null);
});


test('held-object check-in identifies the prior capture without sending client geometry', async () => {
  const original = globalThis.fetch;
  let request;
  globalThis.fetch = async (_url, options) => { request = JSON.parse(options.body); return { ok: true, json: async () => reply() }; };
  try {
    await api.locateObjects('data:image/png;base64,AA==', 18, 'test', false, undefined, { Phrase: 'the bottle', Name: 'bottle' }, scene);
    assert.deepEqual(request.task_object, { phrase: 'the bottle', label: 'bottle', carrying: true, carry_scene_id: scene.id });
    assert.ok(!('scene' in request) && !('free_space' in request));
  } finally { globalThis.fetch = original; }
});


test('unrelated legacy desk observations remain obstacles alongside a scene-bound target', () => {
  const result = api.ReadLocateResponse(reply({ objects: [object, { label: 'cup', x: 5, y: 35, z: -2.3,
    top_z: 6, size_cm: [4, 4], method: 'depth', support_branch: 'desk', task_match: false }] }));
  assert.equal(result.error, null);
  assert.equal(result.objects.length, 2);
  assert.equal(result.objects[0].scene.id, scene.id);
  assert.equal(result.objects[1].scene, undefined);
  // A task match the scene does not bind: the scene is not this task's, so the cup is a desk object.
  const unbound = api.ReadLocateResponse(reply({ objects: [{ label: 'cup', x: 5, y: 35, z: -2.3,
    method: 'depth', support_branch: 'desk', task_match: true }] }));
  assert.equal(unbound.errorCode, null);
  assert.equal(unbound.scene, null);
  assert.equal(unbound.objects[0].label, 'cup');
  assert.equal(api.TaskSupport(unbound.objects[0]).Refusal, null);
});


test('an unrelated measured support without the scene reads as before scenes, under the desk rules', () => {
  const result = api.ReadLocateResponse(reply({ objects: [object, { ...object, label: 'other bottle',
    x: -18, scene_id: undefined, task_match: false }] }));
  assert.equal(result.error, null);
  assert.equal(result.objects[0].scene.id, scene.id, 'the task object keeps its scene');
  assert.equal(result.objects[1].scene, undefined);
  assert.equal(result.objects[1].supportBranch, 'object');
  const support = api.TaskSupport(result.objects[1]);
  assert.equal(support.Refusal, null);
  assert.equal(support.Floor.Scene ?? null, null);
});

test('registered metric depth is independent of the old desk sample hull, without discarding uncertainty gates', () => {
  const raw = { ...object, extrapolated: true, outside_calibration_cm: 12, scene_checked: true,
    localization_frame: 'calibrated_depth_3d', refined: true };
  const measured = api.ReadLocateResponse(reply({ objects: [raw] })).objects[0];
  assert.equal(measured.extrapolated, true, 'diagnostic preserved');
  assert.equal(api.IsUnreliable(measured), false);
  assert.equal(api.IsUnreliable({ ...measured, unstable: true }), true);
  assert.equal(api.IsUnreliable({ ...measured, taskQueryOnly: true }), true);
  assert.equal(api.IsUnreliable({ ...measured, sceneChecked: false }), true);
  assert.equal(api.IsUnreliable({ ...measured, depthMeasured: false }), true);
  assert.equal(api.IsUnreliable({ ...measured, scene: null }), true);
  const unchecked = api.ReadLocateResponse(reply({ objects: [raw], camera_check: { ok: false } })).objects[0];
  assert.equal(api.IsUnreliable(unchecked), true);
  // With no calibrated desk in view: a camera the gripper tip verified waives the tag like a desk-verified one; a camera
  // it could not check ("assumed") lets the task run but keeps the tag (Aaran's decision, 2026-10-07 audit IH1).
  const robot = check => ({ ok: true, alignment_source: 'robot', robot_check: check, tape_check: check === 'verified' ? 'ok' : 'not_visible' });
  const verified = api.ReadLocateResponse(reply({ objects: [raw], camera_check: robot('verified') })).objects[0];
  assert.equal(api.IsUnreliable(verified), false);
  const assumed = api.ReadLocateResponse(reply({ objects: [raw], camera_check: robot('assumed') }));
  assert.equal(assumed.error, null, 'the task runs');
  assert.equal(assumed.objects[0].sceneChecked, false);
  assert.equal(api.IsUnreliable(assumed.objects[0]), true, 'its extrapolated measurement stays [UNRELIABLE]');
  // And a bridge that marks the assumed scene unchecked itself (scene_checked false) reads the same.
  const marked = api.ReadLocateResponse(reply({ objects: [{ ...raw, scene_checked: false }], camera_check: robot('verified') })).objects[0];
  assert.equal(api.IsUnreliable(marked), true);
});

test('an unregistered measured support stops nothing: no camera gate, no scene fields on any command', () => {
  const located = api.ReadLocateResponse(reply({ scene: null, scene_error: 'a unique task object and its supporting surface were not measured',
    objects: [{ ...object, scene_id: undefined, task_match: false }] }));
  assert.equal(located.errorCode, null);
  assert.match(located.sceneNote, /no scene registered \(a unique task object/);
  assert.equal(api.CameraGateFailure(located, false), null);
  assert.equal(api.CameraGateFailure(located, true), null);
  assert.equal(api.MotionSceneIssue, undefined, 'no scene-only motion refusal');
  assert.deepEqual(api.SceneCommandFields(located.scene), {}, 'the bridge plans it without a scene, as before scenes');
  assert.equal(api.CameraGateCodes.includes('scene_unverified'), false);
});

test("the scene's task contact is read with it; malformed, the scene is no scene", () => {
  // back end/SceneGeometry.py TaskContact: an observed solid against the task object, null where none, absent from an older bridge.
  const contact = { points: 334, min: [-29.9, 43.08, 10.32], max: [-26.2, 43.58, 13.43] };
  assert.deepEqual(api.ReadMeasuredScene({ ...scene, task_contact: contact }).task_contact, contact);
  assert.equal(api.ReadMeasuredScene({ ...scene, task_contact: null }).task_contact, null);
  assert.equal('task_contact' in api.ReadMeasuredScene(scene), false);
  for (const bad of [{ ...contact, points: 'many' }, { ...contact, max: [-29.9, 43.58, 13.43] }, 5, { points: 3 }]) {
    assert.equal(api.ReadMeasuredScene({ ...scene, task_contact: bad }), null, JSON.stringify(bad));
  }
});
