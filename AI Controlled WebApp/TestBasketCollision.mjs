import { build as Build } from "esbuild";
import { chromium as Chromium } from "playwright";
import { CadReadyMs, LaunchOptions, StartUiServer } from "./UiTestLib.mjs";
import { execFileSync as Execute } from "node:child_process";
import { readFileSync as ReadFile } from "node:fs";
import { createRequire as CreateRequire } from "node:module";
import Path from "node:path";
import { fileURLToPath as FilePath } from "node:url";
import Assert from "node:assert/strict";

const Root = Path.dirname(FilePath(import.meta.url)), Backend = Path.resolve(Root, "../back end");
const Python = ReadFile(Path.join(Root, "../.backend-python.txt"), "utf8").trim();
const Fixtures = JSON.parse(Execute(Python, ["-c", `
import json, numpy as Np
import bridge_server as Bridge, BasketCollision as Collision
from TestArmMotionEnvelope import ClearPoses
C = Bridge.BridgeController()
W = Collision.CollisionWorld(C.arm_geometry, json.load(open('WorkspaceFrame.json')))
Random = Np.random.default_rng(22)
Limits = {N: sorted((C._ik_angle_from_raw(S, S.range_min), C._ik_angle_from_raw(S, S.range_max))) for N,S in C.servos.items()}
Records = []
for I in range(700):
 A = {N: float(Random.uniform(*Limits[N])) if N in Bridge.IK_JOINTS else C._current_ik_angles()[N] for N in Bridge.MOTOR_NAMES}
 Tip = Collision.Cad.Positions(C.arm_geometry,A)[-1]
 Records.append({'Angles':A,'Reason':W.PoseReason(A), 'Target': {**Tip,'wristPitch':Collision.Cad.Pitch(C.arm_geometry,A)}})
for Values in ClearPoses:
 A = dict(zip(Collision.Names, Values))
 assert W.PoseReason(A) is None
 Records.append({'Angles':A,'Reason':None, 'Target': {**Collision.Cad.Positions(C.arm_geometry,A)[-1],
  'wristPitch':Collision.Cad.Pitch(C.arm_geometry,A)}})
Offset = Np.array(Collision.Model['Mounts']['Right']['PanCm'])
Points = [Np.array(Point) - Offset for Point in ([0,0,73],[0,0,76],[0,0,60],[0,0,68.5])]
Points += [Triangle.mean(axis=0) for Triangle in W.Surface.Triangles[::350]]
Points += [Triangle.mean(axis=0) for Triangle in W.BodySurface.Triangles[::120]]
Head = next(Part for Part in Collision.BodyData['Parts'] if Part['Name'] == 'XLehead_pan_link1')
HeadTarget = dict(zip(('x','y','z'), Np.mean(Head['BoundsCm'],axis=0) - Offset))
Targets = [dict(zip(('x','y','z'),Point)) for Point in Points]
print(json.dumps({'State':C.state_snapshot(),'Records':Records,'HeadTarget':HeadTarget,
 'SurfaceTargets':[{'Target':Point,'Reason':W.TargetReason(Point)} for Point in Targets]}))
`], { cwd: Backend, encoding: "utf8", maxBuffer: 4e6 }));
const Bundle = await Build({ stdin: { contents: `export * from './BasketCollision'; export * from './robotModel';`,
  resolveDir: Path.join(Root, "src/lib"), loader: "ts" }, bundle: true, platform: "node", format: "cjs", write: false });
const Module = { exports: {} };
new Function("require", "module", "exports", Bundle.outputFiles[0].text)(CreateRequire(import.meta.url), Module, Module.exports);
const Api = Module.exports;
for (const Record of Fixtures.Records) Assert.equal(Api.BasketPoseReason(Record.Angles), Record.Reason);
for (const Record of Fixtures.SurfaceTargets) Assert.equal(Api.BasketTargetReason(Record.Target), Record.Reason);
// This side reach is clear; crossing the cart footprint is no longer a blocker.
const WideReach = { shoulder_pan: -96.01400151436108, shoulder_lift: -13.181901323548914,
  elbow_flex: -73.94974264632789, wrist_flex: 58.93324029461547, wrist_roll: 94.39453125, gripper: 0 };
Assert.equal(Api.BasketSurfaceReason(WideReach), null);
Assert.equal(Api.BasketPoseReason(WideReach), null);
Assert.ok(Fixtures.Records.some(Record => Record.Reason === Api.RearwardReason), "Parity sample must include rearward poses");
for (const Y of [-30.001, -30, -29.999]) {
  Assert.equal(Api.BasketTargetReason({ x: 0, y: Y, z: 100 }), Y < -30 ? Api.RearwardReason : null);
}
let BlockedTarget;
for (const Record of Fixtures.Records) {
  if (!Record.Reason?.includes('basket') || Api.BasketTargetReason(Record.Target)) continue;
  const Preview = Api.solveIkPreview(Record.Target, Fixtures.State.elbow_mode, Api.INITIAL_SERVOS);
  if (Preview.collisionReason?.includes('basket') && Preview.errorCm < .1) { BlockedTarget = Record.Target; break; }
}
Assert.ok(BlockedTarget, "Need a reachable tip with a colliding arm pose");
console.log(`PASS: ${Fixtures.Records.length} CAD collision decisions match Python exactly`);

const Server = await StartUiServer();
const Browser = await Chromium.launch(LaunchOptions());
const Page = await Browser.newPage({ viewport: { width: 1920, height: 1080 } });
const State = Fixtures.State, Commands = [], Errors = [];
Page.on("pageerror", Error => Errors.push(Error.message));
await Page.route("**/bridge-launch/**", Route => Route.fulfill({ json: { ok: true } }));
await Page.route("**/ollama/**", Route => Route.fulfill({ json: { models: [] } }));
await Page.route("**/depth-server/**", Route => Route.fulfill({ json: {} }));
await Page.route("**:8000/**", Route => {
  if (!["GET", "HEAD"].includes(Route.request().method()) && !Route.request().url().endsWith('/ik/preview')) Commands.push(Route.request().postDataJSON());
  return Route.fulfill({ json: Route.request().url().endsWith('/ports') ? [] : { ok: false } });
});
let Timer;
await Page.routeWebSocket("**/ws", Socket => {
  Socket.send(JSON.stringify(State));
  Timer = setInterval(() => Socket.send(JSON.stringify(State)), 100);
  Socket.onMessage(Message => Commands.push(JSON.parse(String(Message))));
});
try {
  await Page.goto(Server.Url, { waitUntil: "domcontentloaded" });
  await Page.locator("canvas[data-cad-ready=true][data-assembly-ready=true]").waitFor({ timeout: CadReadyMs });
  const Inputs = Page.locator('.target-grid input');
  const Initial = await Inputs.evaluateAll(Values => Values.map(Input => Number(Input.value)));
  const PitchInput = Page.locator('.wide-field input'), InitialPitch = await PitchInput.inputValue();
  const ClearTarget = Fixtures.Records.at(-2).Target;
  const ClearVisual = { x: ClearTarget.x - State.workspace_offset.x,
    y: ClearTarget.y + State.workspace_offset.y, z: ClearTarget.z + State.workspace_offset.z };
  await Inputs.nth(2).fill('30');
  for (const [Index, Axis] of ['x', 'y', 'z'].entries()) await Inputs.nth(Index).fill(String(ClearVisual[Axis]));
  await PitchInput.fill(String(ClearTarget.wristPitch));
  await Page.waitForFunction(() => Array.from(document.querySelectorAll('button'))
    .some(Button => Button.textContent.trim() === 'Send' && !Button.disabled));
  Assert.equal(Commands.length, 0, 'Previewing a formerly blocked clear pose must not move hardware');
  await Inputs.nth(2).fill('30');
  for (let Index = 0; Index < 3; Index++) await Inputs.nth(Index).fill(String(Initial[Index]));
  await PitchInput.fill(InitialPitch);
  // An invalid draft is visible while editing but cannot be sent; release snaps.
  await Inputs.nth(2).fill("-5");
  Assert.equal(Number(await Inputs.nth(2).inputValue()), -5);
  Assert.equal(await Page.getByRole('button', { name: 'Send', exact: true }).isDisabled(), true);
  await Inputs.nth(2).press("Enter");
  await Page.waitForFunction(() => document.querySelector('canvas[data-cad-ready=true]')?.dataset.targetState === 'valid');
  const Snapped = await Inputs.evaluateAll(Values => Values.map(Input => Number(Input.value)));
  const Model = Api.modelPointFromDisplayPoint({ x: -Snapped[0], y: Snapped[1], z: Snapped[2] });
  Assert.equal(Api.BasketTargetReason(Model), null);
  // Set XYZ first with a high Z to avoid crossing the keep-out while staging.
  await Inputs.nth(2).fill("30");
  const Visual = { x: BlockedTarget.x - State.workspace_offset.x,
    y: BlockedTarget.y + State.workspace_offset.y, z: BlockedTarget.z + State.workspace_offset.z };
  for (const [Index, Axis] of ['x', 'y', 'z'].entries()) await Inputs.nth(Index).fill(String(Visual[Axis]));
  await Page.locator('.wide-field input').fill(String(BlockedTarget.wristPitch));
  Assert.equal(await Page.getByRole('button', { name: 'Send', exact: true }).isDisabled(), true);
  Assert.ok((await Page.locator('[role=status]').allTextContents()).some(Text => Text.includes('basket')));
  const HeadVisual = { x: Fixtures.HeadTarget.x - State.workspace_offset.x,
    y: Fixtures.HeadTarget.y + State.workspace_offset.y, z: Fixtures.HeadTarget.z + State.workspace_offset.z };
  for (const [Index, Axis] of ['x', 'y', 'z'].entries()) await Inputs.nth(Index).fill(String(HeadVisual[Axis]));
  Assert.equal(await Page.getByRole('button', { name: 'Send', exact: true }).isDisabled(), true);
  Assert.ok((await Page.locator('[role=status]').allTextContents()).some(Text => Text.includes('camera head')));
  Assert.equal(Commands.length, 0, 'A target in the camera head never reaches hardware');
  await Inputs.nth(1).fill('-31');
  Assert.equal(await Page.getByRole('button', { name: 'Send', exact: true }).isDisabled(), true);
  Assert.ok((await Page.locator('[role=status]').allTextContents()).some(Text => Text.includes('Y = -30 cm')));
  Assert.equal(Commands.length, 0, 'A target behind the rear limit never reaches hardware');
  await Page.getByRole('button', { name: 'Stop and hold', exact: true }).click();
  await Page.waitForTimeout(150);
  Assert.deepEqual(Commands, [{ type: 'stop_motion' }]);
  Assert.deepEqual(Errors, []);
  await Page.screenshot({ path: Path.join(Backend, 'TestArtifacts/BasketBlockedUi.png') });
  console.log('PASS: formerly blocked clear pose enables Send; basket draft snaps on release; real basket/head/rear collisions block Send; Stop and hold stays available; no motor movement');
} finally {
  clearInterval(Timer);
  await Browser.close(); await Server.Close();
}
