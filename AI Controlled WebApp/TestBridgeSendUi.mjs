import { chromium as Chromium } from "playwright";
import { CadReadyMs, LaunchOptions, StartUiServer, BackendPython } from "./UiTestLib.mjs";
import { mkdirSync as MakeDirectory, writeFileSync as WriteFile } from "node:fs";
import { execFileSync as Execute } from "node:child_process";
import { fileURLToPath as FilePath } from "node:url";
import Path from "node:path";
import Assert from "node:assert/strict";
const Root = Path.dirname(FilePath(import.meta.url)), Backend = Path.resolve(Root, '../back end');
const Python = BackendPython();
const State = JSON.parse(Execute(Python, ['-c', 'import json, bridge_server; print(json.dumps(bridge_server.BridgeController().state_snapshot()))'], { cwd: Backend, encoding: 'utf8' }));
State.arm_connected = true; State.torque = true; State.port = 'FAKE';
const Output = Path.join(Backend, 'TestArtifacts'); MakeDirectory(Output, { recursive: true });
const Server = await StartUiServer();
const Browser = await Chromium.launch(LaunchOptions({ args: ['--use-fake-device-for-media-stream', '--use-fake-ui-for-media-stream'] }));
const Page = await Browser.newPage({ viewport: { width: 1920, height: 1080 } });
const Errors = [], Commands = [], Timers = new Set();
let Ports = [];
Page.on('pageerror', Error => Errors.push(Error.message));
await Page.route('**/bridge-launch/**', Route => Route.fulfill({ json: { ok: true } }));
await Page.route('**/ollama/**', Route => Route.fulfill({ json: { models: [] } }));
await Page.route('**/depth-server/**', Route => Route.fulfill({ json: {} }));
await Page.route('**:8000/**', Route => {
  const Url = new URL(Route.request().url());
  if (Url.pathname === '/ports') return Route.fulfill({ json: Ports });
  if (Url.pathname === '/ik/preview') return Route.fulfill({ json: { ok: false } });
  if (Route.request().method() !== 'GET') Commands.push({ Path: Url.pathname, Payload: Route.request().postData() });
  return Route.fulfill({ json: { ok: true, calibrated: false, points: [] } });
});
let Socket;
await Page.routeWebSocket('**/ws', Connected => {
  Socket = Connected; Connected.send(JSON.stringify(State));
  const Timer = setInterval(() => Connected.send(JSON.stringify(State)), 100); Timers.add(Timer);
  Connected.onClose(() => { clearInterval(Timer); Timers.delete(Timer); });
  Connected.onMessage(Message => { const Command = JSON.parse(String(Message)); Commands.push(Command);
    if (Command.type === 'move_servo') { State.servos[Command.name].target_raw = Command.raw; }
    if (Command.type === 'disconnect') { State.arm_connected = false; State.torque = false; }
  });
});
await Page.route('**/src/components/CadCanvas.tsx*', async Route => {
  const Response = await Route.fetch(), Original = await Response.text();
  const Modified = Original.replace('const Scene = new Three.Scene();', 'const Scene = window.HudScene = new Three.Scene();')
    .replace('Renderer = new Three.WebGLRenderer(', 'Renderer = window.HudRenderer = new Three.WebGLRenderer(')
    .replace('const Controls = new OrbitControls(', 'const Controls = window.HudControls = new OrbitControls(')
    .replace('const Raycaster = new Three.Raycaster();', 'const Raycaster = window.HudRaycaster = new Three.Raycaster();');
  Assert.notEqual(Modified, Original); await Route.fulfill({ response: Response, body: Modified });
});

const UpdatedState = structuredClone(State);
State.collision_version = 'basket-v2'; delete State.speed_control_version; delete State.protection_version;
let RestartRequests = 0, AllowRestart = false;
await Page.route('**/bridge-launch/restart', async Route => {
  RestartRequests++;
  if (!AllowRestart) return Route.fulfill({ status: 409, json: { ok: false, error: 'The arm is still connected. Support it and click Disconnect before loading the bridge update.' } });
  await new Promise(Resolve => setTimeout(Resolve, 600));
  Object.assign(State, UpdatedState, { arm_connected: false, torque: false });
  return Route.fulfill({ json: { ok: true, status: 'updated-disconnected' } });
});
try {
  await Page.goto(Server.Url, { waitUntil: 'domcontentloaded' });
  await Page.locator('canvas[data-cad-ready=true][data-assembly-ready=true]').waitFor({ timeout: CadReadyMs });
  await Page.getByRole('button', { name: 'Set Current', exact: true }).click();
  const Send = Page.getByRole('button', { name: 'Send', exact: true });
  const Banner = Page.locator('.bridge-update-status');
  await Banner.waitFor();
  Assert.match(await Banner.innerText(), /basket avoidance/);
  Assert.equal(await Page.getByRole('button', { name: 'Load bridge update', exact: true }).isDisabled(), true);
  await Send.click();
  await Page.locator('.hud-notice').filter({ hasText: 'Bridge update required: basket avoidance' }).waitFor();
  Assert.equal(await Page.getByText('IK send needs the Python bridge', { exact: true }).count(), 0);
  Assert.deepEqual(Commands, [], 'Old bridge must never receive an unguarded IK command');
  await Page.getByTitle('Restart bridge', { exact: true }).click();
  await Page.locator('.hud-notice').filter({ hasText: 'disconnecting releases torque' }).waitFor();
  Assert.equal(RestartRequests, 0, 'Connected UI refuses restart before issuing a request');
  await Page.getByRole('button', { name: 'Disconnect', exact: true }).click();
  await Page.getByRole('button', { name: 'Connect', exact: true }).waitFor();
  Assert.deepEqual(Commands.map(Command => Command.type), ['disconnect']); Commands.length = 0;
  const Update = Page.getByRole('button', { name: 'Load bridge update', exact: true });
  await Update.click();
  await Page.locator('.hud-notice').filter({ hasText: 'The arm is still connected.' }).waitFor();
  Assert.equal(RestartRequests, 1, 'Server refusal is shown honestly');
  Assert.equal(await Page.locator('.hud-notice').filter({ hasText: 'Updated bridge loaded.' }).count(), 0);
  Assert.equal(await Banner.isVisible(), true);
  AllowRestart = true; Ports = ['FAKE'];
  await Update.click();
  Assert.equal(await Page.getByRole('button', { name: 'Connect', exact: true }).isDisabled(), true, 'Connection is blocked while Python is being replaced');
  await Page.locator('.hud-notice').filter({ hasText: 'Updated bridge loaded.' }).waitFor();
  await Banner.waitFor({ state: 'hidden' });
  Assert.equal(RestartRequests, 2);
  await Page.waitForTimeout(700);
  Assert.deepEqual(Commands, [], 'Reload never reconnects or resumes motion automatically');
  await Send.click();
  await Page.locator('.hud-notice').filter({ hasText: 'Pick a port and click Connect Arm first' }).waitFor();
  State.arm_connected = true; State.torque = false;
  Socket.send(JSON.stringify(State)); await Page.getByRole('button', { name: 'Hold', exact: true }).waitFor();
  await Send.click();
  await Page.locator('.hud-notice').filter({ hasText: 'Hold/torque must be on before Send' }).waitFor();
  State.torque = true; Socket.send(JSON.stringify(State)); await Page.getByRole('button', { name: 'Coast', exact: true }).waitFor();
  await Send.click();
  await Page.waitForFunction(() => document.querySelector('.hud-notices').textContent.includes('IK command sent to bridge'));
  Assert.equal(Commands.length, 1); Assert.equal(Commands[0].type, 'send_ik');
  for (const Key of ['x', 'y', 'z', 'wrist_pitch']) Assert.ok(Number.isFinite(Commands[0].target[Key]));
  Commands.length = 0;
  Socket.close();
  await Page.locator('.connection-pill.offline').waitFor();
  await Send.click();
  await Page.locator('.hud-notice').filter({ hasText: 'Python bridge connection is offline' }).waitFor();
  Assert.deepEqual(Commands, []);
  // Stop and hold while the socket is down goes over POST /motion instead (2026-10-08 safety review: it only set a status).
  await Page.getByRole('button', { name: 'Stop and hold', exact: true }).click();
  await Page.waitForFunction(() => document.querySelector('.hud-notices').textContent.includes('sent over HTTP'));
  Assert.deepEqual(Commands, [{ Path: '/motion', Payload: JSON.stringify({ type: 'stop_motion' }) }]);
  Assert.deepEqual(Errors, []);
  await Page.screenshot({ path: Path.join(Output, 'BridgeSendDiagnostics.png') });
  console.log('PASS: reproduced old-bridge Send rejection; specific persistent diagnostic; connected restart refusal; real restart endpoint invoked; server refusal surfaced; update handshake; no auto-reconnect or replay; disconnected/coast/offline reasons; one explicit compatible send with mocked hardware; Stop and hold over HTTP while the socket is down.');
} finally { Timers.forEach(Timer => clearInterval(Timer)); await Browser.close(); await Server.Close(); }
