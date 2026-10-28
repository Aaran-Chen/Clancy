import { chromium as Chromium } from "playwright";
import { CadReadyMs, LaunchOptions, StartUiServer, BackendPython } from "./UiTestLib.mjs";
import { readFileSync as ReadFile, readdirSync as ReadDirectory, mkdirSync as MakeDirectory, rmSync as Remove, writeFileSync as WriteFile } from "node:fs";
import { createHash as CreateHash } from "node:crypto";
import { execFileSync as Execute } from "node:child_process";
import { fileURLToPath as FilePath } from "node:url";
import Path from "node:path";
import Assert from "node:assert/strict";
const Root = Path.dirname(FilePath(import.meta.url)), Backend = Path.resolve(Root, '../back end');
const Python = BackendPython();
const State = JSON.parse(Execute(Python, ['-c', 'import json, bridge_server; print(json.dumps(bridge_server.BridgeController().state_snapshot()))'], { cwd: Backend, encoding: 'utf8' }));
State.arm_connected = true; State.torque = true; State.port = 'FAKE';
const Output = Path.join(Backend, 'TestArtifacts'); MakeDirectory(Output, { recursive: true });
// back end/TestAutomaticWristBridge.py checks the pose this suite resolves, from the fixture it writes when it passes,
// stamped with a hash of the app's src/ (the same as that script's SourceHash): a failed run leaves none, and a fixture
// from older app code is refused there (audit 2 L7).
const FixtureFile = Path.join(Output, 'AutomaticWristUiResults.json');
Remove(FixtureFile, { force: true });
function SourceHash(Folder) {
  const Files = ReadDirectory(Folder, { recursive: true, withFileTypes: true }).filter(Entry => Entry.isFile())
    .map(Entry => Path.relative(Folder, Path.join(Entry.parentPath ?? Entry.path, Entry.name)).split(Path.sep).join('/')).sort();
  const Digest = CreateHash('sha256');
  for (const Name of Files) Digest.update(Buffer.concat([Buffer.from(`${Name}\n`), ReadFile(Path.join(Folder, Name)), Buffer.from('\n')]));
  return Digest.digest('hex');
}
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

// A release whose search misses App.tsx's half-second budget (TargetSnapLimitMs) restores the previous target: right for
// the UI, and TestTargetSnapUi's subject, but on a loaded PC it made this suite read the old position ("0.034 vs -0.054",
// audit 2's test table). This suite checks what the search answers, so here it gets as long as it needs.
const ReleaseBudget = 'const TargetSnapLimitMs = 500;';
await Page.route('**/src/App.tsx*', async Route => {
  const Response = await Route.fetch(), Original = await Response.text();
  Assert.ok(Original.includes(ReleaseBudget), 'App.tsx still has the half-second release budget this suite lifts');
  const Modified = Original.replace(ReleaseBudget, 'const TargetSnapLimitMs = 60000;').replace('const sendCommand = ', `window.WristStage = (Candidate) => { BeginTargetEdit('Position'); PreviewManualTarget(Candidate); };
  window.WristFinish = FinishTargetEdit; window.WristRead = () => CurrentTargetRef.current; window.WristBusy = () => !!TargetEditRef.current || !!TargetWorkerRef.current; window.WristContext = () => TargetContextRef.current;
  const sendCommand = `);
  Assert.ok(Modified.includes('window.WristStage')); await Route.fulfill({ response: Response, body: Modified });
});
try {
  await Page.goto(Server.Url, { waitUntil: 'domcontentloaded' });
  const Canvas = Page.locator('canvas[data-cad-ready=true][data-assembly-ready=true]'); await Canvas.waitFor({ timeout: CadReadyMs });
  await Page.getByRole('button', { name: 'Minimize Live Feed', exact: true }).click();
  const Fixture = await Page.evaluate(async () => {
    const Api = await import('/src/lib/robotModel.ts');
    const Cad = await import('/src/lib/CadKinematics.ts');
    const Context = window.WristContext(); const Initial = Api.currentAngles(Context.Servos);
    for (const Delta of [-25, 0, 20]) {
      const Angles = { ...Initial, wrist_flex: Initial.wrist_flex + Delta };
      const Exact = { ...Cad.CadForward(Angles).Tool, wristPitch: Cad.CadPitch(Angles) };
      const Request = { ...Exact, wristPitch: Exact.wristPitch + 120 };
      if (Api.solveIkPreview(Exact, Context.ElbowMode, Context.Servos, Context.Limits).reachable && !Api.solveIkPreview(Request, Context.ElbowMode, Context.Servos, Context.Limits).reachable) return { Request, Exact, Home: Api.HOME_MODEL_TARGET_CM };
    }
    throw Error('No recovery fixture found');
  });
  const WaitValid = () => Page.waitForFunction(() => document.querySelector('canvas[data-cad-ready=true]').dataset.targetState === 'valid' && !window.WristBusy());
  const Read = () => Page.evaluate(() => window.WristRead());
  const SamePosition = (A, B) => { for (const Key of ['x', 'y', 'z']) Assert.equal(A[Key], B[Key]); };
  const Validate = async () => Assert.equal(await Page.evaluate(async () => {
    const Api = await import('/src/lib/robotModel.ts'); const Context = window.WristContext(); return Api.solveIkPreview(window.WristRead(), Context.ElbowMode, Context.Servos, Context.Limits).reachable;
  }), true);
  await Page.evaluate(Request => { window.WristStage(Request); window.WristFinish(); }, Fixture.Request);
  await WaitValid();
  const Resolved = await Read(); SamePosition(Resolved, Fixture.Request); Assert.notEqual(Resolved.wristPitch, Fixture.Request.wristPitch); await Validate();
  await Page.getByRole('button', { name: 'Send', exact: true }).click();
  await Page.waitForTimeout(100);
  Assert.equal(Commands.length, 1); Assert.equal(Commands[0].type, 'send_ik');
  SamePosition(Commands[0].target, Fixture.Request); Assert.equal(Commands[0].target.wrist_pitch, Resolved.wristPitch); Commands.length = 0;
  // A quiet drag also resolves in place before release via the guide worker.
  await Page.evaluate(Request => window.WristStage(Request), Fixture.Request);
  await Page.waitForFunction(Pitch => window.WristRead().wristPitch !== Pitch, Fixture.Request.wristPitch);
  SamePosition(await Read(), Fixture.Request); await Page.evaluate(() => window.WristFinish()); await WaitValid();
  // A later edit must not inherit the previous worker's position or wrist pitch.
  await Page.evaluate(Request => window.WristStage(Request), Fixture.Request);
  await Page.waitForTimeout(90);
  await Page.evaluate(Home => { window.WristStage(Home); window.WristFinish(); }, Fixture.Home);
  await WaitValid(); await Page.waitForTimeout(1600); Assert.deepEqual(await Read(), Fixture.Home);
  const SameCamera = (A, B) => { for (const Key of ['Position', 'Target']) Assert.ok(Math.hypot(...A[Key].map((Value, Index) => Value - B[Key][Index])) < 1e-8, 'Camera position remains unchanged'); };
  const Controls = () => Page.evaluate(() => ({ Position: window.HudControls.object.position.toArray(), Target: window.HudControls.target.toArray() }));
  const Project = () => Page.evaluate(() => {
    const Point = window.HudScene.getObjectByName('TargetApproachRay').parent.position.clone().project(window.HudControls.object);
    const Bounds = window.HudRenderer.domElement.getBoundingClientRect();
    return { x: Bounds.x + (Point.x * .5 + .5) * Bounds.width, y: Bounds.y + (-Point.y * .5 + .5) * Bounds.height };
  });
  await Page.getByRole('button', { name: 'Fit model in view', exact: true }).click();
  await Page.waitForFunction(() => document.querySelector('canvas[data-cad-ready=true]').dataset.pitchGuideState === 'ready');
  await Page.evaluate(() => { window.WristGuideGeometry = window.HudScene.getObjectByName('AcceptableWristAngles').geometry; });
  const BeforeWheel = await Read(), CameraBefore = await Controls(), Dot = await Project();
  await Page.mouse.move(Dot.x, Dot.y); await Page.mouse.wheel(0, 40);
  await Page.waitForFunction(Pitch => window.WristRead().wristPitch !== Pitch, BeforeWheel.wristPitch);
  await WaitValid(); await Validate(); SamePosition(await Read(), BeforeWheel);
  SameCamera(await Controls(), CameraBefore);
  Assert.equal(await Page.evaluate(() => window.HudScene.getObjectByName('AcceptableWristAngles').visible && window.HudScene.getObjectByName('AcceptableWristAngles').geometry === window.WristGuideGeometry), true);
  const BeforeLimit = await Read();
  await Page.mouse.move(Dot.x, Dot.y); await Page.mouse.wheel(0, 2400); await Page.waitForTimeout(220); await WaitValid();
  SamePosition(await Read(), BeforeLimit); await Validate(); SameCamera(await Controls(), CameraBefore);
  const BeforeCancel = await Read();
  await Canvas.evaluate((Element, Point) => Element.dispatchEvent(new WheelEvent('wheel', { bubbles: true, cancelable: true, clientX: Point.x, clientY: Point.y, deltaY: -200 })), Dot);
  await Page.keyboard.press('Escape'); await WaitValid(); Assert.deepEqual(await Read(), BeforeCancel);
  const Bounds = await Canvas.boundingBox(), BeforeZoom = await Controls(), PoseBeforeZoom = await Read();
  await Page.mouse.move(Bounds.x + Bounds.width * .8, Bounds.y + Bounds.height * .65); await Page.mouse.wheel(0, 100);
  await Page.waitForTimeout(160); Assert.notDeepEqual(await Controls(), BeforeZoom); Assert.deepEqual(await Read(), PoseBeforeZoom);
  // A clear point inside the tray must stay in place and reach Send. Only the
  // bridge decides how to get there; a blocked direct sweep cannot clamp XYZ.
  const Interior = { x: -1.9532211879722914, y: 14.308657609320962, z: -5.741477864378066, wristPitch: -108.0 };
  await Page.evaluate(Point => { window.WristStage(Point); window.WristFinish(); }, Interior);
  await WaitValid(); Assert.deepEqual(await Read(), Interior); await Validate();
  await Page.getByRole('button', { name: 'Send', exact: true }).click();
  await Page.waitForTimeout(100);
  Assert.equal(Commands.length, 1); Assert.equal(Commands[0].type, 'send_ik');
  SamePosition(Commands[0].target, Interior); Assert.equal(Commands[0].target.wrist_pitch, Interior.wristPitch);
  Commands.length = 0;
  Assert.deepEqual(Commands, []); Assert.deepEqual(Errors, []);
  await Page.screenshot({ path: Path.join(Output, 'AutomaticWristHud.png') });
  WriteFile(FixtureFile, JSON.stringify({ Passed: true, WrittenAt: new Date().toISOString(), SourceHash: SourceHash(Path.join(Root, 'src')),
    Requested: Fixture.Request, Resolved, Errors, Commands }, null, 2));
  console.log('PASS: exact XYZ recovery before and after release; adjusted pitch sent explicitly; stale worker rejection; target-dot wheel controls pitch without zoom; fixed-XYZ angle boundary; Escape; persistent green guide; empty-space zoom; interior basket target preserved and sent; mocked hardware only.');
} finally { Timers.forEach(Timer => clearInterval(Timer)); await Browser.close(); await Server.Close(); }
