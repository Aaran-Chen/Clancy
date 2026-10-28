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

await Page.route('**/src/App.tsx*', async Route => {
  const Response = await Route.fetch(), Original = await Response.text();
  const Modified = Original.replace('const sendCommand = ', `window.SetHudAiTarget = (Invalid) => {
    const Parsed = parseAiResponse(JSON.stringify({ task_status: 'continue', motion_proposal: {
      target_cm: Invalid ? { x: 500, y: 500, z: 500 } : { x: 3, y: 30, z: 8 }, wrist_pitch_degrees: -40
    } }), CurrentTargetRef.current, servos).motion;
    setAiDisplayedProposal(Parsed); setAiProposal(Invalid ? null : Parsed);
    return { Target: visualPointFromDisplayPoint({ ...mirrorDisplayPoint(Parsed.target), wristPitch: Parsed.wristPitchTarget }), Manual: CurrentTargetRef.current };
  }; window.ReadHudManualTarget = () => CurrentTargetRef.current;
  const sendCommand = `);
  Assert.notEqual(Modified, Original); await Route.fulfill({ response: Response, body: Modified });
});

try {
  await Page.goto(Server.Url, { waitUntil: 'domcontentloaded' });
  await Page.locator('canvas[data-cad-ready=true][data-assembly-ready=true]').waitFor({ timeout: CadReadyMs });
  await Page.waitForFunction(() => document.querySelector('video')?.videoWidth > 0);
  const CameraWindow = Page.getByRole('region', { name: 'Live camera window', exact: true });
  const MoveHandle = Page.getByRole('button', { name: 'Move camera window', exact: true });
  const Cube = Page.getByLabel('View cube', { exact: true });
  const Cad = Page.locator('.cad-shell');
  const AssertCube = async () => {
    const A = await Cube.boundingBox(), B = await Cad.boundingBox();
    Assert.ok(Math.abs(A.x - B.x - 12) < 1 && Math.abs(A.y - B.y - 12) < 1, 'Cube is at CAD top left');
  };
  await AssertCube();
  Assert.equal(await Page.locator('.cad-heading, .cad-side, [data-section="Camera"]').count(), 0);
  Assert.equal(await Page.locator('.ai-column').isVisible(), false);
  Assert.equal(await Page.getByRole('separator').count(), 2);
  const InitialScene = await Cad.boundingBox();
  const Directions = { front: [0, 1, 0], back: [0, -1, 0], right: [1, 0, 0], left: [-1, 0, 0], top: [0, 0, 1], bottom: [0, 0, -1] };
  for (const [View, Direction] of Object.entries(Directions)) {
    await Page.evaluate(Direction => {
      const Controls = window.HudControls; Controls.enableDamping = false;
      Controls.object.position.copy(Controls.target).add(Controls.object.position.clone().set(...Direction).multiplyScalar(210).addScalar(25)); Controls.update();
    }, Direction);
    await Page.waitForTimeout(150);
    const Face = Page.getByRole('button', { name: `View ${View}`, exact: true });
    const FaceBounds = await Face.boundingBox();
    await Face.hover();
    const Before = await Face.evaluate(Element => getComputedStyle(Element).transform);
    await Page.mouse.down();
    Assert.equal(await Face.evaluate(Element => getComputedStyle(Element).transform), Before, 'Press must retain cube face geometry');
    await Page.waitForTimeout(140); await Page.mouse.up();
    await Page.waitForTimeout(250);
    const Actual = await Page.evaluate(() => window.HudControls.object.position.clone().sub(window.HudControls.target).normalize().toArray());
    Assert.ok(Actual.reduce((Sum, Value, Index) => Sum + Value * Direction[Index], 0) > .999999, `View ${View} clicks to the correct side: ${Actual}`);
  }
  await Page.getByRole('button', { name: 'Fit model in view', exact: true }).click();
  await Page.evaluate(() => { window.FloatingVideo = document.querySelector('video'); window.FloatingStream = window.FloatingVideo.srcObject; window.FloatingCanvas = window.HudRenderer.domElement; });
  const Identity = async () => Assert.equal(await Page.evaluate(() => document.querySelector('video') === window.FloatingVideo && document.querySelector('video').srcObject === window.FloatingStream && window.HudRenderer.domElement === window.FloatingCanvas), true, 'Moving the camera must preserve video, stream and CAD');
  const DragBy = async (Handle, X, Y, Cancel = false) => {
    const B = await Handle.boundingBox(); await Page.mouse.move(B.x + B.width / 2, B.y + B.height / 2); await Page.mouse.down();
    await Page.mouse.move(B.x + B.width / 2 + X, B.y + B.height / 2 + Y, { steps: 12 });
    if (Cancel) await Page.keyboard.press('Escape'); await Page.mouse.up();
  };
  Assert.equal(await Page.getByRole('button', { name: 'View isometric', exact: true }).count(), 0);
  // innerText is what the theme renders, and the HUD sets its labels in caps.
  Assert.equal((await Page.locator('.camera-window-title').textContent()).trim(), 'Live view');
  Assert.equal(await Page.locator('.camera-window-actions button').count(), 2);
  Assert.equal(await Page.locator('.call-controls, .camera-telemetry').count(), 0);
  await Page.evaluate(() => document.fonts.ready);
  Assert.ok(await Page.evaluate(() => document.fonts.check('400 16px Play') && document.fonts.check('700 16px Play')));
  Assert.ok((await Page.locator('.dashboard-app').evaluate(Element => getComputedStyle(Element).fontFamily)).startsWith('Play'));
  const CubeStart = await Cube.boundingBox();
  const CameraBeforeCubeDrag = await Page.evaluate(() => window.HudControls.object.position.toArray());
  // Drag the visible face, not just empty cube padding.
  await DragBy(Page.getByRole('button', { name: 'View front', exact: true }), 180, 140);
  const CubeMoved = await Cube.boundingBox();
  Assert.ok(Math.abs(CubeMoved.x - CubeStart.x - 180) < 1 && Math.abs(CubeMoved.y - CubeStart.y - 140) < 1);
  Assert.deepEqual(await Page.evaluate(() => window.HudControls.object.position.toArray()), CameraBeforeCubeDrag, 'Dragging a face must not change the CAD camera');
  await DragBy(Cube, 55, 35, true);
  Assert.deepEqual(await Cube.boundingBox(), CubeMoved, 'Escape restores cube position');
  await Page.reload({ waitUntil: 'domcontentloaded' });
  await Page.locator('canvas[data-cad-ready=true][data-assembly-ready=true]').waitFor({ timeout: CadReadyMs });
  await Page.waitForFunction(() => document.querySelector('video')?.videoWidth > 0);
  Assert.deepEqual(await Cube.boundingBox(), CubeMoved, 'Cube position survives reload');
  await Cube.focus(); await Page.keyboard.press('Home'); await AssertCube();
  await Page.evaluate(() => { window.FloatingVideo = document.querySelector('video'); window.FloatingStream = window.FloatingVideo.srcObject; window.FloatingCanvas = window.HudRenderer.domElement; });
  let Before = await CameraWindow.boundingBox();
  await DragBy(MoveHandle, 390, -280);
  let After = await CameraWindow.boundingBox();
  Assert.ok(Math.abs(After.x - Before.x - 390) < 1 && Math.abs(After.y - Before.y + 280) < 1);
  Assert.deepEqual(await Cad.boundingBox(), InitialScene, 'Floating movement never consumes pane space');
  await Identity();
  Before = After; await DragBy(MoveHandle, 90, 40, true); After = await CameraWindow.boundingBox();
  Assert.deepEqual(After, Before, 'Escape cancels a window move');
  await DragBy(Page.getByRole('button', { name: 'Resize camera window', exact: true }), 110, 70);
  After = await CameraWindow.boundingBox(); Assert.ok(Math.abs(After.width - Before.width - 110) < 1 && Math.abs(After.height - Before.height - 70) < 1);
  await Identity();
  Before = After; await MoveHandle.press('ArrowRight'); After = await CameraWindow.boundingBox(); Assert.equal(After.x, Before.x + 10);
  await Page.getByRole('button', { name: 'Minimize Live Feed', exact: true }).click();
  Assert.equal((await CameraWindow.boundingBox()).height, 48);
  Assert.equal((await CameraWindow.boundingBox()).width, 48); await Identity();
  const Tile = Page.getByRole('button', { name: 'Restore Live Feed', exact: true });
  const TileBefore = await CameraWindow.boundingBox();
  await DragBy(Tile, 80, 20);
  const TileAfter = await CameraWindow.boundingBox();
  Assert.equal(TileAfter.width, 48); Assert.equal(TileAfter.height, 48);
  Assert.equal(TileAfter.x, TileBefore.x + 80); Assert.equal(TileAfter.y, TileBefore.y + 20);
  await Page.waitForTimeout(3100);
  Assert.equal(await Page.locator('video').getAttribute('data-capture-ready'), 'true', 'Minimizing preserves usable calibration frames');
  await Page.getByRole('button', { name: 'AI control mode', exact: true }).click(); await Identity();
  Assert.equal(await Page.getByRole('separator').count(), 1);
  Assert.equal(await Page.locator('[data-section="Status"]').count(), 0);
  Assert.equal(await Page.locator('[data-section="Target"]').isVisible(), false);
  const Left = await Page.locator('.control-column').boundingBox(), Scene = await Cad.boundingBox();
  Assert.ok(Left.x + Left.width <= Scene.x && Scene.x + Scene.width > 1900);
  Assert.equal(await Page.locator('.control-column [data-section="Motion"]').isVisible(), true);
  Assert.equal(await Page.getByRole('button', { name: 'Send', exact: true }).count(), 0, 'AI approval replaces the manual Send control');
  Assert.equal(await Page.evaluate(() => window.HudScene.getObjectByName('TargetApproachRay').parent.visible), false, 'No stale manual target in AI mode');
  const Proposal = await Page.evaluate(() => window.SetHudAiTarget(false));
  await Page.waitForFunction(() => window.HudScene.getObjectByName('TargetApproachRay').parent.visible);
  const TargetPosition = () => Page.evaluate(() => window.HudScene.getObjectByName('TargetApproachRay').parent.position.toArray());
  Assert.deepEqual(await TargetPosition(), [Proposal.Target.x, Proposal.Target.y, Proposal.Target.z], 'CAD displays the AI destination');
  const Point = await Page.evaluate(() => {
    const Target = window.HudScene.getObjectByName('TargetApproachRay').parent.position.clone().project(window.HudControls.object);
    const Bounds = window.HudRenderer.domElement.getBoundingClientRect();
    return { X: Bounds.x + (Target.x + 1) * Bounds.width / 2, Y: Bounds.y + (1 - Target.y) * Bounds.height / 2 };
  });
  await Page.mouse.move(Point.X, Point.Y); await Page.mouse.wheel(0, 100); await Page.waitForTimeout(300);
  Assert.deepEqual(await TargetPosition(), [Proposal.Target.x, Proposal.Target.y, Proposal.Target.z]);
  Assert.deepEqual(await Page.evaluate(() => window.ReadHudManualTarget()), Proposal.Manual, 'Scrolling the AI target cannot edit the hidden manual destination');
  await Page.evaluate(() => window.SetHudAiTarget(true));
  await Page.waitForFunction(() => document.querySelector('canvas[data-cad-ready=true]').dataset.targetState === 'invalid');
  Assert.equal(await Page.getByRole('button', { name: 'Approve', exact: true }).count(), 0, 'Rejected AI targets remain unapproved');
  await Page.evaluate(() => window.SetHudAiTarget(false));
  await Page.screenshot({ path: Path.join(Output, 'PlayAiHud.png') });

  await Page.getByRole('button', { name: 'Restore Live Feed', exact: true }).click();
  await Page.getByRole('button', { name: 'Manual control mode', exact: true }).click(); await Identity();
  await Page.getByRole('button', { name: 'Full screen live view', exact: true }).click();
  Assert.ok((await CameraWindow.boundingBox()).width > 1800);
  const Emergency = await Page.getByRole('button', { name: 'Emergency stop', exact: true }).boundingBox();
  Assert.equal(await Page.evaluate(B => !!document.elementFromPoint(B.x + B.width / 2, B.y + B.height / 2)?.closest('.topbar-emergency'), Emergency), true);
  await Page.keyboard.press('Escape'); await Identity();
  await Page.getByRole('button', { name: 'Settings', exact: true }).click();
  const WindowOnTop = await CameraWindow.boundingBox();
  Assert.equal(await Page.evaluate(B => !!document.elementFromPoint(B.x + B.width / 2, B.y + 20)?.closest('.camera-window'), WindowOnTop), true);
  await Page.keyboard.press('Escape');
  await Page.getByRole('button', { name: 'Full screen', exact: true }).click();
  await Page.waitForFunction(() => !!document.fullscreenElement?.querySelector('.camera-window'));
  await Identity(); await AssertCube();
  await Page.getByRole('button', { name: 'Full screen', exact: true }).click();
  await Page.waitForFunction(() => !document.fullscreenElement); await Identity();
  await Page.getByRole('button', { name: 'Settings', exact: true }).click();
  const PopoutButton = Page.getByRole('button', { name: 'Float above other apps', exact: true });
  if (await PopoutButton.count()) {
    // Use the retained OS pop-out through Settings, leaving the camera header clean.
    await PopoutButton.click();
    await Page.waitForFunction(() => !!document.pictureInPictureElement || document.querySelector('.hud-notices')?.textContent.includes('Camera pop-out unavailable'));
    if (await Page.evaluate(() => !!document.pictureInPictureElement)) {
      await PopoutButton.click(); await Page.waitForFunction(() => !document.pictureInPictureElement);
    }
    await Identity();
  }
  await Page.getByRole('button', { name: 'Close settings', exact: true }).click();
  const Saved = await CameraWindow.boundingBox();
  await Page.reload({ waitUntil: 'domcontentloaded' }); await CameraWindow.waitFor();
  Assert.deepEqual(await CameraWindow.boundingBox(), Saved, 'Window position and size persist after reload');
  await Page.setViewportSize({ width: 1000, height: 720 });
  await Page.waitForTimeout(250);
  const Narrow = await CameraWindow.boundingBox();
  Assert.ok(Narrow.x >= 12 && Narrow.y >= 68 && Narrow.x + Narrow.width <= 988 && Narrow.y + Narrow.height <= 708, 'Window remains reachable after resize');
  await AssertCube();
  await Page.setViewportSize({ width: 1920, height: 1080 });
  await Page.waitForTimeout(200);
  await Page.screenshot({ path: Path.join(Output, 'FloatingCameraHud.png') });
  Assert.deepEqual(Commands, [], 'Display controls must never send hardware commands');
  Assert.deepEqual(Errors, []);
  WriteFile(Path.join(Output, 'FloatingCameraResults.json'), JSON.stringify({ CubeViews: Object.keys(Directions), BrowserErrors: Errors, HardwareCommands: Commands }, null, 2));
  console.log('PASS: all six cube faces; top-left placement; floating drag/resize/cancel; persistence; minimized/expanded; mode and fullscreen stream identity; responsive bounds; no motor commands.');
} finally { Timers.forEach(Timer => clearInterval(Timer)); await Browser.close(); await Server.Close(); }
