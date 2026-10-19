import { chromium as Chromium } from "playwright";
import { LaunchOptions, RecordPage, StartUiServer, WaitForCad } from "./UiTestLib.mjs";
import { readFileSync as ReadFile, mkdirSync as MakeDirectory, writeFileSync as WriteFile } from "node:fs";
import { execFileSync as Execute } from "node:child_process";
import { fileURLToPath as FilePath } from "node:url";
import Path from "node:path";
import Assert from "node:assert/strict";
const Root = Path.dirname(FilePath(import.meta.url)), Backend = Path.resolve(Root, '../back end');
const Python = ReadFile(Path.join(Root, '../.backend-python.txt'), 'utf8').trim();
const State = JSON.parse(Execute(Python, ['-c', 'import json, bridge_server; print(json.dumps(bridge_server.BridgeController().state_snapshot()))'], { cwd: Backend, encoding: 'utf8' }));
State.arm_connected = true; State.torque = true; State.port = 'FAKE';
const Output = Path.join(Backend, 'TestArtifacts'); MakeDirectory(Output, { recursive: true });
const Server = await StartUiServer();
const Browser = await Chromium.launch(LaunchOptions({ args: ['--use-fake-device-for-media-stream', '--use-fake-ui-for-media-stream'] }));
const Page = await Browser.newPage({ viewport: { width: 1920, height: 1080 } });
const Errors = [], Commands = [], Timers = new Set();
let Ports = [];
Page.on('pageerror', Error => Errors.push(Error.message));
const PageLines = RecordPage(Page);  // what the page said, printed if the CAD view never comes up (WaitForCad)
const CadReady = () => WaitForCad(Page, { Lines: PageLines, Name: 'TestRobotHud' });
// The one-time local display preference (App.tsx reads public/LocalDisplayPreferences.json): the suite supplies one that
// hides the bottom text, so "local preference versus defaults" below checks the app, not this PC. The real file is
// git-ignored and machine-local: a worktree or a fresh clone has none, and the suite then waited for .cad-footer to hide.
await Page.route('**/LocalDisplayPreferences.json', Route => Route.fulfill({ json: { Revision: 'TestRobotHud.HideBottomText.1', ShowBottomText: false } }));
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
try {
  await Page.goto(Server.Url, { waitUntil: 'domcontentloaded' });
  const Canvas = await CadReady();
  await Page.getByRole('button', { name: 'Set Current', exact: true }).click();
  await Page.waitForFunction(() => document.querySelector('video').videoWidth > 0);
  await Page.waitForFunction(() => Number(document.querySelector('video').dataset.fps) > 0);
  Assert.equal(await Page.locator('.status-strip').count(), 0);
  const Notices = await Page.locator('.hud-notice p').allTextContents();
  Assert.equal(new Set(Notices).size, Notices.length, 'Repeated system updates are deduplicated');
  await Page.locator('.cad-footer').waitFor({ state: 'hidden' });
  Assert.equal(await Canvas.getAttribute('data-render-quality'), 'Ultra');
  Assert.ok(await Page.evaluate(() => window.HudRenderer.getPixelRatio() >= 2));
  Assert.ok((await Page.locator('.app-shell').evaluate(Element => getComputedStyle(Element).fontFamily)).includes('Play'));
  await Page.evaluate(() => { window.InitialHudCanvas = window.HudRenderer.domElement; window.InitialHudVideo = document.querySelector('video'); window.InitialHudStream = window.InitialHudVideo.srcObject; });
  const InactiveHeight = (await Page.locator('[data-joint="elbow_flex"]').boundingBox()).height;
  for (const [Name, Label] of [['shoulder_pan', 'Shoulder Pan'], ['shoulder_lift', 'Shoulder Lift'], ['elbow_flex', 'Elbow Flex'], ['wrist_flex', 'Wrist Flex'], ['wrist_roll', 'Wrist Roll'], ['gripper', 'Gripper']]) {
    await Page.getByRole('button', { name: Label.toLowerCase(), exact: true }).click();
    await Page.waitForFunction(Name => document.querySelector('canvas[data-cad-ready=true]').dataset.selectedJoint === Name, Name);
    Assert.equal(await Page.getByRole('slider', { name: `${Label.toLowerCase()} precision dial`, exact: true }).isVisible(), true);
    Assert.equal(await Page.evaluate(() => window.HudScene.getObjectByName('SelectedJointGuide').visible), true);
    // Lit, and lit in the mode's own colour: the selection light follows the
    // theme, so pin that it matches the live joint handles rather than a hex.
    const Lit = await Page.evaluate(Name => {
      const Handle = window.HudScene.getObjectByName(`LiveJointMarker_${Name}`);
      const Tone = Handle.children[0].material.color.getHexString();
      const Meshes = window.HudScene.getObjectByName('ControlledArm').children.filter(Mesh => Mesh.userData.Joint === Name);
      return { Tone, Count: Meshes.filter(Mesh => Mesh.material.emissive.getHexString() === Tone).length,
               Dark: Meshes.filter(Mesh => Mesh.material.emissive.getHexString() === '000000').length };
    }, Name);
    Assert.ok(Lit.Count > 0 && Lit.Dark === 0, `Selected ${Name} links light up in ${Lit.Tone}`);
  }
  Assert.deepEqual(Commands, [], 'Selection must never move hardware');
  await Page.getByRole('button', { name: 'Clear joint selection', exact: true }).click();
  const Pick = await Page.evaluate(() => {
    const Scene = window.HudScene, Controls = window.HudControls, Ray = window.HudRaycaster;
    const Arm = Scene.getObjectByName('ControlledArm'), Body = Scene.getObjectByName('XLeRobotBody'), Other = Scene.getObjectByName('InactiveArm');
    const Bounds = window.HudRenderer.domElement.getBoundingClientRect();
    const Target = Scene.getObjectByName('TargetApproachRay').parent.position.clone().project(Controls.object);
    for (const Mesh of Arm.children) {
      Mesh.geometry.computeBoundingSphere();
      const Point = Mesh.geometry.boundingSphere.center.clone().applyMatrix4(Mesh.matrixWorld);
      Ray.ray.origin.copy(Controls.object.position); Ray.ray.direction.copy(Point).sub(Controls.object.position).normalize();
      const Hit = Ray.intersectObjects([...Arm.children, ...Body.children, ...Other.children], false)[0];
      const Screen = Point.project(Controls.object);
      if (!Hit || !Arm.children.includes(Hit.object) || Math.abs(Screen.x) >= 1 || Math.abs(Screen.y) >= 1 || Math.hypot((Screen.x - Target.x) * Bounds.width / 2, (Screen.y - Target.y) * Bounds.height / 2) < 28) continue;
      return { X: Bounds.x + (Screen.x + 1) * Bounds.width / 2, Y: Bounds.y + (1 - Screen.y) * Bounds.height / 2, Joint: Hit.object.userData.Joint };
    }
    return null;
  });
  Assert.ok(Pick, 'At least one active-arm mesh must be visible and selectable');
  await Page.mouse.click(Pick.X, Pick.Y);
  await Page.waitForFunction(Name => document.querySelector('canvas[data-cad-ready=true]').dataset.selectedJoint === Name, Pick.Joint);
  Assert.deepEqual(Commands, [], 'Picking the actual 3D mesh only selects it');
  await Page.getByRole('button', { name: 'gripper', exact: true }).click();
  const Raw = State.servos.gripper.target_raw;
  await Page.getByRole('slider', { name: 'gripper precision dial', exact: true }).press('ArrowRight');
  await Page.waitForFunction(Raw => document.querySelector('[data-joint="gripper"] .joint-fine-step b').textContent !== String(Raw), Raw);
  Assert.equal(Commands.length, 1); Assert.equal(Commands[0].type, 'move_servo'); Assert.equal(Commands[0].name, 'gripper'); Assert.equal(Commands[0].raw, Raw + 1);
  Commands.length = 0;
  await Page.getByRole('button', { name: 'elbow flex', exact: true }).click();
  const ExpandedHeight = (await Page.locator('[data-joint="elbow_flex"]').boundingBox()).height;
  Assert.ok(ExpandedHeight > InactiveHeight * 2);
  await Page.waitForTimeout(300);
  await Page.screenshot({ path: Path.join(Output, 'RobotoJointHud.png') });
  for (const Axis of ['X', 'Y', 'Z']) {
    await Page.getByRole('spinbutton', { name: `Target ${Axis}`, exact: true }).focus();
    await Page.waitForFunction(Axis => document.querySelector('canvas[data-cad-ready=true]').dataset.selectedAxis === Axis.toLowerCase(), Axis);
    const Line = await Page.evaluate(() => { const Line = window.HudScene.getObjectByName('ActiveTargetAxis'); return { Visible: Line.visible, Rotation: Line.rotation.toArray(), Color: Line.material.color.getHexString() }; });
    Assert.equal(Line.Visible, true);
    Assert.equal(Line.Color, { X: 'f18b87', Y: '85d999', Z: '8db8ff' }[Axis]);
    if (Axis === 'X') await Page.screenshot({ path: Path.join(Output, 'RobotoTargetHud.png') });
  }
  await Page.getByRole('spinbutton', { name: 'Target Z', exact: true }).press('Enter');
  await Page.waitForFunction(() => !window.HudScene.getObjectByName('ActiveTargetAxis').visible);
  await Page.getByRole('button', { name: 'Full screen live view', exact: true }).click();
  Assert.equal(await Page.locator('.call-expanded').count(), 1);
  await Page.getByRole('button', { name: 'Emergency stop', exact: true }).waitFor();
  Assert.equal(await Page.evaluate(() => document.querySelector('video').srcObject === window.InitialHudStream), true);
  await Page.keyboard.press('Escape'); Assert.equal(await Page.locator('.call-expanded').count(), 0);
  await Page.getByRole('button', { name: 'AI control mode', exact: true }).click();
  Assert.equal(await Page.evaluate(() => document.querySelector('video') === window.InitialHudVideo && document.querySelector('video').srcObject === window.InitialHudStream && window.HudRenderer.domElement === window.InitialHudCanvas), true);
  await Page.getByRole('button', { name: 'Manual control mode', exact: true }).click();
  await Page.getByRole('button', { name: 'Set Current', exact: true }).click();
  const TimedNotice = Page.locator('.hud-notice').filter({ hasText: 'Target matched to live arm tip' }).last();
  await TimedNotice.waitFor();
  await Page.waitForTimeout(9200); Assert.equal(await TimedNotice.isVisible(), true);
  await TimedNotice.waitFor({ state: 'hidden', timeout: 2500 });
  // The floating camera intentionally stays above Settings; move it aside.
  for (let Step = 0; Step < 13; Step++) await Page.getByRole('button', { name: 'Move camera window', exact: true }).press('Shift+ArrowRight');
  await Page.getByRole('button', { name: 'Settings', exact: true }).click();
  const Panel = Page.getByRole('dialog', { name: 'Settings', exact: true });
  const Bottom = Panel.getByRole('checkbox', { name: 'Show bottom visualization text', exact: true });
  Assert.equal(await Bottom.isChecked(), false); await Bottom.check(); await Page.locator('.cad-footer').waitFor({ state: 'visible' });
  await Panel.getByRole('button', { name: 'Reset display settings', exact: true }).click(); Assert.equal(await Bottom.isChecked(), true);
  await Page.reload({ waitUntil: 'domcontentloaded' }); await CadReady();
  await Page.locator('.cad-footer').waitFor({ state: 'visible' }); // One-time local override never undoes a later user choice.
  await Page.getByRole('button', { name: 'Settings', exact: true }).click(); await Bottom.uncheck();
  await Panel.getByLabel('Render quality', { exact: true }).selectOption('Balanced');
  await Page.waitForFunction(() => document.querySelector('canvas[data-cad-ready=true]').dataset.renderQuality === 'Balanced');
  await Panel.getByLabel('Render quality', { exact: true }).selectOption('Ultra');
  await Panel.getByLabel('Camera profile', { exact: true }).selectOption('Detail');
  await Page.waitForFunction(() => document.querySelector('video').videoWidth >= 1280);
  await Page.waitForFunction(() => Number(document.querySelector('video').dataset.fps) >= 15);
  await Page.keyboard.press('Escape');
  const VideoStats = await Page.evaluate(() => ({ Width: document.querySelector('video').videoWidth, Height: document.querySelector('video').videoHeight,
    Quality: { Dropped: document.querySelector('video').getVideoPlaybackQuality().droppedVideoFrames, Total: document.querySelector('video').getVideoPlaybackQuality().totalVideoFrames }, Fps: document.querySelector('video').dataset.fps }));
  await Page.evaluate(() => { window.StalledHudStream = document.querySelector('video').srcObject; document.querySelector('video').pause(); });
  await Page.locator('.robot-avatar').filter({ hasText: 'Camera feed paused' }).waitFor({ timeout: 5000 });
  await Page.getByRole('button', { name: 'Settings', exact: true }).click();
  await Page.getByRole('button', { name: 'Reconnect camera', exact: true }).click();
  await Page.getByRole('button', { name: 'Close settings', exact: true }).click();
  await Page.waitForFunction(() => document.querySelector('video').srcObject !== window.StalledHudStream && document.querySelector('video').videoWidth > 0);
  Assert.equal(await Page.evaluate(() => window.StalledHudStream.getTracks().every(Track => Track.readyState === 'ended')), true);
  State.protection.trip = { at_s: 200, reason: 'Simulated thermal fault', coast_confirmed: false, unconfirmed: ['elbow_flex'] }; State.torque = false; State.motion_fault = 'Simulated thermal fault';
  Socket.send(JSON.stringify(State));
  await Page.getByText('Torque-off unconfirmed', { exact: true }).waitFor();
  await Page.locator('.protection-status').scrollIntoViewIfNeeded();
  await Page.screenshot({ path: Path.join(Output, 'RobotoProtectionHud.png') });
  Assert.deepEqual(Commands, []); Assert.deepEqual(Errors, []);
  await Page.getByRole('button', { name: 'Disconnect', exact: true }).click();
  await Page.getByRole('button', { name: 'Connect', exact: true }).waitFor();
  Assert.deepEqual(Commands.map(Command => Command.type), ['disconnect']); Commands.length = 0;
  Ports = ['FAKE'];
  await Page.reload({ waitUntil: 'domcontentloaded' }); await CadReady();
  await Page.waitForTimeout(500); Assert.deepEqual(Commands, [], 'Disconnect must persist through reload without auto-reconnecting');
  WriteFile(Path.join(Output, 'RobotoHudResults.json'), JSON.stringify({ VideoStats, InactiveHeight, ExpandedHeight, Commands, Errors }, null, 2));
  console.log('PASS: actual joint arc and mesh highlighting; precision step; axis focus; retained camera and stream; local preference versus defaults; render quality; timed top-right notice; protection status; fake video; no unintended commands.');
} finally { for (const Timer of Timers) clearInterval(Timer); await Browser.close(); await Server.Close(); }
