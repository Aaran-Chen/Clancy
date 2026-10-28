import { chromium as Chromium } from "playwright";
import { CadReadyMs, LaunchOptions, StartUiServer, BackendPython } from "./UiTestLib.mjs";
import { mkdirSync as MakeDirectory } from "node:fs";
import { execFileSync as Execute } from "node:child_process";
import { fileURLToPath as FilePath } from "node:url";
import Path from "node:path";
import Assert from "node:assert/strict";
const Root = Path.dirname(FilePath(import.meta.url)), Backend = Path.resolve(Root, "../back end");
const Python = BackendPython();
const State = JSON.parse(Execute(Python, ["-c", "import json, bridge_server; print(json.dumps(bridge_server.BridgeController().state_snapshot()))"], { cwd: Backend, encoding: "utf8" }));
const Output = Path.join(Backend, "TestArtifacts"); MakeDirectory(Output, { recursive: true });
const Server = await StartUiServer();
const Browser = await Chromium.launch(LaunchOptions());
const Page = await Browser.newPage({ viewport: { width: 1920, height: 1080 } });
const Errors = [], Commands = [], Timers = new Set();
await Page.route("**/LocalDisplayPreferences.json", Route => Route.fulfill({ json: {} }));
Page.on("pageerror", Error => Errors.push(Error.message));
await Page.route("**/bridge-launch/**", Route => Route.fulfill({ json: { ok: true } }));
await Page.route("**/ollama/**", Route => Route.fulfill({ json: { models: [] } }));
await Page.route("**/depth-server/**", Route => Route.fulfill({ json: {} }));
await Page.route("**:8000/**", Route => {
  const Url = new URL(Route.request().url());
  if (Url.pathname === "/ports") return Route.fulfill({ json: [] });
  if (Url.pathname === "/ik/preview") return Route.fulfill({ json: { ok: false } });
  if (Route.request().method() !== "GET") Commands.push({ Path: Url.pathname, Payload: Route.request().postData() });
  return Route.fulfill({ json: { ok: true, calibrated: false, points: [], point_count: 0 } });
});
await Page.routeWebSocket("**/ws", Socket => {
  Socket.send(JSON.stringify(State));
  const Timer = setInterval(() => Socket.send(JSON.stringify(State)), 100); Timers.add(Timer);
  Socket.onClose(() => { clearInterval(Timer); Timers.delete(Timer); });
  Socket.onMessage(Message => Commands.push(JSON.parse(String(Message))));
});
await Page.route("**/src/components/CadCanvas.tsx*", async Route => {
  const Response = await Route.fetch(), Original = await Response.text();
  const Instrumented = Original.replace("const Scene = new Three.Scene();", "const Scene = window.SettingsScene = new Three.Scene();")
    .replace("Renderer = new Three.WebGLRenderer(", "Renderer = window.SettingsRenderer = new Three.WebGLRenderer(")
    .replace("const Controls = new OrbitControls(", "const Controls = window.SettingsControls = new OrbitControls(");
  Assert.notEqual(Instrumented, Original);
  await Route.fulfill({ response: Response, body: Instrumented });
});
const WaitObject = (Name, Visible) => Page.waitForFunction(({ Name, Visible }) => {
  const Object = window.SettingsScene.getObjectByName(Name);
  let Shown = !!Object;
  for (let Parent = Object; Parent; Parent = Parent.parent) Shown &&= Parent.visible;
  return Shown === Visible;
}, { Name, Visible });
const Labels = () => Page.evaluate(() => window.SettingsScene.getObjectByName("WorkspaceOrigin").children.filter(Child => Child.isSprite && Child.visible).map(Child => Child.userData.Axis));
try {
  await Page.goto(Server.Url, { waitUntil: "domcontentloaded" });
  await Page.locator("canvas[data-cad-ready=true][data-assembly-ready=true]").waitFor({ timeout: CadReadyMs });
  // The floating camera stays above Settings by design. Move it out of the
  // settings column before exercising controls, including after reload.
  for (let Step = 0; Step < 13; Step++) await Page.getByRole('button', { name: 'Move camera window', exact: true }).press('Shift+ArrowRight');
  await Page.getByRole("button", { name: "Set Current", exact: true }).click();
  await Page.waitForFunction(() => document.querySelector("canvas[data-cad-ready=true]").dataset.pitchGuideState === "ready");
  const Before = await Page.evaluate(() => {
    window.SettingsControls.enableDamping = false; window.SettingsControls.update();
    window.OriginalSettingsCanvas = window.SettingsRenderer.domElement;
    window.OriginalSettingsAxis = window.SettingsScene.getObjectByName("CoordinateAxis+Y").geometry;
    const Axis = window.SettingsScene.getObjectByName("CoordinateAxis+Y");
    const Start = Axis.geometry.getAttribute("instanceStart"), End = Axis.geometry.getAttribute("instanceEnd");
    const Head = window.SettingsScene.getObjectByName("PositiveAxis+Y");
    return { Position: window.SettingsControls.object.position.toArray(), Target: window.SettingsControls.target.toArray(),
      Origin: window.SettingsScene.getObjectByName("WorkspaceOrigin").position.toArray(), Width: Axis.material.linewidth,
      IsWideLine: Axis.isLineSegments2, Start: [Start.getX(0), Start.getY(0), Start.getZ(0)], End: [End.getX(0), End.getY(0), End.getZ(0)], Head: Head.position.toArray() };
  });
  Assert.deepEqual(await Labels(), []); Assert.equal(Before.Width, 3); Assert.equal(Before.IsWideLine, true);
  Assert.ok(Math.hypot(...Before.Start.map((Value, Index) => Value - [0, -7, 0][Index])) < 1e-6);
  Assert.ok(Math.hypot(...Before.End.map((Value, Index) => Value - [0, 7, 0][Index])) < 1e-6);
  Assert.ok(Before.Head[1] > 0);
  await WaitObject("AcceptableWristAngles", true);
  await Page.screenshot({ path: Path.join(Output, "CleanCoordinateGraph.png") });
  const Open = Page.getByRole("button", { name: "Settings", exact: true });
  await Open.click();
  const Panel = Page.getByRole("dialog", { name: "Settings", exact: true });
  await Panel.waitFor();
  const Toggle = Label => Panel.getByRole("checkbox", { name: Label, exact: true });
  // The grasp gap at the fixed finger (lib/GraspGap.ts): 1.0 cm by default, 0.1 cm steps, saved in this browser.
  const GapSlider = Panel.getByRole("slider", { name: "Grasp gap at the fixed finger", exact: true });
  Assert.equal(await GapSlider.inputValue(), "1");
  Assert.deepEqual(await GapSlider.evaluate(Slider => [Slider.min, Slider.max, Slider.step]), ["0.5", "1.5", "0.1"]);
  // Below the default it moves only a round object's plan (the one that corrects for the real finger); the note says so.
  Assert.match(await GapSlider.locator("xpath=ancestor::fieldset").innerText(), /Below the default it applies to round objects \(cans, bottles\) only[^]*Other objects keep 1\.0 cm\./);
  await GapSlider.press("ArrowLeft"); await GapSlider.press("ArrowLeft");
  await Page.waitForFunction(() => localStorage.getItem("SO101.Dashboard.v1.GraspGapCm") === "0.8");
  Assert.match(await GapSlider.locator("xpath=..").innerText(), /Grasp gap at the fixed finger\s*0\.8 cm/);
  await Panel.getByLabel("Coordinate labels", { exact: true }).selectOption("PositiveY");
  await Page.waitForFunction(() => window.SettingsScene.getObjectByName("CoordinateLabel+Y").visible);
  Assert.deepEqual(await Labels(), ["+Y"]);
  await Panel.getByLabel("Coordinate labels", { exact: true }).selectOption("All");
  await Page.waitForFunction(() => window.SettingsScene.getObjectByName("CoordinateLabel+X").visible);
  Assert.deepEqual(await Labels(), ["+X", "+Y", "+Z"]);
  await Panel.getByRole("slider", { name: "Axis line thickness", exact: true }).press("End");
  await Page.waitForFunction(() => window.SettingsScene.getObjectByName("CoordinateAxis+Y").material.linewidth === 6);
  Assert.equal(await Page.evaluate(() => window.SettingsScene.getObjectByName("CoordinateAxis+Y").geometry === window.OriginalSettingsAxis), true, "Changing thickness reuses geometry");
  await Toggle("Show graph").uncheck();
  await WaitObject("WorkspaceOrigin", false); await WaitObject("WorkspaceGrid", false);
  await WaitObject("TargetApproachRay", true); await WaitObject("AcceptableWristAngles", true);
  Assert.equal(await Panel.getByLabel("Coordinate labels", { exact: true }).isDisabled(), true);
  await Toggle("Show graph").check(); await WaitObject("WorkspaceOrigin", true);
  await Toggle("Show floor grid").uncheck(); await WaitObject("WorkspaceGrid", false);
  await Toggle("Show floor grid").check(); await WaitObject("WorkspaceGrid", true);
  await Panel.getByRole("slider", { name: "Grid opacity", exact: true }).press("Home");
  await Page.waitForFunction(() => window.SettingsScene.getObjectByName("WorkspaceGrid").material.opacity === .1);
  for (const [Label, Name] of [["Show origin point", "OriginPoint"], ["Show robot body and basket", "XLeRobotBody"],
    ["Show inactive arm", "InactiveArm"], ["Show approach ray", "TargetApproachRay"], ["Show acceptable-angle indicator", "AcceptableWristAngles"]]) {
    await Toggle(Label).uncheck(); await WaitObject(Name, false);
    await Toggle(Label).check(); await WaitObject(Name, true);
  }
  await Toggle("Show target pose preview").uncheck(); await WaitObject("TargetPosePreview", false); await WaitObject("AiPosePreview", false);
  await Toggle("Show view cube").uncheck(); Assert.equal(await Page.getByLabel("View cube", { exact: true }).isVisible(), false);
  await Toggle("Show control hints").uncheck(); Assert.equal(await Page.locator('.cad-help').isVisible(), false);
  await Toggle("Show shadows").uncheck(); await Page.waitForFunction(() => !window.SettingsRenderer.shadowMap.enabled);
  for (const [Label, Property] of [["Zoom sensitivity", "zoomSpeed"], ["Orbit sensitivity", "rotateSpeed"], ["Pan sensitivity", "panSpeed"]]) {
    await Panel.getByRole("slider", { name: Label, exact: true }).press("End");
    await Page.waitForFunction(Property => window.SettingsControls[Property] === 2, Property);
  }
  const After = await Page.evaluate(() => ({ Position: window.SettingsControls.object.position.toArray(), Target: window.SettingsControls.target.toArray(), Origin: window.SettingsScene.getObjectByName("WorkspaceOrigin").position.toArray() }));
  for (const Key of ["Position", "Target", "Origin"]) Assert.ok(Math.hypot(...Before[Key].map((Value, Index) => Value - After[Key][Index])) < 1e-6, `${Key} must not change with display settings`);
  Assert.equal(await Page.evaluate(() => window.SettingsRenderer.domElement === window.OriginalSettingsCanvas), true);
  await Page.keyboard.press("Escape"); await Panel.waitFor({ state: "hidden" });
  Assert.equal(await Open.getAttribute("aria-expanded"), "false");
  await Page.getByRole("button", { name: "AI control mode", exact: true }).click(); await Open.click();
  Assert.equal(await Panel.getByRole("slider", { name: "Axis line thickness", exact: true }).inputValue(), "6");
  await Page.reload({ waitUntil: "domcontentloaded" });
  await Page.locator("canvas[data-cad-ready=true][data-assembly-ready=true]").waitFor({ timeout: CadReadyMs }); await Open.click();
  Assert.equal(await Panel.getByRole("slider", { name: "Axis line thickness", exact: true }).inputValue(), "6");
  Assert.equal(await Toggle("Show view cube").isChecked(), false);
  Assert.equal(await Panel.getByLabel("Coordinate labels", { exact: true }).inputValue(), "All");
  Assert.equal(await GapSlider.inputValue(), "0.8", "the grasp gap persists across a reload");
  await Panel.getByRole("button", { name: "Reset display settings", exact: true }).click();
  await Page.waitForFunction(() => window.SettingsScene.getObjectByName("CoordinateAxis+Y").material.linewidth === 3);
  Assert.deepEqual(await Labels(), []); Assert.equal(await Toggle("Show acceptable-angle indicator").isChecked(), true);
  Assert.equal(await Toggle("Show view cube").isChecked(), true);
  Assert.equal(await GapSlider.inputValue(), "0.8", "Reset display settings keeps the grasp gap (not a display setting)");
  await Panel.locator('.settings-content').evaluate(Element => Element.scrollTop = 0);
  await Page.screenshot({ path: Path.join(Output, "DisplaySettingsPanel.png") });
  await Page.setViewportSize({ width: 1000, height: 760 });
  const Bounds = await Panel.boundingBox(); Assert.ok(Bounds.x >= 0 && Bounds.x + Bounds.width <= 1000 && Bounds.y + Bounds.height <= 760);
  await Page.getByRole("button", { name: "Emergency stop", exact: true }).waitFor();
  Assert.deepEqual(Commands, []); Assert.deepEqual(Errors, []);
  console.log("PASS: thick unlabeled axes; positive Y direction; every display setting; ray/arc independent of graph; unchanged camera and origin; reusable geometry; modeless drawer; mode/reload persistence; reset; the grasp gap slider (default 1.0 cm, saved, kept by the display reset); narrow layout; no hardware writes or browser errors.");
} finally {
  for (const Timer of Timers) clearInterval(Timer);
  await Browser.close(); await Server.Close();
}
