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
const Errors = [], Commands = [], Requests = [], Preloads = [], Timers = new Set();
let Scenario = "usage", Release;
Page.on("pageerror", Error => Errors.push(Error.message));
await Page.route("**/bridge-launch/**", Route => Route.fulfill({ json: { ok: true } }));
await Page.route("**/depth-server/**", Route => Route.fulfill({ json: {} }));
await Page.route("**/ollama/**", async Route => {
  const Url = new URL(Route.request().url());
  if (Url.pathname.endsWith("/tags")) return Route.fulfill({ json: { models: [{ name: "DashboardTest", size: 1000 }] } });
  if (Url.pathname.endsWith("/show")) return Route.fulfill({ json: { capabilities: ["vision"] } });
  if (Url.pathname.endsWith("/ps")) return Route.fulfill({ json: { models: [{ name: "DashboardTest", context_length: 32768 }] } });
  if (Url.pathname.endsWith("/api/chat")) {
    const Body = Route.request().postDataJSON();
    // The page's warm-up: no messages, so Ollama only loads the model.
    if (Body.messages.length === 0) {
      Preloads.push(Body);
      return Route.fulfill({ json: { model: Body.model, message: { role: "assistant", content: "" }, done: true, done_reason: "load" } });
    }
    Requests.push(Body);
    if (Scenario === "failure") return Route.fulfill({ status: 500, body: "Simulated model error" });
    if (Scenario === "delayed") await new Promise(Resolve => { Release = Resolve; });
    const Data = { model: Body.model, message: { role: "assistant", content: JSON.stringify({ response_to_user: "Dashboard test response", task_status: "done", motion_proposal: null }) },
      done: true, done_reason: "stop", ...(Scenario === "missing" ? {} : { prompt_eval_count: 120, eval_count: 30 }) };
    try { return await Route.fulfill({ json: Data }); } catch { return; }
  }
  return Route.fulfill({ json: { data: [{ id: "DashboardTest" }] } });
});
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
await Page.route("**/three_examples_jsm_controls_OrbitControls__js.js*", async Route => {
  const Response = await Route.fetch(), Original = await Response.text();
  const Instrumented = Original.replace(/export\s*\{\s*OrbitControls\s*\};/, `
    class ObservedOrbitControls extends OrbitControls {
      constructor(...Args) { super(...Args); window.DashboardOrbit = this; }
    }
    export { ObservedOrbitControls as OrbitControls };`);
  Assert.notEqual(Instrumented, Original);
  await Route.fulfill({ response: Response, body: Instrumented });
});
try {
  await Page.goto(Server.Url, { waitUntil: "domcontentloaded" });
  await Page.locator("canvas[data-cad-ready=true][data-assembly-ready=true]").waitFor({ timeout: CadReadyMs });
  await Page.evaluate(() => { window.DashboardVideo = document.querySelector("video"); window.DashboardCanvas = document.querySelector("canvas[data-cad-ready=true]"); });
  const Manual = Page.getByRole("button", { name: "Manual control mode", exact: true });
  const Ai = Page.getByRole("button", { name: "AI control mode", exact: true });
  const Emergency = Page.getByRole("button", { name: "Emergency stop", exact: true });
  Assert.equal(await Manual.getAttribute("aria-pressed"), "true");
  Assert.equal(await Page.getByRole("separator").count(), 2);
  Assert.equal(await Page.locator('[data-section="AiControl"]').isVisible(), false);
  await Page.screenshot({ path: Path.join(Output, "MinimalManualDashboard.png"), animations: 'disabled' });
  const Sizes = () => Page.locator(".workspace > aside:visible, .workspace > section:visible").evaluateAll(Elements => Elements.sort((A, B) => A.getBoundingClientRect().x - B.getBoundingClientRect().x).map(Element => Element.getBoundingClientRect().width));
  const Initial = await Sizes();
  await Page.getByRole("button", { name: "View front", exact: true }).click();
  await Page.evaluate(() => { window.DashboardOrbit.enableDamping = false; window.DashboardOrbit.update(); });
  const CameraPose = () => Page.evaluate(() => ({ Position: window.DashboardOrbit.object.position.toArray(), Target: window.DashboardOrbit.target.toArray(), Zoom: window.DashboardOrbit.object.zoom }));
  const SameCamera = (Actual, Expected) => {
    for (const Key of ["Position", "Target"]) Assert.ok(Math.hypot(...Actual[Key].map((Value, Index) => Value - Expected[Key][Index])) < 1e-6, `Camera ${Key} changed during tiling`);
    Assert.ok(Math.abs(Actual.Zoom - Expected.Zoom) < 1e-9);
  };
  const OriginalCamera = await CameraPose();
  const Divider = Page.getByRole("separator").first();
  const Bounds = await Divider.boundingBox();
  await Page.mouse.move(Bounds.x + Bounds.width / 2, Bounds.y + 180); await Page.mouse.down();
  await Page.mouse.move(Bounds.x + Bounds.width / 2 + 60, Bounds.y + 180, { steps: 20 }); await Page.mouse.up();
  const Resized = await Sizes();
  Assert.ok(Math.abs(Resized[0] - Initial[0] - 60) < 1);
  Assert.ok(Math.abs(Resized[1] - Initial[1] + 60) < 1);
  Assert.ok(Math.abs(Resized[2] - Initial[2]) < 1);
  SameCamera(await CameraPose(), OriginalCamera);
  await Divider.press("Home"); Assert.ok(Math.abs((await Sizes())[0] - 320) < 1);
  await Divider.press("ArrowRight"); Assert.ok(Math.abs((await Sizes())[0] - 336) < 1);
  const Saved = await Sizes();
  const Again = await Divider.boundingBox();
  await Page.mouse.move(Again.x + 3, Again.y + 180); await Page.mouse.down();
  await Page.mouse.move(Again.x + 93, Again.y + 180); await Page.keyboard.press("Escape"); await Page.mouse.up();
  Assert.ok(Math.abs((await Sizes())[0] - Saved[0]) < 1, "Escape restores the divider position");
  for (const Title of ["Live Feed", "Joints", "Current Position", "Target", "Motion"]) {
    await Page.getByRole("button", { name: `Minimize ${Title}`, exact: true }).click();
    Assert.equal(await Page.getByRole("button", { name: `Restore ${Title}`, exact: true }).getAttribute("aria-expanded"), "false");
    await Page.getByRole("button", { name: `Restore ${Title}`, exact: true }).click();
  }
  await Page.getByRole("button", { name: "Minimize Current Position", exact: true }).click();
  await Page.reload({ waitUntil: "domcontentloaded" });
  await Page.locator("canvas[data-cad-ready=true][data-assembly-ready=true]").waitFor({ timeout: CadReadyMs });
  Assert.ok(Math.abs((await Sizes())[0] - Saved[0]) < 1, "Widths persist after reload");
  await Page.getByRole("button", { name: "Restore Current Position", exact: true }).waitFor();
  await Page.getByRole("button", { name: "Restore Current Position", exact: true }).click();
  await Page.evaluate(() => { window.DashboardVideo = document.querySelector("video"); window.DashboardCanvas = document.querySelector("canvas[data-cad-ready=true]"); });
  // Manual mode never asks the model, so nothing loads it (it would hold ~22 GB of VRAM).
  Assert.equal(Preloads.length, 0, "No model warm-up in Manual mode");
  const BeforeModeCamera = await CameraPose();
  await Page.screenshot({ path: Path.join(Output, "DashboardManual.png") });
  await Ai.click();
  await Page.waitForTimeout(100);
  SameCamera(await CameraPose(), BeforeModeCamera);
  Assert.equal(await Page.getByRole("separator").count(), 1);
  Assert.equal(await Page.locator(".slider-column").isVisible(), false);
  Assert.equal(await Page.locator('[data-section="AiStatistics"]').isVisible(), true);
  const AiPanelBounds = await Page.locator('[data-section="AiControl"]').boundingBox();
  const StatisticsBounds = await Page.locator('[data-section="AiStatistics"]').boundingBox();
  Assert.ok(AiPanelBounds.y < StatisticsBounds.y, "The task input comes before usage statistics");
  await Page.locator('.ai-companion[data-state="ready"]').waitFor();
  for (let Attempt = 0; Attempt < 100 && Preloads.length === 0; Attempt++) await Page.waitForTimeout(20);
  Assert.equal(Preloads.length, 1, "AI mode warms the model");
  Assert.equal(await Page.locator('.ai-approval-box .coordinate-tile').count(), 0, "No coordinates are presented as a proposal before the AI proposes a move");
  Assert.equal(await Page.getByRole('textbox', { name: 'AI task', exact: true }).isVisible(), true);
  Assert.equal(await Page.locator('.dashboard-app').evaluate(Element => getComputedStyle(Element, '::after').content), 'none', "No film overlay obscures the workspace");
  await Page.screenshot({ path: Path.join(Output, "MinimalAiDashboard.png"), animations: 'disabled' });
  const Submit = async Text => {
    await Page.getByPlaceholder("Describe a task or movement").fill(Text);
    await Page.getByRole("button", { name: "Send to AI", exact: true }).click();
  };
  await Submit("Describe the workspace; do not move.");
  await Page.waitForFunction(() => document.querySelector('[data-stat="total"] strong')?.textContent === "150");
  Assert.equal(await Page.locator('[data-stat="input"] strong').innerText(), "120");
  Assert.equal(await Page.locator('[data-stat="output"] strong').innerText(), "30");
  Assert.equal(await Page.locator('[data-stat="session"]').innerText(), "150");
  await Submit("Describe it again without moving.");
  await Page.waitForFunction(() => document.querySelector('[data-stat="session"]')?.textContent === "300");
  await Page.getByRole("button", { name: "Minimize Live Feed", exact: true }).click();
  await Page.locator('.ai-column').evaluate(Element => Element.scrollTop = 0);
  await Page.screenshot({ path: Path.join(Output, "DashboardAi.png") });
  Scenario = "missing";
  await Submit("Respond without moving.");
  await Page.waitForFunction(() => document.querySelector('[data-stat="total"] strong')?.textContent === "--");
  Assert.equal(await Page.locator('[data-stat="session"]').innerText(), "300", "Unknown usage is not fabricated");
  Scenario = "failure";
  await Submit("One more description.");
  await Page.getByText("Failed requests: 1", { exact: true }).waitFor();
  Scenario = "delayed";
  await Submit("Slow response with no movement.");
  await Page.waitForFunction(() => document.querySelector('.ai-model-stat span')?.textContent === "Request in progress");
  await Page.locator('.ai-companion[data-state="thinking"]').waitFor();
  while (!Release) await Page.waitForTimeout(20);
  await Manual.click();
  Release();
  await Page.waitForTimeout(300);
  Assert.ok(await Page.evaluate(() => document.querySelector("video") === window.DashboardVideo && document.querySelector("canvas[data-cad-ready=true]") === window.DashboardCanvas), "Modes keep the camera and CAD mounted");
  Assert.equal(await Page.getByRole("button", { name: "Approve", exact: true }).count(), 0);
  Assert.equal(await Page.locator('.ai-task-strip').count(), 0, "Manual mode cancels AI continuation");
  Assert.equal(await Page.getByRole("button", { name: "Restore Live Feed", exact: true }).count(), 1, "Floating camera stays minimized across modes");
  Assert.ok(Math.abs((await Sizes())[0] - Saved[0]) < 1);
  await Page.setViewportSize({ width: 1000, height: 760 });
  await Page.waitForTimeout(200);
  Assert.equal(await Page.locator(".slider-column").isVisible(), true, "Narrow windows retain manual controls");
  await Page.getByRole("button", { name: "Minimize Motion", exact: true }).scrollIntoViewIfNeeded();
  await Page.getByRole("button", { name: "Minimize Motion", exact: true }).click();
  Assert.equal(await Emergency.isVisible(), true);
  const EmergencyBounds = await Emergency.boundingBox();
  Assert.ok(EmergencyBounds.x >= 0 && EmergencyBounds.x + EmergencyBounds.width <= 1000 && EmergencyBounds.y >= 0);
  await Page.setViewportSize({ width: 1920, height: 1080 });
  await Ai.click();
  await Page.reload({ waitUntil: "domcontentloaded" });
  await Ai.waitFor();
  Assert.equal(await Ai.getAttribute("aria-pressed"), "true", "Mode persists after reload");
  Assert.equal(await Page.getByRole("button", { name: "Restore Live Feed", exact: true }).count(), 1);
  await Page.emulateMedia({ reducedMotion: 'reduce' });
  Assert.equal(await Page.locator('.ai-companion .pixel-bot-eyes').evaluate(Element => getComputedStyle(Element).animationName), 'none', "Pixel character respects reduced motion");
  for (const Width of [1366, 900]) {
    await Page.setViewportSize({ width: Width, height: 800 });
    await Page.getByRole('textbox', { name: 'AI task', exact: true }).scrollIntoViewIfNeeded();
    const InputBounds = await Page.getByRole('textbox', { name: 'AI task', exact: true }).boundingBox();
    const PanelBounds = await Page.locator('[data-section="AiControl"]').boundingBox();
    Assert.ok(InputBounds.x >= PanelBounds.x && InputBounds.x + InputBounds.width <= PanelBounds.x + PanelBounds.width, 'Task input stays within its panel');
    const StopBounds = await Emergency.boundingBox();
    Assert.equal(await Page.evaluate(Bounds => Boolean(document.elementFromPoint(Bounds.x + Bounds.width / 2, Bounds.y + Bounds.height / 2)?.closest('.topbar-emergency')), StopBounds), true, 'Emergency stop remains unobstructed');
    const NoticeBounds = await Page.locator('.hud-notices').boundingBox();
    const SceneBounds = await Page.locator('.scene-column').boundingBox();
    const ToolsBounds = await Page.locator('.cad-tools').boundingBox();
    Assert.ok(NoticeBounds.x >= SceneBounds.x && NoticeBounds.x + NoticeBounds.width <= SceneBounds.x + SceneBounds.width, 'Notifications stay off the control panels');
    Assert.ok(NoticeBounds.y >= ToolsBounds.y + ToolsBounds.height, 'Notifications stay below the CAD tools');
    await Page.screenshot({ path: Path.join(Output, `MinimalAiDashboard${Width}.png`), animations: 'disabled' });
  }
  Assert.ok(Requests.length === 5);
  // Once per page load: the prompts and the switch back to AI mode reused it; the reload warmed it again.
  Assert.equal(Preloads.length, 2);
  for (const Preload of Preloads) {
    Assert.deepEqual([Preload.model, Preload.options.num_ctx, Preload.options.num_batch, Preload.keep_alive],
      [Requests[0].model, Requests[0].options.num_ctx, Requests[0].options.num_batch, Requests[0].keep_alive], "The warm-up loads the model the way the calls use it");
  }
  Assert.deepEqual(Commands, [], "Dashboard actions must never move the arm or change torque");
  Assert.deepEqual(Errors, []);
  console.log("PASS: mouse and keyboard tiling; minimum widths; Escape; collapse and restore; per-mode persistence; mounted camera/CAD; real request usage aggregation; missing usage; failures; Manual aborts AI; narrow-window access; persistent emergency stop; no motor commands or browser errors.");
} finally {
  Release?.();
  for (const Timer of Timers) clearInterval(Timer);
  await Browser.close(); await Server.Close();
}
