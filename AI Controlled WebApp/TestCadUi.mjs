import { chromium as Chromium } from "playwright";
import { CadReadyMs, LaunchOptions, StartUiServer } from "./UiTestLib.mjs";
import { readFileSync as ReadFile, mkdirSync as MakeDirectory, writeFileSync as WriteFile } from "node:fs";
import { execFileSync as Execute } from "node:child_process";
import { fileURLToPath as FilePath } from "node:url";
import Path from "node:path";
import Assert from "node:assert/strict";

const Root = Path.dirname(FilePath(import.meta.url));
const Backend = Path.resolve(Root, "../back end");
const Python = ReadFile(Path.join(Root, "../.backend-python.txt"), "utf8").trim();
const State = JSON.parse(Execute(Python, ["-c", "import json, bridge_server; print(json.dumps(bridge_server.BridgeController().state_snapshot()))"], { cwd: Backend, encoding: "utf8" }));
State.status = "CAD preview";
const Output = Path.join(Backend, "TestArtifacts"); MakeDirectory(Output, { recursive: true });
const Server = await StartUiServer();
const Browser = await Chromium.launch(LaunchOptions());
const Page = await Browser.newPage({ viewport: { width: 1920, height: 1080 } });
const Errors = [], Commands = [];
Page.on("pageerror", Error => Errors.push(Error.message));
await Page.route("**/bridge-launch/**", Route => Route.fulfill({ json: { ok: true } }));
await Page.route("**/ollama/**", Route => Route.fulfill({ json: { models: [] } }));
await Page.route("**/depth-server/**", Route => Route.fulfill({ json: {} }));
await Page.route("**:8000/**", Route => {
  const Url = new URL(Route.request().url());
  if (Url.pathname === "/ports") return Route.fulfill({ json: [] });
  if (Url.pathname === "/ik/preview") return Route.fulfill({ json: { ok: false } });
  if (Url.pathname === "/cad/alignment") {
    Commands.push({ Endpoint: Url.pathname, Payload: Route.request().postDataJSON() });
    return Route.fulfill({ json: { ok: false, error: "Test must not capture a physical reference" } });
  }
  if (!["GET", "HEAD"].includes(Route.request().method())) {
    Commands.push({ Endpoint: Url.pathname, Method: Route.request().method(), Payload: Route.request().postData() });
  }
  return Route.fulfill({ json: { ok: true, calibrated: false, points: [], point_count: 0 } });
});
let Timer;
await Page.routeWebSocket("**/ws", Socket => {
  Socket.send(JSON.stringify(State));
  Timer = setInterval(() => Socket.send(JSON.stringify(State)), 150);
  Socket.onMessage(Message => Commands.push(JSON.parse(String(Message))));
});
try {
  await Page.goto(Server.Url, { waitUntil: "domcontentloaded" });
  const Canvas = Page.locator("canvas[data-cad-ready=true]");
  await Canvas.waitFor({ timeout: CadReadyMs });
  await Page.locator("canvas[data-assembly-ready=true]").waitFor({ timeout: 30000 });
  Assert.equal(await Canvas.getAttribute("data-body-parts"), "32");
  Assert.equal(await Canvas.getAttribute("data-view-scope"), "Robot");
  Assert.equal(await Canvas.getAttribute("data-cad-parts"), "17");
  Assert.equal(await Page.locator(".cad-error").count(), 0);
  await Page.getByRole("button", { name: "Set Current", exact: true }).click();
  await Page.screenshot({ path: Path.join(Output, "XLeRobotGui.png") });
  await Page.locator(".cad-shell").screenshot({ path: Path.join(Output, "XLeRobotDetail.png") });
  const Tools = Page.getByLabel("CAD view controls", { exact: true });
  Assert.deepEqual(await Tools.getByRole("button").evaluateAll(Buttons => Buttons.map(Button => Button.getAttribute("aria-label"))),
    ["CAD reference alignment", "Fit model in view", "Show axis spacing", "Full screen"]);
  Assert.equal(await Page.getByRole("button", { name: "Show whole robot", exact: true }).count(), 0);
  Assert.equal(await Page.getByRole("button", { name: "Focus on controlled arm", exact: true }).count(), 0);
  Assert.equal(await Page.locator(".view-cube-face").count(), 6);
  const PositionTiles = Page.locator(".current-position-panel .position-tile strong");
  Assert.equal(await PositionTiles.count(), 4);
  const PositionBefore = await PositionTiles.allTextContents();
  const ReadTargets = () => Page.locator(".target-grid input").evaluateAll(Inputs => Inputs.map(Input => Input.value));
  const TargetsBefore = await ReadTargets();
  Assert.equal(TargetsBefore.length, 3);
  Assert.equal(await Page.locator(".cad-heading, .cad-side").count(), 0);
  Assert.ok(JSON.parse(await Canvas.getAttribute("data-workspace-origin")).every(Value => Math.abs(Value) < 1e-8));
  Assert.deepEqual(await PositionTiles.allTextContents(), PositionBefore);
  Assert.deepEqual(await ReadTargets(), TargetsBefore);
  await Page.getByRole("button", { name: "Show axis spacing", exact: true }).click();
  await Page.getByText("116.00 mm", { exact: false }).waitFor();
  await Page.getByText("135.00 mm", { exact: false }).waitFor();
  await Page.screenshot({ path: Path.join(Output, "CadGui.png") });
  await Page.locator(".cad-shell").screenshot({ path: Path.join(Output, "CadDetail.png") });
  const Before = await Canvas.screenshot();
  await Page.getByRole("button", { name: "View top", exact: true }).click();
  Assert.notDeepEqual(await Canvas.screenshot(), Before);
  await Page.getByRole("button", { name: "Fit model in view", exact: true }).click();
  await Page.getByRole("button", { name: "View front", exact: true }).click();
  Assert.equal(await Canvas.getAttribute("data-view-scope"), "Robot");
  await Page.getByRole("button", { name: "Fit model in view", exact: true }).click();
  await Page.locator(".cad-alignment").click();
  await Page.getByText("Match this reference pose", { exact: true }).waitFor();
  Assert.equal(await Page.getByRole("button", { name: "I've matched this pose, record alignment" }).isDisabled(), true);
  await Page.locator(".cad-shell").screenshot({ path: Path.join(Output, "CadReference.png") });
  await Page.getByRole("button", { name: "Return to live pose", exact: true }).click();
  await Page.locator("canvas[data-view-scope=Robot]").waitFor();
  await Page.getByRole("button", { name: "Show axis spacing", exact: true }).click();
  await Page.setViewportSize({ width: 1440, height: 1000 });
  await Page.screenshot({ path: Path.join(Output, "XLeRobotGui1440.png") });
  await Page.getByRole("button", { name: "Full screen", exact: true }).click();
  await Page.waitForFunction(() => document.fullscreenElement !== null);
  await Page.screenshot({ path: Path.join(Output, "XLeRobotFullscreen.png") });
  await Page.getByRole("button", { name: "Full screen", exact: true }).click();
  await Page.waitForFunction(() => document.fullscreenElement === null);
  Assert.deepEqual(Errors, []);
  Assert.deepEqual(Commands, []);
  WriteFile(Path.join(Output, "XLeRobotUiVerification.json"), JSON.stringify({ LoadedArmParts: 17, LoadedBodyParts: 32, BrowserErrors: Errors, HardwareCommands: Commands, Views: ["Whole robot", "View cube", "Reference", "Fullscreen"], RemovedToolbarButtons: ["Robot", "Arm", "ISO", "Top", "Front"], Viewports: [1920, 1440] }, null, 2));
  console.log("XLeRobot UI: redundant toolbar buttons removed; view cube, full model, fullscreen and dimensions work; no motor commands.");
} finally { clearInterval(Timer); await Browser.close(); await Server.Close(); }
