import { chromium as Chromium } from "playwright";
import { CadReadyMs, LaunchOptions, StartUiServer, BackendPython } from "./UiTestLib.mjs";
import { execFileSync as Execute } from "node:child_process";
import { fileURLToPath as FilePath } from "node:url";
import Path from "node:path";
import Assert from "node:assert/strict";

// Audit 2 M3: a latched fault was recorded (bridge_fault) again on every page load and every reconnect, because the
// socket close forgets the fault the page last saw. One fault is one record: the same bridge event (its time, t)
// seen again after a reconnect or a reload is tagged seen_again; a new event is recorded as new. Everything is mocked
// (bridge, camera); no request reaches the real bridge or arm. It serves this checkout on 5191-5199 (UiTestLib.mjs).
const Root = Path.dirname(FilePath(import.meta.url)), Backend = Path.resolve(Root, "../back end");
const Python = BackendPython();
const State = JSON.parse(Execute(Python, ["-c", "import json, bridge_server; print(json.dumps(bridge_server.BridgeController().state_snapshot()))"], { cwd: Backend, encoding: "utf8" }));
const Fault = "Motion settling timed out on elbow_flex.";
Object.assign(State, { arm_connected: true, torque: true, port: "FAKE", motion_fault: Fault,
  last_bridge_event: { kind: "motion_fault", reason: Fault, joint: "elbow_flex", details: null, t: 1761400000.123, time: "2026-10-25T13:46:40.123Z" } });
const Server = await StartUiServer();
const Browser = await Chromium.launch(LaunchOptions({ args: ["--use-fake-device-for-media-stream", "--use-fake-ui-for-media-stream"] }));
const Page = await Browser.newPage({ viewport: { width: 1600, height: 1000 } });
const Errors = [], Events = [], Timers = new Set();
const Pause = Milliseconds => new Promise(Resolve => setTimeout(Resolve, Milliseconds));
Page.on("pageerror", Error => Errors.push(Error.message));
// The session record is skipped for automated browsers; show it to this test.
await Page.addInitScript(() => Object.defineProperty(Navigator.prototype, "webdriver", { get: () => false }));
await Page.route("**/session-log/**", Route => { const Body = Route.request().postDataJSON?.() ?? null; if (Body) Events.push(Body); return Route.fulfill({ json: { ok: true } }); });
await Page.route("**/bridge-launch/**", Route => Route.fulfill({ json: { ok: true } }));
await Page.route("**/ollama/**", Route => Route.fulfill({ json: { models: [] } }));
await Page.route("**/depth-server/**", Route => Route.fulfill({ json: {} }));
await Page.route("**:8000/**", Route => {
  const Url = new URL(Route.request().url());
  if (Url.pathname === "/ports") return Route.fulfill({ json: ["FAKE"] });
  if (Url.pathname === "/ik/preview") return Route.fulfill({ json: { ok: false } });
  return Route.fulfill({ json: { ok: true, calibrated: false, points: [] } });
});
let Socket;
await Page.routeWebSocket("**/ws", Connected => {
  Socket = Connected; Connected.send(JSON.stringify(State));
  const Timer = setInterval(() => Connected.send(JSON.stringify(State)), 100); Timers.add(Timer);
  Connected.onClose(() => { clearInterval(Timer); Timers.delete(Timer); });
  Connected.onMessage(() => {});
});
const Faults = () => Events.filter(Event => Event.type === "bridge_fault");
const New = () => Faults().filter(Event => !Event.seen_again);
const Reconnect = async () => {
  Socket.close();
  await Page.locator(".connection-pill.offline").waitFor();
  await Page.getByRole("button", { name: "Connect", exact: true }).click();
  await Page.locator(".connection-pill.offline").waitFor({ state: "hidden" });
  await Pause(1000);
};
try {
  await Page.goto(Server.Url, { waitUntil: "domcontentloaded" });
  await Page.locator("canvas[data-cad-ready=true][data-assembly-ready=true]").waitFor({ timeout: CadReadyMs });
  await Pause(1000);
  Assert.equal(New().length, 1, "the fault is recorded once when the page first sees it");
  Assert.equal(New()[0].reason, Fault); Assert.equal(New()[0].joint, "elbow_flex");

  await Reconnect();
  Assert.equal(New().length, 1, `a reconnect never records the same latched fault as new: ${JSON.stringify(Faults())}`);

  await Page.reload({ waitUntil: "domcontentloaded" });
  await Page.locator("canvas[data-cad-ready=true][data-assembly-ready=true]").waitFor({ timeout: CadReadyMs });
  await Pause(1000);
  Assert.equal(New().length, 1, `a page load never records the same latched fault as new: ${JSON.stringify(Faults())}`);
  Assert.ok(Faults().length >= 2 && Faults().slice(1).every(Event => Event.seen_again === true), "repeats are kept, tagged seen_again");

  // Hold clears it; the same trip again (a new bridge event, same text) is a new fault, after a reload too.
  State.motion_fault = null; await Pause(800);
  Object.assign(State, { motion_fault: Fault, last_bridge_event: { ...State.last_bridge_event, t: State.last_bridge_event.t + 60 } });
  await Pause(800);
  Assert.equal(New().length, 2, "a fault after a clear is recorded as new");
  State.motion_fault = null; await Pause(800);
  await Reconnect();
  State.motion_fault = Fault; await Pause(800);
  Assert.equal(New().length, 3, "a fault after a clear is recorded as new, across a reconnect");
  Assert.deepEqual(Errors, []);
  console.log("PASS: a latched fault is recorded once; a reconnect and a page load tag it seen_again; a fault after a clear is recorded as new.");
} finally { Timers.forEach(Timer => clearInterval(Timer)); await Browser.close(); await Server.Close(); }
