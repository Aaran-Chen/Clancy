// Hand guiding in the real page (Playwright, this checkout served on 5191-5199 by UiTestLib, the bridge faked over its
// socket with the real state_snapshot): the Motion panel's button through calibrating, guiding and Hold here; the first
// use's confirmation; the strip and the emergency stop that say the arm goes limp; Send and Home held while guiding; the
// heartbeat; AI mode holding first. Leaves a screenshot in back end/TestArtifacts/HandGuideUi.png.
import { chromium as Chromium } from "playwright";
import { CadReadyMs, LaunchOptions, StartUiServer, BackendPython } from "./UiTestLib.mjs";
import { mkdirSync as MakeDirectory } from "node:fs";
import { execFileSync as Execute } from "node:child_process";
import { fileURLToPath as FilePath } from "node:url";
import Path from "node:path";
import Assert from "node:assert/strict";

const Root = Path.dirname(FilePath(import.meta.url)), Backend = Path.resolve(Root, "../back end");
const Python = BackendPython();
const State = JSON.parse(Execute(Python, ["-c", "import json, bridge_server; print(json.dumps(bridge_server.BridgeController().state_snapshot()))"],
  { cwd: Backend, encoding: "utf8" }));
Assert.equal(State.hand_guide_version, "guide-v1", "the bridge reports hand guiding");
State.arm_connected = true; State.torque = true; State.port = "FAKE";
State.hand_guide = { ...State.hand_guide, calibrated: false };
const Output = Path.join(Backend, "TestArtifacts"); MakeDirectory(Output, { recursive: true });
const Server = await StartUiServer();
const Browser = await Chromium.launch(LaunchOptions({ args: ["--use-fake-device-for-media-stream", "--use-fake-ui-for-media-stream"] }));
const Page = await Browser.newPage({ viewport: { width: 1920, height: 1080 } });
const Errors = [], Commands = [], Dialogs = [], Timers = new Set();
Page.on("pageerror", Error => Errors.push(Error.message));
Page.on("dialog", Dialog => { Dialogs.push(Dialog.message()); void Dialog.accept(); });
await Page.route("**/bridge-launch/**", Route => Route.fulfill({ json: { ok: true } }));
await Page.route("**/ollama/**", Route => Route.fulfill({ json: { models: [] } }));
await Page.route("**/depth-server/**", Route => Route.fulfill({ json: {} }));
await Page.route("**:8000/**", Route => {
  const Url = new URL(Route.request().url());
  if (Url.pathname === "/ports") return Route.fulfill({ json: ["FAKE"] });
  if (Url.pathname === "/ik/preview") return Route.fulfill({ json: { ok: false } });
  return Route.fulfill({ json: { ok: true, calibrated: false, points: [] } });
});
await Page.routeWebSocket("**/ws", Connected => {
  Connected.send(JSON.stringify(State));
  const Timer = setInterval(() => Connected.send(JSON.stringify(State)), 100); Timers.add(Timer);
  Connected.onClose(() => { clearInterval(Timer); Timers.delete(Timer); });
  Connected.onMessage(Message => { Commands.push(JSON.parse(String(Message))); });
});
const Sent = Type => Commands.filter(Command => Command.type === Type);
const Until = async (Condition, What, Ms = 5000) => {
  const End = Date.now() + Ms;
  while (!Condition()) { if (Date.now() > End) throw new Error(`timed out: ${What}`); await new Promise(Resolve => setTimeout(Resolve, 50)); }
};
try {
  await Page.goto(Server.Url, { waitUntil: "domcontentloaded" });
  await Page.locator("canvas[data-cad-ready=true][data-assembly-ready=true]").waitFor({ timeout: CadReadyMs });
  // Not calibrated: the button calibrates, after its confirmation
  const Calibrate = Page.getByRole("button", { name: "Calibrate guiding", exact: true });
  await Calibrate.waitFor();
  await Calibrate.click();
  await Until(() => Sent("hand_guide_calibrate").length === 1, "the calibration command");
  Assert.deepEqual(Sent("hand_guide_calibrate")[0], { type: "hand_guide_calibrate", explicit: true });
  Assert.match(Dialogs.at(-1), /about 6 minutes/);
  State.hand_guide = { ...State.hand_guide, calibrating: true, progress: 0.25, phase: "sweep", roles: {}, notes: [] };
  await Page.getByRole("button", { name: "Stop calibration", exact: true }).waitFor();
  await Page.locator(".hand-guide-status").filter({ hasText: "Calibrating hand guiding · 25 %" }).waitFor();
  State.hand_guide = { ...State.hand_guide, calibrating: false, calibrated: true, calibrated_at: "2026-10-10T12:00:00",
    calibration_result: { ok: true, problems: [], error: null, at_s: 1 } };
  // Calibrated: Hand guide, its first use explained once
  const Guide = Page.getByRole("button", { name: "Hand guide", exact: true });
  await Guide.waitFor();
  await Page.getByRole("button", { name: "Calibrate hand guiding again", exact: true }).waitFor();
  await Guide.click();
  await Until(() => Sent("hand_guide").length === 1, "the hand guide command");
  Assert.deepEqual(Sent("hand_guide")[0], { type: "hand_guide", enable: true, explicit: true });
  Assert.match(Dialogs.at(-1), /goes limp/); Assert.match(Dialogs.at(-1), /Emergency stop/);
  State.hand_guide = { ...State.hand_guide, on: true, phase: "guide", since_s: 3, owner: "this PC", start_left_s: 0,
    roles: { shoulder_pan: "free", shoulder_lift: "comp", elbow_flex: "comp", wrist_flex: "comp", wrist_roll: "free", gripper: "free" },
    free_states: { shoulder_pan: "float", wrist_roll: "float", gripper: "float" }, notes: [],
    assist: { shoulder_lift: { on: true }, elbow_flex: { on: true }, wrist_flex: { on: true }, wrist_roll: { on: true, band_top_deg_s: 23 } } };
  const Hold = Page.getByRole("button", { name: "Hold here", exact: true });
  await Hold.waitFor();
  const Strip = Page.locator(".hand-guide-status");
  await Strip.filter({ hasText: "Hand guiding: move the arm by hand. Hold here to stop" }).waitFor();
  Assert.match(await Strip.innerText(), /The emergency stop, or anything going wrong, makes it go limp: keep a hand on it/);
  await Page.getByRole("button", { name: "Emergency stop: the arm drops", exact: true }).waitFor();
  // Coast keeps its name, and says it drops the arm (2026-10-10 review L5)
  const Coast = Page.getByRole("button", { name: "Coast", exact: true });
  Assert.equal((await Coast.textContent()).trim(), "Coast: arm drops");
  Assert.match(await Coast.getAttribute("title"), /torque off: the arm drops/);
  Assert.equal(await Page.getByRole("button", { name: "Send", exact: true }).isDisabled(), true);
  Assert.equal(await Page.getByRole("button", { name: "Home", exact: true }).isDisabled(), true);
  Assert.equal(await Page.locator(".annunciator").filter({ hasText: "GUIDE" }).count(), 1);
  await Until(() => Sent("hand_guide_heartbeat").length >= 2, "the heartbeat", 3000);
  await Page.screenshot({ path: Path.join(Output, "HandGuideUi.png") });
  await Hold.click();
  await Until(() => Sent("hand_guide").length === 2, "Hold here");
  Assert.deepEqual(Sent("hand_guide")[1], { type: "hand_guide", enable: false });
  // AI mode while guiding: Hold here first
  await Page.getByRole("button", { name: "AI control mode", exact: true }).click();
  await Until(() => Sent("hand_guide").length === 3, "Hold here before AI mode");
  Assert.deepEqual(Sent("hand_guide")[2], { type: "hand_guide", enable: false });
  State.hand_guide = { ...State.hand_guide, on: false, phase: "", last_exit: { kind: "preloaded", reason: "Hold here: hand guiding ended, holding where it is", at_s: 5, mode: "guide" } };
  await Page.getByRole("button", { name: "Emergency stop", exact: true }).waitFor();
  Assert.equal((await Page.getByRole("button", { name: "Coast", exact: true }).textContent()).trim(), "Coast");
  Assert.equal(await Page.getByRole("button", { name: "Coast", exact: true }).getAttribute("title"), null);
  // A second press in this session does not ask again
  await Page.getByRole("button", { name: "Manual control mode", exact: true }).click();
  const Asked = Dialogs.length;
  await Page.getByRole("button", { name: "Hand guide", exact: true }).click();
  await Until(() => Sent("hand_guide").length === 4, "a second start");
  Assert.equal(Dialogs.length, Asked, "the first-use confirmation comes once a session");
  Assert.deepEqual(Errors, []);
  console.log(`PASS hand guiding in the page: ${Commands.length} commands, ${Dialogs.length} confirmations`);
} finally {
  for (const Timer of Timers) clearInterval(Timer);
  await Browser.close(); await Server.Close();
}
