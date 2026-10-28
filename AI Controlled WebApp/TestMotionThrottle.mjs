import { chromium as Chromium } from "playwright";
import { LaunchOptions, StartUiServer, BackendPython } from "./UiTestLib.mjs";
import { mkdirSync as MakeDirectory } from "node:fs";
import { execFileSync as Execute } from "node:child_process";
import { fileURLToPath as FilePath } from "node:url";
import Path from "node:path";
import Assert from "node:assert/strict";

const Root = Path.dirname(FilePath(import.meta.url));
const Backend = Path.resolve(Root, "../back end");
const Python = BackendPython();
const State = JSON.parse(Execute(Python, ["-c", "import json, bridge_server; print(json.dumps(bridge_server.BridgeController().state_snapshot()))"], { cwd: Backend, encoding: "utf8" }));
State.motion_speed_percent = 100; // Explicit fixture, independent of the bridge's 18% startup default.
const Output = Path.join(Backend, "TestArtifacts"); MakeDirectory(Output, { recursive: true });
const Server = await StartUiServer();
const Browser = await Chromium.launch(LaunchOptions());
const Page = await Browser.newPage({ viewport: { width: 1920, height: 1080 } });
const Commands = [], Errors = [], Timers = new Set();
let ApplyChanges = true, LatestSocket;
Page.on("pageerror", Error => Errors.push(Error.message));
await Page.route("**/bridge-launch/**", Route => Route.fulfill({ json: { ok: true } }));
await Page.route("**/ollama/**", Route => Route.fulfill({ json: { models: [] } }));
await Page.route("**/depth-server/**", Route => Route.fulfill({ json: {} }));
await Page.route("**:8000/**", Route => {
  const Url = new URL(Route.request().url());
  if (Url.pathname === "/ports") return Route.fulfill({ json: [] });
  if (Url.pathname === "/ik/preview") return Route.fulfill({ json: { ok: false } });
  Assert.equal(Route.request().method(), "GET", "Throttle must not make unrelated HTTP writes");
  return Route.fulfill({ json: { ok: true, calibrated: false, points: [], point_count: 0 } });
});
await Page.routeWebSocket("**/ws", Socket => {
  LatestSocket = Socket;
  Socket.send(JSON.stringify(State));
  const Timer = setInterval(() => Socket.send(JSON.stringify(State)), 33);
  Timers.add(Timer);
  Socket.onClose(() => { clearInterval(Timer); Timers.delete(Timer); });
  Socket.onMessage(Message => {
    const Command = JSON.parse(String(Message));
    Commands.push(Command);
    if (Command.type === "set_motion_speed" && ApplyChanges) State.motion_speed_percent = Command.percent;
    Socket.send(JSON.stringify(State));
  });
});
// Exercise the calibration command gate and timeout consumer without starting
// camera capture or any calibration motion. All bridge traffic is intercepted.
await Page.route("**/src/App.tsx*", async Route => {
  const Response = await Route.fetch(), Original = await Response.text();
  const Marker = "const MotionTimeoutRef = useRef(SELFCAL_SETTLE_TIMEOUT_MS);";
  Assert.ok(Original.includes(Marker));
  const Instrumented = Original.replace(Marker, `${Marker}
    window.SetThrottleCalibrationForTest = Value => { selfCalRunningRef.current = Value; };
    window.ReadThrottleTimeoutForTest = () => MotionTimeoutRef.current;
    window.ReadThrottleAiWaitForTest = () => Boolean(aiSettleWait);
    window.StartThrottleAiWaitForTest = Elapsed => {
      const Task = { task: "Slow motion regression", step: 1 };
      aiTaskRef.current = Task; setAiTask(Task);
      // The bridge started this motion (its epoch advanced) Elapsed ms ago.
      setAiSettleWait({ approvedAt: Date.now() - Elapsed, epoch: LastMotionEpochRef.current - 1, startedAt: Date.now() - Elapsed });
    };`).replace("const submitAiPrompt =", `
      runAiRequestRef.current = Prompt => { (window.ThrottleAiCheckins ??= []).push(Prompt); };
      const submitAiPrompt =`);
  Assert.ok(Instrumented.includes("window.ThrottleAiCheckins"));
  await Route.fulfill({ response: Response, body: Instrumented });
});
try {
  await Page.goto(Server.Url, { waitUntil: "domcontentloaded" });
  const Slider = Page.getByRole("slider", { name: "Motion speed", exact: true });
  await Slider.waitFor();
  await Page.waitForFunction(() => !document.querySelector('input[aria-label="Motion speed"]').disabled);
  Assert.equal(await Slider.inputValue(), "100");
  Assert.equal(Commands.length, 0, "Loading the page must not send a speed or motor command");
  await Slider.press("Home");
  await Page.waitForFunction(() => document.querySelector('.throttle-heading output').textContent === "010%");
  await Page.waitForTimeout(150);
  Assert.equal(State.motion_speed_percent, 10);
  await Slider.press("ArrowRight");
  await Page.waitForTimeout(150);
  Assert.equal(State.motion_speed_percent, 11);
  Assert.equal(await Slider.getAttribute("aria-valuetext"), "11% of maximum motion speed");
  await Slider.press("End");
  await Page.waitForTimeout(150);
  Assert.equal(State.motion_speed_percent, 100);

  // The same control remains enabled throughout an active, partially completed route.
  State.arm_connected = true; State.torque = true;
  State.motion_route = { stage: 1, stages: 3 };
  for (const Servo of Object.values(State.servos)) Servo.moving = true;
  LatestSocket.send(JSON.stringify(State));
  await Page.waitForTimeout(80);
  Assert.equal(await Slider.isEnabled(), true);
  await Slider.fill("30");
  await Page.waitForFunction(() => document.querySelector('input[aria-label="Motion speed"]').value === "30");
  await Page.waitForTimeout(150);
  Assert.equal(State.motion_speed_percent, 30, "30% is applied during motion");
  await Slider.fill("75"); await Page.waitForTimeout(150);
  Assert.equal(State.motion_speed_percent, 75, "The same moving route can speed up");
  await Slider.fill("30"); await Page.waitForTimeout(150);
  Assert.equal(State.motion_speed_percent, 30, "The same moving route can slow down");
  Assert.deepEqual(State.motion_route, { stage: 1, stages: 3 });
  State.motion_route = null;
  for (const Servo of Object.values(State.servos)) Servo.moving = false;
  await Slider.press("End"); await Page.waitForTimeout(150);

  // Dragging updates smoothly while old telemetry still reports the previous
  // setting, and coalesces a high input rate into bounded bridge traffic.
  ApplyChanges = false;
  const BeforeDrag = Commands.length;
  await Slider.scrollIntoViewIfNeeded();
  const Bounds = await Slider.boundingBox();
  await Page.mouse.move(Bounds.x + Bounds.width - 10, Bounds.y + Bounds.height / 2);
  await Page.mouse.down();
  await Page.mouse.move(Bounds.x + Bounds.width * .37, Bounds.y + Bounds.height / 2, { steps: 70 });
  await Page.mouse.up();
  const Desired = Number(await Slider.inputValue());
  Assert.ok(Desired > 10 && Desired < 100);
  await Page.waitForTimeout(250);
  Assert.equal(Number(await Slider.inputValue()), Desired, "Old telemetry must not jerk the slider back");
  Assert.ok(Commands.length - BeforeDrag < 35, "Rapid drag input is coalesced");
  Assert.equal(Commands.at(-1).percent, Desired, "The final drag value must be sent");
  State.motion_speed_percent = Desired;
  ApplyChanges = true;
  await Page.waitForTimeout(150);
  await Page.locator('.motion-throttle').screenshot({ path: Path.join(Output, "MotionThrottle.png") });
  await Page.screenshot({ path: Path.join(Output, "MotionThrottleUi.png") });

  // A missing acknowledgement reverts to the last hardware-reported setting.
  ApplyChanges = false;
  await Slider.press("Home");
  await Page.waitForTimeout(2300);
  Assert.equal(Number(await Slider.inputValue()), Desired);
  ApplyChanges = true;

  await Page.getByRole("button", { name: "Minimize Live Feed", exact: true }).click();
  await Page.getByRole("button", { name: "Settings", exact: true }).click();
  await Page.getByRole("button", { name: "Calibrate", exact: true }).click();
  const CalibrationSlider = Page.locator('.cal-panel').getByRole("slider", { name: "Motion speed", exact: true });
  await CalibrationSlider.waitFor();
  await Page.evaluate(() => window.SetThrottleCalibrationForTest(true));
  await CalibrationSlider.press("Home");
  State.motion_timeout_ms = 120000;
  await Page.waitForFunction(() => window.ReadThrottleTimeoutForTest() === 120000);
  Assert.equal(State.motion_speed_percent, 10, "Throttle is accepted while calibration is active");
  State.motion_timeout_ms = 60000;
  await Page.waitForFunction(() => window.ReadThrottleTimeoutForTest() === 60000);
  await Page.evaluate(() => window.SetThrottleCalibrationForTest(false));
  await Page.locator('.cal-header button').click();

  for (const Servo of Object.values(State.servos)) Servo.moving = true;
  await Page.waitForTimeout(100);
  await Page.evaluate(() => window.StartThrottleAiWaitForTest(20000));
  await Page.waitForTimeout(650);
  Assert.equal(await Page.evaluate(() => window.ReadThrottleAiWaitForTest()), true,
    "A slow move must not be called finished after the old 12-second timeout");
  Assert.equal(await Page.evaluate(() => window.ThrottleAiCheckins?.length ?? 0), 0);
  State.motion_timeout_ms = 12000;
  await Page.waitForFunction(() => !window.ReadThrottleAiWaitForTest());
  Assert.equal(await Page.evaluate(() => window.ThrottleAiCheckins?.length ?? 0), 0,
    "Timeout pauses the task without sending a false arrival check-in");
  for (const Servo of Object.values(State.servos)) Servo.moving = false;
  State.motion_timeout_ms = 120000;
  await Page.waitForTimeout(100);
  await Page.evaluate(() => window.StartThrottleAiWaitForTest(2000));
  await Page.waitForFunction(() => window.ThrottleAiCheckins?.length === 1);

  // Reopening the UI reads the bridge setting without overwriting it.
  State.motion_speed_percent = 63;
  const BeforeReload = Commands.length;
  await Page.reload({ waitUntil: "domcontentloaded" });
  await Page.waitForFunction(() => document.querySelector('input[aria-label="Motion speed"]')?.value === "63");
  Assert.equal(Commands.length, BeforeReload);
  delete State.speed_control_version;
  await Page.waitForFunction(() => document.querySelector('input[aria-label="Motion speed"]').disabled);
  Assert.match(await Page.locator('.throttle-help').innerText(), /Reload the bridge/);
  Assert.equal(await Page.locator('.throttle-heading output').innerText(), "---%");
  State.speed_control_version = "throttle-v2";
  await Page.waitForFunction(() => !document.querySelector('input[aria-label="Motion speed"]').disabled);
  for (const Timer of Timers) clearInterval(Timer);
  Timers.clear();
  LatestSocket.close({ code: 1000, reason: "Simulated disconnect" });
  await Page.waitForFunction(() => document.querySelector('input[aria-label="Motion speed"]').disabled);
  Assert.match(await Page.locator('.throttle-help').innerText(), /Connect the bridge/);
  Assert.ok(Commands.length > 0);
  Assert.ok(Commands.every(Command => Command.type === "set_motion_speed" && Number.isInteger(Command.percent) && Command.percent >= 10 && Command.percent <= 100), JSON.stringify(Commands));
  Assert.deepEqual(Errors, []);
  console.log("PASS: keyboard endpoints; continuous drag; bounded command rate; delayed and missing acknowledgements; calibration throttle and live timeout; AI waits for slow motion and pauses on timeout; server-owned reconnect value; incompatible and disconnected bridge; zero motor commands; no browser errors.");
} finally {
  for (const Timer of Timers) clearInterval(Timer);
  await Browser.close(); await Server.Close();
}
