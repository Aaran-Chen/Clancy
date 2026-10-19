import { chromium as Chromium } from "playwright";
import { LaunchOptions, StartUiServer } from "./UiTestLib.mjs";
import { readFileSync as ReadFile, mkdirSync as MakeDirectory } from "node:fs";
import { execFileSync as Execute } from "node:child_process";
import { fileURLToPath as FilePath } from "node:url";
import Path from "node:path";
import Assert from "node:assert/strict";

const Root = Path.dirname(FilePath(import.meta.url));
const Backend = Path.resolve(Root, "../back end");
const Python = ReadFile(Path.join(Root, "../.backend-python.txt"), "utf8").trim();
const State = JSON.parse(Execute(Python, ["-c", "import json, bridge_server; print(json.dumps(bridge_server.BridgeController().state_snapshot()))"], { cwd: Backend, encoding: "utf8" }));
State.motion_speed_percent = 100;
const Output = Path.join(Backend, "TestArtifacts"); MakeDirectory(Output, { recursive: true });
const Server = await StartUiServer();
const Browser = await Chromium.launch(LaunchOptions());
const Page = await Browser.newPage({ viewport: { width: 1920, height: 1080 } });
const Commands = [], Errors = [], Timers = new Set();
let ApplyChanges = true, LatestSocket;
State.arm_connected = true; State.torque = true;
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
    Commands.push({ ...Command, ObservedSpeed: State.motion_speed_percent });
    if (Command.type === "set_motion_speed" && ApplyChanges) State.motion_speed_percent = Command.percent;
    Socket.send(JSON.stringify(State));
  });
});

await Page.route("**/src/App.tsx*", async Route => {
  const Response = await Route.fetch(), Original = await Response.text();
  const Modified = Original.replace("const sendCommand = ", `window.StageSpeedTest = (First, Last, Command) => {
    ChangeMotionSpeed(First); ChangeMotionSpeed(Last); if (Command) sendCommand(Command);
  }; const sendCommand = `);
  Assert.notEqual(Modified, Original); await Route.fulfill({ response: Response, body: Modified });
});
try {
  await Page.goto(Server.Url, { waitUntil: "domcontentloaded" });
  const Slider = Page.getByRole("slider", { name: "Motion speed", exact: true });
  await Page.waitForFunction(() => !document.querySelector('input[aria-label="Motion speed"]')?.disabled);
  // The desired value returns to the value in stale telemetry before its send timer.
  ApplyChanges = false;
  await Page.evaluate(() => window.StageSpeedTest(10, 100));
  await Page.waitForTimeout(200);
  Assert.deepEqual(Commands.map(Command => Command.percent), [10, 100], "Old 100% telemetry must not cancel the queued return to 100%");
  ApplyChanges = true; Commands.length = 0;
  State.motion_speed_percent = 100; LatestSocket.send(JSON.stringify(State));
  await Page.waitForTimeout(150);
  for (const Type of ["send_ik", "move_servo", "go_home"]) {
    Commands.length = 0;
    await Page.evaluate(Type => window.StageSpeedTest(99, 10, { type: Type }), Type);
    await Page.waitForTimeout(180);
    const Move = Commands.find(Command => Command.type === Type);
    Assert.equal(Move.ObservedSpeed, 10, "The latest speed must reach the bridge before " + Type);
    Assert.deepEqual(Commands.map(Command => Command.type), ["set_motion_speed", "set_motion_speed", Type]);
  }
  Commands.length = 0;
  await Page.evaluate(() => window.StageSpeedTest(99, 10, { type: "stop_motion" }));
  await Page.waitForTimeout(180);
  Assert.equal(Commands[1].type, "stop_motion", "Stop must not wait for or flush a speed update");
  Assert.deepEqual(Errors, []);
  console.log("PASS: rapid speed reversal survives old telemetry; latest speed precedes IK, joint and Home commands; Stop remains immediate. All hardware mocked.");
} finally { Timers.forEach(Timer => clearInterval(Timer)); await Browser.close(); await Server.Close(); }
