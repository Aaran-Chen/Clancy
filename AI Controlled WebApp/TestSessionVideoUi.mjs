// The session video records only while an AI task runs: what the real page tells the dev server
// (POST /session-log/recording, src/lib/SessionVideo.ts) in headless Edge, with the bridge, camera and model mocked
// in the browser. A task the AI finishes: "start", then "end" with the task_end reason. A task waiting for approval:
// "alive" while it waits (every 15 s; 0.4 s here), and "end" on Stop task. A step of the same task keeps its key; a
// new task gets a new one. A page reloaded mid-task says "end" on its way out. An automated browser sends nothing.
// Safety: the suite serves this checkout on 5191-5199 itself (UiTestLib.mjs, which answers no /session-log route;
// CadTestUrl picks a test server of your own, never a live port); Edge maps the live ports (5180/5181, 8000, 8001,
// 8002, 11434) to a closed port, every request to them is stubbed or refused in the browser, and the camera is fake.
//   node TestSessionVideoUi.mjs
import { chromium as Chromium } from "playwright";
import { CadReadyMs, LaunchOptions, StartUiServer, BackendPython } from "./UiTestLib.mjs";
import { execFileSync as Execute } from "node:child_process";
import Path from "node:path";
import Assert from "node:assert/strict";

const Root = import.meta.dirname, Backend = Path.resolve(Root, "../back end");
const Server = await StartUiServer();
const Base = Server.Url;
const LivePorts = [8000, 8001, 8002, 11434];
const Python = BackendPython();
const State = JSON.parse(Execute(Python, ["-c", "import json, bridge_server; print(json.dumps(bridge_server.BridgeController().state_snapshot()))"],
  { cwd: Backend, encoding: "utf8", env: { ...process.env, SO101_DEPTH_CAMERA: "0", SO101_RECORDER: "0" } }));
Object.assign(State, { arm_connected: true, torque: true, port: "FAKE", motion_epoch: 5, motion_route: null, motion_fault: null });
for (const Servo of Object.values(State.servos)) Servo.moving = false;

const HostRules = LivePorts.flatMap(P => ["127.0.0.1", "localhost"].map(Host => `MAP ${Host}:${P} 127.0.0.1:9`)).join(", ");
const Browser = await Chromium.launch(LaunchOptions({
  args: [`--host-resolver-rules=${HostRules}`, "--use-fake-device-for-media-stream", "--use-fake-ui-for-media-stream"] }));
const Pause = Milliseconds => new Promise(Resolve => setTimeout(Resolve, Milliseconds));
const Live = new RegExp(`^(https?|wss?)://[^/]*:(${LivePorts.join("|")})(/|\\?|$)`);
const CameraCheck = { ok: true, angle_deg: 0.12, offset_cm: 0.08, reason: "", reference_fitted_at: "2026-09-28T21:10:00", seq: null };
const Measured = { ok: true, rms_cm: 0.22, camera_check: CameraCheck, withheld: [],
  objects: [{ label: "cup", x: 0.2, y: 40.8, z: -2.3, top_z: 6.7, height_cm: 9, size_cm: [7, 7], method: "depth", task_match: false, task_query_only: false }] };

// One browser profile with the bridge, Ollama and the dev server's routes faked; Human: navigator.webdriver hidden,
// as the AI task suites do (the record and the recording leave automated browsers out).
async function Device(Human) {
  const Context = await Browser.newContext({ viewport: { width: 1600, height: 1000 }, permissions: ["camera"], serviceWorkers: "block" });
  const D = { Context, Recording: [], Events: [], Replies: [], Blocked: [], Errors: [], Timers: new Set() };
  if (Human) await Context.addInitScript(() => Object.defineProperty(Navigator.prototype, "webdriver", { get: () => false }));
  await Context.route(Live, async Route => {
    const Url = new URL(Route.request().url());
    if (Url.port !== "8000") { D.Blocked.push(Url.href); return Route.abort("blockedbyclient"); }
    if (Url.pathname === "/ports") return Route.fulfill({ json: [] });
    if (Url.pathname === "/ik/preview") return Route.fulfill({ json: { ok: false } });
    if (Url.pathname === "/locate") return Route.fulfill({ json: Measured });
    return Route.fulfill({ json: { ok: true, calibrated: false, points: [], point_count: 0 } });
  });
  await Context.routeWebSocket(Live, Socket => {
    if (!/:8000\/ws/.test(Socket.url())) { D.Blocked.push(`WS ${Socket.url()}`); Socket.close({ code: 1008, reason: "test firewall" }); return; }
    Socket.send(JSON.stringify(State));
    const Timer = setInterval(() => Socket.send(JSON.stringify(State)), 100);
    D.Timers.add(Timer);
    Socket.onClose(() => { clearInterval(Timer); D.Timers.delete(Timer); });
    Socket.onMessage(Message => {
      const Command = JSON.parse(String(Message));
      Socket.send(JSON.stringify({ type: "status", ok: true, text: Command.type === "set_motion_speed" ? `Motion speed set to ${Command.percent}%` : "OK" }));
    });
  });
  await Context.route("**/session-log/recording", Route => { D.Recording.push({ ...Route.request().postDataJSON(), At: Date.now() }); return Route.fulfill({ json: { ok: true } }); });
  await Context.route("**/session-log/event", Route => { D.Events.push(Route.request().postDataJSON()); return Route.fulfill({ json: { ok: true } }); });
  await Context.route(/\/(bridge-launch|depth-server)\//, Route => Route.fulfill({ json: { ok: true } }));
  await Context.route("**/ollama/**", async Route => {
    const Url = new URL(Route.request().url());
    if (Url.pathname.endsWith("/tags")) return Route.fulfill({ json: { models: [{ name: "VideoTest", size: 1000 }] } });
    if (Url.pathname.endsWith("/show")) return Route.fulfill({ json: { capabilities: ["vision"] } });
    if (Url.pathname.endsWith("/ps")) return Route.fulfill({ json: { models: [{ name: "VideoTest", context_length: 32768 }] } });
    if (!Url.pathname.endsWith("/api/chat")) return Route.fulfill({ json: { data: [{ id: "VideoTest" }] } });
    const Body = Route.request().postDataJSON();
    if (Body.messages.length === 0) return Route.fulfill({ json: { model: Body.model, message: { role: "assistant", content: "" }, done: true, done_reason: "load" } });
    const Reply = D.Replies.shift() ?? { response_to_user: "Nothing to do.", task_status: "done", motion_proposal: null };
    return Route.fulfill({ json: { model: Body.model, message: { role: "assistant", content: JSON.stringify(Reply) }, done: true, done_reason: "stop",
      prompt_eval_count: 10, eval_count: 5 } }).catch(() => {});
  });
  await Context.route("**/LocalDisplayPreferences.json", Route => Route.fulfill({ json: {} }));
  // Alive every 0.4 s instead of every 15 s, so the test need not wait.
  await Context.route("**/src/lib/SessionVideo.ts*", async Route => {
    const Response = await Route.fetch(), Original = await Response.text();
    const Modified = Original.replace(/export const TaskAliveMs = (?:15_000|15000|15e3);/, "export const TaskAliveMs = 400;");
    Assert.ok(Modified.includes("TaskAliveMs = 400;"), "the alive interval was found");
    await Route.fulfill({ response: Response, body: Modified });
  });
  D.Page = await Context.newPage();
  D.Page.on("pageerror", Failure => D.Errors.push(Failure.message));
  return D;
}

async function WaitFor(Condition, Message, TimeoutMs = 30000) {
  const Started = Date.now();
  while (!(await Condition())) {
    if (Date.now() - Started > TimeoutMs) throw new Error(`Timed out: ${Message}`);
    await Pause(50);
  }
}

async function Open(D) {
  await D.Page.goto(new URL("/", Base).href, { waitUntil: "domcontentloaded" });
  await D.Page.locator("canvas[data-cad-ready=true][data-assembly-ready=true]").waitFor({ timeout: CadReadyMs });
  await D.Page.getByRole("button", { name: "AI control mode", exact: true }).click();
  await D.Page.waitForFunction(() => document.querySelector("video")?.dataset.captureReady === "true", null, { timeout: 30000 });
  await D.Page.locator('.ai-companion[data-state="ready"]').waitFor({ timeout: 30000 });
}

async function Submit(D, Text) {
  await D.Page.getByPlaceholder("Describe a task or movement").fill(Text);
  await D.Page.getByRole("button", { name: "Send to AI", exact: true }).click();
}

const Signals = D => D.Recording.map(Item => [Item.state, Item.task_id]);
let Failed = false;
const Human = await Device(true);
try {
  await Open(Human);
  Assert.deepEqual(Human.Recording, [], "nothing before a task");

  // A task the AI finishes on its first reply.
  Human.Replies.push({ response_to_user: "A cup on the desk.", task_status: "done", motion_proposal: null });
  await Submit(Human, "Describe the workspace; do not move.");
  await WaitFor(async () => Human.Recording.some(Item => Item.state === "end"), "the first task's end");
  const [Start, ...Rest] = Human.Recording;
  Assert.equal(Start.state, "start");
  Assert.equal(Start.task, "Describe the workspace; do not move.");
  Assert.match(Start.task_id, /^[a-z0-9]+-[a-z0-9]{8}$/);
  const End = Rest.find(Item => Item.state === "end");
  Assert.equal(End.task_id, Start.task_id);
  Assert.equal(End.reason, "the AI reported the task done.", "the task_end record's reason");
  const TaskEnd = Human.Events.find(Event => Event.type === "task_end");
  Assert.equal(TaskEnd.reason, End.reason);
  Assert.ok(Rest.every(Item => Item.task_id === Start.task_id));

  // A task waiting for approval: alive while it waits, then Stop task ends it.
  const Before = Human.Recording.length;
  Human.Replies.push({ response_to_user: "Raising first.", task_status: "continue", motion_proposal: { move_delta_cm: { left: 0, farther: 0, up: 2 } } });
  await Submit(Human, "Lift the gripper 2 cm");
  await Human.Page.getByRole("button", { name: "Approve", exact: true }).waitFor({ timeout: 30000 });
  await Pause(1500);
  const Second = Human.Recording.slice(Before);
  Assert.equal(Second[0].state, "start");
  Assert.notEqual(Second[0].task_id, Start.task_id, "a new task, a new key");
  const Alive = Second.filter(Item => Item.state === "alive");
  Assert.ok(Alive.length >= 2, `alive while waiting for approval (${Alive.length} in 1.5 s at 0.4 s)`);
  Assert.ok(Alive.every(Item => Item.task_id === Second[0].task_id));
  await Human.Page.getByRole("button", { name: "Stop task", exact: true }).click();
  await WaitFor(async () => Human.Recording.slice(Before).some(Item => Item.state === "end"), "Stop task ends the recording");
  Assert.deepEqual(Human.Recording.at(-1), { state: "end", task_id: Second[0].task_id, reason: "Stop task was pressed.", At: Human.Recording.at(-1).At });
  const AfterStop = Human.Recording.length;
  await Pause(1200);
  Assert.equal(Human.Recording.length, AfterStop, "no alive after the end");

  // A page reloaded mid-task says so on its way out.
  Human.Replies.push({ response_to_user: "Raising first.", task_status: "continue", motion_proposal: { move_delta_cm: { left: 0, farther: 0, up: 2 } } });
  await Submit(Human, "Lift the gripper 2 cm again");
  await Human.Page.getByRole("button", { name: "Approve", exact: true }).waitFor({ timeout: 30000 });
  const Third = Human.Recording.at(-1).task_id;
  await Human.Page.reload({ waitUntil: "domcontentloaded" });
  await WaitFor(async () => Human.Recording.some(Item => Item.state === "end" && Item.task_id === Third), "the reload's end", 10000);
  Assert.equal(Human.Recording.find(Item => Item.state === "end" && Item.task_id === Third).reason, "the page was closed or reloaded");
  Assert.deepEqual(Human.Errors, [], "no page errors");
  Assert.deepEqual(Human.Blocked, [], "nothing reached a live port");
  console.log(`PASS human: ${Signals(Human).length} signals: ${[...new Set(Signals(Human).map(Item => Item[0]))].join(", ")}`);

  // An automated browser (navigator.webdriver) sends nothing, as it records nothing in the session record.
  const Robot = await Device(false);
  try {
    await Open(Robot);
    Robot.Replies.push({ response_to_user: "A cup on the desk.", task_status: "done", motion_proposal: null });
    await Submit(Robot, "Describe the workspace; do not move.");
    await Robot.Page.locator(".ai-reply").filter({ hasText: "A cup on the desk." }).waitFor({ timeout: 30000 });
    await Pause(500);
    Assert.deepEqual(Robot.Recording, []);
    Assert.deepEqual(Robot.Blocked, []);
    console.log("PASS automated browser: no recording signals");
  } finally {
    for (const Timer of Robot.Timers) clearInterval(Timer);
    await Robot.Context.close();
  }
  console.log("PASS TestSessionVideoUi");
} catch (Failure) {
  Failed = true;
  console.error(Failure);
  console.error("Recording signals:", JSON.stringify(Human.Recording.map(({ At, ...Item }) => Item)));
} finally {
  for (const Timer of Human.Timers) clearInterval(Timer);
  await Browser.close(); await Server.Close();
}
process.exit(Failed ? 1 : 0);
