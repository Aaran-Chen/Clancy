import { chromium as Chromium } from "playwright";
import { CadReadyMs, LaunchOptions, StartUiServer, BackendPython } from "./UiTestLib.mjs";
import { mkdirSync as MakeDirectory } from "node:fs";
import { execFileSync as Execute } from "node:child_process";
import { fileURLToPath as FilePath } from "node:url";
import Path from "node:path";
import Assert from "node:assert/strict";

// 2026-09-30 05:24-05:31: six tasks in a row for the black servo on the tape
// roll ended in a question. This runs the recorded frame through the page
// against a mocked bridge, camera and model: the phrase "black servo motor
// thing" binds the "small black object" no label named a motor, and the
// user's answer to the model's question goes on with the asked task instead of
// starting an object-less one. No request reaches the real bridge or arm.
// It serves this checkout on 5191-5199 itself (UiTestLib.mjs); CadTestUrl picks a test server of your own.
const Root = Path.dirname(FilePath(import.meta.url)), Backend = Path.resolve(Root, "../back end");
const Python = BackendPython();
const State = JSON.parse(Execute(Python, ["-c", "import json, bridge_server; print(json.dumps(bridge_server.BridgeController().state_snapshot()))"], { cwd: Backend, encoding: "utf8" }));
Object.assign(State, { arm_connected: true, torque: true, port: "FAKE", motion_epoch: 5, motion_route: null, motion_fault: null });
for (const Servo of Object.values(State.servos)) Servo.moving = false;
const Output = Path.join(Backend, "TestArtifacts"); MakeDirectory(Output, { recursive: true });
const Server = await StartUiServer();
const Browser = await Chromium.launch(LaunchOptions({ args: ["--use-fake-device-for-media-stream", "--use-fake-ui-for-media-stream"] }));
const Page = await Browser.newPage({ viewport: { width: 1600, height: 1000 }, permissions: ["camera"] });
const Errors = [], Completions = [], Events = [], Timers = new Set();
const Pause = Milliseconds => new Promise(Resolve => setTimeout(Resolve, Milliseconds));
Page.on("pageerror", Error => Errors.push(Error.message));
// The session record is skipped for automated browsers; show it to this test.
await Page.addInitScript(() => Object.defineProperty(Navigator.prototype, "webdriver", { get: () => false }));
await Page.route("**/session-log/**", Route => { Events.push({ ...Route.request().postDataJSON(), At: Date.now() }); return Route.fulfill({ json: { ok: true } }); });
await Page.route("**/bridge-launch/**", Route => Route.fulfill({ json: { ok: true } }));
await Page.route("**/depth-server/**", Route => Route.fulfill({ json: {} }));

// The model: each scenario queues replies.
let Replies = [];
const Preloads = [];
const LastUserText = Body => Body.messages.filter(Message => Message.role === "user" && Message.content).at(-1).content;
await Page.route("**/ollama/**", async Route => {
  const Url = new URL(Route.request().url());
  if (Url.pathname.endsWith("/tags")) return Route.fulfill({ json: { models: [{ name: "AnswerTest", size: 1000 }] } });
  if (Url.pathname.endsWith("/show")) return Route.fulfill({ json: { capabilities: ["vision"] } });
  if (Url.pathname.endsWith("/ps")) return Route.fulfill({ json: { models: [{ name: "AnswerTest", context_length: 32768 }] } });
  if (!Url.pathname.endsWith("/api/chat")) return Route.fulfill({ json: { data: [{ id: "AnswerTest" }] } });
  const Body = Route.request().postDataJSON();
  if (Body.messages.length === 0) {
    Preloads.push(Body);
    return Route.fulfill({ json: { model: Body.model, message: { role: "assistant", content: "" }, done: true, done_reason: "load" } });
  }
  Completions.push({ At: Date.now(), Body, Text: LastUserText(Body) });
  const Reply = Replies.shift() ?? { response_to_user: "Nothing to do.", task_status: "done", motion_proposal: null };
  return Route.fulfill({ json: { model: Body.model, message: { role: "assistant", content: JSON.stringify(Reply) }, done: true, done_reason: "stop",
    prompt_eval_count: 10, eval_count: 5 } }).catch(() => {});
});

// The bridge: /locate answers the scenario's frame; nothing moves.
const CameraCheck = { ok: true, angle_deg: 0.065, offset_cm: 0.004, reason: "desk matches the calibration", reference_fitted_at: "2026-09-30T16:48:14", seq: 83992 };
const Row = ([label, x, y, z, top_z, size_cm, extrapolated, support_branch, refined, valid_fraction, outside_calibration_cm]) => ({ label, x, y, z, top_z,
  height_cm: Number((top_z - z).toFixed(2)), size_cm, method: "depth", extrapolated, support_branch, support_measured: true, support_z_measured: z, refined,
  task_match: false, task_query_only: false, valid_fraction, outside_calibration_cm });
// The recorded 05:24:55 frame (back end/captures/2026-09-30/recording/ai.jsonl line 206).
const F0524 = { ok: true, rms_cm: 0.334, camera_check: CameraCheck, withheld: [{ label: "camera", reason: "the robot's own parts are not objects" }],
  objects: [["can", 19.47, 41.31, -1.61, 10.12, [5.6, 5.4], true, "desk", false, 0.918, 13.9],
    ["bottle", 11.13, 31.63, -2.28, 5.58, [3.5, 3], false, "desk", true, 0.876, 1.2],
    ["tape", 0.75, 30.71, -2.38, 2.54, [7.9, 4.7], false, "desk", false, 0.988, 0],
    ["keyboard", 3.07, 61.09, 0.12, 1.28, [9.7, 21.7], true, "desk", false, 1, 24.8],
    ["mouse", 12.82, 107.5, 3.86, 10.08, [10.9, 9.8], true, "desk", false, 0.999, 72.2],
    ["cup", -12.66, 31.86, -2.02, -0.19, [7.8, 8.5], false, "desk", true, 0.562, 1.2],
    ["pen", 16, 60.28, 0.17, 2.08, [2, 4.5], true, "desk", false, 0.999, 28],
    ["tape roll", 0.88, 29.84, -2.39, 2.05, [7.9, 4.6], false, "desk", false, 0.997, 0],
    ["pliers", 3.08, 7.44, -8.21, -5.51, [18.6, 8.2], true, "bin", false, 0.862, 16.2],
    ["small black object", 1.01, 30.84, -2.18, 2.88, [4.5, 2.4], false, "desk", false, 0.963, 0]].map(Row) };
// A small black object the gripper can grasp (the safety suite's eraser: 2 cm across the jaws), unnamed, beside a cup.
const Graspable = { ok: true, rms_cm: 0.22, camera_check: CameraCheck, withheld: [],
  objects: [["small black object", -5, 35, -2.3, -0.3, [2, 5], false, "desk", false, 0.95, 0], ["cup", 0.2, 40.8, -2.3, 6.7, [7, 7], false, "desk", false, 0.95, 0]].map(Row) };
let Locate = F0524;
const LocateBodies = [];
await Page.route("**:8000/**", async Route => {
  const Url = new URL(Route.request().url());
  if (Url.pathname === "/ports") return Route.fulfill({ json: [] });
  if (Url.pathname === "/ik/preview") return Route.fulfill({ json: { ok: false } });
  if (Url.pathname === "/locate") {
    LocateBodies.push(Route.request().postDataJSON());
    return Route.fulfill({ json: Locate }).catch(() => {});
  }
  return Route.fulfill({ json: { ok: true, calibrated: false, points: [], point_count: 0 } });
});
await Page.routeWebSocket("**/ws", Connected => {
  Connected.send(JSON.stringify(State));
  const Timer = setInterval(() => Connected.send(JSON.stringify(State)), 100); Timers.add(Timer);
  Connected.onClose(() => { clearInterval(Timer); Timers.delete(Timer); });
  Connected.onMessage(Message => {
    const Command = JSON.parse(String(Message));
    Connected.send(JSON.stringify({ type: "status", ok: true, text: Command.type === "set_motion_speed" ? `Motion speed set to ${Command.percent}%` : "OK" }));
  });
});

const Reply = Page.locator(".ai-reply");
const ReplyText = async () => (await Reply.count()) ? await Reply.innerText() : "";
const Submit = async Text => {
  await Page.getByPlaceholder("Describe a task or movement").fill(Text);
  await Page.getByRole("button", { name: "Send to AI", exact: true }).click();
};
const WaitFor = async (Condition, Message, TimeoutMs = 30000) => {
  const Started = Date.now();
  while (!(await Condition())) {
    if (Date.now() - Started > TimeoutMs) throw new Error(`Timed out: ${Message}\nAI panel: ${await Page.locator(".ai-panel").innerText().catch(() => "")}`);
    await Pause(50);
  }
};
const TaskEnds = Since => Events.filter(Event => Event.type === "task_end" && Event.At >= Since);
try {
  await Page.goto(Server.Url, { waitUntil: "domcontentloaded" });
  await Page.locator("canvas[data-cad-ready=true][data-assembly-ready=true]").waitFor({ timeout: CadReadyMs });
  await Page.getByRole("button", { name: "AI control mode", exact: true }).click();
  await Page.waitForFunction(() => document.querySelector("video")?.dataset.captureReady === "true");
  await Page.locator('.ai-companion[data-state="ready"]').waitFor();
  await WaitFor(async () => Preloads.length === 1, "the model warm-up");

  // 05:24:55: the phrase reaches /locate and binds the "small black object" (its colour; no label names a motor).
  // The model used to be told it was NOT detected and [UNRELIABLE], and asked the user to confirm. Lying 4.5 cm across
  // the jaws it used to stop the task here, before any model call (not graspable); since 2026-10-04 the app tries a
  // grasp it is not sure of (GraspRules.BestEffort), so the model is told the bound object and the BEST-EFFORT GRASP.
  let Since = Date.now();
  Replies.push({ response_to_user: "Is the black servo motor the small black object on the tape roll?", task_status: "ask", motion_proposal: null });
  await Submit("grab the black servo motor thing");
  await WaitFor(async () => TaskEnds(Since).some(Event => Event.source === "model_ask"), "the servo is bound and the model asks");
  Assert.deepEqual(LocateBodies.at(-1).task_object, { phrase: "black servo motor thing", label: "black servo motor thing" });
  Assert.equal(Completions.length, 1, "One model call: the app does not stop a grasp it is only unsure of");
  Assert.match(Completions[0].Text, /TASK OBJECT "black servo motor thing": detected this check-in \(measured as "small black object", the only label with its colour; no measured label names a "motor": check in the image that it is the black servo motor thing, else reply ask\) at X 1\.0, Y 30\.8/);
  Assert.match(Completions[0].Text, /BEST-EFFORT GRASP: the app is not sure this grasp holds the black servo motor thing \(the grip may slip/);
  Assert.ok(Events.some(Event => Event.type === "task_object_bound" && Event.measured_label === "small black object" && Event.source === "colour" && Event.At >= Since));
  await Page.screenshot({ path: Path.join(Output, "AiTaskServoBound.png") });

  // 2026-10-01 07:56: a go-ahead after the app's own stop ("ok i did that") goes on with the stopped task, its
  // object and why it stopped; it used to start an object-less task and the model went for the tape roll. The app
  // stops it here before any model call because the servo is out of the right arm's reach; the user moves it closer.
  const Far = { ...F0524, objects: F0524.objects.map(Object_ => Object_.label === "small black object" ? { ...Object_, x: 16.1, y: 35.8 } : Object_) };
  Locate = Far;
  Since = Date.now();
  await Submit("grab the black servo motor thing");
  await WaitFor(async () => TaskEnds(Since).some(Event => Event.source === "app"), "the app stops the task: out of reach");
  Assert.match(await ReplyText(), /out of the right arm's reach \(46\.3 cm from its shoulder\)/);
  Assert.equal(Completions.length, 1, "Stopped by the app's own check, before any model call");
  Locate = Graspable;
  Replies.push({ response_to_user: "Going to it.", task_status: "done", motion_proposal: null });
  Since = Date.now();
  await Submit("ok i did that");
  await WaitFor(async () => Completions.length > 0 && Completions.at(-1).At >= Since, "the go-ahead reaches the model");
  Assert.match(Completions.at(-1).Text, /^grab the black servo motor thing \(the app stopped it: "[^"]*out of the right arm's reach[^"]*"; the user answered: "ok i did that"\)/);
  Assert.match(Completions.at(-1).Text, /TASK OBJECT "black servo motor thing": detected this check-in/);
  Assert.deepEqual(LocateBodies.at(-1).task_object, { phrase: "black servo motor thing", label: "black servo motor thing" });
  Assert.ok(Events.some(Event => Event.type === "prompt" && /the app stopped it: .*the user answered: "ok i did that"/.test(Event.prompt) && Event.At >= Since));
  // A task with no object is told not to pick one: a go-ahead with nothing stopped is such a task.
  Replies.push({ response_to_user: "Which object?", task_status: "ask", motion_proposal: null });
  Since = Date.now();
  await Submit("ok i did that");
  await WaitFor(async () => TaskEnds(Since).some(Event => Event.source === "model_ask"), "the second go-ahead is asked about");
  Assert.match(Completions.at(-1).Text, /^ok i did that\n/);
  Assert.match(Completions.at(-1).Text, /\nTASK OBJECT: none: the task names no object\. Do not choose an object/);
  // "try again" with the object out of sight (under the gripper, after a stop mid-grasp): the task goes on from
  // where the stopped one last measured it, instead of "NOT detected in any check-in of this task".
  Locate = Far;
  Since = Date.now();
  await Submit("grab the black servo motor thing");
  await WaitFor(async () => TaskEnds(Since).some(Event => Event.source === "app"), "the app stops it again");
  Locate = { ...F0524, objects: F0524.objects.filter(Object_ => Object_.label !== "small black object") };
  Replies.push({ response_to_user: "I cannot see it now.", task_status: "done", motion_proposal: null });
  Since = Date.now();
  await Submit("try again");
  await WaitFor(async () => TaskEnds(Since).some(Event => Event.source === "model_done"), "the go-ahead reaches the model");
  Assert.match(Completions.at(-1).Text, /^grab the black servo motor thing \(the app stopped it: [^\n]*; the user answered: "try again"\)/);
  Assert.match(Completions.at(-1).Text, /TASK OBJECT "black servo motor thing": NOT detected this check-in \(possibly under the gripper or out of view\); last measured at [\d:]+ at X 16\.1, Y 35\.8/);

  // No colour in the phrase: the labels that could be it are offered; the model asks which one.
  Locate = Graspable;
  const Question = "Is the servo motor the small black object at X -5.0, Y 35.0?";
  Replies.push({ response_to_user: Question, task_status: "ask", motion_proposal: null });
  Since = Date.now();
  await Submit("grab the servo motor");
  await WaitFor(async () => TaskEnds(Since).some(Event => Event.source === "model_ask"), "the model asks");
  Assert.match(Completions.at(-1).Text, /TASK OBJECT "servo motor": no measured label names it\. Candidates measured this check-in: [^\n]*small black object \(X -5\.0, Y 35\.0\)/);

  // 05:25:28: the answer goes on with the asked task: the question, the answer and the object it describes.
  Replies.push({ response_to_user: "Going to it.", task_status: "done", motion_proposal: null });
  Since = Date.now();
  await Submit("yes, its the small black object on the desk");
  await WaitFor(async () => Completions.at(-1).At >= Since, "the answer reaches the model");
  const Answer = Completions.at(-1).Text;
  Assert.match(Answer, new RegExp(`^grab the servo motor \\(you asked: "${Question.replace(/[.?]/g, Match => `\\${Match}`)}"; the user answered: "yes, its the small black object on the desk"\\)`));
  Assert.match(Answer, /TASK OBJECT "small black thing": detected this check-in \(measured as "small black object", the only label with its colour\) at X -5\.0, Y 35\.0/);
  Assert.match(Answer, /GRASP PLAN/);
  Assert.deepEqual(LocateBodies.at(-1).task_object, { phrase: "small black thing on the desk", label: "small black thing" });
  Assert.ok(Events.some(Event => Event.type === "prompt" && /the user answered: "yes, its the small black object on the desk"/.test(Event.prompt) && Event.At >= Since));

  // "yes grab that" keeps the asked object; a task of its own after a question is just that task.
  Replies.push({ response_to_user: "Which one, the small black object?", task_status: "ask", motion_proposal: null },
    { response_to_user: "Going to it.", task_status: "done", motion_proposal: null });
  await Submit("grab the servo motor");
  await WaitFor(async () => /Which one, the small black object\?/.test(await ReplyText()), "the second question");
  Since = Date.now();
  await Submit("yes grab that");
  await WaitFor(async () => Completions.at(-1).At >= Since, "the confirmation reaches the model");
  Assert.match(Completions.at(-1).Text, /^grab the servo motor \(you asked: "Which one, the small black object\?"; the user answered: "yes grab that"\)/);
  Assert.match(Completions.at(-1).Text, /TASK OBJECT "servo motor": /);
  Replies.push({ response_to_user: "A desk.", task_status: "done", motion_proposal: null });
  Since = Date.now();
  await Submit("Describe the workspace; do not move.");
  await WaitFor(async () => Completions.at(-1).At >= Since, "the description reaches the model");
  Assert.match(Completions.at(-1).Text, /^Describe the workspace; do not move\./);
  Assert.doesNotMatch(Completions.at(-1).Text, /you asked|^TASK OBJECT "/m);

  Assert.deepEqual(Errors, []);
  console.log("PASS: the recorded servo frame binds the small black object and the model is told its best-effort grasp; a go-ahead after the app's own stop (out of reach) goes on with the stopped task; an object-less task is told to ask; an answer to the model's question goes on with the asked task, the object it describes and the question; a confirmation keeps the asked object; a new task after a question stands alone.");
} finally {
  for (const Timer of Timers) { clearInterval(Timer); clearTimeout(Timer); }
  await Browser.close(); await Server.Close();
}
