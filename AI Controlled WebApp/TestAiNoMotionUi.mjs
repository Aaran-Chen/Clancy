import { chromium as Chromium } from "playwright";
import { CadReadyMs, LaunchOptions, StartUiServer, BackendPython } from "./UiTestLib.mjs";
import { mkdirSync as MakeDirectory } from "node:fs";
import { execFileSync as Execute } from "node:child_process";
import { fileURLToPath as FilePath } from "node:url";
import Path from "node:path";
import Assert from "node:assert/strict";

// 2026-10-25 02:08 (back end/captures/2026-10-25/recording/ai.jsonl): at check-in 2 of "the pill bottle thats already
// in the gripper should go in my hand" the model replied task_status "continue", "I'll move a bit closer and lower to
// release the bottle into your palm", and no motion: the app ended the task. Now it asks once, in the same check-in,
// for the motion it described; a second reply without one ends the task saying it asked twice. "done" and "ask"
// end it as before, and Stop task or Reset AI context during the re-prompt end it as usual. Everything is mocked
// (bridge, camera, model); no request reaches the real bridge or arm. It serves this checkout on 5191-5199 itself
// (UiTestLib.mjs); CadTestUrl picks a test server of your own.
const Root = Path.dirname(FilePath(import.meta.url)), Backend = Path.resolve(Root, "../back end");
const Python = BackendPython();
const State = JSON.parse(Execute(Python, ["-c", "import json, bridge_server; print(json.dumps(bridge_server.BridgeController().state_snapshot()))"], { cwd: Backend, encoding: "utf8" }));
Object.assign(State, { arm_connected: true, torque: true, port: "FAKE", motion_epoch: 5, motion_route: null, motion_fault: null });
for (const Servo of Object.values(State.servos)) Servo.moving = false;
const Output = Path.join(Backend, "TestArtifacts"); MakeDirectory(Output, { recursive: true });
const Server = await StartUiServer();
const Browser = await Chromium.launch(LaunchOptions({ args: ["--use-fake-device-for-media-stream", "--use-fake-ui-for-media-stream"] }));
const Page = await Browser.newPage({ viewport: { width: 1600, height: 1000 }, permissions: ["camera"] });
const Errors = [], Commands = [], Completions = [], Events = [], Timers = new Set(), Failed = [];
const Pause = Milliseconds => new Promise(Resolve => setTimeout(Resolve, Milliseconds));
Page.on("pageerror", Error => Errors.push(Error.message));
Page.on("requestfailed", Request => Failed.push({ Url: Request.url(), At: Date.now() }));
// The session record is skipped for automated browsers; show it to this test.
await Page.addInitScript(() => Object.defineProperty(Navigator.prototype, "webdriver", { get: () => false }));
await Page.route("**/session-log/**", Route => { Events.push({ ...Route.request().postDataJSON(), At: Date.now() }); return Route.fulfill({ json: { ok: true } }); });
await Page.route("**/bridge-launch/**", Route => Route.fulfill({ json: { ok: true } }));
await Page.route("**/depth-server/**", Route => Route.fulfill({ json: {} }));

// The model: each scenario queues replies; a reply held until the test releases it is a call still in flight.
let Replies = [];
const Preloads = [], Held = [];
const LastUserText = Body => Body.messages.filter(Message => Message.role === "user" && Message.content).at(-1).content;
await Page.route("**/ollama/**", async Route => {
  const Url = new URL(Route.request().url());
  if (Url.pathname.endsWith("/tags")) return Route.fulfill({ json: { models: [{ name: "NoMotionTest", size: 1000 }] } });
  if (Url.pathname.endsWith("/show")) return Route.fulfill({ json: { capabilities: ["vision"] } });
  if (Url.pathname.endsWith("/ps")) return Route.fulfill({ json: { models: [{ name: "NoMotionTest", context_length: 32768 }] } });
  if (!Url.pathname.endsWith("/api/chat")) return Route.fulfill({ json: { data: [{ id: "NoMotionTest" }] } });
  const Body = Route.request().postDataJSON();
  if (Body.messages.length === 0) {
    Preloads.push(Body);
    return Route.fulfill({ json: { model: Body.model, message: { role: "assistant", content: "" }, done: true, done_reason: "load" } });
  }
  const Text = LastUserText(Body);
  Completions.push({ At: Date.now(), Body, Text });
  const Reply = Replies.shift() ?? { response_to_user: "Nothing to do.", task_status: "done", motion_proposal: null };
  const Content = typeof Reply === "function" ? await Reply(Text, Body) : Reply;
  return Route.fulfill({ json: { model: Body.model, message: { role: "assistant", content: JSON.stringify(Content) }, done: true, done_reason: "stop",
    prompt_eval_count: 10, eval_count: 5 } }).catch(() => {});
});
const HoldReply = Content => () => new Promise(Resolve => Held.push(() => Resolve(Content)));
const ReleaseHeld = () => { while (Held.length) Held.shift()(); };

// The bridge: /locate measures an eraser and a cup; the websocket runs each move like the real bridge (one status per command).
const CameraCheck = { ok: true, angle_deg: 0.12, offset_cm: 0.08, reason: "", reference_fitted_at: "2026-09-28T21:10:00", seq: null };
const Measured = { ok: true, rms_cm: 0.22, camera_check: CameraCheck, withheld: [],
  objects: [{ label: "eraser", x: -5, y: 35, z: -2.3, top_z: -0.3, height_cm: 2, size_cm: [2, 5], method: "depth",
    support_measured: true, support_z_measured: -2.3, valid_fraction: 0.9, support_branch: "desk", task_match: false, task_query_only: false },
  { label: "cup", x: 0.2, y: 40.8, z: -2.3, top_z: 6.7, height_cm: 9, size_cm: [7, 7], method: "depth", task_match: false, task_query_only: false }] };
await Page.route("**:8000/**", async Route => {
  const Url = new URL(Route.request().url());
  if (Url.pathname === "/ports") return Route.fulfill({ json: [] });
  if (Url.pathname === "/ik/preview") return Route.fulfill({ json: { ok: false } });
  if (Url.pathname === "/locate") return Route.fulfill({ json: Measured }).catch(() => {});
  return Route.fulfill({ json: { ok: true, calibrated: false, points: [], point_count: 0 } });
});
let CommandQueue = Promise.resolve();
const RunCommand = async (Connected, Command) => {
  Commands.push({ ...Command, At: Date.now() });
  const Reply = (Ok, Text) => Connected.send(JSON.stringify({ type: "status", ok: Ok, text: Text }));
  if (!["send_ik", "move_servo", "go_home"].includes(Command.type)) return Reply(true, Command.type === "set_motion_speed" ? `Motion speed set to ${Command.percent}%` : "OK");
  if (State.motion_route) return Reply(false, "Wait for this motion to finish, or press Stop before another move");
  if (Command.type === "go_home") return Reply(true, "Moving home");
  await Pause(300);
  State.motion_epoch += 1; State.motion_route = { stage: 1, stages: 1 };
  if (Command.type === "send_ik") {
    State.target_cm = { x: Command.target.x, y: Command.target.y, z: Command.target.z }; State.wrist_pitch = Command.target.wrist_pitch;
    State.joint_positions = [...State.joint_positions.slice(0, -1), { x: -Command.target.x, y: Command.target.y, z: Command.target.z }];
  } else {
    Object.assign(State.servos[Command.name], { target_raw: Command.raw, present_raw: Command.raw });
  }
  for (const Servo of Object.values(State.servos)) Servo.moving = true;
  Reply(true, "Moving");
  Connected.send(JSON.stringify(State));
  Timers.add(setTimeout(() => {
    State.motion_route = null;
    for (const Servo of Object.values(State.servos)) Servo.moving = false;
  }, 700));
};
await Page.routeWebSocket("**/ws", Connected => {
  Connected.send(JSON.stringify(State));
  const Timer = setInterval(() => Connected.send(JSON.stringify(State)), 100); Timers.add(Timer);
  Connected.onClose(() => { clearInterval(Timer); Timers.delete(Timer); });
  Connected.onMessage(Message => { CommandQueue = CommandQueue.then(() => RunCommand(Connected, JSON.parse(String(Message)))); });
});

const Approve = Page.getByRole("button", { name: "Approve", exact: true });
const Reply = Page.locator(".ai-reply");
const ReplyText = async () => (await Reply.count()) ? await Reply.innerText() : "";
const Strip = Page.locator(".ai-task-strip");
const Submit = async Text => {
  await Page.getByPlaceholder("Describe a task or movement").fill(Text);
  await Page.getByRole("button", { name: "Send to AI", exact: true }).click();
};
const WaitFor = async (Condition, Message, TimeoutMs = 30000) => {
  const Started = Date.now();
  while (!(await Condition())) {
    if (Date.now() - Started > TimeoutMs) {
      throw new Error(`Timed out: ${Message}\nAI panel: ${await Page.locator(".ai-panel").innerText().catch(() => "")}\nLast events: ${JSON.stringify(Events.slice(-6).map(Event => [Event.type, Event.source ?? Event.reason ?? Event.reply]))}`);
    }
    await Pause(50);
  }
};
const TaskEnds = Since => Events.filter(Event => Event.type === "task_end" && Event.At >= Since);
const Since_ = Since => Events.filter(Event => Event.At >= Since);
// Each move goes up from where the arm is, so no reply repeats the last target.
let Rise = 1;
const Up = () => ({ response_to_user: "Moving up.", task_status: "continue", motion_proposal: { move_delta_cm: { left: 0, farther: 0, up: 1 + (Rise++ % 3) * 0.5 } } });
// The recorded 02:08:53 reply.
const Recorded = { response_to_user: "I'm close to your hand now. I'll move a bit closer and lower to release the bottle into your palm.", task_status: "continue", motion_proposal: null };
const Corrected = /Your last reply said task_status "continue" but had no motion_proposal[^\n]*\nGive the motion_proposal JSON for the move you described, or reply task_status "done" or "ask" with motion_proposal null\./;
const TwiceReason = 'the AI twice replied "continue" without a motion: the app asked it again for the motion it described, and it still gave none.';
try {
  await Page.goto(Server.Url, { waitUntil: "domcontentloaded" });
  await Page.locator("canvas[data-cad-ready=true][data-assembly-ready=true]").waitFor({ timeout: CadReadyMs });
  await Page.getByRole("button", { name: "AI control mode", exact: true }).click();
  await Page.waitForFunction(() => document.querySelector("video")?.dataset.captureReady === "true");
  await Page.locator('.ai-companion[data-state="ready"]').waitFor();
  await WaitFor(async () => Preloads.length === 1, "the model warm-up");

  // 1. As at 02:08:53: motion 1 is approved; at check-in 2 the model says "continue" with no motion. The app asks it
  // once for the motion, in the same task and step; the motion it gives is offered for approval and the task goes on.
  let Since = Date.now();
  Replies.push(Up());
  await Submit("Raise the arm a little");
  await Approve.waitFor();
  Replies.push(Recorded, Up());
  let Asked = Completions.length;
  await Approve.click();
  await WaitFor(async () => Completions.length >= Asked + 2, "the check-in and the re-prompt reach the model");
  await Approve.waitFor();
  Assert.match(Completions[Asked].Text, /AUTO CHECK-IN/);
  Assert.match(Completions[Asked + 1].Text, Corrected);
  // The re-prompt is a correction within the check-in: the model's own reply comes right before it.
  const Turn = Completions[Asked + 1].Body.messages;
  Assert.equal(Turn.at(-2).role, "assistant");
  Assert.match(Turn.at(-2).content, /I'll move a bit closer and lower/);
  Assert.equal(await Strip.count(), 1, "The task is still running");
  Assert.deepEqual(TaskEnds(Since), [], "The task did not end");
  const NoMotion = Since_(Since).filter(Event => Event.type === "no_motion");
  Assert.equal(NoMotion.length, 1);
  Assert.match(NoMotion[0].reply, /move a bit closer and lower/);
  Assert.equal(NoMotion[0].task, "Raise the arm a little");
  Assert.equal(NoMotion[0].task_step, 2, "The re-prompt counts toward the same step");
  const Again = Since_(Since).filter(Event => Event.type === "reply" && Event.attempt === "no-motion");
  Assert.equal(Again.length, 1);
  Assert.equal(Again[0].task_status, "continue");
  Assert.ok(Again[0].motion, "The re-prompt's reply has the motion");
  Assert.match(Again[0].feedback, Corrected);
  Assert.ok(Since_(Since).some(Event => Event.type === "result" && Event.approvable && Event.task_status === "continue" && Event.At >= Again[0].At));
  await Page.screenshot({ path: Path.join(Output, "AiNoMotionReprompted.png") });
  Since = Date.now();
  await Page.getByRole("button", { name: "Stop task", exact: true }).click();
  await WaitFor(async () => TaskEnds(Since).length === 1, "Stop task ends it");
  Assert.equal(await Approve.count(), 0);

  // 2. "continue" without a motion twice: the task ends, saying it asked twice.
  Since = Date.now();
  Asked = Completions.length;
  Replies.push(Recorded, { ...Recorded, response_to_user: "Moving closer to your hand now." });
  await Submit("Raise the arm a little");
  await WaitFor(async () => TaskEnds(Since).length === 1, "the task ends after the second reply without a motion");
  Assert.equal(Completions.length, Asked + 2, "Asked once more, not again");
  Assert.match(Completions.at(-1).Text, Corrected);
  Assert.deepEqual(TaskEnds(Since).map(Event => [Event.source, Event.reason]), [["model_no_motion", TwiceReason]]);
  Assert.equal(Since_(Since).filter(Event => Event.type === "no_motion").length, 1);
  Assert.equal(await Approve.count(), 0);
  await WaitFor(async () => await Strip.count() === 0, "the task strip goes");

  // 3. "done" and "ask" end the task as before, with no re-prompt.
  Since = Date.now();
  Asked = Completions.length;
  Replies.push({ response_to_user: "The arm is already up.", task_status: "done", motion_proposal: null });
  await Submit("Raise the arm a little");
  await WaitFor(async () => TaskEnds(Since).length === 1, "done ends the task");
  Assert.deepEqual(TaskEnds(Since).map(Event => [Event.source, Event.reason]), [["model_done", "the AI reported the task done."]]);
  Assert.equal(Completions.length, Asked + 1);
  Assert.equal(Since_(Since).filter(Event => Event.type === "no_motion").length, 0);
  Since = Date.now();
  Asked = Completions.length;
  Replies.push({ response_to_user: "How far up?", task_status: "ask", motion_proposal: null });
  await Submit("Raise the arm a little");
  await WaitFor(async () => TaskEnds(Since).length === 1, "ask ends the task");
  Assert.deepEqual(TaskEnds(Since).map(Event => [Event.source, Event.reason]), [["model_ask", "the AI asked for input."]]);
  Assert.equal(Completions.length, Asked + 1);
  Assert.equal(Since_(Since).filter(Event => Event.type === "no_motion").length, 0);
  await WaitFor(async () => /How far up\?/.test(await ReplyText()), "the question is shown");
  // A "continue" re-prompt answered with "done" or "ask" ends the task that way.
  Since = Date.now();
  Replies.push(Recorded, { response_to_user: "It is in your hand.", task_status: "done", motion_proposal: null });
  await Submit("Raise the arm a little");
  await WaitFor(async () => TaskEnds(Since).length === 1, "done after the re-prompt ends the task");
  Assert.deepEqual(TaskEnds(Since).map(Event => Event.source), ["model_done"]);
  Since = Date.now();
  Replies.push(Recorded, { response_to_user: "Where is your hand?", task_status: "ask", motion_proposal: null });
  await Submit("Raise the arm a little");
  await WaitFor(async () => TaskEnds(Since).length === 1, "ask after the re-prompt ends the task");
  Assert.deepEqual(TaskEnds(Since).map(Event => Event.source), ["model_ask"]);

  // 4. Stop task and Reset AI context while the re-prompt is in flight end the task as usual; the late reply is never approvable.
  for (const [Button, Source] of [["Stop task", "stop_task"], ["Reset AI context", "context_reset"]]) {
    Since = Date.now();
    Asked = Completions.length;
    Replies.push(Recorded, HoldReply(Up()));
    await Submit("Raise the arm a little");
    await WaitFor(async () => Completions.length >= Asked + 2 && Held.length === 1, "the re-prompt is in flight");
    await Page.getByRole("button", { name: Button, exact: true }).click();
    await WaitFor(async () => TaskEnds(Since).length === 1, `${Button} ends the task`);
    ReleaseHeld();
    await Pause(1000);
    Assert.deepEqual(TaskEnds(Since).map(Event => Event.source), [Source]);
    Assert.equal(await Approve.count(), 0, `The reply that arrived after ${Button} is not approvable`);
    Assert.ok(Failed.some(Request => Request.Url.includes("/api/chat") && Request.At >= Since), "The re-prompt call was aborted");
    Assert.equal(Completions.length, Asked + 2);
  }

  Assert.deepEqual(Errors, []);
  console.log("PASS: a task reply of \"continue\" without a motion (the 02:08:53 reply) is asked once, in the same task and step, for the motion it described; the motion it then gives is offered for approval and the task goes on; a second reply without one ends the task saying it asked twice; \"done\" and \"ask\" end it as before (also when they answer the re-prompt); Stop task and Reset AI context during the re-prompt end it as usual, its late reply never approvable; the re-prompt is recorded (no_motion, reply attempt no-motion).");
} finally {
  for (const Timer of Timers) { clearInterval(Timer); clearTimeout(Timer); }
  await Browser.close(); await Server.Close();
}
