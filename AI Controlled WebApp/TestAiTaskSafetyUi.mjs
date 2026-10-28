import { chromium as Chromium } from "playwright";
import { CadReadyMs, LaunchOptions, StartUiServer, BackendPython } from "./UiTestLib.mjs";
import { mkdirSync as MakeDirectory } from "node:fs";
import { execFileSync as Execute } from "node:child_process";
import { fileURLToPath as FilePath } from "node:url";
import Path from "node:path";
import Assert from "node:assert/strict";

// The AI task loop against a mocked bridge, camera and model: the camera gate
// in /locate, check-in timing, stale anchors, no-op and repeated moves, the
// motion limit, the AI target snap (asked vs. sent, jitter at Approve,
// changed joint limits, bridge refusals), and how a task ends (faults, Stop
// and hold, Stop task, a moving arm, time limits: aborted, shown, recorded
// once). No request reaches the real bridge or arm.
const Root = Path.dirname(FilePath(import.meta.url)), Backend = Path.resolve(Root, "../back end");
const Python = BackendPython();
const State = JSON.parse(Execute(Python, ["-c", "import json, bridge_server; print(json.dumps(bridge_server.BridgeController().state_snapshot()))"], { cwd: Backend, encoding: "utf8" }));
Object.assign(State, { arm_connected: true, torque: true, port: "FAKE", motion_epoch: 5, motion_route: null, motion_fault: null });
for (const Servo of Object.values(State.servos)) Servo.moving = false;
const HomeJoints = structuredClone(State.joint_positions);
const Output = Path.join(Backend, "TestArtifacts"); MakeDirectory(Output, { recursive: true });
const Server = await StartUiServer();
const Browser = await Chromium.launch(LaunchOptions({ args: ["--use-fake-device-for-media-stream", "--use-fake-ui-for-media-stream"] }));
const Page = await Browser.newPage({ viewport: { width: 1600, height: 1000 }, permissions: ["camera"] });
const Errors = [], Commands = [], Completions = [], Events = [], Timers = new Set(), Failed = [];
const Pause = Milliseconds => new Promise(Resolve => setTimeout(Resolve, Milliseconds));
Page.on("pageerror", Error => Errors.push(Error.message));
// Requests the page aborted (a model call cut off by Stop, a fault or a time limit).
Page.on("requestfailed", Request => Failed.push({ Url: Request.url(), Error: Request.failure()?.errorText, At: Date.now() }));
// The session record is skipped for automated browsers; show it to this test.
await Page.addInitScript(() => Object.defineProperty(Navigator.prototype, "webdriver", { get: () => false }));
await Page.route("**/session-log/**", Route => { Events.push({ ...Route.request().postDataJSON(), At: Date.now() }); return Route.fulfill({ json: { ok: true } }); });
await Page.route("**/bridge-launch/**", Route => Route.fulfill({ json: { ok: true } }));
await Page.route("**/depth-server/**", Route => Route.fulfill({ json: {} }));

// The model (Ollama's native /api/chat): each scenario queues replies; a reply
// may read the prompt it answers. The page's warm-up (no messages) is kept apart.
let Replies = [];
const Preloads = [];
// The prompt text: the last user message with text (each image is its own message after it).
const LastUserText = Body => Body.messages.filter(Message => Message.role === "user" && Message.content).at(-1).content;
const CurrentPose = Text => Object.fromEntries([["x", /X left\/right: (-?[\d.]+)/], ["y", /Y farther\/closer: (-?[\d.]+)/],
  ["z", /Z up\/down: (-?[\d.]+)/], ["pitch", /Wrist pitch: (-?[\d.]+)/], ["gripper", /gripper: present_raw \d+, present_fraction (-?[\d.]+)/]]
  .map(([Key, Pattern]) => [Key, Number(Text.match(Pattern)?.[1])]));
await Page.route("**/ollama/**", async Route => {
  const Url = new URL(Route.request().url());
  if (Url.pathname.endsWith("/tags")) return Route.fulfill({ json: { models: [{ name: "SafetyTest", size: 1000 }] } });
  if (Url.pathname.endsWith("/show")) return Route.fulfill({ json: { capabilities: ["vision"] } });
  if (Url.pathname.endsWith("/ps")) return Route.fulfill({ json: { models: [{ name: "SafetyTest", context_length: 32768 }] } });
  if (!Url.pathname.endsWith("/api/chat")) return Route.fulfill({ json: { data: [{ id: "SafetyTest" }] } });
  const Body = Route.request().postDataJSON();
  if (Body.messages.length === 0) {
    Preloads.push({ At: Date.now(), Body });
    return Route.fulfill({ json: { model: Body.model, message: { role: "assistant", content: "" }, done: true, done_reason: "load" } });
  }
  const Text = LastUserText(Body);
  Completions.push({ At: Date.now(), Body, Text });
  const Reply = Replies.shift() ?? { response_to_user: "Nothing to do.", task_status: "done", motion_proposal: null };
  const Content = typeof Reply === "function" ? await Reply(Text, Body) : Reply;
  // The page may have aborted the call meanwhile: fulfilling it then fails.
  return Route.fulfill({ json: { model: Body.model, message: { role: "assistant", content: JSON.stringify(Content) }, done: true, done_reason: "stop",
    prompt_eval_count: 10, eval_count: 5 } }).catch(() => {});
});
// A model reply held until the test releases it: a call still in flight.
const Held = [];
const HoldReply = Content => () => new Promise(Resolve => Held.push(() => Resolve(Content)));
const ReleaseHeld = () => { while (Held.length) Held.shift()(); };

// The bridge: /locate answers per scenario; the websocket plans like the real
// bridge, sending this client nothing until planning ends and the epoch moves.
const CameraCheck = { ok: true, angle_deg: 0.12, offset_cm: 0.08, reason: "", reference_fitted_at: "2026-09-28T21:10:00", seq: null };
// The eraser lies with its 2 cm side across the jaws, so the fixed-jaw GRASP
// PLAN can grasp it (lib/GraspPlan.ts); turned 5 cm across, the jaw-plane sweep does, at a flatter wrist.
const Measured = { ok: true, rms_cm: 0.22, camera_check: CameraCheck, withheld: [{ label: "white box", reason: "553 cm away" }],
  objects: [{ label: "eraser", x: -5, y: 35, z: -2.3, top_z: -0.3, height_cm: 2, size_cm: [2, 5], method: "depth",
    support_measured: true, support_z_measured: -2.3, valid_fraction: 0.9, support_branch: "desk" },
  { label: "cup", x: 0.2, y: 40.8, z: -2.3, top_z: 6.7, height_cm: 9, size_cm: [7, 7], method: "depth" }] };
// Locate "hang" never answers; LocateDelayMs holds the answer, and OnLocate
// runs when the call arrives (after the frame was taken). Like the bridge,
// every measured object carries task_match and task_query_only (false unless
// a scenario sets them); LocateBodies keeps each request's body.
let Locate = Measured, LocateCalls = 0, LocateDelayMs = 0, OnLocate = null, PlanMs = 0, StatusOk = true, MotionStartedAt = 0, MotionStoppedAt = 0;
const LocateBodies = [], NotepadWrites = [];
await Page.route("**:8000/**", async Route => {
  const Url = new URL(Route.request().url());
  if (Url.pathname === "/ports") return Route.fulfill({ json: [] });
  if (Url.pathname === "/ik/preview") return Route.fulfill({ json: { ok: false } });
  if (Url.pathname === "/notepad" && Route.request().method() === "POST") NotepadWrites.push(Route.request().postDataJSON());
  if (Url.pathname === "/locate") {
    LocateCalls += 1;
    LocateBodies.push(Route.request().postDataJSON());
    if (Locate === "hang") return;
    OnLocate?.();
    if (LocateDelayMs) await Pause(LocateDelayMs);
    const Answer = typeof Locate === "object" && Array.isArray(Locate.objects)
      ? { ...Locate, objects: Locate.objects.map(Object_ => ({ task_match: false, task_query_only: false, ...Object_ })) } : Locate;
    return (Locate === "abort" ? Route.abort("failed") : Route.fulfill({ json: Answer })).catch(() => {});
  }
  return Route.fulfill({ json: { ok: true, calibrated: false, points: [], point_count: 0 } });
});
// Like the bridge, commands run one at a time and each gets one status reply,
// in order; a move is refused while another route runs.
let Socket, Quiet = false, NeverStart = false, CommandQueue = Promise.resolve();
// The chained motion's scenarios: RefuseWhen(Command) gives the refusal text for a chosen command; the
// gripper may stop GripperShortRaw short of its target (the bridge's route ending there); and with
// TipFollows the reported tip (the last joint position, model frame with X mirrored) goes to each
// send_ik target, TipShortCm short of it along Y.
let RefuseWhen = null, GripperShortRaw = 0, TipFollows = false, TipShortCm = 0;
const Statuses = [];
const Busy = "Wait for this motion to finish, or press Stop before another move";
const RunCommand = async (Connected, Command) => {
  Commands.push({ ...Command, At: Date.now() });
  const Reply = (Ok, Text) => { Statuses.push({ Type: Command.type, Ok, Text, At: Date.now() }); Connected.send(JSON.stringify({ type: "status", ok: Ok, text: Text })); };
  if (Command.type === "toggle_torque") {
    // As bridge_server.toggle_torque: a new epoch, any route ended, every target where its joint is.
    State.motion_epoch += 1; State.motion_route = null; State.torque = !State.torque;
    for (const Servo of Object.values(State.servos)) Object.assign(Servo, { moving: false, target_raw: Servo.present_raw });
    return Reply(true, State.torque ? "Torque on" : "Torque off");
  }
  if (!["send_ik", "move_servo", "go_home"].includes(Command.type)) {
    return Reply(true, Command.type === "set_motion_speed" ? `Motion speed set to ${Command.percent}%` : "OK");
  }
  if (State.motion_route) return Reply(false, Busy);
  if (Command.type === "go_home") return Reply(true, "Moving home");
  Quiet = true;
  await Pause(PlanMs);
  Quiet = false;
  const Refusal = RefuseWhen?.(Command);
  if (!StatusOk || Refusal) return Reply(false, Refusal || Busy);
  if (NeverStart) return Reply(true, "Planning");
  State.motion_epoch += 1; State.motion_route = { stage: 1, stages: 1 }; MotionStartedAt = Date.now();
  if (Command.type === "send_ik") {
    State.target_cm = { x: Command.target.x, y: Command.target.y, z: Command.target.z }; State.wrist_pitch = Command.target.wrist_pitch;
    if (TipFollows) State.joint_positions = [...State.joint_positions.slice(0, -1), { x: -Command.target.x, y: Command.target.y - TipShortCm, z: Command.target.z }];
    // A gripper target carried with the tip move (gripper_with_tip_version): the route opens the jaws on the way.
    if (Command.gripper_raw != null) {
      Object.assign(State.servos.gripper, { target_raw: Command.gripper_raw - GripperShortRaw, present_raw: Command.gripper_raw - GripperShortRaw });
    }
  } else {
    const Raw = Command.name === "gripper" ? Command.raw - GripperShortRaw : Command.raw;
    Object.assign(State.servos[Command.name], { target_raw: Raw, present_raw: Raw });
  }
  for (const Servo of Object.values(State.servos)) Servo.moving = true;
  Reply(true, "Moving");
  Connected.send(JSON.stringify(State));
  Timers.add(setTimeout(() => {
    State.motion_route = null; MotionStoppedAt = Date.now();
    for (const Servo of Object.values(State.servos)) Servo.moving = false;
  }, 700));
};
await Page.routeWebSocket("**/ws", Connected => {
  Socket = Connected; Connected.send(JSON.stringify(State));
  const Timer = setInterval(() => { if (!Quiet) Connected.send(JSON.stringify(State)); }, 100); Timers.add(Timer);
  Connected.onClose(() => { clearInterval(Timer); Timers.delete(Timer); });
  Connected.onMessage(Message => { CommandQueue = CommandQueue.then(() => RunCommand(Connected, JSON.parse(String(Message)))); });
});
// A two-motion limit keeps the limit scenario short (the rule is the same at
// 8), and a 12 s start limit the "never started" one (still above the 8 s plan).
// The arm-still wait and the model and /locate time limits are cut to 5 s.
await Page.route("**/src/lib/AiTaskLoop.ts*", async Route => {
  const Response = await Route.fetch(), Original = await Response.text();
  const Modified = Original.replace("export const MAX_AI_TASK_STEPS = 8;", "export const MAX_AI_TASK_STEPS = 2;")
    .replace(/export const TaskStartLimitMs = (?:25000|25e3);/, "export const TaskStartLimitMs = 12000;")
    .replace(/export const ArmStillWaitMs = (?:10000|1e4);/, "export const ArmStillWaitMs = 5000;")
    .replace(/export const AiModelTimeoutMs = (?:120000|12e4);/, "export const AiModelTimeoutMs = 5000;")
    .replace(/export const LocateTimeoutMs = (?:60000|6e4);/, "export const LocateTimeoutMs = 5000;");
  Assert.ok(Modified.includes("MAX_AI_TASK_STEPS = 2;") && Modified.includes("TaskStartLimitMs = 12000;"));
  Assert.ok(["ArmStillWaitMs", "AiModelTimeoutMs", "LocateTimeoutMs"].every(Name => Modified.includes(`${Name} = 5000;`)));
  await Route.fulfill({ response: Response, body: Modified });
});

const Banner = Page.locator(".camera-gate-banner");
const Approve = Page.getByRole("button", { name: "Approve", exact: true });
const Reply = Page.locator(".ai-reply");
const Submit = async Text => {
  await Page.getByPlaceholder("Describe a task or movement").fill(Text);
  await Page.getByRole("button", { name: "Send to AI", exact: true }).click();
};
const Idle = () => Page.locator('.ai-companion[data-state="ready"], .ai-companion[data-state="idle"]').first().waitFor();
const WaitFor = async (Condition, Message, TimeoutMs = 20000) => {
  const Started = Date.now();
  while (!(await Condition())) {
    if (Date.now() - Started > TimeoutMs) {
      const Shown = await Page.locator(".ai-panel").innerText().catch(() => "");
      throw new Error(`Timed out: ${Message}\nAI panel: ${Shown}\nLast events: ${JSON.stringify(Events.slice(-(Number(process.env.UiEventCount) || 4)).map(Event => [Event.type, Event.reason ?? Event.reply ?? Event.issues ?? Event.kind]))}`);
    }
    await Pause(50);
  }
};
const Stops = () => Events.filter(Event => Event.type === "task_stop");
// The mock arm never moves, so the bridge's held target from an earlier
// scenario would make a repeat of the same delta a genuine no-op. Park it away.
const ParkHeldTarget = () => { State.target_cm = { x: 0, y: 45, z: 0 }; };
try {
  await Page.goto(Server.Url, { waitUntil: "domcontentloaded" });
  await Page.locator("canvas[data-cad-ready=true][data-assembly-ready=true]").waitFor({ timeout: CadReadyMs });
  await Page.getByRole("button", { name: "AI control mode", exact: true }).click();
  await Page.waitForFunction(() => document.querySelector("video")?.dataset.captureReady === "true");
  await Page.locator('.ai-companion[data-state="ready"]').waitFor();
  // AI mode warms the model once, with the load options every call sends.
  await WaitFor(async () => Preloads.length === 1, "the model warm-up");

  // S2: a camera that moved since calibration stops the task before any model call.
  Locate = { ok: false, error_code: "camera_moved", error: "The camera or cart moved since calibration.",
    camera_check: { ...CameraCheck, ok: false, angle_deg: 5.24, offset_cm: 1.31, reason: "desk normal 5.2 deg from the calibration", reference_fitted_at: "2026-09-28T15:04:00" } };
  await Submit("Grab the eraser");
  await Banner.waitFor();
  Assert.match(await Banner.innerText(), /Camera moved 5\.2 deg \/ 1\.3 cm since calibration at 15:04: lock the head and run Automatic calibration/);
  Assert.match(await Banner.innerText(), /camera or cart moved/);
  await WaitFor(async () => await Page.locator(".ai-task-strip").count() === 0, "the task ends");
  Assert.equal(LocateCalls, 1);
  Assert.equal(Completions.length, 0, "No model call on a moved camera");
  Assert.equal(await Approve.count(), 0);
  Assert.match(await Reply.innerText(), /Stopped: Camera moved 5\.2 deg/);
  Assert.match(Stops().at(-1).reason, /^Camera moved/);
  await Page.screenshot({ path: Path.join(Output, "AiCameraGateBanner.png") });
  // Commands that need no measurement still work.
  await Page.getByRole("button", { name: "Home", exact: true }).click();
  await WaitFor(async () => Commands.some(Command => Command.type === "go_home"), "Home is sent");

  // A failed /locate request stops a grasp task the same way.
  Locate = "abort";
  await Submit("Grab the eraser");
  await WaitFor(async () => /bridge did not answer/.test(await Banner.innerText()), "fetch failure banner");
  Assert.equal(Completions.length, 0, "No model call without measurements");
  Assert.equal(await Approve.count(), 0);
  Assert.equal(await Page.locator(".ai-task-strip").count(), 0);

  // A calibration without a drift reference fails closed for a grasp task.
  // This suite's camera is a webcam, which can never carry that reference.
  Locate = { ok: false, error_code: "camera_not_verified", error: "Object positions are blocked: the calibration has no camera-position baseline.",
    camera_check: { ok: false, angle_deg: null, offset_cm: null, reason: "no baseline", reference_fitted_at: null, seq: null } };
  await Submit("Grab the eraser");
  await WaitFor(async () => /Tasks that act on objects need the depth camera/.test(await Banner.innerText()), "not-verified banner");
  Assert.equal(Completions.length, 0);

  // A task that needs no positions still runs without them (the calibration
  // is cleared right now): the model is told nothing was measured.
  Replies.push({ response_to_user: "A desk with an eraser and a cup.", task_status: "done", motion_proposal: null });
  await Submit("Describe the workspace; do not move.");
  await WaitFor(async () => Completions.length === 1, "the description reaches the model");
  Assert.match(Completions[0].Text, /NO MEASURED OBJECT POSITIONS: Object positions are blocked/);
  // Native /api/chat: the warm-up's load options, thinking off, a fixed answer
  // budget, and the prompt text then each image as its own bare-base64 message.
  const FirstAsk = Completions[0].Body, Warm = Preloads[0].Body;
  Assert.deepEqual([FirstAsk.stream, FirstAsk.think, FirstAsk.options.num_predict, FirstAsk.options.temperature], [false, false, 1024, 0.2]);
  Assert.deepEqual([FirstAsk.options.num_ctx, FirstAsk.options.num_batch, FirstAsk.keep_alive], [Warm.options.num_ctx, Warm.options.num_batch, Warm.keep_alive]);
  const Turn = FirstAsk.messages.slice(FirstAsk.messages.findLastIndex(Message => Message.role !== "user") + 1);
  Assert.equal(Turn[0].content, Completions[0].Text);
  Assert.ok(Turn.length >= 2 && Turn.slice(1).every(Message => Message.content === "" && Message.images?.length === 1
    && /^[A-Za-z0-9+/]+=*$/.test(Message.images[0].slice(0, 400))), "each image is its own message, without the data: prefix");
  await WaitFor(async () => /A desk with an eraser/.test(await Reply.innerText()), "the reply is shown");
  Assert.equal(Stops().filter(Stop => /depth camera/.test(Stop.reason)).length, 1, "Only the grasp task was stopped");

  // A passing check clears the banner and its numbers reach the model; no
  // "ground truth" or "±1 cm", and withheld detections are named.
  Locate = Measured;
  Replies.push({ response_to_user: "Raising first.", task_status: "continue", motion_proposal: { move_delta_cm: { left: 0, farther: 0, up: 2 } } });
  await Submit("Grab the eraser");
  await Approve.waitFor();
  await Banner.waitFor({ state: "detached" });
  const First = Completions.at(-1).Text;
  Assert.match(First, /camera check passed: 0\.1 deg \/ 0\.1 cm from the calibration fitted at 21:10/);
  Assert.match(First, /- white box: not measurable: 553 cm away/);
  Assert.match(First, /TASK OBJECT "eraser": detected this check-in at X -5\.0, Y 35\.0/);
  Assert.doesNotMatch(First, /ground truth|±1 cm/i);
  // S9: the verified task object carries a right-arm reach tag, and the rules
  // no longer push the model toward any object "however far it is".
  Assert.match(First, /TASK OBJECT "eraser": detected[^\n]*RIGHT ARM: (graspable|grasp height)/);
  // A grasp gets the fixed-jaw GRASP PLAN: the tip beside the eraser, not over its centre.
  Assert.match(First, /RIGHT ARM: graspable: GRASP POINT X -?[\d.]+, Y [\d.]+, Z -?[\d.]+ with wrist -[\d.]+ deg; HOVER POINT X -?[\d.]+, Y [\d.]+, Z [\d.]+ with wrist -[\d.]+ deg \([\d.]+ cm from the right arm's pan axis\)\. GRASP PLAN \(checked by the app against the gripper's shape\): the tip is the inner face of the FIXED jaw \(the one with the red tape\)[^\n]*BESIDE the eraser, 1\.0 cm outside its left side, NOT over its centre/);
  const System = Completions.at(-1).Body.messages[0].content;
  Assert.doesNotMatch(System, /however far it is/);
  Assert.match(System, /If the TASK OBJECT line says OUT OF REACH or BLOCKED, do not move toward it/);
  Assert.match(System, /GRASP RULE: the tip position you are given, and every tip target you send, is the inner face of the gripper's FIXED jaw/);
  // A hidden object keeps its last verdict, and a tagged one is approached in
  // relative steps (the app refuses target_cm on it).
  Assert.match(System, /When it gives a GRASP PLAN or a ready-made move \(this check-in, or when last measured while the object is hidden under the gripper\)/);
  Assert.match(System, /target_cm is accepted for the hover point even when tagged \[UNRELIABLE\]/);
  Assert.match(System, /Never reuse coordinates from your earlier replies/);
  // One move at a time: the example no longer pairs a tip move with the gripper.
  Assert.match(System, /ONE slider in direct_servo_targets, never both/);
  Assert.doesNotMatch(System, /"wrist_pitch_degrees":-45\.0,"direct_servo_targets"/);

  // S3: route planning keeps the websocket quiet for 8 s. No check-in may fire
  // on that frozen state; it waits for the new epoch, fresh state and a stop.
  const CheckIns = () => Events.filter(Event => Event.type === "task_check_in");
  PlanMs = 8000;
  Replies.push({ response_to_user: "The tip is higher now.", task_status: "done", motion_proposal: null });
  let Before = CheckIns().length;
  const ApprovedAt = Date.now();
  await Approve.click();
  await WaitFor(async () => Commands.some(Command => Command.type === "send_ik" && Command.At >= ApprovedAt), "send_ik is sent");
  await Pause(5000);
  Assert.equal(CheckIns().length, Before, "No check-in while the bridge plans and sends no state");
  await WaitFor(async () => CheckIns().length > Before, "the check-in after the motion", 20000);
  Assert.ok(MotionStartedAt - ApprovedAt >= 7900, "The motion started only after planning");
  Assert.ok(CheckIns().at(-1).At >= MotionStartedAt + 600, "The check-in follows the whole motion (700 ms here), not the minimum alone");
  Assert.ok(CheckIns().at(-1).At >= MotionStoppedAt, "The check-in waits for the motion to stop");
  await WaitFor(async () => Completions.at(-1).Text.includes("AUTO CHECK-IN"), "the check-in reaches the model");
  const CheckIn = Completions.at(-1).Text;
  Assert.match(CheckIn, /Your last approved motion \(motion 1\) started and has stopped: the tip moved/);
  Assert.doesNotMatch(CheckIn, /has finished|arrival position is the current position/);
  // The history sits between the system prompt and this check-in's own messages
  // (its text, then each image on its own).
  const CheckInMessages = Completions.at(-1).Body.messages;
  Assert.deepEqual(CheckInMessages.slice(1, CheckInMessages.findLastIndex(Message => Message.role !== "user") + 1)
    .map(Message => Object.keys(Message).sort().join() + ":" + Message.role),
    ["content,role:user", "content,role:assistant"], "History is whole turns of plain chat messages");
  await WaitFor(async () => await Page.locator(".ai-task-strip").count() === 0, "the task finishes");
  PlanMs = 300;

  // A gripper-only step is a motion too: it produces a check-in.
  Replies.push(Text => ({ response_to_user: "Opening the gripper.", task_status: "continue",
    motion_proposal: { direct_servo_targets: [{ name: "gripper", target_fraction: CurrentPose(Text).gripper > 0.5 ? 0 : 1 }] } }));
  Replies.push({ response_to_user: "Gripper moved.", task_status: "done", motion_proposal: null });
  await Submit("Open the gripper");
  await Approve.waitFor();
  Before = CheckIns().length;
  await Approve.click();
  await WaitFor(async () => Commands.some(Command => Command.type === "move_servo" && Command.name === "gripper"), "move_servo is sent");
  await WaitFor(async () => CheckIns().length > Before, "the gripper check-in");
  await WaitFor(async () => /gripper now at raw/.test(Completions.at(-1).Text), "the check-in states where the gripper is");
  await WaitFor(async () => await Page.locator(".ai-task-strip").count() === 0, "the task finishes");

  // A relative move whose anchor went stale is refused, not sent.
  Replies.push({ response_to_user: "Stepping farther.", task_status: "continue", motion_proposal: { move_delta_cm: { left: 0, farther: 2, up: 0 } } });
  await Submit("Grab the eraser");
  await Approve.waitFor();
  State.joint_positions = HomeJoints.map((Point, Index, All) => Index === All.length - 1 ? { ...Point, x: Point.x + 3 } : Point);
  await Pause(400);
  let Sends = Commands.filter(Command => Command.type === "send_ik").length;
  await Approve.click();
  await WaitFor(async () => /the arm moved 3\.0 cm since this relative move was proposed/.test(await Reply.innerText()), "stale anchor refusal");
  Assert.equal(Commands.filter(Command => Command.type === "send_ik").length, Sends, "A stale relative move is never sent");
  Assert.equal(await Page.locator(".ai-task-strip").count(), 0);
  State.joint_positions = structuredClone(HomeJoints);

  // A refused motion (ok=false from the bridge): nothing moved, so the first
  // refusal in a task goes back to the model at a check-in without using up
  // the step; the second stops the task (it used to stop at the first).
  StatusOk = false;
  ParkHeldTarget();
  Replies.push({ response_to_user: "Up a little.", task_status: "continue", motion_proposal: { move_delta_cm: { left: 0, farther: 0, up: 2 } } },
    { response_to_user: "Up a bit less.", task_status: "continue", motion_proposal: { move_delta_cm: { left: 0, farther: 0, up: 1.5 } } });
  await Pause(300);
  await Submit("Grab the eraser");
  await Approve.waitFor();
  const BeforeRefusal = Completions.length;
  await Approve.click();
  await WaitFor(async () => Completions.length > BeforeRefusal && /the bridge refused it/.test(Completions.at(-1).Text), "the first refusal goes back to the model");
  Assert.match(Completions.at(-1).Text, /AUTO CHECK-IN — motion 1 of 2 max/, "The refused motion does not use up the step");
  Assert.match(Completions.at(-1).Text, /Your last approved motion \(motion 1\) was NOT executed: the bridge refused it: Wait for this motion to finish, or press Stop before another move\. The arm did not move\./);
  Assert.doesNotMatch(Completions.at(-1).Text, /Motions approved so far in this task:/, "The refused motion is not listed as done");
  Assert.ok(Events.some(Event => Event.type === "bridge_refused" && /Wait for this motion/.test(Event.reason)));
  await Approve.waitFor();
  await Approve.click();
  await WaitFor(async () => /bridge refused the approved motion/.test(await Reply.innerText()), "the second refusal stops the task");
  Assert.match(Stops().at(-1).reason, /It refused one earlier in this task too/);
  Assert.equal(await Page.locator(".ai-task-strip").count(), 0);
  StatusOk = true;

  // A refusal of another command while the task waits (Home pressed during
  // the route) is not the approved motion's: the task goes on to its check-in.
  ParkHeldTarget(); PlanMs = 1500;
  Replies.push({ response_to_user: "Up a little.", task_status: "continue", motion_proposal: { move_delta_cm: { left: 0, farther: 0, up: 2 } } },
    { response_to_user: "Done.", task_status: "done", motion_proposal: null });
  await Submit("Grab the eraser");
  await Approve.waitFor();
  Before = CheckIns().length;
  let Clicked = Date.now();
  await Approve.click();
  await WaitFor(async () => Commands.some(Command => Command.type === "send_ik" && Command.At >= Clicked), "send_ik is sent");
  await Page.getByRole("button", { name: "Home", exact: true }).click();
  await WaitFor(async () => Statuses.some(Status => Status.Type === "go_home" && !Status.Ok && Status.At >= Clicked), "the bridge refuses Home during the route");
  await WaitFor(async () => CheckIns().length > Before, "the check-in after the approved motion");
  Assert.ok(!Stops().some(Stop => Stop.At >= Clicked), "Home's refusal did not stop the task");
  await WaitFor(async () => await Page.locator(".ai-task-strip").count() === 0, "the task finishes");
  PlanMs = 300;

  // A gripper step that restates the wrist roll sends only the gripper: a
  // second move would be refused while the first runs.
  const RollRaw = State.servos.wrist_roll.target_raw;
  const RollFraction = Text => Number(Text.match(/wrist_roll: present_raw \d+, present_fraction (-?[\d.]+)/)[1]);
  Replies.push(Text => ({ response_to_user: "Toggling the gripper.", task_status: "continue", motion_proposal: { direct_servo_targets: [
    { name: "gripper", target_fraction: CurrentPose(Text).gripper > 0.5 ? 0 : 1 }, { name: "wrist_roll", target_raw: RollRaw }] } }),
  { response_to_user: "Gripper moved.", task_status: "done", motion_proposal: null });
  await Submit("Open the gripper");
  await Approve.waitFor();
  Before = CheckIns().length;
  let ServoSends = Commands.filter(Command => Command.type === "move_servo").length;
  Clicked = Date.now();
  await Approve.click();
  await WaitFor(async () => CheckIns().length > Before, "the gripper check-in");
  Assert.deepEqual(Commands.filter(Command => Command.type === "move_servo").slice(ServoSends).map(Command => Command.name), ["gripper"]);
  Assert.ok(!Statuses.some(Status => !Status.Ok && Status.At >= Clicked), "Nothing was refused");
  await WaitFor(async () => await Page.locator(".ai-task-strip").count() === 0, "the task finishes");

  // Two sliders changing at once are refused with feedback to split them.
  let Asked = Completions.length;
  ServoSends = Commands.filter(Command => Command.type === "move_servo").length;
  Replies.push(Text => ({ response_to_user: "Both at once.", task_status: "continue", motion_proposal: { direct_servo_targets: [
    { name: "gripper", target_fraction: CurrentPose(Text).gripper > 0.5 ? 0 : 1 }, { name: "wrist_roll", target_fraction: RollFraction(Text) > 0.5 ? 0.2 : 0.8 }] } }),
  async () => ({ response_to_user: `Which first? (approve shown: ${await Approve.count()})`, task_status: "ask", motion_proposal: null }));
  await Submit("Open the gripper");
  await WaitFor(async () => /Which first\?/.test(await Reply.innerText()), "the model asks after the feedback");
  Assert.equal(Completions.length, Asked + 2);
  Assert.match(Completions.at(-1).Text, /Change one slider per motion \(the arm runs one move at a time\): Gripper and Wrist Roll were proposed together/i);
  Assert.match(await Reply.innerText(), /approve shown: 0/);
  Assert.equal(await Approve.count(), 0);
  Assert.equal(Commands.filter(Command => Command.type === "move_servo").length, ServoSends);

  // S4: a proposal equal to the target the bridge is holding (0.7 cm above the
  // sagging tip, as in the incident) is never offered. One corrective turn goes
  // to the model; the task's second no-op stops it.
  let ApproveShownDuringCorrection = null;
  const Holding = async (Text, Corrective) => {
    const Pose = CurrentPose(Text), Offset = State.workspace_offset, Held = { x: Pose.x, y: Pose.y, z: Pose.z + 0.7 };
    if (!Corrective) {
      State.target_cm = { x: -(Held.x - Offset.x), y: Held.y - Offset.y, z: Held.z - Offset.z }; State.wrist_pitch = Pose.pitch;
      await Pause(300);
    } else ApproveShownDuringCorrection = await Approve.count();
    return { response_to_user: "Moving onto the eraser.", task_status: "continue",
      motion_proposal: { target_cm: Held, wrist_pitch_degrees: Pose.pitch } };
  };
  const HoldingPose = { Text: "" };
  Replies.push(async Text => { HoldingPose.Text = Text; return Holding(Text, false); }, async () => Holding(HoldingPose.Text, true));
  Asked = Completions.length;
  await Submit("Grab the eraser");
  await WaitFor(async () => /twice proposed a move that would not move the arm/.test(await Reply.innerText()), "the second no-op stops the task");
  Assert.equal(Completions.length, Asked + 2, "Exactly one corrective turn");
  Assert.match(Completions.at(-1).Text, /would not move the arm\. It was not sent and not counted/);
  Assert.equal(ApproveShownDuringCorrection, 0, "A no-op is never approvable");
  Assert.equal(await Approve.count(), 0);
  Assert.ok(Events.filter(Event => Event.type === "no_op").length >= 2);
  Assert.match(Stops().at(-1).reason, /would not move the arm/);
  // The eraser was planned (in reach): going on is the advice, not moving it (2026-10-01 07:56 sent the user to move a reachable can).
  Assert.match(Stops().at(-1).advice, /^The eraser is within reach and its grasp was planned; .*Say "try again" to go on from where the arm is/);

  // The same target offered again right after itself stops the task.
  let Repeated = null;
  ParkHeldTarget();
  Replies.push(Text => { const Pose = CurrentPose(Text); Repeated = { target_cm: { x: Pose.x, y: Pose.y + 3, z: Pose.z }, wrist_pitch_degrees: Pose.pitch };
    return { response_to_user: "Reaching farther.", task_status: "continue", motion_proposal: Repeated }; },
  () => ({ response_to_user: "Reaching farther again.", task_status: "continue", motion_proposal: Repeated }));
  await Submit("Grab the eraser");
  await Approve.waitFor();
  await Approve.click();
  await WaitFor(async () => /the same target as its previous motion/.test(await Reply.innerText()), "a repeated target stops the task");
  Assert.equal(await Approve.count(), 0);
  Assert.equal(await Page.locator(".ai-task-strip").count(), 0);

  // The motion limit (2 here) ends the task with a stated reason after the
  // last allowed motion is approved.
  ParkHeldTarget();
  Replies.push({ response_to_user: "Up first.", task_status: "continue", motion_proposal: { move_delta_cm: { left: 0, farther: 0, up: 2 } } },
    { response_to_user: "Then farther.", task_status: "continue", motion_proposal: { move_delta_cm: { left: 0, farther: 2, up: 0 } } });
  await Submit("Grab the eraser");
  await Approve.waitFor();
  await Approve.click();
  await WaitFor(async () => Completions.at(-1).Text.includes("This is your last motion for this task: if you cannot finish with it, reply ask"), "last-step check-in");
  await Approve.waitFor();
  Sends = Commands.filter(Command => Command.type === "send_ik").length;
  await Approve.click();
  await WaitFor(async () => /used all 2 motions allowed for one task; the last one was sent/.test(await Reply.innerText()), "the limit is explained");
  Assert.equal(Commands.filter(Command => Command.type === "send_ik").length, Sends + 1);
  Assert.equal(await Page.locator(".ai-task-strip").count(), 0);
  Assert.match(Stops().at(-1).reason, /used all 2 motions/);

  // S9: a task object measured beyond the right arm's reach on a verified
  // camera stops the task before any model call, advising to move it closer.
  Locate = { ...Measured, objects: [{ ...Measured.objects[0], x: 16.1, y: 35.8, top_z: 5.2 }] };
  Asked = Completions.length;
  await Submit("Grab the eraser");
  await WaitFor(async () => /out of the right arm's reach \(46\.3 cm from its shoulder\)/.test(await Reply.innerText()), "an unreachable object stops the task");
  Assert.equal(Completions.length, Asked, "No model call for an unreachable task object");
  Assert.match(await Reply.innerText(), /Move the eraser closer to the right arm/);
  Assert.equal(await Page.locator(".ai-task-strip").count(), 0);
  Locate = Measured;

  // The model copies the NEXT STEP exactly: its motion_proposal, by matching braces.
  const NextStep = Text => {
    const From = Text.indexOf('"motion_proposal":', Text.indexOf("NEXT STEP of the grasp"));
    if (From < 0) return null;
    let Depth = 0;
    for (let Index = From + 18; Index < Text.length; Index++) {
      Depth += Text[Index] === "{" ? 1 : Text[Index] === "}" ? -1 : 0;
      if (Depth === 0) return JSON.parse(Text.slice(From + 18, Index + 1));
    }
    return null;
  };

  // The eraser turned 5 cm across the jaws used to stop the task here, before any model call (the moving jaw would
  // have to close 7.2 cm). Since 2026-10-04 grasp planning tries unless it is sure it cannot work (GraspRules.BestEffort,
  // Aaran: "just let it try unless your code is totally sure it can't grab it"), and the jaw-plane sweep now plans that
  // eraser at a flatter wrist, as the can below. What still stops a task before the model is a grasp the app is sure
  // of: a box wider than the moving jaw ever meets.
  ParkHeldTarget();
  Locate = { ...Measured, objects: [{ label: "box", x: -5, y: 35, z: -2.3, top_z: 4.7, height_cm: 7, size_cm: [12, 12], method: "depth",
    support_measured: true, support_z_measured: -2.3, valid_fraction: 0.9, support_branch: "desk" }] };
  Asked = Completions.length;
  Clicked = Date.now();
  await Submit("Grab the box");
  await WaitFor(async () => /is not graspable by this gripper: it measured 12\.0 x 12\.0 cm; .*close 14\.\d cm/.test(await Reply.innerText()), "a box too wide for the jaws stops the task");
  Assert.equal(Completions.length, Asked, "No model call for an object the gripper certainly cannot grasp");
  Assert.equal(await Page.locator(".ai-task-strip").count(), 0);
  Assert.ok(Events.some(Event => Event.type === "grasp_plan" && Event.At >= Clicked && Event.code === "wide"), "The refusal is recorded");
  Assert.equal(await Approve.count(), 0);

  // What the app is only unsure of is a BEST-EFFORT GRASP: TestGraspPlan.mjs's tin (6.0 x 5.5 cm, 12.1 cm tall) is planned
  // as measured although it may be up to 8 cm across, and its grip may slip. The model is told so, the card of its copy
  // of the NEXT STEP says "Best-effort grasp, may not work: ..." with the same doubts, and grasp_plan records them.
  ParkHeldTarget();
  const Tin = { label: "tin", x: -5.44, y: 36.39, z: -1.97, top_z: 10.16, height_cm: 12.13, size_cm: [6.0, 5.5], method: "depth",
    support_measured: true, support_z_measured: -1.77, valid_fraction: 0.9, support_branch: "desk" };
  Locate = { ...Measured, objects: [Tin] };
  const TinStep = { Seen: null, Text: "" };
  Replies.push(Text => {
    TinStep.Text = Text;
    TinStep.Seen = NextStep(Text);
    return { response_to_user: "Trying the best-effort grasp.", task_status: "continue", motion_proposal: TinStep.Seen };
  });
  Asked = Completions.length;
  Clicked = Date.now();
  await Submit("Grab the tin");
  await WaitFor(async () => await Approve.count() > 0, "the tin's best-effort step is offered");
  Assert.equal(Completions.length, Asked + 1, "One model call: the app does not stop a grasp it is only unsure of");
  const Unsure = TinStep.Text.match(/BEST-EFFORT GRASP: the app is not sure this grasp holds the tin \((.+?)\), but nothing it checks rules it out/)?.[1];
  Assert.ok(Unsure, `the model is told the grasp is best-effort and why: ${TinStep.Text.match(/TASK OBJECT "tin"[^\n]*/)?.[0]}`);
  Assert.match(Unsure, /planned as measured \(6\.0 x 5\.5 cm\)/);
  Assert.match(TinStep.Text, /TASK OBJECT "tin": detected this check-in[^\n]*RIGHT ARM: graspable \(best effort: the app is not sure it holds\): GRASP POINT/);
  Assert.ok(TinStep.Seen, "the prompt carries the plan's NEXT STEP");
  const TinNote = (await Page.locator(".ai-issue-text").allInnerTexts()).find(Text => /^Best-effort grasp, may not work: /.test(Text));
  Assert.ok(TinNote, "the card says the grasp may not work");
  Assert.ok(TinNote.includes(Unsure), `the card names what the model was told: ${TinNote}`);
  const TinEvent = Events.find(Event => Event.type === "grasp_plan" && Event.At >= Clicked && Event.label === "tin");
  Assert.ok(TinEvent && TinEvent.code === null && TinEvent.warnings?.length >= 2, JSON.stringify(TinEvent));
  Assert.ok(TinEvent.warnings.every(Warning => TinNote.includes(Warning)), "the card lists every warning grasp_plan records");
  await Page.screenshot({ path: Path.join(Output, "AiBestEffortCard.png") });
  // At 1600 x 1000 the floating Live view (bottom left by default) covered the foot of this tall card, Approve and Block
  // included (Block was pressed from the keyboard here); it now stands clear of them while they are shown (2026-10-08
  // review): clicked with the mouse, as a user would, which Playwright refuses while another element takes the click.
  const Block = Page.getByRole("button", { name: "Block", exact: true });
  await Block.click({ timeout: 5000 });
  await WaitFor(async () => await Page.locator(".ai-task-strip").count() === 0, "the block ends the task");
  Locate = Measured;

  // Grasp M3: the can of 2026-09-30 16:49 (measured 5.0 x 4.5, 12.1 cm tall) is
  // too wide for the jaws from above; the jaw-plane sweep plans it at a flatter
  // wrist, as if 6.7 cm across, with a head start along the fingers for the
  // closing jaw's push. The model is told so and its copy of the NEXT STEP
  // (the hover point beside the can) is offered.
  ParkHeldTarget();
  const Can = { label: "can", x: -5.44, y: 36.39, z: -1.97, top_z: 10.16, height_cm: 12.13, size_cm: [5.0, 4.5], method: "depth",
    support_measured: true, support_z_measured: -1.77, valid_fraction: 0.9, support_branch: "desk" };
  Locate = { ...Measured, objects: [Can] };
  const CanStep = { Seen: null, Text: "" };
  Replies.push(Text => {
    CanStep.Text = Text;
    CanStep.Seen = NextStep(Text);
    return { response_to_user: "The next step of the can's GRASP PLAN.", task_status: "continue", motion_proposal: CanStep.Seen };
  });
  Asked = Completions.length;
  Clicked = Date.now();
  await Submit("Grab the can");
  await WaitFor(async () => await Approve.count() > 0, "the can's next step is offered");
  Assert.equal(Completions.length, Asked + 1, "No feedback turn: the plan's own step passes the guard and the snap");
  // The grasp point is printed the arm's sag (0.45 cm) above where the jaws close: raised over mid-height Z 4.1 until the
  // jaws reach around the can (2026-10-03: the straddle) with the palm clear of it.
  const Raised = Number(CanStep.Text.match(/The grasp height is (\d\.\d) cm above its mid-height \(at mid-height /)?.[1]);
  Assert.ok(Raised >= 0.5 && Raised <= 3, `raised ${Raised}`);
  const GraspZ = (Math.ceil((4.095 + Raised + 0.45) * 10 - 1e-9) / 10).toFixed(1).replace(".", "\\.");
  Assert.match(CanStep.Text, new RegExp(`TASK OBJECT "can": detected this check-in[^\\n]*RIGHT ARM: graspable: GRASP POINT X -?[\\d.]+, Y [\\d.]+, Z ${GraspZ} with wrist -[\\d.]+ deg; HOVER POINT X -?[\\d.]+, Y [\\d.]+, Z 13\\.2`));
  // Both jaws reach at least 1.5 cm past its centre (GraspPlan.ts StraddleCm).
  const Around = CanStep.Text.match(/so the fixed finger reaches (\d\.\d) cm past its middle and the moving jaw, swinging closed, (\d\.\d) cm past it/)?.slice(1).map(Number) ?? [];
  Assert.ok(Around.length === 2 && Math.min(...Around) >= 1.5, `the jaws around it: ${Around}`);
  Assert.match(CanStep.Text, /1\.0 cm outside its left side \(planned as if it were 6\.7 cm across: a round object measures narrower than it is\)[^\n]*the wrist stays at -[\d.]+ deg all the way down[^\n]*and the gripper opens to \d+% \(exactly that[^\n]*The jaws reach around it before they close[^\n]*Closing, the moving jaw pushes it about \d\.\d cm along the fingers toward their tips[^\n]*The app checked that grip for it 5\.0 to 6\.7 cm across/);
  // No finger offset since 2026-10-03 (the camera saw the real fixed finger where the arm model puts it); the sag is said.
  Assert.doesNotMatch(CanStep.Text, /the real fixed finger stands about/);
  Assert.match(CanStep.Text, /The GRASP POINT's Z is 0\.5 cm above where the jaws close/);
  // The gripper first opens to the plan's fraction, or (already open) the tip goes to the hover point beside the can.
  const Hover = CanStep.Text.match(/HOVER POINT X (-?[\d.]+), Y ([\d.]+), Z ([\d.]+) with wrist (-[\d.]+) deg/).slice(1).map(Number);
  const Opens = Number(CanStep.Text.match(/the gripper opens to (\d+)%/)[1]) / 100;
  Assert.ok(CanStep.Seen?.target_cm ? CanStep.Seen.target_cm.x === Hover[0] && CanStep.Seen.target_cm.x > Can.x + 3.3 && CanStep.Seen.wrist_pitch_degrees === Hover[3]
    : CanStep.Seen?.direct_servo_targets?.[0]?.target_fraction === Opens, JSON.stringify(CanStep.Seen));
  const CanEvent = Events.find(Event => Event.type === "grasp_plan" && Event.At >= Clicked && Event.label === "can");
  Assert.ok(CanEvent && CanEvent.closing === "sweep" && CanEvent.grip_deg <= 20 && CanEvent.grip3d_deg != null && CanEvent.drift_cm > 0.5
    && CanEvent.depth_offset_cm > CanEvent.drift_cm && Math.min(...CanEvent.straddle_cm) >= 1.4, JSON.stringify(CanEvent));
  Assert.deepEqual(CanEvent.diameter_cm, [5, 6.67]);
  await Page.getByRole("button", { name: "Block", exact: true }).click();
  await WaitFor(async () => await Page.locator(".ai-task-strip").count() === 0, "the block ends the task");
  Locate = Measured;

  // 2026-09-30 23:50: the model sent the tip to the object's centre with the
  // jaws open, and the fixed finger came down onto it. The guard refuses it
  // (it is not the NEXT STEP, and says where the fixed finger would go) with
  // the NEXT STEP's JSON; the model's copy of it (open and go to the hover point) is offered.
  ParkHeldTarget();
  const PlanHover = { Seen: null };
  Replies.push({ response_to_user: "Down onto the eraser.", task_status: "continue", motion_proposal: { target_cm: { x: -5, y: 35, z: -1.3 }, wrist_pitch_degrees: -60 } },
    Text => {
      const From = Text.indexOf("Copy the NEXT STEP's motion_proposal exactly: ");
      PlanHover.Seen = From < 0 ? null : NextStep(Text.slice(From).replace("Copy the NEXT STEP's", "NEXT STEP of the grasp"));
      return { response_to_user: "The NEXT STEP.", task_status: "continue", motion_proposal: PlanHover.Seen };
    });
  Asked = Completions.length;
  Clicked = Date.now();
  await Submit("Grab the eraser");
  await WaitFor(async () => await Approve.count() > 0, "the NEXT STEP is offered after the refusal");
  Assert.equal(Completions.length, Asked + 2, "One feedback turn");
  Assert.match(Completions.at(-1).Text, /- That move is not the NEXT STEP, and it ends low beside the eraser \(within 3 cm of it, below its top \+ 1 cm\)[^\n]* At that target the gripper's FIXED finger \(the red-taped jaw; the tip is on its inside\) would go \d\.\d cm into the eraser measured at X -5\.0, Y 35\.0/);
  Assert.ok(PlanHover.Seen?.target_cm, "The refusal gives the NEXT STEP's JSON");
  Assert.ok(PlanHover.Seen.target_cm.x > -5 && [0.8, undefined].includes(PlanHover.Seen.gripper_before), `The hover point is beside the eraser, on its +X side: ${JSON.stringify(PlanHover.Seen)}`);
  Assert.match(Completions[Asked].Text, /\nNEXT STEP TO COPY \(from the TASK OBJECT line; the app refuses any other move near the eraser\): "motion_proposal":\{"target_cm":/);
  Assert.ok(Events.some(Event => Event.type === "grasp_plan" && Event.At >= Clicked && Event.height_by === "mid"));
  await Page.getByRole("button", { name: "Block", exact: true }).click();
  await WaitFor(async () => await Page.locator(".ai-task-strip").count() === 0, "the block ends the task");

  // 2026-10-01 18:54: the model did not copy the NEXT STEP. Here every reply (the first and its 4 revisions) goes down onto
  // the eraser, and each refusal repeats the step's JSON. Instead of ending the task the app offers its own planned step,
  // labelled as the app's; approved, it runs like any motion, and the next check-in tells the model it was the app's.
  ParkHeldTarget();
  const GripperBefore = structuredClone(State.servos.gripper);
  const PlannedCheckIn = { Text: "" };
  const Descend = { response_to_user: "Down to the eraser.", task_status: "continue", motion_proposal: { target_cm: { x: -5, y: 35, z: -1.3 }, wrist_pitch_degrees: -60 } };
  Replies.push(Descend, Descend, Descend, Descend, Descend,
    Text => { PlannedCheckIn.Text = Text; return { response_to_user: "Stopping here.", task_status: "ask", motion_proposal: null }; });
  Asked = Completions.length;
  Clicked = Date.now();
  await Submit("Grab the eraser");
  await WaitFor(async () => await Approve.count() > 0, "the app's planned step is offered", 30000);
  Assert.equal(Completions.length, Asked + 5, "The reply and its 4 revisions");
  for (const Feedback of Completions.slice(Asked + 1)) {
    Assert.match(Feedback.Text, /- That move is not the NEXT STEP[^\n]*Copy the NEXT STEP's motion_proposal exactly: "motion_proposal":\{"target_cm":/);
  }
  Assert.equal(await Page.locator(".ai-motion-copy h2").innerText(), "App's planned step");
  Assert.equal(await Page.locator('[data-planned-step="true"]').innerText(), "This is the app's planned NEXT STEP from the GRASP PLAN, not the AI's: the AI did not follow it. Check it before approving.");
  Assert.match(await Reply.innerText(), /The AI did not follow the app's NEXT STEP in 4 revisions, so the app offers its own planned step instead: .*Approve it to go on, or Block to stop the task\./);
  const Offer = Events.find(Event => Event.type === "planned_step" && Event.At >= Clicked);
  Assert.ok(Offer?.offered && /"target_cm"/.test(Offer.step) && Offer.refused.some(Issue => Issue.startsWith("That move is not the NEXT STEP")), JSON.stringify(Offer));
  await Approve.click();
  await WaitFor(() => Boolean(PlannedCheckIn.Text), "the check-in after the app's planned step", 30000);
  Assert.match(PlannedCheckIn.Text, /That motion was the app's own NEXT STEP from the GRASP PLAN, approved by the user because your proposals did not follow it/);
  Assert.match(PlannedCheckIn.Text, /- Motion 1 \((?:chained, (?:one motion, )?)?the app's planned NEXT STEP\): /);
  Assert.equal(Events.find(Event => Event.type === "approve" && Event.At >= Clicked)?.motion?.planned, true);
  await WaitFor(async () => await Page.locator(".ai-task-strip").count() === 0, "the model's ask ends the task");
  Object.assign(State.servos.gripper, GripperBefore);
  await Pause(300);

  // A contact task's NEXT STEP is not one the app holds the arm to (only a planned grasp's is): its prompt gives the step,
  // but no NEXT STEP TO COPY line claiming the app refuses other moves (2026-10-01 review).
  ParkHeldTarget();
  Replies.push({ response_to_user: "Stopping here.", task_status: "ask", motion_proposal: null });
  Asked = Completions.length;
  await Submit("Push the eraser");
  await WaitFor(async () => /Stopping here\./.test(await Reply.innerText()), "the contact task's first check-in");
  Assert.equal(Completions.length, Asked + 1);
  Assert.match(Completions[Asked].Text, /TASK OBJECT "eraser": detected[^\n]*NEXT STEP of the grasp [^\n]*"motion_proposal":\{/);
  Assert.doesNotMatch(Completions[Asked].Text, /NEXT STEP TO COPY|the app refuses any other move/);
  await WaitFor(async () => await Page.locator(".ai-task-strip").count() === 0, "the ask ends the contact task");

  // S2.5: a task object tagged [UNRELIABLE] is never an absolute descent target
  // (top Z -0.3, so below Z 2.7); a hover above it is, checked in the next image.
  ParkHeldTarget();
  Locate = { ...Measured, objects: [{ ...Measured.objects[0], extrapolated: true }, Measured.objects[1]] };
  const ApproveDuringRefusals = [];
  Replies.push({ response_to_user: "To the eraser.", task_status: "continue", motion_proposal: { target_cm: { x: -5, y: 35, z: 1 } } },
    async () => { ApproveDuringRefusals.push(await Approve.count());
      return { response_to_user: "Above it, then.", task_status: "continue", motion_proposal: { target_cm: { x: -5, y: 35, z: 2.5 } } }; },
    async () => { ApproveDuringRefusals.push(await Approve.count());
      return { response_to_user: "Where exactly is the eraser?", task_status: "ask", motion_proposal: null }; });
  Asked = Completions.length;
  await Submit("Grab the eraser");
  await WaitFor(async () => /Where exactly is the eraser\?/.test(await Reply.innerText()), "two refusals, then the model asks");
  Assert.equal(Completions.length, Asked + 3);
  Assert.match(Completions[Asked].Text, /TASK OBJECT "eraser": detected[^\n]*\[UNRELIABLE[^\n]*target_cm is accepted only for the hover waypoint/);
  for (const Feedback of Completions.slice(Asked + 1).map(Completion => Completion.Text)) {
    Assert.match(Feedback, /Target X -5\.0, Y 35\.0, Z [12]\.\d descends onto the \[UNRELIABLE\] eraser measurement/);
  }
  Assert.deepEqual(ApproveDuringRefusals, [0, 0]);
  Assert.equal(await Approve.count(), 0);
  // The NEXT STEP carries the wrist pitch the app verified: from afar with the
  // jaws closed it opens the gripper and goes to the hover, as ONE chained motion (lib/AiChain.ts).
  Assert.match(Completions[Asked].Text, /NEXT STEP of the grasp [^\n]*open the gripper to 80% and (?:go to the HOVER POINT|hover above it)[^\n]*as ONE chained motion \(the gripper opens while the tip moves\): "motion_proposal":\{"target_cm":\{[^}]*\},"wrist_pitch_degrees":-[\d.]+,"gripper_before":0\.8\}/);

  // 16:10: a task noun that named no located object ("thingon") switched the
  // descent rule off. It guards every [UNRELIABLE] measurement in the frame.
  ParkHeldTarget();
  Locate = { ...Measured, objects: [Measured.objects[0], { ...Measured.objects[1], label: "small black object", x: -8, y: 34, top_z: -1.3, extrapolated: true }] };
  const ApproveDuringOther = [];
  Replies.push({ response_to_user: "Onto it.", task_status: "continue", motion_proposal: { target_cm: { x: -8, y: 34, z: 0 } } },
    async () => { ApproveDuringOther.push(await Approve.count());
      return { response_to_user: "Which object do you mean?", task_status: "ask", motion_proposal: null }; });
  Asked = Completions.length;
  await Submit("Grab the thingon");
  await WaitFor(async () => /Which object do you mean\?/.test(await Reply.innerText()), "the refusal, then the model asks");
  Assert.equal(Completions.length, Asked + 2);
  // "thingon" is read as "thing on": a generic noun no label names, so the model is given the candidates.
  Assert.match(Completions[Asked].Text, /TASK OBJECT "thing": no measured label names it\. Candidates measured this check-in: [^\n]*small black object \(X -8\.0, Y 34\.0\)/);
  Assert.match(Completions.at(-1).Text, /Target X -8\.0, Y 34\.0, Z 0\.0 descends onto the \[UNRELIABLE\] small black object measurement/);
  Assert.deepEqual(ApproveDuringOther, [0]);
  Assert.equal(await Approve.count(), 0);
  Locate = Measured;

  // 16:05, "Grab the black thingon the desk": /locate is told the task's
  // object as the app read it, and the bridge's task query picks the USB
  // drive. At the check-in the same drive, renamed "small black object", is
  // tracked at its spot instead of reported lost.
  ParkHeldTarget();
  const Drive = { label: "USB drive", x: -5, y: 35, z: -2.3, top_z: -0.1, height_cm: 2.2, size_cm: [2, 1.4], method: "depth",
    support_measured: true, support_z_measured: -2.3, valid_fraction: 0.9, support_branch: "desk" };
  Locate = { ...Measured, objects: [Measured.objects[1], { ...Drive, task_match: true }] };
  Replies.push({ response_to_user: "Raising first.", task_status: "continue", motion_proposal: { move_delta_cm: { left: 0, farther: 0, up: 2 } } },
    { response_to_user: "Still on the desk.", task_status: "done", motion_proposal: null });
  Asked = Completions.length;
  let Bodies = LocateBodies.length;
  Clicked = Date.now();
  await Submit("Grab the black thingon the desk");
  await Approve.waitFor();
  Assert.deepEqual(LocateBodies[Bodies].task_object, { phrase: "black thing on the desk", label: "black thing" });
  Assert.match(Completions[Asked].Text, /TASK OBJECT "black thing": detected this check-in \(measured as "USB drive", picked by the task query\) at X -5\.0, Y 35\.0/);
  Assert.match(Completions[Asked].Text, /- USB drive: [^\n]* <- the task object\n/);
  Locate = { ...Measured, objects: [Measured.objects[1], { ...Drive, label: "small black object", x: -4.8, y: 35.2 }] };
  await Approve.click();
  await WaitFor(async () => /Still on the desk\./.test(await Reply.innerText()), "the check-in reaches the model");
  Assert.deepEqual(LocateBodies.at(-1).task_object, { phrase: "black thing on the desk", label: "black thing" }, "Check-ins send it too");
  Assert.match(Completions.at(-1).Text, /TASK OBJECT "black thing": detected this check-in \(measured as "small black object", tracked at the same spot\) at X -4\.8, Y 35\.2/);
  Assert.ok(!Stops().some(Stop => Stop.At >= Clicked), "A renamed object is not lost");
  Assert.deepEqual(Events.filter(Event => Event.type === "task_object_bound" && Event.At >= Clicked).map(Event => [Event.label, Event.measured_label, Event.source]),
    [["black thing", "USB drive", "query"], ["black thing", "small black object", "position"]]);
  Assert.equal(Events.filter(Event => Event.type === "vision" && Event.At >= Clicked).at(-1).task_object.phrase, "black thing on the desk");
  await WaitFor(async () => await Page.locator(".ai-task-strip").count() === 0, "the task finishes");
  Locate = Measured;

  // "Grab the black thing" where no label says black: the model is given the
  // candidates, and its target_estimate_cm on one binds it right after the
  // reply is read, before any check, so Approve checks the hover above that
  // [UNRELIABLE] object against the same objects the reply was checked with.
  ParkHeldTarget();
  Locate = { ...Measured, objects: [Measured.objects[1], { ...Drive, label: "usb drive", extrapolated: true }] };
  Replies.push({ response_to_user: "Above the usb drive.", task_status: "continue", target_estimate_cm: { left: -5, farther: 35, up: -0.1 },
    motion_proposal: { target_cm: { x: -5, y: 35, z: 4.7 } } },
  { response_to_user: "Above it now.", task_status: "done", motion_proposal: null });
  Asked = Completions.length;
  Bodies = LocateBodies.length;
  Clicked = Date.now();
  await Submit("Grab the black thing");
  await Approve.waitFor();
  Assert.deepEqual(LocateBodies[Bodies].task_object, { phrase: "black thing", label: "black thing" });
  Assert.match(Completions[Asked].Text, /TASK OBJECT "black thing": no measured label names it\. Candidates measured this check-in: usb drive \(X -5\.0, Y 35\.0\), cup \(X 0\.2, Y 40\.8\)\. If one of them is it, set target_estimate_cm to its measured centre/);
  Assert.deepEqual(Events.filter(Event => Event.type === "task_object_bound" && Event.At >= Clicked).map(Event => [Event.measured_label, Event.source]),
    [["usb drive", "estimate"]]);
  const ReplyChecks = Events.filter(Event => Event.type === "reply" && Event.At >= Clicked).map(Event => Event.possible);
  Assert.ok(ReplyChecks.length >= 1 && ReplyChecks.every(Possible => Possible === true), `The hover above it passes at the reply: ${ReplyChecks}`);
  Sends = Commands.filter(Command => Command.type === "send_ik").length;
  await Approve.click();
  await WaitFor(async () => Commands.filter(Command => Command.type === "send_ik").length > Sends, "the hover is sent");
  Assert.ok(!Events.some(Event => Event.type === "approve_refused" && Event.At >= Clicked), "Approve agrees with the reply-time check");
  await WaitFor(async () => /Above it now\./.test(await Reply.innerText()), "the check-in reaches the model");
  Assert.match(Completions.at(-1).Text, /TASK OBJECT "black thing": detected this check-in \(measured as "usb drive", tracked at the same spot\) at X -5\.0, Y 35\.0[^\n]*\[UNRELIABLE/);
  await WaitFor(async () => await Page.locator(".ai-task-strip").count() === 0, "the task finishes");
  Locate = Measured;

  // Checked again at approval: the arm reached the proposed point meanwhile
  // (the bridge holds it and the tip is there), so it is not sent, not
  // counted, and the model gets one corrective turn.
  ParkHeldTarget();
  let Proposed = null;
  Replies.push(Text => { const Pose = CurrentPose(Text); Proposed = { x: Pose.x, y: Pose.y + 3, z: Pose.z, Pitch: Pose.pitch };
    return { response_to_user: "Reaching farther.", task_status: "continue",
      motion_proposal: { target_cm: { x: Proposed.x, y: Proposed.y, z: Proposed.z }, wrist_pitch_degrees: Proposed.Pitch } }; },
  { response_to_user: "The arm is already there.", task_status: "done", motion_proposal: null });
  await Submit("Grab the eraser");
  await Approve.waitFor();
  const Offset = State.workspace_offset;
  State.target_cm = { x: -(Proposed.x - Offset.x), y: Proposed.y - Offset.y, z: Proposed.z - Offset.z }; State.wrist_pitch = Proposed.Pitch;
  State.joint_positions = HomeJoints.map((Point, Index, All) => Index === All.length - 1 ? { ...Point, y: Point.y + 3 } : Point);
  await Pause(400);
  Asked = Completions.length;
  Sends = Commands.filter(Command => Command.type === "send_ik").length;
  await Approve.click();
  await WaitFor(async () => /The arm is already there/.test(await Reply.innerText()), "the corrective turn is answered");
  Assert.equal(Completions.length, Asked + 1, "Exactly one corrective turn");
  Assert.match(Completions.at(-1).Text, /would not move the arm\. It was not sent and not counted/);
  Assert.equal(Commands.filter(Command => Command.type === "send_ik").length, Sends, "A no-op is never sent");
  Assert.ok(Events.some(Event => Event.type === "approve_refused" && Event.no_op === true));
  State.joint_positions = structuredClone(HomeJoints);
  await Pause(300);

  // The same impossible target asked for again at a later check-in stops the
  // task instead of revising it again (the incident's phantom eraser).
  ParkHeldTarget();
  const Phantom = { target_cm: { x: 16.1, y: 35.8, z: 5 }, wrist_pitch_degrees: -24.1694 };
  Replies.push({ response_to_user: "To the eraser.", task_status: "continue", motion_proposal: Phantom },
    { response_to_user: "Up first, then.", task_status: "continue", motion_proposal: { move_delta_cm: { left: 0, farther: 0, up: 2 } } },
    { response_to_user: "To the eraser again.", task_status: "continue", motion_proposal: Phantom });
  await Submit("Grab the eraser");
  await Approve.waitFor();
  await Approve.click();
  await WaitFor(async () => /already rejected at an earlier check-in/.test(await Reply.innerText()), "a target rejected at two check-ins stops the task");
  Assert.match(Stops().at(-1).reason, /asked again for X 16\.1, Y 35\.8, Z 5\.0/);
  Assert.equal(await Approve.count(), 0);
  Assert.equal(await Page.locator(".ai-task-strip").count(), 0);

  // An approved motion that cannot be sent (torque off) stops the task.
  ParkHeldTarget();
  Replies.push({ response_to_user: "Up a little.", task_status: "continue", motion_proposal: { move_delta_cm: { left: 0, farther: 0, up: 2 } } });
  await Submit("Grab the eraser");
  await Approve.waitFor();
  State.torque = false;
  await Pause(400);
  Sends = Commands.filter(Command => Command.type === "send_ik").length;
  await Approve.click();
  await WaitFor(async () => /the approved motion could not be sent/.test(await Reply.innerText()), "an unsendable motion stops the task");
  Assert.equal(Commands.filter(Command => Command.type === "send_ik").length, Sends);
  Assert.equal(await Page.locator(".ai-task-strip").count(), 0);
  State.torque = true;
  await Pause(300);

  // The bridge accepts the motion but never starts it: the task stops at the
  // start limit (12 s here) instead of reading frozen state as arrival.
  ParkHeldTarget(); NeverStart = true;
  Replies.push({ response_to_user: "Up a little.", task_status: "continue", motion_proposal: { move_delta_cm: { left: 0, farther: 0, up: 2 } } });
  await Submit("Grab the eraser");
  await Approve.waitFor();
  Before = CheckIns().length;
  Clicked = Date.now();
  await Approve.click();
  await WaitFor(async () => /the bridge did not start the motion within 12 s of approval/.test(await Reply.innerText()), "a motion that never starts stops the task", 25000);
  Assert.ok(Date.now() - Clicked >= 11500);
  Assert.equal(CheckIns().length, Before);
  NeverStart = false;

  // A task object lost far from the gripper (not hidden under it) stops the
  // task at the check-in, before the model chases its old coordinates.
  ParkHeldTarget();
  Replies.push({ response_to_user: "Up a little.", task_status: "continue", motion_proposal: { move_delta_cm: { left: 0, farther: 0, up: 2 } } });
  await Submit("Grab the eraser");
  await Approve.waitFor();
  Locate = { ...Measured, objects: [Measured.objects[1]] };
  Asked = Completions.length;
  await Approve.click();
  await WaitFor(async () => /has not been detected since/.test(await Reply.innerText()), "a lost task object stops the task");
  Assert.match(Stops().at(-1).reason, /the eraser has not been detected since \d\d:\d\d:\d\d, and the gripper is \d+\.\d cm from where it was last measured/);
  Assert.equal(Completions.length, Asked, "No model call for that check-in");
  Locate = Measured;

  // After the gripper closed, the object may be in the jaws and travel with
  // the tip: missing far from where it was seen is then not a stop.
  ParkHeldTarget();
  const GripperState = State.servos.gripper;
  Object.assign(GripperState, { target_raw: GripperState.range_max % 4096, present_raw: GripperState.range_max % 4096 });
  await Pause(300);
  Replies.push({ response_to_user: "Closing on the eraser.", task_status: "continue", motion_proposal: { direct_servo_targets: [{ name: "gripper", target_fraction: 0 }] } },
    { response_to_user: "Holding it.", task_status: "done", motion_proposal: null });
  await Submit("Grab the eraser");
  await Approve.waitFor();
  Locate = { ...Measured, objects: [Measured.objects[1]] };
  Clicked = Date.now();
  await Approve.click();
  await WaitFor(async () => /Holding it\./.test(await Reply.innerText()), "the check-in reaches the model");
  Assert.match(Completions.at(-1).Text, /TASK OBJECT "eraser": NOT detected this check-in[^\n]*RIGHT ARM when last measured: (graspable|grasp height)/);
  Assert.ok(!Stops().some(Stop => Stop.At >= Clicked), "No stop while the object may be in the jaws");
  Locate = Measured;

  // The AI target snap: a -90 wrist the arm cannot reach there becomes the
  // nearest pitch it reaches reliably. The card shows asked vs. sent in the
  // tiles' frame, Approve sends that pitch even after 2 ticks of servo jitter
  // (16:10 was refused after one), and the check-in tells the model.
  ParkHeldTarget();
  const HomeRaw = Object.fromEntries(Object.entries(State.servos).map(([Name, Servo]) => [Name, Servo.present_raw]));
  Replies.push({ response_to_user: "Above the eraser.", task_status: "continue", motion_proposal: { target_cm: { x: -5, y: 34, z: 3 }, wrist_pitch_degrees: -90 } },
    { response_to_user: "Done for now.", task_status: "done", motion_proposal: null });
  await Submit("Grab the eraser");
  await Approve.waitFor();
  const SnapNote = Page.locator(".ai-snap-note");
  await SnapNote.waitFor();
  const SnapText = await SnapNote.innerText();
  Assert.match(SnapText, /^Asked for X 5\.0 Y 34\.0 Z 3\.0, wrist -90\.0 -> will move to X 5\.0 Y 34\.0 Z 3\.0, wrist -\d+\.\d \(same point; wrist \d+\.\d deg flatter\)\. Why: at wrist -90 deg the arm is \d\.\d+ cm too short to reach that point\.$/);
  const SnappedWrist = Number(SnapText.match(/will move to .*, wrist (-\d+\.\d)/)[1]);
  Assert.ok(Events.some(Event => Event.type === "snap" && Event.kind === "pitch"), "The snap is recorded");
  for (const [Name, Tick] of [["shoulder_pan", 2], ["shoulder_lift", -2], ["elbow_flex", 1], ["wrist_flex", -2]]) State.servos[Name].present_raw += Tick;
  await Pause(400);
  Before = CheckIns().length;
  Sends = Commands.filter(Command => Command.type === "send_ik").length;
  Clicked = Date.now();
  await Approve.click();
  await WaitFor(async () => Commands.filter(Command => Command.type === "send_ik").length > Sends, "the snapped target is sent");
  Assert.equal(Number(Commands.filter(Command => Command.type === "send_ik").at(-1).target.wrist_pitch.toFixed(1)), SnappedWrist, "The pitch shown is the pitch sent");
  Assert.ok(!Events.some(Event => Event.type === "approve_refused" && Event.At >= Clicked), "No refusal after 2 ticks of jitter");
  await WaitFor(async () => CheckIns().length > Before, "the check-in after the snapped motion");
  await WaitFor(async () => /Done for now\./.test(await Reply.innerText()), "the check-in reaches the model");
  const SnapCheckIn = Completions.at(-1).Text;
  Assert.match(SnapCheckIn, /You asked for X -5\.0, Y 34\.0, Z 3\.0, wrist -90\.0; that is not reachable \(at wrist -90 deg the arm is \d\.\d+ cm too short to reach that point\), so the app sent the nearest possible target X -5\.0, Y 34\.0, Z 3\.0, wrist -\d+\.\d \(0\.0 cm from your target\)\./);
  Assert.match(SnapCheckIn, /do not ask for the same unreachable point again/);
  Assert.match(SnapCheckIn, /- Motion 1: tip to X -5, Y 34, Z 3 cm, wrist -\d+\.?\d* deg \(you asked for X -5\.0, Y 34\.0, Z 3\.0, wrist -90\.0: snapped 0\.0 cm and \d+\.\d deg of wrist/);
  const Approved = Events.filter(Event => Event.type === "approve").at(-1);
  Assert.equal(Approved.motion.requested_wrist_pitch_degrees, -90);
  Assert.equal(Approved.motion.snap.kind, "pitch");
  for (const [Name, Raw] of Object.entries(HomeRaw)) State.servos[Name].present_raw = Raw;
  await Pause(300);

  // The joint limits change between the proposal and Approve (the bridge
  // sends new ones): the app snaps the request again with them. A small
  // change is sent; a larger one is shown again for another approval.
  const Limits = structuredClone(State.ik_joint_limits);
  const PanLimit = Change => ({ ...Limits, shoulder_pan: [Limits.shoulder_pan[0], State.ik_angles.shoulder_pan - Change] });
  for (const [Change, Resent] of [[0.32, true], [1, false]]) {
    ParkHeldTarget();
    Replies.push({ response_to_user: "Up a little.", task_status: "continue", motion_proposal: { move_delta_cm: { left: 0, farther: 0, up: 2 } } },
      { response_to_user: "Stopping here.", task_status: "done", motion_proposal: null });
    await Submit("Grab the eraser");
    await WaitFor(async () => await Approve.count() > 0, `the proposal is offered (limit change ${Change})`);
    Assert.equal(await SnapNote.count(), 0, "An exact target shows no snap note");
    State.ik_joint_limits = PanLimit(Change);
    await Pause(400);
    Sends = Commands.filter(Command => Command.type === "send_ik").length;
    Clicked = Date.now();
    await Approve.click();
    if (!Resent) {
      await WaitFor(async () => /The arm's limits changed since this was proposed: the target moved/.test(await Page.locator(".ai-panel").innerText()), "the moved target is shown again");
      Assert.equal(Commands.filter(Command => Command.type === "send_ik").length, Sends, "Not sent before the second approval");
      await SnapNote.waitFor();
      Assert.match(await SnapNote.innerText(), /\(0\.\d\d cm away; /);
      await Approve.click();
    }
    await WaitFor(async () => Commands.filter(Command => Command.type === "send_ik").length > Sends, "the re-snapped target is sent");
    // The old limits again before the check-in, whose reach verdict for the eraser uses them.
    State.ik_joint_limits = Limits;
    Assert.ok(Events.some(Event => Event.type === "snap" && Event.resnapped === true && Event.At >= Clicked), "The re-snap is recorded");
    Assert.ok(!Events.some(Event => Event.type === "approve_refused" && Event.At >= Clicked));
    await WaitFor(async () => await Page.locator(".ai-task-strip").count() === 0, "the task finishes");
    Assert.match(Completions.at(-1).Text, /AUTO CHECK-IN/);
    await Pause(300);
  }

  // Approve's check fails and nothing near the request is reachable any more
  // (a joint's limits moved away from it): the task goes on with a check-in
  // saying the proposal was not sent, instead of waiting for an approval that
  // can never come (08:07 and 16:10 hung until Clear). No object is measured
  // here: the narrowed limits would put one out of reach and stop the task.
  ParkHeldTarget();
  Locate = { ...Measured, objects: [] };
  Replies.push({ response_to_user: "Up a little.", task_status: "continue", motion_proposal: { move_delta_cm: { left: 0, farther: 0, up: 2 } } },
    { response_to_user: "The arm cannot move there now. What changed?", task_status: "ask", motion_proposal: null });
  await Submit("Grab the eraser");
  await Approve.waitFor();
  State.ik_joint_limits = { ...Limits, shoulder_pan: [60, 61] };
  await Pause(400);
  Sends = Commands.filter(Command => Command.type === "send_ik").length;
  Clicked = Date.now();
  await Approve.click();
  await WaitFor(async () => /The arm cannot move there now/.test(await Reply.innerText()), "the model hears the proposal was not sent");
  Assert.match(Completions.at(-1).Text, /Your last proposal was not sent: .*The arm did not move\./);
  Assert.equal(Commands.filter(Command => Command.type === "send_ik").length, Sends, "Nothing was sent");
  Assert.ok(Events.some(Event => Event.type === "approve_refused" && Event.resnapped === true && Event.At >= Clicked));
  Assert.equal(await Page.locator(".ai-task-strip").count(), 0, "The model's ask ends the task");
  State.ik_joint_limits = Limits;
  Locate = Measured;
  await Pause(300);

  // A target too far from anything reachable is refused; the feedback gives
  // the nearest possible target and why the app did not use it, and the
  // model can ask for that target instead. The nearest one keeps every part
  // of the gripper clear of the cup beside it (lib/GraspPlan.ts).
  ParkHeldTarget();
  let NearestAsked = null;
  Replies.push({ response_to_user: "Far out.", task_status: "continue", motion_proposal: { target_cm: { x: -5, y: 46, z: 2 }, wrist_pitch_degrees: -90 } },
    Text => {
      const Match = Text.match(/Nearest possible target: X (-?[\d.]+) cm, Y (-?[\d.]+) cm, Z (-?[\d.]+) cm, wrist (-?[\d.]+) deg/);
      NearestAsked = Match && Match.slice(1).map(Number);
      return { response_to_user: "To the nearest point, then.", task_status: "continue",
        motion_proposal: NearestAsked ? { target_cm: { x: NearestAsked[0], y: NearestAsked[1], z: NearestAsked[2] }, wrist_pitch_degrees: NearestAsked[3] } : null };
    });
  Asked = Completions.length;
  await Submit("Grab the eraser");
  await WaitFor(async () => await Approve.count() > 0, "the revised target is offered");
  Assert.equal(Completions.length, Asked + 2, "One feedback turn");
  Assert.match(Completions.at(-1).Text, /Nearest possible target: X -?[\d.]+ cm, Y -?[\d.]+ cm, Z -?[\d.]+ cm, wrist -?[\d.]+ deg \([\d.]+ cm from yours\)\. The app did not use it because /);
  Assert.doesNotMatch(Completions.at(-1).Text, /straight line from the tip/);
  Assert.ok(NearestAsked, "The model read the nearest target");
  Assert.ok(Events.some(Event => Event.type === "snap" && Event.kind === "refused" && Event.code === "too-far"));
  await Page.getByRole("button", { name: "Block", exact: true }).click();
  await WaitFor(async () => await Page.locator(".ai-task-strip").count() === 0, "the block ends the task");

  // How a task ends. Every end goes through one path: a model or /locate call
  // in flight is aborted (its reply never becomes approvable), the chat says
  // why, and the record gets exactly one task_end with its source.
  const ReplyText = async () => (await Reply.count()) ? await Reply.innerText() : "";
  const TaskEnds = Since => Events.filter(Event => Event.type === "task_end" && Event.At >= Since);
  const ModelAborted = Since => Failed.some(Item => /\/ollama\/api\/chat/.test(Item.Url) && Item.At >= Since);
  const InputFree = async () => !(await Page.getByPlaceholder("Describe a task or movement").isDisabled());
  const Up = { response_to_user: "Up a little.", task_status: "continue", motion_proposal: { move_delta_cm: { left: 0, farther: 0, up: 2 } } };
  await WaitFor(InputFree, "the app is idle");

  // CHAINED MOTION (lib/AiChain.ts): with the jaws closed, the NEXT STEP opens the gripper and goes to the
  // eraser's HOVER POINT as ONE motion_proposal. One approval sends exactly two bridge commands, in order:
  // the gripper, then (once it has started and settled) the tip; the model is asked once, after both.
  {
    const ClosedRaw = 699, OpenRaw = 1922; // the gripper's 0.036 and 80% (642 + 0.8 x 1600)
    // The two legs, as with a bridge that takes no gripper target with a tip move (an opening then runs
    // first, as closes always do); the TOGETHER scenario after these restores it.
    const GripperWithTip = State.gripper_with_tip_version;
    delete State.gripper_with_tip_version;
    await Pause(300);
    const CloseGripper = async () => { Object.assign(State.servos.gripper, { present_raw: ClosedRaw, target_raw: ClosedRaw }); await Pause(400); };
    const MovesSince = At => Commands.filter(Command => Command.At >= At && ["move_servo", "send_ik", "go_home"].includes(Command.type));
    const Strip = Page.locator(".ai-task-strip");
    ParkHeldTarget();
    Locate = Measured;
    PlanMs = 300;
    await CloseGripper();
    const Chain = { Seen: null, CheckIn: "" };
    Replies.push(Text => {
      Chain.Seen = NextStep(Text);
      return { response_to_user: "Opening the gripper and going to the hover point.", task_status: "continue", motion_proposal: Chain.Seen };
    }, Text => {
      Chain.CheckIn = Text;
      return { response_to_user: "Above the eraser with the jaws open.", task_status: "done", motion_proposal: null };
    });
    const AskedChain = Completions.length;
    await Submit("Grab the eraser");
    await Approve.waitFor();
    Assert.ok(Chain.Seen?.gripper_before === 0.8 && Chain.Seen.target_cm && Chain.Seen.wrist_pitch_degrees < 0, JSON.stringify(Chain.Seen));
    Assert.equal(Completions.length, AskedChain + 1, "The NEXT STEP's chain passes every check: no feedback turn");
    // The card shows both legs, in the order they run.
    const Steps = await Page.locator(".ai-chain-steps li").allInnerTexts();
    Assert.equal(Steps.length, 2, Steps.join(" | "));
    Assert.match(Steps[0], /^Open the gripper to 80% \(raw 1922\)$/);
    Assert.match(Steps[1], new RegExp(`^Move the tip to X -?[\\d.]+ Y [\\d.]+ Z [\\d.]+ cm, wrist ${Chain.Seen.wrist_pitch_degrees.toFixed(1)} deg$`));
    Assert.match(await Page.locator(".ai-chain-steps").innerText(), /Chained motion: one approval runs both, in this order/);
    const ApprovedAt = Date.now();
    await Approve.click();
    await WaitFor(async () => /chained motion, part 1 of 2/.test(await Strip.innerText().catch(() => "")), "the strip waits on part 1");
    await WaitFor(async () => Completions.length > AskedChain + 1, "the one check-in after both legs", 25000);
    const Moves = MovesSince(ApprovedAt);
    Assert.deepEqual(Moves.map(Command => Command.type), ["move_servo", "send_ik"], "Exactly two bridge commands");
    Assert.equal(Moves[0].name, "gripper");
    Assert.equal(Moves[0].raw, OpenRaw);
    Assert.ok(Math.abs(Moves[1].target.wrist_pitch - Chain.Seen.wrist_pitch_degrees) < 0.05, JSON.stringify(Moves[1].target));
    Assert.ok(Moves[1].At - Moves[0].At >= PlanMs + 600, `The tip leg waited for the gripper to start and settle (${Moves[1].At - Moves[0].At} ms)`);
    Assert.equal(Completions.filter(Completion => Completion.At > ApprovedAt && Completion.At < Moves[1].At).length, 0, "No model call between the legs");
    Assert.equal(Completions.length, AskedChain + 2, "One model call for the chain, one check-in after it");
    Assert.ok(Completions.at(-1).At >= Moves[1].At + PlanMs + 600, "The check-in waits for the tip leg to settle");
    Assert.match(Chain.CheckIn, /AUTO CHECK-IN — motion 2 of 2 max/, "The chain counts as one motion");
    Assert.match(Chain.CheckIn, /- Motion 1 \(chained\): gripper to raw 1922, then tip to X /);
    Assert.match(Chain.CheckIn, /gripper now at raw 1922 \(commanded 1922\)\. It was a chained motion: first open the gripper to 80%, then move the tip to X -?[\d.]+, Y [\d.]+, Z [\d.]+ cm, wrist -[\d.]+ deg; both parts ran, in that order\./);
    const Legs = Events.filter(Event => Event.type === "chain_leg" && Event.At >= ApprovedAt);
    Assert.deepEqual(Legs.map(Event => [Event.leg, Event.kind, Event.result]), [[1, "gripper", "sent"], [2, "tip", "sent"]]);
    Assert.deepEqual(Events.find(Event => Event.type === "approve" && Event.At >= ApprovedAt).motion.chain, { order: "before", gripper_raw: OpenRaw, gripper_fraction: 0.8 });
    await WaitFor(async () => await Strip.count() === 0, "the model's done ends the task");

    // Stop task between the legs: the gripper leg was sent, the tip leg never is, and no check-in follows.
    ParkHeldTarget();
    await CloseGripper();
    Replies.push(Text => ({ response_to_user: "Opening the gripper and going to the hover point.", task_status: "continue", motion_proposal: NextStep(Text) }));
    const AskedStop = Completions.length;
    await Submit("Grab the eraser");
    await Approve.waitFor();
    PlanMs = 1500;
    const StopAt = Date.now();
    await Approve.click();
    await WaitFor(async () => MovesSince(StopAt).some(Command => Command.type === "move_servo"), "the gripper leg is sent");
    await WaitFor(async () => /chained motion, part 1 of 2/.test(await Strip.innerText().catch(() => "")), "the strip waits on part 1");
    await Strip.getByRole("button", { name: "Stop task" }).click();
    await WaitFor(async () => /Stopped: Stop task was pressed\. .*The second part of the chained motion was not sent\./.test(await ReplyText()), "Stop task drops the second leg");
    await Pause(5000); // well past when the gripper leg settles
    Assert.deepEqual(MovesSince(StopAt).map(Command => Command.type), ["move_servo"], "The tip leg is never sent");
    Assert.equal(Completions.length, AskedStop + 1, "No check-in after the stop");
    Assert.ok(Events.some(Event => Event.type === "chain_aborted" && Event.At >= StopAt && Event.source === "stop_task" && Event.part === 1 && Event.dropped === "tip"));
    Assert.deepEqual(TaskEnds(StopAt).map(Event => Event.source), ["stop_task"]);
    Assert.equal(await Strip.count(), 0);
    PlanMs = 300;

    // The owner's ask, word for word: "open the gripper and move up 3 cm" (no object) is one chained motion on one
    // approval. The check-in after it is the task's last allowed motion (2 here): a chain there (close, then lift)
    // still runs both legs, and the task ends once the second is sent, with no further model call.
    ParkHeldTarget();
    await CloseGripper();
    const Last = { CheckIn: "" };
    Replies.push({ response_to_user: "Opening the gripper, then moving up 3 cm.", task_status: "continue",
      motion_proposal: { move_delta_cm: { left: 0, farther: 0, up: 3 }, gripper_before: 0.8 } },
    Text => {
      Last.CheckIn = Text;
      return { response_to_user: "Closing the gripper, then lifting 2 cm.", task_status: "continue",
        motion_proposal: { move_delta_cm: { left: 0, farther: 0, up: 2 }, gripper_before: 0 } };
    });
    const AskedOpen = Completions.length;
    await Submit("open the gripper and move up 3 cm");
    await Approve.waitFor();
    Assert.equal(Completions.length, AskedOpen + 1);
    Assert.deepEqual(await Page.locator(".ai-chain-steps li").allInnerTexts().then(Texts => Texts.map(Text => Text.split(" ")[0])), ["Open", "Move"]);
    const OpenAt = Date.now();
    await Approve.click();
    await WaitFor(async () => Completions.length > AskedOpen + 1 && await Approve.count() > 0, "the last motion is offered after the check-in", 25000);
    Assert.deepEqual(MovesSince(OpenAt).map(Command => [Command.type, Command.name ?? null, Command.raw ?? null]),
      [["move_servo", "gripper", OpenRaw], ["send_ik", null, null]], "The gripper, then the tip");
    Assert.match(Last.CheckIn, /AUTO CHECK-IN — motion 2 of 2 max/);
    Assert.match(Last.CheckIn, /It was a chained motion: first open the gripper to 80%, then move the tip to X -?[\d.]+, Y [\d.]+, Z [\d.]+ cm, wrist -?[\d.]+ deg; both parts ran, in that order\./);
    Assert.deepEqual(await Page.locator(".ai-chain-steps li").allInnerTexts().then(Texts => Texts.map(Text => Text.split(" ")[0])), ["Close", "Move"]);
    const LastAt = Date.now();
    await Approve.click();
    await WaitFor(async () => /Stopped: it used all 2 motions allowed for one task; the last one, a chained motion, was sent\./.test(await ReplyText()), "the task ends after the last chain's second leg", 15000);
    Assert.deepEqual(MovesSince(LastAt).map(Command => [Command.type, Command.name ?? null, Command.raw ?? null]),
      [["move_servo", "gripper", 642], ["send_ik", null, null]], "The close, then the lift");
    Assert.deepEqual(TaskEnds(LastAt).map(Event => Event.source), ["step_limit"]);
    await Pause(2500);
    Assert.equal(Completions.length, AskedOpen + 2, "No model call after the last motion");
    Assert.equal(await Strip.count(), 0);

    // How a chain fails. Each starts with the jaws closed: "open the gripper and move up 3 cm", one approval.
    const OpenUp = { response_to_user: "Opening the gripper, then moving up 3 cm.", task_status: "continue",
      motion_proposal: { move_delta_cm: { left: 0, farther: 0, up: 3 }, gripper_before: 0.8 } };
    const Done = { response_to_user: "Done.", task_status: "done", motion_proposal: null };
    const ChainEvents = (Since, Type) => Events.filter(Event => Event.type === Type && Event.At >= Since);
    // Approves OpenUp in a new task; Then(Text) answers the prompt after it (a check-in, or the refusal's re-prompt).
    const RunChain = async (Then = () => Done) => {
      ParkHeldTarget();
      await CloseGripper();
      const Seen = { After: "" };
      // A chain the task ended early left its check-in's reply unused.
      Replies.length = 0;
      Replies.push(OpenUp, Text => { Seen.After = Text; return Then(Text); });
      Seen.Asked = Completions.length;
      await Submit("open the gripper and move up 3 cm");
      await WaitFor(async () => await Approve.count() > 0, "the chain is offered for approval");
      Seen.At = Date.now();
      await Approve.click();
      return Seen;
    };

    // The bridge refuses the first part: nothing moved, the motion is not counted, and the model hears that neither part ran.
    RefuseWhen = Command => Command.type === "move_servo" && "Gripper route refused";
    let Seen = await RunChain();
    await WaitFor(async () => Completions.length > Seen.Asked + 1, "the re-prompt after the refusal");
    RefuseWhen = null;
    Assert.match(Seen.After, /AUTO CHECK-IN — motion 1 of 2 max/, "Rewound: the refused chain is not counted");
    Assert.match(Seen.After, /Your last approved motion \(motion 1\) was NOT executed: the bridge refused it: Gripper route refused\. The arm did not move\. It was a chained motion: first open the gripper to 80%, then move the tip to X [^;]*; neither part ran: the first \(open the gripper to 80%\) was refused by the bridge\./);
    await WaitFor(async () => await Strip.count() === 0, "the model's done ends the task");
    Assert.deepEqual(MovesSince(Seen.At).map(Command => Command.type), ["move_servo"], "The tip part is never sent");
    Assert.equal(ChainEvents(Seen.At, "chain_aborted").length, 1, "One abort recorded");

    // The bridge refuses the second part: the first ran, so the motion counts, and the one check-in says so.
    RefuseWhen = Command => Command.type === "send_ik" && "Target is outside the reachable workspace";
    Seen = await RunChain();
    await WaitFor(async () => Completions.length > Seen.Asked + 1, "the check-in after the refused second part", 25000);
    RefuseWhen = null;
    Assert.match(Seen.After, /AUTO CHECK-IN — motion 2 of 2 max/);
    Assert.match(Seen.After, /It was a chained motion: first open the gripper to 80%, then move the tip to X [^;]*; only the first part ran \(open the gripper to 80%\): the second \(move the tip to X [^)]*\) was refused by the bridge: the bridge refused it: Target is outside the reachable workspace\./);
    await WaitFor(async () => await Strip.count() === 0, "the model's done ends the task");
    Assert.deepEqual(MovesSince(Seen.At).map(Command => Command.type), ["move_servo", "send_ik"]);

    // The first part does not get where it was sent (the gripper's route ends 300 raw short of 80%): the second is not
    // sent, and the check-in says why.
    GripperShortRaw = 300;
    Seen = await RunChain();
    await WaitFor(async () => Completions.length > Seen.Asked + 1, "the check-in after the first part fell short", 25000);
    GripperShortRaw = 0;
    Assert.match(Seen.After, /only the first part ran \(open the gripper to 80%\): the second \(move the tip to X [^)]*\) was not sent: the gripper stopped at 61% open, not at the 80% it was sent to\./);
    await WaitFor(async () => await Strip.count() === 0, "the model's done ends the task");
    Assert.deepEqual(MovesSince(Seen.At).map(Command => Command.type), ["move_servo"], "No send_ik after a short first part");
    Assert.deepEqual(ChainEvents(Seen.At, "chain_leg").map(Event => [Event.leg, Event.result]), [[1, "sent"], [2, "not-sent"]]);

    // Coast while the first part runs, then Hold (the arm moved by hand meanwhile): the task ends at Coast, and the
    // second part never follows the Hold. The same for a Coast the app did not ask for (a torque-off frame), and a
    // new motion epoch (a slider, Home, or a stop at the bridge).
    PlanMs = 1500;
    Seen = await RunChain();
    await WaitFor(async () => MovesSince(Seen.At).some(Command => Command.type === "move_servo"), "the gripper part is sent");
    await Page.getByRole("button", { name: "Coast", exact: true }).click();
    await WaitFor(async () => /Stopped: Coast was pressed while the first part of a chained motion ran\. Check where the arm is, then send the task again\. The second part of the chained motion was not sent\./.test(await ReplyText()), "Coast ends the task");
    await WaitFor(async () => State.torque === false, "the bridge coasts");
    State.servos.shoulder_lift.present_raw += 180;
    await Pause(1500);
    await Page.getByRole("button", { name: "Hold", exact: true }).click();
    await WaitFor(async () => State.torque === true, "the bridge holds");
    await Pause(4000);
    State.servos.shoulder_lift.present_raw -= 180; State.servos.shoulder_lift.target_raw = State.servos.shoulder_lift.present_raw;
    Assert.deepEqual(MovesSince(Seen.At).map(Command => Command.type), ["move_servo"], "Nothing moves after the Hold");
    Assert.equal(Completions.length, Seen.Asked + 1, "No model call after Coast");
    Assert.deepEqual(TaskEnds(Seen.At).map(Event => Event.source), ["chain_interrupted"]);
    Assert.deepEqual(ChainEvents(Seen.At, "chain_aborted").map(Event => [Event.source, Event.dropped]), [["chain_interrupted", "tip"]]);
    for (const [Name, Interrupt, Said] of [
      ["torque off at the bridge", () => { State.torque = false; }, /Stopped: the arm's torque went off \(Coast\) while the first part of a chained motion ran\./],
      ["a new motion epoch", () => { State.motion_epoch += 1; }, /Stopped: the arm's motion was stopped or replaced \(Hold, Home, Stop or a slider\) while the first part of a chained motion ran\./]]) {
      Seen = await RunChain();
      await WaitFor(async () => /chained motion, part 1 of 2/.test(await Strip.innerText().catch(() => "")) && MotionStartedAt >= Seen.At, `${Name}: the first part runs`);
      await Pause(400); // started (its epoch seen), still moving (the mock motion runs 700 ms), not yet settled
      Interrupt();
      await WaitFor(async () => Said.test(await ReplyText()), `${Name} ends the task`);
      State.torque = true;
      await Pause(3000);
      Assert.deepEqual(MovesSince(Seen.At).map(Command => Command.type), ["move_servo"], `${Name}: the tip part is never sent`);
      Assert.equal(Completions.length, Seen.Asked + 1, `${Name}: no model call`);
      Assert.deepEqual(TaskEnds(Seen.At).map(Event => Event.source), ["chain_interrupted"], Name);
    }
    PlanMs = 300;

    // The bridge never starts the first part: the task stops at the start limit, and the second part is never sent.
    NeverStart = true;
    Seen = await RunChain();
    await WaitFor(async () => /Stopped: the bridge did not start the first part of the chained motion within 12 s of approval\./.test(await ReplyText()), "the start limit", 20000);
    NeverStart = false;
    await Pause(1000);
    Assert.deepEqual(MovesSince(Seen.At).map(Command => Command.type), ["move_servo"]);
    Assert.equal(Completions.length, Seen.Asked + 1);
    Assert.deepEqual(TaskEnds(Seen.At).map(Event => Event.source), ["motion_not_started"]);

    // The task's last allowed motion (2 here) is a chain (close, then lift) whose first part the bridge refuses: the
    // task ends there with the reason (no rewind to the motion before it, no motion past the limit), the abort recorded once.
    RefuseWhen = Command => Command.type === "move_servo" && Command.raw === 642 && "Gripper route refused";
    Seen = await RunChain(() => ({ response_to_user: "Closing the gripper, then lifting 2 cm.", task_status: "continue",
      motion_proposal: { move_delta_cm: { left: 0, farther: 0, up: 2 }, gripper_before: 0 } }));
    await WaitFor(async () => Completions.length > Seen.Asked + 1 && await Approve.count() > 0, "the last motion is offered", 25000);
    const RefusedAt = Date.now();
    await Approve.click();
    const Refused = /Stopped: the bridge refused the first part of the last motion allowed for this task, a chained motion: Gripper route refused\. Nothing moved, and its second part was not sent\. Check the arm and the bridge status, then send the task again\./;
    await WaitFor(async () => Refused.test(await ReplyText()), "the refusal ends the task", 15000);
    RefuseWhen = null;
    await Pause(2500);
    Assert.equal(Completions.length, Seen.Asked + 2, "No model call after the refused last motion");
    Assert.deepEqual(TaskEnds(Seen.At).map(Event => Event.source), ["bridge_refused"]);
    Assert.equal(ChainEvents(RefusedAt, "chain_aborted").length, 1, "The abort is recorded once");
    Assert.deepEqual(ChainEvents(RefusedAt, "bridge_refused").map(Event => [Event.step, Event.part]), [[2, 1]]);
    Assert.deepEqual(MovesSince(Seen.At).map(Command => [Command.type, Command.raw ?? null]), [["move_servo", OpenRaw], ["send_ik", null], ["move_servo", 642]]);
    Assert.equal(await Strip.count(), 0);

    // gripper_after (a release: the move, then the gripper): with the reported tip at the move's target the gripper
    // follows; stopped 2.5 cm short of it (more than ChainRules.ArrivedCm), it does not, and the check-in says why.
    const HomeTip = structuredClone(State.joint_positions);
    TipFollows = true;
    for (const [ShortCm, Expected] of [[0, /; both parts ran, in that order\./], [2.5, /; only the first part ran \(move the tip to X [^)]*\): the second \(open the gripper to 80%\) was not sent: the tip stopped 2\.5 cm from that part's target \(more than 1\.5 cm\)\./]]) {
      TipShortCm = ShortCm;
      ParkHeldTarget();
      await CloseGripper();
      const After = { Text: "" };
      Replies.push({ response_to_user: "Moving up 3 cm, then opening the gripper.", task_status: "continue",
        motion_proposal: { move_delta_cm: { left: 0, farther: 0, up: 3 }, gripper_after: 0.8 } }, Text => { After.Text = Text; return Done; });
      const Asked = Completions.length;
      await Submit("move up 3 cm and then open the gripper");
      await Approve.waitFor();
      Assert.deepEqual(await Page.locator(".ai-chain-steps li").allInnerTexts().then(Texts => Texts.map(Text => Text.split(" ")[0])), ["Move", "Open"]);
      const At = Date.now();
      await Approve.click();
      await WaitFor(async () => Completions.length > Asked + 1, `gripper_after, ${ShortCm} cm short: the check-in`, 25000);
      Assert.match(After.Text, /It was a chained motion: first move the tip to X [^,]*, Y [^,]*, Z [^,]* cm, wrist -?[\d.]+ deg, then open the gripper to 80%/);
      Assert.match(After.Text, Expected, `${ShortCm} cm short`);
      Assert.deepEqual(MovesSince(At).map(Command => Command.type), ShortCm ? ["send_ik"] : ["send_ik", "move_servo"]);
      await WaitFor(async () => await Strip.count() === 0, "the model's done ends the task");
      State.joint_positions = structuredClone(HomeTip);
      await Pause(300);
    }
    TipFollows = false; TipShortCm = 0;

    // TOGETHER (lib/AiChain.ts): with a bridge that takes a gripper target with a tip move, an opening
    // gripper_before is ONE send_ik carrying it, and the route opens the jaws while the tip moves
    // (2026-10-03 14:46: sent as two legs, the still-settling gripper got the tip move refused).
    Assert.equal(GripperWithTip, "open-while-moving-v1");
    State.gripper_with_tip_version = GripperWithTip;
    await Pause(300);
    {
      ParkHeldTarget();
      await CloseGripper();
      const Together = { CheckIn: "" };
      Replies.length = 0;
      Replies.push(OpenUp, Text => { Together.CheckIn = Text; return Done; });
      const Asked = Completions.length;
      await Submit("open the gripper and move up 3 cm");
      await Approve.waitFor();
      const Steps = await Page.locator(".ai-chain-steps li").allInnerTexts();
      Assert.equal(Steps.length, 1, Steps.join(" | "));
      Assert.match(Steps[0], /^Open the gripper to 80% \(raw 1922\) while moving the tip to X -?[\d.]+ Y [\d.]+ Z [\d.]+ cm, wrist -?[\d.]+ deg$/);
      Assert.match(await Page.locator(".ai-chain-steps").innerText(), /Chained motion: one approval, one motion: the gripper opens while the tip moves/);
      const At = Date.now();
      await Approve.click();
      await WaitFor(async () => Completions.length > Asked + 1, "the check-in after the one motion", 25000);
      Assert.deepEqual(MovesSince(At).map(Command => [Command.type, Command.gripper_raw ?? null]), [["send_ik", OpenRaw]],
        "One bridge command: the tip move carrying the opening");
      Assert.match(Together.CheckIn, /AUTO CHECK-IN — motion 2 of 2 max/);
      Assert.match(Together.CheckIn, /- Motion 1 \(chained, one motion\): tip to X [^\n]*, the gripper to raw 1922 on the way/);
      Assert.match(Together.CheckIn, /gripper now at raw 1922 \(commanded 1922\)\. It was a chained motion, run as ONE motion: open the gripper to 80% while moving the tip to X -?[\d.]+, Y [\d.]+, Z [\d.]+ cm, wrist -?[\d.]+ deg; the gripper opened on the way\./);
      Assert.deepEqual(ChainEvents(At, "chain_leg").map(Event => [Event.leg, Event.of, Event.kind, Event.result, Event.gripper_raw]), [[1, 1, "together", "sent", OpenRaw]]);
      Assert.ok(ChainEvents(At, "chain_leg")[0].target_cm, "The record keeps the tip target too");
      Assert.equal(ChainEvents(At, "approve")[0].together, true);
      await WaitFor(async () => await Strip.count() === 0, "the model's done ends the task");
      // Refused by the bridge: nothing moved, and the model hears the one motion did not run.
      RefuseWhen = Command => Command.type === "send_ik" && "No checked route";
      const Refused = await RunChain();
      await WaitFor(async () => Completions.length > Refused.Asked + 1, "the re-prompt after the refusal");
      RefuseWhen = null;
      Assert.match(Refused.After, /was NOT executed: the bridge refused it: No checked route\. The arm did not move\. It was a chained motion, run as ONE motion: open the gripper to 80% while moving the tip to X [^;]*; it did not run: it was refused by the bridge\./);
      await WaitFor(async () => await Strip.count() === 0, "the model's done ends the task");
      Assert.deepEqual(MovesSince(Refused.At).map(Command => Command.type), ["send_ik"]);
    }
  }

  // 07:57: a latched bridge motion fault while the model was thinking ended
  // the task without a word, and the reply was dropped 2 s later. Now the
  // call is aborted, the bridge's text is shown, and it is recorded once, not
  // on every 30 Hz state frame while the fault stays latched.
  ParkHeldTarget();
  Replies.push(HoldReply(Up));
  Asked = Completions.length;
  let Since = Date.now();
  await Submit("Grab the eraser");
  await WaitFor(async () => Completions.length > Asked, "the model call is in flight");
  State.motion_fault = "shoulder_lift drifted 600 steps past its hold point - re-hold manually";
  await WaitFor(async () => /Stopped: the bridge reported a motion fault: shoulder_lift drifted 600 steps past its hold point/.test(await ReplyText()), "the fault stops the task with its reason", 6000);
  await Pause(1500);
  ReleaseHeld();
  await Pause(800);
  Assert.equal(await Approve.count(), 0, "The late reply is never approvable");
  Assert.equal(await Page.locator(".ai-task-strip").count(), 0);
  Assert.ok(ModelAborted(Since), "The model call was aborted");
  Assert.deepEqual(TaskEnds(Since).map(Event => Event.source), ["motion_fault"], "One task_end, not one per state frame");
  Assert.match(TaskEnds(Since)[0].reason, /drifted 600 steps/);
  Assert.equal(Events.filter(Event => Event.type === "bridge_fault" && Event.At >= Since).length, 1, "The fault is recorded once");
  // While it stays latched a new task is refused with the reason, before any capture.
  let Located = LocateCalls;
  Asked = Completions.length;
  await Submit("Grab the eraser");
  await WaitFor(async () => /Not started: the bridge reports a motion fault: shoulder_lift drifted/.test(await ReplyText()), "a prompt during a latched fault is refused", 6000);
  Assert.equal(LocateCalls, Located);
  Assert.equal(Completions.length, Asked);
  State.motion_fault = null;
  await Pause(300);

  // Stop and hold while the model is thinking: the same, with its own source.
  ParkHeldTarget();
  Replies.push(HoldReply(Up));
  Asked = Completions.length;
  Since = Date.now();
  await Submit("Grab the eraser");
  await WaitFor(async () => Completions.length > Asked, "the model call is in flight");
  await Page.getByRole("button", { name: "Stop and hold", exact: true }).click();
  await WaitFor(async () => /Stopped: Stop and hold was pressed/.test(await ReplyText()), "Stop and hold says why the task ended", 6000);
  ReleaseHeld();
  await Pause(800);
  Assert.equal(await Approve.count(), 0);
  Assert.ok(ModelAborted(Since), "The model call was aborted");
  Assert.deepEqual(TaskEnds(Since).map(Event => Event.source), ["stop_and_hold"]);
  Assert.ok(Commands.some(Command => Command.type === "stop_motion" && Command.At >= Since));

  // Stop task while the model is thinking: the call is aborted, the input is
  // free at once, and the reply arriving later never becomes approvable.
  ParkHeldTarget();
  Replies.push(HoldReply(Up));
  Asked = Completions.length;
  Since = Date.now();
  await Submit("Grab the eraser");
  await WaitFor(async () => Completions.length > Asked, "the model call is in flight");
  await Page.getByRole("button", { name: "Stop task", exact: true }).click();
  await WaitFor(InputFree, "the input is free again", 3000);
  ReleaseHeld();
  await Pause(1000);
  Assert.equal(await Approve.count(), 0, "The reply that arrived after Stop is not approvable");
  Assert.ok(ModelAborted(Since), "The model call was aborted");
  Assert.deepEqual(TaskEnds(Since).map(Event => Event.source), ["stop_task"]);
  Assert.match(await ReplyText(), /Stopped: Stop task was pressed/);

  // Stop task while a proposal waits for approval: nothing stays approvable.
  ParkHeldTarget();
  Replies.push(Up);
  await Submit("Grab the eraser");
  await Approve.waitFor();
  const RestPose = CurrentPose(Completions.at(-1).Text);
  Since = Date.now();
  await Page.getByRole("button", { name: "Stop task", exact: true }).click();
  await Pause(500);
  Assert.equal(await Approve.count(), 0);
  Assert.deepEqual(TaskEnds(Since).map(Event => Event.source), ["stop_task"]);

  // Reset AI context while the model is thinking: the call is aborted, the task
  // ends once as context_reset, the notepad is cleared and the chat says so.
  ParkHeldTarget();
  Replies.push(HoldReply(Up));
  Asked = Completions.length;
  Since = Date.now();
  await Submit("Grab the eraser");
  await WaitFor(async () => Completions.length > Asked, "the model call is in flight");
  const NotepadsBefore = NotepadWrites.length;
  await Page.getByRole("button", { name: "Reset AI context", exact: true }).click();
  await WaitFor(InputFree, "the input is free again", 3000);
  ReleaseHeld();
  await Pause(1000);
  Assert.equal(await Approve.count(), 0, "The reply that arrived after the reset is not approvable");
  Assert.ok(ModelAborted(Since), "The model call was aborted");
  Assert.deepEqual(TaskEnds(Since).map(Event => Event.source), ["context_reset"]);
  Assert.ok(NotepadWrites.slice(NotepadsBefore).some(Body => Body.content === ""), "The notepad is cleared");
  Assert.match(await ReplyText(), /Context reset/);
  Assert.ok(Events.some(Event => Event.type === "context_reset" && Event.At >= Since), "The reset is recorded");

  // 22:46: a task sent while the arm was still folding home read its pose at
  // submit, mid-motion. Now it waits for the arm to be still and reads the
  // pose right before the prompt is built.
  State.motion_route = { stage: 1, stages: 2 };
  for (const Servo of Object.values(State.servos)) Servo.moving = true;
  State.joint_positions = HomeJoints.map((Point, Index, All) => Index === All.length - 1 ? { ...Point, x: Point.x + 4 } : Point);
  await Pause(300);
  Replies.push({ response_to_user: "Seen at rest.", task_status: "done", motion_proposal: null });
  Located = LocateCalls;
  Asked = Completions.length;
  await Submit("Describe the workspace; do not move.");
  await Pause(1500);
  Assert.equal(LocateCalls, Located, "Nothing is captured while the arm moves");
  State.motion_route = null; State.joint_positions = structuredClone(HomeJoints);
  for (const Servo of Object.values(State.servos)) Servo.moving = false;
  await WaitFor(async () => /Seen at rest\./.test(await ReplyText()), "the task runs once the arm stops");
  const SeenPose = CurrentPose(Completions.at(-1).Text);
  Assert.ok(Math.abs(SeenPose.x - RestPose.x) < 0.05 && Math.abs(SeenPose.y - RestPose.y) < 0.05,
    `The prompt has the resting pose (X ${SeenPose.x}, Y ${SeenPose.y}; at rest X ${RestPose.x}, Y ${RestPose.y})`);
  Assert.equal(Completions.length, Asked + 1);

  // An arm that never stops: the task is refused with the reason, uncaptured.
  State.motion_route = { stage: 1, stages: 2 };
  for (const Servo of Object.values(State.servos)) Servo.moving = true;
  await Pause(300);
  Located = LocateCalls;
  Asked = Completions.length;
  Since = Date.now();
  await Submit("Grab the eraser");
  await WaitFor(async () => /Stopped: the arm was still moving 5 s after the task was sent/.test(await ReplyText()), "a task on a moving arm is refused", 12000);
  Assert.equal(LocateCalls, Located);
  Assert.equal(Completions.length, Asked);
  Assert.deepEqual(TaskEnds(Since).map(Event => Event.source), ["arm_moving"]);
  State.motion_route = null;
  for (const Servo of Object.values(State.servos)) Servo.moving = false;
  await Pause(300);

  // The arm moving between the frame and the prompt: the image no longer
  // shows where it is, so the step ends before the model call.
  LocateDelayMs = 1200;
  OnLocate = () => { State.joint_positions = HomeJoints.map((Point, Index, All) => Index === All.length - 1 ? { ...Point, x: Point.x + 3 } : Point); };
  Asked = Completions.length;
  Since = Date.now();
  await Submit("Grab the eraser");
  await WaitFor(async () => /Stopped: the arm moved 3\.0 cm while the camera image was measured/.test(await ReplyText()), "the image no longer matches the pose", 15000);
  Assert.equal(Completions.length, Asked);
  Assert.deepEqual(TaskEnds(Since).map(Event => Event.source), ["arm_moving"]);
  LocateDelayMs = 0; OnLocate = null; State.joint_positions = structuredClone(HomeJoints);
  await Pause(300);

  // A wrist-only correction at the same point is a new motion, not the model
  // repeating its previous target.
  ParkHeldTarget();
  let WristFirst = null;
  Replies.push(Text => { const Pose = CurrentPose(Text); WristFirst = { target_cm: { x: Pose.x, y: Pose.y + 3, z: Pose.z }, wrist_pitch_degrees: Pose.pitch };
    return { response_to_user: "Reaching farther.", task_status: "continue", motion_proposal: WristFirst }; },
  () => ({ response_to_user: "Turning the wrist there.", task_status: "continue", motion_proposal: { ...WristFirst, wrist_pitch_degrees: WristFirst.wrist_pitch_degrees - 15 } }));
  await Submit("Grab the eraser");
  await Approve.waitFor();
  await Approve.click();
  await WaitFor(async () => /Turning the wrist there|same target as its previous motion/.test(await ReplyText()), "the wrist-only correction is answered");
  Assert.doesNotMatch(await ReplyText(), /same target as its previous motion/);
  await Approve.waitFor();
  await Page.getByRole("button", { name: "Block", exact: true }).click();
  await WaitFor(async () => await Page.locator(".ai-task-strip").count() === 0, "the block ends the task");

  // An approved gripper move right after a gripper command (inside the 90 ms
  // slider throttle) is sent at once, so the bridge's refusal of it is the
  // approved motion's and reaches the model. It used to be deferred outside
  // the approval's command range, and the check-in claimed it had run. (The
  // Joints panel is hidden in AI mode; the test drives its slider directly.)
  ParkHeldTarget();
  Replies.push(Text => ({ response_to_user: "Toggling the gripper.", task_status: "continue",
    motion_proposal: { direct_servo_targets: [{ name: "gripper", target_fraction: CurrentPose(Text).gripper > 0.5 ? 0 : 1 }] } }),
  { response_to_user: "Noted.", task_status: "ask", motion_proposal: null });
  await Submit("Open the gripper");
  await Approve.waitFor();
  Asked = Completions.length;
  await Page.evaluate(() => {
    const Slider = document.querySelector('input[aria-label="gripper target"]');
    const Setter = Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, "value").set;
    Setter.call(Slider, String(Number(Slider.value) + (Number(Slider.value) + 150 <= Number(Slider.max) ? 150 : -150)));
    Slider.dispatchEvent(new Event("input", { bubbles: true }));
    [...document.querySelectorAll("button")].find(Button => Button.textContent.trim() === "Approve").click();
  });
  await WaitFor(async () => Completions.length > Asked, "the check-in after the approval");
  Assert.match(Completions.at(-1).Text, /was NOT executed: the bridge refused it: Wait for this motion to finish/);
  await WaitFor(async () => await Page.locator(".ai-task-strip").count() === 0, "the model's ask ends the task");
  await Pause(800);

  // A model call that never answers ends the task at its time limit (5 s
  // here, 120 s in use); the call is aborted.
  ParkHeldTarget();
  Replies.push(HoldReply(Up));
  Asked = Completions.length;
  Since = Date.now();
  await Submit("Grab the eraser");
  await WaitFor(async () => Completions.length > Asked, "the model call is in flight");
  await WaitFor(async () => /Stopped: the model \(SafetyTest\) did not answer within 5 s/.test(await ReplyText()), "the model time limit ends the task", 12000);
  Assert.deepEqual(TaskEnds(Since).map(Event => Event.source), ["timeout"]);
  Assert.ok(ModelAborted(Since), "The timed-out call was aborted");
  ReleaseHeld();
  await Pause(600);
  Assert.equal(await Approve.count(), 0);
  Assert.ok(await InputFree());

  // A /locate call that never answers: the same, at its own limit (5 s here, 60 s in use).
  Locate = "hang";
  Asked = Completions.length;
  Since = Date.now();
  await Submit("Grab the eraser");
  await WaitFor(async () => /Stopped: the object measurement \(bridge \/locate\) did not answer within 5 s/.test(await ReplyText()), "the /locate time limit ends the task", 15000);
  Assert.equal(Completions.length, Asked);
  Assert.deepEqual(TaskEnds(Since).map(Event => Event.source), ["timeout"]);
  Locate = Measured;

  // The model's done and ask end tasks too, and a new prompt ends the task it replaces.
  Replies.push({ response_to_user: "All done.", task_status: "done", motion_proposal: null });
  Since = Date.now();
  await Submit("Describe the workspace; do not move.");
  await WaitFor(async () => /All done\./.test(await ReplyText()), "the done reply");
  Assert.deepEqual(TaskEnds(Since).map(Event => Event.source), ["model_done"]);
  ParkHeldTarget();
  Replies.push(Up, { response_to_user: "Fine.", task_status: "ask", motion_proposal: null });
  Since = Date.now();
  await Submit("Grab the eraser");
  await Approve.waitFor();
  await Submit("Describe the workspace; do not move.");
  await WaitFor(async () => /Fine\./.test(await ReplyText()), "the second task's reply");
  Assert.deepEqual(TaskEnds(Since).map(Event => Event.source), ["new_task", "model_ask"]);
  Assert.equal(await Approve.count(), 0);

  // Every prompt of this page load reused the one warm-up: none of them asked again.
  Assert.equal(Preloads.length, 1, "The model is warmed once per page load");
  Assert.deepEqual(Errors, []);
  await Page.screenshot({ path: Path.join(Output, "AiTaskSafety.png") });
  console.log("PASS: camera gate (moved, fetch failure, not verified) stops before the model with a banner; a description still runs; Home still works; check-ins wait for the motion; refusals are tied to the approved command; a first bridge refusal re-prompts, a second stops; one slider per motion; no-op, repeat, rejected-again, unsendable, never-started and lost-object stops; a box too wide for the jaws stops before the model, a best-effort grasp (the tin) goes to the model with its doubts and its card says it may not work; [UNRELIABLE] targets refused; a held object is not lost; the task object goes to /locate, is picked by the task query, tracked when renamed, and bound from the model's estimate with the same checks at reply and Approve; snapped targets are shown, sent after jitter and told to the model; changed limits re-snap; an unsendable approval re-prompts; a fault, Stop and hold, Stop task, Reset AI context, a moving arm and the model and /locate time limits end the task once, abort the call in flight and say why; a task waits for a still arm and reads its pose before the prompt; a wrist-only correction is not a repeat; an approved servo move inside the throttle is attributed; a chained motion (open and hover, open and move up, close and lift as the last motion) sends its two legs in order on one approval with one model call, and Stop task between the legs never sends the second; nor does a refused first part (rewound) or a first part that falls short (the check-in says why), Coast then Hold, a torque-off frame or a new motion epoch between the legs, a first part that never starts, or a refused last-motion chain (the task ends, no motion past the limit); a gripper_after release follows only a tip that arrived; a move off the NEXT STEP low beside the object is refused with the step's JSON, and after every revision the app's own planned step is offered (labelled as the app's), runs once approved, and is told to the model.");
} finally {
  for (const Timer of Timers) { clearInterval(Timer); clearTimeout(Timer); }
  await Browser.close(); await Server.Close();
}
