import { chromium as Chromium } from "playwright";
import { CadReadyMs, LaunchOptions, StartUiServer } from "./UiTestLib.mjs";
import { readFileSync as ReadFile, mkdirSync as MakeDirectory } from "node:fs";
import { execFileSync as Execute } from "node:child_process";
import { fileURLToPath as FilePath } from "node:url";
import Path from "node:path";
import Assert from "node:assert/strict";

// Measured scene command identity against a mocked bridge, camera and model: the camera gate
// in /locate, check-in timing, stale anchors, no-op and repeated moves, the
// motion limit, the AI target snap (asked vs. sent, jitter at Approve,
// changed joint limits, bridge refusals), and how a task ends (faults, Stop
// and hold, Stop task, a moving arm, time limits: aborted, shown, recorded
// once). No request reaches the real bridge or arm.
const Root = Path.dirname(FilePath(import.meta.url)), Backend = Path.resolve(Root, "../back end");
const Python = ReadFile(Path.join(Root, "../.backend-python.txt"), "utf8").trim();
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
// PLAN can grasp it (lib/GraspPlan.ts); turned 5 cm across, it is NOT GRASPABLE.
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
// Test-only access to the current immutable capture identity lets us reproduce a camera refresh
// during an approval without adding a production debug API. Requests still go only to mocks.
await Page.route('**/src/App.tsx*', async Route => {
  const response = await Route.fetch(), source = await response.text();
  const changed = source.replace('const LatestSceneRef = useRef(null);',
    'const LatestSceneRef = useRef(null); globalThis.__testScene = LatestSceneRef;');
  Assert.notEqual(changed, source, 'scene freshness test hook installed');
  await Route.fulfill({ response, body: changed });
});
const Support = { Name: 'measured platform', Kind: 'platform', TopZCm: -2.3, XRangeCm: [-40, 0], YRangeCm: [10, 50],
  BoardAboveZCm: null, Frame: 'cart', Source: 'depth', ObservedOnly: true };
const Scene = { id: 'scene-17', revision: 'geometry-1', frame: 'display_cm', capture_seq: 17, support: Support,
  obstacles: [], free_space: [{ min: [-80, -80, -2], max: [80, 80, 80] }] };
Locate = { ...Measured, objects: [], withheld: [], scene: Scene };
const Moves = () => Commands.filter(Command => ['send_ik', 'move_servo'].includes(Command.type));
const Done = () => WaitFor(async () => await Page.locator('.ai-task-strip').count() === 0, 'task finishes');
try {
  await Page.goto(Server.Url, { waitUntil: 'domcontentloaded' });
  await Page.locator('canvas[data-cad-ready=true][data-assembly-ready=true]').waitFor({ timeout: CadReadyMs });
  await Page.getByRole('button', { name: 'AI control mode', exact: true }).click();
  await Page.waitForFunction(() => document.querySelector('video')?.dataset.captureReady === 'true');
  await Page.locator('.ai-companion[data-state="ready"]').waitFor();

  Replies.push({ response_to_user: 'Raising a little.', task_status: 'continue', motion_proposal: { move_delta_cm: { left: 0, farther: 0, up: 2 } } },
    { response_to_user: 'Move completed.', task_status: 'done', motion_proposal: null });
  await Submit('Move the arm up 2 cm');
  await Approve.waitFor();
  await Approve.click();
  await WaitFor(async () => Moves().length === 1, 'scene IK command sent');
  Assert.equal(Moves()[0].type, 'send_ik');
  Assert.equal(Moves()[0].scene_id, Scene.id);
  Assert.equal(Moves()[0].scene_required, true);
  Assert.equal(Moves()[0].scene_carry, false);
  await Done();

  Replies.push(Text => ({ response_to_user: 'Changing the gripper.', task_status: 'continue',
    motion_proposal: { direct_servo_targets: [{ name: 'gripper', target_fraction: CurrentPose(Text).gripper > 0.5 ? 0 : 1 }] } }),
    { response_to_user: 'Gripper checked.', task_status: 'done', motion_proposal: null });
  await Submit('Open the gripper');
  await Approve.waitFor();
  await Approve.click();
  await WaitFor(async () => Moves().length === 2, 'scene gripper command sent');
  Assert.equal(Moves()[1].type, 'move_servo');
  Assert.equal(Moves()[1].scene_id, Scene.id);
  Assert.equal(Moves()[1].scene_required, true);
  await Done();

  ParkHeldTarget();
  Replies.push({ response_to_user: 'Raising again.', task_status: 'continue', motion_proposal: { move_delta_cm: { left: 0, farther: 0, up: 2 } } },
    { response_to_user: 'The scene changed. Please take another look.', task_status: 'ask', motion_proposal: null });
  await Submit('Move the arm up 2 cm');
  await Approve.waitFor();
  await Page.evaluate(() => { globalThis.__testScene.current = { id: 'scene-18', revision: 'geometry-2' }; });
  const Before = Moves().length;
  await Approve.click();
  await WaitFor(async () => Events.some(Event => Event.type === 'approve_refused' && Event.issues.some(Issue => /scene changed/.test(Issue))), 'stale scene approval refused');
  await Done();
  Assert.equal(Moves().length, Before, 'stale scene never sends a motor command');
  Assert.deepEqual(Errors, []);
  console.log('Passed: scene-bound IK, direct gripper dispatch, and stale scene approval refusal. All bridge/model requests mocked.');
} finally {
  for (const timer of Timers) clearTimeout(timer);
  await Browser.close(); await Server.Close();
}
