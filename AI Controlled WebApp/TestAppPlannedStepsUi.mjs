import { chromium as Chromium } from "playwright";
import { CadReadyMs, LaunchOptions, StartUiServer, BackendPython } from "./UiTestLib.mjs";
import { mkdirSync as MakeDirectory } from "node:fs";
import { execFileSync as Execute } from "node:child_process";
import { fileURLToPath as FilePath } from "node:url";
import Path from "node:path";
import Assert from "node:assert/strict";

// THE APP RUNS THE GRASP PLAN'S STEPS ITSELF (src/lib/AppPlannedSteps.ts), against a mocked bridge, camera and model.
// A grab with a kept GRASP PLAN runs open-and-hover, descend, close-and-lift with ONE model call (at the start): each
// later step is the plan's own NEXT STEP, sent exactly as planned, offered on the approval card labelled as the app's,
// and sent only once approved; after the lift with the jaws holding the eraser, the grab-only task ends there. Anything
// unexpected calls the model, which is told which steps the app ran on its own: the object moved, a refused step, a
// step that fell short, a fault, Stop task, a lost object, and a request that asks for more than the grasp. Stop task
// between the app's steps sends nothing more. No request reaches the real bridge, camera or model.
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
const Errors = [], Commands = [], Completions = [], Events = [], Timers = new Set();
const Pause = Milliseconds => new Promise(Resolve => setTimeout(Resolve, Milliseconds));
Page.on("pageerror", Error => Errors.push(Error.message));
// The session record is skipped for automated browsers; show it to this test.
await Page.addInitScript(() => Object.defineProperty(Navigator.prototype, "webdriver", { get: () => false }));
await Page.route("**/session-log/**", Route => { Events.push({ ...Route.request().postDataJSON(), At: Date.now() }); return Route.fulfill({ json: { ok: true } }); });
await Page.route("**/bridge-launch/**", Route => Route.fulfill({ json: { ok: true } }));
await Page.route("**/depth-server/**", Route => Route.fulfill({ json: {} }));

// The model (Ollama's native /api/chat): each scenario queues replies; a reply may read the prompt it answers.
let Replies = [];
const LastUserText = Body => Body.messages.filter(Message => Message.role === "user" && Message.content).at(-1).content;
await Page.route("**/ollama/**", async Route => {
  const Url = new URL(Route.request().url());
  if (Url.pathname.endsWith("/tags")) return Route.fulfill({ json: { models: [{ name: "PlannedStepsTest", size: 1000 }] } });
  if (Url.pathname.endsWith("/show")) return Route.fulfill({ json: { capabilities: ["vision"] } });
  if (Url.pathname.endsWith("/ps")) return Route.fulfill({ json: { models: [{ name: "PlannedStepsTest", context_length: 32768 }] } });
  if (!Url.pathname.endsWith("/api/chat")) return Route.fulfill({ json: { data: [{ id: "PlannedStepsTest" }] } });
  const Body = Route.request().postDataJSON();
  if (Body.messages.length === 0) return Route.fulfill({ json: { model: Body.model, message: { role: "assistant", content: "" }, done: true, done_reason: "load" } });
  const Text = LastUserText(Body);
  Completions.push({ At: Date.now(), Body, Text });
  const Reply = Replies.shift() ?? { response_to_user: "Nothing to do.", task_status: "ask", motion_proposal: null };
  const Content = typeof Reply === "function" ? await Reply(Text, Body) : Reply;
  return Route.fulfill({ json: { model: Body.model, message: { role: "assistant", content: JSON.stringify(Content) }, done: true, done_reason: "stop",
    prompt_eval_count: 10, eval_count: 5 } }).catch(() => {});
});

// The bridge's /locate: the eraser lies with its 2 cm side across the jaws (the fixed-jaw GRASP PLAN grasps it).
const CameraCheck = { ok: true, angle_deg: 0.12, offset_cm: 0.08, reason: "", reference_fitted_at: "2026-09-28T21:10:00", seq: null };
const Eraser = { label: "eraser", x: -5, y: 35, z: -2.3, top_z: -0.3, height_cm: 2, size_cm: [2, 5], method: "depth",
  support_measured: true, support_z_measured: -2.3, valid_fraction: 0.9, support_branch: "desk" };
const Cup = { label: "cup", x: 14, y: 40.8, z: -2.3, top_z: 6.7, height_cm: 9, size_cm: [7, 7], method: "depth" };
const Measured = { ok: true, rms_cm: 0.22, camera_check: CameraCheck, withheld: [], objects: [Eraser, Cup] };
const Without = { ...Measured, objects: [Cup] };
let Locate = Measured;
await Page.route("**:8000/**", async Route => {
  const Url = new URL(Route.request().url());
  if (Url.pathname === "/ports") return Route.fulfill({ json: [] });
  if (Url.pathname === "/ik/preview") return Route.fulfill({ json: { ok: false } });
  if (Url.pathname === "/locate") {
    const Answer = { ...Locate, objects: Locate.objects.map(Object_ => ({ task_match: false, task_query_only: false, ...Object_ })) };
    return Route.fulfill({ json: Answer }).catch(() => {});
  }
  return Route.fulfill({ json: { ok: true, calibrated: false, points: [], point_count: 0 } });
});

// The websocket bridge: commands run one at a time with one status reply each. A tip move takes the reported tip
// (the last joint position, model frame, X mirrored) to its target, TipShortCm short of it along Y; a gripper close
// stops on the eraser (the bridge's grasp hold: its target is where it stopped, 25% open); an opening gets there.
const GripperMin = State.servos.gripper.range_min, GripperMax = State.servos.gripper.range_max, ClosedRaw = 699;
const HeldRaw = Math.round(GripperMin + 0.25 * (GripperMax - GripperMin));
let Socket, Quiet = false, CommandQueue = Promise.resolve(), RefuseWhen = null, TipShortCm = 0, OnCommand = null;
const PlanMs = 150;
const RunCommand = async (Connected, Command) => {
  Commands.push({ ...Command, At: Date.now() });
  const Reply = (Ok, Text) => Connected.send(JSON.stringify({ type: "status", ok: Ok, text: Text }));
  if (!["send_ik", "move_servo", "go_home"].includes(Command.type)) return Reply(true, "OK");
  if (State.motion_route) return Reply(false, "Wait for this motion to finish, or press Stop before another move");
  if (Command.type === "go_home") return Reply(true, "Moving home");
  Quiet = true;
  await Pause(PlanMs);
  Quiet = false;
  const Refusal = RefuseWhen?.(Command);
  if (Refusal) return Reply(false, Refusal);
  OnCommand?.(Command);
  State.motion_epoch += 1; State.motion_route = { stage: 1, stages: 1 };
  const Gripper = State.servos.gripper;
  const SetGripper = Raw => Object.assign(Gripper, Raw < Gripper.present_raw - 50
    ? { target_raw: Math.max(Raw, HeldRaw), present_raw: Math.max(Raw, HeldRaw), grasp_hold: Raw < HeldRaw }
    : { target_raw: Raw, present_raw: Raw, grasp_hold: false });
  if (Command.type === "send_ik") {
    State.target_cm = { x: Command.target.x, y: Command.target.y, z: Command.target.z }; State.wrist_pitch = Command.target.wrist_pitch;
    State.joint_positions = [...State.joint_positions.slice(0, -1), { x: -Command.target.x, y: Command.target.y - TipShortCm, z: Command.target.z }];
    if (Command.gripper_raw != null) SetGripper(Command.gripper_raw);
  } else if (Command.name === "gripper") SetGripper(Command.raw);
  else Object.assign(State.servos[Command.name], { target_raw: Command.raw, present_raw: Command.raw });
  for (const Servo of Object.values(State.servos)) Servo.moving = true;
  Reply(true, "Moving");
  Connected.send(JSON.stringify(State));
  Timers.add(setTimeout(() => {
    State.motion_route = null;
    for (const Servo of Object.values(State.servos)) Servo.moving = false;
  }, 500));
};
await Page.routeWebSocket("**/ws", Connected => {
  Socket = Connected; Connected.send(JSON.stringify(State));
  const Timer = setInterval(() => { if (!Quiet) Connected.send(JSON.stringify(State)); }, 100); Timers.add(Timer);
  Connected.onClose(() => { clearInterval(Timer); Timers.delete(Timer); });
  Connected.onMessage(Message => { CommandQueue = CommandQueue.then(() => RunCommand(Connected, JSON.parse(String(Message)))); });
});

const Approve = Page.getByRole("button", { name: "Approve", exact: true });
const Strip = Page.locator(".ai-task-strip");
const ReplyText = () => Page.locator(".ai-reply").innerText().catch(() => "");
const Submit = async Text => {
  await Page.getByPlaceholder("Describe a task or movement").fill(Text);
  await Page.getByRole("button", { name: "Send to AI", exact: true }).click();
};
const WaitFor = async (Condition, Message, TimeoutMs = 30000) => {
  const Started = Date.now();
  while (!(await Condition())) {
    if (Date.now() - Started > TimeoutMs) {
      const Shown = await Page.locator(".ai-panel").innerText().catch(() => "");
      throw new Error(`Timed out: ${Message}\nAI panel: ${Shown}\nLast events: ${JSON.stringify(Events.slice(-6).map(Event => [Event.type, Event.reason ?? Event.reply ?? Event.issues ?? Event.model_call_reason]))}`);
    }
    await Pause(50);
  }
};
const Since = (Type, At) => Events.filter(Event => Event.type === Type && Event.At >= At);
const MovesSince = At => Commands.filter(Command => Command.At >= At && ["move_servo", "send_ik", "go_home"].includes(Command.type));
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
// A step's tip target in the bridge's (model) frame, from the tip it was read at (model frame; X mirrored from the AI's
// "left"), or null for an absolute target_cm (checked against the record, in the AI's frame).
const StepTarget = (Step, From) => Step.move_delta_cm
  ? { x: From.x - Step.move_delta_cm.left, y: From.y + Step.move_delta_cm.farther, z: From.z + Step.move_delta_cm.up } : null;
const Close = (A, B, Cm = 0.11) => Math.hypot(A.x - B.x, A.y - B.y, A.z - B.z) <= Cm;
const AppCard = async () => (await Page.locator(".ai-motion-copy h2").innerText()) === "App's planned step"
  && /proposed by the app without asking the AI/.test(await Page.locator('[data-planned-step="true"]').innerText());

// A new grab from the arm's home pose, the jaws nearly closed: the model's first reply copies the NEXT STEP (open the
// gripper and go to the hover point, ONE motion). Then(Text) answers the next model call, which Seen.Next keeps.
const StartGrab = async (Task, Then = () => ({ response_to_user: "Stopping here.", task_status: "ask", motion_proposal: null })) => {
  await WaitFor(async () => await Strip.count() === 0 && await Approve.count() === 0, "no task is left before the next one");
  State.joint_positions = structuredClone(HomeJoints);
  State.target_cm = { x: 0, y: 45, z: 0 };
  Object.assign(State.servos.gripper, { present_raw: ClosedRaw, target_raw: ClosedRaw, grasp_hold: false });
  Locate = Measured; RefuseWhen = null; TipShortCm = 0; OnCommand = null;
  await Pause(600);
  const Seen = { First: null, Next: "", Asked: Completions.length, At: Date.now() };
  Replies.length = 0;
  Replies.push(Text => { Seen.First = NextStep(Text); return { response_to_user: "Opening and going to the hover point.", task_status: "continue", motion_proposal: Seen.First }; },
    (Text, Body) => { Seen.Next = Text; Seen.NextBody = Body; return Then(Text); });
  await Submit(Task);
  await WaitFor(async () => await Approve.count() > 0, "the model's first step is offered");
  Assert.equal(Completions.length, Seen.Asked + 1);
  Assert.ok(Seen.First?.gripper_before === 0.8 && Seen.First.target_cm, `The first step opens and hovers: ${JSON.stringify(Seen.First)}`);
  return Seen;
};
// Approves the card shown now; checks nothing moved before the click.
const ApproveNow = async () => {
  const Before = Date.now();
  await Pause(700);
  Assert.equal(MovesSince(Before).length, 0, "Nothing is sent while the card waits for Approve");
  const At = Date.now();
  await Approve.click();
  return At;
};
// The app's step offered now: its planned_step record (offered, without a model call).
const AppStep = async (Asked, Label) => {
  await WaitFor(async () => await Approve.count() > 0 && await AppCard(), `the app offers ${Label}`);
  Assert.equal(Completions.length, Asked, `${Label}: no model call`);
  const Offer = Since("planned_step", 0).filter(Event => Event.app_run && Event.offered).at(-1);
  Assert.ok(Offer && Offer.model_call_skipped === true && /needs no judgement from the image/.test(Offer.reason), JSON.stringify(Offer));
  return Offer;
};
// The app's steps the next model call was told of: "motion N: ..." entries, and the reason it was called.
const Told = Text => {
  const Line = Text.split("\n").find(Line => Line.startsWith("STEPS THE APP RAN ITSELF")) ?? "";
  return { Line, Motions: [...Line.matchAll(/motion (\d+): /g)].map(Match => Number(Match[1])), Because: Line.match(/The app asks you now because (.*)\.$/)?.[1] ?? null };
};

try {
  await Page.goto(Server.Url, { waitUntil: "domcontentloaded" });
  await Page.locator("canvas[data-cad-ready=true][data-assembly-ready=true]").waitFor({ timeout: CadReadyMs });
  await Page.getByRole("button", { name: "AI control mode", exact: true }).click();
  await Page.waitForFunction(() => document.querySelector("video")?.dataset.captureReady === "true");
  await Page.locator('.ai-companion[data-state="ready"]').waitFor();

  // 1. The whole grab, one model call: open and hover (the model's copy of the NEXT STEP), then descend and close-and-lift
  // (the app's own steps, each approved), then the app ends the grab-only task.
  {
    const Seen = await StartGrab("Grab the eraser");
    const Asked = Completions.length;
    const First = await ApproveNow();
    await WaitFor(async () => MovesSince(First).length === 1, "the open-and-hover motion is sent");
    const Sent1 = MovesSince(First)[0];
    Assert.equal(Sent1.gripper_raw != null, true, "One motion: the tip move carrying the opening");
    // The descent: the plan's step exactly.
    const Descend = await AppStep(Asked, "the descent");
    Assert.match(await ReplyText(), /The app's next GRASP PLAN step \(the AI was not asked: nothing unexpected happened\): .*descend straight down to the GRASP POINT Z -?[\d.]+\. Approve it to go on, or Block to stop the task\./);
    const Step2 = JSON.parse(Descend.step);
    const Second = await ApproveNow();
    await WaitFor(async () => MovesSince(Second).length === 1, "the descent is sent");
    const Sent2 = MovesSince(Second)[0];
    Assert.equal(Sent2.type, "send_ik");
    // The tip follows each send_ik exactly here, so the step is read from the hover target.
    const Want2 = StepTarget(Step2, Sent1.target);
    if (Want2) Assert.ok(Close(Sent2.target, Want2), `Sent ${JSON.stringify(Sent2.target)}, the step ${Descend.step} from ${JSON.stringify(Sent1.target)}`);
    else Assert.ok(Close(Descend.motion.target_cm, Step2.target_cm), `The record ${JSON.stringify(Descend.motion.target_cm)}, the step ${Descend.step}`);
    Assert.deepEqual(Descend.motion.requested_target_cm, Descend.motion.target_cm, "Not snapped");
    Assert.equal(Sent2.target.wrist_pitch, Step2.wrist_pitch_degrees, "The step's own wrist pitch");
    Assert.equal(Events.find(Event => Event.type === "approve" && Event.At >= Second)?.motion?.app_run, true);
    // Close and lift: one chained motion, the plan's step exactly.
    const Lift = await AppStep(Asked, "the close and lift");
    const Step3 = JSON.parse(Lift.step);
    Assert.deepEqual(Step3, { move_delta_cm: { left: 0, farther: 0, up: 6 }, gripper_before: 0 }, Lift.step);
    const From3 = Sent2.target;
    // Held in the jaws and lifted, the camera no longer measures it on the desk.
    OnCommand = Command => { if (Command.type === "send_ik") Locate = Without; };
    const Third = await ApproveNow();
    await WaitFor(async () => MovesSince(Third).length === 2, "the close, then the lift");
    const [Grip, Up] = MovesSince(Third);
    Assert.deepEqual([Grip.type, Grip.name, Grip.raw], ["move_servo", "gripper", GripperMin], "The close to the plan's 0");
    Assert.ok(Close(Up.target, StepTarget(Step3, From3)), `The lift ${JSON.stringify(Up.target)} from ${JSON.stringify(From3)}`);
    // Lifted with the jaws holding it: the app ends the grab.
    await WaitFor(async () => await Strip.count() === 0, "the app ends the task after the lift");
    Assert.match(await ReplyText(), /The GRASP PLAN is complete: the jaws closed partway on the eraser \(the gripper holds at 25% open\) and the arm lifted it\. The app ended the task without asking the AI; check that the eraser is in the jaws\./);
    Assert.deepEqual(Since("task_end", Seen.At).map(Event => Event.source), ["app_done"]);
    Assert.equal(Completions.length, Seen.Asked + 1, "Exactly one model call: the start");
    Assert.equal(MovesSince(Seen.At).length, 4, "Three approvals, four bridge commands (the hover, the descent, the close, the lift)");
    Assert.equal(Since("approve", Seen.At).length, 3, "Every motion was approved");
    Assert.deepEqual(Since("planned_step", Seen.At).map(Event => [Event.offered ?? null, Event.finished ?? null]), [[true, null], [true, null], [null, true]]);
    Assert.equal(Since("reply", Seen.At)[0].model_call_reason, "the task starts: the AI reads the request and picks the object");
    await Page.screenshot({ path: Path.join(Output, "AppPlannedSteps.png") });
  }

  // 2. A request that asks for more than the grasp: after the lift the model is called, told of the two app steps.
  {
    const Seen = await StartGrab("Grab the eraser and put it in the basket");
    const Asked = Completions.length;
    await ApproveNow();
    await AppStep(Asked, "the descent");
    await ApproveNow();
    await AppStep(Asked, "the close and lift");
    OnCommand = Command => { if (Command.type === "send_ik") Locate = Without; };
    await ApproveNow();
    await WaitFor(() => Boolean(Seen.Next), "the model is called after the grasp");
    const Note = Told(Seen.Next);
    Assert.deepEqual(Note.Motions, [2, 3], Note.Line);
    Assert.match(Note.Line, /motion 2: [^;]*descend straight down to the GRASP POINT Z -?[\d.]+; motion 3: [^;]*close the gripper and lift as ONE chained motion/);
    Assert.equal(Note.Because, "the grasp is done, and the request asks for more than picking up the eraser");
    Assert.match(Seen.Next, /- Motion 3 \(chained, the app's planned NEXT STEP, proposed by the app without asking you\): gripper to raw \d+, then tip to X /);
    Assert.match(Seen.Next, /That motion was the app's own NEXT STEP from the GRASP PLAN, proposed by the app without asking you and approved by the user/);
    Assert.doesNotMatch(Seen.Next, /because your proposals did not follow it/);
    Assert.equal(Since("reply", Seen.At).at(-1).model_call_reason, Note.Because);
    Assert.equal(Since("reply", Seen.At).at(-1).app_steps_told, 2);
    await WaitFor(async () => await Strip.count() === 0, "the model's ask ends the task");
  }

  // 3. Unexpected events after an app step: each calls the model with the app's steps so far.
  // The eraser moved 3 cm after the descent was approved.
  {
    const Seen = await StartGrab("Grab the eraser");
    const Asked = Completions.length;
    await ApproveNow();
    await AppStep(Asked, "the descent");
    OnCommand = () => { Locate = { ...Measured, objects: [{ ...Eraser, x: -8 }, Cup] }; };
    await ApproveNow();
    await WaitFor(() => Boolean(Seen.Next), "the moved eraser calls the model");
    const Note = Told(Seen.Next);
    Assert.deepEqual(Note.Motions, [2], Note.Line);
    Assert.match(Note.Because, /GRASP PLAN|gripper's way|eraser/, Note.Because);
    console.log(`moved: ${Note.Because}`);
    await WaitFor(async () => await Strip.count() === 0, "the ask ends the task");
  }
  // The bridge refuses the app's close and lift: the model hears of the refusal, and of the descent (which ran) only.
  {
    const Seen = await StartGrab("Grab the eraser");
    const Asked = Completions.length;
    await ApproveNow();
    await AppStep(Asked, "the descent");
    await ApproveNow();
    await AppStep(Asked, "the close and lift");
    RefuseWhen = Command => Command.type === "move_servo" && Command.name === "gripper" && "Gripper route refused (test)";
    await ApproveNow();
    await WaitFor(() => Boolean(Seen.Next), "the refusal calls the model");
    const Note = Told(Seen.Next);
    Assert.deepEqual(Note.Motions, [2], Note.Line);
    Assert.match(Note.Because, /^the bridge refused the last approved motion \(Gripper route refused \(test\)\)$/);
    Assert.match(Seen.Next, /Your last approved motion \(motion 3\) was NOT executed: the bridge refused it: Gripper route refused \(test\)/);
    await WaitFor(async () => await Strip.count() === 0, "the ask ends the task");
  }
  // The descent stops 3 cm short of its target.
  {
    const Seen = await StartGrab("Grab the eraser");
    const Asked = Completions.length;
    await ApproveNow();
    await AppStep(Asked, "the descent");
    TipShortCm = 3;
    await ApproveNow();
    await WaitFor(() => Boolean(Seen.Next), "the short descent calls the model");
    const Note = Told(Seen.Next);
    Assert.deepEqual(Note.Motions, [2], Note.Line);
    Assert.match(Note.Because, /^the last motion stopped 3\.0 cm from its target \(more than 1\.5 cm\)$/);
    await WaitFor(async () => await Strip.count() === 0, "the ask ends the task");
    TipShortCm = 0;
  }
  // The eraser is lost (not measured) after the descent.
  {
    const Seen = await StartGrab("Grab the eraser");
    const Asked = Completions.length;
    await ApproveNow();
    await AppStep(Asked, "the descent");
    OnCommand = () => { Locate = Without; };
    await ApproveNow();
    await WaitFor(() => Boolean(Seen.Next), "the lost eraser calls the model");
    const Note = Told(Seen.Next);
    Assert.deepEqual(Note.Motions, [2], Note.Line);
    Assert.match(Note.Because, /the gripper hides the eraser|the eraser was not measured/, Note.Because);
    console.log(`lost: ${Note.Because}`);
    await WaitFor(async () => await Strip.count() === 0, "the ask ends the task");
  }
  // A bridge fault while the app's close and lift waits for approval: the task ends as before (no model call, nothing
  // sent); the next task's first model call is told of the app's descent and how the task ended.
  {
    const Seen = await StartGrab("Grab the eraser");
    const Asked = Completions.length;
    await ApproveNow();
    await AppStep(Asked, "the descent");
    await ApproveNow();
    await AppStep(Asked, "the close and lift");
    const FaultAt = Date.now();
    State.motion_fault = "shoulder_lift drifted 600 steps past its hold point (test)";
    await WaitFor(async () => /Stopped: the bridge reported a motion fault: shoulder_lift drifted 600 steps/.test(await ReplyText()), "the fault ends the task");
    await Pause(1500);
    Assert.equal(MovesSince(FaultAt).length, 0, "Nothing is sent after the fault");
    Assert.equal(Completions.length, Asked, "No model call at the fault");
    Assert.equal(await Approve.count(), 0);
    Assert.deepEqual(Since("task_end", FaultAt).map(Event => Event.source), ["motion_fault"]);
    State.motion_fault = null;
    await Pause(500);
    const After = { Text: "" };
    Replies.length = 0;
    Replies.push(Text => { After.Text = Text; return { response_to_user: "Stopping here.", task_status: "ask", motion_proposal: null }; });
    await Submit("Grab the eraser");
    await WaitFor(() => Boolean(After.Text), "the next task's model call");
    const Note = Told(After.Text);
    Assert.deepEqual(Note.Motions, [2], Note.Line);
    Assert.match(Note.Because, /^the last task ended \(the bridge reported a motion fault: shoulder_lift drifted 600 steps past its hold point \(test\)\) and a new prompt was sent$/);
    await WaitFor(async () => await Strip.count() === 0, "the ask ends the task");
    // Told once: the call after it is not told again.
    const Again = { Text: "" };
    Replies.push(Text => { Again.Text = Text; return { response_to_user: "Stopping here.", task_status: "ask", motion_proposal: null }; });
    await Submit("Grab the eraser");
    await WaitFor(() => Boolean(Again.Text), "a later model call");
    Assert.doesNotMatch(Again.Text, /STEPS THE APP RAN ITSELF/);
    await WaitFor(async () => await Strip.count() === 0, "the ask ends the task");
  }
  // Stop task between the app's steps: nothing more is sent, no model call or check-in follows, and the next task's
  // first model call is told of the app's descent.
  {
    const Seen = await StartGrab("Grab the eraser");
    const Asked = Completions.length;
    await ApproveNow();
    await AppStep(Asked, "the descent");
    await ApproveNow();
    await AppStep(Asked, "the close and lift");
    const StopAt = Date.now();
    await Page.getByRole("button", { name: "Stop task", exact: true }).click();
    await WaitFor(async () => /Stopped: Stop task was pressed\./.test(await ReplyText()), "Stop task ends the task");
    await Pause(3000);
    Assert.equal(MovesSince(StopAt).length, 0, "Nothing is sent after Stop task");
    Assert.equal(Completions.length, Asked, "No model call after Stop task");
    Assert.equal(Since("task_check_in", StopAt).length, 0, "No check-in after Stop task");
    Assert.equal(await Approve.count(), 0);
    Assert.deepEqual(Since("task_end", StopAt).map(Event => Event.source), ["stop_task"]);
    const After = { Text: "" };
    Replies.length = 0;
    Replies.push(Text => { After.Text = Text; return { response_to_user: "Stopping here.", task_status: "ask", motion_proposal: null }; });
    await Submit("Grab the eraser");
    await WaitFor(() => Boolean(After.Text), "the next task's model call");
    const Note = Told(After.Text);
    Assert.deepEqual(Note.Motions, [2], Note.Line);
    Assert.match(Note.Because, /^the last task ended \(Stop task was pressed\) and a new prompt was sent$/);
    await WaitFor(async () => await Strip.count() === 0, "the ask ends the task");
  }

  Assert.deepEqual(Errors, []);
  console.log("PASS: a grab with a kept GRASP PLAN runs open-and-hover, descend and close-and-lift with one model call (the start): each later step is the plan's own, sent exactly as planned, labelled as the app's and sent only after Approve, and the grab-only task ends after the lift with the jaws holding it; the model is called, told of the app's steps, for a request that asks for more, an object that moved, a refused step, a step that fell short, a lost object, a fault and Stop task (the next task's first call, once); Stop task between the app's steps sends nothing more.");
} finally {
  for (const Timer of Timers) { clearInterval(Timer); clearTimeout(Timer); }
  await Browser.close(); await Server.Close();
}
