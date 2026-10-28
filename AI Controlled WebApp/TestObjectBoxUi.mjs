import { chromium as Chromium } from "playwright";
import { CadReadyMs, LaunchOptions, StartUiServer, BackendPython } from "./UiTestLib.mjs";
import { mkdirSync as MakeDirectory } from "node:fs";
import { execFileSync as Execute } from "node:child_process";
import { fileURLToPath as FilePath } from "node:url";
import Path from "node:path";
import Assert from "node:assert/strict";

// The 3D view draws the AI task object at its MEASURED footprint and height
// (the "MeasuredObject" box) instead of the 2.2 cm estimate cube, in the scene's
// visual frame (X mirrored from the AI's left-positive frame). An estimate
// with no measured object still shows the cube. Both go when the task ends by
// Stop task, Reset AI context or Manual mode. The bridge, camera and model are
// mocked: no request reaches the real bridge or arm.
const Root = Path.dirname(FilePath(import.meta.url)), Backend = Path.resolve(Root, "../back end");
const Python = BackendPython();
const State = JSON.parse(Execute(Python, ["-c", "import json, bridge_server; print(json.dumps(bridge_server.BridgeController().state_snapshot()))"], { cwd: Backend, encoding: "utf8" }));
Object.assign(State, { arm_connected: true, torque: true, port: "FAKE", motion_epoch: 5, motion_route: null, motion_fault: null });
for (const Servo of Object.values(State.servos)) Servo.moving = false;
const Output = Path.join(Backend, "TestArtifacts"); MakeDirectory(Output, { recursive: true });
const Server = await StartUiServer();
const Browser = await Chromium.launch(LaunchOptions({ args: ["--use-fake-device-for-media-stream", "--use-fake-ui-for-media-stream"] }));
const Page = await Browser.newPage({ viewport: { width: 1600, height: 1000 }, permissions: ["camera"] });
const Errors = [], Commands = [], Completions = [], Timers = new Set();
const Pause = Milliseconds => new Promise(Resolve => setTimeout(Resolve, Milliseconds));
Page.on("pageerror", Error => Errors.push(Error.message));
await Page.route("**/session-log/**", Route => Route.fulfill({ json: { ok: true } }));
await Page.route("**/bridge-launch/**", Route => Route.fulfill({ json: { ok: true } }));
await Page.route("**/depth-server/**", Route => Route.fulfill({ json: {} }));

// The model: each submit takes the next queued reply (a function reads the prompt).
let Replies = [];
const LastUserText = Body => Body.messages.filter(Message => Message.role === "user" && Message.content).at(-1).content;
await Page.route("**/ollama/**", async Route => {
  const Url = new URL(Route.request().url());
  if (Url.pathname.endsWith("/tags")) return Route.fulfill({ json: { models: [{ name: "ObjectBoxTest", size: 1000 }] } });
  if (Url.pathname.endsWith("/show")) return Route.fulfill({ json: { capabilities: ["vision"] } });
  if (Url.pathname.endsWith("/ps")) return Route.fulfill({ json: { models: [{ name: "ObjectBoxTest", context_length: 32768 }] } });
  if (!Url.pathname.endsWith("/api/chat")) return Route.fulfill({ json: { data: [{ id: "ObjectBoxTest" }] } });
  const Body = Route.request().postDataJSON();
  if (Body.messages.length === 0) return Route.fulfill({ json: { model: Body.model, message: { role: "assistant", content: "" }, done: true, done_reason: "load" } });
  const Text = LastUserText(Body);
  Completions.push({ At: Date.now(), Text });
  const Reply = Replies.shift() ?? { response_to_user: "Nothing to do.", task_status: "done", motion_proposal: null };
  const Content = typeof Reply === "function" ? Reply(Text) : Reply;
  return Route.fulfill({ json: { model: Body.model, message: { role: "assistant", content: JSON.stringify(Content) }, done: true, done_reason: "stop",
    prompt_eval_count: 10, eval_count: 5 } }).catch(() => {});
});

// The bridge: /locate measures one standing can (the 2026-09-30 16:49 can, which the
// GRASP PLAN accepts) on a verified camera. Its footprint is not square, so a swapped
// X/Y extent shows; its base is top_z - height_cm.
const CameraCheck = { ok: true, angle_deg: 0.12, offset_cm: 0.08, reason: "", reference_fitted_at: "2026-09-28T21:10:00", seq: null };
const Can = { label: "can", x: -5.44, y: 36.39, z: -1.97, top_z: 10.16, height_cm: 12.13, size_cm: [5.0, 4.5], method: "depth",
  support_measured: true, support_z_measured: -1.77, valid_fraction: 0.9, support_branch: "desk", task_match: false, task_query_only: false };
const Measured = { ok: true, rms_cm: 0.22, camera_check: CameraCheck, withheld: [], objects: [Can] };
let LocateCalls = 0;
await Page.route("**:8000/**", async Route => {
  const Url = new URL(Route.request().url());
  if (Url.pathname === "/ports") return Route.fulfill({ json: [] });
  if (Url.pathname === "/ik/preview") return Route.fulfill({ json: { ok: false } });
  if (Url.pathname === "/locate") { LocateCalls += 1; return Route.fulfill({ json: Measured }); }
  // The frames and task recordings the app saves (/captures) and the notepad are not commands.
  if (Route.request().method() !== "GET" && !/^\/(?:captures|notepad)\b/.test(Url.pathname)) Commands.push({ Path: Url.pathname });
  return Route.fulfill({ json: { ok: true, calibrated: false, points: [], point_count: 0 } });
});
await Page.routeWebSocket("**/ws", Connected => {
  Connected.send(JSON.stringify(State));
  const Timer = setInterval(() => Connected.send(JSON.stringify(State)), 100); Timers.add(Timer);
  Connected.onClose(() => { clearInterval(Timer); Timers.delete(Timer); });
  // Settings the app sends on connect are answered; nothing here moves the arm.
  Connected.onMessage(Message => {
    const Command = JSON.parse(String(Message));
    if (["send_ik", "move_servo", "go_home", "toggle_torque"].includes(Command.type)) Commands.push(Command);
    Connected.send(JSON.stringify({ type: "status", ok: true, text: "OK" }));
  });
});
// The scene and the orbit camera, as TestFloatingCamera reads them.
await Page.route("**/src/components/CadCanvas.tsx*", async Route => {
  const Response = await Route.fetch(), Original = await Response.text();
  const Modified = Original.replace("const Scene = new Three.Scene();", "const Scene = window.HudScene = new Three.Scene();")
    .replace("const Controls = new OrbitControls(", "const Controls = window.HudControls = new OrbitControls(");
  Assert.notEqual(Modified, Original); await Route.fulfill({ response: Response, body: Modified });
});

const Approve = Page.getByRole("button", { name: "Approve", exact: true });
const Reply = Page.locator(".ai-reply");
const Submit = async Text => {
  await Page.getByPlaceholder("Describe a task or movement").fill(Text);
  await Page.getByRole("button", { name: "Send to AI", exact: true }).click();
};
const WaitFor = async (Condition, Message, TimeoutMs = 20000) => {
  const Started = Date.now();
  while (!(await Condition())) {
    if (Date.now() - Started > TimeoutMs) throw new Error(`Timed out: ${Message}\nAI panel: ${await Page.locator(".ai-panel").innerText().catch(() => "")}`);
    await Pause(50);
  }
};
// The measured box, the estimate cube (the 2.2 cm wireframe box) and the target marker, as drawn.
const SceneState = () => Page.evaluate(() => {
  const Scene = window.HudScene;
  const Box = Scene.getObjectByName("MeasuredObject");
  const Estimate = Scene.children.find(Object_ => Object_.isMesh && Object_.geometry.type === "BoxGeometry"
    && Object_.geometry.parameters.width === 2.2 && Object_.material.wireframe);
  const Target = Scene.getObjectByName("TargetApproachRay").parent;
  return { Box: { Visible: Box.visible, Position: Box.position.toArray(), Scale: Box.scale.toArray(), Edges: Box.children.filter(Child => Child.isLineSegments).length,
    Opacity: Box.material.opacity },
    Estimate: Estimate ? { Visible: Estimate.visible, Position: Estimate.position.toArray() } : null,
    Target: { Visible: Target.visible, Position: Target.position.toArray() } };
});
const Near = (Actual, Expected, Label, Tolerance = 1e-6) => Expected.forEach((Value, Index) =>
  Assert.ok(Math.abs(Actual[Index] - Value) <= Tolerance, `${Label}[${Index}]: ${Actual[Index]} is not ${Value} (${JSON.stringify(Actual)})`));
// The can in the scene's visual frame: X mirrored, Z at the middle of base (top - height) to top.
const Base = Can.top_z - Can.height_cm;
const ExpectedCentre = [-Can.x, Can.y, Base + Can.height_cm / 2];
const ExpectedScale = [Can.size_cm[0], Can.size_cm[1], Can.height_cm];
const BoxShown = async () => (await SceneState()).Box.Visible;
const BoxAndCubeGone = async () => { const Now = await SceneState(); return !Now.Box.Visible && !Now.Estimate.Visible; };
const AssertMeasuredBox = async Label => {
  const Now = await SceneState();
  Assert.equal(Now.Box.Visible, true, `${Label}: the measured box shows`);
  Near(Now.Box.Position, ExpectedCentre, `${Label}: box centre`);
  Near(Now.Box.Scale, ExpectedScale, `${Label}: box size`);
  Assert.equal(Now.Box.Edges, 1, `${Label}: the box has its outline`);
  Assert.equal(Now.Estimate.Visible, false, `${Label}: the estimate cube is hidden while the box shows`);
  return Now;
};
// The model's estimate at the can's top centre, in its own (left-positive) frame.
const Estimate = { left: Can.x, farther: Can.y, up: Can.top_z };
// The model copies the NEXT STEP the app planned (its motion_proposal, by matching braces).
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

try {
  await Page.goto(Server.Url, { waitUntil: "domcontentloaded" });
  await Page.locator("canvas[data-cad-ready=true][data-assembly-ready=true]").waitFor({ timeout: CadReadyMs });
  await Page.getByRole("button", { name: "AI control mode", exact: true }).click();
  await Page.waitForFunction(() => document.querySelector("video")?.dataset.captureReady === "true");
  await Page.locator('.ai-companion[data-state="ready"]').waitFor();
  let Now = await SceneState();
  Assert.ok(Now.Estimate, "The estimate cube is in the scene");
  Assert.deepEqual([Now.Box.Visible, Now.Estimate.Visible], [false, false], "Nothing is marked before the AI estimates an object");

  // 1. An estimate with no measured task object (a description names none) keeps the 2.2 cm cube.
  Replies.push({ response_to_user: "A can stands on the desk.", task_status: "done", motion_proposal: null, target_estimate_cm: Estimate });
  await Submit("Describe the workspace; do not move.");
  await WaitFor(async () => /A can stands on the desk/.test(await Reply.innerText()), "the description reply");
  await WaitFor(async () => (await SceneState()).Estimate.Visible, "the estimate cube shows");
  Now = await SceneState();
  Assert.equal(Now.Box.Visible, false, "No measured box without a measured task object");
  Near(Now.Estimate.Position, [-Can.x, Can.y, Can.top_z], "estimate cube");

  // 2. "Grab the can": /locate measures it, the model copies the NEXT STEP and estimates the can.
  // The box takes the measured footprint and height at the measured centre; the cube hides.
  const Step = { Seen: null, Text: "" };
  Replies.push(Text => {
    Step.Text = Text; Step.Seen = NextStep(Text);
    return { response_to_user: "Going beside the can.", task_status: "continue", motion_proposal: Step.Seen, target_estimate_cm: Estimate };
  });
  let Asked = Completions.length, Located = LocateCalls;
  await Submit("Grab the can");
  await WaitFor(async () => await Approve.count() > 0, "the can's next step is offered", 30000);
  Assert.equal(LocateCalls, Located + 1, "The can was measured by /locate");
  Assert.equal(Completions.length, Asked + 1, "One model call");
  Assert.match(Step.Text, /TASK OBJECT "can": detected this check-in at X -5\.4, Y 36\.4/);
  Assert.ok(Step.Seen, "The prompt carried a NEXT STEP");
  await WaitFor(BoxShown, "the measured box shows");
  Now = await AssertMeasuredBox("Grab the can");
  // The cube (hidden) was placed by the same transform: the box sits under the model's estimate.
  Near(Now.Estimate.Position.slice(0, 2), ExpectedCentre.slice(0, 2), "hidden estimate cube over the box");
  Assert.ok(Math.abs(Now.Estimate.Position[2] - (ExpectedCentre[2] + Can.height_cm / 2)) < 1e-6, "The estimate (top centre) is at the box's top face");
  // The planned hover point beside the can (on its left side, +X in the AI's frame, so -X here)
  // is drawn just outside the box's -X face, level with it in Y: the box and the plan agree.
  if (Step.Seen.target_cm) {
    Assert.equal(Now.Target.Visible, true, "The proposed target is drawn");
    const Gap = (ExpectedCentre[0] - ExpectedScale[0] / 2) - Now.Target.Position[0];
    Assert.ok(Gap > 0 && Gap < 6, `The hover point is ${Gap.toFixed(2)} cm outside the box's -X face`);
    Assert.ok(Math.abs(Now.Target.Position[1] - ExpectedCentre[1]) <= ExpectedScale[1] / 2 + 1, "The hover point is level with the box in Y");
    Assert.ok(Now.Target.Position[2] > ExpectedCentre[2] + Can.height_cm / 2, "The hover point is above the box's top");
  }
  const HoverChecked = Boolean(Step.Seen.target_cm);
  // The box in the 3D view: seen from the left, a little behind and above, with the floating camera window out of the way.
  await Page.getByRole("button", { name: "Minimize Live Feed", exact: true }).click();
  await Page.evaluate(Centre => {
    const Controls = window.HudControls; Controls.enableDamping = false;
    Controls.target.set(...Centre); Controls.object.position.set(Centre[0] - 52, Centre[1] - 18, Centre[2] + 34); Controls.update();
  }, ExpectedCentre);
  await Pause(400);
  await Page.locator(".cad-shell").screenshot({ path: Path.join(Output, "ObjectBox.png") });
  await Page.getByRole("button", { name: "Restore Live Feed", exact: true }).click();
  // Stop task ends it: the box and the cube go, and the estimate readout with them.
  await Page.locator(".ai-task-strip").getByRole("button", { name: "Stop task" }).click();
  await WaitFor(BoxAndCubeGone, "Stop task clears the box and the cube");
  Assert.equal(await Page.locator(".ai-target-estimate").count(), 0, "The estimate readout is cleared too");

  // 3. A task the model pauses (ask) keeps its box, like its estimate; Reset AI context clears both.
  Replies.push({ response_to_user: "Is this the can you mean?", task_status: "ask", motion_proposal: null, target_estimate_cm: Estimate });
  await Submit("Grab the can");
  await WaitFor(async () => /Is this the can you mean\?/.test(await Reply.innerText()), "the model asks");
  await WaitFor(BoxShown, "the measured box shows again");
  await AssertMeasuredBox("Paused task");
  await Page.getByRole("button", { name: "Reset AI context", exact: true }).click();
  await WaitFor(BoxAndCubeGone, "Reset AI context clears the box and the cube");

  // 4. Switching to Manual mode clears them as well.
  Replies.push({ response_to_user: "Is this the can?", task_status: "ask", motion_proposal: null, target_estimate_cm: Estimate });
  await Submit("Grab the can");
  await WaitFor(async () => /Is this the can\?/.test(await Reply.innerText()), "the model asks again");
  await WaitFor(BoxShown, "the measured box shows a third time");
  await AssertMeasuredBox("Before Manual mode");
  await Page.getByRole("button", { name: "Manual control mode", exact: true }).click();
  await WaitFor(BoxAndCubeGone, "Manual mode clears the box and the cube");

  Assert.deepEqual(Commands, [], "Nothing here moves the arm or changes torque");
  Assert.deepEqual(Errors, []);
  console.log(`PASS: an estimate alone keeps the 2.2 cm cube; a measured task object is drawn as the MeasuredObject box at its measured footprint and height (scale size_cm x height_cm) and centre (X mirrored, Z = top - height + height/2), outlined, with the cube hidden${HoverChecked ? " and the planned hover point just outside its face" : " (the NEXT STEP only opened the gripper: no hover point to compare)"}; Stop task, Reset AI context and Manual mode clear the box and the cube; no motor commands or browser errors.`);
} finally {
  for (const Timer of Timers) { clearInterval(Timer); clearTimeout(Timer); }
  await Browser.close(); await Server.Close();
}
