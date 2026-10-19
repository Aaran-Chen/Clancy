// Browser-level routing: every model, bridge, camera-service and websocket
// request is mocked. No command can reach the physical arm or real Ollama.
import { chromium as Chromium } from "playwright";
import { CadReadyMs, LaunchOptions, StartUiServer } from "./UiTestLib.mjs";
import { readFileSync as ReadFile } from "node:fs";
import { execFileSync as Execute } from "node:child_process";
import { fileURLToPath as FilePath } from "node:url";
import Path from "node:path";
import Assert from "node:assert/strict";
const Root = Path.dirname(FilePath(import.meta.url)), Backend = Path.resolve(Root, "../back end");
const Python = ReadFile(Path.join(Root, "../.backend-python.txt"), "utf8").trim();
const State = JSON.parse(Execute(Python, ["-c", "import json, bridge_server; print(json.dumps(bridge_server.BridgeController().state_snapshot()))"], { cwd: Backend, encoding: "utf8" }));
Object.assign(State, { arm_connected: true, torque: true, port: "FAKE", motion_epoch: 5, motion_route: null, motion_fault: "Mock motor protection fault" });
for (const Servo of Object.values(State.servos)) Servo.moving = false;
const Server = await StartUiServer();
const Browser = await Chromium.launch(LaunchOptions({ args: ["--use-fake-device-for-media-stream", "--use-fake-ui-for-media-stream"] }));
const Page = await Browser.newPage({ viewport: { width: 1600, height: 1000 }, permissions: ["camera"] });
const Errors = [], Commands = [], Completions = [], Routes = [], Captures = [], Timers = new Set();
let LocateCalls = 0, DepthCalls = 0, RouteMode = "hang", Reply = "Hello! How can I help?", HoldChat = false;
const Held = [];
const Release = () => { while (Held.length) Held.shift()(); };
Page.on("pageerror", Error_ => Errors.push(Error_.message));
await Page.route("**/bridge-launch/**", Route => Route.fulfill({ json: { ok: true } }));
await Page.route("**/session-log/**", Route => Route.fulfill({ json: { ok: true } }));
await Page.route("**/depth-server/**", Route => { DepthCalls++; return Route.fulfill({ json: {} }); });
await Page.route("**/ollama/**", async Route => {
  const Url = new URL(Route.request().url());
  if (Url.pathname.endsWith("/version")) return Route.fulfill({ json: { version: "0.35.0" } });
  if (Url.pathname.endsWith("/tags")) return Route.fulfill({ json: { models: [{ name: "RoutingTest", size: 2000 }, { name: "nimble:latest", size: 9.5e9 }] } });
  if (Url.pathname.endsWith("/show")) return Route.fulfill({ json: { capabilities: ["vision"] } });
  if (Url.pathname.endsWith("/ps")) return Route.fulfill({ json: { models: [{ name: "RoutingTest", context_length: 32768 }] } });
  if (Url.pathname.endsWith("/systemone")) {
    const Body = Route.request().postDataJSON();
    Routes.push(Body);
    if (RouteMode === "hang") await new Promise(Resolve => Held.push(Resolve));
    if (RouteMode === "fail") return Route.fulfill({ status: 500, body: "Unavailable" });
    const Choice = ["text", "camera", "camera_depth"].includes(RouteMode) ? RouteMode : "text";
    return Route.fulfill({ json: { answers: { input_mode: { type: "choice", choice: Choice,
      probabilities: Object.fromEntries(["text", "camera", "camera_depth"].map(Mode => [Mode, Mode === Choice ? 0.98 : 0.01])) } } } }).catch(() => {});
  }
  if (Url.pathname.endsWith("/api/chat")) {
    const Body = Route.request().postDataJSON();
    if (Body.messages.length === 0) return Route.fulfill({ json: { done: true, done_reason: "load", message: { role: "assistant", content: "" } } });
    Completions.push(Body);
    if (HoldChat) await new Promise(Resolve => Held.push(Resolve));
    return Route.fulfill({ json: { done: true, done_reason: "stop", message: { role: "assistant", content: Reply } } }).catch(() => {});
  }
  return Route.fulfill({ json: { data: [{ id: "RoutingTest" }] } });
});
await Page.route("**:8000/**", Route => {
  const Url = new URL(Route.request().url());
  if (Url.pathname === "/ports") return Route.fulfill({ json: [] });
  if (Url.pathname === "/ik/preview") return Route.fulfill({ json: { ok: false } });
  if (Url.pathname.startsWith("/captures")) Captures.push(Url.pathname);
  if (Url.pathname === "/locate") {
    LocateCalls++;
    return Route.fulfill({ json: { ok: true, rms_cm: 0.2, objects: [], camera_check: { ok: true, angle_deg: 0.1, offset_cm: 0.1, reason: "", reference_fitted_at: "2026-10-05T10:00:00" } } });
  }
  return Route.fulfill({ json: { ok: true, calibrated: false, points: [], point_count: 0 } });
});
await Page.routeWebSocket("**/ws", Socket => {
  Socket.send(JSON.stringify(State));
  const Timer = setInterval(() => Socket.send(JSON.stringify(State)), 100); Timers.add(Timer);
  Socket.onClose(() => { clearInterval(Timer); Timers.delete(Timer); });
  Socket.onMessage(Message => { Commands.push(JSON.parse(String(Message))); Socket.send(JSON.stringify({ type: "status", ok: true, text: "OK" })); });
});
const WaitFor = async (Check, Label) => {
  const Until = Date.now() + 20000;
  while (Date.now() < Until) { if (await Check()) return; await Page.waitForTimeout(30); }
  Assert.fail(`${Label}: ${await Page.locator(".ai-panel").innerText()}`);
};
const Submit = async Text => {
  await Page.getByRole("textbox", { name: "AI task", exact: true }).fill(Text);
  await Page.getByRole("button", { name: "Send to AI", exact: true }).click();
};
const Idle = () => WaitFor(() => Page.getByRole("textbox", { name: "AI task", exact: true }).isEnabled(), "request ended");
const Approve = Page.getByRole("button", { name: "Approve", exact: true });
const CanApprove = async () => (await Approve.count()) > 0 && await Approve.isEnabled();
try {
  await Page.goto(Server.Url, { waitUntil: "domcontentloaded" });
  await Page.locator("canvas[data-cad-ready=true][data-assembly-ready=true]").waitFor({ timeout: CadReadyMs });
  await Page.getByRole("button", { name: "AI control mode", exact: true }).click();
  await Page.waitForFunction(() => document.querySelector("video")?.dataset.captureReady === "true");
  // The page warms Nimble as soon as AI mode shows (here held: a cold 35 s load). A clear question sent meanwhile
  // takes the rule classifier's text route at once: no camera, /locate or depth, and Nimble is not asked again.
  await WaitFor(async () => Held.length > 0, "Nimble warm-up started at page load");
  Assert.equal(Routes.length, 1); Assert.equal(Routes[0].state.user_prompt, "Hello"); Assert.equal(Routes[0].keep_alive, "2h");
  await Submit("What is inverse kinematics?"); await Idle();
  Assert.match(await Page.locator(".ai-route-status").innerText(), /^Prompt only \(rules, \d+ ms\): Nimble is warming up; rules: a general question/);
  Assert.equal(Routes.length, 1, "the prompt did not wait on or ask the cold Nimble");
  Assert.equal(LocateCalls, 0); Assert.equal(DepthCalls, 0); Assert.equal(Captures.length, 0);
  Assert.equal(Completions.length, 1); Assert.ok(Completions[0].messages.every(Message => !Message.images));
  // A scene question meanwhile gets the camera by rule; an object task the full path by the safety floor.
  Reply = "A desk.";
  await Submit("What's on the desk?"); await Idle();
  Assert.match(await Page.locator(".ai-route-status").innerText(), /^Prompt \+ camera \(rules, \d+ ms\): Nimble is warming up; rules: asks what the camera shows/);
  Assert.equal(LocateCalls, 0); Assert.deepEqual(Completions.at(-1).messages.map(Message => Message.images?.length ?? 0), [0, 0, 1]);
  Reply = "Hello! How can I help?";
  RouteMode = "text"; Release();
  await WaitFor(async () => (await Page.locator(".ai-route-status").innerText()) === "Nimble ready", "Nimble prepared");
  const Early = Completions.length;

  // A greeting skips all vision/task machinery and is a short main-model call.
  // Hold both routing and completion across recurring fault snapshots.
  RouteMode = "hang"; HoldChat = true;
  await Submit("Hello there");
  await WaitFor(async () => Held.length > 0, "routing despite the latched hardware fault");
  await Page.waitForTimeout(220);
  Assert.equal(await Page.getByRole("textbox", { name: "AI task", exact: true }).isEnabled(), false);
  RouteMode = "text"; Release();
  await WaitFor(async () => Held.length > 0, "text response despite the latched hardware fault");
  await Page.waitForTimeout(220);
  Assert.equal(await Page.getByRole("textbox", { name: "AI task", exact: true }).isEnabled(), false);
  HoldChat = false; Release(); await Idle();
  Assert.equal(LocateCalls, 0); Assert.equal(DepthCalls, 0); Assert.equal(Captures.length, 1, "only the rule camera route's frame");
  Assert.equal(Completions.length, Early + 1);
  Assert.equal(Completions.at(-1).messages.length, 2);
  Assert.equal(Completions.at(-1).options.num_predict, 512);
  Assert.ok(Completions.at(-1).messages.every(Message => !Message.images));
  Assert.match(await Page.locator(".ai-route-status").innerText(), /^Prompt only \(Nimble, \d+ ms\): Nimble: the prompt alone answers it/);
  Assert.match(await Page.locator(".ai-reply").innerText(), /Hello!/);
  Assert.equal(await CanApprove(), false);

  // A latched hardware fault does not disable conversation. It still blocks
  // a physical request before any image or main-model work can begin.
  await Submit("Move left 3 cm"); await Idle();
  Assert.equal(LocateCalls, 0); Assert.equal(Completions.length, Early + 1);
  Assert.match(await Page.locator(".ai-reply").innerText(), /Not started:.*Mock motor protection fault/);
  State.motion_fault = null;
  await Page.waitForTimeout(220);

  // A malicious/mistaken completion cannot make a motion card on text routing.
  Reply = JSON.stringify({ response_to_user: "Moving now", task_status: "continue", motion_proposal: { move_delta_cm: { left: 3 } }, notepad_update: "unsafe" });
  await Submit("Explain recursion"); await Idle();
  Assert.equal(LocateCalls, 0); Assert.equal(Captures.length, 1);
  Assert.equal(await CanApprove(), false);
  Assert.equal(await Page.locator(".ai-task-strip").count(), 0);

  // Physical command ignores a wrong text classification and gets fresh vision.
  Reply = JSON.stringify({ response_to_user: "Scene checked.", task_status: "done", motion_proposal: null });
  const BeforeRoutes = Routes.length;
  await Submit("Move left 3 cm"); await Idle();
  Assert.equal(Routes.length, BeforeRoutes, "physical safety gate skips classifier");
  Assert.equal(LocateCalls, 1); Assert.ok(Completions.at(-1).messages.some(Message => Message.images?.length));
  Assert.match(await Page.locator(".ai-route-status").innerText(), /^Prompt \+ camera \+ depth \(safety rule, \d+ ms\): Physical request/);

  // A measurement decision from Nimble captures and measures.
  RouteMode = "camera_depth";
  await Submit("Can you describe this?"); await Idle();
  Assert.equal(LocateCalls, 2);
  Assert.ok(Completions.at(-1).messages.some(Message => Message.images?.length));
  Assert.match(await Page.locator(".ai-route-status").innerText(), /^Prompt \+ camera \+ depth \(Nimble, \d+ ms\): Nimble: camera and measured positions needed/);

  // A camera decision sends one colour frame and nothing measured: no /locate, no depth map, no task, and even a
  // motion JSON in its reply makes no approval card.
  RouteMode = "camera";
  Reply = JSON.stringify({ response_to_user: "A red cup. Moving now", task_status: "continue", motion_proposal: { move_delta_cm: { left: 3 } } });
  const [LocateBefore, DepthBefore, CapturesBefore, CompletionsBefore] = [LocateCalls, DepthCalls, Captures.length, Completions.length];
  await Submit("What colour is the cup?"); await Idle();
  Assert.equal(LocateCalls, LocateBefore, "camera route: no /locate");
  Assert.equal(DepthCalls, DepthBefore, "camera route: no depth map");
  Assert.equal(Captures.length, CapturesBefore + 1, "camera route: the one frame is archived");
  Assert.equal(Completions.length, CompletionsBefore + 1);
  const CameraCall = Completions.at(-1);
  Assert.deepEqual(CameraCall.messages.map(Message => Message.images?.length ?? 0), [0, 0, 1], "system, prompt, one colour frame");
  Assert.equal(CameraCall.options.num_predict, 512);
  Assert.match(CameraCall.messages[0].content, /no measured positions, depth, live arm state, or ability to act/);
  Assert.equal(CameraCall.messages[1].content, "What colour is the cup?");
  Assert.match(await Page.locator(".ai-route-status").innerText(), /^Prompt \+ camera \(Nimble, \d+ ms\): Nimble: the camera image answers it/);
  Assert.equal(await CanApprove(), false);
  Assert.equal(await Page.locator(".ai-task-strip").count(), 0);

  // A live-scene question Nimble would answer from text alone still gets the camera (the safety floor).
  RouteMode = "text";
  Reply = "I see a desk.";
  const RoutesBefore = Routes.length;
  await Submit("What do you see?"); await Idle();
  Assert.equal(Routes.length, RoutesBefore + 1, "Nimble is asked");
  Assert.equal(LocateCalls, LocateBefore);
  Assert.deepEqual(Completions.at(-1).messages.map(Message => Message.images?.length ?? 0), [0, 0, 1]);
  Assert.match(await Page.locator(".ai-route-status").innerText(), /^Prompt \+ camera \(safety rule, \d+ ms\): Nimble chose prompt only; Asks about the live scene/);

  // A new fault during physical work still aborts that task immediately.
  HoldChat = true;
  Reply = JSON.stringify({ response_to_user: "late physical motion", task_status: "continue", motion_proposal: { move_delta_cm: { left: 3 } } });
  await Submit("Move left 3 cm");
  await WaitFor(async () => Held.length > 0, "physical model response in flight");
  State.motion_fault = "Mock motor protection fault";
  await Idle();
  Assert.match(await Page.locator(".ai-reply").innerText(), /Stopped:.*Mock motor protection fault/);
  Release(); HoldChat = false;
  Assert.equal(await CanApprove(), false);
  State.motion_fault = null;
  await Page.waitForTimeout(220);

  // Stop during routing must not fall through to capture or the main model.
  RouteMode = "hang";
  const BeforeCompletions = Completions.length, BeforeLocate = LocateCalls;
  await Submit("Explain sorting");
  await WaitFor(async () => Held.length > 0, "held routing decision");
  await Page.getByRole("button", { name: "Stop task", exact: true }).click();
  Release(); await Idle();
  Assert.equal(Completions.length, BeforeCompletions); Assert.equal(LocateCalls, BeforeLocate);
  Assert.equal(await CanApprove(), false);

  // Stop during a text response discards even a late malicious motion JSON.
  RouteMode = "text"; HoldChat = true;
  Reply = JSON.stringify({ response_to_user: "late", task_status: "continue", motion_proposal: { move_delta_cm: { left: 3 } } });
  await Submit("What is recursion?");
  await WaitFor(async () => Held.length > 0, "held text response");
  await Page.getByRole("button", { name: "Stop task", exact: true }).click();
  Release(); await Idle(); HoldChat = false;
  Assert.equal(await CanApprove(), false);
  Assert.doesNotMatch(await Page.locator(".ai-reply").innerText(), /late/);

  // Failed routing visibly falls back to the existing vision path.
  RouteMode = "fail";
  Reply = JSON.stringify({ response_to_user: "Using the camera fallback.", task_status: "done", motion_proposal: null });
  await Submit("Tell me something useful"); await Idle();
  Assert.equal(LocateCalls, BeforeLocate + 1);
  Assert.match(await Page.locator(".ai-route-status").innerText(), /^Prompt \+ camera \+ depth \(fallback, \d+ ms\): Nimble unavailable \(HTTP 500\); using camera and measured positions/);
  Assert.ok(Completions.at(-1).messages.some(Message => Message.images?.length));
  Assert.ok(Commands.every(Command => !["send_ik", "move_servo", "go_home", "connect"].includes(Command.type)), "no hardware command in any scenario");
  Assert.deepEqual(Errors, []);
  console.log("TestAiPromptRouterUi passed: warm-up at page load with rule routes (text, camera) while Nimble is cold, text during hardware fault, physical fault refusal, physical/measured/camera/scene-floor routes, no-motion JSON on text and camera, routing/response cancellation, visible failure fallback");
} finally {
  Release(); for (const Timer of Timers) clearInterval(Timer);
  await Browser.close(); await Server.Close();
}
