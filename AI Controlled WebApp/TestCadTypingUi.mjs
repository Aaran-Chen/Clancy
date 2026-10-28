import { chromium as Chromium } from "playwright";
import { LaunchOptions, RecordPage, StartUiServer, WaitForCad, BackendPython } from "./UiTestLib.mjs";
import { execFileSync as Execute } from "node:child_process";
import { fileURLToPath as FilePath } from "node:url";
import Path from "node:path";
import Assert from "node:assert/strict";

// TYPING TO THE ROBOT LEAVES THE CAD VIEW STILL ("when I type to the robot, the 3D grid CAD tweaks out", 2026-10-25).
// App re-renders on every keystroke in the AI box, and each render handed the CAD view a new wrist-angle guide array
// and new callbacks, so the view redrew its whole scene once per key (about 90 WebGL draw calls), and after an orbit
// each of those frames also ran the orbit's leftover damping, nudging the camera on every key. Here, with the camera orbited away from its
// fitted pose, typing a 50-character task one key at a time must draw no frame at all (data-draws), rebuild nothing
// (data-scene-build), leave the camera where it was (data-camera) and keep the canvas's size; the typed task is then
// sent and answered as before. A mocked bridge, camera and model: no request reaches the real ones.
const Root = Path.dirname(FilePath(import.meta.url)), Backend = Path.resolve(Root, "../back end");
const Python = BackendPython();
const State = JSON.parse(Execute(Python, ["-c", "import json, bridge_server; print(json.dumps(bridge_server.BridgeController().state_snapshot()))"], { cwd: Backend, encoding: "utf8" }));
Object.assign(State, { arm_connected: true, torque: true, port: "FAKE", motion_epoch: 5, motion_route: null, motion_fault: null });
for (const Servo of Object.values(State.servos)) Servo.moving = false;

const Server = await StartUiServer();
const Browser = await Chromium.launch(LaunchOptions({ args: ["--use-fake-ui-for-media-stream"] }));
const Errors = [];
let Passed = false;
try {
  const Page = await Browser.newPage({ viewport: { width: 1600, height: 1000 }, permissions: ["camera"] });
  const Lines = RecordPage(Page);
  Page.on("pageerror", Failure => Errors.push(Failure.message));
  await Page.route("**/session-log/**", Route => Route.fulfill({ json: { ok: true } }));
  await Page.route("**/bridge-launch/**", Route => Route.fulfill({ json: { ok: true } }));
  await Page.route("**/depth-server/**", Route => Route.fulfill({ json: {} }));
  const Prompts = [];
  await Page.route("**/ollama/**", async Route => {
    const Url = new URL(Route.request().url());
    if (Url.pathname.endsWith("/tags")) return Route.fulfill({ json: { models: [{ name: "TypingTest", size: 1000 }] } });
    if (Url.pathname.endsWith("/show")) return Route.fulfill({ json: { capabilities: ["vision"] } });
    if (Url.pathname.endsWith("/ps")) return Route.fulfill({ json: { models: [{ name: "TypingTest", context_length: 32768 }] } });
    if (!Url.pathname.endsWith("/api/chat")) return Route.fulfill({ json: { data: [{ id: "TypingTest" }] } });
    const Body = Route.request().postDataJSON();
    if (Body.messages.length === 0) return Route.fulfill({ json: { model: Body.model, message: { role: "assistant", content: "" }, done: true, done_reason: "load" } });
    Prompts.push(JSON.stringify(Body.messages));
    const Reply = { response_to_user: "Which cup do you mean?", task_status: "ask", motion_proposal: null };
    return Route.fulfill({ json: { model: Body.model, message: { role: "assistant", content: JSON.stringify(Reply) }, done: true, done_reason: "stop",
      prompt_eval_count: 10, eval_count: 5 } }).catch(() => {});
  });
  // The bridge's /locate, as TestAppPlannedStepsUi measures the desk.
  const CameraCheck = { ok: true, angle_deg: 0.12, offset_cm: 0.08, reason: "", reference_fitted_at: "2026-09-28T21:10:00", seq: null };
  const Measured = { ok: true, rms_cm: 0.22, camera_check: CameraCheck, withheld: [], objects: [
    { label: "eraser", x: -5, y: 35, z: -2.3, top_z: -0.3, height_cm: 2, size_cm: [2, 5], method: "depth", support_measured: true,
      support_z_measured: -2.3, valid_fraction: 0.9, support_branch: "desk", task_match: false, task_query_only: false },
    { label: "cup", x: 14, y: 40.8, z: -2.3, top_z: 6.7, height_cm: 9, size_cm: [7, 7], method: "depth", task_match: false, task_query_only: false }] };
  await Page.route("**:8000/**", Route => {
    const Url = new URL(Route.request().url());
    if (Url.pathname === "/ports") return Route.fulfill({ json: [] });
    if (Url.pathname === "/ik/preview") return Route.fulfill({ json: { ok: false } });
    if (Url.pathname === "/locate") return Route.fulfill({ json: Measured }).catch(() => {});
    return Route.fulfill({ json: { ok: true, calibrated: false, points: [], point_count: 0 } });
  });
  // The bridge streams its state ten times a second, as the live one does.
  await Page.routeWebSocket("**/ws", Connected => {
    Connected.send(JSON.stringify(State));
    const Timer = setInterval(() => Connected.send(JSON.stringify(State)), 100);
    Connected.onClose(() => clearInterval(Timer));
    Connected.onMessage(() => Connected.send(JSON.stringify({ type: "status", ok: true, text: "OK" })));
  });

  await Page.goto(Server.Url, { waitUntil: "domcontentloaded" });
  const Canvas = await WaitForCad(Page, { Lines, Name: "TestCadTypingUi" });
  await Page.getByRole("button", { name: "AI control mode", exact: true }).click();
  const Input = Page.getByPlaceholder("Describe a task or movement");
  await Input.waitFor();
  // A model is listed before the send button can be used.
  await Page.waitForFunction(() => document.querySelectorAll("canvas[data-cad-ready=true]").length === 1);

  // Orbit away from the fitted view (right drag), so a camera reset would show.
  const Box = await Canvas.boundingBox();
  const Before = await Canvas.getAttribute("data-camera");
  await Page.mouse.move(Box.x + Box.width * 0.2, Box.y + Box.height * 0.15);
  await Page.mouse.down({ button: "right" });
  await Page.mouse.move(Box.x + Box.width * 0.45, Box.y + Box.height * 0.25, { steps: 8 });
  await Page.mouse.up({ button: "right" });
  const Read = () => Page.evaluate(() => {
    const Element = document.querySelector("canvas[data-cad-ready=true]");
    const Bounds = Element.getBoundingClientRect();
    return { Build: Element.dataset.sceneBuild, Draws: Number(Element.dataset.draws), Camera: Element.dataset.camera,
      Size: `${Element.width}x${Element.height}`, Rect: [Bounds.x, Bounds.y, Bounds.width, Bounds.height].map(Value => Value.toFixed(1)).join(","),
      Canvases: document.querySelectorAll("canvas[data-cad-ready]").length };
  });
  // Let the orbit's damping settle: no frame drawn for a full second.
  let Settled = await Read();
  for (let Tries = 0; Tries < 40; Tries++) {
    await Page.waitForTimeout(1000);
    const Now = await Read();
    if (Now.Draws === Settled.Draws) break;
    Settled = Now;
  }
  Assert.notEqual(Settled.Camera, Before, "the right drag should have orbited the camera");
  const Start = await Read();
  Assert.equal(Start.Draws, Settled.Draws, "the view should be idle before typing");

  const Task = "pick up the red eraser and put it in the cup please";
  Assert.ok(Task.length >= 40);
  await Input.click();
  const PerKey = [];
  for (const Key of Task) {
    await Page.keyboard.type(Key);
    await Page.waitForTimeout(40);
    PerKey.push(await Read());
  }
  await Page.waitForTimeout(500);
  const End = await Read();
  Assert.equal(await Input.inputValue(), Task, "every key should reach the AI box");
  const Report = `start ${JSON.stringify(Start)}, end ${JSON.stringify(End)}`;
  Assert.equal(End.Build, Start.Build, `typing must not rebuild the CAD scene: ${Report}`);
  Assert.equal(End.Canvases, 1, `typing must not add or replace the canvas: ${Report}`);
  for (const [Index, Now] of PerKey.entries()) {
    const Where = `after key ${Index + 1} (${JSON.stringify(Task[Index])}): ${JSON.stringify(Now)}, start ${JSON.stringify(Start)}`;
    Assert.equal(Now.Build, Start.Build, `scene rebuilt ${Where}`);
    Assert.equal(Now.Camera, Start.Camera, `camera moved or reset ${Where}`);
    Assert.equal(Now.Size, Start.Size, `canvas resized ${Where}`);
    Assert.equal(Now.Rect, Start.Rect, `canvas moved or resized on the page ${Where}`);
    Assert.equal(Now.Draws, Start.Draws, `the view redrew ${Where}`);
  }
  console.log(`typed ${Task.length} characters: 0 frames drawn, scene build ${End.Build}, canvas ${End.Size}, camera unchanged`);

  // The chat still works: the typed task is sent and answered.
  const Send = Page.getByRole("button", { name: "Send to AI", exact: true });
  await Page.waitForFunction(() => !document.querySelector("button[title='Send to AI']")?.disabled, null, { timeout: 60000 });
  await Send.click();
  await Page.waitForFunction(() => document.querySelector(".ai-reply")?.textContent?.includes("Which cup do you mean?"), null, { timeout: 60000 })
    .catch(async Failure => { throw new Error(`no reply shown (${Prompts.length} model calls). AI panel: ${await Page.locator(".ai-panel").innerText()}\n${Lines.slice(-15).join("\n")}`, { cause: Failure }); });
  Assert.ok(Prompts.some(Prompt => Prompt.includes(Task)), "the model should receive the typed task");
  Assert.equal(await Input.inputValue(), "", "the AI box clears once the task is sent");
  console.log("sent the typed task and showed the model's reply");
  Assert.deepEqual(Errors, [], `page errors: ${Errors.join("; ")}`);
  Passed = true;
} finally {
  await Browser.close();
  await Server.Close();
}
console.log(Passed ? "TestCadTypingUi: PASS" : "TestCadTypingUi: FAIL");
process.exit(Passed ? 0 : 1);
