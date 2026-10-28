import { chromium as Chromium } from "playwright";
import { CadReadyMs, LaunchOptions, StartUiServer, BackendPython } from "./UiTestLib.mjs";
import { mkdirSync as MakeDirectory, writeFileSync as WriteFile } from "node:fs";
import { execFileSync as Execute } from "node:child_process";
import { fileURLToPath as FilePath } from "node:url";
import Path from "node:path";
import Assert from "node:assert/strict";

// The CAD view's rendering work: a settled scene does not redraw or re-solve IK on repeated telemetry, orbiting and
// changed encoder feedback redraw it, and an explicitly rendered capture holds the scene. Frame times and counts are
// measured and written (CadPerformance.json, or CadPerfLabel) but not asserted: they depend on the GPU and how busy
// the PC is. These checks used to run only with CadPerfLabel=CadPerformanceAfter.json, so the default run asserted
// nothing but "no errors" (audit 2 L7), and read a telemetry snapshot from TestArtifacts that git does not keep: the
// state is now the bridge's own state_snapshot(), connected and holding, as in the other UI suites.
const Root = Path.dirname(FilePath(import.meta.url));
const Backend = Path.resolve(Root, "../back end");
const Python = BackendPython();
const State = JSON.parse(Execute(Python, ["-c", "import json, bridge_server; print(json.dumps(bridge_server.BridgeController().state_snapshot()))"],
  { cwd: Backend, encoding: "utf8" }));
Object.assign(State, { arm_connected: true, torque: true, port: "FAKE", motion_fault: null, status: "CAD preview" });
for (const Servo of Object.values(State.servos)) Servo.moving = false;
const Output = Path.join(Backend, "TestArtifacts"); MakeDirectory(Output, { recursive: true });
const Server = await StartUiServer();
const Browser = await Chromium.launch(LaunchOptions());
const Page = await Browser.newPage({ viewport: { width: 1920, height: 1080 } });
const Errors = [], Commands = [];
Page.on("pageerror", Error => Errors.push(Error.message));
await Page.route("**/bridge-launch/**", Route => Route.fulfill({ json: { ok: true } }));
await Page.route("**/ollama/**", Route => Route.fulfill({ json: { models: [] } }));
await Page.route("**/depth-server/**", Route => Route.fulfill({ json: {} }));
await Page.route("**:8000/**", Route => {
  const Url = new URL(Route.request().url());
  if (Url.pathname === "/ports") return Route.fulfill({ json: [] });
  if (Url.pathname === "/ik/preview") return Route.fulfill({ json: { ok: false } });
  if (Url.pathname === "/cad/alignment") {
    Commands.push({ Endpoint: Url.pathname, Payload: Route.request().postDataJSON() });
    return Route.fulfill({ json: { ok: false, error: "Test must not capture a physical reference" } });
  }
  if (!["GET", "HEAD"].includes(Route.request().method())) {
    Commands.push({ Endpoint: Url.pathname, Method: Route.request().method(), Payload: Route.request().postData() });
  }
  return Route.fulfill({ json: { ok: true, calibrated: false, points: [], point_count: 0 } });
});
let Timer;
await Page.routeWebSocket("**/ws", Socket => {
  Socket.send(JSON.stringify(State));
  Timer = setInterval(() => Socket.send(JSON.stringify(State)), 1000 / 30);
  Socket.onMessage(Message => Commands.push(JSON.parse(String(Message))));
});
// Test-only access to the actual camera controls; the application is unchanged.
await Page.route("**/three_examples_jsm_controls_OrbitControls__js.js*", async Route => {
  const Response = await Route.fetch();
  const Original = await Response.text();
  const Instrumented = Original.replace(/export\s*\{\s*OrbitControls\s*\};/, `
class ObservedOrbitControls extends OrbitControls {
  constructor(...Args) { super(...Args); window.CadTestControls = this; }
}
export { ObservedOrbitControls as OrbitControls };`);
  Assert.notEqual(Instrumented, Original);
  await Route.fulfill({ response: Response, body: Instrumented });
});
await Page.route("**/src/lib/CadKinematics.ts*", async Route => {
  const Response = await Route.fetch();
  const Original = await Response.text();
  const Patched = Original.replace("export function CadSolve(", "function OriginalCadSolve(") + `
export function CadSolve(...Args) {
  globalThis.CadSolveStats ??= { Calls: 0, Milliseconds: 0 };
  const Started = performance.now();
  try { return OriginalCadSolve(...Args); }
  finally { globalThis.CadSolveStats.Calls++; globalThis.CadSolveStats.Milliseconds += performance.now() - Started; }
}`;
  Assert.notEqual(Patched, Original);
  await Route.fulfill({ response: Response, body: Patched });
});
await Page.route("**/src/components/CadCanvas.tsx*", async Route => {
  const Response = await Route.fetch();
  const Original = await Response.text();
  const Patched = Original.replace("Renderer = new Three.WebGLRenderer(", "Renderer = window.CadTestRenderer = new Three.WebGLRenderer(");
  await Route.fulfill({ response: Response, body: Patched });
});
try {
  await Page.goto(Server.Url, { waitUntil: "domcontentloaded" });
  await Page.locator("canvas[data-cad-ready=true][data-assembly-ready=true]").waitFor({ timeout: CadReadyMs });
  await Page.waitForFunction(() => document.querySelector("canvas[data-cad-ready=true]")?.dataset.pitchGuideState !== "pending");
  const Setup = await Page.evaluate(() => {
    const Renderer = window.CadTestRenderer, Gl = Renderer.getContext(), Debug = Gl.getExtension("WEBGL_debug_renderer_info");
    return { Gpu: Debug ? Gl.getParameter(Debug.UNMASKED_RENDERER_WEBGL) : "unknown", Triangles: Renderer.info.render.triangles, DrawCalls: Renderer.info.render.calls };
  });
  const Profile = async Orbit => Page.evaluate(async Orbit => {
    const Frames = [], Renderer = window.CadTestRenderer, Controls = window.CadTestControls;
    window.CadSolveStats = { Calls: 0, Milliseconds: 0 };
    const RenderStart = Renderer.info.render.frame, Start = performance.now();
    Controls.autoRotate = Orbit; Controls.autoRotateSpeed = 2;
    if (Orbit) Controls.dispatchEvent({ type: "change" });
    await new Promise(Resolve => {
      let Previous = Start;
      const Frame = Now => {
        Frames.push(Now - Previous); Previous = Now;
        if (Now - Start < 2500) requestAnimationFrame(Frame); else Resolve();
      };
      requestAnimationFrame(Frame);
    });
    Controls.autoRotate = false;
    const Duration = performance.now() - Start;
    Frames.sort((A, B) => A - B);
    return { DurationMs: Duration, Frames: Frames.length, FrameMedianMs: Frames[Math.floor(Frames.length * .5)], FrameP95Ms: Frames[Math.floor(Frames.length * .95)], RenderedFrames: Renderer.info.render.frame - RenderStart, IkCalls: window.CadSolveStats.Calls, IkMilliseconds: window.CadSolveStats.Milliseconds };
  }, Orbit);
  const Idle = await Profile(false), Orbit = await Profile(true);
  await Page.locator(".target-grid input").nth(1).fill("80");
  const Unreachable = await Profile(true);
  const Results = { Setup, Idle, Orbit, Unreachable };
  // Written first, so a failed check below still leaves the measurements.
  WriteFile(Path.join(Output, process.env.CadPerfLabel ?? "CadPerformance.json"), JSON.stringify(Results, null, 2));
  console.log(JSON.stringify(Results));
  Assert.ok(Setup.Triangles > 100000 && Setup.DrawCalls > 0, `the robot is drawn: ${JSON.stringify(Setup)}`);
  Assert.equal(Idle.IkCalls, 0, "Repeated telemetry must not repeat IK");
  Assert.ok(Idle.RenderedFrames <= 2, `A settled scene must not redraw continuously (${Idle.RenderedFrames} frames in ${Math.round(Idle.DurationMs)} ms)`);
  // Orbiting redraws every animation frame (361 in 2.5 s on this PC); 10 is far below that even on a busy PC.
  Assert.ok(Orbit.RenderedFrames > 10, `Orbiting must continue to redraw (${Orbit.RenderedFrames} frames)`);
  Assert.ok(Unreachable.RenderedFrames > 10, `An unreachable target does not stop the redraw (${Unreachable.RenderedFrames} frames)`);
  Assert.equal(Orbit.IkCalls + Unreachable.IkCalls, 0, "Orbiting the view never re-solves IK, an unreachable target included");
  await Page.evaluate(() => { const Controls = window.CadTestControls; Controls.autoRotate = false; Controls.enableDamping = false; Controls.update(); });
  await Page.waitForTimeout(200);
  const BeforePose = await Page.evaluate(() => window.CadTestRenderer.info.render.frame);
  State.servos.shoulder_pan.present_raw += 30;
  await Page.waitForFunction(Before => window.CadTestRenderer.info.render.frame > Before, BeforePose);
  Results.LiveUpdate = "Changed encoder feedback wakes the renderer";
  const Capture = await Page.evaluate(() => {
    window.CadTestControls.dispatchEvent({ type: "change" });
    return new Promise(Resolve => requestAnimationFrame(() => {
      const Canvas = window.CadTestRenderer.domElement;
      const Check = document.createElement("canvas"); Check.width = Canvas.width; Check.height = Canvas.height;
      const Context = Check.getContext("2d"); Context.drawImage(Canvas, 0, 0);
      Resolve({ Image: Canvas.toDataURL("image/png").slice(0, 22), Alpha: Context.getImageData(0, 0, 1, 1).data[3] });
    }));
  });
  Assert.equal(Capture.Image, "data:image/png;base64,");
  Assert.equal(Capture.Alpha, 255, "An explicitly rendered capture must contain the scene");
  WriteFile(Path.join(Output, process.env.CadPerfLabel ?? "CadPerformance.json"), JSON.stringify(Results, null, 2));
  Assert.deepEqual(Errors, []);
  Assert.deepEqual(Commands, []);
  console.log("PASS: a settled scene neither redraws nor re-solves IK; orbiting and an unreachable target redraw without IK; changed encoder feedback wakes the renderer; a rendered capture holds the scene; no commands or browser errors (frame times written, not asserted).");
} finally { clearInterval(Timer); await Browser.close(); await Server.Close(); }
