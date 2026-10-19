import { chromium as Chromium } from "playwright";
import { CadReadyMs, LaunchOptions, StartUiServer } from "./UiTestLib.mjs";
import { readFileSync as ReadFile, mkdirSync as MakeDirectory, writeFileSync as WriteFile } from "node:fs";
import { execFileSync as Execute } from "node:child_process";
import { fileURLToPath as FilePath } from "node:url";
import Path from "node:path";
import Assert from "node:assert/strict";

const Root = Path.dirname(FilePath(import.meta.url));
const Backend = Path.resolve(Root, "../back end");
const Python = ReadFile(Path.join(Root, "../.backend-python.txt"), "utf8").trim();
const State = JSON.parse(Execute(Python, ["-c", "import json, bridge_server; print(json.dumps(bridge_server.BridgeController().state_snapshot()))"], { cwd: Backend, encoding: "utf8" }));
State.status = "CAD preview";
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
  Timer = setInterval(() => Socket.send(JSON.stringify(State)), 150);
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
await Page.route("**/src/components/CadCanvas.tsx*", async Route => {
  const Response = await Route.fetch(), Original = await Response.text();
  const Instrumented = Original.replace("const TargetGroup = new Three.Group();", "const TargetGroup = window.CadTestTarget = new Three.Group();")
    .replace("const Values = Latest.current;", "const Values = Latest.current; window.CadTestPreviewAngles = Values.previewAngles;");
  Assert.notEqual(Instrumented, Original);
  await Route.fulfill({ response: Response, body: Instrumented });
});
try {
  await Page.goto(Server.Url, { waitUntil: "domcontentloaded" });
  const Canvas = Page.locator("canvas[data-cad-ready=true][data-assembly-ready=true]");
  await Canvas.waitFor({ timeout: CadReadyMs });
  await Page.evaluate(() => { window.CadTestControls.enableDamping = false; });
  await Page.getByRole("button", { name: "Set Current", exact: true }).click();
  await Page.waitForFunction(() => document.querySelector("canvas[data-cad-ready=true]")?.dataset.pitchGuideState === "ready");
  const InitialGuide = await Page.evaluate(() => {
    const Arc = window.CadTestTarget.getObjectByName("AcceptableWristAngles");
    window.CadTestInitialGuideGeometry = Arc.geometry;
    return { Visible: Arc.visible, Color: Arc.material.color.getHexString(), Width: Arc.material.linewidth,
      Segments: Arc.geometry.instanceCount, Position: Arc.position.toArray() };
  });
  Assert.equal(InitialGuide.Visible, true); Assert.equal(InitialGuide.Color, "59e695");
  Assert.ok(InitialGuide.Segments > 0 && InitialGuide.Width >= 3);
  Assert.deepEqual(InitialGuide.Position, [0, 0, 0]);
  await Page.getByRole("button", { name: "Fit model in view", exact: true }).click();
  const Snapshot = () => Page.evaluate(() => {
    const Controls = window.CadTestControls;
    return { Target: Controls.target.toArray(), Offset: Controls.object.position.clone().sub(Controls.target).toArray(),
      Forward: Controls.object.getWorldDirection(Controls.object.position.clone()).toArray() };
  });
  const Distance = (First, Second) => Math.hypot(...First.map((Value, Index) => Value - Second[Index]));
  const Drag = async (Button, Start, Delta = { x: 45, y: 20 }) => {
    await Page.mouse.move(Start.x, Start.y);
    await Page.mouse.down({ button: Button });
    await Page.mouse.move(Start.x + Delta.x, Start.y + Delta.y, { steps: 10 });
    await Page.mouse.up({ button: Button });
  };
  const Bounds = await Canvas.boundingBox();
  const Blank = { x: Bounds.x + Bounds.width * .16, y: Bounds.y + Bounds.height * .42 };
  let Before = await Snapshot();
  await Drag("left", Blank);
  let After = await Snapshot();
  Assert.ok(Distance(Before.Target, After.Target) > .1, "Left drag pans the camera target");
  Assert.ok(Distance(Before.Offset, After.Offset) < 1e-6, "Left drag preserves camera orientation");
  Before = After;
  const CubeBefore = await Page.locator(".view-cube-body").getAttribute("style");
  await Drag("right", Blank);
  After = await Snapshot();
  Assert.ok(Distance(Before.Target, After.Target) < 1e-6, "Right drag keeps the orbit center fixed");
  Assert.ok(Distance(Before.Offset, After.Offset) > .1, "Right drag rotates the camera");
  Assert.notEqual(await Page.locator(".view-cube-body").getAttribute("style"), CubeBefore, "Cube follows orbit rotation");
  Assert.equal(await Page.evaluate(() => window.CadTestTarget.getObjectByName("AcceptableWristAngles").geometry === window.CadTestInitialGuideGeometry), true,
    "Camera movement reuses the green guide without running a new angle search");
  await Page.getByRole("button", { name: "Fit model in view", exact: true }).click();
  await Page.getByRole("button", { name: "View front", exact: true }).click();
  const Front = await Snapshot();
  Assert.ok(Front.Offset[1] > 0 && Math.abs(Front.Offset[0]) < 1e-6 && Math.abs(Front.Offset[2]) < 1e-6, "Front face looks from positive Y");
  await Page.getByRole("button", { name: "Fit model in view", exact: true }).click();
  const Targets = () => Page.locator(".target-grid input").evaluateAll(Inputs => Inputs.map(Input => Number(Input.value)));
  const ProjectTarget = async () => Page.evaluate(({ Values, Bounds }) => {
    const Camera = window.CadTestControls.object;
    const Point = Camera.position.clone().set(...Values).project(Camera);
    return { x: Bounds.x + (Point.x * .5 + .5) * Bounds.width, y: Bounds.y + (-Point.y * .5 + .5) * Bounds.height };
  }, { Values: await Targets(), Bounds });
  const DragChecks = [];
  const CheckScreenDrag = async (Name, Alt = false) => {
    const Original = await Targets();
    const Screen = await ProjectTarget(), Delta = { x: 30, y: -20 };
    // Grab off-center to ensure the target does not jump to the pointer.
    const Grab = { x: Screen.x + 8, y: Screen.y - 5 };
    if (Alt) await Page.keyboard.down("Alt");
    await Page.mouse.move(Grab.x, Grab.y);
    await Page.mouse.down({ button: "left" });
    Assert.deepEqual(await Targets(), Original, `${Name}: grabbing does not jump the target`);
    const CameraAtGrab = await Snapshot(), StartScreen = await ProjectTarget();
    await Page.mouse.move(Grab.x + Delta.x, Grab.y + Delta.y, { steps: 10 });
    const Moved = await Targets(), EndScreen = await ProjectTarget();
    const DepthChange = Moved.reduce((Sum, Value, Index) => Sum + (Value - Original[Index]) * CameraAtGrab.Forward[Index], 0);
    const PixelError = Math.hypot(EndScreen.x - StartScreen.x - Delta.x, EndScreen.y - StartScreen.y - Delta.y);
    Assert.ok(Distance(Original, Moved) > .05, `${Name}: left drag moves the target`);
    Assert.ok(Math.abs(DepthChange) < .02, `${Name}: camera depth changed by ${DepthChange} cm`);
    Assert.ok(PixelError < .75, `${Name}: cursor tracking missed by ${PixelError} px`);
    const During = await Snapshot();
    Assert.ok(Distance(CameraAtGrab.Offset, During.Offset) < 1e-6, `${Name}: camera stays fixed while dragging`);
    Assert.ok(Distance(CameraAtGrab.Target, During.Target) < 1e-6, `${Name}: camera does not pan`);
    await Page.mouse.up({ button: "left" });
    if (Alt) await Page.keyboard.up("Alt");
    await Page.waitForFunction(() => document.querySelector("canvas[data-cad-ready=true]")?.dataset.targetState === "valid");
    const Released = await Snapshot();
    Assert.ok(Distance(CameraAtGrab.Offset, Released.Offset) < 1e-6, `${Name}: releasing does not resume old orbit inertia`);
    DragChecks.push({ Name, DepthChangeCm: DepthChange, CursorErrorPx: PixelError });
  };
  const Views = [
    ["Isometric", [1.25, 1.8, .8]], ["Front", [0, 1, 0]], ["Side", [1, 0, 0]],
    ["Top", [0, -.0001, 1]], ["Oblique rear", [-1.4, -1, .6]], ["Alt held", [1.25, 1.8, .8]],
  ];
  for (const [Name, Direction] of Views) {
    for (const [Index, Value] of [30, 35, 25].entries()) await Page.locator('.target-grid input').nth(Index).fill(String(Value));
    await Page.evaluate(Direction => {
      const Controls = window.CadTestControls;
      Controls.enableDamping = false; Controls.update();
      Controls.target.set(25, 30, 20);
      Controls.object.position.copy(Controls.target).add(Controls.object.position.clone().set(...Direction).normalize().multiplyScalar(140));
      Controls.update();
      return new Promise(Resolve => requestAnimationFrame(() => requestAnimationFrame(Resolve)));
    }, Direction);
    await CheckScreenDrag(Name, Name === "Alt held");
  }
  await Page.evaluate(() => { window.CadTestControls.enableDamping = true; });
  await Drag("right", Blank, { x: 12, y: 4 });
  await CheckScreenDrag("Grab during orbit inertia");
  await Page.evaluate(() => { window.CadTestControls.enableDamping = false; });
  const BeforeRight = await Targets();
  Before = await Snapshot();
  await Drag("right", await ProjectTarget());
  After = await Snapshot();
  Assert.deepEqual(await Targets(), BeforeRight, "Right drag over the marker does not move the target");
  Assert.ok(Distance(Before.Offset, After.Offset) > .1, "Right drag over the marker still orbits");
  const WaitValid = () => Page.waitForFunction(() => document.querySelector("canvas[data-cad-ready=true]")?.dataset.targetState === "valid");
  const VerifyTarget = async () => {
    // The app's own verdict on the target it snapped to. Re-solving from the
    // displayed numbers is not the same question: the wrist pitch shown is
    // rounded to a tenth of a degree, and near a joint limit that is enough to
    // land the solve on the other elbow branch.
    const Canvas = Page.locator("canvas[data-cad-ready=true]");
    Assert.equal(await Canvas.getAttribute("data-target-state"), "valid");
    Assert.equal(await Page.locator(String.raw`[data-section="Target"] p.warn-text`).count(), 0, "No blocked-target warning");
    Assert.equal(await Page.getByRole("button", { name: "Send", exact: true }).isDisabled(), false);
    const Result = await Page.evaluate(async ([Elbow, Limits]) => {
      const Api = await import("/src/lib/robotModel.ts");
      const Values = [...document.querySelectorAll(".target-grid input")].map(Input => Number(Input.value));
      const Pitch = Number(document.querySelector(".wide-field input").value);
      const Model = Api.modelPointFromDisplayPoint({ x: -Values[0], y: Values[1], z: Values[2] });
      return Api.solveIkPreview({ ...Model, wristPitch: Pitch }, Elbow, Api.INITIAL_SERVOS, Limits);
    }, [State.elbow_mode, State.ik_joint_limits]);
    Assert.ok(Result.errorCm < 0.1, `Snapped target is still reachable in position: ${JSON.stringify(Result)}`);
  };
  const StartInvalidDrag = async () => {
    const Start = await ProjectTarget();
    await Page.mouse.move(Start.x, Start.y); await Page.mouse.down({ button: "left" });
    // Pointer capture must also handle releasing outside the canvas.
    await Page.mouse.move(Start.x + Bounds.width * 1.5, Start.y - Bounds.height * .5, { steps: 10 });
    await Page.waitForFunction(() => document.querySelector("canvas[data-cad-ready=true]")?.dataset.targetState === "drag-invalid");
  };
  const BeforeInvalid = await Targets();
  const PitchBefore = await Page.locator(".wide-field input").inputValue();
  await StartInvalidDrag();
  const Requested = await Targets();
  Assert.equal(await Page.getByRole("button", { name: "Send", exact: true }).isDisabled(), true);
  const Colors = await Page.evaluate(() => new Promise(Resolve => {
    const Seen = new Set(), Started = performance.now();
    const Tick = () => {
      Seen.add(document.querySelector("canvas[data-cad-ready=true]").dataset.targetColor);
      if (performance.now() - Started > 450) Resolve([...Seen]); else requestAnimationFrame(Tick);
    }; Tick();
  }));
  Assert.deepEqual(new Set(Colors), new Set(["ff4545", "8c2020"]), "Invalid target visibly flashes red");
  await Page.mouse.up({ button: "left" }); await WaitValid(); await VerifyTarget();
  const Snapped = await Targets();
  Assert.ok(Distance(Snapped, Requested) < Distance(BeforeInvalid, Requested), "Release finds a closer valid point, not just the drag start");
  // Position snapping may change wrist pitch to expand the reachable workspace.
  await VerifyTarget();
  Assert.equal(await Page.getByRole("button", { name: "Send", exact: true }).isDisabled(), false);
  const CancellationChecks = [];
  for (const Kind of ["Escape", "pointercancel", "lostpointercapture"]) {
    const Original = await Targets();
    await StartInvalidDrag();
    if (Kind === "Escape") await Page.keyboard.press("Escape");
    else await Canvas.evaluate((Element, Type) => Element.dispatchEvent(new PointerEvent(Type, { pointerId: 1, bubbles: true })), Kind);
    await Page.mouse.up({ button: "left" }); await WaitValid();
    Assert.deepEqual(await Targets(), Original, `${Kind} restores the original target`);
    Assert.equal(await Page.evaluate(() => window.CadTestControls.enabled), true);
    CancellationChecks.push(Kind);
  }
  // Numeric entry uses the same boundary on Enter/blur, not only mouse release.
  const XInput = Page.locator(".target-grid input").nth(0);
  await XInput.fill("500");
  Assert.equal(await Page.getByRole("button", { name: "Send", exact: true }).isDisabled(), true);
  await XInput.press("Enter"); await WaitValid(); await VerifyTarget();
  Assert.notEqual(await XInput.inputValue(), "500");
  const ValidX = await XInput.inputValue();
  await XInput.fill(ValidX);
  Assert.equal(await Page.getByRole("button", { name: "Send", exact: true }).isDisabled(), false, "A valid focused numeric field does not trap Send behind a disabled button");
  await XInput.press("Escape"); await WaitValid();
  await XInput.fill("900"); await XInput.press("Enter");
  await XInput.fill("200");
  await Page.waitForTimeout(350);
  Assert.equal(await XInput.inputValue(), "200", "An older background snap cannot overwrite a newer edit");
  await XInput.press("Enter"); await WaitValid(); await VerifyTarget();
  const ScrollChecks = [];
  const Wheel = Values => Canvas.evaluate((Element, Values) => Element.dispatchEvent(new WheelEvent("wheel", {
    bubbles: true, cancelable: true, deltaX: 0, deltaY: 0, deltaMode: 0, shiftKey: true, ...Values,
  })), Values);
  const PitchInput = Page.locator(".wide-field input");
  const ReadPitch = async () => Number(await PitchInput.inputValue());
  const OriginalPitch = await ReadPitch(), OriginalTarget = await Targets();
  await Page.waitForFunction(() => document.querySelector("canvas[data-cad-ready=true]")?.dataset.pitchGuideState === "ready");
  await Page.evaluate(() => { window.CadTestWheelGuideGeometry = window.CadTestTarget.getObjectByName("AcceptableWristAngles").geometry; });
  const CameraBeforeWheel = await Snapshot();
  await Wheel({ deltaX: 100 });
  await Page.waitForFunction(Pitch => Math.abs(Number(document.querySelector(".wide-field input").value) - Pitch) < 1e-9, OriginalPitch + 5);
  Assert.deepEqual(await Targets(), OriginalTarget, "Wrist scroll does not snap XYZ mid-gesture");
  Assert.equal(await Page.evaluate(() => {
    const Arc = window.CadTestTarget.getObjectByName("AcceptableWristAngles");
    return Arc.visible && Arc.geometry === window.CadTestWheelGuideGeometry;
  }), true, "The green angle guide stays visible and cached while wrist scrolling");
  let CameraAfterWheel = await Snapshot();
  Assert.ok(Distance(CameraBeforeWheel.Offset, CameraAfterWheel.Offset) < 1e-6, "Shift-scroll does not zoom or resume orbit inertia");
  await Page.keyboard.press("Escape"); await WaitValid();
  Assert.equal(await ReadPitch(), OriginalPitch, "Escape cancels the wheel gesture");
  // Many trackpad events in one frame must accumulate, not overwrite each other.
  await Canvas.evaluate(Element => {
    for (let Index = 0; Index < 20; Index++) Element.dispatchEvent(new WheelEvent("wheel", {
      bubbles: true, cancelable: true, shiftKey: true, deltaY: 5,
    }));
  });
  await Page.waitForFunction(Pitch => Math.abs(Number(document.querySelector(".wide-field input").value) - Pitch) < 1e-9, OriginalPitch + 5);
  await WaitValid(); await VerifyTarget();
  ScrollChecks.push("Horizontal wheel, fine trackpad accumulation, delayed single snap and Escape");
  const BeforeZoomTarget = await Targets(), BeforeZoomPitch = await ReadPitch();
  const BeforeZoom = await Snapshot();
  await Page.mouse.move(Blank.x, Blank.y); await Page.mouse.wheel(0, 100);
  await Page.waitForTimeout(80);
  const AfterZoom = await Snapshot();
  Assert.ok(Distance(BeforeZoom.Offset, AfterZoom.Offset) > .1, "Normal wheel zooms");
  Assert.deepEqual(await Targets(), BeforeZoomTarget); Assert.equal(await ReadPitch(), BeforeZoomPitch);
  await Wheel({ deltaY: -10, ctrlKey: true });
  await Page.waitForTimeout(80);
  Assert.equal(await ReadPitch(), BeforeZoomPitch, "Pinch/Control zoom never changes wrist pitch");
  ScrollChecks.push("Normal wheel and pinch zoom preserve the target");
  const Ray = await Page.evaluate(async () => {
    const Api = await import("/src/lib/CadKinematics.ts");
    const Target = window.CadTestTarget, Arrow = Target.getObjectByName("TargetApproachRay");
    const Center = Target.getWorldPosition(Target.position.clone());
    const Start = Arrow.getWorldPosition(Target.position.clone());
    const End = Arrow.localToWorld(Target.position.clone().set(0, Arrow.cone.position.y, 0));
    const Direction = End.clone().sub(Start).normalize();
    const Frame = Api.CadForward(window.CadTestPreviewAngles).Links[Api.CadModel.ToolLink].elements;
    const ToolDirection = Target.position.clone().set(Frame[8], Frame[9], Frame[10]).normalize();
    return { Behind: Start.clone().sub(Center).dot(Direction), Ahead: End.clone().sub(Center).dot(Direction),
      ThroughPointError: Center.clone().sub(Start).cross(Direction).length(),
      ToolAlignment: Direction.dot(ToolDirection), Arrowheads: Arrow.children.filter(Child => Child.isMesh).length,
      Overlay: !Arrow.line.material.depthTest && !Arrow.cone.material.depthTest };
  });
  Assert.ok(Ray.Behind < 0 && Ray.Ahead > 0, "Arrow extends on both sides of the target");
  Assert.ok(Ray.ThroughPointError < 1e-8, "Shaft passes through the target point");
  Assert.ok(Ray.ToolAlignment > .99999, "Arrow follows the preview claw's direction");
  Assert.equal(Ray.Arrowheads, 1); Assert.equal(Ray.Overlay, true);
  await Page.getByRole("button", { name: "Fit model in view", exact: true }).click();
  await Page.waitForFunction(() => document.querySelector("canvas[data-cad-ready=true]")?.dataset.pitchGuideState === "ready");
  await Page.screenshot({ path: Path.join(Output, "TargetSnappedUi.png") });
  Assert.equal(await Canvas.evaluate(Element => Element.dispatchEvent(new MouseEvent("contextmenu", { bubbles: true, cancelable: true }))), false);
  Assert.deepEqual(Errors, []);
  Assert.deepEqual(Commands, []);
  WriteFile(Path.join(Output, "CadTargetDragVerification.json"), JSON.stringify({ DragChecks, Requested, Snapped, Colors, CancellationChecks, ScrollChecks, Ray, InitialGuide, HardwareCommands: Commands }, null, 2));
  console.log("PASS: screen-plane dragging, snap boundaries, scrolling, zoom, approach ray and persistent green angle guide; no motor commands.");
} finally { clearInterval(Timer); await Browser.close(); await Server.Close(); }
