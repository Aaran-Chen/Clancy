/** Clickable joint handles on the live arm and the target pose preview, and
    turning a selected joint by dragging its dial in the 3D view. */
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
// A mocked, connected arm with torque on, so a joint drag goes all the way to a
// move_servo command exactly as the slider's does. Nothing here reaches hardware.
State.arm_connected = true; State.torque = true; State.port = "FAKE";
const Server = await StartUiServer();
const Browser = await Chromium.launch(LaunchOptions());
const Page = await Browser.newPage({ viewport: { width: 1600, height: 1000 } });
const Errors = [], Timers = new Set(), Commands = [];
Page.on("pageerror", Error => Errors.push(Error.message));
await Page.route("**/bridge-launch/**", Route => Route.fulfill({ json: { ok: true } }));
await Page.route("**/depth-server/**", Route => Route.fulfill({ json: {} }));
await Page.route("**/ollama/**", Route => Route.fulfill({ json: { data: [] } }));
await Page.route("**/session-log/**", Route => Route.fulfill({ json: { ok: true } }));
await Page.route("**:8000/**", Route => {
  const Url = new URL(Route.request().url());
  if (Url.pathname === "/ports") return Route.fulfill({ json: [] });
  if (Url.pathname === "/ik/preview") return Route.fulfill({ json: { ok: false } });
  return Route.fulfill({ json: { ok: true, calibrated: false, points: [], point_count: 0 } });
});
await Page.routeWebSocket("**/ws", Socket => {
  Socket.send(JSON.stringify(State));
  const Timer = setInterval(() => Socket.send(JSON.stringify(State)), 150); Timers.add(Timer);
  Socket.onClose(() => { clearInterval(Timer); Timers.delete(Timer); });
  Socket.onMessage(Message => {
    const Command = JSON.parse(String(Message)); Commands.push(Command);
    if (Command.type === "move_servo") State.servos[Command.name].target_raw = Command.raw;
  });
});
await Page.route("**/src/components/CadCanvas.tsx*", async Route => {
  const Response = await Route.fetch(), Original = await Response.text();
  const Modified = Original.replace("const Scene = new Three.Scene();", "const Scene = window.HudScene = new Three.Scene();")
    .replace("const Controls = new OrbitControls(", "const Controls = window.HudControls = new OrbitControls(")
    .replace("const Raycaster = new Three.Raycaster();", "const Raycaster = window.HudRaycaster = new Three.Raycaster();");
  Assert.notEqual(Modified, Original);
  await Route.fulfill({ response: Response, body: Modified });
});
/** Screen point of one visible handle, in page pixels. */
const HandlePoint = (Kind, Joint) => Page.evaluate(([Kind, Joint]) => {
  const Group = window.HudScene.getObjectByName(`${Kind}JointMarker_${Joint}`);
  if (!Group?.visible) return null;
  const Canvas = document.querySelector("canvas[data-cad-ready=true]").getBoundingClientRect();
  const Point = Group.position.clone().project(window.HudControls.object);
  return { X: Canvas.left + (Point.x * 0.5 + 0.5) * Canvas.width, Y: Canvas.top + (-Point.y * 0.5 + 0.5) * Canvas.height };
}, [Kind, Joint]);
/** The selected joint's dial in page pixels: its hub, the knob (the target
    angle), the target ball, and the arc's vertices from the low end of the
    joint's range (fraction 0) to the high end (fraction 1). */
const DialGeometry = () => Page.evaluate(() => {
  const Scene = window.HudScene, Camera = window.HudControls.object;
  const Guide = Scene.getObjectByName("SelectedJointGuide"), Ring = Scene.getObjectByName("SelectedJointArc");
  if (!Guide?.visible) return null;
  const Canvas = document.querySelector("canvas[data-cad-ready=true]").getBoundingClientRect();
  const Project = Point => { const P = Point.clone().project(Camera); return { X: Canvas.left + (P.x * .5 + .5) * Canvas.width, Y: Canvas.top + (-P.y * .5 + .5) * Canvas.height }; };
  const Values = Ring.geometry.attributes.instanceStart.data.array, Count = Values.length / 6;
  const Vertex = Index => { const At = Index < Count ? 6 * Index : 6 * Count - 3;
    return Guide.position.clone().set(Values[At], Values[At + 1], Values[At + 2]).multiplyScalar(Guide.scale.x).add(Guide.position); };
  const Knob = Scene.getObjectByName("SelectedJointKnob"), Target = Scene.getObjectByName("TargetApproachRay")?.parent;
  return { Center: Project(Guide.position), Points: Array.from({ length: Count + 1 }, (_, Index) => Project(Vertex(Index))),
    Knob: Knob?.visible ? Project(Knob.position) : null, Target: Target?.visible ? Project(Target.position) : null };
});
const Camera = () => Page.evaluate(() => ({ Target: window.HudControls.target.toArray(),
  Offset: window.HudControls.object.position.clone().sub(window.HudControls.target).toArray() }));
const Distance = (First, Second) => Math.hypot(...First.map((Value, Index) => Value - Second[Index]));
const Span = Servo => Servo.arc_inverted ? (Servo.range_min - Servo.range_max + 4096) % 4096 : (Servo.range_max - Servo.range_min + 4096) % 4096;
/** Where a raw sits in a joint's configured range: 0 at its low end, 1 at its high end, above 1 outside it. */
const Fraction = (Servo, Raw) => (Servo.arc_inverted ? (Servo.range_min - Raw + 4096) % 4096 : (Raw - Servo.range_min + 4096) % 4096) / Span(Servo);
const Moves = Name => Commands.filter(Command => Command.type === "move_servo" && Command.name === Name);
const Cursor = () => Page.locator("canvas[data-cad-ready=true]").evaluate(Element => getComputedStyle(Element).cursor);
try {
  await Page.goto(Server.Url, { waitUntil: "domcontentloaded" });
  const Canvas = Page.locator("canvas[data-cad-ready=true][data-assembly-ready=true]");
  await Canvas.waitFor({ timeout: CadReadyMs });
  await Page.waitForTimeout(600);

  Assert.equal(await Canvas.getAttribute("data-joint-markers"), "6", "The live arm shows one handle per joint");

  // Moving the target away from the current pose brings up the preview arm, and
  // with it a second set of handles.
  const TargetY = Page.getByRole("spinbutton", { name: "Target Y", exact: true });
  await TargetY.fill("30");
  await TargetY.press("Enter");
  await Page.waitForTimeout(500);
  Assert.equal(await Canvas.getAttribute("data-joint-markers"), "12", "The target pose preview brings its own handles");

  const Live = await HandlePoint("Live", "elbow_flex");
  Assert.ok(Live, "The live elbow handle is on screen");
  await Page.mouse.move(Live.X, Live.Y);
  await Page.waitForTimeout(120);
  Assert.equal(await Canvas.getAttribute("data-hovered-joint"), "Live:elbow_flex", "Hovering a handle reports it");
  Assert.equal(await Canvas.evaluate(Element => getComputedStyle(Element).cursor), "pointer", "A handle shows a pointer cursor");

  await Page.mouse.click(Live.X, Live.Y);
  await Page.waitForTimeout(200);
  Assert.equal(await Canvas.getAttribute("data-selected-joint"), "elbow_flex", "Clicking a handle selects that joint");
  Assert.equal((await Page.locator(".cad-selection-badge strong").textContent()).toLowerCase(), "elbow flex");
  Assert.equal(await Page.locator('.servo-row[data-joint="elbow_flex"].joint-selected').count(), 1, "The joint panel follows the scene");

  await Page.mouse.click(Live.X, Live.Y);
  await Page.waitForTimeout(200);
  Assert.equal(await Canvas.getAttribute("data-selected-joint"), "", "Clicking the same handle again clears the selection");

  const Shoulder = await HandlePoint("Live", "shoulder_pan");
  Assert.ok(Shoulder, "The shoulder handle is on screen");
  await Page.mouse.click(Shoulder.X, Shoulder.Y);
  await Page.waitForTimeout(200);
  Assert.equal(await Canvas.getAttribute("data-selected-joint"), "shoulder_pan");

  // ---- Turning a joint by dragging it in the 3D view ----
  // Same path as the side-panel slider: the target raw changes and move_servo
  // goes out (throttled to one per 90 ms per servo), held inside the joint's
  // configured range. Selecting alone, or pressing without moving, sends nothing.
  await Page.evaluate(() => { window.HudControls.enableDamping = false; document.activeElement?.blur?.(); });
  const Elbow = await HandlePoint("Live", "elbow_flex");
  await Page.mouse.click(Elbow.X, Elbow.Y);
  await Page.waitForFunction(() => document.querySelector("canvas[data-cad-ready=true]").dataset.selectedJoint === "elbow_flex");
  await Page.waitForTimeout(400);
  Assert.deepEqual(Commands.filter(Command => Command.type === "move_servo"), [], "Selecting joints never moves the arm");
  const Servo = State.servos.elbow_flex, StartRaw = Servo.target_raw, StartFraction = Fraction(Servo, StartRaw);
  Assert.ok(StartFraction > .5 && StartFraction <= 1, "Fixture: the elbow starts in the upper half of its range");
  let Dial = await DialGeometry();
  Assert.ok(Dial, "The selected joint shows its arc");
  const Count = Dial.Points.length - 1;
  // Grab the arc where the target sits (the knob), then walk the pointer along
  // the drawn arc toward the low end of the range: the joint must follow it.
  const Grab = Math.round(StartFraction * Count), Walk = 25, GrabPoint = Dial.Points[Grab];
  for (const [Name, Point] of [["knob", GrabPoint], ["handle", Elbow]]) {
    Assert.ok(!Dial.Target || Math.hypot(Point.X - Dial.Target.X, Point.Y - Dial.Target.Y) > 30, `Fixture: the target ball is clear of the elbow's ${Name}`);
    Assert.equal(await Page.evaluate(({ X, Y }) => document.elementFromPoint(X, Y) === document.querySelector("canvas[data-cad-ready=true]"), Point), true, `Fixture: nothing covers the elbow's ${Name}`);
  }
  const CameraBefore = await Camera();
  await Page.mouse.move(GrabPoint.X, GrabPoint.Y);
  await Page.mouse.down();
  await Page.waitForTimeout(150);
  const SentOnPress = Moves("elbow_flex").length;
  for (let Index = Grab - 1; Index >= Grab - Walk; Index--) await Page.mouse.move(Dial.Points[Index].X, Dial.Points[Index].Y);
  const MidDrag = { Drag: await Canvas.getAttribute("data-joint-drag"), Cursor: await Cursor(), Badge: await Page.locator(".cad-selection-badge.dragging").count() };
  await Page.mouse.up();
  await Page.waitForTimeout(350);
  const Turned = Moves("elbow_flex");
  Assert.ok(Turned.length > 0, "Dragging the selected joint's dial in the 3D view sends move_servo for that joint");
  Assert.equal(SentOnPress, 0, "Pressing the dial alone sends nothing");
  const Final = Turned.at(-1).raw, Expected = StartFraction - Walk / Count;
  Assert.ok(Fraction(Servo, Final) < StartFraction, `The joint turns the way the pointer turned the dial (${StartRaw} -> ${Final})`);
  Assert.ok(Math.abs(Fraction(Servo, Final) - Expected) < .03, `The joint turns as far as the dial was turned: fraction ${Fraction(Servo, Final).toFixed(3)}, expected ${Expected.toFixed(3)}`);
  Assert.equal(Commands.filter(Command => Command.type === "move_servo" && Command.name !== "elbow_flex").length, 0, "Only the dragged joint is commanded");
  Assert.deepEqual(MidDrag, { Drag: "elbow_flex", Cursor: "grabbing", Badge: 1 }, "While turning: the canvas reports the joint, a grabbing cursor, a lit badge");
  const CameraAfter = await Camera();
  Assert.ok(Distance(CameraBefore.Target, CameraAfter.Target) < 1e-6 && Distance(CameraBefore.Offset, CameraAfter.Offset) < 1e-6, "Turning a joint never pans or orbits the camera");
  Assert.equal(await Canvas.getAttribute("data-joint-drag"), "", "The drag ends on release");
  Assert.equal(await Canvas.getAttribute("data-selected-joint"), "elbow_flex", "A drag keeps the joint selected");
  // The side panel shows the same target as the 3D view: one command path.
  await Page.waitForFunction(Raw => document.querySelector('[data-joint="elbow_flex"] .joint-fine-step b')?.textContent === String(Raw), Final);
  Assert.equal(Number(await Page.getByRole("slider", { name: "elbow flex target", exact: true }).inputValue()), Math.round((1 - Fraction(Servo, Final)) * 1000),
    "The elbow slider moved to the dragged target");

  // The knob marks the target. Past the end of the range the joint stops at its
  // limit and never beyond, and turning back moves it again straight away.
  Dial = await DialGeometry();
  Assert.ok(Dial.Knob, "The selected joint shows a knob at its target");
  await Page.mouse.move(Dial.Knob.X + 40, Dial.Knob.Y + 40);
  await Page.mouse.move(Dial.Knob.X, Dial.Knob.Y);
  await Page.waitForTimeout(120);
  Assert.equal(await Canvas.getAttribute("data-hovered-joint"), "Dial:elbow_flex", "Hovering the knob reports the dial");
  Assert.equal(await Cursor(), "grab", "The knob shows a grab cursor");
  const KnobAt = Math.round(Fraction(Servo, Final) * Count);
  await Page.mouse.down();
  for (let Index = KnobAt + 1; Index <= Count; Index++) await Page.mouse.move(Dial.Points[Index].X, Dial.Points[Index].Y);
  // On past the end: keep circling the hub the same way for another 70 degrees on screen.
  const End = Dial.Points[Count], Before = Dial.Points[Count - 1], Hub = Dial.Center;
  const Sense = Math.sign((Before.X - Hub.X) * (End.Y - Hub.Y) - (Before.Y - Hub.Y) * (End.X - Hub.X));
  const Radius = Math.hypot(End.X - Hub.X, End.Y - Hub.Y), Start = Math.atan2(End.Y - Hub.Y, End.X - Hub.X);
  for (let Step = 1; Step <= 14; Step++) {
    const Angle = Start + Sense * Step * 5 * Math.PI / 180;
    await Page.mouse.move(Hub.X + Radius * Math.cos(Angle), Hub.Y + Radius * Math.sin(Angle));
  }
  await Page.waitForTimeout(250);
  Assert.equal(Moves("elbow_flex").at(-1).raw, Servo.range_max, "Dragging past the end of the range stops at the joint's limit");
  const Back = Start + Sense * 4 * 5 * Math.PI / 180;
  await Page.mouse.move(Hub.X + Radius * Math.cos(Back), Hub.Y + Radius * Math.sin(Back));
  await Page.mouse.move(Hub.X + Radius * Math.cos(Start - Sense * .2), Hub.Y + Radius * Math.sin(Start - Sense * .2));
  await Page.mouse.up();
  await Page.waitForTimeout(250);
  Assert.ok(Fraction(Servo, Moves("elbow_flex").at(-1).raw) < 1, "Turning back from the limit moves the joint at once");
  for (const Command of Moves("elbow_flex")) Assert.ok(Fraction(Servo, Command.raw) <= 1 + 1e-9, `Every command stays inside the configured range (raw ${Command.raw})`);

  // Grabbing the joint's own handle and pulling it along the arc's direction
  // turns it too; a plain click on it still clears the selection (checked above).
  Dial = await DialGeometry();
  const HubPoint = await HandlePoint("Live", "elbow_flex"), HubFraction = Fraction(Servo, Servo.target_raw);
  const Near = Math.min(Count - 1, Math.max(1, Math.round(HubFraction * Count)));
  const Down = { X: Dial.Points[Near - 1].X - Dial.Points[Near + 1].X, Y: Dial.Points[Near - 1].Y - Dial.Points[Near + 1].Y };
  const DownLength = Math.hypot(Down.X, Down.Y);
  await Page.mouse.move(HubPoint.X, HubPoint.Y);
  await Page.mouse.down();
  for (let Step = 1; Step <= 6; Step++) await Page.mouse.move(HubPoint.X + Down.X / DownLength * 5 * Step, HubPoint.Y + Down.Y / DownLength * 5 * Step);
  await Page.mouse.up();
  await Page.waitForTimeout(300);
  Assert.ok(Fraction(Servo, Moves("elbow_flex").at(-1).raw) < HubFraction - .005, "Pulling the joint's handle along the arc turns it that way");
  Assert.equal(await Canvas.getAttribute("data-selected-joint"), "elbow_flex", "Dragging the handle keeps the joint selected");

  // Escape stops a drag where it is: nothing more is sent after it.
  Dial = await DialGeometry();
  const EscapeAt = Math.round(Fraction(Servo, Servo.target_raw) * Count);
  await Page.mouse.move(Dial.Knob.X, Dial.Knob.Y);
  await Page.mouse.down();
  for (let Index = EscapeAt - 1; Index >= EscapeAt - 6; Index--) await Page.mouse.move(Dial.Points[Index].X, Dial.Points[Index].Y);
  await Page.keyboard.press("Escape");
  await Page.waitForTimeout(250);
  const SentBeforeEscape = Moves("elbow_flex").length;
  Assert.equal(await Canvas.getAttribute("data-joint-drag"), "", "Escape ends the drag");
  const CameraAtEscape = await Camera();
  for (let Index = EscapeAt - 7; Index >= EscapeAt - 14; Index--) await Page.mouse.move(Dial.Points[Index].X, Dial.Points[Index].Y);
  await Page.mouse.up();
  await Page.waitForTimeout(250);
  Assert.equal(Moves("elbow_flex").length, SentBeforeEscape, "Nothing is sent after Escape");
  Assert.ok(Distance(CameraAtEscape.Target, (await Camera()).Target) < 1e-6, "Moving on after Escape does not pan the camera");

  // The camera still pans and orbits when no handle is under the pointer, and
  // a right drag that starts on the dial orbits rather than turning the joint.
  const SentBeforeCamera = Commands.length;
  const Blank = await Page.evaluate(() => {
    const Scene = window.HudScene, Camera = window.HudControls.object, Ray = window.HudRaycaster;
    const Element = document.querySelector("canvas[data-cad-ready=true]"), Canvas = Element.getBoundingClientRect();
    const Project = Point => { const P = Point.clone().project(Camera); return { X: Canvas.left + (P.x * .5 + .5) * Canvas.width, Y: Canvas.top + (-P.y * .5 + .5) * Canvas.height }; };
    const Guide = Scene.getObjectByName("SelectedJointGuide"), Ring = Scene.getObjectByName("SelectedJointArc").geometry.attributes.instanceStart.data.array;
    const Avoid = [...Scene.getObjectByName("JointMarkers").children.filter(Group => Group.visible).map(Group => Group.position),
      Scene.getObjectByName("SelectedJointKnob").position, Guide.position, Scene.getObjectByName("TargetApproachRay").parent.position,
      ...Array.from({ length: Ring.length / 3 }, (_, Index) => Guide.position.clone().set(Ring[3 * Index], Ring[3 * Index + 1], Ring[3 * Index + 2]).multiplyScalar(Guide.scale.x).add(Guide.position))]
      .map(Project);
    const Arm = Scene.getObjectByName("ControlledArm").children;
    for (let Row = 2; Row <= 8; Row++) for (let Column = 1; Column <= 9; Column++) {
      const X = Canvas.left + Canvas.width * Column / 10, Y = Canvas.top + Canvas.height * Row / 10;
      if (document.elementFromPoint(X, Y) !== Element || Avoid.some(Point => Math.hypot(Point.X - X, Point.Y - Y) < 60)) continue;
      Ray.setFromCamera({ x: Column / 5 - 1, y: 1 - Row / 5 }, Camera);
      if (Ray.intersectObjects(Arm, false).length) continue;
      return { X, Y };
    }
    return null;
  });
  Assert.ok(Blank, "Fixture: some empty space in the 3D view");
  let Before3d = await Camera();
  await Page.mouse.move(Blank.X, Blank.Y);
  await Page.mouse.down();
  await Page.mouse.move(Blank.X + 45, Blank.Y + 20, { steps: 10 });
  await Page.mouse.up();
  let After3d = await Camera();
  Assert.ok(Distance(Before3d.Target, After3d.Target) > .1, "Left drag on empty space still pans the camera");
  Assert.ok(Distance(Before3d.Offset, After3d.Offset) < 1e-6, "Panning keeps the camera's orientation");
  Dial = await DialGeometry();
  Before3d = After3d;
  await Page.mouse.move(Dial.Knob.X, Dial.Knob.Y);
  await Page.mouse.down({ button: "right" });
  await Page.mouse.move(Dial.Knob.X + 45, Dial.Knob.Y + 20, { steps: 10 });
  await Page.mouse.up({ button: "right" });
  After3d = await Camera();
  Assert.ok(Distance(Before3d.Target, After3d.Target) < 1e-6 && Distance(Before3d.Offset, After3d.Offset) > .1, "Right drag still orbits, even starting on the dial");
  await Page.waitForTimeout(250);
  Assert.equal(Commands.length, SentBeforeCamera, "Moving the camera never commands the arm");
  Assert.equal(await Canvas.getAttribute("data-selected-joint"), "elbow_flex");

  // The handles are display furniture: they can be switched off.
  await Page.getByRole("button", { name: "Settings", exact: true }).click();
  await Page.getByRole("checkbox", { name: "Show joint handles", exact: true }).uncheck();
  await Page.waitForTimeout(250);
  Assert.equal(await Canvas.getAttribute("data-joint-markers"), "0", "Handles can be turned off");
  await Page.getByRole("checkbox", { name: "Show joint handles", exact: true }).check();
  await Page.waitForTimeout(250);
  Assert.ok(Number(await Canvas.getAttribute("data-joint-markers")) >= 6, "Handles come back");

  Assert.deepEqual(Errors, [], "No page errors");
  console.log("PASS TestJointHandles");
} finally {
  for (const Timer of Timers) clearInterval(Timer);
  await Browser.close(); await Server.Close();
}
