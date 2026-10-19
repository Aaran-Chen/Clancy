// Hover magnifier (components/HoverLoupe.tsx) on the calibration click surfaces, at a
// 2.25 device pixel ratio: it appears beside the pointer without covering it, flips
// near the viewport edges, zooms 2x-8x with the wheel, shows the full-resolution
// source pixel under its crosshair, and that pixel is exactly what a click posts,
// for the marker preview (unmirrored) and the frozen manual frame (shown mirrored).
// The bridge and camera are faked; nothing reaches port 8000.
// Run: node TestHoverLoupeUi.mjs (it serves this checkout on 5191-5199 itself: UiTestLib.mjs)
import { chromium as Chromium } from "playwright";
import { LaunchOptions, StartUiServer } from "./UiTestLib.mjs";
import { readFileSync as ReadFile, mkdirSync as MakeDirectory } from "node:fs";
import { execFileSync as Execute } from "node:child_process";
import { fileURLToPath as FilePath } from "node:url";
import Path from "node:path";
import Assert from "node:assert/strict";

const Root = Path.dirname(FilePath(import.meta.url)), Backend = Path.resolve(Root, "../back end");
const Python = ReadFile(Path.join(Root, "../.backend-python.txt"), "utf8").replace(/^﻿/, "").trim();
const State = JSON.parse(Execute(Python, ["-c", "import json, bridge_server; print(json.dumps(bridge_server.BridgeController().state_snapshot()))"],
  { cwd: Backend, encoding: "utf8" }));
Object.assign(State, { arm_connected: true, torque: true, port: "FAKE" });
// Every pixel's colour names it: R = x mod 256, G = y mod 256, B = 32 (x div 256) + 8 (y div 256).
const Preview = "data:image/png;base64," + Execute(Python, ["-c", `import base64, cv2, numpy as np
Y, X = np.mgrid[0:720, 0:1280]
Image = np.dstack([32 * (X >> 8) + 8 * (Y >> 8), Y & 255, X & 255]).astype(np.uint8)
print(base64.b64encode(cv2.imencode('.png', Image)[1]).decode())`], { encoding: "utf8", maxBuffer: 1 << 26 }).trim();
const Decode = ([R, G, B]) => ({ X: (B >> 5) * 256 + R, Y: ((B & 31) >> 3) * 256 + G });

const Server = await StartUiServer();
const Browser = await Chromium.launch(LaunchOptions({ args: ["--use-fake-device-for-media-stream", "--use-fake-ui-for-media-stream"] }));
const Page = await Browser.newPage({ viewport: { width: 1440, height: 1000 }, deviceScaleFactor: 2.25, permissions: ["camera"] });
const Errors = [], Checks = [], Probes = [];
Page.on("pageerror", Error => Errors.push(Error.message));
await Page.route("**/bridge-launch/**", Route => Route.fulfill({ json: { ok: true } }));
await Page.route("**/ollama/**", Route => Route.fulfill({ json: { models: [] } }));
await Page.route("**/depth-server/**", Route => Route.fulfill({ json: {} }));
await Page.route("**/session-log/**", Route => Route.fulfill({ json: { ok: true } }));
await Page.route("**:8000/**", async Route => {
  const Url = new URL(Route.request().url()), Payload = Route.request().postDataJSON();
  if (Url.pathname === "/selfcal/check") {
    Checks.push(Payload);
    return Route.fulfill({ json: { ok: true, detected: Boolean(Payload?.hint), preview: Preview, method: "red", frames: 3, frames_detected: 3,
      area_px: 300, frame_size: [1280, 720], centroid: Payload?.hint ? [Payload.hint.u * 1280, Payload.hint.v * 720] : null } });
  }
  if (Url.pathname === "/calibration/probe") { Probes.push(Payload); return Route.fulfill({ json: { ok: true, point_count: Probes.length } }); }
  if (Url.pathname === "/depthcam/status") return Route.fulfill({ json: { ok: true, available: false } });
  if (Url.pathname === "/ports") return Route.fulfill({ json: ["FAKE"] });
  return Route.fulfill({ json: { ok: true, calibrated: false, points: [], point_count: 0, min_points: 4, recommended_points: 6 } });
});
const Timers = new Set();
await Page.routeWebSocket("**/ws", Socket => {
  Socket.send(JSON.stringify(State));
  const Timer = setInterval(() => Socket.send(JSON.stringify(State)), 100); Timers.add(Timer);
  Socket.onClose(() => { clearInterval(Timer); Timers.delete(Timer); });
});

// The loupe's state, its centre pixel, and where the pointer's pixel is by an independent calculation.
const Loupe = Page.locator(".hover-loupe[data-visible='true']");
async function Inspect(Selector, X, Y, Mirrored) {
  return Page.evaluate(([Selector, X, Y, Mirrored]) => {
    const Image = document.querySelector(Selector), Box = document.querySelector(".hover-loupe[data-visible='true']");
    const Rect = Image.getBoundingClientRect(), Border = parseFloat(getComputedStyle(Image).borderLeftWidth) || 0;
    const Width = Rect.width - 2 * Border, Height = Rect.height - 2 * Border;
    const Shown = (X - Rect.left - Border) * Image.naturalWidth / Width;
    const Expected = { Column: Math.floor(Mirrored ? Image.naturalWidth - Shown : Shown), Row: Math.floor((Y - Rect.top - Border) * Image.naturalHeight / Height) };
    if (!Box) return { Expected, Visible: false };
    const Canvas = Box.querySelector("canvas"), Context = Canvas.getContext("2d");
    const Centre = Array.from(Context.getImageData(Math.floor(Canvas.width / 2), Math.floor(Canvas.height / 2), 1, 1).data);
    const Probe = document.createElement("canvas"); Probe.width = Image.naturalWidth; Probe.height = Image.naturalHeight;
    const ProbeContext = Probe.getContext("2d"); ProbeContext.drawImage(Image, 0, 0);
    const Column = Number(Box.dataset.column), Row = Number(Box.dataset.row);
    const Source = Array.from(ProbeContext.getImageData(Column, Row, 1, 1).data);
    const LoupeRect = Box.getBoundingClientRect();
    return { Expected, Visible: true, Column, Row, Zoom: Number(Box.dataset.zoom), Centre, Source, CanvasWidth: Canvas.width,
      Box: { Left: LoupeRect.left, Top: LoupeRect.top, Right: LoupeRect.right, Bottom: LoupeRect.bottom },
      Viewport: { Width: innerWidth, Height: innerHeight }, Pointer: getComputedStyle(Box).pointerEvents, Rendering: getComputedStyle(Canvas).imageRendering };
  }, [Selector, X, Y, Mirrored]);
}
function AssertBesidePointer(State, X, Y) {
  Assert.ok(State.Box.Left >= 0 && State.Box.Top >= 0 && State.Box.Right <= State.Viewport.Width && State.Box.Bottom <= State.Viewport.Height,
    `the magnifier stays inside the viewport: ${JSON.stringify(State.Box)}`);
  Assert.ok(!(X >= State.Box.Left && X <= State.Box.Right && Y >= State.Box.Top && Y <= State.Box.Bottom), "the magnifier never covers the aimed point");
  Assert.equal(State.Pointer, "none");
  Assert.equal(State.Rendering, "pixelated");
}

try {
  await Page.goto(Server.Url, { waitUntil: "domcontentloaded" });
  await Page.getByRole("button", { name: "Settings", exact: true }).click();
  await Page.getByRole("button", { name: "Calibrate", exact: true }).click();
  await Page.getByRole("button", { name: "Check marker", exact: true }).click();
  const Image = Page.getByAltText("Click the red tape on the gripper tip");
  await Image.waitFor();
  await Page.waitForFunction(() => document.querySelector("img[alt='Click the red tape on the gripper tip']")?.naturalWidth === 1280);
  const Box = await Image.boundingBox();

  // Hover: beside the pointer, the full-resolution pixel under the crosshair is the one under the pointer.
  let X = Box.x + Box.width * 0.37, Y = Box.y + Box.height * 0.41;
  await Page.mouse.move(X - 5, Y - 5); await Page.mouse.move(X, Y);
  await Loupe.waitFor();
  let Seen = await Inspect("img[alt='Click the red tape on the gripper tip']", X, Y, false);
  AssertBesidePointer(Seen, X, Y);
  Assert.ok(Seen.Box.Left > X && Seen.Box.Top > Y, "away from edges the magnifier sits right of and below the pointer");
  Assert.deepEqual({ Column: Seen.Column, Row: Seen.Row }, Seen.Expected);
  Assert.deepEqual(Seen.Centre, Seen.Source, "the crosshair cell shows the source pixel itself");
  Assert.deepEqual(Decode(Seen.Centre), { X: Seen.Column, Y: Seen.Row }, "and that pixel is the one named, at full resolution");
  Assert.equal(Seen.CanvasWidth, Math.round(184 * 2.25), "the magnifier draws at device resolution");
  Assert.equal(Seen.Zoom, 4);

  // The click posts that same pixel's centre.
  const ChecksBefore = Checks.length;
  await Page.mouse.click(X, Y);
  for (let Wait = 0; Checks.length === ChecksBefore && Wait < 50; Wait += 1) await Page.waitForTimeout(100);
  const Hint = Checks.at(-1).hint;
  Assert.ok(Math.abs(Hint.u * 1280 - (Seen.Column + 0.5)) < 1e-6 && Math.abs(Hint.v * 720 - (Seen.Row + 0.5)) < 1e-6,
    `click hint ${JSON.stringify(Hint)} names pixel ${Seen.Column}, ${Seen.Row}`);

  // Wheel zooms 2x to 8x, the page does not scroll, and the crosshair pixel stays put.
  await Page.mouse.move(X + 1, Y); await Page.mouse.move(X, Y);
  for (let Step = 0; Step < 6; Step += 1) await Page.mouse.wheel(0, -100);
  await Page.waitForTimeout(100);
  Seen = await Inspect("img[alt='Click the red tape on the gripper tip']", X, Y, false);
  Assert.equal(Seen.Zoom, 8);
  Assert.deepEqual(Decode(Seen.Centre), { X: Seen.Column, Y: Seen.Row });
  for (let Step = 0; Step < 9; Step += 1) await Page.mouse.wheel(0, 100);
  await Page.waitForTimeout(100);
  Seen = await Inspect("img[alt='Click the red tape on the gripper tip']", X, Y, false);
  Assert.equal(Seen.Zoom, 2);
  await Page.mouse.wheel(0, -100); await Page.mouse.wheel(0, -100);
  await Page.waitForTimeout(100);

  // Near the right and bottom edges of the viewport it flips to the other side, fully visible.
  await Page.setViewportSize({ width: 900, height: 700 });
  await Image.scrollIntoViewIfNeeded();
  const Corner = await Image.boundingBox();
  X = Corner.x + Corner.width * 0.97; Y = Math.min(Corner.y + Corner.height * 0.95, 700 - 25);
  Assert.ok(X + 24 + 184 > 900 && Y + 24 + 184 > 700 && Y > Corner.y + 5, `a corner point near both viewport edges: ${X}, ${Y}`);
  await Page.mouse.move(X - 3, Y); await Page.mouse.move(X, Y);
  Seen = await Inspect("img[alt='Click the red tape on the gripper tip']", X, Y, false);
  Assert.ok(Seen.Visible);
  AssertBesidePointer(Seen, X, Y);
  Assert.ok(Seen.Box.Right < X && Seen.Box.Bottom < Y, `near the corner it sits left of and above the pointer: ${JSON.stringify(Seen.Box)}`);
  Assert.deepEqual({ Column: Seen.Column, Row: Seen.Row }, Seen.Expected);
  await Page.setViewportSize({ width: 1440, height: 1000 });

  // Leaving the picture hides it.
  await Page.mouse.move(5, 5);
  await Page.waitForFunction(() => !document.querySelector(".hover-loupe[data-visible='true']"));

  // The frozen manual frame is a mirrored capture shown flipped back: the magnifier and the probe click agree.
  const Frozen = Page.getByAltText("Frozen camera frame: click the gripper tip");
  // The fake webcam may still be starting when the panel opened: refreeze until a frame exists.
  for (let Attempt = 0; Attempt < 40 && !(await Frozen.count()); Attempt += 1) {
    await Page.getByRole("button", { name: "Refreeze frame", exact: true }).click();
    await Page.waitForTimeout(250);
  }
  await Frozen.waitFor();
  await Page.waitForFunction(() => document.querySelector("img[alt='Frozen camera frame: click the gripper tip']")?.naturalWidth > 0);
  const Mirrored = await Frozen.evaluate(Node => getComputedStyle(Node).transform !== "none");
  Assert.ok(Mirrored, "the fake webcam faces the arm, so its capture is mirrored and displayed flipped back");
  await Frozen.scrollIntoViewIfNeeded();
  const FrozenBox = await Frozen.boundingBox();
  X = FrozenBox.x + FrozenBox.width * 0.23; Y = FrozenBox.y + FrozenBox.height * 0.62;
  await Page.mouse.move(X - 4, Y); await Page.mouse.move(X, Y);
  await Loupe.waitFor();
  Seen = await Inspect("img[alt='Frozen camera frame: click the gripper tip']", X, Y, true);
  AssertBesidePointer(Seen, X, Y);
  Assert.deepEqual({ Column: Seen.Column, Row: Seen.Row }, Seen.Expected, "the mirrored mapping names the capture pixel");
  Assert.deepEqual(Seen.Centre, Seen.Source, "the crosshair cell shows that capture pixel");
  MakeDirectory(Path.join(Backend, "TestArtifacts"), { recursive: true });
  await Page.screenshot({ path: Path.join(Backend, "TestArtifacts", "HoverLoupeUi.png") });
  await Page.mouse.click(X, Y);
  await Page.waitForFunction(() => document.body.innerText.includes("Point 1 recorded"));
  Assert.equal(Probes.length, 1);
  Assert.equal(Probes[0].mirrored, true);
  Assert.equal(Probes[0].u, Seen.Column + 0.5);
  Assert.equal(Probes[0].v, Seen.Row + 0.5);
  Assert.deepEqual(Probes[0].image_size, await Frozen.evaluate(Node => [Node.naturalWidth, Node.naturalHeight]));

  Assert.deepEqual(Errors, []);
  console.log("PASS: magnifier beside the pointer at 2.25 DPR, full-resolution pixel under the crosshair = clicked pixel (plain and mirrored frames), wheel zoom 2x-8x, edge flip, hides on leave.");
} finally {
  Timers.forEach(clearInterval);
  await Browser.close(); await Server.Close();
}
