// The depth overlay follows the live view at every pane size: in a window taller than 16:9, wider than 16:9, while
// a resize drags, and full screen, the overlay's drawn picture (the pixels on screen, not only its box) covers the
// video's picture exactly. Aaran reported it did not resize with the video pane (GraspPhase1Spec.md, "Not
// addressed"). Bridge and camera are faked; it serves this checkout on 5191-5199 (UiTestLib.mjs).
import { chromium as Chromium } from "playwright";
import { LaunchOptions, StartUiServer, BackendPython } from "./UiTestLib.mjs";
import { execFileSync as Execute } from "node:child_process";
import { fileURLToPath as FilePath } from "node:url";
import Path from "node:path";
import Assert from "node:assert/strict";
const Root = Path.dirname(FilePath(import.meta.url)), Backend = Path.resolve(Root, "../back end");
const Python = BackendPython();
const State = JSON.parse(Execute(Python, ["-c", "import json, bridge_server; print(json.dumps(bridge_server.BridgeController().state_snapshot()))"], { cwd: Backend, encoding: "utf8" }));
State.arm_connected = true; State.torque = true; State.port = "FAKE";
// A white 1280x720 frame, and the bridge's half-resolution overlay (640x360) all opaque blue, so the blue on screen
// is exactly the overlay's picture and any white is video the overlay misses.
const [Jpeg, Png] = Execute(Python, ["-c", `import base64, cv2, numpy as np
Frame = np.full((720, 1280, 3), 255, np.uint8)
Overlay = np.zeros((360, 640, 4), np.uint8); Overlay[..., 0] = 255; Overlay[..., 3] = 255
print(base64.b64encode(cv2.imencode('.jpg', Frame)[1]).decode()); print(base64.b64encode(cv2.imencode('.png', Overlay)[1]).decode())`],
{ encoding: "utf8" }).trim().split(/\r?\n/).map(Line => Buffer.from(Line, "base64"));

const Server = await StartUiServer();
const Browser = await Chromium.launch(LaunchOptions());
const Page = await Browser.newPage({ viewport: { width: 1600, height: 1000 } });
const Errors = [];
Page.on("pageerror", Error => Errors.push(Error.message));
await Page.addInitScript(() => {
  localStorage.setItem("SO101.ViewSettings.v1", JSON.stringify({ CameraSource: "Depth", DepthOverlay: true }));
  if (!sessionStorage.getItem("Placed")) {
    localStorage.setItem("SO101.CameraWindow.v1", JSON.stringify({ X: 600, Y: 300, Width: 300, Height: 360 })); sessionStorage.setItem("Placed", "1");
  }
});
await Page.route("**/bridge-launch/**", Route => Route.fulfill({ json: { ok: true } }));
await Page.route("**/ollama/**", Route => Route.fulfill({ json: { models: [] } }));
await Page.route("**/depth-server/**", Route => Route.fulfill({ json: {} }));
await Page.route("**:8000/**", Route => {
  const Url = new URL(Route.request().url()), After = Number(Url.searchParams.get("after") ?? 0);
  if (Url.pathname === "/ports") return Route.fulfill({ json: [] });
  if (Url.pathname === "/depthcam/status") return Route.fulfill({ json: { ok: true, available: true, running: true, seq: 1, fps: 30, calibrated: false, version: "orbbec-v2" } });
  if (Url.pathname === "/depthcam/frame.jpg") return After >= 1 ? Route.fulfill({ status: 204 })
    : Route.fulfill({ body: Jpeg, contentType: "image/jpeg", headers: { "X-Frame-Seq": "1" } });
  if (Url.pathname === "/depthcam/overlay.png") return After >= 1 ? Route.fulfill({ status: 204 })
    : Route.fulfill({ body: Png, contentType: "image/png", headers: { "X-Frame-Seq": "1", "X-Depth-Range-Cm": "25,150", "Access-Control-Expose-Headers": "X-Frame-Seq, X-Depth-Range-Cm" } });
  return Route.fulfill({ json: { ok: true, calibrated: false, points: [] } });
});
const Timers = new Set();
await Page.routeWebSocket("**/ws", Connected => {
  Connected.send(JSON.stringify(State));
  const Timer = setInterval(() => Connected.send(JSON.stringify(State)), 100); Timers.add(Timer);
  Connected.onClose(() => { clearInterval(Timer); Timers.delete(Timer); });
});

// Where the video draws its picture, from its box and object-fit; and where blue (the overlay) and white (the
// video) are on screen inside the frame, read from a screenshot of it.
const Measure = async Label => {
  const Layout = await Page.evaluate(() => {
    const Frame = document.querySelector(".camera-window .video-frame"), Video = Frame.querySelector("video"), Overlay = Frame.querySelector("canvas.depth-overlay");
    const Rect = Element => { const Box = Element.getBoundingClientRect(); return { X: Box.x, Y: Box.y, W: Box.width, H: Box.height }; };
    return { Frame: Rect(Frame), Video: Rect(Video), Overlay: Rect(Overlay), VideoFit: getComputedStyle(Video).objectFit, OverlayFit: getComputedStyle(Overlay).objectFit,
      VideoSize: [Video.videoWidth, Video.videoHeight], OverlaySize: [Overlay.width, Overlay.height] };
  });
  const Shot = await Page.screenshot({ clip: { x: Layout.Frame.X, y: Layout.Frame.Y, width: Layout.Frame.W, height: Layout.Frame.H } });
  const Drawn = await Page.evaluate(async Data => {
    const Image = new window.Image(); Image.src = `data:image/png;base64,${Data}`; await Image.decode();
    const Canvas = document.createElement("canvas"); Canvas.width = Image.width; Canvas.height = Image.height;
    const Context = Canvas.getContext("2d"); Context.drawImage(Image, 0, 0);
    const Pixels = Context.getImageData(0, 0, Image.width, Image.height).data;
    const Bounds = Test => { let X0 = Infinity, Y0 = Infinity, X1 = -1, Y1 = -1;
      for (let Y = 0; Y < Image.height; Y++) for (let X = 0; X < Image.width; X++) {
        const At = (Y * Image.width + X) * 4; if (!Test(Pixels[At], Pixels[At + 1], Pixels[At + 2])) continue;
        X0 = Math.min(X0, X); Y0 = Math.min(Y0, Y); X1 = Math.max(X1, X); Y1 = Math.max(Y1, Y); }
      return X1 < 0 ? null : { X: X0, Y: Y0, W: X1 - X0 + 1, H: Y1 - Y0 + 1 }; };
    // Blue over white at .55 opacity: blue channel high, red and green clearly lower. Bare white: all three high.
    return { Blue: Bounds((R, G, B) => B > 200 && R < 170 && G < 170), White: Bounds((R, G, B) => R > 235 && G > 235 && B > 235), Width: Image.width, Height: Image.height };
  }, Shot.toString("base64"));
  // The video's picture inside the frame (object-fit contain of a 16:9 frame).
  const Scale = Math.min(Layout.Video.W / 1280, Layout.Video.H / 720);
  const Picture = { X: Layout.Video.X - Layout.Frame.X + (Layout.Video.W - 1280 * Scale) / 2, Y: Layout.Video.Y - Layout.Frame.Y + (Layout.Video.H - 720 * Scale) / 2, W: 1280 * Scale, H: 720 * Scale };
  const Report = JSON.stringify({ Label, Layout, Drawn, Picture });
  if (process.env.SO101_VERBOSE) console.log(Report);
  Assert.equal(Layout.VideoFit, "contain", Report);
  Assert.deepEqual(Layout.VideoSize, [1280, 720], Report);
  Assert.ok(Drawn.Blue, `${Label}: no overlay on screen ${Report}`);
  // The overlay's blue covers the video's picture to within 2 px on every side, and no bare video shows beside it.
  for (const Key of ["X", "Y", "W", "H"]) Assert.ok(Math.abs(Drawn.Blue[Key] - Picture[Key]) <= 2, `${Label}: overlay ${Key} ${Drawn.Blue[Key]} vs picture ${Picture[Key].toFixed(1)} ${Report}`);
  // (A one-pixel sliver at a fractional picture edge is rounding, not a miss.)
  Assert.ok(!Drawn.White || Math.min(Drawn.White.W, Drawn.White.H) <= 1, `${Label}: bare video shows outside the overlay ${Report}`);
  return { Layout, Drawn, Picture };
};

try {
  await Page.goto(Server.Url, { waitUntil: "domcontentloaded" });
  await Page.locator(".camera-window .depth-legend").waitFor();
  await Page.waitForFunction(() => document.querySelector(".camera-window video")?.videoWidth === 1280, null, { timeout: 15000 });
  await Page.waitForFunction(() => document.querySelector(".camera-window canvas.depth-overlay")?.width === 640, null, { timeout: 15000 });
  await Page.mouse.move(5, 500); await Page.waitForTimeout(300);
  const Tall = await Measure("taller than 16:9 (300 x 360)");
  Assert.ok(Tall.Layout.Frame.W / Tall.Layout.Frame.H < 16 / 9 - 0.3);

  // Resize from the bottom-right grip to wider than 16:9, measuring mid-drag and after.
  const Grip = await Page.getByRole("button", { name: "Resize camera window", exact: true }).boundingBox();
  await Page.mouse.move(Grip.x + Grip.width / 2, Grip.y + Grip.height / 2); await Page.mouse.down();
  await Page.mouse.move(Grip.x + 200, Grip.y - 30, { steps: 6 }); await Page.waitForTimeout(200);
  await Measure("mid-drag");
  await Page.mouse.move(Grip.x + 500, Grip.y - 60, { steps: 6 }); await Page.mouse.up(); await Page.waitForTimeout(300);
  const Wide = await Measure("wider than 16:9");
  Assert.ok(Wide.Layout.Frame.W / Wide.Layout.Frame.H > 16 / 9 + 0.3);
  Assert.ok(Wide.Picture.W > Tall.Picture.W * 1.2, `the picture grew with the pane: ${Tall.Picture.W} to ${Wide.Picture.W}`);

  // Full screen, then back.
  await Page.getByRole("button", { name: "Full screen live view", exact: true }).click(); await Page.waitForTimeout(400);
  await Page.mouse.move(5, 5); await Page.waitForTimeout(200);
  const Full = await Measure("full screen");
  Assert.ok(Full.Picture.W > Wide.Picture.W);
  await Page.getByRole("button", { name: "Exit full screen live view", exact: true }).click(); await Page.waitForTimeout(400);
  await Measure("back from full screen");
  // The browser window resized.
  await Page.setViewportSize({ width: 1100, height: 760 }); await Page.waitForTimeout(400);
  await Measure("smaller browser window");
  Assert.deepEqual(Errors, []);
  console.log("PASS: the depth overlay covers the live view's picture exactly taller and wider than 16:9, mid-resize, full screen and after the browser window shrinks.");
} finally {
  Timers.forEach(clearInterval);
  await Browser.close(); await Server.Close();
}
