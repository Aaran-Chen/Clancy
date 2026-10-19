// Live view: hovering with the depth overlay on reads the depth under the cursor into the
// bottom-left legend, and every edge of the window resizes it. Bridge and camera are faked.
import { chromium as Chromium } from "playwright";
import { LaunchOptions, StartUiServer } from "./UiTestLib.mjs";
import { readFileSync as ReadFile } from "node:fs";
import { execFileSync as Execute } from "node:child_process";
import { fileURLToPath as FilePath } from "node:url";
import Path from "node:path";
import Assert from "node:assert/strict";
const Root = Path.dirname(FilePath(import.meta.url)), Backend = Path.resolve(Root, '../back end');
const Python = ReadFile(Path.join(Root, '../.backend-python.txt'), 'utf8').replace(/^﻿/, '').trim();
const State = JSON.parse(Execute(Python, ['-c', 'import json, bridge_server; print(json.dumps(bridge_server.BridgeController().state_snapshot()))'], { cwd: Backend, encoding: 'utf8' }));
State.arm_connected = true; State.torque = true; State.port = 'FAKE';
const [Jpeg, Png] = Execute(Python, ['-c', `import base64, cv2, numpy as np
Frame = np.full((720, 1280, 3), 90, np.uint8); cv2.circle(Frame, (640, 360), 60, (40, 40, 220), -1)
Overlay = np.zeros((360, 640, 4), np.uint8); Overlay[..., 0] = 255; Overlay[..., 3] = 255
print(base64.b64encode(cv2.imencode('.jpg', Frame)[1]).decode()); print(base64.b64encode(cv2.imencode('.png', Overlay)[1]).decode())`],
{ encoding: 'utf8' }).trim().split(/\r?\n/).map(Line => Buffer.from(Line, 'base64'));

const Server = await StartUiServer();
const Browser = await Chromium.launch(LaunchOptions());
const Page = await Browser.newPage({ viewport: { width: 1920, height: 1080 } });
const Errors = [], Lookups = [], Pins = [];
let DepthAt = 'ok';
Page.on('pageerror', Error => Errors.push(Error.message));
await Page.addInitScript(() => {
  localStorage.setItem('SO101.ViewSettings.v1', JSON.stringify({ CameraSource: 'Depth', DepthOverlay: true }));
  if (!sessionStorage.getItem('Placed')) {  // mid-screen, so resizing is not stopped by the screen edge
    localStorage.setItem('SO101.CameraWindow.v1', JSON.stringify({ X: 700, Y: 400, Width: 360, Height: 280 })); sessionStorage.setItem('Placed', '1');
  }
});
await Page.route('**/bridge-launch/**', Route => Route.fulfill({ json: { ok: true } }));
await Page.route('**/ollama/**', Route => Route.fulfill({ json: { models: [] } }));
await Page.route('**/depth-server/**', Route => Route.fulfill({ json: {} }));
await Page.route('**:8000/**', Route => {
  const Url = new URL(Route.request().url()), After = Number(Url.searchParams.get('after') ?? 0);
  if (Url.pathname === '/ports') return Route.fulfill({ json: [] });
  if (Url.pathname === '/depthcam/status') return Route.fulfill({ json: { ok: true, available: true, running: true, seq: 1, fps: 30, calibrated: false, version: 'orbbec-v2' } });
  if (Url.pathname === '/depthcam/frame.jpg') return After >= 1 ? Route.fulfill({ status: 204 })
    : Route.fulfill({ body: Jpeg, contentType: 'image/jpeg', headers: { 'X-Frame-Seq': '1' } });
  if (Url.pathname === '/depthcam/overlay.png') return After >= 1 ? Route.fulfill({ status: 204 })
    : Route.fulfill({ body: Png, contentType: 'image/png', headers: { 'X-Frame-Seq': '1', 'X-Depth-Range-Cm': '25,150', 'Access-Control-Expose-Headers': 'X-Frame-Seq, X-Depth-Range-Cm' } });
  if (Url.pathname === '/depthcam/depth-at') {
    Lookups.push([Number(Url.searchParams.get('u')), Number(Url.searchParams.get('v'))]);
    if (DepthAt === 'old bridge') return Route.fulfill({ status: 404, json: { detail: 'Not Found' } });
    return Route.fulfill({ json: DepthAt === 'hole' ? { ok: false, error: 'no depth at that pixel' }
      : { ok: true, depth_cm: 48.24, display_cm: { x: 3, y: 30.04, z: 8 }, seq: 1 } });
  }
  if (Url.pathname === '/depthcam/point') Pins.push(Url.pathname);
  return Route.fulfill({ json: { ok: true, calibrated: false, points: [] } });
});
const Timers = new Set();
await Page.routeWebSocket('**/ws', Connected => {
  Connected.send(JSON.stringify(State));
  const Timer = setInterval(() => Connected.send(JSON.stringify(State)), 100); Timers.add(Timer);
  Connected.onClose(() => { clearInterval(Timer); Timers.delete(Timer); });
});

try {
  await Page.goto(Server.Url, { waitUntil: 'domcontentloaded' });
  const Legend = Page.locator('.camera-window .depth-legend'), Video = Page.locator('.camera-window video');
  await Legend.waitFor();
  await Page.waitForFunction(() => document.querySelector('.camera-window video')?.videoWidth === 1280, null, { timeout: 15000 });
  await Page.waitForFunction(() => document.querySelector('.camera-window .depth-legend')?.textContent === 'Depth: blue 25 cm to red 150 cm');

  // Hover the middle of the picture: the legend reads the depth there, from the right pixel.
  const Box = await Video.boundingBox();
  await Page.mouse.move(Box.x + Box.width / 2, Box.y + Box.height / 2);
  await Page.waitForFunction(() => document.querySelector('.camera-window .depth-legend')?.textContent === 'Depth 48.2 cm · X 3.0, Y 30.0, Z 8.0 cm');
  const [U, V] = Lookups.at(-1);
  Assert.ok(Math.abs(U - 640) < 2 && Math.abs(V - 360) < 2, `looked up ${U}, ${V}`);
  Assert.equal(await Page.locator('.camera-window .video-frame').evaluate(Element => getComputedStyle(Element).cursor), 'crosshair');
  // Resting still keeps reading (so the number follows moving objects), at most ten times a second.
  const Before = Lookups.length; await Page.waitForTimeout(1000);
  Assert.ok(Lookups.length - Before >= 4 && Lookups.length - Before <= 11, `${Lookups.length - Before} readings in a second`);
  DepthAt = 'hole';
  await Page.waitForFunction(() => document.querySelector('.camera-window .depth-legend')?.textContent === 'No depth reading here');
  // Leaving the picture restores the colour legend and stops the readings.
  await Page.mouse.move(5, 540);
  await Page.waitForFunction(() => document.querySelector('.camera-window .depth-legend')?.textContent === 'Depth: blue 25 cm to red 150 cm');
  const Stopped = Lookups.length; await Page.waitForTimeout(400);
  Assert.equal(Lookups.length, Stopped, 'no readings after the pointer leaves');
  DepthAt = 'old bridge';
  await Page.mouse.move(Box.x + Box.width / 2, Box.y + Box.height / 2);
  await Page.waitForFunction(() => document.querySelector('.camera-window .depth-legend')?.textContent === 'Restart the bridge to read depth on hover');
  await Page.mouse.move(5, 540);

  // Resize from the left edge and the top-left corner: the right and bottom edges stay put.
  const Window = Page.locator('.camera-window');
  const Drag = async (X, Y, DeltaX, DeltaY) => {
    await Page.mouse.move(X, Y); await Page.mouse.down();
    await Page.mouse.move(X + DeltaX / 2, Y + DeltaY / 2, { steps: 4 }); await Page.mouse.move(X + DeltaX, Y + DeltaY, { steps: 4 });
    await Page.mouse.up(); await Page.waitForTimeout(100);
  };
  const Start = await Window.boundingBox();
  await Drag(Start.x + 2, Start.y + Start.height / 2, -120, 0);
  const Wider = await Window.boundingBox();
  Assert.ok(Math.abs(Wider.x - (Start.x - 120)) < 2 && Math.abs(Wider.x + Wider.width - (Start.x + Start.width)) < 2, JSON.stringify({ Start, Wider }));
  // Now wider than 16:9. The video stays inside the frame (an auto grid row used to grow to the
  // video's own 16:9 height and overflow), so the overlay and the hover lookup still line up with it.
  const Layers = await Page.evaluate(() => {
    const Frame = document.querySelector('.camera-window .video-frame'), Video = Frame.querySelector('video'), Overlay = Frame.querySelector('canvas.depth-overlay');
    const Picture = (Element, Width, Height) => {
      const Rect = Element.getBoundingClientRect(), Scale = Math.min(Rect.width / Width, Rect.height / Height);
      return [Rect.x + (Rect.width - Width * Scale) / 2, Rect.y + (Rect.height - Height * Scale) / 2, Width * Scale, Height * Scale];
    };
    const Box = Element => { const Rect = Element.getBoundingClientRect(); return [Rect.x, Rect.y, Rect.width, Rect.height]; };
    return { Frame: Box(Frame), Video: Box(Video), VideoPicture: Picture(Video, Video.videoWidth, Video.videoHeight), OverlayPicture: Picture(Overlay, Overlay.width, Overlay.height) };
  });
  Assert.ok(Layers.Frame[2] / Layers.Frame[3] > 16 / 9 + 0.1, `frame ${Layers.Frame} is wider than 16:9`);
  Layers.Frame.forEach((Value, Index) => Assert.ok(Math.abs(Value - Layers.Video[Index]) < 1, JSON.stringify(Layers)));
  Layers.VideoPicture.forEach((Value, Index) => Assert.ok(Math.abs(Value - Layers.OverlayPicture[Index]) < 1, JSON.stringify(Layers)));
  DepthAt = 'ok'; await Page.mouse.move(5, 540); await Page.waitForTimeout(150);
  await Page.mouse.move(Layers.Frame[0] + Layers.Frame[2] / 2, Layers.Frame[1] + Layers.Frame[3] / 2);
  await Page.waitForFunction(() => document.querySelector('.camera-window .depth-legend')?.textContent === 'Depth 48.2 cm · X 3.0, Y 30.0, Z 8.0 cm');
  const [WideU, WideV] = Lookups.at(-1);
  Assert.ok(Math.abs(WideU - 640) < 2 && Math.abs(WideV - 360) < 2, `wide window looked up ${WideU}, ${WideV}`);
  await Page.mouse.move(5, 540);
  await Drag(Wider.x + 7, Wider.y + 7, -40, -60);  // inside the 16px rounded corner
  const Corner = await Window.boundingBox();
  Assert.ok(Math.abs(Corner.x - (Wider.x - 40)) < 2 && Math.abs(Corner.y - (Wider.y - 60)) < 2, JSON.stringify({ Wider, Corner }));
  Assert.ok(Math.abs(Corner.x + Corner.width - (Wider.x + Wider.width)) < 2 && Math.abs(Corner.y + Corner.height - (Wider.y + Wider.height)) < 2);
  const Saved = await Page.evaluate(() => JSON.parse(localStorage.getItem('SO101.CameraWindow.v1')));
  Assert.ok(Math.abs(Saved.Width - Corner.width) < 2 && Math.abs(Saved.Height - Corner.height) < 2, 'new size persists');
  Assert.equal(await Page.getByRole('button', { name: 'Resize camera window', exact: true }).count(), 1);

  // The camera button (minimized tile) sits at the window's top-right corner, and opens the window from its own top-right corner.
  await Page.getByRole('button', { name: 'Minimize Live Feed', exact: true }).click();
  const Tile = await Window.boundingBox();
  Assert.deepEqual([Tile.x + Tile.width, Tile.y, Tile.width], [Corner.x + Corner.width, Corner.y, 48]);
  await Page.mouse.move(Tile.x + 24, Tile.y + 24); await Page.mouse.down();
  await Page.mouse.move(Tile.x - 176, Tile.y + 124, { steps: 8 }); await Page.mouse.up();  // move the button first
  const Moved = await Window.boundingBox();
  await Page.getByRole('button', { name: 'Restore Live Feed', exact: true }).click();
  const Opened = await Window.boundingBox();
  Assert.deepEqual([Opened.x + Opened.width, Opened.y, Opened.width, Opened.height], [Moved.x + Moved.width, Moved.y, Corner.width, Corner.height],
    JSON.stringify({ Moved, Opened }));

  Assert.deepEqual(Pins, [], 'hover never uses the frame-pinning /depthcam/point');
  Assert.deepEqual(Errors, []);
  console.log('PASS: hover depth readout (value, pixel, refresh rate, holes, leave, old bridge); edge and corner resize; overlay and hover stay on the picture in a window wider than 16:9; camera button opens from its top-right corner; no frame pins.');
} finally {
  Timers.forEach(clearInterval);
  await Browser.close(); await Server.Close();
}
