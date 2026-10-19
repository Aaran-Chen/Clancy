// Semi-automatic calibration end to end (App.tsx runSelfCalibration("semi"),
// components/SemiCalibrationStage.tsx, components/HoverLoupe.tsx) with the bridge,
// arm and camera faked; nothing reaches port 8000.
//  1. 2.25 device pixel ratio at the user's 1707x830 viewport, a mirrored ("Facing")
//     capture: the arm moves only after each decision; the magnifier's crosshair pixel
//     is the posted click (u, v, frame, archive, mirroring, size); a letterbox click posts
//     nothing; Skip, Retake (a new capture of the same pose, no move), Undo and re-click
//     of an earlier frame while the arm holds the next pose, Done early, the automatic fit
//     with a flagged pose saved and applied with its warnings, re-click it, Fit again, saved;
//     Reject calibration only on the user's confirmation; a fit that computes nothing keeps
//     the calibration in use; one Hold at the end.
//  2. Device pixel ratio 1 at 1920x1080, an unmirrored ("Behind") capture: the same
//     mapping check; Stop while the arm holds a pose; a bridge fault while it holds; a
//     servo reaching the heat limit; an old bridge without the click endpoints.
// Run: node TestSemiCalibrationUi.mjs (it serves this checkout on 5191-5199 itself: UiTestLib.mjs)
import { chromium as Chromium } from "playwright";
import { LaunchOptions, StartUiServer } from "./UiTestLib.mjs";
import { readFileSync as ReadFile, mkdirSync as MakeDirectory } from "node:fs";
import { execFileSync as Execute } from "node:child_process";
import { fileURLToPath as FilePath } from "node:url";
import Path from "node:path";
import Assert from "node:assert/strict";

const Root = Path.dirname(FilePath(import.meta.url)), Backend = Path.resolve(Root, "../back end");
const Python = ReadFile(Path.join(Root, "../.backend-python.txt"), "utf8").replace(/^\uFEFF/, "").trim();
const Snapshot = JSON.parse(Execute(Python, ["-c", "import json, bridge_server; print(json.dumps(bridge_server.BridgeController().state_snapshot()))"],
  { cwd: Backend, encoding: "utf8" }));
const Server = await StartUiServer();
const Url = Server.Url;
const Artifacts = Path.join(Backend, "TestArtifacts");
MakeDirectory(Artifacts, { recursive: true });
const Pause = Milliseconds => new Promise(Resolve => setTimeout(Resolve, Milliseconds));
let CurrentPage = null;
const Browser = await Chromium.launch(LaunchOptions({ args: ["--use-fake-device-for-media-stream", "--use-fake-ui-for-media-stream"] }));

async function Session({ Dpr, Viewport, View, PoseCount }) {
  const State = JSON.parse(JSON.stringify(Snapshot));
  Object.assign(State, { arm_connected: true, torque: true, port: "FAKE", motion_fault: null, control_age_ms: 0 });
  for (const Servo of Object.values(State.servos)) Object.assign(Servo, { feedback_ok: true, moving: false, target_raw: Servo.present_raw });
  State.protection.telemetry = { shoulder_lift: { temperature_c: 38, at_s: 1 } };
  const Context = await Browser.newContext({ viewport: Viewport, deviceScaleFactor: Dpr, permissions: ["camera"] });
  await Context.addInitScript(([View]) => {
    localStorage.setItem("SO101.ManualDisconnect", "false");
    const Key = "SO101.ViewSettings.v1";
    const Saved = JSON.parse(localStorage.getItem(Key) ?? "{}");
    localStorage.setItem(Key, JSON.stringify({ ...Saved, WebcamView: View, DepthCameraView: View }));
  }, [View]);
  const Page = await Context.newPage();
  CurrentPage = Page;
  const Log = { Errors: [], Commands: [], Observes: [], Clicks: [], Undos: 0, Skips: [], Solves: [], Finishes: [], Prepares: 0, Rejects: [] };
  const Bridge = { ClicksSupported: true, SolveReplies: [], RejectReplies: [] };
  const Timers = [];
  let Archive = 0;
  Page.on("pageerror", Error => Log.Errors.push(Error.message));
  await Page.route("**/bridge-launch/**", Route => Route.fulfill({ json: { ok: true } }));
  await Page.route("**/ollama/**", Route => Route.fulfill({ json: { models: [] } }));
  await Page.route("**/depth-server/**", Route => Route.fulfill({ json: {} }));
  await Page.route("**/session-log/**", Route => Route.fulfill({ json: { ok: true } }));
  await Page.route("**:8000/**", async Route => {
    const Address = new URL(Route.request().url()), Payload = Route.request().postDataJSON();
    const Path_ = Address.pathname;
    if (Path_ === "/selfcal/clicks") {
      return Bridge.ClicksSupported ? Route.fulfill({ json: { ok: true, version: 1, clicks: [], skipped: [] } })
        : Route.fulfill({ status: 404, json: { detail: "Not Found" } });
    }
    if (Path_ === "/selfcal/prepare") {
      Log.Prepares += 1;
      const Ready = { ...State, motion_fault: null, motion_epoch: State.motion_epoch + 1 };
      Timers.push(setTimeout(() => Object.assign(State, { motion_fault: null, motion_epoch: Ready.motion_epoch }), 300));
      return Route.fulfill({ json: { ok: true, state: Ready, archive_version: 2 } });
    }
    if (Path_ === "/selfcal/reset") return Route.fulfill({ json: { ok: true, archive_version: 2, run_id: "fake" } });
    if (Path_ === "/selfcal/check") return Route.fulfill({ json: { ok: true, detected: false, reason: "no tape selected", frames: 1 } });
    if (Path_ === "/selfcal/poses") {
      return Route.fulfill({ json: { ok: true, poses: Array.from({ length: PoseCount }, (_, Index) =>
        ({ x: -10 + Index, y: 28, z: 10, wrist_pitch: -12, plan_id: `plan-${Index}` })) } });
    }
    if (Path_ === "/motion") {
      Log.Commands.push(Payload);
      if (Payload.type === "stop_motion") await Pause(200);
      return Route.fulfill({ json: { ok: true, state: State } });
    }
    if (Path_ === "/selfcal/observe") {
      Assert.equal(Payload.images.length, 3);
      Assert.equal(Payload.capture_valid, true);
      Assert.ok(Payload.requested_pose.plan_id);
      Archive += 1;
      const CaptureArchive = `C:\\fake\\runs\\fake\\observations\\${Archive}_pose${Payload.pose_index}.json`;
      Log.Observes.push({ PoseIndex: Payload.pose_index, Archive: CaptureArchive, Images: Payload.images, Mirrored: Payload.mirrored,
        Metadata: Payload.frame_metadata });
      return Route.fulfill({ json: { ok: true, detected: false, reason: "not used", capture_archive: CaptureArchive } });
    }
    if (Path_ === "/selfcal/click") {
      Log.Clicks.push(Payload);
      return Route.fulfill({ json: { ok: true, pose_index: Payload.pose_index, replaced: false, native_px: [0, 0], depth_cm: 44.7, target: "tape_centre" } });
    }
    if (Path_ === "/selfcal/click/undo") {
      Log.Undos += 1;
      const Last = Log.Clicks.filter(Click => !Click.Undone).at(-1);
      if (Last) Last.Undone = true;
      return Route.fulfill({ json: { ok: true, pose_index: Last ? Last.pose_index : null, removed: Boolean(Last) } });
    }
    if (Path_ === "/selfcal/click/skip") { Log.Skips.push(Payload); return Route.fulfill({ json: { ok: true, pose_index: Payload.pose_index } }); }
    if (Path_ === "/selfcal/solve") {
      Log.Solves.push(Payload);
      return Route.fulfill({ json: Bridge.SolveReplies.shift() ?? { ok: false, error: "no reply scripted" } });
    }
    if (Path_ === "/selfcal/finish") { Log.Finishes.push(Payload); return Route.fulfill({ json: { ok: true } }); }
    if (Path_ === "/selfcal/reject") {
      Log.Rejects.push(Payload);
      return Route.fulfill({ json: Bridge.RejectReplies.shift() ?? { ok: false, error: "no reply scripted" } });
    }
    if (Path_ === "/depthcam/status") return Route.fulfill({ json: { ok: true, available: false } });
    if (Path_ === "/ports") return Route.fulfill({ json: ["FAKE"] });
    return Route.fulfill({ json: { ok: true, calibrated: false, points: [], point_count: 0, min_points: 4, recommended_points: 6 } });
  });
  await Page.routeWebSocket("**/ws", Socket => {
    Socket.send(JSON.stringify(State));
    const Timer = setInterval(() => Socket.send(JSON.stringify(State)), 100); Timers.push(Timer);
    Socket.onMessage(Message => Log.Commands.push(JSON.parse(String(Message))));
    Socket.onClose(() => clearInterval(Timer));
  });
  await Page.goto(Url, { waitUntil: "domcontentloaded" });
  await Page.getByRole("button", { name: "Minimize Live Feed", exact: true }).click();
  const Close = async () => { Timers.forEach(Timer => { clearTimeout(Timer); clearInterval(Timer); }); await Context.close(); };
  return { Page, State, Log, Bridge, Close };
}

const Moves = Log => Log.Commands.filter(Command => Command.type === "send_ik");
const Holds = Log => Log.Commands.filter(Command => Command.type === "stop_motion");
const FrameSelector = "img.semical-image";
async function WaitFor(Condition, What, Limit = 20000) {
  const Started = Date.now();
  while (!(await Condition())) {
    if (Date.now() - Started > Limit) throw new Error(`Timed out waiting for ${What}`);
    await Pause(50);
  }
}

// The frame, the magnifier at the pointer, and the pixel under the pointer by an independent
// calculation (object-fit contain letterbox, display mirroring).
async function Inspect(Page, X, Y) {
  return Page.evaluate(([Selector, X, Y]) => {
    const Image = document.querySelector(Selector), Box = document.querySelector(".hover-loupe[data-visible='true']");
    const Rect = Image.getBoundingClientRect(), Mirrored = getComputedStyle(Image).transform !== "none";
    const Scale = Math.min(Rect.width / Image.naturalWidth, Rect.height / Image.naturalHeight);
    const Left = Rect.left + (Rect.width - Image.naturalWidth * Scale) / 2, Top = Rect.top + (Rect.height - Image.naturalHeight * Scale) / 2;
    const Shown = (X - Left) / Scale, Down = (Y - Top) / Scale;
    const Inside = Shown >= 0 && Shown < Image.naturalWidth && Down >= 0 && Down < Image.naturalHeight;
    const Expected = Inside ? { Column: Math.floor(Mirrored ? Image.naturalWidth - Shown : Shown), Row: Math.floor(Down) } : null;
    const Result = { Expected, Mirrored, Src: Image.src, Natural: [Image.naturalWidth, Image.naturalHeight],
      Picture: { Left, Top, Width: Image.naturalWidth * Scale, Height: Image.naturalHeight * Scale }, Visible: Boolean(Box) };
    if (!Box) return Result;
    const Canvas = Box.querySelector("canvas"), Context = Canvas.getContext("2d");
    const Centre = Array.from(Context.getImageData(Math.floor(Canvas.width / 2), Math.floor(Canvas.height / 2), 1, 1).data);
    const Probe = document.createElement("canvas"); Probe.width = Image.naturalWidth; Probe.height = Image.naturalHeight;
    const ProbeContext = Probe.getContext("2d"); ProbeContext.drawImage(Image, 0, 0);
    const Column = Number(Box.dataset.column), Row = Number(Box.dataset.row);
    const Source = Array.from(ProbeContext.getImageData(Column, Row, 1, 1).data);
    const LoupeRect = Box.getBoundingClientRect();
    return { ...Result, Column, Row, Centre, Source, CanvasWidth: Canvas.width,
      Box: { Left: LoupeRect.left, Top: LoupeRect.top, Right: LoupeRect.right, Bottom: LoupeRect.bottom },
      Viewport: { Width: innerWidth, Height: innerHeight }, Pointer: getComputedStyle(Box).pointerEvents, Rendering: getComputedStyle(Canvas).imageRendering };
  }, [FrameSelector, X, Y]);
}

// Hover at a fractional point inside the frozen frame, check the magnifier, click, and check the
// posted click names the crosshair pixel of the frame the bridge archived for that pose.
// Still: the pointer has not moved since the previous click, and the new frame arrived under it.
async function ClickAndCheck(Page, Log, Fraction, Dpr, PoseIndex, Still = false) {
  await Pause(400);  // a person looks at a new frame before clicking; a click within 350 ms is refused
  const Picture = (await Inspect(Page, 0, 0)).Picture;
  const X = Picture.Left + Picture.Width * Fraction[0] + 0.37, Y = Picture.Top + Picture.Height * Fraction[1] + 0.61;
  if (!Still) { await Page.mouse.move(X - 6, Y - 3); await Page.mouse.move(X, Y); }
  await Page.locator(".hover-loupe[data-visible='true']").waitFor({ timeout: 5000 });
  // A frame just switched in may still be decoding: the magnifier redraws when it has loaded.
  await Page.waitForFunction(Selector => document.querySelector(Selector).complete, FrameSelector);
  await Page.evaluate(() => new Promise(Resolve => requestAnimationFrame(() => requestAnimationFrame(Resolve))));
  const Seen = await Inspect(Page, X, Y);
  Assert.ok(Seen.Box.Left >= 0 && Seen.Box.Top >= 0 && Seen.Box.Right <= Seen.Viewport.Width && Seen.Box.Bottom <= Seen.Viewport.Height,
    `the magnifier stays inside the viewport: ${JSON.stringify(Seen.Box)}`);
  Assert.ok(!(X >= Seen.Box.Left && X <= Seen.Box.Right && Y >= Seen.Box.Top && Y <= Seen.Box.Bottom), "the magnifier never covers the aimed point");
  Assert.equal(Seen.Pointer, "none");
  Assert.equal(Seen.Rendering, "pixelated");
  Assert.equal(Seen.CanvasWidth, Math.round(184 * Dpr), "the magnifier draws at device resolution");
  Assert.deepEqual({ Column: Seen.Column, Row: Seen.Row }, Seen.Expected, "the crosshair names the pixel under the pointer");
  // The frozen frames are JPEG: the magnifier's (GPU) draw and this probe's can decode a pixel a
  // level or two apart. TestHoverLoupeUi checks the very pixel exactly on a lossless frame; here
  // the strict checks are the geometry above and the posted pixel below.
  Assert.ok(Seen.Centre.every((Value, Channel) => Math.abs(Value - Seen.Source[Channel]) <= 3),
    `the crosshair cell shows that full-resolution source pixel: ${Seen.Centre} vs ${Seen.Source}`);
  const Before = Log.Clicks.length;
  if (Still) { await Page.mouse.down(); await Page.mouse.up(); } else await Page.mouse.click(X, Y);
  await WaitFor(async () => Log.Clicks.length > Before, "the click to reach the bridge");
  const Click = Log.Clicks.at(-1);
  const Observed = Log.Observes.filter(Item => Item.PoseIndex === PoseIndex).at(-1);
  Assert.equal(Click.pose_index, PoseIndex);
  Assert.equal(Click.u, Seen.Column + 0.5, "u is the crosshair pixel's centre (capture pixels)");
  Assert.equal(Click.v, Seen.Row + 0.5, "v is the crosshair pixel's centre");
  Assert.equal(Click.capture_archive, Observed.Archive, "the click names the pose's latest archived capture");
  Assert.equal(Click.frame_index, 1);
  Assert.equal(Seen.Src, Observed.Images[1], "the frozen frame is the archived frame itself");
  Assert.equal(Click.mirrored, Observed.Mirrored);
  Assert.equal(Click.mirrored, Seen.Mirrored, "a mirrored capture is shown flipped back");
  Assert.deepEqual(Click.image_size, Seen.Natural);
  return { Seen, Click };
}

async function OpenCalibration(Page) {
  // The fake webcam must be streaming before a run can capture.
  await Page.waitForFunction(() => [...document.querySelectorAll("video")].some(Video => Video.dataset.captureReady === "true"), null, { timeout: 30000 });
  await Page.getByRole("button", { name: "Settings", exact: true }).click();
  await Page.getByRole("button", { name: "Calibrate", exact: true }).click();
}
const Stage = Page => Page.getByRole("dialog", { name: "Semi-automatic calibration" });
const WaitingAt = async (Page, Ordinal) => Page.locator(".semical-phase.phase-waiting", { hasText: `Pose ${Ordinal} of` }).waitFor({ timeout: 30000 });

try {
  // ---------- 1. DPR 2.25, 1707x830, mirrored capture: the whole flow ----------
  {
    const { Page, Log, Bridge, Close } = await Session({ Dpr: 2.25, Viewport: { width: 1707, height: 830 }, View: "Facing", PoseCount: 14 });
    await OpenCalibration(Page);
    await Page.getByRole("button", { name: "Start semi-automatic calibration", exact: true }).click();
    await Stage(Page).waitFor();
    await WaitingAt(Page, 1);
    // The arm holds pose 1 until a decision: no second move however long the user looks.
    Assert.equal(Moves(Log).length, 1);
    Assert.equal(Moves(Log)[0].plan_id, "plan-0");
    await Pause(1500);
    Assert.equal(Moves(Log).length, 1, "no move while the user is clicking");
    // A layout check at the user's screen: the frame fills most of the stage.
    const Layout = await Inspect(Page, 0, 0);
    Assert.ok(Layout.Picture.Height >= 600, `the frozen frame is large: ${JSON.stringify(Layout.Picture)}`);
    // The whole stage fits the viewport: the pose strip and Emergency stop are visible, not cut off.
    for (const Locator of [Page.locator(".semical-strip"), Stage(Page).getByRole("button", { name: "Emergency stop" })]) {
      const Box = await Locator.boundingBox();
      Assert.ok(Box && Box.y >= 0 && Box.y + Box.height <= 830 && Box.x + Box.width <= 1707, `inside the viewport: ${JSON.stringify(Box)}`);
    }
    // The picture is all inside its (clipping) area: letterboxed, never cropped.
    const Area = await Page.locator(".semical-image-wrap").boundingBox();
    Assert.ok(Layout.Picture.Left >= Area.x - 0.5 && Layout.Picture.Top >= Area.y - 0.5 && Layout.Picture.Left + Layout.Picture.Width <= Area.x + Area.width + 0.5
      && Layout.Picture.Top + Layout.Picture.Height <= Area.y + Area.height + 0.5, `the whole frame is visible: ${JSON.stringify(Layout.Picture)} in ${JSON.stringify(Area)}`);
    // Nothing (the minimized camera window, a panel) lies over the frozen frame.
    const Covered = await Page.evaluate(Selector => {
      const Rect = document.querySelector(Selector).getBoundingClientRect(), Misses = [];
      for (let I = 1; I < 10; I += 1) for (let J = 1; J < 10; J += 1) {
        const X = Rect.left + Rect.width * I / 10, Y = Rect.top + Rect.height * J / 10;
        if (document.elementFromPoint(X, Y) !== document.querySelector(Selector)) Misses.push([Math.round(X), Math.round(Y), document.elementFromPoint(X, Y)?.className]);
      }
      return Misses;
    }, FrameSelector);
    Assert.deepEqual(Covered, [], "the frozen frame is clickable everywhere");
    await Page.screenshot({ path: Path.join(Artifacts, "SemiCalibrationUi.png") });

    // Pose 1: click (mirrored mapping at 2.25 DPR), then the arm moves on.
    await ClickAndCheck(Page, Log, [0.41, 0.37], 2.25, 0);
    await WaitingAt(Page, 2);
    Assert.equal(Moves(Log).length, 2);

    // Pose 2: a click on the letterbox bar records nothing; Skip moves on.
    const Bars = await Inspect(Page, 0, 0);
    const Frame = await Page.locator(FrameSelector).boundingBox();
    if (Bars.Picture.Left - Frame.x > 8) {
      const X = Frame.x + (Bars.Picture.Left - Frame.x) / 2, Y = Frame.y + Frame.height / 2;
      await Page.mouse.move(X, Y - 2); await Page.mouse.move(X, Y);
      Assert.equal((await Inspect(Page, X, Y)).Visible, false, "no magnifier over the letterbox bar");
      await Page.mouse.click(X, Y);
      await Pause(300);
      Assert.equal(Log.Clicks.length, 1, "a click on the bar posts nothing");
    } else Assert.fail(`expected letterbox bars beside a 4:3 frame: ${JSON.stringify(Bars.Picture)} in ${JSON.stringify(Frame)}`);
    await Stage(Page).getByRole("button", { name: "Skip pose", exact: true }).click();
    await WaitingAt(Page, 3);
    Assert.deepEqual(Log.Skips.map(Skip => Skip.pose_index), [1]);
    Assert.equal(Moves(Log).length, 3);

    // Pose 3: Retake captures the same pose again (a new archive), without moving; then click it.
    const ObservesBefore = Log.Observes.length;
    await Stage(Page).getByRole("button", { name: "Retake", exact: true }).click();
    await WaitFor(async () => Log.Observes.length > ObservesBefore, "the retake's capture");
    await WaitingAt(Page, 3);
    Assert.equal(Log.Observes.at(-1).PoseIndex, 2);
    Assert.equal(Moves(Log).length, 3, "a retake does not move the arm");
    await WaitFor(async () => (await Page.locator(FrameSelector).getAttribute("src")) === Log.Observes.at(-1).Images[1], "the retaken frame");
    await ClickAndCheck(Page, Log, [0.62, 0.55], 2.25, 2);
    await WaitingAt(Page, 4);

    // Pose 4: click, then Undo while the arm holds pose 5: pose 4's frame returns for a re-click.
    await ClickAndCheck(Page, Log, [0.3, 0.7], 2.25, 3);
    await WaitingAt(Page, 5);
    await Stage(Page).getByRole("button", { name: "Undo", exact: true }).click();
    await WaitFor(async () => Log.Undos === 1, "the undo");
    await Page.locator(".semical-chip.active", { hasText: /^4$/ }).waitFor();
    Assert.equal(Moves(Log).length, 5, "undo does not move the arm");
    await ClickAndCheck(Page, Log, [0.33, 0.66], 2.25, 3);
    // Back to the frame the arm holds.
    await Page.locator(".semical-chip.active.waiting", { hasText: /^5$/ }).waitFor();
    Assert.equal(Moves(Log).length, 5);

    // Poses 5-11: click each (7 more: 10 clicks in all with poses 1, 3, 4).
    for (let Ordinal = 5; Ordinal <= 11; Ordinal += 1) {
      await WaitingAt(Page, Ordinal);
      if (Ordinal === 8) {
        // A click that lands just after the frame on screen changed was aimed at the previous
        // frame: it is refused, so it can neither record the wrong pose nor move the arm.
        const Posted = Log.Clicks.length;
        await Page.evaluate(Selector => new Promise(Resolve => {
          [...document.querySelectorAll(".semical-chip")].find(Chip => Chip.textContent === "7").click();
          requestAnimationFrame(() => requestAnimationFrame(() => {
            const Rect = document.querySelector(Selector).getBoundingClientRect();
            document.querySelector(Selector).dispatchEvent(new MouseEvent("click", { bubbles: true, clientX: Rect.left + Rect.width / 2,
              clientY: Rect.top + Rect.height / 2 }));
            Resolve();
          }));
        }), FrameSelector);
        await Stage(Page).getByText(/The frame just changed under the pointer/).waitFor();
        await Pause(300);
        Assert.equal(Log.Clicks.length, Posted, "the hasty click posts nothing");
        Assert.equal(Moves(Log).length, 8, "and does not move the arm");
        await Stage(Page).getByRole("button", { name: /The arm holds pose 8: show its frame/ }).click();
        await Page.locator(".semical-chip.active.waiting", { hasText: /^8$/ }).waitFor();
      }
      // Pose 10 arrives under the pointer left where pose 9 was clicked: the magnifier shows the new
      // frame there without a pointer move, and a click without a move records its crosshair pixel.
      const Fraction = Ordinal === 10 ? [0.2 + 0.05 * 2, 0.25 + 0.06 * 4] : [0.2 + 0.05 * (Ordinal % 7), 0.25 + 0.06 * (Ordinal % 5)];
      await ClickAndCheck(Page, Log, Fraction, 2.25, Ordinal - 1, Ordinal === 10);
      if (Ordinal === 6) {
        // Looking back at an earlier frame while the arm moves on: the next pose's frame does not
        // replace it under the pointer; a link leads to it.
        await Page.locator(".semical-chip", { hasText: /^5$/ }).click();
        await WaitingAt(Page, 7);
        await Page.locator(".semical-chip.active", { hasText: /^5$/ }).waitFor();
        await Page.locator(".semical-chip.waiting", { hasText: /^7$/ }).waitFor();
        Assert.equal(await Page.locator(".semical-chip.active").count(), 1);
        await Stage(Page).getByRole("button", { name: /The arm holds pose 7: show its frame/ }).click();
        await Page.locator(".semical-chip.active.waiting", { hasText: /^7$/ }).waitFor();
      }
    }
    await WaitingAt(Page, 12);
    Assert.equal(Moves(Log).length, 12);

    // Done early at pose 12 of 14: no more moves; the fit runs on its own and, with pose 4 flagged and its error over
    // the limit, is saved and applied with those problems as warnings (nothing rejects a calibration but the user).
    const Rows = Log.Clicks.filter(Click => !Click.Undone).map(Click => Click.pose_index);
    const PerPose = Flagged => Rows.map(PoseIndex => ({ pose_index: PoseIndex, observed_px: [600, 300], predicted_px: PoseIndex === Flagged ? [640, 330] : [600.4, 300.3],
      residual_px: PoseIndex === Flagged ? 50 : 0.5, residual_cm: PoseIndex === Flagged ? 3.7 : 0.04, depth_error_cm: 0.2, dropped: PoseIndex === Flagged,
      flagged: PoseIndex === Flagged, reason: PoseIndex === Flagged ? "dropped by the fit; 50.0 px off" : null }));
    const Problems = ["fit error 0.74 cm at the marker (11.0 px) exceeds 0.6 cm", "only 9 clicked poses were fitted (10 or more recommended)"];
    Bridge.SolveReplies.push({ ok: true, computed: true, saved: true, quality_ok: false, problems: Problems,
      quality_warning: "9 clicked poses used: re-click or skip the flagged poses and fit again, or press Reject calibration to restore the previous calibration.",
      fit_rms_cm: 0.74, fit_rms_px: 11.0, holdout_rms_cm: 0.81, plane_rms_cm: 0.52, fitted_at: "2026-10-10T14:02:33",
      per_pose: PerPose(3), observations_used: 9, observation_source: "manual" });
    await Stage(Page).getByRole("button", { name: "Done early", exact: true }).click();
    await Stage(Page).getByText("Saved and applied, with warnings.", { exact: true }).waitFor({ timeout: 30000 });
    for (const Problem of Problems) await Stage(Page).locator('ul[aria-label="Calibration warnings"] li', { hasText: Problem }).waitFor();
    await Stage(Page).getByText(/re-click or skip the flagged poses and fit again, or press Reject calibration/).waitFor();
    Assert.equal(Moves(Log).length, 12, "Done early ends the run without another move");
    Assert.deepEqual(Log.Solves.map(Solve => Solve.source), ["manual"]);
    await WaitFor(async () => Holds(Log).length === 1 && Log.Finishes.length === 1, "the end-of-run Hold and archive");
    Assert.equal(Log.Finishes[0].outcome, "calibrated with warnings");
    Assert.equal(Log.Rejects.length, 0, "no calibration is rejected without the user");
    // The flagged pose is shown for a re-click.
    await Page.locator(".semical-chip.flagged.active", { hasText: /^4$/ }).waitFor();
    await ClickAndCheck(Page, Log, [0.36, 0.64], 2.25, 3);
    Bridge.SolveReplies.push({ ok: true, computed: true, saved: true, quality_ok: true, problems: [], fit_rms_cm: 0.21, fit_rms_px: 2.4,
      holdout_rms_cm: 0.27, plane_rms_cm: 0.31, fitted_at: "2026-10-10T14:05:10", observations_used: 10, per_pose: PerPose(-1), observation_source: "manual" });
    await Stage(Page).getByRole("button", { name: "Fit again", exact: true }).click();
    await Stage(Page).getByText("Calibrated and saved.", { exact: true }).waitFor();
    Assert.equal(Log.Solves.length, 2);
    Assert.equal(Moves(Log).length, 12, "fitting never moves the arm");
    Assert.equal(Holds(Log).length, 1, "exactly one Hold at the end of the run");
    await Page.screenshot({ path: Path.join(Artifacts, "SemiCalibrationUiSaved.png") });
    // Reject calibration in the stage: only after the user confirms, for the calibration shown, and nothing moves.
    Bridge.RejectReplies.push({ ok: true, rejected_fitted_at: "2026-10-10T14:05:10", restored_fitted_at: "2026-10-10T14:02:33",
      restored_mode: "auto", calibrated: true, drift_baseline: true });
    let Question = null;
    Page.once("dialog", Dialog => { Question = Dialog.message(); void Dialog.dismiss(); });
    await Stage(Page).getByRole("button", { name: "Reject calibration", exact: true }).click();
    await WaitFor(async () => Question !== null, "the confirmation");
    Assert.equal(Log.Rejects.length, 0, "a dismissed confirmation rejects nothing");
    Page.once("dialog", Dialog => { void Dialog.accept(); });
    await Stage(Page).getByRole("button", { name: "Reject calibration", exact: true }).click();
    await Stage(Page).getByText("Rejected by you: the calibration it replaced was restored.", { exact: true }).waitFor();
    await Stage(Page).getByText("Calibration fitted at 14:05 rejected: the calibration fitted at 14:02 is in use again.", { exact: true }).waitFor();
    Assert.deepEqual(Log.Rejects.map(Payload => Payload.fitted_at), ["2026-10-10T14:05:10"]);
    Assert.equal(await Stage(Page).getByRole("button", { name: "Reject calibration", exact: true }).count(), 0, "a rejected fit offers no second reject");
    Assert.equal(Moves(Log).length, 12, "rejecting never moves the arm");
    // A fit that computes no camera keeps the calibration in use, and says so.
    Bridge.SolveReplies.push({ ok: false, computed: false, saved: false, observation_source: "manual",
      error: "Could not compute a calibration: the tip poses do not span three dimensions. The current calibration is still in use." });
    await Stage(Page).getByRole("button", { name: "Fit again", exact: true }).click();
    await Stage(Page).getByText("Could not compute a calibration: the current calibration is still in use.", { exact: true }).waitFor();
    await Stage(Page).getByText(/^Not computed: Could not compute a calibration: the tip poses do not span/).waitFor();
    Assert.equal(Log.Solves.length, 3);
    // Closed, the frames can be shown again from the calibration panel.
    await Stage(Page).getByRole("button", { name: "Close semi-automatic calibration", exact: true }).click();
    await Page.getByRole("button", { name: /Show the frozen frames \(10 clicked\)/ }).click();
    await Stage(Page).waitFor();
    Assert.deepEqual(Log.Errors, []);
    await Close();
  }

  // ---------- 2. DPR 1, 1920x1080, unmirrored capture: stop, fault, heat, old bridge ----------
  {
    const { Page, State, Log, Bridge, Close } = await Session({ Dpr: 1, Viewport: { width: 1920, height: 1080 }, View: "Behind", PoseCount: 6 });
    await OpenCalibration(Page);
    const Start = Page.getByRole("button", { name: "Start semi-automatic calibration", exact: true });
    await Start.click();
    await WaitingAt(Page, 1);
    const { Seen } = await ClickAndCheck(Page, Log, [0.58, 0.31], 1, 0);
    Assert.equal(Seen.Mirrored, false);
    // Stop while the arm holds pose 2: one Hold, no further move, the run ends cancelled.
    await WaitingAt(Page, 2);
    await Stage(Page).getByRole("button", { name: "Stop", exact: true }).click();
    await Stage(Page).getByText(/Stopped: Calibration cancelled by user/).waitFor();
    await WaitFor(async () => Log.Finishes.length === 1, "the cancelled run's archive");
    Assert.equal(Moves(Log).length, 2);
    Assert.equal(Holds(Log).length, 1);
    Assert.equal(Log.Finishes[0].cancelled, true);
    Assert.ok(await Stage(Page).getByRole("button", { name: "Fit now", exact: true }).isDisabled(), "1 click cannot be fitted");
    await Stage(Page).getByRole("button", { name: "Close semi-automatic calibration", exact: true }).click();

    // A bridge fault while the arm holds a pose ends the run with its reason, without another move.
    await Start.click();
    await WaitingAt(Page, 1);
    const MovesBefore = Moves(Log).length;
    State.motion_fault = "Servo protection/torque fault (shoulder lift overload)";
    State.motion_epoch += 1;
    await Stage(Page).getByText(/Stopped: Servo protection\/torque fault/).waitFor();
    await WaitFor(async () => Log.Finishes.length === 2, "the faulted run's archive");
    Assert.equal(Moves(Log).length, MovesBefore);
    Assert.equal(Holds(Log).length, 2);
    State.motion_fault = null;
    State.motion_epoch += 1;
    await Stage(Page).getByRole("button", { name: "Close semi-automatic calibration", exact: true }).click();

    // A servo at the heat limit while the arm holds a pose ends the run there (no further move).
    await Start.click();
    await WaitingAt(Page, 1);
    const MovesHot = Moves(Log).length;
    State.protection.telemetry = { shoulder_lift: { temperature_c: 56, at_s: 2 } };
    await Stage(Page).getByText(/A servo reached 56 °C/).waitFor({ timeout: 10000 });
    await WaitFor(async () => Log.Finishes.length === 3, "the hot run's archive");
    Assert.equal(Moves(Log).length, MovesHot);
    Assert.equal(Holds(Log).length, 3);
    State.protection.telemetry = { shoulder_lift: { temperature_c: 38, at_s: 3 } };
    await Stage(Page).getByRole("button", { name: "Close semi-automatic calibration", exact: true }).click();

    // The serial link drops while the arm holds a pose: the run ends there, no further move.
    await Start.click();
    await WaitingAt(Page, 1);
    const MovesDrop = Moves(Log).length;
    State.arm_connected = false;
    await Stage(Page).getByText(/Stopped: The arm disconnected/).waitFor({ timeout: 10000 });
    await WaitFor(async () => Log.Finishes.length === 4, "the dropped run's archive");
    Assert.equal(Moves(Log).length, MovesDrop);
    State.arm_connected = true;
    await Stage(Page).getByRole("button", { name: "Close semi-automatic calibration", exact: true }).click();

    // Emergency stop in the stage while the arm holds a pose: cancelled, no further move.
    await Page.waitForFunction(() => !document.querySelector(".semical-stage"));
    await Start.click();
    await WaitingAt(Page, 1);
    const MovesEmergency = Moves(Log).length;
    await Stage(Page).getByRole("button", { name: "Emergency stop" }).click();
    await Stage(Page).getByText(/Stopped: Calibration cancelled by user/).waitFor({ timeout: 10000 });
    Assert.ok(Log.Commands.some(Command => Command.type === "emergency_stop"), "the emergency stop reached the bridge");
    Assert.equal(Moves(Log).length, MovesEmergency);
    await WaitFor(async () => Log.Finishes.length === 5, "the emergency run's archive");
    await Stage(Page).getByRole("button", { name: "Close semi-automatic calibration", exact: true }).click();

    // A bridge without the click endpoints: refused before anything is prepared or moved.
    Bridge.ClicksSupported = false;
    const Prepared = Log.Prepares, Moved = Moves(Log).length;
    await Start.click();
    await Page.getByText("Restart the bridge to load semi-automatic calibration.", { exact: true }).waitFor();
    Assert.equal(Log.Prepares, Prepared);
    Assert.equal(Moves(Log).length, Moved);
    Assert.deepEqual(Log.Errors, []);
    await Close();
  }
  console.log("PASS: semi-automatic calibration at 2.25 DPR (1707x830, mirrored) and 1 DPR (1920x1080, unmirrored): one move per decision, "
    + "magnifier crosshair = posted click pixel of the archived frame, letterbox ignored, skip, retake without moving, undo and re-click, "
    + "a new frame never replaces an earlier one being looked at, a click right after a frame change is refused, "
    + "done early, automatic fit with a flagged pose saved with its warnings, re-click and fit again saved, Reject calibration on confirmation, "
    + "a fit that computes nothing keeps the calibration, one Hold per run; stop, fault, heat, serial drop and "
    + "emergency stop end a held pose without moving; an old bridge is refused before any motion; no browser errors.");
} catch (Failure) {
  // Where the app stood when a check failed.
  if (CurrentPage && !CurrentPage.isClosed()) {
    const Steps = await CurrentPage.locator(".selfcal-progress, .semical-phase, .semical-note").allInnerTexts().catch(() => []);
    console.error("App status:", JSON.stringify(Steps));
    await CurrentPage.screenshot({ path: Path.join(Artifacts, "SemiCalibrationUiFailure.png") }).catch(() => {});
  }
  throw Failure;
} finally {
  await Browser.close(); await Server.Close();
}
