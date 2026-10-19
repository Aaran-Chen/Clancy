import { chromium as Chromium } from "playwright";
import { Fixture, LaunchOptions, StartUiServer } from "./UiTestLib.mjs";
import { readFileSync as ReadFile, mkdirSync as MakeDirectory } from "node:fs";
import { fileURLToPath as FilePath } from "node:url";
import Path from "node:path";
import Assert from "node:assert/strict";

const Root = Path.dirname(FilePath(import.meta.url));
const Backend = Path.resolve(Root, "../back end");
const Home = JSON.parse(ReadFile(Path.join(Backend, "home_preset.json"), "utf8"));
const Limits = JSON.parse(ReadFile(Path.join(Backend, "gui_limits.json"), "utf8")).motor_raw_limits;
// A real calibration frame (2026-09-13 09:28), kept in git: the captures folder is not (and a worktree has none).
const Image = "data:image/jpeg;base64," + ReadFile(Fixture("Calibration-2026-09-13", "092856_964_pose0_miss.jpg")).toString("base64");
const State = { type: "state", arm_connected: true, torque: true, status: "Simulated arm", home_raw: Home,
  collision_version: JSON.parse(ReadFile(Path.join(Backend, "CollisionModel.json"), "utf8")).Version,
  speed_control_version: "throttle-v2", motion_speed_percent: 100,
  ik_solver_version: "all-poses-v2",
  cad_source_commit: JSON.parse(ReadFile(Path.join(Backend, "CadModel.json"), "utf8")).SourceCommit,
  cad_alignment: JSON.parse(ReadFile(Path.join(Backend, "CadAlignment.json"), "utf8")),
  configuration_key: "simulation", control_age_ms: 0, motion_fault: null,
  motion_epoch: 0, joint_positions: [{ x: 0, y: 0, z: 0 }, { x: 0, y: 0, z: 10 }, { x: 0, y: 10, z: 20 }, { x: 0, y: 20, z: 20 }],
  servos: Object.fromEntries(Object.entries(Home).map(([Name, Raw]) => [Name, {
    present_raw: Raw, target_raw: Raw, range_min: Limits[Name].min, range_max: Limits[Name].max,
    arc_inverted: Limits[Name].arc_inverted, feedback_ok: true, moving: false,
  }])) };
const Server = await StartUiServer();
const Browser = await Chromium.launch(LaunchOptions({ args: ["--use-fake-device-for-media-stream", "--use-fake-ui-for-media-stream"] }));
const Page = await Browser.newPage({ viewport: { width: 1440, height: 1000 }, permissions: ["camera"] });
const Errors = [], Commands = [];
let Scenario = "selection", Observations = 0, Solves = 0, StopsFinished = 0, SuccessMoves = 0;
// The calibration /calibration/status reports (none until a solve saves one), what Reject posts, and the fit's warnings.
let Saved = {};
const Rejects = [];
const Problems = ["fit error 0.74 cm at the marker (11.0 px) exceeds 0.6 cm",
  "the desk plane was found in depth at only 4 of 30 frames (need 6), so later captures could not be checked against this calibration"];
const Pause = Milliseconds => new Promise(Resolve => setTimeout(Resolve, Milliseconds));
const Timers = [];
Page.on("pageerror", Error => Errors.push(Error.message));
await Page.route("**/bridge-launch/**", Route => Route.fulfill({ json: { ok: true } }));
await Page.route("**/ollama/**", Route => Route.fulfill({ json: { models: [] } }));
await Page.route("**/depth-server/**", Route => Route.fulfill({ json: {} }));
await Page.route("**:8000/**", async Route => {
  const Url = new URL(Route.request().url());
  const Payload = Route.request().postDataJSON();
  if (Url.pathname === "/selfcal/prepare") {
    await Pause(250);
    if (Scenario === "hardware-fault") {
      await Route.fulfill({ json: { ok: false, error: State.motion_fault } });
      return;
    }
    const ReadyState = { ...State, motion_fault: null, motion_epoch: State.motion_epoch + 1 };
    // HTTP can finish before older WebSocket frames have drained. Keep sending
    // the old timeout for a while to reproduce the real cancellation race.
    Timers.push(setTimeout(() => Object.assign(State, ReadyState), 500));
    await Route.fulfill({ json: { ok: true, state: ReadyState, archive_version: Scenario === "old-recorder" ? 1 : 2 } });
  } else if (Url.pathname === "/selfcal/reset") {
    await Route.fulfill({ json: { ok: true, archive_version: Scenario === "old-recorder" ? 1 : 2 } });
  } else if (Url.pathname === "/selfcal/check") {
    if (Scenario === "new-fault") {
      State.motion_fault = "Motion settling timed out";
      State.motion_epoch += 1;
      await Pause(300);
    }
    const Detected = Boolean(Payload?.hint) && Scenario !== "missing-tip";
    await Route.fulfill({ json: { ok: true, detected: Detected, preview: Image, method: "red",
      frames: 3, frames_detected: Detected ? 3 : 0, area_px: 738, frame_size: [1280, 720],
      reason: Detected ? null : "Selected red tip is not visible" } });
  } else if (Url.pathname === "/motion") {
    Commands.push(Payload);
    if (Payload.type === "stop_motion") {
      await Pause(1000);
      StopsFinished += 1;
    }
    if (Payload.type === "send_ik" && Scenario === "success" && ++SuccessMoves === 2) {
      // A detour: the bridge starts at the first waypoint's targets and reaches
      // the pose's own targets only at its second stage. Settling must wait for it.
      const Final = Object.fromEntries(Object.entries(State.servos).map(([Name, Servo]) => [Name, Servo.target_raw]));
      State.motion_epoch += 1;
      State.motion_route = { stage: 1, stages: 2 };
      State.servos = Object.fromEntries(Object.entries(State.servos).map(([Name, Servo]) =>
        [Name, { ...Servo, target_raw: Name === "shoulder_pan" ? Servo.target_raw + 60 : Servo.target_raw }]));
      const Reply = JSON.parse(JSON.stringify(State));
      Timers.push(setTimeout(() => { State.motion_route = { stage: 2, stages: 2 }; }, 900));
      Timers.push(setTimeout(() => {
        State.servos = Object.fromEntries(Object.entries(State.servos).map(([Name, Servo]) => [Name, { ...Servo, target_raw: Final[Name] }]));
        State.motion_route = null;
      }, 1800));
      await Route.fulfill({ json: { ok: true, state: Reply } });
      return;
    }
    await Route.fulfill({ json: { ok: true, state: State } });
  } else if (Url.pathname === "/selfcal/poses") {
    await Route.fulfill({ json: { ok: true, sampling: { preferred_y_range_cm: [27, 33], preferred_poses: 0 },
      poses: Array.from({ length: 10 }, (_, Index) => ({ x: 0, y: 15, z: 20, plan_id: `simulated-${Index}` })) } });
  } else if (Url.pathname === "/selfcal/observe") {
    Assert.equal(Payload.images.length, 3);
    Assert.equal(Payload.frame_metadata.length, 3);
    Assert.equal(Payload.capture_valid, true);
    Assert.ok(Payload.requested_pose.plan_id);
    for (const Frame of Payload.frame_metadata) {
      Assert.ok(Number.isFinite(Frame.captured_at_epoch_ms));
      Assert.ok(Frame.joints.length >= 3);
      Assert.ok(Number.isFinite(Frame.servos.elbow_flex.presentRaw));
    }
    Observations += 1;
    await Route.fulfill({ json: { ok: true, detected: true } });
  } else if (Url.pathname === "/selfcal/solve") {
    Solves += 1;
    // What used to reject a fit is a warning on a saved calibration: the bridge saved and applied it.
    Saved = { mode: "auto", calibrated: true, fitted_at: "2026-10-10T14:02:33", quality_problems: Problems, can_reject: true,
      replaced_fitted_at: "2026-10-09T21:10:05", drift_baseline: false, observation_source: "auto" };
    await Route.fulfill({ json: { ok: true, computed: true, saved: true, quality_ok: false, problems: Problems,
      quality_warning: "Check this calibration before relying on it, or press Reject calibration to restore the previous one.",
      observations_used: Observations, plane_rms_cm: 0.1, fitted_at: "2026-10-10T14:02:33", drift_baseline: false } });
  } else if (Url.pathname === "/selfcal/reject") {
    Rejects.push(Payload);
    Saved = { mode: "auto", calibrated: true, fitted_at: "2026-10-09T21:10:05", quality_problems: [], can_reject: false, drift_baseline: true };
    await Route.fulfill({ json: { ok: true, rejected_fitted_at: "2026-10-10T14:02:33", restored_fitted_at: "2026-10-09T21:10:05",
      restored_mode: "auto", calibrated: true, drift_baseline: true } });
  } else if (Url.pathname === "/calibration/status") {
    await Route.fulfill({ json: { ok: true, calibrated: false, points: [], point_count: 0, min_points: 4, recommended_points: 6, ...Saved } });
  } else if (Url.pathname === "/ports") {
    await Route.fulfill({ json: ["SIMULATED"] });
  } else {
    await Route.fulfill({ json: { ok: true, calibrated: false, points: [], point_count: 0 } });
  }
});
let Timer;
await Page.routeWebSocket("**/ws", Socket => {
  Socket.send(JSON.stringify(State));
  Timer = setInterval(() => Socket.send(JSON.stringify(State)), 100);
  Socket.onMessage(Message => Commands.push(JSON.parse(String(Message))));
});
try {
  await Page.goto(Server.Url, { waitUntil: "domcontentloaded" });
  await Page.getByRole("button", { name: "Minimize Live Feed", exact: true }).click();
  await Page.getByRole("button", { name: "Settings", exact: true }).click();
  await Page.getByRole("button", { name: "Calibrate", exact: true }).click();
  await Page.getByRole("button", { name: "Check marker", exact: true }).click();
  const Preview = Page.getByAltText("Click the red tape on the gripper tip");
  await Preview.waitFor();
  await Preview.click({ position: { x: 300, y: 150 } });
  await Page.getByText(/Marker found in 3 of 3 frames/).waitFor();
  const Start = Page.getByRole("button", { name: "Start automatic calibration" });
  Scenario = "missing-tip";
  await Start.click();
  await Page.getByText(/Failed: Selected red tip is not visible/).waitFor();
  Assert.equal(Commands.filter(Command => ["send_ik", "move_servo", "toggle_torque"].includes(Command.type)).length, 0);
  Assert.equal(Commands.filter(Command => Command.type === "stop_motion").length, 0);

  Scenario = "old-recorder";
  await Start.click();
  await Page.getByText(/Failed: Restart the bridge to load complete calibration recording/).waitFor();
  Assert.equal(Commands.filter(Command => Command.type === "send_ik").length, 0);

  Scenario = "hardware-fault";
  State.motion_fault = "Servo protection/torque fault (shoulder lift overload)";
  State.motion_epoch += 1;
  await Start.click();
  await Page.getByText(/Failed: Servo protection\/torque fault/).waitFor();
  Assert.equal(Commands.filter(Command => Command.type === "send_ik").length, 0);

  Scenario = "new-fault";
  State.motion_fault = "Motion settling timed out";
  await Start.click();
  await Page.getByText(/Failed: Motion settling timed out/).waitFor();
  Assert.equal(Commands.filter(Command => Command.type === "send_ik").length, 0, "A new timeout must not be ignored");

  Scenario = "success";
  State.motion_fault = "Motion settling timed out";
  await Start.click();
  // The bridge sends no shoulder-load count any more (the limit was removed 2026-10-03): the warning names none.
  await Page.getByText("No safe calibration poses at Y = 27 to 33 cm: none there is reachable and clear. Continuing with the other spaced poses.").waitFor();
  await Page.getByText("Calibrated, with warnings", { exact: true }).waitFor({ timeout: 60000 });
  Assert.equal(Observations, 10);
  Assert.equal(Solves, 1);
  Assert.equal(Commands.filter(Command => Command.type === "send_ik").length, 10);
  await Start.waitFor({ state: "visible" });
  Assert.equal(StopsFinished, 1, "A retry must wait for stop completion");
  Assert.equal(Commands.filter(Command => Command.type === "stop_motion").length, 1, "Cleanup sends exactly one stop");

  // The fit with problems was saved and applied: its warnings are listed with their numbers, under the run's
  // report and under the calibration in use, and nothing rejects it until the user does.
  await Page.getByText("Saved and applied, with warnings.", { exact: true }).waitFor();
  for (const Problem of Problems) {
    await Page.locator('ul[aria-label="Calibration warnings"] li', { hasText: Problem }).waitFor();
    await Page.locator('ul[aria-label="Warnings of the calibration in use"] li', { hasText: Problem }).waitFor();
  }
  await Page.getByText("In use: the calibration fitted at 14:02, with 2 warnings").waitFor();
  await Page.getByText(/No camera-check baseline: tasks that act on an object stop/).waitFor();
  Assert.equal(Rejects.length, 0, "no calibration is rejected without the user");
  // Reject calibration: asks first, posts the calibration the user saw, and shows the one restored.
  const Reject = Page.getByRole("button", { name: "Reject calibration", exact: true });
  let Question = null;
  Page.once("dialog", Dialog => { Question = Dialog.message(); void Dialog.accept(); });
  await Reject.click();
  await Page.locator(".selfcal-progress").getByText("Calibration fitted at 14:02 rejected: the calibration fitted at 21:10 is in use again.").waitFor();
  Assert.equal(Question, "Reject the calibration fitted at 14:02? The calibration it replaced (fitted at 21:10) is restored and used again.");
  Assert.deepEqual(Rejects.map(Payload => Payload.fitted_at), ["2026-10-10T14:02:33"]);
  await Page.getByText("Rejected by you: the calibration it replaced was restored.", { exact: true }).waitFor();
  await Page.getByText("In use: the calibration fitted at 21:10", { exact: true }).waitFor();
  Assert.ok(await Reject.isDisabled(), "the restored calibration names no older one to restore");
  Assert.equal(Commands.filter(Command => Command.type === "send_ik").length, 10, "rejecting never moves the arm");

  // A user cancellation stays a cancellation even during preparation, sends
  // one stop, and does not run another pose or save another calibration.
  await Start.click();
  await Page.getByRole("button", { name: "Stop", exact: true }).click();
  await Page.getByText(/Failed: Calibration cancelled by user/).waitFor();
  await Start.waitFor({ state: "visible" });
  Assert.equal(StopsFinished, 2);
  Assert.equal(Commands.filter(Command => Command.type === "stop_motion").length, 2);
  Assert.equal(Commands.filter(Command => Command.type === "send_ik").length, 10);
  Assert.equal(Solves, 1);
  Assert.deepEqual(Errors, []);
  const Artifacts = Path.join(Backend, "TestArtifacts");
  MakeDirectory(Artifacts, { recursive: true });
  await Page.screenshot({ path: Path.join(Artifacts, "CalibrationUi.png"), fullPage: true });
  console.log("PASS: marker selection; missing-tip preflight; hardware fault reason; new timeout abort; old timeout recovery through 10 poses (one with a two-stage detour) and solve; a fit with problems saved and applied with its warnings; Reject calibration only on the user's confirmation, restoring the previous one; awaited single-stop cleanup; user cancellation; no browser errors.");
} finally {
  clearInterval(Timer);
  Timers.forEach(clearTimeout);
  await Browser.close(); await Server.Close();
}
