// Releasing a target where the arm cannot go: red for at most half a second (App.tsx
// TargetSnapLimitMs). A wrist angle that cannot reach turns to one that can, keeping the
// position; a position nothing can reach is pulled back along the edit to the reachable edge. Bridge faked.
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
const SnapLimitMs = 500, SlackMs = 150;  // slack: React commit plus the observer's frame, on this PC idle
// The half-second budget is wall clock. A search that misses it on a loaded PC restores the previous target ("No
// reachable spot found within half a second"): the budget working, not the search failing, so that release is tried
// again, three tries in all, and one that always misses fails. The slack scales with how busy the page is (one IK solve
// against its idle cost, 2026-10-08), as the node suites' time bounds do (audit 1 L5, audit 2's test table).
const IdleMsPerSolve = 1.1, Tries = 3;

const Server = await StartUiServer();
const Browser = await Chromium.launch(LaunchOptions());
const Page = await Browser.newPage({ viewport: { width: 1920, height: 1080 } });
const Errors = [], Sent = [];
Page.on('pageerror', Error => Errors.push(Error.message));
await Page.route('**/bridge-launch/**', Route => Route.fulfill({ json: { ok: true } }));
await Page.route('**/ollama/**', Route => Route.fulfill({ json: { models: [] } }));
await Page.route('**/depth-server/**', Route => Route.fulfill({ json: {} }));
await Page.route('**:8000/**', Route => {
  const Url = new URL(Route.request().url());
  if (Url.pathname === '/ports') return Route.fulfill({ json: [] });
  if (Url.pathname.startsWith('/depthcam/')) return Route.fulfill({ status: 404, json: { detail: 'Not Found' } });
  return Route.fulfill({ json: { ok: true, calibrated: false, points: [] } });
});
const Timers = new Set();
await Page.routeWebSocket('**/ws', Connected => {
  Connected.send(JSON.stringify(State));
  Connected.onMessage(Message => Sent.push(String(Message)));
  const Timer = setInterval(() => Connected.send(JSON.stringify(State)), 100); Timers.add(Timer);
  Connected.onClose(() => { clearInterval(Timer); Timers.delete(Timer); });
});

try {
  await Page.goto(Server.Url, { waitUntil: 'domcontentloaded' });
  const Canvas = Page.locator('canvas[data-target-state]');
  await Canvas.waitFor({ timeout: 20000 });
  await Page.waitForFunction(() => document.querySelector('canvas[data-target-state]')?.dataset.targetState === 'valid', null, { timeout: 20000 });
  // Record every target-state change with its time.
  await Page.evaluate(() => {
    const Element = document.querySelector('canvas[data-target-state]');
    window.TargetStates = [];
    new MutationObserver(() => window.TargetStates.push([performance.now(), Element.dataset.targetState]))
      .observe(Element, { attributes: true, attributeFilter: ['data-target-state'] });
  });
  const MsPerSolve = await Page.evaluate(async () => {
    const Api = await import('/src/lib/robotModel.ts');
    const Point = Api.HOME_MODEL_TARGET_CM;
    const Batches = Array.from({ length: 5 }, () => {
      const Started = performance.now();
      for (let Index = 0; Index < 40; Index += 1) Api.solveIkPreview(Point, 'up');
      return (performance.now() - Started) / 40;
    }).sort((A, B) => A - B);
    return Batches[2];
  });
  const Scale = Math.max(1, MsPerSolve / IdleMsPerSolve), Slack = SlackMs * Scale;
  // The release's outcome is the status beside "Arm Workspace" (set with the target, in the same render).
  const MissedBudget = () => Page.evaluate(() => document.querySelector('.scene-toolbar span')?.textContent.includes('within half a second') === true);
  let Retries = 0;
  const Read = async () => Page.evaluate(() => ({
    X: Number(document.querySelector('input[aria-label="Target X"]').value),
    Y: Number(document.querySelector('input[aria-label="Target Y"]').value),
    Z: Number(document.querySelector('input[aria-label="Target Z"]').value),
    Pitch: Number([...document.querySelectorAll('label.wide-field')].find(Label => Label.textContent.includes('Wrist pitch')).querySelector('input').value) }));
  // Type a value, release with Enter, and time how long the target stays red (again when the search missed its budget).
  const Release = async (Field, Value) => {
    for (let Try = 1; ; Try++) {
      await Field.focus(); await Field.fill(String(Value));
      await Page.waitForFunction(() => document.querySelector('canvas[data-target-state]').dataset.targetState === 'drag-invalid', null, { timeout: 5000 });
      const Pressed = await Page.evaluate(() => { window.TargetStates = []; return performance.now(); });
      await Field.press('Enter');
      await Page.waitForFunction(() => !['snapping', 'drag-invalid', 'invalid'].includes(document.querySelector('canvas[data-target-state]').dataset.targetState),
        null, { timeout: 5000 });
      const Done = await Page.evaluate(() => performance.now());
      if (!(await MissedBudget())) return Done - Pressed;
      Assert.ok(Try < Tries, `the release search missed its half-second budget ${Tries} times in a row (IK solve ${MsPerSolve.toFixed(2)} ms, load x${Scale.toFixed(2)})`);
      Retries += 1;
    }
  };
  const Start = await Read();
  const Pitch = Page.locator('label.wide-field', { hasText: 'Wrist pitch' }).locator('input');

  // A wrist angle the arm cannot hold at this position: the wrist turns, the position stays.
  let Adjusted = null;
  for (const Offset of [90, -90, 120, -120, 150]) {
    const Requested = Math.round(Start.Pitch + Offset);
    await Pitch.focus(); await Pitch.fill(String(Requested));
    const Invalid = await Page.waitForFunction(() => document.querySelector('canvas[data-target-state]').dataset.targetState === 'drag-invalid',
      null, { timeout: 1500 }).then(() => true, () => false);
    if (!Invalid) { await Pitch.press('Escape'); continue; }
    await Pitch.fill(String(Start.Pitch)); await Pitch.press('Escape');
    const RedMs = await Release(Pitch, Requested);
    Adjusted = { Requested, RedMs, After: await Read() };
    break;
  }
  Assert.ok(Adjusted, 'found a wrist angle the arm cannot reach at the start position');
  Assert.ok(Adjusted.RedMs <= SnapLimitMs + Slack, `red for ${Adjusted.RedMs.toFixed(0)} ms (load x${Scale.toFixed(2)})`);
  Assert.deepEqual([Adjusted.After.X, Adjusted.After.Y, Adjusted.After.Z], [Start.X, Start.Y, Start.Z], 'position kept');
  Assert.notEqual(Adjusted.After.Pitch, Adjusted.Requested, 'the wrist angle changed');
  Assert.equal(await Canvas.getAttribute('data-target-state'), 'valid');

  // A position nothing can reach: pulled back along the edit (here +Y only) to the reachable edge, within the limit.
  const Before = await Read();
  const FarMs = await Release(Page.locator('input[aria-label="Target Y"]'), 95);
  const After = await Read();
  Assert.ok(FarMs <= SnapLimitMs + Slack, `red for ${FarMs.toFixed(0)} ms (load x${Scale.toFixed(2)})`);
  Assert.deepEqual([After.X, After.Z], [Before.X, Before.Z], 'stays on the edited line');
  Assert.ok(After.Y > Before.Y && After.Y < 95, `pulled back to the reachable edge: Y ${Before.Y} -> ${After.Y}`);
  Assert.equal(await Canvas.getAttribute('data-target-state'), 'valid');

  Assert.equal(Sent.filter(Message => /"type":"(send_ik|move_servo|go_home)"/.test(Message)).length, 0, 'editing the target never moves the arm');
  Assert.deepEqual(Errors, []);
  console.log(`PASS: unreachable wrist angle turned in ${Adjusted.RedMs.toFixed(0)} ms keeping the position; unreachable position pulled back to Y ${After.Y} in ${FarMs.toFixed(0)} ms; no motion sent (IK solve ${MsPerSolve.toFixed(2)} ms, load x${Scale.toFixed(2)}, ${Retries} release(s) retried after a missed budget).`);
} finally {
  Timers.forEach(clearInterval);
  await Browser.close(); await Server.Close();
}
