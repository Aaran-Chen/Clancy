import { chromium } from "playwright";
import { CadReadyMs, LaunchOptions, StartUiServer } from "./UiTestLib.mjs";
import { readFileSync, mkdirSync } from "node:fs";
import { execFileSync } from "node:child_process";
import { fileURLToPath } from "node:url";
import path from "node:path";
import assert from "node:assert/strict";

// A fully mocked visual/accessibility check: it never opens a real bridge or
// sends a command to the arm. It serves this checkout on its own port (UiTestLib.mjs).
const root = path.dirname(fileURLToPath(import.meta.url));
const backend = path.resolve(root, "../back end");
const python = readFileSync(path.join(root, "../.backend-python.txt"), "utf8").trim();
const state = JSON.parse(execFileSync(python, ["-c", "import json, bridge_server; print(json.dumps(bridge_server.BridgeController().state_snapshot()))"], { cwd: backend, encoding: "utf8" }));
const output = path.join(backend, "TestArtifacts", "ModernUi");
mkdirSync(output, { recursive: true });
const Server = await StartUiServer();
const browser = await chromium.launch(LaunchOptions());
const page = await browser.newPage({ viewport: { width: 1920, height: 1080 } });
const errors = [], commands = [], timers = new Set();
page.on("pageerror", error => errors.push(error.message));
await page.route("**/LocalDisplayPreferences.json", route => route.fulfill({ json: {} }));
await page.route("**/session-log/**", route => route.fulfill({ json: { ok: true } }));
await page.route("**/bridge-launch/**", route => route.fulfill({ json: { ok: true } }));
await page.route("**/depth-server/**", route => route.fulfill({ json: {} }));
await page.route("**/ollama/**", route => route.fulfill({ json: { models: [] } }));
await page.route("**:8000/**", route => {
  const url = new URL(route.request().url());
  if (url.pathname === "/ports") return route.fulfill({ json: [] });
  if (url.pathname === "/ik/preview") return route.fulfill({ json: { ok: false } });
  if (route.request().method() !== "GET") commands.push(url.pathname);
  return route.fulfill({ json: { ok: true, calibrated: false, points: [], point_count: 0 } });
});
await page.routeWebSocket("**/ws", socket => {
  socket.send(JSON.stringify(state));
  const timer = setInterval(() => socket.send(JSON.stringify(state)), 100);
  timers.add(timer);
  socket.onClose(() => { clearInterval(timer); timers.delete(timer); });
  socket.onMessage(message => commands.push(JSON.parse(String(message))));
});

async function assertFocusVisible(locator, label) {
  await locator.focus();
  const style = await locator.evaluate(element => {
    const css = getComputedStyle(element);
    return { visible: element.matches(":focus-visible"), width: parseFloat(css.outlineWidth), offset: parseFloat(css.outlineOffset), color: css.outlineColor };
  });
  assert.ok(style.visible && style.width >= 2, `${label} has a visible keyboard outline`);
  assert.notEqual(style.color, "rgba(0, 0, 0, 0)");
}

try {
  await page.goto(Server.Url, { waitUntil: "domcontentloaded" });
  await page.locator("canvas[data-cad-ready=true][data-assembly-ready=true]").waitFor({ timeout: CadReadyMs });
  // Minimize the movable camera so it does not obscure screenshot content.
  await page.getByRole("button", { name: "Minimize Live Feed", exact: true }).click();
  for (const [width, height] of [[1920, 1080], [1366, 768], [1280, 720]]) {
    await page.setViewportSize({ width, height });
    for (const mode of ["Manual", "AI"]) {
      await page.getByRole("button", { name: `${mode} control mode`, exact: true }).click();
      await page.waitForTimeout(500);
      const fit = await page.evaluate(() => {
        const scroll = document.scrollingElement;
        const emergency = document.querySelector(".topbar-emergency").getBoundingClientRect();
        return { width: scroll.scrollWidth, height: scroll.scrollHeight, emergencyRight: emergency.right, emergencyBottom: emergency.bottom, font: getComputedStyle(document.querySelector(".app-shell")).fontFamily };
      });
      assert.ok(fit.width <= width && fit.height <= height, `${mode} fits ${width}x${height}`);
      assert.ok(fit.emergencyRight <= width && fit.emergencyBottom <= height, "Emergency stop remains visible");
      assert.match(fit.font, /Play/, "Keep the existing arcade typeface");
      await page.keyboard.press("Tab");
      const toggle = page.locator(".section-toggle:visible").first();
      await assertFocusVisible(toggle, "Panel heading");
      const focusGeometry = await toggle.evaluate(element => {
        const box = element.getBoundingClientRect(), parent = element.closest(".dashboard-section").getBoundingClientRect();
        const css = getComputedStyle(element);
        return { top: box.top - parent.top, outline: parseFloat(css.outlineWidth) + parseFloat(css.outlineOffset) };
      });
      assert.ok(focusGeometry.top >= focusGeometry.outline, "Panel heading focus stays inside its clipped card");
      await page.screenshot({ path: path.join(output, `${mode}-${width}x${height}${process.env.UiShotSuffix ?? ""}.png`) });
    }
  }
  await page.setViewportSize({ width: 1366, height: 768 });
  await page.getByRole("button", { name: "Settings", exact: true }).click();
  const settings = page.getByRole("dialog", { name: "Settings", exact: true });
  await settings.waitFor();
  await page.keyboard.press("Tab");
  await assertFocusVisible(settings.getByRole("checkbox").first(), "Settings checkbox");
  await assertFocusVisible(settings.getByRole("combobox").first(), "Settings dropdown");
  await page.screenshot({ path: path.join(output, `Settings-1366x768${process.env.UiShotSuffix ?? ""}.png`) });
  await page.emulateMedia({ reducedMotion: "reduce" });
  const motion = await page.locator(".mode-rail button").first().evaluate(element => getComputedStyle(element).transitionDuration);
  assert.equal(motion, "0s", "Reduced motion removes decorative transitions");
  assert.deepEqual(commands, [], "UI polish must never send an arm command");
  assert.deepEqual(errors, [], "No browser errors");
  console.log("PASS: Manual/AI desktop and laptop layout, safety-control visibility, keyboard focus, reduced motion, screenshots; no arm commands.");
} catch (error) {
  console.error("Browser errors:", errors);
  await page.screenshot({ path: path.join(output, "Failure.png") });
  throw error;
} finally {
  for (const timer of timers) clearInterval(timer);
  await browser.close(); await Server.Close();
}

