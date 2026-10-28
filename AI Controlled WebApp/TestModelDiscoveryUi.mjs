// A page loaded before Ollama answers keeps looking for a model (and asks the
// dev server to start Ollama) instead of staying "offline" until a reload.
// Serves this checkout on its own port (UiTestLib.mjs); the bridge and Ollama are mocked.
import { chromium as Chromium } from "playwright";
import { LaunchOptions, StartUiServer, BackendPython } from "./UiTestLib.mjs";
import { execFileSync as Execute } from "node:child_process";
import { fileURLToPath as FilePath } from "node:url";
import Path from "node:path";
import Assert from "node:assert/strict";
const Root = Path.dirname(FilePath(import.meta.url)), Backend = Path.resolve(Root, "../back end");
const Python = BackendPython();
const State = JSON.parse(Execute(Python, ["-c", "import json, bridge_server; print(json.dumps(bridge_server.BridgeController().state_snapshot()))"], { cwd: Backend, encoding: "utf8" }));
const Server = await StartUiServer();
const Browser = await Chromium.launch(LaunchOptions());
const Errors = [], Timers = new Set();
let Ollama = "down", Launches = 0;

async function OpenPage() {
  const Page = await Browser.newPage({ viewport: { width: 1600, height: 900 } });
  Page.on("pageerror", Error => Errors.push(Error.message));
  await Page.route("**/bridge-launch/**", Route => {
    if (Route.request().url().endsWith("/bridge-launch/ollama")) { Launches += 1; return Route.fulfill({ json: { ok: false, status: "starting" } }); }
    return Route.fulfill({ json: { ok: true } });
  });
  await Page.route("**/depth-server/**", Route => Route.fulfill({ json: {} }));
  await Page.route("**/ollama/**", Route => {
    const Url = new URL(Route.request().url());
    // The Vite proxy answers a down Ollama with an error status.
    if (Ollama === "down") return Route.fulfill({ status: 500, body: "" });
    if (Ollama === "empty") {
      if (Url.pathname.endsWith("/v1/models")) return Route.fulfill({ json: { data: [] } });
      return Route.fulfill({ json: { models: [] } });
    }
    if (Url.pathname.endsWith("/tags")) return Route.fulfill({ json: { models: [{ name: "DiscoveryTest", size: 1000 }] } });
    if (Url.pathname.endsWith("/show")) return Route.fulfill({ json: { capabilities: ["vision"] } });
    if (Url.pathname.endsWith("/ps")) return Route.fulfill({ json: { models: [{ name: "DiscoveryTest", context_length: 32768 }] } });
    if (Url.pathname.endsWith("/api/chat")) return Route.fulfill({ json: { model: "DiscoveryTest", message: { role: "assistant", content: "" }, done: true, done_reason: "load" } });
    return Route.fulfill({ json: { data: [{ id: "DiscoveryTest" }] } });
  });
  await Page.route("**:8000/**", Route => {
    const Url = new URL(Route.request().url());
    if (Url.pathname === "/ports") return Route.fulfill({ json: [] });
    if (Url.pathname === "/ik/preview") return Route.fulfill({ json: { ok: false } });
    return Route.fulfill({ json: { ok: true, calibrated: false, points: [], point_count: 0 } });
  });
  await Page.routeWebSocket("**/ws", Socket => {
    Socket.send(JSON.stringify(State));
    const Timer = setInterval(() => Socket.send(JSON.stringify(State)), 200); Timers.add(Timer);
    Socket.onClose(() => { clearInterval(Timer); Timers.delete(Timer); });
  });
  await Page.goto(Server.Url, { waitUntil: "domcontentloaded" });
  return Page;
}

const Companion = Page => Page.locator(".ai-companion");
async function WaitFor(Page, Check, Label, Ms = 12000) {
  const Until = Date.now() + Ms;
  while (Date.now() < Until) { if (await Check()) return; await Page.waitForTimeout(150); }
  Assert.fail(`${Label}: state ${await Companion(Page).getAttribute("data-state")}, text "${await Companion(Page).innerText()}"`);
}

try {
  // 1. Ollama down at load: offline with the real reason, and a start request.
  const Page = await OpenPage();
  await WaitFor(Page, async () => /Waiting for Ollama/.test(await Companion(Page).textContent()), "waiting text");
  Assert.equal(await Companion(Page).getAttribute("data-state"), "offline");
  Assert.doesNotMatch(await Companion(Page).textContent(), /Settings/, "no pointer to a Settings control that does not exist");
  Assert.ok(Launches >= 1, "asked the dev server to start Ollama");
  // 2. Ollama comes up: the next retry finds the model without a reload.
  Ollama = "up";
  await WaitFor(Page, async () => (await Companion(Page).getAttribute("data-state")) !== "offline", "found the model after Ollama started");
  Assert.match(await Page.locator('[data-section="AiControl"]').textContent(), /DiscoveryTest ready \(vision\)/);
  const Before = Launches;
  // 3. A found model stays put: focus events do not search again.
  await Page.evaluate(() => window.dispatchEvent(new Event("focus")));
  await Page.waitForTimeout(500);
  Assert.equal(Launches, Before);
  await Page.close();

  // 4. Ollama up with no models: says so (and does not ask to start Ollama).
  Ollama = "empty"; Launches = 0;
  const Empty = await OpenPage();
  await WaitFor(Empty, async () => /no models installed/.test(await Companion(Empty).textContent()), "no-models text");
  Assert.equal(Launches, 0);
  await Empty.close();
  Assert.deepEqual(Errors, []);
  console.log("TestModelDiscoveryUi passed");
} finally {
  for (const Timer of Timers) clearInterval(Timer);
  await Browser.close(); await Server.Close();
}
