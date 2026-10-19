import { chromium as Chromium } from "playwright";
import { LaunchOptions, StartUiServer } from "./UiTestLib.mjs";
import { readFileSync as ReadFile, mkdirSync as MakeDirectory, writeFileSync as WriteFile } from "node:fs";
import { execFileSync as Execute } from "node:child_process";
import { fileURLToPath as FilePath } from "node:url";
import Path from "node:path";
import Assert from "node:assert/strict";

const Root = Path.dirname(FilePath(import.meta.url));
const Backend = Path.resolve(Root, "../back end");
const Python = ReadFile(Path.join(Root, "../.backend-python.txt"), "utf8").trim();
const State = JSON.parse(ReadFile(Path.join(Backend, "TestArtifacts/ClippingLiveState.json"), "utf8"));
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
try {
  await Page.goto(Server.Url, { waitUntil: "domcontentloaded" });
  await Page.locator("canvas[data-cad-ready=true][data-assembly-ready=true]").waitFor();
  await Page.getByRole("button", { name: "Set Current", exact: true }).click();
  await Page.getByRole("button", { name: "Fit model in view", exact: true }).click();
  for (const View of ["ISO", "Front", "Top"]) {
    await Page.getByRole("button", { name: "View isometric", exact: true }).click();
    if (View !== "ISO") await Page.getByRole("button", { name: `View ${View.toLowerCase()}`, exact: true }).click();
    await Page.locator(".cad-shell").screenshot({ path: Path.join(Output, "Clipping" + View + ".png") });
  }
  Assert.deepEqual(Commands, []);
  Assert.deepEqual(Errors, []);
  console.log("Rendered saved live telemetry without hardware commands");
} finally { clearInterval(Timer); await Browser.close(); await Server.Close(); }
