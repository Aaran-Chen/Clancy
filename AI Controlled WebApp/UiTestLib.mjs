// What every browser (Playwright) suite shares, so that none of them can reach Aaran's live app, the arm or a camera
// (audit 2 L4: eleven suites went to the live app on 5181, the rest did without CadTestUrl).
//
//   import { StartUiServer, LaunchOptions } from "./UiTestLib.mjs";
//   const Server = await StartUiServer();                       // this checkout's app on its own port
//   const Browser = await Chromium.launch(LaunchOptions({ args: [...] }));
//   ...  await Page.goto(Server.Url)  ...
//   finally { await Browser.close(); await Server.Close(); }
//
// StartUiServer serves THIS checkout's app on the first free port of 5191-5199, from Vite in this process, without
// the repo's vite.config.ts: its plugin starts the session recorder and Ollama and answers /bridge-launch/start by
// starting a real bridge on 8000 (the arm's serial port). Here those dev-server routes answer 503 (every suite fakes
// them in the browser anyway), the server listens on 127.0.0.1 only, and Vite's cache goes to dist/ (git-ignored),
// never to node_modules/.vite, the live app's (in a git worktree node_modules is a junction to the main checkout's).
// CadTestUrl picks a server you started yourself instead; a live port there is refused.
//
// LaunchOptions makes every browser a suite opens map the live ports (the app 5180/5181, the bridge 8000, the depth
// server 8001, the recorder 8002, Ollama 11434) to a closed port, so a request a suite forgot to fake fails instead of
// reaching the arm, and gives it Chromium's fake camera, so no real camera is ever opened (the recorder holds the
// webcam; a suite that grants camera permission gets the synthetic test pattern).
import { execFileSync } from "node:child_process";
import Fs from "node:fs";
import Net from "node:net";
import Path from "node:path";
import { fileURLToPath } from "node:url";

const AppDir = Path.dirname(fileURLToPath(import.meta.url));
export const LivePorts = [5180, 5181, 8000, 8001, 8002, 11434];
export const UiPorts = [5191, 5192, 5193, 5194, 5195, 5196, 5197, 5198, 5199];
// How long a suite waits for the CAD view's first load (data-cad-ready, data-assembly-ready): the 6 MB robot model
// parsed and its shaders compiled in headless Edge's software WebGL. A few seconds idle, but past Playwright's default
// 30 s on a loaded PC (audit 2: TestBridgeSendUi "canvas not ready in 30 s" while other suites ran). The wait checks
// only that the view came up, so a longer one hides nothing; a view that never comes up still fails, later.
export const CadReadyMs = 120000;
const ClosedPort = "127.0.0.1:9";
// The dev server's own routes (vite.config.ts and scripts/*.ts): never served by a test server.
const SideRoutes = /^\/(bridge-launch|session-log|ollama|depth-server)(\/|$|\?)/;

// A suite's own programs (the state snapshot from bridge_server, recorders on free ports) never start the camera,
// the recorder or Ollama either, nor warm the router.
Object.assign(process.env, { SO101_DEPTH_CAMERA: "0", SO101_RECORDER: "0", SO101_OLLAMA_AUTOSTART: "0", SO101_ROUTER_WARMUP: "0" });

export function LiveRules() {
  return LivePorts.flatMap(Port => ["127.0.0.1", "localhost"].map(Host => `MAP ${Host}:${Port} ${ClosedPort}`)).join(", ");
}

/** chromium.launch options with the live ports closed and the fake camera; a suite's own host rules come after. */
export function LaunchOptions(Options = {}) {
  const Args = Options.args ?? [];
  const Theirs = Args.filter(Arg => Arg.startsWith("--host-resolver-rules=")).map(Arg => Arg.slice("--host-resolver-rules=".length));
  return { channel: "msedge", headless: true, ...Options,
    args: [...Args.filter(Arg => !Arg.startsWith("--host-resolver-rules=")), `--host-resolver-rules=${[LiveRules(), ...Theirs].join(", ")}`,
      "--use-fake-device-for-media-stream"] };
}

/** True when something already answers on 127.0.0.1:Port (another test server, the Y2K harness, ...). */
function Answers(Port) {
  return new Promise(Resolve => {
    const Socket = Net.connect({ host: "127.0.0.1", port: Port });
    const Done = Value => { Socket.destroy(); Resolve(Value); };
    Socket.setTimeout(400, () => Done(false));
    Socket.once("connect", () => Done(true));
    Socket.once("error", () => Done(false));
  });
}

/**
 * The recorded captures a replay reads, found as back end/RecordedCaptures.py finds them: SO101_REAL_CAPTURES (or
 * SO101_REPLAY_CAPTURES), else this checkout's back end/captures, else, in a git worktree, the main checkout's.
 */
export function RecordedCaptures() {
  const Named = process.env.SO101_REAL_CAPTURES || process.env.SO101_REPLAY_CAPTURES;
  if (Named) return Named;
  const Repo = Path.resolve(AppDir, "..");
  try {
    const [GitDir, Common] = execFileSync("git", ["-C", Repo, "rev-parse", "--path-format=absolute", "--git-dir", "--git-common-dir"],
      { encoding: "utf8" }).trim().split(/\r?\n/).map(Line => Path.resolve(Line));
    if (GitDir.toLowerCase() !== Common.toLowerCase()) return Path.join(Path.dirname(Common), "back end", "captures");
  } catch { /* Not a git checkout: its own folder. */ }
  return Path.join(Repo, "back end", "captures");
}

/** A recorded file kept in git for the tests (back end/TestFixtures/...). */
export function Fixture(...Parts) {
  return Path.join(AppDir, "..", "back end", "TestFixtures", ...Parts);
}

/**
 * An esbuild plugin that gives the bundle the Home the planner suites were written against (back end/TestFixtures/
 * HomePreset.json, home_preset.json as of 2026-09-30) instead of back end/home_preset.json, which Set Home rewrites on
 * this PC. Their fixtures start at INITIAL_SERVOS, and the wrist roll there turns every jaw axis: the Set Home of
 * 2026-10-10 (roll 1513 to 1590, uncommitted) failed 13 of TestGraspPlan's 48 and one of TestSceneUnverifiedReplay's 5.
 */
export const PinnedHome = { name: "pinned-home", setup(Builder) {
  Builder.onLoad({ filter: /home_preset\.json$/ }, () => ({ loader: "json", contents: Fs.readFileSync(Fixture("HomePreset.json"), "utf8") }));
} };

export function CheckedUrl(Text) {
  const Url = new URL(Text);
  const Port = Number(Url.port || (Url.protocol === "https:" ? 443 : 80));
  if (LivePorts.includes(Port)) {
    throw new Error(`CadTestUrl ${Text}: port ${Port} is a live service (5181 is Aaran's app). Leave CadTestUrl unset and the suite serves this checkout on 5191-5199.`);
  }
  return Url.origin;
}

/**
 * A Vite server Create(Port, CacheDir) builds, listening on the first free port of 5191-5199, its cache in dist/ (never
 * node_modules/.vite): { Url, Port, Server, Close }. Also for a suite that loads the real vite.config.ts.
 */
export async function ListenOnUiPort(Create) {
  for (const Port of UiPorts) {
    if (await Answers(Port)) continue;
    const Server = await Create(Port, Path.join(AppDir, "dist", `.ui-test-vite-cache-${Port}`));
    try {
      await Server.listen();
    } catch (Failure) {
      await Server.close().catch(() => {});
      if (/already in use|EADDRINUSE/i.test(String(Failure?.message ?? Failure))) continue;
      throw Failure;
    }
    return { Url: `http://127.0.0.1:${Port}`, Port, Server, Close: () => Server.close() };
  }
  throw new Error(`No free UI test port in ${UiPorts[0]}-${UiPorts.at(-1)} (each serves one suite at a time).`);
}

/** What a page said (console lines, page errors, failed requests), the last 200, for a suite's failure report (WaitForCad). */
export function RecordPage(Page) {
  const Lines = [];
  const Keep = Line => { Lines.push(`${new Date().toISOString().slice(11, 23)} ${Line}`); if (Lines.length > 200) Lines.shift(); };
  Page.on("console", Message => Keep(`[console.${Message.type()}] ${Message.text()}`));
  Page.on("pageerror", Failure => Keep(`[pageerror] ${Failure.message}`));
  Page.on("requestfailed", Request => Keep(`[requestfailed] ${Request.method()} ${Request.url()}: ${Request.failure()?.errorText ?? ""}`));
  return Lines;
}

/**
 * The CAD view's wait every suite does (data-cad-ready, data-assembly-ready), which on a timeout says what it can before
 * failing as before: the canvas's data attributes, the page's last console lines and errors (RecordPage), and a
 * screenshot in back end/TestArtifacts. TestRobotHud once timed out after a reload and left nothing to go on (2026-10-08).
 */
export async function WaitForCad(Page, { Lines = [], Name = "suite", Timeout = CadReadyMs } = {}) {
  const Canvas = Page.locator("canvas[data-cad-ready=true][data-assembly-ready=true]");
  try {
    await Canvas.waitFor({ timeout: Timeout });
    return Canvas;
  } catch (Failure) {
    const Shot = Path.join(AppDir, "..", "back end", "TestArtifacts", `${Name}-cad-not-ready.png`);
    const Data = await Page.evaluate(() => [...document.querySelectorAll("canvas")].map(Canvas => ({ ...Canvas.dataset }))).catch(Error => `unreadable: ${Error.message}`);
    try { Fs.mkdirSync(Path.dirname(Shot), { recursive: true }); await Page.screenshot({ path: Shot, timeout: 15000 }); } catch { /* The page may be gone. */ }
    console.error([`CAD view not ready after ${Timeout} ms in ${Name}. Canvases: ${JSON.stringify(Data)}. Screenshot: ${Shot}`,
      "The page's last lines:", ...(Lines.length ? Lines.slice(-40) : ["(none recorded)"])].join("\n"));
    throw Failure;
  }
}

/** This checkout's app on its own port (or CadTestUrl's server): { Url, Port, Close }. */
export async function StartUiServer({ LogLevel = "warn" } = {}) {
  if (process.env.CadTestUrl) {
    const Url = CheckedUrl(process.env.CadTestUrl);
    return { Url, Port: Number(new URL(Url).port), Close: async () => {} };
  }
  const { createServer } = await import("vite");
  const React = (await import("@vitejs/plugin-react")).default;
  const NodeModules = Fs.realpathSync(Path.join(AppDir, "node_modules"));
  const RefuseSideRoutes = {
    name: "ui-test-refuse-side-routes",
    configureServer(Server) {
      Server.middlewares.use((Request, Response, Next) => {
        if (!SideRoutes.test(Request.url ?? "")) return Next();
        Response.statusCode = 503;
        Response.setHeader("Content-Type", "application/json");
        Response.end(JSON.stringify({ ok: false, error: "UI test server: the bridge, Ollama, depth and session-log routes are faked by the test, not served." }));
      });
    },
  };
  return ListenOnUiPort((Port, CacheDir) => createServer({
    configFile: false, root: AppDir, cacheDir: CacheDir, clearScreen: false, logLevel: LogLevel,
    // Vite's pre-bundled packages sit in the cache under dist/, outside node_modules: no React transform for them.
    plugins: [React({ exclude: [/\/dist\/\.ui-test-vite-cache-\d+\//] }), RefuseSideRoutes],
    server: {
      host: "127.0.0.1", port: Port, strictPort: true, open: false,
      // src/lib imports ../../back end/*.json; in a worktree the packages are behind the node_modules junction.
      fs: { allow: [Path.resolve(AppDir, ".."), AppDir, NodeModules],
        deny: [".env", ".env.*", "*.{crt,pem}", "**/.git/**", "RemotePassword.json*", "ConnectIntent.json*", "*.log"] },
    },
  }));
}
