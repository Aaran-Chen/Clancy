// Who may use the dev server (scripts/RemoteAccess.ts, first in vite.config.ts's plugin): this PC as before, another
// device only with the remote password, another site's page never; another device gets the page's own boot without it,
// and Vite's /__open-in-editor never. The last test serves the page from real Vite with the real vite.config.ts on a test
// port (5191-5199, cache in dist/) and walks its whole module graph as another device.
// A stub server on real sockets runs the middleware: 127.0.0.1 is this PC, the PC's own LAN or tailnet address stands
// in for another device (a client connecting to it has that address, not loopback). The password file is made by
// back end/SetRemotePassword.py in a scratch folder, so the scrypt parameters are checked across Python and Node.
// vite.config.ts itself is loaded with Vite's in-memory "runner" loader (no node_modules/.vite-temp file) and its
// plugin's configureServer is called on a stub (nothing starts: SO101_RECORDER=0, Ollama at a closed port).
import { build as Build } from "esbuild";
import { test as Test, before as Before, after as After } from "node:test";
import Assert from "node:assert/strict";
import { execFileSync as Execute } from "node:child_process";
import Fs from "node:fs";
import Http from "node:http";
import Os from "node:os";
import Path from "node:path";
import { ListenOnUiPort, BackendPython } from "./UiTestLib.mjs";

const App = import.meta.dirname, Backend = Path.resolve(App, "../back end");
const Scratch = Fs.mkdtempSync(Path.join(Os.tmpdir(), "so101-dev-access-"));
Object.assign(process.env, { SO101_REMOTE_PASSWORD_FILE: Path.join(Scratch, "RemotePassword.json"), SO101_RECORDER: "0",
  SO101_OLLAMA_AUTOSTART: "0", OLLAMA_BASE_URL: "http://127.0.0.1:9", DEPTH_BASE_URL: "http://127.0.0.1:9",
  SO101_CAPTURES_ROOT: Path.join(Scratch, "captures") });
const Python = BackendPython();
const Load = async File => {
  const Bundle = await Build({ entryPoints: [File], bundle: true, write: false, platform: "node", format: "esm" });
  return import(`data:text/javascript;base64,${Buffer.from(Bundle.outputFiles[0].text).toString("base64")}`);
};
const Access = await Load("scripts/RemoteAccess.ts");

// This PC's own non-loopback IPv4 address (LAN or tailnet), or null.
const Lan = Object.values(Os.networkInterfaces()).flat().find(Item => Item && Item.family === "IPv4" && !Item.internal)?.address ?? null;
let Servers = [], Port = 0;

function SetPassword() {
  const Output = Execute(Python, ["SetRemotePassword.py", "--generate"], { cwd: Backend, encoding: "utf8", env: process.env });
  return /Remote password: (\S+)/.exec(Output)[1];
}

// One request to the stub; Host defaults to the address dialled. Resolves { Status, Body }.
function Request(Address, Method, Url, Headers = {}) {
  return new Promise((Resolve, Reject) => {
    const Sent = Http.request({ host: Address, port: Port, method: Method, path: Url, headers: { host: `${Address}:${Port}`, ...Headers } }, Reply => {
      const Chunks = [];
      Reply.on("data", Chunk => Chunks.push(Chunk));
      Reply.on("end", () => Resolve({ Status: Reply.statusCode, Body: JSON.parse(Buffer.concat(Chunks).toString("utf8") || "{}") }));
    });
    Sent.on("error", Reject);
    Sent.end();
  });
}
const Reached = Reply => Reply.Status === 200 && Reply.Body.reached === true;
// The guard's refusal code in a reply's text, or what answered instead (a route or Vite itself got the request).
const AccessOf = Text => { try { return JSON.parse(Text).access ?? `no refusal: ${Text.slice(0, 100)}`; } catch { return `not the guard: ${Text.slice(0, 100)}`; } };

Before(async () => {
  const Guard = Access.RemoteAccessMiddleware(Backend);
  const Handle = (Req, Res) => Guard(Req, Res, Failure => {
    Res.statusCode = Failure ? 500 : 200;
    Res.end(JSON.stringify({ reached: !Failure, url: Req.url, password: Req.headers["x-clancy-password"] ?? null }));
  });
  const Local = Http.createServer(Handle);
  await new Promise(Resolve => Local.listen(0, "127.0.0.1", Resolve));
  Port = Local.address().port;
  Servers.push(Local);
  if (Lan) {
    const Remote = Http.createServer(Handle);
    await new Promise(Resolve => Remote.listen(Port, Lan, Resolve));
    Servers.push(Remote);
  }
});
After(() => {
  for (const Server of Servers) Server.close();
  Fs.rmSync(Scratch, { recursive: true, force: true });
});

Test("the rule's parts: Origin, Sec-Fetch-Site, loopback, Host, proxy headers", () => {
  const Host = "192.168.1.20:5181";
  Assert.equal(Access.OriginRefusal({ host: Host }), null, "a program");
  Assert.equal(Access.OriginRefusal({ host: Host, origin: `http://${Host}` }), null, "this server's own page");
  Assert.equal(Access.OriginRefusal({ host: "[::1]:5181", origin: "http://[::1]:5181" }), null);
  for (const Origin of ["http://evil.example", "http://192.168.1.20:3000", "http://192.168.1.20:8000", "null", "file://", "http://192.168.1.20"]) {
    Assert.notEqual(Access.OriginRefusal({ host: Host, origin: Origin }), null, Origin);
  }
  for (const Site of ["cross-site", "same-site"]) Assert.notEqual(Access.OriginRefusal({ host: Host, "sec-fetch-site": Site }), null, Site);
  for (const Site of ["same-origin", "none"]) Assert.equal(Access.OriginRefusal({ host: Host, "sec-fetch-site": Site }), null, Site);
  Assert.equal(Access.IsLocal("127.0.0.1", { host: "127.0.0.1:5181" }), true);
  Assert.equal(Access.IsLocal("::ffff:127.0.0.1", { host: "localhost:5181" }), true);
  Assert.equal(Access.IsLocal("::1", { host: "[::1]:5181" }), true);
  Assert.equal(Access.IsLocal("127.0.0.1", { host: "evil.example:5181" }), false, "DNS rebinding");
  Assert.equal(Access.IsLocal("127.0.0.1", { host: "192.168.1.20:5181" }), false, "this PC by its LAN name");
  Assert.equal(Access.IsLocal("127.0.0.1", { host: "127.0.0.1:5181", "x-forwarded-for": "100.64.0.9" }), false, "a local proxy");
  for (const Client of ["192.168.1.30", "::ffff:192.168.1.30", "100.101.102.103", "fe80::1", "", undefined]) {
    Assert.equal(Access.IsLocal(Client, { host: "127.0.0.1:5181" }), false, String(Client));
  }
  Assert.equal(Access.GuardedRoutes.test("/ollama/api/chat"), true);
  Assert.equal(Access.GuardedRoutes.test("/bridge-launch/start"), true);
  Assert.equal(Access.GuardedRoutes.test("/session-log/event"), true);
  Assert.equal(Access.GuardedRoutes.test("/depth-server/depth"), true);
  for (const Open of ["/", "/index.html", "/src/main.tsx", "/@vite/client", "/LocalDisplayPreferences.json"]) {
    Assert.equal(Access.GuardedRoutes.test(Open), false, Open);
  }
  // Matched as Vite's router (connect) and proxy match them: any case, a plain prefix (Vite proxies "/ollamax" too).
  for (const Guarded of ["/OLLAMA/api/chat", "/Bridge-Launch/start", "/session-log.x", "/ollamax"]) Assert.equal(Access.GuardedRoutes.test(Guarded), true, Guarded);
  for (const Editor of ["/__OPEN-IN-EDITOR", "/__open-in-editor.x", "/__Open-In-Editor/x"]) Assert.equal(Access.PcOnlyRoutes.test(Editor), true, Editor);
  // Every reading of a target's path: raw (as connect reads it), backslashes as slashes, decoded and normalised.
  Assert.deepEqual(Access.RoutePaths("/__OPEN-IN-EDITOR/../src/main.tsx?file=x"), ["/__OPEN-IN-EDITOR/../src/main.tsx", "/__OPEN-IN-EDITOR/../src/main.tsx", "/src/main.tsx"]);
  Assert.deepEqual(Access.RoutePaths("http://192.168.1.20:5181/__open-in-editor?file=x"), ["/__open-in-editor", "/__open-in-editor", "/__open-in-editor"]);
  Assert.equal(Access.BootTarget("/src/main.tsx?t=1"), "/src/main.tsx");
  Assert.equal(Access.BootTarget("/@fs/C:/Users/me/back%20end/BridgeVersion.json?import"), "/@fs/C:/Users/me/back end/BridgeVersion.json");
  for (const Changed of ["/__OPEN-IN-EDITOR/../src/main.tsx", "/src/./main.tsx", "/src/%2e%2e/vite.config.ts", "/src%2fmain.tsx", "/src\\main.tsx",
    "/src/main.tsx#x", "http://192.168.1.20:5181/src/main.tsx", "*"]) {
    Assert.equal(Access.BootTarget(Changed), null, Changed);
  }
});

Test("this PC needs nothing; another site's page is refused", async () => {
  Access.ForgetAttempts();
  const Own = `http://127.0.0.1:${Port}`;
  Assert.ok(Reached(await Request("127.0.0.1", "GET", "/ollama/api/tags")), "a program or a same-origin GET");
  Assert.ok(Reached(await Request("127.0.0.1", "POST", "/session-log/event", { origin: Own })), "Clancy's page on this PC");
  Assert.ok(Reached(await Request("127.0.0.1", "GET", "/ollama/api/ps", { "sec-fetch-site": "same-origin" })));
  for (const Origin of ["http://evil.example:5181", "http://127.0.0.1:3000", "http://127.0.0.1:8000"]) {
    const Reply = await Request("127.0.0.1", "POST", "/bridge-launch/restart", { origin: Origin });
    Assert.deepEqual([Reply.Status, Reply.Body.access], [403, "cross_site"], Origin);
  }
  const Image = await Request("127.0.0.1", "GET", "/bridge-launch/start", { "sec-fetch-site": "cross-site", "sec-fetch-mode": "no-cors" });
  Assert.deepEqual([Image.Status, Image.Body.access], [403, "cross_site"], "another page's <img> or link");
});

Test("another device: the page loads, its routes need the password Python stored", { skip: Lan ? false : "no LAN or tailnet address" }, async () => {
  Access.ForgetAttempts();
  Fs.rmSync(process.env.SO101_REMOTE_PASSWORD_FILE, { force: true });
  const Page = `http://${Lan}:${Port}`;
  for (const Open of ["/", "/src/main.tsx", "/@vite/client"]) Assert.ok(Reached(await Request(Lan, "GET", Open)), Open);
  let Reply = await Request(Lan, "GET", "/ollama/api/tags", { "x-clancy-password": "anything at all" });
  Assert.deepEqual([Reply.Status, Reply.Body.access], [403, "remote_off"], "no password set: other devices get nothing");
  const Password = SetPassword();
  for (const [Key, Code] of [[undefined, "password_required"], ["", "password_required"], ["wrong password", "password_wrong"]]) {
    Reply = await Request(Lan, "POST", "/ollama/api/chat", { origin: Page, ...(Key === undefined ? {} : { "x-clancy-password": Key }) });
    Assert.deepEqual([Reply.Status, Reply.Body.access], [401, Code], String(Key));
  }
  const Passed = await Request(Lan, "POST", "/ollama/api/chat", { origin: Page, "x-clancy-password": Password });
  Assert.ok(Reached(Passed), "Python's scrypt hash checks out in Node");
  Assert.equal(Passed.Body.password, null, "the password stops at the rule: no route or proxy behind it sees it");
  Assert.ok(Reached(await Request(Lan, "GET", "/ollama/api/tags", { "sec-fetch-site": "same-origin", "x-clancy-password": Password })));
  Reply = await Request(Lan, "POST", "/session-log/event", { origin: "http://evil.example:5181", "x-clancy-password": Password });
  Assert.deepEqual([Reply.Status, Reply.Body.access], [403, "cross_site"], "the password does not let another site in");
  // From loopback but not this PC: through a local proxy, or a rebound name.
  Reply = await Request("127.0.0.1", "GET", "/ollama/api/tags", { "x-forwarded-for": "100.64.0.9" });
  Assert.equal(Reply.Status, 401);
  Reply = await Request("127.0.0.1", "GET", "/ollama/api/tags", { host: `evil.example:${Port}`, origin: `http://evil.example:${Port}` });
  Assert.equal(Reply.Status, 401);
  // Five different wrong passwords lock that address out, the right one too; the same wrong one again counts once.
  Access.ForgetAttempts();
  for (let Index = 0; Index < 8; Index++) await Request(Lan, "GET", "/ollama/api/tags", { "x-clancy-password": "the old password" });
  Assert.ok(Reached(await Request(Lan, "GET", "/ollama/api/tags", { "x-clancy-password": Password })), "a stale password retried is one failure");
  for (let Index = 0; Index < 5; Index++) await Request(Lan, "GET", "/ollama/api/tags", { "x-clancy-password": `guess ${Index}` });
  Reply = await Request(Lan, "GET", "/ollama/api/tags", { "x-clancy-password": Password });
  Assert.deepEqual([Reply.Status, Reply.Body.access], [429, "locked_out"]);
  Assert.deepEqual(await Access.CheckPassword(Password, "192.168.1.31", process.env.SO101_REMOTE_PASSWORD_FILE), [true, null], "another address is not locked out");
  Assert.ok(Reached(await Request("127.0.0.1", "GET", "/ollama/api/tags")), "this PC is not affected");
});

Test("another device: the page's own boot without the password, the rest with it, Vite's editor route never", { skip: Lan ? false : "no LAN or tailnet address" }, async () => {
  // 2026-10-08 access review: the guard covered its four routes only and handed every other path to Vite, among them
  // /__open-in-editor (starts the editor on this PC with a file in it, through the shell on Windows) and /@fs/.
  Access.ForgetAttempts();
  Fs.rmSync(process.env.SO101_REMOTE_PASSWORD_FILE, { force: true });
  const Imported = Access.AppBackendImports(App);
  for (const Name of ["BridgeVersion.json", "CadModel.json", "WorkspaceFrame.json", "gui_limits.json"]) Assert.ok(Imported.has(Name), Name);
  Assert.ok(!Imported.has("camera_calibration.json") && !Imported.has("RemotePassword.json"));
  const Backend_ = Name => encodeURI(`/@fs/${Path.join(Backend, Name).replaceAll("\\", "/")}`);
  const Boot = ["/", "/?t=1", "/index.html", "/src/main.tsx", "/src/styles.css?direct", "/@vite/client", "/@vite/env", "/@react-refresh",
    "/node_modules/.vite/deps/react.js?v=1", "/node_modules/@fontsource/play/files/play-latin-400-normal.woff2", "/Cad/SO101Parts.glb",
    "/Fonts/Play-LICENSE.txt", `${Backend_("BridgeVersion.json")}?import`, `${Backend_("CadModel.json")}?import`];
  for (const Url of Boot) Assert.ok(Reached(await Request(Lan, "GET", Url)), Url);
  Assert.equal((await Request(Lan, "HEAD", "/src/main.tsx")).Status, 200, "HEAD (no body)");
  const Rest = ["/vite.config.ts", "/package.json", "/scripts/RemoteAccess.ts", "/UiTestLib.mjs", "/api-output.log", "/dist/x.js",
    "/@fs/C:/Windows/win.ini", Backend_("camera_calibration.json"), Backend_("RemotePassword.json"), Backend_("bridge_server.py"),
    "/src/..%2fvite.config.ts", "/node_modules/..%2f..%2fback%20end/RemotePassword.json", "/Cad/%2e%2e/vite.config.ts",
    "/node_modules%5c..%5cvite.config.ts", "/Cad/does-not-exist.glb", "/@id/C:/Windows/win.ini"];
  for (const Url of Rest) {
    const Reply = await Request(Lan, "GET", Url);
    Assert.deepEqual([Reply.Status, Reply.Body.access], [403, "remote_off"], Url);
  }
  let Reply = await Request(Lan, "POST", "/src/main.tsx");
  Assert.deepEqual([Reply.Status, Reply.Body.access], [403, "remote_off"], "only GET and HEAD for the page");
  for (const Url of ["/__open-in-editor?file=src/App.tsx", "/__open-in-editor", "/__open-in-editor/x", "/__OPEN-IN-EDITOR?file=src/App.tsx",
    "/__OPEN-IN-EDITOR/../src/main.tsx?file=src/App.tsx", "/__Open-In-Editor.x/../../src/main.tsx"]) {
    Reply = await Request(Lan, "GET", Url);
    Assert.deepEqual([Reply.Status, Reply.Body.access], [403, "pc_only"], Url);
  }
  // A guarded route under a path that normalises to the page's boot (Vite's router reads the raw path, any case).
  for (const Url of ["/BRIDGE-LAUNCH/start/../../src/main.tsx", "/Session-Log/event/../../src/main.tsx", "/bridge-launch/start/../../src/main.tsx",
    "/OLLAMA/api/tags", "/src/main.tsx/../../bridge-launch/start"]) {
    Reply = await Request(Lan, "GET", Url);
    Assert.deepEqual([Reply.Status, Reply.Body.access], [403, "remote_off"], Url);
  }
  const Password = SetPassword();
  Reply = await Request(Lan, "GET", "/vite.config.ts");
  Assert.deepEqual([Reply.Status, Reply.Body.access], [401, "password_required"]);
  Assert.ok(Reached(await Request(Lan, "GET", "/vite.config.ts", { "x-clancy-password": Password })), "with the password, the rest too");
  Reply = await Request(Lan, "GET", "/__open-in-editor?file=src/App.tsx", { "x-clancy-password": Password });
  Assert.deepEqual([Reply.Status, Reply.Body.access], [403, "pc_only"], "the password does not open an editor on the PC");
  // This PC: its own page (the error overlay asks same-origin) and programs, never another site's page or a proxy.
  Assert.ok(Reached(await Request("127.0.0.1", "GET", "/__open-in-editor?file=src/App.tsx")), "a program");
  Assert.ok(Reached(await Request("127.0.0.1", "GET", "/__open-in-editor?file=src/App.tsx", { "sec-fetch-site": "same-origin" })), "the overlay");
  for (const Headers of [{ "sec-fetch-site": "cross-site", "sec-fetch-mode": "navigate" }, { origin: "http://evil.example" },
    { origin: `http://127.0.0.1:${Port + 1}` }]) {
    Reply = await Request("127.0.0.1", "GET", "/__open-in-editor?file=src/App.tsx", Headers);
    Assert.deepEqual([Reply.Status, Reply.Body.access], [403, "cross_site"], JSON.stringify(Headers));
  }
  // Another site's page on this PC, by another case of a guarded route's name (Vite's router takes any case).
  for (const Url of ["/BRIDGE-LAUNCH/start", "/Session-Log/recording", "/Bridge-Launch/ollama", "/__OPEN-IN-EDITOR?file=src/App.tsx"]) {
    Reply = await Request("127.0.0.1", "POST", Url, { origin: "http://evil.example" });
    Assert.deepEqual([Reply.Status, Reply.Body.access], [403, "cross_site"], Url);
  }
  Reply = await Request("127.0.0.1", "GET", "/__open-in-editor?file=src/App.tsx", { "x-forwarded-for": "100.64.0.9" });
  Assert.deepEqual([Reply.Status, Reply.Body.access], [403, "pc_only"], "through a local proxy is not this PC");
  for (const Url of ["/vite.config.ts", "/package.json", "/src/main.tsx"]) Assert.ok(Reached(await Request("127.0.0.1", "GET", Url)), `this PC: ${Url}`);
});

Test("a new password works without a restart; removing it turns remote access off", async () => {
  Access.ForgetAttempts();
  const File = process.env.SO101_REMOTE_PASSWORD_FILE;
  const First = SetPassword();
  Assert.deepEqual(await Access.CheckPassword(First, "a", File), [true, null]);
  Fs.utimesSync(File, new Date(1000), new Date(1000));  // a different mtime even within the clock's tick
  const Second = SetPassword();
  Assert.deepEqual(await Access.CheckPassword(First, "a", File), [false, "password_wrong"]);
  Assert.deepEqual(await Access.CheckPassword(Second, "a", File), [true, null]);
  Execute(Python, ["SetRemotePassword.py", "--off"], { cwd: Backend, encoding: "utf8", env: process.env });
  Assert.deepEqual(await Access.CheckPassword(Second, "a", File), [false, "remote_off"]);
});

Test("vite.config.ts: the guard runs before every route and proxy, and the machine files are never served", async () => {
  const { resolveConfig: ResolveConfig } = await import("vite");
  const Config = await ResolveConfig({ root: App, configFile: Path.join(App, "vite.config.ts"), configLoader: "runner",
    cacheDir: Path.join(Scratch, "vite-cache"), logLevel: "silent" }, "serve");
  for (const Name of ["RemotePassword.json", "ConnectIntent.json", "RemotePassword.json.123.tmp"]) {
    Assert.equal(Config.fsDenyGlob(Path.join(Backend, Name).replaceAll("\\", "/")), true, Name);
  }
  Assert.equal(Config.fsDenyGlob(Path.join(Backend, "BridgeVersion.json").replaceAll("\\", "/")), false, "the app's own imports still load");
  Assert.equal(Config.fsDenyGlob(Path.join(App, ".env").replaceAll("\\", "/")), true, "Vite's own defaults kept");
  Assert.equal(Config.fsDenyGlob(Path.join(App, "api-output.log").replaceAll("\\", "/")), true, "the bridge's logs are not served");
  const Plugin = Config.plugins.find(Item => Item.name === "so101-bridge-launcher");
  const Used = [];
  await Plugin.configureServer({ middlewares: { use: (...Args) => Used.push(Args) } });
  Assert.ok(Used.length >= 5);
  Assert.equal(Used[0].length, 1, "the first middleware has no path: it sees every request");
  Assert.deepEqual(Used.slice(1).map(Args => Args[0]), ["/session-log/event", "/session-log/recording", "/bridge-launch/ollama", "/bridge-launch/restart",
    "/bridge-launch/start"]);
  // The first one is the guard: another device without the password never reaches /bridge-launch/start.
  Access.ForgetAttempts();
  SetPassword();
  const Outcome = await new Promise(Resolve => {
    const Response = { statusCode: 200, headers: {}, setHeader(Name, Value) { this.headers[Name] = Value; },
      end: Body => Resolve({ Status: Response.statusCode, Body: JSON.parse(Body) }) };
    Used[0][0]({ url: "/bridge-launch/start", method: "POST", headers: { host: "192.168.1.20:5181" }, socket: { remoteAddress: "192.168.1.30" } },
      Response, () => Resolve({ Status: "passed on" }));
  });
  Assert.deepEqual([Outcome.Status, Outcome.Body?.access], [401, "password_required"]);
});

Test("vite.config.ts: a bare npm run dev:web serves Clancy's port 5181, and fails rather than drift when it is taken", async () => {
  // Vite's default 5173 is no dashboard port, so the bridge refused that page (audit 2 L3); run-app.ps1 adds --port 5181.
  const { resolveConfig: ResolveConfig } = await import("vite");
  const Config = await ResolveConfig({ root: App, configFile: Path.join(App, "vite.config.ts"), configLoader: "runner",
    cacheDir: Path.join(Scratch, "vite-cache"), logLevel: "silent" }, "serve");
  Assert.deepEqual([Config.server.port, Config.server.strictPort], [5181, true]);
  const Scripts = JSON.parse(Fs.readFileSync(Path.join(App, "package.json"), "utf8")).scripts;
  Assert.doesNotMatch(Scripts["dev:web"], /--port/, "the port comes from vite.config.ts (run-app.ps1 passes --port 5181 again)");
});

Test("real Vite with vite.config.ts: another device without the password loads the whole page, and nothing else", async () => {
  // Every module the page loads, walked from index.html through Vite's own rewritten imports (static, dynamic, the grasp
  // planner's worker, CSS and its fonts), plus the assets the page fetches itself; asked from 127.0.0.1 by a Host that
  // is no loopback name, which the rule takes for another device (as a page Edge loads from 192.0.2.10 would be).
  Access.ForgetAttempts();
  Fs.rmSync(process.env.SO101_REMOTE_PASSWORD_FILE, { force: true });  // no password set: the strictest case
  const { createServer: CreateServer } = await import("vite");
  const Started = await ListenOnUiPort((UiPort, CacheDir) => CreateServer({ configFile: Path.join(App, "vite.config.ts"), configLoader: "runner",
    cacheDir: CacheDir, logLevel: "silent", server: { port: UiPort, host: "127.0.0.1", strictPort: true } }));
  const Remote = `192.0.2.10:${Started.Port}`, ThisPc = `127.0.0.1:${Started.Port}`;
  const Get = (Url, Host = Remote, Method = "GET", Headers = {}) => new Promise((Resolve, Reject) => {
    const Sent = Http.request({ host: "127.0.0.1", port: Started.Port, method: Method, path: Url.replaceAll(" ", "%20"), headers: { host: Host, ...Headers } }, Reply => {
      const Chunks = [];
      Reply.on("data", Chunk => Chunks.push(Chunk));
      Reply.on("end", () => Resolve({ Status: Reply.statusCode, Type: String(Reply.headers["content-type"] ?? ""), Text: Buffer.concat(Chunks).toString("utf8") }));
    });
    Sent.on("error", Reject);
    Sent.end();
  });
  try {
    const Seen = new Map(), Queue = ["/"];
    // Vite's rewritten imports are quoted absolute URLs, with the folders' spaces as they are (the browser encodes them);
    // CSS asks for its fonts with url(...).
    const Absolute = /(?:["'`]|url\()(\/(?:src\/|node_modules\/|@vite\/|@fs\/|@id\/|@react-refresh)[^"'`)\n\\]*)/g;
    while (Queue.length) {
      const Url = Queue.shift();
      if (Seen.has(Url) || Seen.size > 1500) continue;
      const Reply = await Get(Url);
      Seen.set(Url, Reply.Status);
      if (Reply.Status !== 200 || !/javascript|html|css/.test(Reply.Type)) continue;
      for (const Match of Reply.Text.matchAll(Absolute)) {
        if (!Seen.has(Match[1]) && !/\$\{|\*/.test(Match[1])) Queue.push(Match[1]);
      }
    }
    for (const Asset of ["/Cad/SO101Parts.glb", "/Cad/XLeRobotBody.glb", "/LocalDisplayPreferences.json"]) {
      if (Fs.existsSync(Path.join(App, "public", Asset))) Seen.set(Asset, (await Get(Asset)).Status);
    }
    // Whatever this PC gets, another device gets too. (In a git worktree Vite itself refuses the fonts: its node_modules
    // is a junction to the main checkout's, outside Vite's serving allow list. The main checkout serves them.)
    const Failed = [];
    for (const [Url, Status] of Seen) {
      if (Status !== 200 && (await Get(Url, ThisPc)).Status === 200) Failed.push([Url, Status]);
    }
    Assert.deepEqual(Failed, [], "every module and asset of the page loads without the password");
    Assert.ok([...Seen.values()].filter(Status => Status === 200).length > 50, "the page's modules load");
    const Urls = [...Seen.keys()];
    for (const Part of ["/src/main.tsx", "/src/App.tsx", "/@vite/client", "/@react-refresh", "GraspPlanWorker", "BridgeVersion.json", "woff2"]) {
      Assert.ok(Urls.some(Url => Url.includes(Part)), `the walk reached ${Part}`);
    }
    // What the page never needs stays closed: Vite's editor route, this folder's other files, /@fs/ elsewhere.
    for (const [Url, Code] of [["/__open-in-editor?file=src/App.tsx", "pc_only"], ["/vite.config.ts", "remote_off"], ["/package.json", "remote_off"],
      ["/@fs/C:/Windows/win.ini", "remote_off"], [encodeURI(`/@fs/${Path.join(Backend, "camera_calibration.json").replaceAll("\\", "/")}`), "remote_off"]]) {
      const Reply = await Get(Url);
      Assert.deepEqual([Reply.Status, JSON.parse(Reply.Text).access], [403, Code], Url);
    }
    // Paths Vite's router (connect) takes for its editor route or this config's routes, any case and by the raw path,
    // that the guard once read as the page's /src/main.tsx: Vite answered "launch-editor-middleware: required query
    // param "file" is missing" (its editor route, reached without the password) and the routes' own 405s (2026-10-08).
    for (const [Url, Code] of [["/__OPEN-IN-EDITOR/../src/main.tsx", "pc_only"], ["/__Open-In-Editor.x/../../src/main.tsx", "pc_only"],
      ["/BRIDGE-LAUNCH/restart/../../src/main.tsx", "remote_off"], ["/Bridge-Launch/ollama/../../src/main.tsx", "remote_off"],
      ["/SESSION-LOG/event/../../src/main.tsx", "remote_off"]]) {
      const Reply = await Get(Url);
      Assert.deepEqual([Reply.Status, AccessOf(Reply.Text)], [403, Code], Url);
    }
    // Another site's page on this PC, by another case of a route's name: refused before the route runs.
    for (const Url of ["/BRIDGE-LAUNCH/ollama", "/Session-Log/event"]) {
      const Reply = await Get(Url, ThisPc, "POST", { origin: "http://evil.example" });
      Assert.deepEqual([Reply.Status, AccessOf(Reply.Text)], [403, "cross_site"], Url);
    }
    console.log(`  another device loaded ${Urls.length} modules and assets without the password; the rest refused`);
  } finally {
    await Started.Close();
  }
});
