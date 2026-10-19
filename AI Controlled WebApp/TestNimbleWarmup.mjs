// The dev server's Nimble warm-up (scripts/NimbleWarmup.ts, started by vite.config.ts): at dev-server start and
// whenever Ollama comes back up, within the same checks as the page (src/lib/NimbleStatus.ts). Ollama is a mock HTTP
// server on a free port here; the real one (11434) is never contacted.
import { build as Build } from "esbuild";
import { test as Test } from "node:test";
import Assert from "node:assert/strict";
import Fs from "node:fs";
import Http from "node:http";
import Os from "node:os";
import Path from "node:path";

const App = import.meta.dirname;
const Load = async File => {
  const Bundle = await Build({ entryPoints: [Path.join(App, File)], bundle: true, write: false, platform: "node", format: "esm" });
  return import(`data:text/javascript;base64,${Buffer.from(Bundle.outputFiles[0].text).toString("base64")}`);
};
const Warmup = await Load("scripts/NimbleWarmup.ts");
const Status = await Load("src/lib/NimbleStatus.ts");
const Gb = 1e9;

/** A mock Ollama: Up switches it on and off; Requests lists what reached it. */
async function MockOllama({ Version = "0.35.0", Installed = [{ name: "nimble:latest", size: 9.5 * Gb }], Loaded = [] } = {}) {
  const Mock = { Up: true, Requests: [], Loaded };
  const Server = Http.createServer((Request, Response) => {
    let Body = "";
    Request.on("data", Chunk => { Body += Chunk; });
    Request.on("end", () => {
      if (!Mock.Up) { Response.statusCode = 503; Response.end("{}"); return; }
      Mock.Requests.push({ Method: Request.method, Path: Request.url, Body: Body ? JSON.parse(Body) : null });
      Response.setHeader("Content-Type", "application/json");
      if (Request.url === "/api/version") return Response.end(JSON.stringify({ version: Version }));
      if (Request.url === "/api/tags") return Response.end(JSON.stringify({ models: Installed }));
      if (Request.url === "/api/ps") return Response.end(JSON.stringify({ models: Mock.Loaded }));
      if (Request.url === "/v1/systemone") return Response.end(JSON.stringify({ answers: { input_mode: { type: "choice", choice: "text" } } }));
      Response.statusCode = 404; Response.end("{}");
    });
  });
  await new Promise(Resolve => Server.listen(0, "127.0.0.1", Resolve));
  Mock.Url = `http://127.0.0.1:${Server.address().port}`;
  Mock.Close = () => new Promise(Resolve => Server.close(Resolve));
  Mock.Warms = () => Mock.Requests.filter(Request => Request.Path === "/v1/systemone");
  return Mock;
}
const Options = (Extra = {}) => ({ Enabled: true, Model: "nimble", KeepAlive: "30m", BudgetGb: 29, TimeoutMs: 2000, ...Extra });
const Until = async (Check, Label) => {
  for (let Index = 0; Index < 200; Index++) { if (await Check()) return; await new Promise(Resolve => setTimeout(Resolve, 10)); }
  Assert.fail(Label);
};

Test("the options: router_startup_keep_alive (30m) for the dev server, off with SO101_ROUTER_WARMUP=0 or router_enabled false", () => {
  const Saved = JSON.parse(Fs.readFileSync(Path.join(App, "../back end/OllamaOptions.json"), "utf8"));
  const Read = Warmup.WarmOptionsFrom(Saved);
  Assert.deepEqual(Read, { Enabled: process.env.SO101_ROUTER_WARMUP !== "0", Model: "nimble", KeepAlive: "30m", BudgetGb: 29, TimeoutMs: 60000 });
  Assert.equal(Warmup.WarmOptionsFrom({ ...Saved, router_enabled: false }).Enabled, false);
  const Before = process.env.SO101_ROUTER_WARMUP;
  process.env.SO101_ROUTER_WARMUP = "0";
  try { Assert.equal(Warmup.WarmOptionsFrom(Saved).Enabled, false); } finally {
    if (Before === undefined) delete process.env.SO101_ROUTER_WARMUP; else process.env.SO101_ROUTER_WARMUP = Before;
  }
});

Test("one warm-up loads Nimble with the startup keep_alive, and skips it when loaded, missing, too old or over budget", async () => {
  const Mock = await MockOllama();
  try {
    Assert.deepEqual(await Warmup.WarmNimble(Mock.Url, Options()), { ok: true, status: "warmed" });
    Assert.equal(Mock.Warms().length, 1);
    Assert.equal(Mock.Warms()[0].Body.model, "nimble"); Assert.equal(Mock.Warms()[0].Body.keep_alive, "30m");
    Assert.equal(Mock.Warms()[0].Body.questions.input_mode.type, "choice");
    Mock.Loaded = [{ name: "nimble:latest", size_vram: 9 * Gb }];
    Assert.deepEqual(await Warmup.WarmNimble(Mock.Url, Options()), { ok: true, status: "already-loaded" });
    Mock.Loaded = [{ name: "qwen3.8:27b", size_vram: 17.5 * Gb }, { name: "qwen2.5:7b", size_vram: 5 * Gb }];
    Assert.match((await Warmup.WarmNimble(Mock.Url, Options())).status, /would not fit beside the loaded models \(22\.5 GB\)/);
    Assert.equal(Mock.Warms().length, 1, "never loads it where Ollama would evict the main model");
    Assert.deepEqual(await Warmup.WarmNimble(Mock.Url, Options({ Enabled: false })), { ok: false, status: "off" });
    Mock.Up = false;
    Assert.equal((await Warmup.WarmNimble(Mock.Url, Options())).ok, false, "Ollama down: no throw");
  } finally { await Mock.Close(); }
  for (const [Setup, Reason] of [[{ Version: "0.34.1" }, /0.35/], [{ Installed: [] }, /Install nimble/]]) {
    const Other = await MockOllama(Setup);
    try {
      const Result = await Warmup.WarmNimble(Other.Url, Options());
      Assert.equal(Result.ok, false); Assert.match(Result.status, Reason); Assert.equal(Other.Warms().length, 0);
    } finally { await Other.Close(); }
  }
});

Test("the dev server's check gives the page's verdicts (src/lib/NimbleStatus.ts) on the same Ollama answers", () => {
  const Cases = [
    [{ version: "0.35.0" }, { models: [{ name: "nimble:latest", size: 9.5 * Gb }] }, { models: [] }],
    [{ version: "0.34.9" }, { models: [{ name: "nimble" }] }, { models: [] }],
    [{ version: "1.2.0" }, { models: [{ name: "nimble" }] }, { models: [{ name: "nimble", size_vram: 9 * Gb }] }],
    [{ version: "0.35.0" }, { models: [] }, { models: [] }],
    [{ version: "0.36.0" }, { models: [{ name: "nimble:latest", size: 9.5 * Gb }] }, { models: [{ name: "a", size_vram: 17.5 * Gb }, { name: "b", size_vram: 5 * Gb }] }],
    [{ version: "0.36.0" }, { models: [{ name: "nimble:latest", size: 9.5 * Gb }] }, { models: [{ name: "a", size_vram: 17.5 * Gb }] }],
    [{}, null, null],
  ];
  for (const [Version, Tags, Ps] of Cases) for (const Budget of [29, 0]) {
    const Page = Status.ReadNimbleStatus(Version, Tags, Ps, "nimble", Budget), Server = Warmup.ServerNimbleStatus(Version, Tags, Ps, "nimble", Budget);
    Assert.deepEqual([Server.Ready, Server.Reason, Server.Loaded], [Page.Ready, Page.Reason, Page.Loaded], JSON.stringify([Version, Ps, Budget]));
  }
});

Test("the warmer warms at start, and again only when Ollama comes back up", async () => {
  const Mock = await MockOllama(), Lines = [];
  const IsRunning = async () => Mock.Up;
  const Warmer = Warmup.StartNimbleWarmer(Mock.Url, Options(), IsRunning, { IntervalMs: 20, Log: Line => Lines.push(Line) });
  try {
    await Until(() => Mock.Warms().length === 1, "warmed at dev-server start");
    await Until(() => Lines.length === 1, "logged");
    Assert.match(Lines[0], /^\[router\] dev server start: nimble warmed$/);
    await new Promise(Resolve => setTimeout(Resolve, 80));
    Assert.equal(Mock.Warms().length, 1, "Ollama stayed up: no repeat");
    Mock.Up = false;
    await new Promise(Resolve => setTimeout(Resolve, 60));
    Mock.Up = true;
    await Until(() => Mock.Warms().length === 2, "warmed again after Ollama came back");
    await Until(() => Lines.length === 2, "logged again");
    Assert.match(Lines[1], /Ollama is up again: nimble warmed/);
  } finally { Warmer.Stop(); await Mock.Close(); }
  // Off: nothing is checked or sent.
  let Checked = 0;
  const Off = Warmup.StartNimbleWarmer("http://127.0.0.1:9", Options({ Enabled: false }), async () => { Checked++; return true; }, { IntervalMs: 5 });
  await new Promise(Resolve => setTimeout(Resolve, 30));
  Off.Stop(); Assert.equal(Checked, 0);
});

Test("vite.config.ts: starting the dev server warms Nimble on the configured Ollama", async () => {
  const Mock = await MockOllama();
  const Scratch = Fs.mkdtempSync(Path.join(Os.tmpdir(), "so101-nimble-warm-"));
  const Saved = { ...process.env };
  Object.assign(process.env, { OLLAMA_BASE_URL: Mock.Url, SO101_OLLAMA_AUTOSTART: "0", SO101_RECORDER: "0", DEPTH_BASE_URL: "http://127.0.0.1:9",
    SO101_CAPTURES_ROOT: Path.join(Scratch, "captures"), SO101_REMOTE_PASSWORD_FILE: Path.join(Scratch, "RemotePassword.json") });
  delete process.env.SO101_ROUTER_WARMUP;
  try {
    const { resolveConfig: ResolveConfig } = await import("vite");
    const Config = await ResolveConfig({ root: App, configFile: Path.join(App, "vite.config.ts"), configLoader: "runner",
      cacheDir: Path.join(Scratch, "vite-cache"), logLevel: "silent" }, "serve");
    const Plugin = Config.plugins.find(Item => Item.name === "so101-bridge-launcher");
    await Plugin.configureServer({ middlewares: { use: () => {} } });
    await Until(() => Mock.Warms().length === 1, "the dev server warmed Nimble");
    Assert.equal(Mock.Warms()[0].Body.keep_alive, "30m", "the startup keep_alive");
    Assert.ok(Mock.Requests.every(Request => Request.Method === "GET" || Request.Path === "/v1/systemone"), "only checks and the one warm-up");
  } finally {
    for (const Key of Object.keys(process.env)) if (!(Key in Saved)) delete process.env[Key];
    Object.assign(process.env, Saved);
    await Mock.Close();
    Fs.rmSync(Scratch, { recursive: true, force: true });
  }
});
