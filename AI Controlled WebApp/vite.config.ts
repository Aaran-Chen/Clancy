import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";
import fs from "node:fs";
import http from "node:http";
import path from "node:path";
import { spawn, type ChildProcess } from "node:child_process";
import { fileURLToPath } from "node:url";
import { RunBridgeRestart } from "./scripts/BridgeRestart";
import { RemoteAccessMiddleware } from "./scripts/RemoteAccess";
import { AttachOllamaLogging, SessionEventHandler } from "./scripts/SessionLog";
import { RecorderUrl, RecorderVersion, TaskVideoHandler } from "./scripts/SessionVideo";
import { StartNimbleWarmer, WarmOptionsFrom } from "./scripts/NimbleWarmup";

// The browser reaches Ollama through this dev-server proxy to avoid CORS.
// Default to the Ollama instance on THIS machine. Override the target with the
// OLLAMA_BASE_URL environment variable to point at a different host.
const ollamaBaseUrl = process.env.OLLAMA_BASE_URL ?? "http://127.0.0.1:11434";

// The UniDepth depth server (see "back end/Depth Server.py") runs on the GPU PC.
// Proxy it here too so the browser avoids CORS. Override with DEPTH_BASE_URL.
// The depth server runs on THIS machine (the RTX 5090 rig), so default to
// localhost. The old 192.168.1.50 was a stale LAN address from another network
// and silently failed every depth request. Override with DEPTH_BASE_URL.
const depthBaseUrl = process.env.DEPTH_BASE_URL ?? "http://127.0.0.1:8001";

const appRoot = fileURLToPath(new URL(".", import.meta.url));
const repoRoot = path.resolve(appRoot, "..");
const backendRoot = path.join(repoRoot, "back end");
const bridgeBaseUrl = "http://127.0.0.1:8000";
const bridgeHealthUrl = `${bridgeBaseUrl}/health`;
const bridgeOut = path.join(appRoot, "api-output.log");
const bridgeErr = path.join(appRoot, "api-error.log");
// The session record: video while an AI task runs (back end/SessionRecorder.py, its own process, switched by
// scripts/SessionVideo.ts) and the model traffic and app events (scripts/SessionLog.ts), per day.
// SO101_CAPTURES_ROOT points tests at a scratch folder instead of the real record.
const capturesRoot = process.env.SO101_CAPTURES_ROOT ?? path.join(backendRoot, "captures");

let bridgeProcess: ChildProcess | null = null;
let bridgeStartPromise: Promise<{ ok: boolean; status: string }> | null = null;
let BridgeRestartPromise: Promise<void> | null = null;

function requestJson(url: string, timeoutMs = 1200): Promise<unknown> {
  return new Promise((resolve, reject) => {
    const request = http.get(url, { timeout: timeoutMs }, (response) => {
      const chunks: Buffer[] = [];
      response.on("data", (chunk: Buffer) => chunks.push(chunk));
      response.on("end", () => {
        try {
          resolve(JSON.parse(Buffer.concat(chunks).toString("utf8")));
        } catch (error) {
          reject(error);
        }
      });
    });
    request.on("timeout", () => {
      request.destroy(new Error("Request timed out"));
    });
    request.on("error", reject);
  });
}

async function isBridgeRunning() {
  try {
    const health = await requestJson(bridgeHealthUrl);
    return (
      typeof health === "object"
      && health !== null
      && "service" in health
      && health.service === "so101-backend-bridge"
    );
  } catch {
    return false;
  }
}

function sleep(ms: number) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

async function waitForBridge(process: ChildProcess) {
  // 100ms granularity over the same ~10s budget: the bridge answers /health
  // about 0.4s after spawn, so a coarser poll just idles on a live server.
  for (let i = 0; i < 100; i += 1) {
    if (await isBridgeRunning()) return true;
    if (process.exitCode != null) return false;
    await sleep(100);
  }
  return false;
}

function backendPython() {
  try {
    const cached = fs.readFileSync(path.join(repoRoot, ".backend-python.txt"), "utf8").replace(/^\uFEFF/, "").trim();
    if (cached) return cached;
  } catch {
    // No cached interpreter yet.
  }
  return "python";
}

// Size-capped logs with a few old ones kept, before a program opens them (back end/LogRotation.py; audit 2 M4:
// api-output.log reached 337 MB in 26 h). Best effort: a log that cannot be rotated is left as it is.
function rotateLogs(files: string[], capMb: number): Promise<void> {
  return new Promise((resolve) => {
    try {
      const child = spawn(backendPython(), ["LogRotation.py", ...files, "--cap-mb", String(capMb), "--keep", "3"], {
        cwd: backendRoot, stdio: "ignore", windowsHide: true,
      });
      const timer = setTimeout(() => { child.kill(); resolve(); }, 10_000);
      child.on("exit", () => { clearTimeout(timer); resolve(); });
      child.on("error", () => { clearTimeout(timer); resolve(); });
    } catch {
      resolve();
    }
  });
}

// Start the session video recorder unless the current one already answers. It is
// detached so it keeps recording across dev-server restarts, and a second copy
// exits by itself, so racing callers are harmless. One that reports another
// version (or none: before 2026-10-08 it recorded around the clock) is replaced:
// the new one, started with --replace-older, stops it first.
let recorderLaunchedAt = 0;

async function ensureRecorder() {
  const recorderUrl = RecorderUrl();
  if (!recorderUrl) return;  // SO101_RECORDER=0: tests and deliberate opt-out
  try {
    const health = await requestJson(`${recorderUrl}/health`);
    // With no readable RecorderVersion.json any recorder that answers stays: replacing it could not start a better one.
    const expected = RecorderVersion(backendRoot);
    if (typeof health === "object" && health !== null && "service" in health && health.service === "so101-recorder"
      && (expected === null || ("version" in health && health.version === expected))) return;
  } catch {
    // Not running.
  }
  // One launch at a time, and one every 30 s at most: its Python and camera take a few seconds to answer.
  if (Date.now() - recorderLaunchedAt < 30_000) return;
  recorderLaunchedAt = Date.now();
  try {
    fs.mkdirSync(capturesRoot, { recursive: true });
    const outLog = path.join(capturesRoot, "recorder-output.log");
    const errLog = path.join(capturesRoot, "recorder-error.log");
    await rotateLogs([outLog, errLog], 5);
    const stdout = fs.openSync(outLog, "a");
    const stderr = fs.openSync(errLog, "a");
    const port = new URL(recorderUrl).port || "8002";
    const child = spawn(backendPython(), ["SessionRecorder.py", "--replace-older", "--root", capturesRoot, "--port", port], {
      cwd: backendRoot, detached: true, stdio: ["ignore", stdout, stderr], windowsHide: true,
    });
    child.on("error", (error) => console.warn(`[recorder] could not start: ${error.message}`));
    child.unref();
    fs.closeSync(stdout);
    fs.closeSync(stderr);
  } catch (error) {
    console.warn(`[recorder] could not start: ${error instanceof Error ? error.message : error}`);
  }
}

// Ollama does not start by itself after a reboot, and the page found no model
// (and stayed "offline") whenever it loaded before Ollama did. Only a local
// Ollama is started; SO101_OLLAMA_AUTOSTART=0 opts out.
let ollamaStartPromise: Promise<{ ok: boolean; status: string }> | null = null;
let ollamaLaunchedAt = 0;

async function isOllamaRunning() {
  try {
    const version = await requestJson(`${ollamaBaseUrl}/api/version`);
    return typeof version === "object" && version !== null && "version" in version;
  } catch {
    return false;
  }
}

function ollamaIsLocal() {
  try {
    return ["127.0.0.1", "localhost", "[::1]", "::1"].includes(new URL(ollamaBaseUrl).hostname);
  } catch {
    return false;
  }
}

async function startOllama() {
  if (await isOllamaRunning()) return { ok: true, status: "already-running" };
  if (process.env.SO101_OLLAMA_AUTOSTART === "0") return { ok: false, status: "autostart-off" };
  if (!ollamaIsLocal()) return { ok: false, status: "remote-host" };
  // A cold start takes a few seconds; one launch a minute never stacks copies.
  if (Date.now() - ollamaLaunchedAt < 60_000) return { ok: false, status: "starting" };
  ollamaLaunchedAt = Date.now();
  // The CLI starts Ollama the way Ollama itself does (on Windows, its tray app
  // hidden, which runs the server), so ask it for the loaded models.
  const installed = process.platform === "win32" && process.env.LOCALAPPDATA
    ? path.join(process.env.LOCALAPPDATA, "Programs", "Ollama", "ollama.exe") : "";
  const cli = installed && fs.existsSync(installed) ? installed : "ollama";
  const launch = (args: string[]) => {
    try {
      const child = spawn(cli, args, { detached: true, stdio: "ignore", windowsHide: true });
      child.on("error", (error) => console.warn(`[ollama] could not start: ${error.message}`));
      child.unref();
      return child;
    } catch (error) {
      console.warn(`[ollama] could not start: ${error instanceof Error ? error.message : error}`);
      return null;
    }
  };
  const viaCli = launch(["ps"]);
  for (let i = 0; i < 100; i += 1) {
    if (await isOllamaRunning()) return { ok: true, status: "started" };
    // The CLI gave up (no tray app on this machine): run the server directly.
    if (i === 40 && viaCli?.exitCode != null) launch(["serve"]);
    await sleep(150);
  }
  return { ok: false, status: "starting" };
}

// Nimble (the page's prompt router) is warmed at dev-server start and whenever Ollama comes (back) up
// (scripts/NimbleWarmup.ts), so the page's first prompt does not meet it cold.
let nimbleWarmer: ReturnType<typeof StartNimbleWarmer> | null = null;

function ensureOllama() {
  ollamaStartPromise ??= startOllama().then((result) => {
    if (result.ok) void nimbleWarmer?.Check();
    return result;
  }).finally(() => { ollamaStartPromise = null; });
  return ollamaStartPromise;
}

function readOllamaOptions(): Record<string, unknown> {
  try {
    return JSON.parse(fs.readFileSync(path.join(backendRoot, "OllamaOptions.json"), "utf8").replace(/^\uFEFF/, ""));
  } catch {
    return {};
  }
}

async function startBridge() {
  if (await isBridgeRunning()) {
    return { ok: true, status: "already-running" };
  }

  // Same order as scripts/run-app.ps1: the interpreter that launcher resolved
  // (if it left a note), then "python" on PATH, then whatever py hands back.
  // Each miss costs a process spawn plus the wait below, so the likely winner
  // goes first.
  const candidates: Array<{ command: string; args: string[] }> = [];
  try {
    const cached = fs.readFileSync(path.join(repoRoot, ".backend-python.txt"), "utf8").trim();
    if (cached) candidates.push({ command: cached, args: [] });
  } catch {
    // No cached interpreter yet — fall through to the generic candidates.
  }
  candidates.push({ command: "python", args: [] }, { command: "py", args: ["-3"] });
  // No bridge holds its logs now: cap them before this one appends (the bridge also leaves the camera polls out).
  await rotateLogs([bridgeOut, bridgeErr], 20);

  let lastError = "";
  for (const candidate of candidates) {
    const stdout = fs.openSync(bridgeOut, "a");
    const stderr = fs.openSync(bridgeErr, "a");
    const child = spawn(candidate.command, [
      ...candidate.args,
      "-m",
      "uvicorn",
      "bridge_server:app",
      // Bind all interfaces so the web app also works from other devices (LAN, Tailscale): the bridge serves them
      // only with the remote password (back end/RemoteAccess.py).
      "--host",
      "0.0.0.0",
      "--port",
      "8000",
      // No proxy stands in front of the bridge: the client address its access rule sees is the real peer's (uvicorn's
      // default rewrote a loopback client's from X-Forwarded-For).
      "--no-proxy-headers",
    ], {
      cwd: backendRoot,
      env: { ...process.env, PYTHONPATH: backendRoot },
      stdio: ["ignore", stdout, stderr],
      windowsHide: true,
    });

    bridgeProcess = child;
    child.on("exit", () => {
      fs.closeSync(stdout);
      fs.closeSync(stderr);
      if (bridgeProcess === child) bridgeProcess = null;
    });
    child.on("error", (error) => {
      lastError = error.message;
    });

    if (await waitForBridge(child)) {
      return { ok: true, status: `started:${candidate.command} ${candidate.args.join(" ")}`.trim() };
    }

    try {
      child.kill();
    } catch {
      // Process may already have exited.
    }
  }

  return { ok: false, status: lastError || "failed-to-start" };
}

function bridgeLauncherPlugin() {
  return {
    name: "so101-bridge-launcher",
    configureServer(server) {
      void ensureRecorder();
      nimbleWarmer?.Stop();
      nimbleWarmer = StartNimbleWarmer(ollamaBaseUrl, WarmOptionsFrom(readOllamaOptions()), isOllamaRunning);
      void ensureOllama();
      // First, ahead of every route below and Vite's own proxies (/ollama, /depth-server): this PC as before, another
      // device only with the remote password, another site's page never (scripts/RemoteAccess.ts).
      server.middlewares.use(RemoteAccessMiddleware(backendRoot));
      server.middlewares.use("/session-log/event", SessionEventHandler(capturesRoot));
      // The page's AI task starts, is alive, ends: the recorder records only then (scripts/SessionVideo.ts).
      server.middlewares.use("/session-log/recording", TaskVideoHandler(() => void ensureRecorder()));
      // The page asks for this while it finds no model (Ollama down).
      server.middlewares.use("/bridge-launch/ollama", async (req, res) => {
        res.setHeader("Content-Type", "application/json");
        if (req.method !== "POST") {
          res.statusCode = 405; res.end(JSON.stringify({ ok: false, status: "Use POST to start Ollama." })); return;
        }
        res.end(JSON.stringify(await ensureOllama()));
      });
      server.middlewares.use("/bridge-launch/restart", async (Request, Response) => {
        Response.setHeader("Content-Type", "application/json");
        if (Request.method !== "POST") {
          Response.statusCode = 405; Response.end(JSON.stringify({ ok: false, error: "Use POST to request a bridge restart." })); return;
        }
        // Browser requests must originate from this dashboard, including on LAN.
        if (Request.headers.origin && Request.headers.origin !== `http://${Request.headers.host}`) {
          Response.statusCode = 403; Response.end(JSON.stringify({ ok: false, error: "Restart must be requested from this dashboard." })); return;
        }
        try {
          BridgeRestartPromise ??= (async () => {
            if (bridgeStartPromise) await bridgeStartPromise;
            await RunBridgeRestart(path.join(appRoot, "scripts", "RestartBridge.ps1"));
          })().finally(() => { BridgeRestartPromise = null; });
          await BridgeRestartPromise;
          Response.end(JSON.stringify({ ok: true, status: "updated-disconnected" }));
        } catch (Failure) {
          Response.statusCode = 409;
          Response.end(JSON.stringify({ ok: false, error: Failure instanceof Error ? Failure.message : "Bridge restart failed." }));
        }
      });
      server.middlewares.use("/bridge-launch/start", async (req, res) => {
        if (req.method !== "POST" && req.method !== "GET") {
          res.statusCode = 405;
          res.end("Method not allowed");
          return;
        }
        if (BridgeRestartPromise) {
          try { await BridgeRestartPromise; } catch { /* The restart endpoint reports its own failure. */ }
        }
        void ensureRecorder();
        bridgeStartPromise ??= startBridge().finally(() => {
          bridgeStartPromise = null;
        });
        const result = await bridgeStartPromise;
        res.setHeader("Content-Type", "application/json");
        res.end(JSON.stringify(result));
      });
    },
  };
}

export default defineConfig({
  plugins: [react(), bridgeLauncherPlugin()],
  server: {
    // Clancy's port, also for a bare `npm run dev:web` (Vite's default 5173 is no dashboard port: the bridge refuses
    // its pages, audit 2 L3). Taken, it fails instead of drifting to 5182 beside a running app.
    port: 5181,
    strictPort: true,
    // Vite's defaults plus the machine-local access files (the remote password's hash, the connect intent) and the
    // logs beside this file: never served, even though the app imports other files from back end and other devices
    // load the page from here.
    fs: { deny: [".env", ".env.*", "*.{crt,pem}", "**/.git/**", "RemotePassword.json*", "ConnectIntent.json*", "*.log"] },
    proxy: {
      "/ollama": {
        target: ollamaBaseUrl,
        changeOrigin: true,
        rewrite: (path) => path.replace(/^\/ollama/, ""),
        // Every model request and reply goes into the session record.
        configure: (proxy) => AttachOllamaLogging(proxy, capturesRoot),
      },
      "/depth-server": {
        target: depthBaseUrl,
        changeOrigin: true,
        rewrite: (path) => path.replace(/^\/depth-server/, ""),
      },
    },
  },
});
