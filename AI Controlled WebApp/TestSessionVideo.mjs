// The session video's switch, dev-server half (scripts/SessionVideo.ts): a page's task signals at
// POST /session-log/recording go to the recorder on this PC. A stub recorder on real sockets shows what is passed on;
// then the real back end/SessionRecorder.py (scratch captures folder, a free port, no camera, no bridge) shows it takes
// what the dev server sends. Never the live recorder on 8002: every recorder address here is checked first.
import { build as Build } from "esbuild";
import { test as Test, before as Before, after as After } from "node:test";
import Assert from "node:assert/strict";
import { spawn as Spawn } from "node:child_process";
import Fs from "node:fs";
import Http from "node:http";
import Net from "node:net";
import Os from "node:os";
import Path from "node:path";

const App = import.meta.dirname, Backend = Path.resolve(App, "../back end");
const Scratch = Fs.mkdtempSync(Path.join(Os.tmpdir(), "so101-session-video-"));
const LivePorts = new Set([5181, 8000, 8001, 8002, 11434]);
const Python = Fs.readFileSync(Path.join(App, "../.backend-python.txt"), "utf8").replace(/^﻿/, "").trim();
const Load = async File => {
  const Bundle = await Build({ entryPoints: [File], bundle: true, write: false, platform: "node", format: "esm" });
  return import(`data:text/javascript;base64,${Buffer.from(Bundle.outputFiles[0].text).toString("base64")}`);
};
const Video = await Load("scripts/SessionVideo.ts");

let Stub, StubPort, DevServer, DevPort, Downs = 0;
const Received = [];

function FreePort() {
  return new Promise(Resolve => {
    const Probe = Net.createServer().listen(0, "127.0.0.1", () => { const Port = Probe.address().port; Probe.close(() => Resolve(Port)); });
  });
}

// Point the dev server at a recorder address that is never a live service's.
function UseRecorder(Url) {
  Assert.ok(!LivePorts.has(Number(new URL(Url).port)), `refusing a live port: ${Url}`);
  process.env.SO101_RECORDER_URL = Url;
  delete process.env.SO101_RECORDER;
  Assert.equal(Video.RecorderUrl(), Url);
}

function Send(Body, Method = "POST") {
  return new Promise((Resolve, Reject) => {
    const Data = typeof Body === "string" ? Body : JSON.stringify(Body);
    const Sent = Http.request({ host: "127.0.0.1", port: DevPort, method: Method, path: "/session-log/recording",
      headers: { "Content-Type": "application/json", "Content-Length": Buffer.byteLength(Data) } }, Reply => {
      const Chunks = [];
      Reply.on("data", Chunk => Chunks.push(Chunk));
      Reply.on("end", () => Resolve({ Status: Reply.statusCode, Body: JSON.parse(Buffer.concat(Chunks).toString("utf8") || "{}") }));
    });
    Sent.on("error", Reject);
    Sent.end(Method === "POST" ? Data : undefined);
  });
}

async function Health(Port) {
  try { return await (await fetch(`http://127.0.0.1:${Port}/health`, { signal: AbortSignal.timeout(1500) })).json(); }
  catch { return null; }
}

Before(async () => {
  Stub = Http.createServer((Request, Response) => {
    let Body = "";
    Request.on("data", Chunk => { Body += Chunk; });
    Request.on("end", () => {
      Received.push({ Method: Request.method, Url: Request.url, Headers: Request.headers, Body: Body ? JSON.parse(Body) : null });
      Response.setHeader("Content-Type", "application/json");
      Response.end(JSON.stringify({ ok: true, accepted: true, recording: true }));
    });
  });
  await new Promise(Resolve => Stub.listen(0, "127.0.0.1", Resolve));
  StubPort = Stub.address().port;
  const Handler = Video.TaskVideoHandler(() => { Downs += 1; });
  DevServer = Http.createServer((Request, Response) => Handler(Request, Response));
  await new Promise(Resolve => DevServer.listen(0, "127.0.0.1", Resolve));
  DevPort = DevServer.address().port;
});

After(async () => {
  await new Promise(Resolve => Stub.close(Resolve));
  await new Promise(Resolve => DevServer.close(Resolve));
  Fs.rmSync(Scratch, { recursive: true, force: true });
});

Test("a task signal reaches the recorder with only the fields it takes, as a program on this PC", async () => {
  UseRecorder(`http://127.0.0.1:${StubPort}`);
  const Before_ = Received.length;
  const Reply = await Send({ state: "start", task_id: "k3x9-1", task: "grab the tin", extra: "dropped", reason: 5 });
  Assert.equal(Reply.Status, 200);
  Assert.deepEqual(Reply.Body, { ok: true, accepted: true, recording: true });
  const [Got] = Received.slice(Before_);
  Assert.equal(Got.Method, "POST");
  Assert.equal(Got.Url, "/task");
  Assert.deepEqual(Got.Body, { state: "start", task_id: "k3x9-1", task: "grab the tin" });
  Assert.equal(Got.Headers["content-type"], "application/json");
  Assert.equal(Got.Headers.host, `127.0.0.1:${StubPort}`, "the name the recorder accepts");
  Assert.equal(Got.Headers.origin, undefined, "no browser headers: the recorder refuses those");
  await Send({ state: "alive", task_id: "k3x9-1" });
  await Send({ state: "end", task_id: "k3x9-1", reason: "the AI reported the task done." });
  Assert.deepEqual(Received.slice(Before_).map(Item => Item.Body), [
    { state: "start", task_id: "k3x9-1", task: "grab the tin" }, { state: "alive", task_id: "k3x9-1" },
    { state: "end", task_id: "k3x9-1", reason: "the AI reported the task done." }]);
  Assert.equal((await Send({ state: "start", task_id: "long", task: "x".repeat(900) })).Status, 200);
  Assert.equal(Received.at(-1).Body.task.length, 500, "a long task is cut, not refused");
});

Test("anything else stops at the dev server", async () => {
  UseRecorder(`http://127.0.0.1:${StubPort}`);
  const Before_ = Received.length;
  Assert.equal((await Send("", "GET")).Status, 405);
  for (const Body of ["not json", "[1, 2]", { state: "begin", task_id: "a" }, { state: "start" }, { state: "start", task_id: "has space" },
    { state: "start", task_id: "y".repeat(81) }, { state: "start", task_id: "big", task: "z".repeat(20000) }]) {
    Assert.equal((await Send(Body)).Status, 400, JSON.stringify(Body).slice(0, 40));
  }
  Assert.equal(Received.length, Before_, "nothing reached the recorder");
});

Test("with the recorder off nothing is contacted; with it down the dev server starts it", async () => {
  UseRecorder(`http://127.0.0.1:${StubPort}`);
  process.env.SO101_RECORDER = "0";
  const Before_ = Received.length;
  const Off = await Send({ state: "start", task_id: "off-1" });
  Assert.deepEqual([Off.Status, Off.Body], [200, { ok: false, recorder: "off" }]);
  Assert.equal(Received.length, Before_);
  UseRecorder(`http://127.0.0.1:${await FreePort()}`);  // nothing listens there
  const Down = await Send({ state: "start", task_id: "down-1" });
  Assert.equal(Down.Status, 502);
  Assert.equal(Down.Body.down, true);
  Assert.equal(Downs, 1, "OnRecorderDown ran (vite.config.ts starts the recorder there)");
});

Test("the recorder version comes from back end/RecorderVersion.json", () => {
  const Version = JSON.parse(Fs.readFileSync(Path.join(Backend, "RecorderVersion.json"), "utf8")).Version;
  Assert.equal(Video.RecorderVersion(Backend), Version);
  Assert.equal(Video.RecorderVersion(Scratch), null);
});

Test("the real recorder takes what the dev server sends", { timeout: 60_000 }, async () => {
  const Port = await FreePort();
  Assert.ok(!LivePorts.has(Port));
  const Recorder = Spawn(Python, ["SessionRecorder.py", "--port", String(Port), "--root", Path.join(Scratch, "captures"), "--webcam", "",
    "--bridge", "http://127.0.0.1:9"], { cwd: Backend, stdio: "ignore", windowsHide: true,
    env: { ...process.env, SO101_CAPTURES_ROOT: Path.join(Scratch, "captures"), SO101_WEBCAM: "", PYTHONDONTWRITEBYTECODE: "1" } });
  try {
    let Ready = null;
    for (let Attempt = 0; Attempt < 100 && !Ready; Attempt++) {
      Ready = await Health(Port);
      if (!Ready) await new Promise(Resolve => setTimeout(Resolve, 200));
    }
    Assert.ok(Ready, "the recorder answered");
    Assert.equal(Ready.version, Video.RecorderVersion(Backend));
    Assert.equal(Ready.task.recording, false, "nothing records before a task");
    UseRecorder(`http://127.0.0.1:${Port}`);
    const Started = await Send({ state: "start", task_id: "real-1", task: "wave at the camera" });
    Assert.deepEqual([Started.Status, Started.Body.ok, Started.Body.accepted, Started.Body.recording], [200, true, true, true]);
    Assert.deepEqual((await Health(Port)).task.tasks.map(Item => [Item.task_id, Item.task]), [["real-1", "wave at the camera"]]);
    const Ended = await Send({ state: "end", task_id: "real-1", reason: "done" });
    Assert.equal(Ended.Body.accepted, true);
    const After_ = await Health(Port);
    Assert.deepEqual(After_.task.tasks, []);
    Assert.ok(After_.task.post_roll_left_s > 5, "recording on for the post-roll");
    const Events = Fs.readFileSync(Path.join(Scratch, "captures", (() => { const Now = new Date(); return `${Now.getFullYear()}-${String(Now.getMonth() + 1).padStart(2, "0")}-${String(Now.getDate()).padStart(2, "0")}`; })(),
      "recording", "recorder_events.jsonl"), "utf8").trim().split("\n").map(Line => JSON.parse(Line));
    Assert.deepEqual(Events.map(Event => Event.kind), ["task_start", "task_end"]);
    const Quit = await fetch(`http://127.0.0.1:${Port}/quit`, { method: "POST", headers: { "Content-Type": "application/json" }, body: "{}" });
    Assert.equal(Quit.status, 403, "Node's fetch sends Sec-Fetch-Mode, like a page: refused");
    await new Promise((Resolve, Reject) => {
      const Sent = Http.request({ host: "127.0.0.1", port: Port, method: "POST", path: "/quit", headers: { "Content-Type": "application/json", "Content-Length": 2 } },
        Reply => { Reply.resume(); Reply.on("end", Resolve); });
      Sent.on("error", Reject);
      Sent.end("{}");
    });
    await new Promise(Resolve => { if (Recorder.exitCode != null) Resolve(); else Recorder.on("exit", Resolve); });
    Assert.equal(Recorder.exitCode, 0);
  } finally {
    if (Recorder.exitCode == null) Recorder.kill();
  }
});
