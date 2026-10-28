// The session record's dev-server half (scripts/SessionLog.ts) through the real vite.config.ts:
// a fake Ollama upstream, a scratch captures folder, and the recorder switched off. The
// last test reads that record back with "back end/ReviewRecording.py".
import { test as Test, after as After, before as Before } from "node:test";
import Assert from "node:assert/strict";
import { execFileSync as Execute } from "node:child_process";
import Fs from "node:fs";
import Http from "node:http";
import Os from "node:os";
import Path from "node:path";
import { ListenOnUiPort, BackendPython } from "./UiTestLib.mjs";

const Scratch = Fs.mkdtempSync(Path.join(Os.tmpdir(), "so101-session-log-"));
const Backend = Path.resolve(import.meta.dirname, "../back end");
let Upstream, UpstreamUrl, Server, Base;
const UpstreamHeaders = [];  // every request's headers as the upstream received them

function Day() {
  const Now = new Date();
  return `${Now.getFullYear()}-${String(Now.getMonth() + 1).padStart(2, "0")}-${String(Now.getDate()).padStart(2, "0")}`;
}
const RecordingPath = Name => Path.join(Scratch, Day(), "recording", Name);
const LogPath = () => RecordingPath("ai.jsonl");
const Records = () => Fs.existsSync(LogPath()) ? Fs.readFileSync(LogPath(), "utf8").trim().split("\n").filter(Boolean).map(Line => JSON.parse(Line)) : [];
const Pause = Milliseconds => new Promise(Resolve => setTimeout(Resolve, Milliseconds));
async function WaitForRecords(Count) {
  for (let Attempt = 0; Attempt < 50 && Records().length < Count; Attempt++) await Pause(40);
  return Records();
}
const Png = "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mP8z8BQDwAEhQGAhKmMIQAAAABJRU5ErkJggg==";
const Jpeg = Buffer.from([0xff, 0xd8, 0xff, 0xe0, 0, 16, 74, 70, 73, 70]).toString("base64");
// The app's request (Ollama's native /api/chat): the text, then each image as
// its own message with bare base64.
const Chat = (Text, Images = [Png]) => ({ model: "fake", stream: false, think: false, keep_alive: "30m",
  options: { num_ctx: 32768, num_batch: 1024, temperature: 0.2, num_predict: 1024 },
  messages: [{ role: "user", content: Text }, ...Images.map(Image => ({ role: "user", content: "", images: [Image] }))] });
// The OpenAI-compatible shape the app sent before 2026-09-29.
const V1Chat = (Text, Url = `data:image/png;base64,${Png}`) => ({ model: "fake", stream: false, messages: [{ role: "user", content: [{ type: "text", text: Text },
  { type: "image_url", image_url: { url: Url } }] }] });
const Post = (Route, Body, Init = {}) => fetch(`${Base}/ollama${Route}`, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(Body), ...Init });

Before(async () => {
  // Echoes the request's first text so every reply names its own request.
  Upstream = Http.createServer((Request, Response) => {
    UpstreamHeaders.push({ Url: Request.url, Headers: Request.headers });
    const Chunks = [];
    Request.on("data", Chunk => Chunks.push(Chunk));
    Request.on("end", () => {
      Response.setHeader("Content-Type", "application/json");
      if (Request.method === "GET") { Response.end(JSON.stringify({ models: [] })); return; }
      const Body = JSON.parse(Buffer.concat(Chunks).toString("utf8"));
      if (Request.url === "/v1/systemone") {
        Response.end(JSON.stringify({ model: Body.model, answers: { input_mode: { type: "choice", choice: "text",
          probabilities: { text: 0.98, camera: 0.01, camera_depth: 0.01 }, confidence: 0.9 } }, usage: { input_tokens: 128, output_tokens: 1 } }));
        return;
      }
      const Native = Request.url.startsWith("/api/");
      // A warm-up: Ollama loads the model and answers without generating.
      if (Native && Body.messages.length === 0) {
        Response.end(JSON.stringify({ model: Body.model, message: { role: "assistant", content: "" }, done: true, done_reason: "load" })); return;
      }
      const Text = Native ? Body.messages[0].content : Body.messages[0].content[0].text;
      const Reply = Content => JSON.stringify(Native ? { model: Body.model, message: { role: "assistant", content: Content }, done: true,
        done_reason: "stop", prompt_eval_count: 1100 + Text.length, eval_count: 7, load_duration: 5_000_000 } : { choices: [{ message: { content: Content } }] });
      if (Text === "fail") { Response.statusCode = 500; Response.end(JSON.stringify({ error: "upstream broke" })); return; }
      if (Text === "slow") { const Timer = setTimeout(() => Response.end(Reply("late")), 1500);
        Response.on("close", () => clearTimeout(Timer)); return; }
      setTimeout(() => Response.end(Reply(`echo:${Text}`)), 20 + Math.round(Text.length % 7) * 15);
    });
  });
  await new Promise(Resolve => Upstream.listen(0, "127.0.0.1", Resolve));
  UpstreamUrl = `http://127.0.0.1:${Upstream.address().port}`;
  // Nothing real starts: no recorder, no Ollama (its plugin would run `ollama ps` for an upstream that is not Ollama),
  // and the depth server is the same fake upstream (audit 2 L4: never a live port).
  Object.assign(process.env, { OLLAMA_BASE_URL: UpstreamUrl, SO101_CAPTURES_ROOT: Scratch, SO101_RECORDER: "0", SO101_OLLAMA_AUTOSTART: "0",
    DEPTH_BASE_URL: UpstreamUrl });
  const { createServer } = await import("vite");
  // The real vite.config.ts, loaded in memory ("runner": no node_modules/.vite-temp file), its cache in dist/ and a
  // port of 5191-5199: never the live app's node_modules/.vite (a worktree's node_modules is the main checkout's).
  ({ Server, Url: Base } = await ListenOnUiPort((Port, CacheDir) => createServer({ configFile: Path.join(import.meta.dirname, "vite.config.ts"),
    configLoader: "runner", cacheDir: CacheDir, logLevel: "silent", server: { port: Port, host: "127.0.0.1", strictPort: true } })));
});

After(async () => {
  await Server?.close();
  await new Promise(Resolve => Upstream?.close(Resolve));
  Fs.rmSync(Scratch, { recursive: true, force: true });
});

Test("a model request is recorded with its reply, and its images stored once beside the log", async () => {
  const Start = Records().length;
  for (let Repeat = 0; Repeat < 2; Repeat++) {
    const Reply = await Post("/api/chat", Chat("hello"));
    Assert.equal((await Reply.json()).message.content, "echo:hello");
  }
  const Last = (await WaitForRecords(Start + 2)).slice(-1)[0];
  Assert.equal(Last.kind, "model");
  Assert.equal(Last.path, "/api/chat");
  Assert.equal(Last.status, 200);
  Assert.equal(Last.request.messages[0].content, "hello");
  Assert.deepEqual(Last.request.options, { num_ctx: 32768, num_batch: 1024, temperature: 0.2, num_predict: 1024 });
  // The native request carries the image as bare base64: it is stored as a file too,
  // not written inline (about 0.4 MB per model call).
  const Image = Last.request.messages[1].images[0];
  Assert.match(Image.image_file, /^images\/[0-9a-f]{40}\.png$/);
  Assert.equal(Image.bytes, Buffer.from(Png, "base64").length);
  Assert.ok(!Fs.readFileSync(LogPath(), "utf8").includes(Png.slice(0, 40)), "no image is written inline");
  Assert.equal(Fs.readdirSync(RecordingPath("images")).length, 1);
  Assert.deepEqual(Fs.readFileSync(RecordingPath(Image.image_file)), Buffer.from(Png, "base64"));
  Assert.equal(Last.reply.message.content, "echo:hello");
  Assert.equal(Last.reply.prompt_eval_count, 1105);
  Assert.ok(typeof Last.t === "number" && Last.duration_ms >= 0);
});

Test("an image is one file whichever way it was sent: native bare base64, or a /v1 data URL", async () => {
  const Start = Records().length;
  await (await Post("/api/chat", Chat("native jpeg", [Jpeg]))).json();
  const Legacy = await (await Post("/v1/chat/completions", V1Chat("legacy jpeg", `data:image/jpeg;base64,${Jpeg}`))).json();
  Assert.equal(Legacy.choices[0].message.content, "echo:legacy jpeg");
  const [Native, V1] = (await WaitForRecords(Start + 2)).slice(Start).sort((A, B) => A.path.localeCompare(B.path));
  Assert.equal(Native.request.messages[1].images[0].image_file, V1.request.messages[0].content[1].image_url.url.image_file);
  Assert.match(Native.request.messages[1].images[0].image_file, /\.jpg$/);
  Assert.equal(V1.reply.choices[0].message.content, "echo:legacy jpeg");
  // A text field is never mistaken for an image.
  Assert.equal(Native.request.messages[0].content, "native jpeg");
});

Test("the page's warm-up request is recorded, marked as a preload", async () => {
  const Start = Records().length;
  const Reply = await (await Post("/api/chat", { model: "fake", messages: [], stream: false, keep_alive: "30m", options: { num_ctx: 32768, num_batch: 1024 } })).json();
  Assert.equal(Reply.done_reason, "load");
  const Last = (await WaitForRecords(Start + 1)).slice(-1)[0];
  Assert.equal(Last.preload, true);
  Assert.equal(Last.reply.done_reason, "load");
  Assert.equal((await WaitForRecords(Start + 1)).filter(Record => Record.preload).length, 1);
});

Test("Nimble's typed routing decision is recorded with its prompt, probabilities and latency", async () => {
  const Start = Records().length;
  const Body = { model: "nimble", state: "Explain inverse kinematics", keep_alive: "30m",
    questions: { input_mode: { type: "choice", instructions: "Which inputs are needed?",
      criteria: { text: "Conversation", camera: "Current camera scene", camera_depth: "Measured positions or action" } } } };
  const Reply = await (await Post("/v1/systemone", Body)).json();
  const Last = (await WaitForRecords(Start + 1)).slice(-1)[0];
  Assert.equal(Last.path, "/v1/systemone");
  Assert.equal(Last.kind, "model");
  Assert.equal(Last.status, 200);
  Assert.deepEqual(Last.request, Body);
  Assert.deepEqual(Last.reply, Reply);
  Assert.equal(Last.reply.answers.input_mode.choice, "text");
  Assert.equal(Last.reply.usage.output_tokens, 1);
  Assert.equal(Last.preload, undefined);
  Assert.ok(Number.isFinite(Last.duration_ms) && Last.duration_ms >= 0);
});

Test("requests sent with Expect: 100-continue are recorded too", async () => {
  const Start = Records().length;
  const Body = JSON.stringify(Chat("expect"));
  const Status = await new Promise((Resolve, Reject) => {
    const Request = Http.request(`${Base}/ollama/api/chat`, { method: "POST", headers: {
      "Content-Type": "application/json", "Content-Length": Buffer.byteLength(Body), Expect: "100-continue" } }, Response => {
      Response.resume(); Response.on("end", () => Resolve(Response.statusCode));
    });
    Request.on("continue", () => Request.end(Body));
    Request.on("error", Reject);
  });
  Assert.equal(Status, 200);
  const Last = (await WaitForRecords(Start + 1)).slice(-1)[0];
  Assert.equal(Last.request.messages[0].content, "expect");
  Assert.equal(Last.reply.message.content, "echo:expect");
  Assert.equal(Last.preload, undefined);
});

Test("concurrent requests each keep their own reply", async () => {
  const Start = Records().length;
  const Texts = ["alpha", "bravo-long", "c", "delta-x", "echo-yy"];
  await Promise.all(Texts.map(Text => Post("/api/chat", Chat(Text)).then(Reply => Reply.json())));
  const New = (await WaitForRecords(Start + Texts.length)).slice(Start);
  Assert.equal(New.length, Texts.length);
  for (const Record of New) Assert.equal(Record.reply.message.content, `echo:${Record.request.messages[0].content}`);
  Assert.equal(new Set(New.map(Record => Record.id)).size, Texts.length);
});

Test("a request the app cancels is still recorded, once, as aborted", async () => {
  const Start = Records().length;
  const Abort = new AbortController();
  const Pending = Post("/api/chat", Chat("slow"), { signal: Abort.signal }).catch(Failure => Failure.name);
  await Pause(200);
  Abort.abort();
  Assert.equal(await Pending, "AbortError");
  const Last = (await WaitForRecords(Start + 1)).slice(-1)[0];
  Assert.equal(Last.kind, "model");
  Assert.equal(Last.aborted, true);
  Assert.equal(Last.request.messages[0].content, "slow");
  await Pause(1700);  // past the upstream's delayed reply: still exactly one record
  Assert.equal(Records().length, Start + 1);
});

Test("reads such as the model list stay out of the record", async () => {
  const Start = Records().length;
  await (await fetch(`${Base}/ollama/api/tags`)).json();
  await Pause(150);
  Assert.equal(Records().length, Start);
});

Test("upstream failures are recorded with their status", async () => {
  const Start = Records().length;
  const Reply = await Post("/api/chat", Chat("fail"));
  Assert.equal(Reply.status, 500);
  const Last = (await WaitForRecords(Start + 1)).slice(-1)[0];
  Assert.equal(Last.status, 500);
  Assert.equal(Last.reply.error, "upstream broke");
});

Test("app events are recorded; kind and time cannot be overridden; bad bodies are refused", async () => {
  const Start = Records().length;
  const Posted = await fetch(`${Base}/session-log/event`, { method: "POST", headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ type: "prompt", prompt: "Grab the eraser", kind: "model", t: 1 }) });
  Assert.deepEqual(await Posted.json(), { ok: true });
  const Last = (await WaitForRecords(Start + 1)).slice(-1)[0];
  Assert.equal(Last.kind, "app");
  Assert.equal(Last.type, "prompt");
  // An event's own "kind" (a snap's "pitch", a bridge fault's "motion_fault") is kept beside it.
  Assert.equal(Last.event_kind, "model");
  Assert.ok(Last.t > 1_000_000_000);
  Assert.equal((await fetch(`${Base}/session-log/event`, { method: "POST", body: "not json" })).status, 400);
  Assert.equal((await fetch(`${Base}/session-log/event`, { method: "POST", body: "[1,2]" })).status, 400);
  Assert.equal((await fetch(`${Base}/session-log/event`)).status, 405);
  Assert.equal(Records().length, Start + 1);
});

Test("ReviewRecording.py reads the record back: native and /v1 calls, warm-ups, task events, bridge events, grounding", async () => {
  const Event = Body => fetch(`${Base}/session-log/event`, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(Body) });
  await Post("/api/chat", Chat("review native"));
  await Post("/v1/chat/completions", V1Chat("review legacy"));
  await Event({ type: "task_object_bound", label: "black thing", measured_label: "USB drive", source: "task_query", distance_cm: 0.4 });
  await Event({ type: "snap", requested: { x: 5.2, y: 34.7, z: 4.2 }, requested_wrist_pitch_degrees: -45, kind: "pitch", distance_cm: 0,
    pitch_change_deg: 1.9, reason: "the wrist cannot reach -45 deg there", snapped: { x: 5.2, y: 34.7, z: 4.2, wrist_pitch_degrees: -43.1 } });
  await Event({ type: "bridge_refused", reason: "IK target is unreachable at this wrist angle", motion: { target_cm: { x: 5.2, y: 34.7, z: 4.2 },
    wrist_pitch_degrees: -43.1, servo_targets: [] }, task: "Grab the black thing", step: 2 });
  await Event({ type: "task_end", reason: "the bridge reports a motion fault: shoulder_lift stalled.", source: "motion_fault", task: "Grab the black thing", step: 2 });
  await Event({ type: "bridge_fault", kind: "protection_trip", reason: "wrist_flex overload", joint: "wrist_flex", details: null });
  // The router's decision as App.tsx records it (lib/AiPromptRouter.ts RouteRecord), and one from before 2026-10-10.
  await Event({ type: "prompt_route", route: "camera", mode: "camera", source: "nimble", reason: "Nimble: the camera image answers it, no measurements needed",
    floor: "text", nimble_choice: "camera", probabilities: { text: 0.02, camera: 0.96, camera_depth: 0.02 }, duration_ms: 212, router_model: "nimble",
    sends: { image: true, measured_positions: false, depth_image: false, task: false } });
  await Event({ type: "prompt_route", route: "camera_depth", mode: "camera_depth", source: "safety", reason: "Acts on an object: fresh camera and measured positions",
    floor: "camera_depth", nimble_choice: null, probabilities: null, duration_ms: 0, router_model: "nimble",
    sends: { image: true, measured_positions: true, depth_image: true, task: true } });
  await Event({ type: "prompt_route", mode: "vision", source: "fallback", reason: "Nimble needs Ollama 0.35 or newer; using camera context", duration_ms: 3, router_model: "nimble" });
  const Now = Date.now() / 1000;
  const Lines = (Name, Rows) => Fs.appendFileSync(RecordingPath(Name), Rows.map(Row => JSON.stringify(Row) + "\n").join(""));
  Lines("bridge_events.jsonl", [{ kind: "motion_fault", reason: "shoulder_lift stalled", joint: "shoulder_lift",
    details: { present_raw: 2011, target_raw: 2150 }, t: Now, time: new Date().toISOString() }]);
  Lines("grounding.jsonl", [
    { kind: "grounding", model: "fake", labels: null, image_size: [1280, 720], reply: "[{\"label\":\"eraser\",\"box\":[1,2,3,4]}]", result: [{ label: "eraser" }],
      endpoint: "/api/chat", done_reason: "stop", prompt_eval_count: 1103, eval_count: 31, load_ms: 16360, t: Now },
    { kind: "grounding", model: "fake", labels: null, image_size: [1280, 720], reply: "[]", result: [], t: Now }]);
  await WaitForRecords(Records().length);
  await Pause(200);
  const Python = BackendPython();
  const Timeline = Execute(Python, ["ReviewRecording.py", "timeline", "--date", Day(), "--full"],
    { cwd: Backend, encoding: "utf8", env: { ...process.env, SO101_CAPTURES_ROOT: Scratch, PYTHONIOENCODING: "utf-8" } });
  const Line = (Kind, Pattern) => Assert.match(Timeline, new RegExp(`${Kind}\\s+${Pattern}`), `${Kind} ${Pattern}\n${Timeline}`);
  Line("MODEL ASK", "fake, 2 messages, images \\['images/[0-9a-f]{40}\\.png'\\]: review native");
  Line("MODEL SAID", "\\(\\d+ ms, 1113 in / 7 out\\) echo:review native");
  Line("MODEL ASK", "fake, 1 messages, images \\['images/[0-9a-f]{40}\\.png'\\]: review legacy");
  Line("MODEL SAID", "\\(\\d+ ms\\) echo:review legacy");
  Line("MODEL LOAD", "fake \\(\\d+ ms\\): load");
  Line("ROUTER ASK", "nimble: Explain inverse kinematics");
  Line("ROUTER SAID", "\\(\\d+ ms, 128 in / 1 out\\)");
  Line("PROMPT_ROUTE", "camera \\(nimble, 212 ms\\): Nimble: the camera image answers it, no measurements needed \\| sends prompt \\+ image \\| Nimble camera: text 0.02, camera 0.96, camera_depth 0.02");
  Line("PROMPT_ROUTE", "camera_depth \\(safety, 0 ms\\): Acts on an object: fresh camera and measured positions \\| sends prompt \\+ image \\+ /locate \\+ depth map \\+ task\\r?\\n");
  Line("PROMPT_ROUTE", "vision \\(fallback, 3 ms\\): Nimble needs Ollama 0.35 or newer; using camera context\\r?\\n");
  Line("TASK_OBJECT_BOUND", "\"black thing\" is the measured \"USB drive\" \\(task_query, 0.4 cm\\)");
  Line("SNAP", "pitch: X 5.2 Y 34.7 Z 4.2, wrist -45.0° -> X 5.2 Y 34.7 Z 4.2, wrist -43.1° \\(0.0 cm, 1.9°\\): the wrist cannot reach -45 deg there");
  Line("BRIDGE_REFUSED", "IK target is unreachable at this wrist angle \\| target X 5.2 Y 34.7 Z 4.2, wrist -43.1° \\(task step 2\\)");
  Line("TASK_END", "motion_fault: the bridge reports a motion fault: shoulder_lift stalled. \\(task step 2\\)");
  Line("BRIDGE FAULT", "motion_fault: shoulder_lift stalled \\(shoulder_lift\\) \\| \\{\"present_raw\": 2011, \"target_raw\": 2150\\}");
  Line("BRIDGE_FAULT", "protection_trip: wrist_flex overload \\(wrist_flex\\)");
  Line("GROUNDING", "fake \\[1280, 720\\]: 1 box\\(es\\): eraser \\| 1103 in / 31 out, load 16.4 s");
  Line("GROUNDING", "fake \\[1280, 720\\]: 0 box\\(es\\): ");
  // A warm-up asks nothing: it is one MODEL LOAD line, not an empty question and answer.
  Assert.doesNotMatch(Timeline, /MODEL ASK\s+fake, 0 messages/);
});

Test("the arm password never reaches an upstream: Ollama's or the depth server's (2026-10-08 access review)", async () => {
  // The page sends X-Clancy-Password with each request to the dev server's routes; the access rule takes it off the
  // request, so neither proxy forwards it (DEPTH_BASE_URL may be another host on the LAN).
  const Start = UpstreamHeaders.length;
  const Key = { "Content-Type": "application/json", "X-Clancy-Password": "not for any upstream" };
  Assert.equal((await fetch(`${Base}/ollama/api/chat`, { method: "POST", headers: Key, body: JSON.stringify(Chat("password")) })).status, 200);
  Assert.equal((await fetch(`${Base}/ollama/api/tags`, { headers: Key })).status, 200);
  Assert.equal((await fetch(`${Base}/depth-server/health`, { headers: Key })).status, 200);
  const Seen = UpstreamHeaders.slice(Start);
  Assert.deepEqual(Seen.map(Item => Item.Url).sort(), ["/api/chat", "/api/tags", "/health"]);
  for (const Item of Seen) Assert.equal(Item.Headers["x-clancy-password"], undefined, Item.Url);
});
