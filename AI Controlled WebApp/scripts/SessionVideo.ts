import Fs from "node:fs";
import Http from "node:http";
import Path from "node:path";

// The session video's switch. back end/SessionRecorder.py records the depth camera and the demo-area webcam only
// while an AI task runs (Aaran, 2026-10-08). The page (src/lib/SessionVideo.ts) posts what its task does to
// POST /session-log/recording: {"state": "start" | "alive" | "end", "task_id", "task"?, "reason"?}, "alive" every
// 15 s while the task runs. This passes it on to the recorder on this PC (POST /task on 127.0.0.1:8002), which takes
// it from programs on this PC only. The route sits behind this server's access rule like the rest of /session-log
// (scripts/RemoteAccess.ts): this PC as before, another device with the remote password, other sites' pages never.
// The page is the source because only it knows when a task starts and ends (every way a task ends goes through
// App.tsx's EndAiTask, which the task_end record already relies on); the bridge sees captures and motions, not
// tasks. The dev server carries it because the page may be on another device, and the recorder answers this PC alone.

const TaskStates = new Set(["start", "alive", "end"]);
const TaskKey = /^[A-Za-z0-9_.:-]{1,80}$/;
const MaxBodyBytes = 16384;
const RecorderTimeoutMs = 2000;

export type TaskSignalBody = { state: string; task_id: string; task?: string; reason?: string };
type RecorderAnswer = { ok: boolean; down?: boolean; [Key: string]: unknown };

// The version the running recorder must report (back end/RecorderVersion.json); another is replaced at launch.
export function RecorderVersion(BackendRoot: string): string | null {
  try {
    return String(JSON.parse(Fs.readFileSync(Path.join(BackendRoot, "RecorderVersion.json"), "utf8").replace(/^﻿/, "")).Version);
  } catch {
    return null;
  }
}

// The recorder's address, or null while recording is off (SO101_RECORDER=0: tests and a deliberate opt-out).
export function RecorderUrl(): string | null {
  if (process.env.SO101_RECORDER === "0") return null;
  return (process.env.SO101_RECORDER_URL || "http://127.0.0.1:8002").replace(/\/+$/, "");
}

// The fields the recorder takes from a page's body, or null when it is not a task signal.
export function TaskSignal(Body: unknown): TaskSignalBody | null {
  if (!Body || typeof Body !== "object" || Array.isArray(Body)) return null;
  const { state, task_id, task, reason } = Body as Record<string, unknown>;
  if (typeof state !== "string" || !TaskStates.has(state) || typeof task_id !== "string" || !TaskKey.test(task_id)) return null;
  return { state, task_id, ...(typeof task === "string" ? { task: task.slice(0, 500) } : {}),
    ...(typeof reason === "string" ? { reason: reason.slice(0, 300) } : {}) };
}

// POST one signal to the recorder; resolves its answer, or { ok: false, error, down } when it does not answer.
export function SendToRecorder(Url: string, Signal: TaskSignalBody): Promise<RecorderAnswer> {
  return new Promise(Resolve => {
    const Body = Buffer.from(JSON.stringify(Signal));
    let Target: URL;
    try { Target = new URL(`${Url}/task`); } catch { Resolve({ ok: false, error: "bad recorder address" }); return; }
    const Request = Http.request(Target, { method: "POST", timeout: RecorderTimeoutMs,
      headers: { "Content-Type": "application/json", "Content-Length": Body.length } }, Reply => {
      const Chunks: Buffer[] = [];
      Reply.on("data", (Chunk: Buffer) => Chunks.push(Chunk));
      Reply.on("end", () => {
        try { Resolve(JSON.parse(Buffer.concat(Chunks).toString("utf8")) as RecorderAnswer); }
        catch { Resolve({ ok: false, error: `the recorder answered ${Reply.statusCode}` }); }
      });
    });
    Request.on("timeout", () => Request.destroy(new Error("the recorder did not answer in time")));
    Request.on("error", (Failure: NodeJS.ErrnoException) => Resolve({ ok: false, error: Failure.message, down: Failure.code === "ECONNREFUSED" }));
    Request.end(Body);
  });
}

// POST /session-log/recording. OnRecorderDown runs when nothing answers on the recorder's port (vite.config.ts
// starts it then); its next "alive" starts the recording, so a task it first hears of mid-way is recorded from there.
export function TaskVideoHandler(OnRecorderDown: () => void = () => {}) {
  return (Request: any, Response: any) => {
    Response.setHeader("Content-Type", "application/json");
    if (Request.method !== "POST") { Response.statusCode = 405; Response.end(JSON.stringify({ ok: false, error: "Use POST." })); return; }
    const Chunks: Buffer[] = [];
    let Size = 0;
    Request.on("data", (Chunk: Buffer) => { Size += Chunk.length; if (Size <= MaxBodyBytes) Chunks.push(Chunk); });
    Request.on("end", async () => {
      let Body: unknown = null;
      try { Body = Size <= MaxBodyBytes ? JSON.parse(Buffer.concat(Chunks).toString("utf8")) : null; } catch { Body = null; }
      const Signal = TaskSignal(Body);
      if (!Signal) {
        Response.statusCode = 400;
        Response.end(JSON.stringify({ ok: false, error: 'Send {"state": "start" | "alive" | "end", "task_id": "<key>"}.' }));
        return;
      }
      const Url = RecorderUrl();
      if (!Url) { Response.end(JSON.stringify({ ok: false, recorder: "off" })); return; }
      const Answer = await SendToRecorder(Url, Signal);
      if (Answer.down) {
        try { OnRecorderDown(); } catch { /* Starting the recorder is best effort. */ }
      }
      if (!Answer.ok) Response.statusCode = 502;
      Response.end(JSON.stringify(Answer));
    });
  };
}
