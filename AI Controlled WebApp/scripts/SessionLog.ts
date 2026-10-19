import Crypto from "node:crypto";
import Fs from "node:fs";
import Path from "node:path";
import Zlib from "node:zlib";

// The session record's text half, beside the video that back end/SessionRecorder.py
// writes: back end/captures/YYYY-MM-DD/recording/
//   ai.jsonl        every model request and reply through the /ollama proxy
//                   (the page's warm-ups marked "preload": true), and the
//                   app's own events (prompts, proposals, decisions)
//   images/<sha1>   every image a request carried (color frames, depth maps),
//                   stored once and referenced as {"image_file": "images/..."}
// Logging never throws into a request: a failed write is reported and skipped.

export const AiLogName = "ai.jsonl";
const LoggedPaths = /^\/(v1\/systemone|v1\/chat\/completions|api\/chat|api\/generate)\b/;
const MaxBodyBytes = 32 * 1024 * 1024;
const MaxReplyChars = 1_000_000;

export function RecordingDirectory(CapturesRoot: string, When: Date): string {
  const Day = `${When.getFullYear()}-${String(When.getMonth() + 1).padStart(2, "0")}-${String(When.getDate()).padStart(2, "0")}`;
  return Path.join(CapturesRoot, Day, "recording");
}

// The file extension of bare image bytes, or null when they are not an image.
function ImageExtension(Bytes: Buffer): string | null {
  if (Bytes.length >= 3 && Bytes[0] === 0xff && Bytes[1] === 0xd8 && Bytes[2] === 0xff) return "jpg";
  if (Bytes.subarray(0, 4).toString("latin1") === "\x89PNG") return "png";
  if (Bytes.subarray(0, 4).toString("latin1") === "RIFF" && Bytes.subarray(8, 12).toString("latin1") === "WEBP") return "webp";
  if (Bytes.subarray(0, 4).toString("latin1") === "GIF8") return "gif";
  return null;
}

// Replace every image with a reference to a file named by its hash: data URLs
// (the /v1 image_url parts), and the bare base64 in a native /api/chat
// message's "images" list, which would otherwise go inline (~0.4 MB a call).
// Both spellings of one image become the same file.
export function StoreImages(Value: unknown, Directory: string, BareBase64 = false): unknown {
  if (typeof Value === "string") {
    const Match = /^data:image\/([a-z0-9.+-]+);base64,/i.exec(Value);
    if (!Match && !BareBase64) return Value;
    const Bytes = Buffer.from(Match ? Value.slice(Match[0].length) : Value, "base64");
    const Extension = Match ? (Match[1].toLowerCase() === "jpeg" ? "jpg" : Match[1].toLowerCase().replace(/[^a-z0-9]/g, "")) : ImageExtension(Bytes);
    if (!Extension) return Value;
    const Name = `${Crypto.createHash("sha1").update(Bytes).digest("hex")}.${Extension}`;
    const Target = Path.join(Directory, "images", Name);
    if (!Fs.existsSync(Target)) {
      Fs.mkdirSync(Path.dirname(Target), { recursive: true });
      // Write then rename, so the bridge storing the same image never sees half a file.
      const Temporary = `${Target}.${process.pid}.tmp`;
      Fs.writeFileSync(Temporary, Bytes);
      try { Fs.renameSync(Temporary, Target); } catch { Fs.rmSync(Temporary, { force: true }); }
    }
    return { image_file: `images/${Name}`, bytes: Bytes.length };
  }
  if (Array.isArray(Value)) return Value.map(Item => StoreImages(Item, Directory, BareBase64));
  if (Value && typeof Value === "object") {
    return Object.fromEntries(Object.entries(Value).map(([Key, Item]) => [Key, StoreImages(Item, Directory, Key === "images" && Array.isArray(Item))]));
  }
  return Value;
}

export function AppendRecord(CapturesRoot: string, Record: Record<string, unknown>, When = new Date()): void {
  try {
    const Directory = RecordingDirectory(CapturesRoot, When);
    Fs.mkdirSync(Directory, { recursive: true });
    // The server's clock stamps every line; a record cannot replace it.
    const Line = { ...(StoreImages(Record, Directory) as object), t: When.getTime() / 1000, time: When.toISOString() };
    Fs.appendFileSync(Path.join(Directory, AiLogName), JSON.stringify(Line) + "\n");
  } catch (Failure) {
    console.warn(`[session-log] could not record ${String(Record.kind)}: ${Failure instanceof Error ? Failure.message : Failure}`);
  }
}

function ParseBody(Buffer_: Buffer, Encoding: string | undefined): unknown {
  let Bytes = Buffer_;
  try {
    if (Encoding === "gzip") Bytes = Zlib.gunzipSync(Bytes);
    else if (Encoding === "deflate") Bytes = Zlib.inflateSync(Bytes);
    else if (Encoding === "br") Bytes = Zlib.brotliDecompressSync(Bytes);
  } catch { /* keep the raw bytes */ }
  const Text = Bytes.toString("utf8");
  try { return JSON.parse(Text); } catch { return Text.length > MaxReplyChars ? Text.slice(0, MaxReplyChars) + " [truncated]" : Text; }
}

type Emitter = { on(Event: string, Listener: (...Args: any[]) => void): unknown };
type ProxyServer = Emitter & { web?: (...Args: any[]) => unknown };
type Pending = { Id: string; Started: number; Chunks: Buffer[]; Size: number; Done?: boolean; Reply?: Pending };
const PendingKey = Symbol("SessionLog");

// Record model traffic at the dev server's /ollama proxy: exactly what the
// model saw and what it answered, whichever part of the app asked.
export function AttachOllamaLogging(Proxy: ProxyServer, CapturesRoot: string, NewId = () => Crypto.randomUUID()): void {
  const Collect = (Stream: Emitter, Into: Pending) => Stream.on("data", (Chunk: Buffer) => {
    Into.Size += Chunk.length;
    if (Into.Size <= MaxBodyBytes) Into.Chunks.push(Chunk);
  });
  // One record per request, from whichever ending comes first: the reply, a proxy
  // error, or the app cancelling it (switching to Manual, Disconnect, a newer prompt).
  const Finish = (Request: any, Entry: Pending, Fields: Record<string, unknown>) => {
    if (Entry.Done) return;
    Entry.Done = true;
    const Body = ParseBody(Buffer.concat(Entry.Chunks), undefined);
    // The page's warm-up asks nothing (no messages): Ollama only loads the model.
    const Preload = Boolean(Body) && typeof Body === "object" && Array.isArray((Body as { messages?: unknown }).messages)
      && (Body as { messages: unknown[] }).messages.length === 0;
    AppendRecord(CapturesRoot, {
      kind: "model", id: Entry.Id, path: String(Request.url), status: null, duration_ms: Date.now() - Entry.Started,
      request: Body, ...(Preload ? { preload: true } : {}),
      ...(Entry.Size > MaxBodyBytes ? { request_truncated_bytes: Entry.Size } : {}), ...Fields,
    });
  };
  // Tee the request body as the proxy starts: web() pipes the request
  // synchronously, so a listener added just before it sees every chunk the
  // proxy does. http-proxy's own 'proxyReq' event is no use here: it fires only
  // once a socket is assigned, and never for requests sent with
  // "Expect: 100-continue" (Windows PowerShell's default for POST).
  const Web = Proxy.web;
  if (typeof Web === "function") {
    Proxy.web = function (this: unknown, Request: any, ...Rest: any[]) {
      try {
        if (Request?.method === "POST" && LoggedPaths.test(String(Request.url ?? ""))) {
          const Entry: Pending = { Id: NewId(), Started: Date.now(), Chunks: [], Size: 0 };
          Request[PendingKey] = Entry;
          Collect(Request, Entry);
          // A response closed before it finished was cancelled by the client; the proxy
          // then drops the upstream reply, so neither 'end' nor 'error' would record it.
          const ClientResponse = Rest[0];
          ClientResponse?.on?.("close", () => {
            if (Entry.Done || ClientResponse.writableFinished) return;
            const Partial = Entry.Reply?.Chunks.length ? Buffer.concat(Entry.Reply.Chunks).toString("utf8").slice(0, MaxReplyChars) : undefined;
            Finish(Request, Entry, { aborted: true, error: "cancelled before the reply finished",
              ...(Partial ? { partial_reply: Partial } : {}) });
          });
        }
      } catch {
        // Recording is best effort; the request itself must go through.
      }
      return Web.call(this, Request, ...Rest);
    };
  }
  Proxy.on("proxyRes", (ProxyResponse: any, Request: any) => {
    const Entry: Pending | undefined = Request[PendingKey];
    if (!Entry) return;
    const Reply: Pending = { Id: Entry.Id, Started: Entry.Started, Chunks: [], Size: 0 };
    Entry.Reply = Reply;
    Collect(ProxyResponse, Reply);
    ProxyResponse.on("end", () => Finish(Request, Entry, { status: ProxyResponse.statusCode,
      reply: ParseBody(Buffer.concat(Reply.Chunks), ProxyResponse.headers?.["content-encoding"]) }));
  });
  Proxy.on("error", (Failure: Error, Request: any) => {
    const Entry: Pending | undefined = Request?.[PendingKey];
    if (Entry) Finish(Request, Entry, { error: Failure.message });
  });
}

// POST /session-log/event from the app: prompts, parsed replies, decisions, outcomes.
export function SessionEventHandler(CapturesRoot: string) {
  return (Request: any, Response: any) => {
    Response.setHeader("Content-Type", "application/json");
    if (Request.method !== "POST") { Response.statusCode = 405; Response.end(JSON.stringify({ ok: false })); return; }
    const Chunks: Buffer[] = [];
    let Size = 0;
    Request.on("data", (Chunk: Buffer) => { Size += Chunk.length; if (Size <= MaxBodyBytes) Chunks.push(Chunk); });
    Request.on("end", () => {
      let Body: unknown;
      try { Body = JSON.parse(Buffer.concat(Chunks).toString("utf8")); } catch { Body = null; }
      if (!Body || typeof Body !== "object" || Array.isArray(Body)) {
        Response.statusCode = 400; Response.end(JSON.stringify({ ok: false, error: "Send one JSON object." })); return;
      }
      // Every event is kind "app"; an event's own kind (a snap's "pitch", a bridge
      // fault's "protection_trip") is kept as event_kind instead of being lost.
      const Event = Body as Record<string, unknown>;
      AppendRecord(CapturesRoot, { ...Event, ...("kind" in Event ? { event_kind: Event.kind } : {}), kind: "app" });
      Response.end(JSON.stringify({ ok: true }));
    });
  };
}
