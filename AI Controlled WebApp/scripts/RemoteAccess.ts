import Crypto from "node:crypto";
import Fs from "node:fs";
import Path from "node:path";

// Who may use this dev server (vite.config.ts): the bridge's rule (back end/RemoteAccess.py) for the routes that reach
// past the page itself. Those are /bridge-launch/* (starts or restarts the bridge, starts Ollama), /session-log/*
// (writes the session record), /ollama/* (the GPU's models: chat, pull, delete) and /depth-server/*.
// Another device gets the page without the password, so it can load Clancy and type it: the page, its modules (src/,
// node_modules/, Vite's /@vite/ and /@react-refresh, the app's imports from back end/ as /@fs/) and the public
// assets (PageBootPath). Everything else this server would answer it gets only with the password: Vite's other files
// under this folder and /@fs/ elsewhere (2026-10-08 access review: the guard covered the four routes alone and passed
// every other path on to Vite). Vite's /__open-in-editor, which starts the editor on this PC with a file in it (a
// process start, through the shell on Windows), answers only this PC's own Clancy page and programs (PcOnlyRoutes).
// Routes are matched the way Vite's router (connect) matches its own, a case-insensitive prefix of the raw path, and
// also on the decoded, normalised path; the page's boot is only a path no reading can change (RoutePaths, BootTarget).
//   1. A browser request must be Clancy's own page, which this server serves: an Origin on the very host and port the
//      request names; with no Origin, Sec-Fetch-Site must be "same-origin" or "none" (another page's <img> or link
//      says "cross-site" or "same-site"). Programs send neither header.
//   2. This PC needs nothing: a loopback client asking for a loopback name, with no proxy header.
//   3. Any other device needs the remote password (back end/SetRemotePassword.py writes its salted scrypt hash to
//      back end/RemotePassword.json, git-ignored) in the X-Clancy-Password header, which the page adds to every
//      request of its own once that device's user typed it (src/lib/RemoteAccess.ts). No password set: refused.
//      Five different wrong passwords from one address in 10 minutes lock it out until they age out.
// The file is read again whenever it changes, so SetRemotePassword.py needs no restart here either.

export const PasswordHeader = "x-clancy-password";
// Prefixes, any case, no boundary: connect (vite.config.ts's server.middlewares.use(route) and Vite's /__open-in-editor)
// takes a route as a case-insensitive prefix of the raw path ending at "/", "." or its end, and Vite's proxy takes
// "/ollama" and "/depth-server" as plain prefixes. Matched on the raw path only as written, the guard let
// "/__OPEN-IN-EDITOR/../src/main.tsx" through as the page's /src/main.tsx while Vite routed it to its editor route,
// from another device without the password, and "/BRIDGE-LAUNCH/..." from another site's page on this PC without the
// Origin check (2026-10-08 Stage 5 verification).
export const GuardedRoutes = /^\/(bridge-launch|session-log|ollama|depth-server)/i;
export const PcOnlyRoutes = /^\/__open-in-editor/i;
// The page's own boot, by path (PageBootPath; GET and HEAD only).
const BootPrefixes = ["/src/", "/node_modules/", "/@vite/"];
const BootPaths = new Set(["/", "/index.html", "/@react-refresh", "/favicon.ico"]);
const ProxyHeaders = ["forwarded", "x-forwarded-for", "x-forwarded-host", "x-real-ip", "via", "tailscale-user-login"];
const FailureWindowMs = 600_000;
const FailureLimit = 5;
const MaximumCandidateLength = 256;
const RefusalNoteMs = 600_000;

export type AccessCode = "cross_site" | "pc_only" | "remote_off" | "password_required" | "password_wrong" | "locked_out";
export const Refusals: Record<AccessCode, [number, string]> = {
  cross_site: [403, "Refused: this request came from a web page that is not Clancy's own."],
  pc_only: [403, "Refused: this works only on the robot's PC itself."],
  remote_off: [403, "Remote access is off: only the robot's PC can use Clancy's server until a remote password is set there (back end/SetRemotePassword.py; README, Using Clancy from another device)."],
  password_required: [401, "This device needs the arm password set on the robot's PC: enter it in Clancy."],
  password_wrong: [401, "Wrong arm password."],
  locked_out: [429, "Too many wrong arm passwords from this device: wait 10 minutes, then try again."],
};

type PasswordRecord = { N: number; R: number; P: number; Salt: Buffer; Hash: Buffer };
const FileCache: { Key: string | null; Record: PasswordRecord | null } = { Key: null, Record: null };
const Verified = new Set<string>();
const Failures = new Map<string, Array<{ At: number; Attempt: string }>>();
const RefusalNotes = new Map<string, number>();

export function PasswordPath(BackendRoot: string): string {
  return process.env.SO101_REMOTE_PASSWORD_FILE || Path.join(BackendRoot, "RemotePassword.json");
}

// The lowercased host of a Host header or URL authority ("127.0.0.1:5181", "[::1]:5181"), without port or brackets.
export function HostName(Value: string | undefined): string | null {
  if (!Value) return null;
  try { return new URL(`http://${Value.trim()}`).hostname.replace(/^\[|\]$/g, "").toLowerCase(); }
  catch { return null; }
}

export function IsLoopbackName(Host: string | null | undefined): boolean {
  if (!Host) return false;
  return Host === "localhost" || Host === "::1" || /^127\.\d{1,3}\.\d{1,3}\.\d{1,3}$/.test(Host);
}

// A client address on this PC's loopback interface (Node gives IPv4 clients of a dual-stack socket as ::ffff:a.b.c.d).
export function IsLoopbackClient(Address: string | undefined): boolean {
  if (!Address) return false;
  const Plain = Address.toLowerCase().replace(/^::ffff:/, "");
  return Plain === "::1" || /^127\.\d{1,3}\.\d{1,3}\.\d{1,3}$/.test(Plain);
}

type Headers = Record<string, string | string[] | undefined>;
const Header = (Headers: Headers, Name: string) => {
  const Value = Headers[Name];
  return Array.isArray(Value) ? Value[0] : Value;
};

// Rule 1: null for a program, a request of Clancy's own page or an address typed into a browser, else why not.
export function OriginRefusal(Headers: Headers): string | null {
  const Origin = Header(Headers, "origin");
  if (Origin === undefined) {
    const Site = Header(Headers, "sec-fetch-site");
    if (Site === undefined || ["none", "same-origin"].includes(Site.trim().toLowerCase())) return null;
    return `a browser request from another page (${Site}) without an Origin`;
  }
  let Parsed: URL;
  try { Parsed = new URL(Origin.trim()); } catch { return `origin ${JSON.stringify(Origin)} cannot be read`; }
  if (!["http:", "https:"].includes(Parsed.protocol)) return `origin ${JSON.stringify(Origin)} is not a web page's`;
  if (Parsed.host.toLowerCase() !== String(Header(Headers, "host") ?? "").trim().toLowerCase()) {
    return `origin ${JSON.stringify(Origin)} is not this server's own page (${Header(Headers, "host")})`;
  }
  return null;
}

// Rule 2: this PC itself, asking by a loopback name, not through a proxy.
export function IsLocal(Client: string | undefined, Headers: Headers): boolean {
  return IsLoopbackClient(Client) && IsLoopbackName(HostName(Header(Headers, "host")))
    && !ProxyHeaders.some(Name => Headers[Name] !== undefined);
}

// The stored hash, read again whenever the file changes; null when none is set or it cannot be read.
export function LoadPassword(File: string): PasswordRecord | null {
  let Key: string;
  try { const Stat = Fs.statSync(File); Key = `${File}|${Stat.mtimeMs}|${Stat.size}`; } catch { return null; }
  if (FileCache.Key === Key) return FileCache.Record;
  let Record: PasswordRecord | null = null;
  try {
    const Raw = JSON.parse(Fs.readFileSync(File, "utf8"));
    if (Raw.Kdf !== "scrypt") throw new Error(`unknown Kdf ${Raw.Kdf}`);
    Record = { N: Number(Raw.N), R: Number(Raw.R), P: Number(Raw.P), Salt: Buffer.from(String(Raw.Salt), "base64"), Hash: Buffer.from(String(Raw.Hash), "base64") };
    if (!(Record.N >= 1024) || !(Record.R >= 1) || !(Record.P >= 1) || !Record.Hash.length) throw new Error("weak or empty hash");
  } catch (Failure) {
    console.warn(`[remote-access] ${File} is unreadable (${Failure instanceof Error ? Failure.message : Failure}): remote access stays off`);
    Record = null;
  }
  FileCache.Key = Key; FileCache.Record = Record;
  return Record;
}

function Scrypt(Candidate: string, Record: PasswordRecord): Promise<Buffer> {
  return new Promise((Resolve, Reject) => {
    Crypto.scrypt(Candidate, Record.Salt, Record.Hash.length,
      { N: Record.N, r: Record.R, p: Record.P, maxmem: Math.min(1024 ** 3, 256 * Record.N * Record.R + 32 * 1024 ** 2) },
      (Failure, Key) => Failure ? Reject(Failure) : Resolve(Key));
  });
}

// Whether Candidate is the remote password: [true, null] or [false, why]. A locked-out address is refused before
// anything is checked; an accepted password is remembered (a hash of it), so only a device's first request pays scrypt.
export async function CheckPassword(Candidate: string | undefined, Client: string, File: string): Promise<[boolean, AccessCode | null]> {
  const Record = LoadPassword(File);
  if (!Record) return [false, "remote_off"];
  if (!Candidate) return [false, "password_required"];
  const Trimmed = Candidate.slice(0, MaximumCandidateLength);
  const Attempt = Crypto.createHash("sha256").update(Buffer.concat([Record.Hash, Buffer.from([0]), Buffer.from(Trimmed, "utf8")])).digest("hex");
  const Now = Date.now();
  const Recent = (Failures.get(Client) ?? []).filter(Item => Now - Item.At <= FailureWindowMs);
  Failures.set(Client, Recent);
  if (new Set(Recent.map(Item => Item.Attempt)).size >= FailureLimit) return [false, "locked_out"];
  if (Verified.has(Attempt)) { Failures.delete(Client); return [true, null]; }
  let Ok = false;
  try {
    const Key = await Scrypt(Trimmed, Record);
    Ok = Key.length === Record.Hash.length && Crypto.timingSafeEqual(Key, Record.Hash);
  } catch (Failure) {
    console.warn(`[remote-access] could not check a password: ${Failure instanceof Error ? Failure.message : Failure}`);
  }
  if (Ok) { Verified.add(Attempt); Failures.delete(Client); return [true, null]; }
  (Failures.get(Client) ?? Failures.set(Client, []).get(Client)!).push({ At: Now, Attempt });
  return [false, "password_wrong"];
}

// Drop the accepted-password memory, every address's failures and the refusal notes (tests).
export function ForgetAttempts(): void {
  Verified.clear(); Failures.clear(); RefusalNotes.clear(); FileCache.Key = null; FileCache.Record = null;
}

// A refusal or a device let in, said once per address and reason per 10 minutes.
function Note(Kind: string, Client: string, Code: string, Detail: string) {
  const Key = `${Client}|${Code}`, Now = Date.now();
  if (Now - (RefusalNotes.get(Key) ?? -Infinity) < RefusalNoteMs) return;
  RefusalNotes.set(Key, Now);
  console.warn(`[remote-access] ${Kind} ${Client}: ${Code} (${Detail})`);
}

function Refuse(Response: any, Code: AccessCode, Detail?: string) {
  const [Status, Text] = Refusals[Code];
  Response.statusCode = Status;
  Response.setHeader("Content-Type", "application/json");
  Response.setHeader("Cache-Control", "no-store");
  Response.end(JSON.stringify({ ok: false, error: Text + (Detail ? ` (${Detail})` : ""), access: Code }));
}

// Whether File lies inside Folder (Folder itself excluded).
function Inside(File: string, Folder: string): boolean {
  const Relative = Path.relative(Path.resolve(Folder), Path.resolve(File));
  return Relative !== "" && !Relative.startsWith("..") && !Path.isAbsolute(Relative);
}

// A folder's real path (a git worktree's node_modules is a junction to the main checkout's), else the path itself.
function RealFolder(Folder: string): string {
  try { return Fs.realpathSync(Folder); } catch { return Path.resolve(Folder); }
}

const SamePath = (A: string, B: string) => process.platform === "win32" ? A.toLowerCase() === B.toLowerCase() : A === B;

// The back end/ JSON files the app imports (src/lib/*.ts: `import ... from "../../../back end/<name>.json"`), by name,
// read once per app folder: what another device's page loads from back end/ through Vite's /@fs/.
const BackendImports = new Map<string, Set<string>>();
export function AppBackendImports(AppRoot: string): Set<string> {
  const Known = BackendImports.get(AppRoot);
  if (Known) return Known;
  const Names = new Set<string>();
  const Walk = (Folder: string) => {
    let Entries: Fs.Dirent[] = [];
    try { Entries = Fs.readdirSync(Folder, { withFileTypes: true }); } catch { return; }
    for (const Entry of Entries) {
      const Full = Path.join(Folder, Entry.name);
      if (Entry.isDirectory()) Walk(Full);
      else if (/\.(ts|tsx)$/.test(Entry.name)) {
        for (const Match of Fs.readFileSync(Full, "utf8").matchAll(/["'](?:\.\.\/)+back end\/([A-Za-z0-9_.-]+\.json)["']/g)) Names.add(Match[1]);
      }
    }
  };
  Walk(Path.join(AppRoot, "src"));
  BackendImports.set(AppRoot, Names);
  return Names;
}

// Vite's /@fs/ URL as a file path (Vite's own fsPathFromId: "/@fs/C:/x" is C:/x, "/@fs/home/x" is /home/x).
function FsPath(Pathname: string): string {
  const File = Pathname.slice("/@fs/".length);
  return File.startsWith("/") || /^[A-Za-z]:/.test(File) ? File : `/${File}`;
}

// Whether another device's browser may have Pathname (decoded, no query) before its user typed the password: what
// loading Clancy needs. /@fs/ (Vite's files outside this folder) only for the back end/ JSON files the app imports and
// for node_modules (the fonts; outside this folder in a git worktree); a public asset only when it is there. No dot
// segment, backslash or NUL (a browser sends none). Vite still applies its own server.fs rules to whatever passes.
export function PageBootPath(Pathname: string, AppRoot: string, BackendRoot: string): boolean {
  if (/[\0\\]/.test(Pathname) || Pathname.split("/").some(Part => Part === "." || Part === "..")) return false;
  if (BootPaths.has(Pathname) || BootPrefixes.some(Prefix => Pathname.startsWith(Prefix))) return true;
  if (Pathname.startsWith("/@fs/")) {
    const File = Path.resolve(FsPath(Pathname));
    if (SamePath(Path.dirname(File), Path.resolve(BackendRoot)) && AppBackendImports(AppRoot).has(Path.basename(File))) return true;
    return Inside(File, RealFolder(Path.join(AppRoot, "node_modules")));
  }
  const Public = Path.join(AppRoot, "public");
  const File = Path.join(Public, Pathname);
  if (!Inside(File, Public)) return false;
  try { return Fs.statSync(File).isFile(); } catch { return false; }
}

const UrlBase = "http://clancy.invalid";

// Every reading of a request target's path a router might take: the raw path as connect reads it (parseurl: up to "?"
// or "#", an absolute-form target's own path), that with backslashes as slashes (Node's url.parse, which parseurl falls
// back to), and the decoded, normalised path (WHATWG URL: dot segments resolved). A route matches if any reading does.
export function RoutePaths(Url: string): string[] {
  const Target = Url.startsWith("/") ? Url : Url.replace(/^[A-Za-z][A-Za-z0-9+.-]*:\/\/[^/?#]*/, "");
  const Raw = /^[^?#]*/.exec(Target)![0] || "/";
  const Paths = [Raw, Raw.replace(/\\/g, "/")];
  try { Paths.push(decodeURIComponent(new URL(Url, UrlBase).pathname)); } catch { /* undecodable: the raw readings */ }
  return Paths;
}

// The decoded path of a target every reading agrees on (origin-form, already normalised, no encoded dot, slash,
// backslash or NUL, no fragment), else null: only such a target can be the page's boot (PageBootPath). A browser
// sends nothing else; "/__OPEN-IN-EDITOR/../src/main.tsx" normalised is "/src/main.tsx", but Vite routes it by its raw path.
export function BootTarget(Url: string): string | null {
  if (!Url.startsWith("/") || Url.includes("#")) return null;
  const Raw = /^[^?]*/.exec(Url)![0];
  if (/%(2e|2f|5c|00)/i.test(Raw)) return null;
  try {
    if (new URL(Raw, UrlBase).pathname !== Raw) return null;
    return decodeURIComponent(Raw);
  } catch { return null; }
}

// The connect middleware vite.config.ts puts in front of its routes and proxies, ahead of Vite's own middlewares, so it
// sees every request this server answers (WebSocket upgrades, Vite's HMR socket among them, never reach middlewares).
export function RemoteAccessMiddleware(BackendRoot: string, AppRoot = Path.resolve(BackendRoot, "..", "AI Controlled WebApp")) {
  return (Request: any, Response: any, Next: (Failure?: unknown) => void) => {
    const Url = String(Request.url ?? "");
    const Headers: Headers = Request.headers ?? {};
    const Client = String(Request.socket?.remoteAddress ?? "");
    // The password is for this rule alone: taken off the request here, so no route or proxy behind it ever passes it
    // on (the /ollama and /depth-server proxies sent it to their upstreams, which may be another host on the LAN).
    const Candidate = Header(Headers, PasswordHeader);
    delete Headers[PasswordHeader];
    const Paths = RoutePaths(Url), Boot = BootTarget(Url);
    const Where = `${Request.method} ${Paths[Paths.length - 1]}`;
    const Local = IsLocal(Client, Headers);
    const Guarded = Paths.some(Item => GuardedRoutes.test(Item));
    if (Paths.some(Item => PcOnlyRoutes.test(Item))) {
      const Problem = OriginRefusal(Headers);
      if (Problem) { Note("refused", Client, "cross_site", `${Where}: ${Problem}`); Refuse(Response, "cross_site", Problem); return; }
      if (!Local) { Note("refused", Client, "pc_only", Where); Refuse(Response, "pc_only"); return; }
      Next();
      return;
    }
    // The page itself: this PC as before; another device its boot without the password, everything else with it.
    if (!Guarded && (Local || (["GET", "HEAD"].includes(String(Request.method)) && Boot !== null
      && PageBootPath(Boot, AppRoot, BackendRoot)))) { Next(); return; }
    const Problem = OriginRefusal(Headers);
    if (Problem) { Note("refused", Client, "cross_site", `${Where}: ${Problem}`); Refuse(Response, "cross_site", Problem); return; }
    if (Local) { Next(); return; }
    void CheckPassword(Candidate, Client, PasswordPath(BackendRoot)).then(([Ok, Code]) => {
      if (!Ok) { Note("refused", Client, Code!, Where); Refuse(Response, Code!); return; }
      Note("let in", Client, "password", Where);
      Next();
    }, Failure => Next(Failure));
  };
}
