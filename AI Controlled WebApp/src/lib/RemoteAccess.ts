// Who this page is to the robot's PC (back end/RemoteAccess.py for the bridge, scripts/RemoteAccess.ts for this dev
// server; Aaran's decision 2, 2026-10-07 audit 2 H2 and M2).
//   This PC: the page was opened by a loopback name (http://127.0.0.1:5181, http://localhost:5181). Nothing changes:
//   no password, the arm connects by itself when the bridge comes up (unless someone disconnected it on purpose).
//   Another device (a phone or laptop on the LAN or the tailnet): the user types the arm password once; this browser
//   keeps it (localStorage) and sends it with every request to the robot's PC, as the X-Clancy-Password header on HTTP
//   (InstallPasswordFetch) and as the first message on the bridge's /ws (a browser cannot put headers on a WebSocket).
//   The password goes nowhere else. Such a page never connects the arm by itself: only its Connect button does.

export const PasswordHeader = "X-Clancy-Password";
export const PasswordStorageKey = "SO101.RemotePassword";
// The window event InstallPasswordFetch raises when the robot's PC refuses one of this page's requests.
export const AccessRefusedEvent = "clancy-access-refused";
export type AccessCode = "cross_site" | "remote_off" | "password_required" | "password_wrong" | "locked_out";
export type AccessProblem = { Code: AccessCode; Text: string };
// The bridge closes another device's /ws with these when it refuses the password (RemoteAccess.CloseCodes).
export const AccessCloseCodes: Record<number, AccessCode> = { 4401: "password_wrong", 4403: "remote_off", 4429: "locked_out" };
const GuardedPaths = /^\/(bridge-launch|session-log|ollama|depth-server)(\/|$)/;
const AccessCodes = new Set<string>(["cross_site", "remote_off", "password_required", "password_wrong", "locked_out"]);

export function IsLoopbackHost(Host: string): boolean {
  const Name = Host.replace(/^\[|\]$/g, "").toLowerCase();
  return Name === "" || Name === "localhost" || Name === "::1" || /^127\.\d{1,3}\.\d{1,3}\.\d{1,3}$/.test(Name);
}

export function IsAccessCode(Value: unknown): Value is AccessCode {
  return typeof Value === "string" && AccessCodes.has(Value);
}

export function StoredPassword(): string | null {
  try { return localStorage.getItem(PasswordStorageKey) || null; } catch { return null; }
}

export function StorePassword(Value: string | null): void {
  try {
    if (Value) localStorage.setItem(PasswordStorageKey, Value);
    else localStorage.removeItem(PasswordStorageKey);
  } catch { /* Storage optional: the password then lasts only until the page closes. */ }
}

// Whether a request goes to the robot's PC and so carries the password: the bridge (port 8000 of the host that served
// this page) or this dev server's own routes. Model servers, other sites and data URLs never see it.
export function CarriesPassword(Url: string, Page: { origin: string; hostname: string }): boolean {
  let Target: URL;
  try { Target = new URL(Url, Page.origin); } catch { return false; }
  if (Target.origin === Page.origin) return GuardedPaths.test(Target.pathname);
  return ["http:", "https:"].includes(Target.protocol) && Target.hostname === Page.hostname && Target.port === "8000";
}

// What the user is told for each refusal (the server sends its own text too; this is the fallback).
export function AccessText(Code: AccessCode, Text?: string | null): string {
  if (Text) return Text;
  return {
    cross_site: "Refused: this request came from a web page that is not Clancy's own.",
    remote_off: "Remote access is off: only the robot's PC can use the arm until a remote password is set there.",
    password_required: "This device needs the arm password set on the robot's PC.",
    password_wrong: "Wrong arm password.",
    locked_out: "Too many wrong arm passwords from this device: wait 10 minutes, then try again.",
  }[Code];
}

// The first message another device's page sends on /ws: the stored password, or none (the bridge then answers
// password_required, or remote_off when no password is set on the PC at all, which tells the page what to show).
export function AuthMessage(): { type: "auth"; password: string } {
  return { type: "auth", password: StoredPassword() ?? "" };
}

// Install once, before the app renders (main.tsx). On another device every fetch of this page to the robot's PC
// carries the stored password, and a refusal from it raises AccessRefusedEvent (the app then asks for the password
// again). On this PC nothing is wrapped.
export function InstallPasswordFetch(Target: Window = window): void {
  if (IsLoopbackHost(Target.location.hostname)) return;
  const Original = Target.fetch.bind(Target);
  Target.fetch = (Input: RequestInfo | URL, Init?: RequestInit) => {
    const Url = typeof Input === "string" ? Input : Input instanceof URL ? Input.href : Input.url;
    if (!CarriesPassword(Url, Target.location)) return Original(Input, Init);
    const Password = StoredPassword();
    let Sent = Init;
    if (Password) {
      const Headers_ = new Headers(Init?.headers ?? (Input instanceof Request ? Input.headers : undefined));
      Headers_.set(PasswordHeader, Password);
      Sent = { ...Init, headers: Headers_ };
    }
    return Original(Input, Sent).then(Reply => {
      if ([401, 403, 429].includes(Reply.status)) {
        void Reply.clone().json().then((Body: { access?: unknown; error?: unknown }) => {
          if (IsAccessCode(Body?.access)) {
            Target.dispatchEvent(new CustomEvent<AccessProblem>(AccessRefusedEvent,
              { detail: { Code: Body.access, Text: AccessText(Body.access, typeof Body.error === "string" ? Body.error : null) } }));
          }
        }).catch(() => { /* Not a refusal of ours. */ });
      }
      return Reply;
    });
  };
}
