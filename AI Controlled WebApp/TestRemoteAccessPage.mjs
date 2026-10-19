// The page's half of the access rule (src/lib/RemoteAccess.ts): which requests carry the arm password, that only the
// robot's PC ever sees it, that nothing is wrapped on this PC, and that a refusal from the PC reaches the app. Pure
// functions on a stand-in window (no browser): TestRemoteAccessUi.mjs drives the real page.
import { build as Build } from "esbuild";
import { test as Test } from "node:test";
import Assert from "node:assert/strict";

const Load = async File => {
  const Bundle = await Build({ entryPoints: [File], bundle: true, write: false, platform: "node", format: "esm" });
  return import(`data:text/javascript;base64,${Buffer.from(Bundle.outputFiles[0].text).toString("base64")}`);
};
const Stored = new Map();
globalThis.localStorage = { getItem: Key => Stored.get(Key) ?? null, setItem: (Key, Value) => Stored.set(Key, String(Value)), removeItem: Key => Stored.delete(Key) };
const Access = await Load("src/lib/RemoteAccess.ts");

// A stand-in window at Href whose fetch records what it was asked and answers Reply.
function FakeWindow(Href, Reply = () => new Response("{}", { status: 200 })) {
  const Target = new EventTarget();
  const Url = new URL(Href);
  Target.location = { href: Url.href, origin: Url.origin, hostname: Url.hostname };
  Target.Sent = [];
  Target.fetch = async (Input, Init) => {
    const Headers_ = new Headers(Init?.headers ?? (Input instanceof Request ? Input.headers : undefined));
    Target.Sent.push({ Url: typeof Input === "string" ? Input : Input.url ?? Input.href, Password: Headers_.get(Access.PasswordHeader), Init });
    return Reply(Input, Init);
  };
  return Target;
}

Test("this PC is a loopback name; everything else is another device", () => {
  for (const Host of ["127.0.0.1", "localhost", "LOCALHOST", "[::1]", "::1", "127.0.1.1", ""]) Assert.equal(Access.IsLoopbackHost(Host), true, Host);
  for (const Host of ["192.168.1.20", "100.101.102.103", "my-pc", "my-pc.tailnet.ts.net", "10.0.0.2", "0.0.0.0", "128.0.0.1"]) {
    Assert.equal(Access.IsLoopbackHost(Host), false, Host);
  }
});

Test("only requests to the robot's PC carry the password", () => {
  const Page = { origin: "http://192.168.1.20:5181", hostname: "192.168.1.20" };
  for (const Url of ["http://192.168.1.20:8000/ports", "http://192.168.1.20:8000/depthcam/frame.jpg?after=3", "/ollama/api/chat",
    "/bridge-launch/start", "/session-log/event", "/depth-server/depth", "http://192.168.1.20:5181/ollama/api/tags"]) {
    Assert.equal(Access.CarriesPassword(Url, Page), true, Url);
  }
  for (const Url of ["http://evil.example:8000/ports", "http://192.168.1.21:8000/ports", "http://192.168.1.20:11434/api/chat",
    "http://192.168.1.20:8001/depth", "/src/main.tsx", "/LocalDisplayPreferences.json", "/ollamax", "data:image/png;base64,AAAA",
    "https://example.com/ollama/api/chat", "ws://192.168.1.20:8000/ws"]) {
    Assert.equal(Access.CarriesPassword(Url, Page), false, Url);
  }
});

Test("on this PC nothing is wrapped and no password is ever sent", async () => {
  Stored.set(Access.PasswordStorageKey, "correct horse battery");
  const Window = FakeWindow("http://127.0.0.1:5181/");
  const Original = Window.fetch;
  Access.InstallPasswordFetch(Window);
  Assert.equal(Window.fetch, Original, "this PC's fetch is untouched");
  Stored.clear();
});

Test("another device: its requests to the PC carry the stored password, nothing else does", async () => {
  const Window = FakeWindow("http://192.168.1.20:5181/");
  Access.InstallPasswordFetch(Window);
  await Window.fetch("http://192.168.1.20:8000/ports");
  Assert.equal(Window.Sent.at(-1).Password, null, "no password stored yet: nothing to send");
  Access.StorePassword("correct horse battery");
  await Window.fetch("http://192.168.1.20:8000/ports");
  await Window.fetch("/ollama/api/chat", { method: "POST", headers: { "Content-Type": "application/json" }, body: "{}" });
  await Window.fetch(new Request("http://192.168.1.20:8000/motion", { method: "POST", headers: { "X-Other": "kept" } }));
  await Window.fetch("http://192.168.1.20:11434/api/tags");
  await Window.fetch("https://example.com/collect");
  Assert.deepEqual(Window.Sent.slice(1).map(Item => Item.Password),
    ["correct horse battery", "correct horse battery", "correct horse battery", null, null]);
  Assert.equal(new Headers(Window.Sent[2].Init.headers).get("Content-Type"), "application/json", "the page's own headers are kept");
  Assert.equal(new Headers(Window.Sent[3].Init.headers).get("X-Other"), "kept", "a Request's headers are kept");
  Assert.deepEqual(Access.AuthMessage(), { type: "auth", password: "correct horse battery" });
  Access.StorePassword(null);
  Assert.deepEqual(Access.AuthMessage(), { type: "auth", password: "" });
});

Test("a refusal from the PC reaches the app; other failures do not", async () => {
  const Answers = {
    "http://192.168.1.20:8000/ports": () => new Response(JSON.stringify({ ok: false, access: "password_wrong", error: "Wrong arm password." }), { status: 401 }),
    "http://192.168.1.20:8000/notepad": () => new Response(JSON.stringify({ ok: false, access: "locked_out", error: "Too many." }), { status: 429 }),
    "http://192.168.1.20:8000/surfaces": () => new Response("not json", { status: 401 }),
    "/ollama/api/tags": () => new Response(JSON.stringify({ error: "model missing" }), { status: 404 }),
  };
  const Window = FakeWindow("http://192.168.1.20:5181/", Input => Answers[typeof Input === "string" ? Input : Input.url]());
  const Heard = [];
  Window.addEventListener(Access.AccessRefusedEvent, Event => Heard.push(Event.detail));
  Access.InstallPasswordFetch(Window);
  const Reply = await Window.fetch("http://192.168.1.20:8000/ports");
  Assert.equal(Reply.status, 401, "the page still gets the reply itself");
  Assert.equal((await Reply.json()).access, "password_wrong", "its body is still readable");
  await Window.fetch("http://192.168.1.20:8000/notepad");
  await Window.fetch("http://192.168.1.20:8000/surfaces");
  await Window.fetch("/ollama/api/tags");
  await new Promise(Resolve => setTimeout(Resolve, 20));
  Assert.deepEqual(Heard, [{ Code: "password_wrong", Text: "Wrong arm password." }, { Code: "locked_out", Text: "Too many." }]);
});
