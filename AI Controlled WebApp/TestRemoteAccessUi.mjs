// Who may use the arm from the page (src/lib/RemoteAccess.ts, App.tsx's automatic connect; Aaran's decision 2,
// 2026-10-07 audit 2 H2 and M2), in headless Edge against a test server, with the bridge faked in the browser:
//   This PC      http://127.0.0.1:<port>: no password, no card, the arm connects by itself as it always did; after a
//                deliberate disconnect (the bridge's disconnected_on_purpose) it waits for a Connect click, which is
//                sent as an explicit connect.
//   Another      http://192.0.2.10:<port> (a documentation address Edge is told to serve from the test server, so the
//   device       page sees a non-loopback host, as on a phone): the password card; wrong and right passwords; the
//                password goes first on /ws and on every request to the robot's PC; the arm never connects by itself,
//                not after unlocking and not after a reload; only Connect sends connect, explicit. "Not now" leaves a
//                Locked chip; Settings forgets the password. Screenshots of the card at desktop and phone width.
// The fake bridge answers like back end/RemoteAccess.py (access message first, 4401 close). Safety: the suite serves this
// checkout on 5191-5199 itself (UiTestLib.mjs; CadTestUrl picks a test server of your own, never a live port); Edge maps
// the live ports (5180/5181, 8000, 8001, 8002, 11434) to a closed port and every request to them is stubbed or refused.
//   node TestRemoteAccessUi.mjs
import { chromium as Chromium } from "playwright";
import { LaunchOptions, StartUiServer } from "./UiTestLib.mjs";
import { execFileSync as Execute } from "node:child_process";
import Fs from "node:fs";
import Path from "node:path";
import Assert from "node:assert/strict";

const Root = import.meta.dirname, Backend = Path.resolve(Root, "../back end");
const Server = await StartUiServer();
const Port = Server.Port;
const LocalUrl = `http://127.0.0.1:${Port}/`, RemoteHost = "192.0.2.10", RemoteUrl = `http://${RemoteHost}:${Port}/`;
const Password = "correct horse battery";
const LivePorts = [8000, 8001, 8002, 11434];
const Python = Fs.readFileSync(Path.join(Root, "../.backend-python.txt"), "utf8").replace(/^﻿/, "").trim();
const BaseState = JSON.parse(Execute(Python, ["-c", "import json, bridge_server; print(json.dumps(bridge_server.BridgeController().state_snapshot()))"],
  { cwd: Backend, encoding: "utf8", env: { ...process.env, SO101_DEPTH_CAMERA: "0", SO101_RECORDER: "0" } }));
Object.assign(BaseState, { arm_connected: false, torque: false, port: "", motion_fault: null, disconnected_on_purpose: null });
const Shots = Path.join(Backend, "TestArtifacts", "RemoteAccessUi");
Fs.mkdirSync(Shots, { recursive: true });

const HostRules = [...LivePorts.flatMap(P => ["127.0.0.1", "localhost", RemoteHost].map(Host => `MAP ${Host}:${P} 127.0.0.1:9`)),
  `MAP ${RemoteHost}:${Port} 127.0.0.1:${Port}`].join(", ");
const Browser = await Chromium.launch(LaunchOptions({ args: [`--host-resolver-rules=${HostRules}`] }));
const Pause = Milliseconds => new Promise(Resolve => setTimeout(Resolve, Milliseconds));
const Live = new RegExp(`^(https?|wss?)://[^/]*:(${LivePorts.join("|")})(/|\\?|$)`);

// One browser profile (a device) with the bridge faked like back end/RemoteAccess.py. Records what the page sent.
async function Device(Width = 1600, Height = 1000) {
  const Context = await Browser.newContext({ viewport: { width: Width, height: Height }, serviceWorkers: "block" });
  const D = { Context, State: structuredClone(BaseState), Socket: [], Http: [], Side: [], Blocked: [], Errors: [], Timers: new Set() };
  // Refused and logged: anything to a live port the stubs below miss.
  await Context.route(Live, Route => { D.Blocked.push(Route.request().url()); return Route.abort("blockedbyclient"); });
  await Context.routeWebSocket(Live, Socket => { D.Blocked.push(`WS ${Socket.url()}`); Socket.close({ code: 1008, reason: "test firewall" }); });
  await Context.route(/\/(session-log|bridge-launch|depth-server|ollama)\//, Route => {
    const Request = Route.request();
    D.Side.push({ Url: Request.url(), Password: Request.headers()["x-clancy-password"] ?? null });
    const Path_ = new URL(Request.url()).pathname;
    if (Path_.endsWith("/api/tags")) return Route.fulfill({ json: { models: [{ name: "qwen3.8:27b", size: 17000000000 }] } });
    if (Path_.endsWith("/api/show")) return Route.fulfill({ json: { capabilities: ["vision"] } });
    if (Path_.endsWith("/api/ps")) return Route.fulfill({ json: { models: [] } });
    return Route.fulfill({ json: { ok: true, data: [] } });
  });
  await Context.route("**/LocalDisplayPreferences.json", Route => Route.fulfill({ json: {} }));
  await Context.route(/:8000\//, Route => {
    const Request = Route.request(), Url = new URL(Request.url());
    if (Request.method() === "OPTIONS") return Route.fulfill({ status: 200, headers: { "access-control-allow-origin": "*", "access-control-allow-headers": "*" } });
    const Given = Request.headers()["x-clancy-password"] ?? null;
    D.Http.push({ Host: Url.hostname, Path: Url.pathname, Method: Request.method(), Password: Given });
    if (Url.hostname !== "127.0.0.1" && Given !== Password) {
      const Code = Given ? "password_wrong" : "password_required";
      return Route.fulfill({ status: 401, headers: { "access-control-allow-origin": "*" }, json: { ok: false, access: Code, error: Code === "password_wrong" ? "Wrong arm password." : "This device needs the arm password set on the robot's PC: enter it in Clancy." } });
    }
    if (Url.pathname === "/ports") return Route.fulfill({ headers: { "access-control-allow-origin": "*" }, json: ["FAKE"] });
    return Route.fulfill({ headers: { "access-control-allow-origin": "*" }, json: { ok: true, calibrated: false, points: [], point_count: 0, content: "" } });
  });
  await Context.routeWebSocket(/:8000\/ws/, Socket => {
    const Remote = new URL(Socket.url()).hostname !== "127.0.0.1";
    let LetIn = !Remote;
    const Stream = () => {
      Socket.send(JSON.stringify(D.State));
      const Timer = setInterval(() => Socket.send(JSON.stringify(D.State)), 150); D.Timers.add(Timer);
      Socket.onClose(() => { clearInterval(Timer); D.Timers.delete(Timer); });
    };
    Socket.onMessage(Raw => {
      const Message = JSON.parse(String(Raw));
      D.Socket.push({ Remote, ...Message });
      if (!LetIn) {
        // The bridge's AuthenticateSocket: the first message must give the password.
        const Code = Message.type !== "auth" || !Message.password ? "password_required" : Message.password === Password ? null : "password_wrong";
        if (Code) {
          Socket.send(JSON.stringify({ type: "access", ok: false, access: Code, text: Code === "password_wrong" ? "Wrong arm password." : "This device needs the arm password set on the robot's PC: enter it in Clancy." }));
          Socket.close({ code: 4401, reason: Code });
          return;
        }
        LetIn = true;
        Socket.send(JSON.stringify({ type: "access", ok: true, access: "password", text: "Arm password accepted" }));
        Stream();
        return;
      }
      if (Message.type === "connect") Object.assign(D.State, { arm_connected: true, torque: true, port: Message.port, disconnected_on_purpose: null });
      if (Message.type === "disconnect") Object.assign(D.State, { arm_connected: false, torque: false, disconnected_on_purpose: { at: "now", by: Remote ? "another device" : "this PC" } });
      Socket.send(JSON.stringify({ type: "status", ok: true, text: "OK" }));
    });
    if (LetIn) Stream();
  });
  D.Page = await Context.newPage();
  D.Page.on("pageerror", Error => D.Errors.push(Error.message));
  D.Connects = () => D.Socket.filter(Message => Message.type === "connect");
  D.Card = () => D.Page.getByRole("dialog", { name: "Arm password" });
  D.Close = async () => {
    for (const Timer of D.Timers) clearInterval(Timer);
    await Context.close();
    Assert.deepEqual(D.Blocked, [], "nothing reached a live port");
    Assert.deepEqual(D.Errors, [], "no page errors");
  };
  return D;
}

async function Until(Check, What, Milliseconds = 20000) {
  const End = Date.now() + Milliseconds;
  while (Date.now() < End) { if (await Check()) return; await Pause(100); }
  throw new Error(`timed out waiting for ${What}`);
}

const Passed = [];
const Step = async (Name, Body) => { await Body(); Passed.push(Name); console.log(`ok - ${Name}`); };

try {
  await Step("this PC: no password, no card, and the arm connects by itself as before", async () => {
    const D = await Device();
    await D.Page.goto(LocalUrl, { waitUntil: "domcontentloaded" });
    await Until(() => D.Connects().length > 0, "the automatic connect");
    Assert.deepEqual(D.Connects(), [{ Remote: false, type: "connect", port: "FAKE" }], "an automatic connect, not an explicit one");
    Assert.equal(D.Socket.filter(Message => Message.type === "auth").length, 0, "no password asked on this PC");
    Assert.ok(D.Http.length > 0 && D.Http.every(Item => Item.Password === null), "no password header on this PC");
    Assert.ok(D.Side.every(Item => Item.Password === null));
    Assert.equal(await D.Card().count(), 0);
    Assert.equal(await D.Page.getByRole("button", { name: "Locked", exact: true }).count(), 0);
    await D.Page.getByRole("button", { name: "Disconnect", exact: true }).waitFor();
    await D.Page.getByRole("button", { name: "Settings", exact: true }).click();
    Assert.equal(await D.Page.getByRole("button", { name: "Forget the arm password" }).count(), 0, "nothing to forget on this PC");
    await D.Close();
  });

  await Step("this PC after a deliberate disconnect: waits for Connect, which is explicit", async () => {
    const D = await Device();
    D.State.disconnected_on_purpose = { at: "2026-10-08T09:00:00", by: "another device" };
    await D.Page.goto(LocalUrl, { waitUntil: "domcontentloaded" });
    await D.Page.locator(".connection-pill.live").waitFor({ state: "attached" });
    await Pause(4000);
    Assert.deepEqual(D.Connects(), [], "no automatic connect after a deliberate disconnect");
    await D.Page.getByRole("button", { name: "Connect", exact: true }).click();
    await Until(() => D.Connects().length > 0, "the clicked connect");
    Assert.deepEqual(D.Connects(), [{ Remote: false, type: "connect", port: "FAKE", explicit: true }]);
    await D.Close();
  });

  await Step("another device: the card, a wrong password, the right one; nothing connects by itself", async () => {
    const D = await Device();
    await D.Page.goto(RemoteUrl, { waitUntil: "domcontentloaded" });
    await D.Card().waitFor();
    Assert.deepEqual(D.Socket.map(({ type, password }) => [type, password]), [["auth", ""]], "the first and only message: auth, no password yet");
    Assert.equal(await D.Page.locator(".connection-pill.live").count(), 0, "not live while locked");
    await D.Page.screenshot({ path: Path.join(Shots, "card_desktop.png"), animations: "disabled" });
    await D.Card().getByLabel("Password").fill("not the password");
    await D.Card().getByRole("button", { name: "Unlock" }).click();
    await D.Card().getByText("Wrong arm password.").waitFor();
    Assert.deepEqual(D.Socket.at(-1), { Remote: true, type: "auth", password: "not the password" });
    await D.Card().getByLabel("Password").fill(Password);
    await D.Card().getByRole("button", { name: "Unlock" }).click();
    await D.Page.locator(".connection-pill.live").waitFor({ state: "attached" });
    await D.Card().waitFor({ state: "detached" });
    Assert.deepEqual(D.Socket.filter(Message => Message.type === "auth").map(Message => Message.password), ["", "not the password", Password]);
    await Pause(4000);
    Assert.deepEqual(D.Connects(), [], "unlocking never connects the arm");
    Assert.equal(await D.Page.evaluate(() => localStorage.getItem("SO101.RemotePassword")), Password, "kept in this browser");
    const Before = D.Http.length;
    await D.Page.getByRole("combobox", { name: "Serial port" }).focus();  // loads the ports again
    await Until(() => D.Http.length > Before, "a bridge request after unlocking");
    Assert.equal(D.Http.at(-1).Password, Password, "requests to the bridge carry the password");
    Assert.ok(D.Side.some(Item => Item.Password === Password), "and the dev server's routes too");
    await D.Page.getByRole("button", { name: "Connect", exact: true }).click();
    await Until(() => D.Connects().length > 0, "the clicked connect");
    Assert.deepEqual(D.Connects(), [{ Remote: true, type: "connect", port: "FAKE", explicit: true }], "only Connect connects, explicitly");
    await D.Page.getByRole("button", { name: "Disconnect", exact: true }).click();
    await Until(() => D.Socket.some(Message => Message.type === "disconnect"), "the disconnect");
    // A reload: the stored password goes first, still no connect by itself.
    D.Socket.length = 0;
    await D.Page.reload({ waitUntil: "domcontentloaded" });
    await D.Page.locator(".connection-pill.live").waitFor({ state: "attached" });
    Assert.deepEqual(D.Socket[0], { Remote: true, type: "auth", password: Password });
    await Pause(4000);
    Assert.deepEqual(D.Connects(), [], "a reloaded page on another device does not connect by itself");
    Assert.equal(await D.Card().count(), 0);
    // Settings: forget it.
    await D.Page.getByRole("button", { name: "Settings", exact: true }).click();
    await D.Page.getByRole("button", { name: "Forget the arm password" }).click();
    await D.Card().waitFor();
    Assert.equal(await D.Page.evaluate(() => localStorage.getItem("SO101.RemotePassword")), null);
    await D.Close();
  });

  await Step("another device: Not now leaves a Locked chip; the card fits a phone", async () => {
    const D = await Device(390, 844);
    await D.Page.goto(RemoteUrl, { waitUntil: "domcontentloaded" });
    await D.Card().waitFor();
    await D.Page.evaluate(() => document.fonts.ready);
    await Pause(300);
    const Box = await D.Page.locator(".access-card").boundingBox();
    Assert.ok(Box.x >= 0 && Box.y >= 0 && Box.x + Box.width <= 390 && Box.y + Box.height <= 844, `the card fits the phone: ${JSON.stringify(Box)}`);
    await D.Page.screenshot({ path: Path.join(Shots, "card_phone.png"), animations: "disabled" });
    await D.Card().getByRole("button", { name: "Not now" }).click();
    await D.Card().waitFor({ state: "detached" });
    const Chip = D.Page.getByRole("button", { name: "Locked", exact: true });
    await Chip.waitFor();
    Assert.ok(await Chip.isVisible(), "the Locked chip shows on a phone");
    await D.Page.screenshot({ path: Path.join(Shots, "locked_phone.png"), animations: "disabled" });
    await Pause(1500);
    Assert.equal(await D.Card().count(), 0, "refused requests do not bring the card back after Not now");
    await Chip.click();
    await D.Card().waitFor();
    Assert.deepEqual(D.Connects(), []);
    await D.Close();
  });
  console.log(`PASS ${Passed.length} of ${Passed.length}: TestRemoteAccessUi (screenshots in ${Shots})`);
} finally {
  await Browser.close(); await Server.Close();
}
