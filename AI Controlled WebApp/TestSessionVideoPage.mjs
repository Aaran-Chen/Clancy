// The session video's switch, page half (src/lib/SessionVideo.ts): what the page tells the dev server when a task
// starts, runs and ends. App.tsx calls Update on every change of its task (a step keeps the task's id) and End when
// the page closes. A fake sender and fake timers; then the real SignalTaskVideo against a stubbed fetch.
import { build as Build } from "esbuild";
import { test as Test } from "node:test";
import Assert from "node:assert/strict";

const Load = async File => {
  const Bundle = await Build({ entryPoints: [File], bundle: true, write: false, platform: "neutral", format: "esm" });
  return import(`data:text/javascript;base64,${Buffer.from(Bundle.outputFiles[0].text).toString("base64")}`);
};
const Video = await Load("src/lib/SessionVideo.ts");

function Recording() {
  const Sent = [];
  let Keys = 0;
  const Signals = new Video.TaskVideoSignals((State, Key, Details = {}) => Sent.push([State, Key, Details]), () => `key-${++Keys}`, 50);
  return { Sent, Signals };
}
const Pause = Milliseconds => new Promise(Resolve => setTimeout(Resolve, Milliseconds));

Test("a task's start, its alive every 15 s, and its end with the task_end reason", async () => {
  Assert.equal(Video.TaskAliveMs, 15_000);
  const { Sent, Signals } = Recording();
  Signals.Update(null, "", null);
  Assert.deepEqual(Sent, [], "no task: nothing");
  Signals.Update(1, "grab the tin", null);
  Assert.deepEqual(Sent, [["start", "key-1", { Task: "grab the tin" }]]);
  Assert.equal(Signals.Running, "key-1");
  Signals.Update(1, "grab the tin", null);  // a step of the same task (the check-ins' nextTask)
  await Pause(180);
  const Alive = Sent.filter(Item => Item[0] === "alive");
  Assert.ok(Alive.length >= 2 && Alive.length <= 4, `${Alive.length} alive at 50 ms over 180 ms`);
  Assert.ok(Alive.every(Item => Item[1] === "key-1"));
  Signals.Update(null, "", "the AI reported the task done.");
  Assert.deepEqual(Sent.at(-1), ["end", "key-1", { Reason: "the AI reported the task done." }]);
  Assert.equal(Signals.Running, null);
  const Count = Sent.length;
  await Pause(150);
  Assert.equal(Sent.length, Count, "no alive after the end");
});

Test("a new task ends the one it replaces and gets its own key; the page closing ends it", () => {
  const { Sent, Signals } = Recording();
  Signals.Update(4, "pick up the cup", null);
  Signals.Update(5, "put it down", null);
  Assert.deepEqual(Sent, [["start", "key-1", { Task: "pick up the cup" }], ["end", "key-1", { Reason: "a new task replaced it" }],
    ["start", "key-2", { Task: "put it down" }]]);
  Signals.Update(null, "", null);
  Assert.deepEqual(Sent.at(-1), ["end", "key-2", { Reason: "the task ended" }]);
  Signals.Update(6, "wave", null);
  Signals.End("the page was closed or reloaded");
  Assert.deepEqual(Sent.at(-1), ["end", "key-3", { Reason: "the page was closed or reloaded" }]);
  Signals.End("again");
  Assert.equal(Sent.at(-1)[1], "key-3", "an end with no task says nothing");
});

Test("keys are distinct, and the body carries only what the recorder takes", () => {
  const Keys = new Set(Array.from({ length: 2000 }, () => Video.NewTaskVideoKey()));
  Assert.equal(Keys.size, 2000);
  for (const Key of Keys) Assert.match(Key, /^[a-z0-9]+-[a-z0-9]{8}$/, "the recorder's task_id pattern");
  Assert.deepEqual(Video.TaskVideoBody("alive", "k-1"), { state: "alive", task_id: "k-1" });
  Assert.equal(Video.TaskVideoBody("start", "k-1", { Task: "x".repeat(900) }).task.length, 500);
  Assert.equal(Video.TaskVideoBody("end", "k-1", { Reason: "y".repeat(900) }).reason.length, 300);
});

Test("SignalTaskVideo posts JSON to the dev server with keepalive, and never from an automated browser", async () => {
  const Calls = [];
  globalThis.fetch = async (Url, Init) => { Calls.push([Url, Init]); return new Response("{}"); };
  Object.defineProperty(globalThis, "navigator", { value: { webdriver: false }, configurable: true });
  Video.SignalTaskVideo("start", "k-9", { Task: "wave" });
  Assert.equal(Calls.length, 1);
  const [Url, Init] = Calls[0];
  Assert.equal(Url, "/session-log/recording");
  Assert.equal(Init.method, "POST");
  Assert.equal(Init.keepalive, true);
  Assert.equal(Init.headers["Content-Type"], "application/json");
  Assert.deepEqual(JSON.parse(Init.body), { state: "start", task_id: "k-9", task: "wave" });
  Object.defineProperty(globalThis, "navigator", { value: { webdriver: true }, configurable: true });
  Video.SignalTaskVideo("end", "k-9");
  Assert.equal(Calls.length, 1, "automated browsers stay out of the recording, as of the record");
  globalThis.fetch = async () => { throw new Error("offline"); };
  Object.defineProperty(globalThis, "navigator", { value: { webdriver: false }, configurable: true });
  Video.SignalTaskVideo("alive", "k-9");  // must not throw
  await Pause(10);
});
