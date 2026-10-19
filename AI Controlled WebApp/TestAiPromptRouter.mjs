import { build as Build } from "esbuild";
import { createRequire as CreateRequire } from "node:module";
import { test as Test } from "node:test";
import Assert from "node:assert/strict";
const Bundle = await Build({ stdin: { contents: "export * from './src/lib/AiPromptRouter'", resolveDir: process.cwd(), loader: "ts" },
  bundle: true, platform: "node", format: "cjs", write: false });
const Module = { exports: {} };
new Function("require", "module", "exports", Bundle.outputFiles[0].text)(CreateRequire(import.meta.url), Module, Module.exports);
const { AiPromptRouter, BuildNimbleRouteBody, BuildTextOnlyMessages, ReadNimbleRoute, PromptRouterOptions } = Module.exports;
PromptRouterOptions.TimeoutMs = 35;
PromptRouterOptions.WarmupTimeoutMs = 100;
// Nimble's typed choice answer: the chosen option and a probability for each of the three.
const Decision = (Choice = "text", Probability = 0.99) => {
  const Rest = (1 - Probability) / 2;
  const Probabilities = Object.fromEntries(["text", "camera", "camera_depth"].map(Mode => [Mode, Mode === Choice ? Probability : Rest]));
  return { answers: { input_mode: { type: "choice", choice: Choice, probabilities: Probabilities, confidence: 0.95 } } };
};
const Response = (Data, Status = 200) => ({ ok: Status === 200, status: Status, json: async () => Data });
function Mock(Decide = () => Response(Decision()), Version = "0.35.0", Models = [{ name: "nimble:latest", size: 9.5e9 }], Loaded = []) {
  const Calls = [];
  globalThis.fetch = async (Url, Init = {}) => {
    Calls.push({ Url, Init });
    if (Url.endsWith("/version")) return Response({ version: Version });
    if (Url.endsWith("/tags")) return Response({ models: Models });
    if (Url.endsWith("/ps")) return Response({ models: Loaded });
    return Decide(Url, Init);
  };
  return Calls;
}
Test("Nimble uses Jev typed choice with bounded input and no chat/reasoning fields", () => {
  const Body = BuildNimbleRouteBody("Hello");
  Assert.equal(Body.model, "nimble");
  Assert.equal(Body.keep_alive, "2h");
  Assert.deepEqual(Body.state, { user_prompt: "Hello" });
  Assert.equal(Body.questions.input_mode.type, "choice");
  Assert.deepEqual(Object.keys(Body.questions.input_mode.criteria), ["text", "camera", "camera_depth"]);
  for (const Key of ["messages", "think", "stream", "options"]) Assert.ok(!(Key in Body));
});
Test("only a valid, confident decision lowers the inputs", () => {
  Assert.equal(ReadNimbleRoute(Decision()).Mode, "text");
  Assert.equal(ReadNimbleRoute(Decision("camera", 0.95)).Mode, "camera");
  // Leaning to text without the margin: text and camera together cover it, so the camera goes along.
  Assert.equal(ReadNimbleRoute({ answers: { input_mode: { type: "choice", choice: "text", probabilities: { text: 0.6, camera: 0.35, camera_depth: 0.05 } } } }).Mode, "camera");
  Assert.equal(ReadNimbleRoute(Decision("text", 0.6)).Mode, "camera_depth");
  Assert.equal(ReadNimbleRoute(Decision("camera_depth", 0.99)).Mode, "camera_depth");
  Assert.throws(() => ReadNimbleRoute({ answers: { input_mode: { choice: "text" } } }), /invalid/);
  Assert.throws(() => ReadNimbleRoute(Decision("text", 1.1)), /invalid/);
  // The first router's two-way answer is not a route.
  Assert.throws(() => ReadNimbleRoute({ answers: { input_mode: { type: "choice", choice: "vision", probabilities: { text: 0.01, vision: 0.99 } } } }), /invalid/);
});
Test("conversation reaches Nimble, while physical and continued tasks require fresh measurements without it", async () => {
  const Calls = Mock(), Router = new AiPromptRouter(), Signal = new AbortController().signal;
  Assert.equal((await Router.Route("Explain inverse kinematics", Signal)).Mode, "text");
  Assert.equal(Calls.at(-1).Url, "/ollama/v1/systemone");
  for (const Prompt of ["How does a servo work?", "Explain how to open a gripper", "Tell me about inverse kinematics"])
    Assert.equal((await Router.Route(Prompt, Signal)).Source, "nimble");
  const Before = Calls.length;
  for (const Prompt of ["Grab the eraser", "Move left 3 cm", "Open the gripper", "continue", "Explain grippers and then open it", "Is the gripper open?"]) {
    const Route = await Router.Route(Prompt, Signal);
    Assert.equal(Route.Source, "safety", Prompt); Assert.equal(Route.Mode, "camera_depth", Prompt);
  }
  Assert.equal((await Router.Route("Yes", Signal, { Answer: true })).Mode, "camera_depth");
  Assert.equal((await Router.Route("a".repeat(8001), Signal)).Source, "safety");
  Assert.equal(Calls.length, Before);
});
Test("missing model and old Ollama use immediate cached fallback to the full inputs", async () => {
  for (const [Version, Models, Reason] of [["0.34.0", [{ name: "nimble" }], /0.35/], ["0.35.0", [], /Install nimble/]]) {
    const Calls = Mock(undefined, Version, Models), Router = new AiPromptRouter(), Signal = new AbortController().signal;
    const First = await Router.Route("Tell me something", Signal);
    Assert.equal(First.Mode, "camera_depth"); Assert.equal(First.Source, "fallback"); Assert.match(First.Reason, Reason);
    const Second = await Router.Route("Explain recursion", Signal);
    Assert.equal(Second.Mode, "text"); Assert.equal(Second.Source, "rules"); Assert.match(Second.Reason, Reason);
    Assert.equal(Calls.length, 3, "cached failure skips network on later prompts (version, tags and ps once)");
  }
});
Test("HTTP failure, malformed response and timeout fall back safely and abort the request", async () => {
  for (const Decide of [() => Response({}, 500), () => Response({}), () => new Promise(() => {})]) {
    const Calls = Mock(Decide), Router = new AiPromptRouter(), Started = performance.now(), Controller = new AbortController();
    const Result = await Router.Route("Tell me something", Controller.signal);
    Assert.equal(Result.Mode, "camera_depth"); Assert.equal(Result.Source, "fallback");
    Assert.ok(performance.now() - Started < 1000);
    if (Result.Reason.includes("timed out")) Assert.equal(Calls.find(Call => Call.Url.endsWith("/systemone")).Init.signal.aborted, true);
    Controller.abort();
    await new Promise(Resolve => setImmediate(Resolve));
  }
});
Test("cancellation discards the route instead of falling back to image capture", async () => {
  let Started;
  const AwaitStarted = new Promise(Resolve => { Started = Resolve; });
  const Calls = Mock(() => { Started(); return new Promise(() => {}); });
  const Controller = new AbortController(), Pending = new AiPromptRouter().Route("Hello", Controller.signal);
  await AwaitStarted;
  Controller.abort();
  await Assert.rejects(Pending, { name: "AbortError" });
  Assert.equal(Calls.at(-1).Init.signal.aborted, true);
  const Before = Calls.length;
  await Assert.rejects(new AiPromptRouter().Route("Hello", Controller.signal), { name: "AbortError" });
  Assert.equal(Calls.length, Before);
});
Test("warm-up is separate and never queues a user prompt behind a cold load", async () => {
  let Finish, Started;
  const AwaitStarted = new Promise(Resolve => { Started = Resolve; });
  Mock(() => { Started(); return new Promise(Resolve => { Finish = Resolve; }); });
  const Router = new AiPromptRouter(), Signal = new AbortController().signal, Warming = Router.Prepare(Signal);
  await AwaitStarted;
  const Routed = await Router.Route("Tell me something", Signal);
  Assert.equal(Routed.Mode, "camera_depth"); Assert.match(Routed.Reason, /warming/);
  // Cold Nimble (warming): clear prompts take the rule classifier's route at once, never waiting on the load.
  const Started_ = performance.now();
  const Text = await Router.Route("what is inverse kinematics?", Signal);
  Assert.deepEqual([Text.Mode, Text.Source], ["text", "rules"]); Assert.match(Text.Reason, /^Nimble is warming up; rules:/);
  Assert.ok(performance.now() - Started_ < 20, "no wait on the cold load");
  const Scene = await Router.Route("what's on the desk", Signal);
  Assert.deepEqual([Scene.Mode, Scene.Source], ["camera", "rules"]);
  const Grab = await Router.Route("grab the can", Signal);
  Assert.deepEqual([Grab.Mode, Grab.Source], ["camera_depth", "safety"]); Assert.match(Grab.Reason, /Acts on an object/);
  Finish(Response(Decision()));
  Assert.equal((await Warming).Ready, true);
});
Test("a cancelled warm-up is replaceable immediately and cannot clear its successor", async () => {
  let FirstStarted, Decisions = 0;
  const Started = new Promise(Resolve => { FirstStarted = Resolve; });
  Mock(() => { Decisions++; if (Decisions === 1) { FirstStarted(); return new Promise(() => {}); } return Response(Decision()); });
  const Router = new AiPromptRouter(), FirstController = new AbortController(), SecondController = new AbortController();
  const First = Router.Prepare(FirstController.signal);
  await Started;
  FirstController.abort();
  const Second = Router.Prepare(SecondController.signal);
  const Results = await Promise.allSettled([First, Second]);
  Assert.equal(Results[0].status, "rejected");
  Assert.equal(Results[1].value.Ready, true);
  Assert.equal((await Router.Route("Hello", SecondController.signal)).Mode, "text");
  Assert.equal(Decisions, 3);
});
Test("idle eviction triggers background warm-up, then routing recovers without a mode switch", async () => {
  const Router = new AiPromptRouter(), Controller = new AbortController(), OriginalNow = Date.now, OriginalKeepAlive = PromptRouterOptions.KeepAlive;
  Mock();
  await Router.Prepare(Controller.signal);
  try {
    const Later = OriginalNow() + 2000;
    Date.now = () => Later; PromptRouterOptions.KeepAlive = "1s";
    Assert.match((await Router.Route("Hello", Controller.signal)).Reason, /warming up after idle/);
    Assert.equal((await Router.Prepare(Controller.signal)).Ready, true);
    Assert.equal((await Router.Route("Hello", Controller.signal)).Mode, "text");
  } finally { Date.now = OriginalNow; PromptRouterOptions.KeepAlive = OriginalKeepAlive; Controller.abort(); }
});
Test("KeepWarm: a fresh /api/ps check warms Nimble again when Ollama dropped it or its load runs out soon, else nothing", async () => {
  const Signal = new AbortController().signal;
  const Settle = () => new Promise(Resolve => setTimeout(Resolve, 5));
  const Decisions = Calls => Calls.filter(Call => Call.Url.endsWith("/systemone"));
  const Expiring = Minutes => [{ name: "nimble:latest", size_vram: 9e9, expires_at: new Date(Date.now() + Minutes * 60000).toISOString() }];
  // Not loaded (Ollama restarted, evicted, or never warmed): warm it, with the page's 2 h keep_alive.
  let Calls = Mock(undefined, "0.35.0", undefined, []);
  let Router = new AiPromptRouter();
  Assert.equal(await Router.KeepWarm(Signal), true); await Settle();
  Assert.equal(Decisions(Calls).length, 1); Assert.equal(JSON.parse(Decisions(Calls)[0].Init.body).keep_alive, "2h");
  Assert.equal(Calls.filter(Call => Call.Url.endsWith("/ps")).length, 1, "one fresh check");
  // Loaded for another hour: nothing to do, and the check is never cached across ticks.
  Calls = Mock(undefined, "0.35.0", undefined, Expiring(60));
  Assert.equal(await Router.KeepWarm(Signal), false); Assert.equal(await Router.KeepWarm(Signal), false);
  Assert.equal(Decisions(Calls).length, 0); Assert.equal(Calls.filter(Call => Call.Url.endsWith("/ps")).length, 2);
  // Running out within the 5 minute margin (even when another client shortened it): warm it again.
  Calls = Mock(undefined, "0.35.0", undefined, Expiring(2));
  Assert.equal(await Router.KeepWarm(Signal), true); await Settle();
  Assert.equal(Decisions(Calls).length, 1);
  // Would not fit the VRAM budget, not installed, or disabled: never loads it.
  Calls = Mock(undefined, "0.35.0", undefined, [{ name: "qwen3.8:27b", size_vram: 17.5e9 }, { name: "other", size_vram: 5e9 }]);
  Assert.equal(await new AiPromptRouter().KeepWarm(Signal), false);
  Calls = Mock(undefined, "0.35.0", [], []);
  Assert.equal(await new AiPromptRouter().KeepWarm(Signal), false);
  PromptRouterOptions.Enabled = false;
  try { Calls = Mock(); Assert.equal(await new AiPromptRouter().KeepWarm(Signal), false); Assert.equal(Calls.length, 0); }
  finally { PromptRouterOptions.Enabled = true; }
  await Settle();
  Assert.equal(Decisions(Calls).length, 0);
  // While a warm-up runs, a tick does nothing.
  let Finish;
  Calls = Mock(() => new Promise(Resolve => { Finish = Resolve; }), "0.35.0", undefined, []);
  Router = new AiPromptRouter();
  const Warming = Router.Prepare(Signal);
  await Settle();
  Assert.equal(await Router.KeepWarm(Signal), false);
  Finish(Response(Decision())); await Warming;
});
Test("a background re-warm started by a prompt outlives that prompt being stopped", async () => {
  let Decisions = 0;
  Mock(() => { Decisions++; return Decisions === 1 ? new Promise(() => {}) : Response(Decision()); });
  const Router = new AiPromptRouter(), Request = new AbortController();
  // The first decision times out: the router warms in the background, then the user stops the request.
  Assert.match((await Router.Route("Explain recursion", Request.signal)).Reason, /timed out; rules:/);
  Request.abort();
  await new Promise(Resolve => setTimeout(Resolve, 5));
  Assert.equal(Decisions, 2, "the background warm-up ran");
  Assert.equal((await Router.Route("Explain recursion", new AbortController().signal)).Source, "nimble");
});
Test("text-only prompt is small, self-contained and has no images or motion tools", () => {
  const Messages = BuildTextOnlyMessages("Explain recursion");
  Assert.equal(Messages.length, 2);
  Assert.equal(Messages[1].content, "Explain recursion");
  Assert.ok(Messages.every(Message => typeof Message.content === "string" && !("images" in Message)));
  Assert.match(Messages[0].content, /no camera images/);
  Assert.match(Messages[0].content, /Do not.*produce motion proposals/);
});
