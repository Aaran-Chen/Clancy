// The three-way prompt router (src/lib/AiPromptRouter.ts) against a mocked Ollama: what each route sends the main
// model and the work it skips, the safety floor Nimble cannot lower, the fallbacks when Nimble is missing, slow or
// wrong, and the AI panel's status line and the session record's prompt_route event. Nothing reaches Ollama: fetch
// is replaced. The browser side is TestAiPromptRouterUi.mjs.
import { build as Build } from "esbuild";
import { readFileSync as ReadFile } from "node:fs";
import { createRequire as CreateRequire } from "node:module";
import { test as Test } from "node:test";
import Assert from "node:assert/strict";
import Path from "node:path";
import { fileURLToPath as FilePath } from "node:url";

const Root = Path.dirname(FilePath(import.meta.url));
const Saved = JSON.parse(ReadFile(Path.join(Root, "../back end/OllamaOptions.json"), "utf8"));
const Bundle = await Build({ stdin: { contents: "export * from './AiPromptRouter'; export { ToOllamaMessages } from './OllamaChat';",
  resolveDir: Path.join(Root, "src/lib"), loader: "ts" }, bundle: true, platform: "node", format: "cjs", write: false });
const Module = { exports: {} };
new Function("require", "module", "exports", Bundle.outputFiles[0].text)(CreateRequire(import.meta.url), Module, Module.exports);
const { AiPromptRouter, AnswerRoute, PromptRouteFloor, PromptRouterOptions, RouteRecord, RouteStatusText, RouteWork, ToOllamaMessages } = Module.exports;

const Defaults = { ...PromptRouterOptions };
PromptRouterOptions.TimeoutMs = 60;
PromptRouterOptions.WarmupTimeoutMs = 200;
const Probabilities = (Choice, P) => Object.fromEntries(["text", "camera", "camera_depth"].map(Mode => [Mode, Mode === Choice ? P : (1 - P) / 2]));
const Nimble = (Choice, P = 0.98) => ({ model: "nimble", answers: { input_mode: { type: "choice", choice: Choice, probabilities: Probabilities(Choice, P), confidence: 0.9 } },
  usage: { input_tokens: 600, output_tokens: 1 } });
const Json = (Data, Status = 200) => ({ ok: Status === 200, status: Status, json: async () => Data });
const Gb = 1e9;
/** A mocked Ollama: Answer(Body) answers /v1/systemone; Loaded is what /api/ps reports. Returns the decision requests. */
function Ollama({ Answer = () => Json(Nimble("text")), Version = "0.35.0", Installed = [{ name: "nimble:latest", size: 9.5 * Gb }], Loaded = [] } = {}) {
  const Asked = [];
  globalThis.fetch = async (Url, Init = {}) => {
    if (Url === "/ollama/api/version") return Json({ version: Version });
    if (Url === "/ollama/api/tags") return Json({ models: Installed });
    if (Url === "/ollama/api/ps") return Json({ models: Loaded });
    if (Url === "/ollama/v1/systemone") { Asked.push({ Body: JSON.parse(Init.body), Signal: Init.signal }); return Answer(JSON.parse(Init.body)); }
    throw new Error(`unexpected request ${Url}`);
  };
  return Asked;
}
const Signal = () => new AbortController().signal;

// The work an answer takes, counted: Capture for the camera frame, Ask for the main model. AnswerRoute is given no
// arm wait, /locate or depth map at all, so a route through it cannot run them.
function Work(Frame = "data:image/jpeg;base64,/9j/AAAA") {
  const Done = { Captures: 0, Asked: [] };
  return { Done, Steps: { Capture: async () => { Done.Captures++; return Frame; },
    Ask: async Messages => { Done.Asked.push(Messages); return "An answer"; }, ViewNote: "The TOP of the image is FARTHER from the robot." } };
}
const Images = Messages => ToOllamaMessages(Messages).flatMap(Message => Message.images ?? []);

Test("the configured defaults: Nimble, an 800 ms routing budget, kept warm 2 hours (30 minutes from the dev server), within a 29 GB budget", () => {
  Assert.equal(Saved.router_timeout_ms, 800);
  Assert.equal(Defaults.TimeoutMs, 800);
  Assert.equal(Defaults.Model, "nimble");
  Assert.equal(Defaults.Enabled, true);
  Assert.equal(Defaults.KeepAlive, "2h"); Assert.equal(Saved.router_keep_alive, "2h");
  Assert.equal(Defaults.StartupKeepAlive, "30m"); Assert.equal(Saved.router_startup_keep_alive, "30m");
  Assert.equal(Defaults.VramBudgetGb, 29);
  Assert.equal(Saved.router_vram_budget_gb, 29);
});

Test("each route sends only its inputs: text no image, camera one frame and no measurements, camera_depth the task path", async () => {
  Assert.deepEqual(RouteWork.text, { Image: false, Locate: false, DepthImage: false, Task: false });
  Assert.deepEqual(RouteWork.camera, { Image: true, Locate: false, DepthImage: false, Task: false });
  Assert.deepEqual(RouteWork.camera_depth, { Image: true, Locate: true, DepthImage: true, Task: true });

  const Text = Work();
  Assert.deepEqual(await AnswerRoute("text", "Explain recursion", Signal(), Text.Steps), { Reply: "An answer", Image: false });
  Assert.equal(Text.Done.Captures, 0, "text takes no frame");
  Assert.equal(Text.Done.Asked.length, 1);
  Assert.deepEqual(Images(Text.Done.Asked[0]), []);
  Assert.match(Text.Done.Asked[0][0].content, /no camera images/);

  const Camera = Work();
  Assert.deepEqual(await AnswerRoute("camera", "What colour is the cup?", Signal(), Camera.Steps), { Reply: "An answer", Image: true });
  Assert.equal(Camera.Done.Captures, 1, "camera takes one frame");
  Assert.equal(Camera.Done.Asked.length, 1);
  const [System, User] = Camera.Done.Asked[0];
  Assert.deepEqual(Images(Camera.Done.Asked[0]), ["/9j/AAAA"], "one colour image, no depth map");
  Assert.match(System.content, /no measured positions, depth, live arm state, or ability to act/);
  Assert.match(System.content, /Do not give positions in centimetres, produce motion proposals/);
  Assert.match(System.content, /The TOP of the image is FARTHER/);
  Assert.equal(User.content[0].text, "What colour is the cup?");
  Assert.doesNotMatch(JSON.stringify(Camera.Done.Asked[0]), /MEASURED OBJECT POSITIONS|motion_proposal|notepad_update/);

  // No frame (camera unavailable): the question goes without one, and the model is told it cannot see.
  const Blind = Work(null);
  Assert.deepEqual(await AnswerRoute("camera", "What do you see?", Signal(), Blind.Steps), { Reply: "An answer", Image: false });
  Assert.deepEqual(Images(Blind.Done.Asked[0]), []);
  Assert.match(Blind.Done.Asked[0][0].content, /could not take a photo/);

  // Stopped while the frame was taken: the main model is not asked.
  const Controller = new AbortController(), Stopped = Work();
  Stopped.Steps.Capture = async () => { Stopped.Done.Captures++; Controller.abort(); return "data:image/jpeg;base64,/9j/AAAA"; };
  await Assert.rejects(AnswerRoute("camera", "What do you see?", Controller.signal, Stopped.Steps), { name: "AbortError" });
  Assert.equal(Stopped.Done.Asked.length, 0);
});

Test("Nimble's three answers reach their routes through the mocked /v1/systemone", async () => {
  for (const [Prompt, Choice, Mode] of [["Explain recursion", "text", "text"], ["What colour is the cup?", "camera", "camera"],
    ["Where is the eraser?", "camera_depth", "camera_depth"], ["How far apart are the two bottles?", "camera_depth", "camera_depth"]]) {
    const Asked = Ollama({ Answer: () => Json(Nimble(Choice)) });
    const Route = await new AiPromptRouter().Route(Prompt, Signal());
    Assert.equal(Route.Mode, Mode, Prompt); Assert.equal(Route.Source, "nimble", Prompt); Assert.equal(Route.Choice, Choice);
    Assert.equal(Asked.length, 1);
    const Body = Asked[0].Body;
    Assert.deepEqual(Body.state, { user_prompt: Prompt });
    Assert.equal(Body.model, "nimble"); Assert.equal(Body.keep_alive, "2h");
    Assert.deepEqual(Object.keys(Body.questions.input_mode.criteria), ["text", "camera", "camera_depth"]);
  }
  // Uncertain: leaning to text without the 0.9 margin sends more, never less.
  Ollama({ Answer: () => Json({ answers: { input_mode: { type: "choice", choice: "text", probabilities: { text: 0.55, camera: 0.3, camera_depth: 0.15 } } } }) });
  const Unsure = await new AiPromptRouter().Route("Tell me something", Signal());
  Assert.equal(Unsure.Mode, "camera_depth"); Assert.match(Unsure.Reason, /below the 0.9 margin/);
});

Test("the safety floor overrides Nimble: object tasks, check-ins, answers and a task under way get camera_depth unasked", async () => {
  const Asked = Ollama({ Answer: () => Json(Nimble("text", 0.999)) }), Router = new AiPromptRouter();
  const Cases = [
    ["Grab the eraser", {}, /Acts on an object/],
    ["Pick up the red cup and put it in the basket", {}, /Acts on an object/],
    ["Explain how to pick up a cup", {}, /Acts on an object/],
    ["Touch the bottle", {}, /Acts on an object/],
    ["Move left 3 cm", {}, /Physical request/],
    ["Open the gripper", {}, /Physical request/],
    ["Is the gripper open?", {}, /arm state/],
    ["Go to x=10 y=20", {}, /Acts on an object/],
    ["x=10, y=20, z=5", {}, /Physical request/],
    ["Hello", { CheckIn: true }, /check-in/],
    ["Task check-in: continue the task", { CheckIn: true }, /check-in/],
    ["Yes, the blue one", { Answer: true }, /Answer to the AI's question/],
    ["What is recursion?", { TaskOngoing: true }, /under way/],
  ];
  for (const [Prompt, Context, Reason] of Cases) {
    const Route = await Router.Route(Prompt, Signal(), Context);
    Assert.equal(Route.Mode, "camera_depth", Prompt); Assert.equal(Route.Source, "safety", Prompt);
    Assert.match(Route.Reason, Reason, Prompt); Assert.equal(Route.Floor, "camera_depth");
    Assert.equal(Route.Choice, null, `${Prompt}: Nimble not asked`);
  }
  Assert.equal(Asked.length, 0, "no prompt that could lead to motion is sent to Nimble");

  // A live-scene question may not drop below the camera: Nimble's "text" is raised and says so.
  for (const Prompt of ["What do you see?", "What is on the table?", "Describe what the camera shows", "Can you see a red cup right now?"]) {
    Assert.equal(PromptRouteFloor(Prompt).Mode, "camera", Prompt);
    const Route = await Router.Route(Prompt, Signal());
    Assert.equal(Route.Mode, "camera", Prompt); Assert.equal(Route.Source, "safety", Prompt);
    Assert.match(Route.Reason, /Nimble chose prompt only; Asks about the live scene/);
  }
  // Conceptual questions with motion words are not physical requests; Nimble decides them.
  for (const Prompt of ["How does a servo work?", "Explain how to open a gripper", "What is inverse kinematics?"]) Assert.equal(PromptRouteFloor(Prompt).Mode, "text", Prompt);
  // Nimble may raise any prompt.
  Ollama({ Answer: () => Json(Nimble("camera_depth")) });
  Assert.equal((await new AiPromptRouter().Route("What do you see?", Signal())).Mode, "camera_depth");
});

Test("timeout, invalid answers and an unavailable router fall back to camera_depth for a prompt the rules cannot place", async () => {
  // "Tell me something" has no clear text or scene signal: the rule classifier leaves it on the full path.
  const Fallback = async (Setup, Reason, Prompt = "Tell me something") => {
    const Asked = Ollama(Setup), Started = performance.now();
    const Route = await new AiPromptRouter().Route(Prompt, Signal());
    Assert.equal(Route.Mode, "camera_depth", String(Reason)); Assert.equal(Route.Source, "fallback", String(Reason));
    Assert.match(Route.Reason, Reason); Assert.match(Route.Reason, /using camera and measured positions/);
    Assert.ok(performance.now() - Started < PromptRouterOptions.TimeoutMs + 400, `${Reason}: routed within its budget`);
    return { Asked, Route };
  };
  // Slow: the request is aborted at the deadline.
  const Slow = await Fallback({ Answer: () => new Promise(() => {}) }, /timed out/);
  Assert.equal(Slow.Asked[0].Signal.aborted, true);
  Assert.ok(Slow.Route.DurationMs >= PromptRouterOptions.TimeoutMs - 5);
  // Invalid: unknown choice, the first router's two-way answer, missing or unnormalised probabilities, HTTP errors.
  await Fallback({ Answer: () => Json({ answers: { input_mode: { type: "choice", choice: "everything", probabilities: Probabilities("text", 0.9) } } }) }, /invalid routing decision/);
  await Fallback({ Answer: () => Json({ answers: { input_mode: { type: "choice", choice: "vision", probabilities: { text: 0.01, vision: 0.99 } } } }) }, /invalid routing decision/);
  await Fallback({ Answer: () => Json({ answers: { input_mode: { type: "choice", choice: "text", probabilities: { text: 0.99, camera: 0.01 } } } }) }, /invalid routing probabilities/);
  await Fallback({ Answer: () => Json({ answers: { input_mode: { type: "choice", choice: "text", probabilities: { text: 0.9, camera: 0.9, camera_depth: 0.9 } } } }) }, /invalid routing probabilities/);
  await Fallback({ Answer: () => Json({}, 500) }, /HTTP 500/);
  await Fallback({ Answer: () => Promise.reject(new TypeError("fetch failed")) }, /fetch failed/);
  // Unavailable: old Ollama, model not installed, routing disabled. None asks Nimble.
  Assert.equal((await Fallback({ Version: "0.34.0" }, /Ollama 0.35/)).Asked.length, 0);
  Assert.equal((await Fallback({ Installed: [{ name: "qwen3.8:27b", size: 17.7 * Gb }] }, /Install nimble/)).Asked.length, 0);
  PromptRouterOptions.Enabled = false;
  try { Assert.equal((await Fallback({}, /disabled/)).Asked.length, 0); } finally { PromptRouterOptions.Enabled = true; }
  // A scene question that needs positions stays on the full path too.
  await Fallback({ Answer: () => Json({}, 503) }, /HTTP 503/, "Where is everything on the desk?");
});

Test("with Nimble unable to answer, the rule classifier sends clear prompts the fast way, never below the floor", async () => {
  const Rules = async (Setup, Prompt, Mode, Reason) => {
    const Asked = Ollama(Setup);
    const Route = await new AiPromptRouter().Route(Prompt, Signal());
    Assert.equal(Route.Mode, Mode, Prompt); Assert.equal(Route.Source, Mode === "camera_depth" ? "safety" : "rules", Prompt);
    Assert.match(Route.Reason, Reason, Prompt);
    Assert.equal(Route.Choice, null); Assert.equal(Route.Probabilities, null);
    return { Asked, Route };
  };
  for (const [Setup, Reason] of [[{ Answer: () => new Promise(() => {}) }, /timed out/], [{ Answer: () => Json({}, 500) }, /HTTP 500/],
    [{ Version: "0.34.0" }, /Ollama 0.35/], [{ Installed: [] }, /Install nimble/]]) {
    await Rules(Setup, "What is inverse kinematics?", "text", Reason);
    await Rules(Setup, "Write a haiku about Mondays", "text", /rules: a general question or writing/);
    await Rules(Setup, "Hello", "text", /rules: a greeting/);
    await Rules(Setup, "What's on the desk?", "camera", /rules: asks what the camera shows/);
    await Rules(Setup, "What do you see?", "camera", Reason);
  }
  // The floor runs first and Nimble is never asked: an object task is camera_depth whatever the router's state.
  const Grab = await Rules({ Answer: () => new Promise(() => {}) }, "Grab the can", "camera_depth", /Acts on an object/);
  Assert.equal(Grab.Asked.length, 0);
  // Never text by rule: arm, object, colour, position or ambiguous words, or a scene question with sizes.
  for (const Prompt of ["How does a servo work?", "What colour is the cup?", "What is this?", "what is that?", "Explain how it works",
    "Which objects are left of the cup?", "How big is the box on the desk?", "Can you describe this?", "Tell me something useful"]) {
    const Route = await new AiPromptRouter().Route(Prompt, Signal());
    Assert.equal(Route.Mode, "camera_depth", Prompt); Assert.notEqual(Route.Source, "rules", Prompt);
  }
  // Every floor-raised prompt in the floor's own table stays camera_depth on the rule path too.
  for (const Prompt of ["Grab the eraser", "Move left 3 cm", "Open the gripper", "Is the gripper open?", "x=10, y=20, z=5", "continue",
    "Explain grippers and then open it", "Pick up the red cup and put it in the basket"]) {
    Ollama({ Answer: () => Json({}, 500) });
    Assert.equal((await new AiPromptRouter().Route(Prompt, Signal())).Mode, "camera_depth", Prompt);
  }
  for (const Context of [{ CheckIn: true }, { Answer: true }, { TaskOngoing: true }]) {
    Ollama({ Answer: () => Json({}, 500) });
    Assert.equal((await new AiPromptRouter().Route("What is inverse kinematics?", Signal(), Context)).Mode, "camera_depth", JSON.stringify(Context));
  }
});

Test("the router never loads Nimble where it would make Ollama evict the main model", async () => {
  const Qwen = { name: "qwen3.8:27b", size_vram: 17.5 * Gb }, Other = { name: "qwen2.5:7b", size_vram: 5 * Gb };
  // qwen3.8:27b alone: 17.5 + 1.1 x 9.5 = 27.95 GB fits the 29 GB budget.
  Ollama({ Loaded: [Qwen] });
  Assert.equal((await new AiPromptRouter().Route("Explain recursion", Signal())).Mode, "text");
  // Another model loaded as well: 22.5 + 10.45 GB would not fit, so Nimble is not asked (not loaded).
  const Asked = Ollama({ Loaded: [Qwen, Other] });
  const Route = await new AiPromptRouter().Route("Explain recursion", Signal());
  Assert.equal(Route.Mode, "text"); Assert.equal(Route.Source, "rules", "the rule classifier answers instead");
  Assert.match(Route.Reason, /nimble \(10\.[45] GB\) would not fit beside the loaded models \(22\.5 GB\) in router_vram_budget_gb 29/);
  Assert.equal(Asked.length, 0);
  // Already loaded: asking it loads nothing, so the budget does not apply.
  Ollama({ Loaded: [Qwen, Other, { name: "nimble:latest", size_vram: 10 * Gb }] });
  Assert.equal((await new AiPromptRouter().Route("Explain recursion", Signal())).Mode, "text");
  // 0 turns the check off.
  PromptRouterOptions.VramBudgetGb = 0;
  try {
    Ollama({ Loaded: [Qwen, Other] });
    Assert.equal((await new AiPromptRouter().Route("Explain recursion", Signal())).Mode, "text");
  } finally { PromptRouterOptions.VramBudgetGb = Defaults.VramBudgetGb; }
});

Test("the AI panel's status line and the session record say which route, who decided and how long it took", async () => {
  Ollama({ Answer: () => Json(Nimble("camera", 0.96)) });
  const Router = new AiPromptRouter();
  const Camera = await Router.Route("What colour is the cup?", Signal());
  Assert.match(RouteStatusText(Camera), /^Prompt \+ camera \(Nimble, \d+ ms\): Nimble: the camera image answers it, no measurements needed$/);
  const Record = RouteRecord(Camera);
  Assert.deepEqual(Object.keys(Record), ["route", "mode", "source", "reason", "floor", "nimble_choice", "probabilities", "duration_ms", "router_model", "sends"]);
  Assert.equal(Record.route, "camera"); Assert.equal(Record.source, "nimble"); Assert.equal(Record.floor, "text");
  Assert.equal(Record.nimble_choice, "camera"); Assert.equal(Record.router_model, "nimble");
  Assert.equal(Record.probabilities.camera, 0.96);
  Assert.ok(Number.isInteger(Record.duration_ms) && Record.duration_ms >= 0);
  Assert.deepEqual(Record.sends, { image: true, measured_positions: false, depth_image: false, task: false });
  Assert.deepEqual(JSON.parse(JSON.stringify(Record)), Record, "the record is plain JSON");

  const Safety = await Router.Route("Grab the eraser", Signal());
  Assert.match(RouteStatusText(Safety), /^Prompt \+ camera \+ depth \(safety rule, \d+ ms\): Acts on an object/);
  Assert.deepEqual(RouteRecord(Safety).sends, { image: true, measured_positions: true, depth_image: true, task: true });
  Assert.equal(RouteRecord(Safety).nimble_choice, null);

  Ollama({ Answer: () => Json({}, 500) });
  const Failed = await new AiPromptRouter().Route("Tell me something", Signal());
  Assert.match(RouteStatusText(Failed), /^Prompt \+ camera \+ depth \(fallback, \d+ ms\): Nimble unavailable \(HTTP 500\); using camera and measured positions$/);
  Assert.equal(RouteRecord(Failed).source, "fallback");
  Ollama({ Answer: () => Json({}, 500) });
  const Ruled = await new AiPromptRouter().Route("Explain recursion", Signal());
  Assert.match(RouteStatusText(Ruled), /^Prompt only \(rules, \d+ ms\): Nimble unavailable \(HTTP 500\); rules: a general question or writing/);
  Assert.equal(RouteRecord(Ruled).source, "rules"); Assert.equal(RouteRecord(Ruled).route, "text");
  Assert.deepEqual(RouteRecord(Ruled).sends, { image: false, measured_positions: false, depth_image: false, task: false });

  Ollama({ Answer: () => Json(Nimble("text", 0.99)) });
  const Text = await new AiPromptRouter().Route("Explain recursion", Signal());
  Assert.match(RouteStatusText(Text), /^Prompt only \(Nimble, \d+ ms\)/);
  Assert.deepEqual(RouteRecord(Text).sends, { image: false, measured_positions: false, depth_image: false, task: false });
});
