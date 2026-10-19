import SavedOptions from "../../../back end/OllamaOptions.json";
import { TaskNeedsPositions } from "./AiMeasurements";
import { KeepAliveMs, type ChatMessage } from "./OllamaChat";
import { NimbleRouteBody, ReadNimbleStatus, type NimbleStatus } from "./NimbleStatus";

const Whole = (Value: unknown, Fallback: number, Max: number) => Number.isFinite(Number(Value)) && Number(Value) > 0
  ? Math.min(Max, Math.trunc(Number(Value))) : Fallback;
const Budget = Number(SavedOptions.router_vram_budget_gb);
export const PromptRouterOptions = {
  Enabled: SavedOptions.router_enabled !== false,
  Model: SavedOptions.router_model?.trim() || "nimble",
  // The whole per-prompt budget (availability check and decision). Never above the first router's 1500 ms:
  // routing must not make a prompt slower than sending everything straight away.
  TimeoutMs: Whole(SavedOptions.router_timeout_ms, 800, 1500),
  WarmupTimeoutMs: Whole(SavedOptions.router_warmup_timeout_ms, 30000, 60000),
  // How long Nimble stays loaded after the page's last use of it (each decision and background re-warm renews it).
  // The page re-warms it only while someone uses the page, so it unloads at most this long after the last use.
  KeepAlive: String(SavedOptions.router_keep_alive ?? "2h"),
  // The dev server's start-up warm-up (scripts/NimbleWarmup.ts), before any page has opened: shorter, so a dev server
  // nobody opens frees Nimble's ~9.5 GB again soon. The page's first warm-up then extends it to KeepAlive.
  StartupKeepAlive: String((SavedOptions as { router_startup_keep_alive?: string }).router_startup_keep_alive ?? "30m"),
  // GPU memory (GB) the loaded models may fill before the router declines to load its model, so loading it
  // never makes Ollama evict the main model (32 GB card: qwen3.8:27b ~17.5 GB + nimble ~9.5 GB). 0 turns it off.
  VramBudgetGb: Number.isFinite(Budget) && Budget >= 0 ? Budget : 29,
};
export const CaptureCountdownSeconds = Math.max(0, Math.min(10, Math.trunc(Number(SavedOptions.capture_countdown_seconds) || 0)));
export const NimbleDecisionUrl = "/ollama/v1/systemone";

// What the main model gets, fewest inputs first. Only camera_depth is the task path: the arm-still wait, the
// capture with /locate's calibrated measurements and the camera gate, the depth map, the task state and motion
// proposals with approval. text and camera are answer-only: nothing they reply is parsed as a motion.
export const RouteModes = ["text", "camera", "camera_depth"] as const;
export type RouteMode = typeof RouteModes[number];
export const RouteWork: Record<RouteMode, { Image: boolean; Locate: boolean; DepthImage: boolean; Task: boolean }> = {
  text: { Image: false, Locate: false, DepthImage: false, Task: false },
  camera: { Image: true, Locate: false, DepthImage: false, Task: false },
  camera_depth: { Image: true, Locate: true, DepthImage: true, Task: true },
};
export const RouteLabels: Record<RouteMode, string> = { text: "Prompt only", camera: "Prompt + camera", camera_depth: "Prompt + camera + depth" };
const SourceLabels = { nimble: "Nimble", safety: "safety rule", rules: "rules", fallback: "fallback" } as const;
const Rank = (Mode: RouteMode) => RouteModes.indexOf(Mode);
const AtLeast = (Mode: RouteMode, Floor: RouteMode): RouteMode => Rank(Mode) >= Rank(Floor) ? Mode : Floor;
export type PromptRoute = { Mode: RouteMode; Source: keyof typeof SourceLabels; Reason: string; DurationMs: number;
  // The safety floor the prompt had, and Nimble's own answer when it was asked.
  Floor: RouteMode; Choice: RouteMode | null; Probabilities: Record<RouteMode, number> | null };
// What the app knows besides the prompt: an automatic check-in, an answer to the model's question, or a task
// (or any arm motion) still under way when the prompt was sent.
export type RouteContext = { CheckIn?: boolean; Answer?: boolean; TaskOngoing?: boolean };
type RouterStatus = NimbleStatus;
// Mode: the route (never below the floor); Own: what Nimble's probabilities alone give; Choice: its top answer.
type NimbleDecision = { Mode: RouteMode; Own: RouteMode; Choice: RouteMode; Probabilities: Record<RouteMode, number> };
const RecordOf = (Value: unknown): Record<string, unknown> | null => Value !== null && typeof Value === "object" && !Array.isArray(Value)
  ? Value as Record<string, unknown> : null;

// Teaching questions often mention motion vocabulary. Let Nimble distinguish those from requests to act; a
// second imperative or a live-scene reference still sets the floor.
const Conceptual = /^\s*(?:please\s+)?(?:explain\b|describe how\b|how (?:does|do|would|can)\b|what (?:is|are)\b|tell me about\b|why\b)/i;
const ActAfter = /\b(?:then|and|also|now)\s+(?:please\s+)?(?:do it|move|grab|pick|open|close|turn|lift|lower|go|start)\b/i;
const LiveScene = /\b(?:right now|currently|in (?:this|the) (?:image|photo)|on (?:my|this|the) (?:desk|table)|can you see)\b/i;
const MotionWords = /\b(move|grab|grasp|pick|place|put|lift|lower|rotate|turn|reach|push|pull|slide|nudge|release|open|close|stop|hold|home|continue|proceed|again)\b/i;
// The arm's state is in the task path's prompt (tip, joints, gripper), not in a photo.
const ArmWords = /\b(gripper|servo|joint|wrist)\b/i;
const Measures = /\b[xyz]\s*[:=]\s*-?\d|\b\d+(?:\.\d+)?\s*(?:cm|mm|degrees?)\b/i;
const SceneWords = /\b(camera|photo|picture|image|scene|workspace|desk|table|look|see|visible|in front of|next to|on top of)\b/i;

/**
 * The fewest inputs a prompt may get, whatever Nimble says. Anything that could lead to motion (a check-in, an
 * answer to the model's question, a task or motion under way, an object to act on, motion words, arm state,
 * coordinates) gets camera_depth: the task path with fresh measurements. These rules can only add inputs.
 */
export function PromptRouteFloor(Prompt: string, Context: RouteContext = {}): { Mode: RouteMode; Reason: string } {
  const Depth = (Reason: string) => ({ Mode: "camera_depth" as const, Reason });
  if (Context.CheckIn) return Depth("Task check-in: fresh camera and measured positions");
  if (Context.Answer) return Depth("Answer to the AI's question: the task goes on with fresh measurements");
  if (Context.TaskOngoing) return Depth("A task or motion is under way: fresh camera and measured positions");
  if (TaskNeedsPositions(Prompt)) return Depth("Acts on an object: fresh camera and measured positions");
  // Do not truncate away a physical instruction at the end of a long prompt.
  if (Prompt.length > 8000) return Depth("Long request: keeping camera and measured positions");
  const Teaching = Conceptual.test(Prompt) && !ActAfter.test(Prompt) && !LiveScene.test(Prompt);
  if (!Teaching && (MotionWords.test(Prompt) || ArmWords.test(Prompt) || Measures.test(Prompt))) {
    return Depth("Physical request or arm state: fresh camera and measured positions");
  }
  if (!Teaching && (SceneWords.test(Prompt) || LiveScene.test(Prompt))) return { Mode: "camera", Reason: "Asks about the live scene: at least the camera image" };
  return { Mode: "text", Reason: "No physical action or scene reference" };
}

// The rule classifier's own signals, for when Nimble cannot answer (cold, warming, slow, missing, failed). Only
// prompts these clearly cover get a lower route; anything else keeps camera_depth, as before.
const Greeting = /^\s*(?:hi|hello|hey|thanks|thank you|good (?:morning|afternoon|evening|night)|bye|goodbye)\b[\s\w,!.']{0,40}$/i;
const TextAsk = /^\s*(?:please\s+)?(?:(?:can|could) you\s+)?(?:explain\b|describe how\b|how (?:does|do|would|can|is|are)\b|what (?:is|are|does)\b|what's\b|tell me about\b|why\b|who\b|when (?:was|did|is)\b|define\b|calculate\b|compute\b|convert\b|solve\b|translate\b|summari[sz]e\b|write\b|draft\b|compose\b|rephrase\b|proofread\b|give me (?:a|an|some) (?:example|joke|summary|definition|idea|list)s?\b)/i;
// A reference only the scene can resolve, or a thing on the desk: never text by rule.
const Deictic = /\b(?:it|its|this|these|those|them|here|there)\b|\bthat\s*[?.!]*\s*$/i;
const PhysicalNouns = /\b(?:objects?|items?|things?|stuff|cups?|mugs?|(?:the|a|that|this|soda|tin|drink)\s+can\b|cans|bottles?|box(?:es)?|blocks?|cubes?|balls?|pens?|pencils?|erasers?|markers?|tape|toys?|cards?|coins?|lids?|baskets?|bowls?|plates?|spoons?|books?|phones?|cables?|wires?|batter(?:y|ies)|screws?|tools?|bins?|trays?|shelf|shelves|drawers?|cabinets?|arms?|robots?|claws?|motors?)\b/i;
const Colours = /\b(?:colou?rs?|red|green|blue|yellow|orange|purple|pink|black|white|brown|gr[ae]y)\b/i;
// What the camera alone cannot answer: where things are, how far, how big.
const PositionWords = /\b(?:where|how far|far|distance|closest|nearest|farthest|furthest|closer|position|located|location|size|how (?:big|tall|wide|long|high)|height|width|length|measure|reach(?:able)?|between|left|right|above|below|behind|under)\b/i;

/**
 * The route a prompt clearly needs, by rule, or null when the rules cannot tell (then camera_depth). Used only when
 * Nimble cannot answer. Never below the floor: a camera_depth floor gives camera_depth; text only where the floor
 * found no scene reference and the prompt is a clear question, explanation or piece of writing with no scene, arm,
 * object, colour, position or ambiguous reference; camera only where the floor found a live-scene question with no
 * position, distance or size in it.
 */
export function RuleRoute(Prompt: string, Floor: RouteMode): { Mode: RouteMode; Reason: string } | null {
  if (Floor === "camera_depth") return { Mode: "camera_depth", Reason: "rules: the safety rule needs camera and measured positions" };
  const Physical = MotionWords.test(Prompt) || ArmWords.test(Prompt) || Measures.test(Prompt) || PhysicalNouns.test(Prompt);
  if (Floor === "camera") {
    if (PositionWords.test(Prompt) || Measures.test(Prompt) || MotionWords.test(Prompt) || ArmWords.test(Prompt)) return null;
    return { Mode: "camera", Reason: "rules: asks what the camera shows, no positions or sizes" };
  }
  if (Physical || SceneWords.test(Prompt) || LiveScene.test(Prompt) || Colours.test(Prompt) || PositionWords.test(Prompt)) return null;
  if (Greeting.test(Prompt)) return { Mode: "text", Reason: "rules: a greeting, no scene, arm or object words" };
  if (Deictic.test(Prompt)) return null;
  if (TextAsk.test(Prompt)) return { Mode: "text", Reason: "rules: a general question or writing, no scene, arm or object words" };
  return null;
}

export function BuildNimbleRouteBody(Prompt: string, KeepAlive = PromptRouterOptions.KeepAlive) {
  return NimbleRouteBody(Prompt, PromptRouterOptions.Model, KeepAlive);
}

// The margin a lowered route needs: an operational margin, not an accuracy guarantee.
export const LowerMargin = 0.9;

/**
 * Nimble's answer as a route, never below Floor. The route is the fewest inputs that cover the request with at
 * least LowerMargin of the probability (a prompt answerable from text alone is answerable with the camera too),
 * so uncertain answers keep more inputs. Throws on anything malformed: the caller falls back to camera_depth.
 */
export function ReadNimbleRoute(Data: unknown, Floor: RouteMode = "text"): NimbleDecision {
  const Answer = RecordOf(RecordOf(RecordOf(Data)?.answers)?.input_mode);
  const Given = RecordOf(Answer?.probabilities);
  if (Answer?.type !== "choice" || !RouteModes.includes(Answer.choice as RouteMode)) throw new Error("Nimble returned an invalid routing decision");
  const Values = RouteModes.map(Mode => Given?.[Mode]);
  if (Values.some(Value => typeof Value !== "number" || !Number.isFinite(Value) || Value < 0 || Value > 1)
      || Math.abs((Values as number[]).reduce((Sum, Value) => Sum + Value, 0) - 1) > 0.02) throw new Error("Nimble returned invalid routing probabilities");
  const Probabilities = Object.fromEntries(RouteModes.map((Mode, Index) => [Mode, Values[Index] as number])) as Record<RouteMode, number>;
  let Covered = 0;
  const Own = RouteModes.find(Candidate => (Covered += Probabilities[Candidate]) >= LowerMargin) ?? "camera_depth";
  return { Mode: AtLeast(Own, Floor), Own, Choice: Answer.choice as RouteMode, Probabilities };
}

async function WithDeadline<T>(Run: (Signal: AbortSignal) => Promise<T>, Parent: AbortSignal, Ms: number): Promise<T> {
  Parent.throwIfAborted();
  const Controller = new AbortController();
  const Abort = () => Controller.abort(Parent.reason);
  Parent.addEventListener("abort", Abort, { once: true });
  let Timer: ReturnType<typeof setTimeout> | undefined;
  let RejectAbort: (() => void) | undefined;
  try {
    return await Promise.race([
      Run(Controller.signal),
      new Promise<never>((_, Reject) => {
        RejectAbort = () => Reject(Controller.signal.reason);
        Controller.signal.addEventListener("abort", RejectAbort, { once: true });
        Timer = setTimeout(() => Controller.abort(new Error("Nimble routing timed out")), Ms);
      }),
    ]);
  } finally {
    clearTimeout(Timer);
    Parent.removeEventListener("abort", Abort);
    if (RejectAbort) Controller.signal.removeEventListener("abort", RejectAbort);
  }
}

const NimbleReasons: Record<RouteMode, string> = {
  text: "Nimble: the prompt alone answers it, no camera needed",
  camera: "Nimble: the camera image answers it, no measurements needed",
  camera_depth: "Nimble: camera and measured positions needed",
};

// KeepWarm warms Nimble again when its load would run out within this.
export const RewarmMarginMs = 5 * 60_000;
// The page keeps Nimble warm only within this long of someone's last key, click, scroll or pointer move (App.tsx).
export const RouterActiveMs = 15 * 60_000;

export class AiPromptRouter {
  private Status: RouterStatus | null = null;
  private CheckedAt = 0;
  private Warmup: { Signal: AbortSignal; Promise: Promise<RouterStatus> } | null = null;
  private UsedAt = 0;
  // Background warm-ups started by a prompt or KeepWarm: never tied to one request, so stopping it cannot cancel them.
  private readonly Background = new AbortController();

  private async Probe(Signal: AbortSignal): Promise<RouterStatus> {
    if (!PromptRouterOptions.Enabled) return { Ready: false, Reason: "Nimble routing disabled" };
    if (this.Status && Date.now() - this.CheckedAt < 30000) return this.Status;
    const Status = await WithDeadline(async ProbeSignal => {
      const [VersionResponse, TagsResponse, LoadedResponse] = await Promise.all([
        fetch("/ollama/api/version", { signal: ProbeSignal }), fetch("/ollama/api/tags", { signal: ProbeSignal }),
        fetch("/ollama/api/ps", { signal: ProbeSignal }),
      ]);
      if (!VersionResponse.ok || !TagsResponse.ok || !LoadedResponse.ok) throw new Error("Nimble availability check failed");
      // The version check, the installed model and the VRAM budget (lib/NimbleStatus.ts).
      return ReadNimbleStatus(await VersionResponse.json(), await TagsResponse.json(), await LoadedResponse.json(),
        PromptRouterOptions.Model, PromptRouterOptions.VramBudgetGb);
    }, Signal, PromptRouterOptions.TimeoutMs);
    this.Status = Status; this.CheckedAt = Date.now();
    return Status;
  }

  private async Decide(Prompt: string, Floor: RouteMode, Signal: AbortSignal, TimeoutMs: number): Promise<NimbleDecision> {
    return WithDeadline(async RequestSignal => {
      const Response = await fetch(NimbleDecisionUrl, { method: "POST", headers: { "Content-Type": "application/json" },
        signal: RequestSignal, body: JSON.stringify(BuildNimbleRouteBody(Prompt)) });
      if (!Response.ok) throw new Error(`Nimble unavailable (HTTP ${Response.status})`);
      return ReadNimbleRoute(await Response.json(), Floor);
    }, Signal, TimeoutMs);
  }

  // Cold loading has its own budget and runs ahead of user prompts. A prompt
  // arriving during warm-up takes the full inputs immediately instead of queuing on it.
  Prepare(Signal: AbortSignal): Promise<RouterStatus> {
    if (this.Warmup && !this.Warmup.Signal.aborted) return this.Warmup.Promise;
    const Warmup = { Signal, Promise: Promise.resolve<RouterStatus>({ Ready: false, Reason: "Nimble is warming up" }) };
    this.Warmup = Warmup;
    Warmup.Promise = (async () => {
      try {
        const Status = await this.Probe(Signal);
        if (!Status.Ready) return Status;
        await this.Decide("Hello", "text", Signal, PromptRouterOptions.WarmupTimeoutMs);
        Signal.throwIfAborted();
        this.UsedAt = Date.now();
        this.Status = { Ready: true, Reason: "Nimble ready", Loaded: true, ExpiresAt: this.UsedAt + KeepAliveMs(PromptRouterOptions.KeepAlive) };
        this.CheckedAt = this.UsedAt;
        return this.Status;
      } catch (Error_) {
        if (Signal.aborted) throw Error_;
        this.Status = { Ready: false, Reason: Error_ instanceof Error ? Error_.message : "Nimble unavailable" };
        this.CheckedAt = Date.now();
        return this.Status;
      } finally { if (this.Warmup === Warmup) this.Warmup = null; }
    })();
    return Warmup.Promise;
  }

  /**
   * The page's keep-warm tick (App.tsx calls it every minute while the page is in use). A fresh /api/ps check, so an
   * Ollama restart or an eviction is seen; when Nimble is not loaded, or its load runs out within RewarmMarginMs,
   * it is warmed again in the background (within the VRAM budget, as always). True when a warm-up was started.
   */
  async KeepWarm(Signal: AbortSignal): Promise<boolean> {
    if (!PromptRouterOptions.Enabled || (this.Warmup && !this.Warmup.Signal.aborted)) return false;
    this.Status = null;
    let Status: RouterStatus;
    try { Status = await this.Probe(Signal); } catch { if (Signal.aborted) return false; this.Status = null; return false; }
    if (!Status.Ready) return false;
    const Ours = this.UsedAt ? this.UsedAt + KeepAliveMs(PromptRouterOptions.KeepAlive) : 0;
    // Ollama's own expiry when it gives one (another client may have shortened it), else this page's last use.
    const Until = Status.Loaded ? Status.ExpiresAt ?? (Ours || Infinity) : 0;
    if (Status.Loaded && Until - Date.now() > RewarmMarginMs) return false;
    if (!Status.Loaded) this.UsedAt = 0;
    void this.Prepare(this.Background.signal).catch(() => {});
    return true;
  }

  async Route(Prompt: string, Signal: AbortSignal, Context: RouteContext = {}): Promise<PromptRoute> {
    Signal.throwIfAborted();
    const Started = performance.now();
    const Floor = PromptRouteFloor(Prompt, Context);
    const Result = (Mode: RouteMode, Source: PromptRoute["Source"], Reason: string, Decision: NimbleDecision | null = null): PromptRoute => ({
      Mode, Source, Reason, DurationMs: performance.now() - Started, Floor: Floor.Mode, Choice: Decision?.Choice ?? null, Probabilities: Decision?.Probabilities ?? null });
    // Nimble cannot answer in time: the rule classifier (RuleRoute, never below the floor) for prompts it clearly
    // covers, else today's full path.
    const Fallback = (Reason: string) => {
      const Rule = RuleRoute(Prompt, Floor.Mode);
      return Rule ? Result(Rule.Mode, "rules", `${Reason}; ${Rule.Reason}`)
        : Result("camera_depth", "fallback", `${Reason}; using camera and measured positions`);
    };
    if (Floor.Mode === "camera_depth") return Result("camera_depth", "safety", Floor.Reason);
    if (this.Warmup && !this.Warmup.Signal.aborted) return Fallback("Nimble is warming up");
    if (this.UsedAt && Date.now() - this.UsedAt >= Math.max(1000, KeepAliveMs(PromptRouterOptions.KeepAlive) - 30000)) {
      this.UsedAt = 0;
      void this.Prepare(this.Background.signal).catch(() => {});
      return Fallback("Nimble is warming up after idle");
    }
    try {
      const Status = await this.Probe(Signal);
      Signal.throwIfAborted();
      if (!Status.Ready) return Fallback(Status.Reason);
      const Decision = await this.Decide(Prompt, Floor.Mode, Signal, Math.max(1, PromptRouterOptions.TimeoutMs - (performance.now() - Started)));
      Signal.throwIfAborted();
      this.UsedAt = Date.now();
      // The floor raised what Nimble would have sent: the floor decided.
      if (Decision.Mode !== Decision.Own) {
        return Result(Decision.Mode, "safety", `Nimble chose ${RouteLabels[Decision.Own].toLowerCase()}; ${Floor.Reason}`, Decision);
      }
      return Result(Decision.Mode, "nimble", Decision.Mode === Decision.Choice ? NimbleReasons[Decision.Mode]
        : `${NimbleReasons[Decision.Mode]} (it leaned to ${RouteLabels[Decision.Choice].toLowerCase()}, below the ${LowerMargin} margin)`, Decision);
    } catch (Error_) {
      if (Signal.aborted) throw Error_;
      const Reason = Error_ instanceof Error ? Error_.message : "Nimble unavailable";
      this.Status = { Ready: false, Reason }; this.CheckedAt = Date.now();
      // Eviction by another GPU model can make an otherwise-ready Nimble cold.
      // Recover off the prompt's critical path with the longer loading budget.
      if (Reason === "Nimble routing timed out") {
        this.Status = null;
        void this.Prepare(this.Background.signal).catch(() => {});
      }
      return Fallback(Reason);
    }
  }
}

/** The AI panel's line for a route: what the main model gets, who decided, how long it took, and why. */
export function RouteStatusText(Route: PromptRoute): string {
  return `${RouteLabels[Route.Mode]} (${SourceLabels[Route.Source]}, ${Math.round(Route.DurationMs)} ms): ${Route.Reason}`;
}

/** The session record's prompt_route event (ai.jsonl). */
export function RouteRecord(Route: PromptRoute, Model = PromptRouterOptions.Model) {
  const Work = RouteWork[Route.Mode];
  return { route: Route.Mode, mode: Route.Mode, source: Route.Source, reason: Route.Reason, floor: Route.Floor,
    nimble_choice: Route.Choice, probabilities: Route.Probabilities, duration_ms: Math.round(Route.DurationMs), router_model: Model,
    sends: { image: Work.Image, measured_positions: Work.Locate, depth_image: Work.DepthImage, task: Work.Task } };
}

export function BuildTextOnlyMessages(Prompt: string): ChatMessage[] {
  return [
    { role: "system", content: "You are a concise, helpful assistant in a robotic-arm app. This is conversation only: you have no camera images, live arm state, or ability to act. Answer the user's question in plain text. Do not invent observations, produce motion proposals, update the robot notepad, or claim any movement was performed. If live workspace information or physical action is needed, ask the user to send that explicit request." },
    { role: "user", content: Prompt },
  ];
}

/**
 * The camera route's request: the prompt and one colour frame, answered in plain text. No measured positions,
 * depth, arm state or ability to act. ImageUrl null: no frame could be captured, and the model says so.
 * ViewNote: which way the image's top and bottom run (the task prompt's AI_VISION_DEPTHWISE line).
 */
export function BuildCameraMessages(Prompt: string, ImageUrl: string | null, ViewNote: string): ChatMessage[] {
  const Rules = "Answer the user's question in plain text. Do not give positions in centimetres, produce motion proposals, update the robot notepad, or claim any movement was performed. If measured positions or physical action are needed, ask the user to send that explicit request.";
  if (!ImageUrl) {
    return [
      { role: "system", content: `You are a concise, helpful assistant in a robotic-arm app. The workspace camera could not take a photo this time, so you cannot see the scene, and you have no measured positions, live arm state, or ability to act. Say so if the question needs the photo. ${Rules}` },
      { role: "user", content: Prompt },
    ];
  }
  return [
    { role: "system", content: `You are a concise, helpful assistant in a robotic-arm app. You are given one live colour photo of the workspace from the fixed camera, and no measured positions, depth, live arm state, or ability to act. An object toward the LEFT side of the photo is on the arm's left. ${ViewNote} Describe only what the photo shows. ${Rules}` },
    { role: "user", content: [{ type: "text", text: Prompt }, { type: "image_url", image_url: { url: ImageUrl } }] },
  ];
}

/**
 * The answer-only routes: the prompt (text), or the prompt and one frame from Capture (camera), asked once.
 * They are given no way to wait for the arm, measure (/locate), make a depth map, start a task or read a motion:
 * the reply is shown as plain text. camera_depth is the task path, which the caller runs itself.
 */
export async function AnswerRoute(Mode: Exclude<RouteMode, "camera_depth">, Prompt: string, Signal: AbortSignal, Steps: {
  Capture: () => Promise<string | null>; Ask: (Messages: ChatMessage[]) => Promise<string>; ViewNote: string }): Promise<{ Reply: string; Image: boolean }> {
  Signal.throwIfAborted();
  const Frame = Mode === "camera" ? await Steps.Capture() : null;
  Signal.throwIfAborted();
  const Reply = await Steps.Ask(Mode === "camera" ? BuildCameraMessages(Prompt, Frame, Steps.ViewNote) : BuildTextOnlyMessages(Prompt));
  return { Reply, Image: Boolean(Frame) };
}
