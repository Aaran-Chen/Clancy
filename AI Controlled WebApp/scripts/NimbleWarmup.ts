// The dev server warms Nimble (the prompt router) when it starts and whenever Ollama comes (back) up, so the page's
// first prompt does not meet a cold router (35 s from disk, measured 2026-10-12). Same checks as the page
// (src/lib/NimbleStatus.ts, repeated below): Ollama 0.35+, the model installed, and the VRAM budget, so it never makes Ollama evict
// the main model. Its keep_alive is router_startup_keep_alive (shorter than the page's router_keep_alive): a dev
// server nobody opens frees the VRAM again. SO101_ROUTER_WARMUP=0 turns it off (the UI test servers do).
// The page's own check is src/lib/NimbleStatus.ts; this project (tsconfig.node.json) cannot import src, so the
// check is repeated here and TestNimbleWarmup.mjs holds both to the same verdicts.

const RecordOf = (Value: unknown): Record<string, unknown> | null => Value !== null && typeof Value === "object" && !Array.isArray(Value)
  ? Value as Record<string, unknown> : null;

/** src/lib/NimbleStatus.ts ReadNimbleStatus, for the dev server. */
export function ServerNimbleStatus(Version: unknown, Tags: unknown, Ps: unknown, Model: string, BudgetGb: number): { Ready: boolean; Reason: string; Loaded?: boolean } {
  const Text = RecordOf(Version)?.version;
  const Parts = typeof Text === "string" ? /^(\d+)\.(\d+)\./.exec(Text) : null;
  if (!Parts || (Number(Parts[1]) === 0 && Number(Parts[2]) < 35)) return { Ready: false, Reason: "Nimble needs Ollama 0.35 or newer" };
  const Normalize = (Name: unknown) => typeof Name === "string" ? Name.replace(/:latest$/, "") : null;
  const IsRouter = (Entry: unknown) => Normalize(RecordOf(Entry)?.name) === Normalize(Model);
  const Models = RecordOf(Tags)?.models;
  const Installed = Array.isArray(Models) ? Models.find(IsRouter) : undefined;
  if (!Installed) return { Ready: false, Reason: `Install ${Model} in Ollama to enable fast routing` };
  const Listed = RecordOf(Ps)?.models;
  const Running = Array.isArray(Listed) ? Listed : [];
  const Mine = Running.find(IsRouter);
  if (BudgetGb > 0 && !Mine) {
    const InUse = Running.reduce((Sum: number, Entry) => Sum + (Number(RecordOf(Entry)?.size_vram) || 0), 0);
    const Needs = (Number(RecordOf(Installed)?.size) || 0) * 1.1;
    if ((InUse + Needs) / 1e9 > BudgetGb) {
      return { Ready: false, Loaded: false, Reason: `${Model} (${(Needs / 1e9).toFixed(1)} GB) would not fit beside the loaded models (${(InUse / 1e9).toFixed(1)} GB) in router_vram_budget_gb ${BudgetGb}` };
    }
  }
  return { Ready: true, Reason: "Nimble ready", Loaded: Boolean(Mine) };
}

// A one-word typed choice: enough to load Nimble (the page's real question is src/lib/NimbleStatus.ts NimbleRouteBody).
function WarmBody(Model: string, KeepAlive: string) {
  return { model: Model, keep_alive: KeepAlive, state: { user_prompt: "Hello" },
    questions: { input_mode: { type: "choice", instructions: "Route a user request in a robotic-arm assistant.",
      criteria: { text: "Conversation only.", camera: "Needs the camera image.", camera_depth: "Needs measured positions or a physical action." } } } };
}

export type WarmOptions = { Enabled: boolean; Model: string; KeepAlive: string; BudgetGb: number; TimeoutMs: number };
export type WarmResult = { ok: boolean; status: string };

export function WarmOptionsFrom(Saved: Record<string, unknown>): WarmOptions {
  const Budget = Number(Saved.router_vram_budget_gb);
  const Timeout = Number(Saved.router_warmup_timeout_ms);
  return {
    Enabled: Saved.router_enabled !== false && process.env.SO101_ROUTER_WARMUP !== "0",
    Model: typeof Saved.router_model === "string" && Saved.router_model.trim() ? Saved.router_model.trim() : "nimble",
    KeepAlive: String(Saved.router_startup_keep_alive ?? "30m"),
    BudgetGb: Number.isFinite(Budget) && Budget >= 0 ? Budget : 29,
    TimeoutMs: Number.isFinite(Timeout) && Timeout > 0 ? Math.min(60_000, Timeout) : 60_000,
  };
}

async function Json(Url: string, Init: RequestInit, TimeoutMs: number): Promise<unknown> {
  const Response = await fetch(Url, { ...Init, signal: AbortSignal.timeout(TimeoutMs) });
  if (!Response.ok) throw new Error(`HTTP ${Response.status} from ${new URL(Url).pathname}`);
  return Response.json();
}

/** One warm-up: the checks, then a one-word decision that loads Nimble. Never throws. */
export async function WarmNimble(BaseUrl: string, Options: WarmOptions): Promise<WarmResult> {
  if (!Options.Enabled) return { ok: false, status: "off" };
  try {
    const [Version, Tags, Ps] = await Promise.all(["/api/version", "/api/tags", "/api/ps"].map(Path => Json(`${BaseUrl}${Path}`, {}, 3000)));
    const Status = ServerNimbleStatus(Version, Tags, Ps, Options.Model, Options.BudgetGb);
    if (!Status.Ready) return { ok: false, status: Status.Reason };
    if (Status.Loaded) return { ok: true, status: "already-loaded" };
    await Json(`${BaseUrl}/v1/systemone`, { method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify(WarmBody(Options.Model, Options.KeepAlive)) }, Options.TimeoutMs);
    return { ok: true, status: "warmed" };
  } catch (Failure) {
    return { ok: false, status: Failure instanceof Error ? Failure.message : String(Failure) };
  }
}

/**
 * Warm now, then every IntervalMs see whether Ollama answers: when it was down and is up again (restarted, or
 * started late), warm again. One warm-up at a time. Check runs the same check at once (after the dev server started
 * Ollama itself); Stop ends it. The timer never keeps Node alive.
 */
export function StartNimbleWarmer(BaseUrl: string, Options: WarmOptions, IsRunning: () => Promise<boolean>,
  { IntervalMs = 30_000, Log = (Text: string) => console.log(Text) } = {}): { Check: () => Promise<void>; Stop: () => void } {
  if (!Options.Enabled) return { Check: async () => {}, Stop: () => {} };
  let Up: boolean | null = null, Busy: Promise<void> | null = null, Stopped = false;
  const Warm = (Why: string) => {
    Busy ??= WarmNimble(BaseUrl, Options).then(Result => {
      if (!Stopped) Log(`[router] ${Why}: ${Options.Model} ${Result.ok ? Result.status : `not warmed (${Result.status})`}`);
    }).finally(() => { Busy = null; });
    return Busy;
  };
  const Check = async () => {
    const Now = await IsRunning().catch(() => false);
    if (Stopped) return;
    const Was = Up;
    Up = Now;
    if (Now && Was !== true) await Warm(Was === null ? "dev server start" : "Ollama is up again");
  };
  void Check();
  const Timer = setInterval(() => void Check(), IntervalMs);
  Timer.unref?.();
  return { Check, Stop: () => { Stopped = true; clearInterval(Timer); } };
}
