import SavedOptions from "../../../back end/OllamaOptions.json";

// Every model call goes through Ollama's native /api/chat (proxied by Vite at
// /ollama) with the num_ctx, num_batch and keep_alive in back end/OllamaOptions.json,
// which the bridge's grounding (object_locator.py) reads too. The OpenAI-compatible
// /v1 endpoint ignored num_ctx, so the model loaded at the desktop app's 262144
// default and left under 1 GiB of VRAM: any other GPU user then slowed each step
// 20-100x. A caller with a different num_ctx or num_batch reloads the model
// (8-15 s), so the app, its warm-up and the bridge must all send the same two.
export const OllamaChatUrl = "/ollama/api/chat";
const Whole = (Value: unknown, Fallback: number) => Number.isFinite(Number(Value)) && Number(Value) > 0 ? Math.trunc(Number(Value)) : Fallback;
export const OllamaOptions = {
  NumCtx: Whole(SavedOptions.num_ctx, 32768),
  NumBatch: Whole(SavedOptions.num_batch, 1024),
  KeepAlive: String(SavedOptions.keep_alive ?? "30m"),
};
// The model the app tries first (OllamaOptions.json "model"); it falls back to the largest vision model.
export const PreferredModel = typeof SavedOptions.model === "string" && SavedOptions.model.trim() ? SavedOptions.model.trim() : null;
// The answer budget (num_predict). Replies are short JSON with thinking off: the
// largest of 42 recorded replies was 288 tokens. The old max_tokens followed the
// loaded context (253952 at 262k), so a runaway reply could run for 20+ minutes.
// If thinking is ever turned back on, this must grow or replies are cut off.
export const AiMaxOutputTokens = 1024;
export const AiTemperature = 0.2;

export type ChatContentPart = { type: "text"; text: string } | { type: "image_url"; image_url: { url: string } };
export type ChatMessage = { role: "system" | "user" | "assistant"; content: string | ChatContentPart[] };
export type OllamaMessage = { role: ChatMessage["role"]; content: string; images?: string[] };

// Each content part becomes its own message of the same role, in order: text as
// {content}, an image as {content: "", images: [base64 without the data: prefix]}.
// This is how the /v1 endpoint split them, and it reproduced its token counts
// exactly (1103, 5354, 5519). One merged message puts the images before the text
// (1098 tokens), and on recorded frames qwen then replied with malformed JSON.
export function ToOllamaMessages(Messages: ChatMessage[]): OllamaMessage[] {
  return Messages.flatMap(({ role, content }): OllamaMessage[] => typeof content === "string" ? [{ role, content }]
    : content.map(Part => Part.type === "text" ? { role, content: Part.text }
      : { role, content: "", images: [Part.image_url.url.slice(Part.image_url.url.indexOf(",") + 1)] }));
}

// think:false: without it qwen3 reasoned for 258 hidden tokens before answering
// (the /v1 endpoint's reasoning_effort "none" has no native equivalent).
export function BuildOllamaChatBody(Messages: ChatMessage[], Model: string, MaxTokens = AiMaxOutputTokens) {
  return {
    model: Model,
    messages: ToOllamaMessages(Messages),
    stream: false,
    think: false,
    keep_alive: OllamaOptions.KeepAlive,
    options: { num_ctx: OllamaOptions.NumCtx, num_batch: OllamaOptions.NumBatch, temperature: AiTemperature,
      num_predict: Math.max(1, Math.min(AiMaxOutputTokens, Math.floor(MaxTokens) || AiMaxOutputTokens)) },
  };
}

// The warm-up: no messages, so Ollama only loads the model (done_reason "load",
// ~20 ms when it is already loaded) with the same load options as every call.
export function BuildOllamaPreloadBody(Model: string) {
  return { model: Model, messages: [], stream: false, keep_alive: OllamaOptions.KeepAlive,
    options: { num_ctx: OllamaOptions.NumCtx, num_batch: OllamaOptions.NumBatch } };
}

// The reply's text. A reply cut off by the budget is an error, not a reply: its
// JSON is incomplete, and nothing read from it may become a motion.
export function ReadOllamaChatReply(Data: unknown, MaxTokens = AiMaxOutputTokens): string {
  const Record_ = Data !== null && typeof Data === "object" ? Data as Record<string, unknown> : null;
  const Message = Record_?.message !== null && typeof Record_?.message === "object" ? Record_.message as Record<string, unknown> : null;
  if (!Record_ || !Message) throw new Error("AI model returned an unreadable response");
  if (typeof Message.content !== "string") throw new Error("AI model returned no text content");
  const DoneReason = typeof Record_.done_reason === "string" ? Record_.done_reason : null;
  const Budget = Math.max(1, Math.min(AiMaxOutputTokens, Math.floor(MaxTokens) || AiMaxOutputTokens));
  if (DoneReason === "length") throw new Error(`AI model's reply was cut off at the ${Budget}-token limit (done_reason length).`);
  const Content = Message.content.trim();
  if (!Content) throw new Error(`AI model returned empty text content${DoneReason ? ` (${DoneReason})` : ""}.`);
  return Content;
}

// Ollama's keep_alive as milliseconds ("30m", "90s", "1h", seconds as a number;
// negative keeps the model loaded forever). Unreadable values get Ollama's own
// default of 5 minutes.
export function KeepAliveMs(Value: string | number): number {
  if (typeof Value === "number") return Value < 0 ? Infinity : Value * 1000;
  const Match = /^(-?\d+(?:\.\d+)?)(ms|s|m|h)?$/.exec(Value.trim());
  if (!Match) return 5 * 60_000;
  const Amount = Number(Match[1]);
  return Amount < 0 ? Infinity : Amount * { ms: 1, s: 1000, m: 60_000, h: 3_600_000 }[(Match[2] ?? "s") as "ms" | "s" | "m" | "h"];
}

// A typed prompt warms the model again once it has been idle this long: five
// minutes before keep_alive would unload it (at least a minute).
export function RewarmAfterMs(KeepAlive: string | number = OllamaOptions.KeepAlive): number {
  return Math.max(60_000, KeepAliveMs(KeepAlive) - 5 * 60_000);
}
