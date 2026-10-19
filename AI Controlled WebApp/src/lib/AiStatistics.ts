export type AiUsage = { Model: string; InputTokens: number | null; OutputTokens: number | null; TotalTokens: number | null; DurationMs: number };
export type AiStatistics = { Responses: number; Failures: number; ReportedTokens: number | null; UnreportedResponses: number; Last: AiUsage | null };
export const EmptyAiStatistics: AiStatistics = { Responses: 0, Failures: 0, ReportedTokens: null, UnreportedResponses: 0, Last: null };

// Token counts from a reply: Ollama's native /api/chat puts them at the top level
// (prompt_eval_count, eval_count); the OpenAI-compatible /v1 in usage.
export function ReadAiUsage(Data: Record<string, unknown>, DurationMs: number, Model: string): AiUsage {
  const Usage = Data.usage !== null && typeof Data.usage === "object" ? Data.usage as Record<string, unknown> : {};
  const Count = (Value: unknown) => typeof Value === "number" && Number.isSafeInteger(Value) && Value >= 0 ? Value : null;
  const InputTokens = Count(Usage.prompt_tokens) ?? Count(Data.prompt_eval_count), OutputTokens = Count(Usage.completion_tokens) ?? Count(Data.eval_count);
  const TotalTokens = Count(Usage.total_tokens) ?? (InputTokens !== null && OutputTokens !== null ? Count(InputTokens + OutputTokens) : null);
  return { Model, InputTokens, OutputTokens, TotalTokens, DurationMs: Math.max(0, DurationMs) };
}

export function AddAiUsage(Previous: AiStatistics, Usage: AiUsage): AiStatistics {
  return { ...Previous, Responses: Previous.Responses + 1, Last: Usage,
    ReportedTokens: Usage.TotalTokens === null ? Previous.ReportedTokens : (Previous.ReportedTokens ?? 0) + Usage.TotalTokens,
    UnreportedResponses: Previous.UnreportedResponses + (Usage.TotalTokens === null ? 1 : 0) };
}
