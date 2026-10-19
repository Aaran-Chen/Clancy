import type { AiStatistics } from "../lib/AiStatistics";

type AiStatisticsPanelProps = { Statistics: AiStatistics; Model: string | null; ContextLength: number | null; Busy: boolean };
const FormatCount = (Value: number | null | undefined) => Value == null ? "--" : Value.toLocaleString();

export default function AiStatisticsPanel({ Statistics, Model, ContextLength, Busy }: AiStatisticsPanelProps) {
  const Last = Statistics.Last;
  return (
    <div className="ai-statistics" aria-label="AI usage statistics">
      <div className="ai-model-stat"><strong>{Model ?? "No model selected"}</strong><span>{Busy ? "Request in progress" : Last ? "Last response" : "No responses yet"}</span></div>
      <div className="ai-stat-grid">
        <div className="ai-stat-tile" data-stat="input"><span>Input tokens</span><strong>{FormatCount(Last?.InputTokens)}</strong></div>
        <div className="ai-stat-tile" data-stat="output"><span>Output tokens</span><strong>{FormatCount(Last?.OutputTokens)}</strong></div>
        <div className="ai-stat-tile" data-stat="total"><span>Total tokens</span><strong>{FormatCount(Last?.TotalTokens)}</strong></div>
        <div className="ai-stat-tile" data-stat="time"><span>Request time</span><strong>{Last ? `${(Last.DurationMs / 1000).toFixed(2)} s` : "--"}</strong></div>
      </div>
      <div className="ai-context-meter">
        <span>Context window: {ContextLength ? `${FormatCount(ContextLength)} tokens` : "not reported"}</span>
        {ContextLength !== null && ContextLength > 0 && Last?.TotalTokens != null
          ? <progress aria-label="Last response context usage" value={Math.min(Last.TotalTokens, ContextLength)} max={ContextLength} /> : null}
      </div>
      <div className="ai-stat-meta"><span>Responses: {Statistics.Responses}</span><span>Failed requests: {Statistics.Failures}</span></div>
      <div className="ai-stat-meta"><span>Reported session tokens</span><strong data-stat="session">{FormatCount(Statistics.ReportedTokens)}</strong></div>
      <p>{Statistics.UnreportedResponses > 0 ? `${Statistics.UnreportedResponses} response(s) did not report token usage. ` : ""}Counts update after each response, including revisions. Session totals reset when the page reloads.</p>
    </div>
  );
}
