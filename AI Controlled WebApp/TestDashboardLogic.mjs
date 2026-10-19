import { build as Build } from "esbuild";
import { test as Test } from "node:test";
import Assert from "node:assert/strict";
const Import = async File => {
  const Bundle = await Build({ entryPoints: [File], bundle: true, write: false, platform: "node", format: "esm" });
  return import(`data:text/javascript;base64,${Buffer.from(Bundle.outputFiles[0].text).toString("base64")}`);
};
const Layout = await Import("src/lib/DashboardLayout.ts");
const Usage = await Import("src/lib/AiStatistics.ts");
Test("old camera column width is reclaimed by the workspace", () => {
  globalThis.localStorage = { getItem: () => "[0.21,0.43,0.18,0.18]" };
  Assert.deepEqual(Layout.ReadColumnWeights("Manual"), [.64, .18, .18]);
});
Test("column fitting and pair resizing preserve space and minimum widths", () => {
  for (const Mode of ["Manual", "Ai"]) {
    const Minimums = Layout.ColumnMinimums[Mode];
    for (let Trial = 1; Trial <= 250; Trial++) {
      const Width = 400 + Trial * 13;
      const Weights = Minimums.map((_, Index) => ((Trial * (Index + 3)) % 31 + 1) / 32);
      const Fitted = Layout.FitColumnWidths(Width, Weights, Minimums);
      const Sum = Values => Values.reduce((Total, Value) => Total + Value, 0);
      Assert.ok(Math.abs(Sum(Fitted) - Math.max(Width, Sum(Minimums))) < 1e-8);
      for (let Index = 0; Index < Minimums.length - 1; Index++) {
        for (const Delta of [-Infinity, -300, 40, 800, Infinity]) {
          const Next = Layout.ResizeColumnPair(Fitted, Minimums, Index, Delta);
          Assert.ok(Next.every((Value, At) => Value >= Minimums[At] - 1e-8));
          Assert.ok(Math.abs(Sum(Next) - Sum(Fitted)) < 1e-8);
          Assert.ok(Math.abs(Sum(Layout.ColumnWeights(Next)) - 1) < 1e-12);
          for (let At = 0; At < Next.length; At++) if (At !== Index && At !== Index + 1) Assert.equal(Next[At], Fitted[At]);
        }
      }
    }
  }
});
Test("corrupt or unavailable local storage does not break the dashboard", () => {
  for (const Stored of ["invalid", "null", "[]", "[1,0,0,0]", "[1e308,1,1,1]"]) {
    globalThis.localStorage = { getItem: () => Stored, setItem: () => { throw Error("Storage unavailable"); } };
    Assert.deepEqual(Layout.ReadColumnWeights("Manual"), Layout.DefaultColumnWeights.Manual);
    Assert.equal(Layout.ReadDashboardMode(), "Manual");
    Assert.doesNotThrow(() => Layout.SaveDashboardSetting("Mode", "Ai"));
  }
});
Test("usage counts are reported data, including zeros; missing values stay unknown", () => {
  const First = Usage.ReadAiUsage({ usage: { prompt_tokens: 120, completion_tokens: 30 } }, 1500, "test");
  Assert.equal(First.TotalTokens, 150);
  const Zero = Usage.ReadAiUsage({ usage: { prompt_tokens: 0, completion_tokens: 0, total_tokens: 0 } }, 10, "test");
  Assert.equal(Zero.TotalTokens, 0);
  for (const Value of [-1, 1.5, "20", Infinity, null]) {
    const Unknown = Usage.ReadAiUsage({ usage: { prompt_tokens: Value, completion_tokens: Value, total_tokens: Value } }, 10, "test");
    Assert.equal(Unknown.TotalTokens, null);
    Assert.equal(Unknown.InputTokens, null);
  }
  // Ollama's native /api/chat reports its counts at the top level.
  const Native = Usage.ReadAiUsage({ message: { content: "{}" }, prompt_eval_count: 5354, eval_count: 88 }, 900, "test");
  Assert.deepEqual([Native.InputTokens, Native.OutputTokens, Native.TotalTokens], [5354, 88, 5442]);
  const Missing = Usage.ReadAiUsage({}, 2000, "test");
  let Stats = Usage.AddAiUsage(Usage.EmptyAiStatistics, First);
  Stats = Usage.AddAiUsage(Stats, Missing);
  Stats = Usage.AddAiUsage(Stats, First);
  Assert.equal(Stats.Responses, 3);
  Assert.equal(Stats.ReportedTokens, 300);
  Assert.equal(Stats.UnreportedResponses, 1);
  Assert.equal(Stats.Last.DurationMs, 1500);
});

Test("AI layout removes the right pane and migrates saved widths", () => {
  globalThis.localStorage = { getItem: () => "[0.31,0.48,0.21]" };
  Assert.deepEqual(Layout.ReadColumnWeights("Ai"), [.31, .69]);
  Assert.equal(Layout.ColumnMinimums.Ai.length, 2);
  globalThis.localStorage = { getItem: () => "[0.35,0.65]" };
  Assert.deepEqual(Layout.ReadColumnWeights("Ai"), [.35, .65]);
});
