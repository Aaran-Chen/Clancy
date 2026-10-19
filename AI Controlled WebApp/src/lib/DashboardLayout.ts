export type DashboardMode = "Manual" | "Ai";
export const DashboardStorageKey = "SO101.Dashboard.v1";
// AI mode's left column holds two panel columns side by side: the AI conversation
// with its approval card, and Current Position, Motion and the statistics.
export const ColumnMinimums = { Manual: [320, 230, 260], Ai: [600, 320] };
export const DefaultColumnWeights = { Manual: [.64, .18, .18], Ai: [.3, .7] };
// Below this scale the panels would be too small to read; they scroll instead.
export const MinimumFitScale = .8;

export function ReadDashboardSetting(Key: string): unknown {
  try { return JSON.parse(localStorage.getItem(`${DashboardStorageKey}.${Key}`) ?? "null"); }
  catch { return null; }
}

export function SaveDashboardSetting(Key: string, Value: unknown) {
  try { localStorage.setItem(`${DashboardStorageKey}.${Key}`, JSON.stringify(Value)); }
  catch { /* Private browsing or full storage must not prevent layout changes. */ }
}

export function ReadDashboardMode(): DashboardMode {
  return ReadDashboardSetting("Mode") === "Ai" ? "Ai" : "Manual";
}

export function ReadColumnWeights(Mode: DashboardMode): number[] {
  return ReadSavedColumnWeights(Mode) ?? [...DefaultColumnWeights[Mode]];
}

// The widths the user set by dragging, or null for the default tiling.
export function ReadSavedColumnWeights(Mode: DashboardMode): number[] | null {
  const Saved = ReadDashboardSetting(`${Mode}.Widths`);
  if (Mode === "Ai" && Array.isArray(Saved) && Saved.length === 3
    && Saved.every(Value => typeof Value === "number" && Number.isFinite(Value) && Value > 0 && Value <= 1)) {
    // Keep the left column's width and return the former right pane to the CAD view.
    return ColumnWeights([Saved[0], Saved[1] + Saved[2]]);
  }
  if (Mode === "Manual" && Array.isArray(Saved) && Saved.length === 4
    && Saved.every(Value => typeof Value === "number" && Number.isFinite(Value) && Value > 0 && Value <= 1)) {
    return ColumnWeights([Saved[0] + Saved[1], Saved[2], Saved[3]]);
  }
  return Array.isArray(Saved) && Saved.length === ColumnMinimums[Mode].length
    && Saved.every(Value => typeof Value === "number" && Number.isFinite(Value) && Value > 0 && Value <= 1)
    ? Saved : null;
}

export function FitColumnWidths(Available: number, Weights: number[], Minimums: number[]): number[] {
  const Total = Math.max(Available, Minimums.reduce((Sum, Value) => Sum + Value, 0));
  const Widths = Minimums.map(() => 0);
  let Remaining = Total;
  let Free = Minimums.map((_, Index) => Index);
  while (Free.length) {
    const Weight = Free.reduce((Sum, Index) => Sum + Weights[Index], 0);
    const Clamped = Free.filter(Index => Remaining * Weights[Index] / Weight < Minimums[Index]);
    if (!Clamped.length) {
      for (const Index of Free) Widths[Index] = Remaining * Weights[Index] / Weight;
      break;
    }
    for (const Index of Clamped) { Widths[Index] = Minimums[Index]; Remaining -= Widths[Index]; }
    Free = Free.filter(Index => !Clamped.includes(Index));
  }
  return Widths;
}

export function ResizeColumnPair(Widths: number[], Minimums: number[], Index: number, Delta: number): number[] {
  const Next = [...Widths];
  const Shift = Math.max(Minimums[Index] - Widths[Index], Math.min(Delta, Widths[Index + 1] - Minimums[Index + 1]));
  Next[Index] += Shift;
  Next[Index + 1] -= Shift;
  return Next;
}

export function ColumnWeights(Widths: number[]): number[] {
  const Total = Widths.reduce((Sum, Value) => Sum + Value, 0);
  return Widths.map(Value => Value / Total);
}

// AI mode's default split for this window: the AI column gets the width its two
// panel columns need to show the heaviest card at this height without scrolling
// (a short window needs a wider conversation, so its text wraps onto fewer
// lines), and at least 36% of the width; the workspace keeps the rest.
export function DefaultAiWeights(Available: number, Height: number): number[] {
  const Conversation = Math.max(390, Math.min(480, 480 - (Height - 640) * .45));
  const Panels = Math.max(260, Math.min(330, Available * .16));
  const Wanted = Math.max(Conversation + Panels + 32, Available * .36);
  const Ai = Math.max(ColumnMinimums.Ai[0], Math.min(Available - ColumnMinimums.Ai[1], Wanted));
  return ColumnWeights([Ai, Math.max(1, Available - Ai)]);
}
