import { ReadDashboardSetting, SaveDashboardSetting } from "./DashboardLayout";
import { GraspRules, type GraspRefusal } from "./GraspPlan";

// THE GRASP GAP (Settings > Grasp gap at the fixed finger): how far outside the object the GRASP
// PLAN puts the fixed jaw's pad (the planner's c, GraspContext.ClearanceCm), saved in this browser.
// "It is still not getting close enough" (2026-10-01 18:56): the user sets it. Every check of a plan
// runs with it as before, so a gap too small for the gripper's margins is refused with the reason.
// Below the default it applies only to a round object's plan (a can, a bottle: GraspPlan.ts
// ClearanceFor), the one plan that corrects for where the real fixed finger stands and checks the
// way down with it nearer still; any other object keeps the default c, its only room for that.
// A task plans with the gap set when it was sent (App.tsx), so a change never re-plans a grasp
// whose object the gripper hides.
// MinCm: the planner keeps every gripper part PlanMarginCm from the object, the fixed pad too, so
// a smaller gap could never plan (0.3 and 0.4 were refused for every object, 2026-10-01 review).
export const GraspGapRange = { MinCm: GraspRules.PlanMarginCm, MaxCm: 1.5, StepCm: 0.1 };
export const DefaultGraspGapCm = GraspRules.FixedJawClearanceCm;
const GraspGapKey = "GraspGapCm";

/** A saved or entered gap: within the range, on its 0.1 cm steps; anything else is the default. */
export function NormalizeGraspGap(Value: unknown): number {
  if (typeof Value !== "number" || !Number.isFinite(Value)) return DefaultGraspGapCm;
  const Clamped = Math.max(GraspGapRange.MinCm, Math.min(GraspGapRange.MaxCm, Value));
  return Number((Math.round(Clamped / GraspGapRange.StepCm) * GraspGapRange.StepCm).toFixed(1));
}

export function ReadGraspGap(): number {
  return NormalizeGraspGap(ReadDashboardSetting(GraspGapKey));
}

export function SaveGraspGap(Cm: number) {
  SaveDashboardSetting(GraspGapKey, NormalizeGraspGap(Cm));
}

/**
 * What a refusal adds about the grasp gap, where it is part of why: the pad
 * placed below the default (GapCm, a round object's plan) and a gripper part
 * too near the object on the way down or at the grasp point ("path", "palm"):
 * a wider gap moves the pad away from it, so it may plan it; placed above the
 * default and too wide for the jaws ("wide"): the moving jaw must close the
 * farther the wider the gap (not that a smaller one plans it: the jaws may
 * not hold it at the default either). Null otherwise (too thin, too flat,
 * unmeasured, out of reach, a neighbour, no pad placed), and at the default.
 */
export function GraspGapHint(Refusal: Pick<GraspRefusal, "Code" | "GapCm">): { Text: string; Advice: string } | null {
  const Cm = Refusal.GapCm;
  if (Cm == null) return null;
  const Setting = `the grasp gap at the fixed finger is set to ${Cm.toFixed(1)} cm in Settings (the default is ${DefaultGraspGapCm.toFixed(1)} cm)`;
  if (Cm < DefaultGraspGapCm - 1e-9 && (Refusal.Code === "path" || Refusal.Code === "palm")) {
    return { Text: `${Setting}: a wider gap may plan it`, Advice: `Or widen the grasp gap: ${Setting}: a wider gap may plan it.` };
  }
  if (Cm > DefaultGraspGapCm + 1e-9 && Refusal.Code === "wide") {
    const Why = "the wider the gap, the farther the moving jaw must close";
    return { Text: `${Setting}: ${Why}`, Advice: `The ${Setting.slice(4)}: ${Why}.` };
  }
  return null;
}
