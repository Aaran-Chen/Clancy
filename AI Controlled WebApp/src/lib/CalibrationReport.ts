// What a camera calibration's fit report means for the user (/selfcal/solve: back end/SelfCalibration.py
// and bridge_server.py). Every run that computes a camera is saved and applied: what used to reject a
// fit (fit or hold-out error, depth disagreement, camera motion during the run, no camera-check
// baseline, too few poses or clicks) is a warning ("problems") shown with its numbers, and only the
// user rejects a calibration (Reject calibration: /selfcal/reject restores the one it replaced). A run
// that computed no camera at all keeps the current calibration. Shared by the calibration panel
// (App.tsx) and the semi-automatic click stage (components/SemiCalibrationStage.tsx).

export type CalibrationFitReport = {
  ok: boolean;
  // false: no camera could be computed (the current calibration stays). Missing from bridges before 2026-10-10.
  computed?: boolean;
  saved?: boolean;
  // The quality problems that used to reject the fit, now warnings; empty when every check passed.
  problems?: string[];
  quality_ok?: boolean;
  // What to do about the problems, in one sentence.
  quality_warning?: string | null;
  warnings?: string[];
  error?: string;
  fitted_at?: string | null;
  can_reject?: boolean;
  drift_baseline?: boolean;
  // Set by the app once the user rejected this calibration.
  rejected?: boolean;
};

export type CalibrationOutcomeKind = "saved" | "saved_with_warnings" | "not_saved" | "not_computed" | "rejected";

export type CalibrationOutcome = {
  Kind: CalibrationOutcomeKind;
  // The report's heading.
  Title: string;
  // The error, or what to do about the problems.
  Detail: string | null;
  // Each quality problem, shown as a warning with its numbers.
  Problems: string[];
  // The run's outcome as /selfcal/finish and the session record keep it.
  RunOutcome: string;
};

export function CalibrationOutcomeOf(Report: CalibrationFitReport): CalibrationOutcome {
  const Problems = Report.problems ?? [];
  if (Report.rejected) {
    return { Kind: "rejected", Title: "Rejected by you: the calibration it replaced was restored.", Detail: null, Problems, RunOutcome: "rejected by the user" };
  }
  if (Report.saved) {
    return Problems.length
      ? { Kind: "saved_with_warnings", Title: "Saved and applied, with warnings.", Detail: Report.quality_warning ?? null, Problems,
        RunOutcome: "calibrated with warnings" }
      : { Kind: "saved", Title: "Calibrated and saved.", Detail: null, Problems, RunOutcome: "calibrated" };
  }
  // A bridge from before this change still refuses fits with problems: it needs a restart.
  if (Report.computed === undefined && !Report.ok && Problems.length) {
    return { Kind: "not_saved", Title: "Not saved: this bridge still rejects calibrations. Restart the bridge to load the update.",
      Detail: Report.error ?? null, Problems, RunOutcome: "not saved (old bridge)" };
  }
  if (Report.computed || (Report.ok && Report.computed !== false)) {
    return { Kind: "not_saved", Title: "Computed but not saved: the current calibration is still in use.", Detail: Report.error ?? null,
      Problems, RunOutcome: "computed, not saved" };
  }
  return { Kind: "not_computed", Title: "Could not compute a calibration: the current calibration is still in use.",
    Detail: Report.error ?? null, Problems: [], RunOutcome: "not computed" };
}

// "14:02" from the bridge's fitted_at ("2026-10-10T14:02:33"), else the text as given.
export function FittedClock(FittedAt: string | null | undefined): string | null {
  if (!FittedAt) return null;
  return FittedAt.match(/(?:^|\D)(\d{1,2}:\d{2})/)?.[1] ?? FittedAt;
}

// The bridge's answer to /selfcal/reject.
export type CalibrationRejectResult = {
  ok: boolean; error?: string; rejected_fitted_at?: string | null; restored_fitted_at?: string | null;
  restored_mode?: "auto" | "manual" | null; calibrated?: boolean; drift_baseline?: boolean;
};

// The sentence shown and logged after the user rejected a calibration.
export function RejectedText(Result: CalibrationRejectResult): string {
  const Rejected = FittedClock(Result.rejected_fitted_at);
  const Restored = FittedClock(Result.restored_fitted_at);
  const What = `Calibration${Rejected ? ` fitted at ${Rejected}` : ""} rejected`;
  if (!Result.calibrated) return `${What}: there was no calibration before it, so the camera is not calibrated now.`;
  const Which = Result.restored_mode === "manual" ? "the manual (probe) calibration" : "the calibration";
  const Check = Result.restored_mode === "auto" && Result.drift_baseline === false
    ? " It has no camera-check baseline, so tasks that act on an object stay blocked until a calibration with one is saved." : "";
  return `${What}: ${Which}${Restored ? ` fitted at ${Restored}` : ""} is in use again.${Check}`;
}

// The confirmation asked before rejecting.
export function RejectQuestion(FittedAt: string | null | undefined, ReplacedFittedAt: string | null | undefined): string {
  const Current = FittedClock(FittedAt), Previous = FittedClock(ReplacedFittedAt);
  return `Reject the calibration${Current ? ` fitted at ${Current}` : ""}? The calibration it replaced`
    + `${Previous ? ` (fitted at ${Previous})` : ""} is restored and used again.`;
}
