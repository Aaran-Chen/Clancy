// Semi-automatic camera calibration ("click the tape"): the arm visits the
// automatic run's poses with the same plan, motion and stillness checks, and
// at each pose the app freezes the captured frame and the user clicks the
// centre of the red tape on the fixed jaw (back end/SelfCalClicks.py). This
// store connects the run loop in App.tsx (which moves the arm and waits for a
// decision) with the click stage (components/SemiCalibrationStage.tsx).

import { CalibrationOutcomeOf, type CalibrationFitReport } from "./CalibrationReport";

// The fewest clicks from which the bridge can compute a camera at all (SelfCalibration.MinSolvableObservations).
// Fewer than SemiCalRecommendedClicks (SelfCalibration.MinObservations) still fits and is saved, with a warning.
export const SemiCalMinClicks = 6;
export const SemiCalRecommendedClicks = 10;
// The arm holds each pose while it waits for a click. After this long without a
// decision the run ends (as Done early does) instead of visiting further poses; the
// arm then holds where it is, as at the end of an automatic run, until the user moves
// it. Frames already captured stay clickable and can be fitted.
export const SemiCalWaitLimitMs = 5 * 60 * 1000;
// The bridge coasts every motor at 60 C (MotorProtection.md); holding loaded poses for
// minutes heats the shoulder, so the run stops visiting poses at this temperature (the
// bridge's own Hold re-arm limit) and asks the user to rest the arm. It does not move the
// arm itself: until the user does, it holds, and the bridge still coasts at 60 C.
export const SemiCalHotC = 55;
// Retake: wait this long, then the pose must pass the settle checks again before the recapture.
export const SemiCalRetakePauseMs = 600;
// A click this soon after the frame on screen changed is refused (it was aimed at the frame before).
export const SemiCalClickSettleMs = 350;
export const SemiCalTargetText = "the centre of the red tape on the fixed jaw";

export type SemiCalStatus = "pending" | "clicked" | "skipped";
export type SemiCalDecision = "clicked" | "skipped" | "retake" | "done";
export type SemiCalPhase = "idle" | "moving" | "waiting" | "retaking" | "fitting" | "review" | "stopped";
export type SemiCalPoseResult = {
  pose_index: number | null; observed_px: number[]; predicted_px: number[]; residual_px: number; residual_cm: number;
  depth_error_cm: number | null; dropped: boolean; flagged: boolean; reason: string | null;
};
export type SemiCalReport = CalibrationFitReport & {
  fit_rms_px?: number; fit_rms_cm?: number;
  holdout_rms_cm?: number | null; depth_rms_cm?: number | null; plane_rms_cm?: number | null; observations_used?: number;
  observations_dropped?: number; tape_offset_cm?: number[]; per_pose?: SemiCalPoseResult[]; observation_source?: string;
};
export type SemiCalFrame = {
  PoseIndex: number; Ordinal: number;
  // The frozen frame exactly as captured (capture orientation; Mirrored captures are shown flipped back).
  Image: string; FrameIndex: number; CaptureArchive: string; Mirrored: boolean; CapturedAt: number;
  Status: SemiCalStatus;
  // The recorded click: the clicked pixel's centre in continuous capture pixels.
  Click: { U: number; V: number } | null; DepthCm: number | null;
  Result: SemiCalPoseResult | null;
};
export type SemiCalSnapshot = {
  Open: boolean; Phase: SemiCalPhase; Total: number; Ordinal: number; Message: string;
  Frames: SemiCalFrame[]; Active: number | null; Waiting: number | null; WaitingSince: number | null;
  // The user is on the frame the run is at (not looking back at an earlier one): a new pose's
  // frame replaces the view only then, so a click aimed at an earlier frame never lands on it.
  Following: boolean;
  Report: SemiCalReport | null; DoneRequested: boolean;
  // Why the run ended early on its own (no click in time, a hot servo); kept until the next run.
  Notice: string | null;
};

const Empty: SemiCalSnapshot = { Open: false, Phase: "idle", Total: 0, Ordinal: 0, Message: "", Frames: [], Active: null, Following: true,
  Waiting: null, WaitingSince: null, Report: null, DoneRequested: false, Notice: null };

export function CountsOf(Frames: SemiCalFrame[]) {
  return { Clicked: Frames.filter(Frame => Frame.Status === "clicked").length, Skipped: Frames.filter(Frame => Frame.Status === "skipped").length,
    Pending: Frames.filter(Frame => Frame.Status === "pending").length, Flagged: Frames.filter(Frame => Frame.Result?.flagged).length };
}

// Per-pose fit results onto the frames they belong to; frames without a result lose an old one.
export function WithReport(Frames: SemiCalFrame[], Report: SemiCalReport | null): SemiCalFrame[] {
  const ByPose = new Map((Report?.per_pose ?? []).map(Row => [Row.pose_index, Row]));
  return Frames.map(Frame => ({ ...Frame, Result: Frame.Status === "clicked" ? ByPose.get(Frame.PoseIndex) ?? null : null }));
}

// After a click on a frame other than the one the arm waits at, show the waiting
// frame again, else the next frame that still needs attention.
export function NextActive(Snapshot: SemiCalSnapshot, After: number): number | null {
  if (Snapshot.Waiting !== null) return Snapshot.Waiting;
  const Order = Snapshot.Frames.map(Frame => Frame.PoseIndex);
  const Start = Math.max(0, Order.indexOf(After));
  const Needs = (Frame: SemiCalFrame) => Frame.Status === "pending" || Boolean(Frame.Result?.flagged);
  const Later = Snapshot.Frames.slice(Start + 1).find(Needs) ?? Snapshot.Frames.slice(0, Start).find(Needs);
  return Later?.PoseIndex ?? After;
}

export class SemiCalStore {
  private State: SemiCalSnapshot = Empty;
  private Listeners = new Set<() => void>();
  private Pending: SemiCalDecision | null = null;

  Subscribe = (Listener: () => void) => { this.Listeners.add(Listener); return () => { this.Listeners.delete(Listener); }; };
  Snapshot = () => this.State;
  private Set(Change: Partial<SemiCalSnapshot>) {
    this.State = { ...this.State, ...Change };
    this.Listeners.forEach(Listener => Listener());
  }
  private UpdateFrame(PoseIndex: number, Change: Partial<SemiCalFrame>) {
    this.Set({ Frames: this.State.Frames.map(Frame => Frame.PoseIndex === PoseIndex ? { ...Frame, ...Change } : Frame) });
  }

  Begin(Total: number) {
    this.Pending = null;
    this.State = { ...Empty, Open: true, Phase: "moving", Total, Message: "Moving to the first pose" };
    this.Listeners.forEach(Listener => Listener());
  }
  Moving(Ordinal: number) {
    this.Set({ Phase: "moving", Ordinal, Waiting: null, WaitingSince: null, Message: `Moving to pose ${Ordinal} of ${this.State.Total}` });
  }
  // A frozen frame for the pose the arm now holds; a retake replaces that pose's frame and click.
  AddFrame(Frame: Omit<SemiCalFrame, "Status" | "Click" | "DepthCm" | "Result">) {
    const Entry: SemiCalFrame = { ...Frame, Status: "pending", Click: null, DepthCm: null, Result: null };
    const Others = this.State.Frames.filter(Existing => Existing.PoseIndex !== Frame.PoseIndex);
    // Done early pressed during a retake still ends the run at this pose.
    this.Pending = this.State.DoneRequested ? "done" : null;
    this.Set({ Frames: [...Others, Entry].sort((A, B) => A.Ordinal - B.Ordinal),
      Active: this.State.Following || this.State.Active === null ? Frame.PoseIndex : this.State.Active, Waiting: Frame.PoseIndex,
      WaitingSince: Date.now(), Phase: "waiting", Ordinal: Frame.Ordinal,
      Message: `Pose ${Frame.Ordinal} of ${this.State.Total}: click ${SemiCalTargetText}` });
  }
  // The run loop takes the user's decision for the waiting pose (null while none).
  TakeDecision(): SemiCalDecision | null { const Decision = this.Pending; this.Pending = null; return Decision; }
  // The first decision for the held pose wins until the run takes it: a Retake pressed just after
  // a click must not replace that click (the bridge already recorded it on this capture).
  Decide(Decision: SemiCalDecision) {
    if (this.State.Waiting === null || this.Pending !== null) return;
    this.Pending = Decision;
    if (Decision === "retake") this.Set({ Phase: "retaking", Message: "Retaking this pose's frame" });
  }
  RequestDone() {
    this.Set({ DoneRequested: true, Message: "Finishing: no more poses; fitting the clicks" });
    // A click already taken for the held pose stands; the run then ends before the next move.
    if (this.State.Waiting !== null && this.Pending === null) this.Pending = "done";
  }
  DoneRequested() { return this.State.DoneRequested; }
  SetClick(PoseIndex: number, Click: { U: number; V: number }, DepthCm: number | null) {
    this.UpdateFrame(PoseIndex, { Status: "clicked", Click, DepthCm, Result: null });
  }
  SetSkipped(PoseIndex: number) { this.UpdateFrame(PoseIndex, { Status: "skipped", Click: null, DepthCm: null, Result: null }); }
  SetPending(PoseIndex: number) { this.UpdateFrame(PoseIndex, { Status: "pending", Click: null, DepthCm: null, Result: null }); }
  // The user picks a frame; looking back at an earlier one stops new frames taking over the view.
  SetActive(PoseIndex: number | null) {
    const Latest = this.State.Frames[this.State.Frames.length - 1]?.PoseIndex ?? null;
    this.Set({ Active: PoseIndex, Following: PoseIndex === null || PoseIndex === this.State.Waiting
      || (this.State.Waiting === null && PoseIndex === Latest) });
  }
  SetMessage(Message: string) { this.Set({ Message }); }
  SetNotice(Notice: string) { this.Set({ Notice }); }
  Fitting() { this.Set({ Phase: "fitting", Waiting: null, WaitingSince: null, Message: "Fitting the camera to your clicks" }); }
  SetReport(Report: SemiCalReport) {
    const Frames = WithReport(this.State.Frames, Report);
    const Flagged = Frames.find(Frame => Frame.Result?.flagged);
    const Outcome = CalibrationOutcomeOf(Report);
    this.Set({ Report, Frames, Phase: "review", Waiting: null, WaitingSince: null, Active: Flagged?.PoseIndex ?? this.State.Active,
      Message: Outcome.Kind === "saved" ? `Calibrated and saved from ${Report.observations_used ?? "?"} clicks`
        : Outcome.Kind === "saved_with_warnings" ? `Saved and applied from ${Report.observations_used ?? "?"} clicks, with ${Outcome.Problems.length} warning${Outcome.Problems.length === 1 ? "" : "s"}`
        : Outcome.Kind === "not_saved" ? "Not saved: the current calibration is still in use"
        : `Not computed: ${Report.error ?? "no camera could be fitted"}` });
  }
  // The user rejected the calibration this report saved: the one it replaced is in use again.
  MarkRejected(Message: string) {
    if (this.State.Report) this.Set({ Report: { ...this.State.Report, rejected: true }, Message });
  }
  End(Message: string) {
    this.Pending = null;
    const Phase = this.State.Report && this.State.Phase === "review" ? "review" : "stopped";
    this.Set({ Phase, Waiting: null, WaitingSince: null, Message: Phase === "review" ? this.State.Message : Message });
  }
  Close() { this.Pending = null; this.Set({ Open: false }); }
  // A new calibration run archives elsewhere: the old frames can no longer be clicked.
  Clear() { this.Pending = null; this.State = Empty; this.Listeners.forEach(Listener => Listener()); }
  // Show an ended run's frames again (clicks and fits still work until the next run starts).
  Reopen() { if (this.State.Frames.length) this.Set({ Open: true }); }
}

export const SemiCal = new SemiCalStore();
