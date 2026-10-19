import { useCallback, useEffect, useRef, useState, useSyncExternalStore, type MouseEvent as ReactMouseEvent } from "react";
import { Check, CornerUpLeft, Crosshair, OctagonAlert, RotateCcw, SkipForward, Square, X } from "lucide-react";
import HoverLoupe, { type LoupeMarker } from "./HoverLoupe";
import { ClickCapturePoint, ClientPointOf, ContentBox, FitOf, ImagePointAt, type SourcePoint } from "../lib/ImagePointer";
import {
  CountsOf, NextActive, SemiCal, SemiCalClickSettleMs, SemiCalHotC, SemiCalMinClicks, SemiCalRecommendedClicks, SemiCalTargetText,
  SemiCalWaitLimitMs, type SemiCalFrame,
} from "../lib/SemiCalibration";
import { CalibrationOutcomeOf } from "../lib/CalibrationReport";
import "./SemiCalibrationStage.css";

// The click stage of semi-automatic calibration: the frozen frame of the pose
// the arm holds, large, with the hover magnifier; the user clicks the centre of
// the red tape on the fixed jaw and the arm moves on. Clicks, undo and skip go
// to the bridge's data-only /selfcal/click endpoints; the run loop in App.tsx
// moves the arm (lib/SemiCalibration.ts carries the decisions between them).

type StageProps = {
  ApiUrl: string;
  Running: boolean;
  OnStop: () => void;
  OnEmergencyStop: () => void;
  // Fit the clicks (App.tsx fitSemiCalibration: /selfcal/solve with source "manual").
  OnFit: () => Promise<unknown>;
  // The user rejects the calibration the fit saved (App.tsx rejectCalibration: /selfcal/reject).
  OnReject: (FittedAt: string | null) => Promise<unknown>;
  // The hottest servo (protection telemetry), shown while the arm holds a pose.
  Hottest: number | null;
};

async function PostJson(Url: string, Body: unknown) {
  const Response = await fetch(Url, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(Body) });
  if (Response.status === 404) throw new Error("Restart the bridge to load semi-automatic calibration.");
  return await Response.json() as Record<string, unknown>;
}

// Where to click: the fixed jaw with its red tape, the target at the tape's centre.
function TargetSketch() {
  return <svg className="semical-sketch" viewBox="0 0 200 116" role="img" aria-label="Reference: click the centre of the red tape on the fixed jaw">
    <rect x="6" y="38" width="62" height="40" rx="3" fill="#3a3f48" stroke="#8a93a3" strokeWidth="1.2" />
    <path d="M68 44 L178 50 L178 64 L68 70 Z" fill="#d9d2c3" stroke="#8a93a3" strokeWidth="1.2" />
    <path d="M70 80 L150 94 L148 104 L70 92 Z" fill="#d9d2c366" stroke="#8a93a366" strokeWidth="1" strokeDasharray="3 2" />
    <rect x="112" y="47" width="32" height="20" rx="1.5" fill="#d8312f" />
    <circle cx="128" cy="57" r="7" fill="none" stroke="#5ff2ff" strokeWidth="1.6" />
    <path d="M128 44 V52 M128 62 V70 M115 57 H123 M133 57 H141" stroke="#5ff2ff" strokeWidth="1.6" />
    <text x="100" y="24" fill="#c4cbd7" fontSize="11" textAnchor="middle">fixed jaw, red tape</text>
    <text x="110" y="114" fill="#9099a8" fontSize="10" textAnchor="middle">moving jaw (not this)</text>
  </svg>;
}

function Clock(Milliseconds: number) {
  const Seconds = Math.max(0, Math.floor(Milliseconds / 1000));
  return `${Math.floor(Seconds / 60)}:${String(Seconds % 60).padStart(2, "0")}`;
}

// In the calibration panel: show an ended semi-automatic run's frames again (they can still be
// clicked and fitted until the next calibration run starts).
export function SemiCalReopenButton() {
  const State = useSyncExternalStore(SemiCal.Subscribe, SemiCal.Snapshot);
  if (State.Open || !State.Frames.length) return null;
  return <button type="button" onClick={() => SemiCal.Reopen()} title="Click, re-click or fit the last semi-automatic run's frames">
    Show the frozen frames ({CountsOf(State.Frames).Clicked} clicked)</button>;
}

export default function SemiCalibrationStage({ ApiUrl, Running, OnStop, OnEmergencyStop, OnFit, OnReject, Hottest }: StageProps) {
  const State = useSyncExternalStore(SemiCal.Subscribe, SemiCal.Snapshot);
  const ImageRef = useRef<HTMLImageElement>(null);
  const PointRef = useRef<SourcePoint | null>(null), HoverRef = useRef<SourcePoint | null>(null);
  const [Aim, SetAim] = useState<SourcePoint | null>(null);
  const [Busy, SetBusy] = useState(false);
  const [Note, SetNote] = useState<string | null>(null);
  const [Now, SetNow] = useState(Date.now());
  const [, SetLayout] = useState(0);
  const Frame = State.Frames.find(Candidate => Candidate.PoseIndex === State.Active) ?? null;
  const Counts = CountsOf(State.Frames);
  const IsWaitingFrame = Frame !== null && State.Waiting === Frame.PoseIndex;
  // The frame the arm holds is clickable only once its capture is in (not while it is retaken);
  // earlier frames stay clickable for a re-click, but nothing is clicked during a fit.
  const Clickable = Frame !== null && State.Phase !== "fitting"
    && !(Running && Frame.PoseIndex === State.Waiting && State.Phase !== "waiting");
  const ClickableRef = useRef(Clickable);
  ClickableRef.current = Clickable;

  useEffect(() => {
    if (State.WaitingSince === null) return;
    const Timer = window.setInterval(() => SetNow(Date.now()), 500);
    return () => window.clearInterval(Timer);
  }, [State.WaitingSince]);
  // When the frame on screen last changed: a click right after a change was aimed at the frame
  // before it (it would record the wrong pose, and on the frame the arm holds, move the arm on).
  const ShownAtRef = useRef(0);
  useEffect(() => { SetAim(null); SetNote(null); ShownAtRef.current = performance.now(); }, [State.Active, Frame?.Image]);
  // Markers drawn over the frame follow its layout.
  useEffect(() => {
    const Redraw = () => SetLayout(Value => Value + 1);
    window.addEventListener("resize", Redraw);
    return () => window.removeEventListener("resize", Redraw);
  }, []);

  const RecordClick = useCallback(async (Target: SemiCalFrame, Point: SourcePoint, Width: number, Height: number) => {
    if (Busy || !ClickableRef.current) return;
    if (performance.now() - ShownAtRef.current < SemiCalClickSettleMs) {
      SetNote("The frame just changed under the pointer: look at it, then click again.");
      return;
    }
    const Click = ClickCapturePoint(Point);
    SetBusy(true); SetNote(null);
    try {
      const Result = await PostJson(`${ApiUrl}/selfcal/click`, { pose_index: Target.PoseIndex, capture_archive: Target.CaptureArchive,
        frame_index: Target.FrameIndex, u: Click.U, v: Click.V, mirrored: Target.Mirrored, image_size: [Width, Height],
        clicked_at: Date.now() / 1000 });
      if (!Result.ok) { SetNote(String(Result.error ?? "The bridge refused the click.")); return; }
      SemiCal.SetClick(Target.PoseIndex, Click, typeof Result.depth_cm === "number" ? Result.depth_cm : null);
      if (SemiCal.Snapshot().Waiting === Target.PoseIndex) SemiCal.Decide("clicked");
      else SemiCal.SetActive(NextActive(SemiCal.Snapshot(), Target.PoseIndex));
    } catch (Failure) {
      SetNote(Failure instanceof Error ? Failure.message : String(Failure));
    } finally { SetBusy(false); }
  }, [ApiUrl, Busy]);

  const OnImageClick = (Event: ReactMouseEvent<HTMLImageElement>) => {
    if (!Frame) return;
    const Image = Event.currentTarget;
    const Point = PointRef.current ?? ImagePointAt(Image, Event.clientX, Event.clientY, Frame.Mirrored);
    if (Point) void RecordClick(Frame, Point, Image.naturalWidth, Image.naturalHeight);
  };

  // Arrow keys nudge the aimed pixel by one source pixel; Enter clicks it.
  useEffect(() => {
    if (!State.Open || !Frame) return;
    const Key = (Event: KeyboardEvent) => {
      const Image = ImageRef.current;
      // Keys meant for a focused control (Enter on a button) never also click the frame: that click moves the arm.
      if (!Image || (Event.target instanceof Element && Event.target.closest("button, input, textarea, select, a, [contenteditable]"))) return;
      const Base = Aim ?? HoverRef.current;
      // A held Enter repeats: only the press itself clicks (a repeat could confirm the next frame).
      if (Event.key === "Enter" && Event.repeat) { Event.preventDefault(); return; }
      if (Event.key === "Enter" && Base && PointRef.current) {
        Event.preventDefault();
        void RecordClick(Frame, Base, Image.naturalWidth, Image.naturalHeight);
        return;
      }
      const Step = { ArrowLeft: [-1, 0], ArrowRight: [1, 0], ArrowUp: [0, -1], ArrowDown: [0, 1] }[Event.key];
      if (!Step || !Base) return;
      Event.preventDefault();
      // Left and right as seen: a frame shown flipped back moves the other way in capture pixels.
      const Across = Frame.Mirrored ? -Step[0] : Step[0];
      const Column = Math.min(Image.naturalWidth - 1, Math.max(0, Base.Column + Across));
      const Row = Math.min(Image.naturalHeight - 1, Math.max(0, Base.Row + Step[1]));
      SetAim({ U: Column + 0.5, V: Row + 0.5, Column, Row });
    };
    window.addEventListener("keydown", Key);
    return () => window.removeEventListener("keydown", Key);
  }, [State.Open, Frame, Aim, RecordClick]);

  const Undo = async () => {
    SetBusy(true); SetNote(null);
    try {
      const Result = await PostJson(`${ApiUrl}/selfcal/click/undo`, {});
      if (!Result.ok) { SetNote(String(Result.error ?? "Undo failed.")); return; }
      if (typeof Result.pose_index === "number") { SemiCal.SetPending(Result.pose_index); SemiCal.SetActive(Result.pose_index); }
      else SetNote("Nothing to undo.");
    } catch (Failure) { SetNote(Failure instanceof Error ? Failure.message : String(Failure)); }
    finally { SetBusy(false); }
  };
  const Skip = async () => {
    if (!Frame) return;
    SetBusy(true); SetNote(null);
    try {
      const Result = await PostJson(`${ApiUrl}/selfcal/click/skip`, { pose_index: Frame.PoseIndex, reason: "skipped in the click stage" });
      if (!Result.ok) { SetNote(String(Result.error ?? "Skip failed.")); return; }
      SemiCal.SetSkipped(Frame.PoseIndex);
      if (SemiCal.Snapshot().Waiting === Frame.PoseIndex) SemiCal.Decide("skipped");
      else SemiCal.SetActive(NextActive(SemiCal.Snapshot(), Frame.PoseIndex));
    } catch (Failure) { SetNote(Failure instanceof Error ? Failure.message : String(Failure)); }
    finally { SetBusy(false); }
  };
  const Fit = async () => {
    SetBusy(true); SetNote(null);
    try { await OnFit(); } finally { SetBusy(false); }
  };
  const Reject = async () => {
    SetBusy(true); SetNote(null);
    try { await OnReject(State.Report?.fitted_at ?? null); } finally { SetBusy(false); }
  };

  if (!State.Open) return null;
  const Image = ImageRef.current;
  const Markers: LoupeMarker[] = [];
  const Overlay: { X: number; Y: number; Colour: string; Kind: "click" | "predicted" }[] = [];
  if (Frame?.Click) Markers.push({ U: Frame.Click.U, V: Frame.Click.V, Color: "#7dffa8" });
  // The fit's prediction is in native pixels (centres on integers); the frame is in capture pixels.
  const Predicted = Frame?.Result && Image?.naturalWidth ? {
    U: Frame.Mirrored ? Image.naturalWidth - (Frame.Result.predicted_px[0] + 0.5) : Frame.Result.predicted_px[0] + 0.5,
    V: Frame.Result.predicted_px[1] + 0.5 } : null;
  if (Predicted) Markers.push({ U: Predicted.U, V: Predicted.V, Color: "#ffb347" });
  if (Image?.naturalWidth) {
    const Box = ContentBox(Image), Fit_ = FitOf(Image), Parent = Image.parentElement?.getBoundingClientRect();
    for (const [Point, Colour, Kind] of [[Frame?.Click, "#7dffa8", "click"], [Predicted, "#ffb347", "predicted"]] as const) {
      if (!Point || !Parent) continue;
      const Client = ClientPointOf(Point.U, Point.V, Box, Image.naturalWidth, Image.naturalHeight, Fit_, Boolean(Frame?.Mirrored));
      Overlay.push({ X: Client.X - Parent.left, Y: Client.Y - Parent.top, Colour, Kind });
    }
  }
  const Waited = State.WaitingSince === null ? 0 : Now - State.WaitingSince;
  const CanFit = !Running && Counts.Clicked >= SemiCalMinClicks && !Busy && State.Phase !== "fitting";
  const Report = State.Report;
  const Outcome = Report ? CalibrationOutcomeOf(Report) : null;
  const ClickNeed = `needs ${SemiCalMinClicks}; ${SemiCalRecommendedClicks} or more recommended`;
  return <div className="semical-stage" role="dialog" aria-label="Semi-automatic calibration">
    <div className="semical-panel">
      <header className="semical-header">
        <div>
          <strong>Semi-automatic calibration</strong>
          <span className="semical-counts">
            {State.Total ? `Pose ${State.Ordinal} of ${State.Total} · ` : ""}{Counts.Clicked} clicked · {Counts.Skipped} skipped
            {Counts.Flagged ? ` · ${Counts.Flagged} flagged` : ""}
          </span>
        </div>
        <span className={`semical-phase phase-${State.Phase}`} role="status">{State.Message}</span>
        {!Running ? <button type="button" className="semical-icon" title="Close" aria-label="Close semi-automatic calibration"
          onClick={() => SemiCal.Close()}><X size={16} /></button> : null}
      </header>
      <div className="semical-body">
        <div className="semical-frame">
          {Frame ? <div className="semical-image-wrap">
            <img ref={ImageRef} src={Frame.Image} alt="Frozen calibration frame: click the centre of the red tape on the fixed jaw"
              className={`semical-image${Clickable ? "" : " locked"}`} style={Frame.Mirrored ? { transform: "scaleX(-1)" } : undefined} draggable={false}
              onLoad={() => SetLayout(Value => Value + 1)} onClick={OnImageClick} />
            <HoverLoupe ElementRef={ImageRef} PointRef={PointRef} MirroredDisplay={Frame.Mirrored} Watch={Frame.Image} Aim={Aim}
              Markers={Markers} OnHover={Point => { HoverRef.current = Point; SetAim(null); }} Name="Calibration frame magnifier" />
            {Overlay.map(Mark => <span key={Mark.Kind} className={`semical-mark mark-${Mark.Kind}`}
              style={{ left: Mark.X, top: Mark.Y, borderColor: Mark.Colour }} />)}
          </div> : <div className="semical-noframe">{State.Phase === "moving" ? "The arm is moving to the first pose" : "No frame yet"}</div>}
          {State.Frames.length ? <div className="semical-strip" role="list" aria-label="Captured poses">
            {State.Frames.map(Item => <button type="button" role="listitem" key={Item.PoseIndex}
              className={`semical-chip status-${Item.Status}${Item.Result?.flagged ? " flagged" : ""}${Item.PoseIndex === State.Active ? " active" : ""}${Item.PoseIndex === State.Waiting ? " waiting" : ""}`}
              title={`Pose ${Item.Ordinal}: ${Item.Status}${Item.Result ? `, ${Item.Result.residual_px.toFixed(1)} px off${Item.Result.reason ? ` (${Item.Result.reason})` : ""}` : ""}`}
              aria-label={`Pose ${Item.Ordinal} ${Item.Status}${Item.Result?.flagged ? " flagged" : ""}`}
              onClick={() => SemiCal.SetActive(Item.PoseIndex)}>
              {Item.Ordinal}
            </button>)}
          </div> : null}
        </div>
        <aside className="semical-side">
          <section className="semical-target">
            <TargetSketch />
            <p>Click <b>{SemiCalTargetText}</b>: the middle of the red face you can see, not the fingertip and not the moving jaw.
              Skip the pose if the jaw hides more than half of the tape or it is blurred.</p>
            <p className="semical-hint"><Crosshair size={13} /> The magnifier follows the pointer; the mouse wheel zooms 2x to 8x,
              arrow keys nudge one pixel and Enter clicks.</p>
          </section>
          {Frame ? <section className="semical-info">
            <span>Pose {Frame.Ordinal}{Frame.PoseIndex === State.Waiting ? " · the arm is holding here" : ""}</span>
            <span>Captured {new Date(Frame.CapturedAt).toLocaleTimeString()} · frame {Frame.FrameIndex + 1}</span>
            {Frame.Click ? <span className="semical-ok">Clicked{Frame.DepthCm !== null ? ` · depth ${Frame.DepthCm.toFixed(1)} cm` : " · no depth here"}</span>
              : Frame.Status === "skipped" ? <span>Skipped</span> : <span>{IsWaitingFrame ? "Waiting for your click" : "Not clicked"}</span>}
            {State.Waiting !== null && !IsWaitingFrame ? <button type="button" className="semical-link"
              onClick={() => SemiCal.SetActive(State.Waiting)}>The arm holds pose {State.Ordinal}: show its frame</button> : null}
            {Frame.Result ? <span className={Frame.Result.flagged ? "semical-bad" : ""}>
              Fit: {Frame.Result.residual_px.toFixed(1)} px ({Frame.Result.residual_cm.toFixed(2)} cm){Frame.Result.flagged ? ` · flagged: ${Frame.Result.reason}. Re-click it or skip it, then fit again.` : ""}
            </span> : null}
            {IsWaitingFrame && State.WaitingSince !== null ? <span className="semical-dim">
              Holding {Clock(Waited)} · ends early after {Clock(SemiCalWaitLimitMs)} without a click
              {Hottest !== null ? ` · hottest servo ${Hottest} °C (ends at ${SemiCalHotC})` : ""}</span> : null}
          </section> : null}
          {State.Notice ? <div className="semical-note" role="alert">{State.Notice}</div> : null}
          {Note ? <div className="semical-note" role="alert">{Note}</div> : null}
          <div className="semical-actions">
            <button type="button" onClick={() => void Undo()} disabled={Busy || Counts.Clicked === 0 || State.Phase === "fitting"} title="Remove the newest click">
              <CornerUpLeft size={15} />Undo</button>
            <button type="button" onClick={() => void Skip()} disabled={Busy || !Clickable || !Frame || Frame.Status === "skipped"}
              title="No click for this pose"><SkipForward size={15} />Skip pose</button>
            <button type="button" onClick={() => SemiCal.Decide("retake")} disabled={Busy || !Running || !IsWaitingFrame || State.Phase !== "waiting" || Frame?.Status !== "pending"}
              title="Wait a moment and capture this pose again"><RotateCcw size={15} />Retake</button>
            {Running ? <button type="button" className="semical-primary" onClick={() => SemiCal.RequestDone()}
              disabled={Counts.Clicked < SemiCalMinClicks || State.DoneRequested || State.Phase === "fitting" || State.Phase === "review"}
              title={`End the run now and fit the clicks (${ClickNeed})`}><Check size={15} />Done early</button>
              : <button type="button" className="semical-primary" onClick={() => void Fit()} disabled={!CanFit}
                title={`Fit the camera to the clicks again (${ClickNeed})`}><Check size={15} />{Report ? "Fit again" : "Fit now"}</button>}
          </div>
          {Report && Outcome ? <div className={`semical-report ${Outcome.Kind === "saved" ? "ok" : Outcome.Kind === "saved_with_warnings" || Outcome.Kind === "rejected" ? "warn" : "bad"}`}>
            <strong>{Outcome.Title}</strong>
            {Report.saved || Report.rejected ? <span>{Report.fit_rms_cm?.toFixed(2) ?? "?"} cm fit ({Report.fit_rms_px?.toFixed(1) ?? "?"} px) ·{" "}
              {Report.holdout_rms_cm?.toFixed(2) ?? "?"} cm held out · about {Report.plane_rms_cm?.toFixed(2) ?? "?"} cm on the table ·{" "}
              {Report.observations_used ?? "?"} clicks used</span> : Outcome.Detail ? <span>{Outcome.Detail}</span> : null}
            {Outcome.Problems.length ? <ul className="semical-problems" aria-label="Calibration warnings">
              {Outcome.Problems.map(Problem => <li key={Problem}>{Problem}</li>)}</ul> : null}
            {Outcome.Kind === "saved_with_warnings" && Outcome.Detail ? <span>{Outcome.Detail}</span> : null}
            {Report.warnings?.length ? <span className="semical-warn">{Report.warnings.join(" · ")}</span> : null}
            {Report.saved && !Report.rejected ? <button type="button" className="semical-reject" onClick={() => void Reject()}
              disabled={Busy || Running || State.Phase === "fitting"} title="Restore the calibration this fit replaced">
              <X size={14} />Reject calibration</button> : null}
          </div> : null}
          <div className="semical-actions stop">
            {Running ? <button type="button" onClick={OnStop} title="Stop moving to poses and hold; the clicks are kept">
              <Square size={14} />Stop</button> : null}
            <button type="button" className="emergency-button semical-emergency" onClick={OnEmergencyStop}>
              <OctagonAlert size={15} />Emergency stop</button>
          </div>
        </aside>
      </div>
    </div>
  </div>;
}
