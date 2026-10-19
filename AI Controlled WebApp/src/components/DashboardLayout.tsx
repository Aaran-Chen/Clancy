import { useEffect, useRef, useState, type ReactNode, type RefObject, type PointerEvent as ReactPointerEvent } from "react";
import { SlidersHorizontal, RotateCcw, Settings2 } from "lucide-react";
import { PixelBot } from "./AiCompanion";
import { ColumnMinimums, ColumnWeights, DefaultAiWeights, DefaultColumnWeights, FitColumnWidths, MinimumFitScale, ReadColumnWeights,
  ReadSavedColumnWeights, ResizeColumnPair, SaveDashboardSetting, type DashboardMode } from "../lib/DashboardLayout";
import "./Dashboard.css";

type DashboardLayoutProps = {
  Mode: DashboardMode;
  OnModeChange: (Mode: DashboardMode) => void;
  OnSettings: () => void;
  SettingsOpen: boolean;
  Left: ReactNode;
  Scene: ReactNode;
  Joints: ReactNode;
  Controls: ReactNode;
};
type DividerDrag = { Element: HTMLElement; PointerId: number; Index: number; X: number; Start: number[]; Current: number[]; Mode: DashboardMode };
// Manual widths are always set; AI's are null until the user drags, and then
// the default split follows the window (DefaultAiWeights).
type Layouts = { Manual: number[]; Ai: number[] | null };

// A column never scrolls in normal use: its sections share the height and
// compact with it (Dashboard.css). Content that still overflows takes the
// denser steps first (the ones a shorter column gets anyway), so text stays
// full size; only if the densest step overflows do the column's panels scale
// down until they fit, never below MinimumFitScale; below that the column
// scrolls rather than hide a control.
function FitColumn(Column: HTMLElement, Seen: WeakMap<HTMLElement, string>) {
  const Content = Column.querySelector<HTMLElement>(":scope > .column-fit");
  if (!Content || !Column.clientHeight) return;
  const Fits = () => [Column, ...Content.querySelectorAll<HTMLElement>(".dashboard-section")]
    .every(Element => Element.scrollHeight <= Element.clientHeight + 1);
  // Live numbers re-render ten times a second; only a change of size or content can change the fit.
  const Signature = `${Column.clientWidth}x${Column.clientHeight}:${Content.getElementsByTagName("*").length}:${Content.textContent?.length ?? 0}`;
  if (Seen.get(Column) === Signature && Fits()) return;
  Seen.set(Column, Signature);
  const Apply = (Scale: number) => {
    Content.style.zoom = Scale < 1 ? String(Scale) : "";
    Content.dataset.fitScale = String(Scale);
  };
  // Dashboard.css's container queries read these: 1 is the step below 1100 px of height, 2 the one
  // below 760 px, and 3 a tighter one that only overflowing content gets.
  const Density = (Step: number) => {
    for (const [Name, From] of [["--fit-dense", 1], ["--fit-compact", 2], ["--fit-tight", 3]] as const) {
      if (Step >= From) Column.style.setProperty(Name, "1"); else Column.style.removeProperty(Name);
    }
    Content.dataset.fitDensity = String(Step);
  };
  Apply(1);
  for (let Step = 0; Step <= 3; Step++) {
    Density(Step);
    if (Fits()) return;
  }
  Apply(MinimumFitScale);
  if (!Fits()) return;
  let Low = MinimumFitScale, High = 1;
  for (let Step = 0; Step < 6; Step++) {
    const Middle = (Low + High) / 2;
    Apply(Middle);
    if (Fits()) Low = Middle; else High = Middle;
  }
  Apply(Math.floor(Low * 100) / 100);
}

function UseColumnFit(Columns: RefObject<HTMLElement | null>[]) {
  useEffect(() => {
    const Seen = new WeakMap<HTMLElement, string>();
    let Queued = false;
    // Runs before paint (observer callbacks), so a column never flashes a scrollbar.
    const Run = () => {
      Queued = false;
      for (const Ref of Columns) if (Ref.current && Ref.current.offsetParent) FitColumn(Ref.current, Seen);
    };
    const Schedule = () => { if (!Queued) { Queued = true; queueMicrotask(Run); } };
    const Resize = new ResizeObserver(Schedule);
    const Mutation = new MutationObserver(Schedule);
    for (const Ref of Columns) {
      if (!Ref.current) continue;
      Resize.observe(Ref.current);
      Mutation.observe(Ref.current, { subtree: true, childList: true, characterData: true, attributes: true, attributeFilter: ["hidden", "class", "data-minimized", "data-truncated"] });
    }
    void document.fonts?.ready.then(() => { for (const Ref of Columns) if (Ref.current) Seen.delete(Ref.current); Schedule(); });
    Schedule();
    return () => { Resize.disconnect(); Mutation.disconnect(); };
  }, []);
}

export default function DashboardLayout({ Mode, OnModeChange, OnSettings, SettingsOpen, Left, Scene, Joints, Controls }: DashboardLayoutProps) {
  const ScrollRef = useRef<HTMLDivElement>(null);
  const ControlRef = useRef<HTMLElement>(null), SliderRef = useRef<HTMLElement>(null);
  const DragRef = useRef<DividerDrag | null>(null);
  const FrameRef = useRef(0);
  const [Size, SetSize] = useState({ Width: 1200, Height: 800 });
  const [Resizing, SetResizing] = useState(false);
  const [Saved, SetSaved] = useState<Layouts>(() => ({ Manual: ReadColumnWeights("Manual"), Ai: ReadSavedColumnWeights("Ai") }));
  const Minimums = ColumnMinimums[Mode];
  const Available = Size.Width - (Minimums.length - 1) * 7;
  const Weights = Saved[Mode] ?? DefaultAiWeights(Available, Size.Height);
  const Widths = FitColumnWidths(Available, Weights, Minimums);
  const Names = Mode === "Manual" ? ["Workspace", "Joints", "Controls"] : ["AI", "Workspace"];
  UseColumnFit([ControlRef, SliderRef]);

  const Apply = (ForMode: DashboardMode, Values: number[], Save = false) => {
    const Next = ColumnWeights(Values);
    SetSaved(Previous => ({ ...Previous, [ForMode]: Next }));
    if (Save) SaveDashboardSetting(`${ForMode}.Widths`, Next);
  };
  const Finish = (Cancel = false) => {
    cancelAnimationFrame(FrameRef.current); FrameRef.current = 0;
    const Drag = DragRef.current;
    DragRef.current = null;
    if (!Drag) return;
    Apply(Drag.Mode, Cancel ? Drag.Start : Drag.Current, !Cancel);
    if (Drag.Element.hasPointerCapture(Drag.PointerId)) Drag.Element.releasePointerCapture(Drag.PointerId);
    SetResizing(false);
  };
  useEffect(() => {
    const Element = ScrollRef.current;
    if (!Element) return;
    const Observer = new ResizeObserver(() => { Finish(true); SetSize({ Width: Element.clientWidth, Height: Element.clientHeight }); });
    Observer.observe(Element);
    const Cancel = () => Finish(true);
    const Key = (Event: KeyboardEvent) => { if (Event.key === "Escape") Finish(true); };
    window.addEventListener("blur", Cancel); window.addEventListener("keydown", Key);
    return () => {
      Observer.disconnect(); cancelAnimationFrame(FrameRef.current);
      window.removeEventListener("blur", Cancel); window.removeEventListener("keydown", Key);
    };
  }, []);
  useEffect(() => { Finish(true); }, [Mode]);

  const Start = (Event: ReactPointerEvent<HTMLDivElement>, Index: number) => {
    if (Event.button !== 0) return;
    Event.preventDefault(); Finish(true);
    Event.currentTarget.focus();
    Event.currentTarget.setPointerCapture(Event.pointerId);
    DragRef.current = { Element: Event.currentTarget, PointerId: Event.pointerId, Index, X: Event.clientX, Start: Widths, Current: Widths, Mode };
    SetResizing(true);
  };
  const Move = (Event: ReactPointerEvent<HTMLDivElement>) => {
    const Drag = DragRef.current;
    if (!Drag || Drag.PointerId !== Event.pointerId) return;
    Drag.Current = ResizeColumnPair(Drag.Start, ColumnMinimums[Drag.Mode], Drag.Index, Event.clientX - Drag.X);
    if (!FrameRef.current) FrameRef.current = requestAnimationFrame(() => {
      FrameRef.current = 0;
      if (DragRef.current === Drag) Apply(Drag.Mode, Drag.Current);
    });
  };
  const ResetWidths = () => {
    Finish(true);
    // AI's default split follows the window; Manual's is fixed proportions.
    const Default = Mode === "Ai" ? null : [...DefaultColumnWeights.Manual];
    SetSaved(Previous => ({ ...Previous, [Mode]: Default }));
    SaveDashboardSetting(`${Mode}.Widths`, Default);
  };
  const Divider = (Index: number) => (
    <div className="dashboard-divider" style={{ gridColumn: Index * 2 + 2, gridRow: 1 }} role="separator" tabIndex={0} aria-orientation="vertical"
      aria-label={`Resize ${Names[Index]} and ${Names[Index + 1]}`} title="Drag to resize. Arrow keys also work; double-click to reset widths."
      aria-valuemin={Math.round(Minimums[Index])} aria-valuemax={Math.round(Widths[Index] + Widths[Index + 1] - Minimums[Index + 1])}
      aria-valuenow={Math.round(Widths[Index])} aria-valuetext={`${Math.round(Widths[Index])} pixels`}
      onPointerDown={Event => Start(Event, Index)} onPointerMove={Move} onPointerUp={() => Finish()}
      onPointerCancel={() => Finish(true)} onLostPointerCapture={() => Finish(true)} onDoubleClick={ResetWidths}
      onKeyDown={Event => {
        const Delta = Event.key === "Home" ? -Infinity : Event.key === "End" ? Infinity
          : Event.key === "ArrowLeft" ? -(Event.shiftKey ? 48 : 16) : Event.key === "ArrowRight" ? (Event.shiftKey ? 48 : 16) : null;
        if (Delta === null) return;
        Event.preventDefault(); Apply(Mode, ResizeColumnPair(Widths, Minimums, Index, Delta), true);
      }}><span /></div>
  );
  return (
    <main className="dashboard-layout" data-mode={Mode} data-resizing={Resizing}>
      <nav className="mode-rail" aria-label="Control mode">
        <span className="mode-rail-label">MODE</span>
        <button type="button" className={Mode === "Manual" ? "active" : ""} aria-pressed={Mode === "Manual"}
          title="Pause AI follow-ups and show manual controls. Use Stop and hold to stop the current move."
          aria-label="Manual control mode" onClick={() => OnModeChange("Manual")}><SlidersHorizontal size={21} /><span>Manual</span></button>
        <button type="button" className={Mode === "Ai" ? "active" : ""} aria-pressed={Mode === "Ai"}
          title="Show AI control and token usage. Motion proposals still require approval."
          aria-label="AI control mode" onClick={() => OnModeChange("Ai")}><PixelBot /><span>AI</span></button>
        <div className="mode-rail-bottom">
          <button type="button" className={SettingsOpen ? "active" : ""} onClick={OnSettings} aria-label="Settings" aria-expanded={SettingsOpen} aria-controls="dashboard-settings"><Settings2 size={20} /><span>Settings</span></button>
          <button type="button" onClick={ResetWidths} title="Reset column widths" aria-label="Reset column widths"><RotateCcw size={17} /><span>Reset<br />widths</span></button>
        </div>
      </nav>
      <div className="dashboard-columns-scroll" ref={ScrollRef}>
        <div className="workspace" style={{ gridTemplateColumns: Widths.map(Value => `${Value}px`).join(" 7px "), minWidth: `${Minimums.reduce((Sum, Value) => Sum + Value, 0) + (Minimums.length - 1) * 7}px` }}>
          <aside className="control-column" ref={ControlRef} style={{ gridColumn: Mode === "Ai" ? 1 : 5, gridRow: 1 }}>
            <div className="column-fit"><div className="ai-column" hidden={Mode === "Manual"}>{Left}</div>{Controls}</div>
          </aside>
          {Divider(0)}
          <section className="scene-column" style={{ gridColumn: Mode === "Ai" ? 3 : 1, gridRow: 1 }}>{Scene}</section>
          <aside className="slider-column" ref={SliderRef} hidden={Mode === "Ai"} style={{ gridColumn: 3, gridRow: 1 }}>
            <div className="column-fit">{Joints}</div>
          </aside>
          {Mode === "Manual" ? Divider(1) : null}
        </div>
      </div>
    </main>
  );
}
