import { useEffect, useRef, useState, type ReactNode, type PointerEvent as ReactPointerEvent } from "react";
import { createPortal } from "react-dom";
import { Minus, Expand, Minimize2, Video } from "lucide-react";
import { CameraWindowStorageKey, ClampCameraBounds, ClearOfRects, MinimizeCameraBounds, ReadCameraBounds, ResizeCameraBounds, ResizeEdges,
  RestoreCameraBounds, SaveCameraBounds, MinimizedCameraSize, type CameraBounds, type ResizeEdge } from "../lib/CameraWindowBounds";
import "./FloatingCameraWindow.css";

type WindowProps = { Expanded: boolean; OnExpandedChange: (Value: boolean) => void; Actions?: ReactNode; children: ReactNode };
type WindowDrag = { Element: HTMLElement; PointerId: number; X: number; Y: number; Start: CameraBounds; Current: CameraBounds; Resize: ResizeEdge | null; Moved: boolean };
const GetViewport = () => ({ Width: window.innerWidth, Height: window.innerHeight, Top: document.fullscreenElement ? 12 : 68 });
// What the window keeps clear of while it is on screen (ClearOfRects): elements the page marks so, such as an AI step's
// Approve and Block, which it covered at 1600 x 1000. Checked a few times a second; the saved place is unchanged, so
// the window goes back there once they are gone.
const KeepClearSelector = "[data-camera-keep-clear]";
const KeepClearEveryMs = 300;

export default function FloatingCameraWindow({ Expanded, OnExpandedChange, Actions, children }: WindowProps) {
  const [Layer] = useState(() => { const Element = document.createElement("div"); Element.className = "camera-window-layer"; return Element; });
  const [Viewport, SetViewport] = useState(GetViewport), [Bounds, SetBounds] = useState(() => ReadCameraBounds(GetViewport()));
  const [Minimized, SetMinimized] = useState(() => { try { return localStorage.getItem(`${CameraWindowStorageKey}.Minimized`) === "true"; } catch { return false; } });
  const [Dragging, SetDragging] = useState(false);
  const [Clear, SetClear] = useState<CameraBounds | null>(null);
  const Drag = useRef<WindowDrag | null>(null), Frame = useRef(0);
  const SuppressRestore = useRef(false);
  const Current = useRef({ Bounds, Expanded, Minimized }); Current.current = { Bounds, Expanded, Minimized };
  const Finish = (Cancel = false) => {
    cancelAnimationFrame(Frame.current); Frame.current = 0;
    const Gesture = Drag.current; Drag.current = null;
    if (!Gesture) return;
    SuppressRestore.current = Gesture.Moved || Cancel;
    const Next = ClampCameraBounds(Cancel ? Gesture.Start : Gesture.Current, GetViewport(), Current.current.Minimized);
    SetBounds(Next); if (!Cancel) SaveCameraBounds(Next);
    if (Gesture.Element.hasPointerCapture(Gesture.PointerId)) Gesture.Element.releasePointerCapture(Gesture.PointerId);
    SetDragging(false);
  };
  useEffect(() => {
    const Attach = () => {
      Finish(true);
      (document.fullscreenElement ?? document.querySelector(".dashboard-app") ?? document.body).appendChild(Layer);
      SetViewport(GetViewport());
    };
    const Resize = () => { Finish(true); SetViewport(GetViewport()); SetBounds(Value => ClampCameraBounds(Value, GetViewport(), Current.current.Minimized)); };
    const Cancel = () => Finish(true);
    const Escape = (Event: KeyboardEvent) => { if (Event.key === "Escape") Finish(true); };
    Attach(); document.addEventListener("fullscreenchange", Attach); window.addEventListener("resize", Resize);
    window.addEventListener("blur", Cancel); window.addEventListener("keydown", Escape);
    return () => { cancelAnimationFrame(Frame.current); Layer.remove(); document.removeEventListener("fullscreenchange", Attach);
      window.removeEventListener("resize", Resize); window.removeEventListener("blur", Cancel); window.removeEventListener("keydown", Escape); };
  }, [Layer]);
  useEffect(() => { Finish(true); if (Expanded) SetMinimized(false); }, [Expanded]);
  useEffect(() => { try { localStorage.setItem(`${CameraWindowStorageKey}.Minimized`, String(Minimized)); } catch { /* Optional preference. */ } }, [Minimized]);
  const Start = (Event: ReactPointerEvent<HTMLElement>, Resize: ResizeEdge | null = null) => {
    if (Event.button !== 0 || Expanded) return;
    Event.preventDefault(); Finish(true); Event.currentTarget.focus(); Event.currentTarget.setPointerCapture(Event.pointerId);
    // From where it is shown (moved clear of the page's buttons or not): a drag never makes it jump, not even for the
    // frame before its first move (the shown place becomes its place; saved only when the drag ends).
    const StartBounds = ClampCameraBounds(Clear ?? Bounds, GetViewport(), Minimized);
    if (Clear) { SetBounds(StartBounds); SetClear(null); }
    SuppressRestore.current = false;
    Drag.current = { Element: Event.currentTarget, PointerId: Event.pointerId, X: Event.clientX, Y: Event.clientY, Start: StartBounds, Current: StartBounds, Resize, Moved: false };
    SetDragging(true);
  };
  const Move = (Event: ReactPointerEvent<HTMLElement>) => {
    const Gesture = Drag.current; if (!Gesture || Event.pointerId !== Gesture.PointerId) return;
    const X = Event.clientX - Gesture.X, Y = Event.clientY - Gesture.Y;
    if (!Gesture.Moved && Math.hypot(X, Y) < 5) return;
    Gesture.Moved = true;
    Gesture.Current = Gesture.Resize ? ResizeCameraBounds(Gesture.Start, Gesture.Resize, X, Y, GetViewport())
      : ClampCameraBounds({ ...Gesture.Start, X: Gesture.Start.X + X, Y: Gesture.Start.Y + Y }, GetViewport(), Minimized);
    if (!Frame.current) Frame.current = requestAnimationFrame(() => { Frame.current = 0; if (Drag.current === Gesture) SetBounds(Gesture.Current); });
  };
  const PointerHandlers = { onPointerMove: Move, onPointerUp: () => Finish(), onPointerCancel: () => Finish(true), onLostPointerCapture: () => Finish(true) };
  const Placed = Expanded ? { X: 12, Y: Viewport.Top, Width: Viewport.Width - 24, Height: Viewport.Height - Viewport.Top - 12 }
    : ClampCameraBounds(Bounds, Viewport, Minimized);
  const Visible = !Expanded && !Dragging && Clear ? Clear : Placed;
  const PlacedRef = useRef(Placed); PlacedRef.current = Placed;
  const Watch = useRef({ Expanded, Dragging, Minimized }); Watch.current = { Expanded, Dragging, Minimized };
  useEffect(() => {
    const Check = () => {
      const { Expanded: Full, Dragging: Moving, Minimized: Tile } = Watch.current;
      if (Full || Moving) return;
      const Rects = [...document.querySelectorAll<HTMLElement>(KeepClearSelector)].map(Element => Element.getBoundingClientRect())
        .filter(Rect => Rect.width > 0 && Rect.height > 0).map(Rect => ({ Left: Rect.left, Top: Rect.top, Right: Rect.right, Bottom: Rect.bottom }));
      const Base = PlacedRef.current, Next = Rects.length ? ClearOfRects(Base, Rects, GetViewport(), Tile) : Base;
      const Moved = Next.X !== Base.X || Next.Y !== Base.Y;
      SetClear(Previous => (!Moved ? null : Previous && Previous.X === Next.X && Previous.Y === Next.Y ? Previous : Next));
    };
    Check();
    const Timer = window.setInterval(Check, KeepClearEveryMs);
    return () => window.clearInterval(Timer);
  }, []);
  const MoveWithKeys = (Event: React.KeyboardEvent<HTMLElement>) => {
    const Directions: Record<string, number[]> = { ArrowLeft: [-1, 0], ArrowRight: [1, 0], ArrowUp: [0, -1], ArrowDown: [0, 1] };
    const Direction = Directions[Event.key]; if (!Direction || Expanded) return;
    Event.preventDefault(); const Step = Event.shiftKey ? 30 : 10;
    const Next = ClampCameraBounds({ ...Visible, X: Visible.X + Direction[0] * Step, Y: Visible.Y + Direction[1] * Step }, Viewport, Minimized);
    SetBounds(Next); SaveCameraBounds(Next);
  };
  return createPortal(<section className={`robot-call camera-window ${Expanded ? "call-expanded" : ""} ${Minimized ? "call-minimized" : ""}`}
    aria-label="Live camera window" data-dragging={Dragging} style={{ left: Visible.X, top: Visible.Y, width: Minimized ? MinimizedCameraSize : Visible.Width, height: Minimized ? MinimizedCameraSize : Visible.Height }}>
    {Minimized && <button className="camera-window-tile" type="button" aria-label="Restore Live Feed" aria-expanded={false} title="Live view. Click to restore or drag to move."
      onPointerDown={Event => Start(Event)} {...PointerHandlers} onKeyDown={MoveWithKeys}
      onClick={Event => { if (Event.detail && SuppressRestore.current) { SuppressRestore.current = false; return; }
        Finish(true); SetMinimized(false);
        const Next = RestoreCameraBounds(ClampCameraBounds(Bounds, GetViewport(), true), GetViewport()); SetBounds(Next); SaveCameraBounds(Next); }}>
      <Video size={21} /></button>}
    <header className="camera-window-header" hidden={Minimized}>
      <div className="camera-window-drag" role="button" tabIndex={0} aria-label="Move camera window" title="Drag to move. Arrow keys fine-tune position; double-click to expand."
        onPointerDown={Event => Start(Event)} {...PointerHandlers} onDoubleClick={() => OnExpandedChange(!Expanded)}
        onKeyDown={MoveWithKeys}><div className="camera-window-title"><strong>Live view</strong></div></div>
      <div className="camera-window-actions">{Actions}<button type="button" aria-label={Expanded ? "Exit full screen live view" : "Full screen live view"}
        title={Expanded ? "Exit full screen" : "Full screen"} aria-expanded={Expanded} onClick={() => OnExpandedChange(!Expanded)}>
        {Expanded ? <Minimize2 size={16} /> : <Expand size={16} />}</button>
        <button type="button" aria-label="Minimize Live Feed" title="Minimize" aria-expanded={!Minimized}
          onClick={() => { Finish(true); if (Expanded) OnExpandedChange(false); SetMinimized(true);
            const Next = MinimizeCameraBounds(ClampCameraBounds(Bounds, GetViewport()), GetViewport()); SetBounds(Next); SaveCameraBounds(Next); }}>
          <Minus size={17} /></button></div>
    </header>
    <div className="camera-window-content" hidden={Minimized}>{children}</div>
    {!Expanded && !Minimized && ResizeEdges.filter(Edge => Edge !== "se").map(Edge => <div key={Edge} className={`camera-window-edge edge-${Edge}`}
      aria-hidden="true" onPointerDown={Event => Start(Event, Edge)} {...PointerHandlers} />)}
    {!Expanded && !Minimized && <div className="camera-window-resize" role="button" tabIndex={0} aria-label="Resize camera window" title="Drag any edge or corner to resize. Arrow keys also work."
      onPointerDown={Event => Start(Event, "se")} {...PointerHandlers} onKeyDown={Event => {
        if (!["ArrowLeft", "ArrowRight", "ArrowUp", "ArrowDown"].includes(Event.key)) return;
        Event.preventDefault(); const Step = Event.shiftKey ? 30 : 10;
        const Next = ClampCameraBounds({ ...Visible, Width: Visible.Width + (Event.key === "ArrowLeft" ? -Step : Event.key === "ArrowRight" ? Step : 0),
          Height: Visible.Height + (Event.key === "ArrowUp" ? -Step : Event.key === "ArrowDown" ? Step : 0) }, Viewport);
        SetBounds(Next); SaveCameraBounds(Next);
      }} />}
  </section>, Layer);
}
