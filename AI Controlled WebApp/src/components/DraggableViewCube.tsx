import { useEffect, useRef, useState, type RefObject } from "react";

type CubePosition = { X: number; Y: number };
type CubeDrag = { PointerId: number; X: number; Y: number; Start: CubePosition; Moved: boolean };
const StorageKey = "SO101.ViewCube.Position.v1";
const ReadPosition = (): CubePosition => {
  try {
    const Saved = JSON.parse(localStorage.getItem(StorageKey) ?? "null");
    if (Saved && Number.isFinite(Saved.X) && Number.isFinite(Saved.Y)) return Saved;
  } catch { /* Position persistence is optional. */ }
  return { X: 12, Y: 12 };
};

export default function DraggableViewCube({ BodyRef, Hidden, OnView }: {
  BodyRef: RefObject<HTMLDivElement | null>; Hidden: boolean; OnView: (View: string) => void;
}) {
  const Element = useRef<HTMLDivElement>(null), Drag = useRef<CubeDrag | null>(null), SuppressClick = useRef(false);
  const [Position, SetPosition] = useState(ReadPosition), [Moving, SetMoving] = useState(false);
  const Current = useRef(Position); Current.current = Position;
  const Clamp = (Value: CubePosition) => {
    const Cube = Element.current, Host = Cube?.parentElement;
    return { X: Math.max(12, Math.min(Value.X, (Host?.clientWidth ?? 152) - (Cube?.offsetWidth || 128) - 12)),
      Y: Math.max(12, Math.min(Value.Y, (Host?.clientHeight ?? 138) - (Cube?.offsetHeight || 114) - 12)) };
  };
  const Apply = (Value: CubePosition, Save = false) => {
    const Next = Clamp(Value); Current.current = Next; SetPosition(Next);
    if (Save) try { localStorage.setItem(StorageKey, JSON.stringify(Next)); } catch { /* Optional preference. */ }
  };
  const Finish = (Cancel = false) => {
    const Gesture = Drag.current; Drag.current = null;
    if (!Gesture) return;
    SuppressClick.current = Gesture.Moved || Cancel;
    Apply(Cancel ? Gesture.Start : Current.current, !Cancel && Gesture.Moved);
    if (Element.current?.hasPointerCapture(Gesture.PointerId)) Element.current.releasePointerCapture(Gesture.PointerId);
    SetMoving(false);
  };
  useEffect(() => {
    const Move = (Event: PointerEvent) => {
      const Gesture = Drag.current; if (!Gesture || Event.pointerId !== Gesture.PointerId) return;
      const X = Event.clientX - Gesture.X, Y = Event.clientY - Gesture.Y;
      if (!Gesture.Moved && Math.hypot(X, Y) < 5) return;
      if (!Gesture.Moved) { Gesture.Moved = true; SetMoving(true); Element.current?.setPointerCapture(Event.pointerId); }
      Event.preventDefault(); Apply({ X: Gesture.Start.X + X, Y: Gesture.Start.Y + Y });
    };
    const Up = (Event: PointerEvent) => { if (Event.pointerId === Drag.current?.PointerId) Finish(Event.type === "pointercancel"); };
    const Cancel = () => Finish(true);
    const Key = (Event: KeyboardEvent) => { if (Event.key === "Escape") Finish(true); };
    const Resize = new ResizeObserver(() => { Finish(true); Apply(Current.current); });
    if (Element.current?.parentElement) Resize.observe(Element.current.parentElement);
    window.addEventListener("pointermove", Move, { passive: false }); window.addEventListener("pointerup", Up);
    window.addEventListener("pointercancel", Up); window.addEventListener("blur", Cancel); window.addEventListener("keydown", Key);
    return () => { Resize.disconnect(); window.removeEventListener("pointermove", Move); window.removeEventListener("pointerup", Up);
      window.removeEventListener("pointercancel", Up); window.removeEventListener("blur", Cancel); window.removeEventListener("keydown", Key); };
  }, []);
  useEffect(() => { if (Hidden) Finish(true); else Apply(Current.current); }, [Hidden]);
  return <div ref={Element} className="view-cube cad-view-cube" role="group" tabIndex={0} aria-label="View cube"
    title="Click a face to change view. Drag to move the cube. Arrow keys move it; Home resets its position."
    hidden={Hidden} data-dragging={Moving} style={{ left: Position.X, top: Position.Y }}
    onPointerDown={Event => {
      if (Event.button !== 0) return;
      Finish(true); SuppressClick.current = false;
      Drag.current = { PointerId: Event.pointerId, X: Event.clientX, Y: Event.clientY, Start: Current.current, Moved: false };
    }} onLostPointerCapture={() => Finish(true)}
    onClickCapture={Event => { if (SuppressClick.current) { Event.preventDefault(); Event.stopPropagation(); SuppressClick.current = false; } }}
    onKeyDown={Event => {
      const Direction = { ArrowLeft: [-1, 0], ArrowRight: [1, 0], ArrowUp: [0, -1], ArrowDown: [0, 1] }[Event.key];
      if (Event.key === "Home") { Event.preventDefault(); Apply({ X: 12, Y: 12 }, true); }
      if (!Direction) return;
      Event.preventDefault(); const Step = Event.shiftKey ? 30 : 10;
      Apply({ X: Current.current.X + Direction[0] * Step, Y: Current.current.Y + Direction[1] * Step }, true);
    }}>
    <div className="view-cube-stage"><div ref={BodyRef} className="view-cube-body">
      {[["front", "Front"], ["back", "Back"], ["right", "Left"], ["left", "Right"], ["top", "Top"], ["bottom", "Bottom"]].map(([Face, View]) =>
        <button key={Face} type="button" className={`view-cube-face view-cube-face-${Face}`} aria-label={`View ${View.toLowerCase()}`}
          onClick={() => OnView(View.toLowerCase())}>{View}</button>)}
    </div></div>
  </div>;
}
