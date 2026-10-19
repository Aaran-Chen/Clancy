import { useEffect, useRef, useState, type PointerEvent } from "react";
import { DialFraction, DialPoint } from "../lib/JointInteraction";
import { arcSpan, fractionToRaw, rawToFraction, trackAngleFromRaw, formatName, type ServoState } from "../lib/robotModel";

export default function JointDial({ Servo, OnChange }: { Servo: ServoState; OnChange: (Raw: number) => void }) {
  const Pointer = useRef<number | null>(null);
  const [Draft, SetDraft] = useState<number | null>(null);
  useEffect(() => {
    const Cancel = () => { Pointer.current = null; SetDraft(null); };
    window.addEventListener("blur", Cancel);
    return () => window.removeEventListener("blur", Cancel);
  }, []);
  const Fraction = Math.max(0, Math.min(1, rawToFraction(Servo, Draft ?? Servo.targetRaw)));
  const [X, Y] = DialPoint(Fraction);
  const Arc = Array.from({ length: 73 }, (_, Index) => DialPoint(Index / 72)).map((Point, Index) => `${Index ? "L" : "M"}${Point.join(" ")}`).join(" ");
  const Move = (Event: PointerEvent<SVGSVGElement>) => {
    if (Pointer.current !== Event.pointerId) return;
    const Bounds = Event.currentTarget.getBoundingClientRect();
    SetDraft(fractionToRaw(Servo, DialFraction((Event.clientX - Bounds.left) / Bounds.width * 200 - 100, (Event.clientY - Bounds.top) / Bounds.height * 200 - 100)));
  };
  const Finish = (Event: PointerEvent<SVGSVGElement>) => {
    if (Pointer.current !== Event.pointerId) return;
    Pointer.current = null;
    if (Event.type === "pointerup" && Draft !== null) OnChange(Draft);
    SetDraft(null);
    if (Event.currentTarget.hasPointerCapture(Event.pointerId)) Event.currentTarget.releasePointerCapture(Event.pointerId);
  };
  return <div className="joint-fine-control">
    <svg viewBox="0 0 200 200" role="slider" tabIndex={0} aria-label={`${formatName(Servo.name)} precision dial`}
      aria-valuemin={0} aria-valuemax={arcSpan(Servo)} aria-valuenow={Math.round(Fraction * arcSpan(Servo))}
      aria-valuetext={`${trackAngleFromRaw(Servo, Draft ?? Servo.targetRaw).toFixed(1)} degrees from range center`}
      onPointerDown={Event => { if (Event.button !== 0) return; Pointer.current = Event.pointerId; Event.currentTarget.setPointerCapture(Event.pointerId); Move(Event); }}
      onPointerMove={Move} onPointerUp={Finish} onPointerCancel={Finish} onLostPointerCapture={Finish}
      onKeyDown={Event => { if (Event.key === "Escape") { Pointer.current = null; SetDraft(null); }
        if (["ArrowLeft", "ArrowDown", "ArrowRight", "ArrowUp", "Home", "End"].includes(Event.key)) {
          Event.preventDefault(); const Step = (Event.shiftKey ? 10 : 1) / Math.max(arcSpan(Servo), 1);
          OnChange(fractionToRaw(Servo, Event.key === "Home" ? 0 : Event.key === "End" ? 1 : Fraction + (["ArrowLeft", "ArrowDown"].includes(Event.key) ? -Step : Step)));
        } }}>
      <circle cx="100" cy="100" r="87" className="dial-orbit" />
      <path d={Arc} className="dial-range" />
      {Array.from({ length: 19 }, (_, Index) => { const A = DialPoint(Index / 18, 81), B = DialPoint(Index / 18, Index % 3 ? 84 : 88); return <line key={Index} x1={A[0]} y1={A[1]} x2={B[0]} y2={B[1]} className="dial-tick" />; })}
      <line x1="100" y1="100" x2={X} y2={Y} className="dial-needle" />
      <circle cx={X} cy={Y} r="5" className="dial-handle" /><circle cx="100" cy="100" r="5" className="dial-hub" />
      <text x="100" y="149" className="dial-value">{trackAngleFromRaw(Servo, Draft ?? Servo.targetRaw).toFixed(1)}°</text>
      <text x="100" y="165" className="dial-caption">{Draft === null ? "ROTATION TARGET" : "RELEASE TO APPLY"}</text>
    </svg>
    <div className="joint-fine-step"><button type="button" aria-label={`Decrease ${formatName(Servo.name)} one step`} onClick={() => OnChange(fractionToRaw(Servo, Fraction - 1 / Math.max(arcSpan(Servo), 1)))}>−</button>
      <span>Fine adjust <b>{Draft ?? Servo.targetRaw}</b></span>
      <button type="button" aria-label={`Increase ${formatName(Servo.name)} one step`} onClick={() => OnChange(fractionToRaw(Servo, Fraction + 1 / Math.max(arcSpan(Servo), 1)))}>+</button></div>
  </div>;
}
