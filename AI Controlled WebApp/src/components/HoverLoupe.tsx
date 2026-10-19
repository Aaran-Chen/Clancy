import { useEffect, useRef, type MutableRefObject, type RefObject } from "react";
import { createPortal } from "react-dom";
import {
  ClampZoom, ContentBox, DisplayScale, FitOf, LoupeDefaultZoom, LoupePlacement, LoupeSourceRect, SourcePointAt,
  type SourcePoint,
} from "../lib/ImagePointer";
import "./HoverLoupe.css";

// A magnifier that follows the pointer over a picture: it shows the area under
// the cursor from the FULL-RESOLUTION source (the decoded image, the video frame
// or a canvas), pixelated, with a crosshair on exactly the source pixel a click
// there records (lib/ImagePointer.ts holds the one mapping both use). It sits
// beside the cursor, flips side near the viewport edges, never takes pointer
// events, and hides when the pointer leaves the picture. The mouse wheel zooms
// 2x to 8x relative to the displayed size while hovering.

export type LoupeMarker = { U: number; V: number; Color: string };
type LoupeSource = { Source: CanvasImageSource; Width: number; Height: number };
type HoverLoupeProps = {
  // The element that draws the picture (its box and object-fit define the mapping).
  ElementRef: RefObject<HTMLImageElement | HTMLVideoElement | HTMLCanvasElement | null>;
  // Where pointer events are read, when the picture sits under an overlay; defaults to ElementRef.
  ListenRef?: RefObject<HTMLElement | null>;
  // The full-resolution pixels to magnify; defaults to the element itself.
  GetSource?: () => LoupeSource | null;
  // The picture is shown flipped left-right (CSS scaleX(-1)).
  MirroredDisplay?: boolean;
  Enabled?: boolean;
  // Redraw every frame while visible (a live video).
  Live?: boolean;
  HideWhilePressed?: boolean;
  Size?: number;
  Markers?: LoupeMarker[];
  // A pixel to show instead of the hovered one (a keyboard nudge); cleared by the parent on pointer moves.
  Aim?: SourcePoint | null;
  // The hovered source pixel as the pointer moves (null off the picture).
  OnHover?: (Point: SourcePoint | null) => void;
  // Kept equal to the pixel under the crosshair (null while hidden). Click handlers
  // record this: Chromium rounds a click's clientX/Y to whole CSS pixels while
  // pointer moves keep fractions, which at a 2.25 device pixel ratio is a 1.6 px miss.
  PointRef?: MutableRefObject<SourcePoint | null>;
  // Re-attach when this changes (the picture element was replaced).
  Watch?: unknown;
  Name?: string;
};

const ZoomKey = "SO101.LoupeZoom";
function StoredZoom() {
  try { const Value = Number(localStorage.getItem(ZoomKey)); return Number.isFinite(Value) && Value > 0 ? ClampZoom(Value) : LoupeDefaultZoom; }
  catch { return LoupeDefaultZoom; }
}

export function ElementSource(Element: HTMLImageElement | HTMLVideoElement | HTMLCanvasElement | null): LoupeSource | null {
  if (!Element) return null;
  if (Element instanceof HTMLImageElement) {
    return Element.complete && Element.naturalWidth > 0 ? { Source: Element, Width: Element.naturalWidth, Height: Element.naturalHeight } : null;
  }
  if (Element instanceof HTMLVideoElement) {
    return Element.readyState >= HTMLMediaElement.HAVE_CURRENT_DATA && Element.videoWidth > 0
      ? { Source: Element, Width: Element.videoWidth, Height: Element.videoHeight } : null;
  }
  return Element.width > 0 && Element.height > 0 ? { Source: Element, Width: Element.width, Height: Element.height } : null;
}

export default function HoverLoupe({ ElementRef, ListenRef, GetSource, MirroredDisplay = false, Enabled = true, Live = false,
  HideWhilePressed = false, Size = 184, Markers, Aim = null, OnHover, PointRef, Watch, Name = "Magnifier" }: HoverLoupeProps) {
  const BoxRef = useRef<HTMLDivElement>(null), CanvasRef = useRef<HTMLCanvasElement>(null), ReadoutRef = useRef<HTMLDivElement>(null);
  const Pointer = useRef<{ X: number; Y: number; Inside: boolean; Pressed: boolean }>({ X: 0, Y: 0, Inside: false, Pressed: false });
  const Zoom = useRef(StoredZoom());
  // Props read by the event handlers without re-attaching them.
  const Props = useRef({ GetSource, MirroredDisplay, Markers, Aim, OnHover, Size, PointRef });
  Props.current = { GetSource, MirroredDisplay, Markers, Aim, OnHover, Size, PointRef };
  const LastHover = useRef<string>("");
  const DrawRef = useRef<() => void>(() => {});

  DrawRef.current = () => {
    const Box = BoxRef.current, Canvas = CanvasRef.current, Element = ElementRef.current;
    const Hide = () => {
      if (Props.current.PointRef) Props.current.PointRef.current = null;
      if (Box) { Box.style.display = "none"; Box.dataset.visible = "false"; }
    };
    if (!Box || !Canvas || !Element || !Pointer.current.Inside || Pointer.current.Pressed) { Hide(); return; }
    const { GetSource: Getter, MirroredDisplay: Mirrored, Markers: Marks, Aim: Aimed, OnHover: Report, Size: Across } = Props.current;
    const Source = Getter ? Getter() : ElementSource(Element);
    if (!Source) { Hide(); return; }
    const Content = ContentBox(Element), Fit = FitOf(Element);
    const Hovered = SourcePointAt(Pointer.current.X, Pointer.current.Y, Content, Source.Width, Source.Height, Fit, Mirrored);
    const HoverKey = Hovered ? `${Hovered.Column},${Hovered.Row}` : "";
    if (HoverKey !== LastHover.current) { LastHover.current = HoverKey; Report?.(Hovered); }
    const Point = Hovered && Aimed ? Aimed : Hovered;
    if (!Point) { Hide(); return; }
    if (Props.current.PointRef) Props.current.PointRef.current = Point;
    const Place = LoupePlacement(Pointer.current.X, Pointer.current.Y, Across, window.innerWidth, window.innerHeight);
    Box.style.display = "block";
    Box.style.width = `${Across}px`; Box.style.height = `${Across}px`;
    Box.style.transform = `translate(${Place.Left}px, ${Place.Top}px)`;
    const Ratio = window.devicePixelRatio || 1, Device = Math.max(1, Math.round(Across * Ratio));
    if (Canvas.width !== Device || Canvas.height !== Device) { Canvas.width = Device; Canvas.height = Device; }
    Canvas.style.width = `${Across}px`; Canvas.style.height = `${Across}px`;
    const Context = Canvas.getContext("2d");
    if (!Context) { Hide(); return; }
    const Scale = DisplayScale(Content, Source.Width, Source.Height, Fit);
    const Rect = LoupeSourceRect(Point, Across, Scale, Zoom.current);
    const PerPixel = Device / Rect.Width;  // device pixels per source pixel
    Context.setTransform(1, 0, 0, 1, 0, 0);
    Context.imageSmoothingEnabled = false;
    Context.fillStyle = "#0d0f12";
    Context.fillRect(0, 0, Device, Device);
    const X0 = Math.max(0, Rect.X), Y0 = Math.max(0, Rect.Y);
    const X1 = Math.min(Source.Width, Rect.X + Rect.Width), Y1 = Math.min(Source.Height, Rect.Y + Rect.Height);
    Context.save();
    if (Mirrored) { Context.translate(Device, 0); Context.scale(-1, 1); }
    if (X1 > X0 && Y1 > Y0) {
      try {
        Context.drawImage(Source.Source, X0, Y0, X1 - X0, Y1 - Y0, (X0 - Rect.X) * PerPixel, (Y0 - Rect.Y) * PerPixel,
          (X1 - X0) * PerPixel, (Y1 - Y0) * PerPixel);
      } catch { /* a source that cannot be drawn yet leaves the backdrop */ }
    }
    for (const Mark of Marks ?? []) {
      const X = (Mark.U - Rect.X) * PerPixel, Y = (Mark.V - Rect.Y) * PerPixel;
      if (X < -20 || Y < -20 || X > Device + 20 || Y > Device + 20) continue;
      Context.lineWidth = 2 * Ratio;
      Context.strokeStyle = "#000c";
      Context.beginPath(); Context.arc(X, Y, 7 * Ratio, 0, Math.PI * 2); Context.stroke();
      Context.lineWidth = 1.2 * Ratio;
      Context.strokeStyle = Mark.Color;
      Context.beginPath(); Context.arc(X, Y, 7 * Ratio, 0, Math.PI * 2); Context.stroke();
    }
    Context.restore();
    // Crosshair: the recorded pixel is the cell at the exact centre (the view is centred on it).
    const Centre = Device / 2, Half = Math.max(PerPixel / 2, 1.5 * Ratio), Gap = Half + 4 * Ratio;
    const Lines = (Width: number, Colour: string) => {
      Context.lineWidth = Width; Context.strokeStyle = Colour;
      Context.beginPath();
      Context.moveTo(0, Centre); Context.lineTo(Centre - Gap, Centre);
      Context.moveTo(Centre + Gap, Centre); Context.lineTo(Device, Centre);
      Context.moveTo(Centre, 0); Context.lineTo(Centre, Centre - Gap);
      Context.moveTo(Centre, Centre + Gap); Context.lineTo(Centre, Device);
      Context.stroke();
      Context.strokeRect(Centre - Half - Width / 2, Centre - Half - Width / 2, 2 * Half + Width, 2 * Half + Width);
    };
    Lines(3 * Ratio, "#000000b0");
    Lines(1 * Ratio, "#5ff2ff");
    const Shown = Mirrored ? Source.Width - 1 - Point.Column : Point.Column;
    if (ReadoutRef.current) ReadoutRef.current.textContent = `x ${Shown} · y ${Point.Row} · ${Zoom.current}x`;
    Object.assign(Box.dataset, { visible: "true", column: String(Point.Column), row: String(Point.Row), zoom: String(Zoom.current),
      left: Place.Left.toFixed(1), top: Place.Top.toFixed(1) });
  };

  useEffect(() => {
    const Target = ListenRef?.current ?? ElementRef.current;
    if (!Enabled || !Target) { Pointer.current.Inside = false; DrawRef.current(); return; }
    let Frame = 0, LiveFrame = 0, LastLive = 0;
    const Schedule = () => { if (!Frame) Frame = requestAnimationFrame(() => { Frame = 0; DrawRef.current(); }); };
    const LiveLoop = (Now: number) => {
      LiveFrame = 0;
      if (!Pointer.current.Inside) return;
      if (Now - LastLive >= 33) { LastLive = Now; DrawRef.current(); }
      LiveFrame = requestAnimationFrame(LiveLoop);
    };
    const Move = (Event: PointerEvent) => {
      Pointer.current.X = Event.clientX; Pointer.current.Y = Event.clientY; Pointer.current.Inside = true;
      if (HideWhilePressed) Pointer.current.Pressed = Event.buttons !== 0;
      // Place the box at once so it tracks the pointer; draw on the next frame.
      DrawRef.current();
      if (Live && !LiveFrame) LiveFrame = requestAnimationFrame(LiveLoop);
    };
    const Leave = () => { Pointer.current.Inside = false; LastHover.current = ""; Props.current.OnHover?.(null); DrawRef.current(); };
    const Wheel = (Event: WheelEvent) => {
      if (!Pointer.current.Inside || BoxRef.current?.dataset.visible !== "true" || Event.deltaY === 0) return;
      Event.preventDefault();
      Zoom.current = ClampZoom(Zoom.current + (Event.deltaY < 0 ? 1 : -1));
      try { localStorage.setItem(ZoomKey, String(Zoom.current)); } catch { /* Storage optional. */ }
      Schedule();
    };
    const Scrolled = () => { if (Pointer.current.Inside) Schedule(); };
    // A picture whose src changed under a still pointer (the next frozen frame) shows its new
    // pixels once decoded, not the previous picture's.
    const Picture = ElementRef.current;
    Picture?.addEventListener("load", Scrolled);
    Target.addEventListener("pointermove", Move);
    Target.addEventListener("pointerdown", Move);
    Target.addEventListener("pointerup", Move);
    Target.addEventListener("pointerleave", Leave);
    Target.addEventListener("pointercancel", Leave);
    Target.addEventListener("wheel", Wheel, { passive: false });
    window.addEventListener("scroll", Scrolled, true);
    window.addEventListener("resize", Scrolled);
    // Re-attached for a new picture (Watch) under a pointer that has not moved: show it there at
    // once, so the crosshair (and what a click records) is never left on the old picture or hidden.
    if (Pointer.current.Inside) Schedule();
    return () => {
      cancelAnimationFrame(Frame); cancelAnimationFrame(LiveFrame);
      Target.removeEventListener("pointermove", Move);
      Target.removeEventListener("pointerdown", Move);
      Target.removeEventListener("pointerup", Move);
      Target.removeEventListener("pointerleave", Leave);
      Target.removeEventListener("pointercancel", Leave);
      Target.removeEventListener("wheel", Wheel);
      window.removeEventListener("scroll", Scrolled, true);
      window.removeEventListener("resize", Scrolled);
      Picture?.removeEventListener("load", Scrolled);
      // The pointer's own state is kept: a Watch change re-attaches over the same spot, while a
      // disabled magnifier clears it above and pointerleave clears it when the pointer goes.
      DrawRef.current();
    };
  }, [Enabled, Live, HideWhilePressed, ListenRef, ElementRef, Watch]);

  // A new aim, mirror or marker set redraws in place.
  useEffect(() => { if (Pointer.current.Inside) DrawRef.current(); }, [Aim, MirroredDisplay, Markers]);

  return createPortal(
    <div ref={BoxRef} className="hover-loupe" role="img" aria-label={Name} data-visible="false" data-loupe={Name}>
      <canvas ref={CanvasRef} />
      <div ref={ReadoutRef} className="hover-loupe-readout" />
    </div>,
    document.body,
  );
}
