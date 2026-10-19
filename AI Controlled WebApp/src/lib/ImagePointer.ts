import { VideoPixelAt, type ElementBox } from "./LiveViewDepth";

// One mapping from a pointer to the picture under it, shared by every click
// surface and the hover magnifier (components/HoverLoupe.tsx), so the pixel the
// magnifier's crosshair marks is exactly the pixel a click records.

export type ImageFit = "contain" | "cover" | "fill";
// U, V: continuous position in the SOURCE image (its own pixels, 0..Width, 0..Height),
// before any display mirroring. Column, Row: the whole pixel that contains it.
export type SourcePoint = { U: number; V: number; Column: number; Row: number };

// The content box (inside border and padding) of an element as drawn on screen,
// in client pixels. Ancestor transforms that scale the element are honoured;
// a scaleX(-1) mirror keeps the box where it is.
export function ContentBox(Element: HTMLElement): ElementBox {
  const Rect = Element.getBoundingClientRect();
  const Style = getComputedStyle(Element);
  const Number_ = (Value: string) => Number.parseFloat(Value) || 0;
  const ScaleX = Element.offsetWidth > 0 ? Rect.width / Element.offsetWidth : 1;
  const ScaleY = Element.offsetHeight > 0 ? Rect.height / Element.offsetHeight : 1;
  const Left = Number_(Style.borderLeftWidth) + Number_(Style.paddingLeft);
  const Right = Number_(Style.borderRightWidth) + Number_(Style.paddingRight);
  const Top = Number_(Style.borderTopWidth) + Number_(Style.paddingTop);
  const Bottom = Number_(Style.borderBottomWidth) + Number_(Style.paddingBottom);
  return { left: Rect.left + Left * ScaleX, top: Rect.top + Top * ScaleY,
    width: Math.max(0, Rect.width - (Left + Right) * ScaleX), height: Math.max(0, Rect.height - (Top + Bottom) * ScaleY) };
}

export function FitOf(Element: HTMLElement): ImageFit {
  const Fit = getComputedStyle(Element).objectFit;
  return Fit === "contain" || Fit === "cover" ? Fit : "fill";
}

// The source point under a pointer on an element that draws a Width x Height
// image with object-fit Fit (letterbox bars give null) and, when MirroredDisplay,
// flipped left-right (CSS scaleX(-1)).
export function SourcePointAt(ClientX: number, ClientY: number, Box: ElementBox, Width: number, Height: number,
  Fit: ImageFit, MirroredDisplay: boolean): SourcePoint | null {
  const Shown = VideoPixelAt(ClientX, ClientY, Box, Width, Height, Fit);
  if (!Shown) return null;
  const U = MirroredDisplay ? Width - Shown.U : Shown.U;
  return { U, V: Shown.V, Column: Math.min(Width - 1, Math.max(0, Math.floor(U))),
    Row: Math.min(Height - 1, Math.max(0, Math.floor(Shown.V))) };
}

// The source point under a pointer on a displayed <img> (its decoded natural size).
export function ImagePointAt(Image: HTMLImageElement, ClientX: number, ClientY: number, MirroredDisplay: boolean): SourcePoint | null {
  if (!(Image.naturalWidth > 0 && Image.naturalHeight > 0)) return null;
  return SourcePointAt(ClientX, ClientY, ContentBox(Image), Image.naturalWidth, Image.naturalHeight, FitOf(Image), MirroredDisplay);
}

// Client position of a source point: the inverse of SourcePointAt.
export function ClientPointOf(U: number, V: number, Box: ElementBox, Width: number, Height: number,
  Fit: ImageFit, MirroredDisplay: boolean): { X: number; Y: number } {
  const Across = Box.width / Width, Down = Box.height / Height;
  const Scale = Fit === "cover" ? Math.max(Across, Down) : Math.min(Across, Down);
  const [ScaleX, ScaleY] = Fit === "fill" ? [Across, Down] : [Scale, Scale];
  const Shown = MirroredDisplay ? Width - U : U;
  return { X: Box.left + (Box.width - Width * ScaleX) / 2 + Shown * ScaleX,
    Y: Box.top + (Box.height - Height * ScaleY) / 2 + V * ScaleY };
}

// Client pixels per source pixel as displayed (the magnifier's 1x).
export function DisplayScale(Box: ElementBox, Width: number, Height: number, Fit: ImageFit): number {
  const Across = Box.width / Width, Down = Box.height / Height;
  return Fit === "cover" ? Math.max(Across, Down) : Fit === "fill" ? Math.sqrt(Across * Down) : Math.min(Across, Down);
}

// What a click records: the centre of the clicked whole pixel, in the continuous
// capture space the bridge's probe and click endpoints take (u, v in 0..Width).
// The bridge's DepthGeometry.NativeFromCapture turns it into the integer native pixel.
export function ClickCapturePoint(Point: SourcePoint): { U: number; V: number } {
  return { U: Point.Column + 0.5, V: Point.Row + 0.5 };
}

// The magnifier's placement: Offset px right of and below the pointer, flipped to
// the other side near a viewport edge, so the box stays fully visible and never
// covers the point being aimed at.
export function LoupePlacement(ClientX: number, ClientY: number, Size: number, ViewportWidth: number, ViewportHeight: number,
  Offset = 24, Margin = 6): { Left: number; Top: number; FlippedX: boolean; FlippedY: boolean } {
  const FlippedX = ClientX + Offset + Size > ViewportWidth - Margin && ClientX - Offset - Size >= Margin;
  const FlippedY = ClientY + Offset + Size > ViewportHeight - Margin && ClientY - Offset - Size >= Margin;
  const Left = FlippedX ? ClientX - Offset - Size : Math.min(ClientX + Offset, Math.max(Margin, ViewportWidth - Margin - Size));
  const Top = FlippedY ? ClientY - Offset - Size : Math.min(ClientY + Offset, Math.max(Margin, ViewportHeight - Margin - Size));
  return { Left, Top, FlippedX, FlippedY };
}

// The source rectangle the magnifier shows: centred on the hovered pixel's centre,
// Size / (DisplayScale x Zoom) source pixels across.
export function LoupeSourceRect(Point: SourcePoint, Size: number, Scale: number, Zoom: number) {
  const Across = Size / Math.max(1e-6, Scale * Zoom);
  return { X: Point.Column + 0.5 - Across / 2, Y: Point.Row + 0.5 - Across / 2, Width: Across, Height: Across };
}

export const LoupeMinZoom = 2;
export const LoupeMaxZoom = 8;
export const LoupeDefaultZoom = 4;
export function ClampZoom(Zoom: number): number {
  return Math.min(LoupeMaxZoom, Math.max(LoupeMinZoom, Math.round(Zoom)));
}
