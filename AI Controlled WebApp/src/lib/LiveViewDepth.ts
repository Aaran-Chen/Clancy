export type ElementBox = { left: number; top: number; width: number; height: number };
export type DepthReading = { ok: boolean; depth_cm?: number; display_cm?: { x: number; y: number; z: number }; error?: string };

// The video pixel under a pointer, for a <video> drawn with object-fit (contain, cover or fill);
// null over the letterbox bars around the picture.
export function VideoPixelAt(ClientX: number, ClientY: number, Box: ElementBox, VideoWidth: number, VideoHeight: number,
  Fit: string): { U: number; V: number } | null {
  if (!(VideoWidth > 0 && VideoHeight > 0 && Box.width > 0 && Box.height > 0)) return null;
  const Across = Box.width / VideoWidth, Down = Box.height / VideoHeight;
  const Scale = Fit === "cover" ? Math.max(Across, Down) : Math.min(Across, Down);
  const [ScaleX, ScaleY] = Fit === "fill" ? [Across, Down] : [Scale, Scale];
  const U = (ClientX - Box.left - (Box.width - VideoWidth * ScaleX) / 2) / ScaleX;
  const V = (ClientY - Box.top - (Box.height - VideoHeight * ScaleY) / 2) / ScaleY;
  return U >= 0 && V >= 0 && U < VideoWidth && V < VideoHeight ? { U, V } : null;
}

// The bottom-left readout for GET /depthcam/depth-at; the arm-frame position appears once calibrated.
export function DepthReadingText(Reading: DepthReading): string {
  if (!Reading.ok || typeof Reading.depth_cm !== "number") {
    return /pixel/.test(Reading.error ?? "") ? "No depth reading here" : Reading.error ?? "Depth reading unavailable";
  }
  const Depth = `Depth ${Reading.depth_cm.toFixed(1)} cm`, At = Reading.display_cm;
  return At ? `${Depth} · X ${At.x.toFixed(1)}, Y ${At.y.toFixed(1)}, Z ${At.z.toFixed(1)} cm` : Depth;
}
