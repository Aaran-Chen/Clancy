export type CameraBounds = { X: number; Y: number; Width: number; Height: number };
export type CameraViewport = { Width: number; Height: number; Top: number };
export type ResizeEdge = "n" | "s" | "e" | "w" | "ne" | "nw" | "se" | "sw";
export const ResizeEdges: ResizeEdge[] = ["n", "s", "e", "w", "ne", "nw", "se", "sw"];
export const MinimizedCameraSize = 48;
export const CameraWindowStorageKey = "SO101.CameraWindow.v1";
const Margin = 12;
const SizeLimits = (Viewport: CameraViewport) => {
  const MaxWidth = Math.max(1, Viewport.Width - Margin * 2), MaxHeight = Math.max(44, Viewport.Height - Viewport.Top - Margin);
  return { MaxWidth, MaxHeight, MinWidth: Math.min(260, MaxWidth), MinHeight: Math.min(190, MaxHeight) };
};

export function ClampCameraBounds(Bounds: CameraBounds, Viewport: CameraViewport, Minimized = false): CameraBounds {
  const { MaxWidth, MaxHeight, MinWidth, MinHeight } = SizeLimits(Viewport);
  const Width = Math.min(MaxWidth, Math.max(MinWidth, Bounds.Width));
  const Height = Math.min(MaxHeight, Math.max(MinHeight, Bounds.Height));
  return { Width, Height, X: Math.max(Margin, Math.min(Bounds.X, Viewport.Width - (Minimized ? MinimizedCameraSize : Width) - Margin)),
    Y: Math.max(Viewport.Top, Math.min(Bounds.Y, Viewport.Height - (Minimized ? MinimizedCameraSize : Height) - Margin)) };
}

// Dragging an edge or corner moves only that edge: the opposite edge stays put, even at the size limits.
export function ResizeCameraBounds(Start: CameraBounds, Edge: ResizeEdge, DeltaX: number, DeltaY: number, Viewport: CameraViewport): CameraBounds {
  const { MinWidth, MinHeight } = SizeLimits(Viewport);
  let Left = Start.X, Top = Start.Y, Right = Start.X + Start.Width, Bottom = Start.Y + Start.Height;
  if (Edge.includes("w")) Left = Math.min(Right - MinWidth, Math.max(Margin, Left + DeltaX));
  if (Edge.includes("e")) Right = Math.max(Left + MinWidth, Math.min(Viewport.Width - Margin, Right + DeltaX));
  if (Edge.includes("n")) Top = Math.min(Bottom - MinHeight, Math.max(Viewport.Top, Top + DeltaY));
  if (Edge.includes("s")) Bottom = Math.max(Top + MinHeight, Math.min(Viewport.Height - Margin, Bottom + DeltaY));
  return ClampCameraBounds({ X: Left, Y: Top, Width: Right - Left, Height: Bottom - Top }, Viewport);
}

// Minimizing leaves the tile where the window's top-right corner was (the minimize button's side);
// restoring grows the window back out from the tile's top-right corner, so the two round-trip.
export function MinimizeCameraBounds(Window: CameraBounds, Viewport: CameraViewport): CameraBounds {
  return ClampCameraBounds({ ...Window, X: Window.X + Window.Width - MinimizedCameraSize }, Viewport, true);
}
export function RestoreCameraBounds(Tile: CameraBounds, Viewport: CameraViewport): CameraBounds {
  return ClampCameraBounds({ ...Tile, X: Tile.X + MinimizedCameraSize - Tile.Width }, Viewport);
}

export type ScreenRect = { Left: number; Top: number; Right: number; Bottom: number };
// The window (or its minimized tile) moved the least way, up, down, left or right, to stand Margin clear of every rect in
// Avoid (the buttons the page marks data-camera-keep-clear: an AI step's Approve and Block), inside the viewport. The
// same bounds when it is clear already or no such place exists. At 1600 x 1000 the window at its default place, bottom
// left, covered the foot of a tall approval card, its Approve and Block included (2026-10-08 review).
export function ClearOfRects(Bounds: CameraBounds, Avoid: ScreenRect[], Viewport: CameraViewport, Minimized = false): CameraBounds {
  const Width = Minimized ? MinimizedCameraSize : Bounds.Width, Height = Minimized ? MinimizedCameraSize : Bounds.Height;
  const Hits = (B: CameraBounds) => Avoid.filter(R => B.X < R.Right + Margin && B.X + Width > R.Left - Margin && B.Y < R.Bottom + Margin && B.Y + Height > R.Top - Margin);
  const Blocking = Hits(Bounds);
  if (!Blocking.length) return Bounds;
  const Left = Math.min(...Blocking.map(R => R.Left)), Right = Math.max(...Blocking.map(R => R.Right));
  const Top = Math.min(...Blocking.map(R => R.Top)), Bottom = Math.max(...Blocking.map(R => R.Bottom));
  const Places = [{ ...Bounds, Y: Top - Height - Margin }, { ...Bounds, Y: Bottom + Margin }, { ...Bounds, X: Left - Width - Margin },
    { ...Bounds, X: Right + Margin }].map(B => ClampCameraBounds(B, Viewport, Minimized)).filter(B => !Hits(B).length);
  const Distance = (B: CameraBounds) => Math.hypot(B.X - Bounds.X, B.Y - Bounds.Y);
  return Places.length ? Places.reduce((Best, B) => (Distance(B) < Distance(Best) ? B : Best)) : Bounds;
}

export function ReadCameraBounds(Viewport: CameraViewport): CameraBounds {
  const Default = { X: 92, Y: Viewport.Height - 300, Width: 360, Height: 280 };
  try {
    const Saved = JSON.parse(localStorage.getItem(CameraWindowStorageKey) ?? "null");
    if (Saved && ["X", "Y", "Width", "Height"].every(Key => typeof Saved[Key] === "number" && Number.isFinite(Saved[Key]))) {
      return ClampCameraBounds(Saved, Viewport, localStorage.getItem(`${CameraWindowStorageKey}.Minimized`) === "true");
    }
  } catch { /* Layout remains usable when storage is unavailable. */ }
  return ClampCameraBounds(Default, Viewport);
}

export function SaveCameraBounds(Bounds: CameraBounds) {
  try { localStorage.setItem(CameraWindowStorageKey, JSON.stringify(Bounds)); } catch { /* Optional preference. */ }
}
