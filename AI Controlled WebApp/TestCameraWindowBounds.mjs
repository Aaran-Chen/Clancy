import { build as Build } from "esbuild";
import { test as Test } from "node:test";
import Assert from "node:assert/strict";
const Bundle = await Build({ entryPoints: ["src/lib/CameraWindowBounds.ts"], bundle: true, write: false, platform: "node", format: "esm" });
const Api = await import(`data:text/javascript;base64,${Buffer.from(Bundle.outputFiles[0].text).toString("base64")}`);
Test("floating window remains reachable at screen edges and after viewport changes", () => {
  for (const Width of [280, 480, 1000, 1920, 3840]) for (const Height of [360, 720, 1080]) {
    for (const X of [-1e6, 12, 500, 1e6]) for (const Y of [-1e6, 68, 500, 1e6]) for (const Minimized of [false, true]) {
      const Bounds = Api.ClampCameraBounds({ X, Y, Width: 1600, Height: 900 }, { Width, Height, Top: 68 }, Minimized);
      Assert.ok(Bounds.X >= 12 && Bounds.Y >= 68);
      Assert.ok(Bounds.X + (Minimized ? Api.MinimizedCameraSize : Bounds.Width) <= Width - 12);
      Assert.ok(Bounds.Y + (Minimized ? Api.MinimizedCameraSize : Bounds.Height) <= Height - 12);
    }
  }
});
Test("invalid or blocked storage falls back safely; saved geometry restores", () => {
  const Viewport = { Width: 1920, Height: 1080, Top: 68 };
  for (const Stored of ["bad json", "null", "{}", '{"X":"4"}', '{"X":0,"Y":0,"Width":1e999,"Height":20}']) {
    globalThis.localStorage = { getItem: () => Stored, setItem: () => { throw Error("Blocked"); } };
    const Bounds = Api.ReadCameraBounds(Viewport);
    Assert.deepEqual(Bounds, { X: 92, Y: 780, Width: 360, Height: 280 });
    Assert.doesNotThrow(() => Api.SaveCameraBounds(Bounds));
  }
  const Saved = { X: 250, Y: 300, Width: 470, Height: 350 };
  globalThis.localStorage = { getItem: () => JSON.stringify(Saved) };
  Assert.deepEqual(Api.ReadCameraBounds(Viewport), Saved);
});

Test("the window stands clear of the page's marked buttons (an AI step's Approve and Block), the least way, inside the screen", () => {
  const Viewport = { Width: 1600, Height: 1000, Top: 68 };
  const Window = Api.ClampCameraBounds({ X: 92, Y: 700, Width: 360, Height: 280 }, Viewport);  // its default place, bottom left
  const Overlap = (B, R, Size) => B.X < R.Right + 12 && B.X + (Size ?? B.Width) > R.Left - 12 && B.Y < R.Bottom + 12 && B.Y + (Size ?? B.Height) > R.Top - 12;
  const Buttons = { Left: 40, Top: 900, Right: 380, Bottom: 940 };  // the foot of a tall card in the left column
  const Moved = Api.ClearOfRects(Window, [Buttons], Viewport);
  Assert.ok(!Overlap(Moved, Buttons), JSON.stringify(Moved));
  Assert.deepEqual([Moved.Width, Moved.Height], [Window.Width, Window.Height], "moved, never resized");
  Assert.ok(Moved.X >= 12 && Moved.Y >= 68 && Moved.X + Moved.Width <= 1588 && Moved.Y + Moved.Height <= 988);
  Assert.equal(Moved.X, Window.X, "up is the least way here");
  Assert.deepEqual(Api.ClearOfRects(Window, [{ Left: 900, Top: 100, Right: 1000, Bottom: 140 }], Viewport), Window, "clear already: unchanged");
  Assert.deepEqual(Api.ClearOfRects(Window, [], Viewport), Window);
  // Two buttons side by side are kept clear of together; the minimized tile too.
  const Pair = [{ Left: 60, Top: 760, Right: 200, Bottom: 800 }, { Left: 210, Top: 760, Right: 350, Bottom: 800 }];
  const Both = Api.ClearOfRects(Window, Pair, Viewport);
  Assert.ok(Pair.every(R => !Overlap(Both, R)), JSON.stringify(Both));
  const Tile = Api.ClampCameraBounds({ X: 100, Y: 900, Width: 360, Height: 280 }, Viewport, true);
  const TileMoved = Api.ClearOfRects(Tile, [{ Left: 90, Top: 890, Right: 160, Bottom: 960 }], Viewport, true);
  Assert.ok(!Overlap(TileMoved, { Left: 90, Top: 890, Right: 160, Bottom: 960 }, Api.MinimizedCameraSize), JSON.stringify(TileMoved));
  // Nowhere to go (the buttons fill the screen): it stays where it is rather than leave the screen.
  const Everywhere = { Left: 0, Top: 0, Right: 1600, Bottom: 1000 };
  Assert.deepEqual(Api.ClearOfRects(Window, [Everywhere], Viewport), Window);
});

Test("every edge and corner resizes, holding the opposite edge still even at the limits", () => {
  const Viewport = { Width: 1920, Height: 1080, Top: 68 }, Start = { X: 500, Y: 400, Width: 400, Height: 300 };
  const Edges = B => ({ Left: B.X, Top: B.Y, Right: B.X + B.Width, Bottom: B.Y + B.Height });
  Assert.deepEqual(Api.ResizeCameraBounds(Start, "se", 110, 70, Viewport), { X: 500, Y: 400, Width: 510, Height: 370 });
  Assert.deepEqual(Api.ResizeCameraBounds(Start, "w", -100, 999, Viewport), { X: 400, Y: 400, Width: 500, Height: 300 });
  Assert.deepEqual(Api.ResizeCameraBounds(Start, "n", 999, -50, Viewport), { X: 500, Y: 350, Width: 400, Height: 350 });
  Assert.deepEqual(Api.ResizeCameraBounds(Start, "nw", 1e6, 1e6, Viewport), { X: 640, Y: 510, Width: 260, Height: 190 });  // shrinks to the minimum
  Assert.deepEqual(Api.ResizeCameraBounds(Start, "ne", 1e6, -1e6, Viewport), { X: 500, Y: 68, Width: 1408, Height: 632 });  // stops at the screen
  for (const Edge of Api.ResizeEdges) for (const DeltaX of [-1e6, -150, 0, 150, 1e6]) for (const DeltaY of [-1e6, -150, 0, 150, 1e6]) {
    const Before = Edges(Start), After = Edges(Api.ResizeCameraBounds(Start, Edge, DeltaX, DeltaY, Viewport));
    for (const [Side, Letter] of [["Left", "w"], ["Right", "e"], ["Top", "n"], ["Bottom", "s"]]) {
      if (!Edge.includes(Letter)) Assert.equal(After[Side], Before[Side], `${Edge} drag must not move the ${Side} edge`);
    }
    Assert.ok(After.Left >= 12 && After.Top >= 68 && After.Right <= 1908 && After.Bottom <= 1068);
    Assert.ok(After.Right - After.Left >= 260 && After.Bottom - After.Top >= 190);
  }
});

Test("minimize and restore both anchor on the top-right corner and round-trip", () => {
  const Viewport = { Width: 1920, Height: 1080, Top: 68 }, Window = { X: 700, Y: 400, Width: 360, Height: 280 };
  const Tile = Api.MinimizeCameraBounds(Window, Viewport);
  Assert.deepEqual(Tile, { X: 1012, Y: 400, Width: 360, Height: 280 });  // tile's top-right = window's top-right (1060, 400)
  Assert.deepEqual(Api.RestoreCameraBounds(Tile, Viewport), Window);
  // A tile moved elsewhere opens with the window's top-right corner at the tile's top-right corner.
  Assert.deepEqual(Api.RestoreCameraBounds({ ...Tile, X: 1500, Y: 700 }, Viewport), { X: 1188, Y: 700, Width: 360, Height: 280 });
  // Near the left or bottom edge the window still opens fully on screen.
  Assert.deepEqual(Api.RestoreCameraBounds({ ...Tile, X: 20, Y: 1020 }, Viewport), { X: 12, Y: 788, Width: 360, Height: 280 });
});

Test("minimized tile retains full size and can persist at the far right edge", () => {
  const Viewport = { Width: 1920, Height: 1080, Top: 68 };
  const Saved = Api.ClampCameraBounds({ X: 9999, Y: 9999, Width: 470, Height: 350 }, Viewport, true);
  Assert.deepEqual(Saved, { X: 1860, Y: 1020, Width: 470, Height: 350 });
  globalThis.localStorage = { getItem: Key => Key.endsWith('.Minimized') ? 'true' : JSON.stringify(Saved) };
  Assert.deepEqual(Api.ReadCameraBounds(Viewport), Saved);
  Assert.deepEqual(Api.ClampCameraBounds(Saved, Viewport), { X: 1438, Y: 718, Width: 470, Height: 350 });
});
