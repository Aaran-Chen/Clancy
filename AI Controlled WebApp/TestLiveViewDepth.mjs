import { build as Build } from "esbuild";
import { test as Test } from "node:test";
import Assert from "node:assert/strict";
const Bundle = await Build({ entryPoints: ["src/lib/LiveViewDepth.ts"], bundle: true, write: false, platform: "node", format: "esm" });
const Api = await import(`data:text/javascript;base64,${Buffer.from(Bundle.outputFiles[0].text).toString("base64")}`);

Test("pointer maps to the video pixel under it, skipping letterbox bars", () => {
  // A 1280x720 picture in a 400x400 box: contain leaves bars top and bottom, 225 px of picture.
  const Box = { left: 100, top: 50, width: 400, height: 400 };
  Assert.deepEqual(Api.VideoPixelAt(300, 250, Box, 1280, 720, "contain"), { U: 640, V: 360 });
  Assert.deepEqual(Api.VideoPixelAt(100, 137.5, Box, 1280, 720, "contain"), { U: 0, V: 0 });
  Assert.equal(Api.VideoPixelAt(300, 60, Box, 1280, 720, "contain"), null);  // on the top bar
  Assert.equal(Api.VideoPixelAt(500, 250, Box, 1280, 720, "contain"), null);  // right edge is outside
  // Cover crops the sides instead: the box centre is still the picture centre, the box edge is not the picture edge.
  Assert.deepEqual(Api.VideoPixelAt(300, 250, Box, 1280, 720, "cover"), { U: 640, V: 360 });
  const Corner = Api.VideoPixelAt(100, 50, Box, 1280, 720, "cover");  // 155.6 px of picture cropped off each side
  Assert.ok(Math.abs(Corner.U - 280) < 1e-9 && Corner.V === 0, JSON.stringify(Corner));
  Assert.deepEqual(Api.VideoPixelAt(200, 150, Box, 1280, 720, "fill"), { U: 320, V: 180 });
  Assert.equal(Api.VideoPixelAt(300, 250, Box, 0, 0, "contain"), null);  // no frame yet
});

Test("readout text", () => {
  Assert.equal(Api.DepthReadingText({ ok: true, depth_cm: 63.456 }), "Depth 63.5 cm");
  Assert.equal(Api.DepthReadingText({ ok: true, depth_cm: 48.2, display_cm: { x: -15, y: 28.04, z: 1.96 } }),
    "Depth 48.2 cm · X -15.0, Y 28.0, Z 2.0 cm");
  Assert.equal(Api.DepthReadingText({ ok: false, error: "no depth at that pixel" }), "No depth reading here");
  Assert.equal(Api.DepthReadingText({ ok: false, error: "pixel outside the depth image" }), "No depth reading here");
  Assert.equal(Api.DepthReadingText({ ok: false, error: "The depth camera is not streaming." }), "The depth camera is not streaming.");
  Assert.equal(Api.DepthReadingText({ ok: false }), "Depth reading unavailable");
});
