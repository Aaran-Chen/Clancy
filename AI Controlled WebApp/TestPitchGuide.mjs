import { build as Build } from "esbuild";
import { createRequire as CreateRequire } from "node:module";
import { test as Test } from "node:test";
import Assert from "node:assert/strict";
import Path from "node:path";
import { fileURLToPath as FilePath } from "node:url";

const Root = Path.dirname(FilePath(import.meta.url));
const Bundle = await Build({ stdin: { contents: `export * from './PitchGuide'; export * from './robotModel';
  export * from './CadKinematics'; export * from './BasketCollision';`, resolveDir: Path.join(Root, "src/lib"), loader: "ts" },
  bundle: true, platform: "node", format: "cjs", write: false });
const Module = { exports: {} };
new Function("require", "module", "exports", Bundle.outputFiles[0].text)(CreateRequire(import.meta.url), Module, Module.exports);
const Api = Module.exports;
const Context = { Servos: Api.INITIAL_SERVOS, ElbowMode: "down" };
const Target = Api.HOME_MODEL_TARGET_CM;

Test("green guide points and segment middles pass the actual IK and basket guards", () => {
  const Segments = Api.BuildPitchGuide(Target, Context);
  Assert.ok(Segments.length > 0);
  for (const { Start, End } of Segments) {
    for (const Pitch of [Start.Pitch, (Start.Pitch + End.Pitch) / 2, End.Pitch]) {
      const Preview = Api.solveIkPreview({ ...Target, wristPitch: Pitch }, Context.ElbowMode, Context.Servos);
      Assert.ok(Preview.reachable, `Green pitch ${Pitch} must be accepted`);
      Assert.equal(Preview.collisionReason, null);
    }
    for (const Point of [Start, End]) Assert.ok(Math.abs(Math.hypot(...Point.Direction) - 1) < 1e-8);
  }
  const Lowest = Segments[0].Start.Pitch, Highest = Segments.at(-1).End.Pitch;
  for (const Pitch of [Lowest - .01, Highest + .01]) {
    Assert.equal(Api.solveIkPreview({ ...Target, wristPitch: Pitch }, Context.ElbowMode, Context.Servos).reachable, false,
      "The ends of the green arc stop at the allowed boundary");
  }
});

Test("no green angles appear inside the basket or outside all physical reach", () => {
  Assert.deepEqual(Api.BuildPitchGuide({ ...Target, x: 0, y: 0, z: -10 }, Context), []);
  Assert.deepEqual(Api.BuildPitchGuide({ ...Target, x: 150 }, Context), []);
});

Test("a fixed wrist range is honored and cannot inherit an older green arc", () => {
  const Angles = Api.currentAngles(Context.Servos);
  const Limits = Object.fromEntries(Api.IK_JOINTS.map(Name => [Name, [Angles[Name], Angles[Name]]]));
  const Segments = Api.BuildPitchGuide(Target, { ...Context, Limits });
  for (const Segment of Segments) for (const Point of [Segment.Start, Segment.End]) {
    Assert.ok(Api.solveIkPreview({ ...Target, wristPitch: Point.Pitch }, Context.ElbowMode, Context.Servos, Limits).reachable);
  }
  const CoveredDegrees = Segments.reduce((Total, Segment) => Total + Segment.End.Pitch - Segment.Start.Pitch, 0);
  Assert.ok(CoveredDegrees <= .5, "A fixed pose stays within the solver's +/-0.25 degree pitch tolerance");
});
