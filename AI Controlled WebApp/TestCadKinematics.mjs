import { build as Build } from "esbuild";
import { readFileSync as ReadFile, writeFileSync as WriteFile } from "node:fs";
import { createRequire as CreateRequire } from "node:module";
import Assert from "node:assert/strict";
import Path from "node:path";
import { fileURLToPath as FilePath } from "node:url";
const Root = Path.dirname(FilePath(import.meta.url));
const Bundle = await Build({ entryPoints: [Path.join(Root, "src/lib/CadKinematics.ts")], bundle: true, platform: "node", format: "cjs", write: false });
const Module = { exports: {} };
new Function("require", "module", "exports", Bundle.outputFiles[0].text)(CreateRequire(import.meta.url), Module, Module.exports);
const Cad = Module.exports;
const Fixtures = ["CadFixtures.json", "CadBoundedFixtures.json"].flatMap(Name => JSON.parse(ReadFile(Path.join(Root, "../back end/TestArtifacts", Name), "utf8")));
let MaximumFkError = 0, MaximumIkError = 0;
const Started = performance.now();
for (const Fixture of Fixtures) {
  const State = Cad.CadForward(Fixture.Angles);
  State.Joints.forEach((Point, Index) => {
    for (const Axis of ["x", "y", "z"]) {
      const Error = Math.abs(Point[Axis] - Fixture.Joints[Index][Axis]);
      Assert.ok(Error < 1e-9); MaximumFkError = Math.max(MaximumFkError, Error);
    }
  });
  const Result = Cad.CadSolve(Fixture.Target, Fixture.Pitch, Fixture.Elbow, Fixture.Initial, Fixture.Limits);
  Assert.ok(Result.Reachable, JSON.stringify({ Fixture, Result }));
  Assert.ok(Result.ErrorCm < 1e-4, JSON.stringify({ Fixture, Result }));
  Assert.ok(Result.PitchError < 1e-4, Result.PitchError);
  MaximumIkError = Math.max(MaximumIkError, Result.ErrorCm);
  for (const Name of ["wrist_roll", "gripper"]) Assert.ok(Math.abs(Result.Angles[Name] - Fixture.Initial[Name]) < 1e-8);
}
Assert.equal(Cad.CadSolve({ x: 1000, y: 1000, z: 1000 }, 0, "up").Reachable, false);
Assert.throws(() => Cad.CadSolve({ x: NaN, y: 0, z: 0 }, 0, "up"));
const Results = { PoseCount: Fixtures.length, MaximumFkDifferenceMm: MaximumFkError * 10, MaximumIkErrorMm: MaximumIkError * 10, SolveSeconds: (performance.now() - Started) / 1000 };
WriteFile(Path.join(Root, "../back end/TestArtifacts/CadFrontendVerification.json"), JSON.stringify(Results, null, 2));
console.log(Results);
