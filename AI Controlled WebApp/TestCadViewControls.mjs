import { build as Build } from "esbuild";
import { createRequire as CreateRequire } from "node:module";
import { test as Test } from "node:test";
import Assert from "node:assert/strict";
import Path from "node:path";
import { fileURLToPath as FilePath } from "node:url";

const Root = Path.dirname(FilePath(import.meta.url));
const Bundle = await Build({ stdin: { contents: `export * from './CadViewControls'; export * from './CadKinematics';
  export * from './robotModel';`, resolveDir: Path.join(Root, "src/lib"), loader: "ts" },
  bundle: true, platform: "node", format: "cjs", write: false });
const Module = { exports: {} };
new Function("require", "module", "exports", Bundle.outputFiles[0].text)(CreateRequire(import.meta.url), Module, Module.exports);
const Api = Module.exports;
const Delta = (X, Y, Mode = 0) => Api.WristWheelDelta({ deltaX: X, deltaY: Y, deltaMode: Mode }, 600);

Test("wheel pitch is proportional to distance, independent of event count or scroll axis", () => {
  Assert.equal(Delta(0, 100), Api.WristWheelStepDegrees);
  Assert.equal(Delta(100, 0), Delta(0, 100), "Windows horizontal Shift+wheel works");
  Assert.ok(Math.abs(Array.from({ length: 20 }, () => Delta(0, 5)).reduce((A, B) => A + B, 0) - Delta(0, 100)) < 1e-12);
  Assert.equal(Delta(0, -100), -Delta(0, 100));
  Assert.equal(Delta(80, 10), Delta(80, 0), "Diagonal events are not double counted");
  Assert.equal(Delta(0, 0), 0);
  Assert.equal(Delta(0, NaN), 0);
});

Test("line and page wheel deltas use the same pixel-normalized scale", () => {
  Assert.equal(Delta(0, 3, 1), Delta(0, 48));
  Assert.equal(Delta(1, 0, 2), Delta(600, 0));
});

Test("approach direction follows the CAD tool axis for rotated, folded and rolled poses", () => {
  const Initial = Api.currentAngles(Api.INITIAL_SERVOS);
  for (const Pan of [-150, 0, 150]) for (const Lift of [-90, 45]) for (const Roll of [0, 90]) {
    const Angles = { ...Initial, shoulder_pan: Pan, shoulder_lift: Lift, wrist_roll: Roll };
    const Frame = Api.CadForward(Angles).Links[Api.CadModel.ToolLink].elements;
    const Direction = Api.TargetApproachDirection(Angles, Api.CadPitch(Angles));
    Assert.ok(Math.hypot(Direction.x - Frame[8], Direction.y - Frame[9], Direction.z - Frame[10]) < 1e-8);
    Assert.ok(Math.abs(Direction.length() - 1) < 1e-12);
    const Turned = Api.TargetApproachDirection(Angles, Api.CadPitch(Angles) + 30);
    Assert.ok(Math.abs(Direction.dot(Turned) - Math.cos(Math.PI / 6)) < 1e-8);
  }
});
