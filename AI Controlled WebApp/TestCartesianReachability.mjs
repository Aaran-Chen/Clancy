import { build as Build } from 'esbuild';
import { createRequire as CreateRequire } from 'node:module';
import { test as Test } from 'node:test';
import Assert from 'node:assert/strict';
import Path from 'node:path';
import { fileURLToPath as FilePath } from 'node:url';

const Root = Path.dirname(FilePath(import.meta.url));
const Bundle = await Build({ stdin: { contents: `export * from './CadKinematics'; export * from './robotModel';
export * from './BasketCollision'; export * from './PitchGuide'; export { TaskObjectReachability } from './AiMeasurements';`, resolveDir: Path.join(Root, 'src/lib'), loader: 'ts' },
bundle: true, platform: 'node', format: 'cjs', write: false });
const Module = { exports: {} };
new Function('require', 'module', 'exports', Bundle.outputFiles[0].text)(CreateRequire(import.meta.url), Module, Module.exports);
const Api = Module.exports;
const Target = { x: -12.441904167611757, y: 6.413114025735255, z: 20.974084794840152, wristPitch: 149.67387933428267 };
const Limits = Object.fromEntries(Api.IK_JOINTS.map(Name => [Name, [-170, 170]]));
const Context = { Servos: Api.INITIAL_SERVOS, ElbowMode: 'down', Limits };

Test('a colliding preferred branch cannot hide a clear pose at the same XYZ and pitch', () => {
  const Initial = Api.currentAngles(Api.INITIAL_SERVOS);
  const OldChoice = Api.CadSolve(Target, Target.wristPitch, 'down', Initial, Limits);
  Assert.ok(OldChoice.Reachable);
  Assert.ok(Api.BasketPoseReason(OldChoice.Angles));
  const Preview = Api.solveIkPreview(Target, 'down', Api.INITIAL_SERVOS, Limits);
  Assert.ok(Preview.reachable);
  Assert.equal(Preview.collisionReason, null);
  Assert.ok(Preview.errorCm < .0001);
  Assert.ok(Math.abs(Preview.effectiveWristPitch - Target.wristPitch) < .0001);
  for (const Name of ['wrist_roll', 'gripper']) Assert.equal(Preview.angles[Name], Initial[Name]);
});

Test('the green wrist guide includes the recovered angle without moving the target', () => {
  const Segments = Api.BuildPitchGuide(Target, Context);
  Assert.ok(Segments.some(Segment => Segment.Start.Pitch <= Target.wristPitch && Segment.End.Pitch >= Target.wristPitch));
  for (const Segment of Segments) {
    const Pitch = (Segment.Start.Pitch + Segment.End.Pitch) / 2;
    const Preview = Api.solveIkPreview({ ...Target, wristPitch: Pitch }, 'down', Api.INITIAL_SERVOS, Limits);
    Assert.ok(Preview.reachable, `Green segment midpoint ${Pitch} must be reachable`);
    Assert.ok(Preview.errorCm < .1);
  }
});

Test('an interior basket endpoint remains valid even though Send needs a detour', () => {
  const Interior = { x: -1.9532211879722914, y: 14.308657609320962, z: -5.741477864378066, wristPitch: -108.0 };
  Assert.equal(Api.BasketTargetReason(Interior), null);
  const Preview = Api.solveIkPreview(Interior, 'down', Api.INITIAL_SERVOS);
  Assert.ok(Preview.reachable);
  Assert.equal(Preview.collisionReason, null);
});

Test('the task object measured at X 16.1, Y 35.8 is tagged OUT OF REACH for the right arm', () => {
  // The 19:04:11 eraser reading (a camera phantom): 46.3 cm from the right
  // arm's shoulder at display X -13.3, Y 0, beyond reach at every wrist pitch.
  const Eraser = { label: 'eraser', x: 16.1, y: 35.8, z: -2.3, topZ: 5.2, extrapolated: false, unstable: false, heightCm: 7.5,
    sizeCm: null, depthMeasured: true, supportMeasured: null, supportZMeasured: null, validFraction: null, supportBranch: null };
  for (const Context of [{ Servos: Api.INITIAL_SERVOS, ElbowMode: 'up' }, { Servos: Api.INITIAL_SERVOS, ElbowMode: 'down', Limits }]) {
    const Tag = Api.TaskObjectReachability(Eraser, Context);
    Assert.equal(Tag.Kind, 'out-of-reach');
    Assert.ok(Math.abs(Tag.BaseDistanceCm - 46.32) < 0.05, `${Tag.BaseDistanceCm} cm`);
    Assert.match(Tag.Text, /^OUT OF REACH \(46\.3 cm from the right arm's pan axis/);
  }
  // The same object moved within reach is not.
  const Near = Api.TaskObjectReachability({ ...Eraser, x: -5, y: 35, topZ: -0.3 }, { Servos: Api.INITIAL_SERVOS, ElbowMode: 'up' });
  Assert.equal(Near.Kind, 'graspable', Near.Text);
});
