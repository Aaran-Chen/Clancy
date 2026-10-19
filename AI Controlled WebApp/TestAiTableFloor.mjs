import { build as Build } from 'esbuild';
import { createRequire as CreateRequire } from 'node:module';
import { test as Test } from 'node:test';
import Assert from 'node:assert/strict';
import Path from 'node:path';
import { fileURLToPath as FilePath } from 'node:url';

const Root = Path.dirname(FilePath(import.meta.url));
const Bundle = await Build({ stdin: { contents: `export * from './AiTableFloor'; export * from './robotModel';
export * from './BasketCollision'; export { CadForward } from './CadKinematics';`, resolveDir: Path.join(Root, 'src/lib'), loader: 'ts' },
bundle: true, platform: 'node', format: 'cjs', write: false });
const Module = { exports: {} };
new Function('require', 'module', 'exports', Bundle.outputFiles[0].text)(CreateRequire(import.meta.url), Module, Module.exports);
const Api = Module.exports;
// AI targets are model-frame points; the rule is written in the AI's display frame.
const Model = Display => Api.modelPointFromDisplayPoint(Display);
// The middle of the top tray's open footprint (model frame).
const TrayCentre = () => [0, 1].map(Axis => (Api.TrayBounds[0].getComponent(Axis) + Api.TrayBounds[1].getComponent(Axis)) / 2);
const Beyond = { x: -15, y: Api.mirrorDisplayPoint({ x: 0, y: Api.BasketBounds[1].y + 15, z: 0 }).y };

Test('the floor sits 1 cm under the measured table surface', () => {
  Assert.ok(Math.abs(Api.TableZCm - -2.3) < 0.01);
  Assert.ok(Math.abs(Api.AiTableFloorCm - (Api.TableZCm - 1)) < 1e-9);
});

Test('a descent below the floor over the table is refused, a rise from there is not', () => {
  const Below = Model({ ...Beyond, z: Api.AiTableFloorCm - 2 });
  Assert.equal(Api.OverBasket(Below), false);
  Assert.match(Api.AiTableFloorReason(Below, -3), /below the table floor/);
  Assert.equal(Api.AiTableFloorReason(Below, 1), null);
  Assert.equal(Api.AiTableFloorReason(Model({ ...Beyond, z: Api.TableZCm + 2 }), -3), null);
  Assert.equal(Api.AiTableFloorReason(Model({ ...Beyond, z: Api.AiTableFloorCm + 0.01 }), -3), null);
});

Test('the old -8 cm floor no longer passes over the table', () => {
  Assert.match(Api.AiTableFloorReason(Model({ ...Beyond, z: -7 }), -12), /below the table floor/);
});

Test('the tray bounds share the frame of AI targets: the pan joint on the rim is over the tray', () => {
  // Independent of the display conversion: CAD forward kinematics places the
  // pan joint, which is bolted to the basket rim near its rear corner.
  const Pan = Api.CadForward({}).Joints[0];
  Assert.equal(Api.OverBasket(Pan), true);
  const [Low, High] = Api.TrayBounds;
  Assert.ok(Pan.z > Low.z && Pan.z < High.z + 5, `pan z ${Pan.z} vs tray ${Low.z}..${High.z}`);
});

Test('over the basket the tip may go lower; the collision model guards it', () => {
  const Centre = Api.mirrorDisplayPoint({ x: TrayCentre()[0], y: TrayCentre()[1], z: 0 });
  const Inside = Model({ x: Centre.x, y: Centre.y, z: Api.TableZCm - 8 });
  Assert.equal(Api.OverBasket(Inside), true);
  Assert.equal(Api.AiTableFloorReason(Inside, -5), null);
});

Test('in front of the tray the floor holds: the whole cart reaches over the desk, the tray\'s open top ends at its front rim', () => {
  // The cart's bounds reach display Y 29.06; the tray's front rim is at Y 23.3
  // (where the depth camera sees it). Low targets on the desk in between were
  // exempt from the floor, and once the rim moved nothing else refused them.
  Assert.ok(Api.mirrorDisplayPoint({ x: 0, y: Api.BasketBounds[1].y, z: 0 }).y > 29);
  let Clear = 0;
  for (let Y = 23.5; Y <= 29.01; Y += 0.5) for (let X = -22; X <= 22.01; X += 1) {
    const Point = Model({ x: X, y: Y, z: -5 });
    if (Api.BasketTargetReason(Point)) continue;
    Clear += 1;
    Assert.equal(Api.OverBasket(Point), false, `X ${X}, Y ${Y} is over the tray`);
    Assert.match(Api.AiTableFloorReason(Point, -3) ?? 'accepted', /below the table floor/, `X ${X}, Y ${Y}, Z -5`);
  }
  Assert.ok(Clear > 300, `${Clear} collision-clear desk points`);
  // The jaw rule too, which the same exemption skipped.
  const Target = { ...Model({ x: -16, y: 28, z: -5 }), wristPitch: -90 };
  const Preview = Api.solveIkPreview(Target, 'up', Api.INITIAL_SERVOS);
  Assert.ok(Preview.reachable);
  Assert.match(Api.AiJawFloorReason(Target, Preview.angles, Api.currentAngles(Api.INITIAL_SERVOS)) ?? 'accepted', /below the table floor/);
  // Just behind the rim, over the tray's open top, low targets stay exempt.
  const Behind = Model({ x: -15, y: 22.9, z: -5 });
  Assert.equal(Api.OverBasket(Behind), true);
  Assert.equal(Api.AiTableFloorReason(Behind, -3), null);
});

Test('a target exactly on the floor passes, whatever float noise the frame round trip adds', () => {
  for (const [X, Y] of [[4.2, 32.5], [4.1, 32.3], [0, 30], [-5, 35], [15, 35], [-20, 40], [10, 45], [-8, 34]]) {
    const Point = Model({ x: X, y: Y, z: Api.AiTableFloorCm });
    if (Api.OverBasket(Point)) continue;
    Assert.equal(Api.AiTableFloorReason(Point, -1), null, `X ${X}, Y ${Y}`);
  }
});

Test('the floor holds the lowest point of the gripper, which hangs under the tip', () => {
  const Home = Api.currentAngles(Api.INITIAL_SERVOS);
  for (const Pitch of [-45, -30, -20]) {
    const Target = { ...Model({ x: -34, y: 32, z: -1.5 }), wristPitch: Pitch };
    const Preview = Api.solveIkPreview(Target, 'up', Api.INITIAL_SERVOS);
    Assert.ok(Preview.reachable, `wrist ${Pitch}`);
    const Drop = -1.5 - Api.GripperLowestZ(Preview.angles);
    Assert.ok(Drop > 0.3 && Drop < 1.6, `wrist ${Pitch}: the gripper reaches ${Drop.toFixed(2)} cm under the tip`);
    Assert.equal(Api.AiJawFloorReason(Target, Preview.angles, Home), null, `wrist ${Pitch}: a grasp at mid-height of a desk object`);
  }
  // The tip 0.3 cm above the floor with a flat wrist: the jaw goes under it.
  const Low = { ...Model({ x: -34, y: 32, z: Api.AiTableFloorCm + 0.3 }), wristPitch: -20 };
  const Angles = Api.solveIkPreview(Low, 'up', Api.INITIAL_SERVOS).angles;
  Assert.ok(Api.GripperLowestZ(Angles) < Api.AiTableFloorCm);
  Assert.match(Api.AiJawFloorReason(Low, Angles, Home), /lowest point of the gripper .*below the table floor/);
  Assert.equal(Api.AiJawFloorReason(Low, Angles, Angles), null, 'Not lower than where it already is: rising out stays allowed');
  // Over the basket the collision model guards the gripper instead.
  const Centre = TrayCentre();
  Assert.equal(Api.AiJawFloorReason({ x: Centre[0], y: Centre[1], z: Low.z, wristPitch: -20 }, Angles, Home), null);
});
