import { build as Build } from "esbuild";
import { createRequire as CreateRequire } from "node:module";
import { test as Test } from "node:test";
import Assert from "node:assert/strict";
import { fileURLToPath as FilePath } from "node:url";
const Bundle = await Build({ stdin: { contents: `export * from './JointInteraction'; export * from './robotModel'; export * from './CadKinematics'; export { Matrix4, PerspectiveCamera, Vector3 } from 'three';`, resolveDir: FilePath(new URL('./src/lib/', import.meta.url)), loader: 'ts' }, bundle: true, platform: 'node', format: 'cjs', write: false });
const Module = { exports: {} }; new Function('require', 'module', 'exports', Bundle.outputFiles[0].text)(CreateRequire(import.meta.url), Module, Module.exports);
const Api = Module.exports;
Test('precision dial maps endpoints and intermediate positions without crossing its excluded arc', () => {
  for (const Fraction of [0, .01, .25, .5, .75, .99, 1]) {
    const [X, Y] = Api.DialPoint(Fraction);
    Assert.ok(Math.abs(Api.DialFraction(X - 100, Y - 100) - Fraction) < 1e-9);
  }
  Assert.equal(Api.DialFraction(0, 100), 1);
});
Test('all joint arcs follow actual calibrated axes, retain wrap-around ranges and remain finite', () => {
  const Angles = Api.currentAngles(Api.INITIAL_SERVOS), State = Api.CadForward(Angles);
  for (const Name of Api.SERVO_ORDER) {
    const Servo = Api.INITIAL_SERVOS[Name], Arc = Api.JointArc(Angles, Servo);
    const Matrix = State.Frames[Name].elements;
    Assert.ok(Math.hypot(...Arc.Center.toArray().map((Value, Index) => Value - Matrix[12 + Index])) < 1e-8);
    Assert.ok(Arc.Segments.length > 12 && Arc.Segments.every(Number.isFinite));
    for (let Index = 0; Index < Arc.Segments.length; Index += 3) {
      const Delta = Arc.Segments.slice(Index, Index + 3).map((Value, Axis) => Value - Arc.Center.getComponent(Axis));
      Assert.ok(Math.abs(Math.hypot(...Delta) - 7.5) < 1e-7);
      Assert.ok(Math.abs(Delta.reduce((Sum, Value, Axis) => Sum + Value * Matrix[8 + Axis], 0)) < 1e-7);
    }
    Assert.ok(Arc.Fraction >= 0 && Arc.Fraction <= 1);
  }
});

const Step = 2 * Math.PI / 4096;
const InArc = (Servo, Raw) => Api.rawToFraction(Servo, Raw) <= 1 + 1e-12;
Test('turning a joint keeps the raw inside its configured arc, like its slider', () => {
  for (const Name of Api.SERVO_ORDER) {
    const Servo = Api.INITIAL_SERVOS[Name];
    const Low = Api.fractionToRaw(Servo, 0), High = Api.fractionToRaw(Servo, 1);
    const Start = Api.fractionToRaw(Servo, .5);
    const Still = Api.TurnJoint(Servo, Start, 0);
    Assert.ok(Still.Raw === Start && Math.abs(Still.Turn) < 1e-12, `${Name}: no turn, no change`);
    for (const Turn of [.05, -.05, .2, -.2]) {
      const Result = Api.TurnJoint(Servo, Start, Turn);
      Assert.ok(InArc(Servo, Result.Raw), `${Name}: ${Turn} stays in the arc`);
      const Turned = Api.JointAxisAngle(Servo, Result.Raw) - Api.JointAxisAngle(Servo, Start);
      if (Result.Raw !== Low && Result.Raw !== High) Assert.ok(Math.abs(Turned - Turn) <= Step, `${Name}: turns ${Turn} rad, got ${Turned}`);
    }
    for (const Turn of [100, -100]) {
      const Result = Api.TurnJoint(Servo, Start, Turn);
      Assert.ok(Result.Raw === Low || Result.Raw === High, `${Name}: a huge turn stops at a limit`);
      Assert.ok(Math.abs(Result.Turn) < Math.abs(Turn) && Math.abs(Result.Turn) <= 2 * Math.PI, `${Name}: the returned turn is clamped`);
      Assert.ok(Math.abs(Api.JointAxisAngle(Servo, Result.Raw) - Api.JointAxisAngle(Servo, Start) - Result.Turn) <= Step);
      const Back = Api.TurnJoint(Servo, Start, Result.Turn - Math.sign(Turn) * .1);
      Assert.notEqual(Back.Raw, Result.Raw, `${Name}: turning back from a limit moves at once`);
    }
    // A range with no span is never turned, so a drag on it commands nothing.
    const Flat = { ...Servo, max: Servo.min };
    Assert.deepEqual(Api.TurnJoint(Flat, Servo.targetRaw, .5), { Raw: Servo.targetRaw, Turn: 0 }, `${Name}: a flat range is left alone`);
    // A target outside the arc is pulled to the nearest end, never driven further out.
    const Outside = Api.clampRaw(Servo.arcInverted ? Low + 40 : Low - 40);
    if (!InArc(Servo, Outside)) Assert.equal(Api.TurnJoint(Servo, Outside, 0).Raw, Low, `${Name}: outside the arc snaps to the near end`);
  }
});
const Dial = (Turn = 0) => {
  const Frame = new Api.Matrix4().makeRotationX(.35).multiply(new Api.Matrix4().makeRotationY(-.6)).setPosition(10, 20, 30);
  return { Frame, Radius: 7.5, Angle: .4 + Turn };
};
const Width = 1600, Height = 1000;
const Look = Offset => {
  const Camera = new Api.PerspectiveCamera(35, Width / Height, .1, 3000); Camera.up.set(0, 0, 1);
  const Center = new Api.Vector3(10, 20, 30); Camera.position.copy(Center).add(Offset); Camera.lookAt(Center);
  Camera.updateMatrixWorld(); Camera.updateProjectionMatrix(); return Camera;
};
const Screen = (Camera, Point) => { const P = Point.clone().project(Camera); return [(P.x * .5 + .5) * Width, (-P.y * .5 + .5) * Height]; };
/** Walks the pointer along the knob's own path and adds up the turns, as the canvas does per pointer move. */
const Follow = (Camera, From, To, Steps = 15) => {
  let Turn = 0, Last = Screen(Camera, Api.JointDialPoint(Dial(From)));
  for (let Index = 1; Index <= Steps; Index++) {
    const Next = Screen(Camera, Api.JointDialPoint(Dial(From + (To - From) * Index / Steps)));
    Turn += Api.DialTurn(Dial(Turn + From), Camera, Width, Height, Last, Next); Last = Next;
  }
  return Turn;
};
Test('dragging a dial that faces the camera keeps the knob under the cursor', () => {
  const Camera = Look(new Api.Vector3(40, -55, 45));
  const Normal = new Api.Vector3().setFromMatrixColumn(Dial().Frame, 2);
  Assert.ok(Math.abs(Normal.dot(Camera.getWorldDirection(new Api.Vector3()))) > .3, 'fixture faces the camera');
  for (const Turn of [.6, -.6, 2.5, -2.5]) Assert.ok(Math.abs(Follow(Camera, 0, Turn, 40) - Turn) < 1e-6, `turn ${Turn}`);
  // One move straight across the dial: the angle the pointer swept, not the pixels it moved.
  const Single = Api.DialTurn(Dial(), Camera, Width, Height, Screen(Camera, Api.JointDialPoint(Dial())), Screen(Camera, Api.JointDialPoint(Dial(1.2))));
  Assert.ok(Math.abs(Single - 1.2) < 1e-6);
});
Test('a dial seen edge on, or grabbed at its hub, turns the way the pointer moves', () => {
  const Normal = new Api.Vector3().setFromMatrixColumn(Dial().Frame, 2).normalize();
  const Side = new Api.Vector3(1, 0, 0).addScaledVector(Normal, -Normal.x).normalize().multiplyScalar(90);
  const Camera = Look(Side);
  Assert.ok(Math.abs(Normal.dot(Camera.getWorldDirection(new Api.Vector3()))) < .05, 'fixture is edge on');
  for (const Turn of [.3, -.3]) {
    const Got = Follow(Camera, 0, Turn, 30);
    Assert.ok(Math.sign(Got) === Math.sign(Turn) && Math.abs(Got - Turn) < .15 * Math.abs(Turn), `edge on ${Turn}: ${Got}`);
  }
  const Facing = Look(new Api.Vector3(40, -55, 45)), Hub = Screen(Facing, new Api.Vector3(10, 20, 30));
  for (const Turn of [.5, -.5]) {
    const Got = Api.DialTurn(Dial(), Facing, Width, Height, Hub, Screen(Facing, Api.JointDialPoint(Dial(Turn))));
    Assert.ok(Math.sign(Got) === Math.sign(Turn) && Number.isFinite(Got), `from the hub toward ${Turn}: ${Got}`);
  }
  Assert.equal(Api.DialTurn(Dial(), Facing, Width, Height, Hub, Hub), 0, 'no move, no turn');
});
