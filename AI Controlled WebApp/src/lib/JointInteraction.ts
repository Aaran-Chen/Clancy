import { Plane, Raycaster, Vector2, Vector3, type Camera, type Matrix4 } from "three";
import { CadForward, CadUrdfAngles, type CadAngles } from "./CadKinematics";
import { angleFromRawValue, arcSpan, currentRawForAngle, fractionToRaw, rawToFraction, type ServoState } from "./robotModel";

export function DialPoint(Fraction: number, Radius = 74) {
  const Angle = (-225 + Math.max(0, Math.min(1, Fraction)) * 270) * Math.PI / 180;
  return [100 + Math.cos(Angle) * Radius, 100 + Math.sin(Angle) * Radius];
}
export function DialFraction(X: number, Y: number) {
  const Degrees = Math.atan2(Y, X) * 180 / Math.PI;
  const Angle = (Degrees + 225 + 360) % 360;
  return Angle <= 270 ? Angle / 270 : Angle > 315 ? 0 : 1;
}
export const JointDialRadiusCm = 7.5;
export function JointArc(Angles: CadAngles, Servo: ServoState, Radius = JointDialRadiusCm) {
  const Frame = CadForward(Angles).Frames[Servo.name];
  const Center = new Vector3().setFromMatrixPosition(Frame);
  const Point = (Raw: number) => {
    const Angle = CadUrdfAngles({ [Servo.name]: angleFromRawValue(Servo, Raw) })[Servo.name];
    return new Vector3(Math.cos(Angle) * Radius, Math.sin(Angle) * Radius, 0).applyMatrix4(Frame);
  };
  const Count = Math.max(2, Math.ceil(arcSpan(Servo) / 4096 * 180));
  const Points = Array.from({ length: Count + 1 }, (_, Index) => Point(fractionToRaw(Servo, Index / Count)));
  const Segments = Points.slice(1).flatMap((Value, Index) => [...Points[Index].toArray(), ...Value.toArray()]);
  return { Center, Segments, Current: Point(currentRawForAngle(Servo)), Target: Point(Servo.targetRaw),
    Fraction: Math.max(0, Math.min(1, rawToFraction(Servo, Servo.targetRaw))) };
}

/** A joint's URDF angle at a raw position: radians about the joint's own z axis
    (the axis its arc is drawn round), linear in raw across the configured arc. */
export function JointAxisAngle(Servo: ServoState, Raw: number) {
  return CadUrdfAngles({ [Servo.name]: angleFromRawValue(Servo, Raw) })[Servo.name];
}

/** Turning a joint Turn radians from StartRaw. The raw is held inside the
    configured arc exactly as the joint's slider holds it, and the turn comes
    back clamped to what that raw really turns, so dragging back from past a
    limit moves the joint again at once instead of first unwinding the excess.
    A range with no span cannot be turned: the start raw comes back unchanged. */
export function TurnJoint(Servo: ServoState, StartRaw: number, Turn: number) {
  const Start = JointAxisAngle(Servo, StartRaw), Low = JointAxisAngle(Servo, fractionToRaw(Servo, 0));
  const Span = JointAxisAngle(Servo, fractionToRaw(Servo, 1)) - Low;
  if (Math.abs(Span) < 1e-9) return { Raw: StartRaw, Turn: 0 };
  const Fraction = Math.max(0, Math.min(1, (Start + (Number.isFinite(Turn) ? Turn : 0) - Low) / Span));
  return { Raw: fractionToRaw(Servo, Fraction), Turn: Low + Fraction * Span - Start };
}

/** A joint's dial in the scene: the joint frame (world, before its own
    rotation), the arc radius and the angle the knob is at. */
export type JointDial = { Frame: Matrix4; Radius: number; Angle: number };
export function JointDialPoint(Dial: JointDial, Angle = Dial.Angle) {
  return new Vector3(Math.cos(Angle) * Dial.Radius, Math.sin(Angle) * Dial.Radius, 0).applyMatrix4(Dial.Frame);
}

/** How far a pointer move turns a joint dial, in radians about the joint axis.
    From and To are pixels from the canvas's top left. Where the dial faces the
    camera, the pointer's own angle round the axis is used, so the knob stays
    under the cursor; near the hub, where that angle is undefined, the move
    along the knob's tangent in the dial's plane turns it, like a lever. Seen
    nearly edge on, the move along the knob's screen tangent turns it. */
export function DialTurn(Dial: JointDial, Camera: Camera, Width: number, Height: number, From: readonly [number, number], To: readonly [number, number]) {
  const Center = new Vector3().setFromMatrixPosition(Dial.Frame);
  const Normal = new Vector3().setFromMatrixColumn(Dial.Frame, 2).normalize();
  const DialPlane = new Plane().setFromNormalAndCoplanarPoint(Normal, Center);
  const Ray = new Raycaster();
  const Hit = ([X, Y]: readonly [number, number]) => {
    Ray.setFromCamera(new Vector2(X / Width * 2 - 1, -Y / Height * 2 + 1), Camera);
    if (Math.abs(Ray.ray.direction.dot(Normal)) < .25) return null;
    return Ray.ray.intersectPlane(DialPlane, new Vector3())?.sub(Center) ?? null;
  };
  const Start = Hit(From), End = Hit(To), Step = .02;
  const Ahead = JointDialPoint(Dial, Dial.Angle + Step), Knob = JointDialPoint(Dial);
  if (Start && End) {
    if (Math.min(Start.length(), End.length()) >= .3 * Dial.Radius) return Math.atan2(Normal.dot(Start.clone().cross(End)), Start.dot(End));
    return End.sub(Start).dot(Ahead.sub(Knob).normalize()) / Dial.Radius;
  }
  const Screen = (Point: Vector3) => { const Projected = Point.clone().project(Camera);
    return new Vector2((Projected.x * .5 + .5) * Width, (-Projected.y * .5 + .5) * Height); };
  const Tangent = Screen(Ahead).sub(Screen(Knob)).divideScalar(Step);
  // A lever turns no faster than the dial's radius seen face on would let it,
  // so a knob pointing at the camera cannot spin the joint on a tiny move.
  const Right = new Vector3().setFromMatrixColumn(Camera.matrixWorld, 0).normalize();
  const RadiusPixels = Screen(Center.clone().addScaledVector(Right, Dial.Radius)).distanceTo(Screen(Center));
  const Move = new Vector2(To[0] - From[0], To[1] - From[1]);
  return Move.dot(Tangent) / Math.max(Tangent.lengthSq(), (.3 * RadiusPixels) ** 2, 1e-9);
}
