import { Matrix4, Vector3 } from "three";
import { CadJointNames } from "./CadKinematics";

// Same Taylor enclosure as ArmMotionEnvelope.py. The radius of the later
// joint bounds each mixed second derivative; the remainder is not optional.
export function BoxEnvelope(Corners: Vector3[], Radii: number[], Transform: Matrix4,
  Frames: Record<string, Matrix4>, Motion: number[]) {
  const Axes = [new Vector3(1, 0, 0), new Vector3(0, 1, 0), new Vector3(0, 0, 1)]
    .map(Axis => Axis.transformDirection(Transform));
  const JointAxes = CadJointNames.map(Name => new Vector3(0, 0, 1).transformDirection(Frames[Name]));
  const Origins = CadJointNames.map(Name => new Vector3().setFromMatrixPosition(Frames[Name]));
  let Remainder = 1e-9;
  for (let First = 0; First < Radii.length; First++) {
    for (let Second = 0; Second < Radii.length; Second++) {
      if (Radii[First] && Radii[Second]) {
        Remainder += 0.5 * Motion[First] * Motion[Second] * Radii[Math.max(First, Second)];
      }
    }
  }
  const Low = new Vector3(Infinity, Infinity, Infinity), High = Low.clone().negate();
  for (const Corner of Corners) {
    const Point = Corner.clone().applyMatrix4(Transform);
    const Error = new Vector3();
    for (let Joint = 0; Joint < Radii.length; Joint++) {
      if (!Radii[Joint] || !Motion[Joint]) continue;
      const Derivative = JointAxes[Joint].clone().cross(Point.clone().sub(Origins[Joint]));
      for (let Axis = 0; Axis < 3; Axis++) {
        Error.setComponent(Axis, Error.getComponent(Axis) + Math.abs(Derivative.dot(Axes[Axis])) * Motion[Joint]);
      }
    }
    Low.min(Corner.clone().sub(Error).addScalar(-Remainder));
    High.max(Corner.clone().add(Error).addScalar(Remainder));
  }
  return { Center: Low.clone().add(High).multiplyScalar(0.5).applyMatrix4(Transform),
    Axes, Half: High.sub(Low).multiplyScalar(0.5) };
}
