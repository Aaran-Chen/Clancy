import { Camera, Matrix4, Vector3 } from "three";
import { CadForward, CadModel, CadPitch, CadUrdfAngles, type CadAngles } from "./CadKinematics";

export const WristWheelStepDegrees = 5;
export function WristWheelDelta(Event: Pick<WheelEvent, "deltaX" | "deltaY" | "deltaMode">, ViewHeight: number) {
  // Windows may translate Shift+wheel into horizontal scrolling. Choose one
  // axis rather than counting a diagonal trackpad gesture twice.
  const Delta = Math.abs(Event.deltaY) >= Math.abs(Event.deltaX) ? Event.deltaY : Event.deltaX;
  const Pixels = Delta * (Event.deltaMode === 1 ? 16 : Event.deltaMode === 2 ? ViewHeight : 1);
  return Number.isFinite(Pixels) ? Pixels * WristWheelStepDegrees / 100 : 0;
}

// Use the selected IK pose's tool axis. A radial line from the shoulder can
// point the wrong way when the elbow folds back or the tool has a lateral offset.
export function TargetApproachDirection(Angles: CadAngles, RequestedPitch: number) {
  const Frame = CadForward(Angles).Links[CadModel.ToolLink];
  const Direction = new Vector3().setFromMatrixColumn(Frame, 2).normalize();
  const Pan = CadUrdfAngles(Angles).shoulder_pan;
  return Direction.applyAxisAngle(new Vector3(Math.cos(Pan), -Math.sin(Pan), 0),
    (RequestedPitch - CadPitch(Angles)) * Math.PI / 180).normalize();
}

// CSS uses screen-down Y. At the Front view (+Y), screen-right is robot -X.
// These bases keep the labels upright while the cube follows camera rotation.
export function ViewCubeRotation(CameraValue: Camera) {
  const ScreenBasis = new Matrix4().makeScale(1, -1, 1);
  const CubeBasis = new Matrix4().makeBasis(new Vector3(-1, 0, 0), new Vector3(0, 0, -1), new Vector3(0, 1, 0));
  const InverseCamera = new Matrix4().makeRotationFromQuaternion(CameraValue.quaternion.clone().invert());
  return ScreenBasis.multiply(InverseCamera).multiply(CubeBasis);
}
