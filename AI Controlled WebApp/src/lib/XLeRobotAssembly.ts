import { Box3, PerspectiveCamera, Vector3 } from "three";
import SceneSource from "./XLeRobotScene.json";
import type { CadPoint } from "./CadKinematics";

export const XLeRobotScene = SceneSource;
export type ArmSide = "Left" | "Right";
export const BinTopZCm = XLeRobotScene.Parts.find(Part => Part.Name === "XLebase_link0")!.BoundsCm[1][2];
export const OriginAssemblyCm = new Vector3(
  (XLeRobotScene.Mounts.Left.PanCm[0] + XLeRobotScene.Mounts.Right.PanCm[0]) / 2,
  (XLeRobotScene.Mounts.Left.PanCm[1] + XLeRobotScene.Mounts.Right.PanCm[1]) / 2, BinTopZCm);

// Keep the controlled arm and all target coordinates in their existing frame.
// Translate the surrounding assembly to the chosen arm's pan axis instead.
export function AssemblyPlacement(SceneBase: CadPoint, LiveSide: ArmSide) {
  const OtherSide = LiveSide === "Right" ? "Left" : "Right";
  const BodyOffset = new Vector3(SceneBase.x, SceneBase.y, SceneBase.z)
    .sub(new Vector3(...XLeRobotScene.Mounts[LiveSide].PanCm as [number, number, number]));
  const OtherPan = new Vector3(...XLeRobotScene.Mounts[OtherSide].PanCm as [number, number, number]).add(BodyOffset);
  const Origin = OriginAssemblyCm.clone().add(BodyOffset);
  const OriginLine = ["Left", "Right"].map(Side => {
    const Pan = XLeRobotScene.Mounts[Side as ArmSide].PanCm;
    return new Vector3(Pan[0], Pan[1], BinTopZCm).add(BodyOffset);
  });
  return { BodyOffset, OtherPan, Origin, OriginLine, FloorZ: BodyOffset.z + XLeRobotScene.FloorZCm };
}

export function BodyBounds(Offset: Vector3) {
  return new Box3(new Vector3(...XLeRobotScene.BoundsCm[0] as [number, number, number]),
    new Vector3(...XLeRobotScene.BoundsCm[1] as [number, number, number])).translate(Offset);
}

export function BoxCorners(Bounds: Box3) {
  return Array.from({ length: 8 }, (_, Index) => new Vector3(
    Index & 1 ? Bounds.max.x : Bounds.min.x, Index & 2 ? Bounds.max.y : Bounds.min.y,
    Index & 4 ? Bounds.max.z : Bounds.min.z));
}

// Fit every corner, including depth, to a padded projection. A diagonal/sphere
// heuristic wastes most of the narrow desktop panel on a tall robot.
export function FitAssemblyCamera(Camera: PerspectiveCamera, Bounds: Box3, View: string) {
  const Directions: Record<string, Vector3> = {
    iso: new Vector3(1.25, 1.8, 0.8), top: new Vector3(0, -0.0001, 1), bottom: new Vector3(0, 0.0001, -1),
    front: new Vector3(0, 1, 0), back: new Vector3(0, -1, 0), right: new Vector3(1, 0, 0), left: new Vector3(-1, 0, 0),
  };
  const Direction = (Directions[View] ?? Directions.iso).clone().normalize();
  const Right = new Vector3().crossVectors(Camera.up, Direction).normalize();
  const Up = new Vector3().crossVectors(Direction, Right).normalize();
  const Center = Bounds.getCenter(new Vector3());
  const Vertical = Math.tan(Camera.fov * Math.PI / 360) * 0.69;
  const Horizontal = Math.tan(Camera.fov * Math.PI / 360) * Camera.aspect * 0.82;
  let Distance = 30;
  for (const Corner of BoxCorners(Bounds)) {
    const Relative = Corner.sub(Center), Depth = Relative.dot(Direction);
    Distance = Math.max(Distance, Depth + Math.abs(Relative.dot(Right)) / Horizontal,
      Depth + Math.abs(Relative.dot(Up)) / Vertical);
  }
  Camera.position.copy(Center).addScaledVector(Direction, Distance);
  Camera.lookAt(Center); Camera.updateMatrixWorld(true); Camera.updateProjectionMatrix();
  return Center;
}
