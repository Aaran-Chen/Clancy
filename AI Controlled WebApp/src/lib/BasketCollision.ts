import { Matrix4, Vector3 } from "three";
import Model from "../../../back end/CollisionModel.json";
import BodyData from "../../../back end/RobotBodySurface.json";
import Frame from "../../../back end/WorkspaceFrame.json";
import { CadForward, CadForwardUrdf, CadModel, CadJointNames, type CadAngles, type CadPoint } from "./CadKinematics";
import { BasketSurface } from "./BasketSurface";
import { BoxEnvelope } from "./ArmMotionEnvelope";

export const BasketCollisionVersion = Model.Version;
const Mount = Model.Mounts[Frame.RobotFrame.ControlledArm as "Right" | "Left"].PanCm;
export const BasketBounds = Model.BasketAssemblyBoundsCm.map(Point => new Vector3(...Point as [number, number, number])
  .sub(new Vector3(...Mount as [number, number, number])));
// The top tray alone (its open top ends at the front rim, display Y 23.3), in
// the same frame. The whole cart reaches Y 29.06, over the desk in front of it.
export const TrayBounds = Model.BasketTrayBoundsCm.map(Point => new Vector3(...Point as [number, number, number])
  .sub(new Vector3(...Mount as [number, number, number])));
const Surface = new BasketSurface(Mount);
const BodySurface = new BasketSurface(Mount, BodyData);
const WorldAxes = [new Vector3(1, 0, 0), new Vector3(0, 1, 0), new Vector3(0, 0, 1)];
const Corners = (Low: number[], High: number[]) => Array.from({ length: 8 }, (_, Index) =>
  new Vector3(...Low.map((Value, Axis) => Index & (1 << Axis) ? High[Axis] : Value) as [number, number, number]));
// The tracking allowance: the rearward limit keeps it (Padding). The basket, cart and body contact checks
// add their own, ContactTrackingAllowanceRaw: 0 since v8 (CAD-exact, 2026-10-04: a pose is refused only
// where a link's CAD box meets the surface; 08:32 the arm stood at the grasp point clear of the basket and
// the bridge refused to close the jaws there, the box 2.4 cm from the cart and only its 16-step envelope
// touching it), matching BasketCollision.py.
const TrackingAllowanceRadians = Model.JointTrackingAllowanceRaw * 2 * Math.PI / 4096;
const ContactAllowanceRadians = Model.ContactTrackingAllowanceRaw * 2 * Math.PI / 4096;
const ContactMotion = CadJointNames.map(() => ContactAllowanceRadians);
// Radius per ancestor joint, matching BasketCollision.py MotionRadii.
function MotionRadii(Link: string, Radius: number) {
  const Radii: number[] = CadJointNames.map(() => 0);
  while (Link !== CadModel.Root) {
    const Joint = CadModel.Joints.find(Value => Value.Child === Link)!;
    const Index = (CadJointNames as readonly string[]).indexOf(Joint.Name);
    if (Index >= 0) Radii[Index] = Radius;
    Radius += Math.hypot(...Joint.Origin.slice(0, 3).map(Row => Row[3]));
    Link = Joint.Parent;
  }
  return Radii;
}
const Boxes = Model.ArmBoxes.map(Box => {
  const Low = new Vector3(...Box.BoundsCm[0] as [number, number, number]);
  const High = new Vector3(...Box.BoundsCm[1] as [number, number, number]);
  const Points = Corners(Box.BoundsCm[0], Box.BoundsCm[1]);
  const Radii = MotionRadii(Box.Link, Math.max(...Points.map(Point => Point.length())));
  const Total = Radii.reduce((Sum, Radius) => Sum + Radius, 0);
  return { ...Box, Center: Low.clone().add(High).multiplyScalar(0.5), Half: High.sub(Low).multiplyScalar(0.5),
    Padding: Model.ClearanceCm + Total * TrackingAllowanceRadians, ContactPadding: Model.ClearanceCm + Total * ContactAllowanceRadians, Points, Radii };
});
// Shared display-coordinate boundary, independent of the active arm mount.
export const RearwardReason = `Arm cannot move behind Y = ${Model.RearwardLimit.MinimumDisplayYCm} cm`;
const MinimumY = Model.RearwardLimit.MinimumDisplayYCm + Frame.RobotFrame.OriginAssemblyCm[1] - Mount[1];
export function RearwardExcess(Links: Record<string, Matrix4>) {
  return Math.max(...Boxes.filter(Box => Box.Link !== "shoulder_link").map(Box => {
    const Transform = Links[Box.Link], Elements = Transform.elements;
    const Minimum = Box.Center.clone().applyMatrix4(Transform).y
      - Math.abs(Elements[1]) * Box.Half.x - Math.abs(Elements[5]) * Box.Half.y - Math.abs(Elements[9]) * Box.Half.z;
    return MinimumY - Minimum + Box.Padding - Model.ClearanceCm;
  }));
}

function ObstacleReason(Center: Vector3, Axes: Vector3[], Half: Vector3, Padding: number): string | null {
  if (Surface.Overlaps(Center, Axes, Half, Padding)) return "basket";
  if (BodySurface.Overlaps(Center, Axes, Half, Padding)) return "robot body or camera head";
  return null;
}

export function BasketTargetReason(Point: CadPoint): string | null {
  const Values = [Point.x, Point.y, Point.z];
  if (!Values.every(Number.isFinite)) return "Target coordinates must be finite";
  const Reason = ObstacleReason(new Vector3(Point.x, Point.y, Point.z), WorldAxes, new Vector3(), Model.ClearanceCm);
  return Reason ? `Target touches or enters ${Reason} material` : Point.y < MinimumY ? RearwardReason : null;
}

// The pan collar has only axial rotation, so its full swept volume is static.
// Its complete sweep must clear the actual basket surfaces.
const Pan = CadForwardUrdf({}).Frames.shoulder_pan;
const PanOrigin = new Vector3().setFromMatrixPosition(Pan);
const MountReason = Boxes.filter(Box => Box.Link === "shoulder_link").map(Box => {
  const Low = new Vector3(Infinity, Infinity, Infinity), High = new Vector3(-Infinity, -Infinity, -Infinity);
  for (const Point of Corners(Box.BoundsCm[0], Box.BoundsCm[1])) {
    const Radius = Math.hypot(Point.x, Point.y);
    for (let Axis = 0; Axis < 3; Axis++) {
      const Middle = PanOrigin.getComponent(Axis) + Pan.elements[8 + Axis] * Point.z;
      const Extent = Radius * Math.hypot(Pan.elements[Axis], Pan.elements[4 + Axis]);
      Low.setComponent(Axis, Math.min(Low.getComponent(Axis), Middle - Extent));
      High.setComponent(Axis, Math.max(High.getComponent(Axis), Middle + Extent));
    }
  }
  return ObstacleReason(Low.clone().add(High).multiplyScalar(0.5), WorldAxes, High.sub(Low).multiplyScalar(0.5), Model.MountClearanceCm);
}).find(Boolean) ?? null;

// Basket, mast and camera-head contact only, independent of the rearward limit:
// a link's CAD box meeting the surface (plus ContactTrackingAllowanceRaw, 0).
// ExtraCm: clearance beyond the model's, for a search that settles on the
// contact boundary (the AI target snap): the bridge checks the pose after
// rounding it to encoder steps, which crossed boundaries the exact pose touched.
export function BasketSurfaceReason(Angles: CadAngles, State = CadForward(Angles), ExtraCm = 0): string | null {
  if (MountReason) return `Shoulder mounting collar intersects the ${MountReason}`;
  for (const Box of Boxes) {
    if (Box.Link === "shoulder_link") continue;
    const Transform: Matrix4 = State.Links[Box.Link];
    const Axes = WorldAxes.map(Axis => Axis.clone().transformDirection(Transform));
    const Center = Box.Center.clone().applyMatrix4(Transform);
    let Envelope: ReturnType<typeof BoxEnvelope> | undefined;
    for (const [Obstacle, Reason] of [[Surface, "basket"], [BodySurface, "robot body or camera head"]] as const) {
      if (!Obstacle.Overlaps(Center, Axes, Box.Half, Box.ContactPadding + ExtraCm)) continue;
      if (!ContactAllowanceRadians) return `${Box.Link.replace(/_/g, " ")} could contact the ${Reason}`;
      Envelope ??= BoxEnvelope(Box.Points, Box.Radii, Transform, State.Frames, ContactMotion);
      if (Obstacle.Overlaps(Envelope.Center, Envelope.Axes, Envelope.Half, Model.ClearanceCm + ExtraCm)) {
        return `${Box.Link.replace(/_/g, " ")} could contact the ${Reason}`;
      }
    }
  }
  return null;
}

export function BasketPoseReason(Angles: CadAngles, ExtraCm = 0): string | null {
  const State = CadForward(Angles);
  // Same order as the bridge, so both report the same reason: contact, then rearward limit.
  return BasketSurfaceReason(Angles, State, ExtraCm) ?? (RearwardExcess(State.Links) > -ExtraCm ? RearwardReason : null);
}
