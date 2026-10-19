import Frame from "../../../back end/WorkspaceFrame.json";
import Hull from "./GripperHull.json";
import { TrayBounds } from "./BasketCollision";
import type { Matrix4 } from "three";
import { CadForward, type CadAngles } from "./CadKinematics";
import { SurfaceContains, type DeclaredSurface } from "./DeclaredSurfaces";
import type { MeasuredScene } from "./SceneGeometry";
import { mirrorDisplayPoint, type Point3 } from "./robotModel";

// The desk surface in the AI's display frame, measured with the gripper tip.
export const TableZCm = Frame.TableZCm;
// The collision model covers the cart, basket, mast and head but not the desk,
// so AI targets are held above this floor, 1 cm under the desk surface.
export const AiTableFloorCm = TableZCm - 1.0;
// A target exactly on the floor, sent as display Z -3.3, came back from the
// model frame as -3.3000000000000003 and was refused.
const FloorToleranceCm = 1e-6;

// A TASK'S FLOOR: the surface its object stands on. The desk's for every task
// whose object is on the desk or in the bin, and for anything without a task
// object; for an object on a declared surface (back end/DeclaredSurfaces.json,
// AiMeasurements TaskSupport) that surface's, the same 1 cm under it. A surface
// above the desk holds the floor everywhere outside the basket (it is a solid
// box or board: the arm has nothing to reach under it in that task); one below
// the desk only over its own extent, the desk's elsewhere, each gripper point
// held to the floor under it. Plain data, so it goes to the snap's worker.
// A depth-observed scene instead applies its support locally; scene solids and observed-space
// route checks hold the rest. Outside that support the desk guard remains until observed
// transit clearance can be certified: missing depth is never treated as empty space.
export type AiFloor = { SurfaceZ: number; FloorZ: number; Surface: DeclaredSurface | null; Scene?: MeasuredScene | null };
export const DeskFloor: AiFloor = { SurfaceZ: TableZCm, FloorZ: AiTableFloorCm, Surface: null };
export const FloorBelowSurfaceCm = 1.0;

/** The floor of a task whose object stands on a declared surface measured at MeasuredZ: under the higher of the two. */
export function SurfaceFloor(Surface: DeclaredSurface, MeasuredZ: number | null = null): AiFloor {
  const SurfaceZ = Math.max(Surface.TopZCm, MeasuredZ ?? Surface.TopZCm);
  return { SurfaceZ, FloorZ: SurfaceZ - FloorBelowSurfaceCm, Surface };
}
/** Whether a floor is the desk's (DeskFloor, or one equal to it). */
export const IsDeskFloor = (Floor: AiFloor) => !Floor.Surface;
/** The floor's Z under a display point. */
export function FloorZAt(Floor: AiFloor, Point: { x: number; y: number }): number {
  if (!Floor.Surface || (!Floor.Scene && Floor.SurfaceZ >= TableZCm)) return Floor.FloorZ;
  return SurfaceContains(Floor.Surface, Point) ? Floor.FloorZ : AiTableFloorCm;
}
/**
 * The floor as the refusals and the prompt name it: "the table floor (Z -3.3 cm, 1 cm under the table surface)",
 * or "this task's floor (Z 11.7 cm, 1 cm under the platform "box platform" the task object stands on)".
 */
export function FloorText(Floor: AiFloor, Z = Floor.FloorZ) {
  return !Floor.Surface || Z !== Floor.FloorZ ? `the table floor (Z ${AiTableFloorCm.toFixed(1)} cm, 1 cm under the table surface)`
    : `this task's floor (Z ${Floor.FloorZ.toFixed(1)} cm, 1 cm under the ${Floor.Surface.Kind} "${Floor.Surface.Name}" the task object stands on)`;
}

/**
 * A model-frame point over the top tray's open footprint, where lower targets
 * are collision-checked. Not the whole cart's: that reaches 5.8 cm past the
 * tray's front rim over the desk, and low targets there were exempt from the
 * floor once the rim moved to where the depth camera sees it (display Y 23.3).
 */
export function OverBasket(Point: { x: number; y: number }) {
  return Point.x >= TrayBounds[0].x && Point.x <= TrayBounds[1].x
    && Point.y >= TrayBounds[0].y && Point.y <= TrayBounds[1].y;
}
// Where that footprint ends toward the desk, in the AI's display frame (for the prompt).
export const TrayFrontRimYCm = mirrorDisplayPoint({ x: 0, y: TrayBounds[1].y, z: 0 }).y;

/**
 * Why an AI target (model frame) breaks the floor (the task's, DeskFloor without
 * one), or null. A move that rises is always allowed.
 */
export function AiTableFloorReason(Target: Point3, RiseCm: number, Floor: AiFloor = DeskFloor): string | null {
  const Display = mirrorDisplayPoint(Target);
  const FloorZ = FloorZAt(Floor, Display);
  if (Display.z >= FloorZ - FloorToleranceCm || RiseCm >= 0 || OverBasket(Target)) return null;
  if (FloorZ === AiTableFloorCm) {
    return `Target Z ${Display.z.toFixed(1)} cm is below the table floor (Z ${AiTableFloorCm.toFixed(1)} cm, 1 cm under the table surface); keep the tip at or above it.`;
  }
  return `Target Z ${Display.z.toFixed(1)} cm is below ${FloorText(Floor)}; keep the tip at or above it.`;
}

/**
 * The lowest point of the gripper and its moving jaw in this pose, as a Z in
 * the AI's display frame. The points are the convex hull of their meshes
 * (scripts/BuildGripperHull.py): the collision boxes hang up to 2 cm below the
 * real jaw at a flat wrist.
 */
export function GripperLowestZ(Angles: CadAngles): number {
  return GripperLowestZOfLinks(CadForward(Angles).Links);
}
/** GripperLowestZ from a pose's link transforms (CadForward(...).Links), for searches that already have them. */
export function GripperLowestZOfLinks(Links: Record<string, Matrix4>): number {
  let Lowest = Number.POSITIVE_INFINITY;
  for (const [Link, Points] of Object.entries(Hull.Links)) {
    const Elements = Links[Link].elements;
    for (const [X, Y, Z] of Points) Lowest = Math.min(Lowest, Elements[2] * X + Elements[6] * Y + Elements[10] * Z + Elements[14]);
  }
  return mirrorDisplayPoint({ x: 0, y: 0, z: Lowest }).z;
}
/**
 * How far the gripper's lowest point stands above the floor (negative: under
 * it): GripperLowestZOfLinks less the floor's Z, or with a floor that changes
 * across the arm's reach (a surface below the desk) the least over every hull
 * point of its height above the floor under it.
 */
export function GripperFloorGapOfLinks(Links: Record<string, Matrix4>, Floor: AiFloor = DeskFloor): number {
  if (!Floor.Surface || (!Floor.Scene && Floor.SurfaceZ >= TableZCm)) return GripperLowestZOfLinks(Links) - Floor.FloorZ;
  let Gap = Number.POSITIVE_INFINITY;
  for (const [Link, Points] of Object.entries(Hull.Links)) {
    const E = Links[Link].elements;
    for (const [X, Y, Z] of Points) {
      const Point = mirrorDisplayPoint({ x: E[0] * X + E[4] * Y + E[8] * Z + E[12], y: E[1] * X + E[5] * Y + E[9] * Z + E[13], z: E[2] * X + E[6] * Y + E[10] * Z + E[14] });
      Gap = Math.min(Gap, Point.z - FloorZAt(Floor, Point));
    }
  }
  return Gap;
}
export const GripperFloorGap = (Angles: CadAngles, Floor: AiFloor = DeskFloor) => GripperFloorGapOfLinks(CadForward(Angles).Links, Floor);

/**
 * Why an AI target's pose (Angles, from its IK) puts the gripper below the
 * floor (the task's, DeskFloor without one), or null. The tip rule alone let a
 * flat wrist press the jaw into the desk: it hangs up to 1.3 cm under the tip.
 * A pose that does not lower the gripper below where it is now (CurrentAngles)
 * is allowed, so the arm can always rise out. The manual controls are not held to it.
 */
export function AiJawFloorReason(Target: Point3, Angles: CadAngles, CurrentAngles: CadAngles, Floor: AiFloor = DeskFloor): string | null {
  if (OverBasket(Target)) return null;
  const Gap = GripperFloorGap(Angles, Floor);
  if (AiJawFloorAllows(Target, Gap, GripperFloorGap(CurrentAngles, Floor))) return null;
  const Raise = (Math.ceil(-Gap * 10) / 10).toFixed(1);
  if (IsDeskFloor(Floor)) {
    const Lowest = Gap + AiTableFloorCm;
    return `At that wrist pitch the lowest point of the gripper would be at Z ${Lowest.toFixed(1)} cm, below the table floor (Z ${AiTableFloorCm.toFixed(1)} cm): raise the target by ${Raise} cm or use a steeper wrist.`;
  }
  return `At that wrist pitch the lowest point of the gripper would be ${(-Gap).toFixed(1)} cm below ${FloorText(Floor)}: raise the target by ${Raise} cm or use a steeper wrist.`;
}

/**
 * The rule of AiJawFloorReason on numbers: the gripper's lowest point may not
 * end under the floor outside the basket (Gap, GripperFloorGap: its height
 * above it), unless it is no lower against it than it is now (CurrentGap).
 */
export function AiJawFloorAllows(Target: Point3, Gap: number, CurrentGap: number) {
  return OverBasket(Target) || Gap >= -FloorToleranceCm || Gap >= CurrentGap - 0.05;
}
