import { FootprintDistance, GraspRules, GripperSamples, type XY } from "./GraspPlan";
import { currentAngles, type Point3, type ServoName, type ServoState } from "./robotModel";
// Types only: AiMeasurements imports GraspPlan, which this file imports.
import type { LocatedObject, LocateResult, WithheldObject } from "./AiMeasurements";

// THE ARM MEASURED AS AN OBJECT. The bridge masks the arm out of the depth
// within 3 cm of its link centre lines (DepthGeometry.ArmExclusion); the open
// moving jaw and the palm's sides stand farther out, so where the gripper is in
// a box their depth stays in it. 2026-10-03 16:41:36, "grab the pill bottle":
// the gripper hovered over the bottle, 60% open, inside the box the detector
// gave the phone (lying flat 20 cm farther, at Y 47). The box's highest points
// were the gripper's: the "phone" was measured at X -3.6, Y 27.2, 13.2 x 12.8 cm,
// 9.8 cm tall, its top the gripper's underside, and at 16:41:40 the guard refused
// the descent onto the bottle because the open jaw would go 4.4 cm into it.
// Such a reading is the gripper's, not the object's: it is no obstacle, no
// neighbour of the task object and no object the model may target, and the
// measured list says so. As ArmOcclusion.ts does for the task object, the
// object keeps its last reading taken with the gripper clear of it (the phone's
// at 16:41:12, with the arm at home); never measured clear of it, it is left out.
//
// The rule, from the recorded frames of 2026-10-03 (the 140 measured objects
// whose check-in gave the arm's pose): the gripper as it stands now (GraspPlan.ts
// GripperSamples, the moving jaw at its angle) is over the measured footprint
// well inside its edge (the phone: 6.4 cm inside; no real object that day had
// the gripper more than 1.1 cm inside it), the measured top lies just under the
// gripper's lowest point there (1.2 cm), and no part of the gripper is in the
// box under its top (the jaws reach around a real object they grasp: the
// 14:50:40 can). A real object under a gripper hovering this low over its
// middle reads the same: it keeps its reading from before the gripper came over
// it. From a bridge that reports it, the share of the box the arm's mask took
// (arm_fraction) must also say the arm is a large part of it: the phone 0.356.
// That share alone tells nothing: the can standing in front of the raised
// gripper at 14:56:15 read 0.407.
export const ArmMadeRules = {
  // The gripper's samples reach at least this far inside the measured footprint, cm: only a box at
  // least 6 cm across both ways can be the gripper's (no can, bottle or tape roll is).
  InsideCm: 3.0,
  // The measured top at most this far under the gripper's lowest point over the footprint, cm.
  TopGapCm: 2.5,
  // The bridge's arm_fraction, when it reports one: at least this.
  MinArmFraction: 0.25,
};

export type ArmMadeObject = { Object: LocatedObject; Reason: string; InsideCm: number; GapCm: number;
  // The object's last reading with the gripper clear of it, used in its place (null: none, left out).
  Kept: ArmClearReading | null };
// An object's last reading with the gripper clear of it (not over its footprint), and when.
export type ArmClearReading = { Object: LocatedObject; At: number };
// By label: what the task (and a task that goes on from it) last measured clear of the gripper.
export type ArmClearReadings = Map<string, ArmClearReading>;

// The measured footprint as the bridge measured it (unpadded: ObjectPrism pads it for the planner).
function MeasuredFootprint(Object_: LocatedObject): XY[] | null {
  if (!Object_.depthMeasured || !Object_.sizeCm || Object_.topZ == null) return null;
  const Hx = Object_.sizeCm[0] / 2, Hy = Object_.sizeCm[1] / 2;
  return [{ x: Object_.x - Hx, y: Object_.y - Hy }, { x: Object_.x + Hx, y: Object_.y - Hy },
    { x: Object_.x + Hx, y: Object_.y + Hy }, { x: Object_.x - Hx, y: Object_.y + Hy }];
}

/** The gripper's surface samples at a servo snapshot (display frame), the moving jaw at its own angle. */
export function GripperSurface(Servos: Record<ServoName, ServoState>): Point3[] {
  const Angles = currentAngles(Servos), Jaw = Angles.gripper;
  return GripperSamples(Angles, Jaw != null ? [Jaw] : []).map(Sample => Sample.P);
}

/**
 * Whether a reading is the gripper itself (see above), against the gripper's
 * surface where the arm stood for the capture: why, or null.
 */
export function ArmMadeReason(Object_: LocatedObject, Gripper: Point3[], Rules = ArmMadeRules): ArmMadeObject | null {
  const Footprint = MeasuredFootprint(Object_);
  if (!Footprint || !Gripper.length) return null;
  if (Object_.armFraction != null && Object_.armFraction < Rules.MinArmFraction) return null;
  const Over = Gripper.map(P => ({ P, Distance: FootprintDistance(Footprint, P) })).filter(Item => Item.Distance <= 0);
  if (!Over.length) return null;
  const InsideCm = -Math.min(...Over.map(Item => Item.Distance));
  const GapCm = Math.min(...Over.map(Item => Item.P.z)) - Object_.topZ!;
  if (InsideCm < Rules.InsideCm || GapCm <= 0 || GapCm > Rules.TopGapCm) return null;
  const Share = Object_.armFraction != null ? `; the arm filled ${Math.round(Object_.armFraction * 100)}% of its box` : "";
  return { Object: Object_, InsideCm, GapCm, Kept: null,
    Reason: `the robot's own gripper, measured as an object: its measured top (Z ${Object_.topZ!.toFixed(1)}) is the underside of the gripper over it (${GapCm.toFixed(1)} cm under it, ${InsideCm.toFixed(1)} cm inside its ${Object_.sizeCm![0].toFixed(1)} x ${Object_.sizeCm![1].toFixed(1)} cm footprint${Share})` };
}

const Clock = (At: number) => new Date(At).toTimeString().slice(0, 8);

/**
 * This capture's readings without the ones that are the gripper itself: each
 * such object keeps its last reading with the gripper clear of it (Clear, by
 * label; marked armClearAt, and not found by this capture's task query), or is
 * left out when it has none; the withheld list says which reading was the
 * gripper's (not measurable, do not target). The task's own object (IsTask) is
 * only left out: ArmOcclusion.ts keeps its last full measurement and says so.
 * Every other reading with the gripper clear of its footprint (padded) becomes
 * its label's last clear one in Clear. Servos: the arm as it stood for the
 * capture (a check-in captures with the arm still). Now: the capture's time.
 */
export function WithoutArmMade(Located: LocateResult | null, Servos: Record<ServoName, ServoState>, Clear: ArmClearReadings | null = null,
  Now = Date.now(), IsTask: (Object_: LocatedObject) => boolean = () => false, Rules = ArmMadeRules): { Located: LocateResult | null; Dropped: ArmMadeObject[] } {
  if (!Located || Located.error || !Located.objects.length) return { Located, Dropped: [] };
  const Gripper = GripperSurface(Servos);
  const Dropped: ArmMadeObject[] = [], Objects: LocatedObject[] = [], Withheld: WithheldObject[] = [];
  for (const Object_ of Located.objects) {
    const Made = ArmMadeReason(Object_, Gripper, Rules);
    if (!Made) {
      const Footprint = MeasuredFootprint(Object_);
      if (Clear && Footprint && !Gripper.some(P => FootprintDistance(Footprint, P) <= GraspRules.TrimPadCm)) Clear.set(Object_.label, { Object: Object_, At: Now });
      Objects.push(Object_);
      continue;
    }
    const Kept = IsTask(Object_) ? null : Clear?.get(Object_.label) ?? null;
    Dropped.push({ ...Made, Kept });
    if (Kept) Objects.push({ ...Kept.Object, taskMatch: false, armClearAt: Kept.At });
    Withheld.push({ Label: Object_.label, Reason: `${Made.Reason}${Kept ? `; its reading at ${Clock(Kept.At)}, with the gripper clear of it, stands` : ""}` });
  }
  if (!Dropped.length) return { Located, Dropped };
  return { Located: { ...Located, objects: Objects, withheld: [...Located.withheld, ...Withheld] }, Dropped };
}
