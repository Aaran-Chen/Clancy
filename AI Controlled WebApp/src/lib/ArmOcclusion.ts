import { FootprintDistance, GraspRules, GripperNear, ObjectPrism, type GraspPlan, type Prism } from "./GraspPlan";
import type { Point3 } from "./robotModel";
// Types only: AiMeasurements imports this file.
import type { LocatedObject } from "./AiMeasurements";

// THE ARM HIDES THE TASK OBJECT. With the gripper over or beside the object (at
// the hover point above it before the descent, or beside it at the grasp point)
// the camera sees the gripper, not the object. 2026-10-03 16:41-16:42, "grab the
// pill bottle": with the open jaws 3.8 cm from where it was measured at 16:41:12
// the bottle was not found at all; the model was told only "NOT detected this
// check-in (possibly under the gripper or out of view)" and a NEXT STEP that
// descended "only if the image shows the open jaws beside it", and asked twice.
// The user's "proceed" then started a task that had dropped the measurement.
// Such a check-in keeps the last full measurement (its position, footprint and
// height) and the GRASP PLAN made from it as the working ones, and says so.
// Not over its last spot, a missing object may really have moved: that stays a
// lost object (App.tsx). After a close (it may be in the jaws) or a close on
// nothing (GraspPlanState.Missed) the grasp's own rules apply instead.

export type ArmOcclusion = {
  // This check-in's reading of it with the arm over it, measured only partly and
  // not used (PartlyMeasured); null when the camera did not measure it at all.
  Partial: LocatedObject | null;
  // How far the tip is from its last measured footprint (padded), cm; 0 or less: over it.
  TipCm: number;
  // Since that measurement: seconds, and motions approved (null: not known).
  AgoS: number;
  MotionsAgo: number | null;
};
// The last full measurement as the task keeps it (TaskObjectState.LastSeen); Motions:
// the motions approved in the task when it was measured.
export type SeenMeasurement = { At: number; Object: LocatedObject; Motions?: number };

// A reading whose top moved more than this from the last full one: the jaws
// above it were measured as its top, or they hide its top.
export const PartialTopCm = 2;
// A partial reading counts as the hidden object only this near its last
// footprint: farther off it may have been pushed or knocked over.
export const PartialFromFootprintCm = 3;

// Where it was last measured, as the plan lock judges the gripper near it (GraspPlan.ts GripperNear): the
// GRASP PLAN's prism and its measured box (a point at its centre when the depth camera gave no size).
function Prisms(Seen: LocatedObject, Plan: GraspPlan | null): Prism[] {
  const Box = ObjectPrism(Seen) ?? { Label: Seen.label, Footprint: [{ x: Seen.x, y: Seen.y }], BaseZ: Seen.z, TopZ: Seen.topZ ?? Seen.z + 2,
    SupportZ: Seen.z, Centre: { x: Seen.x, y: Seen.y }, SizeCm: null };
  return Plan ? [Plan.Prism, Box] : [Box];
}

/**
 * Whether the arm is over or beside where the object was last measured: the
 * tip within GraspRules.NearCm of its footprint and at most 10 cm above its
 * top, as the plan lock holds a re-measure there occluded (GripperNear).
 * TipCm: the tip's distance from that footprint. Null: the arm is not there.
 */
export function ArmOver(Seen: LocatedObject, Tip: Point3, Plan: GraspPlan | null = null): { TipCm: number } | null {
  const Near = Prisms(Seen, Plan).filter(Prism => GripperNear(Prism, Tip));
  if (!Near.length) return null;
  return { TipCm: Math.min(...Near.map(Prism => FootprintDistance(Prism.Footprint, Tip))) };
}

/**
 * Whether a reading of the object is only part of it against its last full
 * measurement Good: no depth size where there was one, a side shrunk or grown
 * past the plan lock's ratio (GraspRules.ShrinkRatio: the jaws hide part of
 * it, or are measured with it: 2026-10-03 14:50 read the can 5.0 x 8.6, not
 * 5.6 x 4.9), its valid depth dropped (GraspRules.ValidDrop), or its top moved
 * more than PartialTopCm.
 */
export function PartlyMeasured(Reading: LocatedObject, Good: LocatedObject): boolean {
  if ((Good.depthMeasured && !Reading.depthMeasured) || (Good.sizeCm && !Reading.sizeCm)) return true;
  const Ratio = GraspRules.ShrinkRatio;
  if (Good.sizeCm && Reading.sizeCm && Reading.sizeCm.some((Size, Axis) => Size / Good.sizeCm![Axis] < Ratio || Size / Good.sizeCm![Axis] > 1 / Ratio)) return true;
  if (Reading.validFraction != null && Good.validFraction != null && Good.validFraction - Reading.validFraction > GraspRules.ValidDrop) return true;
  return Reading.topZ != null && Good.topZ != null && Math.abs(Reading.topZ - Good.topZ) > PartialTopCm;
}

/**
 * Whether the arm hides the task object at this check-in: its last full
 * measurement Seen, this check-in's reading of it (Reading, null when none was
 * picked), the tip (display frame) and the GRASP PLAN. Hidden when the arm is
 * over or beside its last spot (ArmOver) and it was not measured, or measured
 * only partly (PartlyMeasured) near that spot. Null otherwise: no measurement
 * to keep, the arm away from it (a missing object may have moved: a lost
 * object), a full re-measure (it replaces the last one), the jaws closed (it
 * may be in them) or closed on nothing (Missed: the grasp's own rules).
 * Motions: the motions approved in the task so far.
 */
export function JudgeArmOcclusion(Seen: SeenMeasurement | null, Reading: LocatedObject | null, Tip: Point3,
  Options: { Plan: GraspPlan | null; Closed: boolean; Missed: boolean; Now: number; Motions: number }): ArmOcclusion | null {
  if (!Seen || Options.Closed || Options.Missed) return null;
  const Over = ArmOver(Seen.Object, Tip, Options.Plan);
  if (!Over) return null;
  if (Reading && !(PartlyMeasured(Reading, Seen.Object)
    && Math.min(...Prisms(Seen.Object, Options.Plan).map(Prism => FootprintDistance(Prism.Footprint, Reading))) <= PartialFromFootprintCm)) return null;
  return { Partial: Reading, TipCm: Over.TipCm, AgoS: Math.max(0, Math.round((Options.Now - Seen.At) / 1000)),
    MotionsAgo: Seen.Motions != null ? Math.max(0, Options.Motions - Seen.Motions) : null };
}

const Clock = (At: number) => new Date(At).toTimeString().slice(0, 8);
const Size = (Object_: LocatedObject) => Object_.sizeCm ? `footprint about ${Object_.sizeCm[0].toFixed(1)} x ${Object_.sizeCm[1].toFixed(1)} cm` : "no footprint measured";
// A measurement as the model is given it: centre, base, top, height and footprint.
function Measurement(Object_: LocatedObject) {
  const Top = Object_.topZ != null ? `, top Z ${Object_.topZ.toFixed(1)} (${(Object_.heightCm ?? Object_.topZ - Object_.z).toFixed(1)} cm tall, ${Size(Object_)})` : "";
  return `centre X ${Object_.x.toFixed(1)}, Y ${Object_.y.toFixed(1)}, base Z ${Object_.z.toFixed(1)}${Top}`;
}

/**
 * The head of the TASK OBJECT line while the arm hides it (AiMeasurements.ts
 * TaskObjectLine): what this check-in measured of it, that its last full
 * measurement (Seen, measured At) stands with its GRASP PLAN (Planned), and
 * not to ask the user to confirm it from the image; Step: the line goes on
 * with a NEXT STEP to follow. Tag: that measurement's [UNRELIABLE] tag.
 */
export function HiddenText(Label: string, Seen: LocatedObject, At: number, Occlusion: ArmOcclusion, Planned: boolean, Step: boolean, Tag = ""): string {
  const Where = Occlusion.TipCm <= 0 ? "the tip is over its measured footprint" : `the tip is ${Occlusion.TipCm.toFixed(1)} cm from its measured footprint`;
  const Partial = Occlusion.Partial;
  const Reading = Partial
    ? `measured only partly this check-in (as "${Partial.label}" at X ${Partial.x.toFixed(1)}, Y ${Partial.y.toFixed(1)}${Partial.topZ != null ? `, top Z ${Partial.topZ.toFixed(1)}` : ""}, ${Size(Partial)}) because the gripper hides part of it, so that reading is not used`
    : "not measured this check-in because the gripper hides it";
  const Motions = Occlusion.MotionsAgo;
  const Ago = !Motions ? `${Occlusion.AgoS} s ago${Motions === 0 ? ", with no motion since" : ""}` : `${Occlusion.AgoS} s and ${Motions} motion${Motions === 1 ? "" : "s"} ago`;
  return `HIDDEN BY THE GRIPPER, not lost: ${Reading} (${Where}, so the arm covers it in this image).`
    + ` Its last full measurement stands as its working measurement: ${Measurement(Seen)}${Tag}, measured at ${Clock(At)}, ${Ago};`
    + ` the app keeps it${Planned ? " and the GRASP PLAN made from it" : ""}.`
    + ` Do not ask the user to confirm it from the image, and do not reply ask because you cannot see it${Step ? ": follow the NEXT STEP" : ""}.`
    + " Reply ask only if the image shows it somewhere else, clear of the gripper.";
}
