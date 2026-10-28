import type { DeclaredSurface } from "./DeclaredSurfaces";

export type MeasuredSupport = DeclaredSurface & { Source: "depth"; ObservedOnly: true };
export type SceneBox = { min: [number, number, number]; max: [number, number, number] };
export type MeasuredScene = {
  id: string;
  revision: string;
  frame: "display_cm";
  capture_seq: number;
  support: MeasuredSupport;
  obstacles: (SceneBox & { label: string })[];
  free_space?: SceneBox[];
  // Space known to hold nothing to avoid though the camera did not see through it (back end/SceneGeometry.py Build):
  // the arm where it stood for the capture, and the task object's reach, unobserved there and taken as empty (TaskReach).
  known_space?: SceneBox[];
  task_object?: SceneBox;
  // Observed solid touching the task object's measured footprint (back end/SceneGeometry.py TaskContact), null where none:
  // absent from an older bridge's scene (GraspPlan.SceneConflict then compares the boxes).
  task_contact?: (SceneBox & { points: number }) | null;
  // How far the support leans in the display frame (the bridge's plane fit), and the depth uncertainty the boxes are
  // padded by (back end/SceneGeometry.py DepthUncertaintyCm); absent from an older bridge.
  support_tilt_deg?: number;
  depth_uncertainty_cm?: number;
};
// measured: the solid the bridge measured against the task object (SceneContactSolid), its points' box unpadded, and
// removed: the footprint the bridge took out of the scene as the object (its measurement, a neighbour's taken part in
// it). against: the object stands against it where it was measured (that solid, or a cell against its envelope that
// is that solid too: DeskSolids), the one kind of solid a held object may start within the carry margin of
// (GraspPlan CarriedObjectPathReason).
export type SceneSolid = SceneBox & { label: string; measured?: SceneBox; removed?: SceneBox; against?: boolean };
export type SceneIdentity = Pick<MeasuredScene, "id" | "revision">;

export function HasVerifiedDepthFrame(value: { depthMeasured: boolean; scene?: MeasuredScene | null;
  sceneChecked?: boolean; localizationFrame?: string | null }): boolean {
  return value.depthMeasured && !!value.scene && value.sceneChecked === true && value.localizationFrame === "calibrated_depth_3d";
}

const RecordOf = (value: unknown): value is Record<string, unknown> => typeof value === "object" && value !== null;
const Finite = (value: unknown): value is number => typeof value === "number" && Number.isFinite(value);
function Range(value: unknown): value is [number, number] {
  return Array.isArray(value) && value.length === 2 && value.every(Finite) && value[0] < value[1];
}
export function ReadMeasuredSupport(value: unknown): MeasuredSupport | null {
  if (!RecordOf(value) || value.Source !== "depth" || value.ObservedOnly !== true || value.Frame !== "cart"
    || typeof value.Name !== "string" || !value.Name.trim()
    || (value.Kind !== "platform" && value.Kind !== "shelf") || !Finite(value.TopZCm)
    || !Range(value.XRangeCm) || !Range(value.YRangeCm)
    || (value.BoardAboveZCm !== null && !Finite(value.BoardAboveZCm))
    || (value.Kind === "shelf" && (!Finite(value.BoardAboveZCm) || value.BoardAboveZCm <= value.TopZCm))) return null;
  return { Name: value.Name, Kind: value.Kind, Source: "depth", ObservedOnly: true, Frame: "cart", TopZCm: value.TopZCm,
    XRangeCm: [...value.XRangeCm], YRangeCm: [...value.YRangeCm], BoardAboveZCm: value.BoardAboveZCm as number | null };
}
function ReadBox(value: unknown): SceneBox | null {
  if (!RecordOf(value) || !Array.isArray(value.min) || !Array.isArray(value.max)
    || value.min.length !== 3 || value.max.length !== 3 || !value.min.every(Finite) || !value.max.every(Finite)
    || !value.min.every((n, i) => n < (value.max as number[])[i]!)) return null;
  return { min: [...value.min] as SceneBox["min"], max: [...value.max] as SceneBox["max"] };
}
/** Keep the capture identity and its geometry together; partial geometry is never a legacy desk reading. */
export function ReadMeasuredScene(value: unknown): MeasuredScene | null {
  if (!RecordOf(value) || typeof value.id !== "string" || !value.id.trim()
    || typeof value.revision !== "string" || !value.revision.trim() || value.frame !== "display_cm"
    || !Finite(value.capture_seq) || !Number.isInteger(value.capture_seq) || value.capture_seq < 0
    || !Array.isArray(value.obstacles)) return null;
  const support = ReadMeasuredSupport(value.support);
  const obstacles = value.obstacles.map(item => {
    const box = ReadBox(item);
    return box && RecordOf(item) && typeof item.label === "string" ? { ...box, label: item.label } : null;
  });
  if (!support || obstacles.some(item => !item)) return null;
  const free = value.free_space === undefined ? undefined : Array.isArray(value.free_space) ? value.free_space.map(ReadBox) : null;
  if (free === null || free?.some(item => !item)) return null;
  const known = value.known_space === undefined ? undefined : Array.isArray(value.known_space) ? value.known_space.map(ReadBox) : null;
  if (known === null || known?.some(item => !item)) return null;
  const task = value.task_object === undefined ? undefined : ReadBox(value.task_object);
  if (task === null) return null;
  const contactBox = RecordOf(value.task_contact) ? ReadBox(value.task_contact) : null;
  const contact = value.task_contact === undefined || value.task_contact === null ? value.task_contact
    : contactBox && RecordOf(value.task_contact) && Finite(value.task_contact.points) ? { ...contactBox, points: value.task_contact.points } : false;
  if (contact === false) return null;
  return { id: value.id, revision: value.revision, frame: "display_cm", capture_seq: value.capture_seq,
    support, obstacles: obstacles as MeasuredScene["obstacles"], ...(free ? { free_space: free as SceneBox[] } : {}),
    ...(known ? { known_space: known as SceneBox[] } : {}), ...(task ? { task_object: task } : {}),
    ...(contact !== undefined ? { task_contact: contact } : {}),
    ...(Finite(value.support_tilt_deg) ? { support_tilt_deg: value.support_tilt_deg } : {}),
    ...(Finite(value.depth_uncertainty_cm) && value.depth_uncertainty_cm >= 0 ? { depth_uncertainty_cm: value.depth_uncertainty_cm } : {}) };
}

/**
 * A DESK SCENE (2026-10-18): a scene the bridge registered for an object lying on the desk (support_branch "desk").
 * The scene layer was built for cabinet and shelf grasps, and on the desk it refused the white holder at X 2.0, Y 34.4
 * (2026-10-18 20:24) at every wrist pitch: the desk under it was an obstacle the fixed finger had to keep 0.4 cm from,
 * the flying pixels at its edge (0.5-0.7 cm over the desk, just past the bridge's 0.3 cm rim cut-out) were solids the
 * closing jaw had to keep 0.9 cm from, and every gripper box beside it had to be observed free, which no 2 cm cube
 * within a centimetre of a surface or in the object's own shadow ever is (H2, the bridge's AssumeTaskReachFree, off).
 * Such an object plans as a desk object, as every scene-less one does (the desk floor, its measured neighbours, the
 * basket, cart and body; the bridge's collision world without a scene), and the scene's solids that stand clear of
 * the desk stay obstacles for the app's planner and guard (TaskSupport's floor, GraspPlan ScenePrisms): not the
 * support's own cells, not the desk wherever it lies within SupportBandCm of the support's level (that level risen
 * by the support's tilt over the distance from the object, MaxTiltDeg at most), and not the cells against the
 * object's own envelope where the bridge measured no solid touching it (task_contact null: they are its rim). Each
 * solid keeps its padded box.
 * A SOLID AGAINST IT (task_contact, 2026-10-22): the cells against the envelope are that solid too (the bridge merges
 * cells into long boxes: the pill bottle's keyboard ran 19 cm past it in 20 of them), so each keeps what lies outside
 * the footprint the bridge removed as the object (back end/SceneGeometry.py Build: bounds_cm grown by TargetRimCm, the
 * envelope less its padding; above the envelope too), and the contact's own box is a solid (SceneContactSolid). Those
 * parts and the contact are what the object stands against (against): a held object may start within the carry
 * margin of them, of no other solid (GraspPlan CarriedObjectPathReason); and what of that solid the object's
 * measurement took in is a solid beside the object too (GraspPlan TaskContactPrisms, from the contact's removed).
 */
export const DeskSceneRules = { DepthUncertaintyCm: 0.5, SupportBandCm: 1.0, MaxTiltDeg: 10, TargetRimCm: 0.3 };
export const SceneContactLabel = "observed environment against it";
/**
 * The solid the bridge measured against the task object (task_contact), its points' box padded by the depth uncertainty
 * as the scene's cells are, with the footprint the bridge removed as the object (RemovedFootprint); null without one.
 */
export function SceneContactSolid(scene: MeasuredScene): SceneSolid | null {
  const contact = scene.task_contact;
  if (!contact) return null;
  const pad = scene.depth_uncertainty_cm ?? DeskSceneRules.DepthUncertaintyCm;
  const removed = RemovedFootprint(scene);
  return { label: SceneContactLabel, min: contact.min.map(v => v - pad) as SceneBox["min"], max: contact.max.map(v => v + pad) as SceneBox["max"],
    measured: { min: [...contact.min], max: [...contact.max] }, ...(removed ? { removed } : {}), against: true };
}
/**
 * The footprint the bridge removed from the scene as the task object (back end/SceneGeometry.py Build: its bounds_cm
 * grown by TargetRimCm, the envelope less its padding), over the envelope's height; null without an envelope.
 */
export function RemovedFootprint(scene: MeasuredScene): SceneBox | null {
  const task = scene.task_object;
  if (!task) return null;
  const inset = (scene.depth_uncertainty_cm ?? DeskSceneRules.DepthUncertaintyCm) - DeskSceneRules.TargetRimCm;
  return { min: [task.min[0] + inset, task.min[1] + inset, task.min[2]], max: [task.max[0] - inset, task.max[1] - inset, task.max[2]] };
}
const overlaps = (a: SceneBox, b: SceneBox) => [0, 1, 2].every(i => a.min[i]! < b.max[i]! && a.max[i]! > b.min[i]!);
/** What of box a lies outside box b: a where they do not overlap, else up to six boxes. */
export function BoxOutside(a: SceneBox, b: SceneBox): SceneBox[] {
  if (!overlaps(a, b)) return [{ min: [...a.min], max: [...a.max] }];
  const parts: SceneBox[] = [], min = [...a.min] as SceneBox["min"], max = [...a.max] as SceneBox["max"];
  for (const i of [0, 1, 2]) {
    if (min[i]! < b.min[i]!) { const top = [...max] as SceneBox["max"]; top[i] = b.min[i]!; parts.push({ min: [...min], max: top }); min[i] = b.min[i]!; }
    if (max[i]! > b.max[i]!) { const low = [...min] as SceneBox["min"]; low[i] = b.max[i]!; parts.push({ min: low, max: [...max] }); max[i] = b.max[i]!; }
  }
  return parts;
}
export function DeskSolids(scene: MeasuredScene): SceneSolid[] {
  const task = scene.task_object, level = scene.support.TopZCm;
  const pad = scene.depth_uncertainty_cm ?? DeskSceneRules.DepthUncertaintyCm;
  const rise = Math.tan(Math.min(Math.abs(scene.support_tilt_deg ?? 0), DeskSceneRules.MaxTiltDeg) * Math.PI / 180);
  // With a solid against it: the footprint the bridge removed as the object, over the envelope's height. What lies
  // outside it of a cell against the envelope is that solid (against).
  const removed = task && scene.task_contact ? RemovedFootprint(scene) : null;
  return scene.obstacles.flatMap((box): SceneSolid[] => {
    if (box.label === "observed support") return [];
    if (!task || !overlaps(box, task)) return [box];
    return removed ? BoxOutside(box, removed).map(part => ({ ...part, label: box.label, against: true })) : [];
  }).filter(box => {
    const dx = task ? Math.max(task.min[0] - box.max[0], box.min[0] - task.max[0], 0) : 0;
    const dy = task ? Math.max(task.min[1] - box.max[1], box.min[1] - task.max[1], 0) : 0;
    return box.max[2] - pad > level + DeskSceneRules.SupportBandCm + rise * Math.hypot(dx, dy);
  }).map((box): SceneSolid => ({ min: [...box.min] as SceneBox["min"], max: [...box.max] as SceneBox["max"], label: box.label,
    ...(box.against ? { against: true } : {}) }))
    // (The solid the bridge measured against it stays one, though it lies against the object's envelope.)
    .concat(SceneContactSolid(scene) ?? []);
}

export function SceneCommandFields(scene: SceneIdentity | null | undefined, carry = false): Record<string, unknown> {
  return scene ? { scene_id: scene.id, scene_required: true, scene_carry: carry } : {};
}
export function SceneFreshnessIssue(proposal: SceneIdentity | null | undefined, latest: SceneIdentity | null | undefined): string | null {
  if (!proposal) return null;
  return !latest || proposal.id !== latest.id || proposal.revision !== latest.revision
    ? "The scene changed since this move was checked. Take a fresh look and plan again." : null;
}
