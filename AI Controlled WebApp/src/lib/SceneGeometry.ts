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
};
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
    ...(contact !== undefined ? { task_contact: contact } : {}) };
}

export function SceneCommandFields(scene: SceneIdentity | null | undefined, carry = false): Record<string, unknown> {
  return scene ? { scene_id: scene.id, scene_required: true, scene_carry: carry } : {};
}
export function SceneFreshnessIssue(proposal: SceneIdentity | null | undefined, latest: SceneIdentity | null | undefined): string | null {
  if (!proposal) return null;
  return !latest || proposal.id !== latest.id || proposal.revision !== latest.revision
    ? "The scene changed since this move was checked. Take a fresh look and plan again." : null;
}
