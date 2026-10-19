import type { SceneBox } from "./SceneGeometry";

/** Exact AABB union coverage, without turning separated observations into one fictitious free box. */
export function ObservedBoxCovered(Box: SceneBox, Free: SceneBox[]): boolean {
  if (!Box.min.every((Value, Axis) => Number.isFinite(Value) && Number.isFinite(Box.max[Axis]) && Value < Box.max[Axis])) return false;
  let Missing: SceneBox[] = [Box];
  for (const Seen of Free) {
    const Next: SceneBox[] = [];
    for (const Part of Missing) {
      const Low = Part.min.map((Value, Axis) => Math.max(Value, Seen.min[Axis]));
      const High = Part.max.map((Value, Axis) => Math.min(Value, Seen.max[Axis]));
      if (Low.some((Value, Axis) => Value >= High[Axis])) { Next.push(Part); continue; }
      const Core: SceneBox = { min: [...Part.min], max: [...Part.max] };
      for (const Axis of [0, 1, 2]) {
        if (Core.min[Axis] < Low[Axis]) {
          const Piece: SceneBox = { min: [...Core.min], max: [...Core.max] };
          Piece.max[Axis] = Low[Axis]; Next.push(Piece); Core.min[Axis] = Low[Axis];
        }
        if (Core.max[Axis] > High[Axis]) {
          const Piece: SceneBox = { min: [...Core.min], max: [...Core.max] };
          Piece.min[Axis] = High[Axis]; Next.push(Piece); Core.max[Axis] = High[Axis];
        }
      }
    }
    Missing = Next;
    if (!Missing.length) return true;
    // Excessively fragmented evidence cannot certify a corridor in a bounded UI planning pass.
    if (Missing.length > 512) return false;
  }
  return false;
}
