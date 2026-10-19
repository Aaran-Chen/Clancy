import { BasketTargetReason } from "./BasketCollision";
import { TargetApproachDirection } from "./CadViewControls";
import { solveIkPreview, type TargetPose } from "./robotModel";
import type { TargetWorkspaceContext } from "./TargetWorkspace";

export type PitchGuidePoint = { Pitch: number; Direction: number[] };
export type PitchGuideSegment = { Start: PitchGuidePoint; End: PitchGuidePoint };
export const PitchGuideRadiusCm = 7.5;

// Sample the same fixed-XYZ, joint-limited, basket-aware IK used by Send.
// Keep disjoint acceptable ranges separate and refine each blocked boundary.
export function BuildPitchGuide(Target: TargetPose, Context: TargetWorkspaceContext): PitchGuideSegment[] {
  if (!Object.values(Target).every(Number.isFinite) || BasketTargetReason(Target)) return [];
  const Cache = new Map<number, PitchGuidePoint | null>();
  const At = (Pitch: number) => {
    if (Cache.has(Pitch)) return Cache.get(Pitch)!;
    const Preview = solveIkPreview({ ...Target, wristPitch: Pitch }, Context.ElbowMode, Context.Servos, Context.Limits ?? undefined);
    const Point = Preview.reachable ? { Pitch, Direction: TargetApproachDirection(Preview.angles, Pitch).toArray() } : null;
    Cache.set(Pitch, Point); return Point;
  };
  const Pitches = [...new Set([...Array.from({ length: 481 }, (_, Index) => -180 + Index * .75),
    ((Target.wristPitch + 180) % 360 + 360) % 360 - 180])].sort((A, B) => A - B);
  const Segments: PitchGuideSegment[] = [];
  const Continuous = (A: PitchGuidePoint, B: PitchGuidePoint) =>
    A.Direction.reduce((Sum, Value, Axis) => Sum + Value * B.Direction[Axis], 0) > .999;
  const Visit = (Low: number, High: number, Depth = 0) => {
    const Middle = (Low + High) / 2;
    const Start = At(Low), Midpoint = At(Middle), End = At(High);
    if (!Start && !Midpoint && !End) return;
    // Two valid endpoints can straddle an invalid island when the solver changes
    // branches. Check the middle before drawing and split all blocked boundaries.
    // IK can switch between forward- and backward-folded poses. Do not draw a
    // line through the middle of the circle across that orientation discontinuity.
    if (Start && Midpoint && End && Continuous(Start, End) && Continuous(Start, Midpoint) && Continuous(Midpoint, End)) {
      if (High - Low > 1e-6) Segments.push({ Start, End });
      return;
    }
    if (Depth >= 12) return;
    Visit(Low, Middle, Depth + 1);
    Visit(Middle, High, Depth + 1);
  };
  for (let Index = 1; Index < Pitches.length; Index++) {
    Visit(Pitches[Index - 1], Pitches[Index]);
  }
  return Segments;
}
