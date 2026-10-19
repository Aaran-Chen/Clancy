import type { Matrix4 } from "three";
import { CadForward, CadPitch, CadSolve, CadSolveCandidates, GetCadAlignment } from "./CadKinematics";
import { BasketPoseReason, BasketTargetReason } from "./BasketCollision";
import { FindReachableWristTarget, PreviewConverges, type WristSearchOptions } from "./AutomaticWrist";
import type { PitchGuideSegment } from "./PitchGuide";
import { OverBasket } from "./AiTableFloor";
import { currentAngles, GetIkAngleLimits, IK_JOINTS, solveIkPreview, type ArmAngles, type IkAngleLimits,
  type ServoName, type ServoState, type TargetPose } from "./robotModel";

export type TargetWorkspaceContext = {
  Servos: Record<ServoName, ServoState>; Limits?: IkAngleLimits | null; ElbowMode: "up" | "down";
};
const Wrap = (Angle: number) => ((Angle + 180) % 360 + 360) % 360 - 180;
const DistanceSquared = (First: TargetPose, Second: TargetPose) =>
  (First.x - Second.x) ** 2 + (First.y - Second.y) ** 2 + (First.z - Second.z) ** 2;

/** Joint limits pulled in by InsetDeg at both ends (a range narrower than that collapses to its middle). */
export function InsetIkLimits(Limits: IkAngleLimits, InsetDeg: number): IkAngleLimits {
  return Object.fromEntries(IK_JOINTS.flatMap(Name => {
    const Range = Limits[Name];
    if (!Range) return [];
    const Middle = (Range[0] + Range[1]) / 2;
    return [[Name, Range[1] - Range[0] > 2 * InsetDeg ? [Range[0] + InsetDeg, Range[1] - InsetDeg] : [Middle, Middle]]];
  })) as IkAngleLimits;
}

// The AI target snap's bounded search (AiTargetSnap.ts): only targets near
// the request matter, under rules the manual GUI does not have. Model frame.
export type NearestOptions = {
  // Targets farther than this from the request are not accepted.
  MaxCm: number;
  // Poses whose tip is farther than this are dropped unexamined (default MaxCm + 2).
  SearchCm?: number;
  // The tip never ends below this Z.
  MinZ?: number | null;
  // Stay on the request's side of the basket footprint and, over it, of the rim height RimZ.
  SameBasketSide?: boolean;
  RimZ?: number | null;
  // The wrist pitch stays within this many deg of the request's (the IK seeds span it; 90 without one).
  PitchWindowDeg?: number | null;
  // The tip moves at most this far sideways (X/Y).
  MaxXYCm?: number | null;
  // Joint limits pulled in by this much, so the target is not at a stop.
  LimitInsetDeg?: number;
  // Deterministic budget: pose evaluations, not wall time.
  MaxEvaluations?: number;
  // Spacing of the IK seeds across the pitch window.
  SeedPitchStepDeg?: number;
  // Between targets about as near, prefer the smaller wrist change: each deg
  // of it counts as this many cm in the search (never in DistanceCm).
  PitchCostCmPerDeg?: number;
  // Clearance from the basket and body beyond the collision model's, in cm:
  // the nearest target lies on a contact boundary, which the bridge's check of
  // the pose rounded to encoder steps crossed (13 of 33 grid snaps without it).
  CollisionMarginCm?: number;
  // One more rule per pose (the AI's jaw floor), with the pose's link transforms.
  PoseAllowed?: (Angles: ArmAngles, Target: TargetPose, Links: Record<string, Matrix4>) => boolean;
};
export type NearestReport = { Target: TargetPose | null; DistanceCm: number; Evaluations: number };

/**
 * The nearest target to Requested under Options, found near it only: IK
 * seeds for pitches across the window (and their neighbours when infeasible),
 * refined by a pattern search in joint space. Every result converges in the
 * IK the app sends with (errorCm <= 0.02, pitch within 0.08 deg) on the inset
 * limits, so later checks at the normal tolerance and the bridge accept it.
 * Target null when nothing allowed lies within SearchCm.
 */
export function FindNearestLocalTarget(Requested: TargetPose, Context: TargetWorkspaceContext, Options: NearestOptions): NearestReport {
  let Evaluations = 0;
  if (!Object.values(Requested).every(Number.isFinite)) return { Target: null, DistanceCm: Infinity, Evaluations };
  const { Servos, ElbowMode } = Context;
  const Limits = InsetIkLimits(GetIkAngleLimits(Servos, Context.Limits ?? undefined), Options.LimitInsetDeg ?? 0);
  const Budget = Options.MaxEvaluations ?? Infinity;
  const SearchSquared = (Options.SearchCm ?? Options.MaxCm + 2) ** 2;
  const RimZ = Options.SameBasketSide ? Options.RimZ ?? null : null;
  const OverRequested = OverBasket(Requested);
  const Allowed = (Target: TargetPose, Angles: ArmAngles, Links: Record<string, Matrix4>) =>
    (Options.MinZ == null || Target.z >= Options.MinZ - 1e-6)
    && (!Options.SameBasketSide || OverBasket(Target) === OverRequested)
    && (RimZ === null || !OverRequested || (Target.z >= RimZ) === (Requested.z >= RimZ))
    && (Options.PitchWindowDeg == null || Math.abs(Wrap(Target.wristPitch - Requested.wristPitch)) <= Options.PitchWindowDeg)
    && (Options.MaxXYCm == null || Math.hypot(Target.x - Requested.x, Target.y - Requested.y) <= Options.MaxXYCm + 1e-9)
    && (!Options.PoseAllowed || Options.PoseAllowed(Angles, Target, Links));
  const Initial = currentAngles(Servos);
  const Names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex"] as const;
  const Ranges = Names.map(Name => Limits[Name] ?? [-180, 180]);
  type Candidate = { Values: number[]; Target: TargetPose; Score: number };
  const PitchCost = Options.PitchCostCmPerDeg ?? 0, Margin = Options.CollisionMarginCm ?? 0;
  const Evaluate = (Values: number[]): Candidate | null => {
    if (Evaluations >= Budget) return null;
    Evaluations += 1;
    if (Values.some((Value, Index) => Value < Ranges[Index][0] || Value > Ranges[Index][1])) return null;
    const Angles = { ...Initial, ...Object.fromEntries(Names.map((Name, Index) => [Name, Values[Index]])) } as ArmAngles;
    const State = CadForward(Angles);
    const Target = { x: State.Tool.x, y: State.Tool.y, z: State.Tool.z,
      wristPitch: Requested.wristPitch + Wrap(CadPitch(Angles) - Requested.wristPitch) };
    const Squared = DistanceSquared(Target, Requested);
    // Far poses never matter; the collision checks are the costly part.
    if (Squared > SearchSquared || !Allowed(Target, Angles, State.Links) || BasketTargetReason(Target) || BasketPoseReason(Angles, Margin)) return null;
    return { Values, Target, Score: Squared + (PitchCost * (Target.wristPitch - Requested.wristPitch)) ** 2 };
  };
  const Clamp = (Values: number[]) => Values.map((Value, Index) => Math.max(Ranges[Index][0], Math.min(Ranges[Index][1], Value)));
  const Raw: number[][] = [Names.map(Name => Initial[Name])];
  const HalfWidth = Options.PitchWindowDeg ?? 90, Step = Options.SeedPitchStepDeg ?? 5;
  for (let Offset = -HalfWidth; Offset <= HalfWidth + 1e-9; Offset += Step) {
    for (const Solution of CadSolveCandidates(Requested, Requested.wristPitch + Offset, ElbowMode, Initial, Limits, false).slice(0, 4)) {
      Raw.push(Names.map(Name => Solution.Angles[Name]));
    }
  }
  const Free = CadSolve(Requested, Requested.wristPitch, ElbowMode, Initial, Limits, true);
  Raw.push(Names.map(Name => Free.Angles[Name]));
  const Seeds: Candidate[] = [];
  for (const Values of Raw) {
    const Seed = Evaluate(Clamp(Values));
    if (Seed) { Seeds.push(Seed); continue; }
    // Infeasible (a collision, the window, the floor): feasible poses around it.
    for (const Radius of [3, 8]) for (let Index = 0; Index < 81; Index++) {
      if (Index === 40) continue;
      const Offsets = [Index % 3, Math.floor(Index / 3) % 3, Math.floor(Index / 9) % 3, Math.floor(Index / 27) % 3].map(Digit => (Digit - 1) * Radius);
      const Neighbour = Evaluate(Clamp(Values.map((Value, Joint) => Value + Offsets[Joint])));
      if (Neighbour) Seeds.push(Neighbour);
    }
  }
  Seeds.sort((First, Second) => First.Score - Second.Score);
  const Starts: Candidate[] = [];
  for (const Seed of Seeds) {
    if (Starts.every(Other => Math.hypot(...Seed.Values.map((Value, Index) => Value - Other.Values[Index])) > 6)) Starts.push(Seed);
    if (Starts.length >= 6) break;
  }
  let Directions: number[][] = [[]];
  for (const _Name of Names) Directions = Directions.flatMap(Values => [-1, 0, 1].map(Direction => [...Values, Direction]));
  Directions = Directions.filter(Values => Values.some(Boolean));
  const Refined = Starts.map(Start => {
    let Best = Start;
    for (let Size = 4; Size >= 0.015625; Size /= 2) {
      for (let Iteration = 0; Iteration < 32; Iteration++) {
        let Next = Best;
        for (const Direction of Directions) {
          const Trial = Evaluate(Clamp(Best.Values.map((Value, Index) => Value + Direction[Index] * Size)));
          if (Trial && Trial.Score < Next.Score - 1e-12) Next = Trial;
        }
        if (Next === Best) break;
        Best = Next;
      }
    }
    return Best;
  });
  // The IK Send runs (from the servo seed) must converge on the target, with
  // its own pose passing the same rules.
  const Converges = (Target: TargetPose) => {
    const Preview = solveIkPreview(Target, ElbowMode, Servos, Limits);
    if (!PreviewConverges(Preview, Target.wristPitch) || (Margin > 0 && BasketPoseReason(Preview.angles, Margin))) return false;
    return Allowed(Target, Preview.angles, CadForward(Preview.angles).Links);
  };
  for (const Candidate of [...Refined, ...Seeds.slice(0, 12)].sort((First, Second) => First.Score - Second.Score).slice(0, 24)) {
    if (Converges(Candidate.Target)) return { Target: Candidate.Target, DistanceCm: Math.sqrt(DistanceSquared(Candidate.Target, Requested)), Evaluations };
  }
  return { Target: null, DistanceCm: Infinity, Evaluations };
}

// A manual release has half a second (App.tsx TargetSnapLimitMs), and the joint-space search
// below takes 3-18 s. Instead: the farthest valid point on the drag line from the last valid
// target toward the release, by bisection (about a dozen IK solves). With more than one crossing
// it may stop at an earlier edge; the answer is always checked valid, never an invalid target.
export function FindValidAlongLine(Requested: TargetPose, Previous: TargetPose, Context: TargetWorkspaceContext, Steps = 14): TargetPose | null {
  if (![...Object.values(Requested), ...Object.values(Previous)].every(Number.isFinite)) return null;
  const Valid = (Target: TargetPose) => solveIkPreview(Target, Context.ElbowMode, Context.Servos, Context.Limits ?? undefined).reachable;
  const At = (Fraction: number): TargetPose => ({ x: Previous.x + (Requested.x - Previous.x) * Fraction,
    y: Previous.y + (Requested.y - Previous.y) * Fraction, z: Previous.z + (Requested.z - Previous.z) * Fraction,
    wristPitch: Previous.wristPitch + Wrap(Requested.wristPitch - Previous.wristPitch) * Fraction });
  if (!Valid(Previous)) return null;
  let Low = 0, High = 1;
  for (let Step = 0; Step < Steps; Step++) {
    const Middle = (Low + High) / 2;
    if (Valid(At(Middle))) Low = Middle; else High = Middle;
  }
  return Low > 0 ? At(Low) : null;
}

// Search physical joint space, optionally allowing wrist pitch to change. A Cartesian
// line-backtrack cannot find the nearest side of a basket or a joint-limit
// boundary. Multiple joint-space seeds cover both elbow branches and pan wraps;
// local refinement minimizes XYZ distance without weakening collision checks.
// This runs in a worker on release, never in the pointer-move/render loop.
// Options (AI proposals only): the bounded local search above, and a target
// only within Options.MaxCm. Without it the manual GUI's global search is unchanged.
// Wrist (manual release): the wrist-only search's pitch guide and step, so it answers fast.
export function FindNearestValidTarget(Requested: TargetPose, Previous: TargetPose, Context: TargetWorkspaceContext, AdjustWrist = false,
  Options?: NearestOptions, Wrist?: { Guide?: PitchGuideSegment[]; Search?: WristSearchOptions }): TargetPose | null {
  if (Options) {
    const Report = FindNearestLocalTarget(Requested, Context, Options);
    return Report.Target && Report.DistanceCm <= Options.MaxCm ? Report.Target : null;
  }
  if (!Object.values(Requested).every(Number.isFinite)) return null;
  const { Servos, ElbowMode, Limits } = Context;
  const Validate = (Target: TargetPose) => solveIkPreview(Target, ElbowMode, Servos, Limits ?? undefined).reachable;
  if (Validate(Requested)) return { ...Requested };
  if (AdjustWrist) {
    const Exact = FindReachableWristTarget(Requested, Context, Wrist?.Guide, Wrist?.Search);
    if (Exact) return Exact;
  }
  const Initial = currentAngles(Servos);
  const Bounds = GetIkAngleLimits(Servos, Limits ?? undefined);
  const Names: ("shoulder_pan" | "shoulder_lift" | "elbow_flex" | "wrist_flex")[] = AdjustWrist
    ? ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex"] : ["shoulder_pan", "shoulder_lift", "elbow_flex"];
  const Ranges = Names.map(Name => Bounds[Name] ?? [-180, 180]);
  const WristBounds = Bounds.wrist_flex ?? [-180, 180];
  const Direction = GetCadAlignment().Direction.wrist_flex;
  type Candidate = { Values: number[]; Target: TargetPose; Score: number };
  const Evaluate = (Values: number[]): Candidate | null => {
    if (Values.some((Value, Index) => Value < Ranges[Index][0] || Value > Ranges[Index][1])) return null;
    const Angles = { ...Initial, wrist_flex: 0, ...Object.fromEntries(Names.map((Name, Index) => [Name, Values[Index]])) } as ArmAngles;
    if (!AdjustWrist) {
      const Wrist = Wrap(CadPitch(Angles) - Requested.wristPitch) / Direction;
      const Options = [-360, 0, 360].map(Turn => Wrist + Turn)
        .filter(Value => Value >= WristBounds[0] - 0.0001 && Value <= WristBounds[1] + 0.0001)
        .map(Value => Math.max(WristBounds[0], Math.min(WristBounds[1], Value)));
      if (!Options.length) return null;
      Angles.wrist_flex = Options.sort((A, B) => Math.abs(A - Initial.wrist_flex) - Math.abs(B - Initial.wrist_flex))[0];
      // The source CAD transforms are rounded; allow their sub-millidegree
      // residual at an exact joint stop, well inside the existing IK tolerance.
      if (Math.abs(Wrap(CadPitch(Angles) - Requested.wristPitch)) > 0.0001) return null;
    }
    const Point = CadForward(Angles).Tool;
    const Target = { x: Point.x, y: Point.y, z: Point.z, wristPitch: AdjustWrist ? CadPitch(Angles) : Requested.wristPitch };
    if (BasketTargetReason(Target) || BasketPoseReason(Angles)) return null;
    return { Values, Target, Score: DistanceSquared(Target, Requested) };
  };
  const Seeds: Candidate[] = [];
  const Add = (Values: number[]) => { const Candidate = Evaluate(Values); if (Candidate) Seeds.push(Candidate); };
  Add(Names.map(Name => Initial[Name]));
  for (const Target of [Requested, Previous]) {
    const Preview = solveIkPreview(Target, ElbowMode, Servos, Limits ?? undefined);
    Add(Names.map(Name => Preview.angles[Name]));
  }
  // Include endpoints so narrow joint ranges and physical stops are represented.
  const Counts = AdjustWrist ? [6, 8, 8, 6] : [12, 16, 16];
  const Sample = (Values: number[]) => {
    const Index = Values.length;
    if (Index === Names.length) { Add(Values); return; }
    for (let Step = 0; Step <= Counts[Index]; Step++) Sample([...Values, Ranges[Index][0] + Step / Counts[Index] * (Ranges[Index][1] - Ranges[Index][0])]);
  };
  Sample([]);
  Seeds.sort((A, B) => A.Score - B.Score);
  const Starts: Candidate[] = [];
  for (const Seed of Seeds) {
    if (Starts.every(Other => Math.hypot(...Seed.Values.map((Value, Index) => Value - Other.Values[Index])) > 12)) Starts.push(Seed);
    if (Starts.length >= 12) break;
  }
  let Directions: number[][] = [[]];
  for (const Name of Names) Directions = Directions.flatMap(Values => [-1, 0, 1].map(Direction => [...Values, Direction]));
  Directions = Directions.filter(Values => Values.some(Boolean));
  const Refined = Starts.map(Start => {
    let Best = Start;
    for (let Step = 8; Step >= 0.015625; Step /= 2) {
      for (let Iteration = 0; Iteration < 32; Iteration++) {
        let Next = Best;
        for (const Direction of Directions) {
          const Trial = Evaluate(Best.Values.map((Value, Index) => Math.max(Ranges[Index][0], Math.min(Ranges[Index][1], Value + Direction[Index] * Step))));
          if (Trial && Trial.Score < Next.Score - 1e-12) Next = Trial;
        }
        if (Next === Best) break;
        Best = Next;
      }
    }
    return Best;
  });
  // Use the same IK branch selection as Send, not just the generating pose.
  for (const Candidate of [...Refined, ...Seeds.slice(0, 24)].sort((A, B) => A.Score - B.Score)) {
    if (Validate(Candidate.Target)) return Candidate.Target;
  }
  return null;
}
