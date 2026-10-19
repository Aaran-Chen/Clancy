import { Vector3 } from "three";
import { CadForwardUrdf, CadModel, CadPitch, CadSolve } from "./CadKinematics";
import { BasketPoseReason, BasketTargetReason } from "./BasketCollision";
import { currentAngles, GetIkAngleLimits, solveIkPreview, type IkPreview, type Point3, type TargetPose } from "./robotModel";
import type { PitchGuideSegment } from "./PitchGuide";
import type { TargetWorkspaceContext } from "./TargetWorkspace";

const Wrap = (Angle: number) => ((Angle + 180) % 360 + 360) % 360 - 180;

// Converged, not merely inside the 0.1 cm / 0.25 deg acceptance test: the tip
// within 0.02 cm and the pitch within 0.08 deg leave room for encoder rounding
// (up to 0.066 cm and 0.129 deg) and for another IK seed at a later check.
export function PreviewConverges(Preview: IkPreview, Pitch: number) {
  return Preview.reachable && Preview.errorCm <= .02 && Math.abs(Wrap(Preview.effectiveWristPitch - Pitch)) <= .08;
}
export function WristPitchConverges(Target: Point3, Pitch: number, Context: TargetWorkspaceContext) {
  return PreviewConverges(solveIkPreview({ ...Target, wristPitch: Pitch }, Context.ElbowMode, Context.Servos, Context.Limits), Pitch);
}

// The shoulder-lift axis circles the pan axis. No wrist pitch or joint limit
// puts the tip farther from it than the links' summed lengths (the triangle
// inequality), so a point beyond that is out of reach at every pitch.
const Zero = CadForwardUrdf({});
const PanAxis = new Vector3().setFromMatrixColumn(Zero.Frames.shoulder_pan, 2).normalize();
const PanOrigin = new Vector3().setFromMatrixPosition(Zero.Frames.shoulder_pan);
const LiftOffset = new Vector3().setFromMatrixPosition(Zero.Frames.shoulder_lift).sub(PanOrigin);
const LiftCentre = PanOrigin.clone().addScaledVector(PanAxis, LiftOffset.dot(PanAxis));
const LiftRadius = LiftOffset.clone().addScaledVector(PanAxis, -LiftOffset.dot(PanAxis)).length();
const LiftToTipCm = ["elbow_flex", "wrist_flex", "wrist_roll", "gripper_frame_joint"]
  .map(Name => CadModel.Joints.find(Joint => Joint.Name === Name)!)
  .reduce((Sum, Joint) => Sum + Math.hypot(...Joint.Origin.slice(0, 3).map(Row => Row[3])), 0);
/** A model-frame point no pose of the arm reaches (0.1 cm is the IK's position tolerance). */
export function BeyondArmReach(Target: Point3) {
  const Offset = new Vector3(Target.x, Target.y, Target.z).sub(LiftCentre);
  const Along = Offset.dot(PanAxis), Across = Offset.addScaledVector(PanAxis, -Along).length();
  return Math.hypot(Along, Across - LiftRadius) > LiftToTipCm + .1;
}

// AI proposals: a pitch is approved later against newer servo readings and
// sent to the bridge, which solves it again and rounds it to encoder ticks.
// On the tolerance edge it failed both (2026-09-29 07:58 and 16:10), so AI
// pitches are selected strictly and then this far inside the reachable range.
export const AiWristMarginDeg = 0.25;
/**
 * Move a converged pitch at the edge of a reachable range away from the
 * unreachable side (Invalid), rounded to 0.1 deg away from the edge, and check
 * it again; a narrow range halves the step. The edge itself when none passes.
 */
export function InsetWristPitch(Edge: number, Invalid: number, Converges: (Pitch: number) => boolean, MarginDeg = AiWristMarginDeg) {
  const Toward = Math.sign(Invalid - Edge);
  for (let Margin = MarginDeg; Toward && Margin >= .05; Margin /= 2) {
    const Pitch = (Toward > 0 ? Math.floor((Edge - Margin) * 10 + 1e-9) : Math.ceil((Edge + Margin) * 10 - 1e-9)) / 10;
    if (Converges(Pitch)) return Pitch;
  }
  return Edge;
}

export type WristSearchOptions = {
  // Step into the reachable range from its edge, in deg; 0 returns the edge itself.
  MarginDeg?: number;
  // Spacing of the pitch samples when no guide is given.
  StepDeg?: number;
  // The AI target snap: the pitch changes by at most this much (the inset
  // included), and each pose must pass Accept too (its jaw floor).
  MaxChangeDeg?: number;
  Accept?: (Target: TargetPose, Preview: IkPreview) => boolean;
};

// Worker-only search: preserve XYZ first, then minimize the change in pitch.
// The free-pitch numerical seed covers narrow solutions between guide samples.
// Every accepted pitch is rechecked with the strict solver used before Send.
export function FindReachableWristTarget(Requested: TargetPose, Context: TargetWorkspaceContext, Guide?: PitchGuideSegment[],
  { MarginDeg = 0, StepDeg = .75, MaxChangeDeg = Infinity, Accept }: WristSearchOptions = {}): TargetPose | null {
  if (!Object.values(Requested).every(Number.isFinite) || BasketTargetReason(Requested)) return null;
  const PreviewAt = (Pitch: number) => solveIkPreview({ ...Requested, wristPitch: Pitch }, Context.ElbowMode, Context.Servos, Context.Limits);
  // Prefer the interior of a valid range, leaving room for encoder rounding.
  const Check = (Pitch: number) => {
    const Preview = PreviewAt(Pitch);
    return PreviewConverges(Preview, Pitch) && (!Accept || Accept({ ...Requested, wristPitch: Pitch }, Preview));
  };
  // With a margin (AI proposals) the requested pitch must converge too: it is
  // approved and sent later, against newer servo readings.
  if (MarginDeg > 0 ? Check(Requested.wristPitch) : PreviewAt(Requested.wristPitch).reachable) return { ...Requested };
  const Initial = currentAngles(Context.Servos);
  const Free = CadSolve(Requested, Requested.wristPitch, Context.ElbowMode, Initial, GetIkAngleLimits(Context.Servos, Context.Limits), true,
    Angles => BasketPoseReason(Angles) === null);
  const Samples = Guide ? Guide.flatMap(Segment => [Segment.Start.Pitch, Segment.End.Pitch, (Segment.Start.Pitch + Segment.End.Pitch) / 2])
    : Array.from({ length: Math.round(360 / StepDeg) + 1 }, (_, Index) => -180 + Index * StepDeg);
  // The inset may move a pitch up to MarginDeg + 0.1 farther out: sample that much inside the cap.
  const Pitches = [...new Set([CadPitch(Initial), ...(Free.Reachable ? [Free.PitchDegrees] : []), ...Samples]
    .map(Pitch => Requested.wristPitch + Wrap(Pitch - Requested.wristPitch)))]
    .filter(Pitch => Math.abs(Pitch - Requested.wristPitch) <= MaxChangeDeg - (MarginDeg > 0 ? MarginDeg + .1 : 0));
  Pitches.sort((A, B) => Math.abs(A - Requested.wristPitch) - Math.abs(B - Requested.wristPitch));
  for (const Pitch of Pitches) {
    if (!Check(Pitch)) continue;
    let Valid = Pitch, Invalid = Requested.wristPitch;
    for (let Step = 0; Step < 12 && Math.abs(Valid - Invalid) > .005; Step++) {
      const Middle = (Valid + Invalid) / 2;
      if (Check(Middle)) Valid = Middle; else Invalid = Middle;
    }
    return { ...Requested, wristPitch: MarginDeg > 0 ? InsetWristPitch(Valid, Requested.wristPitch, Check, MarginDeg) : Valid };
  }
  return null;
}

// The AI's search: strict, then inset (above). It runs on the page thread, so
// it samples every 1 deg and skips the search for a point out of reach at
// every pitch: never more IK solves than the old 1 deg sweep, usually far fewer.
export function FindInsetWristTarget(Requested: TargetPose, Context: TargetWorkspaceContext, MarginDeg = AiWristMarginDeg): TargetPose | null {
  if (BeyondArmReach(Requested)) return null;
  return FindReachableWristTarget(Requested, Context, undefined, { MarginDeg, StepDeg: 1 });
}
