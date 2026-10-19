import { AiJawFloorAllows, AiJawFloorReason, AiTableFloorReason, DeskFloor, FloorZAt, GripperFloorGap, GripperFloorGapOfLinks, OverBasket,
  type AiFloor } from "./AiTableFloor";
import { AiWristMarginDeg, BeyondArmReach, FindReachableWristTarget, PreviewConverges } from "./AutomaticWrist";
import { BasketBounds, BasketTargetReason } from "./BasketCollision";
import { CadForward } from "./CadKinematics";
import { UnreliableTargetReason, type LocatedObject } from "./AiMeasurements";
import { GraspGuardReason, GuardPoseCheck, type GraspGuard } from "./GraspPlan";
import { FindNearestLocalTarget, InsetIkLimits, type NearestOptions } from "./TargetWorkspace";
import { currentAngles, GetIkAngleLimits, IK_JOINTS, mirrorDisplayPoint, modelPointFromDisplayPoint, solveIkPreview, type IkAngleLimits,
  type Point3, type ServoName, type ServoState, type TargetPose } from "./robotModel";

// AI TARGET SNAP: an AI target the arm cannot reach exactly moves to the
// nearest one it can, the way the manual GUI snaps a dragged target: first the
// wrist pitch alone (the XYZ kept), then the tip by a few cm. Unlike the GUI
// it is capped, never lowers the tip or crosses the basket rim, and refuses
// what the safety rules refuse instead of moving it: a target below the floor,
// a descent onto an [UNRELIABLE] object or into basket material.
// Every snapped target converges in the IK strictly (0.02 cm, 0.08 deg) on
// joint limits pulled in by LimitInsetDeg, so Approve's later check with newer
// servo readings and the bridge's own solve accept it (0.1 cm, 0.25 deg).
// Pure: the same input gives the same result at proposal and at approval. The
// page runs the refusals and the exact check (ExactOnly, an IK solve or two);
// the searches run in TargetWorkspaceWorker (Kind "AiSnap").

// Off: the app refuses every AI target it cannot reach exactly and moves none.
export const AiSnapEnabled = true;
export const SnapRules = {
  // How far the tip may move from the requested point (3D), cm.
  MaxCm: 3.0,
  // The wrist pitch the model gives (or the current one, left out or "auto") is a preference: at the
  // requested point the wrist alone turns to the nearest pitch the arm holds there (reached, clear of
  // the floor, the basket and the guard's objects), however far that is. 2026-10-03: "it says it is too
  // far when it definitely can ... TRIES to get as close to the wrist angle HOWEVER it is not constrained".
  WristOnlyMaxChangeDeg: 180,
  // How far the wrist may turn from the requested pitch where the tip moves too (the position search), deg.
  MaxPitchChangeDeg: 45,
  // How much lower than requested the tip may end, cm.
  MaxDropCm: 0.5,
  // A descent (the request 1 cm or more below the tip) moves at most this far sideways, cm.
  DescentMaxXYCm: 1.0,
  // Joint limits are pulled in this far for the snapped pose, deg.
  LimitInsetDeg: 0.5,
  // Pose evaluations the position search may spend (deterministic, not wall time).
  MaxEvaluations: 60000,
  // Between tip positions about as near, the smaller wrist change wins: 1 deg counts as this many cm.
  PitchCostCmPerDeg: 0.01,
  // A moved tip keeps this much more clearance from the basket and body than
  // the collision model asks, so the bridge's check of the rounded pose passes, cm.
  CollisionMarginCm: 0.2,
};
export type AiSnapRules = typeof SnapRules;

export type SnapInput = {
  // What the model asked for, and the pose it was planned from (model frame).
  Requested: TargetPose;
  Anchor: TargetPose;
  // The servo snapshot the proposal was made with: the IK seed and slider limits.
  Servos: Record<ServoName, ServoState>;
  ElbowMode: "up" | "down";
  // The joint limits the app sends with (the bridge's; undefined for the sliders').
  Limits?: IkAngleLimits | null;
  // Both X and Y came from target_cm (the [UNRELIABLE] rule applies).
  AbsoluteXY: boolean;
  // This frame's [UNRELIABLE] measurements (AI display frame).
  Unreliable: LocatedObject[];
  // An object task's measured objects and GRASP PLAN (GraspPlan.ts): a target
  // the guard refuses is refused, never moved, and a moved one keeps clear of them.
  Grasp?: GraspGuard | null;
};
export type SnapReason = {
  Code: "pitch-geometry" | "joint-limit" | "collision" | "reach" | "jaw-floor" | "window";
  // In the AI's words and display frame: "at wrist -90 deg the arm is 2.6 cm too short to reach that point".
  Text: string;
};
// A target the app found but did not use, with why (for the model's next try).
export type SnapNearest = { Target: TargetPose; DistanceCm: number; PitchChangeDeg: number; Why: string };
export type SnapTimings = { PitchMs: number; PositionMs: number };
export type SnapResult =
  | { Kind: "exact" | "pitch" | "position"; Target: TargetPose; Requested: TargetPose; DistanceCm: number; PitchChangeDeg: number;
      Reason: SnapReason | null; Evaluations: number; Ms: SnapTimings }
  | { Kind: "refused"; Code: "floor" | "unreliable" | "grasp" | "blocked" | "too-far" | "no-solution" | "post-check" | "timeout";
      Text: string; Requested: TargetPose; Nearest: SnapNearest | null; Reason: SnapReason | null; Evaluations: number; Ms: SnapTimings };

const Wrap = (Angle: number) => ((Angle + 180) % 360 + 360) % 360 - 180;
const Distance = (A: Point3, B: Point3) => Math.hypot(A.x - B.x, A.y - B.y, A.z - B.z);
const Number1 = (Value: number) => Number(Value.toFixed(1)).toString();
const Number2 = (Value: number) => Number(Value.toFixed(2)).toString();
const Now = () => (typeof performance !== "undefined" ? performance.now() : Date.now());

export const SnapTimeoutText = "The app's search for the nearest reachable target did not finish, so this target was checked only as it is.";

/**
 * The snap of one AI target (model frame). See the rules above. ExactOnly:
 * the refusals and the exact check alone (one IK solve), for the page when
 * the worker's search failed; anything else is refused "timeout".
 */
export function SnapAiTarget(Input: SnapInput, Rules: AiSnapRules = SnapRules, ExactOnly = false): SnapResult {
  const Requested = { ...Input.Requested };
  const Ms: SnapTimings = { PitchMs: 0, PositionMs: 0 };
  let Evaluations = 0;
  const Refuse = (Code: Extract<SnapResult, { Kind: "refused" }>["Code"], Text: string,
    Nearest: SnapNearest | null = null, Reason: SnapReason | null = null): SnapResult =>
    ({ Kind: "refused", Code, Text, Requested, Nearest, Reason, Evaluations, Ms });
  // Refused, never moved: a target below the floor or onto an [UNRELIABLE]
  // object means the model misread a height, and snapping it up would hide that.
  if (!Object.values(Requested).every(Number.isFinite)) return Refuse("no-solution", "Target coordinates must be finite numbers.");
  // The task's floor: the surface its object stands on (the desk's without one, AiTableFloor.ts).
  const TaskFloor = FloorOf(Input);
  const Floor = AiTableFloorReason(Requested, Requested.z - Input.Anchor.z, TaskFloor);
  if (Floor) return Refuse("floor", Floor);
  const Unreliable = UnreliableReason(Requested, Input);
  if (Unreliable) return Refuse("unreliable", Unreliable);
  const Blocked = BasketTargetReason(Requested);
  if (Blocked) return Refuse("blocked", `${Blocked}.`);

  const Limits = GetIkAngleLimits(Input.Servos, Input.Limits ?? undefined);
  const Inset = InsetIkLimits(Limits, Rules.LimitInsetDeg);
  const CurrentAngles = currentAngles(Input.Servos);
  const CurrentGap = GripperFloorGap(CurrentAngles, TaskFloor);
  const FloorAllows = (Target: TargetPose, Links: ReturnType<typeof CadForward>["Links"]) =>
    AiJawFloorAllows(Target, GripperFloorGapOfLinks(Links, TaskFloor), CurrentGap);
  // The searches also hold every pose they try to the grasp guard's cheap pose
  // rules; the request itself passed the full guard below before any search.
  const GuardPose = GuardPoseCheck(Input.Grasp ?? null, CurrentAngles);
  const JawAllows = (Target: TargetPose, Links: ReturnType<typeof CadForward>["Links"]) =>
    FloorAllows(Target, Links) && (!GuardPose || GuardPose(Target, Links));
  const Finish = (Kind: "exact" | "pitch" | "position", Target: TargetPose, Reason: SnapReason | null): SnapResult => {
    const Problem = PostCheck(Target, Input, Limits);
    if (Problem) return Refuse("post-check", `The app's nearest target failed its own check: ${Problem}`, null, Reason);
    return { Kind, Target, Requested, DistanceCm: Distance(Target, Requested), PitchChangeDeg: Math.abs(Wrap(Target.wristPitch - Requested.wristPitch)),
      Reason, Evaluations, Ms };
  };

  // Exact: the IK converges on the request as it is, clear of the floor.
  const Exact = solveIkPreview(Requested, Input.ElbowMode, Input.Servos, Inset);
  // Refused, never moved: a request that brings the gripper into a measured
  // object (above all the fixed jaw onto the task object) at the IK's closest
  // pose. Moving it a few cm would "rescue" exactly the descent it guards against.
  const Grasped = Input.Grasp ? GraspGuardReason(Requested, Exact.angles, CurrentAngles, Requested.z - Input.Anchor.z, Input.Grasp) : null;
  if (Grasped) return Refuse("grasp", Grasped);
  if (PreviewConverges(Exact, Requested.wristPitch) && FloorAllows(Requested, CadForward(Exact.angles).Links)) return Finish("exact", Requested, null);
  if (ExactOnly) return Refuse("timeout", SnapTimeoutText);

  // The wrist only: the nearest pitch that works at the requested point (WristOnlyMaxChangeDeg), inset from its edge.
  const PitchStarted = Now();
  const PitchOnly = (MaxChangeDeg: number, StepDeg = 1) => BeyondArmReach(Requested) ? null : FindReachableWristTarget(Requested,
    { Servos: Input.Servos, ElbowMode: Input.ElbowMode, Limits: Inset }, undefined,
    { MarginDeg: AiWristMarginDeg, StepDeg, MaxChangeDeg, Accept: (Target, Preview) => JawAllows(Target, CadForward(Preview.angles).Links) });
  // Every deg within MaxPitchChangeDeg of the request, then every 2 to WristOnlyMaxChangeDeg (the edge found by bisection either way).
  const Pitch = PitchOnly(Math.min(Rules.MaxPitchChangeDeg, Rules.WristOnlyMaxChangeDeg))
    ?? (Rules.WristOnlyMaxChangeDeg > Rules.MaxPitchChangeDeg ? PitchOnly(Rules.WristOnlyMaxChangeDeg, 2) : null);
  Ms.PitchMs = Now() - PitchStarted;
  if (Pitch) return Finish("pitch", Pitch, ClassifyRequest(Input, Limits, Inset, Rules, false));

  // The tip: the nearest target within MaxCm, never lower than MaxDropCm
  // under the request or the floor, on the request's side of the basket rim.
  const PositionStarted = Now();
  const Over = OverBasket(Requested);
  const FloorZ = modelPointFromDisplayPoint({ x: 0, y: 0, z: FloorZAt(TaskFloor, mirrorDisplayPoint(Requested)) }).z;
  const Descent = Requested.z < Input.Anchor.z - 1;
  const Options: NearestOptions = {
    MaxCm: Rules.MaxCm, LimitInsetDeg: Rules.LimitInsetDeg, MaxEvaluations: Rules.MaxEvaluations, PitchCostCmPerDeg: Rules.PitchCostCmPerDeg,
    CollisionMarginCm: Rules.CollisionMarginCm,
    // A request already under the floor (allowed only while rising) may not drop at all.
    MinZ: Math.max(Requested.z - Rules.MaxDropCm, Over ? -Infinity : Math.min(FloorZ, Requested.z)),
    // The rim height is the cart's top: the tray's rim, and exactly the display
    // frame's Z 0. TrayBounds' top is 3e-6 cm higher (float rounding), which
    // would put a target at Z 0 under the rim.
    SameBasketSide: true, RimZ: BasketBounds[1].z,
    PitchWindowDeg: Rules.MaxPitchChangeDeg,
    MaxXYCm: Descent ? Rules.DescentMaxXYCm : null,
    PoseAllowed: (_Angles, Target, Links) => JawAllows(Target, Links) && !UnreliableReason(Target, Input),
  };
  const Context = { Servos: Input.Servos, ElbowMode: Input.ElbowMode, Limits: Input.Limits ?? undefined };
  // The wrist a preference (WristOnlyMaxChangeDeg the whole turn): the tip searched around the pitch whose pose
  // comes nearest the point clear of contact (every 15 deg), not only around the one asked for, but only where
  // turning the wrist shortens the tip's move by more than it costs (NearestPosePitch).
  // Near a level wrist most poses are valid and the search refines long: it stops at a third of the budget there.
  const AroundPitch = Rules.WristOnlyMaxChangeDeg >= 180 ? NearestPosePitch(Requested, Input, Rules) : Requested.wristPitch;
  const Around = AroundPitch !== Requested.wristPitch ? { ...Requested, wristPitch: AroundPitch } : Requested;
  const Near = FindNearestLocalTarget(Around, Context, Around === Requested ? Options : { ...Options, MaxEvaluations: Math.round(Rules.MaxEvaluations / 3) });
  Evaluations += Near.Evaluations;
  const Reason = ClassifyRequest(Input, Limits, Inset, Rules, true);
  if (Near.Target && Near.DistanceCm <= Rules.MaxCm) {
    Ms.PositionMs = Now() - PositionStarted;
    return Finish("position", Near.Target, Reason);
  }
  // Nothing allowed within MaxCm. The nearest target found goes back to the
  // model with why it was not used: the same point with any wrist pitch
  // first, else the nearest one with a freer wrist and no sideways cap.
  let Found = (Rules.WristOnlyMaxChangeDeg < 180 ? PitchOnly(180) : null) ?? Near.Target;
  if (Found === Near.Target && Rules.MaxEvaluations - Evaluations > 0) {
    const Wide = FindNearestLocalTarget(Requested, Context, { ...Options, PitchWindowDeg: 90, SeedPitchStepDeg: 10, MaxXYCm: null,
      SearchCm: Rules.MaxCm + 4, MaxEvaluations: Rules.MaxEvaluations - Evaluations });
    Evaluations += Wide.Evaluations;
    if (Wide.Target && (!Found || Distance(Wide.Target, Requested) < Distance(Found, Requested) - 1e-9)) Found = Wide.Target;
  }
  Ms.PositionMs = Now() - PositionStarted;
  const Nearest = Found && !PostCheck(Found, Input, Limits) ? DescribeNearest(Found, Requested, Descent, Rules) : null;
  return Nearest
    ? Refuse("too-far", `The nearest target the arm can reach is ${Number2(Nearest.DistanceCm)} cm from yours and was not used: ${Nearest.Why}.`, Nearest, Reason)
    : Refuse("no-solution", `The app found no reachable target within ${Number1(Rules.MaxCm + 2)} cm of yours that keeps the tip above the floor and clear of the basket.`, null, Reason);
}

/**
 * The pitch (every 15 deg around the request's) whose IK pose misses the point
 * least without contact, each degree of wrist turn counted as the position
 * search counts it (PitchCostCmPerDeg): the wrist turns only where that
 * shortens the tip's move by more, and on a tie it stays as asked. A move the
 * wrist cannot help (a joint limit that holds the tip off the point at every
 * pitch) keeps the requested wrist: 2026-10-03 TestAiTaskSafetyUi, the pan
 * limit narrowed 0.32 deg under a move_delta_cm up 2 at wrist -6.6, the misses
 * equal to 1e-5 cm across -66.6..-6.6, and the re-snap turned the wrist 30 deg
 * to -36.6 for a float-noise gain, so it asked for approval again.
 */
function NearestPosePitch(Requested: TargetPose, Input: SnapInput, Rules: AiSnapRules): number {
  let Best = { Pitch: Requested.wristPitch, Cost: Infinity };
  for (let Offset = 0; Offset <= 180; Offset += 15) {
    for (const Pitch of Offset && Offset < 180 ? [Requested.wristPitch - Offset, Requested.wristPitch + Offset] : [Requested.wristPitch + Offset]) {
      const Preview = solveIkPreview({ ...Requested, wristPitch: Pitch }, Input.ElbowMode, Input.Servos, Input.Limits ?? undefined);
      if (Preview.collisionReason) continue;
      const Cost = (Number.isFinite(Preview.errorCm) ? Preview.errorCm : Infinity) + Offset * Rules.PitchCostCmPerDeg;
      if (Cost < Best.Cost - 1e-6) Best = { Pitch, Cost };
    }
  }
  return Best.Pitch;
}
function UnreliableReason(Target: Point3, Input: SnapInput) {
  return Input.Unreliable.map(Object_ => UnreliableTargetReason(Target, Input.AbsoluteXY, Object_)).find(Boolean) ?? null;
}
/** The floor a snap holds targets to: the task's (its guard's), the desk's without one. */
const FloorOf = (Input: SnapInput): AiFloor => Input.Grasp?.Floor ?? DeskFloor;

// The checks the page repeats (validateAiMotionProposal) on the joint limits
// the app sends with. A failure here is a bug, not a refusal the model caused.
function PostCheck(Target: TargetPose, Input: SnapInput, Limits: IkAngleLimits): string | null {
  const Preview = solveIkPreview(Target, Input.ElbowMode, Input.Servos, Limits);
  if (!Preview.reachable) return Preview.collisionReason ?? "the IK does not reach it";
  return AiTableFloorReason(Target, Target.z - Input.Anchor.z, FloorOf(Input))
    ?? AiJawFloorReason(Target, Preview.angles, currentAngles(Input.Servos), FloorOf(Input))
    ?? UnreliableReason(Target, Input)
    ?? (OverBasket(Target) !== OverBasket(Input.Requested) ? "it is on the other side of the basket rim" : null)
    ?? GraspGuardReason(Target, Preview.angles, currentAngles(Input.Servos), Target.z - Input.Anchor.z, Input.Grasp ?? null);
}

function DescribeNearest(Target: TargetPose, Requested: TargetPose, Descent: boolean, Rules: AiSnapRules): SnapNearest {
  const DistanceCm = Distance(Target, Requested), PitchChangeDeg = Math.abs(Wrap(Target.wristPitch - Requested.wristPitch));
  const Sideways = Math.hypot(Target.x - Requested.x, Target.y - Requested.y);
  const Why = [
    ...(DistanceCm > Rules.MaxCm ? [`it is more than ${Number1(Rules.MaxCm)} cm away (the app moves a target at most ${Number1(Rules.MaxCm)} cm)`] : []),
    ...(PitchChangeDeg > Rules.MaxPitchChangeDeg && Rules.WristOnlyMaxChangeDeg < 180 ? [`the wrist would change by ${Number1(PitchChangeDeg)} deg (the app changes it by at most ${Number1(Rules.MaxPitchChangeDeg)} deg)`] : []),
    ...(Descent && Sideways > Rules.DescentMaxXYCm ? [`it is ${Number2(Sideways)} cm to the side (on a descent the app moves the tip at most ${Number1(Rules.DescentMaxXYCm)} cm sideways)`] : []),
  ];
  return { Target, DistanceCm, PitchChangeDeg, Why: Why.join("; ") || "it failed the app's checks" };
}

/**
 * Why the request itself is not reachable as asked, in the AI's words:
 * from one explained solve at the requested pitch, and (when the wrist alone
 * could not fix it) how the other pitches in the window fail.
 */
function ClassifyRequest(Input: SnapInput, Limits: IkAngleLimits, Inset: IkAngleLimits, Rules: AiSnapRules, WindowFailed: boolean): SnapReason {
  const Requested = Input.Requested;
  const Wrist = `at wrist ${Number1(Requested.wristPitch)} deg`;
  const Preview = solveIkPreview(Requested, Input.ElbowMode, Input.Servos, Limits, true);
  let Reason: SnapReason;
  if (Preview.reachable) {
    const Near = IK_JOINTS.filter(Name => {
      const Range = Inset[Name], Angle = Preview.angles[Name];
      return Range && (Angle < Range[0] || Angle > Range[1]);
    });
    Reason = AiJawFloorReason(Requested, Preview.angles, currentAngles(Input.Servos), FloorOf(Input))
      ? { Code: "jaw-floor", Text: `${Wrist} the lowest point of the gripper would be below ${FloorOf(Input).Surface ? "the floor" : "the table floor"}` }
      : !PreviewConverges(Preview, Requested.wristPitch)
        ? { Code: "reach", Text: `${Wrist} that point is at the very edge of the arm's reach, where the solve is not reliable` }
        : { Code: "joint-limit", Text: `${Wrist} the ${Near.map(Name => Name.replace(/_/g, " ")).join(" and ") || "arm"} would be within ${Number1(Rules.LimitInsetDeg)} deg of its limit` };
  } else if (Preview.collisionReason) {
    Reason = { Code: "collision", Text: `${Wrist} the ${Preview.collisionReason}` };
  } else if (Preview.failure?.Kind === "reach") {
    Reason = { Code: "pitch-geometry", Text: Preview.failure.TooFar
      ? `${Wrist} the arm is ${Number2(Preview.failure.ErrorCm)} cm too short to reach that point`
      : `${Wrist} that point is too close to the shoulder` };
  } else if (Preview.failure?.Kind === "joint-limit") {
    Reason = { Code: "joint-limit", Text: `${Wrist} ${Preview.failure.Joints.map(({ Name, Angle, Range }) =>
      `the ${Name.replace(/_/g, " ")} would need ${Angle.toFixed(1)} deg, its range is ${Range[0].toFixed(1)}..${Range[1].toFixed(1)} deg`).join(" and ")}` };
  } else if (Preview.failure?.Kind === "collision") {
    Reason = { Code: "collision", Text: `${Wrist} only a pose beyond the joint limits reaches it, and there the ${Preview.failure.Reason}` };
  } else {
    Reason = { Code: "reach", Text: `${Wrist} the IK finds no pose for it` };
  }
  if (!WindowFailed) return Reason;
  // The pitches across the window, every 15 deg: a collision among the ones that reach it is the usual cause.
  const Window = Math.min(Rules.WristOnlyMaxChangeDeg, 180);
  let Collision: string | null = null;
  for (let Offset = -Window; Offset <= Window + 1e-9 && !Collision; Offset += 15) {
    if (Offset === 0) continue;
    Collision = solveIkPreview({ ...Requested, wristPitch: Requested.wristPitch + Offset }, Input.ElbowMode, Input.Servos, Limits).collisionReason ?? null;
  }
  return { Code: Reason.Code, Text: `${Reason.Text}; ${Window >= 180 ? "no wrist angle" : `no wrist angle within ${Number1(Window)} deg of yours`} reaches it`
    + (Collision ? ` without contact (where one reaches it, the ${Collision})` : "") };
}
