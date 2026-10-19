import { CloseAndLiftJson, CloseTargetFraction, GripperOpenFraction } from "./AiMeasurements";
import { FootprintDistance, type Prism } from "./GraspPlan";
import type { Point3, TargetPose } from "./robotModel";

// CHAINED MOTION: one AI motion_proposal may chain ONE gripper action with ONE
// tip move, in an explicit order: "gripper_before" (the gripper moves first,
// then the tip) or "gripper_after" (the tip moves first, then the gripper).
// One approval runs both, one after the other: the second is sent only once
// the first has settled (a closing gripper stopped by an object counts: the
// bridge holds it, GRASP HOLD), and every way a task stops drops it. Sent
// alone, each of them cost a model round trip (10-25 s) and an approval.
// A chain counts as ONE motion of the task (MAX_AI_TASK_STEPS).
// What may not be chained (ChainRefusalReason) keeps apart the steps the
// GRASP PLAN and the guard rely on: the descent onto the object stays its own
// motion (the next image checks the open jaws beside it before the close), a
// close is chained only before a straight lift (never after a move, which
// would close the jaws wherever it ends), and the jaws never open or narrow
// beside the object below its top, where the moving jaw swings toward it.
// Every leg is checked with the gripper as it will be then (App.tsx
// validateAiMotionProposal): the tip move after a gripper_before with the
// jaws at the chained fraction, the gripper after a gripper_after at the
// move's end pose. The second leg is checked again before it is sent
// (App.tsx continueChain): the first must have got where it was sent with
// nothing else moving the arm meanwhile (Coast, Hold, Home, a slider or a
// stop end the task instead), and the second is checked from the live pose.
// A held object (GraspGuard.Held) is not the object the rules keep the jaws
// away from: putting it down and releasing it is checked like any other move.
// TOGETHER: a gripper_before that OPENS the jaws runs as ONE motion with its tip
// move, when the bridge takes a gripper target with a tip move
// (gripper_with_tip_version): its route opens the gripper while the arm moves.
// Sent as two legs, the still-settling gripper crept 3 steps while the tip move
// planned and the bridge refused it (2026-10-03 14:46: only the gripper opened).
// The tip move is checked as before, with the jaws open all the way there, and
// the moving jaw also at every opening along the way (App.tsx
// validateAiMotionProposal). A close or a narrowing stays two legs: closing
// while the tip rises would lift before the jaws hold the object, and a close
// stopped by an object mid-route would trip the bridge's stall protection.

export type ChainOrder = "before" | "after";
// The gripper action of a chained motion: its order and target (raw, and the 0..1 fraction, 1 open).
// Asked: the fraction the model asked for when a chained open went past GripperOpenFraction (ChainedFraction).
// Together: it opens the gripper before a tip move (ChainRunsTogether), so it may run as one motion with it.
export type ChainGripper = { Order: ChainOrder; TargetRaw: number; Fraction: number; Asked?: number | null; Together?: boolean };
// One leg of an approved chain, as it is sent (the tip's target in the model frame). "together": the
// tip move with the gripper target its route opens the jaws to on the way, one bridge motion.
export type ChainLeg = { Kind: "gripper"; TargetRaw: number; Fraction: number } | { Kind: "tip"; Target: TargetPose }
  | { Kind: "together"; Target: TargetPose; TargetRaw: number; Fraction: number };

export const ChainRules = {
  // After a close the tip only lifts: at least this far up, at most this far sideways, the wrist within this.
  LiftMinCm: 1,
  LiftSidewaysCm: 0.5,
  LiftWristDeg: 2,
  // A tip move rising less than this that ends within NearObjectCm of the task object's
  // footprint and under its top + AboveTopCm goes down beside it: the descent onto it.
  RiseCm: 0.5,
  NearObjectCm: 3,
  AboveTopCm: 1,
  // The second leg follows a tip move only once the tip stopped this close to that move's target:
  // every settled motion recorded 2026-09-29..10-01 stopped within 1.3 cm of its own (the arm sags
  // about 0.8 cm below it). Farther off, the first leg did not get there and the second is not sent.
  ArrivedCm: 1.5,
};

/**
 * The fraction a chained gripper action runs to: an open asked past
 * GripperOpenFraction goes to it. A wider open can stop short of its target
 * (a full open stopped at 83%, 2026-09-29 22:55), and the second leg would
 * wait for it until the motion timed out; the model is told (ChainGripperText).
 */
export const ChainedFraction = (Asked: number) => Math.min(Asked, GripperOpenFraction);

const Cm = (Value: number) => (Math.abs(Value) < 0.05 ? 0 : Value).toFixed(1);
const Percent = (Fraction: number) => `${Math.round(Fraction * 100)}%`;

/**
 * Reads gripper_before / gripper_after from a motion_proposal: a 0..1
 * fraction (1 open), or an object with target_fraction. Null when neither is
 * given; an Error the model is told when both are, or the value is no fraction.
 */
export function ReadChainGripper(Source: Record<string, unknown>): { Order: ChainOrder; Fraction: number } | { Error: string } | null {
  const Read = (Value: unknown): number | null => {
    if (typeof Value === "object" && Value !== null) {
      const Item = Value as Record<string, unknown>;
      return Read(Item.target_fraction ?? Item.fraction ?? Item.open_fraction);
    }
    if (typeof Value === "string" && !Value.trim()) return null;
    const Number_ = typeof Value === "number" || typeof Value === "string" ? Number(Value) : Number.NaN;
    return Number.isFinite(Number_) ? Number_ : null;
  };
  const Before = Source.gripper_before, After = Source.gripper_after;
  const Given = (Value: unknown) => Value !== undefined && Value !== null;
  if (Given(Before) && Given(After)) {
    return { Error: "A chained motion carries ONE gripper action: give gripper_before or gripper_after, not both." };
  }
  if (!Given(Before) && !Given(After)) return null;
  const Order: ChainOrder = Given(Before) ? "before" : "after";
  const Fraction = Read(Given(Before) ? Before : After);
  if (Fraction == null || Fraction < -0.001 || Fraction > 1.001) {
    return { Error: `gripper_${Order} must be a gripper fraction from 0.0 (closed) to 1.0 (open), e.g. "gripper_${Order}":${GripperOpenFraction}.` };
  }
  return { Order, Fraction: Math.min(1, Math.max(0, Fraction)) };
}

/** How a gripper action counts: widening opens it; a move to CloseTargetFraction or less closes it; less than that narrows it. */
export function ChainGripperKind(From: number, To: number): "open" | "narrow" | "close" {
  if (To > From) return "open";
  return To <= CloseTargetFraction ? "close" : "narrow";
}

/** Whether a chained gripper action may run as one motion with its tip move: a gripper_before that opens the jaws. */
export function ChainRunsTogether(Order: ChainOrder, From: number, To: number) {
  return Order === "before" && ChainGripperKind(From, To) === "open";
}

/**
 * The gripper action in words: "open the gripper to 80%", "close the gripper", "set the gripper to 60% open".
 * Asked: the wider open the model asked for, which a chain runs to GripperOpenFraction (ChainedFraction).
 */
export function ChainGripperText(From: number, To: number, Asked: number | null = null) {
  const Kind = ChainGripperKind(From, To);
  const Capped = Asked != null && Asked > To ? ` (${Percent(Asked)} was asked for; a chained open goes to at most ${Percent(GripperOpenFraction)})` : "";
  return (Kind === "open" ? `open the gripper to ${Percent(To)}` : Kind === "close" ? "close the gripper" : `set the gripper to ${Percent(To)} open`) + Capped;
}

/**
 * The two legs in their order: "first open the gripper to 80%, then move the tip to ...";
 * run together (ChainLeg "together"): "open the gripper to 80% while moving the tip to ...".
 */
export function ChainOrderText(Order: ChainOrder, Gripper: string, Tip: string, Together = false) {
  if (Together) return `${Gripper} while ${Tip.replace(/^move /, "moving ")}`;
  return Order === "before" ? `first ${Gripper}, then ${Tip}` : `first ${Tip}, then ${Gripper}`;
}

export type ChainShape = {
  Order: ChainOrder;
  // The gripper's commanded fraction now, and the chained one.
  FromFraction: number;
  ToFraction: number;
  // The tip where the move starts and where it ends (AI display frame): the target sent, and the
  // one the model asked for when the app's snap moved it. Both must pass.
  Start: Point3;
  End: Point3;
  Requested?: Point3 | null;
  // How far the model asked the wrist to turn (the snap may turn it to reach the point, as for any lift).
  WristTurnDeg: number;
  // The task object's prism (GraspGuard.TaskPrism) and its label, when the task has one and the
  // jaws do not hold it (GraspGuard.Held): a held object is put down and released like any move.
  Task: ChainTask | null;
};
export type ChainTask = { Prism: Prism; Label: string };

/**
 * How far a tip position (AI display frame) is from the task object's
 * footprint when it is low beside it (within NearObjectCm, below its top +
 * AboveTopCm), where opening or narrowing the jaws swings the moving jaw
 * toward it; null when it is not.
 */
export function ChainLowBeside(Task: ChainTask, Point: Point3): number | null {
  const Near = FootprintDistance(Task.Prism.Footprint, Point);
  return Near <= ChainRules.NearObjectCm && Point.z < Task.Prism.TopZ + ChainRules.AboveTopCm ? Near : null;
}

/**
 * Why a chained gripper action may not run where the tip stopped (AI display
 * frame), the second leg's check before it is sent: opening or narrowing low
 * beside the task object. Null: it may run there.
 */
export function ChainGripperHereReason(FromFraction: number, ToFraction: number, Tip: Point3, Task: ChainTask | null): string | null {
  const Kind = ChainGripperKind(FromFraction, ToFraction);
  const Near = Kind === "close" || !Task ? null : ChainLowBeside(Task, Tip);
  if (Near == null) return null;
  return `the tip stopped beside the ${Task!.Label} below its top (${Cm(Near)} cm from it), where the moving jaw would swing toward it as the gripper ${Kind === "open" ? "opens" : "narrows"}`;
}

/**
 * Why a chained motion may not run as one, or null. The checks of each leg
 * (IK, floor, guard, the gripper's swing) are the validator's; these are the
 * rules about which two steps may follow each other without a model call.
 */
export function ChainRefusalReason(Shape: ChainShape): string | null {
  const Ends = [Shape.End, ...(Shape.Requested ? [Shape.Requested] : [])];
  for (const End of Ends) {
    const Reason = ChainShapeReason({ ...Shape, End });
    if (Reason) return Reason;
  }
  return null;
}

function ChainShapeReason(Shape: ChainShape): string | null {
  const Kind = ChainGripperKind(Shape.FromFraction, Shape.ToFraction);
  const Rise = Shape.End.z - Shape.Start.z;
  const Sideways = Math.hypot(Shape.End.x - Shape.Start.x, Shape.End.y - Shape.Start.y);
  const LiftJson = CloseAndLiftJson(6);
  // A wider open can stop short of its target (a full open stopped at 83%, 2026-09-29 22:55),
  // and the second leg would wait for it until the motion timed out.
  if (Shape.ToFraction > GripperOpenFraction + 0.005) {
    return `A chained gripper action may open the gripper to at most ${GripperOpenFraction} (gripper_${Shape.Order} ${Shape.ToFraction} was proposed): a wider open can stop short of its target, and the tip move would wait for it until the motion timed out. Use ${GripperOpenFraction}.`;
  }
  if (Kind === "close") {
    if (Shape.Order === "after") {
      return `A close is never chained after a tip move: the jaws would close wherever the move ends, before anyone has seen where that is. Propose the move alone and close the gripper at the next check-in; that close may be chained with the straight lift after it: ${LiftJson}.`;
    }
    if (Rise < ChainRules.LiftMinCm || Sideways > ChainRules.LiftSidewaysCm || Shape.WristTurnDeg > ChainRules.LiftWristDeg) {
      return `After a close the only tip move that may be chained is a straight lift (at least ${ChainRules.LiftMinCm} cm up, at most ${ChainRules.LiftSidewaysCm} cm sideways, the wrist unchanged); this one goes ${Cm(Rise)} cm up and ${Cm(Sideways)} cm sideways and turns the wrist ${Cm(Shape.WristTurnDeg)} deg. Any other move right after a close would drag the closing jaws, or what they hold, into what is around them. Close the gripper alone, or chain it with the lift: ${LiftJson}.`;
    }
    return null;
  }
  const Task = Shape.Task;
  if (!Task) return null;
  const Near = (Point: Point3) => FootprintDistance(Task.Prism.Footprint, Point);
  const Low = (Point: Point3) => ChainLowBeside(Task, Point) != null;
  if (Rise < ChainRules.RiseCm && Low(Shape.End)) {
    return `That tip move goes down beside the ${Task.Label} (it ends ${Cm(Near(Shape.End))} cm from it, below its top), which is the descent onto it: the descent is never chained with the gripper. Propose it alone, with the gripper already as the GRASP PLAN needs it, so the next image shows the open jaws beside the ${Task.Label} before they close.`;
  }
  const At = Shape.Order === "before" ? Shape.Start : Shape.End;
  if (Low(At)) {
    const Verb = Kind === "open" ? "open" : "narrow";
    return Shape.Order === "before"
      ? `The gripper would ${Verb} beside the ${Task.Label} below its top (the tip is ${Cm(Near(At))} cm from it), where the moving jaw swings toward it: a chained gripper action runs only above or away from it. Rise straight up alone first; the gripper may then be chained with the move from there.`
      : `The gripper would ${Verb} where that move ends, beside the ${Task.Label} below its top (${Cm(Near(At))} cm from it), where the moving jaw swings toward it: a chained gripper action runs only above or away from it. Propose the move alone.`;
  }
  return null;
}

// How each leg of an approved chain went: sent, skipped (it would not have moved anything), refused by
// the bridge, or not sent (a check against the arm as the first leg left it failed, or the send failed).
export type ChainLegResult = "sent" | "skipped" | "refused" | "not-sent";
// Together: it ran as one motion (one "together" leg, its result the first; the second stays null).
export type ChainRun = { Order: ChainOrder; Gripper: string; Tip: string; Results: [ChainLegResult | null, ChainLegResult | null]; Why: string | null;
  Together?: boolean };

/** What an approved chain did, for the check-in after it: which leg ran, which did not and why. */
export function DescribeChainRun(Run: ChainRun) {
  const Legs = Run.Order === "before" ? [Run.Gripper, Run.Tip] : [Run.Tip, Run.Gripper];
  const [First, Second] = Run.Results;
  const Why = Run.Why ? `: ${Run.Why.replace(/\.$/, "")}` : "";
  if (Run.Together) {
    const Head = `It was a chained motion, run as ONE motion: ${ChainOrderText(Run.Order, Run.Gripper, Run.Tip, true)}`;
    if (First === "refused" || First === "not-sent") return `${Head}; it did not run: it was ${First === "refused" ? "refused by the bridge" : "not sent"}${Why}.`;
    return `${Head}; the gripper opened on the way.`;
  }
  const Head = `It was a chained motion: ${ChainOrderText(Run.Order, Run.Gripper, Run.Tip)}`;
  if (First === "refused" || First === "not-sent") return `${Head}; neither part ran: the first (${Legs[0]}) was ${First === "refused" ? "refused by the bridge" : "not sent"}${Why}.`;
  if (Second === "refused" || Second === "not-sent") {
    return `${Head}; only the first part ran (${Legs[0]})${First === "skipped" ? ", and it had nothing to do" : ""}: the second (${Legs[1]}) was ${Second === "refused" ? "refused by the bridge" : "not sent"}${Why}.`;
  }
  if (First === "skipped") return `${Head}; the first part had nothing to do (${Run.Order === "before" ? "the gripper was already there" : "the tip was already there"}), and the second ran.`;
  if (Second === "skipped") return `${Head}; the first part ran, and the second had nothing to do (${Run.Order === "before" ? "the tip was already there" : "the gripper was already there"}).`;
  return `${Head}; both parts ran, in that order.`;
}
