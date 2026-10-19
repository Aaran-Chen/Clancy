import { ReachAt, ReachPitches } from "./AiMeasurements";
import type { ChainLeg } from "./AiChain";
import type { SnapResult } from "./AiTargetSnap";
import { mirrorDisplayPoint, rawToFraction, type IkAngleLimits, type Point3, type ServoName, type ServoState, type TargetPose } from "./robotModel";

// TASK MODE loop: after an approved motion settles, the AI is automatically
// re-prompted with a fresh capture until it reports done/ask or hits this cap.
export const MAX_AI_TASK_STEPS = 8;
// A check-in waits for the bridge to START the approved motion (its motion
// epoch advances), for fresh state after it stops, and then this long from
// the start. Route planning can take several seconds before anything moves.
// The epoch and the bridge's route flag already prove the motion ran and
// ended (every motion is a route, started with its epoch; it ends only with
// every joint at its goal), and the check-in itself waits for the arm to hold
// still (ArmStillHoldMs) before it captures. So this is only a floor: it was
// 1500 ms from the August app, before either existed, and held every short
// move (a 4.4 cm descent flies in 0.8 s) back about 0.7 s before its check-in.
export const TaskSettleMinMs = 300;
export const TaskSettleTimeoutMs = 12000;
export const TaskStartLimitMs = 25000;
// State older than this is from before the websocket went quiet (the bridge
// sends none to this client while it plans), so it cannot show arrival.
export const TaskStateFreshMs = 700;
// A proposal this close to where the arm already is does nothing.
export const NoOpCommandedCm = 0.5;
export const NoOpLiveCm = 1.0;
export const NoOpDeg = 2;
const NoOpServoRaw = 23; // about 2 deg on a 4096-step servo

export type AiChatMessage = { role: "user" | "assistant"; content: string; turn?: number };
// A CHAINED MOTION (AiChain.ts) waits for each leg in turn. Part: the leg being waited on (1 or 2);
// Next: the leg still to send once this one has settled; CheckIn: whether a check-in follows the last
// leg (false for the task's last allowed motion, or a chain with no task); EndsTask: the task's last
// allowed motion, so the task ends once the last leg is sent. Every task end clears the wait, so a stop
// between the legs drops the second one. First: the leg being waited on; StartRaw: every servo's present
// raw when the chain was approved (a gripper leg must leave the arm joints there).
export type TaskSettleChain = { Part: 1 | 2; Next: ChainLeg | null; CheckIn: boolean; EndsTask: boolean;
  First?: ChainLeg | null; StartRaw?: Partial<Record<ServoName, number>> | null };
// StartedEpoch: the bridge's motion epoch when the motion was seen to start. A bridge motion
// starts or stops only with a new epoch, so a later one while a chain's first leg runs means
// something else moved or stopped the arm (Coast, Hold, Home, a slider, a stop).
// SettledAt: when a chain's gripper leg first read settled, while its tip move waits for the
// jaws to hold still (ChainGripperStill).
export type TaskSettleWait = { approvedAt: number; epoch: number; startedAt: number | null; StartedEpoch?: number | null;
  Chain?: TaskSettleChain | null; SettledAt?: number | null };
export type TaskSettleBridge = { Epoch: number; StateAgeMs: number; RouteActive: boolean; Moving: boolean;
  WithinTolerance: boolean; MotionTimeoutMs: number };
export type TaskSettleVerdict = "waiting" | "started" | "settled" | "not-started" | "timeout";

export function FormatAiPromptNumber(Value: number) {
  return Number(Value.toFixed(4)).toString();
}
const Cm = (Value: number) => Value.toFixed(1);
const Distance = (A: Point3, B: Point3) => Math.hypot(A.x - B.x, A.y - B.y, A.z - B.z);
const AngleDelta = (A: number, B: number) => Math.abs((((A - B + 180) % 360) + 360) % 360 - 180);
const RawDelta = (A: number, B: number) => { const D = (((A - B) % 4096) + 4096) % 4096; return Math.min(D, 4096 - D); };

/**
 * Where the post-approval wait stands. Frozen state from before the motion
 * (the bridge sends none while it plans) must never read as arrival, so the
 * motion only counts as started once its epoch passes the one recorded when
 * the command was sent, and as settled only on fresh, still state.
 */
export function TaskSettleVerdictFor(Wait: TaskSettleWait, Now: number, Bridge: TaskSettleBridge): TaskSettleVerdict {
  if (Wait.startedAt === null) {
    if (Bridge.Epoch > Wait.epoch) return "started";
    return Now - Wait.approvedAt >= TaskStartLimitMs ? "not-started" : "waiting";
  }
  const Elapsed = Now - Wait.startedAt;
  if (Elapsed >= TaskSettleMinMs && Bridge.StateAgeMs < TaskStateFreshMs && !Bridge.RouteActive && !Bridge.Moving
      && Bridge.WithinTolerance) return "settled";
  // The bridge's motion_timeout_ms already budgets the whole route, every stage included.
  return Elapsed >= Math.max(TaskSettleTimeoutMs, Bridge.MotionTimeoutMs) ? "timeout" : "waiting";
}

/**
 * Whether something else moved or stopped the arm while a chain's first leg
 * ran: the bridge's epoch moved past the one seen when that leg started
 * (each route start and each stop, Coast, Hold or Home moves it; arriving
 * and a grasp hold do not). Its second leg is then never sent.
 */
export function ChainLegInterrupted(Wait: TaskSettleWait, Epoch: number) {
  return Boolean(Wait.Chain?.Next) && Wait.startedAt !== null && Wait.StartedEpoch != null && Epoch !== Wait.StartedEpoch;
}

// A chain's gripper leg reads settled within the settle tolerance while the jaws can still creep
// toward their goal, or into a grasp's squeeze: 2026-10-03 14:46 the gripper crept 3 steps while
// the tip move after an open planned, and the bridge refused it ("Arm moved during planning").
// The tip move is sent once the gripper held within ChainGripperStillRaw for ChainGripperStillMs,
// or ChainGripperStillWaitMs after the leg settled (the bridge then allows a little creep itself).
export const ChainGripperStillMs = 400;
export const ChainGripperStillRaw = 1;
export const ChainGripperStillWaitMs = 2000;
export type RawSample = { At: number; Raw: number };

/** Whether a chain's tip move may follow its gripper leg now: the jaws held still (Samples: their present raw over time, ms). */
export function ChainGripperStill(Samples: RawSample[], Now: number, SettledAt: number): boolean {
  if (Now - SettledAt >= ChainGripperStillWaitMs) return true;
  const Start = Now - ChainGripperStillMs;
  // The reading from before the window too, so the whole window is covered.
  const Earlier = Samples.filter(Sample => Sample.At <= Start), Before = Earlier[Earlier.length - 1];
  const Window = Samples.filter(Sample => Sample.At > Start && Sample.At <= Now);
  if (!Before || !Window.length) return false;
  const Values = [Before.Raw, ...Window.map(Sample => Sample.Raw)];
  return Math.max(...Values) - Math.min(...Values) <= ChainGripperStillRaw;
}

// A motion whose target the app snapped (AiTargetSnap.ts), in the AI's display frame.
export type TaskMotionSnap = { Requested: Point3; RequestedPitch: number; Target: Point3; Pitch: number; DistanceCm: number; Reason: string | null };
// Chain: a chained motion's run (AiChain.ts DescribeChainRun): which legs ran, in which order.
// Planned: it was the app's own NEXT STEP, offered after the model did not copy it (PlannedStepNote).
export type TaskMotionFact = { Step: number; StartTip: Point3; EndTip: Point3; Target: Point3 | null; Servos: string | null; Snap?: TaskMotionSnap | null;
  Chain?: string | null; Planned?: boolean };

const PointText = (P: Point3) => `X ${Cm(P.x)}, Y ${Cm(P.y)}, Z ${Cm(P.z)}`;
const Wrist = (Pitch: number) => `wrist ${Cm(Pitch)}`;

// THE APP'S PLANNED STEP: the model did not copy a planned grasp's NEXT STEP in all its revision
// attempts (GraspPlan.ts NextStepReason refused each), so the app offered the step itself, read,
// snapped and checked like a reply, for the user to approve. What the card, the chat and the
// next check-in say about it.
export const PlannedStepNote = "This is the app's planned NEXT STEP from the GRASP PLAN, not the AI's: the AI did not follow it. Check it before approving.";
export const PlannedStepFact = "That motion was the app's own NEXT STEP from the GRASP PLAN, approved by the user because your proposals did not follow it: copy each NEXT STEP exactly from now on, or reply ask.";
/** The chat's reply when the app offers its planned step: why, and what the step does. */
export function PlannedStepReply(Attempts: number, StepText: string) {
  const What = StepText.replace(/^NEXT STEP of the grasp \([^)]*\):\s*/, "").replace(/:?\s*"motion_proposal":[\s\S]*$/, "").replace(/\.$/, "");
  return `The AI did not follow the app's NEXT STEP in ${Attempts} revision${Attempts === 1 ? "" : "s"}, so the app offers its own planned step instead: `
    + `${What}. Approve it to go on, or Block to stop the task.`;
}

/** What the last motion measurably did, in the AI's display frame. */
export function DescribeMotionFact(Fact: TaskMotionFact) {
  const Moved = Distance(Fact.StartTip, Fact.EndTip);
  const Snap = Fact.Snap;
  return `Your last approved motion (motion ${Fact.Step}) started and has stopped: the tip moved ${Cm(Moved)} cm, from ${PointText(Fact.StartTip)} to ${PointText(Fact.EndTip)}`
    + (Fact.Target ? `, and is now ${Cm(Distance(Fact.EndTip, Fact.Target))} cm from that motion's target` : "")
    + (Fact.Servos ? `; ${Fact.Servos}` : "")
    + (Moved < 0.5 && !Fact.Servos ? ". That motion made no progress." : ".")
    + (Fact.Chain ? ` ${Fact.Chain}` : "")
    + (Fact.Planned ? ` ${PlannedStepFact}` : "")
    // The model otherwise believes it went where it asked, and asks again.
    + (Snap ? ` You asked for ${PointText(Snap.Requested)}, ${Wrist(Snap.RequestedPitch)}; that is not reachable${Snap.Reason ? ` (${Snap.Reason})` : ""},`
      + ` so the app sent the nearest possible target ${PointText(Snap.Target)}, ${Wrist(Snap.Pitch)} (${Cm(Snap.DistanceCm)} cm from your target).` : "");
}

/** The approved-motions line's note for a snapped target: "(you asked for ..., wrist w0: snapped 0.6 cm, why)". */
export function SnapMotionNote(Snap: TaskMotionSnap) {
  const Turn = AngleDelta(Snap.Pitch, Snap.RequestedPitch);
  return ` (you asked for ${PointText(Snap.Requested)}, ${Wrist(Snap.RequestedPitch)}: snapped ${Cm(Snap.DistanceCm)} cm`
    + `${Turn >= 0.1 ? ` and ${Cm(Turn)} deg of wrist` : ""}${Snap.Reason ? `, ${Snap.Reason.split(";")[0]}` : ""})`;
}

/**
 * The check-in line after the bridge refused the approved motion: nothing
 * moved, so the model must not read the motion as done. With a snapped
 * target it says which point was sent.
 */
export function DescribeBridgeRefusal(Step: number, Text: string, Snap: TaskMotionSnap | null, Chain: string | null = null) {
  return `Your last approved motion (motion ${Step}) was NOT executed: the bridge refused it: ${Text.replace(/\.$/, "")}. The arm did not move.`
    + (Chain ? ` ${Chain}` : "")
    + (Snap ? ` The target sent was the app's nearest possible point ${PointText(Snap.Target)}, ${Wrist(Snap.Pitch)} (you asked for ${PointText(Snap.Requested)}, ${Wrist(Snap.RequestedPitch)}).` : "")
    + " Do not propose the same target again: the bridge would refuse it again. Choose another approach or reply ask.";
}

// Check-ins list the motions actually approved so far: the model otherwise tends
// to copy its previous reply word for word and stall in place. Every claim here
// is measured; the old text asserted arrival even when the arm had not moved.
// Snapped: the last motion went to a snapped target (DescribeMotionFact says where).
export function BuildTaskCheckInPrompt(Task: { task: string; step: number }, Motions: string[], LastMotion: string | null, Snapped = false): string {
  return [
    `AUTO CHECK-IN — motion ${Task.step} of ${MAX_AI_TASK_STEPS} max for the ongoing task: "${Task.task}".`,
    ...(LastMotion ? [LastMotion] : []),
    ...(Snapped ? ["Snapped targets are the closest the arm can get with that approach: do not ask for the same unreachable point again; change the approach (a higher hover, another side) or reply ask."] : []),
    "A fresh camera image and the arm state read after that motion stopped are attached.",
    ...(Motions.length ? ["Motions approved so far in this task:", ...Motions] : []),
    // "Change your approach" is not leave to skip a NEXT STEP (2026-10-01 18:54: the model descended instead of the corrected hover).
    "Coordinates in your earlier replies are stale: use only the positions in this message. Do not repeat earlier replies: judge the NEW image and position. If the tip is not closer to the goal than before, change your approach or reply ask; when the TASK OBJECT line gives a NEXT STEP, that step is the approach: copy it.",
    "Compare the image against what you expected, then reply per TASK MODE:",
    "task_status \"continue\" with your next motion_proposal, or \"done\"/\"ask\" with motion_proposal null.",
    ...(Task.step >= MAX_AI_TASK_STEPS ? ["This is your last motion for this task: if you cannot finish with it, reply ask and say what the user must change."] : []),
  ].join("\n");
}

type ServoTarget = { name: ServoName; targetRaw: number };
type NoOpMotion = { hasTargetMotion: boolean; target: TargetPose; servoTargets: ServoTarget[] };

/**
 * The servo targets that would actually change a servo: the model often
 * restates the gripper where it already is. The bridge runs one move at a
 * time, so only these are sent.
 */
export function ChangingServoTargets<T extends ServoTarget>(Targets: T[], CurrentRaw: Partial<Record<ServoName, number>>): T[] {
  return Targets.filter(Item => CurrentRaw[Item.name] === undefined || RawDelta(Item.targetRaw, CurrentRaw[Item.name]!) > NoOpServoRaw);
}

/**
 * Why a proposal would not move the arm, or null. The bridge's commanded
 * target counts only while the tip is near it (it sags about 0.8 cm below);
 * after Home or a hand move the commanded target is stale. A servo that
 * would change (closing the gripper at the held pose) is a motion.
 */
export function NoOpReason(Motion: NoOpMotion, Live: { Tip: Point3; WristPitch: number },
  Commanded: TargetPose | null, ServoTargetsRaw: Partial<Record<ServoName, number>>): string | null {
  if (ChangingServoTargets(Motion.servoTargets, ServoTargetsRaw).length) return null;
  if (Motion.hasTargetMotion) {
    if (Commanded && Distance(Live.Tip, Commanded) <= 2 && Distance(Motion.target, Commanded) <= NoOpCommandedCm
        && AngleDelta(Motion.target.wristPitch, Commanded.wristPitch) <= NoOpDeg) {
      return `that target is ${Cm(Distance(Motion.target, Commanded))} cm from the target the arm is already holding`;
    }
    if (Distance(Motion.target, Live.Tip) <= NoOpLiveCm && AngleDelta(Motion.target.wristPitch, Live.WristPitch) <= NoOpDeg) {
      return `that target is ${Cm(Distance(Motion.target, Live.Tip))} cm from where the tip already is`;
    }
    return null;
  }
  if (Motion.servoTargets.length) {
    return `${Motion.servoTargets.map(Item => Item.name.replace(/_/g, " ")).join(" and ")} ${Motion.servoTargets.length > 1 ? "are" : "is"} already there`;
  }
  return null;
}

/** NextStep: the TASK OBJECT line's NEXT STEP for the arm as it is, repeated (the model had copied an earlier reply instead). */
export function NoOpFeedbackPrompt(Reason: string, NextStep: string | null = null) {
  return [
    `The app did not accept your last motion_proposal: ${Reason}, so it would not move the arm. It was not sent and not counted.`,
    ...(NextStep ? [`The TASK OBJECT line in this message gives the step for the arm as it is now: ${NextStep}`] : []),
    "Choose a target that makes real progress toward the goal, or reply task_status \"done\" or \"ask\" with motion_proposal null.",
    "Return JSON only.",
  ].join("\n");
}

/**
 * A no-op whose target the app snapped: the request itself is out of reach
 * and its nearest reachable point is where the arm already is, so asking for
 * it again cannot help. Snap in the AI's display frame.
 */
export function SnapNoOpReason(Snap: TaskMotionSnap) {
  return `your target ${PointText(Snap.Requested)}, ${Wrist(Snap.RequestedPitch)} is not reachable${Snap.Reason ? ` (${Snap.Reason})` : ""};`
    + ` its nearest reachable point (${PointText(Snap.Target)}, ${Wrist(Snap.Pitch)}, ${Cm(Snap.DistanceCm)} cm from it) is where the arm already is,`
    + " so the arm cannot get closer to it this way";
}

/**
 * An approved proposal that the app did not send after all (Approve's check
 * against the live state failed): the task goes on with this as its check-in
 * instead of waiting for an approval that can never come.
 */
export function NotSentFeedbackPrompt(Issues: string[]) {
  return [
    `Your last proposal was not sent: ${Issues.join(" ").replace(/\.$/, "")}. The arm did not move.`,
    "Choose a different target from the current arm position, or reply task_status \"ask\" with motion_proposal null.",
    "Return JSON only.",
  ].join("\n");
}

/** A target's identity for repeat detection: display-frame XYZ rounded to 0.5 cm. */
export function TargetKey(Target: Point3) {
  const Display = mirrorDisplayPoint(Target);
  return [Display.x, Display.y, Display.z].map(Value => (Math.round(Value * 2) / 2).toFixed(1)).join("|");
}

// The last offered final target: its key and wrist pitch.
export type FinalTargetMark = { Key: string; WristPitch: number };
export function MarkFinalTarget(Target: TargetPose): FinalTargetMark {
  return { Key: TargetKey(Target), WristPitch: Target.wristPitch };
}

/**
 * The same final target as the last one offered: the same point (TargetKey)
 * and the same wrist within NoOpDeg. Turning the wrist at the same point is
 * a new motion (NoOpReason counts it as one), not the model repeating itself.
 */
export function RepeatsLastTarget(Last: FinalTargetMark | null, Target: TargetPose) {
  return Boolean(Last && Last.Key === TargetKey(Target) && AngleDelta(Target.wristPitch, Last.WristPitch) <= NoOpDeg);
}

// --- Task ends, arm stillness and time limits -------------------------------

/** Who or what ended a task, for its task_end record. */
export type TaskEndSource = "stop_task" | "stop_and_hold" | "emergency_stop" | "motion_fault" | "configuration_change"
  | "self_calibration" | "disconnect" | "manual_mode" | "block" | "new_task" | "arm_moving" | "timeout" | "error"
  | "camera_gate" | "bridge_refused" | "step_limit" | "motion_not_started" | "motion_timeout" | "app"
  | "model_done" | "model_ask" | "model_no_motion" | "context_reset" | "chain_interrupted"
  // Not ready for a task: the running bridge is not the code this page expects, or a joint reading is overridden.
  | "not_ready";

// A task reads the arm's pose only once the arm is still: the 22:46 task was
// planned from a pose the arm was only passing through on its way home, and
// its relative move was refused 5.4 cm later at approval. A task waits this
// long for the arm to stop, and the arm must have been still this long.
export const ArmStillWaitMs = 10000;
export const ArmStillHoldMs = 300;
// The tip may not move farther than this between the camera frame and the
// prompt built from it (the same limit Approve applies to a relative move).
export const ArmMovedDuringCaptureCm = 1;
export type ArmMotionState = { Live: boolean; StateAgeMs: number; RouteActive: boolean; Moving: boolean; StillForMs: number };

/**
 * Why the arm's position cannot be read as where it rests, or null when it
 * can. An arm the bridge does not stream (not connected) cannot be moving.
 * State older than TaskStateFreshMs means the bridge is quiet, which it is
 * while it plans a motion.
 */
// Encoder noise between two state frames of a joint at rest, in raw steps.
export const ArmStillJitterRaw = 2;

export function ArmNotStillReason(State: ArmMotionState): string | null {
  if (!State.Live) return null;
  if (State.RouteActive || State.Moving || State.StillForMs < ArmStillHoldMs) return "the arm is moving";
  if (State.StateAgeMs >= TaskStateFreshMs) return `the bridge has sent no arm state for ${(State.StateAgeMs / 1000).toFixed(1)} s, as it does while it plans a motion`;
  return null;
}

// Time limits for the network steps of one task step. A hung call used to
// leave the task waiting for ever with the input disabled.
export const AiModelTimeoutMs = 120000;
export const LocateTimeoutMs = 60000;

/** A model or /locate call that did not answer in time: the task ends with this message as its reason. */
export class AiStepTimeoutError extends Error {
  What: string;
  Ms: number;
  constructor(What: string, Ms: number) { super(`${What} did not answer within ${Ms / 1000} s`); this.name = "AiStepTimeoutError"; this.What = What; this.Ms = Ms; }
}

/**
 * Runs one network step with its own time limit. Aborting Parent (Stop task,
 * a fault) aborts the step; running out of time aborts it and rejects with
 * AiStepTimeoutError at the deadline, even if the step ignores its signal.
 */
export function WithTimeout<T>(Run: (Signal: AbortSignal) => Promise<T>, Parent: AbortSignal, Ms: number, What: string): Promise<T> {
  const Controller = new AbortController();
  const Forward = () => Controller.abort(Parent.reason);
  if (Parent.aborted) Controller.abort(Parent.reason);
  else Parent.addEventListener("abort", Forward, { once: true });
  let TimedOut = false;
  return new Promise<T>((Resolve, Reject) => {
    const Timer = setTimeout(() => {
      TimedOut = true;
      Controller.abort();
      Reject(new AiStepTimeoutError(What, Ms));
    }, Ms);
    const Finish = () => { clearTimeout(Timer); Parent.removeEventListener("abort", Forward); };
    Run(Controller.signal).then(
      Value => { Finish(); Resolve(Value); },
      Failure => { Finish(); Reject(TimedOut ? new AiStepTimeoutError(What, Ms) : Failure); },
    );
  });
}

// --- History ---------------------------------------------------------------

const StaleKeys = /("(?:target_cm|target_estimate_cm|target_estimate|target_position_cm|target_position|coordinates_cm)"\s*:\s*)\{[^{}]*\}/g;
/** Earlier replies' coordinates are out of date by the next check-in: keep the words, drop the numbers. */
export function MarkStaleCoordinates(Content: string) {
  return Content.replace(StaleKeys, '$1"[stale]"').replace(/\b([XYZ])(\s*[=:]?\s*)[-+]?\d+(?:\.\d+)?/g, "$1$2[stale]");
}

/** One stored turn per request: the prompt and the final reply, with stale coordinates marked. */
export function CompactTurn(Conversation: AiChatMessage[], Turn: number): AiChatMessage[] {
  const User = Conversation.find(Message => Message.role === "user");
  const Reply = [...Conversation].reverse().find(Message => Message.role === "assistant");
  if (!User || !Reply) return [];
  const Feedback = Conversation.filter(Message => Message.role === "user" && Message !== User);
  const Note = Feedback.length ? `\n(The app rejected ${Feedback.length} proposal${Feedback.length === 1 ? "" : "s"} in this turn; last: ${Feedback[Feedback.length - 1].content})` : "";
  return [{ role: "user", content: MarkStaleCoordinates(User.content + Note), turn: Turn },
    { role: "assistant", content: MarkStaleCoordinates(Reply.content), turn: Turn }];
}

/** The newest whole turns: a slice never starts with an orphaned reply or feedback message. */
export function SliceWholeTurns(Messages: AiChatMessage[], MaxTurns: number): AiChatMessage[] {
  const Turns: AiChatMessage[][] = [];
  for (const Message of Messages) {
    const Last = Turns[Turns.length - 1];
    if (Last && Message.turn !== undefined && Last[0].turn === Message.turn) Last.push(Message);
    else Turns.push([Message]);
  }
  return Turns.filter(Turn => Turn[0].role === "user" && Turn.some(Message => Message.role === "assistant"))
    .slice(-Math.max(0, MaxTurns)).flat();
}

// --- Feasibility feedback --------------------------------------------------

type ReachContext = { Servos: Record<ServoName, ServoState>; ElbowMode: "up" | "down"; Limits?: IkAngleLimits | null };

// These hints run on the page's thread for every rejected proposal, where a
// long search freezes Stop and the state stream: a few dozen IK solves each.
const BisectionSteps = 7;
const BasketCoarseCm = 2;

/**
 * The reachable, collision-free point nearest the target on the straight
 * line from the tip, by bisection with the app's own IK (model frame).
 */
export function NearestReachablePoint(From: TargetPose, To: TargetPose, Context: ReachContext) {
  const Pitches = [...new Set([To.wristPitch, From.wristPitch, ...ReachPitches])];
  const At = (T: number) => ({ x: From.x + (To.x - From.x) * T, y: From.y + (To.y - From.y) * T, z: From.z + (To.z - From.z) * T });
  const End = ReachAt(At(1), Context, Pitches);
  if (End.Pitch != null) return { Point: At(1), Pitch: End.Pitch, ShortCm: 0 };
  let Low = 0, High = 1, Best: { Point: Point3; Pitch: number } | null = null;
  const Start = ReachAt(At(0), Context, Pitches);
  if (Start.Pitch == null) return null;
  Best = { Point: At(0), Pitch: Start.Pitch };
  for (let Step = 0; Step < BisectionSteps; Step += 1) {
    const Middle = (Low + High) / 2, Reach = ReachAt(At(Middle), Context, Pitches);
    if (Reach.Pitch != null) { Low = Middle; Best = { Point: At(Middle), Pitch: Reach.Pitch }; } else High = Middle;
  }
  return { ...Best, ShortCm: Distance(Best.Point, To) };
}

/**
 * The lowest height at this X/Y where some wrist pitch clears the basket:
 * 2 cm steps up from the target, then halved to 0.5 cm. Null when the basket
 * is not what blocks the target (no wrist pitch collides there: it is a reach
 * problem, and "clears the basket" would mislead); Z null when no height up
 * to RangeCm is both reachable and clear.
 */
export function BasketClearHeight(Target: TargetPose, Context: ReachContext, RangeCm = 16) {
  const At = (Rise: number) => ReachAt({ x: Target.x, y: Target.y, z: Target.z + Rise }, Context);
  const Here = At(0);
  if (Here.Pitch != null || !Here.Collision) return null;
  let Low = 0;
  for (let Rise = BasketCoarseCm; Rise <= RangeCm; Rise += BasketCoarseCm) {
    const Reach = At(Rise);
    if (Reach.Pitch == null) { Low = Rise; continue; }
    let High = Rise, Pitch = Reach.Pitch;
    while (High - Low > 0.5) {
      const Middle = Low + Math.floor((High - Low) / 0.5 / 2) * 0.5, Try = At(Middle);
      if (Try.Pitch != null) { High = Middle; Pitch = Try.Pitch; } else Low = Middle;
    }
    return { Z: Target.z + High, Pitch };
  }
  return { Z: null, Pitch: null };
}

type FeedbackMotion = { requestedTarget: TargetPose; target: TargetPose; requestedWristPitchTarget: number;
  requestedRelativeInches: { left: number; farther: number; up: number }; servoTargets: { name: ServoName; targetRaw: number }[];
  // The app's target snap (AiTargetSnap.ts), when it ran.
  snap?: SnapResult | null };

export function BuildAiFeasibilityFeedbackPrompt(
  Issues: string[],
  Motion: FeedbackMotion,
  CurrentTarget: TargetPose,
  Context: ReachContext,
  Attempt: number,
  MaxAttempts: number,
) {
  // IMPORTANT: feedback must be in the SAME frame the AI receives positions in
  // (display frame: X positive = left) — mirrorDisplayPoint, NOT the GUI's visual
  // frame (proposalDisplayPointCm), which negates X and would contradict every
  // other coordinate the AI has seen.
  const N = FormatAiPromptNumber;
  const CurrentPoint = mirrorDisplayPoint(CurrentTarget);
  const RejectedPoint = mirrorDisplayPoint(Motion.requestedTarget);
  const ClampedPoint = mirrorDisplayPoint(Motion.target);
  // The clamp line only helps when the app actually changed the target.
  const Clamped = Distance(RejectedPoint, ClampedPoint) >= 0.1 || AngleDelta(Motion.target.wristPitch, Motion.requestedTarget.wristPitch) >= 0.1;
  const Hints: string[] = [];
  const Cartesian = Distance(Motion.requestedTarget, CurrentTarget) >= 0.1 || AngleDelta(Motion.requestedTarget.wristPitch, CurrentTarget.wristPitch) >= 0.1;
  // The snap's own nearest target, searched around the request under the
  // app's rules, with why it was not used. The straight line from the tip
  // (7 fixed pitches) only when the snap found none: at 16:05 it said 8.02
  // cm short where a target 0.67 cm away existed.
  const SnapNearest = Motion.snap?.Kind === "refused" ? Motion.snap.Nearest : null;
  if (SnapNearest) {
    const Point = mirrorDisplayPoint(SnapNearest.Target);
    Hints.push(`Nearest possible target: X ${N(Number(Point.x.toFixed(2)))} cm, Y ${N(Number(Point.y.toFixed(2)))} cm, Z ${N(Number(Point.z.toFixed(2)))} cm, wrist ${N(Number(SnapNearest.Target.wristPitch.toFixed(1)))} deg (${N(Number(SnapNearest.DistanceCm.toFixed(2)))} cm from yours). The app did not use it because ${SnapNearest.Why}.`);
  }
  if (Cartesian && !Motion.servoTargets.length) {
    const Nearest = SnapNearest ? null : NearestReachablePoint(CurrentTarget, Motion.requestedTarget, Context);
    if (Nearest && Nearest.ShortCm >= 0.1) {
      const Point = mirrorDisplayPoint(Nearest.Point);
      Hints.push(`Nearest reachable, collision-free point on the straight line from the tip toward your target: X ${N(Point.x)} cm, Y ${N(Point.y)} cm, Z ${N(Point.z)} cm, wrist ${N(Nearest.Pitch)} deg (${N(Number(Nearest.ShortCm.toFixed(2)))} cm short of your target).`);
    }
    const Clear = Issues.some(Issue => /basket/i.test(Issue)) ? BasketClearHeight(Motion.requestedTarget, Context) : null;
    if (Clear) {
      Hints.push(Clear.Z != null && Clear.Pitch != null
        // Not "every wrist pitch hits the basket": some of them fall short of the point instead.
        ? `At X ${N(RejectedPoint.x)}, Y ${N(RejectedPoint.y)} no wrist pitch reaches below Z ${N(Number(mirrorDisplayPoint({ ...Motion.requestedTarget, z: Clear.Z }).z.toFixed(2)))} cm without touching the basket; that is the lowest clear height there (wrist ${N(Clear.Pitch)} deg). An object lower than that cannot be grasped at this X/Y.`
        : `At X ${N(RejectedPoint.x)}, Y ${N(RejectedPoint.y)} no height up to 16 cm above your target is both reachable and clear of the basket at any wrist pitch.`);
    }
  }
  const ServoLines = Motion.servoTargets.map((Proposal) => {
    const Servo = Context.Servos[Proposal.name];
    return `${Proposal.name}: proposed raw ${Proposal.targetRaw}, fraction ${N(rawToFraction(Servo, Proposal.targetRaw))}`;
  });
  return [
    "The app checked your last JSON motion_proposal with its local SO-101 IK solver and slider limits.",
    "That proposal is NOT possible or valid as written.",
    `Revision attempt ${Attempt} of ${MaxAttempts}.`,
    "",
    "Problems:",
    ...Issues.map((Issue) => `- ${Issue}`),
    "",
    "The arm is currently at:",
    `X ${N(CurrentPoint.x)} cm, Y ${N(CurrentPoint.y)} cm, Z ${N(CurrentPoint.z)} cm, wrist ${N(CurrentTarget.wristPitch)} deg.`,
    "Your rejected target would have been:",
    `move_delta_cm left ${N(Motion.requestedRelativeInches.left)}, farther ${N(Motion.requestedRelativeInches.farther)}, up ${N(Motion.requestedRelativeInches.up)}, wrist_pitch_degrees ${N(Motion.requestedWristPitchTarget)}.`,
    `Absolute rejected target: X ${N(RejectedPoint.x)} cm, Y ${N(RejectedPoint.y)} cm, Z ${N(RejectedPoint.z)} cm, wrist ${N(Motion.requestedTarget.wristPitch)} deg.`,
    ...(Clamped ? ["The app adjusted that to this preview target:",
      `X ${N(ClampedPoint.x)} cm, Y ${N(ClampedPoint.y)} cm, Z ${N(ClampedPoint.z)} cm, wrist ${N(Motion.target.wristPitch)} deg.`] : []),
    ...Hints,
    "All coordinates above are in your usual frame: X positive = left, Y positive = farther, Z positive = up.",
    ...(ServoLines.length ? ["Rejected direct slider targets:", ...ServoLines] : []),
    "",
    "Return JSON only. You may provide a different motion_proposal, or set motion_proposal to null (task_status \"ask\" if the goal cannot be reached).",
  ].join("\n");
}

/**
 * What the user can do after a stop; "move it closer" only when the camera was
 * verified and the object was not planned. Planned: the app had a GRASP PLAN
 * for it, so it was in reach: after 07:56 (2026-10-01) "move the can closer"
 * sent the user to move a can the arm already held its hover point over.
 */
export function TaskStopAdvice(CameraVerified: boolean, Label: string | null, Planned = false) {
  if (!CameraVerified) return "The camera was not verified against its calibration for this task: lock the camera head, run Automatic calibration, then send the task again.";
  return Planned
    ? `The ${Label ?? "object"} is within reach and its grasp was planned; the AI could not make the next move. Say "try again" to go on from where the arm is, or send the task again.`
    : `Move ${Label ? `the ${Label}` : "the object"} closer to the right arm, clear of the basket rim, or rephrase the task, then send it again.`;
}
