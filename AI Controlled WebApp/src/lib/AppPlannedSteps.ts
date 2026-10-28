import { ChainRules, type ChainRun } from "./AiChain";
import type { TaskObjectQuery } from "./TaskObject";
import { Singular } from "./TaskObject";

// THE APP RUNS THE GRASP PLAN'S STEPS ITSELF. On a grasp the planner already works out every step (GRASP PLAN /
// NEXT STEP: open and hover, descend, close and lift), and the model mostly copied the app's own step back: each
// check-in cost a fresh /locate (about 11.7 s) and a model call (about 13.2 s, 80% of it the prefill of an
// 8,100-8,400-token prompt, nothing cached; 2026-10-25 recording). So after a motion that settled with nothing
// unexpected, the app proposes the plan's NEXT STEP itself, through the same approval card (labelled as the app's
// planned step), and the model is not called. The user still approves every motion. The model is called at the
// task's start, and whenever anything is not as planned (AppStepBlocker in App.tsx says why), and then hears which
// steps the app ran on its own since its last reply (AppStepsSummary).

export const AppStepRules = {
  // A tip move that stopped farther than this from its target fell short (AiChain's arrival limit for a chain's first leg).
  ShortCm: ChainRules.ArrivedCm,
  // Less tip travel than this, with no servo move, made no progress (AiTaskLoop.ts DescribeMotionFact says the same).
  NoProgressCm: 0.5,
  // Whether a step may run while the arm hides the task object (lib/ArmOcclusion.ts: the app keeps its last measurement
  // and GRASP PLAN). On: the gripper hiding the object is expected over a grasp, and the model cannot see it then either;
  // every motion is still approved. On the 2026-10-25 pill bottle every check-in after the first had it hidden.
  RunWhileHidden: true,
  // The task object counts as left where it stood (a grasp that did not hold it) within this of its planned spot.
  LeftBehindCm: 2,
  LeftBehindZCm: 1.5,
} as const;

/** A check-in after an approved motion, as the app judges it: Why says what was not as planned (null: nothing). */
export type AppCheckIn = { Why: string | null };

/** One servo of the last motion: where it was commanded and where it stopped; Closing: a gripper close (it stops on the object). */
export type MotionServo = { Name: string; CommandedRaw: number; PresentRaw: number; Closing: boolean };

/**
 * What was not as planned about the motion a check-in follows, or null. MovedCm: how far the tip moved; OffTargetCm:
 * how far it stopped from its target (null without a tip target); a servo off its commanded raw beyond Tolerance
 * (a gripper close stops on the object, as a grasp should); a chained motion with a part not run; a snapped target.
 */
export function MotionAnomaly(Motion: { MovedCm: number; OffTargetCm: number | null; Servos: MotionServo[]; Chain: ChainRun | null;
  Snapped: boolean; Tolerance: number }): string | null {
  if (Motion.Snapped) return "the last motion's target was not reachable as asked, so the app snapped it";
  if (Motion.Chain) {
    const Results = Motion.Chain.Together ? Motion.Chain.Results.slice(0, 1) : Motion.Chain.Results;
    if (Motion.Chain.Why || Results.some(Result => Result !== "sent" && Result !== "skipped")) {
      return `a part of the last chained motion did not run${Motion.Chain.Why ? ` (${Motion.Chain.Why.replace(/\.$/, "")})` : ""}`;
    }
  }
  if (Motion.OffTargetCm != null && Motion.OffTargetCm > AppStepRules.ShortCm) {
    return `the last motion stopped ${Motion.OffTargetCm.toFixed(1)} cm from its target (more than ${AppStepRules.ShortCm} cm)`;
  }
  const Off = Motion.Servos.find(Servo => !Servo.Closing && RawDistance(Servo.PresentRaw, Servo.CommandedRaw) > Motion.Tolerance);
  if (Off) return `the ${Off.Name.replace(/_/g, " ")} stopped at raw ${Off.PresentRaw}, not the raw ${Off.CommandedRaw} it was sent to`;
  if (Motion.MovedCm < AppStepRules.NoProgressCm && !Motion.Servos.length) return "the last motion made no progress";
  return null;
}
const RawDistance = (A: number, B: number) => { const D = (((A - B) % 4096) + 4096) % 4096; return Math.min(D, 4096 - D); };

/**
 * Why a NEXT STEP sentence needs the model, or null when the app can run it as it stands: no motion to copy (reply
 * ask, done, or a plain instruction), or a step that depends on what the image shows ("descend only if the image
 * shows the open jaws beside it", "if the image shows it in the jaws").
 */
export function StepNeedsModel(Text: string, Json: string | null): string | null {
  const Step = StepWords(Text);
  if (!Json) return `the NEXT STEP has no motion to run (${Step})`;
  if (/\bimage\b/i.test(Text)) return `the NEXT STEP depends on what the image shows (${Step})`;
  if (/\breply (?:task_status|ask|done)\b/i.test(Text)) return `the NEXT STEP asks for a reply (${Step})`;
  return null;
}

/** The planner's last step of a grasp lifted with the jaws holding it (AiMeasurements.ts NextPlannedGraspStep). */
export function IsLiftedStep(Text: string) {
  return /^NEXT STEP of the grasp \([^)]*\): it is lifted\. If the image shows it in the jaws, reply task_status "done"/.test(Text.trim());
}

/** What a NEXT STEP does, without its head and its JSON: "descend straight down to the GRASP POINT Z 2.4". */
export function StepWords(Text: string) {
  return Text.replace(/^\s*NEXT STEP of the grasp \([^)]*\):\s*/, "").replace(/:?\s*"motion_proposal":[\s\S]*$/, "").replace(/\.$/, "").trim();
}

// The words a grab-only request may hold besides the object's own: a polite lead, the verb, its particle, articles,
// and where it stands on the desk.
const GraspOnlyVerbs = new Set(["grab", "grasp", "pick", "lift"]);
const GraspOnlyFill = new Set(["please", "can", "could", "would", "will", "you", "the", "a", "an", "my", "that", "this", "up",
  "on", "desk", "table"]);
/**
 * Whether a task asks only to grab or pick up its object (the app may then end it after the GRASP PLAN's lift with
 * the jaws holding it). Conservative: one grab verb and nothing but the object's words and filler; any other word
 * ("and", "put", "basket", "me", "then", a typo) leaves it to the model.
 */
export function GraspOnlyRequest(Task: string, Query: TaskObjectQuery | null): boolean {
  if (!Query || !GraspOnlyVerbs.has(Query.Verb) || Query.VerbFuzzy || Query.ArmMoves) return false;
  const Words = Task.toLowerCase().match(/[a-z]+(?:-[a-z]+)*|\d+(?:\.\d+)?/g) ?? [];
  const Own = new Set([...Query.Name.split(/\s+/), ...Query.Nouns, ...Query.Colours, ...Query.Sizes]);
  let Verbs = 0;
  for (const Word of Words) {
    if (GraspOnlyVerbs.has(Word)) { Verbs += 1; continue; }
    if (GraspOnlyFill.has(Word) || Own.has(Word) || Own.has(Singular(Word))) continue;
    return false;
  }
  return Verbs === 1;
}

// What the card, the chat, the record and the model are told about a step the app ran itself.
export const AppRunStepNote = "This is the app's planned NEXT STEP from the GRASP PLAN, proposed by the app without asking the AI: nothing unexpected happened since the last step. Check it before approving.";
export const AppRunReason = "the last motion settled as planned, the GRASP PLAN was kept with the task object re-measured where it was planned, and its NEXT STEP needs no judgement from the image";
export const AppRunStepFact = "That motion was the app's own NEXT STEP from the GRASP PLAN, proposed by the app without asking you and approved by the user (see STEPS THE APP RAN ITSELF).";
/** The chat's reply when the app proposes its planned step itself. */
export function AppRunStepReply(StepText: string) {
  return `The app's next GRASP PLAN step (the AI was not asked: nothing unexpected happened): ${StepWords(StepText)}. Approve it to go on, or Block to stop the task.`;
}

/** A step the app ran itself, approved: its motion number and what it did. */
export type AppRunStep = { Step: number; What: string; Json: string };
/**
 * The check-in's line for the model when it is called after steps the app ran on its own: which ones, and why it is
 * asked now. Null without any.
 */
export function AppStepsSummary(Steps: AppRunStep[], Reason: string): string | null {
  if (!Steps.length) return null;
  return `STEPS THE APP RAN ITSELF since your last reply, without asking you (each was the GRASP PLAN's NEXT STEP, sent exactly as planned and approved by the user): `
    + Steps.map(Step => `motion ${Step.Step}: ${Step.What}`).join("; ")
    + `. The app asks you now because ${Reason.replace(/\.$/, "")}.`;
}
/** The check-in with the summary after its first line (the AUTO CHECK-IN head). */
export function WithAppStepsSummary(Prompt: string, Summary: string | null) {
  if (!Summary) return Prompt;
  const Break = Prompt.indexOf("\n");
  return Break < 0 ? `${Prompt}\n${Summary}` : `${Prompt.slice(0, Break)}\n${Summary}${Prompt.slice(Break)}`;
}

/** The chat's line when the app ends a grab-only task after the GRASP PLAN's lift with the jaws holding the object. */
export function AppDoneReply(Label: string, OpenPercent: number) {
  return `The GRASP PLAN is complete: the jaws closed partway on the ${Label} (the gripper holds at ${OpenPercent}% open) and the arm lifted it. `
    + `The app ended the task without asking the AI; check that the ${Label} is in the jaws.`;
}
