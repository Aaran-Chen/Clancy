import { Vector3, type Matrix4 } from "three";
import Parts from "./GripperParts.json";
import { AiJawFloorAllows, AiTableFloorReason, DeskFloor, GripperFloorGap, GripperFloorGapOfLinks, GripperLowestZOfLinks, TableZCm,
  type AiFloor } from "./AiTableFloor";
import { PreviewConverges } from "./AutomaticWrist";
import { BasketPoseReason, BasketTargetReason } from "./BasketCollision";
import { CadForward, CadModel, CadPitch, GetCadAlignment, type CadAngles } from "./CadKinematics";
import { ConvexGap, type Support } from "./ConvexGap";
import { CloseOn, ConvexHull2, FixedFingertipZ, Gap, GripIn3D, MovingJawHinge, MovingJawReachCm, OpenClearance, PalmPolygon, StraddleOf,
  SweepRules, SweepWork, type P2 } from "./JawSweep";
import { InsetIkLimits } from "./TargetWorkspace";
import { ObservedBoxCovered } from "./ObservedSpace";
import type { MeasuredScene, SceneBox } from "./SceneGeometry";
import { angleFromRawValue, currentAngles, fractionToRaw, GetIkAngleLimits, IkWork, mirrorDisplayPoint, modelPointFromDisplayPoint, solveIkPreview,
  type IkAngleLimits, type Point3, type ServoName, type ServoState, type TargetPose } from "./robotModel";
// Types only: AiMeasurements imports this file, and the reach scan comes in through Ctx.Scan.
import type { LocatedObject, TaskReachability } from "./AiMeasurements";

// FIXED-JAW GRASP PLAN. The tip the IK and the AI call "the tip"
// (gripper_frame_link, gripper_link (-0.79, -0.02, -9.81)) is the inner pad
// of the gripper's FIXED jaw; only the other jaw moves. Aimed at an object's
// centre with the jaws open, the fixed finger came straight down onto its top
// (2026-09-30 23:50, "Grab the can"). The plan puts the fixed pad just beside
// the object, at a height and wrist pitch where the open moving jaw meets its
// side when it closes, and checks every gripper part (fixed finger, palm and
// servo, open moving jaw: GripperParts.json) against every measured object at
// the grasp, at the hover above it and along the joint-space path between
// them. The guard (GraspGuardReason) holds every AI tip motion of a task to
// the same parts. Display frame D throughout (X left+, Y far+, Z up+, mirrored).

export const GraspRules = {
  // The gripper fraction the steps open to (AiMeasurements GripperOpenFraction): CAD 107.4 deg.
  OpenFraction: 0.8,
  // size_cm is the 5-95% extent (the can read 5.0 against 6.0 real): half-extent = size/2 + this.
  TrimPadCm: 0.5,
  // The fixed pad's plane this far outside the padded footprint (c). See the spec's c sensitivity.
  FixedJawClearanceCm: 1.0,
  // The finger's half-width (0.5) plus 0.25: the band of the footprint the jaws close across.
  FingerBandHalfCm: 0.75,
  // W_eff does not change with pitch: wider than this at the first pitch is refused without a scan.
  WideEarlyExitCm: 5.3,
  // The lowest gripper point with the jaw open stays this far above the support; closing, this far.
  OpenFloorClearanceCm: 0.2,
  ClosingFloorClearanceCm: 0.0,
  // Palm/servo and open-jaw samples over the footprint grown OverTopGrowCm stay PalmClearanceCm above the top.
  PalmClearanceCm: 0.5,
  OverTopGrowCm: 1.0,
  // The fixed pad is flat over 0.9 cm of depth: at least this much of it must face the object.
  MinPadOverlapCm: 0.4,
  // Every part sample this far from every object: the planner, and the runtime guard (cover + loss 0.375).
  PlanMarginCm: 0.5,
  ContactMarginCm: 0.4,
  // The real moving jaw sits about 11 deg more closed than the CAD's (report C 2f): both are checked.
  JawUncertaintyDeg: 11,
  // CAD angle where the pads meet (the closing sweep's end).
  PadsMeetDeg: -11.41,
  PitchStepDeg: 2.5,
  // The wrist pitch a grasp is planned at when every check passes there, else the nearest that passes (equally near, the
  // steeper: PreferredFirst), for every object; a motion proposal without a wrist pitch (target_cm, or "auto") asks for it
  // too. Steep, so the camera still sees the object past the gripper: 2026-10-03's plans at -21 to -35 came in low and hid it.
  // Null: the order before (the steepest the jaw table holds; the sweep's gentlest grip), for the replays of 10-01.
  PreferredPitchDeg: -60 as number | null,
  // Joint-linear samples, like the bridge's first-choice straight line in joint space.
  PathSamples: 10,
  HoverAboveGraspCm: 6,
  HoverAboveTopCm: 3,
  // The lift after the close (AiMeasurements' steps): straight up this far.
  LiftCm: 6,
  MaxIterations: 4,
  // NEXT STEP alignment (d = tip - grasp point): toward the object at most AlignTowardCm, away AlignAwayCm.
  AlignTowardCm: 0.3,
  AlignAwayCm: 1.5,
  AlignAcrossCm: 1.0,
  // The close is offered with the tip this far below / above the grasp point.
  CloseBelowCm: 0.5,
  CloseAboveCm: 1.0,
  // The bridge stops each joint within this many steps of its target (bridge_server.py
  // SLIDER_GOAL_TOLERANCE_RAW): about 1 deg, 0.75 cm at 41 cm from the pan axis. The jaw
  // axis is across the arm's reach, so the pan's share of it is all along the jaws.
  ArrivalToleranceRaw: 12,
  // Holding a step's point (the bridge's target), the tip stops at most this far from it.
  ArrivalCm: 2,
  // Guard near the object (this near its prism, or where the real gripper meets it), below its top:
  // the wrist within WristMatchDeg of the plan's, no deeper than the plan.
  GuardNearCm: 3,
  WristMatchDeg: 10,
  DeeperThanPlanCm: 0.5,
  // The NEXT STEP rule (NextStepReason): a tip move ending within GuardNearCm of the object below its top +
  // NextStepAboveTopCm is the NEXT STEP's (its target within NextStepCm, its wrist within NextStepDeg) or a
  // rise of at least NextStepRiseCm that comes no nearer it. NextStepCm covers the step's 0.1 cm rounding
  // (a relative step and an absolute copy of the same point); NextStepDeg a reply that leaves the wrist out.
  NextStepAboveTopCm: 1,
  NextStepCm: 0.3,
  NextStepDeg: 5,
  NextStepRiseCm: 0.5,
  // Plan lock: the gripper this near the old footprint occludes a re-measure; moves under ReplanMoveCm keep the plan.
  NearCm: 6,
  ReplanMoveCm: 1.0,
  ShrinkRatio: 0.75,
  ValidDrop: 0.15,
  // A commanded full close leaves 0.86 cm between the pads (CAD).
  ThinCm: 0.9,
  // A neighbour where the fixed jaw goes down (its side is set by the pan; the wrist roll is held):
  // the grasp point rises in these steps, up to this far above mid-height and this far under the top,
  // to clear it; else the advice says how far the neighbour must move away (checked, up to this far).
  RaiseStepCm: 0.5,
  MaxRaiseCm: 3,
  RaiseBelowTopCm: 1.5,
  NeighbourMoveMaxCm: 4,
  // The sweep's held pitches (in its order) the raise is tried at.
  RaisePitches: 3,
  // Same-spot twins of the task object (TaskObject.ts SameSpotCm).
  TwinCm: 1.5,
  TwinBaseCm: 1.5,
  // M3, a standing round object (a can, a bottle): size_cm is the 5-95% extent of
  // what the camera sees of its top, which for a disc is about 0.75 of its
  // diameter (the 6.6 cm drink can read 5.0 x 4.5; its readings spread 4.5 to
  // 5.7). Its true diameter is planned as anything from the larger measured
  // side (Lo) to the widest it may be (Hi): that side over RoundMeasuredRatio,
  // or padded TrimPadCm a side like any object when that is wider (small ones:
  // a 2 cm tube stays the 3 cm M1 planned it as). A drink can is a drink can:
  // a "can" is planned from DrinkCanCm to DrinkCanMaxCm across, whatever it read
  // (narrower readings are a can partly seen), unless it read wider. The fixed
  // pad keeps its clearance from the widest, the open jaw clears the widest,
  // and the closing must hold every size in between.
  RoundMeasuredRatio: 0.75,
  DrinkCanCm: 6.6,
  DrinkCanMaxCm: 6.8,
  // A can read wider than DrinkCanMaxCm on one side and a can's width on the other had
  // something beside it measured with it (2026-10-01 07:59: 5.3 x 7.7, the tape roll in
  // front of it): planned as a drink can where its top was measured. Wider than this it
  // may be two things (two cans side by side read 13), and is planned as measured.
  DrinkCanMergedMaxCm: 10.2,
  // The diameters the closing is checked at, from Lo to Hi (and Hi).
  RoundStepCm: 0.2,
  RoundSides: 48,
  // The closing's discs are finer: a contact normal on a polygon edge is off the circle's by up to half a side's turn.
  SweepRoundSides: 96,
  // M3: the closing must also hold with the object this far nearer the palm or the tips than measured,
  // and wherever the jaw's push then carries it along the fingers, every DepthStepCm.
  DepthToleranceCm: 0.5,
  DepthStepCm: 0.05,
  // M3 pitch screen: a pitch whose quick (no IK) closing grip is this much over the limit is not tried.
  ScreenMarginDeg: 1,
  // ... nor on the strict pass one whose quick grip in 3D is this much over it (the placed grip came out up to 2.5 deg
  // above the screen's for 2026-10-03's pill bottle: the strict check refuses it at every height).
  StrictScreenMarginDeg: 3,
  // The steps open the gripper to the plan's fraction within this (raw counts drift); the
  // plan's jaw checks hold over it (the open jaw at the fraction plus this, the real
  // jaw's 11 deg more closed below the fraction minus this).
  OpenToleranceFraction: 0.03,
  // Printed points keep this much more from the basket and body than the collision model asks (AiTargetSnap SnapRules.CollisionMarginCm).
  BasketExtraCm: 0.2,
  // REAL-ARM MEASUREMENT. A round object's sweep plan (the can) puts the fixed pad this much farther from the
  // object along the jaw axis u than c: at the low grasp poses the real fixed finger stands this
  // much nearer the object than the arm model puts it, as the camera measures objects (the
  // camera fit hid it in the tip tape's offset). 2026-10-01 15:07:49, "Grab the can": the depth
  // camera saw the finger 1.5 to 2.1 cm past the CAD's toward the can, the gripper fitted to the
  // depth cloud 1.3 to 2.0 cm, and the descent with the pad planned 1.2 cm from the can pushed
  // the can 0.6 cm before the close squeezed it out. The plan judges the closing, and checks
  // every gripper part, with the object this much nearer the fixed finger (ArmPrism: where the
  // real gripper meets it) and also where the camera measured it (where the guard holds it).
  // To measure: run a can grasp to its GRASP POINT and measure the gap between the fixed pad
  // and the can with a caliper; the plan's gap is c (FixedJawClearanceCm), so set this to
  // itself + c - the measured gap (a can pushed on the way down: increase it).
  // 0 since the camera calibration of 2026-10-02 21:09: at the 2026-10-03 14:50:40 grasp pose
  // ("Grab the can") the depth camera saw the real gripper where the arm model puts it, the fixed
  // finger within 0.2 cm (0.14 cm FARTHER from the can along u), the can where it was measured
  // (its top's centre within 0.2 cm), so the 1.7 cm of 2026-10-01 went with that day's
  // calibration (refitted twice since): planned with it, the fixed pad stood 3.1 cm from the
  // can, the moving jaw pushed it 3 cm and squeezed it out past the fingertips. Re-measure after
  // a recalibration: a grasp pose's depth frame against the arm model (the gripper's points).
  FixedFingerOffsetCm: 0,
  // The real fixed finger may stand this much nearer than planned (the camera's and the arm's
  // error, 2026-10-01 up to 2.1 cm with that day's offset): a round object's sweep plan keeps the
  // fixed finger PlanMarginCm from it there too, on the printed descent and at the grasp pose.
  FixedFingerToleranceCm: 0.4,
  // THE STRADDLE (JawSweep StraddleOf): a standing round object's sweep plan (a can, a bottle, a
  // cup: the closing squeezes those out past the tips) puts it between the jaws deep enough that, at
  // the grasp pose before they close, the fixed finger reaches at least this far past its centre
  // along the fingers and the moving jaw, swinging closed, this far past it along its own length
  // (the jaws surround it); StraddleGoodCm deep where the palm leaves its plan margin, else as deep
  // as it does, then StraddleStepCm to either side while the closing does not hold (StraddleTries
  // closings). A round object the jaws cannot reach that far around at any pitch is refused, never
  // planned short. 2026-10-03 14:48: the can planned at the fixed pad's middle got 1.35 (fixed) and
  // 0.7 (moving). A box keeps the drift's head start (its flat sides grip at the pads; deep in the
  // jaws the moving jaw meets its corner, leaning).
  StraddleCm: 1.5,
  // False: a round object keeps the drift's head start too (the planner before 2026-10-03, for the replays).
  StraddleRoundObjects: true,
  StraddleStepCm: 0.5,
  StraddleTries: 3,
  // A sweep plan reaching this far around it (both jaws) is taken at once; short of it the planner looks on
  // among the pitches and raises it tries anyway, StraddleLookOn more at most, and takes the one that reaches
  // farthest (14:48's can: 1.6 at wrist -16.4 raised 2 cm, 2.0 at -8.8 raised 3).
  StraddleGoodCm: 2.0,
  StraddleLookOn: 4,
  // The NEXT STEP offers the close only with the tip where both jaws still reach this far past the
  // centre (the arm stops a few mm off the grasp point; the alignment's 1.5 cm away and 1 cm across
  // would leave a short grasp).
  StraddleCloseCm: 1.0,
  // A round object's sweep plan: where the closing push leaves its contact with the fixed
  // finger, at least this far inside the finger's tip (gripper z -10.43; the pad's middle is
  // 0.62 inside it), so at or behind the pad's middle, where the head start along the fingers
  // means it to end. The 15:05 plan left it 0.23 cm inside (the palm cut its head start to a
  // quarter of the push's drift) and the 15:08 close squeezed the can 3 cm out past the tips.
  // The push model called that close held and cannot tell such a close from a hold, so this is a
  // geometric rule kept with room for the arm's error along the fingers, not a friction verdict.
  FingertipMarginCm: 0.7,
  // REAL-ARM MEASUREMENT. A round object's sweep plan prints its GRASP POINT this much above where its jaws
  // close: at the low grasp poses the tip stops this far under a commanded point (each joint
  // stops within the bridge's tolerance on its low side under the arm's weight): 2026-10-01
  // 15:07:58 0.47 cm and 15:09:48 0.44 cm under the grasp point, which put the grasp 0.5 cm
  // above the can's mid-height back at mid-height, where the plan's own check said the closing
  // squeezes it out. To measure: the GRASP POINT's Z minus the tip's Z (the bridge's) where a
  // descent to it stopped, averaged over a few descents.
  GraspSagCm: 0.45,
  // PlanGrasp plans first with the strict closing (a round object's FingertipMarginCm and grip in
  // 3D within MaxLeanDeg; the 15:05 plan needed 21 deg in 3D; every sweep plan's closing judged at
  // its printed point), and only without a plan there as before. False: as before only.
  StrictClosing: true,
  // THE BEST-EFFORT PASS (2026-10-04, Aaran: "It won't grab the pill bottle, just let it try unless your code is totally sure
  // it can't grab it"). With no plan from the two passes above, PlanGrasp plans once more holding only what is certain:
  // the arm reaches the grasp and hover points (IK, joint limits), no CAD contact with the basket, cart or body, the desk
  // floor, the neighbours (the guard holds them too), an object thinner than the closed pads or one the open jaw cannot
  // clear even fully open. What only predicts a grip that may fail (the jaws not around it, the grip's friction, the push
  // toward the tips, the palm or a finger near it, the jaw table, a round object's assumed width) becomes the plan's
  // Warnings, which the model, the approval card and the session record are told; the user still approves every motion.
  // It tries the grasp point down to LowerCm under mid-height too (and as measured, not as a round one), bounded by
  // work, never by the clock, so the plan never depends on how busy the PC is (2026-10-07 audit M3: with the old 6 s
  // budget the tin planned at wrist -29 on an idle PC and -24, with four other warnings, under load). The work is
  // BestEffortWork placements, each counted as 1 + its neighbours / BestEffortPrismsPerPlacement (a measured scene's
  // obstacle boxes are neighbours: the 1201 of the 2026-10-06 cabinet made a placement 4 to 7 times a desk one), at
  // least BestEffortMinPlacements: about what the old 6 s covered on this PC when idle, so it plans what that planned
  // (BestEffortPlacements; measured 2026-10-07: the tin's whole search, 96 placements; TestSurfaceTasks' can merged with
  // its box, 10.3 x 9.7 cm, 112 of the 266 its round shape alone needs, refused; the cabinet 35, refused). The shapes in
  // turn as before. A placement's own cost is counted too (BestEffortSolveWork, in IK solves, the closing sweep's
  // vertex-edge pairs counting BestEffortSweepPairsPerSolve to a solve: together they predict a placement's time within
  // 1 to 13% on four scenes, 2026-10-08): a round neighbour beside the object makes each placement's closing sweep 16
  // times a desk one's, and 150 such placements took 60 s (the swing test's can 7 cm from a bottle), where the old 6 s
  // covered 50. 2000 is what that 6 s covered (the merged can 101 placements, the bottle's 56 to 60, the tin all 96).
  // BestEffortRefuseMs only refuses (no best-effort plan at all), never cuts the search short with whatever it found by
  // then. With the work bounded it is a backstop: the bottle's search now ends in 8 to 10 s, not at 60. It stays 60 s
  // so the plan stays the same on a PC several times slower than idle (TestBestEffortGuard: 8 times; at 15 s the tin's
  // 3 s search would already be refused there). False: refused as before.
  BestEffort: true,
  LowerCm: 3,
  BestEffortWork: 150,
  BestEffortPrismsPerPlacement: 400,
  BestEffortMinPlacements: 24,
  BestEffortSolveWork: 2000,
  BestEffortSweepPairsPerSolve: 3_150_000,
  BestEffortRefuseMs: 60000,
  // GRASP ANYTHING (2026-10-18): the best-effort pass's grasp heights, from this far above the object's base to this far
  // under its top, mid-height first, a step at a time either way. Its work bound (above) ends the search only once it has
  // a plan, and its clock returns the best plan found by then: neither refuses an object on its own.
  BestEffortFloorCm: 0.3,
  BestEffortTopCm: 0.5,
  BestEffortHeightStepCm: 1,
  BestEffortNoPlanWork: 2,
  // THE DEEP GRASP (2026-10-04, Aaran: "It needs to go like one inch farther into the object. In the CAD view I want the
  // wireframe of the grab object to be almost touching the servo of the gripper"). A round object's sweep plan (a
  // straddle) puts it as deep in the jaws as the palm and the STS3215 servo at the fixed jaw's base let it: its prism
  // PalmGapCm from their CAD hull (PalmGap: the hull's exact gap, GJK, not the samples' PlanMarginCm + DepthToleranceCm,
  // 1.0 cm), at the grasp pose and all the way down to it; then shallower StraddleStepCm at a time where the closing
  // does not hold. The printed points keep PalmGapCm + DeepRoundingCm (their 0.1 cm rounding) at the planned pose; the
  // guard holds the model's moves near it to GuardPalmGapCm. False: StraddleGoodCm deep, as before 2026-10-04.
  DeepGrasp: true,
  PalmGapCm: 0.25,
  DeepRoundingCm: 0.05,
  GuardPalmGapCm: 0.15,
  // The head starts a deep plan tries, deepest first, DeepStepCm apart (where the closing does not hold deeper).
  DeepTries: 7,
  DeepStepCm: 0.2,
  // A deep plan farther from the palm than PalmGapCm + DeepRoundingCm + this (the arm reaches no deeper at that wrist):
  // SweepScan looks on StraddleLookOn more pitches for a deeper one.
  DeepLookCm: 0.25,
};
// The tool point (the fixed pad's centre) in gripper_link: (-0.79, -0.02, -9.81).
const ToolJoint = CadModel.Joints.find(Joint => Joint.Name === "gripper_frame_joint")!;
const ToolInGripper: P2 = { x: ToolJoint.Origin[0][3], z: ToolJoint.Origin[2][3] };

// Report C 2e (CAD mesh, vertical jaw plane): far-face distance from the fixed
// pad -> the highest the object's top may stand above the tool along -a for
// the closing moving jaw to meet its side, not its top edge. Beyond 5.0: none.
export const JawWindow: [number, number][] = [[1.5, 6.11], [2.0, 5.2], [2.5, 3.3], [3.0, 2.2], [3.5, 1.9], [5.0, 1.8]];
export function MaxTopAboveTool(WidthCm: number): number | null {
  if (WidthCm <= JawWindow[0][0]) return JawWindow[0][1];
  for (let Index = 1; Index < JawWindow.length; Index++) {
    const [W0, H0] = JawWindow[Index - 1], [W1, H1] = JawWindow[Index];
    if (WidthCm <= W1) return H0 + (H1 - H0) * (WidthCm - W0) / (W1 - W0);
  }
  return null;
}
const MaxWindowCm = JawWindow[JawWindow.length - 1][0];
// The fixed pad (gripper_link, cm): x -0.79, y -0.53..0.49, z -10.35..-9.45 (GLB check in BuildGripperParts.py).
const PadCorners = [[-0.79, -0.53, -10.35], [-0.79, 0.49, -10.35], [-0.79, -0.53, -9.45], [-0.79, 0.49, -9.45]];

export type XY = { x: number; y: number };
// A measured object as a vertical prism (display frame). Phase 1: the
// axis-aligned size_cm box, padded. Role marks the task object's prism and a
// same-spot twin of it in a guard (exempt together while it is held or touched).
export type Prism = { Label: string; Footprint: XY[]; BaseZ: number; TopZ: number; SupportZ: number;
  Centre: XY; SizeCm: [number, number] | null; Role?: "task" | "twin";
  // Scene solids never get the task-contact or rising-out exemptions.
  Scene?: boolean;
  // A standing round object (GraspPrism): the diameters its true size may have, smallest to widest, and the best guess.
  // MergedCm: a drink can's side read this wide with something beside it (GraspRules.DrinkCanMergedMaxCm).
  Round?: { LoCm: number; HiCm: number; NominalCm: number; MergedCm?: number } | null };

const Dot2 = (A: XY, B: XY) => A.x * B.x + A.y * B.y;
const Dot3 = (A: Point3, B: Point3) => A.x * B.x + A.y * B.y + A.z * B.z;
const Sub3 = (A: Point3, B: Point3) => ({ x: A.x - B.x, y: A.y - B.y, z: A.z - B.z });
const Tenth = (Value: number) => Number(Value.toFixed(1));
const CeilTenth = (Value: number) => Math.ceil(Value * 10 - 1e-9) / 10;
const Fixed1 = (Value: number) => (Math.abs(Value) < 0.05 ? 0 : Value).toFixed(1);
const Deg = (Pitch: number) => Number(Pitch.toFixed(1)).toString();
const Clock = (At: number) => new Date(At).toTimeString().slice(0, 8);

/** The object as the planner sees it, or null without a depth-measured top and size. */
export function ObjectPrism(Object_: LocatedObject): Prism | null {
  if (!Object_.depthMeasured || Object_.topZ == null || !Object_.sizeCm) return null;
  const Hx = Object_.sizeCm[0] / 2 + GraspRules.TrimPadCm, Hy = Object_.sizeCm[1] / 2 + GraspRules.TrimPadCm;
  return { Label: Object_.label, BaseZ: Object_.z, TopZ: Object_.topZ, SupportZ: Object_.supportZMeasured ?? Object_.z,
    Centre: { x: Object_.x, y: Object_.y }, SizeCm: [Object_.sizeCm[0], Object_.sizeCm[1]],
    Footprint: [{ x: Object_.x - Hx, y: Object_.y - Hy }, { x: Object_.x + Hx, y: Object_.y - Hy },
      { x: Object_.x + Hx, y: Object_.y + Hy }, { x: Object_.x - Hx, y: Object_.y + Hy }] };
}

// Labels of objects that are round in plan when they stand up: their footprint is a disc.
const RoundWords = /\b(can|cans|soda|sodas|cola|coke|tin|tins|bottle|bottles|cup|cups|mug|mugs|jar|jars|glass|tumbler|flask|thermos|canister|tube|cylinder|vase)\b/i;
// A can (a drink can, a spray can, a soda): planned as one, 6.6 cm across.
const CanWords = /\b(can|cans|soda|sodas|cola|coke)\b/i;
/**
 * Whether a measured object is a standing round one (a can, a bottle, a cup):
 * its label says so and it stands at least 0.8 of its larger measured side
 * tall (a can lying down reads 12 x 6.6 and stands 6.6). Its footprint may
 * read long and thin: the 23:50 can, partly hidden by the arm, read 4.3 x 2.3.
 * Only its label can say it is round (or the task's own word for it, Kind: "grab
 * the can" bound to a "cylinder"): a square box measures the same, and its
 * corners are not a disc.
 */
export function RoundObject(Object_: LocatedObject, Kind: string | null = null): boolean {
  const Size = Object_.sizeCm, Height = Object_.heightCm ?? (Object_.topZ != null ? Object_.topZ - Object_.z : null);
  return (RoundWords.test(Object_.label) || (!!Kind && RoundWords.test(Kind))) && !!Size && Height != null && Height >= 0.8 * Math.max(...Size);
}
/**
 * The diameters a standing round object may have (GraspRules RoundMeasuredRatio):
 * from its larger measured side (Lo) to the widest it may be (Hi), and the best
 * guess (Nominal). The 5.0 x 4.5 can: 5.0 to 6.7, 6.7; read 5.6 x 5.0: 5.6 to
 * 6.8; a 2 x 2 tube: 2.0 to 3.0 (its padded box, as M1 planned it). A can read
 * wider than a can on one side only (5.3 x 7.7): a can, from its other side to
 * 6.8 (MergedCm: the wide side), never a 10.3 cm one refused as too wide.
 */
export function RoundSizes(Label: string, Size: [number, number]) {
  const Lo = Math.max(...Size), Narrow = Math.min(...Size);
  if (CanWords.test(Label) && Lo > GraspRules.DrinkCanMaxCm && Narrow <= GraspRules.DrinkCanMaxCm && Lo <= GraspRules.DrinkCanMergedMaxCm) {
    const From = Math.min(Narrow, GraspRules.DrinkCanCm);
    return { LoCm: From, HiCm: GraspRules.DrinkCanMaxCm,
      NominalCm: Number(Math.min(GraspRules.DrinkCanMaxCm, Math.max(From, From / GraspRules.RoundMeasuredRatio)).toFixed(2)), MergedCm: Lo };
  }
  const Widest = Math.max(Lo + 2 * GraspRules.TrimPadCm, Lo / GraspRules.RoundMeasuredRatio);
  const Hi = CanWords.test(Label) && Lo < GraspRules.DrinkCanMaxCm
    ? Math.min(Math.max(Widest, GraspRules.DrinkCanCm), GraspRules.DrinkCanMaxCm) : Widest;
  return { LoCm: Lo, HiCm: Number(Hi.toFixed(2)), NominalCm: Number(Math.min(Hi, Math.max(Lo, Lo / GraspRules.RoundMeasuredRatio)).toFixed(2)) };
}
// A disc's polygon around a centre, circumscribed (every side touches the circle from outside).
function Disc(Centre: XY, Diameter: number, Sides = GraspRules.RoundSides): XY[] {
  const Radius = Diameter / 2 / Math.cos(Math.PI / Sides);
  return Array.from({ length: Sides }, (_, Index) => ({ x: Centre.x + Radius * Math.cos(2 * Math.PI * Index / Sides), y: Centre.y + Radius * Math.sin(2 * Math.PI * Index / Sides) }));
}
/**
 * The prism the GRASP PLAN (and the guard) keeps the gripper clear of. A
 * standing round object measured narrower than it is (the 6.6 cm can read
 * 5.0 x 4.5): the disc of its widest plausible diameter (RoundSizes Hi),
 * which holds its measured extent both ways, so the fixed pad stays c outside
 * the true near face (a box's corners are not a can's: the padded box's put
 * the pad 1.2 cm from it, not 1.0). A small one, whose padded box is its
 * widest (a 2 cm tube, planned 3 cm across): that padded box, as M1 planned
 * it. Anything else: its padded box (ObjectPrism). Neighbours too: a can
 * beside the task object is as wide. Kind: the task's word for it (RoundObject).
 * A can read merged with something beside it (RoundSizes MergedCm): the can's
 * disc where its top was measured (the camera takes the centre from the top,
 * which a lower neighbour does not reach); the neighbour is measured as itself.
 */
export function GraspPrism(Object_: LocatedObject, Kind: string | null = null): Prism | null {
  const Box = ObjectPrism(Object_);
  if (!Box || !RoundObject(Object_, Kind)) return Box;
  const Sizes = RoundSizes(Kind && CanWords.test(Kind) ? Kind : Object_.label, Object_.sizeCm!);
  // The padding made it its widest (not a can's cap under it): a 6 x 6 can's padded box is 7 cm, its widest 6.8.
  const Padded = Math.abs(Sizes.HiCm - (Sizes.LoCm + 2 * GraspRules.TrimPadCm)) < 0.006;
  return { ...Box, Footprint: Padded ? Box.Footprint : Disc(Box.Centre, Sizes.HiCm), Round: Sizes };
}

/**
 * Where the real gripper meets an object the camera measured: moved OffsetCm
 * along -u (toward the fixed finger, GraspRules.FixedFingerOffsetCm), the
 * arm's frame at the grasp poses. Its Label, heights and sizes are kept.
 */
export function ArmPrism(Object_: Prism, Uh: XY, OffsetCm: number): Prism {
  if (!OffsetCm) return Object_;
  const Dx = -OffsetCm * Uh.x, Dy = -OffsetCm * Uh.y;
  return { ...Object_, Centre: { x: Object_.Centre.x + Dx, y: Object_.Centre.y + Dy }, Footprint: Object_.Footprint.map(P => ({ x: P.x + Dx, y: P.y + Dy })) };
}
/**
 * The real-arm corrections a plan gets (GraspRules FixedFingerOffsetCm, GraspSagCm,
 * and the fixed finger checked FixedFingerToleranceCm nearer): a standing round
 * object (a can) planned with the sweep, the case they were measured on
 * (2026-10-01 15:07). Anything else: none (planned as before). On: which.
 */
export function RealArmCorrections(Object_: Prism, Sweep: boolean) {
  const On = Sweep && !!Object_.Round;
  return { On, OffsetCm: On ? GraspRules.FixedFingerOffsetCm : 0, SagCm: On ? GraspRules.GraspSagCm : 0 };
}
/** A plan's object where the real gripper meets it (ArmPrism with the plan's offset), or null for a plan without one. */
export function PlanArmPrism(Plan: GraspPlan): Prism | null {
  return Plan.FingerOffsetCm ? ArmPrism(Plan.Prism, Plan.JawAxis, Plan.FingerOffsetCm) : null;
}

/** Signed distance from a point to a convex polygon (negative inside), any winding; one vertex is a point. */
export function FootprintDistance(Footprint: XY[], Point: XY): number {
  if (Footprint.length === 1) return Math.hypot(Point.x - Footprint[0].x, Point.y - Footprint[0].y);
  let Inside = true, Best = Infinity, Sign = 0;
  for (let Index = 0; Index < Footprint.length; Index++) {
    const A = Footprint[Index], B = Footprint[(Index + 1) % Footprint.length];
    const Ex = B.x - A.x, Ey = B.y - A.y, Length2 = Ex * Ex + Ey * Ey;
    const T = Length2 > 0 ? Math.max(0, Math.min(1, ((Point.x - A.x) * Ex + (Point.y - A.y) * Ey) / Length2)) : 0;
    Best = Math.min(Best, Math.hypot(Point.x - A.x - T * Ex, Point.y - A.y - T * Ey));
    const Cross = Ex * (Point.y - A.y) - Ey * (Point.x - A.x);
    if (Cross !== 0) { if (Sign === 0) Sign = Math.sign(Cross); else if (Math.sign(Cross) !== Sign) Inside = false; }
  }
  return Inside ? -Best : Best;
}

// The polygon clipped to Lo <= p.Normal <= Hi (two half-plane clips).
function ClipBand(Polygon: XY[], Normal: XY, Lo: number, Hi: number): XY[] {
  const Clip = (Points: XY[], Keep: (Point: XY) => number) => {
    const Out: XY[] = [];
    for (let Index = 0; Index < Points.length; Index++) {
      const P = Points[Index], Q = Points[(Index + 1) % Points.length], Kp = Keep(P), Kq = Keep(Q);
      if (Kp >= 0) Out.push(P);
      if ((Kp >= 0) !== (Kq >= 0)) { const T = Kp / (Kp - Kq); Out.push({ x: P.x + T * (Q.x - P.x), y: P.y + T * (Q.y - P.y) }); }
    }
    return Out;
  };
  return Clip(Clip(Polygon, Point => Dot2(Point, Normal) - Lo), Point => Hi - Dot2(Point, Normal));
}

// The prism cut by the plane through T with normal N: where its 3n edges cross it.
function Slice(Object_: Prism, T: Point3, Normal: Point3): Point3[] {
  const Bottom = Object_.Footprint.map(P => ({ x: P.x, y: P.y, z: Object_.BaseZ })), Top = Object_.Footprint.map(P => ({ x: P.x, y: P.y, z: Object_.TopZ }));
  const Edges: [Point3, Point3][] = [];
  for (let Index = 0; Index < Bottom.length; Index++) {
    const Next = (Index + 1) % Bottom.length;
    Edges.push([Bottom[Index], Bottom[Next]], [Top[Index], Top[Next]], [Bottom[Index], Top[Index]]);
  }
  const Out: Point3[] = [];
  for (const [P, Q] of Edges) {
    const Sp = Dot3(Sub3(P, T), Normal), Sq = Dot3(Sub3(Q, T), Normal);
    if (Sp === 0) Out.push(P);
    else if (Sp * Sq < 0) { const K = Sp / (Sp - Sq); Out.push({ x: P.x + K * (Q.x - P.x), y: P.y + K * (Q.y - P.y), z: P.z + K * (Q.z - P.z) }); }
  }
  return Out;
}
// How far the slice in the jaw plane stands above the tool point along the gripper axis (-a).
function TopAboveTool(Object_: Prism, T: Point3, Hinge: Point3, Approach: Point3) {
  const Cut = Slice(Object_, T, Hinge);
  return Cut.length ? Math.max(...Cut.map(Q => -Dot3(Sub3(Q, T), Approach))) : -Infinity;
}

// --- The gripper's parts in the display frame ------------------------------------

export type GripperPart = "FixedFinger" | "PalmAndServo" | "MovingJaw";
type PartData = { Link: string; Samples: number[][]; CoverRadiusCm: number; DecimationLossCm: number };
const PartList = Object.entries(Parts.Parts) as [GripperPart, PartData][];
const JawPart = PartList.find(([Name]) => Name === "MovingJaw")![1];
export const GripperPartsData = Parts;
function ToDisplay(Matrix: Matrix4, Point: number[]): Point3 {
  const E = Matrix.elements;
  return mirrorDisplayPoint({ x: E[0] * Point[0] + E[4] * Point[1] + E[8] * Point[2] + E[12],
    y: E[1] * Point[0] + E[5] * Point[1] + E[9] * Point[2] + E[13], z: E[2] * Point[0] + E[6] * Point[1] + E[10] * Point[2] + E[14] });
}
export type GripperSample = { Part: GripperPart; P: Point3 };
// The palm and servo's CAD hull, and the STS3215 servo's own (gripper_link, cm: BuildGripperParts.py Hulls), and how
// far their 0.001 cm rounding may put a vertex inside the true one.
const PalmHull = (Parts.Parts.PalmAndServo as { Hull: number[][] }).Hull;
const ServoHull = (Parts as { ServoHull: number[][] }).ServoHull;
const HullRoundingCm = 0.001;
// A hull's support map at a pose (display frame), moved by Shift: the display map is affine, so a direction is turned
// into the link frame once and the hull's vertices are searched there.
function HullSupport(Matrix: Matrix4, Hull: number[][], Shift: Point3 | null = null): Support {
  const O = ToDisplay(Matrix, [0, 0, 0]), X = Sub3(ToDisplay(Matrix, [1, 0, 0]), O), Y = Sub3(ToDisplay(Matrix, [0, 1, 0]), O), Z = Sub3(ToDisplay(Matrix, [0, 0, 1]), O);
  if (Shift) { O.x += Shift.x; O.y += Shift.y; O.z += Shift.z; }
  return D => {
    const Dx = Dot3(D, X), Dy = Dot3(D, Y), Dz = Dot3(D, Z);
    let Best = Hull[0], Most = -Infinity;
    for (const P of Hull) { const V = Dx * P[0] + Dy * P[1] + Dz * P[2]; if (V > Most) { Most = V; Best = P; } }
    return { x: O.x + Best[0] * X.x + Best[1] * Y.x + Best[2] * Z.x, y: O.y + Best[0] * X.y + Best[1] * Y.y + Best[2] * Z.y, z: O.z + Best[0] * X.z + Best[1] * Y.z + Best[2] * Z.z };
  };
}
function PrismSupport(Object_: Prism): Support {
  return D => {
    let Best = Object_.Footprint[0], Most = -Infinity;
    for (const P of Object_.Footprint) { const V = D.x * P.x + D.y * P.y; if (V > Most) { Most = V; Best = P; } }
    return { x: Best.x, y: Best.y, z: D.z >= 0 ? Object_.TopZ : Object_.BaseZ };
  };
}
/**
 * THE DEEP GRASP's measure (GraspRules.DeepGrasp): the gap (cm) between a prism and the gripper's palm and
 * servo, their CAD hull (Servo: the STS3215's alone), at a pose's links, the gripper moved rigidly by Shift.
 * Exact (GJK) up to 0.001 cm, from below; 0 where they meet.
 */
export function PalmGap(Links: Record<string, Matrix4>, Object_: Prism, Servo = false, Shift: Point3 | null = null): number {
  const Hull = HullSupport(Links.gripper_link, Servo ? ServoHull : PalmHull, Shift);
  const Toward = Sub3({ ...Object_.Centre, z: (Object_.BaseZ + Object_.TopZ) / 2 }, Hull({ x: 0, y: 0, z: 0 }));
  return Math.max(0, ConvexGap(Hull, PrismSupport(Object_), Toward, 1e-3) - HullRoundingCm);
}
/** The least PalmGap to any of the prisms (Infinity: none). */
const PalmGapTo = (Links: Record<string, Matrix4>, Prisms: Prism[], Shift: Point3 | null = null) =>
  Math.min(Infinity, ...Prisms.map(Object_ => PalmGap(Links, Object_, false, Shift)));
/** The least PalmGap to the prisms along the joint-linear way From -> To (PathSamples + 1 poses) and where (s), or null where none. */
function PalmPathGap(From: CadAngles, To: CadAngles, Prisms: Prism[], Samples = GraspRules.PathSamples): { GapCm: number; S: number } | null {
  if (!Prisms.length) return null;
  let Best: { GapCm: number; S: number } | null = null;
  for (let Index = 0; Index <= Samples; Index++) {
    const S_ = Index / Samples, Angles: CadAngles = { ...To };
    for (const Name of ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex"]) Angles[Name] = From[Name] + S_ * (To[Name] - From[Name]);
    const GapCm = PalmGapTo(CadForward(Angles).Links, Prisms);
    if (!Best || GapCm < Best.GapCm) Best = { GapCm, S: S_ };
  }
  return Best;
}
// A deep plan's own object: its palm samples are judged by PalmGap instead (DeepestContact skips a part at -Infinity).
const DeepTaskMargins: PartMargins = { PalmAndServo: -Infinity };
/**
 * Where the joint-linear way From -> To brings the palm and servo within NeedCm of one of a deep plan's own
 * prisms (PalmGap), as a PathHit against Margin (DepthCm = Margin - the gap, so a message's "within" reads the gap).
 */
function PalmPathHit(From: CadAngles, To: CadAngles, Prisms: Prism[], NeedCm: number, Margin: number): PathHit | null {
  if (!Prisms.length) return null;
  let First = Infinity, Worst: { GapCm: number; S: number; Prism: Prism } | null = null;
  for (let Index = 0; Index <= GraspRules.PathSamples; Index++) {
    const S_ = Index / GraspRules.PathSamples, Angles: CadAngles = { ...To };
    for (const Name of ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex"]) Angles[Name] = From[Name] + S_ * (To[Name] - From[Name]);
    const Links = CadForward(Angles).Links;
    for (const Object_ of Prisms) {
      const GapCm = PalmGap(Links, Object_);
      if (GapCm >= NeedCm) continue;
      First = Math.min(First, S_);
      if (!Worst || GapCm < Worst.GapCm) Worst = { GapCm, S: S_, Prism: Object_ };
    }
  }
  if (!Worst) return null;
  return { Part: "PalmAndServo", Label: Worst.Prism.Label, DepthCm: Margin - Worst.GapCm, At: { ...Worst.Prism.Centre, z: Worst.Prism.TopZ }, Prism: Worst.Prism, S: Worst.S, FirstS: First };
}
/** Every part sample at a pose (display frame); the moving jaw once per angle in JawDegs. */
export function GripperSamples(Angles: CadAngles, JawDegs: number[]): GripperSample[] {
  const Out: GripperSample[] = [];
  const Links = CadForward(Angles).Links;
  for (const [Name, Part] of PartList) {
    if (Name === "MovingJaw") continue;
    for (const Point of Part.Samples) Out.push({ Part: Name, P: ToDisplay(Links[Part.Link], Point) });
  }
  for (const Deg_ of [...new Set(JawDegs)]) {
    const Jaw = CadForward({ ...Angles, gripper: Deg_ }).Links[JawPart.Link];
    for (const Point of JawPart.Samples) Out.push({ Part: "MovingJaw", P: ToDisplay(Jaw, Point) });
  }
  return Out;
}
// The moving jaw at its angle and the real jaw's ~11 deg more closed, never past where the pads meet.
const JawPair = (Deg_: number) => [Deg_, Math.max(Deg_ - GraspRules.JawUncertaintyDeg, GraspRules.PadsMeetDeg)];
/**
 * A sweep plan's palm keeps DepthToleranceCm more from the object it grasps:
 * at a flat wrist the palm comes down just behind the object (toward the
 * arm), and the closing was checked with the object that far off along the
 * fingers, so the palm must clear it there too (the 16:49 can's descent
 * passed 0.6 cm from it).
 */
const SweepTaskMargins = (Margin: number): PartMargins => ({ PalmAndServo: Margin + GraspRules.DepthToleranceCm });
/**
 * The moving jaw angles a plan's open fraction is checked at: the steps take
 * the gripper within OpenToleranceFraction of it, so its widest there, the
 * fraction itself, and the real jaw's 11 deg more closed than its narrowest.
 * The guard checks the gripper where it is, and finds what the plan checked.
 */
export function PlanJawDegs(Servos: Record<ServoName, ServoState>, Fraction: number): number[] {
  const Tolerance = GraspRules.OpenToleranceFraction;
  return [OpenDeg(Servos, Math.min(Fraction + Tolerance, 1)), OpenDeg(Servos, Fraction),
    Math.max(OpenDeg(Servos, Fraction - Tolerance) - GraspRules.JawUncertaintyDeg, GraspRules.PadsMeetDeg)];
}
// Each part's link-frame box corners, for a cheap overlap test before its samples.
const PartCorners = PartList.map(([Name, Part]) => {
  const [Low, High] = (Parts.Parts[Name] as { Bounds: number[][] }).Bounds;
  return { Name, Part, Corners: [0, 1, 2, 3, 4, 5, 6, 7].map(Bits => [Bits & 1 ? High[0] : Low[0], Bits & 2 ? High[1] : Low[1], Bits & 4 ? High[2] : Low[2]]) };
});
/**
 * Whether the gripper at a pose (its links, the moving jaw at its own angle)
 * keeps every part Margin clear of the prisms: the parts' boxes first, the
 * samples only where a box comes near a prism. For the snap's searches, which
 * evaluate thousands of poses; the full guard checks the pose they pick.
 */
export function GripperPoseClear(Links: Record<string, Matrix4>, Prisms: Prism[], Margin: number, Deep: { Tasks: Prism[]; GapCm: number } | null = null): boolean {
  if (!Prisms.length) return true;
  // A deep plan's own prisms (GraspRules.DeepGrasp): the palm and servo by their exact gap, not their samples.
  if (Deep?.Tasks.length && PalmGapTo(Links, Deep.Tasks) < Deep.GapCm) return false;
  const Grown = Prisms.map(Object_ => {
    const Xs = Object_.Footprint.map(P => P.x), Ys = Object_.Footprint.map(P => P.y);
    return { Object_, LoX: Math.min(...Xs) - Margin, HiX: Math.max(...Xs) + Margin, LoY: Math.min(...Ys) - Margin, HiY: Math.max(...Ys) + Margin,
      LoZ: Object_.BaseZ - Margin, HiZ: Object_.TopZ + Margin };
  });
  for (const { Name, Part, Corners } of PartCorners) {
    const Points = Corners.map(Corner => ToDisplay(Links[Part.Link], Corner));
    const Box = { LoX: Math.min(...Points.map(P => P.x)), HiX: Math.max(...Points.map(P => P.x)), LoY: Math.min(...Points.map(P => P.y)),
      HiY: Math.max(...Points.map(P => P.y)), LoZ: Math.min(...Points.map(P => P.z)), HiZ: Math.max(...Points.map(P => P.z)) };
    const Near = Grown.filter(G => G.LoX <= Box.HiX && G.HiX >= Box.LoX && G.LoY <= Box.HiY && G.HiY >= Box.LoY && G.LoZ <= Box.HiZ && G.HiZ >= Box.LoZ);
    if (!Near.length) continue;
    const Samples = Part.Samples.map(Point => ({ Part: Name, P: ToDisplay(Links[Part.Link], Point) }));
    if (DeepestContact(Samples, Near.map(G => G.Object_), Margin, Deep ? DeepTaskMargins : null, Deep?.Tasks ?? null)) return false;
  }
  return true;
}

export type Contact = { Part: GripperPart; Label: string; DepthCm: number; At: Point3; Prism: Prism };
// A part's own margin from the task object (the task prism, Role "task" or the plan's): see TaskMargins.
export type PartMargins = Partial<Record<GripperPart, number>>;
/**
 * The deepest sample within Margin of any prism (null: all clear). DepthCm =
 * Margin - distance (> Margin: inside). TaskMargins: a part's own margin from
 * the prism TaskPrism (the object the plan is for; or each of several: where
 * the camera measured it and where the real gripper meets it), instead of Margin.
 */
export function DeepestContact(Samples: GripperSample[], Prisms: Prism[], Margin: number, TaskMargins: PartMargins | null = null,
  TaskPrism: Prism | Prism[] | null = null): Contact | null {
  let Worst: Contact | null = null;
  for (const Object_ of Prisms) {
    const Own = TaskMargins && (Array.isArray(TaskPrism) ? TaskPrism.includes(Object_) : Object_ === TaskPrism) ? TaskMargins : null;
    const Reach = Math.max(Margin, ...Object.values(Own ?? {}));
    const Xs = Object_.Footprint.map(P => P.x), Ys = Object_.Footprint.map(P => P.y);
    const LoX = Math.min(...Xs) - Reach, HiX = Math.max(...Xs) + Reach, LoY = Math.min(...Ys) - Reach, HiY = Math.max(...Ys) + Reach;
    for (const Sample of Samples) {
      const P = Sample.P;
      if (P.x < LoX || P.x > HiX || P.y < LoY || P.y > HiY || P.z < Object_.BaseZ - Reach || P.z > Object_.TopZ + Reach) continue;
      const Flat = FootprintDistance(Object_.Footprint, P);
      const Dz = P.z > Object_.TopZ ? P.z - Object_.TopZ : P.z < Object_.BaseZ ? Object_.BaseZ - P.z : -Math.min(Object_.TopZ - P.z, P.z - Object_.BaseZ);
      const Distance = Flat > 0 && Dz > 0 ? Math.hypot(Flat, Dz) : Math.max(Flat, Dz);
      const Limit = Own?.[Sample.Part] ?? Margin;
      if (Distance < Limit && (!Worst || Limit - Distance > Worst.DepthCm)) {
        Worst = { Part: Sample.Part, Label: Object_.Label, DepthCm: Limit - Distance, At: P, Prism: Object_ };
      }
    }
  }
  return Worst;
}

export type PathHit = Contact & { S: number; FirstS: number };
/**
 * The deepest contact along a joint-linear move (s = 0.1 .. 1.0 on the four IK
 * joints, roll kept), the moving jaw at JawDeg and the real jaw's ~11 deg more
 * closed, and where the first contact was. Null: clear all the way.
 */
export function PathContact(From: CadAngles, To: CadAngles, JawDeg: number, Prisms: Prism[], Margin: number,
  Samples = GraspRules.PathSamples, JawDegs = JawPair(JawDeg), TaskMargins: PartMargins | null = null, TaskPrism: Prism | Prism[] | null = null): PathHit | null {
  if (!Prisms.length) return null;
  let Worst: PathHit | null = null, First = Infinity;
  for (let Index = 1; Index <= Samples; Index++) {
    const S = Index / Samples;
    const Angles: CadAngles = { ...From };
    for (const Name of ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex"]) Angles[Name] = From[Name] + S * (To[Name] - From[Name]);
    Angles.gripper = JawDeg;
    const Hit = DeepestContact(GripperSamples(Angles, JawDegs), Prisms, Margin, TaskMargins, TaskPrism);
    if (Hit) First = Math.min(First, S);
    if (Hit && (!Worst || Hit.DepthCm > Worst.DepthCm)) Worst = { ...Hit, S, FirstS: First };
  }
  if (Worst) Worst.FirstS = First;
  return Worst;
}

// --- The planner -------------------------------------------------------------------

export type GraspContext = { Servos: Record<ServoName, ServoState>; ElbowMode: "up" | "down"; Limits?: IkAngleLimits | null;
  // Neighbours: every other measured object the gripper must keep clear of.
  Others: Prism[];
  // The steepest wrist pitch that reaches a model-frame point (AiMeasurements ReachScan), or null.
  Scan: (ModelPoint: Point3) => number | null;
  // The gap c the fixed pad goes outside the object: the user's grasp gap setting (GraspGap.ts).
  // Absent: GraspRules.FixedJawClearanceCm. Every check runs as before with it (ClearanceFor).
  ClearanceCm?: number | null;
  // The task's floor (AiTableFloor.ts: the surface the object stands on) the printed points are held
  // to, as the snap and the page hold the model's targets. Absent: the desk's (DeskFloor).
  Floor?: AiFloor | null };

/**
 * A raised surface the task object stands on (a platform, a box top) as a prism the gripper keeps
 * clear of on every joint-linear way (the guard's, the plan's descent), its top 1 cm under the
 * surface like the floor: the floor holds targets, this holds the way to them over its extent. None
 * for the desk, nor for a surface below it (its floor holds each gripper point over it).
 */
export function FloorPrism(Floor: AiFloor | null | undefined): Prism | null {
  const Surface = Floor?.Surface;
  if (!Floor || !Surface || Floor.SurfaceZ < TableZCm) return null;
  const [[X0, X1], [Y0, Y1]] = [Surface.XRangeCm, Surface.YRangeCm];
  // The refusals say "the <Label> measured at X, Y": "the platform "box platform" under the task object (to 1 cm under its top, Z 11.7)".
  return { Label: `${Surface.Kind} "${Surface.Name}" under the task object (to 1 cm under its top, Z ${Floor.FloorZ.toFixed(1)})`, BaseZ: TableZCm, TopZ: Floor.FloorZ, SupportZ: TableZCm,
    Centre: { x: (X0 + X1) / 2, y: (Y0 + Y1) / 2 }, SizeCm: [X1 - X0, Y1 - Y0],
    Footprint: [{ x: X0, y: Y0 }, { x: X1, y: Y0 }, { x: X1, y: Y1 }, { x: X0, y: Y1 }] };
}
/** Every observed board, back, side or neighbour remains a solid on gripper paths and jaw sweeps. */
export function ScenePrisms(Floor: AiFloor | null | undefined): Prism[] {
  return (Floor?.Scene?.obstacles ?? []).map(({ label, min: [X0, Y0, Z0], max: [X1, Y1, Z1] }) => ({
    Label: label, BaseZ: Z0, TopZ: Z1, SupportZ: Z0, Scene: true,
    Centre: { x: (X0 + X1) / 2, y: (Y0 + Y1) / 2 }, SizeCm: [X1 - X0, Y1 - Y0],
    Footprint: [{ x: X0, y: Y0 }, { x: X1, y: Y0 }, { x: X1, y: Y1 }, { x: X0, y: Y1 }],
  }));
}
/**
 * A held object's box stays attached to the wrist, including pan rotation between waypoints.
 * This conservative sampled preview is additional to the bridge's independent-progress route certificate.
 * The support may touch its bottom at pickup; penetration and contact with all other solids still fail.
 */
export function CarriedObjectPathReason(Object_: Prism, AttachedAt: CadAngles, From: CadAngles, To: CadAngles, Obstacles: Prism[], Floor?: AiFloor | null): string | null {
  const Inverse = CadForward(AttachedAt).Links.gripper_link.clone().invert();
  const Local = Object_.Footprint.flatMap(P => [Object_.BaseZ, Object_.TopZ].map(z => {
    const M = modelPointFromDisplayPoint({ ...P, z });
    return new Vector3(M.x, M.y, M.z).applyMatrix4(Inverse);
  }));
  for (let Index = 0; Index <= GraspRules.PathSamples; Index++) {
    const S = Index / GraspRules.PathSamples, Angles: CadAngles = { ...From };
    for (const Name of ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]) Angles[Name] = From[Name] + S * (To[Name] - From[Name]);
    const Matrix = CadForward(Angles).Links.gripper_link;
    const Points = Local.map(P => mirrorDisplayPoint(P.clone().applyMatrix4(Matrix)));
    const Low = ["x", "y", "z"].map(Axis => Math.min(...Points.map(P => P[Axis as keyof Point3])));
    const High = ["x", "y", "z"].map(Axis => Math.max(...Points.map(P => P[Axis as keyof Point3])));
    if (Floor?.Scene && !ObservedBoxCovered({ min: Low as SceneBox["min"], max: High as SceneBox["max"] }, KnownSceneSpace(Floor))) {
      return `the held ${Object_.Label} would enter space the depth camera has not observed`;
    }
    for (const Obstacle of Obstacles) {
      // A scene cell at the support's level (its observed top no more than SupportBandCm over the object's base: the
      // support's tilt, its edge) is where the object stands, as the support is: its observed cell, not its padding,
      // must stay clear. Padded, the cabinet top's front edge beside the pill bottle (2026-10-06 15:01, 1 cm under its
      // base) "touched" the held bottle before it moved.
      const Level = !!Obstacle.Scene && Obstacle.TopZ - SceneRules.DepthUncertaintyCm <= Object_.BaseZ + SceneRules.SupportBandCm;
      const Pad = Level ? SceneRules.DepthUncertaintyCm : 0;
      const Min = [Math.min(...Obstacle.Footprint.map(P => P.x)) + Pad, Math.min(...Obstacle.Footprint.map(P => P.y)) + Pad, Obstacle.BaseZ + Pad];
      const Max = [Math.max(...Obstacle.Footprint.map(P => P.x)) - Pad, Math.max(...Obstacle.Footprint.map(P => P.y)) - Pad, Obstacle.TopZ - Pad];
      const Supporting = Level || Obstacle.TopZ <= Object_.BaseZ + 0.05;
      const Margin = Supporting ? 0 : GraspRules.ContactMarginCm;
      if (Low.every((Value, Axis) => Value < Max[Axis] + Margin - 1e-6 && High[Axis] > Min[Axis] - Margin + 1e-6)) {
        return `the held ${Object_.Label} would touch ${Obstacle.Label} at ${Math.round(S * 100)}% of the way there`;
      }
    }
  }
  return null;
}
// The scene's boxes are padded by its depth uncertainty (back end/SceneGeometry.py DepthUncertaintyCm): the task
// object's envelope, and every obstacle's 1 cm cells. A cell no higher than SupportBandCm over the object's base is
// the support's level: a tilted support rises that much beside it (the cabinet top of 2026-10-06 15:01, 5 deg in the
// arm's frame, rose to 4.5-5.0 cm at the pill bottle's far side, its base at 4.1).
export const SceneRules = { DepthUncertaintyCm: 0.5, SupportBandCm: 1.0 };
/**
 * An observed solid inside the task object as the camera measured it, or null: the object and its surroundings
 * were not measured apart (a shelf board, a neighbour's side merged into its measurement). The bridge measures it
 * on the points (scene task_contact: solid touching the object's measured footprint, null where none). A scene
 * without it (an older bridge): the object's envelope without its padding (its measured points) against each
 * obstacle's observed cell without its own, the support's level skipped: under the object nothing but the support
 * stands. Padded both, the pill bottle on the cabinet of 2026-10-06 15:01 met the cabinet top's cells under its
 * base, and every grasp was refused as "overlaps"; unpadded, the 1 cm cells still met its own rim.
 */
export function SceneConflict(Scene: MeasuredScene | null | undefined): { Box: MeasuredScene["obstacles"][number]; Low: number[]; High: number[] } | null {
  const Task = Scene?.task_object;
  if (!Scene || !Task) return null;
  if (Scene.task_contact !== undefined) {
    const Contact = Scene.task_contact;
    return Contact ? { Box: { min: Contact.min, max: Contact.max, label: "observed environment" }, Low: [...Contact.min], High: [...Contact.max] } : null;
  }
  const Pad = SceneRules.DepthUncertaintyCm;
  const TaskLow = [Task.min[0] + Pad, Task.min[1] + Pad, Task.min[2]], TaskHigh = [Task.max[0] - Pad, Task.max[1] - Pad, Task.max[2] - Pad];
  for (const Box of Scene.obstacles) {
    const Low = Box.min.map(Value => Value + Pad), High = Box.max.map(Value => Value - Pad);
    if (High[2] <= TaskLow[2] + SceneRules.SupportBandCm + 1e-6) continue;
    if (Low.every((Value, Axis) => Value < TaskHigh[Axis] - 1e-6 && High[Axis] > TaskLow[Axis] + 1e-6)) return { Box, Low, High };
  }
  return null;
}
// Observed free space, the scene's known space (the arm where it stood; the task object's reach, unobserved and taken as
// empty: the bridge's SceneGeometry.TaskReach), and the object itself.
const KnownSceneSpace = (Floor: AiFloor) => [...(Floor.Scene?.free_space ?? []), ...(Floor.Scene?.known_space ?? []),
  ...(Floor.Scene?.task_object ? [Floor.Scene.task_object] : [])];
/** Gripper-part boxes along the preview must be covered by real free-space evidence, not merely absent obstacles. */
export function ScenePathCoverageReason(From: CadAngles, To: CadAngles, Floor?: AiFloor | null, JawDegs?: number[]): string | null {
  if (!Floor?.Scene) return null;
  const Known = KnownSceneSpace(Floor);
  if (!Floor.Scene.free_space?.length) return "the depth camera has not observed a free approach corridor";
  for (let Index = 0; Index <= GraspRules.PathSamples; Index++) {
    const S = Index / GraspRules.PathSamples, Angles: CadAngles = { ...From };
    for (const Name of ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]) Angles[Name] = From[Name] + S * (To[Name] - From[Name]);
    for (const Jaw of JawDegs ?? [From.gripper ?? 0]) {
      const Links = CadForward({ ...Angles, gripper: Jaw }).Links;
      for (const { Part, Corners } of PartCorners) {
        const Points = Corners.map(P => ToDisplay(Links[Part.Link], P));
        const Bounds: SceneBox = {
          min: ["x", "y", "z"].map(Axis => Math.min(...Points.map(P => P[Axis as keyof Point3])) - GraspRules.ContactMarginCm) as SceneBox["min"],
          max: ["x", "y", "z"].map(Axis => Math.max(...Points.map(P => P[Axis as keyof Point3])) + GraspRules.ContactMarginCm) as SceneBox["max"],
        };
        if (!ObservedBoxCovered(Bounds, Known)) return "the gripper's path enters space the depth camera has not observed";
      }
    }
  }
  return null;
}
/** The grasp gap the user set (GraspContext.ClearanceCm), or the default. */
const GapSetOf = (Context: { ClearanceCm?: number | null }) => Context.ClearanceCm ?? GraspRules.FixedJawClearanceCm;
/**
 * The gap c a plan puts the fixed pad at: the user's grasp gap, but never below
 * FixedJawClearanceCm on a plan without the real-finger correction
 * (RealArmCorrections On false: anything but a round object's sweep plan).
 * Only that correction moves the pad out for where the real fixed finger
 * stands (FixedFingerOffsetCm) and checks the way down with it
 * FixedFingerToleranceCm nearer; on any other plan the default c is the only
 * room left for the real finger, which the camera saw 1.5 to 2.1 cm nearer the
 * object than the arm model at the low grasp poses (2026-10-01's calibration).
 */
export function ClearanceFor(Context: { ClearanceCm?: number | null }, Corrected: boolean): number {
  const Gap = GapSetOf(Context);
  return Corrected ? Gap : Math.max(Gap, GraspRules.FixedJawClearanceCm);
}
// clear: raised above mid-height, where a check failed (PlanGrasp RaisedPlan).
export type HeightBy = "mid" | "floor" | "palm" | "jaw" | "clear";
// How the closing was judged: the CAD jaw table (objects up to 5 cm from the
// fixed pad, M1), or the jaw-plane sweep (JawSweep.ts, M3): the jaw's push on
// the nominal shape where it was measured (the design's lean), the most
// friction angle the two-pad grip needs over every size and depth it may be,
// where the jaw first meets it (CAD deg), and for a round object the
// diameters checked.
export type GraspClosing = { Mode: "table" }
  | { Mode: "sweep"; PushDeg: number; GripDeg: number; Grip3DDeg: number | null; ContactDeg: number;
    // How far the jaw's push carries it toward the finger tips, and how much nearer the palm than the fixed
    // pad's middle the grasp point puts its centre (the straddle's head start, GraspRules.StraddleCm).
    DriftCm: number; DepthOffsetCm: number; DiameterCm: [number, number] | null;
    // A round object: how far inside the fixed finger's tip the push leaves its contact (GraspRules.FingertipMarginCm).
    TipCm?: number | null };
export type GraspPlan = { Prism: Prism; Pitch: number; HoverPitch: number; Grasp: Point3; Hover: Point3;
  // A shelf's pregrasp is beside it. Closing is a separate check-in; release and retreat keep this wrist.
  Approach?: { Kind: "side"; Pregrasp: Point3; Release: Point3; Retreat: Point3; Pitch: number };
  // u-hat: from the fixed pad into the gap (toward the object), in the display XY plane.
  JawAxis: XY;
  Side: string; OffsetCm: number; WidthEffCm: number;
  // |ux| sx + |uy| sy of the unpadded box: how wide the jaws find it.
  WidthAcrossCm: number;
  HeightBy: HeightBy; TopAboveToolCm: number;
  // HeightBy "clear": how far above its mid-height, and what failed at mid-height there.
  RaiseCm?: number; RaisedFor?: string | null;
  // The gripper fraction the steps open to: 0.8 (M1), or the smallest of 0.5/0.6/0.8 that clears the object (M3).
  OpenFraction: number; Closing: GraspClosing; Ms: number;
  // The poses the printed points were checked at (the gripper at OpenFraction) and the jaw angles
  // checked there (PlanJawDegs): the plan lock checks a re-measure against the descent between them.
  Poses?: { Hover: CadAngles; Grasp: CadAngles; JawDegs: number[] } | null;
  // Moved for the tip tape the camera saw at the hover (TapeCorrection), once.
  Tape?: TapeFix | null;
  // A round object's sweep plan (RealArmCorrections): how much farther from it along u the pad
  // goes for the real finger (GraspRules.FixedFingerOffsetCm), and how far the printed grasp point
  // is above where the jaws close (GraspRules.GraspSagCm). Absent: neither.
  FingerOffsetCm?: number; SagCm?: number;
  // The gap c the fixed pad was planned at outside the object (the user's grasp gap setting,
  // GraspContext.ClearanceCm, as ClearanceFor allows it). Absent (a plan from before the setting):
  // FixedJawClearanceCm. GapSetCm: the user's setting, only where it is not ClearanceCm (a gap below
  // the default on a plan without the real-finger correction, which keeps the default).
  ClearanceCm?: number; GapSetCm?: number;
  // A sweep plan's straddle where its jaws close (JawSweep StraddleOf, GraspRules.StraddleCm): how far
  // the fixed finger and the moving jaw reach past the object's centre, where that centre is in the
  // jaws' plane (gripper x, z), and the level part of the fingers' direction toward their tips
  // (display frame; its length the cosine of their slope: a tip move along it moves the object that
  // much deeper in the jaws, StraddleAtTip).
  Straddle?: Straddle | null;
  // A best-effort plan (GraspRules.BestEffort): what the app is not sure of, short phrases ("the grip may slip: ...").
  Warnings?: string[];
  // A deep plan (GraspRules.DeepGrasp): its palm is held to PalmGap from the object, and where its jaws close (the
  // planned pose, GraspSagCm under the printed grasp point) stands PalmGapCm from it (the palm and servo's hull),
  // ServoGapCm from the STS3215 servo's.
  Deep?: boolean; PalmGapCm?: number; ServoGapCm?: number };
export type Straddle = { FixedCm: number; MovingCm: number; Centre: P2; FingerAxis: XY };
/** The gap c a plan put the fixed pad at (PadPlace). */
export const PlanClearanceCm = (Plan: GraspPlan) => Plan.ClearanceCm ?? GraspRules.FixedJawClearanceCm;
// A new plan's gap (ClearanceFor), and the user's setting where it is not that gap (GraspPlan GapSetCm).
const PlanGap = (Used: number, Set: number) => ({ ClearanceCm: Used, ...(Math.abs(Used - Set) > 1e-9 ? { GapSetCm: Set } : {}) });
// "surface": it stands on a surface besides the desk the app does not plan on (AiMeasurements TaskObjectReachability).
export type GraspRefusalCode = "unmeasured" | "thin" | "wide" | "flat" | "palm" | "neighbour" | "path" | "reach" | "surface";
// Text goes to the model (and the task stop); Advice to the user. CloserCm: a
// "reach" refusal's computed move toward the arm that the app planned it at.
// GapCm: the gap c the pitch that got furthest placed the fixed pad at (GraspGap.ts GraspGapHint).
export type GraspRefusal = { Code: GraspRefusalCode; Text: string; Advice: string; CloserCm?: number | null; GapCm?: number };

const OpenDeg = (Servos: Record<ServoName, ServoState>, Fraction = GraspRules.OpenFraction) => angleFromRawValue(Servos.gripper, fractionToRaw(Servos.gripper, Fraction));
/** The jaw frame at a pose (display frame): opening axis u, hinge h, approach a, and the tool point. */
export function JawFrame(Angles: CadAngles) {
  const Links = CadForward(Angles).Links;
  const E = Links.gripper_link.elements;
  const Tool = mirrorDisplayPoint(new Vector3().setFromMatrixPosition(Links[CadModel.ToolLink]));
  return { Links, U: { x: -E[0], y: E[1], z: E[2] }, H: { x: -E[4], y: E[5], z: E[6] }, A: { x: E[8], y: -E[9], z: -E[10] }, Tool };
}
/** How far the object's slice in the jaw plane stands above a tool point T along -a, at a pose. */
export function JawSliceTopCm(Object_: Prism, T: Point3, Angles: CadAngles) {
  const Frame = JawFrame(Angles);
  return TopAboveTool(Object_, T, Frame.H, Frame.A);
}
const SweepDegs = (Open: number) => Array.from({ length: 16 }, (_, Index) => GraspRules.PadsMeetDeg + (Open - GraspRules.PadsMeetDeg) * Index / 15);

// --- The jaw-plane sweep on a prism (M3) ---------------------------------------------

type JawPlane = { U: Point3; H: Point3; A: Point3; Tool: Point3 };
/**
 * The prism's slice in the jaws' plane (through the tool point, across the
 * hinge) in gripper (x, z): x from the fixed pad into the gap, z along the
 * fingers (the tips at -z). DepthCm moves it toward the palm (+) or the tips.
 */
export function SliceInJawPlane(Object_: Prism, Frame: JawPlane, DepthCm = 0): P2[] {
  return ConvexHull2(Slice(Object_, Frame.Tool, Frame.H).map(Q => {
    const D = Sub3(Q, Frame.Tool);
    return { x: Dot3(D, Frame.U) + ToolInGripper.x, z: -Dot3(D, Frame.A) + ToolInGripper.z + DepthCm };
  }));
}
// The shapes the closing must hold. Nominal: the best estimate of its size
// (a round one: RoundSizes' NominalCm, 6.7 cm for the 5.0 x 4.5 can; else the
// measured footprint padded TrimPadCm a side like every planned object). The
// rest span what it may truly be: a round object every RoundStepCm from its
// larger measured side to its widest plausible one (a jaw step can meet a
// middling one worse); anything else, its measured box unpadded.
type ClosingShape = { Prism: Prism; DiameterCm: number | null; Nominal: boolean };
function ClosingShapes(Object_: Prism): ClosingShape[] {
  const Disc_ = (D: number, Nominal: boolean) => ({ Prism: { ...Object_, Footprint: Disc(Object_.Centre, D, GraspRules.SweepRoundSides) }, DiameterCm: D, Nominal });
  if (Object_.Round) {
    const { LoCm, HiCm, NominalCm } = Object_.Round;
    const Steps = Math.max(1, Math.ceil((HiCm - LoCm) / GraspRules.RoundStepCm - 1e-9));
    const Sizes = Array.from({ length: Steps + 1 }, (_, Index) => Number((LoCm + (HiCm - LoCm) * Index / Steps).toFixed(3)));
    return [Disc_(NominalCm, true), ...Sizes.filter(D => Math.abs(D - NominalCm) > 1e-6).map(D => Disc_(D, false))];
  }
  if (!Object_.SizeCm) return [{ Prism: Object_, DiameterCm: null, Nominal: true }];
  const C = Object_.Centre, Hx = Object_.SizeCm[0] / 2, Hy = Object_.SizeCm[1] / 2;
  return [{ Prism: Object_, DiameterCm: null, Nominal: true }, { Prism: { ...Object_, Footprint: [{ x: C.x - Hx, y: C.y - Hy }, { x: C.x + Hx, y: C.y - Hy },
    { x: C.x + Hx, y: C.y + Hy }, { x: C.x - Hx, y: C.y + Hy }] }, DiameterCm: null, Nominal: false }];
}
export type SweepVerdict = { Ok: boolean; Code: string; Why: string;
  // The jaw's push on the nominal shape where the closing leaves it (the design's lean), the
  // most friction angle the two-pad grip needs over every shape, depth error and the drift
  // the push gives it (in the jaws' plane, and GripIn3D's estimate with the jaws' tilt
  // against its standing side), where the jaw meets the nominal shape (CAD deg), the
  // diameter of the case that decided (null: a box), and how far the push carries the
  // nominal shape toward the finger tips (from where it was planned to stand).
  PushDeg: number | null; GripDeg: number | null; Grip3DDeg: number | null; ContactDeg: number | null; DiameterCm: number | null;
  DriftCm: number | null;
  // A round object: how far inside the fixed finger's tip the push leaves its contact (the least over its shapes), cm.
  TipCm?: number | null;
  // The open fractions (smallest first) whose open jaw, and the real jaw 11 deg more closed, clear the object by OpenClearanceCm.
  Opens: { Fraction: number; Deg: number }[] };
const SizeWords = (Diameter: number | null) => Diameter != null ? `${Diameter.toFixed(1)} cm across` : "";
const FingertipZ = FixedFingertipZ;
// PlanGrasp's first pass (true): a round object's closing is held to the 3D grip and the fingertip
// margin, and every sweep plan's closing is judged again at its printed point. Its fallback pass
// (false, only when the first found no plan) plans as before and the plan says what it lacks.
let Strict = true;
// PlanGrasp's best-effort pass (GraspRules.BestEffort): what only predicts a grip that may fail is a warning
// (PlaceInfo.Warnings, then GraspPlan.Warnings), not a refusal.
let Lenient = false;
// A best-effort plan's warning, once each.
const Warn = (Warnings: string[] | undefined, Why: string) => { if (Warnings && !Warnings.includes(Why)) Warnings.push(Why); };
/**
 * The closing check of the jaw-plane sweep at a pose. Every shape of the
 * object (ClosingShapes), from where it was planned to stand and Tolerance
 * nearer the palm or the tips (the camera's and the arm's error along the
 * fingers), is closed on the way the jaw drives it (CloseOn "push": the jaw
 * meets it where it stands and pushes it against the fixed finger, carrying
 * it along the fingers as its push leans). It must stop the closing jaw
 * StopAboveMeetDeg above where the pads meet, and the two-pad grip must hold
 * wherever it may end: every DepthStepCm over the span the pushes reach (the
 * grip jumps where the jaw meets another of its steps: 11 to 19 deg within
 * 0.05 cm). GripDeg <= MaxLeanDeg. The design held the jaw's push alone to
 * it, as if only the moving jaw's friction held the object; the drift is
 * what that push does, and the pads' grip is what holds it after.
 * Then the open fractions whose jaw clears the plan's prism (at each depth).
 * Not Quick, on PlanGrasp's first pass (Strict), a round object's grip in
 * 3D (GripIn3D) is held to MaxLeanDeg too, and its nominal shape, pushed
 * from where it was planned to stand, must leave its contact with the fixed
 * finger FingertipMarginCm inside the finger's tip (TipCm: how far it is).
 * Quick (the pitch screen, no IK): each shape slid straight against the
 * fixed pad, from its middle (where the planner's head start along the
 * fingers, PlaceAtPitch DepthCm, leaves it after the push) to Tolerance
 * either way, every quarter of it.
 */
export function ClosingSweep(Object_: Prism, Frame: JawPlane, Servos: Record<ServoName, ServoState>,
  Tolerance = GraspRules.DepthToleranceCm, Shapes = ClosingShapes(Object_), Quick = false, Deep = false): SweepVerdict {
  const Opens = SweepRules.OpenFractions.map(Fraction => ({ Fraction, Deg: OpenDeg(Servos, Fraction) }));
  const Widest = Math.max(...Opens.map(Open => Open.Deg));
  const Max = SweepRules.MaxLeanDeg;
  // The pushes from each end of the tolerance (the drift hardly changes between: 0.77 to 0.87 cm for the 6.6 cm can).
  const Starts = Tolerance > 0 ? [-Tolerance, Tolerance] : [0];
  // A deep plan (GraspRules.DeepGrasp): it cannot stand nearer the palm than the palm itself, so the start toward
  // the palm is at most the room there is in the jaws' plane (less 0.02 cm: touching is not overlapping).
  const StartsOf = (Shape: Prism) => !Deep || Tolerance <= 0 ? Starts
    : [-Tolerance, Math.max(0, Math.min(Tolerance, Gap(PalmPolygon, SliceInJawPlane(Shape, Frame)) - 0.02))];
  let Push: { Deg: number; Contact: number; Drift: number } | null = null, Grip = -Infinity, Grip3 = -Infinity, Tip = Infinity;
  let Failed: { Over: number; Code: string; Why: string; DiameterCm: number | null } | null = null;
  const Fail = (Over: number, Code: string, Why: string, DiameterCm: number | null) => { if (!Failed || Over > Failed.Over) Failed = { Over, Code, Why, DiameterCm }; };
  const Judge = (Result: ReturnType<typeof CloseOn>, Shape: ClosingShape, Final: number) => {
    if (!Result.Ok) { Fail(1e6, Result.Code, Result.Why, Shape.DiameterCm); return; }
    Grip = Math.max(Grip, Result.GripDeg!);
    const In3D = Result.Contacts ? GripIn3D(Result.Contacts, Frame.U, Frame.H, Frame.A) : -Infinity;
    Grip3 = Math.max(Grip3, In3D);
    // Not Quick: the jaws' tilt against its standing side counts too (GripIn3D).
    const Needs = Quick || !Object_.Round || !Strict ? Result.GripDeg! : Math.max(Result.GripDeg!, In3D);
    if (Needs <= Max) return;
    const Size = SizeWords(Shape.DiameterCm);
    const Where = [Math.abs(Final) >= 0.05 ? `ending ${Math.abs(Final).toFixed(1)} cm ${Final > 0 ? "nearer the palm" : "nearer the finger tips"} than the fixed pad's middle` : "", Size]
      .filter(Boolean).join(" and ");
    Fail(Needs - Max, "lean", `${Where ? `${Where}, ` : ""}closing jaws would need friction for ${Needs.toFixed(0)} deg at a pad (at most ${Max}${Needs > Result.GripDeg! ? ", with the jaws tilted against its standing side" : ""}) and squeeze it out past the finger tips`, Shape.DiameterCm);
  };
  // A round object pushed from where it was planned: how far inside the fixed finger's tip its contact ends.
  const TipAt = (Result: ReturnType<typeof CloseOn>, Shape: ClosingShape) => {
    if (!Result.Ok || !Result.Contacts) return;
    const Inside = Math.min(...Result.Contacts.Fixed.Patch.map(P => P.z)) - FingertipZ;
    Tip = Math.min(Tip, Inside);
    if (!Strict || Inside >= GraspRules.FingertipMarginCm) return;
    const Size = SizeWords(Shape.DiameterCm);
    Fail((GraspRules.FingertipMarginCm - Inside) * 10, "tip", `${Size ? `${Size}, ` : ""}the closing push would leave its contact with the fixed finger only ${Math.max(Inside, 0).toFixed(1)} cm inside`
      + ` the finger's tip, less than the ${GraspRules.FingertipMarginCm.toFixed(1)} cm that keeps the close from squeezing it out past the tips`, Shape.DiameterCm);
  };
  // A round object on PlanGrasp's first pass: the first failure ends the check, the fingertip's first (it is one push).
  // (And a deep plan on either pass: its head starts are tried deepest first, and a failing one only needs to fail.)
  const Early = (Strict || Deep) && !Quick && !!Object_.Round;
  Shapes: for (const Shape of Shapes) {
    if (Quick) {
      for (const Depth of [0, -0.5, -0.25, 0.25, 0.5]) {
        const Result = CloseOn(SliceInJawPlane(Shape.Prism, Frame, Depth), Widest, SweepRules, "slide", Depth === 0);
        if (Shape.Nominal && Depth === 0 && Result.Ok) Push = { Deg: Result.PushDeg!, Contact: Result.ContactDeg!, Drift: 0 };
        Judge(Result, Shape, Depth);
      }
      continue;
    }
    const Finals: number[] = [];
    const Own = StartsOf(Shape.Prism);
    for (const Start of Shape.Nominal ? (Early ? [0, ...Own] : [...Own, 0]) : Own) {
      const Result = CloseOn(SliceInJawPlane(Shape.Prism, Frame, Start), Widest, SweepRules, "push");
      if (Result.Ok) Finals.push(Start - Result.DriftCm!);
      if (Shape.Nominal && Start === 0 && Result.Ok) Push = { Deg: Result.PushDeg!, Contact: Result.ContactDeg!, Drift: Result.DriftCm! };
      Judge(Result, Shape, Result.Ok ? Start - Result.DriftCm! : Start);
      if (Start === 0 && Shape.Nominal && Object_.Round) TipAt(Result, Shape);
      if (Early && Failed) break Shapes;
    }
    if (!Finals.length) continue;
    // Every DepthStepCm over where the pushes leave it (a step wider each side: the drift
    // changes between the starts), slid straight against the finger there.
    const Low = Math.min(...Finals) - GraspRules.DepthStepCm, High = Math.max(...Finals) + GraspRules.DepthStepCm;
    for (let Depth = Low; Depth <= High + 1e-9; Depth += GraspRules.DepthStepCm) {
      Judge(CloseOn(SliceInJawPlane(Shape.Prism, Frame, Depth), Widest, SweepRules, "slide", false), Shape, Depth);
      if (Early && Failed) break Shapes;
    }
  }
  const Verdict = { PushDeg: Push?.Deg ?? null, GripDeg: Number.isFinite(Grip) ? Grip : null, Grip3DDeg: Number.isFinite(Grip3) ? Grip3 : null,
    ContactDeg: Push?.Contact ?? null, DriftCm: Push?.Drift ?? null, TipCm: Number.isFinite(Tip) ? Tip : null };
  if (Failed) {
    const F = Failed as { Code: string; Why: string; DiameterCm: number | null };
    return { Ok: false, Code: F.Code, Why: F.Why, ...Verdict, DiameterCm: F.DiameterCm, Opens: [] };
  }
  const Planned = StartsOf(Object_).map(Depth => SliceInJawPlane(Object_, Frame, Depth)).filter(Cut => Cut.length >= 3);
  // The open jaw within the steps' tolerance of each fraction: its widest, and the real jaw 11 deg past its narrowest.
  const Band = (Fraction: number) => [OpenDeg(Servos, Math.min(Fraction + GraspRules.OpenToleranceFraction, 1)), OpenDeg(Servos, Fraction - GraspRules.OpenToleranceFraction)];
  const Clear = Opens.filter(Open => Planned.every(Cut => Math.min(...Band(Open.Fraction).map(Deg_ => OpenClearance(Cut, Deg_))) >= SweepRules.OpenClearanceCm));
  if (!Clear.length) {
    return { Ok: false, Code: "open", ...Verdict, DiameterCm: Object_.Round?.NominalCm ?? null, Opens: [],
      Why: `even ${Math.round(Math.max(...SweepRules.OpenFractions) * 100)}% open the moving jaw would not clear it by ${SweepRules.OpenClearanceCm} cm` };
  }
  return { Ok: true, Code: "held", Why: "", ...Verdict, DiameterCm: Object_.Round?.NominalCm ?? null, Opens: Clear };
}

/**
 * How far the closing jaw's push carries the object's nominal shape toward the
 * finger tips from a jaw frame (cm, along the fingers), from Start (cm toward
 * the palm). 0 when the closing fails there (the full check says why).
 */
function NominalDrift(Object_: Prism, Frame: JawPlane, Servos: Record<ServoName, ServoState>, Start: number): number {
  const Nominal = ClosingShapes(Object_).find(Shape => Shape.Nominal)!;
  const Widest = Math.max(...SweepRules.OpenFractions.map(Fraction => OpenDeg(Servos, Fraction)));
  const Result = CloseOn(SliceInJawPlane(Nominal.Prism, Frame, Start), Widest, SweepRules, "push");
  return Result.Ok ? Result.DriftCm! : 0;
}
// Whether a sweep plan of the object straddles it (GraspRules.StraddleCm): a standing round one.
const StraddlesRound = (Object_: Prism) => !!Object_.Round && GraspRules.StraddleRoundObjects;
// The palm keeps this from the object a sweep plan grasps (SweepTaskMargins at the plan's margin).
const StraddlePalmCm = () => GraspRules.PlanMarginCm + GraspRules.DepthToleranceCm;
// The best-effort pass: the palm's samples only kept off the object (a warning when nearer than StraddlePalmCm).
const LenientPalmCm = 0.1;
/**
 * THE STRADDLE'S HEAD START for a sweep plan at a jaw frame whose tool point
 * puts the fixed pad's middle level with the object's centre (gripper z): how
 * much nearer the palm the object's centre must go (cm, the slice moves that
 * far toward the palm) for both jaws to reach GraspRules.StraddleCm past it
 * (MinCm: the fixed finger's tip along the fingers, the moving jaw's reach from
 * its hinge as it swings over it, JawSweep StraddleOf), how far for both to
 * reach StraddleGoodCm past it (GoodCm, Infinity: the moving jaw never does),
 * and how far it may go before its slice comes within the palm's margin of the
 * palm in the jaws' plane (MaxCm, at least MinCm: the palm's 3D check
 * decides). Not Ok: no head start lets the moving jaw reach StraddleCm past it
 * (its centre too far across the jaws from the hinge: too wide to surround).
 */
function StraddleWindow(Object_: Prism, Frame: JawPlane, Margin = StraddlePalmCm()):
  { Ok: true; MinCm: number; GoodCm: number; MaxCm: number } | { Ok: false; Why: string } {
  const Slice = SliceInJawPlane(Object_, Frame);
  if (Slice.length < 3) return { Ok: false, Why: "the jaws' plane would miss it" };
  const At = StraddleOf(Slice), Across = At.Centre.x - MovingJawHinge.x;
  // The centre moves toward the palm (+z) by the head start: the fixed finger reaches past it that much more, and the
  // moving jaw reaches Need past it once the centre is within MovingJawReachCm - Need of the hinge.
  const HeadStart = (Need: number) => {
    const Room = (MovingJawReachCm - Need) ** 2 - Across ** 2;
    return Room < 0 ? Infinity : Math.max(0, Need - At.FixedCm, MovingJawHinge.z - Math.sqrt(Room) - At.Centre.z);
  };
  const MinCm = HeadStart(GraspRules.StraddleCm);
  if (!Number.isFinite(MinCm)) {
    return { Ok: false, Why: `the jaws cannot surround it: its centre would be ${(At.Centre.x - ToolInGripper.x).toFixed(1)} cm across the jaws from the fixed pad,`
      + ` and the ${MovingJawReachCm.toFixed(1)} cm moving jaw then swings over it less than ${GraspRules.StraddleCm.toFixed(1)} cm short of its tip at any depth in the jaws` };
  }
  const GoodCm = HeadStart(GraspRules.StraddleGoodCm);
  const PalmGap = (Cm: number) => Gap(PalmPolygon, Slice.map(P => ({ x: P.x, z: P.z + Cm })));
  if (PalmGap(MinCm) < Margin) return { Ok: true, MinCm, GoodCm, MaxCm: MinCm };
  let Lo = MinCm, Hi = MinCm + 8;
  if (PalmGap(Hi) >= Margin) return { Ok: true, MinCm, GoodCm, MaxCm: Hi };
  for (let Step = 0; Step < 24; Step++) { const Mid = (Lo + Hi) / 2; if (PalmGap(Mid) >= Margin) Lo = Mid; else Hi = Mid; }
  return { Ok: true, MinCm, GoodCm, MaxCm: Lo };
}
/**
 * The head starts a sweep plan tries, first to last: the one for StraddleGoodCm
 * (Target, within MinCm..DeepestCm: the deepest where the palm stops it short of
 * that), then StraddleStepCm to either side, nearest first, StraddleTries in all.
 * A narrow object needs no more than that (deeper, the moving jaw meets it nearer
 * its hinge, more steeply); a 6.6 cm can gets as deep as the palm lets it.
 */
function StraddleDepths(MinCm: number, GoodCm: number, DeepestCm: number): number[] {
  const Target = Math.min(DeepestCm, Math.max(MinCm, GoodCm)), Step = GraspRules.StraddleStepCm;
  const Out = [Target];
  for (let K = 1; Out.length < GraspRules.StraddleTries && (Target - K * Step > MinCm - Step + 1e-6 || Target + K * Step < DeepestCm + Step - 1e-6); K++) {
    for (const D of [Target - K * Step, Target + K * Step]) {
      const Clamped = Math.min(DeepestCm, Math.max(MinCm, D));
      if (Out.length < GraspRules.StraddleTries && Out.every(Other => Math.abs(Other - Clamped) > 1e-6)) Out.push(Clamped);
    }
  }
  return Out;
}

/**
 * The head starts a deep plan tries (GraspRules.DeepGrasp): the deepest the palm and servo let it go, then
 * DeepStepCm shallower at a time, DeepTries in all, never short of the straddle's least (MinCm), which is
 * the last tried.
 */
function DeepDepths(MinCm: number, DeepestCm: number): number[] {
  const Out = [DeepestCm];
  for (let K = 1; Out.length < GraspRules.DeepTries - 1 && DeepestCm - K * GraspRules.DeepStepCm > MinCm + 1e-6; K++) Out.push(DeepestCm - K * GraspRules.DeepStepCm);
  if (DeepestCm - MinCm > 1e-6) Out.push(MinCm);
  return Out;
}
type PlaceInfo = { Uh: XY; Vh: XY; SNear: number; SFar: number; Cv: number; Delta: number; WEff: number; HMax: number | null;
  HeightBy: HeightBy; Bounds: Record<HeightBy, number>; Overlap?: number; TopAboveTool?: number; Estimate?: Point3; Other?: Contact;
  Sweep?: SweepVerdict;
  // Sweep: how much nearer the palm than the fixed pad's middle the grasp point puts the object's centre (cm along
  // the fingers: the straddle's head start), and the straddle there (JawSweep StraddleOf).
  DepthOffsetCm?: number; Straddle?: ReturnType<typeof StraddleOf>;
  // Other was met by the moving jaw's swing as it closes at the grasp point (not on the way down).
  OtherClosing?: boolean;
  // The gap c the pad was placed at (ClearanceFor: the user's grasp gap, or the default where it may not go lower).
  ClearanceCm?: number;
  // The best-effort pass: what the app is not sure of at this placement (GraspRules.BestEffort).
  Warnings?: string[];
  // A deep plan (GraspRules.DeepGrasp: a round object's straddle): its palm is judged by PalmGap.
  Deep?: boolean };
// Stages, in the order a pitch gets through them: 0 reach, 1 wide, 2 flat/pad,
// 3 contact with the object, 4 neighbour, 5 hover/path, 6 printed point check.
// Ok: the open angle the steps use (the smallest clear one), and every open angle clear at the grasp pose.
// SurfaceWrist: off the desk, the hover point or the lift is out of reach at this wrist (SurfaceWristMiss): higher is no better.
type PitchResult = { Ok: true; Pitch: number; T: Point3; Angles: CadAngles; Info: PlaceInfo; OpenDeg: number; OpenFraction: number;
  Opens: { Fraction: number; Deg: number }[] }
  | { Ok: false; Pitch: number; Stage: number; Code: string; Why: string; Info?: PlaceInfo; Collision?: string | null; SurfaceWrist?: boolean };

const PartName: Record<GripperPart, string> = { FixedFinger: "fixed finger", PalmAndServo: "palm", MovingJaw: "open moving jaw" };

// Where the fixed pad goes for a jaw axis: c (Clearance, the grasp gap; and the
// finger's reach past the tool) outside the footprint's near face, centred across the finger's band.
function PadPlace(Object_: Prism, Uh: XY, Delta: number, Clearance: number) {
  const Vh = { x: -Uh.y, y: Uh.x };
  const Across = Object_.Footprint.map(P => Dot2(P, Vh)), Cv = (Math.min(...Across) + Math.max(...Across)) / 2;
  const Band = ClipBand(Object_.Footprint, Vh, Cv - GraspRules.FingerBandHalfCm, Cv + GraspRules.FingerBandHalfCm);
  const SNear = Math.min(...Band.map(P => Dot2(P, Uh))), SFar = Math.max(...Band.map(P => Dot2(P, Uh)));
  const Along = SNear - Clearance - Math.max(Delta, 0);
  return { Vh, Cv, SNear, SFar, Txy: { x: Along * Uh.x + Cv * Vh.x, y: Along * Uh.y + Cv * Vh.y } };
}

/**
 * The grasp point for one wrist pitch (display frame): the fixed pad's plane
 * c outside the padded footprint along u, centred across the finger band,
 * at mid-height raised for the desk (open and closing jaws), the palm and the
 * moving jaw's reach under the top. Iterated: u turns with the pan at T.
 * Sweep (M3): the closing is judged by the jaw-plane sweep instead of the
 * jaw table, and the jaw opens to the smallest fraction that clears it; for
 * a round object (RealArmCorrections) the pad goes FixedFingerOffsetCm
 * farther out along u, so the real fixed finger stands c from it, and the
 * gripper is judged where the real one meets it (ArmPrism) as well as
 * checked clear of where the camera measured it. A round object's sweep plan
 * STRADDLES it (GraspRules.StraddleCm): the grasp point goes out along the
 * fingers until both jaws reach that far past its centre, and on to
 * StraddleGoodCm where the palm keeps its margin from it on the way down
 * (StraddleWindow, then the palm's 3D check), so the jaws surround it before
 * they close; where the closing does not hold there, StraddleStepCm to either
 * side (StraddleDepths; never short of the straddle). A box: the drift's head start.
 * RaiseCm: at least that far above mid-height (to clear a neighbour, PlanGrasp).
 */
export function PlaceAtPitch(Object_: Prism, Pitch: number, Context: GraspContext, Sweep = false, MaxDepthCm = Infinity, RaiseCm = 0): PitchResult {
  const Cache = PlaceCacheFor(Object_, Context), Key = `${Pitch}|${RaiseCm}|${Sweep}|${MaxDepthCm}|${Lenient}|${Strict}|${GraspRules.DeepGrasp}`;
  const Known = Cache?.get(Key);
  if (Known) return Known;
  const Result = PlaceAtPitchOnce(Object_, Pitch, Context, Sweep, MaxDepthCm, RaiseCm);
  // (Every failure: the key holds the passes' strictness; a deep plan's raises repeat many of them.)
  if (Cache && !Result.Ok) Cache.set(Key, Result);
  return Result;
}
function PlaceAtPitchOnce(Object_: Prism, Pitch: number, Context: GraspContext, Sweep: boolean, MaxDepthCm: number, RaiseCm: number): PitchResult {
  const Open = OpenDeg(Context.Servos);
  const { OffsetCm: Offset, On: Corrected } = RealArmCorrections(Object_, Sweep), Clearance = ClearanceFor(Context, Corrected);
  // The bounds hold for every open angle the sweep may pick.
  const Opens = Sweep ? SweepRules.OpenFractions.map(Fraction => OpenDeg(Context.Servos, Fraction)) : [Open];
  const Xs = Object_.Footprint.map(P => P.x), Ys = Object_.Footprint.map(P => P.y);
  const Centre = { x: (Math.min(...Xs) + Math.max(...Xs)) / 2, y: (Math.min(...Ys) + Math.max(...Ys)) / 2 };
  const Mid = (Object_.BaseZ + Object_.TopZ) / 2;
  const Limits = Context.Limits ?? undefined;
  // The first pose only gives the jaw frame to place the pad from: from the raised height (RaisedPlan), and where
  // the arm does not reach the object's centre at this wrist, from beside it across the arm's reach (either side,
  // the pad's distance out), so a pose the grasp point never takes is not called out of reach. 2026-10-03 14:47:36:
  // the gripper link met the basket's rim with the tip on the can's centre at mid-height at every pitch the jaws
  // hold it with, and the can was refused; beside it, 3 cm higher, the arm reaches it at wrist -12.6.
  // (The best-effort pass also tries it lower: a negative RaiseCm.)
  let T: Point3 = { ...Centre, z: Mid + (Lenient ? RaiseCm : Math.max(RaiseCm, 0)) };
  {
    const Reaches = (Point: Point3) => PreviewConverges(solveIkPreview({ ...modelPointFromDisplayPoint(Point), wristPitch: Pitch }, Context.ElbowMode, Context.Servos, Limits), Pitch);
    if (!Reaches(T)) {
      const Radial = FromPan(Centre).Unit, Across = { x: -Radial.y, y: Radial.x };
      const Out = Math.max(Math.max(...Xs) - Math.min(...Xs), Math.max(...Ys) - Math.min(...Ys)) / 2 + Clearance + 0.8;
      const Beside = [1, -1].map(Sign => ({ x: Centre.x + Sign * Out * Across.x, y: Centre.y + Sign * Out * Across.y, z: T.z })).find(Reaches);
      if (Beside) T = Beside;
    }
  }
  let Info: PlaceInfo | undefined;
  // Sweep: a standing round object's straddle (Straddles, StraddleWindow): the head start out along the
  // fingers until both jaws reach StraddleCm past its centre (MinDepthCm), and as deep as the palm leaves
  // room for. Anything else (a box: the jaws grip its flat sides at the pads, and deep in the jaws the
  // moving jaw meets its corner, leaning): the closing jaw's push carries it toward the finger tips (the
  // 16:49 can about 0.8 cm), so the grasp point goes that much farther out along the fingers and it ends
  // on the pad's middle.
  // The best-effort pass: a round object the jaws cannot surround at this pitch keeps the drift's head start, as a box does.
  let Straddles = Sweep && StraddlesRound(Object_);
  const Warnings: string[] | undefined = Lenient ? [] : undefined;
  let DepthCm = 0, MinDepthCm = 0, GoodDepthCm = 0, DeepestCm = 0;
  for (let Iteration = 0; Iteration < GraspRules.MaxIterations; Iteration++) {
    const Preview = solveIkPreview({ ...modelPointFromDisplayPoint(T), wristPitch: Pitch }, Context.ElbowMode, Context.Servos, Limits);
    if (!PreviewConverges(Preview, Pitch)) {
      return { Ok: false, Pitch, Stage: 0, Code: "reach", Info: Iteration && Info ? { ...Info, Estimate: T } : undefined, Collision: Preview.collisionReason ?? null,
        Why: ReachWhy(Pitch, Preview.collisionReason) };
    }
    const Angles = { ...Preview.angles, gripper: Open };
    const Frame = JawFrame(Angles);
    const Norm = Math.hypot(Frame.U.x, Frame.U.y), Uh = { x: Frame.U.x / Norm, y: Frame.U.y / Norm };
    // Where the real gripper meets it (a round object's sweep plan): the pad goes c outside this, the gripper is judged against it.
    const Real = ArmPrism(Object_, Uh, Offset);
    // How far the fixed finger reaches past the tool point toward the object, over the object's height.
    const Finger = GripperSamples(Angles, []).filter(S => S.Part === "FixedFinger" && S.P.z >= Object_.BaseZ - 0.5 && S.P.z <= Object_.TopZ + 0.5);
    const ToolU = Dot2(Frame.Tool, Uh);
    const Delta = Finger.length ? Math.max(...Finger.map(S => Dot2(S.P, Uh) - ToolU)) : 0;
    const { Vh, Cv, SNear, SFar, Txy } = PadPlace(Real, Uh, Delta, Clearance);
    // Heights: the desk (the open jaw and the closing sweep's lowest points under the tool), the palm, the jaw window.
    const Lowest = (Jaw: number) => GripperLowestZOfLinks(CadForward({ ...Angles, gripper: Jaw }).Links);
    const DOpen = Frame.Tool.z - Math.min(...Opens.map(Lowest));
    const DClose = Frame.Tool.z - Math.min(...SweepDegs(Math.max(...Opens)).map(Lowest));
    const Floor = Math.max(Object_.SupportZ + GraspRules.OpenFloorClearanceCm + DOpen, Object_.SupportZ + GraspRules.ClosingFloorClearanceCm + DClose);
    // Raising T is a rigid translation (pan and pitch do not change with Z): shift the samples to T's XY.
    // The sweep's flat pitches put the open jaw BESIDE the object (behind it along the fingers), not over it:
    // only what is over the footprint itself must clear the top; the contact checks keep the rest 0.5 cm away.
    const Shift = { x: Txy.x - Frame.Tool.x, y: Txy.y - Frame.Tool.y };
    // The widest opening sets this bound; a smaller one is taken only where it keeps every part clear (below).
    const Grow = Sweep ? 0 : GraspRules.OverTopGrowCm;
    const Body = GripperSamples(Angles, JawPair(Math.max(...Opens))).filter(S => S.Part !== "FixedFinger")
      .map(S => ({ ...S.P, x: S.P.x + Shift.x, y: S.P.y + Shift.y }))
      .filter(P => FootprintDistance(Real.Footprint, P) <= Grow);
    const BodyLow = Body.length ? Math.min(...Body.map(P => P.z)) - Frame.Tool.z : Infinity;
    const Palm = Object_.TopZ + GraspRules.PalmClearanceCm - BodyLow;
    const WEff = SFar - SNear + Clearance + Math.max(Delta, 0);
    // The sweep judges the closing itself: no jaw-table bound.
    const HMax = Sweep ? null : MaxTopAboveTool(WEff);
    const HAt = (Z: number) => TopAboveTool(Real, { ...Txy, z: Z }, Frame.H, Frame.A);
    let Jaw = -Infinity;
    if (HMax != null && HAt(Mid) > HMax) {
      let Lo = Mid, Hi = Object_.TopZ + 2;
      for (let Step = 0; Step < 30; Step++) { const Z = (Lo + Hi) / 2; if (HAt(Z) > HMax) Lo = Z; else Hi = Z; }
      Jaw = Hi;
    }
    // Raised for a neighbour (PlanGrasp): at least this high.
    const Bounds: Record<HeightBy, number> = { mid: Mid + Math.min(RaiseCm, 0), floor: Floor, palm: Palm, jaw: Jaw, clear: RaiseCm > 0 ? Mid + RaiseCm : -Infinity };
    const By = (Object.entries(Bounds) as [HeightBy, number][]).reduce((A, B) => (B[1] > A[1] + 1e-9 ? B : A))[0];
    let Next = { ...Txy, z: Bounds[By] };
    if (Sweep) {
      // Out along the level part of the approach: the slice moves toward the palm by DepthCm.
      const Level = Math.hypot(Frame.A.x, Frame.A.y) || 1, Out = (Cm: number) => ({ x: Next.x + Cm / Level * Frame.A.x / Level, y: Next.y + Cm / Level * Frame.A.y / Level, z: Next.z });
      // From the pad's middle level with its centre (the tool point here), the deepest the palm leaves room for.
      // (A deep plan: as deep as the palm's shadow in the jaws' plane leaves room for; PalmGap decides, below.)
      const Window = Straddles ? StraddleWindow(Real, { ...Frame, Tool: Next }, GraspRules.DeepGrasp ? 0.05 : Lenient ? LenientPalmCm : undefined) : null;
      if (Window && !Window.Ok && Lenient) {
        Warn(Warnings, `the jaws may not reach around it (${Window.Why.replace(/,\s*$/, "")})`);
        Straddles = false;
      }
      if (Straddles && Window) {
        if (!Window.Ok) {
          return { Ok: false, Pitch, Stage: 1, Code: "straddle", Info: { Uh, Vh, SNear, SFar, Cv, Delta, WEff, HMax, HeightBy: By, Bounds, ClearanceCm: Clearance },
            Why: `at wrist ${Deg(Pitch)} deg ${Window.Why}` };
        }
        MinDepthCm = Window.MinCm;
        GoodDepthCm = Window.GoodCm;
        // (The best-effort pass: shallower than the straddle's least too, where the arm reaches no deeper.)
        DepthCm = Lenient ? Math.min(MaxDepthCm, Window.MaxCm) : Math.max(MinDepthCm, Math.min(MaxDepthCm, Window.MaxCm));
        // A deep plan: the jaw frame settles at the straddle's good head start (or its least), which the arm reaches more
        // often; the deepest the palm and the arm's reach let it go is found from there, after (DeepestCm).
        if (GraspRules.DeepGrasp) {
          DeepestCm = DepthCm;
          DepthCm = Math.min(DepthCm, Lenient ? Math.min(MaxDepthCm, GoodDepthCm) : Math.max(MinDepthCm, Math.min(MaxDepthCm, GoodDepthCm)));
          // (Not even that: from the least head start the arm reaches, as shallow as none on the best-effort pass.)
          const Reaches = (D: number) => PreviewConverges(solveIkPreview({ ...modelPointFromDisplayPoint(Out(D)), wristPitch: Pitch }, Context.ElbowMode, Context.Servos, Limits), Pitch);
          if (!Reaches(DepthCm)) {
            let Lo = Lenient ? 0 : MinDepthCm, Hi = DepthCm;
            if (Reaches(Lo)) for (let Step = 0; Step < 5; Step++) { const Mid_ = (Lo + Hi) / 2; if (Reaches(Mid_)) Lo = Mid_; else Hi = Mid_; }
            DepthCm = Lo;
          }
        }
      } else {
        // The drift from where the current offset puts it (it hardly changes with the start: a fixed point in a step or two).
        DepthCm = Math.min(MaxDepthCm, Math.max(0, NominalDrift(Real, { ...Frame, Tool: Out(DepthCm) }, Context.Servos, 0)));
      }
      Next = Out(DepthCm);
    }
    const Moved = Math.hypot(Next.x - T.x, Next.y - T.y, Next.z - T.z);
    T = Next;
    Info = { Uh, Vh, SNear, SFar, Cv, Delta, WEff, HMax, HeightBy: By, Bounds, DepthOffsetCm: DepthCm, ClearanceCm: Clearance, Warnings,
      Deep: Straddles && GraspRules.DeepGrasp };
    if (HMax == null && !Sweep && !Lenient) {
      return { Ok: false, Pitch, Stage: 1, Code: "wide", Info,
        Why: `the moving jaw would have to close ${WEff.toFixed(1)} cm, more than the ${MaxWindowCm.toFixed(1)} cm it meets the side of` };
    }
    if (Moved < 0.01) break;
  }
  // On a surface besides the desk the hover point and the lift stay at this wrist (SurfaceWristMiss): a pitch that misses
  // them by more than the grasp point can still move toward the palm (DepthCm, its head start) is not judged further.
  if (Context.Floor?.Surface && !SideEntry(Context)) {
    const Miss = (Z: number) => solveIkPreview({ ...modelPointFromDisplayPoint({ ...T, z: Z }), wristPitch: Pitch }, Context.ElbowMode, Context.Servos, Limits).errorCm;
    const Hover = Math.max(T.z + GraspRules.HoverAboveGraspCm, Object_.TopZ + GraspRules.HoverAboveTopCm);
    const Lift = T.z + RealArmCorrections(Object_, Sweep).SagCm + GraspRules.LiftCm, Slack = DepthCm + 0.5;
    if (Miss(Hover) > Slack || Miss(Lift) > Slack) {
      return { Ok: false, Pitch, Stage: 5, Code: "path", Info, SurfaceWrist: true,
        Why: `at wrist ${Deg(Pitch)} deg the arm does not reach the hover point or the lift above the grasp point (off the desk the wrist stays one from the hover point to the lift)` };
    }
  }
  // Where the real gripper meets it, along the plan's jaw axis (the plan's JawAxis: PlanArmPrism).
  const Real = ArmPrism(Object_, Info!.Uh, Offset);
  // Sweep: the head start along the fingers brings the palm nearer the object's side toward the
  // arm. Its palm, at the grasp point and on the way down from the hover point, keeps PlanMarginCm +
  // DepthToleranceCm (+ 0.05 for the printed points' rounding) from it, where the real gripper
  // meets it and where the camera measured it. A straddle: the deepest such head start (from the
  // window's, down to the straddle's least; none: the jaws cannot surround it at this pitch), then
  // the closing judged at StraddleDepths. Else: the largest share of the drift's, the closing
  // check then judging the grip with the drift that is left.
  let Depths = [DepthCm];
  let Back = (_Cm: number): Point3 => T;
  // A straddle: the palm on the way down at a head start, checked as each is tried.
  let PathClear: ((D: number) => boolean) | null = null;
  if (Sweep && (Straddles || DepthCm > 0)) {
    const Frame0 = JawFrame({ ...solveIkPreview({ ...modelPointFromDisplayPoint(T), wristPitch: Pitch }, Context.ElbowMode, Context.Servos, Limits).angles, gripper: Open });
    const Level = Math.hypot(Frame0.A.x, Frame0.A.y) || 1, From = T;
    Back = (Cm: number) => ({ x: From.x - Cm / Level * Frame0.A.x / Level, y: From.y - Cm / Level * Frame0.A.y / Level, z: From.z });
    const Margin = (Lenient ? LenientPalmCm : StraddlePalmCm()) + 0.05;
    // A deep plan: the palm's exact gap (PalmGap), the printed points' rounding to spare.
    const Deep = Straddles && GraspRules.DeepGrasp, DeepGap = GraspRules.PalmGapCm + GraspRules.DeepRoundingCm;
    const PalmClear = (Point: Point3) => {
      const At = solveIkPreview({ ...modelPointFromDisplayPoint(Point), wristPitch: Pitch }, Context.ElbowMode, Context.Servos, Limits);
      const Hover = PregraspPoint(Object_, Point, Pitch, Context);
      const Above = solveIkPreview({ ...modelPointFromDisplayPoint(Hover), wristPitch: Pitch }, Context.ElbowMode, Context.Servos, Limits);
      if (!PreviewConverges(At, Pitch) || !PreviewConverges(Above, Pitch)) return true;
      if (Deep) return (PalmPathGap(Above.angles as CadAngles, At.angles as CadAngles, Real === Object_ ? [Object_] : [Object_, Real])?.GapCm ?? Infinity) >= DeepGap;
      for (let Index = 0; Index <= GraspRules.PathSamples; Index++) {
        const S_ = Index / GraspRules.PathSamples, From_ = Above.angles as CadAngles, To = At.angles as CadAngles, Angles_: CadAngles = { ...From_ };
        for (const Name of ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex"]) Angles_[Name] = From_[Name] + S_ * (To[Name] - From_[Name]);
        if (DeepestContact(GripperSamples(Angles_, []).filter(Sample => Sample.Part === "PalmAndServo"), Real === Object_ ? [Object_] : [Object_, Real], Margin)) return false;
      }
      return true;
    };
    // Head start D puts the tool Back(DepthCm - D).
    const Clear_ = (D: number) => PalmClear(Back(DepthCm - D));
    if (!Straddles) {
      const Share = [1, 0.75, 0.5, 0.25, 0].find(Part => Clear_(DepthCm * Part)) ?? (Lenient ? 0 : null);
      if (Share === 0 && Lenient && !Clear_(0)) Warn(Warnings, "the palm may push it on the way down");
      if (Share == null) {
        return { Ok: false, Pitch, Stage: 3, Code: "palm", Info: { ...Info!, DepthOffsetCm: 0 },
          Why: `at wrist ${Deg(Pitch)} deg the palm would come within ${StraddlePalmCm()} cm of it on the way down` };
      }
      Depths = [DepthCm * Share];
    } else {
      // The palm at the grasp pose alone (where it comes nearest: its face leans over the object there), bisected for
      // the deepest head start; the way down from the hover point is checked at the head starts then tried.
      const Prisms = Real === Object_ ? [Object_] : [Object_, Real];
      // A deep plan: the head start's grasp point, as it will be printed, passes the app's exact reach check.
      const Sag = RealArmCorrections(Object_, Sweep).SagCm;
      const Printable = (Point: Point3) => !!ExactlyReachable({ x: Tenth(Point.x), y: Tenth(Point.y), z: CeilTenth(Point.z + Sag) }, Pitch, Context,
        CeilTenth(Math.max(Point.z + GraspRules.HoverAboveGraspCm, Object_.TopZ + GraspRules.HoverAboveTopCm)));
      const PoseClear = (D: number) => {
        const At = solveIkPreview({ ...modelPointFromDisplayPoint(Back(DepthCm - D)), wristPitch: Pitch }, Context.ElbowMode, Context.Servos, Limits);
        if (Deep) return PreviewConverges(At, Pitch) && PalmGapTo(CadForward(At.angles as CadAngles).Links, Prisms) >= DeepGap;
        return !PreviewConverges(At, Pitch)
          || !DeepestContact(GripperSamples(At.angles as CadAngles, []).filter(Sample => Sample.Part === "PalmAndServo"), Prisms, Margin);
      };
      let Deepest: number | null = null;
      // (The best-effort pass: from no head start up, and none where even that brings the palm onto it, warned.)
      const Least = Lenient ? 0 : MinDepthCm;
      // A deep plan whose least head start the arm does not reach (printed): out of reach at this pitch.
      if (Deep) {
        const Pose = solveIkPreview({ ...modelPointFromDisplayPoint(Back(DepthCm - Least)), wristPitch: Pitch }, Context.ElbowMode, Context.Servos, Limits);
        if (!PreviewConverges(Pose, Pitch) || !Printable(Back(DepthCm - Least))) {
          return { Ok: false, Pitch, Stage: 0, Code: "reach", Info: { ...Info!, Estimate: Back(DepthCm - Least) }, Collision: Pose.collisionReason ?? null, Why: ReachWhy(Pitch, Pose.collisionReason) };
        }
      }
      // (A deep plan: up to the deepest the palm's shadow leaves room for, beyond where the jaw frame settled.)
      const Top = Deep ? Math.max(DeepestCm, DepthCm) : DepthCm;
      if (PoseClear(Top)) Deepest = Top;
      else if (Top - Least > 1e-6 && PoseClear(Least)) {
        let Lo = Least, Hi = Top;
        // (A deep plan to a few hundredths of a cm: Aaran wants it 0.2 to 0.3 cm from the servo.)
        for (let Step = 0; Step < (Deep ? 6 : 4); Step++) { const Mid_ = (Lo + Hi) / 2; if (PoseClear(Mid_)) Lo = Mid_; else Hi = Mid_; }
        Deepest = Lo;
      } else if (Lenient) {
        Deepest = 0;
        Warn(Warnings, "the palm may push it");
      }
      // A deep plan: no deeper than its printed grasp point passes the exact reach check (the least does, above).
      if (Deep && Deepest != null && Deepest > Least + 1e-6 && !Printable(Back(DepthCm - Deepest))) {
        let Lo = Least, Hi = Deepest;
        for (let Step = 0; Step < 6; Step++) { const Mid_ = (Lo + Hi) / 2; if (Printable(Back(DepthCm - Mid_))) Lo = Mid_; else Hi = Mid_; }
        Deepest = Lo;
      }
      if (Deepest != null) {
        Depths = Deep ? DeepDepths(Math.min(MinDepthCm, Deepest), Deepest) : StraddleDepths(Math.min(MinDepthCm, Deepest), GoodDepthCm, Deepest);
        // The best-effort pass: shallower too, down to no head start, where the arm reaches no deeper.
        if (Lenient) for (const D of [Math.min(Deepest, MinDepthCm) / 2, 0]) if (Depths.every(Other => Math.abs(Other - D) > 0.1)) Depths.push(D);
        PathClear = Clear_;
      }
      if (Deepest == null) {
        return { Ok: false, Pitch, Stage: 3, Code: "straddle", Info: { ...Info!, DepthOffsetCm: MinDepthCm },
          Why: `at wrist ${Deg(Pitch)} deg the palm would come within ${(Deep ? DeepGap : StraddlePalmCm()).toFixed(1)} cm of it on the way down before both jaws reach ${GraspRules.StraddleCm.toFixed(1)} cm past its centre`
            + ` (the jaws cannot surround it here)` };
      }
    }
  }
  // The checks at the pose for each head start (StraddleDepths): the first that passes; a sweep plan's
  // closing (or its palm at the grasp point) failing tries the next.
  let Failed: Extract<PitchResult, { Ok: false }> | null = null;
  for (const D of Depths) {
    // Each head start's own warnings (the best-effort pass).
    const Own = Warnings && [...Warnings];
    // (A deep plan: its closing first, the way down only where that holds, the closing being the likelier to fail.)
    const Early = Info!.Deep && !Lenient ? CheckPlacedPose(Object_, Real, Back(DepthCm - D), Pitch, Context, Sweep, { ...Info!, DepthOffsetCm: D, Warnings: Own }) : null;
    if (Early && !Early.Ok) { Failed = Early; continue; }
    if (PathClear && !PathClear(D) && Lenient) Warn(Own, "the palm may touch it on the way down");
    else if (PathClear && !PathClear(D)) {
      Failed ??= { Ok: false, Pitch, Stage: 3, Code: "straddle", Info: { ...Info!, DepthOffsetCm: D },
        Why: `at wrist ${Deg(Pitch)} deg the palm would come within ${(Info!.Deep ? GraspRules.PalmGapCm + GraspRules.DeepRoundingCm : StraddlePalmCm()).toFixed(1)} cm of it on the way down before both jaws reach ${GraspRules.StraddleCm.toFixed(1)} cm past its centre`
          + ` (the jaws cannot surround it here)` };
      continue;
    }
    const Result = Early ?? CheckPlacedPose(Object_, Real, Back(DepthCm - D), Pitch, Context, Sweep, { ...Info!, DepthOffsetCm: D, Warnings: Own });
    if (Result.Ok) return Result;
    Failed = Result;
    // (The best-effort pass: whatever failed, out of reach included, a shallower head start may not.)
    const Shallower = Lenient || !!Info!.Deep || (Sweep && ((Result.Stage === 1 && Result.Code !== "straddle") || (Result.Stage === 3 && Result.Code === "palm")));
    if (!Shallower) break;
  }
  return Failed!;
}

/**
 * PlaceAtPitch's checks at the pose for a placed grasp point T: the arm
 * reaches it, the pad faces the object, the jaw table's top edge (M1) or a
 * sweep plan's closing (ClosingSweep; a round object's straddle first, both
 * jaws StraddleCm past its centre, within the pan's turn since the window:
 * 0.1 cm), then the open fractions whose parts clear the object and its
 * neighbours.
 */
function CheckPlacedPose(Object_: Prism, Real: Prism, T: Point3, Pitch: number, Context: GraspContext, Sweep: boolean, Info: PlaceInfo): PitchResult {
  const Open = OpenDeg(Context.Servos);
  const Preview = solveIkPreview({ ...modelPointFromDisplayPoint(T), wristPitch: Pitch }, Context.ElbowMode, Context.Servos, Context.Limits ?? undefined);
  if (!PreviewConverges(Preview, Pitch)) return { Ok: false, Pitch, Stage: 0, Code: "reach", Info, Collision: Preview.collisionReason ?? null, Why: ReachWhy(Pitch, Preview.collisionReason) };
  // On a surface besides the desk: the printed hover point and the lift at this wrist (PrintedPlan's rule), before the
  // closing is judged: a platform's grasps sit high in the arm's reach, and the closing sweep is the slow part.
  const Unprinted = Context.Floor?.Surface ? SurfaceWristMiss(Object_, T, Pitch, Context, Sweep) : null;
  if (Unprinted) return { Ok: false, Pitch, Stage: 5, Code: "path", Info, Why: Unprinted, SurfaceWrist: true };
  const Angles = { ...Preview.angles, gripper: Open };
  const Frame = JawFrame(Angles);
  const PadZ = PadCorners.map(P => ToDisplay(Frame.Links.gripper_link, P).z);
  const Overlap = Math.min(Object_.TopZ, Math.max(...PadZ)) - Math.max(Object_.BaseZ, Math.min(...PadZ));
  const Final: PlaceInfo = { ...Info, Overlap, TopAboveTool: -Dot3(Sub3({ x: T.x, y: T.y, z: Object_.TopZ }, T), Frame.A) };
  // The best-effort pass (Lenient): what only predicts a grip that may fail is a warning; the pad must still face it.
  const Warnings = Final.Warnings;
  if (!Sweep) {
    const Top = TopAboveTool(Object_, T, Frame.H, Frame.A);
    if (Top > (Final.HMax ?? -Infinity) + 1e-6 && Lenient) Warn(Warnings, "the moving jaw may land on its top edge, not its side");
    else if (Top > (Final.HMax ?? -Infinity) + 1e-6) {
      return { Ok: false, Pitch, Stage: 1, Code: "top-edge", Info: Final, Why: `at wrist ${Deg(Pitch)} deg the moving jaw would land on its top edge, not its side` };
    }
  }
  if (Overlap < GraspRules.MinPadOverlapCm && Overlap > 0 && Lenient) Warn(Warnings, `only ${Overlap.toFixed(1)} cm of the fixed pad faces it`);
  else if (Overlap < GraspRules.MinPadOverlapCm) {
    return { Ok: false, Pitch, Stage: 2, Code: Final.HeightBy === "jaw" ? "wide" : Final.HeightBy === "palm" ? "palm" : "flat", Info: Final,
      Why: `at wrist ${Deg(Pitch)} deg only ${Math.max(Overlap, 0).toFixed(1)} cm of the fixed pad would face it` };
  }
  // The open angles to try: the table's 0.8, or the sweep's that clear the object, smallest first.
  let Candidates = [{ Fraction: GraspRules.OpenFraction, Deg: Open }];
  if (Sweep) {
    // A standing round object: both jaws past its centre where the real jaws meet it. Then the closing there.
    if (StraddlesRound(Object_)) {
      const Straddle = StraddleOf(SliceInJawPlane(Real, Frame));
      Final.Straddle = Straddle;
      const Short = StraddleShortWhy(Straddle, GraspRules.StraddleCm - 0.1);
      if (Short && Lenient) Warn(Warnings, `the jaws may not reach around it (${Short.replace(/^the jaws would not surround it: /, "")})`);
      else if (Short) return { Ok: false, Pitch, Stage: 1, Code: "straddle", Info: Final, Why: `at wrist ${Deg(Pitch)} deg ${Short}` };
    }
    const Verdict = ClosingSweep(Real, Frame, Context.Servos, undefined, undefined, false, !!Final.Deep);
    Final.Sweep = Verdict;
    if (!Verdict.Ok && Lenient) {
      // Whatever the closing's verdict, the moving jaw must meet it where it was measured (not close past it or miss
      // the jaws' plane), and the open jaw must not already be in it: else it is no grasp at all.
      const Nominal = CloseOn(SliceInJawPlane(Real, Frame), OpenDeg(Context.Servos, Math.max(...SweepRules.OpenFractions)), SweepRules, "push");
      if (!Nominal.Ok && (Nominal.Code === "slips" || Nominal.Code === "free")) {
        return { Ok: false, Pitch, Stage: 1, Code: "wide", Info: Final, Why: `at wrist ${Deg(Pitch)} deg ${Nominal.Why}` };
      }
      Candidates = Verdict.Opens.length ? Verdict.Opens : TouchFreeOpens(Real, Frame, Context.Servos);
      if (!Candidates.length) {
        return { Ok: false, Pitch, Stage: 1, Code: "wide", Info: Final,
          Why: `at wrist ${Deg(Pitch)} deg even ${Math.round(Math.max(...SweepRules.OpenFractions) * 100)}% open the moving jaw would be in it: it does not fit between the jaws` };
      }
      Warn(Warnings, `the grip may slip (${Verdict.Why})`);
    } else if (!Verdict.Ok) return { Ok: false, Pitch, Stage: 1, Code: Verdict.Code === "lean" || Verdict.Code === "tip" ? "lean" : "wide", Info: Final, Why: `at wrist ${Deg(Pitch)} deg ${Verdict.Why}` };
    else Candidates = Verdict.Opens;
  }
  let Failed: PitchResult | null = null;
  const Clear: { Fraction: number; Deg: number }[] = [];
  for (const Candidate of Candidates) {
    const Samples = GripperSamples({ ...Angles, gripper: Candidate.Deg }, PlanJawDegs(Context.Servos, Candidate.Fraction));
    const Own = Final.Deep ? DeepTaskMargins : Sweep ? SweepTaskMargins(GraspRules.PlanMarginCm) : null;
    // A deep plan: its palm PalmGapCm from it (the exact gap at the pose).
    const PalmCm = Final.Deep ? PalmGapTo(CadForward({ ...Angles, gripper: Candidate.Deg }).Links, Real === Object_ ? [Object_] : [Object_, Real]) : Infinity;
    const Task = DeepestContact(Samples, [Object_], GraspRules.PlanMarginCm, Own, Object_)
      ?? (Real === Object_ ? null : DeepestContact(Samples, [Real], GraspRules.PlanMarginCm, Own, Real))
      ?? (PalmCm < GraspRules.PalmGapCm ? { Part: "PalmAndServo" as GripperPart, Label: Object_.Label, DepthCm: GraspRules.PlanMarginCm - PalmCm, At: T, Prism: Object_ } : null);
    if (Task && !Lenient) {
      Failed ??= { Ok: false, Pitch, Stage: 3, Code: "palm", Info: Final,
        Why: `at wrist ${Deg(Pitch)} deg the ${PartName[Task.Part]} would come within ${(GraspRules.PlanMarginCm - Task.DepthCm).toFixed(1)} cm of it` };
      continue;
    }
    const Other = DeepestContact(Samples, Context.Others, GraspRules.PlanMarginCm);
    if (Other) {
      if (!Failed || Failed.Ok || Failed.Stage < 4) Failed = { Ok: false, Pitch, Stage: 4, Code: "neighbour", Info: { ...Final, Other },
        Why: `the ${PartName[Other.Part]} would come within ${(GraspRules.PlanMarginCm - Other.DepthCm).toFixed(1)} cm of the ${Other.Label}` };
      continue;
    }
    if (Task && !Clear.length) Warn(Warnings, Task.DepthCm > GraspRules.PlanMarginCm ? `the ${PartName[Task.Part]} may push it` : `the ${PartName[Task.Part]} comes within ${(GraspRules.PlanMarginCm - Task.DepthCm).toFixed(1)} cm of it`);
    Clear.push(Candidate);
  }
  if (!Clear.length) return Failed!;
  return { Ok: true, Pitch, T, Angles: { ...Angles, gripper: Clear[0].Deg }, Info: Final, OpenDeg: Clear[0].Deg, OpenFraction: Clear[0].Fraction, Opens: Clear };
}
/**
 * The best-effort pass (GraspRules.BestEffort) with a closing that may not hold: the open fractions whose open
 * jaw, and the real jaw 11 deg more closed, are not in the object's slice (over the steps' tolerance of each).
 */
function TouchFreeOpens(Object_: Prism, Frame: JawPlane, Servos: Record<ServoName, ServoState>): { Fraction: number; Deg: number }[] {
  const Cut = SliceInJawPlane(Object_, Frame);
  const Band = (Fraction: number) => [OpenDeg(Servos, Math.min(Fraction + GraspRules.OpenToleranceFraction, 1)), OpenDeg(Servos, Fraction - GraspRules.OpenToleranceFraction)];
  return SweepRules.OpenFractions.map(Fraction => ({ Fraction, Deg: OpenDeg(Servos, Fraction) }))
    .filter(Open => Cut.length < 3 || Math.min(...Band(Open.Fraction).map(Deg_ => OpenClearance(Cut, Deg_))) > 0);
}
/**
 * How far both jaws reach past the object's centre (the less, cm) with the tip
 * at Tip instead of a sweep plan's grasp point (its Straddle): the tip off along
 * the fingers moves the object as far deeper in the jaws (by the fingers' slope)
 * or shallower, off along the jaw axis as far across them; up or down keeps it
 * (a standing object). Null: a plan without a straddle.
 */
export function StraddleAtTip(Plan: GraspPlan, Tip: XY): number | null {
  const Straddle = Plan.Straddle;
  if (!Straddle) return null;
  const Dx = Tip.x - Plan.Grasp.x, Dy = Tip.y - Plan.Grasp.y;
  const Deep = Dx * Straddle.FingerAxis.x + Dy * Straddle.FingerAxis.y, Toward = Dx * Plan.JawAxis.x + Dy * Plan.JawAxis.y;
  const Centre = { x: Straddle.Centre.x - Toward, z: Straddle.Centre.z + Deep };
  return Math.min(Centre.z - FingertipZ, MovingJawReachCm - Math.hypot(Centre.x - MovingJawHinge.x, Centre.z - MovingJawHinge.z));
}
// A pose the arm does not take at a wrist pitch: out of reach, or (Collision) in the basket's or the body's way there.
const ReachWhy = (Pitch: number, Collision: string | null | undefined) => Collision
  ? `at wrist ${Deg(Pitch)} deg, at the grasp point beside it, the ${Collision}`
  : `at wrist ${Deg(Pitch)} deg the arm does not reach the grasp point beside it`;
/** Why a straddle is short of Least cm (both jaws past the object's centre), or null. */
function StraddleShortWhy(Straddle: { FixedCm: number; MovingCm: number }, Least: number): string | null {
  if (Math.min(Straddle.FixedCm, Straddle.MovingCm) >= Least) return null;
  return `the jaws would not surround it: the fixed finger would reach ${Math.max(Straddle.FixedCm, 0).toFixed(1)} cm and the moving jaw ${Math.max(Straddle.MovingCm, 0).toFixed(1)} cm`
    + ` past its centre (the app wants ${GraspRules.StraddleCm.toFixed(1)} for both)`;
}

/**
 * The checks of SnapAiTarget's exact branch (AiTargetSnap.ts) on a display
 * point, with the move starting at AnchorZ: the IK angles when the snap would
 * take it as it is, else null. So the printed points are the ones checked.
 * They also keep BasketExtraCm more from the basket and body than the model
 * asks (as the snap's moved targets do): the bridge re-checks the pose
 * rounded to encoder steps, and a refusal there sends it on a detour that
 * knows nothing of the measured objects.
 */
export function ExactlyReachable(Point: Point3, Pitch: number, Context: GraspContext, AnchorZ: number): CadAngles | null {
  const Target: TargetPose = { ...modelPointFromDisplayPoint(Point), wristPitch: Pitch };
  const Anchor = modelPointFromDisplayPoint({ ...Point, z: AnchorZ });
  // The task's floor, as the snap and the page hold the targets the model copies (the desk's without one).
  const Floor = Context.Floor ?? DeskFloor;
  if (AiTableFloorReason(Target, Target.z - Anchor.z, Floor) || BasketTargetReason(Target)) return null;
  const Limits = InsetIkLimits(GetIkAngleLimits(Context.Servos, Context.Limits ?? undefined), 0.5);
  const Preview = solveIkPreview(Target, Context.ElbowMode, Context.Servos, Limits);
  if (!PreviewConverges(Preview, Pitch)) return null;
  if (!AiJawFloorAllows(Target, GripperFloorGapOfLinks(CadForward(Preview.angles).Links, Floor), GripperFloorGap(currentAngles(Context.Servos), Floor))) return null;
  if (BasketPoseReason(Preview.angles, GraspRules.BasketExtraCm)) return null;
  return Preview.angles;
}

// Which side of the object the fixed jaw goes to (-u), in the AI's words.
function SideOf(JawAxis: XY) {
  const Toward = { x: -JawAxis.x, y: -JawAxis.y };
  return [Math.abs(Toward.x) >= 0.38 ? (Toward.x > 0 ? "left" : "right") : null,
    Math.abs(Toward.y) >= 0.38 ? (Toward.y > 0 ? "far" : "near") : null].filter(Boolean).join("-") || "near";
}
const WidthAcross = (Object_: Prism, Uh: XY) => Object_.SizeCm ? Math.abs(Uh.x) * Object_.SizeCm[0] + Math.abs(Uh.y) * Object_.SizeCm[1] : Infinity;

export type PlanResult = { Plan: GraspPlan; Refusal: null; Tried: PitchResult[] } | { Plan: null; Refusal: GraspRefusal; Tried: PitchResult[] };
const Now = () => (typeof performance !== "undefined" ? performance.now() : Date.now());

// The printed grasp and hover points of a placed pitch, checked as printed:
// the snap's exact check, no contact at the guard's margin, a hover pitch,
// the joint-linear descent from the hover clear of every object, and the
// moving jaw's swing as it closes there clear of every other object (the
// guard checks that swing before the close runs: a plan whose close it
// refuses would leave the arm low beside the object). The jaw opens to the
// smallest clear fraction whose descent is clear too: a jaw opened less sits
// nearer the object, and on the way down past a can's rim it can come closer
// than at the grasp point. The jaw is checked over the steps' tolerance of
// that fraction (PlanJawDegs). A round object's sweep plan (RealArmCorrections)
// is checked against it where the camera measured it and where the real
// gripper meets it (ArmPrism), and its grasp point is printed GraspSagCm above
// where its jaws close (T), where the arm stops under it; the pose at T passed
// PlaceAtPitch's checks. Its fixed finger keeps PlanMarginCm from it on the way
// down and at the grasp point with the real finger FixedFingerToleranceCm
// nearer still. On PlanGrasp's first pass a sweep plan's closing is judged
// again where the printed point closes (rounded, less the sag): T sat at the
// edge of the grip's limit, and the rounding moved the 15:05 can's plan past it.
/**
 * On a surface besides the desk (GraspContext.Floor) the grasp sits high in the arm's reach: the wrist stays one
 * from the hover point down to the grasp point and up the lift, as the desk's grasps of 2026-10-04 went (a
 * near-level wrist and a vertical descent). Why a tool point T at Pitch breaks that (the printed hover point or the
 * point GraspRules.LiftCm above the printed grasp point fails the exact reach check at that wrist), or null. At the
 * steeper wrist nearest the preference (-21 deg for a can on a 15 cm box, 35 cm out) the arm reached only 0.5 cm
 * straight up from the grasp point, and the snap turned the wrist 12-18 deg flatter for the lift, the can in the
 * jaws: such pitches give way to flatter ones.
 */
const SideEntry = (Context: GraspContext) => Context.Floor?.Surface?.Kind === "shelf" && !!Context.Floor.Scene;
const PitchCeiling = (Context: GraspContext) => SideEntry(Context) ? 45 : 0;
function PregraspPoint(Object_: Prism, Grasp: Point3, Pitch: number, Context: GraspContext): Point3 {
  if (!SideEntry(Context)) return { ...Grasp, z: Math.max(Grasp.z + GraspRules.HoverAboveGraspCm, Object_.TopZ + GraspRules.HoverAboveTopCm) };
  const At = solveIkPreview({ ...modelPointFromDisplayPoint(Grasp), wristPitch: Pitch }, Context.ElbowMode, Context.Servos, Context.Limits ?? undefined);
  const Axis = JawFrame(At.angles).A, Level = Math.max(Math.hypot(Axis.x, Axis.y), 0.1);
  // Approach along the fingers toward the object. Stay near-side of the support's observed front edge.
  const Distance = Math.max(6, Axis.y > 0.1 ? (Grasp.y - Context.Floor!.Surface!.YRangeCm[0] + 2) * Level / Axis.y : 6);
  return { x: Tenth(Grasp.x - Distance * Axis.x / Level), y: Tenth(Grasp.y - Distance * Axis.y / Level), z: Grasp.z };
}
function SurfaceWristMiss(Object_: Prism, T: Point3, Pitch: number, Context: GraspContext, Sweep: boolean): string | null {
  const Sag = RealArmCorrections(Object_, Sweep).SagCm;
  const Grasp = { x: Tenth(T.x), y: Tenth(T.y), z: CeilTenth(T.z + Sag) };
  const Hover = SideEntry(Context) ? PregraspPoint(Object_, Grasp, Pitch, Context)
    : { ...Grasp, z: CeilTenth(Math.max(T.z + GraspRules.HoverAboveGraspCm, Object_.TopZ + GraspRules.HoverAboveTopCm)) };
  if (SideEntry(Context)) {
    const Release = { ...Grasp, z: CeilTenth(Grasp.z + 1.2) }, Retreat = { ...Hover, z: Release.z };
    if (![Hover, Release, Retreat].every(P => ExactlyReachable(P, Pitch, Context, P.z))) return `at wrist ${Deg(Pitch)} deg the arm cannot reach the side approach, release and retreat at the same wrist`;
    return null;
  }
  if (!ExactlyReachable(Hover, Pitch, Context, Hover.z)) {
    return `at wrist ${Deg(Pitch)} deg the arm does not reach the hover point Z ${Hover.z.toFixed(1)} (off the desk the wrist stays one from the hover point to the lift)`;
  }
  if (!ExactlyReachable({ ...Grasp, z: CeilTenth(Grasp.z + GraspRules.LiftCm) }, Pitch, Context, Grasp.z)) {
    return `at wrist ${Deg(Pitch)} deg the arm does not reach ${GraspRules.LiftCm} cm straight up from the grasp point, where the lift goes (off the desk the wrist stays one from the hover point to the lift)`;
  }
  return null;
}

function PrintedPlan(Object_: Prism, Result: Extract<PitchResult, { Ok: true }>, Context: GraspContext, Started: number):
  { Ok: true; Plan: GraspPlan } | Extract<PitchResult, { Ok: false }> {
  const { Pitch } = Result;
  const { OffsetCm: Offset, SagCm: Sag, On: Corrected } = RealArmCorrections(Object_, !!Result.Info.Sweep);
  const Real = ArmPrism(Object_, Result.Info.Uh, Offset);
  const Tasks = Real === Object_ ? Object_ : [Object_, Real];
  const All = Real === Object_ ? [Object_, ...Context.Others] : [Object_, Real, ...Context.Others];
  const Fail = (Stage: number, Code: string, Why: string) => ({ Ok: false as const, Pitch, Stage, Code, Why, Info: Result.Info });
  const Grasp = { x: Tenth(Result.T.x), y: Tenth(Result.T.y), z: CeilTenth(Result.T.z + Sag) };
  const Hover = SideEntry(Context) ? PregraspPoint(Object_, Grasp, Pitch, Context)
    : { x: Grasp.x, y: Grasp.y, z: CeilTenth(Math.max(Result.T.z + GraspRules.HoverAboveGraspCm, Object_.TopZ + GraspRules.HoverAboveTopCm)) };
  const GraspAngles = ExactlyReachable(Grasp, Pitch, Context, Hover.z);
  if (!GraspAngles) return Fail(6, "verify", `at wrist ${Deg(Pitch)} deg the rounded grasp point fails the app's exact reach check`);
  let HoverPitch: number | null = Pitch;
  let HoverAngles = ExactlyReachable(Hover, Pitch, Context, Hover.z);
  const Unprinted = Context.Floor?.Surface ? SurfaceWristMiss(Object_, Result.T, Pitch, Context, !!Result.Info.Sweep) : null;
  if (Unprinted) return { ...Fail(5, "path", Unprinted), SurfaceWrist: true };
  if (!HoverAngles) {
    HoverPitch = Context.Scan(modelPointFromDisplayPoint(Hover));
    HoverAngles = HoverPitch != null ? ExactlyReachable(Hover, HoverPitch, Context, Hover.z) : null;
  }
  // Where the printed point closes (the jaws where the real gripper meets it): a round object's
  // straddle, both jaws past its centre (the rounding moves it up to 0.07 cm along the fingers), and
  // on PlanGrasp's first pass a sweep plan's closing.
  let Sweep = Result.Info.Sweep, Opens = Result.Opens, Straddle: Straddle | null = null;
  // The best-effort pass (Lenient): this plan's warnings; contact with the object it grasps is one, a neighbour's is not.
  const Warnings = Lenient ? [...(Result.Info.Warnings ?? [])] : undefined;
  const Judged = Lenient ? Context.Others : All;
  if (Sweep && (Strict || StraddlesRound(Object_))) {
    const At = solveIkPreview({ ...modelPointFromDisplayPoint({ ...Grasp, z: Grasp.z - Sag }), wristPitch: Pitch }, Context.ElbowMode, Context.Servos, Context.Limits ?? undefined);
    if (!PreviewConverges(At, Pitch)) return Fail(6, "verify", `at wrist ${Deg(Pitch)} deg the arm does not reach where the rounded grasp point closes`);
    const Frame = JawFrame({ ...At.angles, gripper: Result.OpenDeg });
    if (StraddlesRound(Object_)) {
      const Reach = StraddleOf(SliceInJawPlane(Real, Frame));
      const Short = StraddleShortWhy(Reach, GraspRules.StraddleCm - 0.1);
      if (Short && Lenient) Warn(Warnings, `the jaws may not reach around it (${Short.replace(/^the jaws would not surround it: /, "")})`);
      else if (Short) return Fail(6, "straddle", `at wrist ${Deg(Pitch)} deg, at the rounded grasp point, ${Short}`);
      Straddle = { FixedCm: Reach.FixedCm, MovingCm: Reach.MovingCm, Centre: Reach.Centre, FingerAxis: { x: Frame.A.x, y: Frame.A.y } };
    }
    if (Strict) {
      Sweep = ClosingSweep(Real, Frame, Context.Servos, undefined, undefined, false, !!Result.Info.Deep);
      if (!Sweep.Ok) return Fail(6, Sweep.Code === "open" ? "wide" : "lean", `at wrist ${Deg(Pitch)} deg, at the rounded grasp point, ${Sweep.Why}`);
      Opens = Opens.filter(Open => Sweep!.Opens.some(Other => Other.Fraction === Open.Fraction));
    }
  }
  // The real fixed finger nearer than planned: the fixed finger on the way down from the hover point and at the grasp point.
  const Nearer = Corrected ? ArmPrism(Object_, Result.Info.Uh, Offset + GraspRules.FixedFingerToleranceCm) : null;
  if (Nearer && HoverAngles) {
    for (let Index = 0; Index <= GraspRules.PathSamples; Index++) {
      const S = Index / GraspRules.PathSamples, Angles: CadAngles = { ...HoverAngles };
      for (const Name of ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex"]) Angles[Name] = HoverAngles[Name] + S * (GraspAngles[Name] - HoverAngles[Name]);
      const Hit = DeepestContact(GripperSamples(Angles, []).filter(Sample => Sample.Part === "FixedFinger"), [Nearer], GraspRules.PlanMarginCm);
      if (Hit && Lenient) { Warn(Warnings, "the fixed finger may push it on the way down"); break; }
      if (Hit) return Fail(5, "path", `${S ? `${Math.round(S * 100)}% of the way down from the hover point` : "at the hover point"} the fixed finger would come within ${Math.max(GraspRules.PlanMarginCm - Hit.DepthCm, 0).toFixed(1)} cm of the ${Object_.Label} if the real fixed finger stood ${GraspRules.FixedFingerToleranceCm.toFixed(1)} cm nearer it than planned`);
    }
  }
  let Failed: Extract<PitchResult, { Ok: false }> | null = null;
  const CloseDeg = OpenDeg(Context.Servos, 0);
  const Own = (Margin: number) => Result.Info.Deep ? DeepTaskMargins : Result.Info.Sweep ? SweepTaskMargins(Margin) : null;
  const TaskList = Array.isArray(Tasks) ? Tasks : [Tasks];
  for (const { Fraction, Deg: Open } of Opens) {
    const Jaws = PlanJawDegs(Context.Servos, Fraction);
    // A deep plan: the palm and servo PalmGapCm from it at the rounded grasp point, at the hover point and all the way down.
    if (Result.Info.Deep && HoverAngles) {
      const Way = PalmPathGap({ ...HoverAngles, gripper: Open }, { ...GraspAngles, gripper: Open }, TaskList);
      if (Way && Way.GapCm < GraspRules.PalmGapCm) {
        const Where = Way.S >= 1 ? "at the rounded grasp point" : Way.S > 0 ? `${Math.round(Way.S * 100)}% of the way down from the hover point` : "at the hover point";
        if (Lenient) Warn(Warnings, `the palm may touch it${Way.S < 1 ? " on the way down" : ""}`);
        else { Failed ??= Fail(Way.S >= 1 ? 6 : 5, Way.S >= 1 ? "verify" : "path", `${Where} the palm would come within ${Way.GapCm.toFixed(1)} cm of the ${Object_.Label}`); continue; }
      }
    }
    const Printed = DeepestContact(GripperSamples({ ...GraspAngles, gripper: Open }, Jaws), Judged, GraspRules.ContactMarginCm, Own(GraspRules.ContactMarginCm), Tasks);
    if (Printed) { Failed ??= Fail(6, "verify", `at the rounded grasp point the ${PartName[Printed.Part]} would come within ${(GraspRules.ContactMarginCm - Printed.DepthCm).toFixed(1)} cm of the ${Printed.Label}`); continue; }
    if (!HoverAngles || HoverPitch == null) return Failed ?? Fail(5, "path", `no wrist pitch reaches the hover point Z ${Hover.z.toFixed(1)} above the grasp point`);
    const AtHover = DeepestContact(GripperSamples({ ...HoverAngles, gripper: Open }, Jaws), Judged, GraspRules.PlanMarginCm, Own(GraspRules.PlanMarginCm), Tasks);
    const Path = AtHover ? { ...AtHover, S: 0, FirstS: 0 }
      : PathContact({ ...HoverAngles, gripper: Open }, { ...GraspAngles, gripper: Open }, Open, Judged, GraspRules.PlanMarginCm, GraspRules.PathSamples, Jaws,
        Own(GraspRules.PlanMarginCm), Tasks);
    if (Path) {
      Failed ??= Fail(5, "path", `${Path.S ? `${Math.round(Path.FirstS * 100)}% of the way down from the hover point` : "at the hover point"} the ${PartName[Path.Part]} would come within ${(GraspRules.PlanMarginCm - Path.DepthCm).toFixed(1)} cm of the ${Path.Label}`);
      continue;
    }
    // The close at the grasp point: the moving jaw's swing from this opening to a full close, as the guard checks it.
    const Swing = JawSwingContact({ ...GraspAngles, gripper: Open }, CloseDeg, Context.Others);
    if (Swing) {
      if (!Failed || Failed.Stage < 4) Failed = { ...Fail(4, "neighbour", `closing at the grasp point, the moving jaw would swing within ${(GraspRules.ContactMarginCm - Swing.Hit.DepthCm).toFixed(1)} cm of the ${Swing.Hit.Label}`),
        Info: { ...Result.Info, Other: Swing.Hit, OtherClosing: true } };
      continue;
    }
    let Approach: GraspPlan["Approach"];
    if (Context.Floor?.Scene) {
      const Release = { ...Grasp, z: CeilTenth(Grasp.z + (SideEntry(Context) ? 1.2 : GraspRules.LiftCm)) };
      const Retreat = SideEntry(Context) ? { ...Hover, z: Release.z } : Release;
      const ReleaseAngles = ExactlyReachable(Release, Pitch, Context, Grasp.z);
      const RetreatAngles = ExactlyReachable(Retreat, Pitch, Context, Release.z);
      if (!ReleaseAngles || !RetreatAngles) { Failed ??= Fail(5, "path", "the arm cannot reach the release and withdrawal at the grasp wrist"); continue; }
      const ClosedGrasp = { ...GraspAngles, gripper: CloseDeg }, ClosedRelease = { ...ReleaseAngles, gripper: CloseDeg }, ClosedRetreat = { ...RetreatAngles, gripper: CloseDeg };
      const Withdrawal = PathContact(ClosedGrasp, ClosedRelease, CloseDeg, Context.Others, GraspRules.PlanMarginCm)
        ?? PathContact(ClosedRelease, ClosedRetreat, CloseDeg, Context.Others, GraspRules.PlanMarginCm);
      const Coverage = ScenePathCoverageReason({ ...HoverAngles, gripper: Open }, { ...GraspAngles, gripper: Open }, Context.Floor, Jaws)
        ?? ScenePathCoverageReason(GraspAngles, GraspAngles, Context.Floor, Array.from({ length: GraspRules.PathSamples + 1 }, (_, Index) => Open + (CloseDeg - Open) * Index / GraspRules.PathSamples))
        ?? ScenePathCoverageReason(ClosedGrasp, ClosedRelease, Context.Floor, JawPair(CloseDeg))
        ?? ScenePathCoverageReason(ClosedRelease, ClosedRetreat, Context.Floor, JawPair(CloseDeg));
      const Carried = CarriedObjectPathReason(Object_, GraspAngles, ClosedGrasp, ClosedRelease, Context.Others, Context.Floor)
        ?? CarriedObjectPathReason(Object_, GraspAngles, ClosedRelease, ClosedRetreat, Context.Others, Context.Floor);
      if (Withdrawal || Coverage || Carried) {
        Failed ??= Fail(5, "path", Coverage ?? Carried ?? `withdrawing the gripper would touch ${Withdrawal!.Label}`); continue;
      }
      if (SideEntry(Context)) Approach = { Kind: "side", Pregrasp: Hover, Release, Retreat, Pitch };
    }
    const Info = Result.Info;
    if (Warnings) {
      // Where the printed grasp point and the way down to it bring a part of the gripper near the object it grasps.
      const AtGrasp = DeepestContact(GripperSamples({ ...GraspAngles, gripper: Open }, Jaws), TaskList, GraspRules.ContactMarginCm, Own(GraspRules.ContactMarginCm), Tasks);
      const OnTheWay = AtGrasp ? null : PathContact({ ...HoverAngles, gripper: Open }, { ...GraspAngles, gripper: Open }, Open, TaskList, GraspRules.PlanMarginCm,
        GraspRules.PathSamples, Jaws, Own(GraspRules.PlanMarginCm), Tasks);
      const Touch = AtGrasp ?? OnTheWay;
      if (Touch) Warn(Warnings, `the ${PartName[Touch.Part]} may push it${OnTheWay && OnTheWay.S < 1 ? " on the way down" : ""}`);
    }
    const Closing: GraspClosing = Sweep?.Ok ? { Mode: "sweep", PushDeg: Sweep.PushDeg!, GripDeg: Sweep.GripDeg!, Grip3DDeg: Sweep.Grip3DDeg, TipCm: Sweep.TipCm ?? null,
      ContactDeg: Sweep.ContactDeg!, DriftCm: Sweep.DriftCm ?? 0, DepthOffsetCm: Info.DepthOffsetCm ?? 0,
      DiameterCm: Object_.Round ? [Object_.Round.LoCm, Object_.Round.HiCm] : null } : { Mode: "table" };
    return { Ok: true, Plan: { Prism: Object_, Pitch, HoverPitch, Grasp, Hover, ...(Approach ? { Approach } : {}), JawAxis: Info.Uh, Side: SideOf(Info.Uh),
      OffsetCm: Math.hypot(Grasp.x - Object_.Centre.x, Grasp.y - Object_.Centre.y), WidthEffCm: Info.WEff, WidthAcrossCm: WidthAcross(Object_, Info.Uh),
      HeightBy: Info.HeightBy, TopAboveToolCm: Info.TopAboveTool ?? 0, OpenFraction: Fraction, Closing, Ms: Now() - Started,
      Poses: { Hover: { ...HoverAngles, gripper: Open }, Grasp: { ...GraspAngles, gripper: Open }, JawDegs: Jaws },
      ...(Offset || Sag ? { FingerOffsetCm: Offset, SagCm: Sag } : {}), ...PlanGap(Info.ClearanceCm ?? ClearanceFor(Context, Corrected), GapSetOf(Context)),
      ...(Straddle ? { Straddle } : {}), ...(Warnings ? { Warnings } : {}), ...(Info.Deep ? DeepGaps({ ...Result.Angles, gripper: Open }, TaskList) : {}) } };
  }
  return Failed ?? Fail(6, "lean", `at wrist ${Deg(Pitch)} deg, at the rounded grasp point, no open fraction the plan picked clears it`);
}

/**
 * A deep plan's gaps where its jaws close (the planned pose: the arm stops GraspSagCm under the printed grasp
 * point): to the palm and servo (PalmGapCm), and to the STS3215 servo alone (ServoGapCm).
 */
function DeepGaps(Angles: CadAngles, Prisms: Prism[]) {
  const Links = CadForward(Angles).Links;
  return { Deep: true, PalmGapCm: PalmGapTo(Links, Prisms), ServoGapCm: Math.min(...Prisms.map(Object_ => PalmGap(Links, Object_, true))) };
}

// The furthest stage the tried pitches reached (a planned one counts as furthest).
const BestTried = (Tried: PitchResult[]) => Tried.reduce<PitchResult | null>((A, B) => (!A || (B.Ok ? 99 : B.Stage) > (A.Ok ? 99 : A.Stage) ? B : A), null);
// "reach" only where the arm does not reach: a pose in the basket's or the body's way is "path".
function CodeOf(Best: PitchResult | null): GraspRefusalCode {
  return !Best || Best.Ok ? "reach" : Best.Code === "top-edge" || Best.Code === "lean" || Best.Code === "straddle" ? "wide"
    : Best.Code === "reach" && Best.Collision ? "path"
    : Best.Code === "verify" ? "path" : (["wide", "flat", "palm", "neighbour", "path", "reach"].includes(Best.Code) ? Best.Code as GraspRefusalCode : "path");
}

// How far both jaws of a plan reach past the object's centre (a plan without a straddle, M1: as far as it takes).
const StraddleReach = (Plan: GraspPlan) => Plan.Straddle ? Math.min(Plan.Straddle.FixedCm, Plan.Straddle.MovingCm) : Infinity;
const Reaches = (Plan: GraspPlan) => StraddleReach(Plan) >= GraspRules.StraddleGoodCm;
// A deep plan as deep as its palm lets it (GraspRules.DeepGrasp), within DeepLookCm: else SweepScan looks on, as for a short straddle.
const DeepEnough = (Plan: GraspPlan) => !Plan.Deep || (Plan.PalmGapCm ?? 0) <= GraspRules.PalmGapCm + GraspRules.DeepRoundingCm + GraspRules.DeepLookCm;
// The plan whose jaws reach farther around it (by more than the rounding), else the first; two deep plans: the one nearer the palm.
// (Two deep plans whose jaws both reach StraddleGoodCm around it: the one nearer the palm.)
const Deeper = (A: GraspPlan | null, B: GraspPlan | null) => (!A ? B : !B ? A
  : A.Deep && B.Deep && Reaches(A) && Reaches(B) ? ((B.PalmGapCm ?? Infinity) < (A.PalmGapCm ?? Infinity) - 0.05 ? B : A)
  : StraddleReach(B) > StraddleReach(A) + 0.05 ? B : A);
// Pitches nearer GraspRules.PreferredPitchDeg first; equally near, the steeper (the camera sees more of the object).
export const PreferredFirst = (A: number, B: number) => {
  const Preferred = GraspRules.PreferredPitchDeg ?? 0, Nearer = Math.abs(A - Preferred) - Math.abs(B - Preferred);
  return Math.abs(Nearer) > 1e-9 ? Nearer : A - B;
};
// The preferred pitch among pitches stepped from Steepest to level, when it is one of theirs.
const WithPreferred = (Pitches: number[], Steepest: number) => {
  const Preferred = GraspRules.PreferredPitchDeg;
  return Preferred != null && Preferred > Steepest + 1e-9 && Preferred <= 0 && !Pitches.some(Pitch => Math.abs(Pitch - Preferred) < 1e-6)
    ? [...Pitches, Preferred] : Pitches;
};

type RaiseAt = { Pitch: number; Sweep: boolean };
/**
 * No plan with the grasp point at mid-height: raised in RaiseStepCm steps (up
 * to MaxRaiseCm, and RaiseBelowTopCm under the top) at the given pitches, a
 * pitch at a time in their order, every check run again at each, the first
 * that plans. The fixed jaw goes down
 * on the side the pan puts it (the wrist roll is held), so a neighbour there
 * blocks it at every pitch at mid-height (2026-10-01 08:01: the 2.7 cm tall
 * battery beside the can's left-near side, cleared 0.5 cm higher); 08:00 the
 * same can, a little nearer the basket, had the gripper link at its rim at
 * wrist -19.1 and the grip too lean at -21.6 at mid-height, and planned 0.5
 * and 1 cm higher. The plan says how much higher and why (HeightBy "clear").
 * A sweep plan whose jaws reach short of StraddleGoodCm around it is tried at
 * the top raise too, the deeper kept (higher, the palm rises clear of its top
 * and lets it in deeper). Failures: where each raise
 * failed, for the refusal (a neighbour blocking the raise the palm needed).
 */
function RaisedPlan(Object_: Prism, At: RaiseAt[], Tried: PitchResult[], Context: GraspContext, Started: number, Failures?: PitchResult[]): GraspPlan | null {
  const Mid = (Object_.BaseZ + Object_.TopZ) / 2;
  const Why = (Pitch: number) => {
    const Failed = Tried.find(Result => !Result.Ok && Math.abs(Result.Pitch - Pitch) < 0.01);
    return Failed && !Failed.Ok ? Failed.Why.replace(/^at wrist -?[\d.]+ deg,? /, "") : null;
  };
  // Why a lower raise at a pitch failed, when not as at mid-height (the closing at the fingertip, say).
  const Lower = new Map<number, string>();
  const Raises: number[] = [];
  for (let Raise = GraspRules.RaiseStepCm; Raise <= GraspRules.MaxRaiseCm + 1e-9 && Mid + Raise <= Object_.TopZ - GraspRules.RaiseBelowTopCm; Raise += GraspRules.RaiseStepCm) Raises.push(Raise);
  // Off the desk, the pitches whose hover point or lift a raise already put out of reach (SurfaceWrist): higher is worse.
  const OutOfReach = new Set<number>();
  const Try = (Pitch: number, Sweep: boolean, Raise: number): GraspPlan | null => {
    if (OutOfReach.has(Pitch)) return null;
    const Placed = PlaceAtPitch(Object_, Pitch, Context, Sweep, Infinity, Raise);
    const Printed = Placed.Ok ? PrintedPlan(Object_, Placed, Context, Started) : Placed;
    if (!Printed.Ok) {
      Lower.set(Pitch, `at ${Raise.toFixed(1)} cm above it, ${Printed.Why.replace(/^at wrist -?[\d.]+ deg,? /, "")}`);
      Failures?.push(Printed);
      if (Printed.SurfaceWrist) OutOfReach.add(Pitch);
      return null;
    }
    const Below = Lower.get(Pitch), Mid_ = Why(Pitch);
    const For = [Mid_, Below && !(Mid_ && Below.endsWith(Mid_)) ? Below : null].filter(Boolean).join("; ") || null;
    const Plan = Printed.Plan.HeightBy === "clear" ? { ...Printed.Plan, RaiseCm: Raise, RaisedFor: For } : Printed.Plan;
    Lower.set(Pitch, `at ${Raise.toFixed(1)} cm above it, the jaws would reach only ${StraddleReach(Plan).toFixed(1)} cm past its centre`);
    return Plan;
  };
  // As before (GraspRules.PreferredPitchDeg null): a raise at a time over the pitches, a plan short of StraddleGoodCm
  // tried at the top raise at each, the deepest kept.
  if (GraspRules.PreferredPitchDeg == null) {
    for (const Raise of Raises) {
      for (const { Pitch, Sweep } of At) {
        const Plan = Try(Pitch, Sweep, Raise);
        if (!Plan || Reaches(Plan)) { if (Plan) return Plan; continue; }
        let Best: GraspPlan | null = Plan;
        const Top = Raises[Raises.length - 1];
        if (Top > Raise + 1e-9) for (const Other of At) Best = Deeper(Best, Try(Other.Pitch, Other.Sweep, Top));
        return Best;
      }
    }
    return null;
  }
  // A pitch at a time (At comes nearest the preferred pitch first), each raise up.
  for (const { Pitch, Sweep } of At) {
    for (const Raise of Raises) {
      const Plan = Try(Pitch, Sweep, Raise);
      if (!Plan) continue;
      // Short of StraddleGoodCm around it: at the highest raise too (higher, the palm rises clear of its top and lets it
      // in deeper), the deeper kept.
      const Top = Raises[Raises.length - 1];
      return Reaches(Plan) || Top <= Raise + 1e-9 ? Plan : Deeper(Plan, Try(Pitch, Sweep, Top));
    }
  }
  return null;
}

/**
 * The advice for a grasp still refused for a neighbour: how far it must move
 * straight away from the object for a pitch it blocked to plan (a cm at a
 * time up to NeighbourMoveMaxCm, at mid-height or a little higher), confirmed
 * by planning the object again with it moved that far; without a confirmed
 * distance, none is given.
 */
function NeighbourAdvice(Object_: Prism, CentrePitch: number, Blocked: Extract<PitchResult, { Ok: false }>[], Context: GraspContext, Started: number): string | null {
  const Other = Blocked[0]?.Info?.Other;
  if (!Other) return null;
  const Dx = Other.Prism.Centre.x - Object_.Centre.x, Dy = Other.Prism.Centre.y - Object_.Centre.y, R = Math.hypot(Dx, Dy) || 1;
  const Where = `(the fixed jaw goes down on its ${SideOf(Blocked[0].Info!.Uh)} side, where the ${Other.Label} is)`;
  const MovedAway = (Cm: number): GraspContext => {
    const Ux = Cm * Dx / R, Uy = Cm * Dy / R;
    const Moved: Prism = { ...Other.Prism, Centre: { x: Other.Prism.Centre.x + Ux, y: Other.Prism.Centre.y + Uy },
      Footprint: Other.Prism.Footprint.map(P => ({ x: P.x + Ux, y: P.y + Uy })) };
    return { ...Context, Others: Context.Others.map(P => (P === Other.Prism ? Moved : P)) };
  };
  const Places = (Ctx: GraspContext, Raise: number) => Blocked.slice(0, 2).some(Result => {
    const Placed = PlaceAtPitch(Object_, Result.Pitch, Ctx, !!Result.Info?.Sweep, Infinity, Raise);
    return Placed.Ok && PrintedPlan(Object_, Placed, Ctx, Started).Ok;
  });
  for (let Cm = 1; Cm <= GraspRules.NeighbourMoveMaxCm; Cm++) {
    // (Or at the top raise: where the palm lets a round object's jaws around it, RaisedPlan.)
    if (![0, GraspRules.RaiseStepCm, ...(GraspRules.StraddleRoundObjects && Object_.Round ? [GraspRules.MaxRaiseCm] : [])].some(Raise => Places(MovedAway(Cm), Raise))) continue;
    if (!PlanGrasp(Object_, CentrePitch, MovedAway(Cm), false).Plan) break;
    return `Move the ${Other.Label} at least ${Cm} cm farther from the ${Object_.Label} ${Where}, then send the task again.`;
  }
  return `Move the ${Other.Label} away from the ${Object_.Label} ${Where}, then send the task again.`;
}

/**
 * The grasp plan for a measured object. M1: the pitch nearest
 * GraspRules.PreferredPitchDeg (from CentrePitch, the steepest the reach scan
 * reaches at its centre, mid-height, toward level) whose grasp point, hover
 * point and the descent between them pass every check, the closing judged by
 * the CAD jaw table. M3: an object that table finds too wide somewhere (a
 * can) is planned with the jaw-plane sweep instead (SweepScan), at the held
 * pitch nearest the preferred one. Else the refusal of the stage the best
 * pitch reached. Advise: work out how much closer an out-of-reach object
 * would be planned (off when checking that). Not planned at mid-height:
 * raised (RaisedPlan) at the sweep's held pitches, or where a neighbour
 * blocked it; still refused for a neighbour, the advice says how far it must
 * move (NeighbourAdvice).
 * First with the strict closing (Strict: a round object's 3D grip and
 * fingertip margin, the closing at the printed point), which prefers a
 * flatter wrist, a deeper head start or a higher grasp that leaves the object
 * well inside the fixed finger's tip (2026-10-01 15:08: the can, planned with
 * its contact at the tip, was squeezed out past it); none: as before, and the
 * plan's TipCm and Grip3DDeg say what it lacks (the model is told).
 */
export function PlanGrasp(Object_: Prism, CentrePitch: number, Context: GraspContext, Advise = true): PlanResult {
  if (Context.Floor?.Scene && !Context.Floor.Scene.free_space?.length) {
    return { Plan: null, Refusal: { Code: "surface", Text: "the depth camera has not observed a free approach corridor", Advice: "Expose the approach and withdrawal space to the depth camera before planning this grasp." }, Tried: [] };
  }
  const Conflict = SceneConflict(Context.Floor?.Scene);
  if (Conflict) {
    const At = `X ${Fixed1((Conflict.Low[0] + Conflict.High[0]) / 2)}, Y ${Fixed1((Conflict.Low[1] + Conflict.High[1]) / 2)}, Z ${Fixed1(Conflict.Low[2])} to ${Fixed1(Conflict.High[2])}`;
    return { Plan: null, Refusal: { Code: "path", Text: `the camera measured the ${Object_.Label} running into ${Conflict.Box.label} beside it (at ${At}), so the two were not measured apart`,
      Advice: `Move the ${Object_.Label} 2 to 3 cm away from what stands beside it, then send the task again.` }, Tried: [] };
  }
  const Scene = ScenePrisms(Context.Floor);
  if (Scene.length) Context = { ...Context, Others: [...Context.Others.filter(P => !P.Scene), ...Scene] };
  if (PlaceCache) return PlanGraspStrictFirst(Object_, CentrePitch, Context, Advise);
  PlaceCache = new Map();
  try { return PlanGraspStrictFirst(Object_, CentrePitch, Context, Advise); } finally { PlaceCache = null; }
}
// Within one PlanGrasp: placements whose failure the closing's strictness does not change (out of reach, in the basket's
// way, the jaws cannot surround it), by object, context, pitch, raise and mode: its second pass and its raises reuse them.
let PlaceCache: Map<Prism, Map<GraspContext, Map<string, PitchResult>>> | null = null;
function PlaceCacheFor(Object_: Prism, Context: GraspContext): Map<string, PitchResult> | null {
  if (!PlaceCache) return null;
  let ByContext = PlaceCache.get(Object_);
  if (!ByContext) PlaceCache.set(Object_, ByContext = new Map());
  let ByKey = ByContext.get(Context);
  if (!ByKey) ByContext.set(Context, ByKey = new Map());
  return ByKey;
}
function PlanGraspStrictFirst(Object_: Prism, CentrePitch: number, Context: GraspContext, Advise: boolean): PlanResult {
  const Result = PlanGraspPasses(Object_, CentrePitch, Context, Advise);
  // No plan: the best-effort pass, unless the refusal is certain (an object thinner than the closed pads, or unmeasured).
  // Not for the advice's searches (Advise false: where the object, or a neighbour, would plan with every check).
  if (Result.Plan || !Advise || !GraspRules.BestEffort || Lenient || Result.Refusal.Code === "thin" || Result.Refusal.Code === "unmeasured") return Result;
  const Failures: PitchResult[] = [];
  const Plan = BestEffortPlan(Object_, CentrePitch, Context, Failures);
  if (Plan) return { Plan, Refusal: null, Tried: Result.Tried };
  // Nothing at all, with every check only what is certain left: the physical reason, from where the best-effort pass failed.
  return { ...Result, Refusal: ImpossibleRefusal(Object_, Failures, Result.Refusal) };
}
/**
 * GRASP ANYTHING (2026-10-18): the refusal of an object the best-effort pass found no grasp for, in physical terms: what
 * stopped it at every wrist pitch and height it tried (out of the arm's reach, the arm in the basket's or the body's way,
 * a neighbour in the gripper's way, the desk, or too wide to go between the open jaws). The two strict passes' refusal
 * (Before) keeps its advice where it is the same kind (how much closer, which neighbour to move).
 */
export function ImpossibleRefusal(Object_: Prism, Failures: PitchResult[], Before: GraspRefusal): GraspRefusal {
  const Failed = Failures.filter((Item): Item is Extract<PitchResult, { Ok: false }> => !Item.Ok);
  if (!Failed.length) return Before;
  type Kind = "reach" | "collision" | "neighbour" | "wide" | "contact";
  // (The best-effort pass judges the printed points and the descent against the neighbours only: a part "would come
  // within" a cm "of the" something there is a neighbour in the gripper's way.)
  const KindOf = (Item: Extract<PitchResult, { Ok: false }>): Kind => Item.Code === "reach" ? (Item.Collision ? "collision" : "reach")
    : Item.Code === "neighbour" || / would come within -?[\d.]+ cm of the /.test(Item.Why) ? "neighbour" : Item.Code === "wide" ? "wide" : "contact";
  const Groups = new Map<string, { Kind: Kind; Pitches: number[]; Why: string; Stage: number }>();
  for (const Item of Failed) {
    const Kind = KindOf(Item);
    const Key = Kind === "collision" ? `collision:${Item.Collision}` : Kind === "neighbour" ? `neighbour:${Item.Info?.Other?.Label ?? ""}` : Kind;
    const Group = Groups.get(Key) ?? { Kind, Pitches: [], Why: Item.Why.replace(/^at wrist -?[\d.]+ deg,? (at the grasp point beside it, )?/, ""), Stage: 0 };
    Group.Pitches.push(Item.Pitch);
    Group.Stage = Math.max(Group.Stage, Item.Stage);
    Groups.set(Key, Group);
  }
  const Range = (Pitches: number[]) => { const Lo = Math.min(...Pitches), Hi = Math.max(...Pitches); return Lo === Hi ? `wrist ${Deg(Lo)} deg` : `wrist ${Deg(Lo)}..${Deg(Hi)} deg`; };
  const Parts_ = [...Groups.values()].sort((A, B) => B.Stage - A.Stage || B.Pitches.length - A.Pitches.length).map(Group =>
    Group.Kind === "reach" ? `the arm does not reach a grasp point beside it at ${Range(Group.Pitches)}`
      : Group.Kind === "wide" ? `it does not fit between the jaws fully open at ${Range(Group.Pitches)} (${Group.Why})`
      : `at ${Range(Group.Pitches)} ${Group.Why}`);
  // The code: what stopped the placements that got furthest (a pose the arm reaches that does not fit between the jaws
  // says more than the poses it does not reach).
  const Furthest = Math.max(...Failed.map(Item => Item.Stage));
  const Kinds = new Set(Failed.filter(Item => Item.Stage === Furthest).map(KindOf));
  const Code: GraspRefusalCode = Kinds.has("collision") ? "path" : Kinds.size === 1 && Kinds.has("reach") ? "reach"
    : Kinds.has("neighbour") ? "neighbour" : Kinds.size === 1 && Kinds.has("wide") ? "wide" : "path";
  const Advice = Before.Code === Code ? Before.Advice
    : Code === "path" ? `Move the ${Object_.Label} farther from the basket and the robot's body (or closer to the arm), then send the task again.`
    : Code === "reach" ? `Move the ${Object_.Label} closer to the right arm, then send the task again.`
    : Code === "neighbour" ? `Move what stands beside the ${Object_.Label} away from it, then send the task again.`
    : `Try a narrower object, or turn the ${Object_.Label} so its narrow side faces the jaws.`;
  const Text = `no grasp is physically possible from here: every wrist pitch and grasp height tried (best effort, every grip that may slip allowed) is blocked: ${Parts_.slice(0, 3).join("; ")}`;
  // The camera's own check (the way to the grasp through space it has not observed) stands first, as the strict passes said it.
  if (/depth camera has not observed/.test(Before.Text)) return { ...Before, Text: `${Before.Text}; and ${Text}` };
  return { Code, Text,
    Advice, ...(Code === Before.Code && Before.CloserCm !== undefined ? { CloserCm: Before.CloserCm } : {}), ...(Before.GapCm != null ? { GapCm: Before.GapCm } : {}) };
}
function PlanGraspPasses(Object_: Prism, CentrePitch: number, Context: GraspContext, Advise: boolean): PlanResult {
  if (!Strict || !GraspRules.StrictClosing) {
    const Saved = Strict;
    Strict = false;
    try { return PlanGraspPass(Object_, CentrePitch, Context, Advise); } finally { Strict = Saved; }
  }
  const First = PlanGraspPass(Object_, CentrePitch, Context, false);
  if (First.Plan) return First;
  Strict = false;
  try {
    return PlanGraspPass(Object_, CentrePitch, Context, Advise);
  } finally {
    Strict = true;
  }
}
/**
 * THE BEST-EFFORT PASS (GraspRules.BestEffort): the two passes found no plan. Planned again (Lenient, not
 * Strict) with only the certain checks as refusals, at every GraspRules.PitchStepDeg from CentrePitch to
 * level (nearest the preferred pitch first), each at mid-height, raised a cm at a time up to MaxRaiseCm and
 * lowered down to LowerCm, the jaws' sweep judging any width; a round object also as measured (its padded
 * box: a 5.3 x 2.8 cm reading is not a 7.1 cm disc for sure). The plan with the fewest warnings, the first
 * found among equals, within its work (BestEffortPlacements, and BestEffortSolveWork as BestEffortWorkDone counts
 * it): the same plan on an idle PC and a loaded one. Null: nothing the arm reaches clear of the basket, the desk and
 * the neighbours holds it between the jaws within that work, or the search ran past BestEffortRefuseMs (refused,
 * never a partial pick).
 */
function BestEffortPlan(Object_: Prism, CentrePitch: number, Context: GraspContext, Failures?: PitchResult[]): GraspPlan | null {
  const Started = Now();
  const Budget = BestEffortPlacements(Context);
  const Saved = [Strict, Lenient] as const;
  Strict = false;
  Lenient = true;
  try {
    const Mid = (Object_.BaseZ + Object_.TopZ) / 2;
    // GRASP ANYTHING (2026-10-18, Aaran: "make it so that it can grab anything, unless it's genuinely impossible"): every
    // grasp height from just above the support (BestEffortFloorCm) to just under the top (BestEffortTopCm), mid-height
    // first, then BestEffortHeightStepCm at a time above and below it, nearest first. The desk floor bound (PlaceAtPitch)
    // still lifts a point too low for the jaws. The heights the pass tried before come first (a cm at a time up to
    // MaxRaiseCm, then down to LowerCm, RaiseBelowTopCm from the top and base), so what planned there plans the same.
    const Raises = [0];
    for (let Raise = 1; Raise <= GraspRules.MaxRaiseCm + 1e-9 && Mid + Raise <= Object_.TopZ - GraspRules.RaiseBelowTopCm; Raise++) Raises.push(Raise);
    for (let Lower = 1; Lower <= GraspRules.LowerCm + 1e-9 && Mid - Lower >= Object_.BaseZ + GraspRules.RaiseBelowTopCm; Lower++) Raises.push(-Lower);
    // The rest of the object's height (the second search, only when the first finds nothing).
    const Up = Object_.TopZ - GraspRules.BestEffortTopCm - Mid, Down = Mid - Object_.BaseZ - GraspRules.BestEffortFloorCm;
    const More: number[] = [];
    const Extra = (Raise: number) => { if ([...Raises, ...More].every(Other => Math.abs(Other - Raise) > 0.25)) More.push(Number(Raise.toFixed(3))); };
    for (let K = 1; K * GraspRules.BestEffortHeightStepCm <= Math.max(Up, Down) + 1e-9; K++) {
      const Step = K * GraspRules.BestEffortHeightStepCm;
      if (Step <= Up + 1e-9) Extra(Step);
      if (Step <= Down + 1e-9) Extra(-Step);
    }
    // (And the highest and lowest themselves: a short object's whole height in a step or less.)
    for (const Edge of [Up, -Down]) if (Math.abs(Edge) > 0.05) Extra(Edge);
    const Pitches: number[] = [];
    for (let Pitch = CentrePitch; Pitch <= PitchCeiling(Context) + 1e-9; Pitch += GraspRules.PitchStepDeg) Pitches.push(Number(Pitch.toFixed(4)));
    const Order = GraspRules.PreferredPitchDeg == null ? Pitches : WithPreferred(Pitches, CentrePitch).sort(PreferredFirst);
    const Shapes: { Prism: Prism; Note: string | null }[] = [{ Prism: Object_, Note: null }];
    const Measured = Object_.Round && Object_.SizeCm ? MeasuredBox(Object_) : null;
    if (Measured) {
      Shapes.push({ Prism: Measured, Note: `planned as measured (${Object_.SizeCm![0].toFixed(1)} x ${Object_.SizeCm![1].toFixed(1)} cm), not as a round object`
        + ` up to ${Object_.Round!.HiCm.toFixed(1)} cm across: if it is that wide, the jaws may not hold it` });
    }
    let Best: { Plan: GraspPlan; Score: number } | null = null;
    // One search over the shapes, the pitches and these heights, within Times its work.
    const Search = (Heights: number[], Times: number) => {
      const Worked = BestEffortWorkDone();
      let Placements = 0;
      Search: for (const Shape of Shapes) {
        for (const Pitch of Order) {
          // The clock is a backstop only: past it, the best plan found so far (none: refused with the physical reason).
          if (Now() - Started > GraspRules.BestEffortRefuseMs) break Search;
          for (const Raise of Heights) {
            if (++Placements > Budget * Times || Worked() > GraspRules.BestEffortSolveWork * Times) break Search;
            const Placed = PlaceAtPitch(Shape.Prism, Pitch, Context, true, Infinity, Raise);
            if (!Placed.Ok) { Failures?.push(Placed); continue; }
            const Printed = PrintedPlan(Shape.Prism, Placed, Context, Started);
            if (!Printed.Ok) { Failures?.push(Printed); continue; }
            const Warnings = [...(Shape.Note ? [Shape.Note] : []), ...(Printed.Plan.Warnings ?? [])];
            const Plan: GraspPlan = { ...Printed.Plan, Warnings, Ms: Now() - Started,
              ...(Raise < 0 ? { HeightBy: "clear" as HeightBy, RaiseCm: Raise, RaisedFor: null } : Raise > 0 && Printed.Plan.HeightBy === "clear" ? { RaiseCm: Raise, RaisedFor: null } : {}) };
            const Verdict = Placed.Info.Sweep;
            const Score = BestEffortScore(Warnings, Verdict && !Verdict.Ok ? Verdict.GripDeg ?? 90 : 0);
            if (!Best || Score < Best.Score - 1e-9) Best = { Plan, Score };
            break;
          }
          if (Best && Best.Score === 0) break Search;
        }
      }
    };
    // The heights the pass tried before, within its work: what planned there plans the same. Nothing there: the rest of
    // the object's height, from mid-height out, within BestEffortNoPlanWork times that work (still counted, never timed).
    Search(Raises, 1);
    if (!Best && More.length && Now() - Started <= GraspRules.BestEffortRefuseMs) Search(More, GraspRules.BestEffortNoPlanWork);
    const Found = Best as { Plan: GraspPlan; Score: number } | null;
    return Found?.Plan ?? null;
  } finally {
    [Strict, Lenient] = Saved;
  }
}
/**
 * The best-effort pass's work from now on, as a function: IK solves (robotModel IkWork), each
 * GraspRules.BestEffortSweepPairsPerSolve vertex-edge pairs of the closing sweep (JawSweep SweepWork) one more.
 * Counted, never timed: the same search does the same work on an idle PC and a loaded one.
 */
export function BestEffortWorkDone(): () => number {
  const Solves = IkWork.Solves, Pairs = SweepWork.Pairs;
  return () => IkWork.Solves - Solves + (SweepWork.Pairs - Pairs) / GraspRules.BestEffortSweepPairsPerSolve;
}
/** How many placements the best-effort pass may try with these neighbours (GraspRules.BestEffortWork): its input, never the clock. */
export function BestEffortPlacements(Context: Pick<GraspContext, "Others">): number {
  const Each = 1 + Context.Others.length / GraspRules.BestEffortPrismsPerPlacement;
  return Math.max(GraspRules.BestEffortMinPlacements, Math.floor(GraspRules.BestEffortWork / Each));
}
/**
 * How doubtful a best-effort plan is (lower first): each warning 1, a grip that may slip 2 and the
 * friction it needs (GripDeg, a thousandth a degree: the gentler grip wins among equals), jaws that may
 * not reach around it 1.5.
 */
function BestEffortScore(Warnings: string[], GripDeg: number): number {
  return Warnings.reduce((Sum, Why) => Sum + (Why.startsWith("the grip may slip") ? 2 : Why.startsWith("the jaws may not reach") ? 1.5 : 1), 0) + GripDeg / 1000;
}
/** A round object as measured: its padded box (ObjectPrism), not the disc of its widest plausible size. */
function MeasuredBox(Object_: Prism): Prism {
  const C = Object_.Centre, Hx = Object_.SizeCm![0] / 2 + GraspRules.TrimPadCm, Hy = Object_.SizeCm![1] / 2 + GraspRules.TrimPadCm;
  return { ...Object_, Round: null, Footprint: [{ x: C.x - Hx, y: C.y - Hy }, { x: C.x + Hx, y: C.y - Hy }, { x: C.x + Hx, y: C.y + Hy }, { x: C.x - Hx, y: C.y + Hy }] };
}
function PlanGraspPass(Object_: Prism, CentrePitch: number, Context: GraspContext, Advise: boolean): PlanResult {
  const Started = Now();
  const Tried: PitchResult[] = [];
  // Refused for a neighbour: raised clear of it at the pitches it blocked (unless the sweep's raise tried them), else the advice.
  // Refused for clearance at mid-height (a neighbour, the basket at a reached pose, the descent or the printed
  // point): raised at the first such pitches the sweep's raise did not try (RaisedPlan); a neighbour still in
  // the way gets the advice. Too thin, too flat or unmeasured: rising does not help.
  const RaisedAt = new Set<string>();
  let SweepOnly = false;
  const Refused = (Refusal: GraspRefusal): PlanResult => {
    if (Refusal.Code === "thin" || Refusal.Code === "flat" || Refusal.Code === "unmeasured") return { Plan: null, Refusal, Tried };
    const Failed = Tried.filter((Result): Result is Extract<PitchResult, { Ok: false }> => !Result.Ok);
    const Clearance = Failed.filter(Result => (Result.Code === "neighbour" && !!Result.Info?.Other) || (Result.Stage === 0 && !!Result.Collision) || Result.Stage >= 5);
    const At = Clearance.map(Result => ({ Pitch: Result.Pitch, Sweep: !!Result.Info?.Sweep || SweepOnly }))
      .filter((Item, Index, All) => !RaisedAt.has(`${Item.Pitch}:${Item.Sweep}`) && All.findIndex(Other => Other.Pitch === Item.Pitch && Other.Sweep === Item.Sweep) === Index).slice(0, 3);
    const Plan = At.length ? RaisedPlan(Object_, At, Tried, Context, Started) : null;
    if (Plan) return { Plan, Refusal: null, Tried };
    if (Refusal.Code !== "neighbour") return { Plan: null, Refusal, Tried };
    const Blocked = Failed.filter(Result => Result.Code === "neighbour" && !!Result.Info?.Other);
    const Advice = Advise ? NeighbourAdvice(Object_, CentrePitch, Blocked, Context, Started) : null;
    return { Plan: null, Refusal: Advice ? { ...Refusal, Advice } : Refusal, Tried };
  };
  const First = PlaceAtPitch(Object_, CentrePitch, Context);
  if (First.Info && WidthAcross(Object_, First.Info.Uh) < GraspRules.ThinCm) return Refused(RefusalFor("thin", Object_, First));
  // W_eff hardly changes with pitch: too wide for the jaw table at the first is too wide at all.
  const WideAll = !First.Ok && First.Code === "wide" && (First.Info?.WEff ?? 0) > GraspRules.WideEarlyExitCm;
  SweepOnly = WideAll;
  if (WideAll) Tried.push(First);
  else {
    // The first pitch failed on reach after T moved beside the object: scan where the grasp is, not the centre.
    let Start = CentrePitch;
    if (!First.Ok && First.Code === "reach" && First.Info?.Estimate) {
      Start = Context.Scan(modelPointFromDisplayPoint({ ...First.Info.Estimate, z: (Object_.BaseZ + Object_.TopZ) / 2 })) ?? CentrePitch;
    }
    const Pitches: number[] = [];
    for (let Pitch = Start; Pitch <= PitchCeiling(Context) + 1e-9; Pitch += GraspRules.PitchStepDeg) Pitches.push(Pitches.length ? Number(Pitch.toFixed(4)) : Pitch);
    if (Start !== CentrePitch && First.Ok) Pitches.unshift(CentrePitch);
    // Nearest the preferred pitch first (GraspRules.PreferredPitchDeg, itself when the arm reaches that steep), not the steepest.
    for (const Pitch of GraspRules.PreferredPitchDeg == null ? Pitches : WithPreferred(Pitches, Start).sort(PreferredFirst)) {
      const Result = Pitch === CentrePitch ? First : PlaceAtPitch(Object_, Pitch, Context);
      Tried.push(Result);
      if (!Result.Ok) continue;
      const Printed = PrintedPlan(Object_, Result, Context, Started);
      if (Printed.Ok) return { Plan: Printed.Plan, Refusal: null, Tried };
      Tried[Tried.length - 1] = Printed;
    }
  }
  const TableBest = BestTried(Tried);
  // Only an object the jaw table found too wide somewhere gets the sweep: the table's plans stand.
  if (!Tried.some(Result => !Result.Ok && Result.Stage === 1)) return Refused(RefusalFor(CodeOf(TableBest), Object_, TableBest));
  // Every pitch the sweep holds it with, nearest the preferred first, at mid-height and raised (the wrist is the planner's to
  // choose: 2026-10-03 14:47:36 was refused at the first three).
  const RaisedFailed: PitchResult[] = [];
  const Swept = SweepScan(Object_, CentrePitch, Context, Started, RaisedFailed);
  Tried.push(...Swept.Tried);
  if (GraspRules.PreferredPitchDeg != null) {
    Swept.Held.forEach(Item => RaisedAt.add(`${Item.Pitch}:true`));
    if (Swept.Plan) return { Plan: Swept.Plan, Refusal: null, Tried };
  } else {
    // As before (PreferredPitchDeg null): the sweep's mid-height plan if its jaws reach StraddleGoodCm around it, else raised at
    // its first held pitches, the deeper kept, then every other held pitch RaisePitches at a time on the last pass.
    if (Swept.Plan && Reaches(Swept.Plan)) return { Plan: Swept.Plan, Refusal: null, Tried };
    const Held = Swept.Held.slice(0, GraspRules.RaisePitches).map(Item => ({ Pitch: Item.Pitch, Sweep: true }));
    Held.forEach(Item => RaisedAt.add(`${Item.Pitch}:true`));
    const Raised = Deeper(Swept.Plan, RaisedPlan(Object_, Held, Tried, Context, Started, RaisedFailed));
    if (Raised) return { Plan: Raised, Refusal: null, Tried };
    for (let From = GraspRules.RaisePitches; !Strict && From < Swept.Held.length; From += GraspRules.RaisePitches) {
      const Next = Swept.Held.slice(From, From + GraspRules.RaisePitches).map(Item => ({ Pitch: Item.Pitch, Sweep: true }));
      Next.forEach(Item => RaisedAt.add(`${Item.Pitch}:true`));
      const Other = RaisedPlan(Object_, Next, Tried, Context, Started, RaisedFailed);
      if (Other) return { Plan: Other, Refusal: null, Tried };
    }
  }
  const SweepBest = BestTried(Swept.Tried);
  // Raised, a neighbour is what blocked the jaws where the palm let them around it (a can beside a tall battery, the palm
  // clear of the can only 3 cm up, the fixed finger then at the battery): that neighbour and its advice, not the straddle.
  const RaisedBest = BestTried(RaisedFailed);
  if (RaisedBest && !RaisedBest.Ok && RaisedBest.Code === "neighbour" && RaisedBest.Info?.Other
    && (!SweepBest || SweepBest.Ok || RaisedBest.Stage > SweepBest.Stage)) {
    Tried.push(...RaisedFailed.filter(Result => !Result.Ok && Result.Code === "neighbour" && !!Result.Info?.Other));
    return Refused(RefusalFor("neighbour", Object_, RaisedBest));
  }
  // The sweep held it somewhere but the arm does not reach there: in the basket's way, or how much closer it would.
  if (Swept.Held.length && SweepBest && !SweepBest.Ok && SweepBest.Stage === 0) {
    const Collision = Swept.Tried.reduce<string | null>((Found, Result) => Found ?? (!Result.Ok && Result.Collision ? Result.Collision : null), null);
    const Closer = Advise && !Collision ? CloserToPlan(Object_, Swept.Held, Context) : null;
    return Refused(ReachRefusal(Object_, Swept.Held, Closer, Collision));
  }
  // The jaws cannot surround it (GraspRules.StraddleCm) where the sweep got furthest, or at every pitch it screened: that, not a short grasp.
  const Least = Swept.Least;
  const Straddled = SweepBest && !SweepBest.Ok && SweepBest.Code === "straddle" ? SweepBest
    : !SweepBest && Least && !Least.Verdict.Ok && Least.Verdict.Code === "straddle"
      ? { Ok: false as const, Pitch: Least.Pitch, Stage: 1, Code: "straddle", Why: `at wrist ${Deg(Least.Pitch)} deg ${Least.Verdict.Why}` } : null;
  if (Straddled) return Refused(RefusalFor("wide", Object_, Straddled));
  // The sweep got further than the table: its reason; else the table's, with what the sweep found.
  if (SweepBest && !SweepBest.Ok && SweepBest.Stage > 1) return Refused(RefusalFor(CodeOf(SweepBest), Object_, SweepBest));
  const SweepWhy = SweepBest && !SweepBest.Ok && SweepBest.Stage === 1 ? SweepBest.Why
    : Least && !Least.Verdict.Ok ? `at wrist ${Deg(Least.Pitch)} deg ${Least.Verdict.Why}` : null;
  return Refused(RefusalFor(CodeOf(TableBest), Object_, TableBest, SweepWhy));
}

// A pose with the arm's pan, lift and elbow kept and the wrist flexed to Pitch:
// the gripper's orientation at that pitch, for the quick screen (no IK).
function AtPitch(Base: CadAngles, Pitch: number): CadAngles {
  let Angles = { ...Base };
  for (let Step = 0; Step < 3; Step++) {
    const Now_ = CadPitch(Angles), Probe = CadPitch({ ...Angles, wrist_flex: Angles.wrist_flex + 1 });
    if (Math.abs(Now_ - Pitch) < 1e-3 || Math.abs(Probe - Now_) < 1e-6) break;
    Angles = { ...Angles, wrist_flex: Angles.wrist_flex + (Pitch - Now_) / (Probe - Now_) };
  }
  return Angles;
}
// ReachCm: how far both jaws would reach past its centre there (StraddleOf), the palm keeping its margin at mid-height.
type Screened = { Pitch: number; Verdict: SweepVerdict; T: Point3; ReachCm: number };
// How a screened pitch closes on the object: the friction angle its grip needs.
const ScreenLean = (Item: Screened) => Item.Verdict.GripDeg ?? Infinity;
/**
 * The jaw-plane sweep at every pitch from CentrePitch to level without IK:
 * the gripper turned to that pitch at the pan that reaches the object's
 * centre, the pad placed beside it at mid-height, the closing checked over
 * its plausible shapes where it was measured. Round objects do not turn with
 * the pan, so this is their verdict there; the rest of the check (depth
 * errors, the open jaw) comes with IK, and boxes get ScreenMarginDeg. A
 * round object's straddle head start there (StraddleDepths) is held to the
 * palm's margin from it at that pose too, the palm moved rigidly with the tool
 * (its face leans over the object as the wrist tips down: 2026-10-03's can
 * reached 1.7 cm around at wrist -21, 2.0 at -11 and flatter), and how far the
 * jaws then reach around it ranks the pitches (SweepScan).
 */
export function ScreenPitches(Object_: Prism, CentrePitch: number, Context: GraspContext): Screened[] {
  const Mid = (Object_.BaseZ + Object_.TopZ) / 2;
  const Base = solveIkPreview({ ...modelPointFromDisplayPoint({ ...Object_.Centre, z: Mid }), wristPitch: CentrePitch }, Context.ElbowMode, Context.Servos,
    Context.Limits ?? undefined).angles;
  // The nominal shape and the extremes of what it may be: the middling sizes come with IK.
  const Shapes = ClosingShapes(Object_).filter((Shape, Index, All) => Shape.Nominal || Index === 1 || Index === All.length - 1);
  const Out: Screened[] = [];
  const Grid: number[] = [];
  for (let Pitch = CentrePitch; Pitch <= PitchCeiling(Context) + 1e-9; Pitch += GraspRules.PitchStepDeg) Grid.push(Pitch);
  // (And the preferred pitch where it falls among them, GraspRules.PreferredPitchDeg.)
  for (const Pitch of WithPreferred(Grid, CentrePitch).sort((A, B) => A - B)) {
    const Frame = JawFrame({ ...AtPitch(Base, Pitch), gripper: OpenDeg(Context.Servos) });
    const Norm = Math.hypot(Frame.U.x, Frame.U.y), Uh = { x: Frame.U.x / Norm, y: Frame.U.y / Norm };
    // Where the real gripper meets it (PlaceAtPitch): the pad c outside that, the closing judged there.
    const { OffsetCm: Offset, On } = RealArmCorrections(Object_, true);
    const Real = ArmPrism(Object_, Uh, Offset);
    const { Txy } = PadPlace(Real, Uh, 0, ClearanceFor(Context, On));
    const Pad = { ...Txy, z: Mid };
    const At = Number(Pitch.toFixed(4));
    const Moved = Shapes.map(Shape => ({ ...Shape, Prism: ArmPrism(Shape.Prism, Uh, Offset) }));
    // Not round (a box): slid against the pad from its middle, where the head start along the fingers leaves it after the push.
    if (!StraddlesRound(Object_)) {
      Out.push({ Pitch: At, T: Pad, ReachCm: 0, Verdict: ClosingSweep(Real, { ...Frame, Tool: Pad }, Context.Servos, 0, Moved, true) });
      continue;
    }
    // A round object: the straddle's head starts PlaceAtPitch takes first (StraddleDepths, the palm's margin in the
    // jaws' plane: its 3D check may stop it shallower) and the least, each where the push's drift then leaves it:
    // the quick check slides it against the fixed finger there, the gentler kept.
    // (A deep plan, GraspRules.DeepGrasp: as deep as the palm's shadow leaves room for, then its exact gap decides.)
    const Window = StraddleWindow(Real, { ...Frame, Tool: Pad }, GraspRules.DeepGrasp ? 0.05 : undefined);
    if (!Window.Ok) {
      Out.push({ Pitch: At, T: Pad, ReachCm: 0, Verdict: { Ok: false, Code: "straddle", Why: Window.Why, PushDeg: null, GripDeg: null, Grip3DDeg: null, ContactDeg: null,
        DiameterCm: null, DriftCm: null, Opens: [] } });
      continue;
    }
    const Level = Math.hypot(Frame.A.x, Frame.A.y) || 1;
    const Along = (Cm: number) => ({ x: Pad.x + Cm / Level * Frame.A.x / Level, y: Pad.y + Cm / Level * Frame.A.y / Level, z: Mid });
    // The palm at the pose, moved rigidly to the tool point for a head start: the deepest that keeps its margin (bisected).
    const Palm = GripperSamples({ ...AtPitch(Base, Pitch), gripper: OpenDeg(Context.Servos) }, []).filter(Sample => Sample.Part === "PalmAndServo");
    const Links = CadForward({ ...AtPitch(Base, Pitch), gripper: OpenDeg(Context.Servos) }).Links;
    const PalmClear = (Cm: number) => {
      const To = Along(Cm), Dx = To.x - Frame.Tool.x, Dy = To.y - Frame.Tool.y, Dz = To.z - Frame.Tool.z;
      if (GraspRules.DeepGrasp) return PalmGap(Links, Real, false, { x: Dx, y: Dy, z: Dz }) >= GraspRules.PalmGapCm + GraspRules.DeepRoundingCm;
      return !DeepestContact(Palm.map(Sample => ({ ...Sample, P: { x: Sample.P.x + Dx, y: Sample.P.y + Dy, z: Sample.P.z + Dz } })), [Real], StraddlePalmCm());
    };
    let Deepest = Window.MaxCm;
    if (!PalmClear(Deepest)) {
      let Lo = Window.MinCm, Hi = Deepest;
      for (let Step = 0; Step < 6; Step++) { const Mid_ = (Lo + Hi) / 2; if (PalmClear(Mid_)) Lo = Mid_; else Hi = Mid_; }
      Deepest = Lo;
    }
    const First = GraspRules.DeepGrasp ? Deepest : StraddleDepths(Window.MinCm, Window.GoodCm, Deepest)[0];
    const Straddle = StraddleOf(SliceInJawPlane(Real, { ...Frame, Tool: Along(First) }));
    const ReachCm = Math.min(Straddle.FixedCm, Straddle.MovingCm);
    const Depths = First - Window.MinCm > 0.25 ? [First, Window.MinCm] : [First];
    const Judged = Depths.map(Depth => {
      const T = Along(Depth), Drift = NominalDrift(Real, { ...Frame, Tool: T }, Context.Servos, 0);
      return { Pitch: At, T, ReachCm, Verdict: ClosingSweep(Real, { ...Frame, Tool: Along(Depth - Drift) }, Context.Servos, 0, Moved, true) };
    });
    Out.push(Judged.reduce((A, B) => (ScreenLean(B) < ScreenLean(A) - 1e-9 ? B : A)));
  }
  return Out;
}
/**
 * M3: the jaw-plane sweep over the pitches the screen holds, each placed with
 * IK and checked like a table plan, at mid-height and then raised
 * (RaisedPlan): the first that plans. Those the screen held outright first
 * (before those held within ScreenMarginDeg: its mid-height palm is a guess
 * at a steep wrist), then those whose quick grip in 3D (GripIn3D: the jaws'
 * tilt against the standing side, which grows as the wrist tips down:
 * 2026-10-03's pill bottle 20.5 deg at wrist -20, 34 at -45, 49 at -60) is
 * within StrictScreenMarginDeg of the limit, nearest
 * GraspRules.PreferredPitchDeg first (PreferredFirst: steep, so the camera
 * sees the object past the gripper). PlanGrasp's strict pass tries only
 * those; its last pass (no pitch held it with the strict closing) then the
 * rest, the pitch whose neighbours (a step steeper and flatter, the arm's
 * wrist error) need the least friction in 3D first. Before 2026-10-03 every
 * pitch was ranked that way, and a round object's first by how far its jaws
 * reached around it: wrists near level (the can -9 to -21, the pill bottle
 * -25 to -35), where the gripper hid the object from the camera. A plan
 * whose jaws reach short of StraddleGoodCm around it is raised at its pitch,
 * the deeper kept.
 * Held: the pitches the screen held, Least: the screened pitch that closed
 * most gently. RaisedFailed: where the raises failed.
 */
function SweepScan(Object_: Prism, CentrePitch: number, Context: GraspContext, Started: number, RaisedFailed: PitchResult[]) {
  const Screen = ScreenPitches(Object_, CentrePitch, Context);
  const Lean = ScreenLean;
  const Holds = (Item: Screened) => (Item.Verdict.Ok || Item.Verdict.Code === "lean") && Lean(Item) <= SweepRules.MaxLeanDeg + GraspRules.ScreenMarginDeg;
  // A pitch and its held neighbours: the most 3D friction among them (a refused neighbour counts as the worst).
  const Robust = new Map<Screened, number>();
  Screen.forEach((Item, Index) => {
    const Near = [Screen[Index - 1], Item, Screen[Index + 1]].filter((Other): Other is Screened => !!Other);
    Robust.set(Item, Math.max(...Near.map(Other => Holds(Other) ? Other.Verdict.Grip3DDeg ?? Lean(Other) : Infinity)));
  });
  // Gentle: its screened 3D grip within StrictScreenMarginDeg of the limit (the strict pass tries no other: refused at
  // every height). Those nearest the preferred pitch first; the rest (the last pass) the gentlest first.
  const Gentle = (Item: Screened) => (Item.Verdict.Grip3DDeg ?? 0) <= SweepRules.MaxLeanDeg + GraspRules.StrictScreenMarginDeg;
  const Held = Screen.filter(Holds).sort((A, B) => (Number(B.Verdict.Ok) - Number(A.Verdict.Ok)) || (Number(Gentle(B)) - Number(Gentle(A)))
    || (Gentle(A) ? 0 : Robust.get(A)! - Robust.get(B)!) || PreferredFirst(A.Pitch, B.Pitch));
  const Least = Screen.reduce<Screened | null>((A, B) => (!A || Lean(B) < Lean(A) ? B : A), null);
  const Tried: PitchResult[] = [];
  if (GraspRules.PreferredPitchDeg == null) return { ...SweepScanBefore(Object_, Screen, Holds, Robust, Context, Started), Least };
  // (A round object on the strict pass: the first few, as before; its last pass tries them all.)
  const Tries = Strict ? Held.filter(Gentle).slice(0, Object_.Round ? 2 * GraspRules.RaisePitches : undefined) : Held;
  // A plan whose jaws reach short of StraddleGoodCm around it is kept while StraddleLookOn more pitches are tried: the
  // first that reaches it, else the deepest (2026-10-03 14:48: at wrist -21 the jaws reached 1.6 cm around the can, and
  // 0.5 cm off where the camera saw it the close left it 0.1 cm inside the fixed finger's tip; at -14, 2.0).
  let Best: GraspPlan | null = null, Left = GraspRules.StraddleLookOn;
  for (const Item of Tries) {
    if (Best && Left-- <= 0) break;
    const Result = PlaceAtPitch(Object_, Item.Pitch, Context, true);
    Tried.push(Result);
    let Plan: GraspPlan | null = null;
    if (Result.Ok) {
      const Printed = PrintedPlan(Object_, Result, Context, Started);
      if (Printed.Ok) Plan = Printed.Plan;
      else Tried[Tried.length - 1] = Printed;
    }
    // (Off the desk with the hover point or the lift out of reach at this wrist, higher is no better: SurfaceWrist.)
    const Last = Tried[Tried.length - 1];
    if ((!Plan || !Reaches(Plan)) && !(!Last.Ok && Last.SurfaceWrist)) {
      Plan = Deeper(Plan, RaisedPlan(Object_, [{ Pitch: Item.Pitch, Sweep: true }], Tried, Context, Started, RaisedFailed));
    }
    // (A deep plan short of its palm, DeepEnough: the deepest of those tried meanwhile.)
    const Kept = Best as GraspPlan | null;
    if (Plan && Reaches(Plan) && DeepEnough(Plan)) return { Plan: Plan.Deep && Kept?.Deep ? Deeper(Kept, Plan)! : Plan, Tried, Held, Least };
    if (Plan) Best = Deeper(Best, Plan);
  }
  return { Plan: Best, Tried, Held, Least };
}
/**
 * SweepScan as before 2026-10-03 (GraspRules.PreferredPitchDeg null, the
 * replays of 10-01): a round object's pitches first by how far its jaws reach
 * around it up to StraddleGoodCm in quarter cm (held outright before held
 * within ScreenMarginDeg), then the pitch whose neighbours need the least 3D
 * friction (the 16:49 can 19.5 deg at wrist -25, 24.6 at 0), the middle of
 * its run of held pitches, its own; each at mid-height only (PlanGraspPass
 * raises the first few), a plan short of StraddleGoodCm kept while
 * StraddleLookOn more are tried, the deepest kept; on the strict pass a round
 * object's first 2 RaisePitches only.
 */
function SweepScanBefore(Object_: Prism, Screen: Screened[], Holds: (Item: Screened) => boolean, Robust: Map<Screened, number>,
  Context: GraspContext, Started: number) {
  const Lean = ScreenLean;
  const Runs: Screened[][] = [];
  Screen.forEach((Item, Index) => { if (!Holds(Item)) return; if (Index && Holds(Screen[Index - 1])) Runs[Runs.length - 1].push(Item); else Runs.push([Item]); });
  const Middle = new Map<Screened, number>();
  for (const Run of Runs) for (const Item of Run) Middle.set(Item, Math.abs(Item.Pitch - (Run[0].Pitch + Run[Run.length - 1].Pitch) / 2));
  const Around = (Item: Screened) => StraddlesRound(Object_) ? (Item.Verdict.Ok ? 10 : 0) + Math.round(Math.min(Item.ReachCm, GraspRules.StraddleGoodCm) * 4) / 4 : 0;
  const Held = Runs.flat().sort((A, B) => (Around(B) - Around(A)) || (Robust.get(A)! - Robust.get(B)!) || (Middle.get(A)! - Middle.get(B)!) || Lean(A) - Lean(B));
  const Tried: PitchResult[] = [];
  let Best: GraspPlan | null = null, Left = GraspRules.StraddleLookOn;
  for (const Item of Strict && Object_.Round ? Held.slice(0, 2 * GraspRules.RaisePitches) : Held) {
    if (Best && Left-- <= 0) break;
    const Result = PlaceAtPitch(Object_, Item.Pitch, Context, true);
    Tried.push(Result);
    if (!Result.Ok) continue;
    const Printed = PrintedPlan(Object_, Result, Context, Started);
    if (Printed.Ok && Reaches(Printed.Plan)) return { Plan: Printed.Plan, Tried, Held };
    if (Printed.Ok) Best = Deeper(Best, Printed.Plan);
    else Tried[Tried.length - 1] = Printed;
  }
  return { Plan: Best, Tried, Held };
}

// The pan axis in the display frame, and the unit vector from it to a point.
const PanAxis = () => mirrorDisplayPoint({ x: 0, y: 0, z: 0 });
function FromPan(Point: XY) {
  const Pan = PanAxis(), Dx = Point.x - Pan.x, Dy = Point.y - Pan.y, R = Math.hypot(Dx, Dy) || 1;
  return { R, Unit: { x: Dx / R, y: Dy / R } };
}
const MovedToward = (Object_: Prism, Cm: number): Prism => {
  const { Unit } = FromPan(Object_.Centre), Dx = -Cm * Unit.x, Dy = -Cm * Unit.y;
  return { ...Object_, Centre: { x: Object_.Centre.x + Dx, y: Object_.Centre.y + Dy }, Footprint: Object_.Footprint.map(P => ({ x: P.x + Dx, y: P.y + Dy })) };
};
/**
 * How much closer to the arm (cm, straight toward its pan axis) an object the
 * sweep holds only where the arm does not reach must stand: the first whole
 * cm (up to 15) at which a held pitch reaches its screened grasp point, then
 * confirmed by planning it there (up to 3 cm more). Null: none found.
 */
function CloserToPlan(Object_: Prism, Held: Screened[], Context: GraspContext): number | null {
  const Pitches = Held.slice(0, 4);
  const Reaches = (Cm: number) => Pitches.some(Item => {
    const { Unit } = FromPan(Object_.Centre);
    const T = { x: Item.T.x - Cm * Unit.x, y: Item.T.y - Cm * Unit.y, z: Item.T.z };
    return PreviewConverges(solveIkPreview({ ...modelPointFromDisplayPoint(T), wristPitch: Item.Pitch }, Context.ElbowMode, Context.Servos,
      Context.Limits ?? undefined), Item.Pitch);
  });
  let First: number | null = null;
  for (let Cm = 1; Cm <= 15; Cm++) if (Reaches(Cm)) { First = Cm; break; }
  if (First == null) return null;
  for (let Cm = First; Cm <= First + 3; Cm++) {
    const Moved = MovedToward(Object_, Cm);
    const Pitch = Context.Scan(modelPointFromDisplayPoint({ ...Moved.Centre, z: (Moved.BaseZ + Moved.TopZ) / 2 }));
    if (Pitch != null && PlanGrasp(Moved, Pitch, Context, false).Plan) return Cm;
  }
  return null;
}
// The sweep holds it only at pitches the arm does not reach beside it: in the
// basket's way (Collision: the IK's reason), or out of reach from here (with
// how many cm closer the app planned it, when it did).
function ReachRefusal(Object_: Prism, Held: Screened[], CloserCm: number | null, Collision: string | null): GraspRefusal {
  const Pitches = Held.map(Item => Item.Pitch), Steep = Math.min(...Pitches), Flat = Math.max(...Pitches);
  const Range = `${Steep === Flat ? `wrist ${Deg(Steep)}` : `wrist ${Deg(Steep)}..${Deg(Flat)}`} deg${Steep >= -20 ? ", nearly level" : ""}`;
  const R = Math.hypot(modelPointFromDisplayPoint({ ...Object_.Centre, z: 0 }).x, modelPointFromDisplayPoint({ ...Object_.Centre, z: 0 }).y);
  const Label = Object_.Label;
  // Not out of reach: in the basket's (or the body's) way at every pitch the jaws hold it with and every height tried.
  if (Collision) {
    return { Code: "path", CloserCm: null,
      Text: `the jaws hold it with the ${Range}, and at each of those, at the grasp point beside it up to ${GraspRules.MaxRaiseCm.toFixed(0)} cm above its mid-height, the ${Collision}`,
      Advice: `Move the ${Label} farther from the basket, then send the task again.` };
  }
  const Closer = CloserCm != null ? `; about ${CloserCm} cm closer to the arm (its centre about ${(R - CloserCm).toFixed(0)} cm from the pan axis) the app plans it` : "";
  return { Code: "reach", CloserCm,
    Text: `the jaws hold it only with the ${Range}, and there the arm does not reach the grasp point beside it from here (its centre ${R.toFixed(1)} cm from the pan axis)${Closer}`,
    Advice: CloserCm != null ? `Move the ${Label} about ${CloserCm} cm closer to the right arm (straight toward its shoulder), then send the task again.`
      : `Move the ${Label} closer to the right arm, then send the task again.` };
}

// What the model and the user are told about a refusal, with the gap the best
// pitch placed the pad at (GapCm). SweepWhy: why the jaw-plane sweep held it
// at no pitch either, at its best one (M3 tried it).
function RefusalFor(Code: GraspRefusalCode, Object_: Prism, Best: PitchResult | null, SweepWhy: string | null = null): GraspRefusal {
  const Refusal = RefusalWords(Code, Object_, Best, SweepWhy), Gap = Best?.Info?.ClearanceCm;
  return Gap != null ? { ...Refusal, GapCm: Gap } : Refusal;
}
function RefusalWords(Code: GraspRefusalCode, Object_: Prism, Best: PitchResult | null, SweepWhy: string | null): GraspRefusal {
  const Info = Best?.Info;
  const Size = Object_.SizeCm;
  const Label = Object_.Label;
  // The gap the pad was placed at (the user's grasp gap), as the pitch that got furthest had it.
  const Gap = Info?.ClearanceCm ?? GraspRules.FixedJawClearanceCm, Clearance = Gap.toFixed(1);
  if (Code === "thin") {
    return { Code, Text: `it is narrower than the ${GraspRules.ThinCm.toFixed(1)} cm the jaws leave when fully closed`,
      Advice: `Try an object at least 1 cm across: the closed jaws would not hold the ${Label}.` };
  }
  if (Code === "wide") {
    // What the jaw-plane sweep found at its best pitch: the closing squeezes it out, or the jaw cannot clear it.
    const Lean = SweepWhy ? `; with the wrist nearer level the jaws do not hold it either (at best, ${SweepWhy})` : "";
    if (Best && !Best.Ok && Best.Code === "top-edge") {
      return { Code, Text: `at every wrist pitch the arm reaches beside it, the closing moving jaw would land on its top edge instead of its side${Lean}`,
        Advice: `Try a shorter or narrower object, or turn the ${Label} so its narrow side faces the jaws.` };
    }
    const Narrow = Size ? Math.min(...Size) : null;
    const Turn = Narrow != null && Narrow + 2 * GraspRules.TrimPadCm + Gap <= MaxWindowCm;
    const Measured = Size ? `it measured ${Size[0].toFixed(1)} x ${Size[1].toFixed(1)} cm${Object_.Round ? ` (round: up to ${Object_.Round.HiCm.toFixed(1)} cm across)` : ""}; ` : "";
    // The jaws cannot reach around it (the straddle, PlaceAtPitch): a grasp short of that would push it out past the tips.
    if (Best && !Best.Ok && Best.Code === "straddle") {
      return { Code, Text: `${Measured}with the fixed jaw ${Clearance} cm beside it the jaws cannot reach around it: ${Best.Why}; a grasp with the jaws short of its middle`
        + ` squeezes it out past the finger tips`,
        Advice: `Try a narrower object: the ${Label} is too wide for this gripper's jaws to reach around before they close.` };
    }
    return { Code,
      Text: `${Measured}with the fixed jaw ${Clearance} cm beside it the moving jaw would have to close ${(Info?.WEff ?? 0).toFixed(1)} cm, and this gripper's single moving jaw only meets the side of objects up to ${MaxWindowCm.toFixed(1)} cm from the fixed jaw (wider ones it lands on their top edge)`
        + Lean + (Turn ? `; turned so its ${Narrow!.toFixed(1)} cm side spans the jaws (the arm's left-right there), it would fit` : ""),
      Advice: Turn ? `Turn the ${Label} so its ${Narrow!.toFixed(1)} cm side faces the jaws (the arm's left-right there), then send the task again.`
        : `Try a narrower object: this gripper's jaws would squeeze one this wide out.` };
  }
  if (Code === "flat") {
    return { Code, Text: `it stands only ${(Object_.TopZ - Object_.BaseZ).toFixed(1)} cm above the desk; closing, the jaws reach about 1 cm below the grasp point, so they would close above it or press into the desk`,
      Advice: `Try a taller object: it must stand at least about 1.5 cm above the desk.` };
  }
  if (Code === "neighbour" && Info?.Other) {
    const Other = Info.Other;
    const Distance = (Info.OtherClosing ? GraspRules.ContactMarginCm : GraspRules.PlanMarginCm) - Other.DepthCm;
    const Near = `${Distance < 0 ? `go ${(-Distance).toFixed(1)} cm into` : `come within ${Distance.toFixed(1)} cm of`} the ${Other.Label} measured at X ${Other.Prism.Centre.x.toFixed(1)}, Y ${Other.Prism.Centre.y.toFixed(1)} beside it`;
    return { Code, Text: Info.OtherClosing ? `closing at the grasp point, the gripper's moving jaw would swing out and ${Near}` : `the gripper's ${PartName[Other.Part]} would ${Near}`,
      Advice: `Move the ${Other.Label} away from the ${Label} (the fixed jaw goes down beside it), then send the task again.` };
  }
  if (Code === "unmeasured") {
    return { Code, Text: "its size was not measured by the depth camera, so the app cannot place the fixed jaw beside it",
      Advice: `Make sure the ${Label} is in clear view of the depth camera, then send the task again.` };
  }
  const Why = Best && !Best.Ok ? Best.Why : "the arm does not reach beside it";
  return { Code: Code === "neighbour" ? "path" : Code,
    Text: `no wrist pitch the arm reaches there keeps the gripper clear of it on the way down (${Why})`,
    Advice: `Move the ${Label} a few cm closer to the arm or away from other objects, then send the task again.` };
}
export const UnmeasuredRefusal = (Label: string) => RefusalFor("unmeasured", { Label } as Prism, null);

// --- Closed-loop fixed-finger correction (M3) ------------------------------------------

export const TapeRules = {
  // The most the grasp point moves for the tape: away from the object (safe for the
  // gripper, the camera's own tape tolerance), and toward it (well under c - the guard's
  // margin, so one noisy tape reading can never put the fixed pad within the guard's
  // margin of where the camera measured the object). The least worth moving it.
  MaxAwayCm: 1.0,
  MaxTowardCm: 0.5,
  MinShiftCm: 0.2,
  // The tip counts as at the hover point (where the correction is measured) within these.
  AtHoverCm: 1.5,
  AtHoverZCm: 2,
};
// ResidualCm: where the camera saw the tip tape against where the arm model puts
// it (display frame); AlongCm: its part along u (toward the object); ShiftCm: how
// far the plan moved along u (negative: away from the object); Clamped: at the most it moves.
export type TapeFix = { ResidualCm: Point3; AlongCm: number; ShiftCm: number; Clamped: boolean };
/** Whether the tip is at the plan's hover point, where its tape check corrects the plan. */
export function AtHoverPoint(Plan: GraspPlan, Tip: Point3) {
  return Math.hypot(Tip.x - Plan.Hover.x, Tip.y - Plan.Hover.y) <= TapeRules.AtHoverCm && Math.abs(Tip.z - Plan.Hover.z) <= TapeRules.AtHoverZCm;
}
/**
 * The GRASP PLAN moved for the tip tape (closed-loop fixed-finger correction).
 * At the hover check-in the camera's tape check compares the red tape on the
 * fixed jaw with where the arm model puts it. Whether the arm or the camera
 * is off, the fixed pad would stand that much off beside the object the
 * camera measured, so the grasp and hover points move by minus the residual
 * along the jaw axis u: away from the object at most MaxAwayCm, toward it at
 * most MaxTowardCm (c is 1.0 and the guard keeps 0.4); under MinShiftCm they
 * stay. The object's prism does NOT move: the plan and the guard keep the
 * gripper clear of where the camera measured it, so a wrong reading can
 * only bring the fixed pad to 0.5 cm of it. With a Context the moved grasp
 * point must pass the app's exact reach check, and the moved descent (hover
 * to grasp, the jaw over the plan's band) and grasp pose must keep every
 * gripper part ContactMarginCm clear of the object and of Context.Others;
 * else no plan, and why. Context null: the geometry only (tests of the
 * arithmetic).
 */
export function TapeCorrection(Plan: GraspPlan, ResidualCm: Point3, Context: GraspContext | null):
  { Plan: GraspPlan; Fix: TapeFix; Why: null } | { Plan: null; Fix: TapeFix; Why: string } {
  const U = Plan.JawAxis;
  const AlongCm = ResidualCm.x * U.x + ResidualCm.y * U.y;
  const Wanted = -AlongCm, ShiftCm = Math.max(-TapeRules.MaxAwayCm, Math.min(TapeRules.MaxTowardCm, Wanted));
  const Fix: TapeFix = { ResidualCm, AlongCm, ShiftCm: Math.abs(ShiftCm) < TapeRules.MinShiftCm ? 0 : ShiftCm, Clamped: Math.abs(ShiftCm - Wanted) > 1e-9 };
  if (!Fix.ShiftCm) return { Plan: { ...Plan, Tape: Fix }, Fix, Why: null };
  const Dx = Fix.ShiftCm * U.x, Dy = Fix.ShiftCm * U.y;
  const Grasp = { x: Tenth(Plan.Grasp.x + Dx), y: Tenth(Plan.Grasp.y + Dy), z: Plan.Grasp.z };
  const Hover = { ...Plan.Hover, x: Grasp.x, y: Grasp.y };
  const Moved: GraspPlan = { ...Plan, Grasp, Hover, OffsetCm: Math.hypot(Grasp.x - Plan.Prism.Centre.x, Grasp.y - Plan.Prism.Centre.y), Tape: Fix, Poses: null };
  if (Context) {
    const Open = OpenDeg(Context.Servos, Plan.OpenFraction), Jaws = PlanJawDegs(Context.Servos, Plan.OpenFraction);
    const Angles = ExactlyReachable(Grasp, Plan.Pitch, Context, Hover.z);
    const HoverAngles = ExactlyReachable(Hover, Plan.HoverPitch, Context, Hover.z);
    if (!Angles || !HoverAngles) return { Plan: null, Fix, Why: `the moved ${Angles ? "hover" : "grasp"} point X ${Grasp.x.toFixed(1)}, Y ${Grasp.y.toFixed(1)} fails the app's exact reach check` };
    const Real = PlanArmPrism(Plan);
    const All = Real ? [Plan.Prism, Real, ...Context.Others] : [Plan.Prism, ...Context.Others];
    // (A deep plan, GraspRules.DeepGrasp: the palm and servo GuardPalmGapCm from the object, by their exact gap.)
    const Tasks = Plan.Deep ? (Real ? [Plan.Prism, Real] : [Plan.Prism]) : null, Own = Tasks ? DeepTaskMargins : null;
    const Hit = DeepestContact(GripperSamples({ ...Angles, gripper: Open }, Jaws), All, GraspRules.ContactMarginCm, Own, Tasks)
      ?? PathContact({ ...HoverAngles, gripper: Open }, { ...Angles, gripper: Open }, Open, All, GraspRules.ContactMarginCm, GraspRules.PathSamples, Jaws, Own, Tasks)
      ?? (Tasks ? PalmPathHit({ ...HoverAngles, gripper: Open }, { ...Angles, gripper: Open }, Tasks, GraspRules.GuardPalmGapCm, GraspRules.ContactMarginCm) : null);
    if (Hit) return { Plan: null, Fix, Why: `at the moved grasp point the ${PartName[Hit.Part]} would come within ${Math.max(GraspRules.ContactMarginCm - Hit.DepthCm, 0).toFixed(1)} cm of the ${Hit.Label}` };
    Moved.Poses = { Hover: { ...HoverAngles, gripper: Open }, Grasp: { ...Angles, gripper: Open }, JawDegs: Jaws };
  }
  return { Plan: Moved, Fix, Why: null };
}
/** What the model is told about a tape correction. */
export function TapeText(Fix: TapeFix, Label: string): string {
  const Seen = `the camera saw the tip tape ${Math.abs(Fix.AlongCm).toFixed(1)} cm ${Fix.AlongCm > 0 ? "nearer the" : "farther from the"} ${Label} than the arm model puts it`;
  if (!Fix.ShiftCm) return `${Seen}, within ${TapeRules.MinShiftCm.toFixed(1)} cm: the GRASP PLAN stays`;
  return `${Seen}, so the GRASP POINT moved ${Math.abs(Fix.ShiftCm).toFixed(1)} cm ${Fix.ShiftCm < 0 ? "away from" : "toward"} it${Fix.Clamped ? ` (the most the app moves it that way; the rest is ${(Math.abs(Fix.AlongCm) - Math.abs(Fix.ShiftCm)).toFixed(1)} cm)` : ""}`;
}

// --- Which measured objects the gripper keeps clear of ------------------------------

// Parts of the robot are never objects to keep clear of (they are the robot).
const RobotWords = /\b(robot|robotic|arm|gripper|claw|jaw)s?\b/i;
const Centre = (Object_: LocatedObject) => ({ x: Object_.x, y: Object_.y });
/**
 * Whether a measured object is the task object measured again under another
 * label: within TwinCm of the task measurement or of the plan's source, or
 * inside the task prism, at about the same base. Twins are never neighbours.
 */
export function IsTwin(Object_: LocatedObject, Task: LocatedObject | null, Source: LocatedObject | null, TaskPrism: Prism | null) {
  const Base = (Other: LocatedObject) => Math.abs(Object_.z - Other.z) <= GraspRules.TwinBaseCm;
  const Near = (Other: LocatedObject | null) => !!Other && Math.hypot(Object_.x - Other.x, Object_.y - Other.y) <= GraspRules.TwinCm && Base(Other);
  return Near(Task) || Near(Source)
    || (!!TaskPrism && FootprintDistance(TaskPrism.Footprint, Centre(Object_)) < 0 && Math.abs(Object_.z - TaskPrism.BaseZ) <= GraspRules.TwinBaseCm);
}
/** The measured objects that are neither the task object, its twins, the robot, nor in the bin (unless the task object is). */
export function NeighbourObjects(Measured: LocatedObject[], Task: LocatedObject | null, Source: LocatedObject | null = null, TaskPrism: Prism | null = null) {
  const InBin = (Task ?? Source)?.supportBranch === "bin";
  return Measured.filter(Object_ => Object_ !== Task && Object_ !== Source && !RobotWords.test(Object_.label)
    && (InBin || Object_.supportBranch !== "bin") && !IsTwin(Object_, Task, Source, TaskPrism ?? (Task ? ObjectPrism(Task) : null)));
}
/**
 * The guard's prisms for this frame: every measured object's (as planned: a
 * round one at its widest, GraspPrism), minus the bin's (unless the task
 * object is in it) and the robot's; with a plan, the task object's own
 * measurement and its twins give way to the plan's prism, else the task
 * measurement is the task prism and its twins are marked as such. A round
 * object's sweep plan's object where the real gripper meets it (PlanArmPrism)
 * is held too, as a twin of the task prism (the real fixed finger stands nearer it).
 */
export function GuardPrisms(Measured: LocatedObject[], Task: LocatedObject | null, Plan: GraspPlan | null, Source: LocatedObject | null) {
  const Own = Plan?.Prism ?? (Task ? GraspPrism(Task) : null);
  const TaskPrism: Prism | null = Own ? { ...Own, Role: "task" } : null;
  const Real = Plan ? PlanArmPrism(Plan) : null;
  const InBin = (Task ?? Source)?.supportBranch === "bin";
  const Prisms: Prism[] = [];
  for (const Object_ of Measured) {
    if (Object_ === Task || RobotWords.test(Object_.label) || (!InBin && Object_.supportBranch === "bin")) continue;
    const Twin = (Task || Source || TaskPrism) ? IsTwin(Object_, Task, Source, TaskPrism) : false;
    if (Twin && Plan) continue;
    // A round neighbour at its widest (a can beside it reads 1.6 cm narrower than it is).
    const P = Twin ? ObjectPrism(Object_) : GraspPrism(Object_);
    if (P) Prisms.push(Twin ? { ...P, Role: "twin" } : P);
  }
  if (Real) Prisms.unshift({ ...Real, Label: `${Real.Label} (where the real fixed finger meets it)`, Role: "twin" });
  return { Prisms: TaskPrism ? [TaskPrism, ...Prisms] : Prisms, TaskPrism };
}

// --- The runtime guard ---------------------------------------------------------------

export type GraspIntent = "grasp" | "contact" | "approach";
/**
 * The TASK OBJECT line's NEXT STEP for the arm as it was at the check-in
 * (AiMeasurements PlannedStepOf): its sentence, its one motion_proposal JSON
 * verbatim (null: it gives none, or more than one), and where that JSON's tip
 * move ends (display frame) with its wrist (null: no tip move).
 */
export type PlannedStep = { Text: string; Json: string | null; Target: Point3 | null; Pitch: number | null };
// Plain data, so it can go to the worker with a snap request. Step: a planned grasp's NEXT STEP (NextStepReason).
// Floor: the task's (AiTableFloor.ts, the surface its object stands on), which the snap and the page hold every target to.
export type GraspGuard = { Prisms: Prism[]; TaskPrism: Prism | null; Plan: GraspPlan | null; Refusal: GraspRefusal | null;
  Held: boolean; Intent: GraspIntent; TaskLabel: string; Step?: PlannedStep | null; Floor?: AiFloor | null };

const LongPartName: Record<GripperPart, string> = { FixedFinger: "FIXED finger (the red-taped jaw; the tip is on its inside)",
  PalmAndServo: "palm", MovingJaw: "open moving jaw" };
/** The GRASP PLAN's two points, as the refusals quote them. */
export function PlanPointsText(Plan: GraspPlan) {
  const Point = (P: Point3) => `X ${P.x.toFixed(1)}, Y ${P.y.toFixed(1)}, Z ${P.z.toFixed(1)}`;
  if (Plan.Approach) return `side pregrasp ${Point(Plan.Approach.Pregrasp)}, then grasp ${Point(Plan.Grasp)}, close and check, release ${Point(Plan.Approach.Release)}, retreat ${Point(Plan.Approach.Retreat)} (wrist ${Deg(Plan.Pitch)} throughout)`;
  return `hover point ${Point(Plan.Hover)} (wrist ${Deg(Plan.HoverPitch)}), then grasp point ${Point(Plan.Grasp)} (wrist ${Deg(Plan.Pitch)})`;
}
// With a NEXT STEP, the hint is that step, not the plan's points: at 18:54 (2026-10-01) the plan's
// hover point was the one the arm had stopped off, and the step re-aimed it.
function PlanHint(Guard: GraspGuard) {
  if (Guard.Plan && Guard.Step?.Json) return ` The tip is the FIXED jaw, so a grasp goes BESIDE the ${Guard.TaskLabel}: follow the NEXT STEP: "motion_proposal":${Guard.Step.Json}.`;
  if (Guard.Plan) return ` The tip is the FIXED jaw, so a grasp goes BESIDE the ${Guard.TaskLabel}: use the GRASP PLAN: ${PlanPointsText(Guard.Plan)}.`;
  if (Guard.Refusal) return ` The app found the ${Guard.TaskLabel} NOT GRASPABLE (${Guard.Refusal.Text}): do not move toward it; reply ask.`;
  return "";
}

// The prisms a motion from CurrentAngles must keep clear of: the task object
// (and a same-spot twin) is meant to be touched while held or for a contact
// task, and rising out of an object the gripper already touches is allowed.
// OnDescent: the move runs along a best-effort plan's own descent (OnPlannedDescent).
function ActivePrisms(Guard: GraspGuard, CurrentAngles: CadAngles, Rising: boolean, OnDescent = false) {
  // A best-effort plan (GraspRules.BestEffort) may bring the gripper against the object it grasps on its own descent:
  // its warnings say so, and the user approves each move. Anywhere else the object stays checked like any plan's
  // (2026-10-07 audit M9: a plan warned only "the grip may slip" let a sideways move at Z 4 sweep the fixed finger
  // 3.2 cm through the tin); the guard holds the neighbours either way.
  const Exempt = Guard.Held || Guard.Intent === "contact" || (OnDescent && !!Guard.Plan?.Warnings?.length);
  const Active = Guard.Prisms.filter(P => !(P.Role && Exempt));
  if (!Rising || !Active.length) return Active;
  const Start = GripperSamples(CurrentAngles, JawPair(CurrentAngles.gripper ?? 0));
  return Active.filter(P => P.Scene || !DeepestContact(Start, [P], GraspRules.ContactMarginCm));
}
// How far a point is from the segment A-B (display frame, cm).
function SegmentDistance(Point: Point3, A: Point3, B: Point3) {
  const AB = Sub3(B, A), Length2 = Dot3(AB, AB);
  const T = Length2 > 1e-12 ? Math.max(0, Math.min(1, Dot3(Sub3(Point, A), AB) / Length2)) : 0;
  const Near = { x: A.x + T * AB.x, y: A.y + T * AB.y, z: A.z + T * AB.z };
  return Math.hypot(Point.x - Near.x, Point.y - Near.y, Point.z - Near.z);
}
/**
 * Whether a tip move runs along a best-effort plan's own descent: from its hover point (a side plan's pregrasp)
 * down to its grasp point, the line its printed points were checked along and where its warned contact is the
 * one the user approves. The move starts within GraspRules.ArrivalCm of that line (where the arm stops off a
 * commanded point) and ends on it within NextStepCm (the step's rounding), the wrist within NextStepDeg of the
 * plan's. False without warnings, held or not grasping: those keep their own rules.
 */
export function OnPlannedDescent(Guard: GraspGuard | null, Start: Point3, Target: Point3, Pitch: number): boolean {
  const Plan = Guard?.Plan;
  if (!Guard || !Plan || !Plan.Warnings?.length || Guard.Held || Guard.Intent !== "grasp") return false;
  const Top = Plan.Approach?.Pregrasp ?? Plan.Hover;
  const PlanPitch = Plan.Approach?.Pitch ?? Plan.Pitch;
  return SegmentDistance(Start, Top, Plan.Grasp) <= GraspRules.ArrivalCm
    && SegmentDistance(Target, Top, Plan.Grasp) <= GraspRules.NextStepCm
    && Math.min(Math.abs(Pitch - PlanPitch), Math.abs(Pitch - Plan.HoverPitch)) <= GraspRules.NextStepDeg;
}

/**
 * The guard's pose rules for the snap's searches (AiTargetSnap.ts), which try
 * thousands of poses near a request: the gripper at the pose clear of the
 * prisms (those the start pose touches aside, so it can rise out), and a
 * grasp near its object at the plan's wrist, no deeper than the plan. Poses
 * keep PlanMarginCm, 0.1 cm more than the guard, so a target the model copies
 * rounded still passes it. The snap's post-check runs the full guard, path
 * included, on the pose it picks.
 */
export function GuardPoseCheck(Guard: GraspGuard | null, CurrentAngles: CadAngles): ((Target: TargetPose, Links: Record<string, Matrix4>) => boolean) | null {
  if (!Guard) return null;
  // The task object is left out only for poses on a best-effort plan's own descent (OnPlannedDescent).
  const Sets = [false, true].map(OnDescent => {
    const Active = ActivePrisms(Guard, CurrentAngles, true, OnDescent);
    // A deep plan (GraspRules.DeepGrasp): its own prisms keep GuardPalmGapCm from the palm, 0.05 cm more for the search.
    return { Active, Deep: Guard.Plan?.Deep ? { Tasks: Active.filter(P => P.Role), GapCm: GraspRules.GuardPalmGapCm + 0.05 } : null };
  });
  const Start = Guard.Plan?.Warnings?.length ? StartTip(CurrentAngles) : null;
  const Plan = Guard.Intent === "grasp" && !Guard.Held ? Guard.Plan : null;
  const Real = Plan ? PlanArmPrism(Plan) : null;
  return (Target, Links) => {
    const Display = mirrorDisplayPoint(Target);
    if (Plan && Guard.TaskPrism) {
      if (TaskNear(Guard.TaskPrism, Real, Display) <= GraspRules.GuardNearCm && Display.z < Guard.TaskPrism.TopZ
        && (Math.abs(Target.wristPitch - Plan.Pitch) > GraspRules.WristMatchDeg || Display.z < Plan.Grasp.z - GraspRules.DeeperThanPlanCm)) return false;
    }
    const { Active, Deep } = Sets[Start && OnPlannedDescent(Guard, Start, Display, Target.wristPitch) ? 1 : 0];
    return GripperPoseClear(Links, Active, GraspRules.PlanMarginCm, Deep);
  };
}

/**
 * Why a gripper-only motion must not run in an object task: the moving jaw,
 * swept from its angle now to the target one at the arm's pose (10 samples),
 * would come within ContactMarginCm of a measured object other than the task
 * object (which the jaws close on). Opening beside a neighbour is refused.
 */
export function GripperSweepReason(CurrentAngles: CadAngles, TargetJawDeg: number, Guard: GraspGuard | null): string | null {
  if (!Guard) return null;
  const Swing = JawSwingContact(CurrentAngles, TargetJawDeg, Guard.Prisms.filter(P => !P.Role));
  if (!Swing) return null;
  const Distance = GraspRules.ContactMarginCm - Swing.Hit.DepthCm;
  return `Moving the gripper there would swing the moving jaw ${Distance < 0 ? `${(-Distance).toFixed(1)} cm into` : `within ${Distance.toFixed(1)} cm of`} the ${Swing.Hit.Label} measured at X ${Swing.Hit.Prism.Centre.x.toFixed(1)}, Y ${Swing.Hit.Prism.Centre.y.toFixed(1)} (${Math.round(Swing.S * 100)}% of the way there): move the tip away from it first.`;
}
/**
 * Why a tip move that opens the gripper on the way (an opening gripper_before,
 * which the bridge runs as one motion: lib/AiChain.ts TOGETHER) must not run:
 * on the joint-linear way from CurrentAngles to TargetAngles, the moving jaw at
 * any opening from FromJawDeg to ToJawDeg (PathSamples + 1 openings, each with
 * the real jaw's ~11 deg more closed) would come within ContactMarginCm of a
 * measured object the move must keep clear of. GraspGuardReason checks that way
 * with the jaws open, and GripperSweepReason the swing where the arm starts.
 */
export function OpeningOnTheWayReason(TargetAngles: CadAngles, CurrentAngles: CadAngles, FromJawDeg: number, ToJawDeg: number, RiseCm: number,
  Guard: GraspGuard | null): string | null {
  if (!Guard || Math.abs(ToJawDeg - FromJawDeg) < 0.5) return null;
  const Samples = GraspRules.PathSamples, Margin = GraspRules.ContactMarginCm;
  const JawDegs = Array.from({ length: Samples + 1 }, (_, Index) => FromJawDeg + (Index / Samples) * (ToJawDeg - FromJawDeg)).flatMap(JawPair);
  const OnDescent = OnPlannedDescent(Guard, StartTip(CurrentAngles), StartTip(TargetAngles), CadPitch(TargetAngles));
  const Active = ActivePrisms(Guard, CurrentAngles, RiseCm > 0.5, OnDescent), Tasks = Guard.Plan?.Deep ? Active.filter(P => P.Role) : null;
  const Hit = PathContact(CurrentAngles, TargetAngles, ToJawDeg, Active, Margin, Samples, JawDegs, Tasks ? DeepTaskMargins : null, Tasks);
  if (!Hit) return null;
  const Distance = Margin - Hit.DepthCm;
  return `Opening the gripper on the way there would swing its moving jaw ${Distance < 0 ? `${(-Distance).toFixed(1)} cm into` : `within ${Distance.toFixed(1)} cm of`} the ${Hit.Label} measured at X ${Hit.Prism.Centre.x.toFixed(1)}, Y ${Hit.Prism.Centre.y.toFixed(1)} (from ${Math.round(Hit.FirstS * 100)}% of the way there): open the gripper alone first, then move.`;
}
/**
 * The first contact (ContactMarginCm) of the moving jaw, swung at the arm's
 * pose from its angle now to a target one (10 samples, each with the real
 * jaw's ~11 deg more closed), with a prism it does not already touch; S: how
 * far through the swing. Null: clear.
 */
export function JawSwingContact(CurrentAngles: CadAngles, TargetJawDeg: number, Prisms: Prism[]): { Hit: Contact; S: number } | null {
  const From = CurrentAngles.gripper ?? 0;
  if (Math.abs(TargetJawDeg - From) < 0.5 || !Prisms.length) return null;
  const Margin = GraspRules.ContactMarginCm;
  const JawAt = (Deg_: number) => GripperSamples({ ...CurrentAngles, gripper: Deg_ }, JawPair(Deg_)).filter(S => S.Part === "MovingJaw");
  // A neighbour the jaw already touches is not held against it (it could never move).
  const Start = JawAt(From);
  const Others = Prisms.filter(P => !DeepestContact(Start, [P], Margin));
  if (!Others.length) return null;
  for (let Index = 1; Index <= GraspRules.PathSamples; Index++) {
    const S = Index / GraspRules.PathSamples;
    const Hit = DeepestContact(JawAt(From + S * (TargetJawDeg - From)), Others, Margin);
    if (Hit) return { Hit, S };
  }
  return null;
}

/**
 * How near a point is to the task object, where the camera measured it or
 * where the real gripper meets it (a round object's sweep plan, PlanArmPrism):
 * its grasp point stands GraspRules.FixedFingerOffsetCm farther from the first.
 */
function TaskNear(Task: Prism, Real: Prism | null, Point: XY) {
  return Math.min(FootprintDistance(Task.Footprint, Point), Real ? FootprintDistance(Real.Footprint, Point) : Infinity);
}

// The tip at a pose (display frame), where a move from it starts.
function StartTip(Angles: CadAngles): Point3 {
  const Joints = CadForward(Angles).Joints;
  return mirrorDisplayPoint(Joints[Joints.length - 1]);
}
/** How every NEXT STEP refusal starts: the app knows the model was refused for not following the step. */
export const NextStepRefusalLead = "That move is not the NEXT STEP";
/**
 * Why a tip move must not run in a planned grasp (the NEXT STEP rule): it
 * ends near the task object (within GuardNearCm of where the camera measured
 * it or where the real gripper meets it, below its top + NextStepAboveTopCm)
 * and it is neither the NEXT STEP the TASK OBJECT line gave (its target within
 * NextStepCm, its wrist within NextStepDeg) nor a rise of NextStepRiseCm or
 * more that comes no nearer the object. Target and Start (where the move
 * starts) in the display frame; RiseCm is the move's up change. The jaws
 * holding it (a put-down, a release), a contact task, or no step: null; and
 * moves that end anywhere else are the model's to choose (the guard's other
 * rules still hold them). 2026-10-01 18:54: the step re-aimed the hover point
 * the arm had stopped 0.5 cm toward the can from; the model descended from
 * where it stopped instead, and only "deeper than the plan" refused it, with
 * a hint quoting the plan's points: its revision went down to the grasp
 * height from there, 0.5 cm toward the can, and was approved.
 * Also: what else is wrong with the move (the guard's contact), said after why.
 */
export function NextStepReason(Guard: GraspGuard | null, Target: Point3, Pitch: number, Start: Point3, RiseCm: number, Also: string | null = null): string | null {
  const Step = Guard?.Step ?? null, Task = Guard?.TaskPrism ?? null;
  if (!Guard || !Step || !Task || Guard.Intent !== "grasp" || Guard.Held) return null;
  const Real = Guard.Plan ? PlanArmPrism(Guard.Plan) : null;
  const Near = (Point: XY) => TaskNear(Task, Real, Point);
  if (Near(Target) > GraspRules.GuardNearCm || Target.z >= Task.TopZ + GraspRules.NextStepAboveTopCm) return null;
  const To = Step.Target;
  if (To && Math.hypot(Target.x - To.x, Target.y - To.y, Target.z - To.z) <= GraspRules.NextStepCm
    && Math.abs(Pitch - (Step.Pitch ?? Pitch)) <= GraspRules.NextStepDeg) return null;
  // Up and no nearer it: rising out of the way is never held to the step.
  if (RiseCm >= GraspRules.NextStepRiseCm && Near(Target) >= Near(Start) - 0.05) return null;
  const Label = Guard.TaskLabel;
  return `${NextStepRefusalLead}, and it ends low beside the ${Label} (within ${GraspRules.GuardNearCm} cm of it, below its top + ${GraspRules.NextStepAboveTopCm} cm),`
    + ` where the app accepts only the NEXT STEP or a move up and away from the ${Label}.${Also ? ` ${Also}` : ""}`
    + (Step.Json ? ` Copy the NEXT STEP's motion_proposal exactly: "motion_proposal":${Step.Json}. The step, with why: ${Step.Text}`
      : ` The NEXT STEP: ${Step.Text}`)
    + ` If you think the NEXT STEP is wrong, reply task_status "ask" with motion_proposal null and say why.`;
}
/**
 * Why an AI tip motion (model frame, its IK angles, from CurrentAngles) must
 * not run: a gripper part would come within ContactMarginCm of a measured
 * object on the joint-linear way there; or, grasping with a plan, it goes
 * below the task object's top near it at another wrist or deeper than the
 * plan; or, grasping without one, it descends beside the object; or, with a
 * planned grasp's NEXT STEP, it is not that step near the object (NextStepReason,
 * first, with the contact if there is one). Null: allowed.
 */
export function GraspGuardReason(TargetModel: TargetPose, TargetAngles: CadAngles, CurrentAngles: CadAngles, RiseCm: number,
  Guard: GraspGuard | null): string | null {
  if (!Guard) return null;
  if (Guard.Held && Guard.Floor?.Scene && Guard.Plan?.Poses?.Grasp) {
    const Carried = CarriedObjectPathReason(Guard.Plan.Prism, Guard.Plan.Poses.Grasp, CurrentAngles, TargetAngles, Guard.Prisms.filter(P => !P.Role), Guard.Floor);
    if (Carried) return `That route is blocked: ${Carried}.`;
  }
  const Unknown = ScenePathCoverageReason(CurrentAngles, TargetAngles, Guard.Floor, JawPair(CurrentAngles.gripper ?? 0));
  if (Unknown) return `That route is blocked: ${Unknown}.`;
  const Margin = GraspRules.ContactMarginCm;
  const Jaw = CurrentAngles.gripper ?? 0;
  // The task object is left out only on a best-effort plan's own descent (OnPlannedDescent): elsewhere it is checked.
  const OnDescent = OnPlannedDescent(Guard, StartTip(CurrentAngles), mirrorDisplayPoint(TargetModel), TargetModel.wristPitch);
  // A deep plan (GraspRules.DeepGrasp): the palm and servo GuardPalmGapCm from the object it grasps, by their exact gap.
  const Active = ActivePrisms(Guard, CurrentAngles, RiseCm > 0.5, OnDescent), Tasks = Guard.Plan?.Deep ? Active.filter(P => P.Role) : null;
  const Hit = PathContact(CurrentAngles, TargetAngles, Jaw, Active, Margin, undefined, undefined, Tasks ? DeepTaskMargins : null, Tasks)
    ?? (Tasks ? PalmPathHit(CurrentAngles, TargetAngles, Tasks, GraspRules.GuardPalmGapCm, Margin) : null);
  const Distance = Hit ? Margin - Hit.DepthCm : 0;
  const Contact = Hit ? `At that target the gripper's ${LongPartName[Hit.Part]} would ${Distance < 0 ? `go ${(-Distance).toFixed(1)} cm into` : `come within ${Distance.toFixed(1)} cm of`} the ${Hit.Label} measured at X ${Hit.Prism.Centre.x.toFixed(1)}, Y ${Hit.Prism.Centre.y.toFixed(1)} (from ${Math.round(Hit.FirstS * 100)}% of the way there); every gripper part must stay ${Margin.toFixed(1)} cm clear of measured objects.` : null;
  // A planned grasp's moves near the object are its NEXT STEP's: what to do instead leads, the contact follows.
  const OffStep = Guard.Step ? NextStepReason(Guard, mirrorDisplayPoint(TargetModel), TargetModel.wristPitch, StartTip(CurrentAngles), RiseCm, Contact) : null;
  if (OffStep) return OffStep;
  if (Hit && Contact) return Contact + (Hit.Prism.Role ? PlanHint(Guard) : "");
  const Task = Guard.TaskPrism;
  if (Guard.Intent !== "grasp" || Guard.Held || !Task) return null;
  const Display = mirrorDisplayPoint(TargetModel);
  const Near = FootprintDistance(Task.Footprint, Display);
  if (Guard.Plan) {
    if (TaskNear(Task, PlanArmPrism(Guard.Plan), Display) > GraspRules.GuardNearCm || Display.z >= Task.TopZ) return null;
    const Plan = Guard.Plan;
    if (Math.abs(TargetModel.wristPitch - Plan.Pitch) > GraspRules.WristMatchDeg) {
      return `That target is below the ${Guard.TaskLabel}'s top beside it with the wrist at ${Deg(TargetModel.wristPitch)} deg; the GRASP PLAN was checked only at wrist ${Deg(Plan.Pitch)} deg there.${PlanHint(Guard)}`;
    }
    if (Display.z < Plan.Grasp.z - GraspRules.DeeperThanPlanCm) {
      return `That target goes ${(Plan.Grasp.z - Display.z).toFixed(1)} cm deeper than the GRASP PLAN's grasp point Z ${Plan.Grasp.z.toFixed(1)} beside the ${Guard.TaskLabel}.${PlanHint(Guard)}`;
    }
    return null;
  }
  if (RiseCm < -0.5 && Near <= 2 && Display.z < Task.TopZ + GraspRules.HoverAboveTopCm) {
    return `That target descends beside the ${Guard.TaskLabel}, which has no GRASP PLAN the app checked against the gripper's shape: do not descend beside or onto it.${PlanHint(Guard) || " Reply ask."}`;
  }
  return null;
}

// --- The plan lock ---------------------------------------------------------------------

// What a plan depends on besides the object: the CAD alignment, the wrist roll, the elbow branch and the joint limits.
export type PlanKey = { Version: number; RollRaw: number; Elbow: "up" | "down"; Limits: string };
// An occluded re-measure in the plan's way: its prism (the guard holds it too), how far its
// face toward the fixed finger came (cm, + toward it), and what the model is told.
export type PlanCorridor = { Prism: Prism; ShiftCm: number; Why: string };
export type GraspPlanState = { Reach: TaskReachability; Source: LocatedObject; At: number; Key: PlanKey; Note: string | null;
  // A second measurement agreed with the plan's (its centre unoccluded, or its near face): the descent is offered plainly.
  Confirmed?: boolean;
  // An occluded re-measure the plan's descent or grasp pose would come within ContactMarginCm of.
  Corridor?: PlanCorridor | null;
  // The jaws closed on nothing at this plan's grasp point (AiMeasurements ClosedOnNothing) and the
  // object was not measured cleanly since: the plan is stale, its steps do not descend with it.
  Missed?: boolean;
  // Made again from a re-measure after the jaws closed on nothing (KeepOrReplan AfterMiss): the
  // NEXT STEP says so while the jaws are still closed.
  AfterMiss?: boolean };
// AfterMiss: not kept, and planned again because the jaws closed on nothing with the old plan.
export type PlanVerdict = { Keep: boolean; Note: string | null; Confirmed?: boolean; Corridor?: PlanCorridor | null; Missed?: boolean; AfterMiss?: boolean };

// The object's face nearest the fixed finger along u, within the finger's band across it (the whole footprint if it misses the band).
function NearFace(Object_: Prism, Uh: XY, Across: number) {
  const Vh = { x: -Uh.y, y: Uh.x };
  const Band = ClipBand(Object_.Footprint, Vh, Across - GraspRules.FingerBandHalfCm, Across + GraspRules.FingerBandHalfCm);
  return Math.min(...(Band.length ? Band : Object_.Footprint).map(P => Dot2(P, Uh)));
}
export function GraspPlanKey(Servos: Record<ServoName, ServoState>, Elbow: "up" | "down", Limits?: IkAngleLimits | null): PlanKey {
  return { Version: GetCadAlignment().Version, RollRaw: Servos.wrist_roll.presentRaw, Elbow, Limits: JSON.stringify(GetIkAngleLimits(Servos, Limits ?? undefined)) };
}
const KeyDiffers = (A: PlanKey, B: PlanKey) => A.Version !== B.Version || A.Elbow !== B.Elbow || A.Limits !== B.Limits || Math.abs(A.RollRaw - B.RollRaw) > 10;

/** Whether the gripper is over or beside a prism (a re-measure there is occluded by it). */
export function GripperNear(Object_: Prism, Tip: Point3) {
  return FootprintDistance(Object_.Footprint, Tip) <= GraspRules.NearCm && Tip.z <= Object_.TopZ + 10;
}

/**
 * The part of an object a partly hidden reading saw: its padded box, or for a
 * round object the disc of its padded larger side (the visible part of a disc
 * has no corners: the 23:50:21 can's 4.3 x 2.3 box put a corner 0.2 cm from
 * the fixed finger where the can itself stood 1.8 cm away).
 */
function SeenPrism(Object_: LocatedObject): Prism | null {
  const Box = ObjectPrism(Object_);
  if (!Box || !RoundObject(Object_)) return Box;
  return { ...Box, Footprint: Disc(Box.Centre, Math.max(...Object_.sizeCm!) + 2 * GraspRules.TrimPadCm) };
}
/**
 * Where the GRASP PLAN's own motion (the descent from its hover point and the
 * grasp pose, the jaw over the plan's band) would come within Margin of a
 * prism: the deepest contact, or null (clear, or a plan without poses).
 */
export function PlanMotionContact(Plan: GraspPlan, Object_: Prism, Margin = GraspRules.ContactMarginCm, PalmNeedCm = GraspRules.GuardPalmGapCm): PathHit | null {
  const Poses = Plan.Poses;
  if (!Poses) return null;
  // A deep plan (GraspRules.DeepGrasp): its palm and servo stand near the object by design, judged by their exact gap.
  const Own = Plan.Deep ? DeepTaskMargins : null;
  const AtGrasp = DeepestContact(GripperSamples(Poses.Grasp, Poses.JawDegs), [Object_], Margin, Own, Object_);
  return AtGrasp ? { ...AtGrasp, S: 1, FirstS: 1 }
    : PathContact(Poses.Hover, Poses.Grasp, Poses.Grasp.gripper ?? 0, [Object_], Margin, GraspRules.PathSamples, Poses.JawDegs, Own, Object_)
      ?? (Plan.Deep ? PalmPathHit(Poses.Hover, Poses.Grasp, [Object_], PalmNeedCm, Margin) : null);
}
// A partly hidden reading stops the plan only where what it saw would be in the gripper's way:
// within this of a part sample (under the samples' 0.375 cm cover, so their hull would be in it).
const SeenOverlapCm = 0.1;

/**
 * Keep the GRASP PLAN or plan again from this check-in's measurement. A
 * re-measure with the gripper over the object, or one that shrank (the jaws
 * hide part of it: 23:50:21 read the can 4.3 x 2.3, not 5.0 x 4.5), keeps the
 * plan; so does a gripper close, and a clean one within ReplanMoveCm. An
 * occluded re-measure of the same label is held against the plan's own
 * motion (PlanMotionContact), on every side (the fixed finger's, the palm's
 * toward the arm, the open jaw's beyond it): a full reading as planned (a
 * round one's widest disc, so a reading of its true width is not a move),
 * any part within ContactMarginCm of it; a shrunk one, what it saw
 * (SeenPrism), only where that would be in the gripper's way (SeenOverlapCm:
 * its centre is not where the object's is). Either is the Corridor: it may
 * have moved into the gripper's way. A kept plan is Confirmed only by a re-measure
 * whose centre and near face agree within ReplanMoveCm (the 23:49:22 ->
 * 23:49:43 readings jumped 4.84 cm between two Home check-ins). A round
 * object's sweep plan's motion is also held against where the real gripper
 * meets the re-measure (ArmPrism), the deeper hit the Corridor.
 * Missed: the jaws closed on nothing at this plan's grasp point now
 * (AiMeasurements ClosedOnNothing), or did at an earlier check-in (Previous.Missed).
 * The close may have pushed the object away (2026-10-01 15:08: the can 3 cm
 * along the fingers, and the kept plan sent the jaws down where it had been):
 * a re-measure that did not shrink plans again from where it stands, the
 * gripper near it or not; a shrunk one keeps the plan marked Missed (its steps
 * do not descend) until one does.
 */
export function KeepOrReplan(Previous: { Source: LocatedObject; At: number; Key: PlanKey; Reach?: { Plan?: GraspPlan | null } | null; Missed?: boolean } | null,
  Measured: LocatedObject, Tip: Point3, Closed: boolean, Key: PlanKey, Missed = false): PlanVerdict {
  if (!Previous) return { Keep: false, Note: null };
  if (KeyDiffers(Previous.Key, Key)) return { Keep: false, Note: "the arm's model, wrist roll or limits changed: new GRASP PLAN" };
  const Miss = Missed || !!Previous.Missed;
  if (Closed && !Miss) return { Keep: true, Note: null, Corridor: null };
  const Source = Previous.Source;
  const Moved = Math.hypot(Measured.x - Source.x, Measured.y - Source.y);
  const Plan = Previous.Reach?.Plan ?? null;
  const Old = Plan?.Prism ?? ObjectPrism(Source);
  const Near = !!Old && GripperNear(Old, Tip);
  const Shrunk = (!!Measured.sizeCm && !!Source.sizeCm && Measured.sizeCm.some((Size, Axis) => Size / Source.sizeCm![Axis] < GraspRules.ShrinkRatio))
    || (Measured.validFraction != null && Source.validFraction != null && Source.validFraction - Measured.validFraction > GraspRules.ValidDrop);
  const Size = (Object_: LocatedObject) => Object_.sizeCm ? `${Object_.sizeCm[0].toFixed(1)} x ${Object_.sizeCm[1].toFixed(1)}` : "unmeasured";
  if (Miss) {
    const Where = `re-measured at X ${Measured.x.toFixed(1)}, Y ${Measured.y.toFixed(1)}, ${Moved.toFixed(1)} cm from the ${Clock(Previous.At)} measurement the GRASP PLAN was made from`;
    if (!Shrunk) return { Keep: false, AfterMiss: true, Note: `the jaws closed on nothing at the GRASP PLAN's grasp point (the close may have pushed it); ${Where}: new GRASP PLAN from where it stands now` };
    return { Keep: true, Missed: true, Confirmed: false, Corridor: null,
      Note: `the jaws closed on nothing at the GRASP PLAN's grasp point; ${Where}, but only partly (size ${Size(Measured)}, was ${Size(Source)}): the app plans the grasp again once the camera measures it clear of the gripper` };
  }
  if (Near || Shrunk) {
    const Note = Moved > 0.3
      ? `re-measured at X ${Measured.x.toFixed(1)}, Y ${Measured.y.toFixed(1)} with the gripper over it (size ${Size(Measured)}, was ${Size(Source)}): the GRASP PLAN keeps the ${Clock(Previous.At)} measurement`
      : null;
    const New = Shrunk ? SeenPrism(Measured) : GraspPrism(Measured);
    // (A best-effort plan may bring the gripper against it: no corridor, GraspRules.BestEffort.)
    if (!Plan || !New || Plan.Warnings?.length || Measured.label.toLowerCase() !== Source.label.toLowerCase()) return { Keep: true, Note, Corridor: null };
    // Positive: the face toward the fixed finger came toward it (within the finger's band across the jaws).
    const Across = Dot2(Plan.Grasp, { x: -Plan.JawAxis.y, y: Plan.JawAxis.x });
    const ShiftCm = NearFace(Plan.Prism, Plan.JawAxis, Across) - NearFace(New, Plan.JawAxis, Across);
    const Margin = Shrunk ? SeenOverlapCm : GraspRules.ContactMarginCm;
    // Where the camera measured it and where the real gripper meets it: the deeper hit (the guard then holds that prism).
    // (A deep plan's palm: within GuardPalmGapCm of a full reading, touching a partial one.)
    const PalmNeed = Shrunk ? 0.001 : GraspRules.GuardPalmGapCm;
    const Hits = [PlanMotionContact(Plan, New, Margin, PalmNeed), Plan.FingerOffsetCm ? PlanMotionContact(Plan, ArmPrism(New, Plan.JawAxis, Plan.FingerOffsetCm), Margin, PalmNeed) : null];
    const Hit = Hits.reduce<PathHit | null>((A, B) => (B && (!A || B.DepthCm > A.DepthCm) ? B : A), null);
    // Without the plan's poses (an older plan): the fixed finger's side only, as before.
    const Toward = !Plan.Poses && ShiftCm > PlanClearanceCm(Plan) - 0.3;
    const At = `re-measured with the gripper over it at X ${Measured.x.toFixed(1)}, Y ${Measured.y.toFixed(1)} (size ${Size(Measured)})`;
    const Corridor: PlanCorridor | null = Hit
      ? { Prism: { ...Hit.Prism, Role: "twin" }, ShiftCm, Why: `${At}, the GRASP PLAN's ${PartName[Hit.Part]} would come within ${Math.max(Margin - Hit.DepthCm, 0).toFixed(1)} cm of it ${Hit.S >= 1 ? "at the grasp point" : `${Math.round(Hit.FirstS * 100)}% of the way down`}` }
      : Toward ? { Prism: { ...New, Role: "twin" }, ShiftCm, Why: `${At}, its near side ${ShiftCm.toFixed(1)} cm closer to the fixed jaw than planned` } : null;
    return { Keep: true, Note, Confirmed: !Corridor && Moved <= GraspRules.ReplanMoveCm && Math.abs(ShiftCm) <= GraspRules.ReplanMoveCm, Corridor };
  }
  const SizeKept = !Measured.sizeCm || !Source.sizeCm || Measured.sizeCm.every((Size, Axis) => Math.abs(Size / Source.sizeCm![Axis] - 1) <= 0.25);
  if (Moved <= GraspRules.ReplanMoveCm && SizeKept) return { Keep: true, Note: null, Confirmed: true, Corridor: null };
  return { Keep: false, Note: `it moved ${Moved.toFixed(1)} cm since ${Clock(Previous.At)}: new GRASP PLAN` };
}

/**
 * The jaws closed on nothing (KeepOrReplan Missed) and the clean re-measure plans no grasp (Reach: out
 * of reach, or not graspable where it stands now): the missed plan stays, marked Missed, so its steps
 * still take the closed jaws up clear of the object before the model is told to ask; the note says why.
 */
export function KeepMissed(Previous: GraspPlanState, Measured: LocatedObject, Reach: TaskReachability): GraspPlanState {
  const Why = Reach.Refusal?.Text ?? (Reach.Kind === "out-of-reach" ? "out of the right arm's reach" : Reach.Kind);
  return { ...Previous, Missed: true, Confirmed: false, Corridor: null,
    Note: `the jaws closed on nothing at the GRASP PLAN's grasp point; re-measured at X ${Measured.x.toFixed(1)}, Y ${Measured.y.toFixed(1)}, where the app plans no grasp (${Why}): the jaws rise clear of it first` };
}
