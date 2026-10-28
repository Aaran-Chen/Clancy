import { useCallback, useEffect, useMemo, useRef, useState, type FormEvent, type MouseEvent as ReactMouseEvent } from "react";
import { BasketTargetReason, BasketPoseReason, BasketCollisionVersion } from "./lib/BasketCollision";
import { AiJawFloorReason, AiTableFloorCm, AiTableFloorReason, DeskFloor, TableZCm, TrayFrontRimYCm, type AiFloor } from "./lib/AiTableFloor";
import { SurfaceProblemsNote, SurfaceText } from "./lib/DeclaredSurfaces";
import { SceneCommandFields, SceneFreshnessIssue, type SceneIdentity } from "./lib/SceneGeometry";
import { AiSnapEnabled, SnapAiTarget, SnapRules, type SnapInput, type SnapResult } from "./lib/AiTargetSnap";
import {
  BestEffortNote, BindFromEstimate, BuildGraspGuard, CameraGateFailure, ClosedJawDescentReason, CameraMovedError, DeskRemeasuredNote, GripperClosedAfter, HeldFraction, IsUnreliable, MeasuredObjectsSummary, NextStepOf,
  NamesQuery, NoTaskObjectLine, ParseTaskObject, ReachFactsText, ReadLocateResponse, ResolveTaskObject, RobotCheckNote, TapeCorrectedReach, TaskFloor, TaskIntent, TaskNeedsPositions, TaskObjectLine, TaskObjectReachability, TaskSupport, UnreliableMeasurements,
  WithheldTaskObject, WithheldTaskStop, KeptDeskFloor, PlanSource, CloseTargetFraction,
  UnreliableTargetReason, ClosedOnNothing, NextStepReminder, NoteGripperMove, PlannedStepOf,
  type CameraGate, type GraspState, type LocateResult, type LocatedObject, type ReachContext, type ReachPlanned, type TaskClose, type TaskObjectBinding,
  type TaskObjectQuery, type TaskObjectState, type TaskReachability,
} from "./lib/AiMeasurements";
import { GraspGuardReason, GraspPlanKey, GripperNear, GripperSweepReason, WristRollTurnReason, KeepOrReplan, NeighbourObjects, NextStepRefusalLead, ObjectPrism,
  OpeningOnTheWayReason, type GraspGuard, type GraspPlan } from "./lib/GraspPlan";
import { GraspRules, KeepMissed } from "./lib/GraspPlan";
import {
  AiModelTimeoutMs, AiStepTimeoutError, ArmMovedDuringCaptureCm, ArmNotStillReason, ArmStillJitterRaw, ArmStillWaitMs, LocateTimeoutMs, MarkFinalTarget, RepeatsLastTarget, WithTimeout,
  BuildAiFeasibilityFeedbackPrompt, BuildTaskCheckInPrompt, ChangingServoTargets, CompactTurn, DescribeBridgeRefusal, DescribeMotionFact, MAX_AI_TASK_STEPS, NoOpFeedbackPrompt, NoOpReason,
  NoMotionFeedbackPrompt, NoMotionTwiceReason, NotSentFeedbackPrompt, PlannedStepNote, PlannedStepReply, SliceWholeTurns, SnapMotionNote, SnapNoOpReason,
  ChainGripperStill, ChainLegInterrupted, TargetKey, TaskSettleVerdictFor, TaskStartLimitMs, TaskStopAdvice, type AiChatMessage, type FinalTargetMark, type TaskEndSource, type TaskMotionSnap, type TaskSettleWait,
} from "./lib/AiTaskLoop";
import type { TargetWorkspaceContext } from "./lib/TargetWorkspace";
import type { PitchGuideSegment } from "./lib/PitchGuide";
import { ContinueAskedTask, type AskedTask } from "./lib/TaskObject";
import { JudgeArmOcclusion } from "./lib/ArmOcclusion";
import { AppDoneReply, AppRunReason, AppRunStepNote, AppRunStepReply, AppStepRules, AppStepsSummary, GraspOnlyRequest, IsLiftedStep, MotionAnomaly, StepNeedsModel,
  StepWords, WithAppStepsSummary, type AppCheckIn, type AppRunStep } from "./lib/AppPlannedSteps";
import { WithoutArmMade, type ArmClearReadings } from "./lib/ArmMadeObjects";
import { ChainedFraction, ChainGripperHereReason, ChainGripperKind, ChainGripperText, ChainOrderText, ChainRefusalReason, ChainRules, ChainRunsTogether, DescribeChainRun, ReadChainGripper,
  type ChainGripper, type ChainLeg, type ChainLegResult, type ChainRun } from "./lib/AiChain";
import {
  Activity,
  Check,
  Home,
  Loader2,
  MessageSquare,
  PlugZap,
  RotateCcw,
  Save,
  Send,
  ShieldX,
  Eraser,
  Target,
  Wifi,
  WifiOff,
  X,
  OctagonAlert,
  KeyRound,
  Hand,
} from "lucide-react";
import { CoastWarning, EmergencyStopLabel, EmptyHandGuide, FirstUseKey, HandGuideButton, HandGuideExitNote, HandGuideStrip, HeartbeatMs,
  NeedsCalibration, ReadHandGuide, type HandGuideButtonView, type HandGuideView } from "./lib/HandGuide";
import { CadModel, GetCadAlignment, SetCadAlignment, IkSolverVersion, type CadAlignment } from "./lib/CadKinematics";
import ArmCanvas, { type ArmCanvasHandle, type ObjectBox } from "./components/ArmCanvas";
import ServoPanel from "./components/ServoPanel";
import BotDiagnostics from "./components/BotDiagnostics";
import TipRadar from "./components/TipRadar";
import MotionThrottle from "./components/MotionThrottle";
import DashboardLayout from "./components/DashboardLayout";
import DashboardSection from "./components/DashboardSection";
import AiStatisticsPanel from "./components/AiStatisticsPanel";
import AiCompanion from "./components/AiCompanion";
import AiReply from "./components/AiReply";
import SettingsPanel from "./components/SettingsPanel";
import AccessCard from "./components/AccessCard";
import { AccessCloseCodes, AccessRefusedEvent, AccessText, AuthMessage, IsAccessCode, IsLoopbackHost, StoredPassword, StorePassword,
  type AccessProblem } from "./lib/RemoteAccess";
import LiveCamera, { DepthCanvasFor, DepthSequenceFor, type CameraSource } from "./components/LiveCamera";
import HoverLoupe from "./components/HoverLoupe";
import SemiCalibrationStage, { SemiCalReopenButton } from "./components/SemiCalibrationStage";
import { CountsOf, SemiCal, SemiCalHotC, SemiCalMinClicks, SemiCalRetakePauseMs, SemiCalWaitLimitMs, type SemiCalDecision, type SemiCalReport } from "./lib/SemiCalibration";
import { CalibrationOutcomeOf, FittedClock, RejectedText, RejectQuestion, type CalibrationFitReport, type CalibrationRejectResult } from "./lib/CalibrationReport";
import { ClickCapturePoint, ImagePointAt, type SourcePoint } from "./lib/ImagePointer";
import { AiTaskBridgeReason, BridgeUpdateReason, CommandBlockReason } from "./lib/BridgeCommandGate";
import { TaskVideoSignals } from "./lib/SessionVideo";
import HudNotice from "./components/HudNotice";
import { NormalizeViewSettings, ReadViewSettings, SaveViewSettings, type ViewSettings } from "./lib/ViewSettings";
import { ReadDashboardMode, SaveDashboardSetting, type DashboardMode } from "./lib/DashboardLayout";
import { NormalizeGraspGap, ReadGraspGap, SaveGraspGap } from "./lib/GraspGap";
import { AddAiUsage, EmptyAiStatistics, ReadAiUsage, type AiUsage } from "./lib/AiStatistics";
import { AiMaxOutputTokens, BuildOllamaChatBody, BuildOllamaPreloadBody, OllamaChatUrl, PreferredModel, ReadOllamaChatReply, RewarmAfterMs } from "./lib/OllamaChat";
import { AiPromptRouter, AnswerRoute, CaptureCountdownSeconds, PromptRouterOptions, RouteRecord, RouteStatusText, RouterActiveMs } from "./lib/AiPromptRouter";
import {
  RefreshCadFrame,
  ReuseServoSnapshot,
  GetIkAngleLimits,
  type ArmAngles,
  DISPLAY_HOME_OFFSET_CM,
  type IkAngleLimits,
  HOME_MODEL_TARGET_CM,
  HOME_RAW,
  FixedAnchorVisualCm,
  INITIAL_SERVOS,
  IK_JOINTS,
  IkPreview,
  IkServoName,
  Point3,
  SERVO_ORDER,
  ServoName,
  ServoState,
  TargetPose,
  angleFromRawValue,
  applyDisplayHomeOffsetToJoints,
  arcSpan,
  clampRaw,
  constrainTargetPose,
  currentAngles,
  forwardDisplayKinematics,
  forwardKinematics,
  fractionToRaw,
  mirrorDisplayPoint,
  modelPointFromDisplayPoint,
  rawToFraction,
  solveIkPreview,
  wristPitchFromAngles,
} from "./lib/robotModel";

type StatusEntry = {
  id: number;
  level: "ok" | "warn" | "info";
  text: string;
};

type SocketState = "offline" | "connecting" | "live";
type ServoLimitUpdate = {
  min: number;
  max: number;
  arcInverted: boolean;
};
type SyncedIkPreview = IkPreview & {
  receivedAt: number;
  key: string;
};
type AiMotionProposal = {
  scene?: SceneIdentity | null;
  sceneCarry?: boolean;
  target: TargetPose;
  requestedTarget: TargetPose;
  reply: string;
  rawResponse: string;
  hasTargetMotion: boolean;
  constraintIssues: string[];
  requestedRelativeInches: {
    left: number;
    farther: number;
    up: number;
  };
  requestedWristPitchTarget: number;
  wristPitchTarget: number;
  wristPitchDelta: number;
  relativeInches: {
    left: number;
    farther: number;
    up: number;
  };
  servoTargets: AiServoProposal[];
  // A CHAINED MOTION (lib/AiChain.ts): one gripper action run before or after
  // the tip move, on the same approval. Null for a single move.
  chain?: ChainGripper | null;
  // The arm pose the proposal was planned from, and whether its target depends
  // on it (a relative move, or an absolute target with an axis left out).
  anchor: TargetPose;
  relative: boolean;
  // target_cm gave both X and Y: a point copied from a measurement, even with Z left out.
  absoluteXY: boolean;
  // The target snap (lib/AiTargetSnap.ts): target is its snapped target, or
  // the request when it was exact or refused. Absent when it did not run.
  snap?: SnapResult | null;
  // What the snap and the checks used, for Approve: the proposal-time servo
  // snapshot (the IK seed), a key of the joint limits, CAD alignment and elbow
  // branch, and this frame's [UNRELIABLE] measurements.
  snapSeed?: ServoSeed;
  snapContextKey?: string;
  unreliable?: LocatedObject[];
  // The measured objects every gripper part keeps clear of, the task object's
  // GRASP PLAN or refusal (lib/GraspPlan.ts): Approve checks the same ones.
  guard?: GraspGuard | null;
  // Shown on the approval card: why this proposal is shown again, or that it is the app's planned step.
  approvalNote?: string;
  // The app's own NEXT STEP, offered after the model did not follow it (runAiRequest, PlannedStepNote).
  planned?: boolean;
  // The app's own NEXT STEP, proposed without asking the model (lib/AppPlannedSteps.ts): what it does and its JSON.
  // Planned is set too (the card's label); the check-in after it says the app ran it, not that the model failed.
  appRun?: { What: string; Json: string } | null;
  // The model's proposal is the NEXT STEP the app gave for it, as the app reads that step (SameStep): the GRASP PLAN goes on as planned.
  followsPlan?: boolean;
};
type ServoSeed = Record<ServoName, { presentRaw: number; guiAngleRaw: number | null }>;
type AiServoProposal = {
  name: ServoName;
  targetRaw: number;
};
type AiTaskStatus = "continue" | "done" | "ask";
type AiParsedResponse = {
  userMessage: string;
  motion: AiMotionProposal | null;
  rawResponse: string;
  // AI's guess of where the target object is, in the display frame
  // (x = left, y = farther, z = up), or null if it didn't estimate one.
  targetEstimate: Point3 | null;
  // TASK MODE loop verdict: "continue" keeps the autonomous loop going after the
  // motion is approved; "done"/"ask" (or a missing field) end it.
  taskStatus: AiTaskStatus | null;
};
type AiMessageContent = string | Array<
  | { type: "text"; text: string }
  | { type: "image_url"; image_url: { url: string } }
>;
type AiApiMessage = {
  role: "system" | "user" | "assistant";
  content: AiMessageContent;
};
type AiProposalValidation = {
  possible: boolean;
  issues: string[];
  preview: IkPreview | null;
};
type VisionAttachment = {
  originalImageUrl: string;
  // null when the depth server was unreachable — the color photo is still sent
  // so the vision model can see the scene; only distance refinement is lost.
  depthImageUrl: string | null;
  // The attached images. The measured positions (MeasuredObjectsSummary) are
  // added when the prompt is built, once the task object is picked among them.
  summary: string;
  // This capture's /locate result (null when the bridge did not answer).
  located: LocateResult | null;
};
// Bridge /depthcam/status (subset the UI uses).
type DepthCameraStatus = {
  available: boolean;
  calibrated: boolean;
  fps: number;
  name: string;
  warnings: string[];
  error: string | null;
};
// Bridge /calibration/status payload (subset the UI uses).
type CalibrationStatus = {
  calibrated: boolean;
  mode?: "auto" | "manual" | null;
  reprojection_px?: number | null;
  holdout_px?: number | null;
  plane_rms_cm?: number | null;
  observations_used?: number | null;
  point_count: number;
  min_points: number;
  recommended_points: number;
  rms_cm: number | null;
  max_err_cm: number | null;
  table_z: number | null;
  has_intrinsics: boolean;
  fitted_at?: string | null;
  // The saved automatic calibration's quality warnings (what used to reject it), and whether the
  // user can reject it (/selfcal/reject restores the calibration it replaced).
  quality_problems?: string[];
  can_reject?: boolean;
  replaced_fitted_at?: string | null;
  drift_baseline?: boolean;
  observation_source?: string | null;
  last_rejected?: { fitted_at?: string | null; rejected_at?: string | null } | null;
};

// Self-calibration: the arm shows a printed marker on its gripper to the camera
// from a spread of poses and the bridge fits the whole camera (see
// "back end/SelfCalibration.py"). Report fields mirror /selfcal/solve; every
// computed camera is saved, its quality problems are warnings (lib/CalibrationReport.ts).
type SelfCalReport = CalibrationFitReport & {
  fit_rms_px?: number;
  holdout_rms_px?: number | null;
  fit_rms_cm?: number;
  holdout_rms_cm?: number | null;
  plane_rms_cm?: number | null;
  observations_used?: number;
  observations_dropped?: number;
  marker_edge_mm?: number;
  camera_height_above_table_cm?: number;
  tape_assignment?: { assigned?: number; poses_with_red?: number; changed_from_tracker?: number } | null;
};

type SelfCalState = {
  running: boolean;
  step: string;
  index: number;
  total: number;
  seen: number;
  report: SelfCalReport | null;
};

type SelfCalPose = { x: number; y: number; z: number; wrist_pitch: number; plan_id: string };

const SELFCAL_SETTLE_MIN_MS = 800;
const SELFCAL_SETTLE_TIMEOUT_MS = 12000;
const SELFCAL_DWELL_MS = 500;
const SELFCAL_FRAMES_PER_POSE = 3;
const SELFCAL_FRAME_GAP_MS = 120;
const RAW_TURN = 4096;

// Shortest distance between two raw servo positions on the 0..4095 circle, so
// a joint whose arc wraps through 0 (the wrist flex does) still reads settled.
function circularRawDelta(a: number, b: number) {
  const d = (((a - b) % RAW_TURN) + RAW_TURN) % RAW_TURN;
  return Math.min(d, RAW_TURN - d);
}

function sleep(ms: number) {
  return new Promise<void>((resolve) => window.setTimeout(resolve, ms));
}

// The bridge runs on the same machine as the web server, so reach it via the
// host that served this page — works at 127.0.0.1 locally AND from other
// devices on the LAN (e.g. http://192.168.1.20:5181 from the laptop).
const BRIDGE_HOST = window.location.hostname || "127.0.0.1";
const WS_URL = `ws://${BRIDGE_HOST}:8000/ws`;
const API_URL = `http://${BRIDGE_HOST}:8000`;
const BRIDGE_LAUNCH_URL = "/bridge-launch/start";
// This page opened on the robot's PC itself (a loopback name). Any other device needs the arm password (typed once,
// kept in that browser: lib/RemoteAccess.ts) and never connects the arm by itself (back end/RemoteAccess.py).
const ThisPc = IsLoopbackHost(window.location.hostname);
// How long a Connect click counts for the automatic-connect effect (it waits for the socket and the port list).
const ConnectClickMs = 15000;
// The dev server starts a local Ollama on request (vite.config.ts).
const OLLAMA_LAUNCH_URL = "/bridge-launch/ollama";
// While no model is found, look again this often (and whenever the tab regains focus).
const MODEL_RETRY_MS = 4000;
// Model calls and the warm-up go to Ollama's native /api/chat (lib/OllamaChat.ts).
const OLLAMA_MODELS_URL = "/ollama/v1/models";
const OLLAMA_LOADED_MODELS_URL = "/ollama/api/ps";
// /api/tags lists installed models with details (parameter_size, byte size);
// /api/show reports per-model capabilities (e.g. "vision"). Together they let us
// default to the largest installed model that can actually accept images.
const OLLAMA_TAGS_URL = "/ollama/api/tags";
const OLLAMA_SHOW_URL = "/ollama/api/show";
// UniDepth depth server, proxied by Vite (see vite.config.ts) to avoid CORS.
const DEPTH_URL = "/depth-server/depth";
// Seconds to count down (3..2..1) before grabbing the webcam frame, so the scene
// can be arranged. Set to 0 to capture instantly.
const CAPTURE_COUNTDOWN_SECONDS = CaptureCountdownSeconds;
// A target released where it cannot go shows red for at most this long: the wrist-only fix
// or the nearest valid spot if the search answers in time, else the last valid target.
const TargetSnapLimitMs = 500;
const MAX_AI_REVISION_ATTEMPTS = 4;
// The arm counts as settled when every servo's present position is within this
// many raw units (~2 deg) of its target (see TaskSettleVerdictFor for the rest).
const TASK_SETTLE_RAW_TOLERANCE = 25;
// The bridge's send_ik takes gripper_raw, an opening its route makes on the way (bridge_server.py GripperWithTipVersion).
const GripperWithTipVersion = "open-while-moving-v1";

type AiTaskState = { task: string; step: number };
const FRAME_MS = 1000 / 30;
const PREVIEW_THROTTLE_MS = 30;
const AI_SYSTEM_PROMPT_LEAD = "You are controlling an SO-101 robotic arm through a web approval UI.";
// The reach facts (ReachFactsText) go between the lead and this: the arm's
// 48.7 cm length used to stand there, a reach it has nowhere near the desk.
// Floor: the task's (lib/AiTableFloor.ts): the surface its object stands on names
// the GRASP RULE's surface and the HARD FLOOR; the desk's gives the text as always.
const AiSystemPromptBase = (Floor: AiFloor = DeskFloor) => [
  "You cannot move the arm directly. If you propose a motion, the human must approve it first.",
  "You will receive the current arm tip position in X, Y, Z centimeters and the current wrist pitch angle in degrees.",
  "You can command motion two ways, in the same frame as the position you are given:",
  "1. RELATIVE: motion_proposal.move_delta_cm with left, farther, and up in centimeters. Negative left means right, negative farther means closer, and negative up means down.",
  "2. ABSOLUTE: motion_proposal.target_cm with {\"x\": ..., \"y\": ..., \"z\": ...} — the absolute tip position to move to (X positive = left, Y positive = farther, Z positive = up). Any axis you omit keeps its current value. Prefer ABSOLUTE when you know exactly where the tip should go (e.g. from MEASURED OBJECT POSITIONS).",
  "Use motion_proposal only when you are actually proposing a movement. Otherwise set motion_proposal to null.",
  Floor.Scene ? `GRASP RULE: every tip target is the inner face of the fixed jaw. The depth camera measures the object, its support and the nearby obstacles in this capture. Use the GRASP PLAN exactly: its entry point, grasp point, opening, wrist pitch and checked withdrawal. An object centre or support height is never a grasp target. If there is no plan or the scene is not verified, reply ask with no motion and explain what the camera needs to see.` : `GRASP RULE: the tip position you are given, and every tip target you send, is the inner face of the gripper's FIXED jaw (the jaw with the red tape). Only the other jaw moves: it swings open on the far side and closes the object against the fixed jaw. So a grasp never puts the tip on an object's centre or on its top: the fixed jaw goes down just BESIDE the object and the moving jaw pushes the object onto it. For the task object the app works this out from the object's measured size and the gripper's shape and gives it in the TASK OBJECT line as a GRASP PLAN: the side the fixed jaw goes to, a HOVER POINT and a GRASP POINT, each with its wrist pitch. Use exactly those numbers. ${Floor.Surface
    ? `The task object rests ON ${SurfaceText(Floor.Surface)}, not on the table (Z=${TableZCm.toFixed(1)}): its base Z is that surface, never a grasp height; never aim the tip at that surface or below it.`
    : `Objects rest ON the table (Z=${TableZCm.toFixed(1)}): base Z is the surface under an object, never a grasp height; never aim the tip at the table or below it.`} If the TASK OBJECT line says NOT GRASPABLE, or gives no GRASP PLAN for an object you must grasp, do not descend beside or onto it: reply ask.`,
  // 2026-10-01 18:54: the NEXT STEP said "go to the hover point aimed 0.7 cm past it"; the model descended instead.
  // The refusal is said only where the app makes it: a planned grasp's step (BuildGraspGuard, NextStepReminder).
  "NEXT STEP RULE (the most important rule): when the TASK OBJECT line gives a NEXT STEP with a \"motion_proposal\", your motion_proposal is that JSON copied exactly, every key and number. Do not skip it, reorder it, merge it with another step or improve it, and never descend unless the NEXT STEP says descend. When the message ends with a NEXT STEP TO COPY line (a grasp the app planned), the app refuses any other move that ends low beside the task object. If you think the NEXT STEP is wrong, reply task_status \"ask\" with motion_proposal null and say why.",
  // 2026-10-03 16:42: the pill bottle under the open jaws; the model asked twice to have it confirmed from the image (lib/ArmOcclusion.ts).
  Floor.Scene ? "HIDDEN RULE: an occluded object may retain its last measurement only while the checked scene geometry remains the same. Hidden space is not observed free space. Follow the NEXT STEP, and reply ask if it says the grasp cannot be confirmed; do not invent a lift to reveal it." : "HIDDEN RULE: when the TASK OBJECT line says the object is HIDDEN BY THE GRIPPER, the arm is over or beside where the camera last measured it and covers it in the image: that is normal while grasping, not a lost object. Its last full measurement and GRASP PLAN stand: do not ask the user to confirm it from the image, and do not reply ask because you cannot see it; follow the NEXT STEP. Reply ask only if the image shows it somewhere else, clear of the gripper.",
  Floor.Scene
    ? `SCENE LIMITS: the depth camera measured the supporting surface and surrounding obstacles in this capture. Keep the whole gripper and any held object inside the checked free space. Unknown or hidden space is not clear. Copy only the planned NEXT STEP, including its wrist pitch; do not invent a lift or shortcut. The support floor applies locally over its measured extent, and the desk safety floor still applies elsewhere.`
    : Floor.Surface
    ? `HARD FLOOR: outside the black mesh basket, never propose a motion that would put the gripper tip below Z = ${Floor.FloorZ.toFixed(1)} cm (1 cm under ${SurfaceText(Floor.Surface)}, which the task object stands on)${Floor.SurfaceZ >= TableZCm ? ", anywhere in this task" : `, over that ${Floor.Surface.Kind}; elsewhere below Z = ${AiTableFloorCm.toFixed(1)} cm (1 cm under the table surface)`}. If a downward step would end lower, shrink it so the final Z stays at or above the floor. The app refuses such targets, even if the user asks. Only over the basket's open top (Y less than ${TrayFrontRimYCm.toFixed(1)}, inside its front rim) may the tip go lower; its walls and floor are collision-checked.`
    : `HARD FLOOR: outside the black mesh basket, never propose a motion that would put the gripper tip below Z = ${AiTableFloorCm.toFixed(1)} cm (1 cm under the table surface). If a downward step would end lower, shrink it so the final Z stays at or above ${AiTableFloorCm.toFixed(1)}. The app refuses such targets, even if the user asks. Only over the basket's open top (Y less than ${TrayFrontRimYCm.toFixed(1)}, inside its front rim) may the tip go lower; its walls and floor are collision-checked.`,
  Floor.Scene ? "APPROACH RULE: use only the measured GRASP PLAN and copy its NEXT STEP. A side approach uses PREGRASP, entry, close, checked release clearance, then RETREAT. Close alone and inspect the grasp before withdrawing. Never substitute an overhead waypoint, a chained lift or a different wrist pitch in a confined space." : "APPROACH RULE: NEVER move directly to a grasp or touch target in one motion. For a grasp, the NEXT STEP takes you there one motion at a time: first the GRASP PLAN's HOVER POINT (straight above the GRASP POINT, which is beside the object, not above its centre), then, only when the NEXT STEP says descend, straight down to the GRASP POINT as its own separate motion. For any other target, first go to a waypoint directly above it (same X and Y, Z about 6 cm higher). This costs one extra motion but prevents dragging the gripper into objects or the table on the way in.",
  // The wrist is a preference (lib/AiTargetSnap.ts SnapRules.WristOnlyMaxChangeDeg): the app turns it to the nearest pitch that works.
  // Without one (target_cm) or "auto": GraspRules.PreferredPitchDeg, as the grasp planner prefers (parseAiResponse).
  `WRIST RULE: wrist_pitch_degrees is the wrist pitch you would like, in degrees, not a hard requirement: at your target point the app turns the wrist to the nearest pitch the arm can hold there (reached, above the floor, clear of the basket and of measured objects) whenever yours cannot be held, and says so. With target_cm, leave it out or write "wrist_pitch_degrees":"auto" to get the app's preferred ${GraspRules.PreferredPitchDeg} deg (steep, so the camera still sees what is under the gripper), or the nearest pitch to it the arm can hold there; a move_delta_cm step without it keeps the wrist as it is. Never call a target unreachable because of its wrist pitch alone. For the task object, use the wrist pitch the TASK OBJECT line gives: the GRASP PLAN was checked at it, and beside the object the app holds the wrist to it.`,
  // The snap (lib/AiTargetSnap.ts) with its caps; the figures come from SnapRules.
  AiSnapEnabled
    ? `REACH RULE: if your target cannot be reached exactly, the app moves it to the nearest reachable target by itself and shows the human both. It first changes only the wrist angle (${SnapRules.WristOnlyMaxChangeDeg >= 180 ? "to the nearest pitch that works at your point, however far" : `by at most ${SnapRules.WristOnlyMaxChangeDeg} deg`}), otherwise it moves the tip by at most ${SnapRules.MaxCm} cm${SnapRules.WristOnlyMaxChangeDeg >= 180 ? ", with the wrist that reaches nearest" : ""} (never lower than you asked, never below the floor, never across the basket rim). The next check-in tells you where the arm actually went and why. So aim at the point you actually want and do not pre-compensate. A steep wrist reaches less far from the shoulder (the reach figures above), so expect the app to flatten it far out. Targets more than ${SnapRules.MaxCm} cm from anything reachable, below the floor, descending onto an [UNRELIABLE] object, or bringing any part of the gripper into a measured object (above all the fixed jaw onto the task object) are still refused and returned to you, with the nearest possible target when there is one.`
    : "If your target cannot be reached exactly, the app refuses it and tells you why.",
  "You may add direct_servo_targets inside motion_proposal for slider-level actions.",
  "direct_servo_targets entries use slider names: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper. Use target_fraction from 0.0 to 1.0 or target_raw.",
  "The arm runs one move at a time: a motion_proposal changes either the tip (move_delta_cm or target_cm, with wrist_pitch_degrees) or ONE slider in direct_servo_targets, never both. The one exception is a CHAINED MOTION: one gripper action chained with one tip move.",
  // lib/AiChain.ts: the format, the order, and what is never chained (ChainRefusalReason).
  "CHAINED MOTION: to move the gripper and the tip with ONE motion_proposal, add \"gripper_before\" (the gripper moves first, then the tip; a gripper_before that OPENS the gripper opens it WHILE the tip moves, as one motion) or \"gripper_after\" (the tip moves first, then the gripper) beside its target_cm or move_delta_cm, with the gripper fraction (0.0 closed, 0.8 open). The human approves both at once and the app runs them in that order, the second only once the first has finished (an opening gripper_before: both at once), each checked with the gripper as it will be then; you are re-prompted after both. So \"gripper_before\":0.8 added to a target_cm move opens the gripper to 80% while the tip moves there; {\"move_delta_cm\":{\"left\":0,\"farther\":0,\"up\":6},\"gripper_before\":0.0} closes the gripper and then lifts straight up; \"gripper_after\":0.8 added to a move opens the gripper where the move ends. Never chained (the app refuses them): the descent to an object's grasp point (always its own motion, so the next image shows the open jaws beside it before the close, unless the gripper hides it there), a close after a tip move, a close followed by anything but a straight lift, and opening or narrowing the gripper beside an object below its top (not one the jaws hold: putting a held object down and letting go may be chained). A chained open goes to at most 0.8: a chained fraction above 0.8 runs to 0.8. A chained motion counts as ONE of your motions.",
  "During a task with a task object, never change wrist_roll: the GRASP PLAN and the camera's check of the tip tape assume it stays where it is, and the app refuses it. The one exception: a NEXT STEP that gives the GRASP PLAN's wrist_roll turn; copy it exactly.",
  "GRASP SEQUENCE: open the gripper and go to the GRASP PLAN's hover point (one chained motion), descend straight down to its grasp point with the jaws open (always its own motion), then close the gripper (the moving jaw pushes the object against the fixed jaw) and lift (one chained motion: the gripper closes first, then the tip rises). You do not pick these steps: the NEXT STEP gives the one for the arm as it is now, chained or not, and repeats or corrects a step when the arm stopped off its point (the hover point aimed past it, a rise first). Never descend onto an object with the jaws closed. Report done only after lifting with the object held. To put a held object somewhere (the basket, a cup), move above that place and open the gripper there in one chained motion (\"gripper_after\":0.8 with that move) to release it, then lift.",
  "For the gripper, target_fraction 1.0 means open and target_fraction 0.0 means closed.",
  "The UI accepts one motion_proposal at a time.",
  "Whenever you can tell where the target object is (especially from the camera image), set target_estimate_cm to your best guess of its ABSOLUTE position in the SAME X/Y/Z frame as the current arm position you are given (left positive, farther positive, up positive). Use null if you cannot tell. This only marks the target on screen with a blue box; it does NOT move the arm.",
  "",
  "TASK MODE — you operate in an autonomous loop:",
  "After each approved motion finishes, you are AUTOMATICALLY re-prompted with a fresh camera image and the new arm state. The user does not need to speak between steps, so drive the task yourself.",
  "Every reply MUST include task_status, exactly one of:",
  "- \"continue\": you are mid-task and motion_proposal contains your next step.",
  "- \"done\": the task is complete (or there is nothing to do). Set motion_proposal to null and state the outcome in one short sentence.",
  "- \"ask\": you are blocked or genuinely need information only the user has. Set motion_proposal to null and ask ONE specific question.",
  "Be fast but correct: think briefly, then commit. If the TASK OBJECT line says OUT OF REACH or BLOCKED, do not move toward it: reply \"ask\" and say what the user must change. If it says NOT GRASPABLE (the app says so only when no grasp is physically possible: too wide for the open jaws, out of reach, or every approach collides), do not move toward it either: reply \"ask\" and pass on its reason. A BEST-EFFORT GRASP is to be attempted, not refused. When it gives a GRASP PLAN or a ready-made move (this check-in, or when last measured while the object is hidden under the gripper), copy the NEXT STEP or the ready-made move exactly, wrist_pitch_degrees included (target_cm is accepted for the hover point even when tagged [UNRELIABLE]; the descent onto an [UNRELIABLE] object must be a relative move_delta_cm step). Without measurements, prefer one decisive 2-8 cm step toward the goal over timid 0.5 cm nudges or reckless full-reach jumps.",
  "Each check-in, compare the fresh image against what you expected. If your last motion did not visibly help, do NOT repeat it — change approach, or use \"ask\".",
  "Never reuse coordinates from your earlier replies: only this message's MEASURED OBJECT POSITIONS, TASK OBJECT line and arm position are current.",
  "Finish tasks on your own. Never ask permission to continue (approval is handled by the UI) and never use \"ask\" for things you can decide from the image. Do reply \"ask\" when the task object is out of reach, blocked or not graspable, when it has never been detected, or when your last two motions made no progress.",
  "You have at most 8 motions per task (a chained motion counts as one), so make each one count.",
  "",
  // No wrist pitch in the examples: the model copied the -45 into grasps the arm could not reach.
  "Reply with only JSON in this shape: {\"response_to_user\":\"short conversational response\",\"task_status\":\"continue\",\"motion_proposal\":{\"move_delta_cm\":{\"left\":3.0,\"farther\":-5.0,\"up\":2.0}},\"target_estimate_cm\":{\"left\":12.0,\"farther\":20.0,\"up\":5.0},\"notepad_update\":\"optional — only include if you have something new to write\"}.",
  "Add \"wrist_pitch_degrees\" inside motion_proposal only to set the wrist: the value the TASK OBJECT line gives, or the current wrist pitch plus the change you want (the app takes the nearest pitch that works, WRIST RULE).",
  "A gripper-only step is its own motion_proposal: {\"direct_servo_targets\":[{\"name\":\"gripper\",\"target_fraction\":0.0}]}. To chain the gripper with a tip move, put gripper_before or gripper_after in that move's motion_proposal instead of direct_servo_targets.",
  "You may use any decimal precision in move_delta_cm; multiples of 0.5 cm work best (e.g. 1.0, 3.5, -7.0).",
  "To use an absolute target instead, replace move_delta_cm with target_cm, e.g. \"motion_proposal\":{\"target_cm\":{\"x\":2.0,\"y\":14.5,\"z\":5.0}}.",
  "For no movement, reply with: {\"response_to_user\":\"your message\",\"task_status\":\"done\",\"motion_proposal\":null} (or task_status \"ask\" if you need an answer).",
].join("\n");

// The moving red patch uses the live gripper position. The fixed masking tape's
// saved coordinates (WorkspaceFrame.json FixedAnchorVisualCm) put it on the
// floor beside the cart, not on the basket rim, so the model is not given them.

type CameraView = "Behind" | "Facing";
const AI_VISION_DEPTHWISE: Record<CameraView, string> = {
  Behind: "The camera looks forward from behind the arm: the TOP of the image is FARTHER from the robot (more positive Y), the bottom is closer to it, and the arm and basket are at the bottom.",
  Facing: "The camera faces the arm from in front of it: the BOTTOM of the image is FARTHER from the robot (more positive Y), and the top is closer to the robot.",
};

const AI_VISION_PROMPT_BASE = [
  "",
  "VISION: You are given a live color photo of the workspace from a fixed camera.",
  "ORIENTATION: the photo is aligned with your X axis — an object toward the LEFT side of the image is at MORE POSITIVE X (the arm's left); toward the RIGHT side of the image is at MORE NEGATIVE X (the arm's right).",
  "Describe what you see when asked and identify the object the user refers to.",
  "Distinguish the gray/beige masking tape on the basket rim from the moving red tip tape. The gray/beige tape's position is not measured: do not take coordinates from it.",
  "MOVING MARKER: a piece of red tape stuck to the gripper's FIXED jaw, near its tip, at the end of the white gripper. It moves with the arm and always sits at the current arm tip position you are given (the fixed jaw's inner face). Use it to see where the tip is in the photo. The red and orange cables running along the arm are servo wiring, not markers.",
  "Use the tip tape as your spatial ANCHOR: judge any object's position by how far left/right and nearer/farther it sits from the tip tape (current tip position). To grasp an object, the tip tape ends up just BESIDE it, on the side the GRASP PLAN names, never on top of it; the moving jaw (no tape) closes from the other side. Prefer MEASURED OBJECT POSITIONS when provided. Report absolute object positions as target_estimate_cm, and base move_delta_cm on the arm's current position.",
  "The human approves every move, so estimate rather than refuse, except when the app marks the task object OUT OF REACH, BLOCKED or NOT GRASPABLE: then reply ask.",
].join("\n");

const AI_VISION_DEPTH_PROMPT = [
  "",
  "You are ALSO given a colorized metric depth map of the same frame, pixel-aligned with the photo (turbo colormap:",
  "blue = near the camera, red = far, black = no reading) plus a text summary of distances in centimeters. Depth is",
  "in the CAMERA frame (OpenCV: +X right, +Y down, +Z forward away from the camera, in cm), not the arm frame.",
  "Use it to tell objects apart from the table and which is nearer; take coordinates from MEASURED OBJECT POSITIONS.",
].join("\n");

// Floor: the task's (lib/AiTableFloor.ts: the surface its object stands on), the desk's without a task object.
function buildAiSystemPrompt(notepad: string, visionAttached: boolean, depthAttached: boolean, cameraView: CameraView, Floor: AiFloor = DeskFloor): string {
  const notepadSection = [
    "",
    "NOTEPAD — your persistent memory across all sessions:",
    notepad.trim() ? notepad.trim() : "(empty — nothing recorded yet)",
    "",
    "Use notepad_update in your JSON response to rewrite the full notepad whenever something worth remembering comes up.",
    "Write to it when you hit a limit unexpectedly, notice a pattern in what works or fails, or when knowing something earlier would have changed your approach.",
    "Never write object positions, arm positions or task progress in the notepad: they are out of date by the next task and would mislead you. Keep only lasting lessons.",
    "Replace the entire notepad text each time you update it.",
  ].join("\n");
  const visionSection = visionAttached
    ? AI_VISION_PROMPT_BASE + "\n" + AI_VISION_DEPTHWISE[cameraView] + (depthAttached ? AI_VISION_DEPTH_PROMPT : "")
    : "";
  return [AI_SYSTEM_PROMPT_LEAD, ReachFactsText(Floor), AiSystemPromptBase(Floor)].join("\n") + visionSection + notepadSection;
}

function cloneServos(servos: Record<ServoName, ServoState>) {
  return Object.fromEntries(
    SERVO_ORDER.map((name) => [name, { ...servos[name] }]),
  ) as Record<ServoName, ServoState>;
}

function stepPreviewServo(servo: ServoState, stepRaw: number): ServoState {
  const span = arcSpan(servo);
  if (span <= 0) return servo;

  const present = rawToFraction(servo, servo.presentRaw);
  const target = rawToFraction(servo, servo.targetRaw);
  const delta = target - present;
  const step = Math.min(Math.abs(delta), Math.max(1, stepRaw) / span);
  if (step <= 0) return servo;

  return {
    ...servo,
    presentRaw: fractionToRaw(servo, present + Math.sign(delta) * step),
  };
}

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === "object" && value !== null;
}

function asNumber(value: unknown, fallback: number) {
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

function readDepthCameraStatus(value: unknown): DepthCameraStatus | null {
  if (!isRecord(value) || value.ok !== true || typeof value.available !== "boolean") return null;
  const info = isRecord(value.info) ? value.info : {};
  return {
    available: value.available,
    calibrated: value.calibrated === true,
    fps: asNumber(value.fps, 0),
    name: typeof info.name === "string" ? info.name : "",
    warnings: Array.isArray(value.warnings) ? value.warnings.filter((item): item is string => typeof item === "string") : [],
    error: typeof value.error === "string" ? value.error : null,
  };
}

function asOptionalNumber(value: unknown): number | null {
  if (typeof value === "string") {
    const trimmed = value.trim();
    if (!trimmed) return null;
    const match = trimmed.match(/[-+]?\d*\.?\d+(?:e[-+]?\d+)?/i);
    return match ? Number(match[0]) : null;
  }
  if (isRecord(value)) {
    for (const key of ["value", "amount", "cm", "degrees", "raw", "fraction"]) {
      const nested: number | null = asOptionalNumber(value[key]);
      if (nested != null) return nested;
    }
    return null;
  }
  const number = Number(value);
  return Number.isFinite(number) ? number : null;
}

async function startBridgeFromPageLoad() {
  try {
    const response = await fetch(BRIDGE_LAUNCH_URL, { method: "POST" });
    if (!response.ok) return false;
    const data = await response.json() as unknown;
    return isRecord(data) && data.ok === true;
  } catch {
    return false;
  }
}

function readFirstNumber(source: Record<string, unknown>, keys: string[]) {
  for (const key of keys) {
    const value = asOptionalNumber(source[key]);
    if (value != null) return value;
  }
  return null;
}

function readSignedDelta(
  source: Record<string, unknown> | null,
  positiveKeys: string[],
  negativeKeys: string[],
) {
  if (!source) return null;
  const positive = readFirstNumber(source, positiveKeys);
  if (positive != null) return positive;
  const negative = readFirstNumber(source, negativeKeys);
  return negative == null ? null : -negative;
}

function firstRecord(...values: unknown[]) {
  for (const value of values) {
    if (isRecord(value)) return value;
  }
  return null;
}

function formatTenths(value: number) {
  return value.toFixed(1);
}

function formatAiPromptNumber(value: number) {
  return Number(value.toFixed(4)).toString();
}

function roundHalf(value: number) {
  return Math.round(value * 2) / 2;
}

function formatCm(value: number) {
  return roundHalf(value).toFixed(1);
}

function signedCm(value: number) {
  const rounded = formatCm(Math.abs(value));
  return value < 0 ? `-${rounded}` : rounded;
}

function directionalCm(value: number, positiveLabel: string, negativeLabel: string) {
  const label = value >= 0 ? positiveLabel : negativeLabel;
  return `${formatCm(Math.abs(value))} cm ${label}`;
}

function formatServoLabel(name: ServoName) {
  return name.replace(/_/g, " ");
}

function formatServoTarget(proposal: AiServoProposal, servos: Record<ServoName, ServoState>) {
  const servo = servos[proposal.name];
  const fraction = rawToFraction(servo, proposal.targetRaw);
  if (proposal.name === "gripper") {
    const gripperState =
      fraction >= 0.85
        ? "open"
        : fraction <= 0.15
          ? "closed"
          : `${formatTenths(fraction * 100)}%`;
    return `${gripperState} (${proposal.targetRaw})`;
  }
  return `raw ${proposal.targetRaw} (${formatTenths(fraction * 100)}%)`;
}

// The user-facing X axis (the "visual" frame shown in the Target fields,
// position tiles, AI proposal coordinates, and the 3D scene) is the negation of
// the display X. This makes moving left read as - and moving right as +.
// The workspace translation is applied before this reversible conversion.
function visualPointFromDisplayPoint<T extends Point3>(point: T): T {
  return { ...point, x: -point.x } as T;
}

// A measured object as an axis-aligned box: its footprint (display X and Y extents) from
// its support up to its top. Null without a measured footprint and height.
function ObjectBoxOf(Object_: LocatedObject | null): ObjectBox | null {
  const Height = Object_?.heightCm, Size = Object_?.sizeCm;
  if (!Object_ || !Size || Height == null || !(Height > 0) || !(Size[0] > 0) || !(Size[1] > 0)) return null;
  const Bottom = Object_.topZ != null ? Object_.topZ - Height : Object_.z;
  return { Center: { x: Object_.x, y: Object_.y, z: Bottom + Height / 2 }, SizeCm: [Size[0], Size[1], Height], Label: Object_.label };
}

function displayPointFromVisualPoint<T extends Point3>(point: T): T {
  return { ...point, x: -point.x } as T;
}

// The 3D scene renders in the same user-facing "visual" frame as the numeric
// coordinate readouts (visualPointFromDisplayPoint negates display X). Sharing
// one frame is what keeps the reversed X axis consistent: a point that reads
// X = +5 in the Target / Current Position panels sits on the +X (red axis) side
// of the 3D view, and dragging the target right raises its X in both places.
function sceneJointsFromDisplayJoints(joints: Point3[]): Point3[] {
  return joints.map(visualPointFromDisplayPoint);
}

function proposalDisplayPointCm(target: TargetPose) {
  return visualPointFromDisplayPoint(mirrorDisplayPoint(target));
}

function currentDisplayPointCm(point: Point3) {
  return mirrorDisplayPoint(point);
}

function modelPointFromVisualPoint(point: Point3): Point3 {
  return modelPointFromDisplayPoint(displayPointFromVisualPoint(point));
}

function coordinateTiles(point: { x: number; y: number; z: number }) {
  return [
    { axis: "X", value: point.x },
    { axis: "Y", value: point.y },
    { axis: "Z", value: point.z },
  ];
}

function readPointList(value: unknown): Point3[] | null {
  if (!Array.isArray(value)) return null;
  const points: Point3[] = [];
  for (const item of value) {
    if (!isRecord(item)) return null;
    const x = asNumber(item.x, Number.NaN);
    const y = asNumber(item.y, Number.NaN);
    const z = asNumber(item.z, Number.NaN);
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) return null;
    points.push({ x, y, z });
  }
  return points.length ? points : null;
}

function readIkAngles(value: unknown): ArmAngles | null {
  if (!isRecord(value)) return null;
  const angles = {} as ArmAngles;
  for (const name of SERVO_ORDER) {
    const angle = asNumber(value[name], Number.NaN);
    if (!Number.isFinite(angle)) return null;
    angles[name] = angle;
  }
  return angles;
}

function previewKey(target: TargetPose, elbowMode: "up" | "down", Roll = 0, Grip = 0, Revision = 0) {
  return [
    target.x.toFixed(2),
    target.y.toFixed(2),
    target.z.toFixed(2),
    target.wristPitch.toFixed(2),
    elbowMode,
    Roll.toFixed(5), Grip.toFixed(5), Revision,
  ].join("|");
}

function mapBackendPreview(
  value: unknown,
  key: string,
  target: TargetPose,
): SyncedIkPreview | null {
  if (!isRecord(value) || value.ok !== true) return null;
  const angles = readIkAngles(value.angles);
  const joints = readPointList(value.joints);
  if (!angles || !joints) return null;
  return {
    key,
    receivedAt: Date.now(),
    angles,
    joints: applyDisplayHomeOffsetToJoints(joints),
    reachable: Boolean(value.reachable),
    collisionReason: typeof value.collision_reason === "string" ? value.collision_reason : null,
    errorCm: asNumber(value.error_cm, Number.POSITIVE_INFINITY),
    requestedWristPitch: asNumber(value.requested_wrist_pitch, target.wristPitch),
    effectiveWristPitch: asNumber(value.effective_wrist_pitch, target.wristPitch),
    wristPitchReachable: Boolean(value.wrist_pitch_reachable),
    jointLimitsApplied: false,
  };
}

// Captures are flipped left-right when the camera faces the arm (Settings ›
// camera view): then the raw image-left is the arm's right, and the flip makes
// image-left = +X (the arm's left) so the vision model's left/right matches
// the frame it plans in. A camera behind the arm looking forward (the Orbbec
// on the robot's head) already shows image-left = +X and is not flipped.
// Every consumer of a capture (chat vision, /locate, calibration, depth maps,
// the archive) is sent that capture's own mirroring, so a change of camera or
// view never mixes the two.
function captureVideoFrame(video: HTMLVideoElement | null, mirrored: boolean): string | null {
  if (!video || video.dataset.captureReady !== "true" || video.readyState < HTMLMediaElement.HAVE_CURRENT_DATA) return null;
  // The depth camera's canvas holds exactly the frame whose depth the bridge
  // keeps (DepthSequenceFor), so capture from it rather than from the video.
  const source = DepthCanvasFor(video) ?? video;
  const width = source instanceof HTMLCanvasElement ? source.width : video.videoWidth;
  const height = source instanceof HTMLCanvasElement ? source.height : video.videoHeight;
  if (width <= 0 || height <= 0) return null;

  const canvas = document.createElement("canvas");
  canvas.width = width;
  canvas.height = height;
  const ctx = canvas.getContext("2d");
  if (!ctx) return null;
  if (mirrored) {
    ctx.translate(width, 0);
    ctx.scale(-1, 1);
  }
  ctx.drawImage(source, 0, 0, width, height);
  return canvas.toDataURL("image/jpeg", 0.9);
}

// Fire-and-forget archive of every workspace photo the AI sees, saved by the
// bridge under "back end/captures/<date>/". Mirrored captures are un-flipped
// server-side so the archive shows the desk in natural orientation. Failures
// are silent — archiving must never stall or break the vision flow.
function archiveCapture(imageDataUrl: string, kind: string, mirrored: boolean): void {
  void fetch(`${API_URL}/captures`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ image: imageDataUrl, kind, mirrored }),
  }).catch(() => {});
}

// The session record (scripts/SessionLog.ts, beside the video that
// back end/SessionRecorder.py writes): the prompts typed, how each reply was
// read, and what was approved, blocked or sent. The raw model traffic is
// recorded by the dev server's /ollama proxy. Automated test browsers stay out
// of the record. Fire-and-forget: recording never delays or breaks the AI.
function recordSessionEvent(type: string, details: Record<string, unknown> = {}): void {
  if (navigator.webdriver) return;
  try {
    void fetch("/session-log/event", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ type, client_time: new Date().toISOString(), ...details }),
      keepalive: true,
    }).catch(() => {});
  } catch {
    // Recording is best effort.
  }
}

// A proposal as the record keeps it: the target in the AI's display frame,
// with what the model asked for and the snap that moved it (if any).
function motionRecord(motion: AiMotionProposal | null): Record<string, unknown> | null {
  if (!motion) return null;
  const target = mirrorDisplayPoint(motion.target);
  const requested = mirrorDisplayPoint(motion.requestedTarget);
  return {
    target_cm: motion.hasTargetMotion ? { x: target.x, y: target.y, z: target.z } : null,
    wrist_pitch_degrees: motion.wristPitchTarget,
    requested_target_cm: motion.hasTargetMotion ? { x: requested.x, y: requested.y, z: requested.z } : null,
    requested_wrist_pitch_degrees: motion.requestedWristPitchTarget,
    move_delta_cm: motion.relativeInches,
    servo_targets: motion.servoTargets.map((item) => ({ name: item.name, raw: item.targetRaw })),
    // A chained motion's gripper action and its order (lib/AiChain.ts).
    chain: motion.chain ? { order: motion.chain.Order, gripper_raw: motion.chain.TargetRaw, gripper_fraction: motion.chain.Fraction } : null,
    constraint_issues: motion.constraintIssues,
    snap: snapRecord(motion.snap ?? null),
    // The app's planned NEXT STEP, not the model's proposal.
    ...(motion.planned ? { planned: true } : {}),
    // Proposed by the app without a model call (lib/AppPlannedSteps.ts).
    ...(motion.appRun ? { app_run: true } : {}),
  };
}

function snapRecord(snap: SnapResult | null): Record<string, unknown> | null {
  if (!snap) return null;
  const point = (pose: TargetPose) => { const display = mirrorDisplayPoint(pose); return { x: display.x, y: display.y, z: display.z, wrist_pitch_degrees: pose.wristPitch }; };
  return snap.Kind === "refused"
    ? { kind: "refused", code: snap.Code, text: snap.Text, reason: snap.Reason?.Text ?? null, evaluations: snap.Evaluations,
      nearest: snap.Nearest ? { ...point(snap.Nearest.Target), distance_cm: snap.Nearest.DistanceCm, why: snap.Nearest.Why } : null }
    : { kind: snap.Kind, distance_cm: snap.DistanceCm, pitch_change_deg: snap.PitchChangeDeg, reason: snap.Reason?.Text ?? null,
      snapped: point(snap.Target), evaluations: snap.Evaluations };
}

// The snap of a proposal as the check-ins describe it (AI display frame), or
// null when its target was not moved.
function taskMotionSnap(motion: AiMotionProposal): TaskMotionSnap | null {
  const snap = motion.snap;
  if (!snap || (snap.Kind !== "pitch" && snap.Kind !== "position")) return null;
  return { Requested: mirrorDisplayPoint(snap.Requested), RequestedPitch: snap.Requested.wristPitch,
    Target: mirrorDisplayPoint(snap.Target), Pitch: snap.Target.wristPitch, DistanceCm: snap.DistanceCm, Reason: snap.Reason?.Text ?? null };
}

// --- AI target snap (lib/AiTargetSnap.ts) -----------------------------------

// The proposal-time servo reading the snap and the checks were made from.
function servoSeed(servos: Record<ServoName, ServoState>): ServoSeed {
  return Object.fromEntries(SERVO_ORDER.map(name => [name, { presentRaw: servos[name].presentRaw, guiAngleRaw: servos[name].guiAngleRaw ?? null }])) as ServoSeed;
}

// Live servos (their slider ranges and targets) read at the seed's position:
// Approve checks the target from the same IK seed it was chosen with, so a
// raw tick or two of jitter since the proposal cannot flip the verdict.
function seededServos(live: Record<ServoName, ServoState>, seed: ServoSeed | undefined): Record<ServoName, ServoState> {
  if (!seed) return live;
  return Object.fromEntries(SERVO_ORDER.map(name => [name, { ...live[name], ...seed[name] }])) as Record<ServoName, ServoState>;
}

// What a snap depends on besides the seed: the joint limits it applied, the
// CAD alignment and the elbow branch. Approve re-snaps only when this changed.
function aiSnapContextKey(servos: Record<ServoName, ServoState>, limits: IkAngleLimits | undefined, elbowMode: "up" | "down") {
  return JSON.stringify({ limits: GetIkAngleLimits(servos, limits), cad: GetCadAlignment().Version, elbowMode });
}

// A proposal moved to its snap's target. Exact and refused snaps keep the
// request (a refusal becomes a validation issue); the tip move stays a tip
// move even when the snap lands on the current tip (a no-op, caught later).
function applyAiSnap(motion: AiMotionProposal, snap: SnapResult): AiMotionProposal {
  if (snap.Kind !== "pitch" && snap.Kind !== "position") return { ...motion, snap };
  const target = snap.Target;
  return {
    ...motion,
    snap,
    target,
    relativeInches: { left: motion.anchor.x - target.x, farther: target.y - motion.anchor.y, up: target.z - motion.anchor.z },
    wristPitchTarget: target.wristPitch,
    wristPitchDelta: target.wristPitch - motion.anchor.wristPitch,
  };
}

// The approval card's line for a snapped target, in the visual frame of the
// coordinate tiles (X negated from the AI's): what the model asked for, where
// the arm will go instead, how far, and why. Null when nothing was moved.
function approvalSnapNote(motion: AiMotionProposal): string | null {
  const snap = motion.snap;
  if (!snap || (snap.Kind !== "pitch" && snap.Kind !== "position")) return null;
  const point = (pose: TargetPose) => {
    const visual = proposalDisplayPointCm(pose);
    return `X ${visual.x.toFixed(1)} Y ${visual.y.toFixed(1)} Z ${visual.z.toFixed(1)}, wrist ${formatTenths(pose.wristPitch)}`;
  };
  const turn = Math.abs(snap.Target.wristPitch - snap.Requested.wristPitch);
  const wrist = turn < 0.05 ? "wrist unchanged"
    : `wrist ${formatTenths(turn)} deg ${Math.abs(snap.Target.wristPitch) < Math.abs(snap.Requested.wristPitch) ? "flatter" : "steeper"}`;
  return `Asked for ${point(snap.Requested)} -> will move to ${point(snap.Target)} (`
    + `${snap.Kind === "pitch" ? "same point" : `${snap.DistanceCm.toFixed(2)} cm away`}; ${wrist}).`
    + (snap.Reason ? ` Why: ${snap.Reason.Text}.` : "");
}

// The task object's reach verdict and GRASP PLAN (lib/AiMeasurements.ts TaskObjectReachability), planned in a
// worker (lib/GraspPlanWorker.ts) so the page keeps handling clicks: planned here, a grasp held the page up to
// 11.8 s (2026-10-04 04:56), Emergency Shutoff and Stop and hold included (2026-10-07 audit M4). A grasp on a
// measured scene can take a minute (the 2026-10-06 cabinet's 1201 obstacle boxes: about 48 s for the strict passes
// on this PC when idle, then about 9 s of best-effort work), so the timeout is generous: the page stays responsive
// meanwhile and Stop ends it at once. A worker that cannot start plans here as before; one that fails or times out,
// or a task stopped meanwhile (Signal), resolves null.
const GRASP_PLAN_TIMEOUT_MS = 180000;
function PlanReachability(Object_: LocatedObject, Context: ReachContext, Planned?: ReachPlanned, Signal?: AbortSignal): Promise<TaskReachability | null> {
  let worker: Worker;
  try { worker = new Worker(new URL("./lib/GraspPlanWorker.ts", import.meta.url), { type: "module" }); }
  catch { return Promise.resolve(TaskObjectReachability(Object_, Context, Planned)); }
  return new Promise((resolve) => {
    let done = false;
    const finish = (Reach: TaskReachability | null) => {
      if (done) return;
      done = true;
      window.clearTimeout(timer);
      Signal?.removeEventListener("abort", abort);
      worker.terminate();
      resolve(Reach);
    };
    const abort = () => finish(null);
    const timer = window.setTimeout(() => finish(null), GRASP_PLAN_TIMEOUT_MS);
    Signal?.addEventListener("abort", abort);
    if (Signal?.aborted) { finish(null); return; }
    worker.onmessage = (event: MessageEvent<{ Reach: TaskReachability | null; Error?: string }>) => finish(event.data.Reach);
    worker.onerror = () => finish(null);
    worker.onmessageerror = () => finish(null);
    // A request that cannot be copied to the worker plans here, as a worker that cannot start does.
    try { worker.postMessage({ Object: Object_, Context, Planned, Alignment: GetCadAlignment() }); }
    catch { finish(TaskObjectReachability(Object_, Context, Planned)); }
  });
}

const AI_SNAP_TIMEOUT_MS = 15000;
// The refusals and the exact check cost an IK solve or two and run here; a
// target that must move waits for the search in the target worker (seconds
// for a position snap) without freezing the page. A worker that fails, times
// out or is aborted leaves the exact check's verdict: refused "timeout".
function runAiSnap(input: SnapInput, signal?: AbortSignal): Promise<SnapResult> {
  const quick = SnapAiTarget(input, SnapRules, true);
  if (quick.Kind !== "refused" || quick.Code !== "timeout") return Promise.resolve(quick);
  return new Promise((resolve) => {
    let worker: Worker;
    try { worker = new Worker(new URL("./lib/TargetWorkspaceWorker.ts", import.meta.url), { type: "module" }); }
    catch { resolve(quick); return; }
    let done = false;
    const finish = (result: SnapResult | null) => {
      if (done) return;
      done = true;
      window.clearTimeout(timer);
      signal?.removeEventListener("abort", abort);
      worker.terminate();
      resolve(result ?? quick);
    };
    const abort = () => finish(null);
    const timer = window.setTimeout(() => finish(null), AI_SNAP_TIMEOUT_MS);
    signal?.addEventListener("abort", abort);
    if (signal?.aborted) { finish(null); return; }
    worker.onmessage = (event: MessageEvent<{ Result: SnapResult | null }>) => finish(event.data.Result);
    worker.onerror = () => finish(null);
    worker.postMessage({ Kind: "AiSnap", Input: input, Rules: SnapRules, Alignment: GetCadAlignment() });
  });
}

// --- Chained motion (lib/AiChain.ts) ------------------------------------------

// The servos with the gripper's present and target at Raw: how a chain leaves it.
function servosWithGripper(servos: Record<ServoName, ServoState>, Raw: number): Record<ServoName, ServoState> {
  return { ...servos, gripper: { ...servos.gripper, presentRaw: Raw, targetRaw: Raw } };
}

// The servos a proposal's tip move runs with: after a gripper_before, the jaws
// are at the chained fraction, so the IK, the floor, the snap and the guard
// check the move with the gripper as it will be then.
function chainTipServos(motion: AiMotionProposal, servos: Record<ServoName, ServoState>) {
  return motion.chain?.Order === "before" && motion.hasTargetMotion ? servosWithGripper(servos, motion.chain.TargetRaw) : servos;
}

// A proposal as the no-op rule sees it: a chain's gripper action moves a servo.
// Whether two proposals read from the same anchor ask for the same motion: the same target and wrist (as asked, before
// any snap), the same servo targets and the same chained gripper action. A model's copy of the NEXT STEP is that step.
function SameStep(A: AiMotionProposal, B: AiMotionProposal) {
  const Near = (P: TargetPose, Q: TargetPose) => Math.hypot(P.x - Q.x, P.y - Q.y, P.z - Q.z) <= 0.15;
  const Servos = (Items: AiServoProposal[]) => Items.map(Item => `${Item.name}:${Math.round(Item.targetRaw / 5)}`).sort().join();
  return A.hasTargetMotion === B.hasTargetMotion && (!A.hasTargetMotion || (Near(A.requestedTarget, B.requestedTarget)
      && Math.abs(A.requestedWristPitchTarget - B.requestedWristPitchTarget) <= 0.5))
    && Servos(A.servoTargets) === Servos(B.servoTargets)
    && (A.chain?.Order ?? null) === (B.chain?.Order ?? null) && Math.abs((A.chain?.Fraction ?? -1) - (B.chain?.Fraction ?? -1)) <= 0.005;
}

function noOpView(motion: AiMotionProposal) {
  return { hasTargetMotion: motion.hasTargetMotion, target: motion.target,
    servoTargets: motion.chain ? [...motion.servoTargets, { name: "gripper" as ServoName, targetRaw: motion.chain.TargetRaw }] : motion.servoTargets };
}

// The legs of an approved chain, in the order they run. Together (the bridge
// takes a gripper target with a tip move): an opening gripper_before is ONE
// leg, the tip move whose route opens the jaws on the way.
function chainLegs(motion: AiMotionProposal, Together = false): ChainLeg[] {
  const Chain = motion.chain;
  if (!Chain) return [];
  if (Together && Chain.Together) return [{ Kind: "together", Target: motion.target, TargetRaw: Chain.TargetRaw, Fraction: Chain.Fraction }];
  const Gripper: ChainLeg = { Kind: "gripper", TargetRaw: Chain.TargetRaw, Fraction: Chain.Fraction };
  const Tip: ChainLeg = { Kind: "tip", Target: motion.target };
  return Chain.Order === "before" ? [Gripper, Tip] : [Tip, Gripper];
}

// The bridge command for a tip move (model frame); GripperRaw: the gripper
// target its route opens the jaws to on the way (a chain run together).
function ikCommand(pose: TargetPose, GripperRaw: number | null = null, Scene?: SceneIdentity | null, Carry = false): Record<string, unknown> {
  return { type: "send_ik", target: { x: pose.x, y: pose.y, z: pose.z, wrist_pitch: pose.wristPitch },
    ...(GripperRaw == null ? {} : { gripper_raw: GripperRaw }), ...SceneCommandFields(Scene, Carry) };
}

// The chain's two legs in words (AI display frame), for the approval card, the check-in and the record.
function chainTexts(motion: AiMotionProposal, servos: Record<ServoName, ServoState>) {
  const Chain = motion.chain!;
  const Point = mirrorDisplayPoint(motion.target);
  return {
    Gripper: ChainGripperText(rawToFraction(servos.gripper, servos.gripper.targetRaw), Chain.Fraction, Chain.Asked ?? null),
    Tip: `move the tip to X ${formatAiPromptNumber(Number(Point.x.toFixed(2)))}, Y ${formatAiPromptNumber(Number(Point.y.toFixed(2)))}, Z ${formatAiPromptNumber(Number(Point.z.toFixed(2)))} cm, wrist ${formatAiPromptNumber(Number(motion.target.wristPitch.toFixed(1)))} deg`,
  };
}

// The approval card's two steps, in order, in the visual frame of the
// coordinate tiles (X negated from the AI's); run together, its one step.
function chainCardSteps(motion: AiMotionProposal, servos: Record<ServoName, ServoState>, Together = false): string[] {
  const Chain = motion.chain!;
  const Visual = proposalDisplayPointCm(motion.target);
  const Words = ChainGripperText(rawToFraction(servos.gripper, servos.gripper.targetRaw), Chain.Fraction, Chain.Asked ?? null);
  const Gripper = `${Words[0].toUpperCase()}${Words.slice(1)} (raw ${Chain.TargetRaw})`;
  const Tip = `Move the tip to X ${formatCm(Visual.x)} Y ${formatCm(Visual.y)} Z ${formatCm(Visual.z)} cm, wrist ${formatTenths(motion.wristPitchTarget)} deg`;
  if (Together && Chain.Together) return [`${Gripper} while moving the tip to X ${formatCm(Visual.x)} Y ${formatCm(Visual.y)} Z ${formatCm(Visual.z)} cm, wrist ${formatTenths(motion.wristPitchTarget)} deg`];
  return Chain.Order === "before" ? [Gripper, Tip] : [Tip, Gripper];
}

// Ask the bridge to measure object positions in the frame via the calibrated
// camera (qwen picks the pixels, the geometry does the measuring). With a
// depth-camera capture (depthSeq) the bridge measures each object in 3D from
// that frame's depth. The chat's vision model does the grounding, so the first
// request after Ollama starts does not fall back to a model that cannot see.
// The bridge checks the camera pose against the calibration first and refuses
// with an error_code when it moved. Returns null when the bridge did not
// answer or the reply is unreadable; the caller decides whether that is fatal.
// An aborted call (the task ended, or its time limit ran out) throws instead.
// A task's object goes with it: the bridge runs one strict query for it and
// flags the measured object its box lands on (task_match), or measures that
// box on its own (task_query_only) when no sample named it.
async function locateObjects(
  imageDataUrl: string,
  depthSeq: number | null,
  model: string | null,
  mirrored: boolean,
  Signal?: AbortSignal,
  TaskObject: TaskObjectQuery | null = null,
  CarryScene: SceneIdentity | null = null,
): Promise<LocateResult | null> {
  try {
    const response = await fetch(`${API_URL}/locate`, {
      method: "POST",
      signal: Signal,
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ image: imageDataUrl, depth_seq: depthSeq, mirrored, model: model ?? undefined,
        task_object: TaskObject ? { phrase: TaskObject.Phrase, label: TaskObject.Name,
          ...(CarryScene ? { carrying: true, carry_scene_id: CarryScene.id } : {}) } : undefined }),
    });
    if (!response.ok) return null;
    return ReadLocateResponse(await response.json() as unknown);
  } catch (Failure) {
    if (Signal?.aborted) throw Failure;
    return null;
  }
}

function readDepthSummary(value: unknown): { depthImageUrl: string; summary: string } | null {
  if (!isRecord(value) || value.ok !== true || typeof value.depth_image !== "string") {
    return null;
  }

  const width = asNumber(value.width, 0);
  const height = asNumber(value.height, 0);
  const stats = isRecord(value.depth_stats_cm) ? value.depth_stats_cm : {};
  const colormap = isRecord(value.colormap) ? value.colormap : {};
  const minCm = asNumber(stats.min_cm, Number.NaN);
  const medianCm = asNumber(stats.median_cm, Number.NaN);
  const maxCm = asNumber(stats.max_cm, Number.NaN);
  const nearCm = asNumber(colormap.near_cm, Number.NaN);
  const farCm = asNumber(colormap.far_cm, Number.NaN);

  const sensor = value.source === "orbbec";
  const lines = [
    "Attached vision images:",
    sensor ? "1. Live color frame from the depth camera." : "1. Original live webcam frame.",
    sensor
      ? "2. Depth MEASURED by the camera's stereo depth sensor, aligned pixel-for-pixel with image 1 (black = no reading)."
      : "2. UniDepth colorized metric depth map estimated from the same frame.",
    "Depth map convention: turbo colormap, blue/near through red/far; camera frame is OpenCV +X right, +Y down, +Z forward.",
  ];
  if (width > 0 && height > 0) {
    lines.push(`Depth image size: ${width}x${height}px.`);
  }
  if (Number.isFinite(minCm) && Number.isFinite(medianCm) && Number.isFinite(maxCm)) {
    lines.push(`${sensor ? "Measured" : "Estimated"} depth stats: min ${formatAiPromptNumber(minCm)} cm, median ${formatAiPromptNumber(medianCm)} cm, max ${formatAiPromptNumber(maxCm)} cm.`);
  }
  if (Number.isFinite(nearCm) && Number.isFinite(farCm)) {
    lines.push(`Depth colormap clipped range: near ${formatAiPromptNumber(nearCm)} cm, far ${formatAiPromptNumber(farCm)} cm.`);
  }

  return {
    depthImageUrl: value.depth_image,
    summary: lines.join("\n"),
  };
}

// Depth for the AI: the depth camera's measured frame when the capture came
// from it (depthSeq), otherwise the UniDepth server's estimate from the photo.
async function createDepthAttachment(originalImageUrl: string, depthSeq: number | null, mirrored: boolean,
  Signal?: AbortSignal): Promise<Omit<VisionAttachment, "located"> | null> {
  const response = depthSeq != null
    ? await fetch(`${API_URL}/depthcam/depth`, {
      method: "POST",
      signal: Signal,
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ seq: depthSeq, mirrored }),
    })
    : await fetch(DEPTH_URL, {
      method: "POST",
      signal: Signal,
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ image: originalImageUrl, normalized: true }),
    });
  if (!response.ok) {
    throw new Error(await response.text());
  }
  const depth = readDepthSummary(await response.json() as unknown);
  if (!depth) {
    throw new Error("Depth server returned an unreadable depth image");
  }
  return {
    originalImageUrl,
    depthImageUrl: depth.depthImageUrl,
    summary: depth.summary,
  };
}

function extractJsonObject(text: string): unknown | null {
  const fenced = text.match(/```(?:json)?\s*([\s\S]*?)```/i);
  const source = fenced?.[1] ?? text;
  const start = source.indexOf("{");
  if (start < 0) return null;

  let depth = 0;
  let inString = false;
  let escaped = false;
  for (let i = start; i < source.length; i += 1) {
    const char = source[i];
    if (inString) {
      if (escaped) {
        escaped = false;
      } else if (char === "\\") {
        escaped = true;
      } else if (char === "\"") {
        inString = false;
      }
      continue;
    }
    if (char === "\"") {
      inString = true;
      continue;
    }
    if (char === "{") depth += 1;
    if (char === "}") {
      depth -= 1;
      if (depth === 0) {
        try {
          return JSON.parse(source.slice(start, i + 1));
        } catch {
          return null;
        }
      }
    }
  }

  return null;
}

function isServoName(value: unknown): value is ServoName {
  return typeof value === "string" && SERVO_ORDER.includes(value as ServoName);
}

function readServoActionFraction(item: Record<string, unknown>) {
  if (typeof item.open === "boolean") return item.open ? 1 : 0;
  if (typeof item.closed === "boolean") return item.closed ? 0 : 1;

  const text = [
    item.action,
    item.position,
    item.state,
    item.command,
    item.intent,
  ]
    .filter((value): value is string => typeof value === "string")
    .join(" ")
    .replace(/[_-]/g, " ")
    .toLowerCase();

  if (/\b(open|opened|release|released)\b/.test(text)) return 1;
  if (/\b(close|closed|shut|grip|grab|clamp|hold)\b/.test(text)) return 0;
  return null;
}

function parseAiServoTargets(
  motionSource: Record<string, unknown>,
  servos: Record<ServoName, ServoState>,
): AiServoProposal[] {
  const source =
    motionSource.direct_servo_targets
    ?? motionSource.slider_targets
    ?? motionSource.servo_targets;
  const items = Array.isArray(source) ? source : isRecord(source) ? [source] : [];
  const targets = new Map<ServoName, AiServoProposal>();

  for (const item of items) {
    if (!isRecord(item)) continue;
    const rawName = item.name ?? item.servo ?? item.slider;
    if (!isServoName(rawName)) continue;

    const servo = servos[rawName];
    let targetRaw = asOptionalNumber(
      item.target_raw
      ?? item.raw
      ?? item.value_raw
      ?? item.slider_raw,
    );

    if (targetRaw == null) {
      const targetFraction = asOptionalNumber(
        item.target_fraction
        ?? item.fraction
        ?? item.position_fraction
        ?? item.open_fraction,
      );
      if (targetFraction != null) {
        targetRaw = fractionToRaw(servo, targetFraction);
      }
    }

    if (targetRaw == null) {
      const actionFraction = readServoActionFraction(item);
      if (actionFraction != null) {
        targetRaw = fractionToRaw(servo, actionFraction);
      }
    }

    if (targetRaw == null) continue;
    targets.set(rawName, { name: rawName, targetRaw: clampRaw(targetRaw) });
  }

  return [...targets.values()];
}

function hasServoTargetSource(source: Record<string, unknown>) {
  const targetSource =
    source.direct_servo_targets
    ?? source.slider_targets
    ?? source.servo_targets;
  return Array.isArray(targetSource) || isRecord(targetSource);
}

// A target_cm without a wrist pitch at least this far from the tip gets the preferred pitch (WRIST RULE); nearer, it
// restates where the tip is (with a gripper action, say), and the wrist stays.
const WristChoiceCm = 1;

function parseAiMotionProposalFromObject(
  parsed: Record<string, unknown>,
  currentTarget: TargetPose,
  rawResponse: string,
  servos: Record<ServoName, ServoState>,
): AiMotionProposal | null {
  const motionSource = firstRecord(
    parsed.motion_proposal,
    parsed.motionProposal,
    parsed.movement_proposal,
    parsed.movementProposal,
    parsed.movement,
    parsed.motion,
    parsed.proposal,
  ) ?? parsed;
  const delta = firstRecord(
    motionSource.move_delta_cm,
    motionSource.moveDeltaCm,
    motionSource.delta_cm,
    motionSource.target_delta_cm,
    motionSource.relative_delta_cm,
    motionSource.move_delta_inches,
    motionSource.delta,
    motionSource.relative_move,
    motionSource.relative,
    motionSource.move,
  ) ?? motionSource;
  const absoluteTarget = firstRecord(
    motionSource.target_cm,
    motionSource.target,
    motionSource.target_position_cm,
    motionSource.target_position,
    motionSource.coordinates_cm,
    motionSource.coordinates,
  );

  const left = readSignedDelta(
    delta,
    ["left", "left_cm", "left_inches", "x", "x_cm", "x_inches", "dx", "dx_cm", "delta_x", "delta_x_cm", "deltaX", "leftRight", "left_right"],
    ["right", "right_cm", "right_inches"],
  );
  const farther = readSignedDelta(
    delta,
    ["farther", "farther_cm", "farther_inches", "forward", "forward_cm", "forward_inches", "away", "away_cm", "away_inches", "y", "y_cm", "y_inches", "dy", "dy_cm", "delta_y", "delta_y_cm", "deltaY"],
    ["closer", "closer_cm", "closer_inches", "back", "back_cm", "backward", "backward_cm", "backward_inches", "toward", "toward_cm", "toward_inches"],
  );
  const up = readSignedDelta(
    delta,
    ["up", "up_cm", "up_inches", "z", "z_cm", "z_inches", "dz", "dz_cm", "delta_z", "delta_z_cm", "deltaZ"],
    ["down", "down_cm", "down_inches"],
  );
  // The AI gives absolute targets in the frame it receives positions in (display
  // frame: X positive = left), so fall back to the display-frame current target.
  const displayCurrent = mirrorDisplayPoint(currentTarget);
  const absoluteDisplayX = absoluteTarget
    ? readFirstNumber(absoluteTarget, ["x", "x_cm", "left", "left_cm", "leftRight", "left_right"])
    : null;
  const absoluteY = absoluteTarget
    ? readFirstNumber(absoluteTarget, ["y", "y_cm", "farther", "farther_cm", "forward", "forward_cm", "away", "away_cm"])
    : null;
  const absoluteZ = absoluteTarget
    ? readFirstNumber(absoluteTarget, ["z", "z_cm", "up", "up_cm", "height", "height_cm"])
    : null;
  const hasAbsoluteTarget = absoluteDisplayX != null || absoluteY != null || absoluteZ != null;
  const wristPitchTarget = asOptionalNumber(
    motionSource.wrist_pitch_degrees
      ?? motionSource.target_wrist_pitch_degrees
      ?? motionSource.wrist_pitch,
  );
  const wristDelta = asOptionalNumber(
    motionSource.wrist_pitch_delta_degrees
      ?? delta?.wrist_pitch_delta_degrees,
  ) ?? 0;
  // "auto", or no wrist pitch with a target_cm that moves the tip (WristChoiceCm): the preferred pitch
  // (GraspRules.PreferredPitchDeg), the snap turning it to the nearest the arm holds there. A relative step
  // without one, or a target_cm restating where the tip is, keeps the wrist.
  const wristWord = motionSource.wrist_pitch_degrees ?? motionSource.target_wrist_pitch_degrees ?? motionSource.wrist_pitch;
  const wristAuto = typeof wristWord === "string" && wristWord.trim().toLowerCase() === "auto";
  const servoTargets = parseAiServoTargets(motionSource, servos);
  // gripper_before / gripper_after: a CHAINED MOTION (lib/AiChain.ts).
  const chainRead = ReadChainGripper(motionSource);

  if (
    left == null
    && farther == null
    && up == null
    && !hasAbsoluteTarget
    && wristPitchTarget == null
    && !wristAuto
    && wristDelta === 0
    && servoTargets.length === 0
    && chainRead == null
  ) {
    return null;
  }

  const absolutePoint = hasAbsoluteTarget
    ? modelPointFromDisplayPoint({ x: absoluteDisplayX ?? displayCurrent.x, y: absoluteY ?? displayCurrent.y, z: absoluteZ ?? displayCurrent.z })
    : null;
  const preferredWrist = wristDelta === 0 && (wristAuto || (absolutePoint != null
    && Math.hypot(absolutePoint.x - currentTarget.x, absolutePoint.y - currentTarget.y, absolutePoint.z - currentTarget.z) >= WristChoiceCm))
    ? GraspRules.PreferredPitchDeg : null;
  const rawRequestedTarget = absolutePoint
    ? {
      // Display-frame absolute coords → model frame. Previously this went through
      // modelPointFromVisualPoint, which negates X a second time — so every
      // absolute X the AI sent executed mirrored left/right.
      ...absolutePoint,
      wristPitch: wristPitchTarget ?? preferredWrist ?? currentTarget.wristPitch + wristDelta,
    }
    : {
      x: currentTarget.x - (left ?? 0),
      y: currentTarget.y + (farther ?? 0),
      z: currentTarget.z + (up ?? 0),
      wristPitch: wristPitchTarget ?? preferredWrist ?? currentTarget.wristPitch + wristDelta,
    };
  const requestedTarget = constrainTargetPose(rawRequestedTarget, currentTarget);

  const effectiveLeft = currentTarget.x - requestedTarget.x;
  const effectiveFarther = requestedTarget.y - currentTarget.y;
  const effectiveUp = requestedTarget.z - currentTarget.z;
  const effectiveWristDelta = requestedTarget.wristPitch - currentTarget.wristPitch;
  const constraintIssues: string[] = [];
  const requestedLeft = currentTarget.x - rawRequestedTarget.x;
  const requestedFarther = rawRequestedTarget.y - currentTarget.y;
  const requestedUp = rawRequestedTarget.z - currentTarget.z;
  if (Math.abs(requestedLeft - effectiveLeft) >= 0.1) {
    constraintIssues.push(`Requested X delta ${formatAiPromptNumber(requestedLeft)} cm was clamped to ${formatAiPromptNumber(effectiveLeft)} cm.`);
  }
  if (Math.abs(requestedFarther - effectiveFarther) >= 0.1) {
    constraintIssues.push(`Requested Y delta ${formatAiPromptNumber(requestedFarther)} cm was clamped to ${formatAiPromptNumber(effectiveFarther)} cm.`);
  }
  if (Math.abs(requestedUp - effectiveUp) >= 0.1) {
    constraintIssues.push(`Requested Z delta ${formatAiPromptNumber(requestedUp)} cm was clamped to ${formatAiPromptNumber(effectiveUp)} cm.`);
  }
  const hasEffectiveMotion =
    Math.abs(effectiveLeft) >= 0.01
    || Math.abs(effectiveFarther) >= 0.01
    || Math.abs(effectiveUp) >= 0.01
    || Math.abs(effectiveWristDelta) >= 0.1;
  // A chained gripper action. Restated where the gripper already is, it is
  // dropped like a restated slider; without a tip move it is a gripper step.
  // Chained, an open asked past 0.8 goes to 0.8 (ChainedFraction; the card and
  // the check-in say so), and is dropped when the gripper is already that open.
  let chain: ChainGripper | null = null;
  if (chainRead && "Error" in chainRead) constraintIssues.push(chainRead.Error);
  else if (chainRead) {
    const Changes = (Raw: number) => ChangingServoTargets([{ name: "gripper" as ServoName, targetRaw: Raw }], { gripper: servos.gripper.targetRaw }).length > 0;
    const targetRaw = clampRaw(fractionToRaw(servos.gripper, chainRead.Fraction));
    const Fraction = ChainedFraction(chainRead.Fraction), ChainRaw = clampRaw(fractionToRaw(servos.gripper, Fraction));
    const Capped = Fraction < chainRead.Fraction;
    const Commanded = rawToFraction(servos.gripper, servos.gripper.targetRaw);
    const Opens = !Capped || ChainGripperKind(Commanded, Fraction) === "open";
    // An opening gripper_before may run as one motion with the move (lib/AiChain.ts TOGETHER).
    const Together = ChainRunsTogether(chainRead.Order, Commanded, Fraction);
    if (Changes(targetRaw) && hasEffectiveMotion) {
      if (Opens && Changes(ChainRaw)) chain = { Order: chainRead.Order, TargetRaw: ChainRaw, Fraction, ...(Capped ? { Asked: chainRead.Fraction } : {}),
        ...(Together ? { Together } : {}) };
    } else if (Changes(targetRaw) && !servoTargets.some((proposal) => proposal.name === "gripper")) servoTargets.push({ name: "gripper", targetRaw });
  }
  const hasServoMotion = servoTargets.some(
    (proposal) => servos[proposal.name].targetRaw !== proposal.targetRaw,
  );

  if (!hasEffectiveMotion && !hasServoMotion && !chain) {
    return null;
  }

  return {
    target: requestedTarget,
    requestedTarget: rawRequestedTarget,
    reply: typeof parsed.response_to_user === "string"
      ? parsed.response_to_user
      : typeof parsed.message === "string"
        ? parsed.message
        : rawResponse,
    rawResponse,
    hasTargetMotion: hasEffectiveMotion,
    constraintIssues,
    requestedRelativeInches: {
      left: requestedLeft,
      farther: requestedFarther,
      up: requestedUp,
    },
    requestedWristPitchTarget: rawRequestedTarget.wristPitch,
    wristPitchTarget: requestedTarget.wristPitch,
    wristPitchDelta: effectiveWristDelta,
    relativeInches: {
      left: effectiveLeft,
      farther: effectiveFarther,
      up: effectiveUp,
    },
    servoTargets,
    chain,
    anchor: currentTarget,
    relative: absoluteDisplayX == null || absoluteY == null || absoluteZ == null,
    absoluteXY: absoluteDisplayX != null && absoluteY != null,
  };
}

// Read the AI's "where is the target" estimate. Accepts {left,farther,up} (the
// frame the AI is shown the current position in) or {x,y,z} aliases, and returns
// a display-frame Point3 or null.
function readTargetEstimate(parsed: unknown): Point3 | null {
  if (!isRecord(parsed)) return null;
  const raw = parsed.target_estimate_cm ?? parsed.target_estimate ?? parsed.target_position_cm;
  if (!isRecord(raw)) return null;
  const left = asOptionalNumber(raw.left ?? raw.x);
  const farther = asOptionalNumber(raw.farther ?? raw.forward ?? raw.y);
  const up = asOptionalNumber(raw.up ?? raw.z);
  if (left == null && farther == null && up == null) return null;
  return { x: left ?? 0, y: farther ?? 0, z: up ?? 0 };
}

// Read the TASK MODE verdict, tolerating common synonyms the model may emit.
function readTaskStatus(parsed: unknown): AiTaskStatus | null {
  if (!isRecord(parsed)) return null;
  const raw = parsed.task_status ?? parsed.taskStatus;
  if (typeof raw !== "string") return null;
  const value = raw.trim().toLowerCase();
  if (value === "continue" || value === "continuing" || value === "in_progress") return "continue";
  if (value === "done" || value === "complete" || value === "completed" || value === "finished" || value === "stop") return "done";
  if (value === "ask" || value === "question" || value === "need_help" || value === "blocked") return "ask";
  return null;
}

function parseAiResponse(
  text: string,
  currentTarget: TargetPose,
  servos: Record<ServoName, ServoState>,
): AiParsedResponse {
  const parsed = extractJsonObject(text);
  if (!isRecord(parsed)) {
    return {
      userMessage: text,
      motion: null,
      rawResponse: text,
      targetEstimate: null,
      taskStatus: null,
    };
  }

  const userMessage =
    typeof parsed.response_to_user === "string"
      ? parsed.response_to_user
      : typeof parsed.message === "string"
        ? parsed.message
        : text;

  const motion = parsed.motion_proposal === null || ["done", "ask"].includes(readTaskStatus(parsed) ?? "")
    ? null
    : parseAiMotionProposalFromObject(parsed, currentTarget, text, servos);

  return {
    userMessage,
    motion,
    rawResponse: text,
    targetEstimate: readTargetEstimate(parsed),
    taskStatus: readTaskStatus(parsed),
  };
}

function valueToCm(value: number, unit?: string) {
  const normalized = unit?.toLowerCase() ?? "";
  if (normalized === "in" || normalized.startsWith("inch")) return value * 2.54;
  return value;
}

function escapeRegex(value: string) {
  return value.replace(/[.*+?^${}()|[\]\\]/g, "\\$&");
}

function readDirectionalDeltaFromText(
  text: string,
  positiveTerms: string[],
  negativeTerms: string[],
) {
  let total = 0;
  let found = false;
  const terms = [
    ...positiveTerms.map((term) => ({ term, sign: 1 })),
    ...negativeTerms.map((term) => ({ term, sign: -1 })),
  ];

  for (const { term, sign } of terms) {
    const word = escapeRegex(term);
    const after = new RegExp(
      `\\b${word}\\b(?:\\s+(?:by|about|around|approximately|approx\\.?))?\\s+([-+]?\\d*\\.?\\d+)\\s*(cm|centimeters?|in|inches?)?`,
      "gi",
    );
    const before = new RegExp(
      `([-+]?\\d*\\.?\\d+)\\s*(cm|centimeters?|in|inches?)?\\s+(?:to\\s+the\\s+)?\\b${word}\\b`,
      "gi",
    );
    for (const pattern of [after, before]) {
      for (const match of text.matchAll(pattern)) {
        const amount = Number(match[1]);
        if (!Number.isFinite(amount)) continue;
        total += sign * valueToCm(amount, match[2]);
        found = true;
      }
    }
  }

  return found ? total : null;
}

function readAxisTargetFromText(text: string, axis: "x" | "y" | "z") {
  const pattern = new RegExp(`\\b${axis}\\s*(?:=|:|to)?\\s*([-+]?\\d*\\.?\\d+)\\s*(cm|centimeters?|in|inches?)?`, "i");
  const match = text.match(pattern);
  if (!match) return null;
  const value = Number(match[1]);
  return Number.isFinite(value) ? valueToCm(value, match[2]) : null;
}

function parsePlainTextMotionFallback(
  sourceText: string,
  currentTarget: TargetPose,
  rawResponse: string,
  servos: Record<ServoName, ServoState>,
) {
  const text = sourceText.replace(/[_-]/g, " ").toLowerCase();
  const left = readDirectionalDeltaFromText(text, ["left"], ["right"]);
  const farther = readDirectionalDeltaFromText(
    text,
    ["farther", "forward", "away"],
    ["closer", "back", "backward", "toward"],
  );
  const up = readDirectionalDeltaFromText(text, ["up", "higher", "raise", "raised"], ["down", "lower", "lowered"]);
  const x = readAxisTargetFromText(text, "x");
  const y = readAxisTargetFromText(text, "y");
  const z = readAxisTargetFromText(text, "z");
  const wristMatch = text.match(/\bwrist(?:\s+pitch)?\s*(?:=|:|to)?\s*([-+]?\d*\.?\d+)\s*(?:deg|degree|degrees)?/i);
  const wristPitch = wristMatch ? Number(wristMatch[1]) : null;
  const direct_servo_targets: Array<{ name: ServoName; target_fraction: number }> = [];

  if (/\b(open|release)\b[^.]{0,32}\bgripper\b|\bgripper\b[^.]{0,32}\b(open|release)\b/.test(text)) {
    direct_servo_targets.push({ name: "gripper", target_fraction: 1 });
  }
  if (/\b(close|closed|grab|grip|clamp)\b[^.]{0,32}\bgripper\b|\bgripper\b[^.]{0,32}\b(close|closed|grab|grip|clamp)\b/.test(text)) {
    direct_servo_targets.push({ name: "gripper", target_fraction: 0 });
  }

  const hasRelativeMove = left != null || farther != null || up != null;
  const hasAbsoluteTarget = x != null || y != null || z != null;
  const hasWrist = wristPitch != null && Number.isFinite(wristPitch);
  if (!hasRelativeMove && !hasAbsoluteTarget && !hasWrist && !direct_servo_targets.length) {
    return null;
  }

  return parseAiMotionProposalFromObject(
    {
      response_to_user: rawResponse,
      motion_proposal: {
        ...(hasRelativeMove
          ? { move_delta_cm: { left: left ?? 0, farther: farther ?? 0, up: up ?? 0 } }
          : {}),
        ...(hasAbsoluteTarget
          ? {
              // target_cm is interpreted in the AI's display frame (X positive =
              // left), so missing axes must be filled from that same frame.
              target_cm: {
                x: x ?? mirrorDisplayPoint(currentTarget).x,
                y: y ?? mirrorDisplayPoint(currentTarget).y,
                z: z ?? mirrorDisplayPoint(currentTarget).z,
              },
            }
          : {}),
        ...(hasWrist ? { wrist_pitch_degrees: wristPitch } : {}),
        ...(direct_servo_targets.length ? { direct_servo_targets } : {}),
      },
    },
    currentTarget,
    rawResponse,
    servos,
  );
}

function addFallbackMotionFromText(
  response: AiParsedResponse,
  userPrompt: string,
  modelText: string,
  currentTarget: TargetPose,
  servos: Record<ServoName, ServoState>,
): AiParsedResponse {
  // Only explicit structured proposals can request motion. Prose, negations,
  // and an explicit null/done/ask must never become motor commands.
  void userPrompt; void modelText; void currentTarget; void servos;
  return response.taskStatus === "done" || response.taskStatus === "ask"
    ? { ...response, motion: null } : response;
}

// unreliable: this frame's [UNRELIABLE] measurements and the task object's
// last one. The descent rule used to guard only a matched task object, so a
// task noun that matched nothing (16:10, "thingon") switched it off.
// limits: the joint limits the wrist search and the send use (bridgeIkLimits).
// guard: the task's measured objects, GRASP PLAN or refusal (lib/GraspPlan.ts).
function validateAiMotionProposal(
  motion: AiMotionProposal,
  elbowMode: "up" | "down",
  servos: Record<ServoName, ServoState>,
  unreliable: LocatedObject[] = [],
  limits?: IkAngleLimits,
  grasp: { Object: LocatedObject; State: GraspState; Plan?: GraspPlan | null } | null = null,
  guard: GraspGuard | null = null,
  // The pose the move really starts from, when the IK is seeded from another
  // reading: a chain's second leg is solved as it was approved (servos: its
  // seed) and its path checked from the live joints.
  pathFrom: Record<ServoName, ServoState> | null = null,
): AiProposalValidation {
  const issues = [...motion.constraintIssues];
  if (!Object.values(motion.requestedTarget).every(Number.isFinite)
      || !Object.values(motion.target).every(Number.isFinite)) {
    issues.push("Target coordinates must be finite numbers.");
    return { possible: false, issues, preview: null };
  }
  // The bridge runs one move at a time and refuses a second while the first
  // runs, so only one thing may change. A servo restated where it already is
  // (the gripper left open during an approach) is not sent and not counted.
  const Changing = ChangingServoTargets(motion.servoTargets,
    Object.fromEntries(SERVO_ORDER.map(Name => [Name, servos[Name].targetRaw])));
  if (motion.hasTargetMotion && Changing.length) {
    issues.push("Send Cartesian motion and direct joint/gripper motion as separate approved steps, or chain the gripper with the tip move through gripper_before or gripper_after (not direct_servo_targets).");
  } else if (Changing.length > 1) {
    issues.push(`Change one slider per motion (the arm runs one move at a time): ${Changing.map(Item => formatServoLabel(Item.name)).join(" and ")} were proposed together.`);
  }
  // The GRASP PLAN, the guard's gripper pose and the bridge's tape check all assume the roll stays, except the GRASP PLAN's
  // own turn (lib/GraspPlan.ts FREE WRIST ROLL), checked on its way round like any move (WristRollTurnReason).
  const RollMove = guard ? Changing.find(Item => Item.name === "wrist_roll") : undefined;
  if (RollMove) {
    const Turned = WristRollTurnReason(currentAngles(servos), RollMove.targetRaw, angleFromRawValue(servos.wrist_roll, RollMove.targetRaw), guard);
    if (Turned) issues.push(Turned);
  }
  let preview: IkPreview | null = null;
  // A CHAINED MOTION (lib/AiChain.ts): its tip move is checked with the jaws as
  // its gripper action leaves them (a gripper_before runs first), and the
  // gripper action where the arm is when it runs.
  const Chain = motion.hasTargetMotion ? motion.chain ?? null : null;
  const TipServos = chainTipServos(motion, servos), PathAngles = currentAngles(pathFrom ?? TipServos);
  const CommandedFraction = rawToFraction(servos.gripper, servos.gripper.targetRaw);
  const TipGrasp = Chain?.Order === "before" && grasp
    ? { ...grasp, State: { ...grasp.State, GripperFraction: Chain.Fraction, Closed: GripperClosedAfter(CommandedFraction, Chain.Fraction, grasp.State.Closed) } }
    : grasp;

  if (motion.hasTargetMotion) {
    preview = solveIkPreview(motion.target, elbowMode, TipServos, limits, true);
    if (preview.collisionReason) issues.push(preview.collisionReason);
    // The task's floor (its guard's: the surface the task object stands on), the desk's without one.
    const Floor = guard?.Floor ?? DeskFloor;
    const floorReason = AiTableFloorReason(motion.target, motion.relativeInches.up, Floor);
    if (floorReason) issues.push(floorReason);
    const closedJawReason = ClosedJawDescentReason(motion.target, motion.relativeInches.up, TipGrasp?.Object ?? null, TipGrasp?.State ?? null, TipGrasp?.Plan ?? null);
    if (closedJawReason) issues.push(closedJawReason);
    // The floor holds the jaw too, which a flat wrist hangs up to 1.3 cm under the tip.
    const jawReason = preview.reachable ? AiJawFloorReason(motion.target, preview.angles, PathAngles, Floor) : null;
    if (jawReason) issues.push(jawReason);
    if (!preview.reachable && !preview.collisionReason) issues.push(ikFailureIssue(motion.target, preview));
    // A refused snap refuses the proposal, even when its target passes the
    // normal tolerance: the snap found it only at the edge of the IK's
    // tolerance (16:10's -55.9 deg), or found no target near it within its caps.
    if (motion.snap?.Kind === "refused" && !issues.includes(motion.snap.Text)) issues.push(motion.snap.Text);
    const Unreliable = unreliable.map(Object_ => UnreliableTargetReason(motion.requestedTarget, motion.absoluteXY, Object_)
      ?? UnreliableTargetReason(motion.target, motion.absoluteXY, Object_)).find(Boolean);
    // A snap refused "unreliable" already gave this text.
    if (Unreliable && !issues.includes(Unreliable)) issues.push(Unreliable);
    // Every gripper part (the fixed finger above all) clear of the measured
    // objects on the way there, and a grasp held to its plan (lib/GraspPlan.ts).
    const Guarded = preview.reachable ? GraspGuardReason(motion.target, preview.angles, PathAngles, motion.relativeInches.up, guard) : null;
    if (Guarded && !issues.includes(Guarded)) issues.push(Guarded);
    // An opening gripper_before may run as one motion (lib/AiChain.ts TOGETHER): the
    // moving jaw at every opening on the way, not only open, clear of measured objects.
    const OnTheWay = preview.reachable && !Guarded && Chain?.Together
      ? OpeningOnTheWayReason(preview.angles, PathAngles, currentAngles(servos).gripper ?? 0, angleFromRawValue(servos.gripper, Chain.TargetRaw),
        motion.relativeInches.up, guard)
      : null;
    if (OnTheWay) issues.push(OnTheWay);
  }

  for (const proposal of motion.servoTargets) {
    const servo = servos[proposal.name];
    const fraction = rawToFraction(servo, proposal.targetRaw);
    if (fraction < -0.001 || fraction > 1.001) {
      issues.push(`${formatServoLabel(proposal.name)} raw target ${proposal.targetRaw} is outside the configured slider range.`);
    }
  }
  // The moving jaw swings wide as it opens: not into a measured neighbour.
  const GripperMove = Changing.find(Item => Item.name === "gripper");
  const Swept = GripperMove ? GripperSweepReason(currentAngles(servos), angleFromRawValue(servos.gripper, GripperMove.targetRaw), guard) : null;
  if (Swept) issues.push(Swept);
  if (Chain) {
    // The chained gripper action swings the moving jaw where the arm is when it
    // runs: here (gripper_before), or at the end of the tip move (gripper_after).
    const JawFrom = Chain.Order === "before" ? currentAngles(servos)
      : preview?.reachable ? { ...preview.angles, gripper: currentAngles(servos).gripper } : null;
    const ChainSwept = JawFrom ? GripperSweepReason(JawFrom, angleFromRawValue(servos.gripper, Chain.TargetRaw), guard) : null;
    if (ChainSwept && !issues.includes(ChainSwept)) issues.push(ChainSwept);
    // Which two steps may follow each other without a model call (the descent never, a close only before a straight lift).
    const ChainRule = ChainRefusalReason({ Order: Chain.Order, FromFraction: CommandedFraction, ToFraction: Chain.Fraction,
      Start: mirrorDisplayPoint(motion.anchor), End: mirrorDisplayPoint(motion.target), Requested: mirrorDisplayPoint(motion.requestedTarget),
      WristTurnDeg: Math.abs(motion.requestedTarget.wristPitch - motion.anchor.wristPitch),
      // Held, it is put down and released like any move (as GraspGuardReason skips the plan's rules).
      Task: guard?.TaskPrism && !guard.Held ? { Prism: guard.TaskPrism, Label: guard.TaskLabel } : null });
    if (ChainRule) issues.push(ChainRule);
  }

  return {
    possible: issues.length === 0,
    issues,
    preview,
  };
}

// Why the IK cannot reach a target, from the solve's explanation: out of the
// arm's reach at this wrist, or a named joint limit. It used to say "joint
// slider limits would be exceeded" for every unreachable pose, and the model
// answered "blocked by joint limits" far from any stop.
function ikFailureIssue(target: TargetPose, preview: IkPreview): string {
  const failure = preview.failure;
  const wrist = `At wrist ${formatAiPromptNumber(Number(target.wristPitch.toFixed(2)))} deg`;
  const cm = (value: number) => formatAiPromptNumber(Number(value.toFixed(2)));
  if (failure?.Kind === "reach") {
    return failure.TooFar
      ? `${wrist} the arm is ${cm(failure.ErrorCm)} cm too short to reach that point.`
      : `${wrist} that point is too close to the shoulder: the nearest pose misses it by ${cm(failure.ErrorCm)} cm.`;
  }
  if (failure?.Kind === "joint-limit") {
    return `${wrist} the joint slider limits block that point: ${failure.Joints.map(({ Name, Angle, Range }) =>
      `${Name.replace(/_/g, " ")} would need ${Angle.toFixed(1)} deg, its range is ${Range[0].toFixed(1)}..${Range[1].toFixed(1)} deg`).join("; ")}.`;
  }
  if (failure?.Kind === "collision") {
    return `${wrist} no pose within the joint slider limits reaches that point (beyond them: ${failure.Reason}).`;
  }
  return "The local IK solver marked this arm pose unreachable.";
}

// The snap's input for a proposal (model frame): what the model asked for,
// the pose it planned from, and the request's one IK context (servo
// snapshot, elbow branch, the joint limits the app sends with). A chain's
// tip move is snapped with the jaws as its gripper_before leaves them.
function aiSnapInput(
  motion: AiMotionProposal,
  servos: Record<ServoName, ServoState>,
  elbowMode: "up" | "down",
  limits: IkAngleLimits | undefined,
  unreliable: LocatedObject[],
): SnapInput {
  return { Requested: motion.requestedTarget, Anchor: motion.anchor, Servos: chainTipServos(motion, servos), ElbowMode: elbowMode, Limits: limits,
    AbsoluteXY: motion.absoluteXY, Unreliable: unreliable, Grasp: motion.guard ?? null };
}

function extractNotepadUpdate(content: string): string | null {
  const parsed = extractJsonObject(content);
  if (!isRecord(parsed) || typeof parsed.notepad_update !== "string") return null;
  const update = parsed.notepad_update.trim();
  return update || null;
}

async function saveNotepad(content: string): Promise<void> {
  try {
    await fetch(`${API_URL}/notepad`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ content }),
    });
  } catch {
    // best-effort
  }
}

// One model call through Ollama's native /api/chat, with the options every
// caller shares (lib/OllamaChat.ts: num_ctx, num_batch, keep_alive from back
// end/OllamaOptions.json, thinking off). maxTokens is capped at AiMaxOutputTokens,
// and a reply cut off by it is an error (ReadOllamaChatReply).
async function requestAiCompletion(messages: AiApiMessage[], model: string, maxTokens: number,
  OnUsage?: (Usage: AiUsage) => void, Signal?: AbortSignal) {
  const StartedAt = performance.now();
  const response = await fetch(OllamaChatUrl, {
    method: "POST",
    signal: Signal,
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(BuildOllamaChatBody(messages, model, maxTokens)),
  });

  if (!response.ok) {
    throw new Error(await response.text());
  }

  const data = await response.json() as unknown;
  if (isRecord(data)) OnUsage?.(ReadAiUsage(data, performance.now() - StartedAt, model));
  return ReadOllamaChatReply(data, maxTokens);
}

// The warm-up (BuildOllamaPreloadBody): loads the model with the same options as
// every call, so the first task does not wait 6-19 s for a cold load. Best
// effort; true when Ollama answered.
async function preloadAiModel(model: string): Promise<boolean> {
  try {
    const response = await fetch(OllamaChatUrl, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(BuildOllamaPreloadBody(model)),
    });
    return response.ok;
  } catch {
    return false;
  }
}

function buildAiUserPrompt(
  userText: string,
  currentPosition: { x: number; y: number; z: number; wristPitch: number },
  servos: Record<ServoName, ServoState>,
  visionSummary?: string,
) {
  const display = currentDisplayPointCm(currentPosition);
  const sliderLines = SERVO_ORDER.map((name) => {
    const servo = servos[name];
    return `${name}: present_raw ${servo.presentRaw}, present_fraction ${formatAiPromptNumber(rawToFraction(servo, servo.presentRaw))}`;
  });
  return [
    userText,
    "",
    "Current arm position in centimeters (from actual servo positions):",
    `X left/right: ${formatAiPromptNumber(display.x)} where positive means left`,
    `Y farther/closer: ${formatAiPromptNumber(display.y)} where positive means farther`,
    `Z up/down: ${formatAiPromptNumber(display.z)} where positive means up`,
    `Wrist pitch: ${formatAiPromptNumber(currentPosition.wristPitch)} degrees`,
    "",
    "Current servo positions:",
    ...sliderLines,
    "Gripper slider convention: present_fraction 1.0 is open and 0.0 is closed.",
    ...(visionSummary ? ["", visionSummary] : []),
    "Return JSON only. Include response_to_user. Set motion_proposal to null if you are only talking or need clarification.",
  ].join("\n");
}

function compactOutputText(text: string, maxLength = 150) {
  const compact = text.replace(/\s+/g, " ").trim();
  if (compact.length <= maxLength) return compact;
  return `${compact.slice(0, maxLength - 3)}...`;
}


function pickFirstModel(modelIds: string[]): string | null {
  return modelIds.find((id) => id.trim().length > 0) ?? null;
}

type ModelEntry = {
  id: string;
  type: string | null;
  state: string | null;
  contextLength: number | null;
};

function readNativeModelEntries(value: unknown): ModelEntry[] {
  // Ollama's /api/ps returns { models: [...] }; tolerate OpenAI-style { data: [...] }
  // and bare arrays too.
  const items = isRecord(value) && Array.isArray(value.models)
    ? value.models
    : isRecord(value) && Array.isArray(value.data)
      ? value.data
      : Array.isArray(value)
        ? value
        : [];
  return items.flatMap((item) => {
    if (!isRecord(item)) return [];
    const id = item.model ?? item.name ?? item.id;
    if (typeof id !== "string" || !id.trim()) return [];
    return [{
      id,
      type: typeof item.type === "string" ? item.type : null,
      state: typeof item.state === "string" ? item.state : null,
      contextLength: typeof item.context_length === "number" ? item.context_length : null,
    }];
  });
}

// Pick a model Ollama already has loaded (reported by /api/ps) so we never trigger
// a fresh load of a different model. Entries listed by /api/ps carry no state field,
// so a null state means "already resident" and is preferred just like an explicit
// "loaded".
function pickLoadedModel(entries: ModelEntry[]): ModelEntry | null {
  const usable = entries.filter((entry) => entry.type !== "embeddings");
  return (
    usable.find((entry) => entry.state === "loaded")
    ?? usable.find((entry) => entry.state === null)
    ?? usable.find((entry) => entry.state === "loading")
    ?? null
  );
}

// The context window a resident model was loaded with (/api/ps), for the
// statistics panel; null when it is not loaded or Ollama does not say.
async function loadedContextLengthFor(id: string): Promise<number | null> {
  try {
    const res = await fetch(OLLAMA_LOADED_MODELS_URL);
    if (!res.ok) return null;
    const entries = readNativeModelEntries(await res.json());
    return entries.find((entry) => entry.id === id)?.contextLength ?? null;
  } catch {
    return null;
  }
}

function readModelIds(value: unknown): string[] {
  const items = isRecord(value) && Array.isArray(value.data)
    ? value.data
    : Array.isArray(value)
      ? value
      : [];
  const ids = items.flatMap((item) => {
    if (typeof item === "string") return [item];
    if (!isRecord(item)) return [];
    const id = item.id ?? item.model ?? item.name;
    return typeof id === "string" && id.trim() ? [id] : [];
  });
  return [...new Set(ids)];
}

type InstalledModel = { id: string; parameterSize: number; byteSize: number };

// Parse Ollama's parameter_size strings ("36.0B", "26.9B", "7B", "560M") into a
// comparable number of billions of parameters. Returns 0 when unknown.
function parseParameterSize(value: unknown): number {
  if (typeof value !== "string") return 0;
  const match = value.trim().match(/([\d.]+)\s*([a-zA-Z]?)/);
  if (!match) return 0;
  const amount = Number(match[1]);
  if (!Number.isFinite(amount)) return 0;
  switch (match[2].toUpperCase()) {
    case "M": return amount / 1_000;
    case "K": return amount / 1_000_000;
    default: return amount; // "B" or unitless — already billions
  }
}

// Read /api/tags into a list of installed models with size info, sorted largest
// first (by parameter count, then on-disk bytes as a tiebreak).
function readInstalledModels(value: unknown): InstalledModel[] {
  const items = isRecord(value) && Array.isArray(value.models) ? value.models : [];
  const models = items.flatMap((item): InstalledModel[] => {
    if (!isRecord(item)) return [];
    const id = item.model ?? item.name;
    if (typeof id !== "string" || !id.trim()) return [];
    const details = isRecord(item.details) ? item.details : {};
    return [{
      id,
      parameterSize: parseParameterSize(details.parameter_size),
      byteSize: typeof item.size === "number" ? item.size : 0,
    }];
  });
  return models.sort(
    (a, b) => (b.parameterSize - a.parameterSize) || (b.byteSize - a.byteSize),
  );
}

// True when a model's /api/show capabilities include image input.
function modelHasVision(capabilities: unknown): boolean {
  return Array.isArray(capabilities)
    && capabilities.some((c) => typeof c === "string" && c.toLowerCase() === "vision");
}

/* The CAD view (ArmCanvas) is memoised: App re-renders on every keystroke in the AI box, and a new prop each time
   made the view redraw its whole scene per key (TestCadTypingUi). Its callbacks therefore keep one identity and
   call the latest closure, and its empty wrist-angle guide is one shared array. */
function UseLatestCallback<Args extends unknown[], Result>(Callback: (...Values: Args) => Result) {
  const Latest = useRef(Callback); Latest.current = Callback;
  return useCallback((...Values: Args) => Latest.current(...Values), []);
}
const NoPitchGuide: PitchGuideSegment[] = [];

function App() {
  const [DisplaySettings, SetDisplaySettings] = useState(ReadViewSettings);
  const [SettingsOpen, SetSettingsOpen] = useState(false);
  const [SelectedJoint, SetSelectedJoint] = useState<ServoName | null>(null);
  const [SelectedAxis, SetSelectedAxis] = useState<"x" | "y" | "z" | null>(null);
  const SelectJoint = (Name: ServoName | null) => { SetSelectedJoint(Name); SetSelectedAxis(null); };
  useEffect(() => { SaveViewSettings(DisplaySettings); }, [DisplaySettings]);
  useEffect(() => {
    let Cancelled = false;
    // Optional local operator preference, separate from distributable defaults.
    // Apply once; later settings changes and Reset remain the user's choice.
    void fetch("/LocalDisplayPreferences.json", { cache: "no-store" }).then(Response => Response.ok ? Response.json() : null).then(Value => {
      if (Cancelled || typeof Value?.Revision !== "string" || typeof Value?.ShowBottomText !== "boolean") return;
      try { if (localStorage.getItem("SO101.LocalDisplayPreference") === Value.Revision) return; } catch { /* Session-only settings still work. */ }
      SetDisplaySettings(Previous => ({ ...Previous, ShowBottomText: Value.ShowBottomText }));
      try { localStorage.setItem("SO101.LocalDisplayPreference", Value.Revision); } catch { /* Storage is optional. */ }
    }).catch(() => undefined);
    return () => { Cancelled = true; };
  }, []);
  const ChangeDisplaySettings = (Patch: Partial<ViewSettings>) => SetDisplaySettings(Previous => NormalizeViewSettings({ ...Previous, ...Patch }));
  // The grasp gap at the fixed finger (lib/GraspGap.ts): saved in this browser; a task plans with the one set when it was sent.
  const [GraspGapCm, SetGraspGapCm] = useState(ReadGraspGap);
  const GraspGapRef = useRef(GraspGapCm);
  GraspGapRef.current = GraspGapCm;
  const TaskGraspGapRef = useRef(GraspGapCm);
  const ChangeGraspGap = (Cm: number) => {
    const Gap = NormalizeGraspGap(Cm);
    SetGraspGapCm(Gap);
    SaveGraspGap(Gap);
  };
  const ChangeDepthOverlay = useCallback((Value: boolean) => SetDisplaySettings(Previous => NormalizeViewSettings({ ...Previous, DepthOverlay: Value })), []);
  const [ControlMode, SetControlMode] = useState(ReadDashboardMode);
  const ControlModeRef = useRef(ControlMode);
  const [AiStats, SetAiStats] = useState(EmptyAiStatistics);
  const AiRequestAbortRef = useRef<AbortController | null>(null);
  const socketRef = useRef<WebSocket | null>(null);
  const videoRef = useRef<HTMLVideoElement | null>(null);
  const armCanvasRef = useRef<ArmCanvasHandle | null>(null);
  const emergencyDisconnectRef = useRef(false);
  // Tracks which serial port the arm has already been auto-connected on for the
  // current bridge session, so the auto-connect effect attempts each port once
  // instead of spamming connect commands. Cleared whenever the socket drops.
  const autoArmRef = useRef<string | null>(null);
  const ManualDisconnectRef = useRef<boolean | null>(null);
  if (ManualDisconnectRef.current === null) {
    try { ManualDisconnectRef.current = localStorage.getItem("SO101.ManualDisconnect") === "true"; }
    catch { ManualDisconnectRef.current = false; }
  }
  // When Connect was last clicked. Without a click the arm never connects from another device, nor after anyone
  // disconnected it on purpose; the bridge refuses such a connect too (2026-10-07 audit 2 M2).
  const ConnectClickedAtRef = useRef<number | null>(null);
  // Another device: why the robot's PC refused this page (null once it let it in), whether the password card is open,
  // and whether the user put it away ("Not now"), so a refused camera poll does not bring it back every second.
  const [RemoteRefusal, SetRemoteRefusal] = useState<AccessProblem | null>(null);
  const [AccessCardOpen, SetAccessCardOpen] = useState(false);
  const AccessCardDismissedRef = useRef(false);
  const pendingServoTargetsRef = useRef<Partial<Record<ServoName, { raw: number; updatedAt: number; scene?: SceneIdentity | null; carry?: boolean }>>>({});
  const lastServoCommandAtRef = useRef<Partial<Record<ServoName, number>>>({});
  const servoCommandTimersRef = useRef<Partial<Record<ServoName, number>>>({});
  const previewRequestIdRef = useRef(0);
  const previewAppliedIdRef = useRef(0);
  const previewGenerationRef = useRef(0);
  const previewLastStartedAtRef = useRef(0);
  // Coalescing: at most one preview request is in flight. While one is out, the
  // newest target is parked here and requested as soon as the reply lands, so
  // the bridge always solves the latest drag position instead of a backlog.
  const previewInFlightRef = useRef(false);
  const previewLatestRef = useRef<{ target: TargetPose; elbowMode: "up" | "down"; key: string } | null>(null);
  const [socketState, setSocketState] = useState<SocketState>("offline");
  const [servos, setServos] = useState(() => cloneServos(INITIAL_SERVOS));
  const [CadRevision, SetCadRevision] = useState(() => GetCadAlignment().Version);
  const CadCompatibleRef = useRef(false);
  const BasketCompatibleRef = useRef(false);
  const CommandFailureRef = useRef<string | null>(null);
  const [BridgeUpdate, SetBridgeUpdate] = useState<string | null>(null);
  const [BridgeRestarting, SetBridgeRestarting] = useState(false);
  const BridgeRestartingRef = useRef(false);
  const [MotionSpeedPercent, SetMotionSpeedPercent] = useState(30);
  const [SpeedCompatible, SetSpeedCompatible] = useState(false);
  const SpeedCompatibleRef = useRef(false);
  // The running bridge's code version (state bridge_version; null from a bridge older than the version check):
  // BridgeUpdateReason names a bridge that is not the code this page expects, and AI tasks wait for it.
  const BridgeVersionRef = useRef<string | null>(null);
  // The bridge takes a gripper target with a tip move (gripper_with_tip_version): an opening
  // gripper_before then runs as one motion (lib/AiChain.ts TOGETHER); an older bridge runs two legs.
  const [GripperWithTip, SetGripperWithTip] = useState(false);
  const GripperWithTipRef = useRef(false);
  // The gripper's present raw from each bridge state (Date.now() ms), the last few seconds:
  // a chain's tip move waits until a gripper leg's jaws hold still (ChainGripperStill).
  const GripperSamplesRef = useRef<{ At: number; Raw: number }[]>([]);
  const ServerMotionSpeedRef = useRef(30);
  const PendingMotionSpeedRef = useRef<number | null>(null);
  const SpeedSendTimerRef = useRef<number | undefined>(undefined);
  const SpeedAckTimerRef = useRef<number | undefined>(undefined);
  const LastSpeedSentAtRef = useRef(-Infinity);
  const [bridgeJoints, setBridgeJoints] = useState<Point3[] | null>(null);
  const [bridgeIkPreview, setBridgeIkPreview] = useState<SyncedIkPreview | null>(null);
  // Joint limits in the IK model frame, straight from the bridge. Used by the
  // client-side solver (green wrist arc, wheel clamp, offline fallback) so its
  // idea of "inside the limits" is the bridge's, not the servo-slider frame.
  const [bridgeIkLimits, setBridgeIkLimits] = useState<IkAngleLimits | null>(null);
  const bridgeIkLimitsKeyRef = useRef("");
  const [previewRefreshNonce, setPreviewRefreshNonce] = useState(0);
  const [torque, setTorque] = useState(false);
  const [armConnected, setArmConnected] = useState(false);
  // Hand guiding as the bridge reports it (state hand_guide, lib/HandGuide.ts); the ref for handlers. A session this page
  // or any other started: this page sends the heartbeat that keeps a session it started going (HeartbeatMs).
  const [HandGuideState, SetHandGuideState] = useState<HandGuideView>(EmptyHandGuide);
  const HandGuideRef = useRef<HandGuideView>(EmptyHandGuide);
  const HandGuideSeenRef = useRef(false);
  const HandGuideBeatAtRef = useRef(0);
  // The bridge refused a start because the calibration does not fit the arm any more (a servo or the zero pose changed):
  // the button offers Calibrate guiding until a calibration is saved (lib/HandGuide.ts NeedsCalibration).
  const [HandGuideStale, SetHandGuideStale] = useState(false);
  // The bridge's "disconnected on purpose" (state disconnected_on_purpose: who and when, as text) until an explicit
  // Connect clears it: this page then waits for a Connect click instead of connecting by itself.
  const [DisconnectedOnPurpose, SetDisconnectedOnPurpose] = useState<string | null>(null);
  const DisconnectedOnPurposeRef = useRef<string | null>(null);
  // The bridge's USB serial link to the servos (state.serial_link): down while it reopens the port.
  const [SerialLinkDown, SetSerialLinkDown] = useState(false);
  const SerialLinkDownRef = useRef(false);
  const LastSerialNoteRef = useRef<string | null>(null);
  const [RouteActive, SetRouteActive] = useState(false);
  const [ProtectionState, SetProtectionState] = useState({ Loaded: false, Reason: "", CoastConfirmed: false, Hottest: null as number | null,
    Temperatures: {} as Partial<Record<ServoName, number>>, LimitC: 60 });
  const LastProtectionSeen = useRef<number | null>(null);
  const [elbowMode, setElbowMode] = useState<"up" | "down">("up");
  const [status, setStatus] = useState("Local preview");
  const [ports, setPorts] = useState<string[]>([]);
  const [selectedPort, setSelectedPort] = useState("");
  const [target, setTarget] = useState<TargetPose>(HOME_MODEL_TARGET_CM);
  const [TargetEditing, SetTargetEditing] = useState(false);
  const [TargetSnapping, SetTargetSnapping] = useState(false);
  const TargetEditRef = useRef<{ Original: TargetPose; Candidate: TargetPose; Kind: "Position" | "Pitch" } | null>(null);
  const CurrentTargetRef = useRef(target); CurrentTargetRef.current = target;
  const TargetWorkerRef = useRef<Worker | null>(null);
  // Started when an edit begins: a cold module worker takes ~0.4 s to load, most of the snap limit.
  const WarmTargetWorkerRef = useRef<Worker | null>(null);
  const NewTargetWorker = () => new Worker(new URL("./lib/TargetWorkspaceWorker.ts", import.meta.url), { type: "module" });
  // The pitch guide of the target as drawn, when it has finished: lets the release search skip its sweep.
  const TargetPitchGuideRef = useRef<{ Target: TargetPose; Segments: PitchGuideSegment[] } | null>(null);
  const LastValidTargetRef = useRef<TargetPose | null>(null);
  const TargetContextRef = useRef<TargetWorkspaceContext>({ Servos: servos, ElbowMode: elbowMode });
  TargetContextRef.current = { Servos: servos, ElbowMode: elbowMode, Limits: bridgeIkLimits ?? undefined };
  useEffect(() => () => { TargetWorkerRef.current?.terminate(); WarmTargetWorkerRef.current?.terminate(); }, []);
  const [log, setLog] = useState<StatusEntry[]>([
    { id: 1, level: "info", text: "Standalone UI ready" },
    { id: 2, level: "warn", text: "Hardware commands require the FastAPI bridge" },
  ]);
  const [aiInput, setAiInput] = useState("");
  const [aiBusy, setAiBusy] = useState(false);
  const [aiModel, setAiModel] = useState<string | null>(null);
  const [aiContextLength, setAiContextLength] = useState<number | null>(null);
  const [aiNotepad, setAiNotepad] = useState("");
  const [aiStatus, setAiStatus] = useState("Loading models");
  const [AiRouteStatus, SetAiRouteStatus] = useState("Nimble picks what each prompt needs: prompt only, camera, or camera and depth");
  const PromptRouterRef = useRef(new AiPromptRouter());
  const [aiLastReply, setAiLastReply] = useState("");
  const [aiMessages, setAiMessages] = useState<AiChatMessage[]>([]);
  const [aiProposal, setAiProposal] = useState<AiMotionProposal | null>(null);
  // Mirror for the socket handler: whether anything is still approvable.
  const AiProposalRef = useRef(aiProposal);
  AiProposalRef.current = aiProposal;
  const [aiDisplayedProposal, setAiDisplayedProposal] = useState<AiMotionProposal | null>(null);
  const [aiDisplayedIssues, setAiDisplayedIssues] = useState<string[]>([]);
  // Where the AI thinks the target object is, in the display frame (left/farther/up,
  // same axes as the reported current position). Drawn as a small blue box.
  const [aiTargetEstimate, setAiTargetEstimate] = useState<Point3 | null>(null);
  // The task object as measured (its footprint and height), drawn in place of the estimate's cube.
  const [aiObjectBox, setAiObjectBox] = useState<ObjectBox | null>(null);
  // TASK MODE loop state. aiTask is the ongoing task (null = idle); the ref mirror
  // gives async handlers the latest value. aiContinueOnApproveRef marks that the
  // pending proposal asked to continue, and aiSettleWait drives the post-approval
  // wait for the arm to stop moving before the automatic re-prompt.
  const [aiTask, setAiTask] = useState<AiTaskState | null>(null);
  const aiTaskRef = useRef<AiTaskState | null>(null);
  const aiContinueOnApproveRef = useRef(false);
  const [aiSettleWait, setAiSettleWait] = useState<TaskSettleWait | null>(null);
  const AiSettleWaitRef = useRef(aiSettleWait);
  AiSettleWaitRef.current = aiSettleWait;
  // The approved motion being waited on: where the tip started and what was
  // commanded, so the next check-in states what it measurably did.
  // Chain: a chained motion's legs and how each went (lib/AiChain.ts), for that check-in.
  const LastTaskMotionRef = useRef<{ Step: number; StartTip: Point3; Target: Point3 | null; ServoTargets: AiServoProposal[];
    Snap: TaskMotionSnap | null; Record: Record<string, unknown> | null; Chain?: ChainRun | null; Planned?: boolean; AppRun?: boolean;
    // The GRASP PLAN's own NEXT STEP: the app's (AppRun) or the model's exact copy of it (lib/AppPlannedSteps.ts).
    FollowsPlan?: boolean } | null>(null);
  // The app's own steps approved since the model's last call (lib/AppPlannedSteps.ts): its next call is told of them.
  const AppStepsSinceModelRef = useRef<AppRunStep[]>([]);
  // How long the task's last model call took (ms): what a step the app proposes itself saves, as the record estimates it.
  const LastModelMsRef = useRef<number | null>(null);
  // The chained motion being run: how its legs went (the check-in or the chat
  // says so), and its approved proposal, for the second leg's check.
  const ChainRunRef = useRef<ChainRun | null>(null);
  const ChainProposalRef = useRef<AiMotionProposal | null>(null);
  // CheckIn: how the motion a check-in follows went (lib/AppPlannedSteps.ts), when one did.
  const runAiRequestRef = useRef<(promptText: string, CheckIn?: AppCheckIn) => void>(() => {});
  // The socket handler outlives renders, so it stops tasks through this ref,
  // and hands a refusal of the approved motion to the task through the next.
  const StopTaskWithReasonRef = useRef<(Reason: string, Advice: string) => void>(() => {});
  const BridgeRefusedTaskMotionRef = useRef<(Text: string) => void>(() => {});
  // Refusals of approved motions by the bridge in this task: the first goes
  // back to the model as a check-in, the second stops the task.
  const BridgeRefusalsRef = useRef(0);
  // Camera-to-workspace calibration UI (bridge /calibration endpoints). The user
  // touches the gripper tip to the table, freezes a frame, and clicks the tip
  // pixel — pairing pixels with real FK positions until a homography is fitted.
  const [calOpen, setCalOpen] = useState(false);
  const [calFrame, setCalFrame] = useState<{ url: string; width: number; height: number; mirrored: boolean } | null>(null);
  const [calStatus, setCalStatus] = useState<CalibrationStatus | null>(null);
  const [calMessage, setCalMessage] = useState("");
  const [calBusy, setCalBusy] = useState(false);
  const [selfCal, setSelfCal] = useState<SelfCalState>({
    running: false, step: "", index: 0, total: 0, seen: 0, report: null,
  });
  const selfCalAbortRef = useRef(false);
  // Servo protection events from the bridge (overload / overcurrent / overheat).
  // The newest event time already reported, so reconnects do not replay history.
  const lastFaultSeenRef = useRef<number | null>(null);
  const selfCalFaultRef = useRef<string | null>(null);
  const SelfCalPreparingRef = useRef(false);
  const SelfCalPrepareEpochRef = useRef(Infinity);
  const LastMotionEpochRef = useRef(-1);
  const SelfCalStopPromiseRef = useRef<Promise<void> | null>(null);
  const SelfCalMoveRequestedRef = useRef(false);
  const selfCalRunningRef = useRef(false);
  // The hottest servo (protection telemetry), read by the semi-automatic run while the arm holds a pose.
  const HottestRef = useRef<number | null>(null);
  const RedHintRef = useRef<{ u: number; v: number } | null>(null);
  const [RedPreview, SetRedPreview] = useState<string | null>(null);
  const RedPreviewImageRef = useRef<HTMLImageElement>(null), CalFrameImageRef = useRef<HTMLImageElement>(null);
  // The pixel under each magnifier's crosshair: what a click there records.
  const RedPreviewPointRef = useRef<SourcePoint | null>(null), CalFramePointRef = useRef<SourcePoint | null>(null);
  const LastStateAtRef = useRef(0);
  const MotionHealthRef = useRef(false);
  const ExpectedTargetsRef = useRef<Record<string, number>>({});
  const MovingRef = useRef(false);
  // Whether any joint's measured position changed in the last state frame. The
  // bridge's moving flag stays set while a servo has an unreached target, so a
  // gripper parked short of a full open read as moving for good (2026-09-29
  // 22:55); a task waits for positions that stopped changing instead.
  const PositionsMovingRef = useRef(false);
  const LastPresentRawRef = useRef<number[] | null>(null);
  // When a state frame last showed the arm moving (a task reads the pose only
  // after the arm has been still for a moment).
  const LastArmMotionAtRef = useRef(0);
  // The bridge is executing a basket route (all of its stages); from state frames.
  const RouteActiveRef = useRef(false);
  // The target the bridge is holding (model frame), from state frames.
  const BridgeTargetRef = useRef<TargetPose | null>(null);
  const MotionTimeoutRef = useRef(SELFCAL_SETTLE_TIMEOUT_MS);
  const AiEpochRef = useRef(0);
  const ConfigurationRef = useRef("");
  // The motion fault the bridge has latched (its text), until Hold clears it.
  const MotionFaultRef = useRef<string | null>(null);
  // The latched fault last recorded (its bridge event's kind, reason and time, else its text). Kept across socket
  // closes and, in this browser, page loads: the same fault seen again is recorded seen_again, not as a new one (audit 2 M3).
  const RecordedFaultRef = useRef<string | null>(null);
  // Each task gets an id; the last one whose end was recorded, and why it
  // ended (the task recording's reason). EndAiTask is reached from the socket
  // handler, which outlives renders, through the ref.
  const TaskIdRef = useRef(0);
  const TaskEndRecordedRef = useRef(0);
  const TaskEndReasonRef = useRef<string | null>(null);
  // The task the model last asked a question in: the next prompt may answer it (ContinueAskedTask).
  const AskedTaskRef = useRef<AskedTask | null>(null);
  const EndAiTaskRef = useRef<(Reason: string, Source: TaskEndSource, Options?: { Advice?: string; Status?: string }) => boolean>(() => false);
  const [markerCheck, setMarkerCheck] = useState<{ busy: boolean; text: string; ok: boolean | null }>({
    busy: false, text: "", ok: null,
  });
  const [cameraStatus, setCameraStatus] = useState("Starting camera");
  const [cameraActive, setCameraActive] = useState(false);
  // Mirrors cameraActive for async flows that can start before the first frame.
  const cameraActiveRef = useRef(false);
  cameraActiveRef.current = cameraActive;
  // Camera actions right after load, or while the depth camera reconnects,
  // wait briefly for the first frame instead of failing at once.
  const waitForCamera = async (timeoutMs = 4000) => {
    const started = performance.now();
    while (!cameraActiveRef.current && performance.now() - started < timeoutMs) await sleep(100);
    return cameraActiveRef.current;
  };
  // TASK RECORDING: the webcam is recorded for the whole life of a task (from the
  // user's prompt until the task ends by done/ask/stop/block/emergency/limit) and
  // the .webm is archived by the bridge next to the still captures. The AI never
  // sees it; it is for reviewing what physically happened afterwards.
  const taskRecorderRef = useRef<{ recorder: MediaRecorder; chunks: Blob[]; task: string; startedAt: number } | null>(null);
  const [taskRecording, setTaskRecording] = useState(false);
  // The Orbbec depth camera, streamed by the bridge: undefined until the first
  // status answer, null when the bridge cannot be reached.
  const [depthCamera, setDepthCamera] = useState<DepthCameraStatus | null | undefined>(undefined);
  const [cameraSource, setCameraSource] = useState<CameraSource | null>(null);
  const DepthCameraMissesRef = useRef(0);
  const cameraView: CameraView = cameraSource === "depth" ? DisplaySettings.DepthCameraView : DisplaySettings.WebcamView;
  const cameraMirrored = cameraView === "Facing";
  const CameraMirroredRef = useRef(cameraMirrored);
  CameraMirroredRef.current = cameraMirrored;
  // What each approved motion of the current task was, for the check-ins.
  const TaskMotionsRef = useRef<string[]>([]);
  // The camera check that stopped the last AI request, shown until a later
  // capture passes it: the user must lock the head and recalibrate.
  const [CameraGateBanner, SetCameraGateBanner] = useState<CameraGate | null>(null);
  // Whether this task's latest capture passed the camera check. Stop advice
  // says "move the object closer" only when the positions were trustworthy.
  const CameraVerifiedRef = useRef(false);
  // The object the task acts on, and where it was last measured.
  const TaskObjectRef = useRef<TaskObjectState | null>(null);
  // Each measured label's last reading with the gripper clear of it, in this task and a task that goes on
  // from it (lib/ArmMadeObjects.ts): it stands for a reading that is the gripper itself.
  const ArmClearRef = useRef<ArmClearReadings>(new Map());
  const LatestSceneRef = useRef<SceneIdentity | null>(null);
  // Moves that would do nothing, or repeat, stop the task instead of using up
  // its motions: no-ops so far, the last offered target, and the requested
  // targets the app rejected (key -> the request that rejected it).
  const NoOpCountRef = useRef(0);
  const LastFinalTargetRef = useRef<FinalTargetMark | null>(null);
  const TaskRejectedTargetsRef = useRef(new Map<string, number>());
  // The pending proposal is the task's last allowed motion.
  const TaskEndsAfterApprovalRef = useRef(false);
  // The bridge answers every command with one status, in order, on its socket.
  // Counting both ties a refusal to the approved motion's own commands, not to
  // a Home or slider move made while the task waits.
  const CommandsSentRef = useRef(0);
  const StatusRepliesRef = useRef(0);
  const ApprovedCommandsRef = useRef<{ First: number; Last: number } | null>(null);
  // An approved gripper close in this task: the object may be in the jaws,
  // travelling with the tip, so "lost far from the gripper" does not apply
  // until an approved step opens the gripper again.
  const TaskGripperClosedRef = useRef(false);
  // Where the tip was (display frame) when the task's last close was sent, and the GRASP PLAN it was
  // sent with: a close chained with its lift is judged a miss where it closed (AiMeasurements
  // ClosedOnNothing), not where the lift left the tip. Null once an approved step opens the gripper.
  const TaskCloseRef = useRef<TaskClose | null>(null);
  // The bridge holds the gripper where a close stopped on an object (its grasp
  // hold): a chained close has then settled short of its target on purpose.
  const GripperGraspHoldRef = useRef(false);
  // The grasp as the NEXT STEP line and the closed-jaw rule see it: the task
  // object's last measurement, how far open the gripper is, and whether a
  // close was approved in this task; and its GRASP PLAN, whose prism the closed-jaw rule uses.
  // The target the bridge holds: a step that repeats it would not move the arm.
  const TaskGrasp = (Servos: Record<ServoName, ServoState>): { Object: LocatedObject; State: GraspState; Plan: GraspPlan | null } | null => {
    const Seen = aiTaskRef.current ? TaskObjectRef.current?.LastSeen?.Object : null;
    if (!Seen) return null;
    const Held = BridgeTargetRef.current;
    return { Object: Seen, State: { GripperFraction: rawToFraction(Servos.gripper, Servos.gripper.presentRaw), Closed: TaskGripperClosedRef.current,
      Held: Held ? { Point: mirrorDisplayPoint(Held), Pitch: Held.wristPitch } : null, RollRaw: Servos.wrist_roll.presentRaw },
      Plan: TaskObjectRef.current?.Plan?.Reach.Plan ?? null };
  };
  const [captureCountdown, setCaptureCountdown] = useState<number | null>(null);

  const angles = useMemo(() => currentAngles(servos), [servos, CadRevision]);
  const localJoints = useMemo(() => forwardDisplayKinematics(angles), [angles]);
  const joints = useMemo(
    () => (socketState === "live" && bridgeJoints ? bridgeJoints : localJoints),
    [bridgeJoints, localJoints, socketState],
  );
  // Live mirrors for the self-calibration loop, which runs across many renders
  // inside one async function and must always read the newest bridge state.
  const servosRef = useRef(servos);
  servosRef.current = servos;
  const bridgeJointsRef = useRef(bridgeJoints);
  bridgeJointsRef.current = bridgeJoints;
  const torqueRef = useRef(torque);
  torqueRef.current = torque;
  const armLiveRef = useRef(false);
  armLiveRef.current = socketState === "live" && armConnected;
  // A task reads these after its capture, which spans many renders.
  const ElbowModeRef = useRef(elbowMode);
  ElbowModeRef.current = elbowMode;
  const BridgeIkLimitsRef = useRef(bridgeIkLimits);
  BridgeIkLimitsRef.current = bridgeIkLimits;
  const localIkPreview = useMemo(
    () => solveIkPreview(target, elbowMode, servos, bridgeIkLimits ?? undefined),
    [bridgeIkLimits, elbowMode, servos, target, CadRevision],
  );
  useEffect(() => { if (localIkPreview.reachable && !TargetEditing) LastValidTargetRef.current = target; }, [localIkPreview, target, TargetEditing]);
  const aiProposalIkPreview = useMemo(
    () => {
      const Proposal = aiDisplayedProposal ?? aiProposal;
      if (!Proposal?.hasTargetMotion) return null;
      // A chained motion is shown as it ends: the tip at its target, the gripper at its chained fraction.
      return solveIkPreview(Proposal.target, elbowMode, Proposal.chain ? servosWithGripper(servos, Proposal.chain.TargetRaw) : servos, bridgeIkLimits ?? undefined);
    },
    [aiDisplayedProposal, aiProposal, elbowMode, servos, bridgeIkLimits, CadRevision],
  );
  const AiDirectServoPreviewAngles = useMemo(() => {
    if (!aiProposal || aiProposal.hasTargetMotion || aiProposal.servoTargets.length === 0) {
      return null;
    }
    if (aiDisplayedProposal && aiProposal.rawResponse !== aiDisplayedProposal.rawResponse) {
      return null;
    }
    const previewServos = cloneServos(servos);
    for (const proposal of aiProposal.servoTargets) {
      previewServos[proposal.name] = {
        ...previewServos[proposal.name],
        presentRaw: proposal.targetRaw,
        targetRaw: proposal.targetRaw,
        guiAngleRaw: null,
      };
    }
    return currentAngles(previewServos);
  }, [aiDisplayedProposal, aiProposal, servos, CadRevision]);
  const aiDirectServoPreviewJoints = AiDirectServoPreviewAngles ? forwardDisplayKinematics(AiDirectServoPreviewAngles) : null;
  const currentPreviewKey = previewKey(target, elbowMode, angles.wrist_roll, angles.gripper, CadRevision);
  // A preview belongs to its target, held wrist roll, and CAD reference.
  const currentBridgePreview = bridgeIkPreview
    && bridgeIkPreview.key === currentPreviewKey
    ? bridgeIkPreview
    : null;
  const ikPreview = useMemo(() => {
    const Preview = !TargetEditing && socketState === "live" && currentBridgePreview ? currentBridgePreview : localIkPreview;
    const Reason = Preview.collisionReason ?? BasketTargetReason(target) ?? (Preview.reachable ? BasketPoseReason(Preview.angles) : localIkPreview.collisionReason);
    return Reason ? { ...Preview, reachable: false, collisionReason: Reason } : Preview;
  }, [socketState, currentBridgePreview, localIkPreview, target, TargetEditing]);
  const targetWristPitchReachable = currentBridgePreview
    ? currentBridgePreview.wristPitchReachable
    : localIkPreview.wristPitchReachable;
  const targetEffectiveWristPitch = currentBridgePreview
    ? currentBridgePreview.effectiveWristPitch
    : localIkPreview.effectiveWristPitch;
  const displayTarget = useMemo(
    () => ({ ...mirrorDisplayPoint(target), wristPitch: target.wristPitch }),
    [target],
  );
  const visualTarget = useMemo(
    () => visualPointFromDisplayPoint(displayTarget),
    [displayTarget],
  );
  // Pitch changes and camera motion do not change the allowable angle set.
  // Quantize only the solver's four seed angles to avoid restarting on encoder
  // jitter. Held tool angles, actual joint limits and CAD alignment stay exact.
  const PitchGuideKey = JSON.stringify({ Position: [target.x, target.y, target.z], ElbowMode: elbowMode,
    Seed: IK_JOINTS.map(Name => Math.round(angles[Name] / 2) * 2),
    Tool: [angles.wrist_roll, angles.gripper], Limits: GetIkAngleLimits(servos, bridgeIkLimits ?? undefined), CadRevision });
  const [PitchGuide, SetPitchGuide] = useState<{ Key: string; Segments: PitchGuideSegment[] } | null>(null);
  const ResolvePitchKey = !localIkPreview.reachable && TargetEditRef.current?.Kind !== "Pitch" ? target.wristPitch : null;
  const PitchGuideRequestRef = useRef({ Target: target, Context: TargetContextRef.current, Alignment: GetCadAlignment() });
  PitchGuideRequestRef.current = { Target: target, Context: TargetContextRef.current, Alignment: GetCadAlignment() };
  TargetPitchGuideRef.current = PitchGuide?.Key === PitchGuideKey ? { Target: target, Segments: PitchGuide.Segments } : null;
  useEffect(() => {
    if (PitchGuide?.Key === PitchGuideKey && ResolvePitchKey === null) return;
    let WorkerInstance: Worker | null = null, Cancelled = false;
    const Timer = window.setTimeout(() => {
      try {
        WorkerInstance = new Worker(new URL("./lib/PitchGuideWorker.ts", import.meta.url), { type: "module" });
        const Request = PitchGuideRequestRef.current;
        WorkerInstance.onmessage = (Event: MessageEvent<{ Segments: PitchGuideSegment[]; Resolved?: TargetPose | null }>) => {
          if (!Cancelled) {
            SetPitchGuide({ Key: PitchGuideKey, Segments: Event.data.Segments });
            const Resolved = Event.data.Resolved, Current = CurrentTargetRef.current, Edit = TargetEditRef.current;
            if (Resolved && !selfCalRunningRef.current && !TargetWorkerRef.current && Edit?.Kind !== "Pitch"
              && ["x", "y", "z", "wristPitch"].every(Key => Current[Key as keyof TargetPose] === Request.Target[Key as keyof TargetPose])
              && Resolved.wristPitch !== Current.wristPitch
              && solveIkPreview(Resolved, TargetContextRef.current.ElbowMode, TargetContextRef.current.Servos, TargetContextRef.current.Limits).reachable) {
              if (Edit) Edit.Candidate = Resolved;
              CurrentTargetRef.current = Resolved; setTarget(Resolved);
              setStatus("Wrist angle adjusted to keep the target position reachable");
            }
          }
          WorkerInstance?.terminate();
        };
        WorkerInstance.onerror = () => {
          if (!Cancelled) SetPitchGuide({ Key: PitchGuideKey, Segments: [] });
          WorkerInstance?.terminate();
        };
        WorkerInstance.postMessage({ ...Request, Guide: PitchGuide?.Key === PitchGuideKey ? PitchGuide.Segments : undefined });
      } catch { if (!Cancelled) SetPitchGuide({ Key: PitchGuideKey, Segments: [] }); }
    }, 60);
    return () => { Cancelled = true; window.clearTimeout(Timer); WorkerInstance?.terminate(); };
  }, [PitchGuideKey, ResolvePitchKey]);
  // The AI target estimate arrives in the display frame; convert it to the scene
  // frame the canvas projects (same transform the target marker uses).
  const aiObjectBoxVisual = useMemo(
    () => (aiObjectBox ? { ...aiObjectBox, Center: visualPointFromDisplayPoint(aiObjectBox.Center) } : null),
    [aiObjectBox],
  );
  const aiTargetEstimateVisual = useMemo(
    () => (aiTargetEstimate ? visualPointFromDisplayPoint(aiTargetEstimate) : null),
    [aiTargetEstimate],
  );
  const renderJoints = useMemo(() => sceneJointsFromDisplayJoints(joints), [joints]);
  const renderPreviewJoints = useMemo(
    () => sceneJointsFromDisplayJoints(ikPreview.joints),
    [ikPreview],
  );
  const renderProposalTarget = useMemo(
    () => (
      aiDisplayedProposal?.hasTargetMotion
        ? visualPointFromDisplayPoint({
          ...mirrorDisplayPoint(aiDisplayedProposal.target),
          wristPitch: aiDisplayedProposal.wristPitchTarget,
        })
        : null
    ),
    [aiDisplayedProposal],
  );


  useEffect(() => {
    let cancelled = false;
    let searching = false;
    let found = false;
    let launchAskedAt = 0;
    let timer: number | undefined;

    // "found": a model is set. "down": Ollama did not answer. "none": Ollama
    // answered but has no model installed.
    const loadModel = async (): Promise<"found" | "down" | "none"> => {
      let reachable = false;
      // 1. Default to the LARGEST installed model that can accept images. The user
      //    wants vision, so this takes priority over reusing whatever is already
      //    resident in memory — selecting it may trigger a fresh load, which is fine.
      try {
        const tagsRes = await fetch(OLLAMA_TAGS_URL);
        if (tagsRes.ok) {
          reachable = true;
          // OllamaOptions.json's preferred model first (if installed), then largest first.
          const ranked = readInstalledModels(await tagsRes.json())
            .sort((A, B) => Number(B.id === PreferredModel) - Number(A.id === PreferredModel));
          let visionModel: InstalledModel | null = null;
          for (const model of ranked) {
            try {
              const showRes = await fetch(OLLAMA_SHOW_URL, {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify({ model: model.id }),
              });
              if (showRes.ok) {
                const show = await showRes.json() as unknown;
                if (isRecord(show) && modelHasVision(show.capabilities)) {
                  visionModel = model; // first match = largest, since ranked is sorted
                  break;
                }
              }
            } catch {
              // Skip this model and keep checking smaller ones.
            }
            if (cancelled) return "found";
          }
          if (cancelled) return "found";
          if (visionModel) {
            const contextLength = await loadedContextLengthFor(visionModel.id);
            if (cancelled) return "found";
            setAiModel(visionModel.id);
            setAiContextLength(contextLength);
            setAiStatus(`${visionModel.id} ready (vision)`);
            return "found";
          }
        }
      } catch {
        // /api/tags or /api/show unavailable — fall back to the choices below.
      }

      // 2. No vision-capable model installed — prefer whatever Ollama already has
      //    resident (reported by /api/ps) to avoid forcing a fresh load.
      try {
        const res = await fetch(OLLAMA_LOADED_MODELS_URL);
        if (res.ok) {
          reachable = true;
          const data = await res.json() as unknown;
          const loaded = pickLoadedModel(readNativeModelEntries(data));
          if (cancelled) return "found";
          if (loaded) {
            setAiModel(loaded.id);
            setAiContextLength(loaded.contextLength);
            setAiStatus(loaded.state === "loading" ? `${loaded.id} loading` : `${loaded.id} ready (no vision model installed)`);
            return "found";
          }
          // Nothing loaded yet — fall through to list installed models below.
        }
      } catch {
        // /api/ps unavailable — fall back to the model list below.
      }

      // 3. Last resort: the first installed model from the OpenAI-style list.
      try {
        const res = await fetch(OLLAMA_MODELS_URL);
        if (res.ok) {
          reachable = true;
          const firstModel = pickFirstModel(readModelIds(await res.json() as unknown));
          if (cancelled) return "found";
          if (firstModel) {
            setAiModel(firstModel);
            setAiContextLength(null);
            setAiStatus(`${firstModel} ready (no vision model installed)`);
            return "found";
          }
        }
      } catch {
        // Ollama is not answering.
      }
      return reachable ? "none" : "down";
    };

    // Ollama is often not up yet when the page loads (after a reboot it does not
    // start by itself), so keep looking until a model turns up instead of
    // staying "offline" until a reload.
    const search = async () => {
      if (cancelled || searching || found) return;
      searching = true;
      window.clearTimeout(timer);
      try {
        const result = await loadModel();
        found = result === "found";
        if (cancelled || found) return;
        if (result === "none") {
          setAiStatus("Ollama has no models installed. Pull one, e.g. ollama pull qwen3.8:27b");
        } else {
          // Ask the dev server to start a local Ollama, at most every 30 s.
          if (Date.now() - launchAskedAt > 30_000) {
            launchAskedAt = Date.now();
            void fetch(OLLAMA_LAUNCH_URL, { method: "POST" }).catch(() => undefined);
          }
          setAiStatus("Waiting for Ollama to start, retrying every few seconds");
        }
        timer = window.setTimeout(() => void search(), MODEL_RETRY_MS);
      } finally {
        searching = false;
      }
    };
    const searchNow = () => { if (document.visibilityState === "visible") void search(); };

    setAiStatus("Finding the largest vision model");
    void search();
    window.addEventListener("focus", searchNow);
    document.addEventListener("visibilitychange", searchNow);
    return () => {
      cancelled = true;
      window.clearTimeout(timer);
      window.removeEventListener("focus", searchNow);
      document.removeEventListener("visibilitychange", searchNow);
    };
  }, []);

  // Warm the model once per page load, and again when the model changes, as soon
  // as AI mode shows: a cold load (6-19 s measured) otherwise sits inside the
  // first task's first call (07:57 on 2026-09-29: the task ended 2 s before its
  // reply came). Manual mode never asks the model, so it loads nothing (the model
  // then holds ~22 GB of VRAM until keep_alive runs out). Fire and forget; the
  // statistics then show the context window it was loaded with.
  const PreloadedModelRef = useRef<string | null>(null);
  const ModelWarmAtRef = useRef(0);
  const WarmAiModel = useCallback((Model: string) => {
    ModelWarmAtRef.current = Date.now();
    void preloadAiModel(Model).then(async (Loaded) => {
      if (!Loaded) return;
      const Length = await loadedContextLengthFor(Model);
      if (Length !== null && PreloadedModelRef.current === Model) setAiContextLength(Length);
    });
  }, []);
  useEffect(() => {
    if (!aiModel || ControlMode !== "Ai" || PreloadedModelRef.current === aiModel) return;
    PreloadedModelRef.current = aiModel;
    WarmAiModel(aiModel);
  }, [aiModel, ControlMode, WarmAiModel]);

  // Nimble (the prompt router) warms as soon as AI mode shows, without waiting for the main model's search: a prompt
  // sent while it is still cold or warming takes the rule classifier's route (lib/AiPromptRouter.ts RuleRoute).
  useEffect(() => {
    if (ControlMode !== "Ai") return;
    const Controller = new AbortController();
    SetAiRouteStatus(PromptRouterOptions.Enabled ? "Preparing Nimble routing" : "Nimble routing disabled");
    void PromptRouterRef.current.Prepare(Controller.signal).then(Status => {
      if (!Controller.signal.aborted && !AiRequestAbortRef.current) SetAiRouteStatus(Status.Reason);
    }).catch(() => {});
    return () => Controller.abort();
  }, [ControlMode]);

  // Keep Nimble warm while someone uses the page: every minute (and on coming back to the tab) a fresh /api/ps check
  // warms it again when Ollama restarted, evicted it, or its router_keep_alive is about to run out. Only within
  // RouterActiveMs of the last key, click, scroll or pointer move, so an unattended page lets it unload (its ~9.5 GB
  // of VRAM is free again at most router_keep_alive + RouterActiveMs after the last use).
  useEffect(() => {
    if (ControlMode !== "Ai" || !PromptRouterOptions.Enabled) return;
    const Controller = new AbortController();
    let LastActivity = Date.now();
    const Active = () => { LastActivity = Date.now(); };
    const Tick = () => {
      if (document.visibilityState !== "visible" || Date.now() - LastActivity > RouterActiveMs) return;
      void PromptRouterRef.current.KeepWarm(Controller.signal);
    };
    const Shown = () => { if (document.visibilityState === "visible") { Active(); Tick(); } };
    const Events = ["pointerdown", "pointermove", "keydown", "wheel"] as const;
    for (const Name of Events) window.addEventListener(Name, Active, { passive: true });
    document.addEventListener("visibilitychange", Shown);
    const Timer = window.setInterval(Tick, 60_000);
    return () => {
      Controller.abort(); window.clearInterval(Timer);
      for (const Name of Events) window.removeEventListener(Name, Active);
      document.removeEventListener("visibilitychange", Shown);
    };
  }, [ControlMode]);

  const pushLog = (level: StatusEntry["level"], text: string) => {
    setLog((items) => [
      { id: Date.now() + Math.random(), level, text },
      ...items,
    ].slice(0, 5));
  };

  const updateTarget = (updater: TargetPose | ((previous: TargetPose) => TargetPose)) => {
    TargetWorkerRef.current?.terminate(); TargetWorkerRef.current = null;
    TargetEditRef.current = null; SetTargetEditing(false); SetTargetSnapping(false);
    setTarget((previous) => {
      const next = typeof updater === "function" ? updater(previous) : updater;
      const Reason = BasketTargetReason(next);
      if (Reason) { setStatus(Reason); return previous; }
      return constrainTargetPose(next, previous);
    });
  };

  const BeginTargetEdit = (Kind: "Position" | "Pitch" = "Position") => {
    TargetWorkerRef.current?.terminate(); TargetWorkerRef.current = null;
    if (!TargetEditRef.current) TargetEditRef.current = { Original: target, Candidate: target, Kind };
    else TargetEditRef.current.Kind = Kind;
    SetTargetEditing(true); SetTargetSnapping(false);
    if (!WarmTargetWorkerRef.current) try { WarmTargetWorkerRef.current = NewTargetWorker(); } catch { /* started on release instead */ }
  };
  const PreviewManualTarget = (Candidate: TargetPose) => {
    if (!Object.values(Candidate).every(Number.isFinite)) return;
    if (!TargetEditRef.current) BeginTargetEdit();
    TargetEditRef.current!.Candidate = Candidate;
    CurrentTargetRef.current = Candidate;
    setTarget(Candidate);
  };
  const PreviewVisualTarget = (Updater: TargetPose | ((Previous: TargetPose) => TargetPose)) => {
    const Previous = TargetEditRef.current?.Candidate ?? target;
    const Visual = { ...proposalDisplayPointCm(Previous), wristPitch: Previous.wristPitch };
    const Next = typeof Updater === "function" ? Updater(Visual) : Updater;
    const Model = modelPointFromVisualPoint(Next);
    // A pitch-only edit must not perturb XYZ through a floating-point display
    // round trip, which would also invalidate the cached green angle guide.
    PreviewManualTarget({ wristPitch: Next.wristPitch,
      x: Next.x === Visual.x ? Previous.x : Model.x,
      y: Next.y === Visual.y ? Previous.y : Model.y,
      z: Next.z === Visual.z ? Previous.z : Model.z });
  };
  const FinishTargetEdit = (Cancelled = false) => {
    const Edit = TargetEditRef.current;
    if (!Edit) return;
    TargetWorkerRef.current?.terminate(); TargetWorkerRef.current = null;
    const Requested = Cancelled ? Edit.Original : Edit.Candidate;
    Edit.Candidate = Requested;
    const Valid = (Candidate: TargetPose | null): Candidate is TargetPose => {
      const Context = TargetContextRef.current;
      return !!Candidate && Object.values(Candidate).every(Number.isFinite)
        && solveIkPreview(Candidate, Context.ElbowMode, Context.Servos, Context.Limits).reachable;
    };
    const Finish = (Candidate: TargetPose | null, TimedOut = false) => {
      const Accepted = Valid(Candidate) ? Candidate : [Edit.Original, LastValidTargetRef.current].find(Valid);
      TargetEditRef.current = null; SetTargetEditing(false); SetTargetSnapping(false);
      if (Accepted) { CurrentTargetRef.current = Accepted; setTarget(Accepted); LastValidTargetRef.current = Accepted; }
      setStatus(Candidate && Accepted === Candidate ? ["x", "y", "z"].every(Key => Candidate[Key as keyof TargetPose] === Requested[Key as keyof TargetPose])
        ? "Wrist angle adjusted; target position preserved" : "Target pulled back along the drag to the edge of the reachable area"
        : Accepted ? (TimedOut ? "No reachable spot found within half a second; previous target restored" : "No valid solution found; previous target restored")
        : "No valid solution found within the joint and basket limits.");
    };
    if (Cancelled || Valid(Requested)) {
      TargetEditRef.current = null; SetTargetEditing(false); SetTargetSnapping(false);
      CurrentTargetRef.current = Requested; setTarget(Requested);
      if (Valid(Requested)) LastValidTargetRef.current = Requested;
      return;
    }
    SetTargetSnapping(true);
    let WorkerInstance: Worker;
    try { WorkerInstance = WarmTargetWorkerRef.current ?? NewTargetWorker(); }
    catch { Finish(null); return; }
    WarmTargetWorkerRef.current = null;
    TargetWorkerRef.current = WorkerInstance;
    const Answer = (Target: TargetPose | null, TimedOut = false) => {
      if (TargetWorkerRef.current !== WorkerInstance) return;
      window.clearTimeout(Limit); WorkerInstance.terminate(); TargetWorkerRef.current = null;
      Finish(Target, TimedOut);
    };
    const Limit = window.setTimeout(() => Answer(null, true), TargetSnapLimitMs);
    WorkerInstance.onmessage = (Event: MessageEvent<{ Target: TargetPose | null }>) => Answer(Event.data.Target);
    WorkerInstance.onerror = () => Answer(null);
    const Guide = TargetPitchGuideRef.current;
    WorkerInstance.postMessage({ Requested, Previous: Edit.Original, Kind: Edit.Kind,
      Context: TargetContextRef.current, Alignment: GetCadAlignment(),
      Guide: Guide && ["x", "y", "z"].every(Key => Guide.Target[Key as keyof TargetPose] === Requested[Key as keyof TargetPose]) ? Guide.Segments : undefined });
  };

  const sendCommand = (message: Record<string, unknown>) => {
    const socket = socketRef.current;
    CommandFailureRef.current = CommandBlockReason(message, { SocketOpen: socket?.readyState === WebSocket.OPEN,
      BasketCompatible: BasketCompatibleRef.current, CadCompatible: CadCompatibleRef.current,
      SpeedCompatible: SpeedCompatibleRef.current, Calibrating: selfCalRunningRef.current, Restarting: BridgeRestartingRef.current,
      HandGuiding: HandGuideRef.current.On || HandGuideRef.current.Calibrating });
    if (CommandFailureRef.current) { setStatus(CommandFailureRef.current); return false; }
    // WebSocket commands execute in order. Flush a coalesced speed change before
    // starting a move so short motions cannot finish using the previous speed.
    // Stop, emergency stop and torque commands are never delayed by the fader.
    if (["send_ik", "move_servo", "go_home"].includes(String(message.type))
      && SpeedSendTimerRef.current !== undefined && !SendPendingMotionSpeed()) return false;
    try {
      socket!.send(JSON.stringify(message));
      CommandsSentRef.current += 1;
      return true;
    } catch {
      CommandFailureRef.current = "The bridge connection closed before the command could be sent. Reconnect and send again.";
      setStatus(CommandFailureRef.current); return false;
    }
  };

  const ClearMotionSpeedTimers = () => {
    window.clearTimeout(SpeedSendTimerRef.current);
    window.clearTimeout(SpeedAckTimerRef.current);
    SpeedSendTimerRef.current = undefined;
    SpeedAckTimerRef.current = undefined;
    PendingMotionSpeedRef.current = null;
  };

  const ResetMotionSpeedControl = () => {
    ClearMotionSpeedTimers();
    SpeedCompatibleRef.current = false;
    SetSpeedCompatible(false);
    SetMotionSpeedPercent(ServerMotionSpeedRef.current);
  };

  const SendPendingMotionSpeed = (): boolean => {
    window.clearTimeout(SpeedSendTimerRef.current);
    SpeedSendTimerRef.current = undefined;
    const Pending = PendingMotionSpeedRef.current;
    if (Pending === null) return true;
    if (!sendCommand({ type: "set_motion_speed", percent: Pending })) {
      ResetMotionSpeedControl();
      return false;
    }
    LastSpeedSentAtRef.current = performance.now();
    window.clearTimeout(SpeedAckTimerRef.current);
    // Telemetry owns the confirmed setting. Never leave an unacknowledged
    // local slider value looking as if the hardware accepted it.
    SpeedAckTimerRef.current = window.setTimeout(() => {
      ClearMotionSpeedTimers();
      SetMotionSpeedPercent(ServerMotionSpeedRef.current);
      setStatus("Speed change was not confirmed by the bridge");
    }, 2000);
    return true;
  };

  const ChangeMotionSpeed = (Percent: number) => {
    if (!SpeedCompatibleRef.current || !Number.isFinite(Percent)) return;
    const Next = Math.round(Math.max(10, Math.min(100, Percent)));
    PendingMotionSpeedRef.current = Next;
    SetMotionSpeedPercent(Next);
    window.clearTimeout(SpeedAckTimerRef.current);
    const WaitMs = Math.max(0, 90 - (performance.now() - LastSpeedSentAtRef.current));
    if (WaitMs === 0) SendPendingMotionSpeed();
    else if (SpeedSendTimerRef.current === undefined) {
      SpeedSendTimerRef.current = window.setTimeout(SendPendingMotionSpeed, WaitMs);
    }
  };

  const clearServoCommandTimers = () => {
    for (const timer of Object.values(servoCommandTimersRef.current)) {
      if (timer !== undefined) window.clearTimeout(timer);
    }
    servoCommandTimersRef.current = {};
  };

  // Slider drags are coalesced to one move_servo per servo per 90 ms. An
  // approved AI move is one command (Immediate): it is sent now, so the
  // approval's command range holds it and a refusal of it reaches the task,
  // and it returns whether it was really sent.
  const queueServoMove = (name: ServoName, raw: number, Immediate = false, Scene?: SceneIdentity | null, Carry = false) => {
    if (!BasketCompatibleRef.current) { setStatus("Restart the bridge to enable basket avoidance"); return false; }
    if (selfCalRunningRef.current) return false;
    if (!torqueRef.current) {
      // Coasted: never queue motion across a Hold press. A throttled timer
      // firing after torque enables would otherwise drive a stale drag.
      delete pendingServoTargetsRef.current[name];
      return false;
    }
    const socket = socketRef.current;
    if (!socket || socket.readyState !== WebSocket.OPEN) {
      return false;
    }

    const now = window.performance.now();
    pendingServoTargetsRef.current[name] = { raw, updatedAt: Date.now(), scene: Scene, carry: Carry };
    const sendLatest = () => {
      const pending = pendingServoTargetsRef.current[name];
      if (!pending) return false;
      lastServoCommandAtRef.current[name] = window.performance.now();
      servoCommandTimersRef.current[name] = undefined;
      if (SceneFreshnessIssue(pending.scene, LatestSceneRef.current)) return false;
      return sendCommand({ type: "move_servo", name, raw: pending.raw, ...SceneCommandFields(pending.scene, pending.carry) });
    };

    const lastSent = lastServoCommandAtRef.current[name] ?? 0;
    const waitMs = Immediate ? 0 : Math.max(0, 90 - (now - lastSent));
    if (waitMs <= 0) {
      if (servoCommandTimersRef.current[name] !== undefined) {
        window.clearTimeout(servoCommandTimersRef.current[name]);
        servoCommandTimersRef.current[name] = undefined;
      }
      const Sent = sendLatest();
      if (Immediate) return Sent;
    } else if (servoCommandTimersRef.current[name] === undefined) {
      servoCommandTimersRef.current[name] = window.setTimeout(sendLatest, waitMs);
    }
    return true;
  };

  const loadPorts = async () => {
    try {
      const response = await fetch(`${API_URL}/ports`);
      const Listed = await response.json() as unknown;
      // A refusal (another device without the arm password: {ok: false, access, error}) is no port list; taken as one,
      // it crashed the page (ports.map).
      if (!response.ok || !Array.isArray(Listed)) { pushLog("warn", "Could not load serial ports"); return; }
      const nextPorts = Listed.filter((Port): Port is string => typeof Port === "string");
      setPorts(nextPorts);
      setSelectedPort((current) => current || nextPorts[0] || "");
      if (!nextPorts.length) {
        pushLog("warn", "No serial ports found");
      }
    } catch {
      pushLog("warn", "Could not load serial ports");
    }
  };

  const connectBridge = async () => {
    const currentSocket = socketRef.current;
    if (
      currentSocket
      && (currentSocket.readyState === WebSocket.OPEN
        || currentSocket.readyState === WebSocket.CONNECTING)
    ) {
      return;
    }
    setSocketState("connecting");
    setStatus("Connecting to bridge");
    const bridgeReady = await startBridgeFromPageLoad();
    if (bridgeReady) {
      pushLog("ok", "Python bridge ready");
    } else {
      pushLog("warn", "Python bridge auto-start unavailable");
    }
    const nextSocket = socketRef.current;
    if (
      nextSocket
      && (nextSocket.readyState === WebSocket.OPEN
        || nextSocket.readyState === WebSocket.CONNECTING)
    ) {
      return;
    }
    const socket = new WebSocket(WS_URL);
    socketRef.current = socket;
    CommandsSentRef.current = 0;
    StatusRepliesRef.current = 0;
    ApprovedCommandsRef.current = null;

    // The bridge is live for this page on this PC as soon as the socket opens; on another device once the bridge
    // accepted the password, its answer to the page's first message (lib/RemoteAccess.ts).
    let Live = false;
    const BridgeLive = () => {
      if (Live || socketRef.current !== socket) return;
      Live = true;
      setSocketState("live");
      setStatus("Bridge connected");
      pushLog("ok", "WebSocket bridge connected");
      void loadPorts();
    };

    socket.onopen = () => {
      if (socketRef.current !== socket) return;
      if (ThisPc) { BridgeLive(); return; }
      socket.send(JSON.stringify(AuthMessage()));
      setStatus("Checking the arm password");
    };

    socket.onmessage = (event) => {
      if (socketRef.current !== socket) return;
      try {
        const message = JSON.parse(event.data);
        if (message.type === "access") {
          if (message.ok) {
            AccessCardDismissedRef.current = false;
            SetRemoteRefusal(null); SetAccessCardOpen(false);
            BridgeLive();
          } else if (IsAccessCode(message.access)) {
            const Refusal = { Code: message.access, Text: AccessText(message.access, typeof message.text === "string" ? message.text : null) };
            SetRemoteRefusal(Refusal);
            if (!AccessCardDismissedRef.current) SetAccessCardOpen(true);
            setStatus(Refusal.Text);
            pushLog("warn", Refusal.Text);
          }
          return;
        }
        if (message.type === "state") {
          // A bridge that never asked this device for the password (one from before the access rule) is live as it was.
          if (!Live) BridgeLive();
          const Temperatures = Object.values(message.protection?.telemetry ?? {}).map((Item: any) => Item.temperature_c).filter((Value): Value is number => typeof Value === "number" && Number.isFinite(Value));
          // Each servo's last reported temperature, for the Joints panel's diagnostics and telemetry lines.
          const PerServo = Object.fromEntries(Object.entries(message.protection?.telemetry ?? {}).sort(([A], [B]) => A.localeCompare(B))
            .map(([Name, Item]: [string, any]) => [Name, Item?.temperature_c]).filter(([, Value]) => typeof Value === "number" && Number.isFinite(Value))) as Partial<Record<ServoName, number>>;
          const LimitC = typeof message.protection?.temperature_limit_c === "number" ? message.protection.temperature_limit_c : 60;
          const Protection = { Loaded: message.protection_version === "auto-coast-v2-confirmed", Reason: String(message.protection?.trip?.reason ?? message.motion_fault ?? ""),
            CoastConfirmed: message.protection?.trip?.coast_confirmed === true, Hottest: Temperatures.length ? Math.max(...Temperatures) : null, Temperatures: PerServo, LimitC };
          SetProtectionState(Previous => JSON.stringify(Previous) === JSON.stringify(Protection) ? Previous : Protection);
          HottestRef.current = Protection.Hottest;
          if (Protection.Reason && typeof message.protection?.trip?.at_s === "number" && message.protection.trip.at_s !== LastProtectionSeen.current) {
            LastProtectionSeen.current = message.protection.trip.at_s;
            pushLog("warn", `${Protection.CoastConfirmed ? "Automatic coast" : "Torque-off unconfirmed"}: ${Protection.Reason}`);
          }
          CadCompatibleRef.current = message.cad_source_commit === CadModel.SourceCommit && Boolean(message.cad_alignment)
            && message.ik_solver_version === IkSolverVersion;
          BasketCompatibleRef.current = message.collision_version === BasketCollisionVersion;
          const Speed = message.motion_speed_percent;
          SpeedCompatibleRef.current = message.speed_control_version === "throttle-v2"
            && typeof Speed === "number" && Number.isFinite(Speed) && Speed >= 10 && Speed <= 100;
          SetSpeedCompatible(SpeedCompatibleRef.current);
          GripperWithTipRef.current = message.gripper_with_tip_version === GripperWithTipVersion;
          SetGripperWithTip(GripperWithTipRef.current);
          BridgeVersionRef.current = typeof message.bridge_version === "string" ? message.bridge_version : null;
          const UpdateReason = BridgeUpdateReason(BasketCompatibleRef.current, CadCompatibleRef.current, SpeedCompatibleRef.current,
            BridgeVersionRef.current);
          SetBridgeUpdate(UpdateReason);
          if (SpeedCompatibleRef.current) {
            ServerMotionSpeedRef.current = Speed;
            // A matching old value cannot acknowledge a change still waiting
            // in the send timer (for example dragging 100 -> 10 -> 100).
            if (PendingMotionSpeedRef.current === null
              || (PendingMotionSpeedRef.current === Speed && SpeedSendTimerRef.current === undefined)) {
              ClearMotionSpeedTimers();
              SetMotionSpeedPercent(Speed);
            }
          } else ResetMotionSpeedControl();
          SetRouteActive(Boolean(message.motion_route));
          RouteActiveRef.current = Boolean(message.motion_route);
          const Held = message.target_cm;
          BridgeTargetRef.current = Held && [Held.x, Held.y, Held.z, message.wrist_pitch].every(Number.isFinite)
            ? { x: Held.x, y: Held.y, z: Held.z, wristPitch: message.wrist_pitch } : null;
          LastMotionEpochRef.current = Number(message.motion_epoch ?? -1);
          if (SelfCalPreparingRef.current && LastMotionEpochRef.current >= SelfCalPrepareEpochRef.current) {
            SelfCalPreparingRef.current = false;
          }
          LastStateAtRef.current = performance.now();
          const StateServos = Object.values(message.servos ?? {}) as { feedback_ok?: boolean; moving?: boolean }[];
          MotionHealthRef.current = !message.motion_fault && message.torque && message.arm_connected
            && StateServos.length === SERVO_ORDER.length && StateServos.every(S => S.feedback_ok === true)
            && typeof message.control_age_ms === "number" && message.control_age_ms < 500;
          MovingRef.current = StateServos.some(S => S.moving);
          const PresentRaw = SERVO_ORDER.map(Name => Number(message.servos?.[Name]?.present_raw));
          const GripperRaw = Number(message.servos?.gripper?.present_raw);
          if (Number.isFinite(GripperRaw)) {
            const Now = Date.now();
            GripperSamplesRef.current = [...GripperSamplesRef.current.filter(Sample => Sample.At > Now - 3000), { At: Now, Raw: GripperRaw }];
          }
          const PreviousRaw = LastPresentRawRef.current;
          LastPresentRawRef.current = PresentRaw;
          PositionsMovingRef.current = !PreviousRaw
            || PresentRaw.some((Raw, Index) => !Number.isFinite(Raw) || Math.abs(Raw - PreviousRaw[Index]) > ArmStillJitterRaw);
          if (PositionsMovingRef.current || RouteActiveRef.current) LastArmMotionAtRef.current = performance.now();
          if (typeof message.motion_timeout_ms === "number"
              && Number.isFinite(message.motion_timeout_ms) && message.motion_timeout_ms > 0) {
            MotionTimeoutRef.current = Math.max(SELFCAL_SETTLE_TIMEOUT_MS, message.motion_timeout_ms);
          }
          // A latched fault (a trip latches one too) is shown and recorded
          // once, when it appears, and ends the AI's task with the bridge's
          // text. Repeated fault frames only end physical tasks, pending
          // proposals and settle waits; conversation remains available.
          const Fault = message.motion_fault ? String(message.motion_fault) : null;
          if (Fault !== MotionFaultRef.current) {
            const Event = isRecord(message.last_bridge_event) && message.last_bridge_event.reason === Fault ? message.last_bridge_event : null;
            if (RecordedFaultRef.current === null) {
              try { RecordedFaultRef.current = localStorage.getItem("SO101.RecordedFault"); } catch { /* Storage optional. */ }
            }
            if (Fault) {
              const Key = JSON.stringify(Event ? [Event.kind, Fault, Event.t] : [null, Fault]);
              const SeenAgain = Key === RecordedFaultRef.current;
              pushLog("warn", SeenAgain ? `Bridge fault (still latched): ${Fault}` : `Bridge fault: ${Fault}`);
              recordSessionEvent("bridge_fault", { kind: typeof Event?.kind === "string" ? Event.kind : "motion_fault", reason: Fault,
                joint: typeof Event?.joint === "string" ? Event.joint : null, details: Event?.details ?? null, ...(SeenAgain ? { seen_again: true } : {}) });
              RecordedFaultRef.current = Key;
              try { localStorage.setItem("SO101.RecordedFault", Key); } catch { /* Storage optional. */ }
            } else {
              pushLog("info", "Bridge fault cleared");
              RecordedFaultRef.current = "";
              try { localStorage.removeItem("SO101.RecordedFault"); } catch { /* Storage optional. */ }
            }
            MotionFaultRef.current = Fault;
          }
          if (Fault) {
            const Kind = isRecord(message.last_bridge_event) && message.last_bridge_event.reason === Fault ? message.last_bridge_event.kind : null;
            // Faults stop physical work, including pending approvals/chains.
            // Routing and inert text replies can finish; a vision route checks
            // the latched fault before it starts a task or takes a camera frame.
            if (aiTaskRef.current || AiProposalRef.current || AiSettleWaitRef.current) {
              EndAiTaskRef.current(`${Kind === "protection_trip" ? "the bridge's motor protection tripped" : "the bridge reported a motion fault"}: ${Fault.replace(/\.$/, "")}.`,
                "motion_fault", { Advice: "Fix the cause, press Hold to clear the fault, then send the task again.", Status: "Stopped by a bridge fault" });
            }
            if (selfCalRunningRef.current && !(SelfCalPreparingRef.current
                && Fault.startsWith("Motion settling timed out"))) {
              selfCalAbortRef.current = true;
              selfCalFaultRef.current = Fault;
            }
          }
          if (typeof message.configuration_key === "string" && message.configuration_key !== ConfigurationRef.current) {
            ConfigurationRef.current = message.configuration_key;
            EndAiTaskRef.current("the arm's configuration (home, joint limits, CAD alignment, workspace frame or collision model) changed while the task ran.",
              "configuration_change", { Advice: "Check the new settings, then send the task again." });
          }
          if (message.cad_alignment && message.cad_alignment.Version !== GetCadAlignment().Version) {
            if (SetCadAlignment(message.cad_alignment as CadAlignment)) {
              RefreshCadFrame(); SetCadRevision(GetCadAlignment().Version); setBridgeIkPreview(null);
            }
          }
          if (message.workspace_offset && ["x", "y", "z"].every(Axis => Number.isFinite(message.workspace_offset[Axis]))) {
            Object.assign(DISPLAY_HOME_OFFSET_CM, message.workspace_offset);
          }
          if (message.home_raw) {
            for (const Name of SERVO_ORDER) {
              if (Number.isFinite(message.home_raw[Name])) HOME_RAW[Name] = message.home_raw[Name];
            }
          }
          if (message.servos) {
            setServos((prev) => {
              const next = cloneServos(prev);
              for (const name of SERVO_ORDER) {
                const incoming = message.servos[name];
                if (!incoming) continue;
                const pending = pendingServoTargetsRef.current[name];
                const incomingTarget = incoming.target_raw ?? next[name].targetRaw;
                const keepLocalTarget = Boolean(
                  pending && Date.now() - pending.updatedAt < 800 && incomingTarget !== pending.raw,
                );
                if (pending && incomingTarget === pending.raw) {
                  delete pendingServoTargetsRef.current[name];
                }
                let guiAngleRaw = next[name].guiAngleRaw ?? null;
                if (Object.prototype.hasOwnProperty.call(incoming, "gui_angle_raw")) {
                  guiAngleRaw = incoming.gui_angle_raw == null
                    ? null
                    : clampRaw(asNumber(incoming.gui_angle_raw, guiAngleRaw ?? next[name].presentRaw));
                }
                next[name] = {
                  name,
                  presentRaw: incoming.present_raw ?? next[name].presentRaw,
                  targetRaw: keepLocalTarget ? next[name].targetRaw : incomingTarget,
                  guiAngleRaw,
                  min: incoming.range_min ?? next[name].min,
                  max: incoming.range_max ?? next[name].max,
                  arcInverted: incoming.arc_inverted ?? next[name].arcInverted,
                };
              }
              return ReuseServoSnapshot(prev, next);
            });
          }
          setArmConnected(Boolean(message.arm_connected));
          const OnPurpose = isRecord(message.disconnected_on_purpose) ? message.disconnected_on_purpose : null;
          const OnPurposeText = OnPurpose ? `${String(OnPurpose.by ?? "someone")}, ${String(OnPurpose.at ?? "")}` : null;
          if (OnPurposeText !== DisconnectedOnPurposeRef.current) {
            DisconnectedOnPurposeRef.current = OnPurposeText;
            SetDisconnectedOnPurpose(OnPurposeText);
            // Said once, unless this page pressed Disconnect itself (it said so then).
            if (OnPurposeText && !message.arm_connected && !ManualDisconnectRef.current) {
              pushLog("info", `The arm was disconnected on purpose (${OnPurposeText}): it connects again when Connect is pressed.`);
            }
          }
          // Each USB serial link drop, and what came of it, goes in the log once, in the bridge's words.
          const SerialLink = isRecord(message.serial_link) ? message.serial_link : null;
          const LinkDown = SerialLink?.state === "down";
          const SerialNote = typeof SerialLink?.note === "string" ? SerialLink.note : null;
          if (LinkDown && !SerialLinkDownRef.current) pushLog("warn", "USB serial link to the arm dropped; the bridge is reconnecting");
          if (SerialNote && SerialNote !== LastSerialNoteRef.current) pushLog(/holding|coasting \(torque off\)/.test(SerialNote) ? "info" : "warn", SerialNote);
          SerialLinkDownRef.current = LinkDown;
          if (SerialNote) LastSerialNoteRef.current = SerialNote;
          SetSerialLinkDown(LinkDown);
          setTorque(Boolean(message.torque));
          const Guide = ReadHandGuide(message);
          if (HandGuideSeenRef.current) {
            const ExitNote = HandGuideExitNote(HandGuideRef.current.LastExit, Guide.LastExit);
            if (ExitNote) pushLog(ExitNote.Level, ExitNote.Text);
            const Result = Guide.CalibrationResult;
            if (Result && Result.AtS !== HandGuideRef.current.CalibrationResult?.AtS) {
              pushLog(Result.Ok ? "ok" : "warn", Result.Ok ? "Hand guiding calibrated and saved: it is ready"
                : `Hand-guiding calibration not saved: ${Result.Error ?? Result.Problems.join("; ")}`);
              if (Result.Ok) SetHandGuideStale(false);
            }
          }
          HandGuideSeenRef.current = true;
          HandGuideRef.current = Guide;
          SetHandGuideState(Previous => JSON.stringify(Previous) === JSON.stringify(Guide) ? Previous : Guide);
          if ((Guide.On || Guide.Calibrating) && performance.now() - HandGuideBeatAtRef.current >= HeartbeatMs) {
            HandGuideBeatAtRef.current = performance.now();
            try { socket.send(JSON.stringify({ type: "hand_guide_heartbeat" })); } catch { /* the socket's close says so */ }
          }
          GripperGraspHoldRef.current = message.servos?.gripper?.grasp_hold === true;
          // A chained motion's second leg never follows a Coast (lib/AiChain.ts): the arm
          // may then be moved by hand, and a Hold makes wherever it is the target.
          if (!message.torque && AiSettleWaitRef.current?.Chain?.Next) {
            EndAiTaskRef.current("the arm's torque went off (Coast) while the first part of a chained motion ran.", "chain_interrupted",
              { Advice: "Check where the arm is, then send the task again." });
          }
          setStatus(UpdateReason ?? String(message.status ?? "Bridge live"));
          if (message.elbow_mode === "up" || message.elbow_mode === "down") {
            setElbowMode(message.elbow_mode);
          }
          const nextBridgeJoints = readPointList(message.joint_positions);
          if (nextBridgeJoints && CadCompatibleRef.current) {
            const Next = applyDisplayHomeOffsetToJoints(nextBridgeJoints);
            setBridgeJoints(Previous => Previous?.length === Next.length && Next.every((Point, Index) =>
              Point.x === Previous[Index].x && Point.y === Previous[Index].y && Point.z === Previous[Index].z) ? Previous : Next);
          } else setBridgeJoints(null);
          if (Array.isArray(message.fault_events)) {
            const events = (message.fault_events as unknown[]).filter(isRecord);
            const newest = events[events.length - 1];
            const newestAt = newest && typeof newest.at_s === "number" ? newest.at_s : null;
            if (lastFaultSeenRef.current === null) {
              lastFaultSeenRef.current = newestAt ?? 0;
            } else if (newestAt !== null && newestAt > lastFaultSeenRef.current) {
              for (const event of events) {
                if (typeof event.at_s !== "number" || event.at_s <= (lastFaultSeenRef.current ?? 0)) continue;
                const servo = String(event.servo ?? "servo").replace(/_/g, " ");
                const unload = Array.isArray(event.unload) ? event.unload.map(String) : [];
                const warn = Array.isArray(event.warn) ? event.warn.map(String) : [];
                const detail = [
                  typeof event.load === "number" ? `load ${event.load}` : null,
                  typeof event.temperature_c === "number" ? `${event.temperature_c} C` : null,
                  typeof event.voltage_dv === "number" ? `${(event.voltage_dv / 10).toFixed(1)} V` : null,
                ].filter(Boolean).join(", ");
                if (unload.length) {
                  const text = `${servo} ${unload.join("/")}${detail ? ` (${detail})` : ""}`;
                  pushLog("warn", `Servo protection tripped: ${text}. All motion stopped.`);
                  if (selfCalRunningRef.current) {
                    selfCalFaultRef.current = text;
                    selfCalAbortRef.current = true;
                  }
                } else if (warn.length) {
                  pushLog("warn", `Servo warning: ${servo} ${warn.join("/")}${detail ? ` (${detail})` : ""}`);
                }
              }
              lastFaultSeenRef.current = newestAt;
            }
          }
          if (isRecord(message.ik_joint_limits)) {
            const parsed: IkAngleLimits = {};
            for (const name of IK_JOINTS) {
              const range = (message.ik_joint_limits as Record<string, unknown>)[name];
              if (Array.isArray(range) && range.length === 2) {
                parsed[name] = [asNumber(range[0], -180), asNumber(range[1], 180)];
              }
            }
            const key = JSON.stringify(parsed);
            if (key !== bridgeIkLimitsKeyRef.current) {
              bridgeIkLimitsKeyRef.current = key;
              setBridgeIkLimits(parsed);
            }
          }
        } else if (message.type === "status") {
          const Reply = ++StatusRepliesRef.current;
          if (message.ok && String(message.text).startsWith("Motion speed set to ")) return;
          setStatus(String(message.text ?? "Bridge status"));
          pushLog(message.ok ? "ok" : "warn", String(message.text ?? "Bridge status"));
          if (!message.ok && NeedsCalibration(String(message.text ?? ""))) SetHandGuideStale(true);
          // The bridge refused the motion a task is waiting on: nothing will move.
          const Approved = ApprovedCommandsRef.current;
          // A chained motion's leg counts too, even with no task left to continue.
          if (!message.ok && Approved && Reply >= Approved.First && Reply <= Approved.Last
              && AiSettleWaitRef.current && (aiTaskRef.current || AiSettleWaitRef.current.Chain)) {
            BridgeRefusedTaskMotionRef.current(String(message.text ?? "no reason given"));
          }
        }
      } catch {
        pushLog("warn", "Bridge sent an unreadable message");
      }
    };

    socket.onerror = () => {
      if (socketRef.current === socket) {
        ResetMotionSpeedControl();
        setSocketState("offline");
        setStatus("Bridge unavailable");
        pushLog("warn", `No bridge at ${BRIDGE_HOST}:8000`);
      }
    };

    socket.onclose = (Event) => {
      if (socketRef.current === socket) {
        socketRef.current = null;
        // Another device refused (the bridge's 44xx close): its "access" message said why; the code is the fallback.
        const Refused = AccessCloseCodes[Event.code];
        if (Refused && !ThisPc) {
          const Code = Refused === "password_wrong" && !StoredPassword() ? "password_required" : Refused;
          SetRemoteRefusal(Previous => Previous ?? { Code, Text: AccessText(Code) });
          if (!AccessCardDismissedRef.current) SetAccessCardOpen(true);
        }
        DisconnectedOnPurposeRef.current = null;
        SetDisconnectedOnPurpose(null);
        CadCompatibleRef.current = false;
        BasketCompatibleRef.current = false;
        GripperWithTipRef.current = false;
        SetGripperWithTip(false);
        // Unknown until the next bridge state says.
        MotionFaultRef.current = null;
        BridgeVersionRef.current = null;
        SetBridgeUpdate(null);
        ResetMotionSpeedControl();
        autoArmRef.current = null;
        setSocketState("offline");
        setArmConnected(false);
        setTorque(false);
        setBridgeJoints(null);
        setBridgeIkPreview(null);
        setStatus(emergencyDisconnectRef.current ? "Emergency shutoff" : Refused && !ThisPc ? "Bridge locked: this device needs the arm password"
          : "Local preview");
        emergencyDisconnectRef.current = false;
      }
    };
  };

  // Bring the whole stack up as one. The bridge connects first; the arm then
  // auto-connects (see the effect below) once the socket is live and a serial
  // port is known. There is intentionally no "connect bridge" vs "connect arm"
  // split anymore — it all comes up together.
  const connectAll = async () => {
    if (BridgeRestartingRef.current) return;
    ConnectClickedAtRef.current = performance.now();
    ManualDisconnectRef.current = false;
    try { localStorage.setItem("SO101.ManualDisconnect", "false"); } catch { /* Storage optional. */ }
    autoArmRef.current = null;
    await connectBridge();
    void loadPorts();
  };

  const DisconnectArm = () => {
    ConnectClickedAtRef.current = null;
    ManualDisconnectRef.current = true;
    try { localStorage.setItem("SO101.ManualDisconnect", "true"); } catch { /* Storage optional. */ }
    if (aiTaskRef.current) recordSessionEvent("ai_stopped", { reason: "arm disconnected", task: aiTaskRef.current.task });
    EndAiTask("the arm was disconnected.", "disconnect", { Advice: "Connect it again, then send the task again." });
    selfCalAbortRef.current = true;
    clearServoCommandTimers(); pendingServoTargetsRef.current = {};
    const Sent = sendCommand({ type: "disconnect" });
    pushLog(Sent ? "info" : "warn", Sent ? "Disconnect requested · automatic reconnection paused" : "Bridge unavailable; disconnect could not be confirmed");
  };

  // Reload the Python process only after a supported-arm disconnect. The server
  // independently checks actual hardware state before replacing the process.
  const restartBridge = async () => {
    if (BridgeRestartingRef.current) return;
    if (armConnected || torque) {
      const Reason = "Support the arm and click Disconnect before restarting the bridge; disconnecting releases torque.";
      setStatus(Reason); pushLog("warn", Reason); return;
    }
    BridgeRestartingRef.current = true; SetBridgeRestarting(true);
    ManualDisconnectRef.current = true;
    try { localStorage.setItem("SO101.ManualDisconnect", "true"); } catch { /* Optional persistence. */ }
    clearServoCommandTimers();
    pendingServoTargetsRef.current = {};
    setStatus("Restarting bridge");
    try {
      const Response = await fetch("/bridge-launch/restart", { method: "POST" });
      const Result = await Response.json();
      if (!Response.ok || !Result.ok) throw new Error(Result.error ?? "Bridge restart failed. Check the bridge log.");
      ResetMotionSpeedControl();
      const PreviousSocket = socketRef.current; socketRef.current = null; PreviousSocket?.close();
      autoArmRef.current = null;
      setSocketState("offline"); setArmConnected(false); setTorque(false); setBridgeJoints(null); setBridgeIkPreview(null);
      await connectBridge();
      pushLog("ok", "Updated bridge loaded. Connect the arm when ready; motion has not been resumed.");
      void loadPorts();
    } catch (Failure) {
      const Reason = Failure instanceof Error ? Failure.message : String(Failure);
      setStatus(Reason); pushLog("warn", Reason);
    } finally { BridgeRestartingRef.current = false; SetBridgeRestarting(false); }
  };

  useEffect(() => {
    let cancelled = false;
    const boot = async () => {
      await connectBridge();
      if (cancelled) return;
      void loadPorts();
      fetch(`${API_URL}/notepad`)
        .then((r) => r.json() as Promise<{ content?: string }>)
        .then((data) => { if (!cancelled && typeof data.content === "string") setAiNotepad(data.content); })
        .catch(() => {});
      // The declared surfaces' problems, the bridge's and this page's, said once at load (a surface with one is left out).
      fetch(`${API_URL}/surfaces`, { cache: "no-store" })
        .then((r) => r.json() as Promise<unknown>)
        .catch(() => null)
        .then((data) => {
          const Note = SurfaceProblemsNote(data);
          if (!cancelled && Note) pushLog("warn", Note);
        });
    };
    void boot();
    return () => {
      cancelled = true;
      clearServoCommandTimers();
      ClearMotionSpeedTimers();
      socketRef.current?.close();
    };
  }, []);

  // Auto-connect the arm as soon as the bridge is live and a serial port is
  // available, so the user never has to wire it up by hand — it "comes as one"
  // with the bridge. Each distinct port is attempted once per session (guarded
  // by autoArmRef) so a failed connect never loops. Only on this PC, and not after
  // anyone disconnected the arm on purpose: then, and on every other device, only a
  // Connect click connects it, sent as an explicit connect (the bridge refuses any
  // other; Aaran's decision 2, 2026-10-07 audit 2 M2).
  useEffect(() => {
    if (socketState !== "live" || armConnected || ManualDisconnectRef.current) return;
    const Clicked = ConnectClickedAtRef.current !== null && performance.now() - ConnectClickedAtRef.current < ConnectClickMs;
    if (!Clicked && (!ThisPc || DisconnectedOnPurpose)) return;
    const port = selectedPort || ports[0];
    if (!port || autoArmRef.current === port) return;
    autoArmRef.current = port;
    ConnectClickedAtRef.current = null;
    if (!selectedPort) setSelectedPort(port);
    setStatus(`Connecting arm on ${port}`);
    pushLog("info", `Connecting arm on ${port}`);
    sendCommand({ type: "connect", port, ...(Clicked ? { explicit: true } : {}) });
  }, [socketState, armConnected, ports, selectedPort, DisconnectedOnPurpose]);

  // Another device: the robot's PC refused one of this page's requests (lib/RemoteAccess.ts raises the event): ask
  // for the password, unless the user put the card away.
  useEffect(() => {
    if (ThisPc) return;
    const Refused = (Event: Event) => {
      const Problem = (Event as CustomEvent<AccessProblem>).detail;
      if (!Problem || Problem.Code === "cross_site") return;
      SetRemoteRefusal(Previous => Previous?.Code === Problem.Code && Previous.Text === Problem.Text ? Previous : Problem);
      if (!AccessCardDismissedRef.current) SetAccessCardOpen(true);
    };
    window.addEventListener(AccessRefusedEvent, Refused);
    return () => window.removeEventListener(AccessRefusedEvent, Refused);
  }, []);

  // The password card's actions. Unlocking stores the password in this browser and opens the bridge socket with it
  // if none is open; it never connects the arm (a Connect click from before it does not count either).
  const UnlockBridge = (Password: string | null) => {
    if (Password !== null) StorePassword(Password);
    ConnectClickedAtRef.current = null;
    AccessCardDismissedRef.current = false;
    SetAccessCardOpen(false);
    SetRemoteRefusal(null);
    const Socket = socketRef.current;
    if (!Socket || Socket.readyState === WebSocket.CLOSING || Socket.readyState === WebSocket.CLOSED) void connectBridge();
  };
  const DismissAccessCard = () => { AccessCardDismissedRef.current = true; SetAccessCardOpen(false); };
  const ForgetRemotePassword = () => {
    StorePassword(null);
    socketRef.current?.close();
    const Problem: AccessProblem = { Code: "password_required", Text: AccessText("password_required") };
    SetRemoteRefusal(Problem);
    AccessCardDismissedRef.current = false;
    SetSettingsOpen(false);
    SetAccessCardOpen(true);
    pushLog("info", "Arm password forgotten on this device");
  };

  const emergencyShutoff = () => {
    ConnectClickedAtRef.current = null;
    ManualDisconnectRef.current = true;
    try { localStorage.setItem("SO101.ManualDisconnect", "true"); } catch { /* Storage optional. */ }
    selfCalAbortRef.current = true;
    void fetch(`${API_URL}/emergency-stop`, { method: "POST" }).then(R => R.json()).then(R => {
      setStatus(R.ok ? "Emergency torque-off confirmed" : String(R.error ?? "Emergency stop unconfirmed"));
      if (!R.ok) pushLog("warn", String(R.error));
    }).catch(() => pushLog("warn", "Could not confirm emergency torque-off with the bridge"));
    emergencyDisconnectRef.current = true;
    clearServoCommandTimers();
    pendingServoTargetsRef.current = {};
    const sent = sendCommand({ type: "emergency_stop" });
    // Kill any autonomous task loop immediately, and its request in flight.
    EndAiTask("the emergency stop was pressed.", "emergency_stop", { Advice: "Check the arm, connect it again, then send a new task." });
    if (!sent) {
      emergencyDisconnectRef.current = false;
    }
    window.setTimeout(() => {
      if (emergencyDisconnectRef.current && socketRef.current) {
        socketRef.current.close();
      }
    }, 750);
    setTorque(false);
    setArmConnected(false);
    setBridgeJoints(null);
    setBridgeIkPreview(null);
    setServos((prev) => {
      const next = cloneServos(prev);
      for (const name of SERVO_ORDER) {
        next[name].targetRaw = next[name].presentRaw;
      }
      return next;
    });
    setStatus("Emergency shutoff");
    pushLog(
      "warn",
      sent ? "Emergency shutoff triggered" : "Emergency shutoff local only - bridge was offline",
    );
  };

  useEffect(() => {
    if (socketState === "live" && armConnected) return;

    const timer = window.setInterval(() => {
      setServos((prev) => {
        const next = cloneServos(prev);
        for (const name of SERVO_ORDER) {
          next[name] = stepPreviewServo(next[name], torque ? 18 : 6);
        }
        return ReuseServoSnapshot(prev, next);
      });
    }, FRAME_MS);
    return () => window.clearInterval(timer);
  }, [armConnected, socketState, torque]);

  const pumpPreviewRef = useRef<() => void>(() => undefined);
  pumpPreviewRef.current = () => {
    const latest = previewLatestRef.current;
    if (!latest || previewInFlightRef.current || !CadCompatibleRef.current) return;
    const { target: reqTarget, elbowMode: reqElbow, key } = latest;
    const requestId = previewRequestIdRef.current + 1;
    previewRequestIdRef.current = requestId;
    const generation = previewGenerationRef.current;
    previewInFlightRef.current = true;
    previewLastStartedAtRef.current = window.performance.now();
    fetch(`${API_URL}/ik/preview`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        x: reqTarget.x,
        y: reqTarget.y,
        z: reqTarget.z,
        wrist_pitch: reqTarget.wristPitch,
        elbow_mode: reqElbow,
      }),
    })
      .then((response) => response.json() as Promise<unknown>)
      .then((data) => {
        if (generation !== previewGenerationRef.current) return;
        if (requestId < previewAppliedIdRef.current) return;
        const preview = mapBackendPreview(data, key, reqTarget);
        if (!preview) return;
        previewAppliedIdRef.current = requestId;
        setBridgeIkPreview(preview);
      })
      .catch(() => {
        if (generation === previewGenerationRef.current && requestId >= previewAppliedIdRef.current) {
          setBridgeIkPreview(null);
        }
      })
      .finally(() => {
        previewInFlightRef.current = false;
        // A newer target arrived while this one was solving: request it now.
        const next = previewLatestRef.current;
        if (next && next.key !== key && generation === previewGenerationRef.current) {
          window.setTimeout(() => pumpPreviewRef.current(), 0);
        }
      });
  };

  useEffect(() => {
    if (socketState !== "live") {
      previewGenerationRef.current += 1;
      previewInFlightRef.current = false;
      previewLatestRef.current = null;
      setBridgeIkPreview(null);
      return;
    }

    previewLatestRef.current = { target, elbowMode, key: currentPreviewKey };
    const elapsedMs = window.performance.now() - previewLastStartedAtRef.current;
    const delayMs = Math.max(0, PREVIEW_THROTTLE_MS - elapsedMs);
    const timer = window.setTimeout(() => pumpPreviewRef.current(), delayMs);
    return () => {
      window.clearTimeout(timer);
    };
  }, [currentPreviewKey, elbowMode, previewRefreshNonce, socketState, target]);

  // Returns whether the move went to the bridge, so an approved gripper step
  // can wait for it like any other motion. Approved moves are Immediate.
  const updateServo = (name: ServoName, raw: number, Immediate = false, Scene?: SceneIdentity | null, Carry = false) => {
    if (socketState === "live" && !armConnected) {
      pushLog("warn", "Connect Arm before moving servos");
      setStatus("Arm not connected");
      return false;
    }
    setServos((prev) => ({
      ...prev,
      [name]: { ...prev[name], targetRaw: raw },
    }));
    const Queued = queueServoMove(name, raw, Immediate, Scene, Carry);
    if (!Queued && !Immediate) {
      setStatus(`Preview target set: ${name}`);
    }
    return Queued;
  };

  const updateServoLimits = (name: ServoName, limits: ServoLimitUpdate) => {
    setServos((prev) => {
      const servo = prev[name];
      const nextServo = {
        ...servo,
        min: limits.min,
        max: limits.max,
        arcInverted: limits.arcInverted,
      };
      return {
        ...prev,
        [name]: {
          ...nextServo,
          targetRaw: fractionToRaw(nextServo, rawToFraction(nextServo, nextServo.targetRaw)),
          presentRaw: fractionToRaw(nextServo, rawToFraction(nextServo, nextServo.presentRaw)),
          guiAngleRaw: nextServo.guiAngleRaw == null
            ? null
            : fractionToRaw(nextServo, rawToFraction(nextServo, nextServo.guiAngleRaw)),
        },
      };
    });
    setBridgeIkPreview(null);
    setPreviewRefreshNonce((value) => value + 1);

    if (sendCommand({
      type: "set_servo_limits",
      name,
      min: limits.min,
      max: limits.max,
      arc_inverted: limits.arcInverted,
    })) {
      pushLog("info", `${name.replace(/_/g, " ")} limits saved`);
    } else {
      pushLog("info", `${name.replace(/_/g, " ")} limits changed in preview`);
    }
  };

  const updateServoGuiAngle = (name: ServoName, raw: number | null) => {
    setServos((prev) => {
      const servo = prev[name];
      return {
        ...prev,
        [name]: {
          ...servo,
          guiAngleRaw: raw == null ? null : clampRaw(raw),
        },
      };
    });
    setBridgeIkPreview(null);
    setPreviewRefreshNonce((value) => value + 1);

    const sent = sendCommand({ type: "set_gui_angle", name, raw });
    const label = name.replace(/_/g, " ");
    if (raw == null) {
      pushLog(sent ? "ok" : "info", sent ? `${label} current angle cleared` : `${label} current angle cleared in preview`);
    } else {
      pushLog(sent ? "ok" : "info", sent ? `${label} current angle set` : `${label} current angle set in preview`);
    }
  };

  // A chained motion's second leg waits on the first (lib/AiChain.ts): a manual
  // Coast, Hold or Home meanwhile ends the task, so it is never sent from
  // wherever that leaves the arm.
  const EndPendingChain = (Control: string) => {
    if (!AiSettleWaitRef.current?.Chain?.Next) return;
    EndAiTask(`${Control} was pressed while the first part of a chained motion ran.`, "chain_interrupted",
      { Advice: "Check where the arm is, then send the task again." });
  };

  const goHome = () => {
    EndPendingChain("Home");
    // Snap the IK target pose to home too, so the preview, Send, and the AI's
    // "current target" agree with where the arm is going instead of pointing at
    // the stale pre-Home target.
    updateTarget({ ...HOME_MODEL_TARGET_CM });
    // The bridge brings every joint Home, the wrist roll and the gripper included (the gripper closes only once
    // the arm has arrived, and an object it holds is kept); without it the preview shows the whole saved pose.
    if (sendCommand({ type: "go_home" })) return;
    setServos((prev) => {
      const next = cloneServos(prev);
      for (const name of SERVO_ORDER) next[name].targetRaw = HOME_RAW[name];
      return next;
    });
    setStatus("Home preview");
    pushLog("info", "Home pose loaded in preview");
  };

  const setHome = () => {
    if (!sendCommand({ type: "set_home" })) {
      pushLog("info", "Home save is available through the bridge");
    }
  };

  const toggleTorque = () => {
    if (BridgeRestartingRef.current) return;
    if (socketState === "live" && !armConnected) {
      pushLog("warn", "Connect Arm before toggling torque");
      setStatus("Arm not connected");
      return;
    }
    EndPendingChain(torque ? "Coast" : "Hold");
    // The button's meaning, not a toggle: the bridge runs it ahead of a route still planning (2026-10-07 audit M2),
    // and a second press cannot undo the first.
    if (sendCommand({ type: "toggle_torque", enable: !torque })) return;
    setTorque((value) => !value);
    setStatus(torque ? "Coast preview" : "Hold preview");
  };

  // GripperRaw: a gripper target the route opens the jaws to on the way (a chain run together);
  // the move is then previewed with the jaws there, as it ends.
  const sendTargetPose = (pose: TargetPose, source = "IK command", GripperRaw: number | null = null, Scene?: SceneIdentity | null, Carry = false) => {
    const SceneIssue = SceneFreshnessIssue(Scene, LatestSceneRef.current);
    if (SceneIssue) { setStatus(SceneIssue); pushLog("warn", SceneIssue); return false; }
    if (TargetEditRef.current) { setStatus("Finish placing the target before sending"); return false; }
    const Preview = solveIkPreview(pose, elbowMode, GripperRaw == null ? servosRef.current : servosWithGripper(servosRef.current, GripperRaw),
      bridgeIkLimits ?? undefined);
    if (Preview.collisionReason) { setStatus(Preview.collisionReason); pushLog("warn", Preview.collisionReason); return false; }
    if (!Preview.reachable) { setStatus("Target is outside the reachable workspace"); return false; }
    const BlockReason = CommandBlockReason({ type: "send_ik" }, { SocketOpen: socketRef.current?.readyState === WebSocket.OPEN,
      BasketCompatible: BasketCompatibleRef.current, CadCompatible: CadCompatibleRef.current,
      SpeedCompatible: SpeedCompatibleRef.current, Calibrating: selfCalRunningRef.current, Restarting: BridgeRestartingRef.current,
      HandGuiding: HandGuideRef.current.On || HandGuideRef.current.Calibrating });
    if (BlockReason) { setStatus(BlockReason); pushLog("warn", BlockReason); return false; }
    if (socketState === "live" && !armConnected) {
      setStatus("Arm not connected");
      pushLog("warn", "Pick a port and click Connect Arm first");
      return false;
    }
    if (socketState === "live" && !torque) {
      setStatus("Enable torque first");
      pushLog("warn", "Hold/torque must be on before Send");
      return false;
    }
    if (sendCommand(ikCommand(pose, GripperRaw, Scene, Carry))) {
      pushLog("info", `${source} sent to bridge${GripperRaw == null ? "" : `, opening the gripper to raw ${GripperRaw} on the way`}`);
      return true;
    }
    const Reason = CommandFailureRef.current ?? "The command was not sent. Check the bridge status and try again.";
    setStatus(Reason); pushLog("warn", Reason);
    return false;
  };

  const CaptureCadReference = async () => {
    try {
      if (!CadCompatibleRef.current) throw new Error("Restart the bridge to load CAD alignment support.");
      const Response = await fetch(`${API_URL}/cad/alignment`, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ confirmed: true }) });
      const Result = await Response.json();
      if (!Result.ok || !SetCadAlignment(Result.alignment)) throw new Error(Result.error ?? "Could not record CAD alignment");
      RefreshCadFrame(); SetCadRevision(GetCadAlignment().Version); setBridgeIkPreview(null); setBridgeJoints(null);
      const CurrentAngles = currentAngles(servosRef.current);
      const Tip = forwardKinematics(CurrentAngles).slice(-1)[0];
      setTarget({ ...Tip, wristPitch: wristPitchFromAngles(CurrentAngles) });
      pushLog("ok", "CAD reference recorded. Check workspace and camera alignment before using Cartesian motion.");
      return true;
    } catch (Error) { pushLog("warn", String(Error)); return false; }
  };

  const sendTarget = () => {
    sendTargetPose(target);
  };

  // The Motion panel's hand-guide button (lib/HandGuide.ts HandGuideButton): its first use in a session says what it does
  // and that faults and the emergency stop make the arm go limp; a calibration always asks first.
  const PressHandGuide = (Button: HandGuideButtonView) => {
    if (!Button.Command || Button.DisabledReason) { if (Button.DisabledReason) setStatus(Button.DisabledReason); return; }
    if (Button.Confirm) {
      const Calibrate = Button.Command.type === "hand_guide_calibrate";
      let Explained = false;
      try { Explained = sessionStorage.getItem(FirstUseKey) === "true"; } catch { /* Storage optional. */ }
      if ((Calibrate || !Explained) && !window.confirm(Button.Confirm)) return;
      if (!Calibrate) { try { sessionStorage.setItem(FirstUseKey, "true"); } catch { /* Storage optional. */ } }
    }
    clearServoCommandTimers();
    pendingServoTargetsRef.current = {};
    if (sendCommand(Button.Command)) pushLog("info", `${Button.Label} sent to the bridge`);
  };
  const RecalibrateHandGuide = () => PressHandGuide({ ...HandGuideButton({ ...HandGuideRef.current, Calibrated: false }, HandGuideContext()),
    Label: "Calibrate hand guiding again" });

  const StopAndHold = () => {
    selfCalAbortRef.current = true;
    clearServoCommandTimers();
    pendingServoTargetsRef.current = {};
    // Over the socket when it is open (the bridge runs it ahead of any queued command), else over POST /motion, as the
    // emergency stop and the calibration hold do: a press while the socket reconnects only set a status, and the bridge
    // keeps a route running when its socket drops (2026-10-08 safety review).
    const Sent = sendCommand({ type: "stop_motion" });
    if (!Sent) {
      void fetch(`${API_URL}/motion`, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ type: "stop_motion" }) })
        .then(Response => Response.json()).then(Result => {
          setStatus(Result.ok ? "Stopped and holding position (sent over HTTP: the bridge socket was offline)"
            : `Stop and hold not confirmed: ${String(Result.error ?? "the bridge refused it")}`);
          if (Result.ok) pushLog("info", "Stop and hold sent over HTTP: the bridge socket was offline");
          else pushLog("warn", `Stop and hold over HTTP: ${String(Result.error ?? "refused")}`);
        }).catch(() => {
          setStatus("Stop and hold could not reach the bridge: its socket and HTTP are both offline. Use the emergency stop.");
          pushLog("warn", "Stop and hold could not reach the bridge over the socket or HTTP");
        });
    }
    EndAiTask("Stop and hold was pressed.", "stop_and_hold", { Advice: "Send a new task when ready." });
    setStatus(Sent ? "Stopping and holding position" : "Stopping and holding position (over HTTP: the bridge socket is offline)");
  };

  const toggleElbow = () => {
    const next = elbowMode === "down" ? "up" : "down";
    setElbowMode(next);
    sendCommand({ type: "toggle_elbow" });
  };

  const setCurrentTarget = () => {
    // Prefer the bridge's live joints — the same data the 3D view renders. The
    // local FK's zero offsets can disagree with the bridge's, which used to land
    // the target visibly away from the on-screen tip when the arm was connected.
    const liveTip = socketState === "live" && bridgeJoints && bridgeJoints.length
      ? bridgeJoints[bridgeJoints.length - 1]
      : null;
    const modelJoints = forwardKinematics(angles);
    const tip = liveTip
      ? modelPointFromDisplayPoint(liveTip)
      : modelJoints[modelJoints.length - 1];
    updateTarget((prev) => ({
      ...prev,
      x: Number(tip.x.toFixed(3)),
      y: Number(tip.y.toFixed(3)),
      z: Number(tip.z.toFixed(3)),
      // Match the wrist too, so "Set Current" captures the whole current pose.
      wristPitch: Number(wristPitchFromAngles(angles).toFixed(1)),
    }));
    pushLog("info", liveTip ? "Target matched to live arm tip" : "Target matched to preview tip");
  };

  // A task that acts on objects never runs on unmeasured or unverified
  // positions: it throws CameraMovedError instead of falling back to text.
  // Signal is the task's request: when the task ends (or /locate runs out of
  // time) the capture stops and throws, whatever step it is on. OnFrame runs
  // at the moment the frame is taken. TaskObject goes to /locate with the frame.
  const captureVisionAttachment = async (
    Signal: AbortSignal,
    options?: { skipCountdown?: boolean; needsPositions?: boolean; OnFrame?: () => void; TaskObject?: TaskObjectQuery | null },
  ): Promise<VisionAttachment | null> => {
    const NeedsPositions = Boolean(options?.needsPositions);
    const Unmeasured = (Title: string) => new CameraMovedError({ Code: "camera_unavailable", Title, Detail: "" });
    if (!(await waitForCamera())) {
      if (NeedsPositions) throw Unmeasured("The camera is not available, so object positions cannot be measured: allow camera access, then send the task again.");
      pushLog("warn", "Camera unavailable; sending text only");
      return null;
    }

    try {
      // Auto task check-ins skip the countdown: it exists so a human can arrange
      // the scene, which doesn't apply when the loop re-captures on its own.
      if (!options?.skipCountdown && CAPTURE_COUNTDOWN_SECONDS > 0) {
        for (let remaining = CAPTURE_COUNTDOWN_SECONDS; remaining > 0; remaining -= 1) {
          Signal.throwIfAborted();
          setCaptureCountdown(remaining);
          setAiStatus(`Capturing frame in ${remaining}`);
          await new Promise((resolve) => window.setTimeout(resolve, 1000));
        }
      }
      Signal.throwIfAborted();
      setCaptureCountdown(null);
      setAiStatus("Capturing camera frame");
      // One capture, one mirroring for everything it is sent with.
      const mirrored = CameraMirroredRef.current;
      const originalImageUrl = captureVideoFrame(videoRef.current, mirrored);
      // Read with the capture, before any await: the depth frame of these pixels.
      const depthSeq = DepthSequenceFor(videoRef.current);
      if (!originalImageUrl) {
        if (NeedsPositions) throw Unmeasured("No camera frame could be captured, so object positions cannot be measured: check the camera, then send the task again.");
        pushLog("warn", "Could not capture camera frame; sending text only");
        return null;
      }
      options?.OnFrame?.();
      archiveCapture(originalImageUrl, "vision", mirrored);

      // Calibrated object measurement (bridge /locate). The bridge first checks
      // the camera pose against the calibration; a moved camera stops here,
      // before any model call, because every position it measures is wrong.
      setAiStatus("Measuring object positions");
      const TaskObject = options?.TaskObject ?? null;
      const CarryScene = TaskGripperClosedRef.current && (GripperGraspHoldRef.current
        || rawToFraction(servosRef.current.gripper, servosRef.current.gripper.presentRaw) >= HeldFraction) ? LatestSceneRef.current : null;
      const located = await WithTimeout(LocateSignal => locateObjects(originalImageUrl, depthSeq, aiModel, mirrored, LocateSignal, TaskObject, CarryScene),
        Signal, LocateTimeoutMs, "the object measurement (bridge /locate)");
      Signal.throwIfAborted();
      const PreviousScene = LatestSceneRef.current;
      LatestSceneRef.current = located?.scene ?? null;
      if (PreviousScene?.revision !== located?.scene?.revision) ArmClearRef.current.clear();
      recordSessionEvent("vision", { camera_source: depthSeq != null ? "depth" : "webcam", depth_seq: depthSeq, mirrored,
        measured_objects: located?.objects ?? null, measurement_error: located?.error ?? null,
        error_code: located?.errorCode ?? null, scene_note: located?.sceneNote ?? null, camera_check: located?.cameraCheck ?? null, withheld: located?.withheld ?? null,
        calibration_rms_cm: located?.rmsCm ?? null, task_object: TaskObject ? { phrase: TaskObject.Phrase, label: TaskObject.Name } : null });
      CameraVerifiedRef.current = located?.cameraCheck?.Ok === true && !located.error;
      const Gate = CameraGateFailure(located, NeedsPositions, depthSeq != null ? "depth" : "webcam");
      if (Gate) throw new CameraMovedError(Gate);
      if (CameraVerifiedRef.current) SetCameraGateBanner(null);
      const DeskNote = DeskRemeasuredNote(located?.cameraCheck ?? null);
      if (DeskNote) pushLog("info", DeskNote);
      // No desk in view: the gripper tip verified the camera, or it is assumed where it was calibrated (a warning).
      const RobotNote = RobotCheckNote(located?.cameraCheck ?? null);
      if (RobotNote) pushLog(RobotNote.Level, RobotNote.Text);
      if (!located || located.error) pushLog("warn", `No measured object positions (${compactOutputText(located?.error ?? "the bridge did not answer", 90)}): the AI is estimating from the image`);
      if (located && located.objects.length) {
        const depthCount = located.objects.filter(object => object.depthMeasured).length;
        pushLog("ok", `Measured ${located.objects.length} object position(s) via calibrated camera${depthCount ? `, ${depthCount} in 3D from depth` : ""}`);
      }

      // The depth map is a bonus, not a requirement. If the depth server is
      // unreachable we STILL send the color photo so the vision model can see
      // and describe the scene — only the distance refinement is lost.
      try {
        setAiStatus("Creating depth image");
        const withDepth = await createDepthAttachment(originalImageUrl, depthSeq, mirrored, Signal);
        if (withDepth) {
          pushLog("ok", "Color + depth images attached");
          return { ...withDepth, located };
        }
      } catch (depthError) {
        if (Signal.aborted) throw depthError;
        const text = depthError instanceof Error ? depthError.message : String(depthError);
        pushLog("warn", `Depth unavailable, sending color image only: ${compactOutputText(text)}`);
      }

      pushLog("ok", "Camera image attached (color only, no depth)");
      return {
        originalImageUrl,
        depthImageUrl: null,
        summary: [
          "Attached vision image:",
          "1. A live color photo of the workspace from the fixed camera.",
          "No depth map is available this time, so estimate distances and directions from the photo alone.",
        ].join("\n"),
        located,
      };
    } catch (error) {
      // An ended task or a timeout is not a capture failure: the caller ends the task.
      if (error instanceof CameraMovedError || error instanceof AiStepTimeoutError || Signal.aborted) throw error;
      const text = error instanceof Error ? error.message : String(error);
      if (NeedsPositions) throw Unmeasured(`Vision capture failed, so object positions cannot be measured: ${compactOutputText(text, 90)}`);
      pushLog("warn", `Vision capture failed; sending text only: ${compactOutputText(text)}`);
      return null;
    } finally {
      setCaptureCountdown(null);
    }
  };

  // --- Camera calibration (probe-by-touch, homography fit on the bridge) ----

  const refreshCalStatus = async () => {
    try {
      const response = await fetch(`${API_URL}/calibration/status`);
      if (response.ok) {
        setCalStatus(await response.json() as CalibrationStatus);
        return;
      }
    } catch {
      // Bridge unreachable — leave status null so the UI says so.
    }
    setCalStatus(null);
  };

  const freezeCalFrame = () => {
    const video = videoRef.current;
    const url = captureVideoFrame(video, CameraMirroredRef.current);
    if (!url || !video) {
      setCalFrame(null);
      setCalMessage("Camera unavailable: allow camera access, then Refreeze.");
      return;
    }
    setCalFrame({ url, width: video.videoWidth, height: video.videoHeight, mirrored: CameraMirroredRef.current });
    setCalMessage("Touch the gripper tip to the table, then click that exact tip pixel below.");
  };

  const openCalibration = () => {
    setCalOpen(true);
    void refreshCalStatus();
    freezeCalFrame();
  };
  const OpenCalibrationRef = useRef(openCalibration); OpenCalibrationRef.current = openCalibration;
  const [CameraReconnectKey, SetCameraReconnectKey] = useState(0);

  useEffect(() => {
    let Cancelled = false;
    const Poll = async () => {
      try {
        const Response = await fetch(`${API_URL}/depthcam/status`, { cache: "no-store", signal: AbortSignal.timeout(2500) });
        const Data = await Response.json() as unknown;
        if (!Cancelled) setDepthCamera(readDepthCameraStatus(Data));
      } catch {
        if (!Cancelled) setDepthCamera(null);
      }
    };
    void Poll();
    const Timer = window.setInterval(() => void Poll(), 3000);
    return () => { Cancelled = true; window.clearInterval(Timer); };
  }, []);

  // Auto uses the depth camera whenever the bridge streams it, and only falls
  // back to a webcam after three missed status checks, so a bridge restart
  // does not bounce the view. Never switch mid-calibration or mid-recording.
  const CameraLocked = selfCal.running || taskRecording;
  useEffect(() => {
    if (CameraLocked) return;
    const Setting = DisplaySettings.CameraSource;
    if (Setting !== "Auto") { setCameraSource(Setting === "Depth" ? "depth" : "webcam"); return; }
    if (depthCamera === undefined) return;
    if (depthCamera?.available) { DepthCameraMissesRef.current = 0; setCameraSource("depth"); return; }
    DepthCameraMissesRef.current += 1;
    setCameraSource(Previous => Previous === "depth" && DepthCameraMissesRef.current < 3 ? "depth" : "webcam");
  }, [depthCamera, DisplaySettings.CameraSource, CameraLocked]);

  const depthCameraText = depthCamera === undefined ? "Depth camera: checking"
    : depthCamera === null ? "Depth camera: unavailable (bridge offline or without depth support)"
    : !depthCamera.available ? `Depth camera: not streaming${depthCamera.error ? ` (${depthCamera.error})` : ""}`
    : [`Depth camera: ${depthCamera.name || "connected"}, ${depthCamera.fps.toFixed(0)} fps`,
      depthCamera.calibrated ? "calibrated" : "not calibrated yet: run automatic calibration",
      ...depthCamera.warnings].join(" · ");
  const OpenFloatingCalibration = useCallback(() => OpenCalibrationRef.current(), []);
  const ToggleCameraPopout = async () => {
    try {
      if (document.pictureInPictureElement) await document.exitPictureInPicture();
      else await videoRef.current?.requestPictureInPicture();
    } catch { pushLog("warn", "Camera pop-out unavailable. The live window can still be moved inside the dashboard."); }
  };

  const calRequest = async (path: string) => {
    setCalBusy(true);
    try {
      const response = await fetch(`${API_URL}${path}`, { method: "POST" });
      const data = await response.json() as Record<string, unknown>;
      return data;
    } catch {
      setCalMessage("Bridge unreachable.");
      return null;
    } finally {
      setCalBusy(false);
      void refreshCalStatus();
    }
  };

  const onCalFrameClick = async (event: ReactMouseEvent<HTMLImageElement>) => {
    if (!calFrame || calBusy) return;
    if (socketState !== "live" || !armConnected) {
      setCalMessage("Connect the arm first: each probe pairs the clicked pixel with the real tip position.");
      return;
    }
    const tip = joints[joints.length - 1];
    if (!tip) {
      setCalMessage("No arm position available yet.");
      return;
    }
    // Convert the click (CSS px on the scaled <img>) to capture pixels with the
    // same mapping as the hover magnifier (lib/ImagePointer.ts): inside the
    // border, at the decoded image's own size. The calibration image is
    // DISPLAYED un-mirrored (scaleX(-1) cancels the capture flip so it matches
    // the live feed), but probe pixels must live in the same flipped space as
    // /locate captures, so the mapping mirrors the clicked U back.
    const Image = event.currentTarget;
    const Point = CalFramePointRef.current ?? ImagePointAt(Image, event.clientX, event.clientY, calFrame.mirrored);
    if (!Point) return;
    const { U: u, V: v } = ClickCapturePoint(Point);
    const ImageWidth = Image.naturalWidth, ImageHeight = Image.naturalHeight;
    setCalBusy(true);
    try {
      const response = await fetch(`${API_URL}/calibration/probe`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          u, v,
          x: tip.x, y: tip.y, z: tip.z,
          image_size: [ImageWidth, ImageHeight],
          mirrored: calFrame.mirrored,
        }),
      });
      const data = await response.json() as Record<string, unknown>;
      setCalMessage(data.ok
        ? `Point ${String(data.point_count)} recorded (tip X=${tip.x.toFixed(1)}, Y=${tip.y.toFixed(1)}). Move the arm to a NEW spot, Refreeze, click again.`
        : String(data.error ?? "Probe failed."));
    } catch {
      setCalMessage("Bridge unreachable.");
    } finally {
      setCalBusy(false);
      void refreshCalStatus();
    }
  };

  // Wait until every IK joint is at its target, then a short dwell for the arm
  // to stop wobbling. A timeout aborts the run before images are captured.
  // A detour route starts at its first waypoint's targets and only reaches the
  // pose's own targets at its last stage, so "settled" means: state from after
  // this move (MoveEpoch), the whole route finished, nothing moving, every
  // joint on target, and the targets unchanged through the dwell.
  const waitForSelfCalSettle = async (MoveEpoch: number, Stages: number) => {
    const started = performance.now();
    let settledSince: number | null = null;
    let settledTargets = "";
    while (performance.now() - started < MotionTimeoutRef.current * Math.max(1, Stages)) {
      if (selfCalAbortRef.current || !armLiveRef.current) return false;
      if (!torqueRef.current || !MotionHealthRef.current || performance.now() - LastStateAtRef.current > 700) {
        throw new Error("Joint feedback, torque, or bridge heartbeat was lost.");
      }
      await sleep(100);
      if (performance.now() - started < SELFCAL_SETTLE_MIN_MS) continue;
      const current = servosRef.current;
      const targets = IK_JOINTS.map((name) => current[name].targetRaw).join(",");
      const settled = LastMotionEpochRef.current >= MoveEpoch && !RouteActiveRef.current && !MovingRef.current
        && Object.entries(ExpectedTargetsRef.current).every(([Name, Raw]) => current[Name as ServoName].targetRaw === Raw) && IK_JOINTS.every(
        (name) => circularRawDelta(current[name].presentRaw, current[name].targetRaw) <= TASK_SETTLE_RAW_TOLERANCE,
      );
      if (!settled || (settledSince !== null && targets !== settledTargets)) {
        settledSince = null;
      } else if (settledSince === null) {
        settledSince = performance.now();
        settledTargets = targets;
      } else if (performance.now() - settledSince >= SELFCAL_DWELL_MS) {
        return true;
      }
    }
    return false;
  };

  const holdSelfCalInPlace = () => {
    if (!SelfCalStopPromiseRef.current) {
      SelfCalStopPromiseRef.current = (async () => {
        const Response = await fetch(`${API_URL}/motion`, { method: "POST", headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ type: "stop_motion" }) });
        const Result = await Response.json();
        if (!Response.ok || !Result.ok) throw new Error(Result.error ?? "The bridge did not confirm Hold.");
      })();
    }
    return SelfCalStopPromiseRef.current;
  };

  // Semi-automatic calibration: after each pose's capture the arm holds the pose (torque on,
  // as between automatic poses) until the user clicks its frozen frame, skips it, asks for a
  // retake, or ends the run. Whatever ends a settle ends this wait too: Stop, a fault, lost
  // feedback, torque or heartbeat. A user who walks away ends the run after SemiCalWaitLimitMs,
  // and a servo nearing the protection limit (SemiCalHotC) ends it so the arm can be rested.
  // Returns null when the run was cancelled (the caller then throws the cancel reason).
  const waitForSemiCalDecision = async (): Promise<SemiCalDecision | null> => {
    const Started = performance.now();
    for (;;) {
      if (selfCalAbortRef.current) return null;
      if (!armLiveRef.current) throw new Error("The arm disconnected.");
      if (!torqueRef.current || !MotionHealthRef.current || performance.now() - LastStateAtRef.current > 700) {
        throw new Error("Joint feedback, torque, or bridge heartbeat was lost.");
      }
      const Decision = SemiCal.TakeDecision();
      if (Decision) return Decision;
      if (performance.now() - Started > SemiCalWaitLimitMs) {
        SemiCal.SetNotice(`No click for ${Math.round(SemiCalWaitLimitMs / 60000)} minutes: the run ended here and the arm holds this pose.`);
        pushLog("warn", "Semi-automatic calibration: no click in time, run ended early");
        return "done";
      }
      if (HottestRef.current !== null && HottestRef.current >= SemiCalHotC) {
        SemiCal.SetNotice(`A servo reached ${HottestRef.current} °C: the run ended here. Move the arm to rest and let it cool.`);
        pushLog("warn", `Semi-automatic calibration ended early: a servo reached ${HottestRef.current} °C`);
        return "done";
      }
      await sleep(100);
    }
  };

  // A fit report as the log, the step line and the session record keep it. Every computed camera is
  // saved and applied by the bridge; its quality problems are warnings, and only the user rejects it.
  const reportCalibrationOutcome = (Report: SelfCalReport, Source: "auto" | "manual") => {
    const Outcome = CalibrationOutcomeOf(Report);
    const From = Source === "manual" ? `${Report.observations_used ?? "?"} clicks` : `${Report.observations_used ?? "?"} poses`;
    if (Outcome.Kind === "saved") {
      pushLog("ok", `Camera calibrated from ${From}: about ${Report.plane_rms_cm?.toFixed(2) ?? "?"} cm on the table`);
    } else if (Outcome.Kind === "saved_with_warnings") {
      pushLog("warn", `Camera calibration saved and applied from ${From}, with warnings: ${Outcome.Problems.join("; ")}. Reject it in the calibration panel to restore the previous one.`);
    } else {
      pushLog("warn", `${Outcome.Title} ${Outcome.Detail ?? ""}`.trim());
    }
    recordSessionEvent(Outcome.Kind === "not_computed" ? "calibration_not_computed" : Outcome.Kind === "not_saved" ? "calibration_not_saved" : "calibration_saved", {
      source: Source, outcome: Outcome.RunOutcome, fitted_at: Report.fitted_at ?? null, problems: Outcome.Problems,
      warnings: Report.warnings ?? [], error: Report.error ?? null, drift_baseline: Report.drift_baseline ?? null,
      fit_rms_cm: Report.fit_rms_cm ?? null, holdout_rms_cm: Report.holdout_rms_cm ?? null, plane_rms_cm: Report.plane_rms_cm ?? null,
      observations_used: Report.observations_used ?? null });
    return Outcome;
  };

  // Fit the user's clicks (semi-automatic calibration). The bridge saves and applies every fit it
  // can compute (quality problems are warnings), and archives the report in the run as for an automatic run.
  const fitSemiCalibration = async () => {
    SemiCal.Fitting();
    try {
      const Response = await fetch(`${API_URL}/selfcal/solve`, {
        method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ source: "manual" }),
      });
      const Report = await Response.json() as SemiCalReport;
      if (!Response.ok) throw new Error(Report.error ?? `The bridge answered ${Response.status}.`);
      SemiCal.SetReport(Report);
      const Outcome = reportCalibrationOutcome(Report as SelfCalReport, "manual");
      setSelfCal((prev) => ({ ...prev, report: Report as SelfCalReport, step: Outcome.Kind === "saved" ? "Calibrated from your clicks"
        : Outcome.Kind === "saved_with_warnings" ? "Calibrated from your clicks, with warnings" : Outcome.Title }));
      return Report;
    } catch (Failure) {
      const Reason = Failure instanceof Error ? Failure.message : String(Failure);
      SemiCal.End(`Fit failed: ${Reason}`);
      pushLog("warn", `Semi-automatic fit failed: ${Reason}`);
      return null;
    } finally {
      void refreshCalStatus();
    }
  };

  const runSelfCalibration = async (Mode: "auto" | "semi" = "auto") => {
    if (selfCalRunningRef.current) return;
    const Semi = Mode === "semi";
    const fail = (step: string) => setSelfCal((prev) => ({ ...prev, step }));
    if (!CadCompatibleRef.current) { fail("Restart the bridge to load the CAD solver."); return; }
    if (!BasketCompatibleRef.current) { fail("Restart the bridge to enable basket avoidance."); return; }
    const CheckCancelled = () => {
      if (selfCalAbortRef.current) throw new Error(selfCalFaultRef.current ?? "Calibration cancelled by user.");
    };
    if (socketState !== "live" || !armConnected) {
      fail("Connect the arm first.");
      return;
    }
    if (!cameraActive || !videoRef.current) {
      fail("The camera is not available. Allow camera access, then try again.");
      return;
    }
    if (Semi) {
      // Before anything moves: a bridge without the click endpoints could not record the clicks.
      try {
        const Support = await fetch(`${API_URL}/selfcal/clicks`);
        if (!Support.ok) { fail("Restart the bridge to load semi-automatic calibration."); return; }
      } catch {
        fail("The bridge is unreachable.");
        return;
      }
      if (selfCalRunningRef.current) return;
    }
    EndAiTask(Semi ? "Semi-automatic calibration started." : "Automatic calibration started.", "self_calibration",
      { Advice: "Send the task again when calibration has finished." });
    clearServoCommandTimers();
    pendingServoTargetsRef.current = {};
    selfCalAbortRef.current = false;
    selfCalFaultRef.current = null;
    SelfCalPreparingRef.current = true;
    SelfCalPrepareEpochRef.current = Infinity;
    SelfCalStopPromiseRef.current = null;
    SelfCalMoveRequestedRef.current = false;
    selfCalRunningRef.current = true;
    setSelfCal({ running: true, step: "Preparing", index: 0, total: 0, seen: 0, report: null });
    let ArchiveStarted = false;
    let RunOutcome = "interrupted";
    try {
      const PrepareResponse = await fetch(`${API_URL}/selfcal/prepare`, { method: "POST" });
      if (PrepareResponse.status === 404) throw new Error("Restart the bridge to load the automatic calibration fix.");
      const Prepared = await PrepareResponse.json();
      if (!PrepareResponse.ok || !Prepared.ok) throw new Error(Prepared.error ?? "Calibration preparation failed.");
      if (Prepared.archive_version !== 2) throw new Error("Restart the bridge to load complete calibration recording before starting.");
      if (!Number.isFinite(Prepared.state?.motion_epoch)) throw new Error("The bridge returned no calibration state revision.");
      SelfCalPrepareEpochRef.current = Prepared.state.motion_epoch;
      const PrepareStarted = performance.now();
      while (LastMotionEpochRef.current < SelfCalPrepareEpochRef.current && performance.now() - PrepareStarted < 3000) {
        CheckCancelled();
        await sleep(50);
      }
      SelfCalPreparingRef.current = false;
      CheckCancelled();
      if (LastMotionEpochRef.current < SelfCalPrepareEpochRef.current || performance.now() - LastStateAtRef.current > 700) {
        throw new Error("No fresh bridge state arrived after calibration preparation.");
      }
      const Reset = await (await fetch(`${API_URL}/selfcal/reset`, { method: "POST" })).json();
      if (!Reset.ok) throw new Error(Reset.error ?? "Could not create the calibration archive.");
      if (Reset.archive_version !== 2) throw new Error("Restart the bridge to load complete calibration recording before starting.");
      ArchiveStarted = true;
      // A new run archives elsewhere, so an earlier run's frozen frames can no longer be clicked.
      SemiCal.Clear();
      CheckCancelled();
      const SelfCalMirrored = CameraMirroredRef.current;
      const PreflightImage = captureVideoFrame(videoRef.current, SelfCalMirrored);
      const PreflightSeq = DepthSequenceFor(videoRef.current);
      if (!PreflightImage) throw new Error("No camera frame could be captured.");
      const Preflight = await (await fetch(`${API_URL}/selfcal/check`, {
        method: "POST", headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ images: [PreflightImage], mirrored: SelfCalMirrored, depth_seqs: [PreflightSeq],
          hint: RedHintRef.current, initialize: true }),
      })).json();
      if (Preflight.preview) SetRedPreview(Preflight.preview);
      // Semi-automatic: the user clicks the tape at every pose, so the detector need not find it now.
      if (!Preflight.detected && !Semi) throw new Error(Preflight.reason ?? "Select the red tip tape using Check marker first.");
      CheckCancelled();
      setSelfCal((prev) => ({ ...prev, step: "Choosing calibration positions (about 15 s)" }));
      // A camera behind the arm needs the wrist tilted up to see the tape; one facing it does not.
      const posesResponse = await (await fetch(`${API_URL}/selfcal/poses?view=${SelfCalMirrored ? "Facing" : "Behind"}`)).json() as {
        ok?: boolean; poses?: SelfCalPose[]; error?: string;
        sampling?: { preferred_y_range_cm: number[]; preferred_poses: number };
      };
      if (!posesResponse.ok || !posesResponse.poses?.length) {
        throw new Error(posesResponse.error ?? "The bridge returned no calibration poses.");
      }
      const poses = posesResponse.poses;
      const Sampling = posesResponse.sampling;
      // The bridge no longer filters poses by shoulder load (removed 2026-10-03): none on the far desk means none
      // there was reachable and clear of the cart, mast and head.
      if (Sampling && Sampling.preferred_poses === 0) {
        pushLog("warn", `No safe calibration poses at Y = ${Sampling.preferred_y_range_cm.join(" to ")} cm: none there is reachable and clear. Continuing with the other spaced poses.`);
      }
      CheckCancelled();
      if (!torqueRef.current) {
        CheckCancelled();
        // Hold, as a state (enable): a bare toggle coasted a holding arm whenever this page's torque state was stale
        // (no state yet, another tab), and it waited behind queued commands (2026-10-08 safety review).
        sendCommand({ type: "toggle_torque", enable: true, calibration: true });
        const torqueStarted = performance.now();
        while (!torqueRef.current && performance.now() - torqueStarted < 3000) await sleep(100);
        if (!torqueRef.current) throw new Error("Could not turn torque on.");
      }
      pushLog("info", `${Semi ? "Semi-automatic calibration" : "Self-calibration"} started: ${poses.length} poses`);
      if (Semi) SemiCal.Begin(poses.length);
      let seen = 0;
      for (let index = 0; index < poses.length; index += 1) {
        if (selfCalAbortRef.current) break;
        if (!armLiveRef.current) throw new Error("The arm disconnected.");
        if (Semi && SemiCal.DoneRequested()) break;
        const pose = poses[index];
        SelfCalMoveRequestedRef.current = true;
        setSelfCal((prev) => ({
          ...prev, index: index + 1, total: poses.length, step: `Moving to pose ${index + 1} of ${poses.length}`,
        }));
        if (Semi) SemiCal.Moving(index + 1);
        const Move = await (await fetch(`${API_URL}/motion`, {
          method: "POST", headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ type: "send_ik", slow: true, target: pose, plan_id: pose.plan_id }),
        })).json();
        if (!Move.ok) throw new Error(Move.error ?? "Calibration move was rejected.");
        MotionTimeoutRef.current = Math.max(SELFCAL_SETTLE_TIMEOUT_MS, Number(Move.state.motion_timeout_ms) || 0);
        const RouteStages = Math.max(1, Number(Move.state.motion_route?.stages) || 1);
        // A multi-stage route reports only its first waypoint's targets here.
        ExpectedTargetsRef.current = RouteStages > 1 ? {}
          : Object.fromEntries(IK_JOINTS.map(Name => [Name, Move.state.servos[Name].target_raw]));
        if (!await waitForSelfCalSettle(Number(Move.state.motion_epoch ?? -1), RouteStages)) {
          CheckCancelled();
          const Details = IK_JOINTS.filter(Name => circularRawDelta(servosRef.current[Name].presentRaw, servosRef.current[Name].targetRaw) > TASK_SETTLE_RAW_TOLERANCE)
            .map(Name => `${Name.replace(/_/g, " ")}: ${circularRawDelta(servosRef.current[Name].presentRaw, servosRef.current[Name].targetRaw)} raw steps from target`);
          throw new Error(`Joint settling timed out${Details.length ? ": " + Details.join("; ") : ": target or motion confirmation missing"}.`);
        }
        if (selfCalAbortRef.current) break;
        // Done early while the arm was on its way: end at this pose without asking for a click.
        if (Semi && SemiCal.DoneRequested()) break;

        // The pose's frames with their joint poses, and the bridge's observation (archived with them).
        const CaptureAndObserve = async () => {
          const CapturePositions = cloneServos(servosRef.current);
          const frames: string[] = [];
          const FrameMetadata: { captured_at_epoch_ms: number }[] = [];
          const DepthSeqs: (number | null)[] = [];
          for (let frame = 0; frame < SELFCAL_FRAMES_PER_POSE; frame += 1) {
            const url = captureVideoFrame(videoRef.current, SelfCalMirrored);
            if (url) {
              DepthSeqs.push(DepthSequenceFor(videoRef.current));
              FrameMetadata.push({
                frame_index: frames.length, captured_at_epoch_ms: Date.now(),
                video_time_seconds: videoRef.current?.currentTime,
                total_video_frames: videoRef.current?.getVideoPlaybackQuality?.().totalVideoFrames,
                bridge_state_age_ms: performance.now() - LastStateAtRef.current,
                servos: cloneServos(servosRef.current),
                joints: bridgeJointsRef.current?.map(Point => ({ ...Point })),
                motion_epoch: LastMotionEpochRef.current,
              } as { captured_at_epoch_ms: number });
              frames.push(url);
            }
            await sleep(SELFCAL_FRAME_GAP_MS);
          }
          const CaptureValid = !(frames.length !== SELFCAL_FRAMES_PER_POSE || videoRef.current?.dataset.captureReady !== "true"
              || selfCalAbortRef.current || !MotionHealthRef.current || performance.now() - LastStateAtRef.current > 700
              || SERVO_ORDER.some(Name => circularRawDelta(CapturePositions[Name].presentRaw, servosRef.current[Name].presentRaw) > 8));
          const liveJoints = bridgeJointsRef.current;
          const CaptureStream = videoRef.current?.srcObject;
          const observation = await (await fetch(`${API_URL}/selfcal/observe`, {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({
              images: frames,
              joints: liveJoints?.map((point) => ({ x: point.x, y: point.y, z: point.z })) ?? [],
              mirrored: SelfCalMirrored,
              depth_seqs: DepthSeqs,
              pose_index: index,
              requested_pose: pose, frame_metadata: FrameMetadata,
              servos_at_start: CapturePositions, servos_at_end: cloneServos(servosRef.current),
              camera_settings: CaptureStream instanceof MediaStream ? CaptureStream.getVideoTracks()[0]?.getSettings() : null,
              capture_valid: CaptureValid,
              capture_error: CaptureValid ? null : "The arm moved or lost feedback during image capture.",
            }),
          })).json() as { ok?: boolean; detected?: boolean; reason?: string; error?: string; capture_archive?: string };
          return { observation, frames, FrameMetadata };
        };
        let { observation, frames, FrameMetadata } = await CaptureAndObserve();
        if (!observation.ok) throw new Error(observation.error ?? "The bridge rejected a frame.");
        if (Semi) {
          // Freeze the middle frame of the capture for the click, and hold the pose until the user decides.
          let Decision: SemiCalDecision | null = null;
          for (;;) {
            if (!observation.capture_archive) throw new Error("Restart the bridge: it did not name the archived capture.");
            const Shown = Math.min(1, frames.length - 1);
            SemiCal.AddFrame({ PoseIndex: index, Ordinal: index + 1, Image: frames[Shown], FrameIndex: Shown,
              CaptureArchive: observation.capture_archive, Mirrored: SelfCalMirrored, CapturedAt: FrameMetadata[Shown].captured_at_epoch_ms });
            setSelfCal((prev) => ({ ...prev, step: `Pose ${index + 1} of ${poses.length}: click the tape on the frozen frame` }));
            Decision = await waitForSemiCalDecision();
            if (Decision === null) CheckCancelled();
            if (Decision !== "retake") break;
            // Retake: a moment later the pose must pass the same settle checks as after the move.
            await sleep(SemiCalRetakePauseMs);
            if (!await waitForSelfCalSettle(LastMotionEpochRef.current, 1)) {
              CheckCancelled();
              throw new Error("The arm did not hold still for the retake.");
            }
            ({ observation, frames, FrameMetadata } = await CaptureAndObserve());
            if (!observation.ok) throw new Error(observation.error ?? "The bridge rejected a frame.");
          }
          if (Decision === "clicked") seen += 1;
          if (Decision === "done") break;
          continue;
        }
        if (!observation.detected) {
          pushLog("warn", `Pose ${index + 1} skipped: ${observation.reason ?? "tip not seen"}`);
          setSelfCal((prev) => ({ ...prev, step: `Pose ${index + 1} of ${poses.length}: skipped (${observation.reason ?? "tip not seen"})` }));
          continue;
        }
        seen += 1;
        setSelfCal((prev) => ({
          ...prev,
          seen,
          step: observation.detected
            ? `Pose ${index + 1} of ${poses.length}: marker found`
            : `Pose ${index + 1} of ${poses.length}: marker not visible${observation.reason ? ` (${observation.reason})` : ""}`,
        }));
      }

      CheckCancelled();

      if (Semi) {
        const Counts = CountsOf(SemiCal.Snapshot().Frames);
        if (Counts.Clicked >= SemiCalMinClicks) {
          setSelfCal((prev) => ({ ...prev, step: "Fitting the camera to your clicks" }));
          const Report = await fitSemiCalibration();
          RunOutcome = Report ? CalibrationOutcomeOf(Report).RunOutcome : "fit failed";
        } else {
          // Too few clicks to compute any camera: nothing was fitted, the current calibration stays.
          RunOutcome = `ended with ${Counts.Clicked} clicks; not computed`;
          SemiCal.End(`The run ended with ${Counts.Clicked} clicks, too few to compute a camera: click at least ${SemiCalMinClicks} frames, then Fit now. The current calibration is still in use.`);
          fail(`Ended with ${Counts.Clicked} clicks (need ${SemiCalMinClicks} to compute a calibration)`);
        }
        return;
      }

      setSelfCal((prev) => ({ ...prev, step: "Solving the camera model" }));
      // The solver picks the tape among every pose's red blobs by camera
      // consistency, so poses the live tracker lost or mistook still count.
      if (seen < 10) pushLog("warn", `Live tracking saw the tape at only ${seen} of ${poses.length} poses; solving from every red blob recorded.`);
      const report = await (await fetch(`${API_URL}/selfcal/solve`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: "{}",
      })).json() as SelfCalReport;
      const Outcome = reportCalibrationOutcome(report, "auto");
      RunOutcome = Outcome.RunOutcome;
      setSelfCal((prev) => ({ ...prev, report, step: Outcome.Kind === "saved" ? "Calibrated"
        : Outcome.Kind === "saved_with_warnings" ? "Calibrated, with warnings" : Outcome.Kind === "not_saved" ? "Not saved" : "Not computed" }));
    } catch (error) {
      const Reason = selfCalFaultRef.current ?? (error instanceof Error ? error.message : String(error));
      RunOutcome = Reason;
      fail(`Failed: ${Reason}`);
      pushLog("warn", `Self-calibration failed: ${Reason}`);
      if (Semi) SemiCal.End(`Stopped: ${Reason}`);
    } finally {
      try {
        if (SelfCalMoveRequestedRef.current || SelfCalStopPromiseRef.current) await holdSelfCalInPlace();
      } catch (error) {
        RunOutcome = `Hold unconfirmed: ${error instanceof Error ? error.message : String(error)}`;
        fail(`Hold unconfirmed: ${error instanceof Error ? error.message : String(error)}`);
      }
      if (ArchiveStarted) {
        try {
          const Finish = await (await fetch(`${API_URL}/selfcal/finish`, {
            method: "POST", headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ outcome: RunOutcome, cancelled: selfCalAbortRef.current }),
          })).json();
          if (!Finish.ok) throw new Error(Finish.error ?? "Could not save the run outcome.");
        } catch (Error) {
          pushLog("warn", `Calibration archive finish failed: ${String(Error)}`);
        }
      }
      SelfCalPreparingRef.current = false;
      selfCalRunningRef.current = false;
      setSelfCal((prev) => ({ ...prev, running: false }));
      void refreshCalStatus();
    }
  };

  // One-shot marker check: grab a few live frames and ask the bridge whether the
  // printed marker is readable. No motion, no calibration session change. The
  // first frame is saved under back end/captures/<date>/selfcal with the result
  // drawn on it.
  const checkMarker = async () => {
    setMarkerCheck({ busy: true, ok: null, text: "Looking for the marker" });
    if (!(await waitForCamera()) || !videoRef.current) {
      setMarkerCheck({ busy: false, ok: false, text: "The camera is not available. Allow camera access, then try again." });
      return;
    }
    try {
      const frames: string[] = [];
      const DepthSeqs: (number | null)[] = [];
      const CheckMirrored = CameraMirroredRef.current;
      for (let frame = 0; frame < SELFCAL_FRAMES_PER_POSE; frame += 1) {
        const url = captureVideoFrame(videoRef.current, CheckMirrored);
        if (url) {
          frames.push(url);
          DepthSeqs.push(DepthSequenceFor(videoRef.current));
        }
        await sleep(SELFCAL_FRAME_GAP_MS);
      }
      if (!frames.length) throw new Error("No camera frame could be captured.");
      const result = await (await fetch(`${API_URL}/selfcal/check`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ images: frames, mirrored: CheckMirrored, hint: RedHintRef.current,
          initialize: RedHintRef.current != null, depth_seqs: DepthSeqs }),
      })).json() as {
        preview?: string; area_px?: number; ok?: boolean; detected?: boolean; edge_px?: number | null; frames?: number; frames_detected?: number;
        reason?: string | null; error?: string; frame_size?: number[] | null; archived?: string | null;
        centroid?: number[] | null; candidates?: number | null; hint_dist_px?: number | null;
        tape_distance_cm?: number | null;
      };
      if (result.preview) SetRedPreview(result.preview);
      if (!result.ok) throw new Error(result.error ?? "The bridge could not check the frames.");
      if (result.detected && result.centroid && result.frame_size) {
        // Snap the stored click to the true centroid so later checks and the
        // Start preflight test the tape, not the original finger error.
        RedHintRef.current = { u: result.centroid[0] / result.frame_size[0], v: result.centroid[1] / result.frame_size[1] };
      }
      const size = result.frame_size ? `${result.frame_size[0]}x${result.frame_size[1]}` : "?";
      const HintInfo = result.hint_dist_px != null ? `click ${result.hint_dist_px.toFixed(0)} px from nearest red edge, ` : "";
      const CandInfo = ` ${result.candidates ?? 0} red candidate(s)`;
      setMarkerCheck({
        busy: false,
        ok: Boolean(result.detected),
        text: result.detected
          ? `Marker found in ${result.frames_detected} of ${result.frames} frames, ${result.area_px?.toFixed(0) ?? "?"} red pixels (${size} camera)${result.tape_distance_cm != null ? `, ${result.tape_distance_cm.toFixed(1)} cm from the depth camera` : ""}.`
          : `No marker found: ${result.reason ?? "unknown reason"} (${HintInfo}${size} camera,${CandInfo}). The frame was saved to back end/captures for a closer look.`,
      });
    } catch (error) {
      setMarkerCheck({ busy: false, ok: false, text: `Check failed: ${error instanceof Error ? error.message : String(error)}` });
    }
  };

  const stopSelfCalibration = () => {
    selfCalAbortRef.current = true;
    void holdSelfCalInPlace().catch((error) => pushLog("warn", `Hold unconfirmed: ${String(error)}`));
    setSelfCal((prev) => ({ ...prev, step: "Stopping and holding position" }));
  };

  // Reject calibration: only ever the user's explicit choice (nothing rejects a calibration on its
  // own). The bridge archives the rejected file in its run and restores the one it replaced.
  // FittedAt names the calibration the user was shown, so a newer one is never rejected by mistake.
  // Data only: nothing moves.
  const rejectCalibration = async (FittedAt: string | null) => {
    if (selfCalRunningRef.current) return null;
    const Shown = FittedAt ?? calStatus?.fitted_at ?? null;
    if (!window.confirm(RejectQuestion(Shown, calStatus?.fitted_at === Shown ? calStatus?.replaced_fitted_at : null))) return null;
    setCalBusy(true);
    try {
      const Response = await fetch(`${API_URL}/selfcal/reject`, {
        method: "POST", headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ fitted_at: Shown, reason: "rejected by the user in the calibration panel" }),
      });
      if (Response.status === 404) throw new Error("Restart the bridge to load Reject calibration.");
      const Result = await Response.json() as CalibrationRejectResult;
      if (!Result.ok) throw new Error(Result.error ?? "The bridge did not reject the calibration.");
      const Text = RejectedText(Result);
      setSelfCal((prev) => ({ ...prev, step: Text, report: prev.report?.saved ? { ...prev.report, rejected: true } : prev.report }));
      SemiCal.MarkRejected(Text);
      pushLog("info", Text);
      recordSessionEvent("calibration_rejected", { rejected_fitted_at: Result.rejected_fitted_at ?? null,
        restored_fitted_at: Result.restored_fitted_at ?? null, restored_mode: Result.restored_mode ?? null, calibrated: Result.calibrated ?? null });
      // Positions measured through the rejected camera must not steer a task that is still running.
      EndAiTask("The camera calibration was rejected.", "self_calibration", { Advice: "Send the task again: positions now come from the restored calibration." });
      return Result;
    } catch (Failure) {
      const Reason = Failure instanceof Error ? Failure.message : String(Failure);
      setSelfCal((prev) => ({ ...prev, step: `Reject failed: ${Reason}` }));
      SemiCal.SetNotice(`Reject failed: ${Reason}`);
      pushLog("warn", `Reject calibration failed: ${Reason}`);
      return null;
    } finally {
      setCalBusy(false);
      void refreshCalStatus();
    }
  };

  const fitCalibration = async () => {
    const data = await calRequest("/calibration/fit");
    if (!data) return;
    if (data.ok) {
      const rms = typeof data.rms_cm === "number" ? data.rms_cm.toFixed(2) : "?";
      const worst = typeof data.max_err_cm === "number" ? data.max_err_cm.toFixed(2) : "?";
      setCalMessage(`Calibrated: RMS ${rms} cm, worst point ${worst} cm. Measured object positions are now fed to the AI on every capture.`);
      pushLog("ok", `Camera calibrated (RMS ${rms} cm)`);
    } else {
      setCalMessage(String(data.error ?? "Fit failed."));
    }
  };

  const stopTaskRecording = (reason: string) => {
    const active = taskRecorderRef.current;
    if (!active) return;
    taskRecorderRef.current = null;
    setTaskRecording(false);
    const { recorder, chunks, task, startedAt } = active;
    const finish = () => {
      const blob = new Blob(chunks, { type: recorder.mimeType || "video/webm" });
      const durationS = (Date.now() - startedAt) / 1000;
      if (!blob.size) {
        pushLog("warn", "Task recording was empty (nothing saved)");
        return;
      }
      const params = new URLSearchParams({
        task,
        reason,
        duration_s: durationS.toFixed(1),
        started_at: new Date(startedAt).toISOString(),
      });
      fetch(`${API_URL}/captures/video?${params.toString()}`, {
        method: "POST",
        headers: { "Content-Type": blob.type },
        body: blob,
      })
        .then((r) => r.json() as Promise<{ ok?: boolean; path?: string; error?: string }>)
        .then((data) => {
          if (data.ok && data.path) pushLog("ok", `Task recording saved (${durationS.toFixed(0)} s): ${data.path}`);
          else pushLog("warn", `Task recording not saved: ${data.error ?? "unknown error"}`);
        })
        .catch(() => pushLog("warn", "Task recording not saved: bridge unreachable"));
    };
    if (recorder.state === "inactive") {
      finish();
      return;
    }
    recorder.addEventListener("stop", finish, { once: true });
    recorder.stop();
  };

  const startTaskRecording = (task: string) => {
    if (taskRecorderRef.current) stopTaskRecording("replaced by a new task");
    const stream = videoRef.current?.srcObject;
    if (!(stream instanceof MediaStream) || typeof MediaRecorder === "undefined") {
      pushLog("warn", "Task recording skipped: no camera stream");
      return;
    }
    const mimeType = ["video/webm;codecs=vp9", "video/webm;codecs=vp8", "video/webm"]
      .find((type) => MediaRecorder.isTypeSupported(type));
    let recorder: MediaRecorder;
    try {
      recorder = new MediaRecorder(stream, { ...(mimeType ? { mimeType } : {}), videoBitsPerSecond: 2_000_000 });
    } catch (error) {
      pushLog("warn", `Task recording skipped: ${error instanceof Error ? error.message : String(error)}`);
      return;
    }
    const chunks: Blob[] = [];
    recorder.addEventListener("dataavailable", (event) => {
      if (event.data.size > 0) chunks.push(event.data);
    });
    recorder.start(1000);
    taskRecorderRef.current = { recorder, chunks, task, startedAt: Date.now() };
    setTaskRecording(true);
    pushLog("info", "Task recording started");
  };

  // Every way a task can end (done/ask/stop/block/emergency/step limit) clears
  // aiTask, so this one effect is the single place the recording is finalised,
  // with the reason the task_end record gave.
  useEffect(() => {
    if (!aiTask && taskRecorderRef.current) stopTaskRecording(TaskEndReasonRef.current ?? "task ended");
  }, [aiTask]);

  // The session video (back end/SessionRecorder.py) records only while a task runs: the same effect tells it, through
  // the dev server, when a task starts and ends and that it still runs (lib/SessionVideo.ts). A step of the same task
  // keeps its TaskIdRef, so only a new task or the end changes the recording.
  const TaskVideoRef = useRef<TaskVideoSignals | null>(null);
  TaskVideoRef.current ??= new TaskVideoSignals();
  useEffect(() => {
    TaskVideoRef.current!.Update(aiTask ? TaskIdRef.current : null, aiTask?.task ?? "", TaskEndReasonRef.current);
  }, [aiTask]);
  useEffect(() => {
    // A page closed or reloaded mid-task says so on its way out (keepalive), instead of going quiet for 3 minutes;
    // one the browser brings back from its back/forward cache with the task still on starts recording it again.
    const Leave = () => TaskVideoRef.current?.End("the page was closed or reloaded");
    const Back = (Event: PageTransitionEvent) => {
      if (Event.persisted && aiTaskRef.current) TaskVideoRef.current?.Update(TaskIdRef.current, aiTaskRef.current.task, null);
    };
    window.addEventListener("pagehide", Leave);
    window.addEventListener("pageshow", Back);
    return () => { window.removeEventListener("pagehide", Leave); window.removeEventListener("pageshow", Back); Leave(); };
  }, []);

  // One task_end per task, whichever way it ended (the model's done or ask
  // included): why, and who or what ended it.
  const RecordTaskEnd = (Reason: string, Source: TaskEndSource, Task = aiTaskRef.current) => {
    if (TaskEndRecordedRef.current === TaskIdRef.current) return;
    TaskEndRecordedRef.current = TaskIdRef.current;
    TaskEndReasonRef.current = compactOutputText(Reason, 160);
    recordSessionEvent("task_end", { reason: Reason, source: Source, task: Task?.task ?? null, step: Task?.step ?? null });
  };

  // Every way a task ends comes through here: Stop task, Stop and hold, the
  // emergency stop, a bridge fault, a configuration change, calibration,
  // Disconnect, Manual mode, Block, a time limit or failed request, and the
  // app's own stops (StopTaskWithReason). A model or /locate call in flight
  // is aborted (its reply used to become an approvable proposal after Stop
  // task, and faults dropped replies without a word), nothing is left
  // approvable, the record gets one task_end and the chat says why. With no
  // task, request or proposal left it does nothing, so the fault handler may
  // call it on every state frame. True when a task or request was ended.
  const EndAiTask = (Reason: string, Source: TaskEndSource, Options: { Advice?: string; Status?: string } = {}) => {
    const Task = aiTaskRef.current, Busy = AiRequestAbortRef.current !== null;
    if (!Task && !Busy && !AiProposalRef.current && !AiSettleWaitRef.current) return false;
    // A chained motion waiting to send its second leg (lib/AiChain.ts): the
    // second leg is dropped, never sent, even when its task already ended.
    const PendingChain = AiSettleWaitRef.current?.Chain?.Next ? AiSettleWaitRef.current.Chain : null;
    if (PendingChain) {
      const Run = ChainRunRef.current;
      if (Run) { Run.Results[1] = "not-sent"; Run.Why = Reason; }
      recordSessionEvent("chain_aborted", { reason: Reason, source: Source, part: PendingChain.Part, dropped: PendingChain.Next!.Kind,
        task: Task?.task ?? null, step: Task?.step ?? null });
      pushLog("warn", "Chained motion stopped: its second part was not sent");
    }
    ChainRunRef.current = null;
    ChainProposalRef.current = null;
    const ChainNote = PendingChain ? " The second part of the chained motion was not sent." : "";
    AiEpochRef.current += 1;
    AiRequestAbortRef.current?.abort();
    AiRequestAbortRef.current = null;
    aiContinueOnApproveRef.current = false;
    TaskEndsAfterApprovalRef.current = false;
    aiTaskRef.current = null;
    AiProposalRef.current = null;
    AiSettleWaitRef.current = null;
    setAiTask(null);
    setAiSettleWait(null);
    setAiProposal(null);
    setAiDisplayedProposal(null);
    setAiDisplayedIssues([]);
    setAiTargetEstimate(null); setAiObjectBox(null);
    setCaptureCountdown(null);
    if (!Task && !Busy) {
      // A chain with no task left (approved without one) was still running.
      if (!PendingChain) return false;
      setAiLastReply(`Stopped: ${Reason}${Options.Advice ? ` ${Options.Advice}` : ""}${ChainNote}`);
      setAiStatus(Options.Status ?? "Chained motion stopped");
      return true;
    }
    RecordTaskEnd(Reason, Source, Task);
    // The app's own stops, and any that ask the user to do something and send the task again: a
    // short go-ahead ("ok i did that", "try again", "now grab it") goes on with it (ContinueAskedTask).
    if (Task && Options.Advice && (Source === "app" || /send (?:the task|it) again/i.test(Options.Advice))) {
      AskedTaskRef.current = { Task: Task.task, Question: `${Reason} ${Options.Advice}`, Query: TaskObjectRef.current?.Query ?? null, Stopped: true };
    }
    setAiLastReply(`Stopped: ${Reason}${Options.Advice ? ` ${Options.Advice}` : ""}${ChainNote}`);
    setAiStatus(Options.Status ?? "Task stopped");
    pushLog("warn", `Task stopped: ${compactOutputText(Reason, 110)}`);
    return true;
  };
  EndAiTaskRef.current = EndAiTask;

  // Ends the task with a stated reason and what the user can do about it:
  // nothing is left approvable, the chat says why, and the record keeps it.
  const StopTaskWithReason = (Reason: string, Advice: string, Source: TaskEndSource = "app") => {
    const Task = aiTaskRef.current;
    recordSessionEvent("task_stop", { task: Task?.task ?? null, step: Task?.step ?? null, reason: Reason, advice: Advice });
    if (EndAiTask(Reason, Source, { Advice })) return;
    setAiLastReply(`Stopped: ${Reason} ${Advice}`);
    setAiStatus("Task stopped");
  };
  StopTaskWithReasonRef.current = StopTaskWithReason;

  // The bridge refused the approved task motion (a route or IK refusal comes
  // before anything moves). The first time in a task the model hears it at a
  // check-in and the step is not used up; the second time the task stops.
  const BridgeRefusedTaskMotion = (Text: string) => {
    const Task = aiTaskRef.current, Motion = LastTaskMotionRef.current;
    ApprovedCommandsRef.current = null;
    // A chained motion (lib/AiChain.ts): a refused first leg leaves the second
    // unsent and nothing moved; a refused second leg leaves the first's motion.
    // Its abort is recorded here, once: its wait goes now, so a stop below
    // neither records the dropped second leg again nor could ever send it.
    const Chain = AiSettleWaitRef.current?.Chain ?? null, Run = ChainRunRef.current;
    // Run together (lib/AiChain.ts TOGETHER), the chain was one motion: nothing ran.
    const Together = Boolean(Chain && Run?.Together);
    const Part = Chain ? (Chain.Part === 1 ? "first" : "second") : "";
    const Moved = Chain ? (Together ? " Nothing moved." : Chain.Part === 1 ? " Nothing moved, and its second part was not sent." : " Its first part ran.") : "";
    if (Chain) {
      if (Run) {
        Run.Results[Chain.Part - 1] = "refused";
        if (Chain.Part === 1 && !Together) Run.Results[1] = "not-sent";
      }
      recordSessionEvent("chain_aborted", { reason: `the bridge refused part ${Chain.Part}: ${Text}`, source: "bridge_refused", part: Chain.Part,
        dropped: Chain.Next?.Kind ?? null, task: Task?.task ?? null, step: Task?.step ?? null });
      SetSettleWait(null);
    }
    // No check-in follows it: a chain approved with no task, or the task's last
    // allowed motion. The task ends (LastTaskMotionRef is not this motion's, so
    // a rewind would tell the model about the one before and allow one more).
    if (Chain && (!Task || !Chain.CheckIn)) {
      const Refused = ChainProposalRef.current;
      ChainRunRef.current = null;
      ChainProposalRef.current = null;
      const What = Together ? (Task && Chain.EndsTask ? "the last motion allowed for this task, a chained motion run as one" : "the chained motion, run as one motion")
        : `the ${Part} part of ${Task && Chain.EndsTask ? "the last motion allowed for this task, a chained motion" : "the chained motion"}`;
      if (Task) {
        recordSessionEvent("bridge_refused", { reason: Text, motion: Refused ? motionRecord(Refused) : null, task: Task.task, step: Task.step, part: Chain.Part });
        StopTaskWithReason(`the bridge refused ${What}: ${Text.replace(/\.$/, "")}.${Moved}`,
          "Check the arm and the bridge status, then send the task again.", "bridge_refused");
        return;
      }
      setAiLastReply(`Stopped: the bridge refused ${What}: ${Text.replace(/\.$/, "")}.${Moved}`);
      setAiStatus("Chained motion refused by the bridge");
      return;
    }
    if (Chain?.Part === 2 && Task && Motion && BridgeRefusalsRef.current < 1) {
      // The first leg ran: the motion counted, and its check-in says the second was refused.
      BridgeRefusalsRef.current += 1;
      recordSessionEvent("bridge_refused", { reason: Text, motion: Motion.Record, task: Task.task, step: Motion.Step, part: 2 });
      pushLog("warn", `Bridge refused the second part of the chained motion: ${compactOutputText(Text, 110)}. Asking the AI again.`);
      ChainProposalRef.current = null;
      if (Run) Run.Why = `the bridge refused it: ${Text}`;
      // (Its check-in reads the refusal in the chain's run: lib/AppPlannedSteps.ts MotionAnomaly.)
      TaskCheckIn();
      return;
    }
    if (!Task || !Motion || BridgeRefusalsRef.current >= 1) {
      StopTaskWithReason(`the bridge refused the approved motion${Chain ? Together ? " (a chained motion run as one)" : ` (the ${Part} part of a chained motion)` : ""}: ${Text.replace(/\.$/, "")}.${Moved}${BridgeRefusalsRef.current >= 1 ? " It refused one earlier in this task too." : ""}`,
        "Check the arm and the bridge status, then send the task again.", "bridge_refused");
      return;
    }
    BridgeRefusalsRef.current += 1;
    recordSessionEvent("bridge_refused", { reason: Text, motion: Motion.Record, task: Task.task, step: Motion.Step });
    pushLog("warn", `Bridge refused the approved motion: ${compactOutputText(Text, 110)}. Asking the AI again.`);
    // Nothing moved: the motion is not counted and not listed as done.
    const Rewound: AiTaskState = { ...Task, step: Motion.Step };
    aiTaskRef.current = Rewound;
    setAiTask(Rewound);
    AiSettleWaitRef.current = null;
    setAiSettleWait(null);
    TaskMotionsRef.current = TaskMotionsRef.current.slice(0, -1);
    // An app's step the bridge refused did not run (lib/AppPlannedSteps.ts).
    AppStepsSinceModelRef.current = AppStepsSinceModelRef.current.filter(Step => Step.Step !== Motion.Step);
    LastTaskMotionRef.current = null;
    ChainRunRef.current = null;
    ChainProposalRef.current = null;
    runAiRequestRef.current(BuildTaskCheckInPrompt(Rewound, TaskMotionsRef.current,
      DescribeBridgeRefusal(Motion.Step, Text, Motion.Snap, Motion.Chain ? DescribeChainRun(Motion.Chain) : null)),
      { Why: `the bridge refused the last approved motion (${compactOutputText(Text, 100)})` });
  };
  BridgeRefusedTaskMotionRef.current = BridgeRefusedTaskMotion;

  // The arm's measured tip (model frame) and wrist pitch right now, from the
  // bridge's joints when it streams them.
  const LiveTip = () => {
    const Joints = bridgeJointsRef.current;
    const Angles = currentAngles(servosRef.current);
    const Tip = Joints?.length ? modelPointFromDisplayPoint(Joints[Joints.length - 1]) : forwardKinematics(Angles).slice(-1)[0];
    return { Tip, WristPitch: wristPitchFromAngles(Angles) };
  };

  // An approved gripper move of the task sent from fraction From to To: whether the jaws count as
  // closed from now on (GripperClosedAfter), and for a close where the tip is and which GRASP PLAN it is
  // closing with (TaskCloseRef).
  const NoteTaskGripperMove = (From: number, To: number) => {
    const Noted = NoteGripperMove(From, To, TaskGripperClosedRef.current, TaskCloseRef.current, mirrorDisplayPoint(LiveTip().Tip),
      TaskObjectRef.current?.Plan?.Reach.Plan ?? null);
    TaskGripperClosedRef.current = Noted.Closed;
    TaskCloseRef.current = Noted.Close;
  };

  // The camera check failed: stop before any model call. The model context
  // goes (its positions came from that camera); the chat and banner stay.
  const StopForCameraGate = (Gate: CameraGate) => {
    SetCameraGateBanner(Gate);
    CameraVerifiedRef.current = false;
    setAiMessages([]);
    recordSessionEvent("camera_gate", { code: Gate.Code, title: Gate.Title, detail: Gate.Detail });
    StopTaskWithReason(Gate.Title, Gate.Detail, "camera_gate");
  };

  // The arm's motion as a task sees it, from the state frames.
  const ArmMotionNow = () => ({ Live: armLiveRef.current, StateAgeMs: performance.now() - LastStateAtRef.current,
    RouteActive: RouteActiveRef.current, Moving: PositionsMovingRef.current, StillForMs: performance.now() - LastArmMotionAtRef.current });

  // Waits up to ArmStillWaitMs for the arm to be still: null when it is (or
  // the task ended meanwhile), otherwise why it is not.
  const WaitForArmStill = async (Signal: AbortSignal) => {
    const Started = performance.now();
    for (;;) {
      const Reason = ArmNotStillReason(ArmMotionNow());
      if (!Reason || Signal.aborted) return null;
      if (performance.now() - Started >= ArmStillWaitMs) return Reason;
      await sleep(100);
    }
  };

  const runAiRequest = async (promptText: string, options?: { auto?: boolean; checkIn?: AppCheckIn }) => {
    // An answer to the model's question goes on with the asked task, its question included.
    const Answered = options?.auto ? null : ContinueAskedTask(promptText, AskedTaskRef.current);
    const prompt = Answered?.Task ?? promptText.trim();
    if (!prompt || aiBusy || AiRequestAbortRef.current || selfCalRunningRef.current || ControlModeRef.current !== "Ai") return;
    if (HandGuideRef.current.On || HandGuideRef.current.Calibrating) {
      setAiStatus("Not started: hand guiding is on");
      pushLog("warn", "AI request not started: hand guiding is on. Press Hold here first.");
      return;
    }
    const RequestEpoch = ++AiEpochRef.current;
    let PreviousEnd: string | null = null;

    const requestModel = aiModel;
    if (!requestModel) {
      pushLog("warn", "No AI model loaded yet");
      return;
    }
    const RequestAbort = new AbortController();
    AiRequestAbortRef.current = RequestAbort;
    const RecordUsage = (Usage: AiUsage) => SetAiStats(Previous => AddAiUsage(Previous, Usage));
    // A task (awaiting approval, settling, a chained leg) or arm motion still under way as the prompt is sent,
    // read before a new prompt ends that task: the router keeps such a prompt on the task path (camera_depth).
    const TaskOngoing = Boolean(aiTaskRef.current || AiSettleWaitRef.current || AiProposalRef.current || RouteActiveRef.current);
    // A typed task after a long idle: keep_alive may have unloaded the model, so
    // warm it again while the arm settles, the countdown runs and /locate measures.
    if (!options?.auto && Date.now() - ModelWarmAtRef.current > RewarmAfterMs()) WarmAiModel(requestModel);
    recordSessionEvent(options?.auto ? "task_check_in" : "prompt", {
      prompt, model: requestModel, task: options?.auto ? aiTaskRef.current?.task ?? null : prompt,
      task_step: options?.auto ? aiTaskRef.current?.step ?? null : 1,
    });
    // A fresh user prompt starts (or replaces) the autonomous task; auto check-ins
    // continue the existing one. Either way any stale settle-wait is cancelled.
    if (!options?.auto) {
      // A task still waiting for approval or for its motion ends here, and a
      // chained motion's second leg is not sent.
      if (aiTaskRef.current) RecordTaskEnd("it was replaced by a new task.", "new_task");
      const PendingChain = AiSettleWaitRef.current?.Chain?.Next ? AiSettleWaitRef.current.Chain : null;
      if (PendingChain) {
        recordSessionEvent("chain_aborted", { reason: "a new task was sent.", source: "new_task", part: PendingChain.Part, dropped: PendingChain.Next!.Kind });
        pushLog("warn", "Chained motion stopped by a new task: its second part was not sent");
      }
      ChainRunRef.current = null;
      ChainProposalRef.current = null;
      TaskIdRef.current += 1;
      // How the last task ended: a new task's first model call is told of the app's steps in it (lib/AppPlannedSteps.ts).
      PreviousEnd = TaskEndReasonRef.current;
      TaskEndReasonRef.current = null;
      AiSettleWaitRef.current = null;
      setAiSettleWait(null);
      aiContinueOnApproveRef.current = false;
      aiTaskRef.current = null;
      setAiTask(null);
    }
    setAiBusy(true);
    setAiStatus(`Routing with ${PromptRouterOptions.Model}`);
    SetAiRouteStatus("Choosing what this prompt needs");
    setAiLastReply("");
    AiProposalRef.current = null;
    setAiProposal(null);
    setAiDisplayedProposal(null);
    setAiDisplayedIssues([]);
    setAiTargetEstimate(null); setAiObjectBox(null);

    try {
      // What the main model gets (lib/AiPromptRouter.ts): text, camera, or camera_depth (everything below).
      const Route = await PromptRouterRef.current.Route(prompt, RequestAbort.signal,
        { CheckIn: Boolean(options?.auto), Answer: Boolean(Answered), TaskOngoing });
      if (RequestEpoch !== AiEpochRef.current) return;
      SetAiRouteStatus(RouteStatusText(Route));
      recordSessionEvent("prompt_route", RouteRecord(Route));
      if (Route.Mode !== "camera_depth") {
        // A completely separate completion path (AnswerRoute): the prompt, and on the camera route one colour
        // frame. No arm-still wait, /locate, depth map, camera gate, task recording, notepad writes, proposal
        // parsing or fallback extraction. Even an erroneous model motion stays inert plain text.
        AskedTaskRef.current = null;
        TaskObjectRef.current = null;
        setAiMessages([]);
        const Mirrored = CameraMirroredRef.current;
        const Answer = await AnswerRoute(Route.Mode, prompt, RequestAbort.signal, {
          Capture: async () => {
            setAiStatus("Capturing camera frame");
            const Frame = await waitForCamera() ? captureVideoFrame(videoRef.current, Mirrored) : null;
            if (RequestEpoch !== AiEpochRef.current) return null;
            if (Frame) archiveCapture(Frame, "vision", Mirrored);
            else pushLog("warn", "Camera unavailable; answering without the image");
            recordSessionEvent("camera_frame", { route: Route.Mode, captured: Boolean(Frame), mirrored: Mirrored,
              camera_source: DepthSequenceFor(videoRef.current) != null ? "depth" : "webcam" });
            return Frame;
          },
          Ask: Messages => {
            setAiStatus(`Asking ${requestModel} (${Route.Mode === "text" ? "text only" : "camera image"})`);
            return WithTimeout(Signal => requestAiCompletion(Messages, requestModel, 512, RecordUsage, Signal),
              RequestAbort.signal, AiModelTimeoutMs, `the model (${requestModel})`).finally(() => { ModelWarmAtRef.current = Date.now(); });
          },
          ViewNote: AI_VISION_DEPTHWISE[Mirrored ? "Facing" : "Behind"],
        });
        if (RequestEpoch !== AiEpochRef.current) return;
        setAiLastReply(Answer.Reply);
        setAiStatus(Route.Mode === "text" ? "AI replied (text only)" : Answer.Image ? "AI replied (camera image, no measurements)" : "AI replied (no camera image)");
        recordSessionEvent("reply", { reply: Answer.Reply, mode: Route.Mode, image: Answer.Image, motion: null, possible: null });
        pushLog("info", `AI: ${compactOutputText(Answer.Reply)}`);
        return;
      }
      // camera_depth from here on: the task path.
      // A latched bridge fault refuses every motion until Hold clears it, so a
      // task started now could only be refused later. These prompts used to be
      // dropped without a word, after the whole capture.
      const Fault = MotionFaultRef.current;
      if (Fault) {
        const Reason = `the bridge reports a motion fault: ${Fault.replace(/\.$/, "")}.`, Advice = "Fix the cause, press Hold to clear the fault, then send the task again.";
        if (options?.auto) { EndAiTask(Reason, "motion_fault", { Advice, Status: "Stopped by a bridge fault" }); return; }
        setAiLastReply(`Not started: ${Reason} ${Advice}`);
        setAiStatus("Task not started: bridge fault");
        pushLog("warn", `Task not started: ${compactOutputText(Reason, 110)}`);
        return;
      }
      // A task relies on the bridge's scene layer, tip check and hold rules, so the running bridge must be the code
      // this page expects (2026-10-07 audit 2 H1: it ran day-old code and the scene layer silently did not run). And
      // every joint reading must be the measured one: a GUI "current angle" override moves this page's idea of the
      // arm, which the checks and the relative moves start from, while the bridge plans from the servos (audit M7).
      const Overridden = SERVO_ORDER.filter(Name => servosRef.current[Name].guiAngleRaw != null);
      const NotReady = socketRef.current?.readyState === WebSocket.OPEN && AiTaskBridgeReason(BridgeVersionRef.current)
        ? { Reason: `${AiTaskBridgeReason(BridgeVersionRef.current)!.replace(/^Bridge update required: /, "the bridge must be updated first: ").replace(/\.$/, "")}.`,
          Advice: "Support the arm and click Disconnect, load the bridge update (Motion panel), connect again, then send the task again." }
        : Overridden.length ? { Reason: `the joint panel overrides the current angle of ${Overridden.map(Name => Name.replace(/_/g, " ")).join(", ")}, so this page would plan from where those joints are not.`,
          Advice: "Clear the current-angle override in that joint's menu (Joints panel), then send the task again." } : null;
      if (NotReady) {
        if (options?.auto) { EndAiTask(NotReady.Reason, "not_ready", { Advice: NotReady.Advice, Status: "Stopped: not ready for a task" }); return; }
        setAiLastReply(`Not started: ${NotReady.Reason} ${NotReady.Advice}`);
        setAiStatus("Task not started");
        pushLog("warn", `Task not started: ${compactOutputText(NotReady.Reason, 110)}`);
        return;
      }
      if (!options?.auto) {
        const startedTask: AiTaskState = { task: prompt, step: 1 };
        // The motions of the task this one replaces: a carried measurement counts them as before this one (lib/ArmOcclusion.ts).
        const MotionsBefore = TaskMotionsRef.current.length;
        TaskMotionsRef.current = [];
        // A new task starts a new conversation: the last task's replies carried its
        // object positions and motion list, which the model then reused as if current.
        setAiMessages([]);
        aiTaskRef.current = startedTask;
        setAiTask(startedTask);
        startTaskRecording(prompt);
        AskedTaskRef.current = null;
        const Query = Answered ? Answered.Query : ParseTaskObject(prompt);
        // A go-ahead after the app's stop keeps where the stopped task left its object (its last measurement
        // and GRASP PLAN) for when it is out of sight now (under the gripper), and whether the jaws closed on
        // it. Measured this check-in (the user moved it, as the stop asked), it is bound and planned afresh.
        // An answer to the model's question about the same object keeps them too, with the jaws open (closed, it may be in
        // them), taken up only with the arm over it (Asked, lib/ArmOcclusion.ts): 2026-10-03 16:42 "proceed" with the pill
        // bottle under the open jaws lost its measurement.
        const Stopped = Answered?.Stopped && TaskObjectRef.current?.Query === Query ? TaskObjectRef.current : null;
        const Kept = Stopped ?? (Answered && !TaskGripperClosedRef.current && TaskObjectRef.current?.Query === Query ? TaskObjectRef.current : null);
        const KeptSeen = Kept?.LastSeen ? { ...Kept.LastSeen, Motions: Kept.LastSeen.Motions == null ? undefined : Kept.LastSeen.Motions - MotionsBefore } : null;
        TaskObjectRef.current = Query ? { Label: Query.Name, Query, DetectedNow: false, Binding: null, Candidates: [], LastSeen: null,
          Reachability: null, Plan: null, Intent: TaskIntent(Query), Carried: KeptSeen ? { LastSeen: KeptSeen, Plan: Kept!.Plan ?? null, ...(Stopped ? {} : { Asked: true }) } : null } : null;
        // A task that goes on from the last one (a go-ahead or an answer) keeps the readings taken clear of the gripper.
        if (!Kept) ArmClearRef.current.clear();
        TaskGripperClosedRef.current = Boolean(Stopped) && TaskGripperClosedRef.current;
        if (!TaskGripperClosedRef.current) TaskCloseRef.current = null;
        NoOpCountRef.current = 0;
        LastFinalTargetRef.current = null;
        TaskRejectedTargetsRef.current.clear();
        TaskEndsAfterApprovalRef.current = false;
        LastTaskMotionRef.current = null;
        BridgeRefusalsRef.current = 0;
        // The grasp gap this task plans with: a change in Settings meanwhile applies to the next task,
        // so it never re-plans a grasp whose object the gripper hides.
        TaskGraspGapRef.current = GraspGapRef.current;
      }
      const NeedsPositions = TaskNeedsPositions(aiTaskRef.current?.task ?? prompt);
      // The arm's pose is read once it is still. The 22:46 task read it at
      // submit, while the arm was folding home, and its relative move was
      // refused 5.4 cm later at approval.
      if (ArmNotStillReason(ArmMotionNow())) setAiStatus("Waiting for the arm to stop moving");
      const StillReason = await WaitForArmStill(RequestAbort.signal);
      if (RequestEpoch !== AiEpochRef.current) return;
      if (StillReason) {
        EndAiTask(StillReason === "the arm is moving"
          ? `the arm was still moving ${ArmStillWaitMs / 1000} s after the task was sent, so where it is could not be read.`
          : `the arm's position could not be read ${ArmStillWaitMs / 1000} s after the task was sent: ${StillReason}.`,
        "arm_moving", { Advice: "Send the task again when the arm has stopped." });
        return;
      }
      const Frame: { Tip: Point3 | null } = { Tip: null };
      let visionAttachment: VisionAttachment | null;
      try {
        visionAttachment = await captureVisionAttachment(RequestAbort.signal, {
          skipCountdown: Boolean(options?.auto),
          needsPositions: NeedsPositions,
          OnFrame: () => { Frame.Tip = LiveTip().Tip; },
          TaskObject: TaskObjectRef.current?.Query ?? null,
        });
      } catch (Failure) {
        if (!(Failure instanceof CameraMovedError)) throw Failure;
        if (RequestEpoch === AiEpochRef.current) StopForCameraGate(Failure.Gate);
        return;
      }
      if (RequestEpoch !== AiEpochRef.current) return;
      // The pose, the servos and the anchor for relative moves are read now,
      // right before the prompt is built, and must still match the image:
      // an arm that moved since the frame is not where the image shows it.
      // (A hold correction's brief "moving" is not a move; a route is.)
      const Live = LiveTip();
      const MovedCm = Frame.Tip ? Math.hypot(Live.Tip.x - Frame.Tip.x, Live.Tip.y - Frame.Tip.y, Live.Tip.z - Frame.Tip.z) : 0;
      if (MovedCm > ArmMovedDuringCaptureCm || RouteActiveRef.current) {
        EndAiTask(MovedCm > ArmMovedDuringCaptureCm
          ? `the arm moved ${MovedCm.toFixed(1)} cm while the camera image was measured, so the image no longer shows where it is.`
          : "the arm started moving while the camera image was measured, so the image no longer shows where it is.",
        "arm_moving", { Advice: "Send the task again when the arm has stopped." });
        return;
      }
      const requestServos = cloneServos(servosRef.current);
      const requestElbowMode = ElbowModeRef.current;
      // One set of joint limits for the snap, the checks, the feedback hints
      // and the send: the ones sendTargetPose sends with.
      const requestLimits = BridgeIkLimitsRef.current ?? undefined;
      // What Approve needs to check a proposal the way it was checked here.
      const requestSeed = servoSeed(requestServos);
      const requestSnapKey = aiSnapContextKey(requestServos, requestLimits, requestElbowMode);
      // The bridge's live tip: it is the same source as the on-screen tip AND
      // the calibrated camera measurements, so the AI's "current position" and
      // the "measured object positions" stay in one frame. The local FK's zero
      // offsets can disagree with the bridge's by several cm.
      const requestTip = Live.Tip;
      const requestCurrentPosition = {
        x: requestTip.x,
        y: requestTip.y,
        z: requestTip.z,
        wristPitch: Live.WristPitch,
      };
      // Anchor the AI's relative moves at the arm's ACTUAL pose rather than the
      // last commanded target — after IK clamps or hand-moving the arm in coast
      // mode, the stale target can sit centimeters from where the AI is told it is.
      const requestTarget = constrainTargetPose(
        { ...requestCurrentPosition },
        CurrentTargetRef.current,
      );
      // Track the task's object across check-ins: which measured object it is
      // in this frame (by name, the bridge's task query, where it was last
      // measured, or its colour: ResolveTaskObject), where it was last
      // measured, and (on a verified camera only) whether the right arm can
      // reach it. A renamed object ("USB drive", then "small black object")
      // stays the task object, so it is not reported lost.
      const TaskObject = TaskObjectRef.current;
      // Cached occluded positions and plans belong to their measured geometry, not to the next view.
      if (TaskObject) {
        const Previous = TaskObject.Plan?.Source.scene ?? TaskObject.LastSeen?.Object.scene;
        if (Previous && Previous.revision !== visionAttachment?.located?.scene?.revision) {
          TaskObject.Plan = null; TaskObject.LastSeen = null; TaskObject.Carried = null; TaskObject.Occluded = null;
        }
      }
      // A reading that is the gripper itself (its top the gripper's underside: lib/ArmMadeObjects.ts) is no object:
      // not an obstacle, a neighbour or the task object, and the measured list gives it as not measurable. The object
      // keeps its last reading with the gripper clear of it (ArmClearRef), or is left out without one.
      const ArmMade = WithoutArmMade(visionAttachment?.located ?? null, requestServos, ArmClearRef.current, Date.now(),
        Object_ => Object_.taskMatch || (TaskObject?.Query ? NamesQuery(TaskObject.Query, Object_.label) : false));
      const Located = ArmMade.Located;
      for (const Made of ArmMade.Dropped) {
        pushLog("info", `The measured "${Made.Object.label}" at X ${Made.Object.x.toFixed(1)}, Y ${Made.Object.y.toFixed(1)} is the gripper itself`
          + `${Made.Kept ? `: its reading clear of the gripper at ${new Date(Made.Kept.At).toTimeString().slice(0, 8)} stands` : ": left out"}`);
        recordSessionEvent("arm_made_object", { label: Made.Object.label, x: Made.Object.x, y: Made.Object.y, top_z: Made.Object.topZ,
          size_cm: Made.Object.sizeCm, arm_fraction: Made.Object.armFraction ?? null, inside_cm: Number(Made.InsideCm.toFixed(2)), gap_cm: Number(Made.GapCm.toFixed(2)),
          kept: Made.Kept ? { x: Made.Kept.Object.x, y: Made.Kept.Object.y, top_z: Made.Kept.Object.topZ, size_cm: Made.Kept.Object.sizeCm, at: new Date(Made.Kept.At).toISOString() } : null });
      }
      const Measured = Located && !Located.error ? Located.objects : null;
      // The jaws closed on nothing at the GRASP PLAN's grasp point: that plan is stale (GraspPlan.ts KeepOrReplan).
      // Where they closed counts with the plan they closed with (a chained close's lift has the tip up since).
      const JawsMissed = ClosedOnNothing({ GripperFraction: rawToFraction(requestServos.gripper, requestServos.gripper.presentRaw), Closed: TaskGripperClosedRef.current },
        TaskObjectRef.current?.Plan?.Reach.Plan ?? null, mirrorDisplayPoint(requestTip), TaskCloseRef.current);
      // A measurement picked as the task object: its reach verdict, and the record.
      // A grasp gets the fixed-jaw GRASP PLAN (lib/GraspPlan.ts), kept while the
      // gripper hides part of the object and made again when it moved.
      // Async: the reach verdict and GRASP PLAN are planned in a worker (PlanReachability), the page handling clicks meanwhile.
      // False: the task was stopped or replaced while it planned, or the planner failed (then the task stops saying so).
      // How the GRASP PLAN fared at this check-in's binding: kept (with the verdict's note and corridor, and whether the
      // camera's tape check corrected it) or made again. The app proposes its own next step only after a clean keep.
      let PlanCheck: { Kept: boolean; Note: string | null; Corridor: boolean; Tape: boolean } | null = null;
      const BindTaskObject = async (State: TaskObjectState, Binding: TaskObjectBinding): Promise<boolean> => {
        const Planned = async (Planned_?: ReachPlanned) => {
          const Reach = await PlanReachability(Binding.Object, Context, Planned_, RequestAbort.signal);
          if (RequestEpoch !== AiEpochRef.current || RequestAbort.signal.aborted) return null;
          if (!Reach) StopTaskWithReason(`the grasp plan for the ${State.Label} could not be worked out (the planner failed or took too long).`,
            "Send the task again; if it keeps failing, move the object a little and try again.");
          return Reach;
        };
        State.Binding = Binding;
        State.Candidates = [];
        State.DetectedNow = true;
        let PlanKept = false;
        // A grasp's GRASP PLAN puts the fixed pad the task's grasp gap outside the object (lib/GraspGap.ts), its points
        // held to the floor of the surface the object stands on (lib/AiMeasurements.ts TaskSupport; the desk's as always).
        // (On a frame that registered no scene, a desk object keeps the solids its last plan was made clear of: KeptDeskFloor.)
        const Floor = KeptDeskFloor(TaskSupport(Binding.Object, State.Label).Floor, Binding.Object, State.Plan?.Source ?? null);
        const Context: ReachContext = { Servos: requestServos, ElbowMode: requestElbowMode, Limits: requestLimits, ClearanceCm: TaskGraspGapRef.current,
          ...(Floor.Surface || Floor.Solids?.length ? { Floor } : {}) };
        if (!CameraVerifiedRef.current) State.Reachability = null;
        else if (State.Intent === "grasp") {
          const Key = GraspPlanKey(requestServos, requestElbowMode, requestLimits);
          if (State.Plan && State.Plan.Source.scene?.revision !== Binding.Object.scene?.revision) State.Plan = null;
          const Verdict = KeepOrReplan(State.Plan ?? null, Binding.Object, mirrorDisplayPoint(requestTip), TaskGripperClosedRef.current, Key, JawsMissed);
          const Started = performance.now();
          if (Verdict.Keep && State.Plan) {
            PlanKept = true;
            State.Plan.Note = Verdict.Note;
            // Geometry can remain unchanged across captures; commands still use the new capture's identity.
            State.Plan.Source = { ...State.Plan.Source, scene: Binding.Object.scene, supportGeometry: Binding.Object.supportGeometry };
            State.Plan.Confirmed = Verdict.Confirmed ?? State.Plan.Confirmed;
            if (Verdict.Corridor !== undefined) State.Plan.Corridor = Verdict.Corridor;
            if (Verdict.Missed) State.Plan.Missed = true;
            // At the hover point, the camera's tape check corrects where the fixed jaw goes (once; the model is told).
            const Tape = TapeCorrectedReach(State.Plan.Reach, Located?.cameraCheck ?? null, mirrorDisplayPoint(requestTip), Context,
              NeighbourObjects(Measured ?? [], Binding.Object), State.Label);
            if (Tape) {
              State.Plan.Reach = Tape.Reach;
              State.Plan.Note = [State.Plan.Note, Tape.Note].filter(Boolean).join("; ");
              const Grasp_ = Tape.Reach.Plan?.Grasp;
              recordSessionEvent("grasp_tape", { label: State.Label, residual_cm: Tape.Fix.ResidualCm, along_cm: Number(Tape.Fix.AlongCm.toFixed(2)),
                shift_cm: Number(Tape.Fix.ShiftCm.toFixed(2)), clamped: Tape.Fix.Clamped, moved: Tape.Moved, grasp: Grasp_ ? { x: Grasp_.x, y: Grasp_.y, z: Grasp_.z } : null });
            }
            PlanCheck = { Kept: true, Note: Verdict.Note, Corridor: Boolean(Verdict.Corridor), Tape: Boolean(Tape) };
            State.Reachability = State.Plan.Reach;
          } else {
            // Label: its refusal's advice names it as the stop's headline does (the task's word, not the detector's).
            const Reach = await Planned({ Others: NeighbourObjects(Measured ?? [], Binding.Object), Kind: State.Label, Label: State.Label });
            if (!Reach) return false;
            PlanCheck = { Kept: false, Note: Verdict.Note, Corridor: false, Tape: false };
            State.Reachability = Reach;
            // Missed, and the re-measure plans no grasp: the missed plan stays (Missed), so its steps take the
            // closed jaws up clear of the object before anything else; the model is told why.
            if (!Reach.Plan && JawsMissed && State.Plan) {
              State.Plan = KeepMissed(State.Plan, Binding.Object, Reach);
              State.Reachability = State.Plan.Reach;
            } else State.Plan = Reach.Plan ? { Reach, Source: PlanSource(Binding.Object, Floor), At: Date.now(), Key, Note: Verdict.Note, Confirmed: false, Corridor: null,
              // Made again after the jaws closed on nothing: the NEXT STEP says so while they are still closed.
              ...(Verdict.AfterMiss ? { AfterMiss: true } : {}) } : null;
          }
          const Plan = State.Reachability.Plan ?? null;
          const Point = (P: Point3) => ({ x: P.x, y: P.y, z: P.z });
          recordSessionEvent("grasp_plan", { label: State.Label, kept: Verdict.Keep && Boolean(State.Plan), note: State.Plan?.Note ?? Verdict.Note,
            confirmed: State.Plan?.Confirmed ?? null, missed: State.Plan?.Missed ?? null, jaws_missed: JawsMissed, after_miss: State.Plan?.AfterMiss ?? null,
            corridor_cm: State.Plan?.Corridor ? Number(State.Plan.Corridor.ShiftCm.toFixed(2)) : null,
            code: State.Reachability.Refusal?.Code ?? (Plan ? null : State.Reachability.Kind), pitch: Plan?.Pitch ?? null, grasp: Plan ? Point(Plan.Grasp) : null,
            hover: Plan ? Point(Plan.Hover) : null, hover_pitch: Plan?.HoverPitch ?? null, height_by: Plan?.HeightBy ?? null, side: Plan?.Side ?? null,
            width_eff_cm: Plan ? Number(Plan.WidthEffCm.toFixed(2)) : null, offset_cm: Plan ? Number(Plan.OffsetCm.toFixed(2)) : null,
            open_fraction: Plan?.OpenFraction ?? null, closing: Plan?.Closing.Mode ?? null,
            push_deg: Plan?.Closing.Mode === "sweep" ? Number(Plan.Closing.PushDeg.toFixed(1)) : null,
            grip_deg: Plan?.Closing.Mode === "sweep" ? Number(Plan.Closing.GripDeg.toFixed(1)) : null,
            grip3d_deg: Plan?.Closing.Mode === "sweep" && Plan.Closing.Grip3DDeg != null ? Number(Plan.Closing.Grip3DDeg.toFixed(1)) : null,
            drift_cm: Plan?.Closing.Mode === "sweep" ? Number(Plan.Closing.DriftCm.toFixed(2)) : null,
            depth_offset_cm: Plan?.Closing.Mode === "sweep" ? Number(Plan.Closing.DepthOffsetCm.toFixed(2)) : null,
            // How far the fixed finger and the moving jaw reach past its centre where the jaws close (GraspPlan.ts StraddleOf).
            straddle_cm: Plan?.Straddle ? [Number(Plan.Straddle.FixedCm.toFixed(2)), Number(Plan.Straddle.MovingCm.toFixed(2))] : null,
            diameter_cm: Plan?.Prism.Round ? [Plan.Prism.Round.LoCm, Plan.Prism.Round.HiCm] : null,
            // The grasp gap the plan put the fixed pad at (the task's setting), and the setting it was asked with.
            clearance_cm: Plan?.ClearanceCm ?? null, grasp_gap_cm: TaskGraspGapRef.current,
            // A best-effort plan (GraspPlan.ts GraspRules.BestEffort): what the app is not sure of.
            warnings: Plan?.Warnings ?? null,
            // What it was told of the object's measurement (GraspPlan.ts Caveats: a solid against it not measured apart).
            caveats: Plan?.Caveats ?? null,
            // A deep plan (GraspPlan.ts GraspRules.DeepGrasp): the palm and servo's gap to it, and the STS3215 servo's alone, at the grasp point.
            palm_gap_cm: Plan?.PalmGapCm != null ? Number(Plan.PalmGapCm.toFixed(2)) : null, servo_gap_cm: Plan?.ServoGapCm != null ? Number(Plan.ServoGapCm.toFixed(2)) : null,
            closer_cm: State.Reachability.CloserCm ?? State.Reachability.Refusal?.CloserCm ?? null,
            ms: Math.round(performance.now() - Started) });
        } else {
          const Reach = await Planned();
          if (!Reach) return false;
          State.Reachability = Reach;
        }
        // Measured only partly with the arm over it (lib/ArmOcclusion.ts): the last full measurement stays the working
        // one, with the kept GRASP PLAN's verdict; a plan made again from this reading goes with the reading.
        if (State.Occluded?.Partial && State.LastSeen && (State.Intent !== "grasp" || PlanKept)) {
          if (State.Intent === "grasp") State.LastSeen = { ...State.LastSeen, Reachability: State.Reachability };
          else State.Reachability = State.LastSeen.Reachability;
        } else {
          State.Occluded = null;
          State.LastSeen = { At: Date.now(), Object: Binding.Object, Reachability: State.Reachability, Motions: TaskMotionsRef.current.length };
        }
        recordSessionEvent("task_object_bound", { label: State.Label, measured_label: Binding.Object.label, source: Binding.Source,
          distance_cm: Binding.DistanceCm == null ? null : Number(Binding.DistanceCm.toFixed(2)) });
        return true;
      };
      if (TaskObject) {
        // After a gripper close it may be in the jaws: whatever is left at its spot is not it.
        const Resolution = Measured && TaskObject.Query
          ? ResolveTaskObject(TaskObject.Query, Measured, { LastSeen: TaskObject.LastSeen?.Object ?? null, Track: !TaskGripperClosedRef.current, Tip: mirrorDisplayPoint(requestTip) })
          : { Binding: null, Candidates: [] };
        // The arm over or beside where it was last measured hides it (lib/ArmOcclusion.ts): not measured, or measured
        // only partly, its last full measurement and GRASP PLAN stay the working ones. Judged on a measured frame only.
        const Hidden = (Seen: TaskObjectState["LastSeen"], Plan: TaskObjectState["Plan"]) => Measured
          ? JudgeArmOcclusion(Seen, Resolution.Binding?.Object ?? null, mirrorDisplayPoint(requestTip), { Plan: Plan?.Reach.Plan ?? null,
            Closed: TaskGripperClosedRef.current, Missed: Boolean(Plan?.Missed) || JawsMissed, Now: Date.now(), Motions: TaskMotionsRef.current.length })
          : null;
        // Measured only partly with the arm over it on a go-ahead or an answer: from the carried measurement and plan too.
        const Carried = TaskObject.LastSeen ? null : TaskObject.Carried;
        if (Resolution.Binding && Carried && Hidden(Carried.LastSeen, Carried.Plan)) Object.assign(TaskObject, { LastSeen: Carried.LastSeen, Plan: Carried.Plan });
        TaskObject.Occluded = Resolution.Binding ? Hidden(TaskObject.LastSeen, TaskObject.Plan) : null;
        if (Resolution.Binding) {
          if (!await BindTaskObject(TaskObject, Resolution.Binding)) return;
        } else {
          Object.assign(TaskObject, { Binding: null, Candidates: Resolution.Candidates, DetectedNow: false, Reachability: null });
          // Out of sight on a go-ahead after a stop: from the stopped task's last measurement and plan; after an
          // answer to the model's question, only with the arm over it (it may have moved meanwhile).
          if (Carried && (!Carried.Asked || Hidden(Carried.LastSeen, Carried.Plan))) Object.assign(TaskObject, { LastSeen: Carried.LastSeen, Plan: Carried.Plan });
          // Missed and not measured: its plan stays stale until a clean measurement plans again.
          if (TaskObject.Plan && JawsMissed) TaskObject.Plan.Missed = true;
          TaskObject.Occluded = Hidden(TaskObject.LastSeen, TaskObject.Plan);
        }
        const Occlusion = TaskObject.Occluded, Working = TaskObject.LastSeen?.Object;
        if (Occlusion && Working) {
          const Reading = (Object_: LocatedObject) => ({ label: Object_.label, x: Object_.x, y: Object_.y, z: Object_.z, top_z: Object_.topZ, size_cm: Object_.sizeCm });
          recordSessionEvent("task_object_hidden", { label: TaskObject.Label, partial: Occlusion.Partial ? Reading(Occlusion.Partial) : null, kept: Reading(Working),
            tip_cm: Number(Occlusion.TipCm.toFixed(2)), ago_s: Occlusion.AgoS, motions_ago: Occlusion.MotionsAgo, plan: Boolean(TaskObject.Plan?.Reach.Plan) });
        }
        TaskObject.Carried = null;
        // Withheld, on a level surface nobody declared (the bridge's hint): the user is told how the arm could grasp it there.
        const Undeclared = !Resolution.Binding && TaskObject.Query
          ? Located?.withheld.find(Item => Item.UndeclaredSupportZ != null && NamesQuery(TaskObject.Query!, Item.Label)) : undefined;
        if (Undeclared) {
          pushLog("warn", `The ${TaskObject.Label} could not be measured (${Undeclared.Reason}); it may stand on a surface at Z ${Undeclared.UndeclaredSupportZ!.toFixed(1)}`
            + ` that back end/DeclaredSurfaces.json does not declare. If it does, declare that platform (its top, X and Y range) and reload the page to have the arm grasp it there.`);
        }
        // Seen but withheld, and nothing else places it (not bound now, never measured in this task, not in the
        // jaws): the model would only propose targets read off the image, which nothing guards. Stop and say why.
        const Withheld = !Resolution.Binding && !TaskObject.LastSeen && !TaskGripperClosedRef.current
          ? WithheldTaskObject(Located, TaskObject.Query) : null;
        if (Withheld) {
          const Stop = WithheldTaskStop(Withheld, TaskObject.Label);
          recordSessionEvent("task_object_withheld", { label: TaskObject.Label, measured_label: Withheld.Label, reason: Withheld.Reason,
            task_query_only: Withheld.TaskQueryOnly ?? false, undeclared_support_z: Withheld.UndeclaredSupportZ ?? null });
          StopTaskWithReason(Stop.Reason, Stop.Advice);
          return;
        }
        const Found = Resolution.Binding?.Object ?? null;
        const Seen = TaskObject.LastSeen;
        if (Found && TaskObject.Reachability?.Kind === "out-of-reach" && !IsUnreliable(Found, Found)) {
          // A grasp's verdict says how many cm closer the app planned it (computed, not guessed).
          StopTaskWithReason(`the ${TaskObject.Label} measured at X ${Found.x.toFixed(1)}, Y ${Found.y.toFixed(1)} is out of the right arm's reach (${TaskObject.Reachability.BaseDistanceCm.toFixed(1)} cm from its shoulder).`,
            TaskObject.Reachability.Advice ?? TaskStopAdvice(true, TaskObject.Label));
          return;
        }
        const Tip = mirrorDisplayPoint(requestTip);
        // Not graspable by this gripper (too wide, too flat, a neighbour in the
        // fixed finger's way...), measured cleanly with the gripper away from it.
        const Refusal = TaskObject.Reachability?.Kind === "ungraspable" ? TaskObject.Reachability.Refusal ?? null : null;
        const FoundBox = Found ? ObjectPrism(Found) : null;
        if (Found && Refusal && Refusal.Code !== "unmeasured" && !IsUnreliable(Found, Found) && !(FoundBox && GripperNear(FoundBox, Tip))) {
          StopTaskWithReason(`the ${TaskObject.Label} measured at X ${Found.x.toFixed(1)}, Y ${Found.y.toFixed(1)} is not graspable by this gripper: ${Refusal.Text}.`, Refusal.Advice);
          return;
        }
        // Hidden under the gripper is normal while grasping; lost far from the
        // gripper is not, and the model would only chase stale coordinates.
        // After a gripper close it may be in the jaws, moving with the tip.
        // With a GRASP PLAN the gripper waits beside it, at the grasp point.
        const PlannedAt = TaskObject.Plan?.Reach.Plan?.Grasp ?? null;
        const Away = Seen ? Math.min(Math.hypot(Tip.x - Seen.Object.x, Tip.y - Seen.Object.y),
          PlannedAt ? Math.hypot(Tip.x - PlannedAt.x, Tip.y - PlannedAt.y) : Infinity) : 0;
        if (options?.auto && !Found && Seen && CameraVerifiedRef.current && Away > 8 && !TaskGripperClosedRef.current && !TaskObject.Occluded) {
          StopTaskWithReason(`the ${TaskObject.Label} has not been detected since ${new Date(Seen.At).toTimeString().slice(0, 8)}, and the gripper is ${Away.toFixed(1)} cm from where it was last measured.`,
            `Make sure the ${TaskObject.Label} is in view of the camera and not hidden by the arm, then send the task again.`);
          return;
        }
      }
      // Every [UNRELIABLE] measurement in this frame and the task object's last
      // one: the descent rule guarded only a matched task object, and a task
      // noun that matched nothing (16:10, "thingon") switched it off. A box
      // the bridge did not refine counts, unless it is the task object's.
      const UnreliableNow = () => UnreliableMeasurements(Measured, TaskObject);
      let UnreliableObjects = UnreliableNow();
      // While the task object was never bound, the model's own target_estimate_cm
      // may name it (16:05: 0.03 cm from the USB drive the phrase "black
      // thingon" never matched). Bound right after the reply is read, before it
      // is snapped or validated, so the reply and Approve check the same objects.
      // False: the task was stopped while the estimate's object was planned.
      const BindEstimate = async (Response: AiParsedResponse): Promise<boolean> => {
        if (!TaskObject?.Query || TaskObject.LastSeen || !Measured) return true;
        const Binding = BindFromEstimate(TaskObject.Query, Measured, Response.targetEstimate);
        if (!Binding) return true;
        if (!await BindTaskObject(TaskObject, Binding)) return false;
        UnreliableObjects = UnreliableNow();
        return true;
      };
      // A reply's tip target, moved to the nearest reachable one by the snap
      // (in the worker, awaited). It carries what Approve needs to check it
      // the same way; the snap's outcome goes to the record.
      const TaskLine = TaskObject ? TaskObjectLine(TaskObject, mirrorDisplayPoint(requestTip), TaskGrasp(requestServos)?.State) : null;
      // The line's NEXT STEP, read as a reply would be from the same anchor: repeated as the
      // prompt's last line, and (a planned grasp's) the only move the guard accepts near the object.
      const LineStep = PlannedStepOf(TaskLine, mirrorDisplayPoint(requestTarget), requestTarget.wristPitch);
      // The objects every gripper part keeps clear of and the task object's
      // GRASP PLAN or refusal, as of this frame; whether the jaws hold it now;
      // and, with a plan, its NEXT STEP (GraspPlan.ts NextStepReason).
      const GuardNow = () => BuildGraspGuard(Measured, TaskObject, TaskGripperClosedRef.current
        && rawToFraction(requestServos.gripper, requestServos.gripper.presentRaw) >= HeldFraction, LineStep);
      const SnapResponse = async (Response: AiParsedResponse): Promise<AiParsedResponse> => {
        if (!Response.motion) return Response;
        const Motion: AiMotionProposal = { ...Response.motion, snapSeed: requestSeed, snapContextKey: requestSnapKey, unreliable: UnreliableObjects,
          guard: GuardNow(), scene: Located?.scene ?? null, sceneCarry: Boolean(GuardNow()?.Held),
          constraintIssues: Response.motion.constraintIssues };
        if (!Motion.hasTargetMotion || !AiSnapEnabled) return { ...Response, motion: Motion };
        setAiStatus("Checking the proposed target's reach");
        const Started = performance.now();
        const Snap = await runAiSnap(aiSnapInput(Motion, requestServos, requestElbowMode, requestLimits, UnreliableObjects), RequestAbort.signal);
        if (Snap.Kind !== "exact") {
          recordSessionEvent("snap", { requested: motionRecord(Motion)?.requested_target_cm ?? null, requested_wrist_pitch_degrees: Motion.requestedWristPitchTarget,
            ...snapRecord(Snap), ms: Math.round(performance.now() - Started) });
        }
        return { ...Response, motion: applyAiSnap(Motion, Snap) };
      };
      // Only a step the guard holds the moves near the object to (BuildGraspGuard): the reminder says the app refuses the others.
      const Reminder = TaskObject ? NextStepReminder(GuardNow()?.Step ?? null, TaskObject.Label) : null;

      // THE APP RUNS THE GRASP PLAN'S STEPS ITSELF (lib/AppPlannedSteps.ts). After a motion that settled as planned, with
      // the GRASP PLAN kept and the task object re-measured where it was planned, the plan's NEXT STEP is proposed by the
      // app, without a model call: read, snapped and checked exactly like a reply (and exactly as planned: a step the snap
      // would move goes to the model), on the same approval card. After the plan's lift with the jaws holding the object,
      // a grab-only task ends there. Anything else (the reason first found below) calls the model, which is then told
      // which steps the app ran on its own since its last reply.
      const TaskNow = aiTaskRef.current;
      const HeldNow = TaskGripperClosedRef.current && rawToFraction(requestServos.gripper, requestServos.gripper.presentRaw) >= HeldFraction;
      const AppBlocker = (): string | null => {
        if (!options?.auto) return Answered ? "the user answered the AI's question" : "the task starts: the AI reads the request and picks the object";
        if (!TaskNow) return "no task is running";
        if (!options.checkIn) return "this check-in does not follow an approved motion that settled";
        if (options.checkIn.Why) return options.checkIn.Why;
        if (!TaskObject || TaskObject.Intent !== "grasp") return "the task is not a grasp";
        const State = TaskObject.Plan;
        if (!State?.Reach.Plan) return `the planner has no GRASP PLAN for the ${TaskObject.Label}`;
        if (State.Missed || State.AfterMiss) return "the jaws closed on nothing at the GRASP PLAN's grasp point";
        if (State.Corridor) return `the ${TaskObject.Label} may have moved into the gripper's way`;
        if (TaskObject.Occluded && !AppStepRules.RunWhileHidden) return `the gripper hides the ${TaskObject.Label}, so the AI checks the image`;
        if (PlanCheck && !PlanCheck.Kept) return `the GRASP PLAN was made again${PlanCheck.Note ? ` (${PlanCheck.Note})` : ""}`;
        if (PlanCheck?.Note) return `the GRASP PLAN was kept, but ${PlanCheck.Note}`;
        if (PlanCheck?.Corridor) return `the ${TaskObject.Label} may have moved into the gripper's way`;
        if (PlanCheck?.Tape) return "the camera's tape check corrected the GRASP PLAN";
        if (!LineStep) return "the planner gave no NEXT STEP";
        // The plan's last step: lifted with the jaws holding it. Only a grab-only task ends here, and only with the object
        // not measured where it stood (the jaws did not take it).
        if (IsLiftedStep(LineStep.Text)) {
          if (!HeldNow) return `the jaws do not read as holding the ${TaskObject.Label}`;
          if (!GraspOnlyRequest(TaskNow.task, TaskObject.Query ?? null)) return `the grasp is done, and the request asks for more than picking up the ${TaskObject.Label}`;
          const Source = State.Source;
          const Left = (Measured ?? []).find(Object_ => TaskObject.Query && NamesQuery(TaskObject.Query, Object_.label)
            && Math.hypot(Object_.x - Source.x, Object_.y - Source.y) <= AppStepRules.LeftBehindCm && Math.abs(Object_.z - Source.z) <= AppStepRules.LeftBehindZCm);
          if (Left) return `the ${TaskObject.Label} is still measured where it stood (X ${Left.x.toFixed(1)}, Y ${Left.y.toFixed(1)})`;
          return null;
        }
        if (!TaskObject.DetectedNow || !TaskObject.Binding || !PlanCheck) return `the ${TaskObject.Label} was not measured and bound this check-in`;
        return StepNeedsModel(LineStep.Text, LineStep.Json);
      };
      let ModelCallReason = AppBlocker();
      if (!ModelCallReason && TaskNow && TaskObject?.Plan?.Reach.Plan && LineStep && IsLiftedStep(LineStep.Text)) {
        const Percent = Math.round(rawToFraction(requestServos.gripper, requestServos.gripper.presentRaw) * 100);
        const Reply = AppDoneReply(TaskObject.Label, Percent);
        recordSessionEvent("planned_step", { app_run: true, finished: true, model_call_skipped: true, reason: `the GRASP PLAN's lift is done with the jaws holding the ${TaskObject.Label} (${Percent}% open), and the request was only to pick it up`,
          step: null, task: TaskNow.task, task_step: TaskNow.step, model_ms_saved_estimate: LastModelMsRef.current == null ? null : Math.round(LastModelMsRef.current) });
        aiContinueOnApproveRef.current = false;
        RecordTaskEnd(`the app finished the grasp: the GRASP PLAN's lift is done with the jaws holding the ${TaskObject.Label}.`, "app_done", TaskNow);
        aiTaskRef.current = null;
        setAiTask(null);
        setAiLastReply(Reply);
        setAiObjectBox(ObjectBoxOf(TaskObjectRef.current?.LastSeen?.Object ?? null));
        recordSessionEvent("result", { reply: Reply, task_status: "done", approvable: null, shown: null, issues: [], revisions: 0, target_estimate_cm: null,
          status: "Task finished", app_run: true });
        setAiStatus("Task finished");
        pushLog("ok", `Task finished by the app: ${compactOutputText(Reply)}`);
        return;
      }
      if (!ModelCallReason && TaskNow && TaskObject?.Plan?.Reach.Plan && LineStep?.Json) {
        setAiStatus("Checking the GRASP PLAN's next step");
        const Offer = await SnapResponse(parseAiResponse(`{"response_to_user":"","task_status":"continue","motion_proposal":${LineStep.Json}}`,
          requestTarget, requestServos));
        if (RequestEpoch !== AiEpochRef.current) return;
        const Check = Offer.motion
          ? validateAiMotionProposal(Offer.motion, requestElbowMode, requestServos, UnreliableObjects, requestLimits, TaskGrasp(requestServos), Offer.motion.guard ?? null)
          : null;
        const Idle = Offer.motion && Check?.possible ? NoOpReason(noOpView(Offer.motion), LiveTip(), BridgeTargetRef.current,
          Object.fromEntries(SERVO_ORDER.map(Name => [Name, servosRef.current[Name].targetRaw]))) : null;
        ModelCallReason = !Offer.motion ? "the NEXT STEP's JSON is not a motion"
          : taskMotionSnap(Offer.motion) ? "the NEXT STEP's target is not reachable exactly as planned (the snap would move it)"
          : !Check?.possible ? `the NEXT STEP did not pass the app's checks (${compactOutputText(Check?.issues.join(" ") ?? "", 140)})`
          : Idle ? `the NEXT STEP would not move the arm (${Idle})`
          : Offer.motion.hasTargetMotion && !Offer.motion.chain && RepeatsLastTarget(LastFinalTargetRef.current, Offer.motion.target) ? "the NEXT STEP repeats the last target"
          : null;
        if (!ModelCallReason && Offer.motion) {
          const Note = [AppRunStepNote, BestEffortNote(TaskObject.Plan.Reach.Plan)].filter(Boolean).join(" ");
          const Motion: AiMotionProposal = { ...Offer.motion, planned: true, approvalNote: Note, appRun: { What: StepWords(LineStep.Text), Json: LineStep.Json } };
          const Reply = AppRunStepReply(LineStep.Text);
          if (Motion.hasTargetMotion) LastFinalTargetRef.current = MarkFinalTarget(Motion.target);
          const Status = TaskNow.step >= MAX_AI_TASK_STEPS ? `Last motion of this task (${MAX_AI_TASK_STEPS} of ${MAX_AI_TASK_STEPS}): approving it ends the task`
            : `Task motion ${TaskNow.step} ready: the app's planned step (no AI call needed)`;
          recordSessionEvent("planned_step", { app_run: true, offered: true, model_call_skipped: true, reason: AppRunReason, step: LineStep.Json,
            motion: motionRecord(Motion), task: TaskNow.task, task_step: TaskNow.step,
            model_ms_saved_estimate: LastModelMsRef.current == null ? null : Math.round(LastModelMsRef.current) });
          setAiLastReply(Reply);
          setAiProposal(Motion);
          setAiDisplayedProposal(Motion);
          setAiDisplayedIssues([]);
          setAiTargetEstimate(null);
          setAiObjectBox(ObjectBoxOf(TaskObjectRef.current?.LastSeen?.Object ?? null));
          recordSessionEvent("result", { reply: Reply, task_status: "continue", approvable: motionRecord(Motion), shown: motionRecord(Motion), issues: [],
            revisions: 0, target_estimate_cm: null, status: Status, app_run: true });
          pushLog("ok", "The app's planned NEXT STEP is offered (no AI call needed)");
          pushLog("info", `App: ${compactOutputText(Reply)}`);
          // As a model's "continue" with a motion (below): the task's last allowed motion ends it once approved.
          if (TaskNow.step >= MAX_AI_TASK_STEPS) {
            aiContinueOnApproveRef.current = false;
            TaskEndsAfterApprovalRef.current = true;
            pushLog("warn", `Task loop: motion ${MAX_AI_TASK_STEPS} is the last one allowed`);
          } else {
            aiContinueOnApproveRef.current = true;
          }
          setAiStatus(Status);
          return;
        }
      }
      // The model is called: why, and the steps the app ran on its own since its last call (told once, then cleared).
      // (A new task after one that ended with the app's steps since the model's last reply: those steps, and how it ended.)
      const AppStepsNote = AppStepsSummary(AppStepsSinceModelRef.current, options?.auto ? ModelCallReason ?? "the app found no step to run itself"
        : `the last task ended${PreviousEnd ? ` (${PreviousEnd.replace(/\.$/, "")})` : ""} and a new prompt was sent`);
      const AppStepsTold = AppStepsSinceModelRef.current.length;
      AppStepsSinceModelRef.current = [];
      const ModelPrompt = WithAppStepsSummary(prompt, AppStepsNote);
      const userTextContent = buildAiUserPrompt(
        ModelPrompt,
        requestCurrentPosition,
        requestServos,
        [visionAttachment ? visionAttachment.summary + MeasuredObjectsSummary(Located, TaskObject?.DetectedNow ? TaskObject.Binding?.Object ?? null : null,
          Boolean(TaskObject?.Occluded?.Partial)) : "",
          TaskLine ? `\n${TaskLine}` : aiTaskRef.current ? `\n${NoTaskObjectLine}` : "", Reminder ?? ""].filter(Boolean).join("\n"),
      );
      const userContent: AiMessageContent = visionAttachment
        ? [
            { type: "text", text: userTextContent },
            { type: "image_url", image_url: { url: visionAttachment.originalImageUrl } },
            ...(visionAttachment.depthImageUrl
              ? [{ type: "image_url" as const, image_url: { url: visionAttachment.depthImageUrl } }]
              : []),
          ]
        : userTextContent;
      const baseMessages: AiApiMessage[] = [
        {
          role: "system",
          content: buildAiSystemPrompt(
            aiNotepad,
            Boolean(visionAttachment),
            Boolean(visionAttachment?.depthImageUrl),
            CameraMirroredRef.current ? "Facing" : "Behind",
            // The surface the task object stands on: its floor and the HARD FLOOR's words (the desk's as always without one).
            TaskFloor(TaskObject),
          ),
        },
        // Whole earlier turns only (a cut never orphans a reply), with their
        // coordinates already marked stale when they were stored.
        ...(options?.auto ? SliceWholeTurns(aiMessages, 3).map(({ role, content }) => ({ role, content })) : []),
        { role: "user", content: userContent },
      ];
      // Each model call has its own time limit (AskModel); running out of it ends the task.
      const AskModel = (Messages: AiApiMessage[]) => WithTimeout(Signal => requestAiCompletion(Messages, requestModel,
        AiMaxOutputTokens, RecordUsage, Signal), RequestAbort.signal, AiModelTimeoutMs, `the model (${requestModel})`)
        .finally(() => { ModelWarmAtRef.current = Date.now(); });
      setAiStatus(`Asking ${requestModel}`);
      const ModelStarted = performance.now();
      const content = await AskModel(baseMessages);
      if (RequestEpoch !== AiEpochRef.current) return;
      LastModelMsRef.current = performance.now() - ModelStarted;
      const notepadUpdate = extractNotepadUpdate(content);
      if (notepadUpdate) {
        setAiNotepad(notepadUpdate);
        void saveNotepad(notepadUpdate);
      }
      const parsedInitialResponse = addFallbackMotionFromText(
        parseAiResponse(content, requestTarget, requestServos),
        prompt,
        content,
        requestTarget,
        requestServos,
      );
      if (!await BindEstimate(parsedInitialResponse)) return;
      let aiResponse = await SnapResponse(parsedInitialResponse);
      if (RequestEpoch !== AiEpochRef.current) return;
      let latestAiMotion = aiResponse.motion;
      let aiStatusText = aiResponse.motion ? "Possible motion proposal ready" : "AI model responded";
      let aiLogText = aiResponse.motion ? "AI motion proposal passed feasibility check" : "AI response only";
      let nextConversation: AiChatMessage[] = [
        { role: "user", content: ModelPrompt },
        { role: "assistant", content },
      ];

      let validation = aiResponse.motion
        ? validateAiMotionProposal(aiResponse.motion, requestElbowMode, requestServos, UnreliableObjects, requestLimits, TaskGrasp(requestServos), aiResponse.motion.guard ?? null)
        : null;
      recordSessionEvent("reply", { attempt: 0, reply: aiResponse.userMessage, task_status: aiResponse.taskStatus,
        motion: motionRecord(aiResponse.motion), possible: validation?.possible ?? null, issues: validation?.issues ?? [],
        notepad_update: notepadUpdate ?? null,
        // Why the model was called instead of the app proposing the GRASP PLAN's step itself, how long it took, and how many app steps it was told of.
        model_call_reason: ModelCallReason, model_ms: Math.round(LastModelMsRef.current ?? 0), app_steps_told: AppStepsTold });
      // Whether a proposal of this check-in was refused for not following the NEXT STEP (GraspPlan.ts NextStepReason).
      const StepRefusedNow = () => Boolean(validation?.issues.some(Issue => Issue.includes(NextStepRefusalLead)));
      let StepRefused = StepRefusedNow();
      const StopLabel = TaskObject?.Label ?? null;
      // Why the current proposal would not move the arm, measured against the
      // live tip and the target the bridge is holding right now.
      // A snapped one says the request is out of reach and its nearest point is here.
      const NoOpOf = () => {
        if (!aiResponse.motion || !validation?.possible) return null;
        // A chain whose gripper action moves the gripper moves the arm.
        const Reason = NoOpReason(noOpView(aiResponse.motion), LiveTip(), BridgeTargetRef.current,
          Object.fromEntries(SERVO_ORDER.map(Name => [Name, servosRef.current[Name].targetRaw])));
        const Snapped = Reason ? taskMotionSnap(aiResponse.motion) : null;
        return Snapped ? SnapNoOpReason(Snapped) : Reason;
      };
      // The same requested target rejected at two check-ins means the model is
      // stuck on it (a camera phantom, or out of reach): stop, don't revise.
      const RejectedBefore = () => {
        if (!aiResponse.motion?.hasTargetMotion || !validation || validation.possible) return false;
        const Key = TargetKey(aiResponse.motion.requestedTarget);
        const Earlier = TaskRejectedTargetsRef.current.get(Key);
        if (Earlier !== undefined && Earlier !== RequestEpoch) return true;
        TaskRejectedTargetsRef.current.set(Key, RequestEpoch);
        return false;
      };
      const StopRejectedAgain = () => {
        const Point = mirrorDisplayPoint(aiResponse.motion!.requestedTarget);
        StopTaskWithReason(`the AI asked again for X ${Point.x.toFixed(1)}, Y ${Point.y.toFixed(1)}, Z ${Point.z.toFixed(1)}, which the app already rejected at an earlier check-in (${compactOutputText(validation?.issues.join(" ") ?? "", 140)}).`,
          TaskStopAdvice(CameraVerifiedRef.current, StopLabel, Boolean(TaskObjectRef.current?.Plan?.Reach.Plan)));
      };
      if (RejectedBefore()) { StopRejectedAgain(); return; }
      if (aiResponse.motion && validation) {
        const Offered = validation.possible && !NoOpOf();
        setAiLastReply(aiResponse.userMessage);
        setAiDisplayedProposal(aiResponse.motion);
        setAiDisplayedIssues(validation.issues);
        setAiProposal(Offered ? aiResponse.motion : null);
        setAiStatus(Offered
          ? "Possible motion proposal ready"
          : "Motion proposal shown with feasibility issues");
      }
      let revisionAttempt = 0;
      let lastFailureIssues = validation && !validation.possible ? validation.issues : [];
      const revisionMessages: AiApiMessage[] = [
        ...baseMessages,
        { role: "assistant", content },
      ];
      // One more model turn with the app's feedback; its reply replaces the
      // proposal. False when the request was superseded meanwhile.
      const AskAgain = async (Feedback: string, Note: string, Attempt: number | string) => {
        revisionMessages.push({ role: "user", content: Feedback });
        nextConversation.push({ role: "user", content: Note });
        const revisedContent = await AskModel(revisionMessages);
        if (RequestEpoch !== AiEpochRef.current) return false;
        const revisedNotepadUpdate = extractNotepadUpdate(revisedContent);
        if (revisedNotepadUpdate) {
          setAiNotepad(revisedNotepadUpdate);
          void saveNotepad(revisedNotepadUpdate);
        }
        revisionMessages.push({ role: "assistant", content: revisedContent });
        nextConversation.push({ role: "assistant", content: revisedContent });

        const parsedRevisedResponse = addFallbackMotionFromText(
          parseAiResponse(revisedContent, requestTarget, requestServos),
          prompt,
          revisedContent,
          requestTarget,
          requestServos,
        );
        if (!await BindEstimate(parsedRevisedResponse)) return false;
        aiResponse = await SnapResponse(parsedRevisedResponse);
        if (RequestEpoch !== AiEpochRef.current) return false;
        if (aiResponse.motion) {
          latestAiMotion = aiResponse.motion;
        }
        validation = aiResponse.motion
          ? validateAiMotionProposal(aiResponse.motion, requestElbowMode, requestServos, UnreliableObjects, requestLimits, TaskGrasp(requestServos), aiResponse.motion.guard ?? null)
          : null;
        recordSessionEvent("reply", { attempt: Attempt, feedback: Feedback, reply: aiResponse.userMessage,
          task_status: aiResponse.taskStatus, motion: motionRecord(aiResponse.motion), possible: validation?.possible ?? null,
          issues: validation?.issues ?? [], notepad_update: revisedNotepadUpdate ?? null });
        return true;
      };

      // A task reply of "continue" with no motion (2026-10-25 02:08: "I'll move a bit closer and lower", no JSON)
      // used to end the task. It gets one corrective turn in this check-in, like a no-op's (same task, same step);
      // its answer goes through every check below. A second reply without one ends the task (NoMotionTwiceReason).
      let AskedForMotion = false;
      if (aiTaskRef.current && aiResponse.taskStatus === "continue" && !aiResponse.motion) {
        AskedForMotion = true;
        recordSessionEvent("no_motion", { reply: aiResponse.userMessage, task: aiTaskRef.current.task, task_step: aiTaskRef.current.step });
        setAiStatus("The AI said continue without a motion; asking it for the motion");
        pushLog("warn", "AI replied \"continue\" without a motion: asking it for the motion it described");
        if (!(await AskAgain(NoMotionFeedbackPrompt(), "No-motion feedback: the reply said continue but had no motion_proposal.", "no-motion"))) return;
        if (RejectedBefore()) { StopRejectedAgain(); return; }
        if (validation && !validation.possible) lastFailureIssues = validation.issues;
        StepRefused ||= StepRefusedNow();
        if (aiResponse.motion && validation) {
          aiStatusText = "Possible motion proposal ready";
          aiLogText = "AI gave the motion it described after being asked";
        }
      }

      while (
        aiResponse.motion
        && validation
        && !validation.possible
        && revisionAttempt < MAX_AI_REVISION_ATTEMPTS
      ) {
        revisionAttempt += 1;
        setAiStatus(`Proposal impossible; sending feasibility feedback ${revisionAttempt}/${MAX_AI_REVISION_ATTEMPTS}`);
        lastFailureIssues = validation.issues;
        pushLog("warn", `AI proposal impossible: ${compactOutputText(validation.issues.join(" "))}`);

        // Feedback from the live tip, with the nearest reachable point and the
        // basket clearance worked out by the app's own IK (never the bridge's
        // stateful preview, which send_ik reuses as its warm start).
        const feedbackPrompt = BuildAiFeasibilityFeedbackPrompt(
          validation.issues,
          aiResponse.motion,
          requestTarget,
          { Servos: requestServos, ElbowMode: requestElbowMode, Limits: requestLimits },
          revisionAttempt,
          MAX_AI_REVISION_ATTEMPTS,
        );
        if (!(await AskAgain(feedbackPrompt,
          `Feasibility feedback ${revisionAttempt}: ${lastFailureIssues.join(" ") || "No feasibility details were returned."}`,
          revisionAttempt))) return;
        if (RejectedBefore()) { StopRejectedAgain(); return; }
        if (validation && !validation.possible) {
          lastFailureIssues = validation.issues;
        }
        StepRefused ||= StepRefusedNow();
      }

      // THE APP'S PLANNED STEP: the model was refused for not following a planned grasp's NEXT STEP
      // and still had no possible proposal after every revision. Instead of ending the task, the app
      // offers the step itself (the TASK OBJECT line's JSON, read, snapped and checked exactly like a
      // reply, never a no-op), labelled as the app's on the card; the user still approves it, and the
      // next check-in tells the model it ran (AiTaskLoop.ts PlannedStepFact).
      let PlannedOffered = false;
      if (StepRefused && revisionAttempt >= MAX_AI_REVISION_ATTEMPTS && aiResponse.motion && validation && !validation.possible
          && LineStep?.Json && TaskObject?.Plan?.Reach.Plan && aiTaskRef.current) {
        const Planned = await SnapResponse(parseAiResponse(`{"response_to_user":"","task_status":"continue","motion_proposal":${LineStep.Json}}`,
          requestTarget, requestServos));
        if (RequestEpoch !== AiEpochRef.current) return;
        const PlannedCheck = Planned.motion
          ? validateAiMotionProposal(Planned.motion, requestElbowMode, requestServos, UnreliableObjects, requestLimits, TaskGrasp(requestServos), Planned.motion.guard ?? null)
          : null;
        if (Planned.motion && PlannedCheck?.possible) {
          const Refused = { Response: aiResponse, Validation: validation };
          aiResponse = { ...Planned, userMessage: PlannedStepReply(revisionAttempt, LineStep.Text), taskStatus: "continue",
            motion: { ...Planned.motion, planned: true, approvalNote: PlannedStepNote } };
          validation = PlannedCheck;
          const Idle = NoOpOf();
          if (Idle) { aiResponse = Refused.Response; validation = Refused.Validation; }
          else PlannedOffered = true;
          recordSessionEvent("planned_step", { offered: PlannedOffered, why: Idle ?? null, step: LineStep.Json, motion: motionRecord(Planned.motion),
            refused: Refused.Validation.issues });
        } else {
          recordSessionEvent("planned_step", { offered: false, step: LineStep.Json, issues: PlannedCheck?.issues ?? ["the step's JSON is not a motion"] });
        }
      }

      if (PlannedOffered) {
        aiStatusText = "The app's planned NEXT STEP is ready to approve (the AI did not follow it)";
        aiLogText = "AI did not follow the NEXT STEP: the app's planned step is offered";
        pushLog("warn", `AI did not follow the NEXT STEP in ${revisionAttempt} revisions: offering the app's planned step`);
      } else if (revisionAttempt > 0) {
        if (aiResponse.motion && validation && validation.possible) {
          aiStatusText = `Revised possible motion ready after ${revisionAttempt} attempt${revisionAttempt === 1 ? "" : "s"}`;
          aiLogText = "AI revised proposal passed feasibility check";
        } else {
          if (aiResponse.motion && validation && !validation.possible) {
            aiResponse = {
              ...aiResponse,
              userMessage: [
                aiResponse.userMessage,
                `I tried ${revisionAttempt} revisions, but the latest proposal is still not possible: ${validation.issues.join(" ")}`,
              ].join(" "),
              motion: null,
            };
            aiStatusText = "No possible AI motion after revisions";
            aiLogText = "AI revisions exhausted";
            pushLog("warn", `AI revisions exhausted: ${compactOutputText(validation.issues.join(" "))}`);
          } else {
          aiStatusText = "AI model responded without a motion proposal";
          aiLogText = "AI response after feasibility revisions";
          }
        }
      }

      // A proposal that would not move the arm is never offered: three such
      // approvals once used up the motion limit. It gets one corrective turn
      // and is not counted; the task's second no-op stops it.
      let NoOp = NoOpOf();
      if (NoOp) {
        NoOpCountRef.current += 1;
        recordSessionEvent("no_op", { reason: NoOp, count: NoOpCountRef.current, motion: motionRecord(aiResponse.motion) });
        if (NoOpCountRef.current < 2) {
          setAiStatus("That move would not move the arm; asking for a different target");
          pushLog("warn", `AI proposal would not move the arm: ${NoOp}`);
          // The grasp's NEXT STEP again: the 07:56 model repeated its first reply (open the gripper) instead.
          if (!(await AskAgain(NoOpFeedbackPrompt(NoOp, TaskLine ? NextStepOf(TaskLine) : null), `No-op feedback: ${NoOp}.`, "no-op"))) return;
          if (RejectedBefore()) { StopRejectedAgain(); return; }
          NoOp = NoOpOf();
          if (NoOp) {
            NoOpCountRef.current += 1;
            recordSessionEvent("no_op", { reason: NoOp, count: NoOpCountRef.current, motion: motionRecord(aiResponse.motion) });
          } else if (aiResponse.motion && validation && !validation.possible) {
            aiResponse = { ...aiResponse, motion: null,
              userMessage: `${aiResponse.userMessage} The corrected proposal is not possible: ${validation.issues.join(" ")}` };
            aiStatusText = "No possible AI motion after the correction";
            aiLogText = "AI correction not possible";
          } else if (aiResponse.motion) {
            aiStatusText = "Corrected motion proposal ready";
            aiLogText = "AI corrected a proposal that would not move the arm";
          }
        }
        if (NoOp) {
          StopTaskWithReason(`the AI twice proposed a move that would not move the arm (${NoOp}).`, TaskStopAdvice(CameraVerifiedRef.current, StopLabel, Boolean(TaskObjectRef.current?.Plan?.Reach.Plan)));
          return;
        }
      }
      // The same target offered right after itself: the last one did not get
      // the arm there, and the model is repeating its own earlier reply. The
      // same point with the wrist turned is a new motion, not a repeat, and so
      // is a chain whose gripper action moves the gripper (its tip move is
      // skipped at the send if the arm is already there).
      if (aiResponse.motion?.hasTargetMotion && validation?.possible) {
        if (!aiResponse.motion.chain && RepeatsLastTarget(LastFinalTargetRef.current, aiResponse.motion.target)) {
          const Point = mirrorDisplayPoint(aiResponse.motion.target);
          StopTaskWithReason(`the AI proposed X ${Point.x.toFixed(1)}, Y ${Point.y.toFixed(1)}, Z ${Point.z.toFixed(1)}, wrist ${aiResponse.motion.target.wristPitch.toFixed(1)} again, the same target as its previous motion.`,
            TaskStopAdvice(CameraVerifiedRef.current, StopLabel, Boolean(TaskObjectRef.current?.Plan?.Reach.Plan)));
          return;
        }
        LastFinalTargetRef.current = MarkFinalTarget(aiResponse.motion.target);
      }

      // The model copied the GRASP PLAN's NEXT STEP: after it, the app may propose the plan's next step itself (lib/AppPlannedSteps.ts).
      if (aiResponse.motion && !aiResponse.motion.planned && LineStep?.Json && TaskObject?.Plan?.Reach.Plan) {
        const Step = parseAiResponse(`{"response_to_user":"","task_status":"continue","motion_proposal":${LineStep.Json}}`, requestTarget, requestServos).motion;
        if (Step && SameStep(aiResponse.motion, Step)) aiResponse = { ...aiResponse, motion: { ...aiResponse.motion, followsPlan: true } };
      }
      // A move of a best-effort grasp (GraspPlan.ts GraspRules.BestEffort): the card says what the app is not sure of.
      const BestEffort = aiResponse.motion && !aiResponse.motion.approvalNote ? BestEffortNote(TaskObjectRef.current?.Plan?.Reach.Plan) : null;
      if (BestEffort && aiResponse.motion) aiResponse = { ...aiResponse, motion: { ...aiResponse.motion, approvalNote: BestEffort } };
      if (RequestEpoch !== AiEpochRef.current) return;
      const finalDisplayMotion = aiResponse.motion ?? latestAiMotion;
      const finalDisplayIssues = finalDisplayMotion
        ? validateAiMotionProposal(finalDisplayMotion, requestElbowMode, requestServos, UnreliableObjects, requestLimits, TaskGrasp(requestServos), finalDisplayMotion.guard ?? null).issues
        : [];

      setAiLastReply(aiResponse.userMessage);
      setAiProposal(aiResponse.motion);
      setAiDisplayedProposal(finalDisplayMotion);
      setAiDisplayedIssues(finalDisplayIssues);
      // Blue-box estimate from the FINAL response (revisions included); clears
      // when the AI stops estimating one.
      setAiTargetEstimate(aiResponse.targetEstimate);
      setAiObjectBox(ObjectBoxOf(TaskObjectRef.current?.LastSeen?.Object ?? null));
      // One compact turn per request (prompt and final reply). By the next
      // check-in its target coordinates are stale, so they are stored as such:
      // the model once re-proposed a phantom target copied from its own reply.
      setAiMessages((items) => SliceWholeTurns([...items, ...CompactTurn(nextConversation, RequestEpoch)], 4));
      setAiStatus(aiStatusText);
      recordSessionEvent("result", { reply: aiResponse.userMessage, task_status: aiResponse.taskStatus,
        approvable: motionRecord(aiResponse.motion), shown: motionRecord(finalDisplayMotion), issues: finalDisplayIssues,
        revisions: revisionAttempt, target_estimate_cm: aiResponse.targetEstimate ?? null, status: aiStatusText });
      if (!aiResponse.motion && !finalDisplayMotion) {
        setAiStatus("AI replied, but no movement was found");
        pushLog("warn", "AI reply did not include a movement");
      } else {
        pushLog(aiResponse.motion ? "ok" : "info", aiLogText);
      }
      pushLog("info", `AI: ${compactOutputText(aiResponse.userMessage)}`);

      // TASK MODE loop bookkeeping: only an explicit "continue" WITH an approvable
      // motion keeps the loop alive; anything else ends it here.
      const taskState = aiTaskRef.current;
      if (taskState) {
        if (aiResponse.taskStatus === "continue" && aiResponse.motion) {
          if (taskState.step >= MAX_AI_TASK_STEPS) {
            // The last allowed motion stays approvable; approving it ends the
            // task with the reason instead of starting another check-in.
            aiContinueOnApproveRef.current = false;
            TaskEndsAfterApprovalRef.current = true;
            setAiStatus(`Last motion of this task (${MAX_AI_TASK_STEPS} of ${MAX_AI_TASK_STEPS}): approving it ends the task`);
            pushLog("warn", `Task loop: motion ${MAX_AI_TASK_STEPS} is the last one allowed`);
          } else {
            aiContinueOnApproveRef.current = true;
            setAiStatus(`Task motion ${taskState.step} ready: approving continues the loop`);
          }
        } else {
          aiContinueOnApproveRef.current = false;
          RecordTaskEnd(aiResponse.taskStatus === "done" ? "the AI reported the task done." : aiResponse.taskStatus === "ask" ? "the AI asked for input."
            : AskedForMotion && aiResponse.taskStatus === "continue" ? NoMotionTwiceReason : "the AI replied without a motion to continue with.",
          aiResponse.taskStatus === "done" ? "model_done" : aiResponse.taskStatus === "ask" ? "model_ask" : "model_no_motion", taskState);
          aiTaskRef.current = null;
          setAiTask(null);
          if (aiResponse.taskStatus === "done") {
            setAiStatus("Task finished");
            pushLog("ok", `Task finished: ${compactOutputText(aiResponse.userMessage)}`);
          } else if (aiResponse.taskStatus === "ask") {
            AskedTaskRef.current = { Task: taskState.task, Question: aiResponse.userMessage, Query: TaskObjectRef.current?.Query ?? null };
            setAiStatus("Task paused: AI needs your input");
            pushLog("info", `AI asks: ${compactOutputText(aiResponse.userMessage)}`);
          }
        }
      }
    } catch (error) {
      // An ended task (Stop, a fault) aborted this request: already reported.
      if (RequestEpoch !== AiEpochRef.current) return;
      SetAiStats(Previous => ({ ...Previous, Failures: Previous.Failures + 1 }));
      const text = error instanceof Error ? error.message : String(error);
      const TimedOut = error instanceof AiStepTimeoutError;
      recordSessionEvent("error", { stage: TimedOut ? "timeout" : "ai_request", error: text });
      pushLog("warn", "AI request failed");
      pushLog("warn", `AI error: ${compactOutputText(text)}`);
      // The step cannot go on, so the task ends with the reason: it used to
      // stay "waiting for AI" with nothing to approve, or hang on a call that
      // never answered.
      if (TimedOut) EndAiTask(`${text}.`, "timeout", { Advice: "Check that Ollama and the bridge are running, then send the task again.", Status: "AI request timed out" });
      else EndAiTask(`the AI request failed: ${compactOutputText(text, 200)}`, "error", { Advice: "Check the model, then send the task again.", Status: "AI request failed" });
    } finally {
      if (AiRequestAbortRef.current === RequestAbort) AiRequestAbortRef.current = null;
      setCaptureCountdown(null);
      setAiBusy(false);
    }
  };

  // Keep the ref pointing at the freshest closure so the settle effect's auto
  // check-in always reads current state.
  runAiRequestRef.current = (promptText: string, CheckIn?: AppCheckIn) => {
    void runAiRequest(promptText, { auto: true, checkIn: CheckIn });
  };

  const submitAiPrompt = (event?: FormEvent) => {
    event?.preventDefault();
    const prompt = aiInput.trim();
    if (!prompt || aiBusy || selfCalRunningRef.current) return;
    const RequestEpoch = ++AiEpochRef.current;
    setAiInput("");
    void runAiRequest(prompt);
  };

  // Stop task: the model call still running is aborted, so its reply can
  // never become an approvable proposal, and a proposal waiting for approval
  // goes too, as with every other stop.
  const stopAiTask = (reason = "stopped by user") => {
    recordSessionEvent("task_stop", { task: aiTaskRef.current?.task ?? null, step: aiTaskRef.current?.step ?? null, reason });
    EndAiTask("Stop task was pressed.", "stop_task", { Advice: "Any AI reply still on its way was discarded.", Status: "Task loop stopped" });
    pushLog("info", "Task loop stopped by user");
  };

  // Reset AI context: the model starts fresh. Its conversation turns and its notepad
  // are the only context it carries between calls (Ollama keeps none), so both are
  // cleared; a running task, a pending proposal and the object estimate go first.
  const ResetAiContext = () => {
    recordSessionEvent("context_reset", { task: aiTaskRef.current?.task ?? null });
    EndAiTask("The AI context was reset.", "context_reset", { Advice: "The model starts fresh.", Status: "Context reset" });
    AskedTaskRef.current = null;
    setAiMessages([]);
    setAiTargetEstimate(null); setAiObjectBox(null);
    setAiNotepad("");
    void saveNotepad("");
    setAiStatus("Context reset");
    setAiLastReply("Context reset: the conversation and the notepad are cleared; the next message starts fresh.");
    pushLog("info", "AI context reset: conversation and notepad cleared");
  };

  const ChangeControlMode = (Next: DashboardMode) => {
    if (Next === ControlModeRef.current) return;
    FinishTargetEdit(true); SetSelectedAxis(null); SetSelectedJoint(null);
    if (Next === "Ai" && (HandGuideRef.current.On || HandGuideRef.current.Calibrating)) {
      // Hand guiding is Manual mode only: the arm holds where it is (Hold here) before the AI can use it.
      sendCommand(HandGuideRef.current.On ? { type: "hand_guide", enable: false } : { type: "hand_guide_calibrate", enable: false });
      pushLog("info", "Hand guiding ended for AI mode: the arm holds where it is");
    }
    ControlModeRef.current = Next;
    SetControlMode(Next);
    SaveDashboardSetting("Mode", Next);
    if (Next === "Manual") {
      if (aiTaskRef.current) recordSessionEvent("ai_stopped", { reason: "switched to manual mode", task: aiTaskRef.current.task });
      EndAiTask("the app was switched to Manual mode.", "manual_mode", { Advice: "Switch back to AI mode and send the task again." });
      setAiDisplayedProposal(null); setAiDisplayedIssues([]);
      setAiTargetEstimate(null); setAiObjectBox(null);
      setAiStatus("Paused in manual mode");
    }
  };

  // After an approved task motion, wait for the bridge to start it (its motion
  // epoch passes the one recorded at send), then for fresh, still state before
  // re-prompting with a fresh capture. While the bridge plans a route it sends
  // this client nothing, so frozen pre-motion state must never read as arrival.
  // The bridge's budget follows the throttle; a timeout stops the task instead
  // of claiming arrival. A CHAINED MOTION waits the same way for its first
  // leg (a gripper stopped by an object settles too: the bridge's grasp hold
  // makes where it stopped its target), then sends the second (continueChain)
  // and, in a task, waits for that before the one check-in. Anything else
  // that moves or stops the arm meanwhile ends the task instead.
  useEffect(() => {
    if (!aiSettleWait) return;
    // A run of this effect left from before a stop or a new prompt (they clear
    // the ref at once, the state only at the next render) does nothing.
    if (AiSettleWaitRef.current !== aiSettleWait) return;
    const taskState = aiTaskRef.current;
    const Chain = aiSettleWait.Chain ?? null;
    // A chain with no check-in after it (the task's last allowed motion) still runs both legs.
    if (!taskState && (Chain?.CheckIn ?? true)) {
      setAiSettleWait(null);
      return;
    }
    if (ChainLegInterrupted(aiSettleWait, LastMotionEpochRef.current)) {
      EndAiTask("the arm's motion was stopped or replaced (Hold, Home, Stop or a slider) while the first part of a chained motion ran.", "chain_interrupted",
        { Advice: "Check where the arm is, then send the task again." });
      return;
    }
    const Now = Date.now();
    const Verdict = TaskSettleVerdictFor(aiSettleWait, Now, {
      Epoch: LastMotionEpochRef.current, StateAgeMs: performance.now() - LastStateAtRef.current,
      RouteActive: RouteActiveRef.current, Moving: MovingRef.current, MotionTimeoutMs: MotionTimeoutRef.current,
      WithinTolerance: SERVO_ORDER.every(
        (name) => circularRawDelta(servos[name].presentRaw, servos[name].targetRaw) <= TASK_SETTLE_RAW_TOLERANCE,
      ),
    });
    if (Verdict === "started") {
      setAiSettleWait({ ...aiSettleWait, startedAt: Now, StartedEpoch: LastMotionEpochRef.current });
      return;
    }
    // A gripper leg settled, but its jaws still creep: the tip move waits for them to hold still.
    const GripperCreeps = Verdict === "settled" && Boolean(Chain?.Next) && Chain?.First?.Kind === "gripper"
      && !ChainGripperStill(GripperSamplesRef.current, Now, aiSettleWait.SettledAt ?? Now);
    if (GripperCreeps && aiSettleWait.SettledAt == null) {
      SetSettleWait({ ...aiSettleWait, SettledAt: Now });
      return;
    }
    if (Verdict === "settled" && !GripperCreeps) {
      if (Chain?.Next) {
        continueChain(aiSettleWait);
        return;
      }
      setAiSettleWait(null);
      if (Chain && !Chain.CheckIn) {
        finishChainOutsideTask(null);
        return;
      }
      TaskCheckIn();
      return;
    }
    // The part of a chain that did not start or settle: the second is never sent.
    const Part = Chain ? (ChainRunRef.current?.Together ? "chained " : `${Chain.Part === 1 ? "first" : "second"} part of the chained `) : "";
    if (Verdict === "not-started") {
      StopTaskWithReason(`the bridge did not start the ${Part}motion within ${TaskStartLimitMs / 1000} s of approval.`,
        "Check the bridge status and the arm, then send the task again.", "motion_not_started");
      return;
    }
    if (Verdict === "timeout") {
      StopTaskWithReason(`the ${Part}motion did not settle before its timeout.`, "Check the arm for a stall or an obstruction, then send the task again.", "motion_timeout");
      return;
    }
    // Nudge a re-check even if no servo update re-runs this effect.
    const timer = window.setTimeout(
      () => setAiSettleWait((current) => (current ? { ...current } : current)),
      250,
    );
    return () => window.clearTimeout(timer);
  }, [aiSettleWait, servos]);

  // An approved proposal the app did not send after all (its check against the
  // live state failed). In a task the model hears why at a check-in, counted
  // like a no-op so it cannot loop, instead of the task waiting for an
  // approval that can never come (16:10 and 08:07 hung until Clear).
  const NotSentApproved = (Proposal: AiMotionProposal, Issues: string[], Details: Record<string, unknown> = {}) => {
    recordSessionEvent("approve_refused", { motion: motionRecord(Proposal), issues: Issues, ...Details });
    setAiProposal(null);
    setAiDisplayedProposal(Proposal);
    setAiDisplayedIssues(Issues);
    if (!aiTaskRef.current) { setAiLastReply(`Not sent: ${Issues.join(" ")}`); return; }
    NoOpCountRef.current += 1;
    if (NoOpCountRef.current >= 2) {
      StopTaskWithReason(`the app could not send the AI's proposal, the second time a proposal went nowhere in this task (${compactOutputText(Issues.join(" "), 140)}).`,
        TaskStopAdvice(CameraVerifiedRef.current, TaskObjectRef.current?.Label ?? null, Boolean(TaskObjectRef.current?.Plan?.Reach.Plan)));
      return;
    }
    aiContinueOnApproveRef.current = false;
    TaskEndsAfterApprovalRef.current = false;
    setAiLastReply(`Not sent: ${Issues.join(" ")} Asking the AI for a different target.`);
    runAiRequestRef.current(NotSentFeedbackPrompt(Issues), { Why: `the app could not send the last approved motion (${compactOutputText(Issues.join(" "), 100)})` });
  };

  // The check-in after an approved task motion has settled: what it measurably
  // did (and, for a chain, which of its legs ran), with a fresh capture.
  const TaskCheckIn = () => {
    const taskState = aiTaskRef.current;
    if (!taskState) return;
    pushLog("info", `Task loop: motion settled, starting check-in ${taskState.step}`);
    const Motion = LastTaskMotionRef.current, Live = servosRef.current;
    const Servos = Motion?.ServoTargets.map(Item => `${formatServoLabel(Item.name)} now at raw ${Live[Item.name].presentRaw} (commanded ${Item.targetRaw})`).join(", ");
    const EndTip = LiveTip().Tip;
    const Fact = Motion ? DescribeMotionFact({ Step: Motion.Step, StartTip: mirrorDisplayPoint(Motion.StartTip),
      EndTip: mirrorDisplayPoint(EndTip), Target: Motion.Target && mirrorDisplayPoint(Motion.Target), Servos: Servos || null,
      Snap: Motion.Snap, Chain: Motion.Chain ? DescribeChainRun(Motion.Chain) : null, Planned: Motion.Planned, AppRun: Motion.AppRun }) : null;
    // Whether that motion went as planned: if so, the app may propose the GRASP PLAN's next step itself (lib/AppPlannedSteps.ts).
    const Gap = (A: Point3, B: Point3) => Math.hypot(A.x - B.x, A.y - B.y, A.z - B.z);
    const CheckIn: AppCheckIn = { Why: !Motion ? "the motion this check-in follows is not known" : MotionAnomaly({ MovedCm: Gap(Motion.StartTip, EndTip),
      OffTargetCm: Motion.Target ? Gap(EndTip, Motion.Target) : null, Chain: Motion.Chain ?? null, Snapped: Boolean(Motion.Snap), Tolerance: TASK_SETTLE_RAW_TOLERANCE,
      Servos: Motion.ServoTargets.map(Item => ({ Name: Item.name, CommandedRaw: Item.targetRaw, PresentRaw: Live[Item.name].presentRaw,
        Closing: Item.name === "gripper" && rawToFraction(Live.gripper, Item.targetRaw) <= CloseTargetFraction })) })
      // The app's step offered after the model did not follow the plan: the model is told of it, as before.
      ?? (Motion.Planned ? "the last motion was the app's planned step, offered because the AI did not follow the NEXT STEP"
        : !Motion.FollowsPlan ? "the last motion was the AI's own, not the GRASP PLAN's NEXT STEP" : null) };
    runAiRequestRef.current(BuildTaskCheckInPrompt(taskState, TaskMotionsRef.current, Fact, Boolean(Motion?.Snap)), CheckIn);
  };

  // --- Chained motion (lib/AiChain.ts) ----------------------------------------

  // The settle wait, set in the ref at once: a stop right after a chain's
  // second leg is sent must not read the first leg's wait as still pending.
  const SetSettleWait = (Wait: TaskSettleWait | null) => {
    AiSettleWaitRef.current = Wait;
    setAiSettleWait(Wait);
  };

  // Sends one leg of an approved chain now. "skipped": it would not move
  // anything (the gripper is already at its target; the tip is already at its
  // target, NoOpReason), so nothing is sent and the bridge would never start it.
  const sendChainLeg = (Leg: ChainLeg, Proposal: AiMotionProposal): "sent" | "skipped" | "failed" => {
    if (SceneFreshnessIssue(Proposal.scene, LatestSceneRef.current)) return "failed";
    const Gripper = servosRef.current.gripper;
    const GripperChanges = (Raw: number) => ChangingServoTargets([{ name: "gripper" as ServoName, targetRaw: Raw }], { gripper: Gripper.targetRaw }).length > 0;
    if (Leg.Kind === "gripper") {
      if (!GripperChanges(Leg.TargetRaw)) return "skipped";
      // Sent now, not after the slider throttle, so the command range holds it (as for any approved gripper step).
      const Opening = rawToFraction(Gripper, Leg.TargetRaw) > rawToFraction(Gripper, Gripper.presentRaw);
      if (!updateServo("gripper", Leg.TargetRaw, true, Proposal.scene, Opening ? false : Boolean(Proposal.sceneCarry))) return "failed";
      // An opening ends a close; narrowing to a GRASP PLAN's 50-60% is not one. A close notes where the tip is.
      NoteTaskGripperMove(rawToFraction(Gripper, Gripper.targetRaw), rawToFraction(Gripper, Leg.TargetRaw));
      return "sent";
    }
    // Together: one send_ik whose route opens the gripper on the way. It moves the gripper, so
    // it is never skipped; with the gripper already there it is the tip move alone.
    const Opening = Leg.Kind === "together" && GripperChanges(Leg.TargetRaw) ? Leg.TargetRaw : null;
    if (Opening == null && NoOpReason({ hasTargetMotion: true, target: Leg.Target, servoTargets: [] }, LiveTip(), BridgeTargetRef.current, {})) return "skipped";
    updateTarget(Leg.Target);
    const Carry = Opening == null && (Proposal.sceneCarry || (TaskGripperClosedRef.current && rawToFraction(Gripper, Gripper.presentRaw) >= HeldFraction));
    if (!sendTargetPose(Leg.Target, "AI chained motion", Opening, Proposal.scene, Boolean(Carry))) return "failed";
    if (Opening != null) NoteTaskGripperMove(rawToFraction(Gripper, Gripper.targetRaw), rawToFraction(Gripper, Opening));
    return "sent";
  };

  // Whether the first leg of a chain got where it was sent, before the second
  // follows it: the gripper at its target (or, closing, stopped on an object:
  // the bridge's grasp hold) with the arm joints where they were; the tip
  // within ChainRules.ArrivedCm of its target. Why not, or null.
  const chainArrivalIssue = (First: ChainLeg, StartRaw: Partial<Record<ServoName, number>> | null): string | null => {
    const Live = servosRef.current;
    if (First.Kind === "gripper") {
      const Gripper = Live.gripper, From = StartRaw?.gripper ?? Gripper.targetRaw;
      const Closing = rawToFraction(Gripper, First.TargetRaw) < rawToFraction(Gripper, From);
      if (circularRawDelta(Gripper.presentRaw, First.TargetRaw) > TASK_SETTLE_RAW_TOLERANCE && !(Closing && GripperGraspHoldRef.current)) {
        return `the gripper stopped at ${Math.round(rawToFraction(Gripper, Gripper.presentRaw) * 100)}% open, not at the ${Math.round(rawToFraction(Gripper, First.TargetRaw) * 100)}% it was sent to`;
      }
      const Moved = SERVO_ORDER.filter(Name => Name !== "gripper" && StartRaw?.[Name] != null
        && circularRawDelta(Live[Name].presentRaw, StartRaw[Name]!) > TASK_SETTLE_RAW_TOLERANCE);
      return Moved.length ? `the arm moved while the gripper did (${Moved.map(formatServoLabel).join(", ")})` : null;
    }
    const Off = Math.hypot(...(["x", "y", "z"] as const).map(Axis => LiveTip().Tip[Axis] - First.Target[Axis]));
    return Off > ChainRules.ArrivedCm ? `the tip stopped ${Off.toFixed(1)} cm from that part's target (more than ${ChainRules.ArrivedCm} cm)` : null;
  };

  // The second leg checked again against the arm as the first left it: the
  // tip move solved as it was approved (from its IK seed, so a tick of jitter
  // cannot flip a snapped target's pitch, as at 16:10) with the gripper where
  // it stopped, and its path checked from the live joints (after a close the
  // jaws may hold the task object, which then travels with them, as the
  // check-in's guard has it); a gripper_after's swing and the chain's rule
  // about the jaws low beside the task object, where the tip stopped. Issues:
  // it is not sent.
  const chainLegIssues = (Leg: ChainLeg, Proposal: AiMotionProposal): string[] => {
    const SceneIssue = SceneFreshnessIssue(Proposal.scene, LatestSceneRef.current);
    if (SceneIssue) return [SceneIssue];
    const Live = servosRef.current;
    const Base = Proposal.guard ?? null;
    const Guard = Base && !Base.Held && TaskGripperClosedRef.current && rawToFraction(Live.gripper, Live.gripper.presentRaw) >= HeldFraction
      ? { ...Base, Held: true } : Base;
    if (Leg.Kind === "gripper") {
      const Issues: string[] = [];
      const Swept = GripperSweepReason(currentAngles(Live), angleFromRawValue(Live.gripper, Leg.TargetRaw), Guard);
      if (Swept) Issues.push(Swept);
      const Here = ChainGripperHereReason(rawToFraction(Live.gripper, Live.gripper.targetRaw), Leg.Fraction, mirrorDisplayPoint(LiveTip().Tip),
        Guard?.TaskPrism && !Guard.Held ? { Prism: Guard.TaskPrism, Label: Guard.TaskLabel } : null);
      if (Here) Issues.push(Here);
      return Issues;
    }
    const Seeded = { ...seededServos(Live, Proposal.snapSeed), gripper: Live.gripper };
    const TipOnly: AiMotionProposal = { ...Proposal, chain: null, servoTargets: [] };
    return validateAiMotionProposal(TipOnly, ElbowModeRef.current, Seeded, Proposal.unreliable ?? [], BridgeIkLimitsRef.current ?? undefined,
      TaskGrasp(Live), Guard, Live).issues;
  };

  // How a chain approved with no task went, in the chat. (The parser drops a motion
  // proposed with done or ask, so every chain today runs in a task; this is the fallback.)
  const finishChainOutsideTask = (Note: string | null) => {
    const Run = ChainRunRef.current;
    ChainRunRef.current = null;
    ChainProposalRef.current = null;
    if (!Run) return;
    const Text = DescribeChainRun(Run);
    recordSessionEvent("chain_done", { order: Run.Order, together: Boolean(Run.Together), results: Run.Results, why: Run.Why });
    // Run together, its one motion is the first result.
    const Results = Run.Together ? Run.Results.slice(0, 1) : Run.Results;
    pushLog(Results.every(Result => Result === "sent" || Result === "skipped") ? "ok" : "warn", compactOutputText(Text, 140));
    if (Note) setAiLastReply(Note);
    setAiStatus(Results[Results.length - 1] === "sent" ? "Chained motion sent" : Run.Together ? "Chained motion not sent" : "Chained motion stopped after its first part");
  };

  // The first leg of a chain has settled: check the second against the arm
  // now, send it, and wait for it (in a task, the check-in follows it). The
  // task's last motion ends the task once the last leg is sent.
  const continueChain = (Wait: TaskSettleWait) => {
    // Every task end and new prompt clears the wait at once: a wait that is no longer it sends nothing.
    if (AiSettleWaitRef.current !== Wait) return;
    const Chain = Wait.Chain!, Leg = Chain.Next!, Run = ChainRunRef.current, Proposal = ChainProposalRef.current;
    const Arrival = Chain.First ? chainArrivalIssue(Chain.First, Chain.StartRaw ?? null) : "the chained motion's first part is not known";
    const Issues = Arrival ? [Arrival] : Proposal ? chainLegIssues(Leg, Proposal) : ["the approved chained motion is no longer known"];
    const Epoch = LastMotionEpochRef.current, FirstCommand = CommandsSentRef.current + 1;
    const Sent = Issues.length || !Proposal ? "failed" : sendChainLeg(Leg, Proposal);
    const Result: ChainLegResult = Sent === "failed" ? "not-sent" : Sent;
    const Why = Issues.length ? Issues.join(" ") : Sent === "failed" ? CommandFailureRef.current ?? "the bridge or arm is not ready" : null;
    if (Run) { Run.Results[1] = Result; Run.Why = Why; }
    recordSessionEvent("chain_leg", { leg: 2, of: 2, kind: Leg.Kind, result: Result, why: Why,
      gripper_raw: Leg.Kind === "gripper" ? Leg.TargetRaw : null, target_cm: Leg.Kind === "tip" ? mirrorDisplayPoint(Leg.Target) : null,
      task: aiTaskRef.current?.task ?? null, step: aiTaskRef.current?.step ?? null });
    pushLog(Result === "sent" ? "ok" : "warn", Result === "sent" ? "Chained motion: part 2 of 2 sent"
      : Result === "skipped" ? "Chained motion: part 2 of 2 had nothing to do" : `Chained motion: part 2 of 2 not sent: ${compactOutputText(Why ?? "", 110)}`);
    if (Result === "sent") ApprovedCommandsRef.current = { First: FirstCommand, Last: CommandsSentRef.current };
    if (Chain.CheckIn && Result === "sent") {
      SetSettleWait({ approvedAt: Date.now(), epoch: Epoch, startedAt: null, Chain: { ...Chain, Part: 2, Next: null, First: Leg } });
      return;
    }
    SetSettleWait(null);
    if (Chain.CheckIn) {
      // Nothing more moves: the check-in now, saying what ran and what did not.
      ChainProposalRef.current = null;
      TaskCheckIn();
      return;
    }
    if (Chain.EndsTask && aiTaskRef.current) {
      ChainProposalRef.current = null;
      StopTaskWithReason(`it used all ${MAX_AI_TASK_STEPS} motions allowed for one task; the last one, a chained motion, was sent${Result === "sent" ? "" : Result === "skipped" ? " (its second part had nothing to do)" : ` only in part: its second part was not sent (${compactOutputText(Why ?? "", 120)})`}.`,
        "Check where the arm ended up, then send a new task if the job is not finished.", "step_limit");
      return;
    }
    finishChainOutsideTask(Result === "not-sent" ? `The chained motion stopped after its first part: its second part was not sent: ${Why}` : null);
  };

  // A CHAINED MOTION approved: its first leg is sent now and the second once
  // the first has settled (the settle effect), unless the task ends meanwhile.
  // One approval and one motion of the task; the record keeps both legs. An
  // opening gripper_before, when the bridge takes a gripper target with a tip
  // move, is one leg: the route opens the gripper while the tip moves.
  const approveChain = (Approved: AiMotionProposal, Live: { Tip: Point3; WristPitch: number }) => {
    const Together = Boolean(Approved.chain!.Together) && GripperWithTipRef.current;
    const Legs = chainLegs(Approved, Together), Task = aiTaskRef.current;
    const Texts = chainTexts(Approved, servosRef.current);
    const Run: ChainRun = { Order: Approved.chain!.Order, Gripper: Texts.Gripper, Tip: Texts.Tip, Results: [null, null], Why: null,
      ...(Together ? { Together } : {}) };
    const Continues = aiContinueOnApproveRef.current && Boolean(Task);
    const EndsTask = TaskEndsAfterApprovalRef.current && Boolean(Task);
    aiContinueOnApproveRef.current = false;
    TaskEndsAfterApprovalRef.current = false;
    ChainRunRef.current = Run;
    ChainProposalRef.current = Approved;
    // Where every servo is as the chain starts: a gripper leg must leave the arm joints there.
    const StartRaw = Object.fromEntries(SERVO_ORDER.map(Name => [Name, servosRef.current[Name].presentRaw])) as Record<ServoName, number>;
    const SentAtEpoch = LastMotionEpochRef.current, FirstCommand = CommandsSentRef.current + 1;
    // A "together" leg is the tip move carrying the gripper target: both are recorded.
    const LegRecord = (Leg: ChainLeg) => ({ kind: Leg.Kind, gripper_raw: Leg.Kind !== "tip" ? Leg.TargetRaw : null,
      target_cm: Leg.Kind !== "gripper" ? mirrorDisplayPoint(Leg.Target) : null });
    let Sent = sendChainLeg(Legs[0], Approved);
    Run.Results[0] = Sent === "failed" ? "not-sent" : Sent;
    recordSessionEvent("approve", { motion: motionRecord(Approved), chain: ChainOrderText(Run.Order, Run.Gripper, Run.Tip, Together), together: Together,
      target_sent: Legs[0].Kind !== "gripper" && Sent === "sent", sent: Sent === "sent", task: Task?.task ?? null, step: Task?.step ?? null });
    recordSessionEvent("chain_leg", { leg: 1, of: Legs.length, result: Run.Results[0], task: Task?.task ?? null, step: Task?.step ?? null, ...LegRecord(Legs[0]) });
    setAiProposal(null);
    setAiDisplayedProposal(null);
    setAiDisplayedIssues([]);
    // The first leg had nothing to do: the second is sent at once, as the motion,
    // once it passes its check from the arm as it is (as after any first leg).
    let Next: ChainLeg | null = Legs[1] ?? null;
    if (Sent === "skipped" && !Next) {
      // Run together, nothing moves: the gripper and the tip are already there.
      ChainRunRef.current = null;
      ChainProposalRef.current = null;
      NotSentApproved(Approved, ["Neither part of the chained motion would move the arm: the gripper and the tip are already where it asks."]);
      return;
    }
    if (Sent === "skipped" && Next) {
      const Second: ChainLeg = Next;
      Next = null;
      const Issues = chainLegIssues(Second, Approved);
      if (Issues.length) {
        ChainRunRef.current = null;
        ChainProposalRef.current = null;
        recordSessionEvent("chain_leg", { leg: 2, of: 2, result: "not-sent", why: Issues.join(" "), task: Task?.task ?? null, step: Task?.step ?? null, ...LegRecord(Second) });
        NotSentApproved(Approved, [`The first part of the chained motion had nothing to do, and its second part was not sent: ${Issues.join(" ")}`]);
        return;
      }
      Sent = sendChainLeg(Second, Approved);
      Run.Results[1] = Sent === "failed" ? "not-sent" : Sent;
      recordSessionEvent("chain_leg", { leg: 2, of: 2, result: Run.Results[1], task: Task?.task ?? null, step: Task?.step ?? null, ...LegRecord(Second) });
      if (Sent === "skipped") {
        ChainRunRef.current = null;
        ChainProposalRef.current = null;
        NotSentApproved(Approved, ["Neither part of the chained motion would move the arm: the gripper and the tip are already where it asks."]);
        return;
      }
    }
    if (Sent === "failed") {
      ChainRunRef.current = null;
      ChainProposalRef.current = null;
      const Reason = `the approved motion could not be sent: ${CommandFailureRef.current ?? "the bridge or arm is not ready"}.`;
      if (Task) StopTaskWithReason(Reason, "Connect the arm with torque on, then send the task again.");
      else setAiLastReply(`Not sent: ${Reason}`);
      return;
    }
    ApprovedCommandsRef.current = { First: FirstCommand, Last: CommandsSentRef.current };
    const Approval = Together ? "Chained motion approved and sent as one motion: the gripper opens while the tip moves"
      : Next ? "Chained motion approved: part 1 of 2 sent" : "Chained motion approved and sent";
    setAiStatus(Approval);
    pushLog("ok", Together || Next ? Approval : "Chained motion approved: its first part had nothing to do, part 2 sent");
    // One motion of the task, both legs in their order (run together, the opening on the way).
    if (Task) {
      const Point = mirrorDisplayPoint(Approved.target), Snapped = taskMotionSnap(Approved);
      const TipLine = `tip to X ${formatAiPromptNumber(Point.x)}, Y ${formatAiPromptNumber(Point.y)}, Z ${formatAiPromptNumber(Point.z)} cm, wrist ${formatAiPromptNumber(Approved.wristPitchTarget)} deg`
        + (Snapped ? SnapMotionNote(Snapped) : "");
      const GripperLine = `gripper to raw ${Approved.chain!.TargetRaw}`;
      TaskMotionsRef.current = [...TaskMotionsRef.current, `- Motion ${Task.step} (chained${Together ? ", one motion" : ""}${Approved.appRun ? ", the app's planned NEXT STEP, proposed by the app without asking you"
        : Approved.planned ? ", the app's planned NEXT STEP" : ""}): `
        + (Together ? `${TipLine}, the ${GripperLine} on the way` : Run.Order === "before" ? `${GripperLine}, then ${TipLine}` : `${TipLine}, then ${GripperLine}`)].slice(-MAX_AI_TASK_STEPS);
      if (Approved.appRun) AppStepsSinceModelRef.current = [...AppStepsSinceModelRef.current, { Step: Task.step, ...Approved.appRun }];
    }
    if (Continues && Task) {
      LastTaskMotionRef.current = { Step: Task.step, StartTip: Live.Tip, Target: Approved.target,
        ServoTargets: [{ name: "gripper", targetRaw: Approved.chain!.TargetRaw }], Snap: taskMotionSnap(Approved), Record: motionRecord(Approved), Chain: Run,
        Planned: Approved.planned && !Approved.appRun, AppRun: Boolean(Approved.appRun), FollowsPlan: Boolean(Approved.appRun || Approved.followsPlan) };
      const nextTask: AiTaskState = { ...Task, step: Task.step + 1 };
      aiTaskRef.current = nextTask;
      setAiTask(nextTask);
      pushLog("info", `Task loop: chained motion approved, will re-prompt after ${Together ? "it has" : "both parts have"} settled`);
    } else {
      // No check-in follows (the task's last motion): nothing may read the motion before it as this one.
      LastTaskMotionRef.current = null;
    }
    if (Next || Continues) {
      SetSettleWait({ approvedAt: Date.now(), epoch: SentAtEpoch, startedAt: null,
        Chain: { Part: Next || Together ? 1 : 2, Next, CheckIn: Continues, EndsTask, First: Next || Together ? Legs[0] : Legs[1], StartRaw } });
      return;
    }
    // Both legs are out (the first had nothing to do): nothing is left to wait for.
    if (EndsTask && Task) {
      ChainProposalRef.current = null;
      StopTaskWithReason(`it used all ${MAX_AI_TASK_STEPS} motions allowed for one task; the last one was sent.`,
        "Check where the arm ended up, then send a new task if the job is not finished.", "step_limit");
      return;
    }
    finishChainOutsideTask(null);
  };

  const approveAiMotion = async () => {
    const Proposal = aiProposal;
    if (!Proposal || selfCalRunningRef.current || ControlModeRef.current !== "Ai") return;
    const SceneIssue = SceneFreshnessIssue(Proposal.scene, LatestSceneRef.current);
    if (SceneIssue) { NotSentApproved(Proposal, [SceneIssue]); return; }
    // A relative move was planned from the tip at proposal time. If the arm has
    // moved since, the same delta lands somewhere the user was never shown.
    const Live = LiveTip();
    const AnchorDriftCm = Math.hypot(Live.Tip.x - Proposal.anchor.x, Live.Tip.y - Proposal.anchor.y, Live.Tip.z - Proposal.anchor.z);
    if (Proposal.hasTargetMotion && Proposal.relative && AnchorDriftCm > 1) {
      const Reason = `the arm moved ${AnchorDriftCm.toFixed(1)} cm since this relative move was proposed, so it was not sent.`;
      recordSessionEvent("approve_refused", { motion: motionRecord(Proposal), issues: [Reason], anchor_drift_cm: AnchorDriftCm });
      if (aiTaskRef.current) StopTaskWithReason(Reason, "Send the task again so the move is planned from where the arm is now.");
      else { setAiProposal(null); setAiDisplayedProposal(null); setAiDisplayedIssues([]); setAiLastReply(`Not sent: ${Reason}`); }
      return;
    }
    // Checked again with the joint limits the send below uses and the
    // proposal's [UNRELIABLE] objects, from the servo reading the target was
    // chosen with: a tick of jitter since then once flipped a pitch at the
    // edge of the IK tolerance (16:10). Snapped targets converge strictly.
    const Limits = bridgeIkLimits ?? undefined;
    const Check = (Motion: AiMotionProposal) => validateAiMotionProposal(Motion, elbowMode,
      seededServos(servosRef.current, Motion.snapSeed), Motion.unreliable ?? [], Limits, TaskGrasp(servosRef.current), Motion.guard ?? null);
    let Approved = Proposal;
    const Validation = Check(Proposal);
    if (!Validation.possible) {
      const LiveKey = aiSnapContextKey(servosRef.current, Limits, elbowMode);
      if (!Proposal.hasTargetMotion || !AiSnapEnabled || !Proposal.snapContextKey || LiveKey === Proposal.snapContextKey) {
        NotSentApproved(Proposal, Validation.issues);
        return;
      }
      // The joint limits, CAD alignment or elbow branch changed since the
      // proposal: snap the request again with them. A target that moved is
      // shown for another approval; a small change is sent.
      const Epoch = AiEpochRef.current, Task = aiTaskRef.current;
      setAiProposal(null);
      setAiStatus("The arm's limits changed since this was proposed; finding the nearest target again");
      const Seed = servoSeed(servosRef.current);
      const Snap = await runAiSnap(aiSnapInput(Proposal, servosRef.current, elbowMode, Limits, Proposal.unreliable ?? []));
      if (Epoch !== AiEpochRef.current || Task !== aiTaskRef.current || ControlModeRef.current !== "Ai") return;
      const Resnapped = applyAiSnap({ ...Proposal, snapSeed: Seed, snapContextKey: LiveKey }, Snap);
      const Again = Check(Resnapped);
      const ShiftCm = Math.hypot(Resnapped.target.x - Proposal.target.x, Resnapped.target.y - Proposal.target.y, Resnapped.target.z - Proposal.target.z);
      const TurnDeg = Math.abs(Resnapped.target.wristPitch - Proposal.target.wristPitch);
      recordSessionEvent("snap", { resnapped: true, shift_cm: ShiftCm, turn_deg: TurnDeg, possible: Again.possible, ...snapRecord(Snap) });
      if (!Again.possible) { NotSentApproved(Resnapped, Again.issues, { resnapped: true }); return; }
      if (ShiftCm > 0.3 || TurnDeg > 1) {
        const Shown = { ...Resnapped, approvalNote: "The arm's limits changed since this was proposed: the target moved, check it and approve again." };
        setAiDisplayedProposal(Shown);
        setAiDisplayedIssues([]);
        setAiProposal(Shown);
        setAiStatus("Target changed with the arm's limits: approve again to send it");
        return;
      }
      Approved = Resnapped;
    }
    // Checked again against the live state: a move to where the arm already
    // is is not sent and not counted; the AI gets one chance to correct it.
    const Held = NoOpReason(noOpView(Approved), Live, BridgeTargetRef.current,
      Object.fromEntries(SERVO_ORDER.map(Name => [Name, servosRef.current[Name].targetRaw])));
    const HeldSnap = Held ? taskMotionSnap(Approved) : null;
    const NoOp = HeldSnap ? SnapNoOpReason(HeldSnap) : Held;
    if (NoOp) {
      recordSessionEvent("approve_refused", { motion: motionRecord(Approved), issues: [NoOp], no_op: true });
      setAiProposal(null); setAiDisplayedProposal(null); setAiDisplayedIssues([]);
      if (!aiTaskRef.current) { setAiLastReply(`Not sent: ${NoOp}.`); return; }
      NoOpCountRef.current += 1;
      if (NoOpCountRef.current >= 2) {
        StopTaskWithReason(`the AI twice proposed a move that would not move the arm (${NoOp}).`,
          TaskStopAdvice(CameraVerifiedRef.current, TaskObjectRef.current?.Label ?? null, Boolean(TaskObjectRef.current?.Plan?.Reach.Plan)));
        return;
      }
      aiContinueOnApproveRef.current = false;
      TaskEndsAfterApprovalRef.current = false;
      setAiLastReply(`Not sent: ${NoOp}. Asking the AI for a different target.`);
      runAiRequestRef.current(NoOpFeedbackPrompt(NoOp), { Why: `the last approved motion would not move the arm (${NoOp})` });
      return;
    }
    if (Approved.chain && Approved.hasTargetMotion) {
      approveChain(Approved, Live);
      return;
    }
    // The check-in waits for a motion epoch newer than this one.
    const SentAtEpoch = LastMotionEpochRef.current;
    const FirstCommand = CommandsSentRef.current + 1;
    let sentTarget = false;
    if (Approved.hasTargetMotion) {
      updateTarget(Approved.target);
      sentTarget = sendTargetPose(Approved.target, "AI motion", null, Approved.scene, Boolean(Approved.sceneCarry));
    }
    // Only a servo that would change is sent: a restated one would start a
    // second move, which the bridge refuses while the first runs.
    const CurrentRaw = Object.fromEntries(SERVO_ORDER.map(Name => [Name, servosRef.current[Name].targetRaw]));
    const Gripper = servosRef.current.gripper;
    const ServoMoves = ChangingServoTargets(Approved.servoTargets, CurrentRaw);
    // Sent now, not after the slider throttle: a deferred send fell outside
    // this range, so the bridge's refusal of it never reached the task.
    const SentServos = ServoMoves.map((proposal) => updateServo(proposal.name, proposal.targetRaw, true, Approved.scene,
      proposal.name === "gripper" && rawToFraction(servosRef.current.gripper, proposal.targetRaw) > rawToFraction(servosRef.current.gripper, servosRef.current.gripper.presentRaw)
        ? false : Boolean(Approved.sceneCarry)));
    const Sent = sentTarget || SentServos.some(Boolean);
    ApprovedCommandsRef.current = { First: FirstCommand, Last: CommandsSentRef.current };
    const GripperMove = ServoMoves.find(Item => Item.name === "gripper");
    if (GripperMove && SentServos[ServoMoves.indexOf(GripperMove)]) {
      // An opening ends a close; narrowing to a GRASP PLAN's 50-60% above the object is not one. A close notes where the tip is.
      NoteTaskGripperMove(rawToFraction(Gripper, Gripper.targetRaw), rawToFraction(Gripper, GripperMove.targetRaw));
    }
    recordSessionEvent("approve", { motion: motionRecord(Approved), target_sent: sentTarget, sent: Sent,
      task: aiTaskRef.current?.task ?? null, step: aiTaskRef.current?.step ?? null });
    if (aiTaskRef.current) {
      const Point = mirrorDisplayPoint(Approved.target), Snapped = taskMotionSnap(Approved);
      TaskMotionsRef.current = [...TaskMotionsRef.current, `- Motion ${aiTaskRef.current.step}${Approved.appRun ? " (the app's planned NEXT STEP, proposed by the app without asking you)"
        : Approved.planned ? " (the app's planned NEXT STEP)" : ""}: `
        + (Approved.hasTargetMotion ? `tip to X ${formatAiPromptNumber(Point.x)}, Y ${formatAiPromptNumber(Point.y)}, Z ${formatAiPromptNumber(Point.z)} cm, wrist ${formatAiPromptNumber(Approved.wristPitchTarget)} deg` : "no tip move")
        + (Snapped ? SnapMotionNote(Snapped) : "")
        + (ServoMoves.length ? `; ${ServoMoves.map(Item => `${Item.name} to raw ${Item.targetRaw}`).join(", ")}` : "")].slice(-MAX_AI_TASK_STEPS);
      if (Approved.appRun) AppStepsSinceModelRef.current = [...AppStepsSinceModelRef.current, { Step: aiTaskRef.current.step, ...Approved.appRun }];
    }

    const hasServoTargets = ServoMoves.length > 0;
    if (Approved.hasTargetMotion && hasServoTargets) {
      setAiStatus(sentTarget ? "AI motion and slider change approved" : "AI target staged with slider change");
      pushLog(sentTarget ? "ok" : "info", sentTarget ? "AI motion and slider approved" : "AI target and slider staged");
    } else if (Approved.hasTargetMotion) {
      setAiStatus(sentTarget ? "AI motion approved and sent" : "AI motion approved in preview");
      pushLog(sentTarget ? "ok" : "info", sentTarget ? "AI motion approved" : "AI target staged");
    } else {
      setAiStatus("AI slider change approved");
      pushLog("ok", "AI direct slider change approved");
    }
    setAiProposal(null);
    setAiDisplayedProposal(null);
    setAiDisplayedIssues([]);

    if (TaskEndsAfterApprovalRef.current && aiTaskRef.current) {
      StopTaskWithReason(`it used all ${MAX_AI_TASK_STEPS} motions allowed for one task${Sent ? "; the last one was sent" : ", and the last one could not be sent"}.`,
        "Check where the arm ended up, then send a new task if the job is not finished.", "step_limit");
      return;
    }

    // If this motion was part of a task the AI wants to continue, advance the
    // step and start the settle-wait that leads to the automatic check-in.
    // Any command actually sent counts, gripper steps included.
    if (aiContinueOnApproveRef.current && aiTaskRef.current) {
      aiContinueOnApproveRef.current = false;
      if (!Sent) {
        StopTaskWithReason(`the approved motion could not be sent: ${CommandFailureRef.current ?? "the bridge or arm is not ready"}.`,
          "Connect the arm with torque on, then send the task again.");
        return;
      }
      LastTaskMotionRef.current = { Step: aiTaskRef.current.step, StartTip: Live.Tip,
        Target: Approved.hasTargetMotion ? Approved.target : null, ServoTargets: ServoMoves,
        Snap: Approved.hasTargetMotion ? taskMotionSnap(Approved) : null, Record: motionRecord(Approved), Planned: Approved.planned && !Approved.appRun,
        AppRun: Boolean(Approved.appRun), FollowsPlan: Boolean(Approved.appRun || Approved.followsPlan) };
      const nextTask: AiTaskState = {
        ...aiTaskRef.current,
        step: aiTaskRef.current.step + 1,
      };
      aiTaskRef.current = nextTask;
      setAiTask(nextTask);
      setAiSettleWait({ approvedAt: Date.now(), epoch: SentAtEpoch, startedAt: null });
      pushLog("info", "Task loop: motion approved, will re-prompt after it starts and settles");
    }
  };

  const blockAiMotion = () => {
    recordSessionEvent("block", { motion: motionRecord(aiProposal ?? aiDisplayedProposal),
      task: aiTaskRef.current?.task ?? null, step: aiTaskRef.current?.step ?? null });
    // Blocking a task motion vetoes the whole loop, not just this step, and
    // any revision still being asked for.
    if (EndAiTask("the proposed motion was blocked.", "block", { Status: "AI motion blocked" })) pushLog("info", "Task loop stopped by block");
    setAiProposal(null);
    setAiDisplayedProposal(null);
    setAiDisplayedIssues([]);
    setAiStatus("AI motion blocked");
    pushLog("info", "AI motion blocked");
  };

  const aiMotionDisplay = aiDisplayedProposal ?? aiProposal;
  const proposedPoint = aiMotionDisplay ? proposalDisplayPointCm(aiMotionDisplay.target) : null;
  const currentPoint = proposalDisplayPointCm(target);
  // Show the live arm tip in the same user-facing X convention as the Target
  // fields (visualPointFromDisplayPoint negates display X; Y/Z unchanged), so
  // the arm's X axis reads left = -, right = + just like the target point.
  const currentTip = visualPointFromDisplayPoint(joints[joints.length - 1] ?? { x: 0, y: 0, z: 0 });
  const currentWristPitch = wristPitchFromAngles(angles);
  const currentPositionTiles = [
    { axis: "X", value: currentTip.x, unit: "cm" },
    { axis: "Y", value: currentTip.y, unit: "cm" },
    { axis: "Z", value: currentTip.z, unit: "cm" },
    { axis: "Wrist", value: currentWristPitch, unit: "deg" },
  ];
  const futurePoint = proposedPoint ?? currentPoint;
  // Deltas from the live tip, not from the pose the proposal was planned from:
  // that pose can be stale by the time the user approves.
  const liveTipModel = joints.length ? modelPointFromDisplayPoint(joints[joints.length - 1]) : target;
  const futureDelta = aiMotionDisplay?.hasTargetMotion
    ? { left: liveTipModel.x - aiMotionDisplay.target.x, farther: aiMotionDisplay.target.y - liveTipModel.y, up: aiMotionDisplay.target.z - liveTipModel.z }
    : { left: 0, farther: 0, up: 0 };
  const displayedWristPitchTarget = aiMotionDisplay?.wristPitchTarget ?? currentWristPitch;
  const proposedPitchDelta = displayedWristPitchTarget - currentWristPitch;
  const approvalDiffersFromDisplayed = Boolean(
    aiProposal
    && aiMotionDisplay
    && (
      aiProposal.rawResponse !== aiMotionDisplay.rawResponse
      || Math.abs(aiProposal.target.x - aiMotionDisplay.target.x) >= 0.13
      || Math.abs(aiProposal.target.y - aiMotionDisplay.target.y) >= 0.13
      || Math.abs(aiProposal.target.z - aiMotionDisplay.target.z) >= 0.13
      || Math.abs(aiProposal.wristPitchTarget - aiMotionDisplay.wristPitchTarget) >= 0.1
    ),
  );
  const aiMotionWasAdjusted = Boolean(
    aiMotionDisplay
    && (
      Math.abs(aiMotionDisplay.target.x - aiMotionDisplay.requestedTarget.x) >= 0.13
      || Math.abs(aiMotionDisplay.target.y - aiMotionDisplay.requestedTarget.y) >= 0.13
      || Math.abs(aiMotionDisplay.target.z - aiMotionDisplay.requestedTarget.z) >= 0.13
      || Math.abs(aiMotionDisplay.wristPitchTarget - aiMotionDisplay.requestedWristPitchTarget) >= 0.1
    ),
  );

  const aiSnapNote = aiMotionDisplay ? approvalSnapNote(aiMotionDisplay) : null;

  // Hand guiding: the Motion panel's button and strip (lib/HandGuide.ts).
  function HandGuideContext() {
    return { SocketLive: socketState === "live", ArmConnected: armConnected, Torque: torque,
      Moving: RouteActiveRef.current || MovingRef.current, Fault: ProtectionState.Reason || null, Manual: ControlModeRef.current === "Manual",
      SelfCalibrating: selfCal.running, Restarting: BridgeRestarting };
  }
  const GuideActive = HandGuideState.On || HandGuideState.Calibrating;
  const GuideButton = HandGuideButton(HandGuideStale ? { ...HandGuideState, Calibrated: false } : HandGuideState, HandGuideContext());
  // Beside Hand guide (calibrated, idle): calibrate again, as wide as an icon (the Motion panel keeps its height).
  const GuideRecalibrate = !GuideActive && GuideButton.Label === "Hand guide";
  const GuideStrip = HandGuideStrip(HandGuideState);
  // While guiding, Coast says it drops the arm (as the emergency stop does); its accessible name stays "Coast".
  const CoastDrops = torque ? CoastWarning(HandGuideState) : null;

  // Glareshield lamps: the handful of facts worth seeing without reading a panel.
  const LinkLost = SerialLinkDown && socketState === "live";
  const Annunciators = [
    { Label: "BRIDGE", State: socketState === "live" ? "LIVE" : "LOCAL", Lit: socketState === "live" ? "go" : "off",
      Title: socketState === "live" ? "The bridge is streaming arm state" : "No bridge connection" },
    { Label: "ARM", State: LinkLost ? "LINK LOST" : armConnected ? "ONLINE" : "OFFLINE",
      Lit: LinkLost ? "alarm" : armConnected ? "go" : "off",
      Title: LinkLost ? status : armConnected ? `Serial link open on ${selectedPort || "the saved port"}` : "The arm is not connected" },
    { Label: "TORQUE", State: HandGuideState.On ? "GUIDE" : HandGuideState.Calibrating ? "CALIB" : torque ? "HOLD" : "COAST",
      Lit: HandGuideState.On || HandGuideState.Calibrating ? "alarm" : torque ? "lamp" : "off",
      Title: HandGuideState.On ? "Hand guiding: the arm carries its own weight; any fault makes it go limp"
        : HandGuideState.Calibrating ? "Calibrating hand guiding" : torque ? "Servos are holding their position" : "Servos are free to move by hand" },
    { Label: "ROUTE", State: RouteActive ? "RUN" : "IDLE", Lit: RouteActive ? "lamp" : "off",
      Title: RouteActive ? "A checked route is running" : "No route is running" },
    { Label: "FAULT", State: ProtectionState.Reason ? "TRIP" : "CLEAR", Lit: ProtectionState.Reason ? "alarm" : "off",
      Title: ProtectionState.Reason || "No motor or motion fault" },
  ];

  const CanvasSelectJoint = UseLatestCallback(SelectJoint);
  const CanvasJointChange = UseLatestCallback((Name: ServoName, Raw: number) => { updateServo(Name, Raw); });
  const CanvasCaptureAlignment = UseLatestCallback(CaptureCadReference);
  const CanvasTargetDragStart = UseLatestCallback(BeginTargetEdit);
  const CanvasTargetDragEnd = UseLatestCallback(FinishTargetEdit);
  const CanvasTargetChange = UseLatestCallback(PreviewVisualTarget);
  const CanvasProposalJoints = useMemo(() => aiProposalIkPreview ? sceneJointsFromDisplayJoints(aiProposalIkPreview.joints)
    : AiDirectServoPreviewAngles ? sceneJointsFromDisplayJoints(forwardDisplayKinematics(AiDirectServoPreviewAngles)) : undefined,
  [aiProposalIkPreview, AiDirectServoPreviewAngles]);

  return (
    <div className="app-shell dashboard-app" data-mode={ControlMode} data-scanlines={DisplaySettings.ScanLines}>
      <header className="topbar">
        <div className="brand">
          <div className="brand-mark">
            <Activity size={18} strokeWidth={2.2} />
          </div>
          <div>
            <h1>Clancy <small>Robot workspace</small></h1>
          </div>
        </div>

        <div className="annunciators" role="status" aria-label="Annunciator panel">
          {Annunciators.map((Lamp) => (
            <div className="annunciator" key={Lamp.Label} data-lit={Lamp.Lit} title={Lamp.Title}>
              <b>{Lamp.Label}</b><small>{Lamp.State}</small>
            </div>
          ))}
        </div>

        <div className="topbar-actions">
          <button className="emergency-button topbar-emergency" onClick={emergencyShutoff}>
            <OctagonAlert size={17} /><span>{EmergencyStopLabel(HandGuideState)}</span>
          </button>
          <div className={`connection-pill ${socketState}`}>
            {socketState === "live" ? <Wifi size={15} /> : <WifiOff size={15} />}
            <span>{socketState === "live" ? "Bridge live" : "Local"}</span>
          </div>
          <select
            className="port-select"
            value={selectedPort}
            title="Serial port"
            onChange={(event) => setSelectedPort(event.target.value)}
            onFocus={() => void loadPorts()}
          >
            <option value="">{ports.length ? "Select port" : "No port"}</option>
            {ports.map((port) => (
              <option key={port} value={port}>{port}</option>
            ))}
          </select>
          {!ThisPc && RemoteRefusal && !AccessCardOpen && <button className="chip-button access-chip" aria-label="Locked" title={RemoteRefusal.Text}
            onClick={() => { AccessCardDismissedRef.current = false; SetAccessCardOpen(true); }}>
            <KeyRound size={15} /><span>Locked</span>
          </button>}
          <button className="chip-button" title={armConnected ? "Support the arm before disconnecting; this releases torque"
            : ThisPc ? "Connect the arm" : "Connect the arm (from another device it never connects by itself)"} disabled={BridgeRestarting && !armConnected} onClick={() => armConnected ? DisconnectArm() : void connectAll()}>
            <PlugZap size={16} />
            <span>{armConnected ? "Disconnect" : "Connect"}</span>
          </button>
          <button className="icon-button" title="Restart bridge" disabled={BridgeRestarting} onClick={() => void restartBridge()}>
            <RotateCcw size={18} />
          </button>
        </div>
      </header>

      <DashboardLayout Mode={ControlMode} OnModeChange={ChangeControlMode}
        OnSettings={() => SetSettingsOpen(Previous => !Previous)} SettingsOpen={SettingsOpen}
        Left={<>
          <DashboardSection SectionId="AiControl" Title="AI Control" Mode={ControlMode} Detail={aiStatus} Hidden={ControlMode !== "Ai"} ClassName="ai-panel">
            <AiCompanion Model={aiModel} Busy={aiBusy} Moving={RouteActive} Waiting={Boolean(aiSettleWait)}
              HasProposal={Boolean(aiMotionDisplay)} Fault={ProtectionState.Reason || null} Status={aiStatus} />
            {CameraGateBanner ? (
              // Stays until a later capture passes the camera check.
              <div className="camera-gate-banner" role="alert">
                <strong>{CameraGateBanner.Title}</strong>
                {CameraGateBanner.Detail ? <small>{CameraGateBanner.Detail}</small> : null}
              </div>
            ) : null}
            <form className="ai-input-row" onSubmit={submitAiPrompt}>
              <input
                aria-label="AI task"
                value={aiInput}
                placeholder="Describe a task or movement"
                disabled={aiBusy}
                onChange={(event) => setAiInput(event.target.value)}
              />
              <button className="icon-button" title="Send to AI" disabled={aiBusy || !aiInput.trim() || !aiModel}>
                {aiBusy ? <Loader2 size={18} className="spin" /> : <MessageSquare size={18} />}
              </button>
              <button type="button" className="icon-button" aria-label="Reset AI context"
                title="Reset AI context: end any task and forget the conversation and the notepad" onClick={ResetAiContext}>
                <Eraser size={18} />
              </button>
            </form>

            <div className="ai-route-status" role="status">{AiRouteStatus}</div>
            {aiTask || aiSettleWait || aiBusy ? (
              <div className="ai-task-strip">
                <span>
                  {taskRecording ? <span className="ai-task-rec" title="Webcam is being recorded until the task ends">REC</span> : null}
                  {aiTask ? `Task loop: motion ${aiTask.step}/${MAX_AI_TASK_STEPS}` : aiSettleWait ? "Approved motion" : "AI request"}
                  {aiSettleWait?.Chain ? ` · chained motion, part ${aiSettleWait.Chain.Part} of 2: waiting for it to settle`
                    : aiSettleWait ? " · waiting for motion to settle" : aiBusy ? " · waiting for AI" : " · awaiting approval"}
                </span>
                <button type="button" onClick={() => stopAiTask()}>Stop task</button>
              </div>
            ) : null}

            {aiTargetEstimateVisual ? (
              <div className="ai-target-estimate">
                <span>Estimated object position</span>
                <div>
                  {/* Visual frame (X negated from the AI's left-positive frame) so
                      these numbers match the Target / Current Position panels. */}
                  <b>X {formatCm(aiTargetEstimateVisual.x)}</b>
                  <b>Y {formatCm(aiTargetEstimateVisual.y)}</b>
                  <b>Z {formatCm(aiTargetEstimateVisual.z)}</b>
                </div>
              </div>
            ) : null}

            <div className={`ai-approval-box ${aiMotionDisplay ? "ready" : "idle"}`}>
              {aiMotionDisplay && <div className="ai-motion-copy">
                <h2>{aiMotionDisplay.planned ? "App's planned step" : "Proposed target"}</h2>
                <div className="coordinate-grid">
                  {coordinateTiles(futurePoint).map((item) => (
                    <div className="coordinate-tile" key={item.axis}>
                      <span>{item.axis}</span>
                      <strong>{formatCm(item.value)}</strong>
                      <small>cm</small>
                    </div>
                  ))}
                </div>
                <div className="delta-row">
                  <span>Difference from the arm's current tip</span>
                  <div>
                    {/* dX is shown in the visual frame (left-positive negated) so its
                        sign matches the X tiles above; dY/dZ are the same in both frames. */}
                    <b>dX {signedCm(-futureDelta.left)}</b>
                    <b>dY {signedCm(futureDelta.farther)}</b>
                    <b>dZ {signedCm(futureDelta.up)}</b>
                  </div>
                </div>
                {aiMotionDisplay?.servoTargets.length ? (
                  <div className="servo-proposal-grid">
                    {aiMotionDisplay.servoTargets.map((item) => (
                      <div className="servo-proposal-tile" key={item.name}>
                        <span>{formatServoLabel(item.name)}</span>
                        <strong>{formatServoTarget(item, servos)}</strong>
                      </div>
                    ))}
                  </div>
                ) : null}
                {aiMotionDisplay?.chain && aiMotionDisplay.hasTargetMotion ? (
                  // One approval runs both legs, in this order (lib/AiChain.ts); an opening
                  // gripper_before, with a bridge that takes it, as one motion.
                  <div className="ai-chain-steps" aria-label="Chained motion">
                    <span>{GripperWithTip && aiMotionDisplay.chain.Together
                      ? "Chained motion: one approval, one motion: the gripper opens while the tip moves"
                      : "Chained motion: one approval runs both, in this order"}</span>
                    <ol>
                      {chainCardSteps(aiMotionDisplay, servos, GripperWithTip).map((Step) => <li key={Step}>{Step}</li>)}
                    </ol>
                  </div>
                ) : null}
                {aiMotionDisplay && !aiMotionDisplay.hasTargetMotion ? (
                  <span>No XYZ target move requested.</span>
                ) : (
                  <span>
                    This is {directionalCm(futureDelta.left, "left", "right")},{" "}
                    {directionalCm(futureDelta.farther, "farther", "closer")},{" "}
                    {directionalCm(futureDelta.up, "up", "down")}
                  </span>
                )}
                {aiMotionDisplay?.hasTargetMotion || !aiMotionDisplay ? (
                  <span>
                    Wrist pitch target {formatTenths(displayedWristPitchTarget)} deg,
                    change {proposedPitchDelta >= 0 ? "+" : ""}
                    {formatTenths(proposedPitchDelta)} deg
                  </span>
                ) : null}
                {aiDisplayedIssues.length ? (
                  <span className="ai-issue-text">
                    IK check: {compactOutputText(aiDisplayedIssues.join(" "), 120)}
                  </span>
                ) : null}
                {aiSnapNote ? (
                  <span className="ai-snap-note" data-snap={aiMotionDisplay?.snap?.Kind}>{aiSnapNote}</span>
                ) : aiMotionWasAdjusted ? (
                  <span className="ai-issue-text">
                    Approval uses the nearest possible adjusted target.
                  </span>
                ) : null}
                {aiMotionDisplay?.approvalNote ? (
                  <span className="ai-issue-text" data-planned-step={aiMotionDisplay.planned ? "true" : undefined}>{aiMotionDisplay.approvalNote}</span>
                ) : null}
                {approvalDiffersFromDisplayed ? (
                  <span className="ai-issue-text">
                    Approval target changed after feasibility revision.
                  </span>
                ) : null}
              </div>}
              {aiProposal ? (
                <div className="ai-approval-actions" data-camera-keep-clear="true">
                  <button className="action-button primary" onClick={approveAiMotion}>
                    <Check size={17} />
                    <span>Approve</span>
                  </button>
                  <button className="action-button danger" onClick={blockAiMotion}>
                    <ShieldX size={17} />
                    <span>Block</span>
                  </button>
                </div>
              ) : aiMotionDisplay ? (
                <div className="ai-approval-actions single" data-camera-keep-clear="true">
                  <button className="action-button danger" onClick={blockAiMotion}>
                    <ShieldX size={17} />
                    <span>Clear</span>
                  </button>
                </div>
              ) : (
                <div className="pending-note">Movement proposals appear here for your approval.</div>
              )}
            </div>

            {aiLastReply && <AiReply Text={aiLastReply} />}
          </DashboardSection>
          <DashboardSection SectionId="AiStatistics" Title="AI Statistics" Mode={ControlMode} Detail={aiBusy ? "working" : "session"} Hidden={ControlMode !== "Ai"}>
            <AiStatisticsPanel Statistics={AiStats} Model={aiModel} ContextLength={aiContextLength} Busy={aiBusy} />
          </DashboardSection>
        </>}
        Scene={<>
          <div className="scene-toolbar">
            <div>
              <strong>Arm Workspace</strong>
              <span>{status}</span>
            </div>
            <div className="compact-group" hidden={ControlMode === "Ai"}>
              <button className="chip-button" onClick={setCurrentTarget}>
                <Target size={16} />
                <span>Set Current</span>
              </button>
              <button className="chip-button" onClick={toggleElbow}>
                <RotateCcw size={16} />
                <span>Elbow {elbowMode}</span>
              </button>
            </div>
          </div>
          <ArmCanvas
            ref={armCanvasRef}
            Settings={DisplaySettings}
            Mode={ControlMode}
            SelectedJoint={SelectedJoint} SelectedAxis={SelectedAxis} Servos={servos} OnSelectJoint={CanvasSelectJoint}
            OnJointChange={ControlMode === "Manual" && !GuideActive ? CanvasJointChange : undefined}
            angles={angles}
            previewAngles={ControlMode === "Ai" ? aiProposalIkPreview?.angles ?? AiDirectServoPreviewAngles ?? angles : ikPreview.angles}
            proposalAngles={ControlMode === "Ai" ? undefined : aiProposalIkPreview?.angles ?? AiDirectServoPreviewAngles ?? undefined}
            alignmentRecorded={GetCadAlignment().Verified}
            canCaptureAlignment={armConnected && !torque && CadCompatibleRef.current}
            onCaptureAlignment={CanvasCaptureAlignment}
            joints={renderJoints}
            previewJoints={renderPreviewJoints}
            previewReachable={ControlMode === "Ai" ? aiProposalIkPreview?.reachable ?? true : ikPreview.reachable}
            proposalJoints={CanvasProposalJoints}
            proposalReachable={aiProposalIkPreview?.reachable ?? true}
            proposalTarget={renderProposalTarget}
            proposalEffectiveWristPitch={aiProposalIkPreview?.effectiveWristPitch}
            targetWristPitchReachable={targetWristPitchReachable}
            effectiveWristPitch={targetEffectiveWristPitch}
            pitchGuide={ControlMode === "Ai" ? NoPitchGuide : PitchGuide?.Key === PitchGuideKey ? PitchGuide.Segments : null}
            target={ControlMode === "Ai" && renderProposalTarget ? renderProposalTarget : visualTarget}
            TargetVisible={ControlMode === "Manual" || !!renderProposalTarget}
            TargetReadOnly={ControlMode === "Ai" || GuideActive}
            targetEstimate={aiTargetEstimateVisual}
            objectBox={aiObjectBoxVisual}
            targetEditing={TargetEditing}
            targetSnapping={TargetSnapping}
            onTargetDragStart={CanvasTargetDragStart}
            onTargetDragEnd={CanvasTargetDragEnd}
            onTargetChange={CanvasTargetChange}
          />
          <HudNotice Messages={log} />
        </>}
        Joints={
          <DashboardSection SectionId="Joints" Title="Joints" Mode={ControlMode} Detail="raw position" ClassName="servo-panel-card">
            <BotDiagnostics Servos={servos} Temperatures={ProtectionState.Temperatures} TemperatureLimitC={ProtectionState.LimitC} SelectedJoint={SelectedJoint}
              Status={socketState !== "live" ? "OFFLINE" : ProtectionState.Reason ? "TRIP" : !ProtectionState.Loaded ? "PENDING" : "NOMINAL"} />
            <ServoPanel
              servos={servos}
              SelectedJoint={SelectedJoint} OnSelectJoint={SelectJoint}
              onChange={updateServo}
              onGuiAngleChange={updateServoGuiAngle}
              onLimitsChange={updateServoLimits}
              Temperatures={ProtectionState.Temperatures}
            />
          </DashboardSection>
        }
        Controls={<>
          <DashboardSection SectionId="CurrentPosition" Title="Current Position" Mode={ControlMode} Detail="tip" ClassName="current-position-panel">
            {ControlMode === "Manual" && <TipRadar Tip={currentTip} Target={visualTarget} />}
            <div className="position-grid">
              {currentPositionTiles.map((item) => (
                <div className="position-tile" key={item.axis}>
                  <span>{item.axis}</span>
                  <strong>{item.unit === "cm" ? formatCm(item.value) : formatTenths(item.value)}</strong>
                  <small>{item.unit}</small>
                </div>
              ))}
            </div>
          </DashboardSection>

          <DashboardSection SectionId="Target" Title="Target" Hidden={ControlMode === "Ai"} Mode={ControlMode} Detail="centimeters">
            {ikPreview.collisionReason && <p className="warn-text" role="status">Blocked: {ikPreview.collisionReason}.</p>}
            <div className="target-grid">
              {(["x", "y", "z"] as const).map((axis) => (
                <label key={axis}>
                  <span>{axis.toUpperCase()}</span>
                  <input
                    aria-label={`Target ${axis.toUpperCase()}`}
                    type="number"
                    step="0.001"
                    value={Number(visualTarget[axis].toFixed(3))}
                    onFocus={() => { BeginTargetEdit(); SetSelectedAxis(axis); SetSelectedJoint(null); }}
                    onBlur={() => { FinishTargetEdit(); SetSelectedAxis(null); }}
                    onKeyDown={Event => { if (Event.key === "Enter") Event.currentTarget.blur();
                      if (Event.key === "Escape") { FinishTargetEdit(true); Event.currentTarget.blur(); } }}
                    onChange={(event) =>
                      PreviewVisualTarget({
                        ...visualTarget,
                        [axis]: Number(event.target.value),
                      })
                    }
                  />
                </label>
              ))}
            </div>
            <label className="wide-field">
              <span>Wrist pitch</span>
              <input
                type="number"
                step="1"
                value={Number(target.wristPitch.toFixed(1))}
                onFocus={() => BeginTargetEdit("Pitch")}
                onBlur={() => FinishTargetEdit()}
                onKeyDown={Event => { if (Event.key === "Enter") Event.currentTarget.blur();
                  if (Event.key === "Escape") { FinishTargetEdit(true); Event.currentTarget.blur(); } }}
                onChange={(event) =>
                  PreviewManualTarget({
                    ...(TargetEditRef.current?.Candidate ?? target),
                    wristPitch: Number(event.target.value || 0),
                  })
                }
              />
              <b className={targetWristPitchReachable ? undefined : "warn-text"}>
                {targetWristPitchReachable
                  ? `${target.wristPitch.toFixed(0)} deg`
                  : `closest ${targetEffectiveWristPitch.toFixed(0)} deg`}
              </b>
            </label>
          </DashboardSection>

          <DashboardSection SectionId="Motion" Title="Motion" Mode={ControlMode} Detail={torque ? "holding" : "coast"}>
            {socketState === "live" && BridgeUpdate && <div className="bridge-update-status" role="status" id="bridge-update-status">
              <strong>{BridgeUpdate}</strong><small>{armConnected || torque
                ? "Support the arm and click Disconnect, then load the update."
                : "Load the updated bridge before sending motion."}</small>
              <button type="button" disabled={armConnected || torque || BridgeRestarting} onClick={() => void restartBridge()}>
                {BridgeRestarting ? "Loading bridge update..." : "Load bridge update"}</button>
            </div>}
            <div className={`protection-status ${ProtectionState.Reason ? "tripped" : ""}`} role="status"><ShieldX size={15} /><div><strong>{socketState !== "live" ? "Protection offline" : !ProtectionState.Loaded ? "Protection update pending" : ProtectionState.Reason ? ProtectionState.CoastConfirmed ? "Protection tripped · coasting" : "Torque-off unconfirmed" : "Motor guard ready"}</strong><small>{ProtectionState.Reason || (ProtectionState.Loaded ? `${ProtectionState.Hottest === null ? "Temperature pending" : `${ProtectionState.Hottest} °C peak`} · thermal / stall / feedback` : "Load the updated bridge to enable automatic coast")}</small></div></div>
            {GuideStrip && <div className={`protection-status hand-guide-status ${GuideStrip.Tone}`} role="status" aria-live="polite">
              <Hand size={15} /><div><strong>{GuideStrip.Title}</strong><small>{GuideStrip.Detail}</small></div></div>}
            <MotionThrottle Percent={MotionSpeedPercent} OnChange={ChangeMotionSpeed}
              DisabledReason={socketState !== "live" ? "Connect the bridge to set speed"
                : !SpeedCompatible ? "Reload the bridge to enable speed control" : null} />
            <div className="button-grid">
              <button hidden={ControlMode === "Ai"} className="action-button primary" onClick={sendTarget} disabled={BridgeRestarting || TargetSnapping || !ikPreview.reachable || GuideActive}
                aria-describedby={BridgeUpdate ? "bridge-update-status" : undefined}>
                <Send size={17} />
                <span>Send</span><i className="hud-code" aria-hidden="true" data-code="EXEC" />
              </button>
              <button className={`action-button ${CoastDrops ? "danger" : ""}`} onClick={toggleTorque}
                aria-label={CoastDrops ? "Coast" : undefined} title={CoastDrops?.Title}>
                <PlugZap size={17} />
                <span>{CoastDrops ? CoastDrops.Text : torque ? "Coast" : "Hold"}</span><i className="hud-code" aria-hidden="true" data-code="PWR" />
              </button>
              <button className="action-button" onClick={goHome} disabled={GuideActive} title={GuideActive ? "Press Hold here first" : undefined}>
                <Home size={17} />
                <span>Home</span><i className="hud-code" aria-hidden="true" data-code="RTB" />
              </button>
              <button className="action-button" onClick={setHome} disabled={GuideActive} title={GuideActive ? "Press Hold here first" : undefined}>
                <Save size={17} />
                <span>Set Home</span><i className="hud-code" aria-hidden="true" data-code="MEM" />
              </button>
              <button className="action-button stop-hold" onClick={StopAndHold}>
                <ShieldX size={17} />
                <span>Stop and hold</span><i className="hud-code" aria-hidden="true" data-code="BRK" />
              </button>
            </div>
            {/* Hand guiding: the Hand guide toggle and its narrow Calibrate again, in the slot the Manual HUD budgets for them
                (shown in AI mode too while guiding, so Hold here stays in reach). For a bridge without hand guiding the slot
                stays empty and ManualHud.css shows its dimmed "reserved" row. */}
            {(ControlMode === "Manual" || GuideActive) && <div className="hand-guide-slot" data-slot="hand-guide">
              {HandGuideState.Version !== null && <div className="hand-guide-pair">
                <button className={`action-button hand-guide-button ${GuideButton.Primary ? "primary" : ""}`}
                  onClick={() => PressHandGuide(GuideButton)} aria-disabled={GuideButton.DisabledReason !== null}
                  title={GuideButton.DisabledReason ?? GuideButton.Title}>
                  <Hand size={17} />
                  <span>{GuideButton.Label}</span>
                </button>
                {GuideRecalibrate && <button className="hand-guide-recalibrate" aria-label="Calibrate hand guiding again"
                  aria-disabled={GuideButton.DisabledReason !== null} onClick={RecalibrateHandGuide}
                  title={`Calibrate hand guiding again (calibrated ${HandGuideState.CalibratedAt ?? ""}): after changing a servo, its settings or the zero pose`}>
                  <RotateCcw size={14} />
                </button>}
              </div>}
            </div>}
          </DashboardSection>


        </>}
      />

      {SettingsOpen && <SettingsPanel Settings={DisplaySettings} CameraLocked={selfCal.running || taskRecording} OnChange={ChangeDisplaySettings} OnClose={() => SetSettingsOpen(false)}
        GraspGapCm={GraspGapCm} OnGraspGapChange={ChangeGraspGap} OnForgetRemotePassword={ThisPc ? undefined : ForgetRemotePassword}
        CameraStatus={cameraStatus} DepthCameraStatus={depthCameraText} OnReconnectCamera={() => SetCameraReconnectKey(Value => Value + 1)}
        OnPopoutCamera={document.pictureInPictureEnabled ? () => void ToggleCameraPopout() : undefined}
        OnCalibrate={() => { SetSettingsOpen(false); OpenFloatingCalibration(); }} />}

      {!ThisPc && AccessCardOpen && <AccessCard Problem={RemoteRefusal} Host={BRIDGE_HOST} OnSubmit={UnlockBridge} OnRetry={() => UnlockBridge(null)}
        OnClose={DismissAccessCard} />}

      <LiveCamera VideoRef={videoRef} Profile={DisplaySettings.CameraProfile} Source={cameraSource} DepthFrameUrl={`${API_URL}/depthcam/frame.jpg`}
        OnStatus={setCameraStatus} OnActive={setCameraActive} Countdown={captureCountdown} ReconnectKey={CameraReconnectKey}
        DepthOverlay={DisplaySettings.DepthOverlay} DepthOverlayUrl={`${API_URL}/depthcam/overlay.png`} OnDepthOverlayChange={ChangeDepthOverlay} />

      <SemiCalibrationStage ApiUrl={API_URL} Running={selfCal.running} OnStop={stopSelfCalibration} OnEmergencyStop={emergencyShutoff}
        OnFit={fitSemiCalibration} OnReject={rejectCalibration} Hottest={ProtectionState.Hottest} />
      {calOpen ? (
        <div
          className="cal-overlay"
          onClick={(event) => {
            if (event.target === event.currentTarget) setCalOpen(false);
          }}
        >
          <div className="cal-panel">
            <div className="cal-header">
              <strong>Camera calibration</strong>
              <button type="button" className="icon-button" title="Close" onClick={() => setCalOpen(false)}>
                <X size={16} />
              </button>
            </div>
            <section className="cal-section">
              <div className="cal-section-title">Automatic</div>
              <p className="cal-instructions">
                Keep the red tape on the gripper tip visible. Press <b>Check marker</b>, then click the red tip
                in the image to identify it. Yellow outlines are candidates; green marks your selected tip.
                The fixed gray/beige masking tape is at X={formatTenths(FixedAnchorVisualCm.x)}, Y={formatTenths(FixedAnchorVisualCm.y)} and is not the moving marker.
                Calibration checks the tip before moving, keeps joints holding between poses, and stops on lost
                feedback or a motor fault. Keep the workspace clear.

              </p>
              <div className={`protection-status ${ProtectionState.Reason ? "tripped" : ""}`} role="status"><ShieldX size={15} /><div><strong>{socketState !== "live" ? "Protection offline" : !ProtectionState.Loaded ? "Protection update pending" : ProtectionState.Reason ? ProtectionState.CoastConfirmed ? "Protection tripped · coasting" : "Torque-off unconfirmed" : "Motor guard ready"}</strong><small>{ProtectionState.Reason || (ProtectionState.Loaded ? `${ProtectionState.Hottest === null ? "Temperature pending" : `${ProtectionState.Hottest} °C peak`} · thermal / stall / feedback` : "Load the updated bridge to enable automatic coast")}</small></div></div>
            <MotionThrottle Percent={MotionSpeedPercent} OnChange={ChangeMotionSpeed}
                DisabledReason={socketState !== "live" ? "Connect the bridge to set speed"
                  : !SpeedCompatible ? "Reload the bridge to enable speed control" : null} />
              {selfCal.step ? (
                <div className="selfcal-progress">
                  <span>{selfCal.step}</span>
                  {selfCal.total ? <span>{selfCal.seen} of {selfCal.index} poses saw the marker</span> : null}
                </div>
              ) : null}
              {selfCal.report ? (() => {
                const Report = selfCal.report, Outcome = CalibrationOutcomeOf(Report);
                return <div className={`selfcal-report ${Outcome.Kind === "saved" ? "ok" : Outcome.Kind === "saved_with_warnings" || Outcome.Kind === "rejected" ? "warn" : "bad"}`}>
                  <strong>{Outcome.Title}</strong>
                  {Report.saved || Report.rejected ? (
                    <span>
                      About {Report.plane_rms_cm?.toFixed(2) ?? "?"} cm on the table ·{" "}
                      {Report.fit_rms_cm?.toFixed(2) ?? "?"} cm fit ({Report.fit_rms_px?.toFixed(1) ?? "?"} px) ·{" "}
                      {Report.holdout_rms_cm?.toFixed(2) ?? "?"} cm on held-out poses ·{" "}
                      {Report.observations_used ?? "?"} poses used
                      {Report.tape_assignment?.changed_from_tracker
                        ? ` · tape re-identified at ${Report.tape_assignment.changed_from_tracker}`
                        : ""}
                    </span>
                  ) : Outcome.Detail ? <span>{Outcome.Detail}</span> : null}
                  {Outcome.Problems.length ? (
                    <ul className="selfcal-problems" aria-label="Calibration warnings">
                      {Outcome.Problems.map(Problem => <li key={Problem}>{Problem}</li>)}
                    </ul>
                  ) : null}
                  {Outcome.Kind === "saved_with_warnings" && Outcome.Detail ? <span>{Outcome.Detail}</span> : null}
                  {Report.warnings?.length ? (
                    <span className="selfcal-warn">{Report.warnings.join(" · ")}</span>
                  ) : null}
                </div>;
              })() : null}
              {calStatus?.mode === "auto" ? (
                // The calibration in use, whatever run saved it (also after a reload): its warnings and the
                // only way a calibration is ever rejected, the user's own Reject calibration.
                <div className={`selfcal-report selfcal-current ${calStatus.quality_problems?.length ? "warn" : ""}`}>
                  <strong>In use: the calibration fitted at {FittedClock(calStatus.fitted_at) ?? "?"}
                    {calStatus.observation_source === "manual" ? " from your clicks" : ""}
                    {calStatus.quality_problems?.length ? `, with ${calStatus.quality_problems.length} warning${calStatus.quality_problems.length === 1 ? "" : "s"}` : ""}</strong>
                  {calStatus.quality_problems?.length ? (
                    <ul className="selfcal-problems" aria-label="Warnings of the calibration in use">
                      {calStatus.quality_problems.map(Problem => <li key={Problem}>{Problem}</li>)}
                    </ul>
                  ) : null}
                  {calStatus.drift_baseline === false ? (
                    <span className="selfcal-warn">No camera-check baseline: tasks that act on an object stop (camera not verified) until a calibration with one is saved.</span>
                  ) : null}
                  <div className="cal-actions selfcal-reject">
                    <button type="button" onClick={() => void rejectCalibration(calStatus.fitted_at ?? null)}
                      disabled={calBusy || selfCal.running || !calStatus.can_reject}
                      title={calStatus.can_reject
                        ? `Restore the calibration this one replaced${calStatus.replaced_fitted_at ? ` (fitted at ${FittedClock(calStatus.replaced_fitted_at)})` : ""}`
                        : "This calibration was saved before rejecting was possible, so it names no calibration to restore"}>
                      Reject calibration
                    </button>
                  </div>
                </div>
              ) : null}
              {calStatus?.last_rejected && calStatus.mode !== "auto" ? (
                <div className="selfcal-report">
                  <span>The calibration fitted at {FittedClock(calStatus.last_rejected.fitted_at) ?? "?"} was rejected; the camera is {calStatus.calibrated ? "on the manual calibration" : "not calibrated"} now.</span>
                </div>
              ) : null}
              {RedPreview ? <figure>
                <img ref={RedPreviewImageRef} src={RedPreview} alt="Click the red tape on the gripper tip" style={{ width: "100%", cursor: "crosshair" }}
                  onClick={Event => {
                    if (selfCalRunningRef.current) return;
                    const Image = Event.currentTarget, Point = RedPreviewPointRef.current ?? ImagePointAt(Image, Event.clientX, Event.clientY, false);
                    if (!Point) return;
                    const Click = ClickCapturePoint(Point);
                    RedHintRef.current = { u: Click.U / Image.naturalWidth, v: Click.V / Image.naturalHeight };
                    void checkMarker();
                  }} />
                <HoverLoupe ElementRef={RedPreviewImageRef} PointRef={RedPreviewPointRef} Watch={RedPreview} Name="Marker preview magnifier" />
                <figcaption>Click the red tip, then confirm its green outline.</figcaption>
              </figure> : null}
              <button type="button" className="emergency-button" onClick={emergencyShutoff}>{EmergencyStopLabel(HandGuideState)}</button>
              {markerCheck.text ? (
                <div className={`selfcal-report ${markerCheck.ok === null ? "" : markerCheck.ok ? "ok" : "bad"}`}>
                  <span>{markerCheck.text}</span>
                </div>
              ) : null}
              <div className="cal-actions">
                <button
                  type="button"
                  onClick={() => void checkMarker()}
                  disabled={markerCheck.busy || selfCal.running}
                  title="Look for the marker in the live camera now, without moving the arm"
                >
                  {markerCheck.busy ? "Checking" : "Check marker"}
                </button>
                {selfCal.running ? (
                  <button type="button" onClick={stopSelfCalibration}>Stop</button>
                ) : (<>
                  <button
                    type="button"
                    className="cal-fit"
                    onClick={() => void runSelfCalibration()}
                    disabled={calBusy || socketState !== "live" || !armConnected}
                  >
                    Start automatic calibration
                  </button>
                  <button
                    type="button"
                    className="cal-fit"
                    onClick={() => void runSelfCalibration("semi")}
                    disabled={calBusy || socketState !== "live" || !armConnected}
                    title="The arm visits the same poses; at each one the frame freezes and you click the tape"
                  >
                    Start semi-automatic calibration
                  </button>
                  <SemiCalReopenButton />
                </>)}
              </div>
              <p className="cal-instructions">
                <b>Semi-automatic</b> moves through the same poses with the same checks, but at each pose the frame
                freezes and you click the centre of the red tape on the fixed jaw (a magnifier follows the pointer);
                the arm holds the pose until you click, skip or retake it. Use it when the automatic search misses
                the tape or picks other red.
              </p>
              <p className="cal-instructions">
                Every run that can compute a camera is <b>saved and applied</b>. Anything that looks wrong (fit error,
                camera motion during the run, no camera-check baseline, few poses) is listed as a warning with its
                numbers; only you can reject a calibration, with <b>Reject calibration</b>, which restores the one it replaced.
              </p>
            </section>

            <div className="cal-section-title">Manual</div>
            <p className="cal-instructions">
              Jog the arm so the gripper tip touches the table, click <b>Refreeze</b>, then click the
              tip's exact pixel in the image. Repeat for {calStatus?.recommended_points ?? 6}+ spots spread
              across the whole view (corners too), then <b>Fit</b>. After that, every AI capture includes
              measured object positions accurate to about a centimeter.
            </p>
            <div className="cal-frame">
              {calFrame ? (<>
                <img
                  ref={CalFrameImageRef}
                  src={calFrame.url}
                  alt="Frozen camera frame: click the gripper tip"
                  // A capture flipped for the AI (calFrame.mirrored) is flipped back for
                  // DISPLAY so it matches the live feed. Click coords are mirrored back
                  // into capture space in onCalFrameClick.
                  style={calFrame.mirrored ? { transform: "scaleX(-1)" } : undefined}
                  onClick={onCalFrameClick}
                />
                <HoverLoupe ElementRef={CalFrameImageRef} PointRef={CalFramePointRef} MirroredDisplay={calFrame.mirrored} Watch={calFrame.url}
                  Name="Frozen frame magnifier" />
              </>) : (
                <div className="cal-noframe">No camera frame: allow camera access, then Refreeze.</div>
              )}
            </div>
            <div className="cal-status">
              {calStatus
                ? [
                    `${calStatus.point_count} point${calStatus.point_count === 1 ? "" : "s"}`,
                    calStatus.calibrated
                      ? calStatus.mode === "auto"
                        ? `calibrated automatically (about ${calStatus.plane_rms_cm?.toFixed(2) ?? "?"} cm, ${calStatus.observations_used ?? "?"} poses)`
                        : `calibrated (RMS ${calStatus.rms_cm?.toFixed(2) ?? "?"} cm)`
                      : `not calibrated (need ${calStatus.min_points}+)`,
                    calStatus.mode === "auto" || calStatus.has_intrinsics
                      ? "lens profile OK"
                      : "no lens profile: run calibrate_intrinsics.py for edge accuracy",
                  ].join(" · ")
                : "Bridge unreachable"}
            </div>
            {calMessage ? <div className="cal-message">{calMessage}</div> : null}
            <div className="cal-actions">
              <button type="button" onClick={freezeCalFrame} disabled={calBusy}>Refreeze frame</button>
              <button
                type="button"
                onClick={() => void calRequest("/calibration/undo")}
                disabled={calBusy || !calStatus?.point_count}
              >
                Undo point
              </button>
              <button
                type="button"
                onClick={() => void calRequest("/calibration/clear")}
                disabled={calBusy || !calStatus?.point_count}
              >
                Clear all
              </button>
              <button
                type="button"
                className="cal-fit"
                onClick={() => void fitCalibration()}
                disabled={calBusy || (calStatus?.point_count ?? 0) < (calStatus?.min_points ?? 4)}
              >
                Fit
              </button>
            </div>
          </div>
        </div>
      ) : null}
    </div>
  );
}

export default App;
