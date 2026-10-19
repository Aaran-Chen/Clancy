import { BeyondArmReach, InsetWristPitch, PreviewConverges, WristPitchConverges } from "./AutomaticWrist";
import { CadSolve } from "./CadKinematics";
import { currentAngles, GetIkAngleLimits, mirrorDisplayPoint, modelPointFromDisplayPoint, solveIkPreview, type IkAngleLimits, type Point3,
  type ServoName, type ServoState } from "./robotModel";
import { NamesQuery, ParseTaskObject, TaskIntent, type TaskIntentKind, type TaskObjectBinding, type TaskObjectQuery } from "./TaskObject";
import { AtHoverPoint, FloorPrism, FootprintDistance, GraspPrism, GraspRules, GuardPrisms, ObjectPrism, PlanClearanceCm, PlanGrasp, ScenePrisms, StraddleAtTip, TapeCorrection, TapeText, UnmeasuredRefusal,
  type GraspGuard, type GraspPlan, type GraspPlanState, type GraspRefusal, type HeightBy, type PlannedStep, type Prism, type TapeFix, type XY } from "./GraspPlan";
import { SweepRules } from "./JawSweep";
import { GraspGapHint } from "./GraspGap";
import { HiddenText, type ArmOcclusion } from "./ArmOcclusion";
import { DeclaredSurfaceList, SurfaceContains, SurfaceText, type DeclaredSurface } from "./DeclaredSurfaces";
import { DeskFloor, FloorText, SurfaceFloor, type AiFloor } from "./AiTableFloor";
import ReachEnvelope from "./ReachEnvelope.json";
import { HasVerifiedDepthFrame, ReadMeasuredScene, ReadMeasuredSupport, type MeasuredScene, type MeasuredSupport } from "./SceneGeometry";

export { BindFromEstimate, EditDistance, EstimateGateCm, NamesQuery, ParseTaskObject, ResolveTaskObject, TaskIntent, TrackCm,
  type TaskIntentKind, type TaskObjectBinding, type TaskObjectQuery, type TaskObjectResolution, type TaskObjectSource } from "./TaskObject";

// One calibrated-camera object measurement (bridge /locate), in the AI's frame.
export type LocatedObject = {
  label: string;
  x: number;
  y: number;
  z: number;
  extrapolated: boolean;
  unstable: boolean;
  // Measured by the depth camera: the object's top, height and footprint.
  topZ: number | null;
  heightCm: number | null;
  sizeCm: [number, number] | null;
  depthMeasured: boolean;
  // Whether the surface under the object was measured rather than assumed,
  // and which surface it was (null from a bridge that does not report it).
  supportMeasured: boolean | null;
  supportZMeasured: number | null;
  validFraction: number | null;
  // "surface": a surface besides the desk declared in back end/DeclaredSurfaces.json (lib/DeclaredSurfaces.ts),
  // named by supportSurface, the bridge measured it on; how far its heights lie outside the calibrated band.
  supportBranch: "desk" | "bin" | "object" | "surface" | null;
  supportSurface?: string | null;
  supportGeometry?: MeasuredSupport | null;
  scene?: MeasuredScene | null;
  sceneChecked?: boolean;
  localizationFrame?: "calibrated_depth_3d" | null;
  outsideCalibrationZCm?: number | null;
  // The bridge's strict query for the task's object landed on it, and whether
  // only that query found it (one detection, not voted across the samples).
  taskMatch: boolean;
  taskQueryOnly: boolean;
  // Whether its box was zoom-refined: told the task object, the bridge refines
  // only its boxes, and the rest keep the first pass's, which can be several
  // cm off. How far outside the calibration tips' hull it is (10 cm and more
  // is extrapolated). Both null from a bridge that does not report them.
  refined: boolean | null;
  outsideCalibrationCm: number | null;
  // The share of its box's depth the bridge's arm mask took (0..1); absent from a bridge that
  // does not report it, or one that could not mask the arm (lib/ArmMadeObjects.ts).
  armFraction?: number | null;
  // This check-in's reading was the gripper's (lib/ArmMadeObjects.ts): this is its last reading
  // with the gripper clear of it, taken at this time (ms).
  armClearAt?: number;
};
// The bridge compares the camera pose with the calibration before measuring.
export type CameraCheck = { Ok: boolean; AngleDeg: number | null; OffsetCm: number | null; Reason: string;
  ReferenceFittedAt: string | null; Seq: number | null;
  AlignmentSource?: "robot" | null;
  // With no calibrated desk in view the bridge assumes the camera where it was calibrated and checks it
  // against the gripper tape through every joint (back end/RobotReference.py): "verified" (the tape within
  // 0.75 cm, TapeErrorCm), "assumed" (the tape could not be judged; Warning says why) or "moved" (blocked).
  RobotCheck?: "verified" | "assumed" | "moved"; TapeErrorCm?: number | null; Warning?: string | null;
  // The basket rim's reading there ("ok", "moved", "not_judgeable", "unavailable"): advisory, logged, never deciding.
  RimCheck?: string | null; RimYawDeg?: number | null;
  // The gripper tape check ("ok", "moved", or why it could not judge) and, judged, where the red tape
  // is against where the arm puts it (display frame, measured - predicted): GraspPlan.ts TapeCorrection.
  TapeCheck?: string | null; TapeResidualCm?: Point3 | null;
  // The desk had moved against the arm (the cart was rolled) and, the tape showing the camera had not,
  // the bridge measured it again on this capture; whether it saved it as the new baseline.
  DeskRemeasured?: boolean; DeskSaved?: boolean | null };
// UndeclaredSupportZ: a level surface no DeclaredSurfaces.json entry declares seems to hold it there (the bridge's hint).
// TaskQueryOnly: measured only because the task query's box found it (the task object, under the task's label).
export type WithheldObject = { Label: string; Reason: string; UndeclaredSupportZ?: number; TaskQueryOnly?: true };
export type LocateResult = {
  objects: LocatedObject[];
  rmsCm: number | null;
  error: string | null;
  errorCode: string | null;
  cameraCheck: CameraCheck | null;
  withheld: WithheldObject[];
  scene?: MeasuredScene | null;
  // Why a reply's scene was not used (ReadLocateResponse), for the session log; the objects read as desk objects.
  sceneNote?: string;
};
export type CameraGate = { Code: string; Title: string; Detail: string };

export const CameraGateCodes = ["camera_moved", "camera_check_failed", "camera_not_verified", "desk_changed_unverified"];

// Thrown by the vision capture when a task must not run on this frame's
// measurements; the AI request then stops before any model call.
export class CameraMovedError extends Error {
  Gate: CameraGate;
  constructor(Gate: CameraGate) { super(Gate.Title); this.name = "CameraMovedError"; this.Gate = Gate; }
}

const IsRecord = (Value: unknown): Value is Record<string, unknown> => typeof Value === "object" && Value !== null;
function ReadNumber(Value: unknown): number | null {
  if (typeof Value === "string" && !Value.trim()) return null;
  const Number_ = typeof Value === "number" || typeof Value === "string" ? Number(Value) : Number.NaN;
  return Number.isFinite(Number_) ? Number_ : null;
}

const RobotChecks = ["verified", "assumed", "moved"] as const;

function ReadCameraCheck(Value: unknown): CameraCheck | null {
  if (!IsRecord(Value)) return null;
  const Residual = Array.isArray(Value.tape_residual_cm) && Value.tape_residual_cm.length === 3 ? Value.tape_residual_cm.map(ReadNumber) : null;
  // The tape fields only from a bridge that reports them (an older reply reads as before).
  const Tape = "tape_check" in Value || "tape_residual_cm" in Value ? {
    TapeCheck: typeof Value.tape_check === "string" ? Value.tape_check : null,
    TapeResidualCm: Residual && Residual.every((Item): Item is number => Item != null) ? { x: Residual[0]!, y: Residual[1]!, z: Residual[2]! } : null } : {};
  const Desk = Value.desk_remeasured === true
    ? { DeskRemeasured: true, DeskSaved: typeof Value.desk_saved === "boolean" ? Value.desk_saved : null } : {};
  // The robot's verdict only from a bridge that checked a capture with no desk in view.
  const Verdict = RobotChecks.find(Name => Name === Value.robot_check);
  const Robot = Value.alignment_source === "robot" && Verdict
    ? { RobotCheck: Verdict, TapeErrorCm: ReadNumber(Value.tape_error_cm), Warning: typeof Value.warning === "string" ? Value.warning : null,
      RimCheck: typeof Value.rim_check === "string" ? Value.rim_check : null, RimYawDeg: ReadNumber(Value.rim_yaw_deg) } : {};
  return { Ok: Value.ok === true, AngleDeg: ReadNumber(Value.angle_deg), OffsetCm: ReadNumber(Value.offset_cm),
    Reason: typeof Value.reason === "string" ? Value.reason : "",
    ...(Value.alignment_source === "robot" ? { AlignmentSource: "robot" as const } : {}),
    ReferenceFittedAt: typeof Value.reference_fitted_at === "string" ? Value.reference_fitted_at : null, Seq: ReadNumber(Value.seq), ...Tape, ...Desk, ...Robot };
}

/** Parse a /locate reply; null when it is not a /locate reply at all. */
export function ReadLocateResponse(Data: unknown): LocateResult | null {
  if (!IsRecord(Data)) return null;
  const CameraCheck_ = ReadCameraCheck(Data.camera_check);
  const Withheld = Array.isArray(Data.withheld) ? Data.withheld.flatMap(Item => {
    if (!IsRecord(Item) || typeof Item.label !== "string") return [];
    const Hint = ReadNumber(Item.undeclared_support_z);
    return [{ Label: Item.label, Reason: typeof Item.reason === "string" ? Item.reason : "not measurable", ...(Hint != null ? { UndeclaredSupportZ: Hint } : {}),
      ...(Item.task_query_only === true ? { TaskQueryOnly: true as const } : {}) }];
  }) : [];
  // Say why nothing was measured, so the model knows its own estimates are guesses.
  if (Data.ok === false) {
    return { objects: [], rmsCm: null, error: String(Data.error ?? "measurement failed"),
      errorCode: typeof Data.error_code === "string" ? Data.error_code : null, cameraCheck: CameraCheck_, withheld: Withheld };
  }
  if (Data.ok !== true || !Array.isArray(Data.objects)) return null;
  // The measured scene is optional (Aaran, 2026-10-14: "just remove the scene verification thing"). A scene that
  // reads whole and binds every task match is used as before: its support, free space and obstacles. Without one
  // (none registered, a partial one, one naming another capture, several task matches) the objects read as they did
  // before measured scenes: desk objects, every support check and route the desk's (TaskSupport's desk rules, the
  // bridge's collision world without a scene). SceneNote says why, for the session log only.
  const Decoded = Data.scene == null ? null : ReadMeasuredScene(Data.scene);
  const Binds = (Item: unknown) => !!Decoded && IsRecord(Item) && Item.scene_id === Decoded.id && !!ReadMeasuredSupport(Item.support_geometry);
  const Mismatched = !!Decoded && Data.objects.some(Item => IsRecord(Item)
    && ((Item.scene_id != null && Item.scene_id !== Decoded.id) || (Item.task_match === true && !Binds(Item))));
  const Scene = Decoded && !Mismatched ? Decoded : null;
  const SceneNote = Scene ? null
    : Data.scene != null && !Decoded ? "the bridge's scene did not decode; read as desk objects"
    : Mismatched ? "the bridge's scene did not match the task object; read as desk objects"
    : typeof Data.scene_error === "string" ? `no scene registered (${Data.scene_error}); read as desk objects`
    : Data.objects.some(Item => IsRecord(Item) && (Item.support_geometry != null || Item.scene_id != null)) ? "no scene registered; read as desk objects" : null;
  const objects = Data.objects.flatMap((Item): LocatedObject[] => {
    if (!IsRecord(Item) || typeof Item.label !== "string") return [];
    const x = ReadNumber(Item.x), y = ReadNumber(Item.y), z = ReadNumber(Item.z);
    if (x == null || y == null || z == null) return [];
    const Size = Array.isArray(Item.size_cm) && Item.size_cm.length === 2 && Item.size_cm.every(Number.isFinite)
      ? [Number(Item.size_cm[0]), Number(Item.size_cm[1])] as [number, number] : null;
    const Read = Item.support_branch === "desk" || Item.support_branch === "bin" || Item.support_branch === "object" || Item.support_branch === "surface"
      ? Item.support_branch : null;
    // A support only the camera measured (no declared surface of that name) means nothing without its scene: such an
    // object rests on something under the desk's rules, as before measured scenes ("resting on another object").
    const Observed = Read === "surface" && !(Scene && Binds(Item)) && Item.support_geometry != null
      && !DeclaredSurfaceList.some(Declared => typeof Item.support_surface === "string" && Declared.Name.toLowerCase() === Item.support_surface.toLowerCase());
    const Branch = Observed ? "object" as const : Read;
    // On a declared surface: which, and its heights against the calibration's (only a bridge that measures on surfaces sends them).
    const Surface = Branch === "surface" ? { supportSurface: typeof Item.support_surface === "string" ? Item.support_surface : null,
      outsideCalibrationZCm: ReadNumber(Item.outside_calibration_z_cm) } : {};
    return [{ label: Item.label, x, y, z, extrapolated: Item.extrapolated === true, unstable: Item.unstable === true,
      topZ: ReadNumber(Item.top_z), heightCm: ReadNumber(Item.height_cm), sizeCm: Size, depthMeasured: Item.method === "depth",
      supportMeasured: typeof Item.support_measured === "boolean" ? Item.support_measured : null,
      supportZMeasured: ReadNumber(Item.support_z_measured), validFraction: ReadNumber(Item.valid_fraction), supportBranch: Branch, ...Surface,
      // Only an object bound to the scene in use carries it (and its support); every other one reads as before scenes.
      ...(Scene && Binds(Item) ? { supportGeometry: ReadMeasuredSupport(Item.support_geometry),
        scene: Scene,
        // Not with a camera the gripper tip could not check ("assumed"): it lets the task run, but it is not
        // verified, so its measurements keep their [UNRELIABLE] tags (Aaran's decision, 2026-10-07 audit IH1).
        sceneChecked: !!Scene && Item.scene_id === Scene.id && CameraCheck_?.Ok === true && CameraCheck_.RobotCheck !== "assumed"
          && Item.scene_checked === true && Item.method === "depth",
        localizationFrame: Item.localization_frame === "calibrated_depth_3d" ? "calibrated_depth_3d" as const : null } : {}),
      taskMatch: Item.task_match === true, taskQueryOnly: Item.task_query_only === true,
      refined: typeof Item.refined === "boolean" ? Item.refined : null, outsideCalibrationCm: ReadNumber(Item.outside_calibration_cm),
      ...("arm_fraction" in Item ? { armFraction: ReadNumber(Item.arm_fraction) } : {}) }];
  });
  return { objects, rmsCm: ReadNumber(Data.rms_cm), error: null, errorCode: null, cameraCheck: CameraCheck_, withheld: Withheld, scene: Scene,
    ...(SceneNote ? { sceneNote: SceneNote } : {}) };
}

/** Clock time of the calibration the camera was checked against, for the user. */
export function FormatFittedAt(Value: string | null): string | null {
  if (!Value) return null;
  const Seconds = Number(Value);
  const Parsed = Number.isFinite(Seconds) && Seconds > 1e9 ? new Date(Seconds * 1000)
    : /[zZ]$|[+-]\d{2}:?\d{2}$/.test(Value) ? new Date(Value) : null;
  if (Parsed && Number.isFinite(Parsed.getTime())) {
    return `${String(Parsed.getHours()).padStart(2, "0")}:${String(Parsed.getMinutes()).padStart(2, "0")}`;
  }
  return Value.match(/(?:^|\D)(\d{1,2}:\d{2})/)?.[1] ?? Value;
}

// The offset is signed (farther or nearer); the user needs its size.
function MovementText(Check: CameraCheck | null) {
  return [Check?.AngleDeg != null ? `${Check.AngleDeg.toFixed(1)} deg` : null,
    Check?.OffsetCm != null ? `${Math.abs(Check.OffsetCm).toFixed(1)} cm` : null].filter(Boolean).join(" / ");
}

// The desk-plane limits of the bridge's camera check (CameraDrift.py). A block
// with desk numbers inside them came from another check (the gripper tape, or
// objects not sitting on the desk), so the numbers would say nothing moved.
export const DeskMoveLimitDeg = 0.5;
export const DeskMoveLimitCm = 0.5;

// Grasping, placing or touching needs measured positions; describing the
// scene or a relative jog does not. Erring towards "needs" only blocks more.
// "open" is not here: "is the gripper open?" and "open your jaws" are gripper
// moves, and "open the box" names its object. "hand" counts only when
// something is handed ("raise your hand" is no task).
const PositionVerbs = /\b(grab|grasp|pick|take|get|fetch|bring|hold|lift|carry|place|put|drop|stack|push|pull|slide|drag|touch|tap|poke|press|flip|reach|hover|approach|collect|sort|knock|give|throw|toss|remove|insert|clean|tidy|wipe|point (?:at|to))\b|\bhand(?=\s+(?:me|us|it|them|over|this|that|the|a|an)\b)|\b(?:move|go)\s+(?:(?:the|your)\s+(?:(?:gripper|arm|claw)(?:\s+(?:tip|jaws?))?|tip)\s+)?(?:to|toward|towards|over|above|near|onto|into|beside|behind|next to|in front of)\b/i;
/** Whether a task needs measured positions: an action verb, or an object it acts on (read through verb typos too). */
export function TaskNeedsPositions(Task: string): boolean {
  return PositionVerbs.test(Task) || ParseTaskObject(Task) !== null;
}

/**
 * Why this capture must stop the AI request before any model call, or null.
 * A camera that moved since calibration always stops it. An unverified or
 * unchecked camera, or a missing or failed measurement, stops tasks that act
 * on objects; a description or a relative jog goes on without positions.
 */
export function CameraGateFailure(Located: LocateResult | null, NeedsPositions: boolean,
  Source: "depth" | "webcam" = "depth"): CameraGate | null {
  const Check = Located?.cameraCheck ?? null;
  const FittedAt = FormatFittedAt(Check?.ReferenceFittedAt ?? null);
  const Detail = [Located?.error, Check?.Reason].filter((Text): Text is string => Boolean(Text)).join(" ");
  const Code = Located?.errorCode ?? null;
  if (Check?.AlignmentSource === "robot" && (Code === "camera_moved" || (NeedsPositions && (!Check.Ok || Located?.error)))) {
    return { Code: Code ?? "camera_not_verified", Detail: Check.Reason,
      Title: Located?.error || Check.Reason || "Camera alignment could not be verified. Keep the robot reference clearly in view, then take a fresh look." };
  }
  const Since = `since calibration${FittedAt ? ` at ${FittedAt}` : ""}`;
  if (Code === "camera_moved") {
    const DeskMoved = (Check?.AngleDeg ?? 0) > DeskMoveLimitDeg || Math.abs(Check?.OffsetCm ?? 0) > DeskMoveLimitCm;
    const Movement = DeskMoved ? MovementText(Check) : "";
    if (!Movement && Located?.error) return { Code, Detail: Check?.Reason ?? "", Title: Located.error };
    return { Code, Detail, Title: `Camera moved ${Movement ? `${Movement} ` : ""}${Since}: lock the head and run Automatic calibration.` };
  }
  if (!NeedsPositions) return null;
  if (Code === "camera_not_verified") {
    return { Code, Detail, Title: Source === "webcam"
      ? "Tasks that act on objects need the depth camera: a webcam capture cannot be checked against its calibration. Switch to the depth camera and run Automatic calibration."
      : "The saved calibration has no camera check reference, so object positions cannot be trusted: lock the head and run Automatic calibration." };
  }
  if (Code === "camera_check_failed") {
    return { Code, Detail, Title: "The camera could not be checked against its calibration (the desk was not measured): clear the desk in view, or lock the head and run Automatic calibration." };
  }
  // The desk moved against the arm (the cart was rolled) and the gripper tape, which would tell a still
  // camera from a moved one, could not be judged: not a moved camera, so no recalibration. The bridge's
  // sentence names what lets the tape judge (torque on, arm parked and still, tape in view).
  if (Code === "desk_changed_unverified") {
    return { Code, Detail: Check?.Reason ?? "", Title: Located?.error
      || "The desk moved against the arm since calibration (was the cart moved?): park the arm with torque on and the red tape on the gripper in view, hold it still, then send the task again." };
  }
  if (!Located) {
    return { Code: "locate_unavailable", Detail, Title: "Object positions could not be measured: the bridge did not answer. Check the bridge, then send the task again." };
  }
  if (Located.error) {
    return { Code: "locate_failed", Detail: "", Title: `Object positions could not be measured: ${Located.error}` };
  }
  if (!Check) {
    return { Code: "camera_not_verified", Detail, Title: "The bridge did not check the camera against its calibration: load the bridge update, then run Automatic calibration." };
  }
  if (!Check.Ok) {
    return { Code: "camera_moved", Detail, Title: `Camera moved ${MovementText(Check) ? `${MovementText(Check)} ` : ""}${Since}: lock the head and run Automatic calibration.` };
  }
  return null;
}

/** The log line for a capture the bridge accepted on a re-measured desk (the cart was rolled), or null: never a block. */
export function DeskRemeasuredNote(Check: CameraCheck | null): string | null {
  if (!Check?.Ok || !Check.DeskRemeasured) return null;
  return "The desk moved against the arm since calibration (cart moved): the gripper tape shows the camera has not, so the desk was measured again on this frame"
    + (Check.DeskSaved === false ? " (not saved: the next capture measures it again)." : " and saved as the new baseline.");
}

/**
 * The log line for a capture the bridge accepted with no calibrated desk in view, or null: never a block.
 * Verified (the gripper tape where the calibration puts it) is a note; assumed (the tape could not be
 * judged, so the camera is taken to be where it was calibrated) is a one-line warning saying why.
 */
export function RobotCheckNote(Check: CameraCheck | null): { Level: "ok" | "warn"; Text: string } | null {
  if (!Check?.Ok || Check.AlignmentSource !== "robot") return null;
  const FittedAt = FormatFittedAt(Check.ReferenceFittedAt);
  const Since = FittedAt ? ` from ${FittedAt}` : "";
  if (Check.RobotCheck === "verified") {
    return { Level: "ok", Text: `Camera checked against the gripper tip (no desk in view): the red tape is ${Check.TapeErrorCm != null
      ? `${Check.TapeErrorCm.toFixed(1)} cm` : "within 0.75 cm"} from where the calibration${Since} puts it.` };
  }
  if (Check.RobotCheck === "assumed") {
    return { Level: "warn", Text: Check.Warning
      || `Camera assumed where it was calibrated${FittedAt ? ` at ${FittedAt}` : ""}: the gripper tip could not be checked.` };
  }
  return null;
}

/**
 * The task's object as the bridge saw it but withheld (not measured: no surface under it, outside the
 * workspace, the arm in front of it...), or null: an entry the task query found (TaskQueryOnly) or whose
 * label names the task's object. Only for an object nothing else places: the caller asks when it is not
 * bound this frame and was never measured in this task.
 */
export function WithheldTaskObject(Located: LocateResult | null, Query: TaskObjectQuery | null | undefined): WithheldObject | null {
  if (!Located || !Query) return null;
  return Located.withheld.find(Item => Item.TaskQueryOnly || NamesQuery(Query, Item.Label)) ?? null;
}

/**
 * Why a task stops on its withheld object, and what the user can do (StopTaskWithReason). Told only that
 * it is not measured, the model went on with targets read off the image that no measurement, GRASP PLAN
 * or floor guards (2026-10-06, the pill bottle on the cabinet: before the camera check could pass there).
 */
export function WithheldTaskStop(Item: WithheldObject, Label: string): { Reason: string; Advice: string } {
  const Why = Item.Reason.replace(/^not measurable: /, "");
  const Reason = `the ${Label} is in view but the camera could not measure it (${Why}), so no move toward it can be checked.`;
  const Advice = /robot's own arm/.test(Why) ? `Move the arm to Home so the camera sees the ${Label} clear of it, then send the task again.`
    : /no level surface|surface around the object|no (?:desk|bin) surface|no bin floor|stands above the support/.test(Why)
      ? `The camera needs to see the surface the ${Label} stands on around it: clear the things right beside it, then send the task again.`
      : /more than one measured surface/.test(Why) ? `Move the ${Label} away from the edge and the other things around it, so one surface clearly holds it, then send the task again.`
        : /cm from the camera|outside the workspace|cm across|cm tall/.test(Why) ? `Move the ${Label} closer to the right arm, within its workspace, then send the task again.`
          : `Make sure the ${Label} stands clear of other things and in view of the camera, then send the task again.`;
  // A level surface no DeclaredSurfaces.json entry declares seems to hold it (the bridge's hint, with the automatic supports off).
  return { Reason, Advice: Item.UndeclaredSupportZ != null
    ? `${Advice} If it stands on a platform at Z ${Item.UndeclaredSupportZ.toFixed(1)}, declare that platform in back end/DeclaredSurfaces.json and reload the page.` : Advice };
}

// TaskObject: the task's object in this frame, which the coarse-box rule
// spares: the bridge refines the boxes it takes for it (an unrefined one is a
// refine that failed, or an object the app bound by estimate, colour or position).
function Reasons(Object_: LocatedObject, TaskObject: LocatedObject | null = null) {
  const Outside = Object_.outsideCalibrationCm != null ? `${Object_.outsideCalibrationCm.toFixed(1)} cm ` : "";
  return [...(Object_.extrapolated && !HasVerifiedDepthFrame(Object_) ? [`${Outside}outside the camera-calibrated area`] : []),
    ...(Object_.unstable ? ["vision detections disagreed between samples"] : []),
    ...(Object_.taskQueryOnly ? ["found only by the task query, in one detection"] : []),
    ...(Object_.refined === false && Object_ !== TaskObject ? ["coarse box, not refined"] : [])];
}
// A task-query-only object was never checked against the other samples, so
// it is only a hover target, like a measurement the samples disagreed on or
// a box the bridge did not refine.
export function IsUnreliable(Object_: LocatedObject, TaskObject: LocatedObject | null = null) { return Reasons(Object_, TaskObject).length > 0; }

/**
 * The measurements an absolute target may only hover over: this frame's
 * [UNRELIABLE] ones and the task object's last one if it was. The task
 * object bound in this frame is judged without its refine (Reasons).
 */
export function UnreliableMeasurements(Measured: LocatedObject[] | null, TaskObject: TaskObjectState | null): LocatedObject[] {
  const Bound = TaskObject?.Binding?.Object ?? null, Last = TaskObject?.LastSeen?.Object ?? null;
  return [...new Set([...(Measured ?? []).filter(Object_ => IsUnreliable(Object_, Bound)),
    ...(Last && IsUnreliable(Last, Last) ? [Last] : [])])];
}

/** Grasp height (mid-height) and the hover waypoint above an object, in the AI's frame. */
export function GraspHeights(Object_: LocatedObject) {
  const Top = Object_.topZ ?? Object_.z + 2;
  const GraspZ = Object_.topZ != null ? (Object_.z + Object_.topZ) / 2 : Object_.z + 2;
  return { Top, GraspZ, WaypointZ: Math.max(GraspZ + 6, Top + HoverClearanceCm) };
}
// A hover this far above the object's top clears it even when an [UNRELIABLE]
// measurement is several cm off in X/Y.
const HoverClearanceCm = 3;
// Heights handed to the model are printed to 0.1 cm: a hover rounded down would
// fall under the clearance the refusal below demands.
const CeilTenth = (Value: number) => Math.ceil(Value * 10 - 1e-9) / 10;
const Tenth = (Value: number) => Number(Value.toFixed(1));
/** The heights the model is given for an object, as printed: the hover rounded up. */
function PrintedHeights(Object_: LocatedObject) {
  const { GraspZ, WaypointZ } = GraspHeights(Object_);
  return { GraspZ: Tenth(GraspZ), HoverZ: CeilTenth(WaypointZ) };
}

/**
 * Refuse an absolute target copied from an [UNRELIABLE] task-object
 * measurement when it would descend onto the object: the model used such
 * readings as exact grasp targets while they were several cm off. A hover
 * above it is allowed, since the next image checks the alignment before any
 * descent; relative moves hand-computed by the model often flipped a sign.
 * AbsoluteXY: the target's X and Y were both given as target_cm.
 * Target is model frame; the object is in the AI's frame.
 */
export function UnreliableTargetReason(Target: Point3, AbsoluteXY: boolean, Object_: LocatedObject | null): string | null {
  if (!AbsoluteXY || !Object_ || !IsUnreliable(Object_)) return null;
  const Display = mirrorDisplayPoint(Target);
  if (FromFootprint(Object_, Display) > 3) return null;
  const { Top } = GraspHeights(Object_);
  if (Display.z >= Top + HoverClearanceCm - 1e-6) return null;
  return `Target X ${Display.x.toFixed(1)}, Y ${Display.y.toFixed(1)}, Z ${Display.z.toFixed(1)} descends onto the [UNRELIABLE] ${Object_.label} measurement, which may be off by several cm: the app only accepts it as an absolute target_cm for a hover at Z ${CeilTenth(Top + HoverClearanceCm).toFixed(1)} or higher. Descend with a relative move_delta_cm step once the image shows the tip above it, or reply ask.`;
}

// How far a display point is from an object's measured footprint (its centre
// when the depth camera gave no size): "within 3 cm" counted from the centre
// let a target over the edge of a 5 cm box through.
function FromFootprint(Object_: LocatedObject, Point: XY, Plan: Prism | null = null) {
  return FootprintDistance(Plan?.Footprint ?? ObjectPrism(Object_)?.Footprint ?? [{ x: Object_.x, y: Object_.y }], Point);
}

function SupportText(Object_: LocatedObject) {
  if (Object_.supportGeometry) return `; resting on the ${Object_.supportGeometry.Kind} measured in this capture (top Z ${Object_.supportGeometry.TopZCm.toFixed(1)})`;
  if (Object_.supportBranch === "surface") {
    const Surface = DeclaredSurfaceOf(Object_);
    return `; resting on ${Surface ? SurfaceText(Surface) : `the surface "${Object_.supportSurface ?? "?"}", which this app does not declare`}`;
  }
  if (Object_.supportMeasured === false) return "; base not measured, desk height assumed";
  if (Object_.supportBranch === "bin") return "; resting in the bin";
  if (Object_.supportBranch === "object") return "; resting on another object";
  return "";
}

/** The declared surface the bridge measured an object on (by its name, back end/DeclaredSurfaces.json), or null. */
export function DeclaredSurfaceOf(Object_: LocatedObject, Surfaces: readonly DeclaredSurface[] = DeclaredSurfaceList): DeclaredSurface | null {
  if (Object_.supportBranch !== "surface" || !Object_.supportSurface) return null;
  return Surfaces.find(Surface => Surface.Name.toLowerCase() === Object_.supportSurface!.toLowerCase()) ?? null;
}

// The bridge measures an object on a declared surface within 2 cm of its declared height
// (DepthGeometry DeclaredToleranceCm), touching it within 2 cm of its extent: the same here.
const SurfaceToleranceCm = 2.0;
const SurfaceExtentMarginCm = 2.0;
/**
 * The surface a task object stands on as the task holds the arm to it: its floor
 * (AiTableFloor.ts AiFloor: the desk's for an object on the desk, in the bin or
 * with no support named, as always), or why the app plans no motion to it. On a
 * declared surface the bridge measured it on, the floor is that surface's, top-down
 * as on the desk; refused when the page does not declare it (the file changed and
 * the page was not reloaded), when the reading disagrees with the declaration, or
 * on a shelf with a board above, which no check models yet. A refused raised
 * surface still gives its floor, so nothing near it is held to the desk's.
 */
export function TaskSupport(Object_: LocatedObject | null, Label = Object_?.label ?? "object",
  Surfaces: readonly DeclaredSurface[] = DeclaredSurfaceList): { Floor: AiFloor; Refusal: GraspRefusal | null } {
  // With its measured scene: that scene's support. Without a whole one (no scene, no measured support) the object
  // goes by the rules below, as before measured scenes (ReadLocateResponse reads such an object as a desk object).
  const Measured = Object_?.supportGeometry, Scene = Object_?.scene;
  if (Object_ && Measured && Scene && Object_.depthMeasured && Object_.supportMeasured === true) {
    const Refusal = (Text: string, Floor = DeskFloor) => ({ Floor, Refusal: { Code: "surface" as const, Text,
      Advice: "Keep the object and the space around it in the depth camera's view, then take a fresh look." } });
    const Floor = { ...SurfaceFloor(Measured, Object_.z), Scene };
    if (Math.abs(Object_.z - Measured.TopZCm) > SurfaceToleranceCm || !SurfaceContains(Measured, Object_, SurfaceExtentMarginCm))
      return Refusal("the object no longer matches the surface measured under it", Floor);
    if (!Scene.free_space?.length) return Refusal("the camera cannot yet see a clear approach around the object", Floor);
    return { Floor, Refusal: null };
  }
  if (!Object_ || Object_.supportBranch !== "surface") return { Floor: DeskFloor, Refusal: null };
  const Surface = DeclaredSurfaceOf(Object_, Surfaces);
  const Refuse = (Text: string, Advice: string, Floor = DeskFloor) => ({ Floor, Refusal: { Code: "surface" as const, Text, Advice } });
  if (!Surface) {
    return Refuse(`the bridge measured it on a surface called "${Object_.supportSurface ?? "?"}" that this page does not declare`,
      "Reload the web app so it reads back end/DeclaredSurfaces.json as the bridge does, then send the task again.");
  }
  const Floor = SurfaceFloor(Surface, Object_.z);
  const Name = SurfaceText(Surface);
  if (Math.abs(Object_.z - Surface.TopZCm) > SurfaceToleranceCm) {
    return Refuse(`it was measured at base Z ${Object_.z.toFixed(1)}, more than ${SurfaceToleranceCm} cm from ${Name}`,
      `Measure ${Surface.Name}'s top again and correct TopZCm in back end/DeclaredSurfaces.json.`, Floor);
  }
  if (!SurfaceContains(Surface, Object_, SurfaceExtentMarginCm)) {
    return Refuse(`it was measured at X ${Object_.x.toFixed(1)}, Y ${Object_.y.toFixed(1)}, outside ${Name}`,
      `Check where ${Surface.Name} is (XRangeCm, YRangeCm in back end/DeclaredSurfaces.json), or put the ${Label} on it.`, Floor);
  }
  if (Surface.Kind === "shelf") {
    return Refuse(`it stands on ${Name} under a board at Z ${Surface.BoardAboveZCm!.toFixed(1)}, which the app's checks do not model yet, so it plans no grasp under it`,
      `For now put the ${Label} on an open platform (Kind "platform", nothing above it).`, Floor);
  }
  return { Floor, Refusal: null };
}
/** The floor of a task (TaskSupport): from its object as measured now, else as last measured; the desk's without one. */
export function TaskFloor(State: TaskObjectState | null): AiFloor {
  const Object_ = (State?.DetectedNow ? State.Binding?.Object : null) ?? State?.LastSeen?.Object ?? null;
  return TaskSupport(Object_, State?.Label).Floor;
}

/**
 * The measured-objects block of the prompt; the camera check numbers are this frame's. TaskObject is tagged as the task's;
 * Partly: it was measured only partly with the arm over it, and its TASK OBJECT line keeps its last full measurement (ArmOcclusion.ts).
 */
export function MeasuredObjectsSummary(Located: LocateResult | null, TaskObject: LocatedObject | null = null, Partly = false): string {
  if (!Located || Located.error) {
    return [
      "",
      `NO MEASURED OBJECT POSITIONS: ${Located?.error ?? "the bridge did not answer the measurement request"}`,
      "Positions you read off the image alone are rough guesses, often 10 cm or more off. Do not state precise coordinates you were not given, and never store object positions in the notepad.",
      "Move in modest relative steps you can check at the next look, and tell the user that running the camera calibration would give you measured positions.",
    ].join("\n");
  }
  const Check = Located.cameraCheck;
  const FittedAt = FormatFittedAt(Check?.ReferenceFittedAt ?? null);
  const CheckText = Check?.Ok
    ? `camera check passed: ${MovementText(Check) || "no measurable change"} from the calibration${FittedAt ? ` fitted at ${FittedAt}` : ""}`
    : "camera pose not checked";
  return [
    "",
    `MEASURED OBJECT POSITIONS (measured in this frame, ${CheckText}; SAME frame as your coordinates, X positive = left):`,
    ...(Located.objects.length ? Located.objects.map(Object_ => {
      const Why = Reasons(Object_, TaskObject);
      // This check-in measured the gripper in its place (lib/ArmMadeObjects.ts): its last reading clear of the gripper.
      const Kept = Object_.armClearAt != null ? ` (as measured at ${Clock(Object_.armClearAt)} with the gripper clear of it: this check-in measured the gripper in its place)` : "";
      const Tag = Kept + (Why.length ? ` [UNRELIABLE: ${Why.join("; ")}: may be off by several cm]` : "") + (Object_ !== TaskObject ? ""
        : Partly ? " <- the task object, measured only partly (the gripper hides part of it): use its TASK OBJECT line's last full measurement" : " <- the task object");
      if (Object_.depthMeasured && Object_.topZ != null) {
        const Size = Object_.sizeCm ? `, footprint about ${Object_.sizeCm[0].toFixed(1)} x ${Object_.sizeCm[1].toFixed(1)} cm` : "";
        return `- ${Object_.label}: centre X=${Object_.x.toFixed(1)}, Y=${Object_.y.toFixed(1)}, base Z=${Object_.z.toFixed(1)}, top Z=${Object_.topZ.toFixed(1)} (${(Object_.heightCm ?? Object_.topZ - Object_.z).toFixed(1)} cm tall${Size}; measured by the depth camera${SupportText(Object_)})${Tag}`;
      }
      return `- ${Object_.label}: X=${Object_.x.toFixed(1)}, Y=${Object_.y.toFixed(1)}, base Z=${Object_.z.toFixed(1)} (where it MEETS the table)${Tag}`;
    }) : ["(no objects were measured in this frame)"]),
    ...(Located.withheld.length ? ["Not measurable in this frame (do not target these):",
      ...Located.withheld.map(Item => `- ${Item.Label}: not measurable: ${Item.Reason}`)] : []),
    "These positions were measured in this frame. If the image disagrees with a position by more than 3 cm, do not move toward it: reply ask and say what looks wrong.",
    "Measurements tagged [UNRELIABLE] may be off by several cm: near them target_cm is accepted only for a hover at least 3 cm above the object's top. Descend with relative move_delta_cm steps you can check at the next look, or reply ask.",
    "IMPORTANT: base Z is the SURFACE under the object, NOT a grasp height, and an object's centre is NOT a grasp point: the tip you command is the inner face of the gripper's FIXED jaw, so for a grasp it goes BESIDE the object. Grasp the task object only at the GRASP POINT its TASK OBJECT line gives. Never command the tip down to base Z or below it: that presses the gripper into the table.",
  ].join("\n");
}

// --- The task's target object ---------------------------------------------
// Parsing the task and finding its object among the measurements: TaskObject.ts.

/** The object a task acts on ("Grab the eraser" -> "eraser"; "grab the black one" -> "black thing"), or null. */
export function TaskObjectNoun(Task: string): string | null {
  const Query = ParseTaskObject(Task);
  return Query ? Query.Head ?? Query.Phrase : null;
}

export type TaskReachability = { Kind: "graspable" | "blocked" | "out-of-reach" | "ungraspable"; Text: string; BaseDistanceCm: number;
  // Wrist pitches the app verified at the grasp height and at the hover waypoint (null: none, or not judged).
  GraspPitch?: number | null; HoverPitch?: number | null;
  // For a grasp (GraspPlan.ts): the fixed-jaw GRASP PLAN, or why there is none; the reach scan's pitch at the centre.
  Plan?: GraspPlan | null; Refusal?: GraspRefusal | null; CentrePitch?: number | null;
  // Out of reach for a grasp: how many cm closer to the arm the app planned it, and what the user is told.
  CloserCm?: number | null; Advice?: string | null };
export type TaskObjectState = {
  // The user's words for it ("black thing"); a measured label never replaces it.
  Label: string;
  Query?: TaskObjectQuery | null;
  DetectedNow: boolean;
  // How this check-in's measurement was picked, and while nothing is picked
  // the measured objects it could be (a generic phrase, or two named spots).
  Binding?: TaskObjectBinding | null;
  Candidates?: LocatedObject[];
  // Reachability is the verdict on that measurement, kept for check-ins
  // where the object is hidden (under the gripper while grasping). Motions: the motions approved in the task when it was measured.
  LastSeen: { At: number; Object: LocatedObject; Reachability: TaskReachability | null; Motions?: number } | null;
  // The arm over or beside it hides it this check-in (ArmOcclusion.ts): LastSeen and the plan stay the working ones.
  Occluded?: ArmOcclusion | null;
  Reachability: TaskReachability | null;
  // A grasp's plan, locked to the measurement it came from (GraspPlan.ts KeepOrReplan), and what the task does to the object.
  Plan?: GraspPlanState | null;
  Intent?: TaskIntentKind;
  // A go-ahead after the app stopped the task: the stopped task's last measurement and plan,
  // taken up at the first check-in only if the object is not measured then (under the gripper).
  // Asked: carried by an answer to the model's question, taken up only with the arm over it (ArmOcclusion.ts).
  Carried?: { LastSeen: NonNullable<TaskObjectState["LastSeen"]>; Plan: GraspPlanState | null; Asked?: boolean } | null;
};

const Clock = (At: number) => new Date(At).toTimeString().slice(0, 8);
const UnreliableAdvice = " Because it is tagged [UNRELIABLE], target_cm is accepted only for the hover waypoint above it: descend with relative move_delta_cm steps you can check in the next image.";
// Plain JSON numbers: a model copying "+28.4" would send invalid JSON.
const Cm = (Value: number) => (Math.abs(Value) < 0.05 ? 0 : Value).toFixed(1);

const Deg = (Pitch: number) => Number(Pitch.toFixed(1)).toString();

/**
 * The exact moves from the tip to the hover waypoint and the grasp height, so
 * the model copies numbers instead of subtracting: it flipped the sign of Y
 * while the measured box sat right on the object. Tip is in the AI's frame.
 * With a reach verdict each move carries the wrist pitch the app verified:
 * without one the model used the -45 of the prompt's example, which the arm
 * could not reach there.
 */
function MovesFromTip(Object_: LocatedObject, Tip: Point3, Reach: TaskReachability | null) {
  const { GraspZ, HoverZ } = PrintedHeights(Object_);
  const X = Tenth(Object_.x), Y = Tenth(Object_.y);
  const Wrist = (Pitch: number | null | undefined) => Pitch != null ? `,"wrist_pitch_degrees":${Deg(Pitch)}` : "";
  const Hover = `{"x":${X.toFixed(1)},"y":${Y.toFixed(1)},"z":${HoverZ.toFixed(1)}}`;
  const Delta = `{"left":${Cm(X - Tip.x)},"farther":${Cm(Y - Tip.y)},"up":${Cm(HoverZ - Tip.z)}}`;
  return ` READY-MADE MOVES from the tip now (X ${Tip.x.toFixed(1)}, Y ${Tip.y.toFixed(1)}, Z ${Tip.z.toFixed(1)}):`
    + ` 1) hover above it: "motion_proposal":{"target_cm":${Hover}${Wrist(Reach?.HoverPitch)}} (the same as move_delta_cm ${Delta}${Reach?.HoverPitch != null ? " with that wrist" : ""});`
    + ` 2) only once the image shows the tip over it, descend to the grasp height Z ${GraspZ.toFixed(1)}: "motion_proposal":{"move_delta_cm":{"left":0,"farther":0,"up":${Cm(GraspZ - HoverZ)}}${Wrist(Reach?.GraspPitch)}}.`
    + ` Copy these numbers exactly, Z${Reach?.GraspPitch != null ? " and wrist" : ""} included; do not recompute them.`;
}

/**
 * The gripper as the grasp steps see it: how far open it is now, and whether a
 * close was approved in this task. Held: the target the arm is holding (the
 * bridge's, display frame), so a step never repeats a move that would not move it.
 */
export type GraspState = { GripperFraction: number; Closed: boolean; Held?: { Point: Point3; Pitch: number } | null };
// Opened to this, not 1.0: a full open stopped at raw 1971 (0.83) short of its
// 2242 target and the arm read as moving until the next task timed out (2026-09-29 22:55).
export const GripperOpenFraction = 0.8;
// At least this open counts as open for descending around an object.
const OpenEnoughFraction = 0.6;
// Closed further than this, the jaws met nothing (a full close reads 0.034).
// TODO(grasp M3): calibrate this per object width from a physical close on 1 cm
// and 3 cm blocks (and a 6.6 cm can). The CAD jaw sits about 11 deg off the real
// one (report C 2f), so the reading a held object leaves cannot be computed from
// the plan (GraspPlan Closing.ContactDeg is the CAD angle the jaw meets it at):
// it is NOT derived here until it has been measured on the arm.
export const HeldFraction = 0.08;
// An approved gripper move to this or less in an object task is a close (the object may be
// in the jaws from then on); a move that narrows less (to a GRASP PLAN's 50-60%) is not.
export const CloseTargetFraction = 0.3;
/**
 * Whether the gripper counts as closed in an object task after an approved
 * gripper move from the fraction it was set to (Before) to Target: an opening
 * ends a close, a move to CloseTargetFraction or less is one (the NEXT STEP's
 * close is 0), and a move that narrows less (to a GRASP PLAN's 50-60% above
 * the object) leaves it as it was. Counted as a close, that narrowing
 * exempted the object from the guard and the next step said "it is lifted".
 */
export function GripperClosedAfter(Before: number, Target: number, Closed: boolean): boolean {
  if (Target > Before) return false;
  return Target <= CloseTargetFraction ? true : Closed;
}
/** Where a task's last close was sent from (the tip, display frame) and the GRASP PLAN it closed with (App.tsx TaskCloseRef). */
export type TaskClose = { Tip: Point3; Plan: GraspPlan | null };
/**
 * An approved gripper move of an object task from fraction Before to Target,
 * sent with the tip at Tip and the task's GRASP PLAN Plan: whether the jaws
 * count as closed after it (GripperClosedAfter), and the close on record: a
 * close (to CloseTargetFraction or less) records where it was sent from, unless
 * the jaws were closed already (where they first closed stands); an opening
 * clears it; a narrowing that is no close keeps it. ClosedOnNothing
 * judges a miss where the jaws closed: the close the steps offer is chained
 * with a lift, which has moved the tip by the next check-in.
 */
export function NoteGripperMove(Before: number, Target: number, Closed: boolean, Close: TaskClose | null, Tip: Point3,
  Plan: GraspPlan | null): { Closed: boolean; Close: TaskClose | null } {
  const After = GripperClosedAfter(Before, Target, Closed);
  if (!After) return { Closed: false, Close: null };
  return { Closed: true, Close: Target <= CloseTargetFraction && (!Closed || !Close) ? { Tip: { x: Tip.x, y: Tip.y, z: Tip.z }, Plan } : Close };
}
// The tip counts as over the object within this, and at the grasp height within GraspReachedCm above it.
const OverObjectCm = 1.5;
const GraspReachedCm = 1.0;
const JawsAroundCm = 0.3;
const LiftCm = GraspRules.LiftCm;
const GripperJson = (Fraction: number) => `{"direct_servo_targets":[{"name":"gripper","target_fraction":${Fraction.toFixed(1)}}]}`;
// CHAINED MOTIONS (lib/AiChain.ts): a tip move's JSON with the gripper action that runs before it,
// and the one chain offered after a close: close, then lift straight up (the gripper first).
const ChainedJson = (Move: string, Fraction: number) => `${Move.slice(0, -1)},"gripper_before":${Fraction.toFixed(1)}}`;
export const CloseAndLiftJson = (LiftCm: number) => ChainedJson(`{"move_delta_cm":{"left":0,"farther":0,"up":${LiftCm}}}`, 0);
const ChainCopy = " Copy it exactly: it is ONE motion_proposal, and the app runs the gripper first, then the tip; the next check-in gives the step after it.";
// An opening chained before a move runs as one motion: the gripper opens while the tip moves (AiChain.ts TOGETHER).
const OpenChainCopy = " Copy it exactly: it is ONE motion_proposal, and the gripper opens while the tip moves; the next check-in gives the step after it.";
// A step's copy instruction: the 18:54 model skipped a corrected hover and descended (2026-10-01).
const StepCopy = " Copy it exactly (one step per motion, never skipping ahead); the next check-in gives the step after it.";

/**
 * The one next step of the grasp, from where the tip and the gripper are now:
 * hover, open, descend, close, lift. The model was only ever given hover and
 * descend, so it came down with the jaws closed onto the object and never
 * opened or closed them (2026-09-29 22:47). The descent is measured from the
 * tip, not from the printed hover height the tip may have missed.
 */
function NextGraspStep(Object_: LocatedObject, Tip: Point3, Reach: TaskReachability | null, Grasp: GraspState, Contact = false) {
  const { GraspZ, HoverZ } = PrintedHeights(Object_);
  const X = Tenth(Object_.x), Y = Tenth(Object_.y);
  const Wrist = (Pitch: number | null | undefined) => Pitch != null ? `,"wrist_pitch_degrees":${Deg(Pitch)}` : "";
  const Head = ` NEXT STEP of the grasp (tip now X ${Tip.x.toFixed(1)}, Y ${Tip.y.toFixed(1)}, Z ${Tip.z.toFixed(1)}; gripper ${Math.round(Grasp.GripperFraction * 100)}% open): `;
  const Copy = StepCopy;
  if (Grasp.Closed) {
    if (Grasp.GripperFraction < HeldFraction) {
      return `${Head}the jaws closed all the way, so they missed it. Open the gripper, "motion_proposal":${GripperJson(GripperOpenFraction)}, then hover over it again.`;
    }
    if (Tip.z < GraspZ + LiftCm - 1) {
      return `${Head}the jaws stopped partway, so they hold something. Lift it straight up: "motion_proposal":{"move_delta_cm":{"left":0,"farther":0,"up":${LiftCm}}}.${Copy}`;
    }
    return `${Head}it is lifted. If the image shows it in the jaws, reply task_status "done"; if not, reply ask.`;
  }
  const Top = Object_.topZ ?? Object_.z + 2;
  const HoverMove = `{"target_cm":{"x":${X.toFixed(1)},"y":${Y.toFixed(1)},"z":${HoverZ.toFixed(1)}}${Wrist(Reach?.HoverPitch)}}`;
  const Away = Math.hypot(Tip.x - X, Tip.y - Y) > OverObjectCm;
  // Open first, so the jaws are already open on the way over (the owner's ask, 2026-09-30): with the
  // hover as ONE chained motion (lib/AiChain.ts), unless the tip is low beside it, where opening swings
  // the jaw into it (AiChain ChainRules: 3 cm from the guard's prism, which a round object widens by up to 1 cm).
  if (Grasp.GripperFraction < OpenEnoughFraction) {
    if (Away && (FromFootprint(Object_, Tip) > 4 || Tip.z >= Top + 1)) {
      return `${Head}open the gripper to ${Math.round(GripperOpenFraction * 100)}% and hover above it as ONE chained motion (the gripper opens while the tip moves): "motion_proposal":${ChainedJson(HoverMove, GripperOpenFraction)}.${OpenChainCopy}`;
    }
    return `${Head}open the gripper before approaching it: "motion_proposal":${GripperJson(GripperOpenFraction)}.${Copy}`;
  }
  if (Away) {
    return `${Head}hover above it: "motion_proposal":${HoverMove}.${Copy}`;
  }
  // Below the object's top the open jaws are already around it: close there.
  // A descent that stopped 1-2 cm above the grasp height was proposed again
  // and ended the task as a repeat (15:59, the pill container).
  if (Tip.z > GraspZ + GraspReachedCm && Tip.z > Top - JawsAroundCm) {
    // A grasp without a GRASP PLAN (this step is only ever given without one) does not descend beside or onto it:
    // the system prompt's GRASP RULE says reply ask, and the guard refuses that descent (GraspGuardReason).
    if (!Contact) {
      return `${Head}reply ask: the app has no GRASP PLAN for it, checked against the gripper's shape, so the jaws do not descend beside or onto it. Tell the user the arm stopped above it for that reason.`;
    }
    return `${Head}descend to the grasp height Z ${GraspZ.toFixed(1)}: "motion_proposal":{"move_delta_cm":{"left":${Cm(X - Tip.x)},"farther":${Cm(Y - Tip.y)},"up":${Cm(GraspZ - Tip.z)}}${Wrist(Reach?.GraspPitch)}}.${Copy}`;
  }
  // A contact task (push, slide, touch) only closes: the close and lift chain would pick it up.
  if (Contact) return `${Head}the jaws are around it at the grasp height; close the gripper: "motion_proposal":${GripperJson(0)}.${Copy}`;
  return `${Head}the jaws are around it at the grasp height; ${CloseAndLiftStep}${ChainCopy}`;
}

/**
 * Why a tip motion (model frame) must not run now: it descends onto the task
 * object (Object_, the AI's frame) with the jaws still closed, which pressed
 * them onto the object instead of around it. RiseCm is the motion's up change.
 * With a GRASP PLAN, closed means less open than the plan's fraction (within
 * the steps' tolerance), and the gripper is told to open to that fraction:
 * the NEXT STEP, this rule and the guard all hold the plan's opening.
 */
export function ClosedJawDescentReason(Target: Point3, RiseCm: number, Object_: LocatedObject | null, Grasp: GraspState | null,
  Plan: GraspPlan | Prism | null = null): string | null {
  const Full = Plan && "Grasp" in Plan ? Plan : null, Prism_ = Full ? Full.Prism : Plan as Prism | null;
  const Least = Full ? Full.OpenFraction - GraspRules.OpenToleranceFraction : OpenEnoughFraction;
  if (!Object_ || !Grasp || Grasp.Closed || Grasp.GripperFraction >= Least || RiseCm >= -0.5) return null;
  const Display = mirrorDisplayPoint(Target);
  const Top = Prism_?.TopZ ?? Object_.topZ ?? Object_.z + 2;
  if (FromFootprint(Object_, Display, Prism_) > 3 || Display.z >= Top + 1) return null;
  if (!Full) return `The gripper is closed (${Math.round(Grasp.GripperFraction * 100)}% open): open it before descending onto the ${Object_.label}: "motion_proposal":${GripperJson(GripperOpenFraction)}.`;
  return `The gripper is ${Math.round(Grasp.GripperFraction * 100)}% open, less than the ${Math.round(Full.OpenFraction * 100)}% the GRASP PLAN was checked at: open it before descending onto the ${Object_.label}: "motion_proposal":${GripperJson(Full.OpenFraction)}.`;
}

// --- The fixed-jaw GRASP PLAN, as the model is given it -------------------------

// Plain JSON numbers like Cm, with a bare 0 for no move along an axis.
const PlanCm = (Value: number) => (Math.abs(Value) < 0.05 ? "0" : Value.toFixed(1));
const PlanPoint = (Point: Point3) => `X ${Point.x.toFixed(1)}, Y ${Point.y.toFixed(1)}, Z ${Point.z.toFixed(1)}`;
const HeightWords: Record<HeightBy, (Plan: GraspPlan) => string> = {
  mid: () => "its mid-height",
  floor: () => "raised so the fingertips, closing, stay above the desk",
  palm: () => "raised so the gripper's palm stays above its top",
  jaw: Plan => `${(Plan.Prism.TopZ - Plan.Grasp.z).toFixed(1)} cm under its top, where the moving jaw meets its side, not its top edge`,
  clear: Plan => `${Math.abs(Plan.RaiseCm ?? 0).toFixed(1)} cm ${(Plan.RaiseCm ?? 0) < 0 ? "below" : "above"} its mid-height${Plan.RaisedFor ? ` (at mid-height ${Plan.RaisedFor})` : ""}`,
};
/**
 * What the GRASP PLAN is and why the tip goes beside the object, not over it.
 * A plan from the jaw-plane sweep (M3: a wide or round object) also says how
 * wide it was planned as, the wrist it needs all the way down, how far the
 * jaws open, and how the closing jaw meets it.
 */
function PlanText(Plan: GraspPlan, Label: string) {
  if (Plan.Approach) {
    const Side = Plan.Approach;
    return ` GRASP PLAN for the ${Label}: approach from the open side, keeping the wrist at ${Deg(Side.Pitch)} deg. PREGRASP ${PlanPoint(Side.Pregrasp)}, GRASP POINT ${PlanPoint(Plan.Grasp)}.`
      + ` Open the gripper to ${Math.round(Plan.OpenFraction * 100)}%, enter, then close and check the grasp before moving again.`
      + ` RELEASE CLEARANCE ${PlanPoint(Side.Release)}, RETREAT ${PlanPoint(Side.Retreat)}. The measured surrounding space constrains every step; never substitute an overhead hover or a straight-up lift.`;
  }
  const Round = Plan.Prism.Round, Closing = Plan.Closing, Size = Plan.Prism.SizeCm;
  const Beside = Round?.MergedCm && Size ? ` (it measured ${Size[0].toFixed(1)} x ${Size[1].toFixed(1)} cm, wider than a drink can: something beside it was measured with it,`
    + ` so it is planned as a ${Round.HiCm.toFixed(1)} cm can where its top was measured)`
    : Round ? ` (planned as if it were ${Round.HiCm.toFixed(1)} cm across: a round object measures narrower than it is)` : "";
  const Sizes = Closing.Mode === "sweep" && Closing.DiameterCm ? `${Closing.DiameterCm[0].toFixed(1)} to ${Closing.DiameterCm[1].toFixed(1)} cm across` : "as measured";
  // A round object's sweep plan's corrections for the real arm (GraspRules FixedFingerOffsetCm, GraspSagCm), said with their reasons.
  const Offset = Plan.FingerOffsetCm ?? 0, Sag = Plan.SagCm ?? 0, Gap = PlanClearanceCm(Plan);
  const Real = (Offset ? ` The GRASP POINT is ${(Gap + Offset).toFixed(1)} cm outside its ${Plan.Side} side as the camera measured it:`
      + ` the real fixed finger stands about ${Offset.toFixed(1)} cm nearer the ${Label} than the arm model puts it (it pushed a can it was planned 1.2 cm from), so the real finger comes down ${Gap.toFixed(1)} cm from it`
      + ` (the app checked the way down with it ${GraspRules.FixedFingerToleranceCm.toFixed(1)} cm nearer still); do not move the tip toward the ${Label} to close that gap.` : "")
    + (Sag ? ` The GRASP POINT's Z is ${Sag.toFixed(1)} cm above where the jaws close, because the arm stops about that much lower than a low target.` : "");
  // A sweep plan's straddle (GraspRules.StraddleCm): the jaws reach around it before they close.
  const Around = Plan.Straddle
    ? ` The jaws reach around it before they close: the GRASP POINT is ${Closing.Mode === "sweep" ? Closing.DepthOffsetCm.toFixed(1) : "0.0"} cm farther out along the fingers than its centre,`
      + ` so the fixed finger reaches ${Plan.Straddle.FixedCm.toFixed(1)} cm past its middle and the moving jaw, swinging closed, ${Plan.Straddle.MovingCm.toFixed(1)} cm past it`
      + ` (the app wants at least ${GraspRules.StraddleCm.toFixed(1)} for both: short of its middle, the close squeezes it out past the tips); do not stop the tip short of the GRASP POINT.`
      // A deep plan (GraspPlan.ts GraspRules.DeepGrasp): as deep in the jaws as the palm and the servo let it go.
      + (Plan.Deep && Plan.PalmGapCm != null ? ` It sits deep in the jaws, the gripper's palm and servo ${Plan.PalmGapCm.toFixed(1)} cm from it at the GRASP POINT`
        + ` (the app keeps them at least ${GraspRules.PalmGapCm.toFixed(2)} cm from it on the way down), so do not move the tip farther toward it along the fingers.` : "")
    : "";
  const Sweep = Closing.Mode !== "sweep" ? ""
    : ` It is too wide for the jaws to close on from straight above, so the wrist stays at ${Deg(Plan.Pitch)} deg all the way down`
      + ` (the app picked it itself among the pitches the jaws hold it with and the arm reaches beside it with${GraspRules.PreferredPitchDeg != null
        ? `, as near its preferred ${Deg(GraspRules.PreferredPitchDeg)} deg as that grip allows` : Plan.Straddle ? ", the one whose jaws reach farthest around it" : ""}; copy it: beside the ${Label} the app holds the wrist to it)`
      + ` and the gripper opens to ${Math.round(Plan.OpenFraction * 100)}% (exactly that: wider, the open jaw swings farther out).` + Around
      + (Closing.DriftCm < 0.05 ? "" : Plan.Straddle ? ` Closing, the moving jaw pushes it about ${Closing.DriftCm.toFixed(1)} cm along the fingers toward their tips before the pads grip it.`
        // No straddle (a box): the head start along the fingers is the push's drift, so it ends on the fixed pad's middle.
        : ` Closing, the moving jaw pushes it about ${Closing.DriftCm.toFixed(1)} cm along the fingers toward their tips before the pads grip it`
        + (Closing.DriftCm - Closing.DepthOffsetCm < 0.1 ? `, so the grasp point is ${Closing.DepthOffsetCm.toFixed(1)} cm farther out along the fingers than its middle and it ends on the middle of the fixed pad.`
          : Closing.DepthOffsetCm >= 0.05 ? `; the grasp point is ${Closing.DepthOffsetCm.toFixed(1)} cm farther out along the fingers than its middle for that (farther would bring the palm too near it), so it ends about ${(Closing.DriftCm - Closing.DepthOffsetCm).toFixed(1)} cm toward the tips from the middle of the fixed pad.`
            : `, so it ends about ${Closing.DriftCm.toFixed(1)} cm toward the tips from the middle of the fixed pad.`))
      + Tip(Closing) + ` The app checked that grip for it ${Sizes} and up to ${GraspRules.DepthToleranceCm.toFixed(1)} cm off along the fingers.` + Real;
  // The gap is the user's setting (Settings > Grasp gap at the fixed finger, GraspGap.ts): the model is told which it is,
  // and why a setting below the default was not used (GraspPlan.ts ClearanceFor: only a round object's plan corrects for the real finger).
  const GapWords = Plan.GapSetCm != null && Plan.GapSetCm < Gap
    ? ` The user set the grasp gap to ${Plan.GapSetCm.toFixed(1)} cm, but the fixed jaw keeps the default ${Gap.toFixed(1)} cm beside the ${Label}: only a round object's GRASP PLAN corrects for where the real fixed finger stands.`
    : ` That ${Gap.toFixed(1)} cm is the grasp gap the user set for the fixed jaw.`;
  return BestEffortText(Plan, Label) + ` GRASP PLAN (checked by the app against the gripper's shape): the tip is the inner face of the FIXED jaw (the one with the red tape) and only the other jaw moves, so the tip goes BESIDE the ${Label}, ${Gap.toFixed(1)} cm outside its ${Plan.Side} side${Beside}, NOT over its centre; the open moving jaw then closes across it and pushes it against the fixed jaw.`
    + `${GapWords} The grasp height is ${HeightWords[Plan.HeightBy](Plan)}.${Sweep}`;
}
/**
 * A best-effort plan (GraspRules.BestEffort): the user asked the app to try unless it is sure the grasp cannot
 * work, so the model is told what the app is not sure of, and to follow the steps anyway.
 */
export function BestEffortText(Plan: GraspPlan, Label: string) {
  if (!Plan.Warnings?.length) return "";
  return ` BEST-EFFORT GRASP: the app is not sure this grasp holds the ${Label} (${Plan.Warnings.join("; ")}), but nothing it checks rules it out`
    + ` (the arm reaches it clear of the basket, the desk and the other objects), so it is planned anyway: attempt it, following its steps as for any GRASP PLAN;`
    + ` do not give up or reply ask over these warnings, the user approves each move. After the close, if the image shows the ${Label} slipped out or was pushed away, reply ask.`;
}
/** The approval card's note for a move of a best-effort grasp (BestEffortText), or null. */
export function BestEffortNote(Plan: GraspPlan | null | undefined) {
  return Plan?.Warnings?.length ? `Best-effort grasp, may not work: ${Plan.Warnings.join("; ")}.` : null;
}
/**
 * Where a round object's sweep plan leaves it against the fixed finger
 * (GraspRules.FingertipMarginCm), and what a plan made without that margin
 * or the grip in 3D (GraspPlan.ts PlanGrasp's fallback) lacks.
 */
function Tip(Closing: GraspPlan["Closing"]) {
  if (Closing.Mode !== "sweep" || Closing.TipCm == null) return "";
  const Margin = GraspRules.FingertipMarginCm, Max = SweepRules.MaxLeanDeg, Grip3 = Closing.Grip3DDeg;
  const Lacks = [Closing.TipCm < Margin ? `its contact with the fixed finger ends only ${Math.max(Closing.TipCm, 0).toFixed(1)} cm inside the finger's tip (the app wants ${Margin.toFixed(1)})` : "",
    Grip3 != null && Grip3 > Max ? `the grip needs friction for ${Grip3.toFixed(0)} deg with the jaws tilted against its standing side (at most ${Max})` : ""].filter(Boolean);
  return !Lacks.length ? ` That leaves its contact with the fixed finger ${Closing.TipCm.toFixed(1)} cm inside the finger's tip (nearer the tip, the close can squeeze it out past the tips).`
    : ` No wrist pitch or height the app tried holds it better, but here ${Lacks.join(" and ")}: the close may squeeze it out past the finger tips; if the image after the close shows it slipping out, reply ask.`;
}
const StepHead = (Tip: Point3, Grasp: GraspState) =>
  ` NEXT STEP of the grasp (tip now X ${Tip.x.toFixed(1)}, Y ${Tip.y.toFixed(1)}, Z ${Tip.z.toFixed(1)}; gripper ${Math.round(Grasp.GripperFraction * 100)}% open): `;
const HoverJson = (Plan: GraspPlan) => `{"target_cm":{"x":${Plan.Hover.x.toFixed(1)},"y":${Plan.Hover.y.toFixed(1)},"z":${Plan.Hover.z.toFixed(1)}},"wrist_pitch_degrees":${Deg(Plan.HoverPitch)}}`;
const HoverStep = (Plan: GraspPlan, Label: string) => (Plan.Approach ? `go to the PREGRASP POINT on the open side of the ${Label}: ` : `go to the HOVER POINT, straight above the grasp point beside the ${Label} (not above its centre): `)
  + `"motion_proposal":${HoverJson(Plan)}.`;
// The gripper set (opened, or narrowed to an M3 plan's fraction) and the HOVER POINT as ONE chained motion:
// an opening runs while the tip moves, a narrowing first.
const SetAndHoverStep = (Plan: GraspPlan, Label: string, Set: string, Opens: boolean) => `${Set} and go to the HOVER POINT, straight above the grasp point beside the ${Label}`
  + ` (not above its centre), as ONE chained motion (${Opens ? "the gripper opens while the tip moves" : "the gripper first, then the tip"}): "motion_proposal":${ChainedJson(HoverJson(Plan), Plan.OpenFraction)}.`;
// Close, then lift straight up, as ONE chained motion: the bridge holds what the jaws stop on, then the tip rises.
const CloseAndLiftStep = `close the gripper and lift as ONE chained motion (the moving jaw pushes it against the fixed jaw and the gripper holds it, then the tip rises ${LiftCm} cm straight up): `
  + `"motion_proposal":${CloseAndLiftJson(LiftCm)}.`;
// Measured once so far (no second measurement agreed): the image decides the descent. Hidden by the arm over it
// (ArmOcclusion.ts) the image cannot show the jaws beside it: the step goes on from its last measurement
// (2026-10-03 16:42: the model asked twice, the pill bottle under the open jaws).
const OnceText = (Plan: GraspPlan, Hidden = false) => Hidden
  ? `measured once so far and the gripper now hides it (expected with the jaws over it: the image cannot show the open jaws beside it on its ${Plan.Side} side, and that is no reason to ask; reply ask only if the image shows it somewhere else, clear of the gripper), so go on from its last measurement.`
  : `measured once so far: descend only if the image shows the open jaws beside it on its ${Plan.Side} side, else reply ask.`;
// Narrower than this across the jaws, a close on it can read like a miss (report C 2f).
const NarrowCm = 1.2;
// The arm holds a point when the bridge's target is this near it (AiTaskLoop.ts NoOpCommandedCm and
// NoOpDeg): proposing it again would not move the arm. Its tip then stopped within ArrivalCm of it.
// NoOpReason counts the held target only with the tip within HeldTipCm of it, and a target within
// NoMoveCm of the tip (AiTaskLoop.ts NoOpLiveCm) as no move either. (AiTaskLoop.ts imports this file.)
const HeldCm = 0.5, HeldDeg = 2, HeldTipCm = 2, NoMoveCm = 1.0;
function Holds(Grasp: GraspState, Tip: Point3, Point: Point3, Pitch: number) {
  const Held = Grasp.Held;
  return !!Held && Math.hypot(Held.Point.x - Point.x, Held.Point.y - Point.y, Held.Point.z - Point.z) <= HeldCm
    && Math.abs(Held.Pitch - Pitch) <= HeldDeg && Math.hypot(Tip.x - Point.x, Tip.y - Point.y, Tip.z - Point.z) <= GraspRules.ArrivalCm;
}
/**
 * The up move (cm, in the steps' 0.1 cm) of a rise straight up from the tip
 * to Z that moves the arm (AiTaskLoop.ts NoOpReason). Where the arm holds a
 * target near the tip that the rise would end within HeldCm of, the rise goes
 * HeldCm + 0.1 above that target (AboveHeld); and it is never NoMoveCm or
 * less. 2026-10-01 18:56:43: the chained lift after a missed close held
 * Z 12.2 and its tip stopped at Z 11.4 under it; the rise to the hover height
 * Z 12.7 ended 0.4 cm from that target, so it would not move the arm, and the
 * model copying it twice ended the task.
 */
function RiseUp(Grasp: GraspState, Tip: Point3, Z: number): { Up: number; AboveHeld: boolean } {
  let Up = Number(PlanCm(Z - Tip.z)), AboveHeld = false;
  const Held = Grasp.Held?.Point;
  if (Held && Math.hypot(Tip.x - Held.x, Tip.y - Held.y, Tip.z - Held.z) <= HeldTipCm
    && Math.hypot(Tip.x - Held.x, Tip.y - Held.y, Tip.z + Up - Held.z) <= HeldCm + 0.05) {
    const Clear = CeilTenth(Held.z + HeldCm + 0.1 - Tip.z);
    if (Clear > Up) { Up = Clear; AboveHeld = true; }
  }
  return { Up: Up > 0 && Up <= NoMoveCm ? NoMoveCm + 0.1 : Up, AboveHeld };
}
/**
 * How far the pan's arrival tolerance (GraspRules.ArrivalToleranceRaw) moves the
 * tip at a point: the jaw axis is across the arm's reach, so all of it is along the jaws.
 */
export function PanArrivalCm(Point: XY) {
  const Pan = mirrorDisplayPoint({ x: 0, y: 0, z: 0 });
  return Math.hypot(Point.x - Pan.x, Point.y - Pan.y) * GraspRules.ArrivalToleranceRaw * 2 * Math.PI / 4096;
}
/**
 * The hover point aimed past by the pan's arrival tolerance, away from the
 * object along the jaws. The arm holding the hover point stopped short of it
 * toward the object (2026-10-01 07:56: the pan 10 steps short at 41.4 cm out,
 * the tip 0.67 cm toward the can), and the hover point again would not move it.
 * Aimed this far past it, the pan moves and stops within its tolerance of the
 * aim: between the hover point and this far beyond it, away from the object.
 */
export function CorrectedHover(Plan: GraspPlan): Point3 {
  const Shift = PanArrivalCm(Plan.Hover), U = Plan.JawAxis;
  return { x: Tenth(Plan.Hover.x - Shift * U.x), y: Tenth(Plan.Hover.y - Shift * U.y), z: Plan.Hover.z };
}

/**
 * Whether the jaws closed on nothing at a GRASP PLAN's grasp point: a close
 * approved, the gripper read nearly fully closed (not a narrow object, which a
 * grasp leaves that closed too), and the tip at the plan's grasp point when
 * the jaws closed (Close, NoteGripperMove: where the close was sent from,
 * counted only for the plan it closed with) or now (a plan made again after
 * the miss has its grasp point elsewhere). The plan they missed with is stale
 * (KeepOrReplan). "At the grasp point": anywhere the steps offer the close (the alignment's reach
 * away and across, and the close's above the printed point less its sag), or
 * where the arm holding it stops (ArrivalCm). The close the steps offer is
 * chained with a 6 cm lift, so by the next check-in the tip has risen from
 * the grasp point: 2026-10-01 18:56:43 judged by the tip alone (4.6 cm above
 * it) found no miss, kept the plan, and the step said it was made again.
 */
const ClosedAtCm = Math.max(GraspRules.ArrivalCm,
  Math.hypot(GraspRules.AlignAwayCm, GraspRules.AlignAcrossCm, GraspRules.CloseAboveCm + GraspRules.GraspSagCm));
export function ClosedOnNothing(Grasp: GraspState, Plan: GraspPlan | null, Tip: Point3, Close: TaskClose | null = null): boolean {
  const AtGrasp = (Point: Point3) => !!Plan && Math.hypot(Point.x - Plan.Grasp.x, Point.y - Plan.Grasp.y, Point.z - Plan.Grasp.z) <= ClosedAtCm;
  return !!Plan && Grasp.Closed && Grasp.GripperFraction < HeldFraction && Plan.WidthAcrossCm >= NarrowCm
    && (AtGrasp(Tip) || (!!Close && Close.Plan === Plan && AtGrasp(Close.Tip)));
}

/**
 * The one next step of a planned grasp, from where the tip and the gripper
 * are now: open, the hover point, the descent to the grasp point, close, lift.
 * The tip counts as above the grasp point only within the asymmetric
 * alignment (never more than 0.3 cm toward the object), and below the
 * object's top a misaligned tip rises straight up before anything else.
 * Confirmed: a second measurement agreed with the plan's. Missed: the jaws
 * closed on nothing with this plan and the object was not measured cleanly
 * since (GraspPlanState.Missed): no step descends with it. After a miss with
 * the plan made again (AfterMiss), the steps go on with the new plan.
 * Hidden: the arm over it hides it (ArmOcclusion.ts), so an unconfirmed plan's
 * steps do not wait for the image to show the jaws beside it (OnceText).
 */
export function NextPlannedGraspStep(Plan: GraspPlan, Tip: Point3, Grasp: GraspState, Label: string, Confirmed = true, Missed = false, AfterMiss = false, Hidden = false) {
  const Head = StepHead(Tip, Grasp);
  if (Plan.Approach) {
    const Side = Plan.Approach;
    // Release clearance can be smaller than servo arrival tolerance. Require its held target too,
    // otherwise the grasp point would look like the release point and skip lifting clear of the support.
    const At = (Point: Point3) => Grasp.Held ? Holds(Grasp, Tip, Point, Side.Pitch)
      : Math.hypot(Tip.x - Point.x, Tip.y - Point.y, Tip.z - Point.z) <= 0.35;
    const Move = (Point: Point3) => `"motion_proposal":{"target_cm":{"x":${Point.x.toFixed(1)},"y":${Point.y.toFixed(1)},"z":${Point.z.toFixed(1)}},"wrist_pitch_degrees":${Deg(Side.Pitch)}}`;
    if (Missed || (Grasp.Closed && Grasp.GripperFraction < HeldFraction))
      return `${Head}reply ask: the grasp is not confirmed. Keep the jaws and arm still; the space around the shelf does not permit an unplanned lift or opening.`;
    if (Grasp.Closed) {
      if (At(Side.Retreat)) return `${Head}the object has cleared the support. Reply task_status "done" only if the image shows it held in the jaws; otherwise reply ask. Do not open it here.`;
      if (At(Side.Release)) return `${Head}withdraw along the checked path with the wrist held at ${Deg(Side.Pitch)} deg: ${Move(Side.Retreat)}.${StepCopy}`;
      if (At(Plan.Grasp)) return `${Head}if the image shows it held in the jaws, move only to the checked release clearance: ${Move(Side.Release)}. Otherwise reply ask.${StepCopy}`;
      return `${Head}reply ask: the held object is outside the checked pickup path. Do not invent a lift or retreat.`;
    }
    if (Math.abs(Grasp.GripperFraction - Plan.OpenFraction) > GraspRules.OpenToleranceFraction) {
      if (At(Plan.Grasp)) return `${Head}reply ask: the jaws are not at the checked opening beside the object. Keep still and plan again.`;
      return `${Head}set the gripper to the checked ${Math.round(Plan.OpenFraction * 100)}% opening before entering: "motion_proposal":${GripperJson(Plan.OpenFraction)}.${StepCopy}`;
    }
    if (At(Plan.Grasp)) return `${Head}close the gripper only, then inspect the grasp: "motion_proposal":${GripperJson(0)}.${StepCopy}`;
    if (At(Side.Pregrasp)) return `${Head}enter from the open side at the checked wrist pitch: ${Move(Plan.Grasp)}.${StepCopy}`;
    return `${Head}${HoverStep(Plan, Label)}${StepCopy}`;
  }
  const Rise = (Z: number) => `"motion_proposal":{"move_delta_cm":{"left":0,"farther":0,"up":${PlanCm(Z - Tip.z)}}}`;
  const { Hover, Grasp: Point } = Plan, Top = Plan.Prism.TopZ;
  // Straight up to the HOVER POINT height, or above the target the arm holds there when its tip stopped under it (RiseUp).
  const HoverRise = RiseUp(Grasp, Tip, Hover.z);
  const ToHover = HoverRise.AboveHeld && Grasp.Held
    ? `Z ${(Tip.z + HoverRise.Up).toFixed(1)}, above the HOVER POINT height Z ${Hover.z.toFixed(1)} (the arm already holds a target at Z ${Grasp.Held.Point.z.toFixed(1)} and its tip stopped under it, so a rise to the hover height would not move it)`
    : `the HOVER POINT height Z ${Hover.z.toFixed(1)}`;
  const RiseToHover = `"motion_proposal":{"move_delta_cm":{"left":0,"farther":0,"up":${PlanCm(HoverRise.Up)}}}`;
  // What the next GRASP PLAN is made from after a miss: none yet (Missed), or the re-measure after the close
  // (AfterMiss, KeepOrReplan); a close the app did not count as this plan's miss leaves the plan as it was.
  const Again = Missed
    ? ` The camera has not measured the ${Label} clear of the gripper since, so the app has no GRASP PLAN for where it stands now: it plans the grasp again once the camera sees all of it. Do not descend before that.`
    : AfterMiss ? ` The GRASP PLAN above is made from where the camera measured the ${Label} after the close (the close may have pushed it), not where it was before.` : "";
  if (Grasp.Closed) {
    // The jaws on a narrow object read nearly fully closed: look before calling it a miss.
    if (Grasp.GripperFraction < HeldFraction && Plan.WidthAcrossCm < NarrowCm) {
      return Tip.z < Point.z + 2
        ? `${Head}the jaws closed nearly all the way, which on a ${Plan.WidthAcrossCm.toFixed(1)} cm wide object can still be a grasp. Lift 3 cm and check the image: "motion_proposal":{"move_delta_cm":{"left":0,"farther":0,"up":3}}.${StepCopy}`
        : `${Head}the jaws closed nearly all the way on a narrow object. If the image shows it in the jaws, lift it on: "motion_proposal":{"move_delta_cm":{"left":0,"farther":0,"up":3}}, and reply task_status "done" once it is clear of the desk; if not, they missed it: open the gripper, "motion_proposal":${GripperJson(GripperOpenFraction)}.`;
    }
    if (Grasp.GripperFraction < HeldFraction) {
      return Tip.z < Hover.z - 1
        ? `${Head}the jaws closed all the way, so they missed it. Rise straight up with the jaws closed first, to ${ToHover}: ${RiseToHover}.${Again}${StepCopy}`
        : `${Head}the jaws closed all the way, so they missed it. Open the gripper: "motion_proposal":${GripperJson(Plan.OpenFraction)}.${Again}${StepCopy}`;
    }
    if (Tip.z < Point.z + LiftCm - 1) {
      return `${Head}the jaws stopped partway, so they hold something. Lift it straight up: "motion_proposal":{"move_delta_cm":{"left":0,"farther":0,"up":${LiftCm}}}.${StepCopy}`;
    }
    return `${Head}it is lifted. If the image shows it in the jaws, reply task_status "done"; if not, reply ask.`;
  }
  // Missed with this plan and not measured cleanly since: away from the object, then ask.
  if (Missed) {
    return Tip.z < Hover.z - 1
      ? `${Head}the jaws missed the ${Label} at the GRASP PLAN's grasp point: rise straight up to ${ToHover}: ${RiseToHover}.${Again}${StepCopy}`
      : `${Head}reply ask: the jaws missed the ${Label} and the camera has not measured it clear of the gripper since, so the app has no GRASP PLAN for where it stands now. Tell the user to check that it is in clear view of the camera.`;
  }
  // The plan's open fraction: 80% (M1), or the smallest the jaw-plane sweep found clear (M3). The
  // descent needs the gripper within the steps' tolerance of it (the plan checked that band); an
  // M1 plan may stand wider (the guard checks the jaw where it is).
  const Open = Plan.OpenFraction, Tolerance = GraspRules.OpenToleranceFraction;
  const Beside = Tip.z < Hover.z - 1 && FootprintDistance(Plan.Prism.Footprint, Tip) <= GraspRules.NearCm;
  // d = tip - grasp point, along u (toward the object) and across it.
  const Dx = Tip.x - Point.x, Dy = Tip.y - Point.y;
  const Along = Dx * Plan.JawAxis.x + Dy * Plan.JawAxis.y, Across = -Dx * Plan.JawAxis.y + Dy * Plan.JawAxis.x;
  // A sweep plan's jaws must still reach around it with the tip here (GraspPlan.ts StraddleAtTip): short along
  // the fingers or away from it, they reach less far past its middle.
  const Around = StraddleAtTip(Plan, Tip);
  const Aligned = Along >= -GraspRules.AlignAwayCm && Along <= GraspRules.AlignTowardCm && Math.abs(Across) <= GraspRules.AlignAcrossCm
    && (Around == null || Around >= GraspRules.StraddleCloseCm);
  // The arm holds the hover point (or its corrected aim) and stopped off it: the
  // same point again would not move the arm (2026-10-01 07:56, the task stopped).
  const Aim = CorrectedHover(Plan);
  const AtAim = Holds(Grasp, Tip, Aim, Plan.HoverPitch);
  const AtHover = Holds(Grasp, Tip, Hover, Plan.HoverPitch) || AtAim;
  // With the gripper set, the step after it is the plain move to the HOVER POINT:
  // the gripper and that move are then offered as ONE chained motion (lib/AiChain.ts).
  const HoverNext = !Aligned && Tip.z >= Top - 0.3 && !AtHover;
  const Once = Confirmed ? "" : ` The ${Label} was ${OnceText(Plan, Hidden)}`;
  if (Grasp.GripperFraction < Open - Tolerance) {
    // Opening beside the object would swing the moving jaw into it: rise first.
    if (Beside) {
      const Less = Grasp.GripperFraction < OpenEnoughFraction ? "closed" : `less open than the GRASP PLAN's ${Math.round(Open * 100)}%`;
      return `${Head}the gripper is ${Less} beside the ${Label}: rise straight up to ${ToHover} before opening it: ${RiseToHover}.${StepCopy}`;
    }
    if (HoverNext) return `${Head}${SetAndHoverStep(Plan, Label, `open the gripper to ${Math.round(Open * 100)}%`, true)}${Once}${OpenChainCopy}`;
    return `${Head}open the gripper before approaching it: "motion_proposal":${GripperJson(Open)}.${StepCopy}`;
  }
  // Wider open than an M3 plan checked (it opens less, so the jaw stays clear of what is beside it):
  // set it to the plan's above the object's top (narrowing it there is not a close).
  if (Plan.Closing.Mode === "sweep" && Grasp.GripperFraction > Open + Tolerance) {
    if (Tip.z > Top + 1 || !Beside) {
      const Set = `set the gripper to the ${Math.round(Open * 100)}% open the GRASP PLAN was checked at (wider, the open jaw swings farther out; this is not the close)`;
      if (HoverNext) return `${Head}${SetAndHoverStep(Plan, Label, Set, false)}${Once}${ChainCopy}`;
      return `${Head}${Set}: "motion_proposal":${GripperJson(Open)}.${StepCopy}`;
    }
    return `${Head}the gripper is wider open than the GRASP PLAN's ${Math.round(Open * 100)}% beside the ${Label}: rise straight up to ${ToHover} first: ${RiseToHover}.${StepCopy}`;
  }
  // The close and the straight lift after it, as ONE chained motion (the lift is the step after any close).
  const Close = `${CloseAndLiftStep}${ChainCopy}`;
  if (!Aligned) {
    if (Tip.z < Top - 0.3) {
      return `${Head}the tip is below the ${Label}'s top but not at the grasp point: rise straight up first: ${RiseToHover}.${StepCopy}`;
    }
    if (AtHover) {
      const Off = `the arm holds the ${AtAim ? "HOVER POINT aimed past for the arm's stopping error" : "HOVER POINT"} but its tip stopped ${Math.abs(Along).toFixed(1)} cm ${Along > 0 ? `toward the ${Label}` : `away from the ${Label}`}`
        + `${Math.abs(Across) >= 0.05 ? ` and ${Math.abs(Across).toFixed(1)} cm across` : ""} from where the GRASP PLAN puts the fixed jaw`;
      if (!AtAim && Along > GraspRules.AlignTowardCm && Math.abs(Across) <= GraspRules.AlignAcrossCm) {
        return `${Head}${Off} (the arm stops each joint within about 1 deg of its target, here up to ${PanArrivalCm(Hover).toFixed(1)} cm along the jaws), so the fixed jaw would come down too near it.`
          + ` Do not descend from here: go to the hover point aimed ${PanArrivalCm(Hover).toFixed(1)} cm past it, away from the ${Label}, so the arm stops at it: "motion_proposal":{"target_cm":{"x":${Aim.x.toFixed(1)},"y":${Aim.y.toFixed(1)},"z":${Aim.z.toFixed(1)}},"wrist_pitch_degrees":${Deg(Plan.HoverPitch)}}.${StepCopy}`;
      }
      return `${Head}reply ask: ${Off}, and the same move again would not move the arm. Tell the user the arm did not stop where the GRASP PLAN needs the fixed jaw.`;
    }
    return `${Head}${HoverStep(Plan, Label)}${Confirmed ? "" : ` The ${Label} was ${OnceText(Plan, Hidden)}`}${StepCopy}`;
  }
  // Holding the grasp point, the tip stopped off it by the arm's stopping error: the descent or
  // the rise again would not move the arm. Close where the jaws are around it, if that is safe.
  // The close is offered within CloseBelowCm / CloseAboveCm of where the plan's jaws close: the
  // grasp point less the arm's sag it was printed above (GraspRules.GraspSagCm).
  const AtGrasp = Holds(Grasp, Tip, Point, Plan.Pitch);
  const CloseZ = Point.z - (Plan.SagCm ?? 0);
  if (Tip.z > CloseZ + GraspRules.CloseAboveCm) {
    if (AtGrasp) {
      // Higher than planned: the pad must still face it well below its top, and the moving jaw must not land on its top edge.
      return Plan.HeightBy !== "jaw" && Tip.z <= Top - 1
        ? `${Head}the arm holds the GRASP POINT and its tip stopped ${(Tip.z - Point.z).toFixed(1)} cm above it (the arm stops each joint within about 1 deg of its target), with the open jaws beside it: ${Close}`
        : `${Head}reply ask: the arm holds the GRASP POINT but its tip stopped ${(Tip.z - Point.z).toFixed(1)} cm above it, too high to close on the ${Label}, and the same move again would not move the arm.`;
    }
    return `${Head}the tip is above the grasp point${Confirmed ? "" : `, but the ${Label} was ${OnceText(Plan, Hidden).replace(/\.$/, "")}`}: descend straight down to the GRASP POINT Z ${Point.z.toFixed(1)}: "motion_proposal":{"move_delta_cm":{"left":${PlanCm(Point.x - Tip.x)},"farther":${PlanCm(Point.y - Tip.y)},"up":${PlanCm(Point.z - Tip.z)}},"wrist_pitch_degrees":${Deg(Plan.Pitch)}}.${StepCopy}`;
  }
  if (Tip.z >= CloseZ - GraspRules.CloseBelowCm) {
    return `${Head}the open jaws are beside it at the grasp point: ${Close}`;
  }
  if (AtGrasp) {
    // Lower than planned: only where the desk did not set the grasp height (closing, the jaws reach about 1 cm below it).
    return Plan.HeightBy !== "floor"
      ? `${Head}the arm holds the GRASP POINT and its tip stopped ${(Point.z - Tip.z).toFixed(1)} cm below it (the arm stops each joint within about 1 deg of its target), with the open jaws beside it: ${Close}`
      : `${Head}reply ask: the arm holds the GRASP POINT but its tip stopped ${(Point.z - Tip.z).toFixed(1)} cm below it, too near the desk to close, and the same move again would not move the arm.`;
  }
  return `${Head}the tip is below the grasp point: rise straight up to the GRASP POINT Z ${Point.z.toFixed(1)}: ${Rise(Point.z)}.${StepCopy}`;
}

// A task that only goes above its object (hover, point, move the arm to it): the hover move alone.
function HoverMoveFromTip(Object_: LocatedObject, Tip: Point3, Reach: TaskReachability | null) {
  const { HoverZ } = PrintedHeights(Object_);
  const X = Tenth(Object_.x), Y = Tenth(Object_.y);
  const Wrist = Reach?.HoverPitch != null ? `,"wrist_pitch_degrees":${Deg(Reach.HoverPitch)}` : "";
  return ` READY-MADE MOVE from the tip now (X ${Tip.x.toFixed(1)}, Y ${Tip.y.toFixed(1)}, Z ${Tip.z.toFixed(1)}): hover above it: "motion_proposal":{"target_cm":{"x":${X.toFixed(1)},"y":${Y.toFixed(1)},"z":${HoverZ.toFixed(1)}}${Wrist}}`
    + ` (the same as move_delta_cm {"left":${Cm(X - Tip.x)},"farther":${Cm(Y - Tip.y)},"up":${Cm(HoverZ - Tip.z)}}). Copy these numbers exactly; this task does not descend onto it.`;
}

/**
 * The guard a task's proposals are checked against (GraspPlan.ts
 * GraspGuardReason): this frame's measured objects as prisms, the task
 * object's (its plan's, else its measurement's), the plan or the refusal,
 * and whether the jaws hold it. Null without a task object. A plan the jaws
 * missed with (GraspPlanState.Missed) is stale: the guard holds the object
 * where it was measured, without the plan, so it refuses a descent beside or
 * onto it (the stale grasp point's too) until the app plans it again.
 * Step: the TASK OBJECT line's NEXT STEP (PlannedStepOf), held only where
 * NextStepReason holds it: a grasp with a GRASP PLAN (the missed one's too)
 * whose jaws do not hold the object. So a guard's Step is exactly the step
 * the app refuses other moves near the object for (NextStepReminder says so).
 */
export function BuildGraspGuard(Measured: LocatedObject[] | null, State: TaskObjectState | null, Held: boolean,
  Step: PlannedStep | null = null): GraspGuard | null {
  if (!State) return null;
  const Plan = State.Plan?.Missed ? null : State.Plan?.Reach.Plan ?? null;
  const Current = (State.DetectedNow ? State.Binding?.Object : null) ?? State.LastSeen?.Object ?? null;
  const Verdict = State.DetectedNow ? State.Reachability : State.LastSeen?.Reachability ?? null;
  const { Prisms, TaskPrism } = GuardPrisms(Measured ?? [], Current, Plan, State.Plan?.Source ?? null);
  // A re-measure that came toward the fixed jaw is held against the gripper too.
  const Corridor = State.Plan?.Corridor?.Prism;
  const Intent = State.Intent ?? TaskIntent(State.Query);
  const Holds = !!State.Plan?.Reach.Plan && Intent === "grasp" && !Held && !!TaskPrism;
  // The surface it stands on: every target held to its floor, every way to them clear of a raised one (TaskSupport, FloorPrism).
  const Floor = TaskSupport(Current, State.Label).Floor, Surface = Floor.Scene ? null : FloorPrism(Floor);
  return { Prisms: [...Prisms, ...(Corridor ? [Corridor] : []), ...(Surface ? [Surface] : []), ...ScenePrisms(Floor)], TaskPrism, Plan, Refusal: Verdict?.Refusal ?? null, Held,
    Intent, TaskLabel: State.Label, Step: Holds ? Step : null, ...(Floor.Surface ? { Floor } : {}) };
}

// The approach advice and the ready-made moves come only with a grasp the app
// judged possible (or no verdict at all): BLOCKED and OUT OF REACH mean reply
// ask, and a hover hint there made the model move anyway.
const Approachable = (Reach: TaskReachability | null) => !Reach || (Reach.Kind === "graspable" && Reach.HoverPitch !== null);

// How the measurement was picked, when its label is not simply the task's word.
const PickedBy: Record<TaskObjectBinding["Source"], string> = { query: "picked by the task query", name: "",
  position: "tracked at the same spot", colour: "the only label with its colour", estimate: "from your target_estimate_cm" };
function MeasuredAs(State: TaskObjectState) {
  const Binding = State.Binding;
  if (!Binding) return "";
  const How = PickedBy[Binding.Source];
  if (!How && Binding.Object.label.toLowerCase() === State.Label.toLowerCase()) return "";
  // A noun no label names, bound to a label that may be it under another name: the model checks the image.
  const Head = State.Query?.Head ?? null;
  const Unnamed = Head && (Binding.Source === "colour" || Binding.Source === "estimate") && !NamesQuery(State.Query!, Binding.Object.label)
    ? `; no measured label names a "${Head}": check in the image that it is the ${State.Label}, else reply ask` : "";
  return ` (measured as "${Binding.Object.label}"${How ? `, ${How}` : ""}${Unnamed})`;
}
// Candidates nearest the right arm's pan axis first; the far ones add little.
const MaxCandidates = 6;
function CandidateList(Candidates: LocatedObject[]) {
  const Pan = mirrorDisplayPoint({ x: 0, y: 0, z: 0 });
  const FromPan = (Object_: LocatedObject) => Math.hypot(Object_.x - Pan.x, Object_.y - Pan.y);
  return Candidates.slice().sort((A, B) => FromPan(A) - FromPan(B)).slice(0, MaxCandidates)
    .map(Object_ => `${Object_.label} (X ${Object_.x.toFixed(1)}, Y ${Object_.y.toFixed(1)}${Object_.supportBranch === "bin" ? ", in the bin" : ""})`).join(", ");
}

// A task that names no object. "ok i did that" (2026-10-01 07:56) became such a task, and the
// model went for the tape roll it saw, at wrist -90 that the arm could not reach there.
export const NoTaskObjectLine = "TASK OBJECT: none: the task names no object. Do not choose an object to grasp, touch or move yourself,"
  + " not even one in the image or in MEASURED OBJECT POSITIONS: if the task needs one and does not say which (and none is already in the gripper), reply ask and ask the user which object.";

/** The NEXT STEP sentence of a TASK OBJECT line, for feedback that must repeat it; null without one. */
export function NextStepOf(Line: string): string | null {
  const Match = Line.match(/NEXT STEP of the grasp[\s\S]*?(?= Coordinates in (?:your )?earlier replies|$)/);
  return Match ? Match[0].trim() : null;
}

const MotionKey = '"motion_proposal":';
/** The one "motion_proposal" JSON object in a NEXT STEP sentence, verbatim and parsed; null without exactly one. */
function StepJson(Text: string): { Json: string; Value: Record<string, unknown> } | null {
  const First = Text.indexOf(MotionKey);
  if (First < 0 || Text.indexOf(MotionKey, First + MotionKey.length) >= 0) return null;
  const Start = First + MotionKey.length;
  let Depth = 0;
  for (let Index = Start; Index < Text.length; Index++) {
    Depth += Text[Index] === "{" ? 1 : Text[Index] === "}" ? -1 : 0;
    if (Depth > 0) continue;
    const Json = Text.slice(Start, Index + 1);
    try {
      const Value: unknown = JSON.parse(Json);
      return Value && typeof Value === "object" && !Array.isArray(Value) ? { Json, Value: Value as Record<string, unknown> } : null;
    } catch { return null; }
  }
  return null;
}
const Finite = (Value: unknown) => (typeof Value === "number" && Number.isFinite(Value) ? Value : null);
const Fields = (Value: unknown) => (Value && typeof Value === "object" && !Array.isArray(Value) ? Value as Record<string, unknown> : null);

/**
 * The NEXT STEP a TASK OBJECT line gives (GraspPlan.ts PlannedStep), for the
 * rule that holds a planned grasp's moves near the object to it (NextStepReason)
 * and for the reminder at the end of the prompt. Anchor and AnchorPitch: the
 * tip its relative moves start from (display frame), as the app reads a reply
 * (App.tsx parseAiMotionProposalFromObject: an axis left out keeps the tip's).
 */
export function PlannedStepOf(Line: string | null, Anchor: Point3, AnchorPitch: number): PlannedStep | null {
  const Text = Line ? NextStepOf(Line) : null;
  if (!Text) return null;
  const Found = StepJson(Text);
  if (!Found) return { Text, Json: null, Target: null, Pitch: null };
  const Absolute = Fields(Found.Value.target_cm), Delta = Fields(Found.Value.move_delta_cm);
  const Target = Absolute ? { x: Finite(Absolute.x) ?? Anchor.x, y: Finite(Absolute.y) ?? Anchor.y, z: Finite(Absolute.z) ?? Anchor.z }
    : Delta ? { x: Anchor.x + (Finite(Delta.left) ?? 0), y: Anchor.y + (Finite(Delta.farther) ?? 0), z: Anchor.z + (Finite(Delta.up) ?? 0) } : null;
  return { Text, Json: Found.Json, Target, Pitch: Target ? Finite(Found.Value.wrist_pitch_degrees) ?? AnchorPitch : null };
}

/**
 * The last line of a check-in whose TASK OBJECT line gives a NEXT STEP to
 * copy: its JSON again, where the model reads last (2026-10-01 18:54: the
 * step ended a 2,700-character line, and the model descended instead). Null
 * without one motion_proposal to copy.
 */
export function NextStepReminder(Step: PlannedStep | null, Label: string): string | null {
  if (!Step?.Json) return null;
  return `NEXT STEP TO COPY (from the TASK OBJECT line; the app refuses any other move near the ${Label}): "motion_proposal":${Step.Json}.`
    + " Copy it exactly as your motion_proposal, or reply task_status \"ask\" with motion_proposal null if you think it is wrong.";
}

/** One line per check-in saying whether the task object was seen, and where it was last measured. */
export function TaskObjectLine(State: TaskObjectState, Tip?: Point3, Grasp?: GraspState): string {
  const Seen = State.LastSeen?.Object;
  const Tag = Seen && IsUnreliable(Seen, Seen) ? ` [UNRELIABLE: ${Reasons(Seen, Seen).join("; ")}]` : "";
  const Approach = State.Intent === "approach", Contact = State.Intent === "contact";
  // A planned grasp: the plan and its next step (without the gripper's state,
  // only the hover). Re-measured toward the fixed jaw under the gripper: ask.
  // A plan the jaws missed with (GraspPlanState.Missed): its points and its
  // description are stale, so only its NEXT STEP (away from the object) is given.
  const Corridor = State.Plan?.Corridor ?? null;
  const Missed = !!State.Plan?.Missed;
  const ReachText = (Reach: TaskReachability) => Missed && Reach.Plan
    ? `the GRASP PLAN the jaws missed the ${State.Label} with is stale, so the app gives no GRASP POINT or HOVER POINT for it until the camera measures the ${State.Label} clear of the gripper and the app plans the grasp again`
    : Reach.Text;
  const Planned = (Plan: GraspPlan) => (Missed ? "" : PlanText(Plan, State.Label)) + (!Tip ? ""
    : Corridor && !Grasp?.Closed ? ` NEXT STEP of the grasp: reply ask: the ${State.Label} may have moved into the gripper's way (${Corridor.Why}), so the GRASP PLAN may no longer clear it.`
      : Grasp ? NextPlannedGraspStep(Plan, Tip, Grasp, State.Label, State.Plan?.Confirmed ?? true, Missed, !!State.Plan?.AfterMiss, !!State.Occluded)
        : Missed ? "" : ` NEXT STEP of the grasp: ${HoverStep(Plan, State.Label)}`);
  // A plan kept from an earlier measurement, or made again since: why.
  const Note = State.Plan?.Note ? ` (${State.Plan.Note})` : "";
  // The arm over or beside it hides it (ArmOcclusion.ts): its last full measurement and GRASP PLAN
  // stand, and the steps go on from them; measured only partly, the plan lock's note says why it kept the plan.
  if (State.Occluded && Seen && State.LastSeen) {
    const Reach = State.LastSeen.Reachability, Plan = Reach?.Plan ?? null;
    const Moves = !Tip || !Grasp || Approach || !Approachable(Reach) ? "" : Plan ? Planned(Plan) : NextGraspStep(Seen, Tip, Reach, Grasp, Contact);
    return `TASK OBJECT "${State.Label}": ${HiddenText(State.Label, Seen, State.LastSeen.At, State.Occluded, !!Plan, Moves.includes("NEXT STEP"), Tag)}`
      + (State.Occluded.Partial && State.Plan?.Note ? ` The GRASP PLAN: ${State.Plan.Note}.` : "")
      + (Reach ? ` RIGHT ARM (from that measurement): ${ReachText(Reach)}.` : "") + (Tag && Approachable(Reach) ? UnreliableAdvice : "") + Moves
      + " Coordinates in earlier replies are stale: use only this measurement.";
  }
  if (State.DetectedNow && Seen) {
    // On a declared surface: which, and the floor the task holds the arm to (TaskSupport).
    const OnSurface = Seen.supportBranch === "surface" ? TaskSupport(Seen, State.Label).Floor : null;
    const Top = (Seen.topZ != null ? `, top Z ${Seen.topZ.toFixed(1)}` : "")
      + (OnSurface?.Surface ? `, standing on ${SurfaceText(OnSurface.Surface)}: the tip and the gripper stay above ${FloorText(OnSurface)}` : "");
    const Advice = Tag && Approachable(State.Reachability) ? UnreliableAdvice : "";
    const Plan = State.Reachability?.Plan ?? null;
    const Moves = !Approachable(State.Reachability) ? "" : Plan ? Planned(Plan) : !Tip ? ""
      : Approach ? HoverMoveFromTip(Seen, Tip, State.Reachability)
        : Grasp ? NextGraspStep(Seen, Tip, State.Reachability, Grasp, Contact) : MovesFromTip(Seen, Tip, State.Reachability);
    return `TASK OBJECT "${State.Label}": detected this check-in${MeasuredAs(State)} at X ${Seen.x.toFixed(1)}, Y ${Seen.y.toFixed(1)}, base Z ${Seen.z.toFixed(1)}${Top}${Tag}${Note}.`
      + (State.Reachability ? ` RIGHT ARM: ${ReachText(State.Reachability)}.` : "")
      + Advice + Moves
      + " Coordinates in your earlier replies are stale: use only this measurement.";
  }
  if (Seen && State.LastSeen) {
    const Reach = State.LastSeen.Reachability ? ` RIGHT ARM when last measured: ${ReachText(State.LastSeen.Reachability)}.` : "";
    const Advice = Tag && Approachable(State.LastSeen.Reachability) ? UnreliableAdvice : "";
    // Under the jaws, or carried in them after a close: the grasp goes on from
    // its last measurement. With a plan the tip is beside it, not over its centre.
    const Plan = State.LastSeen.Reachability?.Plan ?? null;
    const Hidden = Tip && Grasp && !Approach && Approachable(State.LastSeen.Reachability)
      && (Grasp.Closed || FromFootprint(Seen, Tip, Plan?.Prism ?? null) <= 3 || (!!Plan && Math.hypot(Tip.x - Plan.Grasp.x, Tip.y - Plan.Grasp.y) <= 3));
    const Moves = !Hidden ? "" : Plan ? Planned(Plan) : NextGraspStep(Seen, Tip!, State.LastSeen.Reachability, Grasp!, Contact);
    return `TASK OBJECT "${State.Label}": NOT detected this check-in (possibly under the gripper or out of view); last measured at ${Clock(State.LastSeen.At)} at X ${Seen.x.toFixed(1)}, Y ${Seen.y.toFixed(1)}${Tag}.${Reach}${Advice}${Moves} Coordinates in earlier replies are stale: do not reuse them.`;
  }
  // "the black thing" (16:02 was told only 'the task object "thing" is not
  // detected'), or a noun measured at two spots: the model says which one.
  if (State.Candidates?.length) {
    const Named = !!State.Query?.Head && State.Candidates.some(Object_ => NamesQuery(State.Query!, Object_.label));
    return `TASK OBJECT "${State.Label}": ${Named ? "more than one measured object could be it" : "no measured label names it"}.`
      + ` Candidates measured this check-in: ${CandidateList(State.Candidates)}.`
      + " If one of them is it, set target_estimate_cm to its measured centre and the app will track it and check its reach; otherwise reply ask.";
  }
  return `TASK OBJECT "${State.Label}": NOT detected in any check-in of this task. Do not guess its coordinates: if you cannot see it in the image, reply ask.`;
}

// Wrist pitches the feedback hints try (AiTaskLoop.ts): they run for every
// rejected proposal on the page thread, so they stay coarse. The task object's
// verdict below scans every 2.5 deg instead.
export const ReachPitches = [-90, -75, -60, -45, -30, -15, 0];
// ClearanceCm: the user's grasp gap (GraspGap.ts), which a grasp's GRASP PLAN puts the fixed pad at (GraspContext).
// Floor: the task's (TaskSupport); TaskObjectReachability finds it from the object itself.
export type ReachContext = { Servos: Record<ServoName, ServoState>; ElbowMode: "up" | "down"; Limits?: IkAngleLimits | null; ClearanceCm?: number | null;
  Floor?: AiFloor | null };

/** The first wrist pitch that reaches a model-frame point clear of the basket, with the ways the others failed. */
export function ReachAt(Point: Point3, Context: ReachContext, Pitches = ReachPitches) {
  let Collision: string | null = null, BestMiss = Number.POSITIVE_INFINITY;
  for (const Pitch of Pitches) {
    // null would mean no joint limits at all; undefined applies the slider limits.
    const Preview = solveIkPreview({ ...Point, wristPitch: Pitch }, Context.ElbowMode, Context.Servos, Context.Limits ?? undefined);
    if (Preview.reachable) return { Pitch, Collision: null, BestMiss: Preview.errorCm };
    if (Preview.collisionReason) Collision ??= Preview.collisionReason;
    else BestMiss = Math.min(BestMiss, Preview.errorCm);
  }
  return { Pitch: null, Collision, BestMiss };
}

// --- Where the right arm reaches the task object -----------------------------

// 2.5 deg steps from straight down to level: near the desk the pitches that
// reach an object and clear the basket can be a window a few degrees wide
// between the steep ones that fall short and the flat ones that touch the
// basket, and 15 deg samples skipped windows up to 12 deg wide.
const ScanPitches = Array.from({ length: 37 }, (_, Index) => -90 + Index * 2.5);
export type ReachFailure = { Pitch: number; Kind: "reach" | "collision"; Reason: string | null };
export type ReachScanResult = {
  // The steepest pitch that reaches the point clear of the basket, inset like an AI proposal's.
  Pitch: number | null;
  // How the steeper pitches failed; with no Pitch, every scanned pitch.
  Failures: ReachFailure[];
  BestMiss: number;
  // The steepest pitch the IK reaches at all, collisions aside (only judged without a Pitch).
  Steepest: number | null;
};

/**
 * The steepest wrist pitch in -90..0 that reaches a model-frame point clear
 * of the basket where the IK converges, moved a little flatter so later checks
 * with other servo readings and the bridge still reach it (InsetWristPitch).
 */
export function ReachScan(Point: Point3, Context: ReachContext): ReachScanResult {
  const Workspace = { Servos: Context.Servos, ElbowMode: Context.ElbowMode, Limits: Context.Limits ?? undefined };
  const Converges = (Pitch: number) => WristPitchConverges(Point, Pitch, Workspace);
  if (BeyondArmReach(Point)) {
    // Out of reach at every pitch: the closest the tip gets, from one free-pitch fit, instead of a scan.
    const Free = CadSolve(Point, 0, Context.ElbowMode, currentAngles(Context.Servos), GetIkAngleLimits(Context.Servos, Workspace.Limits), true);
    return { Pitch: null, Failures: ScanPitches.map(Pitch => ({ Pitch, Kind: "reach", Reason: null })), BestMiss: Free.ErrorCm, Steepest: null };
  }
  const Failures: ReachFailure[] = [];
  let BestMiss = Number.POSITIVE_INFINITY;
  for (const [Index, Pitch] of ScanPitches.entries()) {
    const Preview = solveIkPreview({ ...Point, wristPitch: Pitch }, Context.ElbowMode, Context.Servos, Workspace.Limits);
    if (PreviewConverges(Preview, Pitch)) {
      if (Index === 0) return { Pitch, Failures, BestMiss: 0, Steepest: Pitch };
      // The edge between the steeper sample that failed and this one, then a step inside it.
      let Valid = Pitch, Invalid = ScanPitches[Index - 1];
      for (let Step = 0; Step < 5; Step++) {
        const Middle = (Valid + Invalid) / 2;
        if (Converges(Middle)) Valid = Middle; else Invalid = Middle;
      }
      return { Pitch: InsetWristPitch(Valid, Invalid, Converges), Failures, BestMiss: 0, Steepest: null };
    }
    // Reached but only at the edge of the IK tolerance counts as out of reach.
    if (Preview.collisionReason) Failures.push({ Pitch, Kind: "collision", Reason: Preview.collisionReason });
    else { Failures.push({ Pitch, Kind: "reach", Reason: null }); BestMiss = Math.min(BestMiss, Preview.errorCm); }
  }
  return { Pitch: null, Failures, BestMiss, Steepest: SteepestReached(Point, Context, Failures) };
}

// The steepest pitch the IK reaches, basket aside: the edge before the first
// pitch that failed only on a collision (a pose is checked for contact only
// once the IK reaches it).
function SteepestReached(Point: Point3, Context: ReachContext, Failures: ReachFailure[]) {
  const First = Failures.findIndex(Failure => Failure.Kind === "collision");
  if (First < 0 || /material/.test(Failures[First].Reason ?? "")) return null;
  if (First === 0) return Failures[0].Pitch;
  const Reached = (Pitch: number) => {
    const Preview = solveIkPreview({ ...Point, wristPitch: Pitch }, Context.ElbowMode, Context.Servos, Context.Limits ?? undefined);
    return Preview.errorCm <= 0.1 && Preview.wristPitchReachable;
  };
  let Valid = Failures[First].Pitch, Invalid = Failures[First - 1].Pitch;
  for (let Step = 0; Step < 5; Step++) {
    const Middle = (Valid + Invalid) / 2;
    if (Reached(Middle)) Valid = Middle; else Invalid = Middle;
  }
  return Valid;
}

/**
 * How the scanned wrist pitches failed, as runs ("out of reach at wrist
 * -90..-62.5 deg; gripper link could contact the basket at wrist -60..-45
 * deg"). "At every wrist pitch" only when one collision failed them all: it
 * used to name the first collision found even where steeper pitches fell short.
 */
export function ReachFailureText(Failures: ReachFailure[]): string {
  const Runs: { Kind: ReachFailure["Kind"]; Reason: string | null; From: number; To: number }[] = [];
  for (const Failure of Failures) {
    const Last = Runs[Runs.length - 1];
    if (Last && Last.Kind === Failure.Kind && Last.Reason === Failure.Reason) Last.To = Failure.Pitch;
    else Runs.push({ Kind: Failure.Kind, Reason: Failure.Reason, From: Failure.Pitch, To: Failure.Pitch });
  }
  const Range = (Run: typeof Runs[number]) => Run.From === Run.To ? `${Deg(Run.From)} deg` : `${Deg(Run.From)}..${Deg(Run.To)} deg`;
  if (Runs.length === 1 && Runs[0].Kind === "collision") return `${Runs[0].Reason} at every wrist pitch (${Range(Runs[0])})`;
  return Runs.map(Run => Run.Kind === "reach" ? `out of reach at wrist ${Range(Run)}` : `${Run.Reason} at wrist ${Range(Run)}`).join("; ");
}

/**
 * Whether the controlled (right) arm can grasp the task object, judged with
 * the app's own IK and collision model at the heights and X/Y the model is
 * given. Never the bridge's stateful preview: that would replace the solution
 * the bridge keeps for the next Send. Label: the task's name for it
 * (TaskObject.Label), which a grasp's refusal and its advice use, as the
 * stop's headline does; without one, the detector's label. 2026-10-04 08:30:
 * "the pill bottle ... is not graspable" advised moving "the white cup".
 */
// What a grasp's verdict is planned with besides the object (TaskObjectReachability): its neighbours, the task's word for it.
export type ReachPlanned = { Others: LocatedObject[]; Advise?: boolean; Kind?: string | null; Label?: string | null };
export function TaskObjectReachability(Object_: LocatedObject, Context: ReachContext, Planned?: ReachPlanned): TaskReachability {
  const Label = Planned?.Label || Object_.label;
  const { GraspZ, HoverZ } = PrintedHeights(Object_);
  const Model = (Z: number) => modelPointFromDisplayPoint({ x: Tenth(Object_.x), y: Tenth(Object_.y), z: Z });
  // The model frame's origin is the pan axis.
  const BaseDistanceCm = Math.hypot(Model(GraspZ).x, Model(GraspZ).y);
  const Where = `${BaseDistanceCm.toFixed(1)} cm from the right arm's pan axis`;
  // On a surface besides the desk (back end/DeclaredSurfaces.json): planned and checked against its floor
  // (TaskSupport), top-down as on the desk; refused where the app cannot hold the arm to it.
  const Support = TaskSupport(Object_, Label);
  if (Support.Refusal) {
    return { Kind: "ungraspable", BaseDistanceCm, GraspPitch: null, HoverPitch: null, CentrePitch: null, Plan: null, Refusal: Support.Refusal,
      Text: `NOT GRASPABLE (${Where}): ${Support.Refusal.Text}. Do not move toward it: reply ask and tell the user why` };
  }
  if (Support.Floor.Surface) Context = { ...Context, Floor: Support.Floor };
  let Grasp = ReachScan(Model(GraspZ), Context);
  // A grasp whose centre the arm does not reach at mid-height may be reached higher up its side
  // (2026-09-30 19:10: the pill container BLOCKED at Z 1.3 by the basket, reached from Z 1.5):
  // planned from the pitch that reaches it there; the planner raises the grasp point (RaisedPlan).
  if (Planned && Grasp.Pitch == null && Object_.topZ != null && !BeyondArmReach(Model(GraspZ))) {
    for (let Raise = GraspRules.RaiseStepCm; Raise <= GraspRules.MaxRaiseCm + 1e-9 && GraspZ + Raise <= Object_.topZ - GraspRules.RaiseBelowTopCm; Raise += GraspRules.RaiseStepCm) {
      const Higher = ReachScan(Model(Tenth(GraspZ + Raise)), Context);
      if (Higher.Pitch != null) { Grasp = Higher; break; }
    }
    // Or lower (GraspRules.BestEffort: the planner's best-effort pass grasps it lower too).
    for (let Lower = GraspRules.RaiseStepCm; GraspRules.BestEffort && Grasp.Pitch == null && Lower <= GraspRules.LowerCm + 1e-9
      && GraspZ - Lower >= Object_.z + GraspRules.RaiseBelowTopCm; Lower += GraspRules.RaiseStepCm) {
      const Below = ReachScan(Model(Tenth(GraspZ - Lower)), Context);
      if (Below.Pitch != null) Grasp = Below;
    }
  }
  // Nor at its centre higher up: beside it across the arm's reach, where the fixed jaw goes (GraspPlan.ts PlaceAtPitch
  // starts there too), so a pose the grasp never takes does not make it BLOCKED or out of reach.
  if (Planned && Grasp.Pitch == null && Object_.topZ != null && Object_.sizeCm && !BeyondArmReach(Model(GraspZ))) {
    const At = Model(GraspZ), R = Math.hypot(At.x, At.y) || 1, Out = Math.max(...Object_.sizeCm) / 2 + GraspRules.FixedJawClearanceCm + 0.8;
    for (const Sign of [1, -1]) {
      const Beside = ReachScan({ x: At.x - Sign * Out * At.y / R, y: At.y + Sign * Out * At.x / R, z: At.z }, Context);
      if (Beside.Pitch != null) { Grasp = Beside; break; }
    }
  }
  // A grasp (Planned): the fixed jaw goes beside the object (GraspPlan.ts),
  // planned from the pitch that reaches its centre; the neighbours are Others.
  if (Planned && (Grasp.Pitch != null || (Context.Floor?.Scene && Context.Floor.Surface?.Kind === "shelf"))) {
    // Refused with the pad at a gap off the default (the user's grasp gap): where another gap may plan it, the refusal says which (GraspGapHint).
    const Gap = (Refusal: GraspRefusal): GraspRefusal => {
      const Hint = GraspGapHint(Refusal);
      return Hint ? { ...Refusal, Text: `${Refusal.Text}; ${Hint.Text}`, Advice: `${Refusal.Advice} ${Hint.Advice}` } : Refusal;
    };
    const Refused = (Refusal_: GraspRefusal): TaskReachability => { const Refusal = Gap(Refusal_);
      return { Kind: "ungraspable", BaseDistanceCm, GraspPitch: null, HoverPitch: null,
        CentrePitch: Grasp.Pitch, Plan: null, Refusal, Text: `NOT GRASPABLE (${Where}): ${Refusal.Text}. Do not move toward it: reply ask and tell the user why` }; };
    // Kind: the task's word for it ("can"): a can is planned as one whatever the detector called it.
    const Measured = GraspPrism(Object_, Planned.Kind ?? null);
    if (!Measured) return Refused(UnmeasuredRefusal(Label));
    const Box = { ...Measured, Label };
    // Neighbours as the guard holds them: a round one at its widest; a raised surface it stands on too (FloorPrism).
    const Others = [...Planned.Others.map(Other => GraspPrism(Other)), ...(Context.Floor?.Scene ? ScenePrisms(Context.Floor) : [FloorPrism(Context.Floor)])].filter((Other): Other is Prism => Other !== null);
    const Result = PlanGrasp(Box, Grasp.Pitch ?? 0, { ...Context, Others, Scan: Point => ReachScan(Point, Context).Pitch }, Planned.Advise !== false);
    if (!Result.Plan) return Refused(Result.Refusal);
    const Plan = Result.Plan;
    return { Kind: "graspable", BaseDistanceCm, GraspPitch: Plan.Pitch, HoverPitch: Plan.HoverPitch, CentrePitch: Grasp.Pitch, Plan, Refusal: null,
      Text: GraspableText(Plan, BaseDistanceCm) };
  }
  if (Grasp.Pitch != null) {
    // The descent keeps the hover's wrist when the grasp pitch reaches the hover too.
    const Workspace = { Servos: Context.Servos, ElbowMode: Context.ElbowMode, Limits: Context.Limits ?? undefined };
    const HoverPitch = WristPitchConverges(Model(HoverZ), Grasp.Pitch, Workspace) ? Grasp.Pitch : ReachScan(Model(HoverZ), Context).Pitch;
    return { Kind: "graspable", BaseDistanceCm, GraspPitch: Grasp.Pitch, HoverPitch,
      Text: `graspable at Z ${GraspZ.toFixed(1)} with wrist ${Deg(Grasp.Pitch)} deg; `
        + (HoverPitch != null ? `hover waypoint Z ${HoverZ.toFixed(1)} with wrist ${Deg(HoverPitch)} deg` : `no wrist pitch reaches the hover waypoint Z ${HoverZ.toFixed(1)} above it`)
        + ` (${Where})` };
  }
  // Blocked: the basket stands in the way, or the arm reaches above the object but not down to it.
  if (Grasp.Failures.some(Failure => Failure.Kind === "collision") || ReachScan(Model(HoverZ), Context).Pitch != null) {
    const Steepest = Grasp.Steepest != null ? `; the steepest wrist pitch the arm reaches there is ${Deg(Grasp.Steepest)} deg` : "";
    return { Kind: "blocked", BaseDistanceCm, GraspPitch: null, HoverPitch: null,
      Text: `grasp height Z ${GraspZ.toFixed(1)} BLOCKED (${Where}${Steepest}): ${ReachFailureText(Grasp.Failures)}` };
  }
  const OutOfReach = `OUT OF REACH (${Where}; the closest pose misses by ${Grasp.BestMiss.toFixed(1)} cm)`;
  // A grasp: how much closer the app would plan it, so the user is told where to put it.
  const Spot = Planned && Planned.Advise !== false ? CloserToGrasp(Object_, Context, Planned.Others, Grasp.BestMiss, Planned.Kind ?? null) : null;
  if (Spot) {
    const There = `(its centre about ${(BaseDistanceCm - Spot.Cm).toFixed(0)} cm from the pan axis)`;
    return { Kind: "out-of-reach", BaseDistanceCm, GraspPitch: null, HoverPitch: null, CloserCm: Spot.Cm,
      Text: `${OutOfReach}; ${Spot.TurnDeg === 0 ? `about ${Spot.Cm} cm closer to the arm ${There}` : `moved about ${Spot.MoveCm.toFixed(0)} cm to X ${Spot.At.x.toFixed(1)}, Y ${Spot.At.y.toFixed(1)} ${There}`} the app plans the grasp`,
      Advice: CloserAdvice(Object_, Spot, Label) };
  }
  return { Kind: "out-of-reach", BaseDistanceCm, GraspPitch: null, HoverPitch: null, Text: OutOfReach };
}

// The reach verdict of a GRASP PLAN, as the model reads it.
const GraspableText = (Plan: GraspPlan, BaseDistanceCm: number) => `graspable${Plan.Warnings?.length ? " (best effort: the app is not sure it holds)" : ""}: GRASP POINT ${PlanPoint(Plan.Grasp)} with wrist ${Deg(Plan.Pitch)} deg; `
  + `HOVER POINT ${PlanPoint(Plan.Hover)} with wrist ${Deg(Plan.HoverPitch)} deg (${BaseDistanceCm.toFixed(1)} cm from the right arm's pan axis)`;

/**
 * The closed-loop fixed-finger correction at the hover check-in (GraspPlan.ts
 * TapeCorrection): with the tip at the plan's hover point and this frame's
 * tape check judged ("ok", with its residual), the plan moves by the tape's
 * residual along the jaw axis, once per plan. The new verdict and the note
 * the model is told (the GRASP POINT never moves silently); null: nothing to
 * correct now (no residual, not at the hover, or already corrected).
 */
export function TapeCorrectedReach(Reach: TaskReachability, Check: CameraCheck | null, Tip: Point3, Context: ReachContext,
  Others: LocatedObject[], Label: string): { Reach: TaskReachability; Note: string; Fix: TapeFix; Moved: boolean } | null {
  const Plan = Reach.Plan;
  if (!Plan || Plan.Approach || Plan.Tape || Check?.TapeCheck !== "ok" || !Check.TapeResidualCm || !AtHoverPoint(Plan, Tip)) return null;
  const Neighbours = [...Others.map(Other => GraspPrism(Other)), FloorPrism(Context.Floor)].filter((Other): Other is Prism => Other !== null);
  const Result = TapeCorrection(Plan, Check.TapeResidualCm, { ...Context, Others: Neighbours, Scan: Point => ReachScan(Point, Context).Pitch });
  if (!Result.Plan) {
    // Kept as planned, marked so it is not tried again every check-in.
    return { Reach: { ...Reach, Plan: { ...Plan, Tape: { ...Result.Fix, ShiftCm: 0 } } }, Fix: Result.Fix, Moved: false,
      Note: `${TapeText(Result.Fix, Label).replace(/, so the GRASP POINT moved.*$/, "")}, but ${Result.Why}: the GRASP PLAN stays as planned` };
  }
  const Moved = Result.Fix.ShiftCm !== 0;
  return { Reach: { ...Reach, Plan: Result.Plan, Text: GraspableText(Result.Plan, Reach.BaseDistanceCm) }, Fix: Result.Fix, Moved,
    Note: `${TapeText(Result.Fix, Label)}${Moved ? ` (now ${PlanPoint(Result.Plan.Grasp)})` : ""}` };
}

// Where an out-of-reach object is planned (CloserToGrasp): whole cm closer to the
// right arm's pan axis, straight toward it or turned about it by these, up to
// CloserMaxCm; at most CloserSpots spots judged and CloserPlans of them planned.
const CloserTurnsDeg = [0, 5, -5, 10, -10, 15, -15, 20, -20, 25, -25, 30, -30];
const CloserMaxCm = 20, CloserSpots = 30, CloserPlans = 4;
export type CloserSpot = { Cm: number; TurnDeg: number; At: XY; MoveCm: number };
/**
 * Where an object out of the right arm's reach would be planned, nearest move
 * first: whole cm closer to its pan axis (from the closest pose's miss up to
 * CloserMaxCm), straight toward it or turned about it up to 30 deg. Straight
 * toward it, a far-left can runs into the basket's far rim and the bottle
 * (2026-10-01 07:55: no advice; the user moved it 17 cm to the right, where it
 * planned). A spot where it would stand on or against a measured object is
 * skipped (the 11 cm straight spot put the can on the bottle), and each is
 * planned among the objects measured now. Computed, so the user is told a
 * spot that was checked; spots still out of reach cost no IK (BeyondArmReach).
 * Null when none was found.
 */
function CloserToGrasp(Object_: LocatedObject, Context: ReachContext, Others: LocatedObject[], MissCm: number, Kind: string | null): CloserSpot | null {
  const Pan = mirrorDisplayPoint({ x: 0, y: 0, z: 0 });
  const R = Math.hypot(Object_.x - Pan.x, Object_.y - Pan.y) || 1, Bearing = Math.atan2(Object_.y - Pan.y, Object_.x - Pan.x);
  const Spots: CloserSpot[] = [];
  for (let Cm = Math.max(1, Math.ceil(Number.isFinite(MissCm) ? MissCm : 1)); Cm <= CloserMaxCm && Cm < R - 15; Cm++) {
    for (const TurnDeg of CloserTurnsDeg) {
      const Angle = Bearing + TurnDeg * Math.PI / 180;
      const At = { x: Pan.x + (R - Cm) * Math.cos(Angle), y: Pan.y + (R - Cm) * Math.sin(Angle) };
      Spots.push({ Cm, TurnDeg, At, MoveCm: Math.hypot(At.x - Object_.x, At.y - Object_.y) });
    }
  }
  // Its measured box and each other object's apart by half a cm along X or Y.
  const Size = (Object_: LocatedObject) => Object_.sizeCm ?? [2, 2];
  const Free = (At: XY) => Others.every(Other => Math.abs(Other.x - At.x) >= (Size(Object_)[0] + Size(Other)[0]) / 2 + 0.5
    || Math.abs(Other.y - At.y) >= (Size(Object_)[1] + Size(Other)[1]) / 2 + 0.5);
  Spots.sort((A, B) => A.MoveCm - B.MoveCm || Math.abs(A.TurnDeg) - Math.abs(B.TurnDeg));
  let Planned = 0;
  for (const Spot of Spots.filter(Spot => Free(Spot.At)).slice(0, CloserSpots)) {
    const Reach = TaskObjectReachability({ ...Object_, x: Spot.At.x, y: Spot.At.y }, Context, { Others, Advise: false, Kind });
    if (Reach.Kind === "graspable") return Spot;
    // Refused as still out of the arm's reach beside it (the reach scan reached a point beside it, the planner's grasp
    // point did not): a spot farther in may plan; only the other refusals count toward CloserPlans.
    if (Reach.Kind === "ungraspable" && Reach.Refusal?.Code !== "reach" && ++Planned >= CloserPlans) break;
  }
  return null;
}
// What the user is told to do for a CloserSpot: straight toward the shoulder, or the move in the arm's directions.
// Label: the task's name for it (TaskObjectReachability).
function CloserAdvice(Object_: LocatedObject, Spot: CloserSpot, Label = Object_.label) {
  if (Spot.TurnDeg === 0) return `Move the ${Label} about ${Spot.Cm} cm closer to the right arm (straight toward its shoulder), then send the task again.`;
  const Dx = Spot.At.x - Object_.x, Dy = Spot.At.y - Object_.y;
  // The camera looks from behind the arm: the arm's right is the image's right (X negative).
  const Parts = [Math.abs(Dx) >= 0.5 ? `${Math.abs(Dx).toFixed(0)} cm to the arm's ${Dx < 0 ? "right" : "left"} (the camera image's ${Dx < 0 ? "right" : "left"})` : "",
    Math.abs(Dy) >= 0.5 ? `${Math.abs(Dy).toFixed(0)} cm ${Dy < 0 ? "nearer" : "farther from"} the arm` : ""].filter(Boolean);
  return `Move the ${Label} about ${Spot.MoveCm.toFixed(0)} cm: ${Parts.join(" and ")}, to about X ${Spot.At.x.toFixed(1)}, Y ${Spot.At.y.toFixed(1)}, then send the task again.`;
}

// --- The right arm's reach, for the system prompt ------------------------------

// The steepest wrist pitch against horizontal distance r from the right arm's
// pan axis, at the grasp heights of objects on the desk (Z -1.3 to 0.7), from
// the CAD arm with the slider limits, rounded down (TestAiTaskSafety checks
// them). The prompt used to give the arm's 48.7 cm length, which suggested a
// reach the arm has nowhere near the desk.
export const DeskReachCm: [number, number][] = [[-90, 27], [-75, 31.5], [-60, 35.5], [-45, 39], [-30, 41.5]];
export const DeskReachMaxCm = 42.5;

/** Where the right arm can reach, in the AI's frame. Floor: the task's; above or below the desk, the reach at its height (SurfaceReachText). */
export function ReachFactsText(Floor: AiFloor = DeskFloor) {
  const Pan = mirrorDisplayPoint({ x: 0, y: 0, z: 0 });
  const [[SteepPitch, SteepR], ...Rest] = DeskReachCm;
  return `The right arm's pan axis (its shoulder) is at X ${Deg(Pan.x)}, Y ${Deg(Pan.y)}, Z ${Deg(Pan.z)} in your frame.`
    + ` How steeply the wrist can point at desk height depends on the horizontal distance r from that axis: wrist ${SteepPitch} deg (straight down) reaches only r <= ${SteepR} cm, `
    + Rest.map(([Pitch, R]) => `${Pitch} deg r <= ${R} cm`).join(", ")
    + `, and nothing reaches beyond about ${DeskReachMaxCm} cm. The TASK OBJECT line gives the object's r, the wrist pitches the app verified there and, for a grasp, its GRASP PLAN.`
    + (Floor.Surface ? ` ${SurfaceReachText(Floor)}` : "")
    // 2026-10-01 07:55 the model offered "try with the left arm".
    + " Only this right arm is controlled: there is no other arm to use.";
}

// The reach table (ReachEnvelope.json, TestReachEnvelope.mjs: the app's IK with the basket and body), straight ahead.
const EnvelopeRows = ReachEnvelope.Ahead.Collision as unknown as Record<string, Record<string, [number, number][]>>;
/**
 * How far the right arm reaches at the heights a task on a surface besides the desk grasps at (its
 * surface to 12.5 cm above it): per wrist pitch, the radii it reaches at every row of the reach table
 * in that band ("nowhere" when no radius is reached at all of them).
 */
export function SurfaceReachText(Floor: AiFloor) {
  const From = Floor.SurfaceZ - 2.5, To = Floor.SurfaceZ + 12.5;
  const Rows = ReachEnvelope.ZCm.filter(Z => Z >= From && Z <= To);
  const Band = [-30, 0, -60].map(Pitch => {
    const Spans = Rows.map(Z => EnvelopeRows[String(Pitch)]?.[String(Z)] ?? []);
    const Low = Math.max(...Spans.map(Span => Span.length ? Span[0][0] : Infinity));
    const High = Math.min(...Spans.map(Span => Span.length ? Span[Span.length - 1][1] : -Infinity));
    return Rows.length && Low <= High ? `wrist ${Pitch} deg r ${Low}..${High} cm` : `wrist ${Pitch} deg nowhere`;
  });
  return `The task object stands on ${Floor.Surface ? SurfaceText(Floor.Surface) : "a surface"}, where the desk figures do not hold: from Z ${Rows[0] ?? Math.round(From)} to Z ${Rows[Rows.length - 1] ?? Math.round(To)} the arm reaches ${Band.join(", ")} (straight ahead of its pan axis).`;
}
