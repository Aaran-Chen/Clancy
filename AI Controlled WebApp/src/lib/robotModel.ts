import SavedLimits from "../../../back end/gui_limits.json";
import { CadForward, CadPitch, CadSolve, CadModel, GetCadAlignment, type CadAngles } from "./CadKinematics";
import SavedHome from "../../../back end/home_preset.json";
import WorkspaceFrame from "../../../back end/WorkspaceFrame.json";
import { BasketPoseReason, BasketTargetReason } from "./BasketCollision";
export type ServoName =
  | "shoulder_pan"
  | "shoulder_lift"
  | "elbow_flex"
  | "wrist_flex"
  | "wrist_roll"
  | "gripper";

export type IkServoName = Exclude<ServoName, "wrist_roll" | "gripper">;
export type ArmAngles = Record<IkServoName, number> & Partial<Record<"wrist_roll" | "gripper", number>>;

export type Point3 = {
  x: number;
  y: number;
  z: number;
};

export type TargetPose = Point3 & {
  wristPitch: number;
};

export type IkPreview = {
  angles: ArmAngles;
  joints: Point3[];
  reachable: boolean;
  errorCm: number;
  requestedWristPitch: number;
  effectiveWristPitch: number;
  wristPitchReachable: boolean;
  // True only when the joint limits are what blocks the pose (see Explain in solveIkPreview).
  jointLimitsApplied: boolean;
  collisionReason?: string | null;
  failure?: IkFailure | null;
};

// Why the joint-limited IK cannot reach a pose, from one more solve without the limits.
export type IkFailure =
  // Out of the arm's reach at this wrist pitch even without joint limits:
  // farther from the shoulder than the stretched arm, or closer than the folded one.
  | { Kind: "reach"; ErrorCm: number; TooFar: boolean }
  // Reachable without the limits: these joints would leave their ranges.
  | { Kind: "joint-limit"; Joints: { Name: IkServoName; Angle: number; Range: [number, number] }[] }
  // Reachable without the limits, but only in a pose that contacts the basket or the body.
  | { Kind: "collision"; Reason: string }
  | { Kind: "unexplained" };

export type ServoState = {
  name: ServoName;
  presentRaw: number;
  targetRaw: number;
  guiAngleRaw?: number | null;
  min: number;
  max: number;
  arcInverted: boolean;
};

export const SERVO_ORDER: ServoName[] = [
  "shoulder_pan",
  "shoulder_lift",
  "elbow_flex",
  "wrist_flex",
  "wrist_roll",
  "gripper",
];

export const IK_JOINTS: IkServoName[] = [
  "shoulder_pan",
  "shoulder_lift",
  "elbow_flex",
  "wrist_flex",
];

// Home is a saved motor pose. CadAlignment holds the independent geometric reference.
export const HOME_RAW: Record<ServoName, number> = { ...SavedHome };

export const SERVO_SIGNS: Record<IkServoName, number> = {
  shoulder_pan: -1,
  shoulder_lift: 1,
  elbow_flex: -1,
  wrist_flex: -1,
};

export const DISPLAY_SIGNS: Record<IkServoName, number> = {
  shoulder_pan: 1,
  shoulder_lift: 1,
  elbow_flex: 1,
  wrist_flex: 1,
};

export const INITIAL_SERVOS = Object.fromEntries(SERVO_ORDER.map(Name => {
  const Limits = SavedLimits.motor_raw_limits[Name];
  return [Name, { name: Name, presentRaw: HOME_RAW[Name], targetRaw: HOME_RAW[Name],
    min: Limits.min, max: Limits.max, arcInverted: Limits.arc_inverted }];
})) as Record<ServoName, ServoState>;
const RESOLUTION = 4096;
const DEG = Math.PI / 180;
export const MAX_TARGET_REACH_CM = CadModel.Joints.filter(Joint => Joint.Name !== "shoulder_pan" && Joint.Name !== "gripper")
  .reduce((Sum, Joint) => Sum + Math.hypot(...Joint.Origin.slice(0, 3).map(Row => Row[3])), 0);

export function formatName(name: string) {
  return name.replace(/_/g, " ");
}

export function clampRaw(raw: number) {
  const rounded = Math.round(raw) % RESOLUTION;
  return rounded < 0 ? rounded + RESOLUTION : rounded;
}

export function arcSpan(servo: ServoState) {
  if (servo.arcInverted) {
    return (servo.min - servo.max + RESOLUTION) % RESOLUTION;
  }
  return (servo.max - servo.min + RESOLUTION) % RESOLUTION;
}

export function arcDistanceFromLeft(servo: ServoState, raw: number) {
  const r = clampRaw(raw);
  if (servo.arcInverted) {
    return (servo.min - r + RESOLUTION) % RESOLUTION;
  }
  return (r - servo.min + RESOLUTION) % RESOLUTION;
}

export function rawToFraction(servo: ServoState, raw: number) {
  const span = arcSpan(servo);
  return span === 0 ? 0 : arcDistanceFromLeft(servo, raw) / span;
}

export function fractionToRaw(servo: ServoState, fraction: number) {
  const span = arcSpan(servo);
  const step = Math.round(Math.max(0, Math.min(1, fraction)) * span);
  const raw = servo.arcInverted ? servo.min - step : servo.min + step;
  return clampRaw(raw);
}

export function currentRawForAngle(servo: ServoState) {
  return servo.guiAngleRaw ?? servo.presentRaw;
}

export function angleFromRaw(servo: ServoState) {
  return angleFromRawValue(servo, currentRawForAngle(servo));
}

export function angleFromRawValue(servo: ServoState, raw: number) {
  const joint = servo.name;
  const span = arcSpan(servo);
  if (span === 0) return 0;
  const zeroOffset = arcDistanceFromLeft(servo, GetCadAlignment().ReferenceRaw[joint]);
  let offset = rawToFraction(servo, raw) * span;
  if (offset > span) {
    const LeftDistance = Math.abs((clampRaw(raw) - servo.min + RESOLUTION * 1.5) % RESOLUTION - RESOLUTION / 2);
    const RightDistance = Math.abs((clampRaw(raw) - servo.max + RESOLUTION * 1.5) % RESOLUTION - RESOLUTION / 2);
    offset = LeftDistance <= RightDistance ? -LeftDistance : span + RightDistance;
  }
  return (SERVO_SIGNS[joint as IkServoName] ?? 1) * (offset - zeroOffset) * 360 / RESOLUTION;
}

export function rawFromAngle(servo: ServoState, angleDeg: number) {
  const joint = servo.name;
  const span = arcSpan(servo);
  if (span === 0) return servo.targetRaw;
  const zeroOffset = arcDistanceFromLeft(servo, GetCadAlignment().ReferenceRaw[joint]);
  const offset = zeroOffset + angleDeg * RESOLUTION / (360 * (SERVO_SIGNS[joint as IkServoName] ?? 1));
  return fractionToRaw(servo, offset / span);
}

export function trackAngleFromRaw(servo: ServoState, raw: number) {
  const span = arcSpan(servo);
  if (span === 0) return 0;
  const offset = rawToFraction(servo, raw) * span;
  return (offset - span / 2) * 360 / RESOLUTION;
}

export function rawFromTrackAngle(servo: ServoState, angleDeg: number) {
  const span = arcSpan(servo);
  if (span === 0) return servo.targetRaw;
  const offset = angleDeg * RESOLUTION / 360 + span / 2;
  return fractionToRaw(servo, offset / span);
}

export function currentAngles(servos: Record<ServoName, ServoState>): ArmAngles {
  return Object.fromEntries(SERVO_ORDER.map(Name => [Name, angleFromRaw(servos[Name])])) as ArmAngles;
}

export function ReuseServoSnapshot(Previous: Record<ServoName, ServoState>, Next: Record<ServoName, ServoState>) {
  return SERVO_ORDER.every(Name => (Object.keys(Next[Name]) as (keyof ServoState)[])
    .every(Field => Previous[Name][Field] === Next[Name][Field])) ? Previous : Next;
}
export function displayAngles(Angles: ArmAngles): ArmAngles { return { ...Angles }; }
export function forwardKinematics(Angles: ArmAngles): Point3[] { return CadForward(Angles).Joints; }

function mirrorXPoint(point: Point3): Point3 {
  return { ...point, x: -point.x };
}

const ZERO_IK_ANGLES: Record<IkServoName, number> = {
  shoulder_pan: 0,
  shoulder_lift: 0,
  elbow_flex: 0,
  wrist_flex: 0,
};

// Shared workspace reference. The app negates internal display X for its
// user-facing coordinates; Y and Z keep the same direction.
export const HOME_DISPLAY_POSITION_CM: Point3 = WorkspaceFrame.HomeDisplayPositionCm;
export const FixedAnchorVisualCm = WorkspaceFrame.FixedAnchorVisualCm;
export const RobotFrame = WorkspaceFrame.RobotFrame;

export const DISPLAY_HOME_OFFSET_CM: Point3 = { x: 0, y: 0, z: 0 };
export function RefreshCadFrame() {
  if (RobotFrame) {
    const Pan = CadForward({}).Joints[0], Base = RobotFrame.ArmBasePositionCm;
    Object.assign(DISPLAY_HOME_OFFSET_CM, { x: Pan.x - Base.x, y: Base.y - Pan.y, z: Base.z - Pan.z });
    return;
  }
  const Reference = GetCadAlignment();
  const AnchorAngles = Object.fromEntries(SERVO_ORDER.map(Name => [Name, angleFromRawValue(INITIAL_SERVOS[Name], Reference.AnchorRaw[Name])])) as ArmAngles;
  const Anchor = forwardKinematics(AnchorAngles).slice(-1)[0];
  Object.assign(DISPLAY_HOME_OFFSET_CM, { x: HOME_DISPLAY_POSITION_CM.x + Anchor.x,
    y: HOME_DISPLAY_POSITION_CM.y - Anchor.y, z: HOME_DISPLAY_POSITION_CM.z - Anchor.z });
}
RefreshCadFrame();

export function applyDisplayHomeOffset(point: Point3): Point3 {
  return {
    x: point.x + DISPLAY_HOME_OFFSET_CM.x,
    y: point.y + DISPLAY_HOME_OFFSET_CM.y,
    z: point.z + DISPLAY_HOME_OFFSET_CM.z,
  };
}

export function applyDisplayHomeOffsetToJoints(joints: Point3[]): Point3[] {
  return joints.map(applyDisplayHomeOffset);
}

function removeDisplayHomeOffset(point: Point3): Point3 {
  return {
    x: point.x - DISPLAY_HOME_OFFSET_CM.x,
    y: point.y - DISPLAY_HOME_OFFSET_CM.y,
    z: point.z - DISPLAY_HOME_OFFSET_CM.z,
  };
}

export function mirrorDisplayPoint(point: Point3): Point3 {
  return applyDisplayHomeOffset(mirrorXPoint(point));
}

export function modelPointFromDisplayPoint(point: Point3): Point3 {
  return mirrorXPoint(removeDisplayHomeOffset(point));
}

export const HOME_MODEL_TARGET_CM: TargetPose = {
  ...forwardKinematics(currentAngles(INITIAL_SERVOS)).slice(-1)[0],
  wristPitch: CadPitch(currentAngles(INITIAL_SERVOS)),
};

export function forwardDisplayKinematics(angles: ArmAngles): Point3[] {
  return mirrorDisplayJoints(forwardKinematics(displayAngles(angles)));
}

export function mirrorDisplayJoints(joints: Point3[]): Point3[] {
  return joints.map(mirrorDisplayPoint);
}

// Stage the requested pose unchanged. Exact CAD IK explicitly reports targets
// outside its workspace instead of shifting the position or pitch silently.
export function clampTargetToMaxReach<T extends Point3>(Target: T): T { return Target; }
export function isWristPitchReachable(Target: Point3, Pitch: number) { return CadSolve(Target, Pitch, "up").Reachable; }
export function clampWristPitchToReach(_Target: Point3, Requested: number, _Reference = Requested) { return Requested; }
export function constrainTargetPose(Target: TargetPose, _Previous?: TargetPose): TargetPose { return Target; }
export type IkAngleLimits = Partial<Record<IkServoName, [number, number]>>;
export function GetIkAngleLimits(Servos?: Record<ServoName, ServoState>, AngleLimits?: IkAngleLimits | null): IkAngleLimits {
  return AngleLimits ?? (AngleLimits === null || !Servos ? {} : Object.fromEntries(IK_JOINTS.map(Name => {
    const Values = [angleFromRawValue(Servos[Name], Servos[Name].min), angleFromRawValue(Servos[Name], Servos[Name].max)].sort((A, B) => A - B);
    return [Name, Values];
  })) as IkAngleLimits);
}
// Solves so far: a count of work (GraspPlan's best-effort pass budgets its search by its work, never by the clock).
export const IkWork = { Solves: 0 };
// Explain: for a pose the IK cannot reach, say why (IkFailure). It costs one
// more solve without joint limits, so the searches that try hundreds of
// pitches leave it off and get failure null and jointLimitsApplied false.
export function solveIkPreview(Target: TargetPose, ElbowMode: "up" | "down", Servos?: Record<ServoName, ServoState>, AngleLimits?: IkAngleLimits | null,
  Explain = false): IkPreview {
  IkWork.Solves += 1;
  const Initial = currentAngles(Servos ?? INITIAL_SERVOS);
  const Limits = GetIkAngleLimits(Servos, AngleLimits);
  const PoseAllowed = (Angles: CadAngles) => BasketPoseReason(Angles) === null;
  const Result = CadSolve(Target, Target.wristPitch, ElbowMode, Initial, Limits, false, PoseAllowed);
  const Angles = Result.Angles as ArmAngles;
  const CollisionReason = BasketTargetReason(Target) ?? (Result.Reachable ? BasketPoseReason(Angles as CadAngles) : null);
  const Failure = Explain && !Result.Reachable ? ExplainIkFailure(Target, ElbowMode, Initial, Limits, PoseAllowed) : null;
  return { angles: Angles, joints: mirrorDisplayJoints(forwardKinematics(Angles)), reachable: Result.Reachable && !CollisionReason,
    collisionReason: CollisionReason,
    errorCm: Result.ErrorCm, requestedWristPitch: Target.wristPitch, effectiveWristPitch: Result.PitchDegrees,
    // Every unreachable pose used to count as a limit hit, at 16:05 with each joint far from its stop.
    wristPitchReachable: Result.PitchError <= 0.25, jointLimitsApplied: Failure?.Kind === "joint-limit", failure: Failure };
}
function ExplainIkFailure(Target: TargetPose, ElbowMode: "up" | "down", Initial: ArmAngles, Limits: IkAngleLimits,
  PoseAllowed: (Angles: CadAngles) => boolean): IkFailure {
  const Free = CadSolve(Target, Target.wristPitch, ElbowMode, Initial, {}, false, PoseAllowed);
  const Angles = Free.Angles as ArmAngles;
  if (!Free.Reachable) {
    const Points = forwardKinematics(Angles);
    const Shoulder = Points[1];
    const FromShoulder = (Point: Point3) => Math.hypot(Point.x - Shoulder.x, Point.y - Shoulder.y, Point.z - Shoulder.z);
    return { Kind: "reach", ErrorCm: Free.ErrorCm, TooFar: FromShoulder(Target) > FromShoulder(Points[Points.length - 1]) };
  }
  const Collision = BasketPoseReason(Angles as CadAngles);
  if (Collision) return { Kind: "collision", Reason: Collision };
  const Joints = IK_JOINTS.flatMap(Name => {
    const Range = Limits[Name];
    return Range && (Angles[Name] < Range[0] || Angles[Name] > Range[1]) ? [{ Name, Angle: Angles[Name], Range }] : [];
  });
  return Joints.length ? { Kind: "joint-limit", Joints } : { Kind: "unexplained" };
}
export function moveToward(Current: number, Target: number, Step: number) {
  return Math.abs(Target - Current) <= Step ? Target : Current + Math.sign(Target - Current) * Step;
}
export function wristPitchFromAngles(Angles: ArmAngles) { return CadPitch(Angles); }
export function closestReachableWristPitch(Target: Point3, Requested: number, _Reference = Requested) {
  return { pitch: Requested, requestedReachable: isWristPitchReachable(Target, Requested) };
}
