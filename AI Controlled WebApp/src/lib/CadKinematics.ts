import { Matrix4, Vector3 } from "three";
import ModelSource from "../../../back end/CadModel.json";
import AlignmentSource from "../../../back end/CadAlignment.json";

// JSON imports use the same convention as robotModel.ts (relative to src/lib).
export const CadModel = ModelSource;
export const IkSolverVersion = "all-poses-v2";
export const CadJointNames = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"] as const;
const SolveNames = CadJointNames.slice(0, 4);
const Degrees = Math.PI / 180;
export type CadAngles = Record<string, number>;
export type CadPoint = { x: number; y: number; z: number };
export type CadAlignment = typeof AlignmentSource;
let Alignment: CadAlignment = structuredClone(AlignmentSource);
export function GetCadAlignment() { return Alignment; }
export function SetCadAlignment(Value: CadAlignment) {
  for (const Name of CadJointNames) {
    if (!Number.isFinite(Value.ReferenceRaw?.[Name]) || !Number.isFinite(Value.AnchorRaw?.[Name])
      || !Number.isFinite(Value.UrdfAtReferenceDegrees?.[Name]) || ![1, -1].includes(Value.Direction?.[Name])) return false;
  }
  Alignment = structuredClone(Value);
  return true;
}
export function CadUrdfAngles(Angles: CadAngles): CadAngles {
  return Object.fromEntries(CadJointNames.map(Name => [Name,
    (Alignment.UrdfAtReferenceDegrees[Name] + Alignment.Direction[Name] * (Angles[Name] ?? 0)) * Degrees]));
}
function FromUrdf(Radians: CadAngles): CadAngles {
  return Object.fromEntries(CadJointNames.map(Name => [Name,
    ((Radians[Name] ?? 0) / Degrees - Alignment.UrdfAtReferenceDegrees[Name]) / Alignment.Direction[Name]]));
}
const FromRows = (Rows: number[][]) => new Matrix4().fromArray(Rows.flat()).transpose();
const BaseTransform = FromRows(CadModel.CadToModel);
const JointOrder = [...CadJointNames, "gripper_frame_joint"].map(Name => CadModel.Joints.find(Joint => Joint.Name === Name)!);
const Origins = Object.fromEntries(JointOrder.map(Joint => [Joint.Name, FromRows(Joint.Origin)]));
const Position = (Transform: Matrix4) => new Vector3().setFromMatrixPosition(Transform);
const Point = (Value: Vector3): CadPoint => ({ x: Value.x, y: Value.y, z: Value.z });
function Wrap(Angle: number) { return ((Angle + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI; }

export function CadForwardUrdf(Radians: CadAngles) {
  const Links: Record<string, Matrix4> = { [CadModel.Root]: BaseTransform.clone() };
  const Frames: Record<string, Matrix4> = {};
  for (const Joint of JointOrder) {
    const Frame = Links[Joint.Parent].clone().multiply(Origins[Joint.Name]);
    Frames[Joint.Name] = Frame;
    const Angle = Radians[Joint.Name] ?? 0;
    if (!Number.isFinite(Angle)) throw new Error("CAD joint angles must be finite");
    Links[Joint.Child] = Joint.Type === "fixed" ? Frame.clone() : Frame.clone().multiply(new Matrix4().makeRotationZ(Angle));
  }
  const Tool = Position(Links[CadModel.ToolLink]);
  return { Links, Frames, Tool, Joints: [...CadJointNames.slice(0, 5).map(Name => Point(Position(Frames[Name]))), Point(Tool)] };
}
export function CadForward(Angles: CadAngles) { return CadForwardUrdf(CadUrdfAngles(Angles)); }
function PitchUrdf(Radians: CadAngles, State = CadForwardUrdf(Radians)) {
  const Elements = State.Links[CadModel.ToolLink].elements;
  const Pan = Radians.shoulder_pan ?? 0;
  return Math.atan2(Elements[10], Elements[8] * Math.sin(Pan) + Elements[9] * Math.cos(Pan));
}
export function CadPitch(Angles: CadAngles) { return PitchUrdf(CadUrdfAngles(Angles)) / Degrees; }
function Branch(Radians: CadAngles) {
  const Points = CadForwardUrdf(Radians).Joints;
  const Pan = Radians.shoulder_pan ?? 0;
  const Elevation = (Start: CadPoint, End: CadPoint) => Math.atan2(End.z - Start.z,
    (End.x - Start.x) * Math.sin(Pan) + (End.y - Start.y) * Math.cos(Pan));
  return Wrap(Elevation(Points[2], Points[3]) - Elevation(Points[1], Points[2])) >= 0 ? "up" : "down";
}

function AnalyticSeeds(Target: CadPoint, Pitch: number, Fixed: CadAngles) {
  const Zero = { ...Object.fromEntries(CadJointNames.map(Name => [Name, 0])), ...Fixed };
  const State = CadForwardUrdf(Zero);
  const Shoulder = Position(State.Frames.shoulder_lift);
  const UpperVector = Position(State.Frames.elbow_flex).sub(Shoulder);
  const ForearmVector = Position(State.Frames.wrist_flex).sub(Position(State.Frames.elbow_flex));
  const Distal = State.Tool.clone().sub(Position(State.Frames.wrist_flex));
  const Upper = Math.hypot(UpperVector.y, UpperVector.z);
  const Forearm = Math.hypot(ForearmVector.y, ForearmVector.z);
  const UpperZero = Math.atan2(UpperVector.z, UpperVector.y);
  const ForearmZero = Math.atan2(ForearmVector.z, ForearmVector.y);
  const ToolPitchZero = PitchUrdf(Zero, State);
  const DistalAngle = Pitch + Math.atan2(Distal.z, Distal.y) - ToolPitchZero;
  const DistalLength = Math.hypot(Distal.y, Distal.z);
  const Radius = Math.hypot(Target.x, Target.y);
  const Lateral = State.Tool.x;
  const Magnitude = Math.sqrt(Math.max(0, Radius * Radius - Lateral * Lateral));
  const Seeds: number[][] = [];
  for (const RadialSign of [1, -1]) {
    const Radial = RadialSign * Magnitude;
    const Pan = Math.atan2(Target.x, Target.y) - Math.atan2(Lateral, Radial);
    const R = Radial - Shoulder.y - DistalLength * Math.cos(DistalAngle);
    const Z = Target.z - Shoulder.z - DistalLength * Math.sin(DistalAngle);
    const CosElbow = Math.max(-1, Math.min(1, (R * R + Z * Z - Upper * Upper - Forearm * Forearm) / (2 * Upper * Forearm)));
    for (const Sign of [1, -1]) {
      const Bend = Sign * Math.acos(CosElbow);
      const Elevation = Math.atan2(Z, R) - Math.atan2(Forearm * Math.sin(Bend), Upper + Forearm * Math.cos(Bend));
      const Lift = UpperZero - Elevation;
      const Elbow = ForearmZero - UpperZero - Bend;
      Seeds.push([Pan, Lift, Elbow, ToolPitchZero - Pitch - Lift - Elbow]);
    }
  }
  return Seeds;
}
function LinearSolve(Matrix: number[][], Right: number[]) {
  const Rows = Matrix.map((Row, Index) => [...Row, Right[Index]]);
  for (let Column = 0; Column < 4; Column++) {
    let Pivot = Column;
    for (let Row = Column + 1; Row < 4; Row++) if (Math.abs(Rows[Row][Column]) > Math.abs(Rows[Pivot][Column])) Pivot = Row;
    [Rows[Pivot], Rows[Column]] = [Rows[Column], Rows[Pivot]];
    if (Math.abs(Rows[Column][Column]) < 1e-14) return null;
    const Scale = Rows[Column][Column];
    for (let Index = Column; Index < 5; Index++) Rows[Column][Index] /= Scale;
    for (let Row = 0; Row < 4; Row++) if (Row !== Column) {
      const Factor = Rows[Row][Column];
      for (let Index = Column; Index < 5; Index++) Rows[Row][Index] -= Factor * Rows[Column][Index];
    }
  }
  return Rows.map(Row => Row[4]);
}

export function CadSolveCandidates(Target: CadPoint, PitchDegrees: number, Elbow: "up" | "down", InitialAngles: CadAngles = {}, Limits: Partial<Record<string, [number, number]>> = {}, FreePitch = false) {
  const Initial = CadUrdfAngles(InitialAngles);
  const Fixed = { wrist_roll: Initial.wrist_roll, gripper: Initial.gripper };
  const PitchTarget = PitchDegrees * Degrees;
  if (![Target.x, Target.y, Target.z, PitchTarget].every(Number.isFinite)) throw new Error("IK target and pitch must be finite");
  const Bounds = SolveNames.map(Name => (Limits[Name] ?? [-180, 180]).map(Value =>
    (Alignment.UrdfAtReferenceDegrees[Name] + Alignment.Direction[Name] * Value) * Degrees).sort((A, B) => A - B));
  const Clamp = (Values: number[]) => Values.map((Value, Index) => Math.max(Bounds[Index][0], Math.min(Bounds[Index][1], Value)));
  const RadiansFor = (Values: number[]): CadAngles => ({ ...Fixed, ...Object.fromEntries(SolveNames.map((Name, Index) => [Name, Values[Index]])) });
  const Evaluate = (Values: number[]) => {
    const Radians = RadiansFor(Values);
    const State = CadForwardUrdf(Radians);
    return [State.Tool.x - Target.x, State.Tool.y - Target.y, State.Tool.z - Target.z, FreePitch ? 0 : Wrap(PitchUrdf(Radians, State) - PitchTarget) * 10];
  };
  const Norm = (Values: number[]) => Values.reduce((Sum, Value) => Sum + Value * Value, 0);
  const Seeds = [...AnalyticSeeds(Target, PitchTarget, Fixed), SolveNames.map(Name => Initial[Name])];
  if (FreePitch) for (const Pitch of [-90, 0, 90, 180]) Seeds.push(...AnalyticSeeds(Target, Pitch * Degrees, Fixed));
  const Candidates = Seeds.map(Seed => {
    let Values = Clamp(Seed.map((Value, Index) => {
      const Options = Array.from({ length: 7 }, (_, Turn) => Value + (Turn - 3) * Math.PI * 2);
      const Inside = Options.filter(Option => Option >= Bounds[Index][0] && Option <= Bounds[Index][1]);
      return (Inside.length ? Inside : Options).sort((A, B) => Math.abs(A - Initial[SolveNames[Index]]) - Math.abs(B - Initial[SolveNames[Index]]))[0];
    }));
    let Residual = Evaluate(Values);
    let Damping = 1e-4;
    for (let Iteration = 0; Iteration < 35 && Norm(Residual) > 1e-12; Iteration++) {
      const Columns = Values.map((_, Index) => {
        const Plus = [...Values], Minus = [...Values];
        Plus[Index] += 1e-5; Minus[Index] -= 1e-5;
        const High = Evaluate(Plus), Low = Evaluate(Minus);
        return High.map((Value, Row) => (Row === 3 ? Wrap((Value - Low[Row]) / 10) * 10 : Value - Low[Row]) / 2e-5);
      });
      const Matrix = Columns.map((Column, Row) => Columns.map((Other, Index) =>
        Column.reduce((Sum, Value, K) => Sum + Value * Other[K], 0) + (Row === Index ? Damping : 0)));
      const Right = Columns.map(Column => -Column.reduce((Sum, Value, Index) => Sum + Value * Residual[Index], 0));
      const Delta = LinearSolve(Matrix, Right);
      if (!Delta) break;
      let Accepted = false;
      for (const Fraction of [1, 0.5, 0.25, 0.1]) {
        const Trial = Clamp(Values.map((Value, Index) => Value + Math.max(-0.5, Math.min(0.5, Delta[Index])) * Fraction));
        const Next = Evaluate(Trial);
        if (Norm(Next) < Norm(Residual) - 1e-15) {
          Values = Trial; Residual = Next; Damping = Math.max(1e-8, Damping * 0.3); Accepted = true; break;
        }
      }
      if (!Accepted) { Damping *= 10; if (Damping > 1e6) break; }
    }
    const Radians = RadiansFor(Values);
    const ErrorCm = Math.hypot(...Residual.slice(0, 3));
    const PitchError = Math.abs(Residual[3] / 10 / Degrees);
    const Reachable = ErrorCm <= 0.1 && PitchError <= 0.25;
    const Converged = ErrorCm <= 1e-5 && PitchError <= 1e-5;
    const ActualBranch = Branch(Radians);
    const Travel = SolveNames.reduce((Sum, Name) => Sum + (Radians[Name] - Initial[Name]) ** 2, 0);
    return { Angles: FromUrdf(Radians), ErrorCm, PitchError, Reachable, Branch: ActualBranch,
      PitchDegrees: PitchUrdf(Radians) / Degrees,
      Score: [Converged ? 0 : Reachable ? 1 : 2, Reachable && ActualBranch === Elbow ? 0 : 1, Reachable ? Travel : ErrorCm + PitchError * 0.1] };
  });
  Candidates.sort((A, B) => A.Score[0] - B.Score[0] || A.Score[1] - B.Score[1] || A.Score[2] - B.Score[2]);
  return Candidates;
}

export function CadSolve(Target: CadPoint, PitchDegrees: number, Elbow: "up" | "down", InitialAngles: CadAngles = {},
  Limits: Partial<Record<string, [number, number]>> = {}, FreePitch = false, PoseAllowed?: (Angles: CadAngles) => boolean) {
  const Candidates = CadSolveCandidates(Target, PitchDegrees, Elbow, InitialAngles, Limits, FreePitch);
  return (PoseAllowed ? Candidates.find(Candidate => Candidate.Reachable && PoseAllowed(Candidate.Angles)) : undefined) ?? Candidates[0];
}
