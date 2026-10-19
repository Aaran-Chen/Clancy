import { memo, useMemo, type CSSProperties } from "react";
import { SERVO_ORDER, currentAngles, forwardDisplayKinematics, type ServoName, type ServoState } from "../lib/robotModel";

/* The Joints panel's "B0T DIAGNOSTICS" strip (ManualHud.css): a side view of the arm drawn from its forward
   kinematics, the live pose solid and the commanded pose dashed, the selected joint lit, beside each servo's
   temperature as the bridge's motor guard reports it. Display only: nothing here sends a command. */
type BotDiagnosticsProps = {
  Servos: Record<ServoName, ServoState>;
  Temperatures: Partial<Record<ServoName, number>>;
  TemperatureLimitC: number;
  Status: "NOMINAL" | "TRIP" | "OFFLINE" | "PENDING";
  SelectedJoint: ServoName | null;
};
const Codes: Record<ServoName, string> = { shoulder_pan: "YAW", shoulder_lift: "SHLD", elbow_flex: "ELBW", wrist_flex: "WPCH", wrist_roll: "ROLL", gripper: "GRIP" };
const Names: Record<ServoName, string> = { shoulder_pan: "Shoulder pan", shoulder_lift: "Shoulder lift", elbow_flex: "Elbow flex", wrist_flex: "Wrist flex", wrist_roll: "Wrist roll", gripper: "Gripper" };
const Width = 150, Height = 96;

// A side view: distance out from the base (behind it negative) against height above it, in cm.
function SideView(Servos: Record<ServoName, ServoState>) {
  const Points = forwardDisplayKinematics(currentAngles(Servos)), Base = Points[0];
  return Points.map(Point => ({ H: Math.sign(Point.y - Base.y || 1) * Math.hypot(Point.x - Base.x, Point.y - Base.y), V: Point.z - Base.z }));
}

function BotDiagnostics({ Servos, Temperatures, TemperatureLimitC, Status, SelectedJoint }: BotDiagnosticsProps) {
  const Drawing = useMemo(() => {
    const Commanded = Object.fromEntries(SERVO_ORDER.map(Name => [Name, { ...Servos[Name], presentRaw: Servos[Name].targetRaw, guiAngleRaw: null }])) as Record<ServoName, ServoState>;
    const Live = SideView(Servos), Goal = SideView(Commanded), All = [...Live, ...Goal];
    const MinH = Math.min(-12, ...All.map(P => P.H)), MaxH = Math.max(26, ...All.map(P => P.H)), MinV = -6, MaxV = Math.max(22, ...All.map(P => P.V));
    const Scale = Math.min((Width - 16) / (MaxH - MinH), (Height - 12) / (MaxV - MinV));
    const Map = (P: { H: number; V: number }) => [8 + (P.H - MinH) * Scale, Height - 6 - (P.V - MinV) * Scale];
    const Line = (Points: { H: number; V: number }[]) => Points.map(P => Map(P).map(Value => Value.toFixed(1)).join(",")).join(" ");
    const [BaseX, BaseY] = Map({ H: 0, V: 0 }), FloorY = Map({ H: 0, V: MinV })[1];
    return { Live: Line(Live), Goal: Line(Goal), Nodes: Live.slice(0, SERVO_ORDER.length).map(Map), BaseX, BaseY, FloorY };
  }, [Servos]);
  return (
    <div className="bot-diagnostics" data-status={Status}>
      <div className="hud-subhead"><span>B0T DIAGNOSTICS</span><b>{Status}</b></div>
      <div className="bot-diagnostics-body">
        <svg className="bot-schematic" viewBox={`0 0 ${Width} ${Height}`} aria-hidden="true">
          <path className="bot-floor" d={`M2 ${Drawing.FloorY.toFixed(1)}H${Width - 2}`} />
          <path className="bot-base" d={`M${(Drawing.BaseX - 8).toFixed(1)} ${Drawing.FloorY.toFixed(1)}h16V${Drawing.BaseY.toFixed(1)}h-16z`} />
          <polyline className="bot-goal" points={Drawing.Goal} />
          <polyline className="bot-glow" points={Drawing.Live} />
          <polyline className="bot-arm" points={Drawing.Live} />
          {Drawing.Nodes.map(([X, Y], Index) => <circle key={SERVO_ORDER[Index]} className={SERVO_ORDER[Index] === SelectedJoint ? "selected" : undefined}
            cx={X.toFixed(1)} cy={Y.toFixed(1)} r={3} />)}
        </svg>
        <ul className="bot-temperatures" aria-label="Servo temperatures">
          {SERVO_ORDER.map((Name, Index) => {
            const Celsius = Temperatures[Name];
            const Heat = typeof Celsius === "number" ? Math.max(0, Math.min(1, Celsius / TemperatureLimitC)) : 0;
            return <li key={Name} className={Name === SelectedJoint ? "selected" : undefined} title={Names[Name]}
              data-heat={Heat >= .92 ? "hot" : Heat >= .75 ? "warm" : "ok"}>
              <b aria-hidden="true">J{Index + 1}</b><span>{Codes[Name]}</span>
              <i aria-hidden="true" style={{ "--heat": Heat } as CSSProperties} />
              <em>{typeof Celsius === "number" ? `${Celsius}°C` : "--°C"}</em>
            </li>;
          })}
        </ul>
      </div>
    </div>
  );
}

export default memo(BotDiagnostics);
