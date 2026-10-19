import { memo } from "react";

/* The Current Position panel's plan view in Manual mode (ManualHud.css): the tip seen from above on range rings
   around the workspace origin, the target as a diamond, and the tip's height on a ladder beside it. Decorative:
   the panel's numbers below it say the same, so it is hidden from assistive technology. It sends nothing. */
type Point = { x: number; y: number; z: number };
type TipRadarProps = { Tip: Point; Target: Point | null };
const RangeCm = 40, Radius = 46, Centre = 52, LowCm = -10, HighCm = 40;
const Clamp = (Value: number) => Math.max(-1, Math.min(1, Number.isFinite(Value) ? Value : 0));
const Plot = (Point: Point) => [Centre + Clamp(Point.x / RangeCm) * Radius, Centre - Clamp(Point.y / RangeCm) * Radius];
const Height = (Z: number) => (HighCm - Math.max(LowCm, Math.min(HighCm, Number.isFinite(Z) ? Z : 0))) / (HighCm - LowCm) * 100;

function TipRadar({ Tip, Target }: TipRadarProps) {
  const [TipX, TipY] = Plot(Tip);
  const [GoalX, GoalY] = Target ? Plot(Target) : [TipX, TipY];
  const Sweep = `M${Centre} ${Centre}L${(Centre + Radius * Math.SQRT1_2).toFixed(1)} ${(Centre - Radius * Math.SQRT1_2).toFixed(1)}A${Radius} ${Radius} 0 0 0 ${Centre} ${Centre - Radius}Z`;
  return (
    <div className="tip-radar" aria-hidden="true">
      <svg className="tip-radar-plot" viewBox="0 0 104 104">
        {[1, 2 / 3, 1 / 3].map(Ring => <circle key={Ring} cx={Centre} cy={Centre} r={Radius * Ring} />)}
        <path className="tip-radar-cross" d={`M${Centre} ${Centre - Radius}V${Centre + Radius}M${Centre - Radius} ${Centre}H${Centre + Radius}`} />
        <path className="tip-radar-sweep" d={Sweep} />
        <path className="tip-radar-reach" d={`M${Centre} ${Centre}L${TipX.toFixed(1)} ${TipY.toFixed(1)}`} />
        {Target && <rect className="tip-radar-goal" x={(GoalX - 3.5).toFixed(1)} y={(GoalY - 3.5).toFixed(1)} width={7} height={7}
          transform={`rotate(45 ${GoalX.toFixed(1)} ${GoalY.toFixed(1)})`} />}
        <circle className="tip-radar-tip" cx={TipX.toFixed(1)} cy={TipY.toFixed(1)} r={3} />
      </svg>
      <div className="tip-altitude">
        {Target && <i className="tip-altitude-goal" style={{ top: `${Height(Target.z).toFixed(1)}%` }} />}
        <i className="tip-altitude-now" style={{ top: `${Height(Tip.z).toFixed(1)}%` }} />
      </div>
    </div>
  );
}

export default memo(TipRadar, (Before, After) => JSON.stringify(Before) === JSON.stringify(After));
