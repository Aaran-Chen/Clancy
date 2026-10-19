import { useId, type CSSProperties } from "react";
import "./MotionThrottle.css";

type MotionThrottleProps = {
  Percent: number;
  DisabledReason: string | null;
  OnChange: (Percent: number) => void;
};

export default function MotionThrottle({ Percent, DisabledReason, OnChange }: MotionThrottleProps) {
  const Id = useId();
  return (
    <div className="motion-throttle" data-enabled={!DisabledReason}>
      <div className="throttle-heading">
        <label htmlFor={Id}>Motion speed <span>THROTTLE</span></label>
        <output htmlFor={Id} aria-label="Selected motion speed">
          {DisabledReason ? "---" : String(Percent).padStart(3, "0")}<small>%</small>
        </output>
      </div>
      <div className="throttle-fader" style={{ "--throttle-fill": `${(Percent - 10) / 90 * 100}%` } as CSSProperties}>
        <input id={Id} type="range" min={10} max={100} step={1} value={Percent}
          aria-label="Motion speed" aria-valuetext={`${Percent}% of maximum motion speed`}
          title="Adjust during a move, Home or calibration; changes apply immediately."
          aria-describedby={`${Id}-help`} disabled={Boolean(DisabledReason)}
          onChange={Event => OnChange(Number(Event.currentTarget.value))} />
        <div className="throttle-scale" aria-hidden="true"><span>10</span><span>25</span><span>50</span><span>75</span><span>100</span></div>
      </div>
      <p id={`${Id}-help`} className="throttle-help">
        <span className="throttle-lamp" aria-hidden="true" />
        {DisabledReason ?? "Moves, Home & calibration"}
      </p>
    </div>
  );
}
