import { useEffect, useMemo, useState, type CSSProperties, type MouseEvent } from "react";
import {
  ServoName,
  ServoState,
  SERVO_ORDER,
  arcSpan,
  currentRawForAngle,
  formatName,
  fractionToRaw,
  rawFromTrackAngle,
  rawToFraction,
  trackAngleFromRaw,
} from "../lib/robotModel";

import JointDial from "./JointDial";

type ServoPanelProps = {
  SelectedJoint: ServoName | null;
  OnSelectJoint: (Name: ServoName | null) => void;
  servos: Record<ServoName, ServoState>;
  onChange: (name: ServoName, raw: number) => void;
  onGuiAngleChange: (name: ServoName, raw: number | null) => void;
  onLimitsChange: (
    name: ServoName,
    limits: { min: number; max: number; arcInverted: boolean },
  ) => void;
  /** Each servo's temperature as the bridge's motor guard reports it, for the row's telemetry line. */
  Temperatures?: Partial<Record<ServoName, number>>;
};

type SliderMenu = {
  name: ServoName;
  x: number;
  y: number;
  angleText: string;
  leftText: string;
  rightText: string;
};

function cleanRaw(value: string, fallback: number) {
  const parsed = Number(value);
  if (!Number.isFinite(parsed)) return fallback;
  return Math.max(0, Math.min(4095, Math.round(parsed)));
}

const VISUAL_REVERSED_JOINTS = new Set<ServoName>([
  "shoulder_pan",
  "elbow_flex",
  "wrist_flex",
]);

function isVisualReversed(name: ServoName) {
  return VISUAL_REVERSED_JOINTS.has(name);
}

function displayFraction(name: ServoName, fraction: number) {
  return isVisualReversed(name) ? 1 - fraction : fraction;
}

function rawFractionFromDisplay(name: ServoName, fraction: number) {
  return isVisualReversed(name) ? 1 - fraction : fraction;
}

function displayLeftRaw(name: ServoName, servo: ServoState) {
  return isVisualReversed(name) ? servo.max : servo.min;
}

function displayRightRaw(name: ServoName, servo: ServoState) {
  return isVisualReversed(name) ? servo.min : servo.max;
}

function displayAngleFromRaw(name: ServoName, servo: ServoState, raw: number) {
  const angle = trackAngleFromRaw(servo, raw);
  return isVisualReversed(name) ? -angle : angle;
}

function rawFromDisplayAngle(name: ServoName, servo: ServoState, angleDeg: number) {
  const angle = isVisualReversed(name) ? -angleDeg : angleDeg;
  return rawFromTrackAngle(servo, angle);
}

// Target minus present in encoder steps, the short way round the 4096-step circle.
function RawOffset(target: number, present: number) {
  return ((Math.round(target - present) % 4096) + 6144) % 4096 - 2048;
}

function formatAngle(angleDeg: number) {
  return Number(angleDeg.toFixed(1)).toString();
}

function ServoPanel({ servos, onChange, onGuiAngleChange, onLimitsChange, SelectedJoint, OnSelectJoint, Temperatures = {} }: ServoPanelProps) {
  const [menu, setMenu] = useState<SliderMenu | null>(null);
  const menuServo = useMemo(() => (menu ? servos[menu.name] : null), [menu, servos]);

  useEffect(() => {
    if (!menu) return;
    const close = () => setMenu(null);
    const onKeyDown = (event: KeyboardEvent) => {
      if (event.key === "Escape") close();
    };
    window.addEventListener("pointerdown", close);
    window.addEventListener("keydown", onKeyDown);
    return () => {
      window.removeEventListener("pointerdown", close);
      window.removeEventListener("keydown", onKeyDown);
    };
  }, [menu]);

  const openMenu = (event: MouseEvent, name: ServoName) => {
    event.preventDefault();
    event.stopPropagation();
    const servo = servos[name];
    setMenu({
      name,
      x: Math.min(event.clientX, window.innerWidth - 260),
      y: Math.min(event.clientY, window.innerHeight - 370),
      angleText: formatAngle(displayAngleFromRaw(name, servo, currentRawForAngle(servo))),
      leftText: String(displayLeftRaw(name, servo)),
      rightText: String(displayRightRaw(name, servo)),
    });
  };

  const applyLimits = (patch: Partial<{ min: number; max: number; arcInverted: boolean }>) => {
    if (!menuServo || !menu) return;
    const next = {
      min: patch.min ?? menuServo.min,
      max: patch.max ?? menuServo.max,
      arcInverted: patch.arcInverted ?? menuServo.arcInverted,
    };
    onLimitsChange(menu.name, next);
  };

  const saveMenuInputs = () => {
    if (!menuServo || !menu) return;
    const left = cleanRaw(menu.leftText, displayLeftRaw(menu.name, menuServo));
    const right = cleanRaw(menu.rightText, displayRightRaw(menu.name, menuServo));
    if (isVisualReversed(menu.name)) {
      applyLimits({ min: right, max: left });
    } else {
      applyLimits({ min: left, max: right });
    }
  };

  const saveAngleInput = () => {
    if (!menuServo || !menu) return;
    const parsed = Number(menu.angleText);
    const currentRaw = currentRawForAngle(menuServo);
    if (!Number.isFinite(parsed)) {
      setMenu({
        ...menu,
        angleText: formatAngle(displayAngleFromRaw(menu.name, menuServo, currentRaw)),
      });
      return;
    }
    const raw = rawFromDisplayAngle(menu.name, menuServo, parsed);
    onGuiAngleChange(menu.name, raw);
    setMenu({
      ...menu,
      angleText: formatAngle(displayAngleFromRaw(menu.name, menuServo, raw)),
    });
  };

  return (
    <div className="servo-list">
      {SERVO_ORDER.map((name, index) => {
        const servo = servos[name];
        const currentRaw = currentRawForAngle(servo);
        const targetFraction = displayFraction(name, rawToFraction(servo, servo.targetRaw));
        const presentFraction = displayFraction(name, rawToFraction(servo, currentRaw));
        const span = arcSpan(servo);
        const leftRaw = displayLeftRaw(name, servo);
        const rightRaw = displayRightRaw(name, servo);
        const celsius = Temperatures[name];
        // The heads-up tape (ManualHud.css): a fill to the present position under the target thumb.
        const tape = { "--present": Math.max(0, Math.min(1, presentFraction)), "--target": Math.max(0, Math.min(1, targetFraction)) } as CSSProperties;

        return (
          <div
            className={`servo-row ${SelectedJoint === name ? "joint-selected" : ""}`}
            data-joint={name}
            onClick={() => OnSelectJoint(name)}
            key={name}
            onContextMenu={(event) => openMenu(event, name)}
          >
            <div className="servo-header">
              <i className="servo-index" aria-hidden="true">J{index + 1}</i>
              <button type="button" className="joint-select-button" aria-pressed={SelectedJoint === name} onClick={() => OnSelectJoint(name)}><strong>{formatName(name)}</strong></button>
              <b className="servo-angle" title="Current angle">{formatAngle(displayAngleFromRaw(name, servo, currentRaw))}°</b>
              <span title={servo.guiAngleRaw == null ? "Live raw" : "GUI current angle override"}>
                {Math.round(currentRaw)}{servo.guiAngleRaw == null ? "" : "*"}
              </span>
            </div>
            <div className="slider-stack" style={tape} onContextMenu={(event) => openMenu(event, name)}>
              <div
                className="target-label"
                // Centred over the middle of the travel, flush with the track at its ends,
                // so the tag never leaves the joint's row.
                style={{ left: `${targetFraction * 100}%`, transform: `translateX(${-Math.max(0, Math.min(1, targetFraction)) * 100}%)` }}
              >
                Target
              </div>
              <div
                className="present-marker"
                style={{ left: `${presentFraction * 100}%` }}
              />
              <input
                aria-label={`${formatName(name)} target`}
                type="range"
                onKeyDown={() => OnSelectJoint(name)}
                min="0"
                max="1000"
                value={Math.round(targetFraction * 1000)}
                onChange={(event) => {
                  const fraction = rawFractionFromDisplay(name, Number(event.target.value) / 1000);
                  const raw = fractionToRaw(servo, fraction);
                  onChange(name, raw);
                }}
              />
            </div>
            {SelectedJoint === name && <JointDial Servo={servo} OnChange={Raw => onChange(name, Raw)} />}
            <div className="servo-meta" title={`${span} raw span`}>
              <span>{leftRaw}</span>
              <span className="servo-telemetry">
                {`TGT ${Math.round(servo.targetRaw)}`}{typeof celsius === "number" ? ` · ${celsius} °C` : ""}{` · Δ${RawOffset(servo.targetRaw, currentRaw)}`}
              </span>
              <span>{rightRaw}</span>
            </div>
          </div>
        );
      })}
      {menu && menuServo && (
        <div
          className="slider-context-menu"
          style={{ left: menu.x, top: menu.y }}
          onPointerDown={(event) => event.stopPropagation()}
          onContextMenu={(event) => event.preventDefault()}
        >
          <div className="slider-context-title">{formatName(menu.name)}</div>
          <label>
            <span>Current angle</span>
            <input
              value={menu.angleText}
              inputMode="decimal"
              onChange={(event) => setMenu({ ...menu, angleText: event.target.value })}
              onBlur={saveAngleInput}
              onKeyDown={(event) => {
                if (event.key === "Enter") saveAngleInput();
              }}
            />
          </label>
          <label>
            <span>Left raw</span>
            <input
              value={menu.leftText}
              inputMode="numeric"
              onChange={(event) => setMenu({ ...menu, leftText: event.target.value })}
              onBlur={saveMenuInputs}
              onKeyDown={(event) => {
                if (event.key === "Enter") saveMenuInputs();
              }}
            />
          </label>
          <label>
            <span>Right raw</span>
            <input
              value={menu.rightText}
              inputMode="numeric"
              onChange={(event) => setMenu({ ...menu, rightText: event.target.value })}
              onBlur={saveMenuInputs}
              onKeyDown={(event) => {
                if (event.key === "Enter") saveMenuInputs();
              }}
            />
          </label>
          <div className="slider-context-actions">
            <button
              onClick={() => {
                const raw = Math.round(currentRawForAngle(menuServo));
                setMenu({ ...menu, leftText: String(raw) });
                applyLimits(isVisualReversed(menu.name) ? { max: raw } : { min: raw });
              }}
            >
              Set Current As Left
            </button>
            <button
              onClick={() => {
                const raw = Math.round(currentRawForAngle(menuServo));
                setMenu({ ...menu, rightText: String(raw) });
                applyLimits(isVisualReversed(menu.name) ? { min: raw } : { max: raw });
              }}
            >
              Set Current As Right
            </button>
            <button
              onClick={() => {
                onGuiAngleChange(menu.name, null);
                setMenu({
                  ...menu,
                  angleText: formatAngle(displayAngleFromRaw(menu.name, menuServo, menuServo.presentRaw)),
                });
              }}
            >
              Clear Current Angle
            </button>
            <button onClick={() => applyLimits({ arcInverted: !menuServo.arcInverted })}>
              Toggle Minor / Major Arc
            </button>
            <button
              onClick={() => {
                setMenu({
                  ...menu,
                  leftText: isVisualReversed(menu.name) ? "4095" : "0",
                  rightText: isVisualReversed(menu.name) ? "0" : "4095",
                });
                onLimitsChange(menu.name, { min: 0, max: 4095, arcInverted: false });
              }}
            >
              Reset Full Range
            </button>
          </div>
        </div>
      )}
    </div>
  );
}

export default ServoPanel;
