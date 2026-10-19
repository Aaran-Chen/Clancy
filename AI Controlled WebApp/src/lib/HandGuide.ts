// Hand guiding on the page (docs/HandGuide.md 6): what the bridge's state says about it (hand_guide), the Motion panel's
// one button (Calibrate guiding, Stop calibration, Hand guide or Hold here), the status strip while it runs, the
// emergency stop's label, and the heartbeat the page that started it sends. Pure functions: App.tsx wires them.
// Hand guiding is allowed from any device that has the arm password (Aaran's decision D2, 2026-10-10); any fault while
// guiding makes the arm go limp (D1), and the page says so wherever it can be pressed.

export const HandGuideVersion = "guide-v1";
// The page that started a session sends this every HeartbeatMs; the bridge holds the arm when it has heard nothing
// for 3 s (a closed page, a phone gone to sleep).
export const HeartbeatMs = 500;
export const FirstUseKey = "SO101.HandGuideExplained";

export type HandGuideExit = { Kind: string; Reason: string; AtS: number; Mode: string };
export type HandGuideCalibrationResult = { Ok: boolean; Problems: string[]; Error: string | null; AtS: number };
export type HandGuideView = {
  Version: string | null; On: boolean; Calibrating: boolean; Phase: string; StartLeftS: number; Owner: string | null;
  Roles: Record<string, string>; Notes: string[]; Progress: number | null; Calibrated: boolean; CalibratedAt: string | null;
  CoastBrakes: boolean | null; LastExit: HandGuideExit | null; CalibrationResult: HandGuideCalibrationResult | null;
  Assist: Record<string, { On: boolean; BandTopDegS: number | null }>;
};

export const EmptyHandGuide: HandGuideView = {
  Version: null, On: false, Calibrating: false, Phase: "", StartLeftS: 0, Owner: null, Roles: {}, Notes: [], Progress: null,
  Calibrated: false, CalibratedAt: null, CoastBrakes: null, LastExit: null, CalibrationResult: null, Assist: {},
};

const Record_ = (Value: unknown): Record<string, unknown> | null =>
  typeof Value === "object" && Value !== null && !Array.isArray(Value) ? Value as Record<string, unknown> : null;
const Text = (Value: unknown): string | null => typeof Value === "string" ? Value : null;
const Number_ = (Value: unknown): number | null => typeof Value === "number" && Number.isFinite(Value) ? Value : null;

/** The bridge state's hand_guide (and hand_guide_version), read defensively: a bridge without it reads as EmptyHandGuide. */
export function ReadHandGuide(State: Record<string, unknown>): HandGuideView {
  const Guide = Record_(State.hand_guide);
  const Version = Text(State.hand_guide_version) ?? Text(Guide?.version);
  if (!Guide) return { ...EmptyHandGuide, Version };
  const Exit = Record_(Guide.last_exit), Result = Record_(Guide.calibration_result);
  const Assist: HandGuideView["Assist"] = {};
  for (const [Name, Value] of Object.entries(Record_(Guide.assist) ?? {})) {
    const Item = Record_(Value);
    if (Item) Assist[Name] = { On: Item.on === true, BandTopDegS: Number_(Item.band_top_deg_s) };
  }
  return {
    Version, On: Guide.on === true, Calibrating: Guide.calibrating === true, Phase: Text(Guide.phase) ?? "",
    StartLeftS: Number_(Guide.start_left_s) ?? 0, Owner: Text(Guide.owner),
    Roles: Object.fromEntries(Object.entries(Record_(Guide.roles) ?? {}).filter(([, Role]) => typeof Role === "string")) as Record<string, string>,
    Notes: Array.isArray(Guide.notes) ? Guide.notes.filter((Note): Note is string => typeof Note === "string") : [],
    Progress: Number_(Guide.progress), Calibrated: Guide.calibrated === true, CalibratedAt: Text(Guide.calibrated_at),
    CoastBrakes: typeof Guide.coast_brakes === "boolean" ? Guide.coast_brakes : null,
    LastExit: Exit ? { Kind: Text(Exit.kind) ?? "", Reason: Text(Exit.reason) ?? "", AtS: Number_(Exit.at_s) ?? 0, Mode: Text(Exit.mode) ?? "guide" } : null,
    CalibrationResult: Result ? { Ok: Result.ok === true, AtS: Number_(Result.at_s) ?? 0, Error: Text(Result.error),
      Problems: Array.isArray(Result.problems) ? Result.problems.filter((P): P is string => typeof P === "string") : [] } : null,
    Assist,
  };
}

export type HandGuideContext = {
  SocketLive: boolean; ArmConnected: boolean; Torque: boolean; Moving: boolean; Fault: string | null; Manual: boolean;
  SelfCalibrating: boolean; Restarting: boolean;
};
export type HandGuideButtonView = {
  Label: string; Title: string; Command: Record<string, unknown> | null; DisabledReason: string | null;
  Confirm: string | null; Primary: boolean;
};

export const FirstUseText = "Hand guiding: the shoulder, elbow and wrist carry their own weight and every joint gets a little "
  + "help when you move it slowly. Press it with the arm holding, then keep your hands off for 2 s while it starts.\n\n"
  + "If anything goes wrong (a servo, the USB cable, the supply, heat) the whole arm goes limp, and so does the "
  + "Emergency stop: keep a hand on or under the arm while guiding. Hold here holds it where it is.\n\n"
  + "Keep this page open and the screen on: if it closes or sleeps for 3 s, the arm holds where it is.";
export const CalibrateText = "Calibrate hand guiding (about 6 minutes): every joint sweeps slowly through a stretch of its "
  + "range, checked against the basket and the cart first, then the roll and the gripper spin briefly. Keep clear of the "
  + "arm, with a hand ready. Stop and hold ends it holding where it is.";

/** The Motion panel's hand-guide button for this state: what it says, what it sends, and why it is disabled. */
export function HandGuideButton(View: HandGuideView, Context: HandGuideContext): HandGuideButtonView {
  if (View.On) {
    return { Label: "Hold here", Title: "End hand guiding: the arm holds exactly where it is", Primary: true, Confirm: null,
      Command: { type: "hand_guide", enable: false }, DisabledReason: Context.SocketLive ? null : "The bridge connection is offline" };
  }
  if (View.Calibrating) {
    return { Label: "Stop calibration", Title: "Stop calibrating hand guiding: the arm holds where it is", Primary: true, Confirm: null,
      Command: { type: "hand_guide_calibrate", enable: false }, DisabledReason: Context.SocketLive ? null : "The bridge connection is offline" };
  }
  const Calibrate = !View.Calibrated;
  const Base = Calibrate
    ? { Label: "Calibrate guiding", Title: "Calibrate hand guiding (about 6 min) before its first use", Confirm: CalibrateText,
        Command: { type: "hand_guide_calibrate", explicit: true } }
    : { Label: "Hand guide", Title: "Move the arm by hand: it carries its own weight; faults make it go limp", Confirm: FirstUseText,
        Command: { type: "hand_guide", enable: true, explicit: true } };
  const Reason = !Context.SocketLive ? "Connect the bridge first"
    : View.Version !== HandGuideVersion ? "Load the updated bridge: this one has no hand guiding"
    : !Context.Manual ? "Hand guiding is for Manual mode"
    : Context.Restarting ? "Wait for the bridge update to finish"
    : !Context.ArmConnected ? "Connect the arm first"
    : Context.SelfCalibrating ? "Wait for the camera calibration to finish"
    : Context.Fault ? "Resolve the fault and press Hold first"
    : !Context.Torque ? "Press Hold first: it starts from an arm that holds"
    : Context.Moving ? "Wait for the arm to stop moving"
    : null;
  return { ...Base, Primary: false, DisabledReason: Reason };
}

/** The status strip while a session runs (null otherwise). */
export function HandGuideStrip(View: HandGuideView): { Title: string; Detail: string; Tone: "start" | "guide" | "calibrate" } | null {
  if (View.Calibrating) {
    return { Tone: "calibrate", Title: `Calibrating hand guiding · ${Math.round((View.Progress ?? 0) * 100)} %`,
      Detail: "Keep clear of the arm, a hand ready. Stop calibration (or Stop and hold) ends it holding; the emergency stop or a fault makes it go limp." };
  }
  if (!View.On) return null;
  const From = View.Owner ? ` · guided from ${View.Owner}` : "";
  if (["preload", "ramp", "probe"].includes(View.Phase)) {
    return { Tone: "start", Title: `Hand guiding starts in ${Math.max(0, View.StartLeftS).toFixed(1)} s: let go of the arm${From}`,
      Detail: "It is checking that it carries the arm. The emergency stop, or anything going wrong, makes it go limp: keep a hand near." };
  }
  const Helped = Object.entries(View.Assist).filter(([, Item]) => Item.On && (Item.BandTopDegS === null || Item.BandTopDegS > 0))
    .map(([Name]) => Name.replace(/_/g, " "));
  const Kept = Object.entries(View.Roles).filter(([, Role]) => Role === "keep").map(([Name]) => Name.replace(/_/g, " "));
  const Parts = [Helped.length ? `friction help: ${Helped.join(", ")}` : "no friction help", ...(Kept.length ? [`${Kept.join(", ")} keeps its grip`] : []), ...View.Notes];
  return { Tone: "guide", Title: `Hand guiding: move the arm by hand. Hold here to stop${From}`,
    Detail: `The emergency stop, or anything going wrong, makes it go limp: keep a hand on it. ${Parts.join(" · ")}` };
}

/** The emergency stop's label: while guiding it says that the arm drops. */
export function EmergencyStopLabel(View: HandGuideView): string {
  return View.On || View.Calibrating ? "Emergency stop: the arm drops" : "Emergency stop";
}

/** The Coast button while guiding or calibrating (Coast is torque off on every joint: the arm drops), or null: its visible
 * text and its title. Its accessible name stays "Coast". */
export function CoastWarning(View: HandGuideView): { Text: string; Title: string } | null {
  return View.On || View.Calibrating
    ? { Text: "Coast: arm drops", Title: "Coast turns every joint's torque off: the arm drops. Hold here holds it where it is" }
    : null;
}

/** A line for the log when a session ended (Previous is the last exit already logged), or null. */
export function HandGuideExitNote(Previous: HandGuideExit | null, Next: HandGuideExit | null): { Level: "info" | "warn"; Text: string } | null {
  if (!Next || (Previous && Previous.AtS === Next.AtS && Previous.Reason === Next.Reason)) return null;
  const Limp = Next.Kind === "coast";
  return { Level: Limp ? "warn" : "info", Text: Limp && !/went limp/.test(Next.Reason) ? `Hand guiding went limp: ${Next.Reason}` : Next.Reason };
}

/** Whether a command must wait while hand guiding or its calibration runs (the bridge refuses them too). */
export function BlockedWhileGuiding(Command: string): boolean {
  return ["send_ik", "move_servo", "go_home", "set_home", "set_servo_limits", "set_gui_angle", "set_target"].includes(Command);
}

/** Whether a bridge refusal says hand guiding needs calibrating (again): its calibration is missing, from another version,
 * or no longer fits the arm (a servo's settings or the zero pose changed). */
export function NeedsCalibration(Text: string): boolean {
  return /^Hand guiding not started: /.test(Text) && /calibrate hand guiding|not calibrated|calibrate again/i.test(Text);
}
