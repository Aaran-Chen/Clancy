import { build as Build } from "esbuild";
import { readFileSync as ReadFile } from "node:fs";
import { createRequire as CreateRequire } from "node:module";
import Assert from "node:assert/strict";
import { test as Test } from "node:test";
import Path from "node:path";
import { fileURLToPath as FilePath } from "node:url";

const Root = Path.dirname(FilePath(import.meta.url));
const Source = ReadFile(Path.join(Root, "src/App.tsx"), "utf8") + `
export { parseAiResponse as ParseResponse, addFallbackMotionFromText as Fallback,
  validateAiMotionProposal as Validate, HOME_MODEL_TARGET_CM as HomeTarget, HOME_RAW as Home,
  INITIAL_SERVOS as Servos, currentAngles as Angles, proposalDisplayPointCm as VisualPoint,
  modelPointFromVisualPoint as ModelPoint, buildAiSystemPrompt as AiPrompt,
  FixedAnchorVisualCm as Anchor };
export { RobotFrame, ReuseServoSnapshot, forwardKinematics as Forward,
  wristPitchFromAngles as Pitch, mirrorDisplayPoint as DisplayPoint, rawFromAngle as RawFromAngle } from "./lib/robotModel";
export { CadUrdfAngles as Urdf, GetCadAlignment as Alignment } from "./lib/CadKinematics";`;
const Bundle = await Build({ stdin: { contents: Source, resolveDir: Path.join(Root, "src"), loader: "tsx" },
  bundle: true, platform: "node", format: "cjs", write: false, loader: { ".css": "empty" } });
globalThis.window = { location: { hostname: "localhost" } };
const Module = { exports: {} };
new Function("require", "module", "exports", Bundle.outputFiles[0].text)(CreateRequire(import.meta.url), Module, Module.exports);
const App = Module.exports;

Test("unchanged telemetry reuses its snapshot and every servo field can still update", () => {
  const Previous = structuredClone(App.Servos);
  Assert.equal(App.ReuseServoSnapshot(Previous, structuredClone(Previous)), Previous);
  for (const Name of Object.keys(Previous)) {
    for (const Field of ["presentRaw", "targetRaw", "guiAngleRaw", "min", "max", "arcInverted"]) {
      const Next = structuredClone(Previous);
      Next[Name][Field] = Field === "arcInverted" ? !Next[Name][Field] : (Next[Name][Field] ?? 0) + 1;
      Assert.equal(App.ReuseServoSnapshot(Previous, Next), Next, `${Name}.${Field} must update`);
    }
  }
});

Test("shoulder pan follows corrected physical encoder direction across wraparound", () => {
  const Reference = App.Alignment().ReferenceRaw.shoulder_pan;
  for (const Delta of [-128, 128, 700]) {
    const Servos = structuredClone(App.Servos);
    Servos.shoulder_pan.presentRaw = (Reference + Delta + 4096) % 4096;
    const Radians = App.Urdf(App.Angles(Servos)).shoulder_pan;
    const Expected = (App.Alignment().UrdfAtReferenceDegrees.shoulder_pan + Delta * 360 / 4096) * Math.PI / 180;
    Assert.ok(Math.abs(Radians - Expected) < 1e-10);
  }
});

Test("workspace origin uses the arm midpoint at the bin rim and front is positive Y", () => {
  const Pan = App.VisualPoint({ x: 0, y: 0, z: 0 });
  for (const Axis of ["x", "y", "z"]) Assert.ok(Math.abs(Pan[Axis] - App.RobotFrame.ArmBasePositionCm[Axis]) < 1e-9);
  Assert.equal(Pan.y, 0);
  Assert.ok(Pan.z > 0);
  Assert.ok(App.VisualPoint({ x: 0, y: 10, z: 0 }).y > Pan.y);
  // The fixed tape's saved coordinates put it on the floor beside the cart: the model is not given them.
  Assert.ok(!App.AiPrompt("", true, false).includes(`X=${-App.Anchor.x} cm, Y=${App.Anchor.y} cm`));
  for (const Point of [{ x: -12, y: 11, z: -3 }, { x: 12, y: 23, z: 5 }]) {
    const RoundTrip = App.VisualPoint(App.ModelPoint(Point));
    for (const Axis of ["x", "y", "z"]) Assert.ok(Math.abs(RoundTrip[Axis] - Point[Axis]) < 1e-9);
  }
});

Test("explicit null beats stray root-level servo targets", () => {
  const Reply = App.ParseResponse(JSON.stringify({ response_to_user: "No movement", motion_proposal: null,
    direct_servo_targets: [{ name: "gripper", target_fraction: 1 }] }), App.HomeTarget, App.Servos);
  Assert.equal(Reply.motion, null);
});

Test("done and ask cannot produce motion", () => {
  for (const Status of ["done", "ask"]) {
    const Reply = App.ParseResponse(JSON.stringify({ task_status: Status, motion_proposal: {
      move_delta_cm: { left: 2, farther: 0, up: 0 } } }), App.HomeTarget, App.Servos);
    Assert.equal(Reply.motion, null);
  }
});

Test("prose and negations never become motion commands", () => {
  for (const Text of ["Do not move left 2 cm", "I cannot move down 3 inches", "move left 2 cm"]) {
    const Parsed = App.ParseResponse(Text, App.HomeTarget, App.Servos);
    Assert.equal(App.Fallback(Parsed, Text, Text, App.HomeTarget, App.Servos).motion, null);
  }
});

Test("low targets inside the basket remain blocked by collision checks", () => {
  // The wrist is explicit: an omitted one keeps the home pitch, so the verdict would follow home_preset.json.
  const Parsed = App.ParseResponse(JSON.stringify({ task_status: "continue", motion_proposal: {
    target_cm: { x: 0, y: 20, z: -10 }, wrist_pitch_degrees: -45 } }), App.HomeTarget, App.Servos);
  Assert.ok(Parsed.motion);
  const Result = App.Validate(Parsed.motion, "down", App.Servos);
  Assert.equal(Result.possible, false);
  // Every reachable arm pose for this point contacts the cart or the mast beside it.
  Assert.ok(Result.issues.some(Issue => Issue.includes("could contact the")), Result.issues.join("; "));
  Assert.ok(Result.issues.every(Issue => !Issue.includes("height")));
});

Test("AI accepts reachable negative-height targets outside the basket", () => {
  const Servos = structuredClone(App.Servos);
  // Forward and down past the cart front, below the rim, center of mass still over the cart.
  const Low = { shoulder_pan: 0.34323065528329266, shoulder_lift: -87.45806454570632, elbow_flex: 79.94271769861842, wrist_flex: -27.1169118284174 };
  for (const [Name, Value] of Object.entries(Low)) Servos[Name].presentRaw = Servos[Name].targetRaw = App.RawFromAngle(Servos[Name], Value);
  const CurrentAngles = App.Angles(Servos);
  const Current = { ...App.Forward(CurrentAngles).at(-1), wristPitch: App.Pitch(CurrentAngles) };
  const GoalServos = structuredClone(Servos);
  GoalServos.shoulder_pan.presentRaw -= 34;
  const GoalAngles = App.Angles(GoalServos), Goal = App.Forward(GoalAngles).at(-1);
  Assert.ok(App.VisualPoint(Goal).z < 0);
  const Parsed = App.ParseResponse(JSON.stringify({ task_status: "continue", motion_proposal: {
    target_cm: App.DisplayPoint(Goal), wrist_pitch_degrees: App.Pitch(GoalAngles),
  } }), Current, Servos);
  Assert.ok(Parsed.motion);
  const Result = App.Validate(Parsed.motion, "down", Servos);
  Assert.equal(Result.possible, true, Result.issues.join("; "));
  Assert.ok(Result.preview.reachable);
  Parsed.motion.requestedTarget.z = Number.NaN;
  Assert.equal(App.Validate(Parsed.motion, "down", Servos).possible, false, "Nonfinite coordinates remain forbidden");
});

Test("home preset comes from configuration and does not change CAD calibration", () => {
  const Saved = JSON.parse(ReadFile(Path.join(Root, "../back end/home_preset.json"), "utf8"));
  Assert.deepEqual(App.Home, Saved);
  const Original = App.Home.shoulder_pan;
  const Before = App.Angles(App.Servos).shoulder_pan;
  App.Home.shoulder_pan += 10;
  Assert.equal(App.Angles(App.Servos).shoulder_pan, Before);
  App.Home.shoulder_pan = Original;
});
