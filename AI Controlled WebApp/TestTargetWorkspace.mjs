import { build as Build } from "esbuild";
import { createRequire as CreateRequire } from "node:module";
import { test as Test } from "node:test";
import Assert from "node:assert/strict";
import Path from "node:path";
import { fileURLToPath as FilePath } from "node:url";

const Root = Path.dirname(FilePath(import.meta.url));
const Bundle = await Build({ stdin: { contents: `export * from './TargetWorkspace'; export * from './robotModel';
  export * from './CadKinematics'; export * from './BasketCollision';`, resolveDir: Path.join(Root, "src/lib"), loader: "ts" },
  bundle: true, platform: "node", format: "cjs", write: false });
const Module = { exports: {} };
new Function("require", "module", "exports", Bundle.outputFiles[0].text)(CreateRequire(import.meta.url), Module, Module.exports);
const Api = Module.exports;
const Servos = structuredClone(Api.INITIAL_SERVOS);
// Forward and down past the cart front, below the rim, with the arm's center of mass still over the cart.
const LowAngles = { shoulder_pan: 0.34323065528329266, shoulder_lift: -87.45806454570632, elbow_flex: 79.94271769861842,
  wrist_flex: -27.1169118284174, wrist_roll: Api.currentAngles(Servos).wrist_roll, gripper: Api.currentAngles(Servos).gripper };
for (const [Name, Value] of Object.entries(LowAngles)) {
  const Raw = Api.rawFromAngle(Servos[Name], Value);
  Object.assign(Servos[Name], { presentRaw: Raw, targetRaw: Raw, guiAngleRaw: null });
}
const Angles = Api.currentAngles(Servos);
const Pose = Values => { const Point = Api.CadForward(Values).Tool; return { x: Point.x, y: Point.y, z: Point.z, wristPitch: Api.CadPitch(Values) }; };
const Previous = Pose(Angles), Context = { Servos, ElbowMode: "down" };
const Distance = (A, B) => Math.hypot(A.x - B.x, A.y - B.y, A.z - B.z);
const Verify = (Target, Request, Details = Context) => {
  Assert.ok(Target, "A valid target was found");
  const Preview = Api.solveIkPreview(Target, Details.ElbowMode, Details.Servos, Details.Limits);
  Assert.ok(Preview.reachable, JSON.stringify(Preview));
  Assert.equal(Preview.collisionReason, null);
  Assert.equal(Target.wristPitch, Request.wristPitch, "Pitch is preserved");
};

Test("reachable negative-Z targets stay exactly where placed", () => {
  Assert.ok(Previous.z + Api.RobotFrame.ArmBasePositionCm.z < 0);
  const Result = Api.FindNearestValidTarget(Previous, Previous, Context);
  Verify(Result, Previous); Assert.deepEqual(Result, Previous);
});

Test("nearest point matches the analytic answer on a restricted pan arc", () => {
  const Limits = Object.fromEntries(Api.IK_JOINTS.map(Name => [Name, [Angles[Name], Angles[Name]]]));
  // The whole arc is clear of the basket and keeps the center of mass over the cart.
  Limits.shoulder_pan = [Angles.shoulder_pan - 8, Angles.shoulder_pan];
  for (const Delta of [-20, -4, 3, 20]) {
    const Aim = Pose({ ...Angles, shoulder_pan: Angles.shoulder_pan + Delta });
    const Request = { ...Aim, x: Aim.x * 1.7, y: Aim.y * 1.7, wristPitch: Previous.wristPitch };
    const Expected = Pose({ ...Angles, shoulder_pan: Angles.shoulder_pan + Math.max(-8, Math.min(0, Delta)) });
    const Result = Api.FindNearestValidTarget(Request, Previous, { ...Context, Limits });
    Verify(Result, Request, { ...Context, Limits });
    Assert.ok(Distance(Result, Expected) < .025, `Nearest arc point missed by ${Distance(Result, Expected)} cm`);
  }
});

Test("outside reach and basket targets snap to valid positions instead of returning to the drag start", () => {
  const HomeContext = { Servos: Api.INITIAL_SERVOS, ElbowMode: "up" };
  const Home = Api.HOME_MODEL_TARGET_CM;
  for (const Coordinates of [[80, 20, 5], [-80, 20, 5], [0, 100, 50], [0, 0, -10], [25, -70, -40]]) {
    const Request = { x: Coordinates[0], y: Coordinates[1], z: Coordinates[2], wristPitch: Home.wristPitch };
    const Result = Api.FindNearestValidTarget(Request, Home, HomeContext);
    Verify(Result, Request, HomeContext);
    Assert.ok(Distance(Result, Request) < Distance(Home, Request), "Snap improves on simply resetting the target");
  }
});

Test("snap distance beats an independent dense sample of feasible arm poses", () => {
  const Request = { ...Previous, x: 0, y: 0, z: -10 };
  const Result = Api.FindNearestValidTarget(Request, Previous, Context);
  Verify(Result, Request);
  const Limits = Api.GetIkAngleLimits(Servos);
  let Seed = 7, Best = Infinity, Usable = 0;
  const Random = () => { Seed = (1664525 * Seed + 1013904223) >>> 0; return Seed / 2 ** 32; };
  for (let Index = 0; Index < 18000; Index++) {
    const Trial = { ...Angles, wrist_flex: 0 };
    for (const Name of ["shoulder_pan", "shoulder_lift", "elbow_flex"]) Trial[Name] = Limits[Name][0] + Random() * (Limits[Name][1] - Limits[Name][0]);
    const PitchDelta = ((Api.CadPitch(Trial) - Request.wristPitch + 540) % 360) - 180;
    Trial.wrist_flex = PitchDelta / Api.GetCadAlignment().Direction.wrist_flex;
    if (Trial.wrist_flex < Limits.wrist_flex[0] || Trial.wrist_flex > Limits.wrist_flex[1]) continue;
    const Target = Pose(Trial);
    if (Api.BasketTargetReason(Target) || Api.BasketPoseReason(Trial)) continue;
    Usable++; Best = Math.min(Best, Distance(Target, Request));
  }
  Assert.ok(Usable > 500);
  Assert.ok(Distance(Result, Request) <= Best + .025, `Snap ${Distance(Result, Request)} vs dense sample ${Best}`);
});

Test("impossible fixed joint limits fail closed and nonfinite input is rejected", () => {
  const Limits = Object.fromEntries(Api.IK_JOINTS.map(Name => [Name, [Angles[Name], Angles[Name]]]));
  const Request = { ...Previous, wristPitch: Previous.wristPitch + 90 };
  Assert.equal(Api.FindNearestValidTarget(Request, Previous, { ...Context, Limits }), null);
  for (const Axis of ["x", "y", "z", "wristPitch"]) {
    Assert.equal(Api.FindNearestValidTarget({ ...Previous, [Axis]: NaN }, Previous, Context), null);
  }
});

Test("a release reachable at another wrist angle keeps its position and only turns the wrist", () => {
  // App.tsx shows a released target red for at most TargetSnapLimitMs (0.5 s); the worker
  // tries the wrist first with 4 deg samples (TargetWorkspaceWorker.ts ReleaseWristSearch).
  const Request = [70, 90, 110, 130, -70, -90, -110].map(Offset => ({ ...Previous, wristPitch: Previous.wristPitch + Offset }))
    .find(Candidate => !Api.solveIkPreview(Candidate, Context.ElbowMode, Context.Servos).reachable);
  Assert.ok(Request, "an unreachable wrist angle at a reachable position");
  const Started = performance.now();
  const Result = Api.FindNearestValidTarget(Request, Previous, Context, true, undefined, { Search: { StepDeg: 4 } });
  const Milliseconds = performance.now() - Started;
  Assert.ok(Result, "a wrist angle was found");
  Assert.deepEqual([Result.x, Result.y, Result.z], [Request.x, Request.y, Request.z], "position preserved exactly");
  Assert.notEqual(Result.wristPitch, Request.wristPitch);
  Assert.ok(Api.solveIkPreview(Result, Context.ElbowMode, Context.Servos).reachable);
  // The wrist-only answer, not the multi-second position search behind it.
  Assert.ok(Milliseconds < 1500, `${Milliseconds.toFixed(0)} ms`);
});
