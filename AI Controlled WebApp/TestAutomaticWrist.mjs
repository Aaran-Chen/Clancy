import { build as Build } from "esbuild";
import { createRequire as CreateRequire } from "node:module";
import { test as Test } from "node:test";
import Assert from "node:assert/strict";
import Path from "node:path";
import { fileURLToPath as FilePath } from "node:url";

const Root = Path.dirname(FilePath(import.meta.url));
const Bundle = await Build({ stdin: { contents: `export * from './TargetWorkspace'; export * from './AutomaticWrist'; export * from './robotModel';
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


Test("a reachable point with the wrong wrist pitch keeps exact XYZ", () => {
  const Initial = Api.currentAngles(Api.INITIAL_SERVOS);
  const DefaultContext = { Servos: Api.INITIAL_SERVOS, ElbowMode: 'up' };
  let Recovered = 0;
  for (const Delta of [-25, 0, 20]) {
    const ReachablePose = Pose({ ...Initial, wrist_flex: Initial.wrist_flex + Delta });
    if (!Api.solveIkPreview(ReachablePose, DefaultContext.ElbowMode, DefaultContext.Servos).reachable) continue;
    const Request = { ...ReachablePose, wristPitch: ReachablePose.wristPitch + 120 };
    if (Api.solveIkPreview(Request, DefaultContext.ElbowMode, DefaultContext.Servos).reachable) continue;
    const Start = performance.now();
    const Resolved = Api.FindReachableWristTarget(Request, DefaultContext);
    Assert.ok(Resolved, 'Automatic wrist search must recover a known reachable point');
    Assert.equal(Distance(Resolved, Request), 0, 'XYZ is exact, not snapped away');
    Assert.ok(Api.solveIkPreview(Resolved, DefaultContext.ElbowMode, DefaultContext.Servos).reachable);
    Assert.equal(Api.BasketTargetReason(Resolved), null);
    Assert.notEqual(Resolved.wristPitch, Request.wristPitch);
    console.log('Automatic wrist recovery milliseconds:', Math.round(performance.now() - Start));
    Recovered++;
  }
  Assert.ok(Recovered >= 2);
});
Test("narrow and fixed joint ranges are recovered between sampled angles", () => {
  const Limits = Object.fromEntries(Api.IK_JOINTS.map(Name => [Name, [Angles[Name], Angles[Name]]]));
  const Request = { ...Previous, wristPitch: Previous.wristPitch + 90 };
  const Details = { ...Context, Limits };
  const Resolved = Api.FindReachableWristTarget(Request, Details);
  Assert.ok(Resolved);
  Assert.equal(Distance(Resolved, Request), 0);
  Assert.ok(Api.solveIkPreview(Resolved, Details.ElbowMode, Details.Servos, Limits).reachable);
});
Test("position snapping frees the wrist before changing coordinates", () => {
  const Request = { ...Previous, wristPitch: Previous.wristPitch + 110 };
  const Resolved = Api.FindNearestValidTarget(Request, Previous, Context, true);
  Assert.ok(Resolved); Assert.equal(Distance(Resolved, Request), 0);
  Assert.ok(Api.solveIkPreview(Resolved, Context.ElbowMode, Context.Servos).reachable);
  const Outside = { ...Previous, x: 60, y: 30, z: 10 };
  const Started = performance.now();
  const Snapped = Api.FindNearestValidTarget(Outside, Previous, Context, true);
  Assert.ok(Snapped); Assert.ok(Api.solveIkPreview(Snapped, Context.ElbowMode, Context.Servos).reachable);
  Assert.ok(Distance(Snapped, Outside) < Distance(Previous, Outside));
  console.log('Free-wrist workspace snap milliseconds:', Math.round(performance.now() - Started));
});
Test("already valid targets stay unchanged; collisions and nonfinite input remain blocked", () => {
  Assert.deepEqual(Api.FindReachableWristTarget(Previous, Context), Previous);
  Assert.equal(Api.FindReachableWristTarget({ ...Previous, x: 0, y: 0, z: -10 }, Context), null);
  Assert.equal(Api.FindReachableWristTarget({ ...Previous, x: NaN }, Context), null);
});

Test("the AI's inset search returns a pitch the IK reaches exactly, in 0.1 deg steps, just inside the manual search's edge", () => {
  const Initial = Api.currentAngles(Api.INITIAL_SERVOS);
  const DefaultContext = { Servos: Api.INITIAL_SERVOS, ElbowMode: 'up' };
  let Checked = 0;
  for (const Delta of [-25, 0, 20]) {
    const ReachablePose = Pose({ ...Initial, wrist_flex: Initial.wrist_flex + Delta });
    const Request = { ...ReachablePose, wristPitch: ReachablePose.wristPitch + 120 };
    const Edge = Api.FindReachableWristTarget(Request, DefaultContext);
    if (!Edge || Edge.wristPitch === Request.wristPitch) continue;
    const Inset = Api.FindInsetWristTarget(Request, DefaultContext);
    Assert.ok(Inset, 'The inset search finds what the manual search finds');
    Assert.equal(Distance(Inset, Request), 0, 'XYZ is exact');
    Assert.ok(Api.WristPitchConverges(Inset, Inset.wristPitch, DefaultContext));
    Assert.equal(Inset.wristPitch, Math.round(Inset.wristPitch * 10) / 10);
    const Extra = (Edge.wristPitch - Inset.wristPitch) * Math.sign(Request.wristPitch - Edge.wristPitch);
    Assert.ok(Extra >= 0.04 && Extra <= 0.4, `inset ${Extra.toFixed(3)} deg from the edge ${Edge.wristPitch}`);
    Checked++;
  }
  Assert.ok(Checked >= 2);
  // A pitch the IK reaches exactly is kept; a point beyond the arm's reach at any pitch is refused without a search.
  Assert.deepEqual(Api.FindInsetWristTarget(Previous, Context), Previous);
  Assert.equal(Api.FindInsetWristTarget({ ...Previous, x: 60, y: 30, z: 10 }, Context), null);
});

Test("the out-of-reach shortcut never refuses a point some pose of the arm reaches", () => {
  let Seed = 7;
  const Random = () => (Seed = (Seed * 16807) % 2147483647) / 2147483647;
  for (let Index = 0; Index < 3000; Index++) {
    const Angles = Object.fromEntries(Api.CadJointNames.map(Name => [Name, (Random() - 0.5) * 360]));
    // Every fifth pose stretched straight out, where the arm reaches farthest.
    if (Index % 5 === 0) Object.assign(Angles, { elbow_flex: Api.currentAngles(Api.INITIAL_SERVOS).elbow_flex + 164, wrist_flex: 0 });
    Assert.equal(Api.BeyondArmReach(Api.CadForward(Angles).Tool), false, JSON.stringify(Angles));
  }
  // Tight enough to matter: past the desk's real reach (42.5 cm from the pan axis) it answers without a search.
  const Pan = Api.mirrorDisplayPoint({ x: 0, y: 0, z: 0 });
  Assert.equal(Api.BeyondArmReach(Api.modelPointFromDisplayPoint({ x: Pan.x, y: 44, z: -1.3 })), true);
  Assert.equal(Api.BeyondArmReach(Api.modelPointFromDisplayPoint({ x: Pan.x, y: 40, z: -1.3 })), false);
});
