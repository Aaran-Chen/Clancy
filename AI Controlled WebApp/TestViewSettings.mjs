import { build as Build } from "esbuild";
import { test as Test } from "node:test";
import Assert from "node:assert/strict";
const Bundle = await Build({ entryPoints: ["src/lib/ViewSettings.ts"], bundle: true, write: false, platform: "node", format: "esm" });
const Api = await import(`data:text/javascript;base64,${Buffer.from(Bundle.outputFiles[0].text).toString("base64")}`);
Test("default graph is unlabeled while the ray and green angle guide remain visible", () => {
  const Settings = Api.NormalizeViewSettings(null);
  Assert.equal(Settings.CoordinateLabels, "Hidden");
  Assert.equal(Settings.AxisWidth, 3);
  Assert.equal(Settings.ShowGraph, true);
  Assert.equal(Settings.ShowAngleGuide, true);
  Assert.equal(Settings.ShowApproachRay, true);
});
Test("corrupt, unsupported and out-of-range settings cannot break rendering", () => {
  for (const Invalid of [null, [], true, "bad"]) Assert.deepEqual(Api.NormalizeViewSettings(Invalid), Api.DefaultViewSettings);
  const Result = Api.NormalizeViewSettings({ ShowGraph: "false", ShowGrid: false, CoordinateLabels: "script",
    AxisWidth: 500, GridOpacity: -30, ZoomSpeed: NaN, OrbitSpeed: Infinity, PanSpeed: -5, Torque: false });
  Assert.equal(Result.ShowGraph, true); Assert.equal(Result.ShowGrid, false);
  Assert.equal(Result.CoordinateLabels, "Hidden"); Assert.equal(Result.AxisWidth, 6);
  Assert.equal(Result.GridOpacity, .1); Assert.equal(Result.ZoomSpeed, 1); Assert.equal(Result.OrbitSpeed, 1); Assert.equal(Result.PanSpeed, .25);
  Assert.equal("Torque" in Result, false);
  for (const CoordinateLabels of ["Hidden", "PositiveY", "All"]) Assert.equal(Api.NormalizeViewSettings({ CoordinateLabels }).CoordinateLabels, CoordinateLabels);
});
Test("settings persist and unavailable storage falls back cleanly", () => {
  const Values = new Map();
  globalThis.localStorage = { getItem: Key => Values.get(Key), setItem: (Key, Value) => Values.set(Key, Value) };
  const Saved = { ...Api.DefaultViewSettings, CoordinateLabels: "PositiveY", ShowBody: false, PanSpeed: 1.5 };
  Api.SaveViewSettings(Saved); Assert.deepEqual(Api.ReadViewSettings(), Saved);
  globalThis.localStorage = { getItem: () => { throw Error("Storage blocked"); }, setItem: () => { throw Error("Storage blocked"); } };
  Assert.deepEqual(Api.ReadViewSettings(), Api.DefaultViewSettings);
  Assert.doesNotThrow(() => Api.SaveViewSettings(Saved));
});
// The grasp gap at the fixed finger (src/lib/GraspGap.ts): a planner setting kept beside the display ones, not reset with them.
const GapBundle = await Build({ entryPoints: ["src/lib/GraspGap.ts"], bundle: true, write: false, platform: "node", format: "esm", logLevel: "error" });
const Gap = await import(`data:text/javascript;base64,${Buffer.from(GapBundle.outputFiles[0].text).toString("base64")}`);
// Its least is the planner's PlanMarginCm (0.5): every gripper part, the fixed pad too, keeps that from the object, so 0.3 and
// 0.4 could never plan anything (the review found every box and the 18:53 can refused there); a saved 0.3 reads as 0.5.
Test("the grasp gap persists, stays in its 0.5-1.5 cm range on 0.1 cm steps, and falls back to the planner's 1.0 cm", () => {
  Assert.deepEqual(Gap.GraspGapRange, { MinCm: 0.5, MaxCm: 1.5, StepCm: 0.1 });
  Assert.equal(Gap.DefaultGraspGapCm, 1);
  for (const [Value, Expected] of [[0.8, 0.8], [0.84, 0.8], [0.86, 0.9], [0.5, 0.5], [0.3, 0.5], [0.44, 0.5], [0.05, 0.5], [9, 1.5], [-1, 0.5], [NaN, 1], [Infinity, 1], ["0.5", 1], [null, 1], [undefined, 1]]) {
    Assert.equal(Gap.NormalizeGraspGap(Value), Expected, `${Value}`);
  }
  const Values = new Map();
  globalThis.localStorage = { getItem: Key => Values.get(Key) ?? null, setItem: (Key, Value) => Values.set(Key, Value) };
  Assert.equal(Gap.ReadGraspGap(), 1, "nothing saved");
  Gap.SaveGraspGap(0.6);
  Assert.equal(Values.get("SO101.Dashboard.v1.GraspGapCm"), "0.6");
  Assert.equal(Gap.ReadGraspGap(), 0.6);
  Values.set("SO101.Dashboard.v1.GraspGapCm", "\"tight\""); Assert.equal(Gap.ReadGraspGap(), 1, "corrupt");
  Values.set("SO101.Dashboard.v1.GraspGapCm", "{"); Assert.equal(Gap.ReadGraspGap(), 1, "not JSON");
  globalThis.localStorage = { getItem: () => { throw Error("Storage blocked"); }, setItem: () => { throw Error("Storage blocked"); } };
  Assert.equal(Gap.ReadGraspGap(), 1);
  Assert.doesNotThrow(() => Gap.SaveGraspGap(0.5));
  // A refusal names the setting where it is part of why: a pad placed below the default, refused for a part too near the
  // object (path, palm): a wider gap may plan it. A "wide" refusal grows with the gap, so it never says that: above the
  // default it says the wider gap is why the moving jaw must close so far, and below or at the default nothing.
  const Wider = "the grasp gap at the fixed finger is set to 0.5 cm in Settings (the default is 1.0 cm): a wider gap may plan it";
  for (const Code of ["path", "palm"]) Assert.deepEqual(Gap.GraspGapHint({ Code, GapCm: 0.5 }), { Text: Wider, Advice: `Or widen the grasp gap: ${Wider}.` }, Code);
  Assert.deepEqual(Gap.GraspGapHint({ Code: "wide", GapCm: 1.3 }), {
    Text: "the grasp gap at the fixed finger is set to 1.3 cm in Settings (the default is 1.0 cm): the wider the gap, the farther the moving jaw must close",
    Advice: "The grasp gap at the fixed finger is set to 1.3 cm in Settings (the default is 1.0 cm): the wider the gap, the farther the moving jaw must close." });
  for (const [Code, GapCm] of [["wide", 0.5], ["wide", 1], ["path", 1], ["path", 1.3], ["palm", undefined], ["neighbour", 0.5], ["reach", 0.5], ["thin", 0.5], ["flat", 0.5], ["unmeasured", 0.5]]) {
    Assert.equal(Gap.GraspGapHint({ Code, GapCm }), null, `${Code} ${GapCm}`);
  }
});
