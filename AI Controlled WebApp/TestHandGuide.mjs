// Hand guiding on the page (src/lib/HandGuide.ts and its wiring in App.tsx): the bridge state read, the Motion panel's
// one button in every state, its disabled reasons (any device may guide: nothing asks for this PC), the strip and the
// emergency stop's label that say the arm goes limp, the command gate, and that App.tsx sends the heartbeat, ends
// guiding before AI mode and never starts an AI request while guiding. No browser, no bridge: node TestHandGuide.mjs
import { build as Build } from "esbuild";
import { test as Test } from "node:test";
import Assert from "node:assert/strict";
import { readFileSync } from "node:fs";

const Load = async File => {
  const Bundle = await Build({ entryPoints: [File], bundle: true, write: false, platform: "node", format: "esm" });
  return import(`data:text/javascript;base64,${Buffer.from(Bundle.outputFiles[0].text).toString("base64")}`);
};
const Guide = await Load("src/lib/HandGuide.ts"), Gate = await Load("src/lib/BridgeCommandGate.ts");
const App = readFileSync("src/App.tsx", "utf8");
const Ready = { SocketLive: true, ArmConnected: true, Torque: true, Moving: false, Fault: null, Manual: true, SelfCalibrating: false, Restarting: false };
// What the bridge's state_snapshot sends (bridge_server.py HandGuideSnapshot) while guiding, the gripper keeping a grip.
const Guiding = {
  hand_guide_version: "guide-v1",
  hand_guide: { version: "guide-v1", on: true, calibrating: false, last_exit: null, calibrated: true, calibrated_at: "2026-10-10T12:00:00",
    coast_brakes: false, calibration_result: null, phase: "guide", since_s: 12.5, owner: "another device", notes: ["shoulder lift: no friction help (it moved under the start's test)"],
    roles: { shoulder_pan: "free", shoulder_lift: "comp", elbow_flex: "comp", wrist_flex: "comp", wrist_roll: "free", gripper: "keep" },
    start_left_s: 0, free_states: { shoulder_pan: "float", wrist_roll: "assist" },
    assist: { shoulder_lift: { on: false }, elbow_flex: { on: true }, wrist_flex: { on: true }, shoulder_pan: { on: true, band_top_deg_s: 0 },
      wrist_roll: { on: true, band_top_deg_s: 23.4 } } },
};
const Idle = { hand_guide_version: "guide-v1", hand_guide: { version: "guide-v1", on: false, calibrating: false, calibrated: true, last_exit: null } };

Test("the bridge's hand_guide is read, and an older bridge reads as none", () => {
  const View = Guide.ReadHandGuide(Guiding);
  Assert.equal(View.On, true); Assert.equal(View.Phase, "guide"); Assert.equal(View.Owner, "another device");
  Assert.equal(View.Roles.gripper, "keep"); Assert.equal(View.Assist.wrist_roll.BandTopDegS, 23.4);
  Assert.deepEqual(Guide.ReadHandGuide({}), Guide.EmptyHandGuide);
  Assert.equal(Guide.ReadHandGuide({ hand_guide: "garbage" }).On, false);
});

Test("one button: Calibrate guiding, Stop calibration, Hand guide, Hold here", () => {
  const Uncalibrated = Guide.ReadHandGuide({ hand_guide_version: "guide-v1", hand_guide: { version: "guide-v1", calibrated: false } });
  let Button = Guide.HandGuideButton(Uncalibrated, Ready);
  Assert.equal(Button.Label, "Calibrate guiding");
  Assert.deepEqual(Button.Command, { type: "hand_guide_calibrate", explicit: true });
  Assert.match(Button.Confirm, /about 6 minutes/);
  Button = Guide.HandGuideButton(Guide.ReadHandGuide({ hand_guide_version: "guide-v1", hand_guide: { version: "guide-v1", calibrating: true, progress: 0.4 } }), Ready);
  Assert.equal(Button.Label, "Stop calibration");
  Assert.deepEqual(Button.Command, { type: "hand_guide_calibrate", enable: false });
  Button = Guide.HandGuideButton(Guide.ReadHandGuide(Idle), Ready);
  Assert.equal(Button.Label, "Hand guide");
  Assert.deepEqual(Button.Command, { type: "hand_guide", enable: true, explicit: true });
  Assert.equal(Button.DisabledReason, null);
  Assert.match(Button.Confirm, /goes limp/); Assert.match(Button.Confirm, /Emergency stop/); Assert.match(Button.Confirm, /3 s/);
  Button = Guide.HandGuideButton(Guide.ReadHandGuide(Guiding), { ...Ready, Torque: true, Manual: false });
  Assert.equal(Button.Label, "Hold here");
  Assert.deepEqual(Button.Command, { type: "hand_guide", enable: false });
  Assert.equal(Button.DisabledReason, null, "Hold here always works while the bridge is reachable");
  Assert.equal(Button.Confirm, null);
});

Test("why the button waits: the bridge, the mode, the arm; never which device", () => {
  const View = Guide.ReadHandGuide(Idle);
  for (const [Context, Words] of [[{ SocketLive: false }, /Connect the bridge/], [{ Manual: false }, /Manual mode/], [{ ArmConnected: false }, /Connect the arm/],
    [{ Torque: false }, /Press Hold first/], [{ Moving: true }, /stop moving/], [{ Fault: "elbow overload" }, /Resolve the fault/],
    [{ SelfCalibrating: true }, /camera calibration/], [{ Restarting: true }, /bridge update/]]) {
    Assert.match(Guide.HandGuideButton(View, { ...Ready, ...Context }).DisabledReason, Words);
  }
  Assert.match(Guide.HandGuideButton(Guide.ReadHandGuide({}), Ready).DisabledReason, /Load the updated bridge/);
  Assert.doesNotMatch(Guide.HandGuideButton.toString(), /ThisPc|local/i, "hand guiding is allowed from any device with the password");
});

Test("the strip and the emergency stop say the arm goes limp", () => {
  const View = Guide.ReadHandGuide(Guiding);
  const Strip = Guide.HandGuideStrip(View);
  Assert.equal(Strip.Tone, "guide");
  Assert.match(Strip.Title, /Hold here to stop · guided from another device/);
  Assert.match(Strip.Detail, /The emergency stop, or anything going wrong, makes it go limp: keep a hand on it/);
  Assert.match(Strip.Detail, /friction help: elbow flex, wrist flex, wrist roll/);
  Assert.doesNotMatch(Strip.Detail, /shoulder pan/, "a free joint without a band gets no help");
  Assert.match(Strip.Detail, /gripper keeps its grip/);
  Assert.match(Strip.Detail, /no friction help \(it moved under the start's test\)/);
  const Start = Guide.HandGuideStrip(Guide.ReadHandGuide({ hand_guide: { ...Guiding.hand_guide, phase: "ramp", start_left_s: 1.24 } }));
  Assert.equal(Start.Tone, "start"); Assert.match(Start.Title, /starts in 1\.2 s: let go of the arm/); Assert.match(Start.Detail, /emergency stop, or anything going wrong, makes it go limp/);
  Assert.equal(Guide.HandGuideStrip(Guide.ReadHandGuide(Idle)), null);
  Assert.equal(Guide.EmergencyStopLabel(View), "Emergency stop: the arm drops");
  Assert.equal(Guide.EmergencyStopLabel(Guide.ReadHandGuide(Idle)), "Emergency stop");
  const Calibrating = Guide.HandGuideStrip(Guide.ReadHandGuide({ hand_guide: { calibrating: true, progress: 0.42 } }));
  Assert.match(Calibrating.Title, /42 %/);
  // Coast is torque off on every joint: while guiding (or calibrating) its button says the arm drops (2026-10-10 review L5)
  Assert.deepEqual(Guide.CoastWarning(View), { Text: "Coast: arm drops",
    Title: "Coast turns every joint's torque off: the arm drops. Hold here holds it where it is" });
  Assert.equal(Guide.CoastWarning(Guide.ReadHandGuide({ hand_guide: { calibrating: true } })).Text, "Coast: arm drops");
  Assert.equal(Guide.CoastWarning(Guide.ReadHandGuide(Idle)), null);
});

Test("an ending is logged once, a limp one as a warning", () => {
  const Limp = { Kind: "coast", Reason: "Hand guiding went limp: shoulder_lift at 55 C", AtS: 10, Mode: "guide" };
  Assert.deepEqual(Guide.HandGuideExitNote(null, Limp), { Level: "warn", Text: "Hand guiding went limp: shoulder_lift at 55 C" });
  Assert.equal(Guide.HandGuideExitNote(null, { ...Limp, Reason: "USB serial link to the arm dropped during a move" }).Text,
    "Hand guiding went limp: USB serial link to the arm dropped during a move");
  Assert.equal(Guide.HandGuideExitNote(Limp, Limp), null);
  Assert.equal(Guide.HandGuideExitNote({ ...Limp }, null), null);
  Assert.equal(Guide.HandGuideExitNote(null, { ...Limp, Kind: "preloaded", Reason: "Hold here" }).Level, "info");
});

Test("the command gate holds moves while guiding and lets every stop through", () => {
  const Context = { SocketOpen: true, BasketCompatible: true, CadCompatible: true, SpeedCompatible: true, Calibrating: false, HandGuiding: true };
  for (const Command of ["send_ik", "move_servo", "go_home", "set_home", "set_servo_limits", "set_gui_angle", "set_target"]) {
    Assert.match(Gate.CommandBlockReason({ type: Command }, Context), /Hand guiding is on: press Hold here first/);
    Assert.equal(Gate.CommandBlockReason({ type: Command }, { ...Context, HandGuiding: false }), null);
  }
  for (const Command of ["stop_motion", "emergency_stop", "disconnect", "toggle_torque", "hand_guide", "hand_guide_calibrate", "hand_guide_heartbeat", "set_motion_speed"]) {
    Assert.equal(Gate.CommandBlockReason({ type: Command }, Context), null, Command);
  }
});

Test("App.tsx wires it: the heartbeat, the gate, AI mode, the button and both emergency stops", () => {
  Assert.match(App, /type: "hand_guide_heartbeat"/);
  Assert.match(App, /HeartbeatMs/);
  Assert.equal((App.match(/HandGuiding: HandGuideRef\.current\.On \|\| HandGuideRef\.current\.Calibrating/g) ?? []).length, 2, "both gate checks know");
  Assert.match(App, /if \(Next === "Ai" && \(HandGuideRef\.current\.On \|\| HandGuideRef\.current\.Calibrating\)\)[\s\S]{0,400}type: "hand_guide", enable: false/);
  Assert.match(App, /setAiStatus\("Not started: hand guiding is on"\)/);
  Assert.match(App, /onClick=\{\(\) => PressHandGuide\(GuideButton\)\}/);
  Assert.equal((App.match(/\{EmergencyStopLabel\(HandGuideState\)\}/g) ?? []).length, 2, "every emergency stop says the arm drops");
  Assert.match(App, /const CoastDrops = torque \? CoastWarning\(HandGuideState\) : null/);
  Assert.match(App, /aria-label=\{CoastDrops \? "Coast" : undefined\} title=\{CoastDrops\?\.Title\}/, "Coast keeps its name, says it drops the arm");
  Assert.match(App, /sessionStorage\.getItem\(FirstUseKey\)/);
  Assert.match(App, /State: HandGuideState\.On \? "GUIDE"/);
  Assert.doesNotMatch(App.slice(App.indexOf("const PressHandGuide"), App.indexOf("const StopAndHold")), /ThisPc/, "any device");
});

Test("a start the bridge refuses for want of a fitting calibration asks for one", () => {
  Assert.equal(Guide.NeedsCalibration("Hand guiding not started: elbow_flex's settings changed since hand guiding was calibrated (p): calibrate hand guiding again"), true);
  Assert.equal(Guide.NeedsCalibration("Hand guiding not started: hand guiding is not calibrated: press Calibrate hand guiding first"), true);
  Assert.equal(Guide.NeedsCalibration("Hand guiding not started: the servo supply reads 8.2 V (at least 8.5 V needed); check the 12 V supply and its cable"), false);
  Assert.equal(Guide.NeedsCalibration("Hold here: hand guiding ended, holding where it is"), false);
  Assert.match(App, /NeedsCalibration\(String\(message\.text/);
  Assert.match(App, /aria-label="Calibrate hand guiding again"/);
});
