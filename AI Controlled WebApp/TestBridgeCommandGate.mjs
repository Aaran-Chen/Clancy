import { build as Build } from "esbuild";
import { test as Test } from "node:test";
import Assert from "node:assert/strict";
import { EventEmitter } from "node:events";
const Load = async File => {
  const Bundle = await Build({ entryPoints: [File], bundle: true, write: false, platform: "node", format: "esm" });
  return import(`data:text/javascript;base64,${Buffer.from(Bundle.outputFiles[0].text).toString("base64")}`);
};
const Gate = await Load("src/lib/BridgeCommandGate.ts"), Restart = await Load("scripts/BridgeRestart.ts");
const Ready = { SocketOpen: true, BasketCompatible: true, CadCompatible: true, SpeedCompatible: true, Calibrating: false };
Test("old bridge reports the missing basket router, not a missing connection", () => {
  for (const Command of ["send_ik", "move_servo", "go_home"]) {
    const Reason = Gate.CommandBlockReason({ type: Command }, { ...Ready, BasketCompatible: false });
    Assert.match(Reason, /basket avoidance/); Assert.doesNotMatch(Reason, /offline|needs the Python bridge/);
  }
  Assert.match(Gate.CommandBlockReason({ type: "send_ik" }, { ...Ready, CadCompatible: false }), /CAD solver/);
  Assert.match(Gate.CommandBlockReason({ type: "send_ik" }, { ...Ready, SocketOpen: false, BasketCompatible: false }), /offline/);
  Assert.equal(Gate.CommandBlockReason({ type: "send_ik" }, Ready), null);
});
Test("calibration keeps manual motion blocked while stop and disconnect still work", () => {
  for (const Command of ["send_ik", "move_servo", "go_home", "toggle_torque"]) {
    Assert.match(Gate.CommandBlockReason({ type: Command }, { ...Ready, Calibrating: true }), /calibration is running/);
  }
  for (const Command of ["stop_motion", "emergency_stop", "disconnect"]) {
    Assert.equal(Gate.CommandBlockReason({ type: Command }, { ...Ready, BasketCompatible: false, CadCompatible: false, Calibrating: true }), null);
  }
  Assert.equal(Gate.CommandBlockReason({ type: "send_ik", calibration: true }, { ...Ready, Calibrating: true }), null);
});
Test("restart prevents connection races and preserves emergency controls", () => {
  for (const Command of ["connect", "toggle_torque", "send_ik", "move_servo", "go_home"]) {
    Assert.match(Gate.CommandBlockReason({ type: Command }, { ...Ready, Restarting: true }), /update in progress/);
  }
  for (const Command of ["emergency_stop", "disconnect", "stop_motion"]) {
    Assert.equal(Gate.CommandBlockReason({ type: Command }, { ...Ready, Restarting: true }), null);
  }
});
Test("restart uses the guarded script without shell interpolation and propagates refusal", async () => {
  for (const [Code, Output, Expected] of [[0, "Updated bridge loaded. Arm remains disconnected.", null],
    [1, "Bridge restart cancelled: arm is not confirmed disconnected.", /still connected/], [1, "Port 8000 belongs to another application.", /restart failed/]]) {
    const Calls = [], Child = new EventEmitter();
    Child.stdout = new EventEmitter(); Child.stderr = new EventEmitter(); Child.stdin = { end() {} }; Child.kill = () => {};
    // Like the real script: 'exit' only. The relaunched bridge inherits the
    // output pipes, so 'close' does not fire while it runs.
    const Launch = (...Args) => { Calls.push(Args); queueMicrotask(() => { Child.stderr.emit("data", Buffer.from(Output)); Child.emit("exit", Code); }); return Child; };
    const Result = Restart.RunBridgeRestart("C:/Users/me/Code/Folder with spaces/scripts/RestartBridge.ps1", Launch);
    if (Expected) await Assert.rejects(Result, Expected); else await Result;
    Assert.equal(Calls.length, 1); Assert.equal(Calls[0][0], "powershell.exe");
    Assert.equal(Calls[0][1].at(-1), "C:/Users/me/Code/Folder with spaces/scripts/RestartBridge.ps1");
    Assert.deepEqual(Calls[0][2], { windowsHide: true });
  }
});

Test("an old speed scale is visible as a bridge update even with matching CAD and basket support", () => {
  Assert.match(Gate.BridgeUpdateReason(true, true, false), /expanded motion speed range/);
  Assert.equal(Gate.BridgeUpdateReason(true, true, true), null);
});

Test("a bridge running other code than this page expects is named, and AI tasks wait for it (audit 2 H1)", async () => {
  const { readFileSync } = await import("node:fs");
  const Version = JSON.parse(readFileSync("../back end/BridgeVersion.json", "utf8")).Version;
  Assert.equal(Gate.BridgeVersion, Version, "the page reads the bridge's own version file");
  Assert.equal(Gate.BridgeUpdateReason(true, true, true, Version), null);
  // 2026-10-06: every version string matched while the bridge ran day-old code.
  Assert.match(Gate.BridgeUpdateReason(true, true, true, "stage0"), /not the code this page expects \(bridge "stage0", page "/);
  Assert.match(Gate.BridgeUpdateReason(true, true, true, null), /older than the version check/);
  Assert.match(Gate.AiTaskBridgeReason(undefined), /AI tasks wait for the update/);
  Assert.equal(Gate.AiTaskBridgeReason(Version), null);
  // Manual controls keep working on it: Home, Hold, Stop and moves are not blocked by the version.
  for (const Command of ["go_home", "toggle_torque", "stop_motion", "move_servo", "send_ik"]) Assert.equal(Gate.CommandBlockReason({ type: Command }, Ready), null);
  // RestartBridge.ps1 waits for the same version.
  const Script = readFileSync("scripts/RestartBridge.ps1", "utf8");
  Assert.match(Script, /BridgeVersion\.json/);
  Assert.match(Script, /\$Health\.bridge_version -eq \$ExpectedBridgeVersion/);
});

Test("hand guiding holds the moves and lets every stop through (docs/HandGuide.md 6)", () => {
  for (const Command of ["send_ik", "move_servo", "go_home", "set_home"]) {
    Assert.match(Gate.CommandBlockReason({ type: Command }, { ...Ready, HandGuiding: true }), /Hand guiding is on: press Hold here first/);
  }
  for (const Command of ["stop_motion", "emergency_stop", "disconnect", "toggle_torque", "hand_guide", "hand_guide_heartbeat"]) {
    Assert.equal(Gate.CommandBlockReason({ type: Command }, { ...Ready, HandGuiding: true }), null);
  }
});
