import BridgeVersionFile from "../../../back end/BridgeVersion.json";
import { BlockedWhileGuiding } from "./HandGuide";

// HandGuiding: hand guiding or its calibration runs (the bridge's hand_guide): moves wait for Hold here.
type CommandContext = { SocketOpen: boolean; BasketCompatible: boolean; CadCompatible: boolean; SpeedCompatible: boolean; Calibrating: boolean; Restarting?: boolean;
  HandGuiding?: boolean };

// The bridge code this page was built against (back end/BridgeVersion.json, which the bridge reads too and
// reports as bridge_version). 2026-10-06 the bridge ran day-old code (8f4d27a) under this page for a day and
// every older version string matched, so the scene layer and the tip check silently did not run.
export const BridgeVersion: string = BridgeVersionFile.Version;

/**
 * Why the running bridge must be reloaded before this page relies on it, or null. Bridge: the bridge_version its
 * state reports (null: a bridge older than the version contract itself; left out: not judged).
 */
export function BridgeUpdateReason(BasketCompatible: boolean, CadCompatible: boolean, SpeedCompatible = true,
  Bridge: string | null | undefined = BridgeVersion): string | null {
  if (!BasketCompatible) return "Bridge update required: basket avoidance and robot body protection are not loaded.";
  if (!CadCompatible) return "Bridge update required: the CAD solver is not compatible.";
  if (!SpeedCompatible) return "Bridge update required: the expanded motion speed range is not loaded.";
  if (Bridge !== BridgeVersion) {
    return `Bridge update required: the running bridge is not the code this page expects (bridge ${Bridge ? `"${Bridge}"` : "older than the version check"}, page "${BridgeVersion}"). AI tasks wait for the update; Home, Hold, Stop and manual moves still work.`;
  }
  return null;
}

/** Why an AI task may not start on this bridge, or null: it relies on the bridge's scene layer, tip check and hold rules. */
export function AiTaskBridgeReason(Bridge: string | null | undefined): string | null {
  return Bridge === BridgeVersion ? null : BridgeUpdateReason(true, true, true, Bridge ?? null);
}

export function CommandBlockReason(Message: Record<string, unknown>, Context: CommandContext): string | null {
  if (!Context.SocketOpen) return "Python bridge connection is offline. Reconnect the bridge before sending commands.";
  const Command = String(Message.type);
  if (Context.Restarting && !["stop_motion", "emergency_stop", "disconnect"].includes(Command)) return "Bridge update in progress. Wait for it to finish before connecting or sending commands.";
  if (Context.Calibrating && !Message.calibration && !["stop_motion", "emergency_stop", "disconnect", "set_motion_speed"].includes(Command)) {
    return "Automatic calibration is running. Stop calibration before sending manual commands.";
  }
  if (Context.HandGuiding && BlockedWhileGuiding(Command)) return "Hand guiding is on: press Hold here first.";
  if (["send_ik", "move_servo", "go_home"].includes(Command) && !Context.BasketCompatible) return BridgeUpdateReason(false, Context.CadCompatible);
  if (Command === "send_ik" && !Context.CadCompatible) return BridgeUpdateReason(true, false);
  if (Command === "set_motion_speed" && !Context.SpeedCompatible) return "Bridge update required: speed control is not loaded.";
  return null;
}
