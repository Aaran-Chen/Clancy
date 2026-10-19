import { useEffect, useRef } from "react";
import { Settings2, X } from "lucide-react";
import { DefaultViewSettings, type ViewSettings } from "../lib/ViewSettings";
import { DefaultGraspGapCm, GraspGapRange } from "../lib/GraspGap";
import "./SettingsPanel.css";

// GraspGapCm: the grasp gap at the fixed finger (lib/GraspGap.ts), not a display preference: Reset display settings keeps it.
type SettingsPanelProps = { Settings: ViewSettings; CameraLocked?: boolean; OnChange: (Patch: Partial<ViewSettings>) => void; OnClose: () => void;
  CameraStatus: string; DepthCameraStatus?: string; OnReconnectCamera: () => void; OnCalibrate: () => void; OnPopoutCamera?: () => void;
  GraspGapCm?: number; OnGraspGapChange?: (Cm: number) => void; OnForgetRemotePassword?: () => void };
type ToggleKey = { [Key in keyof ViewSettings]: ViewSettings[Key] extends boolean ? Key : never }[keyof ViewSettings];
type RangeKey = { [Key in keyof ViewSettings]: ViewSettings[Key] extends number ? Key : never }[keyof ViewSettings];

export default function SettingsPanel({ Settings, CameraLocked, OnChange, OnClose, CameraStatus, DepthCameraStatus, OnReconnectCamera, OnCalibrate, OnPopoutCamera,
  GraspGapCm, OnGraspGapChange, OnForgetRemotePassword }: SettingsPanelProps) {
  const PanelRef = useRef<HTMLElement>(null);
  const CloseRef = useRef<HTMLButtonElement>(null);
  const CloseLatest = useRef(OnClose); CloseLatest.current = OnClose;
  useEffect(() => {
    const PreviousFocus = document.activeElement as HTMLElement | null;
    const Panel = PanelRef.current;
    CloseRef.current?.focus();
    const Escape = (Event: KeyboardEvent) => { if (Event.key === "Escape") CloseLatest.current(); };
    window.addEventListener("keydown", Escape);
    return () => {
      window.removeEventListener("keydown", Escape);
      if (document.activeElement === document.body || Panel?.contains(document.activeElement)) PreviousFocus?.focus();
    };
  }, []);
  const Toggle = (Key: ToggleKey, Label: string, Detail?: string, Disabled = false) => (
    <label className="settings-toggle"><span>{Label}{Detail && <small>{Detail}</small>}</span>
      <input type="checkbox" aria-label={Label} checked={Settings[Key]} disabled={Disabled} onChange={Event => OnChange({ [Key]: Event.currentTarget.checked })} />
    </label>
  );
  const Range = (Key: RangeKey, Label: string, Min: number, Max: number, Step: number, Unit: string, Disabled = false) => (
    <label className="settings-range"><span>{Label}<output>{Settings[Key]}{Unit}</output></span>
      <input type="range" min={Min} max={Max} step={Step} value={Settings[Key]} aria-label={Label} disabled={Disabled}
        onChange={Event => OnChange({ [Key]: Number(Event.currentTarget.value) })} />
    </label>
  );
  return (
    <aside ref={PanelRef} id="dashboard-settings" className="settings-panel" role="dialog" aria-modal="false" aria-labelledby="settings-title">
      <div className="settings-heading"><Settings2 size={18} /><h2 id="settings-title">Settings</h2>
        <button ref={CloseRef} type="button" aria-label="Close settings" onClick={OnClose}><X size={18} /></button>
      </div>
      <div className="settings-content">
        <p className="settings-intro">Display preferences are saved in this browser.</p>
        <fieldset><legend>Coordinate graph</legend>
          {Toggle("ShowGraph", "Show graph", "Axes and floor grid")}
          <label className="settings-select"><span>Coordinate labels</span>
            <select aria-label="Coordinate labels" value={Settings.CoordinateLabels} disabled={!Settings.ShowGraph} onChange={Event => OnChange({ CoordinateLabels: Event.currentTarget.value as ViewSettings["CoordinateLabels"] })}>
              <option value="Hidden">Hidden</option><option value="PositiveY">Positive Y only</option><option value="All">X, Y and Z</option>
            </select>
          </label>
          {Range("AxisWidth", "Axis line thickness", 1, 6, .5, " px", !Settings.ShowGraph)}
          {Toggle("ShowOrigin", "Show origin point", undefined, !Settings.ShowGraph)}
          {Toggle("ShowGrid", "Show floor grid", undefined, !Settings.ShowGraph)}
          {Range("GridOpacity", "Grid opacity", .1, .8, .05, "", !Settings.ShowGraph || !Settings.ShowGrid)}
          <p className="settings-note">Red X, green Y, blue Z. Positive Y faces the front of the robot.</p>
        </fieldset>
        <fieldset><legend>Robot and target</legend>
          {Toggle("ShowBody", "Show robot body and basket")}
          {Toggle("ShowParkedArm", "Show inactive arm")}
          {Toggle("ShowPosePreview", "Show target pose preview")}
          {Toggle("ShowJointMarkers", "Show joint handles", "Click a joint on the live or target arm")}
          {Toggle("ScanLines", "Scan lines", "Subtle texture on the AI portrait and backdrop; camera and workspace stay clear")}
          {Toggle("ShowApproachRay", "Show approach ray")}
          {Toggle("ShowAngleGuide", "Show acceptable-angle indicator", "The green arc around the target")}
          <p className="settings-note">Visibility changes only the view. Basket avoidance stays active.</p>
        </fieldset>
        {GraspGapCm != null && OnGraspGapChange && <fieldset><legend>Grasp</legend>
          <label className="settings-range"><span>Grasp gap at the fixed finger<output>{GraspGapCm.toFixed(1)} cm</output></span>
            <input type="range" min={GraspGapRange.MinCm} max={GraspGapRange.MaxCm} step={GraspGapRange.StepCm} value={GraspGapCm} aria-label="Grasp gap at the fixed finger"
              onChange={Event => OnGraspGapChange(Number(Event.currentTarget.value))} />
          </label>
          <p className="settings-note">How far from the object the grasp plan puts the fixed (red-taped) jaw: smaller brings it nearer. Default {DefaultGraspGapCm.toFixed(1)} cm.
            Below the default it applies to round objects (cans, bottles) only: their plan corrects for where the real finger stands and checks the way down
            with it nearer still. Other objects keep {DefaultGraspGapCm.toFixed(1)} cm. A gap too small for an object is refused with the reason. Applies from the next task you send.</p>
        </fieldset>}
        <fieldset><legend>Camera and appearance</legend>
          {Toggle("ShowBottomText", "Show bottom visualization text", "All legends, instructions and context at the bottom")}
          {Toggle("ShowViewCube", "Show view cube")}
          {Toggle("ShowHints", "Show control hints")}
          {Toggle("Shadows", "Show shadows")}
          <label className="settings-select"><span>Render quality</span><select aria-label="Render quality" value={Settings.RenderQuality} onChange={Event => OnChange({ RenderQuality: Event.currentTarget.value as ViewSettings["RenderQuality"] })}>
            <option value="Ultra">Ultra · supersampled</option><option value="Balanced">Balanced · native resolution</option></select></label>
          <label className="settings-select"><span>Camera source</span><select aria-label="Camera source" disabled={CameraLocked} value={Settings.CameraSource} onChange={Event => OnChange({ CameraSource: Event.currentTarget.value as ViewSettings["CameraSource"] })}>
            <option value="Auto">Auto · depth camera when streaming</option><option value="Depth">Depth camera (Orbbec)</option><option value="Webcam">Webcam</option></select></label>
          {DepthCameraStatus && <p className="settings-note" role="status">{DepthCameraStatus}</p>}
          {Toggle("DepthOverlay", "Depth overlay on the live view", "Blue near, red far; depth camera only. Never sent to the AI.")}
          <label className="settings-select"><span>Depth camera view</span><select aria-label="Depth camera view" disabled={CameraLocked} value={Settings.DepthCameraView} onChange={Event => OnChange({ DepthCameraView: Event.currentTarget.value as ViewSettings["DepthCameraView"] })}>
            <option value="Behind">Behind the arm, looking forward</option><option value="Facing">Facing the arm</option></select></label>
          <label className="settings-select"><span>Webcam view</span><select aria-label="Webcam view" disabled={CameraLocked} value={Settings.WebcamView} onChange={Event => OnChange({ WebcamView: Event.currentTarget.value as ViewSettings["WebcamView"] })}>
            <option value="Facing">Facing the arm</option><option value="Behind">Behind the arm, looking forward</option></select></label>
          <p className="settings-note">View sets which way the AI's image is turned so its left is the arm's left. Recalibrate after moving a camera.</p>
          <label className="settings-select"><span>Camera profile</span><select aria-label="Camera profile" disabled={CameraLocked} value={Settings.CameraProfile} onChange={Event => OnChange({ CameraProfile: Event.currentTarget.value as ViewSettings["CameraProfile"] })}>
            <option value="Smooth">Smooth · prefer 720p / 60 fps</option><option value="Detail">Detail · prefer 1080p / 60 fps</option></select></label>
          <p className="settings-note">Profile applies to webcams; the depth camera always streams 1280x720. Changing either reconnects the camera; unavailable during calibration or recording.</p>
          <p className="settings-note" role="status">{CameraStatus}</p>
          <div className="button-grid">
            <button type="button" className="action-button" disabled={CameraLocked} onClick={OnReconnectCamera}>Reconnect camera</button>
            <button type="button" className="action-button" onClick={OnCalibrate}>Calibrate</button>
            {OnPopoutCamera && <button type="button" className="action-button" onClick={OnPopoutCamera}>Float above other apps</button>}
          </div>
          {Range("ZoomSpeed", "Zoom sensitivity", .25, 2, .05, "x")}
          {Range("OrbitSpeed", "Orbit sensitivity", .25, 2, .05, "x")}
          {Range("PanSpeed", "Pan sensitivity", .25, 2, .05, "x")}
        </fieldset>
        {OnForgetRemotePassword && <fieldset><legend>This device</legend>
          <p className="settings-note">This browser keeps the arm password you entered for the robot's PC. Forget it on a device others use.</p>
          <div className="button-grid">
            <button type="button" className="action-button" onClick={OnForgetRemotePassword}>Forget the arm password</button>
          </div>
        </fieldset>}
        <button type="button" className="settings-reset" onClick={() => OnChange({ ...DefaultViewSettings })}>Reset display settings</button>
      </div>
    </aside>
  );
}
