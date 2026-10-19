export type ViewSettings = {
  ShowGraph: boolean; CoordinateLabels: "Hidden" | "PositiveY" | "All";
  ShowGrid: boolean; AxisWidth: number; GridOpacity: number; ShowOrigin: boolean;
  ShowBody: boolean; ShowParkedArm: boolean; ShowPosePreview: boolean; ShowJointMarkers: boolean;
  ShowApproachRay: boolean; ShowAngleGuide: boolean; ShowViewCube: boolean;
  ShowHints: boolean; Shadows: boolean; ScanLines: boolean; ZoomSpeed: number; OrbitSpeed: number; PanSpeed: number;
  ShowBottomText: boolean; CameraProfile: "Smooth" | "Detail"; RenderQuality: "Ultra" | "Balanced";
  // Auto uses the Orbbec depth camera whenever the bridge is streaming it.
  CameraSource: "Auto" | "Depth" | "Webcam";
  // Where each camera looks from. "Facing" (the camera faces the arm) flips
  // captures left-right so image-left is the arm's left; "Behind" (looking
  // forward over the arm, like the Orbbec on the robot's head) needs no flip.
  DepthCameraView: "Behind" | "Facing"; WebcamView: "Behind" | "Facing";
  DepthOverlay: boolean;
};
export const DefaultViewSettings: ViewSettings = {
  ShowGraph: true, CoordinateLabels: "Hidden", ShowGrid: true, AxisWidth: 3, GridOpacity: .35, ShowOrigin: true,
  ShowBody: true, ShowParkedArm: true, ShowPosePreview: true, ShowJointMarkers: true, ShowApproachRay: true, ShowAngleGuide: true,
  ShowViewCube: true, ShowHints: true, Shadows: true, ScanLines: true, ZoomSpeed: 1, OrbitSpeed: 1, PanSpeed: 1,
  ShowBottomText: true, CameraProfile: "Smooth", RenderQuality: "Ultra", CameraSource: "Auto",
  DepthCameraView: "Behind", WebcamView: "Facing", DepthOverlay: false,
};
export const ViewSettingsStorageKey = "SO101.ViewSettings.v1";

export function NormalizeViewSettings(Value: unknown): ViewSettings {
  const Result = { ...DefaultViewSettings };
  if (Value === null || typeof Value !== "object" || Array.isArray(Value)) return Result;
  const Saved = Value as Record<string, unknown>;
  for (const Key of ["ShowGraph", "ShowGrid", "ShowOrigin", "ShowBody", "ShowParkedArm", "ShowPosePreview",
    "ShowJointMarkers", "ShowApproachRay", "ShowAngleGuide", "ShowViewCube", "ShowHints", "Shadows",
    "ScanLines", "ShowBottomText", "DepthOverlay"] as const) {
    if (typeof Saved[Key] === "boolean") Result[Key] = Saved[Key];
  }
  if (Saved.CoordinateLabels === "Hidden" || Saved.CoordinateLabels === "PositiveY" || Saved.CoordinateLabels === "All") Result.CoordinateLabels = Saved.CoordinateLabels;
  if (Saved.CameraProfile === "Smooth" || Saved.CameraProfile === "Detail") Result.CameraProfile = Saved.CameraProfile;
  if (Saved.RenderQuality === "Ultra" || Saved.RenderQuality === "Balanced") Result.RenderQuality = Saved.RenderQuality;
  if (Saved.CameraSource === "Auto" || Saved.CameraSource === "Depth" || Saved.CameraSource === "Webcam") Result.CameraSource = Saved.CameraSource;
  for (const Key of ["DepthCameraView", "WebcamView"] as const) {
    if (Saved[Key] === "Behind" || Saved[Key] === "Facing") Result[Key] = Saved[Key];
  }
  for (const [Key, Min, Max] of [["AxisWidth", 1, 6], ["GridOpacity", .1, .8],
    ["ZoomSpeed", .25, 2], ["OrbitSpeed", .25, 2], ["PanSpeed", .25, 2]] as const) {
    const NumberValue = Saved[Key];
    if (typeof NumberValue === "number" && Number.isFinite(NumberValue)) Result[Key] = Math.max(Min, Math.min(Max, NumberValue));
  }
  return Result;
}

export function ReadViewSettings(): ViewSettings {
  try { return NormalizeViewSettings(JSON.parse(localStorage.getItem(ViewSettingsStorageKey) ?? "null")); }
  catch { return { ...DefaultViewSettings }; }
}

export function SaveViewSettings(Settings: ViewSettings) {
  try { localStorage.setItem(ViewSettingsStorageKey, JSON.stringify(Settings)); }
  catch { /* Display controls still work if browser storage is unavailable. */ }
}
