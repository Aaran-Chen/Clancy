# CAD display settings

Open Settings at the bottom of the left mode rail. The drawer stays open
while using the dashboard, closes with its close button or Escape, and saves
display preferences in this browser. Manual and AI modes share these settings.

The coordinate graph defaults to three 3-pixel colored axes with no labels.
Red is X, green is Y, and blue is Z. Small arrowheads mark the positive ends;
positive Y still faces the robot's front. The origin remains between the arm
mounts at bin-rim height. Coordinate labels can be hidden, show only +Y, or
show +X, +Y and +Z. The former origin badge has been removed.

Settings provides graph visibility, axis thickness, origin-point visibility,
floor-grid visibility and opacity. Hiding the graph hides its axes, labels,
origin point and floor grid together. The approach ray and green wrist-angle
guide have separate visibility controls and remain enabled by default.

Other controls show or hide the robot body and basket, inactive arm, target
and AI pose previews, view cube, control hints and shadows. Zoom, orbit and
pan each have an adjustable sensitivity. These are display preferences;
hiding the basket does not alter collision checks or permit motion through it.
Changing display settings preserves the current camera view. Fit uses the
visible assembly when explicitly selected.

Reset display settings restores display defaults without resetting dashboard
widths, collapsed sections, calibration or motor settings. Malformed saved
values fall back to defaults or are clamped to the supported range. If browser
storage is unavailable, the controls still work for the current session.

Offline verification, from AI Controlled WebApp:

```powershell
node TestViewSettings.mjs
```

```powershell
node TestSettingsUi.mjs
```

The browser test checks actual scene visibility and line width, positive-Y
direction, unchanged camera and origin, reusable geometry, every setting,
mode/reload persistence, reset and narrow-window layout. It intercepts bridge
and model traffic and asserts that no hardware commands were sent.
