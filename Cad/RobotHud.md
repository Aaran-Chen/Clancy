# Synthwave robot dashboard

The dashboard is synthwave (src/components/SynthwaveTheme.css): readable Play
typography, chrome titles in capitals, neon edges, a striped-sun logo, and in the
empty space at the foot of the side columns a perspective grid receding to a
striped sun, painted behind the opaque panels. Each mode has its own theme. AI is
purple: deep indigo and violet surfaces with hot magenta and sunset orange as
decoration. Manual is blue throughout: navy surfaces, electric-blue hairlines and
headings, a blue grid and a cyan sun, blue chrome, the heads-up parts in blue and
cyan, and in the 3D view a navy cabin over an electric-blue grid with the live arm
in cyan. Status colours keep their meaning in both: green for ready and online,
amber for caution, red-orange for the alarm and a solid red emergency stop with
white capitals, so none reads as decoration or as a mode colour. Every text colour
keeps at least the contrast it had before. The 3D view starts at neither mode's
colour, so its first frame and every mode switch apply the right tones. It is all
static and paint only. The workspace and camera have no film, scanline overlay or
glow, and nothing is drawn under text. The saved Scan lines setting adds a faint
texture inside the AI companion's portrait tile and over the empty backdrop horizon.

Manual mode reads as a heads-up display (src/components/ManualHud.css): bracketed
frames; a B0T DIAGNOSTICS strip at the top of Joints (BotDiagnostics.tsx: the arm
drawn side-on from its forward kinematics, live pose solid and commanded pose
dashed, beside each servo's motor-guard temperature); each joint a tape (J
number, angle, raw value; a blue fill and a cyan caret at the live position, the
native slider's thumb as the target; a monospace line with the target raw, the
temperature and the target-minus-live offset); a plan-view radar with a height
ladder over Current Position (TipRadar.tsx); the speed as an LED bar; and the
motion buttons as coded keys (EXEC, PWR, RTB, MEM, BRK, aria-hidden), with Stop
and hold in amber. Below the keys a reserved, dimmed slot (data-slot="hand-guide")
holds the place for the Hand guide toggle and "Calibrate hand guiding"; put those
controls inside it and the placeholder disappears. Every control is the same
element with the same name, role, keyboard and command path as before; the
diagram, radar and codes are decoration. Shorter windows drop the diagram, the
radar, the codes and the reserved row first, so 1280x720 keeps its old density,
and below the tallest windows a selected joint's precision dial takes the
diagram's place.

AI Control comes before AI Statistics. Its small vector pixel robot reflects
actual model availability, requests, proposal review, movement, settling and
motor faults. The portrait is decorative; the adjacent text announces the
state accessibly. Eye blinks and thinking animation respect reduced motion.
An idle AI panel shows an approval placeholder instead of presenting current
coordinates as a proposed movement. Proposal approval and all motor controls
continue through the existing command checks.

The theme lives in src/components/ArcadeHud.css, SynthwaveTheme.css and
ManualHud.css; src/styles.css retains the base layout. AiCompanion.tsx supplies the pixel art without an image request,
canvas loop or extra rendering dependencies. Existing tiling, section
minimization, Play font, floating camera and view cube remain available.

Both arms in the viewport carry clickable joint handles: a filled dot per joint
on the live arm, a ring per joint on the target pose preview, sized to a steady
16 screen pixels and picked within 22 pixels of their centre. Hovering one
reports it on the canvas (data-hovered-joint) and shows a pointer cursor;
clicking selects that joint, and clicking it again clears the selection. The
target ball keeps the top of the pick order, so a handle behind it never steals
a drag. Handles can be switched off in Settings (Show joint handles).

Select a joint by clicking its handle, its row or name, or a visible part of the
active arm in the CAD view. The corresponding link lights up, and a green arc
with an amber current-angle spoke appears around the actual calibrated joint
axis. The arc shows configured joint limits; basket collision checks still
decide whether a proposed motion is allowed. The inactive arm cannot be selected.

In Manual mode the selected joint's arc in the 3D view is also a dial: a knob on
the arc marks the joint's target. Dragging the knob, the arc or the selected
handle turns the joint by how far the pointer rotates around its axis (relative,
so grabbing never jumps), through the same updateServo path as the side-panel
slider: the same gates, joint limits (it stops exactly at a limit), 90 ms command
throttle and motion speed. It moves the real arm live during the drag; nothing is
sent until the pointer has moved 4 px, and Escape, pointer cancellation or window
blur end it. Dragging empty space still pans and orbits. The dial is drawn at
least 44 screen pixels in radius.

The selected row expands into a precision dial. Dragging the dial previews its
value locally and applies it on release. Escape, pointer cancellation and loss
of window focus cancel the drag. Arrow keys or the +/- buttons adjust one raw
encoder step; Shift-arrow adjusts ten. Clicking or selecting alone never sends
a motor command (dragging the 3D dial does, as above). Existing live sliders still use their original motion path;
their rows expand after the click, avoiding a layout shift during pointer drag.

Focusing a target X, Y or Z input highlights the target and draws that colored
world axis through it. The target stays on the original validated edit/snap path.
The approach ray and green wrist guide remain independent of the coordinate graph.

Settings includes Show bottom visualization text, which hides all text along the
bottom of the CAD view. Its distributable default is on. The ignored local file
public/LocalDisplayPreferences.json applies Aaran's off preference once per
browser. Later changes and Reset are respected, including after reload.

System updates appear at the upper right, below the CAD tools, with individual ten-second timers and dismiss controls. Up to three recent updates are shown together;
new updates can displace older ones during bursts. Active protection faults
remain visible in the Motion panel after notifications expire.

The floating Live Feed window presents Roboto as a video call, with a connection indicator,
measured frame rate/resolution, dropped-frame counter, reconnect button and
expand/minimize control. Expanding or switching Manual/AI preserves the video
element and stream. Camera status is no longer declared ready if frames stall.
Profile changes reconnect the camera and are disabled during calibration and
task recording. Smooth requests 720p/60 fps; Detail requests 1080p/60 fps. The
camera can negotiate lower values; the window reports what is actually delivered.

Drag the camera title bar to move it, or use the lower-right grip to resize it.
Arrow keys also move and resize, with Shift for larger steps. Escape cancels
an active drag. Position, size and minimized state persist in local storage;
viewport changes keep the title bar reachable. The window stays above panels
and follows CAD fullscreen without replacing its video element or stream.
The top emergency bar stays accessible. The camera pop-out button opens native
picture-in-picture above other applications; returning keeps the same stream.
Camera calibration is accessible through the title-bar crosshair button.

The CAD view cube now sits in the upper-left corner. Its faces retain their
3D transforms during pointer presses, and all six face presets work, including
the bottom view. Choosing a preset clears residual camera inertia. The old
XLeRobot/CAD/reference/live-arm heading is removed; reference alignment remains
available as a crosshair icon in the upper-right CAD tools.

Camera acquisition and statistics are isolated in a memoized component. Frame
callbacks do no image encoding or whole-app React updates. Statistics update
once per second. Existing archival calibration frames and task recording remain.

Ultra rendering uses supersampling and 4096-pixel shadow maps. Cart normals are
smoothed across curved tessellation seams while sharp edges and vertex positions
are preserved. The local cart source is an STL; there is no finer cart STEP solid
to retessellate. Ultra improves rendering of the available geometry without
inventing geometry or changing collision bounds. Balanced renders at native
resolution. The test on the RTX 5090 measured a 7.5 ms median orbit frame interval,
zero idle renders and zero repeated idle IK calls.

Disconnect is available in the top bar when connected. It cancels pending AI and
servo commands and pauses automatic reconnection, even through a reload. Connect
deliberately resumes it. Support the arm before disconnecting because torque is
released. The emergency button remains available above the camera expansion.

Verification: TestJointHandles.mjs, TestJointInteraction.mjs, TestRobotHud.mjs, TestSettingsUi.mjs,
TestCadMouseControls.mjs, TestDashboardUi.mjs, TestCalibrationUi.mjs,
TestCadPerformance.mjs and the frontend build. Browser tests intercept every
motor/model request and use a synthetic camera where needed. Physical camera
frame delivery must not be inferred from the synthetic camera result.

Floating-window verification: TestFloatingCamera.mjs, TestCameraWindowBounds.mjs,
TestDashboardLogic.mjs, TestDashboardUi.mjs, TestCadUi.mjs,
TestCadMouseControls.mjs and TestCalibrationUi.mjs. The browser test checks real
pointer presses on all six cube faces, drag/resize cancellation, storage,
minimized frame readiness, mode/fullscreen stream identity, native PiP entry
and exit, viewport bounds and absence of motor commands.
