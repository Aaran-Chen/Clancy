# Dashboard modes and tiling

The left rail selects Manual or AI control. Manual shows CAD, joints
and target/motion controls. The camera floats above the dashboard without
occupying a column. AI uses two columns: assistant, usage statistics, current
position and motion controls on the left; CAD on the right. Manual Target and
Send controls are hidden in AI mode. Its CAD target displays the proposed AI
destination and cannot edit the hidden manual target. Invalid proposals remain
visibly invalid and require a valid proposal before approval. The redundant
Status section is removed from both modes. The camera and CAD remain mounted
when changing modes. Switching to Manual cancels pending AI requests and
follow-ups; it does not change torque or stop an already approved arm move.
Use Stop and hold for that. Emergency stop remains in the top bar.

Click a section heading to minimize or restore it. Each mode remembers its
own minimized sections and column widths in local browser storage. Session
usage totals reset on page reload. Missing or blocked browser storage falls
back to a usable layout.

The floating camera shares its size, position and minimized state across modes.
Drag its title bar or use arrow keys to move it. Drag its lower-right corner to
resize it. Escape cancels a drag. The header contains only Live view,
fullscreen and minimize. Fullscreen fills the dashboard beneath the emergency
bar; Escape restores the window. Minimize creates a draggable 48-pixel square
without stopping capture; clicking restores the remembered window dimensions.
Reconnect, Calibrate and native picture-in-picture (Float above other apps)
are in Settings. Existing four-column Manual and
three-column AI preferences return the removed pane space to CAD.

Play by Jonas Hecksher is bundled locally in regular and bold weights, including
its OFL license at public/Fonts/Play-LICENSE.txt. No external font service is needed.

The view cube starts in the CAD top-left corner. Click a face to change view;
drag the cube or a face to reposition it without rotating the camera. Position
persists and stays inside CAD after resizing. Arrow keys move the cube, Home
resets its position, and Escape cancels a drag. The ISO button is removed;
Fit model in view remains available in the CAD toolbar.

Drag a vertical divider to resize its two neighboring columns. The divider
also supports arrow keys, Shift-arrow for larger steps, and Home/End for the
limits. Escape cancels a drag. Double-clicking a divider or pressing Reset
widths restores that mode's default widths. Narrow windows allow horizontal
scrolling instead of hiding the arm controls. CAD pan, zoom and orientation
are preserved during resizing and mode changes.

AI Statistics reads input, output and total tokens from completed model API
responses. Revisions count separately and contribute to reported session
totals. The panel shows request duration, response and failure counts, and the
model's reported context window. Unknown token usage is shown as unavailable,
never estimated from text. Deliberately cancelled requests are not failures.

Offline verification, from AI Controlled WebApp:

```powershell
node TestDashboardLogic.mjs
```

```powershell
node TestDashboardUi.mjs
```

The UI test intercepts every bridge and model request; it never controls the
physical arm or invokes a real model.

## Bridge updates and Send diagnostics

Send distinguishes an offline socket, a connected bridge missing basket
avoidance or the CAD solver, an active calibration, a disconnected arm, and
torque being off. A rejected command retains its specific reason rather than
being replaced by a generic missing-Python message. Basket and CAD compatibility
remain mandatory for Cartesian motion.

If the connected process is outdated, Motion displays the missing capability
and a Load bridge update button. Support the arm and click Disconnect first;
disconnecting releases torque. The update button and top-bar Restart bridge
now call the actual guarded Python restart instead of only reopening a socket.
The server rechecks connection and torque state before replacing the process.
Connect and motion commands are blocked during replacement; emergency stop
and disconnect remain available. Automatic reconnection stays paused afterward,
and no failed Send is queued or replayed. Connect deliberately when ready.

Regression coverage: TestBridgeCommandGate.mjs and TestBridgeSendUi.mjs use
mocked process launching and hardware traffic to verify old-bridge rejection,
explicit restart refusal, update-handshake recovery, connection races and a
successful explicit Send only after the simulated bridge is compatible.


UI coverage: TestFloatingCamera.mjs checks Play loading, all six cube faces,
face dragging without view changes, saved positions, compact camera tiles,
stream identity, two-column AI mode and AI target isolation. TestRobotHud.mjs,
TestDashboardUi.mjs and TestCalibrationUi.mjs cover retained joint controls,
layout persistence, camera recovery through Settings and calibration access.
