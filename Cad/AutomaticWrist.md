# Automatic wrist angle and target scrolling

Manual target placement first preserves the requested XYZ. If the current
wrist pitch cannot reach that position, a worker searches for a valid pitch
close to the requested one. It combines sampled angle ranges with a numerical
solve that allows pitch to vary, including solutions between sampled angles.
Candidates still pass the strict CAD solver, joint limits and basket checks.
Valid pitches stay unchanged. The selected angle appears in the wrist field
and approach arrow and is sent explicitly to Python on Send.

The background guide worker also adjusts pitch during a paused position drag.
On release or Enter, the target worker checks the exact position before
snapping coordinates. If no tested pitch reaches that position, the nearest
position search includes wrist flex as another joint variable. Numerical
search does not establish a mathematical guarantee of the global optimum.

Scroll directly over the target dot to adjust wrist angle. XYZ stays fixed,
including when the requested angle reaches a limit. On release, an invalid
angle resolves to a nearby valid angle or the previous valid target is restored.
The green acceptable-angle guide remains visible and cached during scrolling.
Shift-scroll continues to work elsewhere in the CAD view. Ordinary scrolling
over empty space and Ctrl/pinch gestures still zoom. Escape cancels the gesture.

Search runs outside the rendering thread. Results from superseded edits are
ignored, and accepted candidates are checked against current limits and
alignment. Automatic guide adjustments pause during calibration. Searching,
dragging and scrolling do not issue motor commands. Python still verifies the
chosen pose, encoder rounding and basket route when Send is requested.

Verification: TestAutomaticWrist.mjs, TestAutomaticWristUi.mjs,
TestAutomaticWristBridge.py, TestTargetWorkspace.mjs, TestPitchGuide.mjs,
TestCadKinematics.mjs, TestCadMouseControls.mjs, TestBridgeSendUi.mjs,
TestCalibrationUi.mjs and TestCadPerformance.mjs. Browser tests mock hardware;
the Python check verifies the browser-generated pose after encoder rounding
without connecting to a serial port.
