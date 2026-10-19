# Robot cockpit completion audit

Audited against the user's full sequence of requests before implementation.

Already present: collapsible sections, resizable columns, Manual/AI modes,
reported AI usage, retro speed throttle, target screen-plane dragging,
basket avoidance, grayed inactive arm, view cube, approach ray, green wrist
guide, and the new persistent display settings with an unlabeled thick graph.

Outstanding at audit:

- Joint selection does not highlight the physical joint or show a rotation
  ring. Joint controls do not expand into a precision editor.
- Target coordinate focus does not highlight its world axis.
- Dashboard styling still resembles separate forms rather than a robot HUD.
- Bottom CAD text is only partially covered by Show control hints. The user
  asked for all bottom text to be off for their browser, with defaults still on.
- The bottom status ticker is still present; messages need a top-right home
  and an approximately ten-second lifetime.
- Camera uses a basic video element and unspecified frame rate. There are no
  frame-delivery statistics, recovery controls, or a call-style presentation.
- Cart uses imported mesh normals and limited rendering resolution. Surface
  shading and available geometry need inspection before claiming more detail.
- Protection faults stop motion and retain holding torque. The requested
  automatic coast needs a latched trip, verification and deliberate recovery.

Implementation must preserve target validation, basket collision checks,
recording/calibration captures, keyboard access, reduced-motion preferences,
and the emergency control. Tests must use a fake motor bus and intercepted
browser traffic. Hardware protection must not be reported as active until the
running bridge version is verified.

## Completion

Implemented and covered by offline tests: joint row/mesh selection, calibrated
joint arcs and precision dial, target-axis focus guides, Roboto cockpit styling,
full bottom-text visibility toggle and separate local off preference, timed
top-right notifications, memoized call-style camera with freshness/recovery,
Ultra rendering and cart-normal smoothing, and latched automatic coast with
read-back confirmation. Restored explicit Disconnect with persistent reconnect
pause after discovering the old UI had removed it.

The physical bridge was last observed running the older protection code with
torque on and a wrist-flex feedback fault. The code is ready; loading it remains
pending supported-arm disconnection. No test opened or moved the physical arm.
The live webcam could not be inspected through the available browser connection;
camera delivery/recovery tests used a synthetic camera.
