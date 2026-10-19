# Clancy

Control stack for a 6-joint **SO-101 / LeRobot SO-100** robotic arm. A browser UI
drives the arm either by hand (per-joint sliders, XYZ target) or by chatting with
a local AI model that proposes motions you approve before anything moves.

## About this public repository

Clancy is a whole XLeRobot: an SO-101 arm on a cart with an Orbbec depth camera, driven by a local Ollama vision model
through the web app and the Python bridge below. This repository is code only. Images, recorded and measured data
(camera calibration, servo backups, recorded poses and replay fixtures) and the generated display meshes are left out.
The collision surfaces and a home pose are included, so the app builds and the bridge starts from a fresh clone. Before
using it on your own arm:

1. **Set your own Home.** `back end/home_preset.json` is the home pose of the arm this was built on, in raw servo
   steps. It is not a pose for your arm: put your arm in its home pose and press **Set Home** (which rewrites the file)
   before you ever press **Home**.
2. **Calibrate your camera** (`back end/camera_calibration.json`) with the app's calibration.
3. **Rebuild the display meshes** for the 3D view with the scripts in `Cad/` (see `Cad/ReadMe.md`), from the
   repository root: `pip install -r Cad/Requirements.txt`, then `python Cad/FetchSources.py` (downloads the pinned
   SO-ARM100 files to `~/Downloads/SO101Cad`), `python Cad/PrepareGuiAssets.py` (`SO101Parts.glb`), and with a clone of
   [Vector-Wangel/XLeRobot](https://github.com/Vector-Wangel/XLeRobot) at `../XLeRobot-main`,
   `python Cad/PrepareXLeRobotAssets.py` (`XLeRobotBody.glb`).

The replay tests that read recorded frames (`back end/TestFixtures/`, `back end/captures/`) report them missing.

---

## What's here

```
SO101-Inverse-Kinematics-GUI/
├── RunAllTests.py            every test suite, offline, one command (see Testing)
├── AI Controlled WebApp/     ← the app you actually run (React + Vite)
│   ├── src/App.tsx            UI, AI conversation loop, bridge connection
│   ├── src/components/        ArmCanvas (3D view), ServoPanel (sliders)
│   ├── src/lib/robotModel.ts  browser-side kinematics + frame conversions
│   ├── scripts/run-app.ps1    one-command launcher (bridge + web app)
│   ├── UiTestLib.mjs          the browser suites' own test server (5191-5199) and firewall
│   └── docs/HOW_IT_WORKS.txt  the deep, top-to-bottom walkthrough
│
└── back end/                 ← Python, talks to the hardware
    ├── bridge_server.py       FastAPI + WebSocket bridge (port 8000)
    ├── feetech_simple.py      Feetech STS3215 serial driver
    ├── arm_geometry.py        forward kinematics + IK entry point
    ├── CadKinematics.py       CAD-based FK and exact position/pitch IK
    ├── BasketCollision.py     cart, mast, body and rearward checks; route planner
    ├── RouteCheckPool.py      the route's collision checks in worker processes, side by side
    ├── calibration.py         camera pixel → table plane mapping
    ├── OrbbecCamera.py        Orbbec Gemini 335L service: color + aligned depth
    ├── DepthGeometry.py       depth → 3D objects, depth-aware camera pose fit
    ├── object_locator.py      VLM object grounding via Ollama
    ├── OllamaOptions.json     num_ctx / num_batch / keep_alive every Ollama call shares
    ├── MotorProtection.md     what trips servo protection, and the grasp hold
    ├── SessionRecorder.py     session video while an AI task runs (its own process, port 8002)
    ├── VideoRetention.py      keeps that video 7 days: deletes older video, nothing else
    ├── KeepVideo.py           marks video to keep for good (around the servo faults, or times you name)
    ├── LogRotation.py         size-capped logs, a few old ones kept (bridge, recorder, camera SDK)
    ├── SessionLog.py          records the bridge's own model calls (grounding)
    ├── ReviewRecording.py     look back: timeline, frame at a time, contact sheet
    ├── ScanBus.py             servo bus diagnostic + ID fixer (duplicate-ID symptoms, --set-id)
    ├── SelfCalibration.py     automatic camera calibration from a marker on the gripper
    ├── TestSelfCalibration.py synthetic end-to-end tests for it (no hardware)
    ├── RecordedCaptures.py    where the replay tests find recordings; a missing one fails them
    ├── TestFixtures/          the few recorded frames a replay keeps in git
    ├── Depth Server.py        UniDepth depth estimate for webcams (port 8001)
    ├── test_bridge_home.py    Home-path regression tests
    ├── captures/<date>/       archive: the frames the AI saw (.jpg) and, per AI task, a clip of
    │                          the live view (.webm + sidecar .json with the prompt; kept 7 days)
    └── recorded_poses/        archived pose captures from the old desktop GUI
```

Two earlier frontends (a pygame desktop GUI and a manual-only web app) were
removed once the AI web app superseded them. They are in git history if needed.

---

## Running it

```powershell
npm install --prefix "AI Controlled WebApp"
```

```powershell
npm run dev --prefix "AI Controlled WebApp"
```

That starts the Python bridge and the web app together and opens
<http://127.0.0.1:5181>. `Ctrl+C` stops both.

Web app only, no hardware:

```powershell
npm run dev:web --prefix "AI Controlled WebApp"
```

It serves <http://127.0.0.1:5181> too (`server.port` in `vite.config.ts`, with
`strictPort`: when 5181 is taken it stops instead of moving to another port,
whose page the bridge would refuse).

### Prerequisites

| Need | Notes |
|---|---|
| Node.js 18+ | `npm install` in `AI Controlled WebApp` |
| Python 3.12+ 64-bit | `python -m pip install -r "back end\requirements.txt"` |
| Ollama | a vision-capable model pulled. The app tries `qwen3.8:27b` first (`"model"` in `back end/OllamaOptions.json`), then the largest installed model with vision. In the 2026-10-01 replay of 20 recorded check-ins qwen3.8:27b copied the app's NEXT STEP exactly 93% of the time against qwen3.6's 71% (about 3.8 s against 1.7 s a call, 16 against 22 GiB of VRAM) |
| Orbbec Gemini 335L | optional depth camera; `pyorbbecsdk2` is in `requirements.txt`. Use a USB 3 port: on USB 2 it runs at 15 fps |
| SO-101 arm | optional — without it the app runs in local preview mode |

The launcher searches for an interpreter that has the backend packages and caches
the winner in `.backend-python.txt`, so it only pays that search once. 32-bit
interpreters are skipped (numpy and opencv wheels want 64-bit).

### Ports

| Service | Address | Configured in |
|---|---|---|
| Web app | `127.0.0.1:5181` (listens on every interface) | `server.port` in `vite.config.ts` (`scripts/run-app.ps1` passes `--port 5181` too) |
| Python bridge | `127.0.0.1:8000` (listens on every interface) | `WS_URL` / `API_URL` in `src/App.tsx` |
| Ollama | `127.0.0.1:11434` | `OLLAMA_BASE_URL`, proxied in `vite.config.ts` |
| Depth camera | served by the bridge, `127.0.0.1:8000/depthcam/*` | `SO101_DEPTH_CAMERA=0` leaves it closed |
| Depth server (webcams only) | `127.0.0.1:8001` | `DEPTH_BASE_URL`, proxied in `vite.config.ts` |
| Session recorder | `127.0.0.1:8002` (this PC only) | started by `vite.config.ts` (`SO101_RECORDER_URL`; `SO101_RECORDER=0` turns it off) |

The browser reaches Ollama and the depth server through Vite proxies so it never
hits a CORS wall.

The web app and the bridge listen on every interface so a phone or laptop can
open Clancy, but only this PC uses them freely: any other device needs the
remote password, and a web page from any other site is refused outright (see
[Using Clancy from another device](#using-clancy-from-another-device)).

### Faster conversation with Nimble

Nimble decides what each new prompt sends the main AI (qwen3.8:27b), using
Ollama's typed `/v1/systemone` decision endpoint (a `choice` question with three
options):

| Route | The main AI gets | Skipped |
|---|---|---|
| `text` | the prompt | camera frame, `/locate`, depth map, arm-settling wait, task |
| `camera` | the prompt and one colour camera frame | `/locate` measurements, depth map, camera check, arm-settling wait, task |
| `camera_depth` | the full task path: frame, `/locate` measured positions, depth map, task state | nothing |

The `text` and `camera` replies are plain answers: they cannot create motion
proposals or write the robot's notepad, so they also work during a latched
fault. A safety floor comes first and can only add inputs: task check-ins,
answers to the AI's question, a task or arm motion still under way, anything
acting on an object (grasp, place, touch...), motion words, gripper/joint/wrist
state and coordinates always get `camera_depth` without asking Nimble; scene
words (camera, desk, see, look...) get at least `camera`. Nimble only lowers the
inputs when it is at least 90% sure. The AI panel shows the route, who chose it
(Nimble, safety rule, rules or fallback) and how long it took, for example
`Prompt + camera (Nimble, 180 ms): ...` or `Prompt only (rules, 0 ms): ...`.

Install [Ollama 0.35 or newer](https://ollama.com/download/windows), then install
the [Nimble decision model](https://ollama.com/library/nimble):

```powershell
ollama pull nimble
```

The dev server warms the router when it starts and whenever Ollama comes back
up, and AI mode warms it again; its status appears below the prompt. While
Nimble cannot answer (still loading, missing, older Ollama, malformed answers,
router errors or answers slower than `router_timeout_ms`, 800 ms, at most 1500),
a rule classifier routes instead of waiting, never below the safety floor: clear
general questions, explanations, writing and greetings with no scene, arm,
object, colour or position words get `text` ("what is inverse kinematics?"),
clear scene questions without positions or sizes get `camera` ("what's on the
desk?"), and everything else falls back to `camera_depth`, the path every prompt
took before routing. An uncertain Nimble answer keeps more inputs. The app does
not silently substitute another model.

GPU memory: on the 32 GB RTX 5090, qwen3.8:27b takes about 17.5 GB at
`num_ctx` 32768 and Nimble about 9.5 GB (plus about 1 GB of cache), about 28 GB
together, leaving about 4 GB for the depth server and the desktop. Nimble stays
loaded for `router_keep_alive` (2 hours; every decision renews it, and while
someone has used the page in the last 15 minutes the page warms it again 5
minutes before it would unload or as soon as Ollama dropped it), so it unloads
at most about 2 h 15 min after the page was last used. Close the tab, or set it
shorter (for example `"30m"`), to free those 9.5 GB sooner for games; it is never
pinned. The dev server's own start-up warm-up uses `router_startup_keep_alive`
(30 minutes), so a dev server nobody opens frees it again (`SO101_ROUTER_WARMUP=0`
turns that warm-up off). To keep
Ollama from evicting the main model to make room, the router does not load
Nimble while the loaded models (`/api/ps`) plus 1.1 x Nimble's size would exceed
`router_vram_budget_gb` (29; 0 turns the check off); it then falls back to
`camera_depth`. If `ollama ps` still shows qwen3.8:27b being reloaded, lower the
budget or set `router_enabled` to false.

`back end/OllamaOptions.json` controls `router_enabled`, `router_model`,
`router_timeout_ms`, `router_warmup_timeout_ms`, `router_keep_alive`,
`router_startup_keep_alive` and `router_vram_budget_gb`.
`capture_countdown_seconds` now defaults to zero, removing the former fixed
three-second scene-arrangement delay. Set it to three to restore that delay.
The arm must still settle before a physical task captures its scene. The main
AI and grounding model already use `think: false`; their shared context and
batch settings remain unchanged.

Routing requests, answers and elapsed time are included in the session log and
shown as `ROUTER ASK` / `ROUTER SAID` by `ReviewRecording.py timeline`; each
decision is a `prompt_route` event (route, source, reason, floor, Nimble's
choice and probabilities, time, what was sent), shown as `PROMPT_ROUTE`.
The motion controller also calculates synchronized speeds once per control
dispatch instead of repeatedly for individual joints. This reduces CPU work
while preserving the exact motor commands, braking bounds and physical speed
limits. See `Cad/MotionSpeed.md` for the measured simulation result and the
remaining limits on physical travel speed.

---

## How the pieces talk

```
browser  ──WebSocket :8000──►  bridge_server.py  ──USB serial 1 Mbit──►  6× STS3215
   │   ◄─HTTP /depthcam/*──────────── │ ◄──USB── Orbbec Gemini 335L (color + depth)
   │                                  └── arm_geometry.py (FK/IK)
   │                                      calibration.py  (pixel → cm)
   │                                      object_locator.py (VLM grounding)
   │
   ├──HTTP /ollama──────────►  Ollama :11434       (motion proposals, vision)
   └──HTTP /depth-server────►  Depth Server :8001  (metric depth, optional)
```

The bridge streams full arm state to the browser at 30 Hz and accepts commands on
the same socket: `connect`, `disconnect`, `move_servo`, `toggle_torque`,
`go_home`, `set_home`, `set_servo_limits`, `set_gui_angle`, `set_target`,
`send_ik`, `toggle_elbow`, `set_motion_speed`, `stop_motion`, plus a top-level
`emergency_stop`; `POST /motion` takes the same commands over HTTP. A page on
another device first sends `{"type": "auth", "password": ...}` and gets
`{"type": "access", "ok": ...}` before any state. `connect` with
`"explicit": true` is a Connect click: the only connect the bridge runs from
another device, or after someone disconnected the arm on purpose.

---

## Depth camera (Orbbec Gemini 335L)

The bridge owns the camera (`OrbbecCamera.py`): 1280x720 color with depth
aligned to it in software, at the fastest rate both streams share (30 fps on
USB 3, 15 fps on USB 2). It reconnects by itself after an unplug. The browser
shows it by pulling `/depthcam/frame.jpg` into a canvas, so every capture knows
its frame number; Settings › Camera source picks Auto (depth camera whenever it
streams), Depth camera or Webcam. A webcam never opens the Orbbec.

The layers button on the live view (or Settings › Depth overlay) draws the
depth over the video, blue near to red far, from `/depthcam/overlay.png`. It is
a separate layer: captures for the AI and calibration never include it.

Settings › Depth camera view and Webcam view say where each camera looks from.
"Behind the arm" (the Orbbec on the robot's head, looking forward) sends images
as seen; "Facing the arm" flips them, so in both cases image-left is the arm's
left (+X) and the AI is told which way near and far run. Recalibrate after
moving a camera.

What Qwen gets for each capture:

- The color frame plus the measured depth for that frame (a 3-frame median,
  turbo colormap, black = no reading).
- `MEASURED OBJECT POSITIONS`: Qwen boxes the objects on its native 0-1000 grid,
  and `DepthGeometry.MeasureBox` turns the depth inside each box into the
  object's centre X/Y, base Z, top Z, height and footprint in the AI's frame.
  The table height is measured in a ring around each box, so thin objects (a
  keyboard) still stand out. Objects with no depth fall back to the table-plane
  mapping.

Positions need a calibration made with this camera. Run the one-button
automatic calibration with the camera looking at the arm: at every pose it
records the tip tape's pixel and its 3D point from depth, and the solver keeps
the camera's factory intrinsics fixed and fits only its pose and the tape
offset (`pinhole_depth_v1`). `GET /depthcam/check` later compares the table
plane the camera sees with the calibrated one, which catches a bumped camera.

Where the poses go matters as much as how well they fit. The planner fills the
far desk (`PreferredYRangeCm`, Y 32-40, 12 of 24 poses; the plan spans Y 26-37)
and then covers the rest, giving each wrist pitch at least a quarter: at a single
pitch the tape offset's height and the camera's height trade off exactly, the
residuals cannot see it, and every measured height comes out wrong (the
2026-09-28 14:19 fit did this: 0.17 cm fit, yet the desk read 4 deg tilted and
4 cm off). The solver now refuses a fit whose gripper tilt barely varied.
Nothing limits a pose's shoulder load any more: the `MaximumShoulderLoadKgCm`
cap was removed on 2026-10-03 (446462d). A fixed 3.8 kg cm cap once kept every
pose within Y 26, so the desk was extrapolated. A camera facing the
arm keeps its own plan (`CalibrationSampling.Facing`). The behind-camera plan now
spans display X -14.9..+22.1, the desk where objects sit (the 2026-09-28 22:43
run covered X -34.9..-2.9).

A fit stores the hull of its tips (`observed_tip_hull_cm`), and `/locate` calls
a depth-measured object outside the calibrated area (`extrapolated`) when it
lies more than 10 cm outside that hull (`TipRegionMarginCm`; held-out tips came
back within 0.1-1.3 cm up to 10 cm out), with the distance in
`outside_calibration_cm`. The old test compared the desk contact with the pixels
of the elevated tape and flagged every object of 2026-09-29. Fits without a hull
keep it.

The desk measured through this calibration and by touching it with the tip is
at Z -2.3 (from -2.7 at Y 30 to -1.9 at Y 40: it rises about 4 deg away from the
robot in the arm's frame). `TableZCm` holds that value; depth measurements use
the desk measured around each object, so the slope does not bias them.

After camera alignment passes, depth can measure a local level support under
the task object without an entry in `back end/DeclaredSurfaces.json`. The support
height and observed extent travel with that capture as `support_geometry`.
Declared surfaces remain optional hints and still have to agree with depth.
This removes the requirement to enter each platform or shelf into a desk map;
it does not remove the camera-to-arm registration or the arm's reach limits.

`SceneGeometry.py` extracts occupied boxes and volumes whose complete projected
footprints are seen clear by depth. Shelf sides, back walls and an observed
overhead board become obstacles. Missing depth and space behind an object stay
unknown. The task object's measured envelope is kept separately for grasp
contact and carried-object checks. The bridge stamps an immutable, short-lived
scene identity; a new capture or a changed camera registration revokes previous
motion approvals. Tests use rendered depth scenes and fake motors, never the
physical arm. Generalized shelf pickup still needs physical validation.

```powershell
python "back end\TestDepthCamera.py"
```

covers all of this on ray-cast scenes with the real factory intrinsics: object
measurement from two viewpoints, thin objects and basket floors, the depth
calibration recovering the camera to under 8 mm, a wrong depth scale rejected,
the camera service (pinning, unplug and reconnect) and the bridge endpoints.

### Camera check and the AI task loop

About 1 degree of camera rotation moves every measured position about 1 cm at
desk range, so `/locate` checks the camera against its calibration before Qwen
is asked anything. A capture that fails the check stops a task that needs
positions; the one exception, a camera assumed where it was calibrated when
nothing in view can check it, still runs but is never treated as checked:

- **Camera gate, desk in view.** The calibrated desk decides, as it always has:
  within 0.5 deg / 0.5 cm on its height map with at least 67% of its cells
  agreeing, and the red tape on the gripper (when it can judge) within 1 cm of
  where the arm puts it. A cart move is handled below.
- **Camera gate, no desk in view** (a shelf or cabinet: no plane near the
  calibrated desk, or under 200 of its cells seen as desk; `back end/RobotReference.py`).
  The camera is assumed fixed where it was calibrated and the gripper's red tip
  tape checks it, through every joint and at any wrist roll: within 0.75 cm of
  where the arm puts it the capture is **verified**; positively farther off,
  the camera **moved** and the task stops (recalibrate); when the tape cannot
  judge (arm moving, torque off, jaw not as at calibration, tape hidden or out
  of view) the capture passes as **assumed**, with a one-line warning saying
  why (Aaran's decision, 2026-10-06/07). An assumed capture is not a checked
  one: its measurements keep their `[UNRELIABLE]` tag (`scene_checked` false on
  the bridge, `sceneChecked` false in the app), it keeps a scene tripwire, and
  space the camera has not observed beside the task object is never taken as
  free (`SceneGeometry.AssumeTaskReachFree` stays off). With no desk in view the
  desk-object tripwire cannot fire (no desk under the objects: on the recorded
  cabinet frame with the camera turned 4 deg the bottle measured 4.2 cm off and
  nothing tripped), so its tripwire is the basket rim: when the rim positively
  reads moved against the calibration's own frames, the capture is blocked as
  camera_moved (`object_locator.TRIPWIRE_ASSUMED_RIM`, since 2026-10-08). The
  rim catches some moves, not all: on the 15:01 frame re-rendered as the camera
  turned, drooped or rolled 1-6 deg it read moved 14 times in 24, ok 7 times and
  could not judge 3; the frame as recorded read ok; the real 4.16 deg turn of
  23:01 could not be judged. For a verified camera the rim and the calibration's
  RGB keyframe stay advisory fields only.
- **What a block looks like.** A confirmed camera movement or an unexplained
  desk change stops the task before any model call, and a red banner in the AI
  panel says, for example,
  *Camera moved 5.2 deg / 1.3 cm since calibration at 15:04: lock the head and
  run Automatic calibration.* (When the gripper-tape check or the desk-object
  check caught it, the banner gives that reason instead of the desk numbers.)
  Lock the head (tilt, pan and the camera screw), run Automatic calibration and
  send the task again; the banner clears on the next capture that passes. Home,
  jog and the other manual controls keep working. A task that acts on an object
  also stops when the calibration has no check reference, the desk could not
  be measured, `/locate` fails or the bridge does not answer, instead of
  guessing positions from the image; with the webcam, such tasks need the depth
  camera. Descriptions, relative jogs and gripper commands still run, told that
  no positions were measured.
- **Moving the cart.** The arm, camera and basket ride one cart, so rolling it
  against the desk changes only the desk. When the desk disagrees with its
  baseline, the red tape on the gripper decides: within 0.75 cm of where the
  arm puts it, the camera has not moved on the arm, the capture goes on and the
  desk is measured again on it and saved as the new baseline
  (`camera_model.drift_reference`, with the replaced desk under `desk_history`
  and the old file in `captures/<date>/desk_remeasure/`); no recalibration.
  Farther off, the camera moved and the gate stops as above (a 1 deg droop of
  the head already does). When the tape cannot be judged (torque off, arm
  moving, wrist roll or jaw changed, tape out of view), tasks on objects stop
  with *The desk moved against the arm since calibration (was the cart
  moved?)* and what to do: park the arm, torque on, tape in view, hold still,
  send again. A desk change over 4 deg / 3 cm is not taken for a cart move. A
  roll of the camera about its own line of sight up to about 2 deg is not seen
  by the tape (see `back end/CameraDrift.py`).
- **Check-in timing.** After an approved move, the next check-in waits for the
  bridge to start it (its motion epoch advances past the one at send), then for
  fresh state with the arm still and 0.3 s from the start. While the bridge
  plans a route it sends the app no state, and the old wait fired on that
  frozen state before the arm had moved. A move not started within 25 s, not
  settled within the bridge's budget for the whole route, or refused by the
  bridge stops the task with the reason (a refusal of something else, such as
  Home pressed meanwhile, does not). Gripper steps get a check-in too, and each
  check-in says how far the last motion actually moved the tip. Approve refuses
  a relative move if the tip has moved more than 1 cm since it was proposed;
  the approval box shows the move from the live tip.
- **No-op rule.** A proposal within 0.5 cm and 2 deg of the target the bridge
  is holding, or within 1 cm and 2 deg of the live tip, with no slider changing,
  would not move the arm: it is not offered, not sent and not counted, and the
  AI gets one corrective turn. A second no-op, the same target twice in a row,
  a rejected target asked for again at a later check-in, or the last of the 8
  motions ends the task with the reason in the chat. The arm runs one move at a
  time, so a proposal changes the tip or one slider, never both; a slider
  restated where it already is is ignored.
- **Chained motion, opening on the way.** A `gripper_before` that opens the
  jaws runs as ONE bridge motion with its tip move: `send_ik` carries
  `gripper_raw`, the bridge plans the arm's route with the jaws already open,
  blends the opening in from where they are (done halfway through the arm's
  route time, or later when the gripper's own speed ceiling needs it, at the
  latest on arrival) and certifies every stage again with the moving jaw's
  swing in its box; if that does not certify, the same route opens the gripper
  in place first. The app also checks the moving jaw at every opening along
  the way against measured objects. A close or a narrowing stays two motions
  (closing while the tip rises would lift before the jaws hold anything), and
  its tip move waits until the jaws hold still. The bridge lets the gripper
  creep up to its 8-step goal tolerance while a route plans (2026-10-03 14:46:
  3 steps of creep refused the tip move after an open). An older bridge
  (no `gripper_with_tip_version` in its state) gets the two motions.
- **Task object.** The object a task names is tracked across check-ins: whether
  this frame detected it, where it was last measured, and whether the right arm
  can grasp it (the app's own IK and basket model); that verdict is repeated
  while the object is hidden under the gripper. Measured out of reach, the task
  stops and asks you to move it closer; that advice is given only when the
  camera check passed. Lost more than 8 cm from the gripper, the task stops,
  unless the gripper has closed since (it may be carrying it). `/locate` is sent
  the task's object: the bridge's strict query for it says which measurement
  it is, and a renamed or misspelt object is still found
  (`src/lib/TaskObject.ts`).
- **On a platform or a shelf.** A measured local support uses its own height
  and extent (`lib/AiTableFloor.ts`, `lib/AiMeasurements.ts`) rather than an
  assumed desk height. A registered scene is required for a newly measured
  support. Platforms can use an overhead approach. Shelves with a measured
  overhead board use a side pregrasp, advance, close-only step, a small release
  from the support, and withdrawal at the same wrist pitch. The planner checks
  the gripper and held object against occupied geometry and observed free
  space throughout those legs, not just their endpoints. The release is
  1.2 cm so it exceeds the live no-op tolerance. Unknown clearance, unreachable
  poses or an obstructed withdrawal refuse the grasp. A fresh capture after
  closing can retain the held object's envelope while remeasuring the support,
  obstacles and free space; it never reuses old empty-space observations.
  `TestObservedGrasp.mjs` exercises side entry, release, withdrawal and refusal
  on synthetic scene bounds. It is not evidence of a successful physical pickup.
- **[UNRELIABLE] measurements.** An object more than 10 cm outside the
  calibration tips ("11.2 cm outside the camera-calibrated area"), one the
  samples disagreed on, one only the task query found, or a coarse box the
  bridge did not zoom-refine (it refines only the task object's boxes) is
  tagged [UNRELIABLE]. A target_cm that descends onto it is refused; a hover
  3 cm above its top is not, and the descent is a relative step.
- **Snap.** A target the arm cannot reach exactly is moved to the nearest one
  it can: the wrist alone first (up to 180 deg), then the tip (up to 3 cm, at
  most 0.5 cm lower, never below the floor or across the basket rim). The
  approval card and the next check-in say what was asked and where the arm
  goes instead; anything further is refused with the nearest possible target.
- **Still arm, time limits, endings.** A task reads the arm's pose only after
  300 ms without motion (waiting up to 10 s). A model call has 120 s and
  `/locate` 60 s. The first bridge refusal of an approved move goes back to the
  model with the bridge's reason; a second one ends the task. Every ending
  aborts the call in flight, leaves nothing to approve, says why and records a
  `task_end`.
- **Model calls.** The app and the bridge call Ollama's native `/api/chat` with
  thinking off and the same `num_ctx`, `num_batch` and `keep_alive`
  (`back end/OllamaOptions.json`), so neither makes Ollama reload the model for
  the other; the page loads the model once when AI mode opens.

These are checked by `TestAiTaskSafety.mjs`, `TestAiTargetSnap.mjs`, `TestAiTableFloor.mjs` and the browser suite
`TestAiTaskSafetyUi.mjs` (each serves the app itself on 5191-5199), with every other suite by `python RunAllTests.py`
(see [Testing](#testing)).

---

## Session recording

Everything is recorded per day in `back end/captures/YYYY-MM-DD/recording/`,
so a session can be looked back on:

| File | What | Written by |
|---|---|---|
| `video/HHMMSS_mmm_depthcam.mp4` + `.frames.jsonl` | the depth camera's color stream while an AI task runs, from 5 s before it to 10 s after it, H.264 in segments of at most 10 minutes, with every frame's exact time (`HHMMSS_mmm-1_...` when another segment started in the same millisecond; `HHMMSS_depthcam` before 2026-10-08) | `SessionRecorder.py`, its own process |
| `video/HHMMSS_mmm_webcam.mp4` + `.frames.jsonl` | the same from the webcam that films the demo area (`ReviewRecording.py frame --camera webcam`) | `SessionRecorder.py` |
| `recorder_events.jsonl` | each task the recorder was told of, and when and why each camera's recording started and stopped | `SessionRecorder.py` |
| `ai.jsonl` | every model request and reply through the `/ollama` proxy (warm-ups marked `preload`), and the app's events: prompts, how each reply was read, snaps, approvals, blocks, bridge refusals, task-object binds, bridge faults, errors, camera-check failures, no-op proposals and one `task_end` per task with its reason and source | the dev server (`scripts/SessionLog.ts`) |
| `grounding.jsonl` | the bridge's object-detection calls (the task query too, with token counts) and `/locate` results | `SessionLog.py` |
| `bridge_events.jsonl` | the bridge's motion faults, protection trips, motion aborts, grasp holds and control delays the arm held still through | `bridge_server.py` |
| `route_timing.jsonl` | how each completed route went, stage by stage | `bridge_server.py` |
| `images/` | every image sent to the model (color frames and depth maps), stored once | both logs |
| `depth/` | each `/locate`'s raw depth, as a 16-bit PNG | `SessionLog.py` |

The day folder itself keeps the photos the AI saw (`*_vision.jpg`) and, per task, a
clip of the page's live view (`HHMMSS_mmm_task.webm` + `.json` with the prompt and
how the task ended): the depth camera's colour view when that is the source, the
webcam otherwise.

Requests the app cancels (Manual mode, Disconnect) are recorded too, marked
`aborted`, with an `ai_stopped` event giving the reason. Each new task starts
the model with a fresh conversation, so an earlier task's positions are not
reused.

**When video is recorded** (Aaran, 2026-10-08): only while an AI task runs. The
page tells the dev server when a task starts and ends, and every 15 s that it
still runs (`POST /session-log/recording`, behind the same access rule as the rest
of `/session-log`); the dev server passes that on to the recorder on
`127.0.0.1:8002`, which takes it from programs on this PC only. Between tasks both
cameras are still read, but only their last 5 s are kept, in memory, so a
recording starts 5 s before its task; it ends 10 s after the task. A task whose
page goes quiet (closed, crashed, the dev server stopped) stops recording 3
minutes after its last word; one that never ends is cut at 60 minutes. Before
2026-10-08 the recorder ran around the clock: 56 GB on 2026-10-05, a day with no
task.

**How much:** while a task runs, about 2 to 5 GB an hour of task time: the depth
camera 1.0 to 3.8 GB/h (median 1.9), the webcam up to 1.7 GB/h (measured on the
2026-10-04..06 segments). A typical task of 30 s to 3 minutes, with the 15 s
around it, is about 25 to 270 MB. Recording pauses (nothing is deleted for it)
while the drive has less than 20 GB free.

**Kept 7 days:** `back end/VideoRetention.py` runs when the recorder starts and
every hour. It deletes the recorder's segments with their frame indexes and the
per-task clips with their sidecars, once both the time in the name and the last
write are more than 7 days old, and logs every file in
`captures/video_retention.log`. Nothing else is ever deleted: not `selfcal/` or
`desk_remeasure/` (the calibration archives), the text logs above, `images/` and
`depth/`, the AI's photos, or anything outside `captures`; links and junctions are
not followed. To see what the next pass would delete (add `--delete` to delete it
now, `--days` for another age):

```powershell
python "back end\VideoRetention.py"
```

Video you want past 7 days is kept by name: `captures/video_keep.txt` lists it,
one path relative to `captures` per line (a glob allowed, `#` for a comment),
and the pass never deletes a segment or clip listed there, with its partner.
`back end/KeepVideo.py` fills it: with no arguments it lists the video around
every servo fault and protection trip the bridge recorded
(`bridge_events.jsonl`; 10 minutes before to 2 after, both cameras), `--write`
adds it to the list, and `--at "2026-10-03 10:00"` names a moment of your own.
It copies, moves and deletes nothing. Moving video into a folder that is not a
day (`captures/keep/`) keeps it too.

```powershell
python "back end\KeepVideo.py" --write
```

The recorder that deletes is the one the dev server starts, and the dev server
restarts by itself when `vite.config.ts` changes, as a merge or a pull does. So
keep what you want before such an update, not after. As a safety net, the
recorder's very first retention pass on a captures folder does what
`KeepVideo.py --write` does before it deletes anything (once:
`captures/video_keep_first_pass.json` says it did, so a line you remove later
stays removed), and a pass whose keep step fails deletes nothing.

The dev server starts the recorder with itself and on every page load; a second
copy exits. When the running recorder reports another version than
`back end/RecorderVersion.json` (or none, like the around-the-clock recorder
before 2026-10-08), the dev server starts the current one with `--replace-older`,
which stops the old one first. `SO101_RECORDER=0` turns the recorder off.
Automated test browsers are kept out of the record and of the recording.

**Logs:** the bridge's `AI Controlled WebApp/api-output.log` no longer gets the
camera frame and status polls or `/health` (some twenty lines a second: 337 MB in
26 h before), and every launcher keeps the earlier logs, capped at 20 MB, three of
them (`api-output.1.log`, ...; `back end/LogRotation.py`). The camera SDK's
`back end/Log/OrbbecSDK.log.txt` is cut to 10 MB, three kept, when the bridge
first opens the camera.

```powershell
python "back end\ReviewRecording.py" timeline --from 13:08 --to 13:12
```

```powershell
python "back end\ReviewRecording.py" sheet --from 13:09:40 --to 13:10:30 --every 5
```

`timeline` lists prompts, model traffic (with token counts and model loads),
measurements, snaps, approvals, task ends, bridge events, the recorder's tasks and
recordings and video segments in order; `frame --at 13:09:47` and `sheet` pull
video frames by time.
`back end\TestSessionRecording.py` covers the recorder (including a bridge
restart), the logs and the review tool; `TestTaskRecording.py` the recording
around tasks, `TestVideoRetention.py` the 7-day pass, `TestLogRotation.py` and
`TestBridgeRecordingFiles.py` the logs and capture names.

---

## Motion control

Every move — slider drag, IK send, Home — is planned as a checked route and runs
through one closed-loop velocity follower. Moving joints are synchronized: the
joint with the longest time to go runs at the throttle's cruise speed and the
others are slowed to arrive with it, so the arm follows the checked straight
line between waypoints. Intermediate waypoints are passed without stopping; the
slow-down zone applies once at the route end, where every joint is parked in
position mode. Because a loaded joint can overshoot a waypoint it flies past,
each route edge is checked that far past the waypoint, a joint that runs ahead
is eased off, and near a waypoint no joint goes faster than it can stop inside
that checked overshoot. Where the overshoot cannot be checked (next to an
obstacle), the arm passes that waypoint slowly instead. In open space a straight
move is a single edge, so the arm cruises. Details: `Cad/BasketAvoidance.md`,
Execution.

Targets are clamped to each joint's configured limits. **Measured positions are
not** — a hand-posed joint can legitimately sit outside its limits, and the
controller has to see where it is. Nothing drives it back in on its own: only a
commanded move (its slider, or Home) does, planned and
collision-checked like any other. The state reports such a joint
(`servos.<name>.outside_limits`), and Hold and a limit change say so in their
status. Narrowing a limit past a joint that is holding leaves it holding where
it is (it used to drive the joint into the new range, 150 steps in the audit's
reproduction). A joint outside its arc is read on the side of what it is held
to or routed from: in the middle of its excluded window two raws a step apart
used to read a turn apart, so a wrist roll hand-turned there and held coasted
the whole arm on a step of jitter (2026-10-08; hold-v4).

Homing shares the follower but is capped at a lower speed, since it drives every
joint at once. Home brings all six joints to their saved values, the wrist roll
and the gripper included (until 2026-10-12 it left the roll where it was). The
arm and the roll go on one checked route. The gripper opens on the way, as a tip
move's opening does, but closes only once the arm has arrived: the route's last
stage moves nothing but the jaws, so a jaw stopped by something becomes a grasp
hold instead of a stall trip that coasts the arm (Home used to close the jaws
mid-route). A gripper holding an object (a grasp hold: a close that something
stopped) is left as it is, neither opened nor closed further, and the status
says "Home: gripper left holding the object". Stop, Hold and Coast before the
close cancel it. Set Home saves all six, so save it again if the roll or the
jaws were somewhere you do not want Home to bring them back to.

**Hold** (the torque toggle) points every joint at where it physically is, arms
the motion profile, and only then energises — so it locks the arm where it
stands and is safe to press after hand-posing a coasted arm, also with a joint
outside its slider limits (until 2026-10-07 Hold drove such a joint to its limit
at up to 160 steps/s, unchecked). The page sends which button it showed
(`toggle_torque` with `enable`), so Hold pressed while holding stops a motion and
holds instead of coasting. Holding where it is includes the grip: a stop that
holds (Stop and hold, Hold pressed while moving, a route stopped by a changed
scene) re-applies a grasp's 8-step squeeze instead of letting it go; Coast and
the emergency stop release it. Stop and hold goes over HTTP (`POST /motion`)
when the bridge socket is down, and a `POST /motion` move that arrived before a
Stop is cancelled by it, as a queued socket command is.

### Hand guiding

**Hand guide** (Manual mode, the Motion panel's sixth button) lets you move the arm by hand: the shoulder, elbow and
wrist carry their own weight, so they stay where you leave them, and the base (pan), the wrist roll and the gripper go
limp like Coast, with a little help in the direction you move them on slow, steady moves (about 30 % less force in the
simulation; quick moves feel exactly like Coast). It works from this PC or any device with the arm password. The design,
its numbers and its simulation are `docs/HandGuide.md`; the code is `back end/HandGuide.py` (the law),
`back end/GuideBus.py` (the bus), `back end/HandGuideCalibration.py` (the calibration) and the bridge's `HandGuide*`
methods.

- **First, calibrate it once** (the button reads **Calibrate guiding** until then; again from Settings, Hand guiding):
  about 6 minutes, every joint sweeping slowly through a stretch of its range (each excursion checked against the
  basket and the cart before anything moves), then the roll and the gripper spun briefly to tell whether a servo with
  its torque off floats or brakes. Keep clear with a hand ready. It saves `back end/hand_guide_calibration.json` only
  if every check passes, and the button says why when not. A changed servo setting or zero pose asks for it again.
- **Start**: with the arm holding still, press **Hand guide** and keep your hands off for 2 s while it checks that it
  carries the arm. Refused, with the reason, if the servo supply reads under 8.5 V (or the servos disagree by 0.5 V),
  a servo is at 50 C, the arm moves or is outside its slider range, or the calibration does not fit.
- **Stop**: **Hold here** (the same button), Stop and hold, or Hold: the arm holds exactly where it is.
- **Anything going wrong makes the whole arm go limp** (a servo not answering or restarting, the USB cable, a servo at
  55 C, the supply under 7 V, the control loop pausing, the model check, a fall, too fast), and so does the
  **Emergency stop**, which says "the arm drops" while guiding. **Keep a hand on or under the arm.**
- The page that started it keeps it going: if it closes, or a phone's screen sleeps, for 3 s the arm holds where it is.
  A gripper holding an object keeps its grip. Moves, Home, Set Home, limit edits, camera captures and AI tasks wait for
  Hold here; switching to AI mode holds first.

```powershell
python "back end\TestHandGuideLaw.py"          # the law on the offline plant (--full: all 323 scenarios)
python "back end\TestHandGuideCalibration.py"  # the calibration on the plant
python "back end\TestHandGuideBridge.py"       # the bridge on the fake servo bus
python "back end\TestGuideBus.py"              # the transport and the Present_Load sign
node "AI Controlled WebApp\TestHandGuide.mjs"  # the page's button, strip and gate
```

`GET /servo/dump` (this PC, or with the arm password) returns every servo's registers 0..70, read only, for the first
hardware session's register check (docs/HandGuide.md 9, S1).

`back end/test_bridge_home.py` covers this against a simulated motor bus, and
`back end/TestHomeRollAndGripper.py` Home's wrist roll and gripper (the opening on the way, the close after arrival, a
held object, a stop before the close) on the servo model with the real planner:

```powershell
python "back end\test_bridge_home.py"
python "back end\TestHomeRollAndGripper.py"
```

---

## Safety

- **The AI never moves the arm on its own.** Every proposal waits for you to
  click Approve or Block.
- Proposals are re-checked against the app's own IK before they can be approved;
  unreachable targets are sent back to the model to revise, up to 4 times.
- Every target pose and every route is checked against the CAD surfaces of the
  cart, mast and camera head, including tracking error and independent joint
  progress. The collision model has no height floor: manual targets outside
  the basket are allowed where the whole arm clears (AI targets have the desk
  floor below).
- The top tray's front wall sits in the model where the head depth camera sees
  it (rim at display Y 22.1-23.3, 2.39 cm nearer the robot than the CAD;
  `FrontWallCorrection` in `back end/BasketSurface.json`).
- No reach may put any part of the arm behind display Y -30 (the rearward
  limit). There is no centre-of-mass or tipping check: it was replaced by that
  limit on 2026-09-28 (`back end/TestRearwardLimit.py` asserts its masses and
  footprint are gone). Details: `Cad/BasketAvoidance.md`.
- Stop and hold, Hold/Coast, disconnect and Emergency Shutoff run ahead of any
  command still queued or planning: a route still being planned is cancelled
  at its next check instead of starting and then stopping, and commands sent
  before the stop never run. Set Home, a joint-limit change, a current-angle
  override and Home itself wait until the arm has stopped.
- A route checked against a measured scene stops and holds (torque on) when that
  scene changes or expires mid-route, or the configuration changes; until
  2026-10-07 these coasted every joint, dropping the arm. A scene that would
  expire before the route can finish is refused at the start, and `/locate`
  refuses while such a route runs. Home and manual moves are never bound to a
  measured scene.
- An arm parked inside an obstacle's margin, or lying on the basket rim after a
  coast, can still be moved: the first leg may not come any nearer (or press more
  than 3 mm further in) than the arm already is. Deeper overlap is refused.
- Grasps approach via a checked overhead waypoint or a measured shelf's checked
  side pregrasp, then close before lifting or withdrawing.
- The desk itself is not in the collision model, so AI targets, and the
  gripper's lowest point in their pose, may not go more than 1 cm below the
  table surface (`TableZCm` in `WorkspaceFrame.json`, -2.3 cm, measured
  2026-09-28) anywhere outside the top tray's open top (`BasketTrayBoundsCm`:
  Y less than 23.3); such proposals are refused and sent back to the model. The
  whole cart's bounds reach Y 29.1 over the desk, and once the tray's wall moved
  they exempted low desk targets in front of it, so they are not used for this.
  A task on a measured support uses its local floor and occupied geometry.
  The bridge receives the scene identity, not client-supplied obstacles, and
  checks full CAD arm sweeps against the registered obstacles on every route,
  including detours. The gripper and carried object must also stay in observed
  free space or the permitted target-contact envelope. Unknown space elsewhere
  along the arm is not independently certified clear; observed obstacles are
  still checked against the full arm. Scene contact cannot use the basket's
  start-overlap relief. Missing, stale or superseded scene identities refuse
  dispatch, and scene invalidation stops an active route.
  The robot is left out of its own scene: the cart, basket and mast, and the
  arm where it stood for the capture, with every point within 2 cm of its CAD
  link boxes (its own points lie up to about 2 cm outside them: calibration and
  CAD error). A real board that close to the arm is left out too. Where the
  wrist and the gripper stood is "known" space (taken as free for the gripper)
  only 0.5 cm around their boxes, as axis-aligned boxes: enough for the arm to
  leave its own place, while the space cut out beyond that is unknown, not free
  (`SceneGeometry.KnownArmMarginCm`; until 2026-10-08 it was the whole 2 cm, and
  a board 0.5 to 1.9 cm beside the gripper was cut out and declared free). Still
  open: a board within 2 cm of the forearm or upper arm is invisible to their
  checks, and the boxes of turned parts still declare some free space around
  the gripper (PROGRESS.md, audit 1 IM1).
- Protection coasts the arm on lost feedback, a fault bit or heat, so a false
  alarm drops it. Three causes found and fixed on 2026-09-28:
  - Connect closes the servos' EEPROM lock, as Hold does. Its Goal_Position seed
    switches torque on, and with the lock open every Operating_Mode change is an
    EEPROM write whose status packet takes 20-40 ms, past the SDK's 34 ms
    timeout, so the first route after a connect tripped "motion write failed".
  - Position, status and temperature reads retry once: a single dropped packet
    is not a lost servo, and a silent one still trips.
  - An over-limit temperature is re-read before coasting (a lone 83 C byte on a
    28 C servo once dropped the arm); contradicted reads show in
    `state.protection.glitches`.

  A gripper that closes on an object is held there with a bounded 8-step
  squeeze instead of tripping the stall check, which coasted the arm on a grasp
  (`back end/MotorProtection.md`, Grasp hold).

  A control frame over 750 ms late trips only while something moves; an arm
  holding still through it is read again and goes on holding (2026-10-04 23:11 UTC, 16:11 local:
  a 1.56 s delay with nothing moving coasted and dropped the arm;
  `back end/MotorProtection.md`, Delayed control frames). Every frame 300 ms
  late or long is recorded as `control_stall` with what it waited for (the motor
  lock and who held it, the frame's own steps, or the CPU), and the running
  bridge writes every thread's stack to `control_stalls.log` beside
  `bridge_events.jsonl` when a frame is 0.5 s overdue.

  `GET /bus/stats` counts servo transactions, failures and slow replies.
- Motion profiles are capped in firmware terms on every write (acceleration 51,
  Hold profile speed 200 — 20% of full scale; route speeds at each joint's 100%
  ceiling, at most 1750 steps/s, see `Cad/MotionSpeed.md`).
- **Emergency Shutoff** cuts torque, disconnects, and closes the socket.

### Using Clancy from another device

The bridge (port 8000) and the web app (port 5181) listen on every interface,
so a phone or laptop on the LAN or on Tailscale can open Clancy. Who may do
what (`back end/RemoteAccess.py`, and `scripts/RemoteAccess.ts` for the web
app's own routes):

- **This PC needs nothing.** Clancy opened at `http://127.0.0.1:5181` or
  `http://localhost:5181` works exactly as before: no password, and the arm
  connects by itself when the bridge comes up. Opening the PC's LAN address on
  the PC itself counts as another device.
- **Any other device needs the remote password**, for everything except
  `GET /health`: commands, the arm's state, the depth camera's pictures of the
  room, the notepad, the serial ports, and the web app's model, session-log and
  bridge-launch routes. With no password set, other devices get nothing.
  Without it, the web app gives another device only the page itself (its
  scripts, styles, fonts and robot models), so the page can ask for the
  password; none of the web app's other files. Vite's "open in editor" route,
  which starts an editor on this PC, answers only this PC's own Clancy page.
- **A web page from any other site is refused**, password or not: the bridge
  and the web app's routes take browser requests only from Clancy's own page,
  so a page you happen to have open cannot drive the arm through
  `ws://127.0.0.1:8000`. To the bridge, Clancy's page is the one on port 5181
  (or 5180) of the address it asked; `SO101_DASHBOARD_PORTS` (comma-separated)
  on the bridge adds others, for example Vite's default 5173 from
  `npm run dev:web`.
- **Nothing connects the arm or turns its torque on by itself from another
  device.** Opening Clancy there shows the arm, but only its Connect button
  connects it (connecting energises the servos where the arm is).
- **A deliberate disconnect holds everywhere.** After Disconnect or a page's
  Emergency stop, from any device, no page connects the arm again by itself,
  this PC's included, until someone presses Connect. The bridge keeps this in
  `back end/ConnectIntent.json`, so a restarted bridge still knows. Stopping
  the launcher with `Ctrl+C` is not a deliberate disconnect: the next start
  connects as it always did.

Set the password on this PC, in the `back end` folder:

```powershell
python SetRemotePassword.py --generate
```

That makes a strong one (`abcd-efgh-jkmn-pqrs`), stores it and prints it once.
`python SetRemotePassword.py` asks for your own instead (at least 10
characters, typed twice). `--status` says whether one is set; `--off` removes
it, and other devices are refused again. Only a salted scrypt hash is kept, in
`back end/RemotePassword.json`, which is git-ignored and never committed. The
bridge and the web app read it again on their next request: no restart. After
a new password, every device is asked for it at its next request.

Use the phone:

1. Open `http://<the PC's address>:5181`: its LAN address (for example
   `http://192.168.1.20:5181`) or its Tailscale address (for example
   `http://100.101.102.103:5181`). Use the address, not the PC's name: the web
   app refuses host names it does not know.
2. Enter the password in the **Arm password** card. The browser keeps it, so
   this is once per device; Settings, **Forget the arm password** removes it.
   "Not now" leaves a **Locked** button in the top bar.
3. Press **Connect** when you want the arm on. Emergency stop, Stop and hold,
   Hold/Coast and Disconnect work from the phone too.

Five different wrong passwords from one device lock it out for 10 minutes (the
right one too, meanwhile); the same wrong one sent again counts once. Refused
devices are written to the bridge's error log and to `bridge_events.jsonl` as
`remote_access`. A program on another device puts the password in an
`X-Clancy-Password` header, on HTTP and on the `/ws` handshake alike.
Requests that come through a proxy (they carry `X-Forwarded-For` or the like)
are never this PC and share one lockout, whatever address they claim; the
launchers start the bridge with `--no-proxy-headers`, so the address it sees is
the real one.

On the wire: other devices talk to the PC over plain HTTP, and the page sends
the password with every request it makes to the PC. Over Tailscale (the 100.x
addresses) that traffic is encrypted by WireGuard. On a plain LAN or Wi-Fi it
is not: anyone who can watch that network can read the password (and the camera
pictures) and use it. So prefer the PC's Tailscale address on other devices,
set a password only if you use one, and on a network you do not trust run
`python SetRemotePassword.py --off` (other devices then get nothing) or set a
new password afterwards. The web app takes the password off every request
before passing it on, so its `/ollama` and `/depth-server` proxies never
forward it to Ollama or the depth server, which may be another host.

---

## Testing

```powershell
python RunAllTests.py
```

runs every suite offline, one at a time (about an hour): `tsc` on the app and on
`vite.config.ts` + `scripts/`, every node suite in `AI Controlled WebApp`
(`Test*.mjs`), every browser suite (the Playwright ones, in headless Edge),
every `back end/Test*.py` and `test_bridge_home.py`, and the two CAD checks in
`Cad/` (with `Cad/.venv`'s Python, see `Cad/ReadMe.md`; they never rewrite the
reports in `Downloads/SO101Cad`). `--lanes py,cad,node,ui` picks lanes,
`--only TestSafety.py,TestGraspPlan.mjs` picks suites, `--parallel` runs the
Python and node lanes side by side. Logs and a summary table go to
`back end/TestArtifacts/RunAllTests/<tag>/` (`Summary.md`); each suite is
**pass**, **fail**, **timeout**, **recording missing** (below) or **not set up**
(a CAD check without `Cad/.venv`). A failure is **known** when every check that
failed is one `KnownFailures` in `RunAllTests.py` lists for that suite by name
(today TestSelfCalibration's two table mappings and the two 2026-09-28 replays
below, each with why and who decides): those do not fail the run, a new failing
check in the same suite does. A browser suite that fails is run once more and,
passing then, is reported **flaky** (both logs kept). The run's exit code is 0
when every suite passed, was known or was flaky.

Nothing real is touched: the depth camera, the session recorder and Ollama's
autostart are off, every service URL points at a closed port, session records go
to `back end/TestArtifacts`, and `TestRouteCheckPool` starts 2 workers
(`SO101_TEST_ROUTE_WORKERS`). Each browser suite serves this checkout on its own
port, the first free one of 5191-5199, without `vite.config.ts` (whose plugin
starts the recorder, Ollama and the bridge) and with its cache in `dist/`
(`AI Controlled WebApp/UiTestLib.mjs`); its browser maps the live ports
(5180/5181, 8000-8002, 11434) to a closed port and uses a fake camera, so a test
can never reach the running app, the arm or a camera. `CadTestUrl` points a suite
at a test server of your own instead; a live port is refused.

**Replays of recorded data.** Some suites replay real recordings from
`back end/captures`, which git does not keep (`back end/RecordedCaptures.py`
finds them: `SO101_REAL_CAPTURES`, else this checkout's, else, in a git worktree,
the main checkout's). A replay whose recording is missing fails with
`RECORDING MISSING: <what>: <path>` instead of skipping (the 2026-09-13..10-02
recordings were deleted on 2026-10-03 and their replays then skipped silently for
days); `SO101_MISSING_RECORDINGS=skip` makes them skips on a machine that never
had them. The cart-move replays keep their two frames in git
(`back end/TestFixtures/CartMoved-2026-10-03`). Two replays fail this way now:
the 2026-09-28 droop captures (`TestCartMove`) and that day's failed-task
captures (`TestDepthCamera`) are gone; restore them, or retire those two tests.

**Timing.** A test that bounds a wall-clock time takes the fastest of a few runs
and scales its bound with how busy the PC is (`back end/LoadScale.py`; the node
suites scale by the cost of one IK solve), so a loaded PC does not fail it. The
browser suites give the 3D view's first load 2 minutes (`CadReadyMs`),
`TestNoScroll` measures each layout on a settled frame, and `TestTargetSnapUi`
retries a release whose half-second budget a busy PC made it miss (three tries).

## Configuration files

| File | Holds |
|---|---|
| `back end/arm_geometry.json` | measured segment lengths and joint signs |
| `back end/home_preset.json` | the saved home pose, raw steps per motor |
| `back end/servo_offset_zeroing_backup.json` | the servos' Position Correction (reg 31) values before they were zeroed on 2026-09-12, plus the measurements that showed velocity mode ignores that register |
| `back end/servo_eeprom_limits_backup.json` | the servos' burned angle limits before they were cleared on 2026-09-11 (pan, elbow), for restoring |
| `back end/gui_limits.json` | per-joint travel limits and target reach caps |
| `back end/CollisionModel.json` | arm link envelopes, cart, top tray (`BasketTrayBoundsCm`) and mast bounds, the rearward limit and the tracking allowances (generated by `Cad/PrepareCollisionModel.py`) |
| `back end/RemotePassword.json` | the remote password's salted scrypt hash (`SetRemotePassword.py`): other devices need that password. Machine-local and git-ignored, never committed |
| `back end/ConnectIntent.json` | whether the arm was disconnected on purpose, kept by the bridge until an explicit Connect. Machine-local and git-ignored |
| `back end/hand_guide_calibration.json` | hand guiding's calibration (Calibrate guiding): the fitted gravity model, each joint's friction and velocity slope, whether torque-off floats, and the servo settings and zero pose it was made with. Per-arm data, like `CadAlignment.json` |
| `back end/RecorderVersion.json` | the session recorder's version, which the dev server requires the running recorder to report (`version` in `127.0.0.1:8002/health`), replacing an older one; bump it with every recorder change |
| `back end/BridgeVersion.json` | the bridge's code version, which the page and `RestartBridge.ps1` require the running bridge to report (`bridge_version`); bump it with every bridge change the page relies on |
| `back end/BasketSurface.json` | the cart and basket surfaces collisions are checked against, with the top tray's front wall moved to where the depth camera sees it (`FrontWallCorrection`) |
| `back end/OllamaOptions.json` | `num_ctx`, `num_batch`, `keep_alive` and the message layout every Ollama call sends, the app's and the bridge's alike |
| `back end/camera_calibration.json` | probe points and homography, or the automatic camera model (`pinhole_depth_v1` for the depth camera) |
| `back end/camera_calibration.cart-desk-2026-10-02.bak.json` | the calibration the 2026-10-02..10-06 recordings were taken with (the 2026-10-02 21:09 fit, its desk re-measured 2026-10-03), for the replay tests: a new calibration does not change what they replay |
| `back end/DeclaredSurfaces.json` | Optional support hints: a platform (`Kind` `platform`) or shelf (`shelf`, `BoardAboveZCm`), with `TopZCm` and `XRangeCm` / `YRangeCm` in the display frame. Depth can discover a local support without an entry; declared hints must still agree with measured depth. The bridge re-reads changes; the web app reloads with the file |
| `AI Controlled WebApp/src/lib/ReachEnvelope.json` | the right arm's tip reach with the cart parked, per tip height and wrist pitch, with and without the basket and body (`node TestReachEnvelope.mjs` checks it against the IK; `--write` saves it again) |
| `back end/qwen_notepad.md` | the AI's notes, kept across sessions until **Reset AI context** empties it. The file is tracked in git and the running app rewrites it |

`camera_intrinsics.json` is optional. Without it, clicked pixels are not
undistorted before fitting, which costs accuracy near the frame edges. Generate
it with `back end/calibrate_intrinsics.py`.
