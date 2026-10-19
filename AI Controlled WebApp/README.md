# AI Controlled Arm — Web GUI

A browser app for driving a 6-joint **SO-101 robotic arm** by chatting with a
local AI model. You type something like *"move 5 cm to the left and open the
gripper"*; the AI proposes a motion; you see it previewed on a 3D model, and
**nothing moves until you click Approve.**

This is the AI version of the SO-101 web controller. It reuses the same 3D
preview, joint sliders, and target controls, and adds an AI chat panel with a
human-in-the-loop approval step.

> **New here? Read [docs/HOW_IT_WORKS.txt](docs/HOW_IT_WORKS.txt) first.** It is a
> hyper-detailed, top-to-bottom walkthrough of exactly how this version works:
> every component, the data flow, the coordinate systems, and the AI loop.

---

## The three pieces

This app only works when three programs are running together:

| Piece | What it is | Where it runs |
|-------|------------|---------------|
| **Web app** (this folder) | The React UI you click in | your browser, served by Vite at `http://127.0.0.1:5181` |
| **Python bridge** | Talks to the arm's motors over USB and does the math (inverse kinematics) | `back end/` at the repo root, served at `http://127.0.0.1:8000` |
| **Ollama** | The local AI model that turns your words into a motion proposal | this machine, default `http://127.0.0.1:11434` (override with `OLLAMA_BASE_URL`) |

The web app talks to the bridge over a WebSocket, and to Ollama over HTTP
(proxied by Vite to avoid browser CORS errors).

---

## Project structure

```
AI Controlled WebApp/
├── README.md                  ← you are here (start guide)
├── index.html                 ← the page Vite serves
├── package.json               ← dependencies + npm scripts
├── vite.config.ts             ← dev server (port 5181) + Ollama proxy config
├── UiTestLib.mjs              ← the browser suites' own test server (5191-5199)
├── tsconfig*.json             ← TypeScript settings
│
├── docs/
│   ├── HOW_IT_WORKS.txt        ← the full, detailed explanation
│   └── home-position-check.png ← screenshot of the app at the home pose
│
├── scripts/
│   └── run-app.ps1             ← one-command launcher (bridge + web app)
│
└── src/
    ├── main.tsx                ← React entry point
    ├── App.tsx                 ← the whole app: UI, AI loop, bridge connection
    ├── styles.css              ← all styling
    ├── components/
    │   ├── ArmCanvas.tsx        ← the interactive 3D arm view (re-exports the three.js CadCanvas.tsx)
    │   └── ServoPanel.tsx       ← the per-joint sliders
    └── lib/
        ├── robotModel.ts        ← kinematics math + coordinate conversions
        ├── OllamaChat.ts        ← the native /api/chat request, reply and warm-up
        ├── AiMeasurements.ts    ← /locate replies, [UNRELIABLE] tags, reach verdicts
        ├── TaskObject.ts        ← which measured object the task means
        ├── AiTargetSnap.ts      ← moves an unreachable AI target to the nearest reachable one
        ├── AiTableFloor.ts      ← the desk floor for AI targets and the gripper
        ├── AiTaskLoop.ts        ← check-ins, the still-arm wait, time limits, feedback
        ├── GraspPlanWorker.ts   ← grasp planning off the page's main thread
        ├── RemoteAccess.ts      ← the arm password on another device
        └── SessionVideo.ts      ← tells the recorder when an AI task starts and ends
```

**Rule of thumb:** UI you can see lives in `src/components/`, the math lives in
`src/lib/`, and `App.tsx` wires it all together and runs the AI conversation.

---

## Prerequisites

1. **Node.js 18+** (for the web app).
2. **Python 3.12+ (64-bit)** with the bridge's packages. From the repo root:
   ```powershell
   python -m pip install -r "back end\requirements.txt"
   ```
   The launcher finds an interpreter that has these packages on its own and
   remembers it in `.backend-python.txt`, so it only searches once.
   The packages are `fastapi`, `uvicorn`, `pyserial`, and `scservo_sdk`.
3. **Ollama** running somewhere reachable, with a vision model pulled: the app
   tries `qwen3.8:27b` first (`ollama pull qwen3.8:27b`). See *Configuration* below.
4. *(Optional)* the **SO-101 arm** plugged in over USB. Without it, the app still
   runs in **local preview** mode — you can move the on-screen model and chat
   with the AI, but no hardware moves.

---

## Run it

The easy way — one command starts the Python bridge **and** the web app, then
opens your browser. From the repo root:

```powershell
& ".\AI Controlled WebApp\scripts\run-app.ps1"
```

Or, from inside this folder:

```powershell
npm install   # first time only
npm run dev    # same launcher, via the "dev" script
```

The app opens at **http://127.0.0.1:5181**. Press `Ctrl+C` in the terminal to
stop everything the launcher started.

### Run only the web app (no bridge)

```powershell
npm run dev:web   # Vite alone, at http://127.0.0.1:5181
```

The port is `server.port` in `vite.config.ts` (with `strictPort`: if 5181 is taken
it stops rather than serve another port, whose page the bridge refuses).

---

## Configuration

### Ollama address

The web app reaches Ollama through a Vite proxy (`/ollama` → your server) so the
browser never hits a CORS wall. The default target is the Ollama instance on
**this machine**, `http://127.0.0.1:11434`. Override it before launching to point
at a different host:

```powershell
$env:OLLAMA_BASE_URL = "http://192.168.1.50:11434"
npm run dev
```

How a model is chosen, in order:
1. The model `back end/OllamaOptions.json` names (`"model"`: `qwen3.8:27b`), if it
   is installed and its `/api/show` capabilities include `vision`.
2. Otherwise the largest installed model whose `/api/show` capabilities include
   `vision`.
3. Otherwise a model Ollama already has loaded (from `GET /api/ps`), so it
   doesn't trigger a slow fresh load.
4. Otherwise the first installed model (from `GET /v1/models`).

### Automatic camera calibration (no clicks)

Put a small piece of matte red tape on the gripper tip. Open **Calibrate**,
press **Check marker**, click the red tip in the check image and confirm its
green outline (this seeds the tracker), then press **Start automatic
calibration**. The bridge chooses about 24 poses (`GET /selfcal/poses`, about
15 s: reachable, clear tip targets spread in pan, height and wrist pitch, half of
them on the far desk where objects sit). The arm visits them slowly, and at each one the web app
captures three frames and posts them with the live joint positions
(`POST /selfcal/observe`). `/locate` uses the saved model automatically. Re-run
it whenever the camera or the arm moves; nothing needs measuring.

Every pose keeps all of its stable red blobs, not just the one the live tracker
followed. `POST /selfcal/solve` (`back end/SelfCalibration.py`) drops blobs that
sit still across poses (a notebook, a rim strip), then picks each pose's tape by
camera consistency: a RANSAC over poses finds the camera most poses agree with,
and the full model (focal length, principal point, two radial distortion terms,
camera pose, and the tape's offset from the tip) is refined while each pose
re-picks its nearest blob. On the 2026-09-13 hardware run the live tracker had
lost or mistaken the tape (mostly for the servo wiring) at 14 of 30 poses and
the fit was rejected at 110 px; solving from all blobs matches 27 poses and
passes (0.55 cm fit, 0.76 cm on held-out poses, about 0.8 cm on the table).

A fit is saved only if it passes, judged at the tape in centimetres: fit under
0.6 cm, held-out poses under 0.9 cm, predicted table error under 1 cm, camera
above the table. Pixel limits could not work: the arm's own joint accuracy is a
few millimetres at the tip, 10 to 15 px with this camera. Weak priors, scaled to
the data's own error, keep the principal point near the image centre and the
distortion small, because a tip volume a few centimetres thick cannot separate
them from camera rotation. Both handedness hypotheses are fitted; a runner-up
that needs a camera below the table does not count as ambiguous.

Nothing limits a pose's reach or shoulder load any more: the static shoulder
gravity-moment filter (added after the first real run went limp at 27 cm poses
carrying about twice the resting shoulder load) was removed on 2026-10-03 at
Aaran's request, so the poses go out to the far desk. Wrist roll and the gripper
jaw are never moved during the run. Every pose saves its first frame
with the detection drawn on it to `back end/captures/<date>/selfcal/`, and the
whole run is archived (see `back end/CalibrationRecording.md`);
`back end/ReplayCalibration.py` re-solves an archived run with the current
solver. `back end/TestSelfCalibration.py` checks the solver against synthetic
ground-truth cameras, rendered frames, and red tape among a stationary red
object and wiring the tracker locks onto, with no hardware. The table height is
`TableZCm` in `back end/WorkspaceFrame.json`.

### Control loop and servo protection

The closed-loop velocity controller runs in its own 30 Hz thread
(`ControlLoop` in `back end/bridge_server.py`). It used to run only inside each
browser WebSocket connection, so with no tab connected, or a tab busy with a
command, a joint could keep its last speed until it hit a mechanical stop. The
bridge also polls each servo's `Hardware_Error_Status` about four times a
second; if a servo trips its own protection (overload, overcurrent, overheat)
every motion stream is stopped at once, the event (servo, load, voltage,
temperature) is sent to the web app, and a running self-calibration aborts.
Voltage and encoder flags are reported as warnings only.

### Servo position correction must be zero

The STS3215 (firmware 3.10) reports `Present_Position = encoder - PositionCorrection`
(register 31) in position mode but the raw encoder value in velocity mode. The
bridge streams every motion (sliders, Home, Send) in velocity mode, so a non-zero
correction made every joint appear to jump by that many counts the moment a
motion started, and the closed-loop controller then drove the joint to "fix"
it. On 2026-09-12 the pan servo carried 1449 (127 degrees) and the others 85
(7.5 degrees); all six were zeroed and the saved home/limits shifted to match.
If a servo is ever replaced or recalibrated with a tool that writes register 31,
zero it again (see `back end/servo_offset_zeroing_backup.json`).

### IK preview while dragging, and Send

Dragging the 3D target asks the bridge for an IK preview; the browser also runs
the same solver locally to validate drags and Send. The CAD solver
(`back end/CadKinematics.py`, mirrored in `src/lib/CadKinematics.ts`) seeds both
elbow bends and both forward and backward pan configurations from a closed-form
planar reduction, refines each on the exact CAD transforms inside the joint
limits, and ranks the results by elbow preference and joint travel. The preview
shows the first candidate whose whole arm clears the cart, mast and camera head
and keeps every part of the arm ahead of the rearward limit, so a colliding
preferred pose never hides another clear pose at the same point.

"Reachable" means the same thing for the preview and for Send: within 0.1 cm
(`IK_SEND_MAX_ERROR_CM`) and 0.25 degrees of wrist pitch, including encoder
rounding. The solver reports an impossible pitch instead of silently changing
it; the green arc shows the pitches that work at that point.

Send plans a route for every feasible candidate and keeps the fastest. Each
straight leg is checked as short waypoints the arm flies through without
stopping, so a clear line is taken directly instead of detouring. If the arm
starts inside the safety margin of the mast or basket (for example parked
upright beside the mast), it first leaves by the shortest checked step that
never comes closer to the obstacle. See `Cad/BasketAvoidance.md`.

The green **wrist arc** and the scroll-wheel clamp use the same limit-aware
test as the bridge (both elbow branches, joint limits from the live servo
ranges), so a pitch inside the green is one the solver will accept. The web app
keeps at most one preview request in flight, always requests the newest target
when a reply lands, and keeps showing the previous bridge preview while one is
pending instead of the rougher client-side solver.

### Task recordings

Sending a prompt to the AI starts recording the live view (MediaRecorder, VP9/VP8
webm at ~2 Mbps; the depth camera's colour view when that is the source, the
webcam otherwise). The recording runs for the life of that task and stops on any
of its endings: the AI reports done/ask, you press *Stop task* or block a motion,
the motion limit is hit, or Emergency Shutoff. The clip is posted to the bridge's
`POST /captures/video` and lands in `back end/captures/<date>/<time>_task.webm`
with a `.json` sidecar (prompt, end reason, duration). A blinking REC badge shows
in the task strip while recording. The AI never sees these clips; they are for
reviewing what physically happened.

The same task also switches the session video (`back end/SessionRecorder.py`, both
cameras): `src/lib/SessionVideo.ts` tells the dev server when the task starts and
ends, and every 15 s that it still runs (`POST /session-log/recording`), and the
recorder records from 5 s before the task to 10 s after it. Between tasks nothing
is recorded. Both kinds of video are kept 7 days (`back end/VideoRetention.py`);
see the main README's Session recording.

### Model calls: Ollama's native API, thinking off

Every model call, the app's (`requestAiCompletion` in `src/App.tsx`, body from
`src/lib/OllamaChat.ts`) and the bridge's object locator
(`back end/object_locator.py`), goes to Ollama's native `/api/chat` with
`think: false` and the same `num_ctx` (32768), `num_batch` (1024) and
`keep_alive` (30m) from `back end/OllamaOptions.json`. Callers with different
values make Ollama reload the model (8-15 s) each time the other one calls. The
OpenAI-compatible `/v1` endpoint used before ignored `num_ctx`, so the model
loaded at the desktop app's 262144 tokens and left under 1 GiB of VRAM free.
Each content part is its own message (the "split" layout: the text, then each
image with empty content), which reproduces the `/v1` token counts; one merged
message made the model reply with malformed JSON.

Thinking is off because Qwen3.x models otherwise spend most of each call on
hidden reasoning: on the same vision request, `qwen3.6:latest` (the model then)
took 3.0 s with thinking and 0.7 s without, at the same answer quality for these
JSON-shaped tasks. A reply is
capped at 1024 tokens (`AiMaxOutputTokens`), and one cut off there is an error,
never a motion; if thinking is turned on again, that budget must grow.

Opening the page in AI mode sends one empty warm-up call, so the first task does
not wait for a cold model load (Manual mode loads nothing). A model call has
120 s and `/locate` 60 s; at the limit the task ends with the reason.

Recommended model: `qwen3.8:27b`, the one the app tries first (`"model"` in
`back end/OllamaOptions.json`). In the 2026-10-01 replay of 20 recorded check-ins
it copied the app's NEXT STEP exactly 93% of the time against `qwen3.6:latest`'s
71%, and invented 2 moves in 40 against 10, at about 3.8 s a call against 1.7 s
(16 GiB of VRAM against 22 at `num_ctx` 32768). `qwen3.6:latest` (Qwen3.6-35B-A3B,
a Mixture-of-Experts model with ~3B active parameters) is the faster fallback.

### What the app checks in an AI task

The full rules are in [docs/HOW_IT_WORKS.txt](docs/HOW_IT_WORKS.txt), section 8.

- **A still arm.** A task reads the arm's pose only when no route runs and
  nothing has moved for 300 ms (it waits up to 10 s), and it ends if the tip
  moves more than 1 cm while the image is measured.
- **Object positions.** `/locate` is sent the task's object: the bridge flags
  the measurement its strict query lands on and zoom-refines only that
  object's boxes. Measurements the model may only hover over are tagged
  `[UNRELIABLE]`: more than 10 cm outside the calibration tips ("11.2 cm
  outside the camera-calibrated area"), samples that disagreed, found only by
  the task query, or a coarse box the bridge did not refine (never the task
  object's). A `target_cm` that descends onto one is refused; a hover 3 cm
  above its top is not.
- **Snap.** A target the arm cannot reach exactly is moved to the nearest one
  it can (`src/lib/AiTargetSnap.ts`): first the wrist alone (up to 180 deg),
  then the tip (up to 3 cm, at most 0.5 cm lower, never below the floor or
  across the basket rim). The approval card and the next check-in say what was
  asked and where the arm goes instead. Anything further is refused and sent
  back with the nearest possible target.
- **Desk floor.** Outside the top tray's open top (Y less than 23.3, inside its
  front rim), an AI target and the gripper's lowest point may not go more than
  1 cm below the desk. The whole cart's bounds reach Y 29.1 over the desk and
  are not used for this.
- **Bridge refusals.** The first refusal of an approved move goes back to the
  model with the bridge's reason (for IK: how close the closest pose came and
  which wrist pitch reaches the point); a second one ends the task.
- **Every ending** (Stop, a fault, a time limit, a second refusal, ...) aborts
  the call in flight, leaves nothing to approve, says why in the chat and
  records a `task_end`.

### Ports

| Thing | Address | Set in |
|-------|---------|--------|
| Web app | `127.0.0.1:5181` | `server.port` in `vite.config.ts` (`scripts/run-app.ps1` passes `--port 5181` too) |
| Python bridge | `127.0.0.1:8000` | `WS_URL` / `API_URL` in `src/App.tsx` |
| Ollama | `127.0.0.1:11434` (this machine) | `OLLAMA_BASE_URL` / `vite.config.ts` |
| Session recorder | `127.0.0.1:8002` (this PC only) | started by `vite.config.ts`; video only while an AI task runs, kept 7 days |
| Browser test suites | the first free port of `127.0.0.1:5191-5199` | `UiTestLib.mjs`: each suite serves this checkout itself and never reaches the ports above |

Both the web app and the bridge listen on every interface, but only this PC
uses them without a password. A phone or laptop enters the remote password
once (`back end/SetRemotePassword.py` sets it; `src/lib/RemoteAccess.ts` keeps
it in that browser and sends it), and never connects the arm by itself. Until
then the web app gives it only the page itself, and Vite's "open in editor"
route answers only this PC. Other devices use plain HTTP: prefer the PC's
Tailscale address (encrypted) to its LAN address. See "Using Clancy from
another device" in the repo's README.

---

## Safety

- **The AI never moves the arm on its own.** Every proposed motion waits in an
  approval box for you to click **Approve** or **Block**.
- Before approving, the app runs the proposal through its own inverse-kinematics
  check. An unreachable target is moved to the nearest reachable one within the
  snap's caps, and shown as such; otherwise it is refused and the AI is asked
  to revise (up to 4 times).
- AI targets, and the gripper's lowest point, stay above a floor 1 cm under the
  desk everywhere outside the top tray's open top.
- A gripper that closes on an object holds it with a bounded squeeze instead of
  tripping stall protection and coasting the arm (`back end/MotorProtection.md`).
- Poses and routes that would hit the cart, mast or camera head, or put any
  part of the arm behind the rearward limit (display Y -30), are refused before
  any motor is written. There is no centre-of-mass or tipping check (replaced by
  the rearward limit on 2026-09-28) and no shoulder-load limit (removed
  2026-10-03).
- Object positions come only from a capture whose camera check passed. With no
  calibrated desk in view the gripper's red tape checks the camera; when it
  cannot (arm moving, tape hidden), the capture passes as "assumed" with a
  warning, and its measurements keep their `[UNRELIABLE]` tag: the app never
  treats an assumed camera as a checked one (repo README, "Camera check").
- Grasp planning runs in a worker (`src/lib/GraspPlanWorker.ts`), so Stop and
  hold and Emergency Shutoff stay responsive while a grasp is planned (it held
  the page up to 11.8 s before 2026-10-07). AI tasks wait while the running
  bridge is not the code this page expects (`back end/BridgeVersion.json`) or a
  joint's current angle is overridden in the joint panel.
- The big red **Emergency Shutoff** button cuts motor torque and disconnects
  immediately.
- Only this PC drives the arm without a password; another device enters the
  remote password once, and only its Connect button connects the arm. After a
  Disconnect or Emergency Shutoff, no page connects it again by itself until
  someone presses Connect.
