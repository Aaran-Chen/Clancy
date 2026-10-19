# Automatic coast protection

Version: auto-coast-v2-confirmed (since 2026-10-12; below, Confirmed readings).
The dashboard displays whether the connected bridge
actually advertises this version. A code change alone does not activate it in
an already-running bridge.

A latched motion fault now coasts all motors. Sources include hardware status
bits (including unrecognized bits), lost encoder or protection feedback,
unexpected torque loss, failed motion/hold writes, checked-route departures,
motion timeout, excessive holding drift, and an interrupted control heartbeat
while anything moves (below, Delayed control frames).
An uncaught control-loop exception also attempts coast instead of being ignored.

Additional software policies:

- Poll temperature at most every 250 ms; trip at 60 C. Explicit Hold rearming
  requires temperatures below 55 C and healthy status/position reads.
- Trip after three seconds (two until 2026-10-12) without at least two raw encoder steps of progress
  toward a commanded target while its error exceeds 30 raw steps. Stationary
  healthy Hold and slow continuing progress do not trigger this check. The one
  exception is a grasp, below. No check compares commanded with measured speed,
  so a joint that cannot reach its 100% route ceiling (Cad/MotionSpeed.md) slows
  the route instead of tripping.
- Trip before resuming a control iteration delayed beyond 750 ms while
  anything moves. An arm holding still through the delay is read again
  instead (below).

## Delayed control frames (2026-10-04)

On 2026-10-04 at 23:11:14 (UTC) the control loop resumed 1.56 s after its last
frame and the heartbeat coasted the arm. Nothing was moving: no route, every
command speed 0, every joint holding in position mode, the AI waiting for an
approval. The arm was extended over the desk and dropped (the shoulder sagged
about 110 steps), and stayed coasting until Hold 37 minutes later. The bridge
process itself had not stopped: the session recorder fetched every depth-camera
frame from it throughout those 1.56 s, and `GET /bus/stats` shows no servo
transaction slower than 141 ms. Only the control thread was late (waiting for the
motor lock or for the CPU; the logs cannot tell which).

What a late frame risks depends on what the servos were told:

- Holding still, each STS3215 keeps its own Goal_Position: nothing moves
  unsupervised, and only the bridge's own checks (temperature every 0.25 s,
  status every frame) wait, on readings that change over minutes, and the
  firmware's own overload, overcurrent and overheat protection stays on.
  Coasting such an arm is what drops it.
- In velocity mode (a route, a stream, a hold correction) a joint keeps its last
  speed until the next frame. The STS3215 has no timeout for a silent host (its
  memory table, Waveshare ST3215 register map, has no watchdog register), so
  nothing on the servo bounds that travel. At 750 ms a joint at its 100% route
  ceiling runs up to 1312 steps (115 deg, pan and roll; lift 712 at its 950
  steps/s spec, elbow 938, wrist flex 1238); at the highest speeds commanded on
  2026-10-04 (pan 228, lift 635, elbow 603, wrist flex 402 steps/s) 171 / 476 /
  452 / 302 steps, about 28 cm at the tip for the lift alone from the hover pose
  over the desk. The trip cannot bound that travel (it acts after the delay);
  the route monitor, checking every stage's certified region on the first
  frame back, catches what left it.

Now (ControlHeartbeatLimitS, 0.75 s):

- A frame later than that trips protection, as before, unless the arm held still
  from the end of the last frame to now (`ArmHoldsStill`: torque on, no route,
  calibration, stream or correction, every command speed 0). A move a command
  started, or stopped, while the loop was late still trips.
- Held still, every joint is read again first, as after a serial link drop:
  position, torque, hardware status and operating mode, each retried once. The
  arm goes on holding only if every joint answers, has torque, reports no fault,
  is in position mode and is within `SerialLinkHoldDriftRaw` (30) steps of where
  it held when the last frame ended; the same frame then reads temperatures and
  status before any write. Anything else trips ("Motor control heartbeat was
  delayed: shoulder_lift moved 45 steps while the control loop was delayed").
  If the serial port is lost on those reads, the serial link supervisor takes
  over and reads every joint again when the port returns.
- Each delay the arm held through goes to `bridge_events.jsonl` as
  `control_delayed` (gap and limit in ms), the status says "Control loop delayed
  1.6 s while the arm held still; every joint read back holding" for 20 s (Hold
  or Coast ends it), and `state.protection` carries `heartbeat_limit_ms` and the
  recent `control_delays`. `/health` and `state.protection` report
  `heartbeat_version` `held-still-v1`, so a bridge still running the old rule
  shows. There is no upper bound on a delay held through: the
  servos held by themselves whatever its length, and the trip would act only
  after it anyway.

The 750 ms bound while moving is unchanged. Raising it would only let a longer
unsupervised run go on without a trip. What would bound velocity-mode travel
during a stall is a command the servo bounds itself (a position goal a short
way ahead); the bridge has none today. `AdvancePositionMotion`, named here as
that cure until 2026-10-07, was dead code (nothing ever selected it) and is
gone: routes, streams and hold corrections all run in velocity mode, so a stall
while moving still lets each moving joint run at its last speed until the
first frame back trips protection and the route monitor checks the region.
Streaming bounded position goals on routes is a change to the route executor
still to be designed and measured on the arm.

Connect holds the motor lock for the whole bus setup (about 90 transactions,
12 of them EEPROM writes), and the arm can come out of it holding (the
Goal_Position seeds energise it). The heartbeat used to measure the first
frame's gap from the frame before connect, so a connect over 0.75 s tripped it
and dropped an arm that was holding; connect now restarts the heartbeat at its
end, as Hold does. Nothing is commanded at that point, so the moving-state limit
is unaffected.

### What a stall waited for (2026-10-07)

Stalls of 1.1-1.7 s recurred (2026-10-03 14:47, 15:57; 10-04 23:11; 10-07
00:33 UTC) with nothing moving, and their cause is unknown: the recorder kept
getting frames during the 10-04 one, which argues against the interpreter lock
and for the control thread waiting on the motor lock or the CPU. The bridge now
records, for every control frame that starts 300 ms late or itself runs 300 ms
(`ControlStallLogS`), a `control_stall` event in `bridge_events.jsonl` (also in
`state.protection.control_stalls`) with the gap, how long the frame waited for
the motor lock, which threads held it longer than 50 ms meanwhile and for how
long (the lock is an `InstrumentedLock` that keeps them), the time each step of
the frame took (`phases_ms`: positions, servo status, protection, route check,
commands, route advance), and a one-line cause: the lock and its holder, the
frame's own steps, or the CPU/interpreter (the thread was not running). The
running server also arms `faulthandler` each frame: when no frame has finished
0.5 s after the last one started, every thread's stack is written to
`captures/<date>/recording/control_stalls.log`, by faulthandler's own thread,
which works even while another thread holds the interpreter lock, and the next
frame writes under it when the stall ended and how long no frame finished (a
dump has no time of its own), to match it with its `control_stall` record. These
are diagnostics only; no command, limit or trip changes. Read the next stall's
record before changing anything. Candidates to look at first: the state
snapshot each WebSocket client takes 30 times a second under the motor lock,
which reads `WorkspaceFrame.json` from disk twice (ConfigurationKey and
WorkspaceOffset), while the recorder writes 2-3 GB of video an hour to the same
disk.

The other trip that day, "shoulder_pan left the checked basket route" at
23:50:01, was the route monitor at the switch to a route's last stage, where it
accepted less than the stage before had; it latched and coasted the arm too (the
next route started with the shoulder sagged 293 steps). Cad/BasketAvoidance.md,
Execution, describes it and the route end reach that fixes it.

Fake-bus tests (TestMotorProtection.py; TestProtectionConfirmation.py for confirmed readings): the 23:11:14 servo record held through
a 1.56 s delay without a trip or a write, a moved, unpowered, faulted,
velocity-mode or silent joint after a delay tripping, a stream, route or hold
correction tripping, a move started or stopped during the delay tripping, and a
delay under the limit leaving no protection record (only the `control_stall`
diagnostic above). TestAuditSafety.py: a slow connect to a holding arm does not
trip the first frame, a stall waiting for the lock names the thread that held
it, and a slow frame step names the step.

## Grasp hold (2026-09-29)

A gripper that closes on an object stalls there by design: that is the grasp.
The stall check above used to coast the whole arm about two seconds after the
jaws met the object (fake-bus simulation of the AI's `target_fraction 0.0`
step: the arm dropped with the object). A thin object that stops the jaws less
than 30 steps short of closed escaped that check and was pushed at full torque
in velocity mode until the motion timeout coasted the arm instead (63 s at 30%
speed in the same simulation).

Now, when the gripper stops while closing (toward `target_fraction 0.0`) on a
route that moves nothing else from its current stage on, and makes less than
two steps of progress in 0.5 s while more than its 8-step goal tolerance short:

- It is switched to position mode with its goal 8 steps
  (`GRIPPER_GRASP_SQUEEZE_RAW`, about 0.7 deg) more closed than where the jaws
  stopped. The servo's own position loop then presses on the object with a
  force bounded by that offset instead of pushing at full torque.
- The route ends there (nothing else was moving), and a `grasp_hold` record goes
  to `bridge_events.jsonl` with where the jaws stopped, the goal and the
  commanded close. The state reports `grasp_hold: true` for the gripper, and its
  `target_raw` follows the jaws, so no hold correction drives them further into
  the object or lets it go, and the app's settle check passes.
- A later route that leaves the gripper where it is keeps the squeeze, re-based
  on the planned gripper value. 8 steps is inside the 16-step tracking allowance
  every route certifies, so an object slipping out mid-route never takes the
  jaws off the checked route.
- Any new gripper command, a trip, Coast, the emergency stop or a disconnect
  ends the grasp hold. A stop that holds keeps it (hold-v4, 2026-10-08): Stop
  and hold, Hold pressed while moving, and a route stopped because its measured
  scene or the configuration changed re-apply the squeeze from where the jaws
  rest, so a carried object is not dropped while the arm holds. (Before, such a
  stop held the jaws where they were without the squeeze, which is the grip
  force.)

Every other stall keeps the stall trip (three seconds since 2026-10-12): opening into something, closing
while arm joints also move, or a joint stalling on a route. If writing the
grasp hold fails, it trips as the stall would have. Home closes the jaws only
once the arm has arrived, as its route's last stage (until 2026-10-12 it
closed them mid-route, where an object stopping them tripped the arm), and
leaves a gripper in a grasp hold as it is (it neither opens nor squeezes it).

Needs checking on the arm (the fake bus cannot tell): that 8 steps grips a
real object firmly enough without heating the gripper servo over minutes, and
that the servo's firmware overload protection does not unload the gripper
during the 0.5 s of full-torque push before the hold.

False trips drop the arm just like real ones, so the feedback that feeds them is
made hard to fake (2026-09-28, each with a fake-bus test):

- Position, hardware-status, torque and temperature reads retry once. A single
  missing status packet no longer coasts the arm; a servo that stays silent
  fails both attempts and trips as before.
- An over-limit temperature is read again before coasting. A lone corrupted
  83 C reading on a 28 C shoulder once dropped the arm mid-route; real heating
  takes minutes, so the confirming read costs nothing. Contradicted readings are
  kept in `state.protection.glitches`.
- Since 2026-10-12 a missed temperature reply, a status reading and a stall
  need more than one look (Confirmed readings, below).
- Connect ends by closing the EEPROM lock (Lock = 1), as Hold does. The
  connect-time Goal_Position seed turns torque on, and with the lock open each
  Operating_Mode change is an EEPROM write: its status packet arrived 20-40 ms
  later (measured on the arm), past the SDK's 34 ms timeout, so the first route
  after a connect tripped "motion write failed", and the late packet then broke
  the next read ("Incorrect status packet"). Locked, the same change replies in
  about 1 ms.

`GET /bus/stats` reports servo transactions, failures (with register, servo and
duration: about 34 ms means the servo did not answer, much longer means the
bridge thread was starved) and replies slower than 20 ms, plus `port_lost` and
`reopens` (serial link drops, below).

## Confirmed readings (auto-coast-v2-confirmed, 2026-10-12)

Two false alarms coasted a healthy, holding arm on 2026-10-12 (UTC,
`bridge_events.jsonl`):

- 16:17:25 "Lost shoulder_pan protection feedback": one Present_Temperature
  read had no status packet on either try, every joint's position read fine.
- 16:32:43 "shoulder lift torque unexpectedly off": one status reading read
  torque off with load 63745 (0xF901), 5.6 V and 90 C, positions matching
  their targets (no sag). Every "torque unexpectedly off" with telemetry on
  record (2026-10-03 to 10-12, ten of them) carries that signature: 89-90 C
  with 0-5.6 V (once 15.2 V) and a load with bits beyond the register's range.
  Every real temperature on record is 27-38 C at about 8.9 V.

Now (`ProtectionVersion` auto-coast-v2-confirmed; `TestProtectionConfirmation.py`):

- Temperature: a joint's Present_Temperature must fail on
  `ProtectionFeedbackMisses` (2) polls in a row to trip "Lost <joint>
  protection feedback ... (2 polls in a row)": at most about 0.5 s without a
  temperature, which takes minutes to change. A joint whose position did not
  read in that frame either trips at once, as before; Hold's rearm check still
  refuses on one failed read. Over-limit readings are confirmed by a re-read, as
  before. The 60 C trip and 55 C rearm limits are unchanged.
- Status and torque: a reading of Hardware_Error_Status and Torque_Enable that
  shows anything (a fault or warning bit, torque off while it should be on, or a
  failed read) is read again at once, before anything is written. Only two bad
  readings in a row latch: the fault, or "Cannot read <joint>
  protection/torque status" when both failed. A reading whose telemetry no servo
  can report (`ServoTelemetryImpossible`: Present_Load beyond 0-1000, a supply
  under 6.0 V with 85 C or more, or over 14.0 V) counts as a failed read, so the
  same corrupted reply twice still latches. A servo that really restarted
  (brown-out) keeps torque off on the re-read and is caught in the same frame:
  the added delay is one more reading, a few milliseconds (about 70 ms when both
  reads time out). Fault records now also carry Lock (0 after a restart).
- Stall: `ProtectionStallSeconds` 3.0 s (was 2.0). On a hard stall the servos'
  own protection acts about 2 s in either way: overload (over 80 % output while
  not moving for Protection_Time, then output limited to 20 %; factory setting,
  the bridge writes none of these registers) and overcurrent (2.0 A for 2 s,
  torque off), docs/HandGuide.md. A stall under 80 % output heats the servo far
  slower; one more second at the 2.0 A overcurrent limit (about 18 W at 8.9 V) warms it
  0.4 C (55 g servo), at most about 2 C in the motor alone, against
  a 22 C margin to the 60 C limit, which is still polled every 0.25 s. The grasp
  hold (0.5 s) and the route monitor are unchanged.
- Each tolerated reading goes to `bridge_events.jsonl` as `protection_glitch`
  (joint, register, value, the telemetry and why) and to
  `state.protection.glitches`, so they stay visible. A diagnostic record: it is
  never the latched fault the page shows.

Unchanged: the temperature limits, the servos' firmware protection registers and
torque, speed and acceleration caps, the 750 ms heartbeat while anything moves,
the emergency stop, and position feedback (one retry, then "Lost joint
feedback"). Not known yet: whether the 16:32:43 kind of reply clears on the
immediate re-read on the arm; the `protection_glitch` records will show it. If
the same reply also comes back on the re-read, it still latches "Cannot read".

## Serial link drops (reopen-v1, 2026-10-01)

On 2026-10-01 at 19:07:22 the arm's USB serial adapter (CH343, COM4) dropped off
the USB bus for 23 s. Windows logged it as surprise-removed within 1 ms of the
fault, 2 s after a device (most likely a phone) failed to enumerate on the
neighbouring rear USB4 port. What followed was all software:

1. pyserial raised `ClearCommError failed (PermissionError(13, 'Access is
   denied.', None, 5))` in the middle of a read. Windows fails every call on a
   handle to a removed device that way, for good, even after the device returns.
2. The scservo SDK marks the port busy (`is_using`) when a packet goes out and
   clears it only on its normal returns. The exception left it set, so every
   later call failed "Port is in use!" without touching the port.
3. Nothing reopened the port. The trip's torque-off, Hold at 19:22:07 ("without
   fresh positions") and the emergency torque-off at 19:22:14 all failed that
   way, and a second press then claimed "coast and disconnected" although no
   packet had reached a servo. The servos, powered separately, held torque
   unsupervised for 15 minutes. The manual reconnect coasted the arm silently on
   its first frame.

Now:

- Every servo transaction is serialised in `feetech_simple` and clears the SDK
  busy flag whatever happens. An OS error from the port itself marks it lost
  (`PortLostError`, a `ConnectionError`): the dead handle is closed at once and
  every later call fails immediately, until `Reopen()` opens a fresh handle on
  the same port. Opening sends nothing to the servos.
- The control loop reopens a lost port every 0.1 s (every 0.5 s once the hold
  window below has passed). While the link is down a frame does nothing else,
  every joint is marked stale (`feedback_ok` false), and moves, Hold, Home and
  saving Home are refused with the reason.
- A move in progress when the link drops (a route, stream, calibration or hold
  correction) latches a fault at once: its servos keep their last command, a
  velocity joint keeps its speed, until the link returns. The stop and
  torque-off are sent the moment it does (stop and torque-off only, never a
  position goal), and the arm coasts. This is the existing auto-coast-v1 rule:
  positions that cannot be confirmed fresh end in a confirmed coast.
- A drop while the arm holds still (or coasts) does not latch by itself. If the
  port is back within `SerialLinkHoldWindowS` (2 s), every joint is read again
  (position, torque, hardware status, operating mode) before anything is
  written. The arm goes on holding only if every joint answers, still has the
  torque it should, reports no fault, is in position mode and moved at most
  `SerialLinkHoldDriftRaw` (30) steps; the next frame then reads temperatures
  and status as usual before any write. Anything else latches and coasts.
  Rationale: during the drop the servos keep their own position loop and
  firmware protection, and coasting a healthy, still arm is itself what drops
  it; a still arm can only be trusted once it is read again.
- A coasting arm is read the same way, every joint, after any drop. A joint
  found holding torque all the same (a goal write that went out as the link
  dropped) latches and is coasted, instead of being left energised and
  unsupervised. The status says "the arm is coasting (torque off)" only when
  every joint read back off; a joint that did not answer is named instead.
- A fault a servo reported in the same frame the link dropped (a hardware status
  bit read just before the failing call) is latched before that frame is
  abandoned, even if the bit has cleared when the port returns.
- A drop longer than the window latches "could not reconnect within 2 s: check
  the USB cable". The bridge keeps reopening, and sends the torque-off the moment
  the port returns: the arm then coasts, so support it.
- `SerialLinkDropLimit` (3) drops within `SerialLinkDropWindowS` (60 s) trip
  protection even when each recovered: a flapping link.
- Emergency stop and Disconnect reopen a lost port and retry the torque-off on
  the fresh handle. If the port stays gone they keep the arm attached with the
  coast latched and say so, instead of claiming a coast; the coast is then sent
  when the port returns. With no bus and a torque-off still pending they never
  report "coast and disconnected". Connect sends a pending torque-off at once
  and says so, instead of on its first frame. While the port stays gone the arm
  stays attached, so the restart helper refuses to stop the bridge: plug the
  adapter back in (the torque-off goes out) or connect to its new COM name,
  then disconnect.
- A connect that the link cuts short, after its Goal_Position seeds have
  energised some joints, latches their torque-off, sent the moment the link
  returns.
- A connect to a different port while the old one is lost gives up the dead
  handle; the latched torque-off carries over to the new port. If the arm was
  holding when its port was given up and the new connect fails, its torque-off
  is kept pending too, so Emergency stop cannot claim a coast.
- A plain-words note about a drop ("...; holding at the last position") stays
  the status for 20 s, but Hold or Coast ends it.

What the user sees: the status says what happened in plain words, for example
"USB serial link dropped for 0.4 s; reconnected; holding at the last position",
"USB serial link to the arm dropped 0.3 s ago; reconnecting (the servos keep
holding their last position)" or "USB serial link to the arm dropped and could
not reconnect within 2 s: check the USB cable. The servos keep their last
command until the USB serial link returns ...". The state message carries
`serial_link` (`state` ok/down, `down_s`, the OS `error` and `reopen_error`,
`drops_total`, `drops_recent`, `recoveries`, `failures`, `reopen_attempts`,
`last_outage_s` and the latest `note`), the dashboard's ARM lamp reads LINK
LOST while it is down, and every drop and recovery goes to
`bridge_events.jsonl` as `serial_link_lost`, `serial_link_recovered` or
`serial_link_failed` with those counts (latched faults as `motion_fault`).
`/health` reports `serial_link_version`.

Software cannot reach a servo while the USB link is down: the servos keep their
last command, and only their firmware protection and a hardware power cutoff act
then. The cure for the drop itself is physical (the arm's adapter off the USB4
ports, a short cable with strain relief, no hot-plugging next to it).

These are conservative application thresholds, not manufacturer guarantees.
The servo also has firmware protection. Available feedback is documented in the
[Waveshare ST3215 documentation](https://www.waveshare.com/wiki/ST3215_Servo).

The trip clears every motion stream, holding correction, route and calibration
plan. Velocity goals are stopped first, then torque-off is attempted on every
motor even if one fails, then read-back must confirm all motors are off. The
order matters: on these servos any goal write (speed or position) turns torque
back on. The old order wrote the speed stop after torque-off, so every retry
re-energized the arm; on the hardware it crept under power and Hold refused to
rearm with torque-off unconfirmed. No fresh position goal is written during a
fault because it can clear firmware protection. Unconfirmed joints are named
and the whole sequence is retried while the bus remains available. Software
torque state is disabled regardless of confirmation; the separate confirmation
flag prevents claiming that a broken cable is safe.

A collision check can never trip this by itself during Hold. When a holding joint
sags past its goal tolerance, the stronger velocity correction is used only if its
short return is checked clear; otherwise the servo's own position hold keeps
pulling it back and the status reports the paused correction. The old behavior
latched a fault there, which coasted (dropped) an arm resting beside the mast.

The fault stays latched. Neither Stop, Home nor automatic calibration can clear
it or rearm torque. Resolve the cause, support the arm and explicitly enable Hold.
Rearming checks feedback, hardware status, temperature and torque-off confirmation
before preparing each current-position hold and energizing motors. A partial
enable failure attempts coast again. A requested normal Stop and hold still
retains torque if there is no fault.

Coasting releases the arm, so it can sag or fall. Software cannot guarantee motor
survival or remove power through a disconnected bus, crashed host or failed driver.
It does not replace physical support or a hardware power cutoff.

Offline tests use a fake motor bus: TestMotorProtection.py, TestSafety.py,
test_bridge_home.py, TestMotionSpeed.py and TestBasketCollision.py.
TestSerialLinkRecovery.py runs the real scservo SDK and driver over a fake
serial port (FakeServoSerial.py; only the port names FAKE-COM4 and FAKE-COM5 can
open) and replays 2026-10-01 19:07-19:22: drops mid-read, mid-write, during
torque-off, during Hold and during connect, a fault read just before a drop, and
a port given up for one that fails. No physical arm is moved by those tests. The restart helper refuses
to stop a running bridge while it reports an attached arm or torque on.

## Hand guiding (guide-v1, 2026-10-10)

While hand guiding runs (`hand_guide`, docs/HandGuide.md) its own 100 Hz guide loop owns the servo bus: one SYNC_READ of
every servo a cycle (position, load with its sign in bit 10, supply, temperature, status), every 4th cycle the torque
and Lock read-back, one SYNC_WRITE of the goals and torque limits (GuideBus.py), each cycle under the motor lock. The 30 Hz
control frame does no bus traffic meanwhile: it watches the guide loop, the page that started it and the collision guard
band. Effort is capped every cycle (Torque_Limit 650 / 450 / 250 on lift / elbow / wrist, 400 on the free joints, 750 at
walls), under the servos' 800 overload trip.

**Every fault while guiding makes every joint go limp** (Aaran's decision D1), through `AutoCoast` (torque off on every
joint, read back, retried every frame for a silent one, a latched fault with the reason): a joint silent for 5 cycles, a
servo restart (Lock 0, a turn's jump, torque not as commanded), firmware protection bits, 55 C (the 60 C trip above is
unchanged), the supply under 7.0 V, a guide cycle 150 ms late or the loop gone, the model disagreeing with the URDF, a
fall, over the speed cap, Present_Load not what the goal asks for, the collision check unavailable, Coast and the
emergency stop. A USB drop latches at once (guiding is a move in progress): the servos keep their last goals while the
port is gone, and the torque-off goes out the moment it is back. **Hands off, a limp arm falls**: keep a hand on it.

What holds instead: Hold here, Stop and hold and Hold (the shoulder, elbow and wrist preloaded with the model's effort, so
nothing sags: one SYNC_WRITE with Torque_Limit 1000 and the normal acceleration, read back, then the normal Hold), the
page that started it gone for 3 s, and a start that fails (back to the hold it started from). Entry is refused under
8.5 V, with the servos 0.5 V apart, at 50 C, without a matching calibration, or with the arm moving.

