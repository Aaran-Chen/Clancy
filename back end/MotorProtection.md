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

- Poll temperature at most every 250 ms; trip at 60 C once every poll has read
  it for 1 s (since 2026-10-22, below). Explicit Hold rearming requires
  temperatures below 55 C and healthy status/position reads.
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

What the records showed (2026-10-13..25, 0.4-6.8 s, every one with nothing moving) and what changed:
- "the motor lock, held by asyncio_N": that snapshot, its stack in `WorkspaceOffset` or `ConfigurationKey`
  reading `WorkspaceFrame.json` (a running route read it every frame too, in `CheckMotionRoute`). The file is
  now cached by mtime and size (`JsonFileCache`), read only off the lock (each snapshot before taking it, and
  the `so101-file-watch` thread every 0.25 s, so a mid-route edit still stops the route holding); the snapshot
  writes its configuration key after the lock. `MeasureLockHolds.py`, every file access 200 ms slow: a
  snapshot held the lock 1406 ms, now 0.3; a route frame 603 ms (a heartbeat trip), now 0.3.
- "the CPU or the interpreter lock": the Orbbec open (`OrbbecCamera.py` Context, device query, pipeline
  start) holding the interpreter lock. ad5a7a8 skipped opens only while the arm moved; now an open and a
  motion never overlap (`DepthCamera.OpenLock`/`Opening`, `BridgeController.CameraMustWait` and
  `MotionStartGate`): no open while a motion is checked, planned or runs, and a motion asked for during an
  open waits for it (up to `CameraOpenWaitS`), then is refused.
- 6.1-6.8 s about 6 s after each bridge start: the check workers' spawn (RouteCheckPool) and the first
  camera open together. No motion starts until both ended (`StartupPending`; the refusal says the bridge is
  still starting).

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
  before, and since 2026-10-22 must also persist (below). The 60 C trip and 55 C
  rearm limits are unchanged.
- Status and torque: a reading of Hardware_Error_Status and Torque_Enable that
  shows anything (a fault or warning bit, torque off while it should be on, or a
  failed read) is read again at once, before anything is written. Only two bad
  readings in a row latch: the fault, or "Cannot read <joint>
  protection/torque status" when both failed. A reading whose telemetry no servo
  can report (`ServoTelemetryImpossible`: Present_Load beyond 0-1000, or a
  supply under 6.0 V with 85 C or more; until the round 1 review also a supply
  over 14.0 V), or whose telemetry did not read in full, is a corrupted reply:
  since 2026-10-22 it never latches on its own, even read twice or every frame
  (see below). A servo that really restarted
  (brown-out) keeps torque off on the re-read and is caught in the same frame:
  the added delay is one more reading, a few milliseconds (about 70 ms when both
  reads time out). Fault records now also carry Lock (0 after a restart).
- Corrupted status replies (2026-10-22, `BridgeVersion`
  corrupt-status-glitch-2026-10-22): two corrupted readings in a row (status
  129, load 64001, 0 V, 90 C) latched "Cannot read shoulder_pan
  protection/torque status" and ended a task, every joint exactly on its
  target. Now a reading that is corrupted, re-read corrupted or unanswered, is a
  `protection_glitch` (one per run of such frames, in state.protection.glitches
  and bridge_events.jsonl), never a fault by itself (`CorruptStatusPersists`).
  Since the round 1 review (below) the streak confirms over time and over
  independent reads. A position read failure still latches "Lost joint
  feedback" at once, a hold drift past 512 steps still latches, and a route
  still has its monitor and timeout. Two plain failed readings in a row (no
  reply at all) still latch "Cannot read" at once, and a fault bit or torque off
  read twice with telemetry a servo can report still latches that fault. The
  `ProtectionVersion` string stays auto-coast-v2-confirmed (the page checks it).
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
immediate re-read on the arm; the `protection_glitch` records will show it. On
2026-10-22 the same reply came back on the re-read (two in a row); since then a
corrupted pair never latches on its own (below).

### Confirmed over time (protection-confirmed-v3, 2026-10-22)

`BridgeVersion` protection-confirmed-v3-2026-10-22. An adversarial review of the
corrupt-status change (round 1, 13 confirmed findings) found it both still
false-tripped and missed real faults. The rule now: corrupted telemetry alone (an
impossible value, one unconfirmed bad read, a failed telemetry read) never stops,
coasts or ends a task anywhere, and every real fault still latches promptly.
`ProtectionVersion` stays auto-coast-v2-confirmed (App.tsx checks the string).
Tests: `TestProtectionConfirmation.py` (one per finding, and the exact
2026-10-22 signature on the fake bus and on the real driver),
`TestMotorProtection.py` (the temperature trip). A second review round found 4
more (a restart masked by the re-assert, the hot window kept through a streak,
one corrupted Operating_Mode read after a delay, and a preflight refusal on one
unconfirmed pair); each is fixed below and has its own test. A third round found
5 more (a restart hidden by another goal write, an overheat hidden by intermittent
corruption, a fault reading paired with an unanswered one, an unanswered read
after a delay, and a re-assert goal taken from a corrupted position); see "Round
3" at the end of this list.

- What is corrupted: a status reading whose telemetry no servo reports (load
  over 1000, or under 6.0 V at 85 C or more) or whose telemetry did not read in
  full (a failed Present_Load read used to make a torque-off reading look
  plausible, and it latched). A pair with a corrupted reading in it confirms
  nothing, whatever the other reading said, in either order.
- Over-voltage: a supply over the 14.0 V rating is a value a servo can report
  (one byte of dV, up to 25.5 V); it is a real fault (a wrong power supply),
  latched as the voltage bit once the re-read confirms it. It used to read as
  corrupted, and with the joints holding it never latched.
- The streak (`StatusCorruptStreaks`): per joint Since, LastSeen and where the
  joint was. A gap over `StatusStreakGapS` (0.25 s) starts a new one; a clean
  reading ends it, and so do Hold, Coast, Stop, a latched or cleared fault,
  AutoCoast, the emergency stop, a disconnect and the end of hand guiding (a
  stale start skipped the next streak's window and its glitch record). One
  `protection_glitch` per streak, with the telemetry.
- Torque read off: Torque_Enable is its own register, read before the
  telemetry, so corrupted telemetry no longer throws it away. It is read again on
  its own (twice). Read on: nothing. Read off: re-asserted once per streak with a
  goal write at the joint's present position (any goal write turns torque back
  on; a gripper holding an object gets its squeeze again instead), recorded as
  `torque_reasserted` (a visible event, and in `state.protection.glitches`),
  and read once more. Not where that is unsafe: no fresh position, a protection
  unload bit (a goal write would clear it), a streaming joint, velocity mode, or
  a Lock that does not read 1 on its own re-reads (twice). Hold and connect set
  Lock 1 on every joint; a restart (brown-out) leaves Lock 0. Round 2: a real
  restart whose one frame of telemetry read corrupted was re-asserted (the goal
  write energises a restarted servo), so nothing latched, preflight passed and
  the joint stayed unlocked, its Torque_Limit and profile lost, until the next
  route tripped "motion write failed" mid-move. Now it is never re-asserted: its
  next plausible pair latches, or the 0.3 s confirmation does. It latches
  "torque unexpectedly off" once it has read off on these re-reads for
  `StatusTorqueOffConfirmS` (0.3 s): a pan or a wrist roll limp on target, which
  no position test can see, used to never latch.
- A fault bit read with plausible telemetry in two frames of a streak (each
  paired with a corrupted reply) is confirmed and latches.
- Lost hold: a holding joint latches once it is `StatusCorruptHoldDriftRaw`
  (30 steps, as `SerialLinkHoldDriftRaw`) further from its hold point than it
  was when the streak began, or when its hold point last changed, on two frames
  in a row. It used to be judged against the 12-step goal tolerance, the very
  deadband where an ordinary hold correction starts, so a joint resting off
  target (a correction paused beside the mast) or sagging into a correction
  latched after 2 s, and a joint that really fell waited 2 s.
- Grasp: a gripper holding an object (GraspHold) has its target follow its
  jaws, so position cannot show a lost grip: it latches once the streak has
  lasted `StatusCorruptHoldS` (2 s).
- Route: a timed motion has no hold point to judge by and does not carry on
  blind: past 2 s it stops and holds where it is (`HoldRoute`, torque on, a
  `motion_abort`), and the streak goes on from the new hold point.
- Cap: every streak latches "Cannot read ... corrupted for 5.0 s (limit 5 s)"
  past `StatusCorruptMaxS`, whatever the position (and, since round 3, a chain of
  streaks with corruption on most frames). 5 s is 150 frames of
  corrupted replies against the longest on record of 2; it bounds to seconds how
  long a fault the corruption hides (an over-voltage, a fault bit) goes
  unlatched, while the servos' own unload protections act regardless and
  heating takes minutes.
- `MotionPreflight` refuses a route or Home while a joint's status is in an
  active streak: "Cannot move: <joint> status replies read corrupted; the arm
  holds". Round 2: one corrupted pair read by the preflight itself refused the
  move, and the app counts a bridge refusal toward ending the task. Now those
  joints are read again first, a fresh position and status pair every
  `StatusPreflightIntervalS` (40 ms) up to `StatusPreflightReads` (5) times,
  about 0.2 s under the motor lock with the arm still: the first clean pair ends
  the streak and the move starts; a streak still active after them refuses.
- Temperature: an over-limit reading whose re-read fails is a
  `protection_glitch`, read again at the next poll (one corrupted reply plus one
  missed reply coasted the arm). A confirmed over-limit reading trips once every
  poll has read it over the limit for `ProtectionHotConfirmS` (1 s, 4 polls; a
  poll gap over 0.75 s starts again), and not while that joint's status is in a
  corrupted streak (the 2026-10-22 corruption read 90 C, and the 0.25 s poll
  coasted the arm the 2 s window was meant to keep); the streak's 5 s cap bounds
  that. Round 2: the count kept its start from inside the streak, so the first
  poll after the streak ended tripped. A poll in a streak then started the count
  again (until round 3, below: now only a poll that looks like the corruption is
  left out). Real heating takes minutes against the 22 C margin.
- Control delay and serial link drop (`ReadEveryJoint`): a holding joint whose
  one read shows torque off or a status bit is read again (two status readings);
  only a problem both confirm with plausible telemetry, or two unanswered
  readings, counts. Anything else is a `protection_glitch`, and the frame's own
  status check reads it again before any write. Round 2: Operating_Mode too: a
  read other than position mode (or no answer) while holding is read again
  twice, and is a problem only when both re-reads read another mode, or neither
  answered (one corrupted byte, 28, coasted a holding arm after a delay).
- Missing position feedback is not judged by the streak: `_read_positions_locked`
  latches "Lost joint feedback" first in every frame (the streak's "position did
  not read" branch could never run, and its test set FeedbackOk by hand).
- Round 3 (2026-10-22, each finding confirmed by two skeptics; tests in
  `TestProtectionConfirmation.py`, the hot-count rule also in
  `TestMotorProtection.py`):
  - Restart check (`StatusRestartChecks`, `CheckRestarts`): a real restart whose
    one frame of telemetry read corrupted (or lost one telemetry packet) was
    never latched when a hold correction (a sagging joint) or a route's own goal
    or speed write energised it before the 0.3 s confirmation: the next frame read
    clean, ended the streak, and the joint carried on with Lock 0 and the
    restart's default registers. Now torque read off on the joint's own re-reads
    is remembered until its Lock, read on its own twice once torque no longer
    reads off, reads 1. Lock 0 twice latches "<joint> restarted"; neither 0 nor 1
    for `StatusRestartConfirmS` (1 s) latches as unconfirmed. Only the bridge
    closes Lock (Hold, connect), so a burst cannot make a restarted servo read 1.
    Meanwhile the bridge's own hold-correction and route writes skip that joint
    (so a restart usually latches "torque unexpectedly off" a frame later; since
    round 4 a route holds and a streaming correction is stopped first), the
    check is kept through a Stop that holds, and `MotionPreflight` refuses.
  - Temperature: a whole streak no longer discounts a hot poll. Only a poll that
    looks like the corruption is left out of the 1 s count, neither counting nor
    restarting it: a re-read more than `ProtectionTemperatureAgreeC` (3 C) from
    the read, or, while the joint's status is in a streak, 85 C or more (the
    2026-10-22 replies read 89-90 C). A steady 65 C under status corrupted on
    most frames (clean frames between) never tripped; it now trips in about 1.3 s.
  - Cap over a chain: a clean frame ends a streak, so corruption on most frames
    with a clean one between never reached the 5 s cap. Now a chain of streaks
    (`StatusCorruptChains`) goes on until the joint has read clean for
    `StatusCorruptChainGapS` (1 s), and latches past `StatusCorruptMaxS` once
    corrupted frames are at least half of its frames in the last 5 s. One
    corrupted frame in 8 (most frames clean, where a fault shows) never latches.
  - A fault reading (torque 0, or a fault bit, with telemetry that read clean)
    paired with an unanswered reading, in either order, latched at once. That
    pair now goes to the streak like a corrupted one: torque is read again on its
    own (and confirmed over 0.3 s), a fault bit needs a plausible reading in a
    later frame. A real restart or over-voltage latches one frame later.
  - `ReadEveryJoint` (control delay, serial link drop): while holding, an
    unanswered Torque_Enable or Hardware_Error_Status read goes to two fresh
    status readings (`ConfirmedStatusProblems`), and an unanswered
    Present_Position read is read once more; only failures the re-reads repeat,
    or faults they confirm, are problems.
  - Re-assert goal (`ConfirmedReassertRaw`): the goal is the joint's
    Present_Position read again on its own twice, both within
    `StatusReassertAgreeRaw` (8 steps) of each other and of the frame's reading,
    and within 30 steps of where the streak began, its hold point or its target.
    Otherwise it is not re-asserted ("its position did not confirm") and a torque
    that stays off latches 0.3 s later. One corrupted position read (28 on a pan
    at 3479) had been written as the goal, 645 steps away; the grasp squeeze uses
    the confirmed position too. The closed-loop hold's 512-step drift latch reads
    the position again the same way (`HoldDriftConfirmed`) before latching, as
    that one read also latched "drifted 645 steps".
- Round 4 (2026-10-22, confirmed by two reviewers and three of four skeptics;
  tests in `TestProtectionConfirmation.py`): while a restart check was pending
  the bridge only skipped the joint's writes, so a joint streaming in velocity
  mode kept its last Goal_Speed with nothing supervising it for up to
  `StatusRestartConfirmS` (1 s), longer while its torque re-reads went
  unanswered, and on a route the other joints went on without it (mid-Home,
  shoulder_lift ran on at -315 steps/s for the 10 frames its Lock read 28).
  Now nothing moves unsupervised while a check is pending:
  - A route, calibration or other timed motion holds where it is in the frame the
    check starts (`HoldRouteKeepingStreaks`: `HoldRoute`, torque on, a
    `motion_abort`, never a fault; the streaks, their chains and the check are
    kept). Stop's per-joint sequence stops the moving joint first (Goal_Speed 0
    when its mode reads velocity, else 1) and then holds it at its present
    position, so it cannot jerk.
  - A hold correction streaming into that joint is stopped with one Goal_Speed
    write (`StopForRestartCheck`) and nothing else: 0 when its Operating_Mode
    reads velocity on its own twice (velocity mode: stop), else 1 (position mode:
    the slowest profile toward its goal, which a restart sets where it is;
    velocity mode: 1 step/s). 0 to a position-mode servo would be full speed, and
    a real restart reverts the mode to the EEPROM's position mode. Its stream is
    kept and resumes once the check ends; other joints' corrections go on.
  - A goal-block write energises a restarted servo, the stop write included, but
    nothing the bridge writes there touches Lock: the next frame reads torque on,
    `CheckRestarts` reads Lock 0 twice and latches "<joint> restarted" (on the
    fake bus and the real driver, under a hold correction and mid-Home). A stop
    write that fails latches at once.
  - Today's exact signature (shoulder_pan status 128 then 129, load 64001, 0 V,
    90 C, Lock 28, every joint on target) still never latches and never stops a
    hold. With three torque-off reads no check starts and a route goes on; with
    four (both re-reads off) on a streaming joint, a hold correction pauses for a
    frame and a route holds where it is (not a fault), its Lock then reads 1.
- Round 5 (2026-10-22, confirmed by three reviewers and every skeptic; tests in
  `TestProtectionConfirmation.py`): the route hold round 4 runs in the corrupt
  burst's own frame (`HoldRouteKeepingStreaks` -> `HoldRoute` -> `StopMotion` ->
  `_stop_slider_velocity_locked`) read the pending joint's Operating_Mode and
  Goal_Position back once. `num_retry` only retries a missing reply, not a wrong
  value, so one more garbled read-back (goal 28 on the pan at 3212) raised "Hold
  read-back mismatch", Stop put it in its errors, and `AutoCoast` coasted every
  joint mid-Home with no fault (reproduced offline, today's 4-read burst plus one
  garbled goal). Now, for a joint in an active streak or a pending restart check
  (taken before Stop ends the streaks), the hold is tolerant:
  - A read-back that disagrees (garbled, or no reply) is read again on its own
    twice (`ConfirmHoldReadBack`); a re-read that reads what was written stands
    the hold, recorded as a `protection_glitch` (register, the reads, the hold
    point). Only when both re-reads also disagree does it raise and Stop coast,
    the same confirm rule as `HoldDriftConfirmed`, `ConfirmedReassertRaw` and
    round 2's Operating_Mode rule. A joint that really never took position mode
    reads it on every re-read and still coasts.
  - A write of that hold that fails (a garbled or missing status reply) is written
    once more, the same value (`WriteTolerated`); only a second failure raises.
  - Its mode before the stop is read on its own twice (`ReadsVelocityTwice`, as
    `StopForRestartCheck`): Goal_Speed 0 only when both read velocity, else 1, so a
    garbled or unanswered mode read neither coasts nor writes 0 (full speed) to a
    position-mode servo.
  - Every other joint, and every other caller of the hold (hold corrections and
    routes, the grasp squeeze, Hold), keeps the strict single read-back.
  - A real restart is unchanged: the tolerated hold's goal write energises it,
    `CheckRestarts` reads Lock 0 twice and latches "<joint> restarted" (real
    driver, mid-Home, with a garbled read-back in that hold). Today's exact
    signature still never latches or stops a hold, with or without the garbled
    read-back (goal 42 or 28, mode 33 or 28).
- Round 6 (2026-10-22, BridgeVersion `protection-confirmed-v3.1-2026-10-22`; the
  findings round 5's review confirmed, `.review-round5.md`; tests in
  `TestProtectionConfirmation.py`, fake bus and the real driver):
  - Mode read-back (finding 1, safety): round 5's tolerant read-back stood the
    hold when either re-read read what was written. Position mode is 0, the
    likeliest garble, so a joint that really stayed in velocity mode (its
    Operating_Mode write acked but not taken) with one re-read garbled to 0
    (reads [1, 0, 1] or [1, 1, 0]) was held, and the hold then wrote Goal_Speed =
    the profile speed (200) to a joint in velocity mode, nothing supervising it
    after. Now Operating_Mode stands only with no velocity read among the
    re-reads, and, when the first read was velocity, both re-reads reading
    position. Those reads raise "Hold read-back mismatch" and Stop coasts; no speed
    above 1 reaches the joint. A position-mode joint whose read-back is garbled
    once, to velocity ([1, 0, 0]) or to anything else, still holds.
  - Route envelope (finding 2): one garbled Present_Position read mid-route
    latched "<joint> left the checked basket route" and coasted every joint, also
    in the corrupt burst's own frame, before round 4's hold ran. Now a joint read
    outside its checked envelope has its position read again on its own twice
    (`RouteExitConfirmed`, as `HoldDriftConfirmed`). A re-read inside the envelope
    is a `protection_glitch` and is kept as the joint's position, so the route,
    or the hold that follows in that frame, works from a confirmed position (the
    burst's hold goal is where the joint really is, never the garbled read). It
    latches as before when no re-read lies inside: the two agreeing outside (it
    really left the route), and also, a deliberate choice, when they agree with
    neither each other nor the route or go unanswered, since then no position
    confirms and holding or routing from a garbled one would jump the joint.
    The latched fault's details carry the re-reads (`reads`).
  - Grasp squeeze (findings 3 and 4): a gripper holding an object (GraspHold)
    under a streak or a pending restart check got round 5's tolerant Goal_Speed
    write, but its hold went through `SqueezeGrasp`, which kept the strict single
    read-back: one garbled goal or mode read-back coasted every joint mid-carry and
    dropped the object. `SqueezeGrasp` now takes `tolerant`, which `StopMotion`
    passes for a suspect joint; the grasp, the re-assert and Hold keep it strict.
    The grasp is kept (GraspHold, the 8-step squeeze from where the jaws rest) and
    the check ends once its Lock reads 1; the same read-back garbled on all three
    reads still coasts.
  - Today's exact signature (status 128 then 129, load 64001, 0 V, 90 C, Lock 28,
    every joint on target) still never latches or coasts: mid-Home, mid-carry
    (the burst on the gripper) and with one extra garbled read (a hold read-back
    or the frame's position). A joint really in velocity mode, a position off the
    route that the re-reads confirm and a real restart still latch.
  - Offline tests that probe the route monitor by setting a joint's position by
    hand (`TestFlyByRoutes.py`'s `MonitorInsideCertificates` and three tests that
    use `ProbedPositions`) now have their bus answer that position too, as a joint
    really there would; otherwise the re-read would answer the bus's own position.

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

