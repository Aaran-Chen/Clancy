# Motion speed throttle

## Control-loop overhead audit (2026-10-05)

The latest saved timing file contains 39 completed routes, all at 100% with
eight collision-check workers. Median planning was 0.375 s and median flying
was 2.562 s; the longest plan was 7.718 s and the longest flight 12.046 s.
These are different requests, not a before/after comparison. Turning up the
default throttle would not speed up those already-full-speed requests.

The controller was recomputing the synchronized command for all six joints
separately before writing each joint. It now calculates that command once per
locked dispatch and once when entering a new stage, using the same fresh
feedback, throttle, waypoint reach and braking limits. The result is local to
that dispatch, so it cannot survive a frame, speed change, fault or route switch.
Idle holding corrections retain their separate response.

On the simulated pill-hover request in MeasureRouteSpeed.py, eight alternating
old/new runs reduced control CPU time from a median 92.6 ms to 49.1 ms for the
whole route (47.0% less). Synchronized calculations fell from 767 to 140. Both
versions flew for exactly 131 frames, or 4.37 s at 30 Hz. This frees control-loop
CPU time; it does not claim faster physical travel. TestMotionDispatch.py
compares every simulated servo write and measured position against repeated
calculation, across waypoint changes, live throttle changes and nominal,
late, sticky and overspeed servo models.

Experiments that merged more waypoints were rejected: wider independent-joint
regions often lost their waypoint reach, increasing planning or travel time.
The motor speed ceilings, 18% startup, braking acceleration/latency, collision
certificates and protection thresholds remain unchanged. A substantial increase
in physical route speed still requires measured loaded stopping behavior on
the arm before revising the braking model; this audit did not open a serial
port, move the arm or restart the bridge.

Offline dispatch regression:

```powershell
& (Get-Content ".backend-python.txt") "back end/TestMotionDispatch.py"
```

## Throttle behavior

The Motion panel and Camera calibration window share a retro fader. It sets
10% through 100% of the expanded motion range, including during an active move.
100% is meant to sit just under what each servo can follow. The top route
ceiling is 1750 encoder steps/s (25 times the original 70; it was 1050). Joints
with a lower ceiling of their own stop there: shoulder lift 1050, elbow flex
1250, wrist flex 1650 and gripper 1050; shoulder pan and wrist roll use 1750.
The throttle takes its share of the top ceiling but never more than a joint's
own, so it stays linear until a joint reaches its ceiling. The startup setting
is 18%: 315 steps/s for every joint, physically the same as the old 30% of 1050
(the 10% minimum is now 175 steps/s, was 105). Each joint's driver cap equals
its 100% ceiling (feetech_simple JOINT_GOAL_SPEED_LIMITS), so no route command
is silently clipped. The fine-positioning floor, acceleration,
firmware velocity limit, torque limits and idle Hold profiles remain unchanged.
Actual travel time still depends on the approach slowdown, acceleration and load.
It covers Send, manual joint moves, Home and automatic calibration. Use Stop and
hold to stop.

The setting is the cruise speed of the joint with the longest time to go; the
other moving joints are slowed so they all arrive together, and a joint that
gets ahead of the slowest one is eased off. Routes pass their intermediate
waypoints without stopping, so the slowing approach happens once, at the route
end (or where a joint's own direction reverses), not at every waypoint. Near a
waypoint a joint is also capped at the speed it could still stop from inside
its checked overshoot: 32 steps plus the 16-step allowance, allowing 0.18 s of
latency, 1000 steps/s² of braking and 20% speed overshoot; or, at a tight
waypoint next to an obstacle, only the allowance, allowing a lurch to twice the
command. See Cad/BasketAvoidance.md, Execution. Final targets and direction
reversals also use this braking bound, with only the tracking allowance
available beyond the endpoint. The cruise ceiling therefore increases without
overrunning the checked stopping region. The joints are timed together at
these caps, not at their ceilings (route_executor_version synchronized-v1):
at 100% the braking cap, not the ceiling, sets the lead joint's pace, and each
joint used to brake on its own, reach the waypoint early and stand still there.

Changing speed never starts a motion, changes a goal, clears a fault or enables
torque. Idle holding corrections retain their existing response. Basket route
certificates cover independent progress of every joint, so adjusting speed does
not invalidate the checked route. Final position corrections retain a nonzero
profile speed; zero would mean unlimited speed in position mode.

Stage and calibration settling budgets account for the selected speed. A live
change rescales only the remaining budget. Moving the fader back and forth
does not restart the watchdog, and an expired deadline remains expired.

The bridge owns the setting. Page reloads read it back without overwriting it;
a bridge process restart returns to 18%. Older bridges disable the fader until
they advertise speed_control_version = throttle-v2. Commands use
set_motion_speed with a numeric percent from 10 to 100. Telemetry includes
motion_speed_percent and motion_speed_maximum_multiplier = 25. UI writes are coalesced, stale telemetry cannot snap an
active edit backwards, and unacknowledged edits revert to the last reported value.
A matching telemetry value only confirms a change after its send timer has run;
otherwise a rapid 100 -> 10 -> 100 edit could lose the final update. Send, joint
moves and Home flush a waiting speed update onto the same WebSocket before the
motion command, so a quick move starts with the latest selection. Stop and
emergency commands remain immediate.

Offline checks (no serial hardware):

```powershell
& (Get-Content ".backend-python.txt") "back end/TestMotionSpeed.py"
```

From AI Controlled WebApp:

```powershell
node TestMotionThrottle.mjs
```


Command-order regression (simulated bridge only):

```powershell
node TestMotionSpeedOrdering.mjs
```


What 100% is worth changed with the route work, not with this ceiling: on the
40-case servo model a route at 100% now averages 8.5 s where it averaged 10.8 s
(median 11.4 s to 8.3 s). That came from certifying the waypoint reach with 64
subdivisions instead of 8, so waypoints are flown past rather than stopped at,
and from a 180-step final approach ramp instead of 320. See Cad/BasketAvoidance.md.
The braking model itself was left alone: the logged hardware decelerations
(median 577 to 820, peak 1417 steps/s^2) do not justify assuming more than the
1000 steps/s^2 it already assumes.

TestMotionSpeed.py verifies the per-joint ceilings, endpoint braking, the 18% (315 steps/s) startup, unclipped driver
commands, and speed increases and decreases after a simulated move has already
begun, without replacing the route or changing its targets or torque. Protection,
Home and basket-route regressions remain required when changing these limits.

## Where the 100% ceilings come from (2026-09-30, provisional)

Nothing has been commanded above 1050 on the arm and no speed trace was kept, so
the ceilings come from the servo spec and the arm's static loads, not a
measurement. The servos are STS3215 (model 777, firmware 3.10). The Waveshare
ST3215 (12 V) is rated 45 rpm no-load (3072 steps/s) and 30 kg.cm stall at 12 V,
with a 3073 steps/s maximum speed setting. The servos report about 8.9 V
(Present_Voltage 89 in the 2026-09-28 elbow fault record) without the voltage
warning bit, which fits the 12 V model run below 12 V. Scaled to 8.9 V that is
about 2280 steps/s no-load and 22 kg.cm stall. A linear torque-speed curve at
each joint's worst static gravity moment (the link masses of the bridge's
shoulder-load estimate, which was removed on 2026-10-03, plus a 100 g object at
the jaws) leaves about 950 steps/s for the shoulder lift
(12.9 kg.cm), 1500 for the elbow (7.6), 1970 for wrist flex (3.0) and 2280 for
the unloaded joints. The ceilings sit 16-23% under those, except: shoulder
lift keeps the 1050 it has run at since 2026-09-28 (it may fall short when
lifting a fully stretched arm, which only slows the route), and the gripper
keeps 1050 because it closes onto objects.

A joint that cannot reach its ceiling does not trip anything: protection checks
progress, not speed (MotorProtection.md), routes are monitored against boxes,
and the timing and leash follow the slowest joint. The firmware velocity limit
(register 84, written 51 and 102 for the shoulder lift) is not in the public
register map; if its unit is 50 steps/s (0.732 rpm) it allows 2550, above every
ceiling. Measure each joint on the arm before raising any ceiling.

What 100% is worth on routes: on the servo model with the joints capped at the
speeds above, home to table still takes 7.9 s, mast to home 4.1 s and a 1141-step
pan move 8.7 s, the same as with the 1050 ceiling. Their commands peak at
about 390-550 steps/s because the waypoint braking cap (0.18 s, 1000 steps/s^2, 20%)
binds first; a ceiling only matters on stages longer than about 1000 steps.
Faster routes need that braking model raised, which needs measured stops on the
arm (it assumed 1500 until a route tripped on the arm on 2026-09-15). For scale, on the
model alone: 2000 steps/s^2 gives 6.7 s home to table, 3000 with 0.12 s latency 5.5 s.

## Where a route's time goes on the arm (2026-10-04)

The 27.3 cm move of 2026-10-04 15:31:57 (Home to the hover beside the pill
bottle, opening the jaws on the way, 100%) took 9.7 s from approval to its
check-in. Read off the depth camera video: about 2.3 s planning before the arm
moved, about 7.4 s flying, the check-in at once after. On the servo model the
same route plans in 1.3 s offline and flies in 5.3 s: 15 stages, the lift's
command rising to about 420 steps/s after each waypoint and braking to about
200 before the next (peak 516 of its 1050 ceiling), the braking cap above at
every one. With the reach clock cut in half, as the live bridge's shared CPU
did, it flew 6.1 s. The 4.4 cm descent after it took 2.5 s: 1.2 s planning,
0.8 s flying, then the app's old 1.5 s check-in floor.

Planning now runs its collision checks in worker processes (back end/
RouteCheckPool.py, Cad/BasketAvoidance.md), and the check-in floor is 0.3 s.
Every completed route is kept, with its plan time, flying time, each stage's
time and every control frame, in captures/<date>/recording/route_timing.jsonl
(RecordRouteTiming), so the next measurement comes from the bridge, not the
video. On the servo model the braking assumption is what is left: at 100% the
27.3 cm route flies in 5.3 s at 1000 steps/s^2 (no trip on any harsh model),
4.7 s at 1500, 4.3 s at 2000 (harsh models that brake at only 1000 trip 5 times on 4 of
19 routes; harsh models that brake as assumed, none), 3.3 s at 3000 with
0.12 s latency (trips on the matched models too). Measure the stops on the arm
(.claude/SpeedMeasurement/MeasureJointSpeed.py, bridge stopped) before raising
ROUTE_BRAKE_ACCELERATION or lowering ROUTE_BRAKE_LATENCY_S. The arm's push on
the cart stays small either way: about 0.6 N with the extended arm's centre of
mass braked at the full Acceleration register (5100 steps/s^2, which the servos
already use whenever a command steps up), against the cart's weight.
