# Hand guiding for the SO-101 follower: the design as it will be built

2026-10-10, branch `gravity-hand-guide` (from `codex/generalized-grasp` at 2adf0b6). This is
`.claude/Gravity/GravityDesign.md` with Aaran's decisions of 2026-10-10 folded in; its numbers and reasoning are kept
where nothing changed. Every change is marked **[Changed: Dn]**, where Dn is the decision below that caused it, or
**[New]** for something the decisions forced that the old design did not have. Nothing here was sent to the arm, the
live bridge (127.0.0.1:8000) was not touched, and every number comes from the offline plant in
`back end/HandGuideSim/` (section 7).

**Aaran's decisions (2026-10-10)**

| # | Decision | What it changes |
|---|---|---|
| D1 | Servo or USB dropout, or any fault while guiding: **go limp** (coast, torque off), not hold. E-stop also coasts. | Every fault exit coasts (3.8). The UI says the arm drops. |
| D2 | Hand guiding from **any device** (this PC, phone, laptop with the remote password). | Entry accepts `Access` "password" as well as "local" (3.7, 6). |
| D3 | **Pan and wrist roll swing free** (no position hold). | They float (torque off) at rest; walls still catch them (3.2, 3.9). |
| D4 | **Everything free motion, with a little help**: in Coast the roll and gripper feel too slow. | Every joint gets an assist; pan, roll and gripper float and are helped only on slow, steady moves (3.4); the friction feed-forward is in the first build on all six joints (3.3, 3.10). |
| D5 | No physical shoulder spring. Undecided items take the old defaults. | No payload compensation; no camera capture while guiding (6). |

Names: lift = shoulder_lift, elbow = elbow_flex, wrist = wrist_flex, pan = shoulder_pan, roll = wrist_roll. "Comp
joints" are lift, elbow and wrist (they carry gravity); "free joints" are pan, roll and gripper. Duty units: 1 unit =
0.1 % of full duty; 1000 = stall torque at the present supply (2.16 N m at the 8.9 V the servos report, 2.91 N m at
12 V). "Steps" are encoder counts (4096 per turn). Forces "at the hand" use these levers: lift 35 cm, elbow 25 cm,
wrist 12 cm, pan 30 cm (hand at the gripper), roll 3 cm (fingers on the gripper body), gripper 5 cm (on the jaw tip).

---

## 1. In plain words (for Aaran)

**What it is.** A **Hand guide** button in Clancy's Manual mode, on this PC or any device with the remote password
**[Changed: D2]**. With the arm holding, press it and keep your hands off for about 2 seconds while it starts. Then:

- the shoulder, elbow and wrist carry their own weight: push them wherever you like and they stay where you leave them;
- the base (pan), the wrist roll and the gripper go limp, exactly like Coast **[Changed: D3, D4]**, and get a small
  push in the direction you are moving them when you move them slowly and steadily.

**What it feels like.** The shoulder, elbow and wrist feel as in the old design, a little lighter: a steady push of
about 0.4 to 2.8 N at the gripper moves them at a calm pace (simulated, depending on the unknown gear friction),
against 6 to 15 N to push through an ordinary Hold. The pan, roll and gripper feel like Coast, except that slow, steady
moves (under about 20 to 45 deg/s, depending on how much friction they turn out to have) take about 30 % less force
(up to 38 % on a stiff joint). **Quick flicks feel exactly like Coast**: the assist stays out of the way. If a joint
turns out to have little friction there is nothing worth helping, and it is simply Coast.

**Why only a little help on the roll and gripper.** In Coast the motor is disconnected and floats; what you feel is
the 345:1 gearbox's friction and the motor's own inertia, which the gearbox multiplies by 345 x 345. To help, the servo
has to be switched on, and a switched-on motor brakes itself with its back-EMF (0.57 N m s/rad at the joint: on the
roll that is about 20 N at your fingers at 60 deg/s). The software cancels most of that brake, but not all of it
safely, and the speed reading it needs lags about 25 ms, which makes a switched-on joint feel heavier while you speed
it up. So the help only wins on slow, steady moves, and the code switches it on only there. If bring-up finds that the
servo brakes even in Coast (step S2), the help works at every speed and is much bigger (67 to 77 % less force).

**Its limits.**
- Speed: the shoulder is capped at 45 deg/s (about 0.3 m/s at the gripper). Above about 70 % of the cap it brakes;
  going faster than the cap **makes the whole arm go limp** **[Changed: D1]**.
- Walls: near the slider limits and near the basket, the cart body and the camera head the arm resists and holds its
  last clear pose; pan, roll and gripper are caught there too. A firm push still gets through.
- Strength: each joint's effort is capped every cycle, under the servos' own overload trip.

**How to stop.**
- **Hold here** (the Hand guide button turns into it), Stop and hold, and Hold: the arm holds exactly where it is.
- **Emergency stop: the arm drops**, as today; while guiding its label says so.
- **Anything going wrong makes the whole arm go limp** **[Changed: D1]**: a servo not answering or restarting, the
  USB cable, a servo at 55 C, the supply under 7 V, the control loop pausing, the model check, a fall, over-speed.
  **If your hand is not on the arm, it falls**: simulated from a stretched-out pose, the shoulder drops about 25 deg in a
  quarter of a second and onto whatever is below it within half a second. With a hand under the forearm it moves 4 to
  6 deg. Keep a hand on or under the arm while guiding.
- Closing the page (or a phone screen going to sleep) for 3 s ends guiding **holding**, not limp (3.8).

**Before first use.** A one-off calibration (**Calibrate hand guiding**, about 3.5 minutes) sweeps each joint slowly
along checked routes while you watch. It measures the effort each pose needs, how much each gear drags, and whether
the servos float or brake when switched off. Hand guiding refuses to start without it.

**Start conditions you will see.** It refuses to start, with the reason, if the servo supply reads under 8.5 V
**[New]** (the supply has browned out before), a servo is at 50 C or more, the calibration is missing or stale, the
arm is moving or outside its slider arcs, or the gripper is squeezing an object (3.7).

---

## 2. What the hardware allows

### 2.1 Servo facts that decide the design

(Unchanged from GravityDesign.md 2.1 unless marked.)

- **No torque mode** on factory firmware 3.10. Operating_Mode 0 is position (the arm runs in it); 2 is open-loop PWM;
  1 is speed. Writing a different Operating_Mode clears Torque_Enable and zeroes Goal_Speed and Acceleration, and
  restarts the multi-turn count. **No mode switch is used anywhere in this design.**
- **Position loop** (about 1.4 kHz): duty = P/4 x error + D/8 x error change over 20 ticks, plus the punch (reg 24),
  clamped to +-Torque_Limit (reg 48, RAM) and +-970. So **Torque_Limit is a hard cap on the duty**, and a goal placed
  (effort - punch) / (P/4) + dead zone steps past the present position makes the servo apply that effort.
- **Firmware protection**: overload (status bit 5) after more than 80 % output while not moving for Protection_Time,
  then output limited to 20 %; overcurrent (bit 3, 2.0 A for 2 s) clears torque. **No watchdog**: a servo keeps its last
  goal for as long as the host is silent.
- **[New] The drive at low duty brakes; torque off is expected to float.** The H-bridge runs slow-decay PWM (clean-room
  firmware `pid_motor.c`, `timer0_set_duty`), so an energised motor sees the duty's average voltage minus its back-EMF:
  at duty 0 it is short-circuited, a brake of kt ke / R = 0.57 N m s/rad at the output. With Torque_Enable 0 the
  clean-room README says the motor coasts (floats); whether it really floats or brakes is measured by bring-up S2 and by
  the calibration's spin-down (5, step 6). Everything about the free joints' assist follows from this (2.3).
- **[Changed: verified] Present_Load sign bit.** `feetech_simple.py:99-106` lists Present_Load in `SIGNED_MAGNITUDE`
  and decodes it with **bit 15** (`scs.SCS_TOHOST(value, 15)`, line 276). The servo's direction bit for load is **bit
  10**. Evidence from the arm itself: the 2026-09-28 elbow overload record
  (`back end/captures/2026-09-28/selfcal/runs/20260928_135935_f6fec78880d2/Manifest.json`, bits 32 = overload) reads
  `"load": 1224` at 8.9 V; 1224 = 1024 + 200, i.e. bit 10 set and magnitude 200, exactly the 20 % output the
  firmware's overload protection allows. Decoded with bit 15 it reads +1224, more than full duty, which is impossible.
  The fix (G1): decode Present_Load with bit 10 (and only Present_Load; position, goal and speed keep bit 15), with a
  test that 1224 decodes to -200 and 200 to +200.
- **Loads** (URDF, bare arm, worst over the soft box): lift 0.864 N m, elbow 0.453, wrist 0.117 (with 100 g in the
  jaws 1.268 / 0.743 / 0.274): 400 / 210 / 54 duty units at 8.9 V. Pan, roll and gripper carry no gravity torque worth
  the name (pan 0, roll at most 0.006 N m, the gripper's jaw at most 0.0035 N m: `pan_roll_gripper.py`).
- **Back-EMF**: 0.57 N m s/rad at the output; cancelling it needs 265 duty units per rad/s at 8.9 V.
- **Reflected rotor inertia**: 0.005-0.028 kg m^2 at the output (rotor x 345^2). The roll's own link is 4e-5 kg m^2
  and the gripper's jaw 1.6e-5: on those two joints **almost everything the hand feels is the motor and the gearbox**.
- **Friction**: not published, not yet measured (bring-up S2 and the calibration measure it). The plant sweeps it.
- **[New] Supply**: fault records read 8.9 V (`voltage_dv` 89, 2026-09-28) and, in other records, 4.0 V and 2.0 V
  (brown-outs). The servos are the 12 V model, so 8.9 V is already low.

### 2.2 How weightless can the comp joints feel

Compensation removes the gravity bias only; gear friction (part of it proportional to the carried torque), the
energised motor's back-EMF drag and the reflected inertia remain. Simulated push to move a joint at a steady speed
(force at the hand; `out_guide_final_table.txt`, groups `feel` and `feel_joints`; "low" friction = 0.05 N m + 20 % of
the carried torque, "mid" = 0.15 N m + 50 %; c = the lead; FF = the friction feed-forward at 0.5 of the calibrated
friction):

| Joint, speed | low, c=0 | low, c=0.6 | low, c=0.6 + FF | mid, c=0 | mid, c=0.6 | mid, c=0.6 + FF |
|---|---|---|---|---|---|---|
| lift 15 deg/s | 0.88 N | 0.57 N | **0.36 N** | 2.13 N | 1.74 N | **0.98 N** |
| lift 30 deg/s | 1.49 N | 0.93 N | 0.89 N | 3.21 N | 2.40 N | 2.24 N |
| elbow 30 deg/s | 1.94 N | 1.07 N | 1.00 N | 3.90 N | 2.71 N | 2.40 N |
| wrist 45 deg/s (forearm held) | 3.43 N | 1.92 N | 1.74 N | 5.63 N | 3.42 N | 2.77 N |
| pushing against an ordinary Hold | lift 6.4 N, elbow 9.8 N | | | lift 9.4 N, elbow 15.1 N | | |

The feed-forward helps most at calm speeds (lift at 15 deg/s: 37-44 % less force) and fades out toward the brake
speed (3.3). Host rate and the servo's P barely matter (lift 15 deg/s, mid, c=0.6 + FF: 100 Hz 0.98 N, 30 Hz 1.02 N,
P16 0.98 N). "High" friction (0.35 N m + 90 %) is self-locking: a 4 N m hand cannot move it at all.

### 2.3 [New] Why the free joints can only get a little help

Plain Coast is a floating motor: the hand meets gear friction and the reflected inertia, nothing that grows with
speed. To push on a joint the servo must be energised, and an energised motor brakes at 0.57 N m s/rad. The lead
cancels a share c of that (plus the gear's viscous part: k_v = 274 duty units per rad/s at 8.9 V), and the friction
feed-forward pushes with FF = k_f x the calibrated kinetic friction. At a steady speed v, compared with Coast:

    energised - Coast  =  (1 - c) k_v v + k_loop v - FF        (duty units; k_loop = the goal scheme's own drag)

so energised beats Coast only below the crossover v_x = FF / ((1 - c) k_v + k_loop). Measured on the plant with the roll
forced energised at every speed (`free_drag`; mid friction, force at the roll's 3 cm):

| Roll speed | Coast | energised, c=0 | c=0.9, no FF | c=0.9, FF 0.5 | c=0.9, FF 0.7 |
|---|---|---|---|---|---|
| 10 deg/s | 3.03 N | 5.83 N | 3.60 N | **2.13 N** | **1.57 N** |
| 20 deg/s | 3.19 N | 9.79 N | 4.09 N | **2.73 N** | **2.17 N** |
| 30 deg/s | 3.34 N | 13.6 N | 4.75 N | 3.35 N | **2.78 N** |
| 60 deg/s | 3.81 N | 22.7 N | 6.63 N | 5.26 N | 4.70 N |

The crossover with c = 0.9 and FF 0.5 is 30 deg/s, as the formula gives with k_loop = 1.5 units per rad/s. c cannot go
to 1: at c >= 1 the lead pushes harder the faster the joint goes (3.4, "why a released joint stops").

Two more things the plant showed: (1) the lead uses a speed estimate that lags about 25-30 ms, so while the hand speeds
a joint up, an energised motor drags more than a floating one: before the steadiness gate (3.4), a 30 deg flick of
the roll in 1 s took 9.0 N instead of 5.8 N in Coast, and the gripper 5.4 N instead of 3.5 N; (2) inside the
0.87 deg backlash the roll's output moves freely while the motor stands still, which looks like a gentle start, so the
assist waits until the joint has moved past the backlash.

---

## 3. Control scheme

### 3.1 The decision (unchanged)

**Position-mode effort control** ("virtual displacement") from a dedicated 100 Hz host loop. **No operating-mode
switch at any point.** Each cycle the host computes an effort u (duty units) per energised joint and writes, in one
sync write, Goal_Position = present + lead + steps(u) and Torque_Limit = |u| + margin (within a per-joint floor and
ceiling). The servo's own loop applies u; a host that stops leaves every energised servo holding its last goal at
that cap. The alternatives (PWM mode, low Torque_Limit, current loop, community firmware) and why they lost are in
GravityDesign.md 3.1 and 3.9; nothing in the decisions changes that comparison.

### 3.2 Joint roles **[Changed: D3, D4]**

| Joint | Role while guiding | Why |
|---|---|---|
| lift, elbow, wrist | **comp**: always energised; u = gravity + lead + friction feed-forward (+ brakes, walls) | they carry 0.86 / 0.45 / 0.12 N m of gravity at worst |
| pan, roll, gripper | **free**: Torque_Enable 0 (floating, exactly Coast) at rest, on quick moves and above the assist band; **energised with an assist** (lead + friction feed-forward) only while the hand moves them slowly and steadily inside the band; **caught** (energised where they are, limit 400) at a wall or a slider limit | no gravity load; Coast is the lightest a servo can be except where the feed-forward outweighs the energised drag (2.3) |
| gripper while squeezing an object (GraspHold) | **keep**: left out of the free joints, keeps its squeeze | a free gripper would drop the object (open question 10.3) |

Lock stays 1 throughout; the free joints are released with Torque_Enable 0 only, never `disable_torque` (which writes
Lock 0, the restart signature).

### 3.3 The law for lift, elbow and wrist (per joint j, each 10 ms cycle) **[Changed: D4]**

```
q      = Present_Position (steps, the multi-turn frame position mode reports)
theta  = URDF angles of lift, elbow, wrist from q:  wrap(q - ReferenceRaw) * 2 pi / 4096
v      = least-squares slope of the last 40 ms of positions (circularly unwrapped), rad/s
V      = Present_Voltage
u_g    = alpha_j * G_j(theta) * (V_cal / V)               gravity: the calibrated model, duty units at V_cal
u_v    = c_j * kv_j * v * (V_cal / V)                     lead, only while |v| <= 0.7 v_cap
u_f    = FF_j(theta) * s_j(v, t)                          friction feed-forward, only while |v| <= 0.7 v_cap   [Changed: D4]
         FF_j = k_f * (f_kin0 * V_cal / V + f_kin_load * |G_j(theta) at alpha 1|)
         s_j  = sign(v) * ramp(|v|; 2 -> 6 deg/s) * (1 - ramp(|v|; 0.35 v_cap -> 0.7 v_cap)) * stroke(t)
u_b    = -600 * (|v| - 0.7 v_cap) * sign(v)               brake, only above 0.7 v_cap (lead and feed-forward off)
u_z    = -900 * depth * v                                 zone: last 6 deg before a soft limit, moving outward
u      = u_g + u_v + u_f + u_b + u_z
goal   = q + 0.8 * v * (L + T/2) / step + steps(u)        prediction cancels most of the sample-and-hold lag
steps(u) = sign(u) * ((|u| - punch) / K + deadzone)       K = P/4 (reg 21), punch = reg 24, dead zone = regs 26/27
limit  = clamp(|u| + 150, floor_j, ceiling_j)             lift 150..650, elbow 120..450, wrist 100..250
```

- **G_j, alpha, c, kv, V_cal / V, speed caps, caps**: unchanged from GravityDesign.md 3.3 (alpha 0.95; c 0.6 default,
  0..0.8; kv identified; caps 65 / 45 / 25 % of full duty, walls 750, under the 800 overload trip; back-driven at the
  ceiling and speed cap 1.72 / 1.36 / 1.24 A, under the 2.0 A overcurrent).
- **The friction feed-forward is in the first build** (it was "a later stage"): k_f 0.5 by default, 0..0.7 allowed.
  f_kin0 (the unloaded kinetic friction) and f_kin_load (the share of the carried torque) are the calibration's
  half-band fit (5). It is zero in a deadband (under 2 deg/s, full from 6 deg/s), so a joint at rest gets none; it
  fades out between 35 % and 70 % of the speed cap (lift: full to 16 deg/s, gone at 31.5).
- **stroke(t)**: one continuous stroke in one direction gets the feed-forward for 2 s, which then fades out over 0.5 s;
  it re-arms when the joint comes to rest or reverses. This is the "decay": a joint that kept moving on its own
  (friction lower than the start's probe showed) loses the push within 2.5 s.
- **The start's probe** (3.7) switches a joint's feed-forward off for the session if 1.4 x it moves the joint.

### 3.4 [New] The assist for pan, roll and gripper

Each free joint is a small state machine, run every cycle after the comp joints:

| State | Torque | Enters when | Leaves when |
|---|---|---|---|
| **float** | off (Coast) | the start ends; a stop; above the band; a hard speed-up or slow-down; a stroke used up | see assist and caught |
| **assist** | on, u = lead + feed-forward | it has moved more than 12 steps (1.05 deg, past the 0.87 deg backlash) from where it last rested, its speed has stayed inside the band (v_on <= \|v\| < 0.85 v_top) for 0.15 s, \|accel\| <= 60 deg/s^2, and its stroke is not used up (two cycles in a row) | \|v\| > v_top, or \|accel\| > 150 deg/s^2: float; \|v\| < 2 deg/s for 0.12 s: float (stopped); the stroke is 2 s old: float until it has stopped |
| **caught** | on, goal = the last clear pose (or the limit), Torque_Limit 400 | in the collision guard band, or at a slider limit moving outward | clear again: float |

In the last 6 deg before a slider limit, moving outward, the joint is energised with no lead and no feed-forward and a
zone brake u = -900 x depth x v (the energised motor's own back-EMF adds to it): the limit is felt before it is hit.

**The assist effort (energised in the band):**

```
u      = c * kv * v * (V_cal / V)  +  FF * s(v, t)
FF     = k_f * f_kin * (V_cal / V)                                     f_kin: the calibrated kinetic friction
s(v,t) = sign(v) * ramp(|v|; 2 -> 6 deg/s) * (1 - ramp(|v|; 0.8 v_top -> v_top)) * stroke(t)
goal   = q + 0.8 * v * (L + T/2) / step + steps(u)
limit  = clamp(|u| + 60, 60, ceiling)                                  ceiling: pan 350, roll 400, gripper 400
```

**The band's top** is computed from the calibration, so the assist is switched on only where it beats Coast:

```
v_top = min(v_hi, 0.8 * FF / ((1 - c) * kv * (V_cal / V) + k_loop))   if torque off floats (expected)
v_top = v_hi                                                           if torque off brakes (then energised always wins)
v_top = 0 (no assist: the joint just floats) if it would be under 3 * v_on = 12 deg/s
```

0.8 is the margin under the crossover of 2.3. With the defaults (c 0.9, k_f 0.5) the plant's three friction levels
give: low friction no band (just Coast), mid 23 deg/s, high 43 deg/s; with k_f 0.7: 16 / 32 / 61 deg/s
(`out_design_numbers.txt` section 9). If bring-up finds that torque off brakes, `coast_brakes` is set in the
calibration and the band runs to v_hi with no steadiness gate (a braked Coast is never lighter).

**Parameters**

| Parameter | Comp joints (lift / elbow / wrist) | Free joints (pan / roll / gripper) | Bounds the bridge refuses outside of |
|---|---|---|---|
| lead c | 0.6 | 0.9 | comp 0..0.8; free 0..0.9 |
| kv (duty per rad/s at V_cal) | calibrated | calibrated | 0.75..1.10 x the motor's back-EMF prior (265 at 8.9 V) |
| friction feed-forward k_f | 0.5 x calibrated kinetic friction (pose-dependent) | 0.5 x calibrated kinetic friction | 0..0.7 |
| deadband | 2 deg/s, full at 6 deg/s | same | >= 1 deg/s |
| feed-forward fades out | 35 %..70 % of the speed cap | 80 %..100 % of the band's top | |
| stroke | 2 s, then fades over 0.5 s | 2 s, then floats until it has stopped | |
| switch-on | always on | moved 12 steps, in band 0.15 s, \|a\| <= 60 deg/s^2; v_on 4 deg/s | v_on > v_off >= deadband |
| switch-off | | \|v\| > v_top, \|a\| > 150 deg/s^2, \|v\| < 2 deg/s for 0.12 s, stroke over | |
| v_hi (top of any band) | (speed caps 45 / 50 / 80 deg/s) | 60 / 90 / 90 deg/s | v_on..120 deg/s |
| k_loop | | 2 duty units per rad/s (plant: 1.5) | |
| Torque_Limit | \|u\| + 150 in 150..650 / 120..450 / 100..250; walls 750 | \|u\| + 60 in 60..350 / 400 / 400; caught 400 | free ceilings <= 500 |
| probe at the start | +-1.4 x feed-forward | none (see below) | |

The free joints' ceilings, back-driven at v_hi, draw 1.26 / 1.63 / 1.63 A (overcurrent 2.0 A for 2 s), and these
duties are momentary: the lead matches the back-EMF, so the current while assisting is about the feed-forward's.

**Why a released joint stops (and stays where it is left).** Write the torque on a released, energised joint in duty
units (the hand gone): FF s(v) - F_k sign(v) - (kv_true - c kv) v, where F_k is the real kinetic friction and kv_true
the real back-EMF + viscous slope.

1. **The lead never pushes it on.** With c <= 0.9 and kv <= 1.10 x the motor prior (refused otherwise), c kv is at
   most 0.99 of the real slope, so the last term always opposes motion. It stays passive whatever the 25-30 ms delay:
   the energised joint is J s + b - c b e^(-s tau), whose real part at any frequency is b (1 - c cos(w tau)) >= b (1 - c)
   > 0, so it cannot store and return energy as an oscillation.
2. **The feed-forward is under the friction it cancels.** FF <= 0.7 of the calibrated kinetic friction, so while the
   real friction is at least 70 % of calibrated (warm grease drops it by perhaps 20-30 %) the joint decelerates. The
   deadband takes the push away below 2-6 deg/s; at rest it is exactly zero, and then the joint floats (torque off) after
   0.12 s, so nothing can start it moving.
3. **If the friction is far lower than calibrated** (the feed-forward larger than the real friction, a calibration or
   wear fault): a free joint cannot run past the band's top (it floats there, and from float it is only re-energised
   after 0.15 s of steady motion, which friction-only deceleration of more than 60 deg/s^2 never is), and a stroke ends
   after 2 s, after which it floats until it has stopped. A comp joint is caught by the start's probe; if that were off,
   the 2 s stroke fade, the speed brake and the fall and speed exits bound it.
4. **It stays where it is left**: pan, roll and gripper float with almost no load (roll at most 0.006 N m, the
   gripper's jaw 0.0035 N m, both far under their friction; `pan_roll_gripper.py`); the comp joints hover on the gravity
   model at alpha 0.95, with the joint inside its static friction band. One exception: on a cart tilted 2 deg, the arm
   stretched out with 100 g puts up to 0.050 N m on the pan, about the plant's lowest pan friction, so a floating pan
   could drift: the wheels-locked, level-floor instruction stands (safety case 26).

Simulated (7.2): released from inside the band every free joint stops within 0.0-0.07 s after gliding 0.01-0.67 deg;
from 80 deg/s it glides as in Coast (2.5-8 deg); no creep at all in the 3 s after any release (0.000 deg); with the
feed-forward at 1.25 x the real friction it glides 0.17-0.97 deg; at 2.8 x it runs on 6.8-8.4 deg for 0.5 s, then the
stroke ends and it stops.

**Why the free joints are not probed at the start.** A short push of 1.4 x the feed-forward on a free joint is
invisible: inside the 0.87 deg backlash the output (where the encoder is) does not move, and the energised motor's own
back-EMF holds the motor side to about 2 deg/s, so in 0.15 s it does not even cross the gap (plant, roll at 40 % of its
calibrated friction: 0 steps of output motion). A probe long enough to see it would add over 1 s to every start. Point 3
above bounds what a too-large feed-forward can do instead.

### 3.5 The guide loop **[Changed: D4]**

As GravityDesign.md 3.4, with up to six joints written:

- A thread of its own at 100 Hz on deadline scheduling, raised priority, taking the motor lock for each cycle.
- Per cycle: **SYNC_READ regs 56..70** of all six (1.58 ms on the wire); **SYNC_WRITE regs 42..49** of the energised
  joints (3 comp joints 0.35 ms; all six 0.62 ms); on a cycle where a free joint stops being assisted, **SYNC_WRITE reg
  40 = 0** for those joints (0.14 ms for three); every 4th cycle **SYNC_READ regs 40..55** (1.64 ms). Worst cycle on the
  wire 3.98 ms; mean cycle including adapter and OS overhead 3.7-4.8 ms, 37-48 % of the period.
- The lean transport (one packet, staggered replies against a 5 ms deadline, no `clearPort`, `BusStats` counting) and
  the encoders: goal and position sign bit 15, **load bit 10**.
- The 30 Hz ControlFrame while guiding does no bus traffic; it supervises (heartbeat, collision guard band, temperature
  logic, motion trace, status text).
- The torque read-back every 4th cycle is compared with what the law last commanded per joint (on for comp and
  assisted or caught free joints, off for floating ones): a mismatch means a restart or a stray write: coast.

### 3.6 Timing rules **[Changed: D1]**

A cycle that starts more than 150 ms late ends guiding **limp** (it held a preloaded hold before); during the start it
returns to the entry hold. 50-150 ms is logged. A joint silent for 5 cycles (50 ms): coast. One to four missed replies:
that joint keeps its last goal (no false exit from 1 % random drops or 3-reply bursts, simulated). **[Review 2026-10-10]**
A position reading that implies more than 3 x the joint's speed cap (a free joint's: 3 x 270 deg/s) is a missed reply
too, and two in a row coast; firmware status bits, the torque and Lock read-back and the supply under 7.0 V coast only
once read twice in a row (the status registers are read again on the next cycle), heat once it has lasted 250 ms.

### 3.7 Entering **[Changed: D2, D4, New: supply]**

Refused (with the reason) unless all of these hold, checked under the motor lock:

1. the message has `explicit: true` (a click, never a task or the AI) and came from **this PC or another device with
   the remote password** (`Access` "local" or "password"; RemoteAccess already admits nothing else) **[Changed: D2]**;
2. the arm is connected, torque on (Hold), every joint in position mode, no MotionFault or ProtectionTrip, the USB link
   up, no route, stream, calibration or planning in progress;
3. every joint still (at most 2 steps over 0.5 s) and inside its slider arc;
4. every servo below 50 C;
5. **[New] the supply**: every servo's Present_Voltage at least **8.5 V**, the servos within 0.5 V of each other (a
   connector dropping voltage), and within 15 % of V_cal. Refusal text: "The servo supply reads 8.2 V (at least 8.5 V
   needed): check the 12 V supply and its cable". 8.5 V is about 0.4 V under the 8.9 V the servos have reported; at 8.5 V
   the lift's worst model effort plus 20 % is 502 units, under its 650 ceiling. Bring-up S0 may raise it (to the
   measured resting rail minus 1 V);
6. a calibration whose fingerprint matches (5), with every assist setting inside the bounds of 3.4 (`validate()`);
7. the calibrated model agrees with the URDF prior at the present pose;
8. no control stall in the last 10 s;
9. the gripper is not squeezing an object, or the user confirmed "the gripper keeps its grip" (it then stays out of
   the free joints) **[New]**.

Then: the register ledger, Lock 1, hold corrections cleared, Holding false, `MotionEpoch += 1`, preempt generation
bumped, the guide thread started, and the starting page recorded as the **owner** (3.8, page gone).

**The start (hands off, 2.2 s)** **[Changed: D4]**. PRELOAD 0.6 s and RAMP 1.2 s as before (the comp joints' effort
moves to the model, the anchor spring to the entry pose fades out; more than 24 steps or 8 deg/s of motion aborts back
to the entry hold). Then **PROBE 0.4 s**: each comp joint's effort ramps to the model's +1.4 x its friction
feed-forward over 0.15 s, to -1.4 x over the next 0.15 s, and back over 0.1 s. Gravity keeps these gears pressed on one
side, so a feed-forward larger than the friction left over moves the joint at once: one that moves more than 3 steps
loses its feed-forward for the session (its push stops there; the start goes on; the strip says "Shoulder: no friction
help (it moved under the test)"). The probe never aborts the start by speed; the 24-step check still applies. At the
end pan, roll and the gripper float. The page says "Let go: hand guiding starts in 2 s".

### 3.8 Leaving **[Changed: D1]**

| Exit | When | What is written |
|---|---|---|
| **preloaded hold** (an ending, not a fault) | Hold here, Stop and hold, Hold, switching to AI mode; **the owner page gone for 3 s** | one SYNC_WRITE regs 41..49 for every joint: Acceleration 51, Goal_Position = q + steps(model effort at alpha 1) for the comp joints and q for pan, roll, gripper, Goal_Time 0, Goal_Speed 0, Torque_Limit 1000; read back; the bridge's normal Holding |
| **back to the entry hold** | a failed start (the anchor check, the model check, a stall or a stop during the start) | the entry hold goals, Torque_Limit 1000 (the arm was never handed over) |
| **refused** | entry checks (3.7) | nothing |
| **coast (limp)** **[Changed: D1]** | **every fault while guiding**: a joint silent for 5 cycles; a restart (Lock reads 0, the position frame jumps a turn, torque not as commanded); the USB link dropping (latched at once; the coast goes out the moment it is back); firmware protection bits 2/3/5; 55 C; supply under 7.0 V; a cycle more than 150 ms late; the model disagreeing with the URDF prior; a fall; over the speed cap; Present_Load not what the goal asks for; the collision check stale; E-stop; Coast | the existing `LatchMotionFault` / `AutoCoast` path: Torque_Enable 0 on every joint, read back (`VerifyProtectionCoast`), retried every frame for a joint that does not answer, a latched fault with the reason |

What changed and why it matters:
- **Old design: hold; now: coast.** The old design held (preloaded or plain) on all of these except firmware
  protection, E-stop and Coast, and proposed hold_fault for dropouts. All of them now coast, per D1.
- **A silent servo cannot be made limp** while it is silent: the torque-off is retried until it answers. Simulated
  (lift silent at the mid pose): the lift keeps its last goal while the elbow and wrist coast (elbow 49 deg in 0.5 s),
  then the lift coasts when it answers.
- **During a USB outage nothing can be written**: every energised servo holds its last goal at its cap (hands off the
  arm stays where it is: 0.0 deg in the plant), floating joints stay floating, an assisted free joint holds a goal a few
  degrees ahead at its small cap. The moment the link returns, everything coasts (the existing auto-coast-v1 rule for a
  move in progress, which hand guiding counts as). If it never returns, the arm keeps holding.
- **The owner page gone** is kept as a hold, not a coast: nothing is wrong with the arm, and coasting would drop it
  every time a phone screen sleeps. Any device can press Hold here or E-stop; only the owner keeps it alive.
- **False trips now cost a drop**, so two detectors were made harder to trip by a hand: a fall needs 3 cycles in a row
  (a gear springing back when a hand lets go of a self-locking joint looked like a fall for one or two), and the instant
  over-speed exit (1.15 x the cap) needs 2.

Simulated, hands off (`exit`, `faults_coast`, `stall`, `usb`, `brownout`, low and mid friction): a coast drops the
shoulder 20-31 deg in 0.25 s and 78-107 deg in 0.5 s from the stretched-out and mid poses (the plant has no desk; the
real arm lands on whatever is below it). High friction: no drop (self-locking). **With a light hand under the forearm:
4.4-5.6 deg.** Hold here: 0.00 deg in every case.

### 3.9 Walls **[Changed: D3, D4]**

Unchanged for the comp joints (zone resistance, one-way spring at the limit, the ratchet; the basket, cart body,
camera head and rearward limit by a 30 Hz padded collision check of the measured pose, held with the URDF prior's
effort at 750). Simulated: the elbow pushed with about 1 N m goes 2.0-3.4 deg past its limit and settles within
0.4-1.3 deg of it; a 3.6-5.3 N push at the gripper goes 2.3-5.5 cm into the guard band and the arm comes back to within
1.4 cm of its edge.

Free joints: in the last 6 deg moving outward they are energised with the zone brake (3.4); at the limit, or anywhere
in the guard band, they are caught (energised where they are, or at the limit, with Torque_Limit 400) and floated
again when clear. Simulated: the gripper closed into its slider limit and the roll turned into its own by a light hand
go at most 0.42 deg past and settle within 0.13 deg.

### 3.10 Friction feed-forward **[Changed: D4]**

GravityDesign.md 3.8 kept it out of the first build; D4 puts it in, on all six joints, with the safety conditions that
section asked for, made concrete:

| GravityDesign.md asked for | Now |
|---|---|
| no more than half of the identified minimum kinetic friction | k_f 0.5 by default, 0.7 at most (refused above), of the calibrated kinetic friction (comp joints: including its load-dependent part) |
| faded in with speed | deadband 2 deg/s, full at 6 deg/s |
| off above 0.7 of the speed cap | comp: fades out 35-70 % of the cap; free: fades out over the top 20 % of the band, and the joint floats above it |
| repeatable calibration (two runs within 20 %) | kept (5) |
| a per-joint setting | k_f per joint in the calibration file |
| the bring-up's release test | S9 |
| (new) | the 2 s stroke; the start's probe on the comp joints; the free joints' float above the band |

Simulated on the lift (20 deg/s, mid friction, `friction_ff`): k_f 0.5 takes 1.17 N and glides 0.42 deg after release;
k_f 0.7: 0.96 N, 0.70 deg; real friction 30 % under calibrated: k_f 0.5 glides 1.09 deg, k_f 0.7 is switched off by the
probe; real friction 60 % under calibrated: the probe switches the lift's and elbow's feed-forward off (0.7 deg glide);
with the probe disabled the lift runs on 22 deg in 1.2 s before the stroke ends it, which is why the probe is not
optional.

### 3.11 The PWM backend (not chosen)

Unchanged (GravityDesign.md 3.9). It would not help the free joints either: in PWM mode the drive is the same
slow-decay bridge, with the same back-EMF brake.

---

## 4. Safety case **[Changed: D1-D4]**

| # | Hazard | Detection | Response | Evidence (`out_guide_final_table.txt` group) |
|---|---|---|---|---|
| 1 | Wrong model sign or size at runtime | The calibrated model against the independently computed URDF prior every cycle (ratio 0.5-1.8 above 40 units, within 60 below) | Refuse to start; while guiding **coast** **[D1]** | `start_sign`: 12/12 refused before any write; `runtime_sign`: caught on the first cycle; then the drop of 3.8 |
| 2 | Plausible but wrong model, or a hand on the arm while it starts | The anchored start: more than 24 steps or 8 deg/s | Back to the entry hold | `start_gain`: 0.0 and 2.0 refused; 0.5 and 1.5 abort within 25 steps at low friction, held by friction (and feed-forward switched off) at mid |
| 3 | Effort pushed the wrong way by the servo | Impossible in position mode | | design |
| 4 | Joint falling | More than 10 rad/s^2 toward gravity above 12 deg/s while the model carries over 60 units, **3 cycles in a row** **[New]** | **Coast** **[D1]** | `runtime_sign` with the prior check off: caught after 2-3 deg of motion; no false trip when a hand lets go of a self-locking joint (`feel`, high friction) |
| 5 | Over speed | Speed estimate per joint | Brake above 0.7 of the cap; over the cap for 60 ms or 1.15 x for 2 cycles: **coast** **[D1]** | `speed`: a 90 deg/s yank coasts; 40 deg/s under the 45 cap runs |
| 6 | Effort too large | Torque_Limit every cycle (comp 150..650 / 120..450 / 100..250, walls 750; free 60..350 / 400 / 400, caught 400) | Hardware clamp in the servo | `design_numbers.py` 1, 6, 9 |
| 7 | Slider limits | Joint angle against its arc | Zone resistance, one-way spring (comp); zone brake, caught (free) | `limits`, `free_walls` |
| 8 | Basket, body, camera head, rearward limit | 30 Hz padded collision check, armed after a clear pose | Hold at the last clear pose; free joints caught; resume when clear | `band` |
| 9 | Host stall (1.1-1.7 s, about daily) | A cycle more than 150 ms late | The servos hold their last goals meanwhile; on resume **coast** **[D1]** | `stall`: 0.0 deg during, then the drop of 3.8 |
| 10 | Guide thread crash | The supervisor: thread gone or heartbeat over 60 ms | **Coast** from the 30 Hz frame **[D1]** | design (test plan 7.3) |
| 11 | A servo stops answering | 5 consecutive missed replies | **Coast**; the silent one keeps its last goal until it answers **[D1]** | `bus`: 1 % drops and 3-reply bursts run on; 8 coast; `faults_coast` |
| 12 | USB link lost | The existing link supervisor | During: last goals held; back: **coast at once** (the existing rule for a move) **[D1]** | `usb` |
| 13 | Servo brown-out or restart | Lock 0, a frame jump of a turn, or torque not as commanded | **Coast** **[D1]** | `brownout`: caught in 37 ms |
| 14 | Firmware unload | Status byte of every reply | Coast (as before) | existing |
| 15 | Overheating | Temperature in every reply | Entry refused at 50 C; warning at 50 C ("goes limp at 55 C: support the arm or press Hold here"); **coast** at 55 C **[D1]**; the existing 60 C trip unchanged | `faults_coast` (55 C injected) |
| 16 | Supply sag or brown-out | Present_Voltage every cycle | **Entry refused under 8.5 V or a 0.5 V spread** **[New]**; effort scaled by V_cal / V; under 7.0 V **coast** **[D1]** | `supply`: 8.2 V refused; sag to 7.6 V: 0.0 deg with scaling (28.5 deg without); 6.8 V: coast |
| 17 | Servo settings not as read (P) | Present_Load against the duty the last goal asks for | **Coast** **[D1]** | `p_mismatch`: start abort (low) or coast after 0.3 s (mid) |
| 18 | Exit drops the arm | (design) | Hold here is the preloaded exit, atomic per servo, read back | `exit`: 0.00 deg |
| 19 | Emergency stop | Unchanged | Torque off on every joint: **the arm drops**; while guiding the button reads "Emergency stop: the arm drops" | existing tests |
| 20 | Lowered Torque_Limit left behind | (design) | Every holding exit writes 1000; `enable_torque` and `configure_motors` write it first; RAM only | `feetech_simple.py:555-601` |
| 21 | Entry by the AI, a task, a route; **from another device** **[D2]** | `explicit: true` + `Access` local or password at the bridge; Manual mode and no AI task at the page; no route or plan in progress | Refused otherwise; a device without the password never reaches the bridge (RemoteAccess) | design |
| 22 | Moves while guiding | `MotionPreflight`, `PrepareCalibration`, `CaptureArmPose`, `CaptureCadReference` **[D5]**, the page's `CommandBlockReason` | Refused: "Hand guiding is on: press Hold here first" | probe P6 |
| 23 | Nobody supervising (the owner page gone) | The owner's connection gone for 3 s **[D2]** | Preloaded hold (an ending, not a fault) | design |
| 24 | Multi-turn frame | Goals in the frame the reading came from; a jump of a turn is a restart | Coast | `brownout` |
| 25 | Pinching or striking a person | Speed caps, walls, capped effort | Plain-words instructions; for the person holding the arm | design |
| 26 | Cart moving | Not sensed | Wheels locked; the speed caps and fall still apply | open |
| 27 | **[New] The assist drives a joint by itself** | (design) the bounds of 3.4 (`validate()`), the start's probe (comp), the band and its steadiness gate (free), the 2 s stroke, the deadband, float at rest | Out-of-bounds settings refused; too-large feed-forward switched off (comp) or let go within 2 s (free) | `assist_params`, `free_release`, `release`, `friction_ff` |
| 28 | **[New] A fault coasts while the person is not holding the arm** | (consequence of D1) | The UI says so; the 50 C warning says it will go limp; the first-use confirmation says so | drop numbers 3.8 |
| 29 | **[New] A free gripper drops a held object** | GraspHold at entry | The gripper keeps its squeeze unless the user frees it (3.7, 9) | design |

Defence in depth on the model is unchanged: the URDF prior check is the primary runtime detector of a model error and
is not configurable. What D1 changes is the end state: the old design ended a model error in a stiff hold (0.2-4 deg of
sag); now the arm goes limp, which is safe for a person holding it and drops it if nobody is.

---

## 5. Identification (calibration) routine **[Changed: D4]**

As GravityDesign.md 5 for the comp joints (wrist, elbow, lift; 2-3 configurations each; position-mode goal ramps at
10 and 30 deg/s, the lift 8 and 20 (5 and 20 for every joint until 2026-10-11, about 6 minutes); band centre fitted by MAP regression to the closed form with the URDF as prior; half-band = friction
F(q) = F0 + f |G(q)|; growth with speed gives kv; hover test sets alpha 0.90-1.00). Added:

6. **Free joints**: the same up/down ramps on pan (+-30 deg about its pose), roll (+-90 deg) and gripper (open and
   close, empty), at 10 and 30 deg/s, each move a checked route. Fit per joint: half-band at the slow speed minus kv x
   speed = the kinetic friction f_kin; growth from the slow to the fast speed = kv. Plant: f_kin comes out as the gear's friction plus the
   output bearing's (roll mid: 31.1 units against 29.6 for the gear alone), kv 240-245 against 274 (an underestimate,
   which errs safe: the lead then cancels less).
7. **Spin-down** of roll and gripper: driven at 45 and then 90 deg/s, then Torque_Enable 0, positions read at 100 Hz for
   0.4 s. Coulomb friction alone (a floating driver) makes the run-on grow with v^2, the back-EMF brake with v: the
   driver floats if the ratio of the two run-ons is at least 2.2 and the 90 deg/s run-on at least 3 deg. Plant: 12/12
   classified correctly over three friction levels (floating 2.7-3.6 ratio and 4.8-10.7 deg; braking 1.4-1.5 and
   1.7-1.9 deg). It sets `coast_brakes` for all six (one driver design), and must agree with bring-up S2.

**Accepted only if** (added to the old list): every free joint's f_kin positive and two runs within 20 %; every kv
within 0.75-1.10 of the motor prior and two runs within 10 %; the spin-down verdict the same on roll and gripper.

**Saved** to `back end/hand_guide_calibration.json`: as before, plus per free joint f_kin, kv, and the assist settings
(c, k_f, v_hi), `coast_brakes`, and per comp joint k_f. Changing a setting through the page writes it here and is
validated (3.4) before it is used.

---

## 6. Integration into the bridge and Clancy **[Changed: D1, D2, D4, D5]**

**Bridge** (`back end/bridge_server.py` at 2adf0b6), as GravityDesign.md 6 with these changes:

- New `back end/HandGuide.py` (the law of `HandGuideSim/hand_guide_law.py`, the URDF prior, the calibration fit,
  `validate()`) and `back end/GuideBus.py` (the lean sync transport and encoders; Present_Load bit 10).
- `hand_guide {enable, explicit}` in `handle_command` (4798) with `HandGuideRefusal` next to `ConnectRefusal` (4783):
  **accepts `Access` "local" or "password"** **[D2]**; records the owner connection.
- **Fault exits go through `LatchMotionFault` / `AutoCoast`** (988, 1065) **[D1]**: Torque_Enable 0 on every joint,
  `VerifyProtectionCoast`, the latched fault with the law's reason. A USB drop while guiding latches at once because
  `MotionInProgress()` is true while guiding (`ServiceSerialLink` 1232, `BeginSerialOutage`). No hold_fault, no plain hold.
- The free joints' torque state per cycle: per-joint torque expectations (`ExpectedTorque(Name)`) in
  `_check_servo_faults_locked` (3323) and `ReadEveryJoint` (1347) follow the law's `torque_cmd`.
- **Entry checks the supply** (3.7.5) and the gripper's `GraspHold` (449, 1510) **[New]**.
- Refusals while guiding: `MotionPreflight` (1561), `PrepareCalibration` (1513), `CaptureArmPose` (4587) and
  **`CaptureCadReference` (4731)** **[D5: no camera capture while guiding]**.
- `state_snapshot` (4635) and `/health`: `hand_guide` {on, phase, roles, free_states, assist (per joint: on/off, band
  top), owner (this PC / another device), since_s, last_exit, calibrated, coast_brakes} and `hand_guide_version`
  "guide-v1". `BridgeVersion.json` bumped; `SafetyVersion` stays hold-v4; `ProtectionVersion` stays auto-coast-v1 (the
  coast is its existing rule, now also used for guiding faults).
- `feetech_simple.py`: **Present_Load decoded with bit 10** (2.1), a Lock-preserving per-joint torque-off.

**Clancy** (`AI Controlled WebApp/src`):

- Motion panel: the **Hand guide** button in Manual mode on **any device that has the bridge's password** **[D2]**
  (not only `ThisPc`); disabled with the reason unless connected, holding, not moving, calibrated, the supply OK.
  While guiding it reads **Hold here**; Send and Home are disabled.
- Status strip: "Hand guiding: move the arm by hand. Hold here to stop. **If something goes wrong it goes limp: keep a
  hand on it.**" with the start's countdown, which joints have friction help, and "guided from: this PC / phone".
- The Emergency stop button's label while guiding: **"Emergency stop: the arm drops"**.
- First use in a session (per device): a confirmation that says what it does, the hands-off start, that faults and
  E-stop make the arm go limp, and to keep the screen on (a page gone for 3 s ends guiding, holding).
- `CommandBlockReason` (`lib/BridgeCommandGate.ts`) blocks `send_ik`, `move_servo`, `go_home` and captures while
  guiding; AI requests refused; switching to AI sends Hold here first.
- **Calibrate hand guiding** adds the free joints' sweeps and the spin-down.

---

## 7. Offline simulation (`back end/HandGuideSim/`)

| File | What it is |
|---|---|
| `hand_guide_law.py` | The reference law and supervisor (pure logic, no I/O): comp law, free-joint state machine, start probe, `validate()`, exits. The bridge port copies it. |
| `arm_plant_sim.py` | The plant: arm dynamics, geared two-mass joints with backlash and stick-slip friction for all six, motor (energised = slow-decay with back-EMF; torque off floats, or brakes with `coast_brakes`), servo firmware, bus and host timing, faults. |
| `guide_scenarios.py` | 323 scenarios in 25 groups; `python guide_scenarios.py` (24 worker processes, about 35 s), or name groups. |
| `summarize_scenarios.py` | The table: `out_guide_final_table.txt` (raw JSON is regenerated, not kept). |
| `gap_and_ident_sim.py` | Torque gap (for the PWM option), identification of the comp joints, **the free joints' identification and spin-down**: `out_gap_ident.txt`. |
| `design_numbers.py` | Every other number (caps, currents, bus budget, band tops, supply): `out_design_numbers.txt`. |
| `gravity_model.py`, `gravity_comp_model.py`, `GravityTorqueFromUrdf.py` + `_0.00kg.json`, `_0.10kg.json` | The URDF gravity model (reads this repo's URDF and arm files) and its independent cross-check. |
| `pan_roll_gripper.py` | Gravity on pan, roll and gripper over the slider box, with and without 100 g, and the pan on a tilted cart. |

### 7.1 What the plant models **[Changed]**

As GravityDesign.md 7.1, plus: pan, roll and gripper as geared two-mass joints like the comp joints (reflected rotor
0.008 kg m^2; link inertia about the axis from the URDF at the start pose, pan 0.005 at the feel pose; roll and gripper
a 3e-4 / 2e-4 floor for the fingers on them, their own links being 4e-5 / 1.6e-5); free-joint friction swept at three
levels (F0 0.04 / 0.08 / 0.15 N m with 20 / 30 / 40 % load share: a Coast breakaway at the output of 0.05 / 0.11 /
0.25 N m); output bearing friction (pan 0.01, roll and gripper 0.002 N m); an energised motor brakes at duty 0; torque
off floats, or brakes (`coast_brakes`); every fault exit coasts (a silent servo keeps its last goal until it answers);
a USB drop while guiding latches and coasts on return; injected heat and supply steps.

### 7.2 Results (all 323 scenarios: `out_guide_final_table.txt`)

**Free joints: steady push, plain Coast against hand guiding** (`free_feel`; force at the hand; "assisted" = the force
while the assist was on, which a stiff joint's stick-slip interrupts part of the time):

| Joint | Friction | 10 deg/s | 30 deg/s | faster |
|---|---|---|---|---|
| pan (30 cm) | low | 0.17 N = Coast (no band) | 0.20 = Coast | = Coast |
| pan | mid | 0.33 -> **0.24 N (-27 %)** | 0.36 = Coast (above the band) | = Coast |
| pan | high | 0.64 -> **0.40 N (-38 %)** | 0.67 -> **0.54 N (-19 %)** | = Coast from 50 deg/s |
| roll (3 cm) | low | 1.48 N = Coast (no band) | 1.76 = Coast | = Coast |
| roll | mid | 3.03 -> **2.11 N (-30 %)** | 3.34 = Coast | = Coast |
| roll | high | 6.12 -> **3.90 N assisted (-36 %)**, 4.31 N average | 6.47 -> **5.11 N (-21 %)** | = Coast from 60 deg/s |
| gripper (5 cm) | low | 0.89 N = Coast (no band) | 1.06 = Coast | = Coast |
| gripper | mid | 1.82 -> **1.28 N (-30 %)** | 2.01 = Coast | = Coast |
| gripper | high | 3.67 -> **2.29 N assisted (-38 %)**, 2.55 N average | 3.88 -> **3.05 N (-21 %)** | = Coast from 60 deg/s |

Breakaway (getting a joint at rest moving) is exactly Coast's: the joint floats at rest (roll 1.7 / 3.9 / 8.4 N,
gripper 1.0 / 2.3 / 5.0 N, pan 0.27 / 0.36 / 0.87 N at low / mid / high friction).

**If torque off brakes the motor** (`free_brake`, mid friction): Coast drags hard and the assist cuts it:

| Joint | 10 deg/s | 30 deg/s | 60 deg/s |
|---|---|---|---|
| roll | 6.40 -> 2.12 N (-67 %) | 14.3 -> 3.35 N (-77 %) | 22.3 -> 5.24 N (-76 %) |
| gripper | 3.84 -> 1.27 N (-67 %) | 8.60 -> 2.03 N (-76 %) | 13.4 -> 3.13 N (-77 %) |

**Quick moves** (`free_flick`, 30 deg minimum-jerk, mid friction, peak force at the hand): pan 1.93 N (0.4 s) and
0.73 N (1.0 s); roll 14.9 / 5.9 N; gripper 8.9 / 3.5 N: **identical with and without hand guiding** (the assist never
switches on during a flick).

**Release** (`free_release`; the hand turns the joint for 1.2 s, then lets go; glide, time to stop, motion in the last
2 s): from 15 deg/s (assisted when let go): pan 0.49-0.67 deg in 0.03-0.06 s, roll 0.06-0.15 deg, gripper 0.01-0.08 deg;
from 30 deg/s: 0.16-2.0 deg (2.0 = the pan at low friction, plain Coast); from 80 deg/s (floating, as in Coast): roll and gripper 2.5-8.0 deg in 0.07-0.21 s, the
same as Coast. **Motion in the last 2 s: 0.000 deg in all 37.** Pan, roll and gripper together: same.

**Assist settings that are wrong** (`assist_params`):

| Case | Result |
|---|---|
| k_f 0.9 (roll), lead 1.1 (roll), lead 0.95 (lift), kv 1.5 x the prior (pan), band top 200 deg/s (gripper) | refused before any write, with the reason |
| free joint friction 40 % of calibrated (feed-forward 1.25 x the real friction), let go at 15 deg/s | glides 0.17-0.97 deg, stops in 0.07 s or less |
| free joint friction 25 % of calibrated with k_f 0.7 (2.8 x the real friction) | runs on 6.8-8.4 deg for 0.5 s, then the stroke ends: floats, stops; no creep |
| at the bounds (k_f 0.7, c 0.9, kv at the 1.10 x prior limit) | glides 0.07-0.87 deg |
| validation bypassed: kv 1.25 x the real slope with c 0.9 (lead 1.12 x the whole drag) | glides 0.16-0.96 deg (the band's float bounds it) |
| lift, elbow, wrist friction 40 % of calibrated, k_f 0.7 | the probe switches the lift's and elbow's feed-forward off; release glide 0.73 deg |
| the same with the probe disabled | runs on 22.4 deg in 1.2 s before the stroke ends it |

**Comp joints**:

| Claim | Result |
|---|---|
| Correct starts succeed | 14/14 at four poses, three frictions, model 0.9-1.1: no abort; hover drift 0.00 deg on all six joints |
| Bad starts never move the arm far | sign errors refused before any write; gross gains refused or abort within 25 steps |
| Feel | 2.2 |
| Release (lift from 30 deg/s, feed-forward on) | c 0.6: 0.60-1.95 deg, stops in 0.03-0.13 s; c 0.8: 0.78-2.24 deg; c 1.0: 0.97-3.6 deg; c 1.3 (refused by the bounds) at low friction runs on 28 deg |
| Runtime model error | caught on the first cycle by the URDF check; coasts |
| Stall 1.7 s | the arm holds during it (0.0 deg); coasts on resume |
| Hold here | 0.00 deg (P32 and P16, low and mid friction) |
| Speed | a 90 deg/s yank coasts; 40 deg/s runs |
| Supply | 8.2 V at entry refused; a sag to 7.6 V: 0.0 deg with the voltage scaling, 28.5 deg without (very low friction); 6.8 V coasts |
| Bus, USB, brown-out | 1 % drops and 3-reply bursts run on; 8 missed replies, every USB outage and every restart coast; the restart is caught in 37 ms |
| Faults coast, hands off | shoulder 20-31 deg in 0.25 s, 78-107 deg in 0.5 s; with a hand under the forearm 4.4-5.6 deg |
| Calibration | comp model within 1.1-3.7 units RMS; free friction within the gear + bearing; kv 240-252 (274 true); spin-down 12/12 |

What it does not prove: the friction, backlash and rotor inertia are swept, not measured; whether torque off floats;
the gear's forward and backward efficiencies are modelled as symmetric; the servo model follows the clean-room
firmware; the hands are simple controllers, not people. Bring-up measures the first three and the plant is re-run.

### 7.3 Test plan for the implementation

As GravityDesign.md 7.3, with:

- `back end/HandGuidePlant.py` + `back end/TestHandGuideLaw.py`: every row of 7.2 as an assertion, including: free
  joints float at rest and on flicks; assisted force under Coast's at 10 deg/s (mid, high); equal to Coast above the
  band; release glide under 1 deg from inside the band and no motion afterwards; every out-of-bounds setting refused;
  the comp probe switches the feed-forward off at 40 % friction; every fault exit is "coast"; Hold here 0.00 deg; no
  false fall when a hand lets go of a self-locking lift. A quick subset under 30 s for `RunAllTests.py`.
- `TestGuideBus.py`: as before, **Present_Load bit 10** (1224 -> -200, 200 -> +200), the reg-40 float write.
- `TestHandGuideBridge.py`: as before, plus `Access` "password" accepted and others refused; every fault path coasts
  through `AutoCoast` with the torque read back; the owner page gone for 3 s holds; supply refusals (8.2 V, a 0.6 V
  spread); `CaptureCadReference` refused while guiding; GraspHold keeps the gripper; per-joint torque expectations
  follow the free joints' states.
- Page: the button on another device, the E-stop label, the "goes limp" strip.

---

## 8. Implementation plan **[Changed]**

On `gravity-hand-guide`; each stage ends runnable with its tests passing and a local commit.

1. **G0, the law in the repo**: `back end/HandGuide.py` from `HandGuideSim/hand_guide_law.py`, `HandGuidePlant.py`
   from the plant; `TestHandGuideLaw.py` asserts 7.2.
2. **G1, the transport**: `GuideBus.py`, FakeServoSerial's SYNC_READ / SYNC_WRITE and register semantics, the
   **Present_Load bit-10 fix** with its test; the existing suites still pass.
3. **G2, a read-only servo dump** for the first session (`servo_dump`, reads only).
4. **G3, the bridge state**: hand_guide command (local and password access), refusals (supply, GraspHold), guide thread,
   supervisor frame, **coast exits through AutoCoast**, the preloaded Hold here, per-joint torque expectations,
   refusals of moves and captures, events, state, versions; `TestHandGuideBridge.py`; full `RunAllTests.py`.
5. **G4, calibration**: sweeps for all six joints, the fit, the spin-down, acceptance, the file, `validate()`.
6. **G5, Clancy**: the button (any device), Hold here, strip, E-stop label, gates, the calibration flow; docs.
7. **G6, review**: code review at high effort, PROGRESS.md with what is untested on the arm; then section 9.

---

## 9. First hardware session (with Aaran supporting the arm) **[Changed]**

Prerequisites as before (Aaran loads the branch's bridge himself; wheels locked; pad under the arm; E-stop on screen,
power switch in reach; a spring or kitchen scale and string; a multimeter). Every step starts from Hold with a hand
ready under the forearm. **Any fault makes the arm go limp now: the hand under the forearm is not optional.**

| Step | What | Pass | Abort |
|---|---|---|---|
| S0 Supply | Rail at the servo connector at rest and with the arm held stretched out; Present_Voltage of all six | at least 11.5 V with a 12 V supply (or the real value recorded); under 0.5 V sag; the six within 0.3 V. Set the entry threshold to max(8.5 V, resting reading - 1 V) | under 8.5 V at rest, over 1 V of sag, readings jumping: fix the power first |
| S1 Register dump | `servo_dump` (no writes) | as before; Present_Load reads with bit 10 | as before |
| S2 Friction and float **[Changed: D4]** | Coast, arm supported: force to start each joint moving and to keep it moving slowly, at the hand, both ways; **spin the roll and the gripper fast by hand: does the resistance grow with speed (brake) or stay about the same (float)?** | numbers recorded; float or brake recorded | none (passive) |
| S3 Passive sign check | Hold at three checked poses: Present_Load of lift, elbow, wrist against the URDF's sign (bit 10) | signs agree where the URDF effort exceeds 100 units | any disagreement |
| S4 Calibration | Section 5, including the free joints and the spin-down | every acceptance check; the spin-down agrees with S2 | any trip; more than 40 steps off a ramp; over 50 C |
| S5 Wrist alone | wrist comp, the rest in Hold; alpha 0.25 (it should sink slowly), then 0.5, 0.75, 0.95; **k_f 0** | as before | as before |
| S6 Elbow, then lift | the same ladder, k_f 0 | as before | as before; over 50 C; 0.5 V sag |
| S7 All six | comp lift, elbow, wrist; pan, roll, gripper free (k_f 0 first: they must feel exactly like Coast) | hover at five poses; Hold here holds with no visible motion; walls resist; free joints caught at their limits | anything not asked for |
| S8 Drills **[Changed: D1]** | E-stop while guiding (drops into the hand, as labelled); a 1 s debug pause of the guide thread (holds, then **goes limp into the hand**); optionally the USB cable pulled while Aaran holds the arm (holds during, limp when plugged back) | each as section 3.8 says | anything else |
| S9 Assist **[Changed: D4]** | k_f 0.5 on one joint at a time (roll, gripper, pan, then wrist, elbow, lift); c as calibrated. Move slowly, let go; move quickly, let go; let go after a 3 s slow move | slow moves feel lighter (roll, gripper: about 30 % in the sim); quick moves feel like Coast; let go: stops within 0.2 s and 2 deg, stays put for 5 s; no buzz or oscillation | any creep after letting go, oscillation, a glide over 5 deg: back to k_f 0 for that joint |
| S10 Heat | Hover stretched out for 10 minutes, a hand near | lift under 50 C (the 50 C warning would say "goes limp at 55 C") | 55 C (it goes limp by itself: be holding it) |

Afterwards: the measured friction, backlash, float or brake, kv into the plant; scenarios re-run; PROGRESS.md.

---

## 10. Decisions taken, and what is still open

**Taken (2026-10-10)**: D1-D5 above.

**Choices made in folding them in (flagged for Aaran):**

1. **Which exits count as "faults" (coast) and which as "endings" (hold).** Coast: everything in 3.8's coast row,
   including a control-loop stall (1.1-1.7 s, about once a day), 55 C, under 7 V, the stale collision check and the
   model checks. Hold: Hold here, Stop and hold, Hold, AI mode, and the owner page gone for 3 s. **Consequence: once a
   day or so the arm may go limp by itself while you are not holding it** (the stall). If you would rather the stall and
   the page gone end in a hold (as GravityDesign.md had), that is a one-line change per exit.
2. **A failed start returns to the entry hold** rather than coasting: the start is hands-off ("let go"), the arm was
   never handed over, and the entry hold is where it was. Hardware faults during the start still coast.
3. **The gripper while squeezing an object** keeps its squeeze unless you free it. Alternative: refuse to start.
4. **The free joints' assist is small by physics**, not by caution (2.3); it is about 30 % on slow, steady moves if the
   servos float in Coast. If S2 finds they brake, it is about 70 %.

**Open, measured in the first session**: friction and its load share per joint; float or brake with torque off; real
P, D, punch and dead zones; the supply under load; the lift's heating; whether the sim's feel numbers hold; whether
k_f 0.7 is worth it on the free joints (it passes every bound in the sim: mid friction band 32 deg/s instead of 23).

**Unchanged from GravityDesign.md 10**: calibration file in git like `CadAlignment.json` (recommended yes); fixing the
12 V supply matters more than any software here.

---

## 11. Numbers and where they come from

| Number | Value | Source |
|---|---|---|
| Worst-case holding torque, bare arm | lift 0.864, elbow 0.453, wrist 0.117 N m | `design_numbers.py` 1 |
| Same in duty units at 7.5 / 8.9 / 12 V | lift 475 / 400 / 297; elbow 249 / 210 / 155; wrist 64 / 54 / 40 | `design_numbers.py` 1 |
| Comp caps | 650 / 450 / 250 units = 1.40 / 0.97 / 0.54 N m at 8.9 V; walls 750; overload trip 800 | `design_numbers.py` 1 |
| Comp back-driven current at ceiling and speed cap | 1.72 / 1.36 / 1.24 A (overcurrent 2.0 A) | `design_numbers.py` 6 |
| Free caps | pan 350, roll 400, gripper 400 units (0.76 / 0.86 / 0.86 N m); caught 400; 1.26 / 1.63 / 1.63 A at v_hi | `design_numbers.py` 9 |
| Speed caps, tip speeds | 45 / 50 / 80 deg/s = 0.32 / 0.26 / 0.22 m/s | `design_numbers.py` 2 |
| Free joints' band top (c 0.9, k_f 0.5) | none / 23 / 43 deg/s at low / mid / high plant friction; k_f 0.7: 16 / 32 / 61 | `design_numbers.py` 9 |
| Energised drag left by the lead | (1 - c) x 274 + 1.5 duty units per rad/s; c 0.9: 0.08 N m s/rad at the hand on the roll | `free_drag` |
| Bus per cycle | read 1.58 ms; write 3 joints 0.35, 6 joints 0.62; float write 0.14; 4th-cycle read 1.64; worst 3.98 ms on the wire; cycle 3.7-4.8 ms | `design_numbers.py` 3 |
| Start | 0.6 + 1.2 + 0.4 (probe) = 2.2 s hands off; abort beyond 24 steps (1.7 cm at the tip) or 8 deg/s | `design_numbers.py` 4, 7 |
| Supply | entry at least 8.5 V (lift worst + 20 % = 502 units there), spread 0.5 V; guiding floor 7.0 V (610 units) | `design_numbers.py` 10 |
| Back-EMF | 0.57 N m s/rad; 265 units per rad/s at 8.9 V | `design_numbers.py` 8 |
| Lift holding current and heat | 0.80 A, 2.9 W bare; 1.18 A, 6.1 W with 100 g | `design_numbers.py` 5 |
| Drop on a coast, hands off | shoulder 20-31 deg in 0.25 s, 78-107 deg in 0.5 s; with a hand under the forearm 4.4-5.6 deg | `exit`, `faults_coast` |
| Calibration accuracy | comp 1.1-3.7 units RMS; free f_kin = gear + bearing; kv 240-252 (274 true); spin-down 12/12 | `out_gap_ident.txt` |
| Feel, release, faults | sections 2.2, 2.3, 3.4, 3.8, 7.2 | `out_guide_final_table.txt` |

---

## 12. As built (2026-10-10, branch `gravity-hand-guide`)

Built offline only: nothing was sent to the arm, the live bridge or the dev server. Stages, each a local commit:

| Stage | What | Files | Tests |
|---|---|---|---|
| G0 | The law, ported line for line (this arm's ReferenceRaw per law; the entry's supply spread, supply against V_cal and 50 C checks added to `begin`) | `back end/HandGuide.py`, `back end/HandGuidePlant.py` (runs HandGuideSim's plant and scenarios on it) | `TestHandGuideLaw.py`: 22 checks of 7.2 on the plant (about 10 s); `--full` all 323 scenarios |
| G1 | The lean transport; **Present_Load decoded with bit 10** (1224 reads -200); the fake bus answers SYNC_READ/SYNC_WRITE and reports multi-turn positions | `back end/GuideBus.py`, `feetech_simple.py`, `FakeServoSerial.py` | `TestGuideBus.py` (8) |
| G2 | `GET /servo/dump`: every servo's registers 0..70 in one read-only SYNC_READ, decoded (bring-up S1) | `back end/bridge_server.py` (`ServoDump`) | `TestHandGuideBridge.py` (writes nothing) |
| G3 | `hand_guide` in the bridge: entry checks (3.7), the 100 Hz guide thread, the 30 Hz supervisor, every exit, refusals, state and `/health` | `back end/bridge_server.py` (the `HandGuide*` methods), `BridgeVersion.json` | `TestHandGuideBridge.py` |
| G4 | `hand_guide_calibrate`: the sweeps, the spin-down, the fit and its acceptance, the file | `back end/HandGuideCalibration.py` | `TestHandGuideCalibration.py` (8, on the plant), `TestHandGuideBridge.py` (a whole calibration on the fake bus, then guiding with it) |
| G5 | Clancy: the button, the strip, the emergency stop's label, the gate, the heartbeat, AI mode | `src/lib/HandGuide.ts`, `src/lib/BridgeCommandGate.ts`, `src/App.tsx`, `src/components/SettingsPanel.tsx` | `TestHandGuide.mjs` (7), `TestBridgeCommandGate.mjs` |

**How it is used.** Manual mode, the Motion panel's sixth button: **Calibrate guiding** (until a calibration exists;
again from Settings, Hand guiding), **Stop calibration**, **Hand guide**, **Hold here**. The first Hand guide in a
browser session shows a confirmation (what it does, the hands-off start, that faults and the emergency stop make the
arm go limp, keep the screen on). The bridge accepts `hand_guide` only from a page's live connection (`/ws`; `POST
/motion` is refused) from this PC or a device with the password; that connection is the session's owner and sends
`hand_guide_heartbeat` every 0.5 s.

**Choices made in building it (flagged):**
1. The supervisor's heartbeat limit is **150 ms**, not 60 ms: 3.6 logs 50-150 ms late cycles and coasts after 150 ms, and
   a 60 ms supervisor would have coasted every logged one first. A stall still coasts (D1).
2. A calibration sweep stops (holding) when a joint is more than **80 steps** off its ramp, not 40: with mid friction the
   loaded lift's position error alone changes by about 48 steps between moving up and down.
3. The **hover test** that sets alpha per joint (GravityDesign.md 5, step 4) is not built: alpha is 0.95 for every joint.
4. The calibration's comp sweeps are **one configuration per joint** (from where the arm holds), each joint through
   +-20 to +-30 deg; the MAP prior carries the rest. On the plant this recovers a 10 % heavier arm within 5 %.
5. A gripper squeezing an object **keeps its squeeze** without a question (roles show it as "keep").
6. Owner-page-gone and a failed start end **holding**, as 3.8 says; every other end of a session that is not a Hold
   here, Stop and hold or Hold goes limp.
7. Coast pressed while guiding goes limp (it is Coast); the TORQUE lamp reads GUIDE while guiding.
8. A prepared camera calibration does not block hand guiding: the start cancels it, as Stop and hold does.
9. **The guide loop's clock is `time.perf_counter`**, not the bridge's `time.monotonic`: on this PC's Python 3.12 the
   latter is GetTickCount64, 15.6 ms a tick, which would have quantised the law's 40 ms speed estimate at 100 Hz into
   faked speeds (and false over-speed exits). The plant never showed it (its clock is exact).
10. The free joints' slider arcs are read as one URDF interval each; entry refuses an arc that would cross the URDF's
    180 deg (none does today).

**Untested on the arm** (bring-up, section 9): everything. In particular whether Present_Load's bit-10 sign is the
servo's (S1/S3: the calibration refuses a model whose sign disagrees with the URDF, so a wrong sign cannot guide);
whether the servos report multi-turn positions in position mode (the pan's, lift's, wrist's and roll's arcs cross
encoder zero; since the 2026-10-10 review a wrapping servo is unwrapped and its goals follow its own frame, but what a
single-turn servo does with a goal just past 0 or 4095 is not known); whether torque off floats (S2); the friction, kv
and supply under load; the 5 ms reply deadline on the CH343 adapter.

### Safety review fixes (2026-10-10, branch `hand-guide-safety-fixes`)

| Finding | Now |
|---|---|
| H1 a blocked calibration sweep "stopped, holding" while pushing on | Every calibration stop holds each joint where it is (`CalibrationRun.HoldHere`: the lift, elbow and wrist with the URDF prior's gravity), never at the sweep's goal. Sweep limits: 2 x (prior gravity + back-EMF at the goal's speed) + 200, at most 750 (the plant's worst need: the lift's 645, high friction, 10 % heavier); the gripper's opening and closing at most 250 (about 7 N at the jaw). Every energised joint is tracking-checked on every segment, the spins too, two readings in a row |
| H2 one bad reading dropped the arm | Status bits, torque, Lock and the supply act once read twice in a row (worst added delay one 10 ms cycle; a restart is caught in 49 ms on the plant, was 37); a position jump over 3 x the speed cap is a missed reply (the joint keeps its last goal), two in a row coast; the effort scaling takes the comp joints' median supply; an exit's fresh readings, and the bridge's measured pose, only where plausible |
| H3 raw 0 | Positions unwrapped into the session's (or the calibration's) own frame; every goal is the reported position plus an offset |
| M2 going limp energised the floating joints first | A fault, Coast and the emergency stop first send Torque_Enable 0 to all six; the torque-off after it writes no Goal_Speed to a position-mode joint already off |
| L1 a kept gripper was not watched | Its replies, status, torque, Lock, heat and supply end guiding like the others' |
| L2 only the first comp joint's supply | The floor is the lowest of all six (each confirmed twice) |
| L3 Alpha, V_cal unbounded | validate(): alpha 0.25..1.0 (bring-up S5's ladder to the design's 1.0), V_cal 8.2..12.6 V |
| L4 a start inside the guard band was unguarded | The bridge refuses it ("within 3 deg of ...: move it clear first"); the law also arms the walls from the entry pose |
| L5 Coast while guiding | Reads "Coast: arm drops" (danger style and a title); its accessible name stays Coast |
| L6 | The owner's heartbeat entry goes with its session; Present_Current decoded with bit 15 in the driver too (the vendor SDK's `SMS_STS::ReadCurrent`) |

**Left for Aaran's decision** (unchanged): H4, a 2-7 s control stall ends limp after a stiff soft-hold (the reviewer
suggests ending in a preloaded hold when every joint is still where it was); M1, no idle timeout for an unattended
session; M3, the calibration can be started from another device.

