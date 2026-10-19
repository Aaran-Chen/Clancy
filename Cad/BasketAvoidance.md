# Basket, body, camera-head and rearward avoidance

Basket contact is blocked using the actual CAD surfaces, including lower bins
and supports. The empty tray interior and the gaps between trays are usable
when the whole arm and its route clear the material. A bounding-volume tree
accelerates triangle contact tests; its boxes only cull irrelevant surfaces and
never stand in for the entire solid cart. Solid-containment tests also reject
arm parts embedded in basket material. The geometry uses the same selected arm
mount and CAD transforms as the renderer, independently of the display origin
or camera calibration.

The fixed mast, its mounting structure, and the parked camera head are also
collision obstacles. RobotBodySurface.json contains all six Mast/Head meshes
from the same XLeRobotBody.glb used by the view, in the same assembly coordinates.
Their transforms follow the selected arm mount. Both target points and every
moving arm envelope are checked against these surfaces throughout a route.
The head is modeled in its fixed displayed pose; there is no head-motion
telemetry. Visibility settings do not disable collision checks.

The mast/head model preserves open space between separate parts. Two connected
mounting fragments in the source are not watertight, so those fragments alone
use enclosing convex hulls; this can conservatively block cavities within those
fragments. The entire mast is never replaced by one solid bounding box.
The body surface has its own tree and hash. Basket bounds remain separate so
the taller head does not move the rim used by the tray-entry planner.

Every moving arm visual is enclosed by a box in its CAD link frame, including
the wrist motor, fixed jaw and moving jaw. The fixed mounting bracket is attached
to the cart and is excluded. The rotating shoulder collar is checked over its
entire revolution. There is no additional static clearance around the basket.
Since robot-body-cad-exact-v8 (approved by Aaran 2026-10-04: "If it won't hit
the basket in the CAD view, just assume it won't in real life") basket, cart
and body contact is CAD-exact: a pose, or the region a route commands, is
refused only where a link's CAD box meets a surface
(ContactTrackingAllowanceRaw 0). Before it, a nominally clear pose was rejected
when its 16-step tracking envelope could contact a surface: at 08:32 that day
the bridge refused to close the jaws at a grasp pose the arm stood in, the
gripper box 2.4 cm from the cart. The 16-encoder-step allowance per joint
(JointTrackingAllowanceRaw) remains for what the arm itself does, because
feedback-gated moves can settle a little short or long: the route monitor,
braking room and the rearward limit keep it. A gripper-only move is judged on
the moving jaw's boxes over its travel, the arm holding where it is. These
values are in the shared CollisionModel.json file; BasketSurface.json contains
the CAD triangles, closed-solid components and tree.

The gap this leaves near a measured scene (2026-10-07 audit M5, kept and stated
here). A measured scene's obstacles and observed free space are certified the
same way, over the commanded region with no allowance, and the observed support
under the task object is clipped to its measured top with no pad (the other
scene obstacles get 0.5 cm). The route monitor still accepts every joint 16
steps beyond that region, so a joint sagging under load inside the monitor's
bound can put the gripper below what was certified: on the recorded near-desk
route ends the worst -16/0/+16 combination lowers the gripper's lowest point by
1.38-1.95 cm (`.claude/Audit/TrackingAllowanceDrop.py`). Near a measured support
or under a shelf board the jaws can therefore touch what was certified at 0 cm.
The obvious fixes each break something the arm needs: certifying the scene over
the monitor's region (or padding the support by the gripper's worst 16-step
drop) refuses every grasp low on a measured support (the grasp plan keeps the
open jaw only 0.2 cm above it, `GraspRules.OpenFloorClearanceCm`), and tightening
the monitor near the scene turns ordinary sag into latched faults, which coast
the arm. So it stays as it is until the sag near supports is measured on the arm
(the audit could not: hardware), with the arm's measured sag already in the
grasp plan (`GraspRules.GraspSagCm`, 0.45 cm) and the app's guard keeping every
gripper part 0.4 cm from measured objects. Treat a grasp low on a measured
support as able to touch it.

### Directional motion enclosure

The former check expanded every box equally in every direction by the sum of
its joint-motion radii. This could add about 4 cm around a jaw, including in
directions the joints could barely move it, rejecting visibly clear poses.
The spherical bound remains a cheap first test. When it overlaps an obstacle,
ArmMotionEnvelope.py and its TypeScript counterpart refine that same interval
with a directional enclosure. Each obstacle must overlap both conservative
enclosures to remain a possible contact. The encoder allowance is unchanged.

For every corner p of a link box at the interval midpoint, joint i contributes
the derivative axis_i cross (p - joint_origin_i). Project these derivatives
onto the midpoint box axes and sum their absolute values times the angular
half-widths. The extrema over all eight corners enclose the linear term for
every point inside the box. Joint directions can be positive or negative;
the symmetric angular interval makes the absolute projection the same.

The nonlinear remainder is also enclosed, not omitted: add 0.5 * h^T H h
on every axis, where h is the vector of angular half-widths in radians and
H_ij is the existing motion radius of the later of ancestor joints i and j.
Non-ancestor rows and columns are zero. In the serial rigid-transform product,
each mixed derivative differentiates two rotations, each with operator norm
at most one, acting on the point relative to the later joint. Its distance is
bounded by that joint's motion radius at every pose. The same argument applies
to a repeated derivative of one rotation. Taylor's remainder therefore bounds
the entire independent joint interval, including intermediate poses and
encoder error. A small numerical epsilon keeps the enclosure outward.

This refinement is used by target-pose checks and all path certificates,
including escape edges with their existing per-part inset. The browser uses
the same enclosure for its preview. Rearward-limit checks retain their existing
bound. No basket triangles, arm envelopes, or motor tolerances are removed.
Tests cover interval extremes and interior samples, previous false warnings,
real contact, tracking-only contact, Python/browser parity and fake-motor routes.

## Rearward workspace limit

The center-of-mass footprint restriction has been removed. Sideways, forward
and downward reaches are allowed wherever the arm and its route clear the
basket, mast and head. The rear boundary is display Y = -30 cm, measured from
the same origin as the dashboard, with positive Y toward the robot's front.

RearwardLimit.MinimumDisplayYCm in CollisionModel.json sets that boundary.
Both the browser and bridge convert it into the selected arm's model frame.
Targets behind it are rejected. Every moving link envelope also stays ahead
of it, including its full joint-progress and tracking allowance throughout a
route. Clear endpoints do not permit an intermediate backward sweep. This is
a workspace restriction, not a balance or payload calculation.

## Checking a motion

The bridge rejects colliding or rearward target poses before writing any motor
goal. A motion edge is accepted only if every pose the route monitor would
accept during it is certified clear: the independent joint-progress box between
its two waypoints, widened by the tracking allowance. That whole region is
subdivided directly, largest arm motion first, so sub-boxes shrink toward exact
poses instead of each keeping the full allowance; uncertain intervals are
subdivided and inconclusive checks reject the edge. This avoids relying on a
sparse tip trajectory or simultaneous joint arrival.

A long edge's single box spans poses far from the straight line between its
ends (one joint finished, another not started), which is what used to force
slow detours. Each straight leg is therefore certified through waypoints at
most 6 degrees of joint travel apart, whose short boxes stay close to the line.
Once a route is chosen, waypoints are dropped again wherever a longer stretch of
a leg is clear at the first check: in open space a straight leg becomes one
edge, so the arm cruises along it without slowing at waypoints. The bridge
passes the remaining intermediate waypoints without stopping (see Execution).

### Leaving a pose inside the allowance

A measured arm pose is where the arm physically is. When that pose is clear of
the model but lies inside the tracking allowance of an obstacle, or behind the
rearward limit (for example parked upright beside the mast, or hand-posed in Coast),
every route used to be refused, because every edge leaving it inherits the
start's own envelope. Now the planner leaves by the shortest checked step: it
tries the direction toward the goal, then single-joint and two-joint directions,
finds the nearest pose with the full allowance and every envelope ahead of the
rearward limit, and certifies that one edge with the start's measured clearance on the
faces at the start pose and the full allowance on the far faces. The escape
edge retains the measured contact clearance and rearward envelope allowance
of the starting pose. Everything after that first waypoint
uses the full checks.

A pose that overlaps the model is usually an arm lying on the cart: an
automatic coast drops the arm onto the basket rim, and the link boxes enclose
the parts loosely. On the hardware, an arm resting on the rim showed 0.7 to 1.8
cm of box overlap. Such a pose may also be left: each overlapping box is inset
by its own overlap plus 3 mm (the resolution of the subdivided check) for the
escape edge, so the escape can press at most 3 mm further in, and in practice
lifts straight off (from the rim, a shoulder raise of about 10 degrees clears it).
Overlap deeper than 2.5 cm means the model or the pose is wrong, and is still
refused ("Coast and move the arm clear by hand").

## Planning

The planner first tries the straight line as gated waypoints. If that is
blocked it ranks small repairs by estimated time: single bends at a quarter,
half and three quarters of the way, parallel offsets (raise, traverse, return)
and coordinated subsets of joints, each leg gated. Then ordered joint moves,
then a bidirectional joint-space search. Before returning a detour it removes
bends and pulls them toward the direct line; every replacement leg must pass
the gated check. Every waypoint is snapped to the nearest pose the servos can
hold exactly before it is certified, so the bridge executes exactly the
certified edges; it confirms that every waypoint converts to servo steps and
back unchanged, and otherwise checks every quantized edge again. Only the
leading escape waypoint may use the start relief. (Calibration routes also had
a shoulder-load check until it was removed on 2026-10-03.)

The time estimate uses the current throttle: a leg costs its slowest joint's
travel at cruise speed plus a feedback frame, and a route adds one slowing
approach at the end. It is a relative command-time estimate, not a measured
hardware ETA or a proof of the globally fastest trajectory.

Cartesian IK evaluates the alternate elbow and forward/backward configurations
before rejecting an orientation. Elbow preference and joint travel rank valid
poses; a colliding or rearward preferred pose cannot hide another clear pose at
the same XYZ and pitch. The preview, automatic wrist recovery and green angle
guide use this selection. Send quantizes every feasible candidate, merges
solutions a few encoder steps apart, plans the gated line and small repairs for
each, and keeps the fastest route. Only then does it try tray entry and the
joint-space search.

For a point below the basket rim, the Cartesian planner can construct an entry
through the open top. It starts offline at the final pose and solves a sequence
of raised positions at the same XY and pitch. Each quantized edge must pass the
full independent-progress check. The bridge plans the approach above the rim,
reverses the certified escape sequence to descend, and removes waypoints only
when the longer edge also passes. Final XYZ and pitch stay unchanged. Most tray
targets are now reached by the gated line or a small repair before this runs.
Search remains bounded and cannot guarantee discovery of every physically
possible route: in an offline sample of 60 random clear start and goal pose
pairs, 2 still found no route in time. Hard cases are long, narrow passages
around the mast and head.

Cartesian moves keep roll and jaw position fixed. Other arm joints may
temporarily move away and return even when their requested final angle matches
the starting angle. Home and manual joint destinations use the same planner.
Routes retain joint limits; the calibration shoulder-load restriction was
removed on 2026-10-03, so nothing limits a pose's load. There is no minimum tip
height: reachable poses below the bin rim are allowed wherever
the whole arm and route clear the collision and rearward checks. The calibration
sampling Z range only chooses observation positions; it does not constrain
manual or Cartesian motion.

The unused MinimumTipZCm field is retained in WorkspaceFrame.json solely for
older bridge processes during rollout. The updated bridge and UI ignore it.

## Execution

Only one waypoint is commanded at a time, and feedback outside the checked
region of the current edge still stops the route and coasts every motor.

Intermediate waypoints are flown past: once every joint is within 24 encoder
steps of the waypoint, or already beyond it along its leg, the next edge starts
without parking. Real servos do not arrive together, and at speed a loaded joint
can run past a waypoint before the others reach it. Each edge is therefore
certified, and monitored, 32 encoder steps (the waypoint reach) beyond every
flown-past waypoint along each moving joint's travel, plus the 16-step
allowance. A joint that moved into such a waypoint may still be settling from it
when the next edge starts, so it keeps that reach on both sides while it holds
still, until the route passes a tight waypoint. Joints that did not move keep
only the allowance. The bridge assigns the reach after planning, on the
quantized route, and its route monitor accepts the same region for every joint,
with the one exception described below.

The reach is tried with 64 subdivisions. Most waypoints that look blocked at a
cheaper budget are only under-split, and each one that gives up costs a stop:
at 8 the table route stopped at nearly every waypoint. A whole route may spend
1.2 s on these checks (ROUTE_REACH_ASSIGN_SECONDS); after that the remaining
waypoints fall back to 8 subdivisions, which costs speed and never clearance.

At 100% the braking room before each waypoint, not a joint's ceiling, sets the
pace: with 32 steps of reach the lift slowed from about 400 to 200 steps/s at
every one of the nine waypoints from Home to the hover over the can. A wider
reach everywhere does not certify (it widens the box on every joint, off the
leg). But a joint that carries on the same way after a waypoint only needs it
ahead, along the leg: its overshoot lies inside the next stage's own range as
long as it is no more than its travel on that stage, and the next stage needs
no more reach at its start, because a joint trailing the waypoint is within the
fly-by tolerance. So before the plain reach each waypoint tries, for those
joints only (not the gripper), half their travel on the stage before it, capped
at their travel on the next (WideWaypointReach); the route monitor accepts and
the braking cap uses exactly the reach each waypoint was certified with
(RoutePath.Reach, per joint). A check that passes costs about 0.03 s more than
the plain one, one that misses 0.1-0.2 s, while the time it saves is largest
where braking binds hardest, so it is tried at 50% and above, at 64
subdivisions, until it misses twice on a route. On the servo model at 100% it
took the home-to-hover route from 3.83 to 3.40 s and calibration's first pose
from 3.57 to 3.13 s, planning included about 0.4 s better each.

Where the reach still cannot be certified (close to an obstacle), the waypoint
is tight: the next edge starts only once every joint, moving or not, is within
8 steps of it (half the allowance, because the last joint to arrive is usually
that far short when the stage switches), or is past it and keeps moving the same
way on the next edge, whose box lies ahead of it. Joints approach a tight
waypoint slowly enough to stop inside the plain allowance even if they lurch to
twice their command. The arm does not park there. The escape waypoint from a
pose inside the allowance is always tight. The escape edge is certified with the
start pose's measured clearance on its start faces but monitored with the full
allowance there, deliberately: a trip coasts the arm, which would drop it beside
the mast.

The reach has two ends, and near an obstacle often only one fits. Beside the
basket rim (2026-10-03, the lift off the can) every edge certified the reach
beyond its own waypoint, but not the reach at its start, where joints settling
from the waypoint before would swing back toward the rim. Both waypoints used to
become tight: four of the five on that lift. Now (route_planner_version
flyby-directional-v7) the waypoint before becomes an arrive waypoint instead:
its own edge keeps the reach it certified with, so it is approached like a
flown waypoint (its reach plus the allowance to stop in, 20% overshoot), but the
next edge starts only as at a tight waypoint, once every joint is within 8
steps of it or past it and carrying on, so the next edge needs no reach at its
start. A joint that overshot and turns or holds still on the next edge comes
back first. Only an edge that cannot take its own reach either way still makes
its waypoint tight. On the servo model the lift took 2.8 s at 100% and now
takes 1.8 s, with one tight and three arrive waypoints.

An edge whose own 32-step reach does not certify tries a reduced 16 steps
(ROUTE_REDUCED_REACH_RAW) before its waypoint becomes tight. The arm still flies
past such a waypoint: a joint trailing it within the 24-step fly-by tolerance is
inside the next edge's start, which takes the same 16 steps for the joints
settling from it (16 plus the 16-step allowance). It is approached as a tight
waypoint is, allowing a lurch to twice the command, with the reduced reach plus
the allowance to stop in. The descent to the can (2026-10-03 14:50:27) had three
tight waypoints and has one, 2.5 s to 2.1 s at 100%. Shrinking the previous
waypoint to it, instead of making that one an arrive waypoint, was tried and was
slower on every route it changed: a fast approach and a short wait beat a slow
approach flown past.

The route end has a reach too, for the joints that need it (route_executor_version
synchronized-v2, EndReach). The stage into the last waypoint accepts a joint that
carries on through it up to that waypoint's reach plus the allowance past it, but
the last stage used to accept it only its own travel plus the allowance. Where
the reach was the larger, a joint the stage before rightly let overshoot left the
checked route the moment the last stage began: 2026-10-04 23:50:01, the hover
beside the pill bottle, the pan stood 36 steps past its last waypoint (3037),
19 past the route end (3020), with a 32-step reach against 17 steps of travel;
the last stage allowed 16 past the end, and the arm coasted over the desk. Each
such joint now gets the difference past the route end (there, 15 for the pan and
12 for the wrist flex), certified with the last stage like any waypoint reach,
so the last stage accepts exactly what the stage before did. Where it does not
certify, the end keeps the plain allowance. Braking at the end is unchanged: it
still plans every stop inside the allowance (12 of the 16 steps), so the reach is
margin, not a target, and no route flies differently. On the 21 offline cases
(MeasureRouteSpeed's 17 and 2026-10-04's 15:31 hover, 15:32 descent, 08:32 lift
and the 23:49:53 hover) 12 routes had such a joint; every one of the 12 end
reaches certified, no route was refused or changed a waypoint, tight or arrive
waypoint, simulated flying times were identical at 18% and 100%, and none
tripped on the harsh servo models. A stage that ends at a tight waypoint has the
same mismatch for a joint carrying on into it from a flown one, and keeps it.

Why the pan ran on 36 steps is not known. At the waypoint the braking cap allows
it about 100 steps/s, from which the model stops it within 29 steps (the 17 to
the end plus 12), even 20% over the command; the same route planned offline
commands the pan at most 109 steps/s there, and the 370 stops logged in
route_timing.jsonl that day fit the braking model within 2.3 steps. Running on
36 steps from 100 steps/s takes about 0.31 s before braking starts: a control
frame delayed a few hundred milliseconds, under the 750 ms heartbeat, would
explain it (a 1.56 s delay of
the same loop tripped the heartbeat at 23:11 that day, back end/MotorProtection.md),
but a route that faulted left no frames. Now it does: a route a latched fault
stops is kept in route_timing.jsonl with the fault, its stage and every
waypoint's reach per joint (reach_raw), as an arrival is.

Joints are synchronized (route_executor_version synchronized-v1; v2 adds the end reach above). Every
moving joint has a limit: its ceiling at the throttle, the speed it could still
stop from inside its checked region beyond the current waypoint (its reach plus
the allowance, only the allowance at a tight waypoint or the route end), the
same beyond the waypoint where it must stop at the latest (its own direction
reverses, or the route ends), and the final ramp. The joint with the longest
time to go at those limits runs at its limit and the others are slowed to
match, so they arrive together, which keeps the arm on the checked straight
leg. Real joints still drift apart, so a joint that leads the slowest joint
still travelling by more than 8 steps is eased off, and stops once it leads by
the full reach; a joint still finishing the last waypoint counts as at the
start of the new stage, not behind it. A joint that carries on the same way
after a flown-past waypoint keeps pace with the others through the fly-by
tolerance instead of stopping in it.

Until 2026-10-03 the joints were timed to their ceilings and then each was
capped by its own braking bound, so at 100% (where braking, not the ceiling,
sets the pace) the joints with little travel per stage reached each waypoint
early and stood still until the lead joint arrived, and at the next stage
start the leash held the lead joint back for a joint finishing the last
waypoint within the fly-by tolerance. Joints started and stopped at every
waypoint and the tip zigzagged: on the servo model the home-to-hover route of
2026-10-03 flew 1.64 times the straight tip distance on a route planned at
1.08 (now 1.09), and at 18% it took 6.1 s (now 4.7 s). MeasureRouteSpeed.py
measures these on real requests.

Joints cruise past waypoints and slow only over the distance before they must
stop: the route end, or the waypoint where their own direction reverses. That
final ramp runs over 180 steps rather than a slider drag's 320: the braking
model below already limits every approach to a waypoint, so the longer ramp
only made the last seconds of every route slow. Measured on the 40-case model,
shortening it cut mean execution from 9.3 s to 8.5 s at 100% with no route
monitor trip on the harsh servo models and the final tip error unchanged within
a millimetre. No joint approaches a waypoint faster than it could still stop
inside its reach (none at a tight waypoint) and allowance, assuming up to 0.18 s
from feedback to effect, at least 1000 steps/s² of braking and up to 20% speed
over the command. A joint that reaches a waypoint first waits in velocity mode
at zero speed rather than switching to position mode, which would jerk the arm
at every waypoint.

Those limits come from the hardware. At 100% speed the first fly-by version
tripped "shoulder_lift left the checked basket route" on a long Home route: the
loaded shoulder overshot a flown-past waypoint by more than the allowance while
the elbow was still arriving, and the arm coasted. Servo models with 2 to 5
frames of command latency, 1000 to 1500 steps/s² of braking and a shoulder that
sticks for up to 0.4 s and then lurches reproduce that trip; the current rules
complete home-to-table and table-to-home routes on all of them, at 60% and 100%,
without a trip, as they do for a servo that runs 30% faster than commanded
throughout (the logged hardware held within about 6% over 0.6 s windows).

On the hardware the current rules lifted the arm off the basket rim and ran
Home, the table target and Home again at 30%, 60% and 100% without a trip,
including the Home route that tripped before. Replaying the logged positions
against the monitor showed the closest approach to a bound: 4 steps with a
12-step tight tolerance (hence 8), and 3 steps where the elbow broke out of
stiction at about twice its command and overshot a tight waypoint (hence the
lurch allowance and letting a joint carry on). With those, the closest
approach while moving at 100% was 7 steps. A servo model in which every
breakaway runs at twice the command for 0.1 to 0.2 s still trips at 60% and
100%, so a sticky joint remains the most likely cause of a route trip.

Changed geometry/limits/alignment, faults, lost feedback, timeout, Stop, Coast,
or Disconnect clears the pending route. Stop and hold retains holding torque;
Emergency stop retains its torque-off behavior. No competing move replaces a
route in progress. Planning releases the motor lock so Stop can interrupt it.

A holding joint that sags past its goal tolerance is driven back by the velocity
follower once its short return is checked, with each end's own measured
clearance. If that return cannot be checked (for example resting beside the
mast), the servo's own position hold keeps pulling it back and the status says
"Hold correction paused near an obstacle; the servos are still holding". It
never coasts for this: that used to drop an arm parked beside the mast when a
joint sagged 20 steps.

## Editing targets

The UI allows an invalid drag preview to follow the pointer in its screen plane,
flashes it red, and blocks Send. On release it stays red for at most half a second
(App.tsx TargetSnapLimitMs). A worker, started when the edit began, first keeps the
position and turns only the wrist to the nearest reachable pitch (the cached green
guide when there is one, else 4 deg samples refined to 0.005 deg; typically 0.1-0.2 s).
If no wrist angle reaches the position, it pulls the target back along the edit, from
the last valid target toward the release, to the reachable edge (bisection, about
40 ms). Whatever the worker has not answered within the limit snaps back to the last
valid target. Every answer passes the same IK branch selection, joint limits, whole-arm
basket and rearward checks as Send. (The multi-start joint-space nearest-point search
still serves AI target snaps; it takes 3-18 s, too long for a release.) Escape, pointer
cancellation, and lost capture restore the starting target. Numeric fields use the same
search on Enter or blur.
A new edit cancels an older search, and results are rechecked against current
limits and telemetry before acceptance. These actions only stage a target; the
bridge still certifies the actual route when Send is pressed.

Shift-scroll adjusts wrist pitch proportionally to wheel distance, including
horizontal Shift-wheel events and line/page delta modes. A burst is one edit;
the UI previews at most once per frame and commits after scrolling stops.
Escape cancels the burst. Normal wheel and pinch gestures retain camera zoom.
The direction ray uses the selected CAD tool axis, passes through the target,
and continues behind it as a plain approach tail with one arrowhead ahead.

The green arc behind the target samples acceptable wrist angles at the current
XYZ position with the same joint limits, held tool angles, basket and rearward
checks as Send. Its ends are refined at valid/blocked transitions, and
disconnected ranges stay separate. Segment midpoints are checked too, splitting
gaps and direction changes between alternate IK configurations. A worker computes
it when the position or solver context changes; camera movement and wrist
scrolling reuse the result. Stale ranges are hidden while a changed target is
checked. It is a sampled orientation guide; Send and the bridge retain their
full pose and route validation.

## Versions and limits

Older bridges cannot receive motion from this UI until they advertise the same
collision model version, robot-body-cad-exact-v8 (CAD-exact contact; v6 added the Y = -30 cm limit). Calibration
sampling excludes blocked poses and uses the same planner. Archives include
collision source/configuration and planned raw waypoints so later analysis can
reproduce the route. The bridge advertises route_planner_version as flyby-cad-exact-v8
(v7 certified contact over the tracking allowance; v6 had no arrive waypoints) in health, state snapshots, and archived calibration route events,
and route_executor_version (synchronized-v2) in health and state snapshots.
AI Controlled WebApp/scripts/RestartBridge.ps1 waits for these versions after a restart.

The collision model uses nominal CAD mounting and dimensions. This is software
avoidance for the modeled basket and fixed mast/head, not detection of basket
contents, arbitrary objects, cables, people, the independently moving second
arm, or differences in the real build. The camera does not measure basket
placement for this guard. Balance is not checked. Regenerate the collision
model after changing the CAD assets or mounts.

From the repository root:

```powershell
& "Cad/.venv/Scripts/python.exe" "Cad/PrepareCollisionModel.py"
```

## Tests

The tests never open serial ports.

```powershell
& (Get-Content ".backend-python.txt") "back end/TestBasketCollision.py"
```

Interior/face rejection, all moving parts, clear endpoint pairs with a blocked
sweep, detours, independent servo speeds, quantization, waypoint sequencing,
cancellation, faults, tracking deviation and emergency interruption during
planning.

```powershell
& (Get-Content ".backend-python.txt") "back end/TestBasketRouteOptimization.py"
```

Direct motion without extra waypoints, gates that stay on the nominal line,
small repairs, unchanged tool angles, necessary move-away-and-return joints,
shortcuts that must not cut through an obstacle, load restrictions, global
fallback, and a fly-by route faster than the safe one-joint-at-a-time baseline.

```powershell
& (Get-Content ".backend-python.txt") "back end/TestFlyByRoutes.py"
```

The reported pose beside the mast reaching a table target (one relieved escape
edge, every later edge fully checked, executed to within 0.8 cm), hold drift
beside the mast corrected without coasting, an uncheckable hold return keeping
the servo hold, two arm poses measured lying on the basket rim lifting off and
reaching Home without any box pressing further in than the margin, refusal of a
start pressed deeper into the model, waypoints flown past without parking,
synchronized joint progress, and the braking cap. Two tests scan every encoder
position the route monitor accepts on every stage of a real route and of a
route with bends, and require it to lie inside a box certified for that stage;
a deliberately widened monitor fails them. Others check which joints keep the
waypoint reach (in the monitor and in reach assignment), that tight waypoints
are passed inside the allowance without parking, that a tight waypoint waits
for a joint that holds still but is still settling, and that a joint past a
tight waypoint may carry on only if it keeps moving the same way. The
2026-10-04 23:50:01 switch to the last stage is replayed on the re-planned
route: every position its monitor accepts lies in a certified box, the logged
pose passes the switch with the end reach and trips without it, and the end
reach goes only to joints carrying on with less travel than their reach, only
where it certifies. Execution uses a servo model with 30 Hz frames,
acceleration limits and a lagging shoulder.

```powershell
& (Get-Content ".backend-python.txt") "back end/TestRearwardLimit.py"
```

Rearward boundary equality and both arm mounts, translated display origins,
full box-corner motion bounds, formerly blocked side reaches accepted by IK and
simulated motor execution, rearward targets refused without writes, a backward
sweep between clear endpoints refused, and escape from a hand-posed rearward start.

```powershell
& (Get-Content ".backend-python.txt") "back end/TestBasketSurface.py"
```

Surface tests cover exact contact, full solid containment, arbitrary rotated
boxes against an independent analytic test, the open tray and its floor, both
arm mounts, complete surface-tree coverage, and a pose blocked by the previous
cart box. Python and browser decisions, including the rearward boundary, are compared in
TestBasketCollision.mjs. The source generator validates closed solid components
and records the source asset hashes. Calibration archives retain the
surface JSON and both collision implementations for offline replay.

The IK candidate selection is advertised as all-poses-v2 in health and state.
The UI requires that solver version before sending motion. Zero-width joint
ranges remain fixed during numerical solving, allowing an exactly reachable
fixed pose while still rejecting a target that needs those joints to move.

```powershell
& (Get-Content ".backend-python.txt") "back end/TestCartesianReachability.py"
```

Alternate clear configurations, fixed joints, the rim-entry planner's checked
descent, a valid inside-tray target reached and completed with independent
simulated motor speeds, and rejection of contact/impossible targets without
motor writes. Browser regressions also exercise the green angle guide,
fixed-position wrist scrolling and sending an inside-tray target without snapping
it elsewhere.

```powershell
& (Get-Content ".backend-python.txt") "back end/TestRobotBodyCollision.py"
```

CAD placement for both arm mounts, contact on every mast/head part, solid
containment, nearby usable space, a clear tip with body contact elsewhere on the
arm, blocked Send without writes, and a mast-crossing sweep replaced by a checked
route executed with independent simulated motor speeds.

## Speed of the checks themselves

Every certified edge is a box against the basket and body surfaces, so planning
time is collision-check time. Two exact changes to BasketSurface made the same
answers cheaper: the containment ray's tree walk is scalar (its direction is
fixed, so the near face of every slab is the low bound), and the terms that
depend only on that fixed direction are precomputed once per surface; and the
box/triangle axis test writes out the nine edge crosses instead of broadcasting
a general cross product, after a quick accept when a triangle corner is already
inside the box. Verified identical over 40,000 containment queries and 12,000
oriented box queries against the previous implementation, basket and body, at
both arm mounts: zero disagreements. Box overlap is 2.9x faster on that mix,
containment 5.2x; a fixed set of 120 edge certifications runs 1.6x faster, and
the planner gets through roughly twice the gates in the same seconds.

The planning clock is spent differently too. Single-bend repairs certify far
more useful routes per second here than the sampling fallback, so they take
0.55 of the search rather than 0.3, the repair pass before any detour gets
2.5 s, and the last attempt gets 9 s with the preferred arm configuration taking
0.6 of what is left instead of an even split that was too short for any of them.

The checks also run side by side. The running server starts check workers
(back end/RouteCheckPool.py: one Python process per four logical cores, at most
8; /health reports route_check_workers), each running BasketCollision's
EdgeReason on the geometry and frame the bridge sends, and used only while their
collision code and data hash the same as the bridge's. Assigning the waypoint
reach asks them ahead for every waypoint's likely tries (its start reach taken
as the waypoints before it leave it when flown past), reads their answers and
checks anything else itself, so it decides exactly as it does alone; the full
check of a blended gripper route's stages goes to them too. A certificate holds
at any budget; a miss at a budget counts only for the same or a smaller one
(EdgeReason splits in a fixed order). On 2026-10-04's 27.3 cm move (opening the
jaws on the way) planning went from 1.3 s to 0.55 s offline, the 4.4 cm descent
from 0.46 to 0.3 s, and the same routes came out for every MeasureRouteSpeed
case. On the live bridge, where the camera and web threads share the planner's
interpreter lock, these checks were about twice as slow and the reach clock ran
out before the wider reaches were tried (0.7 s more flying on the 27 cm move);
in the workers neither happens. Offline imports and the tests check in process
unless they start the pool (TestRouteCheckPool.py).

## Historical measured effect

These measurements predate the replacement of the center-of-mass restriction
with the rearward workspace limit. They are not measurements of the current guard.

Offline, on the same 40 random start and goal poses (both clear, center of mass
over the cart), executed on the servo model: the previous planner found routes
for 28 and took 5.3 s to plan on average; 4 of those routes swung the center of
mass past the cart. This planner found all 40, planned in 1.3 s on average, kept
the center of mass over the cart throughout, and on the 28 cases both completed
cut simulated execution from 13.0 s to 10.5 s at 100% speed (38.9 s to 22.3 s
at 30%). The same 40 routes also completed at 100% without a trip on two of the
harsh servo models above, with 3 and 4 frames of latency (11.0 s and 11.7 s on
average). Against the first fly-by version, which tripped on the hardware, the
overshoot protections cost about 3 s of the 10.5 s at 100% and about 2 s at
30%. These are simulator measurements; on the hardware the table target took
15.6 s and the return Home 17.1 s at 100%.

Re-measured on 40 fresh cases (seed 11) after the reach budget, the shorter
final ramp and the faster checks: 38 of 40 planned (the two misses are random
poses needing a 40-waypoint contortion across the robot), mean execution 10.8 s
to 8.5 s and median 11.4 s to 8.3 s at 100%, with mean planning unchanged at
about 1.6 s. The harsh servo model with 4 frames of latency, stiction and
reduced caps completed all 38 without a trip (10.9 s to 9.9 s). Raising the
waypoint reach itself was tried and rejected: at 48 and 64 steps more stages
fail to certify and go tight, and execution got worse (9.8 s and 10.5 s).
