"""Tests for the bridge's Home path, against a simulated motor bus.

Run:  python -m pytest "back end/test_bridge_home.py" -q
  or: python "back end/test_bridge_home.py"

The bug these cover: Home used a one-shot Goal_Position write while every other
motion path (sliders, IK send) used closed-loop velocity streaming. Tearing down
velocity mode zeroed Goal_Speed and nothing ever restored it, so once the user
had dragged a slider or sent an IK move, Home wrote a position target into a
motor whose profile speed was 0 and the arm did not move.
"""

from __future__ import annotations

import bridge_server as bs
from feetech_simple import NORMAL_GOAL_SPEED, OperatingMode, FeetechMotorsBus


class FakeBus:
    """Records register writes and simulates motors converging on their goal."""

    def __init__(self) -> None:
        self.is_connected = True
        self.registers: dict[tuple[str, str], int] = {}
        self.writes: list[tuple[str, str, int]] = []
        self.position: dict[str, int] = {name: 2048 for name in bs.MOTOR_NAMES}
        self.torque_enabled = False

    # -- FeetechMotorsBus surface used by BridgeController ------------------

    def write(self, reg: str, name: str, value: int, *, normalize=True, num_retry=0) -> None:
        self.registers[(reg, name)] = int(value)
        self.writes.append((reg, name, int(value)))

    def read(self, reg: str, name: str, *, normalize=True, num_retry=0) -> int:
        if reg == "Torque_Enable":
            return int(self.torque_enabled)
        if reg == "Present_Position":
            return self.position[name]
        return self.registers.get((reg, name), 0)

    def configure_motors(self, acceleration: int = 0, goal_speed: int = 0) -> None:
        """Mirrors what BridgeController.connect() does on the real bus."""
        for name in bs.MOTOR_NAMES:
            self.write("Operating_Mode", name, OperatingMode.POSITION.value)
            self.write("Acceleration", name, acceleration or bs.NORMAL_ACCELERATION)
            self.write("Goal_Speed", name, goal_speed or NORMAL_GOAL_SPEED)
            # A real servo powers up holding where it already is, so its
            # Goal_Position register reads back as the present position rather
            # than 0. Seed that, or every joint appears to be commanded to 0.
            self.write("Goal_Position", name, self.position[name])

    SetPositionRangeWhileCoasting = FeetechMotorsBus.SetPositionRangeWhileCoasting

    def enable_torque(self, num_retry: int = 0) -> None:
        self.torque_enabled = True

    def disable_torque(self, num_retry: int = 0) -> None:
        self.torque_enabled = False

    def disconnect(self, disable_torque: bool = True) -> None:
        self.is_connected = False

    # -- simulation --------------------------------------------------------

    def reg(self, name: str, reg_name: str) -> int:
        return self.registers.get((reg_name, name), 0)

    def step(self, name: str, steps_per_tick: int = 40) -> None:
        """Advance one motor one tick according to its current mode."""
        mode = self.reg(name, "Operating_Mode")
        if mode == OperatingMode.VELOCITY.value:
            speed = self.reg(name, "Goal_Speed")
            if speed == 0:
                return
            move = min(abs(speed), steps_per_tick)
            self.position[name] = (self.position[name] + (move if speed > 0 else -move)) % 4096
        elif mode == OperatingMode.POSITION.value:
            # Position-mode speed zero is unlimited, not a stop command.
            Speed = abs(self.reg(name, "Goal_Speed"))
            steps_per_tick = min(steps_per_tick, Speed) if Speed else steps_per_tick
            goal = self.reg(name, "Goal_Position")
            delta = (goal - self.position[name] + 2048) % 4096 - 2048
            move = max(-steps_per_tick, min(steps_per_tick, delta))
            self.position[name] = (self.position[name] + move) % 4096


def make_controller() -> tuple[bs.BridgeController, FakeBus]:
    controller = bs.BridgeController()
    bus = FakeBus()
    controller.bus = bus
    controller.port = "FAKE"
    controller.torque_enabled = True
    bus.torque_enabled = True
    # These tests isolate Home/slider control on arbitrary raw fixtures. Physical
    # path policy is exercised separately with real geometry in TestSafety.py.
    controller.ValidateMotionPath = lambda Targets, Calibration=False, **Options: None
    # These deliberately hand-pose impossible arm configurations to exercise
    # motor mode/hold behavior. Physical planning has its own unmocked suite.
    controller.PlanMotionRoute = lambda Targets, Calibration=False, **Options: [
        {Name: Targets.get(Name, Servo.present_raw) for Name, Servo in controller.servos.items()}]
    for name, servo in controller.servos.items():
        bus.position[name] = servo.present_raw
    bus.configure_motors()
    return controller, bus


def run_frames(controller: bs.BridgeController, bus: FakeBus, frames: int = 400) -> None:
    """Drive the same refresh loop the WebSocket endpoint runs at 30 Hz."""
    for _ in range(frames):
        for name in bs.MOTOR_NAMES:
            bus.step(name)
        controller.refresh()


def hold_at(controller: bs.BridgeController, bus: FakeBus, name: str, raw: int) -> int:
    """Place a joint at raw and have the servo genuinely holding it there."""
    controller.servos[name].present_raw = raw
    bus.position[name] = raw
    bus.write("Goal_Position", name, raw)
    return raw


def displace(controller: bs.BridgeController, bus: FakeBus) -> None:
    """Park every home-driven joint but the gripper well away from home, inside its limits, holding there.

    The gripper stays at Home: this bus moves a velocity command's speed in steps every tick (30x a real servo), so
    the gripper-only stage Home closes the jaws on would overshoot the route's allowance here. Home's gripper is
    tested on the servo model with the real planner (TestHomeRollAndGripper.py).
    """
    for name in bs.HOME_MOTOR_NAMES:
        if name == bs.GRIPPER_NAME:
            continue
        servo = controller.servos[name]
        home_fraction = controller._raw_to_track_fraction(servo, controller.home_preset[name])
        raw = controller._fraction_to_raw(servo, 0.15 if home_fraction > 0.5 else 0.85)
        servo.present_raw = raw
        servo.target_raw = raw
        bus.position[name] = raw
        bus.write("Goal_Position", name, raw)


def home_targets(controller: bs.BridgeController) -> dict[str, int]:
    return {
        name: controller._clamp_raw(controller.servos[name], controller.home_preset[name])
        for name in bs.HOME_MOTOR_NAMES
    }


def assert_at_home(controller: bs.BridgeController, bus: FakeBus, tol: int = 40) -> None:
    for name, goal in home_targets(controller).items():
        error = abs((bus.position[name] - goal + 2048) % 4096 - 2048)
        assert error <= tol, f"{name} stopped {error} steps from home (at {bus.position[name]}, want {goal})"


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_home_moves_arm_from_cold_start():
    """Baseline: Home works on a freshly connected arm."""
    controller, bus = make_controller()
    displace(controller, bus)

    ok, _ = controller.go_home()
    assert ok
    run_frames(controller, bus)
    assert_at_home(controller, bus)


def test_home_works_after_a_slider_drag():
    """The regression: a completed slider move used to leave Goal_Speed at 0,
    which killed every later Home press."""
    controller, bus = make_controller()

    # Drag elbow_flex somewhere and let the stream drive it to completion.
    servo = controller.servos["elbow_flex"]
    away = controller._clamp_raw(servo, (controller.home_preset["elbow_flex"] - 700) % 4096)
    ok, _ = controller.move_servo("elbow_flex", away)
    assert ok
    run_frames(controller, bus)
    assert servo.command_raw is None, "slider stream should have settled"

    # The state the bug needed: parked in position mode. Goal_Speed used to be
    # left at 0 here, which is what killed the following Home.
    assert bus.reg("elbow_flex", "Operating_Mode") == OperatingMode.POSITION.value
    assert bus.reg("elbow_flex", "Goal_Speed") != 0

    ok, _ = controller.go_home()
    assert ok
    run_frames(controller, bus)
    assert_at_home(controller, bus)


def test_home_works_after_an_ik_send():
    """IK send also parks the joints in position mode at speed 0."""
    controller, bus = make_controller()
    for name in ("shoulder_lift", "elbow_flex", "wrist_flex"):
        servo = controller.servos[name]
        away = controller._clamp_raw(servo, (controller.home_preset[name] - 500) % 4096)
        controller._send_slider_target_locked(servo, away, num_retry=0)
    run_frames(controller, bus)

    ok, _ = controller.go_home()
    assert ok
    run_frames(controller, bus)
    assert_at_home(controller, bus)


def test_home_repeats():
    """Home must keep working when pressed over and over."""
    controller, bus = make_controller()
    for round_index in range(3):
        for name in bs.HOME_MOTOR_NAMES:
            servo = controller.servos[name]
            away = controller._clamp_raw(servo, (bus.position[name] - 400) % 4096)
            controller._send_slider_target_locked(servo, away, num_retry=0)
        run_frames(controller, bus)

        ok, _ = controller.go_home()
        assert ok, f"round {round_index}"
        run_frames(controller, bus)
        assert_at_home(controller, bus)


def test_home_enables_torque_itself():
    """Torque is off after every connect; Home should turn it on, not no-op."""
    controller, bus = make_controller()
    controller.torque_enabled = False
    bus.torque_enabled = False
    displace(controller, bus)

    ok, _ = controller.go_home()
    assert ok
    assert controller.torque_enabled and bus.torque_enabled
    run_frames(controller, bus)
    assert_at_home(controller, bus)


def test_home_brings_wrist_roll_to_its_home_value():
    """Until 2026-10-12 Home left the roll where it was; it now goes Home with the other joints."""
    controller, bus = make_controller()
    servo = controller.servos["wrist_roll"]
    goal = controller._clamp_raw(servo, controller.home_preset["wrist_roll"])
    hold_at(controller, bus, "wrist_roll", controller._clamp_raw(servo, (goal + 300) % 4096))

    ok, _ = controller.go_home()
    assert ok
    assert controller.MotionRoute["Path"][-1]["wrist_roll"] == goal
    run_frames(controller, bus)
    error = abs((bus.position["wrist_roll"] - goal + 2048) % 4096 - 2048)
    assert error <= bs.SLIDER_GOAL_TOLERANCE_RAW, f"wrist roll stopped {error} steps from Home"
    assert servo.command_raw is None, "the roll parks at Home"


def test_homing_speed_is_capped_below_slider_speed():
    controller, _ = make_controller()
    servo = controller.servos["elbow_flex"]
    servo.present_raw = controller._clamp_raw(servo, (servo.target_raw - 900) % 4096)

    slider_speed = abs(controller._slider_velocity_speed(servo))
    servo.homing = True
    homing_speed = abs(controller._slider_velocity_speed(servo))

    assert homing_speed < slider_speed
    assert homing_speed <= bs.HOME_VELOCITY_MAX_SPEED


def test_homing_flag_clears_on_arrival():
    controller, bus = make_controller()
    displace(controller, bus)

    controller.go_home()
    assert any(controller.servos[name].homing for name in bs.HOME_MOTOR_NAMES)
    run_frames(controller, bus)
    assert not any(servo.homing for servo in controller.servos.values())


def test_home_recovers_an_arm_posed_outside_its_limits():
    """Hand-posing with torque off can leave a joint past its GUI limit.

    Measured positions used to be clamped into the limit range, so the bridge
    read the joint as sitting exactly on the boundary, computed zero error and
    refused to home it.
    """
    controller, bus = make_controller()
    servo = controller.servos["elbow_flex"]
    outside = (servo.range_max + 600) % 4096
    assert not controller._contains_raw(servo, outside), "test needs an out-of-range pose"
    bus.position["elbow_flex"] = outside

    controller.refresh()
    assert servo.present_raw == outside, "measured position must not be clamped"

    ok, _ = controller.go_home()
    assert ok
    run_frames(controller, bus)
    assert_at_home(controller, bus)
    assert controller._contains_raw(servo, bus.position["elbow_flex"]), "must end inside limits"


def test_every_joint_parks_instead_of_hunting():
    """No joint may sit in velocity mode forever.

    A zero goal tolerance is unreachable when the controller has a floor speed,
    so the joint oscillates around its target and never releases the stream.
    """
    controller, bus = make_controller()
    displace(controller, bus)
    controller.go_home()
    run_frames(controller, bus)

    still_streaming = [name for name, s in controller.servos.items() if s.command_raw is not None]
    assert not still_streaming, f"never parked: {still_streaming}"

    for name in bs.MOTOR_NAMES:
        tolerance = controller._slider_goal_tolerance_raw(name)
        assert tolerance >= bs.SLIDER_VELOCITY_MIN_SPEED / 4, (
            f"{name} tolerance {tolerance} is too tight for floor speed "
            f"{bs.SLIDER_VELOCITY_MIN_SPEED} and will hunt"
        )


def test_parked_joints_hold_with_an_armed_profile():
    """A parked joint must be able to drive back to its goal.

    Parking left Goal_Speed at 0, and a position-mode servo with no profile
    speed does not chase Goal_Position — so Hold felt soft and the arm could be
    pushed off target by hand.
    """
    controller, bus = make_controller()
    displace(controller, bus)
    controller.go_home()
    run_frames(controller, bus)

    for name in bs.MOTOR_NAMES:
        if bus.reg(name, "Operating_Mode") != OperatingMode.POSITION.value:
            continue
        assert bus.reg(name, "Goal_Speed") != 0, f"{name} parked with a dead motion profile"

    # Shove a parked joint off its goal; the servo's own loop must pull it back.
    goal = bus.reg("elbow_flex", "Goal_Position")
    bus.position["elbow_flex"] = (goal + 150) % 4096
    for _ in range(60):
        bus.step("elbow_flex")
    drift = abs((bus.position["elbow_flex"] - goal + 2048) % 4096 - 2048)
    assert drift <= 40, f"pushed joint did not recover, sitting {drift} steps off"


def test_hold_locks_where_the_arm_is_not_a_stale_goal():
    """Enabling torque must not snap the arm to a leftover Goal_Position.

    Entering velocity mode parks Goal_Position at raw 0, so energising against
    the stale value would drive the joint hard into its lower endstop.
    """
    controller, bus = make_controller()

    # Stream a joint (which writes Goal_Position 0), then coast and hand-pose it.
    controller._send_slider_target_locked(controller.servos["elbow_flex"], 1500, num_retry=0)
    controller.toggle_torque()
    assert not controller.torque_enabled
    posed = 2100
    bus.position["elbow_flex"] = posed

    ok, _ = controller.toggle_torque()
    assert ok and controller.torque_enabled

    assert bus.reg("elbow_flex", "Goal_Position") == posed, "Hold must target the live position"
    assert bus.reg("elbow_flex", "Goal_Speed") != 0, "Hold must arm the motion profile"
    assert bus.reg("elbow_flex", "Operating_Mode") == OperatingMode.POSITION.value

    run_frames(controller, bus)
    assert abs(bus.position["elbow_flex"] - posed) <= 40, "arm moved when Hold was pressed"


def test_hold_arms_every_joint():
    controller, bus = make_controller()
    controller.toggle_torque()
    controller.toggle_torque()
    for name in bs.MOTOR_NAMES:
        assert bus.reg(name, "Goal_Speed") != 0, f"{name} left with a dead profile after Hold"


def test_home_preview_without_arm():
    """No hardware: Home should still move the on-screen model, the wrist roll and the gripper included."""
    controller = bs.BridgeController()
    controller.bus = None
    for name in ("wrist_roll", "gripper"):
        servo = controller.servos[name]
        servo.target_raw = controller._fraction_to_raw(servo, 0.6)
    ok, text = controller.go_home()
    assert ok and text == "Home loaded in preview"
    for name, goal in home_targets(controller).items():
        assert controller.servos[name].target_raw == goal


def test_hold_redrives_gravity_sag():
    """Hold must actively drive back a joint that droops after torque is on.

    Regression: Hold parked one-shot goals with no active stream, so the
    shoulder lift slowly bent down under gravity and nothing corrected it.
    """
    controller, bus = make_controller()
    controller.torque_enabled = False
    bus.torque_enabled = False
    ok, _ = controller.toggle_torque()
    assert ok and controller.torque_enabled
    assert all(S.Holding for S in controller.servos.values()), "Hold must latch on every joint"

    Target = controller.servos["shoulder_lift"].target_raw
    Sagged = (Target - 120) % 4096
    bus.position["shoulder_lift"] = Sagged
    # Simulate the goal being lost with the sag (inrush dip): the fake position
    # loop auto-converges to Goal_Position on its own, so without clobbering the
    # goal this test would pass even if the bridge never re-issued anything.
    bus.write("Goal_Position", "shoulder_lift", Sagged)
    WritesBefore = len(bus.writes)
    run_frames(controller, bus, frames=200)
    FreshWrites = [W for W in bus.writes[WritesBefore:] if W[1] == "shoulder_lift" and W[0] in ("Goal_Speed", "Goal_Position")]
    assert FreshWrites, "Hold issued no fresh commands after the sag"
    Drift = abs((bus.position["shoulder_lift"] - Target + 2048) % 4096 - 2048)
    assert Drift <= 40, f"Hold did not drive back the sag, sitting {Drift} steps off"


if __name__ == "__main__":
    import sys, traceback

    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    failures = 0
    for test in tests:
        try:
            test()
            print(f"PASS  {test.__name__}")
        except Exception:
            failures += 1
            print(f"FAIL  {test.__name__}")
            traceback.print_exc()
    print(f"\n{len(tests) - failures}/{len(tests)} passed")
    sys.exit(1 if failures else 0)
