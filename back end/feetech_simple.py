"""Feetech STS3215 serial bus wrapper for the SO-101 arm GUI."""

from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import Any

import scservo_sdk as scs


# ---------------------------------------------------------------------------
# Enums
# ---------------------------------------------------------------------------

class MotorNormMode(str, Enum):
    RANGE_0_100   = "range_0_100"
    RANGE_M100_100 = "range_m100_100"
    DEGREES       = "degrees"


class OperatingMode(Enum):
    POSITION = 0
    VELOCITY = 1
    PWM = 2
    STEP = 3


class TorqueMode(Enum):
    ENABLED  = 1
    DISABLED = 0


class PortLostError(ConnectionError):
    """The serial port itself failed: the USB adapter dropped off the bus, was unplugged or reset.

    Windows then fails every call on the old handle (ClearCommError, ReadFile, WriteFile:
    "Access is denied") for good, even after the adapter comes back; only a fresh handle works
    (FeetechMotorsBus.Reopen). A ConnectionError, so every existing handler still catches it.
    """


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------

@dataclass
class Motor:
    id: int
    model: str
    norm_mode: MotorNormMode


@dataclass
class MotorCalibration:
    id: int
    drive_mode: int
    homing_offset: int
    range_min: int
    range_max: int


# ---------------------------------------------------------------------------
# Hardware tables
# ---------------------------------------------------------------------------

CONTROL_TABLE: dict[str, tuple[int, int]] = {
    "Model_Number":          (3,  2),
    "ID":                    (5,  1),
    "Return_Delay_Time":     (7,  1),
    "Min_Position_Limit":    (9,  2),
    "Max_Position_Limit":    (11, 2),
    "Operating_Mode":        (33, 1),
    "Torque_Enable":         (40, 1),
    "Acceleration":          (41, 1),
    "Goal_Position":         (42, 2),
    "Goal_Time":             (44, 2),
    "Goal_Speed":            (46, 2),
    "Torque_Limit":          (48, 2),
    "Lock":                  (55, 1),
    "Present_Position":      (56, 2),
    "Present_Speed":         (58, 2),
    "Present_Load":          (60, 2),
    "Present_Voltage":       (62, 1),
    "Present_Temperature":   (63, 1),
    "Hardware_Error_Status": (65, 1),
    "Moving":                (66, 1),
    "Present_Current":       (69, 2),
    "Maximum_Velocity_Limit":(84, 1),
    "Maximum_Acceleration":  (85, 1),
}

MODEL_NUMBER: dict[str, int] = {"sts3215": 777}
MODEL_RESOLUTION: dict[str, int] = {"sts3215": 4096}

# Registers that use signed-magnitude encoding: the sign bit is bit 15, except where SIGN_BIT names another.
SIGNED_MAGNITUDE: set[str] = {
    "Goal_Position",
    "Present_Position",
    "Goal_Speed",
    "Present_Speed",
    "Present_Load",
    "Present_Current",  # bit 15, as the vendor SDK's SMS_STS::ReadCurrent decodes it (SMS_STS_DIRECTION_BIT_POS)
}
# Present_Load is a duty magnitude 0-1000 with its direction in bit 10 (STS3215 memory table). Decoded with bit 15 it read
# +1224 on the 2026-09-28 elbow overload record (captures/2026-09-28/selfcal/runs/20260928_135935_f6fec78880d2): that is
# bit 10 and 200, the 20 % output an overload allows, so -200 (docs/HandGuide.md 2.1).
SIGN_BIT: dict[str, int] = {"Present_Load": 10}

# ---------------------------------------------------------------------------
# Motion profile limits (hard caps applied on every write)
# ---------------------------------------------------------------------------

_FRACTION = 0.20
_FULL_ACC  = 255
_FULL_SPEED = 1000
_FULL_VEL   = 255
_FULL_TORQUE = 1000

HARD_ACCELERATION        = max(1, round(_FULL_ACC   * _FRACTION))  # 51
# Allow the expanded commanded-motion range without changing normal Hold profiles.
HARD_GOAL_SPEED          = 1750  # Top 100% route ceiling (bridge: 70 x MotionSpeedMaximumMultiplier 25).
# Lower 100% ceilings, steps/s. Each is the bridge's route ceiling for that joint, so a route
# command is never clipped here. Provisional, from the ST3215 spec at the ~8.9 V the servos
# report (Cad/MotionSpeed.md): elbow and wrist flex sit about 15% under the speed they keep
# at their worst static load; shoulder lift stays at the 1050 already run on the arm; the
# gripper stays at 1050 because it closes onto objects.
JOINT_GOAL_SPEED_LIMITS  = {"shoulder_lift": 1050, "elbow_flex": 1250, "wrist_flex": 1650, "gripper": 1050}
HARD_MAX_ACCELERATION    = max(1, round(_FULL_ACC   * _FRACTION))  # 51
HARD_MAX_VELOCITY_LIMIT  = max(1, round(_FULL_VEL   * _FRACTION))  # 51

NORMAL_ACCELERATION = HARD_ACCELERATION
NORMAL_GOAL_SPEED   = max(1, round(_FULL_SPEED * _FRACTION))  # Preserve the 200 Hold profile
NORMAL_TORQUE_LIMIT = _FULL_TORQUE

SHOULDER_LIFT_MAX_VELOCITY_LIMIT = min(_FULL_VEL, HARD_MAX_VELOCITY_LIMIT * 2)
SHOULDER_LIFT_TORQUE_LIMIT = _FULL_TORQUE

_HOME_DIV = 3
HOME_ACCELERATION = max(1, NORMAL_ACCELERATION // _HOME_DIV)   # 17
HOME_GOAL_SPEED   = max(1, NORMAL_GOAL_SPEED   // _HOME_DIV)   # 66

HOME_TOLERANCE_RAW = 20
HOME_TIMEOUT_S     = 12.0
HOME_POLL_S        = 0.05


# ---------------------------------------------------------------------------
# Bus class
# ---------------------------------------------------------------------------

class FeetechMotorsBus:
    """Low-level wrapper around the Feetech SCS protocol for the SO-101 arm."""

    model_resolution_table = MODEL_RESOLUTION

    def __init__(self, port: str, motors: dict[str, Motor], baudrate: int = 1_000_000) -> None:
        self.port    = port
        self.motors  = motors
        self.baudrate = baudrate
        self.calibration: dict[str, MotorCalibration] = {}
        self.PacketFaults: dict[str, int] = {}
        # Transaction timing. A silent servo fails at the SDK's ~34 ms packet timeout; a
        # failure that took far longer means this thread was starved, not the servo.
        self.BusStats: dict[str, Any] = {"attempts": 0, "failures": 0, "slow_ok": 0, "max_ok_ms": 0.0,
                                         "port_lost": 0, "reopens": 0, "recent_failures": deque(maxlen=20)}
        self._port_handler   = scs.PortHandler(port)
        self._packet_handler = scs.PacketHandler(0)
        # The OS error once the port itself failed (PortLostError), until Reopen() succeeds.
        # Meanwhile every transaction fails at once instead of touching the dead handle.
        self.PortLost: str | None = None
        self.PortLostAt: float | None = None  # time.monotonic() of the failure
        self.ReopenError: str | None = None
        # One transaction at a time. The bridge already calls under its own lock; this makes
        # clearing the SDK's busy flag after a transaction (_Transact) safe on its own.
        self._Transaction = threading.Lock()

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    @property
    def is_connected(self) -> bool:
        # A lost port stays attached until disconnect(): the bridge reopens it (Reopen), and an
        # outage must not read as the user having disconnected the arm.
        return bool(self.PortLost) or bool(getattr(self._port_handler, "is_open", False))

    def PortLostText(self) -> str:
        return f"USB serial link to the arm on {self.port} dropped ({self.PortLost})"

    def Reopen(self) -> bool:
        """Open a fresh handle on the same port after PortLost. True once it is open.

        Opening sends nothing to the servos. Their state is unknown until read again: the
        bridge reads every joint before anything that commands motion.
        """
        with self._Transaction:
            if not self.PortLost:
                return True
            try:
                self._port_handler.closePort()
            except Exception:
                pass
            Handler = scs.PortHandler(self.port)
            try:
                if not Handler.setBaudRate(self.baudrate):
                    raise ConnectionError(f"cannot set {self.baudrate} baud")
            except Exception as Exc:
                self.ReopenError = str(Exc) or type(Exc).__name__
                try:
                    if getattr(Handler, "ser", None) is not None:
                        Handler.ser.close()
                except Exception:
                    pass
                return False
            self._port_handler = Handler
            self.PortLost = self.PortLostAt = self.ReopenError = None
            self.BusStats["reopens"] += 1
            return True

    def connect(self) -> None:
        if not self._port_handler.openPort():
            raise ConnectionError(f"Cannot open {self.port}")
        if not self._port_handler.setBaudRate(self.baudrate):
            self._port_handler.closePort()
            raise ConnectionError(f"Cannot set {self.port} to {self.baudrate} baud")
        try:
            self._verify_motors()
        except Exception:
            self._port_handler.closePort()
            raise

    def disconnect(self, disable_torque: bool = True) -> None:
        if disable_torque and self.is_connected and not self.PortLost:
            try:
                self.disable_torque(num_retry=1)
            except Exception:
                pass
        self.PortLost = self.PortLostAt = None
        self._port_handler.closePort()

    def _verify_motors(self) -> None:
        missing, wrong = [], []
        for name, motor in self.motors.items():
            try:
                found = int(self.read("Model_Number", name, normalize=False, num_retry=1))
            except Exception:
                missing.append(f"{name} (id {motor.id})")
                continue
            expected = MODEL_NUMBER[motor.model]
            if found != expected:
                wrong.append(f"{name}: expected {expected}, got {found}")
        if missing or wrong:
            parts: list[str] = []
            if missing:
                parts.append("Missing: " + ", ".join(missing))
            if wrong:
                parts.append("Wrong model: " + "; ".join(wrong))
            raise RuntimeError("\n".join(parts))

    # ------------------------------------------------------------------
    # Internal read / write helpers
    # ------------------------------------------------------------------

    def _addr_len(self, name: str) -> tuple[int, int]:
        try:
            return CONTROL_TABLE[name]
        except KeyError:
            raise KeyError(f"Unknown register: {name}")

    def _encode(self, reg: str, value: int) -> int:
        if reg in SIGNED_MAGNITUDE:
            return scs.SCS_TOSCS(int(value), 15)
        return int(value)

    def _decode(self, reg: str, value: int) -> int:
        if reg in SIGNED_MAGNITUDE:
            return scs.SCS_TOHOST(int(value), SIGN_BIT.get(reg, 15))
        return int(value)

    def _goal_speed_limit_for(self, name: str) -> int:
        return JOINT_GOAL_SPEED_LIMITS.get(name, HARD_GOAL_SPEED)

    def _profile_goal_speed_for(self, name: str, goal_speed: int) -> int:
        if name == "shoulder_lift":
            return min(_FULL_SPEED, int(goal_speed) * 2)
        return int(goal_speed)

    def _max_velocity_limit_for(self, name: str) -> int:
        if name == "shoulder_lift":
            return SHOULDER_LIFT_MAX_VELOCITY_LIMIT
        return HARD_MAX_VELOCITY_LIMIT

    def _torque_limit_for(self, name: str) -> int:
        if name == "shoulder_lift":
            return SHOULDER_LIFT_TORQUE_LIMIT
        return NORMAL_TORQUE_LIMIT

    def _clamp_write(self, reg: str, value: int, name: str = "") -> int:
        v = int(value)
        if reg == "Goal_Speed":
            sign = -1 if v < 0 else 1
            return sign * min(abs(v), self._goal_speed_limit_for(name))
        if reg == "Torque_Limit":
            return max(0, min(_FULL_TORQUE, v))
        if reg == "Acceleration":
            return max(0, min(HARD_ACCELERATION, v))
        if reg == "Maximum_Acceleration":
            return max(0, min(HARD_MAX_ACCELERATION, v))
        if reg == "Maximum_Velocity_Limit":
            return max(0, min(self._max_velocity_limit_for(name), v))
        return v

    def _raw_read(self, motor_id: int, addr: int, length: int) -> tuple[int, int, int]:
        if length not in (1, 2):
            raise ValueError(f"Unsupported length {length}")
        Read = self._packet_handler.read1ByteTxRx if length == 1 else self._packet_handler.read2ByteTxRx
        return self._Transact("read", motor_id, addr, Read, motor_id, addr)

    def _raw_write(self, motor_id: int, addr: int, length: int, value: int) -> tuple[int, int]:
        if length not in (1, 2):
            raise ValueError(f"Unsupported length {length}")
        Write = self._packet_handler.write1ByteTxRx if length == 1 else self._packet_handler.write2ByteTxRx
        return self._Transact("write", motor_id, addr, Write, motor_id, addr, value)

    def _Transact(self, Kind: str, MotorId: int, Address: int, Call, *Arguments):
        """One SDK transaction that never leaves the port marked busy.

        The SDK sets PortHandler.is_using when a packet goes out and clears it only on its
        normal returns. On 2026-10-01 at 19:07 the USB adapter dropped off the bus mid-read,
        pyserial raised, the flag stayed set, and every later call failed "Port is in use!"
        without touching the port for 15 minutes, emergency torque-off included. Transactions
        are serialised here, so clearing the flag afterwards never releases one in flight.

        An OS error from the port itself (pyserial's SerialException is an OSError) marks the
        port lost and raises PortLostError; until Reopen() every call fails at once.
        """
        if self.PortLost:
            raise PortLostError(self.PortLostText())
        with self._Transaction:
            Started = time.perf_counter()
            try:
                Result = Call(self._port_handler, *Arguments)
            except OSError as Exc:
                self._mark_port_lost(Exc, Kind, MotorId, Address, Started)
                raise PortLostError(self.PortLostText()) from Exc
            finally:
                self._port_handler.is_using = False
        self._count_attempt(Kind, MotorId, Address, Result[-2], Started)
        return Result

    def _mark_port_lost(self, Error: Exception, Kind: str, MotorId: int, Address: int, Started: float) -> None:
        if not self.PortLost:
            self.PortLostAt = time.monotonic()
            self.BusStats["port_lost"] += 1
        self.PortLost = str(Error) or type(Error).__name__
        self._record_failure(Kind, MotorId, Address, Started, f"port lost: {self.PortLost}")
        # Let go of the dead handle now: closing one never raises, and Windows can only finish
        # removing the old device once nothing holds it open.
        try:
            self._port_handler.closePort()
        except Exception:
            pass

    def _count_attempt(self, kind: str, motor_id: int, addr: int, comm: int, started: float) -> None:
        if comm != scs.COMM_SUCCESS:
            self._record_failure(kind, motor_id, addr, started, self._packet_handler.getTxRxResult(comm))
            return
        Ms = (time.perf_counter() - started) * 1000.0
        Stats = self.BusStats
        Stats["attempts"] += 1
        Stats["max_ok_ms"] = max(Stats["max_ok_ms"], Ms)
        Stats["slow_ok"] += Ms > 20.0

    def _record_failure(self, kind: str, motor_id: int, addr: int, started: float, result: str) -> None:
        Ms = (time.perf_counter() - started) * 1000.0
        Stats = self.BusStats
        Stats["attempts"] += 1
        Stats["failures"] += 1
        Name = next((N for N, M in self.motors.items() if M.id == motor_id), str(motor_id))
        Register = next((R for R, (A, _) in CONTROL_TABLE.items() if A == addr), str(addr))
        Stats["recent_failures"].append({"at": round(time.time(), 3), "kind": kind, "motor": Name, "register": Register,
                                         "ms": round(Ms, 1), "result": result})

    # ------------------------------------------------------------------
    # Calibration helpers
    # ------------------------------------------------------------------

    def _calibration_for(self, name: str) -> MotorCalibration:
        cal = self.calibration.get(name)
        if cal is None:
            res = MODEL_RESOLUTION[self.motors[name].model]
            cal = MotorCalibration(id=self.motors[name].id, drive_mode=0,
                                   homing_offset=0, range_min=0, range_max=res - 1)
        if cal.range_min == cal.range_max:
            res = MODEL_RESOLUTION[self.motors[name].model]
            cal = MotorCalibration(id=cal.id, drive_mode=cal.drive_mode,
                                   homing_offset=cal.homing_offset,
                                   range_min=0, range_max=res - 1)
        return cal

    # ------------------------------------------------------------------
    # Normalization
    # ------------------------------------------------------------------

    def normalize_position(self, name: str, raw: int) -> float:
        motor = self.motors[name]
        cal   = self._calibration_for(name)
        res   = MODEL_RESOLUTION[motor.model]
        raw   = int(raw) % res
        lo, hi = sorted((cal.range_min, cal.range_max))
        bounded = max(lo, min(hi, raw))
        span    = max(1, hi - lo)

        if motor.norm_mode is MotorNormMode.RANGE_0_100:
            return (bounded - lo) * 100.0 / span
        if motor.norm_mode is MotorNormMode.RANGE_M100_100:
            return ((bounded - lo) * 200.0 / span) - 100.0
        if motor.norm_mode is MotorNormMode.DEGREES:
            mid = (lo + hi) / 2.0
            return (bounded - mid) * 360.0 / max(1, res - 1)
        raise NotImplementedError(motor.norm_mode)

    def unnormalize_position(self, name: str, value: float) -> int:
        motor = self.motors[name]
        cal   = self._calibration_for(name)
        lo, hi = sorted((cal.range_min, cal.range_max))
        span   = max(1, hi - lo)
        res    = MODEL_RESOLUTION[motor.model]

        if motor.norm_mode is MotorNormMode.RANGE_0_100:
            raw = lo + max(0.0, min(100.0, float(value))) * span / 100.0
        elif motor.norm_mode is MotorNormMode.RANGE_M100_100:
            raw = lo + (max(-100.0, min(100.0, float(value))) + 100.0) * span / 200.0
        elif motor.norm_mode is MotorNormMode.DEGREES:
            mid = (lo + hi) / 2.0
            raw = mid + float(value) * max(1, res - 1) / 360.0
        else:
            raise NotImplementedError(motor.norm_mode)

        return max(lo, min(hi, round(raw)))

    # ------------------------------------------------------------------
    # Public read / write
    # ------------------------------------------------------------------

    def read(self, reg: str, name: str, *, normalize: bool = False,
             num_retry: int = 0) -> int | float:
        addr, length = self._addr_len(reg)
        motor_id = self.motors[name].id
        value = comm = error = 0
        for _ in range(num_retry + 1):
            value, comm, error = self._raw_read(motor_id, addr, length)
            # A valid status packet can carry a protection fault AND useful
            # telemetry. Discarding it hides the reason a servo unloaded.
            if comm == scs.COMM_SUCCESS:
                self.PacketFaults[name] = int(error)
            Telemetry = reg.startswith("Present_") or reg in ("Hardware_Error_Status", "Torque_Enable")
            if comm == scs.COMM_SUCCESS and (error == 0 or Telemetry):
                decoded = self._decode(reg, int(value))
                if reg == "Hardware_Error_Status":
                    decoded |= int(error)
                if normalize and reg in SIGNED_MAGNITUDE:
                    return self.normalize_position(name, decoded)
                return decoded
        raise ConnectionError(
            f"Read {reg} from {name} failed: "
            f"{self._packet_handler.getTxRxResult(comm)} "
            f"{self._packet_handler.getRxPacketError(error)}"
        )

    def write(self, reg: str, name: str, value: int | float, *,
              normalize: bool = False, num_retry: int = 0) -> None:
        if normalize and reg in SIGNED_MAGNITUDE:
            value = self.unnormalize_position(name, float(value))
        value = self._clamp_write(reg, int(value), name)
        value = self._encode(reg, int(value))
        addr, length = self._addr_len(reg)
        motor_id = self.motors[name].id
        comm = error = 0
        for _ in range(num_retry + 1):
            comm, error = self._raw_write(motor_id, addr, length, int(value))
            if comm == scs.COMM_SUCCESS:
                self.PacketFaults[name] = int(error)
            StopWrite = (reg in ("Torque_Enable", "Goal_Speed") and value == 0) or (reg == "Lock" and value == 0)
            if comm == scs.COMM_SUCCESS and (error == 0 or StopWrite):
                return
        raise ConnectionError(
            f"Write {reg}={value} to {name} failed: "
            f"{self._packet_handler.getTxRxResult(comm)} "
            f"{self._packet_handler.getRxPacketError(error)}"
        )

    def sync_read(self, reg: str, *, normalize: bool = False,
                  num_retry: int = 0) -> dict[str, int | float]:
        return {name: self.read(reg, name, normalize=normalize, num_retry=num_retry)
                for name in self.motors}

    def sync_write(self, reg: str, values: dict[str, int | float], *,
                   normalize: bool = False, num_retry: int = 0) -> None:
        for name, value in values.items():
            self.write(reg, name, value, normalize=normalize, num_retry=num_retry)

    # ------------------------------------------------------------------
    # Calibration
    # ------------------------------------------------------------------

    def burn_position_limits(self, limits: dict[str, tuple[int, int]],
                             num_retry: int = 1) -> None:
        """Write Min/Max_Position_Limit to servo EEPROM for non-wrapped ranges.
        Must be called before enable_torque() — EEPROM writes require Lock=0."""
        for name, (lo, hi) in limits.items():
            if name not in self.motors or lo >= hi:
                continue
            self.write("Min_Position_Limit", name, lo, num_retry=num_retry)
            self.write("Max_Position_Limit", name, hi, num_retry=num_retry)

    def read_calibration(self) -> dict[str, MotorCalibration]:
        result: dict[str, MotorCalibration] = {}
        for name, motor in self.motors.items():
            res = MODEL_RESOLUTION[motor.model]
            try:
                lo = self.read("Min_Position_Limit", name, normalize=False, num_retry=1)
                hi = self.read("Max_Position_Limit", name, normalize=False, num_retry=1)
            except Exception:
                lo, hi = 0, res - 1
            if lo == hi:
                lo, hi = 0, res - 1
            result[name] = MotorCalibration(
                id=motor.id, drive_mode=0, homing_offset=0,
                range_min=int(lo), range_max=int(hi),
            )
        return result

    # ------------------------------------------------------------------
    # Torque
    # ------------------------------------------------------------------

    def SetPositionRangeWhileCoasting(self, Name: str, Minimum: int, Maximum: int) -> None:
        """Change a position window only with verified torque off, then read it back."""
        if self.read("Torque_Enable", Name, num_retry=1) != 0:
            raise ConnectionError(f"Coast {Name} before correcting its position range")
        if not 0 <= Minimum < Maximum < 4096:
            raise ValueError("Invalid servo position window")
        Current = (self.read("Min_Position_Limit", Name, num_retry=1),
                   self.read("Max_Position_Limit", Name, num_retry=1))
        if Current == (Minimum, Maximum):
            return
        self.write("Lock", Name, 0, num_retry=1)
        self.write("Min_Position_Limit", Name, Minimum, num_retry=1)
        self.write("Max_Position_Limit", Name, Maximum, num_retry=1)
        Actual = (self.read("Min_Position_Limit", Name, num_retry=1),
                  self.read("Max_Position_Limit", Name, num_retry=1))
        if Actual != (Minimum, Maximum):
            raise ConnectionError(f"{Name} position range read-back failed: {Actual}")

    def enable_torque(self, num_retry: int = 0) -> None:
        for name in self.motors:
            self.write("Torque_Limit", name, self._torque_limit_for(name), num_retry=num_retry)
            self.write("Torque_Enable", name, TorqueMode.ENABLED.value, num_retry=num_retry)
            self.write("Lock", name, 1, num_retry=num_retry)

    def disable_torque(self, num_retry: int = 0) -> None:
        Errors = []
        for name in self.motors:
            for Register in ("Torque_Enable", "Lock"):
                try:
                    self.write(Register, name, 0, num_retry=num_retry)
                except PortLostError as Error:
                    # No later write can get through the dead port either.
                    raise PortLostError(f"Torque-off not sent from {name} on: {Error}") from Error
                except Exception as Error:
                    Errors.append(f"{name} {Register}: {Error}")
        if Errors:
            raise ConnectionError("Torque-off incomplete: " + "; ".join(Errors))

    # ------------------------------------------------------------------
    # Motion profiles
    # ------------------------------------------------------------------

    def configure_motors(self, acceleration: int = NORMAL_ACCELERATION,
                         goal_speed: int = NORMAL_GOAL_SPEED) -> None:
        for name in self.motors:
            Position = self.read("Present_Position", name, normalize=False, num_retry=1)
            Fault = self.read("Hardware_Error_Status", name, normalize=False, num_retry=1)
            if Fault:
                raise ConnectionError(f"Cannot configure {name}: hardware fault {Fault}")
            Mode = self.read("Operating_Mode", name, num_retry=1)
            if Mode == OperatingMode.VELOCITY.value:
                self.write("Goal_Speed", name, 0, num_retry=1)
            self.write("Goal_Position", name, Position, num_retry=1)
            self.write("Operating_Mode",       name, OperatingMode.POSITION.value, num_retry=1)
            self.write("Return_Delay_Time",    name, 0,                       num_retry=1)
            self.write("Maximum_Velocity_Limit",name, self._max_velocity_limit_for(name), num_retry=1)
            self.write("Maximum_Acceleration", name, HARD_MAX_ACCELERATION,   num_retry=1)
            self.write("Torque_Limit",          name, self._torque_limit_for(name), num_retry=1)
        self.set_motion_profile(acceleration=acceleration, goal_speed=goal_speed, num_retry=1)
        # Writing Goal_Position above engages torque on the STS3215, so the arm can come out
        # of connect already holding. Close the EEPROM lock as Hold does: with it open, every
        # later Operating_Mode change is an EEPROM write whose reply takes 20-40 ms, past the
        # SDK's 34 ms packet timeout, and the motion loop trips on the missing status packet.
        for name in self.motors:
            self.write("Lock", name, 1, num_retry=1)

    def set_motion_profile(self, *, acceleration: int | None = None,
                           goal_speed: int | None = None, num_retry: int = 0) -> None:
        for name in self.motors:
            if acceleration is not None:
                self.write("Acceleration", name, acceleration, num_retry=num_retry)
            if goal_speed is not None:
                speed = self._profile_goal_speed_for(name, int(goal_speed))
                self.write("Goal_Speed", name, speed, num_retry=num_retry)

    def set_home_motion_profile(self, num_retry: int = 0) -> None:
        self.set_motion_profile(acceleration=HOME_ACCELERATION,
                                goal_speed=HOME_GOAL_SPEED, num_retry=num_retry)

    def set_normal_motion_profile(self, num_retry: int = 0) -> None:
        self.set_motion_profile(acceleration=NORMAL_ACCELERATION,
                                goal_speed=NORMAL_GOAL_SPEED, num_retry=num_retry)

    # ------------------------------------------------------------------
    # Home helper
    # ------------------------------------------------------------------

    def wait_for_positions(self, targets: dict[str, int], *,
                           tolerance: int = HOME_TOLERANCE_RAW,
                           timeout_s: float = HOME_TIMEOUT_S,
                           poll_s: float = HOME_POLL_S,
                           num_retry: int = 0) -> bool:
        deadline = time.perf_counter() + timeout_s
        while time.perf_counter() < deadline:
            present = self.sync_read("Present_Position", normalize=False, num_retry=num_retry)
            if all(self._position_error(name, int(present[name]), tgt) <= tolerance
                   for name, tgt in targets.items()):
                return True
            time.sleep(poll_s)
        return False

    def _position_error(self, name: str, present: int, target: int) -> int:
        res   = MODEL_RESOLUTION[self.motors[name].model]
        delta = (present - target) % res
        if delta > res // 2:
            delta -= res
        return abs(delta)
