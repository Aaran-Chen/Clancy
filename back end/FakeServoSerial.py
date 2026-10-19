"""A pyserial stand-in that answers like the SO-101's six STS3215 servos, for offline tests.

It sits under the REAL scservo_sdk PortHandler/PacketHandler and the REAL FeetechMotorsBus, so
the SDK's own busy flag (is_using) and the driver's error handling run exactly as on the arm.
Installed with Installed(Device), it replaces serial.Serial for the duration; any port name
other than the fake ones (FakePorts) raises AssertionError, so a test can never open a real
serial port.

The USB adapter can drop off the bus the way the CH343 did on 2026-10-01 at 19:07:
- Remove(): Windows surprise removal. Every call on a handle opened before it raises the
  SerialException pyserial raises on a dead handle ("ClearCommError failed (PermissionError(13,
  'Access is denied.', None, 5))"); that handle never recovers, even after Arrive(). Only a new
  open works, and only once the adapter is back.
- RemoveAfterRequest = (Id, Address): the adapter drops just after that request went out, so
  the error surfaces while the SDK reads the reply (mid-read).
- RemoveOnRequest = (Id, Address): the adapter drops while that request is being written, so
  the write itself fails (mid-write) and the servo never receives it.

Servo behaviour modelled: goal writes (position or speed) energise torque, as on the STS3215;
Step() moves joints in velocity mode at their goal speed and in position mode toward their goal.
SYNC_READ (0x82: one status packet per listed servo, in order) and SYNC_WRITE (0x83: no reply) as the hand-guiding loop
uses them (GuideBus.py); Silent ids answer nothing; the position loop's settings (P 32, D 32, I 0, punch 16, dead zones
1/1, firmware 3.10) as the arm's servos report them; SetLoad/SetVoltage write the telemetry a test wants the bridge to
read (Present_Load with its sign in bit 10, as the servo encodes it); Restart(Id) is a brown-out: torque off, Lock 0,
Torque_Limit 1000; OnStep (device, seconds) runs after each Step, for a test's own physics.
"""
from __future__ import annotations

import contextlib
from typing import Callable, Iterator
from unittest.mock import patch

import serial

FakePort = "FAKE-COM4"
# Names the adapter can come back under (Arrive(As=...)); no other name ever opens.
FakePorts = (FakePort, "FAKE-COM5")
# pyserial's text for a call on a surprise-removed CH343 handle (bridge_events.jsonl, 2026-10-01 19:07:22).
DeadHandleText = "ClearCommError failed (PermissionError(13, 'Access is denied.', None, 5))"
OperatingMode, TorqueEnable, GoalPosition, GoalSpeed, PresentPosition = 33, 40, 42, 46, 56
TorqueLimit, Lock, PresentLoad, PresentVoltage, PresentTemperature = 48, 55, 60, 62, 63
SyncRead, SyncWrite, Broadcast = 0x82, 0x83, 0xFE


def SignedMagnitude(Value: int) -> int:
    return -(Value & 0x7FFF) if Value & 0x8000 else Value


class FakeServoDevice:
    """The servo bus behind the adapter: one 256-byte control table per servo id."""

    def __init__(Self, Positions: dict[int, int], TorqueOn: bool = False) -> None:
        Self.Present = True
        Self.Port = FakePort
        Self.Generation = 0
        Self.Tables: dict[int, bytearray] = {}
        for Id, Position in Positions.items():
            Table = bytearray(256)
            Table[3:5] = (777).to_bytes(2, "little")       # Model_Number: sts3215
            Table[5] = Id
            Table[9:11] = (0).to_bytes(2, "little")        # Min_Position_Limit
            Table[11:13] = (4095).to_bytes(2, "little")    # Max_Position_Limit
            Table[TorqueEnable] = int(TorqueOn)
            Table[GoalPosition:GoalPosition + 2] = Position.to_bytes(2, "little")
            Table[PresentPosition:PresentPosition + 2] = Position.to_bytes(2, "little")
            Table[62] = 90                                  # Present_Voltage 9.0 V
            Table[63] = 30                                  # Present_Temperature
            Table[0:2] = bytes([3, 10])                     # firmware 3.10
            Table[21:28] = bytes([32, 32, 0, 16, 0, 1, 1])  # P, D, I, punch, -, dead zones (the arm's servos)
            Table[TorqueLimit:TorqueLimit + 2] = (1000).to_bytes(2, "little")
            Self.Tables[Id] = Table
        Self.RemoveAfterRequest: tuple[int, int] | None = None
        Self.RemoveOnRequest: tuple[int, int] | None = None
        # A glitch rather than an unplug: the old handle dies, but a new open works at once.
        Self.ComeBackAtOnce = False
        Self.Opens = 0
        Self.Handles: list[FakeSerial] = []
        # Every packet the servos received: (instruction, id, address, data bytes). A SYNC_WRITE is kept as one WRITE per
        # servo it addressed (instruction 3), so Writes() sees it; a SYNC_READ as one READ per servo (instruction 2).
        Self.Received: list[tuple[int, int, int, bytes]] = []
        Self.Silent: set[int] = set()  # servo ids that answer nothing (a cable, a dead servo)
        Self.SyncPackets: list[tuple[int, int, int, tuple[int, ...]]] = []  # (instruction, address, length, ids)
        Self.OnStep: Callable[["FakeServoDevice", float], None] | None = None

    def Remove(Self) -> None:
        Self.Present = Self.ComeBackAtOnce
        Self.Generation += 1

    def Arrive(Self, As: str = FakePort) -> None:
        """The adapter is back, under port name As (Windows may give it another COM number)."""
        Self.Present, Self.Port = True, As

    def Register(Self, Id: int, Address: int, Length: int = 1) -> int:
        return int.from_bytes(Self.Tables[Id][Address:Address + Length], "little")

    def Torque(Self) -> dict[int, int]:
        return {Id: Table[TorqueEnable] for Id, Table in Self.Tables.items()}

    def Position(Self, Id: int) -> int:
        return Self.Register(Id, PresentPosition, 2)

    def SetPosition(Self, Id: int, Position: int) -> None:
        """Present_Position as the servo reports it: in position mode it counts on past a turn, a negative count with its
        sign in bit 15 (the bridge keeps that turn count, HardwareRaw)."""
        Position = int(Position)
        Self.Tables[Id][PresentPosition:PresentPosition + 2] = ((-Position | 0x8000) if Position < 0 else Position).to_bytes(2, "little")

    def SetLoad(Self, Id: int, Duty: int) -> None:
        """Present_Load as the servo reports it: magnitude 0-1000, bit 10 set for the negative direction."""
        Duty = max(-1000, min(1000, int(Duty)))
        Self.Tables[Id][PresentLoad:PresentLoad + 2] = ((-Duty | 1024) if Duty < 0 else Duty).to_bytes(2, "little")

    def SetVoltage(Self, Id: int, Volts: float) -> None:
        Self.Tables[Id][PresentVoltage] = round(Volts * 10)

    def SetTemperature(Self, Id: int, Celsius: int) -> None:
        Self.Tables[Id][PresentTemperature] = int(Celsius)

    def Restart(Self, Id: int) -> None:
        """A brown-out reset of one servo: torque off, Lock 0, Torque_Limit 1000, its goal where it is."""
        Table = Self.Tables[Id]
        Table[TorqueEnable], Table[Lock] = 0, 0
        Table[TorqueLimit:TorqueLimit + 2] = (1000).to_bytes(2, "little")
        Table[GoalPosition:GoalPosition + 2] = Table[PresentPosition:PresentPosition + 2]

    def Writes(Self, Address: int | None = None, Since: int = 0) -> list[tuple[int, int, int]]:
        """(id, address, value) of every write the servos received from index Since on."""
        return [(Id, At, int.from_bytes(Data, "little")) for Instruction, Id, At, Data in Self.Received[Since:]
                if Instruction == 3 and (Address is None or At == Address)]

    def Step(Self, Seconds: float) -> None:
        """Move every torqued servo for Seconds: velocity mode at its goal speed, position mode toward its goal."""
        for Id, Table in Self.Tables.items():
            if not Table[TorqueEnable]:
                continue
            Present = Self.Position(Id)
            if Table[OperatingMode] == 1:
                Present += round(SignedMagnitude(Self.Register(Id, GoalSpeed, 2)) * Seconds)
            elif Table[OperatingMode] == 0:
                Goal = SignedMagnitude(Self.Register(Id, GoalPosition, 2))
                Speed = abs(SignedMagnitude(Self.Register(Id, GoalSpeed, 2))) or 3000
                Room = round(Speed * Seconds)
                Present += max(-Room, min(Room, Goal - Present))
            Self.SetPosition(Id, max(0, min(4095, Present)))
        if Self.OnStep is not None:
            Self.OnStep(Self, Seconds)

    def Answer(Self, Packet: bytes) -> bytes:
        Id, Instruction, Parameters = Packet[2], Packet[4], Packet[5:-1]
        if Id == Broadcast and Instruction == SyncRead:
            Address, Length, Ids = Parameters[0], Parameters[1], tuple(Parameters[2:])
            Self.SyncPackets.append((Instruction, Address, Length, Ids))
            Reply = b""
            for Each in Ids:
                if Each in Self.Tables and Each not in Self.Silent:
                    Self.Received.append((2, Each, Address, bytes([Length])))
                    Reply += StatusPacket(Each, bytes(Self.Tables[Each][Address:Address + Length]))
            return Reply
        if Id == Broadcast and Instruction == SyncWrite:
            Address, Length = Parameters[0], Parameters[1]
            Ids = []
            for At in range(2, len(Parameters), Length + 1):
                Each, Data = Parameters[At], bytes(Parameters[At + 1:At + 1 + Length])
                Ids.append(Each)
                if Each in Self.Tables and Each not in Self.Silent:
                    Self.Received.append((3, Each, Address, Data))
                    Self.Tables[Each][Address:Address + Length] = Data
                    if Address <= GoalPosition < Address + Length or Address <= GoalSpeed < Address + Length:
                        Self.Tables[Each][TorqueEnable] = 1  # a goal write energises, as a single write does
            Self.SyncPackets.append((Instruction, Address, Length, tuple(Ids)))
            return b""
        Table = Self.Tables.get(Id)
        if Table is None or Id in Self.Silent:
            return b""
        if Instruction == 2:  # READ address, length
            Address, Length = Parameters[0], Parameters[1]
            Self.Received.append((Instruction, Id, Address, bytes(Parameters[1:2])))
            return StatusPacket(Id, bytes(Table[Address:Address + Length]))
        if Instruction == 3:  # WRITE address, data
            Address, Data = Parameters[0], bytes(Parameters[1:])
            Self.Received.append((Instruction, Id, Address, Data))
            Table[Address:Address + len(Data)] = Data
            if Address in (GoalPosition, GoalSpeed):
                Table[TorqueEnable] = 1  # an STS3215 energises on any goal write (MotorProtection.md)
            return StatusPacket(Id)
        return StatusPacket(Id)


def StatusPacket(Id: int, Data: bytes = b"", Error: int = 0) -> bytes:
    Body = bytes([Id, len(Data) + 2, Error]) + Data
    return b"\xff\xff" + Body + bytes([~sum(Body) & 0xFF])


class FakeSerial:
    """The pyserial surface scservo_sdk.PortHandler uses, for one handle on FakePort."""

    Device: FakeServoDevice | None = None

    def __init__(Self, port=None, baudrate=1_000_000, bytesize=8, timeout=None, **_) -> None:
        if port not in FakePorts:
            raise AssertionError(f"an offline test tried to open the real serial port {port!r}")
        Device = FakeSerial.Device
        if Device is None or not Device.Present or port != Device.Port:
            raise serial.SerialException(f"could not open port {port!r}: FileNotFoundError(2, "
                                         "'The system cannot find the file specified.', None, 2)")
        Self.Generation = Device.Generation
        Self.Rx = bytearray()
        Self.is_open = True
        Self.Calls = 0
        Device.Opens += 1
        Device.Handles.append(Self)

    def _Alive(Self) -> None:
        Self.Calls += 1
        Device = FakeSerial.Device
        if not Self.is_open:
            raise serial.PortNotOpenError()
        if not (Device.Present and Device.Generation == Self.Generation):
            raise serial.SerialException(DeadHandleText)

    def flush(Self) -> None:  # pyserial: polls out_waiting, i.e. ClearCommError
        Self._Alive()

    @property
    def in_waiting(Self) -> int:
        Self._Alive()
        return len(Self.Rx)

    def reset_input_buffer(Self) -> None:  # pyserial: PurgeComm, failure ignored
        Self.Calls += 1
        Self.Rx.clear()

    def read(Self, Size: int = 1) -> bytes:
        Self._Alive()
        Out = bytes(Self.Rx[:Size])
        del Self.Rx[:Size]
        return Out

    def write(Self, Data) -> int:
        Self._Alive()
        Packet, Device = bytes(Data), FakeSerial.Device
        Request = (Packet[2], Packet[5]) if len(Packet) > 5 else None
        if Request is not None and Device.RemoveOnRequest == Request:
            Device.RemoveOnRequest = None
            Device.Remove()
            raise serial.SerialException(DeadHandleText.replace("ClearCommError", "WriteFile"))
        Reply = Device.Answer(Packet)
        if Request is not None and Device.RemoveAfterRequest == Request:
            Device.RemoveAfterRequest = None
            Device.Remove()  # unplugged before the reply came back
            return len(Packet)
        Self.Rx += Reply
        return len(Packet)

    def close(Self) -> None:  # closing a dead handle does not raise in pyserial
        Self.Calls += 1
        Self.is_open = False


@contextlib.contextmanager
def Installed(Device: FakeServoDevice) -> Iterator[FakeServoDevice]:
    """serial.Serial is FakeSerial on Device for the duration: only FakePort can open."""
    Previous = FakeSerial.Device
    FakeSerial.Device = Device
    try:
        with patch.object(serial, "Serial", FakeSerial):
            yield Device
    finally:
        FakeSerial.Device = Previous
