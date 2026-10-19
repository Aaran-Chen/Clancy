"""The hand-guiding loop's lean transport on the bridge's own servo bus (docs/HandGuide.md 3.5).

One SYNC_READ (0x82) or SYNC_WRITE (0x83) packet per call on the FeetechMotorsBus's open port handle, under its
_Transaction lock, so it never interleaves with the bridge's own transactions. Unlike the SDK (GroupSyncRead) it
  * drains the input first (reset_input_buffer) and never calls PortHandler.clearPort(): pyserial's flush() on Windows
    can sleep 50 ms;
  * reads the staggered replies against its own deadline (5 ms) instead of the SDK's 34 ms per silent servo, and keeps
    every reply that came: one servo that misses its reply is that servo's miss only (the SDK stops at the first);
  * counts into the bus's BusStats, so /bus/stats stays meaningful, and marks the port lost on an OS error exactly as
    the driver does (PortLostError).
Encoders: Goal_Position, Present_Position and Present_Speed carry their sign in bit 15, Present_Load in bit 10.

    Bus = GuideBus(controller.bus, MOTOR_NAMES)
    Readings = Bus.ReadPresent()          # {name: {"pos", "speed", "load", "volts", "temp_c", "status", ...} or None}
    Bus.WriteGoals({"shoulder_lift": (2731, 420)})
    Bus.Release(["wrist_roll"])           # Torque_Enable 0 only: Lock stays 1
"""
from __future__ import annotations

import time
from typing import Iterable, Sequence

from feetech_simple import PortLostError

INST_SYNC_READ = 0x82
INST_SYNC_WRITE = 0x83
BROADCAST_ID = 0xFE
# Present_Position (56) .. Present_Current (69-70): position, speed, load, voltage, temperature, async, status, moving
PRESENT_START, PRESENT_LENGTH = 56, 15
# Torque_Enable (40) .. Lock (55): torque, acceleration, goal, goal time, goal speed, torque limit, ..., lock
STATUS_START, STATUS_LENGTH = 40, 16
# Regs 0..33: firmware (0, 1), model (3, 4), id, baud, return delay, ..., P (21), D (22), I (23), punch (24), dead zones (26,
# 27), ..., offset (31, 32), operating mode (33): what a calibration was made with (its fingerprint)
SETTINGS_START, SETTINGS_LENGTH = 0, 34
DEADLINE_S = 0.005


def EncodeSigned(Value: int, Bit: int = 15) -> int:
    """Signed-magnitude encoding: the magnitude, with bit Bit set when Value is negative."""
    Value = int(Value)
    return (-Value | (1 << Bit)) if Value < 0 else Value


def DecodeSigned(Raw: int, Bit: int = 15) -> int:
    Raw = int(Raw)
    return -(Raw & ~(1 << Bit)) if Raw & (1 << Bit) else Raw


def Checksum(Body: Sequence[int]) -> int:
    return ~sum(Body) & 0xFF


def SyncReadPacket(Ids: Sequence[int], Start: int, Length: int) -> bytes:
    Body = [BROADCAST_ID, len(Ids) + 4, INST_SYNC_READ, Start, Length, *Ids]
    return bytes([0xFF, 0xFF, *Body, Checksum(Body)])


def SyncWritePacket(Start: int, Length: int, Data: dict[int, Sequence[int]]) -> bytes:
    Params = []
    for Id, Bytes in Data.items():
        if len(Bytes) != Length:
            raise ValueError(f"servo {Id}: {len(Bytes)} bytes for a {Length}-byte sync write")
        Params += [Id, *Bytes]
    Body = [BROADCAST_ID, len(Params) + 4, INST_SYNC_WRITE, Start, Length, *Params]
    return bytes([0xFF, 0xFF, *Body, Checksum(Body)])


def ParseReplies(Buffer: bytearray, Length: int) -> dict[int, tuple[int, bytes]]:
    """Every complete status packet in Buffer with Length data bytes and a good checksum: {id: (error byte, data)}.
    Consumes what it parsed; a partial packet at the end stays in Buffer."""
    Out: dict[int, tuple[int, bytes]] = {}
    Size = Length + 6  # FF FF ID LEN ERR <data> CHK
    while True:
        Start = Buffer.find(b"\xff\xff")
        if Start < 0:
            del Buffer[:max(0, len(Buffer) - 1)]
            return Out
        if Start:
            del Buffer[:Start]
        if len(Buffer) < 4:
            return Out
        if Buffer[2] == 0xFF:  # a third FF: the header starts one byte later
            del Buffer[:1]
            continue
        if Buffer[3] != Length + 2:
            del Buffer[:2]  # not a reply of this length: skip its header
            continue
        if len(Buffer) < Size:
            return Out
        Packet = bytes(Buffer[:Size])
        del Buffer[:Size]
        if Packet[-1] == Checksum(Packet[2:-1]):
            Out[Packet[2]] = (Packet[4], Packet[5:5 + Length])


class GuideBus:
    """SYNC_READ / SYNC_WRITE for the guide loop on an open FeetechMotorsBus (see the module docstring)."""

    def __init__(Self, Bus, Names: Iterable[str], DeadlineS: float = DEADLINE_S) -> None:
        Self.Bus = Bus
        Self.Names = tuple(Names)
        Self.Ids = {Name: Bus.motors[Name].id for Name in Self.Names}
        Self.DeadlineS = DeadlineS
        Self.LastCycleMs = 0.0

    # -- the transport ---------------------------------------------------------------------------------------------------
    def _Exchange(Self, Kind: str, Packet: bytes, Expect: Sequence[int], Length: int, DeadlineS: float):
        Bus = Self.Bus
        if Bus.PortLost:
            raise PortLostError(Bus.PortLostText())
        Replies: dict[int, tuple[int, bytes]] = {}
        with Bus._Transaction:
            Started = time.perf_counter()
            Handler = Bus._port_handler
            try:
                Serial = Handler.ser
                Serial.reset_input_buffer()
                if Serial.write(Packet) != len(Packet):
                    raise OSError(f"{Kind}: short write")
                if Expect:
                    Buffer = bytearray()
                    Deadline = Started + DeadlineS
                    while True:
                        Chunk = Serial.read(4096)
                        if Chunk:
                            Buffer += Chunk
                            Replies.update(ParseReplies(Buffer, Length))
                            if all(Id in Replies for Id in Expect):
                                break
                        elif time.perf_counter() >= Deadline:
                            break
                        else:
                            time.sleep(0)
            except OSError as Exc:
                Bus._mark_port_lost(Exc, Kind, BROADCAST_ID, Packet[5], Started)
                raise PortLostError(Bus.PortLostText()) from Exc
            finally:
                Handler.is_using = False
        Missing = [Id for Id in Expect if Id not in Replies]
        Ms = (time.perf_counter() - Started) * 1000.0
        Stats = Bus.BusStats
        Stats["attempts"] += 1
        if Missing:
            Stats["failures"] += 1
            Names = [Name for Name, Id in Self.Ids.items() if Id in Missing]
            Stats["recent_failures"].append({"at": round(time.time(), 3), "kind": Kind, "motor": ",".join(Names),
                                             "register": str(Packet[5]), "ms": round(Ms, 1), "result": "no reply"})
        else:
            Stats["max_ok_ms"] = max(Stats["max_ok_ms"], Ms)
            Stats["slow_ok"] += Ms > 20.0
        Self.LastCycleMs = Ms
        for Id, (Error, _) in Replies.items():
            Name = next((N for N, I in Self.Ids.items() if I == Id), None)
            if Name is not None:
                Bus.PacketFaults[Name] = int(Error)
        return Replies

    def SyncRead(Self, Start: int, Length: int, Names: Sequence[str] | None = None,
                 DeadlineS: float | None = None) -> dict[str, tuple[int, bytes] | None]:
        """{name: (error byte, Length data bytes), or None when that servo did not answer in time}."""
        Names = tuple(Names or Self.Names)
        Ids = [Self.Ids[Name] for Name in Names]
        Replies = Self._Exchange("sync_read", SyncReadPacket(Ids, Start, Length), Ids, Length,
                                 Self.DeadlineS if DeadlineS is None else DeadlineS)
        return {Name: Replies.get(Self.Ids[Name]) for Name in Names}

    def SyncWrite(Self, Start: int, Length: int, Data: dict[str, Sequence[int]]) -> None:
        """One SYNC_WRITE of Length bytes from Start to each named servo (no reply comes)."""
        if Data:
            Self._Exchange("sync_write", SyncWritePacket(Start, Length, {Self.Ids[N]: B for N, B in Data.items()}),
                           (), Length, 0.0)

    # -- what the guide loop reads and writes ------------------------------------------------------------------------------
    def ReadPresent(Self, Names: Sequence[str] | None = None) -> dict[str, dict | None]:
        """Regs 56..70 of every servo, decoded: pos (steps, bit 15), speed (bit 15), load (duty, bit 10), volts, temp_c,
        status (the reply's error byte with Hardware_Error_Status), moving, current (bit 15). None: no reply."""
        Out = {}
        for Name, Reply in Self.SyncRead(PRESENT_START, PRESENT_LENGTH, Names).items():
            if Reply is None:
                Out[Name] = None
                continue
            Error, D = Reply
            Word = lambda At: D[At - PRESENT_START] | (D[At - PRESENT_START + 1] << 8)
            Out[Name] = {"pos": DecodeSigned(Word(56), 15), "speed": DecodeSigned(Word(58), 15),
                         "load": DecodeSigned(Word(60), 10), "volts": D[62 - PRESENT_START] / 10.0,
                         "temp_c": float(D[63 - PRESENT_START]), "status": int(Error) | D[65 - PRESENT_START],
                         "moving": D[66 - PRESENT_START], "current": DecodeSigned(Word(69), 15)}
        return Out

    def ReadStatus(Self, Names: Sequence[str] | None = None) -> dict[str, dict | None]:
        """Regs 40..55: torque (40), acceleration (41), goal (42, bit 15), goal_time (44), goal_speed (46, bit 15),
        limit (48), lock (55). None: no reply."""
        Out = {}
        for Name, Reply in Self.SyncRead(STATUS_START, STATUS_LENGTH, Names).items():
            if Reply is None:
                Out[Name] = None
                continue
            Error, D = Reply
            Word = lambda At: D[At - STATUS_START] | (D[At - STATUS_START + 1] << 8)
            Out[Name] = {"torque": bool(D[0]), "acceleration": D[1], "goal": DecodeSigned(Word(42), 15),
                         "goal_time": Word(44), "goal_speed": DecodeSigned(Word(46), 15), "limit": Word(48),
                         "lock": D[55 - STATUS_START], "error": int(Error)}
        return Out

    def ReadSettings(Self, Names: Sequence[str] | None = None) -> dict[str, dict | None]:
        """Regs 0..33 (EEPROM): firmware, model, P, D, I, punch, dead zones, offset and operating mode. Read only at the
        start of a session, with a longer deadline (34 bytes from each servo)."""
        Out = {}
        for Name, Reply in Self.SyncRead(SETTINGS_START, SETTINGS_LENGTH, Names, DeadlineS=max(Self.DeadlineS, 0.02)).items():
            if Reply is None:
                Out[Name] = None
                continue
            _, D = Reply
            Out[Name] = {"firmware": [D[0], D[1]], "model": D[3] | (D[4] << 8), "p": D[21], "d": D[22], "i": D[23],
                         "punch": D[24], "dead_zone": [D[26], D[27]], "offset": DecodeSigned(D[31] | (D[32] << 8), 11),
                         "mode": D[33]}
        return Out

    def WriteGoals(Self, Goals: dict[str, tuple[float, float]], Acceleration: int | None = None,
                   GoalSpeed: int = 0) -> None:
        """Goal_Position (rounded, bit 15) and Torque_Limit (0..1000) per servo, with Goal_Time 0 and Goal_Speed GoalSpeed
        (0: the position loop's own response, as the law assumes): regs 42..49, or 41..49 with Acceleration."""
        Data = {}
        for Name, (Goal, Limit) in Goals.items():
            G = EncodeSigned(round(Goal), 15)
            Lim = max(0, min(1000, round(Limit)))
            S = EncodeSigned(int(GoalSpeed), 15)
            Bytes = [G & 0xFF, (G >> 8) & 0xFF, 0, 0, S & 0xFF, (S >> 8) & 0xFF, Lim & 0xFF, (Lim >> 8) & 0xFF]
            Data[Name] = ([max(0, min(254, int(Acceleration)))] + Bytes) if Acceleration is not None else Bytes
        if Acceleration is None:
            Self.SyncWrite(42, 8, Data)
        else:
            Self.SyncWrite(41, 9, Data)

    def Release(Self, Names: Iterable[str]) -> None:
        """Torque_Enable 0 on these servos (floating), nothing else: Lock stays 1 (disable_torque writes Lock 0, the
        signature the guide loop reads as a servo restart)."""
        Self.SyncWrite(40, 1, {Name: [0] for Name in Names})
