"""Protection readings act only once they persist (ProtectionVersion auto-coast-v2-confirmed, MotorProtection.md). Offline.

2026-10-12 (bridge_events.jsonl, UTC) two false alarms coasted a healthy holding arm:
- 16:17:25 "Lost shoulder_pan protection feedback": one Present_Temperature read had no status packet (both tries), every
  joint's position reading fine. Now a temperature must fail on two polls in a row (about 0.5 s).
- 16:32:43 "shoulder lift torque unexpectedly off": one status reading read torque off with load 63745 (0xF901), 5.6 V
  and 90 C, the signature of every torque-off fault recorded since 2026-09-28, positions consistent with the targets.
  Now a status reading that shows anything is read again at once, and a reading whose telemetry no servo can report is a
  failed read: only two bad readings in a row latch.
2026-10-22 two such corrupted readings in a row (shoulder_pan status 128 then 129, load 64001, 0 V, 90 C, Lock 28) latched
"Cannot read shoulder_pan protection/torque status" with every joint on its target. Now a corrupted reply never latches on
its own: a protection_glitch once per streak, and the streak confirms over time and over independent reads (review round 1,
.review-round1.md, findings 1-13): torque read off is read again on its own and re-asserted with a goal write, latching
if it stays off; a holding joint latches once it moved 30 steps further from its hold point than when the streak began;
a route is held, and a grasp latches, past 2 s; every streak latches past 5 s. An over-limit temperature trips once it
persisted 1 s, never on a missed re-read or during a corrupted status streak; ReadEveryJoint re-reads status too.
Review round 2: a servo whose Lock does not read 1 on its own re-reads (a restart reads 0) is never re-asserted; the hot
window counts only polls outside a streak; ReadEveryJoint re-reads Operating_Mode; MotionPreflight reads a corrupted pair
again before refusing.
Review round 3: torque read off on a joint's own re-reads stays a restart check until its Lock reads 1 (a hold correction,
a route's or Stop's goal write energised a restarted servo and hid it), and nothing the bridge writes reaches it meanwhile;
a steady over-limit temperature counts in a streak (only corrupt-looking ones are left out) and the cap counts a chain of
streaks; a fault reading paired with an unanswered one confirms nothing; ReadEveryJoint re-reads an unanswered torque,
status or position read; a re-assert's goal (and the 512-step drift latch) uses a confirmed position re-read.
Review round 4: while a restart check is pending nothing moves unsupervised: a route or timed motion holds where it is at
once, and a hold correction streaming into that joint is stopped with one Goal_Speed write (0 in velocity mode, else 1),
resuming once its Lock reads 1. Skipping its writes alone left a velocity-mode joint at its last Goal_Speed.
Review round 5: that hold runs in the burst's own frame, and one more garbled read-back (goal 28) raised "Hold read-back
mismatch" and coasted every joint mid-Home. For a joint in a streak or a pending check, a read-back that disagrees is read
again on its own twice and a failed write is written once more; only both re-reads disagreeing (or both writes failing)
still coast, as does a joint that really never took position mode; a real restart still latches on its Lock.
Review round 6: a joint really in velocity mode with one mode re-read garbled to position still coasts; one garbled
Present_Position read mid-route is read again twice instead of latching "left the checked basket route" (a position the
re-reads confirm off the route still latches); a grasping gripper's squeeze under a streak or a pending check is tolerant.
Real faults still latch: a torque-off or fault bit that the re-read confirms, a servo that restarted (brown-out: torque
off, Lock 0) in the same frame, a supply over 14 V, two failed (unanswered) status readings in a row, two missed
temperature polls in a row.

The fake-bus tests use test_bridge_home's FakeBus; the replays run the REAL FeetechMotorsBus and scservo SDK over
FakeServoSerial (only FAKE-COM4 can open) on a simulated clock. Never opens a serial port or the depth camera, never talks
to the live bridge, and never raises this process's priority.

Run:  python TestProtectionConfirmation.py
"""
import time
import unittest
from unittest.mock import patch

import bridge_server as Bridge
from FakeServoSerial import FakePort, FakeServoDevice, Installed
from TestSafety import MakeController
from test_bridge_home import displace as Displace, make_controller as HomeController

# Every joint's present_raw at 2026-10-12 16:32:43 (bridge_events.jsonl), servo ids 1-6.
Positions1632 = {1: 3198, 2: 514, 3: 1784, 4: 156, 5: 1521, 6: 703}
Ids = {Name: Index + 1 for Index, Name in enumerate(Bridge.MOTOR_NAMES)}
HardwareCalls = []
RealMonotonic = time.monotonic
# The 16:32:43 reply's registers: torque 0, load 0xF901, 5.6 V, 90 C (Hardware_Error_Status read 0: bits 128 = torque).
Reply1632 = {40: b"\x00", 60: (0xF901).to_bytes(2, "little"), 62: bytes([56]), 63: bytes([90])}
# The 2026-10-22 status reply's torque, load and supply (status 129, load 64001, 0 V), the temperature left real.
StatusOnly1022 = {40: b"\x00", 60: (64001).to_bytes(2, "little"), 62: bytes([0])}
# The same reply's status bit and load, its torque left real (Torque_Enable reads on): only the status reads as corrupted.
StatusBit1022 = {65: bytes([1]), 60: (64001).to_bytes(2, "little")}


def GuardedOpener():
    HardwareCalls.append("depth camera opened")
    raise RuntimeError("an offline test reached the real depth camera")


def setUpModule():
    for Patch in (patch.object(Bridge.depth_camera, "Opener", GuardedOpener),
                  patch.object(Bridge, "RaiseProcessPriority", lambda: HardwareCalls.append("priority raised") or "normal")):
        Patch.start()
        unittest.addModuleCleanup(Patch.stop)


def CorruptReads(Device: FakeServoDevice, Id: int, Changes: dict[int, bytes], Times: int) -> list[int]:
    """The next Times READs from servo Id of an address in Changes answer with Changes in place of its table: a corrupted
    reply whose checksum is good. Returns the count still left (a one-item list)."""
    Original, Left = Device.Answer, [Times]

    def Answer(Packet):
        if Left[0] and Packet[2] == Id and Packet[4] == 2 and Packet[5] in Changes:
            Left[0] -= 1
            Saved = bytes(Device.Tables[Id])
            for Register, Value in Changes.items():
                Device.Tables[Id][Register:Register + len(Value)] = Value
            try:
                return Original(Packet)
            finally:
                Device.Tables[Id][:] = Saved
        return Original(Packet)
    Device.Answer = Answer
    return Left


def ScriptedReads(Device: FakeServoDevice, Id: int, Script: dict[int, list[bytes]]) -> dict[int, list[bytes]]:
    """Each READ from servo Id at an address in Script answers the next bytes in Script[address] in place of its table
    (a corrupted reply with a good checksum), in order, until that list is empty. Returns Script."""
    Original = Device.Answer

    def Answer(Packet):
        if Packet[2] == Id and Packet[4] == 2 and Script.get(Packet[5]):
            Value, Saved = Script[Packet[5]].pop(0), bytes(Device.Tables[Id])
            Device.Tables[Id][Packet[5]:Packet[5] + len(Value)] = Value
            try:
                return Original(Packet)
            finally:
                Device.Tables[Id][:] = Saved
        return Original(Packet)
    Device.Answer = Answer
    return Script


def MissReplies(Device: FakeServoDevice, Id: int, Address: int, Times: int) -> list[int]:
    """The next Times READs from servo Id at Address get no status packet ("There is no status packet!")."""
    Original, Left = Device.Answer, [Times]

    def Answer(Packet):
        if Left[0] and Packet[2] == Id and Packet[4] == 2 and Packet[5] == Address:
            Left[0] -= 1
            return b""
        return Original(Packet)
    Device.Answer = Answer
    return Left


def ScriptReads(Bus, Name, Script=None, Sustain=None):
    """FakeBus reads of Name answer Script[Register] in order (a value, or an exception to raise) while it lasts, and
    Sustain[Register] for as long as Sustain holds it (a test empties it to end the corruption), else the bus's own
    registers. Returns (Script, Sustain)."""
    Read, Script, Sustain = Bus.read, Script if Script is not None else {}, Sustain if Sustain is not None else {}

    def Scripted(Reg, Motor, **Options):
        if Motor == Name and Script.get(Reg):
            Value = Script[Reg].pop(0)
            if isinstance(Value, Exception):
                raise Value
            return Value
        if Motor == Name and Reg in Sustain:
            return Sustain[Reg]
        return Read(Reg, Motor, **Options)
    Bus.read = Scripted
    return Script, Sustain


def Run(Controller, Clock, Seconds, Each=None):
    """refresh() at 30 Hz for Seconds on the simulated Clock (a one-item list), Each() before each frame. Stops at a
    motion fault; returns the seconds run."""
    Start = Clock[0]
    with patch.object(Bridge.time, "monotonic", lambda: Clock[0]):
        Controller.last_refresh_s = Clock[0]
        for _ in range(round(Seconds * 30)):
            Clock[0] += 1 / 30
            if Each is not None:
                Each()
            Controller.refresh()
            if Controller.MotionFault:
                break
    return Clock[0] - Start


# The 2026-10-22 status telemetry (status 1 from Hardware_Error_Status, load 64001, 0 V, 90 C, Lock 28), torque left real.
Corrupt1022 = {"Hardware_Error_Status": 1, "Present_Load": 64001, "Present_Voltage": 0, "Present_Temperature": 90, "Lock": 28}


def Recording(Controller):
    Events = []
    Controller.EventSink = Events.append
    return Events


def Kinds(Events, Kind):
    return [Event for Event in Events if Event["kind"] == Kind]


class FakeBusTests(unittest.TestCase):
    """The rules on test_bridge_home's FakeBus (reads answer from its registers)."""

    def FailingTemperature(Self, Controller, Bus, Joint, Polls):
        """Present_Temperature from Joint fails on the polls in Polls (1-based); returns a function running poll N."""
        Read = Bus.read

        def Flaky(Reg, Name, **Options):
            if Reg == "Present_Temperature" and Name == Joint and Count[0] in Polls:
                raise ConnectionError(f"Read Present_Temperature from {Name} failed: [TxRxResult] There is no status packet! ")
            return Read(Reg, Name, **Options)
        Count = [0]
        Bus.read = Flaky

        def Poll():
            Count[0] += 1
            Controller.LastProtectionPoll = 0
            Controller.refresh()
        return Poll

    def TestOneMissedTemperatureReplyDoesNotTripButTwoDo(Self):
        Controller, Bus = MakeController()
        Events = Recording(Controller)
        Poll = Self.FailingTemperature(Controller, Bus, "shoulder_pan", {1, 3, 5, 6})
        for _ in range(4):  # missed, read, missed, read: never two in a row
            Poll()
            Self.assertIsNone(Controller.ProtectionTrip)
            Self.assertTrue(Bus.torque_enabled)
        Glitches = Kinds(Events, "protection_glitch")
        Self.assertEqual(len(Glitches), 2, Events)
        Self.assertEqual((Glitches[0]["joint"], Glitches[0]["details"]["register"]), ("shoulder_pan", "Present_Temperature"))
        Self.assertIn("missed reply", Glitches[0]["reason"])
        Self.assertEqual(Controller.state_snapshot()["protection"]["glitches"][-1]["register"], "Present_Temperature")
        Poll()  # missed (poll 5)
        Self.assertIsNone(Controller.ProtectionTrip)
        Poll()  # missed again (poll 6): two in a row
        Self.assertFalse(Bus.torque_enabled)
        Self.assertIn("Lost shoulder_pan protection feedback", Controller.MotionFault)
        Self.assertIn("2 polls in a row", Controller.MotionFault)
        Trip = Kinds(Events, "protection_trip")[0]
        Self.assertEqual((Trip["joint"], Trip["details"]["misses"]), ("shoulder_pan", 2))

    def TestMissedTemperatureOfASilentJointTripsAtOnce(Self):
        # The servo itself silent (its position did not read either): today's behaviour, no second poll.
        # CheckMotorProtection alone, so no frame's position read refreshes FeedbackOk.
        Controller, Bus = MakeController()
        Read = Bus.read
        Bus.read = lambda Reg, Name, **Options: (_ for _ in ()).throw(ConnectionError("no status packet")) \
            if Reg == "Present_Temperature" and Name == "wrist_flex" else Read(Reg, Name, **Options)
        Controller.servos["wrist_flex"].FeedbackOk = False
        Controller.LastProtectionPoll = 0
        Controller.CheckMotorProtection()
        Self.assertFalse(Bus.torque_enabled)
        Self.assertIn("Lost wrist_flex protection feedback", Controller.MotionFault)

    def TestBeforeHoldOneMissedTemperatureStillRefuses(Self):
        # Rearming still needs every temperature read: nothing is energised on an unverified one.
        Controller, Bus = MakeController()
        Read = Bus.read
        Bus.read = lambda Reg, Name, **Options: (_ for _ in ()).throw(ConnectionError("no status packet")) \
            if Reg == "Present_Temperature" and Name == "elbow_flex" else Read(Reg, Name, **Options)
        with Self.assertRaises(RuntimeError) as Raised:
            Controller.CheckMotorProtection(BeforeArm=True)
        Self.assertIn("Cannot verify elbow_flex temperature", str(Raised.exception))

    def TestOneTorqueOffReadingIsReadAgainAndAConfirmedOneLatches(Self):
        Controller, Bus = MakeController()
        Events = Recording(Controller)
        Read, Left = Bus.read, [1]

        def OnceOff(Reg, Name, **Options):
            if Reg == "Torque_Enable" and Name == "elbow_flex" and Left[0]:
                Left[0] -= 1
                return 0
            return Read(Reg, Name, **Options)
        Bus.read = OnceOff
        Controller.refresh()
        Self.assertIsNone(Controller.MotionFault)
        Self.assertTrue(Bus.torque_enabled)
        Glitch = Kinds(Events, "protection_glitch")[0]
        Self.assertEqual((Glitch["joint"], Glitch["details"]["register"], Glitch["details"]["value"]), ("elbow_flex", "Torque_Enable", 0))
        # Off on the re-read too (plausible telemetry): latched in that same frame.
        Left[0] = 2
        Controller.refresh()
        Self.assertIn("elbow flex torque unexpectedly off", Controller.MotionFault)
        Self.assertFalse(Bus.torque_enabled)

    def TestStatusBitsNeedTwoReadingsInARow(Self):
        for Bit in (1, 2, 4, 8, 16, 32, 64):
            with Self.subTest(Bit=Bit):
                Controller, Bus = MakeController()
                Events = Recording(Controller)
                Read, Left = Bus.read, [1]

                def Once(Reg, Name, **Options):
                    if Reg == "Hardware_Error_Status" and Name == "wrist_roll" and Left[0]:
                        Left[0] -= 1
                        return Bit
                    return Read(Reg, Name, **Options)
                Bus.read = Once
                Controller.refresh()
                Self.assertIsNone(Controller.MotionFault)
                Self.assertEqual(Kinds(Events, "protection_glitch")[0]["details"]["value"], Bit)
                Bus.registers[("Hardware_Error_Status", "wrist_roll")] = Bit
                Controller.refresh()
                Self.assertIsNotNone(Controller.MotionFault)
                Self.assertFalse(Bus.torque_enabled)

    def TestOneFailedStatusReadingIsToleratedTwoLatch(Self):
        for Register in ("Hardware_Error_Status", "Torque_Enable"):
            with Self.subTest(Register=Register):
                Controller, Bus = MakeController()
                Events = Recording(Controller)
                Read, Left = Bus.read, [1]

                def Fail(Reg, Name, **Options):
                    if Reg == Register and Name == "shoulder_lift" and Left[0]:
                        Left[0] -= 1
                        raise ConnectionError(f"Read {Reg} from {Name} failed: [TxRxResult] There is no status packet! ")
                    return Read(Reg, Name, **Options)
                Bus.read = Fail
                Controller.refresh()
                Self.assertIsNone(Controller.MotionFault)
                Glitch = Kinds(Events, "protection_glitch")[0]
                Self.assertEqual(Glitch["details"]["register"], Register)
                Left[0] = 2
                Controller.refresh()
                Self.assertIn("Cannot read shoulder_lift protection/torque status", Controller.MotionFault)
                Self.assertFalse(Bus.torque_enabled)

    def TestCorruptStatusEveryFrameNeverLatchesButARealFaultBitStillDoes(Self):
        # 2026-10-22: status 129 with load 64001, 0 V, 90 C on every reading of shoulder_pan, the arm on its targets.
        Controller, Bus = MakeController()
        Events = Recording(Controller)
        Read, Corrupt = Bus.read, [True]
        Telemetry = {"Present_Load": 64001, "Present_Voltage": 0, "Present_Temperature": 90, "Lock": 28}

        def Corrupted(Reg, Name, **Options):
            if Name == "shoulder_pan" and Corrupt[0]:
                if Reg == "Hardware_Error_Status":
                    return 1
                if Reg in Telemetry:
                    return Telemetry[Reg]
            return Read(Reg, Name, **Options)
        Bus.read = Corrupted
        Clock = [500.0]
        with patch.object(Bridge.time, "monotonic", lambda: Clock[0]):
            for _ in range(90):  # 3 s of frames at 30 Hz
                Clock[0] += 1 / 30
                Controller._check_servo_faults_locked()
            Self.assertIsNone(Controller.MotionFault)
            Self.assertTrue(Bus.torque_enabled)
            Glitches = Kinds(Events, "protection_glitch")
            Self.assertEqual(len(Glitches), 1, Events)
            Self.assertEqual((Glitches[0]["joint"], Glitches[0]["details"]["load"]), ("shoulder_pan", 64001))
            # The same fault bit with telemetry a servo can report, read twice: latched at once.
            Telemetry.update(Present_Load=120, Present_Voltage=89, Present_Temperature=31)
            Clock[0] += 1 / 30
            Controller._check_servo_faults_locked()
        Self.assertIsNotNone(Controller.MotionFault)
        Self.assertNotIn("Cannot read", Controller.MotionFault)
        Self.assertFalse(Bus.torque_enabled)

    def TestTodaysExactSignatureNeverLatches(Self):
        # 2026-10-22: shoulder_pan status 128 then 129 (torque read off; then the voltage bit too), load 64001, 0 V, 90 C,
        # Lock 28, every joint on its target; the burst also takes the first independent Torque_Enable re-read.
        for Burst in (3, 4):  # 4: both independent re-reads read off, so torque is re-asserted and reads on after
            with Self.subTest(TorqueReadsOff=Burst):
                Controller, Bus = MakeController()
                Events = Recording(Controller)
                Pan = Controller.servos["shoulder_pan"]
                ScriptReads(Bus, "shoulder_pan", {"Hardware_Error_Status": [0, 1], "Torque_Enable": [0] * Burst,
                                                  "Present_Load": [64001, 64001], "Present_Voltage": [0, 0],
                                                  "Present_Temperature": [90, 90], "Lock": [28, 28]})
                Clock = [Bridge.time.monotonic()]
                Run(Controller, Clock, 3.0)
                Self.assertIsNone(Controller.MotionFault, Controller.status)
                Self.assertTrue(Bus.torque_enabled)
                Glitches = Kinds(Events, "protection_glitch")
                Self.assertEqual(len(Glitches), 1, Events)
                Self.assertEqual((Glitches[0]["joint"], Glitches[0]["details"]["bits"], Glitches[0]["details"]["load"],
                                  Glitches[0]["details"]["voltage_dv"], Glitches[0]["details"]["temperature_c"],
                                  Glitches[0]["details"]["lock"]), ("shoulder_pan", 129, 64001, 0, 90, 28))
                Self.assertFalse(Kinds(Events, "motion_fault") + Kinds(Events, "protection_trip"))
                Reasserted = Kinds(Events, "torque_reasserted")
                Self.assertEqual(len(Reasserted), Burst - 3)
                if Reasserted:
                    Self.assertIn(("Goal_Position", "shoulder_pan", Pan.present_raw), Bus.writes)
                    Self.assertEqual(Reasserted[0]["details"]["torque_after"], 1)
                Self.assertEqual(Controller.StatusCorruptStreaks, {})
                Controller.MotionPreflight()  # clean now: a route may start

    def TestTorqueThatStaysOffUnderCorruptTelemetryLatchesAndPreflightRefuses(Self):
        # Review round 1 finding 1: the pan really off (Torque_Enable 0 on its own reads, also after the goal write), its
        # telemetry impossible on every reading, on target (no gravity load). Before: never latched, preflight passed.
        # Lock reads 1 (the bus's own register) unless Lock is given: a Lock that never reads 1 is never re-asserted.
        for Mode, Lock, Words in ((0, None, "also after a goal write at its present position"),
                                  (1, None, "not re-asserted: it is not in position mode"),
                                  (0, 28, "not re-asserted: its Lock did not read 1 on its own re-reads ([28, 28])")):
            with Self.subTest(Mode=Mode, Lock=Lock):
                Controller, Bus = MakeController()
                Bus.registers[("Operating_Mode", "shoulder_pan")] = Mode
                ScriptReads(Bus, "shoulder_pan", Sustain={"Torque_Enable": 0, "Present_Load": 64001, "Present_Voltage": 0,
                                                          "Present_Temperature": 90, **({"Lock": Lock} if Lock else {})})
                Clock = [Bridge.time.monotonic()]
                Run(Controller, Clock, 1 / 30)
                Self.assertIsNone(Controller.MotionFault)
                with patch.object(Bridge.time, "monotonic", lambda: Clock[0]):
                    with Self.assertRaises(RuntimeError) as Refused:
                        Controller.MotionPreflight()
                Self.assertIn("shoulder_pan status replies read corrupted", str(Refused.exception))
                Took = Run(Controller, Clock, 1.0)
                Self.assertIn("shoulder pan torque unexpectedly off: read off on its own re-reads", Controller.MotionFault)
                Self.assertIn(Words, Controller.MotionFault)
                Self.assertLess(Took, Bridge.StatusTorqueOffConfirmS + 0.1)
                Self.assertFalse(Bus.torque_enabled)
                Reasserts = [Write for Write in Bus.writes if Write[:2] == ("Goal_Position", "shoulder_pan")]
                Self.assertEqual(len(Reasserts), 1 if Mode == 0 and Lock is None else 0, Bus.writes)

    def TestTorqueReadOffOnlyInsideTheCorruptReadingsNeverLatches(Self):
        # Torque_Enable reads 0 only within the status readings: its own re-reads read on. 4 s, under the 5 s cap.
        Controller, Bus = MakeController()
        ScriptReads(Bus, "shoulder_pan", Sustain=dict(Corrupt1022))
        Read = Bus.read

        def InReadings(Reg, Motor, **Options):
            if Reg == "Hardware_Error_Status" and Motor == "shoulder_pan":
                InReadings.Pending = True
            elif Reg == "Torque_Enable" and Motor == "shoulder_pan" and InReadings.Pending:
                InReadings.Pending = False
                return 0
            return Read(Reg, Motor, **Options)
        InReadings.Pending = False
        Bus.read = InReadings
        Clock = [Bridge.time.monotonic()]
        Run(Controller, Clock, 4.0)
        Self.assertIsNone(Controller.MotionFault, Controller.status)
        Self.assertTrue(Bus.torque_enabled)
        Self.assertNotIn(("Goal_Position", "shoulder_pan", Controller.servos["shoulder_pan"].present_raw), Bus.writes)

    def TestAPlausibleReadingPairedWithACorruptedOneNeverLatchesOnOneFrame(Self):
        # Review round 1 finding 8: a fault reading whose telemetry failed passed as plausible, and a plausible reading
        # paired with a corrupted one latched, in either order. Each pair once, then clean: never latched.
        Fail = lambda: ConnectionError("Read Present_Load from shoulder_pan failed: [TxRxResult] There is no status packet! ")
        Plausible = {"Present_Voltage": 89, "Present_Temperature": 31, "Lock": 1}
        Cases = {"both without telemetry": ([0, 0], [Fail(), Fail()], {}),
                 "plausible then corrupted": ([0, 0], [120, 64001], Plausible),
                 "corrupted then plausible": ([0, 0], [64001, 120], Plausible),
                 "voltage bit without telemetry, then corrupted": (None, [Fail(), 64001], {})}
        for Case, (Torque, Loads, Rest) in Cases.items():
            with Self.subTest(Case=Case):
                Controller, Bus = MakeController()
                Events = Recording(Controller)
                Script = {"Present_Load": list(Loads), **{Reg: [Value] * 2 for Reg, Value in Rest.items()}}
                if Torque is not None:
                    Script["Torque_Enable"] = list(Torque)
                else:
                    Script["Hardware_Error_Status"] = [1, 1]
                ScriptReads(Bus, "shoulder_pan", Script)
                Run(Controller, [Bridge.time.monotonic()], 1.0)
                Self.assertIsNone(Controller.MotionFault, Case)
                Self.assertTrue(Bus.torque_enabled)
                Self.assertEqual(len(Kinds(Events, "protection_glitch")), 1, Events)

    def TestAFaultBitWithPlausibleTelemetryInTwoFramesOfAStreakLatches(Self):
        # Confirmed over time: the overload bit read with telemetry a servo can report in two frames, each paired with a
        # corrupted reply. Latched on the second frame.
        Controller, Bus = MakeController()
        Script = {"Hardware_Error_Status": [32] * 4, "Present_Load": [800, 64001, 64001, 800],
                  "Present_Voltage": [89] * 4, "Present_Temperature": [33] * 4, "Lock": [1] * 4}
        ScriptReads(Bus, "elbow_flex", Script)
        Clock = [Bridge.time.monotonic()]
        Run(Controller, Clock, 1 / 30)
        Self.assertIsNone(Controller.MotionFault)
        Run(Controller, Clock, 1 / 30)
        Self.assertIn("elbow flex overload", Controller.MotionFault)
        Self.assertFalse(Bus.torque_enabled)

    def TestARealOverVoltageLatches(Self):
        # Review round 1 finding 2: a 15.5 V supply is a value a servo reports, not a corrupted one. Read twice: latched.
        Controller, Bus = MakeController()
        for Register, Value in (("Hardware_Error_Status", 1), ("Present_Voltage", 155), ("Present_Load", 100),
                                ("Present_Temperature", 35), ("Lock", 1)):
            Bus.registers[(Register, "elbow_flex")] = Value
        Controller.refresh()
        Self.assertIn("elbow flex voltage", Controller.MotionFault)
        Self.assertFalse(Bus.torque_enabled)

    def TestAGraspHoldGripperWithCorruptStatusLatchesPastTheWindow(Self):
        # Review round 1 finding 3: a grasp's target follows its jaws, so position cannot show a lost grip.
        Controller, Bus = MakeController()
        Gripper = Controller.servos["gripper"]
        Gripper.GraspHold = Gripper.Holding = True
        ScriptReads(Bus, "gripper", Sustain=dict(Corrupt1022))
        Clock = [Bridge.time.monotonic()]
        Run(Controller, Clock, 1.9)
        Self.assertIsNone(Controller.MotionFault)
        Took = Run(Controller, Clock, 1.0)
        Self.assertIn("Cannot read gripper protection/torque status for 2.0 s while it holds an object", Controller.MotionFault)
        Self.assertLess(Took, 0.3)
        Self.assertFalse(Bus.torque_enabled)

    def TestCorruptStatusOnARouteHoldsItPastTheWindowAndTheCapLatches(Self):
        # Review round 1 finding 4: a timed motion does not carry on blind. Past 2 s it stops and holds (torque on, no
        # fault); the streak goes on from there, and past StatusCorruptMaxS (5 s) it latches whatever the position.
        Controller, Bus = MakeController()
        Events = Recording(Controller)
        ScriptReads(Bus, "wrist_flex", Sustain=dict(Corrupt1022))
        Clock = [Bridge.time.monotonic()]
        # A timed motion: a stream with a deadline (the pan's position never moves on this bus; HoldRoute ends it at
        # 2 s, before its 3 s stall check).
        Pan = Controller.servos["shoulder_pan"]
        Controller._send_slider_target_locked(Pan, Pan.present_raw + 300)
        Controller.MotionDeadline = Clock[0] + 100
        Run(Controller, Clock, 1.9)
        Self.assertTrue(Controller.MotionDeadline)
        Run(Controller, Clock, 0.3)
        Self.assertEqual(Controller.MotionDeadline, 0.0)
        Self.assertIsNone(Controller.MotionFault)
        Self.assertTrue(Bus.torque_enabled)
        Abort = Kinds(Events, "motion_abort")
        Self.assertEqual(len(Abort), 1, Events)
        Self.assertIn("wrist_flex status replies read corrupted for 2.0 s", Abort[0]["reason"])
        Run(Controller, Clock, 2.7)
        Self.assertIsNone(Controller.MotionFault, "under the 5 s cap")
        Run(Controller, Clock, 0.5)
        Self.assertIn("Cannot read wrist_flex protection/torque status: its replies read corrupted for 5.0 s (limit 5 s)",
                      Controller.MotionFault)
        Self.assertFalse(Bus.torque_enabled)
        Status = [Each for Each in Kinds(Events, "protection_glitch") if Each["details"]["register"] != "Present_Temperature"]
        Self.assertEqual(len(Status), 1, "one streak, one glitch")

    def TestAStreakEndsOnCoastHoldStopAndAGap(Self):
        # Review round 1 findings 5 and 11: one corrupted frame, then Coast and Hold 300 s later, then one corrupted frame
        # on a joint 40 steps off its hold point: a new streak (its own glitch), no latch.
        Controller, Bus = MakeController()
        Events = Recording(Controller)
        Script, _ = ScriptReads(Bus, "shoulder_pan")

        def CorruptFrame():
            Script.update({Reg: [Value, Value] for Reg, Value in Corrupt1022.items()})
        Clock = [500.0]
        with patch.object(Bridge.time, "monotonic", lambda: Clock[0]):
            CorruptFrame()
            Controller._check_servo_faults_locked()
            Self.assertIn("shoulder_pan", Controller.StatusCorruptStreaks)
            Self.assertTrue(Controller.toggle_torque()[0])  # Coast
            Self.assertEqual(Controller.StatusCorruptStreaks, {})
            Clock[0] += 300
            Self.assertTrue(Controller.toggle_torque()[0], Controller.status)  # Hold
            Pan = Controller.servos["shoulder_pan"]
            Pan.target_raw = (Pan.present_raw + 40) % 4096
            CorruptFrame()
            Controller._check_servo_faults_locked()
            Self.assertIsNone(Controller.MotionFault)
            Self.assertEqual(Controller.StatusCorruptStreaks["shoulder_pan"]["Since"], 800.0)
            # A gap over StatusStreakGapS starts a new streak too.
            Clock[0] += 1.0
            CorruptFrame()
            Controller._check_servo_faults_locked()
            Self.assertEqual(Controller.StatusCorruptStreaks["shoulder_pan"]["Since"], 801.0)
            Self.assertEqual(len(Kinds(Events, "protection_glitch")), 3)
            Controller.StopMotion()
            Self.assertEqual(Controller.StatusCorruptStreaks, {})
            CorruptFrame()
            Controller._check_servo_faults_locked()
            Controller.emergency_stop()
            Self.assertEqual(Controller.StatusCorruptStreaks, {})

    def TestAJointRestingOffItsHoldPointIsNoLostHoldButASagIs(Self):
        # Review round 1 findings 9 and 12: a hold correction paused near the mast leaves shoulder_lift 40 steps off its
        # hold point. 3.5 s of corrupted replies (under the 5 s cap): holding. Then it really falls: latched in frames.
        Controller, Bus = MakeController()
        for Servo in Controller.servos.values():
            Servo.Holding = True
        Controller.HoldReturnClear = lambda: False
        Lift = Controller.servos["shoulder_lift"]
        Lift.target_raw = (Lift.present_raw + 40) % 4096
        ScriptReads(Bus, "shoulder_lift", Sustain=dict(Corrupt1022))
        Clock = [Bridge.time.monotonic()]
        Run(Controller, Clock, 3.5)
        Self.assertIsNone(Controller.MotionFault, Controller.status)
        Self.assertTrue(Bus.torque_enabled)

        def Fall():
            Bus.position["shoulder_lift"] = (Bus.position["shoulder_lift"] - 5) % 4096
        Took = Run(Controller, Clock, 1.0, Fall)
        Self.assertIn("Cannot read shoulder_lift protection/torque status and it lost hold", Controller.MotionFault)
        Self.assertLess(Took, 0.35)  # 31 steps further after 7 frames, latched on the 8th
        Self.assertFalse(Bus.torque_enabled)

    def TestTheTemperaturePollDuringACorruptStatusStreakDoesNotTrip(Self):
        # Review round 1 finding 6: the 2026-10-22 telemetry on every read, Present_Temperature 90 C included (the 0.25 s
        # poll too), every joint on target. 4.5 s: holding. Past the 5 s cap the streak latches.
        Controller, Bus = MakeController()
        Events = Recording(Controller)
        ScriptReads(Bus, "shoulder_pan", Sustain=dict(Corrupt1022))
        Clock = [Bridge.time.monotonic()]
        Run(Controller, Clock, 4.5)
        Self.assertIsNone(Controller.MotionFault, Controller.status)
        Self.assertIsNone(Controller.ProtectionTrip)
        Self.assertTrue(Bus.torque_enabled)
        Hot = [Each for Each in Kinds(Events, "protection_glitch") if Each["details"]["register"] == "Present_Temperature"]
        Self.assertTrue(Hot and Hot[0]["details"]["status_corrupt"], Events)
        Run(Controller, Clock, 1.0)
        Self.assertIn("Cannot read shoulder_pan protection/torque status: its replies read corrupted for 5.0 s",
                      Controller.MotionFault)
        Self.assertFalse(Kinds(Events, "protection_trip"))

    def TestAnOverLimitTemperatureWithAMissedReReadDoesNotTrip(Self):
        # Review round 1 finding 7: one 90 C reply and an unanswered re-read were taken as confirmed.
        Controller, Bus = MakeController()
        Events = Recording(Controller)
        ScriptReads(Bus, "shoulder_pan", {"Present_Temperature": [
            90, ConnectionError("no status packet"), ConnectionError("no status packet")]})
        Controller.LastProtectionPoll = 0
        Controller.refresh()
        Self.assertIsNone(Controller.ProtectionTrip)
        Self.assertTrue(Bus.torque_enabled)
        Self.assertIn("re-read failed", Kinds(Events, "protection_glitch")[0]["reason"])
        Run(Controller, [Bridge.time.monotonic()], 2.0)
        Self.assertIsNone(Controller.ProtectionTrip)

    def TestAControlDelayResumeReadsAStatusBitAgain(Self):
        # Review round 1 finding 10: after a held-still control delay, one torque-off read or one corrupted status read
        # coasted the arm. Now each is read again; a confirmed one still trips (TestMotorProtection's delay tests).
        for Case, Script in (("one torque-off read", {"Torque_Enable": [0]}),
                             ("corrupted status", {"Hardware_Error_Status": [1, 1, 1], "Present_Load": [64001, 64001]})):
            with Self.subTest(Case=Case):
                Controller, Bus = MakeController()
                Events = Recording(Controller)
                Controller.refresh()
                Self.assertTrue(Controller.ArmHoldsStill())
                ScriptReads(Bus, "shoulder_lift", Script)
                Controller.last_refresh_s = Bridge.time.monotonic() - 1.2
                Controller.refresh()
                Self.assertIsNone(Controller.ProtectionTrip, Controller.status)
                Self.assertTrue(Bus.torque_enabled)
                Glitch = Kinds(Events, "protection_glitch")[0]
                Self.assertIn("while the control loop was delayed; read again, not confirmed", Glitch["reason"])
                Self.assertEqual(len(Kinds(Events, "control_delayed")), 1)

    def TestLostPositionFeedbackDuringACorruptStreakLatchesAtOnce(Self):
        # Review round 1 finding 13: the streak's "position did not read" branch was unreachable (the frame's position
        # read latches first); it is gone, and this is what a lost position does in a real frame during a streak.
        Controller, Bus = MakeController()
        ScriptReads(Bus, "wrist_flex", Sustain=dict(Corrupt1022))
        Clock = [Bridge.time.monotonic()]
        Run(Controller, Clock, 0.5)
        Self.assertIsNone(Controller.MotionFault)
        Read = Bus.read
        Bus.read = lambda Reg, Name, **Options: (_ for _ in ()).throw(ConnectionError("no status packet")) \
            if Reg == "Present_Position" and Name == "wrist_flex" else Read(Reg, Name, **Options)
        Took = Run(Controller, Clock, 1.0)
        Self.assertEqual(Controller.MotionFault, "Lost joint feedback: wrist_flex")
        Self.assertLess(Took, 0.05)
        Self.assertFalse(Bus.torque_enabled)

    def TestRecordedTorqueOffTelemetryIsImpossibleAndRealTelemetryIsNot(Self):
        Impossible = Bridge.BridgeController.ServoTelemetryImpossible
        # Every "torque unexpectedly off" with telemetry in captures/*/recording/bridge_events.jsonl, 2026-10-03 to 10-12.
        for Load, Volts, Celsius in ((-29953, 40, 90), (-30721, 0, 89), (-31233, 20, 89), (-29953, 0, 90), (-30209, 0, 90),
                                     (-61697, 0, 90), (63745, 0, 90), (63745, 40, 90), (63489, 152, 89), (63745, 56, 90)):
            Self.assertTrue(Impossible({"load": Load, "voltage_dv": Volts, "temperature_c": Celsius}), (Load, Volts, Celsius))
        # A record without the load still reads impossible on the supply with the heat.
        Self.assertTrue(Impossible({"voltage_dv": 56, "temperature_c": 90}))
        # Readings a real servo gives: full load either way, the arm's 8.9 V, a sagging 5.5 V supply at room heat (a
        # brown-out), 12.6 V, the hottest on record (38 C) and the 60 C limit; and no telemetry at all (a status reading
        # whose telemetry did not read is corrupted by ReadServoStatus, not by this test).
        for Telemetry in ({"load": 1000, "voltage_dv": 89, "temperature_c": 38}, {"load": -1000, "voltage_dv": 89, "temperature_c": 30},
                          {"load": 0, "voltage_dv": 55, "temperature_c": 30}, {"load": 200, "voltage_dv": 126, "temperature_c": 60},
                          {"load": 0, "voltage_dv": 0, "temperature_c": 0}, {},
                          # A supply over the 14.0 V rating is a reading a servo gives (review round 1 finding 2).
                          {"load": 100, "voltage_dv": 155, "temperature_c": 35}, {"load": 0, "voltage_dv": 240, "temperature_c": 30}):
            Self.assertIsNone(Impossible(Telemetry), Telemetry)

    def TestARestartedServoUnderCorruptTelemetryIsNeverReassertedAndLatches(Self):
        # Review round 2 finding 1: wrist_roll really restarts (Torque_Enable 0 until a goal write energises it, Lock 0)
        # and its status telemetry reads corrupted (load 64001). Before: the goal write re-asserted it, Lock stayed 0, no
        # fault, preflight passed. Now Lock is read on its own (twice) before any re-assert: 0, so never re-asserted, and
        # it latches: in the next frame once its telemetry reads plausibly, or StatusTorqueOffConfirmS later if it stays
        # corrupted. MotionPreflight refuses meanwhile.
        for Frames, Words in ((1, "wrist roll torque unexpectedly off"),
                              (None, "not re-asserted: its Lock read 0 on its own re-reads (2): the servo restarted")):
            with Self.subTest(CorruptFrames=Frames):
                Controller, Bus = MakeController()
                Events = Recording(Controller)
                State = {"On": 0, "Corrupt": True}
                Read, Write = Bus.read, Bus.write

                def Restarted(Reg, Name, **Options):
                    if Name == "wrist_roll":
                        if Reg == "Torque_Enable":
                            return State["On"]
                        if Reg == "Lock":
                            return 0
                        if Reg == "Present_Load" and State["Corrupt"]:
                            return 64001
                    return Read(Reg, Name, **Options)

                def Energises(Reg, Name, Value, **Options):
                    if Name == "wrist_roll" and Reg in ("Goal_Position", "Goal_Speed"):
                        State["On"] = 1  # on these servos a goal write turns torque back on
                    return Write(Reg, Name, Value, **Options)
                Bus.read, Bus.write = Restarted, Energises
                Clock = [Bridge.time.monotonic()]
                Run(Controller, Clock, 1 / 30)
                Self.assertIsNone(Controller.MotionFault, Controller.status)
                with patch.object(Bridge.time, "monotonic", lambda: Clock[0]):
                    with Self.assertRaises(RuntimeError) as Refused:
                        Controller.MotionPreflight()
                Self.assertIn("wrist_roll", str(Refused.exception))
                State["Corrupt"] = Frames is None
                Took = Run(Controller, Clock, 1.0)
                Self.assertIn(Words, Controller.MotionFault or "", Controller.status)
                Self.assertLess(Took, Bridge.StatusTorqueOffConfirmS + 0.1)
                Self.assertFalse(Bus.torque_enabled)
                Self.assertEqual(State["On"], 0, "never re-asserted")
                Self.assertFalse([Each for Each in Bus.writes if Each[:2] == ("Goal_Position", "wrist_roll")], Bus.writes)
                Self.assertFalse(Kinds(Events, "torque_reasserted"))

    def TestAReassertIsAVisibleEvent(Self):
        # Torque read off on its own re-reads with Lock 1 (no restart): re-asserted, recorded as a visible event (not a
        # diagnostic) and in state.protection.glitches.
        Controller, Bus = MakeController()
        ScriptReads(Bus, "shoulder_pan", {"Hardware_Error_Status": [0, 1], "Torque_Enable": [0] * 4,
                                          "Present_Load": [64001, 64001], "Present_Voltage": [0, 0],
                                          "Present_Temperature": [90, 90], "Lock": [28, 28]})
        Run(Controller, [Bridge.time.monotonic()], 1.0)
        Self.assertIsNone(Controller.MotionFault, Controller.status)
        Self.assertEqual(Controller.LastBridgeEvent["kind"], "torque_reasserted")
        Self.assertIn("Lock 1 (no restart)", Controller.LastBridgeEvent["reason"])
        Self.assertTrue(any(Each["register"] == "Torque_Enable" and Each.get("torque_after") == 1
                            for Each in Controller.ProtectionGlitches))

    def TestOverLimitTemperatureCountsOnlyOutsideACorruptStreak(Self):
        # Review round 2 finding 2: today's signature on every status reading for 1.2 s (the temperature poll reads 90
        # too), then the status reads clean while Present_Temperature stays at 90. Before: the hot window kept its start
        # from inside the streak and tripped on the first poll after it. Now it counts only outside: no trip for 0.9 s; a
        # temperature that really stays at 90 C trips once it has persisted 1 s after the streak ended.
        Controller, Bus = MakeController()
        Events = Recording(Controller)
        _, Sustain = ScriptReads(Bus, "shoulder_pan", Sustain=dict(Corrupt1022))
        Clock = [Bridge.time.monotonic()]
        Run(Controller, Clock, 1.2)
        Self.assertIsNone(Controller.MotionFault, Controller.status)
        with patch.object(Bridge.time, "monotonic", lambda: Clock[0]):
            Self.assertTrue(Controller.ActiveStatusStreaks())
        for Register in ("Hardware_Error_Status", "Present_Load", "Present_Voltage", "Lock"):
            del Sustain[Register]
        Run(Controller, Clock, 0.9)
        Self.assertEqual(Controller.StatusCorruptStreaks, {})
        Self.assertIsNone(Controller.ProtectionTrip, Controller.status)
        Self.assertTrue(Bus.torque_enabled)
        Hot = [Each for Each in Kinds(Events, "protection_glitch") if Each["details"]["register"] == "Present_Temperature"]
        Self.assertTrue(Hot[0]["details"]["status_corrupt"] and not Hot[-1]["details"]["status_corrupt"], Hot)
        Run(Controller, Clock, 0.4)
        Self.assertIn("shoulder_pan temperature 90 C reached software limit 60 C", (Controller.ProtectionTrip or {}).get("reason", ""))
        Self.assertFalse(Bus.torque_enabled)

    def TestOneCorruptedOperatingModeReadAfterAControlDelayDoesNotCoast(Self):
        # Review round 2 finding 3: after a held-still control delay one Operating_Mode read of 28 (every later read
        # real) coasted the arm. Now it is read again twice; one that both re-reads confirm still trips.
        for Case, Script, Trips in (("one corrupted mode read", {"Operating_Mode": [28]}, False),
                                    ("one unanswered mode read", {"Operating_Mode": [ConnectionError("no status packet")]}, False),
                                    ("velocity mode on every read", {"Operating_Mode": [1, 1, 1]}, True)):
            with Self.subTest(Case=Case):
                Controller, Bus = MakeController()
                Events = Recording(Controller)
                Controller.refresh()
                Self.assertTrue(Controller.ArmHoldsStill())
                ScriptReads(Bus, "wrist_flex", Script)
                Controller.last_refresh_s = Bridge.time.monotonic() - 1.5
                Controller.refresh()
                if Trips:
                    Self.assertIn("Motor control heartbeat was delayed: wrist_flex is not in position mode",
                                  (Controller.ProtectionTrip or {}).get("reason", ""))
                    Self.assertFalse(Bus.torque_enabled)
                    continue
                Self.assertIsNone(Controller.ProtectionTrip, Controller.status)
                Self.assertTrue(Bus.torque_enabled)
                Glitch = [Each for Each in Kinds(Events, "protection_glitch") if Each["details"]["register"] == "Operating_Mode"]
                Self.assertIn("while the control loop was delayed; read again, not confirmed", Glitch[0]["reason"])
                Self.assertEqual(len(Kinds(Events, "control_delayed")), 1)

    def TestPreflightReadsACorruptedPairAgainBeforeRefusing(Self):
        # Review round 2 finding 4: today's exact pair (shoulder_pan status 128 then 129, load 64001, 0 V, 90 C, Lock 28,
        # every joint on target), read by MotionPreflight itself, refused the move, and the app counted the refusal
        # toward ending the task. Now the joint is read again first: clean, so the move may start. A streak that goes on
        # still refuses.
        Controller, Bus = MakeController()
        Events = Recording(Controller)
        ScriptReads(Bus, "shoulder_pan", {"Hardware_Error_Status": [0, 1], "Torque_Enable": [0, 0],
                                          "Present_Load": [64001, 64001], "Present_Voltage": [0, 0],
                                          "Present_Temperature": [90, 90], "Lock": [28, 28]})
        Controller.MotionPreflight()
        Self.assertIsNone(Controller.MotionFault)
        Self.assertEqual(Controller.StatusCorruptStreaks, {})
        Self.assertEqual(len(Kinds(Events, "protection_glitch")), 1, Events)
        Self.assertTrue(Bus.torque_enabled)
        Controller, Bus = MakeController()
        ScriptReads(Bus, "shoulder_pan", Sustain=dict(Corrupt1022))
        Started = RealMonotonic()
        with Self.assertRaises(RuntimeError) as Refused:
            Controller.MotionPreflight()
        Self.assertIn("shoulder_pan status replies read corrupted", str(Refused.exception))
        Self.assertLess(RealMonotonic() - Started, 0.5)
        Self.assertIsNone(Controller.MotionFault)
        Self.assertTrue(Bus.torque_enabled)

    # Review round 3 (each finding confirmed by two skeptics; .review-round1.md has rounds 1-2).

    def Restarting(Self, Bus, Name, State):
        """Name restarts: Torque_Enable reads State["On"] (0 until a goal write energises it), Lock 0, Present_Load 64001
        while State["Corrupt"]."""
        Read, Write = Bus.read, Bus.write

        def Restarted(Reg, Motor, **Options):
            if Motor == Name:
                if Reg == "Torque_Enable":
                    return State["On"]
                if Reg == "Lock":
                    return 0
                if Reg == "Present_Load" and State["Corrupt"]:
                    return 64001
            return Read(Reg, Motor, **Options)

        def Energises(Reg, Motor, Value, **Options):
            if Motor == Name and Reg in ("Goal_Position", "Goal_Speed"):
                State["On"] = 1  # on these servos a goal write turns torque back on
            return Write(Reg, Motor, Value, **Options)
        Bus.read, Bus.write = Restarted, Energises

    def TestARestartThatAnotherGoalWriteEnergisesStillLatchesOnItsLock(Self):
        # Round 3 finding 1: wrist_roll restarts (torque off, Lock 0) under one frame of corrupted telemetry, and a goal
        # write from another path (here Stop and hold; a hold correction or a route's write did the same) energises it
        # before the 0.3 s confirmation, so the next frame reads clean. Before: the streak ended, nothing read Lock again
        # and preflight passed. Now the torque read off on its own re-reads is remembered until its Lock reads 1: it reads
        # 0 twice, so it latches as a restart. Until then preflight refuses and the bridge's own writes skip the joint.
        Controller, Bus = MakeController()
        State = {"On": 0, "Corrupt": True}
        Self.Restarting(Bus, "wrist_roll", State)
        Clock = [Bridge.time.monotonic()]
        Run(Controller, Clock, 1 / 30)
        Self.assertIsNone(Controller.MotionFault, Controller.status)
        Self.assertIn("wrist_roll", Controller.StatusRestartChecks)
        State["Corrupt"] = False
        Self.assertTrue(Controller.StopMotion()[0], Controller.status)  # Stop and hold: its goal writes energise it
        Self.assertEqual(State["On"], 1)
        Self.assertIn("wrist_roll", Controller.StatusRestartChecks, "kept through a Stop that holds")
        Took = Run(Controller, Clock, 1.0)
        Self.assertIn("wrist roll restarted: its torque read off on its own re-reads and its Lock still reads 0",
                      Controller.MotionFault or "", Controller.status)
        Self.assertLess(Took, 0.05)
        Self.assertFalse(Bus.torque_enabled)

    def TestTorqueReadOffThenOnWithLockOneEndsTheRestartCheck(Self):
        # The other side of finding 1: torque read off on its own re-reads (not re-asserted: velocity mode, so its Lock
        # was not read then), then on again with Lock 1 on its own re-reads: no restart, a protection_glitch, the arm holds
        # and a move may start. Meanwhile no goal was written to it.
        Controller, Bus = MakeController()
        Events = Recording(Controller)
        Bus.registers[("Operating_Mode", "shoulder_pan")] = 1
        ScriptReads(Bus, "shoulder_pan", {"Torque_Enable": [0] * 4, "Present_Load": [64001, 64001]})
        Clock = [Bridge.time.monotonic()]
        Run(Controller, Clock, 1 / 30)
        Self.assertIn("shoulder_pan", Controller.StatusRestartChecks)
        Bus.registers[("Operating_Mode", "shoulder_pan")] = 0
        Run(Controller, Clock, 1.0)
        Self.assertIsNone(Controller.MotionFault, Controller.status)
        Self.assertEqual(Controller.StatusRestartChecks, {})
        Self.assertTrue(any(Each["details"]["register"] == "Lock" and "no restart" in Each["reason"]
                            for Each in Kinds(Events, "protection_glitch")), Events)
        Self.assertFalse([Each for Each in Bus.writes if Each[:2] == ("Goal_Position", "shoulder_pan")])
        with patch.object(Bridge.time, "monotonic", lambda: Clock[0]):
            Controller.MotionPreflight()

    def OverheatUnderCorruptStatus(Self, Clean):
        """elbow_flex really at 65 C (Present_Temperature on every read), its status reading corrupted (status 1, load
        64001) on every frame but the frames Clean(N) picks. Returns (Controller, Bus, Clock, Frame)."""
        Controller, Bus = MakeController()
        Read, Frame = Bus.read, [0]

        def Reads(Reg, Name, **Options):
            if Name == "elbow_flex":
                if Reg == "Present_Temperature":
                    return 65
                if not Clean(Frame[0]) and Reg in ("Hardware_Error_Status", "Present_Load"):
                    return 1 if Reg == "Hardware_Error_Status" else 64001
            return Read(Reg, Name, **Options)
        Bus.read = Reads
        return Controller, Bus, [Bridge.time.monotonic()], Frame

    def TestARealOverheatTripsWhileTheStatusReadsCorruptedOnMostFrames(Self):
        # Round 3 finding 2: a steady 65 C (over the 60 C limit) with the status corrupted on every frame but one in
        # Every. Before: every poll fell inside a streak and restarted the count, so it never tripped (30 s). Now only
        # temperatures that look like the corruption (85 C or more in a streak, a re-read that disagrees) are left out.
        for Every in (2, 3, 7):
            with Self.subTest(Every=Every):
                Controller, Bus, Clock, Frame = Self.OverheatUnderCorruptStatus(lambda N: N % Every == 0)
                Took = Run(Controller, Clock, 3.0, lambda: Frame.__setitem__(0, Frame[0] + 1))
                Self.assertIn("elbow_flex temperature 65 C reached software limit 60 C",
                              (Controller.ProtectionTrip or {}).get("reason", ""), Controller.status)
                Self.assertLess(Took, 1.7)
                Self.assertFalse(Bus.torque_enabled)

    def TestTheCapCountsCorruptionOnMostFramesNotJustAnUnbrokenStreak(Self):
        # Round 3 finding 2, the cap: status corrupted on 2 frames in 3 (each clean frame ended the streak, so the 5 s cap
        # never applied) latches past 5 s; one corrupted frame in 8 (most frames clean, a fault would show on them) never.
        Controller, Bus = MakeController()
        Script, Sustain = ScriptReads(Bus, "wrist_flex")
        Frame = [0]

        def Pattern(Every, Corrupted):
            def Each():
                Frame[0] += 1
                Sustain.clear()
                if (Frame[0] % Every == 0) == Corrupted:
                    Sustain.update(Corrupt1022)
            return Each
        Clock = [Bridge.time.monotonic()]
        Run(Controller, Clock, 8.0, Pattern(8, True))
        Self.assertIsNone(Controller.MotionFault, Controller.status)
        Run(Controller, Clock, 1.5, Sustain.clear)  # clean for over StatusCorruptChainGapS: that chain ends
        Self.assertEqual(Controller.StatusCorruptChains, {})
        Run(Controller, Clock, 4.9, Pattern(3, False))
        Self.assertIsNone(Controller.MotionFault, "under the 5 s cap")
        Took = Run(Controller, Clock, 1.0, Pattern(3, False))
        Self.assertRegex(Controller.MotionFault or "", r"Cannot read wrist_flex protection/torque status: its replies read "
                                                       r"corrupted on \d+ of its last \d+ frames, a chain of 5\.\d s \(limit 5 s\)")
        Self.assertLess(Took, 0.3)
        Self.assertFalse(Bus.torque_enabled)

    def TestAFaultReadingPairedWithAnUnansweredOneConfirmsNothing(Self):
        # Round 3 finding 3: one bad byte (torque 0, or the voltage bit) whose telemetry read clean, plus one missed reply,
        # in either order, coasted the arm. Now the pair goes to the streak: torque is read again on its own, a fault bit
        # needs a plausible reading in a later frame.
        NoReply = lambda: ConnectionError("Read Hardware_Error_Status failed: [TxRxResult] There is no status packet! ")
        for Case, Joint, Script in (
                ("torque-off byte, then no reply", "shoulder_pan", {"Torque_Enable": [0], "Hardware_Error_Status": [0, NoReply()]}),
                ("no reply, then a torque-off byte", "shoulder_pan", {"Hardware_Error_Status": [NoReply(), 0], "Torque_Enable": [0]}),
                ("voltage bit, then no reply", "elbow_flex", {"Hardware_Error_Status": [1, NoReply()]})):
            with Self.subTest(Case=Case):
                Controller, Bus = MakeController()
                Events = Recording(Controller)
                Bus.registers[("Present_Voltage", Joint)] = 89
                Bus.registers[("Present_Temperature", Joint)] = 31
                ScriptReads(Bus, Joint, Script)
                Run(Controller, [Bridge.time.monotonic()], 1.0)
                Self.assertIsNone(Controller.MotionFault, Controller.status)
                Self.assertTrue(Bus.torque_enabled)
                Self.assertIn("paired with an unanswered one", Kinds(Events, "protection_glitch")[0]["reason"])

    def TestARealFaultWithAnUnansweredReadingStillLatchesAFrameLater(Self):
        # The other side of finding 3: a real over-voltage, and a real restart (torque off, Lock 0), whose first frame
        # loses one reading: latched in the next frame from its plausible readings.
        NoReply = lambda: ConnectionError("Read Hardware_Error_Status failed: [TxRxResult] There is no status packet! ")
        for Case, Joint, Words in (("over-voltage", "elbow_flex", "elbow flex voltage"),
                                   ("restart", "shoulder_pan", "shoulder pan torque unexpectedly off")):
            with Self.subTest(Case=Case):
                Controller, Bus = MakeController()
                for Register, Value in (("Present_Voltage", 155 if Case == "over-voltage" else 89),
                                        ("Present_Temperature", 33), ("Lock", 1)):
                    Bus.registers[(Register, Joint)] = Value
                Bus.registers[("Hardware_Error_Status", Joint)] = 1 if Case == "over-voltage" else 0
                ScriptReads(Bus, Joint, {"Hardware_Error_Status": [NoReply()]},
                            Sustain={"Torque_Enable": 0, "Lock": 0} if Case == "restart" else None)
                Clock = [Bridge.time.monotonic()]
                Run(Controller, Clock, 1 / 30)
                Self.assertIsNone(Controller.MotionFault)
                Took = Run(Controller, Clock, 1.0)
                Self.assertIn(Words, Controller.MotionFault or "", Controller.status)
                Self.assertLess(Took, 0.05)
                Self.assertFalse(Bus.torque_enabled)

    def TestAControlDelayResumeReadsAnUnansweredTorqueOrStatusReadAgain(Self):
        # Round 3 finding 4: after a held-still control delay one unanswered Torque_Enable, Hardware_Error_Status or
        # Present_Position read coasted the arm ("did not answer"). Now each is read again; failures that the re-reads
        # repeat still trip.
        NoReply = lambda: ConnectionError("no status packet")
        for Case, Script, Trips in (
                ("one unanswered Torque_Enable", {"Torque_Enable": [NoReply()]}, None),
                ("one unanswered Hardware_Error_Status", {"Hardware_Error_Status": [NoReply()]}, None),
                ("one unanswered Present_Position", {"Present_Position": [NoReply()]}, None),
                ("Present_Position unanswered on the re-read too", {"Present_Position": [NoReply(), NoReply()]},
                 "wrist_flex did not answer"),
                ("status unanswered on both fresh readings", {"Torque_Enable": [NoReply()],
                                                              "Hardware_Error_Status": [NoReply()] * 3},
                 "wrist_flex status did not answer")):
            with Self.subTest(Case=Case):
                Controller, Bus = MakeController()
                Events = Recording(Controller)
                Controller.refresh()
                Self.assertTrue(Controller.ArmHoldsStill())
                ScriptReads(Bus, "wrist_flex", Script)
                Controller.last_refresh_s = Bridge.time.monotonic() - 1.5
                Controller.refresh()
                if Trips:
                    Self.assertIn(f"Motor control heartbeat was delayed: {Trips}",
                                  (Controller.ProtectionTrip or {}).get("reason", ""))
                    Self.assertFalse(Bus.torque_enabled)
                    continue
                Self.assertIsNone(Controller.ProtectionTrip, Controller.status)
                Self.assertIsNone(Controller.MotionFault, Controller.status)
                Self.assertTrue(Bus.torque_enabled)
                Self.assertEqual(len(Kinds(Events, "control_delayed")), 1)

    def TestAReassertGoalIsAConfirmedPositionNeverOneCorruptedRead(Self):
        # Round 3 finding 5: the pan's torque reads off on its own re-reads, Lock 1, and this frame's Present_Position
        # reply is corrupted (645 steps off, as 28 read on a pan at 3479). Before: that reading was written as the goal
        # (about 57 deg), and a closed-loop hold then latched "drifted 645 steps". Now the position is re-read on its own
        # twice: they do not agree with the frame's reading, so it is not re-asserted, nor latched as drift; the torque
        # that really stays off latches 0.3 s later. A position that confirms is the goal written.
        for Holding, Corrupted in ((False, True), (True, True), (False, False)):
            with Self.subTest(Holding=Holding, CorruptedPosition=Corrupted):
                Controller, Bus = MakeController()
                Events = Recording(Controller)
                if Holding:
                    for Servo in Controller.servos.values():
                        Servo.Holding = True
                else:
                    Controller.StopMotion()
                Bus.writes.clear()
                Pan = Controller.servos["shoulder_pan"]
                Present = Pan.present_raw
                ScriptReads(Bus, "shoulder_pan", {"Present_Position": [(Present + 645) % 4096] if Corrupted else []},
                            Sustain={"Torque_Enable": 0, "Present_Load": 64001, "Present_Voltage": 0, "Present_Temperature": 90})
                Clock = [Bridge.time.monotonic()]
                Run(Controller, Clock, 1 / 30)
                Self.assertIsNone(Controller.MotionFault, Controller.status)
                Goals = [Each[2] for Each in Bus.writes if Each[:2] == ("Goal_Position", "shoulder_pan")]
                Self.assertFalse([Goal for Goal in Goals if Controller._circular_raw_distance(Goal, Present) > 8], Goals)
                if Corrupted:
                    Self.assertEqual(Goals, [])
                    Run(Controller, Clock, 1.0)
                    Self.assertIn("not re-asserted: its position did not confirm", Controller.MotionFault or "")
                else:
                    Self.assertEqual(Goals, [Present])
                    Self.assertEqual(Kinds(Events, "torque_reasserted")[0]["details"]["goal_raw"], Present)

    def TestAGraspHoldReassertSqueezesFromAConfirmedPosition(Self):
        # Round 3 finding 5, the grasp: a gripper holding an object reads torque off on its own re-reads, and this frame's
        # Present_Position reply is corrupted. Never squeezed toward that reading; with a position that confirms, the
        # squeeze is written from it.
        for Corrupted in (True, False):
            with Self.subTest(CorruptedPosition=Corrupted):
                Controller, Bus = MakeController()
                Gripper = Controller.servos["gripper"]
                Gripper.GraspHold = Gripper.Holding = True
                Present = Gripper.present_raw
                Bus.writes.clear()
                ScriptReads(Bus, "gripper", {"Present_Position": [(Present + 900) % 4096] if Corrupted else [],
                                             "Torque_Enable": [0] * 4, "Present_Load": [64001, 64001]})
                Run(Controller, [Bridge.time.monotonic()], 0.5)
                Self.assertIsNone(Controller.MotionFault, Controller.status)
                Goals = [Each[2] for Each in Bus.writes if Each[:2] == ("Goal_Position", "gripper")]
                Squeeze = Bridge.GRIPPER_GRASP_SQUEEZE_RAW
                Self.assertFalse([Goal for Goal in Goals if Controller._circular_raw_distance(Goal, Present) > Squeeze], Goals)
                Self.assertEqual(bool(Goals), not Corrupted, Goals)
                Self.assertTrue(Gripper.GraspHold)

    def TestStallTripsAtThreeSecondsNotTwo(Self):
        Controller, Bus = MakeController()
        Events = Recording(Controller)
        Servo = Controller.servos["elbow_flex"]
        Controller._send_slider_target_locked(Servo, Servo.present_raw - 200)
        for At, Tripped in ((100.0, False), (102.5, False), (102.99, False), (103.0, True)):
            with patch.object(Bridge.time, "monotonic", return_value=At):
                Controller.CheckMotorProtection()
            Self.assertEqual(Controller.MotionFault is not None, Tripped, At)
        Self.assertEqual(Bridge.ProtectionStallSeconds, 3.0)
        Self.assertEqual(Kinds(Events, "protection_trip")[0]["details"]["stalled_s"], 3.0)
        Self.assertFalse(Bus.torque_enabled)

    # Review round 4: a joint under a pending restart check must never keep moving unsupervised.

    def HoldCorrection(Self, Controller, Name, Offset=200):
        """Name, holding, streams a hold correction toward Offset steps away (velocity mode). Returns its servo."""
        Servo = Controller.servos[Name]
        Servo.Holding = True
        Controller._send_slider_target_locked(Servo, Servo.present_raw + Offset)
        Self.assertNotEqual(Servo.command_speed, 0)
        return Servo

    def TestAHoldCorrectionIntoAPendingRestartCheckStopsAtOnceAndResumesAfter(Self):
        # Before: torque read off on its own re-reads (not re-asserted: it is streaming) and the bridge then skipped every
        # write to it, so a correction already streaming kept its last Goal_Speed, nothing supervising it, until the
        # check ended (up to 1 s). Now, in the very frame the check starts, one Goal_Speed write stops it: 0 when its
        # Operating_Mode reads velocity twice, 1 (the slowest position profile; 1 step/s in velocity mode) when it does not
        # (a restart reverts the mode to position, where 0 is full speed). Nothing else is written to it while its Lock
        # reads neither way (28); once it reads 1 the check ends and the correction resumes from its kept stream.
        for Mode, Stop in ((Bridge.OperatingMode.VELOCITY.value, 0), (Bridge.OperatingMode.POSITION.value, 1)):
            with Self.subTest(ModeReads=Mode):
                Controller, Bus = MakeController()
                Events = Recording(Controller)
                Lift = Self.HoldCorrection(Controller, "shoulder_lift")
                Bus.writes.clear()
                _, Sustain = ScriptReads(Bus, "shoulder_lift", {"Torque_Enable": [0] * 4, "Present_Load": [64001, 64001]},
                                         Sustain={"Lock": 28, "Operating_Mode": Mode})
                Clock = [Bridge.time.monotonic()]
                Run(Controller, Clock, 1 / 30)
                Self.assertIsNone(Controller.MotionFault, Controller.status)
                Self.assertIn("shoulder_lift", Controller.StatusRestartChecks)
                Self.assertEqual([Each for Each in Bus.writes if Each[1] == "shoulder_lift"],
                                 [("Goal_Speed", "shoulder_lift", Stop)])
                Self.assertEqual((Lift.command_speed, Lift.command_raw is not None), (0, True), "its stream is kept")
                Self.assertTrue(any(Each["details"]["register"] == "Goal_Speed" and "restart check is pending" in Each["reason"]
                                    for Each in Kinds(Events, "protection_glitch")), Events)
                Run(Controller, Clock, 0.5)
                Self.assertIsNone(Controller.MotionFault, Controller.status)
                Self.assertEqual([Each for Each in Bus.writes if Each[1] == "shoulder_lift"],
                                 [("Goal_Speed", "shoulder_lift", Stop)], "nothing more while the check is pending")
                Sustain.clear()  # its Lock reads 1 again (and its mode as it is)
                Run(Controller, Clock, 2 / 30)
                Self.assertIsNone(Controller.MotionFault, Controller.status)
                Self.assertEqual(Controller.StatusRestartChecks, {})
                Resumed = [Each for Each in Bus.writes if Each[:2] == ("Goal_Speed", "shoulder_lift")][1:]
                Self.assertTrue(Resumed and Resumed[-1][2] not in (0, 1), Bus.writes)
                Self.assertTrue(Bus.torque_enabled)

    def TestATimedMotionHoldsAtOnceWhenARestartCheckStarts(Self):
        # A route (here a timed stream of the pan) with a joint's torque read off on its own re-reads: before, that joint
        # was skipped, running at its last Goal_Speed if it was the one moving, while the rest went on without it. Now the
        # whole motion holds where it is in that frame (HoldRoute: torque on, a motion_abort, never a fault), the moving
        # joint stopped with Goal_Speed 0 before its hold goal at its present position, and the check is kept.
        for Pending in ("shoulder_pan", "wrist_flex"):
            with Self.subTest(Pending=Pending):
                Controller, Bus = MakeController()
                Events = Recording(Controller)
                Clock = [Bridge.time.monotonic()]
                Pan = Controller.servos["shoulder_pan"]
                Start = Pan.present_raw
                Controller._send_slider_target_locked(Pan, Start + 300)
                Controller.MotionDeadline = Clock[0] + 100
                Bus.writes.clear()
                _, Sustain = ScriptReads(Bus, Pending, {"Torque_Enable": [0] * 4, "Present_Load": [64001, 64001]},
                                         Sustain={"Lock": 28})
                Run(Controller, Clock, 1 / 30)
                Self.assertIsNone(Controller.MotionFault, Controller.status)
                Self.assertIn(Pending, Controller.StatusRestartChecks, "kept through the hold")
                Self.assertEqual(Controller.MotionDeadline, 0.0)
                Self.assertIsNone(Pan.command_raw)
                PanWrites = [Each for Each in Bus.writes if Each[1] == "shoulder_pan"]
                Self.assertEqual(PanWrites[0], ("Goal_Speed", "shoulder_pan", 0), PanWrites)
                Self.assertEqual({Each[2] for Each in PanWrites if Each[0] == "Goal_Position"}, {Start}, "held where it is")
                Self.assertEqual(Bus.reg("shoulder_pan", "Operating_Mode"), Bridge.OperatingMode.POSITION.value)
                Abort = Kinds(Events, "motion_abort")
                Self.assertTrue(Abort and f"{Pending} torque read off on its own re-reads" in Abort[0]["reason"], Events)
                Since = len(Bus.writes)
                Run(Controller, Clock, 0.3)
                Self.assertFalse([Each for Each in Bus.writes[Since:] if Each[0] in ("Goal_Speed", "Goal_Position")])
                Sustain.clear()
                Run(Controller, Clock, 2 / 30)
                Self.assertIsNone(Controller.MotionFault, Controller.status)
                Self.assertEqual(Controller.StatusRestartChecks, {})
                Self.assertTrue(Bus.torque_enabled)

    def TestARealRestartUnderAStoppedHoldCorrectionStillLatchesOnItsLock(Self):
        # wrist_roll restarts (torque off, Lock 0) while streaming a hold correction, its telemetry corrupted. The stop
        # write (Goal_Speed 0: its mode still reads velocity) energises it, as any goal-block write does, so its torque
        # reads on in the next frame; its Lock still reads 0 twice: latched as a restart. Never a goal toward a target.
        Controller, Bus = MakeController()
        Roll = Self.HoldCorrection(Controller, "wrist_roll", 150)
        State = {"On": 0, "Corrupt": True}
        Self.Restarting(Bus, "wrist_roll", State)
        Bus.writes.clear()
        Clock = [Bridge.time.monotonic()]
        Run(Controller, Clock, 1 / 30)
        Self.assertIsNone(Controller.MotionFault, Controller.status)
        Self.assertIn("wrist_roll", Controller.StatusRestartChecks)
        Self.assertEqual([Each for Each in Bus.writes if Each[1] == "wrist_roll"], [("Goal_Speed", "wrist_roll", 0)])
        Self.assertEqual(State["On"], 1, "the stop write energised it")
        Took = Run(Controller, Clock, 1.0)
        Self.assertIn("wrist roll restarted: its torque read off on its own re-reads and its Lock still reads 0",
                      Controller.MotionFault or "", Controller.status)
        Self.assertLess(Took, 0.05)
        Self.assertFalse(Bus.torque_enabled)
        Self.assertFalse([Each for Each in Bus.writes if Each[:2] == ("Goal_Position", "wrist_roll")], Bus.writes)
        Self.assertFalse([Each for Each in Bus.writes if Each[:2] == ("Goal_Speed", "wrist_roll") and Each[2]], Bus.writes)
        Self.assertIsNone(Roll.command_raw)

    def TestTodaysExactSignatureDuringAHoldCorrectionOrARouteNeverLatches(Self):
        # 2026-10-22's signature (status 128 then 129, load 64001, 0 V, 90 C, Lock 28, every joint on its target) while
        # the pan streams: never latched, torque stays on. With 3 torque-off reads the re-read reads on: no check, the
        # motion goes on. With 4 a check starts (a streaming joint is not re-asserted): a hold correction pauses a frame
        # and resumes, a timed motion holds where it is; its Lock then reads 1 and the check ends.
        for Burst in (3, 4):
            for Timed in (False, True):
                with Self.subTest(TorqueReadsOff=Burst, Timed=Timed):
                    Controller, Bus = MakeController()
                    Events = Recording(Controller)
                    Clock = [Bridge.time.monotonic()]
                    Pan = Self.HoldCorrection(Controller, "shoulder_pan", 300)
                    if Timed:
                        Controller.MotionDeadline = Clock[0] + 100
                    ScriptReads(Bus, "shoulder_pan", {"Hardware_Error_Status": [0, 1], "Torque_Enable": [0] * Burst,
                                                      "Present_Load": [64001, 64001], "Present_Voltage": [0, 0],
                                                      "Present_Temperature": [90, 90], "Lock": [28, 28]})
                    Run(Controller, Clock, 1.0)
                    Self.assertIsNone(Controller.MotionFault, Controller.status)
                    Self.assertTrue(Bus.torque_enabled)
                    Self.assertFalse(Kinds(Events, "motion_fault") + Kinds(Events, "protection_trip"))
                    Self.assertEqual(Controller.StatusRestartChecks, {})
                    Held = bool(Kinds(Events, "motion_abort"))
                    Self.assertEqual(Held, Burst == 4 and Timed, Events)
                    Self.assertEqual(Pan.command_raw is not None, not Held, "the motion goes on unless it was held")

    # Review round 5: the route hold round 4 runs in the corrupt burst's own frame reads the pending joint back once; one
    # more garbled read-back (or one failed write) raised "Hold read-back mismatch" and Stop coasted every joint.

    def HeldUnderPendingCheck(Self, Garble=None, FailWrite=None, ModeSticks=False, Joint="shoulder_pan"):
        """A timed stream of shoulder_pan while Joint's torque reads off on its own re-reads (Lock 28): the frame holds the
        motion (HoldRouteKeepingStreaks). Garble (register, values): once the hold has written Joint's Operating_Mode
        position, its next reads of that register answer values. FailWrite (register, times): Joint's next writes of
        that register fail. ModeSticks: Joint never takes Operating_Mode position (it really stays in velocity mode).
        Joint "gripper" holds an object (GraspHold), so its hold is the grasp squeeze.
        Runs that frame; returns (Controller, Bus, Events, Clock, Script, Sustain)."""
        Controller, Bus = MakeController()
        Events = Recording(Controller)
        Clock = [Bridge.time.monotonic()]
        Pan = Controller.servos["shoulder_pan"]
        Controller._send_slider_target_locked(Pan, Pan.present_raw + 300)
        Controller.MotionDeadline = Clock[0] + 100
        if Joint == "gripper":
            Gripper = Controller.servos["gripper"]
            Gripper.GraspHold = Gripper.Holding = True
        Bus.writes.clear()
        Script, Sustain = ScriptReads(Bus, Joint, {"Torque_Enable": [0] * 4, "Present_Load": [64001, 64001]},
                                      Sustain={"Lock": 28})
        Write, Fails = Bus.write, list(FailWrite or ())
        Position = Bridge.OperatingMode.POSITION.value

        def Writing(Reg, Motor, Value, **Options):
            if Motor == Joint and Fails and Reg == Fails[0] and Fails[1]:
                Fails[1] -= 1
                raise RuntimeError("[TxRxResult] Incorrect status packet!")
            if Motor == Joint and Reg == "Operating_Mode" and Value == Position:
                if Garble and Garble[1]:
                    Script[Garble[0]] = list(Garble[1])
                    Garble[1].clear()
                if ModeSticks:
                    Bus.writes.append((Reg, Motor, int(Value)))
                    return
            Write(Reg, Motor, Value, **Options)
        Bus.write = Writing
        Run(Controller, Clock, 1 / 30)
        return Controller, Bus, Events, Clock, Script, Sustain

    def TestOneGarbledHoldReadBackUnderAPendingCheckNeverCoasts(Self):
        # The frame that starts the restart check holds the timed motion, and the pan's hold reads back a garbled goal
        # (42, 28) or mode (33, 28). Before: "Hold read-back mismatch" and every joint coasted. Now that register is read
        # again on its own twice; a re-read that reads what was written stands the hold (a protection_glitch), torque on,
        # the check kept; its Lock then reads 1 and the check ends, the arm holding.
        for Register, Garbled in (("Goal_Position", 42), ("Goal_Position", 28), ("Operating_Mode", 33),
                                  ("Operating_Mode", 28)):
            for Reads in ([Garbled], [Garbled, Garbled]):
                with Self.subTest(Register=Register, Reads=Reads):
                    Controller, Bus, Events, Clock, Script, Sustain = Self.HeldUnderPendingCheck(Garble=(Register, list(Reads)))
                    Self.assertFalse(Script.get(Register), "the garbled read-back was read")
                    Self.assertIsNone(Controller.MotionFault, Controller.status)
                    Self.assertTrue(Bus.torque_enabled)
                    Self.assertIn("shoulder_pan", Controller.StatusRestartChecks)
                    Self.assertEqual(Controller.MotionDeadline, 0.0)
                    Self.assertEqual(Bus.reg("shoulder_pan", "Operating_Mode"), Bridge.OperatingMode.POSITION.value)
                    Glitch = [Each for Each in Kinds(Events, "protection_glitch")
                              if Each["details"]["register"] == Register and "hold read-back" in Each["reason"]]
                    Self.assertEqual([Each["details"]["value"] for Each in Glitch], [Garbled], Events)
                    Sustain.clear()
                    Run(Controller, Clock, 0.5)
                    Self.assertIsNone(Controller.MotionFault, Controller.status)
                    Self.assertEqual(Controller.StatusRestartChecks, {})
                    Self.assertTrue(Bus.torque_enabled)
                    Self.assertFalse(Kinds(Events, "motion_fault") + Kinds(Events, "protection_trip"))

    def TestAHoldReadBackThatBothReReadsRepeatStillCoasts(Self):
        # The same hold, but the register reads wrong on the read-back and both re-reads (garbled each time, or the joint
        # really never took position mode): that is a hold that did not take, so Stop still coasts every joint.
        Cases = [("garbled goal", {"Garble": ("Goal_Position", [42] * 3)}, "Goal_Position read 42, re-read [42, 42]"),
                 ("garbled mode", {"Garble": ("Operating_Mode", [33, 28, 33])}, "Operating_Mode read 33, re-read [28, 33]"),
                 ("really velocity", {"ModeSticks": True}, "Operating_Mode read 1, re-read [1, 1]")]
        for Name, Options, Words in Cases:
            with Self.subTest(Name):
                Controller, Bus, Events, *_ = Self.HeldUnderPendingCheck(**Options)
                Self.assertIn("Hold read-back mismatch", Controller.MotionFault or "", Controller.status)
                Self.assertIn(Words, Controller.MotionFault)
                Self.assertFalse(Bus.torque_enabled)
                Self.assertTrue(Kinds(Events, "motion_fault"))

    def TestOneFailedHoldWriteUnderAPendingCheckIsWrittenAgainTwoCoast(Self):
        # One write of the pan's hold fails (a garbled status reply): it is written once more and the hold stands (a
        # protection_glitch). Failing twice in a row is a hold that did not take: Stop coasts, as before.
        for Register in ("Goal_Position", "Operating_Mode", "Goal_Speed"):
            with Self.subTest(Register=Register, Fails=1):
                Controller, Bus, Events, *_ = Self.HeldUnderPendingCheck(FailWrite=(Register, 1))
                Self.assertIsNone(Controller.MotionFault, Controller.status)
                Self.assertTrue(Bus.torque_enabled)
                Self.assertEqual(Bus.reg("shoulder_pan", "Operating_Mode"), Bridge.OperatingMode.POSITION.value)
                Self.assertTrue([Each for Each in Kinds(Events, "protection_glitch") if Each["details"]["register"] == Register
                                 and "written again" in Each["reason"]], Events)
            with Self.subTest(Register=Register, Fails=2):
                Controller, Bus, Events, *_ = Self.HeldUnderPendingCheck(FailWrite=(Register, 2))
                Self.assertIn("stop unconfirmed", Controller.MotionFault or "", Controller.status)
                Self.assertFalse(Bus.torque_enabled)

    # Review round 6 (.review-round5.md): the last confirmed findings of round 5's review.

    def TestAJointReallyInVelocityModeStillCoastsWithOneReReadGarbledToPosition(Self):
        # Finding 1: the pan never takes position mode (it really stays in velocity mode) and the burst garbles one of the
        # two re-reads of its mode read-back to 0 (position): reads [1, 0, 1] or [1, 1, 0]. Before, a re-read reading
        # position stood the hold, and the hold then wrote Goal_Speed = the profile speed to a joint in velocity mode,
        # nothing supervising it after. Now a velocity read among the re-reads (or a first read of velocity that both
        # re-reads do not contradict) is a hold that did not take: Stop coasts, and no speed above 1 is ever written.
        for Reads in ([1, 0], [1, 1, 0]):
            with Self.subTest(Reads=Reads):
                Controller, Bus, Events, *_ = Self.HeldUnderPendingCheck(Garble=("Operating_Mode", list(Reads)),
                                                                         ModeSticks=True)
                Self.assertIn("Hold read-back mismatch", Controller.MotionFault or "", Controller.status)
                Self.assertIn("Operating_Mode read 1", Controller.MotionFault)
                Self.assertFalse(Bus.torque_enabled)
                Self.assertFalse([Each for Each in Bus.writes if Each[:2] == ("Goal_Speed", "shoulder_pan") and Each[2] > 1],
                                 Bus.writes)
        # A position-mode pan whose mode read-back is garbled to velocity once (reads [1, 0, 0]) still holds.
        Controller, Bus, Events, *_ = Self.HeldUnderPendingCheck(Garble=("Operating_Mode", [1]))
        Self.assertIsNone(Controller.MotionFault, Controller.status)
        Self.assertTrue(Bus.torque_enabled)
        Self.assertTrue([Each for Each in Kinds(Events, "protection_glitch") if Each["details"]["register"] == "Operating_Mode"
                         and Each["details"]["reads"] == [1, 0, 0]], Events)

    def HomeOnFakeBus(Self):
        """test_bridge_home's arm displaced from Home and Home started (the fake bus steps its servos each frame).
        Returns (Controller, Bus, Events, Frames), Frames(Count) running Count frames on a simulated clock."""
        Controller, Bus = HomeController()
        Displace(Controller, Bus)
        Clock = [1000.0]
        Self.enterContext(patch.object(Bridge.time, "monotonic", lambda: Clock[0]))
        Controller.last_refresh_s = Clock[0]
        Ok, Text = Controller.go_home()
        Self.assertTrue(Ok, Text)
        Events = Recording(Controller)

        def Frames(Count):
            for _ in range(Count):
                for Name in Bridge.MOTOR_NAMES:
                    Bus.step(Name)
                Clock[0] += 1 / 30
                Controller.refresh()
                if Controller.MotionFault:
                    break
        Frames(4)
        Self.assertIsNotNone(Controller.MotionRoute)
        return Controller, Bus, Events, Frames

    def TestOneGarbledPositionReadMidRouteNeverCoasts(Self):
        # Finding 2: one Present_Position reply of the pan garbled mid-Home (well off its route). Before, CheckMotionRoute
        # latched "shoulder_pan left the checked basket route" on that one read and coasted every joint. Now its position
        # is read again on its own twice; re-reads on the route are a protection_glitch, the confirmed one kept, and Home
        # goes on. A second garbled read among the re-reads changes nothing while one re-read is on the route.
        for Garbled, Reads in ((2090, [2090]), (1500, [1500]), (2090, [2090, 2090 + 200])):
            with Self.subTest(Reads=Reads):
                Controller, Bus, Events, Frames = Self.HomeOnFakeBus()
                ScriptReads(Bus, "shoulder_pan", {"Present_Position": list(Reads)})
                Frames(1)
                Self.assertIsNone(Controller.MotionFault, Controller.status)
                Self.assertIsNotNone(Controller.MotionRoute, "Home goes on")
                Self.assertEqual(Controller.servos["shoulder_pan"].present_raw, Bus.position["shoulder_pan"], "re-read kept")
                Glitch = [Each for Each in Kinds(Events, "protection_glitch") if Each["details"]["register"] == "Present_Position"]
                Self.assertEqual([Each["details"]["value"] for Each in Glitch], [Garbled], Events)
                Frames(400)
                Self.assertIsNone(Controller.MotionFault, Controller.status)
                Self.assertIsNone(Controller.MotionRoute, "Home finished")
                Self.assertTrue(Bus.torque_enabled)

    def TestAPositionOffTheRouteThatBothReReadsConfirmStillLatches(Self):
        # The pan really is off its route (knocked 1000 steps away; the re-reads agree): latched and coasted, as before.
        # Re-reads that agree with neither each other nor the route (no confirmed position to hold at) latch too.
        for Name, Move in (("really off", True), ("unconfirmed", False)):
            with Self.subTest(Name):
                Controller, Bus, Events, Frames = Self.HomeOnFakeBus()
                if Move:
                    Bus.position["shoulder_pan"] = 2090
                else:
                    ScriptReads(Bus, "shoulder_pan", {"Present_Position": [2090, 1500, 2500]})
                Frames(1)
                Self.assertIn("shoulder_pan left the checked basket route", Controller.MotionFault or "", Controller.status)
                Self.assertFalse(Bus.torque_enabled)
                Fault = Kinds(Events, "motion_fault")[0]["details"]
                Self.assertEqual(Fault["reads"], [Bus.position["shoulder_pan"]] * 2 if Move else [1500, 2500], Fault)

    def TestABurstFrameWithAGarbledPositionHoldsAtTheConfirmedPosition(Self):
        # Finding 2 in the burst's own frame: today's 4-read burst on the pan mid-Home (a restart check starts, Home
        # holds), that frame's Present_Position read garbled too. Before: latched off the route before the hold ran. Now
        # the hold runs, at the re-read position (never toward the garbled read), and its Lock then reads 1.
        Controller, Bus, Events, Frames = Self.HomeOnFakeBus()
        _, Sustain = ScriptReads(Bus, "shoulder_pan", {"Present_Position": [2090], "Hardware_Error_Status": [0, 1],
                                                       "Torque_Enable": [0] * 4, "Present_Load": [64001, 64001],
                                                       "Present_Voltage": [0, 0], "Present_Temperature": [90, 90]},
                                 Sustain={"Lock": 28})
        Bus.writes.clear()
        Frames(1)
        Self.assertIsNone(Controller.MotionFault, Controller.status)
        Self.assertIsNone(Controller.MotionRoute, "Home held where it is")
        Self.assertIn("shoulder_pan", Controller.StatusRestartChecks)
        Goals = {Each[2] for Each in Bus.writes if Each[:2] == ("Goal_Position", "shoulder_pan")}
        Self.assertEqual(Goals, {Bus.position["shoulder_pan"]}, "held at the confirmed position")
        Sustain.clear()
        Frames(15)
        Self.assertIsNone(Controller.MotionFault, Controller.status)
        Self.assertEqual(Controller.StatusRestartChecks, {})
        Self.assertTrue(Bus.torque_enabled)
        Self.assertFalse(Kinds(Events, "motion_fault") + Kinds(Events, "protection_trip"))

    def TestAGraspingGripperUnderAPendingCheckToleratesOneGarbledHoldReadBack(Self):
        # Findings 3 and 4: the gripper holds an object (GraspHold) during a timed motion and gets today's burst (torque
        # read off 4 times, load 64001, Lock 28): the frame holds the motion, and the gripper's hold goes through the grasp
        # squeeze. Before, the squeeze read back strictly once: one garbled goal or mode read-back raised and coasted every
        # joint mid-carry, dropping the object. Now it is tolerant like the other suspect joints: the grasp is kept, the
        # check ends once its Lock reads 1. The same read-back garbled on all three reads still coasts.
        for Register, Garbled in (("Goal_Position", 42), ("Goal_Position", 28), ("Operating_Mode", 33), ("Operating_Mode", 28)):
            with Self.subTest(Register=Register, Garbled=Garbled):
                Controller, Bus, Events, Clock, Script, Sustain = Self.HeldUnderPendingCheck(Garble=(Register, [Garbled]),
                                                                                             Joint="gripper")
                Gripper = Controller.servos["gripper"]
                Self.assertFalse(Script.get(Register), "the garbled read-back was read")
                Self.assertIsNone(Controller.MotionFault, Controller.status)
                Self.assertTrue(Bus.torque_enabled)
                Self.assertEqual(Controller.MotionDeadline, 0.0)
                Self.assertTrue(Gripper.GraspHold, "still squeezing the object")
                Goal = Bus.reg("gripper", "Goal_Position")
                Self.assertEqual(Controller._circular_raw_distance(Goal, Gripper.present_raw), Bridge.GRIPPER_GRASP_SQUEEZE_RAW)
                Self.assertTrue([Each for Each in Kinds(Events, "protection_glitch") if Each["details"]["register"] == Register
                                 and "hold read-back" in Each["reason"]], Events)
                Sustain.clear()
                Run(Controller, Clock, 0.5)
                Self.assertIsNone(Controller.MotionFault, Controller.status)
                Self.assertEqual(Controller.StatusRestartChecks, {})
                Self.assertTrue(Gripper.GraspHold)
                Self.assertFalse(Kinds(Events, "motion_fault") + Kinds(Events, "protection_trip"))
        Controller, Bus, *_ = Self.HeldUnderPendingCheck(Garble=("Goal_Position", [42] * 3), Joint="gripper")
        Self.assertIn("gripper Hold read-back mismatch", Controller.MotionFault or "", Controller.status)
        Self.assertFalse(Bus.torque_enabled)


class ReplayTests(unittest.TestCase):
    """The 2026-10-12 trips replayed on the real driver over FakeServoSerial, the arm holding (plain Hold)."""

    def setUp(Self):
        Self.Device = Self.enterContext(Installed(FakeServoDevice(Positions1632)))
        Self.Clock = [1000.0]
        Self.enterContext(patch.object(Bridge.time, "monotonic",
                                       lambda: RealMonotonic() if Self.Clock[0] is None else Self.Clock[0]))
        Self.Controller = Bridge.BridgeController()
        Self.Events = Recording(Self.Controller)
        Ok, Text = Self.Controller.connect(FakePort)
        Self.assertTrue(Ok, Text)
        if Self.Controller.torque_enabled:  # connect's Goal_Position seed energises the servos
            Self.assertTrue(Self.Controller.toggle_torque()[0], Self.Controller.status)
        Self.assertTrue(Self.Controller.toggle_torque()[0], Self.Controller.status)
        Self.Frames(0.5)
        Self.assertIsNone(Self.Controller.MotionFault, Self.Controller.status)
        Self.Events.clear()

    def tearDown(Self):
        Self.assertEqual(HardwareCalls, [], "an offline test reached the depth camera or process priority")

    def Frames(Self, Seconds):
        for _ in range(round(Seconds * 30)):
            Self.Clock[0] += 1 / 30
            Self.Device.Step(1 / 30)
            Self.Controller.refresh()

    def AssertHolding(Self):
        Self.assertIsNone(Self.Controller.MotionFault, Self.Controller.status)
        Self.assertIsNone(Self.Controller.ProtectionTrip)
        Self.assertEqual(Self.Device.Torque(), dict.fromkeys(Positions1632, 1))

    def AssertCoasted(Self, Words):
        Self.assertIn(Words, Self.Controller.MotionFault or "")
        Self.assertEqual(Self.Device.Torque(), dict.fromkeys(Positions1632, 0))

    def TestTheMissedTemperatureReplyOf1617NoLongerCoasts(Self):
        # Both tries of one poll's Present_Temperature from shoulder_pan go unanswered, as at 16:17:25.
        Left = MissReplies(Self.Device, Ids["shoulder_pan"], 63, 2)
        Self.Frames(0.3)
        Self.assertEqual(Left[0], 0, "the poll ran")
        Self.AssertHolding()
        Glitch = Kinds(Self.Events, "protection_glitch")
        Self.assertEqual(len(Glitch), 1, Self.Events)
        Self.assertIn("There is no status packet", Glitch[0]["details"]["error"])
        Self.Frames(1.0)
        Self.AssertHolding()
        # A servo whose temperature stays unanswered (two polls, every try) still coasts, about 0.25 s later.
        MissReplies(Self.Device, Ids["shoulder_pan"], 63, 4)
        Self.Frames(0.6)
        Self.AssertCoasted("Lost shoulder_pan protection feedback")
        Self.assertIn("2 polls in a row", Self.Controller.MotionFault)

    def TestTheCorruptedTorqueOffReplyOf1632NoLongerCoasts(Self):
        # One reading's torque, load, supply and temperature replies as recorded at 16:32:43; the re-read is clean.
        Left = CorruptReads(Self.Device, Ids["shoulder_lift"], Reply1632, 4)
        Self.Frames(1 / 30)
        Self.assertEqual(Left[0], 0)
        Self.AssertHolding()
        Glitch = Kinds(Self.Events, "protection_glitch")[0]
        Self.assertEqual((Glitch["joint"], Glitch["details"]["register"], Glitch["details"]["value"]),
                         ("shoulder_lift", "Torque_Enable", 0))
        Self.assertEqual((Glitch["details"]["load"], Glitch["details"]["voltage_dv"], Glitch["details"]["temperature_c"]),
                         (63745, 56, 90))
        Self.assertIn("beyond the register", Glitch["details"]["impossible"])
        Self.assertFalse(Kinds(Self.Events, "motion_fault"))
        Self.Frames(1.0)
        Self.AssertHolding()

    def TestTheCorruptReplyTwiceInARowOf1022NoLongerCoasts(Self):
        # 2026-10-22: two readings in a row that no servo could give (load 64001, 0 V, 90 C), every joint on its target.
        # A protection_glitch, not a fault: the arm holds.
        Reply1022 = {40: b"\x00", 60: (64001).to_bytes(2, "little"), 62: bytes([0]), 63: bytes([90])}
        Left = CorruptReads(Self.Device, Ids["shoulder_pan"], Reply1022, 8)
        Self.Frames(1 / 30)
        Self.assertEqual(Left[0], 0, "both readings were corrupted")
        Self.AssertHolding()
        Self.assertFalse(Kinds(Self.Events, "motion_fault"))
        Glitch = Kinds(Self.Events, "protection_glitch")
        Self.assertEqual(len(Glitch), 1, Self.Events)
        Self.assertEqual((Glitch[0]["joint"], Glitch[0]["details"]["load"], Glitch[0]["details"]["temperature_c"]),
                         ("shoulder_pan", 64001, 90))
        Self.assertIn("corrupted status reply", Glitch[0]["reason"])
        # Corrupted on every status reading for 3 s (under the 5 s cap), the arm still on target, torque reading on on its
        # own re-reads: still holding, one glitch for the whole streak (it follows on in the next frame). (A Torque_Enable
        # that reads off on every read, also after a goal write, is a torque loss:
        # TestTorqueThatReadsOffOnEveryReadEvenAfterAGoalWriteLatches.)
        CorruptReads(Self.Device, Ids["shoulder_pan"], StatusBit1022, 10 ** 6)
        Self.Frames(3.0)
        Self.AssertHolding()
        Self.assertEqual(len(Kinds(Self.Events, "protection_glitch")), 1, Self.Events)

    def TestTodaysExactSignatureOnTheRealDriverNeverLatches(Self):
        # 2026-10-22 as recorded: shoulder_pan status 128 then 129, load 64001, 0 V, 90 C, Lock 28, every joint on target;
        # the burst takes the first independent Torque_Enable re-read too, then both (so the torque is re-asserted).
        Pan = Ids["shoulder_pan"]
        for Burst in (3, 4):
            Since = len(Self.Device.Received)
            Script = ScriptedReads(Self.Device, Pan, {65: [b"\x00", b"\x01"], 40: [b"\x00"] * Burst,
                                                      60: [(64001).to_bytes(2, "little")] * 2, 62: [b"\x00"] * 2,
                                                      63: [bytes([90])] * 2, 55: [bytes([28])] * 2})
            Self.Frames(2.0)
            Self.assertFalse(any(Script.values()), Script)
            Self.AssertHolding()
            Self.assertFalse(Kinds(Self.Events, "motion_fault"))
            Reasserted = [Write for Write in Self.Device.Writes(42, Since) if Write[0] == Pan]
            Self.assertEqual(Reasserted[:1], [] if Burst == 3 else [(Pan, 42, Positions1632[Pan])])
        Glitches = Kinds(Self.Events, "protection_glitch")
        Self.assertEqual(len(Glitches), 2, Self.Events)
        Self.assertEqual((Glitches[0]["details"]["bits"], Glitches[0]["details"]["load"], Glitches[0]["details"]["voltage_dv"],
                          Glitches[0]["details"]["temperature_c"], Glitches[0]["details"]["lock"]), (129, 64001, 0, 90, 28))
        Self.assertEqual([Each["details"]["torque_after"] for Each in Kinds(Self.Events, "torque_reasserted")], [1])

    def TestTorqueThatReadsOffOnEveryReadEvenAfterAGoalWriteLatches(Self):
        # Torque_Enable 0 on every read (the status readings, two re-reads on its own, and after a goal write at its
        # present position) with impossible telemetry: indistinguishable from a torque loss, so latched past 0.3 s.
        CorruptReads(Self.Device, Ids["shoulder_pan"], StatusOnly1022, 10 ** 6)
        Self.Frames(0.25)
        Self.AssertHolding()
        Self.Frames(0.2)
        Self.AssertCoasted("shoulder pan torque unexpectedly off: read off on its own re-reads for 0.3 s, also after a goal write")

    def Limp(Self, Lift, Sag):
        """From now on the joint Lift has no torque whatever is written, and falls 150 steps a second (up to 150)."""
        def Limp(Device, Seconds):
            Device.Tables[Lift][40] = 0
            Sag[0] = min(150, Sag[0] + 150 * Seconds)
            Device.SetPosition(Lift, Positions1632[Lift] - round(Sag[0]))
        Self.Device.OnStep = Limp

    def TestSustainedCorruptStatusWithASaggingJointLatches(Self):
        # Torque really off, but every status reply and torque read corrupted to read it on: only the position shows it.
        # It falls off its hold point: latched once it is 30 steps further than when the streak began (review round 1
        # finding 9: judged against where the streak began, above the 12-step correction deadband), in a fraction of a
        # second rather than past 2 s.
        Lift = Ids["shoulder_lift"]
        CorruptReads(Self.Device, Lift, {**StatusBit1022, 40: b"\x01"}, 10 ** 6)
        Self.Frames(1.0)
        Self.AssertHolding()
        Sag = [0]
        Self.Limp(Lift, Sag)
        Self.Frames(0.15)
        Self.assertIsNone(Self.Controller.MotionFault, "under 30 steps yet")
        Self.Frames(0.25)
        Self.AssertCoasted("Cannot read shoulder_lift protection/torque status and it lost hold")
        Fault = Kinds(Self.Events, "motion_fault")[0]["details"]
        Self.assertGreater(Fault["away_raw"], Bridge.StatusCorruptHoldDriftRaw)
        Self.assertLess(Sag[0], 75)
        Self.assertEqual(Fault["telemetry"]["load"], 64001)

    def TestARealTorqueLossUnderCorruptTelemetryLatchesWithinAFraction(Self):
        # Torque really off (its register reads 0) and the telemetry impossible: the re-assert brings it on for a
        # moment, it drops again, and it latches within half a second (its torque or its lost hold, whichever first).
        Lift = Ids["shoulder_lift"]
        CorruptReads(Self.Device, Lift, StatusBit1022, 10 ** 6)
        Self.Frames(0.5)
        Self.AssertHolding()
        Sag = [0]
        Self.Limp(Lift, Sag)
        Self.Frames(0.5)
        Self.assertEqual(Self.Device.Torque(), dict.fromkeys(Positions1632, 0))
        Self.assertTrue(any(Words in (Self.Controller.MotionFault or "") for Words in
                            ("shoulder lift torque unexpectedly off", "shoulder_lift protection/torque status and it lost hold")),
                        Self.Controller.MotionFault)

    def TestAConfirmedTorqueOffLatchesInTheSameFrame(Self):
        # Torque really off (a plausible 9.0 V, 30 C servo): the re-read agrees and the fault latches at once.
        Self.Device.Tables[Ids["elbow_flex"]][40] = 0
        Self.Frames(1 / 30)
        Self.AssertCoasted("elbow flex torque unexpectedly off")
        Fault = Kinds(Self.Events, "motion_fault")[0]["details"]["faults"][0]
        Self.assertEqual((Fault["servo"], Fault["bits"], Fault["voltage_dv"], Fault["temperature_c"]), ("elbow_flex", 128, 90, 30))
        Self.assertFalse(Kinds(Self.Events, "protection_glitch"))

    def TestABrownOutResetIsCaughtInTheNextFrame(Self):
        # A real restart: torque off, Lock 0, the goal where it is. Nothing writes between the two readings, so the
        # re-read still reads it off: latched in the first frame after, with Lock 0 in the record.
        Self.Device.Restart(Ids["shoulder_lift"])
        Self.Frames(1 / 30)
        Self.AssertCoasted("shoulder lift torque unexpectedly off")
        Fault = Kinds(Self.Events, "motion_fault")[0]["details"]["faults"][0]
        Self.assertEqual((Fault["servo"], Fault["lock"]), ("shoulder_lift", 0))

    def TestARestartUnderOneFrameOfCorruptTelemetryLatches(Self):
        # Review round 2 finding 1 on the real driver: shoulder_lift restarts (brown-out: torque off, Lock 0) and that
        # frame's two status readings carry load 64001. Never re-asserted (a goal write would energise it, Lock left
        # open): its next plausible pair latches, a frame later, the arm coasted where it held.
        Lift = Ids["shoulder_lift"]
        Before = len(Self.Device.Received)
        Self.Device.Restart(Lift)
        CorruptReads(Self.Device, Lift, {60: (64001).to_bytes(2, "little")}, 2)
        Self.Frames(3 / 30)
        Self.AssertCoasted("shoulder lift torque unexpectedly off")
        Self.assertFalse([Each for Each in Self.Device.Writes(42, Before) if Each[0] == Lift], "no goal write re-asserted it")
        Self.assertFalse(Kinds(Self.Events, "torque_reasserted"))
        Self.assertEqual(len(Kinds(Self.Events, "protection_glitch")), 1, Self.Events)

    def TestAnOverloadBitTwiceLatchesOnceDoesNot(Self):
        Lift = Ids["shoulder_lift"]
        CorruptReads(Self.Device, Lift, {65: bytes([32])}, 1)
        Self.Frames(1 / 30)
        Self.AssertHolding()
        Self.assertEqual(Kinds(Self.Events, "protection_glitch")[0]["details"]["value"], 32)
        CorruptReads(Self.Device, Lift, {65: bytes([32])}, 2)
        Self.Frames(1 / 30)
        Self.AssertCoasted("shoulder lift overload")

    def SagRestart(Self, Readings, Rate):
        """Review round 3 finding 1 on the real driver: shoulder_lift restarts and falls Rate steps a second while its
        torque is off, its status telemetry corrupted (load 64001) on Readings readings. Before: the sag brought a
        hold-correction goal write that energised it before the 0.3 s confirmation, the next frame read clean, and the
        restart (Lock 0) was never latched; preflight passed. Now nothing is written to it while its torque read off on
        its own re-reads and its Lock has not read 1: latched, never energised."""
        Lift = Ids["shoulder_lift"]
        Before = len(Self.Device.Received)
        Self.Device.Restart(Lift)
        CorruptReads(Self.Device, Lift, {60: (64001).to_bytes(2, "little")}, Readings)

        def Gravity(Device, Seconds):
            if not Device.Tables[Lift][40]:  # torque off: it falls
                Device.SetPosition(Lift, Device.Position(Lift) - round(Rate * Seconds))
        Self.Device.SetPosition(Lift, Positions1632[Lift] - round(Rate / 30))
        Self.Device.OnStep = Gravity
        Self.Frames(1.0)
        Self.AssertCoasted("shoulder lift")
        Self.assertFalse([Each for Each in Self.Device.Writes(42, Before) if Each[0] == Lift], "never energised")
        Self.assertEqual((Self.Device.Register(Lift, 55), Self.Device.Register(Lift, 40)), (0, 0))

    def TestARestartSaggingFastUnderOneCorruptedReadingLatches(Self):
        Self.SagRestart(1, 600)

    def TestARestartSaggingUnderTwoCorruptedFramesLatches(Self):
        Self.SagRestart(4, 150)  # 4 readings: two frames, the 2026-10-22 burst; 150 steps/s as Limp

    def TestARestartSaggingFasterUnderTwoCorruptedFramesLatches(Self):
        Self.SagRestart(4, 200)

    def MidHomeRestart(Self, Joint):
        """Round 3 finding 1 on a route: Joint restarts 0.4 s into Home, one status reading of it corrupted. Before: the
        route's own writes energised it, the streak ended, and Home carried on with Lock 0. Now the route's writes skip it
        and it latches: Home ends, its Lock stays 0 and its torque off."""
        Id = Ids[Joint]
        Ok, Text = Self.Controller.go_home()
        Self.assertTrue(Ok, Text)
        Self.Frames(0.4)
        Self.assertIsNotNone(Self.Controller.MotionRoute)
        Self.Device.Restart(Id)
        CorruptReads(Self.Device, Id, {60: (64001).to_bytes(2, "little")}, 1)
        Self.Frames(1.0)
        Self.AssertCoasted(Joint.replace("_", " "))
        Self.assertIsNone(Self.Controller.MotionRoute)
        Self.assertEqual((Self.Device.Register(Id, 55), Self.Device.Register(Id, 40)), (0, 0))

    def TestAPanRestartMidHomeUnderCorruptTelemetryLatches(Self):
        Self.MidHomeRestart("shoulder_pan")

    def TestAnElbowRestartMidHomeUnderCorruptTelemetryLatches(Self):
        Self.MidHomeRestart("elbow_flex")

    # Review round 4 on the real driver: a joint under a pending restart check never keeps moving unsupervised.

    def PendingCheckOn(Self, Joint, LockFrames):
        """Joint's torque reads off on the frame's pair and both its own re-reads (load 64001), its Lock 28 for LockFrames
        frames after."""
        ScriptedReads(Self.Device, Ids[Joint], {40: [b"\x00"] * 4, 60: [(64001).to_bytes(2, "little")] * 2,
                                                55: [bytes([28])] * (2 + 2 * LockFrames)})

    def TestAVelocityHoldCorrectionStopsAtOnceUnderAPendingCheckAndResumes(Self):
        # The probe's case (R4SkipVelocity) on a hold correction: shoulder_lift streams back to its hold point in
        # velocity mode; its torque reads off on its own re-reads, and its Lock reads 28 for 10 frames. Before, it ran
        # on at its last Goal_Speed for all 10. Now Goal_Speed reads 0 on the servo after that very frame, the joint
        # does not move while the check is pending, and the correction resumes once its Lock reads 1.
        Lift = Ids["shoulder_lift"]
        Servo = Self.Controller.servos["shoulder_lift"]
        with Self.Controller.lock:
            Servo.Holding = True
            Self.Controller._send_slider_target_locked(Servo, Servo.present_raw + 150, num_retry=1)
        Self.Frames(3 / 30)
        Self.assertEqual(Self.Device.Register(Lift, 33), 1)
        Self.assertNotEqual(Self.Device.Register(Lift, 46, 2), 0)
        Self.PendingCheckOn("shoulder_lift", 10)
        Self.Frames(1 / 30)
        Self.assertIn("shoulder_lift", Self.Controller.StatusRestartChecks)
        Self.assertEqual(Self.Device.Register(Lift, 46, 2), 0)
        Stopped = Self.Device.Position(Lift)
        for _ in range(8):
            Self.Frames(1 / 30)
            Self.assertIn("shoulder_lift", Self.Controller.StatusRestartChecks)
            Self.assertEqual(Self.Device.Position(Lift), Stopped)
        Self.Frames(0.5)
        Self.AssertHolding()
        Self.assertEqual(Self.Controller.StatusRestartChecks, {})
        Self.assertNotEqual(Self.Device.Position(Lift), Stopped, "the correction resumed")
        Self.assertFalse(Kinds(Self.Events, "motion_fault"))

    def TestARouteHoldsAtOnceUnderAPendingCheck(Self):
        # The probe's case itself: shoulder_lift mid-Home at -315 steps/s. Before, it ran on unsupervised for the 10
        # frames its Lock read 28 (about 100 steps), and Home went on. Now Home holds where it is in that frame (torque on,
        # no fault), no joint moves while the check is pending, and its Lock reading 1 ends the check, the arm holding.
        Lift = Ids["shoulder_lift"]
        Ok, Text = Self.Controller.go_home()
        Self.assertTrue(Ok, Text)
        Self.Frames(12 / 30)
        Self.assertIsNotNone(Self.Controller.MotionRoute)
        Self.assertNotEqual(Self.Controller.servos["shoulder_lift"].command_speed, 0)
        Self.PendingCheckOn("shoulder_lift", 10)
        Self.Frames(1 / 30)
        Self.assertIn("shoulder_lift", Self.Controller.StatusRestartChecks)
        Self.assertIsNone(Self.Controller.MotionRoute)
        Self.assertEqual(Self.Device.Register(Lift, 33), 0, "held in position mode")
        Held = {Id: Self.Device.Position(Id) for Id in Positions1632}
        for _ in range(8):
            Self.Frames(1 / 30)
            Self.assertEqual({Id: Self.Device.Position(Id) for Id in Positions1632}, Held)
        Self.Frames(0.5)
        Self.AssertHolding()
        Self.assertEqual(Self.Controller.StatusRestartChecks, {})
        Self.assertEqual({Id: Self.Device.Position(Id) for Id in Positions1632}, Held)
        Self.assertTrue(Kinds(Self.Events, "motion_abort"))
        Self.assertFalse(Kinds(Self.Events, "motion_fault"))

    def TestARestartUnderAVelocityHoldCorrectionStillLatches(Self):
        # A real restart (torque off, Lock 0) of shoulder_lift while it streams a hold correction, one status reading
        # corrupted. The stop write energises it at speed 0 (still); its Lock still reads 0: latched, coasted, Lock 0.
        Lift = Ids["shoulder_lift"]
        Servo = Self.Controller.servos["shoulder_lift"]
        with Self.Controller.lock:
            Servo.Holding = True
            Self.Controller._send_slider_target_locked(Servo, Servo.present_raw + 150, num_retry=1)
        Self.Frames(3 / 30)
        Before = len(Self.Device.Received)
        Self.Device.Restart(Lift)
        CorruptReads(Self.Device, Lift, {60: (64001).to_bytes(2, "little")}, 1)
        Self.Frames(1.0)
        Self.AssertCoasted("shoulder lift")
        Self.assertEqual((Self.Device.Register(Lift, 55), Self.Device.Register(Lift, 40)), (0, 0))
        Self.assertFalse([Each for Each in Self.Device.Writes(42, Before) if Each[0] == Lift], "no goal toward a target")
        Self.assertFalse([Each for Each in Self.Device.Writes(46, Before) if Each[0] == Lift and Each[2]], "no speed but 0")

    def TestTodaysExactSignatureMidHomeNeverLatches(Self):
        # 2026-10-22's signature on shoulder_pan while Home runs. 3 torque-off reads: the re-read reads on, Home goes on.
        # 4: a check starts (a streaming joint is not re-asserted), so Home holds where it is; its Lock reads 1 and the
        # check ends. Never latched, every joint's torque on.
        Pan = Ids["shoulder_pan"]
        for Burst in (3, 4):
            with Self.subTest(TorqueReadsOff=Burst):
                Self.Events.clear()
                Ok, Text = Self.Controller.go_home()
                Self.assertTrue(Ok, Text)
                Self.Frames(6 / 30)
                Self.assertIsNotNone(Self.Controller.MotionRoute)
                Script = ScriptedReads(Self.Device, Pan, {65: [b"\x00", b"\x01"], 40: [b"\x00"] * Burst,
                                                          60: [(64001).to_bytes(2, "little")] * 2, 62: [b"\x00"] * 2,
                                                          63: [bytes([90])] * 2, 55: [bytes([28])] * 2})
                Self.Frames(1 / 30)
                Self.assertFalse(any(Script.values()), Script)
                Self.assertEqual(Self.Controller.MotionRoute is None, Burst == 4)
                Self.Frames(0.5)
                Self.AssertHolding()
                Self.assertEqual(Self.Controller.StatusRestartChecks, {})
                Self.assertFalse(Kinds(Self.Events, "motion_fault") + Kinds(Self.Events, "protection_trip"))
                Self.assertEqual(bool(Kinds(Self.Events, "motion_abort")), Burst == 4)
                with Self.Controller.lock:
                    Self.Controller.StopMotion()

    # Review round 5 on the real driver: the hold that round 4 runs in the burst's own frame tolerates one garbled read-back.

    def GarbleHoldReadBack(Self, Id, Script, Address, Values):
        """Once servo Id receives the hold's Operating_Mode write (position, 0), its next reads at Address answer Values
        (ScriptedReads' Script, a corrupted reply with a good checksum each)."""
        Original = Self.Device.Answer

        def Answer(Packet):
            Reply = Original(Packet)
            if Values and Packet[2] == Id and Packet[4] == 3 and Packet[5] == 33 and Packet[6] == 0:
                Script[Address] = list(Values)
                Values.clear()
            return Reply
        Self.Device.Answer = Answer

    def SignatureMidHome(Self, Address, Values):
        """Home runs; 2026-10-22's signature on shoulder_pan with 4 torque-off reads (a restart check starts and Home holds
        in that frame), and the hold's read-back of Address garbled as Values. Runs that frame; returns the Script."""
        Pan = Ids["shoulder_pan"]
        Self.Events.clear()
        Ok, Text = Self.Controller.go_home()
        Self.assertTrue(Ok, Text)
        Self.Frames(6 / 30)
        Self.assertIsNotNone(Self.Controller.MotionRoute)
        Script = ScriptedReads(Self.Device, Pan, {65: [b"\x00", b"\x01"], 40: [b"\x00"] * 4,
                                                  60: [(64001).to_bytes(2, "little")] * 2, 62: [b"\x00"] * 2,
                                                  63: [bytes([90])] * 2, 55: [bytes([28])] * 2})
        Self.GarbleHoldReadBack(Pan, Script, Address, list(Values))
        Self.Frames(1 / 30)
        Self.assertFalse(any(Script.values()), Script)
        Self.assertIsNone(Self.Controller.MotionRoute)
        return Script

    def TestTodaysSignatureMidHomeWithAGarbledHoldReadBackNeverCoasts(Self):
        # The reviewers' reproduction (StopReadback.py, case "4 goal"): the burst garbles one more read, the hold's goal
        # (42 or 28) or mode (33 or 28) read-back. Before: "Hold read-back mismatch", every joint coasted mid-Home. Now
        # the re-reads read what was written: a protection_glitch, Home held where it is, its Lock reads 1, never latched.
        for Address, Garbled in ((42, 42), (42, 28), (33, 33), (33, 28)):
            with Self.subTest(Address=Address, Garbled=Garbled):
                Self.SignatureMidHome(Address, [Garbled.to_bytes(2 if Address == 42 else 1, "little")])
                Self.Frames(0.5)
                Self.AssertHolding()
                Self.assertEqual(Self.Controller.StatusRestartChecks, {})
                Self.assertFalse(Kinds(Self.Events, "motion_fault") + Kinds(Self.Events, "protection_trip"))
                Self.assertTrue([Each for Each in Kinds(Self.Events, "protection_glitch")
                                 if Each["details"]["value"] == Garbled and "hold read-back" in Each["reason"]], Self.Events)
                with Self.Controller.lock:
                    Self.Controller.StopMotion()

    def TestAGarbledHoldReadBackThatBothReReadsRepeatStillCoasts(Self):
        # The same, but the goal read-back and both its re-reads read 42: the hold is not confirmed, so Stop coasts.
        Self.SignatureMidHome(42, [(42).to_bytes(2, "little")] * 3)
        Self.AssertCoasted("Goal_Position read 42, re-read [42, 42]")

    def TestARealRestartMidHomeWithAGarbledHoldReadBackStillLatches(Self):
        # A real pan restart mid-Home under a corrupted reading: Home holds in the frame its check starts, the hold's goal
        # read-back garbled (28) is tolerated, the hold's goal write energises it, and its Lock still reads 0: latched.
        Pan = Ids["shoulder_pan"]
        Ok, Text = Self.Controller.go_home()
        Self.assertTrue(Ok, Text)
        Self.Frames(0.4)
        Self.assertIsNotNone(Self.Controller.MotionRoute)
        Self.Device.Restart(Pan)
        CorruptReads(Self.Device, Pan, {60: (64001).to_bytes(2, "little")}, 1)
        Script = ScriptedReads(Self.Device, Pan, {})
        Self.GarbleHoldReadBack(Pan, Script, 42, [(28).to_bytes(2, "little")])
        Self.Frames(1.0)
        Self.AssertCoasted("shoulder pan restarted")
        Self.assertFalse(Script.get(42), "the hold ran and read the garbled goal")
        Self.assertEqual((Self.Device.Register(Pan, 55), Self.Device.Register(Pan, 40)), (0, 0))

    # Review round 6 on the real driver (.review-round5.md findings 1-4).

    # Today's exact signature (status 128 then 129, load 64001, 0 V, 90 C, Lock 28) with 4 torque-off reads, by address.
    def Signature1022(Self, Id, Extra=None):
        return ScriptedReads(Self.Device, Id, {65: [b"\x00", b"\x01"], 40: [b"\x00"] * 4,
                                               60: [(64001).to_bytes(2, "little")] * 2, 62: [b"\x00"] * 2,
                                               63: [bytes([90])] * 2, 55: [bytes([28])] * 2, **(Extra or {})})

    def StartHome(Self):
        Self.Events.clear()
        Ok, Text = Self.Controller.go_home()
        Self.assertTrue(Ok, Text)
        Self.Frames(6 / 30)
        Self.assertIsNotNone(Self.Controller.MotionRoute)

    def TestAPanReallyInVelocityModeWithOneReReadGarbledToPositionStillCoasts(Self):
        # Finding 1 on the real driver: mid-Home, today's burst on the pan holds Home in its frame, the pan's Operating_Mode
        # write is acked but never takes (it stays in velocity mode), and the burst garbles one re-read of the mode
        # read-back to 0. Before: the hold stood and wrote Goal_Speed 200 to a pan in velocity mode, nothing watching it.
        # Now it coasts, and the pan never receives a Goal_Speed above 1 from the hold.
        Pan = Ids["shoulder_pan"]
        for Index, Reads in enumerate(([1, 0], [1, 1, 0])):
            with Self.subTest(Reads=Reads):
                if Index:  # a fresh arm: the last one coasted
                    Self.setUp()
                Self.StartHome()
                Self.assertEqual(Self.Device.Register(Pan, 33), 1, "the pan streams in velocity mode")
                Original = Self.Device.Answer

                def Sticks(Packet, Original=Original):
                    Reply = Original(Packet)
                    if Packet[2] == Pan and Packet[4] == 3 and Packet[5] == 33:
                        Self.Device.Tables[Pan][33] = 1  # acked, never taken
                    return Reply
                Self.Device.Answer = Sticks
                Script = Self.Signature1022(Pan)
                Self.GarbleHoldReadBack(Pan, Script, 33, [bytes([Each]) for Each in Reads])
                Before = len(Self.Device.Received)
                Self.Frames(1 / 30)
                Self.AssertCoasted("Hold read-back mismatch")
                Self.assertIn("Operating_Mode read 1", Self.Controller.MotionFault)
                Self.assertFalse([Each for Each in Self.Device.Writes(46, Before) if Each[0] == Pan and Each[2] > 1],
                                 Self.Device.Writes(46, Before))

    def TestOneGarbledPositionReadMidHomeNeverCoasts(Self):
        # Finding 2 on the real driver: one Present_Position reply of the pan garbled mid-Home, 28 (the reading recorded
        # in round 3) or 2090. Before: "shoulder_pan left the checked basket route", every joint coasted. Now it is read
        # again on its own, the re-read kept (a protection_glitch), and Home goes on.
        Pan = Ids["shoulder_pan"]
        for Index, Garbled in enumerate((28, 2090)):
            with Self.subTest(Garbled=Garbled):
                if Index:  # a fresh arm, off Home again
                    Self.setUp()
                Self.StartHome()
                Script = ScriptedReads(Self.Device, Pan, {56: [Garbled.to_bytes(2, "little")]})
                Self.Frames(1 / 30)
                Self.assertFalse(Script[56], "the garbled position was read")
                Self.AssertHolding()
                Self.assertIsNotNone(Self.Controller.MotionRoute, "Home goes on")
                Self.assertEqual(Self.Controller.servos["shoulder_pan"].present_raw, Self.Device.Position(Pan))
                Self.assertTrue([Each for Each in Kinds(Self.Events, "protection_glitch")
                                 if Each["details"]["register"] == "Present_Position" and Each["details"]["value"] == Garbled],
                                Self.Events)
                Self.Frames(1.0)
                Self.AssertHolding()
                Self.assertIsNotNone(Self.Controller.MotionRoute, "Home still going")
                Self.assertFalse(Kinds(Self.Events, "motion_fault") + Kinds(Self.Events, "protection_trip"))

    def TestTodaysSignatureMidHomeWithAGarbledPositionHoldsAtTheConfirmedPosition(Self):
        # Finding 2 in the burst's own frame: today's 4-read burst on the pan mid-Home, that frame's position read garbled
        # (28) too. Before: latched off the route before round 4's hold ran. Now Home holds where the pan really is (its
        # goal the re-read, never 28), its Lock then reads 1, never latched.
        Pan = Ids["shoulder_pan"]
        Self.StartHome()
        Script = Self.Signature1022(Pan, {56: [(28).to_bytes(2, "little")]})
        Self.Frames(1 / 30)
        Self.assertFalse(any(Script.values()), Script)
        Self.assertIsNone(Self.Controller.MotionFault, Self.Controller.status)
        Self.assertIsNone(Self.Controller.MotionRoute, "Home held")
        Self.assertLessEqual(abs(Self.Device.Register(Pan, 42, 2) - Self.Device.Position(Pan)), 2, "held where it is")
        Self.Frames(0.5)
        Self.AssertHolding()
        Self.assertEqual(Self.Controller.StatusRestartChecks, {})
        Self.assertFalse(Kinds(Self.Events, "motion_fault") + Kinds(Self.Events, "protection_trip"))

    def TestAPanReallyOffItsRouteMidHomeStillLatches(Self):
        # The pan really is off its route (knocked to 2090; both re-reads agree): latched and coasted, as before.
        Pan = Ids["shoulder_pan"]
        Self.StartHome()
        Self.Device.SetPosition(Pan, 2090)
        Self.Frames(1 / 30)
        Self.AssertCoasted("shoulder_pan left the checked basket route")
        Reads = Kinds(Self.Events, "motion_fault")[0]["details"]["reads"]
        Self.assertEqual(len(Reads), 2)
        Self.assertLessEqual(abs(Reads[0] - Reads[1]), Bridge.StatusReassertAgreeRaw)

    def CarryWithGripperBurst(Self, Garbled):
        """The gripper holds an object (GraspHold) while the pan's timed stream runs; today's burst on the gripper holds the
        motion in its frame, the squeeze's goal read-back garbled as Garbled (one read each). Runs that frame."""
        Gripper = Ids["gripper"]
        Servo, Pan = Self.Controller.servos["gripper"], Self.Controller.servos["shoulder_pan"]
        with Self.Controller.lock:
            Servo.GraspHold = Servo.Holding = True
            Self.Controller._send_slider_target_locked(Pan, Pan.present_raw + 300, num_retry=1)
            Self.Controller.MotionDeadline = Self.Clock[0] + 100
        Self.Frames(2 / 30)
        # Its Lock reads 28 on its own re-reads too (10 frames), so it is not re-asserted: a restart check holds the motion.
        Script = Self.Signature1022(Gripper, {55: [bytes([28])] * 22})
        Self.GarbleHoldReadBack(Gripper, Script, 42, [Each.to_bytes(2, "little") for Each in Garbled])
        Self.Frames(1 / 30)
        Self.assertFalse(any(Script[Address] for Address in (65, 40, 60, 62, 63, 42)), Script)
        return Servo

    def TestTodaysSignatureOnAGraspingGripperWithAGarbledReadBackNeverDropsIt(Self):
        # Findings 3 and 4 on the real driver: mid-carry, today's burst on the gripper and one garbled squeeze read-back
        # (goal 28). Before: "gripper Hold read-back mismatch", every joint coasted, the object dropped. Now the motion
        # holds, the gripper keeps squeezing, its Lock reads 1 and the check ends.
        Servo = Self.CarryWithGripperBurst([28])
        Self.assertIsNone(Self.Controller.MotionFault, Self.Controller.status)
        Self.assertIn("gripper", Self.Controller.StatusRestartChecks)
        Self.assertEqual(Self.Controller.MotionDeadline, 0.0)
        Self.assertTrue(Servo.GraspHold)
        Gripper = Ids["gripper"]
        Self.assertEqual(abs(Self.Device.Register(Gripper, 42, 2) - Self.Device.Position(Gripper)),
                         Bridge.GRIPPER_GRASP_SQUEEZE_RAW, "squeezing from where its jaws rest")
        Self.assertTrue([Each for Each in Kinds(Self.Events, "protection_glitch") if Each["details"]["value"] == 28
                         and "hold read-back" in Each["reason"]], Self.Events)
        Self.Frames(0.5)
        Self.assertIsNone(Self.Controller.MotionFault, Self.Controller.status)
        Self.assertEqual(Self.Device.Torque(), dict.fromkeys(Positions1632, 1))
        Self.assertEqual(Self.Controller.StatusRestartChecks, {})
        Self.assertTrue(Servo.GraspHold)
        Self.assertFalse(Kinds(Self.Events, "motion_fault") + Kinds(Self.Events, "protection_trip"))

    def TestAGraspingGripperReadBackThatBothReReadsRepeatStillCoasts(Self):
        Self.CarryWithGripperBurst([28] * 3)
        Self.AssertCoasted("gripper Hold read-back mismatch")


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestProtectionConfirmation` runs the Test* methods too (unittest names this hook)."""
    Prefixed = unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return unittest.TestSuite([Prefixed.loadTestsFromTestCase(FakeBusTests), Prefixed.loadTestsFromTestCase(ReplayTests)])


if __name__ == "__main__":
    Loader = unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Suite = unittest.TestSuite([Loader.loadTestsFromTestCase(FakeBusTests), Loader.loadTestsFromTestCase(ReplayTests)])
    Result = unittest.TextTestRunner(verbosity=2).run(Suite)
    raise SystemExit(not Result.wasSuccessful())
