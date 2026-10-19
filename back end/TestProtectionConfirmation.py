"""Protection readings act only once they persist (ProtectionVersion auto-coast-v2-confirmed, MotorProtection.md). Offline.

2026-10-12 (bridge_events.jsonl, UTC) two false alarms coasted a healthy holding arm:
- 16:17:25 "Lost shoulder_pan protection feedback": one Present_Temperature read had no status packet (both tries), every
  joint's position reading fine. Now a temperature must fail on two polls in a row (about 0.5 s).
- 16:32:43 "shoulder lift torque unexpectedly off": one status reading read torque off with load 63745 (0xF901), 5.6 V
  and 90 C, the signature of every torque-off fault recorded since 2026-09-28, positions consistent with the targets.
  Now a status reading that shows anything is read again at once, and a reading whose telemetry no servo can report is a
  failed read: only two bad readings in a row latch.
Real faults still latch: a torque-off or fault bit that the re-read confirms, a servo that restarted (brown-out: torque
off, Lock 0) in the same frame, two failed status readings in a row, two missed temperature polls in a row.

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

# Every joint's present_raw at 2026-10-12 16:32:43 (bridge_events.jsonl), servo ids 1-6.
Positions1632 = {1: 3198, 2: 514, 3: 1784, 4: 156, 5: 1521, 6: 703}
Ids = {Name: Index + 1 for Index, Name in enumerate(Bridge.MOTOR_NAMES)}
HardwareCalls = []
RealMonotonic = time.monotonic
# The 16:32:43 reply's registers: torque 0, load 0xF901, 5.6 V, 90 C (Hardware_Error_Status read 0: bits 128 = torque).
Reply1632 = {40: b"\x00", 60: (0xF901).to_bytes(2, "little"), 62: bytes([56]), 63: bytes([90])}


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

    def TestRecordedTorqueOffTelemetryIsImpossibleAndRealTelemetryIsNot(Self):
        Impossible = Bridge.BridgeController.ServoTelemetryImpossible
        # Every "torque unexpectedly off" with telemetry in captures/*/recording/bridge_events.jsonl, 2026-10-03 to 10-12.
        for Load, Volts, Celsius in ((-29953, 40, 90), (-30721, 0, 89), (-31233, 20, 89), (-29953, 0, 90), (-30209, 0, 90),
                                     (-61697, 0, 90), (63745, 0, 90), (63745, 40, 90), (63489, 152, 89), (63745, 56, 90)):
            Self.assertTrue(Impossible({"load": Load, "voltage_dv": Volts, "temperature_c": Celsius}), (Load, Volts, Celsius))
        # A record without the load still reads impossible on the supply with the heat.
        Self.assertTrue(Impossible({"voltage_dv": 56, "temperature_c": 90}))
        # Readings a real servo gives: full load either way, the arm's 8.9 V, a sagging 5.5 V supply at room heat (a
        # brown-out), 12.6 V, the hottest on record (38 C) and the 60 C limit; and no telemetry at all.
        for Telemetry in ({"load": 1000, "voltage_dv": 89, "temperature_c": 38}, {"load": -1000, "voltage_dv": 89, "temperature_c": 30},
                          {"load": 0, "voltage_dv": 55, "temperature_c": 30}, {"load": 200, "voltage_dv": 126, "temperature_c": 60},
                          {"load": 0, "voltage_dv": 0, "temperature_c": 0}, {}):
            Self.assertIsNone(Impossible(Telemetry), Telemetry)

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

    def TestTheSameCorruptReplyTwiceInARowStillLatches(Self):
        # Two readings in a row that no servo could give: "Cannot read", coasted, as two failed reads are.
        CorruptReads(Self.Device, Ids["shoulder_lift"], Reply1632, 8)
        Self.Frames(1 / 30)
        Self.AssertCoasted("Cannot read shoulder_lift protection/torque status")
        Fault = Kinds(Self.Events, "motion_fault")[0]
        Self.assertEqual(Fault["details"]["telemetry"]["temperature_c"], 90)

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

    def TestAnOverloadBitTwiceLatchesOnceDoesNot(Self):
        Lift = Ids["shoulder_lift"]
        CorruptReads(Self.Device, Lift, {65: bytes([32])}, 1)
        Self.Frames(1 / 30)
        Self.AssertHolding()
        Self.assertEqual(Kinds(Self.Events, "protection_glitch")[0]["details"]["value"], 32)
        CorruptReads(Self.Device, Lift, {65: bytes([32])}, 2)
        Self.Frames(1 / 30)
        Self.AssertCoasted("shoulder lift overload")


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
