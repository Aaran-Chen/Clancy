"""The hand-guiding transport (GuideBus.py) and the Present_Load sign fix, on the REAL scservo_sdk and FeetechMotorsBus
over FakeServoSerial (only the fake port can open). Offline: never opens a serial port.

    python TestGuideBus.py
"""
import time
import unittest
from unittest.mock import patch

import feetech_simple as Driver
import GuideBus as G
from FakeServoSerial import FakePort, FakeSerial, FakeServoDevice, Installed

Names = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
Ids = {Name: Index + 1 for Index, Name in enumerate(Names)}
Positions = {1: 3472, 2: 3171, 3: 2688, 4: 856, 5: 1521, 6: 703}


def MakeBus():
    Bus = Driver.FeetechMotorsBus(FakePort, {Name: Driver.Motor(Ids[Name], "sts3215", Driver.MotorNormMode.DEGREES)
                                             for Name in Names})
    Bus.connect()
    return Bus


class GuideBusTests(unittest.TestCase):
    def setUp(Self):
        Self.Device = Self.enterContext(Installed(FakeServoDevice(Positions)))
        Self.Bus = MakeBus()
        Self.Guide = G.GuideBus(Self.Bus, Names)

    # -- Present_Load ------------------------------------------------------------------------------------------------------
    def TestPresentLoadCarriesItsSignInBit10(Self):
        # The 2026-09-28 elbow overload record read "load": 1224 at 8.9 V: bit 10 and 200, the 20 % an overload allows.
        Self.assertEqual(Self.Bus._decode("Present_Load", 1224), -200)
        Self.assertEqual(Self.Bus._decode("Present_Load", 200), 200)
        Self.assertEqual(Self.Bus._decode("Present_Position", 0x8000 | 5), -5, "position keeps bit 15")
        Self.assertEqual(Self.Bus._decode("Present_Speed", 0x8000 | 40), -40, "speed keeps bit 15")
        Self.assertEqual(Self.Bus._decode("Present_Position", 1224), 1224)
        Self.Device.SetLoad(Ids["elbow_flex"], -200)
        Self.assertEqual(Self.Device.Register(Ids["elbow_flex"], 60, 2), 1224)
        Self.assertEqual(Self.Bus.read("Present_Load", "elbow_flex"), -200)
        Self.Device.SetLoad(Ids["elbow_flex"], 350)
        Self.assertEqual(Self.Bus.read("Present_Load", "elbow_flex"), 350)
        Self.assertEqual(G.DecodeSigned(1224, 10), -200)
        Self.assertEqual(G.EncodeSigned(-200, 10), 1224)
        Self.assertEqual(G.EncodeSigned(-7, 15), 0x8007)

    def TestPresentCurrentCarriesItsSignInBit15(Self):
        # As the vendor SDK's SMS_STS::ReadCurrent decodes it (SMS_STS_DIRECTION_BIT_POS 15), and as GuideBus.ReadPresent
        # and the servo dump do; the driver read it as a magnitude, a negative current as 32768 and more.
        Self.assertEqual(Self.Bus._decode("Present_Current", 0x8000 | 120), -120)
        Self.assertEqual(Self.Bus._decode("Present_Current", 1224), 1224, "not the load's bit 10")
        Self.Device.Tables[Ids["elbow_flex"]][69:71] = (0x8000 | 37).to_bytes(2, "little")
        Self.assertEqual(Self.Bus.read("Present_Current", "elbow_flex"), -37)
        Self.assertEqual(Self.Guide.ReadPresent(["elbow_flex"])["elbow_flex"]["current"], -37)

    # -- packets -----------------------------------------------------------------------------------------------------------
    def TestPacketsAreTheSdksSyncPackets(Self):
        Sent = []

        class Port:
            is_using = False
            def clearPort(self): pass
            def writePort(self, Packet): Sent.append(bytes(Packet)); return len(Packet)
            def setPacketTimeout(self, Length): pass
        Handler = Driver.scs.PacketHandler(0)
        Reader = Driver.scs.GroupSyncRead(Port(), Handler, 56, 15)
        for Id in (1, 2, 3):
            Reader.addParam(Id)
        Reader.txPacket()
        Self.assertEqual(Sent[-1], G.SyncReadPacket([1, 2, 3], 56, 15))
        Writer = Driver.scs.GroupSyncWrite(Port(), Handler, 42, 8)
        Writer.addParam(2, [1, 2, 3, 4, 5, 6, 7, 8])
        Writer.addParam(5, [8, 7, 6, 5, 4, 3, 2, 1])
        Writer.txPacket()
        Self.assertEqual(Sent[-1], G.SyncWritePacket(42, 8, {2: [1, 2, 3, 4, 5, 6, 7, 8], 5: [8, 7, 6, 5, 4, 3, 2, 1]}))
        Self.assertEqual(Sent[-1][:7], bytes([0xFF, 0xFF, 0xFE, 22, 0x83, 42, 8]))

    def TestRepliesParseAcrossChunksAndBadChecksumsAreDropped(Self):
        Good = [bytes([0xFF, 0xFF, Id, 4, 0, 10 * Id, 0]) for Id in (1, 2)]
        Good = [P + bytes([~sum(P[2:]) & 0xFF]) for P in Good]
        Bad = bytearray(Good[1])
        Bad[-1] ^= 0xFF
        Buffer = bytearray(b"\x00\xff" + Good[0][:3])
        Self.assertEqual(G.ParseReplies(Buffer, 2), {})
        Buffer += Good[0][3:] + bytes(Bad) + Good[1][:4]
        Self.assertEqual(G.ParseReplies(Buffer, 2), {1: (0, bytes([10, 0]))})
        Buffer += Good[1][4:]
        Self.assertEqual(G.ParseReplies(Buffer, 2), {2: (0, bytes([20, 0]))})

    # -- the transport on the fake bus -------------------------------------------------------------------------------------
    def TestReadPresentDecodesEveryServoInOnePacket(Self):
        Self.Device.SetLoad(Ids["shoulder_lift"], -420)
        Self.Device.SetVoltage(Ids["gripper"], 8.7)
        Self.Device.SetTemperature(Ids["elbow_flex"], 47)
        Self.Device.Tables[Ids["wrist_flex"]][65] = 32
        Before = len(Self.Device.SyncPackets)
        Readings = Self.Guide.ReadPresent()
        Self.assertEqual(len(Self.Device.SyncPackets) - Before, 1, "one SYNC_READ for all six")
        Self.assertEqual(Self.Device.SyncPackets[-1], (0x82, 56, 15, (1, 2, 3, 4, 5, 6)))
        Self.assertEqual({N: R["pos"] for N, R in Readings.items()}, {N: Positions[Ids[N]] for N in Names})
        Self.assertEqual(Readings["shoulder_lift"]["load"], -420)
        Self.assertEqual(Readings["gripper"]["volts"], 8.7)
        Self.assertEqual(Readings["elbow_flex"]["temp_c"], 47.0)
        Self.assertEqual(Readings["wrist_flex"]["status"], 32)
        Self.assertEqual(Readings["shoulder_pan"]["volts"], 9.0)

    def TestASilentServoIsItsOwnMissAndCostsTheDeadlineNotTheSdksTimeout(Self):
        Self.Device.Silent = {Ids["elbow_flex"]}
        Failures = Self.Bus.BusStats["failures"]
        Started = time.perf_counter()
        Readings = Self.Guide.ReadPresent()
        Took = time.perf_counter() - Started
        Self.assertIsNone(Readings["elbow_flex"])
        Self.assertEqual(sorted(N for N, R in Readings.items() if R is not None),
                         sorted(N for N in Names if N != "elbow_flex"), "the servos after it still count")
        Self.assertLess(Took, 0.02, "a silent servo costs the 5 ms deadline, not 34 ms")
        Self.assertEqual(Self.Bus.BusStats["failures"], Failures + 1)
        Self.assertEqual(Self.Bus.BusStats["recent_failures"][-1]["motor"], "elbow_flex")
        Self.assertFalse(Self.Bus._port_handler.is_using)

    def TestALateReplyIsDrainedAndFlushIsNeverCalled(Self):
        Flushes = []
        with patch.object(FakeSerial, "flush", lambda S: Flushes.append(1)):
            Self.Bus._port_handler.ser.Rx += bytes([0xFF, 0xFF, 3, 17, 0]) + bytes(15) + bytes([0])  # last cycle's late reply
            Readings = Self.Guide.ReadPresent()
            Self.Guide.WriteGoals({"shoulder_lift": (3180, 400)})
        Self.assertEqual(Flushes, [])
        Self.assertEqual(Readings["elbow_flex"]["pos"], Positions[3])

    def TestWriteGoalsReleaseAndStatusReadBack(Self):
        Lift, Roll = Ids["shoulder_lift"], Ids["wrist_roll"]
        Self.Bus.write("Lock", "wrist_roll", 1)
        Self.Guide.WriteGoals({"shoulder_lift": (-12.4, 420.6), "wrist_roll": (1530, 60)})
        Self.assertEqual(Self.Device.SyncPackets[-1][:3], (0x83, 42, 8))
        Self.assertEqual(Self.Device.Register(Lift, 42, 2), 0x8000 | 12, "goal sign in bit 15")
        Self.assertEqual(Self.Device.Register(Lift, 48, 2), 421)
        Self.assertEqual(Self.Device.Register(Lift, 46, 2), 0, "Goal_Speed 0: the position loop's own response")
        Self.assertEqual(Self.Device.Torque()[Lift], 1, "a goal write energises")
        Self.Guide.WriteGoals({"shoulder_lift": (3180, 1000)}, Acceleration=51)
        Self.assertEqual(Self.Device.SyncPackets[-1][:3], (0x83, 41, 9))
        Self.assertEqual(Self.Device.Register(Lift, 41), 51)
        Self.Guide.Release(["wrist_roll"])
        Self.assertEqual(Self.Device.SyncPackets[-1], (0x83, 40, 1, (Roll,)))
        Self.assertEqual(Self.Device.Torque()[Roll], 0)
        Status = Self.Guide.ReadStatus()
        Self.assertFalse(Status["wrist_roll"]["torque"])
        Self.assertEqual(Status["wrist_roll"]["lock"], 1, "release leaves Lock 1")
        Self.assertEqual((Status["shoulder_lift"]["goal"], Status["shoulder_lift"]["limit"]), (3180, 1000))
        Self.assertTrue(Status["shoulder_lift"]["torque"])
        Settings = Self.Guide.ReadSettings()
        Self.assertEqual({K: Settings["elbow_flex"][K] for K in ("p", "d", "i", "punch", "dead_zone", "firmware")},
                         {"p": 32, "d": 32, "i": 0, "punch": 16, "dead_zone": [1, 1], "firmware": [3, 10]})

    def TestALostPortIsMarkedLikeTheDriverDoes(Self):
        Self.Device.Remove()
        with Self.assertRaises(Driver.PortLostError):
            Self.Guide.ReadPresent()
        Self.assertTrue(Self.Bus.PortLost)
        Self.assertFalse(Self.Bus._port_handler.is_using)
        with Self.assertRaises(Driver.PortLostError):
            Self.Guide.Release(["gripper"])
        Self.Device.Arrive()
        Self.assertTrue(Self.Bus.Reopen())
        Self.assertEqual(Self.Guide.ReadPresent()["gripper"]["pos"], Positions[6])


def load_tests(Loader, Tests, Pattern):
    Prefixed = unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return Prefixed.loadTestsFromTestCase(GuideBusTests)


if __name__ == "__main__":
    Loader = unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Result = unittest.TextTestRunner(verbosity=2).run(Loader.loadTestsFromTestCase(GuideBusTests))
    raise SystemExit(not Result.wasSuccessful())
