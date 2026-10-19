"""The bridge recovers from a dropped USB serial link instead of cascading. Offline only.

2026-10-01 19:07:22 the arm's CH343 USB adapter (COM4) dropped off the bus for 23 s. pyserial
raised "ClearCommError failed (PermissionError(13, 'Access is denied.', None, 5))" mid-read;
the scservo SDK left its port busy flag set, so every later call failed "Port is in use!"
without touching the port; the bridge never reopened it. Hold at 19:22:07 failed for want of
fresh positions, the emergency torque-off at 19:22:14 reached no servo, and the servos held
torque unsupervised for 15 minutes.

These run the REAL scservo_sdk and FeetechMotorsBus over FakeServoSerial (only the fake port
FAKE-COM4 can open) on a simulated clock. Never opens a serial port or the depth camera, and
never raises this process's priority.

Run:  python -m unittest TestSerialLinkRecovery
"""
import time
import unittest
from unittest.mock import patch

import bridge_server as Bridge
import feetech_simple as Driver
from FakeServoSerial import FakePort, FakeServoDevice, Installed

# Every joint's present_raw at 19:07:22 (bridge_events.jsonl), servo ids 1-6.
Positions1907 = {1: 3472, 2: 3171, 3: 2688, 4: 856, 5: 1521, 6: 703}
Ids = {Name: Index + 1 for Index, Name in enumerate(Bridge.MOTOR_NAMES)}
HardwareCalls = []
RealMonotonic = time.monotonic


def GuardedOpener():
    HardwareCalls.append("depth camera opened")
    raise RuntimeError("an offline test reached the real depth camera")


def setUpModule():
    for Patch in (patch.object(Bridge.depth_camera, "Opener", GuardedOpener),
                  patch.object(Bridge, "RaiseProcessPriority", lambda: HardwareCalls.append("priority raised") or "normal")):
        Patch.start()
        unittest.addModuleCleanup(Patch.stop)


class SerialLinkTests(unittest.TestCase):
    def setUp(Self):
        Self.Device = Self.enterContext(Installed(FakeServoDevice(Positions1907)))
        Self.Clock = [1000.0]  # simulated; None switches to the real clock
        Self.enterContext(patch.object(Bridge.time, "monotonic",
                                       lambda: RealMonotonic() if Self.Clock[0] is None else Self.Clock[0]))
        Self.Controller = Bridge.BridgeController()
        Self.Events = []
        Self.Controller.EventSink = Self.Events.append

    def tearDown(Self):
        Self.assertEqual(HardwareCalls, [], "an offline test reached the depth camera or process priority")
        Texts = [Self.Controller.status, str(Self.Controller.MotionFault)] + [Event["reason"] for Event in Self.Events]
        Self.assertFalse([Text for Text in Texts if "Port is in use" in Text], "the SDK busy flag was left set")

    # -- helpers ---------------------------------------------------------------

    def Connect(Self, Hold=True):
        Ok, Text = Self.Controller.connect(FakePort)
        Self.assertTrue(Ok, Text)
        if Self.Controller.torque_enabled:  # connect's Goal_Position seed energises the servos
            Self.assertTrue(Self.Controller.toggle_torque()[0], Self.Controller.status)
        if Hold:
            Self.assertTrue(Self.Controller.toggle_torque()[0], Self.Controller.status)
        Self.Frames(0.5)
        Self.assertIsNone(Self.Controller.MotionFault)
        Self.assertEqual(Self.Controller.status, "Bridge live")
        Self.Events.clear()
        return Self.Controller.bus

    def Frames(Self, Seconds, Until=None):
        """The 30 Hz control loop on the simulated clock; the servos move between frames."""
        for _ in range(round(Seconds * 30)):
            Self.Clock[0] += 1 / 30
            Self.Device.Step(1 / 30)
            Self.Controller.refresh()
            if Until and Until():
                return True
        return False

    def Kinds(Self):
        return [Event["kind"] for Event in Self.Events]

    def AllTorque(Self, Value):
        Self.assertEqual(Self.Device.Torque(), dict.fromkeys(Positions1907, Value))

    def AllowRoutes(Self):
        """Plan straight to the target: these tests are about the link, not the planner (test_bridge_home)."""
        Controller = Self.Controller
        Controller.ValidateMotionPath = lambda Targets, Calibration=False, **Options: None
        Controller.PlanMotionRoute = lambda Targets, Calibration=False: [
            {Name: Targets.get(Name, Servo.present_raw) for Name, Servo in Controller.servos.items()}]

    # -- the driver --------------------------------------------------------------

    def TestFailedTransactionNeverLeavesThePortBusy(Self):
        Bus = Bridge._make_bus(FakePort)
        Bus.connect()
        Handle = Bus._port_handler.ser
        # Mid-read: the request went out, the adapter dropped before the reply.
        Self.Device.RemoveAfterRequest = (Ids["wrist_roll"], 65)
        with Self.assertRaises(Driver.PortLostError) as Raised:
            Bus.read("Hardware_Error_Status", "wrist_roll", num_retry=1)
        Self.assertIn("ClearCommError failed (PermissionError(13, 'Access is denied.', None, 5))", str(Raised.exception))
        Self.assertIn("USB serial link to the arm on FAKE-COM4 dropped", str(Raised.exception))
        Self.assertFalse(Bus._port_handler.is_using, "the SDK busy flag stayed set after the exception")
        Self.assertTrue(Bus.is_connected, "an outage is not a disconnect")
        Self.assertFalse(Handle.is_open, "the dead handle was not let go")
        # Every later call fails at once, without touching the dead handle.
        Calls = Handle.Calls
        with Self.assertRaises(Driver.PortLostError):
            Bus.write("Torque_Enable", "shoulder_pan", 0)
        with Self.assertRaises(Driver.PortLostError) as Raised:
            Bus.disable_torque()
        Self.assertNotIn("Port is in use", str(Raised.exception))
        Self.assertEqual(Handle.Calls, Calls)
        # The port is reopened only once the adapter is back, and works again.
        Self.assertFalse(Bus.Reopen())
        Self.assertIn("could not open port", Bus.ReopenError)
        Self.Device.Arrive()
        Self.assertTrue(Bus.Reopen())
        Self.assertIsNone(Bus.PortLost)
        Self.assertEqual(Bus.read("Present_Position", "wrist_roll"), Positions1907[Ids["wrist_roll"]])
        # Mid-write: the write itself fails and the servo never gets it.
        Self.Device.RemoveOnRequest = (Ids["shoulder_pan"], 40)
        with Self.assertRaises(Driver.PortLostError):
            Bus.write("Torque_Enable", "shoulder_pan", 1)
        Self.assertFalse(Bus._port_handler.is_using)
        Self.assertEqual((Bus.BusStats["port_lost"], Bus.BusStats["reopens"]), (2, 1))
        Results = [Failure["result"] for Failure in Bus.BusStats["recent_failures"]]
        Self.assertEqual(len(Results), 2)
        Self.assertTrue(all(Result.startswith("port lost: ") for Result in Results), Results)

    def TestAnyExceptionInsideTheSdkReleasesThePort(Self):
        # Not only port errors: whatever escapes the SDK mid-transaction must not leave it "in use".
        Bus = Bridge._make_bus(FakePort)
        Bus.connect()
        Handler, Original = Bus._port_handler, Bus._port_handler.readPort
        def Broken(Length):
            Handler.readPort = Original
            raise ValueError("simulated SDK fault")
        Handler.readPort = Broken
        with Self.assertRaises(ValueError):
            Bus.read("Present_Position", "elbow_flex")
        Self.assertFalse(Handler.is_using)
        Self.assertIsNone(Bus.PortLost, "an SDK fault is not a lost port")
        Self.assertEqual(Bus.read("Present_Position", "elbow_flex"), Positions1907[Ids["elbow_flex"]])

    # -- 19:07:22, a short drop ---------------------------------------------------

    def TestBriefDropWhileHoldingKeepsHoldingAfterFreshReads(Self):
        Self.Connect()
        Self.Device.RemoveAfterRequest = (Ids["wrist_roll"], 65)  # the 19:07:22 read
        Self.Frames(0.4)
        Controller = Self.Controller
        Self.assertIsNone(Controller.MotionFault, Controller.status)
        Self.assertIsNone(Controller.ProtectionTrip)
        Self.assertTrue(Controller.torque_enabled)
        Self.assertTrue(Controller.status.startswith("USB serial link to the arm dropped 0.4 s ago; reconnecting"),
                        Controller.status)
        State = Controller.state_snapshot()
        Self.assertEqual(State["serial_link"]["state"], "down")
        Self.assertIn("Access is denied", State["serial_link"]["error"])
        Self.assertFalse(any(Servo["feedback_ok"] for Servo in State["servos"].values()), "stale positions shown as fresh")
        Self.AllTorque(1)
        Self.assertEqual(Self.Kinds(), ["serial_link_lost"])
        # The adapter is back: the port is reopened and every joint read before anything is written.
        Since = len(Self.Device.Received)
        Self.Device.Arrive()
        Self.Frames(0.2)
        Self.assertIsNone(Controller.MotionFault, Controller.status)
        Self.assertTrue(Controller.torque_enabled)
        Self.AllTorque(1)
        Self.assertRegex(Controller.status, r"^USB serial link dropped for 0\.\d s; reconnected; holding at the last position$")
        Self.assertEqual(Self.Kinds(), ["serial_link_lost", "serial_link_recovered"])
        Counts = Self.Events[-1]["details"]["serial_link"]
        Self.assertEqual((Counts["drops_total"], Counts["recoveries"], Counts["failures"]), (1, 1, 0))
        Received = Self.Device.Received[Since:]
        FirstWrite = next((Index for Index, Packet in enumerate(Received) if Packet[0] == 3), len(Received))
        Read = {Id for Instruction, Id, Address, _ in Received[:FirstWrite] if Instruction == 2 and Address == 56}
        Self.assertEqual(Read, set(Positions1907), "a write went out before every joint was read again")
        Self.assertTrue(all(Servo["feedback_ok"] for Servo in Controller.state_snapshot()["servos"].values()))
        # It stays healthy, and says what happened for a while.
        Self.Frames(2.0)
        Self.assertIsNone(Controller.MotionFault)
        Self.assertTrue(Controller.status.startswith("USB serial link dropped for"), Controller.status)
        Self.assertEqual(Controller.state_snapshot()["serial_link"]["state"], "ok")
        Self.Frames(Bridge.SerialLinkNoteSeconds - 2.0 + 0.2)
        Self.assertEqual(Controller.status, "Bridge live")
        Self.assertIsNone(Controller.MotionFault)

    # -- 19:07:22 to 19:22:14 as it happened: a 23 s drop -------------------------

    def TestReplay20261001(Self):
        Controller, Bus = Self.Controller, Self.Connect()
        Since = len(Self.Device.Received)
        Self.Device.RemoveAfterRequest = (Ids["wrist_roll"], 65)
        Self.Frames(Bridge.SerialLinkHoldWindowS + 0.2)
        # The hold window passed: latched, honest about what did not reach the servos.
        Self.assertIn("USB serial link to the arm dropped and could not reconnect within 2 s: check the USB cable",
                      Controller.MotionFault)
        Self.assertFalse(Controller.ProtectionTrip["coast_confirmed"])
        Self.assertIn("The servos keep their last command until the USB serial link returns", Controller.status)
        Self.assertIn("torque-off is sent the moment it does. Support the arm and check the USB cable.", Controller.status)
        Self.AllTorque(1)
        Self.assertFalse(Bus._port_handler.is_using)
        Self.assertEqual(Self.Kinds(), ["serial_link_lost", "motion_fault"])
        Self.assertEqual(Self.Events[-1]["details"]["serial_link"]["failures"], 1)
        # Hold needs fresh reads of every joint: refused without touching anything.
        Received = len(Self.Device.Received)
        Ok, Text = Controller.toggle_torque()
        Self.assertFalse(Ok)
        Self.assertTrue(Text.startswith("Cannot enable Hold: "), Text)
        Self.assertEqual(len(Self.Device.Received), Received)
        # 19:07:46: COM4 is back, 23.4 s after the drop. The latched coast is sent within a retry.
        Self.Frames(23.4 - Bridge.SerialLinkHoldWindowS - 0.2)
        Self.AllTorque(1)
        Self.Device.Arrive()
        Self.assertTrue(Self.Frames(1.0, Until=lambda: Controller.ProtectionTrip["coast_confirmed"]))
        Self.assertLessEqual(Self.Clock[0] - 1000.0 - 0.5, 23.4 + Bridge.SerialLinkRetryIntervalS + 0.1)
        Self.AllTorque(0)
        Self.assertRegex(Controller.status, r"^USB serial link dropped for 23\.\d s; reconnected; all motors coasting\. "
                                            r"Support the arm, then press Hold\.$")
        Self.assertFalse(Self.Device.Writes(Driver.CONTROL_TABLE["Goal_Position"][0], Since=Since),
                         "a position goal was written during fault recovery")
        Self.assertEqual(Self.Kinds(), ["serial_link_lost", "motion_fault", "serial_link_recovered"])
        Self.assertTrue(Self.Events[-1]["details"]["coast_confirmed"])
        # 19:18:38 and 19:22:02 the page logged the same fault again; the bridge has nothing new.
        Failures = Controller.bus.BusStats["failures"]
        Self.Frames(15)
        Self.assertEqual(len(Self.Events), 3)
        Self.assertEqual(Controller.bus.BusStats["failures"], Failures)
        # 19:22:07 Hold, from fresh positions.
        Ok, Text = Controller.toggle_torque()
        Self.assertTrue(Ok, Text)
        Self.AllTorque(1)
        Self.assertIsNone(Controller.ProtectionTrip)
        # 19:22:14 disconnect: a real coast, then a second press that is honest too.
        Self.assertEqual(Controller.disconnect(disable_torque=True), (True, "Emergency shutoff - coast and disconnected"))
        Self.AllTorque(0)
        Self.assertEqual(Controller.disconnect(disable_torque=True), (True, "Emergency shutoff - coast and disconnected"))

    # -- a drop mid-write during a move ---------------------------------------------

    def TestDropMidWriteDuringAMoveStopsAndCoastsOnReturn(Self):
        Controller = Self.Controller
        Self.Connect()
        Self.AllowRoutes()
        Elbow = Controller.servos["elbow_flex"]
        Ok, Text = Controller.move_servo("elbow_flex", Elbow.present_raw - 300)
        Self.assertTrue(Ok, Text)
        Self.Frames(0.3)
        Self.assertIsNotNone(Controller.MotionRoute)
        Self.assertEqual(Self.Device.Register(Ids["elbow_flex"], 33), 1, "the move runs in velocity mode")
        Since = len(Self.Device.Received)
        Self.Device.RemoveOnRequest = (Ids["elbow_flex"], 46)  # the next elbow speed change, as it slows
        Self.assertTrue(Self.Frames(3.0, Until=lambda: Controller.MotionFault is not None))
        Self.assertEqual(Controller.MotionFault, "USB serial link to the arm dropped during a move; the move is cancelled")
        Self.assertIn("WriteFile failed", Self.Events[-1]["details"]["error"])
        Self.assertIsNone(Controller.MotionRoute)
        Self.assertFalse(Controller.ProtectionTrip["coast_confirmed"])
        # Nothing reaches the servos: the elbow keeps its last speed meanwhile.
        Self.assertNotEqual(Driver.scs.SCS_TOHOST(Self.Device.Register(Ids["elbow_flex"], 46, 2), 15), 0)
        Self.AllTorque(1)
        Self.Frames(0.3)
        Self.Device.Arrive()
        Self.assertTrue(Self.Frames(0.5, Until=lambda: Controller.ProtectionTrip["coast_confirmed"]))
        Self.AllTorque(0)
        Self.assertEqual(Self.Device.Register(Ids["elbow_flex"], 46, 2), 0, "the velocity stream was not stopped")
        Self.assertFalse(Self.Device.Writes(Driver.CONTROL_TABLE["Goal_Position"][0], Since=Since))
        Self.assertTrue(Controller.status.endswith("reconnected; all motors coasting. Support the arm, then press Hold."),
                        Controller.status)
        Self.assertEqual(Self.Kinds(), ["serial_link_lost", "motion_fault", "serial_link_recovered"])
        Self.assertTrue(Self.Events[0]["details"]["moving"])

    # -- a drop during the emergency torque-off ---------------------------------------

    def TestDropDuringEmergencyTorqueOffRetriesOnAFreshHandle(Self):
        Controller = Self.Controller
        Self.Connect()
        Self.Device.ComeBackAtOnce = True  # a glitch: the old handle dies, COM4 opens again at once
        Self.Device.RemoveOnRequest = (Ids["shoulder_pan"], 40)
        Ok, Text = Controller.emergency_stop()
        Self.assertEqual((Ok, Text), (True, "Emergency shutoff - coast and disconnected"))
        Self.AllTorque(0)
        Self.assertIsNone(Controller.bus)
        Self.assertEqual(Self.Kinds(), ["serial_link_lost", "serial_link_recovered"])
        Self.assertIn("reconnected to send the emergency stop", Self.Events[-1]["reason"])

    def TestEmergencyStopWithTheLinkDownSendsTheCoastWhenItReturns(Self):
        Controller = Self.Controller
        Self.Connect()
        Self.Device.RemoveOnRequest = (Ids["shoulder_pan"], 40)
        Ok, Text = Controller.emergency_stop()
        Self.assertFalse(Ok)
        Self.assertIn("torque-off not sent, the USB serial link to the arm is down", Text)
        Self.assertIn("The servos keep their last command until the USB serial link returns", Text)
        Self.AllTorque(1)
        Self.assertTrue(Controller.arm_connected, "the bus must stay attached to send the coast later")
        # A second press, or a disconnect, never claims the coast it could not send.
        Self.assertFalse(Controller.emergency_stop()[0])
        Self.assertFalse(Controller.disconnect(disable_torque=True)[0])
        Self.Frames(0.5)
        Self.Device.Arrive()
        Self.assertTrue(Self.Frames(1.0, Until=lambda: Controller.ProtectionTrip["coast_confirmed"]))
        Self.AllTorque(0)
        Self.assertIn("reconnected; all motors coasting", Controller.status)
        Self.assertEqual(Controller.disconnect(disable_torque=True), (True, "Emergency shutoff - coast and disconnected"))
        Self.assertFalse(Controller.arm_connected)

    # -- a drop during Hold -------------------------------------------------------------

    def TestDropWhileEnablingHoldCoastsOnceTheLinkReturns(Self):
        Controller = Self.Controller
        Self.Connect(Hold=False)
        Self.AllTorque(0)
        # Hold writes each joint's goal before energising it; the link drops at the elbow's.
        Self.Device.RemoveOnRequest = (Ids["elbow_flex"], 42)
        Ok, Text = Controller.toggle_torque()
        Self.assertFalse(Ok)
        Self.assertTrue(Text.startswith("Hold could not be completed: the USB serial link to the arm dropped. "
                                        "The servos keep their last command until the USB serial link returns"), Text)
        Self.assertFalse(Controller.ProtectionTrip["coast_confirmed"])
        # The goals that did go out energised two joints (as the real servos do): they must be coasted.
        Self.assertEqual(Self.Device.Torque()[Ids["shoulder_pan"]], 1)
        Self.Frames(0.3)
        Self.Device.Arrive()
        Self.assertTrue(Self.Frames(0.5, Until=lambda: Controller.ProtectionTrip["coast_confirmed"]))
        Self.AllTorque(0)
        # Hold works again from fresh positions.
        Ok, Text = Controller.toggle_torque()
        Self.assertTrue(Ok, Text)
        Self.AllTorque(1)

    def TestHoldIsRefusedWhileTheLinkIsDown(Self):
        Controller = Self.Controller
        Self.Connect(Hold=False)
        Self.Device.Remove()
        Self.Frames(0.2)
        Received = len(Self.Device.Received)
        Ok, Text = Controller.toggle_torque()
        Self.assertFalse(Ok)
        Self.assertEqual(Text, "Cannot enable Hold: " + Controller.SerialLinkDownText())
        Self.assertIsNone(Controller.ProtectionTrip)
        Self.assertEqual(len(Self.Device.Received), Received)

    # -- other drops ------------------------------------------------------------------

    def TestRepeatedDropsTripProtection(Self):
        Controller = Self.Controller
        Self.Connect()
        for Drop in range(1, Bridge.SerialLinkDropLimit + 1):
            Self.Device.Remove()
            Self.Frames(0.2)
            Self.Device.Arrive()
            Self.Frames(0.3)
            if Drop < Bridge.SerialLinkDropLimit:
                Self.assertIsNone(Controller.MotionFault, f"drop {Drop}: {Controller.status}")
                Self.assertIn("holding at the last position", Controller.status)
        Self.assertIn(f"USB serial link to the arm dropped {Bridge.SerialLinkDropLimit} times in 60 s: "
                      "check the USB cable and port", Controller.MotionFault)
        Self.assertTrue(Controller.ProtectionTrip["coast_confirmed"])
        Self.AllTorque(0)
        Fault = next(Event for Event in Self.Events if Event["kind"] == "motion_fault")
        Self.assertEqual(Fault["details"]["serial_link"]["drops_recent"], Bridge.SerialLinkDropLimit)
        Self.assertEqual(Self.Kinds().count("serial_link_lost"), Bridge.SerialLinkDropLimit)
        Self.assertEqual(Self.Kinds().count("serial_link_recovered"), Bridge.SerialLinkDropLimit)

    def TestJointThatMovedDuringTheDropIsNotHeldThere(Self):
        Controller = Self.Controller
        Self.Connect()
        Self.Device.Remove()
        Self.Frames(0.2)
        # The shoulder unloaded (firmware overload protection) and sagged while nobody watched.
        Lift = Ids["shoulder_lift"]
        Self.Device.Tables[Lift][40] = 0
        Self.Device.SetPosition(Lift, Self.Device.Position(Lift) - 200)
        Self.Device.Arrive()
        Self.Frames(0.2)
        Self.assertRegex(Controller.MotionFault, r"^USB serial link dropped for 0\.\d s; reconnected, but shoulder_lift "
                                                 r"torque is off; shoulder_lift moved 200 steps while the link was down$")
        Self.assertTrue(Controller.ProtectionTrip["coast_confirmed"])
        Self.AllTorque(0)

    def TestDropWhileCoastingNeedsNoFault(Self):
        Controller = Self.Controller
        Self.Connect(Hold=False)
        Self.Device.Remove()
        Self.Frames(Bridge.SerialLinkHoldWindowS + 0.5)
        Self.assertIsNone(Controller.MotionFault)
        Self.assertIsNone(Controller.ProtectionTrip)
        Self.assertRegex(Controller.status, r"^USB serial link to the arm down for 2\.\d s; still trying to reconnect: "
                                            r"check the USB cable$")
        Self.assertEqual(Self.Kinds(), ["serial_link_lost", "serial_link_failed"])
        Self.Device.Arrive()
        Self.Frames(0.6)
        Self.assertTrue(Controller.status.endswith("reconnected; the arm is coasting (torque off)"), Controller.status)
        Self.assertIsNone(Controller.MotionFault)

    def TestCommandsWhileTheLinkIsDownSayWhy(Self):
        Controller = Self.Controller
        Self.Connect()
        Self.AllowRoutes()
        Self.Device.Remove()
        Self.Frames(0.2)
        Ok, Text = Controller.move_servo("elbow_flex", Controller.servos["elbow_flex"].present_raw - 100)
        Self.assertFalse(Ok)
        Self.assertTrue(Text.startswith("Cannot move: USB serial link to the arm dropped"), Text)
        Ok, Text = Controller.StopMotion()
        Self.assertFalse(Ok)
        Self.assertIn("nothing was moving", Text)
        Self.assertIsNone(Controller.MotionFault)
        # Coast while the link is down is latched and sent the moment it returns.
        Ok, Text = Controller.toggle_torque()
        Self.assertFalse(Ok)
        Self.assertIn("Coast requested while the USB serial link to the arm was down", Text)
        Self.AllTorque(1)
        Self.Device.Arrive()
        Self.assertTrue(Self.Frames(0.5, Until=lambda: Controller.ProtectionTrip["coast_confirmed"]))
        Self.AllTorque(0)
        # A same-port connect while it is down tries at once instead of claiming "already connected".
        Self.Device.Remove()
        Self.Frames(0.1)
        Ok, Text = Controller.connect(FakePort)
        Self.assertFalse(Ok)
        Self.assertIn("USB serial link to the arm", Text)
        Self.Device.Arrive()
        Ok, Text = Controller.connect(FakePort)
        Self.assertTrue(Ok, Text)
        Self.assertIn("reconnected", Text)

    def TestConnectSendsAPendingTorqueOffAndSaysSo(Self):
        # The old path to 19:22: the arm was disconnected with its torque-off unconfirmed. The next
        # connect then coasted the arm silently on its first frame.
        Controller = Self.Controller
        Controller.ProtectionTrip = {"reason": "Emergency torque-off unconfirmed: test", "at_s": 0.0,
                                     "coast_confirmed": False, "unconfirmed": list(Bridge.MOTOR_NAMES)}
        Ok, Text = Controller.emergency_stop()
        Self.assertFalse(Ok, "no bus and a pending torque-off is not 'coast and disconnected'")
        Self.assertIn("never reached the servos", Text)
        Ok, Text = Controller.connect(FakePort)
        Self.assertTrue(Ok, Text)
        Self.assertEqual(Text, f"Connected on {FakePort} and sent the torque-off pending since: Emergency torque-off "
                               "unconfirmed: test. All motors coasting: support the arm, then press Hold.")
        Self.AllTorque(0)
        Self.assertIn("pending_coast_sent", Self.Kinds())

    def TestConnectingTheAdapterUnderANewPortNameCarriesThePendingTorqueOff(Self):
        Controller = Self.Controller
        Self.Connect()
        Self.Device.Remove()
        Self.Frames(Bridge.SerialLinkHoldWindowS + 0.2)
        Self.assertFalse(Controller.ProtectionTrip["coast_confirmed"])
        # Windows gave the adapter another COM number; the bridge's port stays gone.
        Self.Device.Arrive(As="FAKE-COM5")
        Self.Frames(1.0)
        Self.assertTrue(Controller.SerialLinkLost())
        Self.AllTorque(1)
        Ok, Text = Controller.connect("FAKE-COM5")
        Self.assertTrue(Ok, Text)
        Self.assertTrue(Text.startswith("Connected on FAKE-COM5 and sent the torque-off pending since: USB serial link "
                                        "to the arm dropped and could not reconnect within 2 s"), Text)
        Self.AllTorque(0)
        Self.assertEqual(Controller.state_snapshot()["serial_link"]["state"], "ok")
        Self.assertIn("serial_link_abandoned", Self.Kinds())
        Self.assertTrue(Controller.toggle_torque()[0], Controller.status)
        Self.AllTorque(1)

    def TestDropUnderTheRealControlLoop(Self):
        # The same short drop on the real 30 Hz control thread and the real clock, not the simulated one.
        Controller = Self.Controller
        Self.Connect()
        Self.Clock[0] = None  # the real clock from here on
        Controller.last_refresh_s = 0.0
        Loop = Bridge.ControlLoop(Controller, Bridge.BRIDGE_FRAME_S)
        Loop.start()
        try:
            time.sleep(0.3)
            Self.Device.Remove()
            time.sleep(0.4)
            with Controller.lock:
                Self.assertTrue(Controller.SerialLinkLost())
                Self.assertIsNone(Controller.MotionFault, Controller.status)
            Self.Device.Arrive()
            time.sleep(0.5)
        finally:
            Loop.stop()
        Self.assertIsNone(Controller.MotionFault, Controller.status)
        Self.assertRegex(Controller.status, r"^USB serial link dropped for 0\.\d s; reconnected; holding at the last position$")
        Self.AllTorque(1)

    def TestStateAndHealthReportTheLink(Self):
        Controller = Self.Controller
        Self.Connect()
        Link = Controller.state_snapshot()["serial_link"]
        Self.assertEqual((Link["state"], Link["version"], Link["drops_total"], Link["note"]), ("ok", "reopen-v1", 0, None))
        Self.Device.Remove()
        Self.Frames(0.2)
        Link = Controller.state_snapshot()["serial_link"]
        Self.assertEqual((Link["state"], Link["drops_total"], Link["drops_recent"]), ("down", 1, 1))
        Self.assertGreater(Link["reopen_attempts"], 0)
        Self.assertIn("could not open port", Link["reopen_error"])
        Self.Device.Arrive()
        Self.Frames(0.2)
        Link = Controller.state_snapshot()["serial_link"]
        Self.assertEqual((Link["state"], Link["recoveries"]), ("ok", 1))
        Self.assertIn("holding at the last position", Link["note"])
        Self.assertEqual(Bridge.health()["serial_link_version"], "reopen-v1")

    # -- found by the adversarial review (2026-10-01) ----------------------------------

    def TestAFaultReadJustBeforeTheDropStillLatches(Self):
        # The shoulder reports overload; the port drops one read later in the same frame, and the
        # bit has cleared by the time the port is back. It was hidden: kept in faults, never latched,
        # and the arm went on "holding at the last position".
        Controller = Self.Controller
        Self.Connect()
        Lift = Ids["shoulder_lift"]
        Self.Device.Tables[Lift][65] = 1 << 5
        Self.Device.RemoveAfterRequest = (Ids["elbow_flex"], 65)
        Self.Frames(1 / 30)
        Self.assertTrue(Controller.SerialLinkLost())
        Self.assertEqual(Controller.MotionFault, "Servo protection/torque fault (shoulder lift overload)")
        Self.assertFalse(Controller.ProtectionTrip["coast_confirmed"])
        Self.assertIn("The servos keep their last command until the USB serial link returns", Controller.status)
        Self.Device.Tables[Lift][65] = 0
        Self.Device.Arrive()
        Self.assertTrue(Self.Frames(0.5, Until=lambda: Controller.ProtectionTrip["coast_confirmed"]))
        Self.AllTorque(0)
        Self.assertEqual(Controller.MotionFault, "Servo protection/torque fault (shoulder lift overload)")

    def TestJointsEnergisedByAConnectCutShortAreCoasted(Self):
        # connect's Goal_Position seed energises each joint it reaches (as the STS3215 does); the link
        # dropped at the elbow's. The pan and shoulder then held unsupervised, and the reconnect
        # reported "the arm is coasting (torque off)".
        Controller = Self.Controller
        Self.Device.RemoveOnRequest = (Ids["elbow_flex"], 42)
        Ok, Text = Controller.connect(FakePort)
        Self.assertFalse(Ok)
        Self.assertTrue(Text.startswith(f"Connect on {FakePort} could not be completed: the USB serial link to the arm "
                                        "dropped. The servos keep their last command until the USB serial link returns"),
                        Text)
        Self.assertEqual([Id for Id, On in Self.Device.Torque().items() if On], [Ids["shoulder_pan"], Ids["shoulder_lift"]])
        Self.Frames(0.3)
        Self.Device.Arrive()
        Self.assertTrue(Self.Frames(0.5, Until=lambda: Controller.ProtectionTrip["coast_confirmed"]))
        Self.AllTorque(0)
        Self.assertTrue(Controller.status.endswith("reconnected; all motors coasting. Support the arm, then press Hold."),
                        Controller.status)

    def TestAJointFoundHoldingAfterADropWhileCoastingIsCoasted(Self):
        # Whatever energised it (a goal write that went out as the link dropped), a joint holding
        # torque while the bridge has the arm coasting is coasted, not reported as "torque off".
        Controller = Self.Controller
        Self.Connect(Hold=False)
        Self.Device.Remove()
        Self.Frames(0.3)
        Self.Device.Tables[Ids["wrist_flex"]][40] = 1
        Self.Device.Arrive()
        Self.Frames(0.3)
        Self.assertRegex(Controller.MotionFault, r"^USB serial link dropped for 0\.\d s; reconnected, but wrist_flex "
                                                 r"torque is on although the arm should be coasting$")
        Self.assertTrue(Controller.ProtectionTrip["coast_confirmed"])
        Self.AllTorque(0)

    def TestTheCoastingNoteNeverVouchesForASilentJoint(Self):
        Controller = Self.Controller
        Self.Connect(Hold=False)
        Self.Device.Remove()
        Self.Frames(0.3)
        Elbow = Self.Device.Tables.pop(Ids["elbow_flex"])  # no status packet from it any more
        Self.Device.Arrive()
        Self.Frames(0.3)
        Self.assertIsNone(Controller.MotionFault)
        Self.assertRegex(Controller.status, r"^USB serial link dropped for 0\.\d s; reconnected; torque is off on every "
                                            r"joint that answered \(elbow_flex did not answer")
        # It read on past the silent joint: the wrist and gripper were checked too.
        Self.assertTrue(Controller.servos["gripper"].FeedbackOk)
        Self.Device.Tables[Ids["elbow_flex"]] = Elbow

    def TestGivingUpAHoldingPortForOneThatFailsKeepsTheTorqueOffPending(Self):
        # Holding, the port drops; inside the hold window the user connects to another port, which
        # fails. Emergency stop then claimed "coast and disconnected" with all six still holding.
        Controller = Self.Controller
        Self.Connect()
        Self.Device.Remove()
        Self.Frames(0.3)
        Ok, Text = Controller.connect("FAKE-COM5")
        Self.assertFalse(Ok)
        Self.assertIn(f"Gave up on {FakePort} while the arm was holding, and connect on FAKE-COM5 failed", Text)
        Self.AllTorque(1)
        Ok, Text = Controller.emergency_stop()
        Self.assertFalse(Ok)
        Self.assertIn("never reached the servos", Text)
        Self.Device.Arrive()
        Ok, Text = Controller.connect(FakePort)
        Self.assertTrue(Ok, Text)
        Self.assertIn(f"sent the torque-off pending since: Gave up on {FakePort} while the arm was holding", Text)
        Self.AllTorque(0)

    def TestAHoldThroughNoteEndsWithACoast(Self):
        # "...; holding at the last position" stayed the status for 20 s even after Coast.
        Controller = Self.Controller
        Self.Connect()
        Self.Device.Remove()
        Self.Frames(0.2)
        Self.Device.Arrive()
        Self.Frames(0.2)
        Self.assertIn("holding at the last position", Controller.status)
        Self.assertTrue(Controller.toggle_torque()[0])  # Coast
        Self.Frames(0.5)
        Self.assertEqual(Controller.status, "Bridge live")
        Self.assertIsNone(Controller.state_snapshot()["serial_link"]["note"])


def load_tests(Loader, Tests, Pattern):
    """`python -m unittest TestSerialLinkRecovery` runs the Test* methods too (unittest names this hook)."""
    Prefixed = unittest.TestLoader()
    Prefixed.testMethodPrefix = "Test"
    return Prefixed.loadTestsFromTestCase(SerialLinkTests)


if __name__ == "__main__":
    Loader = unittest.TestLoader()
    Loader.testMethodPrefix = "Test"
    Result = unittest.TextTestRunner(verbosity=2).run(Loader.loadTestsFromTestCase(SerialLinkTests))
    raise SystemExit(not Result.wasSuccessful())
